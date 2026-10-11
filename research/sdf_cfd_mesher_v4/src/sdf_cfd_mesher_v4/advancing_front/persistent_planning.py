"""Authority-keyed active planning reuse; canonical search ordering is unchanged.

There is no numerical sensitivity or trust-region optimizer in the rolling
poll search. Prior moves are advisory. They cannot bypass canonical predecessor
trials, current hard gates, or the complete immediate certificate.
"""
from collections import OrderedDict,Counter
from pathlib import Path
import json,time,copy
import numpy as np
from ..common.authority import array_hash,identity,file_hash
from ..forecast.detailed.cache import DetailedCache
from ..forecast.detailed.continuous import SideIndex
from ..forecast.detailed.patch import LocalityFailure
from ..certificate.incremental import build_scope
from ..certificate.efficient import Filter
from ..certificate.future_selection import SortedBounds
from ..certificate.future_proofs import FutureSpotProofs
from ..certificate.continuous_prism_certificate.repaired_runtime import CertificateSession
from .rolling_correction import bounded_patch
from .local_relaxation import tangent_basis

VERSION='V4PersistentActivePlanning/1'

def support_identity(topology,ids):
    return identity(dict(version=VERSION,topology=topology,active=array_hash(np.unique(np.asarray(ids,dtype='<i8')))))

def transition(previous,current):
    """Exact source membership, never geometric distance; merge/split resets."""
    a=[set(map(int,x)) for x in previous];b=[set(map(int,x)) for x in current]
    links=[[i for i,x in enumerate(a) if x&y] for y in b]
    fan=Counter(i for row in links for i in row)
    out=[]
    for y,parents in zip(b,links):
        if not parents:kind='new'
        elif len(parents)>1:kind='merge_reset'
        elif fan[parents[0]]>1:kind='split_reset'
        elif y!=a[parents[0]]:kind='support_change_reset'
        else:kind='persistent'
        out.append(dict(kind=kind,parents=parents))
    return out

class SpatialQueries:
    """Incremental exact reuse of the existing padded source-edge predicates.

    For unchanged query boxes, only changed target boxes need retesting.
    Changed query boxes use a complete endpoint-index superset, followed by
    the original overlap predicate. Union results equal SideIndex.candidates,
    including its existing padding and locality cap.
    """
    def __init__(self):self.topology=None;self.lo=None;self.hi=None;self.rows={};self.stats=Counter();self.bounds=None
    def update(self,index):
        if self.topology!=index.topology.sha256 or self.lo is None:
            self.rows={};changed=np.arange(len(index.lo));self.topology=index.topology.sha256
        else:changed=np.flatnonzero(np.any(index.lo.view(np.uint64)!=self.lo.view(np.uint64),axis=1)|np.any(index.hi.view(np.uint64)!=self.hi.view(np.uint64),axis=1))
        changed_set=set(map(int,changed));pad=128*np.finfo(float).eps
        for edge in list(self.rows):
            if edge in changed_set:del self.rows[edge];self.stats['query_invalidations']+=1
            elif len(changed):
                old=self.rows[edge];keep=old[~np.isin(old,changed)]
                mask=np.all(index.hi[changed]>=index.lo[edge]-pad,axis=1)&np.all(index.lo[changed]<=index.hi[edge]+pad,axis=1)
                self.rows[edge]=np.union1d(keep,changed[mask]);self.stats['incremental_target_tests']+=len(changed)
        self.lo=index.lo.copy();self.hi=index.hi.copy();self.stats['changed_edge_bounds']+=len(changed)
        if len(changed) or self.bounds is None:self.bounds=SortedBounds(self.lo,self.hi);self.stats['index_builds']+=1
    def candidates(self,core_edges,max_edges):
        selected=[];pad=128*np.finfo(float).eps
        for e in core_edges:
            e=int(e)
            if e not in self.rows:
                self.rows[e],tested=self.bounds.query(self.lo[e],self.hi[e],pad)
                self.stats['query_misses']+=1;self.stats['indexed_target_tests']+=tested;self.stats['full_target_tests_avoided']+=len(self.lo)-tested
            else:self.stats['query_hits']+=1
            selected.append(self.rows[e])
        ids=np.unique(np.concatenate(selected)) if selected else np.empty(0,dtype=np.int64)
        if len(ids)>max_edges or len(ids)==len(self.lo):raise LocalityFailure('ContinuousCandidateScopeExceedsCallerBound')
        return ids

class SpotPlanner:
    def __init__(self):
        self.spatial=SpatialQueries();self.native=None;self.session=CertificateSession()
        self.proofs=FutureSpotProofs()
        self.selections=OrderedDict();self.stats=Counter()
    def select(self,front,top,future,patch,after,budget):
        if budget<0:raise ValueError('NegativeFutureProbeBudget')
        if not 0<=after<=20:raise ValueError('ForecastLayerRange')
        started=time.perf_counter();index=SideIndex(front,future,top);self.spatial.update(index)
        index.candidates=lambda core_edges,max_edges=2048:self.spatial.candidates(core_edges,max_edges)
        scope=build_scope(index,patch.side_edges);key=scope.scope_identity()
        if key in self.selections:
            choices,counts,stored_budget=self.selections[key]
            if stored_budget>=budget:
                self.stats['selection_hits']+=1;self.stats['seconds']+=time.perf_counter()-started
                return choices[after][:budget],counts[after]
        self.stats['selection_misses']+=1
        if self.native is None:self.native=Filter()
        width=len(scope.edges);forest=self.native.forest(scope.quads,width);parts=[]
        try:
            for layer in range(1,len(scope.quads)//width+1):
                packed=forest.pairs(layer);a=(packed>>32).astype(np.int64);b=(packed&0xffffffff).astype(np.int64)
                keep=scope.core[a]|scope.core[b];parts.append(np.stack((a[keep],b[keep]),axis=1))
        finally:forest.close()
        pairs=np.concatenate(parts);sides=np.asarray(scope.sides,dtype=np.int64);maximum=np.maximum(sides[pairs[:,0],0],sides[pairs[:,1],0])
        counts={k:int(np.count_nonzero(maximum>k)) for k in range(21)}
        labels=np.asarray(scope.labels,dtype=np.int64);incident=np.zeros(len(pairs),bool)
        for i in range(4):
            for j in range(4):incident|=np.all(labels[pairs[:,0],i]==labels[pairs[:,1],j],axis=1)
        pairs=pairs[~incident];a=sides[pairs[:,0]];b=sides[pairs[:,1]]
        swap=(a[:,0]>b[:,0])|((a[:,0]==b[:,0])&(a[:,1]>b[:,1]))
        rows=np.unique(np.concatenate((np.where(swap[:,None],b,a),np.where(swap[:,None],a,b)),axis=1),axis=0)
        order=np.lexsort((rows[:,3],rows[:,2],rows[:,1],rows[:,0],np.minimum(rows[:,0],rows[:,2]),np.maximum(rows[:,0],rows[:,2])))
        rows=rows[order];choices={k:[((int(a),int(b)),(int(c),int(d))) for a,b,c,d in rows[np.maximum(rows[:,0],rows[:,2])>k][:budget]] for k in range(21)}
        self.selections[key]=(choices,counts,budget)
        while len(self.selections)>16:self.selections.popitem(last=False)
        self.stats['seconds']+=time.perf_counter()-started;self.stats['enumerated_pairs']+=len(maximum)
        return choices[after],counts[after]

class PersistentPlanner:
    def __init__(self):
        self.supports={};self.detailed=DetailedCache();self.spots=SpotPlanner()
        self.states={};self.previous=[];self.events=[];self.stats=Counter();self.layer=0;self.forecast=None
    def begin(self,layer,specs,topology):
        current=[np.asarray(s['vertices'],dtype='<i8') for s in specs]
        events=transition(self.previous,current)
        compatible=set()
        for ids,spec,event in zip(current,specs,events):
            key=support_identity(topology,ids);state=self.states.get(key)
            mechanism=spec['name']
            if state and state['mechanism']!=mechanism:event['kind']='mechanism_change_reset'
            if event['kind']=='persistent' and state:compatible.add(key)
            event.update(region=key,mechanism=mechanism,layer=layer)
        self.states={k:v for k,v in self.states.items() if k in compatible}
        if not current:
            # No active planning work remains. Do not serialize hot geometry
            # caches through passive layers. This is eviction, not a new risk rule.
            self.supports.clear();self.detailed.entries.clear();self.spots.selections.clear()
            self.spots.spatial.rows.clear();self.spots.spatial.lo=None;self.spots.spatial.hi=None;self.spots.spatial.topology=None
            self.spots.spatial.bounds=None;self.spots.proofs.cache.clear();self.spots.proofs.patches.clear()
        self.previous=current;self.events.extend(events);self.layer=layer
        for e in events:self.stats[e['kind']]+=1
        return events
    def patch(self,top,ids):
        key=support_identity(top.sha256,ids)
        if key not in self.supports:self.supports[key]=bounded_patch(top,ids);self.stats['support_misses']+=1
        else:self.stats['support_hits']+=1
        return self.supports[key]
    def advisory(self,top,patch,nominal,sdf,hazard=None):
        """Transport a previous displacement through the CURRENT tangent basis.

        Advisory only. The runner still evaluates every canonical predecessor.
        A stale numerical tangent basis or gradient is never reused.
        """
        key=support_identity(top.sha256,patch.active);state=self.states.get(key)
        if state is None:return None
        if state.get('hazard')!=hazard:
            del self.states[key];self.stats['hazard_change_reset']+=1;return None
        basis=tangent_basis(sdf.evaluate(nominal).normal)
        direction=np.asarray(state['displacement'],float)
        transported=np.einsum('nij,nj->ni',basis,np.einsum('nji,nj->ni',basis,direction))
        self.stats['advisory_eligible']+=1
        return dict(direction=transported,mode=state['mode'],factor=state['factor'],source_layer=state['layer'])
    def accepted(self,top,patch,mechanism,mode,factor,nominal,proposed,hazard=None):
        key=support_identity(top.sha256,patch.active)
        self.states[key]=dict(layer=self.layer,mechanism=mechanism,hazard=hazard,mode=mode,factor=factor,displacement=(proposed-nominal).tolist())
    def summary(self):
        return dict(regions=dict(self.stats),spatial=dict(self.spots.spatial.stats),selection=dict(self.spots.stats),
            detailed_hits=self.detailed.hits,detailed_misses=self.detailed.misses,
            dependency_metrics={k:v for k,v in self.detailed.last_stats.items() if k!='changed_cell_ids'},
            exact_pair_hits=self.spots.proofs.stats['hits'],exact_pair_misses=self.spots.proofs.stats['misses'],future_proofs=dict(self.spots.proofs.stats))
    def checkpoint(self,forecast_key):
        return dict(version=VERSION,layer=self.layer,forecast=forecast_key,previous=[x.tolist() for x in self.previous],states=copy.deepcopy(self.states))
    def restore(self,data,layer,forecast_key):
        if data['version']!=VERSION or data['layer']!=layer or data['forecast']!=forecast_key:raise ValueError('PlannerCheckpointIdentity')
        self.layer=layer;self.forecast=forecast_key;self.previous=[np.asarray(x,dtype='<i8') for x in data['previous']];self.states=copy.deepcopy(data['states'])
    def save(self,path,forecast_key):
        arrays={}
        def pack(x):
            if isinstance(x,np.ndarray):
                name='a'+str(len(arrays));arrays[name]=x;return {'array':name}
            if isinstance(x,np.generic):return x.item()
            if isinstance(x,dict):return {str(k):pack(v) for k,v in x.items()}
            if isinstance(x,(list,tuple)):return [pack(v) for v in x]
            return x
        s=self.spots.spatial
        data=dict(checkpoint=self.checkpoint(forecast_key),implementation=self.implementation(),
            detailed=self.detailed.entries,supports={k:p.record() for k,p in self.supports.items()},selection=list(self.spots.selections.items()),
            spatial=dict(topology=s.topology,lo=s.lo,hi=s.hi,rows=s.rows),future_proofs=self.spots.proofs.snapshot())
        meta=json.dumps(pack(data),sort_keys=True,separators=(',',':'),allow_nan=False)
        np.savez_compressed(path,metadata=np.asarray(meta),**arrays)
    @staticmethod
    def implementation():
        from ..forecast.detailed import metrics,rolling_spots,patch,dependency_metrics,cache,exact_cache
        from ..certificate import future_proofs,future_selection
        from ..forecast import exact_dyadic
        return identity({str(Path(p).name):file_hash(p) for p in (__file__,metrics.__file__,rolling_spots.__file__,patch.__file__,dependency_metrics.__file__,cache.__file__,exact_cache.__file__,future_proofs.__file__,future_selection.__file__,exact_dyadic.__file__)})
    def load(self,path,layer,forecast_key):
        with np.load(path,allow_pickle=False) as z:
            def unpack(x):
                if isinstance(x,dict):
                    if set(x)=={'array'}:return z[x['array']].copy()
                    return {k:unpack(v) for k,v in x.items()}
                if isinstance(x,list):return [unpack(v) for v in x]
                return x
            data=unpack(json.loads(str(z['metadata'])))
        if data['implementation']!=self.implementation():raise ValueError('PlannerImplementationChanged')
        self.restore(data['checkpoint'],layer,forecast_key)
        from .local_relaxation import RelaxationPatch
        from ..forecast.detailed.patch import Patch
        for key,row in data.get('supports',{}).items():
            d=row['detailed'];patch=Patch(*(np.asarray(d[k],dtype='<i8') for k in ('core_columns','columns','vertices','core_vertices','side_edges')),d['topology_identity'])
            self.supports[key]=RelaxationPatch(*(np.asarray(row[k],dtype='<i8') for k in ('active','fixed_boundary','halo','affected_columns','verification_ring')),patch)
        self.detailed.entries=data['detailed']
        for r in self.detailed.entries.values():r['face_ids']=[tuple(v) for v in r['face_ids']]
        self.spots.selections=OrderedDict((key,({int(k):[tuple(map(tuple,pair)) for pair in v] for k,v in choices.items()},{int(k):v for k,v in counts.items()},budget)) for key,(choices,counts,budget) in data['selection'])
        s=data['spatial'];self.spots.spatial.topology=s['topology'];self.spots.spatial.lo=s['lo'];self.spots.spatial.hi=s['hi'];self.spots.spatial.rows={int(k):v for k,v in s['rows'].items()}
        self.spots.spatial.bounds=None
        if 'future_proofs' in data:self.spots.proofs.restore(data['future_proofs'])
