"""Exact-authority local proof reuse; the qualified pair mathematics is frozen.

The complete scope is a set of source side edges over supplied layer states.
An unchanged pair is reusable only if both surface identities are identical.
Spatial exclusion uses the existing conservative source-edge scan and BVH.
No missing cache record is interpreted as a separated pair.
"""
from dataclasses import dataclass
from collections import OrderedDict,Counter
import hashlib,json,time
import numpy as np
from .continuous_prism_certificate import repaired_kernel as kernel
from .continuous_prism_certificate.runtime import canonical_bytes

VERSION='V4IncrementalSideProof/1'

def canonical_pair_indices(pairs,surface_count):
    """Pack in explicit int64; Windows NumPy default int can be 32-bit."""
    a=np.asarray(pairs,dtype=np.int64).reshape(-1,2)
    if surface_count<0 or surface_count>np.iinfo(np.int32).max or surface_count*surface_count>np.iinfo(np.int64).max:raise ValueError('PairIndexRange')
    if len(a) and (a.min()<0 or a.max()>=surface_count):raise ValueError('InvalidPairIndex')
    if not len(a):return np.empty((0,2),np.int32)
    packed=np.unique(a[:,0]*surface_count+a[:,1])
    return np.stack((packed//surface_count,packed%surface_count),axis=1).astype(np.int32)

@dataclass(frozen=True)
class SurfaceScope:
    quads: np.ndarray
    labels: tuple
    sides: tuple
    core: np.ndarray
    topology: str
    edges: np.ndarray

    def identity(self,i):
        # Binary64 bits, including signed zero, make reuse conservative.
        # The exact kernel consumes those bits as rational coordinates.
        header=dict(version=VERSION,kernel=kernel.VERSION,topology=self.topology,
                    side=self.sides[i],labels=self.labels[i],declared_dimension=2)
        return hashlib.sha256(canonical_bytes(header)+np.asarray(self.quads[i],dtype='<f8').tobytes()).hexdigest()

    def patch(self,i):return kernel.Patch.binary(self.quads[i],self.labels[i])

    def pairs(self):return conservative_pairs(self.quads,self.core)

    def scope_identity(self):
        header=dict(version=VERSION,kernel=kernel.VERSION,topology=self.topology,sides=self.sides,labels=self.labels)
        h=hashlib.sha256(canonical_bytes(header));h.update(np.asarray(self.quads,dtype='<f8').tobytes());h.update(self.core.tobytes())
        return h.hexdigest()

def conservative_pairs(quads,core):
    """Vectorized leaf tests with exactly the frozen AABB overlap predicate.

    Nodes with no core surface on either side are irrelevant to this scope.
    No distance cutoff, transformed geometry, or floating sign test is added.
    Canonical deduplication removes traversal ordering from proof ordering.
    """
    lo,hi=kernel.enumerator.boxes(quads);nodes=[];core=np.asarray(core,bool)
    def build(ids):
        i=len(nodes);L=lo[ids].min(0);H=hi[ids].max(0)
        nodes.append([L,H,ids,None,bool(core[ids].any())])
        if len(ids)>8:
            axis=int(np.argmax(H-L));order=np.lexsort((ids,(lo[ids,axis]+hi[ids,axis])*.5));ids=ids[order];m=len(ids)//2
            nodes[i][3]=(build(ids[:m]),build(ids[m:]))
        return i
    if not len(lo):return np.empty((0,2),np.int32),dict(node_count=0,discoveries=0,duplicates_removed=0)
    work=[(build(np.arange(len(lo),dtype=np.int64)),0)];blocks=[];leaf_tests=0
    while work:
        ia,ib=work.pop();a,b=nodes[ia],nodes[ib]
        if not (a[4] or b[4]) or np.any(a[1]<b[0]) or np.any(b[1]<a[0]):continue
        ca,cb=a[3],b[3]
        if ca is None and cb is None:
            x,y=a[2],b[2];mask=np.all(hi[x,None]>=lo[None,y],axis=2)&np.all(lo[x,None]<=hi[None,y],axis=2)
            mask&=(core[x,None]|core[None,y]);mask&=(x[:,None]!=y[None,:]);leaf_tests+=len(x)*len(y)
            i,j=np.where(mask)
            if len(i):blocks.append(np.stack((np.minimum(x[i],y[j]),np.maximum(x[i],y[j])),axis=1))
        elif ia==ib:
            l,r=ca;work.extend(((l,l),(l,r),(r,r)))
        elif cb is None or (ca is not None and len(a[2])>=len(b[2])):work.extend(((ca[0],ib),(ca[1],ib)))
        else:work.extend(((ia,cb[0]),(ia,cb[1])))
    allpairs=np.concatenate(blocks) if blocks else np.empty((0,2),np.int64)
    pairs=canonical_pair_indices(allpairs,len(lo))
    return pairs,dict(node_count=len(nodes),discoveries=len(allpairs),duplicates_removed=len(allpairs)-len(pairs),leaf_comparisons=leaf_tests)

def build_scope(index,core_edges,*,max_edges=8192,layer_offset=0):
    """Complete changed-vs-changed AND changed-vs-surroundings side scope.

    The existing swept-edge scan reads whole-front AABBs but extracts only the
    bounded interaction halo. A bilinear image stays in its four-point hull;
    thus strict AABB exclusion cannot remove an intersecting image pair.
    """
    edges=index.candidates(core_edges,max_edges=max_edges);ep=index.topology.edges[edges]
    quads=[];labels=[];sides=[];previous=index.front.coordinates
    core_set=set(map(int,core_edges));flags=[]
    for k,x in enumerate(index.forecast.vertex['coordinates'],layer_offset+1):
        quads.extend(np.stack((previous[ep[:,0]],previous[ep[:,1]],x[ep[:,0]],x[ep[:,1]]),axis=1))
        labels.extend(((k-1,int(a)),(k-1,int(b)),(k,int(a)),(k,int(b))) for a,b in ep)
        sides.extend((k,int(e)) for e in edges);flags.extend(int(e) in core_set for e in edges);previous=x
    q=np.asarray(quads,dtype='<f8');flags=np.asarray(flags,bool)
    if not np.isfinite(q).all():raise ValueError('NonfiniteCertificateCoordinate')
    q.setflags(write=False);flags.setflags(write=False)
    return SurfaceScope(q,tuple(labels),tuple(sides),flags,index.topology.sha256,edges)

def pair_identity(scope,a,b,identities=None):
    ia,ib=(scope.identity(a),scope.identity(b)) if identities is None else (identities[a],identities[b])
    return hashlib.sha256(canonical_bytes(dict(version=VERSION,kernel=kernel.VERSION,surfaces=sorted((ia,ib))))).hexdigest()

class ProofCache:
    """Bounded immutable exact records. Eviction changes work, never results.

    Byte accounting bounds stored keys and encoded values, not Python overhead.
    RSS is measured separately. Old identities remain available for rollback.
    """
    def __init__(self,max_bytes=128*1024*1024):
        if max_bytes<0:raise ValueError('NegativeProofCacheBudget')
        self.max_bytes=max_bytes;self.entries=OrderedDict();self.bytes=0
        self.hits=self.misses=self.evictions=0
        self.enumerations=OrderedDict();self.enumeration_bytes=0
        self.enumeration_budget=64*1024*1024
    def get(self,key):
        if key not in self.entries:self.misses+=1;return None
        self.hits+=1;b=self.entries.pop(key);self.entries[key]=b
        return json.loads(b)
    def put(self,key,result):
        b=canonical_bytes(result);size=len(key)+len(b)
        if size>self.max_bytes:return
        if key in self.entries:self.bytes-=len(key)+len(self.entries.pop(key))
        while self.entries and self.bytes+size>self.max_bytes:
            k,v=self.entries.popitem(last=False);self.bytes-=len(k)+len(v);self.evictions+=1
        self.entries[key]=b;self.bytes+=size

    def pairs(self,scope):
        key=scope.scope_identity()
        if key in self.enumerations:
            v=self.enumerations.pop(key);self.enumerations[key]=v
            return v[0],v[1],True
        pairs,stats=scope.pairs();pairs.setflags(write=False)
        if pairs.nbytes<=self.enumeration_budget:
            while self.enumerations and self.enumeration_bytes+pairs.nbytes>self.enumeration_budget:
                _,old=self.enumerations.popitem(last=False);self.enumeration_bytes-=old[0].nbytes
            self.enumerations[key]=(pairs,stats);self.enumeration_bytes+=pairs.nbytes
        return pairs,stats,False

def dependency_changes(old,new):
    before={s:old.identity(i) for i,s in enumerate(old.sides)}
    after={s:new.identity(i) for i,s in enumerate(new.sides)}
    changed=sorted(s for s,h in after.items() if before.get(s)!=h)
    return dict(changed_sides=changed,unchanged_sides=sorted(s for s,h in after.items() if before.get(s)==h),
                removed_sides=sorted(set(before)-set(after)),changed_layers=sorted({s[0] for s in changed}),
                changed_edges=sorted({s[1] for s in changed}))

def proof_invalidation(old,new,completed_pairs=None):
    """Invalidate active proof bindings, retain immutable history for rollback.

    Counts refer only to a completed prefix when a previous scope was bounded.
    No proof is claimed to exist merely because an old pair was enumerated.
    """
    pairs,_=old.pairs()
    if completed_pairs is not None:pairs=pairs[:completed_pairs]
    after={s:new.identity(i) for i,s in enumerate(new.sides)}
    unchanged=np.array([after.get(s)==old.identity(i) for i,s in enumerate(old.sides)])
    valid=unchanged[pairs[:,0]]&unchanged[pairs[:,1]]
    return dict(previous_proofs=len(pairs),invalidated_proofs=int((~valid).sum()),unchanged_proof_bindings=int(valid.sum()))

def private_admission(*,immediate_numeric,immediate_exact,current_certificate_complete,
                      future_certificate,new_earlier_failures,target_maintained_or_improved):
    """Read-only admission decision. No private or production state is advanced."""
    reasons=[]
    if not immediate_numeric or not immediate_exact:reasons.append('ImmediateHardGate')
    if not current_certificate_complete:reasons.append('CurrentCertificateIncompleteOrFailed')
    required=('complete','candidate_pairs','completed_pairs','forbidden','unresolved','invalid_surfaces')
    complete=all(k in future_certificate for k in required) and future_certificate['complete'] and future_certificate['candidate_pairs']==future_certificate['completed_pairs']
    if not complete:reasons.append('FutureCertificateIncomplete')
    for key in ('forbidden','unresolved','invalid_surfaces'):
        if future_certificate.get(key):reasons.append('FutureCertificate:'+key)
    if new_earlier_failures:reasons.append('NewEarlierFutureFailure')
    if not target_maintained_or_improved:reasons.append('TargetTrajectoryRegressed')
    return dict(admissible=not reasons,reasons=reasons,advances_state=False)

def certify(scope,*,cache=None,seconds_budget=180,max_completed_pairs=None,record_sink=None,progress=None):
    """Complete deterministic traversal or explicit incomplete prefix.

    Invalid input surfaces and forbidden/unresolved pairs remain diagnostics.
    A timeout never authorizes a candidate. Runtime is outside scientific data.
    """
    cache=ProofCache(0) if cache is None else cache
    started=time.perf_counter();pairs,bvh,enumeration_hit=cache.pairs(scope);enumeration_seconds=time.perf_counter()-started
    identities=[scope.identity(i) for i in range(len(scope.sides))]
    patches={};invalid={};counts=Counter();methods=Counter();bad=[];unresolved=[];h=hashlib.sha256()
    hits=cache.hits;misses=cache.misses;kernel_calls=0;completed=0
    def get(i):
        if i not in patches:
            p=scope.patch(i);valid,reason=p.valid();patches[i]=p
            if not valid:invalid[i]=dict(side=scope.sides[i],reason=reason)
        return patches[i]
    for i in np.flatnonzero(scope.core):get(int(i))
    for a,b in pairs:
        a,b=int(a),int(b)
        if time.perf_counter()-started>=seconds_budget:break
        if max_completed_pairs is not None and completed>=max_completed_pairs:break
        pa,pb=get(a),get(b)
        key=pair_identity(scope,a,b,identities);r=cache.get(key)
        if r is None:
            if a in invalid or b in invalid:
                r=dict(classification='InvalidSidePatch',method='FrozenPatchValidity',invalid_sides=[scope.sides[i] for i in (a,b) if i in invalid])
            else:r=kernel.certificate.certify(pa,pb);kernel_calls+=1
            cache.put(key,r)
        record=dict(side_a=scope.sides[a],side_b=scope.sides[b],**r)
        encoded=canonical_bytes(record);h.update(encoded)
        if record_sink is not None:record_sink(encoded)
        counts[r['classification']]+=1;methods[r['method']]+=1;completed+=1
        if r['classification']=='CertifiedForbiddenIntersection':bad.append(record)
        elif r['classification'] not in ('CertifiedSeparated','CertifiedAllowedIncidentContact'):unresolved.append(record)
        if progress and completed%1000==0:progress(dict(completed=completed,total=len(pairs),seconds=time.perf_counter()-started,hits=cache.hits-hits))
    complete=completed==len(pairs)
    scientific=dict(complete=complete,candidate_pairs=len(pairs),completed_pairs=completed,counts=dict(counts),methods=dict(methods),
        forbidden=bad,unresolved=unresolved,invalid_surfaces=[invalid[i] for i in sorted(invalid)],records_sha256=h.hexdigest(),
        geometry_sha256=hashlib.sha256(canonical_bytes(identities)).hexdigest(),topology=scope.topology,kernel=kernel.VERSION,
        reason=None if complete else ('ComputationalWorkSlice' if max_completed_pairs is not None and completed>=max_completed_pairs else 'ComputationalTimeBudget'))
    performance=dict(seconds=time.perf_counter()-started,cache_hits=cache.hits-hits,cache_misses=cache.misses-misses,
        exact_kernel_calls=kernel_calls,cache_entries=len(cache.entries),cache_bytes=cache.bytes,cache_evictions=cache.evictions,bvh=bvh,
        enumeration_cache_hit=enumeration_hit,enumeration_cache_bytes=cache.enumeration_bytes,enumeration_seconds=enumeration_seconds)
    return scientific,performance
