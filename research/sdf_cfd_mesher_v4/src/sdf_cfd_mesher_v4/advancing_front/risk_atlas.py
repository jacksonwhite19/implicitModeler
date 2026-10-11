"""Persistent vertex-owned trajectories and dependency-complete active work.

One stable region per source vertex follows the uncoupled SDF dependency.
Connected active supports use the established metric/verification closure.
Passive ownership never exempts a cell or face from immediate whole-layer gates.
"""
from enum import IntEnum
from pathlib import Path
import copy,json
import numpy as np
from ..common.authority import array_hash,identity
from ..forecast.detailed.scientific import scientific_hash
from ..runtime.regions import dependencies
from ..forecast.engine import vertex_trajectories
from .rolling_correction import bounded_patch

class AtlasState(IntEnum):
    PassiveNominal=0
    ActiveRisk=1
    DependencyInvalidated=2
    NeedsDetailedRefresh=3

class RiskAtlas:
    def __init__(self,front,top,schedule,sdf_identity,coordinates,normals,*,copy_buffers=True):
        self.front=front;self.top=top;self.schedule=schedule
        self.authority=identity(dict(topology=top.sha256,geometry=front.geometry_authority,surface=front.surface_authority,
            schedule=array_hash(schedule.targets),sdf=sdf_identity,version='V4DynamicRiskAtlas/1'))
        self.coordinates=np.asarray(coordinates);self.normals=np.asarray(normals);n=len(front.coordinates)
        if copy_buffers:self.coordinates=self.coordinates.copy();self.normals=self.normals.copy()
        if self.coordinates.shape!=(len(schedule.targets),n,3) or self.normals.shape!=self.coordinates.shape:raise ValueError('AtlasForecastShape')
        self.layer=0;self.states=np.zeros(n,np.uint8);self.generations=np.zeros(n,np.int64);self.last_evaluated=np.zeros(n,np.int64)
        self.future_ids=np.array([self._trajectory_id(i) for i in range(n)],dtype='S64')
        self.evidence_id=identity([]);self.supports=[];self.retained=[];self.events=[];self.last_invalidation=None
    def _trajectory_id(self,i):return array_hash(self.coordinates[:,i])
    def require_committed_forecast(self,key):
        if key!=array_hash(self.coordinates):raise ValueError('CommittedForecastBindingMismatch')
    def nominal(self,layer,base):
        if layer!=self.layer+1:raise ValueError('NonsequentialAtlasLayer')
        expected=self.front.coordinates if self.layer==0 else self.coordinates[self.layer-1]
        if not np.array_equal(base.view(np.uint64),expected.view(np.uint64)):raise ValueError('AtlasCommittedDependencyMismatch')
        return self.coordinates[layer-1].copy()
    def activate(self,specs,events,layer):
        if layer!=self.layer+1:raise ValueError('NonsequentialAtlasEvidence')
        self.states[:]=AtlasState.PassiveNominal;self.supports=[bounded_patch(self.top,p['vertices']) for p in specs]
        # Direct warnings remain active even when the current policy does not
        # launch optional work (e.g. layer 1). This never creates extra searches.
        columns=np.unique([c for e in events if e['layer']>=layer for c in e['columns']]).astype(np.int64)
        if len(columns):self.states[np.unique(self.top.triangles[columns])]=AtlasState.ActiveRisk
        for p in self.supports:self.states[p.active]=AtlasState.ActiveRisk
        self.events=copy.deepcopy(events);self.evidence_id=scientific_hash(events);self.last_evaluated[:]=layer
        return self.partition()
    def partition(self):
        active=np.flatnonzero(self.states!=AtlasState.PassiveNominal)
        cols=np.flatnonzero(np.any(self.states[self.top.triangles]!=AtlasState.PassiveNominal,axis=1))
        halo=np.unique(np.concatenate([p.detailed.vertices for p in self.supports])) if self.supports else np.empty(0,np.int64)
        return dict(regions=len(self.states),active_regions=len(active),passive_regions=len(self.states)-len(active),active_vertices=active,
            active_columns=cols,dependency_vertices=halo,work_regions=len(self.supports),active_fraction=len(active)/len(self.states),active_column_fraction=len(cols)/len(self.top.triangles))
    def invalidate(self,changed):
        v,e,t=dependencies(self.top,changed);cols=set(map(int,t))
        for edge in np.unique(self.top.triangle_edges[t]):cols.update(map(int,self.top.edge_triangles.row(edge)))
        cols=np.array(sorted(cols),np.int64);rv=np.unique(self.top.triangles[cols]) if len(cols) else np.empty(0,np.int64)
        self.states[rv]=AtlasState.DependencyInvalidated;self.generations[v]+=1
        dependent=[i for i,p in enumerate(self.supports) if np.intersect1d(v,p.detailed.vertices).size]
        self.last_invalidation=dict(vertices=v,edges=e,columns=cols,regions=rv,detailed_regions=dependent,
            retained_regions=len(self.states)-len(rv),forecast_entries=len(v)*(len(self.schedule.targets)-self.layer))
        return self.last_invalidation
    def apply_preserved_candidate(self,layer,candidate,sdf):
        """Replay input replaces optimizer output, never selects a risk region."""
        expected=self.nominal(layer,self.front.coordinates if self.layer==0 else self.coordinates[self.layer-1])
        changed=np.flatnonzero(np.any(candidate.view(np.uint64)!=expected.view(np.uint64),axis=1))
        passive=self.states==AtlasState.PassiveNominal
        if np.any(passive[changed]):raise ValueError('RequiredCorrectionLeftPassive')
        before=self.future_ids.copy();inv=self.invalidate(changed)
        if len(changed):
            from types import SimpleNamespace
            self.coordinates[layer-1,changed]=candidate[changed]
            sample=sdf.evaluate(candidate[changed]);self.normals[layer-1,changed]=sample.normal
            if layer<len(self.schedule.targets):
                base=SimpleNamespace(coordinates=candidate,committed_layer=layer,target_distances=np.full(len(candidate),self.schedule.targets[layer-1]))
                data,_=vertex_trajectories(base,self.schedule,sdf,changed)
                self.coordinates[layer:,changed]=data['coordinates'];self.normals[layer:,changed]=data['normal']
            for i in changed:self.future_ids[i]=self._trajectory_id(i)
        keep=np.ones(len(candidate),bool);keep[changed]=False
        assert np.array_equal(before[keep],self.future_ids[keep])
        # Retain only this run's generic supports that controlled a change.
        for p in self.supports:
            if np.intersect1d(p.active,changed).size and not any(np.array_equal(p.active,x) for x in self.retained):self.retained.append(p.active.copy())
        self.states[inv['regions']]=AtlasState.NeedsDetailedRefresh;self.layer=layer
        return changed,inv
    def refresh_complete(self,events):
        self.events=copy.deepcopy(events);self.evidence_id=scientific_hash(events)
        self.states[:]=AtlasState.PassiveNominal
        cols=np.unique([c for e in events if e['layer']>self.layer for c in e['columns']]).astype(np.int64)
        if len(cols):self.states[np.unique(self.top.triangles[cols])]=AtlasState.ActiveRisk
    def adopt_evaluated(self,layer,coordinates,normals,events,accepted_supports=()):
        """Accept planner output into the cache, never commit mesh geometry.

        The ordinary local planner already projected and reforecast these
        values. A second forecast would be redundant. Check every unchanged
        vertex's complete trajectory and every already committed coordinate.
        """
        if layer!=self.layer+1:raise ValueError('NonsequentialAtlasLayer')
        if not np.array_equal(coordinates[:layer-1].view(np.uint64),self.coordinates[:layer-1].view(np.uint64)):raise ValueError('ChangedCommittedAtlasPrefix')
        changed=np.flatnonzero(np.any(coordinates[layer-1].view(np.uint64)!=self.coordinates[layer-1].view(np.uint64),axis=1))
        allowed=set(int(v) for p in self.supports for v in p.active)
        for support in accepted_supports:allowed.update(map(int,support))
        if not set(map(int,changed))<=allowed:raise ValueError('CorrectionOutsideActiveSupport')
        keep=np.ones(len(self.states),bool);keep[changed]=False
        if not np.array_equal(coordinates[:,keep].view(np.uint64),self.coordinates[:,keep].view(np.uint64)):raise ValueError('UnrelatedTrajectoryChanged')
        if not np.array_equal(normals[:,keep].view(np.uint64),self.normals[:,keep].view(np.uint64)):raise ValueError('UnrelatedNormalChanged')
        inv=self.invalidate(changed);self.coordinates=coordinates.copy();self.normals=normals.copy()
        for v in changed:self.future_ids[v]=self._trajectory_id(v)
        for support in accepted_supports:
            support=np.asarray(support,np.int64)
            if not any(np.array_equal(support,x) for x in self.retained):self.retained.append(support.copy())
        self.layer=layer;self.refresh_complete(events)
        return inv
    def save(self,path):
        """Compact metadata; existing sparse forecast remains the only backing."""
        p=self.checkpoint()
        # Per-vertex trajectory identities are recomputed from the bound backing
        # on restart, rather than duplicated in twenty persistent files.
        for k in ('states','generations','last_evaluated','future_ids'):p.pop(k)
        p['retained']=[x.tolist() for x in self.retained]
        np.savez_compressed(path,states=self.states,generations=self.generations,last_evaluated=self.last_evaluated,
                            metadata=np.array(json.dumps(p,sort_keys=True)),scientific_hash=np.array(self.scientific_hash()))
    def load(self,path):
        with np.load(path) as z:
            p=json.loads(str(z['metadata']))
            for k in ('states','generations','last_evaluated'):p[k]=z[k]
            p['future_ids']=self.future_ids.copy();p['retained']=[np.asarray(x,np.int64) for x in p['retained']]
            self.restore(p)
            if str(z['scientific_hash'])!=self.scientific_hash():raise ValueError('AtlasCheckpointMismatch')
    def checkpoint(self):
        return dict(authority=self.authority,layer=self.layer,states=self.states.copy(),generations=self.generations.copy(),last_evaluated=self.last_evaluated.copy(),
            future_ids=self.future_ids.copy(),evidence_id=self.evidence_id,events=copy.deepcopy(self.events),retained=[x.copy() for x in self.retained],
            coordinates_hash=array_hash(self.coordinates),normals_hash=array_hash(self.normals))
    def restore(self,snapshot):
        if snapshot['authority']!=self.authority or snapshot['coordinates_hash']!=array_hash(self.coordinates) or snapshot['normals_hash']!=array_hash(self.normals):raise ValueError('AtlasRestartAuthorityMismatch')
        self.layer=snapshot['layer']
        for key in ('states','generations','last_evaluated','future_ids'):setattr(self,key,snapshot[key].copy())
        self.events=copy.deepcopy(snapshot['events']);self.evidence_id=snapshot['evidence_id'];self.retained=[x.copy() for x in snapshot['retained']]
    def scientific_hash(self):
        p=self.checkpoint();p['future_ids']=array_hash(p['future_ids'])
        for key in ('states','generations','last_evaluated'):p[key]=array_hash(p[key])
        # Source IDs have one canonical wire width. NumPy 1.x on Windows infers
        # int32 from JSON lists; reload uses int64. The IDs, not that incidental
        # in-memory width, define the support identity.
        p['retained']=[array_hash(np.asarray(x,dtype='<i8')) for x in p['retained']];return scientific_hash(p)
