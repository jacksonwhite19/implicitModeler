"""Geometry-keyed patch cache and explicit fork of the existing forecast cache."""
import copy,time
import numpy as np
from ...common.authority import array_hash
from ..engine import cache_key
from .patch import patch_key
from .metrics import measure
from .dependency_metrics import LocalMetricEngine

class DetailedCache:
    def __init__(self):
        self.entries={};self.hits=0;self.misses=0;self.engine=LocalMetricEngine();self.last_stats={}
    def get(self,front,forecast,topology,patch):
        key=patch_key(front,forecast,topology,patch)
        if key in self.entries:
            self.hits+=1;self.last_stats=dict(exact_patch_hit=True);return self.entries[key]
        self.misses+=1;r=self.engine.measure(front,forecast,topology,patch)
        self.last_stats=dict(self.engine.last_stats,exact_patch_hit=False);self.entries[key]=r;return r
    def invalidate(self,patches,changed_vertices):
        # Keys include local coordinates: invalidated entries cannot be returned
        # under changed geometry. Return logical invalidations; old geometry
        # entries remain available for exact comparison during this test fork.
        changed=set(map(int,changed_vertices));affected=[i for i,p in enumerate(patches) if changed.intersection(p.vertices)]
        return affected

def fork_changed_forecast(cache,old,new,topology,schedule,sdf,changed):
    """Reuse Task-1 invalidation only after proving the unchanged domain identical.

    This is a test-copy transaction. Never edits either committed front. The old
    cache remains intact. All source and schedule identities are checked.
    """
    changed=np.unique(np.asarray(changed,dtype=np.int64))
    if old.topology_hash!=new.topology_hash or old.vertex_ids!=new.vertex_ids or old.source_ancestry!=new.source_ancestry or old.committed_layer!=new.committed_layer:raise ValueError('NonCoordinateChange')
    if not np.array_equal(old.target_distances,new.target_distances):raise ValueError('TargetChange')
    actual=np.flatnonzero(np.any(old.coordinates!=new.coordinates,axis=1))
    if not np.array_equal(actual,changed):raise ValueError('UnreportedChangedVertices')
    before=cache_key(old,schedule,sdf);after=cache_key(new,schedule,sdf)
    if cache.result.key!=before:raise ValueError('OriginalCacheMismatch')
    from dataclasses import asdict
    a=asdict(before);b=asdict(after);a.pop('committed_coordinates');b.pop('committed_coordinates')
    if a!=b:raise ValueError('ChangedAuthorityBeyondCoordinates')
    fork=copy.deepcopy(cache);fork.result.key=after
    invalid=fork.invalidate(topology,changed)
    result=fork.forecast(new,topology,schedule,sdf)
    unaffected={}
    for name,ids in (('vertex',invalid['vertices']),('edge',invalid['edges']),('triangle',invalid['triangles'])):
        n=next(iter(getattr(result,name).values())).shape[1];keep=np.ones(n,bool);keep[ids]=False
        unaffected[name]=all(np.array_equal(v[:,keep],getattr(cache.result,name)[key][:,keep],equal_nan=True) for key,v in getattr(result,name).items())
    if not all(unaffected.values()):raise AssertionError('UnrelatedForecastChanged')
    return fork,dict(invalidated={k:v.tolist() for k,v in invalid.items()},unrelated_exact=unaffected,**fork.last_stats)
