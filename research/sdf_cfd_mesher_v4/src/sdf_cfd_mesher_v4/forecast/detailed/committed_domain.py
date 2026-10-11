"""Explicit read-only domain for local lookahead after a committed prefix.

This module does not select risk supports or authorize geometry publication.
Only the restored last committed cell is needed for the first future cap pair.
The full restored prefix remains hash-bound, with absolute production layers.
"""
from dataclasses import dataclass
from pathlib import Path
import time
import numpy as np
from ...common.authority import identity, array_hash, file_hash, frozen


def implementation_identity():
    root=Path(__file__).parent
    from ...runtime.frozen_imports import legacy_modules,operator_module
    cq=legacy_modules()['component_quality']
    sources={name:file_hash(root/name) for name in ('committed_domain.py','metrics.py','patch.py')}
    sources.update({name:file_hash(module.__file__) for name,module in
                    [('component_quality',cq),('foundation_geometry',cq.FG),('prism_metrics',cq.PM),('operator',operator_module())]})
    return identity(sources)


@dataclass(frozen=True)
class CommittedDomain:
    prefix: np.ndarray
    layers: np.ndarray
    binding: dict
    sha256: str

    @classmethod
    def create(cls,front,forecast,prefix,binding):
        prefix=frozen(prefix,'<f8');layers=frozen(forecast.layers,'<i8')
        required=('committed','request','schedule','remaining_increments',
                  'remaining_targets','geometry','surface','coordinates','topology')
        if any(not binding.get(k) for k in required):raise ValueError('MissingDetailedAuthority')
        if prefix.shape!=(front.committed_layer+1,len(front.coordinates),3):
            raise ValueError('IncompleteCommittedPrefix')
        if not np.isfinite(prefix).all():raise ValueError('NonfiniteCommittedPrefix')
        if not np.array_equal(prefix[-1],front.coordinates):raise ValueError('PrefixFrontMismatch')
        if not len(layers) or not np.array_equal(layers,np.arange(front.committed_layer+1,front.committed_layer+1+len(layers))):
            raise ValueError('NoncontiguousAbsoluteFutureLayers')
        if binding['coordinates']!=front.coordinate_hash or binding['geometry']!=front.geometry_authority or binding['surface']!=front.surface_authority:
            raise ValueError('DetailedFrontAuthorityMismatch')
        key=forecast.key
        if (key.committed_coordinates!=front.coordinate_hash or key.committed_layer!=front.committed_layer
            or key.geometry!=binding['geometry'] or key.surface!=binding['surface']
            or key.remaining_schedule!=binding['remaining_targets'] or key.topology!=binding['topology']):
            raise ValueError('DetailedForecastAuthorityMismatch')
        targets=np.asarray(forecast.vertex.get('target'))
        if (targets.shape!=(len(layers),len(front.coordinates)) or
            not np.all(targets==targets[:,0,None]) or
            array_hash(targets[:,0])!=binding['remaining_targets']):
            raise ValueError('DetailedRequestWindowMismatch')
        payload=dict(binding=binding,prefix=array_hash(prefix),layers=array_hash(layers),
                     implementation=implementation_identity(),roles='wall bottom only at absolute layer 1; all other caps internal')
        return cls(prefix,layers,dict(binding),identity(payload))

    def validate(self,front,forecast,top):
        other=self.create(front,forecast,self.prefix,self.binding)
        if other.sha256!=self.sha256 or top.sha256!=self.binding['topology']:
            raise ValueError('DetailedDomainMismatch')
        if forecast.vertex['coordinates'].shape!=(len(self.layers),len(front.coordinates),3):
            raise ValueError('DetailedForecastShapeMismatch')


class CommittedDetailedCache:
    """Bounded exact patch cache; invalidation never edits authoritative inputs."""
    def __init__(self,domain,max_entries=4):
        if max_entries<1:raise ValueError('PositiveCacheBoundRequired')
        self.domain=domain;self.max_entries=max_entries;self.entries={};self.last_stats={}

    def get(self,front,forecast,top,patch):
        from .metrics import measure
        from .patch import patch_key
        start=time.perf_counter();self.domain.validate(front,forecast,top)
        key=identity(dict(domain=self.domain.sha256,patch=patch_key(front,forecast,top,patch)))
        hit=key in self.entries
        if not hit:
            result=measure(front,forecast,top,patch,domain=self.domain)
            # Freeze returned scientific arrays; callers cannot poison the cache.
            for group in ('cells','faces'):
                result[group]={k:frozen(v,v.dtype) for k,v in result[group].items()}
            if len(self.entries)>=self.max_entries:self.entries.pop(next(iter(self.entries)))
            self.entries[key]=(patch,result)
        self.last_stats=dict(hits=int(hit),misses=int(not hit),entries=len(self.entries),
                             seconds=time.perf_counter()-start,
                             recomputed_prisms=0 if hit else self.entries[key][1]['counts']['prisms'])
        return self.entries[key][1]

    def invalidate(self,vertices):
        changed=set(map(int,vertices));keys=[k for k,(p,_) in self.entries.items() if changed.intersection(p.vertices)]
        for k in keys:del self.entries[k]
        return dict(invalidated=len(keys),retained=len(self.entries))

    def scientific_identity(self):
        from .scientific import scientific_hash
        return identity(dict(domain=self.domain.sha256,
            entries={key:scientific_hash(value[1]) for key,value in sorted(self.entries.items())}))


def risk_evidence_status(cheap):
    """Fail closed: cheap descriptor events do not cover direct cell/face gates.

    production_risk.select_supports requires rolling_metrics direct event rows.
    No complete direct event inventory exists in a cheap forecast. In particular,
    an empty descriptor event set is not evidence of zero direct risk patches.
    """
    events=cheap.hard_events()
    return dict(status='BlockedDirectRiskEventCoverageRequired',cheap_hard_events=events,
                direct_hard_events='NotMeasured',risk_seeds='NotEstablished',
                patches='NotEstablished',generation_authorized=False,
                reason='Existing production selector requires direct cell/face hard-event coverage; cheap descriptors do not supply it. Whole-shell detailed screening is outside this local-only task.')
