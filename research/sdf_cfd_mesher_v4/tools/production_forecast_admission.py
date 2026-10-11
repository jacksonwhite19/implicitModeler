"""Read-only production authority adapter for existing forecast components.

No fixture wall construction, historical schedule loader, prism construction,
or publisher is exposed here. Detailed lookahead remains fail-closed at its
existing layer-zero/twenty-layer guard. A cheap forecast is not admission.
"""
from dataclasses import asdict
import time
from types import SimpleNamespace
import numpy as np
from production_prism_sizing import read_request
from sdf_cfd_mesher_v4.common.authority import identity, array_hash, frozen
from sdf_cfd_mesher_v4.geometry.front import CommittedFrontState, build_topology
from sdf_cfd_mesher_v4.forecast.schedule import LayerSchedule
from sdf_cfd_mesher_v4.forecast.engine import ForecastCache, cache_key
from sdf_cfd_mesher_v4.forecast.detailed.cache import DetailedCache
from sdf_cfd_mesher_v4.forecast.detailed.patch import column_patch
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash


class TimedProductionSource:
    """Instrument an admitted analytic source; do not invent curvature data."""
    def __init__(self, source, geometry_identity):
        self.source=source;self.seconds=0.
        self.source_identity=identity(dict(analytic_source=source.source_identity,
            geometry_authority=geometry_identity,adapter='TimedProductionSource/1',
            section_curvature='Unavailable; legacy analytic placeholder suppressed'))

    def evaluate(self, points):
        t=time.perf_counter();sample=self.source.evaluate(points);self.seconds+=time.perf_counter()-t
        values=vars(sample).copy()
        # Source.section_curvature is a zero placeholder, not measured curvature.
        values['section_curvature']=np.full(len(points),np.nan)
        return SimpleNamespace(**values)


class ProductionForecastAdapter:
    def __init__(self,state,wall,triangles,surface,request_path,source,expected):
        restored=scientific_hash(state)
        self._bind(state,wall,triangles,surface,request_path,source,expected,restored)

    @classmethod
    def from_immutable_access(cls,state,wall,triangles,surface,request_path,source,expected):
        """Explicit qualified read interface; ordinary legacy entry is unchanged."""
        from immutable_state_access import ImmutableState
        if type(state) is not ImmutableState:raise ValueError('UnqualifiedImmutableAccessType')
        restored=state.verified_legacy_identity()
        result=cls.__new__(cls)
        result._bind(state,wall,triangles,surface,request_path,source,expected,restored)
        state.check()
        return result

    def _bind(self,state,wall,triangles,surface,request_path,source,expected,restored):
        if restored!=expected['committed']:raise ValueError('CommittedFrontIdentityMismatch')
        for key in ('geometry_identity','surface_identity'):
            if surface[key]!=expected[key]:raise ValueError('ProductionAuthorityMismatch:'+key)
        bound=read_request(request_path,surface)
        request_id=identity(bound['declaration'])
        if request_id!=expected['request']:raise ValueError('RequestIdentityMismatch')
        if bound['schedule_identity']!=expected['schedule']:raise ValueError('ScheduleIdentityMismatch')
        if bound['request_file_sha256']!=expected['request_file']:raise ValueError('RequestFileMismatch')
        layers={int(v['last_prism_layer']) for v in state['columns'].values()}
        if len(layers)!=1:raise ValueError('NonuniformCommittedFrontUnsupported')
        layer=layers.pop()
        if layer!=expected['layer']:raise ValueError('CommittedLayerMismatch')
        if not 0<layer<len(bound['increments']):raise ValueError('NoPendingIncrement')
        if array_hash(wall)!=surface['coordinate_hash'] or array_hash(triangles)!=surface['topology_hash']:
            raise ValueError('WallPayloadMismatch')
        if set(state['prism_front'])!={str(i) for i in range(len(triangles))}:
            raise ValueError('SourceColumnInventoryMismatch')
        n=len(wall);ids=tuple(f'layer:{layer}:source:{i}' for i in range(n))
        x=np.array([state['vertices'][k] for k in ids])
        if array_hash(x)!=expected['front_coordinates']:raise ValueError('FrontCoordinateMismatch')
        original=np.array([state['vertices'][f'layer:0:source:{i}'] for i in range(n)])
        if not np.array_equal(original,wall):raise ValueError('RestoredWallMismatch')
        ancestry=[[] for _ in range(n)]
        for i,tri in enumerate(triangles):
            face=state['faces'][state['prism_front'][str(i)]]
            if set(face['vertices'])!={ids[int(v)] for v in tri}:raise ValueError('FrontSourceTopologyMismatch')
            for v in tri:ancestry[int(v)].append(i)
        self.front=CommittedFrontState.create(layer,x,ids,triangles,ancestry,
            np.full(n,bound['targets'][layer-1]),surface['geometry_identity'],surface['surface_identity'])
        self.topology=build_topology(triangles,n)
        self.schedule=LayerSchedule(bound['increments'],bound['targets'],bound['schedule_identity'],
            bound['request_file_sha256'],float(bound['increments'][0]),float.fromhex(bound['declaration']['growth_hex']))
        # Restore the existing prefix, never reconstruct it by extrusion.
        self.committed_prefix=frozen(np.array([[state['vertices'][f'layer:{k}:source:{i}']
            for i in range(n)] for k in range(layer+1)]),'<f8')
        self.remaining_increments=frozen(bound['increments'][layer:],'<f8')
        self.sdf=TimedProductionSource(source,surface['geometry_identity'])
        self.binding=dict(version='V4ProductionForecastBinding/1',committed=restored,
            request=request_id,request_file=bound['request_file_sha256'],schedule=bound['schedule_identity'],
            remaining_increments=array_hash(self.remaining_increments),
            remaining_targets=array_hash(self.schedule.remaining(layer)),
            geometry=surface['geometry_identity'],surface=surface['surface_identity'],
            coordinates=self.front.coordinate_hash,topology=self.topology.sha256,
            ownership=identity(dict(vertex_ids=ids,source_ancestry=self.front.source_ancestry)),
            layer=layer,next_layer=layer+1)
        self.identity=identity(self.binding)


class BoundProductionForecastCache:
    """Enclose the existing cache in a request/committed-state authority domain."""
    def __init__(self,adapter,*,last_layer=None):
        self.binding_identity=adapter.identity
        self.last_layer=last_layer
        self.initial_key=cache_key(adapter.front,adapter.schedule,adapter.sdf,last_layer=last_layer)
        self.cache=ForecastCache()

    def check(self,adapter):
        if identity(adapter.binding)!=self.binding_identity or adapter.identity!=self.binding_identity:
            raise ValueError('ProductionCacheAuthorityMismatch')
        if cache_key(adapter.front,adapter.schedule,adapter.sdf,last_layer=self.last_layer)!=self.initial_key:
            raise ValueError('ProductionCacheInputMismatch')

    def forecast(self,adapter):
        self.check(adapter)
        return self.cache.forecast(adapter.front,adapter.topology,adapter.schedule,adapter.sdf,last_layer=self.last_layer)

    def invalidate(self,adapter,vertices):
        self.check(adapter)
        return self.cache.invalidate(adapter.topology,vertices)

    def scientific_identity(self):
        return identity(dict(binding=self.binding_identity,key=asdict(self.initial_key),
                             result=None if self.cache.result is None else self.cache.result.scientific_hash()))


def detailed_compatibility_probe(adapter,forecast):
    """One stable support tests the existing guard, not a invented risk selector.

    Stop before entering detailed code if its known precondition is satisfied:
    this probe is exclusively for the incompatible production-restoration case.
    """
    if adapter.front.committed_layer==0 and np.array_equal(forecast.layers,np.arange(1,21)):
        raise ValueError('CompatibilityProbeMustNotConstructPrisms')
    patch=column_patch(adapter.topology,0,rings=0)
    cache=DetailedCache()
    try: cache.get(adapter.front,forecast,adapter.topology,patch)
    except ValueError as error:
        if str(error)!='Task2RequiresFrozenTwentyLayerForecast':raise
        return dict(status='BlockedHistoricalDetailedDomain',error=str(error),
            compatibility_support=patch.record(),risk_selected=False,
            detailed_hits=cache.hits,detailed_misses=cache.misses,
            computed_entries=len(cache.entries),provisional_prisms=0)
    raise AssertionError('DetailedDomainChangedRequiresReaudit')


def admission_result(adapter,forecast,detail):
    # Existing hard events are disclosed, not promoted to sufficient admission.
    return dict(status='BlockedDetailedLookaheadAndRiskAdmissionIntegration',
        generation_authorized=False,publication_authorized=False,
        current=adapter.binding['committed'],next_layer=adapter.binding['next_layer'],
        increment=float(adapter.remaining_increments[0]),binding=adapter.identity,
        cheap_hard_events=forecast.hard_events(),detailed_status=detail['status'],
        reason='Existing detailed engine requires committed layer 0 and forecast layers 1..20; no production risk/admission rule can be inferred from descriptor extrema or historical SF2 cohorts.',
        risk_patches='NotEstablished',geometric_certification='Not performed')
