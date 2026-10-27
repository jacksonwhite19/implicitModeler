"""Restore and bind production detailed domain; fail closed on missing risk evidence.

No production detailed patch is fabricated to bypass missing event coverage.
No publisher, layer builder, or checkpoint writer is called.
"""
import json
import time
import sys
import psutil
from capsule_layer2_preflight import V4,L1,SURFACE,REQUEST,save,file_hash,identity,array_hash
from run_production_forecast_admission import restore_adapter,stats
from production_forecast_admission import BoundProductionForecastCache
from production_surface_authority import load_production_wall
from sdf_cfd_mesher_v4.forecast.detailed.committed_domain import CommittedDomain,CommittedDetailedCache,risk_evidence_status

OUT=V4/'results/production_detailed_lookahead_committed_front'


def run(label):
    start=time.perf_counter()
    prior=V4/'results/production_layer2_forecast_admission'
    for name,sha in [('report.md','7142551272964625bd847857f12f2d4c7292ef410a3748f7d581348b691be8d0'),
                     ('AUTHORITY.json','b759737a40052886832d6b53564f3825f9b2eeb68621afc8942625f11bacae81')]:
        if file_hash(prior/name)!=sha:raise ValueError('PriorForecastAuthorityMismatch')
    adapter,times,before=restore_adapter()
    cache=BoundProductionForecastCache(adapter);cheap=cache.forecast(adapter);cold=cache.cache.last_stats.copy()
    cache.forecast(adapter);warm=cache.cache.last_stats.copy()
    if cheap.scientific_hash()!='cecbf2877e70d9a17d63ca00277ec66761e0d6094403a90a472fb6159d5f5b10':
        raise ValueError('QualifiedCheapForecastMismatch')
    begin=time.perf_counter()
    domain=CommittedDomain.create(adapter.front,cheap,adapter.committed_prefix,adapter.binding)
    domain.validate(adapter.front,cheap,adapter.topology)
    times['detailed_domain_binding']=time.perf_counter()-begin
    detailed=CommittedDetailedCache(domain)
    risk=risk_evidence_status(cheap)
    decision=dict(status='BlockedDirectRiskCoverageAndContinuousAdmission',generation_authorized=False,
        publication_authorized=False,requested_layer=2,increment=float(adapter.remaining_increments[0]),
        reason=risk['reason'],additional_dependency='No current production bounded continuous collision support/receipt is established before risk localization.')
    wall,tri,after=load_production_wall(SURFACE)
    if after!=before['surface'] or file_hash(REQUEST)!=before['request'] or file_hash(L1/'A/checkpoints/CURRENT.json')!=before['pointer']:
        raise ValueError('AuthoritativeInputMutation')
    scientific=dict(binding=adapter.binding,domain_identity=domain.sha256,
        committed_prefix_hash=array_hash(adapter.committed_prefix),remaining_schedule=array_hash(adapter.remaining_increments),
        future_layers=cheap.layers.tolist(),cheap_forecast=cheap.scientific_hash(),
        cheap_l2_descriptors={group:{name:stats(a[0]) for name,a in getattr(cheap,group).items() if a.ndim==2} for group in ('vertex','edge','triangle')},
        cheap_hard_events=cheap.hard_events(),risk=risk,risk_identity=identity(risk),
        detailed_provisional_support='NotEstablished',detailed_metrics='NotMeasured',collision_evidence='NotMeasured',
        detailed_prisms=0,detailed_cache_entries=0,detailed_cache_identity=detailed.scientific_identity(),
        decision=decision,decision_identity=identity(decision),
        provisional_cheap_states=dict(vertices=cheap.vertex['coordinates'].shape[0]*len(wall),edges=len(cheap.layers)*len(adapter.topology.edges),triangles=len(cheap.layers)*len(tri)),
        layer2_generated=False,layer2_committed=False,layer3_generated=False,core_generated=False,
        committed_layer=adapter.front.committed_layer,committed_identity=adapter.binding['committed'],
        wall_unchanged=True,request_unchanged=True,committed_unchanged=True,
        section_curvature='NotMeasured; optional diagnostic, not an admission blocker')
    result=dict(scientific=scientific,scientific_identity=identity(scientific),timings=times,
        cheap_cache=dict(cold=cold,warm=warm),total_seconds=time.perf_counter()-start,
        peak_rss=psutil.Process().memory_info().peak_wset,retained_rss=psutil.Process().memory_info().rss)
    save(OUT/(label+'.json'),result)
    print(json.dumps({k:v for k,v in result.items() if k!='scientific'}),flush=True)


if __name__=='__main__':run(sys.argv[1])
