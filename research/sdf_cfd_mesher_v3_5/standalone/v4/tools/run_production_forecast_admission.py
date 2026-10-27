"""Bounded read-only capsule forecast integration evidence, independent runs."""
import json
import time
import gc
import numpy as np
import psutil
from capsule_layer2_preflight import V4,ROOT,L1,SURFACE,REQUEST,COMMITTED,SURFACE_ID,REQUEST_ID,save,file_hash,identity,array_hash
from production_surface_authority import load_production_wall
from production_adaptive_surface import CapsuleGeometry
from geometry_family_authority import Source
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
from production_forecast_admission import ProductionForecastAdapter,BoundProductionForecastCache,detailed_compatibility_probe,admission_result

OUT=V4/'results/production_layer2_forecast_admission'


def stats(a):
    a=np.asarray(a).ravel();finite=a[np.isfinite(a)].astype(float)
    return dict(count=len(a),nonfinite=int(len(a)-len(finite)),
        **(dict(zip(('min','p1','p5','median','p95','p99','max'),map(float,np.quantile(finite,[0,.01,.05,.5,.95,.99,1])))) if len(finite) else {}))


def restore_adapter():
    times={};t=time.perf_counter();wall,tri,surface=load_production_wall(SURFACE);times['wall_load']=time.perf_counter()-t
    expected=dict(committed=COMMITTED,geometry_identity='a52676253aa7996b6ac4d453f40ae838aa9d569d883b8fa64fe6799cd22cbc65',surface_identity=SURFACE_ID,
        request=REQUEST_ID,request_file='37375760bf77a7df2ff6e34ca0a87b6d437dcee604a5d8bf8ddb3c3483c40d80',
        schedule='ae2cfd505b0cb52218c4f000688df150e8f2c65f2d2d28de78785f1de39f0597',layer=1,
        front_coordinates='34ca5df3c122a7436caf8a475c1c8cf0fbe629a6afb26fe1387862ef2f884cf7')
    prior=V4/'results/capsule_advancing_front_layer2'
    for p,h in [(prior/'report.md','ba310aa3747c0e8938e20c8b8e17f037ed99555dc1b6a8a796b419dd46e64c79'),
                (prior/'AUTHORITY.json','c7acf1a903dd26f495666796a5cdfc795eaf59dd6e24d73ddc31d608929a9bdb'),
                (L1/'AUTHORITY.json','ec4c450daf520b903c289d02a35a7493a6bcf0a05191ef184f8f68c916aabab8')]:
        if file_hash(p)!=h:raise ValueError('PriorAuthorityMismatch')
    checkpoint=L1/'A/checkpoints'
    if (checkpoint/'BLOCK_TRANSACTION.json').exists():raise ValueError('ReadOnlyRestoreRequiresNoRecovery')
    print('native restoration start',flush=True);t=time.perf_counter()
    state=NativeCheckpointStore(checkpoint).restart();times['native_restore']=time.perf_counter()-t
    print('native restoration complete',times['native_restore'],flush=True)
    definition=json.loads((SURFACE.parent/'GEOMETRY.json').read_text())['definition']
    if CapsuleGeometry(definition).geometry_identity!=surface['geometry_identity']:raise ValueError('AnalyticSourceIdentityMismatch')
    t=time.perf_counter();adapter=ProductionForecastAdapter(state,wall,tri,surface,REQUEST,Source(dict(components=[definition])),expected)
    times['production_binding']=time.perf_counter()-t
    if scientific_hash(state)!=COMMITTED:raise ValueError('CommittedStateMutation')
    del state;gc.collect()
    return adapter,times,dict(surface=surface,pointer=file_hash(checkpoint/'CURRENT.json'),request=file_hash(REQUEST))


def run(label):
    start=time.perf_counter();adapter,times,before=restore_adapter()
    cache=BoundProductionForecastCache(adapter)
    print('cheap forecast start',flush=True)
    result=cache.forecast(adapter);cold=cache.cache.last_stats.copy();forecast_id=result.scientific_hash()
    print('cheap forecast complete',cold,flush=True)
    cache.forecast(adapter);warm=cache.cache.last_stats.copy()
    n=len(adapter.front.coordinates);ne=len(adapter.topology.edges);nt=len(adapter.topology.triangles)
    # A single stable source vertex exercises incidence invalidation; no coordinate change.
    before_hashes={group:{name:array_hash(value) for name,value in getattr(result,group).items()} for group in ('vertex','edge','triangle')}
    invalid=cache.invalidate(adapter,[0]);t=time.perf_counter();recomputed=cache.forecast(adapter)
    invalid_seconds=time.perf_counter()-t
    after_hashes={group:{name:array_hash(value) for name,value in getattr(recomputed,group).items()} for group in ('vertex','edge','triangle')}
    if before_hashes!=after_hashes:raise ValueError('InvalidationScientificMismatch')
    detail=detailed_compatibility_probe(adapter,result)
    decision=admission_result(adapter,result,detail)
    # No detail primitive was constructed: the existing engine guard fired first.
    arrays={group+'__'+name:value for group in ('vertex','edge','triangle') for name,value in getattr(result,group).items()}
    descriptor={group:{name:stats(value[0]) for name,value in getattr(result,group).items()
                if value.ndim==2} for group in ('vertex','edge','triangle')}
    wall,tri,surface_after=load_production_wall(SURFACE)
    if surface_after!=before['surface'] or file_hash(REQUEST)!=before['request'] or file_hash(L1/'A/checkpoints/CURRENT.json')!=before['pointer']:
        raise ValueError('InputAuthorityMutation')
    scientific=dict(binding=adapter.binding,binding_identity=adapter.identity,forecast_identity=forecast_id,
        cache_identity=cache.scientific_identity(),remaining_increments=[dict(layer=i+2,value=float(x),hex=float(x).hex()) for i,x in enumerate(adapter.remaining_increments)],
        remaining_increments_hash=array_hash(adapter.remaining_increments),forecast_layers=result.layers.tolist(),
        provisional_states=dict(vertices=len(result.layers)*n,edges=len(result.layers)*ne,triangles=len(result.layers)*nt),
        descriptor_l2=descriptor,cheap_hard_events=result.hard_events(),detail=detail,detail_identity=identity(detail),
        decision=decision,decision_identity=identity(decision),risk_identity=None,
        curvature='NotMeasured: analytic source placeholder suppressed, not treated as zero curvature',
        invalidation=dict(source_vertex=adapter.front.vertex_ids[0],ids={k:v.tolist() for k,v in invalid.items()},
                         whole_result_equal=True,outside_region_equal=True),
        no_publication=True,committed_layer=1,committed=COMMITTED,input_preserved=True)
    out=dict(scientific=scientific,scientific_identity=identity(scientific),times=times,
        cache=dict(cold=cold,warm=warm,invalidation=cache.cache.last_stats.copy(),invalidation_seconds=invalid_seconds),
        total_seconds=time.perf_counter()-start,peak_rss=psutil.Process().memory_info().peak_wset,
        retained_rss=psutil.Process().memory_info().rss,array_bytes=sum(a.nbytes for a in arrays.values()))
    save(OUT/(label+'.json'),out)
    # Retain only one forecast bulk; independent replay retains scientific hashes.
    if label=='RUN_A':np.savez_compressed(OUT/'CHEAP_FORECAST.npz',**arrays)
    print(json.dumps({k:v for k,v in out.items() if k!='scientific'}),flush=True)


if __name__=='__main__':
    import sys
    run(sys.argv[1])
