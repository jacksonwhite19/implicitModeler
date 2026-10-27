"""Fresh read-only L2 screening using sealed forecast coordinates, not extrusion."""
import gc
import json
import time
import sys
import hashlib
import numpy as np
import psutil
from capsule_layer2_preflight import V4,L1,SURFACE,REQUEST,save,file_hash,identity,array_hash
from run_production_forecast_admission import restore_adapter
from production_surface_authority import load_production_wall
from sdf_cfd_mesher_v4.forecast.engine import ForecastResult,cache_key
from sdf_cfd_mesher_v4.forecast.detailed.committed_domain import CommittedDomain
from sdf_cfd_mesher_v4.forecast.detailed.scientific import scientific_hash,plain
from production_l2_screening import direct_screen,face_inventory,collision_screen,close_risk,distributions

OUT=V4/'results/production_layer2_risk_coverage'


def load_screen_inputs():
    prior=V4/'results/production_detailed_lookahead_committed_front'
    for name,digest in [('report.md','c528d36137ccc403f9e9977145b1aab2fc28c1f8000394ea8a0b9135bb42ff54'),('AUTHORITY.json','52566c0a2df1a2fde3598ff1af75d71a7bf57102a4ddfa0b22b506834ed262b8')]:
        if file_hash(prior/name)!=digest:raise ValueError('PreviousAuthorityMismatch')
    adapter,times,before=restore_adapter()
    if adapter.front.committed_layer!=1:raise ValueError('ExactlyL2ScreeningOnly')
    if float(adapter.remaining_increments[0]).hex()!='0x1.6b844d56c388ep-13':raise ValueError('Layer2ScheduleMismatch')
    bulk=V4/'results/production_layer2_forecast_admission/CHEAP_FORECAST.npz'
    if file_hash(bulk)!='1817403657aa3460f18079ccc5a5eb9c5d10442f71ad277f741feebf030e799e':raise ValueError('CheapBulkMismatch')
    t=time.perf_counter()
    with np.load(bulk) as z:groups=[{k.split('__',1)[1]:z[k] for k in z.files if k.startswith(g+'__')} for g in ('vertex','edge','triangle')]
    top=adapter.topology
    cheap=ForecastResult(cache_key(adapter.front,adapter.schedule,adapter.sdf),np.arange(2,len(adapter.schedule.increments)+1,dtype=np.int64),*groups,{},dict(vertices=np.asarray(adapter.front.vertex_ids,dtype='U'),edges=top.edges,triangles=top.triangles))
    if cheap.scientific_hash()!='cecbf2877e70d9a17d63ca00277ec66761e0d6094403a90a472fb6159d5f5b10':raise ValueError('CheapScientificMismatch')
    domain=CommittedDomain.create(adapter.front,cheap,adapter.committed_prefix,adapter.binding)
    if domain.sha256!='335c8bfc024ccc28ec55f7e94ace0c648679ce8d281b93adebf75861422d7d2c':raise ValueError('DetailedDomainMismatch')
    domain.validate(adapter.front,cheap,top)
    high=cheap.vertex['coordinates'][0].copy();del cheap,groups;gc.collect()
    times['sealed_forecast_load_and_binding']=time.perf_counter()-t
    return adapter,high,times,before


def run(label):
    start=time.perf_counter();adapter,high,times,before=load_screen_inputs();top=adapter.topology
    print('inputs admitted; direct L2 screening',flush=True)
    direct=direct_screen(top,adapter.committed_prefix,high,layer=2,progress=lambda x:print(json.dumps(x),flush=True))
    times['provisional_batch_construction']=direct['construction_seconds'];times['direct_including_exact']=direct['seconds']
    events=direct['events'];direct_seeds=sorted({c for e in events for c in e['columns']})
    print('direct complete',len(events),'events; broad phase start',flush=True)
    t=time.perf_counter();inventory=face_inventory(top,adapter.committed_prefix[0],adapter.front.coordinates,high);times['face_inventory']=time.perf_counter()-t
    collision=collision_screen(inventory);times['broad_phase']=collision['seconds']
    print('broad phase complete',collision['populations'],flush=True)
    seeds=np.union1d(direct_seeds,collision['seed_columns']).astype(np.int64)
    closure=close_risk(top,seeds);times['patch_closure']=closure.pop('seconds')
    cells=direct['cells'];sides=direct['sides'];caps=direct['caps']
    metrics={group:{k:array_hash(v) for k,v in data.items()} for group,data in [('cells',cells),('sides',sides),('caps',caps)]}
    worst={}
    for k in ('volume','signed_volume','exact_volume','jacobian','exact_jacobian','determinant','aspect'):
        idx=int(np.argmax(cells[k]) if k=='aspect' else np.argmin(cells[k]));worst[k]=dict(source_column=idx,value=float(cells[k][idx]))
    for name in ('weight','skewness','symmetric_volume_ratio'):
        values=np.r_[sides[name],caps[name]];idx=int(np.argmax(values) if name=='skewness' else np.argmin(values))
        is_side=idx<len(top.edges);entity=idx if is_side else idx-len(top.edges)
        worst[name]=dict(kind='side' if is_side else 'committed_cap',entity=entity,source_columns=top.edge_triangles.row(entity).tolist() if is_side else [entity],value=float(values[idx]))
    ownership=dict(source_vertices=len(high),source_edges=len(top.edges),source_columns=len(top.triangles),screened_columns=len(cells['volume']),
        unique_column_ids=len(top.triangles),missing=0,duplicate=0,orphan=0,foreign=0,
        source_binding=adapter.binding['ownership'],rule='Stable source triangle order, one provisional prism per restored active source column; exact original connectivity retained')
    collision_hashes={k:array_hash(collision[k]) for k in ('pairs','incident','seed_columns')}
    inventory_hashes={k:array_hash(v) for k,v in inventory.items()}
    scientific=dict(binding=adapter.binding,committed_prefix=array_hash(adapter.committed_prefix),screen_geometry=identity(dict(low=adapter.front.coordinate_hash,high=array_hash(high),triangles=array_hash(top.triangles))),
        provisional_l2_coordinates=array_hash(high),counts=direct['counts'],topology=direct['topology'],ownership=ownership,metrics=metrics,metric_identity=identity(metrics),
        distributions={group:{k:distributions(v) for k,v in data.items() if v.ndim==1} for group,data in [('cells',cells),('sides',sides),('caps',caps)]},worst=worst,
        combined_face_distributions={k:distributions(np.r_[sides[k],caps[k]]) for k in sides},
        hard_events=plain(events),hard_events_identity=scientific_hash(events),direct_seed_columns=direct_seeds,
        exact_record_hash=direct['exact_record_hash'],collision_populations=collision['populations'],collision_hashes=collision_hashes,
        collision_identity=identity(dict(inventory=inventory_hashes,candidates=collision_hashes)),collision_semantics=collision['semantics'],
        seed_columns=seeds.tolist(),seed_identity=array_hash(seeds),closure=closure,
        decision=dict(status='BlockedCollisionRiskClosureExceedsQualifiedLocality' if closure['blocked_groups'] else 'CompleteBoundedRiskSupports',
                      direct_screening_complete=True,conservative_face_candidate_coverage=True,bounded_support_population_complete=not bool(closure['blocked_groups']),
                      continuous_certification=False,l2_admission=False,publication_authorized=False),
        publication=dict(L2_generated=False,L2_committed=False,L3_generated=False,core_generated=False,committed_layer=1,committed=adapter.binding['committed']))
    wall,tri,surface=load_production_wall(SURFACE)
    if surface!=before['surface'] or file_hash(REQUEST)!=before['request'] or file_hash(L1/'A/checkpoints/CURRENT.json')!=before['pointer']:raise ValueError('InputMutation')
    scientific['input_preservation']=dict(wall=True,L1=True,request=True)
    scientific['decision_identity']=identity(scientific['decision'])
    result=dict(scientific=scientific,scientific_identity=identity(scientific),timings=times,total_seconds=time.perf_counter()-start,
                peak_rss=psutil.Process().memory_info().peak_wset,retained_rss=psutil.Process().memory_info().rss)
    save(OUT/(label+'.json'),result)
    if label=='RUN_A':
        np.savez_compressed(OUT/'SCREEN_ARRAYS.npz',provisional_high=high,**{g+'__'+k:v for g,data in [('cells',cells),('sides',sides),('caps',caps)] for k,v in data.items()},
            candidate_pairs=collision['pairs'],incident=collision['incident'],face_labels=inventory['labels'],face_columns=inventory['columns'],face_kinds=inventory['kinds'])
        with (OUT/'EXACT_WEDGES.jsonl').open('w') as f:
            for row in direct['exact_records']:f.write(json.dumps(row,sort_keys=True,separators=(',',':'))+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='scientific'}),flush=True)


if __name__=='__main__':run(sys.argv[1])
