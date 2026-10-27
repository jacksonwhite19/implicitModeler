"""One bound production step through the generalized, qualified L2 machinery.

The study configuration selects L2 -> L3. Meshing, metrics, collision proofs,
transactions and receipt publication remain the existing generic algorithms.
No continuation after the requested step is exposed by this driver.
"""
import argparse
import gc
import json
import os
from pathlib import Path
import shutil
import time
import numpy as np
import psutil
from capsule_layer2_preflight import V4,SURFACE,REQUEST
from production_surface_authority import load_production_wall
from production_forecast_admission import ProductionForecastAdapter,BoundProductionForecastCache
from production_layer2_run import ProductionNextLayerProvider
from geometry_family_authority import Source
from production_l2_screening import direct_screen,prefix_face_inventory,collision_screen,close_risk,distributions
from production_collision_localization import classify,risk_columns,METHODS
from production_readonly_admission import evaluate,REQUIRED
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore,ReceiptOverlay
from sdf_cfd_mesher_v4.advancing_front.native_receipt_delta import ReceiptDelta
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
from sdf_cfd_mesher_v4.advancing_front.mixed_front_runner import MixedFrontRunner,preserves_committed_cells
from sdf_cfd_mesher_v4.advancing_front.mixed_front_journal import InMemoryMixedFrontJournal
from sdf_cfd_mesher_v4.common.authority import identity,file_hash,array_hash
from path_preserving_deduplication import durable_json,metadata,physical_key

OUT=V4/'results/production_adaptive_capsule_layer3'
BASE=V4/'results/production_adaptive_capsule_layer2/A/checkpoints'
CONFIG=dict(committed='707d9726da9f6a6f7dedc688f4437777aeb2e8fa5cf0958ca6aed83f8f7bfa44',
    geometry_identity='a52676253aa7996b6ac4d453f40ae838aa9d569d883b8fa64fe6799cd22cbc65',
    surface_identity='14770350129d217662b9e936b2b2bd39d9ea3e14b47ad7fedc9e47425da1aafd',
    request='ec28d584b04afb8bd4679d6719d0f600140b63dcff26a0cc50c2607537d33b91',
    request_file='37375760bf77a7df2ff6e34ca0a87b6d437dcee604a5d8bf8ddb3c3483c40d80',
    schedule='ae2cfd505b0cb52218c4f000688df150e8f2c65f2d2d28de78785f1de39f0597',layer=2,
    front_coordinates='f57dc72c0ee93cb95529539f5986a206ee315ba59ca886eb284408deb86896cd')
INCREMENT_HEX='0x1.b2e30b099036bp-13'


def save(name,value):
    path=OUT/name;path.parent.mkdir(parents=True,exist_ok=True);durable_json(path,value)


def resources():
    m=psutil.Process().memory_info()
    return dict(peak_rss=m.peak_wset,retained_rss=m.rss)


def seed(source,destination):
    """Add aliases of verified write-once objects; never share mutable pointers."""
    objects=destination/'objects';objects.mkdir(parents=True,exist_ok=True);rows=[]
    for path in sorted((source/'objects').iterdir()):
        if path.suffix not in ('.json','.sqlite','.npy','.npz'):raise ValueError('UnknownSeedObject')
        target=objects/path.name;original=metadata(path,content=True)
        if not target.exists():os.link(path,target)
        actual=metadata(target)
        if physical_key(original)!=physical_key(actual):raise ValueError('SeedAliasMismatch')
        rows.append(dict(source=str(path),target=str(target),sha256=original['sha256'],
            file_id=original['file_id'],bytes=original['bytes'],cluster_bytes=original['cluster_bytes']))
    return rows


def prepare_base():
    destination=OUT/'BASE/checkpoints'
    if (destination/'CURRENT.json').exists():return
    rows=seed(BASE,destination)
    pointer=json.loads((BASE/'CURRENT.json').read_text())
    assert pointer['scientific']==CONFIG['committed']
    assert file_hash(BASE/pointer['checkpoint'])==pointer['sha256']=='644a5966f33072f1bde40fdc336eb18bf4955cb126791589cc1d48c1fa986388'
    shutil.copyfile(BASE/pointer['checkpoint'],destination/pointer['checkpoint'])
    shutil.copyfile(BASE/'CURRENT.json',destination/'CURRENT.json')
    save('BASE_SHARING.json',rows)


def restore_base():
    start=time.perf_counter();print('restore committed prefix',flush=True)
    state=NativeCheckpointStore(OUT/'BASE/checkpoints').restart()
    wall,tri,surface=load_production_wall(SURFACE)
    definition=json.loads((SURFACE.parent/'GEOMETRY.json').read_text())['definition']
    from production_adaptive_surface import CapsuleGeometry
    assert CapsuleGeometry(definition).geometry_identity==CONFIG['geometry_identity']
    spec=dict(components=[definition])
    adapter=ProductionForecastAdapter(state,wall,tri,surface,REQUEST,Source(spec),CONFIG)
    assert float(adapter.remaining_increments[0]).hex()==INCREMENT_HEX
    assert adapter.front.committed_layer==CONFIG['layer']
    print('restored',scientific_hash(state),time.perf_counter()-start,flush=True)
    return state,adapter,spec,time.perf_counter()-start


def admit():
    prepare_base();started=time.perf_counter();state,adapter,spec,restore_seconds=restore_base()
    next_layer=adapter.binding['next_layer'];del state;gc.collect()
    forecast=BoundProductionForecastCache(adapter).forecast(adapter)
    assert forecast.layers.tolist()==list(range(next_layer,len(adapter.schedule.increments)+1))
    high=forecast.vertex['coordinates'][0]
    arrays={g+'__'+k:v for g in ('vertex','edge','triangle') for k,v in getattr(forecast,g).items()}
    np.savez_compressed(OUT/'FORECAST.npz',**arrays)
    forecast_record=dict(binding=adapter.binding,layers=forecast.layers.tolist(),identity=forecast.scientific_hash(),
        hard_events=forecast.hard_events(),arrays={k:array_hash(v) for k,v in arrays.items()},
        remaining_increments=[dict(layer=next_layer+i,value=float(v),hex=float(v).hex()) for i,v in enumerate(adapter.remaining_increments)],
        curvature='Legacy source curvature placeholder suppressed; not measured by cheap forecast')
    save('FORECAST.json',forecast_record);print('forecast bound layers',forecast.layers.tolist(),flush=True)
    screen=direct_screen(adapter.topology,adapter.committed_prefix,high,layer=next_layer)
    save('DIRECT.json',dict(counts=screen['counts'],events=screen['events'],topology=screen['topology'],
        exact_record_hash=screen['exact_record_hash'],distributions={g:{k:distributions(v) for k,v in screen[g].items() if v.ndim==1} for g in ('cells','sides','caps')},seconds=screen['seconds']))
    np.savez_compressed(OUT/'SCREEN_ARRAYS.npz',provisional_high=high,**{g+'__'+k:v for g in ('cells','sides','caps') for k,v in screen[g].items()})
    with (OUT/'EXACT_WEDGES.jsonl').open('w',encoding='utf8',newline='\n') as f:
        for row in screen['exact_records']:f.write(json.dumps(row,sort_keys=True,separators=(',',':'))+'\n')
    inv=prefix_face_inventory(adapter.topology,adapter.committed_prefix,high)
    broad=collision_screen(inv);print('broad phase',broad['populations'],flush=True)
    points=np.concatenate((*adapter.committed_prefix,high))
    assert np.array_equal(points[inv['labels']],inv['coordinates'])
    proof=classify(points,inv['labels'],broad['pairs'],progress=lambda r:print('localization',r['completed'],flush=True))
    codes=proof['codes'];risks,seeds=risk_columns(codes,broad['pairs'],inv['columns'])
    direct_seeds=sorted({c for e in screen['events'] for c in e['columns']})
    seeds=np.union1d(seeds,direct_seeds).astype(np.int64);closure=close_risk(adapter.topology,seeds)
    closure_seconds=closure.pop('seconds')
    accounting=dict(candidates=len(codes),certified=int(((codes>0)&(codes<10)).sum()),unresolved=int((codes==0).sum()),
        forbidden=int((codes==10).sum()),failure=int((codes==11).sum()),missing=0,duplicate=0)
    np.savez_compressed(OUT/'LOCALIZATION.npz',pairs=broad['pairs'],codes=codes,witness=proof['witness'],rotations=proof['rotations'])
    save('LOCALIZATION.json',dict(accounting=accounting,methods={METHODS[int(k)]:int(v) for k,v in zip(*np.unique(codes,return_counts=True))},
        inventory={k:array_hash(v) for k,v in inv.items()},candidate_identity=array_hash(broad['pairs']),codes_identity=array_hash(codes),
        populations=broad['populations'],risk_seeds=seeds.tolist(),direct_seeds=direct_seeds,closure=closure,exact_rows=proof['exact_rows'],
        timings=dict(broad=broad['seconds'],closure=closure_seconds,**proof['timings'])))
    complete=dict.fromkeys(REQUIRED,True)
    complete['direct_metrics']=not screen['events'];complete['risk_supports']=not closure['blocked_groups']
    evidence=dict(binding=adapter.binding,increment_hex=INCREMENT_HEX,completeness=complete,
        cheap_hard_events=forecast.hard_events(),direct_hard_events=screen['events'],collision_accounting=accounting,
        risk_seeds=seeds.tolist(),patches=closure['groups'],mandatory_detail_requirements=[],
        authorities=dict(forecast=forecast.scientific_hash(),screen=file_hash(OUT/'DIRECT.json'),localization=file_hash(OUT/'LOCALIZATION.json')))
    decision=evaluate(evidence,identity(evidence),adapter.binding,adapter.front.coordinates,adapter.topology,high,schedule=adapter.schedule)
    save('ADMISSION.json',dict(evidence=evidence,evidence_identity=identity(evidence),decision=decision,
        decision_identity=identity(decision),restore_seconds=restore_seconds,total_seconds=time.perf_counter()-started,**resources()))
    print('ADMISSION',decision,flush=True)
    if not decision['generation_attempt_authorized']:raise ValueError('ReadOnlyAdmissionBlocked')


class MeasuredStore(NativeCheckpointStore):
    def __init__(self,path,certificate):
        super().__init__(path);self.certificate=certificate;self.materialized=[];self.receipt_seconds=0.
    def _table(self,value,content_only=False):
        start=time.perf_counter();result=super()._table(value,content_only=content_only)
        if value is self.certificate:self.receipt_seconds=time.perf_counter()-start
        return result
    def _flat_table(self,value):
        result=super()._flat_table(value)
        if isinstance(value,ReceiptOverlay):self.materialized.append(dict(rows=len(value),file=result['file'],bytes=(self.objects/result['file']).stat().st_size))
        return result


def generate(label):
    start=time.perf_counter();state,adapter,spec,restore_seconds=restore_base()
    admission=json.loads((OUT/'ADMISSION.json').read_text());e=admission['evidence']
    assert e['binding']==adapter.binding and identity(e)==admission['evidence_identity']
    assert identity(admission['decision'])==admission['decision_identity'] and admission['decision']['generation_attempt_authorized']
    for name,key in [('DIRECT.json','screen'),('LOCALIZATION.json','localization')]:assert file_hash(OUT/name)==e['authorities'][key]
    folder=OUT/label
    if label=='B':save('REPLAY_SHARING.json',seed(OUT/'A/checkpoints',folder/'checkpoints'))
    provider=ProductionNextLayerProvider(folder,state,adapter,spec)
    runner=MixedFrontRunner(folder/'checkpoints',provider,profile='ProductionFast');runner.store=InMemoryMixedFrontJournal(state)
    try:
        transaction=provider.phase('production_transaction',runner.step)
        committed=runner.store.restart();layer=adapter.binding['next_layer']
        assert preserves_committed_cells(committed,state) and committed['wall']==state['wall'] and committed['first_layer']==state['first_layer']
        assert {c['last_prism_layer'] for c in committed['columns'].values()}=={layer} and provider.generated==1
        cert=committed['certificate'];unedited=(type(cert) is ReceiptOverlay and not cert.edits and not cert.removed and type(cert.base) is ReceiptDelta)
        base=cert.base if unedited else cert
        receipt=dict(wrapper=type(cert).__name__,base=type(base).__name__,unedited_delta=unedited,
            total=len(cert),base_count=len(base.base) if isinstance(base,ReceiptDelta) else None,
            delta_count=len(base.updates) if isinstance(base,ReceiptDelta) else None)
        ready=dict(passed=True,committed=transaction['state'],transaction=transaction,coordinates=array_hash(provider.fronts[-1]),
            topology=array_hash(provider.tri),new_connectivity=array_hash(np.array([c['vertices'] for c in provider.cells if c['layer']==layer],np.int64)),
            combined_connectivity=array_hash(np.array([c['vertices'] for c in provider.cells],np.int64)),
            ownership=identity([(c['id'],c['column']) for c in provider.cells]),quality=file_hash(folder/'QUALITY.json'),
            incidence=file_hash(folder/'INCIDENCE.json'),certificate=provider.check['certificate']['proof_hash'],generated=provider.generated)
        save(label+'/READY.json',ready);save(label+'/RECEIPT_PLAN.json',receipt)
        save(label+'/PHASES.json',dict(times=provider.times,restore_seconds=restore_seconds,**resources()))
        print('READY_FOR_REPLAY',label,identity(ready),flush=True)
        barrier=OUT/'REPLAY_AUTHORIZATION.json'
        while not barrier.exists():time.sleep(1)
        assert json.loads(barrier.read_text())['scientific_identity']==identity(ready)
        assert json.loads((OUT/'A/READY.json').read_text())==json.loads((OUT/'B/READY.json').read_text())==ready
        if label=='B':
            # Replay equality is established before either publisher runs.
            # Publish A first, then share its immutable output objects so B's
            # ordinary writer can verify/reuse them without a second retained
            # physical checkpoint population. Mutable CURRENT remains private.
            while not (OUT/'A/FINAL.json').exists():time.sleep(1)
            save('REPLAY_PUBLICATION_SHARING.json',seed(OUT/'A/checkpoints',folder/'checkpoints'))
        store=MeasuredStore(folder/'checkpoints',cert)
        published=provider.phase('checkpoint',lambda:store.publish(committed))
        assert published==transaction['state']
        pointer=json.loads((folder/'checkpoints/CURRENT.json').read_text())
        envelope=json.loads((folder/'checkpoints'/pointer['checkpoint']).read_text())
        save(label+'/FINAL.json',dict(**ready,pointer=pointer,certificate_descriptor=envelope['components']['certificate'],receipt=receipt,
            full_receipt_flattening=envelope['components']['certificate']['encoding']!='receipt_delta',materialized=store.materialized,
            receipt_publication_seconds=store.receipt_seconds,times=provider.times,restore_seconds=restore_seconds,total_seconds=time.perf_counter()-start,
            scope=dict(committed_layer=layer,L4_generated=False,post_L3_L4_forecast=False,core_generated=False),**resources()))
        print('COMMITTED',label,published,flush=True)
    except Exception as exc:
        save(label+'/FAILURE.json',dict(error=type(exc).__name__+':'+str(exc),times=provider.times,generated=provider.generated,
            authoritative_layer=CONFIG['layer'],**resources()))
        raise


def authorize():
    a=json.loads((OUT/'A/READY.json').read_text());b=json.loads((OUT/'B/READY.json').read_text())
    assert a==b and a['passed'] and a['generated']==1
    save('REPLAY_AUTHORIZATION.json',dict(scientific_identity=identity(a),source='Two independently restored complete production transactions'))


def restore_final(label):
    start=time.perf_counter();state=NativeCheckpointStore(OUT/label/'checkpoints').restart()
    ready=json.loads((OUT/label/'READY.json').read_text());actual=scientific_hash(state)
    assert actual==ready['committed'] and {c['last_prism_layer'] for c in state['columns'].values()}=={CONFIG['layer']+1}
    coords=[array_hash(np.array([state['vertices'][f'layer:{k}:source:{i}'] for i in range(16895)])) for k in range(CONFIG['layer']+2)]
    assert coords[-1]==ready['coordinates']
    assert coords[:3]==['2270c352c34bba85dc898da4ea66dda1484123604d96511551899802d16d157c','34ca5df3c122a7436caf8a475c1c8cf0fbe629a6afb26fe1387862ef2f884cf7',CONFIG['front_coordinates']]
    save(label+'/RESTORE.json',dict(passed=True,scientific=actual,coordinates=coords,cells=len(state['cells']),vertices=len(state['vertices']),
        receipts=len(state['certificate']),seconds=time.perf_counter()-start,**resources()))
    print('DURABLE_RESTORE',label,actual,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['admit','generate','authorize','restore']);p.add_argument('--label',choices=['A','B'],default='A');a=p.parse_args()
    if a.mode=='generate':generate(a.label)
    elif a.mode=='restore':restore_final(a.label)
    else:globals()[a.mode]()
