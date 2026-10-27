"""Apply qualified successful-only retention to an independently sealed layer."""
import argparse,collections,shutil,time
from production_bounded_layers import *
from successful_retention_cleanup import prepare,purge
from path_preserving_deduplication import probe_file_id

def local_inventory(folder):
    rows=[];hashes={}
    for p in sorted(folder.rglob('*')):
        if not p.is_file() or p.suffix=='.log' or 'recovery' in p.parts:continue
        r=metadata(p);key=physical_key(r)
        if key not in hashes:hashes[key]=file_hash(p)
        r['sha256']=hashes[key];rows.append(r)
    return rows

def plan(layer):
    folder=configure(layer);start=time.perf_counter();a=read(folder/'A/READY.json');b=read(folder/'B/READY.json')
    assert a==b and a['passed']
    for label in ('A','B'):
        r=read(folder/label/'RESTORE.json');assert r['passed'] and r['scientific']==a['committed']
    d=read(folder/'A/DIAGNOSTICS.json');assert all(not x['failures'] for scope in d['quality'].values() for x in scope.values())
    assert read(folder/'REGENERATION.json')['passed'] and read(folder/'RECEIPT_AUDIT.json')['proof_payloads_unchanged']
    rows=local_inventory(folder);index={r['path']:r for r in rows};closures=set()
    for label in ('A','B','BASE'):closures.update(checkpoint_closure(folder/label/'checkpoints')[0])
    actions=[];classified=[]
    for original in rows:
        r=dict(original);p=Path(r['path']);name=p.relative_to(folder).as_posix()
        active=r['path'] in closures or name in ('A/QUALITY.json','A/CANDIDATE.npz')
        r.update(restore_dependency=r['path'] in closures,next_layer_dependency=active,failure_evidence=False)
        if active:kind='B';reason='Transitive restart/recovery closure or required next-layer diagnostic cache.'
        elif name in ('A/INCIDENCE.json','A/VOLUME.json'):
            proof=read(folder/'REGENERATION.json')['exports'][p.name];assert proof['sha256']==r['sha256']
            kind='C';reason='Complete byte-exact regeneration from sealed realized prefix.'
            r['removal_proof']=dict(kind='complete_regeneration',sha256=r['sha256'],record='REGENERATION.json')
        elif name=='A/RAW_QUALITY.json' or name.startswith('B/') and 'checkpoints' not in p.parts:
            canon=folder/'A'/p.relative_to(folder/'B') if name.startswith('B/') else folder/'A/QUALITY.json'
            if p.name=='RAW_QUALITY.json':canon=folder/'A/QUALITY.json'
            other=index.get(str(canon.resolve()))
            # Only the same successful export kinds qualified at L3.
            allowed=p.name in ('RAW_QUALITY.json','QUALITY.json','INCIDENCE.json','VOLUME.json','CANDIDATE.npz') or 'RECEIPT_RETIREMENTS' in p.parts
            if allowed and other and other['sha256']==r['sha256'] and other['bytes']==r['bytes']:
                kind='E';reason='Exact successful export duplicate.'
                if p.name in ('INCIDENCE.json','VOLUME.json'):
                    r['removal_proof']=dict(kind='complete_regeneration',sha256=r['sha256'],record='REGENERATION.json')
                else:r['removal_proof']=dict(kind='exact_retained_duplicate',sha256=r['sha256'],canonical=str(canon))
            elif allowed:kind='D';reason='Nonidentical or unproven replay export preserved.';r['failure_evidence']=True
            else:kind='A';reason='Compact replay authority.'
        elif 'RECEIPT_RETIREMENTS' in p.parts or 'FAILURE' in p.name:kind='D';reason='Unique transaction/failure evidence retained.';r['failure_evidence']=True
        elif p.suffix in ('.npz','.jsonl') or 'objects' in p.parts:kind='F';reason='No qualified removal route; preserve.'
        else:kind='A';reason='Compact scientific or qualification authority.'
        r.update(classification=kind,retention_reason=reason,regeneration_source=r.get('removal_proof'),
            references=[dict(source='sealed layer READY and native checkpoint closure',sha256=r['sha256'])])
        classified.append(r)
        if kind in ('C','E'):actions.append(r)
    save(folder/'RETENTION_INVENTORY.json',dict(rows=classified,totals=totals(rows),seconds=time.perf_counter()-start,**cycle.resources()))
    _,envelope=checkpoint_closure(folder/'A/checkpoints')
    certificate=read(folder/'A/CERTIFICATE.json')
    compact=dict(policy='V4RollingSuccessfulQualificationRetention/1',parent_qualification='9cf1240f147a4cd026c351fcde081513c79fef2c93cde95540079c0cb35df97f',
        layer=layer,passed=True,inputs=cycle.CONFIG,increment_hex=INCREMENTS[layer],result=a,
        admission=read(folder/'ADMISSION.json'),quality=d['quality'],determinant=d['determinant'],topology=d['topology'],
        continuous=certificate,proof_population=envelope['components']['certificate'],quality_authority=envelope['components']['quality'],
        final_gates=read(folder/'FINAL_GATE_SUMMARY.json'),diagnostics_sha256=file_hash(folder/'A/DIAGNOSTICS.json'),
        active=[dict(path=r['path'],sha256=r['sha256']) for r in classified if r['classification']=='B'],
        discarded=[dict(path=r['path'],sha256=r['sha256'],bytes=r['bytes'],proof=r['removal_proof']) for r in actions],
        implementation={str(p.relative_to(V4)):file_hash(p) for p in [Path(__file__),V4/'tools/production_bounded_layers.py',V4/'tools/bounded_layer_evidence.py',V4/'tools/successful_retention_cleanup.py']},
        reproduction='Retained source fronts are bound by fresh RESTORE coordinate hashes. bounded_layer_evidence.regenerate rebuilds only existing incidence/volume, without any predictor. Duplicate exports use the retained named canonical. Current QUALITY and CANDIDATE remain active caches.',
        permanent_reports=['ADMISSION.json','FINAL_GATE_SUMMARY.json','A/DIAGNOSTICS.json','A/CERTIFICATE.json','A/READY.json','A/FINAL.json','A/RESTORE.json','B/READY.json','B/FINAL.json','B/RESTORE.json','RECEIPT_AUDIT.json','REGENERATION.json'])
    compact['identity']=identity(compact);save(folder/'COMPACT.json',compact)
    save(folder/'CLEANUP_PLAN.json',dict(successful=True,compact_sha256=file_hash(folder/'COMPACT.json'),inventory_sha256=file_hash(folder/'RETENTION_INVENTORY.json'),actions=actions))
    print('RETENTION_PLAN',layer,len(actions),sum(r['cluster_bytes'] for r in {physical_key(r):r for r in actions}.values()),flush=True)

def apply(layer):
    folder=configure(layer);start=time.perf_counter();p=read(folder/'CLEANUP_PLAN.json')
    assert p['compact_sha256']==file_hash(folder/'COMPACT.json') and p['inventory_sha256']==file_hash(folder/'RETENTION_INVENTORY.json')
    records=prepare(p,folder,folder/'recovery');save(folder/'REMOVALS_PENDING.json',dict(records=records,seconds=time.perf_counter()-start,**cycle.resources()))

def historical(layer,label):
    folder=configure(layer);start=time.perf_counter()
    roots={1:V4/'results/production_adaptive_capsule_first_layer',2:V4/'results/production_adaptive_capsule_layer2',3:L3}
    expected={1:'5b036bc4cd5180b9e4a7e3e6aafe57fe21c7b2ae1cae82cf3c42e446b1805fd0',2:'707d9726da9f6a6f7dedc688f4437777aeb2e8fa5cf0958ca6aed83f8f7bfa44',3:INITIAL}
    state=NativeCheckpointStore(roots[label]/'A/checkpoints').restart();actual=scientific_hash(state)
    assert actual==expected[label]
    save(folder/f'HISTORICAL_L{label}.json',dict(passed=True,scientific=actual,seconds=time.perf_counter()-start,**cycle.resources()))

def finish(layer):
    folder=configure(layer);start=time.perf_counter();p=read(folder/'CLEANUP_PLAN.json');removed={r['path'] for r in p['actions']}
    for label in ('A','B'):
        before=read(folder/label/'RESTORE.json');after=read(folder/label/'POST_RETENTION.json')
        assert after['passed'] and after['scientific']==before['scientific'] and after['readiness']==before['readiness']
    for k in (1,2,3):assert read(folder/f'HISTORICAL_L{k}.json')['passed']
    for r in read(folder/'RETENTION_INVENTORY.json')['rows']:
        if r['path'] not in removed:assert file_hash(r['path'])==r['sha256'],r['path']
    verification=dict(L1_restore=True,L2_restore=True,L3_restore=True,L4_binding_only=True,protected_hashes=True,regeneration=True,
        actual_committed_layer=layer,actual_next_binding_layer=layer+1,no_next_forecast=True)
    control=folder/'recovery/retired_id_control'
    assert not control.exists()
    control.write_bytes(b'qualified deleted-file identity control')
    control_id=metadata(control)['file_id']
    assert probe_file_id(control_id)==0
    control.unlink()
    deleted_error=probe_file_id(control_id)
    assert deleted_error!=0
    records=purge(folder/'recovery',verification)
    reclaimed=0
    for r in {physical_key(r):r for r in p['actions']}.values():
        if probe_file_id(r['file_id'])!=deleted_error:raise ValueError('RemovedPayloadRetirementUnverified')
        reclaimed+=r['cluster_bytes']
    save(folder/'RETENTION_RESULT.json',dict(passed=True,policy='V4RollingSuccessfulQualificationRetention/1',removed=len(records),records=records,
        physical_reclaimed=reclaimed,deleted_file_control_error=deleted_error,logical_removed=sum(r['bytes'] for r in p['actions']),verification=verification,
        seconds=time.perf_counter()-start,**cycle.resources()))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['plan','apply','historical','finish']);p.add_argument('--layer',type=int,required=True);p.add_argument('--historical-layer',type=int);a=p.parse_args()
    if a.mode=='historical':historical(a.layer,a.historical_layer)
    else:globals()[a.mode](a.layer)
