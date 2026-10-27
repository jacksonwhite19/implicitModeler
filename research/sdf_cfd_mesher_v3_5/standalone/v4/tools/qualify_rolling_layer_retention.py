"""Seal and execute the L3 successful-retention experiment; no meshing."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import shutil
import time
from rolling_layer_retention import (L3,OUT,ROOT,V4,TREES,COMMITTED,POLICY,read,save,
    metadata,physical_key,file_hash,resources,totals,checkpoint_closure)
from sdf_cfd_mesher_v4.common.authority import identity
from successful_retention_cleanup import prepare,purge,recover

EXPECTED={'report.md':'3c8aabf822c1538e69642a5373f75dc3d1949fc3137fc48fb1e5d36cce4dc7a6',
    'AUTHORITY.json':'b14ee357feda1394980a48902c61f90e83a428ec8ecaad085d4069b6bc037fbe'}

def protected(discard=()):
    discarded={r['path']:r for r in discard};checks=[]
    for name,h in EXPECTED.items():
        if file_hash(L3/name)!=h:raise ValueError('L3AuthorityChanged')
    for r in read(L3/'INPUT_SEALS.json'):
        if file_hash(r['path'])!=r['sha256']:raise ValueError('InputSealChanged:'+r['path'])
        checks.append(r)
    for tree in [*TREES,L3]:
        for name,h in read(tree/'AUTHORITY.json').get('evidence',{}).items():
            p=str((tree/name).resolve())
            if p in discarded:
                if discarded[p]['sha256']!=h:raise ValueError('DiscardIdentityMismatch')
                continue
            if file_hash(p)!=h:raise ValueError('ProtectedEvidenceChanged:'+p)
            checks.append(dict(path=p,sha256=h))
    return checks

def classify():
    start=time.perf_counter();protected();inv=read(OUT/'INVENTORY.json');regen=read(OUT/'REGENERATION.json')
    if not regen['passed'] or read(OUT/'PRE_L3_A.json')['scientific']!=COMMITTED:raise ValueError('PreconditionsMissing')
    authority=read(L3/'AUTHORITY.json')
    if authority['scientific_identity']!='d3017e1eecb1cb55d63e1612b10c0cfb4e26a05893fcc3dcdc4b85da390d3737':raise ValueError('QualificationMismatch')
    if authority['blockers'] or not authority['scientific']['result']['passed']:raise ValueError('UnsuccessfulLayer')
    if identity(authority['scientific'])!=authority['scientific_identity']:raise ValueError('ScientificAuthorityMismatch')
    if read(L3/'A/READY.json')!=read(L3/'B/READY.json'):raise ValueError('NondeterministicLayer')
    for name,h in authority['scientific']['implementation'].items():
        if file_hash(V4/name)!=h:raise ValueError('ChangedQualifiedImplementation:'+name)
    rows=[];actions=[]
    regen_names={'A/INCIDENCE.json','A/VOLUME.json'}
    dup_names={'A/RAW_QUALITY.json':'A/QUALITY.json','B/RAW_QUALITY.json':'A/QUALITY.json',
        'B/QUALITY.json':'A/QUALITY.json','B/INCIDENCE.json':'A/INCIDENCE.json','B/VOLUME.json':'A/VOLUME.json',
        'B/CANDIDATE.npz':'A/CANDIDATE.npz'}
    all_rows={r['path']:r for r in inv['rows']}
    for original in inv['rows']:
        p=Path(original['path'])
        if not p.is_relative_to(L3):continue
        r=dict(original);name=p.relative_to(L3).as_posix()
        historical=read(L3/'BULK_MANIFEST.json').get(r['path'])
        if historical:
            if historical['sha256']!=r['sha256']:raise ValueError('HistoricalBulkMismatch')
            r['references']=[*r['references'],dict(source=str(L3/'BULK_MANIFEST.json'),kind='historical_bulk',sha256=r['sha256'])]
        r['regeneration_source']=None;r['failure_evidence']=False
        if name=='A/QUALITY.json':
            r.update(classification='B',reason='Existing next-layer diagnostics read prior_quality cells for completed-neighbour determinant comparison.',
                next_layer_dependency=True,regeneration_source='Complete regeneration proven, but retained as required rolling diagnostic cache.')
        elif r['restore_dependency']:
            r['classification']='B';r['reason']='Retained current/replay/historical checkpoint closure. Never discard by CURRENT alone.'
        elif name in regen_names:
            proof=regen['complete_exports'][p.name]
            if proof['sha256']!=r['sha256'] or not proof['exact']:raise ValueError('RegenerationNotExact')
            r.update(classification='C',reason='Complete byte-exact regeneration from sealed L3 coordinates and unchanged validators.',
                regeneration_source='rolling_layer_retention.py regenerate; REGENERATION.json',
                removal_proof=dict(kind='complete_regeneration',sha256=r['sha256'],record='REGENERATION.json'))
        elif name in dup_names:
            canonical=all_rows[str((L3/dup_names[name]).resolve())]
            if canonical['sha256']!=r['sha256'] or canonical['bytes']!=r['bytes']:raise ValueError('DuplicateNotExact')
            # If the canonical export is itself regenerated, each alias is
            # independently covered by the complete regeneration proof.
            if dup_names[name] in regen_names:
                proof=dict(kind='complete_regeneration',sha256=r['sha256'],record='REGENERATION.json')
            else:proof=dict(kind='exact_retained_duplicate',sha256=r['sha256'],canonical=canonical['path'])
            r.update(classification='E',reason='Exact successful export duplicate.',removal_proof=proof,regeneration_source=dup_names[name])
        elif name.startswith('B/RECEIPT_RETIREMENTS/'):
            canonical=all_rows[str((L3/'A'/Path(name).relative_to('B')).resolve())]
            if canonical['sha256']!=r['sha256']:raise ValueError('RetirementDuplicateMismatch')
            r.update(classification='E',reason='Exact duplicate retirement archive; A archive retained.',
                regeneration_source=canonical['path'],removal_proof=dict(kind='exact_retained_duplicate',sha256=r['sha256'],canonical=canonical['path']))
        elif 'RECEIPT_RETIREMENTS' in p.parts or p.suffix=='.log':
            r.update(classification='D',reason='Conservative transaction/debug retention; no failure cleanup.',failure_evidence=True)
        elif p.suffix in ('.npz','.jsonl') or 'checkpoints' in p.parts:
            r.update(classification='F',reason='Preserved; no complete regeneration qualification for this forecast/diagnostic or unreferenced historical object.')
        else:r.update(classification='A',reason='Compact original scientific, input, replay, report, test, or storage authority record.')
        if r['classification'] in ('C','E'):actions.append(r)
        rows.append(r)
    # One physical allocation can have many roles. Active closure takes
    # precedence; historical shared allocation is also reported separately.
    rank={c:i for i,c in enumerate('BADFCE')};groups=defaultdict(list)
    old={physical_key(r) for r in inv['rows'] if not Path(r['path']).is_relative_to(L3)}
    for r in rows:groups[physical_key(r)].append(r)
    breakdown={c:dict(paths=0,logical_bytes=0,physical_bytes=0,new_l3_physical_bytes=0) for c in 'ABCDEF'}
    for r in rows:breakdown[r['classification']]['paths']+=1;breakdown[r['classification']]['logical_bytes']+=r['bytes']
    for key,group in groups.items():
        c=min(group,key=lambda r:rank[r['classification']])['classification'];n=group[0]['cluster_bytes']
        breakdown[c]['physical_bytes']+=n
        if key not in old:breakdown[c]['new_l3_physical_bytes']+=n
    classification=dict(policy=POLICY,rows=rows,breakdown=breakdown,
        active_obsolescence='After a later successful commit and durable restore, only objects absent from ALL retained checkpoint closures and recovery references are candidates. ReceiptDelta bases remain transitive dependencies. No future-layer obsolescence is assumed.',
        unresolved=[r['path'] for r in rows if r['classification']=='F'])
    save('CLASSIFICATION.json',classification)
    _,env=checkpoint_closure(L3/'A/checkpoints')
    d=read(L3/'A/DIAGNOSTICS.json')
    compact=dict(policy=POLICY,successful=True,qualification=authority['scientific_identity'],committed=COMMITTED,
        original_authority=EXPECTED,inputs=authority['scientific']['inputs'],result=authority['scientific']['result'],
        gates=d['quality'],completed_neighbour=d['determinant'],topology=d['topology'],
        exact_gate_summary=read(L3/'FINAL_GATE_SUMMARY.json'),continuous=read(L3/'A/CERTIFICATE.json'),
        complete_receipt_population=env['components']['certificate'],quality_authority=env['components']['quality'],
        permanent_objects=[dict(path=r['path'],sha256=r['sha256'],bytes=r['bytes']) for r in rows if r['classification']=='A'],
        active_prefix_objects=[dict(path=r['path'],sha256=r['sha256'],bytes=r['bytes']) for r in rows if r['classification']=='B'],
        discarded_objects=[dict(path=r['path'],sha256=r['sha256'],bytes=r['bytes'],proof=r['removal_proof']) for r in actions],
        implementation=authority['scientific']['implementation'],source_head=read(OUT/'BASELINE.json')['head'],
        retention_implementation={str(p.relative_to(V4)):file_hash(p) for p in [Path(__file__),V4/'tools/rolling_layer_retention.py',V4/'tools/successful_retention_cleanup.py']},
        reproduction='Set PYTHONPATH to V4/src and V4/tools. Run rolling_layer_retention.py regenerate --record REGENERATION_REPLAY.json --export regenerated against retained L3/A checkpoint and INVENTORY.json. Complete export SHA-256 must equal discarded hashes. No predictor is used. The exported QUALITY.json also restores RAW_QUALITY.json aliases. Exact duplicate candidates/retirements copy from named retained canonical.',
        historical_reference_semantics='Historical authority bytes and content identities remain unchanged. Only listed successful export paths may be absent. This compact record binds their exact regeneration or retained duplicate. Native restore paths are never discarded.',
        future_production='Not implemented. Future production may retain current/final mesh, recovery state, compact certificate, minimal operational audit, and exception bulk without routine A/B replay. Qualification requirements are unchanged.')
    compact['identity']=identity(compact);save('COMPACT_L3.json',compact)
    plan=dict(policy=POLICY,successful=True,committed=COMMITTED,actions=actions,
        compact_sha256=file_hash(OUT/'COMPACT_L3.json'),inventory_sha256=file_hash(OUT/'INVENTORY.json'),
        classification_sha256=file_hash(OUT/'CLASSIFICATION.json'),regeneration_sha256=file_hash(OUT/'REGENERATION.json'))
    save('CLEANUP_PLAN.json',plan);save('PRE_SEALS.json',dict(protected=protected(),seconds=time.perf_counter()-start,**resources()))
    print(json.dumps(dict(breakdown=breakdown,actions=len(actions),logical_removal=sum(r['bytes'] for r in actions),
        physical_reclaimable=sum(r['cluster_bytes'] for r in {physical_key(r):r for r in actions}.values()),compact=compact['identity']),indent=2),flush=True)

def apply():
    start=time.perf_counter();plan=read(OUT/'CLEANUP_PLAN.json')
    for file,key in [('COMPACT_L3.json','compact_sha256'),('INVENTORY.json','inventory_sha256'),('CLASSIFICATION.json','classification_sha256'),('REGENERATION.json','regeneration_sha256')]:
        if file_hash(OUT/file)!=plan[key]:raise ValueError('ChangedPlanSeal')
    protected();result=prepare(plan,L3,OUT/'recovery')
    save('REMOVALS_PENDING.json',dict(records=result,seconds=time.perf_counter()-start,**resources()))

def verify():
    start=time.perf_counter();plan=read(OUT/'CLEANUP_PLAN.json');checks=protected(plan['actions'])
    for label in ('L1_A','L2_A','L3_A','L3_B'):
        r=read(OUT/('POST_'+label+'.json'))
        if not r['passed']:raise ValueError('RestoreFailed')
    pre=read(OUT/'PRE_L3_A.json');post=read(OUT/'POST_L3_A.json')
    if pre['readiness']!=post['readiness']:raise ValueError('ReadinessChanged')
    if post['scientific']!=COMMITTED:raise ValueError('L3Changed')
    removed={r['path'] for r in plan['actions']}
    for r in read(OUT/'INVENTORY.json')['rows']:
        if r['path'] in removed:continue
        if file_hash(r['path'])!=r['sha256']:raise ValueError('RetainedPayloadChanged:'+r['path'])
    save('POST_VERIFICATION.json',dict(L1_restore=True,L2_restore=True,L3_restore=True,L4_binding_only=True,
        protected_hashes=True,regeneration=read(OUT/'REGENERATION.json')['passed'],protected=checks,
        all_retained_payloads_unchanged=True,seconds=time.perf_counter()-start,**resources()))

def finish():
    start=time.perf_counter();records=purge(OUT/'recovery',read(OUT/'POST_VERIFICATION.json'))
    save('CLEANUP_MANIFEST.json',dict(records=records,seconds=time.perf_counter()-start,**resources()))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['classify','apply','verify','finish','recover']);a=p.parse_args()
    if a.mode=='recover':recover(OUT/'recovery')
    else:globals()[a.mode]()
