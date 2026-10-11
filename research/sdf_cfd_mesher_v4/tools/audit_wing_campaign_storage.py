"""Targeted retained-file census. Never generates or publishes a front."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import subprocess
import time

from path_preserving_deduplication import metadata, physical_key, file_hash, durable_json
from record_receipt_resume import disk, status

ROOT = Path(__file__).resolve().parents[3]
V4 = ROOT/'research/sdf_cfd_mesher_v4'
OUT = V4/'results/wing_storage_connection'
CAMPAIGN = V4/'results/wing_receipt_resume/A'


def read(p):
    return json.loads(Path(p).read_text(encoding='utf-8'))


def baseline():
    old = read(V4/'results/wing_receipt_resume/FINAL.json')
    current = status()
    # New task files may already exist; compare baseline entries case-sensitively.
    if [s for s in current if s in old['unrelated_status']] != old['unrelated_status']:
        raise ValueError('UnrelatedStatusChanged')
    files = []
    for r in old['files']:
        p = ROOT/r['path']
        now = dict(path=r['path'], bytes=p.stat().st_size, sha256=file_hash(p))
        if now != r:
            raise ValueError('ProtectedFileChanged', r['path'])
        files.append(now)
    paths = [V4/'results/wing_receipt_resume'/n for n in ('EVIDENCE.json','SUMMARY.json','MONITOR_STOP.json')]
    files += [dict(path=str(p.relative_to(ROOT)),bytes=p.stat().st_size,sha256=file_hash(p)) for p in paths]
    OUT.mkdir(exist_ok=True)
    if (OUT/'BASELINE.json').exists():
        raise ValueError('PreserveBaseline')
    durable_json(OUT/'BASELINE.json', dict(head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        branch=subprocess.check_output(['git','branch','--show-current'],text=True).strip(),
        unrelated_status=old['unrelated_status'],files=files,disk=disk(),time=time.time()))


def stage(p, root):
    parts=p.relative_to(root).parts
    name=p.name
    if 'candidate' in parts or 'new_incidence' in parts or name.startswith(('PUBLIC','REOPEN','REFERENCE','INCIDENCE_MANIFEST','SQL_')):
        return 'publication'
    if name.startswith(('QUALITY_DELTA','NEW_ATLAS','NEW_FORECAST','NEW_RECEIPT','BINDING_PATCH','ASSEMBLY','PRIOR_DETERMINANT','FIRST_LAYER_RECEIPT','CANDIDATE','PREPARE')):
        return 'assembly_prepare'
    if name.startswith(('ADMISSION','DIRECT','FORECAST','LOCALIZATION','SCREEN','TARGET','CONNECTION')):
        return 'admission'
    if 'objects' in parts or 'numeric_delta' in parts or 'numeric_lookup' in parts or name.startswith(('GENERAT','NEW_QUALITY','AFFECTED','GEOMETRY','CHANGED','VOLUME','EXACT','FRESH','PERSISTENT','DEGREE','PROOF_INDEX','INDEX_PIPELINE')):
        return 'generation_proofs_indices'
    return 'controls_or_evidence'


def totals(rows):
    unique={physical_key(r):r for r in rows}
    return dict(files=len(rows),logical=sum(r['bytes'] for r in rows),
        allocation=sum(r['cluster_bytes'] for r in unique.values()),
        file_ids=len(unique),path_allocation=sum(r['cluster_bytes'] for r in rows),
        hardlinked_allocation=sum(r['cluster_bytes'] for r in unique.values() if r['links']>1))


def inventory():
    if (OUT/'INVENTORY.json').exists():raise ValueError('PreserveInventory')
    OUT.mkdir(parents=True,exist_ok=True)
    started=time.perf_counter(); rows=[]; groups=defaultdict(list)
    trees={'wing':CAMPAIGN}
    for n in ('normal_candidate_clean_prism','normal_candidate_clean_replay_prism'):
        trees[n]=V4/'results/remaining_prism_critical_path'/n
    for label, root in trees.items():
        for p in sorted(root.rglob('*')):
            if not p.is_file():continue
            r=metadata(p)
            r['tree']=label;r['relative']=p.relative_to(root).as_posix();r['stage']=stage(p,root)
            r['layer']=next((x for x in p.relative_to(root).parts if x in ('L2','L3','L4','L5')),'control')
            r['realization']='replay' if 'replay' in p.relative_to(root).parts else 'primary'
            rows.append(r);groups[(label,r['layer'],r['realization'],r['stage'])].append(r)
    durable_json(OUT/'INVENTORY.json',dict(rows=rows,totals={k:totals([r for r in rows if r['tree']==k]) for k in trees},
        groups={'/'.join(k):totals(v) for k,v in groups.items()},disk=disk(),seconds=time.perf_counter()-started))
    print(json.dumps(read(OUT/'INVENTORY.json')['totals']))


def dependencies():
    """Hash only active closure, failed L5, named capsule evidence and writers."""
    if (OUT/'DEPENDENCIES.json').exists():raise ValueError('PreserveDependencyRecord')
    from recursive_replay_retention import authenticated_closure
    started=time.perf_counter();rows=[]
    ref=read(CAMPAIGN/'L4/REOPEN.json')['reference']
    with authenticated_closure(ref) as (state,closure,nodes):
        for path,record in sorted(closure.items()):
            r=metadata(path);r['sha256']=file_hash(path)
            if r['sha256']!=record['sha256']:raise ValueError('ChangedActiveAncestry',path)
            rows.append(r)
        if state.scientific!='a68a11559a9301442a19293474b0715c50540bbed0cc4f72105c398fb7c5e374':
            raise ValueError('UnexpectedL4')
    failed=[dict(path=str(p.resolve()),bytes=p.stat().st_size,sha256=file_hash(p)) for p in sorted((CAMPAIGN/'L5').rglob('*')) if p.is_file()]
    capsule=V4/'results/remaining_prism_critical_path'
    source_names=('profile_remaining_prism.py','critical_assembly.py','critical_prism_transition.py','critical_prepare.py',
        'critical_transaction.py','incremental_pair_storage.py','prism_proof_index.py','prism_persistent_hotpath.py',
        'prism_persistent_views.py','optimized_durable_json.py','prism_sealed_file_cache.py',
        'retain_recursive_layer6.py','recursive_production_verification.py','finalize_remaining_control.py',
        'accepted_production_contract.py','composable_state.py','run_authorized_continuation.py')
    sources={}
    commit='28fc3aca116e8e7dd2ca7729e911c1930fd8d4f4'
    import hashlib
    for name in source_names:
        p=V4/'tools'/name;relative=p.relative_to(ROOT).as_posix()
        old=subprocess.run(['git','show',commit+':'+relative],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        sources[relative]=dict(current=file_hash(p),qualified_commit_sha256=hashlib.sha256(old.stdout).hexdigest() if old.returncode==0 else None)
    evidence={str(p.relative_to(ROOT)):file_hash(p) for p in [capsule/n for n in ('AUTHORITY.json','CONTROL_FILE_EVIDENCE.json','STORAGE.json','DETERMINISTIC_REPLAY.json','REPORT.json')]}
    durable_json(OUT/'DEPENDENCIES.json',dict(active=rows,active_totals=totals(rows),nodes=nodes,
        outside_campaign=totals([r for r in rows if not Path(r['path']).is_relative_to(CAMPAIGN)]),
        failed_l5=failed,sources=sources,capsule_evidence=evidence,
        scope='All files in authenticated L4 reader closure and each retained L5 file. No full repository census.',
        seconds=time.perf_counter()-started))
    print(json.dumps(dict(active=totals(rows),failed_l5=len(failed),seconds=time.perf_counter()-started)))


def finish():
    if (OUT/'ACCOUNTING.json').exists():raise ValueError('PreserveAccounting')
    from recursive_replay_retention import authenticated_closure
    import psutil
    started=time.perf_counter();before=read(OUT/'INVENTORY.json');deps=read(OUT/'DEPENDENCIES.json')
    baseline_record=read(OUT/'BASELINE.json');retentions={n:read(OUT/n/'RESULT.json') for n in ('L2_active_closure','L3','L4')}
    removed={r['path']:r for n in retentions for r in read(OUT/n/'PLAN.json')['actions']}
    current=[]
    for old in before['rows']:
        p=Path(old['path'])
        if str(p) in removed:
            if p.exists():raise ValueError('RetiredPathStillExists')
            continue
        now=metadata(p)
        for key in ('bytes','file_id','device','mtime_ns','cluster_bytes'):
            if now[key]!=old[key]:raise ValueError('UnintendedFileChange',str(p),key)
        current.append(dict(now,**{k:old[k] for k in ('tree','relative','stage','layer','realization')}))
    known={r['path'] for r in current if r['tree']=='wing'}
    if {str(p.resolve()) for p in CAMPAIGN.rglob('*') if p.is_file()}!=known:
        raise ValueError('UnexpectedCampaignPath')
    for r in deps['failed_l5']:
        if file_hash(r['path'])!=r['sha256']:raise ValueError('ChangedFailedL5')
    with authenticated_closure(read(CAMPAIGN/'L4/REOPEN.json')['reference']) as (state,closure,nodes):
        if nodes!=deps['nodes']:raise ValueError('ChangedActiveState')
        for r in deps['active']:
            if file_hash(r['path'])!=r['sha256']:raise ValueError('ChangedActiveBytes')
        science=state.scientific
    changed=[]
    for r in baseline_record['files']:
        if (ROOT/r['path']).stat().st_size!=r['bytes'] or file_hash(ROOT/r['path'])!=r['sha256']:changed.append(r['path'])
    expected=['research/sdf_cfd_mesher_v4/tools/run_authorized_continuation.py']
    if changed!=expected:raise ValueError('UnexpectedProtectedSourceChanges',changed)
    current_status=status();unrelated=baseline_record['unrelated_status']
    if [s for s in current_status if s in unrelated]!=unrelated:raise ValueError('UnrelatedWorkChanged')
    wing=[r for r in current if r['tree']=='wing'];active_paths=set(closure)
    active=[r for r in wing if r['path'] in active_paths]
    failure=[r for r in wing if r['layer']=='L5']
    other=[r for r in wing if r not in active and r not in failure]
    groups=defaultdict(list)
    for r in wing:groups[(r['layer'],r['realization'],r['stage'])].append(r)
    control=V4/'results/remaining_prism_critical_path'
    oldfiles=read(control/'CONTROL_FILE_EVIDENCE.json')['files']
    capsule={}
    for name in ('normal_candidate_clean_prism','normal_candidate_clean_replay_prism'):
        records=[r for r in oldfiles if r['path'].startswith(name+'/')]
        capsule[name]=dict(logical=sum(r['bytes'] for r in records),reported_allocation=sum(r['allocated'] for r in records),
            files={r['path'].removeprefix(name+'/'):dict(bytes=r['bytes'],sha256=r['sha256']) for r in records},
            generation=read(control/name/'GENERATION.json')['counts'],assembly=read(control/name/'ASSEMBLY.json'))
    missing={n:(CAMPAIGN/'L5'/n).stat().st_size for n in ('NEW_RECEIPT_INPUTS.json','QUALITY_DELTA.json') if not (CAMPAIGN/'L5/replay'/n).exists()}
    publication={f'L{k}':sum(r['bytes'] for r in before['rows'] if r['tree']=='wing' and r['layer']==f'L{k}' and r['stage']=='publication') for k in (3,4)}
    artifact_rows=[metadata(p) for p in OUT.rglob('*') if p.is_file()]
    result=dict(classification='V4RecursiveSuccessfulReplayRetentionConnectedL4PreservedL5Paused',
        scientific=science,retention=retentions,before=before['totals']['wing'],after=totals(wing),
        preserved_active_campaign=totals(active),preserved_l5_failure=totals(failure),other_retained_evidence=totals(other),
        active_closure=deps['active_totals'],external_ancestry=deps['outside_campaign'],
        reclaimed=dict(logical=sum(r['bytes'] for r in removed.values()),allocation=sum(r['cluster_bytes'] for r in removed.values()),files=len(removed)),
        before_groups=before['groups'],after_groups={'/'.join(k):totals(v) for k,v in groups.items()},capsule=capsule,
        remaining_l5=dict(known_absent_replay_files=missing,known_absent_replay_bytes=sum(missing.values()),
            two_prepare_hot_files_comparable_bytes=2*(CAMPAIGN/'L4/CANDIDATE_HOT.json').stat().st_size,
            measured_prior_publication_logical=publication,unchanged_cap=45120503439,
            measured_headroom=45120503439-totals(wing)['logical'],
            caveat='Not a certified future size upper bound. Control/root/index/page sizes can differ. Retained L5 indices already exist; do not copy full ancestry or proof closure.'),
        preservation=dict(unrelated_status=155,protected_files=len(baseline_record['files']),intentional_changes=changed,
            other_protected_unchanged=len(baseline_record['files'])-len(changed),active_hashes=len(deps['active']),failed_l5_hashes=len(deps['failed_l5'])),
        disk=disk(),study_allocation_before_summary=totals(artifact_rows),seconds=time.perf_counter()-started,
        baseline_to_finish_s=time.time()-baseline_record['time'],peak_process_working_set=psutil.Process().memory_info().peak_wset)
    durable_json(OUT/'ACCOUNTING.json',result)
    print(json.dumps({k:result[k] for k in ('classification','before','after','reclaimed','remaining_l5','preservation','disk')}))


def seal():
    if (OUT/'EVIDENCE.json').exists():raise ValueError('PreserveEvidence')
    source=[V4/'tools'/n for n in ('run_authorized_continuation.py','recursive_replay_retention.py','audit_wing_campaign_storage.py')]
    source += [V4/'tests/test_recursive_replay_retention.py',V4/'WING_STORAGE_CONNECTION_REPORT.md']
    paths=source+[p for p in OUT.rglob('*') if p.is_file()]
    rows=[dict(path=p.relative_to(ROOT).as_posix(),bytes=p.stat().st_size,sha256=file_hash(p)) for p in sorted(paths)]
    result=dict(schema='V4WingStorageConnectionEvidence/1',starting_head=read(OUT/'BASELINE.json')['head'],
        branch=subprocess.check_output(['git','branch','--show-current'],text=True).strip(),files=rows,
        geometry='079606dc36582ed38e51e5485e1f853e247a2ae40b544414c9e649552ae85311',
        last_qualified_layer=4,scientific=read(OUT/'ACCOUNTING.json')['scientific'],
        no_generation=True,no_publication=True,no_cap_change=True,disk=disk(),
        retained_task_before_this_record=totals([metadata(p) for p in paths]),
        note='L2/ is the superseded non-mutating draft. L2_active_closure/, L3/ and L4/ are the applied plans, all protecting latest L4. Test failure XML is retained alongside passing corrected tests.')
    durable_json(OUT/'EVIDENCE.json',result)
    print(json.dumps(dict(evidence=file_hash(OUT/'EVIDENCE.json'),report=file_hash(V4/'WING_STORAGE_CONNECTION_REPORT.md'),
        disk=result['disk'],allocation=result['retained_task_before_this_record'])))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['baseline','inventory','dependencies','finish','seal'])
    p.add_argument('--output',type=Path);a=p.parse_args()
    if a.output:
        OUT=a.output.resolve()
        if not OUT.is_relative_to(V4/'results'):raise ValueError('OutputOutsideV4Results')
    globals()[a.mode]()
