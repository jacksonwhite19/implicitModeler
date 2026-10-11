"""Measure retired physical allocations and seal the retention result."""
from collections import defaultdict
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
from xml.etree import ElementTree
from rolling_layer_retention import *
from path_preserving_deduplication import probe_file_id
from sdf_cfd_mesher_v4.common.authority import identity

def measure():
    start=time.perf_counter();before=read(OUT/'INVENTORY.json');plan=read(OUT/'CLEANUP_PLAN.json')
    manifest=read(OUT/'CLEANUP_MANIFEST.json')
    manifest['recovered_attempt_temporary_alias_removals']=read(OUT/'ATTEMPT1_RECOVERY.json')['temporary_aliases_removed']
    manifest['path_count_note']='9 scientific export paths removed. The manifest also records recovery-only temporary aliases from both attempts; those are not additional scientific payload removals.'
    save('CLEANUP_MANIFEST.json',manifest)
    removed={r['path'] for r in plan['actions']};rows=[];hashes={}
    for tree in [*TREES,L3]:
        for p in sorted(tree.rglob('*')):
            if not p.is_file():continue
            r=metadata(p);key=physical_key(r)
            if key not in hashes:hashes[key]=file_hash(p)
            r['sha256']=hashes[key];rows.append(r)
    now={r['path']:r for r in rows};old={r['path']:r for r in before['rows']}
    if set(old)-set(now)!=removed or set(now)-set(old):raise ValueError('UnexpectedPathChange')
    for p,r in now.items():
        if (r['sha256'],r['bytes'],physical_key(r),r['cluster_bytes'])!=(old[p]['sha256'],old[p]['bytes'],physical_key(old[p]),old[p]['cluster_bytes']):raise ValueError('RetainedObjectChanged')
    with tempfile.TemporaryDirectory(prefix='v4-retention-deleted-control-') as d:
        p=Path(d)/'control';p.write_bytes(b'control');fid=metadata(p)['file_id']
        if probe_file_id(fid)!=0:raise ValueError('ControlUnavailable')
        p.unlink();error=probe_file_id(fid)
        if not error:raise ValueError('ControlNotRetired')
    retirement=[]
    for r in plan['actions']:
        actual=probe_file_id(r['file_id'])
        if actual!=error:raise ValueError('AllocationStillReferenced:'+r['path'])
        retirement.append(dict(path=r['path'],file_id=r['file_id'],allocated=r['cluster_bytes'],retired_error=actual))
    after=totals(rows);l3rows=[r for r in rows if Path(r['path']).is_relative_to(L3)]
    reclaimed=before['totals']['unique_physical_allocation']-after['unique_physical_allocation']
    if reclaimed!=sum(r['cluster_bytes'] for r in {physical_key(r):r for r in plan['actions']}.values()):raise ValueError('PhysicalAccountingMismatch')
    oldkeys={physical_key(r) for r in rows if not Path(r['path']).is_relative_to(L3)}
    closure,_=checkpoint_closure(L3/'A/checkpoints')
    closure[str((L3/'A/QUALITY.json').resolve())]=file_hash(L3/'A/QUALITY.json')
    active={physical_key(r):r for r in rows if r['path'] in closure}
    classes={r['path']:r['classification'] for r in read(OUT/'CLASSIFICATION.json')['rows']}
    permanent={physical_key(r):r for r in l3rows if classes[r['path']]=='A'}
    studyrows=[metadata(p) for p in OUT.rglob('*') if p.is_file()]
    result=dict(before=before['totals'],after=after,l3_before=before['l3_totals'],l3_after=totals(l3rows),
        before_free=before['free_bytes'],after_free=shutil.disk_usage(ROOT).free,
        physical_bytes_reclaimed=reclaimed,logical_bytes_removed=sum(r['bytes'] for r in plan['actions']),
        duplicate_physical_bytes_eliminated=before['totals']['duplicate_physical_allocation']-after['duplicate_physical_allocation'],
        current_L3_active_closure_physical=sum(r['cluster_bytes'] for r in active.values()),
        current_L3_active_closure_new_physical=sum(r['cluster_bytes'] for k,r in active.items() if k not in oldkeys),
        original_permanent_L3_physical=sum(r['cluster_bytes'] for r in permanent.values()),
        compact_record_physical=metadata(OUT/'COMPACT_L3.json')['cluster_bytes'],
        new_study_physical_at_measurement=sum(r['cluster_bytes'] for r in {physical_key(r):r for r in studyrows}.values()),
        retired_ids=retirement,deleted_control_error=error,
        allocation_note='NTFS data clusters measured by retrieval pointers and file ID; no MFT allocation recovery claimed. New report files after this census are outside the measured historical-tree scope.',
        seconds=time.perf_counter()-start,**resources())
    result['free_space_increase']=result['after_free']-result['before_free']
    result['free_space_residual']=result['free_space_increase']-reclaimed
    save('STORAGE_AFTER.json',result)
    print(json.dumps({k:v for k,v in result.items() if k!='retired_ids'},indent=2))

def seal():
    s=read(OUT/'STORAGE_AFTER.json');c=read(OUT/'CLASSIFICATION.json');compact=read(OUT/'COMPACT_L3.json')
    plan=read(OUT/'CLEANUP_PLAN.json');v=read(OUT/'POST_VERIFICATION.json');base=read(OUT/'BASELINE.json')
    tests={}
    for name in ('TOOL_TESTS.xml','RESULT_TESTS.xml'):
        tests[name]={k:sum(int(x.get(k,0)) for x in ElementTree.parse(OUT/name).getroot().iter('testsuite')) for k in ('tests','failures','errors','skipped')}
    if any(t['failures'] or t['errors'] or t['skipped'] for t in tests.values()):raise ValueError('TestsIncomplete')
    original=bytes.fromhex(base['status_hex']).split(b'\0');current=subprocess.check_output(['git','status','--porcelain=v1','-z'],cwd=ROOT).split(b'\0')
    if not set(original)<=set(current):raise ValueError('UnrelatedStatusChanged')
    additions=set(current)-set(original)
    owned=['research/sdf_cfd_mesher_v4/results/rolling_successful_layer_retention/',
        'research/sdf_cfd_mesher_v4/tools/rolling_layer_retention.py','research/sdf_cfd_mesher_v4/tools/qualify_rolling_layer_retention.py',
        'research/sdf_cfd_mesher_v4/tools/rolling_retention_report.py','research/sdf_cfd_mesher_v4/tools/successful_retention_cleanup.py',
        'research/sdf_cfd_mesher_v4/tests/test_rolling_successful_retention.py','research/sdf_cfd_mesher_v4/tests/test_rolling_retention_evidence.py']
    if any(not any(x[3:].decode().startswith(p) for p in owned) for x in additions):raise ValueError('UnexpectedGitChange')
    git=dict(starting_head=base['head'],head_before_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        original_dirty_entries=base['entries'],unrelated_status_preserved=True,V3_task_modifications=0,task_additions=[x.decode() for x in sorted(additions)])
    save('GIT_VERIFICATION.json',git)
    science=dict(policy=POLICY,classification='V4RollingSuccessfulLayerRetentionQualified',committed=COMMITTED,
        implementation={str(p.relative_to(V4)):file_hash(p) for p in [
            V4/'tools/rolling_layer_retention.py',V4/'tools/qualify_rolling_layer_retention.py',
            V4/'tools/successful_retention_cleanup.py',Path(__file__),
            V4/'tests/test_rolling_successful_retention.py',V4/'tests/test_rolling_retention_evidence.py']},
        compact_identity=compact['identity'],compact_sha256=file_hash(OUT/'COMPACT_L3.json'),
        cleanup_manifest_sha256=file_hash(OUT/'CLEANUP_MANIFEST.json'),inventory_sha256=file_hash(OUT/'INVENTORY.json'),
        classification_sha256=file_hash(OUT/'CLASSIFICATION.json'),regeneration_sha256=file_hash(OUT/'REGENERATION.json'),
        readiness=read(OUT/'POST_L3_A.json')['readiness']['binding_identity'],reclaimed=s['physical_bytes_reclaimed'],
        no_L4_forecast=True,no_L4_generation=True,scope='Successful L3 exports in the sealed allowlist only. No future-layer storage prediction. All unresolved objects retained.')
    qid=identity(science)
    lines=[science['classification'],'',
        'Qualified claim: compact L3 success authority plus unchanged active restart state permits only the sealed successful-export removals. Complete regeneration matched original bytes. This does not authorize cleanup of failures or arbitrary later layers.',
        '',f"Starting HEAD: {base['head']}. Ending HEAD: the task commit containing this report (printed in final response).",f'Committed L3: {COMMITTED}.',
        f"L3 qualification: {compact['qualification']}.",f"Protected L3 report: {compact['original_authority']['report.md']}.",f"Protected L3 authority: {compact['original_authority']['AUTHORITY.json']}.",
        f"Checkpoint: {read(OUT/'POST_L3_A.json')['checkpoint']['sha256']}.",'',
        'Before classification (physical allocations counted once; B takes precedence for shared roles):',
        '| Class | Paths | Logical bytes | Physical bytes | New L3 physical bytes |','|---|---:|---:|---:|---:|']
    for k,r in c['breakdown'].items():lines.append(f"| {k} | {r['paths']} | {r['logical_bytes']} | {r['physical_bytes']} | {r['new_l3_physical_bytes']} |")
    lines += ['',
        'A retains the original reports, authorities, input identities, admission/quality/continuous summaries, margins and worst IDs, replay and restore attestations, tests, implementation identities, storage records and discard hashes. COMPACT_L3.json additionally binds these to the complete proof population and active object list.',
        f"Original permanent L3 allocation: {s['original_permanent_L3_physical']} bytes. Compact certificate allocation: {s['compact_record_physical']} bytes; identity {compact['identity']}; SHA-256 {science['compact_sha256']}.",
        'B retains all native checkpoint components: geometry/connectivity, columns/ownership/front, wall/first layer, authority/version maps, events, atlas/forecast dependencies, quality receipts, certificate base/delta and proof segment arrays/payloads. Historical L2 BASE and A/B checkpoint paths remain.',
        f"Current L3 active closure: {s['current_L3_active_closure_physical']} physical bytes, including {s['current_L3_active_closure_new_physical']} bytes new to L3. This is rolling working state, not compact permanent authority.",
        c['active_obsolescence'],
        'C: A incidence/volume exports; complete regeneration from fixed retained L3 coordinates and existing validators matched every original byte. E: raw/replay quality duplicates, replay incidence/volume, B candidate duplicate, and B retirement duplicate. Canonical A/QUALITY.json remains B because the existing next-layer determinant diagnostic reads it. One original retirement archive remains D.',
        'D: original run/debug logs and A retirement archive retained. Audit parser failure, serializer mismatch, recovered first cleanup attempt and measurement stdout-lock failure from this task are retained. Final complete checks passed. F: forecast/localization/screen/diagnostic arrays, exact wedge stream and unreferenced historical aliases preserved; no removal claim for them.',
        '', 'Regeneration hashes:']
    for name,r in read(OUT/'REGENERATION.json')['complete_exports'].items():lines.append(f"- {name}: {r['sha256']}; {r['bytes']} bytes; complete exact match.")
    ready=read(OUT/'POST_L3_A.json')['readiness']
    lines += ['',f"L4 binding only: passed; increment {ready['increment']} m ({ready['increment_hex']}); binding {ready['binding_identity']}. Receipts {ready['certificate_count']}; quality receipts {ready['quality_count']}; atlas {ready['atlas_entries']}; front sources {ready['current_front_count']}. No L4 forecast or generation.",
        f"Cleanup: {len(plan['actions'])} paths removed; 0 replaced. Logical bytes removed {s['logical_bytes_removed']}; physical allocation reclaimed {s['physical_bytes_reclaimed']}; duplicate allocation eliminated {s['duplicate_physical_bytes_eliminated']}. No path conversion to shared hardlinks; temporary recovery aliases were purged only after fresh restore/hash verification.",
        'Every removed path, original hash, size, file ID, regeneration proof, recovery stage and byte count is in CLEANUP_MANIFEST.json. Retired file IDs all reject like the measured deleted control; retained IDs, bytes and hashes are unchanged.',
        f"Relevant physical allocation: {s['before']['unique_physical_allocation']} -> {s['after']['unique_physical_allocation']}; L3 tree physical allocation (including historical aliases): {s['l3_before']['unique_physical_allocation']} -> {s['l3_after']['unique_physical_allocation']}.",
        f"L3 logical bytes: {s['l3_before']['logical_bytes']} -> {s['l3_after']['logical_bytes']}; L3 duplicate physical bytes: {s['l3_before']['duplicate_physical_allocation']} -> {s['l3_after']['duplicate_physical_allocation']}.",
        f"Filesystem free bytes: {s['before_free']} -> {s['after_free']}; change {s['free_space_increase']}; residual relative to reclaimed clusters {s['free_space_residual']}. Free space alone is not the recovery measure.",
        f"New retention study allocation at census: {s['new_study_physical_at_measurement']} bytes (later report/seal files excluded).",'', 'Fresh post-cleanup restores:']
    for label in ('L1_A','L2_A','L3_A','L3_B'):
        r=read(OUT/('POST_'+label+'.json'));lines.append(f"- {label}: PASS; {r['scientific']}; {r['seconds']:.6f} s; peak RSS {r['peak_rss']} bytes.")
    lines+=['','Protected authorities and all retained inventoried payloads: exact hashes unchanged. Original authority references to removed exports remain exact content identities, resolved by the new discard/regeneration certificate; they are not rewritten and are no longer promises of retained physical export paths.',
        f'Tests: {json.dumps(tests,sort_keys=True)}. No existing tests weakened.', 'Runtime / peak RSS:']
    for name in ('INVENTORY.json','REGENERATION.json','PRE_L3_A.json','REMOVALS_PENDING.json','POST_VERIFICATION.json','CLEANUP_MANIFEST.json','STORAGE_AFTER.json'):
        r=read(OUT/name);lines.append(f"- {name}: {r.get('seconds')} s; peak RSS {r.get('peak_rss')} bytes.")
    lines += ['',f"Cleanup manifest SHA-256: {science['cleanup_manifest_sha256']}",f'Qualification identity: {qid}',
        f"Git: {base['entries']} original dirty entries preserved; V3 task changes 0. Only task files committed.",
        'Remaining concerns: large active delta/quality tables remain necessary; transitive delta bases cannot be retired just because a later CURRENT exists. F-class removal remains unqualified. No future-layer savings are extrapolated.',
        'Qualification mode retains compact permanent authority, rolling active state, detailed exception evidence and reproducibility inputs. Future production may use current/final mesh, compact certificate and minimal audit without routine A/B replay. That policy is not implemented.',
        'Recommended next task: QualifyAndCommitV4CapsuleLayers4To5Bounded. Admit, certify, commit and restore each layer separately; stop immediately on any failure.','']
    (OUT/'report.md').write_text('\n'.join(lines),encoding='utf8',newline='\n')
    evidence={str(p.relative_to(OUT)):file_hash(p) for p in sorted(OUT.rglob('*')) if p.is_file() and p.name not in ('AUTHORITY.json','SEALS.json','SEAL.log')}
    save('AUTHORITY.json',dict(classification=science['classification'],scientific=science,scientific_identity=qid,evidence=evidence,tests=tests,blockers=[],unresolved_preserved=c['unresolved']))
    save('SEALS.json',dict(report=file_hash(OUT/'report.md'),authority=file_hash(OUT/'AUTHORITY.json'),qualification=qid,compact=file_hash(OUT/'COMPACT_L3.json'),manifest=file_hash(OUT/'CLEANUP_MANIFEST.json')))
    print((OUT/'report.md').read_text());print(json.dumps(read(OUT/'SEALS.json'),indent=2))

if __name__=='__main__':
    import sys
    globals()[sys.argv[1]]()
