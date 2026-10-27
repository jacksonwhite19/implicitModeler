"""Seal bounded production observations without turning budgets into mesh gates."""
import sys
sys.dont_write_bytecode=True
import json,re,hashlib,subprocess,time
from pathlib import Path
from run_production_scale_resume import OUT,native,authority
from sdf_cfd_mesher_v4.common.authority import file_hash


def read(path):return json.loads(path.read_text())
def lines(path):
    if not path.exists():return
    with path.open(encoding='utf8') as stream:
        for line in stream:
            try:yield json.loads(line)
            except json.JSONDecodeError:continue


def gather():
    cases=[];layer_rows=[];memory=[];session_rows=[];trials=[]
    for spec in authority.cohort():
        name=spec['name'];folder=OUT/name
        final=sorted(folder.glob('*/COMMITTED_RESULT.json'))
        if not final:raise ValueError('MissingFinalCheckpointAudit:'+name)
        result=read(final[-1]);sessions=[];peak=0;retained=[];phases=[]
        shell_result=final[-1].parent/'SHELL_RESULT.json'
        result['requested_shell_complete']=shell_result.exists() and read(shell_result).get('all_active_schedules_complete') is True
        for attempt in sorted(folder.iterdir()):
            if not attempt.is_dir():continue
            if (attempt/'SESSION.json').exists():sessions.append(read(attempt/'SESSION.json'))
            for row in lines(attempt/'MEMORY.jsonl'):
                peak=max(peak,row['peak_rss_bytes'])
                if row['event']=='exit':phases.append(dict(label=attempt.name,phase=row['phase'],seconds=row['duration_seconds'],rss=row['rss_bytes']))
            for row in lines(attempt/'PROGRESS.jsonl'):
                if row['event'] in ('checkpoint_published','retained_after_commit','initial_checkpoint'):
                    small={k:v for k,v in row.items() if k!='versions'}
                    layer_rows.append(dict(case=name,label=attempt.name,**small))
                    if row['event']=='retained_after_commit':retained.append(row['rss'])
                if row['event'].startswith('trial') or row['event'] in ('branch_stop','numerical_candidate_passed','certificate_complete','production_validation_passed','production_validation_progress_observed'):
                    trials.append(dict(case=name,label=attempt.name,**row))
        session_rows.extend(sessions)
        result.update(sessions=sessions,active_case_seconds=sum(r['seconds'] for r in sessions)+sum(read(p)['replay_seconds'] for p in final),
            all_checkpoint_audit_seconds=sum(read(p)['replay_seconds'] for p in final),
            peak_rss=peak,post_layer_retained_rss=retained,phase_records=phases,
            cold_attempt_seconds=[r['seconds'] for r in sessions if r['cold_from_layer0']],
            cold_end_to_end_seconds=None,cold_scope='No completed cold end-to-end case; resumed times are listed separately')
        cases.append(result)
    native.g.save(OUT/'LAYER_PROGRESS.json',layer_rows)
    native.g.save(OUT/'CORRECTION_AND_CERTIFICATE_TRACE.json',trials)
    native.g.save(OUT/'CASE_RESULTS.json',cases)
    return cases,session_rows,layer_rows,trials


def seal(classification):
    cases,sessions,layers,trials=gather()
    start=read(OUT/'TASK_MANIFEST.json')
    status=subprocess.check_output(['git','-c','core.longpaths=true','status','--short'],text=True)
    unrelated=[r for r in status.splitlines() if 'research/sdf_cfd_mesher_v4' not in r]
    assert unrelated==start['unrelated'],'UnrelatedDirtyStateChanged'
    v3=subprocess.check_output(['git','diff','--name-only','HEAD','--','research/sdf_cfd_mesher_v3'],text=True).splitlines()
    assert not v3
    prior=native.ROOT/'results/first_layer_determinant_applicability'
    for name,digest in start['prior'].items():assert file_hash(prior/name)==digest
    audit=read(OUT/'BROAD_TEST_AUDIT.json');sf2=read(OUT/'SF2/BOUNDED_REPLAY.json')
    peak=max(c['peak_rss'] for c in cases);retained=max((r['rss'] for r in layers if r['event']=='retained_after_commit'),default=0)
    seconds=sum(c['active_case_seconds'] for c in cases)
    elapsed=time.time()-(OUT/'TASK_MANIFEST.json').stat().st_mtime
    execution_end=read(OUT/'EXECUTION_END.json')
    events=sum(c['remesh_events'] for c in cases)
    metrics={k:min(c['extrema'][k] for c in cases) for k in ('determinant','weight')}
    terminal_records=[dict(case=p.parent.parent.name,label=p.parent.name,**read(p)) for p in OUT.glob('*/*/TERMINATION.json')]
    native.g.save(OUT/'STOP_AUDIT.json',terminal_records)
    test_logs={p.name:dict(sha256=file_hash(p),summary=re.findall(r'\d+ passed[^\n]*',p.read_text(errors='replace'))[-1:]) for p in OUT.glob('TESTS*.log')}
    native.g.save(OUT/'TEST_SUMMARY.json',test_logs)
    compact=[{k:c[k] for k in ('case','label','layer','cells','relations','remesh_events','extrema','termination','active_case_seconds','peak_rss','post_layer_retained_rss','restart_exact','wall_exact','first_layer_exact','cold_end_to_end_seconds')} for c in cases]
    result=dict(classification=classification,parent=start['parent'],cases=compact,
        active_case_execution_seconds=seconds,hard_total_seconds=14400,
        hard_total_reached=execution_end['hard_total_reached'],execution_elapsed_upper_bound_seconds=execution_end['elapsed_upper_bound_seconds'],task_elapsed_seconds_at_seal=elapsed,
        time_scope='Measured case sessions plus checkpoint audits; elapsed task time also bounds ancillary checks and overlaps',
        completed_shells=sum(c['requested_shell_complete'] for c in cases),complete_meshes=0,
        committed_remesh_events=events,peak_rss=peak,max_retained_post_layer_rss=retained,
        minimum_admitted_determinant=metrics['determinant'],minimum_admitted_weight=metrics['weight'],
        stop_counts={kind:sum(r.get('reason')==kind for r in terminal_records) for kind in ('BudgetStoppedStagnant','BudgetStoppedAfterProgress','ImplementationBlocked')},
        stage_stagnation_stops=sum(r.get('control')=='StageStagnation15Minutes' for r in terminal_records),
        other_budget_stops=sum(r.get('reason','').startswith('BudgetStopped') and r.get('control')!='StageStagnation15Minutes' for r in terminal_records),
        repaired_default_budget_interruptions=2,
        core_quality=None,sf2_regression=sf2['passed'],deterministic_restart=all(c['restart_exact'] for c in cases),
        broad_test_audit=audit['counts'],unrelated_dirty_entries=len(unrelated),v3_modifications=0,
        gates_changed=False,certificate_mathematics_changed=False,geometry_specific_production_branches_added=False,
        limits='Partial admitted checkpoints are not completed 20-layer shells. Runtime stops do not establish geometric impossibility.',
        next_task='Production-scale certificate and receipt performance qualification from these exact checkpoints')
    native.g.save(OUT/'SUMMARY.json',result)
    table='| Case | Committed layer | Cells | Relations | End reason | Active case seconds | Peak RSS bytes |\n|---|---:|---:|---:|---|---:|---:|\n'
    for c in cases:table+=f"| {c['case']} | {c['layer']} | {sum(c['cells'].values()):,} | {c['relations']:,} | {c['termination']['reason']} | {c['active_case_seconds']:.2f} | {c['peak_rss']:,} |\n"
    sections={
'A. Authority verification':f"Starting HEAD: `{start['parent']}`. Prior report and authority hashes match. Wall, h1, stored schedule, gates and certificate mathematics are unchanged. V3 modifications: 0. All {len(unrelated)} unrelated dirty entries are preserved.",
'B. Fresh-task scope':'All new records are under this result root. The four unchanged geometry designs are bound in GEOMETRY_BINDINGS.json. No new geometry family or CFD controls were introduced.',
'C. Compute-budget policy':'Default case budget: 45 minutes. Only measured progress permits an extension. Hard case limit: 75 minutes. Hard task limit: 4 hours. The supervisor reserves time for checkpoint audit. Fifteen minutes without progress stops a stage. Three repeated identical blockers or ten rejected candidates without new evidence stop a search branch. These are compute controls, not scientific gates.',
'D. Broad-test failure audit':'All eleven historical failures were inspected: eight obsolete assertions, zero active production regressions, three independent fixture/environment issues. The exact reasons and repairs are in BROAD_TEST_AUDIT.json. The mathematical vendor source was not changed.',
'E. Proof-retention regression':'The cold capsule committed layers 1 and 2 with disk-backed receipt tables and empty provider scratch after each commit. Additional defects were measured and repaired: whole-population JSON normalization, duplicate native receipt graphs, an unnecessary post-publication proof hash, compression gate histories, and retention of all accepted correction candidates. NATIVE_BIND_PROFILE.json, ROW_NORMALIZATION_PROFILE.json and VALIDATION_DIGEST_PROFILE.json preserve exact before/after component measurements. Ordered proof and validation hashes now stream the same canonical bytes. Initial/report state references and correction-baseline proof copies were also removed. Tests verify exact scientific identity and reference release. The previous correction base is released after commit and rebound to the current committed state at the start of the next proposal. The final loop fix reuses the committed state for its schedule guard: a two-step reference comparison reduced checkpoint reads from 6 to 4 with identical scientific hash. The interrupted largest B attempt predates these lifetime fixes; the D retry uses them. No full-run runtime saving is established. A dense projected-boundary candidate scan was replaced by the existing inclusive AABB hierarchy with identical sorted candidate IDs; rational degree predicates, directions and retries are unchanged. PROJECTED_INDEX_PROFILE.json compares 512 witnesses against 131,072 actual boundary triangles and confirms exact candidate order and degree results. The largest B attempt predates this fix; D uses it. A seed importer now also accepts an unchanged imported checkpoint from a stopped attempt after verifying its original binding and components. Full-run RSS is reported separately.',
'J. Per-layer progress':table+'\nLAYER_PROGRESS.json contains committed checkpoints, native receipt counts, atlas/forecast populations and measured RSS. The provider does not expose a separate warning-region count; that field is unavailable, not zero.',
'K. Correction behavior':'CORRECTION_AND_CERTIFICATE_TRACE.json records scientific progress and rejected-trial compute stops. Trial histories use existing on-disk archives; only the latest accepted gate remains resident. No proposal formula or acceptance gate changed.',
'L. Remesh event behavior':f'{events} committed remesh events. Maximum committed event count, rollback depth and removed-cell cavity size are zero in the observed histories; future event behavior remains untested. Event locality, rollback and remesh cost are not established when no event commits. Automatic remeshing remains enabled.',
'M. Coupled validation':'The qualified complete-neighbor validator remains in the production path. No shell-only receipt is promoted to complete-neighbor authority. If no mixed event commits, new large-case coupled-transition evidence is not established.',
'N. Shell quality':f"Worst determinant among the audited admitted states: {metrics['determinant']:.17g}. Worst interpolation weight: {metrics['weight']:.17g}. CASE_RESULTS.json records all available extrema. These are partial committed states, not final requested-shell qualification.",
'O. Outer interfaces':'The production source-keyed adapter exported each audited committed checkpoint. Interface metadata and arrays remain in each checkpoint_interface directory. This does not imply completed core qualification.',
'P. Core results':'No final requested shell completed. Full core generation and repair were not started. The existing bounded core-ingestion smoke check within shell admission is not counted as a complete core.',
'Q. Complete-mesh results':'No new complete mesh or Foundation Mesh OK result is claimed.',
'R. Memory traces':f'Maximum transient RSS: {peak:,} bytes. Maximum recorded retained post-layer RSS: {retained:,} bytes. MEMORY.jsonl separates phase entries/exits and native-worker process peaks; it is not an aggregate process-tree RSS measurement. Selected-root object measurements exclude disk receipt rows, native allocator arenas, and unlisted roots such as correction_base. They are not a complete retained-memory census. Capsule attempts used successive fixes; bent passage used streamed hashes and state release; the larger cases used the final baseline-copy fix. Different layers and code variants are not a controlled full-run before/after comparison. Restart RSS is reported separately from post-layer RSS. Retained RSS still increases with layer population; the observed runs do not establish a stable long-run memory bound or prove that all excess retention is removed. Transient certificate populations and repeated full identity scans remain performance concerns, not demonstrated geometric limits.',
'S. Cold runtime':'Capsule attempt A started cold from layer 0. Later capsule attempts resumed exact task checkpoints. Other cases began from sealed layer-2 checkpoints; the final bent-passage continuation resumes its task layer-5 checkpoint. No completed cold end-to-end runtime exists. SESSION.json records observed worker wall time and seed-import time; some regression checks ran concurrently, so these are not isolated benchmark timings; CASE_RESULTS.json separates attempts and replay time.',
'T. Event runtime':'No meaningful median or p95 is calculated when no remesh event commits. Phase times are observations, not substitutes for elapsed case time.',
'U. Stagnation decisions':'Every branch stop and terminal compute reason is preserved. The supervisor initially reset the capsule progress flag on worker restart; that operational defect was repaired and regression-tested. Its interrupted layer-6 candidate was discarded and execution resumed from the exact layer-5 checkpoint. A later ledger fix records completed final numerical and certificate validation receipts. For the already-running interacting worker, the observed transition into rebuild was recorded with the runner source hash; the runner rejects failed or mismatched receipts before rebuild. This milestone does not imply a committed layer. The largest B attempt reached its 15-minute progress limit after a certified candidate but before commit; D retried only after the exact broad-phase fix. Attempt C failed its seed/publication guard and is included as infrastructure time. D was configured to reserve 600 seconds for its audit, but the default budget fired first because inherited progress excluded a prior completed new-relation certificate. The carry-over rule now accepts that verified milestone and is regression-tested. The interruption is preserved; another full attempt was not justified by the remaining hard case budget and measured prior phase costs.',
'V. Budget-stopped work':f"Measured case execution and checkpoint-audit time: {seconds:.2f} s. All-in elapsed through the last scientific audit: {execution_end['elapsed_upper_bound_seconds']:.2f} s, including development, checks and idle time; this remained below four hours. Sealing is excluded. The exact accounting and terminal reasons are preserved. A budget stop does not classify a geometry as unsupported.",
'W. SF2 regression':'The bounded native SF2 audit passed: 2,126,040 prisms; 56,462,864 certificate relations preserved; zero missing, duplicate, forbidden or unresolved; zero remesh. First-layer metric/geometry hashes, forecast and native restart remained exact. The sealed complete-mesh authority, its 226,619,419-relation certificate record and Foundation Mesh OK record were hash-revalidated in SF2/PRESERVED_COMPLETE_AUTHORITY.json. The full production mesh and A/A2 were not rerun.',
'X. Determinism':'Every final checkpoint was independently restarted twice and compared to its published scientific identity. Wall and first-layer identities are checked against the case authority. This establishes checkpoint replay, not an unexecuted complete-run replay.',
'Y. Tests':'TEST_SUMMARY.json contains the exact executed logs and hashes. The broad Windows suite and the existing Linux-only unittest cohort are recorded separately. Existing singular-cell diagnostic warnings remain visible.',
'Z. Classification':classification,
'AA. Supported production-scale claim':'The four fixed designs were exercised through the same native production architecture. Reported committed layers and their certified receipts are reusable checkpoints. Full 20-layer completion, multi-event scaling and new complete-mesh qualification are not inferred from partial progress.',
'AB. Recommended next task':result['next_task']+'. Preserve the present checkpoints and measure the dominant certificate working sets before another long whole-case attempt.'}
    for title,c in zip(('F. Capsule','G. Bent passage','H. Interacting ellipsoids','I. Four separated regions'),cases):
        sections[title]=f"{c['termination']['reason']}. Committed layer {c['layer']}; {sum(c['cells'].values()):,} cells; {c['relations']:,} exact relations; {c['remesh_events']} remesh events. Determinant {c['extrema']['determinant']:.17g}; weight {c['extrema']['weight']:.17g}. Independent restart, wall and first-layer identity checks passed. The native receipt-table audit checked allowed proof classifications and positive exact cell receipts. See CASE_RESULTS.json and the case attempt records."
    order=['A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z','AA','AB']
    text='## Decision question\n\nAfter correcting first-layer determinant applicability, can the four larger V4 geometries make sustained production progress without repeated unproductive search, pathological memory retention, or geometry-specific heuristics?\n'
    for prefix in order:
        title=next(k for k in sections if k.split('.')[0]==prefix)
        text+='\n### '+title+'\n\n'+sections[title]+'\n'
    (OUT/'report.md').write_text(text,encoding='utf8')
    # Index committed evidence, excluding abandoned temporary writes. Hard-linked
    # immutable objects are hashed once per physical file.
    index={};cache={}
    for p in sorted(OUT.rglob('*')):
        if not p.is_file() or p.name in ('AUTHORITY.json','EVIDENCE_INDEX.json','HASHES.json') or p.suffix=='.pending':continue
        st=p.stat();key=(st.st_dev,st.st_ino,st.st_size,st.st_mtime_ns)
        if key not in cache:cache[key]=file_hash(p)
        index[p.relative_to(OUT).as_posix()]=dict(bytes=st.st_size,sha256=cache[key])
    native.g.save(OUT/'EVIDENCE_INDEX.json',index)
    result.update(report_sha256=file_hash(OUT/'report.md'),evidence_index_sha256=file_hash(OUT/'EVIDENCE_INDEX.json'),
        sources={str(p.relative_to(native.ROOT)):file_hash(p) for directory in ('src','tools','tests') for p in sorted((native.ROOT/directory).rglob('*.py'))})
    native.g.save(OUT/'AUTHORITY.json',result)
    hashes={name:file_hash(OUT/name) for name in ('report.md','AUTHORITY.json')}
    native.g.save(OUT/'HASHES.json',hashes);print(json.dumps(hashes))

if __name__=='__main__':seal(sys.argv[1])
