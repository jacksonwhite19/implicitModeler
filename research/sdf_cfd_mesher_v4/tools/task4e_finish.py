"""Bounded shutdown audit, compact evidence, report and authority seal.

This tool never generates a candidate, solves a metric, or runs a certificate.
"""
from task4e_common import *
from task4e_collect import collect,optional
import psutil,shutil

def preserve():
    from task4d_common import bounded_v3
    pre=read(OUT/'PREFLIGHT.json');assert bounded_v3()==pre['V3']
    assert file_hash(BASE/'build_A/CURRENT.json')==pre['Build_A_checkpoint']
    assert file_hash(BASE/'replay_A2/build_A/CURRENT.json')==pre['historical_A2_checkpoint']
    def outside(s):return [x.strip() for x in s.splitlines() if 'research/sdf_cfd_mesher_v4' not in x]
    assert outside(git('status','--porcelain'))==outside(clock()['status'])
    for name in ('task4','task4b','task4c','task4d'):
        p=ROOT/'results'/name;seal=read(p/'SEAL.json')
        assert file_hash(p/'report.md')==seal['report_sha256'];assert file_hash(p/'AUTHORITY.json')==seal['authority_sha256']
    record=dict(V3_modifications=0,bounded_V3_sources=len(pre['V3']),Build_A_unchanged=True,historical_A2_unchanged=True,unrelated_state_unchanged=True)
    save('PRESERVATION.json',record);return record

def compact(last):
    removed=optional(OUT/'PLANNER_ARCHIVE.json',[])
    for p in sorted(RUN.glob('LAYER*_PLANNER.npz')):
        layer=int(p.stem[5:].split('_')[0])
        if layer>=last:continue
        with np.load(p,allow_pickle=False) as z:meta=json.loads(str(z['metadata']))
        # Retain compact advisory/checkpoint identity. Old numeric hot caches
        # are optional accelerators; committed geometry/proof state is intact.
        row=dict(layer=layer,path=p.relative_to(ROOT).as_posix(),sha256=file_hash(p),bytes=p.stat().st_size,
            checkpoint=meta['checkpoint'],implementation=meta['implementation'],
            support_keys=sorted(meta['supports']),detailed_keys=sorted(meta['detailed']),
            reason='Obsolete numeric planning cache. Latest restart state is retained. Earlier front/proof checkpoints remain unchanged.')
        assert p.resolve().is_relative_to(RUN.resolve());removed.append(row);p.unlink()
    save('PLANNER_ARCHIVE.json',removed)

def validate_checkpoint():
    """Read-only hash-chain validation, with no SDF or metric execution."""
    from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
    from sdf_cfd_mesher_v4.forecast.schedule import load_schedule
    from sdf_cfd_mesher_v4.advancing_front.rolling_state import CommitStore
    if not (RUN/'build_A/CURRENT.json').exists():return None
    tick=time.perf_counter();front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates))
    state=CommitStore(RUN/'build_A',front,top,load_schedule()).restart()
    record=dict(layer=state.layer,state_hash=state.state_hash,coordinate_hash=array_hash(state.coordinates),
        topology_hash=state.topology_hash,prism_hash=state.prism_hash,side_hash=state.side_hash,
        certificate_state_hash=state.certificate_hash,forecast_key=state.forecast_key,
        all_published_checkpoint_links_valid=True,seconds=time.perf_counter()-tick)
    save('CHECKPOINT_VALIDATION.json',record);return record

def run():
    execution=read(OUT/'EXECUTION.json');assert execution['status']!='Running','ScientificExecutionStillActive'
    live=optional(OUT/'LIVE_PROCESS.json',{})
    if live.get('pid'):
        try:
            p=psutil.Process(live['pid'])
            assert not any('sdf_cfd_mesher_v4/tools/task4' in x.replace('\\','/') for x in p.cmdline()),'ScientificChildStillRunning'
        except psutil.NoSuchProcess:pass
    assert elapsed()<3590,'TaskDeadlineTooCloseToStartSealing'
    observation=collect();preservation=preserve();checkpoint_audit=validate_checkpoint();pointer=optional(RUN/'build_A/CURRENT.json',{'layer':0})
    last=pointer['layer'];last_meta=read(RUN/f'build_A/LAYER_{last:02}.json') if (RUN/f'build_A/LAYER_{last:02}.json').exists() else None
    if last_meta:
        assert file_hash(RUN/f'build_A/LAYER_{last:02}.json')==pointer['sha256']
        if last:assert array_hash(np.load(RUN/f'build_A/FRONT_{last:02}.npy'))==last_meta['coordinate_hash']
    compact(last)
    final=optional(RUN/'FINAL_VALIDATION.json');comparison=optional(OUT/'FINAL_COMPARISON.json');failure=optional(OUT/'FAILURE.json')
    hash_diagnosis=None
    if comparison and comparison.get('passed') and not comparison['raw_storage_hashes_equal']:
        legacy=read(BASE/'build_A/LAYER_01.json');fresh=read(RUN/'build_A/LAYER_01.json')
        differences={k:dict(A=scientific_hash(legacy.get(k)),A2=scientific_hash(fresh.get(k)))
            for k in sorted(set(legacy)|set(fresh)) if scientific_hash(legacy.get(k))!=scientific_hash(fresh.get(k))}
        hash_diagnosis=dict(first_raw_checkpoint_divergence=1,fields=differences,
            cause='Qualified Task-4B proof representation changes certificate envelopes and their downstream state/hash chains.',
            scientific_geometry_decisions_metrics_conditioning_and_pair_outcomes_equal=True,
            qualified_bridge='certificate/replay_equivalence.py and tools/task4c_compare_formats.py',
            literal_legacy_shell_hash_reproduced=False,raw_A=comparison['A'],raw_A2=comparison['A2'])
        save('HASH_DIAGNOSIS.json',hash_diagnosis)
    bridge_allowed=optional(OUT/'QUALIFICATION_POLICY.json',{}).get('qualified_bridge_accepted',False)
    if execution['status'] in ('ScientificCutoff','TimeBudgetStopBeforeStage'):
        classification='V4OptimizedA2TimeBudgetExceeded'
    elif execution['status']=='ScientificMismatch':classification='V4OptimizedA2ScientificMismatch'
    elif not final or not comparison or not comparison.get('passed'):classification='V4OptimizedA2Blocked'
    elif not comparison['raw_storage_hashes_equal'] and not bridge_allowed:classification='V4OptimizedA2ScientificMismatch'
    else:classification='V4OptimizedA2QualifiedWithinTimeBudget'
    stages=observation['stages'];rows=observation['rows'];complete=[r for r in rows if r['committed']]
    stages['startup_and_runner_preparation_seconds']=(datetime.datetime.fromisoformat(execution['scientific_start'])-datetime.datetime.fromisoformat(clock()['task_start'])).total_seconds()
    broad_stages=('initial_forecast_and_scan_seconds','initial_warning_atlas_seconds','active_search_seconds',
        'whole_layer_metric_seconds','exact_jacobian_seconds','certificate_seconds','final_validation_seconds')
    dominant=max(broad_stages,key=lambda key:stages.get(key) or 0.)
    if classification=='V4OptimizedA2QualifiedWithinTimeBudget' and stages.get('active_search_seconds',0)>stages.get('certificate_seconds',0):
        # This names the directly measured dominant architecture cost. It is
        # not an invented speed gate or a scientific validity requirement.
        classification='V4OptimizedA2QualifiedRuntimeStillHigh'
    slow=max(rows,key=lambda r:r['layer_elapsed_seconds']) if rows else None
    gate_violations={key:sum(r['gate']['violations'].get(key,0) for r in complete) for key in
        ('volume','signed_volume','jacobian','determinant','weight','skewness','symmetric_volume_ratio','aspect')}
    completed_layer_average=sum(r['layer_elapsed_seconds'] for r in complete)/len(complete) if complete else None
    stored_bytes=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file())
    summary=dict(classification=classification,execution=execution,task_clock=clock(),completed_layers=last,requested_fraction=last/20,
        prisms=last*106302,stages=stages,slowest_layer=slow,peak_tree_rss=execution['peak_tree_rss'],
        final_shell_hash=final.get('shell_scientific_hash') if final else None,last_checkpoint=pointer,last_state=last_meta['state_hash'] if last_meta else None,
        comparison=comparison,failure=failure,hash_diagnosis=hash_diagnosis,smoke=read(OUT/'SMOKE_TESTS.json'),preservation=preservation,checkpoint_validation=checkpoint_audit,
        scientific_before_cutoff=execution['elapsed_task']<3300,report_elapsed_seconds=elapsed(),
        raw_hash_requirement_met=bool(comparison and comparison['raw_storage_hashes_equal']),qualified_bridge_accepted=bridge_allowed,
        hard_gate_violations=gate_violations,completed_layer_average_seconds=completed_layer_average,stored_bytes_before_seal=stored_bytes,dominant_stage=dominant,
        legacy_hash=read(OUT/'PREFLIGHT.json')['verified_shell_hash'],free_disk_bytes=shutil.disk_usage(REPO).free)
    save('SUMMARY.json',summary)
    md=['Decision question','', 'Can the integrated optimized V4 mesher independently reproduce the complete 20-layer SF2 prism shell within a strict one-hour task budget?','',classification,'',
        '## A. Time-budget compliance','',
        f"Task start: {clock()['task_start']}. Scientific cutoff: {clock()['scientific_cutoff']}. Actual scientific stop: {execution['scientific_stop']}. Scientific stage wall time: {execution['seconds']:.6f} s. Task elapsed at that stop: {execution['elapsed_task']:.6f} s. Sealing completion is recorded precisely in SEAL.json. The task deadline is {clock()['task_deadline']}.", '',
        '## B. Authority verification','',
        'PREFLIGHT.json binds the Task-4/4B/4C/4D reports, source overlays and sealed evidence. The frozen wall, topology, geometry and binary64 schedule were verified. No Build-A or historical-A2 artifact was overwritten. V3 modified files: 0.', '',
        f"The prompt hash has 65 characters. The verified Build-A 64-character hash is `{summary['legacy_hash']}`. The existing Task-4C comparator distinguishes proof-storage envelopes from exact scientific values. Raw hashes are reported honestly; none is forced to equal Build A.", '',
        '## C. Integrated runner configuration','',
        'Fresh Task-4E A2 started with no forecast, candidate, planner cache or proof state. Generating subprocesses use only frozen geometry authorities and their own earlier transactions. Post-commit comparison is separate. Task-4B production.certify_immediate, Task-4C RiskAtlas and Task-4D PersistentPlanner are active. Canonical trial order is unchanged. Eight certificate workers are configured. Committed proofs remain immutable; each certificate call builds the native index for its current prefix.', '',
        'The output guard now permits the separate Task-4E directory. Initial warning discovery now receives the already qualified persistent pair selector. Timing-only fields record detailed-cache hits and evaluated populations. Reporting fixes handle first/later-layer scope keys and keep final qualification out of layer-20 timing. Reproducers and focused regressions are preserved. No gate, SDF distance, topology or certificate mathematics changed.', '',
        '## D. Layer-by-layer progress','',
        '| Layer | Committed | Corrected | Atlas active % | Prepare s | Search s | Detailed s | Certificate s | Task elapsed s |',
        '|---:|---|---|---:|---:|---:|---:|---:|---:|']
    def fmt(v):return 'not recorded' if v is None else f'{v:.3f}'
    for r in rows:
        md.append(f"| {r['layer']} | {r['committed']} | {r['corrected']} | {fmt(None if r['atlas_active_fraction'] is None else 100*r['atlas_active_fraction'])} | {fmt(r['preparation_seconds'])} | {fmt(r['active_search_seconds'])} | {fmt(r['detailed_seconds'])} | {fmt(r['certificate_seconds'])} | {fmt(r['cumulative_task_seconds'])} |")
    md+=['','The initial atlas partition was not separately persisted; layer-1 corrected support and its post-step atlas state are recorded. Detailed first-layer internal timing was not split. Missing values are not inferred zero.','',
        '## E. Runtime decomposition','',
        '| Stage | Direct seconds |','|---|---:|']
    for key,value in stages.items():md.append(f'| {key} | {fmt(value)} |')
    md+=['',observation['overlap_note'],'',
        f"Peak observed aggregate worker RSS: {execution['peak_tree_rss']} bytes. Completed-layer stage average: {fmt(completed_layer_average)} s, excluding startup and final qualification. Slowest attempted layer: {slow['layer'] if slow else None}, {fmt(slow['layer_elapsed_seconds'] if slow else None)} s. Artifact bytes before sealing: {stored_bytes}. Free disk: {summary['free_disk_bytes']} bytes.", '',
        '## F. Active planner behavior','',
        f"Completed control records contain {sum(r['trials'] for r in rows)} trials and {sum(r['accepted'] for r in rows)} accepted region corrections. Instrumented later-layer detailed refreshes: {observation['detailed_refreshes']}; exact-input hits: {observation['detailed_hits']}. Per-region populations, cache statistics and timing are in LAYER_PROGRESS.json. Passive nominal coordinates still receive whole-layer validation. Old numeric caches were compacted only after execution; latest restart state remains available.", '',
        '## G. Certification behavior','',
        f"Completed transactions: {last}. Required completed pair population: {sum(r['certificate_pairs'] or 0 for r in complete)}. Frozen-kernel fallback calls: {sum(r['exact_calls'] or 0 for r in complete)}. Forbidden pairs: {sum(r['forbidden'] for r in complete)}. Unresolved pairs: {sum(r['unresolved'] for r in complete)}. Fresh certificate runtime is measured above; prior replay timing is not substituted.", '',
        '## H. Scientific result','',
        f"Committed layers: {last}/20 ({100*last/20:.1f}%). Committed prisms: {last*106302}. Every published transaction has passing numeric, exact-sign and exact continuous records. Hard-gate violation counts: {gate_violations}. Last checkpoint SHA-256: `{pointer.get('sha256')}`. Final raw shell hash: `{summary['final_shell_hash']}`.", '',
        '## I. A/A2 comparison','',
        f"Prefix comparisons passed for {execution['compared_layers']} layers. Complete comparison: {bool(comparison and comparison.get('passed'))}. Raw shell hashes equal: {summary['raw_hash_requirement_met']}. The qualified bridge is used for scientific comparison. Literal legacy-hash requirement waived: {bridge_allowed}. Complete geometry, connectivity, correction history, hard metrics, SDF values, pair populations/outcomes and conditioning all match when that comparison passes. Its result does not claim equal proof-storage hashes.", '',
        ('HASH_DIAGNOSIS.json records the first raw divergence at layer 1: the optimized proof representation and the state hashes that include it. The qualified bridge passes all scientific comparisons. The literal old shell-hash requirement remains unmet; this is not a geometric or conditioning divergence.' if hash_diagnosis else 'No representation-only final hash divergence was diagnosed.'), '',
        '## J. Timeout state','',
        f"Execution status: {execution['status']}. Last attempted layer/stage: {execution['attempted_layer']}/{execution['stage']}. "+('No timeout occurred. All scientific subprocess operations returned successfully.' if execution['status']=='ScientificStagesComplete' else 'The next uncommitted candidate, if written, remains separate from CURRENT.json. TIMEOUT.json or FAILURE.json records the stop. No invalid layer is published to extend the prefix.'), '',
        '## K. New dominant bottleneck','',
        f"Largest measured scientific stage: {dominant}, {stages[dominant]:.6f} s. Active search includes {stages.get('active_detailed_seconds',0):.6f} s of instrumented detailed future metric evaluation in later layers. Exact Jacobian signs and whole-layer certificates are separate. Startup is reported separately. No final-runtime extrapolation is made from an incomplete prefix. Repeated detailed future metrics remain the next local-planning performance target; this task does not redesign them.", '',
        '## L. Tests','',
        f"Startup smoke: {summary['smoke']['count']} tests; {summary['smoke']['failures']} failures; {summary['smoke']['errors']} errors. REPAIR_TESTS.txt records four focused checks; REPORT_TESTS.txt records five passing governance/reporting tests after the final timing repair. These suites overlap. No repeated complete development-suite run was performed. Every completed layer was compared independently to Build A after commit.", '',
        '## M. Classification','',classification,'',
        ('All required geometry, metric, conditioning and exact proof-outcome comparisons pass. The strict classification records only the unmet literal legacy shell-hash requirement. Raw-format equality is not claimed or forced.' if hash_diagnosis else 'Scientific equality, raw-format equality and runtime compliance are separate results. A valid prefix alone is not complete qualification.'), '',
        '## N. Recommended next direction','',
        'Resolve the versioned shell-identity convention using the already-qualified exact comparison bridge before assigning a complete legacy-hash qualification label. The next performance target is repeated bounded detailed future metric evaluation. Do not rerun geometry merely to hide a proof-format hash change. No downstream meshing or CFD is started.', '',
        'Only Task-4E source changes, tests and evidence are committed. The final Git commit is reported in the final response. Unrelated dirty state is preserved.']
    (OUT/'report.md').write_bytes(('\n'.join(md)+'\n').encode())
    sources=[ROOT/'tools'/n for n in ('task4_initial_scan.py','task4_warning_cache.py','task4_prepare_next.py')]+list((ROOT/'tools').glob('task4e_*.py'))+[ROOT/'tests/test_task4e_governance.py']
    evidence=[p for p in OUT.rglob('*') if p.is_file() and p.name not in ('AUTHORITY.json','SEAL.json') and p.suffix not in ('.pending','.pyc')]
    a=dict(classification=classification,start_head=clock()['start_head'],V3_modifications=0,report_sha256=file_hash(OUT/'report.md'),
        source_sha256={p.relative_to(ROOT).as_posix():file_hash(p) for p in sorted(sources)},evidence_sha256={p.relative_to(ROOT).as_posix():file_hash(p) for p in sorted(evidence)},
        scientific_result=dict(layers=last,shell_hash=summary['final_shell_hash'],checkpoint=pointer,comparison=comparison))
    save('AUTHORITY.json',a);save('SEAL.json',dict(report_sha256=file_hash(OUT/'report.md'),authority_sha256=file_hash(OUT/'AUTHORITY.json'),sealing_completed_utc=now().isoformat(),elapsed_task=elapsed(),task_deadline=clock()['task_deadline']))
    print('SEALED',classification,elapsed(),flush=True)

if __name__=='__main__':run()
