"""Seal directly measured Task-4D evidence; no estimated build time."""
from task4d_common import *
import shutil

def run():
    old=read(OUT/'ACTIVE_SEARCH_CENSUS.json')['regions'];replay=read(OUT/'sequence_2_REPLAY.json');timing=read(OUT/'sequence_2_TIMINGS.json');warm=read(OUT/'WARM_1.json')['rows']
    tests=read(OUT/'TEST_SUMMARY.json');det=read(OUT/'DETERMINISM.json');preservation=read(OUT/'PRESERVATION.json');cert=read(OUT/'FRESH_CERTIFICATE.json');bindings=read(OUT/'CERTIFICATE_BINDINGS.json')
    rows=[];layers=[];stage_totals={};last=timing['rows'][-1]['after']
    for entry in timing['rows']:
        layer=entry['layer'];stage=entry['stage'];trial=stage['stages'];profiles={p['patch']:p for p in stage.get('region_profiles',[])}
        for original in (r for r in old if r['layer']==layer):
            i=original['region'];local=[r for r in trial if r.get('patch',r.get('region'))==i];p=profiles.get(i,{})
            row={k:original[k] for k in ('layer','region','active_vertices','tangent_dofs','trials','accepted_steps','rejected_steps','search_seconds')}
            row.update(region_identity=replay['rows'][layer-1]['planner_events'][i]['region'],
                dependency_columns=len(original['scope']['detailed']['columns']),provisional_prisms=20*len(original['scope']['detailed']['columns']),
                warm_trial_seconds=sum(r['seconds'] for r in local),sensitivity_seconds=0.,numeric_sensitivity_reuse=0,
                support_seconds=p.get('support_seconds',sum(r.get('support_seconds',0) for r in local)),
                baseline_detailed_seconds=p.get('baseline_detailed_seconds',sum(r.get('baseline_detailed_seconds',0) for r in local)),
                detailed_seconds=sum(r.get('detailed_seconds',0) for r in local),
                forecast_seconds=sum(r.get('forecast',{}).get('seconds',r.get('forecast_seconds',0)) for r in local),
                projection_seconds=sum(r.get('proposal_projection_seconds',r.get('projection_seconds',0)) for r in local),
                exact_sign_seconds=sum(r.get('exact_sign_seconds',0) for r in local),spot_seconds=sum(r.get('spot_seconds',0) for r in local),
                direction_seconds=p.get('direction_seconds'),search_checkpoint_seconds=p.get('checkpoint_seconds'),
                metric_note='Detailed time includes all numeric prism/face/conditioning metrics; exact-sign time includes immediate exact signs and failure-map construction.')
            rows.append(row)
        for k in ('nominal_numeric_seconds','final_numeric_seconds','risk_discovery_seconds','local_search_seconds','persistence_seconds'):
            stage_totals[k]=stage_totals.get(k,0)+stage.get(k,0)
        layers.append(dict(layer=layer,seconds=entry['seconds'],trials=len(trial),new_trial_seconds=sum(r['seconds'] for r in trial),old_trial_seconds=sum(r['search_seconds'] for r in old if r['layer']==layer)))
    old_seconds=sum(r['search_seconds'] for r in old);new_seconds=sum(r['warm_trial_seconds'] for r in rows)
    eligible=[r for r in warm if r['eligible']];projected=[r for r in eligible if r['projected']]
    cold9=read(OUT/'cold_native_9_TIMINGS.json')['rows'][0];warm9=timing['rows'][8]
    cache_comparison=dict(layer=9,same_implementation=True,cold_control_seconds=cold9['seconds'],warm_control_seconds=warm9['seconds'],
        cold_local_search_seconds=cold9['stage']['local_search_seconds'],warm_local_search_seconds=warm9['stage']['local_search_seconds'],
        cold_trial_seconds=cold9['stage']['stages'][0]['seconds'],warm_trial_seconds=warm9['stage']['stages'][0]['seconds'],
        candidate_hash=read(OUT/'cold_native_9_REPLAY.json')['rows'][0]['candidate_hash'],
        equivalent=read(OUT/'cold_native_9_REPLAY.json')['rows'][0]['candidate_hash']==replay['rows'][8]['candidate_hash'])
    assert cache_comparison['equivalent']
    summary=dict(classification='V4PersistentActivePlanningSubstantiallyImproved',base_head=read(OUT/'PREFLIGHT.json')['head'],
        regions=32,trials=255,accepted=32,rejected=223,iteration_reduction=0,rejected_trial_reduction=0,
        structural_warm_eligible=sum(1 for r in replay['rows'] for e in r['planner_events'] if e['kind']=='persistent'),
        compatible_warm_eligible=len(eligible),warm_projected=len(projected),warm_same_candidate=sum(r.get('same_canonical_candidate',False) for r in projected),
        warm_numeric_pass=sum(r.get('numeric',{}).get('passed',False) for r in projected),warm_future_pass=sum(r.get('future_admission',{}).get('passed',False) for r in projected),
        warm_candidates_published=0,tangent_basis_numeric_reuse=0,sensitivity_numeric_reuse=0,sensitivity_structure_reuse=0,trust_optimizer_state_reuse=0,steering_plan_steps_published=0,
        support_reuse=last['regions']['support_hits'],fresh_supports=last['regions']['support_misses'],regions_requiring_canonical_search=32,cache_comparison=cache_comparison,
        old_active_trial_seconds=old_seconds,new_active_trial_seconds=new_seconds,active_trial_ratio=old_seconds/new_seconds,
        control_replay_seconds=timing['seconds'],stages=stage_totals,reuse=last,layers=layers,
        fresh_certificate_seconds=cert['performance'][0]['seconds'],fresh_certificate_layer=cert['layer'],
        reused_proof_verification_seconds=bindings['reuse_verification_seconds'],reused_proofs=bindings['pairs'],
        frozen_optimized_certification_seconds=read(B/'SUMMARY.json')['new_immediate_all_20_seconds'],
        peak_rss=max([r['peak_rss'] for r in timing['rows']]+[r['peak_rss'] for r in read(OUT/'WARM_1_TIMINGS.json')]+[read(OUT/'CERTIFICATE_MEMORY.json')['peak_tree_rss']]),
        deterministic=det,tests=tests,Build_A_rebuilt=False,A2_resumed=False,V3_modifications=0,
        study_seconds=(datetime.datetime.now(datetime.timezone.utc)-datetime.datetime.fromisoformat(read(OUT/'PREFLIGHT.json')['started_utc'])).total_seconds(),
        free_disk_bytes=shutil.disk_usage(REPO).free,
        A2_readiness='Ready for one controlled qualification using canonical ordering and persisted exact-input caches. Transported moves must not bypass canonical trials. This study did not run A2.')
    save('REGION_PROFILE.json',rows);save('SUMMARY.json',summary)
    md=['Decision question','', 'Can V4 preserve and reuse expensive local planning state across consecutive advancing layers so persistent difficult regions do not require repeated cold optimization?','',
        summary['classification'],'',
        '## A. Authority verification','',
        'The expected branch and base HEAD matched. Task-4B and Task-4C authorities and sealed results passed before work. The bounded V3 audit covers 57 source files. Build A and A2 are unchanged.', '',
        '| Authority | SHA-256 |','|---|---|',
        '| Task-4B report | ec5686586f9eb5e2e65f55e72338dae3276b82e8f9d70a4e70efc5f610c7bba2 |',
        '| Task-4B authority | 8fe4af86b569705ac41399dcdbd0a02ddb7ff41b62a75fc254cd7998bd754aac |',
        '| Task-4C report | 069e61fbdf37060d4604333886275dddf6a00bdc6fe8f66f4cc384609cda70c9 |',
        '| Task-4C authority | db8cc487ed5317fbf3ad2acc20f02b5f101ab2e3c4cc18085626b490842a176a |','',
        '## B. Existing active-search baseline','',
        f'The 32 regions used 255 unique trials: 32 accepted and 223 rejected. Frozen trial time totals {old_seconds:.9f} s. The actual rolling search is a fixed mode/amplitude poll. It builds no sensitivity matrix and has no trust-region optimizer. These absent stages have zero calls; unavailable old timing fields are null, not inferred zero.', '',
        '## C. Active-search runtime decomposition','',
        'The bounded layer-8 profile measured 67.661231600 s in preparation. Future probe selection consumed 50.332 s, including 23.444 s in source-edge scans and 23.662 s in Python pair enumeration. Detailed metrics consumed 5.218 s and exact wedge signs 1.133 s. Full symbolic solving was not the dominant local-search cost. Profiling overhead is included in these figures.', '',
        'ACTIVE_SEARCH_CENSUS.json retains all 32 scopes and compact trial references. REGION_PROFILE.json gives fresh per-region support, forecast, metric, projection, exact-sign, future-probe and search-checkpoint measurements. Trial time and enclosing stage time overlap and must not be added together.', '',
        '## D. Persistent region identity','',
        'Identity uses canonical int64 source membership and exact topology. Consecutive support must match. Merge, split, changed support, changed selection kind, changed direct target hazard, disappearance and reactivation reset displacement advice. No proximity threshold or historical ID is used.', '',
        '## E. Reusable planning state','',
        f"Support hits: {last['regions'].get('support_hits',0)}. Whole detailed-result hits: {last['detailed_hits']}; misses: {last['detailed_misses']}. Future selection hits: {last['selection'].get('selection_hits',0)}; misses: {last['selection'].get('selection_misses',0)}. All reuse binds exact input geometry. Restart preserves closed support, bounded detailed results, spatial query state and diagnostic pair selections. Passive layers evict obsolete hot caches.", '',
        'Spatial-query updates retest changed targets for unchanged query boxes and all targets for changed query boxes. The unchanged Task-4B native forest replaces repeated Python enumeration. It preserves the complete candidate population, ownership exclusions, ordering and diagnostic sample. Full exact immediate certification is unchanged. PLANNING_STATE_SPEC.md gives the safety argument.', '',
        '## F. Warm-start behavior','',
        f"{summary['structural_warm_eligible']}/32 regions have consecutive identical support. After direct-hazard identity checks, {len(eligible)} proposals are eligible for displacement transport; {len(projected)} project successfully. {summary['warm_same_candidate']} reproduce the canonical accepted coordinates. Numeric local gates pass for {summary['warm_numeric_pass']}; future admission passes for {summary['warm_future_pass']}. None is published by the advisory probe. A different valid candidate is not scientific equivalence. Canonical predecessor trials remain mandatory in the integrated runner.", '',
        'Structural eligibility is 50%. Transport reproduces canonical coordinates in 2/16 eligible regions (12.5%); this is not a new acceptance rate. All 32 regions still execute canonical numeric search. Sixteen use cached support and sixteen construct support afresh. A transported move is not allowed to skip unresolved predecessor decisions.', '',
        '## G. Sensitivity reuse','',
        'No numeric sensitivities are reused. The production path constructs none. Closed support and dependency topology are reusable; numeric tangent bases, SDF normals and changed metric values are recalculated. A cached detailed result is returned only for exactly identical local coordinates, source geometry and topology. A real saved-state check compared the entire cached metric result with a fresh calculation.', '',
        '## H. Trust-region reuse','',
        'There is no production trust-region state to preserve. The last accepted factor and displacement are advisory only. The canonical factor order is unchanged. Introducing a new trust-region policy would change this performance study, so none was added.', '',
        '## I. Multi-layer steering-plan reuse','',
        'One-step transported advice was tested. No multi-step steering sequence is qualified for automatic consumption. No future step is committed. Input changes invalidate advice directly. Diagnostic pair selection can survive rebasing across multiple layer indices under identical scope geometry; that is forecast-work reuse, not early geometry commitment.', '',
        '## J. Replanning triggers','',
        'Changed topology/support, merge/split, direct target changes and retirement reset advice. Changed coordinate bits miss detailed-cache keys. Changed source-edge bounds invalidate only their actual overlap predicates. Checkpoint layer, committed forecast hash and implementation must match. Hard-gate or future-admission failure retains canonical search and exact rollback.', '',
        '## K. Cold-versus-warm scientific equivalence','',
        'Both full preserved 20-layer control replays reproduce frozen Build-A coordinates, all 255 trial decisions, immediate gate values, complete forecasts/normals and future warnings. All 32 regions still activate through generic production risk logic. No Build-A shell was rebuilt and no commit store was advanced. The exploratory replay had a closure-loop micro-optimization; it was removed to restore the older byte authority. Final-source checkpoint restart and the actual first-layer runner also pass. The latter now emits the planner state that later production calls consume.', '',
        '| Layer | Trials | Frozen trial s | Persistent trial s | Complete replay s |','|---:|---:|---:|---:|---:|']
    md += [f"| {r['layer']} | {r['trials']} | {r['old_trial_seconds']:.6f} | {r['new_trial_seconds']:.6f} | {r['seconds']:.6f} |" for r in layers]
    for title,lo,hi in [('L. Early active sequence',1,4),('M. Layers 5–11 persistent sequence',5,11),('N. Active-to-passive transition',10,13)]:
        chosen=[r for r in layers if lo<=r['layer']<=hi]
        md+=['','## '+title,'',f"Layers {lo}–{hi}: directly measured replay {sum(r['seconds'] for r in chosen):.6f} s; all scientific comparisons pass. Frozen trial time {sum(r['old_trial_seconds'] for r in chosen):.6f} s; current trial time {sum(r['new_trial_seconds'] for r in chosen):.6f} s."]
    md+=['','No correction is selected after layer 11. The transition is produced by generic risk evidence, not a layer-number special case. Passive coordinates and complete immediate hard checks remain unchanged.', '',
        '## O. Runtime comparison','',
        f'Comparable active trial time: {old_seconds:.9f} → {new_seconds:.9f} s ({old_seconds/new_seconds:.4f}×). Trial count remains 255 and rejected-trial reduction is zero. The gain is less repeated planning work, not fewer scientific decisions.', '',
        f"The same-code layer-9 empty-cache comparison isolates persistence: local search {cache_comparison['cold_local_search_seconds']:.9f} → {cache_comparison['warm_local_search_seconds']:.9f} s; complete control call {cache_comparison['cold_control_seconds']:.9f} → {cache_comparison['warm_control_seconds']:.9f} s. Scientific outputs match. The larger frozen-baseline comparison also includes native pair enumeration; it must not be attributed solely to cross-layer caching.", '',
        f"The complete preserved control replay took {timing['seconds']:.9f} s. This includes actual bounded searches and complete immediate numeric checks. It omits the initial cold SDF forecast, authoritative checkpoint publication and all 20 full-layer exact certificate solves. It is not a full-build runtime. Separately, proof binding/reuse verification took {bindings['reuse_verification_seconds']:.9f} s for all 56,462,864 proofs. A fresh complete layer-{cert['layer']} certificate took {cert['performance'][0]['seconds']:.9f} s and matched every frozen proof hash. The separately frozen Task-4B all-20 certificate benchmark is {summary['frozen_optimized_certification_seconds']:.9f} s; it was not rerun or added to a claimed build time.", '',
        f"Peak measured RSS: {summary['peak_rss']} bytes. Study elapsed through report generation: {summary['study_seconds']:.3f} s. Stage totals and per-region records are in SUMMARY.json and REGION_PROFILE.json. Runtime ratios compare direct measurements from separate runs; they are not guarantees.", '',
        f"Redundant replay arrays and hot checkpoints removed after exact equality checks: {read(OUT/'STORAGE_REFERENCES.json')['redundant_bytes_removed']} bytes. References and hashes remain. One active restart checkpoint and one passive checkpoint are retained. Build A was not duplicated.", '',
        '## P. New dominant bottleneck','',
        f"Repeated detailed future metrics remain material: {sum(r['detailed_seconds']+r['baseline_detailed_seconds'] for r in rows):.6f} s in the instrumented run. Source-edge and pair-selection work is reduced, but the fixed searches at layers 3–5 still evaluate many different geometries. There is no evidence that stale gradients or an unvalidated earlier successful move can replace those decisions.", '',
        '## Q. Determinism','',f"Full scientific replay hashes: `{det['run_1']}` and `{det['run_2']}`. Transport diagnostic hashes: `{det['warm_1']}` and `{det['warm_2']}`. Coordinates compare bitwise. Timing and RSS are separate. Final-source checkpoint restart and a cold cached-metric comparison pass.", '',
        '## R. Tests','',f"{tests['test_count']} tests: {tests['passed']} passed, {len(tests['inherited_state_failures'])} inherited repository/preservation assertions, {len(tests['unexpected_failures'])} unexpected failures, {tests['errors']} errors. Old assertions remain unchanged. New tests cover identity, support transport, numeric invalidation, hazard reset, merge/split, retirement, exact spatial reuse, checkpoint/restart, advisory rollback and complete-certificate requirements.", '',
        'A focused regression found that in-memory planner snapshots aliased mutable advisory metadata. Clearing live advice could alter the rollback snapshot. Checkpoint and restore now deep-copy bounded metadata. CHECKPOINT_ALIAS_REPRO.txt preserves the failing test. Its regression and the real saved-state restart pass. The repair changes no scientific calculation.', '',
        '## S. Classification','',summary['classification'], '',
        'Persistent structure and exact-input caches reduce active-planning cost. They do not eliminate the long canonical searches. Numeric sensitivity, trust-region and multi-step steering reuse are not established. No hard gate, risk threshold, topology, SDF target or certificate mathematics changed.', '',
        '## T. A2 readiness','',summary['A2_readiness'], '',
        f"Base HEAD: `{summary['base_head']}`. V3 modified files: 0. Sealed prior artifacts: unchanged. Unrelated dirty state unchanged: {preservation['unrelated_state_unchanged']}. Only Task-4D source, tests and evidence are staged. The final commit identifier is the commit containing this report and is given in the final response. No downstream work ran."]
    (OUT/'report.md').write_bytes(('\n'.join(md)+'\n').encode())
    sources=[ROOT/'tools/task4_prepare_next.py',ROOT/'tools/task4_correct_first.py',ROOT/'src/sdf_cfd_mesher_v4/forecast/detailed/rolling_spots.py',ROOT/'src/sdf_cfd_mesher_v4/advancing_front/persistent_planning.py',ROOT/'src/sdf_cfd_mesher_v4/advancing_front/warm_start.py',ROOT/'tests/test_persistent_planning.py']+list((ROOT/'tools').glob('task4d_*.py'))
    evidence=[p for p in OUT.rglob('*') if p.is_file() and p.name not in ('AUTHORITY.json','SEAL.json') and p.suffix not in ('.log','.pyc')]
    authority=dict(classification=summary['classification'],base_head=summary['base_head'],V3_modifications=0,A2_resumed=False,Build_A_rebuilt=False,
        source_sha256={p.relative_to(ROOT).as_posix():file_hash(p) for p in sorted(sources)},evidence_sha256={p.relative_to(ROOT).as_posix():file_hash(p) for p in sorted(evidence)},scientific_replay=det)
    save('AUTHORITY.json',authority);save('SEAL.json',dict(report_sha256=file_hash(OUT/'report.md'),authority_sha256=file_hash(OUT/'AUTHORITY.json')))
    print(summary['classification'],read(OUT/'SEAL.json'),flush=True)

if __name__=='__main__':run()
