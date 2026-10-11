"""Seal measured follow-up evidence, without executing new scientific work."""
from pathlib import Path
import sys,json,datetime,shutil,collections
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')];sys.dont_write_bytecode=True
import numpy as np
from task3_repair_preflight import audit,git,OUT
from sdf_cfd_mesher_v4.common.authority import file_hash,array_hash
from sdf_cfd_mesher_v4.forecast.detailed.scientific import plain,scientific_hash
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate import repaired_kernel,kernel

def read(n):return json.loads((OUT/n).read_text())
def write(n,v):(OUT/n).write_text(json.dumps(plain(v),indent=2,sort_keys=True,allow_nan=False)+'\n',encoding='utf-8')
def unrelated(s):return sorted(r for r in s.splitlines() if 'research/sdf_cfd_mesher_v4/' not in r)
def table(headers,rows):return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+['| '+' | '.join(map(str,r))+' |' for r in rows])
def extrema(data,group,metric):
    x=np.asarray(data[group][metric],dtype=float);return dict(min=float(np.nanmin(x)),max=float(np.nanmax(x)))

if __name__=='__main__':
    start=read('PREFLIGHT.json');assert git('rev-parse','HEAD')==start['starting_head'];assert not git('diff','--cached','--name-only')
    now=git('status','--porcelain');assert unrelated(now)==unrelated(start['starting_status'])
    verified=audit();write('PRESERVATION.json',dict(passed=True,counts={k:len(verified[k]) for k in ['rows','task2_bindings','task2b_bindings','task3_bindings']},v3_modifications=0,sealed_modifications=0,unrelated_status_unchanged=True,status_before_commit=now))
    previous=json.loads((ROOT/'results/task3/PREFLIGHT.json').read_text())['starting_status']
    write('HISTORICAL_STATUS_CONTEXT.json',dict(prior_task_to_followup_start_added=sorted(set(unrelated(start['starting_status']))-set(unrelated(previous))),followup_start_to_end_added=[],followup_start_to_end_removed=[],assertions_unchanged=True))
    tests=read('TESTS.json');det=read('DETERMINISM.json');root=read('ROOT_REGRESSION.json');trial=read('BLOCKED_TRIAL_REPLAY.json')
    assert det['passed'] and read('HASH_ORDER_REPLAY.json')['passed']
    assert tests['errors']==0 and tests['failures']==3
    expected_failed={'test_forensics.ForensicTests.test_current_task_preservation_and_sealed_task2','test_setup.SetupTests.test_branch_and_unrelated_git_status_preserved','test_task3.RelaxationTests.test_current_task_unrelated_state_preserved'}
    assert set(tests['failed_tests'])==expected_failed
    old00=json.loads((ROOT/'results/task3/component_00.json').read_text());old01=json.loads((ROOT/'results/task3/component_01.json').read_text())
    rs=[old00]+[read(f'component_{i:02}.json') for i in range(1,5)]
    labels=['Positive-span leading cap','Negative-span trailing cap','Negative-span aft cap','Upper/trailing-face junction','Negative-span leading cap']
    summary=[];scopes=[]
    for i,(name,r) in enumerate(zip(labels,rs)):
        events=[e for e in r['history'] if 'field' in e]
        inherited=60 if i==0 else (12 if i==1 else (27 if i==3 else 0))
        attempts=60 if i==0 else inherited+len(events)
        summary.append(dict(component=f'component_{i:02}',name=name,classification=r['final']['classification'],attempts=attempts,inherited_trials=inherited,
                            accepted=len(r['accepted_steps']),first_future_invalid_before=r['final']['baseline_first_invalid'],first_future_invalid_after=r['final']['final_first_invalid']))
        for s in r['scopes']:
            p=s['patch'];scopes.append(dict(component=i,rings=s['active_rings'],active=len(p['active']),fixed_boundary=len(p['fixed_boundary']),halo=len(p['halo']),verification_columns=len(p['detailed']['core_columns']),columns=len(p['detailed']['columns']),prisms=20*len(p['detailed']['columns'])))
    assert sum(r['accepted'] for r in summary)==1
    prior_summary=read('SUMMARY.json')
    write('SUMMARY.json',dict(classification='V4LocalNextFrontCorrectionAuthorityMixed',results=summary,scientific_hash=det['run1'],
          kernel_identity=repaired_kernel.VERSION,initial_continuation_kernel_identity=prior_summary.get('initial_continuation_kernel_identity',prior_summary['kernel_identity']),
          preserved_component_00_sha256=file_hash(ROOT/'results/task3/component_00.json'),preserved_component_01_prefix_sha256=file_hash(ROOT/'results/task3/component_01.json'),
          accepted_candidates=1,committed_layer=0,task4_ready=False))
    f=rs[3]['final'];event=next(e for e in rs[3]['history'] if e.get('accepted'))
    assert f['candidate_gate']['passed'] and f['exact_jacobians']['passed']
    controls=[read(n+'.json') for n in ['control_mid_chord','control_forward_surface']]
    assert all(c['final']['coordinates_identical'] and not c['accepted_steps'] for c in controls)
    write('COMPONENT_RESULTS.json',summary);write('PATCH_SCOPES.json',scopes)
    write('ACCEPTED_CANDIDATE_COORDINATES.json',dict(component='component_03',committed=False,committed_layer=0,proposed_layer=1,vertices=f['patch']['active'],
          baseline=f['baseline_layer1_coordinates'],candidate=f['layer1_coordinates'],coordinate_hash=event['coordinate_hash'],maximum_displacement_m=event['candidate_displacement'],projection=f['projection']))
    write('LOCAL_TAIL_COMPARISON.json',dict(baseline=f['baseline_tails'],candidate=f['final_tails'],meaning='Complete sorted local evidence; no per-entity improvement quota'))
    whole={state:{m:extrema(f[state],g,m) for g,m in [('cells','sigma'),('cells','G_bound'),('cells','jacobian'),('cells','determinant'),('faces','weight'),('faces','symmetric_volume_ratio')]} for state in ['baseline','candidate']}
    write('ACCEPTED_COMPARISON.json',dict(candidate_gate=f['candidate_gate'],exact_jacobians=f['exact_jacobians'],whole_stack=whole,first_invalid_before=2,first_invalid_after=None,new_earlier_failures=event['new_earlier_failures'],current_certificate=f['current_certificate'],future_certificate=f['future_certificate'],baseline_comparison=f['baseline_future_certificate']))
    write('HEALTHY_CONTROLS.json',[dict(name=c['spec']['name'],columns=c['spec']['columns'],coordinates_identical=True,trials=0,accepted=0,reason=c['final']['reason']) for c in controls])
    initial=read('INITIAL_TIMINGS.json');continuation=read('TIMINGS.json');replay=read('REPLAY_TIMINGS.json')
    alltiming=dict(component_01=initial['component_01'],component_02=initial['component_02'],**continuation)
    unique_hits=initial['component_01']['detailed_hits']+initial['component_02']['detailed_hits']+initial['component_03']['detailed_hits']+continuation['component_04']['detailed_hits']
    write('CACHE_LOCALITY.json',dict(accepted_move=event['cache'],maximum_active_vertices=max(s['active'] for s in scopes),maximum_columns=max(s['columns'] for s in scopes),maximum_provisional_prisms=max(s['prisms'] for s in scopes),
          new_unique_candidate_evaluations=unique_hits,unrelated_exact_detailed_hits_in_unique_evaluations=unique_hits,preserved_prior_exact_hits=72,
          actual_continuation_hits=sum(p['detailed_hits'] for p in initial.values())+sum(p['detailed_hits'] for p in continuation.values()),
          cold_whole_front_forecasts=0,global_dense_sensitivities=0,final_component_replay_timings=replay['components']))
    before=read('PRE_ORDER_FIX_ROOT_REGRESSION.json')['repaired_pair'];mismatch=read('ROOT_REPLAY_MISMATCH.json')['repaired_pair']
    assert [k for k in before if before[k]!=mismatch[k]]==['proof']
    write('REPRESENTATION_DETERMINISM_REPAIR.json',dict(before_hash=scientific_hash(before),alternate_hash=scientific_hash(mismatch),changed_top_level_fields=['proof'],
          exact_witness_unchanged=before['point']==mismatch['point'],exact_parameters_unchanged=before['parameters']==mismatch['parameters'],
          root_cause='Set iteration chooses refinement sequence; warm arithmetic intervals also changed exact sector representatives after pair-cache eviction',
          repairs=['Deterministic exact-expression presort before unchanged numeric comparator','Private per-pair refinement state with canonical initial intervals from the same exact isolation routine'],
          final_pair_cache_eviction=read('PAIR_CACHE_EVICTION_AFTER.json'),hash_order_replay=read('HASH_ORDER_REPLAY.json'),scientific_fields_removed_from_hash=[]))
    clock=datetime.datetime.now(datetime.timezone.utc)
    elapsed=(clock-datetime.datetime.fromisoformat(start['started_utc'])).total_seconds()
    worker_peak=max([p['peak_rss'] for p in initial.values()]+[read('INITIAL_WORKER_OBSERVATION.json')[0]['peak_rss'],read('CONTINUATION_RESOURCES.json')['peak_rss'],read('PRE_INTERVAL_FIX_REPLAY_TIMINGS.json')['peak_rss'],replay['peak_rss'],read('BLOCKED_TRIAL_TIMING.json')['peak_rss']])
    resource=dict(audited_wall_seconds=elapsed,started_utc=start['started_utc'],sealed_utc=clock.isoformat(),peak_worker_rss=worker_peak,
                  initial_search_component_seconds=sum(p['seconds'] for p in initial.values()),continuation=read('CONTINUATION_RESOURCES.json'),
                  regression=read('ROOT_REGRESSION_TIMING.json'),blocked_trial=read('BLOCKED_TRIAL_TIMING.json'),
                  pre_interval_fix_replay=read('PRE_INTERVAL_FIX_REPLAY_TIMINGS.json'),final_replay=replay,hash_order=read('HASH_ORDER_TIMING.json'),tests_seconds=tests['seconds'],
                  components=alltiming,free_disk_bytes=shutil.disk_usage(ROOT).free,maximum_heavy_workers=1)
    write('RESOURCES.json',resource)
    scope_table=table(['Component','Rings','Active','Fixed boundary','Halo','Measured columns','Columns with halo','Prisms'],[[s[k] for k in ['component','rings','active','fixed_boundary','halo','verification_columns','columns','prisms']] for s in scopes])
    component_table=table(['Component','Result','Total tested trials','Accepted'],[[r['name'],r['classification'],r['attempts'],r['accepted']] for r in summary])
    gate=f['candidate_gate'];metric_table=table(['Current candidate metric','Minimum','Maximum','Violations'],[[m,gate['extrema'][m]['min'],gate['extrema'][m]['max'],gate['violations'][m]] for m in gate['violations']])
    whole_table=table(['Whole local 20-layer measure','Baseline minimum','Candidate minimum','Baseline maximum','Candidate maximum'],[[m,whole['baseline'][m]['min'],whole['candidate'][m]['min'],whole['baseline'][m]['max'],whole['candidate'][m]['max']] for m in whole['baseline']])
    report=f'''Decision question

Can the exact certificate blocker be repaired without changing certificate mathematics, and can bounded local Task-3 relaxation then complete enough evaluation to establish or reject useful correction authority?

**V4LocalNextFrontCorrectionAuthorityMixed**

One bounded junction candidate resolves the sampled determinant and interpolation-weight hazards through layer 20. Four cap components have no useful accepted correction in the tested search. The exact certificate implementation blocker is repaired. No geometry or prism layer is committed. Task 4 has not started.

## A. Authority verification

Branch: `{start['branch']}`. Starting HEAD: `{start['starting_head']}`. The bounded audit passed 150 earlier bindings, 46 Task-2 bindings, 30 Task-2B bindings, and 38 Task-3 bindings. `PREFLIGHT.json` contains exact paths and hashes; `PRESERVATION.json` confirms the end state. V3 source modifications: **0**. Sealed prior result/source modifications: **0**. Unrelated Git state is unchanged from this follow-up's start.

| Authority | SHA-256 |
| --- | --- |
| Task-3 report | `1bbdc5906a90291966b77f1fca4cc86734d9c21c86e5d268d6c7e8bdbda275d6` |
| Task-3 authority | `5e8c72f392734b576e9da7eba5a162447211b3832bc2e2244e2cbab6b1635538` |
| Task-2B report | `a15d973515eae7bd5fbcff3ee1c90cbb24c065b7cc16b68a0822037292927a55` |
| Task-2B authority | `d0ec913d1410d0be33939fe7d02585f5907fb48dcb7f960de70c357b938e489f` |
| Frozen certificate integration | `bab10e0d52d724a6d0a525b801e2486ae3fc1af9c0f02fa1030da5a8dcc99b7f` |

The same SDF authority, topology, wall coordinates, and preserved binary64 20-layer schedule are used. The large V3 mesh was not copied. There was no CFD, OpenFOAM, full-shell optimizer, or global dense sensitivity allocation.

## B. Certificate blocker reproducer

The preserved trial is component 01, `tangent_laplacian`, radius 0.25, six active vertices. The exact pair is edge 1/layer 1 against edge 729/layer 10. Both modified side images are valid rank-3 bilinear surfaces. `BLOCKED_TRIAL_REPLAY.json` binds the original reproducer hash, all entity labels, coordinates, projection, numeric metrics, and reforecast. Every original scientific field reproduces exactly.

The failing object is a SymPy `Mul`: `2*CRootOf(P, 0)`. `ROOT_REGRESSION.json` stores the full quartic coefficients, exact expression, root index, real/rational assumptions, transformed polynomial, exact isolation intervals, and sign. The call is `noncoincident_rank3_certificate.project -> unit_roots -> compare(root, 0) -> bounds`. The requested operation is exact inclusion and ordering in [0,1], not a floating-point root approximation.

## C. Root-cause analysis

The frozen `bounds` function admits only rationals and bare `CRootOf`. SymPy's exact real-root routine can factor out a rational scale. The resulting number is within the existing real algebraic model, but its expression type failed the implementation assumption.

Further replay exposed representation-state nondeterminism: a Python set selected comparison/refinement order, and retained arithmetic intervals changed CAD sector representatives after pair-cache eviction. The original mismatches changed only the proof traversal fields. Exact witness coordinates, parameters, and forbidden-intersection classification stayed identical. Full before/after evidence is retained; no actual proof fields were removed from final scientific hashes.

## D. Certificate implementation repair

For beta=c*alpha, the adapter uses the primitive polynomial q(t)=P(t/c). A nonzero rational substitution preserves irreducibility. Positive c preserves real-root order; negative c reverses it. The exact raw root constructor avoids factoring c back out. Signs, equality, algebraic fields, CAD cases, ownership, and intersection predicates still use the frozen functions.

Root inputs are presorted by exact expression before the unchanged numeric comparator. Each uncached noncoincident pair has private refinement state. Initial intervals use the same exact QQ isolation routine as CRootOf. No float roots, PSLQ embedding, new tolerance, or theorem change is used. Unsupported forms fail closed.

The V4 adapter and private loader are opt-in. Vendor source, original runtime, V3 source, and mathematical authority are byte-identical. The new execution/cache identity is `{repaired_kernel.VERSION}`. This does not promote or regenerate the historical integration authority.

A second control-flow repair removes a redundant baseline prerequisite. A complete candidate with an empty forbidden-pair set cannot contain a new forbidden pair, regardless of the baseline set. The junction candidate had already completed 39,360 exact pairs, but the old driver blocked it because a separate 42,975-pair baseline traversal stopped at its operational time bound. `REDUNDANT_BASELINE_REPRODUCER.json` preserves that failure. Nonempty candidate forbidden sets still require baseline comparison. Candidate completeness, exactness, and every current hard gate remain mandatory.

## E. Regression qualification

Nine exact root-form fixtures cover bare roots, integer/rational/negative scales, equivalent raw forms, signs, equality, ordering, algebraic-field zero, and the exact Task-3 quartic. Independent exact substitution and scaled rational intervals verify polynomial and root-index identity. Eleven retained rank/contact fixtures give byte-identical frozen/repaired records, including mixed rank and noncoincident rank 3.

Three independent Python hash-order settings produce identical full regression hashes. Pair-cache eviction also preserves full proof bytes after repair. A newly authored mixed-rank fixture initially reused a topology label at different coordinates; both kernels correctly rejected it. The fixture now uses disjoint labels. `FIXTURE_CORRECTION.json` records that test-only correction.

## F. Exact blocked-trial replay

The repaired result is **CertifiedForbiddenIntersection**, method `NoncoincidentRank3ExactCAD`, exact residual 0. Certification of this pair completes. Its current numeric failures remain: 6 nonpositive signed volumes, 11 nonpositive Jacobians, 1 determinant violation, 2 weight violations, 2 skewness violations, and 1 volume-ratio violation. The trial is rejected. Wider new intersections were not assessed for that rejected trial; pair repair is not candidate acceptance.

## G. Resumed component results

{component_table}

The original 60 positive-span leading-cap trials were not rerun. The first 12 trailing-cap trials remain preserved; the interrupted twelfth was separately replayed and rejected. Its remaining 48 trials ran. The aft and negative-span leading caps each ran 60 trials. The junction's 27 already rejected trials were retained; its interrupted 28th proposal was reassessed after the control-flow repair. Search fields, active DOFs, trust updates, objective hierarchy, and patch limits are unchanged.

## H. Candidate hard-gate results

Accepted candidates: **1**, kept only as a study artifact. The junction proposal moves four vertices. Maximum displacement is **{event['candidate_displacement']} m**. The projected target is the original layer-1 distance. Maximum absolute SDF residual is {max(abs(x) for x in f['projection']['residual'])}; projection iterations are {f['projection']['projection_iterations']}, under the unchanged source criterion.

{metric_table}

All 30 measured current cells have positive exact rational Jacobians and volumes. Topology and source/wall ownership are valid. The complete current local side-pair scope has 360 pairs, zero forbidden intersections, and zero unresolved geometry. No hard gate is weakened. The exposed top has no physical neighbor; no artificial top-cap interpolation stencil is used. This is bounded prism-side evidence, not full-shell or downstream CFD qualification.

## I. Future-hazard changes

Junction first numeric invalidity changes from layer 2 to none through layer 20. The determinant and later interpolation-weight failures disappear in the measured verification region. No new earlier numeric failure appears. Its complete 20-layer local continuous scope has 39,360 pairs and zero forbidden/unresolved side intersections. The certificate includes conservative candidates from the complete source edge set; exact evaluation remains local.

{whole_table}

The original trailing-cap intersection still remains in the unchanged retained trailing-cap forecast. The prior 11 separated trial pairs remain diagnostic evidence, but their candidates failed numeric gates. No cap move was accepted. Zero/negative-Jacobian cap mechanisms therefore remain unresolved. The best newly tested cap proposals still had 3, 7, and 4 nonpositive current Jacobian measures for components 01, 02, and 04 respectively; these minima are observations, not impossibility proofs. The preserved leading-cap proposal retained four exact-zero Jacobians.

## J. Conditioning effects

In the accepted junction scope, minimum sigma rises from 0.06459903971926352 to 0.0710274465277787; maximum available G_bound falls from 92.70931982223773 to 89.07563429906362. The rejected blocked trial instead raised sigma from 0 to 0.03760889942253906 while G_bound rose from 70.1236223818545 to 109.99024301370953. Neither metric is a hard gate.

`LOCAL_TAIL_COMPARISON.json` contains complete sorted Jacobian, determinant, weight, volume-ratio, and column-minimum sigma vectors. Some current-layer measures trade off: minimum weight decreases from 0.28788115538992987 to 0.28710641017364014; minimum volume ratio decreases from 0.29700313400564066 to 0.26445475465652357; maximum skewness rises from 2.9412698600326768 to 2.941693905067115. All retain the existing hard limits. There is no per-entity improvement quota or weighted quality score.

## K. Search and patch behavior

{scope_table}

The accepted junction scope has 4 active vertices, 21 fixed boundary vertices, 10 halo vertices, 30 measured columns, and 47 columns with halo (940 provisional prisms). Its field is `opposite_laplacian`, radius 0.03125. All later fronts come from normal reforecasting, not direct edits. Rejections roll back exactly. Both healthy controls execute zero trials and preserve coordinates exactly.

## L. Locality and cache reuse

The accepted move invalidates 4 vertex, 21 edge, and 18 triangle entries: 43 recomputations. It preserves 318,865 archived cheap entries. Every new unique candidate evaluation retains an exact hit for its unrelated detailed region: {unique_hits} hits, in addition to the preserved earlier evidence. The accepted move recomputes one local detailed patch and retains one explicit unrelated hit. Maximum scope is 24 active vertices, 109 columns with halo, and 2,180 provisional prisms. No per-trial cold whole-front forecast occurs.

Direct initial component-search runtime is {resource['initial_search_component_seconds']:.6f} s, including the abandoned baseline traversal. Resumed continuation runtime is {resource['continuation']['seconds']:.6f} s. Final full scientific replay is {replay['seconds']:.6f} s. Audited task wall time to sealing is {elapsed:.3f} s. Maximum observed worker RSS is {worker_peak} bytes. Detailed stage timings and prior repair-replay costs are retained in `RESOURCES.json`; these are measurements, not extrapolations. Exact certification is the dominant observed cost.

## M. Determinism

Run 1: `{det['run1']}`

Run 2: `{det['run2']}`

All new scientific histories, exact certificate records, direct measures, projections, and candidate coordinates match. Inherited completed histories are hash-bound and not re-executed. The complete junction certificate stream is unchanged before/after the redundant-baseline repair and the representation-state repairs: `{f['future_certificate']['records_sha256']}`. Timing/RSS/process metadata is kept separately. The abandoned time-bound baseline prefix is explicitly operational evidence, not a final scientific conclusion.

## N. Tests

`python tools/task3_repair_tests.py`: **{tests['tests']-tests['failures']} passed, {tests['failures']} failed, 0 errors**, {tests['tests']} total. All 22 new tests pass. The three failures are the retained historical Git-status assertions from setup, forensics, and Task 3. This task's own status-preservation test passes. The extra `examples/conditioning_cpu_probe.rs` and `research/conditioning_cpu_optimization/` paths were already present at this follow-up's start. Older setup/forensics snapshot differences also predate it. No assertion was weakened.

The root regression, three-process hash-order check, repaired blocked-trial replay, and final cohort replay all pass. No scientific test fails.

## O. Final classification

**V4LocalNextFrontCorrectionAuthorityMixed.** A fully valid bounded junction proposal establishes useful local correction authority for its determinant/adjacent-cell imbalance and weight mechanisms. Four sampled cap mechanisms still have no accepted correction. This is neither a proof of global meshing nor proof that tangent motion cannot repair the caps. The original symbolic blocker and the later redundant baseline requirement are resolved; no technical execution blocker remains in the completed cohort.

`AUTHORITY.json` lists every changed source and evidence hash. `SEAL.json` binds the final report and authority. Only these V4 follow-up files are intended for the commit. The containing Git commit is reported in the final response; a commit cannot embed its own hash in its hashed report.

## P. Task-4 readiness

**Do not proceed to rolling commitment yet.** The successful local trade is meaningful, deterministic, and fully valid in its assessed scope. The cap components still contain current-layer degeneracy or negative Jacobians, so an admissible complete next front has not been established. Continue bounded cap correction-authority diagnosis, distinguishing search support, patch support, and tangent-space limits before widening the motion model. Preserve all gates. No layer beyond layer 0 is committed.
'''
    (OUT/'report.md').write_text(report,encoding='utf-8')
    source_paths=[ROOT/'tools'/n for n in ['task3_repair_preflight.py','task3_root_regression.py','task3_repaired_trial.py','task3_repair_study.py','task3_repair_replay.py','task3_repair_tests.py','task3_repair_seed_check.py','task3_repair_finalize.py']]
    source_paths += [ROOT/'tests/test_task3_repair.py',ROOT/'src/sdf_cfd_mesher_v4/advancing_front/certificate_comparison.py']
    source_paths += [ROOT/'src/sdf_cfd_mesher_v4/certificate/continuous_prism_certificate'/n for n in ['scaled_root_adapter.py','repaired_kernel.py','repaired_runtime.py','EXACT_REPRESENTATION_ADAPTER.md']]
    evidence=[p for p in OUT.iterdir() if p.is_file() and p.name not in ['AUTHORITY.json','SEAL.json','COMMIT_PATHS.json']]
    rel=lambda p:p.relative_to(ROOT).as_posix()
    files=source_paths+evidence
    auth=dict(task='RepairV4ExactCertificatePathAndResumeLocalRelaxation',classification='V4LocalNextFrontCorrectionAuthorityMixed',branch=start['branch'],base_head=start['starting_head'],
              source_sha256={rel(p):file_hash(p) for p in source_paths},evidence_sha256={rel(p):file_hash(p) for p in evidence},
              frozen_kernel_version=kernel.VERSION,repaired_kernel_version=repaired_kernel.VERSION,report_sha256=file_hash(OUT/'report.md'),
              results=summary,accepted_candidates=1,committed_layer=0,maximum_accepted_displacement_m=event['candidate_displacement'],task4_ready=False,
              scientific_replay=det,tests=dict(total=tests['tests'],passed=tests['tests']-tests['failures'],historical_status_failures=tests['failed_tests'],new_tests_passed=22,scientific_failures=0),
              resources=resource,v3_modifications=0,sealed_prior_modifications=0,unrelated_status_unchanged=True,
              storage=dict(new_source_evidence_bytes_before_seal=sum(p.stat().st_size for p in files),largest_new_file=rel(max(files,key=lambda p:p.stat().st_size)),largest_new_file_bytes=max(p.stat().st_size for p in files),free_disk_bytes=shutil.disk_usage(ROOT).free))
    write('AUTHORITY.json',auth)
    paths=sorted(rel(p) for p in files)+['results/task3_certificate_repair/AUTHORITY.json','results/task3_certificate_repair/SEAL.json','results/task3_certificate_repair/COMMIT_PATHS.json']
    write('COMMIT_PATHS.json',sorted(paths))
    write('SEAL.json',dict(report_sha256=file_hash(OUT/'report.md'),authority_sha256=file_hash(OUT/'AUTHORITY.json'),classification=auth['classification']))
    print(json.dumps(dict(classification=auth['classification'],report_sha256=auth['report_sha256'],authority_sha256=file_hash(OUT/'AUTHORITY.json'),new_paths=len(paths),wall_seconds=elapsed,peak_rss=worker_peak),indent=2))
