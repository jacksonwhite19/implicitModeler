"""Seal the bounded Task-3 stop; no new geometry search or certification."""
from pathlib import Path
import sys,json,datetime,shutil
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')];sys.dont_write_bytecode=True
from task3_preflight import audit,git
from task3_study import components,CONFIG
from sdf_cfd_mesher_v4.common.authority import file_hash
from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
from sdf_cfd_mesher_v4.advancing_front.local_relaxation import build_patch
OUT=ROOT/'results/task3'

def read(n):return json.loads((OUT/n).read_text())
def write(n,v):(OUT/n).write_text(json.dumps(v,indent=2,sort_keys=True)+'\n',encoding='utf-8')
def table(headers,rows):return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+['| '+' | '.join(map(str,r))+' |' for r in rows])
def unrelated(s):return sorted(r for r in s.splitlines() if 'research/sdf_cfd_mesher_v4/' not in r)

if __name__=='__main__':
    start=read('PREFLIGHT.json');assert git('rev-parse','HEAD')==start['starting_head'];assert not git('diff','--cached','--name-only')
    current=git('status','--porcelain');assert unrelated(current)==unrelated(start['starting_status'])
    verify=audit();write('PRESERVATION.json',dict(passed=True,prior_bindings=len(verify['rows']),task2_bindings=len(verify['task2_bindings']),task2b_bindings=len(verify['task2b_bindings']),v3_modifications=0,sealed_output_modifications=0,unrelated_status_unchanged=True,status_before_commit=current))
    def oldstatus(p,key):
        s=json.loads((ROOT/p).read_text())[key]
        return set('?? research/' if r.startswith('?? research/') else r.lstrip() for r in unrelated(s))
    now=oldstatus('results/task3/PREFLIGHT.json','starting_status')
    write('HISTORICAL_STATUS_TEST_CONTEXT.json',dict(
        setup_to_task3_start_added=sorted(now-oldstatus('authorities/START_STATE.json','status_porcelain')),
        task2b_to_task3_start_added=sorted(now-oldstatus('results/nominal_forecast_forensics/PREFLIGHT.json','starting_status')),
        task3_start_to_end_difference=[],explanation='Both failed assertions compare to older snapshots. These differences predate Task 3. Tests and unrelated work are unchanged.'))
    a=read('component_00.json');b=read('component_01.json');failure=read('CERTIFICATE_FAILURE_REPRODUCER.json');replay=read('DETERMINISM.json');tests=read('TESTS.json');timing=read('TIMINGS.json');rt=read('REPLAY_TIMINGS.json');stop=read('STOP.json')
    front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates));definitions=[]
    for spec in components():
        p=build_patch(top,spec['columns']);definitions.append(dict(**spec,initial_patch=p.record(),execution='executed prefix' if spec['name'] in ['component_00','component_01'] else 'not executed after certificate stop'))
    write('COMPONENT_DEFINITIONS.json',definitions)
    result_rows=[dict(component='component_00',classification='NoUsefulCorrectionFound',attempts=60,accepted=0,known_weight_first_invalid_before=18,accepted_after=18),
                 dict(component='component_01',classification='LocalCorrectionBlocked',attempts=12,accepted=0,reason='Exact certificate representation error on twelfth trial')]
    result_rows += [dict(component=f'component_{i:02}',classification='LocalCorrectionBlocked',attempts=0,accepted=0,reason='Not evaluated after required global task stop; no inference about correction authority') for i in (2,3,4)]
    write('COMPONENT_RESULTS.json',result_rows)
    write('ACCEPTED_CANDIDATES.json',dict(candidates=[],maximum_accepted_displacement_m=None,proposed_next_front_changed=False,committed_layer=0,reason='All evaluated proposals fail current numeric gates; exact certificate stop prevents further search'))
    write('HEALTHY_CONTROLS.json',dict(controls=[r for r in definitions if r['control']],scientific_control_execution=False,reason='Required stop before controls',no_op_behavior_unit_test='passed; no scientific control result claimed'))
    write('BASELINE_REFERENCE.json',dict(task2b_report_path='../nominal_forecast_forensics/report.md',task2b_report_sha256='a15d973515eae7bd5fbcff3ee1c90cbb24c065b7cc16b68a0822037292927a55',
           component_00_original_matches=[s['original_exact_cell_matches'] for s in a['scopes']],component_01_original_matches=[s['original_exact_cell_matches'] for s in b['scopes']],
           exact_baseline_intersection_reproduced=True,remaining_baselines='Preserved Task-2B histories; no new evaluation after stop'))
    sizes=[]
    for name,r in [('00',a),('01',b)]:
        for s in r['scopes']:
            p=s['patch'];sizes.append([name,s['active_rings'],len(p['active']),2*len(p['active']),len(p['fixed_boundary']),len(p['halo']),len(p['verification_ring']),len(p['detailed']['core_columns']),len(p['detailed']['columns']),20*len(p['detailed']['columns'])])
    size_table=table(['Component','Active rings','Active vertices','Tangent DOFs','Fixed core boundary','Halo vertices','Verification-ring columns','Measured columns','Columns with halo','Provisional prisms'],sizes)
    certsource=ROOT/'src/sdf_cfd_mesher_v4/certificate/continuous_prism_certificate/vendor/exact_algebraic.py'
    write('CERTIFICATE_BLOCKER_SOURCE.json',dict(path=str(certsource.relative_to(ROOT)).replace('\\','/'),sha256=file_hash(certsource),function='bounds',line=24,
        source_unchanged=True,input_validity=[s['valid'] for s in failure['surfaces']],input_ranks=[s['rank'] for s in failure['surfaces']],
        python=stop['python'],sympy=stop['sympy'],error=failure['result'],mathematics_changed=False))
    lastfail={k:v for k,v in failure['candidate_gate']['violations'].items() if v}
    cache_rows=[]
    for name,r in [('00',a),('01',b)]:
        for scope in r['scopes']:
            e=next(e for e in r['history'] if e.get('scope')==scope['active_rings'] and 'cache' in e);c=e['cache']
            cache_rows.append([name,scope['active_rings'],c['invalidated_vertices'],c['invalidated_edges'],c['invalidated_triangles'],c['recomputed_entries'],c['unchanged_entries']])
    write('CACHE_LOCALITY.json',dict(scope_counts=cache_rows,columns=['component','rings','vertices','edges','triangles','recomputed','unchanged_archive_entries'],
        detailed_unrelated_exact_hits=72,whole_front_cold_forecasts=0,committed_coordinates_changed=False,
        identical_warm_unit_check=dict(hits=1,misses=1,recomputed_on_second_call=0),timings=rt['components']))
    clock=datetime.datetime.now(datetime.timezone.utc);elapsed=(clock-datetime.datetime.fromisoformat(start['started_utc'])).total_seconds()
    # Direct process wall times returned during this task, including the first
    # uncaught stop and a failed first-field-only reproducer assumption.
    invocations=dict(initial_interrupted_search=8.3631376,initial_reproducer_wrong_field_assumption=1.9524867,
                     bounded_failure_locator_and_replay=4.8306058,structured_stop_search_replay=8.5031294,
                     final_scientific_replay=9.1044415,initial_test_suite=3.5091728,final_test_suite=4.1913452)
    resource=dict(started_utc=start['started_utc'],sealed_utc=clock.isoformat(),wall_seconds_to_seal=elapsed,
                  measured_process_wall_seconds=invocations,measured_process_wall_seconds_sum=sum(invocations.values()),
                  main_component_search_seconds=sum(v['seconds'] for v in timing.values()),replay=rt,
                  peak_worker_rss=rt['peak_rss'],one_worker=True,free_disk_bytes=shutil.disk_usage(ROOT).free)
    write('RESOURCES.json',resource)
    report=f'''Decision question

Can bounded local relaxation of the proposed next front prevent genuine forecasted prism hazards while keeping the current candidate layer fully valid?

**V4LocalNextFrontRelaxationBlocked**

The search stopped at the user's exact-certificate stop condition. No candidate was accepted. The leading-cap search found geometric responses, but four exact-zero Jacobians still blocked its wall-normal proposals. The trailing-cap search separated the known future edge pair in 11 rejected trials, then reached an unsupported algebraic-root representation in the unchanged certificate. All 12 trailing-cap trials also failed current numeric gates. This is not a certificate failure on an otherwise accepted candidate.

Only layer 0 remains committed. No rolling advancement, layer commitment, geometry optimization outside the bounded proposals, SDF-distance relaxation, topology change, compression, layer termination, CFD, or OpenFOAM execution occurred.

## A. Authority verification

Branch: `{start['branch']}`. Base HEAD: `{start['starting_head']}`. Initial and final bounded audits passed 150 earlier bindings, 46 Task-2 bindings, and 30 Task-2B bindings. No V3 source or sealed output changed. Full source/authority hashes are in `PREFLIGHT.json`; `PRESERVATION.json` records the end check.

| Authority | SHA-256 |
| --- | --- |
| Task-2 report | `c70194ba70539ef6ce5ef31a567e8650f97c3aa54a99309b373abbcfc2485825` |
| Task-2 authority | `310f84a630b8f436e0a80bdd83d196ac3ad40bf9371711b5eaa60f06eba49068` |
| Task-2B report | `a15d973515eae7bd5fbcff3ee1c90cbb24c065b7cc16b68a0822037292927a55` |
| Task-2B authority | `d0ec913d1410d0be33939fe7d02585f5907fb48dcb7f960de70c357b938e489f` |
| Certificate integration | `bab10e0d52d724a6d0a525b801e2486ae3fc1af9c0f02fa1030da5a8dcc99b7f` |
| Common geometry | `f453b7430703e9ae2c4938cd10f600c565d3fb7a0ce6f8e22e1161561326d1a3` |
| SF2 surface | `dc390bedc2198cb315e15961f848cd67b47641c30ae4ec0b96944996d1f8196f` |

The preserved binary64 schedule is loaded directly: h1=0.000166527069 m, growth=1.163359674, 20 layers, final target=0.020000000131704564 m. No schedule was regenerated. The accepted large polyMesh was not copied or read.

## B. Relaxation implementation

`advancing_front/local_relaxation.py` implements deterministic patch closure, two tangent DOFs per active vertex, proposal projection, sparse trajectory overlays, local cache identity, numeric admission, per-entity future failure comparison, exact final sign checking, and rollback. `local_certificate.py` adds bounded production-pair evidence and a fail-closed exception adapter. It changes no certificate mathematics. Study drivers and tests are Task-3-only files.

The tangent basis uses the normalized authoritative SDF gradient and a deterministic least-aligned Cartesian axis. A direction is projected into that basis, normalized when nonzero, scaled by the local move radius, and passed to the unchanged `predict_next_front` projection. Pre-projection points, tangent vectors, projected points, residuals, iteration counts, and source error bounds are saved for each trial. Projection termination remains the native source criterion.

Five geometric poll directions are tried in fixed order: incident wall-area normal, neighbor-centroid direction, its negative, and the two sums/differences with the wall normal. These guide tangent movement; they do not replace SDF authority. The geometric maximum radius is the preserved V3 expression in metres: min(0.25 times total nominal thickness, half the shortest incident wall edge). All radii use the preserved schedule.

The frozen two-way radius update is reused. Rejected polls shrink by half; an ordinary success can hold, and a supplied strong boundary agreement can grow to the frozen maximum. This study has no linear prediction model, so successful agreement would be left unavailable and held. The executed history contains only rejected polls and shrink steps. Six radius rounds, five directions, at most two active-ring scopes, 96 active vertices, 256 local columns, and three accepted steps are computational development limits. They are not convergence proofs or mesh thresholds.

Only layer-1 candidate coordinates are independent variables. The unchanged Task-1 trajectory routine generates layers 2–20 from the modified proposed origin. Its private origin adapter does not create or mutate a `CommittedFrontState`. All fixed boundary, halo, and unrelated coordinates use immutable archived arrays. Rejection leaves the incumbent proposal byte-identical. There is no commit API.

## C. Test components

`COMPONENT_DEFINITIONS.json` records all five user-selected development components and two planned controls. Their IDs are study inputs, not production risk detection.

| Component | Development target | Result |
| --- | --- | --- |
| 00 | Positive-span leading cap; later weight failure | NoUsefulCorrectionFound: 60 rejected trials across two bounded scopes |
| 01 | Negative-span trailing cap; coplanarity and exact intersection | LocalCorrectionBlocked: 12 trials, certificate stop on the twelfth |
| 02 | Negative-span aft cap; zero/negative Jacobians | LocalCorrectionBlocked: not executed after required stop |
| 03 | Upper/trailing-face junction; determinant and weight | LocalCorrectionBlocked: not executed after required stop |
| 04 | Negative-span leading cap; zero corner Jacobian | LocalCorrectionBlocked: not executed after required stop |
| Controls | Source columns 22890 and 3497 | Planned; no scientific control execution after stop |

{size_table}

The initial verification scope includes all columns incident to active vertices, a fixed adjacent-column ring, and one further face-neighbor halo for complete measured faces. Active-ring expansion is topological and deterministic. The second component-00 scope was used because no valid improvement passed in the first. No attempt expanded toward the full connected shell.

## D. Baseline hazards

The exact Task-2B intersection reproduces unchanged before searching: edge 1/layer 1 versus edge 729/layer 10, witness `(1, -1/4, 10823385/8589934592)`. `BASELINE_INTERSECTION.json` stores the complete exact record and entity trace. Every overlapping original cell measure in the evaluated verification scopes matches its Task-2 record exactly; counts are in `BASELINE_REFERENCE.json`. Remaining component histories are referenced in the sealed Task-2B authority, not recomputed after the stop.

Expanded component-00 scopes contain 11 and 15 layer-1 nonpositive Jacobians before movement. These differ from the original small-cohort count because the verification region is larger. The target's later weight failure is layer 18. Component 01 retains coplanar/Jacobian hazards and the exact cross-layer pair. No baseline authority drift was found.

## E. Candidate-layer validity

**Accepted candidates: 0.** No fully hard-gate-valid candidate was established. It would be misleading to describe all candidates as passing merely because the accepted set is empty. The accepted-coordinate artifact explicitly contains an empty list.

Admission retains positive volume and exact Jacobian, topology and source/wall ownership, determinant >=0.0011, interpolation weight >=0.055, skewness <=3.63636, symmetric volume ratio >=0.01, aspect <=1000, and complete passing candidate continuous evidence. Numeric checks include the complete verification cells and internal side faces. The exposed outer candidate cap has no physical neighbor; its later interface with layer 2 is forecast evidence, not an invented current neighbor.

All 60 component-00 trials failed current numeric admission. The 12 component-01 trials also failed. The certificate-stop trial had these nonzero violation counts: `{lastfail}`. Its minimum exact-polynomial binary64 Jacobian was -8.246823986467814e-09; determinant minimum 0.0010184356149281482; weight minimum 1.4809064770759408e-12; skewness maximum 5.876668946247538; volume-ratio minimum 7.559958476654126e-15. None of these failures was waived.

## F. Per-component correction results

Component 00: final retained proposal equals baseline exactly. The outer-ring wall-area-normal trial at radius 0.5 delayed the target weight failure from layer 18 to layer 20 and created no new earlier numeric failure in the verification region. Its maximum vertex displacement was 0.00026860524288731757 m. It still had four zero Jacobians and was rejected. At radius 1, the target weight hazard disappeared through layer 20, but four zero Jacobians and three newly earlier numeric failures blocked acceptance. Maximum tested displacement in this component was 0.0005372610730934459 m. The bounded result does not prove no solution exists.

Component 01: 11 rejected trials made the exact known pair `CertifiedSeparated`. This is a response to permitted layer-1 motion and normal reforecasting. It is not an admitted correction: current gates failed, and nearby complete continuous scope was not certified. The twelfth trial used `tangent_laplacian`, radius 0.25, six active vertices and 27 measured columns; certificate evaluation stopped. Maximum tested component displacement was 0.002054987752179096 m. The retained proposal remains the baseline.

Components 02–04 were not searched after the stop. Their classifications indicate interrupted assessment, not evidence that their mechanisms cannot respond. No accepted first-invalid-layer change or accepted maximum displacement exists. `COMPONENT_RESULTS.json` and complete executed search histories preserve this distinction.

## G. Exact intersection result

The original pair remains forbidden in the retained unchanged forecast. It is exactly separated in 11 rejected trial forecasts. The final attempted pair is **unresolved by execution failure**, not certified separated or forbidden. Both modified input surfaces pass the preserved input-validity checks and have rank 3. Stable source edges, vertices, layers, coordinates, ownership, SDF values, and all projection data are saved in `CERTIFICATE_FAILURE_REPRODUCER.json`.

The unchanged `vendor/exact_algebraic.py:24`, function `bounds`, accepts rational values and bare `CRootOf` objects. Root isolation produced `2*CRootOf(...)`; the type assumption raised `ArithmeticError: ExpectedExactRealRoot`. Source SHA-256: `{file_hash(certsource)}`. Runtime: Python {stop['python']}, SymPy {stop['sympy']}. No environment drift is inferred from this observation. The full quartic and exact error text are preserved.

The failure reproduces on the same binary64 side coordinates. A V4 adapter now records the exception and stops. It does not convert the result to separation, approximate a root, or change the qualified source. This is stop condition 9. No production-fixture qualification was rerun. A separate certificate follow-up is needed before continuing this search.

## H. Jacobian and thickness result

For the already-tested outer-ring component-00 wall-normal proposal at radius 1, exact rational wedge evaluation found **79 positive and 4 exactly zero** layer-1 Jacobians across the 83 measured columns. The remaining zeros cannot be accepted as rounding-scale positive thickness. `REJECTED_PROPOSAL_EXACT_SIGNS.json` includes every exact value and corner location.

These rejected proposals demonstrate that some local geometry responds, while full candidate validity remains absent. The four zeros also remain at radius 0.5 in direct metrics. No proof of feasible local repair, cap-wide necessity, or global coupling follows from this bounded search.

## I. Interpolation-weight result

The component-00 target can move from first failure 18 to 20 under a rejected early-front displacement. Complete removal in another rejected trial came with collateral failures. The frozen 0.055 gate and complete local face trajectories remain in the stored baseline/final datasets and search summaries. There is no accepted weight improvement. The component-03 weight target was not evaluated after the stop.

## J. Conditioning effects

Sigma and G_bound remain diagnostics. No threshold was introduced. Final retained values equal baseline because no move was accepted. In the rejected certificate-stop trial, the minimum candidate sigma was 0.03760889942253906 versus a baseline minimum of 0; maximum available candidate G_bound was 109.99024301370953 versus baseline 70.1236223818545. The operator tail can improve in one sense and worsen in another while geometric gates fail. These values do not justify acceptance.

## K. Local-tail behavior

Each executed component artifact stores full baseline and final sorted Jacobian, determinant, weight, volume-ratio, and sigma-column-minimum tails, together with complete bounded cell/face measures. The retained final tails are identical to baseline. The rejected stop trial additionally stores complete trial tails and metrics. No severe-tail threshold or weighted scalar score was created.

The comparison rule first requires current validity, then target removal, later first invalidity, and same-metric sorted-tail comparison. A per-entity check rejects any earlier future hard failure in the verification region. Complete nearby intersection comparison is required before acceptance. That acceptance path was not qualified by a successful case in this blocked run.

## L. Healthy controls

Both planned controls remain unexecuted after the required stop. The no-op transaction behavior passes its unit test with exact coordinate identity. This is not presented as a completed SF2 control experiment. `HEALTHY_CONTROLS.json` records the limitation.

## M. Cache and locality

{table(['Component','Active rings','Vertex entries','Edge entries','Triangle entries','Recomputed entries','Unchanged archive entries'],cache_rows)}

Only active vertex trajectories and their dependent edge/triangle descriptors are recomputed. Original arrays remain immutable and supply all other values. Each of the 72 evaluated trial prefixes also returns the unrelated detailed control patch as the exact same cached object. Detailed counters: component 00 has 60 hits/63 misses; component 01 has 12 hits/14 misses, including baseline scopes. The identical local-forecast unit replay gives one miss followed by one exact hit and zero second-call recomputation. No full cold forecast is run per candidate.

Maximum executed scope is 23 active vertices, 46 tangent DOFs, 83 measured columns, 107 columns with halo, and 2,140 local provisional prisms. No global direction Jacobian, LP, sensitivity matrix, or full-shell optimizer exists on this path. The rejected V3 matrix size of 1,791,265,237,440 bytes is not allocated.

## N. Runtime and memory

Main structured component execution: {resource['main_component_search_seconds']!r} s. Full deterministic replay with exact stop and rejected-proposal signs: {rt['seconds']!r} s. Replay component-00 forecast/detailed times: {rt['components']['component_00']['forecast_seconds']!r}/{rt['components']['component_00']['detailed_seconds']!r} s; component-01: {rt['components']['component_01']['forecast_seconds']!r}/{rt['components']['component_01']['detailed_seconds']!r} s.

Peak worker RSS: **{resource['peak_worker_rss']} bytes**, from the Windows process high-water mark during final replay. One worker. Recorded process wall time across the initial interrupted run, the failed first-field-only reproducer assumption, the bounded failure locator, structured replay, final scientific replay, and both test runs totals {resource['measured_process_wall_seconds_sum']!r} s. These are measured execution times, not extrapolations. Total task wall time to sealing is {elapsed!r} s. `RESOURCES.json` preserves stage timings and their scope. Free disk at seal: {resource['free_disk_bytes']} bytes.

## O. Determinism

Only the executed prefix was replayed; no new search continued past the blocker. Complete histories, candidate arrays, baseline/final data, the exact error reproducer, and rejected-proposal rational signs match:

- Run 1: `{replay['run1']}`
- Run 2: `{replay['run2']}`

The exception reproducer also independently matches twice. Timing, RSS, process metadata, and paths are excluded. Actual scientific fields and the exact exception expression remain included. The initial redundant component-00 artifact was removed only after exact scientific equality to its retained replay was verified.

## P. Tests

Command: `python research/sdf_cfd_mesher_v4/tools/task3_tests.py`.

**77 passed, 2 failed, 0 errors, 0 skipped; 79 total.** All 17 new Task-3 tests pass. Retained scientific tests pass. The two failed assertions compare unrelated Git status against older setup and Task-2B snapshots. The extra unrelated status rows existed at this task's start. `HISTORICAL_STATUS_TEST_CONTEXT.json` records both deltas. The new current-task preservation test passes. No retained test or contract was changed.

The expected-exception regression test passing does not mean certificate coverage was repaired. The scientific assessment remains blocked by that exact runtime limitation.

## Q. Classification

**V4LocalNextFrontRelaxationBlocked**.

Bounded tangent movement and authoritative projection demonstrably change selected future metrics and the known pair's intersection result. However, no fully valid current candidate was accepted, and certificate evaluation failed on a later rejected diagnostic proposal. Useful admissible correction authority is not established. This is not a proof of geometric impossibility or a global-coupling architecture failure.

V3 modifications caused by the task: **0**. Sealed Task-1, Task-2 and Task-2B modifications: **0**. Unrelated dirty work remains unchanged. Only Task-3 source, tests, tools and evidence are intended for the narrow commit. Exact committed paths are listed in `AUTHORITY.json` and verified against Git after committing. The final commit hash is reported in delivery; this report is sealed before the commit to avoid self-reference.

## R. Task-4 readiness

**No.** First resolve and qualify the exact algebraic-root representation limitation in a separately authorized certificate follow-up. Then resume bounded Task-3 feasibility work, including the four remaining leading-cap zero Jacobians, the other three mechanisms, complete corrected-candidate certification, collateral checks, and actual healthy controls. No Task-4 rolling commitment was implemented or started.

Report and authority hashes are in `SEAL.json`; the authority binds all sources and evidence without a circular hash.
'''
    (OUT/'report.md').write_text(report,encoding='utf-8')
    sources=[ROOT/'src/sdf_cfd_mesher_v4/advancing_front/local_relaxation.py',ROOT/'src/sdf_cfd_mesher_v4/advancing_front/local_certificate.py',ROOT/'tests/test_task3.py']+sorted((ROOT/'tools').glob('task3_*.py'))
    evidence=sorted(p for p in OUT.iterdir() if p.is_file() and p.name not in ['AUTHORITY.json','SEAL.json'])
    paths=sorted(str(p.relative_to(ROOT)).replace('\\','/') for p in sources+evidence+[OUT/'AUTHORITY.json',OUT/'SEAL.json'])
    authority=dict(task='ImplementV4BoundedLocalNextFrontRelaxation',classification='V4LocalNextFrontRelaxationBlocked',base_head=start['starting_head'],branch=start['branch'],
                   source_sha256={str(p.relative_to(ROOT)).replace('\\','/'):file_hash(p) for p in sources},evidence_sha256={str(p.relative_to(ROOT)).replace('\\','/'):file_hash(p) for p in evidence},
                   report_sha256=file_hash(OUT/'report.md'),scientific_replay=replay,component_results=result_rows,accepted_candidates=0,committed_layers=0,
                   v3_modifications=0,sealed_prior_modifications=0,unrelated_status_unchanged=True,intended_commit_paths=paths,
                   task4_ready=False,stop_condition=9,tests=dict(total=79,passed=77,failed=tests['failed_tests'],new_tests_passed=17),source_and_output_bytes=sum(p.stat().st_size for p in sources+evidence))
    write('AUTHORITY.json',authority);write('SEAL.json',dict(report_sha256=file_hash(OUT/'report.md'),authority_sha256=file_hash(OUT/'AUTHORITY.json')))
    print(json.dumps(dict(classification=authority['classification'],seal=read('SEAL.json'),files=len(paths),wall_seconds=elapsed)))
