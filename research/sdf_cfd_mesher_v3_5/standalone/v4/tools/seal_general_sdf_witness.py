"""Seal local witness evidence, preserving all earlier scientific authorities."""
import json,hashlib,subprocess,time,datetime,re
from pathlib import Path
import qualify_general_sdf_witness as w
from task3c_io import read
q=w.q;ROOT=q.ROOT;OUT=w.OUT
load=lambda n:json.loads((OUT/n).read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
save=w.save
start=load('START.json');assert start['head']=='32bf689c43f88cac1d6cb7d16aa908e2b8f86606'
verified={}
for folder in ('layer_capacity_valid_prefix','complete_capacity_fixtures','layer_capacity_forecast','layer_capacity_witness','task4l'):
 path=ROOT/'results'/folder;a=json.loads((path/'AUTHORITY.json').read_text())
 for group in ('source_sha256','evidence_sha256'):
  for name,h in a[group].items():assert sha(ROOT/name)==h,name
 verified[folder]=dict(report=sha(path/'report.md'),authority=sha(path/'AUTHORITY.json'))
assert verified['layer_capacity_valid_prefix']==dict(report='864ebb89fe6fc52c2e229649ac62c11a1f71463e62ff0bfb3a6f3fb18ddde163',authority='694605b8810e4b8f3bb059b9bcdacfdb8f3034f7c38bdf6bbffe332b1143f907')
status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines();other=lambda rows:[v for v in rows if 'sdf_cfd_mesher_v4' not in v];assert other(status)==other(start['status'])
save('PRESERVATION.json',dict(authorities=verified,unrelated_entries=len(other(status)),unrelated_unchanged=True,V3_modifications=0,production_builder_modifications=0))
old=json.loads((q.OUT/'TERMINATION_INPUTS.json').read_text());prefix=json.loads((q.OUT/'RESULTS.json').read_text());refs=[]
for name,valid,first in [('parallel_gap',9,10),('near_contact',0,1),('symmetric_contact',7,8),('spherical_cavity',12,13),('cylindrical_cavity',15,16),('parallel_wide',20,None),('curved_wide',20,None)]:
 r=next(x for x in old if x['region_id']==name);assert r['last_fully_valid_layer']==valid and r['first_unadmitted_layer']==first
 hist=next(x for x in prefix if x['fixture']==name)['history'];assert all(h['passed'] and h['certificate']['passed'] for h in hist if h['committed'])
 refs.append(dict(name=name,last_valid=valid,first_unadmitted=first,witness=r['witness'],sealed_result_unchanged=True))
save('ANALYTIC_AND_CONTROL_REGRESSION.json',refs)
s=q.load_schedule();cf=q.CapacityForecast();run=ROOT/'results/task4l/fresh';top=read(run/'FINAL_VALIDATION.json')['topology_hash'];sf=[];tick=time.perf_counter()
for layer in range(1,21):
 r=cf.analyze('sf2_history',s,layer,events=read(run/f'LAYER{layer}_WARNINGS.json'),geometry_identity=sha(run/f'build_A/FRONT_{layer:02}.npy'),topology_identity=top)
 assert r['predicted_first_infeasible_layer'] is None and not r['termination_authority'];sf.append(dict(layer=layer,status=r['status'],identity=r['scientific_input_identity']))
save('SF2_REGRESSION.json',dict(history=sf,geometry_changes=0,correction_changes=0,layer_count_changes=0,capacity_limits=0,seconds=time.perf_counter()-tick))
d=load('RESULTS.json');t=load('TIMINGS.json');u=load('REPLAY_TIMINGS.json');g=d['general'];channels=d['channels'];assert load('DETERMINISM.json')['equal']
records=[]
for r in channels:
 terminal=r['history'][-1];kind='ExactJacobianFailure' if terminal.get('exact_failures') else terminal['failures'][0]['metric']
 records.append(dict(region_id=r['name'],last_valid_layer=r['last_valid_layer'],predicted_geometric_limit=None,first_hard_invalid_or_unadmitted=r['first_unadmitted_layer'],warning_layer=0,observed_quality_gate=kind,direct_capacity_witness=None,witness_geometry_identity=terminal['coordinate_hash'],steerability='NotRestoredBy104BoundedTrials',remaining_requested_thickness=r['remaining'],core_complete=True,closure_independent_failed_inputs=r['failed_input_wall_unchanged'] and r['failed_input_SDF_normal_unchanged'],termination_authority=False,scope='Reached tested prefix, not maximum over all possible steering'))
records.append(dict(region_id='general_asymmetric_passage',last_valid_layer=3,predicted_geometric_limit=None,first_hard_invalid_or_unadmitted=4,warning_layer=0,first_direct_diagnostic_target_layer=4,witness_available_from_committed_layer=3,warning_lead=4,witness_types=['DirectOpposingFrontConvergenceWitness','ExactCollisionWitness'],witness_geometry_identity=q.identity(g['diagnostic_radial_certificate']),steerability='NotRestoredBy104BoundedTrials',remaining_requested_thickness=q.schedule_demand(s,3),core_complete=True,closure_independent_failed_inputs=True,termination_authority=False,scope='Witnesses establish opposing/radial collision only; no unavoidable capacity limit proved'))
save('TERMINATION_INPUTS.json',records)
save('FALSE_CAPACITY_ACCOUNTING.json',dict(false_capacity_claims=0,new_hard_capacity_claims=0,general_warning_only=1,channel_quality_limits=3,steering_successes=0,known_analytic_misses=0,termination_ready=False))
log=(OUT/'tests.txt').read_text();match=re.search(r'Ran (\d+) tests in ([0-9.]+)s',log);assert match and '\nOK' in log
save('TESTS.json',dict(count=int(match[1]),seconds=float(match[2]),passed=True,command='python -B -m unittest test_general_sdf_witness test_capacity_valid_prefix test_complete_capacity_fixtures test_capacity_forecast test_capacity_witness'))
# A compact case reproduces the qualified adapter selection repair.
case=g['diagnostic_radial_certificate']['checks'][0];save('EXACT_ADAPTER_REPRODUCER.json',case)
save('REPAIRS.json',dict(changes=[dict(scope='New exterior-closure prototype only',cause='Rounded closest point can equal boundary query while a plane comparison differs by roundoff; division by zero in gradient normalization',repair='Retain deterministic limiting face normal when computed distance is exactly zero',test='test_exterior_alternative_zero_distance_gradient'),dict(scope='New diagnostic driver only',cause='Raw CertificateSession bypassed qualified root-key wrapper and raised ExpectedExactRealRoot for a scaled algebraic root',repair='Dispatch diagnostic pair through qualified efficient_proof.prove unchanged',test='test_exact_adapter_reproducer')],production_changes=0,interrupted_exploratory_attempt='An initial all-pair radial diagnostic was interrupted. It supplied no scientific conclusion; final diagnostic selects at most eight cross-component candidates.'))
committed=[h for r in channels for h in r['history'] if h['committed']];pairs=sum(h['certificate']['required_pairs'] for h in committed);calls=sum(h['certificate']['exact_calls'] for h in committed)
replay_seconds=sum(x['seconds'] for x in u['channels'])+u['general']['seconds'];peak=max(t['peak_rss'],u['peak_rss'],load('SUPPORT_AUDIT_TIMING.json')['peak_rss']);capacity_claims=0
brackets=sum(len(ray['brackets']) for row in g['ray_brackets'] for ray in row['rays']);iterations=[row['history'][-1]['iteration'] for row in g['traces']]
stat=dict(committed_channel_layers=len(committed),new_after_sealed_layer5=sum(r['last_valid_layer']-5 for r in channels),required_certificate_pairs=pairs,exact_fallbacks=calls,steering_trials=416,steering_successes=0,general_failed_vertices=len(g['failed_vertices']),exact_target_existence_brackets=brackets,diagnostic_exact_calls=len(g['diagnostic_radial_certificate']['checks']),false_capacity_claims=0)
save('OPERATION_COUNTS.json',stat)
classification='V4GeneralSDFCapacityWitnessPromising'
table='\n'.join('| '+r['region_id']+' | '+str(r['last_valid_layer'])+' | '+str(r['first_hard_invalid_or_unadmitted'])+' | '+r['observed_quality_gate']+' |' for r in records[:-1])
coretable='\n'.join('| '+r['name']+' | '+str(len(r['authority']['core_vertices']))+' | '+str(len(r['authority']['core_triangles']))+' | '+str(r['authority']['minimum_topological_rings'])+' |' for r in channels)
wall_seconds=(datetime.datetime.now(datetime.timezone.utc)-datetime.datetime.fromisoformat(start['utc'])).total_seconds()
save('TASK_CLOCK.json',dict(start_utc=start['utc'],sealing_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),elapsed_seconds=wall_seconds,scope='Includes exploration, interrupted diagnostic, implementation, tests and sealing; not a mesher performance benchmark'))
report=f'''## Decision question

Can V4 remove closure artifacts from the channel studies and establish a direct, local capacity witness on non-analytic SDF geometry that is strong enough to support future termination-topology design?

### A. Authority verification

Starting HEAD {start['head']} matches. Prior valid-prefix report and authority hashes match the request. All sealed source/evidence bindings for Task-4L, capacity forecast, fixture foundations and earlier witness studies remain unchanged. No V3, qualified builder, hard gate, certificate mathematics or prism connectivity change. {len(other(status))} unrelated dirty entries are preserved.

### B. Previous blocker

The old channel layer-6 projector stops at its operational 64-iteration cap. The same authoritative projections converge in 144–148 iterations. This was not a proof that the target level set was absent. The general-SDF layer-4 case also exceeds 64 iterations while gradients remain nonzero. Failure of the projector and failure of the converged mesh candidate are separate results.

### C. Channel closure redesign

Selected closure authority keeps all original core and metric-halo wall coordinates and triangle connectivity bitwise unchanged. The remote upper extent increases from 0.1 to 0.2 m; planar lateral closure increases from 0.05 to 0.1 m; the narrow end is repositioned from -0.007 to -0.0065 m. This is not described as moving every boundary farther away. Definitions and changed vertex IDs are explicit.

The study invokes the unchanged projector with a bounded operational cap of 256, retaining its original SDF residual/error criterion. No production default or scientific tolerance changes. Closure revision alone does not explain removal of the old stop; the measured projection cap was also necessary. All complete-front nominal SDF projections reach requested target 20 in the independent diagnostic pass.

A second complete-solid-wall closure was tested and rejected: planar variants reach 8 before required-side irregularity, while the curved variant reaches 4 before projection failure. Its core triangles are coordinate-identical, but it provides no additional continuation authority.

| Channel | Core vertices | Core triangles | Minimum graph rings to changed closure |
|---|---:|---:|---:|
{coretable}

Closure-only cells still show numeric failures. They are not claimed to be a qualified full shell. This is the existing scientific-core contract, not a fixture-only exception: all core cells, actual metric dependencies and required continuous interactions are checked. CLOSURE_ISOLATION.json records non-core failures and confirms zero core hard failures on committed layers. No required relation is skipped because one side belongs to a closure.

### D. Channel scientific-core continuation

The sealed layer-1–5 core and complete metric-halo coordinates are loaded by verified checkpoint identity. Revised closure history is constructed separately, SDF residuals checked, and all five migrated layers recertified against it. Continuation starts from that exact sealed core at layer 5. Later accepted layers use ordinary numeric, exact-Jacobian and complete core-side certification. Future/closure projection diagnostics never commit a layer.

| Channel | Last valid reached | First unadmitted | First controlling validation condition |
|---|---:|---:|---|
{table}

The symmetric planar channel has four exact nonpositive Jacobian results at layer 10. The asymmetric channel requires edge 96 (owners 43/44) at layer 9; its bilinear side is NonRegularPlanarBilinearSide and cannot enter the exact certificate. This is an explicit validation failure, not a certified forbidden intersection. The curved passage first violates determinant >= 0.0011 at layer 8: column 1026 gives 0.000914695582827545.

Failure-local wall inputs are unchanged by the closure revision. Their old/new SDF values and normals agree at source floating-point precision; active geometry comes from the unchanged channel walls. No failed input uses a moved end-closure vertex. Thus the new stops are local quality/interaction-support failures rather than the old distant layer-6 projection abort. They do not prove geometric capacity exhaustion.

### E. Channel classifications

All three are CoreQualityLimitBeforeCapacity for the tested continuation. Each receives 104 bounded trials from existing area/axis and existing Laplacian tangent directions, with unchanged amplitude ordering, dependency-complete support, SDF reprojection and immediate validation. No trial restores validity. These are reached valid prefixes, not globally maximal prefixes or proofs against every possible steering field. No channel receives capacity authority.

### F. General-SDF projection failure reproducer

Sixteen source vertices reproduce the layer-4 cap failure from sealed layer 3. Target distance is {g['traces'][0]['target']!r} m. GENERAL_SDF.json records each source ID, coordinates, neighbor topology, component owner, normalized gradient, phi and residual at every iteration. Nearest-sphere ownership alternates; the gradients remain unit and nondegenerate. The same unchanged Newton iteration converges in {min(iterations)}–{max(iterations)} iterations when the cap permits it.

Classification of the projection failure is DiscreteProjectionDefect: the operational cap truncates convergence near a nonsmooth nearest-source switch. This does not imply that increasing the cap makes a valid prism layer. The converged layer-4 candidate has 15 exact nonpositive Jacobians and remains uncommitted.

### G. Target-level-set existence study

There are {brackets} local ray brackets over gradient and tangent probes. Exact binary64-as-rational squared-distance comparisons establish opposite signs at their endpoints for the authoritative minimum-of-sphere-distances SDF. Continuity therefore proves a target-level solution between each endpoint pair. Subsequent binary64 bisection is diagnostic only. No empirical tolerance, ray non-crossing or failed Newton iteration is used as proof of absence.

These probes establish local existence, not a globally consistent source-ownership chart or a valid prism mesh. TargetLevelSetAbsent is not supported for this failure.

### H. Opposing-front / offset-pinch study

For components 1 and 2, the exact squared inequality between centre distance and the sum of radii plus twice the requested offset first changes to overlap at layer 4. The other two pairs first overlap at layer 5. This directly establishes opposing offset-branch convergence on this composite SDF; no medial-axis distance is invented.

The qualified optimized exact adapter certifies a forbidden noncoincident rank-3 intersection for radial-continuation side pair [7757, 8695]. It was selected from 311 cross-component current-layer AABB survivors; one exact call sufficed. The compact reproducer stores both surfaces, labels and exact algebraic proof. This radial candidate is explicitly not SDF-admitted. Its collision is not proof that all tangent steering must fail, and it is not counted as a committed-prefix certificate.

### I. Steering probe

The general-SDF case receives 104 existing tangent-direction/amplitude trials on dependency-complete local support. Every trial is evaluated and rejected under ordinary validation. There is no demonstrated valid continuation and no exhaustive impossibility proof. Across channels and the general case, 416 trials produce zero accepted corrections. No normal-motion DOF, topology change, margin or new gate is introduced.

### J. General-SDF witness classification

GeneralSDFWarningOnly remains appropriate. DirectOpposingFrontConvergenceWitness and ExactCollisionWitness describe the measured diagnostic geometry. They do not authorize an unavoidable capacity limit. Missing evidence is either a fully validated steered continuation or a direct obstruction applying to every admissible continuation of the relevant local mesh branch. Failed bounded optimization cannot supply that obstruction.

### K. Last-valid-layer result

The general case retains last valid 3 / first unadmitted 4. The initial warning was available at layer 0, giving four layers of descriptive lead. The direct layer-4 diagnostic is available from committed layer 3. The converged nominal candidate fails exact Jacobian positivity. No new general-SDF layer commits.

### L. Analytic fixture regression

All prior hashes and witness records remain unchanged: parallel 9/10, near-contact 0/1, symmetric 7/8, sphere 12/13, cylinder 15/16 (valid/unadmitted). Their witness types remain bound to the previous authority. They were not rebuilt.

### M. Feasible-control regression

Both sealed wide controls retain all 20 passing layers and zero capacity claims. Their complete committed gate/certificate histories and authority hashes were checked without reconstruction.

### N. SF2 regression

A read-only pass over all 20 Task-4L histories gives zero capacity limits. Source/evidence hashes preserve all geometry, 32 correction decisions, layer counts and prior scientific results. No SF2 build ran.

### O. Termination-planner records

TERMINATION_INPUTS.json binds region, reached valid layer, observed invalid/unadmitted layer, warning onset, evidence identity, steerability result, exact remaining requested thickness and core/closure scope. Geometric capacity limits are null. All records set termination_authority=false. The general direct witness is diagnostic-only; channel records carry hard-quality evidence rather than capacity claims. False capacity claims: zero.

### P. Runtime and locality

Final measured qualification cohort: {t['seconds']:.6f} s. Independent replay: {replay_seconds:.6f} s. Closure setup: {sum(v['setup_seconds'] for v in t['channels']):.6f} s; independent closure projection: {sum(v['closure_projection_seconds'] for v in t['channels']):.6f} s; channel rolling continuation including probes: {sum(v['rolling_seconds'] for v in t['channels']):.6f} s. General projection diagnosis: {t['general']['projection_diagnosis_seconds']:.6f} s; SDF witness probes: {t['general']['SDF_witness_seconds']:.6f} s. Steering total: {sum(v['steering_seconds'] for v in t['channels'])+t['general']['steering_seconds']:.6f} s; certification total including diagnostic pair: {sum(v['certificate_seconds'] for v in t['channels'])+t['general']['certificate_seconds']:.6f} s. These stage times overlap where stated and must not be summed into a second total.

Peak RSS: {peak} bytes. Passing migrated/continued channel prefixes contain {len(committed)} layers ({stat['new_after_sealed_layer5']} newly continued after layer 5), {pairs} required certificate relations and {calls} exact fallbacks, with zero forbidden/unresolved committed results. Support/alternative audit: {load('SUPPORT_AUDIT_TIMING.json')['seconds']:.6f} s separately. Exploratory runs and the interrupted all-pair diagnostic are excluded from the final cohort measurement; TASK_CLOCK.json records task elapsed time. This is not a full-build performance claim.

### Q. Determinism

Independent channel restart/continuation and general-SDF replay reproduce geometry identities, traces, trial decisions, exact outcomes, classifications and records. Scientific hash {load('DETERMINISM.json')['scientific_hash']}. Runtime/RSS are separate. Prior checkpoints are verified before use. No stale receipt is reused for revised closure geometry.

### R. Tests

{match[1]} tests passed in {match[2]} s. Command: {load('TESTS.json')['command']}. Tests cover core/halo preservation, restart beyond layer 6, closure projection and core isolation, exact projection traces, rational existence brackets, opposing convergence, warning-only fallback, failed-poll non-authority, qualified exact adapter reproduction, deterministic replay and preserved SF2 evidence. Existing fixture/capacity/prefix tests remain unchanged.

Two new-study implementation defects were repaired: a zero-distance gradient normalization in the rejected solid-closure prototype, and diagnostic use of a raw certificate entry point that bypassed qualified root-key handling. REPAIRS.json preserves the causes and focused regressions. No production or certificate mathematics changed. No test claims an offset-pinch impossibility theorem that this study did not establish.

### S. Classification

{classification}. Channel continuation is no longer stopped by the old distant projection cap and now has defensible tested local-quality boundaries. The general-SDF projection failure is explained, and direct convergence/collision evidence exists, but it remains insufficient for unavoidable capacity authority.

### T. Termination-topology readiness

No. First qualify branch-aware local continuation/steering using the existing physical DOFs, or establish a direct obstruction for the admissible mesh branch. A Newton cap, a failed poll, or collision of one diagnostic candidate is not permission to terminate. No termination policy or transition topology was implemented.
'''
(OUT/'report.md').write_text(report,newline='\n')
sources=['tools/channel_closure_study.py','tools/qualify_general_sdf_witness.py','tools/audit_general_sdf_support.py','tools/seal_general_sdf_witness.py','tests/test_general_sdf_witness.py']
evidence={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in sorted(OUT.rglob('*')) if p.is_file() and p.name not in ('AUTHORITY.json','SEAL.json')}
save('AUTHORITY.json',dict(classification=classification,starting_head=start['head'],source_sha256={p:sha(ROOT/p) for p in sources},evidence_sha256=evidence,report_sha256=sha(OUT/'report.md'),termination_ready=False,production_builder_modified=False,V3_modified=False))
save('SEAL.json',dict(report_sha256=sha(OUT/'report.md'),authority_sha256=sha(OUT/'AUTHORITY.json'),persistent_bytes=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file())))
print(json.dumps(load('SEAL.json'),indent=2))
