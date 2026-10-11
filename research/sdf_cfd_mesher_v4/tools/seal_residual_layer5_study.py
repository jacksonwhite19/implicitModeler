"""Seal the bounded negative result; no production or capacity authority."""
import hashlib,json,subprocess
from fractions import Fraction
import residual_layer5_study as s
r=s.r;q=s.q;ROOT=r.g.ROOT;OUT=s.OUT
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
load=lambda n:r.load(OUT/n)
head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
assert head=='bcf511c05f5f3f1a8a2540312952478ff2270a20'
assert subprocess.check_output(['git','branch','--show-current'],text=True).strip()=='feature/sdf-cfd-mesher-v4-advancing-front'
status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines()
other=[z for z in status if 'sdf_cfd_mesher_v4' not in z]
assert other==load('START.json')['status']
assert not subprocess.check_output(['git','diff','--name-only','--','research/sdf_cfd_mesher_v3'],text=True).strip()
prior={}
previous=r.load(r.OUT/'AUTHORITY.json')
for name in ['compressed_layer_recovery']+list(previous['prior_authorities']):
    folder=ROOT/'results'/name;a=r.load(folder/'AUTHORITY.json')
    for group in ('source_sha256','evidence_sha256'):
        for path,h in a.get(group,{}).items():assert sha(ROOT/path)==h,path
    prior[name]=dict(report=sha(folder/'report.md'),authority=sha(folder/'AUTHORITY.json'))
assert prior['compressed_layer_recovery']['report']=='b95f9a7e0db83a18ee9f6a27487baf1028245bf92aeceaa1fa92ff04c60d9ab9'
actual='413b7f31209c1c0438d1e03b1863bbd3054fcf9acb84308a730c7abc03a49057'
assert prior['compressed_layer_recovery']['authority']==actual
blob=subprocess.check_output(['git','show',head+':research/sdf_cfd_mesher_v4/results/compressed_layer_recovery/AUTHORITY.json'])
assert hashlib.sha256(blob).hexdigest()==actual
s.save('AUTHORITY_VERIFICATION.json',dict(prior=prior,unrelated_dirty_entries=len(other),unrelated_unchanged=True,
    supplied_authority_typo='Supplied digest omits 1c after 413b7f31209c; committed blob and all manifest bindings verify.',
    actual_previous_authority=actual,V3_task_modifications=0,production_changes=0))
base=load('BASELINE.json');cause=load('CAUSE.json');sensitivity=load('SENSITIVITY.json');support=load('SUPPORT.json');neighbor=load('NEIGHBOR_SUPPORT.json')
assert load('DETERMINISM.json')['equal']
assert load('ITERATE_DETERMINISM.json')['equal'] and load('COMPRESSION_DETERMINISM.json')['equal']
tests=load('TESTS.json');assert tests['passed']
probes=load('PROBES.json');signs=load('SIGN_PROBES.json');smoothing=load('LOCAL_SMOOTHING.json');escape=load('BRANCH_ESCAPE.json');nc=load('NEIGHBOR_COUPLED.json')
rows=probes+signs+smoothing+escape+[nc]+load('DIRECTION_CONFLICTS.json')
assert all(not row['result']['gate']['passed'] for row in rows)
classification='V4ResidualLayer5FailureInsufficient'
case_class='Layer5NoSolutionFound'
exact=base['record']['gate']['exact_failures'][0]
timing=load('RUNTIME.json');replay=load('REPLAY_RUNTIME.json');tc=load('THICKNESS_CONDITIONING.json');coll=load('COLLISION_RESULTS.json')
qualification_seconds=sum(load(n)['seconds'] for n in ('RUNTIME.json','REPLAY_RUNTIME.json','ITERATE_TIMING.json','ITERATE_REPLAY_TIMING.json','COMPRESSION_TIMING.json','COMPRESSION_REPLAY_TIMING.json'))
def summary(row):
    z=row['result'];return dict(exact_failed_cells=len(z['gate'].get('exact_failures',[])),numeric_failures=len(z['numeric_events']),
        residual_jacobian=z['residual_cell']['jacobian'],residual_sign=z['residual_cell']['jacobian_sign'],passed=z['gate']['passed'],
        max_tangent_displacement=z['max_tangent_displacement'])
result=dict(classification=classification,case_class=case_class,last_valid_layer=4,layer5_committed=False,layer6_attempted=False,
    baseline_coordinate_hash=base['record']['coordinate_hash'],baseline_exact=exact,
    final_candidates=[dict(mode=z['mode'],**summary(z)) for z in probes],
    sign_restoration=[dict(mode=z['mode'],**summary(z)) for z in signs],
    neighbor_coupled=summary(nc),selected_smoothing=summary(smoothing[-2]),
    fully_valid_candidate_found=False,infeasibility_proved=False,termination_candidate=False,
    production_integration=False,wall_changes=0,first_layer_changes=0,production_changes=0,
    verified_endpoint_count=len(rows),recorded_solver_iterates=sum(len(z.get('history',[])) for z in probes+signs+[nc]))
result['best_residual_iterates']=[dict(mode=z['mode'],source=z['source'],**summary(z)) for z in load('ITERATE_AUDIT.json')]
result['strict_compression_from_seed']=load('SEED_COMPRESSION.json')
s.save('RESULT.json',result)
for name in ('SF2_REGRESSION.json','ANALYTIC_CONTROL_REGRESSION.json'):
    s.save(name,dict(unchanged=True,source=str((r.OUT/name).relative_to(ROOT)),source_sha256=sha(r.OUT/name),
        evidence=r.load(r.OUT/name),method='Read-only sealed evidence and complete authority-binding verification; no mesh rebuild'))
def walk(value):
    if isinstance(value,dict):
        if value.get('coordinate_hash')==base['record']['coordinate_hash']:return True
        return any(walk(v) for v in value.values())
    if isinstance(value,list):return any(walk(v) for v in value)
    return False
assert walk(r.load(r.OUT/'result.json'))
s.save('REPRODUCER_IDENTITY.json',dict(matched_previous_candidate=True,coordinate_hash=base['record']['coordinate_hash'],
    source='compressed_layer_recovery/result.json; expanded support, receding H5, retained factor 0.125',
    scope='One preserved one-failure endpoint selected for prior individual-height and coupled probes; no recovery horizon comparison rerun'))
table='\n'.join(f"| {z['mode']} | {len(z['result']['gate'].get('exact_failures',[]))} | {len(z['result']['numeric_events'])} | {float(Fraction(z['result']['residual_cell']['jacobian'])):.12g} | rejected |" for z in probes)
direct={z['mode']:z for z in signs}
maxmove=max(z['result']['max_tangent_displacement'] for z in rows)
basej=float(Fraction(exact['jacobian']));basevol=float(Fraction(exact['signed_volume']))
report=f'''## Decision question

Why does one exact Jacobian remain nonpositive at general-SDF layer 5, and can that single local failure be repaired using the practical local correction tools already available?

### A. Authority verification

Input HEAD `{head}` and the feature branch match. All {len(prior)} prior authority manifests and their source/evidence bindings verify unchanged. The supplied prior report hash matches. The supplied prior authority digest contains a transcription error: it omits `1c` after `413b7f31209c`. The actual digest is `{actual}`. It matches the committed Git blob and all manifest bindings. No prior evidence was rewritten.

V3 has zero task modifications. All {len(other)} unrelated dirty entries remain unchanged. Production code, wall geometry, first-layer height, topology, schedule authority and certificate mathematics are unchanged.

### B. Exact failure reproducer

The compressed layer-4 seed replays bitwise and passes all gates and its 43,162-relation certificate. The selected historical H5/factor-0.125 candidate reproduces coordinate identity `{base['record']['coordinate_hash']}`. This is the same endpoint previously used for individual-height and coupled probes. Its horizon is only a reproduction input; no recovery strategy comparison is repeated.

The failed prism is column **1232**, source triangle **[527, 645, 670]**. Its exact minimum is at reference corner **0**, height **1**. Exact Jacobian:

`{exact['jacobian']}`

This is approximately **{basej:.16g} m³**. Corner 1 is also negative; its full rational is in EXACT_UPPER_CORNERS.json. Signed volume is **{basevol:.16g} m³**, also negative. “One failure” meant one failed cell, not one failed corner and not a candidate passing every other gate. Numeric diagnostics expose **8 determinant**, **1 skewness**, **1 interpolation-weight**, and **2 cell volume/Jacobian** events: 12 events in total. Ordinary validation correctly stopped at the first decisive exact gate in the prior study.

### C. Minimal support

The first support has 3 active vertices, 6 tangent variables and 3 explicit height variables. It affects {len(support['affected_columns'])} prisms, with {len(support['metric_columns'])} metric-dependency columns, {len(support['side_edges'])} side faces, {len(support['verification_ring'])} verification-ring columns, {len(support['fixed_boundary'])} fixed boundary vertices and {len(support['halo'])} halo vertices. One-ring height blending expands to {len(neighbor['active'])} active vertices and {3*len(neighbor['active'])} total variables. Lower geometry remains fixed and valid; it has no zero-area triangles. Every verified endpoint uses all 1,536 scientific columns for final admission.

### D. Direct geometric cause

All three upper points lie on the intersection of source-1 and source-2 sphere offsets to the existing SDF admission precision. Subtracting the two squared sphere equations gives a plane with normal `c2-c1 = [-0.0035, 0.0045, 0.0003]`. The recorded upper area vector points against that plane normal. The exact upper-area/displacement triple products are negative at vertices 527 and 645. This establishes a folded upper prism face under the current branch-constrained projection, despite equal positive increments at all three vertices.

The failure is not simply a zero or negative requested height. It is not an invalid layer-4 seed. The valid compressed ancestry and subsequent projection onto the intersecting offsets produce a reversed local volume trajectory. Other tangent/height configurations are not excluded by this diagnosis. No global absence of a target level set is claimed.

### E. Jacobian sensitivity

SENSITIVITY.json records every local hard-metric row against all 9 controls. Central differences use cube-root machine epsilon in dimensionless control coordinates. The failed normalized Jacobian responds to both tangent and height controls. The measured finite-difference constraint and coordinate-response ranks are {sensitivity['constraint_rank']} and {sensitivity['geometry_rank']}. These are numerical local ranks at a piecewise-smooth ownership transition, not exact tangent-space rank or infeasibility proofs. DIRECTION_CONFLICTS.json independently verifies both signs of every control with exact wedge checks and full numeric diagnostics.

### F. Hard-gate conflict

Improving one signed corner does not ensure a valid prism or valid neighbors. The failed cell's determinant is 3.854633812139241e-08, below 0.0011. Side 1592 has skewness 8.346048684516772, above 3.63636. Four determinant failures at columns 784, 785, 787 and 848, plus cap-785 weight 0.04266658115506743, are outside the one-ring affected set. Those fixed failures cannot be removed by this support. This is a direct limitation of the chosen local solve, not proof that the broader allowed correction model is infeasible.

### G. Compression-only probe

The local height-only SQP allows both smaller and larger increments within computational bounds; it is explicitly a height-adjustment probe. Separate strictly compression-only trials multiply the one-ring heights by 1/2, 1/4, 1/16, 1/64 and 1/256. None passes. They create 4–5 failed exact cells.

A second strict control compresses only the three controlling vertices and projects directly from the layer-4 seed. It preserves every outside coordinate bitwise. At factor 1/256, height is 0.04413459122568509 µm and Jacobian is -3.646255851958036e-15 m³: still negative, with 19 numeric failures. Approaching zero by collapsing an increment does not restore strict positivity or quality. No minimum valid height is established. These bounds are computational, not new scientific gates.

### H. Tangent-only probe

The six-variable tangent solve, a signed-volume/Jacobian restoration solve, and eight geometry-directed branch-escape trials all remain invalid. Source-aware reprojection reaches the assigned SDF distances. Tangent freedom is active; failed search is not a rank-deficiency proof.

### I. Coupled probe

Standard SciPy SLSQP is used deterministically on 9 controls, then 36 controls on the one-ring support. An auxiliary common slack guides restoration of existing normalized constraints. A separate sign-restoration diagnostic targets signed volume and Jacobians. Neither solver score nor optimizer success is admission evidence. Iteration caps and variable bounds are computational limits only; they do not define scientific feasibility. Final endpoints:

| Search | Exact failed cells | Numeric events | Residual-cell Jacobian (m³) | Admission |
|---|---:|---:|---:|---|
{table}

The sign-restoration coupled endpoint reaches {float(Fraction(direct['coupled']['result']['residual_cell']['jacobian'])):.12g} m³ at cell 1232, but remains negative and has collateral failures. Every stored solver iterate is also reconstructed to select the best exact residual-cell Jacobian, rather than assuming the final iterate is best. Best tangent-only: -1.1750543671055797e-12 m³, with 2 failed cells and 17 numeric events. Best coupled: -5.565339935770193e-13 m³, with 3 failed cells and 17 numeric events. Neither removes the residual sign failure. No bounded-set emptiness or unavoidable collision proof is established.

### J. Local smoothing

Convex graph-neighbor height blending is tested at 0.25, 0.5 and 1.0. The 0.5 blend reduces numeric events from 12 to 11, while leaving the controlling cell unchanged. It is selected only as the best numeric-count smoothing diagnostic, not as a qualified best mesh. Its exact collision count worsens. Full blending creates another exact failure. No smoothing endpoint is accepted.

### K. Exact candidate verification

{len(rows)} final/directional endpoints have exact production gate records; {result['recorded_solver_iterates']} solver iterates are preserved as search diagnostics. Numeric fields are also evaluated independently when exact rejection would normally skip them. Only fully valid candidates could commit; none does. Reprojection enforces the existing source error bound. The zero trial returns the original coordinates bitwise, avoiding an unnecessary round-trip projection. Rejected trials do not mutate the lower front, schedules or outer coordinates.

### L. Failure migration / collision result

The ordinary complete certificate is run diagnostically for the baseline and selected smoothing endpoint, despite their failed volume/quality gates. Baseline: **{coll[0]['certificate']['required_pairs']:,} relations, 26 forbidden, zero unresolved**. Smoothing: **{coll[1]['certificate']['required_pairs']:,} relations, 28 forbidden, zero unresolved**. Both have zero invalid side-patch records. Exact proof records and pair identities are preserved. Passing a selected pair is never substituted for complete coverage. Trial improvements cannot be accepted because other quality failures persist or migrate, and collisions remain.

### M. Thickness accounting

Nominal layer-5 increment: **{tc['nominal_increment']*1e6:.15g} µm**. Rejected baseline and selected smoothing increments at the failed triangle: **{tc['baseline_height'][527]*1e6:.15g} µm** each. Nominal cumulative offset: {tc['nominal_cumulative']:.17g} m. Local candidate offset: {tc['baseline_actual'][527]:.17g} m. Maximum candidate debt is **{max(tc['baseline_debt'])*1000:.15g} mm**. All per-vertex values and changes from the seed are explicit in THICKNESS_CONDITIONING.json.

These are rejected candidate heights, not committed layer-5 heights. Actual state remains layer 4 with maximum debt 0.22546912781357467 mm. No thickness recovery is committed. The largest endpoint tangent displacement tested is {maxmove:.15g} m, not an accepted move.

### N. Conditioning

Layer-4 sigma minimum / G maximum: **0.12093646405001122 / 25.253773477046682**. Invalid layer-5 baseline and selected smoothing: **0.0020128949079683918 / 2152.7746632387275**, controlled by cell 1232. No conditioning threshold is introduced. There is no final valid layer-5 conditioning result.

### O. Layer-6 smoke result

Not attempted. Layer 5 has no valid committed predecessor state for layer 6.

### P. SF2 regression

Read-only sealed Task-4 history and bound artifacts verify unchanged. Zero geometry, correction, layer-count or capacity-authority changes. No SF2 rebuild or production call site is added.

### Q. Analytic capacity regression

Sealed last-valid / first-unadmitted results remain parallel 9/10, near-contact 0/1, symmetric 7/8, sphere 12/13 and cylinder 15/16. Both feasible controls retain 20 layers. No shrinking-layer reinterpretation of those limits is made.

### R. Runtime and locality

Complete bounded run: **{timing['seconds']:.6f} s**. Independent replay: **{replay['seconds']:.6f} s**. Including the best-iterate audit and strict seed-compression control with their independent replays, measured qualification execution totals **{qualification_seconds:.6f} s**. Peak process RSS: **{max(timing['peak_rss'],replay['peak_rss']):,} bytes**. Timing files separate solver, verification and certificate costs. The largest active support is 12 vertices / 36 controls. Validation covers the small complete fixture, not SF2. These times exclude earlier development probes, test execution and reporting.

### S. Determinism

All 14 main scientific records replay exactly: baseline, support, sensitivities, solver decisions, endpoint coordinates/hashes, hard gates, collision proofs, thickness and conditioning. The separate all-iterate audit and strict seed-compression controls also replay exactly. Runtime and RSS are excluded. Main scientific identity: `{load('DETERMINISM.json')['scientific_hash']}`. The transient projector-round-trip issue in the new adapter is fixed by exact zero-trial rollback and covered by regression testing.

### T. Tests

{tests['summary']}. Exact commands and logs are recorded in TESTS.json and tests.txt. Prior relevant tests are retained. No unrelated full suite or full-shell run is used.

### U. Classification

**{classification}**. Case result: **{case_class}**. The direct cause is identified, but no fully valid layer-5 repair and no general local infeasibility proof is established. A lower solver score or fewer numeric events is insufficient.

### V. Production correction recommendation

Do not integrate these probes as a production repair. The useful result is the complete conflict set: the selected “single-Jacobian” endpoint also has numeric and collision failures. Any later correction study must address both interacting fronts and those constraints together; a cell-only repair cannot qualify this endpoint. This task does not redesign that planner.

### W. Termination-topology readiness

**No.** No direct unavoidable obstruction under the practical correction toolkit is proven. The current negative bounded search does not justify termination topology. No termination policy, topology change or production integration is implemented.
'''
(OUT/'report.md').write_text(report,encoding='utf-8',newline='\n')
sources=['tools/residual_layer5_study.py','tools/seal_residual_layer5_study.py','tests/test_residual_layer5_study.py']
evidence={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in sorted(OUT.iterdir()) if p.is_file() and p.suffix!='.log' and p.name not in ('AUTHORITY.json','HASHES.json')}
s.save('AUTHORITY.json',dict(task='DiagnoseAndRepairV4ResidualLayer5JacobianFailure',input_head=head,classification=classification,
    report_sha256=sha(OUT/'report.md'),source_sha256={p:sha(ROOT/p) for p in sources},evidence_sha256=evidence,
    scientific_identity=load('DETERMINISM.json')['scientific_hash'],prior_authorities=prior,
    production_modified=False,V3_modified=False,termination_implemented=False,infeasibility_proved=False))
s.save('HASHES.json',dict(report=sha(OUT/'report.md'),authority=sha(OUT/'AUTHORITY.json')))
print(json.dumps(load('HASHES.json'),indent=2))
