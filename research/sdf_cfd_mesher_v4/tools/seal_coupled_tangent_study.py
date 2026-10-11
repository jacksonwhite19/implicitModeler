"""Seal bounded coupled-feasibility results without promoting search to proof."""
import json,hashlib,subprocess,re,datetime
import coupled_tangent_study as c
ROOT=c.ROOT;OUT=c.OUT;q=c.q
load=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
start=load(OUT/'START.json');assert start['head']=='8fbd53ed8f97ad517dc153f66b0c83586a9ef55b'
verified={}
for folder in ('branch_aware_capacity_study','general_sdf_capacity_witness','layer_capacity_valid_prefix','complete_capacity_fixtures','layer_capacity_forecast','layer_capacity_witness','task4l'):
    path=ROOT/'results'/folder;a=load(path/'AUTHORITY.json')
    for group in ('source_sha256','evidence_sha256'):
        for name,h in a[group].items():assert sha(ROOT/name)==h,name
    verified[folder]=dict(report=sha(path/'report.md'),authority=sha(path/'AUTHORITY.json'))
assert verified['branch_aware_capacity_study']==dict(report='fecddfa3cf8a04f577d7678e279824b68c386f61b8d01260a0628b515102f23c',authority='2f099dffa6acc584bb64258ada1e2a8ce27cbff48060292a16c92eac6005d62e')
status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines();other=lambda rows:[v for v in rows if 'sdf_cfd_mesher_v4' not in v];assert other(start['status'])==other(status)
c.save('AUTHORITY_VERIFICATION.json',dict(authorities=verified,unrelated_dirty_entries=len(other(status)),unrelated_unchanged=True,V3_modifications=0,production_modifications=0))
names=['narrowing_escape','asymmetric_narrowing','curved_passage','general_asymmetric_passage','general_asymmetric_passage__opposed_z','general_asymmetric_passage__opposed_minus_z']
results=[load(OUT/name/'result.json') for name in names];timings=[load(OUT/name/'timing.json') for name in names];replay=[load(OUT/name/'replay_timing.json') for name in names]
for name in names:assert load(OUT/name/'determinism.json')['equal']
diagnostics=load(OUT/'EXACT_DIAGNOSTICS.json');assert load(OUT/'DIAGNOSTIC_DETERMINISM.json')['equal'];dt=load(OUT/'DIAGNOSTIC_TIMINGS.json')
g=results[3];g4=results[4];ga=diagnostics[3];g4a=diagnostics[4]
assert not any(r['feasible'] for r in results)
reg=load(c.d.OUT/'ANALYTIC_CONTROL_REGRESSION.json');c.save('ANALYTIC_CONTROL_REGRESSION.json',dict(source_hash=sha(c.d.OUT/'ANALYTIC_CONTROL_REGRESSION.json'),results=reg,unchanged=True))
sf=load(c.d.OUT/'SF2_REGRESSION.json');assert all(sf[k]==0 for k in ('geometry_changes','correction_changes','layer_count_changes','capacity_limits'))
c.save('SF2_REGRESSION.json',dict(source_hash=sha(c.d.OUT/'SF2_REGRESSION.json'),geometry_changes=0,correction_changes=0,layer_count_changes=0,capacity_authority_changes=0,preserved_history=sf['history']))
log=(OUT/'tests.txt').read_text();test=re.search(r'Ran (\d+) tests in ([0-9.]+)s',log);assert test and '\nOK' in log
c.save('TESTS.json',dict(passed=True,count=int(test[1]),seconds=float(test[2]),command='python -B -m unittest test_coupled_tangent_feasibility test_branch_aware_capacity test_general_sdf_witness test_capacity_valid_prefix test_complete_capacity_fixtures test_capacity_forecast test_capacity_witness'))
refs=[]
for name,r in zip(names,results):
    for model in r['models']:
        path=OUT/name/f"model_{model['iteration']:02}.npz";data=c.np.load(path)
        key=q.identity(dict(u=q.array_hash(data['u']),values=q.array_hash(data['values']),J=q.array_hash(data['J'])));assert key==model['identity']
        refs.append(str(path.relative_to(OUT)).replace('\\','/'))
unreferenced=[str(p.relative_to(OUT)).replace('\\','/') for p in OUT.rglob('model_*.npz') if str(p.relative_to(OUT)).replace('\\','/') not in refs]
c.save('MODEL_INDEX.json',dict(final_replay_matrices=refs,exploratory_matrices=unreferenced,scope='Only final_replay_matrices bind final solver histories. Other preserved matrices belong to earlier exploratory models.'))
summary=[]
for r,a in zip(results,diagnostics):
    gate=r['best_gate'];kind='PositiveExactJacobianOrVolume' if gate.get('exact_failures') else (gate['failures'][0]['metric'] if gate.get('failures') else r.get('model_blocker'))
    summary.append(dict(name=r['name'],classification=r['classification'],active_vertices=len(r['support']['active_vertices']),tangent_dofs=r['support']['tangent_dofs'],exact_basis_rank=r['support']['rank']['independent_rank'],iterations=len(r['history']),candidates=sum('gate'in h for h in r['history']),exact_failures=len(gate.get('exact_failures',[])),controlling_gate=kind,maximum_tangent_displacement=a['maximum_tangent_displacement'],new_numeric_failures=len(a['new_numeric_failures']),capacity_authority=False,feasible=False))
c.save('SUMMARY.json',summary)
peak=max([t['peak_rss'] for t in timings+replay]+[dt['first']['peak_rss'],dt['replay']['peak_rss']]);total=sum(t['seconds'] for t in timings+replay)+dt['first']['seconds']+dt['replay']['seconds']
sensitivity=sum(t['sensitivity_seconds'] for t in timings);verification=sum(t['verification_seconds'] for t in timings);solver_overhead=sum(t['seconds']-t['sensitivity_seconds']-t['verification_seconds'] for t in timings)
c.save('RUNTIME.json',dict(final_first_runs_seconds=sum(t['seconds'] for t in timings),final_replays_seconds=sum(t['seconds'] for t in replay),diagnostic_seconds=dt['first']['seconds']+dt['replay']['seconds'],completed_final_scientific_seconds=total,sensitivity_first_runs_seconds=sensitivity,verification_first_runs_seconds=verification,solver_and_serialization_first_runs_seconds=solver_overhead,peak_rss=peak,scope='Final bounded runs only. Exploratory models, setup, tests, and sealing are excluded; not a production runtime estimate.'))
c.save('RECOMMENDATION.json',dict(classification='V4CoupledTangentFeasibilityPromising',general_result='CoupledTangentNoSolutionFound',production_integration=False,termination_topology_ready=False,general_infeasibility_proved=False,limited_obstruction='21 fixed zero-area lower faces in expanded asymmetric-channel support; does not establish scientific-core capacity',next_step='Resolve branch-constrained feasible-set coverage and bounds for the remaining coupled general-SDF constraints. Repair or restart channel interaction-halo history before claiming coupled channel feasibility. No termination authority.'))
table='\n'.join(f"| {r['name']} | {r['active_vertices']} / {r['tangent_dofs']} | {r['exact_failures']} | {r['classification']} |" for r in summary)
before=ga['conditioning_before'];after=ga['conditioning_after'];after4=g4a['conditioning_after']
report=f'''## Decision question

Does a fully hard-gate-valid local tangent solution exist for the remaining general-SDF and channel failures when all controlling tangent degrees of freedom are solved together?

### A. Authority verification

Input HEAD `{start['head']}` and both supplied branch-study hashes match. Seven prior authorities and their source/evidence bindings verify unchanged. The production builder, projection, hard gates, certificate mathematics, topology, schedule, and V3 are unchanged. {len(other(status))} unrelated dirty entries remain unchanged. Only isolated V4 study tools, tests, and evidence are added.

### B. General-SDF active support

The two sides 7757 and 9024 are current-layer sides. Both upper edge pairs and every vertex of the 15 failed Jacobian cells are active. One topological neighbor ring gives 44 active vertices, 120 directly affected columns, 36 verification-ring columns, and 267 local side faces. Fixed lower vertices belong to committed layer 3 and cannot move. Full acceptance verifies all 1,536 original scientific-core cells plus the dependency-complete local scope. The sensitivity matrix is local; no SF2 or whole-shell dense model exists. Support records include active, fixed boundary, halo, column, side, basis, and ownership IDs.

### C. Tangent basis and rank

The authoritative normalized SDF gradient supplies two tangent basis vectors per active vertex. Exact rational rank of the binary64 basis blocks is 88/88. This is the independent linear DOF-map rank, not proof that branch-constrained nonlinear motions span a free 88-dimensional neighborhood. Source ancestry and branch constraints can restrict the admissible directions. No normal-distance DOF is added.

### D. Coupled sensitivity model

All local cell volume, signed volume, Jacobian, determinant, aspect, face interpolation weight, skewness, and symmetric-volume-ratio precursors are represented. Scaling uses unchanged hard-gate constants and local area times the stored layer increment. Scaled slacks guide search only. Final strict positivity is checked by exact Jacobian arithmetic; LP tolerances never become mesh acceptance tolerances.

The first model differentiated the nonsmooth branch-clamped projector. It predicted improvement that finite candidates did not deliver. That exploratory result is preserved. The final sphere model differentiates the smooth ancestral-sphere radial precursor and imposes nonelastic other-source clearance inequalities. These model samples need not themselves be globally SDF-admitted. Every actual candidate uses the unchanged study branch projector. Channels differentiate their existing projected continuation. Central-difference step is cbrt(binary64 epsilon) in scaled solver coordinates, a numerical differentiation parameter only. Three step-halving probes gave maximum scaled derivative changes from {min(v['maximum_absolute_change'] for v in ga['finite_difference_consistency']):.9g} to {max(v['maximum_absolute_change'] for v in ga['finite_difference_consistency']):.9g}. No validated interval enclosure or exact nonlinear infeasibility claim follows.

### E. Collision-separation sensitivity

The guiding measure is min(side B projection) minus max(side A projection) along the fixed source-center axis. It uses the actual four corners of each bilinear side. Positive separation is useful sufficient geometric guidance, not a replacement for certification and not a necessary condition for all feasible solutions. Sensitivity rows and both side supports are stored.

The zero-start candidate changes pair 7757/9024 from forbidden to **CertifiedSeparated**, conservative proof code 2, outcome 1. The four-Jacobian-failure candidate from the opposed-z start retains a **CertifiedForbiddenIntersection**. These are different candidates. No complete new-layer certificate is claimed for either rejected candidate, and removal of this pair does not establish that every other interaction is clear.

### F. Jacobian feasibility

The baseline has 15 exact failed cells. Individual response rows identify improving and worsening DOFs for each. Three deterministic starts produce 10, 4, and 12 remaining exact failures, respectively. The zero start removes 10 original numeric events but introduces four new Jacobian events within the affected support. The opposed-z candidate introduces one new numeric failure. Neither removes all exact failures. A lower failure count is not acceptance.

### G. Feasibility solver

Deterministic HiGHS dual-simplex LP steps maximize the worst dimensionless local slack. Explicit branch inequalities are not elastic. Tangent step boxes use the existing production geometric radii, min(0.25 stored total thickness, 0.5 shortest wall-neighbor edge), and the frozen two-way trust update. Trust grows after useful boundary steps and shrinks after failed predictions. The overall solver box is a computational search bound, not a new scientific displacement limit; no infeasibility claim covers unsearched motions.

The main cohort is capped at 16 iterations per case; asymmetric setup stops before LP because required inputs are nonfinite. Two independent opposed-z starts use 12 iterations each. Identical current-vector models are reused by exact coordinate identity. Solver histories include LP status, predictions, finite results, trust decisions, gates, and candidate hashes. This is a feasibility search aid, not a new production quality objective.

### H. Exact candidate verification

Finite candidates are branch-reprojected to the unchanged binary64 target. Ordinary exact Jacobians, hard metrics, expanded support, and verification ring decide rejection. Complete immediate certification remains mandatory if those gates pass. None did. Independent verification reconstructs saved candidates from tangent variables and matches all stored gate results. The original pair is separately checked with the qualified proof adapter on all three general-SDF final candidates. No layer was committed; rollback leaves the source candidate unchanged.

### I. General-SDF result

**CoupledTangentNoSolutionFound.** Coupled motion improves failure counts and one candidate separates the original collision, but no single candidate meets all requirements. Maximum tangent displacement is {ga['maximum_tangent_displacement']:.17g} m; zero-start maximum reprojection correction is {ga['maximum_reprojection_correction']:.17g} m. Per-vertex distributions and target residuals are stored. The last valid layer remains 3; layer 4 stays unadmitted. This result does not prove the tangent DOFs insufficient.

### J. Channel results

| Case/start | Active vertices / DOFs | Exact failed cells in expanded check | Result |
|---|---:|---:|---|
{table}

Narrowing and curved-passage best candidates do not move from the projected baseline. The enlarged dependency checks expose existing collateral quality failures beyond the old narrow scientific core. Asymmetric support contains 21 exactly zero-area fixed lower triangles, all outside its original scientific core, and nonfinite required metric inputs. It is reported **Blocked**, not a core capacity limit. With these lower triangles fixed, the bottom-corner Jacobian is exactly zero for any upper point; this is a direct obstruction for this enlarged-support state only. No channel gains a valid continuation. Original sealed boundaries remain 9/10, 8/9, and 7/8.

### K. Failure migration / collateral tails

Full tails are stored for Jacobian, determinant, signed/unsigned volume, aspect, interpolation weight, skewness, and symmetric volume ratio, with entity-level events and nonfinite counts. The zero-start general candidate introduces four Jacobian failures within the affected support; no new failure migrates outside it. Its unchanged exterior coordinates do not make the candidate valid. The four-failure start also retains a collision. Expanded channel failures are distinguished from newly created failures; existing invalid interaction-halo geometry is never exempted.

### L. Conditioning

General-SDF original-core diagnostics (all 1,536 cells): baseline minimum weighted sigma {before['minimum_sigma']:.17g}, maximum available G_bound {before['maximum_available_G_bound']:.17g}. Zero-start candidate: sigma {after['minimum_sigma']:.17g}, G_bound {after['maximum_available_G_bound']:.17g}. Four-failure opposed-z candidate: sigma {after4['minimum_sigma']:.17g}, G_bound {after4['maximum_available_G_bound']:.17g}. These are diagnostics on rejected geometry, not feasible-candidate qualification. No sigma/G threshold or objective is used.

### M. Infeasibility evidence

No general-SDF tangent infeasibility proof exists. Full basis rank, finite-difference sensitivities, failed LP iterations, and three starts do not prove an empty nonlinear feasible set. No validated interval bounds cover all admissible moves. The exact lower-face obstruction in the asymmetric enlarged support is preserved with coordinates and rational cross products; it does not prove geometric capacity exhaustion of the scientific core or exclude a corrected earlier halo history.

### N. Analytic fixture regression

All sealed evidence is unchanged. Valid/unadmitted boundaries remain parallel 9/10, near-contact 0/1, symmetric 7/8, sphere 12/13, cylinder 15/16. Both feasible controls retain their 20-layer qualification. No unnecessary rebuild ran.

### O. SF2 regression

Preserved read-only history verifies zero geometry, correction, layer-count, or capacity-authority changes. No SF2 build ran.

### P. Runtime and locality

Final first runs: {sum(t['seconds'] for t in timings):.6f} s; independent solver replays: {sum(t['seconds'] for t in replay):.6f} s. First-run sensitivity construction: {sensitivity:.6f} s; staged exact verification: {verification:.6f} s; solver plus persistence/other overhead: {solver_overhead:.6f} s. Independent diagnostics and replay: {dt['first']['seconds']+dt['replay']['seconds']:.6f} s. Total completed final measured work: {total:.6f} s. Peak recorded final RSS: {peak} bytes ({peak/1048576:.2f} MiB). These measurements exclude earlier exploratory runs, setup, tests, and sealing and are not a production-runtime estimate. The initial curved exploratory run repeated unchanged models and took 544.47 s; final runs reuse identical models. No scientific result depends on runtime.

### Q. Determinism

All six final solver runs have independent exact scientific replay. Support, basis, matrix identities, LP/trust decisions, coordinates, gates, and classifications match. Diagnostic exact pair results, conditioning, and derivative checks also replay. Runtime and RSS are excluded. MODEL_INDEX.json distinguishes final referenced matrices from retained exploratory matrices. A final review replaced an unconditional trust-boundary flag with the frozen actual-step 0.8 boundary-use rule. Replaying every saved LP confirms all trust decisions and candidate trajectories remain unchanged (TRUST_BOUNDARY_AUDIT.json). No nondeterministic scheduling enters scientific hashes.

### R. Tests

{test[1]} tests passed in {test[2]} s. Command: `{load(OUT/'TESTS.json')['command']}`. Tests cover two-sided support, packed pair indexing, exact tangent rank, sensitivity identity, per-cell response, coupled improvement without false admission, two-way trust, reprojection/rollback, collateral rejection, exact separation, limited lower-face obstruction, channel outcomes, all replays, and SF2 preservation.

### S. Classification

**V4CoupledTangentFeasibilityPromising.** Coordinated moves materially reduce Jacobian failures and can separate the original collision. No fully hard-gate-valid candidate exists among the tested candidates; existence in the full admissible model remains unknown. No general infeasibility or capacity authority is claimed.

### T. Production steering recommendation

**Do not integrate yet.** The evidence supports further bounded branch-constrained feasibility work, but it does not qualify a production optimizer. Missing evidence is a single candidate passing every immediate hard gate and complete certification, or a validated bound proving the entire admissible set empty. Channel support needs a valid interaction-halo history before broader coupled motion can be qualified.

### U. Termination-topology readiness

**No.** Neither a general-SDF feasible solution nor a direct general-SDF infeasibility proof was established. The asymmetric fixed-halo obstruction is too narrow to confer core termination authority. No layer termination, topology change, new physical DOF, or safety margin was implemented.
'''
(OUT/'report.md').write_text(report,encoding='utf-8',newline='\n')
for p in OUT.rglob('*'):
    if p.is_file() and p.suffix in ('.json','.log','.txt','.md'):
        if p.suffix=='.json' and p.stat().st_size>100000:p.write_text(json.dumps(load(p),sort_keys=True,separators=(',',':'),allow_nan=False)+'\n',newline='\n')
        else:p.write_bytes(p.read_bytes().replace(b'\r\n',b'\n'))
sources=['tools/coupled_tangent_study.py','tools/audit_coupled_support.py','tools/audit_coupled_trust.py','tools/verify_coupled_tangent_study.py','tools/seal_coupled_tangent_study.py','tests/test_coupled_tangent_feasibility.py']
evidence={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in sorted(OUT.rglob('*')) if p.is_file() and p.name not in ('AUTHORITY.json','SEAL.json')}
c.save('AUTHORITY.json',dict(task='QualifyV4CoupledLocalTangentFeasibility',input_head=start['head'],classification='V4CoupledTangentFeasibilityPromising',termination_ready=False,production_integration=False,V3_modified=False,source_sha256={s:sha(ROOT/s) for s in sources},evidence_sha256=evidence))
c.save('SEAL.json',dict(report_sha256=sha(OUT/'report.md'),authority_sha256=sha(OUT/'AUTHORITY.json'),persistent_bytes=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file()),sealed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat()))
print((OUT/'SEAL.json').read_text())
