"""Verify frozen authorities, report measured limits, and seal this study."""
import json,hashlib,subprocess,re,datetime
import qualify_branch_aware_capacity as d
q=d.q;OUT=d.OUT;ROOT=d.ROOT
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
load=lambda p:json.loads(p.read_text())
start=load(OUT/'START.json');assert start['head']=='c9a8df33c8b0b38ae1048503b814b11e6da25ec4'
verified={}
for folder in ('general_sdf_capacity_witness','layer_capacity_valid_prefix','complete_capacity_fixtures','layer_capacity_forecast','layer_capacity_witness','task4l'):
    path=ROOT/'results'/folder;authority=load(path/'AUTHORITY.json')
    for group in ('source_sha256','evidence_sha256'):
        for name,h in authority[group].items():assert sha(ROOT/name)==h,name
    verified[folder]=dict(report=sha(path/'report.md'),authority=sha(path/'AUTHORITY.json'))
assert verified['general_sdf_capacity_witness']==dict(report='190e98f58736deb0d6bba2d02b70fe249fc8f9147ac47d1e7f2f5dd594754fbd',authority='dae2308edcae6fb30931a0fcf50ead8e0547e63ff7de190df78b441dd52f2ee1')
status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines();other=lambda rows:[v for v in rows if 'sdf_cfd_mesher_v4' not in v]
assert other(status)==other(start['status'])
d.save('AUTHORITY_VERIFICATION.json',dict(authorities=verified,unrelated_entries=len(other(status)),unrelated_unchanged=True,V3_modifications=0,production_builder_modifications=0))
old=load(q.OUT/'TERMINATION_INPUTS.json');reg=[]
for name,valid,first in [('parallel_gap',9,10),('near_contact',0,1),('symmetric_contact',7,8),('spherical_cavity',12,13),('cylindrical_cavity',15,16),('parallel_wide',20,None),('curved_wide',20,None)]:
    row=next(r for r in old if r['region_id']==name);assert row['last_fully_valid_layer']==valid and row['first_unadmitted_layer']==first
    reg.append(dict(name=name,last_valid=valid,first_unadmitted=first,witness=row['witness'],evidence='Hash-verified sealed valid-prefix history; no rerun'))
d.save('ANALYTIC_CONTROL_REGRESSION.json',reg)
sf=load(d.w.OUT/'SF2_REGRESSION.json');assert all(sf[k]==0 for k in ('geometry_changes','correction_changes','layer_count_changes','capacity_limits'))
d.save('SF2_REGRESSION.json',dict(source=str((d.w.OUT/'SF2_REGRESSION.json').relative_to(ROOT)),sha256=sha(d.w.OUT/'SF2_REGRESSION.json'),geometry_changes=0,correction_changes=0,layer_count_changes=0,capacity_limits=0,history=sf['history'],evidence='Read-only reuse after complete sealed evidence/source hash verification'))
summary=load(OUT/'SUMMARY.json');audit=load(OUT/'BRANCH_AUDIT.json');t=load(OUT/'TIMINGS.json');rt=load(OUT/'REPLAY_TIMINGS.json');at=load(OUT/'AUDIT_TIMINGS.json');det=load(OUT/'DETERMINISM.json');assert det['equal'] and load(OUT/'AUDIT_DETERMINISM.json')['equal']
g=load(OUT/'general_asymmetric_passage.json');ga=audit[-1];cases=[load(OUT/(r['name']+'.json')) for r in summary]
log=(OUT/'tests.txt').read_text();match=re.search(r'Ran (\d+) tests in ([0-9.]+)s',log);assert match and '\nOK' in log
d.save('TESTS.json',dict(count=int(match[1]),seconds=float(match[2]),passed=True,command='python -B -m unittest test_branch_aware_capacity test_general_sdf_witness test_capacity_valid_prefix test_complete_capacity_fixtures test_capacity_forecast test_capacity_witness'))
records=[]
for r in cases:
    records.append(dict(region_id=r['name'],branch_source_identity=r['source_identity'],last_valid_layer=r['last_valid_layer'],first_unadmitted_layer=r['first_unadmitted_layer'],first_warning_layer=0,capacity_authority=False,direct_unavoidable_witness=None,candidate_hash=r['projected_coordinate_hash'],classification=r['classification'],steerability='NoValidCandidateInBoundedStudy',remaining=q.schedule_demand(q.load_schedule(),r['last_valid_layer']),termination_authority=False))
d.save('LOCAL_PLANNER_RECORDS.json',records)
d.save('RECOMMENDATION.json',dict(classification='V4BranchAwareContinuationPromising',production_integration=False,termination_topology_ready=False,false_capacity_claims=0,next_task='Local branch-consistent multi-vertex feasibility study on the two overlapping sphere fronts, with complete immediate gates and bounded exact interaction proofs. Keep channel interaction-halo failures separate.'))
d.save('STUDY_LIMITS_AND_REPAIRS.json',dict(production_changes=0,limits=['All source-distance admission uses the existing floating SDF error bound; no symbolic equality of projected coordinates is claimed.','Sphere construction enumerates floating active-set candidates and checks them against the authoritative SDF; failure of enumeration is not a level-set absence proof.','Channel predictor-corrector observes feature ownership; it does not qualify a universal branch constraint for arbitrary SDFs.','Sixteen asymmetric-channel trials failed during projection/evaluation; no steering completeness is claimed.','Full diagnostic certificate on rejected geometry was manually interrupted after slow exact work; it confers no evidence. The final fixed one-pair diagnostic is independently replayed.','Full scientific-core immediate certification remains mandatory for any acceptance; no candidate passed earlier gates.'],repairs=[dict(cause='Initial cohort filter included analytic spherical cavities in the exterior-sphere construction',fix='Exclude sealed cavity controls from new cohort; reject cavity input explicitly',test='test_cavity_not_silently_exterior'),dict(cause='Diagnostic numeric metrics included elapsed seconds in initial replay identity',fix='Remove timing field before scientific hashing',test='test_scientific_replay')]))
peak=max(t['peak_rss'],rt['peak_rss'],at['first']['peak_rss'],at['replay']['peak_rss']);total=t['seconds']+rt['seconds']+at['first']['seconds']+at['replay']['seconds']
table='\n'.join(f"| {r['name']} | {r['last_valid_layer']} | {r['first_unadmitted_layer']} | {r['classification']} | {a['steering_counts']['evaluated']} / 104 |" for r,a in zip(summary,audit))
proof=ga['diagnostic_certificate']['checks'][0]['result']['fallback'][0]['result']
numeric=g['diagnostic_metrics_after_rejection'];counts={}
for e in numeric['events']:counts[e['metric']]=counts.get(e['metric'],0)+1
projection=sum(v['projection_seconds'] for v in t['cases']);steering=sum(v['steering_seconds'] for v in t['cases']);certificate=at['first']['certificate_seconds']
report=f'''## Decision question

Are the remaining general-SDF and channel failures caused by inadequate branch-aware continuation, or by genuine unavoidable local geometric obstruction?

### A. Authority verification

Input HEAD: `{start['head']}`. All source and evidence bindings in six prior authorities verified, including the user-specified report and authority hashes. No prior result, V3 file, production projector, builder, gate, or certificate kernel changed. {len(other(status))} unrelated dirty entries remain unchanged. This is study-only code under V4 tools.

### B. Ownership-transition diagnosis

All 16 affected general-SDF vertices reproduce the ordinary path. Convergence takes {min(ga['ordinary_iterations'])}–{max(ga['ordinary_iterations'])} Newton updates. The authoritative SDF is the minimum of three sphere distances. At overlapping offset balls, the nearest owner alternates: {min(ga['ordinary_owner_switches'])}–{max(ga['ordinary_owner_switches'])} switches per affected vertex. Correcting one sphere distance can violate the other. Gradients remain nonzero. Target solutions exist; the old 64-update failure was operational, not proof of absent geometry.

The transitions are classified BranchAmbiguous. Intersecting offset branches admit more than one source-consistent direction. A nearest-owner change alone does not prove an unintended jump, nor a smooth ownership-only representation change. Complete ordinary iteration traces and eight-step source-constrained traces are stored for every affected vertex.

### C. Branch identity

Branch identity hashes the SDF definition/version and source primitive ID. Wall ancestry fixes the source. IDs do not depend on objects or allocation. Wedges expose normalized plane owners; frusta expose meridian segment owners. Unknown SDF constructions fail explicitly. Owner ties use deterministic source ordering; source residual admission, not a branch-distance threshold, detects a source-inconsistent jump.

### D. Branch-aware projection

The bounded study uses eight target-continuation steps. Each final target is the unchanged stored binary64 schedule value. For the composite spheres, candidate projection enumerates the source offset sphere, source-pair intersection circles, and triple intersections. It selects an SDF-admitted source-consistent point nearest the predictor with canonical ties. Intermediate steps are numerical continuation, not new mesh layers or physical DOFs.

All projected vertices satisfy the existing authoritative SDF error bound, have finite coordinates and valid gradients. Every general-SDF point retains its source constraint. This means target-distance admission to the existing numerical authority, not exact symbolic equality of a binary64 point to an irrational surface. No new tolerance is used. Degenerate/unavailable candidate constructions fail closed and do not prove absence. Channels use eight ordinary Newton continuation stages with the existing residual admission check; feature ownership is traced rather than frozen across legitimate feature transitions.

### E. General-SDF layer-4 result

The operational convergence problem is removed in eight bounded source-projection steps. The reconstructed layer still has 15 exact nonpositive-Jacobian cells, the same count as the 256-cap ordinary candidate. Last valid layer remains 3; layer 4 is unadmitted. Diagnostic numeric calculations after rejection report event counts {counts}; complete cell/face values are stored. These calculations confer no admission.

One fixed nearest cross-component current-side pair, 7757/9024, among {ga['diagnostic_certificate']['candidate_pairs']} AABB survivors gives `{proof['classification']}` by `{proof['method']}`. Surfaces, labels, algebraic result, and proof are stored and replayed. This is an exact rejection witness for this SDF-admitted candidate, not a proof that all possible continuations collide. A full diagnostic certificate on rejected geometry was interrupted; no full-candidate completeness claim is made.

### F. General-SDF steering result

The existing 13 tangent fields and eight amplitude choices generated 104 trials, all evaluated. None passes. The lowest observed exact-failure count is {ga['steering_counts']['minimum_exact_failures_on_evaluated_trial']}; this is not a valid continuation. Source-aware reprojection is applied after tangent motion; no normal-distance freedom is introduced. Input candidate state remains unchanged after rejection. Staging stops before certification when an earlier mandatory gate fails.

### G. Channel branch analysis

| Fixture | Last valid | Unadmitted | Classification | Evaluated trials |
|---|---:|---:|---|---:|
{table}

The first three rows are the channel cohort. Narrowing retains four exact-Jacobian failures; asymmetric narrowing retains required interaction-side irregularity; curved passage retains a determinant violation. Core owner changes are {len(audit[0]['changed_core_owners'])}, {len(audit[1]['changed_core_owners'])}, and {len(audit[2]['changed_core_owners'])}, respectively. Thus the curved failure is not explained by a nearest-feature switch. Wedge feature changes remain geometrically possible; this bounded study does not establish a universal ancestry constraint there. No channel gained valid continuation. Asymmetric trials include 16 projection/evaluation errors; narrowing has 24 duplicate candidates and asymmetric has 7. Do not equate 104 attempts with 104 independent feasible directions.

### H. Direct obstruction study

No unavoidable obstruction is proved. Prior offset-ball convergence remains a diagnostic witness. The new exact collision concerns one admitted candidate only. Neither that collision nor failed steering establishes target-level-set absence, zero thickness for every continuation, or global tangent infeasibility. Capacity authority and false capacity claims remain zero. General-SDF classification stays GeneralSDFWarningOnly.

### I. Tangent-space feasibility

Exact dyadic bottom-corner derivatives were evaluated for all three corners of each failed exact cell: 45 general-SDF corner records and 12 narrowing records. Two general-SDF bottom corners are nonpositive, and both have nonzero exact tangent derivatives. This rules out a zero-linear-control explanation for those two corners; it does not prove simultaneous finite-move feasibility. Other failures can occur at other wedge Jacobian extrema. No global dense sensitivity or TangentSpaceObstructionWitness was constructed.

### J. Analytic-fixture regression

Hash-verified valid/unadmitted boundaries remain parallel 9/10, near-contact 0/1, symmetric 7/8, sphere 12/13, cylinder 15/16. Witness types are preserved in ANALYTIC_CONTROL_REGRESSION.json. Both feasible controls retain all 20 qualified layers. No unnecessary fixture rebuild was performed.

### K. SF2 regression

Read-only sealed history: zero geometry, correction, or layer-count changes; zero capacity-limit authority. All Task-4L evidence bindings remain unchanged. No full shell build ran.

### L. Runtime and locality

Measured cohort: {t['seconds']:.6f} s; independent replay: {rt['seconds']:.6f} s. First-run projection: {projection:.6f} s; steering: {steering:.6f} s. Bounded diagnostic audit: {at['first']['seconds']:.6f} s, including {certificate:.6f} s exact diagnostic work; audit replay: {at['replay']['seconds']:.6f} s. Sum of these completed measured runs: {total:.6f} s. These figures exclude interrupted exploratory diagnostics and setup/report work; they are not total task time or a production benchmark. Peak measured RSS: {peak} bytes ({peak/1048576:.2f} MiB) across recorded completed runs. Active vertex/support IDs are stored per trial. Work uses four local frozen fixtures only.

### M. Determinism

Two full cohort runs and two diagnostic audits match exactly after excluding time/RSS. Cohort scientific hash: `{det['scientific_hash']}`. Identity covers projected coordinates, ownership traces, staged decisions, exact gates, numeric diagnostics, and classifications. The audit separately binds exact collision evidence. No geometry commits occurred.

### N. Tests

{match[1]} tests passed in {match[2]} s. Command: `{json.loads((OUT/'TESTS.json').read_text())['command']}`. Tests cover source identity, active-circle projection, source-jump rejection, target admission, replay, rollback, cavity rejection, channel results, exact candidate collision scope, tangent evidence, and sealed SF2 regression. Earlier V4 capacity tests were retained and run. Development repairs affected only new driver scope and removal of timing from scientific hashes.

### O. Classification

**V4BranchAwareContinuationPromising**. Deterministic ancestry-aware projection resolves the operational target-reaching problem and supplies stronger candidate-specific collision evidence. It does not restore a valid mesh or establish unavoidable capacity. The primary scientific continuation question remains unresolved; no successful layer continuation is claimed.

### P. Production integration recommendation

**No.** Keep this projector study-only. The explicit construction applies to the exposed three-sphere authority, not arbitrary SDFs. There is no evidence that it improves valid mesh continuation, and channel branch constraints remain incomplete. Next investigate bounded coupled tangent feasibility on the overlapping fronts with the existing gates; avoid another projection-cap-only change.

### Q. Termination-topology readiness

**No.** The general-SDF case still lacks either a valid continuation or an unavoidable obstruction proof. Deterministic local records explicitly carry no termination authority. No termination, topology, safety-margin policy, or layer-count change is implemented.
'''
(OUT/'report.md').write_text(report,encoding='utf-8',newline='\n')
# Compact large evidence without changing loaded scientific values.
for p in OUT.rglob('*.json'):
    if p.stat().st_size>100000:p.write_text(json.dumps(load(p),sort_keys=True,separators=(',',':'),allow_nan=False)+'\n',newline='\n')
sources=['tools/branch_aware_projection_study.py','tools/qualify_branch_aware_capacity.py','tools/audit_branch_aware_capacity.py','tools/seal_branch_aware_capacity.py','tests/test_branch_aware_capacity.py']
evidence={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in sorted(OUT.rglob('*')) if p.is_file() and p.name not in ('AUTHORITY.json','SEAL.json')}
d.save('AUTHORITY.json',dict(task='QualifyV4BranchAwareProjectionAndSteering',input_head=start['head'],classification='V4BranchAwareContinuationPromising',production_integration=False,termination_ready=False,V3_modified=False,source_sha256={p:sha(ROOT/p) for p in sources},evidence_sha256=evidence))
d.save('SEAL.json',dict(report_sha256=sha(OUT/'report.md'),authority_sha256=sha(OUT/'AUTHORITY.json'),sealed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),persistent_bytes=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file())))
print((OUT/'SEAL.json').read_text())
