"""Seal the bounded correction study, including unresolved support limits."""
import json,hashlib,subprocess,re,datetime
import general_local_correction as g
ROOT=g.ROOT;OUT=g.OUT;q=g.q;np=g.np
load=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
start=load(OUT/'START.json');assert start['head']=='6915c6ad853bf45ffd7bf8f17114f8736737d658'
assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==start['head']
verified={}
for name in ('coupled_tangent_feasibility','branch_aware_capacity_study','general_sdf_capacity_witness','layer_capacity_valid_prefix','complete_capacity_fixtures','layer_capacity_forecast','layer_capacity_witness','task4l'):
    p=ROOT/'results'/name;a=load(p/'AUTHORITY.json')
    for group in ('source_sha256','evidence_sha256'):
        for path,h in a[group].items():assert sha(ROOT/path)==h,path
    verified[name]=dict(report=sha(p/'report.md'),authority=sha(p/'AUTHORITY.json'))
assert verified['coupled_tangent_feasibility']==dict(report='34de581a3dd007ad93befd90ee8946b7cf8795298f8ee7880816cd67ae2b5b47',authority='9bc5f8d30498eaaa6a3962bb5a14a170bb614f0125576bf8cd6b26768b5a5c8a')
status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines();other=lambda v:[s for s in v if 'sdf_cfd_mesher_v4' not in s]
assert other(start['status'])==other(status)
assert [s for s in status if 'sdf_cfd_mesher_v3' in s]==[s for s in start['status'] if 'sdf_cfd_mesher_v3' in s]
assert not subprocess.check_output(['git','diff','--name-only','--','research/sdf_cfd_mesher_v3'],text=True).strip()
g.save('AUTHORITY_VERIFICATION.json',dict(prior=verified,V3_modifications=0,unrelated_dirty_entries=len(other(status)),unrelated_unchanged=True,production_changes=0))
for name in ('ANALYTIC_CONTROL_REGRESSION.json','SF2_REGRESSION.json'):
    p=g.c.OUT/name;g.save(name,dict(source=str(p.relative_to(ROOT)),source_sha256=sha(p),unchanged=True,results=load(p)))
names=['general_asymmetric_passage','narrowing_escape','asymmetric_narrowing','curved_passage','narrowing_escape__expanded','curved_passage__expanded']
rows=[load(OUT/n/'result.json') for n in names];r=rows[0]
assert all(load(OUT/n/'determinism.json')['equal'] for n in names)
assert load(OUT/'RECOVERY_DETERMINISM.json')['equal']
timings=[load(OUT/n/t) for n in names for t in ('timing.json','replay_timing.json')]
timings += [load(OUT/'RECOVERY_TIMING.json'),load(OUT/'RECOVERY_REPLAY_TIMING.json')]
total=sum(t['seconds'] for t in timings);peak=max(t['peak_rss'] for t in timings)
g.save('RUNTIME.json',dict(final_run_and_replay_seconds_sum=total,peak_single_process_rss=peak,concurrent_cases=True,aggregate_peak_rss_measured=False,
                         scope='Sum of final bounded case and replay durations. Exploratory probes, setup, tests and sealing excluded. Not task wall time or full-shell runtime.',cases=timings))
log=(OUT/'tests.txt').read_text(encoding='utf-8-sig');m=re.search(r'Ran (\d+) tests in ([0-9.]+)s',log);assert m and '\nOK' in log
g.save('TESTS.json',dict(passed=True,count=int(m[1]),seconds=float(m[2]),command='python -B -m unittest test_general_local_correction test_coupled_tangent_feasibility test_branch_aware_capacity test_general_sdf_witness test_capacity_valid_prefix test_complete_capacity_fixtures test_capacity_forecast test_capacity_witness'))
case=g.c.Case(q.definitions(q.load_schedule().targets)[-1]);cols=case.patch.affected_columns
committed=np.broadcast_to(case.s.targets[:3,None],(3,len(case.x)));targets=np.array(r['local_targets'])
for recover,key in ((False,'schedule_without_recovery'),(True,'schedule_recovery_proposal')):
    assert q.identity(g.w.clean(g.local_schedule(case.s,committed,targets,recover=recover)))==q.identity(r[key])
assert q.identity(g.w.clean(g.plan_future(case.s,committed,targets,2,recover=True)))==q.identity(r['plan'])
g.save('FINAL_SCHEDULE_IDENTITY.json',dict(current_source_matches_final_replayed_schedules=True,nominal_all_layers_bitwise_preserved=True))
np.savez_compressed(OUT/'COLUMN_HEIGHTS.npz',columns=cols,vertices=case.f[cols],committed_heights=np.array(r['schedule_without_recovery']['heights'])[case.f[cols],:3],next_heights=np.array(r['schedule_without_recovery']['adjusted_next_height'])[case.f[cols]],remaining_heights=np.array(r['schedule_without_recovery']['heights'])[case.f[cols],4:],recovery_heights=np.array(r['schedule_recovery_proposal']['heights'])[case.f[cols],4:])
g.save('COLUMN_HEIGHTS.json',dict(scope='Three explicit vertex heights per triangular column; no invented mean offset',columns=len(cols),vertex_targets='general_asymmetric_passage/targets.npy',committed_changed_layers=[4],later_layers_proposed_only=list(range(5,21)),wall_and_h1_unchanged=True))
summary=[dict(name=v['name'],classification=v['classification'],active_vertices=len(v['support']['active_vertices']) if 'support' in v else len(v['active_vertices']),last_valid=v.get('last_valid_layer',4),termination_candidate=False) for v in rows]
g.save('SUMMARY.json',summary)
classification='V4GeneralLocalFrontCorrectionPromising'
g.save('RECOMMENDATION.json',dict(classification=classification,production_adoption=False,bounded_integration_study_justified=True,termination_topology_ready=False,termination_candidates=[],reason='Compression resolves the general-SDF immediate layer. Channel expanded-support foundations and future depth recovery remain unqualified. Failed searches are not capacity proofs.'))
cert=r['accepted_gate']['certificate'];state=r['schedule_without_recovery'];recover=r['schedule_recovery_proposal'];smooth=load(OUT/'RECOVERY_DIAGNOSTIC.json');before=r['conditioning_before'];after=r['conditioning_after']
table='\n'.join(f"| {v['name']} | {v['active_vertices']} | {v['last_valid']} | {v['classification']} |" for v in summary)
report=f'''## Decision question

Can a practical local advancing-front correction framework using standard tangential smoothing, local normal compression, growth redistribution, and bounded multi-layer steering resolve difficult prism regions that tangent-only V4 steering cannot?

### A. Authority verification

HEAD `{start['head']}` and the feature branch match. Eight sealed prior authorities and every referenced source/evidence hash verify unchanged. V3, the production builder, wall geometry, topology, first-layer h1, hard gates and exact certificate code are unchanged. All {len(other(status))} unrelated dirty entries remain unchanged. This is isolated V4 study code.

### B. Existing tangent-only limitation

The preserved general-SDF baseline has 15 exact Jacobian failures and collision 7757/9024. Hash-bound tangent studies found 10 failures with that pair removed, or four with collision retained. They proved neither feasibility nor infeasibility. Those exact-input results are reused; the old tangent program was not needlessly rerun. The current baseline and all three fixed general-SDF prefix layers were independently revalidated over the complete 1,536-cell scientific scope.

### C. Correction-framework architecture

The framework separates fixed committed coordinates, local vertex target offsets, nominal stored schedule, uncommitted proposals and exact admission. The driver tries nominal advancement, existing tangent fields, then scalar normal compression. Geometry is copied for trials. Only a fully passing candidate enters study state. Rejected candidates do not mutate the prefix. The general-SDF case stops at compression because that simpler mechanism resolves its immediate layer. Future recovery is a separate diagnostic.

Locality is defined by existing dependency-complete patches. General-SDF correction uses 44 active vertices, 120 directly affected columns and 36 verification-ring columns. Both collision sides are included. The correction family has one scalar compression variable and a graph-harmonic taper. Existing 88 tangent coordinates remain available but are not needed for this successful candidate. All 1,536 scientific cells are verified; no dense global solve is introduced.

### D. Reused standard tools and algorithms

Installed SciPy 1.15.3 supplies sparse Dirichlet Laplacian smoothing and Brent geometric-progression fitting. Bounded scalar bracketing/bisection supplies the compression search. Existing V4 tangent fields, branch/source projection, dependency patches, primitive metrics, exact Jacobian records and continuous-certificate adapter are reused. Existing frozen component-compression and SLSQP/trust-region code were inspected. Their old margins, sigma guards and complete-future acceptance policy were not imported. No package was installed.

The preference is: pass every existing hard gate and complete exact certificate, then retain the least compression found in the bounded scalar family. No weighted quality score is accepted. Twelve bracket refinements are a computational limit, not a new validity tolerance or proof of global minimality. Feasibility monotonicity outside that tested bracket is not assumed.

### E. Local normal compression

At active vertices the new target is previous target plus the nominal next increment times a harmonic factor. Failure vertices receive the scalar factor; the halo tapers it toward one. Outside the active set the nominal target is unchanged. The source-aware sphere projector enforces each explicit adjusted target under the existing binary64 SDF source-error contract. No alternate distance field is used.

The retained factor is **{r['compression_factor']:.15g}**, or {100*r['compression_factor']:.8f}% of the nominal increment at the most compressed vertices. Minimum next height is **{min(state['adjusted_next_height']):.17g} m**, versus nominal {state['nominal_next_height']:.17g} m. Only layer 4 is changed in the admitted study candidate. Wall and first-layer h1 remain exact. This substantial compression needs CFD spacing review; it is not a qualified production spacing policy. No universal minimum height is invented, and no zero-height candidate is accepted.

### F. Growth redistribution

Two explicit proposals are implemented: scale future nominal increments proportionally, or fit a smooth geometric progression with Brent's method. Both preserve the nominal schedule as an immutable reference. Recovery is proposed, never silently credited as achieved. Proportional recovery has a maximum joining growth ratio of {float(np.max(recover['growth'])):.12g}; the smooth alternative reaches ratio {max(smooth['plan']['ratios']):.12g}. These are diagnostics, not new gates.

The proportional layer-5 preview has {len(r['future_join'].get('exact_failures',[]))} exact Jacobian failures. The smooth geometric preview has {len(smooth['preview']['gate'].get('exact_failures',[]))}. Neither is committed. No full-depth recovery is qualified.

### G. Multi-layer correction

The planner exposes explicit target and height arrays for a caller-selected horizon. This study uses two layers: accepted layer 4 and one recovery join at layer 5. Two is the smallest horizon that tests that join, not a universal planning rule. Both plans preserve committed history and remain uncommitted. A general nonlinear coordinated multi-layer optimizer is not qualified here. It is unnecessary for immediate layer 4 and cannot repair an already-invalid fixed support. Future-plan failure does not revoke valid immediate-layer admission.

### H. General-SDF result

**CompressionResolved.** All 15 exact Jacobian failures are removed without an explicit tangent correction. The successful mechanism is compression plus the existing branch-aware reprojection. Maximum coordinate difference from the original candidate is {r['maximum_coordinate_change']:.17g} m. The maximum adjusted-target residual is {r['target_residual_max']:.17g} m and every point passes the existing SDF admission contract. The checkpoint is study-only; production state is untouched.

### I. Channel results

| Study | Active vertices | Rebuilt valid prefix | Result |
|---|---:|---:|---|
{table}

The table includes initial and expanded support attempts. It does not replace the earlier scientific-core prefix authorities. Narrowing and curved expanded-scope histories rebuild valid layers 1–6, then fail at layer 7. Narrowing is controlled by required side regularity, including side 4231 / edge 19. Curved is controlled by determinant and interpolation weight in the expanded dependencies. Initial compression fields leave some newly controlling vertices fixed; those failed attempts are retained only as support diagnostics.

A second attempt includes those vertices and their complete neighbor dependencies. Both enlarged scopes expose first-layer hard-quality failures. The asymmetric previous support has 25 exactly zero-area lower triangles across all metric-dependency columns, including the 21 previously reported verification-core triangles. It is not used for solving. Rebuilding from the unchanged wall exposes first-layer interpolation-weight failure; enlarging support and testing existing tangent fields does not restore complete validity. Normal compression is forbidden at layer 1. No later-layer solver is run around that invalid fixed support.

Thus the requested channel layers 10, 9 and 8 are **not qualified as corrected**. The bounded support-repair attempts remain SupportBlocked. The task does not weaken ownership, discard required dependencies, change wall triangulation, or label these fixture defects as geometric capacity. A clean channel support under fixed wall/h1 remains required.

### J. Exact hard-gate verification

The accepted general-SDF layer passes positive volume, exact positive Jacobian, determinant >= 0.0011, interpolation weight >= 0.055, skewness <= 3.63636, symmetric volume ratio >= 0.01 and aspect <= 1000. Ordinary topology and ownership checks pass. All current scientific cells, affected dependencies and the verification ring are evaluated. There are zero numeric failure events and zero failures outside the affected scope. Exact acceptance is independent of the scalar search and SciPy algorithms.

### K. Collision results

Pair 7757/9024 is CertifiedSeparated, conservative proof code 2 / outcome 1. The complete immediate certificate covers **{cert['required_pairs']:,}** required relations with {cert['exact_calls']} exact fallbacks, zero forbidden results, zero unresolved results and no invalid required side. The AABB-disjoint complement is covered by the existing conservative proof. Candidate proof identity: `{cert['proof_identity']}`. Pair removal alone was not used as acceptance.

### L. Layer-height and total-thickness accounting

The nominal total is {state['requested_total']:.17g} m. Without future recovery, the most compressed vertex plans {min(state['planned_total']):.17g} m, leaving **{max(state['unrecovered']):.17g} m** unrecovered. No later thickness is claimed recovered. Proposals can sum to nominal depth algebraically, but their next preview is invalid. Actual complete-stack attainability remains unknown.

`COLUMN_HEIGHTS.npz` stores all three vertex heights for each affected triangular column, committed heights, next height, remaining nominal increments and recovery proposals. A mean column height is not substituted for vertex geometry. The local binary64 schedule remains explicit. A regression repair prevents cumulative-offset subtraction from falsely recording layer 2 as changed; nominal committed increments are retained bitwise. The reproducer is saved. This repair changes bookkeeping only, not candidate geometry or gates.

### M. Conditioning

General-SDF weighted sigma minimum changes from {before['minimum_sigma']:.17g} to {after['minimum_sigma']:.17g}. Maximum available G_bound changes from {before['maximum_available_G_bound']:.17g} to {after['maximum_available_G_bound']:.17g}. These diagnostics have no hard threshold. Channel conditioning is not promoted from invalid support states.

### N. Analytic capacity regression

Hash-verified nominal valid/unadmitted boundaries remain parallel 9/10, near-contact 0/1, symmetric 7/8, sphere 12/13 and cylinder 15/16. Both feasible controls remain qualified through 20 layers in sealed evidence. No fixture is rebuilt or changed. Compression does not erase the direct geometric bound on requested total depth; taking smaller layers is not proof of recovering that depth.

### O. SF2 regression

Preserved Task-4L authority and history verify. Zero geometry, correction, layer-count or capacity-authority changes occur. No production module imports the new study framework. Nominal production advancement is unchanged and receives no adjusted heights. SF2 is not rebuilt.

### P. Runtime and locality

Final case runs, independent replays and recovery diagnostics total **{total:.6f} s** when their durations are summed. This is not task wall time; some cases ran concurrently. Peak single-process RSS is **{peak:,} bytes** ({peak/1048576:.2f} MiB). Aggregate concurrent peak RSS was not measured. Exploration, tests and sealing are excluded from that sum. General-SDF compression verification dominates; its timing record separates projection, total verification and included certificate time. No full-shell runtime estimate is made.

### Q. Determinism

All six final case histories replay exactly, including support, trial order, candidate coordinates, target schedules, gates and certificate hashes. Both recovery proposals also replay exactly. Runtime and RSS are separate from scientific identities. Checkpoint geometry and target hashes are persisted. No accepted geometry depends on process completion order.

### R. Tests

{int(m[1])} focused and retained tests pass in {float(m[2]):.3f} s. The exact command is in TESTS.json. Tests cover first-layer protection, compression fields, harmonic neighbors, bitwise schedule bookkeeping, recovery proposals, multi-layer noncommit semantics, fixed-support checks, full exact accepted-candidate verification, collision removal, rollback, failure migration, channel fail-closed behavior, deterministic replay, analytic authority and SF2 nonchange. No unrelated large suite or full SF2 build ran.

### S. Classification

**{classification}**. Normal compression resolves the principal immediate general-SDF failure. Channel support qualification and future total-depth recovery remain incomplete. This is not a claim that all difficult geometries are solved.

### T. Production steering recommendation

A bounded integration study of explicit local normal compression is justified by the valid deterministic general-SDF result. Broad production adoption is **not yet qualified**: repair the channel support contracts and qualify later-layer spacing behavior first. Keep nominal-first behavior, explicit local schedules, exact acceptance and immutable rollback. Production is not modified here.

### U. Termination-candidate result

**No new termination candidate is established.** The general-SDF immediate obstruction is resolved. Channels are support-blocked, not proven incapable. Recovery previews fail but do not prove no recovery exists. Actual termination-topology implementation should not begin on the basis of these unresolved cases. No termination, layer deletion, topology transition, safety margin or layer-count policy is implemented.
'''
(OUT/'report.md').write_text(report,encoding='utf-8',newline='\n')
sources=[ROOT/'tools'/n for n in ('general_local_correction.py','run_general_local_correction.py','diagnose_local_recovery.py','seal_general_local_correction.py')]+[ROOT/'tests/test_general_local_correction.py']
evidence={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in sorted(OUT.rglob('*')) if p.is_file() and p.name not in ('AUTHORITY.json','HASHES.json')}
authority=dict(task='ImplementV4GeneralLocalFrontCorrectionFramework',classification=classification,input_head=start['head'],source_sha256={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in sources},evidence_sha256=evidence,prior_authorities=verified,production_modified=False,V3_modified=False,termination_implemented=False,report_sha256=sha(OUT/'report.md'),scientific_hashes={n:q.identity(r) for n,r in zip(names,rows)})
g.save('AUTHORITY.json',authority);g.save('HASHES.json',dict(report_sha256=sha(OUT/'report.md'),authority_sha256=sha(OUT/'AUTHORITY.json')))
print(classification,load(OUT/'HASHES.json'),flush=True)
