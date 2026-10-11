"""Seal a negative bounded recovery result without creating capacity authority."""
import hashlib,json,subprocess,re
import compressed_layer_recovery as r
q=r.q;np=r.np;ROOT=r.g.ROOT;OUT=r.OUT
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
start=r.load(OUT/'START.json');head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip();assert head==start['head']=='d80a332a7551594683014e91f6163661275dc17e'
assert subprocess.check_output(['git','branch','--show-current'],text=True).strip()=='feature/sdf-cfd-mesher-v4-advancing-front'
prior={}
for name in ('general_local_front_correction','coupled_tangent_feasibility','branch_aware_capacity_study','general_sdf_capacity_witness','layer_capacity_valid_prefix','complete_capacity_fixtures','layer_capacity_forecast','layer_capacity_witness','task4l'):
    folder=ROOT/'results'/name;a=r.load(folder/'AUTHORITY.json')
    for group in ('source_sha256','evidence_sha256'):
        for path,h in a[group].items():assert sha(ROOT/path)==h,path
    prior[name]=dict(report=sha(folder/'report.md'),authority=sha(folder/'AUTHORITY.json'))
assert prior['general_local_front_correction']==dict(report='a2ed0d487cab3854ac0592b856e9a67bf6188a95c90315fe23dc4ce186813af9',authority='db44c531212afafc8064536553c3f9e6b394212c2c2bad2b8081ade8be8f8134')
status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines();other=lambda ss:[s for s in ss if 'sdf_cfd_mesher_v4' not in s]
assert other(status)==other(start['status']);assert not subprocess.check_output(['git','diff','--name-only','--','research/sdf_cfd_mesher_v3'],text=True).strip()
r.save('AUTHORITY_VERIFICATION.json',dict(prior=prior,V3_task_modifications=0,unrelated_dirty_entries=len(other(status)),unrelated_unchanged=True,production_modified=False))
data=r.load(OUT/'result.json');coupled=r.load(OUT/'COUPLED_PROBE.json');height=r.load(OUT/'HEIGHT_PROBE.json');seed=r.load(r.g.OUT/'general_asymmetric_passage/result.json')
assert data['last_valid']==4 and not any(v['committed'] for v in data['layers'])
for name in ('DETERMINISM.json','COUPLED_DETERMINISM.json','HEIGHT_DETERMINISM.json','ASSIGNED_SECANT_DETERMINISM.json'):assert r.load(OUT/name)['equal']
folder=r.g.OUT/'general_asymmetric_passage'
r.save('checkpoint_04.json',dict(private_base=True,production_modified=False,coordinate_file=str((folder/'candidate.npy').relative_to(ROOT)),coordinate_sha256=sha(folder/'candidate.npy'),target_file=str((folder/'targets.npy').relative_to(ROOT)),target_sha256=sha(folder/'targets.npy'),qualified_seed=data['seed']))
r.save('LAYER_STATUS.json',[dict(layer=4,status='QualifiedCompressedSeed',passed=True),dict(layer=5,status='RejectedUncommitted',passed=False)]+[dict(layer=k,status='NotAttemptedInvalidImmediatePredecessor',passed=None) for k in (6,7,8)])
case=r.c.Case(q.definitions(q.load_schedule().targets)[-1]);history=[np.full(len(case.x),v) for v in case.s.targets[:3]]+[np.array(seed['local_targets'])];debt=case.s.targets[3]-history[-1]
r.save('THICKNESS_ACCOUNTING.json',r.accounting(case.s,history,debt))
r.save('CONDITIONING.json',dict(layer4=seed['conditioning_after'],accepted_later_layers=[],scope='No conditioning trajectory exists beyond layer 4. Rejected candidates are not committed evidence.'))
r.save('NEIGHBOR_TRANSITIONS.json',dict(seed=r.neighbors(case,history[-1],history[-1]-history[-2]),later_transitions_qualified=False,hard_smoothness_threshold=None))
e=data['expansions'][0];patch=q.bounded_patch(case.top,np.array(e['active']))
r.save('ACTIVE_SUPPORT.json',dict(initial=case.support,expansions=data['expansions'],height_control_vertices=e['centres'],tangent_control_vertices=e['active'],halo=patch.halo,fixed_outer_boundary=patch.fixed_boundary,verification_ring=patch.verification_ring,affected_columns=patch.affected_columns,verified_cells=1536,maximum_tangent_dofs=170,wall_unchanged=True))
rows=[];minimum_height=float('inf');maximum_debt=0.
for a in data['layers'][0]['attempts']:
    if a['expansion']==0:active=case.active;centres=case.support['controlling_vertices']
    else:active=np.array(e['active']);centres=e['centres']
    weights=r.g.harmonic_weights(case.top,centres,active);p=r.plan(case.s,history,'receding',a['horizon'],weights)
    targets=history[-1]+p['height']*(1-(1-a['factor'])*weights);targets[weights==0]=case.s.targets[4]
    assert q.array_hash(targets)==a['target_hash']
    minimum_height=min(minimum_height,float(min(targets-history[-1])));maximum_debt=max(maximum_debt,float(max(case.s.targets[4]-targets)))
    rows.append(dict(horizon=a['horizon'],factor=a['factor'],expansion=a['expansion'],minimum_height=float(min(targets-history[-1])),maximum_unrecovered=float(max(case.s.targets[4]-targets)),exact_failures=len(a['result'].get('gate',{}).get('exact_failures',[])),committed=False))
r.save('PATHOLOGICAL_CANDIDATES.json',dict(trials=rows,minimum_tested_positive_height=minimum_height,maximum_tested_debt=maximum_debt,near_zero_trend='Successively halved computational trial increments; no minimum physical-height gate',repeated_compression_across_committed_layers=False,capacity_authority=False))
r.save('FUTURE_WARNINGS.json',dict(layer5=[dict(strategy=v['strategy'],horizon=v['horizon'],exact_failures=len(v['result']['gate'].get('exact_failures',[]))) for v in data['comparison']],later_forecast_refreshes=0,reason='No post-seed layer admitted; no claimed layer-6-to-8 trajectory'))
for name in ('SF2_REGRESSION.json','ANALYTIC_CONTROL_REGRESSION.json'):
    p=r.g.OUT/name;r.save(name,dict(unchanged=True,source=str(p.relative_to(ROOT)),source_sha256=sha(p),evidence=r.load(p)))
r.save('SERIALIZATION_REPRODUCER.json',dict(defect='TypeError: Object of type ndarray is not JSON serializable',cause='Coupled adapter replaced normalized scalar target with a local NumPy target array before replay hashing',repair='Normalize the final adapter record through existing clean() before hashing',scientific_coordinates_changed=False))
main=r.load(OUT/'TIMING.json');replay=r.load(OUT/'REPLAY_TIMING.json');cp=r.load(OUT/'COUPLED_TIMING.json');cr=r.load(OUT/'COUPLED_REPLAY_TIMING.json')
peak=max(t['peak_rss'] for t in (main,replay,cp,cr));total=sum(t['seconds'] for t in (main,replay,cp,cr))
r.save('RUNTIME.json',dict(main_seconds=main['seconds'],main_replay_seconds=replay['seconds'],coupled_seconds=cp['seconds'],coupled_replay_seconds=cr['seconds'],sum_final_main_and_coupled_seconds=total,peak_single_process_rss=peak,plan_seconds_replay=replay['recovery_plan_seconds'],tangent_inclusive_seconds_replay=replay['tangent_inclusive_seconds'],projection_seconds_replay=sum(t['projection'] for t in replay['stages']),hard_gate_seconds_replay=sum(t['hard_gates'] for t in replay['stages']),new_candidate_certificate_seconds_replay=sum(t['certificate'] for t in replay['stages']),scope='Individual bounded process timings; not task wall time. Seed verification is included in main total, not new-candidate stages. Height probes have separate stage timings. Exploration and tests excluded; cases sometimes concurrent.'))
log=(OUT/'tests.txt').read_text();m=re.search(r'Ran (\d+) tests in ([0-9.]+)s',log);assert m and '\nOK' in log
command='python -B -m unittest test_compressed_layer_recovery test_general_local_correction test_coupled_tangent_feasibility test_branch_aware_capacity test_general_sdf_witness test_capacity_valid_prefix test_complete_capacity_fixtures test_capacity_forecast test_capacity_witness'
r.save('TESTS.json',dict(count=int(m[1]),passed=True,seconds=float(m[2]),command=command))
classification='V4CompressedLayerRecoveryInsufficient'
r.save('RECOMMENDATION.json',dict(classification=classification,accepted_after_layer4=0,production_integration=False,termination_topology_ready=False,termination_candidate=False,infeasibility_proved=False,general_result='No valid continuation established in the bounded search',result_class_note='No successful trajectory class applies. This is neither a technical blocker nor a proven recovery-exhaustion witness.',next_step='Isolate the layer-5 cell 1232 and neighboring quality conflicts under joint local height and tangent constraints; the tested families do not prove infeasibility.'))
maxmove=max(z.get('maximum_tangent_displacement',0) for z in coupled['history'])
table='\n'.join(f"| {v['strategy']} | {v['horizon']} | {len(v['result']['gate'].get('exact_failures',[]))} | rejected |" for v in data['comparison'])
report=f'''## Decision question

Can V4 follow a locally compressed valid prism layer with several fully valid advancing-front layers that smoothly recover toward the nominal schedule without introducing new quality or collision failures?

### A. Authority verification

Input HEAD `{head}` and the feature branch match. The supplied report and authority hashes match. Nine prior authority manifests and all their source/evidence bindings verify unchanged. V3 has zero task modifications. All {len(other(status))} pre-existing dirty entries are preserved, including the untracked V3 directory. The wall, first layer, topology, gates, certificate mathematics and production runner are unchanged.

### B. Qualified compressed seed

The exact layer-4 coordinate and target hashes replay. Source-branch projection reproduces its coordinates bitwise. All immediate hard gates and the full 43,162-relation exact certificate reproduce, with zero forbidden or unresolved results. The private checkpoint references the sealed coordinate and target files rather than duplicating them. Layer 4 is not reoptimized. Its minimum increment remains 36.72735100661176 micrometres; maximum unrecovered thickness remains {max(debt)*1000:.15g} mm.

### C. Recovery architecture

Private state separates committed fronts, actual target offsets, nominal binary64 reference and provisional future plans. Only an independently validated immediate layer can append to that state. Candidate projection uses the existing source-aware sphere projector; ordinary V4 gates and complete immediate certification remain the only admission path. Rejection leaves seed coordinates and schedules untouched. No production integration is made.

### D. Local layer-height state

Each vertex has an explicit offset, increment, growth ratio, nominal reference and debt. A triangular column retains its three vertex values; no synthetic mean height replaces geometry. Accounting reports recovered thickness relative to the seed. Recovery percentage is unavailable when initial lost thickness is zero. There are no accepted offsets after layer 4, so actual recovered thickness is **zero** and actual debt is unchanged.

### E. Recovery strategies

Immediate control uses the next stored nominal increment without forcing catch-up. Uniform recovery adds one horizon share of the lost thickness. Smooth recovery approaches the nominal increment over the chosen horizon. Neighbor-aware recovery uses the existing graph-harmonic transition. Receding recovery replans from actual committed offsets and can reduce its next local increment. Horizons 2, 3, 4 and 5 are computational study choices, not production deadlines.

Plans contain explicit provisional increments and targets through their horizon, but only the first entry is tested for admission. The receding implementation is unit-tested; repeated replanning after an actual later commit is **not scientifically qualified**, because no later layer passes. Existing tangent fields, the existing deterministic local HiGHS tangent solver, previous-valid-displacement predictors and a bounded individual-height pattern search provide additional local probes. No heavy dependency or new physical DOF is introduced.

### F. Horizon comparison

| Strategy | Horizon | Layer-5 exact failed cells | Admission |
|---|---:|---:|---|
{table}

All initial recovery plans fail. Longer smooth horizons reduce exact failures but do not achieve validity. The receding scalar search tests 72 combinations across the original and expanded support. Its best trials retain one failed cell, column 1232. No success is inferred from fewer failures or a better solver score.

### G. Per-layer advancement

Layer 4 remains the last valid private layer. Layer 5 remains uncommitted. Layers 6, 7 and 8 are **not attempted**, because their required valid predecessor could not be produced. Advancing directly to those layers would skip the required rolling validation. There are no later accepted checkpoints, heights or recovery trajectory to report. LAYER_STATUS.json records this explicitly.

### H. Hard-gate results

The seed passes every frozen gate. All tested layer-5 candidates fail admission. Initial ordinary recovery has 26 exact Jacobian failures. Expanded compression improves that count to one, but does not remove it. Standard tangent fields test 120 candidates. The coupled 170-DOF solve performs 12 iterations and its selected final candidate has {len(coupled['best_gate']['exact_failures'])} exact failed cells; its improving linearized score does not qualify geometry. Individual-height changes on the three controlling vertices test 18 candidates without removing the remaining failure.

Rejected exact-Jacobian cases stop at that mandatory gate. They are not described as passing other gates that were skipped. Coupled diagnostic tails record collateral failures. Every candidate that could pass would still require the complete production numerical and exact acceptance path.

### I. Exact certification

The seed certificate is re-executed and exactly matches its sealed proof identity. No new layer-5 complete certificate is claimed, because no candidate passes the preceding hard gates. Zero later layers are admitted, rather than admitting an uncertified layer. Removal of a selected pair is never substituted for complete coverage.

### J. Neighbor-column transitions

The initial 44 active vertices expand to 85 when controlling rejected-candidate cells require it. The expanded group contains 35 height-control vertices plus a harmonic transition. It has 170 nominal tangent DOFs. Active, fixed outer, halo, affected-column and verification-ring identities are stored. All 1,536 fixture scientific cells remain in final verification scope. This is a small fixture-local problem, not a full-shell planner.

Target and increment differences across actual neighboring edges are recorded without a new smoothness gate. No transition beyond the seed qualifies. The maximum coupled trial tangent displacement is {maxmove:.17g} m; accepted post-seed displacement is zero. The secant predictor also tests measured SDF targets. Every measured-height variant has nonpositive increments at some vertices and is rejected; the assigned-positive-height variant still leaves exact failures.

### K. Thickness recovery

The requested nominal total remains 0.020000000131704564 m. Seed maximum debt is {max(debt):.17g} m. Actual thickness recovered after the seed is **0 m (0% where initial debt is nonzero)**. The nominal schedule is not rejoined. Algebraic recovery in a proposed schedule is not physical recovery.

The smallest positive increment tested by scalar recovery is {minimum_height:.17g} m. The largest proposed debt is {maximum_debt:.17g} m. These directly measured trial values show compression tending downward and debt increasing; they define no universal minimum height or compression limit. No near-zero trial is admitted. Repeated compression across committed later layers and practical stability remain unmeasured because no later layer commits.

### L. Future warning behavior

Layer-5 warning populations are stored for each horizon. Later forecast entries remain provisional. There are zero post-seed forecast refreshes driven by an accepted layer. No claim is made that a fixed schedule solves layers 6–8 or the remaining 20-layer request. Failure of these local searches is not evidence that every possible trajectory is absent.

### M. Conditioning

The last accepted conditioning remains the compressed seed: weighted sigma minimum **0.12093646405001122**, maximum available G_bound **25.253773477046682**. No accepted later conditioning trajectory exists. Neither diagnostic is used as a hard threshold or as a reason to reject an otherwise valid mesh.

### N. Analytic capacity regression

Sealed analytic valid/unadmitted boundaries remain parallel 9/10, near-contact 0/1, symmetric 7/8, sphere 12/13 and cylinder 15/16. Their direct geometric witnesses and source hashes are unchanged. No fixture is forced to succeed by shrinking layers, and no nominal-depth capacity authority is erased. No new termination candidate is established by this study.

### O. SF2 regression

Task-4L evidence and source bindings verify unchanged. Wall, h1, geometry, correction decisions, layer count and capacity authority have zero changes. New code has no production call site. Nominally valid production regions therefore receive no compression or termination recommendation from this study. SF2 is not rerun. Channels are not rebuilt: the prerequisite of a qualified recovery strategy and clean channel support is unmet.

### P. Runtime and locality

Main comparison/search: {main['seconds']:.6f} s; independent replay: {replay['seconds']:.6f} s. Coupled probe: {cp['seconds']:.6f} s; replay: {cr['seconds']:.6f} s. These four durations sum to {total:.6f} s, not task wall time. Height probes have separate stage timings. Exploration, test and sealing time are excluded. Some processes ran concurrently.

The instrumented main replay spends {replay['recovery_plan_seconds']:.6f} s constructing recovery plans, {sum(t['projection'] for t in replay['stages']):.6f} s in candidate projection and {sum(t['hard_gates'] for t in replay['stages']):.6f} s in hard-gate work. Tangent probing takes {replay['tangent_inclusive_seconds']:.6f} s inclusive of its projection and validation. New-candidate certification time is {sum(t['certificate'] for t in replay['stages']):.6f} s; seed recertification is included separately in main total. Peak measured single-process RSS is {peak:,} bytes ({peak/1048576:.2f} MiB); aggregate concurrent RSS is not claimed.

### Q. Determinism

Main plans, local targets, candidate coordinates, ownership records, rejection histories, expanded support and seed proof identity replay exactly. Coupled and individual-height probes also replay exactly. Runtime and RSS are excluded from scientific hashes. A replay serialization defect was repaired: a NumPy local-target array must be normalized with the existing clean() function before hashing. Its reproducer and regression are preserved; scientific coordinates did not change.

### R. Tests

**{int(m[1])} tests pass** in {float(m[2]):.3f} s. TESTS.json gives the exact command. Tests cover seed replay, all horizons, provisional planning, receding state updates, neighbor targets, branch projection, rollback, rejection, exact seed certification, debt persistence, zero-denominator percentages, pathological trials, coupled-score nonacceptance, support expansion, serialization, determinism and preserved analytic/SF2 evidence. Receding state-update unit tests are not passed off as a validated physical recovery trajectory.

### S. Classification

**{classification}**. The fixed qualified compression seed remains valid, but this bounded framework does not establish practical post-seed continuation. No successful general-SDF trajectory class applies. This is a negative feasibility-search result, not a technical infrastructure blocker and not a proof of unavoidable geometric exhaustion.

### T. Production integration recommendation

**No.** The requirement for several subsequent fully valid layers is unmet. Do not integrate compression/recovery into the production SF2 runner. Isolate the persistent layer-5 cell 1232 and neighboring quality conflicts under joint local height/tangent constraints. The present finite trials and linearizations do not prove the admissible set empty.

### U. Termination-topology readiness

**No new justification from this study.** True analytic capacity fixtures remain limited, but the general-SDF distinction between recoverable geometry and unavoidable obstruction is not resolved. Failed recovery searches alone do not authorize termination. No topology change, layer deletion, safety margin, first-layer change or termination policy is implemented.
'''
(OUT/'report.md').write_text(report,encoding='utf-8',newline='\n')
sources=[ROOT/'tools'/name for name in ('compressed_layer_recovery.py','recovery_coupled_probe.py','recovery_height_probe.py','seal_compressed_layer_recovery.py')]+[ROOT/'tests/test_compressed_layer_recovery.py']
evidence={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in sorted(OUT.rglob('*')) if p.is_file() and p.name not in ('AUTHORITY.json','HASHES.json')}
r.save('AUTHORITY.json',dict(task='QualifyV4CompressedLayerRecoveryAndProductionIntegration',classification=classification,input_head=head,source_sha256={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in sources},evidence_sha256=evidence,prior_authorities=prior,report_sha256=sha(OUT/'report.md'),production_modified=False,V3_modified=False,termination_implemented=False,scientific_identity=q.identity(data)))
r.save('HASHES.json',dict(report_sha256=sha(OUT/'report.md'),authority_sha256=sha(OUT/'AUTHORITY.json')))
print(classification,r.load(OUT/'HASHES.json'),flush=True)
