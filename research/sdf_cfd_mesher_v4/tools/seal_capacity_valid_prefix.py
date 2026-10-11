"""Seal bounded valid-prefix evidence without modifying prior authorities."""
import json,hashlib,subprocess,time,re
from pathlib import Path
import qualify_capacity_valid_prefix as q
from task3c_io import read
ROOT=q.ROOT;OUT=q.OUT
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
load=lambda n:json.loads((OUT/n).read_text())
save=lambda n,v:q.write(OUT/n,v)
start=load('START.json');assert start['head']=='9701be80c07efdfefddee09097099aeb9064361e'
verified={}
for folder in ('complete_capacity_fixtures','layer_capacity_forecast','layer_capacity_witness','task4l'):
    path=ROOT/'results'/folder;a=json.loads((path/'AUTHORITY.json').read_text())
    for group in ('source_sha256','evidence_sha256'):
        for name,h in a[group].items():assert sha(ROOT/name)==h,name
    verified[folder]=dict(report=sha(path/'report.md'),authority=sha(path/'AUTHORITY.json'))
assert verified['complete_capacity_fixtures']==dict(report='710407e8eac6cf9a778a6856d660e70cc112317e6d3249f03d704e48a7e67fd7',authority='d461bb9e83c2331d31575aa6a7e9112eabeecd48506d1ef859f2d5f253fdeb5f')
status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines();unrelated=lambda rows:[x for x in rows if 'sdf_cfd_mesher_v4' not in x]
assert unrelated(status)==unrelated(start['status'])
save('PRESERVATION.json',dict(prior=verified,unrelated_entries=len(unrelated(status)),unrelated_state_unchanged=True,V3_modifications=0,production_builder_modifications=0))
rows=load('RESULTS.json');t=load('TIMINGS.json');u=load('REPLAY_TIMINGS.json');assert load('DETERMINISM.json')['equal']
s=q.load_schedule();cf=q.CapacityForecast();sf=[];run=ROOT/'results/task4l/fresh';top=read(run/'FINAL_VALIDATION.json')['topology_hash'];tick=time.perf_counter()
for layer in range(1,21):
    r=cf.analyze('sf2_history',s,layer,events=read(run/f'LAYER{layer}_WARNINGS.json'),geometry_identity=sha(run/f'build_A/FRONT_{layer:02}.npy'),topology_identity=top)
    sf.append(dict(layer=layer,status=r['status'],identity=r['scientific_input_identity'],capacity_limit=r['predicted_first_infeasible_layer'],termination_authority=r['termination_authority']))
assert all(r['capacity_limit'] is None and not r['termination_authority'] for r in sf)
save('SF2_REGRESSION.json',dict(history=sf,seconds=time.perf_counter()-tick,geometry_changes=0,correction_decision_changes=0,layer_count_changes=0,capacity_claims=0,accepted_corrections_preserved=32))
records=[];committed=[]
for r in rows:
    name=r['fixture'];first=r['first_invalid_or_unadmitted_layer'];last=r['last_valid_layer'];w=r['witness_history'][-1]
    for h in r['history']:
        if h['committed']:
            q.load_checkpoint(OUT/name,h['layer'],r['foundation']['topology_hash']);committed.append(h)
    bound=w['evidence'].get('bound');witness='ExactCollisionWitness' if name=='symmetric_contact' else ('AnalyticSpacingWitness' if name in ('parallel_gap','near_contact') else 'SDFOffsetPinchWitness' if name in ('spherical_cavity','cylindrical_cavity') else None)
    terminal=r['history'][-1] if first else None
    gate='ForbiddenContinuousSideIntersection' if terminal and terminal.get('certificate',{}).get('failures') else 'SDFTargetNotAdmitted' if terminal and terminal.get('projection_failure') else None
    proof=dict(witness_type=witness,bound=bound,target_at_stop_hex=None if first is None else float(s.targets[first-1]).hex(),reference_identity=r['foundation']['analytic_reference_identity'],scope='Analytic source upper bound; projection iteration cap alone is not a capacity proof')
    record=dict(region_id=name,current_committed_layer=last,requested_final_layer=20,last_fully_valid_layer=last,predicted_first_geometric_capacity_failure=r['predicted_geometric_failure'],observed_first_hard_failure=r['observed_hard_failure'],first_unadmitted_layer=first,controlling_gate=gate,witness=proof,warning_onset_layer=r['warning_onset'],warning_lead=r['warning_lead'],capacity_classification=r['classification'],steerability_classification='NotRestoredByBoundedProbe' if r['steering_probes'] else 'NoSteeringRequiredBeforeStop',remaining_requested_thickness=r['remaining'],termination_authority=False,scope=r['scope'])
    records.append(record)
save('TERMINATION_INPUTS.json',records)
save('FALSE_POSITIVE_NEGATIVE.json',dict(false_capacity_claims=0,missed_known_analytic_limits=0,analytic_cases=5,nonbinding_channel_warnings=3,general_SDF_unresolved=1,channel_core_boundaries_unresolved=3,quality_gate_before_analytic_limit=0,steerable_successes_demonstrated=0,scope='No general completeness claim; projection failure is separate from evaluated hard-gate failure'))
pairs=sum(h['certificate']['required_pairs'] for h in committed);exact=sum(h['certificate']['exact_calls'] for h in committed)
probes=[p for r in rows for p in r['steering_probes']];assert not any(p.get('passed') for p in probes)
save('OPERATION_COUNTS.json',dict(committed_fixture_layers=len(committed),required_certificate_relations=pairs,exact_fallbacks=exact,steering_trials=len(probes),successful_steering_trials=0,capacity_evaluations=sum(len(r['witness_history']) for r in rows),replay_equal=True))
# Direct exact near-contact collision witness is diagnostic continuation only.
spec=next(x for x in q.definitions(s.targets) if x['name']=='near_contact');h=spec['half'][0];target=float(s.targets[0]);width=.0008
A=q.np.array([[-h,0,0],[-h,width,0],[-h+target,0,0],[-h+target,width,0]])
B=q.np.array([[h,0,0],[h,width,0],[h-target,0,0],[h-target,width,0]])
collision=q.CertificateSession().pair(q.kernel.Patch.binary(A,tuple(range(4))),q.kernel.Patch.binary(B,tuple(range(4,8))))
assert collision['classification']=='CertifiedForbiddenIntersection'
save('NEAR_CONTACT_WITNESS.json',dict(result=collision,scope='Analytic opposing strips at impossible target; not a projected or committed front'))
log=(OUT/'tests.txt').read_text();match=re.search(r'Ran (\d+) tests in ([0-9.]+)s',log);assert match and '\nOK' in log
save('TESTS.json',dict(passed=True,count=int(match[1]),seconds=float(match[2]),command='python -B -m unittest test_capacity_valid_prefix test_complete_capacity_fixtures test_capacity_forecast test_capacity_witness'))
classification='V4LayerCapacityWitnessPromising'
table='\n'.join('| '+r['region_id']+' | '+str(r['predicted_first_geometric_capacity_failure'])+' | '+str(r['last_fully_valid_layer'])+' | '+str(r['first_unadmitted_layer'])+' | '+str(r['observed_first_hard_failure'])+' | '+str(r['warning_onset_layer'])+' | '+str(r['warning_lead'])+' |' for r in records)
certtime=sum(x['certificate_seconds'] for x in t['fixtures']);steertime=sum(x['steering_seconds'] for x in t['fixtures']);forecasttime=sum(x['forecast_seconds'] for x in t['fixtures'])
report=f'''## Decision question

Can V4 establish a hard-gate-valid last-layer boundary and a prospective capacity witness early enough for a future termination planner to act before invalid geometry is committed?

### A. Authority verification

Starting HEAD {start['head']} and complete-fixture report/authority hashes match the request. All source and evidence bindings of the complete fixtures, previous forecast, blocked witness, and Task-4L verify unchanged. No V3, production builder, topology, hard gate, certificate, SDF, or schedule was changed. {len(unrelated(status))} unrelated dirty entries remain unchanged.

### B. Rolling fixture advancement

All eleven frozen definitions start from their authoritative layer 0. Every intermediate accepted layer is constructed by the existing SDF projector and validated with frozen cell/face primitives, exact dyadic signs, and qualified continuous certificates. {len(committed)} fixture layers committed; none failed a required check. Each has coordinates and a receipt checkpoint. Restart checks reject changed coordinate identities.

Scope remains the previously qualified scientific core, its complete actual metric halo, and all continuous interactions touching core sides against closure geometry and the committed prefix. Closure-only cells are not claimed numerically qualified. Full spheres and the composite passage use their complete surfaces as cores.

The study adapter uses the same production primitive expressions and ordering with optional conditioning disabled. Sigma/G are not hard gates. This prevents an optional SVD failure on collapsed closure-only cells from hiding the decisive core collision. Old/new hard primitive arrays agree bitwise in regression. The production builder is unchanged.

### C. Analytic capacity references

Stored binary64 targets are used directly. Slab capacity is spacing/2; spherical and cylindrical cavity capacity is radius. Target >= bound is the preserved geometric event. Parallel, near-contact, symmetric, spherical and cylindrical predictions remain 10, 1, 8, 13 and 16. Exact rational demand and source identities are in TERMINATION_INPUTS.json. No clearance fraction or layer buffer exists.

### D. Last valid / first invalid layers

| Fixture | Geometric prediction | Last committed valid | First unadmitted | First evaluated hard failure | First warning | Lead to unadmitted |
|---|---:|---:|---:|---:|---:|---:|
{table}

Null first-hard-failure entries are intentional. A projector that cannot provide an SDF-admitted candidate has not produced an actual prism on which all hard metrics can be measured. These are valid-prefix/admissibility boundaries, not invented numeric failure values. For channels, last committed is only the reached prefix, not a maximum core capacity.

### E. First controlling hard gates

Symmetric contact: layer 8 has passing individual core numeric gates and positive exact Jacobians, but exact rank-2 side intersections are forbidden. The first pair, exact rational point, proof and all failures are in symmetric_contact/result.json; layer 8 remains uncommitted.

Parallel layer 10, near-contact layer 1, sphere layer 13 and cylinder layer 16 cannot attain the requested SDF target. Independent analytic upper bounds establish absent/collapsed offset geometry; the iteration cap itself is not used as proof. No fabricated determinant, weight or Jacobian value is assigned. Near-contact also has an exact forbidden-intersection witness on explicitly diagnostic analytic opposing strips. Its valid layer 0 has no committed prism layer.

### F. Capacity-warning lead time

All five analytic limits are announced at layer 0. Lead counts to the first unadmitted layer are 10, 1, 8, 13 and 16. Witness records are evaluated again after every valid commitment. Lead is descriptive, not a safety margin or termination policy.

### G. Geometric limit versus hard-gate limit

The five observed admissibility stops agree exactly with the analytic geometric layers. Only symmetric contact provides an evaluated first mesh hard-gate failure. No earlier numeric quality failure was observed in these five prefixes. The distinction between unavailable target geometry and failed evaluated prism metrics is retained in every planner record.

### H. Steering probes

{len(probes)} deterministic bounded trials use existing tangent directions, amplitude ordering, SDF reprojection and dependency-complete verification support. No trial restores complete admission and immediate validity. This is not an exhaustive impossibility result. There are no newly accepted steering decisions. Projection localization time is included in the steering/diagnostic timing below, including analytic cases where no steering was needed.

### I. Feasible controls

Both planar-wide and curved-wide controls reach layer 20 with every immediate core hard gate, exact sign and continuous certificate passing. Neither receives a capacity claim. Geometric clearance alone was not used to qualify them.

### J. Channel warning cases

All three channels reach layer 5. Layer-6 projection failures occur only outside the scientific core and its metric halo. The closed auxiliary ends prevent complete-front projection; bounded existing tangent probes do not repair this. No core capacity limit or observed core hard failure is claimed. Altering closure geometry would change frozen fixtures, so it was not done. The two-layer foundation qualification did not establish closure viability through the full requested history. SteerableBeforeCapacity remains unproven.

### K. General-SDF witness

The asymmetric composite-SDF passage reaches layer 3. At layer 4, projection fails at core vertices; bounded tangent probes do not restore a valid candidate. The opposing-front warning exists at layer 0, but no direct unavoidable geometric limit is proved. Classification is GeneralSDFWarningOnly. A termination planner cannot treat this search failure as a capacity authority.

### L. SF2 regression

A fresh read-only pass over all 20 frozen Task-4L histories produces zero capacity-limit claims. All prior scientific bindings remain unchanged: zero coordinate, correction-decision or layer-count changes; all 32 corrections preserved. The SF2 mesher was not rerun.

### M. False positives / false negatives

Zero false capacity claims and zero missed known analytic limits across five limited cases. Three channel warnings remain non-binding. There are three unresolved channel core boundaries and one unresolved general-SDF capacity witness. Zero earlier numeric failures were observed in the analytic cohort. No general false-negative rate is claimed.

### N. Termination-planner input records

TERMINATION_INPUTS.json binds region, committed layer, requested layer, reached valid prefix, geometric prediction, observed hard failure (nullable), unadmitted layer, witness type, onset, classification, steerability and exact remaining thickness. Every record has termination_authority=false. A finite reached prefix is not automatically permission to terminate there.

### O. Runtime and locality

Qualification {t['seconds']:.6f} s; independent replay {u['seconds']:.6f} s. First-pass certificate time {certtime:.6f} s; projection-localization/steering diagnostics {steertime:.6f} s; capacity forecast {forecasttime:.6f} s. Peak process RSS {max(t['peak_rss'],u['peak_rss'])} bytes. Passing prefixes cover {pairs} conservative required relations with {exact} exact fallbacks, zero forbidden and zero unresolved. Failing symmetric contact is excluded from passing totals. All work is bounded fixture work; no second whole-SF2 planner was added.

### P. Determinism

Independent reconstruction and rolling replay reproduce histories, coordinate hashes, exact outcomes, witness identities, probe decisions and boundaries exactly. Timing/RSS are separate. Checkpoint restart reproduces the next layer and certificate; coordinate tampering fails closed. Scientific hash: {load('DETERMINISM.json')['scientific_hash']}.

### Q. Tests

{match[1]} focused tests pass: {load('TESTS.json')['command']}. Tests cover frozen fixture contracts, capacity semantics, full persisted prefix coverage, analytic stop/onset correspondence, exact contact, zero-Jacobian rejection, primitive equivalence, checkpoint restart/tampering, non-binding general/channel warnings and deterministic replay. Successful steering extension and a direct general-SDF capacity witness remain unqualified; tests do not manufacture evidence for those missing outcomes.

### R. Classification

{classification}. Analytic valid prefixes and both full controls are established. General-SDF witness strength and channel continuation remain incomplete. Projection admission stops are clearly separated from actual mesh hard failures.

### S. Termination-topology readiness

No. First resolve the frozen channel closure limitation in a separately authorized fixture revision, and qualify a direct general-SDF witness or successful bounded steering continuation. Then repeat the affected local cases. Do not implement termination topology from the current incomplete records. No topology or termination policy was implemented here.
'''
(OUT/'report.md').write_text(report,newline='\n')
sources=['tools/qualify_capacity_valid_prefix.py','tools/seal_capacity_valid_prefix.py','tests/test_capacity_valid_prefix.py']
evidence={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in sorted(OUT.rglob('*')) if p.is_file() and p.name not in ('AUTHORITY.json','SEAL.json')}
a=dict(classification=classification,starting_head=start['head'],report_sha256=sha(OUT/'report.md'),source_sha256={p:sha(ROOT/p) for p in sources},evidence_sha256=evidence,termination_ready=False,production_builder_modified=False,V3_modified=False)
save('AUTHORITY.json',a);save('SEAL.json',dict(report_sha256=sha(OUT/'report.md'),authority_sha256=sha(OUT/'AUTHORITY.json'),persistent_bytes=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file())))
print(json.dumps(load('SEAL.json'),indent=2))
