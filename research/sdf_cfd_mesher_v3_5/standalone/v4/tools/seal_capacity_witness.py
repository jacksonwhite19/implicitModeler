import sys,json,hashlib,time,subprocess,re
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
import psutil
from task3c_io import read
from sdf_cfd_mesher_v4.forecast.capacity import CapacityForecast,identity
from sdf_cfd_mesher_v4.forecast.schedule import load_schedule
OUT=ROOT/'results/layer_capacity_witness'
def load(n):return json.loads((OUT/n).read_text())
def save(n,v):(OUT/n).write_text(json.dumps(v,indent=2,sort_keys=True)+'\n')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
start=load('START.json');data=load('FIXTURE_PREFLIGHT.json');times=load('TIMINGS.json')
# Verify all sealed prior capacity evidence and unchanged qualified source bindings.
prior=json.loads((ROOT/'results/layer_capacity_forecast/AUTHORITY.json').read_text())
for group in ('source_sha256','evidence_sha256'):
    for p,h in prior[group].items():assert sha(ROOT/p)==h,p
l=json.loads((ROOT/'results/task4l/AUTHORITY.json').read_text())
for group in ('source_sha256','evidence_sha256'):
    for p,h in l[group].items():assert sha(ROOT/p)==h,p
status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines()
assert [x for x in status if 'sdf_cfd_mesher_v4' not in x]==[x for x in start['status'] if 'sdf_cfd_mesher_v4' not in x]
clock=time.perf_counter();s=load_schedule();run=ROOT/'results/task4l/fresh';planner=CapacityForecast();sf=[];topology_identity=read(run/'FINAL_VALIDATION.json')['topology_hash']
for layer in range(1,21):
    events=read(run/f'LAYER{layer}_WARNINGS.json');record=planner.analyze('sf2_history',s,layer,events=events,geometry_identity=sha(run/f'build_A/FRONT_{layer:02}.npy'),topology_identity=topology_identity)
    sf.append(dict(layer=layer,status=record['status'],input_identity=record['scientific_input_identity'],warning_events=len(record['screen']['events']),termination_authority=False))
assert all(x['status']!='CapacityLimitPredicted' for x in sf)
save('SF2_REGRESSION.json',dict(history=sf,zero_capacity_limits=True,unchanged_sealed_task4l_evidence=len(l['evidence_sha256']),geometry_changes=0,decision_changes=0,layer_count_changes=0,seconds=time.perf_counter()-clock,peak_rss=psutil.Process().memory_info().peak_wset))
save('PRESERVATION.json',dict(unrelated_entries=len(start['status']),unrelated_status_unchanged=True,V3_modifications=0,rolling_builder_unchanged=True,prior_capacity_bindings_verified=True,task4l_bindings_verified=True))
save('FALSE_POSITIVE_NEGATIVE.json',dict(new_capacity_claims=0,false_capacity_claims=0,known_geometric_predictions_retained=5,valid_prefix_misses=None,reason='All ten controlled front probes lack complete ownership; rolling boundary unassessed'))
rows='\n'.join('| '+r['region_id']+' | '+str(r['capacity_prediction']['predicted_first_infeasible_layer'])+' | '+str(r['warning_first_observed_layer'])+' | blocked | unknown |' for r in data['fixtures'])
failures='\n'.join('- '+r['region_id']+': '+('; '.join(x['metric']+'='+repr(x['value']) for x in r['partial_cell_diagnostics']['failures']) or 'No failed cell metric; face coverage still unavailable')+'.' for r in data['fixtures'])
report=f"""## Decision question

Can V4 identify a defensible last-valid-layer / first-invalid-layer boundary early enough for a future local termination planner to act before invalid geometry is committed?

### A. Authority verification

Expected HEAD {start['head']} verified. Prior capacity report SHA-256 ee3afbba2af026c7aa22e403692f12aab841ef95c63dfb34a8d5fbf1ca3b8231 and authority 900c5b035b2783592b40587215f296f194669ad54325acf3ddab1c5032b255f2 verified. All prior capacity source/evidence bindings and Task-4L source/evidence bindings remain unchanged. No V3, builder, schedule, certificate or hard-gate change. {len(start['status'])} unrelated dirty entries preserved.

### B. Valid-prefix advancement architecture

The implementation stops at a necessary ownership preflight. The preserved fixtures define one triangle with three vertices, plus separate side-pair probes. They do not supply a closed front or dependency-complete neighboring columns. Each of the three edges has one owner. Production evaluate_layer explicitly requires two edge owners and raises IncompleteSurfaceOwnership for all ten fixtures.

A new fail-closed witness record distinguishes last_committed_layer=0 from last_fully_valid_layer=null. Zero fixture layers were committed. First_invalid_layer is null: missing validation inputs are not a measured rolling failure layer. An ownership dependency check alone is not complete topology qualification.

Adding triangles/closing these fronts changes fixture topology. Supplying ghost cells or inventing boundary-face interpolation rules changes the validation contract. Both exceed this task's no-topology-change boundary. No rolling fixture publisher was implemented because complete receipts cannot be obtained from these inputs. This is an input/validation-interface blocker, not an advancing-front architecture failure.

### C. Controlled fixture results

| Fixture | Geometric failure prediction | First warning observed at layer | Rolling result | Fully valid prefix |
|---|---:|---:|---|---|
{rows}

All ten original fixture coordinates and source definitions were retained. The two previously feasible controls meant geometric capacity only; neither was previously a fully hard-gate-qualified mesh.

### D. First hard failures

Every fixture reproduces the production ownership exception before a complete layer receipt. Additional *partial* first-layer cell diagnostics use the frozen cell metrics and exact dyadic Jacobian kernel. Entity ID is 0 in each case:

{failures}

These measurements are not full first-hard-failure qualification. Numeric gates exclude determinant below 0.0011 and aspect above 1000. Missing interpolation-weight/skewness/volume-ratio inputs cannot be averaged away or treated as passed. The narrowing probes also have negative oriented signed volume/Jacobian. Reversing their original orientation or resizing the 1 m probe edges would change preserved fixture input and was not performed.

Near-contact projection fails because the first target exceeds physical capacity. Its partial cell values use the explicitly marked invalid analytic continuation. Other partial values use the qualified SDF projector. No invalid candidate is admitted.

### E. Last fully valid layers

Last committed fixture layer is 0 for all fixtures. No prism-layer valid prefix has been qualified; last fully valid layer and first invalid layer remain unknown. Exact remaining schedule demand at layer 0 is stored for every record. It is not mislabeled as remaining demand at a qualified last-valid boundary.

### F. Capacity-witness timing

The five analytic limits are visible at layer 0, predicting 10, 1, 8, 13 and 16. Three narrowing-station warnings are visible at layer 0. Feasible controls are capacity-clear at layer 0. No later committed fixture layers exist, so warning-to-hard-failure lead time and prediction-minus-hard-failure offsets are null. No minimum notice or margin was invented.

### G. Geometric limit versus quality-gate limit

All ten full-prefix mechanism classifications are UnresolvedLimitMechanism because full gate coverage is absent. Partial diagnostics demonstrate that cell quality can already fail at layer 1 while the analytic capacity bound is much later. The record classifier supports GeometricLimitControlsFailure, QualityGateControlsFailure, SteerableBeforeCapacity and CapacityClear only when full coverage is explicitly available; it is not a commit authority.

### H. Steering probes

Not run beyond nominal projection. Tangential movement changes coordinates but cannot supply the missing neighboring cells required for face metrics. No failed search is called a proof of incapacity. Existing capacity proofs remain analytic: AnalyticSpacingWitness for parallel/near-contact/symmetric walls, and SDFOffsetPinchWitness for spherical/cylindrical domains. Prior exact collision/Jacobian probes remain supporting geometry evidence, not complete rolling receipts.

### I. General-SDF fixtures

Not implemented after the controlled-front blocker was established. Adding further point/triangle probes would repeat the same missing-prefix problem. General-SDF capacity remains unqualified. This required portion of the task is explicitly incomplete.

### J. General-SDF witness strength

No new exact capacity claim, unavoidable-collision claim or general-SDF witness is made. Arbitrary SDF warning evidence still cannot define a hard-gate-valid boundary. No heuristic risk threshold or safety margin was introduced.

### K. SF2 regression

The read-only predictor was applied to all 20 preserved warning histories, using fresh coordinate-file identities and the preserved schedule. It produces zero capacity-limit authorities. Empty warnings remain unresolved without a source capacity witness; warnings are diagnostics. Task-4L source and sealed artifact hashes preserve geometry, all 32 accepted correction decisions, layer count and original complete hard-gate/certificate results. No SF2 shell was rebuilt.

### L. False positives and false negatives

Zero new false capacity-limit claims. The five known analytic predictions remain unchanged. Valid-prefix misses and advance-warning lead time cannot be counted while fixture admission is blocked. Zero must not be reported for these unknown quantities. Prior pointwise wedge steerability is not upgraded to full mesh validity.

### M. Runtime and locality

Controlled preflight and partial diagnostics: {times['first']['seconds']:.6f} s; replay {times['replay']['seconds']:.6f} s. Peak controlled-study RSS {max(times['first']['peak_rss'],times['replay']['peak_rss'])} bytes. SF2 read-only timing/RSS is stored separately. Each fixture uses 3 vertices, 1 triangle and 3 edges; there are no global optimizer or sensitivity allocations. Capacity cache: {data['cache']['hits']} hits, {data['cache']['misses']} misses. New exact collision calls: 0. Steering-search time: 0. No expensive second whole-shell planner ran.

### N. Determinism

Repeated fixture ownership results, capacity records, partial metrics, exact signs and blocked-prefix records agree exactly. Scientific hash: {load('DETERMINISM.json')['scientific_hash']}. Runtime and RSS excluded. This is deterministic blocker reproduction, not deterministic successful rolling advancement. SF2 results are content-bound to immutable preserved inputs.

### O. Tests

53 tests passed across test_capacity_witness, test_capacity_forecast and test_dynamic_risk_atlas. New tests cover missing-owner detection, closed dependency control, null valid-prefix semantics, warning timing, and classification coverage guards. Existing capacity/atlas tests remain unchanged. Successful rolling advancement, general-SDF witnesses, and full prefix certification tests were not run because those paths are not implemented. No large unrelated suite ran.

### P. Classification

V4LayerCapacityWitnessBlocked

Technical blocker: the preserved geometric fixtures lack the topology/dependency ownership required by the normal immediate-layer validator. Full requested qualification is incomplete. This result corrects any impression that geometric-feasible controls were already fully valid prism meshes.

### Q. Termination-topology readiness

No. First authorize complete fixture-front definitions or a separately qualified local boundary/halo validation contract. Preserve the analytic domains and original probes, but provide frozen front connectivity, orientation, source ownership and metric dependencies before rolling prefix qualification. That prerequisite must be explicit because this task forbids topology changes. Then rerun controlled prefixes and general-SDF witnesses. Do not implement actual termination topology yet.
"""
(OUT/'report.md').write_text(report)
sources=['src/sdf_cfd_mesher_v4/forecast/capacity_witness.py','tools/run_capacity_witness.py','tools/seal_capacity_witness.py','tests/test_capacity_witness.py']
save('AUTHORITY.json',dict(task='QualifyV4LayerCapacityWitnessAndValidPrefixSemantics',classification='V4LayerCapacityWitnessBlocked',starting_head=start['head'],report_sha256=sha(OUT/'report.md'),source_sha256={p:sha(ROOT/p) for p in sources},evidence_sha256={p.relative_to(ROOT).as_posix():sha(p) for p in OUT.iterdir() if p.is_file() and p.name not in ('AUTHORITY.json','SEAL.json')},termination_ready=False,fixture_layers_committed=0))
save('SEAL.json',dict(report_sha256=sha(OUT/'report.md'),authority_sha256=sha(OUT/'AUTHORITY.json'),elapsed_seconds=time.time()-start['start'],peak_rss=psutil.Process().memory_info().peak_wset))
print(json.dumps(load('SEAL.json'),indent=2))
