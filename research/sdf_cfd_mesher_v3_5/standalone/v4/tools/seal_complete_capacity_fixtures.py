"""Seal fixture-only qualification and verify prior authorities unchanged."""
import json,hashlib,subprocess,time,re
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/complete_capacity_fixtures'
def load(n):return json.loads((OUT/n).read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(n,v):(OUT/n).write_text(json.dumps(v,indent=2,sort_keys=True)+'\n')
start=load('START.json');rows=load('RESULTS.json');t=load('TIMINGS.json');u=load('REPLAY_TIMINGS.json')
verified={}
for folder in ('task4l','layer_capacity_forecast','layer_capacity_witness'):
    a=json.loads((ROOT/'results'/folder/'AUTHORITY.json').read_text())
    for group in ('source_sha256','evidence_sha256'):
        for name,h in a[group].items():assert sha(ROOT/name)==h,name
    verified[folder]=dict(authority=sha(ROOT/'results'/folder/'AUTHORITY.json'),report=sha(ROOT/'results'/folder/'report.md'))
status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines()
assert [x for x in status if 'sdf_cfd_mesher_v4' not in x]==[x for x in start['status'] if 'sdf_cfd_mesher_v4' not in x]
save('PRESERVATION.json',dict(prior=verified,unrelated_entries=len(start['status']),unrelated_status_unchanged=True,V3_modifications=0,production_changes=0))
assert all(r['foundation']['passed'] for r in rows)
assert all(r['passed_smoke_layers']==(0 if r['definition']['name']=='near_contact' else 2) for r in rows)
manifest=[]
for r in rows:
    f=r['foundation'];spec=r['definition'];m=dict(name=spec['name'],geometry_id=f['geometry_identity'],coordinate_hash=f['coordinate_hash'],topology_id=f['topology_hash'],topology_hash=f['topology_hash'],SDF_definition_id=f['sdf_identity'],analytic_reference_id=f['analytic_reference_identity'],file=spec['name']+'.npz',file_sha256=sha(OUT/(spec['name']+'.npz')),
      source_vertex_ids=dict(first=0,count=f['vertices']),source_triangle_ids=dict(first=0,count=f['triangles']),core_vertices=f['patch']['core_vertices'],core_edges=f['patch']['side_edges'],core_triangles=f['patch']['core_columns'],halo_vertices=f['halo_vertices'],halo_triangles=f['halo_triangles'],closure_only_triangles=f['closure_only_triangles'])
    if spec['kind']=='box':m['boundary_independence']='All core/metric-halo points have abs(y),abs(z)<=0.0012; lateral closure at 0.05. Maximum requested offset 0.020000000131704564 is below 0.0488 side clearance.'
    if spec['kind']=='cylinder':m['boundary_independence']='Core/metric halo has abs(z)<=0.0012; caps at abs(z)=0.05. Requested offset below 0.0488 cap clearance; radial bound unchanged.'
    manifest.append(m)
save('FIXTURE_MANIFEST.json',manifest)
smokes=[s for r in rows for s in r['smoke'] if s['passed']];pairs=sum(s['certificate']['required_pairs'] for s in smokes);exact=sum(s['certificate']['exact_calls'] for s in smokes)
textrows='\n'.join('| '+r['definition']['name']+' | '+str(r['foundation']['vertices'])+' | '+str(r['foundation']['triangles'])+' | '+str(len(r['foundation']['patch']['core_columns']))+' | '+str(r['passed_smoke_layers'])+' | '+str(r['capacity']['predicted_first_infeasible_layer'])+' |' for r in rows)
gate_rows=[]
for metric in ('volume','signed_volume','jacobian','determinant','aspect','weight','skewness','symmetric_volume_ratio'):
    vals=[s['gate']['extrema'][metric] for s in smokes];gate_rows.append('| '+metric+' | '+repr(min(v['min'] for v in vals))+' | '+repr(max(v['max'] for v in vals))+' |')
test=re.search(r'Ran (\d+) tests in ([0-9.]+)s',(OUT/'tests.txt').read_text());assert test and 'OK' in (OUT/'tests.txt').read_text()
save('TESTS.json',dict(command='python -B -m unittest test_complete_capacity_fixtures test_capacity_forecast test_capacity_witness test_dynamic_risk_atlas',count=int(test[1]),seconds=float(test[2]),passed=True))
report=f"""## Decision question

Can small deterministic capacity-test fronts satisfy ordinary V4 topology, ownership and bounded prism-validation assumptions while retaining interpretable capacity mechanisms?

### A. Authority and scope

Started at {start['head']}. Task-4L, prior capacity forecast and blocked witness source/evidence bindings all verify unchanged. The new authorities live separately under complete_capacity_fixtures. Original single-triangle probes remain sealed. No V3, production rolling builder, metric definition, certificate mathematics, schedule or hard gate changed. No termination or downstream meshing was implemented.

### B. Fixture architecture

All eleven surfaces are closed, consistently oriented triangle manifolds. Every edge, including closure-only edges, has exactly two actual owners. No ghost neighbors, substituted boundary weights, skipped core faces or fixture-only gate rules exist.

Box and cylindrical fixtures use bounded scientific cores. Their metric halo is exactly the set of actual neighboring columns required by existing Patch/require_closure. Complete closure geometry remains available to collision screening. Only core cells, incident core sides and interlayer caps are qualified. Sphere fixtures and the general multi-body fixture use their complete surfaces as cores. A whole closure mesh is not claimed to have passed core-only numeric qualification.

### C. Layer-0 foundation

Every fixture passes finite coordinates, triangle indices, nonzero area, unique triangles, opposite orientation on shared edges, two-owner incidence, SDF wall residual checks, and source identity checks. IDs are deterministic source ordinals bound to coordinate/topology/SDF hashes.

Initial nonintersection follows construction: injective rectangular box/wedge maps; noncrossing rings and cap fans for cylinders/frusta; radial subdivision of octahedral spherical sectors; and strictly disjoint bounding spheres between multi-body components. The tests check the geometric construction preconditions. This is not an unreported approximate intersection tolerance.

### D. Fixture inventory and smoke result

| Fixture | Vertices | Triangles | Core columns | Passing smoke layers | Analytic capacity first failure |
|---|---:|---:|---:|---:|---:|
{textrows}

Ten fixtures pass layers 1 and 2. The near-contact wall front passes foundation but cannot reach target layer 1. Its box inradius is half the first-layer height. Projection stops, no layer is admitted, and a separate exact opposing-strip continuation returns CertifiedForbiddenIntersection. Those strips are diagnostic impossible-target geometry, not an accepted candidate.

### E. Analytic references and boundary independence

The preserved binary64 schedule is loaded by load_schedule, including source hash verification. Targets are never regenerated from rounded h1/growth values. Limited box half-gaps correspond to first failures 10, 1 and 8; sphere radius to 13; cylinder radius to 16. Direct comparisons use exact rational interpretations of stored binary64 numbers.

For a box cavity phi is minus the exact signed distance to the box: inside, min(half_extent-abs(coordinate)). Maximum phi equals the smallest half extent. The measured opposing faces retain their exact gap. Core and metric-halo coordinates remain within 0.0012 laterally, while closure is at 0.05. Closure clearance 0.0488 exceeds the full requested 0.020000000131704564 offset, so lateral closure cannot control these core references.

For the finite cylinder, phi is minus the capped-cylinder signed distance. Scientific core/halo lies within abs(z)<=0.0012 and caps are at abs(z)=0.05. The same clearance inequality isolates radial collapse. Spherical cavity phi=R-norm(x); its complete surface has no triangulation boundary.

The wide box and wide sphere have capacity through 20 by direct geometry and pass two-layer hard smoke validation. They are not claimed to be fully hard-gate-qualified 20-layer prism stacks; that is the next study.

### F. Narrowing and general-SDF foundations

Symmetric and asymmetric channels are closed widening wedge cavities. Their half widths increase along the longitudinal coordinate. The scientific station has half-gap 0.003; the wider interior permits tangential escape from the station. Closure lies outside the dependency-complete measured core. These fixtures produce non-binding station warnings, not global capacity claims.

The curved passage is a closed widening frustum. Its radius increases along the axis, again making fixed-station radius a warning rather than an invariant capacity. It has complete side-wall and end-cap topology. No full steering qualification was attempted.

The general asymmetric passage is fluid outside three disjoint unequal spheres. Its authoritative SDF is the minimum of exact sphere distances, with deterministic gradient selection at ties. It is a non-analytic composite field at competing closest features. The component walls are smooth; no closed-form final capacity layer is supplied. Later offset merger/topology constraints may limit it. It is a general witness challenge, not asserted to have globally steerable capacity. Initial component separation is directly checked.

### G. Authoritative SDF access

All fixtures return signed distance, gradient, normalized normal, source roundoff bound and deterministic source identity using the existing predictor interface. Cavity distance is positive in fluid. Exterior multi-body distance is positive outside solids. Wedge distances use exact geometric face/edge/vertex closest-point definitions; outside distance is not approximated by maximum plane violation. Frustum distance uses closest points to its meridian side/cap segments. Numerical evaluation and projection have floating roundoff envelopes; these are not physical clearance thresholds or mesh gates.

Two new-generator defects were repaired before sealing: a duplicated closure row after remapping, and one-rounding-unit cap projection drift that produced a spurious tangential normal. REPAIRS.json preserves reproducers. Tests cover both. No prior source was repaired or overwritten.

### H. Ordinary hard validation

The smoke adapter calls existing evaluate_layer for actual neighbor geometry, then existing numeric_gates on dependency-complete core quantities, matching the existing detailed patch scope. Exact dyadic Jacobian/volume signs are checked for every measured prism. SDF projection uses the qualified predict_next_front. No candidate correction, gate adjustment, optimizer change or new physical degree of freedom is used.

Measured extrema across the twenty passing core smoke layers:

| Metric | Minimum | Maximum |
|---|---:|---:|
{chr(10).join(gate_rows)}

All reported gates pass. Topology and source ownership are qualified by the foundation and unchanged connectivity. Conditioning remains diagnostic; no sigma or G_bound gate was added.

### I. Exact continuous certification

The qualified production surface representation, persistent native layer index, conservative candidate enumeration and exact-safe prove hierarchy are reused. For each new smoke layer, every candidate touching a core side at any prefix layer is certified against the complete closure geometry. This includes new-vs-new and new-vs-old interactions. Core side validity is checked. The complement is certified by bilinear convex-hull AABB separation.

Passing layers cover {pairs} required local relations, with {exact} frozen exact fallbacks, zero forbidden results, zero unresolved results and no invalid core side surfaces. One additional exact call proves the intentional near-contact overlap. No full future certificate or full shell build runs. Pair populations, outcomes, classes, geometry hashes and exact proof identities replay exactly. Core scope is explicit and does not confer whole-closure numeric validity.

### J. Capacity smoke regression

Predictions remain 10, 1, 8, 13 and 16 for the five limited fixtures. Both wide controls are CapacityClearThroughRequestedLayers for geometric capacity. All four narrowing/general cases remain CapacityWarning with no claimed first capacity layer. None acquires termination authority. Historical SF2 and all prior results are untouched; no SF2 rebuild was necessary.

### K. Locality, runtime and storage

Largest complete fixture: 1282 vertices and 2560 triangles. Largest measured core: 1536 triangles. Two smoke layers only; no valid-prefix search. First qualification pass {t['total_seconds']:.6f} s, replay {u['total_seconds']:.6f} s. Peak RSS {max(t['peak_rss'],u['peak_rss'])} bytes. Per-fixture timings, scope and identity tables are persisted. Only new fixture fronts and two smoke states are stored; no accepted SF2 shell is copied.

### L. Determinism and tests

Independent construction/replay matches coordinates, topology, core/halo identities, gate extrema, exact Jacobian outcomes, certificate populations/outcomes and capacity statuses. Scientific hash {load('DETERMINISM.json')['scientific_hash']}. Runtime/RSS are excluded.

{test[1]} focused tests pass. Modules: complete fixture foundations, preserved capacity forecast, witness semantics and dynamic atlas. Tests reject open edges, duplicate triangles and orientation corruption; verify source normals, fixture repair cases, complete core dependencies, control smoke validity, intended near-contact failure and warning-only channel semantics. Prior tests remain unchanged.

### M. Classification and next task

V4CompleteCapacityFixturesQualified

The original ownership blocker is removed for these new authorities. Near-contact's zero-layer smoke prefix is intentional physical infeasibility, not an ownership exception. The qualification applies to layer-0 foundations and two-layer core smoke checks only.

Next rerun controlled valid-prefix/witness qualification on these frozen new fixtures, including bounded steering and general-SDF witness strength. Do not begin actual termination topology yet. No layer-count map, margin, topology transition, layer deletion or production commit behavior was implemented.
"""
(OUT/'report.md').write_text(report,encoding='utf-8')
sources=['tools/complete_capacity_fixtures.py','tools/qualify_complete_capacity_fixtures.py','tools/seal_complete_capacity_fixtures.py','tests/test_complete_capacity_fixtures.py']
save('AUTHORITY.json',dict(task='BuildV4CompleteCapacityQualificationFixtures',classification='V4CompleteCapacityFixturesQualified',starting_head=start['head'],report_sha256=sha(OUT/'report.md'),source_sha256={p:sha(ROOT/p) for p in sources},evidence_sha256={p.relative_to(ROOT).as_posix():sha(p) for p in OUT.iterdir() if p.is_file() and p.name not in ('AUTHORITY.json','SEAL.json')},production_builder_modified=False,V3_modified=False,termination_implemented=False,full_valid_prefix_qualified=False))
save('SEAL.json',dict(report_sha256=sha(OUT/'report.md'),authority_sha256=sha(OUT/'AUTHORITY.json'),task_elapsed_seconds=time.time()-start['start'],artifact_bytes=sum(p.stat().st_size for p in OUT.iterdir() if p.is_file())))
print(json.dumps(load('SEAL.json'),indent=2))
