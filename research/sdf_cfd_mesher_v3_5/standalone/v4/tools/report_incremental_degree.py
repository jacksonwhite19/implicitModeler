"""Report completed evidence only; never runs a mesh or degree audit."""
import json,hashlib,subprocess,datetime
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/incremental_degree_filling_validation';REPO=ROOT.parents[1]
def load(p):return json.loads(p.read_text())
def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):p.write_text(json.dumps(v,sort_keys=True,indent=2)+'\n',encoding='utf-8')

def main():
    cases={n:dict(layer=load(OUT/n/'LAYER.json'),comparison=load(OUT/n/'GLOBAL_COMPARISON.json'),seal=load(OUT/n/'SEAL.json')) for n in ('capsule_10_11','capsule_11_12','bent_5_6')}
    for z in cases.values():assert z['comparison']['passed'] and z['comparison']['replay_exact'] and z['seal']['passed']
    first=cases['capsule_10_11'];second=cases['capsule_11_12'];bent=cases['bent_5_6'];a=first['layer'];d=a['degree_metrics'];b=second['layer'];bd=b['degree_metrics']
    replay=load(OUT/'FINAL_CODE_DEGREE_REPLAY.json');assert replay['passed']
    start=load(OUT/'START.json');profile=load(OUT/'DEPENDENCY_PROFILE.json')
    for n,h in start['sealed'].items():assert sha(OUT.parent/'production_fast_incremental_validation'/n)==h
    current=subprocess.check_output(['git','status','--porcelain=v1','-z'],cwd=REPO).decode()
    def outside(s):return sorted(r for r in s.split('\0') if r and 'research/sdf_cfd_mesher_v4/' not in r)
    assert outside(current)==outside(start['status'])
    save(OUT/'GIT_PRESERVATION.json',dict(start_head=start['head'],unrelated_status_entries=len(outside(current)),unrelated_status_exact=True,
        prior_seals_unchanged=True,v3_task_edits=0,v3_full_byte_inventory='Not measured; pre-existing untracked V3 remains unchanged by task actions'))
    for name,z in cases.items():
        delta=z['layer']['degree_metrics']['delta'];old={tuple(sorted(f)):f for f in delta['retired']};new={tuple(sorted(f)):f for f in delta['added']}
        save(OUT/name/'BOUNDARY_DELTA_AUTHORITY.json',dict(delta=delta,identity=z['layer']['degree_metrics']['delta_identity'],
            changed_oriented_faces=[dict(retired=old[k],added=new[k]) for k in sorted(old.keys()&new.keys())],
            retired_count=len(old),added_count=len(new),unchanged_count=delta['unchanged'],
            source='Topology boundary set difference, independently equal to oriented sum of added cell boundaries'))
    runtime='\n'.join(f'| {k} | {v:.6f} | {bd["times"][k]:.6f} |' for k,v in d['times'].items())
    counts='\n'.join(f'| {k} | {v} | {bd["counts"][k]} |' for k,v in d['counts'].items() if not isinstance(v,dict))
    classification='V4IncrementalDegreeFillingQualified'
    text=f'''## Decision question

Can V4 replace the ordinary-layer full-shell oriented boundary-degree audit with an exact dependency-local update while preserving witness-by-witness equivalence with the existing global filling predicate?

### A. Authority verification

Starting HEAD {start['head']} is the requested ProductionFast qualification commit. Prior report and AUTHORITY hashes match the requested values and remain unchanged. Capsule input is the preserved verified Patricia layer 10; reference is the canonical-block sealed layer 11. The final saved layer-12 geometry is comparison evidence, not a new geometry authority. All new artifacts are under this task's result directory. No broader cohort, full capsule shell, or layers beyond 12 were run.

### B. Current global degree predicate

Positive exact cell volume/Jacobian, degree one at every exact cell-interior witness, exact cell/boundary volume equality, complete continuous face certification and oriented manifold incidence remain jointly required. None is inferred from the degree test alone. Final seal runs the original complete global degree audit.

### C. Degree implementation trace

tools/cavity_volume_audit.py: audit -> degree -> tools/projected_boundary_index.py: ProjectedBoundaryIndex.containing -> certificate/aabb_index.py: AABBIndex. The witness is the exact rational mean of distinct cell vertices. Binary64 coordinates become exact Fractions via the existing rational helper. Directions are (1,k,k*k+1), k=1..8. Exact projected determinants/barycentric coordinates and crossing x determine signed forward crossings. Parallel projected triangles, on-witness crossings and forward edge/vertex hits cause retries. Unresolved witnesses or degree other than one fail. An AST regression proves that removing the optional counters leaves the original qualified degree function unchanged.

### D. Boundary delta

For both capsule transitions: 8192 oriented top faces retire, 8192 new oriented tops enter, and 8192 wall faces remain unchanged. No boundary side face or orientation-only change occurs. These are measured topology-set differences, not layer-number assumptions. The canonical record includes oriented face lists, geometry and topology hashes, cell/column/layer ancestry and exact closed-cell chain equality. Duplicate boundary faces and incomplete cell support reject the optimized proof.

### E. Exact incremental formulation

For a common admissible ray, signed crossing sums are additive: deg(Bnew,w)=deg(Bold,w)+deg(added,w)-deg(retired,w). Direct projected delta-face search was measured first: all {profile['historical']} historical witnesses selected, {profile['ray_projected_pairs']} candidate pairs, {profile['ray_query_seconds']:.6f} s for the query. This would not provide useful locality.

The implemented alternative proves Bnew-Bold equals the oriented sum of the newly added prism-cell boundaries. Each closed cell chain is supported inside its vertex box. Outside that box its degree is exactly zero: the chain contracts inside the box without crossing the witness. Shared sides cancel with opposite orientations. Thus witnesses outside every added-cell box have zero delta degree. Witnesses inside conservative boxes are fully reevaluated against the complete new boundary. This is an exact zero-contribution proof, not approximate degree acceptance. FORMULATION.md gives the full argument and required direction guard.

### F. Direction and degeneracy semantics

Only old first-direction (1,1,2) receipts can be reused. Added projected-parallel faces and possible edge/vertex collinearity are conservatively screened. Outward-rounded interval cross products may exclude a hit only when zero is outside the interval. Possible degeneracy, any previous retry direction, or an affected support witness invokes the original global predicate for that witness. Retired faces cannot introduce a new degeneracy. A point on an added face is already in an added-cell box. Consequently reused rows preserve both degree and the exact first-valid ray. No d1 receipt is combined with d2 crossing counts.

### G. Degree receipts

V4ClosedChainDegreeReceipt/1 binds implementation hashes, prior root, canonical boundary delta, exact rational witness identities, degree/ray rows, affected selection and the closed-chain/first-ray reuse rule. Retry semantics are those of the bound first-valid-direction implementation. A missing prior receipt causes a one-time complete bootstrap audit. Roots are deterministic for the same version and history; no insertion-history-independent root claim is made. The root is separate from, and does not reinterpret, the preserved final mesh identity. The first benchmark predates added duplicate/support guards and a memory-field label correction; final-code degree-only replay on the same saved geometry passes with root {replay['root']}. That replay did not advance another capsule layer.

### H. Affected-witness indexing

Point cKDTrees query infinity-norm balls containing outward-rounded 3D cell boxes and 2D projected edge/face boxes, followed by inclusive box filtering. There is no dense witness-by-face matrix. Index construction still visits the historical point population; the expensive exact degree predicate is dependency-local. Bootstrap, final seal, and small index/metadata work remain global in size.

### I. Screening completeness

The initial support study selected {profile['cell_box_selected']} historical witnesses in {profile['cell_box_pairs']} cell-box/witness pairs. Every exact box hit is retained by conservative rounding. Projected edge screening uses outward rounding at every interval arithmetic operation. False positives are permitted. Tests include exact rational edge points, dense exact-box comparisons, an added edge that changes the valid ray, duplicate rejection and full global equality. Capsule production screening selected {d['counts']['selected']} historical witnesses; {d['counts']['reused']} were proved unchanged.

### J. New witness evaluation

All 8192 new witnesses run the original complete-boundary degree predicate. No prior degree is assigned to a new cell. On layer 11 this requires {d['counts']['new_full_boundary']['candidate_pairs']} exact candidate contributions and {d['times']['new_witness_full_degree']:.6f} s.

### K. Historical witness reuse/update

Layer 11 has 81920 historical witnesses. {d['counts']['selected']} are reevaluated against the full new boundary; {d['counts']['reused']} retain direction-compatible zero-delta authority. The reevaluated set requires {d['counts']['historical_full_boundary']['candidate_pairs']} exact contributions. Direct changed-face subtraction evaluations: 0; the implementation uses the closed-chain zero-support proof plus selected complete-boundary evaluation instead. The {d['counts']['projected_edge_witness_pairs']} projected edge/witness pairs are screening operations, not exact crossing contributions. The original global comparison inspects {first['comparison']['stats']['candidate_pairs']} candidate contributions.

### L. Exact volume aggregate

The previously qualified exact Fraction cell-volume aggregate is unchanged. All three production transitions pass exact cell/boundary volume equality. New exact Jacobian checks and old cell receipt reuse remain on the same qualified path. Focused aggregate regressions pass.

### M. Boundary-volume aggregate

The existing boundary-volume update uses the same old/new oriented boundary membership. Degree delta construction checks its chain identity from those faces. Exact boundary contributions are retained/retired/added by the existing aggregate; its implementation is unchanged. Geometry, incidence and full quality comparisons independently pass at final seal.

### N. Capsule 10→11

Baseline total 70.227009 s; optimized {a['seconds']:.6f} s. Baseline global degree 35.188565 s; optimized ordinary degree stage {a['timings']['incremental_degree']:.6f} s. Bootstrap {load(OUT/'capsule_10_11/BOOTSTRAP.json')['seconds']:.6f} s is separate and includes establishing the old global degree receipts. Result: 90112 cells; 10127812 certified relations; 825248 new relations; no remesh; exact wall and first-layer preservation.

### O. Global witness-by-witness comparison

All 90112 degree values AND ray vectors equal the unchanged complete reference exactly. Global reference runtime {first['comparison']['seconds']:.6f} s. Unresolved 0. Repeated degree evaluation reproduces the root, affected selection, contribution counters and results exactly. Final-code saved-geometry replay also passes and is invariant to reversed addition enumeration.

### P. Final seal

The original complete degree audit runs after the ordinary commit as an independent qualification reference, before publication of SEAL.json. Complete hard quality, incidence, coordinates, connectivity, layer state, atlas/forecast, relation set and proof outcomes match the preserved layer-11 authority. Missing/duplicate/forbidden/unresolved: 0/0/0/0. Scientific identity {first['seal']['scientific']}. The generic seal and seal_new entry points now explicitly run the complete degree predicate and reject stale degree receipts. Benchmarks invoked that same complete reference immediately before their independent seal; the final wrapper integration is covered by the focused stale-receipt test. Final auditing remains global; its work is not charged to ordinary commit timing or removed from qualification.

### Q. Capsule 11→12

Attempted only after capsule 10→11 full seal passed and the runtime reduction was measured. Total {b['seconds']:.6f} s versus 75.177814 s; ordinary degree {b['timings']['incremental_degree']:.6f} s versus 37.814181 s. All {second['comparison']['witnesses']} witness degree/ray rows equal the global reference; exact replay passes. Complete saved coordinates/connectivity, all geometry component hashes, full quality and prospective incidence, complete relation coverage and fresh proof outcomes match the prior accepted layer 12. Inherited proofs come from the same verified source checkpoint. Missing/duplicate/forbidden/unresolved: 0/0/0/0. No layer 13 was attempted.

### R. Bent-passage regression

Preserved 5→6 transition: {bent['layer']['seconds']:.6f} s; ordinary degree {bent['layer']['timings']['incremental_degree']:.6f} s; {bent['comparison']['witnesses']} witness degree/ray rows exactly equal global reference. Independent full mesh/quality/certificate seal and deterministic degree replay pass. Missing/duplicate/forbidden/unresolved: 0/0/0/0. No geometry-specific tuning.

### S. SF2 regression

17 native SF2 tests pass, included with degree tests in SF2_DEGREE_TESTS.txt. Preserved full shell/certificate authority remains 2126040 cells, 56462864 relations, zero remesh, exact wall/first-layer/restart, and zero missing/duplicate/forbidden/unresolved. Its provider stays on the conservative path. The full reference degree predicate is AST-identical apart from optional counters. A separate stored full SF2 per-witness degree table was not found in the inspected native final audit: full SF2 degree rows and a fresh full SF2 degree run are Not measured. This is a bounded preservation regression, not a new SF2 incremental-degree performance or full-degree qualification claim.

### T. Remesh fallback

Changed or retired witnesses, changed committed coordinates, unsupported cell types and unproved boundary chains use the existing complete global audit. General ProductionFast remesh retains the original provider path. Incremental historical/new degree rejection is translated to the existing GlobalDegreeFailure signal, preserving the runner's rejected-nominal correction handoff. Three focused cases cover incremental historical/new rejection and failed global fallback. This failure-only adapter does not change the successful benchmark path. Focused retirement, unsupported-type, unproved-chain, transaction rollback and termination tests pass. No large remesh case was run. Mixed-topology incremental degree support is not claimed.

### U. Runtime and memory

| Degree substage | 10→11 seconds | 11→12 seconds |
|---|---:|---:|
{runtime}

These disjoint subscopes are nested inside the ordinary degree stage. Their sum excludes small precheck/result bookkeeping. Global reference, degree replay and independent seal are outside accepted-layer timing. Exact screening-only versus interval-query-exclusive timing is Not measured separately; both are in direction_screening.

| Population | 10→11 | 11→12 |
|---|---:|---:|
{counts}

Layer-11 index array bytes: {d['memory']['index_array_bytes']}; interval arrays: {d['memory']['interval_array_bytes']}; largest one-query ID array: {d['memory']['largest_query_ids_bytes']}. Degree-row serialized Python representation: {d['memory'].get('receipt_repr_bytes',d['memory'].get('receipt_canonical_bytes'))} bytes. The initial label receipt_canonical_bytes was inaccurate and is corrected to receipt_repr_bytes in current code; it was never an authority hash. Native tree allocations, exact Python receipt heap, and aggregate temporary-buffer peak: Not measured. Layer-11 peak RSS {a['peak_rss']} bytes; retained {a['retained_rss']}. Layer-12 peak {b['peak_rss']}; retained {b['retained_rss']}. Current indexes are rebuilt in memory; no durable index or receipt table is required per ordinary layer.

The largest ordinary layer-11 stages are continuous certification {a['timings']['continuous_certificate']:.6f} s, incremental degree {a['timings']['incremental_degree']:.6f} s, and hard quality {a['timings']['hard_quality']:.6f} s. No estimated runtime savings are reported.

### V. Determinism

All production comparisons replay degree selection, directions, counts and receipt root exactly. The final-code capsule replay reverses addition order and retains the same root. Global result arrays are saved with SHA-256 bindings. The capsule and bent final mesh identities remain exact. Boundary roots are bound to geometry/topology and implementation version, not timing or object IDs.

### W. Tests

Final focused suite: 112 passed in 3.08 s, two expected warnings from the existing intentionally invalid prism. SF2 plus then-current degree suite: 27 passed in 21.77 s; ten degree tests overlap. There are 129 distinct passing tests across these runs. The first focused run had one incorrect synthetic expectation: the symmetric octahedron retried the same ray before and after. A fixed analytic shear removes that unintended symmetry; the revised test exercises an actual first-ray to second-ray transition and passes. A later final-seal stale-receipt test initially assigned the already-selected ray and therefore did not corrupt the receipt; assigning an incorrect degree makes the intended rejection check pass. Both initial test failures remain recorded. No production mismatch occurred. Existing full V4 tests were retained; the entire repository suite was not rerun.

### X. Classification

{classification}. Scope: ordinary insertion-only complete all-prism analytic advancement, with exact global fallback outside that scope and unchanged final audit. Capsule 10→11 and 11→12, bent passage, exact replay, full global comparisons and bounded SF2 preservation pass. The SF2 per-witness-table limitation above is explicit. Index construction still scales with witness count, and continuous certification still searches relevant historical faces. No universal scaling or mixed-remesh acceleration claim is made.

### Y. Recommended next task

Resume bounded production-scale geometry advancement using ProductionFast. If another performance task is chosen first, target measured continuous certification: it is now the largest ordinary stage. Do not change its candidate completeness or exact proof mathematics. No additional production advancement is performed in this task.
'''
    (OUT/'report.md').write_text(text,encoding='utf-8')
    artifact_hashes={str(p.relative_to(OUT)).replace('\\','/'):sha(p) for p in OUT.rglob('*') if p.is_file() and p.name!='AUTHORITY.json'}
    paths=['tools/incremental_degree.py','tools/production_fast_layer.py','tools/cavity_volume_audit.py','tools/profile_degree_dependencies.py','tools/benchmark_incremental_degree.py','tools/replay_saved_degree.py','tools/report_incremental_degree.py','tools/benchmark_production_fast.py','tests/test_incremental_degree.py']
    save(OUT/'AUTHORITY.json',dict(schema='V4IncrementalDegreeFillingQualification/1',classification=classification,
        starting_head=start['head'],report_sha256=sha(OUT/'report.md'),artifacts=artifact_hashes,
        implementation={p:sha(ROOT/p) for p in paths},degree_authority='V4ClosedChainDegreeReceipt/1',
        final_code_capsule_11_root=replay['root'],reference_predicate_unchanged=True,final_global_audit_required=True,
        capsule_transitions=2,sf2_scope='Bounded preservation; full per-witness degree table not measured',v3_edits=0,
        next_task='Resume bounded ProductionFast production-scale advancement; measured performance target is continuous certification'))
    print(json.dumps(dict(classification=classification,report_sha256=sha(OUT/'report.md'),authority_sha256=sha(OUT/'AUTHORITY.json')),indent=2))

if __name__=='__main__':main()
