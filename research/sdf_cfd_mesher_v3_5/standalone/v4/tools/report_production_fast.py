"""Seal measured task evidence; does not execute geometry or validation."""
import json,hashlib,subprocess
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/production_fast_incremental_validation'
REPO=ROOT.parents[1]
def load(p):return json.loads(p.read_text())
def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):p.write_text(json.dumps(v,indent=2,sort_keys=True)+'\n')
def identity(v):return hashlib.sha256(json.dumps(v,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def main():
    a=load(OUT/'capsule_10_11_final/LAYER.json');s=load(OUT/'capsule_10_11_final/SEAL.json')
    b=load(OUT/'capsule_11_12/LAYER.json');bs=load(OUT/'capsule_11_12/SEAL.json')
    bent=load(OUT/'bent_5_6_fixed/LAYER.json');bents=load(OUT/'bent_5_6_fixed/SEAL.json')
    recovery=load(OUT/'capsule_11_12/FINAL_REPRESENTATION_AUDIT.json')
    assert all(x['passed'] for x in (s,bs,bents,recovery))
    composition=dict(schema='V4ProductionFastFinalSeal/1',geometry=bs['geometry_components'],
        quality=bs['quality_digest'],prospective=bs['prospective_digest'],
        certificate={k:bs['certificate'][k] for k in ('authority','root','count')})
    save(OUT/'capsule_11_12/FINAL_AUTHORITY.json',dict(scientific=identity(composition),composition=composition,
        seal_sha256=sha(OUT/'capsule_11_12/SEAL.json'),representation_audit_sha256=sha(OUT/'capsule_11_12/FINAL_REPRESENTATION_AUDIT.json'),
        geometry_sha256=recovery['geometry_sha256'],scope='Explicit final-seal composition; not a reinterpretation of legacy checkpoint hashes'))
    initial=load(OUT/'START.json')
    for p,h in initial['sealed'].items():assert sha(REPO/p)==h
    status=subprocess.check_output(['git','status','--porcelain=v1','-z'],cwd=REPO).decode('utf-8')
    def unrelated(text):return sorted(r for r in text.split('\0') if r and 'research/sdf_cfd_mesher_v4/' not in r)
    assert unrelated(status)==unrelated(initial['status']),'Unrelated dirty state changed'
    tracked_v3=subprocess.check_output(['git','diff','--name-only','HEAD','--','research/sdf_cfd_mesher_v3'],cwd=REPO).decode().splitlines()
    assert not tracked_v3
    save(OUT/'GIT_PRESERVATION.json',dict(start_head=initial['head'],unrelated_status_exact=True,
        unrelated_status_entries=len(unrelated(status)),v3_tracked_modification_count=0,
        v3_note='Pre-existing untracked V3 directory preserved; no V3 edits issued. Full byte inventory not measured.',prior_seals_unchanged=True))
    baseline=dict(total=570.063030,proposal=260.700261,numerical=91.791116,volume_conformity=41.824380,
        certification_validation=45.964027,checkpoint_publication=33.832317,checkpoint_loads=[14.612922,14.429621,18.265013],
        peak_rss=4327010304,retained_rss=2761502720)
    save(OUT/'BASELINE.json',dict(values=baseline,source='Preserved successful canonical-block layer 11; user supplied disjoint timers and stored TRACE.json, BLOCK_TIMERS.json and PERFORMANCE.json',
        scope='No baseline rerun; digest aggregate 177.093604 s has a wider overlapping scope and is not added'))
    count_table=[
        ('Complete cell-quality evaluations',180224,8192),
        ('Historical complete cell-quality evaluations',163840,0),
        ('Historical changed-role determinant evaluations',327680,8192),
        ('All determinant calculations (local plus prospective)',360448,24576),
        ('Internal-face metric evaluations',434176,20480),
        ('Unchanged historical internal-face metric evaluations',393216,0),
        ('Historical exact Jacobians, quality plus volume audit',245760,0),
        ('Historical admission SDF points',40980,0),
        ('Complete cell/connectivity reconstructions',4,0),
        ('Complete ownership-map rebuilds',4,0),
        ('Complete scientific-state hash passes','Not measured',0)]
    save(OUT/'WORK_COUNTS.json',dict(rows=[dict(operation=k,before=x,after=y) for k,x,y in count_table],
        semantics='Before values are code-derived per successful layer unless marked. After counts apply to ordinary commit, exclude bootstrap and independent final seal.',
        determinant_note='Historical before includes two local and two prospective full passes. After evaluates 8192 former-top local tensors, reuses their prospective tensors, and evaluates 8192 new local plus 8192 new prospective tensors.',
        sdf_note='Counts are admission checks. Existing provider.geometry still constructs analytic historical fronts then restores committed coordinates; its internal SDF call count was not measured.',
        topology_note='Four ownership rebuilds exclude native face packing and two additional prospective incidence scans. Metadata dictionaries still copy; no zero historical memory-touch claim.'))
    rows='\n'.join(f'| {k} | {x} | {y} |' for k,x,y in count_table)
    timings='\n'.join(f'| {k} | {v:.6f} | {b["timings"].get(k,0):.6f} |' for k,v in a['timings'].items())
    remainder=a['seconds']-sum(a['timings'].values())
    classification='V4ProductionFastIncrementalValidationQualified'
    text=f'''## Decision question

Can V4 separate production execution from qualification persistence and validate an ordinary advancing-front layer only over scientifically affected dependencies while preserving every frozen hard gate and the independent final whole-shell audit?

### A. Authority verification

Starting HEAD: {initial['head']}, on the requested feature branch. The stopped canonical-block task was already committed. Prior report and AUTHORITY hashes match START.json and remain unchanged. The copied block layer-10 checkpoint referenced blocks previously removed by compaction. The preserved Patricia layer-10 checkpoint remained valid and was used instead. No historical authority was edited.

### B. Existing layer-11 baseline

Preserved accepted runtime: 570.063030 s. Proposal: 260.700261 s; numerical validation: 91.791116 s; certification validation: 45.964027 s. Global volume/conformity audit: 41.824380 s, nested in proposal. Checkpoint publication: 33.832317 s. Three checkpoint loads: 14.612922, 14.429621, 18.265013 s. These were not rerun. Initial restoration/bootstrap in the new benchmark is reported separately; the comparison is an execution-profile comparison, not a cold-start speedup claim.

### C. Execution-profile architecture

ExecutionProfile defines ProductionFast, Qualification and Diagnostic. MixedFrontRunner dispatches the same qualified analytic ordinary construction to FastLayer. Other providers, mixed topology, partial cohorts, and rejected nominal candidates retain the existing provider correction/remesh path through an in-memory journal. No case-name or geometry-size branch is added. The optimization is qualified for ordinary complete all-prism cohorts, not a new mixed-remesh performance claim.

### D. ProductionFast state contract

Maintain one current native geometry, incremental ownership/boundary state, current numeric/exact/prospective quality maps, exact cell and boundary volume sums, rational cell witnesses, immutable compact proof segments, current delta proofs, cached scientific entity identities, vertex-face incidence, and a transaction generation. Candidate caches are private until all predicates pass. Failed candidates leave current state intact. There is no ordinary durable checkpoint, raw receipt SQLite overlay, historical replay root, or global authenticated publication. Core ingestion remains enabled and writes its small required exchange file. State is explicitly unsealed; scientific_hash rejects it until final sealing. Conservative correction/remesh handoff can still perform full validation and receipt binding; its performance is not qualified here.

### E. Qualification state contract

Qualification remains the default. It keeps native checkpoints, the durable transaction journal, complete receipt publication, restart verification, and the existing provider validation callbacks. Existing checkpoint, retention and transaction tests pass. No expensive Qualification layer rerun was needed because preserved accepted authority and independent full comparisons were available.

### F. Diagnostic state contract

Diagnostic uses the same durable scientific path and adds an external timing record. Its dispatch test proves the instrumentation does not alter returned scientific results. This task adds the profile boundary, not a comprehensive native-memory census. Detailed benchmark artifacts are qualification instrumentation outside ordinary production commits.

### G. Current whole-shell validation

The former path called complete hard quality twice, rebuilt ownership, traversed complete determinant incidence, checked historical SDF targets and repeatedly hashed native dictionaries. The ordinary fast path replaces those calls with a single affected-set evaluation. Before counts are code-derived; they are not sampled profiler counters.

| Operation | Before | After |
|---|---:|---:|
{rows}

Historical geometry still participates in the global degree predicate and exact-safe new-old certificate search. Dictionary metadata copies and face-index remapping remain shell-size operations. Historical SDF admission checks are eliminated; existing analytic front construction still produces historical arrays before committed coordinates overwrite them. Its internal SDF-call population is Not measured.

### H. Dependency-local validation design

Topology.update derives touched faces from inserted/retired/replaced cells. Quality.update derives affected owner cells from those faces. New cells receive every original cell predicate; their incident internal faces receive the original pair metrics. Historical changed-role cells receive the local determinant recomputation. Unchanged receipts are reused only inside a private cache bootstrapped from a verified checkpoint with matching implementation authority. The fast append does not permit mutation of committed coordinates or cells. General correction/remesh remains on the conservative validated path.

### I. Cell metric reuse

8192 new cells are hard-evaluated. 81920 prior cells avoid full reevaluation. Geometry-only volume/aspect/exact values are retained. 8192 former top cells have a determinant-only update. Counter 'new_changed_cells' excludes those determinant-only historical cells. No geometry-specific population is hard-coded.

### J. Face metric reuse

20480 new or newly internal faces receive weight, skewness, volume-ratio and non-orthogonality evaluation through the existing validator. 196608 unchanged historical internal faces are reused. 8192 old exposed cap faces are newly internal; they are correctly included among the 20480 and are not treated as reusable internal-face receipts.

### K. Determinant applicability

Selected-face tensor mathematics and 0.0011 threshold are unchanged. Shell-local determinant changes when a previous cap becomes internal. Its 8192 historical cells are recomputed. Their prospective selected-face masks remain identical, so prospective determinants are reused and ownership dependencies rebound. New cells evaluate both local and prospective tensors. Full final incidence independently matches every retained row.

### L. Exact Jacobian reuse

8192 new exact cell/Jacobian checks; zero historical exact recomputation during the ordinary commit. Historical exact signed volumes and Jacobians come from verified committed evidence. The independent seal intentionally reruns the global quality/exact checks. The known-invalid zero-height prism test rejects and leaves prior receipts intact.

### M. SDF validation reuse

4098 new front points receive the original source-error-bound admission check. Historical admission targets are unchanged by append and are reused. The sealed comparison proves wall, first layer, all coordinates and schedule authority exactly equal. General correction/source-change handling remains conservative.

### N. Incremental topology

Topology.update updates owner lists and exposed membership for touched faces. It rejects excess owners and inconsistent internal winding. Boundary comparison and vertex/edge manifold checks cover the current exposed front. native_append changes only new cell entities and touched face roles. Cached vertex-face incidence and per-entity identities feed the existing rebuild_dependencies; atlas and forecast compare exactly with the full reconstruction. An unordered quad incidence key was initially misused as a polygon perimeter in dependency hashing; the fixed code recovers the ordered owner face. The failed bent attempt is retained, and a focused regression covers the distinction and conservative receipt handoff.

### O. Exact volume aggregate

Exact Fraction total = prior total minus removed/replaced contributions plus new contributions. It equals the independent full total on focused fixtures and the complete layer-12 audit. No historical exact cell validity is recomputed by this aggregate.

### P. Boundary volume aggregate

Remove old cap contributions, add new caps, and update any changed boundary coordinates. Exact cell and oriented-boundary totals match. Unchanged wall contributions remain authoritative. This is an arithmetic aggregate only, not a replacement for the degree or continuous certificate predicates.

### Q. Degree audit

The original cavity_volume_audit.degree remains global and exact. Layer 11 checks all 90112 cell witnesses against 16384 oriented boundary triangles. Layer 12 checks 98304 witnesses. No old degree result is reused. Layer-11 degree time: {a['timings']['global_degree']:.6f} s. Layer-12: {b['timings']['global_degree']:.6f} s. The layer-12 independent global volume audit also passed, including positive cells, exact filling balance and zero unresolved degree witnesses.

### R. Proposal evidence reuse

The private candidate is evaluated once and committed directly after all predicates pass. The numerical and certification wrappers do not regenerate identical evidence. ProductionFast calls the same LocalTerminationPolicy with a generation-bound validated nominal ledger. The current analytic provider's accepted-nominal policy has no separate detailed lookahead or future-spot stage; no such check is removed by this change. Rejected candidates hand back to its existing correction/escalation logic. Gmsh ingestion remains in acceptance.

### S. Scientific/component identity

Immutable prior authority plus transaction generation and changed entity hashes protect reuse. Cached geometry_inputs are updated for new cells and changed face roles; authority/schedule identities remain bound. Complete scientific hash passes during ordinary commit: zero. Individual component/hash-exclusive time: Not measured; it is nested in topology/dependency/certificate scopes. The seal for layer 11 reproduces legacy scientific identity exactly. Layer 12 publishes an explicit V4ProductionFastFinalSeal/1 composition over geometry, complete quality, prospective incidence and the unchanged canonical-block certificate authority. It does not rename or reinterpret the legacy hash format.

### T. Capsule 10→11 benchmark

Final accepted runtime: {a['seconds']:.6f} s versus 570.063030 s. Bootstrap: {load(OUT/'capsule_10_11_final/BOOTSTRAP.json')['seconds']:.6f} s separately. Initial optimized replay was 72.750520 s; the final replay includes the incremental dependency-cache optimization. A later conservative-handoff face-key fix does not execute on this accepted ordinary path and is covered by tests.

| Disjoint measured stage | 10→11 seconds | 11→12 seconds |
|---|---:|---:|
{timings}
| Uninstrumented commit/bookkeeping remainder | {remainder:.6f} | {b['seconds']-sum(b['timings'].values()):.6f} |
| Total | {a['seconds']:.6f} | {b['seconds']:.6f} |

The former proposal/numerical/certification wrappers are fused and their after-times are not separately measured. Hard quality is the single 9.416030 s scope, not an additive numerical wrapper. Candidate generation versus exact kernel versus fresh-proof hashing are not separately measured inside continuous_certificate. Ordinary durable persistence operations are zero; their separate runtime is Not measured. Core-ingestion exchange I/O remains included. Final seal is outside these timers.

### U. Final seal equivalence

Layer 11: exact coordinates, connectivity, local layer state, all quality/determinant/weight/exact arrays, wall, first layer, atlas, forecast and events. Complete relation population {s['coverage']['active']}; missing 0, duplicate 0, forbidden 0, unresolved 0. Independent complete AABB coverage and full quality/incidence evaluation pass. Legacy proof digest {s['proof_digest']}. Final scientific identity {s['scientific']}. Reference receipt storage is bound only after independent geometry, metric and proof-content equality; it is not borrowed to establish the comparison. Seal timings are in SEAL.json and remain outside hot-path performance.

### V. Capsule 11→12 if attempted

Attempted only after the first 10→11 independent seal passed. Accepted {b['seconds']:.6f} s; 98304 cells; {b['relations']} relations; {b['new_relations']} new relations; no remesh. It starts from the equivalent preserved sealed layer 11 and includes separate bootstrap, not a same-process two-step timing. Complete quality and coverage pass with all four failure counts zero. Canonical root {bs['certificate']['root']}. Independent seal: {bs['seconds']:.6f} s; quality {bs['times']['quality']:.6f}, coverage {bs['times']['coverage']:.6f}, receipt-root reconstruction {bs['times']['independent_receipt_root']:.6f} s. Saved geometry was independently reconstructed from already accepted coordinates, hash-matched, globally audited and persisted; no new layer was generated by that recovery. Final geometry SHA-256 {recovery['geometry_sha256']}. Layers 13–20 were not attempted.

### W. Bent-passage regression

Preserved 5→6 transition: {bent['seconds']:.6f} s; {bent['cells']} cells; {bent['relations']} relations; {bent['new_relations']} new. Independent hard quality, prospective incidence, complete relation coverage and proof outcomes equal preserved authority exactly. Missing/duplicate/forbidden/unresolved: 0/0/0/0. Final scientific identity {bents['scientific']}. No separate geometry-specific logic.

### X. SF2 regression

17 native SF2 plumbing tests passed in 19.62 s, including restored preserved artifacts. The preserved bounded replay authority remains 2126040 cells, 56462864 relations, zero remesh and zero missing/duplicate/forbidden/unresolved, exact first-layer/wall and restart. No new full SF2 shell or A/A2 run. A generic restoration defect initially exhausted disk by joining the entire compressed archive before bounded extraction; streaming the authenticated original parts removes that temporary duplicate while retaining part, combined and extracted-file SHA-256 checks. The preserved SF2 full-population result is reused, not described as newly recomputed.

### Y. Qualification/restart regression

Native checkpoint, durable remesh and block-retention tests pass. In-memory transaction failure injection at seven existing journal boundaries preserves the prior authority and proof availability. Diagnostic dispatch has exact scientific return equality. Capsule repeated 10→11 seals and bent preserved replay are exact. Large remesh under ProductionFast is not run; the optimized branch declines mixed/local cohorts and retains original callbacks. Other providers are never replaced by the analytic adapter.

### Z. Runtime and memory

Layer 11 peak RSS: baseline 4327010304 bytes; final fast {a['peak_rss']} bytes. Retained RSS: baseline 2761502720; final fast {a['retained_rss']}. Layer 12 peak {b['peak_rss']}; retained {b['retained_rss']}. Bent peak {bent['peak_rss']}; retained {bent['retained_rss']}. These are process measurements at accepted-step end, not exact ownership attribution. Independent seal memory is not included in these accepted-step peaks; separate final-audit peak RSS: Not measured. No universal memory-scaling claim is made from two transitions. The degree, continuous certificate and hard quality are the three largest measured ordinary costs.

### AA. Tests

Final focused validation/checkpoint/incidence suites: 70 passed in 19.08 s, with two expected invalid-prism warnings. Existing remesh transaction, termination policy, complete-neighbor and proof-dependency suites: 69 passed in 1.75 s. Native SF2 suite: 17 passed in 19.62 s. Total: 156 passing tests across these final runs. Earlier 47-pass run had two SF2 extraction errors from disk exhaustion; both passed after the stream fix. One new lazy-proof test initially omitted the required proof method field; the test fixture was corrected. No frozen metric implementation, threshold, V3 source, or certificate kernel was changed. The full repository test suite was not run; focused affected suites and real transition audits were used.

### AB. Classification

{classification}. Scope: ordinary complete all-prism analytic cohorts, exact final seal, conservative compatibility with existing general provider transactions. This is not full capsule completion, broad CFD qualification, mixed-remesh acceleration, or a claim that all historical work disappeared. Degree remains global; proof search/remapping, native metadata copies and bootstrap/final seal still scale with shell size. General conservative fallback may retain costly full validation and receipt persistence. First-layer/shell scientific gates and final mathematics remain unchanged. Layer-12 geometry and compact proof delta are versioned; the large reproducible .cblock working objects remain local and are bound individually by SEAL.json. The inherited sealed checkpoint/proof segments remain required. This is a final authority artifact, not a newly qualified general-purpose layer-12 restart format.

### AC. Recommended next task

Qualify exact incremental degree/filling validation. The unchanged global degree predicate is now the dominant measured stage (35.188565 s on layer 11 and 37.814181 s on layer 12). Keep it global until an equivalent dependency-local proof is qualified. Do not run additional capsule layers in this task. No performance estimate is made for that future work.
'''
    (OUT/'report.md').write_text(text,encoding='utf-8')
    evidence={str(p.relative_to(OUT)).replace('\\','/'):sha(p) for p in OUT.rglob('*') if p.is_file() and p.suffix in ('.json','.txt','.md') and p.name not in ('AUTHORITY.json',) and 'seal_objects' not in p.parts}
    implementation={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in [
        ROOT/'src/sdf_cfd_mesher_v4/advancing_front/execution_profile.py',ROOT/'src/sdf_cfd_mesher_v4/advancing_front/mixed_front.py',
        ROOT/'src/sdf_cfd_mesher_v4/advancing_front/mixed_front_runner.py',ROOT/'src/sdf_cfd_mesher_v4/advancing_front/mixed_front_journal.py',
        *[ROOT/'tools'/n for n in ('production_fast_layer.py','incremental_layer_validation.py','live_proof_receipts.py','benchmark_production_fast.py','persist_completed_fast_geometry.py','native_sf2_artifacts.py','report_production_fast.py')],
        ROOT/'tests/test_incremental_layer_validation.py']}
    save(OUT/'AUTHORITY.json',dict(schema='V4ProductionFastIncrementalValidation/1',classification=classification,
        starting_head=initial['head'],report_sha256=sha(OUT/'report.md'),evidence=evidence,implementation=implementation,
        capsule_layer_11=s['scientific'],capsule_layer_12=identity(composition),bent=bents['scientific'],
        legacy_seals=initial['sealed'],scope='Qualified ordinary complete all-prism analytic cohorts; general providers retain conservative callbacks',
        final_degree_global=True,scientific_gates_changed=False,v3_edits=0,
        next_task='Qualify exact incremental degree/filling validation'))
    print(json.dumps(dict(classification=classification,report_sha256=sha(OUT/'report.md'),authority_sha256=sha(OUT/'AUTHORITY.json')),indent=2))

if __name__=='__main__':main()
