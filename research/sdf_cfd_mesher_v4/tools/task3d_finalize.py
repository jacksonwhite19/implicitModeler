"""Seal measured Task-3D evidence. No scientific execution or geometry writes."""
from task3d_preflight import *
from sdf_cfd_mesher_v4.forecast.detailed.scientific import scientific_hash
from sdf_cfd_mesher_v4.certificate.incremental import private_admission
import shutil

CLASS='V4FutureCertificationLocalityInsufficient'
def read(name):return json.loads((OUT/name).read_text())
def write(name,value):(OUT/name).write_text(json.dumps(value,indent=2,sort_keys=True)+'\n')
def table(headers,rows):
    return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+['| '+' | '.join(map(str,r))+' |' for r in rows])
def maximum_rss(x):
    if isinstance(x,dict):return max([v for k,v in x.items() if k in ('peak_rss','peak_wset') and isinstance(v,(int,float))]+[maximum_rss(v) for v in x.values()]+[0])
    if isinstance(x,list):return max([maximum_rss(v) for v in x]+[0])
    return 0

if __name__=='__main__':
    start=read('PREFLIGHT.json');assert git('rev-parse','HEAD')==start['starting_head'];assert not git('diff','--cached','--name-only')
    preserved=audit();outside=lambda s:sorted(x for x in s.splitlines() if 'research/sdf_cfd_mesher_v4/' not in x)
    before=outside(start['starting_status']);after=outside(git('status','--porcelain'))
    prior_actual={r['path']:r['actual'] for r in start['prior_frozen_audit']['rows']}
    original_change=[r['path'] for r in preserved['prior_frozen_audit']['rows'] if prior_actual.get(r['path'])!=r['actual']]
    preservation=dict(frozen_authorities_passed=True,task3c_bindings=preserved['task3c_bindings'],bounded_original_changes_since_start=original_change,
        v3_modifications=0,sealed_result_modifications=0,unrelated_status_unchanged=before==after,unrelated_status_rows=len(after),
        unrelated_added=sorted(set(after)-set(before)),unrelated_removed=sorted(set(before)-set(after)),
        preexisting_live_origin_drift=preserved['prior_frozen_audit']['outside_scope_origin_drift'])
    assert not original_change;write('PRESERVATION.json',preservation)
    decomposition=read('PAIR_DECOMPOSITION.json');equiv=read('EQUIVALENCE.json');et=read('EQUIVALENCE_TIMINGS.json')
    proofs=[read(f'PROPOSAL_{i}.json') for i in range(2)];pt=read('PROPOSAL_TIMINGS.json');det=read('DETERMINISM.json');tests=read('TESTS.json')
    assert det['passed'] and det['run1']==det['run2'];assert equiv['complete_record_equality']
    assert all(not p['admitted_to_private_state'] and not p['certificate']['complete'] for p in proofs)
    assert all(p['inherited_identical_geometry_pairs']==0 for p in decomposition)
    expected=set(json.loads((ROOT/'results/task3c/TESTS.json').read_text())['failed_tests'])
    assert set(tests['failed_tests'])==expected and tests['errors']==0
    now=datetime.datetime.now(datetime.timezone.utc);elapsed=(now-datetime.datetime.fromisoformat(start['started_utc'])).total_seconds()
    peak=max(maximum_rss(read(p.name)) for p in OUT.glob('*TIMINGS.json'))
    resources=dict(task_elapsed_seconds=elapsed,maximum_recorded_worker_peak_rss=peak,free_disk_bytes=shutil.disk_usage(ROOT).free,
        original_decomposition_seconds=read('DECOMPOSITION_TIMINGS.json')['total_seconds'],vectorized_decomposition_seconds=read('DECOMPOSITION_REPLAY_TIMINGS.json')['total_seconds'],
        no_full_forecast_regeneration=True,no_global_dense_allocation=True,maximum_active_support_vertices=400,maximum_columns=3121,maximum_provisional_prisms=62420,
        certificate_source_edges=[d['candidate_edges'] for d in decomposition],certificate_side_surfaces=[d['candidate_side_surfaces'] for d in decomposition])
    write('RESOURCES.json',resources)
    decisions=[private_admission(immediate_numeric=p['immediate_hard_gates']['passed'],immediate_exact=p['immediate_exact_jacobians']['passed'],
        current_certificate_complete=p['current_passed'],future_certificate=p['certificate'],new_earlier_failures=p['new_earlier_numeric_failures'],target_maintained_or_improved=True) for p in proofs]
    assert all(not d['admissible'] and not d['advances_state'] for d in decisions)
    steering=dict(private_steps=0,accepted_private_candidates=0,production_committed_layer=0,admission_decisions=decisions,
        reason='Neither expanded proposal has a complete required future certificate. Private steering admission is false.',
        targets=dict(positive_cap_weight_min=0.03957811542889278,positive_cap_weight_first_invalid=18,negative_aft_exact_jacobian_first_invalid=13,
                     negative_trailing_known_pair='CertifiedSeparated',negative_leading_selected_jacobians='Positive through layer 20'),
        no_new_geometry=True,task4_ready=False)
    write('PRIVATE_STEERING.json',steering)
    summary=dict(task='ImplementV4IncrementalFutureCertificationAndResumePrivateSteering',classification=CLASS,
        bounded_equivalence_passed=True,cap_admission_complete=False,cap_proposals_admitted=0,private_steps=0,task4_ready=False,
        original_candidate_pairs=[d['candidate_pairs'] for d in decomposition],reusable_baseline_pair_proofs=[0,0],
        candidate_pairs_examined=[p['certificate']['completed_pairs'] for p in proofs],remaining_pairs=[p['full_future_scope_not_examined'] for p in proofs],
        warm_prefix_hits=[p['cache']['warm_hits'] for p in proofs],scientific_hash=det['run1'],v3_modifications=0)
    write('SUMMARY.json',summary)
    pair_table=table(['Scope','Baseline pairs','Candidate pairs','Changed / changed','Changed / unchanged','Unchanged / unchanged'],[
        ['Positive cap' if d['proposal']==0 else 'Negative cap',d['baseline_pairs'],d['candidate_pairs'],d['changed_changed_pairs'],d['changed_unchanged_pairs'],d['unchanged_unchanged_pairs']] for d in decomposition])
    layer_table=table(['Scope','Same layer','Adjacent layers','Nonadjacent layers','Future side surfaces'],[
        ['Positive' if d['proposal']==0 else 'Negative',*[sum(x['count'] for x in d['pair_count_by_layer_pair'] if fn(abs(x['layers'][0]-x['layers'][1]))) for fn in (lambda k:k==0,lambda k:k==1,lambda k:k>1)],d['candidate_side_surfaces']] for d in decomposition])
    profile_table=table(['Scope','Exact pairs examined','Unexamined','Cold seconds','Warm-prefix seconds','Warm exact calls','Cache record bytes'],[
        ['Positive' if p['proposal']==0 else 'Negative',p['certificate']['completed_pairs'],p['full_future_scope_not_examined'],round(t['cold']['seconds'],6),round(t['warm']['seconds'],6),t['warm']['exact_kernel_calls'],t['cold']['cache_bytes']] for p,t in zip(proofs,pt['stages'])])
    failed='\n'.join('- `'+n+'`' for n in tests['failed_tests'])
    report=f'''## Decision question

Can exact future certification for a locally corrected advancing-front candidate be made incremental and local without changing certificate mathematics, and does that enable valid private steering of the remaining cap hazards?

**{CLASS}**.

Exact incremental proof reuse passes bounded qualification. It does not remove the first expanded-cap workload: every required candidate pair has at least one changed surface. Stable pair IDs are mostly inherited, but **zero baseline pair proofs are reusable**. Both expanded proposals remain inadmissible because their future certificates are incomplete. No private or production layer advances.

### A. Authority verification

Branch: `{start['branch']}`. Starting HEAD: `{start['starting_head']}`.

- Task-3C report: `680f0d49d5867244aab17c01971ccf5fa146b2651419837b953b091e177023ef`.
- Task-3C authority: `5ed30f0be4d02c5b972dbb74a37ad9a4829d3d9c92bc5c6061c238c815de2651`.
- Qualified certificate mathematics integration remains `bab10e0d52d724a6d0a525b801e2486ae3fc1af9c0f02fa1030da5a8dcc99b7f`.

All active frozen source and sealed-result bindings pass. V3 modifications: **0**. Bounded original-source changes during this task: **0**. The two live production-origin differences in `naca_spline.rs` and `wing.rs` predate this task and were already recorded in Task-3C. This execution uses unchanged frozen V4 copies. Historical tests remain unchanged.

Unrelated Git status changes from {len(before)} to {len(after)} rows during the run. External work adds a modified `src/render/camera.rs`, four camera/viewport probe examples, and `research/viewport_fidelity_navigation/`. Those six entries are listed in PRESERVATION.json. No prior unrelated entry disappears. This task leaves all external work untouched and stages none of it; it does not claim the unrelated status stayed identical.

### B. Origin of the 1.18M-pair workload

{pair_table}

{layer_table}

The positive proposal changes 389 vertex trajectories and 39,080 side surfaces. The negative proposal changes 392 trajectories and 39,040 side surfaces. Those vertices differ at every layer 1-20. Their interaction halos contain 4,322 and 4,301 source edges. All pairs involving the changed core are genuinely affected; the old traversal already excluded unchanged-vs-unchanged pairs outside this scope.

Of the candidate pair IDs, 1,171,719 and 1,171,553 also existed in the baseline enumeration. Their geometry is different, so their old exact proofs cannot transfer. There are 9,110 and 8,530 new pair IDs. The correction removes 1,270,369 and 1,275,077 baseline AABB candidate pairs. This is a geometric broad-phase reduction relative to the nominal baseline, not a new cache optimization.

PAIR_DECOMPOSITION.json records every layer-pair count, source chord/span bin, changed edge and changed vertex by layer. Spatial bins are descriptive partitions of the source extent, not risk thresholds. Pairs outside changed interaction support: **0** in each required candidate scope.

### C. Proof identity and dependency model

`certificate/incremental.py` implements `SurfaceScope`, `ProofCache`, `dependency_changes`, `proof_invalidation`, `conservative_pairs`, `certify`, and a read-only `private_admission` guard. PROOF_IDENTITY.md specifies the exact keys.

Surface identity binds ordered binary64 coordinates, ownership labels, source edge/layer IDs, dimension, topology and repaired-kernel authority. Pair identity binds both surface identities. No approximate equality or ID-only reuse is allowed. Exact record bytes are immutable. Enumeration-cache identity also binds core scope.

### D. Incremental invalidation design

Any changed endpoint invalidates its incident side geometry. Actual reforecasted coordinates are inspected at every layer, including both neighboring transitions. Old active pair bindings invalidate when either surface changes. Immutable old records remain under their old keys for rollback. Missing records never mean separation.

Tests cover one moved vertex, multiple vertices, one changed future layer, several changed layers, exact rollback and identical replay. On the bounded SF2 case, {equiv['proof_invalidation']['invalidated_proofs']} existing proofs invalidate and {equiv['proof_invalidation']['unchanged_proof_bindings']} retain their exact inputs. The cap baseline has no reusable pair identity, so computing 2.44M baseline proofs solely to seed its next candidate would add work without reuse. Those cap baseline proofs were deliberately not computed. Potential invalidated bindings are distinguished from computed proof records.

### E. Spatial candidate completeness

The unchanged swept-edge AABB scan includes all potentially interacting external edges. The new deterministic BVH applies the same inclusive side-AABB predicate, vectorizes leaf tests, and skips branches that contain no core surface on either side. It includes changed-vs-unchanged interactions and cross-layer pairs. No distance cutoff or new geometry theorem is added. Bilinear images lie in their control-point hulls, which provides the retained conservative exclusion argument.

Full canonical pair hashes match the original enumeration for both cap candidates and both baselines. Candidate pair reduction from the new index: **0**. The index reduces enumeration overhead, not mathematical scope.

The first large-scope check exposed a new Windows 32-bit pair-packing overflow in this study's wrapper. The frozen kernel was unaffected. BVH_PACKING_REPRODUCER.json and BVH_PACKING_BEFORE.py.txt preserve the failure. Explicit 64-bit packing repairs it. The regression and all four large pair-population comparisons then pass. No failing enumeration is used in the accepted evidence.

### F. Full-versus-incremental equivalence

The actual SF2 sampling-control case contains **1,791 complete future pairs**. One source vertex receives a deterministic projected tangent probe of h1/32; it is a cache test, not an accepted geometry correction.

The independent frozen complete caller and new incremental caller produce identical normalized exact records, forbidden sets and unresolved sets. Scientific record hash: `{equiv['normalized_full_hash']}`.

- Cold baseline: 1,791 exact pair calls.
- Changed candidate: **488 hits, 1,303 new exact calls** ({100*488/1791:.6f}% reuse).
- Identical warm candidate: **1,791 hits, zero exact calls**.
- Rollback: **1,791 hits, zero exact calls**.

This qualifies exact reuse on the tested bounded changes. It does not imply cap-wide changes have the same reuse fraction.

### G. Expanded-support proposal replay

Both saved layer-1 coordinate sets, subsequent forecast coordinates, normals, topology and pair-population hashes reproduce exactly. Their complete immediate certificates are reused only with identical authority and geometry. The current gates still pass: 2,340 and 2,334 exact positive Jacobians/volumes, and 24,912 and 24,819 passing current-layer side pairs. The negative-cap known edge-1/layer-1 versus edge-729/layer-10 pair remains exactly `CertifiedSeparated` on direct replay.

{profile_table}

The two cold prefixes contain {len(proofs[0]['certificate']['forbidden'])} and {len(proofs[1]['certificate']['forbidden'])} forbidden witnesses, and {len(proofs[0]['certificate']['unresolved'])} and {len(proofs[1]['certificate']['unresolved'])} unresolved records. This is **prefix evidence only**. The unexamined remainders are unknown. Neither certificate is complete. Both private admissions remain false.

### H. Certificate work reduction

The fixed 250,000-pair abort is removed from this opt-in path. Exact proof work is limited by measured runtime and bounded cache memory, not a larger forced pair allowance. Each cold cap profile uses the existing 180-second operational allowance. The number it actually completes defines its deterministic replay slice. The replay cap is a computational bound, not a scientific validity condition.

The original four-population decomposition took {resources['original_decomposition_seconds']:.6f} s; the vectorized equivalent took {resources['vectorized_decomposition_seconds']:.6f} s. The complete small candidate took {et['full_seconds']:.6f} s with the frozen full caller and {et['incremental']['seconds']:.6f} s incrementally. Its cold baseline took {et['cold_baseline']['seconds']:.6f} s and its identical warm replay {et['warm']['seconds']:.6f} s. These are direct run measurements, not timing guarantees.

For the two cap corrections, initial baseline-proof reuse is **0%** and initial required narrow-phase pair-count reduction is **0**. Warm prefix reuse is 100%, but neither full future proof exists. No full-cap completion runtime is extrapolated from these prefixes.

### I. Private multi-step steering

Not entered. Accepted private candidates: **0**. Private steps: **0**. Production committed layer: **0**. The read-only admission guard rejects incomplete future coverage even when the immediate layer and the known intersection test pass. It requires explicit coverage counts and all failure sets; a bare complete flag is insufficient. This new API boundary has a missing-evidence regression test. No invalid intermediate step or direct future-coordinate edit is used.

### J. Remaining cap hazards

No new geometry search occurs in this task. The frozen positive-cap proposal still has weight minimum 0.03957811542889278 and first weight failure at layer 18. The aft-cap exact Jacobian first fails at layer 13. The selected trailing-cap and leading-cap exact Jacobian failures were removed through layer 20 by Task-3C. The known trailing-cap intersection remains separated. Absence of other future intersections is unproved for both complete scopes.

### K. Conditioning effects

The proposals are unchanged, so no new conditioning trade is claimed. Task-3C's positive local minimum sigma changed 0 to 0.06568645499996636 and maximum available G_bound 90.35271487645 to 81.17360267527995. Negative local minimum sigma changed 0 to 0.004254083531504211, while maximum available G_bound increased 433.17234486131184 to 545.0125182660814. Singular unavailable baseline operators are excluded from those available-gain maxima. No conditioning gate is added.

### L. Runtime, memory, and cache behavior

Study elapsed at sealing: **{elapsed:.3f} s**. Maximum recorded worker peak RSS: **{peak} bytes**. Exact-record cache budget: 128 MiB of encoded keys/results; enumeration-array budget: 64 MiB. These are computational memory controls; Python overhead is measured through RSS. No cache eviction occurs in the measured prefixes. The lookup cache is memory-only (persistent lookup storage: 0 bytes). The compressed positive/negative scientific proof streams occupy {pt['stages'][0]['persisted_proof_bytes']} / {pt['stages'][1]['persisted_proof_bytes']} bytes. They preserve all exact completed records for verification.

Maximum existing detailed scope remains 400 active-support vertices, 3,121 columns and 62,420 provisional prisms. The broad phase reads full source-edge AABBs to prove exclusion but never constructs a full-shell connected exact domain or a dense sensitivity system. Task-1 forecast archives and Task-3C coordinates are reused. No cold whole-front forecast, CFD, OpenFOAM, production commitment or V3 modification occurs. RESOURCES.json records storage and free disk.

### M. Determinism

Run 1: `{det['run1']}`.

Run 2: `{det['run2']}`.

Exact record bytes, pair populations, changed coordinates, complete bounded equivalence, both cap prefixes and cache identities reproduce. Full-cap certificate determinism is not claimed because those scopes are incomplete. Runtime/RSS and the reason an operational scheduler yielded are excluded; the exact completed coverage and unexamined remainder remain scientific fields.

### N. Tests

**{tests['tests']} tests: {tests['tests']-tests['failures']-tests['errors']} passed, {tests['failures']} failed, {tests['errors']} errors.** All {tests['tests']-119} new incremental tests pass. The seven failures match the pre-existing Task-3C repository-state/source-origin failures. Historical contracts are unchanged:

{failed}

The exact check command is `python research/sdf_cfd_mesher_v4/tools/task3d_tests.py`. Scientific drivers and replays are listed in AUTHORITY.json. Incremental/full equality and overflow repair are tested independently before expanded execution.

### O. Classification

**{CLASS}**.

The exact incremental design is valid on the bounded qualification case, and enumeration is faster. The required expanded-cap work remains large because the proposal changes every required pair's geometry. There is no unaltered baseline proof population to reuse. This is a measured failure of the proposed work-reduction premise for these first cap corrections, not a certificate-mathematics defect, topology impossibility or proof that global coupling is necessary.

The cap scopes remain local in memory, but complete exact assessment has not become practical within the measured repeated-candidate allowance. No cap-wide qualification or steering success is claimed. Further work must reduce the cost of proofs for genuinely changed cap geometry; identical-geometry reuse alone cannot supply the initial complete proof state.

### P. Task-4 readiness

**No.** Immediate hard-gate validity and dependency-complete cap patches are preserved, but neither required future certificate is complete and no private sequence is admitted. Production rolling commitment is not justified by this study. Task 4 is not started.

All committed paths are Task-3D V4 files listed by AUTHORITY.json, plus that authority, checksums and the local log-ignore file. The final Git hash is reported in the final response. SHA256SUMS.json binds this report and authority without circular hashing.
'''
    (OUT/'report.md').write_text(report,encoding='utf-8')
    sources=[ROOT/'src/sdf_cfd_mesher_v4/certificate/incremental.py',ROOT/'tests/test_incremental_certificate.py']+sorted((ROOT/'tools').glob('task3d_*.py'))
    evidence=sorted(p for p in OUT.iterdir() if p.is_file() and p.suffix!='.log' and p.name not in ('AUTHORITY.json','SHA256SUMS.json','.gitignore'))
    authority=dict(task=summary['task'],classification=CLASS,branch=start['branch'],starting_head=start['starting_head'],v3_modifications=0,
        accepted_private_candidates=0,production_committed_layer=0,task4_ready=False,scientific_hash=det['run1'],
        report_sha256=file_hash(OUT/'report.md'),source_sha256={str(p.relative_to(ROOT)):file_hash(p) for p in sources},
        evidence_sha256={str(p.relative_to(ROOT)):file_hash(p) for p in evidence},
        frozen_task3c_authority='5ed30f0be4d02c5b972dbb74a37ad9a4829d3d9c92bc5c6061c238c815de2651',
        scope='Complete bounded SF2 equivalence; incomplete expanded-cap future certificates with exact reproducible prefixes.')
    write('AUTHORITY.json',authority)
    write('SHA256SUMS.json',dict(report_sha256=file_hash(OUT/'report.md'),authority_sha256=file_hash(OUT/'AUTHORITY.json')))
    for _ in range(5):
        total=sum(p.stat().st_size for p in ROOT.rglob('*') if p.is_file())
        own=sources+[p for p in OUT.iterdir() if p.is_file() and p.suffix!='.log'];new=sum(p.stat().st_size for p in own);largest=max(own,key=lambda p:p.stat().st_size)
        update=dict(v4_total_bytes=total,new_task_bytes=new,largest_new_file=str(largest.relative_to(ROOT)),largest_new_file_bytes=largest.stat().st_size,
            files_above_100_MB=[str(p.relative_to(ROOT)) for p in own if p.stat().st_size>100_000_000])
        if all(resources.get(k)==v for k,v in update.items()):break
        resources.update(update);write('RESOURCES.json',resources)
        authority['evidence_sha256'][str((OUT/'RESOURCES.json').relative_to(ROOT))]=file_hash(OUT/'RESOURCES.json')
        write('AUTHORITY.json',authority);write('SHA256SUMS.json',dict(report_sha256=file_hash(OUT/'report.md'),authority_sha256=file_hash(OUT/'AUTHORITY.json')))
    else:raise AssertionError('StorageAccountingDidNotStabilize')
    print(json.dumps(read('SHA256SUMS.json'),indent=2))
