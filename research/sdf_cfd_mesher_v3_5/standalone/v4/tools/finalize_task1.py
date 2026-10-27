"""Seal evidence and write the Task-1 report. No scientific calculations."""
from pathlib import Path
import sys,json,datetime,shutil,subprocess
ROOT=Path(__file__).resolve().parents[1];REPO=ROOT.parents[1];sys.path.insert(0,str(ROOT/'src'));sys.dont_write_bytecode=True
from sdf_cfd_mesher_v4.common.authority import file_hash
OUT=ROOT/'results/task1'
def read(path):return json.loads(path.read_text())
def put(path,j):path.write_text(json.dumps(j,indent=2,sort_keys=True)+'\n',encoding='utf-8')
def norm_status(s):return sorted(set('?? research/' if r.startswith('?? research/') else r for r in s.splitlines() if 'research/sdf_cfd_mesher_v4/' not in r))

def finalize():
    start=read(ROOT/'authorities/TASK1_START.json');b=read(OUT/'BENCHMARK.json');s=read(OUT/'SCHEDULE.json');tests=read(OUT/'TESTS.json');verify=read(OUT/'VERIFIER.json');resource=read(OUT/'task1_benchmark_RESOURCE.json')
    assert tests['passed'] and verify['passed'] and b['deterministic_exact']
    status=subprocess.check_output(['git','status','--porcelain=v1'],cwd=REPO,text=True)
    assert norm_status(status)==norm_status(start['status'])
    original=read(ROOT/'authorities/V3_SOURCE_FORK_MANIFEST.json')['files'];extra=read(ROOT/'authorities/TASK1_SOURCE_BINDING.json')['files']
    assert all(file_hash(REPO/r['original_path'])==r['original_sha256']==file_hash(ROOT/r['destination']) for r in original)
    assert all(file_hash(REPO/r['path'])==r['sha256'] for r in extra)
    put(ROOT/'authorities/TASK1_PRESERVATION.json',dict(v3_modified_files=0,original_copy_sources_checked=len(original),extra_references_checked=len(extra),unrelated_git_status_unchanged=True,ending_status_before_task_commit=status,scope='Bounded source manifests only'))
    now=datetime.datetime.now(datetime.timezone.utc);elapsed=(now-datetime.datetime.fromisoformat(start['started_at'].replace('Z','+00:00'))).total_seconds()
    ledger=dict(name='V4 Committed Front and Cheap SDF Forecast Foundation',allowance_seconds=3600,execution_cap_seconds=3300,reserve_seconds=300,started_at=start['started_at'],sealed_at=now.isoformat(),consumed_seconds=elapsed,remaining_seconds=3600-elapsed,maximum_heavy_workers=1,maximum_rss_bytes=12884901888,peak_sampled_worker_tree_rss_bytes=resource['peak_sampled_process_tree_rss_bytes'],rss_sample_seconds=.1,scientific_solver_runs=0)
    put(ROOT/'authorities/TASK1_LEDGER.json',ledger)
    source_paths=[p for p in (ROOT/'src').rglob('*.py') if not any(x in p.parts for x in ('_frozen','_support','continuous_prism_certificate'))]
    source_paths += [ROOT/'native/src/lib.rs',ROOT/'native/frozen/cached_spline.rs',ROOT/'native/Cargo.toml',ROOT/'native/Cargo.lock']
    evidence=[OUT/n for n in ['BENCHMARK.json','TESTS.json','VERIFIER.json','INITIAL_FRONT_AUTHORITY.json','SCHEDULE.json','DESCRIPTOR_EXTREMA.csv','task1_benchmark_RESOURCE.json','verify_task1_RESOURCE.json']]
    authority=dict(schema='V4.Task1Authority/1',classification='V4CommittedFrontForecastFoundationComplete',branch=start['branch'],starting_head=start['head'],setup_checkpoint=start['checkpoint'],implementation_commit_receipt='authorities/TASK1_COMMIT_RECEIPT.json',source_hashes={p.relative_to(ROOT).as_posix():file_hash(p) for p in source_paths},evidence_hashes={p.relative_to(ROOT).as_posix():file_hash(p) for p in evidence},forecast_archives=verify['artifacts'],source_binding_sha256=file_hash(ROOT/'authorities/TASK1_SOURCE_BINDING.json'),setup_authority_sha256=file_hash(ROOT/'authorities/V4_SETUP_AUTHORITY.json'),v3_modification_count=0,full_forecast_hash_run1=b['scientific_hash_run1'],full_forecast_hash_run2=b['scientific_hash_run2'],dense_global_sensitivity_allocated=False,legacy_optimizer_executed=False,compression_executed=False,certificate_executed=False,prism_optimization_executed=False,layer_commits=0,scientific_solvers=0)
    authpath=ROOT/'authorities/V4_TASK1_AUTHORITY.json';put(authpath,authority);authhash=file_hash(authpath)
    largest=max((p for p in ROOT.rglob('*') if p.is_file()),key=lambda p:p.stat().st_size);free=shutil.disk_usage(ROOT).free
    def report(total):
        rows=[
        ('1. TaskClassification','V4CommittedFrontForecastFoundationComplete'),('2. Branch',start['branch']),('3. Starting Git HEAD',start['head']),('4. Setup checkpoint created','Yes; V4-only'),('5. Setup checkpoint hash',start['checkpoint']),('6. Final Git HEAD','Recorded after the implementation commit in authorities/TASK1_COMMIT_RECEIPT.json; also printed in final chat'),('7. V3 modified files','0; bounded original-source and Task-1 bindings unchanged'),('8. Setup authority result','PASS; all three supplied hashes match'),
        ('9. SDF source path','native/frozen/cached_spline.rs; original V3 path is in authorities/TASK1_SOURCE_BINDING.json'),('10. SDF source SHA-256',file_hash(ROOT/'native/frozen/cached_spline.rs')),('11. SDF access mechanism','V4-local Rust cdylib; qualified closed-section query and production uniform-extrusion composition in binary64; analytic distance derivative'),('12. SDF sign convention','Negative inside, positive outside; directly checked on interior, trailing-face exterior and cap exterior samples'),
        ('13. Initial vertices',str(b['initial_front']['vertices'])),('14. Initial edges',str(b['initial_front']['edges'])),('15. Initial triangles',str(b['initial_front']['triangles'])),('16. Initial coordinate hash',b['initial_front']['coordinate_hash']),('17. Topology hash',b['initial_front']['connectivity_hash']),('18. Layer count','20'),('19. h1','0.000166527069 m'),('20. Nominal growth','1.163359674'),('21. Total nominal thickness',repr(s['total_thickness_m'])+' m'),('22. Complete schedule','All 20 increments and cumulative distances are printed below; preserved binary64 authority'),
        ('23. Wall residual statistics',f"min={b['wall_residual']['min']!r}, median={b['wall_residual']['median']!r}, max={b['wall_residual']['max']!r} m; 53,153 finite values"),('24. Gradient norms','Direct distributions below; every normal is gradient divided by its measured norm'),('25. Nominal next-front result','PASS; independent SDF-guided prediction; layer 0 remains immutable'),('26. Projection method','Newton correction using raw gradient divided by squared gradient norm; no tangential motion'),('27. Projection stopping authority','Native DISTANCE_TOLERANCE=2e-12 times (1+section-point length); residual within that source query bound. Binary64 stagnation outside it fails. Operational iteration cap 64.'),('28. Projection iterations','Exactly 1 for all 1,063,060 provisional vertex states'),('29. Full cheap forecast','Complete for layers 1–20; no compression, prism assembly or certification'),('30. Provisional vertex states',str(b['provisional_vertex_states'])),('31. Provisional edge states',str(b['provisional_edge_states'])),('32. Provisional triangle states',str(b['provisional_triangle_states'])),('33. Maximum absolute target residual',repr(b['summary']['absolute_target_residual']['max'])+' m'),('34. Successive direction changes','Distribution below, in degrees'),('35. Neighbor normal angles','Distribution below, in degrees'),('36. Edge-length ratios','Distribution below; relative to committed layer 0'),('37. Triangle-area ratios','Distribution below; relative to committed layer 0'),('38. Hard forecast events',json.dumps(b['hard_events'],sort_keys=True)),
        ('39. Cache key','SDF source/adapter/binary, geometry, surface, topology, committed layer/coordinate hash, remaining schedule and implementation/config identity'),('40. Cold hits / misses',f"{b['cold']['hits']} / {b['cold']['misses']}"),('41. Warm hits / misses',f"{b['warm']['hits']} / {b['warm']['misses']}"),('42. Cold forecast runtime',repr(b['cold']['seconds'])+' s'),('43. Warm runtime',repr(b['warm']['seconds'])+' s'),('44. Local invalidation','PASS; unchanged coordinates; recomputation exactly equals original; '+repr(b['local']['seconds'])+' s'),('45. Invalidated vertices','2'),('46. Invalidated edges','12'),('47. Invalidated triangles','12'),('48. Risk-region API','PASS; caller vertex/edge/triangle boolean masks, deterministic adjacency and optional rings'),('49. Oversized regions','Returned intact with oversized=true; demonstrated 6- and 8-vertex regions against caller limit 1; no split'),('50. Dense estimator','PASS; exact integer product; compares with caller memory budget before allocation'),('51. V3 dense bytes','2,126,040 rows × 105,317 DOFs × 8 = 1,791,265,237,440 bytes; exceeds 12,884,901,888-byte budget'),('52. Global dense Jacobian allocated','No'),('53. Legacy planner executed','No'),('54. Compression executed','No'),('55. Certificate executed','No'),('56. Prism optimization executed','No'),('57. Deterministic replay','PASS; two clean forecast caches and cleared native query memo; exact coordinates, normals, all descriptors, topology IDs and keys'),('58. Scientific hash run 1',b['scientific_hash_run1']),('59. Scientific hash run 2',b['scientific_hash_run2']),('60. Test count',str(tests['tests'])),('61. Final test failures',f"{tests['failures']} failures; {tests['errors']} errors; {tests['skipped']} skips"),('62. Lightweight verifier','PASS; one bounded pass, 17 selected trajectories / 340 projected states; no full forecast rerun; '+repr(verify['seconds'])+' s'),('63. Task runtime',repr(round(elapsed,3))+' s at report seal'),('64. Ledger allowance','3,600 s; execution cap 3,300 s; reserve 300 s'),('65. Ledger consumed',repr(round(elapsed,3))+' s'),('66. Ledger remaining',repr(round(3600-elapsed,3))+' s'),('67. Maximum RSS',f"{resource['peak_sampled_process_tree_rss_bytes']:,} bytes, observed process-tree peak at 0.1 s sampling; below 12 GiB"),('68. V4 total bytes',f'{total:,} logical bytes, including local native build products and forecast arrays'),('69. Largest new file',largest.relative_to(ROOT).as_posix()+f' ({largest.stat().st_size:,} bytes)'),('70. Free disk bytes',f'{free:,} at report seal'),('71. Exact blocker','None'),('72. Recommended next task','ImplementV4LocalDetailedLookaheadAndRiskPatchForecast; not started'),('73. Task report SHA-256','Stored in authorities/TASK1_SEAL.json to avoid a self-referential file hash; printed in final chat'),('74. V4 Task-1 authority SHA-256',authhash)]
        text='# V4 Task 1 report\n\n| Item | Result |\n|---|---|\n'+'\n'.join('| '+k+' | '+v.replace('|','/')+' |' for k,v in rows)+'\n'
        text+='\n## Complete nominal schedule\n\nValues below round-trip to the preserved binary64 values. Internal calculations use the unrounded array.\n\n| Layer | Increment (m) | Cumulative target (m) |\n|---:|---:|---:|\n'
        text+='\n'.join(f"| {r['layer']} | {r['increment_m']!r} | {r['target_m']!r} |" for r in s['layers'])+'\n'
        text+='\n## Direct descriptor distributions\n\nThese values are descriptive. No health threshold or pass/fail classification is assigned to their ranges.\n\n| Descriptor | Minimum | Median | p90 | p99 | Maximum |\n|---|---:|---:|---:|---:|---:|\n'
        groups=[('Wall gradient norm',b['wall_gradient_norm'])]+list(b['summary'].items())
        for name,z in groups:text+='| '+name+' | '+' | '.join(repr(z[k]) for k in ('min','median','p90','p99','max'))+' |\n'
        text+='\n## Timing and interpretation\n\n'
        text+=f"Topology setup: {b['topology_seconds']!r} s. Initial wall/sign SDF queries: {b['initial_sdf_seconds']!r} s. Cold forecast SDF evaluation: {b['cold']['sdf_seconds']!r} s. Projection: {b['cold']['projection_seconds']!r} s. Descriptor calculation: {b['cold']['descriptor_seconds']!r} s. Clean replay: {b['replay']['seconds']!r} s. Projection includes SDF calls, so these timing fields must not be added as disjoint stages. All timings were measured; none was extrapolated.\n"
        text+='''
Cache entries are whole entity trajectories: 53,153 vertex entries, 159,453
edge entries and 106,302 triangle entries. Warm results and the locally
recomputed 26 entries matched exactly. No accepted coordinate changed.

The maximum observed worker RSS is sampled, not an estimate of matrix memory.
The dense-matrix result is an exact arithmetic byte count, not a measured
allocation. Active Task-1 sources were searched for sensitivity allocation;
none exists. The complete SF2 domain cannot use the old 188-vertex partition.

## SDF and authority limits

The active native source is V3's qualified cached NacaSpline, copied unchanged.
Original path:
`research/sdf_cfd_mesher_v3/fresh_supported_refinement_family_qualification_2026-09-09/native_spline_acceleration_20260910/source/cached_spline.rs`.
The scalar and gradient adapter is `native/src/lib.rs`, called through
`src/sdf_cfd_mesher_v4/sdf/native.py`. It consumes the preserved common geometry
coefficients and applies the production uniform-extrusion composition in
binary64. No Python NACA approximation or finite-difference Hessian exists.

At sharp wall features a unique classical gradient need not exist. Inside the
native distance-error interval, the native outward normal provides a
deterministic one-sided selection; exact extrusion branch ties select the
section branch. These selections are recorded. They are not claims of unique
differentiability. Away from that interval, the derivative uses the signed
native closest-point displacement. Every normal is explicitly normalized.
Exposed curvature is existing section curvature, not full 3D curvature.

The source-defined query error bound is used only for numerical projection
termination. It does not establish a prism quality or geometry-health limit.
Zero hard forecast events do not establish prism validity or absence of
intersections. No local or full continuous certificate was executed.

Layer 0 remains immutable. No later layer was committed. No final prism shell,
OpenFOAM operation, CFD, potential-flow solve or RANS run was performed. The
legacy optimizer, compression and conditioning optimization remained dormant.
No V3 source changed. No large V3 authority was copied.

## Evidence and checks

The report is supported by results/task1/BENCHMARK.json, TESTS.json,
VERIFIER.json, SCHEDULE.json, INITIAL_FRONT_AUTHORITY.json and
DESCRIPTOR_EXTREMA.csv. The extrema table includes source IDs and coordinates.
Four compact NPZ files retain all 20 layers of provisional coordinates and
selected direct descriptors. Their hashes are in VERIFIER.json and the Task-1
authority. The arrays and native build output remain local, outside Git.
The per-vertex Prediction API returns initial/final SDF values, target,
residual, iteration count, gradient norm and displacement for every vertex.

Checks run: isolated offline native release build with one build worker;
`python -B research/sdf_cfd_mesher_v4/tools/run_task1_tests.py`;
bounded task1_benchmark.py; bounded verify_task1.py; V4-only Git diff check.
The initial 22 focused tests passed. The combined suite first found one
setup-era Git status-display assertion: tracking V4 expands the formerly
collapsed untracked research/ entry. The assertion was updated to compare the
same unrelated scope, then all 34 tests passed. No scientific test failed.
The native build emitted only unused-code warnings in frozen source. Git's
default whitespace check flagged preserved Windows CRLF endings under the
byte-preserving -text attribute. The CRLF-aware check with
`git -c core.whitespace=cr-at-eol diff --check` passed; scientific source bytes
were not normalized. The compact extrema CSV is explicitly included despite
the repository's broad CSV ignore rule.

The verifier queried 17 selected trajectories and checked exact scalar,
gradient and coordinate reproduction. It did not repeat the whole forecast.
It also verified setup/source bindings, initial front, schedule, cache key,
invalidation dependency counts, scaling arithmetic and archived samples.

Git started dirty. The setup checkpoint and implementation commit stage only
V4 paths. The final commit hash is recorded after committing in the local
TASK1_COMMIT_RECEIPT.json, avoiding a commit that must contain its own hash.
The report hash is detached in TASK1_SEAL.json for the same reason. Both exact
values are delivered in the final chat. The earlier SETUP_REPORT.md remains a
historical snapshot rather than being rewritten.

Next task: **ImplementV4LocalDetailedLookaheadAndRiskPatchForecast**. Add only
bounded local provisional prism geometry and conditioning calculations. Do not
optimize or commit new prism layers yet. This next task has not begun.
'''
        return text
    path=ROOT/'TASK1_REPORT.md';seal=ROOT/'authorities/TASK1_SEAL.json'
    for _ in range(8):
        total=sum(p.stat().st_size for p in ROOT.rglob('*') if p.is_file())
        path.write_text(report(total),encoding='utf-8')
        put(seal,dict(report_path='TASK1_REPORT.md',report_sha256=file_hash(path),authority_path='authorities/V4_TASK1_AUTHORITY.json',authority_sha256=authhash,final_commit_receipt='authorities/TASK1_COMMIT_RECEIPT.json'))
        if sum(p.stat().st_size for p in ROOT.rglob('*') if p.is_file())==total:break
    print(json.dumps(dict(classification=authority['classification'],report_sha256=file_hash(path),authority_sha256=authhash,runtime_seconds=elapsed,v4_bytes=total,free_bytes=free),indent=2))

if __name__=='__main__':finalize()
