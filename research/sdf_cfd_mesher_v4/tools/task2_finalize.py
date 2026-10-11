"""Seal the bounded Task-2 result; no forecast or geometry evaluation."""
from pathlib import Path
import sys,json,datetime,shutil,subprocess
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(ROOT/'tools'));sys.dont_write_bytecode=True
from sdf_cfd_mesher_v4.common.authority import REPO,file_hash
from task2_preflight import audit,git
from task2_describe import read,failures
from sdf_cfd_mesher_v4.forecast.detailed.scientific import plain
OUT=ROOT/'results/task2'
def write(name,value):(OUT/name).write_bytes((json.dumps(plain(value),indent=2,sort_keys=True,allow_nan=False)+'\n').encode())
def table(headers,rows):
    return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join('---' for _ in headers)+' |']+['| '+' | '.join(str(x).replace('|','/') for x in r)+' |' for r in rows])
def main():
    preserved=audit();start=json.loads((OUT/'PREFLIGHT.json').read_text());hist=json.loads((OUT/'HISTORICAL_SELECTION_SOURCE.json').read_text())
    assert file_hash(REPO/hist['path'])==hist['sha256']
    def unrelated(s):return sorted(r for r in s.splitlines() if 'research/sdf_cfd_mesher_v4/' not in r)
    assert unrelated(git('status','--porcelain'))==unrelated(start['starting_status'])
    preserved.update(v3_modified_files=0,unrelated_status_preserved=True,historical_selection_source_unchanged=True)
    write('PRESERVATION.json',preserved)
    s=read('SUMMARY.json');cohort=read('PATCH_SELECTION.json');pm=read('PATCH_METRICS.json');comp=read('DESCRIPTOR_COMPARISONS.json');continuous=read('CONTINUOUS.json');cache=read('CACHE_LOCALITY.json');scaling=read('PATCH_SCALING.json');det=read('DETERMINISM.json');tests=read('TESTS.json');examples=read('COUNTEREXAMPLES.json');nonfinite=read('NONFINITE_METRICS.json')
    det['continuous_replay']='not run after fail-closed production rejection';det['full_cohort_determinism']='incomplete';write('DETERMINISM.json',det)
    files=list((ROOT/'src/sdf_cfd_mesher_v4/forecast/detailed').glob('*.py'))+list((ROOT/'tools').glob('task2_*.py'))+[ROOT/'tests/test_task2.py']
    source={p.relative_to(ROOT).as_posix():file_hash(p) for p in sorted(files)}
    evidence={p.relative_to(ROOT).as_posix():file_hash(p) for p in sorted(OUT.iterdir()) if p.is_file() and p.name not in ('report.md','AUTHORITY.json','SEAL.json')}
    wall_seconds=(datetime.datetime.now(datetime.timezone.utc)-datetime.datetime.fromisoformat(start['started_utc'].replace('Z','+00:00'))).total_seconds()
    allfiles=[p for p in ROOT.rglob('*') if p.is_file()];newfiles=files+[p for p in OUT.iterdir() if p.is_file()]
    largest=max(newfiles,key=lambda p:p.stat().st_size)
    authority=dict(schema='V4.Task2ScientificAuthority/1',classification=s['classification'],starting_head=start['starting_head'],branch=start['branch'],source_sha256=source,evidence_sha256=evidence,
                   task1_forecast_hash=read('RESTORE.json')['scientific_hash'],direct_scientific_hash_run1=det['direct_hash_run1'],direct_scientific_hash_run2=det['direct_hash_run2'],
                   full_cohort_replay_complete=False,continuous_forbidden_intersections_observed=1,continuous_unfinished_patches=15,v3_modified_files=0,unrelated_status_preserved=True,
                   layers_committed=0,global_dense_allocation=False,optimization_executed=False,compression_executed=False,CFD_executed=False,report='results/task2/report.md',
                   study_seconds=s['total_seconds'],task_wall_seconds_at_seal=wall_seconds,peak_rss=s['peak_rss'],v4_bytes_at_seal=sum(p.stat().st_size for p in allfiles),
                   new_bytes_before_report=sum(p.stat().st_size for p in newfiles),largest_new_file=largest.relative_to(ROOT).as_posix(),largest_new_bytes=largest.stat().st_size,free_disk_bytes=shutil.disk_usage(ROOT).free)
    write('AUTHORITY.json',authority)
    lines=['Decision question','',
      'Can V4 construct bounded local provisional prism stacks, calculate direct quality and reconstruction-conditioning evidence, and relate that evidence to cheap Task-1 descriptors without recreating V3 global coupling?','',
      s['classification'],'',
      'The bounded geometry, metric, cache and descriptive comparison infrastructure works. The scientific study stopped at a qualified production certificate rejection. This is useful bounded partial work, not an architecture-failure classification. It does not qualify the nominal forecast as a prism mesh.','',
      '## A. Authority verification','',
      'All 150 bounded preflight checks passed and passed again at sealing. The accepted polyMesh identity record was checked; the large mesh was neither loaded nor copied. Frozen V3 sources and Task-1 sources remain unchanged.','',
      table(['Authority','SHA-256'],[(r['role']+' / '+Path(r['path']).name,r['actual']) for r in start['rows'] if r['role'] in ('frozen V4 authority','common geometry','SF2 surface','certificate integration authority')]),'',
      'Accepted SF2 polyMesh canonical authority: `3ed2471b2dea7b5ffa26b9a3caf43b2182f7b50b8c434b3117ed7a8e5434804f`. Its record hash is `2409994fc1995766dba047a13370d72b66e50d1116f673d436676e54a923bca2`.','',
      'Restored Task-1 scientific hash: `'+authority['task1_forecast_hash']+'`. Restoration queried saved coordinates to recover omitted diagnostics; it performed zero trajectory projections. One initial execution aborted on a singular production solve; the corrected run restored the same archive again. No cold whole-front forecast was run.','',
      'Coordinate hash: `'+s['initial_coordinate_hash']+'`. Topology hash: `'+s['topology_hash']+'`. The committed front remains layer 0. The preserved 20-layer binary64 schedule and nominal total thickness 0.020000000131704564 m are unchanged.','',
      '## B. Implementation','',
      'All new paths are under V4. No existing Task-1 implementation or copied scientific source was edited. See IMPLEMENTATION.md for the exact source formulas, interfaces and boundary scope.','',
      table(['File','Important symbols / role'],[
       ('src/sdf_cfd_mesher_v4/forecast/detailed/archive.py','restore: reproduce the complete Task-1 cache identity'),
       ('.../patch.py','Patch, column_patch, require_closure, prisms, patch_key'),
       ('.../metrics.py','cell_metrics, measure: frozen primitive adapters'),
       ('.../cache.py','DetailedCache, fork_changed_forecast'),
       ('.../continuous.py','SideIndex: full-input broad phase, bounded exact pair work'),
       ('.../scientific.py','canonical, scientific_hash; explicit nonfinite encoding'),
       ('.../__init__.py','package declaration'),
       ('tests/test_task2.py','15 focused tests'),
       ('tools/task2_preflight.py','bounded source and authority verification'),
       ('tools/task2_study.py','cohort, measurements, replay, locality, scaling, certificate'),
       ('tools/task2_describe.py','row-level rank relationships and counterexamples'),
       ('tools/task2_tests.py','retained setup/Task-1 and new tests'),
       ('tools/task2_verify.py','stored scientific hash and one rejected-pair verification'),
       ('tools/task2_finalize.py','bounded preservation and evidence sealing')]),'',
      'Each core consists of one anchor triangle plus one topological ring: four columns. A one-face-neighbor halo closes internal side metrics. All 20 layers close internal horizontal dependencies. Sigma and G_bound use all five actual cell faces. No reconstruction stencil is truncated. Source IDs, canonical edge IDs, source triangle/column IDs and layer IDs remain stable.','',
      'The provisional layer-20 exterior cap has no core-mesh neighbor. Its internal-face measures are unavailable for all 68 core columns. Wall faces are boundary faces, not artificial internal faces. Twenty singular production solves are also unavailable. No alternate reconstruction operator was used.','',
      '## C. Patch cohort','',
      'All patches cover layers 1–20. Anchor layers below describe sampling reasons, not a restriction on measurement. Coordinates are normalized unit-chord x and physical span y in meters. Controls retain their historical label only; they are not V4 physical truth. Overlapping anchors were deduplicated and additional reasons remain in PATCH_SELECTION.json.','',
      table(['Patch','Anchor column','Selection reason','Anchor layer','Wall x / y','Core vertices / columns','Halo vertices / columns'],[
       (r['patch_id'],r['anchor'],r['selection']['reason'],r['selection']['layer'],str(r['wall_centroid'][0])+' / '+str(r['wall_centroid'][1]),str(len(r['core_vertices']))+' / '+str(len(r['core_columns'])),str(len(r['vertices'])-len(r['core_vertices']))+' / '+str(len(r['columns'])-len(r['core_columns']))) for r in cohort]),'',
      'The cohort contains forward, middle and aft chord locations, both span ends and interior span. Early, middle and late layers are all measured. Historical locations are used only in the study runner; no production risk mask exists.','',
      '## D. Direct prism geometry','',
      f"Measured {s['measured_cells']} distinct core cells and {s['measured_unique_faces']} distinct existing shell internal faces. The 68 core columns are a small selected cohort, not the complete SF2 shell.",'',
      table(['Metric','Observed minimum','Median','Observed finite maximum','Finite / total'],[(k,d['min'],d['median'],d['max'],str(d['finite'])+' / '+str(d['count'])) for k,d in s['distributions'].items()]),'',
      'Twenty aspect values are positive infinity. Twenty weighted gains are positive infinity because sigma is exactly zero. The finite-maximum table is not a substitute for those failures. Nine faces have undefined weight, skew and non-orthogonality because degenerate neighboring geometry gives undefined centers. These are recorded explicitly; NaN is never counted as a pass.','',
      table(['Existing hard gate','Violations in measured scope'],list(s['hard_gate_violations'].items())+[
       ('forbidden continuous side intersection','at least 1 observed; count incomplete'),('unresolved continuous scope','15 patches unfinished, including the rejected patch'),('undefined internal face metrics','9 faces')]),'',
      'Counts overlap. Face counts and cell counts must not be added. Positive volume requires both the frozen signed wedge volume and Foundation-style volume to be positive. The minimum signed volume is 0.0; the tiny positive Foundation-style volume is not accepted in its place.','',
      table(['Extreme','Value','Patch / column / layer'],[(k,r[k],r['patch_id']+' / '+str(r['column'])+' / '+str(r['layer'])) for k,r in s['extrema'].items()]),'',
      'The maximum finite skewness is the unchanged source calculation with its rootVSmall arithmetic at nearly degenerate geometry. It is not a new metric. Nominal compression factors remain 1.0 because no compression was applied.','',
      '## E. Reconstruction conditioning','',
      'A = sqrt(W) N uses face-area fractions. G_weighted = 1/sigma_min. G_bound uses the preserved constrained production operator and the layer-1 wall constraint. No sigma floor or gain ceiling is applied.','',
      table(['Patch','Minimum sigma','Column / layer','Maximum available G_bound','Column / layer','Unavailable gains'],[
       (r['patch_id'],r['sigma_min']['sigma'],str(r['sigma_min']['column'])+' / '+str(r['sigma_min']['layer']),r['G_bound_max']['G_bound'] if r['G_bound_max'] else 'unavailable',str(r['G_bound_max']['column'])+' / '+str(r['G_bound_max']['layer']) if r['G_bound_max'] else 'unavailable',r['G_bound_unavailable']) for r in pm]),'',
      'The minimum positive sigma is 0.06459903971926352 at column 58145, layer 1. Its G_bound is 92.70931982223773, the largest available value. Column 78678 has sigma 0.0 and zero signed volume through all 20 layers; its production solve is singular. No finite gain is claimed for those cells. The strongest conditioning extreme in this sample is not confined to layer 20.','',
      '## F. Continuous geometry','',
      'Exact work used the unchanged qualified production pair kernel. Broad-phase closure scanned all 159,453 source edges through all 20 saved fronts. It compared core sides against all overlapping candidate sides, including different layers. Detailed geometry remained bounded. No fixture qualification ran.','',
      table(['Patch','Candidate source edges','Pairs completed / candidates','Result','Seconds'],[(r['patch_id'],len(r['candidate_edges']),str(r['completed_pairs'])+' / '+str(r['candidate_pairs']),r['classification'],r['seconds']) for r in continuous]),'',
      'The first two patches complete the core-side-pair scope only. They do not certify side-cap intersections or full cell overlap. There is no global zero-intersection claim.','',
      'The third patch rejected side `(layer 1, edge 1)` against `(layer 10, edge 729)`. Method: `Rank2ExactMixedPlane`. Classification: `CertifiedForbiddenIntersection`. Exact witness: `(1, -1/4, 10823385/8589934592)`. Authorized contact is empty. The exact residual is 0. The proof and intersection segment are preserved in CONTINUOUS.json.','',
      'Edge 1 joins wall vertices 0 and 22489; its adjacent columns are 29349 and 38902. Edge 729 joins vertices 123 and 38991; its adjacent columns are 78681 and 78682. Vertex 0 is the negative-span trailing-upper endpoint; vertex 123 is the trailing-lower endpoint.','',
      'The production certificate is fail closed. It stopped after 108 of 14,157 candidate pairs in patch_002. The other 14 patches were not certified. The recorded forbidden-intersection count is a lower bound. This triggered the Task-2 stop boundary; geometry was not moved to obtain a pass.','',
      '## G. Descriptor relationships','',
      'Measured facts: DESCRIPTOR_COMPARISONS.json contains ranges, finite sample counts, signed rank correlations and top/bottom-five coincidences for each descriptor/direct-metric pair. ENTITY_RANKS.json preserves row-level rank ordering. Direct rows and faces preserve IDs and coordinates. LAYER_STRATIFIED_ASSOCIATIONS.json reports each layer separately.','',
      table(['Cheap descriptor','rho with sigma','rho with G_bound','rho with weight','rho with skew'],[
       (d,*[next(r['spearman'] for r in comp if r['descriptor']==d and r['metric']==m) for m in ('sigma','G_bound','weight','skewness')]) for d in [r['descriptor'] for r in examples]]),'',
      'Sample counts are 1,360 for sigma, 1,340 for G_bound, and 1,351 for cell-aggregated weight/skew. Missing values are excluded pairwise and recorded. Rank ties use average ranks; top-five selections break ties by stable cell order. These are descriptive rank tails, not scientific acceptance thresholds.','',
      'Descriptive associations: edge and area ratios relate more to interpolation weight than to sigma or gain. Neighbor normal angle and normal convergence show similar weight associations and only modest pooled conditioning associations. Curvature is positively associated with sigma in this cohort; a simple high-curvature-is-bad rule is not supported. Direction change has weak pooled conditioning association. Residual and step length show larger pooled correlations, but they contain arithmetic and layer effects and are not validated risk indicators.','',
      'Measured redundancy: edge ratio versus triangle-area ratio rho = 0.9979209436409764. Neighbor angle versus normal convergence rho = 0.9948544839152946. These descriptors are nearly redundant in this selected cohort. Their future utility in other geometry is unproved.','',
      'Hypotheses, not conclusions: normal direction changes at the tip/trailing-edge and the deterministic one-sided SDF normal rule may explain some nominal failures. The current study proves the failures and their locations, not that causal mechanism. Small SDF residuals only describe iso-distance projection; they do not certify prism geometry.','',
      'The cohort is not a random population sample. Twenty layers from each column are dependent observations. No significance test, causal claim, combined score, production threshold or production risk mask is supplied.','',
      '## H. Counterexamples','',
      table(['Descriptor','Low-value direct failure: column/layer, value, rank','High-value measured numeric pass: column/layer, value, rank'],[
       (r['descriptor'],str(r['lowest_descriptor_among_direct_failures']['row']['column'])+'/'+str(r['lowest_descriptor_among_direct_failures']['row']['layer'])+', '+str(r['lowest_descriptor_among_direct_failures']['row'][r['descriptor']])+', '+str(r['lowest_descriptor_among_direct_failures']['descriptor_rank'])+'/'+str(s['measured_cells']),str(r['highest_descriptor_among_measured_gate_passes']['row']['column'])+'/'+str(r['highest_descriptor_among_measured_gate_passes']['row']['layer'])+', '+str(r['highest_descriptor_among_measured_gate_passes']['row'][r['descriptor']])+', '+str(r['highest_descriptor_among_measured_gate_passes']['descriptor_rank'])+'/'+str(s['measured_cells'])) for r in examples]),'',
      'Column 19138, layer 18 has edge ratio 1.0, neighbor normal angle 0 degrees and normal convergence 0, but interpolation weight 0.04819425405600387 violates 0.055. Column 38589, layer 1 has zero successive direction change but fails Jacobian, weight and symmetric-volume gates. These are observed false-negative possibilities for rules based only on large descriptor values.','',
      'Column 105527, layer 19 has edge ratio 32.56734047693956 and triangle-area ratio 87.11093574931249, yet all measured numeric gates pass: weight 0.05536243401347567, determinant 0.3116887255050571, sigma 0.15200840978581337. This is not a full geometry acceptance claim; its certificate scope is unfinished. The maximum direction-change and residual anchors also include measured numeric passes.','',
      'In total, 1,280 sampled cells pass the measured internal numeric scope and 80 have a violation or undefined measure. None is called a fully accepted prism cell without continuous and exterior-neighbor closure.','',
      '## I. Cache locality','',
      'Test-only modification: vertex 60, x from 0.0009765625 to 0.0009765625000000002 m; delta = 2.168404344971009e-19 m, one binary64 ULP toward positive infinity. The accepted wall remains unchanged.','',
      'Invalidated cheap trajectories/descriptors: 1 vertex, 5 edges, 5 triangles. Recomputed entries: 11. Unrelated hits: 318,897. Recompute runtime: '+str(cache['seconds'])+' s. Detailed cold cache: 17 misses. Identical warm request: 17 hits, zero new misses. Changed test state: one patch misses; 16 unrelated patches are exact hits. Cumulative detailed counters after original-reference checks: 50 hits and 18 misses.','',
      'The changed patch includes 4 core columns, 10 columns with halo, 80 measured prisms and 200 local prisms with halo. The conservative patch cache recomputes the affected patch, not a full connected component. Every unrelated cheap scientific array was compared exactly. Locality stage, including cache-copy verification: '+str(s['stages']['cache_locality']['seconds'])+' s; sampled peak RSS '+str(s['stages']['cache_locality']['peak_rss'])+' bytes.','',
      '## J. Scaling','',
      table(['Rings','Core vertices','Halo vertices','Columns with halo','Prisms with halo','Measured faces','Reconstruction rows','Sensitivity DOFs','Seconds','Peak RSS bytes','Output bytes'],[(r['rings'],r['core_vertices'],r['halo_vertices'],r['columns'],r['prisms'],r['faces'],r['reconstruction_rows'],r['sensitivity_dofs'],r['seconds'],r['peak_rss'],r['persistent_bytes']) for r in scaling['local']]),'',
      'These three timings include up to 50 ms of sampling-thread shutdown wait and JSON persistence. They are direct stage wall times, not isolated kernel timings. They resolve bounded cost but do not resolve a useful runtime slope at this scale. RSS includes the resident restored whole-front cheap cache. Local entity counts and output bytes grow with patch size; no formal asymptotic claim follows from three measurements.','',
      'The rejected V3 allocation remains 2,126,040 rows × 105,317 DOFs × 8 bytes = 1,791,265,237,440 bytes, versus 12,884,901,888 allowed bytes. The existing guard rejects it. Task 2 creates no sensitivity matrix or sensitivity DOFs. Detailed reconstruction arrays contain five face rows and three vector columns per local cell.','',
      '## K. Determinism','',
      'Direct cohort run 1: `'+det['direct_hash_run1']+'`.','',
      'Direct cohort run 2: `'+det['direct_hash_run2']+'`.','',
      'Exact direct replay passed. The payload includes selection, stable cell/face IDs, all measured direct rows, cheap descriptor rows, comparisons and ranks. Timings and RSS are separate. Full scientific-cohort replay, including completed certificate streams, is incomplete because certification stopped at the forbidden intersection. It is not reported as a full Task-2 determinism pass.','',
      '## L. Tests and runtime','',
      f"{tests['tests']} tests passed; {tests['failures']} failures, {tests['errors']} errors, {tests['skipped']} skips. All 34 setup/Task-1 tests were retained unchanged; 15 focused Task-2 tests were added. Command: `python research/sdf_cfd_mesher_v4/tools/task2_tests.py`. Tests cover local construction, stable IDs, stencil closure/rejection, determinism, cache identity/invalidation, unrelated reuse, dense allocation rejection, singular-cell unavailability and serialization. The bounded verifier reproduced the stored direct scientific hash and the exact one-pair forbidden-intersection result. It did not resume cohort certification.",'',
      'The initial study execution failed on a singular production normal-equation matrix. The adapter now records that mathematical unavailability while preserving sigma and primitive geometry. INITIAL_SINGULAR_SOLVE_STOP.json and INITIAL_ATTEMPT_TIMINGS.json preserve the failed attempt. No alternate gain definition was introduced. Frozen formulas also emitted divide-by-zero/invalid-value warnings for degenerate cells; those values are retained, not suppressed into passes.','',
      table(['Stage','Seconds','Sampled peak RSS bytes'],[(k,v['seconds'],v['peak_rss']) for k,v in s['stages'].items()]),'',
      'Final study execution: '+str(s['total_seconds'])+' s. Initial aborted restoration/metric attempt: 57.83822240000169 s (sum of recorded stage times). Task wall time at seal: '+str(wall_seconds)+' s. Peak sampled worker RSS: '+str(s['peak_rss'])+' bytes, below 12 GiB. RSS sampling interval: 50 ms. One heavy worker. No CFD, OpenFOAM, optimizer, compression solver, final shell generation or layer commit ran.','',
      '## M. Scientific conclusion','',
      'Yes for bounded extraction and direct evidence: the local implementation measures actual provisional prisms without restoring V3 global sensitivity coupling. No for completing the requested closed scientific cohort: exact continuous geometry fails and the remaining scope is unfinished.','',
      'Edge/area expansion and neighbor-normal descriptors have some evidence for ranking interpolation weight in this cohort, but weak evidence for ranking the complete sigma/G_bound tail. No individual cheap descriptor reliably identifies all observed failures. Exact projection residual and small trajectory direction change can coexist with invalid prisms. Large area/edge ratios can coexist with passing measured numeric gates.','',
      'This study does not establish a production risk threshold, risk mask, causal model, full-shell quality distribution, complete intersection count, optimal patch size, correct sharp-feature construction policy, or suitability for CFD. The architecture did not hit global dependency closure or a dense allocation. The scientific blocker is the fixed nominal geometry and the incomplete certificate cohort.','',
      '## N. Recommended next architecture step','',
      'Retain the bounded local measurement and cache architecture. Before Task 3, diagnose the exact negative-span trailing-edge cross-layer intersection and zero-volume column using the frozen normal/ownership authorities. Resolve the intended sharp-feature/cap proposal contract, then complete bounded continuous evidence and replay. The current evidence does not support proceeding directly to next-front relaxation. Task 3 was not started.','',
      '## Git, storage and seal','',
      'Branch: `'+start['branch']+'`. Starting HEAD: `'+start['starting_head']+'`. Setup checkpoint remains `a82ca5fbe5a9bf0c7379d397512f4c2e45459773`. V3 files modified by this task: **0**. All 150 bounded source/authority checks still pass. Unrelated Git status is unchanged.','',
      'Only new Task-2 source, tests, tools and results under V4 are staged for one bounded-partial commit. The final commit is identified by Git metadata and the final chat; it is not embedded in its own sealed report. Command to resolve it: `git log -1 --format=%H -- research/sdf_cfd_mesher_v4/results/task2/report.md`.','',
      'V4 bytes before final report/seal: '+str(authority['v4_bytes_at_seal'])+'. New Task-2 bytes before report: '+str(authority['new_bytes_before_report'])+'. Largest new file: `'+authority['largest_new_file']+'`, '+str(authority['largest_new_bytes'])+' bytes. Free disk: '+str(authority['free_disk_bytes'])+' bytes. No new file exceeds 100 MB. No large mesh authority was copied.','',
      'Machine-readable authority: AUTHORITY.json. The detached SEAL.json stores the final report and authority SHA-256 values, avoiding self-referential hashes. Source paths and hashes, evidence paths and hashes, and limitations are recorded in the authority.','']
    (OUT/'report.md').write_bytes('\n'.join(lines).encode())
    seal=dict(report_sha256=file_hash(OUT/'report.md'),authority_sha256=file_hash(OUT/'AUTHORITY.json'))
    write('SEAL.json',seal);print(json.dumps(seal))
if __name__=='__main__':main()
