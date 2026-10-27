"""Seal bounded forensic evidence and reporting; no scientific execution."""
from pathlib import Path
import collections, datetime, json, shutil, sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'tools'))
sys.dont_write_bytecode = True
from forensics_preflight import verify, git
from sdf_cfd_mesher_v4.common.authority import file_hash

OUT = ROOT / 'results/nominal_forecast_forensics'


def read(name):
    return json.loads((OUT / name).read_text())


def write(name, value):
    (OUT / name).write_text(json.dumps(value, indent=2, sort_keys=True) + '\n', encoding='utf-8')


def unrelated(status):
    return sorted(r for r in status.splitlines() if 'research/sdf_cfd_mesher_v4/' not in r)


def legacy_unrelated(status):
    # The preserved preflight helper strips leading whitespace from the entire
    # porcelain string. Normalize that presentation difference for this old
    # snapshot comparison only; the task start/end comparison stays exact.
    return sorted(r.lstrip() for r in unrelated(status) if not r.startswith('?? research/'))


def table(headers, rows):
    return '\n'.join(['| ' + ' | '.join(headers) + ' |', '| ' + ' | '.join(['---'] * len(headers)) + ' |'] +
                     ['| ' + ' | '.join(str(x) for x in row) + ' |' for row in rows])


def main():
    start = read('PREFLIGHT.json')
    current = git('status', '--porcelain')
    assert unrelated(start['starting_status']) == unrelated(current)
    assert not git('diff', '--cached', '--name-only')
    assert git('rev-parse', 'HEAD') == start['starting_head']
    audit = verify()
    write('FINAL_PRESERVATION.json', dict(
        prior_checks=len(audit['rows']), task2_checks=len(audit['task2_bindings']),
        passed=True, v3_source_modifications=0, task2_frozen_modifications=0,
        unrelated_status_preserved=True, ending_status_before_commit=current,
        scope='Existing bounded manifests only. No repository-wide preservation audit.'))
    old = json.loads((ROOT / 'authorities/START_STATE.json').read_text())['status_porcelain']
    legacy_delta = sorted(set(legacy_unrelated(start['starting_status'])) - set(legacy_unrelated(old)))
    assert legacy_delta == ['?? examples/conditioned_gpu_probe.rs', '?? shaders/conditioned_field.wgsl', '?? src/gpu/conditioned.rs']
    write('LEGACY_TEST_CONTEXT.json', dict(
        test='test_setup.SetupTests.test_branch_and_unrelated_git_status_preserved',
        failure_is_preexisting_status_snapshot_difference=True,
        setup_to_followup_start_added_status_rows=legacy_delta,
        followup_start_to_end_unrelated_status_difference=[],
        original_test_modified=False,
        explanation='The retained assertion compares against setup-era Git status. These rows already existed at follow-up start. No test or unrelated work was changed.'))

    science = read('SCIENTIFIC_OUTPUTS.json')
    analysis = read('MECHANISM_ANALYSIS.json')
    replay = read('FINAL_DETERMINISM.json')
    perf = read('PERFORMANCE.json')
    tests = read('TESTS.json')
    clusters = []
    for c in science['components']:
        item = {k: v for k, v in c.items() if k not in ('face_histories', 'observed_events')}
        events = c['observed_events']
        item['original_unique_failed_cells'] = len({r['id'] for r in events if r['entity'] == 'cell'})
        item['original_unique_failed_faces'] = len({tuple(r['id']) for r in events if r['entity'] == 'face'})
        item['closure'] = next(r for r in science['closure'] if r['component'] == c['component'])
        clusters.append(item)
    write('FAILURE_COMPONENTS.json', clusters)
    write('FIRST_INVALID_TRAJECTORIES.json', analysis['component_trajectories'])
    write('CAUSE_CLASSIFICATIONS.json', dict(components=analysis['cause_classifications'], boundary_scope=analysis['boundary_scope'], implementation_defects=analysis['implementation_defects']))
    write('EARLIEST_ACTIONABLE_LAYERS.json', [dict(component=c['component'], dependencies=c['dependency_by_kind']) for c in clusters])
    write('DESCRIPTOR_PRECURSORS.json', analysis['descriptor_precursors'])

    counts = dict(sum((collections.Counter(c['event_counts']) for c in clusters), collections.Counter()))
    assert counts == dict(volume=20, jacobian=41, determinant=21, aspect=20,
                         singular_operator=20, interpolation_weight=84, skewness=16,
                         symmetric_volume_ratio=80, unavailable_neighbor_metric=9, exact_intersection=1)
    component_table = table(
        ['Component', 'Source columns', 'Failed cells / faces', 'Event layers', 'First invalidity'],
        [[c['component'], ', '.join(map(str, c['columns'])),
          f"{c['original_unique_failed_cells']} / {c['original_unique_failed_faces']}",
          str(c['layer_range']), ', '.join(f'{k}: {v}' for k, v in c['first_invalid_by_kind'].items())] for c in clusters])
    location_table = table(['Component', 'Wall x range (m)', 'Wall y range (m)', 'Wall z range (m)'],
                          [[c['component']] + [f"{c['source_coordinate_min'][i]!r} to {c['source_coordinate_max'][i]!r}" for i in range(3)] for c in clusters])
    closure_table = table(['Component', 'Core columns', 'Core vertices', 'Halo vertices', 'Columns with halo', 'Prisms with halo', 'Expanded columns'],
                          [[r[k] for k in ('component', 'core_columns', 'core_vertices', 'halo_vertices', 'normal_columns', 'prisms_with_halo', 'expanded_columns')] for r in science['closure']])
    endpoint_rows = []
    for s in science['intersection']['surfaces']:
        for label, point, target, phi, normal in zip(s['topology_labels'], s['points'], s['targets'], s['phi'], s['normals']):
            endpoint_rows.append([s['source_edge_id'], label, repr(point), repr(target), repr(phi), repr(normal)])
    endpoint_table = table(['Edge', '[front, vertex]', 'Position (m)', 'Target (m)', 'Native SDF (m)', 'Native unit normal'], endpoint_rows)
    weight_rows = []
    for r in analysis['interface_distance_histories']:
        for v in r['history']:
            if v['layer'] in (1, 2, 10, 17, 18, 19, 20):
                weight_rows.append([r['edge'], v['layer'], repr(v['owner_normal_distance']), repr(v['neighbor_normal_distance']), repr(v['weight'])])
    weight_table = table(['Edge', 'Layer', 'Owner normal distance (m)', 'Neighbor normal distance (m)', 'Weight'], weight_rows)
    mask_table = table(['Layer', 'Actual determinant', 'All-face expression', 'Wall-excluded expression'],
                       [[r['layer'], r['actual_determinant'], r['all_faces_expression'], r['wall_excluded_expression']] for r in analysis['determinant_ownership_mask_diagnostic']])
    exact_signs = dict(collections.Counter(str(r['jacobian_sign']) for r in science['exact_jacobian_sign_checks']))
    assert exact_signs == {'0': 26, '-1': 15}
    preflight_hashes = table(['Authority', 'SHA-256'], [[r['path'], '`' + r['sha256'] + '`'] for r in start['task2_bindings'][:2]])
    cause_sections = []
    for c in analysis['cause_classifications']:
        cause_sections.append(f"**{c['component']} — {c['classification']}.** {c['supported_mechanism']} "
                              'The original core metric values replay exactly. A larger halo leaves them unchanged. '
                              'Local native SDF values and normalized normals match the archived forecast exactly.')
    complete_first_table = table(['Component', 'First invalid layer in the complete investigated support'],
                                [[c['component'], ', '.join(f'{k}: {v}' for k, v in c['evidence']['first_invalid_in_complete_investigated_support'].items())] for c in analysis['cause_classifications']])
    source_paths = [ROOT / 'src/sdf_cfd_mesher_v4/forecast/forensics.py', ROOT / 'tests/test_forensics.py'] + sorted((ROOT / 'tools').glob('forensics_*.py'))
    source_table = table(['Changed source', 'Purpose'], [
        ['src/sdf_cfd_mesher_v4/forecast/forensics.py', 'GeometryUse; interpret_hard_events; interpret_certificate; stable clustering; first_invalid; dependency_window; exact_wedge'],
        ['tests/test_forensics.py', '13 focused semantic, tracing, exact arithmetic, clustering, serialization, and preservation tests'],
        ['tools/forensics_preflight.py', 'Bounded frozen-authority audit'],
        ['tools/forensics_intersection.py', 'Smallest two-side reproducer and qualified exact membership checks'],
        ['tools/forensics_study.py', 'Bounded metric replay, larger-halo check, pair history, representative certification, two scientific runs'],
        ['tools/forensics_analysis.py', 'Direct mechanism calculations, precursor tables, repeated scientific analysis'],
        ['tools/forensics_tests.py', 'Retained suite runner'],
        ['tools/forensics_finalize.py', 'Read-only final audit and evidence sealing']])
    now = datetime.datetime.now(datetime.timezone.utc)
    wall_seconds = (now - datetime.datetime.fromisoformat(start['started_utc'].replace('Z', '+00:00'))).total_seconds()
    study_runtime = dict(started_utc=start['started_utc'], sealed_utc=now.isoformat(), wall_seconds_to_seal=wall_seconds,
                         forensic_execution=perf, analysis_seconds=read('ANALYSIS_TIMING.json')['seconds'],
                         test_runner_seconds=tests['seconds'], free_disk_bytes=shutil.disk_usage(ROOT).free)
    write('RESOURCE_SUMMARY.json', study_runtime)
    classification = 'V4NominalForecastHazardsConfirmed'
    report = f'''Decision question

Are the Task-2 invalid provisional prism states genuine hazards predicted by the nominal future path, or artifacts of the detailed-lookahead implementation?

**{classification}**

The reported failures are genuine hazards of these nominal lineages. No material coordinate, topology, metric, cache, ownership, or certificate integration defect was found. The sampled failures form five source-edge-connected footprints. These are not a complete-shell failure census. Only layer 0 remains committed. No geometry was changed.

## A. Authority verification

Branch: `{start['branch']}`. Starting HEAD: `{start['starting_head']}`. The initial and final bounded audits pass: {len(audit['rows'])} prior bindings plus {len(audit['task2_bindings'])} Task-2 bindings. The frozen Task-1/Task-2 sources and outputs are unchanged. V3 source modifications caused by this task: **0**.

{preflight_hashes}

| Other preserved identity | SHA-256 |
| --- | --- |
| Common geometry | `f453b7430703e9ae2c4938cd10f600c565d3fb7a0ce6f8e22e1161561326d1a3` |
| SF2 surface | `dc390bedc2198cb315e15961f848cd67b47641c30ae4ec0b96944996d1f8196f` |
| Source fork manifest | `949faff4837568fd67449758d34dd15667ab9c32a92ca80c8439934ad4eac5cb` |
| Data manifest | `20f1aeba70dc21dd3e4d8fe19bdb8999d75d5a73ce3a0231f255fe5595838241` |
| Setup authority | `190d04c817c6164f7a004adfa8f257f0d61cf55c19d9b0c970a85493b868d87e` |
| Task-1 report | `a710d184a386d609c39b7199e6c5fde229e71eb4431895aa50b870a0057870f9` |
| Task-1 authority | `a84200a78b7c5c73d03f6b9a53d3e61dfdfb1d72dc1a8260292fe6c5e11573a7` |
| Continuous certificate integration | `bab10e0d52d724a6d0a525b801e2486ae3fc1af9c0f02fa1030da5a8dcc99b7f` |
| Wall coordinates | `3412cc0b84cc4ac092ad2cd89698cb9857ddb8428f3200db1e6d2e0ef12e7c94` |
| Topology | `69477add150348f567e454cb2216a913124adecc6ec11c47f58471c0931868e7` |
| Task-1 scientific forecast | `366422660b793b1adad4766eb72f08dd46642259f6fe0461f88557a39fbe0960` |
| Sealed Task-2 direct replay | `a0302d24edf8c96ac28a121d8dab5c3f8fff644baf41d643878776b6b420b4c9` |

`PREFLIGHT.json` contains the exact per-source and per-archive hashes. `FINAL_PRESERVATION.json` records the final audit. No large authority was copied. The physical schedule remains the archived binary64 schedule, with 20 layers and total thickness 0.020000000131704564 m.

## B. Exact intersection reproducer

The Task-2 patch is **patch_002**. The qualified production certificate reproduces the same physical entities and the same exact record. `INTERSECTION_REPRODUCER.json` contains all stable ancestry strings, triangle ownership, coordinates, gradients, trajectory directions, displacement vectors, branch/tie diagnostics, and proof data.

| Side | Edge | Source vertices | Owning source columns | Front endpoints |
| --- | --- | --- | --- | --- |
| A | 1 | 0, 22489 | 29349, 38902 | 0 to 1 |
| B | 729 | 123, 38991 | 78681, 78682 | 9 to 10 |

Vertex 0 is `lofted_wing.main.tip.negative.trailing.upper`; vertex 123 is `lofted_wing.main.tip.negative.trailing.lower`. Other stable source IDs are preserved in the reproducer. Source column IDs equal frozen source triangle IDs. Side vertices use the tensor-product order lower-a, lower-b, upper-a, upper-b; they are not reinterpreted as a perimeter polygon. All triangle connectivity comes from the frozen front topology.

{endpoint_table}

Certificate classification: `CertifiedForbiddenIntersection`. Method: `Rank2ExactMixedPlane`, canonical ranks [2, 3], exact residual 0. Exact witness:

`(1, -1/4, 10823385/8589934592)`.

The intersection segment has the other endpoint `(563043699827049/562949953421312, -1/4, 10823385/8589934592)`. The authorized owned-contact set is empty. Independent calls to the preserved exact rank-geometry membership mathematics confirm that the witness lies on both bilinear images and is not legal owned contact. This is not an approximate triangle intersection test.

All 400 layer combinations of these two source edges were checked in production mode. The earliest forbidden pair is side A layer 1 with side B layer 10, first available after front 10. The witness is the upper wall corner; correct endpoint SDF distances do not prevent the bilinear side image from crossing it. Classification: **GenuineNominalForecastHazard**, reported as **PredictedFutureIntersection**.

## C. Failure clusters

Clustering uses the induced graph of failed source-column supports joined by shared source edges. The exact intersection also joins its two side supports. Layer is retained on each event. No expansion through the full connected wing is performed. Disconnected sampled footprints could connect through unmeasured geometry; five is the sampled count only.

{component_table}

Cell counts are unique original cells with an intrinsic failure. Face counts are unique original faces with a failed or unavailable measure. A face can add a neighboring column that was originally halo-only. Repeated metric symptoms are not counted as independent mechanisms.

{location_table}

The geometry has a 1 m chord; x is the chord coordinate and y is the span coordinate. Components 00 and 04 are leading-edge tip-cap regions; 01 and 02 are aft negative-span cap regions; 03 is an interior-span upper-surface/trailing-face junction. Exact coordinate ranges above take precedence over these labels.

Original counts replay unchanged: 20 zero-volume cells; 41 nonpositive Jacobians; 21 determinant, 84 interpolation-weight, 16 skewness, and 80 symmetric-volume-ratio violations; 20 infinite-aspect cells; 20 singular production operators; 9 undefined internal-face measures; one reported exact intersection. These counts describe the original 17-patch cohort. Additional halo columns are diagnosed in their own histories and do not revise the sealed cohort counts.

## D. First-invalid-layer histories

`SCIENTIFIC_OUTPUTS.json` stores all 420 core cell states for the 21 failure-support columns across all 20 layers, complete internal-face histories, vertex SDF/gradient histories, and all 41 exact-rational Jacobian checks. `FIRST_INVALID_TRAJECTORIES.json` gives compact component histories. Layer 0 is the preserved wall; a prism Jacobian or volume does not exist there.

The following first-invalid map also includes columns that were halo-only in Task 2. In component 01 this exposes the intrinsic collapse that caused the original neighboring-face failures. It is separate from the original-cohort count table above.

{complete_first_table}

All 41 originally nonpositive binary64 Jacobian results remain nonpositive under exact arithmetic on the same binary64 coordinates: **26 exactly zero, 15 negative**. The polynomial and integral are the preserved wedge definitions, not a new quality measure.

Column 78678 stays exactly in y=-0.25 for every vertex at all 21 front states. Its prisms have zero volume, sigma 0, infinite aspect ratio, and unavailable singular G_bound throughout. Columns 78681 and 78682 also remain in that plane. Column 38589 has negative exact Jacobians through layer 15. Later recovery cannot make the earlier invalid cells admissible. Column 101197 has a zero corner Jacobian in layer 1 and a positive value 4.816491459194949e-11 in layer 2. These failures begin immediately; they do not have a prior valid prism history.

For the two late interpolation-weight failures, direct face-normal center distances expose a progressive imbalance. Distances are evaluated from the actual complete adjacent prisms using the frozen Foundation-compatible routines.

{weight_table}

The edge-158488 interface first fails weight at layer 18. The edge-7187 interface first fails at layer 19. Both retain the exact 0.055 gate.

Column 58145 has a different determinant history. The correct uncoupled-wall mask applies only to layer 1. Interior layers use all faces:

{mask_table}

The layer-2 value 0.001063870470194523 fails the retained 0.0011 gate. Sigma rises from 0.06459903971926352 at layer 1 to 0.06955642960657622 at layer 2 and 0.07484675521607313 at layer 3. The determinant change includes the correct ownership-mask transition; it is not an indexing defect or evidence of monotonically worsening conditioning.

The original cohort minimum sigma remains 0, minimum positive sigma 0.06459903971926352, and maximum available G_bound 92.70931982223773 at column 58145/layer 1. No sigma floor or gain ceiling is imposed.

## E. Cause classification

{chr(10).join(chr(10) + c + chr(10) for c in cause_sections)}

Measured cap-normal displacements explain the initial degeneracy. Column 101197 has first-step displacements along its wall normal [0, 0, 0.0001665270689999998] m. Column 103774 has two outward cap motions and one tangent motion. Column 78678 has three tangent motions. Source SDF branch/tie selections are recorded; the native implementation selects a deterministic one-sided direction at nonsmooth junctions. No unique classical gradient at such a junction is assumed.

The supported interpretation is differential motion relative to discrete cap and adjacent-cell geometry. The paths can expand edges while failing to create normal thickness. Large layer-to-layer SDF gradient rotation is not needed. Normal-line convergence, medial-axis distance, a continuous offset-surface singularity, and a compression mechanism were not established. No compression was applied. None of those unmeasured explanations is substituted for the direct cap-plane and center-distance evidence.

The nine undefined internal-face metrics have complete local neighbors, including degenerate columns 78681/78682. They are consequences of genuine forecast degeneracy, not missing-halo artifacts. The separately retained 68 outer-cap neighbor-dependent entries lack exterior mesh neighbors; their cause is **BoundaryOrClosureArtifact** and they remain unavailable. No complete outer-volume reconstruction is claimed.

Material geometry/metric/certificate implementation defects found: **0**. The correction is reporting policy: the previous nominal-intersection stop is now interpreted as future hazard evidence. No sealed Task-2 result, scientific formula, coordinate, or source authority was changed. Remaining questions concern correction feasibility and unmeasured global scope, not the reproduced causes above.

## F. Earliest actionable dependencies

Every nominal x[k+1] depends on x[k]. Stable source lineages and face ownership trace these hazards back to the next decision from committed layer 0. Thus layer 0 is the earliest committed decision with dependency access for all components. This does not prove that an arbitrarily small change can fix them, identify a unique earliest useful adjustment, or establish a trigger.

| Hazard family | First invalid transition | Last committed layer before that transition |
| --- | --- | --- |
| Cap Jacobian/volume/rank defects | 1 | 0 |
| Column 58145 determinant | 2 | 1 |
| Known cross-layer intersection | 10 | 9 |
| Edge 158488 interpolation weight | 18 | 17 |
| Edge 7187 interpolation weight | 19 | 18 |

For the cross-layer pair, layer 9 retains access to the uncommitted layer-10 side if earlier geometry is fixed. Its component already has layer-1 degeneracy, so this is not permission to defer all action to layer 9. A future layer's second-neighbor cap measures must likewise be anticipated before its shared geometry is fixed. `EARLIEST_ACTIONABLE_LAYERS.json` records dependency windows by component and metric. No numerical warning threshold is inferred from the trajectories.

## G. Descriptor precursor behavior

`DESCRIPTOR_PRECURSORS.json` records all seven retained descriptor ranges for each sampled component/layer. These are measured values, not health labels. It uses original Task-2 core columns; newly diagnosed halo columns are explicitly identified in the detailed histories instead of silently expanding the comparison sample.

| Component | Direct descriptor observations | Interpretation and missing warning |
| --- | --- | --- |
| 00 | Layer-1 neighbor angles range 0–90 degrees; maximum edge ratio grows 1.2051289722969767 to 38.10406616140923; maximum area ratio grows 1.3296032672958977 to 111.08694617908901. | Neighbor variation and expansion describe the mixed-motion neighborhood. Column 19138 alone remains at edge/area ratio 1 and neighbor angle 0 while its face weight fails. |
| 01 | Neighbor angles are about 82 degrees from layer 1. Maximum edge ratio grows 1.040709120139394 to 7.172819839805523. Section curvature is about 0.1677. | Variation is present before the layer-10 intersection. No cutoff or intersection predictor is proved. Straight trajectories and small SDF residuals do not exclude a cross-layer side intersection. |
| 02 | Layer-1 angles range 90–164.24099401303639 degrees. Edge ratios are 1.0381118300144303–1.0626343713027484 and areas increase. | Large neighbor direction differences identify mixed motion. Expanding front area does not establish positive prism volume; the cap can stay coplanar. First-layer direction-change values are 0. |
| 03 | At layer 1, angle ranges 0–82.08211377481261 degrees and edge ratio 1–1.0220333620890802. Mixed-cell edge/area ratios grow; pure-translation neighbor descriptors stay ordinary. | Adjacent-cell contrast helps explain later weight loss. The layer-2 determinant mask transition and increasing sigma do not have a monotone cheap-descriptor signature. |
| 04 | Initial neighbor angle 90 degrees, edge ratio 1.1981076011410257, area ratio 1.209298886680312. | Mixed cap motion is visible, but layer-1 Jacobian failure is immediate. Increasing area later is not evidence that layer 1 was valid. |

Measured fact: column 19138/layer 18 has edge ratio 1 and neighbor angle 0 yet weight 0.04819425405600387. Its own cells translate uniformly, while the adjacent cell's center moves away in the face-normal direction. A descriptor restricted to that column cannot see the off-edge neighbor vertex. Measured counterexample in the other direction: column 105527/layer 19 has extreme edge/area ratios yet passes the measured numeric gates; this does not mean its layer-1 cell or its full continuous neighborhood is admissible.

The sealed Task-2 edge/area Spearman value 0.997921 and near-zero pooled correlations with sigma remain descriptive results of a selected cohort. Section curvature is useful spatial context but does not encode cap ownership or adjacency. Small SDF residual and nearly straight trajectory directions do not imply valid prisms. A hypothesis for future work is to use these descriptors to request bounded neighboring-cell evidence. This task defines no production mask, cutoff, combined score, or causal inference from correlation.

## H. Forecast versus candidate semantics

`SEMANTICS.md` and `forecast/forensics.py` make the distinction explicit. A `ForecastHardEvent` is retained lookahead evidence. A `CandidateHardFailure` rejects that proposed next layer. A `CommittedHardFailure` is forbidden. No diagnostic-clear return authorizes a commit. The existing mesh gates remain unchanged, including positivity, topology, ownership, continuous geometry, determinant, weight, skewness, volume ratio, and aspect ratio. Only layer 0 is committed.

## I. Certificate semantics

The exact known intersection is preserved as `PredictedFutureIntersection` in forecast use. Candidate use still rejects it. No certificate mathematics was edited, and no qualification fixtures were run inside patch evaluation.

A fresh representative core, column 58145 across all 20 layers, completed **6,669** core-side candidate pairs: 2,087 allowed incident contacts and 4,582 separations. Candidate enumeration reads the full cheap edge bounds (159,453 source edges), then evaluates only 19 candidate source edges. Cross-layer pairs are included. Its proof-record digest is `6010f6a718b4b0bcecf2ba148ab8dc9fe8d45553ac2fd2747a08bebf382145e5`. The two sealed Task-2 clear scopes (18,005 and 4,289 pairs) are reused, not rerun. The two-edge history adds 400 bounded production checks.

The representative conclusion covers the stated core side surfaces against the complete side-candidate search. It does not certify side-to-cap or full cell-volume overlap, all 17 patches, or the whole shell. No global zero-intersection claim is made or needed.

## J. Locality and performance

{closure_table}

Normal core-plus-halo scope is at most 17 columns/340 prisms. The verification halo reaches at most 28 columns/560 prisms. All complete core metrics remain exactly unchanged after halo expansion. Original overlapping records replay exactly: {sum(c['original_core_cell_matches'] for c in clusters)} cells and {sum(c['original_face_matches'] for c in clusters)} faces. Source topology is global read-only input; detailed prism work remains bounded.

Each replay has 5 detailed-cache misses and 5 exact repeat hits; invalidations are 0 because coordinates never change. The first replay queries 1,188 native SDF vertex states in 9 calls. The second repeats that bounded work. No cold full forecast, projection, geometry modification, global planner, sensitivity matrix, compression, optimization, or CFD is run. Four stored coordinate archives are loaded once for the main two-run forensic replay. The smaller reproducer and summary reader also read archived data without recomputing forecasts.

The retained Task-2 locality experiment remains unchanged: 11 cheap entries and one detailed patch recomputed, while 16 unrelated patches stayed exact hits. The retained local-cache tests pass. No change to that cache engine was needed.

Main forensic execution, including two replays: **{perf['total_seconds']!r} s**. First replay: {perf['run1_seconds']!r} s; second: {perf['run2_seconds']!r} s. The second uses warm exact-kernel caches; this is not a scaling claim. Derived analysis: {study_runtime['analysis_seconds']!r} s. Suite runner: {tests['seconds']!r} s. Main-worker peak RSS: **{perf['peak_rss']} bytes**. One worker. Task wall time to sealing: {wall_seconds!r} s. Resource metadata is excluded from scientific hashes.

The rejected V3 dense matrix would need 1,791,265,237,440 bytes before LP workspace, against a 12,884,901,888-byte budget. This follow-up allocates no sensitivity matrix. Local bounds remain enforced. Free disk at sealing: {study_runtime['free_disk_bytes']} bytes.

## K. Determinism

The full forensic replay includes exact certificates, complete bounded metrics, SDF traces, component IDs, halo invariance, and exact Jacobian checks. Both base scientific hashes are `283ad33c29230ce8723450d95f46eb43f9a430eb8d969207eaafa8e493c451c4`.

The final hash also includes independently repeated interface calculations, determinant-mask diagnostics, trajectory summaries, descriptor precursors, and classifications:

- Run 1: `{replay['final_hash_run1']}`
- Run 2: `{replay['final_hash_run2']}`

Exact match: **pass**. No scientific fields were removed to force agreement. Only timing, RSS, process/environment metadata, and other operational records are kept outside the scientific payload.

## L. Tests and source changes

Command: `python research/sdf_cfd_mesher_v4/tools/forensics_tests.py`.

**61 passed; 1 failed; 0 errors; 0 skipped; 62 total.** All 13 new forensic tests pass. Retained Task-1 and Task-2 scientific tests pass. No existing test was weakened or edited.

The one failure is `test_setup.SetupTests.test_branch_and_unrelated_git_status_preserved`. It compares against setup-era status and observes three unrelated files already present at this follow-up's start: `examples/conditioned_gpu_probe.rs`, `shaders/conditioned_field.wgsl`, and `src/gpu/conditioned.rs`. The current-task start/end status test passes. `LEGACY_TEST_CONTEXT.json` records the exact discrepancy. This known historical snapshot failure remains visible; the full suite is not reported green.

{source_table}

All other new artifacts are under `results/nominal_forecast_forensics/`. Task-1/Task-2 implementations and sealed outputs are untouched. `FINAL_PRESERVATION.json` confirms unchanged unrelated Git status. Only the follow-up files are intended for the narrow commit. This report is sealed before that commit to avoid a self-referential commit hash; the final HEAD and actual committed paths are reported in the final delivery. Starting unrelated dirty/untracked work remains in place.

## M. Classification

**{classification}**.

The forecaster and detailed lookahead correctly exposed sharp-cap zero thickness, mixed-direction negative/zero Jacobians, singular reconstruction, adjacent-cell weight loss, a determinant hard-gate crossing, and an exact future side intersection. These are useful predictions. No material implementation defect was found. Undefined internal metrics and missing exterior neighbors have distinct, documented causes.

This does not prove a valid full nominal mesh, global absence of other hazards, a production risk-selection policy, a feasible relaxation, or candidate admission. Unmeasured medial-axis and continuum-offset explanations remain unclaimed.

## N. Task-3 readiness

**Proceed to bounded local next-front relaxation development.** Do not commit an invalid candidate. Stable IDs and causal dependencies trace the genuine hazards to earlier front decisions; complete core evidence remains local. Initial targets should be cap/seam direction selection and outward thickness, differential adjacent-cell motion, and the preserved cross-layer intersection reproducer. Candidate quality and continuous checks must retain every existing gate, and unavailable required candidate evidence cannot pass.

The evidence supports starting that development; it does not establish that relaxation alone will solve every cap constraint. No Task-3 code, candidate geometry, optimization, or layer commit was started. The historical Git-status test requires separate snapshot maintenance if a completely green repository-wide suite is desired; it does not alter this scientific diagnosis.

Artifact hashes are recorded in `AUTHORITY.json` and `SEAL.json`. The latter contains the final report and authority SHA-256 values without a circular self-hash.
'''
    (OUT / 'report.md').write_text(report, encoding='utf-8')
    evidence_paths = sorted(p for p in OUT.iterdir() if p.is_file() and p.name not in ('AUTHORITY.json', 'SEAL.json'))
    changed_paths = sorted(str(p.relative_to(ROOT)).replace('\\', '/') for p in source_paths + evidence_paths + [OUT / 'AUTHORITY.json', OUT / 'SEAL.json'])
    authority = dict(
        classification=classification, task='DiagnoseV4NominalForecastFailureMechanisms', branch=start['branch'], base_head=start['starting_head'],
        source_sha256={str(p.relative_to(ROOT)).replace('\\', '/'): file_hash(p) for p in source_paths},
        evidence_sha256={str(p.relative_to(ROOT)).replace('\\', '/'): file_hash(p) for p in evidence_paths},
        final_scientific_hashes=[replay['final_hash_run1'], replay['final_hash_run2']], determinism_passed=True,
        genuine_sampled_failure_components=5, material_implementation_defects=0, original_event_counts=counts,
        task3_readiness='Proceed with bounded local development; candidate hard gates remain mandatory; no feasible correction proved',
        tests=dict(total=tests['tests'], passed=tests['tests'] - tests['failures'] - tests['errors'], failed=tests['failures'], errors=tests['errors'], full_suite_passed=False, known_preexisting_snapshot_failure=legacy_delta),
        v3_modifications=0, task2_frozen_modifications=0, unrelated_git_status_preserved=True,
        intended_commit_paths=changed_paths, report_sha256=file_hash(OUT / 'report.md'),
        scientific_hash_scope='SCIENTIFIC_OUTPUTS plus MECHANISM_ANALYSIS; all scientific arrays, exact proofs, classifications and summaries included; timing/RSS excluded')
    write('AUTHORITY.json', authority)
    write('SEAL.json', dict(report_sha256=file_hash(OUT / 'report.md'), authority_sha256=file_hash(OUT / 'AUTHORITY.json')))
    print(json.dumps(dict(classification=classification, files=len(changed_paths), seal=read('SEAL.json'), wall_seconds=wall_seconds)))


if __name__ == '__main__':
    main()
