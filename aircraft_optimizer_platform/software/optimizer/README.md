# Optimizer

Purpose: platform-owned optimizer implementation.

Responsibilities:

- Campaign creation.
- Candidate registry.
- Candidate lifecycle.
- Run database.
- Artifact registry.
- Evaluation orchestration.
- Analysis module execution.
- Scoring.
- Lineage tracking.
- Failure classification.
- Optimizer ask/tell integration.

Initial implementation target:

- v0.1 Platform Skeleton.

Current architecture work:

- Core contracts live in `contracts/`.
- The exporter is treated as a module adapter, not as the platform root.
- v0.1 runnable slice is specified in `v0_1_skeleton_spec.md`.
- v0.1 uses the local-first environment strategy from `..\..\environment_strategy.md`.
- The initial fixed-wing variable schema lives in `schemas\fixed_wing_uav_reference.variables.v0_1.json`.
- The v0.1 fixture pipeline config lives in `pipelines\v0_1_fixture.pipeline.json`.
- The primary sequential gated workflow is documented in `contracts\sequential_gated_pipeline_contract.md` and configured in `pipelines\sequential_gated_v0_1.pipeline.json`.

Status: v0.1 skeleton implemented and smoke-tested.

## v0.1 Fixture

Run tests:

```powershell
python -m pytest
```

Run the local fixture from this folder:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main run-fixture --workspace ..\..\runs\v0_1_fixture
```

Summarize a fixture workspace:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main summarize --workspace ..\..\runs\v0_1_fixture
```

The summary includes table counts, event types, evaluation statuses, runner
states, module statuses, artifact types, and failure categories.

## Runner State

Current candidate execution state is stored in `candidate_runner_states`.
Detailed history still lives in events, evaluations, module attempts, failures,
and artifacts; `candidate_runner_states` is the dashboard-friendly current-state
view.

Current states include:

- `queued`
- `running`
- `meshing`
- `cfd_running`
- `scored`
- `promoted`
- `rejected`
- `failed`

The fixture, wing-options study, sequential-gated optimizer, and real no-inlet
export validation path write runner state records. The native dashboard prefers
these records and falls back to inferred state only for older workspaces.

Run the primary sequential gated optimizer skeleton:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main run-sequential-gated --workspace ..\..\runs\sequential_gated_v0_1 --iterations 3
```

This command proposes one candidate at a time through the Halton ask/tell policy. Passing candidates continue through fixture geometry/export and low-fidelity screening; failing candidates stop at `pre_export_screening` and are preserved as classified failures.
Passing candidates also run geometry-definition screening after geometry provider output and before STL export. That screen checks feature identity, bounding-box reasonableness, estimated mesh-cell budget, narrow-feature risk, and virtual-layout metadata before any export work is attempted.

The active v0.1 schema still preserves the original four required wing variables
and adds optional fixed-topology stress-test knobs:

- `wing.airfoil_selector`: numeric selector rounded to one of three airfoil
  families: `thin_loiter`, `balanced_uav`, or `high_lift_stable`.
- `fuselage.length_delta_mm`: fuselage length change from `-100` to `+100 mm`.
- `fuselage.nose_bluntness`: normalized nose-shape proxy from `0` to `1`.
- `fuselage.tail_bluntness`: normalized tail-shape proxy from `0` to `1`.

These are recorded in each candidate and used by the low-fidelity screening
module as relative ranking signals only. They do not authorize topology changes.

Run the expanded 15-candidate stress test:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main run-sequential-gated --workspace ..\..\runs\stress_15_airfoil_fuselage_20260627 --iterations 15
```

Latest result for that workspace:

- `15` candidates registered.
- `12` evaluated.
- `3` failed pre-export screening.
- failure checks: `min_taper_ratio` plus `min_tip_chord_mm`, `min_aspect_ratio`,
  and `max_taper_ratio`.
- promoted candidate:
  `candidate_2561163970fd4babb73ba495c3c63d97`.
- promoted score: `score.low_fidelity_total = 9.095638`.

Run the expanded-variable real export-only check:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main run-real-no-inlet-export-batch --workspace ..\..\runs\real_export_expanded_vars_abs_20260627 --iterations 3 --timeout-seconds 1200
```

Latest result for that workspace:

- `3` generated Rhai geometries from candidate design variables.
- `3` real direct sparse OML STL exports.
- `3/3` export quality gates passed.
- `3/3` `check-cfd-readiness` checks passed.
- `0` mesh or CFD runs.
- Export screenshots:
  `..\..\runs\real_export_expanded_vars_abs_20260627\export_screenshots`.

Run with an editable virtual component layout config:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main run-sequential-gated --workspace ..\..\runs\sample_edf_layout --iterations 1 --virtual-components-config .\configs\virtual_components_printed_edf_sample.v0_1.json
python -m aircraft_optimizer.cli.main report-layout --workspace ..\..\runs\sample_edf_layout
```

Report on the latest evaluation in a workspace:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main report-evaluation --workspace ..\..\runs\v0_1_fixture
```

Report on the latest campaign in a workspace:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main report-campaign --workspace ..\..\runs\v0_1_fixture
```

Compare the top two scored candidates in the latest campaign:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main report-comparison --workspace ..\..\runs\v0_1_fixture
```

Check the external WSL CFD tool environment:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main check-cfd-tools
```

Check whether a parsed OML STL export-result JSON is suitable for CFD setup:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main check-cfd-readiness --export-result .\examples\exporter_results\oml_stl_passed.json
python -m aircraft_optimizer.cli.main check-cfd-readiness --export-result .\examples\exporter_results\oml_stl_failed_quality_gate.json
```

The readiness check is a pre-CFD topology and quality gate. It does not run SU2/OpenFOAM and does not calculate static margin.

Check an STL surface directly before meshing:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main check-cfd-surface --stl ..\..\..\su2_sandbox\runs\basic_no_inlet_wing_span_plus20_openfoam\aircraft_poisson_d8.stl
```

This gate reports boundary/nonmanifold edges, duplicate and degenerate triangles, triangle quality, minimum edge length, and triangle/vertex counts. A failed raw surface can still proceed to a repair/remesh experiment, but the failed gate must be preserved as candidate evidence.

Validate a solver-ready CFD volume mesh result before solver execution:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main check-mesh-result --mesh ..\..\..\su2_sandbox\runs\occ_lenticular_wing_50mph\external_flow.su2 --mesh-format su2
```

For CGNS meshes generated by Gmsh, pass the SU2 preprocessing log and explicit semantic marker mapping:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main check-mesh-result `
  --mesh ..\..\..\su2_sandbox\runs\occ_lenticular_wing_50mph_cgns\external_flow.cgns `
  --mesh-format cgns `
  --su2-log ..\..\..\su2_sandbox\runs\occ_lenticular_wing_50mph_cgns\su2_cgns_sections_stdout.log `
  --aircraft-marker 3_S_7 `
  --farfield-marker 3_S_8 --farfield-marker 3_S_9 --farfield-marker 3_S_10 `
  --farfield-marker 3_S_11 --farfield-marker 3_S_12 --farfield-marker 3_S_13 `
  --fluid-marker 5_V_1
```

Native SU2 meshes expose the fluid as the implicit single volume. CGNS meshes preserve solver-visible section names such as `3_S_7` only in the adapter marker map; platform-facing semantics remain `aircraft`, `farfield`, and `fluid`.
Use `modules\mesh_result_adapter.py` when this check needs to be recorded in the run database as a `mesh_validation` module attempt with `cfd_mesh_result_json` and mesh/log artifacts.

Validate an OpenFOAM smoke result:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main check-openfoam-result `
  --case-dir ..\..\..\su2_sandbox\runs\basic_no_inlet_wing_span_plus20_openfoam\case `
  --check-mesh-log ..\..\..\su2_sandbox\runs\basic_no_inlet_wing_span_plus20_openfoam\case\log.checkMesh `
  --solver-log ..\..\..\su2_sandbox\runs\basic_no_inlet_wing_span_plus20_openfoam\case\log.potentialFoam
```

Use the Gmsh boundary-layer development policy for the current Gmsh
`create-topology` prism-layer plumbing path:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main check-openfoam-result `
  --case-dir ..\..\..\custom_cfd_mesher_experiment\runs\gmsh_bl_auto_selector_v0_2_20260623\fcv01_long_glider\gmsh_bl\openfoam_case `
  --check-mesh-log ..\..\..\custom_cfd_mesher_experiment\runs\gmsh_bl_auto_selector_v0_2_20260623\fcv01_long_glider\gmsh_bl\openfoam_case\log.checkMesh `
  --solver-log ..\..\..\custom_cfd_mesher_experiment\runs\gmsh_bl_auto_selector_v0_2_20260623\fcv01_long_glider\gmsh_bl\openfoam_case\log.potentialFoam `
  --acceptance-mode gmsh_bl_development
```

Use the snappy high-fidelity development policy for the current v0.4
snappyHexMesh surface-fidelity path:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main check-openfoam-result `
  --case-dir ..\..\..\custom_cfd_mesher_experiment\runs\snappy_hifi_1layer_all_variants_candidate_20260625\fcv01_long_glider\openfoam_case `
  --check-mesh-log ..\..\..\custom_cfd_mesher_experiment\runs\snappy_hifi_1layer_all_variants_candidate_20260625\fcv01_long_glider\openfoam_case\log.checkMesh `
  --solver-log ..\..\..\custom_cfd_mesher_experiment\runs\snappy_hifi_1layer_all_variants_candidate_20260625\fcv01_long_glider\openfoam_case\log.potentialFoam `
  --acceptance-mode snappy_hifi_development
```

The current OpenFOAM validator checks `checkMesh`, patch face counts, non-orthogonality, skewness, `potentialFoam` completion, continuity error, and interpolated velocity error. It is a mesh/solver plumbing gate, not validated aerodynamic scoring. `gmsh_bl_development` is non-scoring and exists because the current Gmsh BL meshes pass strict OpenFOAM `checkMesh` while retaining high localized non-orthogonality near sharp feature regions. `snappy_hifi_development` is also non-scoring and exists for the current high-fidelity snappy path; it requires `mesh_ok`, at least `50,000` aircraft patch faces, max non-orthogonality <= `70`, and max skewness <= `12`.

Persist an already-run OpenFOAM smoke result into the optimizer database:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main persist-openfoam-smoke `
  --workspace ..\..\runs\openfoam_smoke_plumbing `
  --case-dir ..\..\..\su2_sandbox\runs\basic_no_inlet_wing_span_plus20_openfoam\case_prepared_edgefan_l3_l4_smooth_t1 `
  --check-mesh-log ..\..\..\su2_sandbox\runs\basic_no_inlet_wing_span_plus20_openfoam\case_prepared_edgefan_l3_l4_smooth_t1\log.checkMesh_relaxed_skew6 `
  --solver-log ..\..\..\su2_sandbox\runs\basic_no_inlet_wing_span_plus20_openfoam\case_prepared_edgefan_l3_l4_smooth_t1\log.potentialFoam `
  --acceptance-mode relaxed_development `
  --strict-check-mesh-log ..\..\..\su2_sandbox\runs\basic_no_inlet_wing_span_plus20_openfoam\case_prepared_edgefan_l3_l4_smooth_t1\log.checkMesh `
  --relaxed-check-mesh-log ..\..\..\su2_sandbox\runs\basic_no_inlet_wing_span_plus20_openfoam\case_prepared_edgefan_l3_l4_smooth_t1\log.checkMesh_relaxed_skew6 `
  --aircraft-iso ..\..\..\su2_sandbox\runs\basic_no_inlet_wing_span_plus20_openfoam\case_prepared_edgefan_l3_l4_smooth_t1\aircraft_iso_surface.png
```

For the current Gmsh boundary-layer path, use:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main persist-openfoam-smoke `
  --workspace ..\..\runs\openfoam_smoke_plumbing `
  --case-dir ..\..\..\custom_cfd_mesher_experiment\runs\gmsh_bl_auto_selector_v0_2_20260623\fcv01_long_glider\gmsh_bl\openfoam_case `
  --check-mesh-log ..\..\..\custom_cfd_mesher_experiment\runs\gmsh_bl_auto_selector_v0_2_20260623\fcv01_long_glider\gmsh_bl\openfoam_case\log.checkMesh `
  --solver-log ..\..\..\custom_cfd_mesher_experiment\runs\gmsh_bl_auto_selector_v0_2_20260623\fcv01_long_glider\gmsh_bl\openfoam_case\log.potentialFoam `
  --acceptance-mode gmsh_bl_development `
  --strict-check-mesh-log ..\..\..\custom_cfd_mesher_experiment\runs\gmsh_bl_auto_selector_v0_2_20260623\fcv01_long_glider\gmsh_bl\openfoam_case\log.checkMesh
```

This command records `openfoam_smoke_validation` as an analysis module attempt and stores the case reference, logs, validation JSON, and aircraft-only ISO screenshot when supplied. It does not execute OpenFOAM, and development results are always non-scoring.

Validate a non-potential steady/no-slip OpenFOAM development result:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main check-openfoam-steady-result `
  --case-dir ..\..\..\custom_cfd_mesher_experiment\runs\gmsh_bl_no_slip_smoke_v0_1_20260624\fcv04_laminar_p02_u03_20 `
  --solver-log ..\..\..\custom_cfd_mesher_experiment\runs\gmsh_bl_no_slip_smoke_v0_1_20260624\fcv04_laminar_p02_u03_20\log.incompressibleFluid `
  --check-mesh-log ..\..\..\custom_cfd_mesher_experiment\runs\gmsh_bl_auto_selector_v0_2_20260623\fcv04_compact_wide_tail\gmsh_bl\openfoam_case\log.checkMesh
```

For the current snappy v0.4 bounded coefficient-development path, use
`--acceptance-mode snappy_hifi_coefficient_development`. That named policy is
`scoring_tier=confirmation_scoring_development`; it requires completed solver
execution, no fatal errors, force coefficients, y-plus availability, local
continuity <= `1e-4`, final velocity residual <= `1e-4`, Cd/Cl/Cm
final-window spans <= `0.005`, y-plus p95 <= `60`, and y-plus max <= `250`.
Final scoring remains blocked.

Persist a non-potential steady/no-slip result into an existing optimizer
workspace:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main persist-openfoam-steady `
  --workspace ..\..\runs\gmsh_bl_openfoam_persistence_v0_1 `
  --evaluation-id evaluation_f832b0049cb947b9a54652c58c9d70d2 `
  --case-dir ..\..\..\custom_cfd_mesher_experiment\runs\gmsh_bl_no_slip_smoke_v0_1_20260624\fcv04_laminar_p02_u03_20 `
  --solver-log ..\..\..\custom_cfd_mesher_experiment\runs\gmsh_bl_no_slip_smoke_v0_1_20260624\fcv04_laminar_p02_u03_20\log.incompressibleFluid `
  --check-mesh-log ..\..\..\custom_cfd_mesher_experiment\runs\gmsh_bl_auto_selector_v0_2_20260623\fcv04_compact_wide_tail\gmsh_bl\openfoam_case\log.checkMesh `
  --force-coeffs ..\..\..\custom_cfd_mesher_experiment\runs\gmsh_bl_no_slip_smoke_v0_1_20260624\fcv04_laminar_p02_u03_20\postProcessing\aircraftForceCoeffs\0\forceCoeffs.dat
```

This records `openfoam_steady_validation`, parses residuals and force
coefficients, reports coefficient-window stability, records `yPlus` percentiles
when a field is available via optional `--yplus`, and stores qualified scoring
metadata. The legacy `scoring_allowed=false` field means final scoring is
blocked, not that rough or confirmation scoring evidence must be ignored.

Import a full existing five-variant Gmsh BL selector run into a fresh optimizer
workspace:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main ingest-gmsh-bl-run `
  --workspace ..\..\runs\gmsh_bl_openfoam_persistence_v0_1 `
  --summary ..\..\..\custom_cfd_mesher_experiment\runs\gmsh_bl_auto_selector_v0_2_20260623\summary.json
```

This command creates one campaign, one candidate and evaluation per variant,
and persists each case as `openfoam_smoke_validation` with
`acceptance_mode=gmsh_bl_development`. It imports existing evidence only; it
does not rerun Gmsh or OpenFOAM.

Import the current five-variant snappy v0.4 evidence package:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main ingest-snappy-hifi-run `
  --workspace ..\..\runs\snappy_hifi_openfoam_persistence_v0_4 `
  --run-root ..\..\..\custom_cfd_mesher_experiment\runs\snappy_hifi_1layer_all_variants_candidate_20260625
```

This command creates one candidate and evaluation per v0.4 variant, persists
each surface-deviation report as `snappy_surface_fidelity_validation`, each
`potentialFoam` case as `openfoam_smoke_validation` with
`acceptance_mode=snappy_hifi_development`, and each bounded kOmegaSST no-slip
development run as `openfoam_steady_validation` when present. Steady imports
use `acceptance_mode=snappy_hifi_coefficient_development`, persist the OpenFOAM
`yPlus` field as `openfoam_yplus_field`, and expose
`openfoam_steady.yplus_p50`, `p95`, `p99`, and `max` metrics. It imports
existing evidence only, does not rerun snappyHexMesh or OpenFOAM, and keeps
final scoring blocked.

Run or import the wrapped snappy/OpenFOAM backend path:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main run-snappy-openfoam-backend `
  --workspace ..\..\runs\snappy_backend_wrapper_smoke_v0_4 `
  --run-root ..\..\..\custom_cfd_mesher_experiment\runs\snappy_hifi_1layer_all_variants_candidate_20260625 `
  --skip-execution
```

Without `--skip-execution`, this command runs the current snappy v0.4 meshing
harness, `potentialFoam`, bounded laminar no-slip startup, bounded
laminar-start `kOmegaSST`, and then imports the artifacts into the optimizer
database. Use a fresh `--run-root` for execution.

The snappy backend import also creates `rough_cfd_scoring` module attempts when
steady OpenFOAM evidence is available. The default user-editable scoring config
is:

```text
configs\rough_cfd_stable_efficient_drone.v0_1.json
```

Override the scoring behavior without editing code:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main run-snappy-openfoam-backend `
  --workspace ..\..\runs\snappy_backend_custom_scoring `
  --run-root ..\..\..\custom_cfd_mesher_experiment\runs\raw_direct_faired_cap_snappy_ranking_v0_1_three_variant_20260625 `
  --preset .\configs\snappy_openfoam_external_aero_ranking.v0_1.json `
  --rough-scoring-config .\configs\rough_cfd_stable_efficient_drone.v0_1.json `
  --skip-execution
```

The default rough score favors L/D, low drag, usable positive lift, small
pitching moment, and solver/mesh confidence. It is intended for optimizer
ranking only; final scoring remains blocked.

Rough scoring can now consume a mesh-reuse alpha sweep when the run root
contains the configured `alpha_sweep_summary_name`. The default ranking preset
uses a cheap first pass, `0/+4 deg`, with rotated freestream vectors and matching
lift and drag axes. Expanded survivor scoring uses `-2/0/+4/+8/+10 deg` and can
be selected during import with:

```powershell
--alpha-sweep-summary-name alpha_sweep_summary_survivor_end60_np8.json
```

The scorer records best raw L/D, CL slope, and Cm trend, but ranks with the best
positive-lift point. In the first-pass `candidate_screening` stage, pitch moment
is a soft review signal because two points are not enough to prove the real trim
picture; the scorer only rejects clearly bad results such as missing forces,
non-positive lift across sampled alphas, invalid drag, solver failure, or absurd
coefficients. Expanded survivor scoring applies the stricter pitch-moment
selection. If no alpha summary exists, it falls back to the single-condition
force coefficients. `score.rough_total` is still optimizer-relative rough
scoring, not final engineering scoring.

When `run-snappy-openfoam-backend` is allowed to execute instead of
`--skip-execution`, it runs the configured alpha stage after meshing and the
baseline no-slip solver. The default is the first pass. For existing run roots,
the same command can import either staged summary with `--skip-execution` and
`--alpha-sweep-summary-name`.

Adaptive promotion is handled by optimizer policy, not a fixed survivor count,
but it is no longer the default launch mode. Fresh backend runs default to the
cheap first-pass stage so a normal candidate batch does not accidentally start
the full expanded sweep. Use `--alpha-strategy adaptive` only when deliberately
running baseline-building or promoted-survivor expansion. In adaptive mode,
first-pass results are compared at `+4 deg`; only hard failures or obvious
underperformance are rejected. A candidate with `>=65%` less `CL` at `+4 deg`
than the best baseline case is the current clear-reject example.

Fresh backend runs default to first-pass alpha execution:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main run-snappy-openfoam-backend `
  --workspace ..\..\runs\candidate_006_cfd `
  --run-root ..\..\..\custom_cfd_mesher_experiment\runs\candidate_006_cfd `
  --preset .\configs\snappy_openfoam_external_aero_ranking.v0_1.json
```

The current v0.2 snappy/OpenFOAM preset keeps candidates sequential and uses
OpenFOAM parallelism inside each case. It also enables conservative early alpha
pruning for first-pass sweeps. The pruning gate never fires after `0 deg`
alone; it only skips remaining alpha cases after at least two attempted alpha
results show an obvious failure such as repeated solver failure, coefficient
blowup, no meaningful positive lift, or very weak lift with very poor L/D.
Healthy or merely uncertain candidates continue through the full first-pass
alpha set.

To run explicit adaptive promotion instead:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main run-snappy-openfoam-backend `
  --workspace ..\..\runs\candidate_006_cfd `
  --run-root ..\..\..\custom_cfd_mesher_experiment\runs\candidate_006_cfd `
  --preset .\configs\snappy_openfoam_external_aero_ranking.v0_1.json `
  --alpha-strategy adaptive `
  --adaptive-baseline-summary ..\..\..\custom_cfd_mesher_experiment\runs\candidate_001_cfd\alpha_sweep_summary_survivor_end60_np8.json `
  --adaptive-baseline-summary ..\..\..\custom_cfd_mesher_experiment\runs\candidate_002_cfd\alpha_sweep_summary_survivor_end60_np8.json
```

In explicit adaptive mode, when the provided baseline contains fewer than the
configured baseline count, the backend runs the expanded survivor sweep to keep
building the baseline. Once the baseline exists, it runs first pass, evaluates
the adaptive policy, runs the expanded sweep only for promoted candidates, writes
`alpha_sweep_summary_adaptive_end<time>_np<procs>.json`, and imports that
combined summary.

To inspect the current promotion bar before a run:

```powershell
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main report-adaptive-cfd-promotion `
  --preset .\configs\snappy_openfoam_external_aero_ranking.v0_1.json `
  --baseline-summary ..\..\..\custom_cfd_mesher_experiment\runs\candidate_001_cfd\alpha_sweep_summary_survivor_end60_np8.json `
  --candidate-alpha-summary ..\..\..\custom_cfd_mesher_experiment\runs\candidate_006_cfd\alpha_sweep_summary_first_pass_end60_np8.json `
  --variant-id candidate_006
```

The report prints the baseline count, whether the baseline is ready, the best
baseline `CL` at the comparison alpha, the exact first-pass `CL` reject
threshold, hard rejection reasons, and the candidate decision when candidate
data is supplied.

As of 2026-06-26, the raw direct-STL rough-CFD baseline is established with
five expanded survivor sweeps. Use these summaries when launching adaptive
candidate shakedowns:

```powershell
--adaptive-baseline-summary ..\..\..\custom_cfd_mesher_experiment\runs\raw_direct_faired_cap_snappy_ranking_v0_1_three_variant_20260625\alpha_sweep_summary_survivor_end60_np8.json `
--adaptive-baseline-summary ..\..\..\custom_cfd_mesher_experiment\runs\raw_direct_faired_cap_snappy_baseline_fcv01_v0_1_20260626\alpha_sweep_summary_survivor_end60_np8.json `
--adaptive-baseline-summary ..\..\..\custom_cfd_mesher_experiment\runs\raw_direct_faired_cap_snappy_baseline_fcv05_v0_1_20260626\alpha_sweep_summary_survivor_end60_np8.json
```

Current promotion bar:

- Expanded baseline count: `5`.
- Best baseline `CL(+4 deg)`: `0.428726964`.
- Median baseline `CL(+4 deg)`: `0.397947692`.
- Clear first-pass reject threshold: `CL(+4 deg) <= 0.1500544374`.

This bar is deliberately permissive. It only rejects obvious underperformers;
survivor ranking is still done by the expanded alpha sweep.

The first larger run should use
`configs/rough_cfd_15_candidate_shakedown.v0_1.json` as its campaign boundary
contract. It defines the intended `5` established baselines plus `10` new
adaptive candidates, hard mesh/solver gates, runtime review limits, and
human-review-only cases.

The current 10 new candidates are wing-focused `shv` variants:

```text
configs/cfd_wing_shakedown_10_variants.v0_1.json
configs/raw_direct_wing_shakedown_10_stl_batch.v0_1.json
configs/wing_shakedown_10_export_manifest.v0_1.json
```

To launch them as a normal first-pass backend batch, use
`run-snappy-openfoam-backend` with `--input-stl-map`, `--variant-config`, and
the ranking preset. Add `--alpha-strategy adaptive` and the established
`--adaptive-baseline-summary` paths only when intentionally running promoted
survivor expansion.

CFD development status:

- The snappy v0.4 high-fidelity surface path is the default backend
  evidence path.
- Absolute resolved-wall layer probes are preserved under
  `custom_cfd_mesher_experiment/runs/snappy_resolved_yplus_fcv04_v0_1_20260625`
  and
  `custom_cfd_mesher_experiment/runs/snappy_resolved_lowre_solver_retry_fcv04_v0_1_20260625`,
  but are rejected for default use because kOmegaSST ran away.
- Low-Re wall treatment exists in the mesher experiment scripts for bounded
  experiments only. Do not treat it as validated scoring CFD.

The fixture creates:

- `..\..\runs\v0_1_fixture\optimizer.db`
- one campaign
- one variable schema
- one versioned pipeline config loaded into campaign config
- three candidates: one seed, one deterministic child mutation, and one deterministic failed-export fixture
- three candidate lineage records
- three evaluations: two complete and one failed
- three automatic-fixture geometry provider results
- three generated Rhai artifacts
- three generated parameter trace artifacts
- three mock module attempts
- three OML STL fixture adapter module attempts: two success and one failed quality gate
- one SU2 history fixture adapter module attempt
- two placeholder scoring module attempts
- copied/referenced OML export fixture artifacts
- three human-review annotation fixtures
- event log entries

The fixture and sequential gated skeleton do not run CAD, real exporter subprocesses, CFD solver cases, dashboard, or production optimizer algorithms beyond the minimal Halton ask/tell policy.

Static margin remains deferred until a CFD/stability module supplies a neutral point. The post-CFD layout-adjustment module can then move allowed internal components to improve the requested static margin while keeping fixed components such as the EDF locked in place.

The primary autonomous optimizer flow should remain sequential:

```text
design -> cheap pre-export checks -> geometry/export if allowed -> analysis if allowed -> results -> next design
```

The real optimizer pilot now applies a relaxed hard pre-export gate before SDF
export. The gate records `pre_export_screening_summary.json` in each generation
export workspace and only exports candidates that pass. This policy is
deliberately more permissive than the default screening module: it is meant to
reject only obviously implausible planforms, component-layout failures, or
extreme geometry risk before expensive SDF export, meshing, and CFD. Borderline
or uncertain candidates should continue so the optimizer can learn from CFD
evidence.

Do not use the five-option wing sweep as the default control pattern for real export or CFD. It is a diagnostic study harness.

## Native Optimizer Dashboard

The optimizer dashboard is a native Rust app at the repo root:

```text
src\bin\optimizer_dashboard.rs
```

It reads an optimizer workspace database, lists candidates, rough scores,
measured aero data, mesh/y-plus metrics, failures, module attempts, and
generation lineage. Candidate inspection uses the fast standalone native SDF
viewer rather than an embedded preview pane.

Dashboard tabs:

- `Overview`: campaign summary, best-result table, score/L-D and drag/lift
  scatter plots, and proposed next candidates when iterations have been
  generated.
- `Candidate`: selected-candidate detail plus an `Open / Update SDF Viewer`
  action.
- `Scoring`: score-vs-metric scatter plots and selected-candidate component /
  detractor breakdown.
- `Curves`: selected-candidate alpha sweep curves for CL, CD, L/D, and CM.
- `History`: operational run history from `candidate_runner_states` where
  available, with parent summaries, scoring metrics, mesh/y-plus status, and
  the lineage map.
- `Controls`: writes traceable dashboard control-request JSON files for
  start, pause, stop, generate-next, promote, and rerun requests.

Build from the repo root:

```powershell
cargo build --release --bin optimizer_dashboard
```

Run against the current pilot workspace:

```powershell
.\target\release\optimizer_dashboard.exe `
  --workspace .\aircraft_optimizer_platform\runs\pilot_loop_gen2_3_snappy_firstpass_import
```

The viewer action loads the best available candidate target. For current
imported CFD candidates, the dashboard first looks for generated Rhai/SDF
sources under:

```text
dual_contouring\direct_sparse_sdf_mc_experiment\scratch
```

It falls back to matching optimizer export-result JSON under:

```text
dual_contouring\direct_sparse_sdf_mc_experiment\logs
```

The embedded WGPU viewer path is intentionally disabled for now because it was
much slower and unstable inside the dashboard layout. The standalone
`sdf_viewer.exe` remains the fast preview path.

Dashboard-to-viewer control is process-isolated. The dashboard writes the
selected preview target to:

```text
<workspace>\dashboard_sdf_viewer_target.txt
```

and launches:

```powershell
.\target\release\sdf_viewer.exe --watch-target <workspace>\dashboard_sdf_viewer_target.txt
```

The viewer polls that file and reloads when the dashboard sends a new
candidate. This keeps the fast WGPU viewer while avoiding an unstable embedded
renderer.

When `Sync Viewer` is enabled in the dashboard, selecting a candidate updates
the linked viewer session automatically. The `Open / Update SDF Viewer` button
is still available to start or recover the viewer session manually.

The Python optimizer package remains the headless orchestration layer. It does
not serve the primary dashboard UI.

Dashboard controls do not directly run a background optimizer daemon yet. They
write request files under:

```text
<workspace>\dashboard_control_requests
```

A future runner/controller should consume those files and append normal
optimizer events, candidates, evaluations, and module attempts.

## Record Examples

Static example records live in:

```text
examples\records
```

Current examples:

- `candidate.json`
- `geometry_provider_result.json`
- `evaluation.json`
- `artifact.json`
- `module_attempt.json`
- `event.json`
- `failure.json`

Exporter result fixtures live in:

```text
examples\exporter_results
```

These fixtures use the existing OML exporter wrapper result shape. They are parser fixtures only; the platform skeleton still does not call the real exporter.
