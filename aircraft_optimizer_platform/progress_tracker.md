# Aircraft Optimizer Platform Progress Tracker

Date created: 2026-06-19

## Purpose

This file is the running project log for the aircraft optimizer platform. Update it as roadmap phases move forward, decisions are made, implementation starts, tests are run, blockers appear, or scope changes.

Use this file for concise factual progress. Keep deeper design detail in `opt_output.md`, roadmap sequencing in `roadmap.md`, and agent operating instructions in `AGENT_README.md`.

## Current Status

Overall status: v0.1 platform skeleton implemented and smoke-tested.

Current phase: Phase 1 - Headless Candidate Evaluation Platform, initial skeleton.

Next build target: run a larger real optimizer shakedown through the resumable pilot CLI, then tighten promotion/finalist controls and dashboard monitoring around the real export/mesh/rough-CFD loop.

Implicit-kernel research status: Phase 0 SDF conditioning literature review complete enough to unblock diagnostic-first local conditioning design. Production SDF-native CFD and PhysicsNeMo surrogate implementation remain deferred until optimizer automation, conditioning stability, CFD data schema, and validation evidence mature.

## Roadmap Phase Status

| Phase | Name | Status | Notes |
|---|---|---|---|
| 0 | Platform Foundation Definition | Complete | Initial architecture, roadmap, prompt capture, README, agent README, progress tracker, environment strategy, and contracts created. |
| 1 | Headless Candidate Evaluation Platform | In progress | v0.1 SQLite skeleton, candidate/evaluation/artifact/event records, manual-reference geometry provider, mock module, CLI fixture, and smoke test implemented. |
| 2 | Low-Fidelity Fixed-Wing Optimizer MVP | Not started | Depends on Phase 1 records and module contracts. |
| 3 | Local Dashboard and Human Review Workflow | Not started | Depends on stable database and artifact store. |
| 4 | Medium-Fidelity Aerodynamics | Not started | Tool choice should be validated against geometry export path. |
| 5 | Pareto, Comparison, and Campaign Analytics | Not started | Depends on meaningful campaign data. |
| 6 | Selective High-Fidelity CFD | Not started | Do not start until cheap gates and artifact/version records are reliable. |
| 7 | Scale, Workers, and Dataset Exports | Not started | Depends on stable local execution semantics. |
| 8 | Surrogate-Assisted Optimization | Not started | Depends on accumulated high-quality traceable data. |
| 9 | MDAO Execution Layer | Not started | Depends on stable discipline contracts. |
| 10 | Expanded Aircraft Families and Autonomous Studies | Not started | Requires explicit aircraft-family schemas and topology/configuration model. |

## Completed Work Log

### 2026-06-19

- Created platform folder: `aircraft_optimizer_platform`.
- Created `opt_prompt.md` to preserve the original platform request.
- Created `opt_output.md` with the full architecture and roadmap design.
- Renamed the folder from an architecture-study name to the full platform name.
- Updated internal references so docs refer to the project as the optimizer platform.
- Created `roadmap.md` with phased development plan, release milestones, and immediate next build target.
- Created `README.md`.
- Created `AGENT_README.md`.
- Created this progress tracker.

### 2026-06-20

- Documented the validated direct sparse SDF OML exporter as the optimizer-facing CFD STL export contract.
- Added `oml_export_contract.md`.
- Updated `README.md`, `roadmap.md`, `opt_output.md`, and `AGENT_README.md` to reference the OML export contract.
- Recorded `direct_sparse_oml_fast` as the default optimizer export path.
- Recorded `direct_sparse_oml_high_fidelity` as the selected-design rerun/finalization path.
- Confirmed shell/manufacturing export remains parked and should not enter the default optimizer flow.
- Validation basis: direct sparse exporter dry-run/API smoke tests passed in `dual_contouring\direct_sparse_sdf_mc_experiment`; prior baseline OML fast and high-fidelity exports were clean/watertight.
- Established `aircraft_optimizer_platform` as the master folder for the broader aircraft design/optimization platform.
- Added `master_platform_manifest.md` to track component inventory, current source-of-truth locations, intended master-folder layout, and import policy.
- Added `software/` placeholder component folders for manual SDF generation, automatic SDF generation, exporters, OML STL export, optimizer, dashboard, and analysis modules.
- Updated `README.md` and `AGENT_README.md` with master-folder integration rules.
- Recorded policy: wrap existing working code in place first; import or vendor code only after interface/provenance/validation status are clear.
- Created `dependency_inventory.md`.
- Initially copied too many manual Rhai/SDF inputs, then pruned that import back to a curated set.
- Current Rhai destination is `software\sdf_generation_manual\curated_rhai`.
- Current curated Rhai set contains 6 files: scratchpad plane, validated direct sparse no-inlet OML model, three inlet variants, and one parked shell reference.
- Broader Rhai history remains referenced in source folders instead of copied into the master folder.
- Added `software\sdf_generation_manual\curated_rhai\FEATURE_MAP.md` with returned feature names, roles, and embedded bounds for the curated Rhai references.
- Recorded that existing automatic Rhai generation scripts under `dual_contouring\direct_sparse_sdf_mc_experiment\scripts\make_*.py` are examples/references only, not production candidates.
- Re-centered architecture on optimizer/platform contracts rather than exporter details.
- Added `software\optimizer\contracts` with candidate, geometry provider, evaluation, artifact, module adapter, and platform interface overview contracts.
- Recorded that the exporter is a downstream module adapter, not the platform root.
- Added database schema, event log, and failure contracts under `software\optimizer\contracts`.
- Added `software\optimizer\v0_1_skeleton_spec.md` to define the first runnable platform slice with SQLite, records, events, a mock/manual geometry provider, mock module, and artifact registry.
- Added `environment_strategy.md` and recorded the v0.1 local-first decision: Python environment, SQLite, file-backed artifacts, pytest, and no Docker until real service/tool boundaries exist.
- Implemented the v0.1 Python optimizer skeleton under `software\optimizer`.
- Added `pyproject.toml`, SQLite `schema.sql`, database helpers, event logging, artifact registry, manual-reference geometry provider, mock module, CLI fixture command, and pytest smoke test.
- Ran `python -m pytest` in `software\optimizer`: 1 test passed.
- Ran the fixture CLI with `PYTHONPATH=src`: created `runs\v0_1_fixture\optimizer.db` and one copied Rhai artifact.
- Verified fixture database counts: campaigns=1, variable_schemas=1, candidates=1, evaluations=1, geometry_provider_results=1, artifacts=1, module_attempts=1, events=9, failures=0.
- Added `aircraft_optimizer_platform\.gitignore` to ignore generated run output and Python caches.
- Added standard-library typed record helpers for candidate seeds, variable values, module metrics, and failure drafts.
- Added static example JSON records under `software\optimizer\examples\records`.
- Added a read-only `summarize` CLI command for headless database inspection.
- Expanded the smoke test suite to cover fixture creation, example JSON parsing, candidate seed validation, and database summary output.
- Re-ran `python -m pytest` in `software\optimizer`: 4 tests passed.
- Re-ran fixture CLI and `summarize`; summary showed campaigns=1, variable_schemas=1, candidates=1, evaluations=1, geometry_provider_results=1, artifacts=1, module_attempts=1, events=9, failures=0.
- Added `software\exporters\oml_stl\export_adapter_contract.md`.
- Defined the OML STL adapter request/result contract, artifact kinds, metric keys, failure mapping, event sequence, integration rules, and phased implementation path.
- Updated exporter README, dependency inventory, and master manifest to point at the adapter contract.
- Confirmed status remains contract-only: exporter implementation is not imported or called by the platform skeleton.
- Reviewed the existing OML exporter wrapper JSON shape from `dual_contouring\direct_sparse_sdf_mc_experiment\scripts\optimizer_export_presets.py`.
- Added platform-side OML STL exporter result parser under `software\optimizer\src\aircraft_optimizer\exporters`.
- Added checked-in parser fixtures under `software\optimizer\examples\exporter_results` for passed export, quality-gate failure, and missing-summary failure.
- Added tests proving exporter wrapper JSON normalizes into module status, metrics, artifact references, and failure drafts without invoking the real exporter.
- Re-ran `python -m pytest` in `software\optimizer`: 7 tests passed.
- Added OML STL fixture adapter persistence through module attempts, artifact rows, metrics, failure rows, and events.
- Updated the v0.1 fixture so it records a successful `oml_stl_export` fixture module attempt without invoking the real exporter.
- Added a failed-fixture persistence test proving quality-gate failures are preserved as failure records.
- Expanded `summarize` output to include module statuses, artifact types, and failure categories.
- Re-ran `python -m pytest` in `software\optimizer`: 8 tests passed.
- Ran the fixture CLI with `PYTHONPATH=src`; the latest evaluation contains `mock_geometry_metrics:success`, `oml_stl_export:success`, 6 artifacts, and 0 failures.
- Added `software\sdf_generation_auto\geometry_generator_contract.md`.
- Defined the production automatic geometry-generator contract, including request/result schema, artifact requirements, initial fixed-wing variable groups, validation responsibilities, and failure mapping.
- Added `software\optimizer\schemas\fixed_wing_uav_reference.variables.v0_1.json`.
- Added standard-library variable schema loading and candidate validation helpers.
- Updated the v0.1 fixture to validate candidate variables against the versioned fixed-wing schema before persistence.
- Added schema validation tests for valid and out-of-bounds candidates.
- Added a no-execution fixed-wing fixture geometry generator that emits a generated Rhai source and parameter trace from the candidate/schema.
- Updated the v0.1 fixture to persist the automatic-fixture geometry provider result, generated Rhai artifact, and parameter trace artifact.
- Added candidate lineage and user annotation repository helpers.
- Updated the v0.1 fixture to persist seed lineage and a `fixture_baseline` annotation.
- Expanded summary output to include candidate lineage count, annotation count, and annotation tags.
- Re-ran `python -m pytest` in `software\optimizer`: 10 tests passed.
- Ran a clean temp fixture and summary; verified campaigns=1, candidates=1, candidate_lineage=1, user_annotations=1, evaluations=1, geometry_provider_results=1, artifacts=7, module_attempts=2, failures=0, events=19.
- Added a read-only `report-evaluation` CLI command and `evaluation_report` helper.
- The report includes evaluation, candidate variables, lineage, geometry-provider results, module attempts, artifacts, failures, and annotations for the latest or requested evaluation.
- Re-ran `python -m pytest` in `software\optimizer`: 11 tests passed.
- Ran a clean temp `report-evaluation` CLI smoke test and verified traceability records are emitted without raw SQL.
- Added a deterministic second candidate to the v0.1 fixture.
- The second candidate is generation 1, created by `deterministic_mutation`, and records the seed candidate as its parent.
- The child candidate receives its own evaluation, generated Rhai artifact, parameter trace artifact, mock module attempt, OML STL fixture adapter attempt, annotation, and lineage event.
- Added a read-only `report-campaign` CLI command and `campaign_report` helper.
- The campaign report lists campaign metadata, candidates, evaluations, lineage, and module status counts.
- Re-ran `python -m pytest` in `software\optimizer`: 12 tests passed.
- Ran a clean temp fixture plus `summarize`, `report-evaluation`, and `report-campaign` CLI smoke checks; verified candidates=2, evaluations=2, candidate_lineage=2, artifacts=14, module_attempts=4, failures=0, and parent-child lineage is visible.
- Added `fixture_scoring` placeholder scoring module.
- The fixture now records one scoring module attempt per candidate using a deterministic aspect-ratio proxy with low confidence metadata.
- Updated `report-campaign` so candidates include `score.fixture_total` and the report includes a ranking section.
- Re-ran `python -m pytest` in `software\optimizer`: 12 tests passed.
- Ran a clean temp fixture plus `summarize` and `report-campaign`; verified candidates=2, evaluations=2, candidate_lineage=2, artifacts=14, module_attempts=6, failures=0, events=41, and the child candidate ranks above the seed by placeholder score.
- Checked local tooling: winget is available, WSL2 Ubuntu 22.04 is available, Docker CLI exists but is old/unstable, and conda/mamba were not installed on Windows.
- Confirmed non-interactive sudo is not available in WSL, so apt-based OpenFOAM installation cannot be automated here.
- Installed micromamba 2.8.1 for Windows under `%LOCALAPPDATA%\AircraftOptimizerPlatform\micromamba`.
- Installed micromamba 2.8.1 inside WSL under `~/.local/aircraft_optimizer_platform/micromamba`.
- Created WSL micromamba env `aop-cfd` under `~/.local/aircraft_optimizer_platform/mamba_root/envs/aop-cfd`.
- Installed `python=3.11`, `su2=8.5.0`, `gmsh`, and `meshio` into `aop-cfd`.
- Verified `SU2_CFD --help` reports SU2 v8.5.0 "Harrier".
- Verified `gmsh -version` reports 4.15.2.
- Verified `meshio --help` works through the `meshio` console command.
- Added `tools\cfd\README.md` and `tools\cfd\bootstrap_wsl_aop_cfd.sh` to document and recreate the CFD tool environment.
- Added `software\analysis_modules\cfd_su2_adapter_contract.md`.
- Defined the future SU2 CFD adapter request/result contract, artifact expectations, failure mapping, and phased integration plan.
- Added `check-cfd-tools` CLI command for WSL `aop-cfd` tool detection.
- Verified `check-cfd-tools` returns available=true with SU2 v8.5.0, Gmsh 4.15.2, and meshio 5.3.5.
- Added checked-in SU2 history fixture and parser for CFD result metrics without running a solver.
- Added deterministic failed-export candidate fixture.
- The failed candidate records a failed OML export module attempt, a failure row, a failed evaluation, and a `fixture_failed_export` annotation.
- Added `--failed-only` filtering to `report-campaign`.
- Added `report-comparison` CLI command and candidate comparison report for the top two scored successful candidates.
- Added `su2_cfd_fixture` module adapter that persists parsed SU2 history fixture metrics and a `su2_history_csv` artifact without running SU2.
- Re-ran `python -m pytest` in `software\optimizer`: 15 tests passed.
- Verified clean fixture summary includes candidates=3, evaluations=3, candidate_lineage=3, artifacts=21, module_attempts=9, failures=1, and `su2_cfd_fixture:success`.
- Added `pipelines\v0_1_fixture.pipeline.json`.
- Added pipeline config loader/validator and wired the fixture campaign/evaluations to the versioned pipeline config.
- Added `environment_fingerprints` table and repository helper.
- Fixture evaluations now reference a shared environment fingerprint that records Python/platform metadata and notes that CFD tools are not checked during the fixture run.
- Re-ran `python -m pytest` in `software\optimizer`: 16 tests passed.
- Added `su2_case_dry_run` case-builder module.
- The dry-run case builder writes a generated SU2 config and case manifest, registers `su2_config_cfg` and `su2_case_manifest_json` artifacts, and records metrics proving no solver run occurred.
- Wired the dry-run SU2 case builder into the successful child fixture before the SU2 history fixture parser.
- Updated SU2 CFD adapter contract to mark tool detection, history parsing, and dry-run case generation as implemented skeleton capabilities.
- Updated `README.md` and `AGENT_README.md` with the current Phase 1 skeleton status.
- Re-ran `python -m pytest` in `software\optimizer`: 17 tests passed.
- Refactored v0.1 fixture orchestration out of `cli\main.py` into `software\optimizer\src\aircraft_optimizer\orchestration`.
- Added reusable helpers for candidate registration with lineage, evaluation start records, fixture geometry persistence, and successful module-attempt execution.
- Reduced `cli\main.py` from 1179 lines to 716 lines while preserving fixture behavior.
- Updated `pipelines\v0_1_fixture.pipeline.json` to include `su2_case_dry_run` explicitly.
- Ran CLI smoke check with `PYTHONPATH=src`: `run-fixture`, `summarize`, and `report-campaign --failed-only` succeeded; summary showed candidates=3, evaluations=3, artifacts=23, module_attempts=10, failures=1, and `su2_case_dry_run:success`.
- Re-ran `python -m pytest` in `software\optimizer`: 17 tests passed.
- Added request-driven single-candidate evaluator API through `CandidateEvaluationRequest` and `evaluate_fixture_candidate`.
- The evaluator validates candidate variables, records lineage, starts an evaluation, persists fixture geometry, runs fixture modules, preserves export failures, records annotations, and can include SU2 dry-run/history fixture modules.
- Added optimizer tracking tables: `optimizer_runs` and `optimizer_iterations`.
- Added repository helpers for optimizer run/iteration creation and completion.
- Wired the v0.1 fixture to record a deterministic optimizer run with three iterations: seed, deterministic mutation, and deterministic failure fixture.
- Added `report-optimizer` CLI/report output for optimizer run state, iteration rationale, proposed candidates, and ranking.
- Ran CLI smoke check with `PYTHONPATH=src`: `run-fixture`, `summarize`, and `report-optimizer` succeeded; summary showed optimizer_runs=1 and optimizer_iterations=3.
- Re-ran `python -m pytest` in `software\optimizer`: 19 tests passed.
- Added `optimizers\wing_options.py` with a deterministic five-candidate no-inlet wing option sweep.
- Added `run-wing-options` CLI command.
- The wing option sweep evaluates five span/chord/taper/sweep combinations through the request-driven evaluator, using fixture geometry/export/scoring only.
- Each option records a candidate, lineage event, evaluation, optimizer iteration, artifacts, module attempts, annotation, and ranking contribution.
- CLI smoke check with `PYTHONPATH=src`: `run-wing-options`, `summarize`, and `report-optimizer` succeeded.
- Smoke summary showed candidates=5, evaluations=5, optimizer_runs=1, optimizer_iterations=5, module_attempts=15, artifacts=35, failures=0, and `fixture_scoring:success` count=5.
- Current best wing option under placeholder fixture scoring was `long_high_sweep_taper`; this is not a real aero conclusion.
- Re-ran `python -m pytest` in `software\optimizer`: 20 tests passed.
- Added `low_fidelity_fixed_wing` screening module.
- The module estimates planform area, aspect ratio, taper ratio, Oswald efficiency, profile drag, induced drag, L/D, and `score.low_fidelity_total` from wing variables only.
- Wired `run-wing-options` to run `low_fidelity_fixed_wing` and rank by `score.low_fidelity_total` while still preserving `fixture_scoring` for continuity.
- Updated `report-optimizer` so optimizer reports rank by the optimizer run's declared objective metric instead of hard-coding `score.fixture_total`.
- Added `low_fidelity_fixed_wing` to `v0_1_fixture.pipeline.json` as an optional analytic screening module.
- CLI smoke check with `PYTHONPATH=src`: `run-wing-options`, `summarize`, and `report-optimizer` succeeded.
- Smoke summary showed candidates=5, evaluations=5, optimizer_runs=1, optimizer_iterations=5, module_attempts=20, artifacts=35, failures=0, and `low_fidelity_fixed_wing:success` count=5.
- Current best wing option under low-fidelity screening was `high_aspect_moderate_sweep`. This is still a screening estimate, not validated aerodynamics.
- Re-ran `python -m pytest` in `software\optimizer`: 21 tests passed.
- Added disabled-by-default real no-inlet geometry/export adapter boundary under `external\real_adapter_boundary.py`.
- The boundary resolves the curated no-inlet Rhai source, direct sparse exporter wrapper, SDF sidecar, feature name, preset, strict quality gates, result JSON path, working directory, and exact dry-run command.
- Added `preflight-real-adapters` CLI command.
- CLI preflight verified available=true, execution_enabled=false, execution_performed=false, missing_paths=[], preset=`direct_sparse_oml_fast`, feature=`aircraft_oml_oblique15_y_slice_frame`, and strict topology quality gates.
- Re-ran `python -m pytest` in `software\optimizer`: 22 tests passed.
- Updated platform direction to make sequential gated execution the primary optimizer workflow.
- Added `software\optimizer\contracts\sequential_gated_pipeline_contract.md`.
- Added `software\optimizer\pipelines\sequential_gated_v0_1.pipeline.json`.
- Added test coverage that the sequential pipeline config uses `candidate_batch_size=1`, places pre-export screening before export, and keeps real export/CFD disabled by default.
- Updated README, AGENT_README, roadmap, and optimizer README to clarify that batch sweeps are diagnostic/secondary tools, not the default autonomous export/CFD loop.
- Added `modules\pre_export_screening.py` with cheap planform, wing-loading, aspect-ratio, taper-ratio, sweep, and static-margin proxy checks.
- Added `optimizers\sequential_gated.py` implementing a one-candidate-at-a-time gated optimizer skeleton.
- Added `run-sequential-gated` CLI command.
- The sequential runner records one optimizer iteration per candidate, preserves failed pre-export candidates, and sends only passing candidates to fixture geometry/export and low-fidelity analysis.
- Added tests for pre-export failure classification and the full sequential gated runner.
- Smoke-ran `run-sequential-gated --iterations 3`, `summarize`, and `report-optimizer` with `PYTHONPATH=src`.
- Smoke result: 3 candidates, 3 evaluations, 3 optimizer iterations, 9 module attempts, 2 geometry provider results, 1 pre-export failure, 2 evaluated candidates, and best candidate selected by `score.low_fidelity_total`.
- Re-ran `python -m pytest` in `software\optimizer`: 25 tests passed.
- Added `optimizers\ask_tell.py` with a deterministic Halton ask/tell proposal policy over the active variable schema.
- Replaced the hardcoded sequential proposal list with the Halton ask/tell policy.
- Expanded pre-export screening with mean chord, minimum tip chord, span/root ratio, and export-risk score checks.
- Added `modules\virtual_components.py` with non-exported battery, payload, avionics, propulsion, fuselage shell, and wing structure mass/CG assumptions.
- Wired virtual component total mass, CG, and layout feasibility gates into `pre_export_screening`.
- Added `contracts\pre_export_gate_catalog.md` covering pre-definition, pre-export geometry-definition, post-export/pre-CFD, and pre-CFD analysis gates.
- Tuned the virtual component static-margin upper gate to avoid rejecting normal rough-estimate exploration while preserving bad-CG/static-margin rejection capability.
- Updated virtual component assumptions to mostly 3D-printed construction: printed wing shell/ribs/spar and printed fuselage shell/bulkheads.
- Re-ran `run-sequential-gated --iterations 5` after switching to printed-structure assumptions.
- Smoke result with printed virtual structure model: 5 candidates, 5 evaluations, 20 module attempts, 5 geometry provider results, 0 failures, and 5 evaluated candidates.
- Re-ran `python -m pytest` in `software\optimizer`: 27 tests passed.
- Added configurable virtual component assumptions for print density, rib spacing, spar allowance, bulkhead count, fuselage usable envelope, component masses, component dimensions, and target CG.
- Added smart internal layout placement for battery, avionics, and EDF within allowed x-ranges to reduce CG error.
- Added internal layout gates for fuselage fit feasibility, CG error, and remaining layout wiggle room.
- Added per-component inner-surface keepout assumptions, with custom mass/dimensions/keepout accepted through the virtual component assumptions override.
- Added tests for movable battery/avionics/EDF placement and oversized EDF rejection.
- Added tests for custom component mass, dimensions, and keepout, plus excessive keepout rejection.
- Re-ran `run-sequential-gated --iterations 5` with component keepout-aware layout gates active.
- Smoke result with keepout-aware layout gates: 5 candidates, 5 evaluations, 20 module attempts, 5 geometry provider results, 0 failures, and 5 evaluated candidates.
- Re-ran `python -m pytest` in `software\optimizer`: 31 tests passed.
- Added checked-in sample config: `software\optimizer\configs\virtual_components_printed_edf_sample.v0_1.json`.
- The sample config defines a 70 mm diameter, 120 mm long, 300 g EDF; 3.0 in x 1.5 in x 1.5 in, 300 g battery; and 1.5 in x 1.5 in x 1.0 in, 175 g electronics package.
- Added `run-sequential-gated --virtual-components-config <path>`.
- Sample one-candidate run passed pre-export screening: estimated mass `1.767421 kg`, CG `x=341.05 mm`, CG error `0.0 mm`, static margin deferred, layout feasible, remaining layout wiggle room `288.7 mm`, EDF centered on fuselage axis with `y=0`, `z=0`.
- Re-ran `python -m pytest` in `software\optimizer`: 32 tests passed.
- Added `report-layout` CLI command and `layout_report` database report.
- `report-layout` returns candidate variables, pre-export status, failed checks, mass properties, target CG, static margin availability, layout feasibility/issues, assumptions, and component placement.
- Smoke-ran `report-layout` after a one-candidate sample EDF run; output showed feasible layout, no failed checks, estimated mass `1.767421 kg`, CG `x=341.05 mm`, static margin deferred, layout wiggle room `288.7 mm`, and EDF at `y=0`, `z=0`.
- Re-ran `python -m pytest` in `software\optimizer`: 33 tests passed.
- Updated virtual layout policy so EDF is fixed in x/y/z and does not participate in CG correction.
- Forced internal battery, payload, avionics, and EDF placement onto the aircraft X axis (`y=0`, `z=0`) for the current simplified layout model.
- Removed pre-export static margin calculation/gating. Static margin is deferred until CFD/stability neutral point analysis exists.
- Layout metadata records initial CG, target CG, CG shift, final CG error, `static_margin_available=false`, and `static_margin_deferred_until=cfd_neutral_point_analysis`.
- Sample EDF report after the update: initial CG `x=331.620267 mm`, final CG `x=341.05 mm`, CG shift `9.429733 mm`, CG error `0.0 mm`, static margin `null`, EDF fixed at `x=232.5 mm`, `y=0`, `z=0`.
- Re-ran `python -m pytest` in `software\optimizer`: 33 tests passed.
- Re-ran `run-sequential-gated --iterations 5` with smart layout gates active.
- Smoke result with smart layout gates: 5 candidates, 5 evaluations, 20 module attempts, 5 geometry provider results, 0 failures, and 5 evaluated candidates.
- Re-ran `python -m pytest` in `software\optimizer`: 29 tests passed.
- Smoke-ran `run-sequential-gated --iterations 5`, `summarize`, and `report-optimizer` with `PYTHONPATH=src`.
- Smoke result: 5 candidates, 5 evaluations, 5 optimizer iterations, 20 module attempts, 5 geometry provider results, 0 failures, 5 evaluated candidates, and best candidate selected by `score.low_fidelity_total`.
- Re-ran `run-sequential-gated --iterations 5` after adding virtual component gates.
- Smoke result with virtual mass/CG gates: 5 candidates, 5 evaluations, 20 module attempts, 5 geometry provider results, 0 failures, and 5 evaluated candidates.
- Re-ran `python -m pytest` in `software\optimizer`: 27 tests passed.
- Added real OML STL execution adapter under `modules\oml_stl_real_adapter.py`.
- Added one-shot `run-real-no-inlet-export` CLI command.
- The command creates a campaign, candidate, lineage record, evaluation, manual no-inlet geometry provider result, real `oml_stl_export` module attempt, artifact records, and annotation, then stops before analysis/scoring/CFD.
- Ran real no-inlet export once with `direct_sparse_oml_fast`.
- Result: module_status=success, failures=0, analysis_modules_run=false.
- Runtime: exporter `runtime_total_s=270.5013739001006`, wrapper runtime `272.91736480011605` seconds.
- Mesh/export metrics: boundary_edges=0, nonmanifold_edges=0, connected_components=1, duplicate_triangles=0, long_chord_sections_ge_75mm=0, vertices=573624, triangles=1147244, sdf_sample_count=33401460, active_tile_count=6720.
- Platform artifact store copied the STL, export result JSON, stdout log, stderr log, and geometry Rhai source into the run workspace.
- Real STL artifact path from smoke run: `C:\Users\Jackson\AppData\Local\Temp\aop_real_no_inlet_0d3986b2bac441d6979924ea62bc077b\campaigns\real_no_inlet_export\artifacts\artifact_11fb54c360f24c49a0ad058b74cf5da1_direct_sdf_oml_spacing_1p0.stl`.
- Re-ran `python -m pytest` in `software\optimizer`: 22 tests passed.

## 2026-06-21 - Post-CFD Layout Contract, Geometry Screen, and CFD Readiness Gate

- Added `software\optimizer\contracts\post_cfd_stability_layout_contract.md`.
- Added `modules\stability_layout_adjustment.py` for post-CFD neutral-point/static-margin layout adjustment.
- Static margin remains deferred before CFD; the post-CFD path shifts only movable internal components and keeps the EDF fixed.
- Added `modules\geometry_definition_screening.py`.
- Wired geometry-definition screening into the sequential evaluator after geometry provider output and before OML STL export.
- Geometry-definition screening currently checks required feature identity, bounding-box size, estimated mesh-cell budget, narrow-feature risk, and virtual-layout feasibility metadata.
- Added `modules\cfd_readiness.py` for parsed OML STL export-result pre-CFD screening.
- Added `check-cfd-readiness --export-result <path>` CLI command.
- CFD-readiness fixture smoke accepted `examples\exporter_results\oml_stl_passed.json`.
- CFD-readiness fixture smoke rejected `examples\exporter_results\oml_stl_failed_quality_gate.json` on `max_boundary_edges` and `max_nonmanifold_edges`.
- Re-ran `run-sequential-gated --iterations 3` with the sample 70 mm EDF component config: 3 candidates, 3 evaluations, 15 module attempts, 3 geometry provider results, 0 failures.
- Re-ran `python -m pytest` in `software\optimizer`: 39 tests passed.

## 2026-06-21 - CFD Mesh Export Contract

- Added `software\optimizer\contracts\mesh_export_contract.md`.
- Contract distinguishes OML STL surface artifacts from solver-ready CFD volume meshes.
- Accepted first mesh formats: native SU2 `.su2` and CGNS `.cgns`.
- Required platform semantic regions: `fluid`, `aircraft`, and `farfield`.
- Added marker-map policy: keep stable platform names, but allow adapters to map to solver-visible names such as CGNS sections `3_S_7`.
- Added minimum volume-mesh acceptance gates: positive nodes, positive volume cells, aircraft faces, farfield faces, and a rendered solver config using the marker map.
- Recorded sandbox evidence: sphere STL worked through Gmsh/SU2, simple manifold wing STLs failed volume meshing, native Gmsh/OCC wing worked, and CGNS worked only when using solver-visible section names.

## 2026-06-21 - Mesh Result Validator

- Added `software\optimizer\src\aircraft_optimizer\modules\mesh_result_validation.py`.
- Added `check-mesh-result` CLI command for native SU2 `.su2` meshes and CGNS `.cgns` meshes read through SU2 preprocessing logs.
- Native SU2 validation now records `fluid` as `implicit_single_volume` when the mesh has positive volume cells.
- CGNS validation requires explicit marker mapping from platform semantics to solver-visible section names, for example `aircraft=3_S_7`, `farfield=3_S_8..3_S_13`, and `fluid=5_V_1`.
- Added tests for native SU2 marker inference, CGNS section-name mapping, and empty-mesh rejection.
- Added `software\optimizer\src\aircraft_optimizer\modules\mesh_result_adapter.py` so mesh validation can persist as a normal database module attempt.
- Mesh validation persistence writes a generated `cfd_mesh_result_json` artifact and references the input `cfd_volume_mesh` plus optional `su2_preprocessing_log`.
- Smoke-ran `check-mesh-result` against `su2_sandbox\runs\occ_lenticular_wing_50mph\external_flow.su2`: passed with 555 nodes, 2305 volume cells, 116 aircraft faces, and 540 farfield faces.
- Smoke-ran `check-mesh-result` against `su2_sandbox\runs\occ_lenticular_wing_50mph_cgns\external_flow.cgns` plus `su2_cgns_sections_stdout.log`: passed with the explicit `3_S_*`/`5_V_1` marker map.
- Re-ran `python -m pytest` in `software\optimizer`: 43 tests passed.

## 2026-06-21 - Full-Aircraft SU2 Retry

- Confirmed native SU2 and CGNS mesh formats both work for the known-good native Gmsh/OCC lenticular wing case.
- Retried CFD setup on the full no-inlet aircraft STL from `su2_sandbox\examples\basic_no_inlet_oml.stl`.
- Surface topology check still passed: `boundary_edges=0`, `nonmanifold_edges=0`, `degenerate_triangles=0`, `triangles=1147244`.
- Parametric Gmsh path failed in `su2_sandbox\runs\basic_no_inlet_50mph_retry_parametric` with `Wrong topology of boundary mesh for parametrization`.
- Discrete Gmsh path failed in `su2_sandbox\runs\basic_no_inlet_50mph_retry_discrete` with `Invalid boundary mesh (overlapping facets) on surface 2 surface 374`.
- Added `--angle-tolerance-facet-overlap` to `su2_sandbox\scripts\make_external_flow_mesh.py` and retried at `0.001` in `su2_sandbox\runs\basic_no_inlet_50mph_retry_discrete_tol0001`.
- Lowered tolerance got past the first overlap check but failed boundary recovery with `PLC Error: A segment and a facet intersect at point`.
- No full-aircraft native SU2 or CGNS volume mesh was produced, so SU2 CFD was not run on the full aircraft.
- Current conclusion: the blocker is full-aircraft CFD surface/volume meshing, not SU2 execution or marker naming.

## 2026-06-21 - Full-Aircraft Coarse CFD Smoke

- Added `su2_sandbox\scripts\repair_aircraft_stl.py` for PyMeshLab-based screened Poisson and isotropic repair experiments.
- Added `su2_sandbox\scripts\make_voxel_external_flow_su2.py` for a coarse Cartesian/hexahedral fallback SU2 mesh generated directly from a watertight repaired STL.
- Installed sandbox-only Python packages in `su2_sandbox\.venv`: `trimesh`, `scipy`, `scikit-image`, `pymeshlab`, and `rtree`.
- Repaired `su2_sandbox\examples\basic_no_inlet_oml.stl` with screened Poisson depth 8.
- Repair result: triangle count reduced from `1147244` to `126876`, `boundary_edges=0`, `nonmanifold_edges=0`, `degenerate_triangles=0`, one watertight connected component, and volume close to original.
- Gmsh still did not produce positive tetrahedra from the repaired surface; discrete Gmsh wrote `NELEM=0`, `NPOIN=0`, and the parametric path timed out.
- Generated fallback full-aircraft voxel mesh in `su2_sandbox\runs\basic_no_inlet_50mph_voxel_40mm_compact`.
- Mesh result: `31072` nodes, `28015` hexahedral volume cells, `334` aircraft faces, `5670` farfield faces.
- `check-mesh-result` passed on the voxel SU2 mesh.
- Ran SU2 at 50 mph on the full-aircraft voxel mesh; SU2 exited successfully and converged in 11 iterations.
- Outputs written: `history.csv`, `flow.vtu`, `surface_flow.vtu`, and `restart_flow.dat`.
- Final screen values at iteration 10: `rms[P] = -6.609795`, `CL = 0.143943`, `CD = -0.032322`.
- Interpretation: this is a solver-path smoke result only; the coarse voxel wall is not acceptable for production aerodynamic scoring.

## 2026-06-21 - Full-Aircraft CGNS Check

- Tested whether CGNS improved the full-aircraft coarse voxel mesh path.
- Installed sandbox-only `h5py` so MeshIO could write CGNS.
- MeshIO converted `su2_sandbox\runs\basic_no_inlet_50mph_voxel_40mm_compact\external_flow.su2` to `su2_sandbox\runs\basic_no_inlet_50mph_voxel_40mm_cgns\external_flow.cgns`, but the file was only `31636` bytes and MeshIO could not read it back.
- SU2 rejected the converted CGNS file with: `external_flow.cgns was not found or is not a properly formatted CGNS file. Note that SU2 expects unstructured CGNS files in ADF data format.`
- Current conclusion: CGNS is viable when generated by the Gmsh/OCC path, but it does not improve the current full-aircraft voxel fallback. Native `.su2` remains the working full-aircraft path.

## 2026-06-21 - OpenFOAM snappyHexMesh Exploration

- Confirmed OpenFOAM 13 is installed under `/opt/openfoam13`; tools available after `source /opt/openfoam13/etc/bashrc`.
- Added `su2_sandbox\scripts\make_openfoam_snappy_case.py`.
- Generated case `su2_sandbox\runs\openfoam_basic_no_inlet_snappy_d8` from the screened-Poisson repaired full-aircraft STL.
- OpenFOAM `surfaceCheck` on the original full-aircraft STL reported closed surface, one connected part, no illegal triangles, but extremely poor triangle quality with minimum quality `7.50816e-20`.
- OpenFOAM `surfaceCheck` on the repaired STL reported closed surface, one connected part, no illegal triangles, 14 nearby/small-edge point pairs, and low-quality triangles.
- Ran `blockMesh`, `surfaceFeatures`, `snappyHexMesh -overwrite`, `checkMesh`, and `foamToVTK -constant`.
- `snappyHexMesh` finished without errors.
- `checkMesh` passed with `Mesh OK`.
- Resulting OpenFOAM mesh: `35977` cells, `46339` points, patches `farfield` and `aircraft`.
- Aircraft patch has `4669` faces; farfield patch has `3584` faces.
- Cell mix: `30172` hexahedra, `399` prisms, `5393` polyhedra, and `13` tet wedges.
- Mesh quality: max non-orthogonality `67.594183`, average non-orthogonality `10.665788`, max skewness `3.2752337`.
- VTK outputs written under `su2_sandbox\runs\openfoam_basic_no_inlet_snappy_d8\VTK`.
- Current conclusion: snappyHexMesh is a better near-term path than Gmsh for repaired STL-based full-aircraft meshing. The original STL still has serious triangle-quality defects, but the repaired STL is usable for OpenFOAM cut-cell/polyhedral meshing.

## 2026-06-21 - OpenFOAM Full-Aircraft Potential Solver Smoke

- Added `su2_sandbox\scripts\write_openfoam_potential_case.py`.
- Generated first solver inputs for `su2_sandbox\runs\openfoam_basic_no_inlet_snappy_d8` at `50 mph` (`22.352 m/s`) in +X.
- Boundary setup: `farfield` uses fixed freestream velocity and pressure; `aircraft` uses inviscid `slip` velocity and `zeroGradient` pressure.
- Ran `potentialFoam -writep -writePhi`; OpenFOAM exited successfully.
- Potential solve final continuity error was `3.1940588e-05`; interpolated velocity error was `0.00068514459`.
- OpenFOAM wrote `U`, `p`, and `phi` fields under `su2_sandbox\runs\openfoam_basic_no_inlet_snappy_d8\0`.
- Refreshed VTK outputs with `foamToVTK`.
- Interpretation: this proves the repaired STL/snappy mesh can support a full-aircraft OpenFOAM solver run. It is inviscid solver-path evidence, not validated aerodynamic scoring or drag prediction.
- Current near-term CFD direction: use OpenFOAM/snappyHexMesh for full-aircraft sandbox integration experiments; keep SU2 for known-good marker/mesh contract tests and as a future solver option once surface/volume meshing is cleaner.

## 2026-06-21 - OpenFOAM Contract Gates and Mutated SDF Full Path

- Added optimizer module `software\optimizer\src\aircraft_optimizer\modules\cfd_surface_quality.py`.
- Added optimizer module `software\optimizer\src\aircraft_optimizer\modules\openfoam_result_validation.py`.
- Added CLI commands:
  - `check-cfd-surface`
  - `check-openfoam-result`
- Added contract doc `software\analysis_modules\cfd_openfoam_adapter_contract.md`.
- Added unit coverage for direct STL surface gates and OpenFOAM log validation.
- Re-ran optimizer tests: `47 passed`.
- Created candidate SDF variant `su2_sandbox\examples\basic_no_inlet_wing_span_plus20.rhai`.
- Candidate mutation: `wing_span` changed from `700.0 mm` to `720.0 mm`.
- Ran real direct sparse OML export using `direct_sparse_oml_fast` and the mutated SDF.
- Export artifact: `dual_contouring\direct_sparse_sdf_mc_experiment\stl\basic_no_inlet_wing_span_plus20_spacing_1p0.stl`.
- Export runtime: `336.79527850006707 s`.
- Export metrics: `1,157,918` triangles, `578,965` vertices, `10` boundary edges, `0` nonmanifold edges, `1` connected component, `0` duplicate triangles, and `0` long-chord sections >= `75 mm`.
- Strict raw export gate failed because `boundary_edges=10`; this was preserved in `su2_sandbox\runs\basic_no_inlet_wing_span_plus20_openfoam\export_result.json` and `raw_surface_quality.json`.
- Repaired the raw STL with screened Poisson depth 8.
- Repair artifact: `su2_sandbox\runs\basic_no_inlet_wing_span_plus20_openfoam\aircraft_poisson_d8.stl`.
- Repair reduced the surface to `121,602` faces and `60,803` vertices.
- Repaired surface gate: `boundary_edges=0`, `nonmanifold_edges=0`, `duplicate_triangles=0`, `degenerate_triangles=0`; strict tiny-edge check still warned with `min_edge_length_mm=5.8228827e-05`.
- Generated OpenFOAM case `su2_sandbox\runs\basic_no_inlet_wing_span_plus20_openfoam\case`.
- Ran `blockMesh`, `surfaceFeatures`, `snappyHexMesh -overwrite`, `checkMesh`, `potentialFoam -writep -writePhi`, and `foamToVTK`.
- `checkMesh` passed with `Mesh OK`.
- Mesh result: `36,314` cells, `46,776` points, `4,754` aircraft faces, `3,584` farfield faces, max non-orthogonality `67.793763`, max skewness `2.9661243`.
- `potentialFoam` result: continuity error `2.9203639e-05`, interpolated velocity error `0.00068384544`.
- Platform OpenFOAM validator passed with no failed checks; result saved to `su2_sandbox\runs\basic_no_inlet_wing_span_plus20_openfoam\openfoam_result_validation.json`.
- VTK output: `su2_sandbox\runs\basic_no_inlet_wing_span_plus20_openfoam\case\VTK\case_0.vtk`.
- Interpretation: the platform now has a working sandbox chain for one modified SDF candidate from real OML export through OpenFOAM potential-flow smoke. Raw export quality still needs improvement before CFD can be fully policy-gated without repair.

## 2026-06-21 - Mesh Fidelity Rejection and Refined snappy Experiments

- User inspected the first OpenFOAM VTK output and rejected it as simulation-quality mesh: too faceted, poor wing blend capture, and poor leading/trailing-edge fidelity.
- Root cause: the accepted smoke path used screened-Poisson surface repair, which globally smoothed/decimated the raw OML, and the snappy case only produced `4,754` aircraft patch faces.
- Parameterized `su2_sandbox\scripts\make_openfoam_snappy_case.py` with base-cell counts, surface feature angle, feature refinement level, surface min/max refinement levels, max cell limits, and optional layer switch.
- Generated high-refinement raw-STL case `su2_sandbox\runs\basic_no_inlet_wing_span_plus20_openfoam\case_raw_refined_l4_l5`.
- Raw high-refinement result: `607,826` cells, `127,437` aircraft patch faces, but `checkMesh` failed with max skewness `5.8468523` and 4 highly skew faces.
- Exported VTK for visual inspection: `case_raw_refined_l4_l5\VTK\aircraft\aircraft_0.vtk`.
- Added `su2_sandbox\scripts\render_aircraft_iso_screenshot.py` using ParaView `pvpython`.
- Generated aircraft-only ISO screenshot `case_raw_refined_l4_l5\aircraft_iso.png` from the `aircraft` patch VTK, not the farfield/volume mesh.
- Generated middle-ground raw-STL case `case_raw_refined_l3_l4`.
- Middle-ground result: `292,213` cells, `45,687` aircraft patch faces, but `checkMesh` failed with max skewness `4.2781628` and 3 highly skew faces.
- Added `su2_sandbox\scripts\close_small_stl_holes.py` for feature-preserving small-loop closure experiments.
- Local cap repaired the raw export's single 10-edge boundary loop and preserved the high-resolution surface, but the refined local-cap mesh still failed `checkMesh` with similar skewness.
- Attempted low-quality triangle removal plus bounded local capping; this created additional tiny cap triangles and is not accepted as an improvement.
- Current conclusion: Poisson repair is not acceptable for simulation-quality geometry, and raw snappy refinement improves visual fidelity but exposes STL sliver/triangle-quality defects. The next real fix must improve CFD surface export/feature-preserving remeshing before force/scoring CFD.
- Forward rule: future mesh runs should attach or reference an aircraft-only ISO screenshot so human review can judge geometry capture alongside automated gates.

## 2026-06-21 - Current OpenFOAM Option Optimization

- Added `su2_sandbox\scripts\summarize_openfoam_mesh_cases.py` to parse OpenFOAM case reports, `checkMesh` logs, optional skew-face VTK files, and aircraft-only screenshot paths into CSV/Markdown summaries.
- Wrote current summaries to `su2_sandbox\runs\basic_no_inlet_wing_span_plus20_openfoam\mesh_case_summary.csv` and `mesh_case_summary.md`.
- Ran `checkMesh -writeSets -writeSurfaces -setFormat vtk -surfaceFormat vtk` on the current near-pass case `case_local_cap_l3_l4_smooth_t1`.
- OpenFOAM localized its only hard failure to `2` skew faces at approximate centroid `x=0.61765`, `y=-0.137837`, `z=0.165003` meters.
- Generated aircraft-only ISO screenshot `case_local_cap_l3_l4_smooth_t1\aircraft_iso_surface.png` from the `aircraft` patch only.
- Tested targeted tuning case `case_local_cap_l3_l4_t05_noimplicit`: local-cap source, l3/l4 surface refinement, snap tolerance `0.5`, `nSmoothPatch=8`, `nSolveIter=200`, `nRelaxIter=10`, `nFeatureSnapIter=20`, implicit feature snap disabled.
- Rejected that tuning branch: `checkMesh` still failed with `2` skew faces, max skewness worsened from `4.5113177` to `4.6455738`, and max non-orthogonality worsened from `66.763739` to `74.987393` with `2` severely non-orthogonal faces reported.
- Generated aircraft-only ISO screenshot `case_local_cap_l3_l4_t05_noimplicit\aircraft_iso_surface.png`.
- Current best current-option mesh remains `case_local_cap_l3_l4_smooth_t1`: it preserves far more geometry than the Poisson smoke mesh and fails only on two localized skew faces, but it is not CFD-ready yet.
- Current optimization conclusion: broad snappy tuning is running out of value. The next useful work is targeted defect localization and source-surface/mesh interaction cleanup around the failing regions.

## 2026-06-21 - Current-Options Limit and New-Vehicle Pipeline Replay

- Added `su2_sandbox\scripts\diagnose_openfoam_defect_neighborhood.py` to map OpenFOAM written failure surfaces such as `skewFaces.vtk` back to nearby source STL triangle neighborhoods.
- Added `su2_sandbox\scripts\weld_short_stl_edges.py` for conservative source-surface short-edge welding experiments.
- Diagnosed the best current mesh `case_local_cap_l3_l4_smooth_t1`: OpenFOAM skew faces localize near `x=0.61765`, `y=-0.137837`, `z=0.165003` meters. Nearby source STL neighborhoods include tiny edges around `0.0106 mm` and low local triangle quality, but the defect persists after conservative source cleanup.
- Tested OpenFOAM `surfaceClean`; rejected because it aborted during triangle collapse on the full aircraft surface.
- Tested OpenFOAM `surfaceLambdaMuSmooth`; rejected because the output surface was no longer closed and lost major aircraft extent.
- Tested conservative `0.02 mm` source edge weld. It stayed watertight and removed `3,065` tiny edges, but the resulting snappy mesh still failed default `checkMesh` with `2` skew faces and max skewness `4.5158711`.
- Tested default snap, no-feature snap, stricter snappy skew thresholds, and snap tolerance `2.0`; all failed to produce a default-clean high-fidelity mesh. No-feature snap was significantly worse with max skewness `11.07659`.
- Verified the best near-pass current mesh is solver-runnable: `potentialFoam -writep -writePhi` completed on `case_local_cap_l3_l4_smooth_t1` at `50 mph` with continuity error `4.8306887e-05` and interpolated velocity error `0.00011907502`.
- Created new continuous-parameter vehicle `su2_sandbox\examples\basic_no_inlet_root180_tip70.rhai` with `wing_root_chord=180.0` and `wing_tip_chord=70.0`.
- Ran the full new-vehicle path: real direct sparse OML export, bounded local cap, snappyHexMesh, checkMesh, aircraft-only screenshot, and `potentialFoam`.
- New vehicle export metrics: runtime `333.33091029990464 s`, `1,174,180` triangles, `587,104` vertices, `26` boundary edges before local cap, `0` non-manifold edges, `1` connected component, and `0` duplicate triangles.
- New vehicle local cap repaired one `26`-edge loop and produced `0` boundary edges and `0` non-manifold edges.
- New vehicle snappy result: `437,027` cells, `49,189` aircraft patch faces, max non-orthogonality `64.594848`, max skewness `4.5113068`, and `2` highly skew faces. Default `checkMesh` still failed one check.
- New vehicle `potentialFoam` completed at `50 mph` with continuity error `4.6055185e-05` and interpolated velocity error `0.00011906816`.
- Aircraft-only ISO screenshot for the new vehicle: `su2_sandbox\runs\basic_no_inlet_root180_tip70_openfoam\case_local_cap_l3_l4_smooth_t1\aircraft_iso_surface.png`.
- Current conclusion: the OpenFOAM/snappy current-options path is solver-runnable and repeatable, but not default `checkMesh` clean at simulation-quality visual fidelity. The blocker is localized geometry/mesh interaction at sharp aircraft features, not global topology, marker naming, or solver setup.

## 2026-06-22 - Relaxed OpenFOAM Development Acceptance and Surface Prep

- Added explicit relaxed-development OpenFOAM policy in `software\optimizer\src\aircraft_optimizer\modules\openfoam_result_validation.py`.
- Relaxed-development policy keeps non-orthogonality, solver residual, patch-count, and cell-count gates, but allows max skewness up to `6.0` and does not require the default strict `Mesh OK` log.
- Added regression coverage in `software\optimizer\tests\test_v0_1_flow.py`: strict policy rejects max skewness `4.5113177`, relaxed-development policy accepts the same solver-runnable case.
- Validation command: `py -3.12 -m pytest software\optimizer\tests\test_v0_1_flow.py -k openfoam_result_validation -q`; result `3 passed, 45 deselected`.
- Added `su2_sandbox\scripts\prepare_cfd_surface.py` as the pipeline-facing raw STL to CFD-surface prep step.
- Updated `su2_sandbox\scripts\close_small_stl_holes.py` with cap methods `fan`, `edge_fan`, and `delaunay`; `edge_fan` avoids adding an artificial center vertex and preserves all boundary edges.
- Added relaxed acceptance visibility to `su2_sandbox\scripts\summarize_openfoam_mesh_cases.py` through the `relaxed_skew6_ok` column.
- Ran `checkMesh -skewThreshold 6` on the current best case and the new root180/tip70 case. Both reported `Mesh OK` under relaxed skew policy.
- Generated prepared edge-fan CFD surface `su2_sandbox\runs\basic_no_inlet_wing_span_plus20_openfoam\aircraft_prepared_edgefan_cap.stl`.
- Edge-fan prepared surface result: `0` boundary edges, `0` non-manifold edges, watertight, `1,157,926` faces, `578,965` vertices.
- Meshed prepared edge-fan surface as `case_prepared_edgefan_l3_l4_smooth_t1`.
- Strict `checkMesh` still failed with max skewness `4.5113177` and `2` highly skew faces.
- Relaxed `checkMesh -skewThreshold 6` passed with `Mesh OK`.
- `potentialFoam` completed on `case_prepared_edgefan_l3_l4_smooth_t1`; continuity error `4.8714215e-05`, interpolated velocity error `0.00011907502`.
- Wrote machine-readable acceptance result to `case_prepared_edgefan_l3_l4_smooth_t1\openfoam_acceptance_validation.json`: strict failed on `mesh_ok` and `max_skewness`; relaxed-development passed.
- Wrote equivalent acceptance result for the new vehicle to `basic_no_inlet_root180_tip70_openfoam\case_local_cap_l3_l4_smooth_t1\openfoam_acceptance_validation.json`.
- Current decision: use relaxed-development OpenFOAM acceptance for smoke CFD and optimizer plumbing, but keep strict default `checkMesh` as the future scoring-CFD standard.

## 2026-06-22 - OpenFOAM Smoke Persistence in Optimizer

- Added optimizer module `software\optimizer\src\aircraft_optimizer\modules\openfoam_smoke_adapter.py`.
- Added CLI command `persist-openfoam-smoke`.
- Wired `openfoam_smoke_validation` into `software\optimizer\pipelines\sequential_gated_v0_1.pipeline.json` as an optional relaxed-development non-scoring analysis mode.
- The persistence adapter records an already-run OpenFOAM case; it does not launch OpenFOAM.
- Persisted metadata includes `acceptance_mode`, `analysis_role=solver_smoke_non_scoring`, `scoring_allowed=false`, the parsed OpenFOAM validation result, and registered artifact ids.
- Persisted artifacts include the case-directory reference, validation JSON, checkMesh log, solver log, optional strict/relaxed checkMesh logs, and optional aircraft-only ISO screenshot.
- Added regression coverage in `software\optimizer\tests\test_v0_1_flow.py` for relaxed-development persistence.
- Validation command: `py -3.12 -m pytest software\optimizer\tests\test_v0_1_flow.py -k "openfoam or sequential_gated_pipeline_config" -q`; result `5 passed, 44 deselected`.
- Compile validation: `py -3.12 -m py_compile software\optimizer\src\aircraft_optimizer\modules\openfoam_smoke_adapter.py software\optimizer\src\aircraft_optimizer\modules\openfoam_result_validation.py software\optimizer\src\aircraft_optimizer\cli\main.py`; passed.
- JSON validation: `py -3.12 -m json.tool software\optimizer\pipelines\sequential_gated_v0_1.pipeline.json`; passed.
- CLI proof workspace: `aircraft_optimizer_platform\runs\openfoam_smoke_plumbing`.
- CLI proof persisted sandbox case `su2_sandbox\runs\basic_no_inlet_wing_span_plus20_openfoam\case_prepared_edgefan_l3_l4_smooth_t1` with `acceptance_mode=relaxed_development`.
- CLI proof result: module attempt `module_6219ab637322495c9147879bd6f1e149` completed successfully for evaluation `evaluation_ed962958fd954b9da8ec5631a9344b0d`.
- Persisted proof metrics included `openfoam.ready=1`, `openfoam.mesh_ok=1`, `openfoam.cells=438245`, `openfoam.aircraft_faces=49267`, `openfoam.max_non_orthogonality=66.763739`, `openfoam.max_skewness=4.5113177`, `openfoam.continuity_error=4.8714215e-05`, and `openfoam.interpolated_velocity_error=0.000119075`.
- Current decision: relaxed-development OpenFOAM smoke can now be traced through the optimizer database, but it remains solver-plumbing evidence only and must not be used for scoring.

## 2026-06-22 - Strict snappy Pass via Local STL Feature Conditioning

- Forked the export-quality workstream: current direct sparse exporter remains intact; experiments happen as post-export/pre-mesh STL conditioning under `su2_sandbox`.
- Added `su2_sandbox\scripts\local_smooth_stl_patch.py` to apply a bounded local Laplacian smoothing patch around a known bad mesh region.
- Integrated optional local smoothing into `su2_sandbox\scripts\prepare_cfd_surface.py` so raw exporter STL can be conditioned in one repeatable step.
- Conditioning recipe that passed strict snappy:
  - Start from raw exported STL.
  - Repair small open loop with `prepare_cfd_surface.py --cap-method edge_fan`.
  - Apply local smoothing centered at `617.65,-137.837,165.003 mm`.
  - Radius `8 mm`.
  - Iterations `4`.
  - Relaxation `0.12`.
- Current aircraft local smoothing report: `287` selected vertices, max accumulated displacement `0.2800435654747079 mm`, p95 accumulated displacement `0.21766132983756933 mm`.
- Current aircraft strict-pass case: `su2_sandbox\runs\basic_no_inlet_wing_span_plus20_openfoam\case_local_smooth_r8_i4_l3_l4_smooth_t1`.
- Current aircraft strict `checkMesh`: `Mesh OK`, `438,245` cells, `49,266` aircraft faces, max non-orthogonality `66.763739`, max skewness `3.8155389`.
- Current aircraft `potentialFoam`: completed at `50 mph`; continuity error `2.4541523e-05`, interpolated velocity error `0.00011907507`.
- Current aircraft strict acceptance JSON: `case_local_smooth_r8_i4_l3_l4_smooth_t1\openfoam_strict_acceptance_validation.json`.
- Current aircraft aircraft-only ISO screenshot: `case_local_smooth_r8_i4_l3_l4_smooth_t1\aircraft_iso_surface.png`.
- Replayed the same conditioning recipe on the new `basic_no_inlet_root180_tip70` vehicle.
- New vehicle local smoothing report: `287` selected vertices, max accumulated displacement `0.2800435654747079 mm`, p95 accumulated displacement `0.21766132983756933 mm`.
- New vehicle strict-pass case: `su2_sandbox\runs\basic_no_inlet_root180_tip70_openfoam\case_local_smooth_r8_i4_l3_l4_smooth_t1`.
- New vehicle strict `checkMesh`: `Mesh OK`, `437,027` cells, `49,190` aircraft faces, max non-orthogonality `64.594853`, max skewness `3.5688954`.
- New vehicle `potentialFoam`: completed at `50 mph`; continuity error `2.4571642e-05`, interpolated velocity error `0.00011906823`.
- New vehicle strict acceptance JSON: `case_local_smooth_r8_i4_l3_l4_smooth_t1\openfoam_strict_acceptance_validation.json`.
- Current interpretation: the repeated strict snappy failure was caused by a small local sharp-feature/triangle arrangement, not by the whole STL or OpenFOAM setup. A sub-0.3 mm local STL conditioning patch fixes strict `checkMesh` on two vehicle variants.
- Risk: the smoothing center is currently hard-coded from known failure localization. Generalization should detect bad STL regions or map failed `skewFaces.vtk` back to source coordinates automatically, then apply bounded local conditioning.

## 2026-06-22 - Automated Strict snappy STL Repair Loop

- Added `su2_sandbox\scripts\auto_repair_snappy_stl.py`.
- The script keeps the direct sparse exporter intact and operates only as a post-export/pre-mesh OpenFOAM conditioning loop.
- Automated flow:
  - Prepare raw STL with `edge_fan` small-loop capping.
  - Generate and run a strict snappyHexMesh case.
  - Run default `checkMesh`.
  - If strict skewness fails, run `checkMesh -writeSets -writeSurfaces -setFormat vtk -surfaceFormat vtk`.
  - Parse `skewFaces.vtk`, convert skew-face centers from OpenFOAM meters to STL millimeters, cluster nearby defects, and apply bounded local smoothing.
  - Rebuild and rerun strict snappy until default `checkMesh` passes or the repair iteration/displacement budget is exceeded.
- Fixed the Windows-to-WSL path bridge in the new script by mapping Windows paths directly to `/mnt/<drive>/...`.
- Added explicit OpenFOAM environment sourcing through `--openfoam-bashrc`, defaulting to `/opt/openfoam13/etc/bashrc`.
- Validation command:
  `.\.venv\Scripts\python scripts\auto_repair_snappy_stl.py --input-stl ..\dual_contouring\direct_sparse_sdf_mc_experiment\stl\basic_no_inlet_wing_span_plus20_spacing_1p0.stl --run-dir runs\basic_no_inlet_wing_span_plus20_auto_repair_strict --max-repair-iterations 2 --run-potential --overwrite`.
- Validation result: status `strict_mesh_ok`.
- Iteration `00`: strict default `checkMesh` reproduced the known failure with `438,245` cells, `49,267` aircraft faces, max non-orthogonality `66.763739`, max skewness `4.5113177`, and `2` highly skew faces.
- The script parsed two skew-face centers and clustered them at `617.650375,-137.83725,165.00275 mm`.
- Iteration `01`: strict default `checkMesh` passed with `438,245` cells, `49,266` aircraft faces, max non-orthogonality `66.763739`, and max skewness `3.8155389`.
- `potentialFoam` ran at `50 mph` on the repaired case.
- Rendered aircraft-only ISO screenshot with ParaView `pvpython`.
- Primary report: `su2_sandbox\runs\basic_no_inlet_wing_span_plus20_auto_repair_strict\auto_repair_report.json`.
- Final case: `su2_sandbox\runs\basic_no_inlet_wing_span_plus20_auto_repair_strict\case_repair_iter_01`.
- Final screenshot: `su2_sandbox\runs\basic_no_inlet_wing_span_plus20_auto_repair_strict\case_repair_iter_01\aircraft_iso_surface.png`.
- Current decision: automatic strict-snappy repair is now the preferred current-option path for exporter/STL mesh-conditioning experiments, but not yet a scoring-CFD policy.
- Remaining risk: this has only been validated on the known no-inlet defect family. It needs more aircraft variants and an explicit geometry-capture metric before it can become optimizer production policy.

## 2026-06-22 - snappyHexMesh Strategy Evidence Report

- Produced a standalone evidence report for the OpenFOAM/snappyHexMesh meshing option.
- Report path: `su2_sandbox\runs\basic_no_inlet_wing_span_plus20_auto_repair_strict\snappyhexmesh_strategy_evidence.md`.
- Added geometry-capture tooling:
  - `su2_sandbox\scripts\render_aircraft_view_set.py`.
  - `su2_sandbox\scripts\measure_aircraft_patch_capture.py`.
- Rendered aircraft-only geometry evidence:
  - ISO, top, side, front.
  - Wing root/blend, leading-edge, trailing-edge, wingtip, and nose/tail closeups.
- Generated patch/source comparison report: `su2_sandbox\runs\basic_no_inlet_wing_span_plus20_auto_repair_strict\geometry_capture_report.json`.
- Geometry-capture metric is approximate nearest-vertex distance, not exact point-to-triangle Hausdorff.
- Patch/source counts:
  - Raw source STL faces: `1,157,918`.
  - Conditioned STL faces: `1,157,926`.
  - Final snappy aircraft patch faces: `49,266`.
- Approximate patch-to-source nearest-vertex distance: p50 `0.386 mm`, p95 `0.600 mm`, max `1.727 mm`.
- Approximate sampled-source-to-patch nearest-vertex distance: p50 `1.238 mm`, p95 `2.702 mm`, max `3.862 mm`.
- Verdict: `CONDITIONAL`.
- Recommendation: snappyHexMesh is suitable for near-term optimizer plumbing and mesh-development integration, but not yet for scoring CFD.
- Blocking issue for scoring CFD: visible blockiness/stair-stepping remains around thin wing edges, tips, and transition/refinement bands even though strict `checkMesh` passes.
- Next highest-value fix: feature-aware refinement for LE/TE/tips/blends plus exact or sampled point-to-triangle surface deviation metrics.

## 2026-06-22 - Gmsh vs snappyHexMesh Evidence Review

- Reviewed matching mesher decision evidence for Gmsh and snappyHexMesh on the wing-span +20 no-inlet aircraft basis.
- Gmsh evidence case: `custom_cfd_mesher_experiment\runs\decision_gmsh_span_plus20_8k`.
- Gmsh result: `CONDITIONAL`.
- Gmsh produced strict `checkMesh` pass, OpenFOAM/SU2 handoff artifacts, and `potentialFoam` smoke completion.
- Gmsh metrics: `91,586` tetrahedra, `18,214` points, `8,000` aircraft faces, max skewness `3.3085963`, max non-orthogonality `88.702112`, `1,525` severe non-orthogonal faces, p95/p99/max surface deviation `0.1041/0.1545/0.3823 mm`, full pipeline runtime `60.928 s`.
- Gmsh strengths: fast runtime, direct SU2 and OpenFOAM handoff, explicit point-to-surface deviation audit, and strong automation potential.
- Gmsh concerns: aggressive 8k wall-surface reduction, faceted visual surface, long/ugly triangle behavior in wing closeups, no boundary-layer/prism strategy, and many severe non-orthogonal faces.
- snappyHexMesh evidence case: `su2_sandbox\runs\basic_no_inlet_wing_span_plus20_auto_repair_strict\case_repair_iter_01`.
- snappyHexMesh result: `CONDITIONAL`.
- snappyHexMesh produced strict `checkMesh` pass, native OpenFOAM case artifacts, and `potentialFoam` smoke completion after bounded local auto-repair.
- snappyHexMesh metrics: `438,245` cells, `520,861` points, `49,266` aircraft faces, max skewness `3.8155389`, max non-orthogonality `66.763739`, patch-to-source p95 about `0.600 mm`, sampled source-to-patch p95 about `2.702 mm`, auto-repair wrapper runtime about `929.6 s`.
- snappyHexMesh strengths: native OpenFOAM path, higher aircraft patch density, strict `checkMesh` pass, traceable failure/repair loop, and no relaxed mesh policy needed for the final repaired case.
- snappyHexMesh concerns: much slower runtime, visible blockiness/stair-stepping around LE/TE/tips/blends, approximate nearest-vertex deviation metric only, and geometry-dependent repair/refinement risk.
- Current decision: do not choose a single mesher yet. Continue refining both. Treat Gmsh as the fast tetrahedral/SU2-capable path and snappyHexMesh as the native OpenFOAM path.
- Scoring-CFD status: blocked for both. Gmsh needs bounded feature refinement, boundary-layer/prism strategy, and severe non-orthogonality mitigation. snappyHexMesh needs feature-aware LE/TE/tip/blend refinement and better point-to-triangle geometry-capture metrics.

## 2026-06-22 - Shared Mesh Comparison Contract and Variant Set

- Added `software\optimizer\contracts\mesh_comparison_acceptance_contract.md`.
- Added `software\optimizer\configs\mesh_comparison_variants.v0_1.json`.
- The contract defines common Gmsh/snappyHexMesh evidence requirements: verdicts, required artifacts, strict mesh checks, solver smoke, aircraft-only screenshot set, geometry-capture metrics, runtime reporting, and cross-mesher decision rules.
- The contract explicitly states that no current mesh result should be marked `scoring_ready` until boundary-layer strategy, force reporting, and solver validation are defined and passed.
- The five fixed-topology no-inlet variants are:
  - `mc_v01_span720_reference`: span `720 mm`, root chord `170 mm`, tip chord `80 mm`, sweep `15 deg`.
  - `mc_v02_high_aspect_low_sweep`: span `900 mm`, root chord `150 mm`, tip chord `55 mm`, sweep `5 deg`.
  - `mc_v03_low_aspect_high_sweep`: span `580 mm`, root chord `220 mm`, tip chord `115 mm`, sweep `32 deg`.
  - `mc_v04_high_taper_aggressive_tip`: span `760 mm`, root chord `210 mm`, tip chord `45 mm`, sweep `20 deg`.
  - `mc_v05_broad_chord_negative_sweep`: span `650 mm`, root chord `200 mm`, tip chord `100 mm`, sweep `-3 deg`.
- Updated `README.md`, `AGENT_README.md`, and `software\optimizer\contracts\README.md` to reference the new comparison contract and variant file.
- Validation command: `py -3.12 -m json.tool aircraft_optimizer_platform\software\optimizer\configs\mesh_comparison_variants.v0_1.json`; passed.

## 2026-06-22 - Improved snappy Baseline l4/l5

- Ran higher-fidelity snappy refinement variants through the automatic STL repair loop.
- Preferred snappy baseline case: `su2_sandbox\runs\basic_no_inlet_wing_span_plus20_auto_repair_l4_l5\case_repair_iter_01`.
- Preferred command used surface refinement `4-5`, feature level `4`, max local cells `5,000,000`, max global cells `14,000,000`, and max repair iterations `3`.
- Initial l4/l5 attempt failed strict `checkMesh` with `6` skew faces and max skewness `5.9239091`.
- Auto-repair applied six local smoothing patches, each radius `8 mm`, `4` iterations, relaxation `0.12`.
- Maximum nearest-baseline STL displacement after repair: `0.3180982114527392 mm`.
- Final l4/l5 strict `checkMesh`: `Mesh OK`.
- Final l4/l5 mesh metrics:
  - Cells: `978,881`.
  - Points: `1,175,505`.
  - Aircraft faces: `134,254`.
  - Farfield faces: `14,336`.
  - Max skewness: `3.8826061`.
  - Max non-orthogonality: `74.777901`.
  - Min/max cell volume: `1.0648682e-10` / `4.4177484e-05 m^3`.
- `potentialFoam` completed at `50 mph`.
  - Continuity error: `2.4640308e-05`.
  - Interpolated velocity error: `0.00011759704`.
- Added sampled point-to-triangle metric script: `su2_sandbox\scripts\measure_surface_deviation.py`.
- l4/l5 sampled point-to-triangle source-to-patch surface deviation:
  - p50 `0.00552 mm`.
  - p95 `0.0472 mm`.
  - max `0.5146 mm`.
- Also tested `surface 4-5 / feature level 5`.
  - Case: `su2_sandbox\runs\basic_no_inlet_wing_span_plus20_auto_repair_l4_l5_feat5\case_repair_iter_01`.
  - Aircraft faces: `150,197`.
  - Max skewness: `3.9954521`, very close to the strict `4.0` threshold.
  - Source-to-patch sampled point-to-triangle p95: `0.0532 mm`, slightly worse than feature level `4`.
- Current snappy-only decision: use `surface level 4-5 / feature level 4` as the preferred snappy development baseline.
- Do not make `feature level 5` the snappy default yet; its quality margin is too thin and its measured geometry-capture gain is not better.
- Report path: `su2_sandbox\runs\basic_no_inlet_wing_span_plus20_auto_repair_l4_l5\snappy_refinement_improvement_report.md`.

## 2026-06-22 - snappy Comparison Runtime Guardrail

- Attempted the shared comparison variant `mc_v01_span720_reference` with the corrected contract geometry: span `720 mm`, root chord `170 mm`, tip chord `80 mm`, sweep `15 deg`.
- The raw direct-sparse STL exported successfully but failed the strict raw surface gate with `52` boundary edges.
- OpenFOAM `surfaceCheck` reported the prepared surface was not closed: `52` edges connected to one face and `0` edges connected to more than two faces.
- The existing auto-cap policy refused the loop because the loop span was about `19.64 mm x 6.21 mm`, above the current `10 mm` cap threshold.
- The first comparison harness attempt still ran four full snappy repair iterations and took about `6,117 s` for snappy-side work, then failed strict `checkMesh` with max skewness about `5.1156`.
- Added runtime guardrails:
  - `auto_repair_snappy_stl.py` now supports OpenFOAM command timeouts, snappy command timeout, and fail-fast when the prepared surface remains open.
  - `run_snappy_mesh_comparison_variants.py` now defaults to one repair iteration, `1200 s` snappy timeout, `2400 s` variant timeout, and requires a closed prepared STL before launching snappy.
- Validation: reran `mc_v01_span720_reference` with the existing STL and fail-fast enabled; it stopped in about `26 s` with `fail_for_plumbing` because the prepared STL remained open. snappyHexMesh was not launched.
- Current blocker for this variant: fix or intentionally cap the `52`-edge boundary loop before running snappy. Do not spend full snappy runtime on this case until the surface-prep gate passes.

## 2026-06-22 - Relaxed snappy Skew Policy and v01 Smoke Result

- User accepted `max skewness < 8` as acceptable for this meshing option.
- Updated snappy automation so strict `checkMesh` failure is still recorded, but meshes under a configured relaxed skew threshold can proceed as relaxed/conditional plumbing evidence.
- Updated `auto_repair_snappy_stl.py`:
  - Added `--acceptable-max-skewness`.
  - Records `relaxed_mesh_ok` when skewness is below policy threshold but OpenFOAM strict `checkMesh` still fails.
  - Uses `foamToVTK -noInternal` for boundary-only VTK export instead of full-volume conversion.
- Updated `run_snappy_mesh_comparison_variants.py`:
  - Added `--acceptable-max-skewness`, default `8.0`.
  - Records `relaxed_mesh_check_pass` separately from strict `checkMesh` pass.
  - Marks relaxed accepted cases as conditional plumbing evidence instead of hard failure.
- OpenFOAM-native surface cleanup results for `mc_v01_span720_reference`:
  - `surfaceConvert -clean` took about `659.5 s` and did not close the original `52` boundary edges.
  - `surfacePointMerge` at `0.001 mm` took about `470.3 s` and did not change vertex count or close the surface.
  - The broader OpenFOAM cleanup sweep was stopped because runtime was excessive.
- Bounded explicit cap result:
  - Delaunay cap with `max_loop_span_mm=20` closed the `52`-edge loop.
  - Added `50` cap faces and no new vertices.
  - OpenFOAM `surfaceCheck`: closed surface, no illegal triangles, one connected part, two normal-orientation zones.
  - Cap max edge was about `5.63 mm`, better than edge-fan cap max edge about `19.11 mm`.
- Best relaxed snappy case so far:
  - Case: `su2_sandbox\runs\mesh_comparison_snappy\mc_v01_span720_reference_delaunay_cap20_l4_l5_repair1\case_repair_iter_01`.
  - Cells: `997,604`.
  - Points: `1,198,615`.
  - Aircraft faces: `137,566`.
  - Farfield faces: `14,336`.
  - Max skewness: `4.1238511` with `1` highly skew face.
  - Max non-orthogonality: `69.360357`.
  - Strict `checkMesh`: failed one skewness check.
  - Relaxed skew policy `< 8`: accepted.
  - `potentialFoam` completed at `50 mph`.
  - Continuity error: `2.424652e-05`.
  - Interpolated velocity error: `0.00011748414`.
  - Aircraft-only ISO screenshot: `su2_sandbox\runs\mesh_comparison_snappy\mc_v01_span720_reference_delaunay_cap20_l4_l5_repair1\case_repair_iter_01\aircraft_iso_surface.png`.
- Repeated local smoothing is not reliable here:
  - A second smoothing pass from the conditioned STL worsened max skewness from `4.1238511` to `4.9746398`.
  - Stop smoothing this branch unless changing smoothing parameters deliberately.

## Active Decisions

- The optimizer platform consumes the existing implicit CAD/SDF geometry system through an adapter. It does not redesign the CAD system.
- v1 scope is fixed-wing small UAV loiter/endurance optimization.
- Initial optimization variables are continuous only.
- No initial topology changes, aircraft family changes, adding/removing wings, or adding/removing fins.
- The system must preserve failed candidates as useful data.
- Reruns and rescoring must create new records instead of overwriting old results.
- The dashboard is standalone and local; it is not assumed to be integrated into the CAD application.
- High-fidelity CFD is selective and gated, not the default path for every candidate.
- The primary autonomous optimizer workflow is sequential and gated: design, cheap pre-export checks, export if policy allows, analysis if policy allows, record results, then propose the next design.
- Batch sweeps are secondary workflows for diagnostics, DOE, parallel campaigns, stress tests, or future surrogate-data generation.
- The active v0.1 proposal policy is deterministic Halton ask/tell, not a final production optimizer.
- ML and MDAO support are roadmap targets, not initial implementation targets.
- Default CFD STL export for normal optimizer iterations is `direct_sparse_oml_fast`.
- Smaller-cell/high-fidelity OML export is an explicit rerun/finalization option for converged or promising designs, not the default.
- Geometry definition and STL export validation must stay separate functions.
- Production CFD-ready OML exports should use strict default gates: `boundary_edges=0`, `nonmanifold_edges=0`, `connected_components=1`, `duplicate_triangles=0`, and `long_chord_sections_ge_75mm=0`.
- OpenFOAM candidate attempts should preserve both raw surface-gate failures and repaired-surface mesh/solver results when repair is used.
- Static margin must not be calculated or gated before CFD/stability neutral-point data exists. Use post-CFD component shifting to improve margin when geometry and component ranges allow it.
- OML STL topology gates are necessary but not sufficient for real CFD. Real CFD requires a solver-ready volume mesh, acceptable surface triangle quality for the chosen mesher, and explicit semantic-to-solver marker mapping.
- CGNS section names like `3_S_7` may be used inside adapter metadata, but the platform-facing marker contract remains `aircraft`, `farfield`, and `fluid`.
- For repaired STL-based full-aircraft CFD experiments, keep both Gmsh and OpenFOAM/snappyHexMesh active. Gmsh is the faster tetrahedral/SU2-capable path; snappyHexMesh is the native OpenFOAM path. Do not choose a default until both have been tested on several variants with common geometry-capture gates.
- `aircraft_optimizer_platform` is the master folder for platform documentation and eventual platform-owned/vendored software.
- Existing CAD/SDF and exporter code should not be moved casually; use adapters and provenance records first.
- Manual SDF generation, automatic SDF generation, exporters, optimizer, dashboard, and analysis modules are separate component areas under `software/`.
- Curated Rhai scripts are references, not all canonical optimizer inputs. The validated no-inlet direct sparse OML model is the strongest current canonical export reference.
- Exporter integrations must conform to optimizer/platform contracts instead of defining platform records.
- OML STL exporter integration must pass through `software\exporters\oml_stl\export_adapter_contract.md`; optimizer orchestration should not call exporter scripts directly.
- v0.1 implementation should prove records/events/artifacts with mock modules before calling real CAD/exporter tools.
- Docker is deferred for v0.1. Add it later for dashboard/API, workers, PostgreSQL, or heavy tool environments.
- Keep standard-library dataclasses and plain JSON records for the immediate v0.1 skeleton. Revisit Pydantic or JSON Schema when configuration validation becomes broader than the current tests.
- Export bbox and CFD farfield bbox are separate policies. Export bboxes prevent
  aircraft clipping; CFD farfield bboxes prevent boundary-condition contamination.
- Optimizer-facing STL exports should use SDF-probed auto-bbox by default.
  Manual bboxes are explicit overrides only.
- Gmsh/OpenFOAM CFD farfields should scale from measured aircraft extents by
  default instead of using one fixed domain size for every aircraft.
- Aerodynamic force convention for CFD smoke runs is now:
  - freestream/drag: `+X`
  - lift: `+Z`
  - pitch axis: `+Y`
  - older runs that used lift `+Y` should be treated as side-force-like and not
    compared directly to current vertical-lift `Cl`.

## 2026-06-22 Gmsh Gated Exported Variant

- Added/validated the one-command Gmsh/OpenFOAM candidate smoke runner:
  `custom_cfd_mesher_experiment\scripts\run_gated_mesh_smoke.py`.
- Exported a new no-inlet aircraft variant from Rhai with:
  - `35 deg` wing sweep.
  - `+20%` wing root chord.
  - `+20%` horizontal and vertical tail dimensions.
- Initial default-bbox STL export failed strict topology with `430` boundary
  edges. Boundary localization showed clipping on hard bbox planes.
- Re-exported with a wider bbox:
  `--bbox-min=-32,-224,-256 --bbox-max=736,112,528`.
- Wider-bbox export reduced the defect to one tiny four-edge opening. Source
  prep capped that loop with max span about `1.01 mm`, producing a watertight
  one-body source surface.
- Full gated pipeline passed:
  `custom_cfd_mesher_experiment\runs\sweep35_root120_tail120_gated_smoke`.
- Added and verified an opt-in bounded source-prep flag on the runner:
  `--prep-source-cap-loop-span-mm 2.0`. Verification run:
  `custom_cfd_mesher_experiment\runs\sweep35_root120_tail120_runner_prep_skip_solver`.
- Mesh metrics:
  - Cells: `357,474`.
  - Points: `69,300`.
  - Aircraft faces: `31,106`.
  - `defaultFaces`: `0`.
  - Max skewness: `2.2681885`.
  - Max non-orthogonality: `85.721881`.
  - Severe non-orthogonal faces: `143`.
  - SU2 validation: valid.
  - OpenFOAM `checkMesh`: `Mesh OK`.
- 60-step OpenFOAM `incompressibleFluid` smoke completed without floating
  point exception. Final values: `Cd=0.0660081909`, `Cl=0.0010694494`,
  `Cm=-1.75269211e-05`.
- Force stability summary rejected the run for scoring due to lift/moment tail
  drift. Treat this as mesh/solver plumbing evidence, not aerodynamic scoring.
- Implemented SDF-probed automatic exporter bboxes. On the same 35-degree
  swept/larger-tail variant, the automatic bbox selected:
  - search bounds: `[-128,-512,-384]` to `[1024,512,768]` mm.
  - active bounds before margin: `[-16,-176,-240]` to `[688,64,496]` mm.
  - final export bbox: `[-64,-224,-288]` to `[736,112,544]` mm.
  - result: no clipping recurrence; the remaining `4` boundary edges are the
    known tiny tile-opening issue, not a bbox issue.
- Implemented dynamic CFD farfield sizing from measured STL extents. Dynamic
  farfield verification run:
  `custom_cfd_mesher_experiment\runs\sweep35_root120_tail120_autobbox_dynamic_farfield_skip_solver`.
- Dynamic farfield policy for that variant:
  - aircraft extent: `0.6762,0.2130,0.7159 m`.
  - upstream padding: `1.014 m`.
  - downstream padding: `2.029 m`.
  - side Y padding: `0.426 m`.
  - side Z padding: `1.432 m`.
  - farfield mesh size: `0.1074 m`.
- Dynamic farfield mesh passed: `407,749` cells, `79,224` points, `31,126`
  aircraft faces, zero `defaultFaces`, max skewness `1.8671639`, max
  non-orthogonality `85.703803`, `116` severe non-orthogonal faces, SU2 valid,
  OpenFOAM `Mesh OK`.
- Dynamic farfield 60-step OpenFOAM smoke completed without floating point
  exception. Final values: `Cd=0.065960178`, `Cl=0.000533424136`,
  `Cm=-9.61068887e-05`. Force stability still failed for scoring.
- Fixed the remaining tiny tile-opening issue at the exporter level:
  - Export result:
    `dual_contouring\direct_sparse_sdf_mc_experiment\logs\optimizer_export_result_sweep35_root120_tail120_autobbox_capfix.json`.
  - Exported STL:
    `dual_contouring\direct_sparse_sdf_mc_experiment\stl\direct_sdf_oml_sweep35_root120_tail120_autobbox_capfix_spacing_1p0.stl`.
  - Export gate: pass.
  - Boundary edges: `0`.
  - Nonmanifold edges: `0`.
  - Connected components: `1`.
  - Tiny loop cap provenance: one four-vertex loop, span
    `0.0809,0.0106,1.0099 mm`, perimeter `2.0290 mm`, four new faces.
- Reran the full gated mesh/solver path without mesher-side source prep:
  `custom_cfd_mesher_experiment\runs\sweep35_root120_tail120_exporter_capfix_dynamic_forces_zlift`.
- Corrected-axis mesh passed: `404,723` cells, `78,688` points, `30,964`
  aircraft faces, zero `defaultFaces`, max skewness `2.4848425`, max
  non-orthogonality `86.385123`, `121` severe non-orthogonal faces, SU2 valid,
  OpenFOAM `Mesh OK`.
- Corrected-axis 60-step OpenFOAM smoke completed without floating point
  exception. Force convention was lift `0,0,1`, drag `1,0,0`, pitch axis
  `0,1,0`. Final values: `Cd=0.065971069`, `Cl=-0.113468059`,
  `Cm=0.0267205536`. Force stability gate passed for this smoke window.
- Decision: the Gmsh path is now repeatable enough for near-term optimizer
  integration as a gated CFD smoke pipeline, but real scoring still needs a
  scoring-grade solver policy and longer stability/convergence criteria.
- 2026-06-22 snappy comparison pause:
  - Stopped the five-variant snappy comparison because the inlet geometry is
    about to change and the current no-inlet/old-inlet variant set is no longer
    the right acceptance target.
  - Killed the active snappy comparison harness, v02 auto-repair subprocess,
    and WSL `snappyHexMesh` wrappers for
    `mesh_comparison_snappy_relaxed_skew8_delaunay_cap20_v2`; no matching WSL
    snappy process remained afterward.
  - Preserved the completed v01 reference evidence at
    `su2_sandbox\runs\mesh_comparison_snappy_relaxed_skew8_delaunay_cap20_v2\mc_v01_span720_reference`.
  - v01 status: Delaunay cap closed the `52` boundary edges, final status
    `relaxed_mesh_ok`, max skewness `6.218329`, max non-orthogonality
    `69.878293`, and `potentialFoam`/screenshot reports were written.
  - Harness fixes retained for the next valid geometry set: absolute run paths,
    explicit `--cap-method`, aircraft-boundary-only VTK export, raw STL gate
    failure preserved but not fatal when a prep/cap stage is responsible for
    closure, and relaxed non-orthogonality ceiling defaulted to `70`.
- 2026-06-22 snappy evidence infrastructure:
  - Added partial-run-safe reporting to the snappy comparison harness. It now
    writes `run_manifest.json`, `mesh_quality_report.json`,
    `geometry_capture_report.json`, `solver_smoke_report.json`,
    `evidence_bundle.json`, `evidence_report.md`, and summary JSON/Markdown
    even when a variant fails far enough to classify.
  - Added stable snappy failure taxonomy codes for geometry, meshing, solver,
    artifact, and automation failures.
  - Added report-only refresh mode so existing completed cases can validate
    the evidence schema without rerunning export, snappy, or solver steps.
  - Added feature-region proxy geometry-capture metrics to
    `measure_surface_deviation.py`, using `x` as longitudinal and `z` as
    lateral/span by default.
  - Refreshed the v01 no-inlet reference evidence only; no new acceptance run
    should be inferred from the paused old-geometry folder.
  - Added `snappy_mesher_evidence_contract.md` and updated the comparison
    contract to require evidence bundles, taxonomy, and feature-region metrics.
- 2026-06-23 faired-cap inlet snappy/OpenFOAM comparison:
  - Ran the approved five full-path solid faired-cap inlet STL variants through
    a fresh snappyHexMesh/OpenFOAM external-flow smoke harness.
  - Final accepted run root:
    `su2_sandbox\runs\openfoam_snappy_faired_cap_inlets_v0_1_20260623_skew8_v2`.
  - Inputs came from
    `software\optimizer\configs\faired_cap_inlet_stl_variants.v0_1.json`.
  - Solver smoke used `potentialFoam -writep` at `50 mph` / `22.352 m/s` in
    `+X`, with aircraft `slip`; no lift/drag scoring was treated as valid.
  - All five variants completed OpenFOAM mesh generation, report generation,
    aircraft-only screenshots, surface-deviation measurement, and
    `potentialFoam -writep`.
  - All five are `conditional_development_only`: strict `checkMesh` failed, but
    max skewness stayed below the approved development threshold of `8`, solver
    smoke completed, and source-to-patch p95 was approximately `0.048-0.052 mm`.
  - No case is scoring-CFD ready. There are no boundary/prism layers, and all
    cases retain strict OpenFOAM quality failures.
  - Consolidated report:
    `su2_sandbox\runs\openfoam_snappy_faired_cap_inlets_v0_1_20260623_skew8_v2\openfoam_snappy_faired_cap_inlets_final_report.md`.
  - Visual sheets:
    `aircraft_iso_contact_sheet.png` and `inlet_faired_cap_contact_sheet.png`
    in the same run root.

## 2026-06-23 - Single-Inlet Gmsh Stress Test and Non-Orthogonality Diagnostics

- Exported the current high-fidelity single-inlet OML STL through the current
  direct sparse exporter path:
  `dual_contouring\direct_sparse_sdf_mc_experiment\stl\direct_sdf_single_inlet_autobbox_capfix_spacing_0p75_spacing_0p75.stl`.
- Export report:
  `dual_contouring\direct_sparse_sdf_mc_experiment\logs\optimizer_export_result_single_inlet_autobbox_capfix_hifi.json`.
- Export gate passed: `2,448,750` triangles, `1,224,373` vertices,
  `0` boundary edges, `0` nonmanifold edges, `1` connected component.
- The fast 1.0 mm single-inlet export completed but failed strict topology
  gate with `2` nonmanifold edges; keep it out of CFD.
- Baseline 4 mm Gmsh mesh run:
  `custom_cfd_mesher_experiment\runs\single_inlet_current_hifi_autobbox_capfix_dynamic_fine4mm_zlift`.
- Baseline mesh was OpenFOAM/SU2 valid but failed strict mesh gate:
  `732,270` cells, `150,255` points, `94,042` aircraft faces, max skewness
  `2.2654979`, max non-orthogonality `88.484038`, `418` severe
  non-orthogonal faces.
- Added automatic failed-mesh diagnostics to
  `custom_cfd_mesher_experiment\scripts\run_gated_mesh_smoke.py`: on mesh gate
  failure, run `checkMesh -writeSurfaces -writeSets -nonOrthThreshold 70` and
  localize `nonOrthoFaces.vtk`.
- Expanded `custom_cfd_mesher_experiment\scripts\localize_checkmesh_faces.py`
  to report ranked bad-face voxels and suggested refinement boxes.
- Baseline localization report:
  `custom_cfd_mesher_experiment\runs\single_inlet_current_hifi_autobbox_capfix_dynamic_fine4mm_zlift\nonortho_localization.json`.
- Bad-face localization: `401 / 418` severe faces lie inside aircraft bounds.
  The largest hot spots are symmetric mid/right-tip and mid/left-tip lower-z
  feature-transition regions, plus tail/center upper and high-z regions.
- Rejected broad hotspot volume refinement boxes as the next fix: the first
  eight boxes at `0.0028 m` target size drove Gmsh toward about `196M`
  blockfill points and crashed.
- Tested algorithm and cleanup variants:
  - Algorithm 10: `1,023,485` cells, `528` severe faces; worse.
  - Algorithm 4 without Netgen: `1,003,488` cells, failed `checkMesh`,
    `659` severe faces; worse.
  - 5 mm no-smooth remesh: `580,949` cells, `476` severe faces; worse.
  - 4 mm smoothed 6-iteration remesh:
    `674,257` cells, `408` severe faces; slight improvement only.
  - 4 mm smoothed 10-iteration remesh:
    `714,696` cells, `422` severe faces; worse than 6-iteration smoothing.
- Added explicit `--smooth-remesh` support to the gated Gmsh runner, but do
  not make smoothing the default until it produces a clear gate pass.
- Validation performed: `py_compile` on the touched mesher scripts, manual
  `checkMesh -writeSurfaces`, localization JSON generation, and aircraft-only
  ISO screenshots for the tested mesh cases.
- Current decision: the single-inlet geometry is a legitimate stress case.
  Generic cleanup/smoothing is insufficient; next fix should be
  geometry-aware local surface conditioning or tighter feature-specific sizing,
  not broad volume refinement boxes.

## 2026-06-23 - CFD Faired-Cap Inlet Export Mode

- Added a procedural Rhai SDF primitive `faired_inlet_cap(...)` in the
  `dual_contouring` sidecar API for shallow smooth cap/blister solids driven by
  local center, streamwise axis, crosswise axis, outward normal, length, width,
  height, and embed depth.
- Added `direct_sparse_sdf_mc_experiment\scripts\make_cfd_inlet_mode_rhai.py`
  to generate explicit `flow_through`, `faired_cap`, and `none` inlet-mode
  scripts from the baseline single-inlet Rhai aircraft.
- Integrated `--inlet-mode {flow_through,faired_cap,none}` into
  `direct_sparse_sdf_mc_experiment\scripts\optimizer_export_presets.py`.
- Added `direct_sparse_sdf_mc_experiment\scripts\diagnose_faired_cap_stl.py`
  for STL topology diagnostics plus cap-region triangle counts, bounding boxes,
  max cap height, and debug CSV output.
- Generated and exported:
  `direct_sparse_sdf_mc_experiment\stl\direct_sdf_oml_inlet_faired_cap_spacing_1p0.stl`
  and
  `direct_sparse_sdf_mc_experiment\stl\direct_sdf_oml_inlet_flow_through_spacing_1p0.stl`.
- Faired-cap export metrics: `1,113,258` triangles, `556,631` welded vertices,
  `0` boundary edges, `0` nonmanifold edges, `1` connected component,
  `0` duplicate triangles, watertight `true`.
- Faired-cap region diagnostics:
  - inlet cap: `39,092` triangles, bbox approximately
    `[255.0, -50.06, 43.74]` to `[401.0, 50.06, 63.0]`, max cap height
    `11.995 mm` above base.
  - exhaust cap: `24,468` triangles, bbox approximately
    `[674.0, -39.0, -31.9]` to `[704.18, 39.0, 31.9]`, max cap height
    `9.141 mm` above base after tail blending.
- Flow-through comparison export remained separate and unchanged in intent:
  `1,376,712` triangles, `688,352` welded vertices, `0` boundary edges,
  `2` nonmanifold edges, `1` connected component, watertight `false`.
- Validation performed: Python `py_compile`, Rust formatter, targeted
  `cargo test --lib scripting::tests::test_faired_inlet_cap_primitive`, rebuilt
  `batch_sdf_distance_only`, sidecar point probes on both generated scripts,
  full fast-preset STL exports for both inlet modes, and standalone topology
  diagnostics.
- Known limitation: the first faired-cap implementation is a local SDF bump
  blended into the aircraft body, not a true lip-curve projection onto an
  arbitrary fuselage surface. It is good enough for optimizer-level OML CFD
  screening, but detailed inlet CFD should still use `flow_through`.

## 2026-06-23 - Five Faired-Cap Vehicle Variant STL Exports

- Added the five-variant manifest:
  `software\optimizer\configs\cfd_faired_cap_vehicle_variants.v0_1.json`.
- Added batch exporter:
  `dual_contouring\direct_sparse_sdf_mc_experiment\scripts\export_faired_cap_variants.py`.
- Extended inlet-mode generation and preset export wrappers to support numeric
  geometry overrides such as fuselage length, wing span/chord/sweep/position,
  tail dimensions, and inlet station.
- Exported five capped-inlet full-aircraft STL variants for mesher testing:
  - `fcv01_long_glider`:
    `direct_sparse_sdf_mc_experiment\stl\direct_sdf_oml_fcv01_long_glider_faired_cap_spacing_1p0.stl`.
  - `fcv02_short_swept`:
    `direct_sparse_sdf_mc_experiment\stl\direct_sdf_oml_fcv02_short_swept_faired_cap_spacing_1p0.stl`.
  - `fcv03_high_aspect_mild`:
    `direct_sparse_sdf_mc_experiment\stl\direct_sdf_oml_fcv03_high_aspect_mild_faired_cap_spacing_1p0.stl`.
  - `fcv04_compact_wide_tail`:
    `direct_sparse_sdf_mc_experiment\stl\direct_sdf_oml_fcv04_compact_wide_tail_faired_cap_spacing_1p0.stl`.
  - `fcv05_aft_wing_fast`:
    `direct_sparse_sdf_mc_experiment\stl\direct_sdf_oml_fcv05_aft_wing_fast_faired_cap_spacing_1p0.stl`.
- Final diagnostics for all five exported STLs: `0` boundary edges, `0`
  nonmanifold edges, `1` connected component, `0` duplicate triangles,
  watertight `true`.
- Wrote mesher handoff summaries:
  `direct_sparse_sdf_mc_experiment\logs\faired_cap_variant_export_matrix.json`
  and
  `direct_sparse_sdf_mc_experiment\logs\faired_cap_variant_export_matrix.csv`.
- Initial `fcv04` compact/wide-tail dimensions produced `2` boundary edges and
  `1` nonmanifold edge. The variant was moderated and rerun under the same ID
  and output names; the final replacement export is watertight.
- Validation performed: JSON syntax validation, Python `py_compile`, dry-run of
  the batch exporter, five full direct-sparse exports, per-variant standalone
  faired-cap STL diagnostics, and combined export matrix generation.
- Next action: give the five STL paths plus diagnostics matrix to both meshing
  threads and require each mesher to report pass/fail, runtime, mesh-quality
  metrics, geometry-capture screenshots, and solver smoke status per variant.

## 2026-06-23 - Faired-Cap Orientation Correction and Re-Export

- Corrected the faired-cap implementation after visual review showed the first
  version was wrong: it had replaced the inlet with a fuselage-top Z-facing
  blister and added an unwanted exhaust/outlet cap.
- Updated `faired_cap` generation so it now preserves the external inlet
  fairing, does not subtract the flow-through duct void, caps the inlet's
  forward X-facing mouth with `outward_normal = [-1, 0, 0]`, and removes the
  exhaust cap entirely.
- Updated `oml_export_contract.md` to state that `faired_cap` has no visible
  outlet/exhaust feature. The outlet should simply not be cut.
- Re-exported all five faired-cap variants with the corrected inlet-mouth cap.
  The first corrected `fcv01_long_glider` pass had a tiny topology defect, so
  its dimensions were moderated and that variant was rerun.
- Final corrected diagnostics:
  - `fcv01_long_glider`: `1,359,774` triangles, `679,887` welded vertices,
    `0` boundary edges, `0` nonmanifold edges, watertight `true`.
  - `fcv02_short_swept`: `1,028,924` triangles, `514,462` welded vertices,
    `0` boundary edges, `0` nonmanifold edges, watertight `true`.
  - `fcv03_high_aspect_mild`: `1,294,238` triangles, `647,119` welded
    vertices, `0` boundary edges, `0` nonmanifold edges, watertight `true`.
  - `fcv04_compact_wide_tail`: `1,283,578` triangles, `641,789` welded
    vertices, `0` boundary edges, `0` nonmanifold edges, watertight `true`.
  - `fcv05_aft_wing_fast`: `1,215,926` triangles, `607,963` welded vertices,
    `0` boundary edges, `0` nonmanifold edges, watertight `true`.
- Regenerated the mesher handoff matrix:
  `dual_contouring\direct_sparse_sdf_mc_experiment\logs\faired_cap_variant_export_matrix.json`
  and
  `dual_contouring\direct_sparse_sdf_mc_experiment\logs\faired_cap_variant_export_matrix.csv`.
- Generated quick visual sanity-check renders:
  `dual_contouring\direct_sparse_sdf_mc_experiment\views\corrected_fcv02_iso.png`
  and
  `dual_contouring\direct_sparse_sdf_mc_experiment\views\corrected_fcv02_inlet_closeup.png`.
- Known limitation: this is still a simple SDF mouth cap blended onto the
  preserved inlet fairing. It fixes the orientation and outlet behavior, but a
  future higher-quality inlet cap should derive directly from the true lip
  boundary and local surface normal field.

## 2026-06-23 - Faired-Cap Mouth-Recess Correction, `fcv01` Only

- Visual review found the prior faired-cap result still wrong: one iteration
  produced an oversized square/plate-like cap, and the simplified no-recess
  iteration exposed the solid swept inlet as a rounded rectangular block.
- Updated `make_cfd_inlet_mode_rhai.py` so `faired_cap` no longer subtracts the
  full flow-through `duct_void`, but does preserve the inlet lip with a short
  local mouth recess. The recess is immediately closed by `faired_inlet_cap` and
  does not continue through the fuselage or create an outlet.
- Updated `faired_inlet_cap` in `dual_contouring\src\scripting\api.rs` from a
  rectangular footprint to a rounded superellipse footprint so the visible cap
  is bounded by the inlet opening instead of forming a square plate.
- Patched `direct_sparse_sdf_marching_cubes_export.py` to treat scikit-image's
  `RuntimeError: No surface found at the given iso value` the same as the
  existing empty-tile `ValueError` skip path.
- Rebuilt `target\release\batch_sdf_distance_only.exe`, then re-exported only
  `fcv01_long_glider`:
  `python direct_sparse_sdf_mc_experiment\scripts\export_faired_cap_variants.py --only fcv01_long_glider`.
- Latest `fcv01_long_glider` diagnostics:
  `1,361,720` triangles, `680,860` welded vertices, `0` boundary edges,
  `0` nonmanifold edges, `1` connected component, watertight `true`.
- Generated current visual review images:
  `dual_contouring\direct_sparse_sdf_mc_experiment\views\fcv01_faired_cap_mouth_recess_closeup.png`
  and
  `dual_contouring\direct_sparse_sdf_mc_experiment\views\fcv01_faired_cap_mouth_recess_aircraft_iso.png`.
- Important interpretation: the cap is now bounded to the inlet face region, but
  the remaining boxy rounded-rect inlet form comes from the baseline inlet outer
  SDF geometry. If that shape is unacceptable, the next fix is an inlet external
  geometry redesign, not another cap patch.
- Do not treat the five-variant faired-cap STL set as current. Only `fcv01` has
  been regenerated under this latest mouth-recess contract.

## 2026-06-23 - Outer-Face Faired-Cap Adjustment, `fcv01` Only

- Visual review found the prior mouth-recess cap was still matched to the inner
  inlet opening, leaving a flat outer lip/rim around the capped face.
- Updated `make_cfd_inlet_mode_rhai.py` so the `faired_cap` footprint and local
  mouth recess use `intake_outer_height`, `intake_outer_width`, and
  `outer_start` instead of the inner opening dimensions.
- Set the cap height to `14%` of outer inlet height. This is still a shallow
  optimizer-CFD fairing, not a physically complete real-inlet drag model.
- Re-exported only `fcv01_long_glider`:
  `python direct_sparse_sdf_mc_experiment\scripts\export_faired_cap_variants.py --only fcv01_long_glider`.
- Latest `fcv01_long_glider` diagnostics:
  `1,357,344` triangles, `678,672` welded vertices, `0` boundary edges,
  `0` nonmanifold edges, `1` connected component, watertight `true`.
- Generated current visual review images:
  `dual_contouring\direct_sparse_sdf_mc_experiment\views\fcv01_faired_cap_outer_face_closeup.png`
  and
  `dual_contouring\direct_sparse_sdf_mc_experiment\views\fcv01_faired_cap_outer_face_aircraft_iso.png`.
- Current limitation: this cap is still a generic superellipse fairing mated to
  a rounded-rect swept inlet body. If the corner/edge blend is not visually
  acceptable, the next implementation should derive the cap directly from the
  exact `outer_start` lip profile instead of approximating it with a standalone
  primitive.

## 2026-06-23 - Inlet Section-Cut Diagnosis and Short-Stub Cap, `fcv01` Only

- Generated 1 mm STL section-cut contact sheets around the current `fcv01`
  inlet in XZ, XY, and YZ planes:
  `dual_contouring\direct_sparse_sdf_mc_experiment\plots\inlet_cap_section_cuts\fcv01_outer_face_current`.
- Diagnosis from the section cuts: the visible offset/extension was caused by
  preserving `inlet_outer_visible`, which is the full swept inlet/duct body. It
  extended aft through the inlet crop and was not merely a cap placement issue.
- Updated `faired_cap` generation to stop using the full swept
  `inlet_outer_visible` body. It now builds only a short outer-profile inlet-face
  stub and unions the rounded cap into that stub. `flow_through` remains the
  mode that preserves the full duct geometry.
- Updated `faired_inlet_cap` from a superellipse footprint to a rounded-rect
  footprint matching the inlet profile corner-radius proportion.
- Tested a `2 mm` face stub/embed. It reduced the visual support but failed
  topology: `292` boundary edges, `2` connected components, watertight `false`.
  This artifact is rejected.
- Current accepted test export uses a `6 mm` face stub/embed. Latest
  `fcv01_long_glider` diagnostics:
  `1,286,250` triangles, `643,127` welded vertices, `0` boundary edges,
  `0` nonmanifold edges, `1` connected component, watertight `true`.
- Generated current section-cut evidence:
  `dual_contouring\direct_sparse_sdf_mc_experiment\plots\inlet_cap_section_cuts\fcv01_short_stub_6mm_rounded_rect_cap`.
- Generated current visual review image:
  `dual_contouring\direct_sparse_sdf_mc_experiment\views\fcv01_faired_cap_short_stub_6mm_closeup.png`.
- Interpretation: the large erroneous aft duct extension is removed. The inlet
  still reads somewhat blocky because a constant-section face stub is still
  present. If visual review rejects this, the next implementation should replace
  the stub with a direct lip-boundary cap surface instead of using
  `profile_duct_solid` for the support geometry.

## 2026-06-23 - Solid Optimizer Inlet Redesign, `fcv01` Only

- Rejected the short-stub approach after visual review showed it had become
  only a capped face/stub and no longer preserved the visible inlet body.
- Updated `dual_contouring\direct_sparse_sdf_mc_experiment\scripts\make_cfd_inlet_mode_rhai.py`
  so `faired_cap` now creates a separate solid optimizer inlet path/body instead
  of trying to convert the flow-through inlet into a capped solid.
- Current `faired_cap` behavior: union the aircraft OML with a solid external
  inlet body that continues aft toward the fuselage merge, then union a shallow
  rounded-rect fairing cap on the forward X-facing mouth. It does not subtract
  a duct void and does not create an outlet/exhaust cut.
- Re-exported only `fcv01_long_glider`:
  `python direct_sparse_sdf_mc_experiment\scripts\export_faired_cap_variants.py --only fcv01_long_glider`.
- Latest `fcv01_long_glider` diagnostics:
  `1,337,364` triangles, `668,684` welded vertices, `0` boundary edges,
  `0` nonmanifold edges, `1` connected component, `0` duplicate triangles,
  watertight `true`.
- Exported STL:
  `dual_contouring\direct_sparse_sdf_mc_experiment\stl\direct_sdf_oml_fcv01_long_glider_faired_cap_spacing_1p0.stl`.
- Generated section-cut evidence:
  `dual_contouring\direct_sparse_sdf_mc_experiment\plots\inlet_cap_section_cuts\fcv01_solid_optimizer_inlet_v1`.
- Generated closeup render:
  `dual_contouring\direct_sparse_sdf_mc_experiment\views\fcv01_solid_optimizer_inlet_v1_closeup.png`.
- Visual interpretation: section cuts now show a continuous solid inlet body
  behind the cap instead of a clipped vertical face or short stub. The v1 shape
  is still a squared rounded-rect optimizer inlet with visible fuselage blend
  saddles, so it is acceptable as the next review candidate but not yet final
  aero-quality inlet geometry.
- Do not rerun/export the other four capped-inlet variants until the user
  approves this `fcv01` solid optimizer inlet direction.

## 2026-06-23 - Full-Path Solid Optimizer Inlet, `fcv01` Only

- Visual review accepted the general solid-inlet direction but found the v1
  body still stopped too early. The desired behavior is to keep the full solid
  inlet path buried inside the fuselage and only avoid a visible aft outlet.
- Updated `dual_contouring\direct_sparse_sdf_mc_experiment\scripts\make_cfd_inlet_mode_rhai.py`
  so `faired_cap` no longer uses the abbreviated
  `solid_optimizer_inlet_path`. It now builds `solid_optimizer_inlet_body` from
  the source `duct_path`, `outer_start`, and `outer_end`, while still excluding
  `duct_void` and the aft exhaust extension from `faired_cap`.
- Added reusable STL section-cut diagnostic:
  `dual_contouring\direct_sparse_sdf_mc_experiment\scripts\plot_faired_cap_stl_section_cuts.py`.
- Re-exported only `fcv01_long_glider`:
  `python direct_sparse_sdf_mc_experiment\scripts\export_faired_cap_variants.py --only fcv01_long_glider`.
- Latest `fcv01_long_glider` diagnostics:
  `1,355,886` triangles, `677,943` welded vertices, `0` boundary edges,
  `0` nonmanifold edges, `1` connected component, `0` duplicate triangles,
  watertight `true`.
- Exported STL:
  `dual_contouring\direct_sparse_sdf_mc_experiment\stl\direct_sdf_oml_fcv01_long_glider_faired_cap_spacing_1p0.stl`.
- Generated section-cut evidence:
  `dual_contouring\direct_sparse_sdf_mc_experiment\plots\inlet_cap_section_cuts\fcv01_full_path_solid_optimizer_inlet_v2`.
- Generated inlet closeup renders:
  `dual_contouring\direct_sparse_sdf_mc_experiment\views\fcv01_full_path_solid_optimizer_inlet_v2_closeup.png`
  and
  `dual_contouring\direct_sparse_sdf_mc_experiment\views\fcv01_full_path_solid_optimizer_inlet_v2_closeup_no_edges.png`.
- Visual interpretation: the YZ section sweep now shows the solid inlet body
  continues through the buried path instead of stopping at the first fuselage
  contact. No outlet/exhaust cut is present in `faired_cap`.
- Do not export the other four capped-inlet variants until the user approves
  this full-path `fcv01` geometry.

## 2026-06-23 - Approved Full-Path Faired-Cap Variant Export Set

- User approved the full-path solid optimizer inlet direction after visual
  review of `fcv01_full_path_solid_optimizer_inlet_v2`.
- Reviewed cap sizing before propagating to the full set:
  `intake_outer_height = 60 mm`, `intake_outer_width = 88 mm`,
  `cap_length = 60 mm`, `cap_width = 88 mm`, and
  `cap_height = 8.4 mm` (`14%` of inlet height). This remains a shallow
  optimizer-CFD fairing, not a production inlet-detail model.
- Kept the current rounded-rect cap primitive. The face is not perfectly
  tangent everywhere after union with the solid inlet body, but the visible
  edge is localized to the capped inlet face and is acceptable for the current
  aircraft-level external-flow screening geometry.
- Re-exported all five full-path `faired_cap` variants with:
  `python direct_sparse_sdf_mc_experiment\scripts\export_faired_cap_variants.py`.
- All five exports completed with `status = passed`, `boundary_edges = 0`,
  `nonmanifold_edges = 0`, `connected_components = 1`,
  `duplicate_triangles = 0`, and `watertight = true`.
- Refreshed STL outputs:
  - `fcv01_long_glider`:
    `dual_contouring\direct_sparse_sdf_mc_experiment\stl\direct_sdf_oml_fcv01_long_glider_faired_cap_spacing_1p0.stl`
    (`1,355,886` triangles).
  - `fcv02_short_swept`:
    `dual_contouring\direct_sparse_sdf_mc_experiment\stl\direct_sdf_oml_fcv02_short_swept_faired_cap_spacing_1p0.stl`
    (`1,025,030` triangles).
  - `fcv03_high_aspect_mild`:
    `dual_contouring\direct_sparse_sdf_mc_experiment\stl\direct_sdf_oml_fcv03_high_aspect_mild_faired_cap_spacing_1p0.stl`
    (`1,290,378` triangles).
  - `fcv04_compact_wide_tail`:
    `dual_contouring\direct_sparse_sdf_mc_experiment\stl\direct_sdf_oml_fcv04_compact_wide_tail_faired_cap_spacing_1p0.stl`
    (`1,279,706` triangles).
  - `fcv05_aft_wing_fast`:
    `dual_contouring\direct_sparse_sdf_mc_experiment\stl\direct_sdf_oml_fcv05_aft_wing_fast_faired_cap_spacing_1p0.stl`
    (`1,212,018` triangles).
- Refreshed run summary:
  `dual_contouring\direct_sparse_sdf_mc_experiment\logs\faired_cap_variant_export_run_summary.json`.
- Generated visual review images:
  `dual_contouring\direct_sparse_sdf_mc_experiment\views\faired_cap_full_path_v2`.
- Inlet closeup contact sheet:
  `dual_contouring\direct_sparse_sdf_mc_experiment\views\faired_cap_full_path_v2\inlet_closeup_contact_sheet.png`.
- Current status: the full-path capped-inlet STL set is ready for mesher
  comparison. Next run roots should be fresh and should not reuse the stale
  no-inlet or earlier capped-inlet comparison evidence.

## 2026-06-23 - Gmsh/OpenFOAM Full-Path Faired-Cap Mesher Evidence Received

- Received Gmsh/OpenFOAM run root:
  `custom_cfd_mesher_experiment\runs\faired_cap_gmsh_openfoam_20260623`.
- Top-level reports:
  `comparison_summary.md`, `comparison_summary.json`, and
  `potential_foam_rerun_commands.md`.
- Important run note: the first `potentialFoam -writep` setup had an invalid
  `0/p` boundary dictionary. The cases were not remeshed; only `0/U`, `0/p`,
  `potentialFoam`, and potential summaries were regenerated after the writer
  fix.
- `fcv03_high_aspect_mild` original Gmsh run failed in `gmsh_pipeline`; the
  comparison summary correctly uses the successful
  `fcv03_high_aspect_mild_retry_5mm` case.
- Gmsh verdict from supplied evidence: all five approved full-path
  faired-cap variants are `pass_for_mesher_plumbing`, strict `checkMesh =
  pass`, and `potentialFoam = complete`.
- Summary metrics:
  - `fcv01_long_glider`: `749,233` cells, `90,434` aircraft faces,
    max skew `2.718`, max non-orthogonality `82.72`, severe >70 faces `67`,
    source-to-remesh p95/p99/max `0.0626/0.0993/0.1897 mm`.
  - `fcv02_short_swept`: `590,292` cells, `67,488` aircraft faces,
    max skew `2.526`, max non-orthogonality `83.64`, severe >70 faces `44`,
    source-to-remesh p95/p99/max `0.0672/0.1042/0.1852 mm`.
  - `fcv03_high_aspect_mild_retry_5mm`: `564,165` cells, `57,680`
    aircraft faces, max skew `2.567`, max non-orthogonality `82.94`,
    severe >70 faces `93`, source-to-remesh p95/p99/max
    `0.0915/0.1412/0.2459 mm`.
  - `fcv04_compact_wide_tail`: `793,477` cells, `85,898` aircraft faces,
    max skew `2.711`, max non-orthogonality `81.40`, severe >70 faces `79`,
    source-to-remesh p95/p99/max `0.0639/0.1015/0.1615 mm`.
  - `fcv05_aft_wing_fast`: `663,539` cells, `80,598` aircraft faces,
    max skew `2.538`, max non-orthogonality `78.58`, severe >70 faces `47`,
    source-to-remesh p95/p99/max `0.0673/0.1035/0.1738 mm`.
- Boundary-layer/prism status for all Gmsh cases:
  `none_tetrahedral_slip_wall_smoke`. These are solver-plumbing meshes, not
  scoring-CFD meshes.
- Created local visual contact sheet:
  `custom_cfd_mesher_experiment\runs\faired_cap_gmsh_openfoam_20260623\gmsh_aircraft_iso_contact_sheet.png`.
- Preliminary interpretation before snappy evidence arrives: Gmsh now looks
  robust for automated OpenFOAM plumbing on the approved full-path
  faired-cap family, with strong surface-deviation numbers and completed
  potentialFoam smoke runs. Main caveats are no boundary layers, slip wall
  smoke setup only, and persistent severe non-orthogonal faces despite strict
  `checkMesh` passing.

## 2026-06-23 - Five Faired-Cap Inlet Gmsh/OpenFOAM Comparison Run

- Ran a fresh Gmsh/OpenFOAM comparison root for the approved full-path
  faired-cap inlet STL set:
  `custom_cfd_mesher_experiment\runs\faired_cap_gmsh_openfoam_20260623`.
- Inputs were the current `fcv01` through `fcv05` full-path solid faired-cap
  STLs in
  `dual_contouring\direct_sparse_sdf_mc_experiment\stl`.
- Meshing path: Gmsh `4.15.2`, tetrahedral volume mesh, dynamic farfield
  sizing, split farfield patches, OpenFOAM `13`, aircraft patch configured as
  slip for `potentialFoam -writep`.
- Added/updated automation:
  - `custom_cfd_mesher_experiment\scripts\run_faired_cap_gmsh_openfoam_comparison.py`.
  - `custom_cfd_mesher_experiment\scripts\setup_potential_foam_smoke.py`.
- The first `potentialFoam` attempt exposed a case-writer bug in `0/p`, not a
  mesh failure. Fixed the writer and reran `potentialFoam` on the same meshes
  without remeshing. Rerun details are preserved in
  `custom_cfd_mesher_experiment\runs\faired_cap_gmsh_openfoam_20260623\potential_foam_rerun_commands.md`.
- `fcv03_high_aspect_mild` failed the initial 4 mm Gmsh 3D generation attempt;
  the failure directory is preserved. A documented 5 mm retry succeeded and is
  the selected `fcv03` result for the comparison summary.
- Selected results:
  - `fcv01_long_glider`: `749,233` cells, `90,434` aircraft faces, max skew
    `2.7177439`, max non-orthogonality `82.721159`, severe non-orth faces
    `67`, strict `checkMesh` pass, `potentialFoam` complete.
  - `fcv02_short_swept`: `590,292` cells, `67,488` aircraft faces, max skew
    `2.5259618`, max non-orthogonality `83.644565`, severe non-orth faces
    `44`, strict `checkMesh` pass, `potentialFoam` complete.
  - `fcv03_high_aspect_mild` selected 5 mm retry: `564,165` cells, `57,680`
    aircraft faces, max skew `2.5671852`, max non-orthogonality `82.943344`,
    severe non-orth faces `93`, strict `checkMesh` pass, `potentialFoam`
    complete.
  - `fcv04_compact_wide_tail`: `793,477` cells, `85,898` aircraft faces, max
    skew `2.7105344`, max non-orthogonality `81.396964`, severe non-orth faces
    `79`, strict `checkMesh` pass, `potentialFoam` complete.
  - `fcv05_aft_wing_fast`: `663,539` cells, `80,598` aircraft faces, max skew
    `2.5376748`, max non-orthogonality `78.579953`, severe non-orth faces
    `47`, strict `checkMesh` pass, `potentialFoam` complete.
- Source-to-remesh deviation p95/p99/max for the selected five cases ranged
  from approximately `0.063/0.099/0.162 mm` to `0.091/0.141/0.246 mm`.
- Generated aircraft-only ISO screenshots and per-variant `variant_report.json`
  files for all selected cases.
- Summary artifacts:
  `custom_cfd_mesher_experiment\runs\faired_cap_gmsh_openfoam_20260623\comparison_summary.md`
  and
  `custom_cfd_mesher_experiment\runs\faired_cap_gmsh_openfoam_20260623\comparison_summary.json`.
- Decision status: Gmsh now passes as a repeatable faired-cap full-aircraft
  mesher-plumbing path across the selected five-variant set. It is still not
  scoring-quality CFD because there are no prism/boundary-layer cells, the
  aircraft patch is slip for the smoke test, and `potentialFoam` results are
  not drag/lift scoring data.

## 2026-06-23 - Snappy Faired-Cap Five-Variant Evidence and Mesher Direction

- Reviewed snappyHexMesh/OpenFOAM evidence package:
  `su2_sandbox\runs\openfoam_snappy_faired_cap_inlets_v0_1_20260623_skew8_v2`.
- Visual artifacts reviewed:
  `aircraft_iso_contact_sheet.png` and `inlet_faired_cap_contact_sheet.png`.
- Snappy command family used OpenFOAM-13 `snappyHexMesh`, surface levels `4-5`,
  feature level `4`, max global cells `14,000,000`, acceptable relaxed skewness
  `8.0`, freestream `50 mph`, and `potentialFoam -writep`.
- Snappy results:
  - `fcv01_long_glider`: `760,820` cells, max skew `7.9435439`, max non-orth
    `74.809369`, severe non-orth faces `9`, strict `checkMesh` failed,
    relaxed development accepted, `potentialFoam` complete, runtime `578.2 s`.
  - `fcv02_short_swept`: `662,071` cells, max skew `5.648455`, max non-orth
    `74.999556`, severe non-orth faces `6`, strict `checkMesh` failed,
    relaxed development accepted, `potentialFoam` complete, runtime `899.6 s`.
  - `fcv03_high_aspect_mild`: `731,708` cells, max skew `5.6934682`, max
    non-orth `74.854602`, severe non-orth faces `13`, strict `checkMesh`
    failed, relaxed development accepted, `potentialFoam` complete, runtime
    `1256.4 s`.
  - `fcv04_compact_wide_tail`: `764,188` cells, max skew `5.4198904`, max
    non-orth `74.790403`, severe non-orth faces `2`, strict `checkMesh`
    failed, relaxed development accepted, `potentialFoam` complete, runtime
    `1268.2 s`.
  - `fcv05_aft_wing_fast`: `708,235` cells, max skew `6.4277317`, max non-orth
    `74.982503`, severe non-orth faces `5`, strict `checkMesh` failed,
    relaxed development accepted, `potentialFoam` complete, runtime `714.1 s`.
- Snappy source-to-patch p95 deviation is good in the aggregate
  (`0.0478-0.0519 mm`), but max deviation is much worse than Gmsh
  (`0.739-1.789 mm`) and the aircraft/inlet images remain visibly faceted.
- Side-by-side mesher decision:
  - Gmsh selected set: all five strict mesh gates pass, all five
    `potentialFoam` smoke tests complete, total selected runtime `764.7 s`.
  - Snappy selected set: all five `potentialFoam` smoke tests complete, but all
    five strict `checkMesh` checks fail and require relaxed skewness policy;
    total runtime `4716.5 s`.
  - Gmsh max skewness range: approximately `2.53-2.72`.
  - Snappy max skewness range: approximately `5.42-7.94`.
  - Gmsh non-orthogonality/severe non-orth counts are not solved yet, but they
    are currently less blocking than snappy's repeated strict-check failure and
    runtime cost.
- Decision status: use Gmsh as the near-term default mesher-plumbing path for
  optimizer integration. Keep snappy as a secondary research path only,
  especially if future `addLayers`/hex-dominant boundary-layer work becomes
  valuable. Neither path is scoring-quality CFD yet because both are still
  no-boundary-layer, slip-wall `potentialFoam` plumbing runs.

## 2026-06-23 - Gmsh v0.2 Hardening Report Reviewed

- Reviewed:
  `custom_cfd_mesher_experiment\runs\gmsh_optimizer_hardening_v0_2_20260623\GMSH_NEXT_STEP_REPORT.md`.
- Reviewed preset:
  `custom_cfd_mesher_experiment\runs\gmsh_optimizer_hardening_v0_2_20260623\gmsh_optimizer_preset_v0_2.json`.
- Final Gmsh recommendation: `keep_current_preset_with_formalized_v0_2_policy`.
- Formal preset name: `gmsh_openfoam_external_aero_v0_2`.
- Primary policy:
  - Keep the current 4 mm primary surface/remesh sizing.
  - Keep `default,Netgen,Relocate3D` optimization.
  - Use automatic 5 mm retry if 4 mm Gmsh volume generation crashes or fails.
  - Keep `potentialFoam -writep` as the required plumbing smoke test.
  - Keep `scoring_allowed=false`.
- Development gates in the preset:
  - strict `checkMesh` required.
  - required patches: `aircraft`, `inlet`, `outlet`, `side_ymin`,
    `side_ymax`, `side_zmin`, `side_zmax`.
  - `defaultFaces` must be `0`.
  - max skewness gate `4.0`.
  - max non-orthogonality gate `89.0`.
  - max severe non-orth faces gate `250`.
  - recommended surface fidelity limits: p95 `<=0.12 mm`, p99 `<=0.18 mm`.
- Bounded improvement sweep result:
  - `4.5 mm + default,Netgen,Relocate3D` crashed on `fcv01`.
  - `4.5 mm + default,Relocate3D` without Netgen generated a strict
    `checkMesh`-OK mesh for `fcv01` but failed patch-map policy with
    `defaultFaces=16` and worsened severe non-orth faces to `237`.
  - `4.5 mm + default,Netgen,Relocate3D` improved `fcv03` severe non-orth faces
    from `93` to `82`, but is rejected as a global default because the same
    setting crashed on `fcv01`.
- Non-orthogonality localization:
  - dominant worst region is `fuselage_tail` for `fcv01`, `fcv02`, `fcv03`,
    and `fcv05`.
  - dominant worst region is `wingtip` for `fcv04`.
  - report states the selected severe non-orth clusters are aircraft-feature
    adjacent, not farfield/block-transition dominated.
  - localization artifacts are available as `nonOrthoFaces.vtk`,
    `nonOrthoFace_centroids.csv`, and overlay screenshots under the
    Gmsh follow-up run tree.
- Scoring-CFD draft:
  - first scoring candidate should be OpenFOAM `simpleFoam`.
  - switch aircraft wall from slip to no-slip for scoring attempts.
  - start with `kOmegaSST`.
  - report `Cl`, `Cd`, `Cm`, raw forces/moments, coefficient histories,
    residuals, continuity, and final-window averages.
  - trusted drag/lift remains blocked until boundary-layer/prism cells and a
    wall-treatment/y+ policy exist.
- Decision status: adopt `gmsh_openfoam_external_aero_v0_2` as the first
  optimizer mesh-backend contract for OpenFOAM plumbing, but do not promote it
  to scoring CFD.

## 2026-06-23 - Gmsh Follow-Up: Feature Closeups, Non-Orth Localization, fcv03 Improvement

- Added follow-up utility:
  `custom_cfd_mesher_experiment\scripts\generate_gmsh_followup_report.py`.
- Generated aircraft-only feature closeups for all five selected Gmsh
  faired-cap variants:
  `inlet_cap_body`, `wing_leading_edge`, `wing_trailing_edge`,
  `wing_root_fuselage_blend`, `wingtip_left`, `wingtip_right`, and
  `tail_root_blend`.
- Reran `checkMesh -writeSurfaces -writeSets -nonOrthThreshold 70` on all five
  selected cases and preserved `nonOrthoFaces.vtk`, top/ISO bad-face overlays,
  and JSON localization summaries under:
  `custom_cfd_mesher_experiment\runs\faired_cap_gmsh_openfoam_20260623\followup_gmsh_geometry_nonortho`.
- Severe-face localization summary:
  - `fcv01_long_glider`: `67` severe faces, dominant heuristic location
    `tail`.
  - `fcv02_short_swept`: `44` severe faces, dominant heuristic location
    `tail`.
  - `fcv03_high_aspect_mild`: `93` severe faces, dominant heuristic location
    `tail`.
  - `fcv04_compact_wide_tail`: `79` severe faces, dominant heuristic location
    `wingtip`.
  - `fcv05_aft_wing_fast`: `47` severe faces, dominant heuristic location
    `tail`.
- No selected case localized severe non-orthogonal faces to the farfield
  transition; clusters are feature-adjacent and matter mainly for scoring-CFD
  promotion, not `potentialFoam` plumbing.
- Ran one bounded improvement case for `fcv03_high_aspect_mild`:
  `target_edge_mm = 4.5`, `surface_size = 0.0045 m`,
  `max_surface_distance_mm = 0.18`, same dynamic farfield policy.
- Improvement result:
  `618,052` cells, `124,156` points, `69,382` aircraft faces, max skewness
  `2.5360147`, max non-orthogonality `82.928003`, `82` severe faces,
  strict `checkMesh` pass, and `potentialFoam` complete.
- Compared to the selected 5 mm `fcv03` case, the 4.5 mm case reduced severe
  faces from `93` to `82` and improved remesher-reported source-to-remesh
  p95/p99 deviation from `0.0915/0.1412 mm` to `0.0813/0.1259 mm`, but raised
  cell count from `564,165` to `618,052`. It is useful as an optional
  higher-fidelity rerun, not worth adopting as the default yet.
- Final follow-up report:
  `custom_cfd_mesher_experiment\runs\faired_cap_gmsh_openfoam_20260623\followup_gmsh_geometry_nonortho\gmsh_followup_final_report.md`.
- Final JSON summary:
  `custom_cfd_mesher_experiment\runs\faired_cap_gmsh_openfoam_20260623\followup_gmsh_geometry_nonortho\gmsh_followup_final_summary.json`.
- OpenFOAM readiness conclusion: `simpleFoam` with `kOmegaSST`, no-slip
  aircraft, split farfield BCs, and force coefficients is the next reasonable
  non-scoring experiment. Meaningful drag scoring still requires a
  boundary-layer/prism-layer strategy and wall-treatment policy.

## 2026-06-23 - Gmsh Optimizer Backend Hardening v0.2

- Generated the Gmsh/OpenFOAM optimizer hardening package:
  `custom_cfd_mesher_experiment\runs\gmsh_optimizer_hardening_v0_2_20260623`.
- Final report:
  `custom_cfd_mesher_experiment\runs\gmsh_optimizer_hardening_v0_2_20260623\GMSH_NEXT_STEP_REPORT.md`.
- Formalized preset:
  `custom_cfd_mesher_experiment\runs\gmsh_optimizer_hardening_v0_2_20260623\gmsh_optimizer_preset_v0_2.json`.
- Comparison matrix:
  `custom_cfd_mesher_experiment\runs\gmsh_optimizer_hardening_v0_2_20260623\comparison_matrix.csv`
  and
  `custom_cfd_mesher_experiment\runs\gmsh_optimizer_hardening_v0_2_20260623\comparison_matrix.json`.
- Per-variant selected-policy reports:
  `custom_cfd_mesher_experiment\runs\gmsh_optimizer_hardening_v0_2_20260623\selected_v0_2_policy_baseline`.
- Updated Gmsh non-orthogonality localization to write
  `nonOrthoFace_centroids.csv` and feature-region summaries using the
  optimizer-facing feature labels: `inlet_cap`, `inlet_side_wall`,
  `wing_leading_edge`, `wing_trailing_edge`, `wing_root_blend`, `wingtip`,
  `tail`, `fuselage_nose`, `fuselage_tail`, `farfield_wake_region`, and
  `random_volume_cells`.
- Selected recommendation:
  `keep_current_preset_with_formalized_v0_2_policy`.
- v0.2 policy:
  - Primary Gmsh baseline remains `4 mm` target edge/surface sizing with
    `default,Netgen,Relocate3D`.
  - Automatic retry is `5 mm` target edge/surface sizing when 4 mm Gmsh volume
    generation fails; this is validated by the selected `fcv03` result.
  - Required gates remain strict `checkMesh`, clean expected patch mapping,
    `defaultFaces = 0`, severe non-orthogonality localization artifacts,
    aircraft-only screenshots, and successful `potentialFoam -writep`.
  - Path remains `scoring_allowed=false`.
- Bounded improvement probes:
  - `4.5 mm + default,Netgen,Relocate3D` crashed on `fcv01_long_glider`
    during Gmsh/Netgen optimization; preserved under
    `failed_probe_4p5_default_netgen_relocate3d`.
  - `4.5 mm + default,Relocate3D` without Netgen generated a strict
    `checkMesh`-OK `fcv01` mesh, but failed the platform patch gate with
    `defaultFaces=16` and worsened severe non-orthogonality to `237`.
  - The earlier `fcv03` 4.5 mm improvement remains useful as a bounded
    single-variant diagnostic, but is not adopted as the global default.
- Severe non-orthogonality localization on the selected five-variant baseline:
  - `fcv01_long_glider`: `67` severe faces, dominant `fuselage_tail`.
  - `fcv02_short_swept`: `44` severe faces, dominant `fuselage_tail`.
  - `fcv03_high_aspect_mild`: `93` severe faces, dominant `fuselage_tail`.
  - `fcv04_compact_wide_tail`: `79` severe faces, dominant `wingtip`.
  - `fcv05_aft_wing_fast`: `47` severe faces, dominant `fuselage_tail`.
- No selected baseline case has the dominant severe non-orthogonal cluster in
  the farfield/block transition. The unresolved issue is aircraft-feature-
  adjacent non-orthogonality, which is acceptable for plumbing but still a
  scoring-CFD promotion risk.
- OpenFOAM next-step draft is `simpleFoam` with `kOmegaSST`, no-slip aircraft,
  split farfield boundary conditions, `forceCoeffs`, geometry-derived
  `Aref/Lref/CofR`, drag `+X`, lift `+Z`, pitch `+Y`, and final-window
  coefficient reporting.
- Drag/lift scoring remains blocked until boundary-layer/prism strategy,
  wall-treatment/y+ policy, no-slip solver stability, and force/moment
  convergence checks are working.

## 2026-06-23 - First Gmsh/OpenFOAM No-Slip Steady Solver Plumbing Run

- Created the first bounded no-slip steady OpenFOAM solver experiment on the
  selected Gmsh v0.2 backend:
  `custom_cfd_mesher_experiment\runs\gmsh_openfoam_simplefoam_plumbing_v0_1_20260623`.
- Run report:
  `custom_cfd_mesher_experiment\runs\gmsh_openfoam_simplefoam_plumbing_v0_1_20260623\SIMPLEFOAM_PLUMBING_REPORT.md`.
- Variant: `fcv01_long_glider`.
- Source mesh case:
  `custom_cfd_mesher_experiment\runs\faired_cap_gmsh_openfoam_20260623\fcv01_long_glider\gmsh_mesh\gmsh\openfoam_case`.
- New no-slip case:
  `custom_cfd_mesher_experiment\runs\gmsh_openfoam_simplefoam_plumbing_v0_1_20260623\fcv01_long_glider\openfoam_case_incompressible_komega_20`.
- Solver path: OpenFOAM 13 `foamRun -solver incompressibleFluid`, steady
  incompressible simpleFoam-class plumbing.
- Setup: `kOmegaSST`, aircraft `noSlip`, split farfield patches, 50 mph
  freestream, force coefficients enabled, drag `+X`, lift `+Z`, pitch `+Y`.
- Mesh check before solve:
  - `checkMesh`: `Mesh OK`.
  - Cells: `749,233`.
  - Points: `152,161`.
  - Max aspect ratio: `128.98031`.
  - Max non-orthogonality: `82.721159`.
  - Max skewness: `2.7177439`.
- Solver result:
  - Completed `20` steps.
  - Fatal error: no.
  - Floating point exception: no.
  - OpenFOAM warnings: `0`.
  - Execution time: `81.967068 s`.
  - Clock time: `93 s`.
  - Last pressure residual: initial `8.6029658e-04`, final `3.5582173e-05`.
  - Last velocity residual: initial `4.7952705e-02`, final `7.8987134e-04`.
- Force coefficient output was produced, but failed stability checks:
  - Final row: `Cd = -2.95776509`, `Cl = 0.0805856235`,
    `Cm = -0.24218945`.
  - Late-window coefficient drift is large and `Cd` changes sign.
- Decision status: conditional plumbing pass only. This proves the no-slip
  OpenFOAM case generation, field dictionaries, turbulence setup, solver
  launch, and force-coefficient plumbing can run on the accepted Gmsh mesh. It
  does not produce trusted aerodynamic scoring data.
- New recommended optimizer persistence label:
  `openfoam_steady_plumbing`, with `scoring_allowed=false`.

## 2026-06-23 - No-Slip CFD Coefficient Instability Diagnosis

- Added focused diagnosis report:
  `custom_cfd_mesher_experiment\runs\gmsh_openfoam_simplefoam_plumbing_v0_1_20260623\CFD_COEFFICIENT_DIAGNOSIS_REPORT.md`.
- Verified the coordinate and force convention from the actual `fcv01` Gmsh
  case:
  - `X`: aircraft length / freestream direction.
  - `Y`: wingspan direction.
  - `Z`: vertical direction.
  - `dragDir = +X`, `liftDir = +Z`, `pitchAxis = +Y`.
  - Freestream velocity is `(22.352 0 0)` m/s.
- Actual aircraft bounds for `fcv01_long_glider`:
  - `X`: `-0.000010` to `0.760010 m`.
  - `Y`: `-0.400000` to `0.400000 m`.
  - `Z`: `-0.049989` to `0.143000 m`.
- Conclusion: the coefficient instability is not primarily an axis/sign
  convention problem.
- Ran OpenFOAM wall diagnostics on the `kOmegaSST` no-slip case:
  - `foamPostProcess -solver incompressibleFluid -func yPlus -latestTime`.
  - Aircraft patch y+: min `0.95845508`, max `324.81977`, average
    `32.226444`.
  - This is a mixed, uncontrolled wall-treatment regime and is not acceptable
    for scoring drag.
- Ran a laminar no-slip diagnostic on the same mesh/farfield:
  `custom_cfd_mesher_experiment\runs\gmsh_openfoam_simplefoam_plumbing_v0_1_20260623\fcv01_long_glider\openfoam_case_incompressible_laminar_20`.
- Laminar diagnostic result:
  - Completed `20` steps.
  - Fatal error: no.
  - Floating point exception: no.
  - Last pressure residual final `2.4931254e-05`.
  - Last velocity residual final `1.8090071e-04`.
  - Final coefficients: `Cd = 0.21356223`, `Cl = 0.0785798676`,
    `Cm = -0.00905222554`.
  - Coefficient drift still fails the stability gate, but the result is much
    calmer than the `kOmegaSST` case.
- Diagnosis ranking:
  1. Missing boundary-layer/prism mesh.
  2. Mixed/uncontrolled y+ and wall-function behavior.
  3. Turbulence wall treatment sensitivity.
  4. Feature-adjacent mesh quality near trailing edges, roots, tips, and tail.
  5. Uniform startup initialization.
  6. Rough reference values.
  7. Axis/sign convention, currently low-likelihood.
- Decision: keep no-slip OpenFOAM runs as `openfoam_steady_plumbing` only.
  Require y+ reporting and coefficient stability gates before any coefficient
  can influence optimizer scoring.

## 2026-06-23 - CFD Gates and Gmsh Boundary-Layer Pilot

- Added optimizer-facing CFD gates in
  `custom_cfd_mesher_experiment/scripts/cfd_mesh_gates.py`:
  - `axis-reference` gate for aircraft bounds, axis convention, reference area,
    reference length, coefficient directions, and center of rotation.
  - `yplus` gate for parsed OpenFOAM `foamPostProcess -func yPlus` logs with
    `low_re`, `wall_function`, `custom`, and `observe` policies.
- Verified the axis/reference convention on `fcv01_long_glider`:
  - `X`: fuselage length / drag direction.
  - `Y`: wingspan / pitch axis.
  - `Z`: vertical / lift direction.
  - Gate passed with `dragDir=(1,0,0)`, `liftDir=(0,0,1)`,
    `pitchAxis=(0,1,0)`, `Aref=0.12`, `Lref=0.7111267`.
  - Artifact:
    `custom_cfd_mesher_experiment/runs/gmsh_openfoam_simplefoam_plumbing_v0_1_20260623/fcv01_long_glider/axis_reference_gate.json`.
- Verified the first y+ gate on the previous no-slip `kOmegaSST` plumbing run:
  - Aircraft y+: min `0.95845508`, max `324.81977`, average `32.226444`.
  - `wall_function` policy fails, as expected, because y+ is mixed and
    uncontrolled.
  - `observe` policy passes for non-scoring diagnostics.
  - Artifacts:
    `custom_cfd_mesher_experiment/runs/gmsh_openfoam_simplefoam_plumbing_v0_1_20260623/fcv01_long_glider/yplus_wall_function_gate.json`
    and
    `custom_cfd_mesher_experiment/runs/gmsh_openfoam_simplefoam_plumbing_v0_1_20260623/fcv01_long_glider/yplus_observe_gate.json`.
- Fixed the boundary-layer helper's farfield-box API call in
  `custom_cfd_mesher_experiment/scripts/gmsh_boundary_layer_mesher.py`.
- Ran bounded Gmsh boundary-layer/prism pilots under
  `custom_cfd_mesher_experiment/runs/gmsh_prism_layer_pilot_v0_1_20260623`.
- Prism pilot result:
  - Recombined prism attempts without suppression failed inside Gmsh with
    extrusion / 3D PLC errors.
  - A no-recombine attempt generated tetrahedra only and failed strict
    OpenFOAM `checkMesh` with open cells, negative volumes, wrong-oriented
    faces, high non-orthogonality, and skewness.
  - A suppressed, very coarse recombined attempt generated mixed Gmsh
    tetra/prism elements, but OpenFOAM conversion failed strict `checkMesh`:
    invalid vertex-label faces, disconnected regions, zero-area faces,
    negative/zero-volume cells, and highly skew cells. Conversion without
    `-keepOrientation` still failed and reported many inverted prisms.
  - Best prism pilot artifact root:
    `custom_cfd_mesher_experiment/runs/gmsh_prism_layer_pilot_v0_1_20260623/fcv01_bl_4k_h00008_recombine_suppressed`.
- Decision:
  - Keep the selected Gmsh tetra/farfield path as the near-term default
    OpenFOAM plumbing mesher.
  - Do not promote Gmsh prism/boundary-layer extrusion yet.
  - Treat no-slip OpenFOAM coefficients as plumbing diagnostics only until a
    valid boundary-layer strategy and y+ policy pass.

## 2026-06-23 - Fast snappyHexMesh Boundary-Layer Pilot

- Added opt-in snappy layer controls to
  `custom_cfd_mesher_experiment/scripts/make_openfoam_case.py`.
  - Default behavior remains unchanged: `addLayers false` unless explicitly
    requested.
  - New controls include surface-layer count, layer thickness, expansion ratio,
    layer feature angle, relaxed iterations, mesh-quality limits, and optional
    local refinement boxes.
- Ran bounded `fcv01_long_glider` pilots under
  `custom_cfd_mesher_experiment/runs/snappy_layer_pilot_v0_1_20260623`.
- Best current layered case:
  `custom_cfd_mesher_experiment/runs/snappy_layer_pilot_v0_1_20260623/fcv01_layers2_tiprefine_fast`.
- Best case meshing result:
  - `snappyHexMesh` runtime: `64.25 s`; total meshing/check wrapper runtime:
    about `89.5 s`.
  - Cells: `114408`.
  - Points: `134138`.
  - Aircraft patch faces: `7996`.
  - Hexahedra: `102476`.
  - Prisms: `1281`.
  - Polyhedra: `10640`.
  - Max aspect ratio: `65.582418`.
  - Max non-orthogonality: `69.9966`.
  - Max skewness: `3.934065`.
  - Strict `checkMesh`: `Mesh OK`.
  - Aircraft-only ISO screenshot:
    `custom_cfd_mesher_experiment/runs/snappy_layer_pilot_v0_1_20260623/fcv01_layers2_tiprefine_fast/aircraft_iso.png`.
- Local refinement note:
  - Initial layered snappy runs produced one skew face at about `4.09`, located
    near the right wingtip.
  - Adding small mirrored wingtip refinement boxes cleared the strict
    `checkMesh` failure without a large runtime increase.
- Solver plumbing:
  - `potentialFoam -writep` completed in about `3.34 s`.
  - Continuity error: `3.4731093e-07`.
  - Interpolated velocity error: `0.00022023224`.
  - No warnings or fatal errors.
- Short no-slip `kOmegaSST` plumbing run:
  - Completed `20` steps in about `11.02 s`.
  - No fatal error and no floating point exception.
  - Final pressure residual: `4.6110436e-05`.
  - Final velocity residual: `9.554966e-04`.
  - Coefficients are unstable and remain non-scoring.
- y+ result on the layered mesh:
  - Aircraft y+: min `0.98656759`, max `1918.7168`, average `59.473735`.
  - `observe` policy passes for recording.
  - `wall_function` policy fails because min is below `30` and max is above
    `300`.
- Decision:
  - Promote this as a promising boundary-layer research branch, not as scoring
    CFD.
  - Keep Gmsh tetra/farfield as the fast default OpenFOAM plumbing mesher for
    now.
  - Next boundary-layer work should improve layer coverage / y+ control before
    coefficient scoring is allowed.

## 2026-06-23 - Five-Variant snappy Layered Backend Trial

- Added a reusable five-variant runner:
  `custom_cfd_mesher_experiment/scripts/run_snappy_layer_comparison.py`.
- Runner behavior:
  - Uses the prepared aircraft surfaces from
    `custom_cfd_mesher_experiment/runs/faired_cap_gmsh_openfoam_20260623`.
  - Generates an OpenFOAM case with opt-in `snappyHexMesh` layers.
  - Runs `surfaceCheck`, `blockMesh`, `surfaceFeatures`, `snappyHexMesh`,
    strict `checkMesh`, `potentialFoam -writep`, compact JSON summaries, and
    aircraft-only ISO screenshots.
  - Supports retrying failed variants with known hotspot refinement boxes.
- First five-variant run:
  `custom_cfd_mesher_experiment/runs/snappy_layer_five_variant_v0_1_20260623`.
  - Runtime: `484.6 s`.
  - Strict layered mesh pass count: `1/5`.
  - All five completed `potentialFoam`.
  - Four failures were single-check skewness failures; no topology failures.
- Hotspot retry run:
  `custom_cfd_mesher_experiment/runs/snappy_layer_five_variant_v0_1_hotspot_retry_20260623`.
  - Retried the four failed cases with localized refinement boxes.
  - `fcv02_short_swept` passed.
  - `fcv03_high_aspect_mild` passed.
  - `fcv04_compact_wide_tail` still failed skewness.
  - `fcv05_aft_wing_fast` still failed skewness.
- Final bounded retry run:
  `custom_cfd_mesher_experiment/runs/snappy_layer_five_variant_v0_1_final_hotspot_retry_20260623`.
  - Retried `fcv04_compact_wide_tail` and `fcv05_aft_wing_fast` with stronger
    local hotspot boxes.
  - `fcv05_aft_wing_fast` passed.
  - `fcv04_compact_wide_tail` still failed skewness and worsened with extra
    local refinement.
- Best current layered results:
  - `fcv01_long_glider`: pass, `115250` cells, max skew `3.9339299`.
  - `fcv02_short_swept`: pass, `107115` cells, max skew `3.3518884`.
  - `fcv03_high_aspect_mild`: pass, `121040` cells, max skew `3.8794142`.
  - `fcv04_compact_wide_tail`: fail, best observed max skew `5.4009313`.
  - `fcv05_aft_wing_fast`: pass, `117038` cells, max skew `3.433431`.
- Decision:
  - snappy layered meshing is promising and fast enough for a secondary
    boundary-layer backend, but it is not yet robust enough to replace the
    selected Gmsh tetra/farfield backend.
  - Do not chase `fcv04` further with manual hotspot boxes in this pass.
  - Next useful work is to automate skew-hotspot localization/refinement or test
    a slightly different snappy base/refinement policy for compact/wide-tail
    layouts.

## 2026-06-23 - Rejected Global Conservative snappy Layer Preset

- Tested a heavier first-pass snappy layer preset without learned hotspot boxes:
  `custom_cfd_mesher_experiment/runs/snappy_layer_conservative_nohotspot_v0_1_20260623`.
- Settings increased global cost and nominal fidelity:
  - Base cells: `52,32,52`.
  - Surface max level: `4`.
  - Feature level: `3`.
  - Cells between levels: `5`.
  - Snap tolerance: `0.4`.
  - Smoothing: `15`.
  - Two thinner layers.
- Result:
  - Run timed out before all five variants completed.
  - `fcv01_long_glider`: failed strict OpenFOAM quality after about `502 s`;
    `321453` cells, max non-orthogonality `74.996505`, max skewness
    `5.6209795`.
  - `fcv02_short_swept`: failed strict OpenFOAM quality after about `423 s`;
    `285859` cells, max non-orthogonality `74.934343`, max skewness
    `4.2159086`.
- Decision:
  - Reject global conservative snappy refinement as an optimizer default path.
  - It is slower and not more reliable.
  - The issue is not simply insufficient global resolution; it is local
    feature/transition quality around sharp aircraft geometry.
  - Any viable snappy-layer path needs first-pass geometry-aware local controls,
    not post-failure manual hotspot tuning and not blanket refinement.

## 2026-06-23 - Rejected Feature-Aware First-Pass snappy Layer Preset

- Added a geometry-derived feature refinement generator to
  `custom_cfd_mesher_experiment/scripts/run_snappy_layer_comparison.py`.
- The generator identifies high-dihedral STL feature edges before meshing,
  bins them spatially, and emits local refinement boxes without using any
  post-failure hotspot information.
- Tested all five approved faired-cap aircraft variants in one fresh run:
  `custom_cfd_mesher_experiment/runs/snappy_layer_feature_aware_firstpass_v0_1_20260623`.
- Command:
  `.\su2_sandbox\.venv\Scripts\python.exe custom_cfd_mesher_experiment\scripts\run_snappy_layer_comparison.py --run-root custom_cfd_mesher_experiment\runs\snappy_layer_feature_aware_firstpass_v0_1_20260623 --feature-refinement-boxes --feature-refinement-level 4 --feature-angle-deg 28 --feature-box-count 12 --feature-box-grid 8,8,5 --feature-box-padding-frac 0.035 --skip-screenshots`
- Result:
  - Runtime: about `737.8 s`.
  - Strict OpenFOAM mesh pass count: `0 / 5`.
  - All five cases still completed `potentialFoam`, so solver plumbing remained
    intact, but strict mesh quality failed on every case.
  - `fcv01_long_glider`: `161422` cells, max non-orthogonality `69.997724`,
    max skewness `4.3870624`.
  - `fcv02_short_swept`: `166700` cells, max non-orthogonality `69.995375`,
    max skewness `4.2983547`.
  - `fcv03_high_aspect_mild`: `172619` cells, max non-orthogonality
    `69.996274`, max skewness `4.8867419`.
  - `fcv04_compact_wide_tail`: `159440` cells, max non-orthogonality
    `69.999661`, max skewness `11.26525`.
  - `fcv05_aft_wing_fast`: `194507` cells, max non-orthogonality
    `69.997974`, max skewness `7.2702844`.
- Decision:
  - Reject this feature-box strategy as a first-pass snappy-layer default.
  - It increased cell count and runtime without improving strict quality.
  - The root issue is not just missing local refinement around visible STL
    features; the current snappy layer/snap/refinement interaction is producing
    poor local transition cells around sharp/thin aircraft features.
  - Do not use post-failure hotspot retries as an optimizer production path.
  - Keep Gmsh tetra/farfield as the near-term default OpenFOAM plumbing mesher.
  - Keep snappy layers as research only until it can pass all five variants on
    the first attempt with no learned hotspots.

## 2026-06-23 - Gmsh Boundary-Layer Prism Revisit

- Added boundary-layer controls to
  `custom_cfd_mesher_experiment/scripts/gmsh_boundary_layer_mesher.py`:
  - `--auto-orient-discrete-surfaces`
  - `--optimize none`
  - `--skip-layer-surface-boxes`
  - repeatable layer-surface selection reporting
- Added a repeatable pilot runner:
  `custom_cfd_mesher_experiment/scripts/run_gmsh_bl_pilot_matrix.py`.
- Important finding:
  - Gmsh `create-geometry` boundary-layer extrusion is not viable on the
    current aircraft STL surfaces. It creates a small number of invalid prism
    cells even with a single very thin layer.
  - The failure is not mainly layer height. `25 um`, `50 um`, and `100 um`
    one-layer cases all produced the same basic negative-volume/non-orthogonal
    failure pattern on `fcv01`.
  - The old coarse-prism failure was partly caused by surface decimation and
    handoff topology, but the deeper issue was `create-geometry` remeshing the
    classified STL surfaces before layer extrusion.
- Positive finding:
  - Gmsh `create-topology` with one recombined prism layer preserves the
    source/remeshed surface triangles, creates one prism per aircraft patch
    triangle, and can produce OpenFOAM-valid layered meshes.
- Best single-case result:
  - Run:
    `custom_cfd_mesher_experiment/runs/gmsh_bl_classification_matrix_v0_1_20260623/angle80_create-topology`.
  - Variant: `fcv01_long_glider`.
  - Mesh: `1107398` cells, `90434` prisms, aircraft patch `90434` faces.
  - Strict `checkMesh`: pass.
  - Max skewness: `3.8761905`.
  - Max non-orthogonality: `88.516724`; severe `>70 deg`: `1181`.
  - `potentialFoam`: completed with no warnings; continuity error
    `4.9427109e-06`, interpolated velocity error `2.8577374e-10`.
- Five-variant `create-topology` one-layer result:
  - Run:
    `custom_cfd_mesher_experiment/runs/gmsh_bl_five_variant_create_topology_v0_1_20260623`.
  - `fcv01_long_glider`: strict pass, `1107398` cells, `90434` prisms,
    max skewness `3.8761905`, `potentialFoam` completed.
  - `fcv02_short_swept`: strict fail on skewness, `841776` cells,
    `67488` prisms, max skewness `4.308684`.
  - `fcv03_high_aspect_mild`: strict fail on skewness, `867638` cells,
    `57680` prisms, max skewness `4.2698814`.
  - `fcv04_compact_wide_tail`: strict fail on skewness, `1079117` cells,
    `85898` prisms, max skewness `4.8898757`.
  - `fcv05_aft_wing_fast`: strict pass, `984564` cells, `80598` prisms,
    max skewness `3.5009827`, `potentialFoam` completed.
- No-slip solver smoke:
  - Case:
    `custom_cfd_mesher_experiment/runs/gmsh_bl_classification_matrix_v0_1_20260623/angle80_create-topology/simplefoam_komega_20`.
  - Solver: `foamRun -solver incompressibleFluid` with `kOmegaSST`,
    `noSlip` aircraft wall, 50 mph freestream, and force coefficients enabled.
  - Result: failed; reached `Time = 11s`, then hit a floating point exception
    in the pressure solve.
  - Coefficients became numerically meaningless before failure, so this is not
    scoring evidence.
  - Interpretation: a valid one-layer prism mesh is not enough by itself for a
    stable scoring CFD setup; solver setup, wall treatment, y+ policy, and mesh
    non-orthogonality still need work.
- Failed/limited improvement attempts:
  - Gmsh optimizers (`default`, `Netgen`, `Relocate3D`) reduce some tetra cell
    counts and non-orthogonality but do not reduce the prism-driven max
    skewness on the failed variants.
  - Near-zero local thickness suppression creates degenerate prisms and is not
    viable.
  - Partial patch-layer suppression is not clean with the current Gmsh volume
    algorithms; mixed quad/triangle boundaries were rejected or produced PLC
    intersection errors.
  - A smoothed 3 mm BL-specific surface remesh for `fcv04` improved general
    triangle quality but made strict skewness worse and raised the mesh to
    about `1.49M` cells.
  - A 5-6 mm no-smooth BL-specific surface remesh reduced cell count and
    improved `fcv04` skewness, but still failed strict `checkMesh` with max
    skewness around `4.5`.
- Decision:
  - Gmsh prism layers are viable enough to keep researching.
  - Gmsh `create-topology` is the current best boundary-layer route.
  - Do not promote Gmsh prism layers to optimizer default yet.
  - The next blocker is aircraft surface triangle quality for prism extrusion,
    especially a few sliver/high-skew triangles near sharp aircraft features.
  - The next useful work is a BL-specific surface quality gate/remesher that
    targets prism suitability, not just surface fidelity.

## 2026-06-23 - BL Surface Gate and Five-Variant Passing Prism Set

- Added BL-specific surface diagnostics:
  - `custom_cfd_mesher_experiment/scripts/bl_surface_gate.py`
  - `custom_cfd_mesher_experiment/scripts/correlate_openfoam_skew_to_surface.py`
- The BL surface gate reports:
  - triangle quality
  - triangle aspect ratio
  - minimum triangle altitude
  - minimum/maximum triangle angle
  - bad-triangle clusters by aircraft-relative region
  - optional bad-triangle CSV output
- The skew-correlation tool maps OpenFOAM `skewFaces.vtk` problem-face
  centers back to nearest STL surface triangles.
- Findings:
  - Generic sliver counts alone do not predict pass/fail. `fcv05` has ugly
    local triangles but still passes.
  - `fcv04` failures correlate with poor local surface triangles near
    aft/tail/wingtip regions.
  - `fcv02` and `fcv03` failures were initially tiny-count transition/skew
    artifacts, not obvious global surface-quality failures.
- Bounded remesh/layer sweeps produced a strict-passing layered OpenFOAM mesh
  for all five variants.
- Final passing layered set summary saved at:
  `custom_cfd_mesher_experiment/runs/gmsh_bl_final_passing_set_v0_1_20260623.json`.
- Passing cases:
  - `fcv01_base_h50`:
    - Case:
      `custom_cfd_mesher_experiment/runs/gmsh_bl_classification_matrix_v0_1_20260623/angle80_create-topology`.
    - Strict `checkMesh`: pass.
    - Cells: `1107398`; prisms: `90434`; max skewness `3.8761905`;
      max non-orthogonality `88.516724`.
    - `potentialFoam`: completed; continuity error `4.9427109e-06`.
  - `fcv02_base_h150`:
    - Case:
      `custom_cfd_mesher_experiment/runs/gmsh_bl_final_knob_probe_v0_1_20260623/fcv02_base_h150`.
    - Strict `checkMesh`: pass.
    - Cells: `832580`; prisms: `67488`; max skewness `3.933887`;
      max non-orthogonality `89.488434`.
    - `potentialFoam`: completed; continuity error `0.00024326691`.
  - `fcv03_fd45_h50`:
    - Case:
      `custom_cfd_mesher_experiment/runs/gmsh_bl_feature_angle_remesh_sweep_v0_1_20260623/fcv03_high_aspect_mild_fd45/gmsh_bl`.
    - Strict `checkMesh`: pass.
    - Cells: `1042530`; prisms: `85906`; max skewness `3.3616408`;
      max non-orthogonality `88.648954`.
    - `potentialFoam`: completed; continuity error `6.7508836e-06`.
  - `fcv04_fd15_h100`:
    - Case:
      `custom_cfd_mesher_experiment/runs/gmsh_bl_final_knob_probe_v0_1_20260623/fcv04_fd15_h100`.
    - Strict `checkMesh`: pass.
    - Cells: `1127112`; prisms: `86624`; max skewness `3.0316763`;
      max non-orthogonality `86.873024`.
    - `potentialFoam`: completed; continuity error `1.8720106e-06`.
  - `fcv05_base_h50`:
    - Case:
      `custom_cfd_mesher_experiment/runs/gmsh_bl_five_variant_create_topology_v0_1_20260623/fcv05_aft_wing_fast`.
    - Strict `checkMesh`: pass.
    - Cells: `984564`; prisms: `80598`; max skewness `3.5009827`;
      max non-orthogonality `88.629854`.
    - `potentialFoam`: completed; continuity error `8.2257634e-06`.
- Decision:
  - Gmsh `create-topology` prism layers are now proven viable for OpenFOAM
    mesher/plumbing on the five faired-cap variants.
  - This is still not a single production preset. The passing set currently
    uses variant-specific remesh/layer settings.
  - Do not claim scoring CFD. The earlier no-slip `kOmegaSST` smoke still
    diverged and hit an FPE.
  - Next useful work is to convert these successful settings into a deterministic
    first-pass selection policy driven by BL surface-gate metrics, then rerun
    all five without post-failure retries.

## 2026-06-23 - Gmsh Boundary-Layer Auto-Selector v0.2

- Goal:
  - Convert the hand-selected five-variant Gmsh `create-topology` prism-layer
    passing set into a deterministic first-pass selector.
  - Avoid hotspot retries as the normal path.
  - Preserve strict OpenFOAM `checkMesh` and `potentialFoam -writep` as the
    current non-scoring plumbing gates.
- Fresh run root:
  `custom_cfd_mesher_experiment/runs/gmsh_bl_auto_selector_v0_2_20260623`.
- Platform preset/config:
  `software/optimizer/configs/gmsh_openfoam_external_aero_bl.v0_2.json`.
- Command:

```powershell
.\su2_sandbox\.venv\Scripts\python.exe custom_cfd_mesher_experiment\scripts\run_gmsh_bl_auto_selector.py --run-root custom_cfd_mesher_experiment\runs\gmsh_bl_auto_selector_v0_2_20260623
```

- Selector behavior:
  - Always starts with a 4 mm no-smooth remesh at `feature_deg=25`.
  - Runs the BL surface gate to classify bad-triangle clusters.
  - Selects feature angle and first-layer height from deterministic rules.
  - Runs Gmsh `create-topology`, one prism layer, OpenFOAM conversion,
    strict `checkMesh`, and `potentialFoam -writep`.
- Result:
  - Strict OpenFOAM mesh pass count: `5/5`.
  - `potentialFoam -writep` pass count: `5/5`.
  - Total runtime: `855.0 s`.
- Aircraft-only screenshots:
  - Generated after the selector run with `render_stl_view_set.py` from each
    selected remeshed aircraft surface.
  - Screenshot folders:
    - `custom_cfd_mesher_experiment/runs/gmsh_bl_auto_selector_v0_2_20260623/fcv01_long_glider/aircraft_only_screenshots`
    - `custom_cfd_mesher_experiment/runs/gmsh_bl_auto_selector_v0_2_20260623/fcv02_short_swept/aircraft_only_screenshots`
    - `custom_cfd_mesher_experiment/runs/gmsh_bl_auto_selector_v0_2_20260623/fcv03_high_aspect_mild/aircraft_only_screenshots`
    - `custom_cfd_mesher_experiment/runs/gmsh_bl_auto_selector_v0_2_20260623/fcv04_compact_wide_tail/aircraft_only_screenshots`
    - `custom_cfd_mesher_experiment/runs/gmsh_bl_auto_selector_v0_2_20260623/fcv05_aft_wing_fast/aircraft_only_screenshots`
- Variant metrics:

| Variant | Feature deg | Layer height | Cells | Prisms | Max skewness | Max non-orthogonality | potentialFoam |
|---|---:|---:|---:|---:|---:|---:|---|
| `fcv01_long_glider` | `25` | `50 um` | `1107798` | `90414` | `3.8761905` | `88.520055` | pass |
| `fcv02_short_swept` | `25` | `150 um` | `846362` | `67456` | `3.933887` | `88.958331` | pass |
| `fcv03_high_aspect_mild` | `45` | `50 um` | `1042530` | `85906` | `3.3616408` | `88.648954` | pass |
| `fcv04_compact_wide_tail` | `15` | `100 um` | `1127112` | `86624` | `3.0316763` | `86.873024` | pass |
| `fcv05_aft_wing_fast` | `25` | `50 um` | `984724` | `80538` | `3.5009827` | `88.971341` | pass |

- Decision:
  - Adopt `gmsh_openfoam_external_aero_bl_v0_2` as the current default
    non-scoring OpenFOAM mesher-plumbing preset.
  - This is the first boundary-layer/prism-layer path that passes all five
    approved faired-cap variants without per-case manual retries.
  - Keep `scoring_allowed=false`. The no-slip OpenFOAM solver path still needs
    stabilization, y-plus policy, force/moment coefficient checks, and
    convergence gates before lift/drag can be treated as trusted.
- Caveat:
  - The selector is evidence-backed on the five approved variants, not a
    general aircraft-family model.
  - High non-orthogonality remains near sharp feature regions, especially
    trailing-edge/transition areas. It is acceptable for plumbing evidence
    because strict `checkMesh` passes, but it remains a scoring-CFD risk.

## 2026-06-23 - Optimizer OpenFOAM Policy Update for Gmsh BL

- Goal:
  - Let the optimizer persist the new Gmsh boundary-layer OpenFOAM smoke runs
    without misclassifying them under older snappy/tetra-oriented quality
    ceilings.
- Updated:
  - `software/optimizer/src/aircraft_optimizer/modules/openfoam_result_validation.py`
    now defines `GMSH_BL_DEVELOPMENT_OPENFOAM_RESULT_POLICY`.
  - `software/optimizer/src/aircraft_optimizer/modules/openfoam_smoke_adapter.py`
    accepts the new `gmsh_bl_development` mode.
  - `software/optimizer/src/aircraft_optimizer/cli/main.py` exposes
    `--acceptance-mode gmsh_bl_development` for `persist-openfoam-smoke`.
  - `software/optimizer/tests/test_v0_1_flow.py` includes a regression test
    proving default strict rejects high non-orthogonality while
    `gmsh_bl_development` accepts the Gmsh BL plumbing case only when OpenFOAM
    reports `Mesh OK` and `potentialFoam` completes.
- Policy:
  - `acceptance_mode`: `gmsh_bl_development`.
  - `require_mesh_ok`: `true`.
  - `max_non_orthogonality`: `90.0`.
  - `max_skewness`: `4.0`.
  - `scoring_allowed`: remains `false` through the smoke adapter.
- Validation:

```powershell
python -m pytest software\optimizer\tests\test_v0_1_flow.py -k "openfoam_result_validation"
python -m pytest software\optimizer\tests\test_v0_1_flow.py
$env:PYTHONPATH='software\optimizer\src'; python -m aircraft_optimizer.cli.main check-openfoam-result --case-dir ..\custom_cfd_mesher_experiment\runs\gmsh_bl_auto_selector_v0_2_20260623\fcv01_long_glider\gmsh_bl\openfoam_case --check-mesh-log ..\custom_cfd_mesher_experiment\runs\gmsh_bl_auto_selector_v0_2_20260623\fcv01_long_glider\gmsh_bl\openfoam_case\log.checkMesh --solver-log ..\custom_cfd_mesher_experiment\runs\gmsh_bl_auto_selector_v0_2_20260623\fcv01_long_glider\gmsh_bl\openfoam_case\log.potentialFoam --acceptance-mode gmsh_bl_development
```

- Result:
  - Targeted OpenFOAM validation tests: `4 passed`.
  - Full optimizer flow test file: `50 passed`.
  - Live `fcv01_long_glider` Gmsh BL case validation:
    `passed=true`, `openfoam.ready=1`, `cells=1107798`,
    `aircraft_faces=90414`, `max_non_orthogonality=88.520055`,
    `max_skewness=3.8761905`, `continuity_error=5.6656479e-06`.

## 2026-06-24 - Gmsh BL Evidence Import and No-Slip Smoke

- Goal:
  - Persist all five selected Gmsh BL/OpenFOAM plumbing cases as optimizer
    records.
  - Run one bounded no-slip OpenFOAM experiment on the current prism mesh.
- Added CLI:
  - `ingest-gmsh-bl-run`.
  - The command creates a dedicated campaign, registers one candidate per
    faired-cap variant, starts an evaluation for each, and persists each
    existing OpenFOAM smoke result with `acceptance_mode=gmsh_bl_development`.
- Import command:

```powershell
$env:PYTHONPATH='software\optimizer\src'; python -m aircraft_optimizer.cli.main ingest-gmsh-bl-run --workspace runs\gmsh_bl_openfoam_persistence_v0_1 --summary ..\custom_cfd_mesher_experiment\runs\gmsh_bl_auto_selector_v0_2_20260623\summary.json
```

- Import result:
  - Workspace: `aircraft_optimizer_platform/runs/gmsh_bl_openfoam_persistence_v0_1`.
  - Campaign: `campaign_cf89c5a0e70e4061b6f9bd587f4f669a`.
  - Imported variants: `5`.
  - Candidates: `5`.
  - Complete evaluations: `5`.
  - Successful `openfoam_smoke_validation` module attempts: `5`.
  - Artifacts: `30`.
  - Failures: `0`.
  - Aircraft-only ISO screenshots registered: `5`.
- No-slip experiment basis:
  - Variant: `fcv04_compact_wide_tail`.
  - Reason: cleanest selected Gmsh BL mesh metrics among the five-variant set.
  - Source case:
    `custom_cfd_mesher_experiment/runs/gmsh_bl_auto_selector_v0_2_20260623/fcv04_compact_wide_tail/gmsh_bl/openfoam_case`.
  - Reference values used:
    - `U = 22.352 m/s`.
    - `Aref = 0.1007 m^2`.
    - `lRef = 0.1325 m`.
    - `liftDir = +Z`, `dragDir = +X`, `pitchAxis = +Y`.
- First no-slip attempt:
  - Case:
    `custom_cfd_mesher_experiment/runs/gmsh_bl_no_slip_smoke_v0_1_20260624/fcv04_laminar_30`.
  - Solver: `foamRun -solver incompressibleFluid`.
  - Model: laminar, aircraft `noSlip`, forceCoeffs enabled.
  - Result: stopped after timeout; no FPE, but numerically useless.
  - Last recorded time: `30`.
  - Continuity exploded to approximately `1.1e11` local sum at the last visible
    step.
  - Force coefficients exploded to nonphysical magnitudes around `1e24`.
- Conservative no-slip rerun:
  - Case:
    `custom_cfd_mesher_experiment/runs/gmsh_bl_no_slip_smoke_v0_1_20260624/fcv04_laminar_relaxed_10`.
  - Changes:
    - `endTime = 10`.
    - `pRelax = 0.1`.
    - `uRelax = 0.2`.
    - `nOuterCorrectors = 2`.
    - `nCorrectors = 2`.
    - `nNonOrthogonalCorrectors = 5`.
  - Result:
    - Completed without FPE.
    - Last pressure final residual: `8.3387522e-05`.
    - Last velocity final residual: `2.2326019e-04`.
    - Last continuity local sum: `5.4818201e-04`.
    - Last force coefficients: `Cd = 1.55848177`,
      `Cl = 0.125735569`, `Cm = -0.23558824`.
  - Verdict:
    - This is useful no-slip solver-plumbing evidence.
    - It is not scoring CFD. Coefficients are not converged or stable enough
      over 10 iterations, and the run is laminar rather than a validated
      wall-treatment/RANS setup.
- Decision:
  - Gmsh BL meshing is no longer the immediate blocker for no-slip plumbing.
  - The next blocker is solver setup/stability and coefficient convergence
    policy.
  - Keep `scoring_allowed=false`.

## 2026-06-24 - OpenFOAM Steady Parser and Coefficient-Stability Sweep

- Goal:
  - Add platform persistence for non-potential OpenFOAM steady/no-slip runs.
  - Run a small coefficient-stability sweep on the current best Gmsh BL prism
    mesh.
  - Verify the solver choice against the installed OpenFOAM 13 environment.
- Solver check:
  - Installed OpenFOAM version reports `OpenFOAM-13`.
  - `foamRun -solver incompressibleFluid` is available through the modular
    `libincompressibleFluid.so` path.
  - Standalone `icoFoam` exists, but it is a transient laminar solver and is
    not the right default for steady external-aircraft coefficient plumbing.
  - No installed standalone `isoFoam` path was found; the likely relevant
    comparison is `icoFoam`, not `isoFoam`.
  - Current decision: keep `foamRun -solver incompressibleFluid` for bounded
    steady incompressible development. Use laminar first for solver plumbing;
    retry `kOmegaSST` only after steady controls and coefficient gates are
    working.
- Added platform modules:
  - `software/optimizer/src/aircraft_optimizer/modules/openfoam_steady_result_validation.py`.
  - `software/optimizer/src/aircraft_optimizer/modules/openfoam_steady_adapter.py`.
  - CLI commands:
    - `check-openfoam-steady-result`.
    - `persist-openfoam-steady`.
- Parser/persistence behavior:
  - Parses `foamRun -solver incompressibleFluid` logs.
  - Records completion, fatal/FPE status, pressure and velocity residuals,
    continuity errors, execution time, forceCoeffs, and coefficient-window
    stability.
  - Persists artifacts as `openfoam_steady_validation`.
  - Always records `scoring_allowed=false`.
- Coefficient-stability sweep basis:
  - Variant: `fcv04_compact_wide_tail`.
  - Mesh source:
    `custom_cfd_mesher_experiment/runs/gmsh_bl_auto_selector_v0_2_20260623/fcv04_compact_wide_tail/gmsh_bl/openfoam_case`.
  - Solver: `foamRun -solver incompressibleFluid`.
  - Model: laminar, aircraft `noSlip`, `forceCoeffs` enabled.
  - Reference values:
    - `U = 22.352 m/s`.
    - `Aref = 0.1007 m^2`.
    - `lRef = 0.1325 m`.
    - drag `+X`, lift `+Z`, pitch `+Y`.
- Sweep results:

| Case | pRelax | uRelax | End | Completed | FPE | Cd last | Cl last | Cd span last 3 | Cl span last 3 | Continuity local | Verdict |
|---|---:|---:|---:|---|---|---:|---:|---:|---:|---:|---|
| `fcv04_laminar_relaxed_10` | `0.1` | `0.2` | `10` | yes | no | `1.55848177` | `0.125735569` | `2.233738276` | `0.283322471` | `5.4818201e-04` | unstable coefficients |
| `fcv04_laminar_p005_u01_10` | `0.05` | `0.1` | `10` | yes | no | `-5.20296532` | `-0.58194548` | `0.92725262` | `0.256950245` | `6.5222911e-04` | unstable coefficients |
| `fcv04_laminar_p02_u03_10` | `0.2` | `0.3` | `10` | yes | no | `-0.166594283` | `0.0427411837` | `0.526924093` | `0.0855832343` | `1.185484e-04` | improving |
| `fcv04_laminar_p02_u03_20` | `0.2` | `0.3` | `20` | yes | no | `0.107564632` | `0.0733830739` | `0.009578139` | `0.002713398` | `2.88911e-05` | best non-scoring plumbing result |

- Best run:
  `custom_cfd_mesher_experiment/runs/gmsh_bl_no_slip_smoke_v0_1_20260624/fcv04_laminar_p02_u03_20`.
- Persistence:
  - Persisted against imported `fcv04` evaluation
    `evaluation_f832b0049cb947b9a54652c58c9d70d2`.
  - Module attempt:
    `module_9087883bcb334000a25b72f2e708e2df`.
  - Workspace summary after persistence:
    - module attempts: `6`.
    - `openfoam_smoke_validation`: `5` success.
    - `openfoam_steady_validation`: `1` success.
    - artifacts: `36`.
    - failures: `0`.
- Validation:

```powershell
python -m pytest software\optimizer\tests\test_v0_1_flow.py
```

- Result:
  - `52 passed`.
- Decision:
  - We are using the right OpenFOAM solver path for this stage:
    `foamRun -solver incompressibleFluid`.
  - The immediate next step is not changing to `icoFoam`; it is extending the
    stable laminar setup and then reintroducing `kOmegaSST` with conservative
    controls and explicit y-plus/wall-treatment reporting.
  - `fcv04_laminar_p02_u03_20` is still not scoring CFD. It is the first
    coefficient-stable no-slip plumbing result under the current simple window
    metric.

## Open Questions

- What units and coordinate frame should be canonical across optimizer records?
- Should v0.1 use JSON Schema, Pydantic models, or both for configuration validation?
- Should the first dashboard stack be React or Svelte?
- Should SQLite remain one database per campaign, or should early implementation use one workspace-level database?
- Should geometry-provider bbox metadata be required for all generated feature variants, or only when a feature extends outside the baseline aircraft envelope?
- Which existing manual SDF/Rhai files should become canonical platform examples versus archived diagnostics?
- Should the master folder eventually vendor the full implicit CAD/SDF Rust library, or keep it as a sibling dependency?
- Should the exporter implementation be copied into `software/exporters/oml_stl` or wrapped in place through a stable adapter first?

## Blockers

No active blockers.

## 2026-06-24 - Hybrid Prism/Gmsh OpenFOAM Backend Promotion

- Added optimizer preset:
  `software/optimizer/configs/hybrid_prism_gmsh_openfoam.v0_1.json`.
- Preset status:
  - `default_openfoam_plumbing_backend`.
  - `scoring_allowed=false`.
  - backend: direct body-fitted prism shell plus Gmsh tetrahedral farfield
    volume plus exact OpenFOAM polyMesh interface merge.
- Updated mesh handoff contract:
  `software/optimizer/contracts/mesh_export_contract.md`.
  The hybrid preset is now the preferred OpenFOAM plumbing backend. The older
  `gmsh_openfoam_external_aero_bl.v0_2.json` preset remains comparison
  evidence.
- Hybrid evidence root:
  `custom_cfd_mesher_experiment/runs/hybrid_shell_gmsh_exact_merge_five_variant_netgen_v0_1_20260624`.
- Hybrid evidence result:
  - five conditioned faired-cap variants tested.
  - strict OpenFOAM `checkMesh`: `5/5` pass.
  - `potentialFoam -writep`: `5/5` complete.
  - cell range: `752,726` to `992,789`.
  - severe non-orthogonal face range: `178` to `470`.
  - runtime range: about `103` to `141` seconds per variant.
- Important mesher decision:
  - Use `custom_cfd_mesher_experiment/scripts/merge_prism_shell_gmsh_outer.py`.
  - Do not use OpenFOAM `mergeMeshes` plus `stitchMesh` for this interface; it
    produced invalid combined meshes.
  - Use Gmsh `default,Netgen` optimization for the outer volume. A no-optimizer
    outer-volume run produced an isolated high-aspect-cell failure on fcv04.

## 2026-06-24 - Hybrid Mesh No-Slip Steady Probe

- Added run report:
  `custom_cfd_mesher_experiment/runs/hybrid_shell_gmsh_simplefoam_forcecoeffs_probe_v0_1_20260624/NO_SLIP_STEADY_PROBE_REPORT.md`.
- Probe variant:
  `fcv04_compact_wide_tail`.
- Source mesh:
  `custom_cfd_mesher_experiment/runs/hybrid_shell_gmsh_exact_merge_five_variant_netgen_v0_1_20260624/fcv04_compact_wide_tail/merged_openfoam_case`.
- No-slip steady setup:
  - solver: `foamRun -solver incompressibleFluid`.
  - model: laminar.
  - aircraft wall: `noSlip`.
  - forceCoeffs enabled.
  - drag `+X`, lift `+Z`, pitch `+Y`.
- Results:
  - `fcv04_laminar_20`: failed at time step `2` with pressure-solver FPE.
  - `fcv04_laminar_p01_u025_potential_10`: `potentialFoam` pressure solve
    diverged during attempted no-slip initialization.
  - `fcv04_laminar_fixed_farfield_p01_u025_5`: failed at time step `2` with
    pressure-solver FPE.
- Structured validation:
  `fcv04_laminar_fixed_farfield_p01_u025_5/openfoam_steady_validation.json`.
- Failed validation checks:
  - `solver_completed`.
  - `fatal_error`.
  - `local_continuity`.
  - `velocity_final_residual`.
- Interpretation:
  - The hybrid mesh is valid for OpenFOAM mesher plumbing.
  - The no-slip steady solver path is not stable on this hybrid mesh yet.
  - No `Cd`, `Cl`, or `Cm` from these hybrid no-slip probes may be used for
    optimizer scoring.
  - Current likely blockers are no-slip wall treatment/y-plus policy, prism/tet
    transition behavior under pressure correction, single farfield boundary
    policy, and pressure solver/scheme sensitivity.

## 2026-06-24 - Hybrid Split-Farfield and Transient Diagnosis

- Added run report:
  `custom_cfd_mesher_experiment/runs/hybrid_shell_gmsh_split_farfield_probe_v0_1_20260624/SPLIT_FARFIELD_TRANSIENT_PROBE_REPORT.md`.
- Added split-farfield support to the hybrid exact-merge path:
  - `custom_cfd_mesher_experiment/scripts/run_hybrid_shell_gmsh_exact_merge_smoke.py`
    now accepts `--farfield-patches split`.
  - `custom_cfd_mesher_experiment/scripts/merge_prism_shell_gmsh_outer.py`
    now accepts `--outer-boundary-patches auto` and preserves non-interface
    outer boundary patches through the exact merge.
- Split-farfield mesh result on `fcv04_compact_wide_tail`:
  - strict `checkMesh`: pass.
  - `potentialFoam -writep`: complete.
  - cells: `992,789`.
  - points: `278,107`.
  - max skewness: `3.5425877`.
  - max non-orthogonality: `84.458095`.
  - severe non-orthogonal faces: `470`.
- Split-farfield steady no-slip result:
  - Case: `fcv04_split_laminar_p02_u03_10`.
  - Solver: `foamRun -solver incompressibleFluid`.
  - Result: failed at time step `2` with a pressure-solver floating point
    exception.
  - Interpretation: split farfield boundary patching alone is not the cause of
    the steady no-slip failure.
- Transient diagnostic:
  - Case `fcv04_split_icoFoam_dt1e5_5steps` failed immediately with max
    Courant number `96.693276`.
  - Case `fcv04_split_icoFoam_dt1e7_5steps` completed five steps with small
    continuity errors when `deltaT = 1e-7 s`.
  - Case `fcv04_split_icoFoam_dt1e7_10steps` completed ten steps, no FPE,
    final max Courant about `0.82582329`, final local continuity about
    `9.10e-15`, and clock time about `120 s`.
- Diagnosis:
  - The hybrid mesh is not inherently impossible for no-slip flow.
  - The current blocker is the interaction between tiny near-wall/prism cells,
    no-slip startup, pressure correction, time scale/Courant number, and
    wall-treatment policy.
  - The stable transient time step is currently too small for default optimizer
    scoring throughput unless first-layer sizing, ramping, or solver strategy
    improves.
  - SU2 may be useful later as an alternate solver backend, but it would still
    need the same quality mesh, marker, wall-treatment, y-plus, and convergence
    controls. It is not expected to make a bad near-wall/time-scale setup good
    by itself.
- Validation:
  - `py_compile` passed for the two updated hybrid scripts.
  - `python -m pytest software\optimizer\tests\test_v0_1_flow.py`: `52 passed`.

## 2026-06-24 - Parallel SnappyHexMesh Benchmark

- Added benchmark report:
  `custom_cfd_mesher_experiment/runs/snappy_parallel_benchmark_v0_1_20260624/SNAPPY_PARALLEL_BENCHMARK.md`.
- Goal:
  - Test whether a longer all-snappy OpenFOAM meshing path becomes viable when
    run in parallel on 8 or 12 cores.
  - Basis: `fcv04_compact_wide_tail`, using the existing prepared faired-cap
    aircraft surface.
- User decision:
  - A development skewness ceiling of `12.0` is acceptable for this benchmark.
  - Preserve strict `checkMesh` failures, but allow `checkMesh -skewThreshold 12`
    as relaxed/development plumbing evidence.
- Results:
  - `fcv04_compact_wide_tail_np8`, levels `2/3`:
    - snappy time `48.746942 s`.
    - full mesh sequence about `96.8 s`.
    - cells `142,246`, points `166,773`, aircraft faces `9,524`.
    - strict `checkMesh`: fail on skewness.
    - max skewness `7.1323516`, max non-orthogonality `69.999283`.
  - `fcv04_compact_wide_tail_np12`, levels `2/3`:
    - snappy time `48.032817 s`.
    - full mesh sequence about `121.2 s`.
    - cells `142,465`, points `167,041`, aircraft faces `9,544`.
    - strict `checkMesh`: fail on skewness.
    - `checkMesh -skewThreshold 12`: pass.
    - `potentialFoam -writep`: complete.
  - `fcv04_compact_wide_tail_np12_l3_l4`, levels `3/4`:
    - snappy time `81.102856 s`.
    - full mesh/check sequence about `162.7 s`.
    - cells `243,636`, points `290,177`, aircraft faces `23,962`.
    - strict `checkMesh`: fail on skewness.
    - `checkMesh -skewThreshold 12`: pass.
    - max skewness `8.9609095`, max non-orthogonality `71.555609`.
    - severe non-orthogonal faces over `70 deg`: `7`.
    - average aircraft layers `1.53`, overall thickness `0.000709 m`.
    - `potentialFoam -writep`: complete; continuity error `3.3746103e-07`.
- Visual artifacts:
  - `custom_cfd_mesher_experiment/runs/snappy_parallel_benchmark_v0_1_20260624/fcv04_compact_wide_tail_np12/aircraft_iso.png`.
  - `custom_cfd_mesher_experiment/runs/snappy_parallel_benchmark_v0_1_20260624/fcv04_compact_wide_tail_np12_l3_l4/aircraft_iso.png`.
- Decision:
  - Parallel snappy is not a 30-minute blocker at these bounded settings.
  - The higher-refinement `l3_l4` case is plausible for continued
    high-fidelity/scoring-mesher research.
  - Do not replace the hybrid prism/Gmsh default yet. Snappy still needs relaxed
    skewness acceptance, has limited layer coverage, and must prove no-slip
    solver stability before promotion.

## 2026-06-24 - Snappy No-Slip Probe on `np12_l3_l4`

- Basis:
  `custom_cfd_mesher_experiment/runs/snappy_parallel_benchmark_v0_1_20260624/fcv04_compact_wide_tail_np12_l3_l4`.
- Steady no-slip case:
  `custom_cfd_mesher_experiment/runs/snappy_parallel_benchmark_v0_1_20260624/fcv04_compact_wide_tail_np12_l3_l4_steady_p02_u03_20`.
- Setup:
  - solver: `foamRun -solver incompressibleFluid`.
  - model: laminar.
  - aircraft wall: `noSlip`.
  - farfield: `freestreamVelocity` / `freestreamPressure`.
  - pRelax `0.2`, uRelax `0.3`.
  - `nCorrectors = 2`.
  - `nNonOrthogonalCorrectors = 5`.
  - forceCoeffs enabled with drag `+X`, lift `+Z`, pitch `+Y`.
- Steady result:
  - completed `20` iterations.
  - no floating point exception.
  - final local continuity: `4.086786e-05`.
  - final cumulative continuity: `5.8111579e-05`.
  - final coefficients:
    - `Cd = 0.0131006477`.
    - `Cl = 0.119423802`.
    - `Cm = 0.0593625912`.
  - last-five coefficient spans:
    - `Cd = 0.1981965219`.
    - `Cl = 0.012090851`.
    - `Cm = 0.3061817792`.
  - verdict: solver-stability improvement over the hybrid exact-merge no-slip
    probe, but still non-scoring because force coefficients are not stable.
- Tiny-step transient no-slip case:
  `custom_cfd_mesher_experiment/runs/snappy_parallel_benchmark_v0_1_20260624/fcv04_compact_wide_tail_np12_l3_l4_icoFoam_dt1e7_10steps`.
- Transient setup:
  - solver: `icoFoam`.
  - model: laminar transient.
  - `deltaT = 1e-7 s`.
  - end time `1e-6 s`.
- Transient result:
  - completed ten steps.
  - no floating point exception.
  - final max Courant number: `0.0088329801`.
  - final local continuity: `4.7987122e-15`.
  - final cumulative continuity: `-3.2563446e-11`.
  - wall time: about `20 s`.
- Decision:
  - Snappy now has better no-slip stability evidence than the hybrid
    prism/Gmsh exact-merge mesh on the same tough `fcv04` basis.
  - Keep hybrid prism/Gmsh as the fast OpenFOAM plumbing backend for now.
  - Promote parallel snappy `np12_l3_l4` to the next scoring-CFD development
    candidate.
  - Next useful work is a longer steady no-slip window, conservative
    `kOmegaSST`/wall-function trial, y-plus reporting, and strict coefficient
    stability gates.

## 2026-06-24 - Snappy Laminar-Start RANS and Thick-Layer Mesh Probe

- Basis:
  `custom_cfd_mesher_experiment/runs/snappy_parallel_benchmark_v0_1_20260624`.
- Baseline `np12_l3_l4` longer laminar no-slip:
  - case:
    `custom_cfd_mesher_experiment/runs/snappy_parallel_benchmark_v0_1_20260624/fcv04_compact_wide_tail_np12_l3_l4_steady_laminar_p02_u03_100`.
  - completed 100 iterations with no floating point exception.
  - final coefficients: `Cd = 0.128830847`, `Cl = 0.13786821`,
    `Cm = -0.0277343679`.
  - coefficient tail was stable by the current development gate.
- Uniform-start kOmegaSST on the same mesh:
  - case:
    `custom_cfd_mesher_experiment/runs/snappy_parallel_benchmark_v0_1_20260624/fcv04_compact_wide_tail_np12_l3_l4_kOmegaSST_p01_u02_100_yplus`.
  - completed without fatal crash but is invalid: final `Cd = 62.6980396`,
    final `Cl = -9.89020499`, final `Cm = 28.3081085`.
  - y-plus max reached `8736.6619`; `88` aircraft faces were above `300`.
  - hotspot evidence:
    `diagnostics/yplus_aircraft_top_hotspots.csv`,
    `diagnostics/yplus_gt300_hotspots.vtk`, and
    `diagnostics/yplus_gt300_hotspots_iso.png`.
- Laminar-start kOmegaSST on baseline `np12_l3_l4`:
  - case:
    `custom_cfd_mesher_experiment/runs/snappy_parallel_benchmark_v0_1_20260624/fcv04_compact_wide_tail_np12_l3_l4_kOmegaSST_laminarStart_k002_om50_p01_u02_100`.
  - initialized `U` and `p` from the stable laminar run.
  - used `k = 0.02`, `omega = 50`, pRelax `0.1`, uRelax `0.2`,
    and `k/omega` relaxation `0.2`.
  - completed 100 iterations with no floating point exception.
  - final coefficients: `Cd = 0.133347982`, `Cl = 0.148654779`,
    `Cm = -0.0361726807`.
  - y-plus distribution at time `100`: p50 `11.5870`, p95 `33.8173`,
    max `133.42108`.
- Thick-layer snappy variant:
  - case:
    `custom_cfd_mesher_experiment/runs/snappy_parallel_benchmark_v0_1_20260624/fcv04_compact_wide_tail_np12_l3_l4_layers3_thick`.
  - changes: requested `3` layers, final layer thickness `0.35`,
    min layer thickness `0.05`, layer iterations `50`, relaxed iterations `20`.
  - snappy time `82.909495 s`.
  - cells `261,192`, points `311,258`, aircraft faces `23,962`.
  - average aircraft layers improved from `1.53` to `2.26`.
  - overall layer thickness improved from `0.000709 m` to `0.00183 m`.
  - max skewness `8.9609095`, max non-orthogonality `69.999388`.
  - `checkMesh -skewThreshold 12`: pass.
  - `potentialFoam -writep`: complete; continuity error `2.9786134e-07`.
  - aircraft-only screenshot:
    `custom_cfd_mesher_experiment/runs/snappy_parallel_benchmark_v0_1_20260624/fcv04_compact_wide_tail_np12_l3_l4_layers3_thick/aircraft_iso.png`.
- Thick-layer laminar-start kOmegaSST:
  - case:
    `custom_cfd_mesher_experiment/runs/snappy_parallel_benchmark_v0_1_20260624/fcv04_compact_wide_tail_np12_l3_l4_layers3_thick_kOmegaSST_laminarStart_k002_om50_p01_u02_100`.
  - completed 100 iterations with no floating point exception.
  - final coefficients: `Cd = 0.13286367`, `Cl = 0.161833846`,
    `Cm = -0.0381510329`.
  - y-plus distribution at time `100`: p50 `11.3280`, p95 `42.6546`,
    max `148.22055`, `407` faces below `1`, `2844` faces above `30`.
- Decision:
  - Snappy no-slip solver behavior is now substantially better than the
    hybrid exact-merge path on `fcv04`.
  - Use laminar initialization before turbulent steady tests; uniform-start
    kOmegaSST is rejected.
  - The thick-layer snappy variant is the current scoring-CFD development
    candidate, but not validated scoring CFD.
  - Gmsh/hybrid remains the fast optimizer plumbing backend until snappy proves
    repeatability across all approved variants.
- Open risk:
  - y-plus is still mixed. The thick-layer mesh improves layer completeness but
    does not create a clean wall-function band or a validated low-y-plus mesh.
  - Need repeatability on the other four faired-cap variants before changing
    optimizer defaults.

## 2026-06-24 - Gmsh Laminar-Start RANS Transfer Check

- New probe root:
  `custom_cfd_mesher_experiment/runs/gmsh_hybrid_laminar_start_rans_probe_v0_1_20260624`.
- Purpose:
  - Check whether the snappy finding, "initialize `kOmegaSST` from a stable
    laminar no-slip state instead of uniform freestream", also changes the
    Gmsh result.
- Hybrid exact-merge Gmsh source:
  `custom_cfd_mesher_experiment/runs/hybrid_shell_gmsh_exact_merge_five_variant_netgen_v0_1_20260624/fcv04_compact_wide_tail/merged_openfoam_case`.
- Hybrid exact-merge mesh gate:
  - cells `992,789`, points `278,107`, prisms `259,872`,
    tetrahedra `732,917`, aircraft faces `86,624`.
  - max non-orthogonality `84.458095`.
  - severe non-orthogonal faces over 70 degrees: `470`.
  - max skewness `3.5425877`.
  - `checkMesh`: `Mesh OK`.
- Hybrid exact-merge fresh laminar case:
  `custom_cfd_mesher_experiment/runs/gmsh_hybrid_laminar_start_rans_probe_v0_1_20260624/fcv04_hybrid_laminar_p02_u03_100`.
- Hybrid exact-merge laminar result:
  - target was 100 iterations with `foamRun -solver incompressibleFluid`,
    laminar, pRelax `0.2`, uRelax `0.3`.
  - run hit the `300 s` cap during the first time step.
  - `Ux` final residual at time `1s` was `1714.4319`.
  - pressure solver was still doing 1000-iteration correction passes.
  - no valid laminar state was produced, so laminar-start `kOmegaSST` is not
    possible on this mesh yet.
- Older Gmsh BL source:
  `custom_cfd_mesher_experiment/runs/gmsh_bl_auto_selector_v0_2_20260623/fcv04_compact_wide_tail/gmsh_bl/openfoam_case`.
- Older Gmsh BL laminar initializer:
  `custom_cfd_mesher_experiment/runs/gmsh_bl_no_slip_smoke_v0_1_20260624/fcv04_laminar_p02_u03_20`.
- Older Gmsh BL laminar-start kOmegaSST case:
  `custom_cfd_mesher_experiment/runs/gmsh_hybrid_laminar_start_rans_probe_v0_1_20260624/fcv04_gmsh_bl_kOmegaSST_laminarStart_k002_om50_p01_u02_20`.
- Older Gmsh BL RANS result:
  - completed 20 iterations.
  - no fatal error and no floating point exception.
  - execution time `184.56776 s`.
  - final pressure residual `1.7305073e-05`.
  - final velocity residual `8.737535e-06`.
  - final coefficients: `Cd = 0.111670777`, `Cl = 0.0777210189`,
    `Cm = -0.0084807387`.
  - y-plus at time `20`: p50 `0.038610623`, p95 `0.1397087875`,
    max `4.840371`, average `0.0533875038`.
- Decision:
  - The laminar-start lesson does transfer to Gmsh when a stable laminar
    no-slip state exists.
  - It does not rescue the current hybrid exact-merge mesh, because that mesh
    still fails before laminar initialization can be produced.
  - Older Gmsh BL is low-y-plus/resolved-wall style and slow. It remains
    development evidence only.
  - Snappy thick-layer remains the stronger near-term scoring-CFD development
    candidate on `fcv04`.

## 2026-06-24 - Snappy Thick-Layer Repeatability on Remaining Four Variants

- New run root:
  `custom_cfd_mesher_experiment/runs/snappy_thick_layer_four_variant_v0_1_20260624`.
- New report:
  `custom_cfd_mesher_experiment/runs/snappy_thick_layer_four_variant_v0_1_20260624/SNAPPY_THICK_LAYER_FOUR_VARIANT_REPORT.md`.
- Harness updates:
  - `custom_cfd_mesher_experiment/scripts/run_snappy_layer_comparison.py`
    now supports `--variant-ids`, `--parallel-procs`, and
    `--check-skew-threshold`.
  - The snappy harness now uses OpenFOAM 13-compatible parallel reconstruction:
    `reconstructPar -constant -noFields`.
  - The VTK export now uses `foamToVTK -constant -noInternal -excludePatches
    '(farfield)'` instead of the removed `-patches` option.
  - New no-slip ladder runner:
    `custom_cfd_mesher_experiment/scripts/run_no_slip_laminar_start_rans.py`.
- Preset tested:
  - 12-core snappy.
  - surface levels `3/4`.
  - feature level `3`.
  - requested layers `3`.
  - final layer thickness `0.35`.
  - min layer thickness `0.05`.
  - development mesh gate: `checkMesh -skewThreshold 12`.
  - no-slip ladder: 100-step laminar, then 100-step laminar-start
    `kOmegaSST`.
- Four-variant mesh/potential results:
  - all four remaining approved variants passed relaxed mesh gate.
  - all four completed `potentialFoam -writep`.
  - total mesh/potential runtime: about `640 s`.
- Four-variant no-slip ladder results:
  - all four completed 100 laminar no-slip iterations.
  - all four completed 100 laminar-start `kOmegaSST` iterations.
  - no fatal errors and no floating point exceptions.
  - total no-slip ladder runtime: about `951 s`.
- Summary table:
  - `fcv01_long_glider`: `239,227` cells, max skew `4.352`,
    max non-orthogonality `69.999937`, RANS `Cd=0.134480`,
    `Cl=0.052421`, `Cm=0.040966`, y+ p50/p95/max
    `12.18/43.32/140.89`, coefficient gate pass.
  - `fcv02_short_swept`: `210,115` cells, max skew `5.111`,
    max non-orthogonality `69.997809`, RANS `Cd=0.110174`,
    `Cl=0.061822`, `Cm=-0.001629`, y+ p50/p95/max
    `10.46/43.61/116.82`, coefficient gate failed on `Cm` drift only.
  - `fcv03_high_aspect_mild`: `231,728` cells, max skew `5.189`,
    max non-orthogonality `69.999520`, RANS `Cd=0.132522`,
    `Cl=0.089450`, `Cm=0.025425`, y+ p50/p95/max
    `11.21/43.53/138.58`, coefficient gate pass.
  - `fcv05_aft_wing_fast`: `231,389` cells, max skew `7.086`,
    max non-orthogonality `69.996338`, RANS `Cd=0.118197`,
    `Cl=0.032539`, `Cm=0.002554`, y+ p50/p95/max
    `11.47/44.27/112.90`, coefficient gate failed on `Cm` drift only.
- Screenshots:
  - `custom_cfd_mesher_experiment/runs/snappy_thick_layer_four_variant_v0_1_20260624/fcv01_long_glider/aircraft_iso.png`.
  - `custom_cfd_mesher_experiment/runs/snappy_thick_layer_four_variant_v0_1_20260624/fcv02_short_swept/aircraft_iso.png`.
  - `custom_cfd_mesher_experiment/runs/snappy_thick_layer_four_variant_v0_1_20260624/fcv03_high_aspect_mild/aircraft_iso.png`.
  - `custom_cfd_mesher_experiment/runs/snappy_thick_layer_four_variant_v0_1_20260624/fcv05_aft_wing_fast/aircraft_iso.png`.
- Decision:
  - Snappy thick-layer now has repeatable mesh, potential, laminar no-slip, and
    laminar-start RANS evidence across all five approved faired-cap variants
    when combined with the previous `fcv04` run.
  - Promote snappy thick-layer to the leading scoring-CFD development backend.
  - Do not call it validated scoring CFD yet because strict skewness still
    fails on a few faces, y-plus remains mixed, force/moment references are
    provisional, and no mesh-convergence or validation study exists.
- Next:
  - Create a formal `snappy_openfoam_external_aero_v0_2` development preset and
    wire it as a candidate backend.
  - Update coefficient stability gates to combine absolute and relative
    thresholds, especially for near-zero `Cm`.

## Next Actions

1. Wire `hybrid_prism_gmsh_openfoam_v0_1` into a real optimizer mesher backend
   command/module instead of only preserving it as a config and external driver.
2. Add raw-exporter STL conditioning/gating before hybrid meshing so the
   backend can accept fresh optimizer candidates, not just preconditioned
   evidence surfaces.
3. Port dynamic farfield sizing into the hybrid driver so farfield dimensions
   scale with candidate size.
4. Investigate the hybrid no-slip pressure failure with bounded tests:
   transient freestream ramp, first-layer height sweep, prism layer
   count/transition changes, pressure solver/scheme checks, and y-plus
   reporting. Split farfield has been tested and is not sufficient by itself.
5. Add optimizer-facing database fields for mesh settings, strict quality
   metrics, severe-face localization, surface-fidelity metrics, solver-smoke
   residuals, screenshots, and artifact paths.
6. Keep snappyHexMesh as an active secondary research path for selected
   high-fidelity/scoring candidates. Next snappy test should extend
   `np12_l3_l4` no-slip steady behavior, add y-plus reporting, and try a
   conservative `kOmegaSST` wall-function setup.
7. Keep real export and CFD policy-gated per candidate; do not batch-export all
   optimizer proposals by default.

## Notes

- Keep this tracker factual and append-only where possible.
- When a decision changes, add a new entry explaining the change instead of silently editing history.
- When implementation begins, log commands, test results, generated artifacts, and validation status here.

## 2026-06-24 - Snappy wall-function y-plus probe

- Added explicit snappy layer sizing controls:
  - `custom_cfd_mesher_experiment/scripts/make_openfoam_case.py`
  - `custom_cfd_mesher_experiment/scripts/run_snappy_layer_comparison.py`
- Added no-slip RANS y-plus wall-function gate and absolute-plus-relative
  force coefficient drift handling:
  - `custom_cfd_mesher_experiment/scripts/run_no_slip_laminar_start_rans.py`
  - `custom_cfd_mesher_experiment/scripts/summarize_force_coeffs.py`
- Added conditional wall-function preset:
  - `custom_cfd_mesher_experiment/presets/snappy_openfoam_external_aero_wall_function_candidate_v0_1.json`
- Added run summary:
  - `custom_cfd_mesher_experiment/runs/snappy_wall_function_probe_20260624_summary.md`
- Best bounded fcv04 wall-function probe:
  - run root:
    `custom_cfd_mesher_experiment/runs/snappy_wall_function_probe_1layer_v0_1_20260624`.
  - surface levels `2/3`, feature level `3`, one surface layer,
    relative final layer thickness `0.75`, min layer thickness `0.15`.
  - mesh: `138,273` cells, `9,583` aircraft faces, max skew `7.161`,
    max non-orthogonality `69.999`.
  - `potentialFoam`, 100-step laminar no-slip, and 100-step laminar-start
    `kOmegaSST` all completed.
  - RANS y-plus: p50 `27.76`, p95 `100.04`, p99 `150.04`, max `205.03`.
  - RANS coefficients at step 100: `Cd=0.134867`, `Cl=0.076668`,
    `Cm=-0.064880`; force stability gate passed with the new absolute-plus-
    relative drift policy.
- Interpretation:
  - OpenFOAM wall-function boundary conditions were already present for
    turbulent `k`, `omega`, and `nut`.
  - The remaining issue is mostly near-wall mesh spacing and surface fidelity,
    not missing wall-function boundary-condition setup.
  - The wall-function candidate is much closer to the desired y-plus band and
    faster than the high-fidelity thick-layer run, but it is visibly coarser
    and does not strictly clear p50 y+ >= 30 on fcv04.
- Decision:
  - Keep the higher-fidelity snappy thick-layer path as the leading
    scoring-CFD development baseline.
  - Keep `snappy_openfoam_external_aero_wall_function_candidate_v0_1` as a
    conditional wall-function alternative pending all-five-variant validation
    and geometry-capture review.
  - Do not claim scoring-quality CFD yet.

## 2026-06-24 - Surface fidelity priority over wall-function y-plus forcing

- User clarified that aircraft surface fidelity must be maintained.
- Added preferred high-fidelity snappy preset:
  - `custom_cfd_mesher_experiment/presets/snappy_openfoam_external_aero_hifi_surface_v0_2.json`.
- Demoted the coarser wall-function preset to research-only:
  - `custom_cfd_mesher_experiment/presets/snappy_openfoam_external_aero_wall_function_candidate_v0_1.json`.
- Ran a high-fidelity-surface, one-layer fcv04 probe:
  - run root:
    `custom_cfd_mesher_experiment/runs/snappy_wall_function_probe_hifi_surface_1layer_v0_1_20260624`.
  - surface levels `3/4`, feature level `3`, one surface layer,
    relative final layer thickness `0.75`, min layer thickness `0.15`.
  - mesh: `229,376` cells, `23,962` aircraft faces, max skew `8.961`,
    max non-orthogonality `69.999`.
  - `potentialFoam`, 100-step laminar no-slip, and 100-step laminar-start
    `kOmegaSST` all completed.
  - RANS y-plus: p50 `11.35`, p95 `43.23`, p99 `65.27`, max `164.49`.
  - RANS coefficients at step 100: `Cd=0.124502`, `Cl=0.170242`,
    `Cm=-0.031281`; force stability gate passed.
- Interpretation:
  - High-fidelity surface settings preserve geometry capture and solver
    stability, but do not move median y+ into the target wall-function band.
  - Coarsening the surface improves y+ but drops fcv04 aircraft patch faces
    from `23,962` to `9,583`, which is too much fidelity loss for the default
    optimizer mesh path.
- Decision:
  - Default scoring-development path should preserve surface levels `3/4`.
  - y-plus remains a recorded warning/gate, not a reason to degrade surface
    fidelity automatically.
  - Future y-plus work should look for ways to improve near-wall treatment
    without reducing aircraft patch fidelity.

## 2026-06-24 - High-fidelity three-layer vs one-layer comparison

- Ran fresh all-five-variant high-fidelity surface comparison:
  - three-layer default root:
    `custom_cfd_mesher_experiment/runs/snappy_hifi_default_3layer_five_variant_v0_2_20260624`.
  - one-layer fast alternative root:
    `custom_cfd_mesher_experiment/runs/snappy_hifi_1layer_five_variant_v0_2_20260624`.
  - report:
    `custom_cfd_mesher_experiment/runs/snappy_hifi_surface_layer_comparison_20260624.md`.
- Added fast high-fidelity one-layer preset:
  - `custom_cfd_mesher_experiment/presets/snappy_openfoam_external_aero_hifi_surface_fast_1layer_v0_2.json`.
- Updated preferred high-fidelity preset evidence:
  - `custom_cfd_mesher_experiment/presets/snappy_openfoam_external_aero_hifi_surface_v0_2.json`.
- Results:
  - both presets preserve aircraft patch face counts on all five variants.
  - both presets passed relaxed `checkMesh -skewThreshold 12`, `potentialFoam`,
    100-step laminar no-slip, and 100-step kOmegaSST startup on all five.
  - no fatal errors or floating point exceptions occurred.
  - one-layer reduces cells by roughly 10-13 percent and is faster.
  - one-layer modestly raises p50 y+, but p50 remains below `30` on every
    variant, so it does not solve the wall-function target issue.
- Decision:
  - keep `snappy_openfoam_external_aero_hifi_surface_v0_2` as the preferred
    scoring-CFD development baseline.
  - use `snappy_openfoam_external_aero_hifi_surface_fast_1layer_v0_2` only for
    faster high-fidelity smoke/scoping runs.
  - keep the coarser wall-function preset research-only because it sacrifices
    aircraft surface capture.

## 2026-06-24 - Promoted higher-fidelity snappy surface preset

- User asked whether the current meshes were fine enough near leading and
  trailing edges; closeup review showed the v0.2 surface was usable at full
  aircraft scale but still too visibly stair-stepped around thin edges.
- Added and promoted:
  - `custom_cfd_mesher_experiment/presets/snappy_openfoam_external_aero_hifi_surface_v0_3.json`.
- Ran fresh five-variant v0.3 evidence:
  - run root:
    `custom_cfd_mesher_experiment/runs/snappy_hifi_surface_v0_3_five_variant_20260624`.
  - settings: surface levels `4/5`, feature level `5`, automatic feature
    refinement boxes enabled, three relative surface layers, 12-core snappy.
  - all five variants passed `checkMesh -skewThreshold 12`.
  - all five completed `potentialFoam`.
  - all five completed 100-step laminar no-slip and 100-step laminar-start
    `kOmegaSST`; no floating-point exceptions or fatal errors occurred.
  - mesh sizes: `739k-882k` cells, `58.7k-69.8k` aircraft patch faces.
  - max skewness range: `4.02-5.76`.
  - max non-orthogonality remained below `70` in the development gate.
  - bidirectional source-to-patch surface deviation:
    p95 `0.09-0.13 mm`, p99 `1.33-1.80 mm`, max `3.81-5.45 mm`.
  - RANS y+ remains below wall-function target:
    p50 roughly `9-11`, p95 roughly `23.6-23.9`.
- Evidence files:
  - summary CSV:
    `custom_cfd_mesher_experiment/runs/snappy_hifi_surface_v0_3_five_variant_20260624/v0_3_summary_metrics.csv`.
  - LE/TE contact sheet:
    `custom_cfd_mesher_experiment/runs/snappy_hifi_surface_v0_3_five_variant_20260624/feature_contact_sheet_le_te.png`.
  - roots/tips contact sheet:
    `custom_cfd_mesher_experiment/runs/snappy_hifi_surface_v0_3_five_variant_20260624/feature_contact_sheet_roots_tips.png`.
- Decision:
  - v0.3 supersedes v0.2 as the preferred high-fidelity OpenFOAM development
    mesh preset.
  - v0.2 can remain as the faster historical baseline, but should not be used
    for edge-fidelity-sensitive optimizer work.
  - CFD coefficients from v0.3 are still development evidence, not validated
    scoring CFD, because y+ strategy and mesh convergence are unresolved.

## 2026-06-25 - Snappy v0.3 optimizer persistence wiring

- Added platform-owned preset config:
  - `software/optimizer/configs/snappy_openfoam_external_aero_hifi_surface.v0_3.json`.
- Added OpenFOAM acceptance mode:
  - `snappy_hifi_development`.
  - Requires `mesh_ok`, at least `50,000` aircraft patch faces, max
    non-orthogonality <= `70`, max skewness <= `12`, completed
    `potentialFoam`, and low continuity/interpolated-velocity errors.
- Added CLI import command:
  - `ingest-snappy-hifi-run`.
  - Imports an existing snappy v0.3 run root into a database-backed optimizer
    workspace without rerunning meshing or OpenFOAM.
  - Persists `openfoam_smoke_validation` for `potentialFoam`.
  - Persists `openfoam_steady_validation` for the bounded kOmegaSST
    no-slip development run when present.
  - Keeps `scoring_allowed=false`.
- Added campaign completion helper so imported evidence campaigns do not remain
  marked `running`.
- Imported the current five-variant v0.3 evidence root:
  - source:
    `custom_cfd_mesher_experiment/runs/snappy_hifi_surface_v0_3_five_variant_20260624`.
  - workspace:
    `aircraft_optimizer_platform/runs/snappy_hifi_openfoam_persistence_v0_3`.
- Database result:
  - `5` candidates.
  - `5` complete evaluations.
  - `5` successful `snappy_surface_fidelity_validation` attempts.
  - `5` successful `openfoam_smoke_validation` attempts.
  - `5` successful `openfoam_steady_validation` attempts.
  - `70` artifacts, including `5` `openfoam_yplus_field` artifacts.
  - `0` failures.
  - campaign status: `complete`.
- Validation:
  - `python -m pytest -q` from `software/optimizer`: `54 passed`.
  - `check-openfoam-result` with `--acceptance-mode snappy_hifi_development`
    passed on the fcv01 v0.3 case with zero failed checks.
  - Sample persisted surface metric: fcv01 bidirectional p95 deviation
    `0.101655672849 mm`, `snappy_surface.ready=1`.
  - Persisted y-plus metric ranges across the five kOmegaSST steady attempts:
    p50 `9.18-11.23`, p95 `23.59-23.88`, max `79.84-118.11`.
  - Added `snappy_hifi_coefficient_development` for bounded non-scoring
    steady evidence. The policy requires completed solver execution, no fatal
    errors, force coefficients, y-plus availability, local continuity <=
    `1e-4`, final velocity residual <= `1e-4`, Cd/Cl/Cm final-window spans <=
    `0.005`, y-plus p95 <= `60`, and y-plus max <= `250`.
  - Rebuilt the persisted snappy v0.3 workspace through that policy; all five
    steady attempts passed with zero failed checks.
- Decision:
  - snappy v0.3 is now the optimizer's current default OpenFOAM
    development-mesh evidence path.
  - The next implementation step should wrap the snappy v0.3 mesh/smoke/steady
    sequence behind an optimizer backend interface. Keep force coefficients
    non-scoring until mesh convergence, wall-treatment, and validation improve.

## 2026-06-25 - Snappy backend wrapper and first y-plus study

- Added wrapped backend CLI:
  - `run-snappy-openfoam-backend`.
  - Runs the current snappy v0.3 meshing harness, `potentialFoam`, bounded
    laminar no-slip startup, bounded laminar-start `kOmegaSST`, then imports
    results into the optimizer database.
  - Also supports `--skip-execution` to import an existing backend run root.
- Smoke-tested wrapper import against existing v0.3 evidence:
  - workspace:
    `aircraft_optimizer_platform/runs/snappy_backend_wrapper_smoke_v0_1`.
  - result: `5` candidates, `5` complete evaluations, `15` module attempts,
    `70` artifacts, `0` failures.
- Added bounded y-plus study harness:
  - `custom_cfd_mesher_experiment/scripts/run_snappy_yplus_study.py`.
  - Run root:
    `custom_cfd_mesher_experiment/runs/snappy_yplus_study_fcv04_v0_2_20260625`.
  - Variant: `fcv04_compact_wide_tail`.
- y-plus study results:
  - Baseline fcv04 v0.3 reference: `779,782` cells, `68,000` aircraft faces,
    y+ p50 `11.2138465`, p95 `23.7886005`, max `84.753544`.
  - `rel_3layer_0p55_0p08`: `754,094` cells, `68,013` aircraft faces,
    max skewness `4.0165875`, max non-orthogonality `69.998976`,
    y+ p50 `7.2044735`, p95 `35.696176`, max `95.361091`.
  - `rel_1layer_0p75_0p12`: `649,650` cells, `68,013` aircraft faces,
    max skewness `4.0165875`, max non-orthogonality `69.999585`,
    y+ p50 `6.4525157`, p95 `24.610902`, max `84.918091`.
  - Both probes completed mesh, `potentialFoam`, laminar no-slip, and
    laminar-start `kOmegaSST` without fatal errors.
- Decision:
  - Thicker relative layers do not move this high-fidelity surface mesh toward
    wall-function y+; both probes lowered median y+ versus baseline.
  - Do not degrade surface fidelity or simply inflate layer thickness to chase
    wall-function y+.
  - Next y-plus work should either test an explicitly resolved-wall strategy
    with tighter first-cell control and convergence evidence, or run a bounded
    mesh-convergence/validation comparison before changing the default preset.
- Validation:
  - `python -m pytest -q` from `software/optimizer`: `54 passed`.

## 2026-06-25 - Resolved-wall y-plus probe and wall-treatment split

- Added explicit snappy/OpenFOAM wall-treatment support:
  - `custom_cfd_mesher_experiment/scripts/setup_incompressible_fluid_smoke.py`
    now accepts `--wall-treatment wall_function|low_re`.
  - `low_re` uses `kLowReWallFunction` and `nutLowReWallFunction` on the
    aircraft patch while preserving `omegaWallFunction`.
  - `custom_cfd_mesher_experiment/scripts/run_no_slip_laminar_start_rans.py`
    passes wall treatment through to the kOmegaSST leg.
  - `custom_cfd_mesher_experiment/scripts/run_snappy_yplus_study.py` now has
    separate `wall-function-probe` and `resolved-wall-probe` modes, exposes
    layer expansion ratio, and records direct `checkMesh` quality fields.
- Ran a bounded absolute-layer resolved-wall probe on `fcv04_compact_wide_tail`:
  - Run root:
    `custom_cfd_mesher_experiment/runs/snappy_resolved_yplus_fcv04_v0_1_20260625`.
  - Config: `abs_4layer_0p18mm_0p015mm`, absolute layers, four surface
    layers, expansion ratio `1.18`, final layer thickness `0.00018 m`,
    minimum thickness `0.000015 m`.
  - Mesh result: `752,613` cells, `880,271` points, `68,013` aircraft faces,
    max non-orthogonality `73.707673`, `17` severe non-orthogonal faces,
    max skewness `4.0165875`, strict `checkMesh` OK, `potentialFoam` OK.
  - Surface-fidelity settings were unchanged from the high-fidelity surface
    recipe: surface levels `4/5`, feature level `5`, automatic feature boxes.
  - Wall-function RANS result: rejected. It ran away by the mid-run and was
    stopped after force coefficients reached nonphysical magnitudes. Partial
    y-plus at time `50`: p50 `0.21364296`, p95 `9.18769468`, max `2290.6981`.
- Retried the same absolute-layer mesh with `low_re` wall treatment only:
  - Run root:
    `custom_cfd_mesher_experiment/runs/snappy_resolved_lowre_solver_retry_fcv04_v0_1_20260625`.
  - Mesh was copied from the absolute-layer probe; no remeshing was performed.
  - RANS result: rejected. Low-Re wall fields were applied, but force
    coefficients were already nonphysical by time `16-17`.
  - Partial y-plus at time `10`: p50 `4.6706769`, p95 `34.9852982`,
    max `955.32938`.
- Decision:
  - Do not adopt ultra-thin absolute wall layers as the optimizer default.
  - Do not assume y-plus can be fixed by layer thickness alone; wall treatment,
    solver startup, and stability gates matter.
  - Preserve the current snappy v0.3 high-fidelity surface preset as the
    default development path until a replacement passes all five variants.
  - The best near-term candidate remains the stable one-layer relative probe
    from `snappy_yplus_study_fcv04_v0_2_20260625`: it preserved aircraft patch
    face count, completed mesh/potential/laminar/kOmegaSST, reduced cells to
    `649,650`, and gave y-plus p50 `6.4525157`, p95 `24.610902`, max
    `84.918091`. It still does not meet a wall-function y-plus band and must
    be tested on all five variants before any default change.
- Next action:
  - Run the one-layer relative candidate across all five variants with the
    same surface-fidelity checks. If it preserves surface capture and solver
    stability across the set, promote it to a candidate preset. If not, keep
    v0.3 and move scoring-CFD work to solver-startup/convergence strategy
    rather than further thinning wall layers.

## 2026-06-25 - Promoted snappy one-layer v0.4 development backend

- Ran the stable one-layer relative snappy preset across all five approved
  faired-cap variants:
  - Run root:
    `custom_cfd_mesher_experiment/runs/snappy_hifi_1layer_all_variants_candidate_20260625`.
  - Summary:
    `custom_cfd_mesher_experiment/runs/snappy_hifi_1layer_all_variants_candidate_20260625/one_layer_candidate_summary.md`.
  - Metrics CSV:
    `custom_cfd_mesher_experiment/runs/snappy_hifi_1layer_all_variants_candidate_20260625/one_layer_candidate_metrics.csv`.
- Mesh and solver result:
  - strict development OpenFOAM mesh gate: `5/5` pass.
  - `potentialFoam -writep`: `5/5` complete.
  - 100-step laminar no-slip: `5/5` complete.
  - 100-step laminar-start `kOmegaSST`: `5/5` complete.
  - No fatal errors or floating-point exceptions in the bounded solver ladder.
  - Mesh/potential runtime total: `2008.7 s`.
  - Solver ladder runtime total: `3783.3 s`.
- Surface fidelity result:
  - Aircraft patch faces: `58,749-69,652`.
  - Bidirectional source-to-patch p95 deviation: `0.0928-0.1260 mm`.
  - Bidirectional p99 deviation: `1.3524-1.8108 mm`.
  - Bidirectional max deviation: `4.2127-5.3169 mm`.
  - All five variants remain inside the current development surface-fidelity
    policy.
- y-plus result:
  - p50 range: `5.97-6.66`.
  - p95 range: `20.60-25.50`.
  - max range: `77.98-120.84`.
  - This is stable development evidence, not a validated wall-function/scoring
    setup.
- Added platform preset:
  - `software/optimizer/configs/snappy_openfoam_external_aero_hifi_surface.v0_4.json`.
  - v0.4 keeps surface levels `4/5`, feature level `5`, automatic feature
    refinement boxes, one relative surface layer, and wall-function
    kOmegaSST development smoke.
- Updated optimizer backend wrapper:
  - Default snappy backend preset is now v0.4.
  - The wrapper now passes `--layer-expansion-ratio` explicitly.
  - The wrapper now passes configured `--wall-treatment` into the
    laminar-start RANS ladder.
  - `ingest-snappy-hifi-run` now reads pipeline version and surface-fidelity
    report name from the preset instead of hardcoding v0.3 paths.
- Imported v0.4 evidence into:
  - `aircraft_optimizer_platform/runs/snappy_hifi_openfoam_persistence_v0_4`.
  - Import result: five candidates, five complete evaluations, five successful
    surface-fidelity attempts, five successful OpenFOAM smoke attempts, five
    successful OpenFOAM steady attempts, seventy artifacts, zero failures.
- Added helper:
  - `custom_cfd_mesher_experiment/scripts/convert_vtk_surface_to_stl.py`.
  - Converts OpenFOAM/ParaView aircraft patch VTK output to STL for
    source-to-patch fidelity audits.
- Decision:
  - Promote `snappy_openfoam_external_aero_hifi_surface.v0_4.json` as the
    current default development backend for optimizer meshing and non-scoring
    OpenFOAM smoke/steady evidence.
  - Do not treat v0.4 force coefficients as trusted scoring CFD until a
    bounded mesh-convergence, wall-treatment/y-plus, and validation study
    exists.
- Validation:
  - `python -m pytest -q` from `software/optimizer`: `54 passed`.
  - Final process check found no active `foamRun`, `snappyHexMesh`, `mpirun`,
    `checkMesh`, or `potentialFoam` jobs.
- Next action:
  - Run the v0.4 backend wrapper on a fresh exported aircraft STL path, then
    start a controlled convergence/scoring-CFD study before letting CFD
    coefficients influence optimizer objective scores.

## 2026-06-25 - Added STL-native ranking-CFD tier and parallel no-slip solver support

- Decision:
  - Do not route normal optimizer candidates through VLM, OpenVSP
    reconstruction, or other alternate geometry definitions.
  - The generated STL remains the geometry source of truth for aerodynamic
    evaluation.
  - Cheap pre-screening should reject physically impossible candidates, but
    plausible candidates should get a bounded CFD signal.
  - Split CFD into tiers:
    - ranking CFD: medium-cost, STL-native, useful for relative ordering,
      always non-scoring until calibrated.
    - confirmation/scoring-development CFD: stricter mesh and solver reruns on
      survivors, still non-scoring until validation gates improve.
- Added ranking preset:
  - `software/optimizer/configs/snappy_openfoam_external_aero_ranking.v0_1.json`.
  - Uses the current snappy/OpenFOAM STL path with surface levels `4/5`,
    feature level `5`, one relative surface layer, 12-core meshing, 8-rank
    parallel no-slip solver execution, and a 60-step kOmegaSST ladder.
  - Keeps `scoring_allowed=false`.
  - Ranking surface-fidelity review threshold is relaxed versus scoring:
    p95 over `1.0 mm` requires review, while scoring still treats p95 over
    `0.25 mm` as not ready.
- Updated no-slip solver ladder:
  - `custom_cfd_mesher_experiment/scripts/run_no_slip_laminar_start_rans.py`
    now supports `--parallel-procs`.
  - Parallel mode writes `system/decomposeParDict`, runs `decomposePar -force`,
    executes `foamRun -parallel -solver incompressibleFluid`, and reconstructs
    the latest time for summaries.
  - Force-coefficient and y-plus summaries now tolerate reconstructed/latest
    time paths instead of assuming only serial output layout.
  - Short or selected probes now write distinct default summary filenames such
    as `no_slip_ladder_summary_end2_np2.json` instead of overwriting the
    canonical full-run `no_slip_ladder_summary.json`.
- Updated optimizer wrapper:
  - `run-snappy-openfoam-backend` now passes configured solver
    `parallel_procs` and optional `rans_timeout_s` from the preset to the
    no-slip solver ladder.
- Fresh no-inlet evidence informing this split:
  - Exported STL:
    `dual_contouring/direct_sparse_sdf_mc_experiment/stl/direct_sdf_oml_spacing_1p0.stl`.
  - Medium v0.4 backend run:
    `custom_cfd_mesher_experiment/runs/fresh_no_inlet_snappy_v0_4_backend_v0_1`.
  - Optimizer import workspace:
    `aircraft_optimizer_platform/runs/fresh_no_inlet_snappy_v0_4_backend_v0_1_import`.
  - Mesh/solver completed and produced stable non-scoring RANS evidence, but
    scoring-level surface fidelity failed on the larger fresh STL
    (`p95 = 0.775 mm`, gate `0.25 mm`).
  - Fine mesh passed the strict surface-fidelity p95 gate, but mesh plus
    potential took about `83 min`, making it unsuitable for every optimizer
    candidate.
- Validation:
  - `python -m py_compile custom_cfd_mesher_experiment/scripts/run_no_slip_laminar_start_rans.py aircraft_optimizer_platform/software/optimizer/src/aircraft_optimizer/cli/main.py`: pass.
  - `python -m json.tool software/optimizer/configs/snappy_openfoam_external_aero_ranking.v0_1.json`: pass.
  - `python -m pytest -q` from `software/optimizer`: `54 passed`.
  - Two-step MPI solver smoke on existing `fcv01_long_glider` v0.4 mesh:
    - command used `--end-time 2 --write-interval 1 --parallel-procs 2`.
    - laminar completed, no FPE.
    - kOmegaSST completed, no FPE.
    - smoke summary:
      `custom_cfd_mesher_experiment/runs/snappy_hifi_1layer_all_variants_candidate_20260625/parallel_solver_smoke_end2_summary.json`.
    - variant summary:
      `custom_cfd_mesher_experiment/runs/snappy_hifi_1layer_all_variants_candidate_20260625/fcv01_long_glider/parallel_solver_smoke_end2_variant_summary.json`.
- Next action:
  - Run `snappy_openfoam_external_aero_ranking.v0_1.json` on one fresh
    exported STL end-to-end, then start a small rank-correlation study
    comparing ranking CFD versus finer reruns on the same STL candidates.

## 2026-06-25 - Fresh STL ranking-CFD run and first rank-correlation evidence

- Ran the ranking-CFD preset end-to-end on the fresh no-inlet STL:
  - Preset:
    `software/optimizer/configs/snappy_openfoam_external_aero_ranking.v0_1.json`.
  - Input STL:
    `dual_contouring/direct_sparse_sdf_mc_experiment/stl/direct_sdf_oml_spacing_1p0.stl`.
  - Run root:
    `custom_cfd_mesher_experiment/runs/fresh_no_inlet_snappy_ranking_v0_1_20260625`.
  - Variant:
    `fresh_no_inlet_oml_ranking_v0_1`.
  - Aircraft-only ISO:
    `custom_cfd_mesher_experiment/runs/fresh_no_inlet_snappy_ranking_v0_1_20260625/fresh_no_inlet_oml_ranking_v0_1/aircraft_iso.png`.
- Mesh/smoke result:
  - Total mesh/potential/surface-fidelity runtime: `713.3 s`.
  - Mesh driver runtime: `668.1 s`.
  - Cells: `899,245`.
  - Points: `1,058,715`.
  - Aircraft patch faces from `checkMesh`: `90,085`.
  - Candidate surface faces from VTK-to-STL: `200,684`.
  - Max non-orthogonality: `69.999141`.
  - Max skewness: `3.5947067`.
  - `potentialFoam -writep`: complete, no warnings, continuity error
    `2.2202515e-07`, interpolated velocity error `0.00021142464`.
- Surface fidelity:
  - Reference STL triangles: `1,147,244`.
  - Bidirectional p95: `0.7754 mm`.
  - Bidirectional p99: `2.1931 mm`.
  - Bidirectional max: `2.7923 mm`.
  - This fails the strict scoring p95 gate (`0.25 mm`) but remains within the
    ranking-CFD review threshold (`1.0 mm` p95).
- No-slip ranking solver result:
  - Summary:
    `custom_cfd_mesher_experiment/runs/fresh_no_inlet_snappy_ranking_v0_1_20260625/no_slip_ladder_summary_end60_np8.json`.
  - 8-rank MPI laminar leg: complete, no FPE, runtime `169.6 s`.
  - 8-rank MPI kOmegaSST leg: complete, no FPE, runtime `196.5 s`.
  - Final kOmegaSST coefficients at time `60`:
    Cd `0.0935024102`, Cl `-0.0992240036`, Cm `0.0927873847`.
  - Final kOmegaSST residuals:
    pressure `6.17184e-07`, velocity `3.3962763e-06`.
  - kOmegaSST tail force gate: pass.
  - y-plus: p50 `4.5920`, p95 `9.0586`, max `39.7930`; still below the
    wall-function target band and therefore not scoring CFD.
- Imported accepted run into:
  - `aircraft_optimizer_platform/runs/fresh_no_inlet_snappy_ranking_v0_1_import_with_steady`.
  - Import result: one candidate, one complete evaluation, three module
    attempts.
  - `openfoam_smoke_validation`: success.
  - `openfoam_steady_validation`: success.
  - `snappy_surface_fidelity_validation`: failed, category
    `meshing.surface_fidelity_gate_failed`, as expected for strict scoring
    fidelity.
  - Earlier partial import without steady evidence remains at
    `aircraft_optimizer_platform/runs/fresh_no_inlet_snappy_ranking_v0_1_import`;
    use the `_with_steady` workspace as the accepted import.
- Added ranking policy/import hardening:
  - Registered `snappy_ranking_cfd_development` for OpenFOAM smoke validation.
  - Registered `SNAPPY_RANKING_CFD_DEVELOPMENT_POLICY` for steady validation.
  - `run-snappy-openfoam-backend` now imports steady results using the preset
    acceptance mode instead of hardwiring the high-fidelity policy.
  - Import now finds `no_slip_ladder_summary_*.json` when the canonical
    `no_slip_ladder_summary.json` is absent.
- Added rank-correlation scaffold:
  - Script:
    `custom_cfd_mesher_experiment/scripts/compare_force_coeff_rankings.py`.
  - Output folder:
    `aircraft_optimizer_platform/runs/cfd_rank_correlation_v0_1`.
  - Report:
    `aircraft_optimizer_platform/runs/cfd_rank_correlation_v0_1/ranking_vs_confirmation_summary.md`.
  - CSV:
    `aircraft_optimizer_platform/runs/cfd_rank_correlation_v0_1/ranking_vs_confirmation_coefficients.csv`.
- First proxy rank-correlation result:
  - Basis: existing five-variant v0.4 kOmegaSST runs.
  - Method: compare five-step mean coefficients at time `60` against time
    `100` from the same completed runs. This is a proxy, not a separate mesh
    convergence study.
  - Cd low-is-better Spearman: `1.000`, exact order match.
  - Cl high-is-better Spearman: `1.000`, exact order match.
  - Absolute Cm low-is-better Spearman: `0.700`, max rank delta `2`.
  - Decision: ranking CFD is promising for Cd/Cl ordering; do not trust moment
    ranking yet.
- Validation:
  - `python -m py_compile` on touched Python scripts/modules: pass.
  - `python -m pytest -q` from `software/optimizer`: `55 passed`.
  - Final process check found no active `foamRun`, `snappyHexMesh`, `mpirun`,
    `checkMesh`, `potentialFoam`, `decomposePar`, or `reconstructPar` jobs.
- Next action:
  - Run the ranking preset on two or three additional fresh/exported STL
    variants, then compare their ordering against stricter confirmation reruns
    before letting ranking CFD influence optimizer objective scores.

## 2026-06-25 - Three-variant ranking CFD batch and separate-root confirmation comparison

- Ran the ranking-CFD preset on three additional faired-cap variants:
  - Preset:
    `software/optimizer/configs/snappy_openfoam_external_aero_ranking.v0_1.json`.
  - Run root:
    `custom_cfd_mesher_experiment/runs/faired_cap_snappy_ranking_v0_1_three_variant_20260625`.
  - Imported workspace:
    `aircraft_optimizer_platform/runs/faired_cap_snappy_ranking_v0_1_three_variant_import`.
  - Variants:
    `fcv02_short_swept`, `fcv03_high_aspect_mild`,
    `fcv04_compact_wide_tail`.
- Important provenance caveat:
  - The built-in faired-cap selector in
    `custom_cfd_mesher_experiment/scripts/run_snappy_layer_comparison.py`
    uses prepared/remeshed `aircraft_surface_iso.stl` inputs from
    `custom_cfd_mesher_experiment/runs/faired_cap_gmsh_openfoam_20260623`,
    not the original direct exporter STL files under
    `dual_contouring/direct_sparse_sdf_mc_experiment/stl`.
  - Treat this batch as prepared-surface ranking/confirmation evidence. The
    fresh no-inlet run remains the current direct-export STL ranking example.
- Mesh/potential batch result:
  - `pass_count`: `3/3`.
  - Mesh/potential/surface-fidelity total runtime: about `789.5 s`.
  - `fcv02_short_swept`: `634,975` cells, `58,749` aircraft faces,
    max non-orthogonality `69.997984`, max skewness `5.6756764`,
    surface p95 `0.0928 mm`.
  - `fcv03_high_aspect_mild`: `680,790` cells, `67,576` aircraft faces,
    max non-orthogonality `69.999382`, max skewness `4.7108569`,
    surface p95 `0.1260 mm`.
  - `fcv04_compact_wide_tail`: `649,650` cells, `68,013` aircraft faces,
    max non-orthogonality `69.999585`, max skewness `4.0165875`,
    surface p95 `0.1023 mm`.
  - Aircraft-only ISO screenshots:
    - `custom_cfd_mesher_experiment/runs/faired_cap_snappy_ranking_v0_1_three_variant_20260625/fcv02_short_swept/aircraft_iso.png`.
    - `custom_cfd_mesher_experiment/runs/faired_cap_snappy_ranking_v0_1_three_variant_20260625/fcv03_high_aspect_mild/aircraft_iso.png`.
    - `custom_cfd_mesher_experiment/runs/faired_cap_snappy_ranking_v0_1_three_variant_20260625/fcv04_compact_wide_tail/aircraft_iso.png`.
- 60-step no-slip ranking solver result:
  - Summary:
    `custom_cfd_mesher_experiment/runs/faired_cap_snappy_ranking_v0_1_three_variant_20260625/no_slip_ladder_summary_end60_np8.json`.
  - `3/3` laminar complete, no FPE.
  - `3/3` laminar-start kOmegaSST complete, no FPE.
  - `fcv02_short_swept`: Cd `0.0938845222`, Cl `0.0597294787`,
    Cm `0.00595926717`, y-plus p50 `4.2571`.
  - `fcv03_high_aspect_mild`: Cd `0.11191339`, Cl `0.0887090667`,
    Cm `0.0243979688`, y-plus p50 `4.7091`.
  - `fcv04_compact_wide_tail`: Cd `0.106399954`, Cl `0.0846027573`,
    Cm `-0.00439852155`, y-plus p50 `4.4862`.
  - All three still sit below the wall-function y-plus target band and remain
    non-scoring ranking/development CFD.
- Import result:
  - Workspace summary: three candidates, three complete evaluations,
    nine module attempts, zero failures.
  - `snappy_surface_fidelity_validation`: `3/3` success.
  - `openfoam_smoke_validation`: `3/3` success.
  - `openfoam_steady_validation`: `3/3` success.
- Extended rank-correlation script:
  - `custom_cfd_mesher_experiment/scripts/compare_force_coeff_rankings.py`
    now supports a separate `--confirmation-run-root`.
- Separate-root rank-correlation result:
  - Ranking root:
    `custom_cfd_mesher_experiment/runs/faired_cap_snappy_ranking_v0_1_three_variant_20260625`.
  - Confirmation root:
    `custom_cfd_mesher_experiment/runs/snappy_hifi_1layer_all_variants_candidate_20260625`.
  - Output:
    `aircraft_optimizer_platform/runs/cfd_rank_correlation_separate_roots_v0_1`.
  - Report:
    `aircraft_optimizer_platform/runs/cfd_rank_correlation_separate_roots_v0_1/ranking_vs_confirmation_summary.md`.
  - Cd low-is-better Spearman: `1.000`, exact order match.
  - Cl high-is-better Spearman: `1.000`, exact order match.
  - Absolute Cm low-is-better Spearman: `0.500`, max rank delta `1`.
  - Decision: this strengthens the case that 60-step ranking CFD is useful for
    Cd/Cl ordering on prepared faired-cap variants, but moment ranking remains
    weak and should not drive optimization yet.
- Validation:
  - No active `foamRun`, `snappyHexMesh`, `mpirun`, `checkMesh`,
    `potentialFoam`, `decomposePar`, or `reconstructPar` processes after the
    run.
- Next action:
  - Add a direct-export batch runner or selector path that uses the original
    direct exporter STLs instead of prepared Gmsh-remeshed surfaces, then repeat
    the same ranking-vs-confirmation comparison on raw direct STL inputs.

## 2026-06-25 - Raw direct STL ranking/confirmation batch

- Added raw direct-STL batch support:
  - `custom_cfd_mesher_experiment/scripts/run_snappy_layer_comparison.py`
    now accepts `--input-stl-map`.
  - Raw batch config:
    `software/optimizer/configs/raw_direct_faired_cap_stl_batch.v0_1.json`.
  - This avoids accidentally using the built-in prepared Gmsh-remeshed
    faired-cap inputs when the goal is raw exporter STL evidence.
- Ran snappy/OpenFOAM ranking mesh/smoke on three raw direct exporter STLs:
  - Run root:
    `custom_cfd_mesher_experiment/runs/raw_direct_faired_cap_snappy_ranking_v0_1_three_variant_20260625`.
  - Inputs:
    - `dual_contouring/direct_sparse_sdf_mc_experiment/stl/direct_sdf_oml_fcv02_short_swept_faired_cap_spacing_1p0.stl`.
    - `dual_contouring/direct_sparse_sdf_mc_experiment/stl/direct_sdf_oml_fcv03_high_aspect_mild_faired_cap_spacing_1p0.stl`.
    - `dual_contouring/direct_sparse_sdf_mc_experiment/stl/direct_sdf_oml_fcv04_compact_wide_tail_faired_cap_spacing_1p0.stl`.
  - Mesh/smoke pass: `3/3`.
  - Mesh/potential/surface-fidelity runtime: about `1854.3 s`.
- Raw mesh quality and surface fidelity:
  - `fcv02_short_swept`: `750,492` cells, `875,721` points,
    `71,039` aircraft faces, max non-orthogonality `69.999896`, max skewness
    `6.0315084`, surface p95 `0.1220 mm`, p99 `1.3377 mm`, max `4.0305 mm`.
  - `fcv03_high_aspect_mild`: `773,593` cells, `913,527` points,
    `82,388` aircraft faces, max non-orthogonality `69.999021`, max skewness
    `4.3792139`, surface p95 `0.1528 mm`, p99 `1.7716 mm`, max `4.5216 mm`.
  - `fcv04_compact_wide_tail`: `800,615` cells, `944,619` points,
    `88,141` aircraft faces, max non-orthogonality `69.996891`, max skewness
    `3.7162671`, surface p95 `0.1224 mm`, p99 `1.4175 mm`, max `4.4852 mm`.
  - All three raw direct STL cases pass the current strict
    scoring-development surface p95 gate of `0.25 mm`.
- Aircraft-only ISO screenshots:
  - `custom_cfd_mesher_experiment/runs/raw_direct_faired_cap_snappy_ranking_v0_1_three_variant_20260625/fcv02_short_swept/aircraft_iso.png`.
  - `custom_cfd_mesher_experiment/runs/raw_direct_faired_cap_snappy_ranking_v0_1_three_variant_20260625/fcv03_high_aspect_mild/aircraft_iso.png`.
  - `custom_cfd_mesher_experiment/runs/raw_direct_faired_cap_snappy_ranking_v0_1_three_variant_20260625/fcv04_compact_wide_tail/aircraft_iso.png`.
- Ran 60-step ranking no-slip solver ladder:
  - Summary:
    `custom_cfd_mesher_experiment/runs/raw_direct_faired_cap_snappy_ranking_v0_1_three_variant_20260625/no_slip_ladder_summary_end60_np8.json`.
  - `3/3` laminar complete, no FPE.
  - `3/3` laminar-start kOmegaSST complete, no FPE.
  - Final kOmegaSST:
    - `fcv02_short_swept`: Cd `0.0942421604`, Cl `0.0588768756`,
      Cm `0.00543586887`.
    - `fcv03_high_aspect_mild`: Cd `0.114208533`, Cl `0.0912733042`,
      Cm `0.0273501301`.
    - `fcv04_compact_wide_tail`: Cd `0.105019949`, Cl `0.0822385326`,
      Cm `-0.00649194812`.
- Ran 100-step confirmation no-slip solver ladder on the same raw direct-STL
  meshes:
  - Summary:
    `custom_cfd_mesher_experiment/runs/raw_direct_faired_cap_snappy_ranking_v0_1_three_variant_20260625/no_slip_ladder_summary_end100_np8.json`.
  - `3/3` laminar-start kOmegaSST complete, no FPE.
  - Final kOmegaSST:
    - `fcv02_short_swept`: Cd `0.0939788781`, Cl `0.0675025581`,
      Cm `0.00387756739`.
    - `fcv03_high_aspect_mild`: Cd `0.113498036`, Cl `0.0983549355`,
      Cm `0.0323698937`.
    - `fcv04_compact_wide_tail`: Cd `0.104864047`, Cl `0.0902197079`,
      Cm `-0.00856241515`.
  - 100-step y-plus remains below the wall-function target band:
    p50 about `4.83-5.38`, p95 about `20.4-22.5`.
- Raw direct-STL rank-correlation result:
  - Output folder:
    `aircraft_optimizer_platform/runs/cfd_rank_correlation_raw_direct_stl_v0_1`.
  - Report:
    `aircraft_optimizer_platform/runs/cfd_rank_correlation_raw_direct_stl_v0_1/ranking_vs_confirmation_summary.md`.
  - Cd low-is-better Spearman: `1.000`, exact order match.
  - Cl high-is-better Spearman: `1.000`, exact order match.
  - Absolute Cm low-is-better Spearman: `1.000`, exact order match.
  - This is the strongest ranking-CFD evidence so far because it uses raw
    direct exporter STLs rather than prepared/remeshed surfaces.
- Imported the raw direct-STL batch into:
  - `aircraft_optimizer_platform/runs/raw_direct_faired_cap_snappy_ranking_v0_1_import`.
  - Import result: three candidates, three complete evaluations, nine module
    attempts, zero failures.
  - `snappy_surface_fidelity_validation`: `3/3` success.
  - `openfoam_smoke_validation`: `3/3` success.
  - `openfoam_steady_validation`: `3/3` success.
- Validation:
  - `python -m py_compile` on touched mesher scripts: pass.
  - Final process check found no active `foamRun`, `snappyHexMesh`, `mpirun`,
    `checkMesh`, `potentialFoam`, `decomposePar`, or `reconstructPar` jobs.
- Next action:
  - Promote the raw direct-STL ranking path as the current optimizer CFD
    ranking candidate, then integrate it into the sequential optimizer flow
    behind explicit cost/fidelity gates. Keep final scoring blocked until
    wall-treatment/y-plus and external validation are improved.

### 2026-06-25 - CFD scoring terminology cleanup

- Decision:
  - Use one scoring concept with explicit qualifiers instead of saying rough CFD
    is "non-scoring."
  - `rough_scoring`: medium-cost CFD evidence usable for optimizer-relative
    candidate ranking.
  - `confirmation_scoring_development`: stricter survivor reruns used to check
    whether rough rankings hold.
  - `final_scoring`: engineering/reporting score that remains blocked until
    wall treatment, y-plus, mesh convergence, and validation evidence improve.
- Implementation:
  - Added `scoring_tier`, `score_usage`, `rough_scoring_allowed`, and
    `final_scoring_allowed` to OpenFOAM steady validation results.
  - Kept legacy `scoring_allowed` as an alias for `final_scoring_allowed`.
  - Marked `snappy_ranking_cfd_development` as `scoring_tier=rough_scoring`
    with `score_usage=optimizer_relative_ranking`.
  - Updated the snappy ranking preset and persistence metadata so force
    coefficients can be used for rough optimizer scoring while final scoring
    remains blocked.
- Validation:
  - `pytest software\optimizer\tests\test_v0_1_flow.py -q`: `55 passed`.

### 2026-06-25 - Rough CFD scoring module

- Added `rough_cfd_scoring` as a real scoring module for optimizer-relative CFD
  ranking.
- Added default user-editable config:
  - `software/optimizer/configs/rough_cfd_stable_efficient_drone.v0_1.json`.
- The default score targets a stable, efficient fixed-wing drone:
  - high rough L/D,
  - low Cd,
  - usable positive Cl,
  - small absolute Cm,
  - solver confidence,
  - mesh/surface-fidelity confidence.
- User control:
  - `ingest-snappy-hifi-run` and `run-snappy-openfoam-backend` now accept
    `--rough-scoring-config`.
  - The snappy ranking preset points to the default scoring config through
    `optimizer_import.rough_scoring_config`.
- Fresh raw direct-STL rough-scoring import:
  - Workspace:
    `aircraft_optimizer_platform/runs/raw_direct_faired_cap_snappy_rough_scoring_v0_1_import`.
  - Source run root:
    `custom_cfd_mesher_experiment/runs/raw_direct_faired_cap_snappy_ranking_v0_1_three_variant_20260625`.
  - Imported three candidates, three complete evaluations, twelve module
    attempts, zero failures.
  - `rough_cfd_scoring`: `3/3` success.
- Current rough score order:
  - `fcv04_compact_wide_tail`: `93.663`.
  - `fcv02_short_swept`: `90.704`.
  - `fcv03_high_aspect_mild`: `88.201`.
- Interpretation:
  - This is now real rough optimizer scoring.
  - It is not final engineering scoring; final scoring remains blocked until
    wall treatment/y-plus, mesh convergence, and validation improve.
- Validation:
  - `python -m py_compile` on touched rough-scoring and CLI files: pass.
  - Focused optimizer test file: `57 passed`.

### 2026-06-25 - Rough scoring selection and alpha-sweep policy

- Added explicit negative detractors to rough CFD scoring:
  - high absolute pitching moment,
  - lift outside the configured usable band,
  - coefficient instability,
  - surface-deviation risk,
  - solver residual/continuity risk.
- Re-imported raw direct-STL batch with detractor-enabled rough scoring:
  - Workspace:
    `aircraft_optimizer_platform/runs/raw_direct_faired_cap_snappy_rough_scoring_v0_2_import`.
  - Optimizer run:
    `optrun_5cead0f1e0014a48a4e74380ba284d1f`.
  - Primary objective: `score.rough_total`.
  - Best candidate by optimizer report:
    `fcv04_compact_wide_tail`.
  - Counts: three candidates, three complete evaluations, twelve module
    attempts, one optimizer run, zero failures.
- Current score order remained:
  - `fcv04_compact_wide_tail`: `93.663`.
  - `fcv02_short_swept`: `90.704`.
  - `fcv03_high_aspect_mild`: `88.201`.
- Target assessment:
  - The current targets are acceptable placeholders for single-condition
    relative ranking, but they are not mission-calibrated.
  - A single alpha can produce useful ranking evidence, but it cannot estimate
    a real usable L/D envelope or Cm trend.
- Alpha-sweep decision:
  - Record planned rough scoring sweep angles: `-2, 0, +4, +8 deg`.
  - Reuse the same mesh per candidate.
  - Run separate solver cases with rotated freestream vectors.
  - Aggregate best usable L/D, CL slope, and Cm trend before using rough CFD as
    a stronger optimizer objective.

### 2026-06-26 - Implemented mesh-reuse alpha sweep and polar-aware rough scoring

- Added `+10 deg` to the rough CFD alpha sweep.
- Added alpha-sweep execution helper:
  - `custom_cfd_mesher_experiment/scripts/run_no_slip_alpha_sweep.py`.
  - Reuses each existing OpenFOAM mesh and creates separate laminar-start plus
    `kOmegaSST` cases for each alpha.
  - Uses freestream direction `(cos(alpha), 0, sin(alpha))`, drag axis aligned
    with freestream, lift axis `(-sin(alpha), 0, cos(alpha))`, and pitch axis
    `Y`.
- Full raw direct-STL sweep completed:
  - Run root:
    `custom_cfd_mesher_experiment/runs/raw_direct_faired_cap_snappy_ranking_v0_1_three_variant_20260625`.
  - Summary:
    `custom_cfd_mesher_experiment/runs/raw_direct_faired_cap_snappy_ranking_v0_1_three_variant_20260625/alpha_sweep_summary_end60_np8_v2.json`.
  - Angles: `-2, 0, +4, +8, +10 deg`.
  - Cases: `3` variants x `5` alphas = `15` completed RANS cases.
  - Runtime: `4373.8 s`.
- Raw best L/D from the sweep:
  - `fcv02_short_swept`: best L/D `3.444` at `+10 deg`,
    CL-alpha `0.04716/deg`, Cm-alpha `0.02606/deg`.
  - `fcv03_high_aspect_mild`: best L/D `4.363` at `+10 deg`,
    CL-alpha `0.08330/deg`, Cm-alpha `0.03937/deg`.
  - `fcv04_compact_wide_tail`: best L/D `4.309` at `+10 deg`,
    CL-alpha `0.07849/deg`, Cm-alpha `0.02024/deg`.
- Added polar-aware rough scoring:
  - `software/optimizer/src/aircraft_optimizer/modules/rough_cfd_scoring.py`.
  - `ingest-snappy-hifi-run` and `run-snappy-openfoam-backend` load
    `alpha_sweep_summary_end60_np8_v2.json` when present.
  - The scorer stores best raw L/D but selects the best positive-lift L/D point
    within the configured absolute Cm limit for `score.rough_total`.
- Fresh polar-aware optimizer import:
  - Workspace:
    `aircraft_optimizer_platform/runs/raw_direct_faired_cap_snappy_alpha_sweep_scoring_v0_1_import`.
  - Best candidate:
    `fcv04_compact_wide_tail`.
  - Score order:
    - `fcv04_compact_wide_tail`: `72.250`, scoring alpha `+4 deg`, selected
      L/D `3.263`.
    - `fcv02_short_swept`: `69.279`, scoring alpha `0 deg`, selected L/D
      `0.625`.
    - `fcv03_high_aspect_mild`: `64.195`, scoring alpha `0 deg`, selected L/D
      `0.799`.
- Interpretation:
  - High-alpha L/D improved for all three candidates, but the +8/+10 cases
    carry high positive Cm on these variants. The stable-efficient score should
    not blindly choose the best raw L/D when trim/moment limits are violated.
  - The current sweep is too expensive for every early optimizer candidate
    (`~73 min` for three variants without remeshing). Use it as a stronger
    rough-scoring step for plausible candidates, and consider a cheaper first
    pass such as `0/+4 deg` before expanding to the full sweep.
- Validation:
  - `python -m py_compile` on touched mesher scripts and optimizer files: pass.
  - Optimizer tests: `58 passed`.

### 2026-06-26 - Added two-stage alpha rough-scoring workflow

- Added staged alpha sweep controls:
  - First pass: `0, +4 deg`.
  - Survivor expanded pass: `-2, 0, +4, +8, +10 deg`.
- Updated alpha runner:
  - `custom_cfd_mesher_experiment/scripts/run_no_slip_alpha_sweep.py`.
  - New `--stage-name` and `--stage-role` metadata.
  - New `--reuse-existing` mode to summarize already-run per-alpha cases
    without launching OpenFOAM.
- Generated first-pass summary from existing cases:
  - `custom_cfd_mesher_experiment/runs/raw_direct_faired_cap_snappy_ranking_v0_1_three_variant_20260625/alpha_sweep_summary_first_pass_end60_np8.json`.
  - Runtime to summarize existing cases: `2.3 s`.
  - Completed cases: `3` variants x `2` alphas = `6`.
- Generated stage-labeled survivor summary from existing cases:
  - `custom_cfd_mesher_experiment/runs/raw_direct_faired_cap_snappy_ranking_v0_1_three_variant_20260625/alpha_sweep_summary_survivor_end60_np8.json`.
  - Runtime to summarize existing cases: `5.6 s`.
  - Completed cases: `3` variants x `5` alphas = `15`.
- Added optimizer import control:
  - `ingest-snappy-hifi-run --alpha-sweep-summary-name <filename>`.
  - `run-snappy-openfoam-backend --alpha-sweep-summary-name <filename>`.
  - Ranking preset default now points to
    `alpha_sweep_summary_first_pass_end60_np8.json`.
  - Expanded survivor imports can pass
    `--alpha-sweep-summary-name alpha_sweep_summary_survivor_end60_np8.json`.
- Updated `run-snappy-openfoam-backend` execution path:
  - Non-`--skip-execution` runs now launch the configured alpha stage after
    meshing and the baseline no-slip solver.
  - Default execution stage is the cheap first pass.
- Added stage metadata to rough-scoring output:
  - `alpha_sweep_stage_name`.
  - `alpha_sweep_stage_role`.
- First-pass import:
  - Workspace:
    `aircraft_optimizer_platform/runs/raw_direct_faired_cap_snappy_alpha_first_pass_scoring_v0_1_import`.
  - Best candidate:
    `fcv04_compact_wide_tail`.
  - Score order:
    - `fcv04_compact_wide_tail`: `72.250`, scoring alpha `+4 deg`.
    - `fcv02_short_swept`: `69.279`, scoring alpha `0 deg`.
    - `fcv03_high_aspect_mild`: `64.195`, scoring alpha `0 deg`.
- Survivor import:
  - Workspace:
    `aircraft_optimizer_platform/runs/raw_direct_faired_cap_snappy_alpha_survivor_scoring_v0_1_import`.
  - Best candidate remained:
    `fcv04_compact_wide_tail`.
- Interpretation:
  - On this three-variant set, the cheap `0/+4` pass produces the same selected
    scoring alpha and rank order as the expanded sweep because the higher-alpha
    L/D points violate the configured pitch-moment limit.
  - Keep the first pass as the default optimizer rough-scoring stage and reserve
    the expanded survivor sweep for candidates that are already plausible.

### 2026-06-26 - Relaxed two-point first-pass rejection behavior

- Policy update:
  - Two-point first-pass CFD is a broad screen, not a strict aerodynamic
    decision.
  - For `stage_role=candidate_screening` with two completed alpha points, the
    rough scorer now selects the best positive-lift L/D point even if that point
    exceeds the normal absolute Cm limit.
  - High Cm is recorded as `exceeds_abs_cm_limit` on the selected polar point
    and remains a detractor/soft-review item, but it does not reject the
    candidate from first-pass optimizer use.
  - Expanded survivor scoring remains stricter about pitch-moment limits.
- Added queryable metric:
  - `score.rough_alpha_screening_tolerant`.
- Fresh tolerant first-pass import:
  - Workspace:
    `aircraft_optimizer_platform/runs/raw_direct_faired_cap_snappy_alpha_first_pass_tolerant_v0_3_import`.
  - Best candidate:
    `fcv04_compact_wide_tail`.
  - Score order:
    - `fcv04_compact_wide_tail`: `72.250`, scoring alpha `+4 deg`.
    - `fcv03_high_aspect_mild`: `65.510`, scoring alpha `+4 deg`.
    - `fcv02_short_swept`: `62.369`, scoring alpha `+4 deg`.
  - All three remained usable for optimizer first-pass ranking.

### 2026-06-26 - Added adaptive CFD promotion policy

- Added optimizer policy module:
  - `software/optimizer/src/aircraft_optimizer/optimizers/adaptive_cfd_promotion.py`.
- Policy behavior:
  - No fixed survivor count.
  - First `5` expanded sweeps establish a campaign-local baseline envelope.
  - After baseline exists, first-pass candidates are compared against the
    expanded baseline at `+4 deg`.
  - Reject only clear failures:
    - missing force coefficients,
    - non-positive drag,
    - absurd coefficients,
    - no positive lift at sampled alphas,
    - or `>=65%` less `CL` at `+4 deg` than the best baseline candidate.
  - Otherwise promote the candidate to the expanded survivor sweep because the
    two-point first pass is intentionally incomplete.
- Added tests for:
  - baseline-building phase,
  - the `65%` lift-loss rejection example,
  - uncertain-but-not-clearly-bad first-pass promotion,
  - hard failure rejection.
- Validation:
  - `python -m py_compile` on touched optimizer files: pass.
  - JSON config validation: pass.
  - Optimizer tests: `64 passed`.

### 2026-06-26 - Wired adaptive alpha promotion into backend execution

- Updated `run-snappy-openfoam-backend`:
  - Default `--alpha-strategy adaptive`.
  - `--alpha-strategy first-pass` forces the cheap `0/+4 deg` stage.
  - `--alpha-strategy survivor-expanded` forces the full `-2/0/+4/+8/+10 deg`
    stage.
  - `--alpha-strategy none` skips launching alpha sweeps.
  - `--adaptive-baseline-summary <path>` can be repeated to provide prior
    expanded/adaptive summaries.
- Adaptive execution behavior:
  - If fewer than `min_expanded_baseline_candidates` baseline sweeps are
    provided, run the survivor-expanded sweep for the current candidate/run to
    keep building the baseline.
  - Once baseline exists, run first pass for the current candidate/run.
  - Evaluate each first-pass report through
    `decide_adaptive_cfd_promotion`.
  - Run survivor-expanded sweep only for promoted variants.
  - Write a combined import summary:
    `alpha_sweep_summary_adaptive_end<time>_np<procs>.json`.
  - The combined summary preserves per-variant `adaptive_promotion_decision`
    and whether the imported evidence came from `first_pass` or
    `survivor_expanded`.
- Added tests for:
  - adaptive config-to-policy loading,
  - adaptive summary name generation,
  - combined adaptive summary using survivor reports for promoted candidates and
    first-pass reports for rejected candidates.
- Validation:
  - `python -m py_compile` on touched CLI/optimizer/test files: pass.
  - JSON config validation: pass.
  - Optimizer tests: `66 passed`.

### 2026-06-26 - Added adaptive promotion bar report

- Added CLI report:
  - `report-adaptive-cfd-promotion`.
  - Inputs:
    - `--preset`.
    - repeated `--baseline-summary`.
    - optional `--candidate-alpha-summary`.
    - optional `--variant-id`.
  - Outputs:
    - baseline count,
    - baseline readiness,
    - best baseline `CL` at compare alpha,
    - exact first-pass `CL` reject threshold,
    - hard reject reasons,
    - candidate adaptive decision when candidate data is supplied.
- Wired backend logging:
  - Adaptive backend runs now print `adaptive_cfd_promotion_baseline` before
    branching.
  - Per-candidate adaptive decisions print
    `adaptive_cfd_promotion_decision`.
- Smoke report on existing raw direct-STL summaries:
  - Baseline summary:
    `custom_cfd_mesher_experiment/runs/raw_direct_faired_cap_snappy_ranking_v0_1_three_variant_20260625/alpha_sweep_summary_survivor_end60_np8.json`.
  - Current expanded baseline count: `3`.
  - Baseline ready: `false` because policy requires `5`.
  - Provisional best baseline `CL` at `+4 deg`: `0.428726964`.
  - Provisional reject threshold at `+4 deg`: `0.1500544374`.
  - Example `fcv02_short_swept` first-pass decision:
    `run_expanded_baseline_sweep`, because baseline is not established.
- Validation:
  - `python -m py_compile` on touched files: pass.
  - Optimizer tests: `69 passed`.

### 2026-06-26 - Established five-candidate rough-CFD promotion baseline

- Completed the two missing raw direct-STL baseline variants with the current
  snappy/OpenFOAM rough-scoring preset:
  - `fcv01_long_glider`:
    `custom_cfd_mesher_experiment/runs/raw_direct_faired_cap_snappy_baseline_fcv01_v0_1_20260626`.
  - `fcv05_aft_wing_fast`:
    `custom_cfd_mesher_experiment/runs/raw_direct_faired_cap_snappy_baseline_fcv05_v0_1_20260626`.
- Both runs used the exporter raw STL inputs from
  `dual_contouring/direct_sparse_sdf_mc_experiment/stl`.
- Fixed backend alpha-sweep execution so negative alpha lists are passed as
  `--alphas-deg=<values>`. This avoids argparse treating `-2.0,...` as an
  option.
- Imported completed run evidence into optimizer workspaces:
  - `aircraft_optimizer_platform/runs/raw_direct_faired_cap_snappy_baseline_fcv01_v0_1_import`.
  - `aircraft_optimizer_platform/runs/raw_direct_faired_cap_snappy_baseline_fcv05_v0_1_import`.
- Current adaptive promotion bar using the original three-variant survivor
  summary plus these two new survivor summaries:
  - Expanded baseline count: `5`.
  - Baseline ready: `true`.
  - Best baseline `CL` at `+4 deg`: `0.428726964`.
  - Median baseline `CL` at `+4 deg`: `0.397947692`.
  - First-pass clear-reject threshold: `CL(+4 deg) <= 0.1500544374`.
- Interpretation:
  - The backend is ready for a controlled shakedown run using adaptive rough
    CFD promotion.
  - This is not yet a real unattended optimizer campaign because candidate
    generation bounds and campaign stop rules still need to be locked down.
  - y-plus remains low for wall functions, so results remain rough relative
    ranking evidence, not final scoring CFD.
- Added controlled shakedown boundary config:
  - `software/optimizer/configs/rough_cfd_15_candidate_shakedown.v0_1.json`.
  - Intended structure: five existing expanded baselines plus ten new adaptive
    candidates.
  - Defines geometry constraints, mesh gates, solver gates, runtime review
    limits, and human-review-only cases.
- Validation:
  - `python -m pytest -q`: `69 passed`.

### 2026-06-26 - Built 10 wing-focused shakedown candidates

- Added new candidate config:
  - `software/optimizer/configs/cfd_wing_shakedown_10_variants.v0_1.json`.
- Added matching raw STL map:
  - `software/optimizer/configs/raw_direct_wing_shakedown_10_stl_batch.v0_1.json`.
- Added generated export manifest:
  - `software/optimizer/configs/wing_shakedown_10_export_manifest.v0_1.json`.
- Candidate scope:
  - Same faired-cap fixed-wing UAV family.
  - No topology changes.
  - Fuselage and inlet family unchanged.
  - Primary variation across wing span, sweep, root chord, tip chord, and
    wing leading-edge position.
  - Minor tail-size compensation on a few more aggressive wing placements.
- Exported all 10 raw direct STLs with `direct_sparse_oml_fast`.
- `shv02_broad_root_mild_sweep` initially produced a tiny topology defect
  under the relaxed export gate. It was adjusted from
  `root=205/tip=82/sweep=10` to `root=195/tip=88/sweep=12` and re-exported
  with strict gates.
- Final source-STL validation:
  - All 10 shakedown STLs are watertight.
  - All 10 have `0` boundary edges.
  - All 10 have `0` nonmanifold edges.
  - All 10 have `1` connected component.
- Backend integration:
  - `run-snappy-openfoam-backend` now supports `--input-stl-map` for batch
    raw-STL runs.
  - It also supports `--variant-config` so imports preserve shakedown wing
    parameter metadata.
- Validation:
  - Candidate/export JSON files parse.
  - `python -m py_compile src\aircraft_optimizer\cli\main.py`: pass.
  - `python -m pytest -q`: `69 passed`.

### 2026-06-26 - Ran 10-candidate rough-CFD shakedown

- Run root:
  - `custom_cfd_mesher_experiment/runs/wing_shakedown_10_snappy_adaptive_v0_1_20260626_retry2`.
- Optimizer import workspace:
  - `aircraft_optimizer_platform/runs/wing_shakedown_10_snappy_adaptive_v0_1_retry2_import`.
- Fixed two backend plumbing issues found during launch:
  - `run_snappy_layer_comparison.py` now resolves repo-relative STL map paths
    against the project root.
  - `run-snappy-openfoam-backend` now rejects simultaneous `--input-stl` and
    `--input-stl-map` at the CLI boundary.
- Mesh and smoke-test result:
  - All 10 candidates completed the Snappy/OpenFOAM layered mesh pipeline.
  - All 10 passed strict OpenFOAM mesh checks.
  - All 10 completed `potentialFoam`.
  - Snappy mesh runtimes ranged from about `279.6 s` to `561.5 s`.
  - Final mesh sizes were about `768k` to `880k` cells.
- Rough no-slip solver result:
  - All 10 completed the 60-step laminar-start/kOmegaSST no-slip ladder.
  - y-plus remains below wall-function target, so results remain rough
    comparative evidence, not final scoring CFD.
- Alpha strategy adjustment:
  - The adaptive backend attempted to run an expanded alpha sweep because the
    supplied previous baseline count was below the configured threshold.
  - That expanded sweep was stopped to avoid a 50-case solver run.
  - A clean first-pass alpha sweep was then run directly on the existing meshes:
    `0 deg` and `+4 deg` for all 10 candidates.
- Rough-scoring import:
  - Imported all 10 candidates into the optimizer database with rough scoring
    enabled and final scoring disabled.
  - Best current rough-scoring candidate:
    `shv09_long_span_swept`.
  - Ranking CSV:
    `custom_cfd_mesher_experiment/runs/wing_shakedown_10_snappy_adaptive_v0_1_20260626_retry2/rough_scoring_ranked_summary.csv`.
- Current top five by rough score:
  - `shv09_long_span_swept`: score `88.03`, best L/D `3.164`,
    `CL=0.387`, `CD=0.122`, `CM=-0.012` at `+4 deg`.
  - `shv05_aft_wing_moderate_sweep`: score `86.36`, best L/D `2.883`,
    `CL=0.335`, `CD=0.116`, `CM=-0.012` at `+4 deg`.
  - `shv03_compact_high_sweep`: score `83.39`, best L/D `2.420`,
    `CL=0.261`, `CD=0.108`, `CM=0.005` at `+4 deg`.
  - `shv06_wide_low_taper`: score `68.00`, best L/D `3.517`,
    penalized mainly by high positive pitch moment.
  - `shv02_broad_root_mild_sweep`: score `67.79`, best L/D `3.407`,
    penalized mainly by high positive pitch moment.
- Lessons:
  - The mesh path is robust across this bounded 10-candidate wing sweep.
  - Runtime is now solver-dominated after meshing.
  - The adaptive policy needs a cheaper default for early optimizer campaigns:
    first-pass alpha should run by default once any usable baseline exists, and
    expanded sweeps should be reserved for promoted candidates.

### 2026-06-26 - Changed backend alpha default to first-pass

- Updated `run-snappy-openfoam-backend` so `--alpha-strategy` defaults to
  `first-pass`, not `adaptive`.
- Rationale:
  - The 10-candidate shakedown proved that full expanded alpha sweeps can turn
    a useful optimizer batch into a multi-hour solver campaign.
  - Normal candidate batches should run the cheaper `0/+4 deg` first-pass
    rough-scoring stage by default.
  - `--alpha-strategy adaptive` remains available explicitly for deliberate
    baseline-building or promoted survivor expansion.
- Updated docs/preset text:
  - `software/optimizer/README.md`.
  - `software/optimizer/configs/snappy_openfoam_external_aero_ranking.v0_1.json`.
- Operational rule:
  - Use first-pass rough scoring for broad candidate exploration.
  - Use expanded survivor sweeps only on promoted candidates or when explicitly
    building/refreshing the baseline envelope.

### 2026-06-26 - Ran first bounded optimizer pilot loop

- Goal:
  - Start from the 10-candidate wing shakedown rough-CFD results.
  - Generate `6` first-generation child variants from the best current
    candidates.
  - Run rough-CFD first-pass scoring.
  - Generate `3` second-generation child variants from gen-1 results.
  - Run rough-CFD first-pass scoring and stop for review.
- New candidate configs:
  - `software/optimizer/configs/cfd_pilot_loop_gen1_6_variants.v0_1.json`.
  - `software/optimizer/configs/cfd_pilot_loop_gen2_3_variants.v0_1.json`.
- New STL maps:
  - `software/optimizer/configs/raw_direct_pilot_loop_gen1_6_stl_batch.v0_1.json`.
  - `software/optimizer/configs/raw_direct_pilot_loop_gen2_3_stl_batch.v0_1.json`.
- Export result:
  - Gen 1: all `6` STLs exported and diagnosed successfully.
  - Gen 2: all `3` STLs exported and diagnosed successfully.
  - All exported STLs were watertight, single-component, with zero boundary
    edges and zero nonmanifold edges according to the faired-cap diagnostics.
- CFD result:
  - Gen-1 run root:
    `custom_cfd_mesher_experiment/runs/pilot_loop_gen1_6_snappy_firstpass_20260626`.
  - Gen-1 optimizer import:
    `aircraft_optimizer_platform/runs/pilot_loop_gen1_6_snappy_firstpass_import`.
  - Gen-2 run root:
    `custom_cfd_mesher_experiment/runs/pilot_loop_gen2_3_snappy_firstpass_20260626`.
  - Gen-2 optimizer import:
    `aircraft_optimizer_platform/runs/pilot_loop_gen2_3_snappy_firstpass_import`.
  - All `9` pilot-loop candidates completed rough scoring.
  - All `9` completed the no-slip first-pass alpha sweep at `0 deg` and
    `+4 deg`.
  - Some imported OpenFOAM smoke-validation modules recorded recoverable
    `max_non_orthogonality` failures, but the rough no-slip solver and scoring
    modules completed.
- Best result:
  - Previous best from 10-candidate shakedown:
    `shv09_long_span_swept`, rough score `88.030`, best L/D `3.164`.
  - Gen-1 winner:
    `opg1_06_shv06_trimmed_high_ld`, rough score `88.040`, best L/D `3.575`.
  - Gen-2 winner:
    `opg2_01_opg1_06_span_tip_tune`, rough score `90.094`, best L/D `3.647`,
    `CL=0.464`, `CD=0.127`, `CM=0.004` at `+4 deg`.
- Summary artifacts:
  - `runs/pilot_loop_20260626_summary/README.md`.
  - `runs/pilot_loop_20260626_summary/pilot_loop_rough_scoring_ranked_summary.csv`.
  - `runs/pilot_loop_20260626_summary/pilot_loop_20260626_summary.json`.
- Interpretation:
  - The first bounded loop did produce useful optimizer behavior: it rescued the
    high-L/D but pitch-penalized shv06 family, then improved it in gen 2.
  - The next implementation step should make candidate generation code-driven
    instead of manually captured in JSON configs.
  - Final scoring remains blocked; these are still rough relative CFD results.

### 2026-06-27 - Added native optimizer dashboard with SDF viewer launch

- Added a native Rust dashboard app:
  - `src/bin/optimizer_dashboard.rs`.
  - Build command: `cargo build --release --bin optimizer_dashboard`.
  - Run command:
    `target/release/optimizer_dashboard.exe --workspace aircraft_optimizer_platform/runs/pilot_loop_gen2_3_snappy_firstpass_import`.
  - Uses the existing fast standalone SDF viewer for candidate inspection.
- Replaced the temporary Python web dashboard direction:
  - Removed `software/optimizer/src/aircraft_optimizer/ui/dashboard.py`.
  - Removed the `serve-dashboard` CLI command.
  - The optimizer package remains the headless orchestration layer.
- Dashboard capabilities:
  - Reads an optimizer workspace `optimizer.db` directly.
  - Lists candidates, inferred generations, rough scores, L/D, CL, CD, CM,
    failure counts, module attempts, mesh/y-plus metrics, design variables,
    and candidate lineage.
  - Adds tabs for overview, selected-candidate detail, scoring plots, alpha
    sweep curves, operational history/lineage, and dashboard controls.
  - Shows scatter plots for score vs L/D, drag vs lift, score vs moment, and
    score vs mesh quality.
  - Overview/scoring plots now include grid lines, tick labels, generation
    legend, marker outlines, and generation-specific point shapes.
  - Shows selected-candidate CL/CD/L-D/CM alpha curves from
    `rough_cfd_scoring` metadata when available.
  - Candidate detail now shows the selected candidate's generated L/D, CL, CD,
    and CM alpha plots directly, not only numeric summaries.
  - Shows score components and detractors from the rough-scoring metadata.
  - Shows proposed next candidates from `optimizer_iterations` when the
    optimizer has generated unevaluated iteration records.
  - Shows operational history plus a lineage map using `candidate_lineage`
    parent links.
  - Auto-refreshes every `10 s`.
  - Resolves viewer targets for imported CFD candidates by matching the
    candidate variant id to optimizer export-result JSON files under
    `dual_contouring/direct_sparse_sdf_mc_experiment/logs`.
  - Candidate viewer targets now prefer generated Rhai/SDF sources in
    `dual_contouring/direct_sparse_sdf_mc_experiment/scratch` over export-result
    JSON/STL artifacts.
  - The Candidate tab launches `target/release/sdf_viewer.exe` with the selected
    Rhai/SDF target instead of trying to render the viewer inside the dashboard.
  - Added a preview-only fallback Rhai helper for `faired_inlet_cap` so current
    faired-cap optimizer scripts can load in the viewer even though that helper
    is exporter-side.
  - Direct live-SDF benchmark on
    `opg2_01_opg1_06_span_tip_tune` completed in about `319 ms`, with no STL
    artifact and `mesh_ms=0`.
- Viewer integration:
  - Embedded WGPU rendering in the dashboard was disabled after it proved much
    slower and unstable compared with the original bare viewer app.
  - Current best gen-2 target resolves directly to the generated Rhai/SDF source
    so the bare viewer does not need to open the export-result JSON first.
  - Added process-isolated viewer control: the dashboard writes the selected
    Rhai/SDF path to `<workspace>/dashboard_sdf_viewer_target.txt` and launches
    `sdf_viewer.exe --watch-target <file>`.
  - The viewer polls that target file and reloads when the dashboard sends a new
    candidate, giving dashboard-driven selection without embedding WGPU inside
    the dashboard.
  - Added `Sync Viewer` mode so selecting a candidate in the dashboard updates
    the linked viewer session automatically.
  - Renamed the old `Tree` tab to `History` and made it operational: it now
    summarizes ran/running/pending/failed candidates, shows a run-history table
    with parents, score, aero metrics, mesh/y-plus status, and keeps the lineage
    graph as a secondary map.
- Validation:
  - `cargo check --bin optimizer_dashboard` passed.
- Control model:
  - Dashboard controls write request JSON files under
    `<workspace>/dashboard_control_requests`.
  - Supported request actions: start, pause, stop, generate-next, promote
    selected candidate, rerun rough scoring, and rerun final scoring.
  - A future runner/controller still needs to consume these requests and append
    normal optimizer records. The dashboard does not pretend to be a resident
    optimizer daemon.

### 2026-06-27 - Added first-class runner state records

- Added `candidate_runner_states` to the optimizer SQLite schema:
  - One current-state row per candidate.
  - Stores `state`, `stage`, `active_module`, `progress`, priority, timestamps,
    reason/message, and metadata.
  - Intended states include `queued`, `running`, `meshing`, `cfd_running`,
    `scored`, `promoted`, `rejected`, and `failed`.
- Added `upsert_candidate_runner_state(...)` repository helper.
- Wired runner-state writes into:
  - v0.1 fixture campaign.
  - Generic fixture candidate evaluator.
  - Wing-options optimizer study.
  - Sequential gated optimizer study, including pre-export gate states.
  - Real no-inlet export validation path.
- Updated database summary and campaign/optimizer reports to include runner
  states.
- Updated the native dashboard to prefer `candidate_runner_states` for History
  and candidate-list state display, with fallback inference for older databases.
- Validation:
  - `python -m pytest aircraft_optimizer_platform/software/optimizer/tests/test_v0_1_flow.py -q`
    passed: `69 passed`.
  - `cargo check --bin optimizer_dashboard` passed.
  - `cargo build --release --bin optimizer_dashboard --bin sdf_viewer` passed
    after stopping the running dashboard process that had locked the executable.

### 2026-06-27 - Expanded optimizer stress-test variables and ran 15 candidates

- Expanded the active fixed-wing v0.1 variable schema while preserving fixed
  topology:
  - original required wing variables: span, root chord, tip chord, sweep.
  - optional bounded `wing.airfoil_selector`, rounded to one of three airfoil
    families: `thin_loiter`, `balanced_uav`, `high_lift_stable`.
  - optional `fuselage.length_delta_mm`, from `-100` to `+100 mm`.
  - optional normalized `fuselage.nose_bluntness`.
  - optional normalized `fuselage.tail_bluntness`.
- Updated `HaltonAskTellPolicy` to support the expanded schema dimensionality.
- Updated normalized schema validation so optional variables stay optional for
  legacy candidates, while required variables and present variables still need
  normalized values.
- Updated low-fidelity screening to include:
  - airfoil selector effects on cruise `CL` proxy and `CD0` proxy.
  - fuselage length drag/layout proxy.
  - nose bluntness drag proxy.
  - tail bluntness drag/manufacturing/layout proxy.
  - explicit `score.static_layout_proxy` and
    `score.manufacturability_proxy` metrics.
- Ran the expanded sequential gated stress test:

```text
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main run-sequential-gated --workspace ..\..\runs\stress_15_airfoil_fuselage_20260627 --iterations 15
```

- Stress-run result:
  - `15` candidates registered.
  - `12` candidates evaluated.
  - `3` candidates failed pre-export screening.
  - failures were sensible gate failures:
    `min_taper_ratio` plus `min_tip_chord_mm`, `min_aspect_ratio`, and
    `max_taper_ratio`.
  - promoted candidate:
    `candidate_2561163970fd4babb73ba495c3c63d97`.
  - promoted score:
    `score.low_fidelity_total = 9.095638`.
  - workspace:
    `aircraft_optimizer_platform\runs\stress_15_airfoil_fuselage_20260627`.
- Validation:
  - `python -m pytest tests\test_v0_1_flow.py -q` passed:
    `69 passed`.
- Interpretation:
  - The current headless optimizer can now stress the candidate lifecycle across
    a more realistic fixed-topology design space.
  - The airfoil selector is a numeric fixed-choice placeholder, not a topology
    change and not final aerodynamic truth.
  - The next useful step is to wire these extra knobs into real geometry/export
    candidate generation so the dashboard preview and CFD backend evaluate the
    actual changed fuselage/nose/tail/airfoil geometry, not only screening
    proxies.

### 2026-06-27 - Wired expanded variables into real no-inlet export-only path

- Added generated real-export no-inlet Rhai geometry:
  - `software\optimizer\src\aircraft_optimizer\geometry\real_no_inlet_generator.py`.
  - It replaces fixed-topology top-level Rhai bindings for wing span, root
    chord, tip chord, sweep, fuselage length, nose bluntness, tail bluntness,
    and main-wing airfoil.
  - Airfoil selector mapping:
    - `0`: `thin_loiter`, NACA `2409`.
    - `1`: `balanced_uav`, NACA `2412`.
    - `2`: `high_lift_stable`, NACA `4412`.
  - Generated geometry writes a trace JSON beside the Rhai source.
- Added export-request support for candidate-specific generated Rhai paths,
  candidate-specific result JSON folders, and unique STL output stems.
- Added CLI command:

```text
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main run-real-no-inlet-export-batch --workspace ..\..\runs\real_export_expanded_vars_abs_20260627 --iterations 3 --timeout-seconds 1200
```

- First attempt failed because generated Rhai paths were relative to the
  optimizer folder while the exporter runs from `dual_contouring`. Fixed by
  passing absolute generated Rhai paths into the exporter request.
- Fresh export-only batch result:
  - workspace:
    `aircraft_optimizer_platform\runs\real_export_expanded_vars_abs_20260627`.
  - `3` candidates registered and evaluated.
  - `3/3` real direct sparse OML exports succeeded.
  - `3/3` platform `check-cfd-readiness` checks passed.
  - `0` CFD or mesh runs were started.
  - Exported topology/readiness checks:
    - boundary edges: `0` for all three.
    - nonmanifold edges: `0` for all three.
    - connected components: `1` for all three.
    - duplicate triangles: `0` for all three.
    - long chord sections `>=75 mm`: `0` for all three.
  - Triangle counts:
    - candidate 0: `1,187,232`.
    - candidate 1: `1,052,100`.
    - candidate 2: `1,048,368`.
  - Export runtimes:
    - candidate 0: `313.65 s`.
    - candidate 1: `277.58 s`.
    - candidate 2: `259.67 s`.
  - Aircraft-only ISO screenshots:
    `aircraft_optimizer_platform\runs\real_export_expanded_vars_abs_20260627\export_screenshots`.
- Validation:
  - `python -m pytest tests\test_v0_1_flow.py -q` passed:
    `70 passed`.
- Interpretation:
  - The expanded optimizer variables now reach the actual real no-inlet Rhai
    export source and produce CFD-ready STL surface artifacts.
  - Export runtime is currently several minutes per candidate, so the next
    implementation step should avoid redundant full exports while iterating and
    should keep export-only checks separate from mesh/CFD launch gates.

### 2026-06-27 - Mesh-only snappy dry run on expanded real-export STLs

- Added mesh-only support to the snappy/OpenFOAM comparison runner:
  - `custom_cfd_mesher_experiment\scripts\run_snappy_layer_comparison.py`.
  - New `--skip-solver` mode stops after case generation, snappyHexMesh,
    `checkMesh`, aircraft-only screenshot generation, and optional surface
    fidelity audit.
  - Mesh-only reports now use `pass_for_mesh_only` instead of requiring a
    `potentialFoam` result.
- Fixed the lightweight STL preview camera orientation so generated ISO
  screenshots no longer appear vertically flipped:
  - `custom_cfd_mesher_experiment\scripts\render_stl_iso_screenshot.py`.
  - `custom_cfd_mesher_experiment\scripts\render_stl_view_set.py`.
- Created optimizer mesh-input map:
  - `aircraft_optimizer_platform\runs\real_export_expanded_vars_abs_20260627\mesh_input_stl_map.json`.
- Ran a mesh-only batch on the three successful expanded real-export STLs:

```text
python custom_cfd_mesher_experiment\scripts\run_snappy_layer_comparison.py --run-root custom_cfd_mesher_experiment\runs\real_export_expanded_vars_snappy_mesh_only_20260627 --velocity 22.352,0,0 --base-cells 44,28,44 --pad 0.8 --surface-min-level 4 --surface-max-level 5 --feature-level 5 --n-cells-between-levels 4 --snap-tolerance 0.5 --n-smooth-patch 10 --n-surface-layers 1 --layer-relative-sizes true --layer-expansion-ratio 1.2 --final-layer-thickness 0.75 --min-layer-thickness 0.12 --layer-feature-angle 70 --n-layer-iter 50 --n-relaxed-iter 20 --max-global-cells 6000000 --max-local-cells 3000000 --feature-refinement-level 5 --feature-angle-deg 24 --feature-box-count 18 --feature-box-grid 10,10,6 --feature-box-padding-frac 0.03 --parallel-procs 12 --check-skew-threshold 12 --feature-refinement-boxes --include-known-hotspot-boxes --surface-fidelity-audit --skip-solver --input-stl-map aircraft_optimizer_platform\runs\real_export_expanded_vars_abs_20260627\mesh_input_stl_map.json
```

- Result root:
  `custom_cfd_mesher_experiment\runs\real_export_expanded_vars_snappy_mesh_only_20260627`.
- Result summary:
  - `3/3` cases completed mesh generation.
  - `3/3` passed mesh-only verdicts.
  - `0` CFD solver cases were run; `potentialFoam` status is intentionally
    `skipped` for every variant.
  - total batch runtime: `1459.98 s`.
- Mesh metrics:
  - `expvars_00_20e021d4`: `928,611` cells, `1,092,497` points,
    `94,021` aircraft patch faces, max non-orthogonality `69.994281`,
    max skewness `3.9110553`, bidirectional surface deviation p95
    `1.030735 mm`, p99 `2.375032 mm`, max `2.788244 mm`, runtime
    `507.62 s`.
  - `expvars_01_ce18b95c`: `1,015,461` cells, `1,179,460` points,
    `91,209` aircraft patch faces, max non-orthogonality `69.999241`,
    max skewness `3.833238`, bidirectional surface deviation p95
    `0.312998 mm`, p99 `2.043945 mm`, max `2.866143 mm`, runtime
    `415.97 s`.
  - `expvars_02_f0e50b6e`: `883,971` cells, `1,039,497` points,
    `87,290` aircraft patch faces, max non-orthogonality `69.997699`,
    max skewness `5.9692461`, bidirectional surface deviation p95
    `0.903624 mm`, p99 `2.420970 mm`, max `2.907165 mm`, runtime
    `536.39 s`.
- Corrected export preview screenshots:
  `aircraft_optimizer_platform\runs\real_export_expanded_vars_abs_20260627\export_screenshots_corrected`.
- Validation:
  - `python -m py_compile custom_cfd_mesher_experiment\scripts\run_snappy_layer_comparison.py custom_cfd_mesher_experiment\scripts\render_stl_iso_screenshot.py custom_cfd_mesher_experiment\scripts\render_stl_view_set.py`
    passed.
- Interpretation:
  - The expanded-variable real STLs are meshable with the current snappy
    ranking/development path.
  - Runtime is high for optimizer-scale mesh-only work: roughly `7-9 min` per
    candidate before any solver execution.
  - Surface fidelity is mixed: candidate 01 is close to the current final
    scoring p95 target, while candidates 00 and 02 are rougher and should be
    treated as development/ranking mesh evidence only.
  - Next action is to run a bounded solver smoke test on these exact meshes
    only after visual review confirms that leading edge, trailing edge, wing
    root, wingtip, tail, and fuselage capture are acceptable.

### 2026-06-27 - Mesh closeup review and tail-clearance pre-export gate

- Generated feature closeups from the actual OpenFOAM aircraft patch VTKs for
  the three expanded real-export snappy meshes:
  - `custom_cfd_mesher_experiment\runs\real_export_expanded_vars_snappy_mesh_only_20260627\expvars_00_20e021d4\feature_closeups`.
  - `custom_cfd_mesher_experiment\runs\real_export_expanded_vars_snappy_mesh_only_20260627\expvars_01_ce18b95c\feature_closeups`.
  - `custom_cfd_mesher_experiment\runs\real_export_expanded_vars_snappy_mesh_only_20260627\expvars_02_f0e50b6e\feature_closeups`.
- Updated `custom_cfd_mesher_experiment\scripts\render_vtk_feature_closeups.py`
  to infer span and vertical axes from aircraft bounds before applying
  feature crop boxes. This avoids mis-cropping meshes where the exporter output
  uses `x=length`, `z=span`, and `y=vertical/thickness`.
- Visual review finding:
  - The fuselage-side splotches are local snappy refinement islands from the
    automatic feature-refinement-box selector, not intentional geometry
    features.
  - The selector bins high-angle STL feature edges and can pick localized
    source-triangulation/noise clusters on otherwise smooth fuselage sides.
  - This is not by itself a `checkMesh` or solver-plumbing blocker, but it
    wastes cells and makes surface-fidelity review noisier.
  - Do not promote this mesh-only set to solver smoke until the refinement-box
    strategy is either accepted visually or changed to target explicit feature
    families instead of arbitrary top-ranked edge-angle clusters.
- Added a pre-export geometry-constraint gate for the fixed-tail no-inlet
  template:
  - `software\optimizer\src\aircraft_optimizer\modules\pre_export_screening.py`.
  - New policy key:
    `min_tail_fuselage_end_clearance_mm`, default `20.0`.
  - New metrics:
    `pre_export.fuselage_length_mm`,
    `pre_export.tail_aft_x_mm`,
    `pre_export.tail_fuselage_end_clearance_mm`.
  - The gate rejects candidates where the fixed horizontal/vertical tail aft
    extent is too close to or aft of the generated fuselage end after
    `fuselage.length_delta_mm` is applied.
- Added regression coverage:
  - `test_pre_export_screening_rejects_tail_aft_of_shortened_fuselage`.
  - Relaxed the sequential-gated smoke test so it does not assume a fixed
    number of early Halton candidates survive as pre-export gates get stricter.
- Validation:
  - `python -m py_compile custom_cfd_mesher_experiment\scripts\render_vtk_feature_closeups.py aircraft_optimizer_platform\software\optimizer\src\aircraft_optimizer\modules\pre_export_screening.py`
    passed.
  - `PYTHONPATH=src python -m pytest tests\test_v0_1_flow.py -q` passed:
    `71 passed`.
- Next action:
  - Replace or constrain automatic surface-feature refinement boxes so they
    target known feature zones such as leading edge, trailing edge, tips, and
    tail/fuselage blends instead of arbitrary high-angle buckets on smooth
    fuselage panels.

### 2026-06-27 - Feature-zone refinement measures and tip-only mesh candidate

- Updated `custom_cfd_mesher_experiment\scripts\run_snappy_layer_comparison.py`
  so automatic refinement logic:
  - infers the actual span/vertical axes from STL bounds.
  - places corrected tip seed boxes on the span tips rather than the vertical
    thickness axis.
  - labels and records every refinement box in `variant_summary.json`.
  - constrains optional feature-refinement boxes to named feature zones:
    nose, wing root/blend, wing LE, wing TE, wingtips, tail root/blend, and
    tail edges.
- Fixed `custom_cfd_mesher_experiment\scripts\summarize_openfoam_run.py` so
  aircraft and farfield patch face counts are parsed even when `checkMesh`
  reports a patch-topology warning instead of a plain `ok` line.
- Ran constrained feature-zone mesh-only rerun on the three expanded real-export
  STLs:
  - root:
    `custom_cfd_mesher_experiment\runs\real_export_expanded_vars_snappy_mesh_only_feature_zones_20260627`.
  - `3/3` mesh-only runs passed.
  - no solver was run.
  - comparison artifact:
    `custom_cfd_mesher_experiment\runs\real_export_expanded_vars_snappy_mesh_only_feature_zones_20260627\old_vs_new_mesh_measure_comparison.json`.
- Feature-zone result:
  - It did not remove the visible dark/splotchy fuselage patches in screenshots.
  - It improved p95 surface deviation only slightly for candidates 00/01 and
    moderately for candidate 02.
  - It increased cell counts by `5-10%`.
  - It introduced a variant-01 aircraft patch warning:
    `multiply connected (shared edge)` with two conflicting points near
    `x=0.372 m`, `y=0.0007 m`, `z=-0.130 m`.
  - Conclusion: do not adopt feature-zone boxes as the new default.
- Ran a corrected tip-seed-only control:
  - candidate 00 root:
    `custom_cfd_mesher_experiment\runs\real_export_expanded_vars_snappy_mesh_only_tip_only_probe_20260627`.
  - candidates 01/02 root:
    `custom_cfd_mesher_experiment\runs\real_export_expanded_vars_snappy_mesh_only_tip_only_remaining_20260627`.
  - combined measure comparison:
    `custom_cfd_mesher_experiment\runs\real_export_expanded_vars_snappy_tip_only_measure_comparison_20260627.json`.
- Tip-only result versus previous feature-box baseline:
  - candidate 00: cells `928,611 -> 795,532` (`-14.33%`), p95
    `1.030735 -> 0.157581 mm` (`84.71%` improvement), runtime
    `507.62 -> 416.28 s`.
  - candidate 01: cells `1,015,461 -> 771,944` (`-23.98%`), p95
    `0.312998 -> 0.114522 mm` (`63.41%` improvement), runtime
    `415.97 -> 340.01 s`.
  - candidate 02: cells `883,971 -> 754,888` (`-14.60%`), p95
    `0.903624 -> 0.172089 mm` (`80.96%` improvement), runtime
    `536.39 -> 595.21 s`.
  - all three remained `pass_for_mesh_only`.
- Visual interpretation:
  - The dark/splotchy fuselage pattern remains visible even in the tip-only
    meshes, so screenshots alone should not be treated as proof of accidental
    local refinement.
  - The measured surface-fidelity improvement shows the splotches are likely
    rendering/normal/source-triangulation/snap-transition artifacts rather than
    a reliable indicator of bad local mesh sizing.
  - Continue using screenshots for human review, but gate mesh promotion on
    topology, patch counts, surface-fidelity distributions, and solver smoke.
- Added preset candidate:
  - `software\optimizer\configs\snappy_openfoam_external_aero_ranking.v0_2_tip_only_mesh_candidate.json`.
  - It disables automatic feature-refinement boxes and records the tip-only
    evidence roots.
  - It is mesh-only pending solver smoke, not the active rough-CFD scoring
    default.
- Validation:
  - `python -m py_compile custom_cfd_mesher_experiment\scripts\run_snappy_layer_comparison.py`
    passed.
  - `python -m py_compile custom_cfd_mesher_experiment\scripts\summarize_openfoam_run.py`
    passed.
  - `python -m json.tool software\optimizer\configs\snappy_openfoam_external_aero_ranking.v0_2_tip_only_mesh_candidate.json`
    passed.
- Next action:
  - Run `potentialFoam` on the three tip-only meshes, then run the bounded
    no-slip rough-CFD ladder only if solver smoke is stable.

### 2026-06-27 - Tip-only snappy solver smoke and bounded no-slip ladder

- Ran `potentialFoam -writep` on the three tip-only snappy meshes.
  - Summary:
    `custom_cfd_mesher_experiment\runs\real_export_expanded_vars_snappy_tip_only_potential_smoke_20260627\potential_smoke_summary.json`.
  - Result: `3/3` completed, no warnings or fatal errors.
  - Continuity errors were about `1.93e-7` to `2.54e-7`.
  - Interpolated velocity errors were about `2.07e-4` to `2.09e-4`.
  - Last pressure residuals were about `7.50e-9` to `8.80e-9`.
- Ran the bounded 60-step no-slip ladder on the same tip-only meshes.
  - Candidate 00 summary:
    `custom_cfd_mesher_experiment\runs\real_export_expanded_vars_snappy_mesh_only_tip_only_probe_20260627\no_slip_ladder_summary_tip_only_end60_np8.json`.
  - Candidates 01/02 summary:
    `custom_cfd_mesher_experiment\runs\real_export_expanded_vars_snappy_mesh_only_tip_only_remaining_20260627\no_slip_ladder_summary_tip_only_end60_np8.json`.
  - Combined evidence:
    `custom_cfd_mesher_experiment\runs\real_export_expanded_vars_snappy_tip_only_solver_summary_20260627.json`.
- Solver result:
  - Laminar completed `3/3`.
  - Laminar-start/kOmegaSST completed `3/3`.
  - Short kOmegaSST force-tail stability passed `3/3`.
  - Candidate 00 laminar force-tail stability failed, but its kOmegaSST case
    passed.
  - kOmegaSST final coefficients at `0 deg`:
    - `expvars_00_20e021d4`: `Cd=0.08349`, `Cl=-0.09055`, `Cm=0.06993`.
    - `expvars_01_ce18b95c`: `Cd=0.08894`, `Cl=-0.07567`, `Cm=0.05922`.
    - `expvars_02_f0e50b6e`: `Cd=0.08421`, `Cl=-0.07979`, `Cm=0.05876`.
  - kOmegaSST y+ remains below the wall-function target:
    - p50 about `4.31-4.40`.
    - p95 about `8.87-10.19`.
    - max about `40.12-47.36`.
    - `0/3` pass the current wall-function y+ gate.
- Updated preset candidate:
  - `software\optimizer\configs\snappy_openfoam_external_aero_ranking.v0_2_tip_only_mesh_candidate.json`.
  - Status changed from mesh-only pending solver smoke to rough-scoring
    development pending y+ strategy.
- Interpretation:
  - The tip-only preset is now solver-plumbing clean on the three expanded
    real-export STLs and is a credible rough-scoring development candidate.
  - It is not final scoring CFD because y+ and wall treatment are not yet
    validated for trusted drag.
- Next action:
  - Keep the tip-only preset as the current fast/robust snappy candidate, then
    run a bounded y+ strategy study that preserves the current surface
    fidelity: either adjust first-layer/layer-thickness policy for low-y+
    integration or explicitly switch the rough-scoring wall treatment target
    away from wall-function assumptions.

### 2026-06-28 - Real 6 + 3 optimizer pilot with rough CFD

- Added a reusable pilot orchestration script:
  - `software\optimizer\scripts\run_real_optimizer_pilot.py`.
  - It exports generation-1 candidates, writes snappy backend STL maps and
    variant configs, runs the snappy/OpenFOAM backend with first-pass alpha
    rough CFD, ranks the results, generates generation-2 children from scored
    parents, and writes combined summaries.
- Fixed backend preset handling:
  - `software\optimizer\src\aircraft_optimizer\cli\main.py`.
  - `run-snappy-openfoam-backend` now honors
    `mesher.feature_refinement_boxes=false` instead of always passing
    `--feature-refinement-boxes`.
  - The backend now passes explicit force-axis settings from the preset into
    the no-slip ladder and alpha sweep.
- Fixed alpha-sweep axis handling:
  - `custom_cfd_mesher_experiment\scripts\run_no_slip_alpha_sweep.py`.
  - Added `--vertical-axis y|z`.
  - For `--vertical-axis y`, velocity/angle of attack are applied in the
    `X/Y` plane, lift is measured in `+Y`, and pitch is about `+Z`.
- Updated the tip-only preset:
  - `software\optimizer\configs\snappy_openfoam_external_aero_ranking.v0_2_tip_only_mesh_candidate.json`.
  - Recorded the current real no-inlet exporter frame as
    `X=length`, `Y=vertical`, `Z=span`.
  - Set `vertical_axis=y`, `drag_dir=[1,0,0]`, `lift_dir=[0,1,0]`,
    and `pitch_axis=[0,0,1]`.
- Launched the real optimizer pilot:
  - command:
    `python scripts\run_real_optimizer_pilot.py --workspace ..\..\runs\real_optimizer_pilot_6x3_20260628 --run-slug real_optimizer_pilot_6x3_20260628 --initial-count 6 --children-count 3 --timeout-seconds 1200 --alpha-strategy first-pass`.
  - The long wrapper timed out while generation-1 alpha scoring was still
    running, but the underlying OpenFOAM jobs continued and completed.
  - The first gen-1 alpha sweep used the old/wrong `Z` lift convention. It was
    preserved as:
    `custom_cfd_mesher_experiment\runs\real_optimizer_pilot_6x3_20260628_gen1_snappy\alpha_sweep_summary_first_pass_end60_np8_wrong_z_lift.json`.
  - Re-ran generation-1 alpha scoring in place with the corrected `Y` lift
    convention; no re-export or remesh was required.
- Completed generation 1:
  - export workspace:
    `aircraft_optimizer_platform\runs\real_optimizer_pilot_6x3_20260628\exports_gen1`.
  - snappy/OpenFOAM run root:
    `custom_cfd_mesher_experiment\runs\real_optimizer_pilot_6x3_20260628_gen1_snappy`.
  - ranking:
    `aircraft_optimizer_platform\runs\real_optimizer_pilot_6x3_20260628\gen1_ranked_summary.csv`.
  - best gen-1 candidate: `opg1_03_8c359e83`, rough score `69.1398`,
    `L/D=3.0722`, `CL=0.2894`, `CD=0.09421`, `Cm=-0.11598` at `+4 deg`.
- Completed generation 2:
  - export workspace:
    `aircraft_optimizer_platform\runs\real_optimizer_pilot_6x3_20260628\exports_gen2`.
  - snappy/OpenFOAM run root:
    `custom_cfd_mesher_experiment\runs\real_optimizer_pilot_6x3_20260628_gen2_snappy`.
  - ranking:
    `aircraft_optimizer_platform\runs\real_optimizer_pilot_6x3_20260628\gen2_ranked_summary.csv`.
  - lineage:
    - `opg2_00_a13bd8a7` from `opg1_03_8c359e83`.
    - `opg2_01_79dab1c3` from `opg1_05_70dd4a2b`.
    - `opg2_02_c84d9004` from `opg1_00_7ef2ec1e`.
- Combined result:
  - summary:
    `aircraft_optimizer_platform\runs\real_optimizer_pilot_6x3_20260628\real_optimizer_pilot_summary.json`.
  - combined ranking:
    `aircraft_optimizer_platform\runs\real_optimizer_pilot_6x3_20260628\combined_ranked_summary.csv`.
  - Regenerated the generation and combined ranking CSVs after fixing the
    pilot summary parser so they include mesh cells, points, aircraft faces,
    max non-orthogonality, max skewness, and surface p95/p99.
  - best overall: `opg2_01_79dab1c3`, rough score `69.3423`,
    `L/D=2.4982`, `CL=0.2389`, `CD=0.09564`, `Cm=-0.07237` at `+4 deg`.
  - This was a small improvement over the best gen-1 rough score and had a
    better pitch-moment penalty than the highest-L/D gen-1 candidate.
- Validation:
  - `python -m py_compile custom_cfd_mesher_experiment\scripts\run_no_slip_alpha_sweep.py aircraft_optimizer_platform\software\optimizer\src\aircraft_optimizer\cli\main.py aircraft_optimizer_platform\software\optimizer\scripts\run_real_optimizer_pilot.py`
    passed.
  - `PYTHONPATH=src python -m pytest tests\test_v0_1_flow.py -q` passed:
    `71 passed`.
  - Final process check found no active Python/OpenFOAM/snappy jobs from the
    pilot.
- Interpretation:
  - The first real optimizer loop now exists end-to-end: generate, export,
    mesh, rough-CFD score, rank, mutate, rerun, and summarize.
  - The run should still be treated as rough relative scoring only. Final CFD
    scoring remains blocked by y-plus/wall-treatment and mesh-convergence
    validation.
- Next action:
  - Add a first-class resumable CLI command for this pilot flow so a long
    OpenFOAM run can restart cleanly from completed export/mesh/alpha artifacts
    after a timeout or workstation interruption.

### 2026-06-28 - Resumable real optimizer pilot CLI

- Added first-class optimizer CLI support for the pilot loop:
  - `software\optimizer\src\aircraft_optimizer\cli\main.py`.
  - New command:
    `python -m aircraft_optimizer.cli.main run-real-optimizer-pilot --workspace ..\..\runs\<run_name> --run-slug <run_name> --initial-count 6 --children-count 3 --timeout-seconds 1200 --alpha-strategy first-pass --resume`.
- Hardened resume behavior:
  - `software\optimizer\scripts\run_real_optimizer_pilot.py`.
  - Completed export maps and variant configs are reused.
  - Existing backend run roots require `comparison_summary.json`; otherwise the run stops instead of guessing through partial mesh artifacts.
  - Missing first-pass alpha summaries can be recovered in place for the first-pass strategy.
  - Completed scoring imports are not duplicated when the scoring workspace database already exists.
- Verified the new CLI command against the completed pilot workspace:
  - `aircraft_optimizer_platform\runs\real_optimizer_pilot_6x3_20260628`.
  - The resume pass returned the same best candidate, `opg2_01_79dab1c3`, without restarting expensive mesh/CFD stages.
- Validation:
  - `python -m py_compile aircraft_optimizer_platform\software\optimizer\scripts\run_real_optimizer_pilot.py aircraft_optimizer_platform\software\optimizer\src\aircraft_optimizer\cli\main.py custom_cfd_mesher_experiment\scripts\run_no_slip_alpha_sweep.py` passed.
  - `PYTHONPATH=src python -m pytest -q` from `software\optimizer` passed: `71 passed`.
- Interpretation:
  - The real optimizer loop is now usable as a repeatable development command:
    generate, export, mesh, rough-CFD score, rank, mutate, rerun, summarize,
    and resume after interruption.
  - This is still rough relative scoring. Final engineering use remains blocked
    by calibration, finalist confirmation workflow, y-plus/wall-treatment
    validation, and mesh-convergence evidence.
- Next action:
  - Run a larger `15-30` candidate real shakedown using the CLI, then use the
    results to tune variable bounds, promotion thresholds, and finalist
    confirmation policy.

### 2026-06-28 - Mobile read-only run monitor

- Added a bare-bones mobile web monitor:
  - `software\dashboard\mobile_monitor.py`.
  - Stdlib-only Python HTTP server; no npm, Flask, or FastAPI dependency.
  - Read-only: it does not write to optimizer databases, run artifacts, or
    control-request folders.
- The monitor reads:
  - export counts from the optimizer workspace,
  - snappy/OpenFOAM variant folders and per-variant partial summaries,
  - `comparison_summary.json`,
  - `alpha_sweep_summary_first_pass_end60_np8.json`,
  - `gen1_ranked_summary.csv` / `combined_ranked_summary.csv`,
  - the background run log tail.
- Updated the monitor so mesh/potential progress appears as each variant
  writes `variant_summary.json`; it no longer waits for the batch-level
  `comparison_summary.json`.
- Updated the monitor to show the no-slip steady solver ladder separately from
  mesh/potential plumbing, including latest steady CL/CD/Cm when available.
- Updated the monitor to detect rough-CFD alpha sweep startup from alpha case
  directories instead of waiting for the final aggregate summary.
- Corrected alpha progress for the current no-slip setup: each requested alpha
  creates a laminar starter case and an SST-started case, so the 14 exported
  candidates produce up to 84 alpha case directories for the current
  `0/4/10 deg` first-pass sweep.
- Added the `ngrok-skip-browser-warning` header to dashboard status polling so
  it works behind ngrok after the browser warning is accepted.
- Started the monitor for the current 15-candidate run on port `8765`.
- Started ngrok for the read-only monitor:
  - `https://tetched-charlie-dextrously.ngrok-free.dev`
- Verified:
  - `/health` returned `ok`.
  - `/api/status` returned the current run phase and counts.
  - ngrok `/api/status` returned JSON when called with the skip-warning header.
- Current run state when last checked:
  - `15` candidates generated/export attempted.
  - `14` passed export; `1` failed the export quality gate with
    `boundary_edges=2` and `nonmanifold_edges=1`.
  - `14/14` backend candidates passed mesh/potential plumbing.
  - The no-slip steady ladder completed for all 14 backend candidates.
  - Rough-CFD alpha sweep had started; final ranking was not available yet.
- Access URLs observed on this machine:
  - local: `http://127.0.0.1:8765/`
  - Wi-Fi LAN: `http://192.168.1.76:8765/`
  - Tailscale: `http://100.85.71.50:8765/`
  - ngrok: `https://tetched-charlie-dextrously.ngrok-free.dev`

### 2026-06-28 - 15-candidate full optimizer shakedown complete

- Run:
  - `runs\real_optimizer_15candidate_full_20260628`.
  - Backend root:
    `..\custom_cfd_mesher_experiment\runs\real_optimizer_15candidate_full_20260628_gen1_snappy`.
- Completion:
  - Optimizer process exited normally.
  - `15` candidates were generated/export attempted.
  - `14` passed export; `1` failed export quality gates with
    `boundary_edges=2` and `nonmanifold_edges=1`.
  - `14/14` backend candidates passed mesh/potential plumbing.
  - `14/14` completed the no-slip laminar starter plus SST/RANS follow-up
    ladder with no fatal errors or floating point exceptions.
  - `14/14` completed first-pass alpha sweep at `0/4/10 deg`; this produced
    `84` OpenFOAM alpha case directories because each alpha uses both laminar
    starter and SST-started cases.
- Runtime:
  - Mesh/potential comparison summary: about `6446 s`.
  - No-slip steady ladder: about `3998 s`.
  - Alpha sweep: about `11695 s`.
  - Practical interpretation: the current full no-slip rough-scoring path is
    usable but too expensive for high-throughput optimizer iteration without a
    faster rough tier or early pruning policy.
- Ranking:
  - Best candidate: `opg1_06_e9db11f1`.
  - Rough score: `82.027`.
  - Best aggregate L/D: `3.844` at `10 deg`.
  - Scoring point used for the rough score: `4 deg`, because the `10 deg`
    point exceeded the configured pitch-moment limit.
  - `4 deg` scoring coefficients: `CL=0.2141`, `CD=0.08439`,
    `Cm=-0.04304`.
  - Geometry variables:
    `span=668.75 mm`, `root_chord=144.44 mm`, `tip_chord=59.2 mm`,
    `sweep=29.29 deg`, `fuselage_length_delta=-7.69 mm`,
    `nose_bluntness=0.353`, `tail_bluntness=0.316`.
- Result interpretation:
  - The optimizer correctly did not simply pick the highest-L/D design.
    Higher-L/D candidates were penalized by larger pitch moment and/or other
    score terms.
  - Every ranked candidate selected `10 deg` as its best aggregate L/D point,
    which suggests the first-pass alpha set should expand above `10 deg` or
    should use a mission/trim target rather than just the best sampled L/D
    point.
  - The current pipeline is end-to-end functional, traceable, and resumable,
    but not yet efficient enough for large optimization campaigns.
- Review artifact:
  - `runs\real_optimizer_15candidate_full_20260628\review\ranking_review.png`.
- Next action:
  - Add a faster candidate-screening CFD tier or early alpha-pruning policy so
    full no-slip alpha sweeps are reserved for promising candidates instead of
    every generated design.

### 2026-06-28 - Conservative early alpha pruning

- Kept the optimizer execution policy as one candidate at a time, with
  OpenFOAM parallelism inside each mesh/solver case.
- Added configurable early alpha pruning to:
  - `custom_cfd_mesher_experiment\scripts\run_no_slip_alpha_sweep.py`.
- Wired preset-driven pruning flags through:
  - `software\optimizer\src\aircraft_optimizer\cli\main.py`.
  - `software\optimizer\scripts\run_real_optimizer_pilot.py`.
- Enabled conservative first-pass pruning in:
  - `software\optimizer\configs\snappy_openfoam_external_aero_ranking.v0_2_tip_only_mesh_candidate.json`.
- Policy:
  - Never prune after `0 deg` alone.
  - Require at least two attempted alpha results.
  - Skip remaining alpha cases only for obvious failures:
    repeated solver failure, coefficient blowup, no meaningful positive lift,
    or very weak lift with very poor L/D.
  - Healthy or uncertain candidates continue through the full first-pass alpha
    set.
- Validation:
  - `python -m py_compile custom_cfd_mesher_experiment\scripts\run_no_slip_alpha_sweep.py aircraft_optimizer_platform\software\optimizer\src\aircraft_optimizer\cli\main.py aircraft_optimizer_platform\software\optimizer\scripts\run_real_optimizer_pilot.py`
    passed.
  - Direct synthetic decision check confirmed no pruning after one alpha,
    pruning after two no-lift alphas, and no pruning for a healthy trend.
  - `PYTHONPATH=src python -m pytest -q` from `software\optimizer` passed:
    `72 passed`.
- Next action:
  - Wire the real optimizer pilot to run pre-export screening as a hard gate
    before export, so physically bad planforms are rejected before SDF export
    and CFD.

### 2026-06-29 - Real optimizer hard pre-export gate

- Wired a real pre-export hard gate into:
  - `software\optimizer\scripts\run_real_optimizer_pilot.py`.
- Behavior:
  - Each generation now writes
    `pre_export_screening_summary.json` before SDF export.
  - Only candidates that pass the hard gate are sent to the real SDF/STL
    exporter.
  - Backend meshing/CFD is skipped cleanly if no candidates survive a
    generation.
  - Candidate execution remains sequential; OpenFOAM parallelism remains inside
    each mesh/solver case.
- Policy:
  - Added `REAL_OPTIMIZER_HARD_PRE_EXPORT_POLICY`, a relaxed policy derived
    from `DEFAULT_SCREENING_POLICY`.
  - This is intentionally permissive. It rejects obviously implausible
    planforms, component-layout failures, or extreme geometry risk, but avoids
    rejecting borderline candidates that may still produce useful CFD evidence.
- Sanity check:
  - The same 15 Halton proposals from the completed shakedown all pass the
    relaxed hard gate, so prior useful borderline candidates would not have
    been discarded.
  - A deliberately tiny-wing test candidate fails before export with
    `min_planform_area_m2`.
- Validation:
  - `python -m py_compile aircraft_optimizer_platform\software\optimizer\scripts\run_real_optimizer_pilot.py aircraft_optimizer_platform\software\optimizer\tests\test_v0_1_flow.py`
    passed.
  - Focused pre-export tests passed: `4 passed, 69 deselected`.
  - Full optimizer suite passed: `73 passed`.
- Next action:
  - Run a smaller real shakedown, for example `8-10` candidates, to confirm the
    new summary artifact appears correctly and that early alpha pruning only
    fires for genuinely bad CFD behavior.

### 2026-06-29 - 8-candidate rough CFD optimizer shakedown

- Run:
  - `runs\real_optimizer_8candidate_alpha4_prune_20260629`.
  - Backend:
    `..\custom_cfd_mesher_experiment\runs\real_optimizer_8candidate_alpha4_prune_20260629_gen1_snappy`.
- Scope:
  - `8` generation-1 candidates.
  - First-pass rough scoring at `0, 4, 8, 12 deg`.
  - Conservative pruning enabled after at least `2` bad alpha results.
- Results:
  - Pre-export screening: `8/8` passed, `0` failed.
  - STL export: `8/8` passed, `0` failed.
  - Snappy/OpenFOAM mesh reports: `8/8`.
  - Steady/plumbing case dirs: `16/16`.
  - Alpha force reports: `32/32`.
  - Early pruned candidates: `0`; skipped alpha cases: `0`.
- Best rough-scoring candidate:
  - `opg1_06_f237e7a3`.
  - Rough score: `82.174253574511`.
  - Best sampled L/D: `3.7936003334957724` at `12 deg`.
  - `CL=0.648717292`, `CD=0.171003067`, `Cm=-0.177792578`.
  - Mesh: `772886` cells, `921796` points, `79614` aircraft faces.
  - Surface fidelity: `p95=0.096 mm`, `p99=0.596 mm`.
- Notes:
  - The conservative alpha-pruning gate did not fire; none of the eight
    candidates were clearly bad after the first two alpha points.
  - The mobile monitor was updated to show pre-export pass/fail counts,
    early-pruning counts, and the correct four-alpha denominator while ignoring
    non-scoring alpha seed folders.
- Validation:
  - `python -m py_compile aircraft_optimizer_platform\software\dashboard\mobile_monitor.py`
    passed during the run.
- Next action:
  - Review the 8-candidate ranking and decide whether to run a true pilot
    optimizer generation from the top candidates, or tighten scoring/constraints
    before letting the optimizer generate children.

### 2026-06-29 - Stricter rough-curve scoring policy

- Updated rough CFD scoring to be more critical of high-alpha-only candidates.
- Added scoring support for:
  - Estimated required level-flight `CL` at the configured mission speed.
  - Interpolated `alpha_at_required_cl`.
  - Interpolated `ld_at_required_cl`.
  - Average usable L/D across the sampled positive-lift alpha curve.
  - L/D curve consistency, defined as average positive L/D divided by best
    positive L/D.
  - Detractors for excessive required alpha, level-flight lift shortfall, and
    weak L/D curve consistency.
- Updated:
  - `software\optimizer\src\aircraft_optimizer\modules\rough_cfd_scoring.py`
    to version `0.2.0`.
  - `software\optimizer\configs\rough_cfd_stable_efficient_drone.v0_1.json`.
  - `software\optimizer\scripts\run_real_optimizer_pilot.py` so future ranked
    CSVs include required-CL and curve-consistency fields.
- Re-scored the completed 8-candidate run without rerunning CFD:
  - Review CSV:
    `runs\real_optimizer_8candidate_alpha4_prune_20260629\review\strict_curve_rescore.csv`.
  - New top candidate: `opg1_03_8a84935c`, score `81.82`.
  - `opg1_06_f237e7a3` dropped from old rank `1` to strict rank `7` because
    it did not reach estimated required level-flight `CL` in the sampled
    `0/4/8/12 deg` alpha range.
  - `opg1_05_674bf32d` became the second-best compromise candidate.
- Validation:
  - `python -m py_compile` passed for the touched scorer/ranker/monitor files.
  - Focused tests passed: `13 passed, 62 deselected`.
- Next action:
  - Use the stricter score for the next child-generation pilot and promote
    from `opg1_03_8a84935c` and `opg1_05_674bf32d`; keep `opg1_06_f237e7a3`
    only as an interesting low-drag/low-Cm design that needs more wing area or
    higher-alpha confirmation.

### 2026-06-29 - Generous Cm policy for thrust-vectoring concept

- Relaxed pitch-moment influence in rough scoring because the vehicle is
  expected to use thrust vectoring and current Cm values are still rough CFD
  estimates.
- Updated:
  - `software\optimizer\src\aircraft_optimizer\modules\rough_cfd_scoring.py`
    to version `0.2.1`.
  - `software\optimizer\configs\rough_cfd_stable_efficient_drone.v0_1.json`.
  - Rough-scoring tests to keep strict trim behavior available as a user-tuned
    override instead of the default.
- Policy:
  - `pitch_moment_trim` weight reduced from `0.08` to `0.03`.
  - `abs_cm_target` relaxed from `0.015` to `0.08`.
  - `abs_cm_limit` relaxed from `0.08` to `0.65`.
  - High-abs-Cm detractor relaxed to start at `0.35`, limit at `0.9`, weight
    `3.0`.
- Re-scored the completed 8-candidate run without rerunning CFD:
  - Review CSV:
    `runs\real_optimizer_8candidate_alpha4_prune_20260629\review\generous_cm_curve_rescore.csv`.
  - New top candidate remains `opg1_03_8a84935c`, score `96.42`.
  - `opg1_07_b677555a` moves up to second under generous Cm, but carries
    `Cm=-0.532` and should remain a review item.
  - `opg1_05_674bf32d` is nearly tied for second and remains the cleaner
    compromise candidate.
  - `opg1_06_f237e7a3` remains low under curve/mission scoring because it did
    not reach estimated required level-flight CL in the sampled alpha range.
- Validation:
  - `python -m py_compile` passed for rough scoring.
  - Focused rough scoring tests passed: `6 passed, 69 deselected`.
- Next action:
  - Use `opg1_03_8a84935c` as the primary seed and consider both
    `opg1_05_674bf32d` and `opg1_07_b677555a` as secondary seeds, with
    `opg1_07` flagged for human review because its Cm is large even if not
    disqualifying.

### 2026-06-29 - Native MC symmetry export path

- Corrected the no-inlet marching-cubes export path away from the legacy
  `aircraft_oml_oblique15_y_slice_frame` diagnostic feature.
- New no-inlet optimizer feature:
  - `aircraft_oml_native_mc`
  - Native aircraft frame: `X = length`, `Y = span`, `Z = vertical`.
  - Generated candidate Rhai now returns `body_outer` as the final SDF
    expression.
- Added optimizer-wrapper support for symmetric half export:
  - `--symmetry-half-mirror`
  - `--symmetry-axis y`
  - `--symmetry-plane-mm 0`
  - Final result summary points to the mirrored/welded full-aircraft STL.
- Evidence run:
  - Root:
    `custom_cfd_mesher_experiment\runs\native_symmetry_export_probe_20260629`.
  - Full native MC export passed strict topology in `235.96 s`.
  - Half native export + mirror wrapper passed strict topology in `201.34 s`.
  - Legacy oblique full-export baseline for the same candidate was `325.55 s`.
  - Native full-vs-half-mirror surface comparison:
    p95 `0.754 mm`, p99 `1.000 mm`, max `1.814 mm`,
    area ratio `0.9999999`.
- Updated:
  - `dual_contouring\direct_sparse_sdf_mc_experiment\scripts\optimizer_export_presets.py`
  - `dual_contouring\direct_sparse_sdf_mc_experiment\scripts\mirror_half_stl.py`
  - `software\optimizer\src\aircraft_optimizer\geometry\real_no_inlet_generator.py`
  - `software\optimizer\src\aircraft_optimizer\external\real_adapter_boundary.py`
  - `software\optimizer\src\aircraft_optimizer\modules\geometry_definition_screening.py`
  - Curated no-inlet Rhai and export docs.
- Current limitation:
  - Validated for the symmetric no-inlet OML case. Inlet/native asymmetric
    variants still need their own symmetry policy before enabling mirror by
    default.

### 2026-06-29 - Normal generated-geometry native symmetry export smoke

- Ran the normal optimizer-facing no-inlet export path from freshly generated
  candidate geometry, not the hand-edited symmetry probe Rhai.
- Workspace:
  `runs\normal_native_symmetry_export_smoke_20260629`.
- Generated geometry:
  `generated_real_geometry\candidate_normal_native_symmetry_smoke\real_no_inlet_candidate_geometry.rhai`.
- Request builder produced the expected command:
  - `--feature aircraft_oml_native_mc`
  - `--bbox-min=-64,0,-80`
  - `--bbox-max=736,416,192`
  - `--symmetry-half-mirror`
  - `--symmetry-axis y`
  - `--symmetry-plane-mm 0`
- Export result:
  - Status: `passed`
  - Final STL:
    `dual_contouring\direct_sparse_sdf_mc_experiment\stl\normal_native_symmetry_smoke_symmetry_mirrored_spacing_1p0.stl`
  - Runtime: `202.53 s`
  - Triangles: `1,147,220`
  - Vertices: `573,612`
  - Boundary edges: `0`
  - Nonmanifold edges: `0`
  - Connected components: `1`
  - Duplicate triangles: `0`
- This confirms the new default optimizer command path produces the clean
  mirrored full-aircraft STL from generated geometry.

### 2026-06-29 - Snappy mesher decision and mirrored mesh probe

- Project mesher policy clarified:
  - `snappyHexMesh` is the active optimizer mesher for candidate meshing,
    rough-CFD scoring development, and OpenFOAM solver runs.
  - Gmsh remains historical plumbing/comparison evidence only unless a later
    evidence-backed decision explicitly supersedes this.
- Mirrored STL export remains the default no-inlet export path, with
  `--full-export` retained as a manual fallback.
- OpenFOAM-native mirrored mesh probe:
  - Half-domain `blockMesh`/`snappyHexMesh` case clamped at `Y=0`.
  - `mirrorMesh` used to generate the full-volume mesh.
  - Coarse no-layer probe passed `checkMesh -skewThreshold 12` and completed
    `potentialFoam`.
  - Evidence summary:
    `custom_cfd_mesher_experiment\runs\mirrored_mesh_probe_20260629\SNAPPY_MIRRORED_MESH_PROBE_SUMMARY.json`.
- Current follow-up:
  - Run a layered/high-fidelity mirrored snappy probe and compare against a
    direct full snappy mesh for runtime, patch counts, non-orthogonality,
    skewness, potentialFoam, and centerline-artifact checks.

### 2026-06-29 - Mesh fidelity pivot after mirrored/layered Snappy probes

- Confirmed current mesher policy remains `snappyHexMesh` for optimizer CFD
  development and OpenFOAM runs.
- Layered mirrored Snappy result:
  - Split-centerline-cap half mesh passed quality gates.
  - Mirroring plus `createPatch` cap-to-aircraft merge produced a final
    two-patch case: `farfield` and closed `aircraft`.
  - Final mirrored layered mesh passed `checkMesh -skewThreshold 12` and
    completed `potentialFoam`.
  - Evidence:
    `custom_cfd_mesher_experiment\runs\mirrored_mesh_probe_20260629`.
- Direct full layered Snappy comparison:
  - Case:
    `custom_cfd_mesher_experiment\runs\mirrored_mesh_probe_20260629\snappy_full_1layer_l4_l5`.
  - Passed `checkMesh -skewThreshold 12` and completed `potentialFoam`.
  - Direct full mesh was cleaner on skewness than the mirrored layered mesh,
    but slower.
- Quality-first follow-up:
  - High-refinement direct full Snappy probe:
    `custom_cfd_mesher_experiment\runs\mesh_quality_refinement_20260629\snappy_full_1layer_l5_l6`.
  - Result: passed mesh quality, about `1.196M` cells and `180,362`
    aircraft patch faces, but took about `19 min`.
  - Aircraft-only screenshot:
    `custom_cfd_mesher_experiment\runs\mesh_quality_refinement_20260629\snappy_full_1layer_l5_l6\aircraft_iso.png`.
- Rejected refinement direction:
  - Broad local refinement boxes around wing/tail/fuselage and narrower
    wing/tail boxes both ran too long and were killed.
  - Conclusion: region boxes are too blunt for optimizer defaults.
- Current blocker:
  - Trailing-edge and fuselage-side mesh fidelity still need improvement
    before spending more time on mirrored mesh speedups.
  - The likely next implementation path is surface-region labeling/splitting
    for Snappy so trailing edges, wing/tail roots, and selected fuselage/blend
    areas can receive targeted surface refinement without refining large
    volumes.

### 2026-06-30 - Implicit aircraft roadmap and Phase 0 SDF conditioning review

- Added `implicit_aircraft_design_roadmap.md` as the long-term kernel-first
  roadmap overlay for local SDF conditioning, geometry-query APIs,
  high-fidelity CFD datasets, future PhysicsNeMo-style surrogates, and later
  SDF-native CFD research.
- Added `research\phase0_sdf_conditioning_literature_review.md` with a focused
  literature review covering SDF redistancing, fast marching, fast sweeping,
  narrow-band level sets, OpenVDB/NanoVDB, scikit-fmm, Voxblox/nvblox, Lethe,
  AMReX embedded boundaries, Basilisk embedded boundaries, immersed-boundary
  CFD, cut-cell CFD, and PhysicsNeMo/FNO surrogate timing.
- Gate decision:
  - Phase 1 local SDF conditioning architecture work is unblocked.
  - Production SDF-native CFD remains deferred research.
  - Production PhysicsNeMo or other AI-surrogate training remains deferred until
    optimizer automation and CFD data quality are trusted.
  - Current OpenFOAM/snappyHexMesh optimizer path remains the production CFD
    path.
- Updated `README.md`, `roadmap.md`, `AGENT_README.md`, and
  `master_platform_manifest.md` to reference the new roadmap and research gate.

### 2026-06-30 - Phase 0 review hardening and conditioned-cache architecture

- Strengthened `research\phase0_sdf_conditioning_literature_review.md` with:
  - explicit success criteria for local conditioning,
  - explicit non-goals,
  - a first-class `Conditioned Geometry Cache` subsystem,
  - technical risk register,
  - fallback architecture if incremental conditioning fails.
- Clarified the main architecture split:
  `canonical geometry graph -> conditioned geometry cache -> clients`.
- Added Phase 7, `Continuous Geometry Platform`, to
  `implicit_aircraft_design_roadmap.md`.
- Updated `AGENT_README.md` so future agents preserve the cache as derived,
  disposable, rebuildable, versioned state rather than a source of truth.

### 2026-06-30 - Phase 1 conditioned-cache execution plan

Planned execution order:

1. Add a formal conditioned geometry cache contract under
   `software\sdf_generation_auto`.
2. Extend artifact/module vocabulary with conditioning artifact kinds, metric
   names, confidence semantics, and fallback status.
3. Add a local-vs-full conditioning prototype fixture using simple analytic
   shapes so dirty-region behavior can be tested without changing the real
   aircraft exporter.
4. Wire dirty-region/cache-readiness metadata into geometry-provider outputs
   while preserving current exporter and optimizer behavior.
5. Run focused tests and documentation checks.

### 2026-06-30 - Phase 1 conditioned-cache contract and fixture pass

- Added `software\sdf_generation_auto\conditioned_geometry_cache_contract.md`
  as the Phase 1 contract for the derived, disposable conditioned geometry
  cache.
- Extended optimizer artifact and module-adapter contracts with conditioning
  artifact types, metric names, confidence semantics, and fallback flags.
- Added a pure-Python local-vs-full conditioning fixture covering radius edits,
  box/corner edits, wing-tip-like edits, and thin-shell edits.
- Added geometry-provider metadata for:
  - `conditioned_geometry_cache`
  - `dirty_regions`
  - `fallback_mode`
  - `optimizer_awareness_required`
- Current production behavior is unchanged:
  - exporters still sample the existing geometry path directly,
  - the conditioned cache is marked `unavailable`,
  - the optimizer does not need to know that conditioning exists.
- Verification:
  - `python -m pytest tests/test_conditioning_fixture.py`: `3 passed`
  - `python -m pytest`: `78 passed`
  - `git diff --check` on touched Phase 1 files: no whitespace errors

### 2026-06-30 - Corrected conditioning ownership boundary

- Clarified that SDF conditioning and the conditioned geometry cache are core
  geometry-generator features, not optimizer features.
- Moved reusable conditioning fixture and metadata helpers from the optimizer
  package into generator-owned package code under
  `software\sdf_generation_auto\src\geometry_generator_conditioning`.
- Kept optimizer wiring as downstream evidence consumption only:
  - geometry-provider outputs can record cache state and dirty regions,
  - optimizer tests can verify the metadata contract,
  - optimizer logic still does not own or implement cache construction.
