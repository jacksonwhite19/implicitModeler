# Agent README

This file is for agents working on the aircraft optimizer platform.

## Project Identity

This is the full aircraft optimizer platform. Keep language, filenames, and future work aligned with that framing.

Working folder:

```text
C:\Users\Jackson\Desktop\02_Projects\09b_Implicit_CAD_claude\aircraft_optimizer_platform
```

## Required Files

Do not remove or rename these without explicit user approval:

- `opt_prompt.md`
- `opt_output.md`
- `master_platform_manifest.md`
- `dependency_inventory.md`
- `environment_strategy.md`
- `implicit_aircraft_design_roadmap.md`
- `research/phase0_sdf_conditioning_literature_review.md`
- `oml_export_contract.md`
- `roadmap.md`
- `progress_tracker.md`
- `README.md`
- `AGENT_README.md`

## Operating Rules

- Do not write production optimizer code unless the user asks to begin implementation.
- Do not redesign the existing implicit CAD/SDF aircraft modeling system.
- Treat the implicit geometry kernel as the canonical aircraft representation; meshes, STLs, OpenFOAM cases, dashboards, datasets, and AI inputs are derived artifacts.
- Treat any conditioned geometry cache as derived, disposable, rebuildable, versioned state between the canonical graph and downstream clients.
- Do not begin production SDF-native CFD or PhysicsNeMo surrogate implementation until the optimizer loop, local SDF conditioning, CFD data schema, and validation evidence are ready.
- Phase 1 local SDF conditioning design is unblocked by `research/phase0_sdf_conditioning_literature_review.md`, but implementation should remain diagnostic-first and locally gated.
- Treat geometry generation as an external provider consumed through an adapter.
- Treat OML STL export as a separate adapter from geometry definition.
- Treat OML STL as a surface artifact, not as a solver-ready CFD mesh.
- Real CFD requires a volume mesh conforming to `software\optimizer\contracts\mesh_export_contract.md`.
- Gmsh/snappyHexMesh comparison work must use `software\optimizer\contracts\mesh_comparison_acceptance_contract.md` and `software\optimizer\configs\mesh_comparison_variants.v0_1.json`.
- Keep platform concerns decoupled: geometry, analysis, scoring, optimization, logging, artifacts, dashboard, and database.
- For v0.1, use the local-first environment strategy. Do not introduce Docker unless the user explicitly asks or a real service/tool boundary exists.
- Treat `aircraft_optimizer_platform` as the master folder for the overall aircraft design/optimization platform.
- Before moving or copying existing SDF/exporter code into this folder, record source path, intended destination, validation status, and provenance in `master_platform_manifest.md`.
- Use `dependency_inventory.md` for source locations, copied inputs, skipped inputs, and open dependency contracts.
- Preserve exact project scope decisions unless the user changes them.
- Update `progress_tracker.md` after meaningful progress, decisions, blockers, or validation results.
- If roadmap sequencing changes, update `roadmap.md` and log the reason in `progress_tracker.md`.
- If architecture changes, update `opt_output.md` and log the decision in `progress_tracker.md`.
- Prefer small, explicit edits over broad rewrites.
- Keep documentation factual and implementation-ready.

## Current Implementation Status

The v0.1 headless skeleton is active under:

```text
software\optimizer
```

It currently includes SQLite records, fixture campaign generation, candidate lineage, user annotations, environment fingerprints, optimizer run/iteration tracking, OML STL fixture-result parsing, SU2 tool detection, SU2 history fixture parsing, a dry-run SU2 case builder, reusable orchestration helpers, request-driven single-candidate evaluation, a five-candidate wing-option diagnostic sweep, a sequential gated optimizer runner, a Halton ask/tell proposal policy, a cheap pre-export screening module, a geometry-definition screening module, a non-exported virtual component mass/CG model, a post-CFD layout-adjustment module, a CFD-readiness gate for parsed OML export metrics, a CFD mesh-result validation module and persistence adapter for native SU2 and CGNS meshes, OpenFOAM strict/relaxed smoke-result validation, an OpenFOAM smoke persistence adapter for non-scoring solver-plumbing evidence, an expanded fixed-topology v0.1 schema with wing variables plus bounded airfoil selector, fuselage length delta, nose bluntness, and tail bluntness, a low-fidelity fixed-wing screening module that uses those knobs as relative ranking signals, a disabled-by-default real no-inlet export boundary, a one-shot real no-inlet STL export validation command, a sequential gated pipeline contract/config, and read-only CLI reports.

Before changing behavior, run:

```text
python -m pytest
```

from `software\optimizer`.

The fixture intentionally does not invoke real CAD, real exporter subprocesses, or real CFD solver execution.

Current primary optimizer smoke command:

```text
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main run-sequential-gated --workspace ..\..\runs\sequential_gated_v0_1 --iterations 3
```

This command proposes one candidate per iteration through the Halton ask/tell policy, runs pre-export screening first, sends only passing candidates to fixture geometry/export and low-fidelity analysis, records failed pre-export candidates, and leaves real export/CFD disabled. The pre-export screen includes a configurable non-exported virtual layout model for battery, avionics, payload, and EDF placement.

Latest expanded stress-run command:

```text
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main run-sequential-gated --workspace ..\..\runs\stress_15_airfoil_fuselage_20260627 --iterations 15
```

Latest expanded stress-run outcome: `15` registered, `12` evaluated, `3`
pre-export failures, promoted candidate
`candidate_2561163970fd4babb73ba495c3c63d97` with
`score.low_fidelity_total = 9.095638`.

Latest expanded-variable real export-only command:

```text
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main run-real-no-inlet-export-batch --workspace ..\..\runs\real_export_expanded_vars_abs_20260627 --iterations 3 --timeout-seconds 1200
```

Latest expanded-variable real export-only outcome: `3/3` real direct sparse
OML STL exports passed, `3/3` `check-cfd-readiness` checks passed, and no mesh
or CFD run was started. Use this workspace as the current proof that expanded
candidate variables reach generated Rhai and real STL export:
`aircraft_optimizer_platform\runs\real_export_expanded_vars_abs_20260627`.

Latest real optimizer pilot:

```text
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main run-real-optimizer-pilot --workspace ..\..\runs\real_optimizer_pilot_6x3_20260628 --run-slug real_optimizer_pilot_6x3_20260628 --initial-count 6 --children-count 3 --timeout-seconds 1200 --alpha-strategy first-pass --resume
```

Outcome: complete. The pilot flow is now available as a first-class resumable
CLI command. It
ran `6` generation-1 real no-inlet exports, meshed and first-pass rough-CFD
scored them, generated `3` generation-2 children from scored parents, and ran
the same rough-CFD path. Summary:
`aircraft_optimizer_platform\runs\real_optimizer_pilot_6x3_20260628\real_optimizer_pilot_summary.json`.
Combined ranking:
`aircraft_optimizer_platform\runs\real_optimizer_pilot_6x3_20260628\combined_ranked_summary.csv`.
Best candidate: `opg2_01_79dab1c3`, rough score `69.3423`,
`L/D=2.4982` at `+4 deg`, `CL=0.2389`, `CD=0.09564`,
`Cm=-0.07237`.

Important pilot finding: the first gen-1 alpha sweep used the old convention
(`Z` lift, angle of attack in `X/Z`), which is wrong for the current real
no-inlet STL frame (`X=length`, `Y=vertical`, `Z=span`). Preserve the wrong
summary as evidence only:
`custom_cfd_mesher_experiment\runs\real_optimizer_pilot_6x3_20260628_gen1_snappy\alpha_sweep_summary_first_pass_end60_np8_wrong_z_lift.json`.
The corrected path uses drag `+X`, lift `+Y`, pitch `+Z`, and angle of attack
in the `X/Y` plane. Use `--resume` after timeouts so completed export, mesh,
and alpha artifacts are reused instead of rerun.

Do not calculate or gate static margin in pre-export screening. Pre-export layout may move battery/avionics within allowed ranges toward a configurable target CG, but static margin requires a neutral point from later CFD/stability analysis. After that neutral point exists, use the post-CFD layout-adjustment path to test whether movable components can hit the requested static margin; fixed components such as the EDF must remain fixed.

## Current Scope Constraints

Initial platform scope:

- Small UAVs.
- Fixed-wing aircraft.
- Loiter/endurance optimization.
- Continuous variables only.
- Existing implicit CAD/SDF geometry provider.

Initial exclusions:

- No topology changes.
- No adding/removing wings or fins.
- No aircraft-family changes.
- No VTOL/tailsitter implementation yet.
- No default full-CFD campaign.
- No surrogate model implementation yet.
- No full MDAO coupling yet.

## Current OML Export Decision

The current validated optimizer-facing CFD STL export contract is documented in:

```text
oml_export_contract.md
```

Default path:

- `direct_sparse_oml_fast`
- `spacing_mm = 1.0`
- `tile_size_mm = 16`
- `sdf-workers = 8`

High-fidelity selected-design path:

- `direct_sparse_oml_high_fidelity`
- `spacing_mm = 0.75`
- `tile_size_mm = 24`
- `sdf-workers = 8`

Use fast export by default during optimizer iteration. Use high-fidelity export only when a few designs are converging or when preparing selected designs for higher-fidelity analysis.

The primary autonomous optimizer loop is sequential and gated:

```text
design -> cheap pre-export checks -> geometry/export if allowed -> analysis if allowed -> results -> next design
```

Do not build a default workflow that exports all candidates up front. Batch sweeps are allowed as diagnostic studies, DOE studies, parallel campaigns, or future surrogate-data generation, but they are not the primary optimizer control flow.

Do not route shell/manufacturing export into the default optimizer flow yet. Shell export is runtime-viable but parked because shell SDF topology defects remain unresolved.

Production CFD-ready OML gate defaults:

- `boundary_edges = 0`
- `nonmanifold_edges = 0`
- `connected_components = 1`
- `duplicate_triangles = 0`
- `long_chord_sections_ge_75mm = 0`

Relaxed gates are acceptable only for geometry stress tests and must be recorded in the candidate/evaluation artifact metadata.

Passing OML STL topology gates is not sufficient for real CFD execution. A candidate must also pass a volume-mesh preflight that produces positive node/cell counts and maps platform semantic markers (`aircraft`, `farfield`, `fluid`) to solver-visible marker names. CGNS section names such as `3_S_7` are acceptable only as backend marker-map values, not as platform-level names.

Native SU2 meshes use the implicit single volume for the `fluid` semantic region. CGNS meshes need an explicit `--fluid-marker`, usually a solver-visible section name such as `5_V_1`.

CFD force conventions live in:

```text
software\analysis_modules\cfd_aerodynamic_conventions.md
```

Current real no-inlet optimizer convention is drag `+X`, lift `+Y`, pitch axis `+Z`, and angle of attack in the `X/Y` plane because the current exporter frame is `X=length`, `Y=vertical`, `Z=span`. Older sandbox runs that used `Z` as lift should be treated as stale convention evidence and not compared directly to corrected rough-CFD `Cl`.

The current CLI readiness command is:

```text
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main check-cfd-readiness --export-result .\examples\exporter_results\oml_stl_passed.json
```

This is a pre-solver gate only. It does not run SU2/OpenFOAM.

The current mesh-result validation command is:

```text
$env:PYTHONPATH='src'
python -m aircraft_optimizer.cli.main check-mesh-result --mesh <mesh-path> --mesh-format su2
```

For CGNS, include `--su2-log <log-path>` plus explicit `--aircraft-marker`, `--farfield-marker`, and `--fluid-marker` arguments.

## Master Folder Integration

The master component map lives in:

```text
master_platform_manifest.md
```

The intended implementation homes live under:

```text
software/
```

Current stance:

- Wrap existing working code in place before importing it.
- Import or vendor code only after the interface is stable.
- Do not delete original source files unless explicitly requested.
- Separate manual SDF generation, automatic SDF generation, OML STL export, optimizer core, dashboard, and analysis modules.
- Keep scratch/diagnostic files out of canonical component folders until triaged.
- Rhai scripts may be copied into `software/sdf_generation_manual/curated_rhai`, but keep the set small. Prefer a scratchpad plane plus one or two validated exporter models/inlet variants over broad historical imports.

## Roadmap Discipline

Follow `roadmap.md`.

  Immediate next build target:

- Keep the optimizer STL-native. Do not recreate generated aircraft in VLM,
  OpenVSP, or other alternate geometry systems for normal candidate
  evaluation.
- Use cheap pre-screening only to reject physically impossible candidates.
  Plausible candidates should receive some CFD signal.
- Use
  `software\optimizer\configs\snappy_openfoam_external_aero_ranking.v0_1.json`
  as the first medium-cost ranking-CFD preset. It is intended to sort
  candidates by relative aerodynamic behavior, not to provide trusted absolute
  lift/drag.
- Latest accepted fresh-STL ranking run:
  `custom_cfd_mesher_experiment\runs\fresh_no_inlet_snappy_ranking_v0_1_20260625`.
  Accepted import workspace:
  `aircraft_optimizer_platform\runs\fresh_no_inlet_snappy_ranking_v0_1_import_with_steady`.
  Do not use the earlier partial import workspace without steady evidence as
  the accepted record.
- First proxy rank-correlation report:
  `aircraft_optimizer_platform\runs\cfd_rank_correlation_v0_1\ranking_vs_confirmation_summary.md`.
  Time-60 versus time-100 ordering matched for Cd and Cl across the five v0.4
  faired-cap runs; moment ordering is not reliable yet.
- First separate-root ranking/confirmation report:
  `aircraft_optimizer_platform\runs\cfd_rank_correlation_separate_roots_v0_1\ranking_vs_confirmation_summary.md`.
  Three ranking runs under
  `custom_cfd_mesher_experiment\runs\faired_cap_snappy_ranking_v0_1_three_variant_20260625`
  matched the existing v0.4 confirmation ordering for Cd and Cl. Caveat: the
  built-in faired-cap selector uses prepared/remeshed `aircraft_surface_iso.stl`
  files from `custom_cfd_mesher_experiment\runs\faired_cap_gmsh_openfoam_20260623`,
  not raw direct exporter STLs.
- Strongest current raw direct-STL ranking evidence:
  `custom_cfd_mesher_experiment\runs\raw_direct_faired_cap_snappy_ranking_v0_1_three_variant_20260625`.
  Use this as the preferred ranking-CFD evidence anchor. It used raw direct
  exporter STLs from
  `dual_contouring\direct_sparse_sdf_mc_experiment\stl`, completed 60-step
  ranking and 100-step confirmation kOmegaSST for `fcv02`, `fcv03`, and
  `fcv04`, and matched rank order exactly for Cd, Cl, and abs(Cm). Correlation
  report:
  `aircraft_optimizer_platform\runs\cfd_rank_correlation_raw_direct_stl_v0_1\ranking_vs_confirmation_summary.md`.
  Imported workspace:
  `aircraft_optimizer_platform\runs\raw_direct_faired_cap_snappy_ranking_v0_1_import`.
- Current polar-aware rough-scored raw direct-STL import workspace:
  `aircraft_optimizer_platform\runs\raw_direct_faired_cap_snappy_alpha_first_pass_tolerant_v0_3_import`.
  It adds one `rough_cfd_scoring` module attempt per candidate using
  `software\optimizer\configs\rough_cfd_stable_efficient_drone.v0_1.json`
  plus the first-pass alpha summary
  `custom_cfd_mesher_experiment\runs\raw_direct_faired_cap_snappy_ranking_v0_1_three_variant_20260625\alpha_sweep_summary_first_pass_end60_np8.json`.
  First-pass angles are `0, +4 deg`; all 6 RANS cases completed. Expanded
  survivor scoring uses
  `custom_cfd_mesher_experiment\runs\raw_direct_faired_cap_snappy_ranking_v0_1_three_variant_20260625\alpha_sweep_summary_survivor_end60_np8.json`
  with `-2, 0, +4, +8, +10 deg`.
  Current tolerant first-pass order is `fcv04_compact_wide_tail` (`72.250`,
  scored at `+4 deg`), `fcv03_high_aspect_mild` (`65.510`, scored at `+4 deg`),
  then `fcv02_short_swept` (`62.369`, scored at `+4 deg`). Users can override
  weights and targets with `--rough-scoring-config`, and can choose the alpha
  summary with `--alpha-sweep-summary-name`. The workspace includes an optimizer
  run with `score.rough_total` as the primary objective.
- Alpha-sweep note: the backend can now write separate per-alpha solver cases
  and aggregate them. Treat the resulting L/D envelope as rough ranking
  evidence only, not final validated aerodynamics.
- First-pass screening is intentionally tolerant: with only `0/+4 deg`, the
  scorer uses the best positive-lift L/D point and marks high-Cm points for soft
  review instead of rejecting or collapsing them to the trim-limited point.
  Expanded survivor scoring remains the stricter place to resolve moment/trim
  concerns.
- Adaptive survivor promotion belongs to the optimizer policy, not to a fixed
  top-N filter. Use
  `aircraft_optimizer.optimizers.adaptive_cfd_promotion.decide_adaptive_cfd_promotion`.
  Current policy: run expanded sweeps for the first `5` baseline candidates;
  after that, reject first-pass candidates only for hard failures or clear
  underperformance such as `>=65%` less `CL` at `+4 deg` than the best expanded
  baseline case. Otherwise promote to the expanded sweep.
- `run-snappy-openfoam-backend` now has alpha execution strategies:
  - `--alpha-strategy adaptive` default: baseline-building, then first-pass plus
    conditional survivor expansion.
  - `--alpha-strategy first-pass`: force `0/+4 deg`.
  - `--alpha-strategy survivor-expanded`: force `-2/0/+4/+8/+10 deg`.
  - `--alpha-strategy none`: do not launch alpha sweeps.
  - `--adaptive-baseline-summary <path>` can be repeated to provide prior
    expanded/adaptive summaries for the evolving optimizer baseline.
- Use `report-adaptive-cfd-promotion` before candidate runs when the operator
  wants to inspect the current baseline and exact first-pass reject threshold.
  It accepts repeated `--baseline-summary`, optional
  `--candidate-alpha-summary`, and optional `--variant-id`.
- Use
  `software\optimizer\configs\snappy_openfoam_external_aero_hifi_surface.v0_4.json`
  for stricter confirmation/scoring-development reruns on survivors.
- Preserve surface fidelity and keep scoring terminology qualified. The
  ranking preset may produce `scoring_tier=rough_scoring` for optimizer-relative
  ranking. The legacy `scoring_allowed=false` flag means final/engineering
  scoring is still blocked until rank-correlation, mesh-convergence,
  wall-treatment/y-plus, and validation evidence are stronger.

Do not treat `low_fidelity_fixed_wing` as validated aerodynamics. It is a screening estimate for optimizer plumbing and early ranking behavior.

Use `preflight-real-adapters --workspace <path>` to verify the no-inlet Rhai reference, exporter script, and SDF sidecar are visible. This preflight does not execute the exporter.

Use `run-real-no-inlet-export --workspace <path>` only for one controlled candidate. It executes the direct sparse OML exporter, copies the STL/result/log artifacts into the platform artifact store, and does not run aero/CFD/scoring analysis.

The no-slip solver ladder can now run `foamRun` in parallel through
`run_no_slip_laminar_start_rans.py --parallel-procs <n>`. The mesher was
already parallelized; this keeps medium-cost ranking CFD from being dominated
by a serial RANS leg.

Do not skip directly to dashboard polish, surrogate models, MDAO, real CAD/exporter calls, or real CFD solver execution before the headless orchestration layer is request-driven and easier to maintain.

Current dashboard direction:

- Use the native Rust dashboard app at
  `..\src\bin\optimizer_dashboard.rs`.
- Build it from the repo root with
  `cargo build --release --bin optimizer_dashboard`.
- It reads optimizer SQLite workspaces directly and launches the existing fast
  standalone `src\bin\sdf_viewer.rs` viewer for candidate inspection.
- Candidate preview is controlled by
  `<workspace>\dashboard_sdf_viewer_target.txt`; the dashboard launches
  `sdf_viewer.exe --watch-target <file>` and the viewer reloads when that file
  changes.
- `Sync Viewer` mode should remain the default dashboard behavior: selecting a
  candidate updates the linked viewer session automatically.
- It owns dashboard-side tabs for overview, candidate detail, scoring plots,
  alpha curves, operational history/lineage, proposed-next preview, and
  controls.
- Operational candidate state belongs in `candidate_runner_states`. Use
  `upsert_candidate_runner_state(...)` from the Python optimizer repositories
  for queued/running/meshing/cfd_running/scored/promoted/rejected/failed updates
  instead of adding dashboard-only state inference.
- Control buttons write request JSON files under
  `<workspace>\dashboard_control_requests`. Do not treat those request files as
  executed work until a runner/controller consumes them and appends normal
  optimizer records.
- Do not reintroduce the temporary Python `serve-dashboard` web UI as the
  primary dashboard. The Python optimizer package should remain the headless
  orchestration layer.
- Do not re-enable embedded WGPU viewer rendering in the dashboard without a
  bounded stability/performance test. The first embedded attempt was much
  slower than the bare viewer and could crash the app.

Current CFD sandbox direction:

- Historical hybrid OpenFOAM mesher-plumbing backend:
  hybrid direct-prism-shell plus Gmsh outer-volume mesh, using the platform
  preset in
  `software\optimizer\configs\hybrid_prism_gmsh_openfoam.v0_1.json`.
- Latest five-variant evidence root:
  `custom_cfd_mesher_experiment\runs\hybrid_shell_gmsh_exact_merge_five_variant_netgen_v0_1_20260624`.
- Result:
  - strict OpenFOAM `checkMesh`: `5/5` pass.
  - `potentialFoam -writep`: `5/5` complete.
  - cells: `752,726` to `992,789`.
  - severe non-orthogonal faces: `178` to `470`.
  - runtime: about `103` to `141` seconds per variant.
- Selector summary:
  - start from a watertight conditioned aircraft surface.
  - build a direct prism shell with cumulative offsets `0.2/0.5/1.0 mm`.
  - export `aircraft_wall.stl` and `outer_shell.stl`.
  - build a Gmsh tetrahedral farfield volume from `outer_shell.stl`.
  - use Gmsh optimizers `default,Netgen`.
  - convert the Gmsh outer volume with `gmshToFoam`.
  - exact-merge the shell and Gmsh outer volume with
    `custom_cfd_mesher_experiment\scripts\merge_prism_shell_gmsh_outer.py`.
  - require strict `checkMesh` and `potentialFoam -writep`.
- Keep `scoring_allowed=false`. This is the first all-five automated
  hybrid boundary-layer/prism plumbing pass, not trusted lift/drag CFD. The
  no-slip OpenFOAM path still needs stable convergence, y-plus/wall-treatment
  policy, and force/moment coefficient gates.
- Historical Gmsh boundary-layer/prism mesh preset:
  `software\optimizer\configs\gmsh_openfoam_external_aero_bl.v0_2.json`.
  Keep it as comparison evidence, not the preferred backend, now that the exact
  hybrid merge passes all five variants.
- Latest hybrid no-slip steady probe:
  `custom_cfd_mesher_experiment\runs\hybrid_shell_gmsh_simplefoam_forcecoeffs_probe_v0_1_20260624`.
  The fcv04 laminar no-slip runs with `foamRun -solver incompressibleFluid`
  failed by time step 2 with pressure-solver floating point exceptions. The
  mesh passed strict `checkMesh` and `forceCoeffs` wrote rows, but coefficients
  were unstable. Treat this as `fail_solver_stability_non_scoring`.
- Latest hybrid split-farfield/transient diagnostic:
  `custom_cfd_mesher_experiment\runs\hybrid_shell_gmsh_split_farfield_probe_v0_1_20260624`.
  Split farfield patch preservation now works in the exact-merge path and the
  split mesh passes strict `checkMesh` plus `potentialFoam -writep`. A matching
  split-farfield steady no-slip run still failed at time step 2 with a
  pressure-solver floating point exception. Transient `icoFoam` failed at
  `deltaT=1e-5 s` with max Courant about `96.7`, then completed five steps at
  `deltaT=1e-7 s` with small continuity errors. A ten-step extension at the
  same timestep also completed, with final max Courant about `0.826` and final
  local continuity about `9.10e-15`, but required about `120 s` to advance only
  `1e-6 s` of physical time. Current diagnosis: the hybrid mesh is
  OpenFOAM-valid and not inherently impossible for no-slip flow, but direct
  full-speed steady startup is too stiff with the current tiny near-wall cells
  and wall-treatment/time-scale policy. Do not treat SU2 as a magic fix; it may
  be a future alternate backend, but it needs the same mesh quality, marker,
  wall-treatment, y-plus, reference-value, and convergence gates.
- Latest parallel snappy benchmark:
  `custom_cfd_mesher_experiment\runs\snappy_parallel_benchmark_v0_1_20260624`.
  The useful case is
  `fcv04_compact_wide_tail_np12_l3_l4`: 12-core snappy, surface levels `3/4`,
  feature level `3`, `243,636` cells, `23,962` aircraft faces, max skewness
  `8.9609095`, max non-orthogonality `71.555609`, and `7` severe non-orthogonal
  faces. Strict default `checkMesh` fails on skewness, but
  `checkMesh -skewThreshold 12` passes and `potentialFoam -writep` completes.
  The user approved `12.0` as a development skewness ceiling for this benchmark.
  The same mesh completed a 20-iteration laminar no-slip
  `foamRun -solver incompressibleFluid` case without FPE and completed a
  ten-step `icoFoam` transient at `deltaT=1e-7 s` with final max Courant about
  `0.00883`, much lower than the hybrid transient probe. Treat snappy as the
  next scoring-CFD development mesh candidate, while keeping hybrid prism/Gmsh
  as the faster OpenFOAM plumbing backend until snappy proves repeatability and
  coefficient stability.
- Persisted optimizer workspace for the five selected Gmsh BL cases:
  `aircraft_optimizer_platform\runs\gmsh_bl_openfoam_persistence_v0_1`.
  It contains five imported candidates, five complete evaluations, five
  successful `openfoam_smoke_validation` attempts, thirty artifacts, and zero
  failures.
- First stable no-slip plumbing run on the Gmsh BL prism path:
  `custom_cfd_mesher_experiment\runs\gmsh_bl_no_slip_smoke_v0_1_20260624\fcv04_laminar_relaxed_10`.
  It completed a conservative 10-iteration laminar `foamRun -solver
  incompressibleFluid` run with aircraft `noSlip` and `forceCoeffs`.
  Treat the coefficients as non-scoring and non-converged. The failed
  `fcv04_laminar_30` attempt in the same root shows the less-relaxed setup
  diverged badly without an FPE.
- Current best no-slip plumbing run:
  `custom_cfd_mesher_experiment\runs\gmsh_bl_no_slip_smoke_v0_1_20260624\fcv04_laminar_p02_u03_20`.
  It completed 20 laminar steady iterations with pRelax `0.2`, uRelax `0.3`,
  no FPE, final local continuity `2.88911e-05`, and final-window coefficient
  spans `Cd=0.009578139`, `Cl=0.002713398`. Final coefficients were
  `Cd=0.107564632`, `Cl=0.0733830739`, `Cm=-0.0101037588`.
- Persistence for non-potential OpenFOAM steady/no-slip evidence is now wired
  through `openfoam_steady_validation`, `check-openfoam-steady-result`, and
  `persist-openfoam-steady`. The best fcv04 run was persisted as
  `module_9087883bcb334000a25b72f2e708e2df` against evaluation
  `evaluation_f832b0049cb947b9a54652c58c9d70d2`.
- Solver decision:
  Continue using `foamRun -solver incompressibleFluid` for bounded steady
  incompressible no-slip development. OpenFOAM 13 has standalone `icoFoam`,
  but it is transient laminar and is not the default for steady coefficient
  plumbing. No standalone `isoFoam` path was found on this install.
- Historical selected non-prism OpenFOAM mesher-plumbing backend:
  Gmsh v0.2 tetra/farfield policy in
  `custom_cfd_mesher_experiment\runs\gmsh_optimizer_hardening_v0_2_20260623\gmsh_optimizer_preset_v0_2.json`.
  Keep it as fallback evidence only unless the prism path regresses.
- Historical non-prism hardening report:
  `custom_cfd_mesher_experiment\runs\gmsh_optimizer_hardening_v0_2_20260623\GMSH_NEXT_STEP_REPORT.md`.
- The older non-prism v0.2 policy keeps the current 4 mm baseline with
  `default,Netgen,Relocate3D`, plus a documented 5 mm automatic retry when
  4 mm Gmsh volume generation fails. It requires strict `checkMesh`,
  `defaultFaces = 0`, clean expected patch mapping, severe non-orthogonality
  localization artifacts, aircraft-only screenshots, and `potentialFoam -writep`.
- Do not adopt the tested 4.5 mm global preset yet. The 4.5 mm Netgen probe
  crashed on `fcv01`, and the 4.5 mm no-Netgen probe produced `defaultFaces=16`
  with worse severe non-orthogonality. The `fcv03` 4.5 mm result is a useful
  bounded diagnostic only.
- Keep final scoring blocked for all OpenFOAM CFD paths until a no-slip
  OpenFOAM solver case, boundary-layer/prism strategy, wall-treatment/y+ policy,
  and force coefficient convergence checks are working. Rough scoring is allowed
  only when a named policy explicitly emits `scoring_tier=rough_scoring`.
- First no-slip steady OpenFOAM plumbing run:
  `custom_cfd_mesher_experiment\runs\gmsh_openfoam_simplefoam_plumbing_v0_1_20260623`.
  The `fcv01_long_glider` Gmsh v0.2 mesh completed 20 steps with
  `foamRun -solver incompressibleFluid`, `kOmegaSST`, aircraft `noSlip`, split
  farfield patches, and `forceCoeffs`. Treat it as
  `openfoam_steady_plumbing` with `scoring_allowed=false` because force
  coefficients were unstable and that older no-slip run had no accepted
  prism/boundary-layer strategy.
- Coefficient diagnosis in the same root found the axis/sign convention is
  internally consistent: `X` is length/flow, `Y` is span, `Z` is vertical,
  drag is `+X`, lift is `+Z`, and pitch is `+Y`. The higher-priority blocker is
  wall treatment: the `kOmegaSST` no-slip case reported aircraft y+ min
  `0.95845508`, max `324.81977`, average `32.226444`. A same-mesh laminar
  diagnostic was much calmer but still failed force-coefficient stability.
- Fresh Gmsh/OpenFOAM five-variant evidence root:
  `custom_cfd_mesher_experiment\runs\faired_cap_gmsh_openfoam_20260623`.
  It uses the approved full-path solid faired-cap inlet STLs for `fcv01` through
  `fcv05`; do not substitute old no-inlet or stale capped-inlet runs as current
  acceptance evidence.
- Gmsh follow-up root:
  `custom_cfd_mesher_experiment\runs\faired_cap_gmsh_openfoam_20260623\followup_gmsh_geometry_nonortho`.
  Use `gmsh_followup_final_report.md` and `gmsh_followup_final_summary.json`
  for feature closeups, `nonOrthoFaces.vtk` localization, bad-face overlays,
  and the bounded `fcv03` 4.5 mm improvement result.
- Selected Gmsh results for all five variants pass strict `checkMesh` and
  complete `potentialFoam -writep` at 50 mph. `fcv03_high_aspect_mild` uses the
  documented 5 mm retry because the initial 4 mm Gmsh volume generation crashed;
  preserve that failed case in comparisons.
- The `fcv03` 4.5 mm improvement result is not the default: it reduced severe
  non-orthogonal faces from `93` to `82` and improved surface fidelity, but
  increased cells and still leaves scoring-CFD-blocking feature clusters.
- Treat this Gmsh run as mesher/solver-plumbing evidence only. It has no
  prism/boundary-layer strategy and uses a slip aircraft patch for
  `potentialFoam`, so it is not scoring-quality CFD and should not produce
  trusted lift/drag.
- Keep snappyHexMesh as a secondary research path for future layered or
  hex-dominant boundary-layer work, but do not block Gmsh optimizer plumbing
  integration on snappy.
- Keep SU2 as a known-good solver/marker contract path for simple native Gmsh/OCC meshes and as a future option once aircraft surface meshing improves.
- Treat the full-aircraft `potentialFoam` run as mesh/solver plumbing proof only; it is inviscid and not a scoring-quality drag analysis.
- The first modified-SDF full path used `su2_sandbox\examples\basic_no_inlet_wing_span_plus20.rhai` with `wing_span = 720.0`; it exported, repaired, meshed, solved with `potentialFoam`, and passed `check-openfoam-result`.
- Preserve raw surface-gate failures even when a repair path later succeeds; the wing-span +20 raw export had `10` boundary edges before repair.
- The Poisson-repaired smoke mesh is not simulation quality. It passed `checkMesh`, but user visual inspection rejected it as too faceted with poor wing blend and leading/trailing-edge fidelity.
- Historical l3/l4 OpenFOAM current-option mesh `su2_sandbox\runs\basic_no_inlet_wing_span_plus20_openfoam\case_local_cap_l3_l4_smooth_t1` preserved more geometry than Poisson but failed strict `checkMesh` on localized skew faces.
- Use `su2_sandbox\scripts\summarize_openfoam_mesh_cases.py` after OpenFOAM experiments to refresh `mesh_case_summary.csv` and `mesh_case_summary.md`.
- For failing `checkMesh` cases, run `checkMesh -writeSets -writeSurfaces -setFormat vtk -surfaceFormat vtk` to preserve localized failure geometry such as `skewFaces.vtk`.
- Every mesh run should generate an aircraft-only ISO screenshot from the `aircraft` patch VTK, not the farfield or volume mesh.
- Current-options status after auto-repair: the OpenFOAM/snappy path is repeatable and `potentialFoam` will run. The preferred snappy baseline now passes strict default `checkMesh`, but it is still not scoring-CFD-ready.
- Rejected current-option fixes include OpenFOAM `surfaceClean`, OpenFOAM `surfaceLambdaMuSmooth`, conservative short-edge welding alone, disabling feature snap, stricter snappy skew thresholds, and snap tolerance `2.0`.
- Relaxed-development OpenFOAM acceptance is now explicit. Use `checkMesh -skewThreshold 6` and the platform `RELAXED_DEVELOPMENT_OPENFOAM_RESULT_POLICY` for smoke CFD/plumbing only.
- Strict default `checkMesh` remains the scoring-CFD target. If strict fails but relaxed passes, label the result `relaxed_development`, not production CFD.
- Optimizer persistence for this path is `openfoam_smoke_validation` through `persist-openfoam-smoke`. It records existing OpenFOAM logs and artifacts; it does not run OpenFOAM and always sets `scoring_allowed=false`.
- Use `su2_sandbox\scripts\prepare_cfd_surface.py` for raw exporter STL prep. The preferred cap method is currently `edge_fan`; unconstrained Delaunay did not close the current boundary loop.
- Strict snappy pass has now been achieved without changing the direct sparse exporter: apply `prepare_cfd_surface.py --cap-method edge_fan` and local STL smoothing around `617.65,-137.837,165.003 mm` with radius `8 mm`, `4` iterations, relaxation `0.12`.
- Current strict-pass reference case: `su2_sandbox\runs\basic_no_inlet_wing_span_plus20_openfoam\case_local_smooth_r8_i4_l3_l4_smooth_t1`.
- New-vehicle strict-pass replay: `su2_sandbox\runs\basic_no_inlet_root180_tip70_openfoam\case_local_smooth_r8_i4_l3_l4_smooth_t1`.
- The hard-coded smoothing-center experiment has been generalized into `su2_sandbox\scripts\auto_repair_snappy_stl.py`.
- Auto-repair reference run: `su2_sandbox\runs\basic_no_inlet_wing_span_plus20_auto_repair_strict`.
- Auto-repair behavior: prepare raw STL, run strict snappy, parse `skewFaces.vtk`, cluster localized skew defects, apply bounded local STL smoothing, rerun strict snappy, and preserve every failed and repaired case.
- Auto-repair validation: iteration `00` reproduced strict failure with max skewness `4.5113177` and `2` skew faces; iteration `01` passed default `checkMesh` with max skewness `3.8155389`, then ran `potentialFoam` at `50 mph`.
- Improved snappy baseline: `su2_sandbox\runs\basic_no_inlet_wing_span_plus20_auto_repair_l4_l5\case_repair_iter_01`.
- Improved snappy baseline settings: surface level `4-5`, feature level `4`, auto-repair enabled.
- Improved snappy baseline metrics: `978,881` cells, `1,175,505` points, `134,254` aircraft faces, max skewness `3.8826061`, max non-orthogonality `74.777901`, strict `Mesh OK`, `potentialFoam` complete.
- Sampled point-to-triangle source-to-patch deviation for improved baseline: p50 `0.00552 mm`, p95 `0.0472 mm`, max `0.5146 mm`.
- Feature level `5` was tested but should not be the default: it has `150,197` aircraft faces, but max skewness `3.9954521` is too close to the strict threshold and sampled source-to-patch p95 was slightly worse at `0.0532 mm`.
- Do not treat auto-repair as final scoring-CFD policy yet. It is the current best snappy mesh-conditioning workflow and needs five-variant coverage plus feature-aware deviation buckets.
- Current mesher comparison policy: Gmsh is the selected near-term default for
  OpenFOAM plumbing; snappyHexMesh remains secondary research evidence, mainly
  for future boundary-layer or hex-dominant strategies.
- Current Gmsh single-inlet status: high-fidelity single-inlet export is clean, but the best current Gmsh mesh remains conditional/failing for scoring due to localized severe non-orthogonal faces. Standard 4 mm no-smooth mesh had `418` severe faces; 4 mm smoothed 6-iteration mesh improved only to `408`; 5 mm coarser mesh worsened to `476`; algorithm 10 and no-Netgen variants worsened.
- Gmsh gated runner now writes `checkMesh -writeSurfaces -writeSets -nonOrthThreshold 70` diagnostics on mesh-gate failure and localizes `nonOrthoFaces.vtk` into `nonortho_localization.json`.
- Do not use broad volume refinement boxes generated from bad-face hot spots as the next Gmsh fix. The first eight hot-spot boxes at `0.0028 m` target size forced Gmsh toward about `196M` blockfill points and crashed. Prefer geometry-aware local surface conditioning or much tighter feature-specific sizing.
- Use `--smooth-remesh` only as an explicit experiment in `custom_cfd_mesher_experiment\scripts\run_gated_mesh_smoke.py`; it is not a default because it only marginally improved the single-inlet severe-nonorth count.
- snappy comparison guardrail: do not launch snappyHexMesh when the prepared STL remains open. The `mc_v01_span720_reference` comparison STL currently has a `52`-edge boundary loop that `surfaceCheck` reports as open and the current `10 mm` auto-cap policy refuses. Fix/cap that loop first, or record a fail-fast surface-prep failure.
- snappy runtime guardrail: use `run_snappy_mesh_comparison_variants.py` defaults unless deliberately overriding them. The comparison harness now caps snappy at `1200 s`, caps one variant at `2400 s`, and defaults to one repair iteration.
- snappy relaxed-quality policy: the current accepted comparison threshold is `max skewness < 8`. Strict OpenFOAM `checkMesh` failure must still be recorded, but relaxed accepted meshes may proceed to `potentialFoam` and be marked conditional plumbing evidence.
- For `mc_v01_span720_reference`, OpenFOAM `surfaceConvert -clean` and tiny `surfacePointMerge` were too slow and did not close the `52`-edge loop. The best bounded prep so far is a Delaunay cap with `max_loop_span_mm=20`, adding `50` faces and no vertices.
- Best relaxed v01 snappy case so far: `su2_sandbox\runs\mesh_comparison_snappy\mc_v01_span720_reference_delaunay_cap20_l4_l5_repair1\case_repair_iter_01`; max skewness `4.1238511`, `1` skew face, `potentialFoam` complete.
- Current paused snappy comparison root:
  `su2_sandbox\runs\mesh_comparison_snappy_relaxed_skew8_delaunay_cap20_v2`.
  Do not resume that five-variant run as acceptance evidence after the inlet
  redesign. Keep it only as no-inlet reference evidence and script validation.
- Completed v01 in that paused root:
  `su2_sandbox\runs\mesh_comparison_snappy_relaxed_skew8_delaunay_cap20_v2\mc_v01_span720_reference`.
  It is `relaxed_mesh_ok` with max skewness `6.218329` and max
  non-orthogonality `69.878293`.
- Snappy harness updates to preserve: absolute run paths, explicit cap method,
  default Delaunay cap at `20 mm`, `foamToVTK -noInternal`, raw STL gate failure
  recorded but not fatal when the prepared surface closes successfully, and a
  relaxed non-orthogonality ceiling default of `70`.
- Snappy reporting contract:
  `software\optimizer\contracts\snappy_mesher_evidence_contract.md`.
  Every future snappy comparison variant should emit `run_manifest.json`,
  `evidence_bundle.json`, `evidence_report.md`, mesh/geometry/solver JSON, logs,
  and aircraft-only screenshots when available.
- The snappy harness is partial-run safe and continues by default. Use
  `--stop-on-failure` only when intentionally debugging one variant.
- Use `--report-existing` only to refresh schema/report artifacts for already
  completed reference cases. Do not use report-only refreshed old-geometry cases
  as acceptance evidence after an inlet or aircraft-family change.
- Current automated geometry-capture feature buckets are proxy metrics. They
  use `x` as longitudinal and `z` as lateral/span unless explicitly overridden.
  Screenshots remain required for judging actual LE/TE/blend/tip capture.
- Current inlet redesign guardrail: `faired_cap` should be a separate solid
  optimizer inlet body with a shallow forward cap. It should follow the full
  buried inlet path and terminate inside the fuselage, not stop at first
  fuselage contact and not cut an aft outlet. Do not resurrect the earlier
  flow-through-to-cap, abbreviated-path, or short-stub approaches as acceptance
  evidence. The approved `fcv01` through `fcv05` full-path faired-cap STL set is
  current for mesher comparison.
- Approved faired-cap inlet snappy/OpenFOAM run:
  `su2_sandbox\runs\openfoam_snappy_faired_cap_inlets_v0_1_20260623_skew8_v2`.
  All five variants completed `potentialFoam -writep`, but all are
  `conditional_development_only` because strict `checkMesh` failed while
  relaxed skew `<8` accepted the meshes for plumbing evidence. Do not treat
  these as scoring-CFD cases.
- Current snappy scoring-development evidence root:
  `custom_cfd_mesher_experiment\runs\snappy_parallel_benchmark_v0_1_20260624`.
  The current best fcv04 mesh is
  `fcv04_compact_wide_tail_np12_l3_l4_layers3_thick`, with surface levels
  `3/4`, feature level `3`, requested `3` layers, final layer thickness `0.35`,
  min layer thickness `0.05`, and `checkMesh -skewThreshold 12` as the accepted
  development gate. It has `261,192` cells, `23,962` aircraft faces, average
  aircraft layers `2.26`, max skewness `8.9609095`, and max non-orthogonality
  `69.999388`.
- Do not start snappy kOmegaSST scoring-development runs from uniform
  freestream fields. Uniform-start kOmegaSST on `np12_l3_l4` completed but
  produced invalid coefficients and y-plus max above `8700`. Use a stable
  laminar no-slip run to initialize `U` and `p`, then run conservative
  kOmegaSST with lower turbulence initialization and `k/omega` relaxation.
- Current conservative snappy RANS setup that behaved normally on fcv04:
  `k = 0.02`, `omega = 50`, pRelax `0.1`, uRelax `0.2`, `k/omega` relaxation
  `0.2`, `nCorrectors = 2`, `nNonOrthogonalCorrectors = 5`, force axes drag
  `+X`, lift `+Z`, pitch `+Y`.
- Current snappy RANS caveat: coefficient stability is now plausible on fcv04,
  but it is still not validated scoring CFD. y-plus remains mixed; the
  thick-layer mesh has p50 about `11.3`, p95 about `42.7`, and max about `148`.
  Require y-plus, coefficient-window, continuity, residual, and screenshot
  evidence before using any no-slip result as more than development evidence.
- Gmsh laminar-start RANS status is split by mesh path. Fresh evidence root:
  `custom_cfd_mesher_experiment\runs\gmsh_hybrid_laminar_start_rans_probe_v0_1_20260624`.
  The hybrid exact-merge Gmsh `fcv04` mesh still cannot produce a stable
  laminar no-slip state: the fresh `fcv04_hybrid_laminar_p02_u03_100` case hit
  the `300 s` cap in the first step with `Ux` final residual `1714.4319` and
  pressure solver correction passes stuck at 1000 iterations. Do not attempt
  `kOmegaSST` on that hybrid mesh until laminar no-slip startup is fixed.
- The older Gmsh BL mesh does benefit from laminar-start turbulence:
  `fcv04_gmsh_bl_kOmegaSST_laminarStart_k002_om50_p01_u02_20` completed 20
  iterations with no FPE and final coefficients `Cd = 0.111670777`,
  `Cl = 0.0777210189`, `Cm = -0.0084807387`. Its y-plus is extremely low
  though: p50 about `0.039`, p95 about `0.140`, max about `4.84`. Treat this
  as low-y-plus/resolved-wall development evidence only, not a wall-function
  scoring default.
- Snappy thick-layer repeatability evidence root:
  `custom_cfd_mesher_experiment\runs\snappy_thick_layer_four_variant_v0_1_20260624`.
  Combined with the earlier `fcv04` thick-layer run, snappy now has
  five-variant evidence for mesh generation, `potentialFoam`, 100-step laminar
  no-slip, and 100-step laminar-start `kOmegaSST` without fatal errors or
  floating point exceptions. Treat snappy thick-layer as the leading
  scoring-CFD development backend.
- Current snappy thick-layer caveats still apply: strict OpenFOAM skewness
  fails on a small number of faces, relaxed `checkMesh -skewThreshold 12` is
  the current development gate, y-plus remains mixed, force/moment references
  are provisional, and coefficients are not validated scoring data.
- Coefficient gate caution: the current relative-drift gate can reject
  near-zero `Cm` even when `Cd` and `Cl` are stable. Before automated scoring
  acceptance, replace this with an absolute-plus-relative threshold policy.
- The force coefficient gate has now been updated in
  `custom_cfd_mesher_experiment/scripts/summarize_force_coeffs.py` to require
  both relative and absolute drift violations before failing a coefficient.
- Wall-function y-plus probe status:
  - Preferred high-fidelity preset:
    `custom_cfd_mesher_experiment/presets/snappy_openfoam_external_aero_hifi_surface_v0_2.json`.
  - Preset:
    `custom_cfd_mesher_experiment/presets/snappy_openfoam_external_aero_wall_function_candidate_v0_1.json`.
  - Best bounded fcv04 case:
    `custom_cfd_mesher_experiment/runs/snappy_wall_function_probe_1layer_v0_1_20260624`.
  - Report:
    `custom_cfd_mesher_experiment/runs/snappy_wall_function_probe_20260624_summary.md`.
  - The wall-function candidate is research-only. It improves y-plus behavior
    but uses a coarser surface mesh, dropping fcv04 aircraft faces from
    `23,962` to `9,583`.
  - Do not promote the coarser wall-function preset to default while surface
    fidelity is the priority.
  - A high-fidelity one-layer probe preserved the `23,962` fcv04 aircraft face
    count and completed the solver ladder, but y-plus stayed mixed
    (`p50=11.35`, `p95=43.23`, `max=164.49`).
  - Current default policy: preserve surface levels `3/4`; record y-plus as a
    warning/gate; do not auto-degrade surface capture to hit y-plus.
- Fresh five-variant high-fidelity comparison:
  - Previous preferred preset:
    `custom_cfd_mesher_experiment/presets/snappy_openfoam_external_aero_hifi_surface_v0_2.json`.
  - Fast one-layer alternative:
    `custom_cfd_mesher_experiment/presets/snappy_openfoam_external_aero_hifi_surface_fast_1layer_v0_2.json`.
  - Comparison report:
    `custom_cfd_mesher_experiment/runs/snappy_hifi_surface_layer_comparison_20260624.md`.
  - Both preserve aircraft patch face counts and complete mesh/potential/
    laminar/kOmegaSST gates on all five variants.
  - One-layer is faster and lighter but does not solve y-plus; keep three-layer
    as the historical v0.2 comparison baseline.
- Previous high-fidelity snappy preset:
  - Preset:
    `aircraft_optimizer_platform/software/optimizer/configs/snappy_openfoam_external_aero_hifi_surface.v0_3.json`.
  - Evidence root:
    `custom_cfd_mesher_experiment/runs/snappy_hifi_surface_v0_3_five_variant_20260624`.
  - Persisted optimizer workspace:
    `aircraft_optimizer_platform/runs/snappy_hifi_openfoam_persistence_v0_3`.
  - Summary CSV:
    `custom_cfd_mesher_experiment/runs/snappy_hifi_surface_v0_3_five_variant_20260624/v0_3_summary_metrics.csv`.
  - v0.3 uses surface levels `4/5`, feature level `5`, automatic feature
    refinement boxes, three relative surface layers, and 12-core snappy.
  - All five faired-cap variants pass the development mesh gate, potentialFoam,
    100-step laminar no-slip, and 100-step laminar-start kOmegaSST.
  - Aircraft patch face counts are `58.7k-69.8k`; bidirectional source-to-patch
    p95 deviation is `0.09-0.13 mm`.
  - Optimizer acceptance mode:
    `snappy_hifi_development`.
  - Import command:
    `python -m aircraft_optimizer.cli.main ingest-snappy-hifi-run --workspace ..\..\runs\snappy_hifi_openfoam_persistence_v0_3 --run-root ..\..\..\custom_cfd_mesher_experiment\runs\snappy_hifi_surface_v0_3_five_variant_20260624`.
  - Imported workspace status: five candidates, five complete evaluations,
    five successful surface-fidelity attempts, five successful OpenFOAM smoke
    attempts, five successful OpenFOAM steady attempts, seventy artifacts,
    zero failures, campaign complete.
  - The importer persists one `openfoam_yplus_field` artifact per steady run.
    Current y-plus ranges are p50 `9.18-11.23`, p95 `23.59-23.88`, and max
    `79.84-118.11`.
  - The steady importer uses `snappy_hifi_coefficient_development`, which
    requires completed solver execution, no fatal errors, force coefficients,
    y-plus availability, local continuity <= `1e-4`, final velocity residual
    <= `1e-4`, Cd/Cl/Cm final-window spans <= `0.005`, y-plus p95 <= `60`, and
    y-plus max <= `250`.
  - y-plus remains below wall-function target. Treat coefficients as stable
    development evidence only, not validated scoring CFD.
  - First bounded fcv04 y-plus layer study:
    `custom_cfd_mesher_experiment/runs/snappy_yplus_study_fcv04_v0_2_20260625`.
    Two probes with thicker relative layers both completed, but neither moved
    median y-plus toward the wall-function band:
    `rel_3layer_0p55_0p08` reached p50 `7.20`, p95 `35.70`, max `95.36`;
    `rel_1layer_0p75_0p12` reached p50 `6.45`, p95 `24.61`, max `84.92`.
    At the time, this supported keeping the surface-fidelity preset as the
    default development path and not coarsening/degrading the surface only to
    chase wall-function y-plus.
  - Resolved-wall absolute-layer probe:
    `custom_cfd_mesher_experiment/runs/snappy_resolved_yplus_fcv04_v0_1_20260625`.
    The absolute four-layer mesh preserved the high-fidelity surface settings
    and passed strict mesh/potential gates, but the kOmegaSST ladder ran away.
    A solver-only low-Re wall-treatment retry at
    `custom_cfd_mesher_experiment/runs/snappy_resolved_lowre_solver_retry_fcv04_v0_1_20260625`
    also ran away. Do not promote ultra-thin absolute layers without a new
    bounded solver-startup strategy and successful five-variant evidence.
  - `setup_incompressible_fluid_smoke.py` supports `--wall-treatment
    wall_function|low_re`. Use `low_re` only for explicit resolved-wall
    experiments; it is not currently a proven scoring path.
  - Superseded by v0.4 on 2026-06-25. Keep this entry as historical evidence,
    not as the default backend.

- Current preferred high-fidelity snappy preset:
  - Preset:
    `aircraft_optimizer_platform/software/optimizer/configs/snappy_openfoam_external_aero_hifi_surface.v0_4.json`.
  - Evidence root:
    `custom_cfd_mesher_experiment/runs/snappy_hifi_1layer_all_variants_candidate_20260625`.
  - Persisted optimizer workspace:
    `aircraft_optimizer_platform/runs/snappy_hifi_openfoam_persistence_v0_4`.
  - Summary artifacts:
    `custom_cfd_mesher_experiment/runs/snappy_hifi_1layer_all_variants_candidate_20260625/one_layer_candidate_summary.md`.
    `custom_cfd_mesher_experiment/runs/snappy_hifi_1layer_all_variants_candidate_20260625/one_layer_candidate_metrics.csv`.
  - v0.4 uses surface levels `4/5`, feature level `5`, automatic feature
    refinement boxes, one relative surface layer, and 12-core snappy.
  - All five faired-cap variants pass strict development mesh gates,
    `potentialFoam`, 100-step laminar no-slip, and 100-step laminar-start
    `kOmegaSST`.
  - Aircraft patch face counts are `58.7k-69.7k`; bidirectional
    source-to-patch p95 deviation is `0.09-0.13 mm`.
  - Current y-plus ranges are p50 `5.97-6.66`, p95 `20.60-25.50`, and max
    `77.98-120.84`.
  - Imported workspace status: five candidates, five complete evaluations,
    five successful surface-fidelity attempts, five successful OpenFOAM smoke
    attempts, five successful OpenFOAM steady attempts, seventy artifacts,
    zero failures, campaign complete.
  - This is still non-scoring development CFD. y-plus remains below the
    wall-function target band and no mesh-convergence or validation study has
    been completed.
  - Use v0.4 for edge-fidelity-sensitive optimizer meshing unless a later
    entry explicitly supersedes it.
  - The rough-scoring CFD path now has five expanded raw direct-STL baseline
    sweeps. Use those summaries as the adaptive promotion baseline before
    running new candidate shakedowns:
    - `custom_cfd_mesher_experiment/runs/raw_direct_faired_cap_snappy_ranking_v0_1_three_variant_20260625/alpha_sweep_summary_survivor_end60_np8.json`.
    - `custom_cfd_mesher_experiment/runs/raw_direct_faired_cap_snappy_baseline_fcv01_v0_1_20260626/alpha_sweep_summary_survivor_end60_np8.json`.
    - `custom_cfd_mesher_experiment/runs/raw_direct_faired_cap_snappy_baseline_fcv05_v0_1_20260626/alpha_sweep_summary_survivor_end60_np8.json`.
  - The current adaptive first-pass clear-reject bar is
    `CL(+4 deg) <= 0.1500544374`, based on best baseline `CL(+4 deg) =
    0.428726964` and the configured 65% lift-loss limit.
  - Before launching the first 15-candidate shakedown, follow
    `software/optimizer/configs/rough_cfd_15_candidate_shakedown.v0_1.json`.
    It is a campaign boundary contract, not a final-CFD validation standard.
  - The current 10 new wing-focused shakedown candidates are:
    - `software/optimizer/configs/cfd_wing_shakedown_10_variants.v0_1.json`.
    - `software/optimizer/configs/raw_direct_wing_shakedown_10_stl_batch.v0_1.json`.
    - `software/optimizer/configs/wing_shakedown_10_export_manifest.v0_1.json`.
  - `run-snappy-openfoam-backend` supports `--input-stl-map` for batch STL
    runs and `--variant-config` so imported candidates retain their wing
    parameter metadata.
  - When passing negative alpha lists through backend code, use
    `--alphas-deg=<comma-separated-values>` so argparse does not parse the
    leading negative value as an option.
  - Latest expanded-variable real-export mesh-only checkpoint:
    `custom_cfd_mesher_experiment/runs/real_export_expanded_vars_snappy_mesh_only_20260627`.
    It uses `aircraft_optimizer_platform/runs/real_export_expanded_vars_abs_20260627/mesh_input_stl_map.json`
    and `--skip-solver` on the current snappy path. The three meshes completed
    with `pass_for_mesh_only`; no `potentialFoam`, RANS, or scoring CFD was run.
    Cell counts are `883,971`, `928,611`, and `1,015,461`; max
    non-orthogonality is just under `70 deg`; bidirectional p95 surface
    deviation is `0.313-1.031 mm`. Use this only as export-to-mesh plumbing and
    visual-review evidence.
  - The lightweight STL preview renderers have been corrected for upright ISO
    screenshots. Corrected previews for the expanded real exports are under
    `aircraft_optimizer_platform/runs/real_export_expanded_vars_abs_20260627/export_screenshots_corrected`.
  - Current raw expanded-STL mesh-only finding:
    automatic feature-refinement boxes are not the preferred default for these
    raw no-inlet exports. A constrained feature-zone rerun passed mesh-only but
    added cells and introduced a small variant-01 patch-topology warning. The
    corrected tip-seed-only probe passed all three variants, cut cells by
    `14-24%`, and improved p95 source-to-patch deviation by `63-85%`.
    Candidate preset:
    `software/optimizer/configs/snappy_openfoam_external_aero_ranking.v0_2_tip_only_mesh_candidate.json`.
    Evidence:
    `custom_cfd_mesher_experiment/runs/real_export_expanded_vars_snappy_mesh_only_tip_only_probe_20260627`,
    `custom_cfd_mesher_experiment/runs/real_export_expanded_vars_snappy_mesh_only_tip_only_remaining_20260627`,
    and combined comparison
    `custom_cfd_mesher_experiment/runs/real_export_expanded_vars_snappy_tip_only_measure_comparison_20260627.json`.
    Follow-up solver evidence:
    `custom_cfd_mesher_experiment/runs/real_export_expanded_vars_snappy_tip_only_potential_smoke_20260627/potential_smoke_summary.json`
    and
    `custom_cfd_mesher_experiment/runs/real_export_expanded_vars_snappy_tip_only_solver_summary_20260627.json`.
    `potentialFoam -writep` completed `3/3` with no warnings. The bounded
    60-step no-slip ladder completed `3/3` laminar and `3/3`
    laminar-start/kOmegaSST runs. The short kOmegaSST force tails passed
    stability gates on all three variants, but y+ remains below the
    wall-function target: median y+ is about `4.3-4.4`, p95 about
    `8.9-10.2`, and `0/3` pass the y+ wall-function gate. Treat this as
    rough-scoring development evidence, not final scoring CFD.

## Environment Discipline

Follow `environment_strategy.md`.

## Current Mesher Decision

- Use `snappyHexMesh` for all current optimizer candidate meshing, rough-CFD
  scoring development, and OpenFOAM solver runs.
- Treat Gmsh as historical plumbing/comparison evidence unless the user
  explicitly asks for a Gmsh rerun or a later evidence-backed decision
  supersedes this policy.
- Current mirrored-mesh work should stay OpenFOAM-native: half-domain
  `blockMesh`/`snappyHexMesh`, then `mirrorMesh`, with quality and surface
  fidelity checked against the direct full-snappy case.

Current v0.1 environment:

- Python 3.12.
- Local virtual environment or `uv`.
- SQLite.
- File-backed artifacts.
- pytest.
- WSL micromamba CFD tools are optional and detected by `check-cfd-tools`; the fixture must remain independent of those external tools.

Do not add Docker, Docker Compose, PostgreSQL, dashboard services, real CFD tools, or real CAD/exporter subprocesses to the v0.1 skeleton.

## Progress Tracker Format

When updating `progress_tracker.md`, add concise entries with:

- Date.
- What changed.
- Files touched.
- Validation performed.
- Open questions or blockers.
- Next action.

Prefer append-style updates. If correcting an earlier decision, add a new note explaining the correction.

## Architecture Priorities

Highest priorities:

- Reproducibility.
- Traceability.
- Candidate lifecycle clarity.
- Replaceable modules.
- Failure preservation.
- Artifact provenance.
- Versioned scoring and analysis.
- Future ML/MDAO readiness.

Avoid:

- Hidden state.
- Dashboard-only state.
- Overwriting results.
- Tool-specific assumptions leaking into core records.
- Using filenames as primary identity.
- Coupling optimizer logic directly to CFD or CAD internals.

## Expected Documentation Updates

For any future implementation task:

1. Update `progress_tracker.md` before or after the work with the intent.
2. Make the change.
3. Record validation results.
4. Record any unresolved risks.

For any design-only task:

1. Update the relevant design document.
2. Add a short progress tracker note.
3. Keep the roadmap aligned if phase sequencing changes.
