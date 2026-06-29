# Master Platform Manifest

Date created: 2026-06-20

## Purpose

This file defines `aircraft_optimizer_platform` as the master folder for the aircraft design and optimization ecosystem.

The master folder should eventually be standalone: it should contain or vendor the SDF generation software, manual and automatic aircraft definition paths, STL export adapters, optimizer core, analysis modules, dashboard, schemas, artifacts, and integration documentation.

Until source files are intentionally moved or copied, this manifest records the current source-of-truth locations and the intended destination layout.

## Master Folder Role

`aircraft_optimizer_platform` is the long-term platform root.

It should own:

- Platform-level requirements and architecture.
- Roadmap and progress tracking.
- Component inventory.
- Integration contracts.
- Runtime schemas.
- Optimizer implementation.
- Dashboard implementation.
- Analysis module wrappers.
- Exporter adapter wrappers.
- Eventually, imported or vendored SDF geometry generation code.

It should not silently fork existing CAD/exporter code. Any imported code should preserve provenance and validation status.

## Current Source-of-Truth Map

| Component | Current source | Master-folder status | Notes |
|---|---|---|---|
| Platform architecture | `aircraft_optimizer_platform\opt_output.md` | In master folder | Current platform architecture report. |
| Roadmap | `aircraft_optimizer_platform\roadmap.md` | In master folder | Current phase roadmap and release milestones. |
| Progress tracker | `aircraft_optimizer_platform\progress_tracker.md` | In master folder | Running execution log. |
| Dependency inventory | `aircraft_optimizer_platform\dependency_inventory.md` | In master folder | Tracks source locations, copied inputs, skipped inputs, and dependency contracts still needed. |
| OML STL export contract | `aircraft_optimizer_platform\oml_export_contract.md` | In master folder | Contract for direct sparse SDF OML exporter. |
| OML STL export adapter contract | `aircraft_optimizer_platform\software\exporters\oml_stl\export_adapter_contract.md` | In master folder | Optimizer module request/result, artifact, metric, and failure mapping contract. |
| Direct sparse OML STL exporter | `dual_contouring\direct_sparse_sdf_mc_experiment` | Referenced, not imported | Validated exporter path; should be wrapped or imported deliberately. |
| Manual SDF/Rhai aircraft definitions | `dual_contouring\scratchpad_plane.rhai`, selected `dual_contouring\direct_sparse_sdf_mc_experiment\scratch\*.rhai`; broader Rhai history remains in source tree | Curated copy started | 6 references copied to `software\sdf_generation_manual\curated_rhai`; broad 90-file import was pruned. |
| Automatic SDF/Rhai generation scripts | `dual_contouring\direct_sparse_sdf_mc_experiment\scripts\make_*.py` and related generator scripts | Reference only, not imported | Existing scripts are examples/references, not production candidates. |
| Automatic geometry generator contract | `aircraft_optimizer_platform\software\sdf_generation_auto\geometry_generator_contract.md` | In master folder | Production contract for converting typed candidate variables to geometry-provider output. |
| Core implicit CAD/SDF library | root `src\sdf`, `src\export`, `components`, `shaders` | External source for now | Do not move until dependency boundaries are understood. |
| Optimizer prototype/example | `examples\optimizer_example.py`, `dual_contouring\examples\optimizer_example.py` | Referenced, not adopted | Needs review before becoming platform optimizer core. |
| Optimizer interface contracts | `aircraft_optimizer_platform\software\optimizer\contracts` | In master folder | Thin platform contracts for candidates, geometry provider output, evaluations, artifacts, and modules. |
| v0.1 optimizer skeleton spec | `aircraft_optimizer_platform\software\optimizer\v0_1_skeleton_spec.md` | In master folder | First runnable platform slice: SQLite, records, events, mock provider, mock module, artifacts. |
| Fixed-wing variable schema | `aircraft_optimizer_platform\software\optimizer\schemas\fixed_wing_uav_reference.variables.v0_1.json` | In master folder | Initial bounded v0.1 fixed-wing UAV variable schema. |
| SU2 CFD adapter contract | `aircraft_optimizer_platform\software\analysis_modules\cfd_su2_adapter_contract.md` | In master folder | Future CFD module contract; no solver integration yet. |
| Dashboard | None yet | Not started | Future standalone local web app. |
| Run database/schema | None yet | Not started | Next build target for v0.1 platform skeleton. |

## Intended Master Layout

The following layout is the target organization. Some folders currently contain only README placeholders until code is imported or created.

```text
aircraft_optimizer_platform/
  README.md
  AGENT_README.md
  progress_tracker.md
  roadmap.md
  opt_output.md
  opt_prompt.md
  oml_export_contract.md
  master_platform_manifest.md
  dependency_inventory.md

  software/
    README.md
    sdf_generation_manual/
      README.md
    sdf_generation_auto/
      README.md
      geometry_generator_contract.md
    exporters/
      README.md
      oml_stl/
        README.md
        export_adapter_contract.md
    optimizer/
      README.md
      schemas/
        fixed_wing_uav_reference.variables.v0_1.json
    dashboard/
      README.md
    analysis_modules/
      README.md
```

Future implementation folders can be added only when there is actual code, schema, or tests to put there.

## Component Boundaries

### Manual SDF Generation

Role:

- Human-authored Rhai or equivalent aircraft definitions.
- Reference aircraft families.
- Baseline fixed-wing UAV definitions.
- Geometry examples for debugging and validation.

Current reference examples:

- `examples\complete_aircraft.rhai`
- `examples\aircraft_with_nacelles.rhai`
- `examples\aircraft_smooth_blend.rhai`
- `dual_contouring\examples\complete_aircraft.rhai`
- `dual_contouring\direct_sparse_sdf_mc_experiment\scratch\*.rhai`

Current copied destination:

- `software\sdf_generation_manual\curated_rhai`

Curated copied set:

- `scratchpad_plane.rhai`
- `direct_sparse_oml_aircraft_no_inlet.rhai`
- `direct_sparse_oml_inlet_native_frame.rhai`
- `direct_sparse_oml_twin_side_inlets_native_frame.rhai`
- `direct_sparse_oml_inlet_wingtips_full_h115_inner7_outer12p8_native_frame.rhai`
- `parked_shell_no_internal_oblique15_y_slice_frame.rhai`

Migration requirement:

- Separate canonical examples from scratch/debug files.
- Preserve original file paths and validation status when importing.
- Capture feature names, coordinate frame, bbox assumptions, and validation status.

### Automatic SDF Generation

Role:

- Generate parameterized aircraft definitions from candidate variables.
- Translate optimizer design vectors into geometry-provider inputs.
- Eventually support fixed-wing, VTOL, and tailsitter aircraft-family schemas.

Current source examples:

- `dual_contouring\direct_sparse_sdf_mc_experiment\scripts\make_complex_oml_inlet_rhai.py`
- `dual_contouring\direct_sparse_sdf_mc_experiment\scripts\make_complex_oml_inlet_wingtips_rhai.py`
- `dual_contouring\direct_sparse_sdf_mc_experiment\scripts\make_twin_side_inlet_aircraft_rhai.py`

Platform requirement:

- Do not adopt the current scripts as production generators.
- Use them only to understand prior parameterization and Rhai emission patterns.
- Design a fresh production geometry-generator contract around typed aircraft definitions.
- Preserve generated Rhai script path, feature name, bbox, and generator version/hash when production generation exists.
- Current contract: `software\sdf_generation_auto\geometry_generator_contract.md`

### STL Exporter

Role:

- Convert geometry-provider output into optimizer/CFD-ready OML STL artifacts.
- Preserve export metrics, quality gates, result JSON, logs, and artifact references.

Current validated path:

- `dual_contouring\direct_sparse_sdf_mc_experiment`
- Default preset: `direct_sparse_oml_fast`
- High-fidelity preset: `direct_sparse_oml_high_fidelity`
- Export contract: `oml_export_contract.md`
- Adapter contract: `software\exporters\oml_stl\export_adapter_contract.md`

Migration requirement:

- Keep exporter separate from geometry definition.
- Wrap the existing exporter through a platform adapter before moving code.
- Keep shell/manufacturing export out of the default optimizer flow until shell topology defects are resolved.

### Optimizer

Role:

- Campaign setup.
- Candidate generation.
- Candidate registry.
- Evaluation orchestration.
- Scoring.
- Lineage.
- Analysis and export module execution.

Current source examples:

- `examples\optimizer_example.py`
- `dual_contouring\examples\optimizer_example.py`

Migration requirement:

- Review before adopting.
- Build v0.1 around records and interfaces first.
- Avoid coupling optimizer directly to CAD internals or exporter script internals.

### Dashboard

Role:

- Local standalone web app for campaign monitoring, candidate browsing, ranking, convergence plots, artifacts, failure analysis, and annotations.

Current status:

- Not started.

Migration requirement:

- Dashboard must read from platform database/artifact store.
- Dashboard state must not become the record of truth.

## Import Policy

Before importing any existing code into the master folder:

1. Identify the current source path.
2. Identify whether the file is canonical, experimental, scratch, or obsolete.
3. Record the original path and current validation status.
4. Decide whether to copy, move, vendor, wrap in place, or leave external.
5. Update this manifest and `progress_tracker.md`.
6. Do not delete or overwrite the original source unless explicitly requested.

Recommended default:

- Wrap existing working code in place first.
- Import/copy code only after the interface is stable.
- Move code only after dependent scripts and paths are audited.

## Immediate Integration Work

Next master-folder tasks:

1. Create a component inventory for manual SDF generation inputs.
2. Create a component inventory for automatic SDF generation scripts.
3. Review optimizer example files and decide whether they are reference-only or seed code.
4. Define the geometry provider record schema.
5. Review exporter result JSON shape against the OML export adapter contract.
6. Define the v0.1 package layout for optimizer implementation.
7. Decide whether `aircraft_optimizer_platform` will vendor the CAD/SDF core or depend on it as a sibling package during early development.

## Current Architectural Position

The master folder should become standalone, but not by unstructured copying. The first step is a controlled manifest and optimizer-centered contract boundary. The second step is a thin platform skeleton that can call geometry providers, exporters, and analysis modules by contract. The third step is deliberate import or vendoring of the stable pieces.

The exporter is a downstream module, not the platform root.
