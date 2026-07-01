# Dependency Inventory

Date created: 2026-06-20

## Purpose

This file is the master inventory of external dependencies, source folders, and platform inputs that `aircraft_optimizer_platform` depends on or may eventually vendor.

Use this alongside `master_platform_manifest.md`:

- `master_platform_manifest.md` describes the target master-folder organization and migration policy.
- `dependency_inventory.md` tracks what exists now, where it lives, what was copied, what was skipped, and what contracts are still needed.

## Current Inventory Status

| Area | Status | Current action |
|---|---|---|
| Manual Rhai/SDF inputs | Curated copy started | 6 Rhai references copied into `software/sdf_generation_manual/curated_rhai`. |
| Automatic Rhai/SDF generation scripts | Reference only | Existing `make_*.py` scripts are examples/references, not production candidates; production contract is under `software\sdf_generation_auto`. |
| OML STL exporter | Single no-inlet execution validated | Current implementation remains in `dual_contouring\direct_sparse_sdf_mc_experiment`; platform adapter contract is under `software\exporters\oml_stl`; disabled preflight and one-shot real export validation live in optimizer helpers. |
| Core implicit CAD/SDF Rust library | External dependency | Not moved; needs dependency boundary map. |
| Optimizer examples | External/reference | Not reviewed yet. |
| Optimizer interface contracts | Started | Contracts added under `software/optimizer/contracts`, including persistence, events, and failures. |
| v0.1 optimizer skeleton | Specified | First runnable skeleton specified in `software/optimizer/v0_1_skeleton_spec.md`. |
| Fixed-wing variable schema | Started | Initial v0.1 schema lives at `software\optimizer\schemas\fixed_wing_uav_reference.variables.v0_1.json`. |
| Dashboard | Not started | No dependency inventory yet. |
| Analysis tools | Planning only | Open-source tool recommendations exist in `opt_output.md`; implementation dependencies not selected yet. |
| SDF conditioning research tools | Reference only | Phase 0 review covers OpenVDB/NanoVDB, scikit-fmm, Voxblox/nvblox, Lethe, AMReX EB, Basilisk EB, and PhysicsNeMo; no production dependency has been adopted. |
| CFD/mesh tool environment | Installed externally | WSL user-local `aop-cfd` micromamba env installed with SU2 8.5.0, Gmsh 4.15.2, and meshio; documented under `tools\cfd`. |

## Manual Rhai/SDF Inputs

### Curated Copy In Master Folder

Destination:

```text
aircraft_optimizer_platform\software\sdf_generation_manual\curated_rhai
```

Copied references:

| Master copy | Original source | Status | Why included |
|---|---|---|---|
| `scratchpad_plane.rhai` | `dual_contouring\scratchpad_plane.rhai` | Reference | Current scratchpad aircraft/plane source used in the CAD repo. |
| `direct_sparse_oml_aircraft_no_inlet.rhai` | `dual_contouring\direct_sparse_sdf_mc_experiment\scratch\aircraft_oml_oblique15_y_slice_frame.rhai` | Canonical export reference | Current validated no-inlet OML model used by the direct sparse exporter. |
| `direct_sparse_oml_inlet_native_frame.rhai` | `dual_contouring\direct_sparse_sdf_mc_experiment\scratch\aircraft_oml_inlet_native_frame.rhai` | Inlet reference | Inlet-bearing OML source from the SDF export folder. |
| `direct_sparse_oml_twin_side_inlets_native_frame.rhai` | `dual_contouring\direct_sparse_sdf_mc_experiment\scratch\aircraft_oml_twin_side_inlets_native_frame.rhai` | Inlet reference | Twin-side-inlet aircraft source from the SDF export folder. |
| `direct_sparse_oml_inlet_wingtips_full_h115_inner7_outer12p8_native_frame.rhai` | `dual_contouring\direct_sparse_sdf_mc_experiment\scratch\aircraft_oml_inlet_wingtips_full_h115_inner7_outer12p8_native_frame.rhai` | Inlet/wingtip reference | More complete inlet + wingtip model from the SDF export folder. |
| `parked_shell_no_internal_oblique15_y_slice_frame.rhai` | `dual_contouring\direct_sparse_sdf_mc_experiment\scratch\aircraft_shell_no_internal_oblique15_y_slice_frame.rhai` | Parked shell reference | Kept only as shell/manufacturing context; not default optimizer geometry. |

The previous broad 90-file copy was pruned. The master folder should carry a few curated Rhai references and point back to source folders for the broader history.

Feature map:

```text
software\sdf_generation_manual\curated_rhai\FEATURE_MAP.md
```

### Intentionally Not Copied

Not copied:

```text
root tmp/debug Rhai files
root examples and component demo files not directly needed for aircraft optimization
dual_contouring\examples
dual_contouring\components
dual_contouring\export_work\scripts
dual_contouring\direct_sparse_sdf_mc_experiment\scratch\component_sections
dual_contouring\test_scripts\z_slice_export_experiment\...\logs\...
dual_contouring\test_scripts\z_slice_export_experiment\...\sweep_runs\...
dual_contouring\test_scripts\z_slice_export_experiment\...\candidates\...
```

Reason:

- The master folder should only carry a few high-value reference Rhai files.
- Broader history remains discoverable in the source tree.
- Specific files can be copied later if they become canonical inputs or useful regression references.

## SDF Conditioning Research Tools

Current review:

```text
aircraft_optimizer_platform\research\phase0_sdf_conditioning_literature_review.md
```

Status:

- Reference only.
- No new production runtime dependency has been adopted.
- OpenVDB/NanoVDB should be evaluated before building a custom sparse-volume
  tree.
- scikit-fmm may be useful for prototype fixtures or regression oracles.
- Voxblox/nvblox provide useful incremental block-update patterns, but their
  TSDF/robotics assumptions should not replace the analytic CAD kernel.
- Lethe, AMReX embedded boundaries, and Basilisk embedded boundaries remain
  SDF-native CFD research candidates.
- PhysicsNeMo remains deferred until optimizer automation, conditioning
  stability, CFD dataset schema, and data quality are ready.

## Automatic Rhai/SDF Generation Scripts

Current reference scripts:

```text
dual_contouring\direct_sparse_sdf_mc_experiment\scripts\make_complex_oml_inlet_rhai.py
dual_contouring\direct_sparse_sdf_mc_experiment\scripts\make_complex_oml_inlet_wingtips_rhai.py
dual_contouring\direct_sparse_sdf_mc_experiment\scripts\make_twin_side_inlet_aircraft_rhai.py
```

Status:

- Not copied.
- Reference/example only.
- Do not treat these as production geometry generators.
- Useful for understanding parameterization patterns and Rhai emission patterns.

If referenced later, capture:

- Script purpose.
- Inputs.
- Outputs.
- Generated feature name.
- Coordinate frame.
- Bbox assumptions.
- Coupling to exporter paths.
- Which assumptions are experiment-specific.

Production contract:

```text
aircraft_optimizer_platform\software\sdf_generation_auto\geometry_generator_contract.md
```

## OML STL Exporter

Current source:

```text
dual_contouring\direct_sparse_sdf_mc_experiment
```

Current contract:

```text
aircraft_optimizer_platform\oml_export_contract.md
aircraft_optimizer_platform\software\exporters\oml_stl\export_adapter_contract.md
```

Current implementation status:

- Referenced, not imported.
- Keep wrapped in place for now.
- Disabled-by-default real no-inlet boundary preflight added in `software\optimizer\src\aircraft_optimizer\external\real_adapter_boundary.py`.
- CLI command: `preflight-real-adapters`.
- One-shot real no-inlet execution added in `software\optimizer\src\aircraft_optimizer\modules\oml_stl_real_adapter.py` and `software\optimizer\src\aircraft_optimizer\optimizers\real_no_inlet_export.py`.
- CLI command: `run-real-no-inlet-export`.
- Validation status: one controlled `direct_sparse_oml_fast` no-inlet export passed strict topology gates and copied platform artifacts.

Known entrypoints:

```text
dual_contouring\direct_sparse_sdf_mc_experiment\scripts\run_oml_fast_export.py
dual_contouring\direct_sparse_sdf_mc_experiment\scripts\run_oml_high_fidelity_export.py
dual_contouring\direct_sparse_sdf_mc_experiment\scripts\optimizer_export_presets.py
dual_contouring\direct_sparse_sdf_mc_experiment\scripts\direct_sparse_sdf_marching_cubes_export.py
```

Default optimizer preset:

- `direct_sparse_oml_fast`

Selected-design high-fidelity preset:

- `direct_sparse_oml_high_fidelity`

Needed platform contract:

- Export adapter request schema. Defined in `software\exporters\oml_stl\export_adapter_contract.md`.
- Export adapter result schema. Defined in `software\exporters\oml_stl\export_adapter_contract.md`.
- Artifact registration fields. Defined in `software\exporters\oml_stl\export_adapter_contract.md`.
- Failure classification mapping. Defined in `software\exporters\oml_stl\export_adapter_contract.md`.
- Runtime dependency list.

Current preflight-resolved paths:

```text
geometry source: aircraft_optimizer_platform\software\sdf_generation_manual\curated_rhai\direct_sparse_oml_aircraft_no_inlet.rhai
exporter script: dual_contouring\direct_sparse_sdf_mc_experiment\scripts\optimizer_export_presets.py
SDF sidecar: dual_contouring\target\release\batch_sdf_distance_only.exe
feature: aircraft_oml_oblique15_y_slice_frame
preset: direct_sparse_oml_fast
```

## Core Implicit CAD/SDF Runtime

Current source candidates:

```text
src\sdf
src\export
components
shaders
Cargo.toml
Cargo.lock
dual_contouring\src
dual_contouring\Cargo.toml
dual_contouring\Cargo.lock
```

Status:

- External dependency for now.
- Do not move until the dependency boundary is mapped.

Needed inventory fields:

- Required Rust binaries.
- Required build command.
- Required runtime command(s).
- Which modules are needed for geometry evaluation versus export.
- Which modules are UI/demo/support only.
- Whether the platform should vendor this library or call it as a sibling dependency.

## Optimizer Seed Examples

Current source candidates:

```text
examples\optimizer_example.py
dual_contouring\examples\optimizer_example.py
```

Status:

- Not reviewed.
- Reference-only until proven otherwise.

Needed review:

- Does it already encode candidate variables?
- Does it call existing geometry generation directly?
- Does it contain useful objective/scoring logic?
- Does it violate the desired decoupling between optimizer, geometry, exporter, and scoring?

## Platform-Owned Variable Schemas

Current schema:

```text
aircraft_optimizer_platform\software\optimizer\schemas\fixed_wing_uav_reference.variables.v0_1.json
```

Status:

- Platform-owned.
- Initial fixed-wing UAV operational subset.
- Continuous variables only.
- Bounded and unit-tagged.
- Used by the v0.1 fixture before candidate persistence.

## Immediate Next Inventory Work

1. Decide whether the parked shell Rhai reference should stay copied or become source-only documentation.
2. Review exporter entrypoints and result JSON shape against the adapter contract.
3. Map Rust/Python runtime dependencies needed to run the exporter from a platform adapter.
4. Draft a fresh production geometry-generator contract instead of adopting the current `make_*.py` examples.
