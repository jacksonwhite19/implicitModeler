# Manual SDF Generation

Purpose: curated human-authored SDF/Rhai aircraft definitions used as reference geometry, baselines, validation inputs, and family templates.

Current source candidates:

- `examples\complete_aircraft.rhai`
- `examples\aircraft_with_nacelles.rhai`
- `examples\aircraft_smooth_blend.rhai`
- `dual_contouring\examples\complete_aircraft.rhai`
- selected files from `dual_contouring\direct_sparse_sdf_mc_experiment\scratch`

Import requirements:

- Preserve original path and hash.
- Record feature name.
- Record coordinate frame.
- Record bbox assumptions.
- Record validation/export status.
- Separate canonical examples from scratch diagnostics.

## Current Curated Set

Current destination:

```text
curated_rhai
```

Current count: 6 Rhai files.

Copied references:

- `FEATURE_MAP.md`
- `scratchpad_plane.rhai`
- `direct_sparse_oml_aircraft_no_inlet.rhai`
- `direct_sparse_oml_inlet_native_frame.rhai`
- `direct_sparse_oml_twin_side_inlets_native_frame.rhai`
- `direct_sparse_oml_inlet_wingtips_full_h115_inner7_outer12p8_native_frame.rhai`
- `parked_shell_no_internal_oblique15_y_slice_frame.rhai`

Not copied by default:

- broad examples/component demos
- root temporary/debug scripts
- component-section scratch files
- generated z-slice diagnostic/log/sweep Rhai copies under `dual_contouring\test_scripts\z_slice_export_experiment`

Status: curated references copied; initial feature map created.
