# Curated Rhai Feature Map

Date created: 2026-06-20

## Purpose

This file records the optimizer-relevant feature names and rough roles for the small curated Rhai reference set.

These files are reference geometry inputs, not the full source history. Broader history remains in the original source folders.

## Curated Files

| File | Original source | Returned feature | Role | Notes |
|---|---|---|---|---|
| `scratchpad_plane.rhai` | `dual_contouring\scratchpad_plane.rhai` | conditional `combined` / cutaway expression | Scratchpad aircraft reference | Useful for understanding current hand-authored aircraft modeling; not the current direct sparse exporter baseline. |
| `direct_sparse_oml_aircraft_no_inlet.rhai` | `dual_contouring\direct_sparse_sdf_mc_experiment\scratch\aircraft_oml_oblique15_y_slice_frame.rhai` | `aircraft_oml_native_mc` | Current canonical no-inlet OML exporter reference | Returns native `body_outer` for marching-cubes export; legacy oblique/y-slice features remain diagnostic only. |
| `direct_sparse_oml_inlet_native_frame.rhai` | `dual_contouring\direct_sparse_sdf_mc_experiment\scratch\aircraft_oml_inlet_native_frame.rhai` | `aircraft_oml_inlet_native_frame` | Inlet OML reference | Native scratchpad frame: X longitudinal, Y spanwise, Z vertical. |
| `direct_sparse_oml_twin_side_inlets_native_frame.rhai` | `dual_contouring\direct_sparse_sdf_mc_experiment\scratch\aircraft_oml_twin_side_inlets_native_frame.rhai` | `aircraft_oml_twin_side_inlets_native_frame` | Twin-side-inlet OML reference | Native scratchpad frame. |
| `direct_sparse_oml_inlet_wingtips_full_h115_inner7_outer12p8_native_frame.rhai` | `dual_contouring\direct_sparse_sdf_mc_experiment\scratch\aircraft_oml_inlet_wingtips_full_h115_inner7_outer12p8_native_frame.rhai` | `aircraft_oml_inlet_wingtips_native_frame` | Inlet + wingtip OML reference | Native scratchpad frame. |
| `parked_shell_no_internal_oblique15_y_slice_frame.rhai` | `dual_contouring\direct_sparse_sdf_mc_experiment\scratch\aircraft_shell_no_internal_oblique15_y_slice_frame.rhai` | `aircraft_shell_no_internal_oblique15_y_slice_frame` | Parked shell/manufacturing reference | Not default optimizer geometry; shell export remains parked due unresolved topology defects. |

## Known Bounds From Embedded Metadata

### No-Inlet Direct Sparse OML

```text
feature = aircraft_oml_native_mc
role = outer_mold_line
min = [-128.0, -512.0, -128.0]
max = [1024.0, 512.0, 256.0]
```

### Native Inlet OML References

Primary OML part:

```text
name = aircraft_oml
patch_name = aircraft
role = outer_mold_line
min = [-20.0, -360.0, -40.0]
max = [740.0, 360.0, 120.0]
```

Duct internal:

```text
name = duct_internal
patch_name = duct
role = flow_path_internal
min = [240.0, -32.0, -24.0]
max = [740.0, 32.0, 92.0]
```

Internal structure:

```text
name = internal_structure
patch_name = structure
role = internal_structure
min = [40.0, -320.0, -30.0]
max = [620.0, 320.0, 90.0]
```

## Optimizer Use

Recommended default optimizer/export reference:

```text
direct_sparse_oml_aircraft_no_inlet.rhai
feature = aircraft_oml_native_mc
```

Recommended inlet exploration references:

```text
direct_sparse_oml_inlet_native_frame.rhai
direct_sparse_oml_twin_side_inlets_native_frame.rhai
direct_sparse_oml_inlet_wingtips_full_h115_inner7_outer12p8_native_frame.rhai
```

Do not use `parked_shell_no_internal_oblique15_y_slice_frame.rhai` in the default optimizer export flow.
