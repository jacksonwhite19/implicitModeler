# OML Export Contract

Date: 2026-06-20

This document defines the current CFD/optimizer STL export contract for the aircraft optimizer platform.

The implementation currently lives in the existing implicit CAD repository under:

```text
C:\Users\Jackson\Desktop\02_Projects\09b_Implicit_CAD_claude\dual_contouring\direct_sparse_sdf_mc_experiment
```

This is an export adapter contract, not an optimizer implementation. The optimizer platform does not own geometry generation or marching-cubes internals. It should request an OML SDF from the geometry provider, call this export path, validate the output, and preserve the result as candidate evidence.

## Position

Use direct sparse SDF marching cubes as the default OML/CFD STL export path.

Default behavior:

- Use the fast preset for normal optimizer iterations.
- Use the smaller-cell high-fidelity preset only when a few candidate designs are converging or when final CFD preparation needs a denser surface.
- Do not use shell/manufacturing export as part of the optimizer default path yet.
- Treat geometry feature construction and STL export as separate functions.

## Default Preset

Preset name:

```text
direct_sparse_oml_fast
```

Use this as the default optimizer OML export path.

Settings:

- Feature: `aircraft_oml_native_mc`
- Coordinate frame: native aircraft frame, `X = length`, `Y = span`, `Z = vertical`
- `spacing_mm = 1.0`
- `tile_size_mm = 16`
- `coarse_spacing_mm = 4`
- `narrow_band_threshold_mm = 3`
- `tile_dilation = 1`
- SDF sidecar: `target/release/batch_sdf_distance_only.exe`
- SDF workers: `8`
- For symmetric no-inlet aircraft, optimizer runs should export the positive
  `Y` half and mirror/weld across `Y=0` using the wrapper
  `--symmetry-half-mirror` mode. Full native export remains the fallback/debug
  path.

Expected baseline runtime:

```text
about 3.4 minutes for half-export + mirror on the reference no-inlet candidate
```

Known baseline result:

- Boundary edges: `0`
- Nonmanifold edges: `0`
- Connected components: `1`
- Duplicate triangles: `0`
- Known-section long chord count: `0`

Command:

```powershell
python direct_sparse_sdf_mc_experiment\scripts\run_oml_fast_export.py
```

## High-Fidelity Preset

Preset name:

```text
direct_sparse_oml_high_fidelity
```

Use this only for converged or promising designs where the extra runtime is justified.

Settings:

- Feature: `aircraft_oml_native_mc`
- Coordinate frame: native aircraft frame, `X = length`, `Y = span`, `Z = vertical`
- `spacing_mm = 0.75`
- `tile_size_mm = 24`
- `coarse_spacing_mm = 4`
- `narrow_band_threshold_mm = 3`
- `tile_dilation = 1`
- SDF sidecar: `target/release/batch_sdf_distance_only.exe`
- SDF workers: `8`

Expected baseline runtime:

```text
about 10 minutes
```

Known baseline result:

- Boundary edges: `0`
- Nonmanifold edges: `0`
- Connected components: `1`
- Duplicate triangles: `0`
- Known-section long chord count: `0`

Command:

```powershell
python direct_sparse_sdf_mc_experiment\scripts\run_oml_high_fidelity_export.py
```

## Optimizer-Facing Wrapper

The optimizer-facing wrapper is:

```text
direct_sparse_sdf_mc_experiment\scripts\optimizer_export_presets.py
```

It provides two separate surfaces:

Geometry/export preset definitions:

- `direct_sparse_oml_fast_preset(...)`
- `direct_sparse_oml_high_fidelity_preset(...)`

Execution and validation:

- `run_optimizer_oml_export(preset, limits, ...)`
- `MeshQualityLimits(...)`

The optimizer should create or receive a geometry-provider output, map it into a preset, then pass the preset and quality limits to the runner.

## Quality Gates

Default CFD-ready gate:

- `max_boundary_edges = 0`
- `max_nonmanifold_edges = 0`
- `max_connected_components = 1`
- `max_duplicate_triangles = 0`
- `max_long_chord_sections_ge_75mm = 0`

Optional additional gates:

- `max_runtime_s`
- `max_high_aspect_count`
- `max_aspect_p99`
- `max_p99_edge_length`
- `max_edge_length`

The optimizer may relax topology limits for intentionally strange geometry stress tests, but production CFD candidate exports should use the strict gate.

## Machine-Readable Result

Each export should write a JSON result with:

- preset name
- command used
- preset definition
- quality limits
- status
- pass/fail result
- gate failures
- STL path
- runtime metrics
- SDF sample count
- active tile count
- vertices and triangles
- topology metrics
- known-section chord audit summary
- stdout/stderr log paths

This result JSON should be registered as a candidate artifact in the future optimizer database.

## Geometry Provider Boundary

The optimizer should not directly build aircraft geometry. The geometry provider should supply one of:

- an SDF feature name in an existing Rhai script
- a generated Rhai script path plus feature name
- optional native-frame bbox metadata for geometry variants

## Inlet Export Modes

For early optimizer CFD, inlet geometry should use explicit export modes:

- `inlet_mode = "faired_cap"`: default optimizer CFD mode. This should be a
  purpose-built solid optimizer inlet geometry, not a post-processed or clipped
  flow-through inlet. The generated SDF should union a coherent solid external
  inlet body into the fuselage and preserve the full buried inlet body/path
  rather than trimming it at first fuselage contact, because the visible portion
  affects aircraft-level drag and buried solid geometry is harmless in an OML
  union. It should not create an internal duct void, internal duct walls,
  fan-face features, or any outlet/exhaust cut. The inlet's forward X-facing
  mouth should be closed with a smooth shallow fairing/cap. The outlet should
  not be cut at all for this mode; the solid inlet path should terminate inside
  the fuselage before the aft exit, with no visible exhaust cap or exit feature.
  The result should be a single watertight external OML shell when topology
  gates pass.
- `inlet_mode = "flow_through"`: preserve actual inlet/duct/opening geometry
  for later detailed inlet CFD. This mode is not the default aircraft-level
  optimizer CFD mode.
- `inlet_mode = "none"`: clean no-inlet aircraft geometry when supported by
  the source aircraft definition.

Current generated inlet-mode scripts are produced by:

```powershell
python direct_sparse_sdf_mc_experiment\scripts\optimizer_export_presets.py --preset direct_sparse_oml_fast --inlet-mode faired_cap
```

`faired_cap` exports should record inlet-mouth cap metadata and diagnostics,
including cap triangle counts, cap bounding boxes, max cap height, watertight
status, boundary edges, nonmanifold edges, connected components, and duplicate
triangles. Flow-through exports should remain available as a separate evidence
path and should not be silently converted to faired caps.

The exporter should then produce:

- OML STL
- validation metrics
- result JSON
- logs
- optional section plots

This keeps geometry definition and export validation separate.

## Bbox Policy

Baseline OML may use the existing default bbox path.

Geometry variants that extend outside the baseline contour-derived bbox should provide explicit native-frame bounds:

- side inlets
- larger wingtips or fins
- external stores or pods
- unusual tail surfaces
- any feature outside the baseline aircraft envelope

Example:

```powershell
python direct_sparse_sdf_mc_experiment\scripts\optimizer_export_presets.py --preset direct_sparse_oml_fast --bbox-min=-32,-430,-140 --bbox-max=720,430,220
```

Future geometry-provider metadata should include a conservative export bbox so the optimizer does not need to infer it from old contour sources.

## Tile Compatibility Rule

For clean tile seams:

```text
tile_size_mm / spacing_mm = integer
```

Validated combinations:

- `16 / 1.0 = 16`
- `24 / 0.75 = 32`

Do not add optimizer presets that violate this rule.

## Candidate Record Integration

When the optimizer platform exists, each candidate export should record:

- `candidate_id`
- `evaluation_id`
- preset name
- feature name
- SDF script path
- geometry input hash
- geometry provider version
- exporter script version/hash
- SDF sidecar path/version/hash
- bbox used
- quality limits used
- result JSON path
- STL path
- pass/fail status
- gate failures
- runtime and topology metrics

Failed exports should be preserved, not discarded.

## Current Recommendation

For optimizer development:

1. Make `direct_sparse_oml_fast` the default CFD STL export for candidate evaluation.
2. Add `direct_sparse_oml_high_fidelity` as an explicit rerun/finalization option.
3. Require strict CFD-ready gates for production candidate exports.
4. Allow relaxed gates only for geometry stress tests or exploratory feature development.
5. Keep shell/manufacturing export out of the default optimizer flow.
6. Build the future optimizer around the result JSON as the export evidence contract.
