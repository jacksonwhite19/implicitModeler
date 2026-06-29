# snappyHexMesh Evidence Automation Contract

Status: v0.1 automation contract.

Purpose: define the reusable evidence bundle produced by the snappyHexMesh
meshing path. This is infrastructure for comparing and integrating the mesher;
it is not a scoring-CFD acceptance standard.

## Run Modes

The snappy comparison harness supports three useful modes:

- Full run: export, surface prep, snappyHexMesh, `potentialFoam`, screenshots,
  geometry-capture metrics, reports, and summary.
- Report-only refresh: regenerate reports from an existing completed case
  without export, meshing, rendering, or solver execution.
- Input generation only: generate variant Rhai files without launching the
  exporter or mesher.

Full comparison runs should be paused whenever the aircraft geometry family is
changing. Report-only refresh is allowed for validating evidence schemas on old
reference cases, but those refreshed cases must not be promoted to acceptance
evidence for a new geometry family.

## Required Output Files

Each variant folder should contain these stable files, even on failure when
enough context exists:

- `run_manifest.json`
- `evidence_bundle.json`
- `evidence_report.md`
- `mesh_quality_report.json`
- `geometry_capture_report.json`
- `solver_smoke_report.json`
- `raw_export_result.json`, when export was attempted
- `raw_surface_quality.json`, when a source STL exists
- `auto_repair_report.json`, when the snappy repair process reached report
  writing
- `failure_traceback.txt`, when the harness catches an exception

The run root should contain:

- `snappy_mesh_comparison_summary.json`
- `snappy_mesh_comparison_summary.md`

The summary files are partial-run safe: they should be rewritten after every
variant attempt and should include completed failures rather than requiring all
variants to finish.

## Evidence Bundle Schema

`evidence_bundle.json` uses schema name
`snappy_mesh_evidence_bundle.v1` and records:

- `variant_id`
- `status`
- `taxonomy`
- `artifacts`
- `commands`

The artifact map should preserve paths to source STL, prepared STL reports,
mesh-quality report, solver-smoke report, geometry-capture report, screenshots,
logs, and tracebacks when present. Do not use the evidence bundle as the only
source of detailed metrics; keep the metric JSON files separate and queryable.

## Failure Taxonomy

Use stable machine-readable failure codes:

- `geometry.raw_export_missing_stl`
- `geometry.raw_export_failed`
- `geometry.raw_export_gate_failed`
- `geometry.prepared_surface_not_closed`
- `geometry.surface_prep_displacement_budget`
- `meshing.auto_repair_report_missing`
- `meshing.snappy_timeout`
- `meshing.checkmesh_failed`
- `meshing.relaxed_quality_only`
- `solver.potential_not_completed`
- `artifact.screenshots_incomplete`
- `artifact.surface_deviation_missing`
- `automation.exception`

Strict `checkMesh` failure and relaxed acceptance must both be recorded.
Current relaxed policy accepts max skewness below `8` only when
non-orthogonality also remains within the configured ceiling, currently `70`
degrees by default.

## Geometry-Capture Metrics

The geometry-capture report should include global point-to-triangle distance
summaries and feature-region proxy buckets. The current proxy buckets are:

- `fuselage_nose_proxy`
- `fuselage_tail_proxy`
- `leading_edge_proxy`
- `trailing_edge_proxy`
- `wing_root_blend_proxy`
- `wingtip_proxy`

These buckets are deterministic screening metrics only. The aircraft-only
screenshot set remains the human geometry-capture review artifact.

Default axis convention for the current SDF/export frame:

- longitudinal axis: `x`
- lateral/span axis: `z`

If the aircraft frame changes, record the axis convention in
`geometry_capture_report.json` and do not compare old feature buckets directly
against new-axis runs.

## Obsolete Geometry Handling

When the aircraft definition changes materially, especially inlets or fuselage
integration, do not resume an old five-variant run as acceptance evidence.
Keep old run roots for script validation and historical reference only, then
start a new run root and variant config for the new geometry family.
