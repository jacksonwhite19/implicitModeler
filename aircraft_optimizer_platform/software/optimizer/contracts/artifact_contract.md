# Artifact Contract

Date created: 2026-06-20

## Purpose

Artifacts are files or external objects produced during candidate evaluation.

Artifacts include Rhai scripts, STL files, result JSON, logs, screenshots, plots, CFD cases, solver outputs, reports, and videos.

## Required Fields

```yaml
artifact_id: string
candidate_id: string
evaluation_id: string
module_attempt_id: string | null
artifact_type: string
path: string
content_hash: string
hash_algorithm: sha256
created_at: datetime
producer_module: string
producer_version: string
mime_type: string | null
size_bytes: integer
metadata: map
source:
  generated | copied_reference | external_reference
source_original_path: string | null
status: available | missing | deleted | archived
```

## Artifact Types

Initial types:

- `geometry_source_rhai`
- `geometry_provider_result_json`
- `oml_stl`
- `export_result_json`
- `module_log_jsonl`
- `stdout_log`
- `stderr_log`
- `screenshot_top`
- `screenshot_side`
- `screenshot_front`
- `screenshot_isometric`
- `metrics_json`
- `score_result_json`
- `conditioning_request_json`
- `conditioning_result_json`
- `conditioned_cache_manifest_json`
- `conditioned_sdf_block_store`
- `conditioning_diagnostics_json`
- `conditioning_slice_plot`
- `conditioning_comparison_report`

Future types:

- `cfd_mesh`
- `cfd_case`
- `cfd_residual_plot`
- `cfd_surface_visualization`
- `mission_plot`
- `surrogate_dataset`
- `report_html`
- `video_timelapse`

## Path Rules

- Store paths relative to the campaign root where possible.
- Preserve original source path for copied references.
- Do not use filenames as primary identity.
- Record content hashes for reproducibility.

## Metadata Rules

Metadata should include:

- Module-specific settings.
- Quality gates.
- Units.
- Coordinate frame when relevant.
- Validation status.
- Runtime environment references.

Exporter-specific tile settings belong in artifact or module metadata, not in the candidate record.

Conditioning-specific metadata should include canonical graph version/hash,
cache contract version, cache state, update mode, dirty-region summary,
conditioning backend, confidence, and fallback mode. A conditioned geometry cache
artifact is derived state; it must not be treated as the aircraft source of
truth.
