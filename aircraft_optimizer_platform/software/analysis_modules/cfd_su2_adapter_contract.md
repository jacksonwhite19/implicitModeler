# SU2 CFD Adapter Contract

Date created: 2026-06-20

## Purpose

This document defines the future optimizer-facing adapter contract for SU2 CFD analysis.

This is not an implementation. The current optimizer skeleton must not invoke SU2 during normal fixture runs.

## Current Tool Environment

```text
WSL2 Ubuntu 22.04
micromamba env: aop-cfd
SU2: 8.5.0
Gmsh: 4.15.2
meshio: 5.3.5
```

Environment documentation:

```text
tools\cfd\README.md
```

## Boundary

The SU2 adapter consumes mesh/config artifacts and returns CFD result artifacts and metrics.

It must not generate aircraft geometry, export STL from SDF, decide optimization policy, score candidates directly, mutate candidate records, or hide solver failures.

## Module Specification

```yaml
module_name: su2_cfd
module_kind: analysis
fidelity_level: medium | high
expected_runtime_class: expensive
deterministic: mostly
cache_policy: input_hash
input_contract_version: su2_cfd_request.v1
output_contract_version: su2_cfd_result.v1
```

## Request Schema

```yaml
module_attempt_id: string
evaluation_id: string
candidate_id: string
execution_mode: dry_run | execute
inputs:
  mesh_artifact_id: string
  su2_config_artifact_id: string
  geometry_provider_result_id: string
  export_module_attempt_id: string | null
  flow_condition:
    mach: number
    reynolds: number
    angle_of_attack_deg: number
    sideslip_deg: number
  solver_settings:
    solver: euler | rans | navier_stokes
    turbulence_model: string | null
    max_iterations: integer
    convergence_tolerance: number
resource_limits:
  timeout_seconds: number
  mpi_ranks: integer
  omp_threads: integer
```

## Result Metrics

Expected metric keys:

- `cfd.lift_coefficient`
- `cfd.drag_coefficient`
- `cfd.lift_to_drag`
- `cfd.moment_coefficient_pitch`
- `cfd.iterations`
- `cfd.final_residual`

All metrics must include value, unit, confidence, and source metadata.

## Artifact Types

| Artifact kind | Required | Purpose |
|---|---:|---|
| `su2_config_cfg` | Yes | Solver configuration used for the run. |
| `su2_case_manifest_json` | Yes | Case manifest tying config, mesh reference, candidate, evaluation, and execution mode together. |
| `su2_mesh` | Yes | Mesh consumed by SU2. |
| `su2_stdout_log` | Yes when executed | SU2 stdout. |
| `su2_stderr_log` | Yes when executed | SU2 stderr. |
| `su2_history_csv` | Yes on solver start | Iteration history and coefficients. |
| `su2_restart` | Optional | Restart/solution file. |
| `su2_surface_flow` | Optional | Surface solution output. |
| `su2_volume_flow` | Optional | Volume solution output. |

## Failure Mapping

| Failure code | Category | Stage | Retryable | Meaning |
|---|---|---|---:|---|
| `runtime.su2_missing` | runtime | setup | No | `SU2_CFD` is not available. |
| `runtime.gmsh_missing` | runtime | setup | No | Gmsh is unavailable when required. |
| `artifact.missing_mesh` | artifact | setup | No | Mesh artifact does not exist. |
| `artifact.missing_config` | artifact | setup | No | SU2 config artifact does not exist. |
| `cfd.setup_failed` | cfd | setup | Maybe | Case directory or config preparation failed. |
| `cfd.solver_failed` | cfd | solve | Maybe | SU2 exited nonzero. |
| `cfd.timeout` | cfd | solve | Yes | Solver exceeded timeout. |
| `cfd.diverged` | cfd | solve | Maybe | Residuals diverged or became invalid. |
| `cfd.not_converged` | cfd | solve | Maybe | Solver finished without convergence. |
| `artifact.missing_history` | artifact | parse | Maybe | Expected history output was not produced. |
| `cfd.parse_failed` | cfd | parse | Maybe | History/result parsing failed. |

## Integration Phases

1. Tool detection only: verify `SU2_CFD`, `gmsh`, and `meshio` versions. Implemented in the optimizer CLI through `check-cfd-tools`.
2. Config/history fixture parser. Initial SU2 history fixture parser exists under `software\optimizer`.
3. Dry-run adapter that creates a case directory and validates inputs without solving. Implemented as `su2_case_dry_run` in the v0.1 fixture.
4. Execute a tiny canonical SU2 case unrelated to aircraft optimization. Not started.
5. Mesh handoff from a known-good mesh artifact.
6. Candidate CFD only after cheap gates pass.

## Current Status

Status: contract defined, tool environment installed, adapter not implemented.
