# Aircraft Optimizer Platform Architecture and Roadmap

Date: 2026-06-19

## Executive Position

Build the optimizer as a campaign orchestration and evidence system, not as an aircraft geometry application. The existing implicit CAD/SDF system remains the geometry provider. The new platform owns design-variable definitions, candidate lifecycle, analysis orchestration, scoring, traceability, artifact capture, optimizer state, dashboards, and later MDAO/ML data readiness.

The longer-term implicit-kernel direction is documented in `implicit_aircraft_design_roadmap.md`. That overlay keeps the same adapter boundary while making the implicit geometry kernel the canonical aircraft representation. STLs, meshes, OpenFOAM cases, datasets, screenshots, dashboards, and AI inputs are derived artifacts. The Phase 0 research gate for local SDF conditioning is recorded in `research/phase0_sdf_conditioning_literature_review.md`: diagnostic-first local conditioning design is unblocked, while production SDF-native CFD and AI-surrogate implementation remain deferred.

The formal geometry architecture split is `canonical geometry graph -> conditioned geometry cache -> clients`. The canonical graph owns the aircraft definition. The conditioned geometry cache is derived, disposable, rebuildable, versioned, incrementally updateable, and query optimized. Downstream clients should consume typed geometry queries or derived artifacts rather than coupling directly to procedural geometry internals.

The central architecture decision is to make every major capability a replaceable module behind typed contracts:

- Geometry is consumed through a geometry-provider adapter.
- Analyses are run through analysis-module adapters.
- Scoring is versioned separately from analysis.
- Optimizers propose candidates through an ask/tell interface.
- Logging, metrics, artifacts, lineage, and failure classification are first-class outputs.
- Dashboard reads from the run database and artifact store, not from transient optimizer memory.

This keeps the system usable for simple fixed-wing endurance studies while leaving a credible path to multi-fidelity MDAO, VTOL/tailsitter configurations, surrogate models, and large campaign management.

## 1. Overall System Architecture

Recommended architecture:

```text
User / Dashboard / CLI
        |
Campaign Service
        |
Optimization Engine <-----> Candidate Registry / Run Database
        |
Evaluation Orchestrator
        |
Analysis Pipeline
        |
Geometry Provider Adapter -> Existing implicit CAD/SDF generator
        |
Artifact Store / Metrics Store / Logs / Failure Taxonomy
```

The platform should be organized around immutable campaign records and candidate evaluation records. A candidate is a parameter vector plus metadata. An evaluation is the execution of a candidate through a specific pipeline version under specific software and configuration versions.

Core principles:

- The optimizer never directly calls CFD internals.
- CFD modules never directly know scoring rules.
- Geometry generation is an adapter call, not an optimizer concern.
- Results are never just scalar scores; they are typed observations with provenance.
- Every state transition is persisted before expensive work starts.
- Failed candidates are retained as valuable data.

Recommended deployment for early versions:

- Single workstation, local services, file-backed artifact store.
- Python orchestration process for analysis and optimization.
- Local web dashboard.
- SQLite first, PostgreSQL-compatible schema later.
- Containerized or environment-captured analysis tools where practical.

## 2. Major Subsystems

### Campaign Service

Owns run creation, run configuration, optimizer selection, variable bounds, constraints, scoring configuration, analysis pipeline selection, and global run status.

Responsibilities:

- Create optimization campaigns.
- Freeze run configuration.
- Store campaign-level metadata.
- Start, pause, resume, and stop campaigns.
- Record software and environment fingerprints.
- Expose run state to the dashboard.

### Candidate Registry

Owns candidate identity, design variables, lineage, tags, lifecycle status, and evaluation attempts.

Responsibilities:

- Assign stable candidate IDs.
- Store candidate parameter vectors.
- Store parent-child relationships.
- Store mutation/crossover/operator metadata.
- Support manual candidate insertion.
- Support user annotations.
- Prevent loss of failed candidates.

### Evaluation Orchestrator

Executes the analysis pipeline for each candidate.

Responsibilities:

- Materialize candidate inputs.
- Request geometry from the existing CAD/SDF provider.
- Run validation and analysis modules in order.
- Apply early-exit gates.
- Persist partial results.
- Classify failures.
- Track runtime/cost.
- Emit artifacts.

### Analysis Module Framework

Defines a standard contract for all analysis modules.

Initial modules:

- Geometry validation.
- Geometry metrics.
- Mass/CG estimate.
- Low-fidelity aerodynamics.
- Medium-fidelity aerodynamics.
- High-fidelity CFD.
- Scoring.

Future modules:

- Structural analysis.
- Propulsion.
- Controls.
- Mission simulation.
- Thermal systems.
- Noise/acoustics.
- Manufacturing constraints.

### Scoring Service

Converts typed analysis outputs into objectives, constraints, penalties, rankings, and Pareto status.

Scoring must be versioned independently because the same candidate may need to be rescored later without rerunning CFD.

### Artifact Service

Manages meshes, screenshots, solver inputs, solver outputs, logs, plots, reports, videos, and derived thumbnails.

Artifacts are content-addressed where possible and linked to candidate/evaluation IDs.

### OML Export Adapter

The current validated OML/CFD STL export path is the direct sparse SDF marching-cubes exporter documented in `oml_export_contract.md`.

This adapter should remain separate from geometry definition:

- Geometry provider output: SDF feature name or generated Rhai script path, plus optional native-frame bbox.
- Export adapter input: geometry-provider output, export preset, and quality limits.
- Export adapter output: STL artifact, machine-readable result JSON, logs, and validation metrics.

Default optimizer preset:

- `direct_sparse_oml_fast`
- `spacing_mm = 1.0`
- `tile_size_mm = 16`
- `sdf_workers = 8`

High-fidelity rerun/finalization preset:

- `direct_sparse_oml_high_fidelity`
- `spacing_mm = 0.75`
- `tile_size_mm = 24`
- `sdf_workers = 8`

The fast preset should be used during normal optimization iterations. The high-fidelity preset should be used after the optimizer is converging on a smaller number of promising designs or when preparing selected designs for higher-fidelity CFD.

Production CFD-ready exports should pass configurable topology gates, defaulting to zero boundary edges, zero nonmanifold edges, one connected component, zero duplicate triangles, and no known-section long chords. Relaxed gates are acceptable only for geometry stress tests and should be recorded in the candidate/evaluation artifact metadata.

### Dashboard

Standalone local web app. It should read from the run database and artifact store through a local API.

Dashboard should never be required for campaign execution. Campaigns must be runnable headless.

## 3. Data Model Recommendations

Use a relational core for traceability and queryability, plus file/object storage for large artifacts. Avoid storing critical metadata only in filenames.

Key entities:

### Campaign

- `campaign_id`
- `name`
- `created_at`
- `objective_set_id`
- `variable_schema_id`
- `constraint_schema_id`
- `optimizer_config_id`
- `analysis_pipeline_id`
- `scoring_config_id`
- `status`
- `notes`

### Variable Schema

- `variable_schema_id`
- `aircraft_family`
- `geometry_provider`
- `schema_version`
- `variables`
- `bounds`
- `units`
- `scaling`
- `default_values`
- `frozen_parameters`

Initial variable schema should allow continuous variables for the selected fixed-wing family. Do not encode topology changes in v1. Bounded numeric selectors are acceptable for fixed, enumerated subchoices such as airfoil family when they do not add/remove aircraft components or change aircraft topology; the selected option must be recorded in the candidate design vector and schema metadata.

### Candidate

- `candidate_id`
- `campaign_id`
- `candidate_index`
- `design_vector`
- `normalized_design_vector`
- `aircraft_definition_ref`
- `created_by`
- `created_at`
- `generation`
- `status`
- `parent_candidate_ids`
- `lineage_event_id`

### Evaluation

- `evaluation_id`
- `candidate_id`
- `campaign_id`
- `pipeline_version`
- `started_at`
- `finished_at`
- `status`
- `failure_id`
- `runtime_seconds`
- `compute_cost_estimate`
- `environment_fingerprint_id`

A candidate can have multiple evaluations, for example low-fidelity only, medium-fidelity rerun, high-fidelity CFD rerun, or rescoring under a new scoring version.

### Analysis Result

- `result_id`
- `evaluation_id`
- `module_name`
- `module_version`
- `metric_name`
- `value`
- `units`
- `source`
- `fidelity_level`
- `confidence`
- `runtime_seconds`
- `validity_window`
- `artifact_refs`

Recommended value shape:

```json
{
  "metric": "LiftToDrag",
  "value": 14.2,
  "units": "dimensionless",
  "source": "vspaero_vlm",
  "fidelity": "medium",
  "confidence": 0.62,
  "runtime_cost": {
    "wall_seconds": 38.4,
    "cpu_seconds": 122.1,
    "estimated_cost_class": "medium"
  },
  "provenance": {
    "module": "aero_vspaero",
    "module_version": "0.3.1",
    "input_hash": "sha256:...",
    "artifact_ids": ["artifact_..."]
  }
}
```

### Objective / Constraint Result

- `score_result_id`
- `evaluation_id`
- `scoring_config_id`
- `objective_values`
- `constraint_values`
- `penalty_values`
- `aggregate_score`
- `pareto_rank`
- `dominated_by`
- `score_explanation`

### Artifact

- `artifact_id`
- `evaluation_id`
- `candidate_id`
- `artifact_type`
- `path`
- `content_hash`
- `created_at`
- `producer_module`
- `mime_type`
- `size_bytes`
- `metadata`

### User Annotation

- `annotation_id`
- `candidate_id`
- `evaluation_id`
- `user`
- `tag`
- `comment`
- `created_at`
- `resolved_at`

Suggested built-in tags:

- `promising`
- `investigate`
- `rerun_cfd`
- `visually_interesting`
- `suspicious_result`
- `bad_geometry_but_interesting`
- `holdout_validation`
- `do_not_select`

## 4. Run Database Architecture

Start with SQLite for v1 because it is simple, inspectable, easy to back up, and enough for a local workstation MVP. Design the schema so it can migrate to PostgreSQL when campaigns become concurrent or multi-user.

Recommended layout:

```text
aircraft_optimizer_platform/
  opt_output.md
  opt_prompt.md

future runtime layout:
  optimizer_runs/
    campaigns/
      campaign_<id>/
        run.db
        config/
        artifacts/
        logs/
        reports/
```

Database guidance:

- Use normalized relational tables for campaign, candidate, evaluation, result, artifact, lineage, failure, and annotation metadata.
- Store high-volume time-series logs either as compressed JSONL/Parquet files referenced from the DB or in a dedicated telemetry table if volume is small.
- Use append-only event logs for lifecycle events.
- Use stable IDs rather than path-derived identity.
- Keep migration scripts from the first implementation.
- Treat schema version as a first-class field.

Recommended database tables:

- `campaigns`
- `variable_schemas`
- `optimizer_configs`
- `analysis_pipelines`
- `scoring_configs`
- `candidates`
- `candidate_lineage`
- `evaluations`
- `analysis_results`
- `score_results`
- `failures`
- `artifacts`
- `environment_fingerprints`
- `software_components`
- `events`
- `user_annotations`
- `manual_actions`

## 5. Candidate Lifecycle

Recommended lifecycle:

```text
proposed
  -> registered
  -> geometry_requested
  -> geometry_generated
  -> geometry_validated
  -> metrics_computed
  -> low_fidelity_complete
  -> medium_fidelity_complete
  -> high_fidelity_complete
  -> scored
  -> ranked
  -> archived
```

Failure states:

```text
proposed
  -> rejected_by_bounds
  -> rejected_by_constraints
  -> geometry_failed
  -> validation_failed
  -> meshing_failed
  -> analysis_failed
  -> scoring_failed
  -> timed_out
```

Lifecycle rules:

- Persist `registered` before starting geometry generation.
- Persist every module start and completion.
- Store partial results even if later modules fail.
- Failed candidates remain queryable and useful for future failure prediction.
- Re-evaluation creates a new evaluation record, not a mutated old result.
- Manual reruns must record the user action and reason.

## 6. Analysis Module Architecture

Each analysis module should implement the same logical contract:

```text
ModuleSpec:
  name
  version
  inputs
  outputs
  required_artifacts
  optional_artifacts
  fidelity_level
  expected_runtime_class
  deterministic_or_stochastic
  cache_policy

run(context, inputs) -> AnalysisResultBundle
```

The module context should include:

- Candidate ID.
- Evaluation ID.
- Working directory.
- Input artifact paths.
- Run configuration.
- Environment variables.
- Timeout budget.
- Resource budget.
- Logger.
- Artifact registration callback.

Module output should include:

- Typed metrics.
- Confidence metadata.
- Runtime/cost metadata.
- Produced artifacts.
- Warnings.
- Failure classification if failed.

Escalating fidelity pipeline:

1. Geometry validation: checks parameter bounds, geometry creation, watertightness where relevant, self-intersections, non-manifold geometry, impossible dimensions.
2. Geometry metrics: span, area, volume, wetted area estimates, aspect ratio, taper ratio, thickness ratios, inlet areas, reference lengths.
3. Mass/CG estimate: statistical or rule-based estimate from geometry metrics and material assumptions.
4. Low-fidelity aero: fast analytical or empirical estimates.
5. Medium-fidelity aero: AVL, VSPAERO, panel/VLM methods where valid.
6. High-fidelity CFD: SU2 or OpenFOAM through a strict adapter.
7. Scoring: objectives, constraints, penalties, explanation.

Early-exit gates:

- Hard geometry invalidity.
- Basic stability or CG envelope impossible.
- Mass estimate exceeds mission feasibility.
- Low-fidelity L/D below threshold.
- Mesh risk score too high.
- Candidate dominated under cheap metrics with no exploration need.

## 7. Optimization Architecture

Use an optimizer abstraction with an ask/tell interface:

```text
ask(n) -> proposed candidates
tell(candidate_id, objectives, constraints, metadata) -> optimizer update
```

This supports evolutionary algorithms, Bayesian optimization, random search, surrogate-assisted methods, and future distributed evaluation.

Initial optimizer modes:

- Latin hypercube / Sobol sampling for design-space exploration.
- NSGA-II/III or related algorithms through pymoo for multi-objective studies.
- Optuna for single/multi-objective studies with pruning and persistent studies.
- Nevergrad for derivative-free black-box optimization.

Recommended objectives for initial endurance/loiter studies:

- Maximize estimated endurance.
- Maximize L/D or mission-specific efficiency.
- Minimize mass.
- Minimize power required.

Recommended constraints:

- Geometry validity.
- Wing loading bounds.
- Static margin bounds.
- Stall speed maximum.
- Minimum payload volume or internal volume.
- Maximum span or packaging dimension.
- Thrust/power margin.
- Manufacturability or minimum thickness constraints.

Keep scoring separate from optimizer state. The optimizer receives objective/constraint values, but the database stores the scored explanation and raw analysis metrics.

## 8. Dashboard Architecture

Build a standalone local web application with a local API. Do not integrate it into the CAD application.

Recommended architecture:

```text
Browser UI
  |
FastAPI local API
  |
Run database + artifact store
  |
Optional websocket/event stream for live updates
```

Core views:

- Campaign list.
- Campaign monitor.
- Candidate table.
- Candidate detail page.
- Pareto front explorer.
- Convergence plots.
- Metric correlation explorer.
- Failure analysis dashboard.
- Artifact gallery.
- Candidate comparison.
- Lineage graph.
- User annotations and review queue.

Candidate detail should show:

- Design variables.
- Geometry metadata.
- Scores and constraints.
- Analysis confidence.
- Lifecycle status.
- Parent/child lineage.
- Standard screenshots.
- Logs and failure details.
- User tags and notes.
- Rerun actions.

Dashboard must be read-mostly. Mutating actions such as rerun CFD, mark promising, or exclude candidate should go through explicit API endpoints that write audit events.

## 9. Logging Architecture

Use three layers of logging:

### Event Log

Append-only lifecycle events:

- Campaign created.
- Candidate proposed.
- Candidate registered.
- Module started.
- Module completed.
- Module failed.
- Artifact produced.
- Candidate scored.
- User tag added.
- Rerun requested.

### Structured Module Logs

JSONL logs with:

- Timestamp.
- Candidate ID.
- Evaluation ID.
- Module.
- Severity.
- Message.
- Structured fields.
- External command.
- Return code.
- Runtime.

### Raw Tool Logs

Capture stdout/stderr and input decks from external tools such as AVL, SU2, OpenFOAM, VSPAERO, Gmsh, and XFOIL.

Rules:

- Do not rely on terminal output as the record of truth.
- Store command lines and environment variables with sensitive values redacted.
- Store solver config files as artifacts.
- Store random seeds.
- Store warnings, not only failures.

## 10. Artifact Management Architecture

Artifacts should be stable, addressable, and linked to their producers.

Standard v1 artifacts per candidate:

- Aircraft definition JSON.
- Geometry export reference.
- Mesh or surface file reference.
- Top view screenshot.
- Side view screenshot.
- Front view screenshot.
- Isometric screenshot.
- Metrics JSON.
- Analysis summary JSON.
- Score summary JSON.
- Module logs.

Future artifacts:

- CFD mesh.
- CFD residual plots.
- Pressure coefficient visualization.
- Streamline images.
- Mission simulation plots.
- Structural mode shapes.
- Best-candidate evolution videos.
- Optimization timelapse videos.
- Convergence galleries.

Recommended screenshot service:

- Deterministic camera presets.
- Consistent scale and framing.
- Neutral background.
- Candidate ID and version metadata embedded in image metadata or sidecar JSON.
- Same image dimensions across candidates.
- Optional overlay-free raw image plus annotated image.

Use PyVista/VTK for local rendering and screenshots. VTK is the underlying visualization stack, and PyVista provides a more ergonomic Python interface for mesh visualization.

## 11. Traceability and Versioning Strategy

Every evaluation must be reproducible from captured state.

Track:

- Candidate design variables.
- Variable schema version.
- Constraint schema version.
- Aircraft family version.
- Geometry provider version.
- Geometry input hash.
- Geometry output hash.
- Analysis pipeline version.
- Analysis module versions.
- External tool versions.
- Scoring version.
- Optimizer version.
- Optimizer state snapshot.
- Random seeds.
- OS and Python versions.
- Package lockfile or environment export.
- Git commit hashes for in-house code.
- Command lines and configs for external tools.

Recommended versioning model:

- Immutable `analysis_pipeline_id`.
- Immutable `scoring_config_id`.
- Immutable `optimizer_config_id`.
- Content hashes for artifacts.
- Semantic versions for internal modules.
- Git commit hashes for source repos.
- Environment fingerprints for runtime.

Important rule: never overwrite a result when rescoring or rerunning. Create a new evaluation or score result linked to the same candidate.

## 12. Candidate Lineage System

Lineage should be explicit rather than inferred from generation number.

Lineage event fields:

- `lineage_event_id`
- `child_candidate_id`
- `parent_candidate_ids`
- `operator`
- `operator_version`
- `mutation_summary`
- `mutation_vector`
- `reason`
- `optimizer_state_ref`
- `created_at`

Example:

```json
{
  "child_candidate_id": 108,
  "parent_candidate_ids": [42],
  "operator": "gaussian_mutation",
  "operator_version": "1.0.0",
  "mutation_summary": [
    {"variable": "wing.sweep_deg", "delta": 3.0},
    {"variable": "wing.taper_ratio", "delta": -0.05}
  ],
  "reason": "optimizer_exploration",
  "generation": 7
}
```

Support lineage queries:

- Show candidate ancestry.
- Find all descendants of a promising candidate.
- Compare siblings.
- Identify mutation operators that produce failures.
- Identify families that improve under medium/high fidelity.

## 13. Failure Taxonomy Proposal

Failures should be classified consistently with severity, stage, recoverability, and evidence.

Top-level categories:

### Input and Configuration

- `input.invalid_variable_schema`
- `input.missing_required_variable`
- `input.out_of_bounds`
- `input.unit_mismatch`
- `input.invalid_constraint_definition`
- `input.invalid_pipeline_config`

### Geometry

- `geometry.provider_error`
- `geometry.invalid_aircraft_definition`
- `geometry.self_intersection`
- `geometry.non_manifold_surface`
- `geometry.open_surface`
- `geometry.topology_issue`
- `geometry.invalid_component_overlap`
- `geometry.impossible_dimension`
- `geometry.export_failure`
- `geometry.timeout`

### Geometry Metrics

- `metrics.reference_area_failed`
- `metrics.volume_failed`
- `metrics.wetted_area_failed`
- `metrics.invalid_reference_frame`
- `metrics.outlier_metric_detected`

### Mass and Balance

- `mass.estimate_failed`
- `mass.cg_out_of_bounds`
- `mass.negative_mass`
- `mass.inconsistent_component_mass`
- `mass.payload_volume_violation`

### Meshing

- `meshing.surface_mesh_failed`
- `meshing.volume_mesh_failed`
- `meshing.non_manifold_input`
- `meshing.bad_cell_quality`
- `meshing.boundary_tag_missing`
- `meshing.boundary_layer_failed`
- `meshing.timeout`

### Low/Medium Fidelity Aero

- `aero_low.unsupported_geometry`
- `aero_low.solver_setup_failed`
- `aero_low.no_convergence`
- `aero_low.invalid_coefficients`
- `aero_medium.vlm_invalid_assumption`
- `aero_medium.solver_failed`
- `aero_medium.stability_derivative_failed`

### CFD

- `cfd.case_setup_failed`
- `cfd.mesh_import_failed`
- `cfd.boundary_condition_error`
- `cfd.initialization_failed`
- `cfd.divergence`
- `cfd.timeout`
- `cfd.resource_exhausted`
- `cfd.invalid_residual_history`
- `cfd.invalid_force_coefficients`
- `cfd.postprocess_failed`

### Scoring and Optimization

- `scoring.missing_required_metric`
- `scoring.invalid_metric_value`
- `scoring.version_mismatch`
- `optimization.constraint_violation`
- `optimization.dominated_solution`
- `optimization.optimizer_state_error`
- `optimization.duplicate_candidate`

### Artifact and Infrastructure

- `artifact.write_failed`
- `artifact.hash_mismatch`
- `artifact.missing_expected_artifact`
- `runtime.external_tool_missing`
- `runtime.environment_mismatch`
- `runtime.worker_crash`
- `runtime.cancelled_by_user`

Failure fields:

- Category.
- Stage.
- Severity.
- Recoverability.
- Retryable boolean.
- Human-readable message.
- Evidence artifact IDs.
- Suggested next action.

## 14. Future Surrogate-Model Readiness

Do not build the ML system in v1, but preserve the data it will need.

Store:

- Full design vectors with units and normalized values.
- Geometry-derived features.
- Analysis metrics at every fidelity.
- Failure labels and failure stage.
- Runtime/cost labels.
- Solver convergence histories.
- Mesh quality metrics.
- Screenshots and geometry artifacts.
- Candidate lineage and operator metadata.
- Optimizer state snapshots.
- Human annotations.

ML-relevant future targets:

- Drag prediction.
- L/D prediction.
- Stability derivative prediction.
- Mass/CG prediction.
- CFD convergence/failure prediction.
- Mesh failure prediction.
- Fidelity correction between low/medium/high methods.
- Candidate ranking surrogate.
- Acquisition policy acceleration.

Recommended data-export strategy:

- Provide reproducible dataset exports as Parquet plus artifact manifests.
- Include train/validation split metadata at export time.
- Include leakage controls so descendants or reruns are not accidentally split across train/test in invalid ways.
- Preserve failed candidates, because failure prediction is one of the highest-value early ML uses.

## 15. Future MDAO Readiness

MDAO readiness requires discipline modules, typed variables, coupled execution, and derivative/sensitivity support later.

Prepare for:

- Aerodynamics.
- Structures.
- Mass properties.
- Propulsion.
- Mission simulation.
- Controls.
- Thermal systems.

Design requirements:

- Use typed inputs/outputs with units.
- Separate discipline modules from orchestration.
- Support multiple fidelity levels per discipline.
- Store variable ownership and coupling relationships.
- Allow module dependencies to form a DAG, not only a fixed linear pipeline.
- Preserve partial derivatives or sensitivity estimates when available.
- Support OpenMDAO as a future coupled execution layer rather than forcing every v1 module into OpenMDAO immediately.

Recommended path:

- v1 uses a simple pipeline orchestrator.
- v2 adds DAG execution and cached discipline outputs.
- v3 introduces OpenMDAO wrappers for selected coupled analyses.
- v4 supports gradient-based or hybrid MDAO workflows where tool maturity justifies it.

OpenMDAO is a strong long-term fit because it is an open-source Python framework for systems analysis and multidisciplinary optimization with support for model decomposition, coupled solving, efficient parallel numerical methods, and analytic derivatives.

## 16. Recommended Technology Stack

### Core Language

Python should be the orchestration language for v1 because the analysis ecosystem, optimization libraries, dashboards, and scientific tooling are strongest there.

Use Rust only if later components need high-performance geometry processing or robust CLI tooling. Do not make Rust a required first step for the optimizer.

### Optimization

- pymoo: best initial choice for multi-objective evolutionary optimization and Pareto workflows.
- Optuna: useful for persistent studies, pruning, and experiment management, especially for scalar objectives and later surrogate/model tuning.
- Nevergrad: useful for derivative-free black-box optimization and algorithm variety.
- OpenMDAO: future MDAO framework, not the first orchestration dependency unless coupled discipline analysis is already needed.

### Low/Medium Fidelity Aero

- AVL: good for early fixed-wing stability and VLM-style analysis, but requires careful geometry simplification and assumptions.
- XFOIL: useful for airfoil-level polar generation in limited regimes, but fragile and not a full aircraft solver.
- OpenVSP/VSPAERO: useful if the existing geometry can be translated or paralleled into OpenVSP-compatible representations. OpenVSP is an open-source parametric aircraft geometry tool; VSPAERO is distributed with it and provides aerodynamic analysis capabilities.

### High Fidelity CFD

- SU2: recommended first high-fidelity CFD integration target for aerodynamic design studies because it is open source, released under LGPL 2.1, and built around multiphysics simulation and design.
- OpenFOAM: powerful and broadly used open-source CFD, especially for complex workflows, but often heavier to automate reliably for optimization campaigns.

### Meshing and Visualization

- Gmsh: recommended first general meshing tool because it is open source and scriptable.
- PyVista/VTK: recommended for geometry visualization, screenshots, galleries, and later CFD visualization.

### Data and Storage

- SQLite for v1 local runs.
- PostgreSQL for multi-worker or multi-user campaigns.
- Parquet for ML-ready exports.
- JSON/JSON Schema or Pydantic models for typed configs.
- File-based artifact store with content hashes.

### Local API and Dashboard

- FastAPI backend.
- React or Svelte frontend.
- Plotly or ECharts for interactive charts.
- WebSocket or server-sent events for live run updates.

### Workflow Execution

- Start with a simple local worker queue.
- Later add Celery, Dramatiq, Prefect, or a lightweight custom worker model.
- Avoid a heavyweight workflow system until distributed execution is needed.

### Source Notes

Current official project pages describe OpenMDAO as open-source and MDAO-oriented, SU2 as open-source LGPL 2.1 multiphysics simulation/design software, OpenFOAM as free/open-source CFD, Gmsh as an open-source 3D finite element mesh generator, OpenVSP as an open-source aircraft geometry tool with VSPAERO materials, and pymoo/Optuna/Nevergrad as Python optimization frameworks.

Sources checked:

- OpenMDAO: https://openmdao.org/
- OpenMDAO docs: https://openmdao.org/newdocs/versions/latest/main.html
- SU2: https://su2code.github.io/
- SU2 download/license: https://su2code.github.io/download.html
- OpenFOAM: https://www.openfoam.com/
- OpenFOAM Foundation: https://openfoam.org/
- OpenVSP: https://openvsp.org/
- NASA VSPAERO basics: https://www.nasa.gov/reference/openvsp-vspaero-basics/
- pymoo: https://pymoo.org/
- Optuna: https://optuna.org/
- Nevergrad: https://facebookresearch.github.io/nevergrad/
- Gmsh: https://gmsh.info/

## 17. Recommended Development Phases

### Phase 0: Architecture Freeze and Schema Prototype

Deliverables:

- Written architecture.
- Candidate/evaluation schema.
- Failure taxonomy.
- Artifact manifest schema.
- Variable schema for initial fixed-wing UAV family.
- Example campaign config.

No optimizer implementation yet.

### Phase 1: Headless Evaluation Harness

Deliverables:

- Candidate registry.
- Geometry-provider adapter stub to existing CAD/SDF generator.
- Geometry validation module.
- Geometry metrics module.
- Standard screenshot generation.
- SQLite run database.
- Artifact store.
- Structured logs.

Goal: evaluate a manually supplied candidate end to end through geometry and metrics.

### Phase 2: Low-Fidelity Optimization MVP

Deliverables:

- Continuous variable optimizer integration.
- Low-fidelity aero module.
- Mass/CG estimate.
- Scoring v1.
- Early-exit gates.
- Candidate lineage.
- Basic dashboard candidate table and plots.

Goal: run small fixed-wing endurance campaigns without CFD.

### Phase 3: Medium-Fidelity Aero and Pareto Workflows

Deliverables:

- AVL and/or VSPAERO adapter.
- Pareto ranking.
- Candidate comparison view.
- Failure dashboard.
- User annotation workflow.
- Rescoring support.

Goal: support useful engineering exploration and ranking.

### Phase 4: High-Fidelity CFD Integration

Deliverables:

- SU2 adapter.
- Mesh generation pipeline.
- CFD case template system.
- Solver monitoring.
- CFD failure classification.
- CFD visualization artifacts.
- Rerun selected candidate at high fidelity.

Goal: use CFD selectively, not for every candidate.

### Phase 5: Campaign Scale and Data Exports

Deliverables:

- Worker queue.
- PostgreSQL option.
- Dataset export to Parquet.
- Optimizer state snapshots.
- Large-campaign dashboard performance.
- Timelapse/video artifact generation.

Goal: support large candidate populations and future surrogate modeling.

### Phase 6: MDAO and Multi-Mission Expansion

Deliverables:

- DAG execution.
- OpenMDAO wrappers for selected disciplines.
- Mission simulation.
- Structural analysis.
- Propulsion coupling.
- Multi-mission objectives.

Goal: transition from pipeline optimization to coupled multidisciplinary workflows.

## 18. Risks and Architectural Pitfalls

Major risks:

- Coupling optimizer directly to CAD or CFD internals.
- Treating scalar score as the only important result.
- Failing to store failed candidates.
- Overwriting results instead of creating new evaluations.
- Mixing scoring versions with analysis versions.
- Building dashboard-only state that is not reproducible headlessly.
- Starting with high-fidelity CFD before cheap gates work.
- Letting topology changes into v1.
- Using filenames as the primary database.
- Ignoring units and coordinate frames.
- Under-specifying geometry-provider contract.
- Treating screenshots as cosmetic rather than standardized artifacts.
- Choosing OpenMDAO too early as the only execution model before discipline interfaces are stable.
- Building ML features before the data foundation is trustworthy.
- Failing to capture external solver versions and command configs.

Mitigations:

- Define typed interfaces first.
- Keep every module replaceable.
- Persist lifecycle events and partial results.
- Version all schemas and scoring configs.
- Start with continuous-only fixed-wing variables.
- Use cheap analysis gates before CFD.
- Require every artifact to have producer, hash, and metadata.
- Make reruns append-only.

## 19. Proposed v1 MVP Scope

v1 should prove the data architecture and candidate lifecycle, not solve every aircraft design problem.

Recommended v1 MVP:

- Fixed-wing small UAV family only.
- Continuous variables only.
- Existing CAD/SDF geometry provider consumed through an adapter.
- Manual campaign config file.
- SQLite run database.
- Candidate registry.
- Geometry generation request and artifact registration.
- Geometry validation.
- Geometry metrics.
- Mass/CG estimate.
- Low-fidelity aero estimate.
- Scoring v1 for loiter/endurance.
- pymoo-based exploration or NSGA-II.
- Candidate lineage.
- Failure taxonomy v1.
- Standard four-view screenshots.
- Basic local dashboard:
  - campaign status
  - candidate table
  - score ranking
  - convergence plot
  - artifact gallery
  - failure list
  - user tags

Explicit v1 exclusions:

- Topology changes.
- VTOL/tailsitter geometry.
- Full CFD automation for every candidate.
- Structural optimization.
- Mission simulation beyond simple endurance estimates.
- Surrogate model training.
- Distributed cluster execution.
- Deep OpenMDAO coupling.

## 20. Roadmap from MVP to Full Autonomous Aircraft Optimizer

### MVP: Traceable Fixed-Wing Loiter Optimizer

Primary outcome:

- A local user can run a fixed-wing endurance optimization campaign, browse candidates, see lineage, compare scores, inspect failures, and reproduce results.

### Near Term: Engineering-Useful Multi-Fidelity Optimizer

Add:

- Medium-fidelity aero.
- Better static stability estimates.
- Pareto front visualization.
- Candidate comparison.
- Rerun selected candidates at higher fidelity.
- More robust screenshots and reports.
- Dataset export.

Primary outcome:

- The platform becomes useful for design exploration, not just architecture validation.

### Mid Term: Selective CFD and Campaign Scale

Add:

- SU2 integration.
- Mesh quality gates.
- CFD case templates.
- CFD residual and visualization artifacts.
- Worker queue.
- PostgreSQL option.
- Large-campaign browsing.
- Failure analytics.
- Best-candidate evolution videos.

Primary outcome:

- Expensive CFD is used only where it adds value, and its failures become analyzable data.

### Long Term: Surrogate-Assisted Optimization

Add:

- ML-ready dataset exports.
- Surrogate model training outside the core optimizer.
- Surrogate inference adapters.
- Fidelity correction models.
- Mesh/CFD failure prediction.
- Active learning candidate selection.

Primary outcome:

- The optimizer accelerates using accumulated campaign data without sacrificing traceability.

### Full Ecosystem: MDAO and Autonomous Design Studies

Add:

- OpenMDAO-based coupled workflows.
- Structural analysis.
- Propulsion modeling.
- Mission simulation.
- Controls and stability coupling.
- Thermal systems where relevant.
- VTOL and tailsitter aircraft families.
- Multi-mission objectives.
- Automated engineering report generation.

Primary outcome:

- The platform becomes a long-term autonomous design environment for small unmanned aircraft, with discipline modules and optimization strategies that can evolve independently.

## Recommended Architectural Contract Summary

The foundation should be built around these contracts:

- `GeometryProvider`: consumes candidate definition, returns geometry artifacts and metadata.
- `AnalysisModule`: consumes typed inputs/artifacts, returns typed metrics, confidence, artifacts, and failures.
- `ScoringConfig`: consumes metrics, returns objectives, constraints, penalties, and explanations.
- `OptimizerAdapter`: ask/tell interface for candidate proposal and optimizer updates.
- `ArtifactRegistry`: stores paths, hashes, types, producer metadata, and candidate/evaluation links.
- `LineageRegistry`: records parentage, mutations, operators, and reasons.
- `FailureClassifier`: normalizes failures across geometry, meshing, CFD, scoring, and runtime.
- `DashboardAPI`: read/query plus audited mutation endpoints for annotations and reruns.

If these contracts are kept clean, the platform can start small and still grow into a credible multi-fidelity MDAO optimization ecosystem.
