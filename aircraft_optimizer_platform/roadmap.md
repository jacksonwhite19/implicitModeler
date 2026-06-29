# Aircraft Optimizer Platform Roadmap

Date: 2026-06-19

## Roadmap Position

This roadmap treats the project as a full aircraft optimizer platform. The first releases should establish durable platform foundations: traceability, candidate lifecycle, replaceable analysis modules, artifact management, dashboard visibility, and clean interfaces to the existing implicit CAD/SDF geometry system.

The platform should grow in controlled layers. Do not start with high-fidelity CFD, topology changes, surrogate models, or full MDAO coupling. Those capabilities depend on trustworthy candidate records, repeatable evaluation pipelines, and disciplined artifact/version tracking.

The primary autonomous execution model is sequential and gated: propose one candidate, run cheap pre-export checks, export only when policy allows it, analyze only when policy allows it, record the result, update optimizer state, and then propose the next candidate. Batch studies remain useful secondary workflows for diagnostics, DOE, parallel campaigns, and future surrogate-data generation.

## Phase 0: Platform Foundation Definition

Goal: freeze the platform contracts before implementation.

Deliverables:

- Platform architecture document in `opt_output.md`.
- Canonical variable schema for the initial fixed-wing UAV family.
- Candidate, evaluation, artifact, failure, lineage, and annotation schemas.
- Analysis module interface specification.
- Geometry provider adapter contract for the existing CAD/SDF system.
- OML/CFD STL export contract using the direct sparse SDF exporter.
- Scoring configuration contract.
- Optimizer ask/tell contract.
- Failure taxonomy v1.
- Artifact naming and storage policy.
- Reproducibility/versioning policy.

Exit criteria:

- A candidate can be described unambiguously as a continuous design vector plus schema version, with any bounded numeric selectors explicitly documented in the schema.
- A future implementation team can build the database and interfaces without redesigning subsystem boundaries.
- v1 exclusions are explicit: no topology changes, no aircraft-family changes, no automatic full-CFD campaign, no surrogate training, no deep OpenMDAO coupling.

## Phase 1: Headless Candidate Evaluation Platform

Goal: evaluate manually supplied candidates through geometry, validation, metrics, artifacts, and persistence.

Core capabilities:

- Local CLI or script entrypoint to create a campaign.
- SQLite run database.
- Candidate registry.
- Geometry provider adapter calling the existing CAD/SDF generator.
- OML export adapter using `direct_sparse_oml_fast` by default.
- Geometry validation module.
- Geometry metrics module.
- Standard four-view screenshot generation.
- Artifact registry.
- Structured event log.
- Failure classification for geometry and artifact failures.

Key artifacts:

- Aircraft definition JSON.
- Geometry export reference.
- OML STL export result JSON.
- OML STL artifact from the direct sparse exporter.
- Top, side, front, and isometric screenshots.
- Geometry metrics JSON.
- Candidate/evaluation records in `run.db`.
- Module logs.

Exit criteria:

- A user can submit one candidate and receive a fully traceable evaluation record.
- Failed geometry is recorded as a classified result, not discarded.
- OML export pass/fail is stored from configurable boundary, nonmanifold, connected-component, duplicate-triangle, and chord-audit gates.
- Screenshots and metrics are linked to the candidate and evaluation IDs.
- Re-running the same candidate creates a new evaluation record rather than overwriting old results.

## Phase 2: Low-Fidelity Fixed-Wing Optimizer MVP

Goal: run the first sequential optimization campaigns for small fixed-wing UAV loiter/endurance using continuous variables only.

Core capabilities:

- Continuous variable bounds and normalization.
- Initial design sampling: Latin hypercube, Sobol, or random seed generation feeding one candidate at a time into the evaluation loop.
- pymoo integration for multi-objective exploration.
- Mass/CG estimate module.
- Low-fidelity aero estimate module.
- Scoring v1 for loiter/endurance.
- Constraint handling.
- Candidate lineage records.
- Pre-export screening and early-exit gates.
- Sequential ask/evaluate/tell optimizer loop.
- Basic campaign status reporting.

Initial objectives:

- Maximize estimated endurance.
- Maximize L/D or mission efficiency proxy.
- Minimize mass or power required.

Initial constraints:

- Geometry validity.
- Span/packaging limits.
- Wing loading bounds.
- CG envelope.
- Static margin estimate.
- Stall speed maximum.
- Minimum internal/payload volume.

Exit criteria:

- The optimizer can propose, evaluate, score, and learn from fixed-wing candidates one at a time.
- Each candidate has design variables, lineage, lifecycle status, metrics, score, artifacts, and failure status where applicable.
- The platform can avoid expensive analyses using cheap gates.
- Results are reproducible from stored configs and artifacts.

## Phase 3: Local Dashboard and Human Review Workflow

Goal: make campaigns inspectable and reviewable through a standalone local web app.

Core capabilities:

- FastAPI local backend over the run database and artifact store.
- Browser dashboard.
- Campaign list and run monitor.
- Candidate table with sorting/filtering.
- Candidate detail view.
- Screenshot gallery.
- Ranking view.
- Convergence plots.
- Failure analysis page.
- User tags and comments.
- Audited actions for annotation and rerun requests.

Human-in-the-loop tags:

- `promising`
- `investigate`
- `rerun_cfd`
- `visually_interesting`
- `suspicious_result`
- `holdout_validation`
- `do_not_select`

Exit criteria:

- A user can browse a campaign without reading raw logs.
- A user can compare candidates and mark candidates for further investigation.
- Dashboard mutations write audit events.
- The optimizer remains runnable headless without the dashboard.

## Phase 4: Medium-Fidelity Aerodynamics

Goal: add higher-quality aerodynamic analysis for selected candidates while preserving the same module interface.

Candidate tools:

- AVL for fixed-wing VLM/stability workflows.
- OpenVSP/VSPAERO if geometry translation is practical and validated.
- XFOIL for airfoil polar support where appropriate.

Core capabilities:

- Medium-fidelity aero adapter.
- Geometry simplification/export path for aero tools.
- Stability derivative outputs where supported.
- Confidence metadata on aero results.
- Medium-fidelity gating rules.
- Comparison of low-fidelity versus medium-fidelity predictions.
- Rescoring without rerunning geometry.

Exit criteria:

- Selected candidates can be evaluated at medium fidelity.
- The platform records source, confidence, runtime, and artifacts for each aero metric.
- Low/medium disagreement is visible in the dashboard.
- Medium-fidelity failures are classified and queryable.

## Phase 5: Pareto, Comparison, and Campaign Analytics

Goal: make optimization results useful for engineering decisions.

Core capabilities:

- Pareto ranking.
- Pareto front visualization.
- Objective trade-space plots.
- Candidate comparison view.
- Metric correlation explorer.
- Constraint violation analysis.
- Lineage graph.
- Operator success/failure analysis.
- Run summary report generation.

Exit criteria:

- A user can identify best candidates, tradeoffs, and failure clusters.
- The platform can explain why a candidate ranks well or poorly.
- Candidate ancestry is visible and queryable.
- Campaign summaries are generated from database records, not manual notes.

## Phase 6: STL-Native CFD Ranking and Selective Confirmation

Goal: give plausible STL-generated candidates a practical CFD signal, then spend expensive high-fidelity CFD only on survivors.

Recommended first target:

- OpenFOAM with the snappyHexMesh-based STL-native pipeline, because current
  evidence shows it can preserve generated aircraft surface fidelity and run
  no-slip solver plumbing across the approved variants.

Secondary/advanced target:

- SU2 as a future comparison path once the same STL surface fidelity, marker,
  volume mesh, wall-treatment, and convergence contracts can be met.

Core capabilities:

- Medium-cost ranking-CFD mesh/solver preset for most plausible candidates.
- Stricter confirmation/scoring-development preset for top candidates.
- Rank-correlation study comparing ranking CFD against finer reruns.
- Mesh generation pipeline using generated STLs as the geometry source of truth.
- CFD case template system.
- Solver adapter.
- Timeout/resource budgets.
- Residual monitoring.
- Force/moment extraction.
- CFD visualization artifacts.
- CFD failure taxonomy.
- Manual and automatic `rerun_cfd` workflows.

Exit criteria:

- Plausible candidates receive a bounded ranking-CFD evaluation unless they
  fail hard physical or geometry gates.
- A user can select promising candidates for higher-fidelity CFD validation.
- CFD setup, mesh, solver configs, logs, residuals, and postprocessed coefficients are stored as artifacts.
- CFD failures are classified into setup, meshing, divergence, timeout, invalid coefficients, or postprocess failures.
- Ranking CFD can influence candidate ordering only with explicit non-scoring
  fidelity metadata.
- Confirmed CFD results can update scores through a new evaluation or score
  result without destroying lower-fidelity records.

## Phase 7: Scale, Workers, and Dataset Exports

Goal: support larger campaigns and preserve data for future ML.

Core capabilities:

- Local worker queue.
- Parallel candidate evaluation.
- PostgreSQL-compatible database option.
- Campaign pause/resume.
- Optimizer state snapshots.
- Artifact garbage-collection policy for temporary files.
- Parquet dataset export.
- Dataset manifests.
- Train/validation split metadata.
- Failure-label exports.

Exit criteria:

- Large candidate populations remain browseable.
- Campaigns can resume after interruption.
- Dataset exports include design vectors, geometry metrics, analysis results, scores, failures, artifacts, versions, and lineage.
- Failed candidates are included in ML-ready exports.

## Phase 8: Surrogate-Assisted Optimization

Goal: introduce surrogate support after enough trustworthy data exists.

Core capabilities:

- Surrogate dataset builder.
- External model training workflow.
- Surrogate inference module adapter.
- Prediction confidence and applicability-domain metadata.
- Surrogate-assisted candidate prefiltering.
- Fidelity correction models.
- Mesh/CFD failure prediction.
- Active-learning candidate selection.

Exit criteria:

- Surrogate predictions are stored as analysis results with source and confidence metadata.
- Surrogates can accelerate campaigns without replacing ground-truth analysis records.
- The dashboard distinguishes predicted, low-fidelity, medium-fidelity, and CFD-backed results.

## Phase 9: MDAO Execution Layer

Goal: move from linear pipelines to coupled multidisciplinary workflows.

Core capabilities:

- DAG-based discipline execution.
- Discipline input/output contracts with units.
- Coupling metadata.
- OpenMDAO wrappers for mature discipline modules.
- Sensitivity/derivative storage where available.
- Coupled aero/mass/propulsion/mission workflows.
- Multi-fidelity discipline selection.

Disciplines:

- Aerodynamics.
- Structures.
- Mass properties.
- Propulsion.
- Mission simulation.
- Controls.
- Thermal systems where relevant.

Exit criteria:

- Selected workflows can run as coupled MDAO problems.
- Discipline modules remain replaceable.
- The existing candidate/evaluation/artifact/versioning foundation still applies.
- MDAO results are traceable at the same level as earlier pipeline results.

## Phase 10: Expanded Aircraft Families and Autonomous Studies

Goal: support broader small-UAV design spaces while preserving family-specific schemas.

Core capabilities:

- VTOL aircraft family schemas.
- Tailsitter schemas.
- Multi-mission objectives.
- Structural optimization.
- Propulsion sizing.
- Controls-aware constraints.
- Mission simulation loops.
- Automated study planning.
- Best-candidate evolution videos.
- Engineering report generation.

Exit criteria:

- Aircraft family changes are represented by explicit schema versions, not ad hoc variable hacks.
- Topology-changing optimization is introduced only with a formal genotype/configuration model.
- Autonomous studies produce traceable reports, artifacts, and candidate histories.

## Release Milestones

### v0.1 Platform Skeleton

- Schema definitions.
- Local database.
- Candidate registry.
- Artifact registry.
- Geometry provider adapter stub.
- Event logging.

### v0.2 Geometry Evaluation

- Existing CAD/SDF geometry integration.
- Direct sparse OML STL export adapter.
- Fast OML preset as default and high-fidelity preset as explicit rerun/finalization option.
- Geometry validation.
- Geometry metrics.
- Standard screenshots.
- Failure taxonomy v1.

### v0.3 Low-Fidelity Optimization

- Sequential continuous optimizer.
- Pre-export screening gates.
- Mass/CG estimate.
- Low-fidelity aero.
- Scoring v1.
- Candidate lineage.

### v0.4 Dashboard MVP

- Local dashboard.
- Candidate browsing.
- Ranking.
- Convergence plots.
- Artifact gallery.
- User annotations.

### v0.5 Medium-Fidelity Analysis

- AVL or VSPAERO adapter.
- Medium-fidelity results.
- Pareto explorer.
- Candidate comparison.

### v0.6 Selective CFD Validation

- Mesh pipeline.
- SU2 adapter.
- CFD artifacts.
- CFD failure classification.
- Selected-candidate reruns.

### v0.7 Campaign Scale

- Worker queue.
- Resume/restart.
- PostgreSQL option.
- Dataset exports.
- Timelapse/gallery generation.

### v1.0 Fixed-Wing Optimizer Platform

- Stable fixed-wing UAV loiter/endurance optimizer.
- Traceable multi-fidelity evaluations.
- Dashboard-based review.
- Reproducible run database.
- Clean artifact store.
- Report generation.

### v2.0 Surrogate-Ready Platform

- ML-ready dataset pipeline.
- Surrogate inference adapters.
- Failure prediction support.
- Active-learning hooks.

### v3.0 MDAO Platform

- DAG execution.
- OpenMDAO integration.
- Coupled discipline workflows.
- Mission and propulsion coupling.

### v4.0 Autonomous Small-UAV Design Ecosystem

- Fixed-wing, VTOL, and tailsitter families.
- Multi-mission optimization.
- Structural and controls-aware workflows.
- Autonomous study generation.
- Engineering report and visualization suite.

## Immediate Next Build Target

The next concrete build target should be `v0.1 Platform Skeleton`.

Build only:

- Config schemas.
- SQLite run database.
- Candidate registry.
- Evaluation records.
- Artifact registry.
- Event logging.
- Geometry provider adapter interface.
- A no-op/mock analysis module.

Do not build:

- Production CFD.
- Surrogate models.
- VTOL/tailsitter support.
- Topology mutation.
- Full MDAO coupling.

This keeps the first implementation small while creating the durable records and interfaces that every later phase depends on.
