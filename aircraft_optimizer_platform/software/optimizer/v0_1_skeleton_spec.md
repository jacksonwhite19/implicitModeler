# v0.1 Platform Skeleton Specification

Date created: 2026-06-20

## Purpose

This spec defines the first runnable optimizer-platform slice.

The goal is not optimization performance. The goal is to prove durable records, event logging, artifact registration, and module boundaries before wiring real exporters, aero tools, dashboards, or optimizers.

## Scope

v0.1 should implement:

- Local Python package.
- SQLite run database.
- Campaign creation.
- Candidate creation.
- Mock/manual geometry provider.
- Mock module adapter.
- Artifact registry.
- Event log.
- Failure records.
- Minimal CLI or script entrypoint.
- Example run fixture.

v0.1 should not implement:

- Real optimization algorithms.
- Production automatic geometry generation.
- Real STL export adapter.
- CFD.
- Dashboard.
- Surrogate models.
- MDAO.
- Topology changes.

## First Runnable Flow

```text
create campaign
  -> register one manual/reference candidate
  -> create evaluation
  -> run mock/manual geometry provider
  -> register copied Rhai reference as artifact
  -> run mock module adapter
  -> write module metrics
  -> write events
  -> close evaluation
```

## Recommended Initial Package Layout

```text
software/optimizer/
  pyproject.toml
  README.md
  v0_1_skeleton_spec.md
  contracts/
  src/
    aircraft_optimizer/
      __init__.py
      ids.py
      time.py
      config/
      db/
        schema.sql
        connection.py
        repositories.py
      events/
        event_log.py
      records/
        candidate.py
        geometry.py
        evaluation.py
        artifact.py
        module.py
        failure.py
      geometry/
        mock_provider.py
        manual_reference_provider.py
      modules/
        base.py
        mock_module.py
      artifacts/
        registry.py
      cli/
        main.py
  tests/
    test_v0_1_flow.py
```

## Minimal CLI

Initial command shape:

```powershell
python -m aircraft_optimizer.cli.main run-fixture --workspace .\runs\v0_1_fixture
```

Expected behavior:

- Creates a SQLite database.
- Creates a campaign.
- Registers one candidate.
- Uses a curated Rhai reference from `software\sdf_generation_manual\curated_rhai`.
- Creates an evaluation.
- Runs a mock module.
- Registers artifacts.
- Writes events.
- Prints the run directory and database path.

## Initial Fixture Candidate

Use a manual/reference candidate, not a real optimizer-generated candidate.

Candidate fields:

```yaml
aircraft_family: fixed_wing_uav_reference
created_by: seed
design_variables:
  wing.span_mm:
    value: 700.0
    unit: mm
  wing.root_chord_mm:
    value: 170.0
    unit: mm
  wing.tip_chord_mm:
    value: 65.0
    unit: mm
  wing.sweep_deg:
    value: 15.0
    unit: deg
```

Geometry provider output should point to:

```text
software\sdf_generation_manual\curated_rhai\direct_sparse_oml_aircraft_no_inlet.rhai
feature = aircraft_oml_oblique15_y_slice_frame
```

## v0.1 Validation

Required checks:

- Database file exists.
- Required tables exist.
- One campaign record exists.
- One candidate record exists.
- One evaluation record exists.
- One geometry provider result exists.
- At least one artifact record exists.
- At least one module attempt exists.
- Event log contains campaign, candidate, evaluation, geometry, module, and artifact events.
- No real exporter or CAD process is launched.

## Acceptance Criteria

v0.1 is accepted when:

- A fresh checkout can run the fixture command.
- The run creates a self-contained local run directory.
- The database can be inspected with SQLite.
- The output records follow the contracts in `contracts/`.
- The implementation does not depend on dashboard state.
- The implementation does not move or modify CAD/SDF source files.

## Next After v0.1

After v0.1 skeleton works, add:

1. Real manual-reference geometry provider.
2. OML STL exporter adapter as a module.
3. Exporter runtime dependency inventory.
4. Export adapter request/result schema.
5. Basic artifact hashing and log capture.
