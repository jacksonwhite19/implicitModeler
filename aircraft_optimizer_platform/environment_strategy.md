# Environment Strategy

Date created: 2026-06-20

## Decision

Use a local-first development environment for v0.1.

Do not start with Docker for the optimizer skeleton. Docker is useful later when the platform has real service boundaries, heavy external tools, or reproducibility requirements that exceed a local Python environment.

## v0.1 Environment

Use:

- Python 3.12.
- Local virtual environment or `uv` environment.
- `pyproject.toml` as dependency source of truth.
- SQLite database.
- File-backed artifact store.
- `pytest` smoke tests.

Do not use in v0.1:

- Docker.
- Docker Compose.
- PostgreSQL.
- Dashboard service.
- Real CFD tools.
- Real exporter execution.
- Real CAD/SDF subprocess execution.

## Why Local First

The v0.1 skeleton only needs to prove:

- Candidate records.
- Geometry provider records.
- Evaluation records.
- Module attempts.
- Artifacts.
- Event log.
- Failures.
- SQLite persistence.

Adding Docker before these records exist would add environment complexity without improving the platform architecture.

## When to Add Docker

Add Docker or a devcontainer when at least one of these is true:

- The FastAPI dashboard/API service exists.
- A worker process exists.
- PostgreSQL replaces or supplements SQLite.
- SU2, OpenFOAM, Gmsh, or other heavy external tools need isolated setup.
- Multi-machine or repeatable CI execution matters.
- The frontend/backend need a stable local composition.

Likely first Docker milestone:

```text
Phase 3: Dashboard/API + local worker
```

Likely first heavy-tool container milestone:

```text
Phase 6: Selective high-fidelity CFD
```

## Future Docker Shape

Potential future services:

```text
api: FastAPI backend
worker: evaluation worker
db: PostgreSQL
frontend: React/Vite dashboard
tool_su2: optional CFD runtime image
tool_openfoam: optional CFD runtime image
```

Until those services exist, keep v0.1 clean and local.

## CFD Tool Environment

Current decision:

- Use WSL2 Ubuntu 22.04 plus a user-local micromamba environment for CFD tooling.
- Environment name: `aop-cfd`.
- Installed packages: `python=3.11`, `su2=8.5.0`, `gmsh`, `meshio`.
- Bootstrap script: `tools\cfd\bootstrap_wsl_aop_cfd.sh`.

Rationale:

- No conda/mamba was available on Windows initially.
- Non-interactive sudo is not available inside WSL, so apt-based OpenFOAM installation cannot be automated from this agent session.
- SU2 is available on conda-forge linux-64 and verified in WSL.
- The optimizer skeleton still does not invoke CFD; this is preparation for future adapter work.
