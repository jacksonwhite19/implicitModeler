# Automatic SDF Generation

Purpose: generate aircraft SDF/Rhai definitions and derived geometry-cache
state from typed design variables and aircraft-family schemas.

Current reference examples:

- `dual_contouring\direct_sparse_sdf_mc_experiment\scripts\make_complex_oml_inlet_rhai.py`
- `dual_contouring\direct_sparse_sdf_mc_experiment\scripts\make_complex_oml_inlet_wingtips_rhai.py`
- `dual_contouring\direct_sparse_sdf_mc_experiment\scripts\make_twin_side_inlet_aircraft_rhai.py`

Platform role:

- Convert typed design vectors into geometry-provider inputs.
- Emit generated script path, feature name, bbox, generator version/hash, and parameter trace.
- Own the conditioned geometry cache as derived state from the canonical
  geometry graph.
- Stay separate from STL export and analysis scoring.

Status:

- Not imported.
- Existing scripts are reference/example material only.
- They are not production geometry-kernel code.
- A production automatic generator should be designed fresh around typed
  aircraft-family schemas and design variables.

Backend implementation anchor:

- `..\..\src\sdf\conditioning.rs`

Legacy Python planning helpers:

- `src\geometry_generator_conditioning`

Contract:

- `geometry_generator_contract.md`
- `conditioned_geometry_cache_contract.md`
