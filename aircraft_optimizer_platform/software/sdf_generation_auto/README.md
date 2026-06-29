# Automatic SDF Generation

Purpose: generate aircraft SDF/Rhai definitions from typed optimizer variables and aircraft-family schemas.

Current reference examples:

- `dual_contouring\direct_sparse_sdf_mc_experiment\scripts\make_complex_oml_inlet_rhai.py`
- `dual_contouring\direct_sparse_sdf_mc_experiment\scripts\make_complex_oml_inlet_wingtips_rhai.py`
- `dual_contouring\direct_sparse_sdf_mc_experiment\scripts\make_twin_side_inlet_aircraft_rhai.py`

Platform role:

- Convert candidate design vectors into geometry-provider inputs.
- Emit generated script path, feature name, bbox, generator version/hash, and parameter trace.
- Stay separate from STL export and analysis scoring.

Status:

- Not imported.
- Existing scripts are reference/example material only.
- They are not production candidates.
- A production automatic generator should be designed fresh around typed aircraft-family schemas and optimizer variables.

Contract:

- `geometry_generator_contract.md`
