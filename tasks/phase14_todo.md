# Phase 14: Mesh Import to SDF

## Steps
- [ ] Step 1: Create `src/mesh/import.rs` (STL binary/ASCII + OBJ parsers + validation)
- [ ] Step 2: Update `src/mesh/mod.rs` (pub mod import)
- [ ] Step 3: Create `src/sdf/mesh_import.rs` (BVH + approx/accurate SDF + MeshSdf wrapper)
- [ ] Step 4: Update `src/sdf/mod.rs` (pub mod mesh_import)
- [ ] Step 5: Update `src/scripting/mod.rs` (mesh cache + project_dir params)
- [ ] Step 6: Update `src/scripting/api.rs` (register_mesh_functions)
- [ ] Step 7: Update `src/app.rs` (mesh cache, pass to eval, toolbar button, project tree)
- [ ] Step 8: Build and run tests

## Acceptance Criteria
- `import_mesh("cube.stl")` returns SdfHandle composable with subtract/union
- Binary + ASCII STL parse correctly; OBJ triangulates quads
- Approximate SDF correct sign at center vs exterior
- Tests pass: cube SDF matches SdfBox, sphere matches Sphere within tolerance
