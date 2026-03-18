# Phase 15: Multi-Shell Composite Layup

## Steps
- [x] Step 1: Create `src/materials/mod.rs` + `src/materials/composite.rs` (material types + presets)
- [x] Step 2: Update `src/lib.rs` (pub mod materials)
- [x] Step 3: Create `src/sdf/aerospace/composite.rs` (CompositeSdf, CompositeLayup, ShellLayer)
- [x] Step 4: Update `src/sdf/aerospace/mod.rs` (pub mod composite + re-exports)
- [x] Step 5: Update `src/scripting/mod.rs` (MaterialHandle, LayerHandle)
- [x] Step 6: Update `src/scripting/api.rs` (register_composite_functions)
- [x] Step 7: Update `src/app.rs` (layup summary panel)
- [ ] Step 8: FEA integration — per-layer material in CalculiX .inp
- [ ] Step 9: Build and run integration tests

## Acceptance Criteria
- `composite_layup(parent, [layers])` returns SdfHandle at correct total offset
- `layer_at()` correctly identifies layer at test points
- `wing_composite`, `fuselage_composite`, `printed_shell` convenience fns work
- Core layer with lattice infill produces valid distances
- Tests pass
