# Phase 12: Mechanical Patterns + Named Dimensions

## Steps

- [x] Step 1: Add `indexmap` to Cargo.toml
- [ ] Step 2: Create `src/sdf/aerospace/mechanical.rs` with all 10 SDF functions
- [ ] Step 3: Update `src/sdf/aerospace/mod.rs` to export mechanical module
- [ ] Step 4: Add `register_mechanical_functions` to `src/scripting/api.rs`
- [ ] Step 5: Add `dimensions: IndexMap<String, f64>` to AppState + dimension commands to `src/undo.rs`
- [ ] Step 6: Add `dimensions` param to `evaluate_script_full` in `src/scripting/mod.rs`
- [ ] Step 7: Update `app.rs`: pass dimensions to evaluate, add pending re-eval flag
- [ ] Step 8: Create `src/ui/dimensions.rs` (dimensions panel UI)
- [ ] Step 9: Update `src/ui/mod.rs` to export dimensions module
- [ ] Step 10: Integrate dimensions panel into project tree area in `app.rs`
- [ ] Step 11: Build and verify

## Acceptance Criteria
- All 10 mechanical functions callable from Rhai scripts
- `bolt_circle(2.0, 15.0, 4, 5.0)` returns SdfHandle subtractable from a plate
- `dimensions` panel visible below project tree
- Dimension values injected as bare Rhai constants before eval
- Changing a dimension value triggers re-eval on next frame
- Ctrl+Z undoes dimension edits; Ctrl+Z undoes deletions
