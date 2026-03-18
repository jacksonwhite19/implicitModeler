# Phase 13: Sweep Operation

## Steps
- [ ] Step 1: Create `src/sdf/sweep.rs` (SweepPath trait + 4 path types + Bishop frame + Sweep SDF)
- [ ] Step 2: Update `src/sdf/mod.rs` (pub mod sweep)
- [ ] Step 3: Add `RectProfile` and `NGonProfile` to `src/sdf/profiles.rs`
- [ ] Step 4: Add `PathHandle` and `ProfileHandle` to `src/scripting/mod.rs`
- [ ] Step 5: Add `register_sweep_functions()` + drone wrappers to `src/scripting/api.rs`
- [ ] Step 6: Build and verify; run tests

## Acceptance Criteria
- `sweep(circle_profile(2.0), line_path(0,0,0, 0,0,10))` produces a result ~ cylinder(2,10)
- All 4 path types construct without panic
- Drone wrappers `cable_channel`, `carbon_rod`, `control_rod` callable from Rhai
- `thread_hole` visual grooves visible when exported
- Tests in sweep.rs pass
