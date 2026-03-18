# Phase 17: Split Body with Alignment Features

## Steps
- [x] Step 1: Create src/sdf/print/mod.rs + update src/sdf/mod.rs
- [x] Step 2: Create src/sdf/print/split.rs (SplitPlane, SplitResult, split_body, split_body_multi, verify_split_fit)
- [x] Step 3: Create src/sdf/print/alignment.rs (AlignmentFeature, generate_positive, generate_negative)
- [x] Step 4: Register Rhai API in src/scripting/api.rs (PlaneHandle, AlignmentHandle, register_print_functions)
- [x] Step 5: UI in src/app.rs (Split Body section in print analysis panel)
- [x] Step 6: Tests embedded in split.rs and alignment.rs (7 new tests)
- [x] Step 7: Build and verify all tests pass (199 total, 0 failed)

## Acceptance Criteria
- split_z(sphere, 0.0) → top half inside at Z=+5, outside at Z=-5
- pins_and_sockets generate_positive/generate_negative produce correct geometry
- verify_split_fit returns fits:true for correctly clearanced split
- split_multi with 2 planes → 3 parts
