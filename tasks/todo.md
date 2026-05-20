# Dual Contouring Export Plan

## Acceptance Criteria
- [ ] `build_export_mesh` can produce STL/OBJ meshes through a clean dual-contouring-first export path.
- [ ] The dual-contouring implementation has deterministic mesh-quality diagnostics for boundary and non-manifold edges.
- [ ] Legacy adaptive/export cleanup code is not on the primary non-aero export path.
- [ ] Relevant tests compile and pass.

## Steps
- [x] Step 1: Audit current export seams and define the minimal cleanup boundary.
- [x] Step 2: Replace the existing dual-contouring prototype with a simpler watertight uniform implementation and mesh diagnostics.
- [>] Step 3: Route primary export through dual contouring with explicit fallback behavior.
- [ ] Step 4: Add or update regression tests for simple closed shapes and thin fixed-wing-relevant features.
- [ ] Step 5: Run targeted verification and record results in `output.md`.
