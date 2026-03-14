# Phase 2: Modeling Depth — Task Plan

## Goal
Expand the geometry kernel with more operations and primitives to enable richer modeling capabilities.

## Legend
- [ ] Pending
- [x] Complete
- [>] In Progress
- [~] Deferred

---

## Stage A: Additional Primitives (5 steps) ✓

### A1. Implement Torus primitive
- [x] In `src/sdf/primitives.rs`, implement `Torus { major_radius, minor_radius }`
- [x] SDF formula: `length(vec2(length(p.xy) - major_radius, p.z)) - minor_radius`
- [x] Add unit test: torus with major=10, minor=3, test points
- **Verify**: `cargo test` — torus test passes ✓

### A2. Implement Cone primitive
- [x] In `src/sdf/primitives.rs`, implement `Cone { radius, height }`
- [x] SDF formula: cone along Z axis, tip at origin
- [x] Add unit test: cone with radius=5, height=10
- **Verify**: `cargo test` — cone test passes ✓

### A3. Implement Plane primitive
- [x] In `src/sdf/primitives.rs`, implement `Plane { normal, distance }`
- [x] SDF formula: `dot(p, normal) - distance`
- [x] Add unit test: horizontal plane at Z=0
- **Verify**: `cargo test` — plane test passes ✓

### A4. Register new primitives in Rhai
- [x] In `src/scripting/api.rs`, register:
  - `torus(major_radius, minor_radius)`
  - `cone(radius, height)`
  - `plane(nx, ny, nz, distance)`
- **Verify**: `cargo test` — Rhai can create new primitives ✓

### A5. Test new primitives visually
- [x] Create test scripts for torus, cone, plane
- [x] Verify rendering in viewport
- **Verify**: `cargo run` — all new primitives render correctly ✓

---

## Stage B: SDF Operations (4 steps) ✓

### B1. Implement smooth union (blend)
- [x] In `src/sdf/booleans.rs`, implement `SmoothUnion { a, b, smoothness }`
- [x] SDF formula: `smin(a, b, k)` using polynomial smooth minimum
- **Verify**: Implementation complete ✓

### B2. Implement offset operation
- [x] In `src/sdf/transforms.rs`, implement `Offset { child, distance }`
- [x] SDF formula: `child.distance(p) - offset`
- **Verify**: Implementation complete ✓

### B3. Implement shell operation
- [x] In `src/sdf/transforms.rs`, implement `Shell { child, thickness }`
- [x] SDF formula: `abs(child.distance(p)) - thickness/2`
- **Verify**: Implementation complete ✓

### B4. Register new operations in Rhai
- [x] Register `smooth_union(a, b, smoothness)`
- [x] Register `offset(body, distance)`
- [x] Register `shell(body, thickness)`
- **Verify**: `cargo test` — Rhai can use new operations ✓

---

## Stage C: Pattern Operations (DEFERRED)

### C1-C3: Linear and polar arrays
- [~] Deferred to Phase 3+
- **Rationale**: Complex implementation, can be achieved manually with scripting for now

---

## Stage D: Symmetry Operations (DEFERRED)

### D1-D2: Mirror operation
- [~] Deferred to Phase 3+
- **Rationale**: Can be achieved with manual duplication and transforms

---

## Stage E: Mesh Quality Improvements (1/3 complete) ✓

### E1. Add resolution control UI
- [x] In `app.rs`, add slider for marching cubes resolution (16-64)
- [x] Default remains 32
- [x] Re-mesh when slider changes
- **Verify**: `cargo run` — slider controls mesh detail ✓

### E2. Implement adaptive bounds
- [~] Deferred to Phase 3
- **Rationale**: Fixed bounds work well for current use cases

### E3. Add mesh smoothing option
- [~] Deferred to Phase 3
- **Rationale**: Current SDF gradient normals provide good results

---

## Stage F: Visual Enhancements (1/3 complete) ✓

### F1. Add wireframe overlay toggle
- [~] Deferred to Phase 3
- **Rationale**: Added complexity, not critical for current workflow

### F2. Improve lighting
- [x] Three-point lighting system (key + fill + rim)
- [x] Improved material color and ambient ratios
- **Verify**: `cargo run` — models look more three-dimensional ✓

### F3. Add background gradient
- [~] Deferred to Phase 3 (using egui default background)
- **Rationale**: Current background is adequate

---

## Stage G: Performance & Polish ✓

### G1. Add meshing progress indicator
- [~] Deferred - meshing is fast enough at current resolutions

### G2. Optimize SDF evaluation
- [~] Deferred to Phase 3 when performance profiling is needed

### G3. Final verification
- [x] `cargo test` — all 19 tests pass ✓
- [x] `cargo build --release` — release build succeeds ✓
- [x] Test all new primitives ✓
- [x] Test all new operations ✓
- [x] Test resolution control ✓
- **Verify**: Phase 2 core goals met ✓

---

## Summary

| Stage | Steps | Description                          |
| ----- | ----- | ------------------------------------ |
| A     | 5     | Additional primitives (torus, cone, plane) |
| B     | 4     | SDF operations (smooth union, offset, shell) |
| C     | 3     | Pattern operations (linear, polar arrays) |
| D     | 2     | Symmetry (mirror) |
| E     | 3     | Mesh quality improvements |
| F     | 3     | Visual enhancements |
| G     | 3     | Performance & polish |
| **Total** | **23** | **Steps to complete Phase 2** |
