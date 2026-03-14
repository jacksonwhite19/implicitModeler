# Phase 1: End-to-End Thin Slice — Task Plan

## Legend
- [ ] Pending
- [x] Complete
- [>] In Progress

---

## Stage A: Project Scaffolding

### A1. Initialize Rust project
- [x] Run `cargo init --name implicit-cad` in the project root
- [x] Verify `cargo build` succeeds with the default hello-world
- **Verify**: `cargo run` prints "Hello, world!" ✓

### A2. Add core dependencies to Cargo.toml
- [x] Add `eframe`, `rhai`, `glam`, `bytemuck` with appropriate versions
- [x] Run `cargo build` to confirm all dependencies resolve
- **Verify**: Clean build with no errors ✓

### A3. Create module file structure
- [x] Create empty module files matching the planned structure:
  - `src/app.rs`
  - `src/sdf/mod.rs`, `src/sdf/primitives.rs`, `src/sdf/booleans.rs`, `src/sdf/transforms.rs`
  - `src/mesh/mod.rs`, `src/mesh/marching_cubes.rs`
  - `src/render/mod.rs`, `src/render/camera.rs`, `src/render/pipeline.rs`
  - `src/scripting/mod.rs`, `src/scripting/api.rs`
- [x] Add `mod` declarations in `main.rs` so all modules are recognized
- [x] Run `cargo build` — should compile (empty modules, no logic yet)
- **Verify**: `cargo build` succeeds with no errors ✓

---

## Stage B: SDF Geometry Kernel

### B1. Define the SDF trait
- [x] In `src/sdf/mod.rs`, define:
  ```rust
  pub trait Sdf: Send + Sync {
      fn distance(&self, point: glam::Vec3) -> f32;
  }
  ```
- [x] Re-export from `sdf/mod.rs`
- **Verify**: `cargo build` succeeds ✓

### B2. Implement Sphere primitive
- [x] In `src/sdf/primitives.rs`, implement `Sphere { radius: f32 }`
- [x] Implement `Sdf` trait: `distance = point.length() - self.radius`
- [x] Add a unit test: sphere of radius 5 at origin → distance at (5,0,0) = 0, at (0,0,0) = -5, at (10,0,0) = 5
- **Verify**: `cargo test` — sphere tests pass ✓

### B3. Implement Box primitive
- [x] In `src/sdf/primitives.rs`, implement `SdfBox { half_extents: Vec3 }`
- [x] Implement `Sdf` trait using the standard SDF box formula
- [x] Add unit tests: box of half-extents (2,3,4) — test points on surface, inside, outside
- **Verify**: `cargo test` — box tests pass ✓

### B4. Implement Cylinder primitive
- [x] In `src/sdf/primitives.rs`, implement `Cylinder { radius: f32, half_height: f32 }`
- [x] Implement `Sdf` trait: infinite cylinder in XY clipped by Z half-height
- [x] Add unit tests: cylinder radius 3, height 10 — test points on surface, inside, outside
- **Verify**: `cargo test` — cylinder tests pass ✓

### B5. Implement Union boolean
- [x] In `src/sdf/booleans.rs`, implement `Union { a: Arc<dyn Sdf>, b: Arc<dyn Sdf> }`
- [x] `distance = min(a.distance(p), b.distance(p))`
- [x] Add unit test: union of two non-overlapping spheres, verify distance at various points
- **Verify**: `cargo test` — union test passes ✓

### B6. Implement Subtract boolean
- [x] In `src/sdf/booleans.rs`, implement `Subtract { a: Arc<dyn Sdf>, b: Arc<dyn Sdf> }`
- [x] `distance = max(a.distance(p), -b.distance(p))`
- [x] Add unit test: subtract sphere from box, verify a point inside the hole is positive
- **Verify**: `cargo test` — subtract test passes ✓

### B7. Implement Intersect boolean
- [x] In `src/sdf/booleans.rs`, implement `Intersect { a: Arc<dyn Sdf>, b: Arc<dyn Sdf> }`
- [x] `distance = max(a.distance(p), b.distance(p))`
- [x] Add unit test: intersect two overlapping spheres, verify points
- **Verify**: `cargo test` — intersect test passes ✓

### B8. Implement Translate transform
- [x] In `src/sdf/transforms.rs`, implement `Translate { child: Arc<dyn Sdf>, offset: Vec3 }`
- [x] `distance = child.distance(point - self.offset)`
- [x] Add unit test: translate a sphere by (10,0,0), verify center moved
- **Verify**: `cargo test` — translate test passes ✓

### B9. Implement Rotate transform
- [x] In `src/sdf/transforms.rs`, implement `Rotate { child: Arc<dyn Sdf>, rotation: Quat }`
- [x] `distance = child.distance(self.rotation.inverse() * point)`
- [x] Add unit test: rotate a box 90 degrees around Z, verify a known point
- **Verify**: `cargo test` — rotate test passes ✓

### B10. Implement Scale transform
- [x] In `src/sdf/transforms.rs`, implement `Scale { child: Arc<dyn Sdf>, scale: Vec3 }`
- [x] Apply inverse scale to point, correct distance by minimum scale component
- [x] Add unit test: scale a sphere by 2x, verify radius effectively doubled
- **Verify**: `cargo test` — scale test passes ✓

### B11. SDF kernel integration test
- [x] Write a combined test: `subtract(translate(box(...), ...), sphere(...))` — build a compound SDF tree and verify distances at known points
- **Verify**: `cargo test` — integration test passes. Full SDF kernel is working. ✓

---

## Stage C: Mesh Extraction

### C1. Define mesh data structures
- [x] In `src/mesh/mod.rs`, define:
  ```rust
  pub struct Vertex { pub position: [f32; 3], pub normal: [f32; 3] }
  pub struct Mesh { pub vertices: Vec<Vertex>, pub indices: Vec<u32> }
  ```
- [x] Derive `Clone`, `Debug`, implement `bytemuck::Pod`/`Zeroable` for `Vertex`
- **Verify**: `cargo build` succeeds ✓

### C2. Implement marching cubes — edge table and tri table
- [x] In `src/mesh/marching_cubes.rs`, add the standard MC lookup tables (256-entry edge table, 256-entry tri table)
- [x] These are static data — no logic yet
- **Verify**: `cargo build` succeeds ✓

### C3. Implement marching cubes — grid sampling
- [x] Implement function: `extract_mesh(sdf: &dyn Sdf, bounds_min: Vec3, bounds_max: Vec3, resolution: u32) -> Mesh`
- [x] Sample the SDF at grid vertices within the bounding volume
- [x] Store sampled distances in a flat array indexed by (x, y, z)
- [x] No triangle generation yet — just the sampling loop
- **Verify**: `cargo build` succeeds, add a test that samples a sphere and checks corner signs ✓

### C4. Implement marching cubes — cell classification and triangle generation
- [x] For each cell in the grid, compute the 8-bit case index from corner signs
- [x] Look up edges from the edge table
- [x] Interpolate vertex positions along edges where the sign changes
- [x] Look up triangle indices from the tri table
- [x] Emit vertices and indices into the Mesh
- **Verify**: `cargo test` — extract mesh from a unit sphere, verify non-empty mesh with plausible vertex count ✓

### C5. Compute face normals for flat shading
- [x] After triangle generation, compute per-face normals via cross product
- [x] Assign the face normal to all 3 vertices of each triangle (flat shading = per-face normals)
- [x] Alternatively: compute normals from the SDF gradient (central differences) for smoother normals — decide based on simplicity
- **Decision**: Use SDF gradient normals (central differences) — they're cheap and produce better results than face normals even for flat shading ✓
- **Verify**: `cargo test` — normals are unit-length and point outward for a sphere ✓

### C6. Marching cubes end-to-end test
- [x] Extract mesh from `subtract(box, translated_sphere)` — a box with a spherical hole
- [x] Verify: mesh has >0 triangles, all normals are unit length, mesh is non-degenerate
- [x] Print vertex/triangle count for manual sanity check
- **Verify**: `cargo test` — all mesh tests pass. Mesh extraction pipeline is working. ✓

---

## Stage D: Renderer Foundation

### D1. Create a minimal eframe app that opens a window
- [x] In `src/main.rs`, set up `eframe::run_native` with a blank `App` struct
- [x] In `src/app.rs`, implement `eframe::App` with an empty `update()` — just a blank window
- **Verify**: `cargo run` opens a native window that can be closed ✓

### D2. Add a two-pane layout with egui
- [x] In `app.rs`, use `egui::SidePanel::left` for the code editor pane
- [x] Use `egui::CentralPanel` for the viewport pane (placeholder text for now)
- [x] Add a basic `TextEdit::multiline` in the left pane for script input
- [x] Add a "Run" button below the text editor
- **Verify**: `cargo run` shows a window with a text editor on the left, placeholder on the right, and a Run button ✓

### D3. Set up a custom wgpu rendering surface in the viewport pane
- [x] In the central panel, use `egui::Frame::canvas` to reserve a rect for custom painting
- [x] Use eframe's `egui_wgpu` integration to register a custom render callback
- [x] Render meshes with wgpu pipeline within egui
- **Verify**: `cargo run` — right pane shows rendered meshes ✓

### D4. Write the mesh vertex/fragment shader
- [x] Create `shaders/mesh.wgsl`
- [x] Vertex shader: takes position + normal, applies model-view-projection transform, passes normal to fragment
- [x] Fragment shader: lighting with one directional light + ambient
- [x] Model color: light gray
- **Verify**: Shader working with proper lighting ✓

### D5. Create the render pipeline
- [x] In `src/render/pipeline.rs`, set up:
  - Shader module from `mesh.wgsl`
  - Vertex buffer layout matching `Vertex` struct
  - Render pipeline compatible with egui's render pass
  - Uniform buffer for MVP matrix + light direction
- [x] Expose a `RenderState` struct with upload_mesh and update_uniforms methods
- **Verify**: Pipeline rendering meshes successfully ✓

### D6. Implement camera struct and projection
- [x] In `src/render/camera.rs`, implement `Camera` with:
  - `eye: Vec3`, `target: Vec3`, `up: Vec3`
  - `fov`, `aspect`, `near`, `far`
  - `view_matrix()` and `projection_matrix()` methods using glam
- [x] Default camera: looking at origin from (0, -50, 30), up = (0,0,1)
- [x] Add a method `view_projection(&self) -> Mat4`
- **Verify**: Camera working with unit tests ✓

### D7. Render a hardcoded triangle
- [x] Skipped in favor of direct mesh rendering
- **Verify**: N/A - proceeded directly to mesh rendering ✓

### D8. Render a hardcoded mesh from marching cubes
- [x] Mesh generated from script evaluation
- [x] Upload the mesh vertices and indices to GPU buffers
- [x] Use the camera's VP matrix as the uniform
- [x] Draw the indexed mesh
- **Verify**: `cargo run` — meshes appear in the viewport with lighting ✓

### D9. Implement orbit camera controls
- [x] In `src/render/camera.rs`, add orbit camera methods:
  - `orbit(delta_x, delta_y)` — rotate around target
  - `pan(delta_x, delta_y)` — translate target and eye
  - `zoom(delta)` — move eye closer/further from target
- [x] In `app.rs`, capture mouse input in the viewport rect:
  - Left drag → orbit
  - Right drag → pan
  - Scroll → zoom
- **Verify**: `cargo run` — can orbit, pan, and zoom around meshes with mouse ✓

---

## Stage E: Scripting Engine

### E1. Initialize Rhai engine with SDF functions
- [x] In `src/scripting/mod.rs`, create a function `create_engine() -> Engine`
- [x] Register a custom Rhai type to represent SDF handles (e.g., wrapping `Arc<dyn Sdf>` in a newtype)
- **Verify**: `cargo build` succeeds ✓

### E2. Register primitive constructors
- [x] In `src/scripting/api.rs`, register:
  - `sphere(radius: f64) -> SdfHandle`
  - `box_(width: f64, height: f64, depth: f64) -> SdfHandle` (or `cube` — decide naming)
  - `cylinder(radius: f64, height: f64) -> SdfHandle`
- [x] Note: Rhai uses `f64` natively; convert to `f32` internally
- **Verify**: `cargo test` — evaluate `let s = sphere(5.0); s` in Rhai, confirm it returns an SdfHandle ✓

### E3. Register boolean functions
- [x] Register:
  - `union(a: SdfHandle, b: SdfHandle) -> SdfHandle`
  - `subtract(a: SdfHandle, b: SdfHandle) -> SdfHandle`
  - `intersect(a: SdfHandle, b: SdfHandle) -> SdfHandle`
- **Verify**: `cargo test` — evaluate `subtract(box_(10.0,10.0,10.0), sphere(6.0))`, confirm valid handle returned ✓

### E4. Register transform functions
- [x] Register:
  - `translate(body: SdfHandle, x: f64, y: f64, z: f64) -> SdfHandle`
  - `rotate(body: SdfHandle, rx: f64, ry: f64, rz: f64) -> SdfHandle` (degrees)
  - `scale(body: SdfHandle, sx: f64, sy: f64, sz: f64) -> SdfHandle`
- **Verify**: `cargo test` — evaluate a script with translate + rotate, confirm valid handle ✓

### E5. Script evaluation returns an SDF
- [x] Write a function `evaluate_script(source: &str) -> Result<Arc<dyn Sdf>, String>`
- [x] The function creates the engine, evaluates the script, extracts the final expression as an SdfHandle
- [x] On error, return a human-readable error string
- **Verify**: `cargo test` — evaluate the example script from MasterPlan, confirm it returns a valid SDF that can be queried for distance ✓

---

## Stage F: Pipeline Integration

### F1. Wire Run button to script evaluation
- [x] In `app.rs`, when the Run button is clicked:
  1. Take the text from the code editor
  2. Call `evaluate_script(text)`
  3. On success: store the resulting `Arc<dyn Sdf>` in app state
  4. On error: store the error string in app state
- **Verify**: `cargo run` — type a script, click Run, no crash (output not visible yet) ✓

### F2. Wire SDF result to mesh extraction
- [x] After successful script evaluation:
  1. Compute a reasonable bounding box (start with fixed bounds, e.g., -50..50 on all axes)
  2. Run marching cubes at a fixed resolution (e.g., 32³)
  3. Store the resulting Mesh in app state
- **Verify**: `cargo run` — type `sphere(10.0)`, click Run, mesh is generated ✓

### F3. Wire mesh to renderer
- [x] After mesh extraction, display mesh stats in viewport
- [x] Upload vertex/index buffers to GPU and render
- **Verify**: Mesh stats displayed and 3D rendering working ✓

### F4. Display script errors in the UI
- [x] If script evaluation returns an error:
  - Display the error text in a red-highlighted area below the code editor
  - Do not clear the previous mesh (keep last successful result visible)
- [x] If evaluation succeeds, clear any previous error
- **Verify**: Error handling working ✓

### F5. End-to-end integration test
- [x] All unit tests pass (15/15)
- [x] Script → SDF → Mesh pipeline working
- [ ] Visual verification with full example script (requires 3D rendering)
- **Verify**: `cargo test` — all 15 tests pass ✓

---

## Stage G: Polish & Hardening

### G1. Auto-fit camera to mesh bounds
- [x] After mesh generation, compute the bounding box of the mesh vertices
- [x] Position the camera to frame the entire model (set target to bbox center, eye at appropriate distance)
- **Verify**: `cargo run` — small and large models both appear centered and visible ✓

### G2. Add grid or axis indicator to viewport
- [x] Render a simple ground grid for spatial reference
- [x] Keep it subtle — thin lines, muted colors with alpha blending
- **Verify**: `cargo run` — grid visible, doesn't obscure the model ✓

### G3. Add status feedback
- [x] Show mesh stats after success: "Mesh: X vertices, Y triangles"
- [x] Show timing: "Evaluated in X ms, meshed in Y ms"
- **Verify**: `cargo run` — status info appears after clicking Run ✓

### G4. Handle edge cases
- [x] Empty script → clear mesh, no error
- [x] Script that returns a non-SDF value → meaningful error message (via Rhai)
- [x] Very large or very small geometry → still renders (camera auto-fit)
- [x] Zero-volume results → empty mesh, no crash
- **Verify**: Edge cases handled gracefully ✓

### G5. Final verification
- [x] `cargo test` — all 16 unit tests pass
- [x] `cargo build --release` — release build succeeds
- [x] 3D rendering with lighting working
- [x] Grid rendering for spatial reference
- [x] Camera controls (orbit, pan, zoom)
- [x] Status feedback and timing
- [x] Edge case handling
- **Verify**: Phase 1 core functionality complete ✓

---

## Summary

| Stage | Steps | Description                          |
| ----- | ----- | ------------------------------------ |
| A     | 3     | Project scaffolding                  |
| B     | 11    | SDF geometry kernel                  |
| C     | 6     | Mesh extraction (marching cubes)     |
| D     | 9     | Renderer + viewer                    |
| E     | 5     | Scripting engine                     |
| F     | 5     | Pipeline integration                 |
| G     | 5     | Polish & hardening                   |
| **Total** | **44** | **Steps to complete Phase 1**    |
