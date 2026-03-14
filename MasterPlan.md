# MasterPlan.md — Implicit CAD Modeler

This is the authoritative source of truth for the project.
All implementation decisions, scope boundaries, and architectural direction live here.

---

## 1. Project Summary

A **code-first implicit CAD modeler** that uses signed distance fields (SDFs) to define geometry.
Users write scripts to construct models through primitives, booleans, and transforms.
A native desktop viewer renders the result in a two-pane layout (code left, 3D viewport right).

Inspired by **nTop** (SDF-based modeling) and **OpenSCAD** (code-first UI).

**This is an implicit modeler, not a code-based traditional CAD tool.**
Geometry is represented as mathematical fields (signed distance functions), not boundary representations (B-Rep) or meshes.
A sphere is `f(x,y,z) = sqrt(x² + y² + z²) - r`, not a collection of triangles.
Meshes are only generated at the final step for display/export via marching cubes.
This enables capabilities impossible in B-Rep: smooth field-driven blends, variable offsets, lattice infills, and topology-independent booleans with no edge-case failures.

---

## 2. Technology Stack

| Component          | Choice             | Rationale                                                        |
| ------------------ | ------------------ | ---------------------------------------------------------------- |
| **Language**        | Rust               | Performance for SDF eval, memory safety, cargo ecosystem         |
| **Renderer**        | wgpu               | Maps to DX12 on Windows, modern GPU abstraction, cross-plat ready |
| **UI Framework**    | egui (via eframe)  | Immediate-mode UI, trivial two-pane layout, native desktop app   |
| **Scripting**       | Rhai               | Rust-native embedding, intuitive JS-like syntax, easy type exposure |
| **Mesh Extraction** | Marching Cubes     | Standard SDF-to-mesh algorithm, well understood, improvable later |
| **Target OS**       | Windows            | Primary target. Architecture allows cross-platform expansion later |
| **Build System**    | Cargo              | Standard Rust toolchain                                          |

### Why Rust + Rhai?

- **Rust** gives us the performance needed for evaluating SDFs at millions of points (marching cubes, future real-time preview) without the footguns of C++.
- **Rhai** provides an embedded scripting language with clean syntax that looks natural for geometry definition:

```rhai
let base = box(40.0, 20.0, 10.0);
let hole = cylinder(5.0, 12.0);
let hole = translate(hole, 10.0, 0.0, 0.0);
let result = subtract(base, hole);
let result = rotate(result, 0.0, 0.0, 45.0);
```

No semicolons required. Familiar variable assignment and function calls.

### Why wgpu + egui?

- **wgpu** abstracts over DirectX 12/Vulkan/Metal — we target DX12 on Windows now but can go cross-platform trivially.
- **egui** is an immediate-mode UI library ideal for tool UIs. The two-pane code+viewport layout, dropdown menus, and parameter panels are all straightforward.

---

## 3. Architecture Overview

```
┌─────────────────────────────────────────────────┐
│                   Application                    │
│  ┌──────────────┐          ┌──────────────────┐  │
│  │  Code Editor  │          │   3D Viewport    │  │
│  │   (egui)      │          │   (wgpu)         │  │
│  └──────┬───────┘          └────────▲─────────┘  │
│         │                           │             │
│         ▼                           │             │
│  ┌──────────────┐          ┌────────┴─────────┐  │
│  │ Rhai Script   │          │  Mesh Renderer   │  │
│  │ Engine        │          │  (flat shading)  │  │
│  └──────┬───────┘          └────────▲─────────┘  │
│         │                           │             │
│         ▼                           │             │
│  ┌──────────────┐          ┌────────┴─────────┐  │
│  │ Geometry      │          │  Marching Cubes  │  │
│  │ Kernel (SDF)  │─────────▶│  Mesh Extraction │  │
│  └──────────────┘          └──────────────────┘  │
└─────────────────────────────────────────────────┘
```

### Data Flow

1. User writes script in the code editor pane
2. Script is evaluated by the Rhai engine
3. Rhai calls into the Geometry Kernel, producing an SDF tree
4. Marching Cubes extracts a triangle mesh from the SDF
5. The Mesh Renderer displays the mesh in the 3D viewport with flat shading
6. User can orbit/pan/zoom the camera to inspect the model

### Module Boundaries

| Module             | Responsibility                                    | Depends On      |
| ------------------ | ------------------------------------------------- | --------------- |
| `sdf`              | SDF primitives, booleans, transforms              | (none)          |
| `mesh`             | Marching cubes mesh extraction                     | `sdf`           |
| `render`           | wgpu renderer, camera, shading                     | `mesh`          |
| `scripting`        | Rhai engine setup, function registration           | `sdf`           |
| `ui`               | egui layout, code editor, viewport integration     | `render`, `scripting` |
| `app`              | Application entry point, event loop, state mgmt    | all             |

---

## 4. Phased Implementation Plan

### Phase 1: End-to-End Thin Slice (CURRENT)

**Goal**: User writes a script defining primitives + booleans + transforms. A window shows the code on the left and the rendered mesh on the right. This proves the entire pipeline works.

**Acceptance Criteria**:
- Native window opens with two-pane layout
- Code editor pane on the left with syntax highlighting
- 3D viewport on the right with orbit/pan/zoom controls
- User can type a script using `box()`, `sphere()`, `cylinder()` primitives
- Boolean operations: `union()`, `subtract()`, `intersect()`
- Transforms: `translate()`, `rotate()`, `scale()`
- Pressing a "Run" button (or keyboard shortcut) evaluates the script
- Mesh appears in the viewport with flat shading
- Errors in the script display clearly (not a crash)

**Detailed Steps** (to be expanded in `tasks/todo.md`):

1. **Project setup** — cargo init, add dependencies (wgpu, egui/eframe, rhai, glam)
2. **SDF primitives** — sphere, box, cylinder as Rust types implementing an SDF trait
3. **SDF booleans** — union, subtract, intersect operating on SDF trait objects
4. **SDF transforms** — translate, rotate, scale wrapping SDF trait objects
5. **Marching cubes** — mesh extraction from an SDF over a bounding volume
6. **wgpu renderer** — flat-shaded triangle mesh renderer with a directional light
7. **Camera controls** — orbit, pan, zoom via mouse input
8. **egui app shell** — two-pane window with code editor and viewport
9. **Rhai integration** — register SDF functions, evaluate scripts, return SDF tree
10. **Pipeline wiring** — script evaluation triggers mesh extraction and re-render
11. **Error display** — script errors shown in the UI, not panics
12. **Polish & verify** — test with representative scripts, fix edge cases

---

### Phase 2: Modeling Depth (FUTURE)

Expand the geometry kernel with more operations:

- Offset / shell operations
- Fillet and chamfer (via SDF blending)
- Linear and polar patterns
- Symmetry operations
- More primitives (torus, cone, wedge, etc.)
- Improved mesh quality (adaptive marching cubes or dual contouring)

---

### Phase 3: Viewer Polish (FUTURE)

Improve the viewer to feel like a real CAD tool:

- Smooth shading with proper normals
- Edge display / wireframe overlay
- Grid and axis indicators
- Selection and highlighting
- Measurement tools
- Coarse-then-fine progressive rendering

---

### Phase 4: UI Features (FUTURE)

Add the "blocks" UI and richer editing:

- Dropdown/palette to insert predefined code blocks (fuselage, wing, cube, etc.)
- Parameter panels for selected objects
- Script auto-completion
- Undo/redo
- Save/load project files

---

### Phase 5: Export & Integration (FUTURE)

- STL export
- OBJ export
- Step file consideration
- Batch/headless mode for CI/automation

---

### Phase 6: Advanced (FUTURE)

- Node graph editor (visual DAG editing alongside text scripting)
- GPU-accelerated SDF evaluation
- Lattice generation
- Field-driven operations
- Optimization and simulation hooks
- additional features for aerospace use (NACA airfoil library, fuselage, wing, tail code, etc)
---

## 5. Rendering Specification (Phase 1)

Target look: **standard CAD viewport**.

- Background: light gray gradient (top lighter, bottom slightly darker)
- Shading: flat per-face with a single directional light + ambient
- Model color: neutral mid-gray with slight blue tint (#B0B8C0 or similar)
- No textures, no PBR
- Camera: perspective projection, orbit around model center
- Controls: left-drag = orbit, right-drag = pan, scroll = zoom

---

## 6. Scripting API (Phase 1)

Functions exposed to Rhai:

### Primitives
- `sphere(radius)` — centered at origin
- `box(width, height, depth)` — centered at origin
- `cylinder(radius, height)` — centered at origin, along Z axis

### Booleans
- `union(a, b)` — combines two bodies
- `subtract(a, b)` — removes b from a
- `intersect(a, b)` — keeps only the overlap

### Transforms
- `translate(body, x, y, z)` — moves body
- `rotate(body, rx, ry, rz)` — rotates body (degrees, Euler XYZ)
- `scale(body, sx, sy, sz)` — scales body

### Example Script
```rhai
let base = box(40.0, 20.0, 10.0);
let hole = cylinder(4.0, 12.0);
let hole = translate(hole, 10.0, 5.0, 0.0);
let part = subtract(base, hole);

let sphere1 = sphere(6.0);
let sphere1 = translate(sphere1, -15.0, 0.0, 5.0);
let part = union(part, sphere1);

let part = rotate(part, 0.0, 0.0, 30.0);
part
```

The last expression in the script is the body that gets meshed and displayed.

---

## 7. Constraints & Non-Goals (Phase 1)

**In scope:**
- Single-body output (one SDF tree → one mesh)
- Flat shading only
- Basic orbit/pan/zoom camera
- Script-triggered re-evaluation (not live typing)
- Windows only

**Out of scope (for now):**
- Multi-body / assembly
- Undo/redo
- File save/load
- Export (STL, etc.)
- GPU SDF evaluation
- Node graph UI
- Cross-platform builds
- Auto-completion or syntax highlighting beyond basic

---

## 8. Dependencies (Phase 1)

```toml
[dependencies]
eframe = "0.29"          # egui desktop app framework
wgpu = "23"              # GPU rendering
rhai = "1"               # Embedded scripting
glam = "0.29"            # Math (vectors, matrices)
bytemuck = "1"           # Safe casting for GPU buffers
```

Exact versions will be pinned at project setup. These are approximate targets.

---

## 9. File Structure (Phase 1)

```
implicit-cad/
├── Cargo.toml
├── src/
│   ├── main.rs              # Entry point, eframe app setup
│   ├── app.rs               # Application state, egui layout
│   ├── sdf/
│   │   ├── mod.rs           # SDF trait definition
│   │   ├── primitives.rs    # Sphere, Box, Cylinder
│   │   ├── booleans.rs      # Union, Subtract, Intersect
│   │   └── transforms.rs    # Translate, Rotate, Scale
│   ├── mesh/
│   │   ├── mod.rs           # Mesh types
│   │   └── marching_cubes.rs # MC algorithm
│   ├── render/
│   │   ├── mod.rs           # Renderer setup
│   │   ├── camera.rs        # Camera + controls
│   │   └── pipeline.rs      # Render pipeline, shaders
│   └── scripting/
│       ├── mod.rs           # Rhai engine setup
│       └── api.rs           # Register SDF functions
├── shaders/
│   └── mesh.wgsl            # Vertex + fragment shader
```

---

## 10. Risk Register

| Risk                                  | Mitigation                                          |
| ------------------------------------- | --------------------------------------------------- |
| wgpu + egui integration complexity    | Use egui's built-in wgpu backend (eframe)           |
| Marching cubes mesh quality           | Start with simple uniform grid; improve later        |
| Rhai performance for complex scripts  | Scripts build an SDF tree, not evaluate fields; fast |
| Camera control feel                   | Use proven orbit camera math from reference impls    |
| Scope creep                           | MasterPlan defines Phase 1 boundary strictly         |
