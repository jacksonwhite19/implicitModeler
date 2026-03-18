# AGENT_REFERENCE.md — Implicit CAD Modeler

**Last updated**: 2026-03-17
**Purpose**: Comprehensive agent/developer reference for the Implicit CAD Modeler codebase. Read this before making any changes. Prefer MasterPlan.md for high-level goals and scope.

---

## Table of Contents

1. [Project Summary](#1-project-summary)
2. [Dependency Inventory](#2-dependency-inventory)
3. [Repository Layout](#3-repository-layout)
4. [Architecture Overview](#4-architecture-overview)
5. [Module Reference](#5-module-reference)
   - 5.1 [sdf — Geometry Kernel](#51-sdf--geometry-kernel)
   - 5.2 [mesh — Mesh Extraction](#52-mesh--mesh-extraction)
   - 5.3 [render — Rendering System](#53-render--rendering-system)
   - 5.4 [scripting — Rhai Engine](#54-scripting--rhai-engine)
   - 5.5 [ui — User Interface](#55-ui--user-interface)
   - 5.6 [app — Application State](#56-app--application-state)
   - 5.7 [aero — Aerodynamic Analysis](#57-aero--aerodynamic-analysis)
   - 5.8 [fea — Finite Element Analysis](#58-fea--finite-element-analysis)
   - 5.9 [analysis — Geometry Analysis](#59-analysis--geometry-analysis)
   - 5.10 [materials — Composite Materials](#510-materials--composite-materials)
   - 5.11 [components — Component Registry](#511-components--component-registry)
   - 5.12 [node_graph — Node Graph Editor](#512-node_graph--node-graph-editor)
   - 5.13 [notebook — Notebook Mode](#513-notebook--notebook-mode)
   - 5.14 [library — Component Library](#514-library--component-library)
   - 5.15 [version_control — Version Control](#515-version_control--version-control)
   - 5.16 [geometry_analysis — CG and Interference](#516-geometry_analysis--cg-and-interference)
   - 5.17 [project — Project File Management](#517-project--project-file-management)
   - 5.18 [settings — Application Settings](#518-settings--application-settings)
   - 5.19 [export — Export System](#519-export--export-system)
   - 5.20 [undo — Undo/Redo](#520-undo--undoredo)
   - 5.21 [headless — Headless Mode](#521-headless--headless-mode)
6. [Scripting API Reference](#6-scripting-api-reference)
   - 6.1 [Primitive Shapes](#61-primitive-shapes)
   - 6.2 [Boolean Operations](#62-boolean-operations)
   - 6.3 [Transforms](#63-transforms)
   - 6.4 [Patterns](#64-patterns)
   - 6.5 [Aerospace Shapes](#65-aerospace-shapes)
   - 6.6 [Structural and Mechanical](#66-structural-and-mechanical)
   - 6.7 [Composite Layup](#67-composite-layup)
   - 6.8 [Control Surfaces](#68-control-surfaces)
   - 6.9 [Print / Manufacturing](#69-print--manufacturing)
   - 6.10 [Fasteners and Panels](#610-fasteners-and-panels)
   - 6.11 [Field Operations](#611-field-operations)
   - 6.12 [Lattice Structures](#612-lattice-structures)
   - 6.13 [Sweep Operations](#613-sweep-operations)
   - 6.14 [Component Placement](#614-component-placement)
   - 6.15 [Aerodynamic Analysis](#615-aerodynamic-analysis)
   - 6.16 [Propulsion Analysis](#616-propulsion-analysis)
   - 6.17 [Mass and CG Tracking](#617-mass-and-cg-tracking)
   - 6.18 [FEA Setup](#618-fea-setup)
   - 6.19 [Reference Points and Queries](#619-reference-points-and-queries)
   - 6.20 [Mesh Import](#620-mesh-import)
   - 6.21 [Dimensions (Named Constants)](#621-dimensions-named-constants)
   - 6.22 [Script Cell System](#622-script-cell-system)
7. [Data Flow and Evaluation Pipeline](#7-data-flow-and-evaluation-pipeline)
8. [Rhai Type Handles](#8-rhai-type-handles)
9. [Project File Format](#9-project-file-format)
10. [Rendering Architecture](#10-rendering-architecture)
11. [FEA Pipeline Details](#11-fea-pipeline-details)
12. [Testing Conventions](#12-testing-conventions)
13. [Engineering Conventions](#13-engineering-conventions)
14. [Common Patterns](#14-common-patterns)
15. [Scope Boundaries and Non-Goals](#15-scope-boundaries-and-non-goals)
16. [Known Constraints and Gotchas](#16-known-constraints-and-gotchas)

---

## 1. Project Summary

The Implicit CAD Modeler is a code-first, SDF-based (signed distance field) parametric geometry system. Users write Rhai scripts to build geometry through primitives, booleans, and transforms. A native desktop viewer (egui + wgpu) renders the result in a two-pane layout: code editor on the left, 3D viewport on the right.

**Core philosophy**: geometry is a computable field, not a mesh. A sphere is `f(x,y,z) = length(xyz) - r`, not a collection of triangles. Meshes are extracted at the final step only, via marching cubes. This enables smooth SDF blends, variable offsets, and topology-independent booleans.

**Inspiration**: nTop (SDF-based modeling) and OpenSCAD (code-first UI).

**Primary target OS**: Windows (DX12 via wgpu). Architecture is cross-platform ready.

**Language**: Rust + Rhai scripting language.

---

## 2. Dependency Inventory

Dependencies from `Cargo.toml` (exact versions as declared):

### Runtime Dependencies

| Crate | Version | Features | Purpose |
|---|---|---|---|
| `eframe` | 0.29 | `wgpu` | egui desktop app framework with wgpu backend |
| `rhai` | 1.20 | — | Embedded scripting language for geometry definition |
| `glam` | 0.29 | — | SIMD math library (Vec2, Vec3, Quat, Mat4) |
| `bytemuck` | 1.18 | `derive` | Safe casting for GPU vertex buffers |
| `serde` | 1.0 | `derive` | Serialization/deserialization derive macros |
| `serde_json` | 1.0 | — | JSON project file I/O |
| `toml` | 0.8 | `parse` | TOML configuration file parsing |
| `chrono` | 0.4 | `serde` | Timestamps in project files and analysis results |
| `rfd` | 0.15 | — | Native file dialogs (open/save) |
| `clap` | 4.5 | `derive` | CLI argument parsing for headless mode |
| `lazy_static` | 1.4 | — | Static global initialization (polar database, etc.) |
| `rayon` | 1.10 | — | Data parallelism for marching cubes and analysis |
| `indexmap` | 2 | `serde` | Ordered maps for named dimensions |

### Dev Dependencies

| Crate | Version | Purpose |
|---|---|---|
| `tempfile` | 3.8 | Temporary files in tests |

### Notes on Key Dependencies

**eframe/egui**: The wgpu feature flag is mandatory — it switches egui's rendering backend from the default glow (OpenGL) to wgpu (DX12 on Windows). All custom rendering (mesh display, grid, axes, section view, raymarch preview) runs as custom wgpu passes inside the egui frame.

**rhai**: Scripts are evaluated in a fresh `Engine` per evaluation call. All SDF functions are registered as native Rust closures. The engine does NOT persist state across evaluations.

**glam**: All geometry uses `Vec3` (f32). The SDF trait operates on `Vec3`. Distance values are `f32`. Quaternions (`Quat`) are used for rotations internally; scripts take Euler angles in degrees which are converted in the API layer.

**rayon**: Used in marching cubes (`adaptive_mc`) to parallelize the grid evaluation. Also used in thickness computation and some analysis passes.

**indexmap**: Used for `dimensions` in the project file so that named constants appear in a consistent insertion-order in the UI.

---

## 3. Repository Layout

```
implicit-cad/
├── Cargo.toml                  # Package manifest and dependencies
├── Cargo.lock                  # Locked dependency versions
├── CLAUDE.md                   # AI coding agent guidelines (READ FIRST)
├── MasterPlan.md               # Authoritative project goals and scope
├── AGENT_REFERENCE.md          # This file — developer/agent reference
├── ProjectOverview.md          # High-level project description
├── README.md                   # User-facing readme
├── PROGRESS.md                 # Phase completion notes
├── PHASE7_COMPLETE.md          # Phase 7 completion record
├── PHASE8_COMPLETE.md          # Phase 8 completion record
├── scratchpad.md               # Scratch notes (not authoritative)
├── output.md                   # Output/log scratch file
├── shaders/
│   └── mesh.wgsl               # Vertex + fragment WGSL shader
├── src/
│   ├── main.rs                 # Binary entry point, CLI parsing
│   ├── lib.rs                  # Library crate root, module declarations
│   ├── app.rs                  # Application state struct + egui layout
│   ├── project.rs              # Project file (de)serialization
│   ├── settings.rs             # Application settings persistence
│   ├── headless.rs             # Headless batch evaluation mode
│   ├── undo.rs                 # Undo/redo infrastructure
│   ├── sdf/                    # Geometry kernel
│   │   ├── mod.rs              # Sdf trait definition
│   │   ├── primitives.rs       # Sphere, SdfBox, Cylinder, Torus, Cone, Plane
│   │   ├── booleans.rs         # Union, Subtract, Intersect, smooth variants
│   │   ├── transforms.rs       # Translate, Rotate, Scale, Offset, Shell, Twist, Bend
│   │   ├── patterns.rs         # LinearArray, PolarArray, Mirror
│   │   ├── profiles.rs         # SplineProfile (2D cross-section splines)
│   │   ├── spine.rs            # LongitudinalSplines (fuselage spine curves)
│   │   ├── sweep.rs            # Sweep operations, SweepPath trait
│   │   ├── mesh_import.rs      # Mesh-as-SDF (closest-point SDF from triangle mesh)
│   │   ├── query.rs            # SDF query utilities (bounding box, surface snapping)
│   │   ├── aerospace/          # Aerospace-specific geometry
│   │   │   ├── mod.rs
│   │   │   ├── section.rs      # Section2D trait (2D cross-sections)
│   │   │   ├── airfoil.rs      # NACA airfoil generation
│   │   │   ├── wing.rs         # Wing with airfoils and taper
│   │   │   ├── fuselage.rs     # Parametric lofted fuselage
│   │   │   ├── nacelle.rs      # Engine nacelle shapes
│   │   │   ├── nose_tail.rs    # Haack Series, Ogive, Ellipsoid noses/tails
│   │   │   ├── inlets.rs       # NACA inlets, EDF ducts, inlet lips
│   │   │   ├── structural.rs   # Wing ribs, spars
│   │   │   ├── structural_drone.rs # Drone-specific bulkheads, motor arms, mounts
│   │   │   ├── mechanical.rs   # Bolt circles, countersinks, counterbores, slots
│   │   │   ├── composite.rs    # CompositeSdf, CompositeLayup, ShellLayer
│   │   │   ├── control_surfaces.rs # Ailerons, elevators, rudders, flaps, elevons
│   │   │   └── stability_geometry.rs # Tail volume coefficients, planform area
│   │   ├── field/              # Scalar field operations
│   │   │   ├── mod.rs          # Field trait definition
│   │   │   ├── primitives.rs   # ConstantField, SdfField, position fields
│   │   │   ├── arithmetic.rs   # FieldAdd, FieldMultiply, FieldMin, FieldMax, FieldAbs
│   │   │   ├── gradients.rs    # GradientField, RadialField, AxialRadialField
│   │   │   ├── operations.rs   # OffsetByField, ShellWithField, BlendByField
│   │   │   └── lattice.rs      # GyroidLattice, CubicLattice, DiamondLattice, GyroidWithField
│   │   ├── lattice/            # Conformal lattice structures
│   │   │   ├── mod.rs
│   │   │   └── conformal.rs    # ConformalGyroid, ConformalDiamond, ConformalSchwarzP
│   │   └── print/              # 3D-print-specific geometry
│   │       ├── mod.rs          # SplitPlane, AlignmentFeature, split_body
│   │       ├── alignment.rs    # Alignment pin/socket features
│   │       ├── bracket.rs      # Mounting bracket geometry
│   │       ├── fasteners.rs    # Screw holes, heat-set inserts, bosses
│   │       ├── joints.rs       # JointDelta (tolerance-aware joints)
│   │       ├── panels.rs       # Access panels, battery hatches
│   │       ├── split.rs        # Multi-plane body splitting
│   │       └── tolerance.rs    # ToleranceSettings, ToleranceCompensated
│   ├── mesh/
│   │   ├── mod.rs              # Vertex, Mesh types; compute_volume
│   │   ├── marching_cubes.rs   # Standard marching cubes algorithm
│   │   ├── adaptive_mc.rs      # Adaptive marching cubes, MeshQuality enum
│   │   └── import.rs           # STL/OBJ parsing, TriangleMesh, validate_mesh
│   ├── render/
│   │   ├── mod.rs
│   │   ├── camera.rs           # Orbit camera, pan/zoom
│   │   ├── pipeline.rs         # wgpu render pipeline, RenderState
│   │   ├── grid.rs             # Ground grid renderer
│   │   ├── axes.rs             # XYZ axis indicator renderer
│   │   ├── wireframe.rs        # Wireframe overlay renderer
│   │   └── raymarch.rs         # SDF raymarching renderer, section view, thickness viz
│   ├── scripting/
│   │   ├── mod.rs              # Engine setup, ScriptResult, all handle types
│   │   ├── api.rs              # All register_* functions (very large file)
│   │   └── errors.rs           # Script error formatting and display
│   ├── ui/
│   │   ├── mod.rs              # UI panel dispatch
│   │   ├── autocomplete.rs     # Code editor autocomplete
│   │   ├── dimensions.rs       # Named dimensions panel
│   │   ├── library_panel.rs    # Component library browser panel
│   │   ├── project_tree.rs     # Project tree / structure panel
│   │   ├── project_wizard.rs   # New project wizard
│   │   ├── script_variable_detector.rs # Detect Rhai variables from script text
│   │   ├── spine_editor.rs     # Longitudinal spine editor
│   │   ├── spline_editor.rs    # 2D spline profile editor
│   │   ├── syntax.rs           # Syntax highlighting for code editor
│   │   ├── templates.rs        # Script template insertion
│   │   ├── version_control_panel.rs # Version control UI panel
│   ├── aero/
│   │   ├── mod.rs
│   │   ├── polars.rs           # AirfoilPolar, PolarDatabase
│   │   ├── polar_data.rs       # NACA polar generation
│   │   ├── flight_condition.rs # FlightCondition struct (altitude, airspeed, etc.)
│   │   ├── lifting_line.rs     # Lifting line theory solver
│   │   ├── stability.rs        # Neutral point, static margin, trim analysis
│   │   ├── drag.rs             # Drag polar, CD0 breakdown
│   │   ├── inlet_analysis.rs   # Inlet aerodynamic analysis
│   │   ├── propulsion_db.rs    # Motor/prop database
│   │   ├── propulsion.rs       # Thrust curves, power, efficiency
│   │   └── performance.rs      # Rate of climb, range, endurance, glide
│   ├── fea/
│   │   ├── mod.rs
│   │   ├── setup.rs            # FEASetup, FEAConfig, boundary condition types
│   │   ├── meshing.rs          # Voxel-to-tetrahedral mesh generation
│   │   ├── inp.rs              # CalculiX .inp file writer
│   │   ├── frd.rs              # CalculiX .frd result reader
│   │   ├── calculix.rs         # CalculiX subprocess runner
│   │   ├── pipeline.rs         # FEAPipeline orchestration, FEAGridResult, GridField
│   │   └── viz.rs              # FEA result visualization data
│   ├── analysis/
│   │   ├── mod.rs
│   │   ├── thickness.rs        # Thickness map computation
│   │   ├── measurements.rs     # Volume, cross-section, distance measurements
│   │   ├── print_analysis.rs   # Overhang, wall thickness, print orientation analysis
│   │   ├── hole_detection.rs   # Automatic mounting hole detection
│   │   └── aero/               # Aerodynamic analysis from geometry
│   │       ├── mod.rs
│   │       └── inlet_analysis.rs
│   ├── materials/
│   │   ├── mod.rs
│   │   └── composite.rs        # CompositeMaterial definition
│   ├── geometry_analysis/
│   │   ├── mod.rs
│   │   ├── cg_sensitivity.rs   # CG sensitivity to dimension changes
│   │   └── interference.rs     # Assembly interference detection
│   ├── components/
│   │   ├── mod.rs
│   │   ├── library.rs          # Built-in component library entries
│   │   └── registry.rs         # ComponentRegistry, ComponentInstance
│   ├── node_graph/
│   │   ├── mod.rs
│   │   ├── types.rs            # Node types and graph data structures
│   │   ├── codegen.rs          # Node graph to Rhai code generation
│   │   └── ui.rs               # Node graph egui editor UI
│   ├── notebook/
│   │   ├── mod.rs
│   │   ├── types.rs            # Notebook and cell types
│   │   ├── codegen.rs          # Notebook to script conversion
│   │   └── ui.rs               # Notebook egui editor UI
│   ├── library/
│   │   ├── mod.rs
│   │   ├── manager.rs          # LibraryManager (load/save user library)
│   │   ├── metadata.rs         # Library item metadata
│   │   └── thumbnail.rs        # Library item thumbnail generation
│   └── version_control/
│       ├── mod.rs
│       └── operations.rs       # Snapshot, diff, restore operations
├── tasks/
│   ├── liveStatus.md           # Current execution status (agent progress channel)
│   ├── todo.md                 # Active task checklist
│   ├── lessons.md              # Lessons learned from mistakes
│   └── phase*.md               # Per-phase task files
├── tests/                      # Integration test files
├── examples/                   # Example Rhai scripts
└── test_scripts/               # Test Rhai scripts
```

---

## 4. Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                     Application (app.rs)                     │
│  ┌─────────────────┐              ┌───────────────────────┐  │
│  │   Code Editor   │              │     3D Viewport       │  │
│  │   (egui text)   │              │  (wgpu custom pass)   │  │
│  └────────┬────────┘              └──────────▲────────────┘  │
│           │                                  │               │
│           ▼                                  │               │
│  ┌─────────────────┐              ┌──────────┴────────────┐  │
│  │  Rhai Engine    │              │    Render System       │  │
│  │  (scripting/)   │              │  Mesh / Raymarch /     │  │
│  └────────┬────────┘              │  Grid / Axes / Wire    │  │
│           │                       └──────────▲────────────┘  │
│           ▼                                  │               │
│  ┌─────────────────┐              ┌──────────┴────────────┐  │
│  │  Geometry Kernel│              │   Mesh Extraction     │  │
│  │  (sdf/)        │─────────────▶│  Marching Cubes       │  │
│  └─────────────────┘              │  (mesh/)              │  │
│                                   └───────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐    │
│  │  Analysis: aero/ fea/ analysis/ geometry_analysis/   │    │
│  └──────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

### Data Flow (Normal Evaluation)

1. User writes or edits a Rhai script in the code editor pane.
2. User triggers evaluation (Run button or Ctrl+Enter).
3. `scripting::evaluate_script_full()` creates a fresh Rhai `Engine`, registers all SDF functions, evaluates the script.
4. The script builds an SDF tree (a tree of `Arc<dyn Sdf>` nodes). The last expression is the root.
5. `ScriptResult` is returned containing: `sdf`, `mass_points`, `fea_setup`, `layups`, `reference_points`.
6. `adaptive_mc::extract_mesh()` runs marching cubes on the SDF over a bounding volume. Parallelized via rayon.
7. The resulting `Mesh` (vertices + indices) is uploaded to the GPU as a wgpu vertex/index buffer.
8. The render loop draws the mesh with flat or smooth shading, plus optional overlays (wireframe, grid, axes, section clip planes, thickness map).

### Module Dependency Graph

```
sdf         (no deps on other project modules)
mesh        → sdf
render      → mesh (for Vertex type), glam
scripting   → sdf, mesh, aero, fea, materials, analysis
app         → scripting, render, mesh, project, ui, components, node_graph, notebook, undo, library
aero        (no deps on other project modules)
fea         → sdf, mesh, aero, materials
analysis    → sdf, mesh, geometry_analysis
geometry_analysis → sdf
materials   (no deps on other project modules)
components  (no deps on other project modules)
```

---

## 5. Module Reference

### 5.1 `sdf` — Geometry Kernel

**Path**: `src/sdf/`

#### Core Trait

```rust
pub trait Sdf: Send + Sync {
    fn distance(&self, point: Vec3) -> f32;
}
```

All geometry is represented as types implementing `Sdf`. The `distance` function returns:
- Negative value: point is **inside** the shape
- Zero: point is **on the surface**
- Positive value: point is **outside** the shape

All SDF types are wrapped in `Arc<dyn Sdf>` for shared ownership. This is the fundamental composition mechanism — booleans and transforms hold `Arc<dyn Sdf>` children.

#### Primitives (`sdf/primitives.rs`)

| Type | Constructor | Description |
|---|---|---|
| `Sphere` | `Sphere::new(radius: f32)` | Sphere centered at origin |
| `SdfBox` | `SdfBox::new(half_extents: Vec3)` | Box centered at origin; half_extents are half-dimensions |
| `Cylinder` | `Cylinder::new(radius: f32, half_height: f32)` | Cylinder along Z axis |
| `Torus` | `Torus::new(major_radius: f32, minor_radius: f32)` | Torus in XY plane |
| `Cone` | `Cone::new(radius: f32, height: f32)` | Cone, tip at origin, base in -Z direction |
| `Plane` | `Plane::new(normal: Vec3, distance: f32)` | Infinite half-space |

**Note on `SdfBox`**: The Rhai API takes `width, height, depth` as full dimensions and divides by 2 internally. Rust code takes `half_extents` directly.

**Note on `Cylinder`**: The Rhai API takes `height` as total height; the constructor takes `half_height`.

#### Booleans (`sdf/booleans.rs`)

| Type | Formula | Description |
|---|---|---|
| `Union` | `min(a, b)` | Combines two bodies |
| `Subtract` | `max(a, -b)` | Removes b from a |
| `Intersect` | `max(a, b)` | Keeps only the overlap |
| `SmoothUnion` | IQ polynomial smin | Smooth union with blend radius |
| `SmoothSubtract` | IQ opSmoothSubtraction | Smooth removal with chamfer |
| `SmoothIntersect` | IQ polynomial smax | Smooth intersection |

The smooth operations use Inigo Quilez's polynomial blending formulas. The `k` / `smoothness` parameter controls the blend radius in scene units.

#### Transforms (`sdf/transforms.rs`)

| Type | Constructor | Description |
|---|---|---|
| `Translate` | `Translate::new(child, offset: Vec3)` | Translate by offset vector |
| `Rotate` | `Rotate::new(child, rotation: Quat)` | Rotate by quaternion (inverse applied to query point) |
| `Scale` | `Scale::new(child, scale: Vec3)` | Non-uniform scale (min scale used for distance correction) |
| `Offset` | `Offset::new(child, distance: f32)` | Expand (positive) or shrink (negative) by distance |
| `Shell` | `Shell::new(child, thickness: f32)` | Hollow shell — keeps surface band of given thickness |
| `Twist` | `Twist::new(child, axis: Vec3, rate: f32)` | Twist around axis by `rate` degrees/unit (approximate SDF) |
| `Bend` | `Bend::new(child, axis: Vec3, curvature: f32)` | Bend along axis by curvature rad/unit (approximate SDF) |

**Important**: `Twist` and `Bend` produce approximate (non-Lipschitz-1) SDFs. Do not use them for precise offset operations. Safe for marching cubes and raymarching at moderate deformation.

#### Patterns (`sdf/patterns.rs`)

- `LinearArray`: copies an SDF at regular intervals along a direction vector
- `PolarArray`: copies an SDF in a radial pattern around an axis
- `Mirror`: reflects an SDF across a plane (defined by normal)

#### Aerospace (`sdf/aerospace/`)

Large collection of aerospace-specific geometry. See Section 6.5 for scripting API.

Key types:
- `Section2D` trait: a 2D cross-section shape usable as a fuselage station
- `Airfoil`: NACA 4-digit airfoil SDF (parameterized by digits string)
- `ExtrudedAirfoil`: airfoil extruded to a span
- `LoftedFuselage`: lofted body from a sequence of (position, cross-section) stations
- `CrossSection`: parametric fuselage cross-section (circle, ellipse, rounded rect)
- `HaackNose`, `HaackTail`, `TangentOgive`, `EllipsoidNose`: low-drag nose/tail shapes
- `NacaInlet`, `InletLip`, `EdfDuct`: air inlet geometry
- `ControlSurface`, `ControlSurfaceResult`: ailerons, elevators, rudders, flaps, elevons

#### Field System (`sdf/field/`)

Scalar fields implement the `Field` trait:
```rust
pub trait Field: Send + Sync {
    fn value(&self, point: Vec3) -> f32;
}
```

Field types:
- `ConstantField(f32)`: uniform scalar value everywhere
- `SdfField`: wraps an SDF, uses its distance value as the field
- `PositionXField`, `PositionYField`, `PositionZField`: use coordinate axis as field
- `FieldAdd`, `FieldMultiply`, `FieldMin`, `FieldMax`, `FieldAbs`: arithmetic on fields
- `GradientField`: field that varies linearly along a direction
- `RadialField`: field that varies with radial distance from an axis
- `AxialRadialField`: combined axial and radial gradient
- `OffsetByField`: offset an SDF by a field value at each point (variable thickness offset)
- `ShellWithField`: variable-thickness shell
- `BlendByField`: blend two SDFs with a field-driven weight
- `GyroidLattice`, `CubicLattice`, `DiamondLattice`: unit-cell lattice fields
- `GyroidWithField`: gyroid lattice with field-driven cell size modulation

#### Lattice (`sdf/lattice/`)

- `ConformalGyroid`: gyroid lattice conformally mapped to a body's local frame
- `ConformalDiamond`: diamond lattice conformal variant
- `ConformalSchwarzP`: Schwarz-P lattice conformal variant

#### Print (`sdf/print/`)

Manufacturing-specific geometry:
- `SplitPlane`: axis-aligned split for printing large parts
- `AlignmentFeature`: alignment pins/sockets for multi-part prints
- `ToleranceSettings`: printer preset tolerance values (layer height, xy shrink, etc.)
- `ToleranceCompensated`: wraps an SDF and applies tolerance compensation
- `JointDelta`: tolerance-aware snap-fit joints
- Fastener functions: `clearance_hole`, `countersink_hole`, `heat_set_boss`
- Panel functions: `AccessPanel`, battery hatches, FC access cutouts

---

### 5.2 `mesh` — Mesh Extraction

**Path**: `src/mesh/`

#### Types

```rust
pub struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
}

pub struct Mesh {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
}
```

`Vertex` is `#[repr(C)]` and derives `Pod + Zeroable` (bytemuck) so it can be uploaded directly to GPU buffers without copying.

#### `compute_volume(mesh: &Mesh) -> f32`

Computes signed mesh volume using the divergence theorem (signed tetrahedral volumes summed over all triangles). Units are scene units cubed (mm³ if scene is in mm).

#### Marching Cubes (`mesh/marching_cubes.rs`)

Standard uniform-grid marching cubes. Evaluates the SDF at each grid vertex, interpolates crossing points, generates triangles. Used for preview at coarse resolution.

#### Adaptive Marching Cubes (`mesh/adaptive_mc.rs`)

Parallel adaptive marching cubes with quality settings:

```rust
pub enum MeshQuality {
    Draft,    // fast, low poly
    Standard, // balanced
    Fine,     // high quality
    Ultra,    // maximum detail
}
```

This is the primary mesh extraction path used by the app. Parallelized with rayon.

#### Mesh Import (`mesh/import.rs`)

```rust
pub struct TriangleMesh {
    pub vertices: Vec<Vec3>,
    pub triangles: Vec<[u32; 3]>,
}

pub struct MeshValidationResult { ... }

pub fn parse_stl(data: &[u8]) -> Result<TriangleMesh, String>;
pub fn parse_obj(data: &str) -> Result<TriangleMesh, String>;
pub fn validate_mesh(mesh: &TriangleMesh) -> MeshValidationResult;
```

Used by `sdf/mesh_import.rs` to create closest-point SDFs from imported geometry, and by the scripting API's `import_mesh()` function. Mesh parse results are cached in `MeshCache` (keyed by path + mtime).

---

### 5.3 `render` — Rendering System

**Path**: `src/render/`

All rendering is done via wgpu as custom rendering passes inside eframe's egui integration. The render system does not use egui's painter; it uses raw wgpu commands.

#### `Camera` (`render/camera.rs`)

Orbit camera around a target point:
- Left-drag: orbit
- Right-drag: pan
- Scroll: zoom

Exposes view matrix, projection matrix, and `eye_position()`.

#### `RenderState` (`render/pipeline.rs`)

Core wgpu state: device, queue, surface, render pipeline for mesh drawing. Handles vertex/index buffer management, shader loading from `shaders/mesh.wgsl`.

Shading modes:
- Flat shading (per-face normals)
- Smooth shading (per-vertex normals from marching cubes)

#### `GridRenderer` (`render/grid.rs`)

Renders an infinite grid plane in the viewport.

#### `AxesRenderer` (`render/axes.rs`)

Renders XYZ axis indicators (colored arrows/lines in the viewport corner).

#### `WireframeRenderer` (`render/wireframe.rs`)

Renders the mesh as a wireframe overlay.

#### `RaymarchRenderer`, `SdfGrid`, `SectionUniforms`, `ThicknessUniforms` (`render/raymarch.rs`)

GPU raymarching renderer for:
- Real-time SDF preview (without full marching cubes)
- Section view clipping (slice through the model)
- Thickness visualization (heat map of wall thickness)

`SdfGrid` is a voxel grid representation of the SDF uploaded to the GPU as a 3D texture for raymarching.

---

### 5.4 `scripting` — Rhai Engine

**Path**: `src/scripting/`

#### Engine Lifecycle

A new `rhai::Engine` is created on every script evaluation call. All functions are re-registered each time. This is intentional — it ensures no state leaks between evaluations and simplifies the design.

#### `evaluate_script(source: &str) -> Result<ScriptResult, String>`

Convenience wrapper. No profiles, spine, or FEA field access.

#### `evaluate_script_with_profiles(source, profiles, splines) -> Result<ScriptResult, String>`

Used by the app for scripts that use `spline()` or `spline_section()`.

#### `evaluate_script_full(...)` — Full Evaluator

```rust
pub fn evaluate_script_full(
    source:             &str,
    profiles:           Option<Arc<RwLock<HashMap<String, SplineProfile>>>>,
    splines:            Option<Arc<LongitudinalSplines>>,
    stress_field:       Option<Arc<dyn Field>>,
    displacement_field: Option<Arc<dyn Field>>,
    dimensions:         &indexmap::IndexMap<String, f64>,
    project_dir:        Option<&Path>,
    mesh_cache:         Option<Arc<Mutex<MeshCache>>>,
    library_sources:    &[(String, String)],
) -> Result<ScriptResult, String>
```

Parameters:
- `profiles`: resolves `spline(name)` and `spline_section(name)` calls
- `splines`: resolves `spline_fuselage(stations, length)` calls
- `stress_field` / `displacement_field`: FEA result fields available as `stress_field()` / `displacement_field()` in scripts
- `dimensions`: named constants injected as Rhai constants into the script scope
- `project_dir`: base directory for resolving relative mesh paths in `import_mesh()`
- `mesh_cache`: shared cache of parsed STL/OBJ meshes (keyed by path + mtime)
- `library_sources`: list of `(module_name, rhai_source)` pairs registered as static Rhai modules

#### `ScriptResult`

```rust
pub struct ScriptResult {
    pub sdf:              Arc<dyn Sdf>,
    pub mass_points:      Vec<MassPoint>,
    pub fea_setup:        FEASetup,
    pub layups:           Vec<Arc<CompositeLayup>>,
    pub reference_points: Vec<ReferencePoint>,
}
```

Methods:
- `center_of_gravity() -> Option<Vec3>`: weighted average of mass points
- `total_mass_g() -> f32`: sum of all declared masses

#### Script Cell System

Scripts can be divided into named sections using delimiter comments:
```
# === Section Name ===
```

`parse_cells(script: &str) -> Vec<ScriptCell>` parses these delimiters. If no delimiters are present, a single cell named "Script" covers the entire script.

```rust
pub struct ScriptCell {
    pub id: String,         // "cell_0", "cell_1", etc.
    pub name: String,       // human-readable name from delimiter
    pub start_line: usize,  // 0-indexed line of delimiter
    pub end_line: usize,    // 0-indexed inclusive last line
    pub status: CellStatus,
}

pub enum CellStatus {
    Pending,
    Ok,
    Error { message: String, line: Option<usize> },
    Skipped,
}
```

#### Error Formatting (`scripting/errors.rs`)

`format_script_error(source, message)` — enriches error messages with line context. `build_error_string(formatted)` — formats the final error string for display in the UI.

Special case: if a script ends with a semicolon on the last line, the return type is `()` instead of `SdfHandle`, and the engine produces a misleading "output type incorrect" error. The evaluator catches this pattern and produces a friendly "Script must end with an SDF expression" message.

---

### 5.5 `ui` — User Interface

**Path**: `src/ui/`

UI components are egui panels rendered from `app.rs`. The UI module provides specialized panel functions.

| File | Contents |
|---|---|
| `mod.rs` | Panel dispatch, overall UI layout helpers |
| `autocomplete.rs` | Code editor autocomplete popup |
| `dimensions.rs` | Named dimensions table editor panel |
| `library_panel.rs` | Component library browser panel, `LibraryPanelState` |
| `project_tree.rs` | Script structure tree view, `ProjectTree`, find/rename helpers |
| `project_wizard.rs` | New project dialog wizard |
| `script_variable_detector.rs` | Detect variable names from Rhai script text |
| `spine_editor.rs` | 2D editor for longitudinal spine curves, `SpineEditorState` |
| `spline_editor.rs` | 2D editor for cross-section spline profiles, `SplineEditorState` |
| `syntax.rs` | Syntax highlighting for the code editor |
| `templates.rs` | Template/snippet insertion helpers |
| `version_control_panel.rs` | Version snapshot/diff/restore UI panel |

---

### 5.6 `app` — Application State

**Path**: `src/app.rs`

`App` is the main egui application struct. It holds:

**State fields** (some participate in undo/redo via `AppState`):
- `state: AppState` — user-editable state (script text, spline profiles, spine, dimensions, etc.)
- `undo_history: UndoHistory` — undo/redo command history

**Geometry state**:
- `current_sdf: Option<Arc<dyn Sdf>>` — last evaluated SDF tree
- `current_mesh: Option<Mesh>` — last extracted mesh
- `current_sdf_grid: Option<Arc<SdfGrid>>` — SDF grid for raymarching

**Rendering**:
- `camera: Camera` — orbit camera
- `resolution: u32` — marching cubes resolution
- `smooth_normals: bool` — smooth vs flat shading
- `mesh_quality: MeshQuality` — adaptive MC quality level
- `show_wireframe: bool` — wireframe overlay toggle

**UI modes**:
- `editor_mode: EditorMode` — Script / NodeGraph / Notebook
- `node_graph: NodeGraph`, `graph_ui_state: GraphUiState`
- `notebook: Notebook`

**FEA overlay**:
- `FEAOverlayMode` enum: `None | Stress | Displacement`

**Split for printing**:
- `SplitAxisUi` enum: `Z | X | Y | Arbitrary`
- `SplitAlignUi` enum: `None | Pins | Groove | Dovetail | BoltHoles`

**Key enums** (defined in app.rs):
```rust
pub enum FEAOverlayMode { None, Stress, Displacement }
pub enum SplitAxisUi { Z, X, Y, Arbitrary }
pub enum SplitAlignUi { None, Pins, Groove, Dovetail, BoltHoles }
```

---

### 5.7 `aero` — Aerodynamic Analysis

**Path**: `src/aero/`

Provides aerodynamic analysis capabilities accessible from Rhai scripts.

#### Key Types

**`FlightCondition`** (`flight_condition.rs`): altitude, airspeed, air density, dynamic pressure.

**`AirfoilPolar`** (`polars.rs`): CL/CD polar data for an airfoil at given conditions. `PolarDatabase` is a collection of polars keyed by airfoil designation.

**`LiftingLineResult`** (`lifting_line.rs`): wing lift distribution from lifting line theory (span efficiency, induced drag, CL distribution along span).

**`StaticMarginResult`** (`stability.rs`): static margin, neutral point location, whether the aircraft is stable.

**`TrimResult`** (`stability.rs`): trimmed CL, elevator deflection, AOA for level flight.

**`DragPolarResult`** (`drag.rs`): CD0 breakdown (skin friction, form drag, interference), induced drag factor, total polar. Includes `s_ref_m2`, `cl_max`, `v_stall_ms`.

**`MotorSpec`** / **`PropSpec`** / **`PropulsionSetup`** (`propulsion_db.rs`, `propulsion.rs`): motor and propeller specifications and combined propulsion system.

**`PropulsionResult`** (`propulsion.rs`): thrust, power draw, efficiency, RPM at a given airspeed.

**`ClimbResult`** / **`RangeEnduranceResult`** / **`GlideResult`** (`performance.rs`): flight performance metrics.

---

### 5.8 `fea` — Finite Element Analysis

**Path**: `src/fea/`

Provides a pipeline from SDF geometry to CalculiX FEA and back.

#### Pipeline

1. `FEASetup` (declared in script via `fea_*` functions) — boundary conditions, material, regions
2. `meshing.rs` — voxel mesh of SDF → tetrahedral mesh (`TetMesh`)
3. `inp.rs` — writes CalculiX `.inp` input file
4. `calculix.rs` — runs CalculiX as a subprocess
5. `frd.rs` — reads CalculiX `.frd` result file
6. `pipeline.rs` — orchestrates all steps, produces `FEAGridResult` with `GridField` (stress, displacement fields on grid)
7. `viz.rs` — computes `FEAVizData` for rendering

#### Key Types

```rust
pub struct FEASetup {
    pub config:   FEAConfig,
    pub regions:  Vec<FEARegion>,
    // boundary conditions, forces, torques, motor loads...
}

pub enum FEARegion {
    FixedAxis(FEAAxisRegion),
    Force(FEAForceRegion),
    Pressure(FEAPressureRegion),
    Torque(FEATorqueRegion),
    Motor(FEAMotorRegion),
}

pub struct FEAConfig {
    pub material: MaterialPreset,
    pub resolution: u32,
    // ...
}

pub enum MaterialPreset {
    Aluminum6061, Steel4130, CarbonFiberUD,
    FiberglassWoven, PLA, PETG, Nylon,
    // ...
}
```

`FEAPipeline` sends `FEAMessage` variants over a channel to update the UI during the async FEA solve.

---

### 5.9 `analysis` — Geometry Analysis

**Path**: `src/analysis/`

#### `thickness.rs`

`compute_thickness(sdf, config) -> ThicknessResult` — computes wall thickness at sample points by ray marching inward from the surface and finding the second surface crossing.

#### `measurements.rs`

- `compute_model_properties(mesh) -> MeasurementResults` — volume, surface area, bounding box
- `measure_cross_section(sdf, plane) -> CrossSectionMeasurement` — cross-section area and perimeter at a plane
- `measure_distance(sdf, p1, p2) -> PointDistanceMeasurement` — distance between two surface points
- `ray_march_grid(...)` — utility for grid-based SDF evaluation
- `snap_to_surface(sdf, point) -> Vec3` — snap a point to the nearest surface

#### `print_analysis.rs`

Print orientation analysis: overhang angles, unsupported regions, wall thickness warnings. Configurable with `PrintAnalysisSettings` (build direction, printer preset, thresholds).

#### `hole_detection.rs`

`detect_mounting_holes(sdf, config) -> Vec<DetectedHole>` — automatically detects cylindrical holes suitable for fasteners by analyzing the SDF field.

#### `aero/` (sub-module)

Aerodynamic analysis derived from geometry (inlet performance, boundary layer estimates, etc.).

---

### 5.10 `materials` — Composite Materials

**Path**: `src/materials/`

`CompositeMaterial` — defines composite material properties (fiber orientation, ply thickness, stiffness matrix). Used by `CompositeLayup` and FEA.

---

### 5.11 `components` — Component Registry

**Path**: `src/components/`

`ComponentRegistry` — registry of named parametric components. `ComponentInstance` — a placed instance with parameter values. `library.rs` — built-in component definitions.

Used for the "blocks" palette in the UI — predefined geometry snippets that can be inserted and parameterized.

---

### 5.12 `node_graph` — Node Graph Editor

**Path**: `src/node_graph/`

Visual node graph editor as an alternative to text scripting:
- `NodeGraph` — the graph data structure (nodes, edges)
- `codegen.rs` — converts the node graph to equivalent Rhai script
- `ui.rs` — egui-based visual node editor

Editor modes are selected via `EditorMode` enum in `app.rs`.

---

### 5.13 `notebook` — Notebook Mode

**Path**: `src/notebook/`

Jupyter-style notebook mode where the script is divided into named cells that can be run individually:
- `Notebook` — contains a list of cells
- `types.rs` — cell types and state
- `codegen.rs` — converts notebook to a single evaluable script
- `ui.rs` — egui-based notebook editor

---

### 5.14 `library` — Component Library

**Path**: `src/library/`

User library of saved components:
- `LibraryManager` — loads/saves user component library from disk
- `metadata.rs` — library item metadata (name, category, description, tags)
- `thumbnail.rs` — generates preview thumbnails for library items

---

### 5.15 `version_control` — Version Control

**Path**: `src/version_control/`

Lightweight project versioning:
- `operations.rs` — snapshot, diff, restore operations on project files

Used by the version control UI panel in the app.

---

### 5.16 `geometry_analysis` — CG and Interference

**Path**: `src/geometry_analysis/`

#### `cg_sensitivity.rs`

```rust
pub struct CgSensitivityResult {
    pub envelope: CgEnvelope,
    pub component_sensitivities: Vec<ComponentCgSensitivity>,
    pub dimension_sensitivities: Vec<DimensionCgSensitivity>,
}

pub fn compute_cg_sensitivity(...) -> CgSensitivityResult;
```

Computes how the CG moves in response to dimension changes or component re-positioning.

#### `interference.rs`

```rust
pub struct InterferenceResult {
    pub pairs: Vec<InterferencePair>,
}

pub fn check_assembly_interference(components: &[ComponentHandle]) -> InterferenceResult;
```

Detects overlapping components using SDF evaluation. `InterferenceSeverity`: Hard (significant overlap), Soft (marginal/tolerance), Clear.

---

### 5.17 `project` — Project File Management

**Path**: `src/project.rs`

#### `Project` struct

The serializable project file format. Serialized to JSON by `serde_json`.

```rust
pub struct Project {
    pub version: String,
    pub script: String,
    pub resolution: u32,
    pub smooth_normals: bool,
    pub show_wireframe: bool,
    pub camera_position: [f32; 3],
    pub camera_target: [f32; 3],
    pub timestamp: Option<String>,
    pub node_graph: Option<NodeGraph>,
    pub notebook: Option<Notebook>,
    pub profiles: Option<HashMap<String, SplineEditorState>>,
    pub splines: Option<LongitudinalSplines>,
    pub section_view: Option<SectionView>,
    pub fea_config: Option<FEAConfig>,
    pub dimensions: IndexMap<String, f64>,
    pub print_analysis_settings: Option<PrintAnalysisSettings>,
    // ...
}
```

#### `SectionView` and `SectionPlane`

Section view clipping planes:
```rust
pub struct SectionPlane {
    pub axis: Axis,       // X | Y | Z
    pub position: f32,
    pub enabled: bool,
    pub flip: bool,       // which side to keep
}
```

---

### 5.18 `settings` — Application Settings

**Path**: `src/settings.rs`

`AppSettings` — persistent application preferences (UI layout, default resolution, color theme, CalculiX path, etc.). Serialized as TOML.

---

### 5.19 `export` — Export System

**Path**: `src/export/`

Export functions for geometry and manufacturing packages:
- STL export
- `ManufacturingPackage` — bundles geometry, material spec, FEA results, and print analysis for handoff

---

### 5.20 `undo` — Undo/Redo

**Path**: `src/undo.rs`

Command-pattern undo/redo:

```rust
pub struct AppState {
    pub script: String,
    // spline profiles, spine, dimensions, ...
}

pub struct UndoHistory { ... }
```

Command types:
- `ScriptTextCommand` — records script text changes
- `SplineShapeResetCommand` — records spline profile changes
- `LongitudinalSplineEditCommand` — records spine curve edits
- `RenameCommand` — records symbol rename operations

---

### 5.21 `headless` — Headless Mode

**Path**: `src/headless.rs`

Batch evaluation without a UI window. Used for scripted geometry generation, CI pipelines, or automated STL export. Selected via CLI flags (parsed by `clap` in `main.rs`).

---

## 6. Scripting API Reference

All functions below are registered in `src/scripting/api.rs` and available in Rhai scripts. All geometry functions return `SdfHandle`. Functions that perform analysis return Rhai maps or typed handles.

**Important script convention**: the last expression in a script (without a trailing semicolon) must be an `SdfHandle`. This is what gets meshed and displayed. If the last line ends with `;`, evaluation will fail with a type error.

---

### 6.1 Primitive Shapes

```rhai
sphere(radius)                         // SdfHandle — sphere centered at origin
box_(width, height, depth)             // SdfHandle — box centered at origin (full dimensions)
cylinder(radius, height)               // SdfHandle — cylinder along Z, centered at origin
torus(major_radius, minor_radius)      // SdfHandle — torus in XY plane
cone(radius, height)                   // SdfHandle — cone, tip at origin, base below
plane(nx, ny, nz, distance)            // SdfHandle — infinite half-space
```

**Note**: `box_` uses an underscore because `box` is a Rust keyword. In scripts, always write `box_(...)`.

---

### 6.2 Boolean Operations

```rhai
union(a, b)                            // SdfHandle — min(a, b)
subtract(a, b)                         // SdfHandle — removes b from a
intersect(a, b)                        // SdfHandle — keeps only overlap
smooth_union(a, b, k)                  // SdfHandle — smooth union (k = blend radius)
smooth_subtract(base, tool, k)         // SdfHandle — smooth removal with chamfer
smooth_intersect(a, b, k)              // SdfHandle — smooth intersection
```

---

### 6.3 Transforms

```rhai
translate(body, x, y, z)              // SdfHandle
rotate(body, rx, ry, rz)              // SdfHandle — Euler XYZ, degrees
scale(body, sx, sy, sz)               // SdfHandle — non-uniform scale
offset(body, distance)                 // SdfHandle — expand (+) or shrink (-)
shell(body, thickness)                 // SdfHandle — hollow shell
twist(body, ax, ay, az, rate)          // SdfHandle — twist around axis (degrees/unit)
bend(body, ax, ay, az, curvature)      // SdfHandle — bend along axis (radians/unit)
```

---

### 6.4 Patterns

```rhai
linear_array(body, x, y, z, count)    // SdfHandle — count copies spaced by (x,y,z)
polar_array(body, ax, ay, az, count)   // SdfHandle — count copies around axis
mirror(body, nx, ny, nz)              // SdfHandle — mirror across plane with given normal
```

---

### 6.5 Aerospace Shapes

#### Airfoil and Wing

```rhai
airfoil(naca_digits, chord)            // SdfHandle — NACA 4-digit airfoil (e.g. "2412")
airfoil_section(naca_digits)           // SectionHandle — 2D airfoil cross-section
extruded_airfoil(naca, chord, span)    // SdfHandle — airfoil extruded to span
wing(naca, root_chord, tip_chord, span, sweep_deg)  // SdfHandle — tapered swept wing
wing_from_stations(stations, span)     // SdfHandle — wing from list of (x, section) stations
```

#### Fuselage

```rhai
circle_section(radius)                 // SectionHandle — circular cross-section
ellipse_section(rx, ry)                // SectionHandle — elliptical cross-section
fuselage_station(position, section)    // StationHandle — positioned fuselage station
fuselage(stations, length)             // SdfHandle — lofted fuselage from stations
fuselage_parametric(length, width, height, nose_factor, tail_factor)  // SdfHandle
```

#### Nacelles and Inlets

```rhai
nacelle(radius, length)                // SdfHandle — simple nacelle
naca_inlet(width, height, depth, lip_radius)   // SdfHandle — NACA flush inlet
edf_duct(fan_radius, length, lip_t)    // SdfHandle — EDF duct
inlet_lip(radius, length, lip_t)       // SdfHandle — inlet with rounded lip
```

#### Nose and Tail Shapes

```rhai
haack_nose(radius, length, c)          // SdfHandle — Haack Series nose (c=0: LV-Haack, c=1/3: LD-Haack)
haack_tail(radius, length, c)          // SdfHandle — Haack Series tail
tangent_ogive(radius, length)          // SdfHandle — tangent ogive nose
ellipsoid_nose(radius, length)         // SdfHandle — ellipsoidal nose
```

---

### 6.6 Structural and Mechanical

#### Wing Structural

```rhai
rib_slab(root_pos, tip_pos, thickness, chord_range)   // SdfHandle — wing rib
spar_cylinder(root_pos, tip_pos, radius)               // SdfHandle — tubular spar
```

#### Drone Structural

```rhai
bulkhead_at_station(fuselage_sdf, x_pos, thickness)   // SdfHandle — circular bulkhead
lightening_hole_pattern(bulkhead, center, radius, count, hole_r)  // SdfHandle
motor_arm(root, tip, diameter)         // SdfHandle — motor arm tube
motor_mount(position, diameter, length)  // SdfHandle — motor mount tube
rod_mount(center, direction, rod_r, length, clamp_r)  // SdfHandle — rod/tube clamp mount
generate_mounts_sdf(components)        // SdfHandle — union of all component keepouts
bulkhead_with_keepouts(fuselage, x_pos, thickness, comps)  // SdfHandle
cable_hole_at(bulkhead, position, radius)  // SdfHandle
```

#### Mechanical Features

```rhai
bolt_circle(center_x, center_y, center_z, radius, count, hole_r, depth)  // SdfHandle
bolt_square(cx, cy, cz, side, hole_r, depth)           // SdfHandle — 4 holes at corners
bolt_rect(cx, cy, cz, w, h, hole_r, depth)             // SdfHandle — 4 holes at corners
countersink(x, y, z, shaft_r, head_r, depth)           // SdfHandle
counterbore(x, y, z, shaft_r, bore_r, bore_depth, total_depth)  // SdfHandle
slot(cx, cy, cz, nx, ny, nz, length, width, depth)     // SdfHandle — elongated slot
chamfer_edge(edge_sdf, radius)                          // SdfHandle — chamfer an edge
thread_hole(x, y, z, nominal_r, depth, pitch)          // SdfHandle — threaded hole
fc_mount(cx, cy, cz, hole_spacing, hole_r, standoff_h, standoff_r)  // SdfHandle — flight controller mount
motor_mount_pattern(cx, cy, cz, pattern, hole_r, depth)  // SdfHandle
```

---

### 6.7 Composite Layup

```rhai
composite_material(name, density_kg_m3, e1_gpa, e2_gpa, g12_gpa, nu12, t_ply_mm)  // MaterialHandle
composite_layer(material, thickness_mm, angle_deg)     // LayerHandle
composite_layup_config(layers)                         // LayupConfigHandle — list of layers
wing_composite(wing_sdf, layup)                        // SdfHandle — composite wing shell
fuselage_composite(fuse_sdf, layup)                    // SdfHandle — composite fuselage shell
printed_shell(body_sdf, wall_thickness_mm, layup)      // SdfHandle — printed + composite shell
apply_layup(body, layup_config)                        // SdfHandle — apply layup to any body
```

---

### 6.8 Control Surfaces

```rhai
hinge(hinge_type, x, y, z, nx, ny, nz)   // HingeHandle — hinge specification
linkage(horn_side, horn_offset, slot_w, slot_l)  // LinkageHandle
aileron(wing_sdf, span_start, span_end, chord_fraction, hinge, linkage)  // SdfHandle + separates from wing
elevator(htail_sdf, span_fraction, chord_fraction, hinge, linkage)       // SdfHandle
rudder(vtail_sdf, span_fraction, chord_fraction, hinge, linkage)         // SdfHandle
flap(wing_sdf, span_start, span_end, chord_fraction, hinge, linkage)     // SdfHandle
elevon(wing_sdf, span_start, span_end, chord_fraction, hinge, linkage)   // SdfHandle
wing_with_ailerons(wing_sdf, aileron_span_frac, chord_frac, hinge, linkage)  // SdfHandle
```

---

### 6.9 Print / Manufacturing

#### Body Splitting

```rhai
split_body(body, axis, position)       // SdfHandle — keeps the portion on one side of split plane
split_body_multi(body, planes)         // SdfHandle — multiple plane splitting
```

#### Alignment Features

```rhai
alignment_pin(x, y, z, radius, height)   // SdfHandle — cylindrical pin
alignment_socket(x, y, z, radius, depth, clearance)  // SdfHandle — socket for pin
```

#### Bracket

```rhai
l_bracket(width, height, thickness, leg_h)   // SdfHandle — L-bracket
u_bracket(width, height, depth, thickness)   // SdfHandle — U-bracket
```

---

### 6.10 Fasteners and Panels

```rhai
// Fastener functions
clearance_hole(spec_name, x, y, z, depth)    // SdfHandle — clearance hole for screw
countersink_hole(spec_name, x, y, z, depth)  // SdfHandle — countersink hole
heat_set_boss(spec_name, x, y, z)            // SdfHandle — boss for heat-set insert

// Tolerance
tolerance_settings(preset_name)               // ToleranceSettings handle
tolerance_compensated(body, settings)         // SdfHandle — tolerance-compensated body

// Panel functions
battery_hatch(cx, cy, cz, w, h, thickness, margin)  // SdfHandle — battery access hatch
fc_access_panel(cx, cy, cz, w, h, thickness)         // SdfHandle — FC access panel

// Joint
joint_delta(nominal, delta)                   // JointDelta handle
```

---

### 6.11 Field Operations

```rhai
constant_field(value)                         // FieldHandle
sdf_field(body)                               // FieldHandle — SDF distance as field
position_x_field()                            // FieldHandle — x coordinate as field
position_y_field()                            // FieldHandle
position_z_field()                            // FieldHandle

field_add(a, b)                               // FieldHandle
field_multiply(a, b)                          // FieldHandle
field_min(a, b)                               // FieldHandle
field_max(a, b)                               // FieldHandle
field_abs(f)                                  // FieldHandle

gradient_field(nx, ny, nz, scale)             // FieldHandle — linear gradient
radial_field(ax, ay, az, scale)               // FieldHandle — radial from axis
axial_radial_field(ax, ay, az, axial_s, radial_s)  // FieldHandle

offset_by_field(body, field)                  // SdfHandle — variable offset driven by field
shell_with_field(body, field)                 // SdfHandle — variable-thickness shell
blend_by_field(a, b, field)                   // SdfHandle — field-driven blend between two bodies
```

---

### 6.12 Lattice Structures

```rhai
gyroid_lattice(cell_size, thickness)          // SdfHandle — gyroid TPMS lattice
cubic_lattice(cell_size, strut_r)             // SdfHandle — cubic strut lattice
diamond_lattice(cell_size, strut_r)           // SdfHandle — diamond lattice

gyroid_with_field(cell_size_field, thickness) // SdfHandle — gyroid with variable cell size

conformal_gyroid(body, cell_size, thickness)  // SdfHandle — gyroid conformally mapped to body
conformal_diamond(body, cell_size, thickness) // SdfHandle
conformal_schwarz_p(body, cell_size, thickness)  // SdfHandle
```

---

### 6.13 Sweep Operations

```rhai
line_path(x1, y1, z1, x2, y2, z2)    // PathHandle — linear sweep path
arc_path(cx, cy, cz, radius, start_deg, end_deg)  // PathHandle — arc path
sweep(profile, path)                   // SdfHandle — sweep a profile along a path
```

---

### 6.14 Component Placement

```rhai
component(body, margin)                // ComponentHandle — geometry + keepout zone
component_named(body, margin, name, mass_g)  // ComponentHandle with name and mass
place(comp, x, y, z)                   // ComponentHandle — move component to position
geometry(comp)                         // SdfHandle — extract geometry from component
keepout(comp)                          // SdfHandle — extract keepout zone
auto_fuselage(components, clearance)   // SdfHandle — fuselage automatically sized around components
instance(body)                         // SdfHandle — create a shared instance (shared Arc)
```

---

### 6.15 Aerodynamic Analysis

```rhai
flight_condition(altitude_m, airspeed_ms)     // FlightConditionHandle
naca_polar(naca_digits, re)                   // PolarHandle — airfoil polar data

lifting_line(polar, fc, half_span, root_chord, tip_chord, sweep_deg, dihedral_deg)  // Map
// Returns: {cl, cd, span_efficiency, lift_n, drag_n, ...}

static_margin(wing_polar, htail_polar, fc, wing_geom, tail_geom)  // StabilityResultHandle
// Returns handle; use .static_margin_pct, .is_stable, etc.

trim_analysis(stability_result, fc, weight_n)  // TrimResultHandle

drag_polar(wing_polar, fc, wing_sdf, fuse_sdf, htail_sdf, vtail_sdf)  // DragPolarHandle

tail_volumes(wing_area, mac, span, htail_area, htail_arm, vtail_area, vtail_arm)  // Map
// Returns: {h_tail_volume, v_tail_volume, recommendation}

stability_result_to_map(result)               // Map from StabilityResultHandle
trim_result_to_map(result)                    // Map from TrimResultHandle
drag_polar_to_map(result)                     // Map from DragPolarHandle
```

---

### 6.16 Propulsion Analysis

```rhai
motor(name)                           // MotorHandle — look up by name (e.g. "T-Motor F80")
motor_custom(kv, max_current_a, weight_g, diameter_mm)  // MotorHandle
list_motors()                         // Array of motor name strings

prop(name)                            // PropHandle — look up by name
prop_by_size(diameter_in, pitch_in)   // PropHandle
prop_custom(diameter_in, pitch_in, ct0, ct_slope, cp0, cp_slope)  // PropHandle
list_props()                          // Array of prop name strings

propulsion_setup(motor, prop, cells, cap_mah)  // PropulsionHandle
propulsion_setup_full(motor, prop, cells, cap_mah, max_continuous_a, kv_override)  // PropulsionHandle

propulsion_analysis(setup, fc, weight_n)  // Map
// Returns: {thrust_n, power_w, current_a, efficiency, rpm, ...}

propulsion_thrust_at(setup, airspeed_ms, fc)  // f64 — thrust at given airspeed

range_endurance(setup, wing_sdf, fuse_sdf, htail_sdf, vtail_sdf, fc, weight_n)  // Map
rate_of_climb(setup, wing_sdf, fuse_sdf, htail_sdf, vtail_sdf, fc, weight_n)   // Map
glide_performance(wing_sdf, fuse_sdf, htail_sdf, vtail_sdf, fc, weight_n)      // Map

recommend_motor_prop(thrust_n, cruise_ms, max_weight_g)  // Array of Maps
```

Propulsion was added in the session ending 2026-03-17. The motor database has 8 motors and 10 props. `propulsion_analysis` runs an iterative RPM convergence algorithm.

---

### 6.17 Mass and CG Tracking

```rhai
mass_at(mass_g, x, y, z)             // declares a point mass (no geometry returned)
mass_named(name, mass_g, x, y, z)    // named point mass
ref_point(name, x, y, z)             // declares a named reference point (shown in viewer)
```

Mass points are accumulated during script evaluation and accessible via `ScriptResult::mass_points` and `ScriptResult::center_of_gravity()`.

---

### 6.18 FEA Setup

```rhai
fea_material(preset_name)             // sets material: "Aluminum6061", "Steel4130", etc.
fea_resolution(n)                     // sets voxel mesh resolution

fea_fix_x(x_min, x_max, y_min, y_max, z_min, z_max)   // fixed boundary in X direction
fea_fix_y(...)                        // fixed boundary in Y
fea_fix_z(...)                        // fixed boundary in Z
fea_fix_all(...)                      // fully fixed boundary

fea_force(x, y, z, fx, fy, fz)       // point force at position
fea_pressure(x_min, x_max, ..., pressure_pa)  // distributed pressure
fea_torque(cx, cy, cz, tx, ty, tz)   // torque load
fea_motor_load(position, thrust_n, torque_nm, axis_x, axis_y, axis_z)  // motor load
```

FEA setup declarations accumulate in `FEASetup` during script evaluation. The FEA pipeline is triggered separately from the UI.

---

### 6.19 Reference Points and Queries

```rhai
ref_point(name, x, y, z)             // declares a named reference point
point(x, y, z)                        // PointHandle — a 3D point
snap_to_surface_fn(body, point)       // PointHandle — nearest surface point
```

Reference points are rendered as colored spheres in the viewport with name labels.

---

### 6.20 Mesh Import

```rhai
import_mesh(path)                     // SdfHandle — import STL or OBJ as closest-point SDF
```

Path can be relative to the project directory. Mesh files are cached; re-parsed only when the file's mtime changes.

**Important**: Mesh-as-SDF is a closest-point SDF wrapper, not a true mathematical SDF. It is expensive to evaluate. For marching cubes, prefer using it at coarser resolutions or for small meshes. For large or complex imported meshes, the distance query is O(triangles).

---

### 6.21 Dimensions (Named Constants)

Dimensions are defined in the project's Dimensions panel (not in the script itself). They are injected as Rhai constants into the script scope before evaluation:

```rhai
// If "wingspan" = 800.0 is defined in the Dimensions panel:
let wing = wing("2412", root_chord, root_chord * 0.6, wingspan / 2.0, 15.0);
```

Named dimensions appear as `const` bindings — they cannot be reassigned in the script.

---

### 6.22 Script Cell System

Divide a script into sections using delimiter comments:

```rhai
# === Fuselage ===
let fuse = fuselage_parametric(400.0, 80.0, 60.0, 0.3, 0.4);

# === Wing ===
let wing = wing("2412", 120.0, 80.0, 300.0, 5.0);

# === Final Assembly ===
union(fuse, wing)
```

In notebook mode, each cell can be run independently. The cell delimiter syntax is: `# ===` at the start, `===` at the end, with the cell name in between.

---

## 7. Data Flow and Evaluation Pipeline

### Normal Script Evaluation

```
User edits script
       │
       ▼
evaluate_script_full()
  - Creates fresh rhai::Engine
  - Registers all SDF functions as closures
  - Injects dimension constants into scope
  - Registers library modules
  - Calls engine.eval_with_scope::<SdfHandle>()
       │
       ▼
ScriptResult {
  sdf,              ← root SdfHandle (Arc<dyn Sdf>)
  mass_points,      ← accumulated via mass_at()
  fea_setup,        ← accumulated via fea_*() calls
  layups,           ← accumulated via composite_layup_config()
  reference_points  ← accumulated via ref_point()
}
       │
       ▼
adaptive_mc::extract_mesh(sdf, bounding_box, quality)
  - Parallelized via rayon
  - Returns Mesh { vertices, indices }
       │
       ▼
Upload to GPU (wgpu vertex/index buffers)
       │
       ▼
Render loop draws mesh each frame
```

### FEA Pipeline

```
Script declares fea_*() boundary conditions
       │
       ▼
ScriptResult.fea_setup populated
       │
FEA button clicked in UI
       ▼
FEAPipeline::run(sdf, fea_setup)
  - meshing: SDF → voxel → TetMesh
  - inp: TetMesh + BCs → .inp file
  - calculix: runs CalculiX subprocess
  - frd: reads .frd result file
  - Returns FEAGridResult with stress/displacement GridField
       │
       ▼
evaluate_script_full(..., stress_field, displacement_field, ...)
  - FEA fields available as stress_field() / displacement_field() in scripts
  - Can be used in OffsetByField / ShellWithField for topology optimization feedback
```

### Mesh Import Cache

```
import_mesh("path/to/model.stl")
       │
       ▼
Check MeshCache (HashMap<PathBuf, (SystemTime, Arc<TriangleMesh>)>)
  - If cached and mtime matches: use cached TriangleMesh
  - If not cached or mtime changed: parse_stl() / parse_obj(), insert into cache
       │
       ▼
Create closest-point SDF wrapper around TriangleMesh
Return SdfHandle
```

---

## 8. Rhai Type Handles

All Rust types exposed to Rhai scripts are wrapped in newtype handles. These are `Clone` (backed by `Arc` or by copying small structs).

| Handle Type | Wraps | Purpose |
|---|---|---|
| `SdfHandle` | `Arc<dyn Sdf>` | SDF geometry object |
| `FieldHandle` | `Arc<dyn Field>` | Scalar field |
| `SectionHandle` | `Arc<dyn Section2D>` | 2D cross-section shape |
| `StationHandle` | `{ position: f32, section: Arc<dyn Section2D> }` | Fuselage station |
| `ComponentHandle` | `{ geometry, keepout, mass_g, name }` | Placed component with clearance |
| `PathHandle` | `Arc<dyn SweepPath>` | Sweep path |
| `ProfileHandle` | `Arc<dyn Section2D>` | 2D sweep profile |
| `MaterialHandle` | `Arc<CompositeMaterial>` | Composite material spec |
| `LayerHandle` | `Arc<ShellLayer>` | Composite shell layer |
| `LayupConfigHandle` | `Vec<Arc<ShellLayer>>` | Composite layup stack |
| `HingeHandle` | `HingeSpec` | Control surface hinge |
| `LinkageHandle` | `LinkageSpec` | Control surface linkage |
| `PointHandle` | `Vec3` | 3D point |
| `MountingHoleHandle` | `MountingHole` | Single mounting hole |
| `MountingHoleSetHandle` | `MountingHoleSet` | Set of mounting holes |
| `PolarHandle` | `Arc<AirfoilPolar>` | Airfoil polar data |
| `FlightConditionHandle` | `FlightCondition` | Flight condition |
| `StabilityResultHandle` | `StaticMarginResult` | Stability analysis result |
| `TrimResultHandle` | `TrimResult` | Trim analysis result |
| `DragPolarHandle` | `DragPolarResult` | Drag polar result |
| `MotorHandle` | `Arc<MotorSpec>` | Motor specification |
| `PropHandle` | `Arc<PropSpec>` | Propeller specification |
| `PropulsionHandle` | `Arc<PropulsionSetup>` | Motor+prop+battery system |

**Important implementation detail**: handles are `Clone` because Rhai requires this for all registered types. The `Arc` wrappers make cloning cheap (reference count increment, no deep copy). The `Sdf` and `Field` traits require `Send + Sync` for rayon parallelism.

---

## 9. Project File Format

Project files are JSON (`.implicitcad` or `.json` extension). The root is a `Project` struct:

```json
{
  "version": "0.1.0",
  "script": "let s = sphere(10.0);\ns",
  "resolution": 64,
  "smooth_normals": false,
  "show_wireframe": false,
  "camera_position": [0.0, -200.0, 100.0],
  "camera_target": [0.0, 0.0, 0.0],
  "timestamp": "2026-03-17T12:00:00Z",
  "dimensions": {
    "wingspan": 800.0,
    "root_chord": 120.0
  },
  "section_view": {
    "plane_a": { "axis": "X", "position": 0.0, "enabled": false, "flip": false },
    "plane_b": null
  },
  "fea_config": null,
  "node_graph": null,
  "notebook": null,
  "profiles": null,
  "splines": null,
  "print_analysis_settings": null
}
```

Optional fields (`Option<T>`) are omitted when `None`. The `dimensions` map (`IndexMap`) preserves insertion order.

**Serialization**: `serde_json::to_string_pretty` / `serde_json::from_str`. Project auto-saves are written to the same path periodically.

---

## 10. Rendering Architecture

The app uses eframe's wgpu integration. Custom rendering happens via `egui_wgpu::CallbackTrait`.

### Render Passes (per frame)

1. **Background**: egui draws the UI panels
2. **Viewport callback**: custom wgpu render pass, executed inside the egui viewport panel:
   a. Grid pass (`GridRenderer`)
   b. Axes pass (`AxesRenderer`)
   c. Mesh pass (`RenderState`) — draws the SDF-extracted mesh
   d. Wireframe pass (`WireframeRenderer`) — optional overlay
   e. Raymarch pass (`RaymarchRenderer`) — section view clipping or thickness heat map

### Shader (`shaders/mesh.wgsl`)

Vertex + fragment shader for mesh rendering:
- Input: `position: vec3<f32>`, `normal: vec3<f32>`
- Uniform: model-view-projection matrix, camera eye position, shading mode
- Lighting: single directional light + ambient term
- Output: per-vertex (smooth) or per-face (flat) shading in a neutral gray-blue color

### Camera (`render/camera.rs`)

Orbit camera parameterized by:
- `target: Vec3` — look-at point (orbit center)
- `distance: f32` — distance from target
- `yaw: f32`, `pitch: f32` — spherical angles

Controls:
- Left mouse drag → orbit (change yaw/pitch)
- Right mouse drag → pan (move target)
- Scroll wheel → zoom (change distance)

### Section View

Section view uses the `RaymarchRenderer` with `SectionUniforms` specifying up to two clipping planes. Each plane is defined by an axis (X/Y/Z) and a position. The `flip` field determines which side of the plane is kept.

### Thickness Visualization

The `RaymarchRenderer` with `ThicknessUniforms` computes per-pixel wall thickness via dual ray marching (outward and inward from each surface point) and colors the result as a heat map (blue = thin, red = thick).

---

## 11. FEA Pipeline Details

### Boundary Condition Types

Declared via Rhai `fea_*()` functions, stored in `FEASetup`:

| Rhai Function | Rust Type | Description |
|---|---|---|
| `fea_fix_x/y/z/all()` | `FEAAxisRegion` | Fixed displacement boundary |
| `fea_force()` | `FEAForceRegion` | Point or distributed force |
| `fea_pressure()` | `FEAPressureRegion` | Surface pressure load |
| `fea_torque()` | `FEATorqueRegion` | Torque load |
| `fea_motor_load()` | `FEAMotorRegion` | Combined thrust + torque from motor |

### Material Presets

```rust
pub enum MaterialPreset {
    Aluminum6061,
    Steel4130,
    CarbonFiberUD,
    FiberglassWoven,
    PLA,
    PETG,
    Nylon,
    // ...
}
```

Each preset defines `E` (Young's modulus), `nu` (Poisson's ratio), `rho` (density), `yield_strength`.

### FEA Messages

The `FEAPipeline` communicates progress via an `mpsc` channel of `FEAMessage`:
```rust
pub enum FEAMessage {
    Progress { stage: String, percent: f32 },
    Result(FEAGridResult),
    Error(String),
}
```

The UI polls this channel and updates progress indicators.

### Grid Fields

```rust
pub struct GridField {
    pub values: Vec<f32>,      // flat array of field values on regular grid
    pub grid_dims: [usize; 3], // grid dimensions (nx, ny, nz)
    pub bounds_min: Vec3,
    pub bounds_max: Vec3,
}
```

Grid fields can be passed back into script evaluation as `stress_field` / `displacement_field` for field-driven post-processing (e.g., remove material where stress is low).

---

## 12. Testing Conventions

Tests are co-located with source files in `#[cfg(test)]` modules and in `tests/` for integration tests.

### Unit Test Pattern

Each module with non-trivial logic has a `#[cfg(test)] mod tests` block. Key test coverage:

- `sdf/primitives.rs`: sphere, box, cylinder, torus, cone, plane — boundary, surface, exterior points
- `sdf/booleans.rs`: union, subtract, intersect, smooth_intersect — mathematical properties
- `sdf/transforms.rs`: translate, rotate, scale, twist (zero rate = identity, twist rotates), bend (zero = identity, curves shape)
- `sdf/mod.rs`: compound SDF integration test (box - sphere)
- `aero/propulsion.rs`: 5 tests — motor lookup, CT at zero/Jmax, static thrust, thrust vs airspeed, max airspeed
- `aero/performance.rs`: 3 tests — endurance at best speed, range at best range speed, glide ratio

### Running Tests

```
cargo test
```

Expected: all tests pass. As of 2026-03-17: 328 tests pass, 0 fail.

### Test File Conventions

- Unit tests live in `#[cfg(test)] mod tests` at the bottom of each source file
- Integration tests live in `tests/`
- Test scripts (Rhai) live in `test_scripts/`

### Adding New Tests

When adding new functionality, add the minimum tests that would have caught the bug or validated the feature. For SDF primitives: test inside, on surface, and outside. For analysis functions: test known-good values and boundary conditions.

---

## 13. Engineering Conventions

### Coordinate System

- **X**: forward (aircraft nose direction for aerospace geometry)
- **Y**: left/right (span direction for wings)
- **Z**: up (altitude direction)

All geometry is in millimeters by default (though no units are enforced — the SDF is dimensionless). If you work in meters, all dimensions and forces must be consistent.

### SDF Sign Convention

- **Negative distance**: inside the geometry (solid material)
- **Zero**: on the surface
- **Positive distance**: outside the geometry (air)

Marching cubes extracts the zero-isosurface. The mesh surface is where `distance = 0`.

### Arc<dyn Sdf> Pattern

All SDF nodes in the tree are `Arc<dyn Sdf>`. New SDF types must:
1. Implement `Sdf` trait (`fn distance(&self, point: Vec3) -> f32`)
2. Implement `Send + Sync` (usually automatic if all fields are `Send + Sync`)
3. Be constructable into `Arc<Self>` for composition

### Error Handling

- Script evaluation errors are returned as `Err(String)` and displayed in the UI error panel
- Panics are not acceptable in production paths; use `Result<>` or `Option<>`
- FEA errors are reported via `FEAMessage::Error(String)` channel

### Module Boundaries

- `sdf`: no dependencies on other project modules — pure geometry
- `mesh`: only depends on `sdf` (for the `Sdf` trait parameter)
- `aero`: no dependencies on other project modules — pure analysis
- `materials`: no dependencies on other project modules
- `scripting/api.rs`: the main integration point — imports from all other modules

### Numeric Types

- SDF distances: `f32`
- Script API: accepts `f64` from Rhai (Rhai's native float type) and casts to `f32` at the API boundary
- Geometry math: `glam::Vec3` (f32), `glam::Quat` (f32), `glam::Mat4` (f32)
- Analysis values (aerodynamic, mass): `f32` internally, `f64` in Rhai return maps

---

## 14. Common Patterns

### Adding a New SDF Primitive

1. Add struct + `impl Sdf` in `src/sdf/primitives.rs` (or a new file in `sdf/`)
2. Add `pub use` in the relevant `mod.rs`
3. Add `register_fn` in `src/scripting/api.rs` (in the appropriate `register_*` function)
4. Add unit tests in the same file
5. Update this reference document

**Template**:
```rust
pub struct MyShape {
    pub param: f32,
}

impl MyShape {
    pub fn new(param: f32) -> Self { Self { param } }
}

impl Sdf for MyShape {
    fn distance(&self, point: Vec3) -> f32 {
        // return signed distance
    }
}
```

Registration in api.rs:
```rust
engine.register_fn("my_shape", |param: f64| {
    SdfHandle(Arc::new(MyShape::new(param as f32)))
});
```

### Adding a New Rhai Function

1. Open `src/scripting/api.rs`
2. Find the appropriate `register_*` function or create a new one
3. Add `engine.register_fn("function_name", |args| { ... });`
4. If the function needs state (e.g., an accumulator), pass it as a captured `Arc<Mutex<T>>`
5. Add the function to the `register_sdf_functions` dispatcher if needed
6. Document in this reference under Section 6

### Adding a New Handle Type

1. Add the struct to `src/scripting/mod.rs`
2. Add `engine.register_type::<MyHandle>()` in `register_sdf_functions` or the appropriate `register_*` function
3. Register any methods on the handle if needed

### Adding Analysis Functions

Pattern from propulsion analysis (latest addition, 2026-03-17):
1. Implement analysis logic in the appropriate analysis module (`aero/`, `analysis/`, etc.)
2. Add pub use in the module's `mod.rs`
3. Add handle types in `scripting/mod.rs`
4. Add `register_*_functions(engine)` in `scripting/api.rs`
5. Call the new registration function from `register_sdf_functions()`
6. Add unit tests in the analysis module

---

## 15. Scope Boundaries and Non-Goals

From MasterPlan.md — what is explicitly in scope vs out of scope.

### Currently In Scope

- SDF-based geometry: primitives, booleans, transforms, patterns, aerospace shapes
- Rhai scripting API covering all geometry operations
- Marching cubes mesh extraction (uniform and adaptive)
- egui desktop application with code editor and 3D viewport
- Camera: orbit, pan, zoom
- Flat and smooth shading
- Section view (clip planes)
- Thickness visualization
- STL export
- Project file save/load (JSON)
- Named dimensions (parametric constants)
- Undo/redo
- Component placement system with keepout zones
- Mass and CG tracking
- Aerodynamic analysis (lifting line, static margin, drag polar, propulsion, performance)
- FEA via CalculiX
- Composite layup modeling
- Control surface generation
- Print/manufacturing features (split, alignment, tolerance compensation)
- Field-driven operations (variable offset, shell, blend)
- Lattice infill (gyroid, diamond, cubic, conformal variants)
- Node graph editor (code generation from visual nodes)
- Notebook mode (cell-based scripting)
- User component library
- Version control snapshots
- Geometry analysis (measurements, hole detection, CG sensitivity, interference)

### Out of Scope / Not Planned

- GPU-accelerated SDF evaluation (future phase)
- Multi-body assemblies with constraint solving
- B-Rep or NURBS geometry
- Cross-platform builds (macOS, Linux) — architecture supports it but not targeted
- Auto-completion beyond current implementation
- Real-time typing → mesh update (intentional design decision: requires explicit run)
- STEP import/export
- Full CFD simulation (lifting line + basic drag polar is the extent)
- DXF/drawing output
- Multi-user collaboration

---

## 16. Known Constraints and Gotchas

### Rhai `box` Keyword Conflict

`box` is a Rust keyword (used for `Box<T>`). The function must be named `box_` in both the Rhai API and in scripts. Do not attempt to register a function named `box` — it will not compile.

### Script Must End Without Semicolon

The last expression in a script must be an `SdfHandle` (not `SdfHandle;`). If the last line ends with a semicolon, Rhai returns `()` and the evaluator produces an error. The error formatter catches this and shows a friendly message.

### SDF Evaluation Cost

SDF evaluation is the hot path. Complex SDF trees (deep boolean hierarchies, mesh-as-SDF) can be slow when evaluated at high marching cubes resolutions. Profile before optimizing.

For `import_mesh()`, the closest-point SDF wrapper iterates all triangles per query point at O(n_triangles) per call. For large meshes, use coarser marching cubes resolution.

### Approximate SDFs

`Twist` and `Bend` produce approximate (non-Lipschitz-1) SDFs. They are correct at zero deformation but the distance property degrades at high twist rates or high curvature. Symptoms: marching cubes may produce artifacts at high deformation values. Do not use for precision offset operations.

### Rayon Thread Count

Rayon uses all available cores for parallel SDF evaluation. This is generally desirable but can interfere with UI responsiveness during mesh extraction. Consider wrapping expensive extractions in `std::thread::spawn` + channel if UI freezes are a problem.

### Rhai Engine Per Evaluation

A fresh `rhai::Engine` is constructed on every `evaluate_script_full()` call. This means:
- All function registration overhead is paid on every evaluation
- There is no persistent state between evaluations
- Library modules are re-compiled and registered on every evaluation

For very fast re-evaluation, consider caching the compiled library ASTs.

### MeshCache Keyed by mtime

The `MeshCache` invalidates on mtime change. If a mesh file is modified but has the same mtime (e.g., written by a tool that does not update mtime), the old cached version will be served. This is an edge case; normal workflow is unaffected.

### Coordinate Convention for Aerospace

Aerospace geometry is built in a right-handed coordinate system with X=forward, Y=right (or left depending on convention), Z=up. The `wing_with_airfoil` function extrudes the wing along +Y and -Y from the centerline. Fuselage is oriented along the X axis.

### NACA Airfoil Digit String

`airfoil("2412", chord)` takes a string of 4 digits, not a number. Always quote it: `"2412"`, not `2412`. `is_valid_naca_4digit(s)` validates the string before use.

### Propulsion Database

The motor database (`propulsion_db.rs`) has 8 motors and 10 propellers hardcoded. `list_motors()` and `list_props()` return the available names. For motors or props not in the database, use `motor_custom()` and `prop_custom()`.

### FEA Requires CalculiX Binary

The FEA pipeline (`fea/calculix.rs`) spawns CalculiX as an external subprocess. The path to the CalculiX binary must be set in `AppSettings`. If CalculiX is not installed or the path is wrong, the FEA run will fail with an error message in the UI.

### Section View Dual Planes

The `SectionView` supports up to two clipping planes (`plane_a` and `plane_b`). Both can be enabled simultaneously for a cross-section cut. Only axis-aligned planes (X, Y, Z) are supported; arbitrary plane clipping is a future feature.

---

*End of AGENT_REFERENCE.md*
*Generated: 2026-03-17*
*Source of truth: MasterPlan.md (project scope), src/ (implementation details)*
