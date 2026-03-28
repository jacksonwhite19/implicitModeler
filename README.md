# Implicit CAD

A code-first CAD modeler using signed distance fields (SDFs). Write scripts to construct 3D models through primitives, boolean operations, and transforms.

![Implicit CAD Screenshot](screenshot.png)

## Features

### Feature Status
- `Stable` features are the core geometry, transforms, export, project, and most fabrication tools.
- `Experimental` features are analysis-oriented helpers where outputs are useful but should still be validated independently for production decisions.
- `Legacy` features are compatibility shorthands kept for older example scripts; prefer the newer explicit APIs when available.

### Core Capabilities
- **Script-Based Modeling**: Define geometry using the Rhai scripting language
- **Real-Time Preview**: Instant 3D visualization with orbit/pan/zoom controls
- **SDF-Based Geometry**: Smooth, mathematically precise models
- **Mesh Export**: Export to STL, OBJ, and manufacturing package outputs
- **Project Management**: Save/load projects with full state preservation
- **Batch Processing**: Headless mode for automation and CI/CD integration
- **Undo/Redo**: Full edit history for productive workflows
- **Aircraft Workflow**: Template-driven fixed-wing project setup with manufacturing presets
- **Workflow Constraints and Variants**: Project-level assembly rules and saved design variants for dimension-driven iteration
- **Workflow Summaries**: Consolidated manufacturing and flight checks in the project workflow panel
- **Component Modules and Mounting**: Import reusable hardware components, place them in host geometry, and generate trays/brackets procedurally

### Geometry Primitives
- Sphere, Box, Cylinder
- Torus, Cone, Plane

### Boolean Operations
- Union, Subtract, Intersect
- Smooth Union (with adjustable blending)

### Transformations
- Translate, Rotate, Scale
- Offset (expand/contract)
- Shell (create hollow versions)

### Aerospace Features (Phase 6)
- **NACA Airfoils**: Parametric generation of 4-digit airfoils with chord scaling
- **Parametric Wings**: Full geometric control (taper, sweep, dihedral, twist)
- **Fuselage Generation**: Lofted sections with nose/tail shaping
- **Engine Nacelles**: Composite nacelle primitives
- **Automatic Blending**: Smooth component integration with filleting

### UI Features
- **Script Examples**: 40+ built-in example scripts with stable / experimental / legacy maturity labels
- **Quick Insert Menu**: 18 code snippets across 4 categories (primitives, operations, transforms, patterns)
- **Smooth Normals**: Toggle for better organic shapes
- **Wireframe Overlay**: Toggle mesh edge visualization
- **Resolution Control**: Adjust mesh quality (16-64 grid)
- **Keyboard Shortcuts**: F5/Ctrl+R to run, Ctrl+S to save, Ctrl+Z/Y for undo/redo
- **Progress Indicators**: Visual feedback during processing
- **Error Display**: Clear, styled error messages with smart syntax hints
- **Project Files**: Save/load .icad project files with Ctrl+S/O
- **Undo/Redo**: Full edit history with Ctrl+Z/Ctrl+Y
- **Auto-Save**: Automatic crash protection (saves every 30s to temp directory)
- **Line Counter**: Real-time script size display

## Quick Start

### Build and Run

```bash
# Build release version
cargo build --release

# Run the application
cargo run --release
```

### Basic Usage

1. **Write a Script** in the left panel:
```rhai
// Create a sphere
let s = sphere(10.0);
s
```

2. **Press F5** or click "Run" to visualize

3. **Explore Examples** from the dropdown menu

4. **Control the Camera**:
   - Left-drag to orbit
   - Right-drag to pan
   - Scroll to zoom
   - Home key to reset

5. **Export** your model using the STL, OBJ, or package export actions

6. **Save** your project with Ctrl+S for later editing

### Batch/Headless Mode

Process scripts without GUI for automation:

```bash
# Single file
implicit-cad --headless --script input.rhai --output result.stl

# Project file
implicit-cad --headless --script aircraft.icad --output result.stl

# Batch directory
implicit-cad --headless --batch ./scripts --batch-output ./models --format obj

# With custom quality and metrics
implicit-cad --headless --script input.rhai --output result.stl --mesh-quality fine --output-metrics metrics.json

# Manufacturing package from a project
implicit-cad --headless --script aircraft.icad --output ./package --format package
```

**CLI Options**:
- `--headless` - Run without GUI
- `--script <file>` - Input `.rhai` or `.icad` file
- `--output <file>` - Output mesh path
- `--batch <dir>` - Process all .rhai files in directory
- `--batch-output <dir>` - Output directory for batch mode
- `--format <stl|obj|package>` - Export format (default: stl)
- `--resolution <num>` - Mesh grid resolution (default: 32)
- `--mesh-quality <draft|normal|fine|ultra>` - Quality preset mapped to export resolution
- `--smooth-normals` - Enable smooth normal averaging
- `--dim <NAME=VALUE>` - Override named dimensions in headless mode
- `--output-metrics <file>` - Write metrics JSON for the evaluated model

## Scripting API

Status note: the scripting surface mixes stable APIs with some compatibility aliases. Prefer the primary geometry, print, and analysis functions shown in the help panel over older shorthand wrappers when both exist.

### Component Mounting Workflow

The current component/bracket workflow is built around reusable component maps and a granular tray-first bracket generator.

Component modules typically export:

- `physical`: the actual hardware body
- `keepout`: minimum no-intersection clearance
- `service_keepout`: optional removable/service no-go volume
- `tray_seed`: optional explicit face region used to build a connected tray shell
- `fastener_keepout`: optional screw/fastener no-go volume used for local pad-up reinforcement
- `mount_points`: bracket/support origins defined with `mount_point(...)`
- `boss`: optional oversized mounting/boss geometry

The main mounting entry point is:

```rhai
mount_component_granular(parent, component_map, x, y, z)
```

Useful helpers:

```rhai
bracket_offset(component_map, offset_mm)
tray_clearance(component_map, clearance_mm)
tray_thickness(component_map, thickness_mm)
support_density(component_map, level)   // 1..10
hide_part(value_map, "component_physical")
hide_parts(value_map, ["component_physical", "raw_bracket"])
```

The mounting pipeline is:

1. place the component map into the host geometry
2. build a connected tier-1 tray shell from `tray_seed` when available
3. branch supports from the tray/mount points into the host
4. cut back by keepout/service volumes
5. blend into the host and return stage outputs

The returned map currently includes:

- `tray`
- `fastener_pads`
- `raw_bracket`
- `cut_bracket`
- `blended_bracket`
- `bracket`
- `assembly`
- `component_physical`
- `debug_paths`
- `debug_summary`

Example:

```rhai
import "components/electronics_box" as ebox;

let fuselage_outer = fuselage_elliptical(220.0, 90.0, 70.0, 45.0, 50.0, 0.9, 0.8, 8.0);
let fuselage_shell = shell(fuselage_outer, 1.6);
let fuse_center = bbox_center(fuselage_outer);

let c = ebox::component();
let c = bracket_offset(c, 0.5);
let c = tray_clearance(c, 0.5);
let c = tray_thickness(c, 1.0);
let c = support_density(c, 6);

let placed = mount_component_granular(
    fuselage_shell,
    c,
    fuse_center.x, 0.0, fuse_center.z + 2.0
);

union(intersect(placed.assembly, fuselage_outer), placed.component_physical)
```

## Fixed-Wing Workflow

Use `File -> New Project...` and choose the fixed-wing template to start from the canonical aircraft workflow. That template now includes:

- top-level project dimensions for the airframe and internals
- project-level assembly constraints for keeping major internals aligned to reference stations
- manufacturing presets for foamboard, LW-PLA shell, carbon tube spar, balsa hybrid, and molded shell
- integrated internal installation helpers for battery, FC, servo, and antenna hardware
- service access geometry for battery and FC hatches
- saved design variants for switching between dimension sets
- workflow summaries for manufacturability and flight metrics
- shared GUI/headless export behavior

The in-app example browser now exposes three maturity levels that match the help panel:

- `Stable`: examples built on the canonical, supported API surface
- `Experimental`: examples that rely on analysis-oriented features whose behavior may still evolve
- `Legacy`: compatibility examples kept for older scripting patterns

Gold-standard complete-aircraft examples currently include:

- `Gold Standard: Conventional Trainer`
- `Gold Standard: Flying Wing UAV`
- `Gold Standard: Twin-Boom Pusher`

### Primitives

```rhai
sphere(radius)
box_(width, height, depth)
cylinder(radius, height)
torus(major_radius, minor_radius)
cone(radius, height)
plane(nx, ny, nz, distance)
```

### Boolean Operations

```rhai
union(a, b)
subtract(a, b)
intersect(a, b)
smooth_union(a, b, smoothness)
```

### Transformations

```rhai
translate(shape, x, y, z)
rotate(shape, rx, ry, rz)  // degrees
scale(shape, sx, sy, sz)
offset(shape, distance)
shell(shape, thickness)
```

### Aerospace Functions

```rhai
// NACA airfoils
naca(designation, chord)  // e.g., naca("2412", 10.0)

// Parametric wings
wing_with_airfoil(
    airfoil,      // NACA designation (e.g., "2412")
    root_chord,   // Chord at wing root
    tip_chord,    // Chord at wing tip
    span,         // Total span
    sweep,        // Leading edge sweep (degrees)
    dihedral,     // Dihedral angle (degrees)
    twist         // Tip twist (degrees, negative = washout)
)

// Parametric fuselage
fuselage_parametric(
    length,       // Total length
    diameter,     // Maximum diameter
    nose_shape,   // Nose shape (0=pointed, 1=rounded)
    tail_shape    // Tail shape (0=pointed, 1=rounded)
)

// Engine nacelles
nacelle(length, diameter, inlet_diameter, exhaust_diameter)

// Smooth blending (alias for smooth_union)
blend(shape_a, shape_b, radius)
```

## Example Scripts

### Smooth Blended Spheres
```rhai
let s1 = sphere(8.0);
let s2 = translate(sphere(8.0), 10.0, 0.0, 0.0);
smooth_union(s1, s2, 3.0)
```

### Hollow Torus
```rhai
let t = torus(15.0, 3.0);
shell(t, 1.0)
```

### Bracket with Holes
```rhai
let base = box_(40.0, 20.0, 10.0);
let hole = cylinder(4.0, 12.0);
let hole = translate(hole, 10.0, 5.0, 0.0);
let part = subtract(base, hole);
let part = offset(part, 2.0);
part
```

### Complete Aircraft Assembly
```rhai
// Fuselage
let fuselage = fuselage_parametric(60.0, 8.0, 0.7, 0.5);

// Main wing
let wing = wing_with_airfoil("2412", 12.0, 6.0, 40.0, 8.0, 2.0, -1.5);
let wing = translate(wing, 25.0, 0.0, 0.0);

// Horizontal tail
let h_tail = wing_with_airfoil("0012", 6.0, 3.0, 15.0, 5.0, 0.0, 0.0);
let h_tail = translate(h_tail, 52.0, 0.0, 0.0);

// Vertical tail
let v_tail = wing_with_airfoil("0009", 8.0, 4.0, 8.0, 15.0, 0.0, 0.0);
let v_tail = rotate(v_tail, 0.0, 0.0, 90.0);
let v_tail = translate(v_tail, 52.0, 0.0, 5.0);

// Blend all components
let aircraft = blend(fuselage, wing, 2.5);
let aircraft = blend(aircraft, h_tail, 1.5);
let aircraft = blend(aircraft, v_tail, 1.5);
aircraft
```

## Technical Details

- **Language**: Rust
- **Rendering**: wgpu (GPU-accelerated)
- **UI**: egui/eframe
- **Scripting**: Rhai
- **Mesh Generation**: Marching Cubes algorithm
- **Normal Computation**: SDF gradients with optional smoothing
- **Project Format**: JSON-based .icad files
- **CLI**: clap for command-line interface
- **File Dialogs**: rfd for native save/load dialogs

## Architecture

```
┌─────────────────────────────────────────┐
│         Application (eframe)             │
│  ┌──────────┐        ┌──────────────┐   │
│  │  Code    │        │  3D Viewport │   │
│  │  Editor  │───────▶│  (wgpu)      │   │
│  └──────────┘        └──────────────┘   │
│       │                      ▲           │
│       ▼                      │           │
│  ┌──────────┐        ┌──────────────┐   │
│  │  Rhai    │        │   Marching   │   │
│  │  Script  │───────▶│   Cubes      │   │
│  │  Engine  │        │   Meshing    │   │
│  └──────────┘        └──────────────┘   │
│       │                                  │
│       ▼                                  │
│  ┌──────────┐                            │
│  │ Geometry │                            │
│  │  Kernel  │                            │
│  │  (SDF)   │                            │
│  └──────────┘                            │
└─────────────────────────────────────────┘
```

## Testing

Run the test suite:
```bash
cargo test
```

All 19 unit tests should pass, covering:
- SDF primitives
- Boolean operations
- Transformations
- Mesh extraction
- Scripting engine

## Export Formats

### STL (Binary)
- Standard 3D printing format
- Computed face normals
- Compatible with slicers

### OBJ
- Includes vertex normals
- Human-readable format
- Compatible with 3D software

## Performance

- **Mesh Quality**: Adjustable resolution (16³ to 64³)
- **Typical Performance**: 32³ resolution in <50ms
- **Smooth Normals**: Minimal overhead (~10% slower)

## Future Enhancements

Planned features:
- Wireframe overlay mode
- Material presets
- File save/load
- Pattern operations (arrays)
- Mirror/symmetry operations
- Adaptive bounding boxes

## License

See LICENSE file for details.

## Contributing

See PROGRESS.md for development history and current status.
