# Implicit CAD

A code-first CAD modeler using signed distance fields (SDFs). Write scripts to construct 3D models through primitives, boolean operations, and transforms.

![Implicit CAD Screenshot](screenshot.png)

## Features

### Core Capabilities
- **Script-Based Modeling**: Define geometry using the Rhai scripting language
- **Real-Time Preview**: Instant 3D visualization with orbit/pan/zoom controls
- **SDF-Based Geometry**: Smooth, mathematically precise models
- **Mesh Export**: Export to STL and OBJ formats for 3D printing and rendering
- **Project Management**: Save/load projects with full state preservation
- **Batch Processing**: Headless mode for automation and CI/CD integration
- **Undo/Redo**: Full edit history for productive workflows

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
- **Script Examples**: 10 built-in example scripts
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

5. **Export** your model using the STL or OBJ buttons

6. **Save** your project with Ctrl+S for later editing

### Batch/Headless Mode

Process scripts without GUI for automation:

```bash
# Single file
implicit-cad --headless --script input.rhai --output result.stl

# Batch directory
implicit-cad --headless --batch ./scripts --batch-output ./models --format obj

# With custom resolution
implicit-cad --headless --script input.rhai --output result.stl --resolution 64 --smooth-normals
```

**CLI Options**:
- `--headless` - Run without GUI
- `--script <file>` - Input script path
- `--output <file>` - Output mesh path
- `--batch <dir>` - Process all .rhai files in directory
- `--batch-output <dir>` - Output directory for batch mode
- `--format <stl|obj>` - Export format (default: stl)
- `--resolution <num>` - Mesh grid resolution (default: 32)
- `--smooth-normals` - Enable smooth normal averaging

## Scripting API

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
