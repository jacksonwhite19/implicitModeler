# Phase 6: Aerospace Features — Detailed Plan

**Status**: Planning
**Priority**: High (follows Phase 5 completion)
**Target Use Cases**: Conceptual modeling → CFD optimization → Full vehicle construction

---

## Executive Summary

Phase 6 adds comprehensive aerospace-specific primitives and operations to enable rapid aircraft concept modeling and CFD-ready geometry generation. The focus is on parametric, script-driven component generation with automatic blending and optional smart assembly.

---

## Feature Categories

### A. Airfoil Library & Generation
### B. Wing Primitives
### C. Fuselage Generation
### D. Tail Surfaces
### E. Engine Nacelles
### F. Assembly & Blending
### G. Advanced Features (Future)

---

## Stage A: Airfoil Library & Generation (PRIORITY 1)

### Objective
Parametric NACA airfoil generation with automatic chord scaling.

### Implementation Requirements

#### A1. NACA Airfoil Parser & Generator

**Supported Series**:
- **4-digit series** (e.g., NACA 2412)
  - Format: MPXX where M=max camber (%), P=position (tenths), XX=thickness (%)
  - Example: `naca("2412", chord)` → 2% camber at 40% chord, 12% thickness

- **5-digit series** (e.g., NACA 23012)
  - Format: LPQXX where L=lift coefficient, P/Q=camber position, XX=thickness
  - Example: `naca("23012", chord)` → designed for CL=0.3, specific camber distribution

- **6-series (laminar flow)** (e.g., NACA 63₂-215)
  - Format: 6[series]-[cl][thickness]
  - Example: `naca("63-215", chord)` or `naca("632-215", chord)`
  - Designed for laminar flow over portions of the chord

#### A2. Scripting API

```rhai
// Basic usage - returns 2D airfoil profile (SDF cross-section)
let airfoil = naca("2412", 1.0);  // chord = 1.0 meter

// With explicit chord
let airfoil = naca("23012", 2.5);  // chord = 2.5 meters

// 6-series
let airfoil = naca("63-215", 1.0);
```

#### A3. Airfoil Properties

**Input Parameters**:
- `designation`: String (e.g., "2412", "23012", "63-215")
- `chord`: Float (chord length in model units)

**Output**: 2D SDF profile that can be:
- Extruded to create wing sections
- Lofted between different airfoils
- Used in `wing_with_airfoil()` primitives

**Internal Representation**:
- Compute upper and lower surface coordinates
- Convert to 2D SDF using signed distance to polyline or spline
- Cache common airfoils for performance

#### A4. Verification

- [ ] Generate NACA 0012 (symmetric) and verify thickness distribution
- [ ] Generate NACA 2412 (cambered) and verify camber line
- [ ] Generate NACA 23012 (5-digit) and compare to reference data
- [ ] Generate NACA 63-215 (6-series) and verify shape
- [ ] Test chord scaling from 0.1 to 100.0
- [ ] Verify SDF produces watertight extrusions

---

## Stage B: Wing Primitives (PRIORITY 1)

### Objective
Parametric wing generation with full geometric control.

### Implementation Requirements

#### B1. Wing with Airfoil Function

```rhai
wing_with_airfoil(
    airfoil,        // Airfoil profile (from naca() or custom)
    root_chord,     // Chord at wing root (meters)
    tip_chord,      // Chord at wing tip (meters)
    span,           // Total span (meters)
    sweep,          // Leading edge sweep angle (degrees)
    dihedral,       // Dihedral angle (degrees, + is up)
    twist           // Tip twist relative to root (degrees, + is nose up)
)
```

**Example Usage**:
```rhai
// Simple tapered wing
let airfoil = naca("2412", 1.0);
let wing = wing_with_airfoil(
    airfoil,
    2.0,      // root chord
    1.0,      // tip chord (taper ratio = 0.5)
    10.0,     // 10m span
    15.0,     // 15° sweep
    3.0,      // 3° dihedral
    -2.0      // -2° washout
);
```

#### B2. Geometric Controls

**Taper**:
- Linear chord variation from root to tip
- `taper_ratio = tip_chord / root_chord`

**Sweep**:
- Leading edge sweep angle
- Sweeps the 1/4 chord line (standard aerodynamic reference)
- Positive = swept back

**Dihedral/Anhedral**:
- Vertical angle of wing relative to fuselage
- Positive = tips higher than root (dihedral)
- Negative = tips lower than root (anhedral)

**Twist (Washout/Washin)**:
- Linear twist variation from root to tip
- Positive = nose-up at tip (washin)
- Negative = nose-down at tip (washout, typical)
- Applied about airfoil chord line

#### B3. Winglet/Wing Tip Handling

**Approach 1: Separate Wing Feature** (Recommended)
```rhai
// Main wing
let main_wing = wing_with_airfoil(naca("2412", 1.0), 2.0, 1.0, 10.0, 15.0, 3.0, -2.0);

// Winglet (small vertical wing)
let winglet = wing_with_airfoil(naca("0012", 1.0), 0.8, 0.4, 1.5, 5.0, 0.0, 0.0);
let winglet = rotate(winglet, 90.0, 0.0, 0.0);  // Vertical orientation
let winglet = translate(winglet, 0.0, 5.0, 0.5);  // Position at wing tip

// Combine
let wing_assembly = union(main_wing, winglet);
```

**Approach 2: Integrated Winglet** (Future Enhancement)
```rhai
let wing = wing_with_airfoil(...);
let wing = add_winglet(wing, height, cant_angle, taper);
```

#### B4. Internal Implementation

**Algorithm**:
1. Create airfoil cross-section SDF (2D)
2. Generate wing planform path (root to tip with sweep)
3. Apply dihedral transformation
4. Loft airfoil along path with:
   - Linear chord scaling (taper)
   - Linear twist variation
5. Return 3D SDF

**Performance Considerations**:
- Cache airfoil SDFs
- Optimize lofting for real-time preview
- Consider adaptive sampling for high-aspect-ratio wings

#### B5. Verification

- [ ] Generate rectangular wing (no taper, sweep, dihedral, twist)
- [ ] Test full taper (tip_chord = 0.5 * root_chord)
- [ ] Test sweep angles: 0°, 15°, 30°, 45°
- [ ] Test dihedral: -5°, 0°, +5°
- [ ] Test washout: -3° twist
- [ ] Combine all parameters simultaneously
- [ ] Verify smooth lofting with no artifacts
- [ ] Export to STL and verify watertight mesh

---

## Stage C: Fuselage Generation (PRIORITY 2)

### Objective
Lofted fuselage from cross-sections with parametric control.

### Implementation Requirements

#### C1. Fuselage from Sections

```rhai
fuselage_loft(
    sections,       // Array of (station, profile, scale)
    smooth          // Smoothing factor (0.0 = linear, 1.0 = smooth spline)
)
```

**Example Usage**:
```rhai
// Define cross-sections at stations
let sections = [
    (0.0,   "circle", 0.2),    // Nose: circular, radius 0.2m
    (2.0,   "circle", 1.0),    // Forward fuselage: radius 1.0m
    (8.0,   "ellipse", 1.0),   // Mid fuselage: ellipse, scale 1.0
    (12.0,  "circle", 0.8),    // Aft fuselage: circular, radius 0.8m
    (15.0,  "circle", 0.1)     // Tail: small circle
];

let fuselage = fuselage_loft(sections, 0.8);  // Smooth lofting
```

#### C2. Parametric Fuselage Constructor

```rhai
fuselage_parametric(
    total_length,       // Total fuselage length
    max_diameter,       // Maximum diameter (at widest point)
    nose_length,        // Length of nose cone
    tail_length,        // Length of tail cone
    nose_type,          // "cone", "elliptical", "ogive"
    tail_type,          // "cone", "elliptical", "boat-tail"
    cross_section       // "circular", "elliptical", "blended"
)
```

**Example Usage**:
```rhai
let fuselage = fuselage_parametric(
    15.0,           // 15m long
    2.0,            // 2m max diameter
    3.0,            // 3m nose
    2.0,            // 2m tail
    "elliptical",   // Smooth nose
    "cone",         // Tapered tail
    "circular"      // Round cross-section
);
```

#### C3. Cross-Section Types

**Supported Profiles**:
- **Circle**: `radius` parameter
- **Ellipse**: `width, height` parameters (or aspect ratio)
- **Blended/Area-Ruled**: Custom profiles for area ruling
- **Custom**: User-defined 2D SDF profile

**Auto-Generated Stations**:
If user doesn't specify stations, generate automatically:
- Dense stations at nose and tail (high curvature)
- Sparse stations at constant section (low curvature)
- Typical: 5-10 stations for simple fuselages

#### C4. Nose/Tail Shapes

**Nose Types**:
- `"cone"`: Linear taper (simple, drag-inefficient)
- `"elliptical"`: Smooth elliptical curve (good subsonic)
- `"ogive"`: Tangent ogive (good supersonic)
- `"parabolic"`: Parabolic curve (moderate drag)

**Tail Types**:
- `"cone"`: Linear taper
- `"elliptical"`: Smooth closure
- `"boat-tail"`: Upsweep (common for jet aircraft)
- `"blunt"`: Flat or slightly rounded (rockets)

#### C5. Lofting Algorithm

**Implementation**:
1. Parse section definitions (station, profile type, scale)
2. Generate 2D SDF for each profile type
3. Create 3D path along fuselage centerline
4. Interpolate between sections:
   - Linear interpolation if `smooth = 0.0`
   - Catmull-Rom or B-spline if `smooth > 0.0`
5. Blend cross-sections smoothly
6. Return 3D SDF

**Smoothing**:
- Use smooth min/max for SDF blending
- Apply smoothness factor to control blend radius
- Ensure no pinching or bulging artifacts

#### C6. Verification

- [ ] Generate simple cylindrical fuselage
- [ ] Test nose types: cone, elliptical, ogive
- [ ] Test tail types: cone, elliptical, boat-tail
- [ ] Verify smooth lofting between dissimilar sections
- [ ] Test elliptical vs circular cross-sections
- [ ] Generate area-ruled fuselage (wasp waist)
- [ ] Export and verify watertight mesh

---

## Stage D: Tail Surfaces (PRIORITY 2)

### Objective
Tail surfaces as scaled/rotated wing features (OpenVSP approach).

### Implementation Requirements

#### D1. Horizontal Stabilizer

```rhai
// Just a small wing!
let h_stab_airfoil = naca("0012", 1.0);  // Symmetric
let h_stab = wing_with_airfoil(
    h_stab_airfoil,
    1.5,        // Root chord
    0.8,        // Tip chord
    3.0,        // Span
    10.0,       // Slight sweep
    0.0,        // No dihedral (flat)
    0.0         // No twist
);

// Position at tail
let h_stab = translate(h_stab, 12.0, 0.0, 0.5);  // x, y, z
```

#### D2. Vertical Stabilizer

```rhai
// Vertical wing (rotated 90°)
let v_stab_airfoil = naca("0012", 1.0);
let v_stab = wing_with_airfoil(
    v_stab_airfoil,
    2.0,        // Root chord
    1.0,        // Tip chord
    2.5,        // "Span" (height)
    15.0,       // Sweep
    0.0,        // N/A for vertical
    0.0         // No twist
);

// Rotate to vertical and position
let v_stab = rotate(v_stab, 90.0, 0.0, 0.0);  // Rotate about X
let v_stab = translate(v_stab, 12.0, 0.0, 0.0);
```

#### D3. T-Tail Configuration

```rhai
// Horizontal stab mounted on top of vertical stab
let h_stab = translate(h_stab, 12.0, 0.0, 2.5);  // Mounted high
let assembly = union(v_stab, h_stab);
```

#### D4. V-Tail Configuration

```rhai
// Two surfaces at angles (dihedral)
let v_tail_1 = wing_with_airfoil(airfoil, 1.5, 0.8, 2.0, 10.0, 45.0, 0.0);
let v_tail_2 = wing_with_airfoil(airfoil, 1.5, 0.8, 2.0, 10.0, -45.0, 0.0);
let v_tail = union(v_tail_1, v_tail_2);
```

**No Control Surfaces** (as specified):
- Elevator, rudder not modeled separately
- Entire tail is solid surface
- Control deflections can be added in future phase if needed

#### D5. Verification

- [ ] Create conventional tail (horizontal + vertical)
- [ ] Create T-tail configuration
- [ ] Create V-tail configuration
- [ ] Verify proper positioning and orientation
- [ ] Test blending with fuselage (Stage F)

---

## Stage E: Engine Nacelles (PRIORITY 3)

### Objective
Simple nacelle primitive for engine pods.

### Implementation Requirements

#### E1. Nacelle Primitive

```rhai
nacelle(
    length,             // Total nacelle length
    max_diameter,       // Maximum diameter
    inlet_diameter,     // Inlet diameter (front)
    exhaust_diameter,   // Exhaust diameter (rear)
    inlet_type,         // "flush", "scoop", "pitot"
    exhaust_type        // "flush", "cone", "chevron"
)
```

**Example Usage**:
```rhai
let engine = nacelle(
    4.0,        // 4m long
    1.2,        // 1.2m max diameter
    1.0,        // 1.0m inlet
    0.9,        // 0.9m exhaust (slight taper)
    "scoop",    // Scoop inlet
    "cone"      // Cone exhaust
);

// Position under wing
let engine = translate(engine, 2.0, 3.0, -1.5);
```

#### E2. Nacelle Shapes

**Inlet Types**:
- `"flush"`: Flat inlet face
- `"scoop"`: Curved scoop (subsonic)
- `"pitot"`: Center body inlet (supersonic)
- `"lip"`: Rounded lip (high-bypass turbofan)

**Exhaust Types**:
- `"flush"`: Flat exhaust
- `"cone"`: Tapered cone (afterburning)
- `"chevron"`: Serrated edge (noise reduction)

**Body Shape**:
- Smooth ellipsoidal body
- Taper from max diameter to inlet/exhaust
- Support for pylon mounting (future)

#### E3. Internal Implementation

- Use fuselage lofting logic with pre-defined sections
- Inlet and exhaust as special end caps
- Return as SDF primitive

#### E4. Verification

- [ ] Generate simple cylindrical nacelle
- [ ] Test inlet types
- [ ] Test exhaust types
- [ ] Position multiple nacelles on aircraft
- [ ] Verify blending with wing (pylon mount)

---

## Stage F: Assembly & Blending (PRIORITY 1)

### Objective
Manual positioning with optional smart attachment and automatic blending at intersections.

### Implementation Requirements

#### F1. Manual Positioning (Current System)

```rhai
// Standard translate/rotate approach
let fuselage = fuselage_parametric(...);
let wing = wing_with_airfoil(...);

// Position wing manually
let wing = translate(wing, 5.0, 0.0, -0.5);  // x, y, z from origin
let wing = rotate(wing, 2.0, 0.0, 0.0);      // Incidence angle

let aircraft = union(fuselage, wing);
```

**Advantages**:
- Full control
- Explicit positioning
- Easy to understand

#### F2. Smart Attachment (Optional Enhancement)

```rhai
// Attach wing to fuselage at a station
let wing = attach_to_fuselage(
    wing,
    fuselage,
    station: 5.0,        // Fuselage station (x-position)
    vertical_offset: -0.5,   // Below centerline
    incidence: 2.0       // Incidence angle (degrees)
);
```

**Benefits**:
- Automatic positioning relative to fuselage
- Wing follows fuselage if fuselage changes
- Reduces manual calculation

**Implementation**:
- Query fuselage at station to get diameter/geometry
- Calculate attachment point
- Apply transforms automatically
- Return transformed wing

#### F3. Automatic Blending at Intersections ⭐ KEY FEATURE

```rhai
blend(body_a, body_b, radius)
```

**Example Usage**:
```rhai
let fuselage = fuselage_parametric(...);
let wing = wing_with_airfoil(...);
let wing = translate(wing, 5.0, 0.0, -0.5);

// Blend with 0.5m radius fillet
let aircraft = blend(fuselage, wing, 0.5);
```

**Default Behavior**:
- Simple constant-radius fillet (0.2m - 0.5m default)
- Smooth SDF blending using `smooth_union()`
- Applied at all intersections automatically

**Advanced Blending** (Future):
```rhai
// Custom blend profiles
let aircraft = blend_custom(
    fuselage,
    wing,
    profile: "elliptical",  // Elliptical fillet cross-section
    root_radius: 0.8,       // Larger radius at wing root
    tip_radius: 0.2         // Smaller radius toward tip
);
```

#### F4. Internal Implementation

**Simple Blend**:
- Use existing `smooth_union()` operation
- Radius parameter controls smoothness
- Default radius: 0.3m (adjustable)

**Advanced Blend** (Future):
- Variable radius along intersection curve
- Custom fillet profiles (circular, elliptical, parabolic)
- Curvature-continuous (G2) blending

#### F5. Example Aircraft Assembly

```rhai
// Airfoil
let airfoil = naca("2412", 1.0);

// Fuselage
let fuselage = fuselage_parametric(
    15.0,           // 15m long
    2.0,            // 2m diameter
    3.0,            // 3m nose
    2.0,            // 2m tail
    "elliptical",
    "cone",
    "circular"
);

// Main wing
let wing = wing_with_airfoil(airfoil, 2.0, 1.0, 10.0, 15.0, 3.0, -2.0);
let wing = translate(wing, 5.0, 0.0, -0.5);

// Blend wing to fuselage
let aircraft = blend(fuselage, wing, 0.4);

// Horizontal stabilizer
let h_stab = wing_with_airfoil(naca("0012", 1.0), 1.5, 0.8, 3.0, 10.0, 0.0, 0.0);
let h_stab = translate(h_stab, 12.0, 0.0, 0.5);

// Vertical stabilizer
let v_stab = wing_with_airfoil(naca("0012", 1.0), 2.0, 1.0, 2.5, 15.0, 0.0, 0.0);
let v_stab = rotate(v_stab, 90.0, 0.0, 0.0);
let v_stab = translate(v_stab, 12.0, 0.0, 0.0);

// Combine tail
let tail = union(h_stab, v_stab);
let aircraft = blend(aircraft, tail, 0.3);

// Engine nacelles
let nacelle = nacelle(4.0, 1.2, 1.0, 0.9, "scoop", "cone");
let nacelle_left = translate(nacelle, 4.0, -3.0, -1.5);
let nacelle_right = translate(nacelle, 4.0, 3.0, -1.5);

let aircraft = blend(aircraft, nacelle_left, 0.2);
let aircraft = blend(aircraft, nacelle_right, 0.2);

// Final aircraft
aircraft
```

#### F6. Verification

- [ ] Test simple blend between fuselage and wing
- [ ] Verify blend radius parameter works correctly
- [ ] Test multiple blends (wing + tail + nacelles)
- [ ] Ensure no artifacts or surface discontinuities
- [ ] Export to STL and verify watertight mesh
- [ ] Visual inspection of blend quality

---

## Stage G: Advanced Features (FUTURE)

### G1. Field-Driven Optimization
- Variable thickness based on stress fields
- Topology optimization integration
- Lattice infill for internal structures

### G2. Control Surfaces (Future)
- Separate elevator, rudder, aileron geometry
- Deflection angles
- Hinge lines and gaps

### G3. Landing Gear (Future)
- Gear leg primitives
- Wheel assemblies
- Retraction kinematics

### G4. Propeller/Rotor (Future)
- Blade generation from airfoil sections
- Twist and taper
- Hub geometry

---

## Implementation Priorities

### Phase 6A (Essential - Implement First)
1. **Airfoil Library** (Stage A) - Foundation for all wing surfaces
2. **Wing Primitives** (Stage B) - Core aerodynamic surfaces
3. **Blending** (Stage F3) - Critical for realistic aircraft geometry

### Phase 6B (Core Features)
4. **Fuselage** (Stage C) - Complete airframe modeling
5. **Tail Surfaces** (Stage D) - Use existing wing primitives

### Phase 6C (Extensions)
6. **Nacelles** (Stage E) - Propulsion integration
7. **Smart Attachment** (Stage F2) - Productivity enhancement

### Phase 6D (Advanced - Future)
8. **Advanced Blending** (Stage F4) - Variable radius, custom profiles
9. **Field-Driven Features** (Stage G1) - Optimization integration
10. **Control Surfaces** (Stage G2) - Detailed modeling

---

## Testing Strategy

### Unit Tests
- [ ] NACA airfoil coordinate generation (compare to reference data)
- [ ] Wing planform geometry calculations
- [ ] Fuselage section interpolation
- [ ] Blend radius calculations

### Integration Tests
- [ ] Full aircraft assembly (fuselage + wing + tail)
- [ ] Export to STL and verify in external viewer
- [ ] Measure geometric properties (span, chord, area)
- [ ] CFD mesh generation readiness

### Example Scripts
Create example aircraft configurations:
- [ ] Simple trainer aircraft (straight wing, single engine)
- [ ] Business jet (swept wing, T-tail, twin engines)
- [ ] Airliner (high-aspect-ratio wing, underwing engines)
- [ ] Fighter concept (delta wing, area-ruled fuselage)

---

## Technical Considerations

### Performance
- Cache airfoil SDFs (avoid recomputation)
- Optimize lofting for real-time preview
- Consider LOD for complex assemblies

### Accuracy
- NACA airfoil equations must match reference data
- Wing geometry tolerances: ±0.1% chord
- Blend quality: no visible discontinuities

### Export Quality
- Watertight meshes for CFD
- Sufficient resolution for curvature capture
- Support for high-resolution export (128³+ grid)

---

## Dependencies

### New Crates (Potential)
```toml
# For spline interpolation (fuselage lofting)
splines = "4.0"  # Or implement custom Catmull-Rom

# For airfoil coordinate calculations
# (May implement directly - equations are well-known)
```

### Internal Modules
```
src/
├── aerospace/
│   ├── mod.rs
│   ├── airfoil.rs       // NACA airfoil generation
│   ├── wing.rs          // Wing primitives
│   ├── fuselage.rs      // Fuselage lofting
│   ├── nacelle.rs       // Nacelle primitives
│   └── blend.rs         // Advanced blending operations
```

---

## Success Criteria

Phase 6 is complete when:

✅ User can generate NACA 4/5/6-series airfoils parametrically
✅ User can create wings with full geometric control (taper, sweep, dihedral, twist)
✅ User can create fuselages via lofted sections or parametric definition
✅ User can assemble complete aircraft with automatic blending
✅ Exported geometry is watertight and CFD-ready
✅ Example scripts demonstrate trainer, business jet, and airliner configurations
✅ Performance is acceptable (<2s for typical aircraft at 32³ resolution)

---

## Workflow Example: CFD Optimization Loop

```rhai
// Initial design
let airfoil = naca("2412", 1.0);
let wing = wing_with_airfoil(airfoil, 2.0, 1.0, 10.0, 15.0, 3.0, -2.0);
let fuselage = fuselage_parametric(15.0, 2.0, 3.0, 2.0, "elliptical", "cone", "circular");
let aircraft = blend(fuselage, wing, 0.4);

// Export to STL for CFD
// (Run CFD analysis externally)

// Iterate - modify sweep angle based on CFD results
let wing = wing_with_airfoil(airfoil, 2.0, 1.0, 10.0, 20.0, 3.0, -2.0);  // 20° sweep
let aircraft = blend(fuselage, wing, 0.4);

// Re-export and re-analyze
// Continue iteration...
```

**Key Advantages**:
- Parametric: Easy to modify any parameter
- Fast iteration: Script-based, not GUI clicking
- Repeatable: Script is version-controlled
- Automatable: Can be driven by optimization algorithm

---

## Future Integration Points

### Optimization Frameworks
- Connect to Dakota, OpenMDAO, or custom optimizers
- Automatic parameter sweeps
- Multi-objective optimization (drag, weight, cost)

### CFD Integration
- Direct export to OpenFOAM, SU2, ANSYS Fluent
- Automatic surface mesh refinement
- Volume mesh generation preparation

### Manufacturing
- Export for 3D printing (wind tunnel models)
- Mold generation for composite layup
- CNC toolpath preparation

---

## Conclusion

Phase 6 transforms the Implicit CAD tool into a **specialized aerospace design platform** while maintaining the core SDF-based architecture. The parametric, script-driven approach enables rapid iteration and optimization workflows critical for modern aircraft design.

**Estimated Implementation Time**: 3-4 weeks for Phase 6A-6B (essential + core features)

**Next Steps**:
1. Review and approve this plan
2. Create detailed task list for Phase 6A (airfoils, wings, blending)
3. Begin implementation with airfoil library
