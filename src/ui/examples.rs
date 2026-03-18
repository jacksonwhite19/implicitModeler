// Example scripts for the in-app example browser.
#![allow(dead_code)]

// ---------------------------------------------------------------------------
// Data types
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq)]
pub enum Difficulty {
    Beginner,
    Intermediate,
    Advanced,
}

impl Difficulty {
    pub fn label(self) -> &'static str {
        match self {
            Difficulty::Beginner     => "Beginner",
            Difficulty::Intermediate => "Intermediate",
            Difficulty::Advanced     => "Advanced",
        }
    }
}

pub struct ExampleScript {
    pub id:                &'static str,
    pub title:             &'static str,
    pub category:          &'static str,
    pub difficulty:        Difficulty,
    pub description:       &'static str,
    pub script:            &'static str,
    pub related_functions: &'static [&'static str],
    pub tags:              &'static [&'static str],
}

pub fn get_examples() -> &'static [ExampleScript] {
    EXAMPLES
}

pub fn examples_by_category(cat: &str) -> impl Iterator<Item = &'static ExampleScript> {
    EXAMPLES.iter().filter(move |e| e.category == cat)
}

pub static CATEGORIES: &[&str] = &[
    "Getting Started",
    "Fuselage Design",
    "Wing Design",
    "Control Surfaces",
    "Internal Structure",
    "Materials and Lattice",
    "Inlets and Ducts",
    "Fabrication",
    "Analysis",
    "Complete Assemblies",
];

// ---------------------------------------------------------------------------
// Scripts
// ---------------------------------------------------------------------------

static EX01_SCRIPT: &str = r#"
// Example 01 — Basic Primitives and Booleans
// Demonstrates the core SDF primitive set and boolean operations.

// === Primitives ===
let s  = sphere(20.0);               // radius 20 mm
let b  = box_(40.0, 30.0, 15.0);    // width, height, depth
let c  = cylinder(10.0, 40.0);      // radius, height
let t  = torus(25.0, 6.0);          // major radius, minor radius

// Translate so shapes don't overlap
let b2 = translate(b, 60.0, 0.0, 0.0);
let c2 = translate(c, 0.0, 60.0, 0.0);
let t2 = translate(t, -60.0, 0.0, 0.0);

// === Booleans ===
// Union: combine two solids into one
let combined = union(s, b2);

// Subtract: drill a cylinder through a box
let drilled = subtract(box_(30.0, 30.0, 30.0), cylinder(8.0, 40.0));
let drilled = translate(drilled, 0.0, -70.0, 0.0);

// Intersect: keep only the region shared by both shapes
let isect_base = sphere(25.0);
let isect_cut  = box_(30.0, 30.0, 30.0);
let intersected = intersect(isect_base, isect_cut);
let intersected  = translate(intersected, -70.0, -70.0, 0.0);

// Smooth union: organic blend between two spheres
let sa = sphere(15.0);
let sb = translate(sphere(15.0), 22.0, 0.0, 0.0);
let smooth = smooth_union(sa, sb, 8.0);
let smooth  = translate(smooth, 70.0, -70.0, 0.0);

// Final scene: everything together
union(union(union(union(union(combined, c2), t2), drilled), intersected), smooth)
"#;

static EX02_SCRIPT: &str = r#"
// Example 02 — Transforms and Patterns
// Shows translate, rotate, scale, linear_array, polar_array, and mirror_y.

// === Base shape ===
let peg = cylinder(4.0, 20.0);  // small peg to copy around

// === Linear array ===
// 5 pegs spaced 15 mm apart along X
let row = linear_array(peg, 5, 15.0, 0.0, 0.0);

// === Polar array ===
// 8 copies of a small box arranged in a ring
let tooth = translate(box_(5.0, 3.0, 8.0), 20.0, 0.0, 0.0);
let ring  = polar_array(tooth, 8);

// Raise the ring above the linear row
let ring = translate(ring, 0.0, 0.0, 30.0);

// === Scale ===
let base = box_(20.0, 20.0, 5.0);
let stretched = scale(base, 2.0, 1.0, 1.0);  // 2x along X
let stretched = translate(stretched, 0.0, 50.0, 0.0);

// === Rotate ===
let bar = box_(60.0, 8.0, 8.0);
let bar_rot = rotate(bar, 0.0, 0.0, 45.0);   // 45 degrees about Z
let bar_rot = translate(bar_rot, 0.0, -50.0, 0.0);

// === Mirror ===
// Build a half-shape, then mirror to get symmetry
let half_fin = box_(40.0, 3.0, 20.0);
let half_fin = translate(half_fin, 20.0, 0.0, -30.0);
let full_fin = mirror_y(half_fin);

union(union(union(union(row, ring), stretched), bar_rot), full_fin)
"#;

static EX03_SCRIPT: &str = r#"
// Example 03 — Fields and Field-Driven Geometry
// Scalar fields modulate geometry thickness and blending continuously
// across space — impossible in traditional B-Rep CAD.

// === Two base shapes ===
let box_a = box_(40.0, 40.0, 40.0);
let box_b = translate(sphere(28.0), 30.0, 0.0, 0.0);

// === Gyroid field ===
// Encodes a periodic minimal-surface density pattern (scale in mm)
let gf = gyroid_field(12.0);

// blend_by_field: where field is high -> shape A, low -> shape B
// This creates a spatially varying boolean that morphs between the two
let blended = blend_by_field(box_a, box_b, gf);

// === offset_by_field ===
// Shell a cylinder, then thicken the wall using a radial density gradient
let tube  = shell(cylinder(20.0, 60.0), 2.0);
let rgrad = radial_field(0.0, 0.0, 0.0);    // distance from Z-axis
let bumpy = offset_by_field(tube, rgrad, 3.0);  // +/-3 mm driven by radius
let bumpy = translate(bumpy, 80.0, 0.0, 0.0);

// === Gradient field for directional taper ===
// Thicken more toward +Z
let slab      = box_(50.0, 50.0, 20.0);
let slab_sh   = shell(slab, 1.5);
let z_grad    = gradient_field(0.0, 0.0, 1.0);
let tapered   = offset_by_field(slab_sh, z_grad, 4.0);
let tapered   = translate(tapered, -80.0, 0.0, 0.0);

union(union(blended, bumpy), tapered)
"#;

static EX04_SCRIPT: &str = r#"
// Example 04 — Station-Based Fuselage
// Build a fuselage from explicit cross-section stations.
// Each station is [axial_position_mm, cross_section_SDF].
// Reference aircraft: 200 mm diameter, 600 mm long.

// === Cross-section helpers ===
// ellipse_section(rx, rz) is wider on sides — good for
// fuselages with a flat belly or high wing configuration.

let stations = [
    [0.00, circle_section(10.0)],           // nose tip
    [0.10, ellipse_section(80.0, 75.0)],
    [0.30, ellipse_section(100.0, 95.0)],   // max diameter station
    [0.58, ellipse_section(100.0, 95.0)],   // constant section through cabin
    [0.83, ellipse_section(70.0, 60.0)],
    [1.00, circle_section(12.0)],           // tail tip
];

let fuse = fuselage(stations);

// Apply a shell to see the outer skin only (optional for display)
// let fuse_shell = shell(fuse, 2.5);

fuse
"#;

static EX05_SCRIPT: &str = r#"
// Example 05 — Fuselage with Spline Cross-Sections
// The profile editor lets you draw arbitrary cross-section shapes by name.
// Those named profiles are then referenced here with spline_fuselage().
//
// WORKFLOW:
//   1. Open the Profile Editor panel (sidebar -> Profiles).
//   2. Draw or import your desired cross-section shape.
//   3. Give it a name, e.g. "fuse_body".
//   4. Reference it here with spline_fuselage("fuse_body", stations).
//
// The fallback below uses circle_section / ellipse_section so this script
// is always runnable even before you create named profiles.

// Once profiles are defined in the editor, replace station SDFs with:
//   spline_fuselage("fuse_body", stations)
let stations = [
    [0.00, circle_section(8.0)],
    [0.13, ellipse_section(90.0, 80.0)],
    [0.33, ellipse_section(100.0, 95.0)],
    [0.67, ellipse_section(100.0, 95.0)],
    [0.87, ellipse_section(65.0, 55.0)],
    [1.00, circle_section(10.0)],
];

let fuse = fuselage(stations);

// Parametric fuselage variant: simpler, single smooth body
// nose_sharp and tail_sharp in [0,1]: 0 = blunt, 1 = sharp
let fuse_param = fuselage_parametric(600.0, 200.0, 0.6, 0.8);
let fuse_param = translate(fuse_param, 0.0, 250.0, 0.0);

union(fuse, fuse_param)
"#;

static EX06_SCRIPT: &str = r#"
// Example 06 — Longitudinal Spine Constraints (Hard-Chine Hull)
// The spine editor lets you define keel, deck, and chine curves as
// 3-D splines. The fuselage loft then respects those edge curves,
// producing ruled surfaces ideal for composite or foam construction.
//
// WORKFLOW:
//   1. Open Spine Editor (sidebar -> Spines).
//   2. Define "keel", "deck", "port_chine", "starboard_chine" curves.
//   3. The fuselage builder will loft between them automatically.
//
// Fallback below shows a typical hard-chine hull cross-section using
// the station-based builder with rect_section (flat panels).

// Hard-chine cross-sections using rect_section for flat panel sides
let stations = [
    [0.00, circle_section(12.0)],
    [0.08, rect_section(60.0, 50.0)],
    [0.25, rect_section(140.0, 80.0)],
    [0.50, rect_section(160.0, 90.0)],
    [0.75, rect_section(140.0, 80.0)],
    [0.93, rect_section(80.0,  50.0)],
    [1.00, circle_section(14.0)],
];

let hull = fuselage(stations);

// Offset inward to create hull shell thickness (3 mm skin)
let inner = offset(hull, -3.0);
let shell_body = subtract(hull, inner);

shell_body
"#;

static EX07_SCRIPT: &str = r#"
// Example 07 — Von Karman Nose and Haack Tail Cones
// Aerodynamically optimized nose and tail minimize wave drag.
// Reference aircraft: 200 mm diameter, 600 mm body total.

// === Nose cone — Von Karman profile ===
// length 120 mm, base diameter 200 mm
let nose = von_karman_nose(120.0, 200.0);

// === Cylindrical center body ===
let body_len = 360.0;
let body = cylinder(100.0, body_len);
// Place body immediately aft of nose
let body = translate(body, 0.0, 0.0, 120.0 + body_len / 2.0);

// === Tail cone ===
// tail_cone(length, diam_start, diam_end)
let tail = tail_cone(120.0, 200.0, 24.0);
let tail = translate(tail, 0.0, 0.0, 480.0);

// === Haack alternative nose (lower Cd for subsonic blunt-body designs) ===
let haack = haack_nose(120.0, 200.0);
let haack = translate(haack, 0.0, 250.0, 0.0);

// Assemble main fuselage
let fuse = union(union(nose, body), tail);

union(fuse, haack)
"#;

static EX08_SCRIPT: &str = r#"
// Example 08 — Wing with Airfoil, Taper and Dihedral
// Reference aircraft half-wing, then mirror for full span.
// NACA 2412: 2% camber, max at 40% chord, 12% thick.

// wing_with_airfoil(naca4, root_chord, tip_chord, half_span,
//                   sweep_deg, dihedral_deg, twist_deg)
let half_wing = wing_with_airfoil(
    "2412",   // NACA designation
    140.0,    // root chord mm
    100.0,    // tip chord mm
    400.0,    // half-span mm
    3.0,      // LE sweep degrees
    2.0,      // dihedral degrees (tip up)
    -1.5      // washout: tip LE pitched down relative to root
);

// Mirror about Y-axis for the full symmetric wing
let full_wing = mirror_y(half_wing);

// Position wing at fuselage mid-body station (200 mm from nose)
let full_wing = translate(full_wing, 0.0, 0.0, 200.0);

full_wing
"#;

static EX09_SCRIPT: &str = r#"
// Example 09 — Wing from Multiple Sections
// wing_from_sections lets you specify an explicit airfoil, chord, and
// position for each span station. Useful when you need different airfoils
// at root, mid-span, and tip (e.g. thicker root for structure, thinner tip).
//
// sections format: array of [y_mm, chord_mm, naca_string, twist_deg, sweep_offset_mm]

let sections = [
    [  0.0, 140.0, "2412",  0.0,  0.0],   // root: NACA 2412, full chord
    [200.0, 122.0, "2412", -0.8, 10.5],   // mid-span: slight taper and sweep
    [400.0, 100.0, "2409", -1.5, 21.0],   // tip: thinner NACA 2409 + washout
];

let half_wing = wing_from_sections(sections);

// Mirror for full wing
let full_wing = mirror_y(half_wing);

full_wing
"#;

static EX10_SCRIPT: &str = r#"
// Example 10 — Elevon Configuration (Flying Wing)
// A high-sweep flying wing with elevons for pitch and roll control.
// Elevons combine elevator and aileron function — no separate tail needed.

// High-sweep delta-ish wing: root 200 mm, tip 60 mm, 25 deg sweep, no dihedral
let half_fw = wing_with_airfoil("4412", 200.0, 60.0, 350.0, 25.0, 0.0, -2.0);

// elevon(wing, span_start_frac, span_end_frac, chord_frac)
// Inboard elevon: 40-65% of half-span, 25% of local chord
let elev_inboard  = elevon(half_fw, 0.40, 0.65, 0.25);

// Outboard elevon: 68-92% of half-span
let elev_outboard = elevon(half_fw, 0.68, 0.92, 0.25);

// Union control surfaces onto the wing half
let half_with_cs = union(union(half_fw, elev_inboard), elev_outboard);

// Mirror for full flying wing
let full_fw = mirror_y(half_with_cs);

full_fw
"#;

static EX11_SCRIPT: &str = r#"
// Example 11 — Horizontal and Vertical Tail Assembly
// Reference aircraft: H-stab 280 mm span, 70 mm chord NACA 0009
//                     V-stab 120 mm height, 80 mm chord NACA 0009

// === Horizontal stabiliser ===
// Symmetric airfoil (0009), slight taper, no dihedral
let half_htail = wing_with_airfoil("0009", 70.0, 50.0, 140.0, 5.0, 0.0, 0.0);
let htail = mirror_y(half_htail);
// Position aft of main wing: ~520 mm from nose, mounted on centreline
let htail = translate(htail, 0.0, 0.0, 520.0);

// === Vertical stabiliser ===
// Built as a wing rotated 90 degrees about the longitudinal axis
let vtail = wing_with_airfoil("0009", 80.0, 55.0, 120.0, 8.0, 0.0, 0.0);
// Rotate 90 deg so span points upward (Z axis)
let vtail = rotate(vtail, 90.0, 0.0, 0.0);
// Place on top of tail section
let vtail = translate(vtail, 0.0, 0.0, 510.0);

// === Tail assembly ===
union(htail, vtail)
"#;

static EX12_SCRIPT: &str = r#"
// Example 12 — Aileron and Split Flap
// Outboard aileron for roll control, inboard split flap for lift augmentation.
// Reference wing: root 140 mm, tip 100 mm, half-span 400 mm.

let half_wing = wing_with_airfoil("2412", 140.0, 100.0, 400.0, 3.0, 2.0, -1.5);

// === Aileron ===
// aileron(wing, span_start_frac, span_end_frac, chord_frac, thickness_mm)
// Outboard 55-92% of half-span, 28% chord
let ail = aileron(half_wing, 0.55, 0.92, 0.28, 2.5);

// === Inboard flap (split flap approximation) ===
// A split flap is the lower surface only — approximate with a subtract.
// Flap region: 10-50% of half-span, 30% chord depth
let flap_vol = box_(120.0, 4.0, 42.0);
let flap_vol = translate(flap_vol, 0.0, 80.0, -10.0);  // inboard position
let wing_with_flap_gap = subtract(half_wing, flap_vol);

// Combine: wing (with flap gap) + aileron hinge line
let half_complete = union(wing_with_flap_gap, ail);

// Mirror for full wing
mirror_y(half_complete)
"#;

static EX13_SCRIPT: &str = r#"
// Example 13 — Elevator and Rudder
// Conventional tail: elevator on H-stab, rudder on V-stab.

// === H-stab ===
let half_htail = wing_with_airfoil("0009", 70.0, 50.0, 140.0, 5.0, 0.0, 0.0);
let htail = mirror_y(half_htail);

// elevator(htail, span_frac, chord_frac)
// Full-span elevator: 0->1, rear 35% of chord
let elev = elevator(htail, 1.0, 0.35);
let htail_with_elev = union(htail, elev);
let htail_with_elev = translate(htail_with_elev, 0.0, 0.0, 520.0);

// === V-stab ===
let vtail = wing_with_airfoil("0009", 80.0, 55.0, 120.0, 8.0, 0.0, 0.0);
let vtail = rotate(vtail, 90.0, 0.0, 0.0);

// rudder(vtail, span_frac, chord_frac)
// Upper 80% of V-stab span, rear 38% of chord
let rud = rudder(vtail, 0.80, 0.38);
let vtail_with_rud = union(vtail, rud);
let vtail_with_rud = translate(vtail_with_rud, 0.0, 0.0, 510.0);

union(htail_with_elev, vtail_with_rud)
"#;

static EX14_SCRIPT: &str = r#"
// Example 14 — Internal Component Placement
// Battery, FC, ESC, and four servos with keepout volumes.
// component_named(name, sdf, margin_mm, mass_g) creates a tagged component.

// === Define components ===
// Battery: 3S 2200 mAh LiPo, approx 115x35x25 mm
let battery = component_named("battery",          box_(115.0, 35.0, 25.0), 3.0, 185.0);
let fc       = component_named("flight_controller", box_(36.0, 36.0, 8.0),  5.0,  38.0);
let esc      = component_named("esc",               box_(60.0, 25.0, 12.0), 3.0,  35.0);
let servo    = component_named("servo",             box_(23.0, 12.0, 24.0), 2.0,   9.0);

// === Place in fuselage (origin at nose) ===
let batt_p = place(battery,  50.0,  0.0, -5.0);   // forward bay
let fc_p   = place(fc,      185.0,  0.0, 10.0);   // centre
let esc_p  = place(esc,     165.0,  0.0, 25.0);   // above FC
let srv1   = place(servo,   280.0,  40.0,  0.0);
let srv2   = place(servo,   280.0, -40.0,  0.0);
let srv3   = place(servo,   540.0,  30.0,  0.0);
let srv4   = place(servo,   540.0, -30.0,  0.0);

// === Interference check (components vs each other) ===
let names    = ["battery","flight_controller","esc","servo","servo","servo","servo"];
let keepouts = [keepout(batt_p), keepout(fc_p), keepout(esc_p),
                keepout(srv1), keepout(srv2), keepout(srv3), keepout(srv4)];
let result   = interference_check_no_parent(names, keepouts);
// result.has_critical_interference -> bool
// result.pairs                     -> array of conflicting name pairs

// Return all geometry for visualisation
union(union(union(union(union(union(
    geometry(batt_p),
    geometry(fc_p)),
    geometry(esc_p)),
    geometry(srv1)),
    geometry(srv2)),
    geometry(srv3)),
    geometry(srv4))
"#;

static EX15_SCRIPT: &str = r#"
// Example 15 — Ribs and Spars
// Structural elements generated directly from wing geometry.
// rib_slab and spar_cylinder are aware of the wing's twist and taper.

let half_wing = wing_with_airfoil("2412", 140.0, 100.0, 400.0, 3.0, 2.0, -1.5);

// === Ribs ===
// rib_slab(wing, span_frac, thickness_mm) -> flat rib cut from airfoil profile
let rib_root = rib_slab(half_wing, 0.05, 2.0);   // near root
let rib_mid  = rib_slab(half_wing, 0.50, 2.0);   // mid-span
let rib_tip  = rib_slab(half_wing, 0.90, 2.0);   // near tip

// === Spars ===
// spar_cylinder(wing, chord_frac, radius_mm) -> tube running full half-span
let front_spar = spar_cylinder(half_wing, 0.25, 4.0);   // quarter-chord (main spar)
let rear_spar  = spar_cylinder(half_wing, 0.70, 2.5);   // rear spar

// === Assemble: wing skin + ribs + spars ===
let wing_shell = shell(half_wing, 1.2);  // 1.2 mm skin

let structure = union(union(union(union(
    wing_shell,
    rib_root),
    rib_mid),
    rib_tip),
    front_spar);

union(structure, rear_spar)
"#;

static EX16_SCRIPT: &str = r#"
// Example 16 — Bulkheads with Lightening Holes
// Fuselage bulkheads at key structural stations.
// lightening_hole_pattern removes circular pockets to save mass.

let stations = [
    [0.00, circle_section(10.0)],
    [0.10, ellipse_section(80.0, 75.0)],
    [0.30, ellipse_section(100.0, 95.0)],
    [0.58, ellipse_section(100.0, 95.0)],
    [0.83, ellipse_section(70.0,  60.0)],
    [1.00, circle_section(12.0)],
];
let fuse = fuselage(stations);
let fuse_shell = shell(fuse, 2.5);

// === Bulkheads at three stations ===
// bulkhead_at_station(fuse, axial_pos_mm, thickness_mm)
let bh1 = bulkhead_at_station(fuse, 100.0, 3.0);  // forward
let bh2 = bulkhead_at_station(fuse, 280.0, 3.0);  // centre (wing attach)
let bh3 = bulkhead_at_station(fuse, 460.0, 3.0);  // tail

// === Lightening holes ===
// lightening_hole_pattern(slab, n_holes, hole_radius_mm, margin_mm)
let bh1_l = lightening_hole_pattern(bh1, 6, 12.0, 8.0);
let bh2_l = lightening_hole_pattern(bh2, 8, 14.0, 8.0);  // larger centre BH
let bh3_l = lightening_hole_pattern(bh3, 5, 10.0, 7.0);

// === Union all ===
union(union(union(
    fuse_shell,
    bh1_l),
    bh2_l),
    bh3_l)
"#;

static EX17_SCRIPT: &str = r#"
// Example 17 — Composite Wing Sandwich
// Carbon-fibre-foam-carbon sandwich panel construction.
// layup_config defines the laminate; sandwich_panel applies it.

let half_wing = wing_with_airfoil("2412", 140.0, 100.0, 400.0, 3.0, 2.0, -1.5);
let wing_skin_sdf = shell(half_wing, 0.5);  // thin surface for skin reference

// === Layup configuration ===
// layup_config(n_plies, ply_angle_deg, ply_thick_mm, material_name)
let layup = layup_config(2, 45.0, 0.12, "carbon_ud");
// Add additional plies for quasi-isotropic laminate
let layup = add_layer(layup,   0.0, 0.12);   // 0 deg ply
let layup = add_layer(layup, -45.0, 0.12);   // -45 deg ply
let layup = add_layer(layup,  90.0, 0.12);   // 90 deg ply (closes QI set)

// === Foam core ===
// foam_core(sdf, core_thickness_mm) — lightweight structural core
let core = foam_core(half_wing, 6.0);

// === Sandwich panel ===
// sandwich_panel(skin_sdf, core_thick_mm, skin_thick_mm)
// 4-ply skin each face: 4 * 0.12 = 0.48 mm
let sandwich = sandwich_panel(wing_skin_sdf, 6.0, 0.48);

// Mirror for full wing
mirror_y(sandwich)
"#;

static EX18_SCRIPT: &str = r#"
// Example 18 — Conformal Gyroid Lattice Infill
// Gyroid lattice fills the interior of a fuselage section,
// oriented to follow its curved surfaces (conformal).

let stations = [
    [0.00, circle_section(10.0)],
    [0.15, ellipse_section(90.0, 85.0)],
    [0.37, ellipse_section(100.0, 95.0)],
    [0.65, ellipse_section(100.0, 95.0)],
    [0.89, ellipse_section(60.0, 55.0)],
    [1.00, circle_section(12.0)],
];
let fuse = fuselage(stations);

// === Outer skin (2.5 mm thick) ===
let fuse_skin = shell(fuse, 2.5);

// === Conformal gyroid fills the solid interior ===
// conformal_gyroid(sdf, scale_mm, strut_thickness_mm)
// scale = size of one gyroid cell; thickness = strut width
let infill = conformal_gyroid(fuse, 14.0, 1.2);

// The conformal variant aligns cell orientation with the SDF gradient,
// giving smoother surface transitions than the uniform gyroid.

// Union skin + lattice infill (lattice is already bounded by fuse SDF)
union(fuse_skin, infill)
"#;

static EX19_SCRIPT: &str = r#"
// Example 19 — NACA Flush Inlet
// A NACA flush inlet is recessed into the OML surface with no protruding lip,
// giving minimum drag at subsonic speeds. The characteristic 7° ramp angle
// creates the sub-atmospheric pressure that scoops in boundary-layer air.

// === Reference fuselage ===
let stations = [
    [0.00, circle_section(10.0)],
    [0.14, ellipse_section(90.0, 85.0)],
    [0.34, ellipse_section(100.0, 95.0)],
    [0.60, ellipse_section(100.0, 95.0)],
    [0.86, ellipse_section(60.0, 55.0)],
    [1.00, circle_section(12.0)],
];
let fuse = fuselage(stations);

// === NACA flush inlet ===
// naca_flush_inlet(width_mm, length_mm, depth_mm, fuse_sdf)
// Automatically positions the inlet on the top surface of the fuselage.
// The 7° ramp angle and tapering side walls are built in.
let inlet = naca_flush_inlet(30.0, 60.0, 18.0, fuse);

// Subtract the inlet pocket from the fuselage
let fuse_with_inlet = subtract(fuse, inlet);

// View as a shell to see the inner surface clearly
shell(fuse_with_inlet, 2.0)
"#;

static EX20_SCRIPT: &str = r#"
// Example 20 — EDF Buried Inlet with S-Duct
// A buried inlet scoops air from the fuselage surface into a fan housed
// within the body, with an S-shaped duct offsetting from the inlet axis
// to the fan centreline. Common in UAV and subsonic EDF designs.

// === Main fuselage body ===
let fuse = fuselage_parametric(600.0, 200.0, 0.55, 0.75);

// === Buried inlet scoop ===
// buried_inlet(throat_radius_mm, duct_length_mm, surface_offset_mm)
// Creates an elliptical surface scoop blending into a cylindrical duct.
// Throat radius 30 mm (60 mm dia), 120 mm duct, 90 mm from centreline.
let b_inlet = buried_inlet(30.0, 120.0, 90.0);
let b_inlet = translate(b_inlet, 120.0, 0.0, 0.0);  // 120 mm from nose

// Cut inlet into fuselage
let fuse_cut = subtract(fuse, b_inlet);

// === S-duct transition ===
// s_duct(inlet_r_mm, exit_r_mm, length_mm, lateral_offset_mm)
// Transitions from inlet axis to fan centreline offset 35 mm downward.
let s = s_duct(30.0, 27.0, 200.0, -35.0);
let s = translate(s, 150.0, 0.0, 55.0);

// === EDF nacelle / fan housing ===
let nacelle = shell(cylinder(32.0, 65.0), 3.0);
let nacelle = rotate(nacelle, 0.0, 90.0, 0.0);
let nacelle = translate(nacelle, 385.0, 0.0, 20.0);

// === Exhaust nozzle ===
let nozzle = shell(cone(32.0, 40.0), 2.0);
let nozzle = rotate(nozzle, 0.0, 90.0, 0.0);
let nozzle = translate(nozzle, 450.0, 0.0, 20.0);

// Assemble: fuselage + inlet cut + S-duct + nacelle + nozzle
union(union(union(fuse_cut, s), nacelle), nozzle)
"#;

static EX21_SCRIPT: &str = r#"
// Example 21 — Sweep: Cable Routing Conduits
// Sweeping a circular profile along a path creates cable conduits,
// hydraulic lines, or structural tubes that follow any 3D curve.
//
// extrude(profile, length_mm) is the simplest case: straight tube.
// For curved paths use sweep(profile, path_sdf).

// === Simple straight conduit using extrude ===
// Circle cross-section, 3 mm radius (6 mm OD), 200 mm long
let c1 = translate(extrude(circle_section(3.0), 200.0),  90.0,  15.0, 50.0);
let c2 = translate(extrude(circle_section(2.0), 200.0),  90.0,   0.0, 50.0);  // signal
let c3 = translate(extrude(circle_section(2.0), 200.0),  90.0, -15.0, 50.0);  // servo

// === Heat-set boss for conduit clamp ===
// Clamp attachment point at mid-run
// heat_set_boss(outer_r, height, insert_r, insert_depth)
let boss = heat_set_boss(6.0, 8.0, 2.5, 4.0);
let boss = translate(boss, 90.0, 0.0, 150.0);

// === Example of revolve for a swept elbow ===
// revolve(profile, axis_dir, sweep_deg)
let elbow_profile = translate(circle_section(3.0), 20.0, 0.0, 0.0);
let elbow = revolve(elbow_profile, "z", 90.0);
let elbow = translate(elbow, 90.0, 30.0, 50.0);

let conduits = union(union(c1, c2), c3);
union(union(conduits, boss), elbow)
"#;

static EX22_SCRIPT: &str = r#"
// Example 22 — Sweep: Carbon Rod Structural Members
// Structural tube members following the main and rear spar lines.
// We use chord_point references and cylinder tubes along the spar axis.

let half_wing = wing_with_airfoil("2412", 140.0, 100.0, 400.0, 3.0, 2.0, -1.5);

// === Reference points along main spar (25% chord) ===
let spar_root = chord_point(half_wing, 0.0,  0.25);  // root, quarter-chord
let spar_tip  = chord_point(half_wing, 1.0,  0.25);  // tip,  quarter-chord

// === Carbon tube: 8 mm OD, 6 mm ID ===
let tube_od = cylinder(4.0, 400.0);    // outer radius 4 mm
let tube_id = cylinder(3.0, 402.0);    // inner radius 3 mm (through)
let carbon_tube = subtract(tube_od, tube_id);

// Rotate tube to lie along Y-axis (span direction), place at spar root point
let main_spar = rotate(carbon_tube, 90.0, 0.0, 0.0);
let main_spar = translate_p(main_spar, spar_root);

// === Rear spar tube: 6 mm OD ===
let rear_root = chord_point(half_wing, 0.0, 0.70);
let rear_tube = subtract(cylinder(3.0, 400.0), cylinder(2.2, 402.0));
let rear_spar = rotate(rear_tube, 90.0, 0.0, 0.0);
let rear_spar = translate_p(rear_spar, rear_root);

// Combine spars with wing shell
let wing_skin = shell(half_wing, 1.0);
union(union(wing_skin, main_spar), rear_spar)
"#;

static EX23_SCRIPT: &str = r#"
// Example 23 — Mesh Import for Component Keepout
// Import an STL mesh (e.g. a commercial component) and use it as a
// keepout volume to ensure nothing else overlaps with it.
//
// USAGE:
//   1. Export your component's STL to a known path.
//   2. Replace the path string below with the actual file path.
//   3. mesh_keepout(mesh, margin_mm) inflates the mesh by margin_mm.
//
// The fallback cylinder is used when no mesh file is present,
// so this script always runs without an actual file.

// === Import mesh (replace path with your STL) ===
// let motor_mesh    = import_mesh("C:/models/sunnysky_x2212.stl");
// let motor_keepout = mesh_keepout(motor_mesh, 3.0);

// === Fallback: approximate Sunnysky X2212 with primitives ===
// Approx dimensions: 28 mm diameter, 35 mm long
let motor_body   = cylinder(14.0, 35.0);
let motor_bell   = cylinder(16.0, 8.0);
let motor_bell   = translate(motor_bell, 0.0, 0.0, 35.0);
let motor_approx = union(motor_body, motor_bell);

// Keepout: add 3 mm clearance margin around motor
let motor_ko = offset(motor_approx, 3.0);

// Place motor at nose
let motor_placed = translate(motor_approx, 0.0, 0.0, -20.0);
let ko_placed    = translate(motor_ko,     0.0, 0.0, -20.0);

// Fuselage nose region for context
let nose     = von_karman_nose(120.0, 200.0);
let fuse_cut = subtract(nose, ko_placed);

union(fuse_cut, motor_placed)
"#;

static EX24_SCRIPT: &str = r#"
// Example 24 — Split Body for Printing
// Large parts must be split to fit print beds.
// Alignment features (pin + socket) ensure accurate re-assembly.

// Full fuselage shell
let stations = [
    [0.00, circle_section(10.0)],
    [0.14, ellipse_section(90.0, 80.0)],
    [0.34, ellipse_section(100.0, 95.0)],
    [0.60, ellipse_section(100.0, 95.0)],
    [0.86, ellipse_section(60.0, 55.0)],
    [1.00, circle_section(12.0)],
];
let fuse = shell(fuselage(stations), 2.5);

// === Split at X=0 (longitudinal mid-plane, port/starboard halves) ===
// split_body(sdf, axis, position_mm) -> splits the solid at that plane
let halves = split_body(fuse, "x", 0.0);

// === Add alignment pins and sockets automatically ===
// add_alignment_features(sdf, axis, position, n_pins) adds matching
// pins to one half and sockets to the other
let aligned = add_alignment_features(fuse, "x", 0.0, 4);

// Individual pin/socket for manual placement reference:
let pin    = alignment_pin(2.0, 8.0);       // r=2 mm, h=8 mm
let socket = alignment_socket(2.0, 8.5);    // slight clearance on depth

let pin_demo    = translate(pin,    0.0, 110.0, 100.0);
let socket_demo = translate(socket, 0.0, 110.0, 120.0);

union(aligned, union(pin_demo, socket_demo))
"#;

static EX25_SCRIPT: &str = r#"
// Example 25 — Tolerance Compensation
// FDM printers are dimensionally inaccurate: holes print smaller,
// bosses print larger. tolerance_compensate corrects for this.
//
// Settings map keys:
//   xy_shrink    — XY plane shrinkage factor (e.g. 0.002 = 0.2%)
//   z_shrink     — Z-axis shrinkage factor
//   hole_offset  — add to hole radii (positive = larger holes)
//   boss_offset  — subtract from bosses (positive = smaller bosses)
//   min_wall     — warn if wall < this value (mm)

// === Base bracket ===
let bracket = box_(60.0, 40.0, 8.0);

// M3 clearance holes: 1.75 mm radius (3.5 mm dia)
let hole  = cylinder(1.75, 10.0);
let holes = union(
    translate(hole,  20.0,  12.0, 0.0),
    translate(hole, -20.0,  12.0, 0.0));
let holes = union(holes, union(
    translate(hole,  20.0, -12.0, 0.0),
    translate(hole, -20.0, -12.0, 0.0)));
let bracket = subtract(bracket, holes);

// Heat-set boss for M3 threaded insert
let boss = heat_set_boss(4.5, 6.0, 2.0, 5.5);
let bracket = union(bracket, translate(boss, 0.0, 0.0, 8.0));

// === Tolerance compensation ===
let settings = #{
    xy_shrink:   0.002,
    z_shrink:    0.001,
    hole_offset: 0.15,
    boss_offset: 0.10,
    min_wall:    1.2,
};
tolerance_compensate(bracket, settings)
"#;

static EX26_SCRIPT: &str = r#"
// Example 26 — Access Panels and Hatches
// Battery hatch on the belly, electronics access panel on the side.
// Panels sit flush with the outer mould line.

// Reference fuselage
let stations = [
    [0.00, circle_section(10.0)],
    [0.12, ellipse_section(85.0, 80.0)],
    [0.31, ellipse_section(100.0, 95.0)],
    [0.60, ellipse_section(100.0, 95.0)],
    [0.86, ellipse_section(65.0, 55.0)],
    [1.00, circle_section(12.0)],
];
let fuse = shell(fuselage(stations), 2.5);

// === Battery hatch ===
// battery_hatch(fuse, cx, cy, cz, width_mm, height_mm)
// Centred at (150, 0, -95): belly of forward fuselage, 90x50 mm opening
let fuse2 = battery_hatch(fuse, 150.0, 0.0, -95.0, 90.0, 50.0);

// === Electronics access panel ===
// access_panel(fuse, cx, cy, cz, width, height, face_axis)
// Side panel at (230, 95, 0): starboard, 60x40 mm
access_panel(fuse2, 230.0, 95.0, 0.0, 60.0, 40.0, "y")
"#;

static EX27_SCRIPT: &str = r#"
// Example 27 — Screw Holes and Heat-Set Inserts
// M3 bolt pattern and brass heat-set insert bosses for a mounting plate.

// === Base plate ===
let plate = box_(80.0, 60.0, 6.0);

// === Countersunk M3 holes in a bolt circle ===
// bolt_circle(n_holes, pcd_radius_mm, bolt_radius_mm, depth_mm)
// 4 holes on 30 mm PCD, M3 clearance: r = 1.75 mm
let m3_circle = bolt_circle(4, 30.0, 1.75, 8.0);
let plate = subtract(plate, m3_circle);

// === Countersink for flush-head screws ===
// countersink(shaft_radius, depth_mm, half_angle_deg)
let cs_pattern = bolt_circle(4, 30.0, 1.75, 3.0);  // same PCD
let plate = subtract(plate, cs_pattern);

// === Heat-set bosses for side-entry inserts ===
// heat_set_boss(outer_r, height, insert_r, insert_depth)
// M3 insert: 2.5 mm insert radius, 4 mm insert depth
let boss1 = translate(heat_set_boss(5.0, 8.0, 2.5, 4.0), -30.0, -18.0, 6.0);
let boss2 = translate(heat_set_boss(5.0, 8.0, 2.5, 4.0),  30.0, -18.0, 6.0);

// === Bolt grid for larger panel attachment ===
// bolt_square(spacing_mm, bolt_radius_mm, depth_mm)
let m2_grid = bolt_square(40.0, 1.1, 6.0);  // M2 clearance
let plate = subtract(plate, m2_grid);

union(union(plate, boss1), boss2)
"#;

static EX28_SCRIPT: &str = r#"
// Example 28 — Manufacturing Export Workflow
// Full prepare-for-print pipeline:
//   1. Build geometry   2. Split for bed size
//   3. Add alignment    4. Check wall thickness
//   5. Check overhangs

// === Fuselage ===
let fuse = shell(fuselage_parametric(600.0, 200.0, 0.6, 0.8), 2.5);

// === Split at mid-length (Z=300) for two print halves ===
// split_body returns an array [front_half, rear_half]
let halves  = split_body(fuse, "z", 300.0);

// === Alignment features: 4 pins at split plane ===
let aligned = add_alignment_features(fuse, "z", 300.0, 4);

// === Wall thickness probes ===
// wall_thickness_at(sdf, x, y, z, direction_string)
let t_side   = wall_thickness_at(fuse,  95.0,  0.0, 200.0, "x");  // side wall
let t_belly  = wall_thickness_at(fuse,   0.0, -95.0, 300.0, "y");  // belly
let t_dorsal = wall_thickness_at(fuse,   0.0,  95.0, 300.0, "y");  // dorsal

// === Overhang analysis ===
// print_overhang_angle(sdf, build_direction_z: bool)
// true  = printing upright; false = printing on its side
let oh_upright = print_overhang_angle(fuse, true);
let oh_on_side = print_overhang_angle(fuse, false);

// Console output: t_side, t_belly, t_dorsal, oh_upright, oh_on_side

aligned
"#;

static EX29_SCRIPT: &str = r#"
// Example 29 — Lifting Line Theory
// Spanwise lift distribution and drag polar sweep using Prandtl's
// lifting line method for the reference wing.

// === Reference wing ===
let half_wing = wing_with_airfoil("2412", 140.0, 100.0, 400.0, 3.0, 2.0, -1.5);

// === Flight condition: sea-level, cruise ===
// flight_condition_sl(velocity_ms, aoa_deg)
let fc_cruise = flight_condition_sl(18.0, 3.0);

// === Single operating point ===
let ll_result = run_lifting_line(half_wing, fc_cruise);
// ll_result.cl, ll_result.cd_induced, ll_result.span_efficiency

// === Polar sweep: alpha from -4 to 14 degrees ===
let polar = run_lifting_line_polar(half_wing, fc_cruise, -4.0, 14.0, 1.0);
// polar is an array of {alpha, cl, cd} maps

// === Airfoil section polar for reference ===
let aero       = get_polar("2412");
let cl_cruise  = cl_at(aero, 3.0);
let cd_section = cd_at(aero, 3.0);
let cl_max_val = cl_max(aero);
let stall_a    = alpha_stall(aero);

// Dynamic conditions
let q  = dynamic_pressure(fc_cruise);
let re = reynolds(fc_cruise, 140.0);  // Reynolds at root chord

// Results printed to console: ll_result
ll_result
"#;

static EX30_SCRIPT: &str = r#"
// Example 30 — Static Stability and CG Analysis
// Neutral point, static margin, and trim analysis for the reference aircraft.
// CG ~220 mm from nose, AUW 10.79 N (1.1 kg).

// === Reference surfaces ===
let half_wing  = wing_with_airfoil("2412", 140.0, 100.0, 400.0, 3.0, 2.0, -1.5);
let half_htail = wing_with_airfoil("0009",  70.0,  50.0, 140.0, 5.0, 0.0,  0.0);
let htail      = translate(mirror_y(half_htail), 0.0, 0.0, 520.0);
let fuse       = fuselage_parametric(600.0, 200.0, 0.6, 0.8);

let fc = flight_condition_sl(18.0, 3.0);

// === Neutral point (aerodynamic centre of whole aircraft) ===
// Returns x_mm from nose
let np = neutral_point(half_wing, htail, fuse, fc);

// === Static margin ===
// static_margin(wing, htail, fuse, fc, cg_x_mm) -> fraction of MAC
// Positive = stable; typically want 5-15% MAC for a UAV
let sm = static_margin(half_wing, htail, fuse, fc, 220.0);

// === Allowable CG range ===
let cg_range = required_cg_range(half_wing, htail, fuse, fc);
// cg_range.forward_limit_mm, cg_range.aft_limit_mm

// === Trim analysis ===
// trim_analysis(wing, htail, fuse, fc, weight_n) -> elevator deflection
let trim = trim_analysis(half_wing, htail, fuse, fc, 10.79);
// trim.elev_deflection_deg, trim.aoa_trim_deg

// Results printed to console: np, sm, cg_range, trim
np
"#;

static EX31_SCRIPT: &str = r#"
// Example 31 — Drag Polar and L/D Optimisation
// Full component drag buildup, polar curve, and best-glide airspeed.

let half_wing  = wing_with_airfoil("2412", 140.0, 100.0, 400.0, 3.0, 2.0, -1.5);
let half_htail = wing_with_airfoil("0009",  70.0,  50.0, 140.0, 5.0, 0.0,  0.0);
let htail      = translate(mirror_y(half_htail), 0.0, 0.0, 520.0);
let vtail      = translate(rotate(
    wing_with_airfoil("0009", 80.0, 55.0, 120.0, 8.0, 0.0, 0.0),
    90.0, 0.0, 0.0), 0.0, 0.0, 510.0);
let fuse  = fuselage_parametric(600.0, 200.0, 0.6, 0.8);
let fc    = flight_condition_sl(18.0, 3.0);

// === Drag polar (weighted by lift requirement) ===
let polar = drag_polar_weighted(half_wing, fuse, htail, vtail, fc, 10.79);
// polar.cd0, polar.cdi, polar.cd_total, polar.cl_cruise

// === L/D max ===
let ld = ld_max(half_wing, fuse, htail, vtail, fc);
// ld.ld_max, ld.cl_opt, ld.cd_opt

// === Best glide speed ===
let v_glide = best_glide_speed(half_wing, fuse, htail, vtail, fc, 10.79);

// Results printed to console: polar, ld, v_glide
polar
"#;

static EX32_SCRIPT: &str = r#"
// Example 32 — Propulsion System Analysis
// Motor selection, prop matching, power and thrust curves.
// Reference: Sunnysky X2212, APC 9x4.7, 3S 2200 mAh.

// === Motor and prop ===
let m  = motor("Sunnysky X2212");   // looks up motor database
let p  = prop("APC 9x4.7");

// === Propulsion setup ===
// propulsion_setup(motor, prop, cells, capacity_mah)
let setup = propulsion_setup(m, p, 3, 2200);

let fc = flight_condition_sl(18.0, 0.0);

// === Full analysis at cruise ===
// propulsion_analysis(setup, fc, weight_n)
let analysis = propulsion_analysis(setup, fc, 10.79);
// analysis.static_thrust_n, analysis.cruise_thrust_n,
// analysis.cruise_power_w, analysis.efficiency, analysis.amps

// === Thrust at specific airspeeds ===
let t_cruise = propulsion_thrust_at(setup, 18.0, fc);
let t_climb  = propulsion_thrust_at(setup,  5.0, fc);

// === Motor/prop recommendation (if selection is undecided) ===
// recommend_motor_prop(required_thrust_n, cruise_ms, max_weight_g)
let rec = recommend_motor_prop(12.0, 18.0, 1200);
// rec.motor, rec.prop, rec.cells

// Results printed to console: analysis, t_cruise, rec
analysis
"#;

static EX33_SCRIPT: &str = r#"
// Example 33 — Range and Endurance
// Maximum range, endurance, rate of climb, and glide performance.

let half_wing  = wing_with_airfoil("2412", 140.0, 100.0, 400.0, 3.0, 2.0, -1.5);
let half_htail = wing_with_airfoil("0009",  70.0,  50.0, 140.0, 5.0, 0.0,  0.0);
let htail      = translate(mirror_y(half_htail), 0.0, 0.0, 520.0);
let vtail      = translate(rotate(
    wing_with_airfoil("0009", 80.0, 55.0, 120.0, 8.0, 0.0, 0.0),
    90.0, 0.0, 0.0), 0.0, 0.0, 510.0);
let fuse  = fuselage_parametric(600.0, 200.0, 0.6, 0.8);

let setup = propulsion_setup(motor("Sunnysky X2212"), prop("APC 9x4.7"), 3, 2200);
let fc    = flight_condition_sl(18.0, 3.0);

// === Range and endurance ===
// range_endurance(setup, wing, fuse, htail, vtail, fc, weight_n)
let re = range_endurance(setup, half_wing, fuse, htail, vtail, fc, 10.79);
// re.range_km, re.endurance_min, re.best_range_speed_ms, re.best_endurance_speed_ms

// === Rate of climb ===
let roc = rate_of_climb(setup, half_wing, fuse, htail, vtail, fc, 10.79);
// roc.roc_ms, roc.roc_fpm, roc.best_climb_speed_ms

// === Glide (engine off) ===
let glide = glide_performance(half_wing, fuse, htail, vtail, fc, 10.79);
// glide.ld_max, glide.min_sink_ms, glide.best_glide_speed_ms

// Results printed to console: re, roc, glide
re
"#;

static EX34_SCRIPT: &str = r#"
// Example 34 — Print Analysis
// Check overhang angles and wall thickness for FDM printability.
// FDM rule of thumb: overhangs > 45 deg need supports;
// walls < 1.2 mm may not print reliably.

// === Test bracket ===
let base   = box_(80.0, 50.0, 5.0);
let wall   = translate(box_(5.0, 50.0, 30.0), 37.5, 0.0, 17.5);
let gusset = translate(rotate(box_(25.0, 50.0, 5.0), 0.0, 45.0, 0.0), 20.0, 0.0, 12.5);
let bracket = union(union(base, wall), gusset);

// === Wall thickness probes ===
// wall_thickness_at(sdf, x, y, z, direction_string)
let t_base   = wall_thickness_at(bracket,  0.0,  0.0,  5.0, "z");  // base top face
let t_wall   = wall_thickness_at(bracket, 37.5,  0.0, 20.0, "x");  // vertical wall
let t_gusset = wall_thickness_at(bracket, 22.0,  0.0, 14.0, "y");  // gusset

// === Overhang analysis ===
// true  = printing flat (base on bed) -> gusset overhangs
// false = printing on its side        -> likely no critical overhangs
let oh_flat = print_overhang_angle(bracket, true);
let oh_side = print_overhang_angle(bracket, false);

// INTERPRETATION:
//   t_base, t_wall, t_gusset should all be ~5 mm
//   oh_flat.max_angle_deg > 45 -> gusset needs support when flat
//   oh_side.max_angle_deg < 45 -> prefer printing on its side

bracket
"#;

static EX35_SCRIPT: &str = r#"
// Example 35 — FEA Setup with Motor and Gravity Loads
// Define boundary conditions and applied loads for structural FEA.
// The FEA solver reads these annotations from the exported mesh.

// === Wing main spar — carbon tube ===
let spar_od  = cylinder(4.0, 400.0);
let spar_id  = cylinder(3.0, 402.0);
let spar     = rotate(subtract(spar_od, spar_id), 90.0, 0.0, 0.0);
let spar     = translate(spar, 0.0, 0.0, 35.0);   // quarter-chord offset

// === Fixed face: wing root (Y=0 plane) ===
// fea_fixed_face(sdf, axis_string, position_mm)
let spar_fixed = fea_fixed_face(spar, "y", 0.0);

// === Gravity load (distributed body force, 9.81 m/s^2 downward) ===
let spar_grav = fea_gravity(spar_fixed);

// === Point load: 3g manoeuvre tip load ===
// AUW 1.1 kg, 3g, half load per wing: ~16 N upward at tip
// fea_load_point(x, y, z, fx_n, fy_n, fz_n)
let spar_loaded = fea_load_point(0.0, 400.0, 35.0, 0.0, 0.0, 16.0);

// === Pressure load: aerodynamic lift on upper skin ===
// fea_pressure(sdf, face_axis, face_position_mm, pressure_pa)
// Approximate upper surface lift pressure ~500 Pa at cruise
let wing_skin  = wing_with_airfoil("2412", 140.0, 100.0, 400.0, 3.0, 2.0, -1.5);
let wing_press = fea_pressure(wing_skin, "z", 5.0, 500.0);

// Return spar geometry for mesh export
spar
"#;

static EX36_SCRIPT: &str = r#"
// Example 36 — Stress-Field Driven Lattice
// Lattice cell size is graded by distance from a known stress
// concentration, producing denser (stronger) lattice where needed.

// === Wing spar with smooth fillet at root ===
let spar_base   = cylinder(4.0, 400.0);
let root_fillet = sphere(8.0);
let spar        = rotate(smooth_union(spar_base, root_fillet, 5.0), 90.0, 0.0, 0.0);

// === Radial stress field centred at root ===
// Near root (small radius) = high stress -> dense lattice
let stress_field = radial_field(0.0, 0.0, 0.0);

// Two gyroid densities: fine near root, coarse toward tip
let fine_gf   = gyroid_field(6.0);   // fine lattice — high stress zones
let coarse_gf = gyroid_field(14.0);  // coarse lattice — low stress zones

// blend_by_field: stress_field drives the mix
let graded_gf = blend_by_field(fine_gf, coarse_gf, stress_field);

// Spar outer shell (1 mm skin) for print surface
let spar_skin = shell(spar, 1.0);

// Interior volume for lattice
let spar_interior = subtract(spar, offset(spar, -1.0));

// Apply graded offset — positive field near root expands struts there
let lattice = offset_by_field(spar_interior, stress_field, -1.5);

union(spar_skin, lattice)
"#;

static EX37_SCRIPT: &str = r#"
// Example 37 — CG Sensitivity Analysis
// Shows how each component's mass and position affect overall CG
// and its relationship to the neutral point (static margin).

// === Reference aircraft components ===
let battery = component_named("battery",          box_(115.0, 35.0, 25.0), 3.0, 185.0);
let fc       = component_named("flight_controller", box_(36.0, 36.0, 8.0),  5.0,  38.0);
let esc      = component_named("esc",               box_(60.0, 25.0, 12.0), 3.0,  35.0);
let motor_c  = component_named("motor",             cylinder(14.0, 35.0),   3.0,  60.0);
let rx       = component_named("receiver",          box_(40.0, 14.0, 8.0),  2.0,  15.0);

// === Place components (origin at nose) ===
let batt_p  = place(battery,  50.0,  0.0, -5.0);
let fc_p    = place(fc,      185.0,  0.0, 10.0);
let esc_p   = place(esc,     165.0,  0.0, 25.0);
let motor_p = place(motor_c, -20.0,  0.0,  0.0);
let rx_p    = place(rx,      300.0,  0.0,  8.0);

let comps = [batt_p, fc_p, esc_p, motor_p, rx_p];

// Neutral point ~235 mm from nose; MAC = 122 mm (interpolated)
let np_x = 235.0;
let mac  = 122.0;

// cg_sensitivity(components_array, np_x_mm, mac_mm)
let sens = cg_sensitivity(comps, np_x, mac);
// sens.cg_x_mm            — current CG position
// sens.static_margin_frac — current SM as fraction of MAC
// sens.sensitivity        — [{name, dcg_per_100g_shift}]

// Results printed to console: sens
sens
"#;

static EX38_SCRIPT: &str = r#"
// Example 38 — Assembly Interference Check
// Verify all components fit within the fuselage and do not overlap.

// === Fuselage ===
let fuse = fuselage_parametric(600.0, 200.0, 0.6, 0.8);

// === Components ===
let batt = component_named("battery",          box_(115.0, 35.0, 25.0), 3.0, 185.0);
let fc   = component_named("flight_controller", box_(36.0, 36.0, 8.0),  5.0,  38.0);
let esc  = component_named("esc",               box_(60.0, 25.0, 12.0), 3.0,  35.0);
let cam  = component_named("camera",            cylinder(18.0, 24.0),   4.0,  28.0);
let rx   = component_named("receiver",          box_(40.0, 14.0, 8.0),  2.0,  15.0);

// === Place components ===
let batt_p = place(batt,  50.0,  0.0, -8.0);
let fc_p   = place(fc,   185.0,  0.0, 10.0);
let esc_p  = place(esc,  165.0,  0.0, 28.0);   // stacked above FC — may interfere
let cam_p  = place(cam,   30.0,  0.0,  0.0);   // nose camera
let rx_p   = place(rx,   300.0,  0.0,  8.0);

// === Interference check (no parent — components vs each other) ===
let names    = ["battery", "flight_controller", "esc", "camera", "receiver"];
let keepouts = [keepout(batt_p), keepout(fc_p), keepout(esc_p),
                keepout(cam_p),  keepout(rx_p)];
let result   = interference_check_no_parent(names, keepouts);
// result.has_critical_interference -> bool
// result.pairs                     -> [{name_a, name_b, overlap_mm3}]

// Visualise all component volumes
union(union(union(union(
    geometry(batt_p),
    geometry(fc_p)),
    geometry(esc_p)),
    geometry(cam_p)),
    geometry(rx_p))
"#;

static EX39_SCRIPT: &str = r#"
// Example 39 — Reference Points and Geometry Queries
// Query surfaces, bounding boxes, and named reference points for
// precise downstream placement and measurement.

let half_wing = wing_with_airfoil("2412", 140.0, 100.0, 400.0, 3.0, 2.0, -1.5);

// === Leading and trailing edge queries ===
let le_root = leading_edge(half_wing,  0.0);   // root LE point
let le_tip  = leading_edge(half_wing,  1.0);   // tip  LE point
let te_root = trailing_edge(half_wing, 0.0);
let te_tip  = trailing_edge(half_wing, 1.0);

// chord_point(wing, span_frac, chord_frac) -> point on chord line
let quarter_root = chord_point(half_wing, 0.0, 0.25);   // root quarter-chord
let quarter_mid  = chord_point(half_wing, 0.5, 0.25);   // mid-span quarter-chord
let tip_pt       = wing_tip(half_wing);
let root_pt      = wing_root(half_wing);

// === Named reference points (persist across scripts) ===
ref_point("wing_le_root",      le_root);
ref_point("wing_quarter_root", quarter_root);

// === Bounding box ===
let bb_min  = bbox_min(half_wing);
let bb_max  = bbox_max(half_wing);
let bb_size = bbox_size(half_wing);    // [width, depth, height]
let bb_ctr  = bbox_center(half_wing);

// === Surface point query ===
// surface_point(sdf, ox, oy, oz, dx, dy, dz) shoots ray, returns hit point
let upper_surf = surface_point(half_wing, 0.0, 0.0, 60.0,  0.0, 0.0,  1.0);
let lower_surf = surface_point(half_wing, 0.0, 0.0, 60.0,  0.0, 0.0, -1.0);

// === Cross-section centre at mid-span ===
let mid_ctr = cross_section_center(half_wing, "y", 200.0);

half_wing
"#;

static EX40_SCRIPT: &str = r#"
// Example 40 — Complete Fixed-Wing Airframe
// Full reference aircraft: fuselage, wing, tails, propulsion,
// and a summary of key aero and manufacturing checks.
//
// REFERENCE:  wingspan 800 mm, NACA 2412, root 140, tip 100
//             fuselage 200 mm dia x 600 mm long
//             AUW 10.79 N (1.1 kg), cruise 18 m/s, CG ~220 mm from nose

// === Fuselage ===
let nose  = von_karman_nose(120.0, 200.0);
let body  = translate(cylinder(100.0, 360.0), 0.0, 0.0, 300.0);
let tail  = translate(tail_cone(120.0, 200.0, 24.0), 0.0, 0.0, 480.0);
let fuse  = shell(union(union(nose, body), tail), 2.5);

// === Wing ===
let half_wing = wing_with_airfoil("2412", 140.0, 100.0, 400.0, 3.0, 2.0, -1.5);
let full_wing = translate(mirror_y(half_wing), 0.0, 0.0, 200.0);

// === Tail surfaces ===
let half_htail = wing_with_airfoil("0009", 70.0, 50.0, 140.0, 5.0, 0.0, 0.0);
let htail      = translate(mirror_y(half_htail), 0.0, 0.0, 520.0);
let vtail      = translate(
    rotate(wing_with_airfoil("0009", 80.0, 55.0, 120.0, 8.0, 0.0, 0.0), 90.0, 0.0, 0.0),
    0.0, 0.0, 510.0);

// === Propulsion analysis ===
let setup     = propulsion_setup(motor("Sunnysky X2212"), prop("APC 9x4.7"), 3, 2200);
let fc        = flight_condition_sl(18.0, 3.0);
let prop_data = propulsion_analysis(setup, fc, 10.79);

// === Stability ===
let np = neutral_point(half_wing, htail, fuse, fc);
let sm = static_margin(half_wing, htail, fuse, fc, 220.0);

// === Drag and range ===
let polar = drag_polar_weighted(half_wing, fuse, htail, vtail, fc, 10.79);
let re    = range_endurance(setup, half_wing, fuse, htail, vtail, fc, 10.79);

// === Manufacturing check ===
let t_skin = wall_thickness_at(fuse, 95.0, 0.0, 300.0, "x");

// Console output: prop_data, np, sm, polar, re, t_skin

// === Full airframe assembly ===
union(union(union(union(fuse, full_wing), htail), vtail),
    translate(cylinder(14.0, 35.0), 0.0, 0.0, -20.0))
"#;

// ---------------------------------------------------------------------------
// Examples array
// ---------------------------------------------------------------------------

pub static EXAMPLES: &[ExampleScript] = &[
    ExampleScript {
        id:                "primitives_booleans",
        title:             "Basic Primitives and Booleans",
        category:          "Getting Started",
        difficulty:        Difficulty::Beginner,
        description:       "Sphere, box, cylinder, torus, and boolean operations",
        script:            EX01_SCRIPT,
        related_functions: &["sphere", "box_", "cylinder", "union", "subtract", "intersect", "smooth_union"],
        tags:              &["primitives", "boolean", "sphere", "cylinder", "union"],
    },
    ExampleScript {
        id:                "transforms_patterns",
        title:             "Transforms and Patterns",
        category:          "Getting Started",
        difficulty:        Difficulty::Beginner,
        description:       "Translate, rotate, scale, arrays, and mirror operations",
        script:            EX02_SCRIPT,
        related_functions: &["translate", "rotate", "scale", "linear_array", "polar_array", "mirror_y"],
        tags:              &["transform", "pattern", "array", "mirror", "rotate"],
    },
    ExampleScript {
        id:                "fields_geometry",
        title:             "Fields and Field-Driven Geometry",
        category:          "Getting Started",
        difficulty:        Difficulty::Intermediate,
        description:       "Scalar fields to modulate geometry thickness and blending",
        script:            EX03_SCRIPT,
        related_functions: &["gyroid_field", "blend_by_field", "offset_by_field", "radial_field", "gradient_field"],
        tags:              &["field", "gyroid", "blend", "offset", "modulate"],
    },
    ExampleScript {
        id:                "station_fuselage",
        title:             "Station-Based Fuselage",
        category:          "Fuselage Design",
        difficulty:        Difficulty::Beginner,
        description:       "Elliptical cross-section fuselage built from station list",
        script:            EX04_SCRIPT,
        related_functions: &["fuselage", "circle_section", "ellipse_section", "shell"],
        tags:              &["fuselage", "station", "cross-section", "ellipse", "airframe"],
    },
    ExampleScript {
        id:                "spline_fuselage",
        title:             "Fuselage with Spline Cross-Sections",
        category:          "Fuselage Design",
        difficulty:        Difficulty::Intermediate,
        description:       "Fuselage using named spline profiles from the profile editor",
        script:            EX05_SCRIPT,
        related_functions: &["spline_fuselage", "fuselage", "fuselage_parametric", "circle_section", "ellipse_section"],
        tags:              &["fuselage", "spline", "profile", "parametric", "cross-section"],
    },
    ExampleScript {
        id:                "spine_fuselage",
        title:             "Longitudinal Spine Constraints",
        category:          "Fuselage Design",
        difficulty:        Difficulty::Advanced,
        description:       "Hard-chine hull using keel/deck/chine spine curves",
        script:            EX06_SCRIPT,
        related_functions: &["fuselage", "rect_section", "circle_section", "shell", "offset"],
        tags:              &["fuselage", "spine", "hard-chine", "hull", "composite"],
    },
    ExampleScript {
        id:                "nose_tail",
        title:             "Von Karman Nose and Haack Tail",
        category:          "Fuselage Design",
        difficulty:        Difficulty::Beginner,
        description:       "Aerodynamically optimized nose and tail cones",
        script:            EX07_SCRIPT,
        related_functions: &["von_karman_nose", "haack_nose", "tail_cone", "cylinder", "union"],
        tags:              &["nose", "tail", "aerodynamic", "von-karman", "drag"],
    },
    ExampleScript {
        id:                "wing_basic",
        title:             "Wing with Airfoil, Taper and Dihedral",
        category:          "Wing Design",
        difficulty:        Difficulty::Beginner,
        description:       "Reference aircraft half-wing with NACA 2412 profile",
        script:            EX08_SCRIPT,
        related_functions: &["wing_with_airfoil", "mirror_y", "translate"],
        tags:              &["wing", "airfoil", "NACA", "taper", "dihedral"],
    },
    ExampleScript {
        id:                "wing_sections",
        title:             "Wing from Multiple Sections",
        category:          "Wing Design",
        difficulty:        Difficulty::Intermediate,
        description:       "Wing built from explicit cross-section stations at root, mid, and tip",
        script:            EX09_SCRIPT,
        related_functions: &["wing_from_sections", "mirror_y"],
        tags:              &["wing", "sections", "loft", "airfoil", "taper"],
    },
    ExampleScript {
        id:                "elevon_wing",
        title:             "Elevon Configuration",
        category:          "Wing Design",
        difficulty:        Difficulty::Intermediate,
        description:       "Flying wing with elevon control surfaces",
        script:            EX10_SCRIPT,
        related_functions: &["wing_with_airfoil", "elevon", "mirror_y", "union"],
        tags:              &["flying-wing", "elevon", "control-surface", "delta", "tailless"],
    },
    ExampleScript {
        id:                "tail_assembly",
        title:             "Horizontal and Vertical Tail Assembly",
        category:          "Wing Design",
        difficulty:        Difficulty::Beginner,
        description:       "H-stab and V-stab sized for reference aircraft",
        script:            EX11_SCRIPT,
        related_functions: &["wing_with_airfoil", "mirror_y", "rotate", "translate", "union"],
        tags:              &["tail", "stabiliser", "h-stab", "v-stab", "assembly"],
    },
    ExampleScript {
        id:                "aileron_flap",
        title:             "Aileron and Split Flap",
        category:          "Control Surfaces",
        difficulty:        Difficulty::Intermediate,
        description:       "Outboard aileron and inboard flap on reference wing",
        script:            EX12_SCRIPT,
        related_functions: &["wing_with_airfoil", "aileron", "subtract", "mirror_y"],
        tags:              &["aileron", "flap", "control-surface", "wing", "roll"],
    },
    ExampleScript {
        id:                "elevator_rudder",
        title:             "Elevator and Rudder",
        category:          "Control Surfaces",
        difficulty:        Difficulty::Beginner,
        description:       "Tail control surface geometry for conventional layout",
        script:            EX13_SCRIPT,
        related_functions: &["elevator", "rudder", "wing_with_airfoil", "mirror_y", "union"],
        tags:              &["elevator", "rudder", "tail", "control-surface", "conventional"],
    },
    ExampleScript {
        id:                "internal_components",
        title:             "Internal Component Placement",
        category:          "Internal Structure",
        difficulty:        Difficulty::Intermediate,
        description:       "Battery, FC, ESC, and servos with keepout volumes",
        script:            EX14_SCRIPT,
        related_functions: &["component_named", "place", "keepout", "geometry", "interference_check_no_parent"],
        tags:              &["components", "battery", "flight-controller", "keepout", "placement"],
    },
    ExampleScript {
        id:                "ribs_spars",
        title:             "Ribs and Spars",
        category:          "Internal Structure",
        difficulty:        Difficulty::Intermediate,
        description:       "Auto-generated structural elements from wing geometry",
        script:            EX15_SCRIPT,
        related_functions: &["rib_slab", "spar_cylinder", "shell", "wing_with_airfoil", "union"],
        tags:              &["rib", "spar", "structure", "wing", "internal"],
    },
    ExampleScript {
        id:                "bulkheads",
        title:             "Bulkheads with Lightening Holes",
        category:          "Internal Structure",
        difficulty:        Difficulty::Intermediate,
        description:       "Fuselage bulkheads at structural stations with weight reduction",
        script:            EX16_SCRIPT,
        related_functions: &["bulkhead_at_station", "lightening_hole_pattern", "shell", "fuselage", "union"],
        tags:              &["bulkhead", "lightening", "fuselage", "structure", "weight"],
    },
    ExampleScript {
        id:                "composite_wing",
        title:             "Composite Wing Sandwich",
        category:          "Materials and Lattice",
        difficulty:        Difficulty::Intermediate,
        description:       "Wing skin as carbon-foam-carbon sandwich layup",
        script:            EX17_SCRIPT,
        related_functions: &["layup_config", "add_layer", "foam_core", "sandwich_panel", "shell"],
        tags:              &["composite", "carbon", "sandwich", "layup", "foam-core"],
    },
    ExampleScript {
        id:                "gyroid_lattice",
        title:             "Conformal Gyroid Lattice Infill",
        category:          "Materials and Lattice",
        difficulty:        Difficulty::Advanced,
        description:       "Gyroid lattice confined to fuselage interior volume",
        script:            EX18_SCRIPT,
        related_functions: &["conformal_gyroid", "shell", "fuselage", "union", "gyroid_lattice"],
        tags:              &["gyroid", "lattice", "infill", "conformal", "fuselage"],
    },
    ExampleScript {
        id:                "naca_inlet",
        title:             "NACA Flush Inlet",
        category:          "Inlets and Ducts",
        difficulty:        Difficulty::Intermediate,
        description:       "Flush inlet cutout on fuselage upper surface",
        script:            EX19_SCRIPT,
        related_functions: &["naca_flush_inlet", "fuselage", "subtract", "shell", "translate"],
        tags:              &["inlet", "NACA", "flush", "duct", "fuselage"],
    },
    ExampleScript {
        id:                "edf_inlet",
        title:             "EDF Buried Inlet with S-Duct",
        category:          "Inlets and Ducts",
        difficulty:        Difficulty::Advanced,
        description:       "Buried inlet with S-shaped transition duct",
        script:            EX20_SCRIPT,
        related_functions: &["buried_inlet", "s_duct", "fuselage_parametric", "subtract", "shell"],
        tags:              &["EDF", "buried-inlet", "s-duct", "propulsion", "intake"],
    },
    ExampleScript {
        id:                "sweep_cable",
        title:             "Sweep — Cable Routing",
        category:          "Fabrication",
        difficulty:        Difficulty::Beginner,
        description:       "Sweep a circle cross-section along a path for cable conduits",
        script:            EX21_SCRIPT,
        related_functions: &["extrude", "revolve", "circle_section", "heat_set_boss", "translate"],
        tags:              &["sweep", "extrude", "cable", "conduit", "routing"],
    },
    ExampleScript {
        id:                "sweep_carbon_rod",
        title:             "Sweep — Carbon Rod Structure",
        category:          "Fabrication",
        difficulty:        Difficulty::Intermediate,
        description:       "Structural tube members swept along wing spar paths",
        script:            EX22_SCRIPT,
        related_functions: &["chord_point", "cylinder", "subtract", "rotate", "translate_p"],
        tags:              &["carbon", "spar", "tube", "structure", "wing"],
    },
    ExampleScript {
        id:                "mesh_import",
        title:             "Mesh Import for Component Keepout",
        category:          "Fabrication",
        difficulty:        Difficulty::Beginner,
        description:       "Import an STL mesh file as a collision keepout volume",
        script:            EX23_SCRIPT,
        related_functions: &["import_mesh", "mesh_keepout", "offset", "subtract", "union"],
        tags:              &["mesh", "import", "STL", "keepout", "clearance"],
    },
    ExampleScript {
        id:                "split_body",
        title:             "Split Body for Printing",
        category:          "Fabrication",
        difficulty:        Difficulty::Beginner,
        description:       "Split fuselage at midplane with alignment features",
        script:            EX24_SCRIPT,
        related_functions: &["split_body", "add_alignment_features", "alignment_pin", "alignment_socket", "shell"],
        tags:              &["split", "print", "alignment", "pin", "fabrication"],
    },
    ExampleScript {
        id:                "tolerance_comp",
        title:             "Tolerance Compensation",
        category:          "Fabrication",
        difficulty:        Difficulty::Intermediate,
        description:       "Print-ready geometry with FDM dimensional compensation",
        script:            EX25_SCRIPT,
        related_functions: &["tolerance_compensate", "heat_set_boss", "bolt_circle", "cylinder", "subtract"],
        tags:              &["tolerance", "FDM", "compensation", "print", "dimensional"],
    },
    ExampleScript {
        id:                "access_panels",
        title:             "Access Panels and Hatches",
        category:          "Fabrication",
        difficulty:        Difficulty::Beginner,
        description:       "Battery hatch and electronics access panel in fuselage",
        script:            EX26_SCRIPT,
        related_functions: &["battery_hatch", "access_panel", "fuselage", "shell"],
        tags:              &["access", "hatch", "panel", "battery", "fuselage"],
    },
    ExampleScript {
        id:                "fasteners",
        title:             "Screw Holes and Heat Set Inserts",
        category:          "Fabrication",
        difficulty:        Difficulty::Beginner,
        description:       "M3 clearance holes and brass heat-set insert bosses",
        script:            EX27_SCRIPT,
        related_functions: &["bolt_circle", "bolt_square", "heat_set_boss", "countersink", "subtract"],
        tags:              &["fastener", "screw", "heat-set", "insert", "M3"],
    },
    ExampleScript {
        id:                "mfg_export",
        title:             "Manufacturing Export Workflow",
        category:          "Fabrication",
        difficulty:        Difficulty::Beginner,
        description:       "Split, align, check wall thickness, and export for production",
        script:            EX28_SCRIPT,
        related_functions: &["split_body", "add_alignment_features", "wall_thickness_at", "print_overhang_angle", "shell"],
        tags:              &["manufacturing", "export", "wall-thickness", "overhang", "split"],
    },
    ExampleScript {
        id:                "lifting_line",
        title:             "Lifting Line Theory",
        category:          "Analysis",
        difficulty:        Difficulty::Intermediate,
        description:       "Spanwise lift distribution on the reference wing",
        script:            EX29_SCRIPT,
        related_functions: &["run_lifting_line", "run_lifting_line_polar", "flight_condition_sl", "get_polar", "dynamic_pressure"],
        tags:              &["lifting-line", "lift", "distribution", "polar", "aerodynamics"],
    },
    ExampleScript {
        id:                "stability_analysis",
        title:             "Static Stability and CG Analysis",
        category:          "Analysis",
        difficulty:        Difficulty::Intermediate,
        description:       "Neutral point, static margin, and CG envelope for reference aircraft",
        script:            EX30_SCRIPT,
        related_functions: &["neutral_point", "static_margin", "required_cg_range", "trim_analysis", "flight_condition_sl"],
        tags:              &["stability", "CG", "neutral-point", "static-margin", "trim"],
    },
    ExampleScript {
        id:                "drag_polar",
        title:             "Drag Polar and L/D Optimisation",
        category:          "Analysis",
        difficulty:        Difficulty::Intermediate,
        description:       "Component drag buildup, polar curve, and best-glide airspeed",
        script:            EX31_SCRIPT,
        related_functions: &["drag_polar_weighted", "ld_max", "best_glide_speed", "flight_condition_sl"],
        tags:              &["drag", "polar", "lift-to-drag", "glide", "efficiency"],
    },
    ExampleScript {
        id:                "propulsion_analysis",
        title:             "Propulsion System Analysis",
        category:          "Analysis",
        difficulty:        Difficulty::Intermediate,
        description:       "Motor, prop, and battery analysis with thrust and power curves",
        script:            EX32_SCRIPT,
        related_functions: &["motor", "prop", "propulsion_setup", "propulsion_analysis", "propulsion_thrust_at", "recommend_motor_prop"],
        tags:              &["motor", "propulsion", "thrust", "power", "battery"],
    },
    ExampleScript {
        id:                "range_endurance",
        title:             "Range and Endurance",
        category:          "Analysis",
        difficulty:        Difficulty::Intermediate,
        description:       "Maximum range and endurance from battery energy and drag",
        script:            EX33_SCRIPT,
        related_functions: &["range_endurance", "rate_of_climb", "glide_performance", "propulsion_setup"],
        tags:              &["range", "endurance", "climb", "glide", "battery"],
    },
    ExampleScript {
        id:                "print_analysis",
        title:             "Print Analysis",
        category:          "Analysis",
        difficulty:        Difficulty::Beginner,
        description:       "Check overhang angles and minimum wall thickness for FDM printing",
        script:            EX34_SCRIPT,
        related_functions: &["wall_thickness_at", "print_overhang_angle"],
        tags:              &["print", "overhang", "wall-thickness", "FDM", "analysis"],
    },
    ExampleScript {
        id:                "fea_setup",
        title:             "FEA Setup with Motor and Gravity Loads",
        category:          "Analysis",
        difficulty:        Difficulty::Advanced,
        description:       "Define boundary conditions, thrust load, and gravity for structural FEA",
        script:            EX35_SCRIPT,
        related_functions: &["fea_fixed_face", "fea_load_point", "fea_gravity", "fea_pressure", "shell"],
        tags:              &["FEA", "structural", "boundary-condition", "load", "stress"],
    },
    ExampleScript {
        id:                "stress_lattice",
        title:             "Stress-Field Driven Lattice",
        category:          "Analysis",
        difficulty:        Difficulty::Advanced,
        description:       "Lattice density graded by stress concentration location",
        script:            EX36_SCRIPT,
        related_functions: &["radial_field", "gyroid_field", "blend_by_field", "offset_by_field", "shell"],
        tags:              &["stress", "lattice", "gradient", "graded", "field-driven"],
    },
    ExampleScript {
        id:                "cg_sensitivity",
        title:             "CG Sensitivity Analysis",
        category:          "Analysis",
        difficulty:        Difficulty::Intermediate,
        description:       "How each component's mass affects CG position and static margin",
        script:            EX37_SCRIPT,
        related_functions: &["cg_sensitivity", "component_named", "place", "geometry"],
        tags:              &["CG", "sensitivity", "mass", "stability", "balance"],
    },
    ExampleScript {
        id:                "interference_check",
        title:             "Assembly Interference Check",
        category:          "Analysis",
        difficulty:        Difficulty::Intermediate,
        description:       "Verify all components fit inside fuselage without overlap",
        script:            EX38_SCRIPT,
        related_functions: &["interference_check_no_parent", "component_named", "place", "keepout", "geometry"],
        tags:              &["interference", "clearance", "assembly", "component", "check"],
    },
    ExampleScript {
        id:                "geometry_queries",
        title:             "Reference Points and Geometry Queries",
        category:          "Analysis",
        difficulty:        Difficulty::Intermediate,
        description:       "Query surfaces, bounding boxes, and named reference points",
        script:            EX39_SCRIPT,
        related_functions: &["leading_edge", "trailing_edge", "chord_point", "bbox_size", "surface_point", "ref_point"],
        tags:              &["reference-point", "bounding-box", "query", "leading-edge", "surface"],
    },
    ExampleScript {
        id:                "complete_airframe",
        title:             "Complete Fixed-Wing Airframe",
        category:          "Complete Assemblies",
        difficulty:        Difficulty::Advanced,
        description:       "Full reference aircraft with all systems integrated and analyzed",
        script:            EX40_SCRIPT,
        related_functions: &["von_karman_nose", "wing_with_airfoil", "propulsion_analysis", "static_margin", "drag_polar_weighted", "range_endurance"],
        tags:              &["complete", "airframe", "assembly", "fixed-wing", "full-aircraft"],
    },
];
