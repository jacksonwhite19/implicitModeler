// Scratchpad plane from scratchpad.md
// Coordinate convention:
//   X = longitudinal (nose to tail)
//   Y = spanwise
//   Z = vertical

// ---------------------------------------------------------------------------
// Top-level dimensions
// Edit these directly or expose them in the Dimensions panel.
// ---------------------------------------------------------------------------

let fuselage_length = 700.0;
let fuselage_width = 175.0;
let fuselage_height = 100.0;
let nose_length = 120.0;
let tail_length = 65.0;
let nose_bluntness = 1.80;
let tail_bluntness = 0.97;
let fuselage_smoothness = 0.70;   // 0 = rigid frame match, 1 = stronger fairing

let wing_span = 700.0;
let wing_root_chord = 170.0;
let wing_tip_chord = 65.0;
let wing_sweep_deg = 15.0;
let wing_le_x = 230.0;
let wing_root_blend_k = 16.0;
let shell_thickness = 1.0;
let wing_rib_thickness = 2.2;
let fuselage_bulkhead_thickness = 3.0;
let show_shell = true;
let show_structure = true;
let inspect_cutaway = false;
let cutaway_y_min = 0.0;
let cutaway_z_min = 15.0;
let inlet_mode = "single";   // "none", "single", "dual"

let intake_x = 250.0;
let intake_top_z = 68.0;
let intake_outer_width = fuselage_width * 0.675;
let intake_outer_height = intake_outer_width * 0.70;
let intake_inner_width = intake_outer_width - 12.0;
let intake_inner_height = intake_outer_height - 10.0;
let duct_diameter = 90.0;
let intake_surface_offset = 10.0;
let intake_blend_k = 14.0;
let duct_blend_k = 16.0;
let duct_profile_samples = 160;
let duct_open_extension = 70.0;
let intake_mouth_clearance = 8.0;
let intake_face_extension = 10.0;
let intake_front_raise = 5.0;
let inlet_morph_start = 0.10;
let inlet_morph_end = 0.90;
let dual_inlet_guess_y = 52.0;
let dual_inlet_guess_z = 52.0;
let dual_inlet_end_diameter = 70.0;

let htail_span = 400.0;
let htail_root_chord = 98.0;
let htail_tip_chord = 80.0;
let htail_le_x = 500.0;
let tail_z = 13.0;
let tail_blend_k = 10.0;

let vtail_span = 250.0;
let vtail_root_chord = 105.0;
let vtail_tip_chord = 66.0;
let vtail_sweep_deg = 35.0;
let vtail_le_x = 500.0;

// ---------------------------------------------------------------------------
// Fuselage
// Elliptical body with direct nose/tail control.
// ---------------------------------------------------------------------------
let fuselage = fuselage_elliptical(
    fuselage_length, fuselage_width, fuselage_height,
    nose_length, tail_length,
    nose_bluntness, tail_bluntness,
    fuselage_smoothness
);
let fuselage_top_z = fuselage_height * 0.5;
let intake_mouth_z = fuselage_top_z + intake_outer_height * 0.5 + intake_mouth_clearance + intake_front_raise;

// ---------------------------------------------------------------------------
// Wing
// 700 mm span, 15 deg sweep, 80 mm root chord, 65 mm tip chord
// LE at x = 375 mm
// ---------------------------------------------------------------------------

let wing = translate(
    wing_with_airfoil("2412", wing_root_chord, wing_tip_chord, wing_span, wing_sweep_deg, 0.0, 0.0),
    wing_le_x, 0.0, 0.0
);

let body_outer = smooth_union(fuselage, wing, wing_root_blend_k);

// ---------------------------------------------------------------------------
// Tail
// Conventional tail, no control surfaces
// H-tail: 325 mm span, 65/65 chord, no sweep, LE at x = 500 mm
// V-tail: 200 mm total span, 60/30 chord, 30 deg sweep, centered on h-tail
// ---------------------------------------------------------------------------

// wing_with_airfoil already returns a full-span surface.
let htail = translate(
    wing_with_airfoil("0009", htail_root_chord, htail_tip_chord, htail_span, 0.0, 0.0, 0.0),
    htail_le_x, 0.0, tail_z
);

// Build the fin from the same wing primitive, rotate it upright, then clip away
// the lower half so the root sits on the fuselage centerplane without a hacky cant.
let vtail_raw = rotate(
    wing_with_airfoil("0009", vtail_root_chord, vtail_tip_chord, vtail_span, vtail_sweep_deg, 0.0, 0.0),
    90.0, 0.0, 0.0
);
let vtail_upper_only = intersect(
    vtail_raw,
    translate(box_(1200.0, 1200.0, 1200.0), 0.0, 0.0, 600.0)
);
let vtail = translate(vtail_upper_only, vtail_le_x, 0.0, tail_z);

// Final outer mold line
let body_outer = smooth_union(body_outer, htail, tail_blend_k);
let body_outer = smooth_union(body_outer, vtail, tail_blend_k);

// ---------------------------------------------------------------------------
// Intake + internal duct
// Canonical path using conformal_profile_duct_x(...) for a single dorsal inlet
// or mirrored_dual_conformal_profile_duct_x(...) for a symmetric dual inlet.
// ---------------------------------------------------------------------------
let single_outer_start = rounded_rect_profile(
    intake_outer_width,
    intake_outer_height,
    intake_outer_height * 0.24
);
let single_inner_start = rounded_rect_profile(
    intake_inner_width,
    intake_inner_height,
    intake_inner_height * 0.24
);
let single_outer_end = circle_profile((duct_diameter + shell_thickness * 2.0) * 0.5);
let single_inner_end = circle_profile(duct_diameter * 0.5);
let single_duct_path = spline_path([
    [intake_x, 0.0, intake_mouth_z],
    [intake_x + 14.0, 0.0, intake_mouth_z],
    [intake_x + 42.0, 0.0, intake_mouth_z - 1.5],
    [intake_x + 86.0, 0.0, intake_mouth_z - 5.5],
    [intake_x + 140.0, 0.0, intake_mouth_z - 12.0],
    [intake_x + 208.0, 0.0, 28.0],
    [intake_x + 268.0, 0.0, 8.0],
    [intake_x + 308.0, 0.0, 0.0],
    [fuselage_length - 22.0, 0.0, 0.0]
]);

let dual_outer_start = rounded_rect_profile(50.0, 28.0, 4.0);
let dual_inner_start = rounded_rect_profile(46.0, 24.0, 3.0);
let dual_outer_end = circle_profile(dual_inlet_end_diameter * 0.5 + shell_thickness);
let dual_inner_end = circle_profile(dual_inlet_end_diameter * 0.5);
let dual_left_path = spline_path([
    [intake_x, dual_inlet_guess_y, intake_mouth_z - 6.0],
    [intake_x + 75.0, dual_inlet_guess_y - 10.0, intake_mouth_z - 18.0],
    [intake_x + 145.0, 22.0, 24.0],
    [intake_x + 210.0, 8.0, 8.0],
    [intake_x + 250.0, 0.0, 0.0],
    [fuselage_length - 22.0, 0.0, 0.0]
]);

let inlet_parts = if inlet_mode == "single" {
    conformal_profile_duct_x(
        fuselage,
        single_outer_start,
        single_inner_start,
        intake_x,
        0.0,
        intake_mouth_z,
        point(1.0, 0.0, 0.0),
        intake_surface_offset,
        single_duct_path,
        single_outer_end,
        single_inner_end,
        intake_face_extension,
        duct_open_extension,
        inlet_morph_start,
        inlet_morph_end,
        duct_profile_samples
    )
} else if inlet_mode == "dual" {
    mirrored_dual_conformal_profile_duct_x(
        fuselage,
        dual_outer_start,
        dual_inner_start,
        intake_x,
        dual_inlet_guess_y,
        dual_inlet_guess_z,
        point(1.0, 0.0, 0.0),
        dual_left_path,
        dual_outer_end,
        dual_inner_end,
        intake_surface_offset,
        intake_face_extension,
        duct_open_extension + 20.0,
        0.15,
        0.75,
        duct_profile_samples
    )
} else {
    [box_(0.01, 0.01, 0.01), box_(0.01, 0.01, 0.01), box_(0.01, 0.01, 0.01)]
};

let inlet_outer = inlet_parts[0];
let duct_void = inlet_parts[1];
let duct_shell_internal = inlet_parts[2];

let body_outer = if inlet_mode != "none" {
    smooth_union(body_outer, inlet_outer, intake_blend_k)
} else {
    body_outer
};

// Hollow shell
let shell_body = if inlet_mode != "none" {
    union(
        subtract(shell(body_outer, shell_thickness), duct_void),
        duct_shell_internal
    )
} else {
    shell(body_outer, shell_thickness)
};
let fuselage_structure_keepout = offset(fuselage, shell_thickness + 1.0);

// Wing internal structure
let wing_rib_inboard_l = subtract(rib_at_station(wing, 95.0, wing_rib_thickness), fuselage_structure_keepout);
let wing_rib_inboard_r = subtract(rib_at_station(wing, -95.0, wing_rib_thickness), fuselage_structure_keepout);
let wing_rib_mid_l = subtract(rib_at_station(wing, 185.0, wing_rib_thickness), fuselage_structure_keepout);
let wing_rib_mid_r = subtract(rib_at_station(wing, -185.0, wing_rib_thickness), fuselage_structure_keepout);
let wing_rib_out_l = subtract(rib_at_station(wing, 285.0, wing_rib_thickness), fuselage_structure_keepout);
let wing_rib_out_r = subtract(rib_at_station(wing, -285.0, wing_rib_thickness), fuselage_structure_keepout);

// Fuselage bulkheads
let bh_nose_x = 120.0;
let bh_wing_x = 300.0;
let bh_aft_x = 520.0;

let bh_nose = lightening_hole_pattern(
    bulkhead_at_station(fuselage, bh_nose_x, fuselage_bulkhead_thickness),
    4, 10.0, 8.0
);
let bh_wing = lightening_hole_pattern(
    bulkhead_at_station(fuselage, bh_wing_x, fuselage_bulkhead_thickness),
    6, 14.0, 10.0
);
let bh_aft = lightening_hole_pattern(
    bulkhead_at_station(fuselage, bh_aft_x, fuselage_bulkhead_thickness),
    5, 10.0, 8.0
);

let structure = union(
    union(union(wing_rib_inboard_l, wing_rib_inboard_r), union(wing_rib_mid_l, wing_rib_mid_r)),
    union(
        union(wing_rib_out_l, wing_rib_out_r),
        union(union(bh_nose, bh_wing), bh_aft)
    )
);
let structure = if inlet_mode != "none" {
    subtract(structure, duct_void)
} else {
    structure
};

let shell_visible = if show_shell { shell_body } else { box_(0.01, 0.01, 0.01) };
let structure_visible = if show_structure { structure } else { box_(0.01, 0.01, 0.01) };
let combined = union(shell_visible, structure_visible);

if inspect_cutaway {
    subtract(
        combined,
        translate(
            box_(fuselage_length * 0.65, fuselage_width * 0.5, fuselage_height),
            fuselage_length * 0.48,
            cutaway_y_min + fuselage_width * 0.25,
            cutaway_z_min + fuselage_height * 0.5
        )
    )
} else {
    combined
}
