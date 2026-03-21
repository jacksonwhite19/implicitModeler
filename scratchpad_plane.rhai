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
let add_s_duct = true;

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
// Raised dorsal intake + internal S-duct
// Built from explicit outer/inner custom profiles using the refined
// centerline-distance duct kernel.
// ---------------------------------------------------------------------------
let duct_path = spline_path([
    [intake_x - 8.0,   0.0, intake_mouth_z],
    [intake_x + 10.0,  0.0, intake_mouth_z],
    [intake_x + 34.0,  0.0, intake_mouth_z - 1.0],
    [intake_x + 76.0,  0.0, intake_mouth_z - 5.0],
    [intake_x + 132.0, 0.0, intake_mouth_z - 13.0],
    [intake_x + 192.0, 0.0, 26.0],
    [intake_x + 254.0, 0.0, 8.0],
    [intake_x + 288.0, 0.0, 0.0],
    [fuselage_length - 16.0, 0.0, 0.0]
]);
let duct_void_path = spline_path([
    [intake_x - 8.0 - intake_face_extension, 0.0, intake_mouth_z],
    [intake_x - 8.0,   0.0, intake_mouth_z],
    [intake_x + 10.0,  0.0, intake_mouth_z],
    [intake_x + 34.0,  0.0, intake_mouth_z - 1.0],
    [intake_x + 76.0,  0.0, intake_mouth_z - 5.0],
    [intake_x + 132.0, 0.0, intake_mouth_z - 13.0],
    [intake_x + 192.0, 0.0, 26.0],
    [intake_x + 254.0, 0.0, 8.0],
    [intake_x + 288.0, 0.0, 0.0],
    [fuselage_length - 16.0, 0.0, 0.0],
    [fuselage_length + duct_open_extension, 0.0, 0.0]
]);

let outer_start = rounded_rect_profile(
    intake_outer_height,
    intake_outer_width,
    intake_outer_height * 0.24
);
let inner_start = rounded_rect_profile(
    intake_inner_height,
    intake_inner_width,
    intake_inner_height * 0.24
);
let outer_end = circle_profile((duct_diameter + shell_thickness * 2.0) * 0.5);
let inner_end = circle_profile(duct_diameter * 0.5);

let inlet_outer = if add_s_duct {
    profile_duct_solid(
        duct_path,
        outer_start,
        outer_end,
        duct_profile_samples
    )
} else {
    box_(0.01, 0.01, 0.01)
};
let duct_void = if add_s_duct {
    profile_duct_solid(
        duct_void_path,
        inner_start,
        inner_end,
        duct_profile_samples
    )
} else {
    box_(0.01, 0.01, 0.01)
};
let duct_shell_internal = if add_s_duct {
    intersect(
        profile_duct(
            duct_path,
            outer_start,
            outer_end,
            inner_start,
            inner_end,
            duct_open_extension,
            duct_open_extension,
            duct_profile_samples
        ),
        offset(fuselage, shell_thickness * 3.0)
    )
} else {
    box_(0.01, 0.01, 0.01)
};
let body_outer = if add_s_duct {
    smooth_union(body_outer, inlet_outer, intake_blend_k)
} else {
    body_outer
};

// Hollow shell
let shell_body = if add_s_duct {
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
let structure = if add_s_duct {
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
