// 800 mm blended-wing-body EDF loitering platform
// Spec baseline:
// - 800 mm span
// - 500 mm overall length
// - 215 mm max body width
// - 95 mm max body height
// - dorsal conformal-style rectangular inlet
// - internal 70 mm circular duct
// - hollow shell + medium structural layout

// ---------------------------------------------------------------------------
// Primary dimensions
// ---------------------------------------------------------------------------

let body_length = 500.0;
let body_width = 172.0;
let body_height = 95.0;
let shell_thickness = 1.6;

let span = 800.0;
let root_chord = 310.0;
let tip_chord = 62.0;
let root_le_x = 92.0;
let wing_sweep_deg = 39.0;

let body_smooth_k = 18.0;
let wing_blend_k = 18.0;
let tail_blend_k = 10.0;

// ---------------------------------------------------------------------------
// Duct / inlet
// ---------------------------------------------------------------------------

let inlet_x = body_length * 0.35;
let inlet_width = 70.0;
let inlet_height = 50.0;
let inlet_wall = shell_thickness;
let inlet_clearance = 10.0;
let inlet_face_extension = 10.0;
let duct_diameter = 70.0;
let duct_samples = 160;
let duct_start_z = body_height * 0.5 + inlet_height * 0.5 + inlet_clearance;

// ---------------------------------------------------------------------------
// Tail
// ---------------------------------------------------------------------------

let vtail_span = 179.4;
let vtail_root_chord = 106.6;
let vtail_tip_chord = 54.6;
let vtail_sweep_deg = 28.0;
let vtail_z = 20.0;
let vtail_x = body_length - 78.0;
let vtail_y = 76.0;

// ---------------------------------------------------------------------------
// Structure
// ---------------------------------------------------------------------------

let rib_thickness = 2.0;
let bulkhead_thickness = 2.6;
let show_shell = true;
let show_structure = true;
let inspect_cutaway = false;

// ---------------------------------------------------------------------------
// Helper geometry
// ---------------------------------------------------------------------------

let centerbody = fuselage_elliptical(
    body_length,
    body_width,
    body_height,
    95.0,
    58.0,
    1.55,
    0.92,
    0.82
);

let full_wing = translate(
    wing_with_airfoil("2311", root_chord, tip_chord, span, wing_sweep_deg, 0.0, -1.5),
    root_le_x,
    0.0,
    -4.0
);
let blended_planform = smooth_union(centerbody, full_wing, wing_blend_k);

// Add a shallow crown to make the centerbody read as a true lifting body.
let crown = translate(
    sphere(92.0),
    body_length * 0.46,
    0.0,
    16.0
);
let body_outer = smooth_union(blended_planform, scale(crown, 1.55, 1.05, 0.52), 20.0);

// ---------------------------------------------------------------------------
// Dorsal inlet and internal duct
// ---------------------------------------------------------------------------

let duct_path = spline_path([
    [inlet_x - 6.0,  0.0, duct_start_z],
    [inlet_x + 10.0, 0.0, duct_start_z],
    [inlet_x + 28.0, 0.0, duct_start_z - 2.0],
    [inlet_x + 54.0, 0.0, duct_start_z - 8.0],
    [inlet_x + 88.0, 0.0, duct_start_z - 20.0],
    [inlet_x + 122.0, 0.0, 28.0],
    [inlet_x + 156.0, 0.0, 8.0],
    [inlet_x + 186.0, 0.0, 0.0],
    [body_length - 12.0, 0.0, 0.0]
]);

let duct_void_path = spline_path([
    [inlet_x - 6.0 - inlet_face_extension, 0.0, duct_start_z],
    [inlet_x - 6.0,  0.0, duct_start_z],
    [inlet_x + 10.0, 0.0, duct_start_z],
    [inlet_x + 28.0, 0.0, duct_start_z - 2.0],
    [inlet_x + 54.0, 0.0, duct_start_z - 8.0],
    [inlet_x + 88.0, 0.0, duct_start_z - 20.0],
    [inlet_x + 122.0, 0.0, 28.0],
    [inlet_x + 156.0, 0.0, 8.0],
    [inlet_x + 186.0, 0.0, 0.0],
    [body_length - 12.0, 0.0, 0.0],
    [body_length + 54.0, 0.0, 0.0]
]);

let outer_start = rounded_rect_profile(inlet_height, inlet_width, inlet_height * 0.24);
let inner_start = rounded_rect_profile(inlet_height - inlet_wall * 2.0, inlet_width - inlet_wall * 2.0, (inlet_height - inlet_wall * 2.0) * 0.24);
let outer_end = circle_profile((duct_diameter + inlet_wall * 2.0) * 0.5);
let inner_end = circle_profile(duct_diameter * 0.5);

let inlet_outer = profile_duct_solid(
    duct_path,
    outer_start,
    outer_end,
    duct_samples
);

let duct_void = profile_duct_solid(
    duct_void_path,
    inner_start,
    inner_end,
    duct_samples
);

let duct_shell_internal = intersect(
    profile_duct(
        duct_path,
        outer_start,
        outer_end,
        inner_start,
        inner_end,
        60.0,
        60.0,
        duct_samples
    ),
    offset(centerbody, shell_thickness * 3.0)
);

let body_outer = smooth_union(body_outer, inlet_outer, 14.0);

// ---------------------------------------------------------------------------
// V-tail at aft corners
// ---------------------------------------------------------------------------

let vtail_base = rotate(
    wing_with_airfoil("0009", vtail_root_chord, vtail_tip_chord, vtail_span, vtail_sweep_deg, 0.0, 0.0),
    62.0, 0.0, 0.0
);
let vtail_single = intersect(
    vtail_base,
    translate(box_(600.0, 600.0, 600.0), 0.0, 0.0, 300.0)
);
let vtail_right = translate(vtail_single, vtail_x, vtail_y, vtail_z);
let vtail_left = mirror_y(vtail_right);
let body_outer = smooth_union(body_outer, union(vtail_right, vtail_left), tail_blend_k);

// ---------------------------------------------------------------------------
// Shell
// ---------------------------------------------------------------------------

let shell_body = union(
    subtract(shell(body_outer, shell_thickness), duct_void),
    duct_shell_internal
);

// ---------------------------------------------------------------------------
// Internal structure
// ---------------------------------------------------------------------------

let structure_keepout = offset(centerbody, shell_thickness + 1.0);

let rib_root_l = subtract(rib_at_station(full_wing, 75.0, rib_thickness), structure_keepout);
let rib_root_r = subtract(rib_at_station(full_wing, -75.0, rib_thickness), structure_keepout);
let rib_mid_l = subtract(rib_at_station(full_wing, 170.0, rib_thickness), structure_keepout);
let rib_mid_r = subtract(rib_at_station(full_wing, -170.0, rib_thickness), structure_keepout);
let rib_outer_l = subtract(rib_at_station(full_wing, 290.0, rib_thickness), structure_keepout);
let rib_outer_r = subtract(rib_at_station(full_wing, -290.0, rib_thickness), structure_keepout);

let bh_1 = lightening_hole_pattern(bulkhead_at_station(centerbody, 95.0, bulkhead_thickness), 4, 10.0, 8.0);
let bh_2 = lightening_hole_pattern(bulkhead_at_station(centerbody, 180.0, bulkhead_thickness), 5, 12.0, 9.0);
let bh_3 = lightening_hole_pattern(bulkhead_at_station(centerbody, 285.0, bulkhead_thickness), 5, 12.0, 9.0);
let bh_4 = lightening_hole_pattern(bulkhead_at_station(centerbody, 390.0, bulkhead_thickness), 4, 10.0, 8.0);

let structure = union(
    union(union(rib_root_l, rib_root_r), union(rib_mid_l, rib_mid_r)),
    union(
        union(rib_outer_l, rib_outer_r),
        union(union(bh_1, bh_2), union(bh_3, bh_4))
    )
);
let structure = subtract(structure, duct_void);

// ---------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------

let shell_visible = if show_shell { shell_body } else { box_(0.01, 0.01, 0.01) };
let structure_visible = if show_structure { structure } else { box_(0.01, 0.01, 0.01) };
let combined = union(shell_visible, structure_visible);

if inspect_cutaway {
    subtract(
        combined,
        translate(
            box_(body_length * 0.60, body_width * 0.45, body_height * 1.4),
            body_length * 0.48,
            body_width * 0.20,
            body_height * 0.52
        )
    )
} else {
    combined
}
