// Updated scratchpad aircraft using current export-safe surface tools
// and profile-based inlet/duct geometry.
// Coordinate convention:
//   X = longitudinal (nose to tail)
//   Y = spanwise
//   Z = vertical

// ---------------------------------------------------------------------------
// Primary dimensions
// ---------------------------------------------------------------------------

let fuselage_length = 700.0;
let fuselage_width = 175.0;
let fuselage_height = 100.0;
let nose_length = 120.0;
let tail_length = 65.0;
let nose_bluntness = 1.80;
let tail_bluntness = 0.97;
let fuselage_smoothness = 0.74;

let shell_thickness = 1.5;
let min_te_thickness = 1.5;
let min_le_radius = 1.5;

let wing_span = 700.0;
let wing_root_chord = 170.0;
let wing_tip_chord = 65.0;
let wing_sweep_deg = 15.0;
let wing_le_x = 230.0;
let wing_root_blend_k = 16.0;

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
// Inlet / duct
// Smaller and cleaner than the old scratchpad version.
// ---------------------------------------------------------------------------

let inlet_mode = "single";   // "none" or "single"
let intake_x = 286.0;
let intake_outer_width = 69.0;
let intake_outer_height = 30.0;
let inlet_wall = shell_thickness;
let intake_inner_width = intake_outer_width - inlet_wall * 2.0;
let intake_inner_height = intake_outer_height - inlet_wall * 2.0;
let duct_diameter = 44.0;
let inlet_clearance = 9.0;
let intake_face_extension = 8.0;
let duct_open_extension = 36.0;
let duct_profile_samples = 160;
let inlet_blend_k = 10.0;

// ---------------------------------------------------------------------------
// Structure / display
// ---------------------------------------------------------------------------

let wing_rib_thickness = 2.2;
let fuselage_bulkhead_thickness = 3.0;
let show_shell = true;
let show_structure = true;
let inspect_cutaway = false;
let cutaway_y_min = 0.0;
let cutaway_z_min = 15.0;

// ---------------------------------------------------------------------------
// Fuselage
// ---------------------------------------------------------------------------

let fuselage = fuselage_elliptical(
    fuselage_length,
    fuselage_width,
    fuselage_height,
    nose_length,
    tail_length,
    nose_bluntness,
    tail_bluntness,
    fuselage_smoothness
);

let fuselage_top_z = fuselage_height * 0.5;
let intake_mouth_z = fuselage_top_z + intake_outer_height * 0.5 + inlet_clearance;

// ---------------------------------------------------------------------------
// Lifting surfaces using export-safe airfoil generation
// ---------------------------------------------------------------------------

let wing = translate(
    wing_with_airfoil_export_safe(
        "2412",
        wing_root_chord,
        wing_tip_chord,
        wing_span,
        wing_sweep_deg,
        0.0,
        0.0,
        min_te_thickness,
        min_le_radius
    ),
    wing_le_x,
    0.0,
    0.0
);

let body_outer = smooth_union(fuselage, wing, wing_root_blend_k);

let htail = translate(
    wing_with_airfoil_export_safe(
        "0009",
        htail_root_chord,
        htail_tip_chord,
        htail_span,
        0.0,
        0.0,
        0.0,
        min_te_thickness,
        min_le_radius
    ),
    htail_le_x,
    0.0,
    tail_z
);

let vtail_raw = rotate(
    wing_with_airfoil_export_safe(
        "0009",
        vtail_root_chord,
        vtail_tip_chord,
        vtail_span,
        vtail_sweep_deg,
        0.0,
        0.0,
        min_te_thickness,
        min_le_radius
    ),
    90.0,
    0.0,
    0.0
);

let vtail_upper_only = intersect(
    vtail_raw,
    translate(box_(1200.0, 1200.0, 1200.0), 0.0, 0.0, 600.0)
);
let vtail = translate(vtail_upper_only, vtail_le_x, 0.0, tail_z);

let body_outer = smooth_union(body_outer, htail, tail_blend_k);
let body_outer = smooth_union(body_outer, vtail, tail_blend_k);

// ---------------------------------------------------------------------------
// Profile-based dorsal inlet and internal duct
// ---------------------------------------------------------------------------

let duct_path_points = [
    [intake_x - 4.0, 0.0, intake_mouth_z],
    [intake_x + 8.0, 0.0, intake_mouth_z],
    [intake_x + 22.0, 0.0, intake_mouth_z - 1.0],
    [intake_x + 42.0, 0.0, intake_mouth_z - 4.0],
    [intake_x + 72.0, 0.0, intake_mouth_z - 10.0],
    [intake_x + 116.0, 0.0, 25.0],
    [intake_x + 156.0, 0.0, 10.0],
    [intake_x + 194.0, 0.0, 2.0],
    [fuselage_length - 20.0, 0.0, 0.0]
];
let duct_void_path_points = [
    [intake_x - 4.0 - intake_face_extension, 0.0, intake_mouth_z],
    [intake_x - 4.0, 0.0, intake_mouth_z],
    [intake_x + 8.0, 0.0, intake_mouth_z],
    [intake_x + 22.0, 0.0, intake_mouth_z - 1.0],
    [intake_x + 42.0, 0.0, intake_mouth_z - 4.0],
    [intake_x + 72.0, 0.0, intake_mouth_z - 10.0],
    [intake_x + 116.0, 0.0, 25.0],
    [intake_x + 156.0, 0.0, 10.0],
    [intake_x + 194.0, 0.0, 2.0],
    [fuselage_length - 20.0, 0.0, 0.0],
    [fuselage_length + duct_open_extension, 0.0, 0.0]
];

let duct_path = spline_path(duct_path_points);
let duct_void_path = spline_path(duct_void_path_points);

let outer_start = rounded_rect_profile(
    intake_outer_width,
    intake_outer_height,
    intake_outer_height * 0.22
);
let inner_start = rounded_rect_profile(
    intake_inner_width,
    intake_inner_height,
    intake_inner_height * 0.20
);
let outer_end = circle_profile((duct_diameter + inlet_wall * 2.0) * 0.5);
let inner_end = circle_profile(duct_diameter * 0.5);

let inlet_outer = profile_duct_solid(
    duct_path,
    outer_start,
    outer_end,
    duct_profile_samples
);

let duct_void = profile_duct_solid(
    duct_void_path,
    inner_start,
    inner_end,
    duct_profile_samples
);

let duct_shell_internal = profile_duct(
    duct_path,
    outer_start,
    outer_end,
    inner_start,
    inner_end,
    60.0,
    60.0,
    duct_profile_samples
);

let body_outer = if inlet_mode != "none" {
    smooth_union(body_outer, inlet_outer, inlet_blend_k)
} else {
    body_outer
};

let aero_body_outer = if inlet_mode != "none" {
    subtract(body_outer, duct_void)
} else {
    body_outer
};

// ---------------------------------------------------------------------------
// Shell and structure
// ---------------------------------------------------------------------------

let shell_body = if inlet_mode != "none" {
    union(
        subtract(shell(body_outer, shell_thickness), duct_void),
        duct_shell_internal
    )
} else {
    shell(body_outer, shell_thickness)
};

let fuselage_structure_keepout = offset(fuselage, shell_thickness + 1.0);

let wing_rib_inboard_l = subtract(rib_at_station(wing, 95.0, wing_rib_thickness), fuselage_structure_keepout);
let wing_rib_inboard_r = subtract(rib_at_station(wing, -95.0, wing_rib_thickness), fuselage_structure_keepout);
let wing_rib_mid_l = subtract(rib_at_station(wing, 185.0, wing_rib_thickness), fuselage_structure_keepout);
let wing_rib_mid_r = subtract(rib_at_station(wing, -185.0, wing_rib_thickness), fuselage_structure_keepout);
let wing_rib_out_l = subtract(rib_at_station(wing, 285.0, wing_rib_thickness), fuselage_structure_keepout);
let wing_rib_out_r = subtract(rib_at_station(wing, -285.0, wing_rib_thickness), fuselage_structure_keepout);

let bh_nose = lightening_hole_pattern(
    bulkhead_at_station(fuselage, 120.0, fuselage_bulkhead_thickness),
    4,
    10.0,
    8.0
);
let bh_wing = lightening_hole_pattern(
    bulkhead_at_station(fuselage, 300.0, fuselage_bulkhead_thickness),
    6,
    14.0,
    10.0
);
let bh_aft = lightening_hole_pattern(
    bulkhead_at_station(fuselage, 520.0, fuselage_bulkhead_thickness),
    5,
    10.0,
    8.0
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

// ---------------------------------------------------------------------------
// Aero export metadata
// ---------------------------------------------------------------------------

let refinement_radii = [18.0, 18.0, 18.0, 18.0, 18.0, 18.0, 16.0, 16.0, 16.0];

let aero_parts = [
    #{
        name: "aircraft_oml",
        sdf: aero_body_outer,
        aero_role: "outer_mold_line",
        patch_name: "aircraft",
        bounds_min_mm: [-20.0, -360.0, -40.0],
        bounds_max_mm: [740.0, 360.0, 120.0],
        include_in_modes: ["external", "external_plus_inlets", "full_fluid_boundary"],
        preserve_feature: true,
        feature_tags: ["leading_edge", "trailing_edge", "wing_body_junction", "tail_body_junction", "inlet_lip"],
        min_export_scale_mm: 0.8,
        min_feature_size_mm: shell_thickness,
        feature_protection_strength: 3.0,
        refinement_body_sdf: body_outer,
        refinement_void_sdf: duct_void,
        refinement_path_points_mm: duct_path_points,
        refinement_path_radii_mm: refinement_radii,
        refinement_path_forward_extend_mm: 8.0,
        refinement_path_aft_extend_mm: 12.0,
        refinement_path_target_cell_mm: 0.5,
        priority: 0
    },
    #{
        name: "duct_internal",
        sdf: duct_shell_internal,
        aero_role: "flow_path_internal",
        patch_name: "duct",
        bounds_min_mm: [240.0, -32.0, -24.0],
        bounds_max_mm: [740.0, 32.0, 92.0],
        include_in_modes: ["external_plus_inlets", "full_fluid_boundary"],
        preserve_feature: true,
        feature_tags: ["inlet_lip", "nozzle_exit"],
        min_export_scale_mm: 0.6,
        min_feature_size_mm: inlet_wall,
        feature_protection_strength: 2.5,
        refinement_path_points_mm: duct_path_points,
        refinement_path_radii_mm: refinement_radii,
        refinement_path_forward_extend_mm: 8.0,
        refinement_path_aft_extend_mm: 12.0,
        refinement_path_target_cell_mm: 0.5,
        priority: 1
    },
    #{
        name: "internal_structure",
        sdf: structure,
        aero_role: "internal_structure",
        patch_name: "structure",
        bounds_min_mm: [40.0, -320.0, -30.0],
        bounds_max_mm: [620.0, 320.0, 90.0],
        include_in_modes: [],
        preserve_feature: false,
        min_export_scale_mm: 1.0,
        min_feature_size_mm: wing_rib_thickness,
        feature_protection_strength: 0.0,
        priority: 2
    }
];

let aero_export = #{
    parts: aero_parts
};

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
            box_(fuselage_length * 0.65, fuselage_width * 0.5, fuselage_height),
            fuselage_length * 0.48,
            cutaway_y_min + fuselage_width * 0.25,
            cutaway_z_min + fuselage_height * 0.5
        )
    )
} else {
    combined
}
