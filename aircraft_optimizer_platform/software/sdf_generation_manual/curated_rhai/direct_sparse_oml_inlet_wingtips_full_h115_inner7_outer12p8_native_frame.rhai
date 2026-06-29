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

let inlet_mode = "single"; // experiment: enable dorsal inlet/duct opening
let intake_x = 286.0;
let intake_outer_width = 88.0; // experiment: actual inlet width
let intake_outer_height = 60.0; // experiment: actual inlet height
let inlet_wall = 2.5; // experiment: inlet wall thickness
let intake_inner_width = intake_outer_width - inlet_wall * 2.0; // derived by constant_wall_profile_inlet_parts
let intake_inner_height = intake_outer_height - inlet_wall * 2.0; // derived by constant_wall_profile_inlet_parts
let duct_diameter = 44.0;
let inlet_clearance = 9.0;
let intake_face_extension = 8.0;
let duct_open_extension = 260.0; // experiment: extend duct void past aft fuselage for exhaust
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
let intake_mouth_z = fuselage_top_z + intake_outer_height * 0.5 + inlet_clearance - 5.0; // use actual inlet height and lower inlet face 5 mm

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
    [intake_x + 36.0, 0.0, intake_mouth_z],
    [intake_x + 84.0, 0.0, intake_mouth_z],
    [intake_x + 122.0, 0.0, intake_mouth_z - 12.0],
    [intake_x + 158.0, 0.0, intake_mouth_z - 32.0],
    [intake_x + 196.0, 0.0, intake_mouth_z - 54.0],
    [intake_x + 236.0, 0.0, 12.0],
    [intake_x + 274.0, 0.0, 0.0],
    [intake_x + 330.0, 0.0, 0.0],
    [fuselage_length - 60.0, 0.0, 0.0]
];
let duct_void_path_points = [
    [intake_x - 4.0 - intake_face_extension, 0.0, intake_mouth_z],
    [intake_x - 4.0, 0.0, intake_mouth_z],
    [intake_x + 36.0, 0.0, intake_mouth_z],
    [intake_x + 84.0, 0.0, intake_mouth_z],
    [intake_x + 122.0, 0.0, intake_mouth_z - 12.0],
    [intake_x + 158.0, 0.0, intake_mouth_z - 32.0],
    [intake_x + 196.0, 0.0, intake_mouth_z - 54.0],
    [intake_x + 236.0, 0.0, 12.0],
    [intake_x + 274.0, 0.0, 0.0],
    [intake_x + 330.0, 0.0, 0.0],
    [fuselage_length - 60.0, 0.0, 0.0],
    [fuselage_length + duct_open_extension, 0.0, 0.0]
];

let duct_path = spline_path(duct_path_points);
let duct_void_path = spline_path(duct_void_path_points);

// profile_duct_solid maps this start profile into the duct frame with the
// first profile axis showing up as section height in the native-frame plots.
// Swap arguments here so the rendered mouth is actually 88 mm wide x 60 mm tall.
let outer_start = rounded_rect_profile(
    intake_outer_height,
    intake_outer_width,
    intake_outer_height * 0.22
);
let inner_start = rounded_rect_profile(
    intake_inner_height,
    intake_inner_width,
    intake_inner_height * 0.20
);
let outer_end = rounded_rect_profile(
    duct_diameter + inlet_wall * 2.0,
    duct_diameter + inlet_wall * 2.0,
    (duct_diameter + inlet_wall * 2.0) * 0.5
);
let inner_end = circle_profile(duct_diameter * 0.5);

let inlet_outer = profile_duct_solid(
    duct_path,
    outer_start,
    outer_end,
    duct_profile_samples
);

let duct_void_core = profile_duct_solid(
    duct_path,
    inner_start,
    inner_end,
    duct_profile_samples
);
let duct_void_inlet_extension_path = spline_path([
    [intake_x - 4.0 - intake_face_extension, 0.0, intake_mouth_z],
    [intake_x - 4.0, 0.0, intake_mouth_z]
]);
let duct_void_inlet_extension = profile_duct_solid(
    duct_void_inlet_extension_path,
    inner_start,
    inner_start,
    duct_profile_samples
);
let duct_void_exhaust_extension_path = spline_path([
    [fuselage_length - 60.0, 0.0, 0.0],
    [fuselage_length + duct_open_extension, 0.0, 0.0]
]);
let duct_void_exhaust_extension = profile_duct_solid(
    duct_void_exhaust_extension_path,
    inner_end,
    inner_end,
    duct_profile_samples
);
let duct_void = union(
    union(duct_void_core, duct_void_inlet_extension),
    duct_void_exhaust_extension
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

// Full-length visibility mask keeps the inlet available through its transition
// while preventing any accidental far-field tail of the helper from joining OML.
let inlet_outer_visible = intersect(
    inlet_outer,
    translate(box_(430.0, 240.0, 180.0), intake_x + 201.0, 0.0, intake_mouth_z - 4.0)
);

let body_outer = if inlet_mode != "none" {
    smooth_union(body_outer, inlet_outer_visible, inlet_blend_k)
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
// Experimental wingtip / wing-fence helper
// ---------------------------------------------------------------------------

let wingtip_enable = true;
let wingtip_snap_to_main_wing_tip = true;
// "up", "down", or "full". Full keeps both halves of the rotated wing so the
// feature points both above and below the main wing/fence station.
let wingtip_span_mode = "full";
let wingtip_custom_x = wing_le_x + tan(to_rad(wing_sweep_deg)) * (wing_span * 0.5);
let wingtip_custom_y_abs = wing_span * 0.5;
let wingtip_custom_z = 0.0;

let wingtip_airfoil = "0010";
let wingtip_root_chord = 58.0;
let wingtip_tip_chord = 36.0;
let wingtip_length = 105.800000;
let wingtip_sweep_deg = 12.0;
let wingtip_dihedral_deg = 0.0; // negative value is anhedral in this rotated frame
let wingtip_twist_deg = 0.0;
let wingtip_te_thickness = min_te_thickness;
let wingtip_le_radius = min_le_radius;
// "skin_function" uses an experimental helper whose root station follows the
// parent wing tip airfoil skin. "boolean_skin_clip" keeps the previous fallback.
let wingtip_root_match_method = "skin_function";
// Main wing is currently NACA 2412, so max thickness is 12% of local chord.
let main_wing_tip_thickness_ratio = 0.12;
let main_wing_tip_max_thickness = wing_tip_chord * main_wing_tip_thickness_ratio;
// Root fillet radii for strength at the wing/wingtip junction.
// Outer controls the body/wingtip smooth union. Inner controls the skin-matched
// one-sided root cut. In full mode, the fin is built from up+down halves so the
// inner radius applies to both root exits.
let wingtip_outer_fillet_radius_mm = 12.800000;
let wingtip_inner_fillet_radius_mm = 7.000000;
// Positive values move the wingtip/fence inward from the actual wingtip.
// Default is zero so the wingtip centerline is exactly at the main wingtip.
let wingtip_mount_inset_y = 0.0;

fn make_vertical_wingtip_raw(airfoil, root_chord, tip_chord, length, sweep_deg, dihedral_deg, twist_deg, te_mm, le_mm) {
    rotate(
        wing_with_airfoil_export_safe(
            airfoil,
            root_chord,
            tip_chord,
            length,
            sweep_deg,
            dihedral_deg,
            twist_deg,
            te_mm,
            le_mm
        ),
        90.0,
        0.0,
        0.0
    )
}

fn make_skin_matched_vertical_wingtip_raw(airfoil, root_chord, tip_chord, length, sweep_deg, dihedral_deg, twist_deg, te_mm, le_mm, parent_airfoil, parent_chord, span_mode, inner_fillet_radius) {
    if span_mode == "full" {
        union(
            skin_matched_wingtip_export_safe(
                airfoil,
                root_chord,
                tip_chord,
                length,
                sweep_deg,
                dihedral_deg,
                twist_deg,
                te_mm,
                le_mm,
                parent_airfoil,
                parent_chord,
                "up",
                inner_fillet_radius
            ),
            skin_matched_wingtip_export_safe(
                airfoil,
                root_chord,
                tip_chord,
                length,
                sweep_deg,
                dihedral_deg,
                twist_deg,
                te_mm,
                le_mm,
                parent_airfoil,
                parent_chord,
                "down",
                inner_fillet_radius
            )
        )
    } else {
        skin_matched_wingtip_export_safe(
            airfoil,
            root_chord,
            tip_chord,
            length,
            sweep_deg,
            dihedral_deg,
            twist_deg,
            te_mm,
            le_mm,
            parent_airfoil,
            parent_chord,
            span_mode,
            inner_fillet_radius
        )
    }
}

fn make_main_wing_tip_skin_cutter(chord, te_mm, le_mm) {
    wing_with_airfoil_export_safe(
        "2412",
        chord,
        chord,
        1200.0,
        0.0,
        0.0,
        0.0,
        te_mm,
        le_mm
    )
}

fn clip_wingtip_span(raw, skin_cutter, mode, guard_clearance_z) {
    let raw_trimmed_to_wing_skin = subtract(raw, skin_cutter);
    if mode == "up" {
        intersect(
            raw_trimmed_to_wing_skin,
            translate(box_(1200.0, 1200.0, 1200.0), 0.0, 0.0, 600.0 - guard_clearance_z)
        )
    } else if mode == "down" {
        intersect(
            raw_trimmed_to_wing_skin,
            translate(box_(1200.0, 1200.0, 1200.0), 0.0, 0.0, -600.0 + guard_clearance_z)
        )
    } else {
        raw
    }
}

fn make_wingtip_at(airfoil, root_chord, tip_chord, length, sweep_deg, dihedral_deg, twist_deg, te_mm, le_mm, span_mode, skin_cutter, guard_clearance_z, root_match_method, parent_airfoil, parent_chord, inner_fillet_radius, x, y, z) {
    if root_match_method == "skin_function" {
        translate(
            make_skin_matched_vertical_wingtip_raw(
                airfoil,
                root_chord,
                tip_chord,
                length,
                sweep_deg,
                dihedral_deg,
                twist_deg,
                te_mm,
                le_mm,
                parent_airfoil,
                parent_chord,
                span_mode,
                inner_fillet_radius
            ),
            x,
            y,
            z
        )
    } else {
        let raw = make_vertical_wingtip_raw(
            airfoil,
            root_chord,
            tip_chord,
            length,
            sweep_deg,
            dihedral_deg,
            twist_deg,
            te_mm,
            le_mm
        );
        translate(clip_wingtip_span(raw, skin_cutter, span_mode, guard_clearance_z), x, y, z)
    }
}

let wingtip_half_span = wing_span * 0.5;
let wingtip_snap_x = wing_le_x + tan(to_rad(wing_sweep_deg)) * wingtip_half_span;
let wingtip_x = if wingtip_snap_to_main_wing_tip { wingtip_snap_x } else { wingtip_custom_x };
let wingtip_y_abs = if wingtip_snap_to_main_wing_tip { wingtip_half_span - wingtip_mount_inset_y } else { wingtip_custom_y_abs };
let wingtip_z = if wingtip_snap_to_main_wing_tip { 0.0 } else { wingtip_custom_z };
let wingtip_skin_cutter = make_main_wing_tip_skin_cutter(wing_tip_chord, min_te_thickness, min_le_radius);

let wingtip_right = make_wingtip_at(
    wingtip_airfoil,
    wingtip_root_chord,
    wingtip_tip_chord,
    wingtip_length,
    wingtip_sweep_deg,
    wingtip_dihedral_deg,
    wingtip_twist_deg,
    wingtip_te_thickness,
    wingtip_le_radius,
    wingtip_span_mode,
    wingtip_skin_cutter,
    main_wing_tip_max_thickness,
    wingtip_root_match_method,
    "2412",
    wing_tip_chord,
    wingtip_inner_fillet_radius_mm,
    wingtip_x,
    wingtip_y_abs,
    wingtip_z
);
let wingtip_left = make_wingtip_at(
    wingtip_airfoil,
    wingtip_root_chord,
    wingtip_tip_chord,
    wingtip_length,
    wingtip_sweep_deg,
    wingtip_dihedral_deg,
    wingtip_twist_deg,
    wingtip_te_thickness,
    wingtip_le_radius,
    wingtip_span_mode,
    wingtip_skin_cutter,
    main_wing_tip_max_thickness,
    wingtip_root_match_method,
    "2412",
    wing_tip_chord,
    wingtip_inner_fillet_radius_mm,
    wingtip_x,
    -wingtip_y_abs,
    wingtip_z
);
let wingtip_pair = union(wingtip_left, wingtip_right);
let aero_body_outer_wingtips = if wingtip_enable {
    smooth_union(aero_body_outer, wingtip_pair, wingtip_outer_fillet_radius_mm)
} else {
    aero_body_outer
};


// ---------------------------------------------------------------------------
// Direct sparse SDF MC experiment output
// ---------------------------------------------------------------------------

// Native scratchpad frame: X longitudinal, Y spanwise, Z vertical.
let aircraft_oml_inlet_wingtips_native_frame = aero_body_outer_wingtips;
aircraft_oml_inlet_wingtips_native_frame
