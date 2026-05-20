// Component-level topology diagnostics for export-safe geometry.

let body_length = 500.0;
let body_width = 172.0;
let body_height = 95.0;
let shell_thickness = 1.6;

let span = 800.0;
let root_chord = 310.0;
let tip_chord = 62.0;
let root_le_x = 92.0;
let wing_sweep_deg = 39.0;
let wing_blend_k = 18.0;

let tail_blend_k = 10.0;
let vtail_span = 179.4;
let vtail_root_chord = 106.6;
let vtail_tip_chord = 54.6;
let vtail_sweep_deg = 28.0;
let vtail_z = 20.0;
let vtail_x = body_length - 78.0;
let vtail_y = 76.0;

let inlet_x = body_length * 0.35;
let inlet_width = 70.0;
let inlet_height = 50.0;
let inlet_wall = shell_thickness;
let inlet_clearance = 10.0;
let inlet_face_extension = 10.0;
let duct_diameter = 70.0;
let duct_samples = 160;
let duct_start_z = body_height * 0.5 + inlet_height * 0.5 + inlet_clearance;

let export_safe_te_mm = 1.6;
let export_safe_le_mm = 1.0;

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

let wing_safe_shape = wing_with_airfoil_export_safe(
    "2311",
    root_chord,
    tip_chord,
    span,
    wing_sweep_deg,
    0.0,
    -1.5,
    export_safe_te_mm,
    export_safe_le_mm
);
let wing_safe = translate(wing_safe_shape, root_le_x, 0.0, -4.0);

let blended_planform = smooth_union(centerbody, wing_safe, wing_blend_k);
let crown = translate(sphere(92.0), body_length * 0.46, 0.0, 16.0);
let fuselage_only = smooth_union(centerbody, scale(crown, 1.55, 1.05, 0.52), 20.0);
let body_outer_pre_inlet = smooth_union(blended_planform, scale(crown, 1.55, 1.05, 0.52), 20.0);

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

let outer_start = rounded_rect_profile(inlet_height, inlet_width, inlet_height * 0.24);
let outer_end = circle_profile((duct_diameter + inlet_wall * 2.0) * 0.5);
let inlet_only = profile_duct_solid(duct_path, outer_start, outer_end, duct_samples);

let vtail_safe_base = rotate(
    wing_with_airfoil_export_safe(
        "0009",
        vtail_root_chord,
        vtail_tip_chord,
        vtail_span,
        vtail_sweep_deg,
        0.0,
        0.0,
        export_safe_te_mm,
        export_safe_le_mm
    ),
    62.0,
    0.0,
    0.0
);
let vtail_safe_shape = intersect(
    vtail_safe_base,
    translate(box_(600.0, 600.0, 600.0), 0.0, 0.0, 300.0)
);
let vtail_safe_right = translate(vtail_safe_shape, vtail_x, vtail_y, vtail_z);
let vtail_safe_left = mirror_y(vtail_safe_right);
let vtail_safe = union(vtail_safe_right, vtail_safe_left);

let aero_export = #{
    parts: [
        #{
            name: "wing_safe",
            sdf: wing_safe,
            aero_role: "outer_mold_line",
            patch_name: "wing_safe",
            bounds_min_mm: [70.0, -405.0, -30.0],
            bounds_max_mm: [490.0, 405.0, 30.0],
            include_in_modes: ["external"],
            preserve_feature: true,
            feature_tags: ["leading_edge", "trailing_edge"],
            min_export_scale_mm: 0.7,
            min_feature_size_mm: min_feature_size_mm(wing_safe_shape),
            feature_protection_strength: 3.0
        },
        #{
            name: "vtail_safe",
            sdf: vtail_safe,
            aero_role: "outer_mold_line",
            patch_name: "vtail_safe",
            bounds_min_mm: [400.0, -130.0, 0.0],
            bounds_max_mm: [540.0, 130.0, 120.0],
            include_in_modes: ["external"],
            preserve_feature: true,
            feature_tags: ["leading_edge", "trailing_edge", "tail_body_junction"],
            min_export_scale_mm: 0.7,
            min_feature_size_mm: min_feature_size_mm(
                wing_with_airfoil_export_safe(
                    "0009",
                    vtail_root_chord,
                    vtail_tip_chord,
                    vtail_span,
                    vtail_sweep_deg,
                    0.0,
                    0.0,
                    export_safe_te_mm,
                    export_safe_le_mm
                )
            ),
            feature_protection_strength: 3.0
        },
        #{
            name: "fuselage_only",
            sdf: fuselage_only,
            aero_role: "outer_mold_line",
            patch_name: "fuselage_only",
            bounds_min_mm: [-8.0, -110.0, -60.0],
            bounds_max_mm: [548.0, 110.0, 150.0],
            include_in_modes: ["external"],
            preserve_feature: true,
            feature_tags: [],
            min_export_scale_mm: 1.0,
            feature_protection_strength: 1.0
        },
        #{
            name: "inlet_only",
            sdf: inlet_only,
            aero_role: "outer_mold_line",
            patch_name: "inlet_only",
            bounds_min_mm: [150.0, -52.0, -10.0],
            bounds_max_mm: [560.0, 52.0, 110.0],
            include_in_modes: ["external"],
            preserve_feature: true,
            feature_tags: ["inlet_lip"],
            min_export_scale_mm: 0.5,
            feature_protection_strength: 2.5
        }
    ]
};

union(union(wing_safe, vtail_safe), union(fuselage_only, inlet_only))
