// Compare original vs export-safe wing and V-tail geometry at fixed cell sizes.

let span = 800.0;
let root_chord = 310.0;
let tip_chord = 62.0;
let root_le_x = 92.0;
let wing_sweep_deg = 39.0;
let shell_thickness = 1.6;

let vtail_span = 179.4;
let vtail_root_chord = 106.6;
let vtail_tip_chord = 54.6;
let vtail_sweep_deg = 28.0;
let vtail_z = 20.0;
let vtail_x = 422.0;
let vtail_y = 76.0;

let export_safe_te_mm = 1.6;
let export_safe_le_mm = 1.0;

let wing_orig_shape = wing_with_airfoil(
    "2311",
    root_chord,
    tip_chord,
    span,
    wing_sweep_deg,
    0.0,
    -1.5
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

let wing_orig = translate(wing_orig_shape, root_le_x, 0.0, -4.0);
let wing_safe = translate(wing_safe_shape, root_le_x, 0.0, -4.0);

let wing_orig_min_feature = min_feature_size_mm(wing_orig_shape);
let wing_safe_min_feature = min_feature_size_mm(wing_safe_shape);

let vtail_orig_base = rotate(
    wing_with_airfoil(
        "0009",
        vtail_root_chord,
        vtail_tip_chord,
        vtail_span,
        vtail_sweep_deg,
        0.0,
        0.0
    ),
    62.0,
    0.0,
    0.0
);
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

let vtail_orig_shape = intersect(
    vtail_orig_base,
    translate(box_(600.0, 600.0, 600.0), 0.0, 0.0, 300.0)
);
let vtail_safe_shape = intersect(
    vtail_safe_base,
    translate(box_(600.0, 600.0, 600.0), 0.0, 0.0, 300.0)
);

let vtail_orig_right = translate(vtail_orig_shape, vtail_x, vtail_y, vtail_z);
let vtail_orig_left = mirror_y(vtail_orig_right);
let vtail_safe_right = translate(vtail_safe_shape, vtail_x, vtail_y, vtail_z);
let vtail_safe_left = mirror_y(vtail_safe_right);

let vtail_orig = union(vtail_orig_right, vtail_orig_left);
let vtail_safe = union(vtail_safe_right, vtail_safe_left);

let vtail_orig_min_feature = min_feature_size_mm(
    wing_with_airfoil("0009", vtail_root_chord, vtail_tip_chord, vtail_span, vtail_sweep_deg, 0.0, 0.0)
);
let vtail_safe_min_feature = min_feature_size_mm(
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
);

let aero_export = #{
    parts: [
        #{
            name: "wing_orig",
            sdf: wing_orig,
            aero_role: "outer_mold_line",
            patch_name: "wing_orig",
            bounds_min_mm: [70.0, -405.0, -30.0],
            bounds_max_mm: [490.0, 405.0, 30.0],
            include_in_modes: ["external"],
            preserve_feature: true,
            feature_tags: ["leading_edge", "trailing_edge"],
            min_export_scale_mm: 0.7,
            min_feature_size_mm: wing_orig_min_feature,
            feature_protection_strength: 3.0
        },
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
            min_feature_size_mm: wing_safe_min_feature,
            feature_protection_strength: 3.0
        },
        #{
            name: "vtail_orig",
            sdf: vtail_orig,
            aero_role: "outer_mold_line",
            patch_name: "vtail_orig",
            bounds_min_mm: [400.0, -130.0, 0.0],
            bounds_max_mm: [540.0, 130.0, 120.0],
            include_in_modes: ["external"],
            preserve_feature: true,
            feature_tags: ["leading_edge", "trailing_edge", "tail_body_junction"],
            min_export_scale_mm: 0.7,
            min_feature_size_mm: vtail_orig_min_feature,
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
            min_feature_size_mm: vtail_safe_min_feature,
            feature_protection_strength: 3.0
        }
    ]
};

union(union(wing_orig, wing_safe), union(vtail_orig, vtail_safe))
