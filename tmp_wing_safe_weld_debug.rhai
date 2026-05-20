let span = 800.0;
let root_chord = 310.0;
let tip_chord = 62.0;
let root_le_x = 92.0;
let wing_sweep_deg = 39.0;

let export_safe_te_mm = 1.6;
let export_safe_le_mm = 1.0;

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
        }
    ]
};

wing_safe
