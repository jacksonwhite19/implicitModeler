// Aircraft with underwing engine nacelles

// Fuselage
let fuselage = fuselage_parametric(60.0, 8.0, 0.7, 0.5);

// Main wing
let wing = wing_with_airfoil("2412", 12.0, 6.0, 40.0, 8.0, 2.0, -1.5);
let wing = translate(wing, 25.0, 0.0, 0.0);

// Horizontal tail
let h_tail = wing_with_airfoil("0012", 6.0, 3.0, 15.0, 5.0, 0.0, 0.0);
let h_tail = translate(h_tail, 52.0, 0.0, 0.0);

// Vertical tail
let v_tail = wing_with_airfoil("0009", 8.0, 4.0, 8.0, 15.0, 0.0, 0.0);
let v_tail = rotate(v_tail, 0.0, 0.0, 90.0);
let v_tail = translate(v_tail, 52.0, 0.0, 5.0);

// Engine nacelles
let nacelle_left = nacelle(
    8.0,        // length
    2.2,        // diameter
    2.0,        // inlet diameter
    1.8         // exhaust diameter
);
let nacelle_left = translate(nacelle_left, 28.0, -3.0, -10.0);  // Under left wing

let nacelle_right = nacelle(8.0, 2.2, 2.0, 1.8);
let nacelle_right = translate(nacelle_right, 28.0, 3.0, -10.0);  // Under right wing

// Assemble aircraft with blending
let aircraft = blend(fuselage, wing, 2.5);
let aircraft = blend(aircraft, h_tail, 1.5);
let aircraft = blend(aircraft, v_tail, 1.5);
let aircraft = blend(aircraft, nacelle_left, 1.0);
let aircraft = blend(aircraft, nacelle_right, 1.0);

let aero_export = #{
    parts: [
        #{
            name: "fuselage_oml",
            sdf: fuselage,
            aero_role: "outer_mold_line",
            patch_name: "fuselage"
        },
        #{
            name: "main_wing_oml",
            sdf: wing,
            aero_role: "outer_mold_line",
            patch_name: "main_wing",
            preserve_feature: true,
            feature_tags: ["leading_edge", "trailing_edge", "wing_body_junction"],
            min_export_scale_mm: 0.8,
            feature_protection_strength: 2.0
        },
        #{
            name: "horizontal_tail_oml",
            sdf: h_tail,
            aero_role: "outer_mold_line",
            patch_name: "horizontal_tail",
            preserve_feature: true,
            feature_tags: ["leading_edge", "trailing_edge", "tail_body_junction"],
            min_export_scale_mm: 0.8,
            feature_protection_strength: 1.5
        },
        #{
            name: "vertical_tail_oml",
            sdf: v_tail,
            aero_role: "outer_mold_line",
            patch_name: "vertical_tail",
            preserve_feature: true,
            feature_tags: ["leading_edge", "trailing_edge", "tail_body_junction"],
            min_export_scale_mm: 0.8,
            feature_protection_strength: 1.5
        },
        #{
            name: "left_nacelle_oml",
            sdf: nacelle_left,
            aero_role: "outer_mold_line",
            patch_name: "left_nacelle",
            preserve_feature: true,
            feature_tags: ["inlet_lip", "nozzle_exit"],
            min_export_scale_mm: 0.5,
            feature_protection_strength: 2.0
        },
        #{
            name: "right_nacelle_oml",
            sdf: nacelle_right,
            aero_role: "outer_mold_line",
            patch_name: "right_nacelle",
            preserve_feature: true,
            feature_tags: ["inlet_lip", "nozzle_exit"],
            min_export_scale_mm: 0.5,
            feature_protection_strength: 2.0
        }
    ]
};

aircraft
