// Complete aircraft assembly with fuselage, wings, and tail

// Create fuselage
let fuselage = fuselage_parametric(
    60.0,   // length
    8.0,    // max diameter
    0.7,    // nose shape (0=pointed, 1=rounded)
    0.5     // tail shape
);

// Create main wing
let wing = wing_with_airfoil(
    "2412",     // Cambered airfoil for lift
    12.0,       // root chord
    6.0,        // tip chord
    40.0,       // span
    8.0,        // sweep (degrees)
    2.0,        // dihedral (degrees)
    -1.5        // washout for stability
);
let wing = translate(wing, 25.0, 0.0, 0.0);
let wing_le_path = [
    leading_edge(wing, 0.0),
    leading_edge(wing, 0.2),
    leading_edge(wing, 0.4),
    leading_edge(wing, 0.6),
    leading_edge(wing, 0.8),
    leading_edge(wing, 1.0)
];
let wing_te_path = [
    trailing_edge(wing, 0.0),
    trailing_edge(wing, 0.2),
    trailing_edge(wing, 0.4),
    trailing_edge(wing, 0.6),
    trailing_edge(wing, 0.8),
    trailing_edge(wing, 1.0)
];
let wing_root_blend_path = [
    chord_point(wing, 0.03, 0.05),
    chord_point(wing, 0.03, 0.25),
    chord_point(wing, 0.03, 0.50),
    chord_point(wing, 0.03, 0.75),
    chord_point(wing, 0.03, 0.95)
];

// Create horizontal stabilizer
let h_tail = wing_with_airfoil(
    "0012",     // Symmetric airfoil
    6.0,        // root chord
    3.0,        // tip chord
    15.0,       // span
    5.0,        // slight sweep
    0.0,        // no dihedral
    0.0         // no twist
);
let h_tail = translate(h_tail, 52.0, 0.0, 0.0);
let h_tail_le_path = [
    leading_edge(h_tail, 0.0),
    leading_edge(h_tail, 0.33),
    leading_edge(h_tail, 0.66),
    leading_edge(h_tail, 1.0)
];
let h_tail_te_path = [
    trailing_edge(h_tail, 0.0),
    trailing_edge(h_tail, 0.33),
    trailing_edge(h_tail, 0.66),
    trailing_edge(h_tail, 1.0)
];
let h_tail_root_blend_path = [
    chord_point(h_tail, 0.05, 0.1),
    chord_point(h_tail, 0.05, 0.5),
    chord_point(h_tail, 0.05, 0.9)
];

// Create vertical stabilizer (rotated horizontal surface)
let v_tail = wing_with_airfoil(
    "0009",     // Thin symmetric airfoil
    8.0,        // root chord
    4.0,        // tip chord
    8.0,        // height (span when rotated)
    15.0,       // sweep
    0.0,        // N/A for vertical
    0.0         // no twist
);
let v_tail = rotate(v_tail, 0.0, 0.0, 90.0);  // Rotate to vertical
let v_tail = translate(v_tail, 52.0, 0.0, 5.0);
let v_tail_le_path = [
    leading_edge(v_tail, 0.0),
    leading_edge(v_tail, 0.33),
    leading_edge(v_tail, 0.66),
    leading_edge(v_tail, 1.0)
];
let v_tail_te_path = [
    trailing_edge(v_tail, 0.0),
    trailing_edge(v_tail, 0.33),
    trailing_edge(v_tail, 0.66),
    trailing_edge(v_tail, 1.0)
];
let v_tail_root_blend_path = [
    chord_point(v_tail, 0.05, 0.1),
    chord_point(v_tail, 0.05, 0.5),
    chord_point(v_tail, 0.05, 0.9)
];

// Blend everything together with smooth fillets
let aircraft = blend(fuselage, wing, 2.5);
let aircraft = blend(aircraft, h_tail, 1.5);
let aircraft = blend(aircraft, v_tail, 1.5);

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
            feature_protection_strength: 2.0,
            refinement_paths: [
                #{
                    points_mm: wing_le_path,
                    radii_mm: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
                    target_cell_mm: 0.5
                },
                #{
                    points_mm: wing_te_path,
                    radii_mm: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
                    target_cell_mm: 0.5
                },
                #{
                    points_mm: wing_root_blend_path,
                    radii_mm: [3.0, 3.0, 3.0, 3.0, 3.0],
                    target_cell_mm: 0.75
                }
            ]
        },
        #{
            name: "horizontal_tail_oml",
            sdf: h_tail,
            aero_role: "outer_mold_line",
            patch_name: "horizontal_tail",
            preserve_feature: true,
            feature_tags: ["leading_edge", "trailing_edge", "tail_body_junction"],
            min_export_scale_mm: 0.8,
            feature_protection_strength: 1.5,
            refinement_paths: [
                #{
                    points_mm: h_tail_le_path,
                    radii_mm: [1.5, 1.5, 1.5, 1.5],
                    target_cell_mm: 0.6
                },
                #{
                    points_mm: h_tail_te_path,
                    radii_mm: [1.5, 1.5, 1.5, 1.5],
                    target_cell_mm: 0.6
                },
                #{
                    points_mm: h_tail_root_blend_path,
                    radii_mm: [2.0, 2.0, 2.0],
                    target_cell_mm: 0.8
                }
            ]
        },
        #{
            name: "vertical_tail_oml",
            sdf: v_tail,
            aero_role: "outer_mold_line",
            patch_name: "vertical_tail",
            preserve_feature: true,
            feature_tags: ["leading_edge", "trailing_edge", "tail_body_junction"],
            min_export_scale_mm: 0.8,
            feature_protection_strength: 1.5,
            refinement_paths: [
                #{
                    points_mm: v_tail_le_path,
                    radii_mm: [1.5, 1.5, 1.5, 1.5],
                    target_cell_mm: 0.6
                },
                #{
                    points_mm: v_tail_te_path,
                    radii_mm: [1.5, 1.5, 1.5, 1.5],
                    target_cell_mm: 0.6
                },
                #{
                    points_mm: v_tail_root_blend_path,
                    radii_mm: [2.0, 2.0, 2.0],
                    target_cell_mm: 0.8
                }
            ]
        }
    ]
};

aircraft
