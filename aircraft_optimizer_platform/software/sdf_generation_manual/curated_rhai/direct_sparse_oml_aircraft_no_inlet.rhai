// Simple aircraft, no inlet, no internal structure.
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

let min_te_thickness = 1.5;
let min_le_radius = 2.25;

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
let vtail_root_chord = 160.0;
let vtail_tip_chord = 66.0;
let vtail_sweep_deg = 35.0;
let vtail_le_x = 500.0;

let local_refine_cell = 0.75;
let wing_sweep_tan = 0.26794919243; // tan(15 deg)
let vtail_sweep_tan = 0.70020753821; // tan(35 deg)

// ---------------------------------------------------------------------------
// Main airframe
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

let wing_half_span = wing_span * 0.5;
let wing_tip_le_x = wing_le_x + wing_sweep_tan * wing_half_span;
let wing_root_te_x = wing_le_x + wing_root_chord;
let wing_tip_te_x = wing_tip_le_x + wing_tip_chord;

// Diagnostic slice-frame variants.
// Rotating +90 degrees about X maps old spanwise Y into tracer Z, so the
// existing Z-slice contour pipeline can run spanwise sections without using DC.
let aircraft_oml_y_slice_frame = rotate(body_outer, 90.0, 0.0, 0.0);

// Diagnostic oblique slice-frame variants.
// Y-slice frame maps aircraft Y into tracer Z. Pre-yawing by +30 degrees maps
// aircraft (X/Y) onto an oblique slice coordinate, so tracer Z becomes:
// aircraft_x * sin(30deg) + aircraft_y * cos(30deg).
let aircraft_oml_oblique30_y_slice_frame = rotate(rotate(body_outer, 0.0, 0.0, 30.0), 90.0, 0.0, 0.0);
let aircraft_oml_oblique15_y_slice_frame = rotate(rotate(body_outer, 0.0, 0.0, 15.0), 90.0, 0.0, 0.0);

// ---------------------------------------------------------------------------
// Export metadata
// ---------------------------------------------------------------------------

let htail_half_span = htail_span * 0.5;
let htail_root_te_x = htail_le_x + htail_root_chord;
let htail_tip_te_x = htail_le_x + htail_tip_chord;

let vtail_half_span = vtail_span * 0.5;
let vtail_tip_le_x = vtail_le_x + vtail_sweep_tan * vtail_half_span;
let vtail_root_te_x = vtail_le_x + vtail_root_chord;
let vtail_tip_te_x = vtail_tip_le_x + vtail_tip_chord;

let aircraft_refinement_paths = [
    #{
        points_mm: [[wing_tip_le_x, -wing_half_span, 0.0], [wing_le_x, 0.0, 0.0], [wing_tip_le_x, wing_half_span, 0.0]],
        radii_mm: [7.0, 9.0, 7.0],
        target_cell_mm: local_refine_cell
    },
    #{
        points_mm: [[wing_tip_te_x, -wing_half_span, 0.0], [wing_root_te_x, 0.0, 0.0], [wing_tip_te_x, wing_half_span, 0.0]],
        radii_mm: [8.0, 10.0, 8.0],
        target_cell_mm: local_refine_cell
    },
    #{
        points_mm: [[htail_le_x, -htail_half_span, tail_z], [htail_le_x, 0.0, tail_z], [htail_le_x, htail_half_span, tail_z]],
        radii_mm: [5.0, 7.0, 5.0],
        target_cell_mm: local_refine_cell
    },
    #{
        points_mm: [[htail_tip_te_x, -htail_half_span, tail_z], [htail_root_te_x, 0.0, tail_z], [htail_tip_te_x, htail_half_span, tail_z]],
        radii_mm: [5.0, 7.0, 5.0],
        target_cell_mm: local_refine_cell
    },
    #{
        points_mm: [[vtail_le_x, 0.0, tail_z], [vtail_tip_le_x, 0.0, tail_z + vtail_half_span]],
        radii_mm: [6.0, 5.0],
        target_cell_mm: local_refine_cell
    },
    #{
        points_mm: [[vtail_root_te_x, 0.0, tail_z], [vtail_tip_te_x, 0.0, tail_z + vtail_half_span]],
        radii_mm: [6.0, 5.0],
        target_cell_mm: local_refine_cell
    }
];

let aero_parts = [
    #{
        name: "aircraft_oml",
        sdf: body_outer,
        aero_role: "outer_mold_line",
        patch_name: "aircraft_no_inlet",
        bounds_min_mm: [-20.0, -360.0, -40.0],
        bounds_max_mm: [740.0, 360.0, 120.0],
        include_in_modes: ["external", "full_fluid_boundary"],
        preserve_feature: true,
        feature_tags: ["leading_edge", "trailing_edge", "wing_body_junction", "tail_body_junction"],
        min_export_scale_mm: 0.8,
        min_feature_size_mm: min_te_thickness,
        feature_protection_strength: 3.0,
        refinement_body_sdf: body_outer,
        refinement_paths: aircraft_refinement_paths,
        priority: 0
    },
    #{
        name: "aircraft_oml_y_slice_frame",
        sdf: aircraft_oml_y_slice_frame,
        aero_role: "outer_mold_line",
        patch_name: "aircraft_no_inlet_y_slice_frame",
        bounds_min_mm: [-28.0, -160.0, -373.0],
        bounds_max_mm: [748.0, 160.0, 373.0],
        include_in_modes: ["diagnostic"],
        preserve_feature: true,
        feature_tags: ["diagnostic_y_slice_frame", "no_inlet", "no_internal_structure"],
        min_export_scale_mm: 0.8,
        min_feature_size_mm: min_te_thickness,
        feature_protection_strength: 3.0,
        refinement_body_sdf: aircraft_oml_y_slice_frame,
        priority: 3
    },
    #{
        name: "aircraft_oml_oblique30_y_slice_frame",
        sdf: aircraft_oml_oblique30_y_slice_frame,
        aero_role: "outer_mold_line",
        patch_name: "aircraft_no_inlet_oblique30_y_slice_frame",
        bounds_min_mm: [-210.0, -160.0, -395.0],
        bounds_max_mm: [820.0, 160.0, 690.0],
        include_in_modes: ["diagnostic"],
        preserve_feature: true,
        feature_tags: ["diagnostic_oblique_y_slice_frame", "no_inlet", "no_internal_structure"],
        min_export_scale_mm: 0.8,
        min_feature_size_mm: min_te_thickness,
        feature_protection_strength: 3.0,
        refinement_body_sdf: aircraft_oml_oblique30_y_slice_frame,
        priority: 6
    },
    #{
        name: "aircraft_oml_oblique15_y_slice_frame",
        sdf: aircraft_oml_oblique15_y_slice_frame,
        aero_role: "outer_mold_line",
        patch_name: "aircraft_no_inlet_oblique15_y_slice_frame",
        bounds_min_mm: [-130.0, -160.0, -380.0],
        bounds_max_mm: [830.0, 160.0, 570.0],
        include_in_modes: ["diagnostic"],
        preserve_feature: true,
        feature_tags: ["diagnostic_oblique_y_slice_frame", "no_inlet", "no_internal_structure"],
        min_export_scale_mm: 0.8,
        min_feature_size_mm: min_te_thickness,
        feature_protection_strength: 3.0,
        refinement_body_sdf: aircraft_oml_oblique15_y_slice_frame,
        priority: 8
    }
];

let aero_export = #{
    parts: aero_parts
};

// Native marching-cubes solid OML SDF for backend/CFD export.
// This intentionally avoids legacy y-slice/oblique diagnostic frames.
body_outer
