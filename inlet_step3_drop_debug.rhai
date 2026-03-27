// Step 3 debug scene:
// - generate the mouth section once as a conformal custom profile
// - use that exact same profile for the straight mouth segment
// - use that exact same profile again for the drop segment
// - no shape morphing yet

let show_fuselage = false;
let show_shell = true;

let lip_offset = 10.0;

let fuselage = fuselage_elliptical(
    500.0,
    170.0,
    170.0,
    120.0,
    130.0,
    0.72,
    0.62,
    0.70
);

let mouth_face_x = 151.0;
let handoff_x = 209.0;
let mouth_width = 72.0;
let mouth_height = 39.0;
let mouth_corner_radius = 2.0;

let mouth_section_data = conformal_rounded_rect_profile(
    fuselage,
    mouth_face_x,
    0.0,
    110.0,
    mouth_width,
    mouth_height,
    mouth_corner_radius,
    lip_offset,
    96
);

let mouth_profile = mouth_section_data[0];
let mouth_center_z = mouth_section_data[1];

let mouth_path = polyline_path([
    [mouth_face_x, 0.0, mouth_center_z],
    [handoff_x, 0.0, mouth_center_z]
]);

let mouth_solid = profile_duct_fixed_solid(
    mouth_path,
    mouth_profile,
    mouth_profile,
    96
);

let continuation_path = spline_path([
    [handoff_x, 0.0, mouth_center_z],
    [240.0, 0.0, mouth_center_z],
    [270.0, 0.0, 92.0],
    [300.0, 0.0, 72.0],
    [340.0, 0.0, 40.0]
]);

let continuation_solid = profile_duct_fixed_solid(
    continuation_path,
    mouth_profile,
    mouth_profile,
    128
);

let shell = union(mouth_solid, continuation_solid);

if show_fuselage && show_shell {
    union(fuselage, shell)
} else if show_fuselage {
    fuselage
} else {
    shell
}
