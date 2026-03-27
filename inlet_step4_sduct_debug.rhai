// Step 4 debug scene:
// - generate one conformal mouth profile
// - reuse that exact saved profile in a fixed-profile S-duct
// - end on the centerline

let show_fuselage = true;
let show_shell = true;

let lip_offset = 10.0;
let fuselage_shell_thickness = 1.6;
let duct_wall_thickness = 2.0;
let mouth_width = 72.0;
let mouth_height = 39.0;
let mouth_corner_radius = 2.0;
let mouth_face_x = 151.0;
let mouth_hold_x = 215.0;

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

let profile_data = conformal_rounded_rect_profile(
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

let mouth_outer_profile = profile_data[0];
let mouth_center_z = profile_data[1];

let inner_profile_data = conformal_rounded_rect_profile(
    fuselage,
    mouth_face_x,
    0.0,
    110.0,
    mouth_width - duct_wall_thickness * 2.0,
    mouth_height - duct_wall_thickness * 2.0,
    0.5,
    lip_offset + duct_wall_thickness,
    96
);

let mouth_inner_profile = inner_profile_data[0];

let duct_path = spline_path([
    [mouth_face_x, 0.0, mouth_center_z],
    [mouth_hold_x, 0.0, mouth_center_z],
    [290.0, 0.0, 76.0],
    [360.0, 0.0, 0.0],
    [430.0, 0.0, 0.0]
]);

let duct_outer = profile_duct_fixed_solid(
    duct_path,
    mouth_outer_profile,
    mouth_outer_profile,
    224
);

let duct_void_path = spline_path([
    [mouth_face_x - 14.0, 0.0, mouth_center_z],
    [mouth_hold_x, 0.0, mouth_center_z],
    [290.0, 0.0, 76.0],
    [360.0, 0.0, 0.0],
    [450.0, 0.0, 0.0]
]);

let duct_void = profile_duct_fixed_solid(
    duct_void_path,
    mouth_inner_profile,
    mouth_inner_profile,
    224
);
let duct_shell = subtract(duct_outer, duct_void);
let fuselage_shell = shell(fuselage, fuselage_shell_thickness);
let fuselage_shell_cut = subtract(fuselage_shell, duct_outer);

if show_fuselage && show_shell {
    union(fuselage_shell_cut, duct_shell)
} else if show_fuselage {
    fuselage_shell_cut
} else {
    duct_shell
}
