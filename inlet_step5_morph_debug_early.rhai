// Step 5 debug scene:
// - saved conformal mouth profile
// - morph to a circular duct on a user-controlled schedule

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

let morph_start = 0.00;   // begin morph at 60% of path length
let morph_end = 0.20;     // reach circular section by 90%

let outer_radius = 35.0;
let inner_radius = outer_radius - duct_wall_thickness;

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

let outer_profile_data = conformal_rounded_rect_profile(
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

let mouth_outer_profile = outer_profile_data[0];
let mouth_inner_profile = inner_profile_data[0];
let mouth_center_z = outer_profile_data[1];

let outer_end_profile = circle_profile(outer_radius);
let inner_end_profile = circle_profile(inner_radius);

let duct_path = spline_path([
    [mouth_face_x, 0.0, mouth_center_z],
    [mouth_hold_x, 0.0, mouth_center_z],
    [290.0, 0.0, 76.0],
    [360.0, 0.0, 0.0],
    [430.0, 0.0, 0.0]
]);

let duct_outer = profile_duct_fixed_solid_scheduled(
    duct_path,
    mouth_outer_profile,
    outer_end_profile,
    morph_start,
    morph_end,
    224
);

let duct_void_path = spline_path([
    [mouth_face_x - 14.0, 0.0, mouth_center_z],
    [mouth_hold_x, 0.0, mouth_center_z],
    [290.0, 0.0, 76.0],
    [360.0, 0.0, 0.0],
    [450.0, 0.0, 0.0]
]);

let duct_void = profile_duct_fixed_solid_scheduled(
    duct_void_path,
    mouth_inner_profile,
    inner_end_profile,
    morph_start,
    morph_end,
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
