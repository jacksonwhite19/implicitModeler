// High-level conformal_profile_duct_x debug scene
// Returns [outer_body, duct_void, duct_shell, solved_outer_start_profile,
//          solved_inner_start_profile, mouth_center_point]

let show_fuselage = true;

let fuselage_shell_thickness = 1.6;

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

let parts = conformal_profile_duct_x(
    fuselage,
    rounded_rect_profile(72.0, 39.0, 2.0),
    rounded_rect_profile(68.0, 35.0, 1.0),
    155.0,
    0.0,
    92.0,
    point(1.0, 0.0, 0.0),
    5.0,
    spline_path([
        [155.0, 0.0, 95.0],
        [220.0, 0.0, 95.0],
        [300.0, 0.0, 38.0],
        [360.0, 0.0, 8.0],
        [410.0, 0.0, 0.0],
        [455.0, 0.0, 0.0]
    ]),
    circle_profile(35.0),
    circle_profile(33.0),
    18.0,
    42.0,
    0.10,
    0.90,
    192
);

let outer_body = parts[0];
let duct_void = parts[1];
let duct_shell = parts[2];

let fuselage_shell = shell(fuselage, fuselage_shell_thickness);
let fuselage_shell_cut = subtract(fuselage_shell, duct_void);

if show_fuselage {
    union(fuselage_shell_cut, duct_shell)
} else {
    union(outer_body, duct_shell)
}
