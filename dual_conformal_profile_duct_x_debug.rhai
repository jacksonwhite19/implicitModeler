// Mirrored dual conformal profile duct debug scene
// Demonstrates a bifurcated intake pair merging toward a centerline duct.

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

let parts = mirrored_dual_conformal_profile_duct_x(
    fuselage,
    rounded_rect_profile(50.0, 28.0, 4.0),
    rounded_rect_profile(46.0, 24.0, 3.0),
    155.0,
    55.0,
    55.0,
    point(1.0, 0.0, 0.0),
    spline_path([
        [155.0, 55.0, 62.0],
        [230.0, 44.0, 50.0],
        [300.0, 20.0, 20.0],
        [350.0, 8.0, 8.0],
        [390.0, 0.0, 0.0],
        [470.0, 0.0, 0.0]
    ]),
    circle_profile(28.0),
    circle_profile(26.0),
    10.0,
    18.0,
    90.0,
    0.15,
    0.75,
    192
);

let outer_body = parts[0];
let duct_void = parts[1];
let duct_shell = parts[2];

let fuselage_shell = shell(fuselage, fuselage_shell_thickness);
let fuselage_shell_cut = subtract(fuselage_shell, outer_body);

if show_fuselage {
    union(fuselage_shell_cut, duct_shell)
} else {
    union(outer_body, duct_shell)
}
