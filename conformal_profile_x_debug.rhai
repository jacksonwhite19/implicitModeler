// conformal_profile_x debug scene
// Shows three saved conformal mouth profiles extruded straight:
// 1. dorsal trapezoid
// 2. circular cheek inlet
// 3. rounded-rectangle side inlet

let show_fuselage = true;
let show_profiles = true;
let mouth_length = 12.0;

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

let trap_profile = custom_profile([
    [-28.0, -18.0],
    [ 28.0, -18.0],
    [ 22.0,  18.0],
    [-22.0,  18.0]
]);

let dorsal_data = conformal_profile_x(
    fuselage,
    trap_profile,
    145.0,
    0.0,
    112.0,
    point(1.0, 0.0, 0.0),
    5.0,
    96
);
let dorsal_profile = dorsal_data[0];
let dorsal_center = dorsal_data[1];
let dorsal_mouth = profile_duct_fixed_solid(
    polyline_path([
        [dorsal_center.x, dorsal_center.y, dorsal_center.z],
        [dorsal_center.x + mouth_length, dorsal_center.y, dorsal_center.z]
    ]),
    dorsal_profile,
    dorsal_profile,
    96
);

let cheek_data = conformal_profile_x(
    fuselage,
    circle_profile(18.0),
    255.0,
    92.0,
    40.0,
    point(1.0, 0.0, 0.0),
    4.0,
    96
);
let cheek_profile = cheek_data[0];
let cheek_center = cheek_data[1];
let cheek_mouth = profile_duct_fixed_solid(
    polyline_path([
        [cheek_center.x, cheek_center.y, cheek_center.z],
        [cheek_center.x + mouth_length, cheek_center.y, cheek_center.z]
    ]),
    cheek_profile,
    cheek_profile,
    96
);

let side_data = conformal_profile_x(
    fuselage,
    rounded_rect_profile(46.0, 30.0, 6.0),
    345.0,
    -88.0,
    28.0,
    point(1.0, 0.0, 0.0),
    4.0,
    96
);
let side_profile = side_data[0];
let side_center = side_data[1];
let side_mouth = profile_duct_fixed_solid(
    polyline_path([
        [side_center.x, side_center.y, side_center.z],
        [side_center.x + mouth_length, side_center.y, side_center.z]
    ]),
    side_profile,
    side_profile,
    96
);

let mouths = union(
    union(dorsal_mouth, cheek_mouth),
    side_mouth
);

if show_fuselage && show_profiles {
    union(fuselage, mouths)
} else if show_fuselage {
    fuselage
} else {
    mouths
}
