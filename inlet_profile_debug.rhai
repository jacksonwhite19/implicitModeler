// Profile debug scene:
// - generate one conformal mouth profile
// - extrude it straight for 10 mm
// - compare directly against the fuselage in section cuts

let show_fuselage = true;
let show_shell = true;

let lip_offset = 10.0;
let mouth_width = 72.0;
let mouth_height = 39.0;
let mouth_corner_radius = 2.0;
let mouth_face_x = 151.0;
let mouth_length = 10.0;

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

let mouth_profile = profile_data[0];
let mouth_center_z = profile_data[1];

let shell = profile_duct_fixed_solid(
    polyline_path([
        [mouth_face_x, 0.0, mouth_center_z],
        [mouth_face_x + mouth_length, 0.0, mouth_center_z]
    ]),
    mouth_profile,
    mouth_profile,
    96
);

if show_fuselage && show_shell {
    union(fuselage, shell)
} else if show_fuselage {
    fuselage
} else {
    shell
}
