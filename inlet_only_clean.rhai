let samples = 96;

let fuselage = fuselage_elliptical(
    500.0,
    170.0,
    95.0,
    120.0,
    130.0,
    0.72,
    0.62,
    0.70
);

let fairing_guides = [
    [155.0, 0.0, 72.0],
    [185.0, 0.0, 82.0],
    [225.0, 0.0, 82.0],
    [255.0, 0.0, 72.0]
];

let duct_path = spline_path([
    [155.0, 0.0, 72.0],
    [176.0, 0.0, 72.0],
    [204.0, 0.0, 64.0],
    [240.0, 0.0, 42.0],
    [302.0, 0.0, 10.0],
    [370.0, 0.0,  0.0],
    [448.0, 0.0,  0.0]
]);

let outer_start = rounded_rect_profile(70.0, 50.0, 8.0);
let inner_start = rounded_rect_profile(66.0, 46.0, 7.0);
let outer_end = circle_profile(37.0);
let inner_end = circle_profile(35.0);

let inlet_parts = conformal_profile_inlet(
    fuselage,
    fairing_guides,
    duct_path,
    outer_start,
    outer_end,
    inner_start,
    inner_end,
    12.0,
    14.0,
    18.0,
    samples
);

let outer_fairing = inlet_parts[0];
let internal_shell = inlet_parts[2];

union(outer_fairing, internal_shell)
