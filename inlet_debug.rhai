// Debug-only custom-profile S-duct using the refined centerline-distance kernel.
// Rounded-rectangle outer mouth and custom inner mouth transitioning to a circular outlet.

let wall_thickness = 2.0;
let smoothness = 0.98;
let samples = 160;

let duct_path = spline_path([
    [0.0,   0.0, 46.0],
    [20.0,  0.0, 52.0],
    [56.0,  0.0, 58.0],
    [108.0, 0.0, 60.0],
    [164.0, 0.0, 48.0],
    [218.0, 0.0, 24.0],
    [266.0, 0.0,  8.0],
    [312.0, 0.0,  0.0],
    [430.0, 0.0,  0.0]
]);

let outer_start = rounded_rect_profile(100.0, 76.0, 14.0);
let inner_start = custom_profile([
    [48.0,  0.0],
    [30.0, 22.0],
    [ 0.0, 28.0],
    [-30.0, 22.0],
    [-48.0,  0.0],
    [-20.0,-18.0],
    [ 20.0,-18.0]
]);
let outer_end = circle_profile(47.0);
let inner_end = circle_profile(45.0);

profile_duct(
    duct_path,
    outer_start,
    outer_end,
    inner_start,
    inner_end,
    70.0,
    70.0,
    samples
)
