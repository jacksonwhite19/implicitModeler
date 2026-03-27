// Step 1 debug scene:
// - reference fuselage only
// - short dorsal scoop only
// - hollow and open at the mouth
// - explicit stand-off above the fuselage
// - no internal descent / routing yet

let show_fuselage = true;
let show_scoop = true;
let blend_into_fuselage = false;
let show_outer_only = false;

let scoop_clearance = 6.0;
let scoop_length = 54.0;
let wall_thickness = 2.0;

let fuselage = fuselage_elliptical(
    500.0,
    170.0,
    150.0,
    120.0,
    130.0,
    0.72,
    0.62,
    0.70
);

// Short scoop above the fuselage crown.
// The path sits well above the body so the lower lip is visibly offset.
let scoop_path = spline_path([
    [155.0, 0.0, 99.0],
    [168.0, 0.0, 101.0],
    [188.0, 0.0, 101.0],
    [209.0, 0.0, 96.0]
]);

let scoop_outer = profile_duct_solid(
    scoop_path,
    rounded_rect_profile(70.0, 50.0, 8.0),
    rounded_rect_profile(54.0, 34.0, 7.0),
    96
);

let scoop_inner = profile_duct_solid(
    scoop_path,
    rounded_rect_profile(66.0, 46.0, 7.0),
    rounded_rect_profile(50.0, 30.0, 6.0),
    96
);

// Extend the inner body slightly forward so the mouth opens cleanly.
let mouth_cut = translate(
    box_(74.0, 56.0, 28.0),
    147.0,
    0.0,
    101.0
);

let scoop_shell = subtract(
    scoop_outer,
    union(scoop_inner, mouth_cut)
);

let combined = if blend_into_fuselage {
    smooth_union(fuselage, scoop_shell, 4.0)
} else {
    union(fuselage, scoop_shell)
};

if show_fuselage && show_scoop {
    combined
} else if show_fuselage {
    fuselage
} else if show_outer_only {
    scoop_outer
} else {
    scoop_shell
}
