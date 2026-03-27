// Debug script for the robust conformal-profile inlet system.
// Builds:
// - a simple elliptical fuselage
// - a dorsal conformal fairing whose underside matches an offset OML
// - an internal duct void
// - a solid final body with the inlet carved through it

let samples = 96;
let oml_offset = 12.0;
let face_clearance = 2.0;
let inlet_open_extension = 14.0;
let outlet_open_extension = 32.0;

let show_airframe = true;
let show_fairing = true;
let show_internal_shell = false;
let show_void = false;

let fuselage = fuselage_elliptical(
    500.0,  // length
    170.0,  // width
    150.0,  // height
    120.0,  // nose length
    130.0,  // tail length
    0.72,   // nose bluntness
    0.62,   // tail bluntness
    0.70    // fairing smoothness
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
    oml_offset,
    face_clearance,
    inlet_open_extension,
    outlet_open_extension,
    samples
);

let outer_fairing = inlet_parts[0];
let duct_void = inlet_parts[1];
let internal_shell = inlet_parts[2];

let outer_body = smooth_union(fuselage, outer_fairing, 6.0);
let inlet_airframe = subtract(
    outer_body,
    duct_void
);

if show_airframe {
    inlet_airframe
} else if show_fairing && show_internal_shell {
    union(outer_fairing, internal_shell)
} else if show_fairing {
    outer_fairing
} else if show_internal_shell {
    internal_shell
} else if show_void {
    duct_void
} else {
    outer_fairing
}
