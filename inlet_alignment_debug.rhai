// Inlet / fuselage alignment debug scene.
// Purpose:
// - quantify whether the lower OUTER fairing surface is actually outside the fuselage
// - provide named probe points and simple physical rulers for side/front inspection
//
// Suggested use:
// 1. Snap to Right view to inspect the X/Z ruler and probe spacing.
// 2. Snap to Front view to inspect the Y/Z mouth shape.
// 3. Use the section tool if needed, but the printed clearances below should already
//    tell you if each probe is outside (+) or penetrating (-) the fuselage.

let shell_thickness = 1.6;
let samples = 96;
let probe_ball_r = 2.6;

let show_airframe = true;
let show_outer_fairing = true;
let show_internal_shell = true;
let show_duct_void = false;
let show_probe_markers = true;
let show_rulers = true;

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
    [170.0, 0.0, 72.0],
    [194.0, 0.0, 68.0],
    [228.0, 0.0, 52.0],
    [286.0, 0.0, 18.0],
    [360.0, 0.0,  0.0],
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
let duct_void = inlet_parts[1];
let internal_shell = inlet_parts[2];

let airframe_shell = shell(fuselage, shell_thickness);
let inlet_airframe = subtract(
    union(airframe_shell, union(outer_fairing, internal_shell)),
    duct_void
);

// Probe the lower outer fairing surface from below at several X stations.
let probe_x0 = 160.0;
let probe_x1 = 180.0;
let probe_x2 = 200.0;
let probe_x3 = 225.0;

let probe_outer_0 = ref_point("outer_160", surface_point(outer_fairing, probe_x0, 0.0, 20.0, 0.0, 0.0, 1.0));
let probe_outer_1 = ref_point("outer_180", surface_point(outer_fairing, probe_x1, 0.0, 20.0, 0.0, 0.0, 1.0));
let probe_outer_2 = ref_point("outer_200", surface_point(outer_fairing, probe_x2, 0.0, 20.0, 0.0, 0.0, 1.0));
let probe_outer_3 = ref_point("outer_225", surface_point(outer_fairing, probe_x3, 0.0, 20.0, 0.0, 0.0, 1.0));

let probe_fuse_0 = ref_point("fuse_160", closest_point_p(fuselage, probe_outer_0));
let probe_fuse_1 = ref_point("fuse_180", closest_point_p(fuselage, probe_outer_1));
let probe_fuse_2 = ref_point("fuse_200", closest_point_p(fuselage, probe_outer_2));
let probe_fuse_3 = ref_point("fuse_225", closest_point_p(fuselage, probe_outer_3));

let clearance_0 = sdf_distance_p(fuselage, probe_outer_0);
let clearance_1 = sdf_distance_p(fuselage, probe_outer_1);
let clearance_2 = sdf_distance_p(fuselage, probe_outer_2);
let clearance_3 = sdf_distance_p(fuselage, probe_outer_3);

print("outer_160 signed clearance to fuselage = " + clearance_0 + " mm");
print("outer_180 signed clearance to fuselage = " + clearance_1 + " mm");
print("outer_200 signed clearance to fuselage = " + clearance_2 + " mm");
print("outer_225 signed clearance to fuselage = " + clearance_3 + " mm");
print("Positive = outside, negative = penetrating the fuselage.");

let probe_markers =
    union(
        translate_p(sphere(probe_ball_r), probe_outer_0),
        union(
            translate_p(sphere(probe_ball_r), probe_outer_1),
            union(
                translate_p(sphere(probe_ball_r), probe_outer_2),
                union(
                    translate_p(sphere(probe_ball_r), probe_outer_3),
                    union(
                        translate_p(sphere(probe_ball_r), probe_fuse_0),
                        union(
                            translate_p(sphere(probe_ball_r), probe_fuse_1),
                            union(
                                translate_p(sphere(probe_ball_r), probe_fuse_2),
                                translate_p(sphere(probe_ball_r), probe_fuse_3)
                            )
                        )
                    )
                )
            )
        )
    );

// Side-view X/Z ruler below the model.
let side_ruler_base = translate(box_(340.0, 2.0, 2.0), 250.0, -95.0, -24.0);
let side_tick_140 = translate(box_(2.0, 2.0, 18.0), 140.0, -95.0, -24.0);
let side_tick_160 = translate(box_(2.0, 2.0, 24.0), 160.0, -95.0, -24.0);
let side_tick_180 = translate(box_(2.0, 2.0, 24.0), 180.0, -95.0, -24.0);
let side_tick_200 = translate(box_(2.0, 2.0, 24.0), 200.0, -95.0, -24.0);
let side_tick_220 = translate(box_(2.0, 2.0, 24.0), 220.0, -95.0, -24.0);
let side_tick_240 = translate(box_(2.0, 2.0, 24.0), 240.0, -95.0, -24.0);

// Front-view Y/Z mouth grid slightly ahead of the inlet face.
let mouth_grid_x = 146.0;
let front_v0 = translate(box_(2.0, 2.0, 110.0), mouth_grid_x, -40.0, 65.0);
let front_v1 = translate(box_(2.0, 2.0, 110.0), mouth_grid_x, -20.0, 65.0);
let front_v2 = translate(box_(2.0, 2.0, 110.0), mouth_grid_x,   0.0, 65.0);
let front_v3 = translate(box_(2.0, 2.0, 110.0), mouth_grid_x,  20.0, 65.0);
let front_v4 = translate(box_(2.0, 2.0, 110.0), mouth_grid_x,  40.0, 65.0);
let front_h0 = translate(box_(2.0, 90.0, 2.0), mouth_grid_x, 0.0, 30.0);
let front_h1 = translate(box_(2.0, 90.0, 2.0), mouth_grid_x, 0.0, 50.0);
let front_h2 = translate(box_(2.0, 90.0, 2.0), mouth_grid_x, 0.0, 70.0);
let front_h3 = translate(box_(2.0, 90.0, 2.0), mouth_grid_x, 0.0, 90.0);

let rulers =
    union(
        side_ruler_base,
        union(
            side_tick_140,
            union(
                side_tick_160,
                union(
                    side_tick_180,
                    union(
                        side_tick_200,
                        union(
                            side_tick_220,
                            union(
                                side_tick_240,
                                union(
                                    front_v0,
                                    union(
                                        front_v1,
                                        union(
                                            front_v2,
                                            union(
                                                front_v3,
                                                union(
                                                    front_v4,
                                                    union(
                                                        front_h0,
                                                        union(front_h1, union(front_h2, front_h3))
                                                    )
                                                )
                                            )
                                        )
                                    )
                                )
                            )
                        )
                    )
                )
            )
        )
    );

ref_point("tick_x160", point(160.0, -95.0, -6.0));
ref_point("tick_x200", point(200.0, -95.0, -6.0));
ref_point("grid_z50", point(mouth_grid_x, 48.0, 50.0));
ref_point("grid_z70", point(mouth_grid_x, 48.0, 70.0));

let scene =
    if show_airframe {
        inlet_airframe
    } else if show_outer_fairing && show_internal_shell {
        union(outer_fairing, internal_shell)
    } else if show_outer_fairing {
        outer_fairing
    } else if show_internal_shell {
        internal_shell
    } else if show_duct_void {
        duct_void
    } else {
        outer_fairing
    };

let scene = if show_probe_markers {
    union(scene, probe_markers)
} else {
    scene
};

if show_rulers {
    union(scene, rulers)
} else {
    scene
}
