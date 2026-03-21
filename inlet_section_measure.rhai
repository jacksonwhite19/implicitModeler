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
let duct_void = inlet_parts[1];

let face_pt = furthest_point(outer_fairing, -1.0, 0.0, 0.0);
let face_x = face_pt.x;
let top_fuse = surface_point(fuselage, face_x, 0.0, 200.0, 0.0, 0.0, -1.0);

print("face_pt = [" + face_pt.x + ", " + face_pt.y + ", " + face_pt.z + "]");
print("face_x = " + face_x + " mm");
print("fuselage_top_z_at_face_x = " + top_fuse.z + " mm");

let z = 0.0;
let step = 0.5;
let in_outer = false;
let in_void = false;
let outer_start_z = 0.0;
let void_start_z = 0.0;

while z <= 140.0 {
    let d_outer = sdf_distance(outer_fairing, face_x, 0.0, z);
    let d_void = sdf_distance(duct_void, face_x, 0.0, z);

    if !in_outer && d_outer <= 0.0 {
        in_outer = true;
        outer_start_z = z;
    }
    if in_outer && d_outer > 0.0 {
        in_outer = false;
        print("outer_face_material_z_interval = [" + outer_start_z + ", " + z + "] mm");
    }

    if !in_void && d_void <= 0.0 {
        in_void = true;
        void_start_z = z;
    }
    if in_void && d_void > 0.0 {
        in_void = false;
        print("duct_void_z_interval = [" + void_start_z + ", " + z + "] mm");
    }

    z = z + step;
}

if in_outer {
    print("outer_face_material_z_interval = [" + outer_start_z + ", 140.0] mm");
}
if in_void {
    print("duct_void_z_interval = [" + void_start_z + ", 140.0] mm");
}

union(outer_fairing, translate_p(sphere(2.0), top_fuse))
