// Primitive/control-shape topology diagnostics.

let sphere_shape = sphere(100.0);
let rounded_box_shape = smooth_union(box_(180.0, 80.0, 40.0), sphere(10.0), 10.0);
let fuselage_primitive = union(
    cylinder(35.0, 160.0),
    translate(sphere(35.0), 0.0, 0.0, 160.0)
);
let ellipsoid_body = scale(sphere(60.0), 1.6666667, 0.8333333, 0.5);
let airfoil_shape = wing_with_airfoil_export_safe(
    "0012",
    120.0,
    120.0,
    80.0,
    0.0,
    0.0,
    0.0,
    1.6,
    1.0
);

let aero_export = #{
    parts: [
        #{
            name: "sphere_100",
            sdf: sphere_shape,
            aero_role: "outer_mold_line",
            patch_name: "sphere_100",
            bounds_min_mm: [-110.0, -110.0, -110.0],
            bounds_max_mm: [110.0, 110.0, 110.0],
            include_in_modes: ["external"]
        },
        #{
            name: "rounded_box",
            sdf: rounded_box_shape,
            aero_role: "outer_mold_line",
            patch_name: "rounded_box",
            bounds_min_mm: [-100.0, -100.0, -60.0],
            bounds_max_mm: [100.0, 100.0, 60.0],
            include_in_modes: ["external"]
        },
        #{
            name: "fuselage_primitive",
            sdf: fuselage_primitive,
            aero_role: "outer_mold_line",
            patch_name: "fuselage_primitive",
            bounds_min_mm: [-50.0, -50.0, -40.0],
            bounds_max_mm: [50.0, 50.0, 210.0],
            include_in_modes: ["external"]
        },
        #{
            name: "ellipsoid_body",
            sdf: ellipsoid_body,
            aero_role: "outer_mold_line",
            patch_name: "ellipsoid_body",
            bounds_min_mm: [-110.0, -60.0, -35.0],
            bounds_max_mm: [110.0, 60.0, 35.0],
            include_in_modes: ["external"]
        },
        #{
            name: "extruded_airfoil_thick_te",
            sdf: airfoil_shape,
            aero_role: "outer_mold_line",
            patch_name: "extruded_airfoil_thick_te",
            bounds_min_mm: [-10.0, -45.0, -15.0],
            bounds_max_mm: [130.0, 45.0, 15.0],
            include_in_modes: ["external"],
            min_feature_size_mm: min_feature_size_mm(airfoil_shape)
        }
    ]
};

union(union(sphere_shape, rounded_box_shape), union(fuselage_primitive, ellipsoid_body))
