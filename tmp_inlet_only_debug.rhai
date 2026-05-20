let body_length = 500.0;
let body_height = 95.0;
let shell_thickness = 1.6;

let inlet_x = body_length * 0.35;
let inlet_width = 70.0;
let inlet_height = 50.0;
let inlet_wall = shell_thickness;
let inlet_clearance = 10.0;
let duct_diameter = 70.0;
let duct_samples = 160;
let duct_start_z = body_height * 0.5 + inlet_height * 0.5 + inlet_clearance;

let duct_path = spline_path([
    [inlet_x - 6.0,  0.0, duct_start_z],
    [inlet_x + 10.0, 0.0, duct_start_z],
    [inlet_x + 28.0, 0.0, duct_start_z - 2.0],
    [inlet_x + 54.0, 0.0, duct_start_z - 8.0],
    [inlet_x + 88.0, 0.0, duct_start_z - 20.0],
    [inlet_x + 122.0, 0.0, 28.0],
    [inlet_x + 156.0, 0.0, 8.0],
    [inlet_x + 186.0, 0.0, 0.0],
    [body_length - 12.0, 0.0, 0.0]
]);

let outer_start = rounded_rect_profile(inlet_height, inlet_width, inlet_height * 0.24);
let outer_end = circle_profile((duct_diameter + inlet_wall * 2.0) * 0.5);
let inlet_only = profile_duct_solid(duct_path, outer_start, outer_end, duct_samples);

let aero_export = #{
    parts: [
        #{
            name: "inlet_only",
            sdf: inlet_only,
            aero_role: "outer_mold_line",
            patch_name: "inlet_only",
            bounds_min_mm: [150.0, -52.0, -10.0],
            bounds_max_mm: [560.0, 52.0, 110.0],
            include_in_modes: ["external"],
            preserve_feature: true,
            feature_tags: ["inlet_lip"],
            min_export_scale_mm: 0.5,
            feature_protection_strength: 2.5
        }
    ]
};

inlet_only
