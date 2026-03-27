// Step 3 debug scene:
// - exact conformal lower mouth strip
// - simple scoop shell at the mouth
// - short straight rectangular duct aft
// - no internal descent into the fuselage yet

let show_fuselage = true;
let show_shell = true;

let lip_offset = 10.0;
let lip_thickness = 1.5;
let wall_height = 38.0;
let roof_thickness = 6.0;
let wall_thickness = 2.0;
let wall_base_z = 77.0;

let strip_x_center = 180.0;
let strip_x_length = 58.0;
let strip_y_width = 72.0;
let strip_z_height = 18.0;
let strip_z_center = 86.0;

let wall_z_center = wall_base_z + wall_height * 0.5;
let roof_z_center = wall_base_z + wall_height - 2.0;

let duct_x_center = 243.0;
let duct_x_length = 76.0;
let duct_floor_z = 84.0;
let duct_roof_z = roof_z_center;
let duct_wall_height = duct_roof_z - duct_floor_z;
let duct_wall_z_center = duct_floor_z + duct_wall_height * 0.5;

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

// Exact conformal band between two offset OML surfaces.
let outer_iso = offset(fuselage, lip_offset + lip_thickness);
let inner_iso = offset(fuselage, lip_offset);
let oml_band = subtract(outer_iso, inner_iso);

let clip_box = translate(
    box_(strip_x_length, strip_y_width, strip_z_height),
    strip_x_center,
    0.0,
    strip_z_center
);

let lip_strip = intersect(oml_band, clip_box);

// Mouth shell.
let mouth_left_wall = translate(
    box_(strip_x_length, wall_thickness, wall_height),
    strip_x_center,
    -(strip_y_width * 0.5 - wall_thickness * 0.5),
    wall_z_center
);

let mouth_right_wall = translate(
    box_(strip_x_length, wall_thickness, wall_height),
    strip_x_center,
    (strip_y_width * 0.5 - wall_thickness * 0.5),
    wall_z_center
);

let mouth_roof = translate(
    box_(strip_x_length, strip_y_width, roof_thickness),
    strip_x_center,
    0.0,
    roof_z_center
);

// Short straight duct shell aft of the mouth.
let duct_floor = translate(
    box_(duct_x_length, strip_y_width, lip_thickness),
    duct_x_center,
    0.0,
    duct_floor_z
);

let duct_left_wall = translate(
    box_(duct_x_length, wall_thickness, duct_wall_height),
    duct_x_center,
    -(strip_y_width * 0.5 - wall_thickness * 0.5),
    duct_wall_z_center
);

let duct_right_wall = translate(
    box_(duct_x_length, wall_thickness, duct_wall_height),
    duct_x_center,
    (strip_y_width * 0.5 - wall_thickness * 0.5),
    duct_wall_z_center
);

let duct_roof = translate(
    box_(duct_x_length, strip_y_width, roof_thickness),
    duct_x_center,
    0.0,
    roof_z_center
);

// Simple overlap-based handoff between scoop and straight duct.
let shell = union(
    smooth_union(lip_strip, duct_floor, 3.0),
    union(
        mouth_left_wall,
        union(
            mouth_right_wall,
            union(
                mouth_roof,
                union(
                    duct_floor,
                    union(
                        duct_left_wall,
                        union(duct_right_wall, duct_roof)
                    )
                )
            )
        )
    )
);

if show_fuselage && show_shell {
    union(fuselage, shell)
} else if show_fuselage {
    fuselage
} else {
    shell
}
