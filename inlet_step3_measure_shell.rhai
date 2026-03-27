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
let bridge_x_center = 232.0;
let bridge_x_length = 48.0;

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

let bridge_floor = translate(
    box_(bridge_x_length, strip_y_width, lip_thickness),
    bridge_x_center,
    0.0,
    wall_base_z + lip_thickness * 0.5
);

let bridge_left_wall = translate(
    box_(bridge_x_length, wall_thickness, wall_height),
    bridge_x_center,
    -(strip_y_width * 0.5 - wall_thickness * 0.5),
    wall_z_center
);

let bridge_right_wall = translate(
    box_(bridge_x_length, wall_thickness, wall_height),
    bridge_x_center,
    (strip_y_width * 0.5 - wall_thickness * 0.5),
    wall_z_center
);

let bridge_roof = translate(
    box_(bridge_x_length, strip_y_width, roof_thickness),
    bridge_x_center,
    0.0,
    roof_z_center
);

union(
    lip_strip,
    union(
        mouth_left_wall,
        union(
            mouth_right_wall,
            union(
                mouth_roof,
                union(
                    bridge_floor,
                    union(
                        bridge_left_wall,
                        union(bridge_right_wall, bridge_roof)
                    )
                )
            )
        )
    )
)
