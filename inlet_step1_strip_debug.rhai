// Step 2 simple scoop shell:
// - lower lip is the exact conformal strip between two fuselage offsets
// - add vertical side walls and a flat roof above that strip
// - no internal bend / no full duct yet

let show_fuselage = true;
let show_strip = true;
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

// Restrict to a short mouth strip on the upper fuselage.
let clip_box = translate(
    box_(strip_x_length, strip_y_width, strip_z_height),
    strip_x_center,
    0.0,
    strip_z_center
);

let lip_strip = intersect(oml_band, clip_box);

// Simple vertical side walls and flat roof over the conformal strip.
let left_wall = translate(
    box_(strip_x_length, wall_thickness, wall_height),
    strip_x_center,
    -(strip_y_width * 0.5 - wall_thickness * 0.5),
    wall_z_center
);

let right_wall = translate(
    box_(strip_x_length, wall_thickness, wall_height),
    strip_x_center,
    (strip_y_width * 0.5 - wall_thickness * 0.5),
    wall_z_center
);

let roof = translate(
    box_(strip_x_length, strip_y_width, roof_thickness),
    strip_x_center,
    0.0,
    roof_z_center
);

let scoop_shell = union(
    lip_strip,
    union(left_wall, union(right_wall, roof))
);

if show_fuselage && show_strip && show_shell {
    union(fuselage, scoop_shell)
} else if show_strip && show_shell {
    scoop_shell
} else if show_fuselage && show_strip {
    union(fuselage, lip_strip)
} else if show_fuselage {
    fuselage
} else {
    lip_strip
}
