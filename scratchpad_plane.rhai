// Scratchpad plane from scratchpad.md
// Coordinate convention:
//   X = longitudinal (nose to tail)
//   Y = spanwise
//   Z = vertical

// ---------------------------------------------------------------------------
// Top-level dimensions
// Edit these directly or expose them in the Dimensions panel.
// ---------------------------------------------------------------------------

let fuselage_length = 700.0;
let fuselage_width = 175.0;
let fuselage_height = 100.0;
let nose_length = 130.0;
let fuselage_smoothness = 0.70;   // 0 = rigid frame match, 1 = stronger fairing

let wing_span = 700.0;
let wing_root_chord = 115.0;
let wing_tip_chord = 65.0;
let wing_sweep_deg = 15.0;
let wing_le_x = 280.0;
let wing_root_blend_k = 16.0;

let htail_span = 580.0;
let htail_root_chord = 98.0;
let htail_tip_chord = 80.0;
let htail_le_x = 500.0;
let tail_z = 18.0;
let tail_blend_k = 10.0;

let vtail_span = 310.0;
let vtail_root_chord = 95.0;
let vtail_tip_chord = 53.0;
let vtail_sweep_deg = 30.0;
let vtail_le_x = 500.0;

// ---------------------------------------------------------------------------
// Fuselage
// 700 mm long, elliptical 175 mm wide x 100 mm tall.
// The built-in nose cones are axisymmetric, so use a dense elliptical loft nose
// here to preserve both width and height correctly.
// ---------------------------------------------------------------------------

let fuse_rx = fuselage_width * 0.5;
let fuse_rz = fuselage_height * 0.5;

let stations = [
    fuselage_station(0.0,                    ellipse_section(fuse_rx * 0.03, fuse_rz * 0.02)),
    fuselage_station(nose_length * 0.10,    ellipse_section(fuse_rx * 0.14, fuse_rz * 0.10)),
    fuselage_station(nose_length * 0.20,    ellipse_section(fuse_rx * 0.28, fuse_rz * 0.20)),
    fuselage_station(nose_length * 0.32,    ellipse_section(fuse_rx * 0.44, fuse_rz * 0.34)),
    fuselage_station(nose_length * 0.46,    ellipse_section(fuse_rx * 0.60, fuse_rz * 0.50)),
    fuselage_station(nose_length * 0.60,    ellipse_section(fuse_rx * 0.74, fuse_rz * 0.66)),
    fuselage_station(nose_length * 0.74,    ellipse_section(fuse_rx * 0.86, fuse_rz * 0.80)),
    fuselage_station(nose_length * 0.86,    ellipse_section(fuse_rx * 0.94, fuse_rz * 0.90)),
    fuselage_station(nose_length * 0.96,    ellipse_section(fuse_rx * 0.98, fuse_rz * 0.96)),
    fuselage_station(nose_length + 0.0,     ellipse_section(fuse_rx,        fuse_rz)),
    fuselage_station(nose_length + 40.0,    ellipse_section(fuse_rx,        fuse_rz)),
    fuselage_station(nose_length + 105.0,   ellipse_section(fuse_rx,        fuse_rz)),
    fuselage_station(nose_length + 190.0,   ellipse_section(fuse_rx,        fuse_rz * 0.99)),
    fuselage_station(nose_length + 285.0,   ellipse_section(fuse_rx * 0.99, fuse_rz * 0.98)),
    fuselage_station(nose_length + 370.0,   ellipse_section(fuse_rx * 0.96, fuse_rz * 0.95)),
    fuselage_station(nose_length + 445.0,   ellipse_section(fuse_rx * 0.90, fuse_rz * 0.90)),
    fuselage_station(nose_length + 500.0,   ellipse_section(fuse_rx * 0.80, fuse_rz * 0.82)),
    fuselage_station(nose_length + 540.0,   ellipse_section(fuse_rx * 0.64, fuse_rz * 0.66)),
    fuselage_station(nose_length + 555.0,   ellipse_section(fuse_rx * 0.46, fuse_rz * 0.48)),
    fuselage_station(fuselage_length - 2.0, ellipse_section(fuse_rx * 0.26, fuse_rz * 0.30)),
];
let fuselage = lofted_fuselage_smooth(stations, fuselage_smoothness);

// ---------------------------------------------------------------------------
// Wing
// 700 mm span, 15 deg sweep, 80 mm root chord, 65 mm tip chord
// LE at x = 375 mm
// ---------------------------------------------------------------------------

let wing = translate(
    wing_with_airfoil("2412", wing_root_chord, wing_tip_chord, wing_span, wing_sweep_deg, 0.0, 0.0),
    wing_le_x, 0.0, 0.0
);

let body = smooth_union(fuselage, wing, wing_root_blend_k);

// ---------------------------------------------------------------------------
// Tail
// Conventional tail, no control surfaces
// H-tail: 325 mm span, 65/65 chord, no sweep, LE at x = 500 mm
// V-tail: 200 mm total span, 60/30 chord, 30 deg sweep, centered on h-tail
// ---------------------------------------------------------------------------

// wing_with_airfoil already returns a full-span surface.
let htail = translate(
    wing_with_airfoil("0009", htail_root_chord, htail_tip_chord, htail_span, 0.0, 0.0, 0.0),
    htail_le_x, 0.0, tail_z
);

// Build the fin from the same wing primitive, rotate it upright, then clip away
// the lower half so the root sits on the fuselage centerplane without a hacky cant.
let vtail_raw = rotate(
    wing_with_airfoil("0009", vtail_root_chord, vtail_tip_chord, vtail_span, vtail_sweep_deg, 0.0, 0.0),
    90.0, 0.0, 0.0
);
let vtail_upper_only = intersect(
    vtail_raw,
    translate(box_(1200.0, 1200.0, 1200.0), 0.0, 0.0, 600.0)
);
let vtail = translate(vtail_upper_only, vtail_le_x, 0.0, tail_z);

// Final solid assembly
let body = smooth_union(body, htail, tail_blend_k);
smooth_union(body, vtail, tail_blend_k)
