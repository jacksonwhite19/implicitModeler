// Export test fixture: hollow fixed-wing panel.
// Units: millimeters.
// Airfoil: NACA 2212.
// Span: 700 mm.
// Wall thickness: 2 mm.
// Trailing edge: intentionally blunt for STL export robustness.

let wall_mm = 2.0;
let span_mm = 700.0;
let root_chord_mm = 180.0;
let tip_chord_mm = 110.0;
let sweep_deg = 0.0;
let dihedral_deg = 0.0;
let twist_deg = 0.0;
let min_trailing_edge_mm = 4.0;
let min_leading_edge_radius_mm = 2.0;
let tip_cap_thickness_mm = wall_mm * 2.0;

let outer = wing_with_airfoil_export_safe(
    "2212",
    root_chord_mm,
    tip_chord_mm,
    span_mm,
    sweep_deg,
    dihedral_deg,
    twist_deg,
    min_trailing_edge_mm,
    min_leading_edge_radius_mm
);

let hollow_shell = shell(outer, wall_mm);

// Keep the finite wing tips solid. The shell operation hollows inward from the
// whole surface, so without these ribs the span-end faces become open cavities.
let cap_slab = box_(10000.0, tip_cap_thickness_mm, 10000.0);
let positive_tip_cap = intersect(
    outer,
    translate(cap_slab, 0.0, span_mm / 2.0 - tip_cap_thickness_mm / 2.0, 0.0)
);
let negative_tip_cap = intersect(
    outer,
    translate(cap_slab, 0.0, -span_mm / 2.0 + tip_cap_thickness_mm / 2.0, 0.0)
);

union(union(hollow_shell, positive_tip_cap), negative_tip_cap)
