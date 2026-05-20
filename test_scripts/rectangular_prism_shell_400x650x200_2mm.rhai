// Export test fixture: hollow rectangular prism.
// Units: millimeters.
// Outer dimensions: X=400, Y=650, Z=200.
// Wall thickness: 2 mm.

let wall_mm = 2.0;

let outer = box_(400.0, 650.0, 200.0);
let inner = box_(
    400.0 - 2.0 * wall_mm,
    650.0 - 2.0 * wall_mm,
    200.0 - 2.0 * wall_mm
);

subtract(outer, inner)
