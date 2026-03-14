// Simple tapered, swept wing with NACA 2412 airfoil

let wing = wing_with_airfoil(
    "2412",      // NACA designation
    10.0,        // root chord
    5.0,         // tip chord (taper ratio = 0.5)
    30.0,        // span
    5.0,         // sweep (degrees)
    3.0,         // dihedral (degrees)
    -2.0         // twist/washout (degrees, negative = nose down at tip)
);

wing
