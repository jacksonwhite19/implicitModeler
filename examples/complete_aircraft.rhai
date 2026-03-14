// Complete aircraft assembly with fuselage, wings, and tail

// Create fuselage
let fuselage = fuselage_parametric(
    60.0,   // length
    8.0,    // max diameter
    0.7,    // nose shape (0=pointed, 1=rounded)
    0.5     // tail shape
);

// Create main wing
let wing = wing_with_airfoil(
    "2412",     // Cambered airfoil for lift
    12.0,       // root chord
    6.0,        // tip chord
    40.0,       // span
    8.0,        // sweep (degrees)
    2.0,        // dihedral (degrees)
    -1.5        // washout for stability
);
let wing = translate(wing, 25.0, 0.0, 0.0);

// Create horizontal stabilizer
let h_tail = wing_with_airfoil(
    "0012",     // Symmetric airfoil
    6.0,        // root chord
    3.0,        // tip chord
    15.0,       // span
    5.0,        // slight sweep
    0.0,        // no dihedral
    0.0         // no twist
);
let h_tail = translate(h_tail, 52.0, 0.0, 0.0);

// Create vertical stabilizer (rotated horizontal surface)
let v_tail = wing_with_airfoil(
    "0009",     // Thin symmetric airfoil
    8.0,        // root chord
    4.0,        // tip chord
    8.0,        // height (span when rotated)
    15.0,       // sweep
    0.0,        // N/A for vertical
    0.0         // no twist
);
let v_tail = rotate(v_tail, 0.0, 0.0, 90.0);  // Rotate to vertical
let v_tail = translate(v_tail, 52.0, 0.0, 5.0);

// Blend everything together with smooth fillets
let aircraft = blend(fuselage, wing, 2.5);
let aircraft = blend(aircraft, h_tail, 1.5);
let aircraft = blend(aircraft, v_tail, 1.5);

aircraft
