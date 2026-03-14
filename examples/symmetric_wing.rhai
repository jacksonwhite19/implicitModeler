// Full symmetric wing using mirror_y
// Demonstrates: mirror, math constants, user functions

fn make_wing_half() {
    wing_with_airfoil(
        "2412",    // NACA 4-digit designation
        12.0,      // root chord
        6.0,       // tip chord
        25.0,      // half-span
        20.0,      // sweep angle (degrees)
        5.0,       // dihedral (degrees)
        -2.0       // washout twist (degrees)
    )
}

// Create full symmetric wing by mirroring the half-wing
let half_wing = make_wing_half();
let full_wing = mirror_y(half_wing);

full_wing
