// Aircraft with improved wing/body blending

// Fuselage
let fuselage = fuselage_parametric(60.0, 8.0, 0.7, 0.5);

// Main wing - positioned at fuselage mid-section
let wing = wing_with_airfoil("2412", 12.0, 6.0, 40.0, 8.0, 2.0, -1.5);
let wing = translate(wing, 25.0, 0.0, 0.0);

// Horizontal tail
let h_tail = wing_with_airfoil("0012", 6.0, 3.0, 15.0, 5.0, 0.0, 0.0);
let h_tail = translate(h_tail, 52.0, 0.0, 0.0);

// Vertical tail
let v_tail = wing_with_airfoil("0009", 8.0, 4.0, 8.0, 15.0, 0.0, 0.0);
let v_tail = rotate(v_tail, 0.0, 0.0, 90.0);
let v_tail = translate(v_tail, 52.0, 0.0, 5.0);

// Improved blending with larger radii
// Main wing: use large radius (4.0) for smooth fillet
let aircraft = blend(fuselage, wing, 4.0);

// Tail surfaces: smaller radius for cleaner attachment
let aircraft = blend(aircraft, h_tail, 2.0);
let aircraft = blend(aircraft, v_tail, 2.0);

aircraft
