// Simple gyroid lattice confined to a box
// Demonstrates TPMS (triply periodic minimal surface) structure

// Create gyroid with 5.0 cell size and 0.8 thickness
let lattice = gyroid(5.0, 0.8);

// Confine to a box
let box = box_(40.0, 40.0, 40.0);

// Intersect to get lattice-filled box
intersect(box, lattice)
