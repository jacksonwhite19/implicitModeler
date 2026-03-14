// Conformal lattice - variable density based on position
// Demonstrates field-controlled lattice thickness

let shape = sphere(30.0);

// Create radial field: denser (thicker struts) at center, lighter at edges
let density_field = radial_field(
    0.0, 0.0, 0.0,  // center
    0.0, 30.0,       // inner radius, outer radius
    1.5, 0.3         // thick at center, thin at edge
);

// Create gyroid with field-controlled thickness
let lattice = gyroid_with_field(5.0, density_field);

// Intersect with sphere to get conformal lattice structure
intersect(shape, lattice)
