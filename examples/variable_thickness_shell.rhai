// Variable thickness shell - thick at center, thin at edges
// Demonstrates radial field controlling shell thickness

let sphere = sphere(20.0);

// Create radial field: thick (4.0) at center, thin (1.0) at edges
let thickness = radial_field(
    0.0, 0.0, 0.0,  // center point
    0.0, 20.0,       // inner radius, outer radius
    4.0, 1.0         // thick at center, thin at edge
);

shell_with_field(sphere, thickness)
