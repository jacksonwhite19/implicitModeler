// Field arithmetic demonstration
// Combines multiple fields to create variable offset

// Base thickness of 2.0 everywhere
let base_thickness = constant_field(2.0);

// Add position variation - offset increases along X axis
let position_variation = multiply_fields(
    position_x_field(),
    constant_field(0.1)
);

// Combine: thickness = 2.0 + 0.1*x
let combined = add_fields(base_thickness, position_variation);

// Apply to sphere
offset_by_field(sphere(10.0), combined)
