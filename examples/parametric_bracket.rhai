// Parametric bracket with bolt hole pattern
// Demonstrates: user functions, linear_array, loops, math

fn bolt_hole(r, depth) {
    cylinder(r, depth + 2.0)
}

fn mounting_bracket(width, depth, thickness, hole_r, hole_count, hole_spacing) {
    // Base plate
    let plate = box_(width, depth, thickness);

    // Row of bolt holes using linear_array
    let hole = bolt_hole(hole_r, thickness);
    let hole_row = linear_array(hole, hole_count, hole_spacing, 0.0, 0.0);

    // Center the hole row
    let offset_x = -((hole_count - 1) * hole_spacing) / 2.0;
    let hole_row = translate(hole_row, offset_x, 0.0, 0.0);

    subtract(plate, hole_row)
}

// Create bracket: 60 wide, 30 deep, 8 thick, 3mm holes, 4 holes, 12mm spacing
let bracket = mounting_bracket(60.0, 30.0, 8.0, 3.0, 4, 12.0);

// Add gusset ribs using mirror for symmetry
let rib = translate(box_(3.0, 30.0, 12.0), 22.0, 0.0, 0.0);
let ribs = mirror_x(rib);
let bracket = union(bracket, ribs);

bracket
