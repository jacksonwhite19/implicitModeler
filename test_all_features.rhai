// Comprehensive test of all features
// This script tests all primitives, operations, and combinations

// Test 1: All primitives
let s = sphere(5.0);
let b = box_(8.0, 6.0, 4.0);
let c = cylinder(3.0, 10.0);
let t = torus(10.0, 2.0);
let cone = cone(5.0, 10.0);
let p = plane(0.0, 0.0, 1.0, -5.0);

// Test 2: Boolean operations
let u = union(sphere(5.0), translate(sphere(5.0), 8.0, 0.0, 0.0));
let su = subtract(sphere(8.0), cylinder(3.0, 20.0));
let i = intersect(box_(10.0, 10.0, 10.0), sphere(8.0));

// Test 3: Smooth union
let s1 = sphere(6.0);
let s2 = translate(sphere(6.0), 8.0, 0.0, 0.0);
let smooth = smooth_union(s1, s2, 2.0);

// Test 4: Transforms
let rotated = rotate(box_(10.0, 5.0, 3.0), 0.0, 0.0, 45.0);
let scaled = scale(sphere(5.0), 1.5, 1.0, 0.8);
let translated = translate(torus(10.0, 2.0), 0.0, 0.0, 5.0);

// Test 5: Offset and shell
let offset_shape = offset(sphere(8.0), 2.0);
let shell_shape = shell(torus(12.0, 3.0), 0.8);

// Test 6: Complex combination
let base = box_(30.0, 15.0, 8.0);
let hole1 = cylinder(3.0, 10.0);
let hole1 = translate(hole1, -8.0, 0.0, 0.0);
let hole2 = cylinder(3.0, 10.0);
let hole2 = translate(hole2, 8.0, 0.0, 0.0);
let part = subtract(base, hole1);
let part = subtract(part, hole2);
let part = offset(part, 1.5);

// Return final test result (complex part)
part
