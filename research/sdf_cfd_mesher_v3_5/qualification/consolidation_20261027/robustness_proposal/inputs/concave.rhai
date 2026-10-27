let a = translate(sphere(0.80), -2.1, -1.0, 0.0);
let a = smooth_union(a, translate(sphere(0.80), -1.2, 0.0, 0.0), 0.45);
let a = smooth_union(a, translate(sphere(0.80), 0.0, 0.50, 0.0), 0.45);
let a = smooth_union(a, translate(sphere(0.80), 1.2, 0.0, 0.0), 0.45);
smooth_union(a, translate(sphere(0.80), 2.1, -1.0, 0.0), 0.45)
