let a = translate(scale(sphere(1.0), 1.8, 1.5, 1.3), -1.6, 0.0, 0.0);
let b = translate(scale(sphere(1.0), 1.3, 1.1, 1.0), 1.6, 0.35, 0.25);
smooth_union(a, b, 0.50)
