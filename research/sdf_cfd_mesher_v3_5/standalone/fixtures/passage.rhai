let body = scale(sphere(1.0), 3.5, 2.7, 2.4);
let passage = translate(sphere(1.35), -4.2, 0.0, 0.0);
let passage = smooth_union(passage, translate(sphere(1.30), -3.0, 0.30, 0.0), 0.35);
let passage = smooth_union(passage, translate(sphere(1.15), -1.8, 0.60, 0.0), 0.35);
let passage = smooth_union(passage, translate(sphere(1.00), -0.6, 0.75, 0.0), 0.35);
let passage = smooth_union(passage, translate(sphere(1.00), 0.6, 0.75, 0.0), 0.35);
let passage = smooth_union(passage, translate(sphere(1.15), 1.8, 0.60, 0.0), 0.35);
let passage = smooth_union(passage, translate(sphere(1.30), 3.0, 0.30, 0.0), 0.35);
let passage = smooth_union(passage, translate(sphere(1.35), 4.2, 0.0, 0.0), 0.35);
smooth_subtract(body, passage, 0.25)
