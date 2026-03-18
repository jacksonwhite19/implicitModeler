// Mounting hole detection from triangle meshes.
#![allow(dead_code)] // Planned feature — not yet wired into the UI
//
// Detects circular holes by grouping axis-aligned face clusters, fitting circles
// to boundary vertex loops, and estimating depth via bounding-box marching.

use glam::Vec3;
use crate::mesh::import::TriangleMesh;
use std::collections::HashMap;

#[derive(Clone, Debug)]
pub struct DetectedHole {
    pub position:   Vec3,   // center of hole entry face
    pub direction:  Vec3,   // axis direction (normal of cluster)
    pub radius:     f32,    // detected radius
    pub depth:      f32,    // estimated depth
    pub confidence: f32,    // 0–1
}

// ── Primary axis classification ───────────────────────────────────────────────

/// Returns Some(normal) if the triangle's normal aligns closely with a primary axis.
fn primary_axis(normal: Vec3) -> Option<Vec3> {
    const THRESHOLD: f32 = 0.9;
    let axes = [
        Vec3::X, Vec3::NEG_X,
        Vec3::Y, Vec3::NEG_Y,
        Vec3::Z, Vec3::NEG_Z,
    ];
    axes.iter()
        .filter_map(|&a| {
            let d = normal.dot(a);
            if d > THRESHOLD { Some(a) } else { None }
        })
        .next()
}

/// Axis key: encode ±X/±Y/±Z as i8 tuple to use in a HashMap.
fn axis_key(a: Vec3) -> (i8, i8, i8) {
    (a.x as i8, a.y as i8, a.z as i8)
}

// ── Cluster ───────────────────────────────────────────────────────────────────

struct Cluster {
    triangle_indices: Vec<usize>,
    centroid:         Vec3,
    normal:           Vec3,
}

/// Merge triangles into positional clusters within a single axis-group.
/// Two triangles merge when their centroids are within 2 * max_radius.
fn cluster_triangles(
    tris: &[(usize, Vec3)], // (tri_idx, centroid)
    max_radius: f32,
    normal: Vec3,
) -> Vec<Cluster> {
    let merge_dist = 2.0 * max_radius;
    let mut clusters: Vec<Cluster> = Vec::new();

    for &(idx, centroid) in tris {
        // Try to merge into an existing cluster.
        let mut merged = false;
        for c in &mut clusters {
            if (c.centroid - centroid).length() < merge_dist {
                c.triangle_indices.push(idx);
                // Recompute centroid as mean.
                let n = c.triangle_indices.len() as f32;
                c.centroid = c.centroid * ((n - 1.0) / n) + centroid / n;
                merged = true;
                break;
            }
        }
        if !merged {
            clusters.push(Cluster {
                triangle_indices: vec![idx],
                centroid,
                normal,
            });
        }
    }

    clusters
}

// ── Circle fit ────────────────────────────────────────────────────────────────

/// Fit a circle (center + radius) to 2D points using the algebraic least-squares
/// approach: center = mean, radius = mean distance.
fn fit_circle(pts: &[Vec3], plane_normal: Vec3) -> (Vec3, f32, f32) {
    // Build two orthogonal axes perpendicular to plane_normal.
    let up = if plane_normal.abs().dot(Vec3::Y) < 0.9 { Vec3::Y } else { Vec3::Z };
    let ax = plane_normal.cross(up).normalize_or_zero();
    let ay = plane_normal.cross(ax).normalize_or_zero();

    let n = pts.len() as f32;
    let mean2d_x: f32 = pts.iter().map(|p| p.dot(ax)).sum::<f32>() / n;
    let mean2d_y: f32 = pts.iter().map(|p| p.dot(ay)).sum::<f32>() / n;

    // Recover 3D center on the plane.
    // Use the plane-projected mean of the 3D points as center.
    let center3d = ax * mean2d_x + ay * mean2d_y;

    let dists: Vec<f32> = pts.iter().map(|p| {
        let px = p.dot(ax) - mean2d_x;
        let py = p.dot(ay) - mean2d_y;
        (px * px + py * py).sqrt()
    }).collect();

    let mean_r = dists.iter().sum::<f32>() / dists.len() as f32;
    let variance = dists.iter().map(|d| (d - mean_r) * (d - mean_r)).sum::<f32>() / dists.len() as f32;
    let residual = variance.sqrt();

    (center3d, mean_r, residual)
}

// ── Boundary extraction ───────────────────────────────────────────────────────

/// Returns indices of boundary vertices (edges appearing exactly once).
fn boundary_vertices(mesh: &TriangleMesh, tri_indices: &[usize]) -> Vec<u32> {
    // Count how many times each undirected edge appears within this cluster.
    let mut edge_count: HashMap<(u32, u32), u32> = HashMap::new();
    for &ti in tri_indices {
        let [a, b, c] = mesh.triangles[ti];
        for (ea, eb) in [(a, b), (b, c), (c, a)] {
            let key = if ea < eb { (ea, eb) } else { (eb, ea) };
            *edge_count.entry(key).or_insert(0) += 1;
        }
    }
    // Boundary edges appear exactly once.
    let mut bv: std::collections::HashSet<u32> = std::collections::HashSet::new();
    for ((va, vb), cnt) in &edge_count {
        if *cnt == 1 {
            bv.insert(*va);
            bv.insert(*vb);
        }
    }
    bv.into_iter().collect()
}

// ── Depth estimation ──────────────────────────────────────────────────────────

/// Estimate depth by marching from `position` along `-direction` and counting
/// steps while still inside the mesh AABB.
fn estimate_depth(
    position: Vec3,
    direction: Vec3,
    bounds_min: Vec3,
    bounds_max: Vec3,
) -> f32 {
    let step = 1.0_f32;
    let max_steps = 60_usize;
    let margin = 1.0_f32;

    let dir = -direction.normalize(); // march into the solid
    let mut count = 0;

    for i in 1..=max_steps {
        let p = position + dir * (i as f32 * step);
        if p.x >= bounds_min.x - margin && p.x <= bounds_max.x + margin
            && p.y >= bounds_min.y - margin && p.y <= bounds_max.y + margin
            && p.z >= bounds_min.z - margin && p.z <= bounds_max.z + margin
        {
            count += 1;
        } else {
            break;
        }
    }

    count as f32 * step
}

// ── Public API ────────────────────────────────────────────────────────────────

pub fn detect_mounting_holes(
    mesh: &TriangleMesh,
    min_radius: f32,
    max_radius: f32,
) -> Vec<DetectedHole> {
    if mesh.triangles.is_empty() {
        return Vec::new();
    }

    // Step 1: Group triangles by primary axis.
    let mut axis_groups: HashMap<(i8, i8, i8), Vec<(usize, Vec3)>> = HashMap::new();
    for (i, (tri, &normal)) in mesh.triangles.iter().zip(mesh.normals.iter()).enumerate() {
        if let Some(axis) = primary_axis(normal) {
            let centroid = mesh.triangle_centroid(i);
            axis_groups.entry(axis_key(axis)).or_default().push((i, centroid));
        }
        let _ = tri;
    }

    let mut holes = Vec::new();

    for ((ax, ay, az), tris) in &axis_groups {
        let normal = Vec3::new(*ax as f32, *ay as f32, *az as f32);

        // Step 2: Cluster triangles by position.
        let clusters = cluster_triangles(tris, max_radius, normal);

        for cluster in &clusters {
            // Step 3: Find boundary vertices.
            let bv_indices = boundary_vertices(mesh, &cluster.triangle_indices);

            // Step 4: Skip if too few boundary vertices.
            if bv_indices.len() < 8 {
                continue;
            }

            let bv_pts: Vec<Vec3> = bv_indices.iter()
                .map(|&i| mesh.vertices[i as usize])
                .collect();

            // Step 5: Fit circle to boundary vertices.
            let (center_2d, radius, residual) = fit_circle(&bv_pts, normal);

            // Reconstruct 3D position: project centroid onto plane, add center offset.
            let position = center_2d + normal * (cluster.centroid.dot(normal));

            // Step 6/7: Check radius range and residual.
            if radius < min_radius || radius > max_radius {
                continue;
            }
            if residual > 0.5 {
                continue;
            }

            // Step 8: Roundness check.
            let up = if normal.abs().dot(Vec3::Y) < 0.9 { Vec3::Y } else { Vec3::Z };
            let ax2 = normal.cross(up).normalize_or_zero();
            let ay2 = normal.cross(ax2).normalize_or_zero();
            let dists: Vec<f32> = bv_pts.iter().map(|p| {
                let dx = p.dot(ax2) - position.dot(ax2);
                let dy = p.dot(ay2) - position.dot(ay2);
                (dx * dx + dy * dy).sqrt()
            }).collect();
            let min_d = dists.iter().cloned().fold(f32::MAX, f32::min);
            let max_d = dists.iter().cloned().fold(0.0_f32, f32::max);
            let roundness = if max_d > 1e-5 { min_d / max_d } else { 0.0 };
            if roundness < 0.85 {
                continue;
            }

            // Step 9: Depth estimation.
            let depth = estimate_depth(
                position,
                normal,
                mesh.bounds_min,
                mesh.bounds_max,
            );

            // Step 10: Mounting hole filter.
            if depth < 1.5 * radius {
                continue;
            }

            // Step 11: Confidence.
            let residual_score = (1.0 - residual / 0.5).max(0.0);
            let depth_score = (depth / (1.5 * radius)).min(1.0);
            let confidence = (residual_score * roundness * depth_score).clamp(0.0, 1.0);

            // Step 12: Keep holes with confidence > 0.7.
            if confidence > 0.7 {
                holes.push(DetectedHole {
                    position,
                    direction: normal,
                    radius,
                    depth,
                    confidence,
                });
            }
        }
    }

    // Sort by radius descending.
    holes.sort_by(|a, b| b.radius.partial_cmp(&a.radius).unwrap_or(std::cmp::Ordering::Equal));
    holes
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::import::TriangleMesh;

    #[test]
    fn test_detect_mounting_holes_basic() {
        // Simple flat mesh — won't have holes but should not panic.
        let mesh = TriangleMesh {
            vertices: vec![
                Vec3::new(-10.0, -10.0, 0.0),
                Vec3::new( 10.0, -10.0, 0.0),
                Vec3::new( 10.0,  10.0, 0.0),
                Vec3::new(-10.0,  10.0, 0.0),
            ],
            triangles: vec![[0, 1, 2], [0, 2, 3]],
            normals: vec![Vec3::Z, Vec3::Z],
            bounds_min: Vec3::new(-10.0, -10.0, 0.0),
            bounds_max: Vec3::new( 10.0,  10.0, 0.0),
        };
        let holes = detect_mounting_holes(&mesh, 1.0, 5.0);
        // No assertion on count — must not panic, and a flat square has no circular holes.
        let _ = holes;
    }
}
