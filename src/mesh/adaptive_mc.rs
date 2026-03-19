//! Adaptive octree marching cubes.
//!
//! Concentrates triangles near the isosurface by subdividing an octree until
//! leaf cells reach `target_cell_size`.  Cells that are provably entirely
//! inside or outside the shape are pruned early, so the algorithm is fast even
//! for large models with thin features (e.g. wing trailing edges).
//!
//! Normals are computed from the SDF gradient (central differences), which
//! gives naturally smooth shading without any averaging heuristics.

use rayon::prelude::*;
use glam::Vec3;
use crate::sdf::Sdf;
use crate::mesh::{Mesh, Vertex};
use super::marching_cubes::{EDGE_TABLE, TRI_TABLE, CUBE_CORNERS, EDGE_VERTICES};

// ─── quality presets ────────────────────────────────────────────────────────

/// Quality level for mesh export.  Higher quality → smaller cells → more
/// triangles and longer generation time.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum MeshQuality {
    /// ~0.5 unit cells — instant preview
    Draft,
    /// ~0.2 unit cells — good for most purposes
    Normal,
    /// ~0.1 unit cells — catches thin features (trailing edges, blends)
    Fine,
    /// ~0.05 unit cells — maximum fidelity, slow
    Ultra,
}

impl Default for MeshQuality {
    fn default() -> Self { Self::Normal }
}

impl MeshQuality {
    pub fn label(self) -> &'static str {
        match self {
            Self::Draft  => "Draft",
            Self::Normal => "Normal",
            Self::Fine   => "Fine",
            Self::Ultra  => "Ultra",
        }
    }

    pub fn to_resolution(self) -> u32 {
        match self {
            Self::Draft => 24,
            Self::Normal => 32,
            Self::Fine => 48,
            Self::Ultra => 64,
        }
    }

    pub fn target_cell_size_mm(self) -> f32 {
        match self {
            Self::Draft => 2.0,
            Self::Normal => 1.0,
            Self::Fine => 0.5,
            Self::Ultra => 0.25,
        }
    }

    pub fn all() -> &'static [MeshQuality] {
        &[Self::Draft, Self::Normal, Self::Fine, Self::Ultra]
    }
}

// ─── internals ──────────────────────────────────────────────────────────────

/// A cube cell whose 8 corners have already been evaluated.
struct OctreeCell {
    min:     Vec3,
    size:    f32,
    corners: [f32; 8], // SDF values at corners in XYZ bit-index order
}

fn eval_corners(sdf: &dyn Sdf, min: Vec3, size: f32) -> [f32; 8] {
    std::array::from_fn(|i| {
        let [cx, cy, cz] = CUBE_CORNERS[i];
        sdf.distance(min + Vec3::new(cx as f32, cy as f32, cz as f32) * size)
    })
}

/// Returns `true` if the cell is provably inside or outside the surface.
/// Uses the Lipschitz-1 property of SDFs: the surface is at least |d| away
/// from any point with distance value d.  If all corners have the same sign
/// and the minimum |value| exceeds the cell half-diagonal, the surface cannot
/// pass through the cell.
fn can_prune(corners: &[f32; 8], size: f32) -> bool {
    let all_pos = corners.iter().all(|&v| v >= 0.0);
    let all_neg = corners.iter().all(|&v| v <= 0.0);
    if !all_pos && !all_neg {
        return false; // sign change → surface present
    }
    let half_diag = size * 0.866_025_4; // sqrt(3)/2
    corners.iter().map(|v| v.abs()).fold(f32::MAX, f32::min) > half_diag
}

fn has_surface(corners: &[f32; 8]) -> bool {
    let all_pos = corners.iter().all(|&v| v >= 0.0);
    let all_neg = corners.iter().all(|&v| v <= 0.0);
    !all_pos && !all_neg
}

/// Build the octree down to `split_depth` levels (stopping early if the cell
/// is already ≤ target_size or can be pruned).  Returns the non-pruned frontier
/// cells for parallel processing.
fn collect_seeds(
    sdf: &dyn Sdf,
    root_min: Vec3,
    root_size: f32,
    target_size: f32,
    split_depth: u32,
) -> Vec<(Vec3, f32)> {
    let mut stack: Vec<(Vec3, f32, u32)> = vec![(root_min, root_size, 0)];
    let mut seeds = Vec::new();

    while let Some((min, size, depth)) = stack.pop() {
        let corners = eval_corners(sdf, min, size);
        if can_prune(&corners, size) { continue; }

        // Already at target resolution → let process_subtree handle it
        if size <= target_size * 1.001 {
            seeds.push((min, size));
            continue;
        }

        // At split depth → hand off to parallel workers
        if depth >= split_depth {
            seeds.push((min, size));
            continue;
        }

        // Subdivide
        let half = size * 0.5;
        for dz in 0..2usize {
            for dy in 0..2usize {
                for dx in 0..2usize {
                    let child_min = min + Vec3::new(dx as f32, dy as f32, dz as f32) * half;
                    stack.push((child_min, half, depth + 1));
                }
            }
        }
    }
    seeds
}

/// Process one seed subtree down to `target_size`, collecting surface cells.
fn process_subtree(
    sdf: &dyn Sdf,
    seed_min: Vec3,
    seed_size: f32,
    target_size: f32,
) -> Vec<OctreeCell> {
    let mut stack: Vec<(Vec3, f32)> = vec![(seed_min, seed_size)];
    let mut cells = Vec::new();

    while let Some((min, size)) = stack.pop() {
        let corners = eval_corners(sdf, min, size);
        if can_prune(&corners, size) { continue; }

        if size <= target_size * 1.001 {
            if has_surface(&corners) {
                cells.push(OctreeCell { min, size, corners });
            }
            continue;
        }

        let half = size * 0.5;
        for dz in 0..2usize {
            for dy in 0..2usize {
                for dx in 0..2usize {
                    let child_min = min + Vec3::new(dx as f32, dy as f32, dz as f32) * half;
                    stack.push((child_min, half));
                }
            }
        }
    }
    cells
}

/// Run standard marching cubes on a single cell and return its triangle vertex
/// positions (no normals yet — those are computed in batch later).
fn mc_cell_positions(cell: &OctreeCell) -> Vec<Vec3> {
    let mut cube_index = 0u8;
    for (i, &val) in cell.corners.iter().enumerate() {
        if val < 0.0 { cube_index |= 1 << i; }
    }

    let edge_flags = EDGE_TABLE[cube_index as usize];
    if edge_flags == 0 { return vec![]; }

    // Corner world positions — must use same ordering as CUBE_CORNERS for table correctness
    let cp: [Vec3; 8] = std::array::from_fn(|i| {
        let [cx, cy, cz] = CUBE_CORNERS[i];
        cell.min + Vec3::new(cx as f32, cy as f32, cz as f32) * cell.size
    });

    // Interpolate surface crossing on each active edge
    let mut ev = [Vec3::ZERO; 12];
    for edge in 0..12usize {
        if edge_flags & (1 << edge) != 0 {
            let [v0, v1] = EDGE_VERTICES[edge];
            let d0 = cell.corners[v0];
            let d1 = cell.corners[v1];
            let t = if (d1 - d0).abs() > 1e-7 {
                (-d0 / (d1 - d0)).clamp(0.0, 1.0)
            } else {
                0.5
            };
            ev[edge] = cp[v0].lerp(cp[v1], t);
        }
    }

    // Emit triangles
    let mut positions = Vec::new();
    let tri_row = &TRI_TABLE[cube_index as usize];
    let mut i = 0;
    while i < 16 && tri_row[i] != -1 {
        positions.push(ev[tri_row[i] as usize]);
        positions.push(ev[tri_row[i + 1] as usize]);
        positions.push(ev[tri_row[i + 2] as usize]);
        i += 3;
    }
    positions
}

/// SDF gradient (central differences) — gives the outward surface normal.
#[inline]
fn sdf_gradient(sdf: &dyn Sdf, p: Vec3, eps: f32) -> Vec3 {
    Vec3::new(
        sdf.distance(p + Vec3::X * eps) - sdf.distance(p - Vec3::X * eps),
        sdf.distance(p + Vec3::Y * eps) - sdf.distance(p - Vec3::Y * eps),
        sdf.distance(p + Vec3::Z * eps) - sdf.distance(p - Vec3::Z * eps),
    ).normalize_or_zero()
}

/// Weld vertices within `epsilon` of each other to close cracks at octree
/// level boundaries.  Uses a spatial hash (round to nearest grid cell).
fn weld_vertices(mesh: &mut Mesh, epsilon: f32) {
    if epsilon <= 0.0 { return; }
    let inv = 1.0 / epsilon;
    let mut grid: std::collections::HashMap<[i32; 3], u32> =
        std::collections::HashMap::with_capacity(mesh.vertices.len());
    let mut remap = vec![0u32; mesh.vertices.len()];
    let mut new_verts: Vec<Vertex> = Vec::with_capacity(mesh.vertices.len());

    for (i, v) in mesh.vertices.iter().enumerate() {
        let key = [
            (v.position[0] * inv).round() as i32,
            (v.position[1] * inv).round() as i32,
            (v.position[2] * inv).round() as i32,
        ];
        if let Some(&idx) = grid.get(&key) {
            remap[i] = idx;
        } else {
            let idx = new_verts.len() as u32;
            grid.insert(key, idx);
            remap[i] = idx;
            new_verts.push(*v);
        }
    }
    mesh.vertices = new_verts;
    for idx in &mut mesh.indices { *idx = remap[*idx as usize]; }
}

/// Remove zero-area (degenerate) triangles.
fn remove_degenerate_triangles(mesh: &mut Mesh, cell_size: f32) {
    let min_area_sq = (cell_size * 0.001).powi(2);
    let clean: Vec<u32> = mesh.indices.chunks(3)
        .filter(|tri| {
            let p0 = Vec3::from_array(mesh.vertices[tri[0] as usize].position);
            let p1 = Vec3::from_array(mesh.vertices[tri[1] as usize].position);
            let p2 = Vec3::from_array(mesh.vertices[tri[2] as usize].position);
            (p1 - p0).cross(p2 - p0).length_squared() > min_area_sq
        })
        .flatten()
        .copied()
        .collect();
    mesh.indices = clean;
}

// ─── public API ─────────────────────────────────────────────────────────────

/// Extract a triangle mesh using adaptive octree marching cubes.
///
/// `target_cell_size` controls the finest leaf-cell size.  Smaller values
/// capture thinner features and produce more triangles.
///
/// Normals are always computed from the SDF gradient and are inherently smooth;
/// the `smooth_normals` flag enables an additional vertex-normal averaging pass
/// for ultra-smooth shading on very coarse meshes.
pub fn extract_mesh_adaptive(
    sdf:              &dyn Sdf,
    bounds_min:       Vec3,
    bounds_max:       Vec3,
    target_cell_size: f32,
    smooth_normals:   bool,
) -> Mesh {
    use std::time::Instant;
    let t0 = Instant::now();

    // Build a cubic root cell that contains the entire model.
    let center     = (bounds_min + bounds_max) * 0.5;
    let max_extent = (bounds_max - bounds_min).max_element();
    let mut root_size = target_cell_size;
    while root_size < max_extent * 1.01 { root_size *= 2.0; }
    let root_min = center - Vec3::splat(root_size * 0.5);

    // Phase 1 — serial octree down to split_depth 5 (≤32 768 seeds)
    let seeds = collect_seeds(sdf, root_min, root_size, target_cell_size, 5);
    let t1 = Instant::now();

    // Phase 2 — parallel fine descent per seed subtree
    let surface_cells: Vec<OctreeCell> = seeds
        .par_iter()
        .flat_map(|&(min, size)| process_subtree(sdf, min, size, target_cell_size))
        .collect();
    let t2 = Instant::now();

    // Phase 3 — parallel MC per cell (positions only, no normals yet)
    let all_positions: Vec<Vec3> = surface_cells
        .par_iter()
        .flat_map(|cell| mc_cell_positions(cell))
        .collect();
    let t3 = Instant::now();

    // Build sequential indices (every three verts = one triangle)
    let all_indices: Vec<u32> = (0..all_positions.len() as u32).collect();

    // Phase 4 — parallel SDF-gradient normals
    let normal_eps = (target_cell_size * 0.3).max(0.001);
    let normals: Vec<Vec3> = all_positions
        .par_iter()
        .map(|&p| sdf_gradient(sdf, p, normal_eps))
        .collect();
    let t4 = Instant::now();

    let vertices: Vec<Vertex> = all_positions.iter().zip(normals.iter())
        .map(|(&p, &n)| Vertex {
            position: [p.x, p.y, p.z],
            normal:   [n.x, n.y, n.z],
        })
        .collect();

    let mut mesh = Mesh { vertices, indices: all_indices };

    // Weld cracks at octree level boundaries
    weld_vertices(&mut mesh, target_cell_size * 0.45);

    // Optional extra smooth-normal pass (averages normals at shared positions)
    if smooth_normals {
        apply_smooth_normals_weighted(&mut mesh);
    }

    // Remove degenerate triangles
    remove_degenerate_triangles(&mut mesh, target_cell_size);

    let t5 = Instant::now();

    eprintln!(
        "Adaptive MC: cell={:.3}  seeds={}  surface={}  verts={}  tris={}",
        target_cell_size, seeds.len(), surface_cells.len(),
        mesh.vertices.len(), mesh.indices.len() / 3
    );
    eprintln!(
        "  Phase1={:?}  Phase2={:?}  Phase3={:?}  Normals={:?}  Post={:?}  Total={:?}",
        t1-t0, t2-t1, t3-t2, t4-t3, t5-t4, t5-t0
    );

    mesh
}

/// Weighted average of normals at spatially coincident vertices (post-weld).
/// Uses face-area weighting for better quality on irregular meshes.
fn apply_smooth_normals_weighted(mesh: &mut Mesh) {
    // Accumulate area-weighted normals per vertex
    let n = mesh.vertices.len();
    let mut accum = vec![Vec3::ZERO; n];

    for tri in mesh.indices.chunks(3) {
        let (i0, i1, i2) = (tri[0] as usize, tri[1] as usize, tri[2] as usize);
        let p0 = Vec3::from_array(mesh.vertices[i0].position);
        let p1 = Vec3::from_array(mesh.vertices[i1].position);
        let p2 = Vec3::from_array(mesh.vertices[i2].position);
        let face_normal = (p1 - p0).cross(p2 - p0); // magnitude = 2 * area
        accum[i0] += face_normal;
        accum[i1] += face_normal;
        accum[i2] += face_normal;
    }

    for (v, acc) in mesh.vertices.iter_mut().zip(accum.iter()) {
        let n = acc.normalize_or_zero();
        if n != Vec3::ZERO {
            v.normal = [n.x, n.y, n.z];
        }
    }
}

// ─── unit tests ─────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::primitives::Sphere;

    #[test]
    fn test_adaptive_sphere_normal() {
        let s = Sphere::new(5.0);
        let mesh = extract_mesh_adaptive(
            &s,
            Vec3::splat(-8.0), Vec3::splat(8.0),
            0.5, false,
        );
        assert!(!mesh.vertices.is_empty(), "Should produce vertices");
        assert_eq!(mesh.indices.len() % 3, 0, "Index count divisible by 3");

        // All normals should be unit-length
        for v in &mesh.vertices {
            let len = Vec3::from_array(v.normal).length();
            assert!((len - 1.0).abs() < 0.01, "Normal not unit: {}", len);
        }
    }

    #[test]
    fn test_adaptive_fine_captures_more_tris() {
        let s = Sphere::new(5.0);
        let coarse = extract_mesh_adaptive(&s, Vec3::splat(-8.0), Vec3::splat(8.0), 1.0, false);
        let fine   = extract_mesh_adaptive(&s, Vec3::splat(-8.0), Vec3::splat(8.0), 0.3, false);
        assert!(
            fine.indices.len() > coarse.indices.len(),
            "Fine ({}) should have more tris than coarse ({})",
            fine.indices.len() / 3, coarse.indices.len() / 3
        );
    }
}
