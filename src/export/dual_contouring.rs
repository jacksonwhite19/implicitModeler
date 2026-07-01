use std::collections::HashMap;

use glam::{Mat3, Vec3};

use crate::export::adaptive_octree::{AdaptiveOctree, collect_surface_leaves};
use crate::mesh::marching_cubes::{CUBE_CORNERS, EDGE_VERTICES};
use crate::mesh::{Mesh, Vertex};
use crate::sdf::Sdf;

#[derive(Clone, Copy, Debug)]
pub struct HermiteSample {
    pub edge_index: usize,
    pub position: Vec3,
    pub normal: Vec3,
}

#[derive(Clone, Copy, Debug)]
pub struct DualVertex {
    pub position: Vec3,
    pub sample_count: usize,
}

#[derive(Clone, Debug)]
struct UniformDualCell {
    vertex: DualVertex,
    corner_distances: [f32; 8],
}

fn sdf_gradient(sdf: &dyn Sdf, p: Vec3, eps: f32) -> Vec3 {
    Vec3::new(
        sdf.distance(p + Vec3::X * eps) - sdf.distance(p - Vec3::X * eps),
        sdf.distance(p + Vec3::Y * eps) - sdf.distance(p - Vec3::Y * eps),
        sdf.distance(p + Vec3::Z * eps) - sdf.distance(p - Vec3::Z * eps),
    )
    .normalize_or_zero()
}

fn cell_corner_positions(cell_min: Vec3, cell_size: f32) -> [Vec3; 8] {
    std::array::from_fn(|i| {
        let [cx, cy, cz] = CUBE_CORNERS[i];
        cell_min + Vec3::new(cx as f32, cy as f32, cz as f32) * cell_size
    })
}

pub fn sample_hermite_cell(sdf: &dyn Sdf, cell_min: Vec3, cell_size: f32) -> Vec<HermiteSample> {
    let corners = cell_corner_positions(cell_min, cell_size);
    let corner_distances: [f32; 8] = std::array::from_fn(|i| sdf.distance(corners[i]));
    let eps = (cell_size * 0.2).max(0.01);
    let mut samples = Vec::new();

    for (edge_index, [a_idx, b_idx]) in EDGE_VERTICES.iter().copied().enumerate() {
        let da = corner_distances[a_idx];
        let db = corner_distances[b_idx];
        let sign_change = (da <= 0.0 && db >= 0.0) || (da >= 0.0 && db <= 0.0);
        if !sign_change || (db - da).abs() < 1e-8 {
            continue;
        }
        let t = (-da / (db - da)).clamp(0.0, 1.0);
        let position = corners[a_idx].lerp(corners[b_idx], t);
        let normal = sdf_gradient(sdf, position, eps);
        samples.push(HermiteSample {
            edge_index,
            position,
            normal,
        });
    }

    samples
}

pub fn solve_qef_vertex(
    samples: &[HermiteSample],
    cell_min: Vec3,
    cell_max: Vec3,
) -> Option<DualVertex> {
    if samples.is_empty() {
        return None;
    }

    let mut fallback = Vec3::ZERO;
    for sample in samples {
        fallback += sample.position;
    }
    fallback /= samples.len() as f32;
    if samples.len() < 3 {
        return Some(DualVertex {
            position: fallback.clamp(cell_min, cell_max),
            sample_count: samples.len(),
        });
    }

    let mut ata = Mat3::ZERO;
    let mut atb = Vec3::ZERO;
    for sample in samples {
        let n = sample.normal.normalize_or_zero();
        if n == Vec3::ZERO {
            continue;
        }
        let outer = Mat3::from_cols(n * n.x, n * n.y, n * n.z);
        ata += outer;
        atb += n * n.dot(sample.position);
    }
    if ata.determinant().abs() < 1e-6 {
        return Some(DualVertex {
            position: fallback.clamp(cell_min, cell_max),
            sample_count: samples.len(),
        });
    }

    let solved = ata.inverse() * atb;
    Some(DualVertex {
        position: solved.clamp(cell_min, cell_max),
        sample_count: samples.len(),
    })
}

fn remove_sliver_triangles(mesh: &mut Mesh, max_edge_ratio: f32, min_area_eps: f32) {
    let mut clean = Vec::with_capacity(mesh.indices.len());
    for tri in mesh.indices.chunks_exact(3) {
        let p0 = Vec3::from_array(mesh.vertices[tri[0] as usize].position);
        let p1 = Vec3::from_array(mesh.vertices[tri[1] as usize].position);
        let p2 = Vec3::from_array(mesh.vertices[tri[2] as usize].position);
        let e0 = p0.distance(p1);
        let e1 = p1.distance(p2);
        let e2 = p2.distance(p0);
        let max_edge = e0.max(e1).max(e2);
        let min_edge = e0.min(e1).min(e2).max(1e-6);
        let area = 0.5 * (p1 - p0).cross(p2 - p0).length();
        if area >= min_area_eps && max_edge / min_edge <= max_edge_ratio {
            clean.extend_from_slice(tri);
        }
    }
    mesh.indices = clean;
}

pub fn fill_boundary_loops(mesh: &mut Mesh) {
    let mut edge_counts: HashMap<(u32, u32), usize> = HashMap::new();
    let mut oriented_edges = Vec::new();
    for tri in mesh.indices.chunks_exact(3) {
        for &(a, b) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            let key = if a < b { (a, b) } else { (b, a) };
            *edge_counts.entry(key).or_insert(0) += 1;
            oriented_edges.push((a, b));
        }
    }

    let boundary_oriented: Vec<(u32, u32)> = oriented_edges
        .into_iter()
        .filter(|&(a, b)| {
            let key = if a < b { (a, b) } else { (b, a) };
            edge_counts.get(&key).copied().unwrap_or(0) == 1
        })
        .collect();

    let mut next_from: HashMap<u32, Vec<u32>> = HashMap::new();
    for (a, b) in &boundary_oriented {
        next_from.entry(*a).or_default().push(*b);
    }

    let mut visited = HashMap::<(u32, u32), bool>::new();
    for edge in boundary_oriented {
        if visited.contains_key(&edge) {
            continue;
        }
        let mut loop_vertices = vec![edge.0, edge.1];
        visited.insert(edge, true);
        let start = edge.0;
        let mut current = edge.1;
        let mut guard = 0usize;
        while current != start && guard < 1024 {
            guard += 1;
            let Some(nexts) = next_from.get(&current) else {
                break;
            };
            let mut advanced = false;
            for &next in nexts {
                if !visited.contains_key(&(current, next)) {
                    loop_vertices.push(next);
                    visited.insert((current, next), true);
                    current = next;
                    advanced = true;
                    break;
                }
            }
            if !advanced {
                break;
            }
        }
        if loop_vertices.len() < 4 || *loop_vertices.last().unwrap() != start {
            continue;
        }
        loop_vertices.pop();

        let mut centroid = Vec3::ZERO;
        let mut normal = Vec3::ZERO;
        for &idx in &loop_vertices {
            centroid += Vec3::from_array(mesh.vertices[idx as usize].position);
            normal += Vec3::from_array(mesh.vertices[idx as usize].normal);
        }
        centroid /= loop_vertices.len() as f32;
        normal = normal.normalize_or_zero();
        let center_idx = mesh.vertices.len() as u32;
        mesh.vertices.push(Vertex {
            position: centroid.to_array(),
            normal: normal.to_array(),
        });
        for pair in loop_vertices.windows(2) {
            mesh.indices
                .extend_from_slice(&[center_idx, pair[0], pair[1]]);
        }
        mesh.indices.extend_from_slice(&[
            center_idx,
            *loop_vertices.last().unwrap(),
            loop_vertices[0],
        ]);
    }
}

pub fn boundary_edge_count(mesh: &Mesh) -> usize {
    let mut edges: HashMap<(u32, u32), usize> = HashMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        for &(a, b) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            let key = if a < b { (a, b) } else { (b, a) };
            *edges.entry(key).or_insert(0) += 1;
        }
    }
    edges.values().filter(|&&count| count == 1).count()
}

fn corner_index(ix: usize, iy: usize, iz: usize, nx: usize, ny: usize) -> usize {
    ix + (nx + 1) * (iy + (ny + 1) * iz)
}

fn cell_corner_distances(
    point_distances: &[f32],
    i: usize,
    j: usize,
    k: usize,
    nx: usize,
    ny: usize,
) -> [f32; 8] {
    std::array::from_fn(|corner| {
        let [cx, cy, cz] = CUBE_CORNERS[corner];
        point_distances[corner_index(i + cx as usize, j + cy as usize, k + cz as usize, nx, ny)]
    })
}

pub fn extract_dual_contour_mesh(
    sdf: &dyn Sdf,
    bounds_min: Vec3,
    bounds_max: Vec3,
    cell_size: f32,
) -> Mesh {
    let size = bounds_max - bounds_min;
    let nx = ((size.x / cell_size).ceil().max(1.0)) as usize;
    let ny = ((size.y / cell_size).ceil().max(1.0)) as usize;
    let nz = ((size.z / cell_size).ceil().max(1.0)) as usize;
    let step = Vec3::new(size.x / nx as f32, size.y / ny as f32, size.z / nz as f32);

    let mut point_distances = vec![0.0f32; (nx + 1) * (ny + 1) * (nz + 1)];
    for k in 0..=nz {
        for j in 0..=ny {
            for i in 0..=nx {
                let p =
                    bounds_min + Vec3::new(i as f32 * step.x, j as f32 * step.y, k as f32 * step.z);
                point_distances[corner_index(i, j, k, nx, ny)] = sdf.distance(p);
            }
        }
    }

    let mut cells: HashMap<(usize, usize, usize), UniformDualCell> = HashMap::new();
    let mut vertices = Vec::new();
    let mut cell_to_vertex = HashMap::new();
    let eps = step.max_element() * 0.2;

    for k in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                let cell_min =
                    bounds_min + Vec3::new(i as f32 * step.x, j as f32 * step.y, k as f32 * step.z);
                let corner_distances = cell_corner_distances(&point_distances, i, j, k, nx, ny);
                let all_pos = corner_distances.iter().all(|&d| d >= 0.0);
                let all_neg = corner_distances.iter().all(|&d| d <= 0.0);
                if all_pos || all_neg {
                    continue;
                }
                let samples = sample_hermite_cell(sdf, cell_min, step.max_element());
                if let Some(dual) = solve_qef_vertex(&samples, cell_min, cell_min + step) {
                    let normal = sdf_gradient(sdf, dual.position, eps);
                    let vertex_index = vertices.len() as u32;
                    vertices.push(Vertex {
                        position: dual.position.to_array(),
                        normal: normal.to_array(),
                    });
                    cell_to_vertex.insert((i, j, k), vertex_index);
                    cells.insert(
                        (i, j, k),
                        UniformDualCell {
                            vertex: dual,
                            corner_distances,
                        },
                    );
                }
            }
        }
    }

    let mut indices = Vec::new();

    let push_quad = |indices: &mut Vec<u32>, a: u32, b: u32, c: u32, d: u32, flip: bool| {
        if flip {
            indices.extend_from_slice(&[a, c, b, a, d, c]);
        } else {
            indices.extend_from_slice(&[a, b, c, a, c, d]);
        }
    };

    // X-oriented edges
    if ny > 0 && nz > 0 {
        for i in 0..nx {
            for j in 1..ny {
                for k in 1..nz {
                    let d0 = point_distances[corner_index(i, j, k, nx, ny)];
                    let d1 = point_distances[corner_index(i + 1, j, k, nx, ny)];
                    if (d0 >= 0.0 && d1 >= 0.0) || (d0 <= 0.0 && d1 <= 0.0) {
                        continue;
                    }
                    let cells4 = [(i, j - 1, k - 1), (i, j, k - 1), (i, j, k), (i, j - 1, k)];
                    if let [Some(&a), Some(&b), Some(&c), Some(&d)] =
                        cells4.map(|idx| cell_to_vertex.get(&idx))
                    {
                        push_quad(&mut indices, a, b, c, d, d0 < 0.0);
                    }
                }
            }
        }
    }

    // Y-oriented edges
    if nx > 0 && nz > 0 {
        for i in 1..nx {
            for j in 0..ny {
                for k in 1..nz {
                    let d0 = point_distances[corner_index(i, j, k, nx, ny)];
                    let d1 = point_distances[corner_index(i, j + 1, k, nx, ny)];
                    if (d0 >= 0.0 && d1 >= 0.0) || (d0 <= 0.0 && d1 <= 0.0) {
                        continue;
                    }
                    let cells4 = [(i - 1, j, k - 1), (i, j, k - 1), (i, j, k), (i - 1, j, k)];
                    if let [Some(&a), Some(&b), Some(&c), Some(&d)] =
                        cells4.map(|idx| cell_to_vertex.get(&idx))
                    {
                        push_quad(&mut indices, a, b, c, d, d0 >= 0.0);
                    }
                }
            }
        }
    }

    // Z-oriented edges
    if nx > 0 && ny > 0 {
        for i in 1..nx {
            for j in 1..ny {
                for k in 0..nz {
                    let d0 = point_distances[corner_index(i, j, k, nx, ny)];
                    let d1 = point_distances[corner_index(i, j, k + 1, nx, ny)];
                    if (d0 >= 0.0 && d1 >= 0.0) || (d0 <= 0.0 && d1 <= 0.0) {
                        continue;
                    }
                    let cells4 = [(i - 1, j - 1, k), (i, j - 1, k), (i, j, k), (i - 1, j, k)];
                    if let [Some(&a), Some(&b), Some(&c), Some(&d)] =
                        cells4.map(|idx| cell_to_vertex.get(&idx))
                    {
                        push_quad(&mut indices, a, b, c, d, d0 < 0.0);
                    }
                }
            }
        }
    }

    let _ = cells
        .values()
        .map(|c| c.vertex.sample_count + c.corner_distances.len())
        .sum::<usize>();

    let mut mesh = Mesh { vertices, indices };
    remove_sliver_triangles(
        &mut mesh,
        25.0,
        step.max_element() * step.max_element() * 1e-4,
    );
    fill_boundary_loops(&mut mesh);
    mesh
}

pub fn extract_dual_contour_mesh_from_octree(
    sdf: &dyn Sdf,
    octree: &AdaptiveOctree,
    finest_cell_size: f32,
) -> Mesh {
    let leaves = collect_surface_leaves(octree);
    if leaves.is_empty() {
        return Mesh {
            vertices: Vec::new(),
            indices: Vec::new(),
        };
    }

    let mut domain_min = leaves[0].min;
    let mut domain_max = leaves[0].max;
    for leaf in &leaves[1..] {
        domain_min = domain_min.min(leaf.min);
        domain_max = domain_max.max(leaf.max);
    }
    let pad = Vec3::splat(finest_cell_size.max(0.05));
    let domain_min = domain_min - pad;
    let domain_max = domain_max + pad;
    let size = domain_max - domain_min;
    let nx = ((size.x / finest_cell_size).ceil().max(1.0)) as usize;
    let ny = ((size.y / finest_cell_size).ceil().max(1.0)) as usize;
    let nz = ((size.z / finest_cell_size).ceil().max(1.0)) as usize;
    let step = Vec3::new(size.x / nx as f32, size.y / ny as f32, size.z / nz as f32);

    let mut point_distances = vec![0.0f32; (nx + 1) * (ny + 1) * (nz + 1)];
    for k in 0..=nz {
        for j in 0..=ny {
            for i in 0..=nx {
                let p =
                    domain_min + Vec3::new(i as f32 * step.x, j as f32 * step.y, k as f32 * step.z);
                point_distances[corner_index(i, j, k, nx, ny)] = sdf.distance(p);
            }
        }
    }

    let mut leaf_vertices = Vec::new();
    let mut leaf_vertex_map = Vec::with_capacity(leaves.len());
    let eps = finest_cell_size.max(0.05) * 0.2;
    for leaf in &leaves {
        let cell_size = (leaf.max - leaf.min).max_element();
        let samples = sample_hermite_cell(sdf, leaf.min, cell_size);
        if let Some(dual) = solve_qef_vertex(&samples, leaf.min, leaf.max) {
            let normal = sdf_gradient(sdf, dual.position, eps);
            leaf_vertex_map.push(Some(leaf_vertices.len() as u32));
            leaf_vertices.push(Vertex {
                position: dual.position.to_array(),
                normal: normal.to_array(),
            });
        } else {
            leaf_vertex_map.push(None);
        }
    }

    let mut fine_cell_owner: HashMap<(usize, usize, usize), u32> = HashMap::new();
    for (leaf_idx, leaf) in leaves.iter().enumerate() {
        let Some(vertex_index) = leaf_vertex_map[leaf_idx] else {
            continue;
        };
        let min_i = (((leaf.min.x - domain_min.x) / step.x).round()).max(0.0) as usize;
        let min_j = (((leaf.min.y - domain_min.y) / step.y).round()).max(0.0) as usize;
        let min_k = (((leaf.min.z - domain_min.z) / step.z).round()).max(0.0) as usize;
        let max_i = (((leaf.max.x - domain_min.x) / step.x).round()).max(0.0) as usize;
        let max_j = (((leaf.max.y - domain_min.y) / step.y).round()).max(0.0) as usize;
        let max_k = (((leaf.max.z - domain_min.z) / step.z).round()).max(0.0) as usize;
        for k in min_k..max_k.min(nz) {
            for j in min_j..max_j.min(ny) {
                for i in min_i..max_i.min(nx) {
                    fine_cell_owner.insert((i, j, k), vertex_index);
                }
            }
        }
    }

    let mut indices = Vec::new();
    let push_quad = |indices: &mut Vec<u32>, a: u32, b: u32, c: u32, d: u32, flip: bool| {
        let ordered = if flip { [a, c, b, d] } else { [a, b, c, d] };
        let mut unique = Vec::with_capacity(4);
        for idx in ordered {
            if unique.last().copied() != Some(idx) && !unique.contains(&idx) {
                unique.push(idx);
            }
        }
        match unique.len() {
            3 => indices.extend_from_slice(&[unique[0], unique[1], unique[2]]),
            4 => indices.extend_from_slice(&[
                unique[0], unique[1], unique[2], unique[0], unique[2], unique[3],
            ]),
            _ => {}
        }
    };

    if ny > 0 && nz > 0 {
        for i in 0..nx {
            for j in 1..ny {
                for k in 1..nz {
                    let d0 = point_distances[corner_index(i, j, k, nx, ny)];
                    let d1 = point_distances[corner_index(i + 1, j, k, nx, ny)];
                    if (d0 >= 0.0 && d1 >= 0.0) || (d0 <= 0.0 && d1 <= 0.0) {
                        continue;
                    }
                    let cells4 = [(i, j - 1, k - 1), (i, j, k - 1), (i, j, k), (i, j - 1, k)];
                    if let [Some(&a), Some(&b), Some(&c), Some(&d)] =
                        cells4.map(|idx| fine_cell_owner.get(&idx))
                    {
                        push_quad(&mut indices, a, b, c, d, d0 < 0.0);
                    }
                }
            }
        }
    }
    if nx > 0 && nz > 0 {
        for i in 1..nx {
            for j in 0..ny {
                for k in 1..nz {
                    let d0 = point_distances[corner_index(i, j, k, nx, ny)];
                    let d1 = point_distances[corner_index(i, j + 1, k, nx, ny)];
                    if (d0 >= 0.0 && d1 >= 0.0) || (d0 <= 0.0 && d1 <= 0.0) {
                        continue;
                    }
                    let cells4 = [(i - 1, j, k - 1), (i, j, k - 1), (i, j, k), (i - 1, j, k)];
                    if let [Some(&a), Some(&b), Some(&c), Some(&d)] =
                        cells4.map(|idx| fine_cell_owner.get(&idx))
                    {
                        push_quad(&mut indices, a, b, c, d, d0 >= 0.0);
                    }
                }
            }
        }
    }
    if nx > 0 && ny > 0 {
        for i in 1..nx {
            for j in 1..ny {
                for k in 0..nz {
                    let d0 = point_distances[corner_index(i, j, k, nx, ny)];
                    let d1 = point_distances[corner_index(i, j, k + 1, nx, ny)];
                    if (d0 >= 0.0 && d1 >= 0.0) || (d0 <= 0.0 && d1 <= 0.0) {
                        continue;
                    }
                    let cells4 = [(i - 1, j - 1, k), (i, j - 1, k), (i, j, k), (i - 1, j, k)];
                    if let [Some(&a), Some(&b), Some(&c), Some(&d)] =
                        cells4.map(|idx| fine_cell_owner.get(&idx))
                    {
                        push_quad(&mut indices, a, b, c, d, d0 < 0.0);
                    }
                }
            }
        }
    }

    let mut mesh = Mesh {
        vertices: leaf_vertices,
        indices,
    };
    remove_sliver_triangles(
        &mut mesh,
        25.0,
        step.max_element() * step.max_element() * 1e-4,
    );
    fill_boundary_loops(&mut mesh);
    mesh
}

#[cfg(test)]
mod tests {
    use glam::Vec3;

    use super::*;
    use std::sync::Arc;

    use crate::export::adaptive_octree::{AdaptiveOctreeSettings, build_adaptive_octree};
    use crate::sdf::booleans::Union;
    use crate::sdf::primitives::{SdfBox, Sphere};

    #[test]
    fn sample_hermite_cell_finds_edge_crossings() {
        let sphere = Sphere::new(5.0);
        let samples = sample_hermite_cell(&sphere, Vec3::new(-4.0, -4.0, -4.0), 4.0);
        assert!(
            !samples.is_empty(),
            "surface cell should yield Hermite edge samples"
        );
    }

    #[test]
    fn sample_hermite_cell_returns_unit_normals() {
        let shape = Sphere::new(5.0);
        let samples = sample_hermite_cell(&shape, Vec3::new(-4.0, -4.0, -4.0), 4.0);
        assert!(
            samples
                .iter()
                .all(|s| (s.normal.length() - 1.0).abs() < 0.05)
        );
    }

    #[test]
    fn sample_hermite_cell_skips_cells_without_surface() {
        let solid = SdfBox::new(Vec3::splat(50.0));
        let samples = sample_hermite_cell(&solid, Vec3::new(-2.0, -2.0, -2.0), 2.0);
        assert!(
            samples.is_empty(),
            "fully interior cell should not emit edge crossings"
        );
    }

    #[test]
    fn solve_qef_vertex_returns_point_inside_cell() {
        let sphere = Sphere::new(5.0);
        let cell_min = Vec3::new(-4.0, -4.0, -4.0);
        let cell_size = 4.0;
        let samples = sample_hermite_cell(&sphere, cell_min, cell_size);
        let dual = solve_qef_vertex(&samples, cell_min, cell_min + Vec3::splat(cell_size)).unwrap();
        assert!(dual.position.cmple(cell_min + Vec3::splat(cell_size)).all());
        assert!(dual.position.cmpge(cell_min).all());
    }

    #[test]
    fn solve_qef_vertex_falls_back_on_degenerate_system() {
        let samples = vec![
            HermiteSample {
                edge_index: 0,
                position: Vec3::new(1.0, 1.0, 1.0),
                normal: Vec3::X,
            },
            HermiteSample {
                edge_index: 1,
                position: Vec3::new(1.0, 2.0, 1.0),
                normal: Vec3::X,
            },
        ];
        let dual = solve_qef_vertex(&samples, Vec3::ZERO, Vec3::splat(4.0)).unwrap();
        assert!(dual.position.x >= 0.0 && dual.position.x <= 4.0);
    }

    #[test]
    fn solve_qef_vertex_falls_back_for_too_few_samples() {
        let samples = vec![
            HermiteSample {
                edge_index: 0,
                position: Vec3::new(1.0, 1.0, 1.0),
                normal: Vec3::Y,
            },
            HermiteSample {
                edge_index: 1,
                position: Vec3::new(1.5, 1.0, 1.0),
                normal: Vec3::Y,
            },
        ];
        let dual = solve_qef_vertex(&samples, Vec3::ZERO, Vec3::splat(4.0)).unwrap();
        assert!(dual.position.x >= 0.0 && dual.position.x <= 4.0);
    }

    #[test]
    fn extract_dual_contour_mesh_builds_surface_for_sphere() {
        let sphere = Sphere::new(5.0);
        let mesh = extract_dual_contour_mesh(&sphere, Vec3::splat(-6.0), Vec3::splat(6.0), 1.5);
        assert!(!mesh.vertices.is_empty());
        assert!(!mesh.indices.is_empty());
        assert_eq!(mesh.indices.len() % 3, 0);
    }

    #[test]
    fn remove_sliver_triangles_drops_extreme_aspect_faces() {
        let mut mesh = Mesh {
            vertices: vec![
                Vertex {
                    position: [0.0, 0.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                },
                Vertex {
                    position: [10.0, 0.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                },
                Vertex {
                    position: [0.001, 0.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                },
            ],
            indices: vec![0, 1, 2],
        };
        remove_sliver_triangles(&mut mesh, 5.0, 1e-8);
        assert!(mesh.indices.is_empty());
    }

    #[test]
    fn extract_dual_contour_mesh_handles_thin_feature() {
        let thin = SdfBox::new(Vec3::new(10.0, 1.0, 0.4));
        let mesh = extract_dual_contour_mesh(
            &thin,
            Vec3::new(-6.0, -2.0, -1.0),
            Vec3::new(6.0, 2.0, 1.0),
            0.25,
        );
        assert!(!mesh.vertices.is_empty(), "thin feature should still mesh");
    }

    #[test]
    fn extract_dual_contour_mesh_handles_junction_shape() {
        let a: Arc<dyn crate::sdf::Sdf> = Arc::new(Sphere::new(4.0));
        let b: Arc<dyn crate::sdf::Sdf> = Arc::new(crate::sdf::transforms::Translate::new(
            Arc::new(SdfBox::new(Vec3::new(8.0, 2.0, 2.0))),
            Vec3::new(2.0, 0.0, 0.0),
        ));
        let shape = Union::new(a, b);
        let mesh = extract_dual_contour_mesh(&shape, Vec3::splat(-6.0), Vec3::splat(6.0), 0.75);
        assert!(
            !mesh.indices.is_empty(),
            "junction geometry should produce triangles"
        );
    }

    #[test]
    fn extract_dual_contour_mesh_from_octree_handles_mixed_leaf_sizes() {
        let sphere = Sphere::new(8.0);
        let octree = build_adaptive_octree(
            &sphere,
            Vec3::splat(-10.0),
            Vec3::splat(10.0),
            &AdaptiveOctreeSettings {
                max_depth: 5,
                min_cell_size_mm: 1.0,
                surface_band_mm: 2.0,
                error_tolerance_mm: 0.25,
                curvature_refine_threshold: 0.1,
                feature_scale_factor: 2.0,
                max_leaf_cells: 16_384,
            },
        );
        let mesh = extract_dual_contour_mesh_from_octree(&sphere, &octree, 1.0);
        assert!(!mesh.vertices.is_empty());
        assert!(!mesh.indices.is_empty());
    }

    #[test]
    fn extract_dual_contour_mesh_from_octree_reduces_boundary_edges() {
        let sphere = Sphere::new(8.0);
        let octree = build_adaptive_octree(
            &sphere,
            Vec3::splat(-10.0),
            Vec3::splat(10.0),
            &AdaptiveOctreeSettings {
                max_depth: 5,
                min_cell_size_mm: 1.0,
                surface_band_mm: 2.0,
                error_tolerance_mm: 0.25,
                curvature_refine_threshold: 0.1,
                feature_scale_factor: 2.0,
                max_leaf_cells: 16_384,
            },
        );
        let mesh = extract_dual_contour_mesh_from_octree(&sphere, &octree, 1.0);
        assert!(
            boundary_edge_count(&mesh) < mesh.indices.len() / 3,
            "stitched octree mesh should not devolve into mostly open boundaries"
        );
    }
}
