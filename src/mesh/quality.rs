use std::collections::{HashMap, HashSet, VecDeque};

use glam::Vec3;
use serde::Serialize;

use super::{Mesh, TriangleMesh};

#[derive(Clone, Debug, Default, Serialize)]
pub struct MeshQualityReport {
    pub vertex_count: usize,
    pub triangle_count: usize,
    pub connected_component_count: usize,
    pub degenerate_triangle_count: usize,
    pub manifold_edge_error_count: usize,
    pub non_manifold_edge_count: usize,
    pub boundary_edge_count: usize,
    pub watertight: bool,
    pub min_edge_length_mm: f32,
    pub max_edge_length_mm: f32,
    pub avg_triangle_aspect_ratio: f32,
    pub max_triangle_aspect_ratio: f32,
}

fn canonical_edge(a: u32, b: u32) -> (u32, u32) {
    if a < b { (a, b) } else { (b, a) }
}

fn analyze_positions(vertices: &[Vec3], triangles: &[[u32; 3]]) -> MeshQualityReport {
    let mut report = MeshQualityReport {
        vertex_count: vertices.len(),
        triangle_count: triangles.len(),
        ..Default::default()
    };
    if triangles.is_empty() || vertices.is_empty() {
        return report;
    }

    let mut edge_counts: HashMap<(u32, u32), usize> = HashMap::new();
    let mut tri_adjacency: Vec<HashSet<usize>> = vec![HashSet::new(); triangles.len()];
    let mut edge_to_triangles: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    let mut min_edge = f32::MAX;
    let mut max_edge = 0.0f32;
    let mut aspect_sum = 0.0f32;
    let mut aspect_count = 0usize;
    let mut aspect_max = 0.0f32;

    for (tri_idx, tri) in triangles.iter().enumerate() {
        let [a, b, c] = *tri;
        if a as usize >= vertices.len()
            || b as usize >= vertices.len()
            || c as usize >= vertices.len()
        {
            report.degenerate_triangle_count += 1;
            continue;
        }
        let p0 = vertices[a as usize];
        let p1 = vertices[b as usize];
        let p2 = vertices[c as usize];
        let e0 = p0.distance(p1);
        let e1 = p1.distance(p2);
        let e2 = p2.distance(p0);
        min_edge = min_edge.min(e0.min(e1).min(e2));
        max_edge = max_edge.max(e0.max(e1).max(e2));

        let area2 = (p1 - p0).cross(p2 - p0).length_squared();
        if area2 < 1e-16 {
            report.degenerate_triangle_count += 1;
        } else {
            let longest = e0.max(e1).max(e2);
            let area = 0.5 * area2.sqrt();
            let altitude = if longest > 1e-6 {
                (2.0 * area) / longest
            } else {
                0.0
            };
            let aspect = if altitude > 1e-6 {
                longest / altitude
            } else {
                f32::INFINITY
            };
            aspect_sum += aspect;
            aspect_count += 1;
            aspect_max = aspect_max.max(aspect);
        }

        for &(u, v) in &[(a, b), (b, c), (c, a)] {
            let key = canonical_edge(u, v);
            *edge_counts.entry(key).or_insert(0) += 1;
            edge_to_triangles.entry(key).or_default().push(tri_idx);
        }
    }

    for tri_ids in edge_to_triangles.values() {
        if tri_ids.len() < 2 {
            continue;
        }
        for &lhs in tri_ids {
            for &rhs in tri_ids {
                if lhs != rhs {
                    tri_adjacency[lhs].insert(rhs);
                }
            }
        }
    }

    let mut visited = vec![false; triangles.len()];
    for start in 0..triangles.len() {
        if visited[start] {
            continue;
        }
        report.connected_component_count += 1;
        let mut queue = VecDeque::from([start]);
        visited[start] = true;
        while let Some(cur) = queue.pop_front() {
            for &next in &tri_adjacency[cur] {
                if !visited[next] {
                    visited[next] = true;
                    queue.push_back(next);
                }
            }
        }
    }

    report.boundary_edge_count = edge_counts.values().filter(|&&count| count == 1).count();
    report.non_manifold_edge_count = edge_counts.values().filter(|&&count| count > 2).count();
    report.manifold_edge_error_count = edge_counts.values().filter(|&&count| count != 2).count();
    report.watertight = report.connected_component_count == 1
        && report.degenerate_triangle_count == 0
        && report.boundary_edge_count == 0
        && report.non_manifold_edge_count == 0;
    report.min_edge_length_mm = if min_edge.is_finite() { min_edge } else { 0.0 };
    report.max_edge_length_mm = max_edge;
    report.avg_triangle_aspect_ratio = if aspect_count > 0 {
        aspect_sum / aspect_count as f32
    } else {
        0.0
    };
    report.max_triangle_aspect_ratio = aspect_max;
    report
}

pub fn analyze_mesh_quality(mesh: &Mesh) -> MeshQualityReport {
    let vertices: Vec<Vec3> = mesh
        .vertices
        .iter()
        .map(|v| Vec3::from_array(v.position))
        .collect();
    let triangles: Vec<[u32; 3]> = mesh
        .indices
        .chunks_exact(3)
        .map(|tri| [tri[0], tri[1], tri[2]])
        .collect();
    analyze_positions(&vertices, &triangles)
}

pub fn analyze_triangle_mesh_quality(mesh: &TriangleMesh) -> MeshQualityReport {
    analyze_positions(&mesh.vertices, &mesh.triangles)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::Vertex;

    #[test]
    fn watertight_tetrahedron_reports_closed_topology() {
        let mesh = Mesh {
            vertices: vec![
                Vertex {
                    position: [1.0, 1.0, 1.0],
                    normal: [0.0; 3],
                },
                Vertex {
                    position: [-1.0, -1.0, 1.0],
                    normal: [0.0; 3],
                },
                Vertex {
                    position: [-1.0, 1.0, -1.0],
                    normal: [0.0; 3],
                },
                Vertex {
                    position: [1.0, -1.0, -1.0],
                    normal: [0.0; 3],
                },
            ],
            indices: vec![0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3],
        };
        let report = analyze_mesh_quality(&mesh);
        assert_eq!(report.connected_component_count, 1);
        assert_eq!(report.boundary_edge_count, 0);
        assert_eq!(report.non_manifold_edge_count, 0);
        assert!(report.watertight);
    }

    #[test]
    fn single_triangle_reports_open_boundary() {
        let mesh = TriangleMesh {
            vertices: vec![Vec3::ZERO, Vec3::X, Vec3::Y],
            triangles: vec![[0, 1, 2]],
            normals: vec![Vec3::Z],
            bounds_min: Vec3::ZERO,
            bounds_max: Vec3::ONE,
        };
        let report = analyze_triangle_mesh_quality(&mesh);
        assert_eq!(report.boundary_edge_count, 3);
        assert!(!report.watertight);
    }
}
