use std::collections::HashMap;

use glam::Vec3;
use implicit_cad::export::dual_contouring::analyze_mesh_topology;
use implicit_cad::export::{ExportBackend, build_export_mesh_with_backend};
use implicit_cad::mesh::Mesh;
use implicit_cad::sdf::aerospace::{AirfoilExportOptions, wing_with_airfoil_export_safe};

fn main() {
    let wing = wing_with_airfoil_export_safe(
        "2212",
        14.0,
        9.0,
        36.0,
        4.0,
        2.0,
        -1.0,
        AirfoilExportOptions {
            min_trailing_edge_thickness_mm: 1.5,
            min_leading_edge_radius_mm: 0.8,
        },
    );
    let bounds_min = Vec3::new(-4.37, -22.41, -6.29);
    let bounds_max = Vec3::new(20.63, 22.59, 8.71);
    let result = build_export_mesh_with_backend(
        &wing,
        bounds_min,
        bounds_max,
        1.0,
        ExportBackend::AdaptiveDualContouring,
        false,
    );
    let topology = analyze_mesh_topology(&result.mesh);
    println!(
        "vertices={} triangles={} boundary={} non_manifold={} degenerate={} duplicate={}",
        result.mesh.vertices.len(),
        result.mesh.indices.len() / 3,
        topology.boundary_edges,
        topology.non_manifold_edges,
        topology.degenerate_triangles,
        topology.duplicate_triangles
    );

    let mut edge_counts: HashMap<(u32, u32), usize> = HashMap::new();
    let mut triangle_counts: HashMap<[u32; 3], usize> = HashMap::new();
    for tri in result.mesh.indices.chunks_exact(3) {
        let mut key = [tri[0], tri[1], tri[2]];
        key.sort_unstable();
        *triangle_counts.entry(key).or_insert(0) += 1;
        for &(a, b) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            let key = if a < b { (a, b) } else { (b, a) };
            *edge_counts.entry(key).or_insert(0) += 1;
        }
    }
    let mut duplicate_histogram: HashMap<usize, usize> = HashMap::new();
    for count in triangle_counts.values().copied().filter(|count| *count > 1) {
        *duplicate_histogram.entry(count).or_insert(0) += 1;
    }
    let mut non_manifold_histogram: HashMap<usize, usize> = HashMap::new();
    for count in edge_counts.values().copied().filter(|count| *count > 2) {
        *non_manifold_histogram.entry(count).or_insert(0) += 1;
    }
    println!("duplicate_triangle_multiplicity={:?}", duplicate_histogram);
    println!(
        "non_manifold_edge_multiplicity={:?}",
        non_manifold_histogram
    );
    for cap in [1usize, 2, 3] {
        let mut kept_counts: HashMap<[u32; 3], usize> = HashMap::new();
        let mut capped_indices = Vec::new();
        for tri in result.mesh.indices.chunks_exact(3) {
            let mut key = [tri[0], tri[1], tri[2]];
            key.sort_unstable();
            let count = kept_counts.entry(key).or_insert(0);
            if *count < cap {
                capped_indices.extend_from_slice(tri);
                *count += 1;
            }
        }
        let capped = Mesh {
            vertices: result.mesh.vertices.clone(),
            indices: capped_indices,
        };
        let capped_topology = analyze_mesh_topology(&capped);
        println!(
            "cap_duplicate_triangles_{}: triangles={} boundary={} non_manifold={} duplicate={}",
            cap,
            capped.indices.len() / 3,
            capped_topology.boundary_edges,
            capped_topology.non_manifold_edges,
            capped_topology.duplicate_triangles
        );
    }
    let mut directed_counts: HashMap<(u32, u32), usize> = HashMap::new();
    let mut half_edge_indices = Vec::new();
    for tri in result.mesh.indices.chunks_exact(3) {
        let edges = [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])];
        let duplicate_direction = edges
            .iter()
            .any(|edge| directed_counts.get(edge).copied().unwrap_or(0) >= 1);
        let saturated_counter_edge = edges
            .iter()
            .any(|&(a, b)| directed_counts.get(&(b, a)).copied().unwrap_or(0) >= 2);
        if duplicate_direction || saturated_counter_edge {
            continue;
        }
        half_edge_indices.extend_from_slice(tri);
        for edge in edges {
            *directed_counts.entry(edge).or_insert(0) += 1;
        }
    }
    let half_edge = Mesh {
        vertices: result.mesh.vertices.clone(),
        indices: half_edge_indices,
    };
    let half_edge_topology = analyze_mesh_topology(&half_edge);
    println!(
        "directed_half_edge_filter: triangles={} boundary={} non_manifold={} duplicate={}",
        half_edge.indices.len() / 3,
        half_edge_topology.boundary_edges,
        half_edge_topology.non_manifold_edges,
        half_edge_topology.duplicate_triangles
    );

    let mut bmin = Vec3::splat(f32::INFINITY);
    let mut bmax = Vec3::splat(f32::NEG_INFINITY);
    let mut samples = Vec::new();
    for ((a, b), count) in edge_counts {
        if count != 1 {
            continue;
        }
        let pa = Vec3::from_array(result.mesh.vertices[a as usize].position);
        let pb = Vec3::from_array(result.mesh.vertices[b as usize].position);
        bmin = bmin.min(pa).min(pb);
        bmax = bmax.max(pa).max(pb);
        if samples.len() < 16 {
            samples.push((pa, pb));
        }
    }
    println!(
        "boundary_bbox=({:.3},{:.3},{:.3})..({:.3},{:.3},{:.3})",
        bmin.x, bmin.y, bmin.z, bmax.x, bmax.y, bmax.z
    );
    for (i, (a, b)) in samples.iter().enumerate() {
        println!(
            "boundary_sample_{}=({:.3},{:.3},{:.3})->({:.3},{:.3},{:.3})",
            i, a.x, a.y, a.z, b.x, b.y, b.z
        );
    }
}
