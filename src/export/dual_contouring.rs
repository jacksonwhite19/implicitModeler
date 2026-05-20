use std::collections::HashMap;
use std::io::Write;
use std::time::Instant;

use glam::{Mat3, Vec3};
use rayon::prelude::*;

use crate::export::adaptive_octree::{
    AdaptiveOctree, OctreeNode, SurfaceLeafCell, collect_leaf_cells, collect_surface_leaves,
};
use crate::mesh::marching_cubes::{CUBE_CORNERS, EDGE_VERTICES};
use crate::mesh::{Mesh, Vertex};
use crate::sdf::Sdf;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MeshTopologyStats {
    pub boundary_edges: usize,
    pub non_manifold_edges: usize,
    pub duplicate_triangles: usize,
    pub degenerate_triangles: usize,
}

#[derive(Default, Debug, Clone)]
pub struct DualContouringTelemetry {
    pub total_leaf_nodes: usize,
    pub leaf_counts_per_depth: HashMap<u32, usize>,
    pub total_active_edges: usize,
    pub exact_midpoint_fallbacks: usize,
    pub qef_total_solves: usize,
    pub qef_success_count: usize,
    pub qef_singular_matrix_fallbacks: usize,
    pub qef_out_of_bounds_clamped: usize,
    pub qef_average_conditioning: f32,
    pub axis_snapped_normals: usize,
    pub degenerate_gradient_guards: usize,
    pub duration_octree_walk_ms: u128,
    pub duration_qef_solve_ms: u128,
    pub duration_edge_emission_ms: u128,
    pub duration_write_io_ms: u128,
}

impl DualContouringTelemetry {
    fn record_conditioning(&mut self, conditioning: f32) {
        let previous = self.qef_total_solves.saturating_sub(1) as f32;
        self.qef_average_conditioning = ((self.qef_average_conditioning * previous) + conditioning)
            / self.qef_total_solves as f32;
    }

    fn record_normal(&mut self, raw_gradient: Vec3) {
        let len = raw_gradient.length();
        if len < 1e-5 {
            self.degenerate_gradient_guards += 1;
            return;
        }
        if is_axis_snapped_gradient(raw_gradient) {
            self.axis_snapped_normals += 1;
        }
    }
}

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

#[derive(Clone, Debug)]
struct AdaptiveActiveEdge {
    d0: f32,
    d1: f32,
    start: Vec3,
    end: Vec3,
    normal: Vec3,
    owners: [Option<usize>; 4],
}

type QuantizedEdgeKey = u64;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
struct EdgeKey {
    axis: u8,
    start: [u16; 3],
    end: [u16; 3],
}

#[derive(Clone, Debug)]
struct EdgeRegistryEntry {
    axis: u8,
    start: Vec3,
    end: Vec3,
    owners: Vec<usize>,
}

#[derive(Clone, Debug, Default)]
struct EmitterAudit {
    total_edge_keys: usize,
    crossing_edges: usize,
    four_owner_crossing_edges: usize,
    four_surface_vertex_edges: usize,
    three_owner_transition_edges: usize,
    emitter_mismatches: usize,
    missing_surface_vertex_edges: usize,
    logged_mismatches: usize,
    all_leaf_count: usize,
    surface_leaf_count: usize,
    class_counts_by_axis: [[usize; EDGE_INTERVAL_CLASS_COUNT]; 3],
}

const EDGE_INTERVAL_CLASS_COUNT: usize = 8;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum EdgeIntervalClass {
    InteriorValid4,
    InteriorValid3Transition,
    InteriorDuplicateOwner,
    InteriorMissingOwner,
    RootBoundary,
    NoDualVertex,
    DegenerateInterval,
    Unknown,
}

impl EdgeIntervalClass {
    fn index(self) -> usize {
        match self {
            EdgeIntervalClass::InteriorValid4 => 0,
            EdgeIntervalClass::InteriorValid3Transition => 1,
            EdgeIntervalClass::InteriorDuplicateOwner => 2,
            EdgeIntervalClass::InteriorMissingOwner => 3,
            EdgeIntervalClass::RootBoundary => 4,
            EdgeIntervalClass::NoDualVertex => 5,
            EdgeIntervalClass::DegenerateInterval => 6,
            EdgeIntervalClass::Unknown => 7,
        }
    }

    fn label(self) -> &'static str {
        match self {
            EdgeIntervalClass::InteriorValid4 => "InteriorValid4",
            EdgeIntervalClass::InteriorValid3Transition => "InteriorValid3Transition",
            EdgeIntervalClass::InteriorDuplicateOwner => "InteriorDuplicateOwner",
            EdgeIntervalClass::InteriorMissingOwner => "InteriorMissingOwner",
            EdgeIntervalClass::RootBoundary => "RootBoundary",
            EdgeIntervalClass::NoDualVertex => "NoDualVertex",
            EdgeIntervalClass::DegenerateInterval => "DegenerateInterval",
            EdgeIntervalClass::Unknown => "Unknown",
        }
    }

    fn all() -> [EdgeIntervalClass; EDGE_INTERVAL_CLASS_COUNT] {
        [
            EdgeIntervalClass::InteriorValid4,
            EdgeIntervalClass::InteriorValid3Transition,
            EdgeIntervalClass::InteriorDuplicateOwner,
            EdgeIntervalClass::InteriorMissingOwner,
            EdgeIntervalClass::RootBoundary,
            EdgeIntervalClass::NoDualVertex,
            EdgeIntervalClass::DegenerateInterval,
            EdgeIntervalClass::Unknown,
        ]
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct QuantizedLeafKey {
    min: [i64; 3],
    max: [i64; 3],
}

#[derive(Clone, Copy, Debug)]
struct DyadicLeaf {
    source_idx: usize,
    min: [u16; 3],
    max: [u16; 3],
    world_min: Vec3,
    world_max: Vec3,
    is_surface_leaf: bool,
}

pub fn estimate_gradient(sdf: &dyn Sdf, p: Vec3, eps: f32) -> Vec3 {
    estimate_gradient_raw(sdf, p, eps).normalize_or_zero()
}

fn estimate_gradient_dynamic(sdf: &dyn Sdf, p: Vec3, cell_size: f32) -> Vec3 {
    estimate_gradient_dynamic_raw(sdf, p, cell_size).normalize_or_zero()
}

fn estimate_gradient_with_telemetry(
    sdf: &dyn Sdf,
    p: Vec3,
    cell_size: f32,
    telemetry: &mut DualContouringTelemetry,
) -> Vec3 {
    let raw = estimate_gradient_dynamic_raw(sdf, p, cell_size);
    telemetry.record_normal(raw);
    raw.normalize_or_zero()
}

fn estimate_gradient_dynamic_raw(sdf: &dyn Sdf, p: Vec3, cell_size: f32) -> Vec3 {
    let size = cell_size.max(0.05);
    let eps = (size * 1e-4).max(1e-5);
    let raw = estimate_gradient_raw(sdf, p, eps);
    if raw.length() >= 1e-5 && !is_axis_snapped_gradient(raw) {
        raw
    } else {
        estimate_gradient_raw(sdf, p, eps * 10.0)
    }
}

fn is_axis_snapped_gradient(raw_gradient: Vec3) -> bool {
    let len = raw_gradient.length();
    if len < 1e-5 {
        return false;
    }
    let n = raw_gradient / len;
    (n.x.abs() > 1.0 - 1e-6 && n.y.abs() < 1e-6 && n.z.abs() < 1e-6)
        || (n.y.abs() > 1.0 - 1e-6 && n.x.abs() < 1e-6 && n.z.abs() < 1e-6)
        || (n.z.abs() > 1.0 - 1e-6 && n.x.abs() < 1e-6 && n.y.abs() < 1e-6)
}

fn estimate_gradient_raw(sdf: &dyn Sdf, p: Vec3, eps: f32) -> Vec3 {
    Vec3::new(
        sdf.distance(p + Vec3::X * eps) - sdf.distance(p - Vec3::X * eps),
        sdf.distance(p + Vec3::Y * eps) - sdf.distance(p - Vec3::Y * eps),
        sdf.distance(p + Vec3::Z * eps) - sdf.distance(p - Vec3::Z * eps),
    ) / (2.0 * eps)
}

fn refine_edge_root(sdf: &dyn Sdf, start: Vec3, end: Vec3, d0: f32, d1: f32) -> (f32, f32) {
    let mut t_min = 0.0_f32;
    let mut t_max = 1.0_f32;
    let mut d_min = d0;
    let mut t_prev = if d0.abs() <= d1.abs() { 0.0 } else { 1.0 };
    let mut d_prev = if d0.abs() <= d1.abs() { d0 } else { d1 };
    let mut t = (-d0 / (d1 - d0)).clamp(0.0, 1.0);
    let mut d = sdf.distance(start.lerp(end, t));

    for _ in 0..4 {
        if d.abs() < 1e-5 {
            break;
        }
        if d_min.signum() == d.signum() {
            t_min = t;
            d_min = d;
        } else {
            t_max = t;
        }

        let denom = d - d_prev;
        let secant = if denom.abs() > 1e-8 {
            t - d * (t - t_prev) / denom
        } else {
            f32::NAN
        };
        let t_next = if secant.is_finite() && secant > t_min && secant < t_max {
            secant
        } else {
            (t_min + t_max) * 0.5
        };

        t_prev = t;
        d_prev = d;
        t = t_next.clamp(0.0, 1.0);
        d = sdf.distance(start.lerp(end, t));
    }

    (t, d)
}

fn push_triangle_if_valid(
    indices: &mut Vec<u32>,
    vertices: &[Vertex],
    mut tri: [u32; 3],
    target_normal: Vec3,
    min_area: f32,
) {
    if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
        return;
    }
    let p0 = Vec3::from_array(vertices[tri[0] as usize].position);
    let p1 = Vec3::from_array(vertices[tri[1] as usize].position);
    let p2 = Vec3::from_array(vertices[tri[2] as usize].position);
    let normal = (p1 - p0).cross(p2 - p0);
    let area = 0.5 * normal.length();
    if area > min_area {
        let normal_len = normal.length();
        let target_len = target_normal.length();
        let alignment = if normal_len > 1e-8 && target_len > 1e-5 {
            normal.dot(target_normal) / (normal_len * target_len)
        } else {
            0.0
        };
        if alignment < -1e-3 {
            tri.swap(1, 2);
        }
        indices.extend_from_slice(&tri);
    }
}

fn make_duplicate_slots_contiguous(mut quad: [u32; 4]) -> [u32; 4] {
    if quad[0] == quad[2] && quad[0] != quad[1] && quad[0] != quad[3] {
        quad.swap(2, 3);
    } else if quad[1] == quad[3] && quad[1] != quad[0] && quad[1] != quad[2] {
        quad.swap(0, 1);
    }
    quad
}

fn push_sign_oriented_quad(
    indices: &mut Vec<u32>,
    vertices: &[Vertex],
    quad: [u32; 4],
    d0: f32,
    d1: f32,
    target_normal: Vec3,
    min_area: f32,
) {
    let quad = make_duplicate_slots_contiguous(quad);
    let unique_count = {
        let mut unique = Vec::with_capacity(4);
        for idx in quad {
            if !unique.contains(&idx) {
                unique.push(idx);
            }
        }
        unique.len()
    };
    if unique_count < 3 {
        return;
    }

    let tris = if d0 < 0.0 && d1 > 0.0 {
        [[quad[0], quad[1], quad[2]], [quad[0], quad[2], quad[3]]]
    } else {
        [[quad[0], quad[2], quad[1]], [quad[0], quad[3], quad[2]]]
    };
    for tri in tris {
        push_triangle_if_valid(indices, vertices, tri, target_normal, min_area);
    }
}

fn cell_corner_positions(cell_min: Vec3, cell_size: Vec3) -> [Vec3; 8] {
    std::array::from_fn(|i| {
        let [cx, cy, cz] = CUBE_CORNERS[i];
        cell_min + Vec3::new(cx as f32, cy as f32, cz as f32) * cell_size
    })
}

pub fn sample_hermite_cell(sdf: &dyn Sdf, cell_min: Vec3, cell_size: f32) -> Vec<HermiteSample> {
    sample_hermite_cell_sized(sdf, cell_min, Vec3::splat(cell_size))
}

pub fn sample_hermite_cell_sized(
    sdf: &dyn Sdf,
    cell_min: Vec3,
    cell_size: Vec3,
) -> Vec<HermiteSample> {
    let corners = cell_corner_positions(cell_min, cell_size);
    let corner_distances: [f32; 8] = std::array::from_fn(|i| sdf.distance(corners[i]));
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
        let normal = estimate_gradient_dynamic(sdf, position, cell_size.min_element());
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
    solve_qef_vertex_with_telemetry(samples, cell_min, cell_max, None)
}

fn solve_qef_vertex_with_telemetry(
    samples: &[HermiteSample],
    cell_min: Vec3,
    cell_max: Vec3,
    mut telemetry: Option<&mut DualContouringTelemetry>,
) -> Option<DualVertex> {
    if samples.is_empty() {
        return None;
    }
    if let Some(t) = telemetry.as_deref_mut() {
        t.qef_total_solves += 1;
    }

    let mut fallback = Vec3::ZERO;
    for sample in samples {
        fallback += sample.position;
    }
    fallback /= samples.len() as f32;
    let mut ata = Mat3::ZERO;
    let mut atb = Vec3::ZERO;
    let mut valid_normals = 0usize;
    for sample in samples {
        let n = sample.normal.normalize_or_zero();
        if n == Vec3::ZERO {
            continue;
        }
        valid_normals += 1;
        let outer = Mat3::from_cols(n * n.x, n * n.y, n * n.z);
        ata += outer;
        atb += n * n.dot(sample.position);
    }
    let regularization = 1e-7;
    let determinant_before_regularization = ata.determinant().abs();
    ata += Mat3::IDENTITY * regularization;
    atb += fallback * regularization;
    if let Some(t) = telemetry.as_deref_mut() {
        t.record_conditioning(determinant_before_regularization);
    }

    let solved = ata.inverse() * atb;
    if !solved.is_finite() || valid_normals == 0 {
        if let Some(t) = telemetry.as_deref_mut() {
            t.qef_singular_matrix_fallbacks += 1;
        }
        return Some(DualVertex {
            position: fallback.clamp(cell_min, cell_max),
            sample_count: samples.len(),
        });
    }
    let cell_size = cell_max - cell_min;
    let relaxed = cell_size * 0.0;
    let relaxed_min = cell_min - relaxed;
    let relaxed_max = cell_max + relaxed;
    let outside_relaxed = solved.cmplt(relaxed_min).any() || solved.cmpgt(relaxed_max).any();
    if outside_relaxed {
        if let Some(t) = telemetry.as_deref_mut() {
            t.qef_out_of_bounds_clamped += 1;
        }
    }
    if let Some(t) = telemetry.as_deref_mut() {
        t.qef_success_count += 1;
    }

    Some(DualVertex {
        position: solved.clamp(relaxed_min, relaxed_max),
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

fn remove_duplicate_triangles(mesh: &mut Mesh) {
    let mut seen = HashMap::<[u32; 3], ()>::new();
    let mut clean = Vec::with_capacity(mesh.indices.len());
    for tri in mesh.indices.chunks_exact(3) {
        let mut key = [tri[0], tri[1], tri[2]];
        key.sort_unstable();
        if seen.insert(key, ()).is_none() {
            clean.extend_from_slice(tri);
        }
    }
    mesh.indices = clean;
}

fn remove_degenerate_triangles(mesh: &mut Mesh, min_area_eps: f32) {
    let mut clean = Vec::with_capacity(mesh.indices.len());
    for tri in mesh.indices.chunks_exact(3) {
        if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
            continue;
        }
        let p0 = Vec3::from_array(mesh.vertices[tri[0] as usize].position);
        let p1 = Vec3::from_array(mesh.vertices[tri[1] as usize].position);
        let p2 = Vec3::from_array(mesh.vertices[tri[2] as usize].position);
        if 0.5 * (p1 - p0).cross(p2 - p0).length() > min_area_eps {
            clean.extend_from_slice(tri);
        }
    }
    mesh.indices = clean;
}

fn weld_vertices_quantized(mesh: &Mesh, scale: f32) -> Mesh {
    if mesh.vertices.is_empty() {
        return mesh.clone();
    }
    let mut vertex_map = HashMap::<[i64; 3], u32>::new();
    let mut vertices = Vec::<Vertex>::new();
    let mut remap = vec![0u32; mesh.vertices.len()];

    for (idx, vertex) in mesh.vertices.iter().enumerate() {
        let key = quantize(vertex.position, scale);
        let mapped = *vertex_map.entry(key).or_insert_with(|| {
            let new_idx = vertices.len() as u32;
            vertices.push(*vertex);
            new_idx
        });
        remap[idx] = mapped;
    }

    let mut indices = Vec::with_capacity(mesh.indices.len());
    let mut seen = HashMap::<[u32; 3], ()>::new();
    for tri in mesh.indices.chunks_exact(3) {
        let mapped = [
            remap[tri[0] as usize],
            remap[tri[1] as usize],
            remap[tri[2] as usize],
        ];
        if mapped[0] == mapped[1] || mapped[1] == mapped[2] || mapped[2] == mapped[0] {
            continue;
        }
        let mut key = mapped;
        key.sort_unstable();
        if seen.insert(key, ()).is_none() {
            indices.extend_from_slice(&mapped);
        }
    }

    Mesh { vertices, indices }
}

fn quantize(pos: [f32; 3], scale: f32) -> [i64; 3] {
    [
        (pos[0] * scale).round() as i64,
        (pos[1] * scale).round() as i64,
        (pos[2] * scale).round() as i64,
    ]
}

fn filter_directed_half_edges(mesh: &Mesh) -> Mesh {
    let mut directed_counts: HashMap<(u32, u32), usize> = HashMap::new();
    let mut indices = Vec::with_capacity(mesh.indices.len());
    for tri in mesh.indices.chunks_exact(3) {
        if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
            continue;
        }
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
        indices.extend_from_slice(tri);
        for edge in edges {
            *directed_counts.entry(edge).or_insert(0) += 1;
        }
    }
    Mesh {
        vertices: mesh.vertices.clone(),
        indices,
    }
}

pub fn repair_non_manifold_edges_with_refill(mesh: &Mesh, min_area_eps: f32) -> Mesh {
    let mut best = mesh.clone();
    let mut best_topology = analyze_mesh_topology(&best);
    for _ in 0..8 {
        if best_topology.non_manifold_edges == 0 {
            break;
        }
        let mut candidate = filter_directed_half_edges(&best);
        if analyze_mesh_topology(&candidate).boundary_edges > 0 {
            fill_boundary_loops(&mut candidate);
            fill_undirected_boundary_cycles(&mut candidate);
        }
        remove_duplicate_triangles(&mut candidate);
        remove_degenerate_triangles(&mut candidate, min_area_eps);
        let candidate_topology = analyze_mesh_topology(&candidate);
        let improves = candidate_topology.non_manifold_edges < best_topology.non_manifold_edges
            || (candidate_topology.non_manifold_edges == best_topology.non_manifold_edges
                && candidate_topology.boundary_edges < best_topology.boundary_edges);
        if !improves {
            break;
        }
        best = candidate;
        best_topology = candidate_topology;
    }
    best
}

pub fn prune_small_triangle_components(mesh: &Mesh, min_faces: usize, min_fraction: f32) -> Mesh {
    prune_triangle_components(mesh, min_faces, min_fraction, 1)
}

pub fn prune_triangle_components(
    mesh: &Mesh,
    min_faces: usize,
    min_fraction: f32,
    keep_largest_count: usize,
) -> Mesh {
    let tri_count = mesh.indices.len() / 3;
    if tri_count == 0 {
        return mesh.clone();
    }

    let mut edge_to_tris = HashMap::<(u32, u32), Vec<usize>>::new();
    for (tri_idx, tri) in mesh.indices.chunks_exact(3).enumerate() {
        for &(a, b) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            let edge = if a < b { (a, b) } else { (b, a) };
            edge_to_tris.entry(edge).or_default().push(tri_idx);
        }
    }

    let mut visited = vec![false; tri_count];
    let mut components = Vec::<Vec<usize>>::new();
    for start in 0..tri_count {
        if visited[start] {
            continue;
        }
        let mut component = Vec::new();
        let mut stack = vec![start];
        visited[start] = true;
        while let Some(tri_idx) = stack.pop() {
            component.push(tri_idx);
            let tri = &mesh.indices[tri_idx * 3..tri_idx * 3 + 3];
            for &(a, b) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
                let edge = if a < b { (a, b) } else { (b, a) };
                if let Some(neighbors) = edge_to_tris.get(&edge) {
                    for &next_tri in neighbors {
                        if !visited[next_tri] {
                            visited[next_tri] = true;
                            stack.push(next_tri);
                        }
                    }
                }
            }
        }
        components.push(component);
    }

    if components.len() <= keep_largest_count.max(1) {
        return mesh.clone();
    }

    let threshold = min_faces.max((tri_count as f32 * min_fraction).ceil() as usize);
    let mut sorted_sizes = components
        .iter()
        .map(|component| component.len())
        .collect::<Vec<_>>();
    sorted_sizes.sort_unstable_by(|a, b| b.cmp(a));
    let keep_size_floor = sorted_sizes
        .get(keep_largest_count.saturating_sub(1))
        .copied()
        .unwrap_or(usize::MAX);
    let mut keep_tri = vec![false; tri_count];
    for component in components {
        if component.len() >= threshold || component.len() >= keep_size_floor {
            for tri_idx in component {
                keep_tri[tri_idx] = true;
            }
        }
    }

    let mut remap = vec![u32::MAX; mesh.vertices.len()];
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for (tri_idx, tri) in mesh.indices.chunks_exact(3).enumerate() {
        if !keep_tri[tri_idx] {
            continue;
        }
        for &old_idx in tri {
            let remapped = &mut remap[old_idx as usize];
            if *remapped == u32::MAX {
                *remapped = vertices.len() as u32;
                vertices.push(mesh.vertices[old_idx as usize].clone());
            }
            indices.push(*remapped);
        }
    }

    Mesh { vertices, indices }
}

pub fn log_mesh_component_diagnostics(mesh: &Mesh, label: &str) {
    let tri_count = mesh.indices.len() / 3;
    if tri_count == 0 {
        return;
    }

    let mut edge_to_tris = HashMap::<(u32, u32), Vec<usize>>::new();
    for (tri_idx, tri) in mesh.indices.chunks_exact(3).enumerate() {
        for &(a, b) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            let edge = if a < b { (a, b) } else { (b, a) };
            edge_to_tris.entry(edge).or_default().push(tri_idx);
        }
    }

    let mut visited = vec![false; tri_count];
    let mut components = Vec::<Vec<usize>>::new();
    for start in 0..tri_count {
        if visited[start] {
            continue;
        }
        let mut component = Vec::new();
        let mut stack = vec![start];
        visited[start] = true;
        while let Some(tri_idx) = stack.pop() {
            component.push(tri_idx);
            let tri = &mesh.indices[tri_idx * 3..tri_idx * 3 + 3];
            for &(a, b) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
                let edge = if a < b { (a, b) } else { (b, a) };
                if let Some(neighbors) = edge_to_tris.get(&edge) {
                    for &next_tri in neighbors {
                        if !visited[next_tri] {
                            visited[next_tri] = true;
                            stack.push(next_tri);
                        }
                    }
                }
            }
        }
        components.push(component);
    }
    components.sort_by_key(|component| std::cmp::Reverse(component.len()));

    let Ok(mut log) = std::fs::OpenOptions::new()
        .append(true)
        .create(true)
        .open("output.md")
    else {
        return;
    };
    let _ = writeln!(log, "\n## Mesh Component Diagnostics: {}", label);
    let _ = writeln!(log, "- component_count: {}", components.len());
    for (component_idx, component) in components.iter().take(24).enumerate() {
        let mut vertex_seen = HashMap::<u32, bool>::new();
        let mut bounds_min = Vec3::splat(f32::INFINITY);
        let mut bounds_max = Vec3::splat(f32::NEG_INFINITY);
        for &tri_idx in component {
            let tri = &mesh.indices[tri_idx * 3..tri_idx * 3 + 3];
            for &vertex_idx in tri {
                if vertex_seen.insert(vertex_idx, true).is_none() {
                    let p = Vec3::from_array(mesh.vertices[vertex_idx as usize].position);
                    bounds_min = bounds_min.min(p);
                    bounds_max = bounds_max.max(p);
                }
            }
        }
        let _ = writeln!(
            log,
            "- component {}: triangles={} vertices={} bounds_min=({:.3},{:.3},{:.3}) bounds_max=({:.3},{:.3},{:.3})",
            component_idx,
            component.len(),
            vertex_seen.len(),
            bounds_min.x,
            bounds_min.y,
            bounds_min.z,
            bounds_max.x,
            bounds_max.y,
            bounds_max.z
        );
    }
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
                .extend_from_slice(&[center_idx, pair[1], pair[0]]);
        }
        mesh.indices.extend_from_slice(&[
            center_idx,
            loop_vertices[0],
            *loop_vertices.last().unwrap(),
        ]);
    }
}

pub fn fill_undirected_boundary_cycles(mesh: &mut Mesh) {
    let mut edge_counts: HashMap<(u32, u32), usize> = HashMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        for &(a, b) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            let key = if a < b { (a, b) } else { (b, a) };
            *edge_counts.entry(key).or_insert(0) += 1;
        }
    }

    let boundary_edges: Vec<(u32, u32)> = edge_counts
        .into_iter()
        .filter_map(|(edge, count)| (count == 1).then_some(edge))
        .collect();
    if boundary_edges.is_empty() {
        return;
    }

    let mut adjacency: HashMap<u32, Vec<u32>> = HashMap::new();
    for &(a, b) in &boundary_edges {
        adjacency.entry(a).or_default().push(b);
        adjacency.entry(b).or_default().push(a);
    }

    let mut visited = HashMap::<(u32, u32), bool>::new();
    for &(start_a, start_b) in &boundary_edges {
        let edge_key = if start_a < start_b {
            (start_a, start_b)
        } else {
            (start_b, start_a)
        };
        if visited.contains_key(&edge_key) {
            continue;
        }

        let mut loop_vertices = vec![start_a, start_b];
        visited.insert(edge_key, true);
        let mut prev = start_a;
        let mut current = start_b;
        let mut closed = false;
        for _ in 0..4096 {
            let Some(neighbors) = adjacency.get(&current) else {
                break;
            };
            let next = neighbors.iter().copied().find(|&candidate| {
                if candidate == prev && neighbors.len() > 1 {
                    return false;
                }
                let key = if current < candidate {
                    (current, candidate)
                } else {
                    (candidate, current)
                };
                !visited.contains_key(&key) || candidate == start_a
            });
            let Some(next) = next else {
                break;
            };
            if next == start_a {
                closed = true;
                break;
            }
            let key = if current < next {
                (current, next)
            } else {
                (next, current)
            };
            visited.insert(key, true);
            loop_vertices.push(next);
            prev = current;
            current = next;
        }

        if !closed || loop_vertices.len() < 3 {
            continue;
        }

        let mut centroid = Vec3::ZERO;
        let mut normal = Vec3::ZERO;
        for &idx in &loop_vertices {
            let vertex = &mesh.vertices[idx as usize];
            centroid += Vec3::from_array(vertex.position);
            normal += Vec3::from_array(vertex.normal);
        }
        centroid /= loop_vertices.len() as f32;
        normal = normal.normalize_or_zero();
        let center_idx = mesh.vertices.len() as u32;
        mesh.vertices.push(Vertex {
            position: centroid.to_array(),
            normal: normal.to_array(),
        });

        for i in 0..loop_vertices.len() {
            let a = loop_vertices[i];
            let b = loop_vertices[(i + 1) % loop_vertices.len()];
            let pa = Vec3::from_array(mesh.vertices[a as usize].position);
            let pb = Vec3::from_array(mesh.vertices[b as usize].position);
            let pc = centroid;
            let tri_normal = (pa - pc).cross(pb - pc);
            if tri_normal.dot(normal) >= 0.0 {
                mesh.indices.extend_from_slice(&[center_idx, a, b]);
            } else {
                mesh.indices.extend_from_slice(&[center_idx, b, a]);
            }
        }
    }
}

pub fn cap_boundary_edge_components(mesh: &mut Mesh) {
    let mut edge_counts: HashMap<(u32, u32), usize> = HashMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        for &(a, b) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            let key = if a < b { (a, b) } else { (b, a) };
            *edge_counts.entry(key).or_insert(0) += 1;
        }
    }
    let boundary_edges: Vec<(u32, u32)> = edge_counts
        .into_iter()
        .filter_map(|(edge, count)| (count == 1).then_some(edge))
        .collect();
    if boundary_edges.is_empty() {
        return;
    }

    let mut adjacency = HashMap::<u32, Vec<u32>>::new();
    for &(a, b) in &boundary_edges {
        adjacency.entry(a).or_default().push(b);
        adjacency.entry(b).or_default().push(a);
    }

    let mut visited = HashMap::<u32, bool>::new();
    for &(start, _) in &boundary_edges {
        if visited.contains_key(&start) {
            continue;
        }
        let mut component_vertices = Vec::new();
        let mut stack = vec![start];
        visited.insert(start, true);
        while let Some(v) = stack.pop() {
            component_vertices.push(v);
            if let Some(nexts) = adjacency.get(&v) {
                for &next in nexts {
                    if !visited.contains_key(&next) {
                        visited.insert(next, true);
                        stack.push(next);
                    }
                }
            }
        }
        if component_vertices.len() < 3 {
            continue;
        }
        let component_set = component_vertices
            .iter()
            .copied()
            .map(|idx| (idx, true))
            .collect::<HashMap<_, _>>();

        let centroid = component_vertices
            .iter()
            .map(|&idx| Vec3::from_array(mesh.vertices[idx as usize].position))
            .sum::<Vec3>()
            / component_vertices.len() as f32;
        let normal = component_vertices
            .iter()
            .map(|&idx| Vec3::from_array(mesh.vertices[idx as usize].normal))
            .sum::<Vec3>()
            .normalize_or_zero();
        let centroid_idx = mesh.vertices.len() as u32;
        mesh.vertices.push(Vertex {
            position: centroid.to_array(),
            normal: normal.to_array(),
        });

        for &(a, b) in &boundary_edges {
            let a_in = component_set.get(&a).copied().unwrap_or(false);
            let b_in = component_set.get(&b).copied().unwrap_or(false);
            if a_in && b_in {
                mesh.indices.extend_from_slice(&[a, b, centroid_idx]);
            }
        }
    }
}

pub fn boundary_edge_count(mesh: &Mesh) -> usize {
    analyze_mesh_topology(mesh).boundary_edges
}

pub fn non_manifold_edge_count(mesh: &Mesh) -> usize {
    analyze_mesh_topology(mesh).non_manifold_edges
}

pub fn analyze_mesh_topology(mesh: &Mesh) -> MeshTopologyStats {
    let mut edges: HashMap<(u32, u32), usize> = HashMap::new();
    let mut triangles: HashMap<[u32; 3], usize> = HashMap::new();
    let mut degenerate_triangles = 0usize;
    for tri in mesh.indices.chunks_exact(3) {
        if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
            degenerate_triangles += 1;
            continue;
        }
        let p0 = Vec3::from_array(mesh.vertices[tri[0] as usize].position);
        let p1 = Vec3::from_array(mesh.vertices[tri[1] as usize].position);
        let p2 = Vec3::from_array(mesh.vertices[tri[2] as usize].position);
        if 0.5 * (p1 - p0).cross(p2 - p0).length() <= 1e-8 {
            degenerate_triangles += 1;
            continue;
        }
        let mut sorted_tri = [tri[0], tri[1], tri[2]];
        sorted_tri.sort_unstable();
        *triangles.entry(sorted_tri).or_insert(0) += 1;
        for &(a, b) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            let key = if a < b { (a, b) } else { (b, a) };
            *edges.entry(key).or_insert(0) += 1;
        }
    }
    MeshTopologyStats {
        boundary_edges: edges.values().filter(|&&count| count == 1).count(),
        non_manifold_edges: edges.values().filter(|&&count| count > 2).count(),
        duplicate_triangles: triangles
            .values()
            .filter(|&&count| count > 1)
            .map(|count| count - 1)
            .sum(),
        degenerate_triangles,
    }
}

fn quantize_coord(value: f32, origin: f32, step: f32) -> i64 {
    ((value - origin) / step).round() as i64
}

fn quantize_point(point: Vec3, domain_min: Vec3, step: f32) -> [i64; 3] {
    [
        quantize_coord(point.x, domain_min.x, step),
        quantize_coord(point.y, domain_min.y, step),
        quantize_coord(point.z, domain_min.z, step),
    ]
}

fn quantized_leaf_key(leaf: &SurfaceLeafCell, domain_min: Vec3, step: f32) -> QuantizedLeafKey {
    QuantizedLeafKey {
        min: quantize_point(leaf.min, domain_min, step),
        max: quantize_point(leaf.max, domain_min, step),
    }
}

fn quantized_node_key(node: &OctreeNode, domain_min: Vec3, step: f32) -> QuantizedLeafKey {
    QuantizedLeafKey {
        min: quantize_point(node.min, domain_min, step),
        max: quantize_point(node.max, domain_min, step),
    }
}

fn quantized_edge_key(
    axis: u8,
    start: Vec3,
    end: Vec3,
    domain_min: Vec3,
    step: f32,
) -> QuantizedEdgeKey {
    let mut a = quantize_point(start, domain_min, step);
    let mut b = quantize_point(end, domain_min, step);
    if b < a {
        std::mem::swap(&mut a, &mut b);
    }
    fn mix(mut x: u64) -> u64 {
        x = x.wrapping_add(0x9e37_79b9_7f4a_7c15);
        x = (x ^ (x >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        x = (x ^ (x >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        x ^ (x >> 31)
    }
    let mut key = mix(axis as u64);
    for coord in a.into_iter().chain(b) {
        key ^= mix(coord as u64)
            .wrapping_add(key << 6)
            .wrapping_add(key >> 2);
    }
    key
}

#[derive(Clone, Copy, Debug)]
struct IntegerProbeSpace {
    domain_min: Vec3,
    scale: Vec3,
}

impl IntegerProbeSpace {
    fn from_octree(octree: &AdaptiveOctree) -> Self {
        let units = (1u64 << 10) as f32;
        let extent = (octree.root.max - octree.root.min).max(Vec3::splat(1e-5));
        Self {
            domain_min: octree.root.min,
            scale: Vec3::new(units / extent.x, units / extent.y, units / extent.z),
        }
    }

    fn to_int(self, p: Vec3) -> [i64; 3] {
        [
            ((p.x - self.domain_min.x) * self.scale.x).round() as i64,
            ((p.y - self.domain_min.y) * self.scale.y).round() as i64,
            ((p.z - self.domain_min.z) * self.scale.z).round() as i64,
        ]
    }

    fn to_vec3(self, p: [i64; 3]) -> Vec3 {
        Vec3::new(
            self.domain_min.x + p[0] as f32 / self.scale.x,
            self.domain_min.y + p[1] as f32 / self.scale.y,
            self.domain_min.z + p[2] as f32 / self.scale.z,
        )
    }

    fn to_i32_key(self, p: Vec3) -> [i32; 3] {
        let q = self.to_int(p);
        [
            q[0].clamp(i32::MIN as i64, i32::MAX as i64) as i32,
            q[1].clamp(i32::MIN as i64, i32::MAX as i64) as i32,
            q[2].clamp(i32::MIN as i64, i32::MAX as i64) as i32,
        ]
    }

    fn to_u16_key(self, p: Vec3) -> [u16; 3] {
        let q = self.to_int(p);
        [
            q[0].clamp(0, 1024) as u16,
            q[1].clamp(0, 1024) as u16,
            q[2].clamp(0, 1024) as u16,
        ]
    }
}

fn point_inside_node(point: Vec3, node: &OctreeNode, eps: f32) -> bool {
    point.x >= node.min.x - eps
        && point.x <= node.max.x + eps
        && point.y >= node.min.y - eps
        && point.y <= node.max.y + eps
        && point.z >= node.min.z - eps
        && point.z <= node.max.z + eps
}

fn find_leaf_at_point<'a>(node: &'a OctreeNode, point: Vec3, eps: f32) -> Option<&'a OctreeNode> {
    if !point_inside_node(point, node, eps) {
        return None;
    }
    let Some(children) = &node.children else {
        return Some(node);
    };
    children
        .iter()
        .find_map(|child| find_leaf_at_point(child, point, eps))
}

fn contains_dyadic_point(min: [u16; 3], max: [u16; 3], point: [u16; 3]) -> bool {
    point[0] >= min[0]
        && point[0] < max[0].max(min[0] + 1)
        && point[1] >= min[1]
        && point[1] < max[1].max(min[1] + 1)
        && point[2] >= min[2]
        && point[2] < max[2].max(min[2] + 1)
}

fn find_leaf_at_dyadic_point<'a>(
    node: &'a OctreeNode,
    point: [u16; 3],
    probe_space: IntegerProbeSpace,
) -> Option<&'a OctreeNode> {
    let min = probe_space.to_u16_key(node.min);
    let mut max = probe_space.to_u16_key(node.max);
    if max == min {
        max = [
            (min[0] + 1).min(1024),
            (min[1] + 1).min(1024),
            (min[2] + 1).min(1024),
        ];
    }
    if !contains_dyadic_point(min, max, point) && point != max {
        return None;
    }
    let Some(children) = &node.children else {
        return Some(node);
    };
    children
        .iter()
        .find_map(|child| find_leaf_at_dyadic_point(child, point, probe_space))
}

fn edge_owner_probe_points(axis: u8, mid: Vec3, probe: IntegerProbeSpace) -> [Vec3; 4] {
    let base = probe.to_int(mid);
    let offsets: [[i64; 3]; 4] = match axis {
        0 => [[0, -1, -1], [0, 1, -1], [0, 1, 1], [0, -1, 1]],
        1 => [[-1, 0, -1], [1, 0, -1], [1, 0, 1], [-1, 0, 1]],
        _ => [[-1, -1, 0], [1, -1, 0], [1, 1, 0], [-1, 1, 0]],
    };
    offsets.map(|offset| {
        probe.to_vec3([
            base[0] + offset[0],
            base[1] + offset[1],
            base[2] + offset[2],
        ])
    })
}

fn axis_value(p: Vec3, axis: u8) -> f32 {
    match axis {
        0 => p.x,
        1 => p.y,
        _ => p.z,
    }
}

fn set_axis_value(p: &mut Vec3, axis: u8, value: f32) {
    match axis {
        0 => p.x = value,
        1 => p.y = value,
        _ => p.z = value,
    }
}

fn local_axis_count(min: f32, max: f32, target: f32) -> usize {
    ((max - min) / target.max(0.05)).ceil().max(1.0) as usize
}

fn local_axis_coord(min: f32, max: f32, steps: usize, idx: usize) -> f32 {
    min + (max - min) * (idx as f32 / steps as f32)
}

fn sample_hermite_leaf_subgrid(
    sdf: &dyn Sdf,
    leaf: &SurfaceLeafCell,
    finest_cell_size: f32,
) -> Vec<HermiteSample> {
    let size = leaf.max - leaf.min;
    let nx = local_axis_count(leaf.min.x, leaf.max.x, finest_cell_size);
    let ny = local_axis_count(leaf.min.y, leaf.max.y, finest_cell_size);
    let nz = local_axis_count(leaf.min.z, leaf.max.z, finest_cell_size);
    let sample_budget = (nx + 1) * (ny + 1) * (nz + 1);
    if sample_budget > 65_536 {
        return sample_hermite_cell_sized(sdf, leaf.min, size);
    }

    let point_count = (nx + 1) * (ny + 1) * (nz + 1);
    let mut distances = Vec::with_capacity(point_count);
    for k in 0..=nz {
        let z = local_axis_coord(leaf.min.z, leaf.max.z, nz, k);
        for j in 0..=ny {
            let y = local_axis_coord(leaf.min.y, leaf.max.y, ny, j);
            for i in 0..=nx {
                let x = local_axis_coord(leaf.min.x, leaf.max.x, nx, i);
                distances.push(sdf.distance(Vec3::new(x, y, z)));
            }
        }
    }

    let idx = |i: usize, j: usize, k: usize| -> usize { i + (nx + 1) * (j + (ny + 1) * k) };
    let mut samples = Vec::new();
    let mut push_segment_sample = |axis: u8, start: Vec3, end: Vec3, d0: f32, d1: f32| {
        let sign_change = (d0 <= 0.0 && d1 >= 0.0) || (d0 >= 0.0 && d1 <= 0.0);
        if !sign_change || (d1 - d0).abs() < 1e-8 {
            return;
        }
        let t = (-d0 / (d1 - d0)).clamp(0.0, 1.0);
        let position = start.lerp(end, t);
        samples.push(HermiteSample {
            edge_index: axis as usize,
            position,
            normal: estimate_gradient_dynamic(sdf, position, size.min_element()),
        });
    };

    for k in 0..=nz {
        let z = local_axis_coord(leaf.min.z, leaf.max.z, nz, k);
        for j in 0..=ny {
            let y = local_axis_coord(leaf.min.y, leaf.max.y, ny, j);
            for i in 0..nx {
                let x0 = local_axis_coord(leaf.min.x, leaf.max.x, nx, i);
                let x1 = local_axis_coord(leaf.min.x, leaf.max.x, nx, i + 1);
                push_segment_sample(
                    0,
                    Vec3::new(x0, y, z),
                    Vec3::new(x1, y, z),
                    distances[idx(i, j, k)],
                    distances[idx(i + 1, j, k)],
                );
            }
        }
    }
    for k in 0..=nz {
        let z = local_axis_coord(leaf.min.z, leaf.max.z, nz, k);
        for j in 0..ny {
            let y0 = local_axis_coord(leaf.min.y, leaf.max.y, ny, j);
            let y1 = local_axis_coord(leaf.min.y, leaf.max.y, ny, j + 1);
            for i in 0..=nx {
                let x = local_axis_coord(leaf.min.x, leaf.max.x, nx, i);
                push_segment_sample(
                    1,
                    Vec3::new(x, y0, z),
                    Vec3::new(x, y1, z),
                    distances[idx(i, j, k)],
                    distances[idx(i, j + 1, k)],
                );
            }
        }
    }
    for k in 0..nz {
        let z0 = local_axis_coord(leaf.min.z, leaf.max.z, nz, k);
        let z1 = local_axis_coord(leaf.min.z, leaf.max.z, nz, k + 1);
        for j in 0..=ny {
            let y = local_axis_coord(leaf.min.y, leaf.max.y, ny, j);
            for i in 0..=nx {
                let x = local_axis_coord(leaf.min.x, leaf.max.x, nx, i);
                push_segment_sample(
                    2,
                    Vec3::new(x, y, z0),
                    Vec3::new(x, y, z1),
                    distances[idx(i, j, k)],
                    distances[idx(i, j, k + 1)],
                );
            }
        }
    }
    samples
}

fn collect_leaf_axis_coordinates(leaves: &[SurfaceLeafCell]) -> [Vec<f32>; 3] {
    let mut coords = [Vec::new(), Vec::new(), Vec::new()];
    for leaf in leaves {
        coords[0].push(leaf.min.x);
        coords[0].push(leaf.max.x);
        coords[1].push(leaf.min.y);
        coords[1].push(leaf.max.y);
        coords[2].push(leaf.min.z);
        coords[2].push(leaf.max.z);
    }
    for axis_coords in &mut coords {
        axis_coords.sort_by(|a, b| a.total_cmp(b));
        axis_coords.dedup_by(|a, b| (*a - *b).abs() <= 1e-4);
    }
    coords
}

fn coordinate_span_indices(coords: &[f32], min: f32, max: f32) -> (usize, usize) {
    let eps = 1e-4;
    let start = coords.partition_point(|coord| *coord < min - eps);
    let end = coords.partition_point(|coord| *coord <= max + eps);
    (start, end)
}

fn leaf_face_edge_segments(
    leaf: &SurfaceLeafCell,
    axis_coordinates: &[Vec<f32>; 3],
) -> Vec<(u8, Vec3, Vec3)> {
    let mut segments = Vec::new();
    for normal_axis in 0..3u8 {
        let face_coords = [
            axis_value(leaf.min, normal_axis),
            axis_value(leaf.max, normal_axis),
        ];
        let tangent_axes: [u8; 2] = match normal_axis {
            0 => [1, 2],
            1 => [0, 2],
            _ => [0, 1],
        };
        for face_coord in face_coords {
            for edge_axis in tangent_axes {
                let sweep_axis = tangent_axes[usize::from(tangent_axes[0] == edge_axis)];
                let edge_coords = &axis_coordinates[edge_axis as usize];
                let sweep_coords = &axis_coordinates[sweep_axis as usize];
                let (edge_start, edge_end) = coordinate_span_indices(
                    edge_coords,
                    axis_value(leaf.min, edge_axis),
                    axis_value(leaf.max, edge_axis),
                );
                let (sweep_start, sweep_end) = coordinate_span_indices(
                    sweep_coords,
                    axis_value(leaf.min, sweep_axis),
                    axis_value(leaf.max, sweep_axis),
                );
                if edge_end <= edge_start + 1 || sweep_end <= sweep_start {
                    continue;
                }
                for &sweep_coord in &sweep_coords[sweep_start..sweep_end] {
                    for edge_idx in edge_start..(edge_end - 1) {
                        let a_coord = edge_coords[edge_idx];
                        let b_coord = edge_coords[edge_idx + 1];
                        if b_coord <= a_coord + 1e-5 {
                            continue;
                        }
                        let mut start = leaf.min;
                        let mut end = leaf.min;
                        set_axis_value(&mut start, normal_axis, face_coord);
                        set_axis_value(&mut end, normal_axis, face_coord);
                        set_axis_value(&mut start, sweep_axis, sweep_coord);
                        set_axis_value(&mut end, sweep_axis, sweep_coord);
                        set_axis_value(&mut start, edge_axis, a_coord);
                        set_axis_value(&mut end, edge_axis, b_coord);
                        segments.push((edge_axis, start, end));
                    }
                }
            }
        }
    }
    segments
}

fn leaf_native_edge_segments(leaf: &SurfaceLeafCell) -> [(u8, Vec3, Vec3); 12] {
    let min = leaf.min;
    let max = leaf.max;
    [
        (
            0,
            Vec3::new(min.x, min.y, min.z),
            Vec3::new(max.x, min.y, min.z),
        ),
        (
            0,
            Vec3::new(min.x, max.y, min.z),
            Vec3::new(max.x, max.y, min.z),
        ),
        (
            0,
            Vec3::new(min.x, min.y, max.z),
            Vec3::new(max.x, min.y, max.z),
        ),
        (
            0,
            Vec3::new(min.x, max.y, max.z),
            Vec3::new(max.x, max.y, max.z),
        ),
        (
            1,
            Vec3::new(min.x, min.y, min.z),
            Vec3::new(min.x, max.y, min.z),
        ),
        (
            1,
            Vec3::new(max.x, min.y, min.z),
            Vec3::new(max.x, max.y, min.z),
        ),
        (
            1,
            Vec3::new(min.x, min.y, max.z),
            Vec3::new(min.x, max.y, max.z),
        ),
        (
            1,
            Vec3::new(max.x, min.y, max.z),
            Vec3::new(max.x, max.y, max.z),
        ),
        (
            2,
            Vec3::new(min.x, min.y, min.z),
            Vec3::new(min.x, min.y, max.z),
        ),
        (
            2,
            Vec3::new(max.x, min.y, min.z),
            Vec3::new(max.x, min.y, max.z),
        ),
        (
            2,
            Vec3::new(min.x, max.y, min.z),
            Vec3::new(min.x, max.y, max.z),
        ),
        (
            2,
            Vec3::new(max.x, max.y, min.z),
            Vec3::new(max.x, max.y, max.z),
        ),
    ]
}

fn build_edge_leaf_registry(
    leaves: &[SurfaceLeafCell],
    probe_space: IntegerProbeSpace,
) -> HashMap<EdgeKey, EdgeRegistryEntry> {
    let axis_coordinates = collect_leaf_axis_coordinates(leaves);
    let mut registry = HashMap::<EdgeKey, EdgeRegistryEntry>::new();

    for (leaf_idx, leaf) in leaves.iter().enumerate() {
        for (axis, start, end) in leaf_face_edge_segments(leaf, &axis_coordinates) {
            let mut key_start = probe_space.to_u16_key(start);
            let mut key_end = probe_space.to_u16_key(end);
            if key_end < key_start {
                std::mem::swap(&mut key_start, &mut key_end);
            }
            if key_start == key_end {
                continue;
            }
            let key = EdgeKey {
                axis,
                start: key_start,
                end: key_end,
            };
            let entry = registry.entry(key).or_insert_with(|| EdgeRegistryEntry {
                axis,
                start,
                end,
                owners: Vec::with_capacity(4),
            });
            if !entry.owners.contains(&leaf_idx) {
                entry.owners.push(leaf_idx);
            }
        }
    }

    registry
}

fn build_dyadic_leaves(
    leaves: &[SurfaceLeafCell],
    surface_lookup: &HashMap<EdgeKey, usize>,
    probe_space: IntegerProbeSpace,
) -> Vec<DyadicLeaf> {
    leaves
        .iter()
        .enumerate()
        .map(|(idx, leaf)| {
            let min = probe_space.to_u16_key(leaf.min);
            let max = probe_space.to_u16_key(leaf.max);
            let key = EdgeKey {
                axis: 255,
                start: min,
                end: max,
            };
            DyadicLeaf {
                source_idx: idx,
                min,
                max,
                world_min: leaf.min,
                world_max: leaf.max,
                is_surface_leaf: surface_lookup.contains_key(&key),
            }
        })
        .collect()
}

fn build_surface_leaf_lookup(
    leaves: &[SurfaceLeafCell],
    probe_space: IntegerProbeSpace,
) -> HashMap<EdgeKey, usize> {
    leaves
        .iter()
        .enumerate()
        .map(|(idx, leaf)| {
            (
                EdgeKey {
                    axis: 255,
                    start: probe_space.to_u16_key(leaf.min),
                    end: probe_space.to_u16_key(leaf.max),
                },
                idx,
            )
        })
        .collect()
}

fn edge_midpoint_key(key: EdgeKey) -> [u16; 3] {
    [
        ((key.start[0] as u32 + key.end[0] as u32) / 2) as u16,
        ((key.start[1] as u32 + key.end[1] as u32) / 2) as u16,
        ((key.start[2] as u32 + key.end[2] as u32) / 2) as u16,
    ]
}

fn transverse_axes(axis: u8) -> [usize; 2] {
    match axis {
        0 => [1, 2],
        1 => [0, 2],
        _ => [0, 1],
    }
}

fn quadrant_owner_leaves(
    key: EdgeKey,
    octree: &AdaptiveOctree,
    probe_space: IntegerProbeSpace,
    surface_lookup: &HashMap<EdgeKey, usize>,
) -> ([Option<usize>; 4], [Option<usize>; 4]) {
    let mid = edge_midpoint_key(key);
    let transverse = transverse_axes(key.axis);
    let offsets = [(-1i32, -1i32), (1, -1), (1, 1), (-1, 1)];
    let mut all_owners = [None; 4];
    let mut surface_owners = [None; 4];
    for (slot, (oa, ob)) in offsets.into_iter().enumerate() {
        let mut p = mid;
        p[transverse[0]] = (p[transverse[0]] as i32 + oa).clamp(0, 1024) as u16;
        p[transverse[1]] = (p[transverse[1]] as i32 + ob).clamp(0, 1024) as u16;
        if let Some(leaf) = find_leaf_at_dyadic_point(&octree.root, p, probe_space) {
            let leaf_key = EdgeKey {
                axis: 255,
                start: probe_space.to_u16_key(leaf.min),
                end: probe_space.to_u16_key(leaf.max),
            };
            let owner = surface_lookup.get(&leaf_key).copied();
            all_owners[slot] = owner;
            surface_owners[slot] = owner;
        }
    }
    (all_owners, surface_owners)
}

fn unique_owner_count(owners: &[Option<usize>; 4]) -> usize {
    let mut unique = Vec::with_capacity(4);
    for owner in owners.iter().flatten().copied() {
        if !unique.contains(&owner) {
            unique.push(owner);
        }
    }
    unique.len()
}

fn key_touches_root_boundary(key: EdgeKey) -> bool {
    key.start.contains(&0)
        || key.end.contains(&0)
        || key.start.contains(&1024)
        || key.end.contains(&1024)
}

fn classify_edge_interval(
    key: EdgeKey,
    topology_owners: &[Option<usize>; 4],
    dual_vertex_owners: &[Option<usize>; 4],
    sign_delta: f32,
) -> EdgeIntervalClass {
    if sign_delta.abs() < 1e-8 || key.start == key.end {
        return EdgeIntervalClass::DegenerateInterval;
    }
    if topology_owners.iter().any(Option::is_none) {
        return if key_touches_root_boundary(key) {
            EdgeIntervalClass::RootBoundary
        } else {
            EdgeIntervalClass::InteriorMissingOwner
        };
    }
    if dual_vertex_owners.iter().any(Option::is_none) {
        return EdgeIntervalClass::NoDualVertex;
    }
    match unique_owner_count(topology_owners) {
        4 => EdgeIntervalClass::InteriorValid4,
        3 => EdgeIntervalClass::InteriorValid3Transition,
        1 | 2 => EdgeIntervalClass::InteriorDuplicateOwner,
        _ => EdgeIntervalClass::Unknown,
    }
}

fn record_interval_class(audit: &mut EmitterAudit, axis: u8, class: EdgeIntervalClass) {
    if let Some(axis_counts) = audit.class_counts_by_axis.get_mut(axis as usize) {
        axis_counts[class.index()] += 1;
    }
}

fn log_emitter_mismatch(
    key: EdgeKey,
    entry: &EdgeRegistryEntry,
    d0: f32,
    d1: f32,
    audit: &mut EmitterAudit,
) {
    audit.emitter_mismatches += 1;
    if audit.logged_mismatches >= 64 {
        return;
    }
    audit.logged_mismatches += 1;
    let Ok(mut log) = std::fs::OpenOptions::new()
        .append(true)
        .create(true)
        .open("output.md")
    else {
        return;
    };
    let _ = writeln!(log, "### Emitter Mismatch (Structural)");
    let _ = writeln!(
        log,
        "- Key: axis={} start={:?} end={:?}",
        key.axis, key.start, key.end
    );
    let _ = writeln!(
        log,
        "- segment: start=({:.6},{:.6},{:.6}) end=({:.6},{:.6},{:.6})",
        entry.start.x, entry.start.y, entry.start.z, entry.end.x, entry.end.y, entry.end.z
    );
    let _ = writeln!(log, "- sdf: d0={:.8} d1={:.8}", d0, d1);
    let _ = writeln!(log, "- Found Owners: {:?}", entry.owners);
    let _ = writeln!(log, "- Expected: 4");
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

    let point_count = (nx + 1) * (ny + 1) * (nz + 1);
    let point_distances: Vec<f32> = (0..point_count)
        .into_par_iter()
        .map(|idx| {
            let plane = (nx + 1) * (ny + 1);
            let k = idx / plane;
            let rem = idx - k * plane;
            let j = rem / (nx + 1);
            let i = rem - j * (nx + 1);
            let p = bounds_min + Vec3::new(i as f32 * step.x, j as f32 * step.y, k as f32 * step.z);
            sdf.distance(p)
        })
        .collect();

    let cell_count = nx * ny * nz;
    let eps = (step.min_element() * 0.2).max(0.01);

    let cell_results: Vec<Option<(usize, UniformDualCell, Vertex)>> = (0..cell_count)
        .into_par_iter()
        .map(|cell_idx| {
            let k = cell_idx / (nx * ny);
            let rem = cell_idx - k * nx * ny;
            let j = rem / nx;
            let i = rem - j * nx;
            let cell_min =
                bounds_min + Vec3::new(i as f32 * step.x, j as f32 * step.y, k as f32 * step.z);
            let corner_distances = cell_corner_distances(&point_distances, i, j, k, nx, ny);
            let all_pos = corner_distances.iter().all(|&d| d >= 0.0);
            let all_neg = corner_distances.iter().all(|&d| d <= 0.0);
            if all_pos || all_neg {
                return None;
            }
            let samples = sample_hermite_cell_sized(sdf, cell_min, step);
            let dual = solve_qef_vertex(&samples, cell_min, cell_min + step)?;
            let normal = estimate_gradient(sdf, dual.position, eps);
            Some((
                cell_idx,
                UniformDualCell {
                    vertex: dual,
                    corner_distances,
                },
                Vertex {
                    position: dual.position.to_array(),
                    normal: normal.to_array(),
                },
            ))
        })
        .collect();

    let mut cells: Vec<Option<UniformDualCell>> = vec![None; cell_count];
    let mut cell_to_vertex: Vec<Option<u32>> = vec![None; cell_count];
    let mut vertices = Vec::new();
    for (cell_idx, cell, vertex) in cell_results.into_iter().flatten() {
        cell_to_vertex[cell_idx] = Some(vertices.len() as u32);
        vertices.push(vertex);
        cells[cell_idx] = Some(cell);
    }

    let mut indices = Vec::new();

    let push_quad = |indices: &mut Vec<u32>, a: u32, b: u32, c: u32, d: u32, flip: bool| {
        if flip {
            indices.extend_from_slice(&[a, c, b, a, d, c]);
        } else {
            indices.extend_from_slice(&[a, b, c, a, c, d]);
        }
    };

    let cell_index = |i: usize, j: usize, k: usize| -> usize { i + nx * (j + ny * k) };
    let cell_vertex =
        |i: usize, j: usize, k: usize| -> Option<u32> { cell_to_vertex[cell_index(i, j, k)] };

    // X-oriented grid edges. Each active edge emits a quad connecting the four
    // surface cells that share that edge.
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
                    if let [Some(a), Some(b), Some(c), Some(d)] =
                        cells4.map(|(ci, cj, ck)| cell_vertex(ci, cj, ck))
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
                    if let [Some(a), Some(b), Some(c), Some(d)] =
                        cells4.map(|(ci, cj, ck)| cell_vertex(ci, cj, ck))
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
                    if let [Some(a), Some(b), Some(c), Some(d)] =
                        cells4.map(|(ci, cj, ck)| cell_vertex(ci, cj, ck))
                    {
                        push_quad(&mut indices, a, b, c, d, d0 < 0.0);
                    }
                }
            }
        }
    }

    let _ = cells
        .iter()
        .flatten()
        .map(|c| c.vertex.sample_count + c.corner_distances.len())
        .sum::<usize>();

    let mut mesh = Mesh { vertices, indices };
    remove_sliver_triangles(
        &mut mesh,
        25.0,
        step.max_element() * step.max_element() * 1e-4,
    );
    mesh
}

pub fn extract_dual_contour_mesh_from_octree(
    sdf: &dyn Sdf,
    octree: &AdaptiveOctree,
    finest_cell_size: f32,
) -> Mesh {
    extract_dual_contour_mesh_from_octree_with_telemetry(sdf, octree, finest_cell_size).0
}

pub fn extract_dual_contour_mesh_from_octree_with_telemetry(
    sdf: &dyn Sdf,
    octree: &AdaptiveOctree,
    finest_cell_size: f32,
) -> (Mesh, DualContouringTelemetry) {
    let total_start = Instant::now();
    let mut telemetry = DualContouringTelemetry::default();
    let octree_walk_start = Instant::now();
    let surface_leaves = collect_surface_leaves(octree);
    if surface_leaves.is_empty() {
        telemetry.duration_octree_walk_ms = octree_walk_start.elapsed().as_millis();
        return (
            Mesh {
                vertices: Vec::new(),
                indices: Vec::new(),
            },
            telemetry,
        );
    }
    let leaves = collect_leaf_cells(octree);
    telemetry.total_leaf_nodes = surface_leaves.len();
    for leaf in &surface_leaves {
        *telemetry
            .leaf_counts_per_depth
            .entry(leaf.depth)
            .or_insert(0) += 1;
    }

    let probe_space = IntegerProbeSpace::from_octree(octree);
    let surface_lookup = build_surface_leaf_lookup(&leaves, probe_space);
    let dyadic_leaves = build_dyadic_leaves(&leaves, &surface_lookup, probe_space);
    let edge_registry = build_edge_leaf_registry(&surface_leaves, probe_space);
    telemetry.duration_octree_walk_ms = octree_walk_start.elapsed().as_millis();

    let mut leaf_samples: Vec<Vec<HermiteSample>> = vec![Vec::new(); leaves.len()];
    let mut active_edges = Vec::<(EdgeKey, AdaptiveActiveEdge)>::new();
    let mut audit = EmitterAudit {
        total_edge_keys: edge_registry.len(),
        all_leaf_count: dyadic_leaves.len(),
        surface_leaf_count: surface_leaves.len(),
        ..EmitterAudit::default()
    };

    let edge_emission_start = Instant::now();
    let mut registry_entries: Vec<(EdgeKey, EdgeRegistryEntry)> =
        edge_registry.into_iter().collect();
    registry_entries.sort_by_key(|(key, _)| *key);
    for (key, entry) in registry_entries {
        let d0 = sdf.distance(entry.start);
        let d1 = sdf.distance(entry.end);
        let sign_change = (d0 <= 0.0 && d1 >= 0.0) || (d0 >= 0.0 && d1 <= 0.0);
        if !sign_change || (d1 - d0).abs() < 1e-8 {
            continue;
        }
        audit.crossing_edges += 1;
        let (topology_owners, surface_owners) =
            quadrant_owner_leaves(key, octree, probe_space, &surface_lookup);
        let interval_class =
            classify_edge_interval(key, &topology_owners, &surface_owners, d1 - d0);
        record_interval_class(&mut audit, key.axis, interval_class);
        if !matches!(
            interval_class,
            EdgeIntervalClass::InteriorValid4 | EdgeIntervalClass::InteriorValid3Transition
        ) {
            if interval_class == EdgeIntervalClass::NoDualVertex {
                audit.missing_surface_vertex_edges += 1;
            }
            log_emitter_mismatch(key, &entry, d0, d1, &mut audit);
            continue;
        }
        if interval_class == EdgeIntervalClass::InteriorValid4 {
            audit.four_owner_crossing_edges += 1;
            audit.four_surface_vertex_edges += 1;
        } else {
            audit.three_owner_transition_edges += 1;
        }

        let initial_t = (-d0 / (d1 - d0)).clamp(0.0, 1.0);
        if initial_t == 0.5 {
            telemetry.exact_midpoint_fallbacks += 1;
        }
        let (t, _) = refine_edge_root(sdf, entry.start, entry.end, d0, d1);
        let position = entry.start.lerp(entry.end, t);
        let local_cell_size = entry
            .owners
            .iter()
            .filter_map(|&owner| leaves.get(owner))
            .map(|leaf| leaf.max.distance(leaf.min) / 3.0_f32.sqrt())
            .fold(finest_cell_size.max(0.05), f32::min);
        let normal =
            estimate_gradient_with_telemetry(sdf, position, local_cell_size, &mut telemetry);
        let sample = HermiteSample {
            edge_index: entry.axis as usize,
            position,
            normal,
        };
        let surface_owners = surface_owners.map(|owner| owner.expect("surface owner checked"));
        for owner in surface_owners.iter().copied() {
            leaf_samples[owner].push(sample);
        }
        active_edges.push((
            key,
            AdaptiveActiveEdge {
                d0,
                d1,
                start: entry.start,
                end: entry.end,
                normal: sample.normal,
                owners: [
                    Some(surface_owners[0]),
                    Some(surface_owners[1]),
                    Some(surface_owners[2]),
                    Some(surface_owners[3]),
                ],
            },
        ));
    }
    active_edges.sort_by_key(|(key, _)| *key);
    telemetry.total_active_edges = active_edges.len();
    if std::env::var_os("IMPLICIT_CAD_DC_TRACE").is_some() {
        eprintln!(
            "adaptive_dc_trace: leaves={} registry_edge_keys={} crossing_edges={} emitter_mismatches={} active_edges={}",
            leaves.len(),
            audit.total_edge_keys,
            audit.crossing_edges,
            audit.emitter_mismatches,
            active_edges.len()
        );
    }

    let mut leaf_vertices = Vec::new();
    let mut leaf_vertex_map = Vec::with_capacity(leaves.len());
    let qef_solve_start = Instant::now();
    for (leaf, samples) in leaves.iter().zip(leaf_samples.iter()) {
        if let Some(dual) =
            solve_qef_vertex_with_telemetry(samples, leaf.min, leaf.max, Some(&mut telemetry))
        {
            let mut normal = Vec3::ZERO;
            for sample in samples {
                normal += sample.normal;
            }
            normal = normal.normalize_or_zero();
            leaf_vertex_map.push(Some(leaf_vertices.len() as u32));
            leaf_vertices.push(Vertex {
                position: dual.position.to_array(),
                normal: normal.to_array(),
            });
        } else {
            leaf_vertex_map.push(None);
        }
    }
    telemetry.duration_qef_solve_ms = qef_solve_start.elapsed().as_millis();

    let mut indices = Vec::new();
    for (_, edge) in active_edges {
        let quad = edge
            .owners
            .map(|owner| owner.and_then(|leaf_idx| leaf_vertex_map[leaf_idx]));
        if let [Some(a), Some(b), Some(c), Some(d)] = quad {
            let edge_len = edge.start.distance(edge.end);
            let min_area = (edge_len * finest_cell_size.max(0.05) * 1e-5).max(1e-8);
            push_sign_oriented_quad(
                &mut indices,
                &leaf_vertices,
                [a, b, c, d],
                edge.d0,
                edge.d1,
                edge.normal,
                min_area,
            );
        }
    }
    telemetry.duration_edge_emission_ms = edge_emission_start
        .elapsed()
        .as_millis()
        .saturating_sub(telemetry.duration_qef_solve_ms);

    let mut mesh = Mesh {
        vertices: leaf_vertices,
        indices,
    };
    remove_sliver_triangles(
        &mut mesh,
        100.0,
        finest_cell_size.max(0.05) * finest_cell_size.max(0.05) * 1e-6,
    );
    remove_duplicate_triangles(&mut mesh);
    remove_degenerate_triangles(
        &mut mesh,
        finest_cell_size.max(0.05) * finest_cell_size.max(0.05) * 1e-8,
    );
    let structural_topology = analyze_mesh_topology(&mesh);
    if std::env::var_os("IMPLICIT_CAD_DC_TRACE").is_some() {
        eprintln!(
            "adaptive_dc_stage: stage=primary boundary={} non_manifold={} duplicate={} degenerate={} vertices={} triangles={}",
            structural_topology.boundary_edges,
            structural_topology.non_manifold_edges,
            structural_topology.duplicate_triangles,
            structural_topology.degenerate_triangles,
            mesh.vertices.len(),
            mesh.indices.len() / 3
        );
    }
    if std::env::var_os("IMPLICIT_CAD_DC_PRIMARY_ONLY").is_some() {
        let selected = weld_vertices_quantized(&mesh, 1_000_000.0);
        log_final_face_index_integrity(&selected);
        log_adaptive_emitter_audit(
            &audit,
            &structural_topology,
            &analyze_mesh_topology(&selected),
        );
        return (selected, telemetry);
    }
    if structural_topology.non_manifold_edges > 0 {
        let filtered = filter_directed_half_edges(&mesh);
        let filtered_topology = analyze_mesh_topology(&filtered);
        if std::env::var_os("IMPLICIT_CAD_DC_TRACE").is_some() {
            eprintln!(
                "adaptive_dc_stage: stage=primary_filtered boundary={} non_manifold={} duplicate={} degenerate={} vertices={} triangles={}",
                filtered_topology.boundary_edges,
                filtered_topology.non_manifold_edges,
                filtered_topology.duplicate_triangles,
                filtered_topology.degenerate_triangles,
                filtered.vertices.len(),
                filtered.indices.len() / 3
            );
        }
        if filtered_topology.non_manifold_edges < structural_topology.non_manifold_edges {
            mesh = filtered;
        }
    }
    if analyze_mesh_topology(&mesh).boundary_edges > 0 {
        fill_boundary_loops(&mut mesh);
        if std::env::var_os("IMPLICIT_CAD_DC_TRACE").is_some() {
            let topology = analyze_mesh_topology(&mesh);
            eprintln!(
                "adaptive_dc_stage: stage=directed_fill boundary={} non_manifold={} duplicate={} degenerate={} vertices={} triangles={}",
                topology.boundary_edges,
                topology.non_manifold_edges,
                topology.duplicate_triangles,
                topology.degenerate_triangles,
                mesh.vertices.len(),
                mesh.indices.len() / 3
            );
        }
        fill_undirected_boundary_cycles(&mut mesh);
        if std::env::var_os("IMPLICIT_CAD_DC_TRACE").is_some() {
            let topology = analyze_mesh_topology(&mesh);
            eprintln!(
                "adaptive_dc_stage: stage=undirected_fill boundary={} non_manifold={} duplicate={} degenerate={} vertices={} triangles={}",
                topology.boundary_edges,
                topology.non_manifold_edges,
                topology.duplicate_triangles,
                topology.degenerate_triangles,
                mesh.vertices.len(),
                mesh.indices.len() / 3
            );
        }
    }
    remove_duplicate_triangles(&mut mesh);
    remove_degenerate_triangles(
        &mut mesh,
        finest_cell_size.max(0.05) * finest_cell_size.max(0.05) * 1e-8,
    );
    mesh = repair_non_manifold_edges_with_refill(
        &mesh,
        finest_cell_size.max(0.05) * finest_cell_size.max(0.05) * 1e-8,
    );
    let raw_topology = analyze_mesh_topology(&mesh);
    let mut final_candidate = filter_directed_half_edges(&mesh);
    if analyze_mesh_topology(&final_candidate).boundary_edges <= 20_000 {
        fill_boundary_loops(&mut final_candidate);
        fill_undirected_boundary_cycles(&mut final_candidate);
        remove_duplicate_triangles(&mut final_candidate);
        remove_degenerate_triangles(
            &mut final_candidate,
            finest_cell_size.max(0.05) * finest_cell_size.max(0.05) * 1e-8,
        );
    }
    let filtered = final_candidate;
    let filtered_topology = analyze_mesh_topology(&filtered);
    if std::env::var_os("IMPLICIT_CAD_DC_TRACE").is_some() {
        eprintln!(
            "adaptive_dc_topology_choice: raw_boundary={} raw_non_manifold={} raw_duplicate={} raw_degenerate={} filtered_boundary={} filtered_non_manifold={} filtered_duplicate={} filtered_degenerate={}",
            raw_topology.boundary_edges,
            raw_topology.non_manifold_edges,
            raw_topology.duplicate_triangles,
            raw_topology.degenerate_triangles,
            filtered_topology.boundary_edges,
            filtered_topology.non_manifold_edges,
            filtered_topology.duplicate_triangles,
            filtered_topology.degenerate_triangles
        );
    }
    let mut selected = if filtered_topology.boundary_edges <= raw_topology.boundary_edges
        && filtered_topology.non_manifold_edges <= raw_topology.non_manifold_edges
        && filtered_topology.duplicate_triangles <= raw_topology.duplicate_triangles
        && filtered_topology.degenerate_triangles <= raw_topology.degenerate_triangles
    {
        filtered
    } else {
        telemetry.duration_edge_emission_ms = telemetry.duration_edge_emission_ms.max(
            total_start
                .elapsed()
                .as_millis()
                .saturating_sub(telemetry.duration_qef_solve_ms),
        );
        mesh
    };
    let welded = weld_vertices_quantized(&selected, 1_000_000.0);
    let selected_topology = analyze_mesh_topology(&selected);
    let welded_topology = analyze_mesh_topology(&welded);
    if welded_topology.boundary_edges <= selected_topology.boundary_edges
        && welded_topology.non_manifold_edges <= selected_topology.non_manifold_edges
        && welded_topology.duplicate_triangles <= selected_topology.duplicate_triangles
        && welded_topology.degenerate_triangles <= selected_topology.degenerate_triangles
    {
        selected = welded;
    }
    log_final_face_index_integrity(&selected);
    log_adaptive_emitter_audit(
        &audit,
        &structural_topology,
        &analyze_mesh_topology(&selected),
    );
    (selected, telemetry)
}

fn log_adaptive_emitter_audit(
    audit: &EmitterAudit,
    structural_topology: &MeshTopologyStats,
    final_topology: &MeshTopologyStats,
) {
    let Ok(mut log) = std::fs::OpenOptions::new()
        .append(true)
        .create(true)
        .open("output.md")
    else {
        return;
    };
    let _ = writeln!(log, "\n## Adaptive Emitter Registry Audit");
    let _ = writeln!(log, "- all_leaf_count: {}", audit.all_leaf_count);
    let _ = writeln!(log, "- surface_leaf_count: {}", audit.surface_leaf_count);
    let _ = writeln!(log, "- total_edge_keys: {}", audit.total_edge_keys);
    let _ = writeln!(log, "- crossing_edges: {}", audit.crossing_edges);
    let _ = writeln!(
        log,
        "- four_owner_crossing_edges: {}",
        audit.four_owner_crossing_edges
    );
    let closure_pct = if audit.crossing_edges == 0 {
        100.0
    } else {
        audit.four_owner_crossing_edges as f32 * 100.0 / audit.crossing_edges as f32
    };
    let _ = writeln!(
        log,
        "- four_owner_crossing_edge_percent: {:.3}",
        closure_pct
    );
    let _ = writeln!(
        log,
        "- four_surface_vertex_edges: {}",
        audit.four_surface_vertex_edges
    );
    let surface_pct = if audit.crossing_edges == 0 {
        100.0
    } else {
        audit.four_surface_vertex_edges as f32 * 100.0 / audit.crossing_edges as f32
    };
    let _ = writeln!(
        log,
        "- four_surface_vertex_edge_percent: {:.3}",
        surface_pct
    );
    let _ = writeln!(
        log,
        "- three_owner_transition_edges: {}",
        audit.three_owner_transition_edges
    );
    let _ = writeln!(log, "- emitter_mismatches: {}", audit.emitter_mismatches);
    let _ = writeln!(
        log,
        "- missing_surface_vertex_edges: {}",
        audit.missing_surface_vertex_edges
    );
    let _ = writeln!(log, "- logged_mismatches: {}", audit.logged_mismatches);
    let _ = writeln!(
        log,
        "- primary_boundary_edges_before_patch: {}",
        structural_topology.boundary_edges
    );
    let _ = writeln!(
        log,
        "- primary_non_manifold_edges_before_patch: {}",
        structural_topology.non_manifold_edges
    );
    let _ = writeln!(
        log,
        "- final_boundary_edges: {}",
        final_topology.boundary_edges
    );
    let _ = writeln!(
        log,
        "- final_non_manifold_edges: {}",
        final_topology.non_manifold_edges
    );
    let _ = writeln!(log, "\n## Adaptive Interval Classification");
    for axis in 0..3 {
        let _ = writeln!(log, "### Axis {}", axis);
        for class in EdgeIntervalClass::all() {
            let _ = writeln!(
                log,
                "- {}: {}",
                class.label(),
                audit.class_counts_by_axis[axis][class.index()]
            );
        }
    }
}

fn log_final_face_index_integrity(mesh: &Mesh) {
    let Ok(mut log) = std::fs::OpenOptions::new()
        .append(true)
        .create(true)
        .open("output.md")
    else {
        return;
    };
    let mut index_use_count = vec![0usize; mesh.vertices.len()];
    for &idx in &mesh.indices {
        index_use_count[idx as usize] += 1;
    }
    let _ = writeln!(log, "\n## Final Mesh Face Index Audit");
    for (face_idx, face) in mesh.indices.chunks_exact(3).take(3).enumerate() {
        let face_indices = [face[0], face[1], face[2]];
        let _ = writeln!(log, "### Final Face {}", face_idx);
        let _ = writeln!(log, "Indices: {:?}", face_indices);
        for idx in face_indices {
            let v = mesh.vertices[idx as usize];
            let _ = writeln!(
                log,
                "Vertex index {}: {:?} (Shared? {})",
                idx,
                v,
                index_use_count[idx as usize] > 1
            );
        }
    }
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
        let topology = analyze_mesh_topology(&mesh);
        assert_eq!(topology.boundary_edges, 0);
        assert_eq!(topology.non_manifold_edges, 0);
        assert_eq!(topology.degenerate_triangles, 0);
        assert_eq!(topology.duplicate_triangles, 0);
        assert_eq!(non_manifold_edge_count(&mesh), 0);
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
        let topology = analyze_mesh_topology(&mesh);
        assert_eq!(topology.non_manifold_edges, 0);
        assert_eq!(topology.degenerate_triangles, 0);
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
    fn extract_dual_contour_mesh_from_octree_produces_triangle_mesh() {
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
        assert_eq!(mesh.indices.len() % 3, 0);
    }
}
