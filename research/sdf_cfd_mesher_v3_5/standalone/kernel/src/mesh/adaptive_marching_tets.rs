//! Conforming adaptive-octree marching tetrahedra.
//!
//! This module is intentionally independent from the adaptive marching-cubes
//! transition repair path.  It converts public `AdaptiveOctree` leaves into a
//! shared finest-lattice face registry, builds a star tetrahedralization of
//! each leaf cell, and extracts the zero set with marching tetrahedra.

use crate::mesh::adaptive_sampling::{AdaptiveOctree, AdaptiveOctreeNode};
use crate::mesh::{Mesh, Vertex};
use crate::sdf::Sdf;
use glam::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::time::Instant;

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct GridPointKey {
    pub xyz: [i64; 3],
    pub denominator_level: u8,
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd, Serialize, Deserialize)]
pub enum Axis {
    X,
    Y,
    Z,
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct CanonicalFaceKey {
    pub axis: Axis,
    pub plane: i64,
    pub origin_uv: [i64; 2],
    pub extent_uv: [i64; 2],
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct TetEdgeKey {
    pub a: GridPointKey,
    pub b: GridPointKey,
}

impl TetEdgeKey {
    fn new(a: GridPointKey, b: GridPointKey) -> Self {
        if a <= b {
            Self { a, b }
        } else {
            Self { a: b, b: a }
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FaceRegistryEntry {
    pub key: CanonicalFaceKey,
    pub incident_cells: Vec<u64>,
    pub triangles: Vec<[GridPointKey; 3]>,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct AdaptiveTetExtractionStats {
    pub leaf_count: usize,
    pub max_level: u8,
    pub edge_registry_points: usize,
    pub face_registry_entries: usize,
    pub face_triangles: usize,
    pub boundary_invalid_cells: usize,
    pub cell_boundary_triangles: usize,
    pub tetrahedra: usize,
    pub positive_tetrahedra: usize,
    pub non_positive_tetrahedra: usize,
    pub sdf_cache_hits: usize,
    pub sdf_cache_misses: usize,
    pub crossing_vertices: usize,
    pub surface_triangles: usize,
    pub boundary_edges: usize,
    pub non_manifold_edges: usize,
    pub duplicate_triangles: usize,
    pub degenerate_triangles: usize,
    pub runtime_ms: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AdaptiveMarchingTetsReport {
    pub mesh: MeshSummary,
    pub stats: AdaptiveTetExtractionStats,
    pub status: String,
    pub failure_reasons: Vec<String>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MeshSummary {
    pub vertices: usize,
    pub triangles: usize,
    pub mesh_hash: String,
}

#[derive(Clone, Debug)]
pub struct AdaptiveMarchingTetsSurface {
    pub mesh: Mesh,
    pub stats: AdaptiveTetExtractionStats,
    pub vertex_sources: Vec<AdaptiveTetVertexSource>,
    pub triangle_sources: Vec<AdaptiveTetTriangleSource>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AdaptiveTetVertexSource {
    pub vertex_id: u32,
    pub tet_edge_key: TetEdgeKey,
    pub endpoint_keys: [GridPointKey; 2],
    pub endpoint_signed_distances: [f32; 2],
    pub interpolation_parameter: f32,
    pub incident_tetrahedron_ids: Vec<u64>,
    pub incident_leaf_cell_ids: Vec<u64>,
    pub incident_leaf_levels: Vec<u8>,
    pub min_local_cell_dimension: f32,
    pub max_local_cell_dimension: f32,
    pub local_cell_diagonal: f32,
    pub geometrically_ambiguous: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AdaptiveTetTriangleSource {
    pub triangle_id: u64,
    pub source_tetrahedron_id: u64,
    pub source_leaf_cell_id: u64,
    pub source_leaf_level: u8,
    pub marching_case: String,
    pub raw_centroid: [f32; 3],
    pub raw_triangle_normal: [f32; 3],
}

#[derive(Clone, Debug)]
struct LeafCell {
    id: u64,
    level: u8,
    origin: [i64; 3],
    extent: i64,
    min: Vec3,
    max: Vec3,
}

#[derive(Clone, Debug)]
struct Tet {
    id: u64,
    leaf_id: u64,
    leaf_level: u8,
    min_cell_dimension: f32,
    max_cell_dimension: f32,
    cell_diagonal: f32,
    vertices: [GridPointKey; 4],
}

#[derive(Clone, Copy, Debug)]
struct FaceRect {
    cell_id: u64,
    origin_uv: [i64; 2],
    extent: i64,
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct LineKey {
    axis: Axis,
    fixed: [i64; 2],
}

#[derive(Default)]
struct SdfSampleCache {
    values: BTreeMap<GridPointKey, f32>,
    hits: usize,
    misses: usize,
}

impl SdfSampleCache {
    fn distance(&mut self, sdf: &dyn Sdf, key: GridPointKey, p: Vec3) -> f32 {
        if let Some(value) = self.values.get(&key) {
            self.hits += 1;
            *value
        } else {
            self.misses += 1;
            let value = sdf.distance(p);
            self.values.insert(key, value);
            value
        }
    }
}

#[derive(Default)]
struct VertexAssembly {
    vertices: Vec<Vertex>,
    crossing_to_vertex: BTreeMap<TetEdgeKey, u32>,
    vertex_sources: Vec<AdaptiveTetVertexSource>,
    triangle_sources: Vec<AdaptiveTetTriangleSource>,
}

pub fn extract_mesh_adaptive_marching_tets(
    sdf: &dyn Sdf,
    octree: &AdaptiveOctree,
) -> (Mesh, AdaptiveTetExtractionStats) {
    let surface = extract_adaptive_marching_tets_with_sources(sdf, octree);
    (surface.mesh, surface.stats)
}

pub fn extract_adaptive_marching_tets_with_sources(
    sdf: &dyn Sdf,
    octree: &AdaptiveOctree,
) -> AdaptiveMarchingTetsSurface {
    let start = Instant::now();
    let max_level = octree
        .leaves
        .iter()
        .map(|leaf| leaf.level)
        .max()
        .unwrap_or(0)
        .min(u8::MAX as u32) as u8;
    let root_min = vec3_from_array(octree.bounds_min);
    let root_size = octree.root_size as f32;
    let leaves = octree
        .leaves
        .iter()
        .map(|leaf| leaf_to_cell(leaf, max_level))
        .collect::<Vec<_>>();
    let (face_registry, cell_face_keys) = build_face_registry(&leaves, max_level);
    let mut stats = AdaptiveTetExtractionStats {
        leaf_count: leaves.len(),
        max_level,
        face_registry_entries: face_registry.len(),
        face_triangles: face_registry
            .values()
            .map(|entry| entry.triangles.len())
            .sum(),
        ..AdaptiveTetExtractionStats::default()
    };

    let mut tets = Vec::<Tet>::new();
    let mut next_tet_id = 0_u64;
    for leaf in &leaves {
        let Some(boundary) =
            cell_boundary_triangles(leaf, &face_registry, &cell_face_keys, max_level)
        else {
            stats.boundary_invalid_cells += 1;
            continue;
        };
        stats.cell_boundary_triangles += boundary.len();
        let center = cell_center_key(leaf, max_level);
        let dims = leaf_dimensions(leaf);
        for tri in boundary {
            let tet = orient_tet([tri[0], tri[1], tri[2], center], root_min, root_size);
            if tet_volume_keys(&tet, root_min, root_size) > 0.0 {
                stats.positive_tetrahedra += 1;
            } else {
                stats.non_positive_tetrahedra += 1;
            }
            tets.push(Tet {
                id: next_tet_id,
                leaf_id: leaf.id,
                leaf_level: leaf.level,
                min_cell_dimension: dims[0],
                max_cell_dimension: dims[1],
                cell_diagonal: dims[2],
                vertices: tet,
            });
            next_tet_id += 1;
        }
    }
    stats.tetrahedra = tets.len();
    stats.edge_registry_points = collect_grid_points(&face_registry, &leaves, max_level).len();

    let mut sample_cache = SdfSampleCache::default();
    let mut assembly = VertexAssembly::default();
    let mut indices = Vec::<u32>::new();
    for tet in &tets {
        march_tet(
            sdf,
            tet,
            root_min,
            root_size,
            &mut sample_cache,
            &mut assembly,
            &mut indices,
        );
    }
    stats.sdf_cache_hits = sample_cache.hits;
    stats.sdf_cache_misses = sample_cache.misses;
    stats.crossing_vertices = assembly.vertices.len();
    stats.surface_triangles = indices.len() / 3;
    let mesh = Mesh {
        vertices: assembly.vertices,
        indices,
    };
    let topo = mesh_topology_counts(&mesh);
    stats.boundary_edges = topo.boundary_edges;
    stats.non_manifold_edges = topo.non_manifold_edges;
    stats.duplicate_triangles = topo.duplicate_triangles;
    stats.degenerate_triangles = topo.degenerate_triangles;
    stats.runtime_ms = start.elapsed().as_secs_f64() * 1000.0;
    AdaptiveMarchingTetsSurface {
        mesh,
        stats,
        vertex_sources: assembly.vertex_sources,
        triangle_sources: assembly.triangle_sources,
    }
}

pub fn summarize_extraction(
    mesh: &Mesh,
    stats: AdaptiveTetExtractionStats,
) -> AdaptiveMarchingTetsReport {
    let mut failure_reasons = Vec::new();
    if stats.boundary_invalid_cells > 0 {
        failure_reasons.push("invalid_cell_boundary".to_string());
    }
    if stats.non_positive_tetrahedra > 0 {
        failure_reasons.push("non_positive_tetrahedra".to_string());
    }
    if stats.boundary_edges > 0 {
        failure_reasons.push("surface_boundary_edges".to_string());
    }
    if stats.non_manifold_edges > 0 {
        failure_reasons.push("surface_non_manifold_edges".to_string());
    }
    if stats.duplicate_triangles > 0 {
        failure_reasons.push("duplicate_surface_triangles".to_string());
    }
    if stats.degenerate_triangles > 0 {
        failure_reasons.push("degenerate_surface_triangles".to_string());
    }
    AdaptiveMarchingTetsReport {
        mesh: MeshSummary {
            vertices: mesh.vertices.len(),
            triangles: mesh.indices.len() / 3,
            mesh_hash: mesh_hash(mesh),
        },
        stats,
        status: if failure_reasons.is_empty() {
            "success".to_string()
        } else {
            "failed_closed".to_string()
        },
        failure_reasons,
    }
}

fn leaf_to_cell(leaf: &AdaptiveOctreeNode, max_level: u8) -> LeafCell {
    let shift = max_level as u32 - leaf.level;
    let extent = 1_i64 << shift;
    LeafCell {
        id: leaf.id,
        level: leaf.level as u8,
        origin: [
            leaf.grid_coord[0] as i64 * extent,
            leaf.grid_coord[1] as i64 * extent,
            leaf.grid_coord[2] as i64 * extent,
        ],
        extent,
        min: vec3_from_array(leaf.bounds_min),
        max: vec3_from_array(leaf.bounds_max),
    }
}

fn leaf_dimensions(leaf: &LeafCell) -> [f32; 3] {
    let d = leaf.max - leaf.min;
    [d.x.min(d.y).min(d.z), d.x.max(d.y).max(d.z), d.length()]
}

fn push_unique_u64(values: &mut Vec<u64>, value: u64) {
    if !values.contains(&value) {
        values.push(value);
        values.sort_unstable();
    }
}

fn push_unique_u8(values: &mut Vec<u8>, value: u8) {
    if !values.contains(&value) {
        values.push(value);
        values.sort_unstable();
    }
}

fn build_face_registry(
    leaves: &[LeafCell],
    max_level: u8,
) -> (
    BTreeMap<CanonicalFaceKey, FaceRegistryEntry>,
    BTreeMap<u64, Vec<CanonicalFaceKey>>,
) {
    let mut plane_rects = BTreeMap::<(Axis, i64), Vec<FaceRect>>::new();
    for leaf in leaves {
        for axis in [Axis::X, Axis::Y, Axis::Z] {
            for side in [0_i64, 1_i64] {
                let plane = match axis {
                    Axis::X => leaf.origin[0] + side * leaf.extent,
                    Axis::Y => leaf.origin[1] + side * leaf.extent,
                    Axis::Z => leaf.origin[2] + side * leaf.extent,
                };
                let (u0, v0) = match axis {
                    Axis::X => (leaf.origin[1], leaf.origin[2]),
                    Axis::Y => (leaf.origin[0], leaf.origin[2]),
                    Axis::Z => (leaf.origin[0], leaf.origin[1]),
                };
                plane_rects
                    .entry((axis, plane))
                    .or_default()
                    .push(FaceRect {
                        cell_id: leaf.id,
                        origin_uv: [u0, v0],
                        extent: leaf.extent,
                    });
            }
        }
    }
    let mut edge_cuts = build_edge_cut_registry(leaves);
    for _ in 0..8 {
        if !augment_edge_cuts_from_face_cuts(&plane_rects, &mut edge_cuts) {
            break;
        }
    }
    let mut registry = BTreeMap::<CanonicalFaceKey, FaceRegistryEntry>::new();
    let mut cell_face_keys = BTreeMap::<u64, Vec<CanonicalFaceKey>>::new();
    for leaf in leaves {
        for axis in [Axis::X, Axis::Y, Axis::Z] {
            for side in [0_i64, 1_i64] {
                let plane = match axis {
                    Axis::X => leaf.origin[0] + side * leaf.extent,
                    Axis::Y => leaf.origin[1] + side * leaf.extent,
                    Axis::Z => leaf.origin[2] + side * leaf.extent,
                };
                let (u0, v0) = match axis {
                    Axis::X => (leaf.origin[1], leaf.origin[2]),
                    Axis::Y => (leaf.origin[0], leaf.origin[2]),
                    Axis::Z => (leaf.origin[0], leaf.origin[1]),
                };
                let rect = FaceRect {
                    cell_id: leaf.id,
                    origin_uv: [u0, v0],
                    extent: leaf.extent,
                };
                let Some(coplanar) = plane_rects.get(&(axis, plane)) else {
                    continue;
                };
                for key in locally_tiled_face_keys(axis, plane, rect, coplanar, &edge_cuts) {
                    cell_face_keys.entry(leaf.id).or_default().push(key);
                    registry
                        .entry(key)
                        .and_modify(|entry| {
                            if !entry.incident_cells.contains(&leaf.id) {
                                entry.incident_cells.push(leaf.id);
                                entry.incident_cells.sort_unstable();
                            }
                        })
                        .or_insert_with(|| FaceRegistryEntry {
                            key,
                            incident_cells: vec![leaf.id],
                            triangles: face_triangles(key, max_level),
                        });
                }
            }
        }
    }
    for keys in cell_face_keys.values_mut() {
        keys.sort_unstable();
        keys.dedup();
    }
    (registry, cell_face_keys)
}

fn build_edge_cut_registry(leaves: &[LeafCell]) -> BTreeMap<LineKey, BTreeSet<i64>> {
    let mut cuts = BTreeMap::<LineKey, BTreeSet<i64>>::new();
    for leaf in leaves {
        let [x0, y0, z0] = leaf.origin;
        let x1 = x0 + leaf.extent;
        let y1 = y0 + leaf.extent;
        let z1 = z0 + leaf.extent;
        for y in [y0, y1] {
            for z in [z0, z1] {
                cuts.entry(line_key(Axis::X, y, z))
                    .or_default()
                    .extend([x0, x1]);
            }
        }
        for x in [x0, x1] {
            for z in [z0, z1] {
                cuts.entry(line_key(Axis::Y, x, z))
                    .or_default()
                    .extend([y0, y1]);
            }
        }
        for x in [x0, x1] {
            for y in [y0, y1] {
                cuts.entry(line_key(Axis::Z, x, y))
                    .or_default()
                    .extend([z0, z1]);
            }
        }
    }
    cuts
}

fn line_key(axis: Axis, a: i64, b: i64) -> LineKey {
    LineKey {
        axis,
        fixed: [a, b],
    }
}

fn augment_edge_cuts_from_face_cuts(
    plane_rects: &BTreeMap<(Axis, i64), Vec<FaceRect>>,
    edge_cuts: &mut BTreeMap<LineKey, BTreeSet<i64>>,
) -> bool {
    let mut pending = Vec::<(LineKey, i64)>::new();
    for (&(axis, plane), coplanar) in plane_rects {
        let (u_axis, v_axis) = face_tangent_axes(axis);
        for rect in coplanar {
            let u0 = rect.origin_uv[0];
            let v0 = rect.origin_uv[1];
            let u1 = u0 + rect.extent;
            let v1 = v0 + rect.extent;
            let mut u_cuts = BTreeSet::<i64>::from([u0, u1]);
            let mut v_cuts = BTreeSet::<i64>::from([v0, v1]);
            for other in coplanar {
                if other.cell_id == rect.cell_id {
                    continue;
                }
                let ou0 = other.origin_uv[0];
                let ov0 = other.origin_uv[1];
                let ou1 = ou0 + other.extent;
                let ov1 = ov0 + other.extent;
                if ou0 >= u1 || ou1 <= u0 || ov0 >= v1 || ov1 <= v0 {
                    continue;
                }
                for cut in [ou0, ou1] {
                    if cut > u0 && cut < u1 {
                        u_cuts.insert(cut);
                    }
                }
                for cut in [ov0, ov1] {
                    if cut > v0 && cut < v1 {
                        v_cuts.insert(cut);
                    }
                }
            }
            add_boundary_edge_cuts(
                axis,
                plane,
                u0,
                u1,
                v0,
                v1,
                edge_cuts,
                &mut u_cuts,
                &mut v_cuts,
            );
            for cut in u_cuts {
                if cut > u0 && cut < u1 {
                    for v in [v0, v1] {
                        pending.push((face_boundary_line_key(axis, u_axis, plane, v), cut));
                    }
                }
            }
            for cut in v_cuts {
                if cut > v0 && cut < v1 {
                    for u in [u0, u1] {
                        pending.push((face_boundary_line_key(axis, v_axis, plane, u), cut));
                    }
                }
            }
        }
    }
    let mut changed = false;
    for (key, cut) in pending {
        if edge_cuts.entry(key).or_default().insert(cut) {
            changed = true;
        }
    }
    changed
}

fn locally_tiled_face_keys(
    axis: Axis,
    plane: i64,
    rect: FaceRect,
    coplanar: &[FaceRect],
    edge_cuts: &BTreeMap<LineKey, BTreeSet<i64>>,
) -> Vec<CanonicalFaceKey> {
    let u0 = rect.origin_uv[0];
    let v0 = rect.origin_uv[1];
    let u1 = u0 + rect.extent;
    let v1 = v0 + rect.extent;
    let mut u_cuts = BTreeSet::<i64>::from([u0, u1]);
    let mut v_cuts = BTreeSet::<i64>::from([v0, v1]);
    for other in coplanar {
        if other.cell_id == rect.cell_id {
            continue;
        }
        let ou0 = other.origin_uv[0];
        let ov0 = other.origin_uv[1];
        let ou1 = ou0 + other.extent;
        let ov1 = ov0 + other.extent;
        if ou0 >= u1 || ou1 <= u0 || ov0 >= v1 || ov1 <= v0 {
            continue;
        }
        for cut in [ou0, ou1] {
            if cut > u0 && cut < u1 {
                u_cuts.insert(cut);
            }
        }
        for cut in [ov0, ov1] {
            if cut > v0 && cut < v1 {
                v_cuts.insert(cut);
            }
        }
    }
    add_boundary_edge_cuts(
        axis,
        plane,
        u0,
        u1,
        v0,
        v1,
        edge_cuts,
        &mut u_cuts,
        &mut v_cuts,
    );
    let u_cuts = u_cuts.into_iter().collect::<Vec<_>>();
    let v_cuts = v_cuts.into_iter().collect::<Vec<_>>();
    let mut keys = Vec::<CanonicalFaceKey>::new();
    for u_pair in u_cuts.windows(2) {
        for v_pair in v_cuts.windows(2) {
            keys.push(CanonicalFaceKey {
                axis,
                plane,
                origin_uv: [u_pair[0], v_pair[0]],
                extent_uv: [u_pair[1] - u_pair[0], v_pair[1] - v_pair[0]],
            });
        }
    }
    keys.sort_unstable();
    keys.dedup();
    keys
}

fn add_boundary_edge_cuts(
    face_axis: Axis,
    plane: i64,
    u0: i64,
    u1: i64,
    v0: i64,
    v1: i64,
    edge_cuts: &BTreeMap<LineKey, BTreeSet<i64>>,
    u_cuts: &mut BTreeSet<i64>,
    v_cuts: &mut BTreeSet<i64>,
) {
    let (u_axis, v_axis) = face_tangent_axes(face_axis);
    for v in [v0, v1] {
        for cut in edge_cuts
            .get(&face_boundary_line_key(face_axis, u_axis, plane, v))
            .into_iter()
            .flatten()
        {
            if *cut > u0 && *cut < u1 {
                u_cuts.insert(*cut);
            }
        }
    }
    for u in [u0, u1] {
        for cut in edge_cuts
            .get(&face_boundary_line_key(face_axis, v_axis, plane, u))
            .into_iter()
            .flatten()
        {
            if *cut > v0 && *cut < v1 {
                v_cuts.insert(*cut);
            }
        }
    }
}

fn face_tangent_axes(axis: Axis) -> (Axis, Axis) {
    match axis {
        Axis::X => (Axis::Y, Axis::Z),
        Axis::Y => (Axis::X, Axis::Z),
        Axis::Z => (Axis::X, Axis::Y),
    }
}

fn face_boundary_line_key(
    face_axis: Axis,
    line_axis: Axis,
    plane: i64,
    other_tangent: i64,
) -> LineKey {
    match (face_axis, line_axis) {
        (Axis::X, Axis::Y) => line_key(Axis::Y, plane, other_tangent),
        (Axis::X, Axis::Z) => line_key(Axis::Z, plane, other_tangent),
        (Axis::Y, Axis::X) => line_key(Axis::X, plane, other_tangent),
        (Axis::Y, Axis::Z) => line_key(Axis::Z, other_tangent, plane),
        (Axis::Z, Axis::X) => line_key(Axis::X, other_tangent, plane),
        (Axis::Z, Axis::Y) => line_key(Axis::Y, other_tangent, plane),
        _ => unreachable!("face boundary line must be tangent to face"),
    }
}

fn face_triangles(key: CanonicalFaceKey, level: u8) -> Vec<[GridPointKey; 3]> {
    let [u, v] = key.origin_uv;
    let [du, dv] = key.extent_uv;
    let p00 = face_point(key.axis, key.plane, u, v, level);
    let p10 = face_point(key.axis, key.plane, u + du, v, level);
    let p01 = face_point(key.axis, key.plane, u, v + dv, level);
    let p11 = face_point(key.axis, key.plane, u + du, v + dv, level);
    if ((key.plane + u + v) & 1) == 0 {
        vec![[p00, p10, p11], [p00, p11, p01]]
    } else {
        vec![[p00, p10, p01], [p10, p11, p01]]
    }
}

fn face_point(axis: Axis, plane: i64, u: i64, v: i64, level: u8) -> GridPointKey {
    let xyz = match axis {
        Axis::X => [plane, u, v],
        Axis::Y => [u, plane, v],
        Axis::Z => [u, v, plane],
    };
    GridPointKey {
        xyz,
        denominator_level: level,
    }
}

fn cell_boundary_triangles(
    leaf: &LeafCell,
    registry: &BTreeMap<CanonicalFaceKey, FaceRegistryEntry>,
    cell_face_keys: &BTreeMap<u64, Vec<CanonicalFaceKey>>,
    max_level: u8,
) -> Option<Vec<[GridPointKey; 3]>> {
    let mut triangles = Vec::new();
    for key in cell_face_keys.get(&leaf.id)? {
        let entry = registry.get(key)?;
        for tri in &entry.triangles {
            let mut oriented = *tri;
            if !face_triangle_points_outward(oriented, leaf, max_level) {
                oriented.swap(1, 2);
            }
            triangles.push(oriented);
        }
    }
    if cell_boundary_is_closed(&triangles) {
        Some(triangles)
    } else {
        None
    }
}

fn face_triangle_points_outward(tri: [GridPointKey; 3], leaf: &LeafCell, max_level: u8) -> bool {
    let a = grid_point_to_lattice_vec(tri[0]);
    let b = grid_point_to_lattice_vec(tri[1]);
    let c = grid_point_to_lattice_vec(tri[2]);
    let normal = (b - a).cross(c - a);
    let centroid = (a + b + c) / 3.0;
    let cell_center = Vec3::new(
        leaf.origin[0] as f32 + leaf.extent as f32 * 0.5,
        leaf.origin[1] as f32 + leaf.extent as f32 * 0.5,
        leaf.origin[2] as f32 + leaf.extent as f32 * 0.5,
    );
    let _ = max_level;
    normal.dot(centroid - cell_center) > 0.0
}

fn cell_boundary_is_closed(triangles: &[[GridPointKey; 3]]) -> bool {
    let mut edges = BTreeMap::<(GridPointKey, GridPointKey), usize>::new();
    for tri in triangles {
        for (a, b) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            let key = if a <= b { (a, b) } else { (b, a) };
            *edges.entry(key).or_default() += 1;
        }
    }
    edges.values().all(|&count| count == 2)
}

fn cell_center_key(leaf: &LeafCell, max_level: u8) -> GridPointKey {
    GridPointKey {
        xyz: [
            leaf.origin[0] * 2 + leaf.extent,
            leaf.origin[1] * 2 + leaf.extent,
            leaf.origin[2] * 2 + leaf.extent,
        ],
        denominator_level: max_level + 1,
    }
}

fn orient_tet(mut tet: [GridPointKey; 4], root_min: Vec3, root_size: f32) -> [GridPointKey; 4] {
    if tet_volume_keys(&tet, root_min, root_size) < 0.0 {
        tet.swap(1, 2);
    }
    tet
}

fn tet_volume_keys(tet: &[GridPointKey; 4], root_min: Vec3, root_size: f32) -> f32 {
    let a = grid_point_to_world(tet[0], root_min, root_size);
    let b = grid_point_to_world(tet[1], root_min, root_size);
    let c = grid_point_to_world(tet[2], root_min, root_size);
    let d = grid_point_to_world(tet[3], root_min, root_size);
    (b - a).cross(c - a).dot(d - a) / 6.0
}

fn march_tet(
    sdf: &dyn Sdf,
    tet: &Tet,
    root_min: Vec3,
    root_size: f32,
    cache: &mut SdfSampleCache,
    assembly: &mut VertexAssembly,
    indices: &mut Vec<u32>,
) {
    let keys = tet.vertices;
    let positions = keys.map(|key| grid_point_to_world(key, root_min, root_size));
    let values =
        std::array::from_fn::<_, 4, _>(|idx| cache.distance(sdf, keys[idx], positions[idx]));
    let inside = values.map(|v| v < 0.0);
    let inside_count = inside.iter().filter(|&&v| v).count();
    if inside_count == 0 || inside_count == 4 {
        return;
    }

    let mut inside_ids = Vec::new();
    let mut outside_ids = Vec::new();
    for idx in 0..4 {
        if inside[idx] {
            inside_ids.push(idx);
        } else {
            outside_ids.push(idx);
        }
    }

    let mut emit = |tri_edges: [(usize, usize); 3], marching_case: &str| {
        let tri = tri_edges.map(|(a, b)| {
            crossing_vertex(
                sdf,
                tet,
                keys[a],
                keys[b],
                positions[a],
                positions[b],
                values[a],
                values[b],
                assembly,
            )
        });
        if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
            return;
        }
        let mut tri = tri;
        if assembled_triangle_area(&assembly.vertices, tri) <= 1e-10 {
            return;
        }
        if !triangle_matches_sdf_normal(sdf, &assembly.vertices, tri) {
            tri.swap(1, 2);
        }
        if assembled_triangle_area(&assembly.vertices, tri) <= 1e-10 {
            return;
        }
        let tri_id = (indices.len() / 3) as u64;
        assembly.triangle_sources.push(AdaptiveTetTriangleSource {
            triangle_id: tri_id,
            source_tetrahedron_id: tet.id,
            source_leaf_cell_id: tet.leaf_id,
            source_leaf_level: tet.leaf_level,
            marching_case: marching_case.to_string(),
            raw_centroid: assembled_triangle_centroid(&assembly.vertices, tri).to_array(),
            raw_triangle_normal: assembled_triangle_normal(&assembly.vertices, tri).to_array(),
        });
        indices.extend_from_slice(&tri);
    };

    match inside_count {
        1 => {
            let i = inside_ids[0];
            emit(
                [
                    (i, outside_ids[0]),
                    (i, outside_ids[1]),
                    (i, outside_ids[2]),
                ],
                "one_inside",
            );
        }
        3 => {
            let o = outside_ids[0];
            emit(
                [(inside_ids[0], o), (inside_ids[2], o), (inside_ids[1], o)],
                "three_inside",
            );
        }
        2 => {
            let i0 = inside_ids[0];
            let i1 = inside_ids[1];
            let o0 = outside_ids[0];
            let o1 = outside_ids[1];
            emit([(i0, o0), (i1, o0), (i1, o1)], "two_inside_quad_a");
            emit([(i0, o0), (i1, o1), (i0, o1)], "two_inside_quad_b");
        }
        _ => {}
    }
}

#[allow(clippy::too_many_arguments)]
fn crossing_vertex(
    sdf: &dyn Sdf,
    tet: &Tet,
    ka: GridPointKey,
    kb: GridPointKey,
    pa: Vec3,
    pb: Vec3,
    da: f32,
    db: f32,
    assembly: &mut VertexAssembly,
) -> u32 {
    let key = TetEdgeKey::new(ka, kb);
    if let Some(&idx) = assembly.crossing_to_vertex.get(&key) {
        if let Some(source) = assembly.vertex_sources.get_mut(idx as usize) {
            push_unique_u64(&mut source.incident_tetrahedron_ids, tet.id);
            push_unique_u64(&mut source.incident_leaf_cell_ids, tet.leaf_id);
            push_unique_u8(&mut source.incident_leaf_levels, tet.leaf_level);
            source.min_local_cell_dimension =
                source.min_local_cell_dimension.min(tet.min_cell_dimension);
            source.max_local_cell_dimension =
                source.max_local_cell_dimension.max(tet.max_cell_dimension);
            source.local_cell_diagonal = source.local_cell_diagonal.max(tet.cell_diagonal);
        }
        return idx;
    }
    let t = if (db - da).abs() > 1e-8 {
        (-da / (db - da)).clamp(0.0, 1.0)
    } else {
        0.5
    };
    let p = pa.lerp(pb, t);
    let n = sdf_gradient(sdf, p);
    let idx = assembly.vertices.len() as u32;
    assembly.vertices.push(Vertex {
        position: p.to_array(),
        normal: n.to_array(),
    });
    assembly.crossing_to_vertex.insert(key, idx);
    let endpoints = if ka <= kb { [ka, kb] } else { [kb, ka] };
    let distances = if ka <= kb { [da, db] } else { [db, da] };
    let interpolation_parameter = if ka <= kb { t } else { 1.0 - t };
    assembly.vertex_sources.push(AdaptiveTetVertexSource {
        vertex_id: idx,
        tet_edge_key: key,
        endpoint_keys: endpoints,
        endpoint_signed_distances: distances,
        interpolation_parameter,
        incident_tetrahedron_ids: vec![tet.id],
        incident_leaf_cell_ids: vec![tet.leaf_id],
        incident_leaf_levels: vec![tet.leaf_level],
        min_local_cell_dimension: tet.min_cell_dimension,
        max_local_cell_dimension: tet.max_cell_dimension,
        local_cell_diagonal: tet.cell_diagonal,
        geometrically_ambiguous: da.abs() <= 1.0e-7
            || db.abs() <= 1.0e-7
            || (db - da).abs() <= 1.0e-8,
    });
    idx
}

fn triangle_matches_sdf_normal(sdf: &dyn Sdf, vertices: &[Vertex], tri: [u32; 3]) -> bool {
    let a = Vec3::from_array(vertices[tri[0] as usize].position);
    let b = Vec3::from_array(vertices[tri[1] as usize].position);
    let c = Vec3::from_array(vertices[tri[2] as usize].position);
    let normal = (b - a).cross(c - a).normalize_or_zero();
    let centroid = (a + b + c) / 3.0;
    normal.dot(sdf_gradient(sdf, centroid)) >= 0.0
}

fn assembled_triangle_area(vertices: &[Vertex], tri: [u32; 3]) -> f32 {
    let a = Vec3::from_array(vertices[tri[0] as usize].position);
    let b = Vec3::from_array(vertices[tri[1] as usize].position);
    let c = Vec3::from_array(vertices[tri[2] as usize].position);
    0.5 * (b - a).cross(c - a).length()
}

fn assembled_triangle_centroid(vertices: &[Vertex], tri: [u32; 3]) -> Vec3 {
    let a = Vec3::from_array(vertices[tri[0] as usize].position);
    let b = Vec3::from_array(vertices[tri[1] as usize].position);
    let c = Vec3::from_array(vertices[tri[2] as usize].position);
    (a + b + c) / 3.0
}

fn assembled_triangle_normal(vertices: &[Vertex], tri: [u32; 3]) -> Vec3 {
    let a = Vec3::from_array(vertices[tri[0] as usize].position);
    let b = Vec3::from_array(vertices[tri[1] as usize].position);
    let c = Vec3::from_array(vertices[tri[2] as usize].position);
    (b - a).cross(c - a).normalize_or_zero()
}

fn sdf_gradient(sdf: &dyn Sdf, p: Vec3) -> Vec3 {
    let eps = 1e-3f32;
    Vec3::new(
        sdf.distance(p + Vec3::X * eps) - sdf.distance(p - Vec3::X * eps),
        sdf.distance(p + Vec3::Y * eps) - sdf.distance(p - Vec3::Y * eps),
        sdf.distance(p + Vec3::Z * eps) - sdf.distance(p - Vec3::Z * eps),
    )
    .normalize_or_zero()
}

fn grid_point_to_lattice_vec(key: GridPointKey) -> Vec3 {
    let scale = if key.denominator_level == 0 {
        1.0
    } else {
        1.0 / (1_i64 << key.denominator_level) as f32
    };
    Vec3::new(key.xyz[0] as f32, key.xyz[1] as f32, key.xyz[2] as f32) * scale
}

fn grid_point_to_world(key: GridPointKey, root_min: Vec3, root_size: f32) -> Vec3 {
    let denom = (1_i64 << key.denominator_level).max(1) as f32;
    let factor = root_size / denom;
    root_min + Vec3::new(key.xyz[0] as f32, key.xyz[1] as f32, key.xyz[2] as f32) * factor
}

fn collect_grid_points(
    face_registry: &BTreeMap<CanonicalFaceKey, FaceRegistryEntry>,
    leaves: &[LeafCell],
    max_level: u8,
) -> BTreeSet<GridPointKey> {
    let mut points = BTreeSet::new();
    for entry in face_registry.values() {
        for tri in &entry.triangles {
            points.extend(tri);
        }
    }
    for leaf in leaves {
        points.insert(cell_center_key(leaf, max_level));
    }
    points
}

#[derive(Default)]
struct MeshTopo {
    boundary_edges: usize,
    non_manifold_edges: usize,
    duplicate_triangles: usize,
    degenerate_triangles: usize,
}

fn mesh_topology_counts(mesh: &Mesh) -> MeshTopo {
    let mut edges = BTreeMap::<(u32, u32), usize>::new();
    let mut tris = BTreeSet::<[u32; 3]>::new();
    let mut topo = MeshTopo::default();
    for tri in mesh.indices.chunks_exact(3) {
        let t = [tri[0], tri[1], tri[2]];
        if t[0] == t[1] || t[1] == t[2] || t[2] == t[0] || triangle_area(mesh, t) <= 1e-12 {
            topo.degenerate_triangles += 1;
        }
        let mut canonical = t;
        canonical.sort_unstable();
        if !tris.insert(canonical) {
            topo.duplicate_triangles += 1;
        }
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            let edge = if a <= b { (a, b) } else { (b, a) };
            *edges.entry(edge).or_default() += 1;
        }
    }
    topo.boundary_edges = edges.values().filter(|&&count| count == 1).count();
    topo.non_manifold_edges = edges.values().filter(|&&count| count > 2).count();
    topo
}

fn triangle_area(mesh: &Mesh, tri: [u32; 3]) -> f32 {
    let a = Vec3::from_array(mesh.vertices[tri[0] as usize].position);
    let b = Vec3::from_array(mesh.vertices[tri[1] as usize].position);
    let c = Vec3::from_array(mesh.vertices[tri[2] as usize].position);
    0.5 * (b - a).cross(c - a).length()
}

fn mesh_hash(mesh: &Mesh) -> String {
    let mut h = 1469598103934665603u64;
    for v in &mesh.vertices {
        for x in v.position {
            h ^= x.to_bits() as u64;
            h = h.wrapping_mul(1099511628211);
        }
    }
    for idx in &mesh.indices {
        h ^= *idx as u64;
        h = h.wrapping_mul(1099511628211);
    }
    format!("{h:016x}")
}

fn vec3_from_array(v: [f64; 3]) -> Vec3 {
    Vec3::new(v[0] as f32, v[1] as f32, v[2] as f32)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::adaptive_sampling::{AdaptiveSamplingConfig, build_adaptive_octree};
    use crate::sdf::primitives::Sphere;

    #[test]
    fn tet_edge_key_canonicalizes_endpoint_order() {
        let a = GridPointKey {
            xyz: [0, 0, 0],
            denominator_level: 2,
        };
        let b = GridPointKey {
            xyz: [1, 0, 0],
            denominator_level: 2,
        };
        assert_eq!(TetEdgeKey::new(a, b), TetEdgeKey::new(b, a));
    }

    #[test]
    fn sphere_marching_tets_extracts_deterministic_surface() {
        let sphere = Sphere::new(3.7);
        let config = AdaptiveSamplingConfig {
            min_depth: 2,
            max_depth: 3,
            base_cell_size: 4.0,
            max_leaf_cells: 4096,
            balance_face_neighbors: true,
            ..AdaptiveSamplingConfig::default()
        };
        let octree = build_adaptive_octree(&sphere, &config).unwrap();
        let (a_mesh, a_stats) = extract_mesh_adaptive_marching_tets(&sphere, &octree);
        let (b_mesh, b_stats) = extract_mesh_adaptive_marching_tets(&sphere, &octree);
        assert!(!a_mesh.vertices.is_empty());
        assert!(!a_mesh.indices.is_empty());
        assert_eq!(mesh_hash(&a_mesh), mesh_hash(&b_mesh));
        assert_eq!(a_stats.face_registry_entries, b_stats.face_registry_entries);
        assert_eq!(a_stats.boundary_edges, 0);
        assert_eq!(a_stats.non_manifold_edges, 0);
        assert_eq!(a_stats.degenerate_triangles, 0);
    }

    #[test]
    fn marching_tets_retains_source_metadata() {
        let sphere = Sphere::new(3.7);
        let config = AdaptiveSamplingConfig {
            min_depth: 2,
            max_depth: 3,
            base_cell_size: 4.0,
            max_leaf_cells: 4096,
            balance_face_neighbors: true,
            ..AdaptiveSamplingConfig::default()
        };
        let octree = build_adaptive_octree(&sphere, &config).unwrap();
        let a = extract_adaptive_marching_tets_with_sources(&sphere, &octree);
        let b = extract_adaptive_marching_tets_with_sources(&sphere, &octree);

        assert_eq!(a.vertex_sources.len(), a.mesh.vertices.len());
        assert_eq!(a.triangle_sources.len(), a.mesh.indices.len() / 3);
        assert_eq!(a.vertex_sources, b.vertex_sources);
        assert_eq!(a.triangle_sources, b.triangle_sources);
        assert_eq!(mesh_hash(&a.mesh), mesh_hash(&b.mesh));

        for (idx, source) in a.vertex_sources.iter().enumerate().take(64) {
            assert_eq!(source.vertex_id as usize, idx);
            assert!(source.min_local_cell_dimension.is_finite());
            assert!(source.max_local_cell_dimension >= source.min_local_cell_dimension);
            assert!(source.local_cell_diagonal >= source.max_local_cell_dimension);
            assert!(!source.incident_tetrahedron_ids.is_empty());
            assert!(!source.incident_leaf_cell_ids.is_empty());
            assert!(!source.incident_leaf_levels.is_empty());
        }

        for (idx, source) in a.triangle_sources.iter().enumerate().take(64) {
            assert_eq!(source.triangle_id as usize, idx);
            assert!(source.raw_centroid.iter().all(|v| v.is_finite()));
            assert!(source.raw_triangle_normal.iter().all(|v| v.is_finite()));
        }
    }

    #[test]
    fn face_registry_has_shared_entries_for_balanced_octree() {
        let sphere = Sphere::new(3.0);
        let config = AdaptiveSamplingConfig {
            min_depth: 1,
            max_depth: 2,
            base_cell_size: 6.0,
            max_leaf_cells: 1024,
            balance_face_neighbors: true,
            ..AdaptiveSamplingConfig::default()
        };
        let octree = build_adaptive_octree(&sphere, &config).unwrap();
        let max_level = octree.leaves.iter().map(|l| l.level).max().unwrap() as u8;
        let leaves = octree
            .leaves
            .iter()
            .map(|l| leaf_to_cell(l, max_level))
            .collect::<Vec<_>>();
        let (registry, cell_face_keys) = build_face_registry(&leaves, max_level);
        assert!(
            registry
                .values()
                .any(|entry| entry.incident_cells.len() == 2)
        );
        assert!(registry.values().all(|entry| !entry.triangles.is_empty()));
        assert!(cell_face_keys.values().all(|keys| !keys.is_empty()));
    }
}
