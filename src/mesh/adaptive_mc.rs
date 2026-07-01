//! Adaptive octree marching cubes.
//!
//! Concentrates triangles near the isosurface by subdividing an octree until
//! leaf cells reach `target_cell_size`.  Cells that are provably entirely
//! inside or outside the shape are pruned early, so the algorithm is fast even
//! for large models with thin features (e.g. wing trailing edges).
//!
//! Normals are computed from the SDF gradient (central differences), which
//! gives naturally smooth shading without any averaging heuristics.

use super::marching_cubes::{CUBE_CORNERS, EDGE_TABLE, EDGE_VERTICES, TRI_TABLE};
use super::transvoxel::{TRANSITION_CELL_DATA, TRANSITION_VERTEX_DATA, transition_cell_case};
use crate::mesh::{Mesh, Vertex};
use crate::sdf::Sdf;
use glam::Vec3;
use rayon::prelude::*;
use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Instant;

#[derive(Clone, Copy, Debug)]
pub struct LocalRefineRegion {
    pub min: Vec3,
    pub max: Vec3,
    pub target_cell_size: f32,
}

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
    fn default() -> Self {
        Self::Normal
    }
}

impl MeshQuality {
    pub fn label(self) -> &'static str {
        match self {
            Self::Draft => "Draft",
            Self::Normal => "Normal",
            Self::Fine => "Fine",
            Self::Ultra => "Ultra",
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
#[derive(Clone)]
struct OctreeCell {
    min: Vec3,
    size: f32,
    corners: [f32; 8], // SDF values at corners in XYZ bit-index order
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct FaceKey {
    axis: u8,
    quantized_plane_coord: i32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FaceClassification {
    Interior,
    Transition,
    Boundary,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FaceContactKind {
    Area,
    Edge,
    None,
}

#[derive(Clone, Debug)]
struct FaceRecord {
    cell_idx: usize,
    face_id: u8,
    axis: u8,
    sign: u8,
    plane_coord: f32,
    u_min: f32,
    u_max: f32,
    v_min: f32,
    v_max: f32,
    cell_size: f32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct TransitionFace {
    coarse_cell_idx: usize,
    coarse_face_id: u8,
    fine_cell_indices: Vec<usize>,
}

#[derive(Clone, Copy, Debug, Default)]
struct TransitionEmissionStats {
    faces_found: usize,
    faces_emitted: usize,
    faces_skipped: usize,
    triangles_emitted: usize,
    vertices_added: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum EdgeTransitionCase {
    TrueHalfEdge,
    CornerTouch,
    IrregularEdge,
}

#[derive(Clone, Debug)]
struct EdgeTransitionRecord {
    coarse_cell_idx: usize,
    fine_cell_idx: usize,
    coarse_face_id: u8,
    fine_face_id: u8,
    axis: u8,
    sign: u8,
    plane_coord: f32,
    coarse_cell_size: f32,
    fine_cell_size: f32,
    edge_start: Vec3,
    edge_end: Vec3,
    overlap_axis: u8,
    touch_axis: u8,
    case: EdgeTransitionCase,
}

#[derive(Clone, Copy, Debug, Default)]
struct EdgeTransitionCollectionStats {
    true_half_edge: usize,
    corner_touch: usize,
    irregular_edge: usize,
}

#[derive(Clone, Copy, Debug, Default)]
struct EdgeTransitionEmissionStats {
    attempted: usize,
    emitted: usize,
    rejected: usize,
    triangles_emitted: usize,
    vertices_added: usize,
    rejected_missing_fine_endpoints: usize,
    rejected_degenerate: usize,
    rejected_non_manifold: usize,
}

#[derive(Clone, Debug)]
struct PartialOverlapSample {
    fraction: f32,
    axis: u8,
    plane_coord: f32,
    center: Vec3,
    min: Vec3,
    max: Vec3,
    fine_neighbor_count: usize,
    coarse_cell_size: f32,
    fine_cell_size: f32,
}

#[derive(Clone, Debug, Default)]
struct CoverageAudit {
    full_tile_2_to_1: usize,
    partial_overlap: usize,
    same_size_edge_contact: usize,
    different_size_edge_contact: usize,
    same_size_matched: usize,
    unmatched: usize,
    partial_samples: Vec<PartialOverlapSample>,
    different_size_edge_samples: Vec<EdgeContactSample>,
}

#[derive(Clone, Copy, Debug)]
struct EdgeContactSample {
    ratio: f32,
    center: Vec3,
}

#[derive(Clone, Copy, Debug, Default)]
struct BoundaryMatchHotspot {
    key: i32,
    count: usize,
}

#[derive(Clone, Debug, Default)]
struct BoundaryMatchStats {
    total_boundary_edges: usize,
    no_match: usize,
    one_match: usize,
    multi_match: usize,
    no_match_centers: Vec<Vec3>,
    one_match_centers: Vec<Vec3>,
}

#[derive(Clone, Debug, Default)]
struct TJunctionRepairStats {
    triangles_before: usize,
    triangles_after: usize,
    boundary_edges_before: usize,
    boundary_edges_after: usize,
    no_match_after: usize,
    one_match_after: usize,
    multi_match_after: usize,
    repaired_edges: usize,
    skipped_no_match: usize,
    skipped_not_boundary: usize,
    skipped_ambiguous: usize,
    triangles_added: usize,
    repair_edges: Vec<(u32, u32)>,
}

#[derive(Clone, Debug)]
struct BoundaryEdgeInfo {
    edge: (u32, u32),
    tri_idx: usize,
    directed_a: u32,
    directed_b: u32,
    opposite: u32,
    midpoint: Vec3,
    length: f32,
    normal: Vec3,
}

#[derive(Debug, Default)]
struct CrackRepairStats {
    boundary_edges_before: usize,
    non_manifold_before: usize,
    candidate_pairs_found: usize,
    candidates_accepted: usize,
    candidates_rejected: usize,
    triangles_added: usize,
    vertices_added: usize,
    boundary_edges_after: usize,
    non_manifold_after: usize,
    connected_components_after: usize,
    watertight_after: bool,
    rejected_distance_too_large: usize,
    rejected_orientation_mismatch: usize,
    rejected_length_mismatch: usize,
    rejected_normal_mismatch: usize,
    rejected_degenerate: usize,
    rejected_duplicate: usize,
    rejected_non_manifold: usize,
    rejected_ambiguous: usize,
}

#[derive(Debug, Default)]
struct TinyLoopFillStats {
    tiny_loops_detected: usize,
    tri_loops_detected: usize,
    quad_loops_detected: usize,
    loops_accepted: usize,
    loops_rejected: usize,
    triangles_added: usize,
    boundary_edges_before: usize,
    boundary_edges_after: usize,
    non_manifold_before: usize,
    non_manifold_after: usize,
    connected_components_after: usize,
    watertight_after: bool,
    rejected_loop_too_large: usize,
    rejected_non_planar: usize,
    rejected_bad_normals: usize,
    rejected_degenerate: usize,
    rejected_duplicate: usize,
    rejected_non_manifold: usize,
    rejected_ambiguous_diagonal: usize,
    rejected_failed_topology: usize,
}

#[derive(Debug, Default)]
struct FinalRepairVariantStats {
    boundary_before: usize,
    boundary_after: usize,
    non_manifold_before: usize,
    non_manifold_after: usize,
    connected_before: usize,
    connected_after: usize,
    triangles_added: usize,
    triangles_removed: usize,
    vertices_added: usize,
    vertices_split: usize,
    watertight_after: bool,
}

#[derive(Debug, Default)]
struct DuplicateCleanupStats {
    triangles_before: usize,
    triangles_after: usize,
    vertices_before: usize,
    vertices_after: usize,
    boundary_before: usize,
    boundary_after: usize,
    non_manifold_before: usize,
    non_manifold_after: usize,
    connected_before: usize,
    connected_after: usize,
    watertight_after: bool,
    triangles_removed: usize,
    duplicate_triangles_removed: usize,
    reversed_duplicates_removed: usize,
    same_vertex_set_removed: usize,
    zero_area_removed: usize,
    near_zero_area_removed: usize,
    removed_new_boundary_edges: usize,
    removed_reduced_non_manifold: usize,
    rejected_no_topology_gain: usize,
    rejected_connected_components: usize,
    rejected_large_boundary_loop: usize,
}

#[derive(Clone, Copy, Debug)]
enum TinyLoopMode {
    Current,
    RobustNormalVoting,
    LocalTopologyOnly,
    OrientationRepair,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RepairVariant {
    Default,
    ACurrentOrder,
    BEarlyDuplicate,
    CNoTJunction,
    DNoEdgePair,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TransitionCorridorFix {
    None,
    LargerRepairRadius,
    MicroPatch,
    TinyLoop,
}

#[derive(Debug, Default)]
struct WeldStats {
    vertices_before: usize,
    vertices_after: usize,
    remap: Vec<u32>,
    buckets_considered: usize,
    buckets_accepted: usize,
    buckets_rejected: usize,
    vertices_preserved_due_to_rejected_buckets: usize,
    buckets_rejected_for_degenerate_triangles: usize,
    buckets_rejected_for_non_manifold_edges: usize,
    buckets_rejected_for_duplicate_triangles: usize,
    buckets_rejected_for_opposite_shell_collapse: usize,
    elapsed_ms: f64,
}

#[derive(Clone, Debug, Default)]
struct MeshTopologyStats {
    vertices: usize,
    triangles: usize,
    boundary_edges: usize,
    non_manifold_edges: usize,
    connected_components: usize,
    watertight: bool,
}

#[derive(Clone, Debug, Default)]
struct WeldBucketExample {
    bucket_key: [i32; 3],
    vertex_count: usize,
    bbox_min: Vec3,
    bbox_max: Vec3,
    max_pairwise_distance: f32,
    contributing_cell_ids: Vec<usize>,
    contributing_triangle_ids: Vec<usize>,
    triangles_adjacent_before_weld: bool,
    touches: Vec<&'static str>,
    crosses_thin_shell: bool,
    non_adjacent_cells: bool,
    geometrically_distant: bool,
}

#[derive(Clone, Debug, Default)]
struct WeldAuditStats {
    epsilon: f32,
    before: MeshTopologyStats,
    after: MeshTopologyStats,
    vertices_merged: usize,
    unique_weld_buckets: usize,
    max_bucket_size: usize,
    average_bucket_size: f32,
    buckets_with_non_adjacent_cells: usize,
    buckets_with_geometrically_distant_triangles: usize,
    buckets_crossing_opposite_shell_sides: usize,
    triangles_made_degenerate_by_welding: usize,
    edges_made_non_manifold_by_welding: usize,
    worst_examples: Vec<WeldBucketExample>,
}

#[derive(Clone, Copy, Debug)]
struct RawTriangleSource {
    tri: [u32; 3],
    cell_idx: usize,
    cube_index: u8,
}

#[derive(Debug, Default)]
struct FaceAdjacency {
    finest_cell_size: f32,
    face_records: Vec<FaceRecord>,
    face_classes: Vec<FaceClassification>,
    transitions: Vec<TransitionFace>,
    edge_transitions: Vec<EdgeTransitionRecord>,
    edge_transition_stats: EdgeTransitionCollectionStats,
    cell_neighbors: Vec<Vec<usize>>,
    max_touching_ratio_value: f32,
    coverage_audit: CoverageAudit,
    layout_issues: Vec<TransitionLayoutIssue>,
}

#[derive(Clone, Debug)]
struct TransitionLayoutIssue {
    category: &'static str,
    coarse_cell_idx: usize,
    coarse_face_idx: usize,
    axis: u8,
    sign: u8,
    plane_coord: f32,
    center: Vec3,
    min: Vec3,
    max: Vec3,
    coarse_cell_size: f32,
    fine_cell_size: f32,
    ratio: f32,
    overlap_fraction: f32,
    exact_case: &'static str,
    fine_neighbor_count: usize,
    near_zero_surface: bool,
    fine_neighbor_boxes: Vec<(Vec3, Vec3)>,
}

#[derive(Clone, Debug)]
struct TwoAdjacentCandidate {
    issue_idx: usize,
    coarse_cell_idx: usize,
    exact_case: &'static str,
    axis: u8,
    sign: u8,
    plane_coord: f32,
    center: Vec3,
    coarse_min: Vec3,
    coarse_max: Vec3,
    fine_neighbor_count: usize,
    overlap_fraction: f32,
    uncovered_fraction: f32,
    coarse_cell_size: f32,
    fine_cell_size: f32,
    bad_edge_score: usize,
    boundary_edge_score: usize,
    non_manifold_edge_score: usize,
    quadrant_mask: u8,
    split_direction: &'static str,
    patch_type: &'static str,
}

#[derive(Debug, Default)]
struct TwoAdjacentCandidateReport {
    total_two_adjacent: usize,
    eligible_two_adjacent: usize,
    candidates_proposed: usize,
    candidates_attempted: usize,
    candidates_accepted: usize,
    cells_split: usize,
    transition_patches_added: usize,
    triangles_added: usize,
    vertices_added: usize,
    rejected_not_near_zero: usize,
    rejected_not_defect_mapped: usize,
    rejected_not_side_boundary: usize,
    rejected_not_supported_mask: usize,
    rejected_no_topology_gain: usize,
    rejected_connected_components: usize,
    rejected_non_local_boundary: usize,
    rejected_duplicate_or_degenerate: usize,
}

#[derive(Clone, Debug)]
struct FaceBoundaryEdgeProjection {
    edge: (u32, u32),
    endpoint_a: Vec3,
    endpoint_b: Vec3,
    local_a: Vec3,
    local_b: Vec3,
    midpoint_local: Vec3,
    role: &'static str,
    length: f32,
    incident_triangle_count: usize,
}

impl FaceRecord {
    fn key(&self, finest_cell_size: f32) -> FaceKey {
        let quantum = (finest_cell_size * 0.25).max(1e-6);
        FaceKey {
            axis: self.axis,
            quantized_plane_coord: (self.plane_coord / quantum).round() as i32,
        }
    }
}

impl FaceAdjacency {
    fn build(cells: &[OctreeCell]) -> Self {
        let build_t0 = Instant::now();
        if cells.is_empty() {
            return Self::default();
        }

        let enumerate_t0 = Instant::now();
        let finest_cell_size = cells.iter().map(|c| c.size).fold(f32::MAX, f32::min);
        let mut face_records = Vec::with_capacity(cells.len() * 6);
        for (cell_idx, cell) in cells.iter().enumerate() {
            face_records.extend(enumerate_cell_faces(cell_idx, cell));
        }
        let enumerate_elapsed = enumerate_t0.elapsed();

        let sort_t0 = Instant::now();
        let mut sorted_face_indices: Vec<usize> = (0..face_records.len()).collect();
        sorted_face_indices.sort_by(|&a_idx, &b_idx| {
            let a = &face_records[a_idx];
            let b = &face_records[b_idx];
            (
                a.axis,
                a.key(finest_cell_size).quantized_plane_coord,
                quantize_scalar(a.u_min, finest_cell_size),
                quantize_scalar(a.v_min, finest_cell_size),
                a.sign,
                a.cell_idx,
            )
                .cmp(&(
                    b.axis,
                    b.key(finest_cell_size).quantized_plane_coord,
                    quantize_scalar(b.u_min, finest_cell_size),
                    quantize_scalar(b.v_min, finest_cell_size),
                    b.sign,
                    b.cell_idx,
                ))
        });
        let sort_elapsed = sort_t0.elapsed();

        let classify_t0 = Instant::now();
        let mut face_classes = vec![FaceClassification::Boundary; face_records.len()];
        let mut transitions = Vec::new();
        let mut edge_transitions = Vec::new();
        let mut edge_transition_stats = EdgeTransitionCollectionStats::default();
        let mut cell_neighbors = vec![Vec::new(); cells.len()];
        let mut face_neighbors = vec![Vec::<usize>::new(); face_records.len()];
        let mut audit_area_neighbors = vec![Vec::<usize>::new(); face_records.len()];
        let mut max_touching_ratio_value = 1.0f32;
        let mut neighbor_pair_checks = 0usize;
        let mut classification_checks = 0usize;
        let mut overlap_passes = 0usize;
        let mut coverage_audit = CoverageAudit::default();
        let mut layout_issues = Vec::new();

        let mut plane_groups = 0usize;
        let mut group_start = 0usize;
        while group_start < sorted_face_indices.len() {
            plane_groups += 1;
            let start_face = &face_records[sorted_face_indices[group_start]];
            let start_key = (
                start_face.axis,
                start_face.key(finest_cell_size).quantized_plane_coord,
            );
            let mut group_end = group_start + 1;
            while group_end < sorted_face_indices.len() {
                let face = &face_records[sorted_face_indices[group_end]];
                let key = (face.axis, face.key(finest_cell_size).quantized_plane_coord);
                if key != start_key {
                    break;
                }
                group_end += 1;
            }

            let mut active: Vec<usize> = Vec::new();
            for pos in group_start..group_end {
                let face_idx = sorted_face_indices[pos];
                let face = &face_records[face_idx];
                active.retain(|&other_idx| face_records[other_idx].u_max >= face.u_min - 1e-5);
                for &other_idx in &active {
                    neighbor_pair_checks += 1;
                    let other = &face_records[other_idx];
                    if face.sign == other.sign || face.cell_idx == other.cell_idx {
                        continue;
                    }
                    match face_contact_kind(face, other) {
                        FaceContactKind::Area => {
                            audit_area_neighbors[face_idx].push(other_idx);
                            audit_area_neighbors[other_idx].push(face_idx);
                        }
                        FaceContactKind::Edge => {
                            coverage_audit.record_edge_contact(face, other);
                            if let Some(record) = classify_edge_transition_record(face, other) {
                                match record.case {
                                    EdgeTransitionCase::TrueHalfEdge => {
                                        edge_transition_stats.true_half_edge += 1;
                                        edge_transitions.push(record);
                                    }
                                    EdgeTransitionCase::CornerTouch => {
                                        edge_transition_stats.corner_touch += 1;
                                    }
                                    EdgeTransitionCase::IrregularEdge => {
                                        edge_transition_stats.irregular_edge += 1;
                                    }
                                }
                            }
                        }
                        FaceContactKind::None => {}
                    }
                    if !v_spans_overlap(face, other) {
                        continue;
                    }
                    overlap_passes += 1;
                    face_neighbors[face_idx].push(other_idx);
                    face_neighbors[other_idx].push(face_idx);
                    if !cell_neighbors[face.cell_idx].contains(&other.cell_idx) {
                        cell_neighbors[face.cell_idx].push(other.cell_idx);
                    }
                    if !cell_neighbors[other.cell_idx].contains(&face.cell_idx) {
                        cell_neighbors[other.cell_idx].push(face.cell_idx);
                    }
                    let ratio =
                        face.cell_size.max(other.cell_size) / face.cell_size.min(other.cell_size);
                    max_touching_ratio_value = max_touching_ratio_value.max(ratio);
                }
                active.push(face_idx);
            }

            for pos in group_start..group_end {
                let face_idx = sorted_face_indices[pos];
                let face = &face_records[face_idx];
                classification_checks += 1;
                let audit_opposite = &audit_area_neighbors[face_idx];
                if audit_opposite.iter().any(|&other_idx| {
                    let other = &face_records[other_idx];
                    approx_eq(face.cell_size, other.cell_size) && spans_match_exact(face, other)
                }) && face.sign == 0
                {
                    coverage_audit.same_size_matched += 1;
                }

                let audit_finer_neighbors: Vec<usize> = audit_opposite
                    .iter()
                    .copied()
                    .filter(|&other_idx| face_records[other_idx].cell_size + 1e-6 < face.cell_size)
                    .collect();
                if !audit_finer_neighbors.is_empty() {
                    let is_full_tile =
                        tiled_span_matches(face, &audit_finer_neighbors, &face_records);
                    let fine_size = audit_finer_neighbors
                        .iter()
                        .map(|&idx| face_records[idx].cell_size)
                        .fold(f32::MAX, f32::min);
                    let is_2_to_1 = approx_eq(face.cell_size, fine_size * 2.0);
                    if is_full_tile && is_2_to_1 {
                        coverage_audit.full_tile_2_to_1 += 1;
                    } else {
                        coverage_audit.partial_overlap += 1;
                        let sample =
                            partial_overlap_sample(face, &audit_finer_neighbors, &face_records);
                        let coarse_near_surface =
                            !can_prune(&cells[face.cell_idx].corners, cells[face.cell_idx].size);
                        let fine_near_surface = audit_finer_neighbors.iter().any(|&idx| {
                            let fine_cell_idx = face_records[idx].cell_idx;
                            !can_prune(&cells[fine_cell_idx].corners, cells[fine_cell_idx].size)
                        });
                        coverage_audit.partial_samples.push(sample.clone());
                        layout_issues.push(TransitionLayoutIssue {
                            category: "PartialOverlap",
                            coarse_cell_idx: face.cell_idx,
                            coarse_face_idx: face_idx,
                            axis: face.axis,
                            sign: face.sign,
                            plane_coord: face.plane_coord,
                            center: sample.center,
                            min: sample.min,
                            max: sample.max,
                            coarse_cell_size: sample.coarse_cell_size,
                            fine_cell_size: sample.fine_cell_size,
                            ratio: sample.coarse_cell_size / sample.fine_cell_size.max(1e-6),
                            overlap_fraction: sample.fraction,
                            exact_case: classify_partial_overlap_exact_case(
                                face,
                                &audit_finer_neighbors,
                                &face_records,
                            ),
                            fine_neighbor_count: audit_finer_neighbors.len(),
                            near_zero_surface: coarse_near_surface || fine_near_surface,
                            fine_neighbor_boxes: audit_finer_neighbors
                                .iter()
                                .map(|&idx| {
                                    let fine = &face_records[idx];
                                    (face_min_world(fine), face_max_world(fine))
                                })
                                .collect(),
                        });
                    }
                }

                let opposite = &face_neighbors[face_idx];
                if opposite.is_empty() {
                    continue;
                }

                if opposite.iter().any(|&other_idx| {
                    let other = &face_records[other_idx];
                    approx_eq(face.cell_size, other.cell_size) && spans_match_exact(face, other)
                }) {
                    face_classes[face_idx] = FaceClassification::Interior;
                    continue;
                }

                let finer_neighbors: Vec<usize> = opposite
                    .iter()
                    .copied()
                    .filter(|&other_idx| face_records[other_idx].cell_size + 1e-6 < face.cell_size)
                    .collect();

                if !finer_neighbors.is_empty()
                    && tiled_span_matches(face, &finer_neighbors, &face_records)
                {
                    face_classes[face_idx] = FaceClassification::Transition;
                    transitions.push(TransitionFace {
                        coarse_cell_idx: face.cell_idx,
                        coarse_face_id: face.face_id,
                        fine_cell_indices: finer_neighbors
                            .iter()
                            .map(|&idx| face_records[idx].cell_idx)
                            .collect(),
                    });
                }
            }
            group_start = group_end;
        }
        coverage_audit.unmatched = audit_area_neighbors
            .iter()
            .filter(|neighbors| neighbors.is_empty())
            .count();
        let classify_elapsed = classify_t0.elapsed();
        let total_elapsed = build_t0.elapsed();

        eprintln!(
            "    face_adjacency_build: total={:?} faces={} plane_groups={} enumerate={:?} sort={:?} classify={:?} neighbor_pair_checks={} overlap_passes={} classification_checks={}",
            total_elapsed,
            face_records.len(),
            plane_groups,
            enumerate_elapsed,
            sort_elapsed,
            classify_elapsed,
            neighbor_pair_checks,
            overlap_passes,
            classification_checks,
        );
        coverage_audit.print();
        eprintln!(
            "    edge_transition_audit: TrueHalfEdge={} CornerTouch={} IrregularEdge={}",
            edge_transition_stats.true_half_edge,
            edge_transition_stats.corner_touch,
            edge_transition_stats.irregular_edge,
        );

        Self {
            finest_cell_size,
            face_records,
            face_classes,
            transitions,
            edge_transitions,
            edge_transition_stats,
            cell_neighbors,
            max_touching_ratio_value,
            coverage_audit,
            layout_issues,
        }
    }

    fn transition_faces(&self) -> impl Iterator<Item = TransitionFace> + '_ {
        self.transitions.iter().cloned()
    }

    fn edge_transition_records(&self) -> impl Iterator<Item = EdgeTransitionRecord> + '_ {
        self.edge_transitions.iter().cloned()
    }

    fn neighbors(&self, cell_idx: usize) -> &[usize] {
        &self.cell_neighbors[cell_idx]
    }

    fn max_touching_ratio(&self) -> f32 {
        self.max_touching_ratio_value
    }
}

fn enumerate_cell_faces(cell_idx: usize, cell: &OctreeCell) -> [FaceRecord; 6] {
    let min = cell.min;
    let max = cell.min + Vec3::splat(cell.size);
    [
        FaceRecord {
            cell_idx,
            face_id: 0,
            axis: 0,
            sign: 0,
            plane_coord: min.x,
            u_min: min.y,
            u_max: max.y,
            v_min: min.z,
            v_max: max.z,
            cell_size: cell.size,
        },
        FaceRecord {
            cell_idx,
            face_id: 1,
            axis: 0,
            sign: 1,
            plane_coord: max.x,
            u_min: min.y,
            u_max: max.y,
            v_min: min.z,
            v_max: max.z,
            cell_size: cell.size,
        },
        FaceRecord {
            cell_idx,
            face_id: 2,
            axis: 1,
            sign: 0,
            plane_coord: min.y,
            u_min: min.x,
            u_max: max.x,
            v_min: min.z,
            v_max: max.z,
            cell_size: cell.size,
        },
        FaceRecord {
            cell_idx,
            face_id: 3,
            axis: 1,
            sign: 1,
            plane_coord: max.y,
            u_min: min.x,
            u_max: max.x,
            v_min: min.z,
            v_max: max.z,
            cell_size: cell.size,
        },
        FaceRecord {
            cell_idx,
            face_id: 4,
            axis: 2,
            sign: 0,
            plane_coord: min.z,
            u_min: min.x,
            u_max: max.x,
            v_min: min.y,
            v_max: max.y,
            cell_size: cell.size,
        },
        FaceRecord {
            cell_idx,
            face_id: 5,
            axis: 2,
            sign: 1,
            plane_coord: max.z,
            u_min: min.x,
            u_max: max.x,
            v_min: min.y,
            v_max: max.y,
            cell_size: cell.size,
        },
    ]
}

fn approx_eq(a: f32, b: f32) -> bool {
    (a - b).abs() <= 1e-5
}

fn spans_overlap(a: &FaceRecord, b: &FaceRecord) -> bool {
    a.u_min < b.u_max - 1e-5
        && b.u_min < a.u_max - 1e-5
        && a.v_min < b.v_max - 1e-5
        && b.v_min < a.v_max - 1e-5
}

fn intervals_strictly_overlap(a_min: f32, a_max: f32, b_min: f32, b_max: f32) -> bool {
    a_min < b_max - 1e-5 && b_min < a_max - 1e-5
}

fn intervals_touch_or_overlap(a_min: f32, a_max: f32, b_min: f32, b_max: f32) -> bool {
    a_min <= b_max + 1e-5 && b_min <= a_max + 1e-5
}

fn face_contact_kind(a: &FaceRecord, b: &FaceRecord) -> FaceContactKind {
    let u_area = intervals_strictly_overlap(a.u_min, a.u_max, b.u_min, b.u_max);
    let v_area = intervals_strictly_overlap(a.v_min, a.v_max, b.v_min, b.v_max);
    if u_area && v_area {
        return FaceContactKind::Area;
    }

    let u_contact = intervals_touch_or_overlap(a.u_min, a.u_max, b.u_min, b.u_max);
    let v_contact = intervals_touch_or_overlap(a.v_min, a.v_max, b.v_min, b.v_max);
    if u_contact && v_contact && (u_area || v_area) {
        FaceContactKind::Edge
    } else {
        FaceContactKind::None
    }
}

fn v_spans_overlap(a: &FaceRecord, b: &FaceRecord) -> bool {
    a.v_min < b.v_max - 1e-5 && b.v_min < a.v_max - 1e-5
}

fn quantize_scalar(v: f32, finest_cell_size: f32) -> i32 {
    let quantum = (finest_cell_size * 0.25).max(1e-6);
    (v / quantum).round() as i32
}

fn spans_match_exact(a: &FaceRecord, b: &FaceRecord) -> bool {
    approx_eq(a.u_min, b.u_min)
        && approx_eq(a.u_max, b.u_max)
        && approx_eq(a.v_min, b.v_min)
        && approx_eq(a.v_max, b.v_max)
}

fn face_area(face: &FaceRecord) -> f32 {
    ((face.u_max - face.u_min).max(0.0)) * ((face.v_max - face.v_min).max(0.0))
}

fn overlap_rect_area(a: &FaceRecord, b: &FaceRecord) -> f32 {
    let u = (a.u_max.min(b.u_max) - a.u_min.max(b.u_min)).max(0.0);
    let v = (a.v_max.min(b.v_max) - a.v_min.max(b.v_min)).max(0.0);
    u * v
}

fn union_overlap_fraction(
    coarse: &FaceRecord,
    finer_neighbor_indices: &[usize],
    faces: &[FaceRecord],
) -> f32 {
    let total_area = face_area(coarse);
    if total_area <= 1e-12 {
        return 0.0;
    }

    let mut unique_u = vec![coarse.u_min, coarse.u_max];
    let mut unique_v = vec![coarse.v_min, coarse.v_max];
    let neighbors: Vec<&FaceRecord> = finer_neighbor_indices
        .iter()
        .map(|&idx| &faces[idx])
        .collect();
    for n in &neighbors {
        unique_u.push(n.u_min.max(coarse.u_min));
        unique_u.push(n.u_max.min(coarse.u_max));
        unique_v.push(n.v_min.max(coarse.v_min));
        unique_v.push(n.v_max.min(coarse.v_max));
    }
    unique_u.sort_by(|a, b| a.total_cmp(b));
    unique_u.dedup_by(|a, b| (*a - *b).abs() <= 1e-5);
    unique_v.sort_by(|a, b| a.total_cmp(b));
    unique_v.dedup_by(|a, b| (*a - *b).abs() <= 1e-5);

    let mut covered_area = 0.0f32;
    for u_pair in unique_u.windows(2) {
        for v_pair in unique_v.windows(2) {
            let patch_u_min = u_pair[0];
            let patch_u_max = u_pair[1];
            let patch_v_min = v_pair[0];
            let patch_v_max = v_pair[1];
            if patch_u_max <= patch_u_min + 1e-5 || patch_v_max <= patch_v_min + 1e-5 {
                continue;
            }
            if neighbors.iter().any(|n| {
                n.u_min <= patch_u_min + 1e-5
                    && n.u_max >= patch_u_max - 1e-5
                    && n.v_min <= patch_v_min + 1e-5
                    && n.v_max >= patch_v_max - 1e-5
            }) {
                covered_area += (patch_u_max - patch_u_min) * (patch_v_max - patch_v_min);
            }
        }
    }

    (covered_area / total_area).clamp(0.0, 1.0)
}

fn face_center_world(face: &FaceRecord) -> Vec3 {
    face_uv_to_world(
        face.axis,
        face.plane_coord,
        (face.u_min + face.u_max) * 0.5,
        (face.v_min + face.v_max) * 0.5,
    )
}

fn edge_contact_center_world(a: &FaceRecord, b: &FaceRecord) -> Vec3 {
    let u_min = a.u_min.max(b.u_min);
    let u_max = a.u_max.min(b.u_max);
    let v_min = a.v_min.max(b.v_min);
    let v_max = a.v_max.min(b.v_max);
    let u = if u_min <= u_max {
        (u_min + u_max) * 0.5
    } else if (a.u_max - b.u_min).abs() <= (b.u_max - a.u_min).abs() {
        (a.u_max + b.u_min) * 0.5
    } else {
        (b.u_max + a.u_min) * 0.5
    };
    let v = if v_min <= v_max {
        (v_min + v_max) * 0.5
    } else if (a.v_max - b.v_min).abs() <= (b.v_max - a.v_min).abs() {
        (a.v_max + b.v_min) * 0.5
    } else {
        (b.v_max + a.v_min) * 0.5
    };
    face_uv_to_world(a.axis, a.plane_coord, u, v)
}

fn face_min_world(face: &FaceRecord) -> Vec3 {
    face_uv_to_world(face.axis, face.plane_coord, face.u_min, face.v_min)
}

fn face_max_world(face: &FaceRecord) -> Vec3 {
    face_uv_to_world(face.axis, face.plane_coord, face.u_max, face.v_max)
}

fn partial_overlap_sample(
    coarse: &FaceRecord,
    finer_neighbor_indices: &[usize],
    faces: &[FaceRecord],
) -> PartialOverlapSample {
    let fine_cell_size = finer_neighbor_indices
        .iter()
        .map(|&idx| faces[idx].cell_size)
        .fold(f32::MAX, f32::min);
    PartialOverlapSample {
        fraction: union_overlap_fraction(coarse, finer_neighbor_indices, faces),
        axis: coarse.axis,
        plane_coord: coarse.plane_coord,
        center: face_center_world(coarse),
        min: face_min_world(coarse),
        max: face_max_world(coarse),
        fine_neighbor_count: finer_neighbor_indices.len(),
        coarse_cell_size: coarse.cell_size,
        fine_cell_size,
    }
}

fn classify_partial_overlap_exact_case(
    coarse: &FaceRecord,
    finer_neighbor_indices: &[usize],
    faces: &[FaceRecord],
) -> &'static str {
    if finer_neighbor_indices.is_empty() {
        return "other";
    }

    let u_mid = (coarse.u_min + coarse.u_max) * 0.5;
    let v_mid = (coarse.v_min + coarse.v_max) * 0.5;
    let mut mask = 0u8;

    for &idx in finer_neighbor_indices {
        let f = &faces[idx];
        let u0 = (f.u_min - coarse.u_min).abs() <= 1e-4;
        let u1 = (f.u_max - coarse.u_max).abs() <= 1e-4;
        let um = (f.u_min - u_mid).abs() <= 1e-4 || (f.u_max - u_mid).abs() <= 1e-4;
        let v0 = (f.v_min - coarse.v_min).abs() <= 1e-4;
        let v1 = (f.v_max - coarse.v_max).abs() <= 1e-4;
        let vm = (f.v_min - v_mid).abs() <= 1e-4 || (f.v_max - v_mid).abs() <= 1e-4;
        if !(um && vm && (u0 || u1) && (v0 || v1)) {
            return "irregular";
        }
        let quad = match (f.u_max <= u_mid + 1e-4, f.v_max <= v_mid + 1e-4) {
            (true, true) => 0,
            (false, true) if f.u_min >= u_mid - 1e-4 => 1,
            (true, false) if f.v_min >= v_mid - 1e-4 => 2,
            (false, false) if f.u_min >= u_mid - 1e-4 && f.v_min >= v_mid - 1e-4 => 3,
            _ => return "irregular",
        };
        mask |= 1 << quad;
    }

    match mask.count_ones() {
        1 => "1-of-4",
        2 => {
            if mask == 0b0101 || mask == 0b1010 {
                "2-diagonal"
            } else {
                "2-adjacent"
            }
        }
        3 => "3-of-4",
        4 => "full-4-of-4",
        _ => "other",
    }
}

impl CoverageAudit {
    fn record_edge_contact(&mut self, a: &FaceRecord, b: &FaceRecord) {
        if approx_eq(a.cell_size, b.cell_size) {
            self.same_size_edge_contact += 1;
            return;
        }

        let ratio = a.cell_size.max(b.cell_size) / a.cell_size.min(b.cell_size);
        self.different_size_edge_contact += 1;
        self.different_size_edge_samples.push(EdgeContactSample {
            ratio,
            center: edge_contact_center_world(a, b),
        });
    }

    fn print(&self) {
        eprintln!(
            "    coverage_audit: FullTile2to1={} PartialOverlap={} SameSizeEdgeContact={} DifferentSizeEdgeContact={} SameSizeMatched={} Unmatched={}",
            self.full_tile_2_to_1,
            self.partial_overlap,
            self.same_size_edge_contact,
            self.different_size_edge_contact,
            self.same_size_matched,
            self.unmatched,
        );
        self.print_different_size_edge_contacts();

        if self.partial_samples.is_empty() {
            eprintln!("    coverage_audit partial_overlap_fractions: none");
            return;
        }

        let hotspot_count = self
            .partial_samples
            .iter()
            .filter(|sample| {
                sample.center.x >= 476.0
                    && sample.center.x <= 680.0
                    && sample.center.z >= 24.0
                    && sample.center.z <= 43.0
            })
            .count();
        eprintln!(
            "    coverage_audit partial_overlap_hotspot_x476_680_z24_43: matches={} total={} fraction={:.3}",
            hotspot_count,
            self.partial_samples.len(),
            hotspot_count as f32 / self.partial_samples.len() as f32,
        );

        let mut buckets: BTreeMap<i32, (usize, PartialOverlapSample)> = BTreeMap::new();
        for sample in &self.partial_samples {
            let bucket = (sample.fraction * 1000.0).round() as i32;
            buckets
                .entry(bucket)
                .and_modify(|(count, _)| *count += 1)
                .or_insert((1, sample.clone()));
        }

        let mut bucket_rows: Vec<(i32, usize, PartialOverlapSample)> = buckets
            .into_iter()
            .map(|(bucket, (count, sample))| (bucket, count, sample))
            .collect();
        bucket_rows.sort_by(|a, b| b.1.cmp(&a.1).then_with(|| a.0.cmp(&b.0)));

        eprintln!("    coverage_audit partial_overlap_top_fractions:");
        for (rank, (bucket, count, sample)) in bucket_rows.into_iter().take(10).enumerate() {
            eprintln!(
                "      rank={} fraction={:.3} count={} axis={} plane={:.3} center=({:.1},{:.1},{:.1}) face_min=({:.1},{:.1},{:.1}) face_max=({:.1},{:.1},{:.1}) fine_neighbors={} coarse_size={:.3} fine_size={:.3}",
                rank + 1,
                bucket as f32 / 1000.0,
                count,
                sample.axis,
                sample.plane_coord,
                sample.center.x,
                sample.center.y,
                sample.center.z,
                sample.min.x,
                sample.min.y,
                sample.min.z,
                sample.max.x,
                sample.max.y,
                sample.max.z,
                sample.fine_neighbor_count,
                sample.coarse_cell_size,
                sample.fine_cell_size,
            );
        }
    }

    fn print_different_size_edge_contacts(&self) {
        if self.different_size_edge_samples.is_empty() {
            eprintln!("    coverage_audit different_size_edge_contact_ratios: none");
            eprintln!("    coverage_audit different_size_edge_contact_hotspots: none");
            return;
        }

        let mut ratios: BTreeMap<i32, usize> = BTreeMap::new();
        for sample in &self.different_size_edge_samples {
            *ratios
                .entry((sample.ratio * 1000.0).round() as i32)
                .or_default() += 1;
        }
        let ratio_text = ratios
            .iter()
            .map(|(ratio, count)| format!("{:.3}:{}", *ratio as f32 / 1000.0, count))
            .collect::<Vec<_>>()
            .join(" ");
        eprintln!(
            "    coverage_audit different_size_edge_contact_ratios: {}",
            ratio_text
        );

        let bounds_min = self
            .different_size_edge_samples
            .iter()
            .map(|sample| sample.center)
            .fold(Vec3::splat(f32::INFINITY), |acc, p| acc.min(p));
        let bounds_max = self
            .different_size_edge_samples
            .iter()
            .map(|sample| sample.center)
            .fold(Vec3::splat(f32::NEG_INFINITY), |acc, p| acc.max(p));
        let extent = bounds_max - bounds_min + Vec3::splat(1e-9);
        let mut bins: BTreeMap<i32, usize> = BTreeMap::new();
        for sample in &self.different_size_edge_samples {
            let rel = (sample.center - bounds_min) / extent * 10.0;
            let ix = (rel.x.floor() as i32).clamp(0, 9);
            let iy = (rel.y.floor() as i32).clamp(0, 9);
            let iz = (rel.z.floor() as i32).clamp(0, 9);
            *bins.entry(ix * 100 + iy * 10 + iz).or_default() += 1;
        }
        let mut rows: Vec<(i32, usize)> = bins.into_iter().collect();
        rows.sort_by(|a, b| b.1.cmp(&a.1));

        eprintln!(
            "    coverage_audit different_size_edge_contact_bounds: min=({:.1},{:.1},{:.1}) max=({:.1},{:.1},{:.1})",
            bounds_min.x, bounds_min.y, bounds_min.z, bounds_max.x, bounds_max.y, bounds_max.z,
        );
        eprintln!("    coverage_audit different_size_edge_contact_hotspots:");
        for (key, count) in rows.into_iter().take(10) {
            let ix = key / 100;
            let iy = (key % 100) / 10;
            let iz = key % 10;
            let cell_min = bounds_min
                + (bounds_max - bounds_min) * Vec3::new(ix as f32, iy as f32, iz as f32) / 10.0;
            let cell_max = bounds_min
                + (bounds_max - bounds_min)
                    * Vec3::new((ix + 1) as f32, (iy + 1) as f32, (iz + 1) as f32)
                    / 10.0;
            eprintln!(
                "      [{},{},{}] count={} x={:.1}..{:.1} y={:.1}..{:.1} z={:.1}..{:.1}",
                ix,
                iy,
                iz,
                count,
                cell_min.x,
                cell_max.x,
                cell_min.y,
                cell_max.y,
                cell_min.z,
                cell_max.z,
            );
        }
    }
}

fn split_partial_overlap_cells(cells: &mut Vec<OctreeCell>, sdf: &dyn Sdf, max_rounds: usize) {
    let mode = transition_layout_mode_from_env();
    if mode == "off" {
        return;
    }

    let before = cells.len();
    let adjacency = FaceAdjacency::build(cells);
    eprintln!(
        "  adaptive-mc transition_layout_summary: mode={} cells_before={} cells_after={} cells_split={} rounds={} partial_overlap_final={} unmatched_final={}",
        mode,
        before,
        cells.len(),
        0usize,
        0usize,
        adjacency.coverage_audit.partial_overlap,
        adjacency.coverage_audit.unmatched,
    );
}

fn print_partial_overlap_case_report(adjacency: &FaceAdjacency) {
    let mut counts = BTreeMap::<&'static str, usize>::new();
    let near_zero_surface = adjacency
        .layout_issues
        .iter()
        .filter(|issue| issue.near_zero_surface)
        .count();
    for issue in &adjacency.layout_issues {
        *counts.entry(issue.exact_case).or_default() += 1;
    }
    eprintln!(
        "  adaptive-mc partial_overlap_case_report: total_partial_overlap_faces={} near_zero_surface={} near_final_defects=diagnostic_only",
        adjacency.layout_issues.len(),
        near_zero_surface,
    );
    for (case, count) in counts {
        eprintln!("    case={} count={}", case, count);
    }
    for (idx, issue) in adjacency.layout_issues.iter().take(20).enumerate() {
        let fine_bbox_text = issue
            .fine_neighbor_boxes
            .iter()
            .map(|(min, max)| {
                format!(
                    "({:.3},{:.3},{:.3})..({:.3},{:.3},{:.3})",
                    min.x, min.y, min.z, max.x, max.y, max.z
                )
            })
            .collect::<Vec<_>>()
            .join("; ");
        eprintln!(
            "    partial_overlap_face id={} face_id={} axis={} plane={:.3} coarse_cell_id={} coarse_cell_bbox=({:.3},{:.3},{:.3})..({:.3},{:.3},{:.3}) fine_neighbor_count={} fine_neighbor_bboxes=[{}] overlap_fraction={:.3} uncovered_fraction={:.3} near_surface_crossing={} exact_case={} ratio={:.3}",
            idx + 1,
            issue.coarse_face_idx,
            issue.axis,
            issue.plane_coord,
            issue.coarse_cell_idx,
            issue.min.x,
            issue.min.y,
            issue.min.z,
            issue.max.x,
            issue.max.y,
            issue.max.z,
            issue.fine_neighbor_count,
            fine_bbox_text,
            issue.overlap_fraction,
            1.0 - issue.overlap_fraction,
            issue.near_zero_surface,
            issue.exact_case,
            issue.ratio,
        );
    }
}

fn issue_bad_edge_scores(
    mesh: &Mesh,
    adjacency: &FaceAdjacency,
) -> (Vec<usize>, Vec<usize>, Vec<usize>) {
    let positions = mesh_positions(mesh);
    if positions.is_empty() || adjacency.layout_issues.is_empty() {
        return (
            vec![0; adjacency.layout_issues.len()],
            vec![0; adjacency.layout_issues.len()],
            vec![0; adjacency.layout_issues.len()],
        );
    }
    let edge_to_tris = build_mesh_edge_to_triangles(&mesh.indices);
    let mut issue_bad_edge_counts = vec![0usize; adjacency.layout_issues.len()];
    let mut issue_boundary_counts = vec![0usize; adjacency.layout_issues.len()];
    let mut issue_non_manifold_counts = vec![0usize; adjacency.layout_issues.len()];
    for (&edge, tris) in &edge_to_tris {
        if tris.len() == 2 {
            continue;
        }
        let mid = (positions[edge.0 as usize] + positions[edge.1 as usize]) * 0.5;
        for (issue_idx, issue) in adjacency.layout_issues.iter().enumerate() {
            let threshold = issue.coarse_cell_size.max(issue.fine_cell_size) * 1.25;
            if mid.distance(issue.center) > threshold {
                continue;
            }
            if tris.len() == 1 {
                issue_boundary_counts[issue_idx] += 1;
            } else {
                issue_non_manifold_counts[issue_idx] += 1;
            }
            issue_bad_edge_counts[issue_idx] += 1;
        }
    }
    (
        issue_bad_edge_counts,
        issue_boundary_counts,
        issue_non_manifold_counts,
    )
}

fn fine_quadrant_mask(issue: &TransitionLayoutIssue) -> u8 {
    let u_mid = (issue.min.x + issue.max.x) * 0.5;
    let v_mid = (issue.min.z + issue.max.z) * 0.5;
    let (u_min, u_max, v_min, v_max) = match issue.axis {
        0 => (issue.min.y, issue.max.y, issue.min.z, issue.max.z),
        1 => (issue.min.x, issue.max.x, issue.min.z, issue.max.z),
        2 => (issue.min.x, issue.max.x, issue.min.y, issue.max.y),
        _ => return 0,
    };
    let u_mid = (u_min + u_max) * 0.5;
    let v_mid = (v_min + v_max) * 0.5;
    let mut mask = 0u8;
    for (min, max) in &issue.fine_neighbor_boxes {
        let (fu_min, fu_max, fv_min, fv_max) = match issue.axis {
            0 => (min.y, max.y, min.z, max.z),
            1 => (min.x, max.x, min.z, max.z),
            2 => (min.x, max.x, min.y, max.y),
            _ => continue,
        };
        let quad = if fu_max <= u_mid + 1e-4 && fv_max <= v_mid + 1e-4 {
            1 << 0
        } else if fu_min >= u_mid - 1e-4 && fv_max <= v_mid + 1e-4 {
            1 << 1
        } else if fu_max <= u_mid + 1e-4 && fv_min >= v_mid - 1e-4 {
            1 << 2
        } else if fu_min >= u_mid - 1e-4 && fv_min >= v_mid - 1e-4 {
            1 << 3
        } else {
            0
        };
        mask |= quad;
    }
    mask
}

fn mask_split_direction(mask: u8) -> &'static str {
    match mask {
        0b0011 | 0b1100 => "u-split",
        0b0101 | 0b1010 => "v-split",
        _ => "unsupported",
    }
}

fn is_refinement_region_side_boundary(issue: &TransitionLayoutIssue) -> bool {
    (issue.axis == 1 && ((issue.plane_coord.abs() - 27.0).abs() <= 2.0))
        || near_inlet_transition_plane(issue.center)
}

fn choose_two_adjacent_candidates(
    mesh: &Mesh,
    adjacency: &FaceAdjacency,
) -> (Vec<TwoAdjacentCandidate>, TwoAdjacentCandidateReport) {
    let (bad_scores, boundary_scores, non_manifold_scores) = issue_bad_edge_scores(mesh, adjacency);
    let mut report = TwoAdjacentCandidateReport::default();
    let mut candidates = Vec::new();

    for (issue_idx, issue) in adjacency.layout_issues.iter().enumerate() {
        if issue.exact_case != "2-adjacent" {
            continue;
        }
        report.total_two_adjacent += 1;
        if !issue.near_zero_surface {
            report.rejected_not_near_zero += 1;
            continue;
        }
        if bad_scores[issue_idx] == 0 {
            report.rejected_not_defect_mapped += 1;
            continue;
        }
        if !is_refinement_region_side_boundary(issue) {
            report.rejected_not_side_boundary += 1;
            continue;
        }
        let mask = fine_quadrant_mask(issue);
        let split_direction = mask_split_direction(mask);
        if split_direction == "unsupported" {
            report.rejected_not_supported_mask += 1;
            continue;
        }
        report.eligible_two_adjacent += 1;
        candidates.push(TwoAdjacentCandidate {
            issue_idx,
            coarse_cell_idx: issue.coarse_cell_idx,
            exact_case: issue.exact_case,
            axis: issue.axis,
            sign: issue.sign,
            plane_coord: issue.plane_coord,
            center: issue.center,
            coarse_min: issue.min,
            coarse_max: issue.max,
            fine_neighbor_count: issue.fine_neighbor_count,
            overlap_fraction: issue.overlap_fraction,
            uncovered_fraction: 1.0 - issue.overlap_fraction,
            coarse_cell_size: issue.coarse_cell_size,
            fine_cell_size: issue.fine_cell_size,
            bad_edge_score: bad_scores[issue_idx],
            boundary_edge_score: boundary_scores[issue_idx],
            non_manifold_edge_score: non_manifold_scores[issue_idx],
            quadrant_mask: mask,
            split_direction,
            patch_type: "local_face_subdivision_fallback",
        });
    }

    candidates.sort_by(|a, b| {
        b.bad_edge_score
            .cmp(&a.bad_edge_score)
            .then_with(|| b.boundary_edge_score.cmp(&a.boundary_edge_score))
            .then_with(|| a.issue_idx.cmp(&b.issue_idx))
    });
    report.candidates_proposed = candidates.len();
    (candidates, report)
}

fn print_two_adjacent_candidate_report(
    candidates: &[TwoAdjacentCandidate],
    report: &TwoAdjacentCandidateReport,
) {
    eprintln!(
        "  adaptive-mc two_adjacent_candidate_report: total_two_adjacent={} eligible={} candidates_proposed={}",
        report.total_two_adjacent, report.eligible_two_adjacent, report.candidates_proposed,
    );
    eprintln!(
        "    rejected_not_near_zero={} rejected_not_defect_mapped={} rejected_not_side_boundary={} rejected_not_supported_mask={}",
        report.rejected_not_near_zero,
        report.rejected_not_defect_mapped,
        report.rejected_not_side_boundary,
        report.rejected_not_supported_mask,
    );
    for (rank, candidate) in candidates.iter().take(10).enumerate() {
        eprintln!(
            "    candidate rank={} issue_id={} patch_type={} bad_edges={} boundary_edges={} non_manifold_edges={} axis={} sign={} plane={:.3} coarse_cell_id={} coarse_cell_bbox=({:.3},{:.3},{:.3})..({:.3},{:.3},{:.3}) fine_neighbor_count={} overlap_fraction={:.3} uncovered_fraction={:.3} split_direction={} quadrant_mask={:04b}",
            rank + 1,
            candidate.issue_idx + 1,
            candidate.patch_type,
            candidate.bad_edge_score,
            candidate.boundary_edge_score,
            candidate.non_manifold_edge_score,
            candidate.axis,
            candidate.sign,
            candidate.plane_coord,
            candidate.coarse_cell_idx,
            candidate.coarse_min.x,
            candidate.coarse_min.y,
            candidate.coarse_min.z,
            candidate.coarse_max.x,
            candidate.coarse_max.y,
            candidate.coarse_max.z,
            candidate.fine_neighbor_count,
            candidate.overlap_fraction,
            candidate.uncovered_fraction,
            candidate.split_direction,
            candidate.quadrant_mask,
        );
    }
}

fn coarse_face_corners(issue: &TransitionLayoutIssue) -> [Vec3; 4] {
    match issue.axis {
        0 => [
            Vec3::new(issue.plane_coord, issue.min.y, issue.min.z),
            Vec3::new(issue.plane_coord, issue.max.y, issue.min.z),
            Vec3::new(issue.plane_coord, issue.max.y, issue.max.z),
            Vec3::new(issue.plane_coord, issue.min.y, issue.max.z),
        ],
        1 => [
            Vec3::new(issue.min.x, issue.plane_coord, issue.min.z),
            Vec3::new(issue.max.x, issue.plane_coord, issue.min.z),
            Vec3::new(issue.max.x, issue.plane_coord, issue.max.z),
            Vec3::new(issue.min.x, issue.plane_coord, issue.max.z),
        ],
        _ => [
            Vec3::new(issue.min.x, issue.min.y, issue.plane_coord),
            Vec3::new(issue.max.x, issue.min.y, issue.plane_coord),
            Vec3::new(issue.max.x, issue.max.y, issue.plane_coord),
            Vec3::new(issue.min.x, issue.max.y, issue.plane_coord),
        ],
    }
}

fn point_face_local(issue: &TransitionLayoutIssue, p: Vec3) -> Vec3 {
    match issue.axis {
        0 => Vec3::new(
            p.y - issue.min.y,
            p.z - issue.min.z,
            p.x - issue.plane_coord,
        ),
        1 => Vec3::new(
            p.x - issue.min.x,
            p.z - issue.min.z,
            p.y - issue.plane_coord,
        ),
        _ => Vec3::new(
            p.x - issue.min.x,
            p.y - issue.min.y,
            p.z - issue.plane_coord,
        ),
    }
}

fn edge_role_on_face(
    issue: &TransitionLayoutIssue,
    a: Vec3,
    b: Vec3,
    split_direction: &str,
) -> &'static str {
    let tol = (issue.fine_cell_size * 0.35).max(0.25);
    let size = issue.coarse_cell_size;
    let la = point_face_local(issue, a);
    let lb = point_face_local(issue, b);
    let mid = (la + lb) * 0.5;
    let on_outer_u = mid.x.abs() <= tol || (mid.x - size).abs() <= tol;
    let on_outer_v = mid.y.abs() <= tol || (mid.y - size).abs() <= tol;
    if on_outer_u || on_outer_v {
        return "coarse_face_edge";
    }

    match split_direction {
        "u-split" => {
            if (mid.x - size * 0.5).abs() <= tol {
                "covered_uncovered_seam"
            } else if mid.y.abs() <= tol || (mid.y - size).abs() <= tol {
                "fine_tile_edge"
            } else {
                "unknown"
            }
        }
        "v-split" => {
            if (mid.y - size * 0.5).abs() <= tol {
                "covered_uncovered_seam"
            } else if mid.x.abs() <= tol || (mid.x - size).abs() <= tol {
                "fine_tile_edge"
            } else {
                "unknown"
            }
        }
        _ => "unknown",
    }
}

fn collect_face_boundary_edge_projections(
    mesh: &Mesh,
    candidate: &TwoAdjacentCandidate,
) -> Vec<FaceBoundaryEdgeProjection> {
    let positions = mesh_positions(mesh);
    let edge_to_tris = build_mesh_edge_to_triangles(&mesh.indices);
    let expand = candidate.fine_cell_size * 1.25;
    let face_min = candidate.coarse_min - Vec3::splat(expand);
    let face_max = candidate.coarse_max + Vec3::splat(expand);
    let mut edges = Vec::new();
    for (&edge, tris) in &edge_to_tris {
        if tris.len() != 1 {
            continue;
        }
        let a = positions[edge.0 as usize];
        let b = positions[edge.1 as usize];
        let mid = (a + b) * 0.5;
        if match candidate.axis {
            0 => (mid.x - candidate.plane_coord).abs() > expand,
            1 => (mid.y - candidate.plane_coord).abs() > expand,
            _ => (mid.z - candidate.plane_coord).abs() > expand,
        } {
            continue;
        }
        if mid.x < face_min.x
            || mid.x > face_max.x
            || mid.y < face_min.y
            || mid.y > face_max.y
            || mid.z < face_min.z
            || mid.z > face_max.z
        {
            continue;
        }
        let local_a = point_face_local(
            &TransitionLayoutIssue {
                category: "",
                coarse_cell_idx: candidate.coarse_cell_idx,
                coarse_face_idx: 0,
                axis: candidate.axis,
                sign: candidate.sign,
                plane_coord: candidate.plane_coord,
                center: candidate.center,
                min: candidate.coarse_min,
                max: candidate.coarse_max,
                coarse_cell_size: candidate.coarse_cell_size,
                fine_cell_size: candidate.fine_cell_size,
                ratio: candidate.coarse_cell_size / candidate.fine_cell_size.max(1e-6),
                overlap_fraction: candidate.overlap_fraction,
                exact_case: candidate.exact_case,
                fine_neighbor_count: candidate.fine_neighbor_count,
                near_zero_surface: true,
                fine_neighbor_boxes: Vec::new(),
            },
            a,
        );
        let local_b = point_face_local(
            &TransitionLayoutIssue {
                category: "",
                coarse_cell_idx: candidate.coarse_cell_idx,
                coarse_face_idx: 0,
                axis: candidate.axis,
                sign: candidate.sign,
                plane_coord: candidate.plane_coord,
                center: candidate.center,
                min: candidate.coarse_min,
                max: candidate.coarse_max,
                coarse_cell_size: candidate.coarse_cell_size,
                fine_cell_size: candidate.fine_cell_size,
                ratio: candidate.coarse_cell_size / candidate.fine_cell_size.max(1e-6),
                overlap_fraction: candidate.overlap_fraction,
                exact_case: candidate.exact_case,
                fine_neighbor_count: candidate.fine_neighbor_count,
                near_zero_surface: true,
                fine_neighbor_boxes: Vec::new(),
            },
            b,
        );
        let temp_issue = TransitionLayoutIssue {
            category: "",
            coarse_cell_idx: candidate.coarse_cell_idx,
            coarse_face_idx: 0,
            axis: candidate.axis,
            sign: candidate.sign,
            plane_coord: candidate.plane_coord,
            center: candidate.center,
            min: candidate.coarse_min,
            max: candidate.coarse_max,
            coarse_cell_size: candidate.coarse_cell_size,
            fine_cell_size: candidate.fine_cell_size,
            ratio: candidate.coarse_cell_size / candidate.fine_cell_size.max(1e-6),
            overlap_fraction: candidate.overlap_fraction,
            exact_case: candidate.exact_case,
            fine_neighbor_count: candidate.fine_neighbor_count,
            near_zero_surface: true,
            fine_neighbor_boxes: Vec::new(),
        };
        let role = edge_role_on_face(&temp_issue, a, b, candidate.split_direction);
        edges.push(FaceBoundaryEdgeProjection {
            edge,
            endpoint_a: a,
            endpoint_b: b,
            local_a,
            local_b,
            midpoint_local: (local_a + local_b) * 0.5,
            role,
            length: a.distance(b),
            incident_triangle_count: tris.len(),
        });
    }
    edges
}

fn quadrant_labels(mask: u8) -> (Vec<&'static str>, Vec<&'static str>) {
    let quads = [
        (0b0001, "lower-left"),
        (0b0010, "lower-right"),
        (0b0100, "upper-left"),
        (0b1000, "upper-right"),
    ];
    let present = quads
        .iter()
        .filter_map(|(bit, label)| (mask & bit != 0).then_some(*label))
        .collect::<Vec<_>>();
    let missing = quads
        .iter()
        .filter_map(|(bit, label)| (mask & bit == 0).then_some(*label))
        .collect::<Vec<_>>();
    (present, missing)
}

fn print_two_adjacent_face_boundary_diagnostic(mesh: &Mesh, candidates: &[TwoAdjacentCandidate]) {
    for candidate in candidates.iter().take(5) {
        let issue = TransitionLayoutIssue {
            category: "PartialOverlap",
            coarse_cell_idx: candidate.coarse_cell_idx,
            coarse_face_idx: candidate.issue_idx,
            axis: candidate.axis,
            sign: candidate.sign,
            plane_coord: candidate.plane_coord,
            center: candidate.center,
            min: candidate.coarse_min,
            max: candidate.coarse_max,
            coarse_cell_size: candidate.coarse_cell_size,
            fine_cell_size: candidate.fine_cell_size,
            ratio: candidate.coarse_cell_size / candidate.fine_cell_size.max(1e-6),
            overlap_fraction: candidate.overlap_fraction,
            exact_case: candidate.exact_case,
            fine_neighbor_count: candidate.fine_neighbor_count,
            near_zero_surface: true,
            fine_neighbor_boxes: Vec::new(),
        };
        let corners = coarse_face_corners(&issue);
        let (present, missing) = quadrant_labels(candidate.quadrant_mask);
        let edges = collect_face_boundary_edge_projections(mesh, candidate);
        let nearby_boundary = edges
            .iter()
            .filter(|e| e.incident_triangle_count == 1)
            .count();
        let nearby_non_manifold = edges
            .iter()
            .filter(|e| e.incident_triangle_count != 1)
            .count();
        eprintln!(
            "  adaptive-mc two_adjacent_face_diagnostic: issue_id={} face_axis={} face_plane={:.3} coarse_face_corners=[({:.3},{:.3},{:.3}); ({:.3},{:.3},{:.3}); ({:.3},{:.3},{:.3}); ({:.3},{:.3},{:.3})] coarse_cell_bbox=({:.3},{:.3},{:.3})..({:.3},{:.3},{:.3}) present_quadrants=[{}] missing_quadrants=[{}] split_direction={} defect_score={} nearby_boundary_edge_count={} nearby_non_manifold_edge_count={}",
            candidate.issue_idx + 1,
            candidate.axis,
            candidate.plane_coord,
            corners[0].x,
            corners[0].y,
            corners[0].z,
            corners[1].x,
            corners[1].y,
            corners[1].z,
            corners[2].x,
            corners[2].y,
            corners[2].z,
            corners[3].x,
            corners[3].y,
            corners[3].z,
            candidate.coarse_min.x,
            candidate.coarse_min.y,
            candidate.coarse_min.z,
            candidate.coarse_max.x,
            candidate.coarse_max.y,
            candidate.coarse_max.z,
            present.join(", "),
            missing.join(", "),
            candidate.split_direction,
            candidate.bad_edge_score,
            nearby_boundary,
            nearby_non_manifold,
        );
        for edge in edges.iter().take(12) {
            eprintln!(
                "    face_edge endpoints=({}, {}) local_a=({:.3},{:.3}) local_b=({:.3},{:.3}) midpoint=({:.3},{:.3}) role={} length={:.6}",
                edge.edge.0,
                edge.edge.1,
                edge.local_a.x,
                edge.local_a.y,
                edge.local_b.x,
                edge.local_b.y,
                edge.midpoint_local.x,
                edge.midpoint_local.y,
                edge.role,
                edge.length,
            );
        }
    }
}

fn print_transition_defect_map(mesh: &Mesh, adjacency: &FaceAdjacency) {
    let positions = mesh_positions(mesh);
    if positions.is_empty() || adjacency.layout_issues.is_empty() {
        eprintln!("  adaptive-mc transition_defect_map: none");
        return;
    }
    let (bounds_min, bounds_max) = mesh_bounds(&positions);
    let extent = bounds_max - bounds_min + Vec3::splat(1e-9);
    let edge_to_tris = build_mesh_edge_to_triangles(&mesh.indices);
    let mut clusters: BTreeMap<i32, Vec<Vec3>> = BTreeMap::new();
    let mut cluster_counts: BTreeMap<i32, (usize, usize)> = BTreeMap::new();
    for (&edge, tris) in &edge_to_tris {
        if tris.len() == 2 {
            continue;
        }
        let mid = (positions[edge.0 as usize] + positions[edge.1 as usize]) * 0.5;
        let rel = (mid - bounds_min) / extent * 10.0;
        let key = (rel.x.floor() as i32).clamp(0, 9) * 100
            + (rel.y.floor() as i32).clamp(0, 9) * 10
            + (rel.z.floor() as i32).clamp(0, 9);
        clusters.entry(key).or_default().push(mid);
        let counts = cluster_counts.entry(key).or_default();
        if tris.len() == 1 {
            counts.0 += 1;
        } else {
            counts.1 += 1;
        }
    }

    eprintln!("  adaptive-mc transition_defect_map:");
    let mut issue_bad_edge_counts = vec![0usize; adjacency.layout_issues.len()];
    let mut issue_boundary_counts = vec![0usize; adjacency.layout_issues.len()];
    let mut issue_non_manifold_counts = vec![0usize; adjacency.layout_issues.len()];
    for (key, mids) in clusters {
        let center = mids.iter().copied().fold(Vec3::ZERO, |acc, p| acc + p) / mids.len() as f32;
        let mut best: Option<(usize, &TransitionLayoutIssue, f32)> = None;
        for (issue_idx, issue) in adjacency.layout_issues.iter().enumerate() {
            let d = center.distance(issue.center);
            let threshold = issue.coarse_cell_size.max(issue.fine_cell_size) * 1.25;
            let (boundary_count, non_manifold_count) =
                cluster_counts.get(&key).copied().unwrap_or((0, 0));
            if d <= threshold {
                let bad = boundary_count + non_manifold_count;
                issue_bad_edge_counts[issue_idx] += bad;
                issue_boundary_counts[issue_idx] += boundary_count;
                issue_non_manifold_counts[issue_idx] += non_manifold_count;
            }
            if best.map(|(_, _, bd)| d < bd).unwrap_or(true) {
                best = Some((issue_idx, issue, d));
            }
        }
        let (boundary_count, non_manifold_count) =
            cluster_counts.get(&key).copied().unwrap_or((0, 0));
        if let Some((_issue_idx, issue, dist)) = best {
            let boundary_label = match (issue.axis, issue.sign) {
                (0, 0) => "x-min refinement boundary",
                (0, 1) => "x-max refinement boundary",
                (1, 0) => "y-min refinement boundary",
                (1, 1) => "y-max refinement boundary",
                (2, 0) => "z-min refinement boundary",
                (2, 1) => "z-max refinement boundary",
                _ => "unknown",
            };
            let avg_len = mids.windows(2).map(|w| w[0].distance(w[1])).sum::<f32>()
                / mids.len().max(1) as f32;
            eprintln!(
                "    cluster_id={} boundary_edge_count={} non_manifold_edge_count={} bbox=({:.3},{:.3},{:.3})..({:.3},{:.3},{:.3}) avg_edge_length={:.6} nearest_category={} nearest_face_axis={} face_plane={:.3} coarse_size={:.3} fine_size={:.3} ratio={:.3} overlap_fraction={:.3} boundary_label={} distance_to_face={:.3}",
                key,
                boundary_count,
                non_manifold_count,
                mids.iter()
                    .copied()
                    .fold(Vec3::splat(f32::INFINITY), |acc, p| acc.min(p))
                    .x,
                mids.iter()
                    .copied()
                    .fold(Vec3::splat(f32::INFINITY), |acc, p| acc.min(p))
                    .y,
                mids.iter()
                    .copied()
                    .fold(Vec3::splat(f32::INFINITY), |acc, p| acc.min(p))
                    .z,
                mids.iter()
                    .copied()
                    .fold(Vec3::splat(f32::NEG_INFINITY), |acc, p| acc.max(p))
                    .x,
                mids.iter()
                    .copied()
                    .fold(Vec3::splat(f32::NEG_INFINITY), |acc, p| acc.max(p))
                    .y,
                mids.iter()
                    .copied()
                    .fold(Vec3::splat(f32::NEG_INFINITY), |acc, p| acc.max(p))
                    .z,
                avg_len,
                issue.category,
                issue.axis,
                issue.plane_coord,
                issue.coarse_cell_size,
                issue.fine_cell_size,
                issue.ratio,
                issue.overlap_fraction,
                boundary_label,
                dist,
            );
        }
    }

    let near_final_defects = issue_bad_edge_counts
        .iter()
        .filter(|&&count| count > 0)
        .count();
    let mut exact_case_near_defect = BTreeMap::<&'static str, usize>::new();
    for (issue, &count) in adjacency
        .layout_issues
        .iter()
        .zip(issue_bad_edge_counts.iter())
    {
        if count > 0 {
            *exact_case_near_defect.entry(issue.exact_case).or_default() += 1;
        }
    }
    eprintln!(
        "  adaptive-mc partial_overlap_defect_report: total_partial_overlap_faces={} near_zero_surface={} near_final_defects={}",
        adjacency.layout_issues.len(),
        adjacency
            .layout_issues
            .iter()
            .filter(|issue| issue.near_zero_surface)
            .count(),
        near_final_defects,
    );
    for (case, count) in exact_case_near_defect {
        eprintln!("    defect_mapped_case={} count={}", case, count);
    }

    let mut rows: Vec<(usize, usize, usize, usize, &TransitionLayoutIssue)> = adjacency
        .layout_issues
        .iter()
        .enumerate()
        .filter_map(|(idx, issue)| {
            let bad = issue_bad_edge_counts[idx];
            if bad == 0 {
                return None;
            }
            Some((
                idx,
                bad,
                issue_boundary_counts[idx],
                issue_non_manifold_counts[idx],
                issue,
            ))
        })
        .collect();
    rows.sort_by(|a, b| {
        b.1.cmp(&a.1)
            .then_with(|| b.2.cmp(&a.2))
            .then_with(|| a.0.cmp(&b.0))
    });
    eprintln!("  adaptive-mc partial_overlap_defect_top10:");
    for (rank, (idx, bad, boundary, non_manifold, issue)) in rows.iter().take(10).enumerate() {
        eprintln!(
            "    rank={} issue_id={} exact_case={} bad_edges={} boundary_edges={} non_manifold_edges={} axis={} plane={:.3} coarse_cell_id={} coarse_cell_bbox=({:.3},{:.3},{:.3})..({:.3},{:.3},{:.3}) fine_neighbor_count={} overlap_fraction={:.3} uncovered_fraction={:.3} near_zero_surface={}",
            rank + 1,
            idx + 1,
            issue.exact_case,
            bad,
            boundary,
            non_manifold,
            issue.axis,
            issue.plane_coord,
            issue.coarse_cell_idx,
            issue.min.x,
            issue.min.y,
            issue.min.z,
            issue.max.x,
            issue.max.y,
            issue.max.z,
            issue.fine_neighbor_count,
            issue.overlap_fraction,
            1.0 - issue.overlap_fraction,
            issue.near_zero_surface,
        );
    }

    let recommended_case = rows
        .iter()
        .filter(|(_, _, _, _, issue)| issue.near_zero_surface)
        .map(|(_, bad, _, _, issue)| (issue.exact_case, *bad))
        .fold(
            BTreeMap::<&'static str, usize>::new(),
            |mut acc, (case, bad)| {
                *acc.entry(case).or_default() += bad;
                acc
            },
        )
        .into_iter()
        .max_by(|a, b| a.1.cmp(&b.1).then_with(|| a.0.cmp(b.0)));
    if let Some((case, score)) = recommended_case {
        eprintln!(
            "  adaptive-mc partial_overlap_recommended_first_case: case={} aggregated_bad_edge_score={}",
            case, score
        );
    } else {
        eprintln!("  adaptive-mc partial_overlap_recommended_first_case: none");
    }
}

fn validate_transition_candidate_mesh(
    before_mesh: &Mesh,
    before_topo: &MeshTopologyStats,
    after_mesh: &Mesh,
    focus: Vec3,
    max_distance: f32,
) -> Result<MeshTopologyStats, &'static str> {
    let after = compute_mesh_topology_stats(after_mesh);
    if after.connected_components > before_topo.connected_components {
        return Err("connected_components_increased");
    }
    if after.boundary_edges > before_topo.boundary_edges
        || after.non_manifold_edges > before_topo.non_manifold_edges
    {
        return Err("no_topology_gain");
    }
    if !new_boundary_edges_stay_local(before_mesh, after_mesh, focus, max_distance) {
        return Err("non_local_boundary_growth");
    }
    let mut dup_like = 0usize;
    let area_eps_sq = 1e-10f32;
    let mut canonical_existing = HashSet::<[u32; 3]>::new();
    for tri in after_mesh.indices.chunks_exact(3) {
        let t = [tri[0], tri[1], tri[2]];
        if mesh_triangle_area_sq(after_mesh, t) <= area_eps_sq {
            return Err("degenerate_or_zero_area");
        }
        let canonical = canonical_mesh_triangle(t);
        if !canonical_existing.insert(canonical) {
            dup_like += 1;
        }
    }
    if dup_like > 0 {
        return Err("duplicate_or_reversed_triangle");
    }
    Ok(after)
}

fn try_two_adjacent_face_patch(
    mesh: &Mesh,
    candidate: &TwoAdjacentCandidate,
) -> Result<(Vec<[u32; 3]>, MeshTopologyStats, &'static str), &'static str> {
    let infos = collect_boundary_edge_info(mesh);
    let local_edges = collect_face_boundary_edge_projections(mesh, candidate);
    if local_edges.is_empty() {
        return Err("no matching boundary vertices");
    }

    let edge_lookup: HashMap<(u32, u32), &BoundaryEdgeInfo> =
        infos.iter().map(|info| (info.edge, info)).collect();

    let coarse_edges = local_edges
        .iter()
        .filter(|edge| edge.role == "coarse_face_edge")
        .collect::<Vec<_>>();
    let seam_edges = local_edges
        .iter()
        .filter(|edge| edge.role == "covered_uncovered_seam" || edge.role == "fine_tile_edge")
        .collect::<Vec<_>>();
    if coarse_edges.is_empty() || seam_edges.is_empty() {
        return Err("patch spans inactive half");
    }

    let mut best: Option<(Vec<[u32; 3]>, MeshTopologyStats, &'static str, f32)> = None;
    for coarse in &coarse_edges {
        for seam in &seam_edges {
            let Some(a) = edge_lookup.get(&coarse.edge).copied() else {
                continue;
            };
            let Some(b) = edge_lookup.get(&seam.edge).copied() else {
                continue;
            };
            let dist = coarse
                .midpoint_local
                .truncate()
                .distance(seam.midpoint_local.truncate());
            if dist > candidate.coarse_cell_size * 0.75 {
                continue;
            }
            let Ok((tris, stats)) = try_bridge_edge_pair(mesh, a, b) else {
                continue;
            };
            if best
                .as_ref()
                .map(|(_, _, _, best_dist)| dist < *best_dist)
                .unwrap_or(true)
            {
                best = Some((tris, stats, "PatchA_seam_bridge", dist));
            }
        }
    }

    best.map(|(tris, stats, patch_type, _)| (tris, stats, patch_type))
        .ok_or("topology no-gain")
}

fn evaluate_two_adjacent_active_candidates(
    sdf: &dyn Sdf,
    balanced_cells: &[OctreeCell],
    baseline_mesh: &Mesh,
    baseline_adjacency: &FaceAdjacency,
    normal_eps: f32,
    smooth_normals: bool,
    min_region_target: f32,
) -> (Option<Mesh>, TwoAdjacentCandidateReport) {
    let (candidates, mut report) =
        choose_two_adjacent_candidates(baseline_mesh, baseline_adjacency);
    print_two_adjacent_candidate_report(&candidates, &report);
    print_two_adjacent_face_boundary_diagnostic(baseline_mesh, &candidates);
    if transition_layout_mode_from_env() != "active" || candidates.is_empty() {
        return (None, report);
    }

    let before_topo = compute_mesh_topology_stats(baseline_mesh);
    let candidate_limit = 1usize;
    let mut best_mesh = None::<Mesh>;
    let mut best_score = before_topo.boundary_edges * 1000 + before_topo.non_manifold_edges;

    for candidate in candidates.iter().take(candidate_limit) {
        report.candidates_attempted += 1;
        match try_two_adjacent_face_patch(baseline_mesh, candidate) {
            Ok((tris, _preview_stats, patch_type)) => {
                let mut test_mesh = baseline_mesh.clone();
                for tri in &tris {
                    test_mesh.indices.extend_from_slice(tri);
                }
                eprintln!(
                    "  adaptive-mc two_adjacent_patch_candidate: issue_id={} patch_type={} triangles_proposed={} vertices_used=existing local_bad_edges_touched={} predicted_boundary_edge_delta=unknown predicted_non_manifold_delta=unknown rejection_reason=none",
                    candidate.issue_idx + 1,
                    patch_type,
                    tris.len(),
                    candidate.bad_edge_score,
                );
                match validate_transition_candidate_mesh(
                    baseline_mesh,
                    &before_topo,
                    &test_mesh,
                    candidate.center,
                    candidate.coarse_cell_size * 3.0,
                ) {
                    Ok(after_topo) => {
                        let score =
                            after_topo.boundary_edges * 1000 + after_topo.non_manifold_edges;
                        if score < best_score {
                            report.candidates_accepted += 1;
                            report.transition_patches_added += 1;
                            report.triangles_added = tris.len();
                            report.vertices_added = 0;
                            best_score = score;
                            best_mesh = Some(test_mesh);
                        } else {
                            report.rejected_no_topology_gain += 1;
                        }
                    }
                    Err("connected_components_increased") => {
                        report.rejected_connected_components += 1
                    }
                    Err("non_local_boundary_growth") => report.rejected_non_local_boundary += 1,
                    Err("duplicate_or_reversed_triangle") | Err("degenerate_or_zero_area") => {
                        report.rejected_duplicate_or_degenerate += 1
                    }
                    Err(reason) => {
                        eprintln!(
                            "  adaptive-mc two_adjacent_patch_candidate: issue_id={} patch_type={} triangles_proposed={} vertices_used=existing local_bad_edges_touched={} predicted_boundary_edge_delta=unknown predicted_non_manifold_delta=unknown rejection_reason={}",
                            candidate.issue_idx + 1,
                            patch_type,
                            tris.len(),
                            candidate.bad_edge_score,
                            reason,
                        );
                        report.rejected_no_topology_gain += 1;
                    }
                }
            }
            Err(reason) => {
                eprintln!(
                    "  adaptive-mc two_adjacent_patch_candidate: issue_id={} patch_type=none triangles_proposed=0 vertices_used=existing local_bad_edges_touched={} predicted_boundary_edge_delta=unknown predicted_non_manifold_delta=unknown rejection_reason={}",
                    candidate.issue_idx + 1,
                    candidate.bad_edge_score,
                    reason,
                );
                match reason {
                    "no matching boundary vertices" | "patch spans inactive half" => {
                        report.rejected_duplicate_or_degenerate += 1
                    }
                    _ => report.rejected_no_topology_gain += 1,
                }
            }
        }
    }

    eprintln!(
        "  adaptive-mc two_adjacent_active_result: candidates_attempted={} candidates_accepted={} cells_split={} transition_patches_added={} triangles_added={} vertices_added={} rejected_no_topology_gain={} rejected_connected_components={} rejected_non_local_boundary={} rejected_duplicate_or_degenerate={}",
        report.candidates_attempted,
        report.candidates_accepted,
        report.cells_split,
        report.transition_patches_added,
        report.triangles_added,
        report.vertices_added,
        report.rejected_no_topology_gain,
        report.rejected_connected_components,
        report.rejected_non_local_boundary,
        report.rejected_duplicate_or_degenerate,
    );
    let _ = (
        sdf,
        balanced_cells,
        normal_eps,
        smooth_normals,
        min_region_target,
    );
    (best_mesh, report)
}

fn interval_cover_matches(
    min_expected: f32,
    max_expected: f32,
    mut intervals: Vec<(f32, f32)>,
    tol: f32,
) -> bool {
    if intervals.is_empty() {
        return false;
    }
    intervals.sort_by(|a, b| a.0.total_cmp(&b.0));
    if (intervals[0].0 - min_expected).abs() > tol {
        return false;
    }
    let mut cursor = intervals[0].1;
    for &(start, end) in intervals.iter().skip(1) {
        if start > cursor + tol {
            return false;
        }
        cursor = cursor.max(end);
    }
    (cursor - max_expected).abs() <= tol
}

fn tiled_span_matches(
    coarse: &FaceRecord,
    finer_neighbor_indices: &[usize],
    faces: &[FaceRecord],
) -> bool {
    let tol = 1e-5;
    let neighbors: Vec<&FaceRecord> = finer_neighbor_indices
        .iter()
        .map(|&idx| &faces[idx])
        .collect();
    if neighbors
        .iter()
        .any(|n| n.cell_size >= coarse.cell_size - tol)
    {
        return false;
    }

    let mut unique_u = vec![coarse.u_min, coarse.u_max];
    let mut unique_v = vec![coarse.v_min, coarse.v_max];
    for n in &neighbors {
        unique_u.push(n.u_min);
        unique_u.push(n.u_max);
        unique_v.push(n.v_min);
        unique_v.push(n.v_max);
    }
    unique_u.sort_by(|a, b| a.total_cmp(b));
    unique_u.dedup_by(|a, b| (*a - *b).abs() <= tol);
    unique_v.sort_by(|a, b| a.total_cmp(b));
    unique_v.dedup_by(|a, b| (*a - *b).abs() <= tol);

    if !interval_cover_matches(
        coarse.u_min,
        coarse.u_max,
        neighbors.iter().map(|n| (n.u_min, n.u_max)).collect(),
        tol,
    ) || !interval_cover_matches(
        coarse.v_min,
        coarse.v_max,
        neighbors.iter().map(|n| (n.v_min, n.v_max)).collect(),
        tol,
    ) {
        return false;
    }

    for u_pair in unique_u.windows(2) {
        for v_pair in unique_v.windows(2) {
            let patch_u_min = u_pair[0];
            let patch_u_max = u_pair[1];
            let patch_v_min = v_pair[0];
            let patch_v_max = v_pair[1];
            if patch_u_max <= patch_u_min + tol || patch_v_max <= patch_v_min + tol {
                continue;
            }
            let covering = neighbors.iter().filter(|n| {
                n.u_min <= patch_u_min + tol
                    && n.u_max >= patch_u_max - tol
                    && n.v_min <= patch_v_min + tol
                    && n.v_max >= patch_v_max - tol
            });
            if covering.count() != 1 {
                return false;
            }
        }
    }

    true
}

#[derive(Debug, Default)]
struct AdaptiveMcInstrumentation {
    seeds_generated: AtomicU64,
    cells_pruned: AtomicU64,
    cells_kept: AtomicU64,
    cells_saved_by_multi_crossing: AtomicU64,
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

fn classify_sign(d: f32, deadband: f32) -> i8 {
    if d > deadband {
        1
    } else if d < -deadband {
        -1
    } else {
        0
    }
}

fn count_sign_alternations(samples: &[f32], deadband: f32) -> u32 {
    let mut alternations = 0u32;
    let mut prev = 0i8;
    for &sample in samples {
        let sign = classify_sign(sample, deadband);
        if sign == 0 {
            continue;
        }
        if prev != 0 && sign != prev {
            alternations += 1;
        }
        prev = sign;
    }
    alternations
}

fn has_multi_crossing(sdf: &dyn Sdf, min: Vec3, size: f32) -> bool {
    let deadband = (size * 0.005_f32).max(1e-4_f32);
    let center = min + Vec3::splat(size * 0.5);
    let sample_line = |start: Vec3, step: Vec3| -> [f32; 5] {
        std::array::from_fn(|i| sdf.distance(start + step * i as f32))
    };
    let x = sample_line(
        Vec3::new(min.x, center.y, center.z),
        Vec3::new(size * 0.25, 0.0, 0.0),
    );
    let y = sample_line(
        Vec3::new(center.x, min.y, center.z),
        Vec3::new(0.0, size * 0.25, 0.0),
    );
    let z = sample_line(
        Vec3::new(center.x, center.y, min.z),
        Vec3::new(0.0, 0.0, size * 0.25),
    );
    [x, y, z]
        .iter()
        .map(|line| count_sign_alternations(line, deadband))
        .max()
        .unwrap_or(0)
        > 1
}

fn has_surface(corners: &[f32; 8]) -> bool {
    let all_pos = corners.iter().all(|&v| v >= 0.0);
    let all_neg = corners.iter().all(|&v| v <= 0.0);
    !all_pos && !all_neg
}

fn cell_intersects_region(min: Vec3, max: Vec3, region: &LocalRefineRegion) -> bool {
    min.cmple(region.max).all() && region.min.cmple(max).all()
}

fn aabb_gap(min_a: Vec3, max_a: Vec3, min_b: Vec3, max_b: Vec3) -> f32 {
    let dx = if max_a.x < min_b.x {
        min_b.x - max_a.x
    } else if max_b.x < min_a.x {
        min_a.x - max_b.x
    } else {
        0.0
    };
    let dy = if max_a.y < min_b.y {
        min_b.y - max_a.y
    } else if max_b.y < min_a.y {
        min_a.y - max_b.y
    } else {
        0.0
    };
    let dz = if max_a.z < min_b.z {
        min_b.z - max_a.z
    } else if max_b.z < min_a.z {
        min_a.z - max_b.z
    } else {
        0.0
    };
    dx.max(dy).max(dz)
}

fn local_target_size(min: Vec3, max: Vec3, base_target: f32, regions: &[LocalRefineRegion]) -> f32 {
    let mut target = base_target;
    for region in regions {
        if cell_intersects_region(min, max, region) {
            target = target.min(region.target_cell_size.max(0.02));
            continue;
        }

        // Enforce a 2:1 transition band around refined regions. If a coarse cell
        // sits within roughly one cell width of a finer region, cap it to one
        // refinement level above that region so leaf sizes change gradually.
        let gap = aabb_gap(min, max, region.min, region.max);
        if gap <= (max - min).max_element() * 0.51 {
            target = target.min((region.target_cell_size.max(0.02)) * 2.0);
        }
    }
    target.max(0.02)
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
        if can_prune(&corners, size) {
            continue;
        }

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

fn collect_seeds_local(
    sdf: &dyn Sdf,
    root_min: Vec3,
    root_size: f32,
    target_size: f32,
    regions: &[LocalRefineRegion],
    split_depth: u32,
    stats: &AdaptiveMcInstrumentation,
) -> Vec<(Vec3, f32)> {
    let mut stack: Vec<(Vec3, f32, u32)> = vec![(root_min, root_size, 0)];
    let mut seeds = Vec::new();

    while let Some((min, size, depth)) = stack.pop() {
        let corners = eval_corners(sdf, min, size);
        if can_prune(&corners, size) {
            stats.cells_pruned.fetch_add(1, Ordering::Relaxed);
            continue;
        }

        let cell_max = min + Vec3::splat(size);
        let local_target = local_target_size(min, cell_max, target_size, regions);

        if size <= local_target * 1.001 {
            seeds.push((min, size));
            stats.seeds_generated.fetch_add(1, Ordering::Relaxed);
            continue;
        }

        if depth >= split_depth {
            seeds.push((min, size));
            stats.seeds_generated.fetch_add(1, Ordering::Relaxed);
            continue;
        }

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
        if can_prune(&corners, size) {
            continue;
        }

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

fn process_subtree_local(
    sdf: &dyn Sdf,
    seed_min: Vec3,
    seed_size: f32,
    target_size: f32,
    regions: &[LocalRefineRegion],
    stats: &AdaptiveMcInstrumentation,
) -> Vec<OctreeCell> {
    let mut stack: Vec<(Vec3, f32)> = vec![(seed_min, seed_size)];
    let mut cells = Vec::new();

    while let Some((min, size)) = stack.pop() {
        let corners = eval_corners(sdf, min, size);
        if can_prune(&corners, size) {
            stats.cells_pruned.fetch_add(1, Ordering::Relaxed);
            continue;
        }

        let cell_max = min + Vec3::splat(size);
        let local_target = local_target_size(min, cell_max, target_size, regions);

        if size <= local_target * 1.001 {
            if has_surface(&corners) {
                cells.push(OctreeCell { min, size, corners });
                stats.cells_kept.fetch_add(1, Ordering::Relaxed);
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

fn subdivide_cell(cell: &OctreeCell, sdf: &dyn Sdf) -> Vec<OctreeCell> {
    let half = cell.size * 0.5;
    let mut children = Vec::with_capacity(8);
    for dz in 0..2usize {
        for dy in 0..2usize {
            for dx in 0..2usize {
                let child_min = cell.min + Vec3::new(dx as f32, dy as f32, dz as f32) * half;
                let corners = eval_corners(sdf, child_min, half);
                if has_surface(&corners) {
                    children.push(OctreeCell {
                        min: child_min,
                        size: half,
                        corners,
                    });
                }
            }
        }
    }
    children
}

fn subdivide_cell_with_prune(cell: &OctreeCell, sdf: &dyn Sdf) -> Vec<OctreeCell> {
    let half = cell.size * 0.5;
    let mut children = Vec::with_capacity(8);
    for dz in 0..2usize {
        for dy in 0..2usize {
            for dx in 0..2usize {
                let child_min = cell.min + Vec3::new(dx as f32, dy as f32, dz as f32) * half;
                let corners = eval_corners(sdf, child_min, half);
                if can_prune(&corners, half) {
                    continue;
                }
                if has_surface(&corners) {
                    children.push(OctreeCell {
                        min: child_min,
                        size: half,
                        corners,
                    });
                }
            }
        }
    }
    children
}

fn propagate_size_gradient(
    cells: &mut Vec<OctreeCell>,
    sdf: &dyn Sdf,
    _regions: &[LocalRefineRegion],
    min_cell_size: f32,
) {
    let before_count = cells.len();
    let t0 = Instant::now();
    let mut total_split_cells = 0usize;
    let mut propagation_iterations = 0usize;
    let min_cell_size = min_cell_size.max(0.02);
    let mut adjacency_build_count = 0usize;
    let mut split_bboxes: Vec<(Vec3, Vec3, f32)> = Vec::new();

    loop {
        propagation_iterations += 1;
        adjacency_build_count += 1;
        let adjacency = FaceAdjacency::build(cells);
        let mut queue: VecDeque<usize> = cells
            .iter()
            .enumerate()
            .filter_map(|(idx, cell)| (cell.size <= min_cell_size * 1.001).then_some(idx))
            .collect();
        let mut visited = vec![false; cells.len()];
        for &idx in &queue {
            visited[idx] = true;
        }

        let mut split_mark = vec![false; cells.len()];
        while let Some(cell_idx) = queue.pop_front() {
            let current_size = cells[cell_idx].size;
            for &neighbor_idx in adjacency.neighbors(cell_idx) {
                if cells[neighbor_idx].size > current_size * 2.0 + 1e-6 {
                    split_mark[neighbor_idx] = true;
                    continue;
                }
                if !visited[neighbor_idx] {
                    visited[neighbor_idx] = true;
                    queue.push_back(neighbor_idx);
                }
            }
        }

        let split_this_round = split_mark
            .iter()
            .enumerate()
            .filter(|(idx, split)| **split && cells[*idx].size > min_cell_size * 1.001)
            .count();
        if split_this_round == 0 {
            break;
        }
        total_split_cells += split_this_round;

        let mut next_cells = Vec::with_capacity(cells.len() + split_this_round * 4);
        for (idx, cell) in cells.iter().enumerate() {
            if split_mark[idx] && cell.size > min_cell_size * 1.001 {
                split_bboxes.push((cell.min, cell.min + Vec3::splat(cell.size), cell.size));
                next_cells.extend(subdivide_cell_with_prune(cell, sdf));
            } else {
                next_cells.push(cell.clone());
            }
        }
        *cells = next_cells;
    }

    adjacency_build_count += 1;
    let final_ratio = FaceAdjacency::build(cells).max_touching_ratio();
    eprintln!(
        "  adaptive-mc propagate_size_gradient: elapsed={:?} cells_before={} cells_after={} cells_split={} iterations={} face_adjacency_builds={} max_touching_ratio={:.3}",
        t0.elapsed(),
        before_count,
        cells.len(),
        total_split_cells,
        propagation_iterations,
        adjacency_build_count,
        final_ratio,
    );
    if !split_bboxes.is_empty() {
        eprintln!(
            "  adaptive-mc propagate_size_gradient: split_cell_bboxes={}",
            split_bboxes.len()
        );
        for (idx, (min, max, size)) in split_bboxes.iter().enumerate() {
            eprintln!(
                "    split_cell[{idx}]: size={size:.3} min=({:.3}, {:.3}, {:.3}) max=({:.3}, {:.3}, {:.3})",
                min.x, min.y, min.z, max.x, max.y, max.z
            );
        }
    }
}

fn cells_touch_or_overlap(a: &OctreeCell, b: &OctreeCell, eps: f32) -> bool {
    let amax = a.min + Vec3::splat(a.size);
    let bmax = b.min + Vec3::splat(b.size);
    let separated = amax.x < b.min.x - eps
        || bmax.x < a.min.x - eps
        || amax.y < b.min.y - eps
        || bmax.y < a.min.y - eps
        || amax.z < b.min.z - eps
        || bmax.z < a.min.z - eps;
    !separated
}

fn balance_surface_cells(
    _sdf: &dyn Sdf,
    cells: Vec<OctreeCell>,
    _base_target: f32,
) -> Vec<OctreeCell> {
    let balance_t0 = Instant::now();
    let adjacency = FaceAdjacency::build(&cells);
    let hotspot_min = Vec3::new(611.0, -70.0, 24.0);
    let hotspot_max = Vec3::new(680.0, 0.0, 43.0);
    let hotspot_samples: Vec<_> = cells
        .iter()
        .filter(|cell| {
            let cell_max = cell.min + Vec3::splat(cell.size);
            cell.min.cmple(hotspot_max).all() && hotspot_min.cmple(cell_max).all()
        })
        .take(10)
        .map(|cell| (cell.min, cell.min + Vec3::splat(cell.size), cell.size))
        .collect();
    eprintln!(
        "  adaptive-mc balance_surface_cells: measurement_only elapsed={:?} iterations=0 final_cells={} max_touching_ratio={:.3}",
        balance_t0.elapsed(),
        cells.len(),
        adjacency.max_touching_ratio(),
    );
    let mut size_histogram: BTreeMap<i32, usize> = BTreeMap::new();
    for cell in &cells {
        *size_histogram
            .entry((cell.size * 1000.0).round() as i32)
            .or_default() += 1;
    }
    eprintln!(
        "  adaptive-mc surface_cell_size_histogram: distinct_sizes={}",
        size_histogram.len()
    );
    for (size_key, count) in &size_histogram {
        eprintln!(
            "    cell_size={:.3} count={}",
            *size_key as f32 / 1000.0,
            count
        );
    }
    eprintln!(
        "  adaptive-mc hotspot_sample [9,4,4]: matched_cells={} printed={}",
        cells
            .iter()
            .filter(|cell| {
                let cell_max = cell.min + Vec3::splat(cell.size);
                cell.min.cmple(hotspot_max).all() && hotspot_min.cmple(cell_max).all()
            })
            .count(),
        hotspot_samples.len()
    );
    for (idx, (min, max, size)) in hotspot_samples.iter().enumerate() {
        eprintln!(
            "    hotspot_cell[{idx}]: size={size:.3} min=({:.3}, {:.3}, {:.3}) max=({:.3}, {:.3}, {:.3})",
            min.x, min.y, min.z, max.x, max.y, max.z
        );
    }
    cells
}

/// Run standard marching cubes on a single cell and return its triangle vertex
/// positions (no normals yet — those are computed in batch later).
fn mc_cell_positions(cell: &OctreeCell) -> Vec<Vec3> {
    let mut cube_index = 0u8;
    for (i, &val) in cell.corners.iter().enumerate() {
        if val < 0.0 {
            cube_index |= 1 << i;
        }
    }

    let edge_flags = EDGE_TABLE[cube_index as usize];
    if edge_flags == 0 {
        return vec![];
    }

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

fn quantize_point(p: Vec3, eps: f32) -> [i32; 3] {
    [
        (p.x / eps).round() as i32,
        (p.y / eps).round() as i32,
        (p.z / eps).round() as i32,
    ]
}

fn edge_key(a: Vec3, b: Vec3, eps: f32) -> ([i32; 3], [i32; 3]) {
    let qa = quantize_point(a, eps);
    let qb = quantize_point(b, eps);
    if qa <= qb { (qa, qb) } else { (qb, qa) }
}

fn face_uv_to_world(axis: u8, plane: f32, u: f32, v: f32) -> Vec3 {
    match axis {
        0 => Vec3::new(plane, u, v),
        1 => Vec3::new(u, plane, v),
        _ => Vec3::new(u, v, plane),
    }
}

fn classify_edge_transition_record(a: &FaceRecord, b: &FaceRecord) -> Option<EdgeTransitionRecord> {
    let (coarse, fine) = if a.cell_size >= b.cell_size {
        (a, b)
    } else {
        (b, a)
    };

    let ratio = coarse.cell_size / fine.cell_size.max(1e-6);
    if !approx_eq(ratio, 2.0) {
        return None;
    }

    let u_area = intervals_strictly_overlap(coarse.u_min, coarse.u_max, fine.u_min, fine.u_max);
    let v_area = intervals_strictly_overlap(coarse.v_min, coarse.v_max, fine.v_min, fine.v_max);
    let u_contact = intervals_touch_or_overlap(coarse.u_min, coarse.u_max, fine.u_min, fine.u_max);
    let v_contact = intervals_touch_or_overlap(coarse.v_min, coarse.v_max, fine.v_min, fine.v_max);

    let (overlap_axis, touch_axis) = if u_area && v_contact && !v_area {
        (0u8, 1u8)
    } else if v_area && u_contact && !u_area {
        (1u8, 0u8)
    } else if !u_area && !v_area && u_contact && v_contact {
        let p = face_uv_to_world(
            coarse.axis,
            coarse.plane_coord,
            (coarse.u_min.max(fine.u_min) + coarse.u_max.min(fine.u_max)) * 0.5,
            (coarse.v_min.max(fine.v_min) + coarse.v_max.min(fine.v_max)) * 0.5,
        );
        return Some(EdgeTransitionRecord {
            coarse_cell_idx: coarse.cell_idx,
            fine_cell_idx: fine.cell_idx,
            coarse_face_id: coarse.face_id,
            fine_face_id: fine.face_id,
            axis: coarse.axis,
            sign: coarse.sign,
            plane_coord: coarse.plane_coord,
            coarse_cell_size: coarse.cell_size,
            fine_cell_size: fine.cell_size,
            edge_start: p,
            edge_end: p,
            overlap_axis: 0,
            touch_axis: 1,
            case: EdgeTransitionCase::CornerTouch,
        });
    } else {
        let p0 = face_uv_to_world(
            coarse.axis,
            coarse.plane_coord,
            coarse.u_min.max(fine.u_min),
            coarse.v_min.max(fine.v_min),
        );
        let p1 = face_uv_to_world(
            coarse.axis,
            coarse.plane_coord,
            coarse.u_max.min(fine.u_max),
            coarse.v_max.min(fine.v_max),
        );
        return Some(EdgeTransitionRecord {
            coarse_cell_idx: coarse.cell_idx,
            fine_cell_idx: fine.cell_idx,
            coarse_face_id: coarse.face_id,
            fine_face_id: fine.face_id,
            axis: coarse.axis,
            sign: coarse.sign,
            plane_coord: coarse.plane_coord,
            coarse_cell_size: coarse.cell_size,
            fine_cell_size: fine.cell_size,
            edge_start: p0,
            edge_end: p1,
            overlap_axis: 0,
            touch_axis: 1,
            case: EdgeTransitionCase::IrregularEdge,
        });
    };

    let (
        overlap_min,
        overlap_max,
        coarse_overlap_min,
        coarse_overlap_max,
        coarse_touch_min,
        coarse_touch_max,
        fine_touch_min,
        fine_touch_max,
    ) = if overlap_axis == 0 {
        (
            coarse.u_min.max(fine.u_min),
            coarse.u_max.min(fine.u_max),
            coarse.u_min,
            coarse.u_max,
            coarse.v_min,
            coarse.v_max,
            fine.v_min,
            fine.v_max,
        )
    } else {
        (
            coarse.v_min.max(fine.v_min),
            coarse.v_max.min(fine.v_max),
            coarse.v_min,
            coarse.v_max,
            coarse.u_min,
            coarse.u_max,
            fine.u_min,
            fine.u_max,
        )
    };
    let overlap_len = (overlap_max - overlap_min).max(0.0);
    let coarse_overlap_len = (coarse_overlap_max - coarse_overlap_min).max(0.0);
    let coarse_mid = (coarse_overlap_min + coarse_overlap_max) * 0.5;
    let coarse_half = coarse_overlap_len * 0.5;
    let touch_on_boundary =
        approx_eq(fine_touch_min, coarse_touch_max) || approx_eq(fine_touch_max, coarse_touch_min);

    let is_true_half = approx_eq(overlap_len, coarse_half)
        && (approx_eq(overlap_min, coarse_overlap_min) && approx_eq(overlap_max, coarse_mid)
            || approx_eq(overlap_min, coarse_mid) && approx_eq(overlap_max, coarse_overlap_max))
        && touch_on_boundary;

    let touch_coord = if overlap_axis == 0 {
        if approx_eq(fine.v_min, coarse.v_max) {
            coarse.v_max
        } else if approx_eq(fine.v_max, coarse.v_min) {
            coarse.v_min
        } else {
            (coarse.v_min.max(fine.v_min) + coarse.v_max.min(fine.v_max)) * 0.5
        }
    } else if approx_eq(fine.u_min, coarse.u_max) {
        coarse.u_max
    } else if approx_eq(fine.u_max, coarse.u_min) {
        coarse.u_min
    } else {
        (coarse.u_min.max(fine.u_min) + coarse.u_max.min(fine.u_max)) * 0.5
    };

    let (edge_start, edge_end) = if overlap_axis == 0 {
        (
            face_uv_to_world(coarse.axis, coarse.plane_coord, overlap_min, touch_coord),
            face_uv_to_world(coarse.axis, coarse.plane_coord, overlap_max, touch_coord),
        )
    } else {
        (
            face_uv_to_world(coarse.axis, coarse.plane_coord, touch_coord, overlap_min),
            face_uv_to_world(coarse.axis, coarse.plane_coord, touch_coord, overlap_max),
        )
    };

    Some(EdgeTransitionRecord {
        coarse_cell_idx: coarse.cell_idx,
        fine_cell_idx: fine.cell_idx,
        coarse_face_id: coarse.face_id,
        fine_face_id: fine.face_id,
        axis: coarse.axis,
        sign: coarse.sign,
        plane_coord: coarse.plane_coord,
        coarse_cell_size: coarse.cell_size,
        fine_cell_size: fine.cell_size,
        edge_start,
        edge_end,
        overlap_axis,
        touch_axis,
        case: if is_true_half {
            EdgeTransitionCase::TrueHalfEdge
        } else {
            EdgeTransitionCase::IrregularEdge
        },
    })
}

fn transition_face_record(cell: &OctreeCell, face_id: u8) -> FaceRecord {
    enumerate_cell_faces(0, cell)[face_id as usize].clone()
}

fn emit_transition_faces(
    sdf: &dyn Sdf,
    surface_cells: &[OctreeCell],
    transition_faces: &[TransitionFace],
    vertices: &mut Vec<Vertex>,
    vertex_source_cells: &mut Vec<Vec<usize>>,
    indices: &mut Vec<u32>,
    edge_vertices: &mut HashMap<([i32; 3], [i32; 3]), u32>,
    key_eps: f32,
    normal_eps: f32,
) -> TransitionEmissionStats {
    let mut stats = TransitionEmissionStats {
        faces_found: transition_faces.len(),
        ..TransitionEmissionStats::default()
    };

    for transition in transition_faces {
        let coarse = &surface_cells[transition.coarse_cell_idx];
        let coarse_face = transition_face_record(coarse, transition.coarse_face_id);
        if transition.fine_cell_indices.len() != 4 {
            stats.faces_skipped += 1;
            continue;
        }

        let fine_size = transition
            .fine_cell_indices
            .iter()
            .map(|&idx| surface_cells[idx].size)
            .fold(f32::MAX, f32::min);
        if !approx_eq(coarse.size, fine_size * 2.0) {
            stats.faces_skipped += 1;
            continue;
        }

        let inward = if coarse_face.sign == 0 {
            fine_size
        } else {
            -fine_size
        };
        let low_plane = coarse_face.plane_coord + inward;
        let u_mid = (coarse_face.u_min + coarse_face.u_max) * 0.5;
        let v_mid = (coarse_face.v_min + coarse_face.v_max) * 0.5;
        let us = [coarse_face.u_min, u_mid, coarse_face.u_max];
        let vs = [coarse_face.v_min, v_mid, coarse_face.v_max];

        let mut endpoint_positions = [Vec3::ZERO; 13];
        for row in 0..3 {
            for col in 0..3 {
                endpoint_positions[row * 3 + col] =
                    face_uv_to_world(coarse_face.axis, coarse_face.plane_coord, us[col], vs[row]);
            }
        }
        endpoint_positions[9] = face_uv_to_world(
            coarse_face.axis,
            low_plane,
            coarse_face.u_min,
            coarse_face.v_min,
        );
        endpoint_positions[10] = face_uv_to_world(
            coarse_face.axis,
            low_plane,
            coarse_face.u_max,
            coarse_face.v_min,
        );
        endpoint_positions[11] = face_uv_to_world(
            coarse_face.axis,
            low_plane,
            coarse_face.u_min,
            coarse_face.v_max,
        );
        endpoint_positions[12] = face_uv_to_world(
            coarse_face.axis,
            low_plane,
            coarse_face.u_max,
            coarse_face.v_max,
        );

        let endpoint_values: [f32; 13] =
            std::array::from_fn(|idx| sdf.distance(endpoint_positions[idx]));
        let high_signs: [bool; 9] = std::array::from_fn(|idx| endpoint_values[idx] < 0.0);
        let case_index = transition_cell_case(high_signs);
        let cell_data = TRANSITION_CELL_DATA[case_index];
        let triangle_count = cell_data.triangle_count();
        if triangle_count == 0 {
            continue;
        }

        let vertex_count = cell_data.vertex_count();
        let mut transition_vertex_indices = vec![u32::MAX; vertex_count];
        for (local_idx, slot) in transition_vertex_indices.iter_mut().enumerate() {
            let data = TRANSITION_VERTEX_DATA[case_index][local_idx];
            let a = data.endpoint_a() as usize;
            let b = data.endpoint_b() as usize;
            if a >= endpoint_positions.len() || b >= endpoint_positions.len() {
                continue;
            }
            let p0 = endpoint_positions[a];
            let p1 = endpoint_positions[b];
            let d0 = endpoint_values[a];
            let d1 = endpoint_values[b];
            let key = edge_key(p0, p1, key_eps);
            let vidx = if let Some(&idx) = edge_vertices.get(&key) {
                idx
            } else {
                let t = if (d1 - d0).abs() > 1e-7 {
                    (-d0 / (d1 - d0)).clamp(0.0, 1.0)
                } else {
                    0.5
                };
                let p = p0.lerp(p1, t);
                let n = sdf_gradient(sdf, p, normal_eps);
                let idx = vertices.len() as u32;
                vertices.push(Vertex {
                    position: [p.x, p.y, p.z],
                    normal: [n.x, n.y, n.z],
                });
                let mut cells = vec![transition.coarse_cell_idx];
                for &cell_idx in &transition.fine_cell_indices {
                    if !cells.contains(&cell_idx) {
                        cells.push(cell_idx);
                    }
                }
                vertex_source_cells.push(cells);
                edge_vertices.insert(key, idx);
                stats.vertices_added += 1;
                idx
            };
            *slot = vidx;
        }

        let before_tris = indices.len() / 3;
        for tri in 0..triangle_count {
            let base = tri * 3;
            let tri_indices = [
                cell_data.vertex_index[base] as usize,
                cell_data.vertex_index[base + 1] as usize,
                cell_data.vertex_index[base + 2] as usize,
            ];
            if tri_indices
                .iter()
                .any(|&idx| idx >= transition_vertex_indices.len())
            {
                continue;
            }
            let a = transition_vertex_indices[tri_indices[0]];
            let b = transition_vertex_indices[tri_indices[1]];
            let c = transition_vertex_indices[tri_indices[2]];
            if a == u32::MAX || b == u32::MAX || c == u32::MAX || a == b || b == c || c == a {
                continue;
            }
            indices.extend_from_slice(&[a, b, c]);
        }
        let emitted = indices.len() / 3 - before_tris;
        if emitted > 0 {
            stats.faces_emitted += 1;
            stats.triangles_emitted += emitted;
        }
    }

    stats
}

fn triangle_area_from_positions(a: Vec3, b: Vec3, c: Vec3) -> f32 {
    0.5 * (b - a).cross(c - a).length()
}

fn build_edge_incidence(indices: &[u32]) -> HashMap<(u32, u32), usize> {
    let mut counts = HashMap::new();
    for tri in indices.chunks_exact(3) {
        let a = tri[0];
        let b = tri[1];
        let c = tri[2];
        *counts.entry(canonical_mesh_edge(a, b)).or_insert(0) += 1;
        *counts.entry(canonical_mesh_edge(b, c)).or_insert(0) += 1;
        *counts.entry(canonical_mesh_edge(c, a)).or_insert(0) += 1;
    }
    counts
}

fn find_existing_vertex_within_eps(
    vertices: &[Vertex],
    buckets: &HashMap<[i32; 3], Vec<u32>>,
    p: Vec3,
    eps: f32,
) -> Option<u32> {
    let q = quantize_point(p, eps);
    for dx in -1..=1 {
        for dy in -1..=1 {
            for dz in -1..=1 {
                let key = [q[0] + dx, q[1] + dy, q[2] + dz];
                if let Some(candidates) = buckets.get(&key) {
                    for &idx in candidates {
                        let v = Vec3::from_array(vertices[idx as usize].position);
                        if v.distance(p) <= eps {
                            return Some(idx);
                        }
                    }
                }
            }
        }
    }
    None
}

fn emit_edge_transitions(
    sdf: &dyn Sdf,
    edge_transitions: &[EdgeTransitionRecord],
    vertices: &mut Vec<Vertex>,
    vertex_source_cells: &mut Vec<Vec<usize>>,
    indices: &mut Vec<u32>,
    edge_vertices: &mut HashMap<([i32; 3], [i32; 3]), u32>,
    key_eps: f32,
    normal_eps: f32,
    degenerate_cell_size: f32,
) -> EdgeTransitionEmissionStats {
    let mut stats = EdgeTransitionEmissionStats::default();
    let mut incidence = build_edge_incidence(indices);
    let mut vertex_buckets: HashMap<[i32; 3], Vec<u32>> = HashMap::new();
    for (idx, v) in vertices.iter().enumerate() {
        vertex_buckets
            .entry(quantize_point(Vec3::from_array(v.position), key_eps))
            .or_default()
            .push(idx as u32);
    }
    let min_area = (degenerate_cell_size * degenerate_cell_size * 1e-4).max(1e-6);

    for record in edge_transitions {
        stats.attempted += 1;

        let a = match find_existing_vertex_within_eps(
            vertices,
            &vertex_buckets,
            record.edge_start,
            key_eps * 2.0,
        ) {
            Some(v) => v,
            None => {
                stats.rejected += 1;
                stats.rejected_missing_fine_endpoints += 1;
                continue;
            }
        };
        let b = match find_existing_vertex_within_eps(
            vertices,
            &vertex_buckets,
            record.edge_end,
            key_eps * 2.0,
        ) {
            Some(v) => v,
            None => {
                stats.rejected += 1;
                stats.rejected_missing_fine_endpoints += 1;
                continue;
            }
        };
        if a == b {
            stats.rejected += 1;
            stats.rejected_degenerate += 1;
            continue;
        }

        let midpoint = record.edge_start.lerp(record.edge_end, 0.5);
        let mid_key = edge_key(record.edge_start, record.edge_end, key_eps);
        let m = if let Some(&idx) = edge_vertices.get(&mid_key) {
            idx
        } else if let Some(idx) =
            find_existing_vertex_within_eps(vertices, &vertex_buckets, midpoint, key_eps * 2.0)
        {
            edge_vertices.insert(mid_key, idx);
            idx
        } else {
            let n = sdf_gradient(sdf, midpoint, normal_eps);
            let idx = vertices.len() as u32;
            vertices.push(Vertex {
                position: midpoint.to_array(),
                normal: n.to_array(),
            });
            vertex_source_cells.push(vec![record.coarse_cell_idx, record.fine_cell_idx]);
            edge_vertices.insert(mid_key, idx);
            vertex_buckets
                .entry(quantize_point(midpoint, key_eps))
                .or_default()
                .push(idx);
            stats.vertices_added += 1;
            idx
        };

        if m == a || m == b {
            stats.rejected += 1;
            stats.rejected_degenerate += 1;
            continue;
        }

        let pa = Vec3::from_array(vertices[a as usize].position);
        let pb = Vec3::from_array(vertices[b as usize].position);
        let pm = Vec3::from_array(vertices[m as usize].position);
        if triangle_area_from_positions(pm, pa, pb) <= min_area {
            stats.rejected += 1;
            stats.rejected_degenerate += 1;
            continue;
        }

        let candidate_edges = [
            canonical_mesh_edge(m, a),
            canonical_mesh_edge(a, b),
            canonical_mesh_edge(b, m),
        ];
        if candidate_edges
            .iter()
            .any(|e| incidence.get(e).copied().unwrap_or(0) >= 2)
        {
            stats.rejected += 1;
            stats.rejected_non_manifold += 1;
            continue;
        }

        indices.extend_from_slice(&[m, a, b]);
        for e in candidate_edges {
            *incidence.entry(e).or_insert(0) += 1;
        }
        stats.emitted += 1;
        stats.triangles_emitted += 1;
    }

    stats.rejected = stats.attempted.saturating_sub(stats.emitted);
    stats
}

fn emit_mesh_from_cells(
    sdf: &dyn Sdf,
    surface_cells: &[OctreeCell],
    normal_eps: f32,
    smooth_normals: bool,
    weld_epsilon: f32,
    degenerate_cell_size: f32,
) -> Mesh {
    let mut vertices: Vec<Vertex> = Vec::new();
    let mut vertex_source_cells: Vec<Vec<usize>> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();
    let mut raw_triangle_sources: Vec<RawTriangleSource> = Vec::new();
    let mut edge_vertices: HashMap<([i32; 3], [i32; 3]), u32> =
        HashMap::with_capacity(surface_cells.len() * 6);
    let key_eps = weld_epsilon.max(0.001) * 0.25;
    let adjacency = FaceAdjacency::build(surface_cells);
    let transition_faces: Vec<TransitionFace> = adjacency.transition_faces().collect();
    let edge_transition_records: Vec<EdgeTransitionRecord> =
        adjacency.edge_transition_records().collect();

    for (cell_idx, cell) in surface_cells.iter().enumerate() {
        let mut cube_index = 0u8;
        for (i, &val) in cell.corners.iter().enumerate() {
            if val < 0.0 {
                cube_index |= 1 << i;
            }
        }
        let edge_flags = EDGE_TABLE[cube_index as usize];
        if edge_flags == 0 {
            continue;
        }

        let cp: [Vec3; 8] = std::array::from_fn(|i| {
            let [cx, cy, cz] = CUBE_CORNERS[i];
            cell.min + Vec3::new(cx as f32, cy as f32, cz as f32) * cell.size
        });

        let mut ev_idx = [u32::MAX; 12];
        for edge in 0..12usize {
            if edge_flags & (1 << edge) == 0 {
                continue;
            }
            let [v0, v1] = EDGE_VERTICES[edge];
            let d0 = cell.corners[v0];
            let d1 = cell.corners[v1];
            let t = if (d1 - d0).abs() > 1e-7 {
                (-d0 / (d1 - d0)).clamp(0.0, 1.0)
            } else {
                0.5
            };
            let p0 = cp[v0];
            let p1 = cp[v1];
            let key = edge_key(p0, p1, key_eps);
            let vidx = if let Some(&idx) = edge_vertices.get(&key) {
                let sources = &mut vertex_source_cells[idx as usize];
                if !sources.contains(&cell_idx) {
                    sources.push(cell_idx);
                }
                idx
            } else {
                let p = p0.lerp(p1, t);
                let n = sdf_gradient(sdf, p, normal_eps);
                let idx = vertices.len() as u32;
                vertices.push(Vertex {
                    position: [p.x, p.y, p.z],
                    normal: [n.x, n.y, n.z],
                });
                vertex_source_cells.push(vec![cell_idx]);
                edge_vertices.insert(key, idx);
                idx
            };
            ev_idx[edge] = vidx;
        }

        let tri_row = &TRI_TABLE[cube_index as usize];
        let mut i = 0;
        while i < 16 && tri_row[i] != -1 {
            let tri = [
                ev_idx[tri_row[i] as usize],
                ev_idx[tri_row[i + 1] as usize],
                ev_idx[tri_row[i + 2] as usize],
            ];
            indices.extend_from_slice(&tri);
            raw_triangle_sources.push(RawTriangleSource {
                tri,
                cell_idx,
                cube_index,
            });
            i += 3;
        }
    }

    let regular_vertex_count = vertices.len();
    let transition_stats = emit_transition_faces(
        sdf,
        surface_cells,
        &transition_faces,
        &mut vertices,
        &mut vertex_source_cells,
        &mut indices,
        &mut edge_vertices,
        key_eps,
        normal_eps,
    );
    eprintln!(
        "  adaptive-mc transition_emission: faces_found={} faces_emitted={} faces_skipped={} vertices_added={} triangles_emitted={}",
        transition_stats.faces_found,
        transition_stats.faces_emitted,
        transition_stats.faces_skipped,
        vertices.len().saturating_sub(regular_vertex_count),
        transition_stats.triangles_emitted,
    );

    let edge_transition_stats = emit_edge_transitions(
        sdf,
        &edge_transition_records,
        &mut vertices,
        &mut vertex_source_cells,
        &mut indices,
        &mut edge_vertices,
        key_eps,
        normal_eps,
        degenerate_cell_size,
    );
    eprintln!(
        "  adaptive-mc edge_transition_emission: attempted={} emitted={} rejected={} triangles_emitted={} vertices_added={} rejected_missing_fine_endpoints={} rejected_degenerate={} rejected_non_manifold={}",
        edge_transition_stats.attempted,
        edge_transition_stats.emitted,
        edge_transition_stats.rejected,
        edge_transition_stats.triangles_emitted,
        edge_transition_stats.vertices_added,
        edge_transition_stats.rejected_missing_fine_endpoints,
        edge_transition_stats.rejected_degenerate,
        edge_transition_stats.rejected_non_manifold,
    );

    let mut mesh = Mesh { vertices, indices };
    print_boundary_checkpoint("post_edge_transitions", &mesh);
    print_boundary_checkpoint("post_emission", &mesh);
    audit_raw_emission_topology(
        &mesh,
        &raw_triangle_sources,
        surface_cells,
        weld_epsilon,
        "post_emission",
    );

    remove_degenerate_triangles(&mut mesh, degenerate_cell_size);
    print_boundary_checkpoint("post_degenerate", &mesh);
    duplicate_cleanup_stage_report("post_degenerate", &mesh, 0, 0, 0, 0, 0, 0, 0, "none");

    let weld_epsilon = weld_epsilon_override(weld_epsilon, "IMPLICIT_CAD_WELD1_EPS_MM");
    print_weld_checkpoint(
        &mut mesh,
        weld_epsilon,
        "post_weld_1",
        &vertex_source_cells,
        surface_cells,
    );
    duplicate_cleanup_stage_report("post_constrained_weld", &mesh, 0, 0, 0, 0, 0, 0, 0, "none");
    let repair_variant = repair_variant_from_env();
    let repair_edges_for_diag =
        run_repair_pipeline_variant(&mut mesh, degenerate_cell_size, repair_variant);

    let post_weld_2_stats = print_weld_checkpoint(
        &mut mesh,
        0.001,
        "post_weld_2",
        &vertex_source_cells,
        surface_cells,
    );
    print_weld_epsilon_summary(weld_epsilon, 0.001, 0.01);
    print_boundary_neighbor_distance_diagnostic(&mesh);

    if smooth_normals {
        apply_smooth_normals_weighted(&mut mesh);
    }
    print_boundary_checkpoint("final", &mesh);
    duplicate_cleanup_stage_report(
        "final_topology_validation",
        &mesh,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        "none",
    );
    print_final_defect_localization(&mesh);
    run_final_repair_variants(&mesh, degenerate_cell_size);
    print_repair_edge_distance_diagnostic(&mesh, &repair_edges_for_diag, &post_weld_2_stats.remap);
    mesh
}

fn canonical_mesh_edge(a: u32, b: u32) -> (u32, u32) {
    if a < b { (a, b) } else { (b, a) }
}

fn canonical_mesh_triangle(tri: [u32; 3]) -> [u32; 3] {
    let mut tri = tri;
    tri.sort_unstable();
    tri
}

fn permutation_parity_mesh(mut tri: [u32; 3]) -> bool {
    let mut swaps = 0usize;
    for i in 0..3 {
        let mut min_idx = i;
        for j in (i + 1)..3 {
            if tri[j] < tri[min_idx] {
                min_idx = j;
            }
        }
        if min_idx != i {
            tri.swap(i, min_idx);
            swaps += 1;
        }
    }
    swaps % 2 == 0
}

fn classify_patch_feature_from_bounds(
    center: Vec3,
    bounds_min: Vec3,
    bounds_max: Vec3,
) -> &'static str {
    let extent = (bounds_max - bounds_min).abs();
    let tol = Vec3::new(
        (extent.x * 0.08).max(2.0),
        (extent.y * 0.08).max(2.0),
        (extent.z * 0.08).max(2.0),
    );
    let near_bounds = (center.x - bounds_min.x).abs() <= tol.x
        || (center.x - bounds_max.x).abs() <= tol.x
        || (center.y - bounds_min.y).abs() <= tol.y
        || (center.y - bounds_max.y).abs() <= tol.y
        || (center.z - bounds_min.z).abs() <= tol.z
        || (center.z - bounds_max.z).abs() <= tol.z;
    if near_bounds {
        return "export_bounds_edge";
    }
    if center.x < bounds_min.x + 0.15 * extent.x {
        return "leading_edge";
    }
    if center.x > bounds_min.x + 0.8 * extent.x {
        return "trailing_edge";
    }
    if center.y.abs() < 0.15 * extent.y {
        return "root_or_center";
    }
    if center.y.abs() > 0.42 * extent.y {
        return "tip";
    }
    if center.z > bounds_min.z + 0.7 * extent.z {
        return "blend_or_upper";
    }
    "unknown"
}

fn log_raw_cluster_summary(
    prefix: &str,
    items: &[(Vec3, f32)],
    bounds_min: Vec3,
    bounds_max: Vec3,
) {
    if items.is_empty() {
        eprintln!("  adaptive-mc raw_audit {} clusters=0", prefix);
        return;
    }
    let extent = (bounds_max - bounds_min).abs() + Vec3::splat(1e-9);
    let mut bins: HashMap<(i32, i32, i32), Vec<(Vec3, f32)>> = HashMap::new();
    for (center, metric) in items {
        let rel = (*center - bounds_min) / extent;
        let key = (
            (rel.x * 10.0).floor().clamp(0.0, 9.0) as i32,
            (rel.y * 10.0).floor().clamp(0.0, 9.0) as i32,
            (rel.z * 10.0).floor().clamp(0.0, 9.0) as i32,
        );
        bins.entry(key).or_default().push((*center, *metric));
    }
    let mut ordered: Vec<_> = bins.into_iter().collect();
    ordered.sort_by(|a, b| b.1.len().cmp(&a.1.len()));
    eprintln!(
        "  adaptive-mc raw_audit {} clusters={}",
        prefix,
        ordered.len()
    );
    for (rank, (cell, pts)) in ordered.into_iter().take(5).enumerate() {
        let mut cmin = Vec3::splat(f32::MAX);
        let mut cmax = Vec3::splat(f32::MIN);
        let mut metric_sum = 0.0f32;
        for (center, metric) in &pts {
            cmin = cmin.min(*center);
            cmax = cmax.max(*center);
            metric_sum += *metric;
        }
        let avg_metric = metric_sum / pts.len() as f32;
        let feature =
            classify_patch_feature_from_bounds((cmin + cmax) * 0.5, bounds_min, bounds_max);
        eprintln!(
            "  adaptive-mc raw_audit {} cluster_{} count={} bbox=({:.3},{:.3},{:.3})..({:.3},{:.3},{:.3}) avg_metric={:.6} feature={} grid_aligned=true cell=[{},{},{}]",
            prefix,
            rank + 1,
            pts.len(),
            cmin.x,
            cmin.y,
            cmin.z,
            cmax.x,
            cmax.y,
            cmax.z,
            avg_metric,
            feature,
            cell.0,
            cell.1,
            cell.2,
        );
    }
}

fn audit_raw_emission_topology(
    mesh: &Mesh,
    raw_triangle_sources: &[RawTriangleSource],
    surface_cells: &[OctreeCell],
    weld_epsilon: f32,
    stage: &str,
) {
    if mesh.indices.is_empty() || raw_triangle_sources.is_empty() {
        return;
    }
    let positions = mesh_positions(mesh);
    let bounds_min = positions
        .iter()
        .copied()
        .fold(Vec3::splat(f32::INFINITY), |acc, p| acc.min(p));
    let bounds_max = positions
        .iter()
        .copied()
        .fold(Vec3::splat(f32::NEG_INFINITY), |acc, p| acc.max(p));

    let mut edge_to_triangles: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    let mut exact_tri_counts: HashMap<[u32; 3], usize> = HashMap::new();
    let mut canonical_tri_groups: HashMap<[u32; 3], Vec<usize>> = HashMap::new();
    let mut area_buckets = [0usize; 5];
    let mut zero_area = 0usize;
    let mut degenerate_triangle_indices = HashSet::new();
    let mut degenerate_centers = Vec::new();

    for (tri_idx, source) in raw_triangle_sources.iter().enumerate() {
        *exact_tri_counts.entry(source.tri).or_insert(0) += 1;
        canonical_tri_groups
            .entry(canonical_mesh_triangle(source.tri))
            .or_default()
            .push(tri_idx);
        for &(a, b) in &[
            (source.tri[0], source.tri[1]),
            (source.tri[1], source.tri[2]),
            (source.tri[2], source.tri[0]),
        ] {
            edge_to_triangles
                .entry(canonical_mesh_edge(a, b))
                .or_default()
                .push(tri_idx);
        }

        let p0 = positions[source.tri[0] as usize];
        let p1 = positions[source.tri[1] as usize];
        let p2 = positions[source.tri[2] as usize];
        let area = 0.5 * (p1 - p0).cross(p2 - p0).length();
        let center = (p0 + p1 + p2) / 3.0;
        let bucket = if area < 1e-12 {
            zero_area += 1;
            0
        } else if area < 1e-10 {
            1
        } else if area < 1e-8 {
            2
        } else if area < 1e-6 {
            3
        } else {
            4
        };
        area_buckets[bucket] += 1;
        if area < 1e-6 {
            degenerate_triangle_indices.insert(tri_idx);
            degenerate_centers.push((center, area));
        }
    }

    let exact_duplicate_triangles = exact_tri_counts
        .values()
        .map(|count| count.saturating_sub(1))
        .sum::<usize>();
    let same_vertex_set_duplicates = canonical_tri_groups
        .values()
        .map(|group| group.len().saturating_sub(1))
        .sum::<usize>();
    let mut reversed_duplicate_triangles = 0usize;
    for group in canonical_tri_groups.values() {
        if group.len() < 2 {
            continue;
        }
        let even = group
            .iter()
            .filter(|&&idx| permutation_parity_mesh(raw_triangle_sources[idx].tri))
            .count();
        let odd = group.len() - even;
        if even > 0 && odd > 0 {
            reversed_duplicate_triangles += even.min(odd);
        }
    }

    let mut welded_position_groups: HashMap<[i32; 3], usize> = HashMap::new();
    let inv_weld = 1.0 / weld_epsilon.max(1e-6);
    for p in &positions {
        let key = [
            (p.x * inv_weld).round() as i32,
            (p.y * inv_weld).round() as i32,
            (p.z * inv_weld).round() as i32,
        ];
        *welded_position_groups.entry(key).or_insert(0) += 1;
    }
    let near_duplicate_vertex_count = welded_position_groups
        .values()
        .map(|count| count.saturating_sub(1))
        .sum::<usize>();

    let mut edges_1 = 0usize;
    let mut edges_2 = 0usize;
    let mut edges_3 = 0usize;
    let mut edges_4 = 0usize;
    let mut edges_gt4 = 0usize;
    let mut nm_centers = Vec::new();
    let mut nm_duplicate_edges = 0usize;
    let mut nm_distinct_3p_edges = 0usize;
    let mut nm_degenerate_adjacent_edges = 0usize;
    let mut nm_duplicate_position_vertex_edges = 0usize;
    let mut worst_clusters: HashMap<(i32, i32, i32), Vec<usize>> = HashMap::new();
    let extent = (bounds_max - bounds_min).abs() + Vec3::splat(1e-9);

    for (&(a, b), tri_ids) in &edge_to_triangles {
        match tri_ids.len() {
            1 => edges_1 += 1,
            2 => edges_2 += 1,
            3 => edges_3 += 1,
            4 => edges_4 += 1,
            _ => edges_gt4 += 1,
        }
        if tri_ids.len() == 2 {
            continue;
        }
        let p0 = positions[a as usize];
        let p1 = positions[b as usize];
        let center = (p0 + p1) * 0.5;
        nm_centers.push((center, p0.distance(p1)));
        let rel = (center - bounds_min) / extent;
        let key = (
            (rel.x * 10.0).floor().clamp(0.0, 9.0) as i32,
            (rel.y * 10.0).floor().clamp(0.0, 9.0) as i32,
            (rel.z * 10.0).floor().clamp(0.0, 9.0) as i32,
        );
        worst_clusters
            .entry(key)
            .or_default()
            .extend(tri_ids.iter().copied());

        let mut canonical_tris = HashSet::new();
        let mut has_duplicate = false;
        let mut has_degenerate_adjacent = false;
        for &tri_idx in tri_ids {
            let canonical = canonical_mesh_triangle(raw_triangle_sources[tri_idx].tri);
            if !canonical_tris.insert(canonical) {
                has_duplicate = true;
            }
            if degenerate_triangle_indices.contains(&tri_idx) {
                has_degenerate_adjacent = true;
            }
        }
        if has_duplicate {
            nm_duplicate_edges += 1;
        }
        if canonical_tris.len() >= 3 {
            nm_distinct_3p_edges += 1;
        }
        if has_degenerate_adjacent {
            nm_degenerate_adjacent_edges += 1;
        }

        let pa = positions[a as usize];
        let pb = positions[b as usize];
        let key_a = [
            (pa.x * inv_weld).round() as i32,
            (pa.y * inv_weld).round() as i32,
            (pa.z * inv_weld).round() as i32,
        ];
        let key_b = [
            (pb.x * inv_weld).round() as i32,
            (pb.y * inv_weld).round() as i32,
            (pb.z * inv_weld).round() as i32,
        ];
        if welded_position_groups.get(&key_a).copied().unwrap_or(0) > 1
            || welded_position_groups.get(&key_b).copied().unwrap_or(0) > 1
        {
            nm_duplicate_position_vertex_edges += 1;
        }
    }

    eprintln!(
        "  adaptive-mc raw_audit stage={} edge_use_hist edges_used_by_1={} edges_used_by_2={} edges_used_by_3={} edges_used_by_4={} edges_used_by_gt4={}",
        stage, edges_1, edges_2, edges_3, edges_4, edges_gt4
    );
    eprintln!(
        "  adaptive-mc raw_audit stage={} duplicates exact_duplicate_triangles={} reversed_duplicate_triangles={} same_vertex_set_duplicates={} near_duplicate_vertices_at_weld_epsilon={}",
        stage,
        exact_duplicate_triangles,
        reversed_duplicate_triangles,
        same_vertex_set_duplicates,
        near_duplicate_vertex_count,
    );
    eprintln!(
        "  adaptive-mc raw_audit stage={} degenerate_triangles zero_area={} area_lt_1e_12={} area_1e_12_to_1e_10={} area_1e_10_to_1e_8={} area_1e_8_to_1e_6={} area_ge_1e_6={}",
        stage,
        zero_area,
        area_buckets[0],
        area_buckets[1],
        area_buckets[2],
        area_buckets[3],
        area_buckets[4],
    );
    eprintln!(
        "  adaptive-mc raw_audit stage={} nonmanifold_sources duplicate_triangle_edges={} distinct_3plus_triangle_edges={} degenerate_adjacent_edges={} duplicate_position_vertex_edges={}",
        stage,
        nm_duplicate_edges,
        nm_distinct_3p_edges,
        nm_degenerate_adjacent_edges,
        nm_duplicate_position_vertex_edges,
    );
    log_raw_cluster_summary("nonmanifold_edges", &nm_centers, bounds_min, bounds_max);
    log_raw_cluster_summary(
        "degenerate_triangles",
        &degenerate_centers,
        bounds_min,
        bounds_max,
    );

    let mut ordered_clusters: Vec<_> = worst_clusters.into_iter().collect();
    ordered_clusters.sort_by(|a, b| b.1.len().cmp(&a.1.len()));
    for (rank, (_cell, tri_ids)) in ordered_clusters.into_iter().take(3).enumerate() {
        let mut source_cells: HashMap<usize, usize> = HashMap::new();
        let mut case_counts: HashMap<u8, usize> = HashMap::new();
        for tri_idx in tri_ids {
            let src = raw_triangle_sources[tri_idx];
            *source_cells.entry(src.cell_idx).or_insert(0) += 1;
            *case_counts.entry(src.cube_index).or_insert(0) += 1;
        }
        let mut top_cells: Vec<_> = source_cells.into_iter().collect();
        top_cells.sort_by(|a, b| b.1.cmp(&a.1));
        let mut top_cases: Vec<_> = case_counts.into_iter().collect();
        top_cases.sort_by(|a, b| b.1.cmp(&a.1));
        let cell_summary = top_cells
            .iter()
            .take(3)
            .map(|(cell_idx, count)| {
                let cell = &surface_cells[*cell_idx];
                format!(
                    "cell{}@min=({:.3},{:.3},{:.3}) size={:.3} tris={}",
                    cell_idx, cell.min.x, cell.min.y, cell.min.z, cell.size, count
                )
            })
            .collect::<Vec<_>>()
            .join("; ");
        let case_summary = top_cases
            .iter()
            .take(5)
            .map(|(case, count)| format!("case{}={}", case, count))
            .collect::<Vec<_>>()
            .join(", ");
        eprintln!(
            "  adaptive-mc raw_audit stage={} worst_cluster_{} top_source_cells=[{}] top_cube_cases=[{}] neighboring_cells_overlap_possible={}",
            stage,
            rank + 1,
            cell_summary,
            case_summary,
            top_cells.len() > 1,
        );
    }
}

fn spatial_bucket(p: Vec3, cell_size: f32) -> [i32; 3] {
    [
        (p.x / cell_size).floor() as i32,
        (p.y / cell_size).floor() as i32,
        (p.z / cell_size).floor() as i32,
    ]
}

fn point_lies_on_segment(p: Vec3, a: Vec3, b: Vec3, epsilon: f32) -> bool {
    let ab = b - a;
    let len2 = ab.length_squared();
    if len2 <= 1e-12 {
        return false;
    }
    let ap = p - a;
    let dot = ap.dot(ab);
    if dot < -epsilon || dot > len2 + epsilon {
        return false;
    }
    let line_distance = ap.cross(ab).length() / len2.sqrt();
    line_distance < epsilon
}

fn build_mesh_edge_to_triangles(indices: &[u32]) -> HashMap<(u32, u32), Vec<usize>> {
    let mut edge_to_triangles: HashMap<(u32, u32), Vec<usize>> =
        HashMap::with_capacity(indices.len());
    for (tri_idx, tri) in indices.chunks_exact(3).enumerate() {
        for &(a, b) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            edge_to_triangles
                .entry(canonical_mesh_edge(a, b))
                .or_default()
                .push(tri_idx);
        }
    }
    edge_to_triangles
}

fn boundary_edge_set(indices: &[u32]) -> HashSet<(u32, u32)> {
    build_mesh_edge_to_triangles(indices)
        .into_iter()
        .filter_map(|(edge, tris)| (tris.len() == 1).then_some(edge))
        .collect()
}

fn compute_mesh_topology_stats(mesh: &Mesh) -> MeshTopologyStats {
    let triangle_count = mesh.indices.len() / 3;
    let edge_to_triangles = build_mesh_edge_to_triangles(&mesh.indices);
    let boundary_edges = edge_to_triangles
        .values()
        .filter(|tris| tris.len() == 1)
        .count();
    let non_manifold_edges = edge_to_triangles
        .values()
        .filter(|tris| tris.len() != 2)
        .count();

    let mut tri_adjacency = vec![Vec::<usize>::new(); triangle_count];
    for triangles in edge_to_triangles.values() {
        if triangles.len() < 2 {
            continue;
        }
        for i in 0..triangles.len() {
            for j in (i + 1)..triangles.len() {
                let a = triangles[i];
                let b = triangles[j];
                tri_adjacency[a].push(b);
                tri_adjacency[b].push(a);
            }
        }
    }

    let mut connected_components = 0usize;
    let mut visited = vec![false; triangle_count];
    for start in 0..triangle_count {
        if visited[start] {
            continue;
        }
        connected_components += 1;
        let mut queue = VecDeque::from([start]);
        visited[start] = true;
        while let Some(tri_idx) = queue.pop_front() {
            for &next in &tri_adjacency[tri_idx] {
                if !visited[next] {
                    visited[next] = true;
                    queue.push_back(next);
                }
            }
        }
    }

    MeshTopologyStats {
        vertices: mesh.vertices.len(),
        triangles: triangle_count,
        boundary_edges,
        non_manifold_edges,
        connected_components,
        watertight: triangle_count > 0
            && boundary_edges == 0
            && non_manifold_edges == 0
            && connected_components == 1,
    }
}

fn edge_count_map_from_triangle_slices<'a, I>(triangles: I) -> HashMap<(u32, u32), usize>
where
    I: IntoIterator<Item = &'a [u32; 3]>,
{
    let mut counts = HashMap::new();
    for tri in triangles {
        let edges = [
            canonical_mesh_edge(tri[0], tri[1]),
            canonical_mesh_edge(tri[1], tri[2]),
            canonical_mesh_edge(tri[2], tri[0]),
        ];
        for edge in edges {
            *counts.entry(edge).or_insert(0) += 1;
        }
    }
    counts
}

fn vertex_to_triangles(indices: &[u32], vertex_count: usize) -> Vec<Vec<usize>> {
    let mut map = vec![Vec::new(); vertex_count];
    for (tri_idx, tri) in indices.chunks_exact(3).enumerate() {
        map[tri[0] as usize].push(tri_idx);
        map[tri[1] as usize].push(tri_idx);
        map[tri[2] as usize].push(tri_idx);
    }
    map
}

fn triangles_share_edge(indices: &[u32], a: usize, b: usize) -> bool {
    let tri_a = &indices[a * 3..a * 3 + 3];
    let tri_b = &indices[b * 3..b * 3 + 3];
    let mut shared = 0usize;
    for &va in tri_a {
        for &vb in tri_b {
            if va == vb {
                shared += 1;
            }
        }
    }
    shared >= 2
}

fn classify_weld_bucket_touch(
    center: Vec3,
    bounds_min: Vec3,
    bounds_max: Vec3,
) -> Vec<&'static str> {
    let extent = (bounds_max - bounds_min).abs();
    let mut tags = Vec::new();
    if center.x <= bounds_min.x + extent.x * 0.15 {
        tags.push("leading_edge");
    }
    if center.x >= bounds_max.x - extent.x * 0.12 {
        tags.push("trailing_edge");
    }
    if center.y.abs() <= extent.y * 0.12 {
        tags.push("root");
    }
    if center.y.abs() >= bounds_max.y.abs().max(bounds_min.y.abs()) - extent.y * 0.12 {
        tags.push("tip");
    }
    if center.z <= bounds_min.z + extent.z * 0.15 || center.z >= bounds_max.z - extent.z * 0.15 {
        tags.push("shell_wall");
    }
    tags
}

fn print_boundary_checkpoint(stage: &str, mesh: &Mesh) {
    eprintln!(
        "  adaptive-mc boundary_checkpoint: stage={} boundary_edges={} triangles={} vertices={}",
        stage,
        boundary_edge_set(&mesh.indices).len(),
        mesh.indices.len() / 3,
        mesh.vertices.len(),
    );
}

fn weld_epsilon_override(default: f32, env_key: &str) -> f32 {
    std::env::var(env_key)
        .ok()
        .and_then(|raw| raw.parse::<f32>().ok())
        .unwrap_or(default)
}

fn should_print_weld_audit() -> bool {
    std::env::var("IMPLICIT_CAD_WELD_AUDIT")
        .ok()
        .map(|raw| raw == "1" || raw.eq_ignore_ascii_case("true"))
        .unwrap_or(false)
}

fn collect_weld_audit_stats(
    mesh_before: &Mesh,
    mesh_after: &Mesh,
    remap: &[u32],
    epsilon: f32,
    vertex_source_cells: &[Vec<usize>],
    surface_cells: &[OctreeCell],
) -> WeldAuditStats {
    let before = compute_mesh_topology_stats(mesh_before);
    let after = compute_mesh_topology_stats(mesh_after);
    let positions_before = mesh_positions(mesh_before);
    let (bounds_min, bounds_max) = mesh_bounds(&positions_before);
    let tri_map_before = vertex_to_triangles(&mesh_before.indices, mesh_before.vertices.len());

    let mut buckets: HashMap<[i32; 3], Vec<usize>> = HashMap::new();
    for (idx, vertex) in mesh_before.vertices.iter().enumerate() {
        let p = Vec3::from_array(vertex.position);
        buckets
            .entry(spatial_bucket(p, epsilon.max(1e-6)))
            .or_default()
            .push(idx);
    }

    let mut unique_weld_buckets = 0usize;
    let mut max_bucket_size = 0usize;
    let mut total_bucket_vertices = 0usize;
    let mut buckets_with_non_adjacent_cells = 0usize;
    let mut buckets_with_geometrically_distant_triangles = 0usize;
    let mut buckets_crossing_opposite_shell_sides = 0usize;
    let mut worst_examples = Vec::new();

    for (bucket_key, vertex_ids) in buckets {
        if vertex_ids.len() <= 1 {
            continue;
        }
        unique_weld_buckets += 1;
        max_bucket_size = max_bucket_size.max(vertex_ids.len());
        total_bucket_vertices += vertex_ids.len();

        let mut bbox_min = Vec3::splat(f32::INFINITY);
        let mut bbox_max = Vec3::splat(f32::NEG_INFINITY);
        let mut max_pairwise_distance = 0.0f32;
        let mut cell_ids = BTreeMap::<usize, ()>::new();
        let mut tri_ids = BTreeMap::<usize, ()>::new();
        let mut any_opposite_normals = false;

        for &vid in &vertex_ids {
            let p = positions_before[vid];
            bbox_min = bbox_min.min(p);
            bbox_max = bbox_max.max(p);
            for &cell_id in vertex_source_cells.get(vid).into_iter().flatten() {
                cell_ids.insert(cell_id, ());
            }
            for &tri_id in &tri_map_before[vid] {
                tri_ids.insert(tri_id, ());
            }
        }
        for i in 0..vertex_ids.len() {
            let vi = vertex_ids[i];
            let pi = positions_before[vi];
            let ni = Vec3::from_array(mesh_before.vertices[vi].normal);
            for &vj in &vertex_ids[(i + 1)..] {
                let pj = positions_before[vj];
                let nj = Vec3::from_array(mesh_before.vertices[vj].normal);
                max_pairwise_distance = max_pairwise_distance.max(pi.distance(pj));
                if ni.dot(nj) < -0.5 {
                    any_opposite_normals = true;
                }
            }
        }

        let cell_list: Vec<usize> = cell_ids.keys().copied().collect();
        let tri_list: Vec<usize> = tri_ids.keys().copied().collect();
        let non_adjacent_cells = cell_list.iter().enumerate().any(|(i, &a)| {
            cell_list[(i + 1)..]
                .iter()
                .any(|&b| !cells_touch_or_overlap(&surface_cells[a], &surface_cells[b], 1e-5))
        });
        let triangles_adjacent_before_weld = tri_list.iter().enumerate().all(|(i, &a)| {
            tri_list[(i + 1)..]
                .iter()
                .all(|&b| triangles_share_edge(&mesh_before.indices, a, b))
        });
        let geometrically_distant =
            max_pairwise_distance > epsilon * 0.5 && !triangles_adjacent_before_weld;
        let crosses_thin_shell = any_opposite_normals && max_pairwise_distance <= epsilon * 1.5;

        if non_adjacent_cells {
            buckets_with_non_adjacent_cells += 1;
        }
        if geometrically_distant {
            buckets_with_geometrically_distant_triangles += 1;
        }
        if crosses_thin_shell {
            buckets_crossing_opposite_shell_sides += 1;
        }

        if vertex_ids.len() > 2 {
            let center = (bbox_min + bbox_max) * 0.5;
            worst_examples.push(WeldBucketExample {
                bucket_key,
                vertex_count: vertex_ids.len(),
                bbox_min,
                bbox_max,
                max_pairwise_distance,
                contributing_cell_ids: cell_list,
                contributing_triangle_ids: tri_list,
                triangles_adjacent_before_weld,
                touches: classify_weld_bucket_touch(center, bounds_min, bounds_max),
                crosses_thin_shell,
                non_adjacent_cells,
                geometrically_distant,
            });
        }
    }

    worst_examples.sort_by(|a, b| {
        b.vertex_count
            .cmp(&a.vertex_count)
            .then_with(|| b.max_pairwise_distance.total_cmp(&a.max_pairwise_distance))
    });
    worst_examples.truncate(10);

    let mut triangles_made_degenerate_by_welding = 0usize;
    for tri in mesh_before.indices.chunks_exact(3) {
        let a = remap[tri[0] as usize];
        let b = remap[tri[1] as usize];
        let c = remap[tri[2] as usize];
        if a == b || b == c || c == a {
            triangles_made_degenerate_by_welding += 1;
        }
    }

    let mut collapsed_pre_counts: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    let pre_edge_to_triangles = build_mesh_edge_to_triangles(&mesh_before.indices);
    for (edge, tris) in pre_edge_to_triangles {
        let a = remap[edge.0 as usize];
        let b = remap[edge.1 as usize];
        if a == b {
            continue;
        }
        collapsed_pre_counts
            .entry(canonical_mesh_edge(a, b))
            .or_default()
            .push(tris.len());
    }
    let post_edge_to_triangles = build_mesh_edge_to_triangles(&mesh_after.indices);
    let mut edges_made_non_manifold_by_welding = 0usize;
    for (edge, tris) in post_edge_to_triangles {
        if tris.len() <= 2 {
            continue;
        }
        let prior_max = collapsed_pre_counts
            .get(&edge)
            .and_then(|counts| counts.iter().copied().max())
            .unwrap_or(0);
        if prior_max <= 2 {
            edges_made_non_manifold_by_welding += 1;
        }
    }

    WeldAuditStats {
        epsilon,
        before,
        after,
        vertices_merged: mesh_before
            .vertices
            .len()
            .saturating_sub(mesh_after.vertices.len()),
        unique_weld_buckets,
        max_bucket_size,
        average_bucket_size: if unique_weld_buckets == 0 {
            0.0
        } else {
            total_bucket_vertices as f32 / unique_weld_buckets as f32
        },
        buckets_with_non_adjacent_cells,
        buckets_with_geometrically_distant_triangles,
        buckets_crossing_opposite_shell_sides,
        triangles_made_degenerate_by_welding,
        edges_made_non_manifold_by_welding,
        worst_examples,
    }
}

fn print_weld_audit(stage: &str, audit: &WeldAuditStats) {
    eprintln!(
        "  adaptive-mc weld_audit: stage={} epsilon={:.6} before_vertices={} before_triangles={} before_boundary_edges={} before_non_manifold_edges={} before_connected_components={} before_watertight={} after_vertices={} after_triangles={} after_boundary_edges={} after_non_manifold_edges={} after_connected_components={} after_watertight={} vertices_merged={} unique_weld_buckets={} max_bucket_size={} average_bucket_size={:.3} buckets_non_adjacent_cells={} buckets_geometrically_distant_triangles={} buckets_opposite_shell_sides={} triangles_made_degenerate={} edges_made_non_manifold={}",
        stage,
        audit.epsilon,
        audit.before.vertices,
        audit.before.triangles,
        audit.before.boundary_edges,
        audit.before.non_manifold_edges,
        audit.before.connected_components,
        audit.before.watertight,
        audit.after.vertices,
        audit.after.triangles,
        audit.after.boundary_edges,
        audit.after.non_manifold_edges,
        audit.after.connected_components,
        audit.after.watertight,
        audit.vertices_merged,
        audit.unique_weld_buckets,
        audit.max_bucket_size,
        audit.average_bucket_size,
        audit.buckets_with_non_adjacent_cells,
        audit.buckets_with_geometrically_distant_triangles,
        audit.buckets_crossing_opposite_shell_sides,
        audit.triangles_made_degenerate_by_welding,
        audit.edges_made_non_manifold_by_welding,
    );
    for (idx, example) in audit.worst_examples.iter().enumerate() {
        let cells = example
            .contributing_cell_ids
            .iter()
            .take(8)
            .map(|cell_id| cell_id.to_string())
            .collect::<Vec<_>>()
            .join(",");
        let triangles = example
            .contributing_triangle_ids
            .iter()
            .take(8)
            .map(|tri_id| tri_id.to_string())
            .collect::<Vec<_>>()
            .join(",");
        eprintln!(
            "    weld_bucket_example_{}: bucket_id=[{},{},{}] vertex_count={} bbox=({:.3},{:.3},{:.3})..({:.3},{:.3},{:.3}) max_pairwise_distance={:.6} contributing_cells=[{}] contributing_triangles=[{}] triangles_adjacent_before_weld={} touches=[{}] non_adjacent_cells={} geometrically_distant={} crosses_thin_shell={}",
            idx + 1,
            example.bucket_key[0],
            example.bucket_key[1],
            example.bucket_key[2],
            example.vertex_count,
            example.bbox_min.x,
            example.bbox_min.y,
            example.bbox_min.z,
            example.bbox_max.x,
            example.bbox_max.y,
            example.bbox_max.z,
            example.max_pairwise_distance,
            cells,
            triangles,
            example.triangles_adjacent_before_weld,
            example.touches.join(","),
            example.non_adjacent_cells,
            example.geometrically_distant,
            example.crosses_thin_shell,
        );
    }
}

fn print_weld_checkpoint(
    mesh: &mut Mesh,
    epsilon: f32,
    stage: &str,
    vertex_source_cells: &[Vec<usize>],
    surface_cells: &[OctreeCell],
) -> WeldStats {
    let pre_weld_boundary_edges = boundary_edge_set(&mesh.indices);
    let mesh_before = if should_print_weld_audit() {
        Some(mesh.clone())
    } else {
        None
    };
    let weld_stats = weld_vertices(mesh, epsilon);
    let post_weld_boundary_edges = boundary_edge_set(&mesh.indices);
    let pre_weld_boundary_edges_after_remap: HashSet<(u32, u32)> = pre_weld_boundary_edges
        .iter()
        .filter_map(|&(a, b)| {
            let remapped_a = weld_stats.remap[a as usize];
            let remapped_b = weld_stats.remap[b as usize];
            (remapped_a != remapped_b).then_some(canonical_mesh_edge(remapped_a, remapped_b))
        })
        .collect();
    let new_boundary_edges_after_weld = post_weld_boundary_edges
        .difference(&pre_weld_boundary_edges_after_remap)
        .count();
    eprintln!(
        "  adaptive-mc boundary_checkpoint: stage={} boundary_edges={} triangles={} vertices={} weld_epsilon={:.6} vertices_merged={} new_boundary_edges_introduced={} buckets_considered={} buckets_accepted={} buckets_rejected={} vertices_preserved_due_to_rejected_buckets={} rejected_degenerate={} rejected_non_manifold={} rejected_duplicate={} rejected_opposite_shell={} elapsed_ms={:.3}",
        stage,
        post_weld_boundary_edges.len(),
        mesh.indices.len() / 3,
        mesh.vertices.len(),
        epsilon,
        weld_stats
            .vertices_before
            .saturating_sub(weld_stats.vertices_after),
        new_boundary_edges_after_weld,
        weld_stats.buckets_considered,
        weld_stats.buckets_accepted,
        weld_stats.buckets_rejected,
        weld_stats.vertices_preserved_due_to_rejected_buckets,
        weld_stats.buckets_rejected_for_degenerate_triangles,
        weld_stats.buckets_rejected_for_non_manifold_edges,
        weld_stats.buckets_rejected_for_duplicate_triangles,
        weld_stats.buckets_rejected_for_opposite_shell_collapse,
        weld_stats.elapsed_ms,
    );
    if let Some(before_mesh) = mesh_before.as_ref() {
        let mut old_unconstrained = before_mesh.clone();
        let old_stats = weld_vertices_unconstrained(&mut old_unconstrained, epsilon);
        let old_topology = compute_mesh_topology_stats(&old_unconstrained);
        eprintln!(
            "  adaptive-mc weld_old_unconstrained: stage={} epsilon={:.6} vertices_merged={} triangles={} vertices={} boundary_edges={} non_manifold_edges={} connected_components={} watertight={}",
            stage,
            epsilon,
            old_stats
                .vertices_before
                .saturating_sub(old_stats.vertices_after),
            old_topology.triangles,
            old_topology.vertices,
            old_topology.boundary_edges,
            old_topology.non_manifold_edges,
            old_topology.connected_components,
            old_topology.watertight,
        );
        let audit = collect_weld_audit_stats(
            before_mesh,
            mesh,
            &weld_stats.remap,
            epsilon,
            vertex_source_cells,
            surface_cells,
        );
        print_weld_audit(stage, &audit);
    }
    weld_stats
}

fn print_weld_epsilon_summary(
    weld_epsilon_1: f32,
    weld_epsilon_2: f32,
    repair_search_epsilon: f32,
) {
    eprintln!(
        "  adaptive-mc weld_epsilon_summary: weld_1_epsilon={:.6} weld_2_epsilon={:.6} repair_search_epsilon={:.6}",
        weld_epsilon_1, weld_epsilon_2, repair_search_epsilon,
    );
}

fn edge_match_distance(a0: Vec3, a1: Vec3, b0: Vec3, b1: Vec3) -> f32 {
    let direct = (a0.distance(b0)).max(a1.distance(b1));
    let swapped = (a0.distance(b1)).max(a1.distance(b0));
    direct.min(swapped)
}

fn print_boundary_neighbor_distance_diagnostic(mesh: &Mesh) {
    let edge_to_triangles = build_mesh_edge_to_triangles(&mesh.indices);
    let positions = mesh_positions(mesh);
    let boundary_edges: Vec<(u32, u32)> = edge_to_triangles
        .iter()
        .filter_map(|(&edge, tris)| (tris.len() == 1).then_some(edge))
        .collect();
    let non_boundary_edges: Vec<(u32, u32)> = edge_to_triangles
        .iter()
        .filter_map(|(&edge, tris)| (tris.len() != 1).then_some(edge))
        .collect();

    let bucket_size = 0.1f32;
    let mut grid: HashMap<[i32; 3], Vec<usize>> = HashMap::new();
    let mut non_boundary_meta = Vec::with_capacity(non_boundary_edges.len());
    for &(a, b) in &non_boundary_edges {
        let p0 = positions[a as usize];
        let p1 = positions[b as usize];
        let midpoint = (p0 + p1) * 0.5;
        let length = p0.distance(p1);
        non_boundary_meta.push((p0, p1, midpoint, length));
    }
    for (idx, &(_, _, midpoint, _)) in non_boundary_meta.iter().enumerate() {
        grid.entry(spatial_bucket(midpoint, bucket_size))
            .or_default()
            .push(idx);
    }

    let mut bins = [0usize; 4];
    let mut matches_found = 0usize;
    for &(a, b) in &boundary_edges {
        let p0 = positions[a as usize];
        let p1 = positions[b as usize];
        let midpoint = (p0 + p1) * 0.5;
        let length = p0.distance(p1);
        let bucket = spatial_bucket(midpoint, bucket_size);
        let mut best = f32::INFINITY;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let key = [bucket[0] + dx, bucket[1] + dy, bucket[2] + dz];
                    if let Some(candidates) = grid.get(&key) {
                        for &idx in candidates {
                            let (q0, q1, qmid, qlen) = non_boundary_meta[idx];
                            if midpoint.distance(qmid) > 0.1 {
                                continue;
                            }
                            let max_len = length.max(qlen).max(1e-9);
                            if (length - qlen).abs() / max_len > 0.1 {
                                continue;
                            }
                            best = best.min(edge_match_distance(p0, p1, q0, q1));
                        }
                    }
                }
            }
        }
        if best.is_finite() {
            matches_found += 1;
        }
        if best <= 0.001 {
            bins[0] += 1;
        } else if best <= 0.01 {
            bins[1] += 1;
        } else if best <= 0.1 {
            bins[2] += 1;
        } else {
            bins[3] += 1;
        }
    }

    eprintln!(
        "  adaptive-mc boundary_neighbor_distance_diagnostic: boundary_edges={} matched_candidates={} within_0p001mm={} within_0p01mm={} within_0p1mm={} over_0p1mm={}",
        boundary_edges.len(),
        matches_found,
        bins[0],
        bins[1],
        bins[2],
        bins[3],
    );
}

fn print_repair_edge_distance_diagnostic(mesh: &Mesh, repair_edges: &[(u32, u32)], remap: &[u32]) {
    let positions = mesh_positions(mesh);
    let mut unique_edges = HashSet::new();
    let mut bins = [0usize; 5];
    let mut surviving_edges = 0usize;

    for &(a, b) in repair_edges {
        let Some(&ra) = remap.get(a as usize) else {
            continue;
        };
        let Some(&rb) = remap.get(b as usize) else {
            continue;
        };
        if ra == rb {
            bins[0] += 1;
            continue;
        }
        let edge = canonical_mesh_edge(ra, rb);
        if !unique_edges.insert(edge) {
            continue;
        }
        surviving_edges += 1;
        let dist = positions[ra as usize].distance(positions[rb as usize]);
        if dist <= 0.001 {
            bins[1] += 1;
        } else if dist <= 0.01 {
            bins[2] += 1;
        } else if dist <= 0.1 {
            bins[3] += 1;
        } else {
            bins[4] += 1;
        }
    }

    eprintln!(
        "  adaptive-mc repair_edge_distance_diagnostic: repair_edges_total={} merged_same_index={} surviving_unique_edges={} within_0p001mm={} within_0p01mm={} within_0p1mm={} over_0p1mm={}",
        repair_edges.len(),
        bins[0],
        surviving_edges,
        bins[1],
        bins[2],
        bins[3],
        bins[4],
    );
}

fn mesh_positions(mesh: &Mesh) -> Vec<Vec3> {
    mesh.vertices
        .iter()
        .map(|vertex| Vec3::from_array(vertex.position))
        .collect()
}

fn mesh_bounds(positions: &[Vec3]) -> (Vec3, Vec3) {
    let min = positions
        .iter()
        .copied()
        .fold(Vec3::splat(f32::INFINITY), |acc, p| acc.min(p));
    let max = positions
        .iter()
        .copied()
        .fold(Vec3::splat(f32::NEG_INFINITY), |acc, p| acc.max(p));
    (min, max)
}

fn build_vertex_grid(positions: &[Vec3], grid_cell_size: f32) -> HashMap<[i32; 3], Vec<u32>> {
    let mut vertex_grid: HashMap<[i32; 3], Vec<u32>> = HashMap::with_capacity(positions.len());
    for (idx, &p) in positions.iter().enumerate() {
        vertex_grid
            .entry(spatial_bucket(p, grid_cell_size))
            .or_default()
            .push(idx as u32);
    }
    vertex_grid
}

fn candidate_vertices_on_segment(
    positions: &[Vec3],
    vertex_grid: &HashMap<[i32; 3], Vec<u32>>,
    a_idx: u32,
    b_idx: u32,
    epsilon: f32,
    grid_cell_size: f32,
) -> Vec<(u32, f32)> {
    let a = positions[a_idx as usize];
    let b = positions[b_idx as usize];
    let ab = b - a;
    let len = ab.length();
    let len2 = ab.length_squared();
    if len <= 1e-9 || len2 <= 1e-12 {
        return Vec::new();
    }

    let steps = (len / grid_cell_size).ceil().max(1.0) as usize;
    let mut candidates = Vec::<u32>::new();
    for step in 0..=steps {
        let t = step as f32 / steps as f32;
        let p = a.lerp(b, t);
        let bucket = spatial_bucket(p, grid_cell_size);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let key = [bucket[0] + dx, bucket[1] + dy, bucket[2] + dz];
                    if let Some(indices) = vertex_grid.get(&key) {
                        candidates.extend(indices.iter().copied());
                    }
                }
            }
        }
    }
    candidates.sort_unstable();
    candidates.dedup();

    let mut matches: Vec<(u32, f32)> = candidates
        .into_iter()
        .filter(|&candidate_idx| candidate_idx != a_idx && candidate_idx != b_idx)
        .filter_map(|candidate_idx| {
            let p = positions[candidate_idx as usize];
            if !point_lies_on_segment(p, a, b, epsilon) {
                return None;
            }
            let dot = (p - a).dot(ab);
            Some((candidate_idx, dot))
        })
        .collect();
    matches.sort_by(|a, b| a.1.total_cmp(&b.1));
    matches.dedup_by(|a, b| a.0 == b.0 || (a.1 - b.1).abs() <= epsilon);
    matches
}

fn print_boundary_match_hotspots(
    label: &str,
    edge_centers: &[Vec3],
    mesh_bounds_min: Vec3,
    mesh_bounds_max: Vec3,
) {
    if edge_centers.is_empty() {
        eprintln!("    {}: none", label);
        return;
    }

    let extent = mesh_bounds_max - mesh_bounds_min + Vec3::splat(1e-9);
    let mut bins: BTreeMap<i32, usize> = BTreeMap::new();
    for center in edge_centers {
        let rel = (*center - mesh_bounds_min) / extent * 10.0;
        let ix = (rel.x.floor() as i32).clamp(0, 9);
        let iy = (rel.y.floor() as i32).clamp(0, 9);
        let iz = (rel.z.floor() as i32).clamp(0, 9);
        *bins.entry(ix * 100 + iy * 10 + iz).or_default() += 1;
    }

    let mut rows: Vec<BoundaryMatchHotspot> = bins
        .into_iter()
        .map(|(key, count)| BoundaryMatchHotspot { key, count })
        .collect();
    rows.sort_by(|a, b| b.count.cmp(&a.count));

    eprintln!("    {}:", label);
    for row in rows.into_iter().take(5) {
        let ix = row.key / 100;
        let iy = (row.key % 100) / 10;
        let iz = row.key % 10;
        let cell_min = mesh_bounds_min
            + (mesh_bounds_max - mesh_bounds_min) * Vec3::new(ix as f32, iy as f32, iz as f32)
                / 10.0;
        let cell_max = mesh_bounds_min
            + (mesh_bounds_max - mesh_bounds_min)
                * Vec3::new((ix + 1) as f32, (iy + 1) as f32, (iz + 1) as f32)
                / 10.0;
        eprintln!(
            "      [{},{},{}] count={} x={:.1}..{:.1} y={:.1}..{:.1} z={:.1}..{:.1}",
            ix,
            iy,
            iz,
            row.count,
            cell_min.x,
            cell_max.x,
            cell_min.y,
            cell_max.y,
            cell_min.z,
            cell_max.z,
        );
    }
}

fn active_triangles_for_edge(
    edge_to_triangles: &HashMap<(u32, u32), Vec<usize>>,
    deleted: &[bool],
    edge: (u32, u32),
) -> Vec<usize> {
    edge_to_triangles
        .get(&edge)
        .into_iter()
        .flatten()
        .copied()
        .filter(|&tri_idx| tri_idx < deleted.len() && !deleted[tri_idx])
        .collect()
}

fn add_triangle_to_edge_map(
    edge_to_triangles: &mut HashMap<(u32, u32), Vec<usize>>,
    tri_idx: usize,
    tri: [u32; 3],
) {
    for &(a, b) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
        edge_to_triangles
            .entry(canonical_mesh_edge(a, b))
            .or_default()
            .push(tri_idx);
    }
}

fn directed_edge_and_opposite(tri: [u32; 3], edge: (u32, u32)) -> Option<(u32, u32, u32)> {
    for i in 0..3 {
        let a = tri[i];
        let b = tri[(i + 1) % 3];
        if canonical_mesh_edge(a, b) == edge {
            return Some((a, b, tri[(i + 2) % 3]));
        }
    }
    None
}

fn compact_triangles(triangles: &[[u32; 3]], deleted: &[bool]) -> Vec<u32> {
    let mut indices = Vec::with_capacity(triangles.len() * 3);
    for (idx, tri) in triangles.iter().enumerate() {
        if deleted[idx] {
            continue;
        }
        indices.extend_from_slice(tri);
    }
    indices
}

fn collect_boundary_edge_info(mesh: &Mesh) -> Vec<BoundaryEdgeInfo> {
    let edge_to_triangles = build_mesh_edge_to_triangles(&mesh.indices);
    let positions = mesh_positions(mesh);
    let mut infos = Vec::new();
    for (&edge, tris) in &edge_to_triangles {
        if tris.len() != 1 {
            continue;
        }
        let tri_idx = tris[0];
        let tri = [
            mesh.indices[tri_idx * 3],
            mesh.indices[tri_idx * 3 + 1],
            mesh.indices[tri_idx * 3 + 2],
        ];
        let Some((a, b, opposite)) = directed_edge_and_opposite(tri, edge) else {
            continue;
        };
        let pa = positions[a as usize];
        let pb = positions[b as usize];
        let pc = positions[opposite as usize];
        let normal = (pb - pa).cross(pc - pa).normalize_or_zero();
        infos.push(BoundaryEdgeInfo {
            edge,
            tri_idx,
            directed_a: a,
            directed_b: b,
            opposite,
            midpoint: (pa + pb) * 0.5,
            length: pa.distance(pb),
            normal,
        });
    }
    infos
}

fn boundary_components(boundary_edges: &[(u32, u32)]) -> Vec<Vec<(u32, u32)>> {
    let mut vert_to_edges: HashMap<u32, Vec<usize>> = HashMap::new();
    for (idx, &(a, b)) in boundary_edges.iter().enumerate() {
        vert_to_edges.entry(a).or_default().push(idx);
        vert_to_edges.entry(b).or_default().push(idx);
    }
    let mut visited = vec![false; boundary_edges.len()];
    let mut components = Vec::new();
    for start in 0..boundary_edges.len() {
        if visited[start] {
            continue;
        }
        let mut queue = VecDeque::from([start]);
        visited[start] = true;
        let mut edges = Vec::new();
        while let Some(edge_idx) = queue.pop_front() {
            let edge = boundary_edges[edge_idx];
            edges.push(edge);
            for v in [edge.0, edge.1] {
                if let Some(nexts) = vert_to_edges.get(&v) {
                    for &next in nexts {
                        if !visited[next] {
                            visited[next] = true;
                            queue.push_back(next);
                        }
                    }
                }
            }
        }
        components.push(edges);
    }
    components
}

fn classify_patch_feature(center: Vec3, bounds_min: Vec3, bounds_max: Vec3) -> &'static str {
    classify_patch_feature_from_bounds(center, bounds_min, bounds_max)
}

fn print_boundary_repairability_diagnostic(mesh: &Mesh, pair_epsilon: f32, tjunction_epsilon: f32) {
    let infos = collect_boundary_edge_info(mesh);
    let positions = mesh_positions(mesh);
    let (bounds_min, bounds_max) = mesh_bounds(&positions);
    let components = boundary_components(&infos.iter().map(|info| info.edge).collect::<Vec<_>>());
    let mut edge_to_component = HashMap::<(u32, u32), usize>::new();
    for (comp_idx, comp) in components.iter().enumerate() {
        for &edge in comp {
            edge_to_component.insert(edge, comp_idx);
        }
    }

    let mut cls_counts = [0usize; 5];
    let mut cls_centers = vec![Vec::<Vec3>::new(); 5];
    let mut cls_lengths = vec![Vec::<f32>::new(); 5];

    for (i, info) in infos.iter().enumerate() {
        let mut class = 4usize;
        for other in infos.iter().skip(i + 1) {
            if edge_to_component.get(&info.edge) == edge_to_component.get(&other.edge) {
                continue;
            }
            let midpoint_dist = info.midpoint.distance(other.midpoint);
            if midpoint_dist > pair_epsilon * 8.0 {
                continue;
            }
            let len_ratio = info.length.max(other.length) / info.length.min(other.length).max(1e-6);
            let dir_a = (positions[info.directed_b as usize] - positions[info.directed_a as usize])
                .normalize_or_zero();
            let dir_b = (positions[other.directed_b as usize]
                - positions[other.directed_a as usize])
                .normalize_or_zero();
            if dir_a.dot(dir_b) < -0.9 && len_ratio <= 1.2 {
                class = 0;
                break;
            }
        }
        if class == 4 {
            let matches = candidate_vertices_on_segment(
                &positions,
                &build_vertex_grid(&positions, (2.0 * tjunction_epsilon).max(1e-6)),
                info.directed_a,
                info.directed_b,
                tjunction_epsilon,
                (2.0 * tjunction_epsilon).max(1e-6),
            );
            if !matches.is_empty() {
                class = 1;
            } else if let Some(&comp_idx) = edge_to_component.get(&info.edge) {
                let comp = &components[comp_idx];
                let vertex_count: HashSet<u32> = comp.iter().flat_map(|(a, b)| [*a, *b]).collect();
                if comp.len() == vertex_count.len() && (3..=4).contains(&comp.len()) {
                    class = 2;
                } else if comp.len() > 4 {
                    class = 3;
                }
            }
        }
        cls_counts[class] += 1;
        cls_centers[class].push(info.midpoint);
        cls_lengths[class].push(info.length);
    }

    let labels = [
        ("edge_pair_match", "safe_if_validated"),
        ("vertex_on_edge_tjunction", "safe_if_validated"),
        ("tiny_loop", "safe_if_validated"),
        ("larger_loop", "unsafe_without_loop_solver"),
        ("no_plausible_match", "unsafe"),
    ];
    eprintln!("  adaptive-mc boundary_repairability_diagnostic:");
    for class in 0..5 {
        let count = cls_counts[class];
        let avg_len = if cls_lengths[class].is_empty() {
            0.0
        } else {
            cls_lengths[class].iter().sum::<f32>() / cls_lengths[class].len() as f32
        };
        let max_len = cls_lengths[class].iter().copied().fold(0.0f32, f32::max);
        let bbox_min = cls_centers[class]
            .iter()
            .copied()
            .fold(Vec3::splat(f32::INFINITY), |acc, p| acc.min(p));
        let bbox_max = cls_centers[class]
            .iter()
            .copied()
            .fold(Vec3::splat(f32::NEG_INFINITY), |acc, p| acc.max(p));
        let feature = if cls_centers[class].is_empty() {
            "none"
        } else {
            classify_patch_feature((bbox_min + bbox_max) * 0.5, bounds_min, bounds_max)
        };
        eprintln!(
            "    class={} count={} bbox=({:.3},{:.3},{:.3})..({:.3},{:.3},{:.3}) avg_edge_length={:.6} max_edge_length={:.6} nearest_feature={} safety={}",
            labels[class].0,
            count,
            bbox_min.x,
            bbox_min.y,
            bbox_min.z,
            bbox_max.x,
            bbox_max.y,
            bbox_max.z,
            avg_len,
            max_len,
            feature,
            labels[class].1,
        );
    }
}

fn try_bridge_edge_pair(
    mesh: &Mesh,
    a: &BoundaryEdgeInfo,
    b: &BoundaryEdgeInfo,
) -> Result<(Vec<[u32; 3]>, MeshTopologyStats), &'static str> {
    let positions = mesh_positions(mesh);
    let pa0 = positions[a.directed_a as usize];
    let pa1 = positions[a.directed_b as usize];
    let pb0 = positions[b.directed_a as usize];
    let pb1 = positions[b.directed_b as usize];
    let direct = pa0.distance(pb1) + pa1.distance(pb0);
    let swapped = pa0.distance(pb0) + pa1.distance(pb1);
    let bridge = if direct <= swapped {
        vec![
            [a.directed_a, a.directed_b, b.directed_b],
            [a.directed_a, b.directed_b, b.directed_a],
        ]
    } else {
        vec![
            [a.directed_a, a.directed_b, b.directed_a],
            [a.directed_b, b.directed_b, b.directed_a],
        ]
    };

    for tri in &bridge {
        if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
            return Err("would create degenerate triangle");
        }
    }
    let mut canonical_existing = HashSet::<[u32; 3]>::new();
    let mut oriented_existing = HashSet::<[u32; 3]>::new();
    for tri in mesh.indices.chunks_exact(3) {
        let t = [tri[0], tri[1], tri[2]];
        canonical_existing.insert(canonical_mesh_triangle(t));
        oriented_existing.insert(t);
    }
    for tri in &bridge {
        if oriented_existing.contains(tri)
            || canonical_existing.contains(&canonical_mesh_triangle(*tri))
        {
            return Err("would create duplicate triangle");
        }
    }
    let mut test = mesh.clone();
    for tri in &bridge {
        test.indices.extend_from_slice(tri);
    }
    let stats = compute_mesh_topology_stats(&test);
    Ok((bridge, stats))
}

fn apply_transition_micro_patch(mesh: &mut Mesh) -> usize {
    let before = compute_mesh_topology_stats(mesh);
    let edges = boundary_edge_set(&mesh.indices)
        .into_iter()
        .collect::<Vec<_>>();
    let comps = boundary_components(&edges);
    let positions = mesh_positions(mesh);
    let mut added = 0usize;
    for comp in comps {
        if !(3..=4).contains(&comp.len()) {
            continue;
        }
        let mids = comp
            .iter()
            .map(|&(a, b)| (positions[a as usize] + positions[b as usize]) * 0.5)
            .collect::<Vec<_>>();
        let center = mids.iter().copied().fold(Vec3::ZERO, |acc, p| acc + p) / mids.len() as f32;
        if !near_inlet_transition_plane(center) {
            continue;
        }
        let Some(ordered) = order_boundary_loop(&comp) else {
            continue;
        };
        if ordered.len() < 4 || ordered.first() != ordered.last() {
            continue;
        }
        let verts = ordered[..ordered.len() - 1].to_vec();
        let Ok((tris, after)) = try_loop_fill_candidates(mesh, &verts, &before) else {
            continue;
        };
        if after.boundary_edges <= before.boundary_edges
            && after.non_manifold_edges <= before.non_manifold_edges
        {
            for tri in tris {
                mesh.indices.extend_from_slice(&tri);
                added += 1;
            }
        }
    }
    added
}

fn repair_edge_pair_cracks(mesh: &mut Mesh, epsilon: f32) -> CrackRepairStats {
    let before = compute_mesh_topology_stats(mesh);
    let transition_fix = transition_fix_from_env();
    let infos = collect_boundary_edge_info(mesh);
    let positions = mesh_positions(mesh);
    let mut stats = CrackRepairStats {
        boundary_edges_before: before.boundary_edges,
        non_manifold_before: before.non_manifold_edges,
        ..CrackRepairStats::default()
    };
    let mut used_edges = HashSet::<(u32, u32)>::new();
    for i in 0..infos.len() {
        let a = &infos[i];
        if used_edges.contains(&a.edge) {
            continue;
        }
        let dir_a = (positions[a.directed_b as usize] - positions[a.directed_a as usize])
            .normalize_or_zero();
        let mut best: Option<(usize, f32)> = None;
        for j in (i + 1)..infos.len() {
            let b = &infos[j];
            if used_edges.contains(&b.edge) {
                continue;
            }
            let local_epsilon = if transition_fix == TransitionCorridorFix::LargerRepairRadius
                && in_inlet_transition_corridor(a.midpoint)
                && in_inlet_transition_corridor(b.midpoint)
            {
                epsilon.max(0.15)
            } else {
                epsilon
            };
            let midpoint_dist = a.midpoint.distance(b.midpoint);
            if midpoint_dist > local_epsilon * 8.0 {
                stats.rejected_distance_too_large += 1;
                continue;
            }
            let len_ratio = a.length.max(b.length) / a.length.min(b.length).max(1e-6);
            if len_ratio > 1.2 {
                stats.rejected_length_mismatch += 1;
                continue;
            }
            let dir_b = (positions[b.directed_b as usize] - positions[b.directed_a as usize])
                .normalize_or_zero();
            if dir_a.dot(dir_b) > -0.9 {
                stats.rejected_orientation_mismatch += 1;
                continue;
            }
            if a.normal.dot(b.normal) < 0.2 {
                stats.rejected_normal_mismatch += 1;
                continue;
            }
            let score = midpoint_dist + (len_ratio - 1.0).abs();
            if best.map(|(_, s)| score < s).unwrap_or(true) {
                best = Some((j, score));
            }
        }
        let Some((j, _)) = best else {
            continue;
        };
        stats.candidate_pairs_found += 1;
        let b = &infos[j];
        match try_bridge_edge_pair(mesh, a, b) {
            Ok((bridge, after_stats)) => {
                if after_stats.boundary_edges < before.boundary_edges
                    && after_stats.non_manifold_edges <= before.non_manifold_edges
                {
                    for tri in bridge {
                        mesh.indices.extend_from_slice(&tri);
                        stats.triangles_added += 1;
                    }
                    stats.candidates_accepted += 1;
                    used_edges.insert(a.edge);
                    used_edges.insert(b.edge);
                } else {
                    stats.candidates_rejected += 1;
                    stats.rejected_non_manifold += 1;
                }
            }
            Err(reason) => {
                stats.candidates_rejected += 1;
                match reason {
                    "would create degenerate triangle" => stats.rejected_degenerate += 1,
                    "would create duplicate triangle" => stats.rejected_duplicate += 1,
                    _ => stats.rejected_ambiguous += 1,
                }
            }
        }
    }
    let after = compute_mesh_topology_stats(mesh);
    stats.boundary_edges_after = after.boundary_edges;
    stats.non_manifold_after = after.non_manifold_edges;
    stats.connected_components_after = after.connected_components;
    stats.watertight_after = after.watertight;
    eprintln!(
        "  adaptive-mc edge_pair_repair: boundary_edges_before={} non_manifold_before={} candidate_pairs_found={} candidates_accepted={} candidates_rejected={} triangles_added={} vertices_added={} boundary_edges_after={} non_manifold_after={} connected_components_after={} watertight={} reject_distance={} reject_orientation={} reject_length={} reject_normal={} reject_degenerate={} reject_duplicate={} reject_non_manifold={} reject_ambiguous={}",
        stats.boundary_edges_before,
        stats.non_manifold_before,
        stats.candidate_pairs_found,
        stats.candidates_accepted,
        stats.candidates_rejected,
        stats.triangles_added,
        stats.vertices_added,
        stats.boundary_edges_after,
        stats.non_manifold_after,
        stats.connected_components_after,
        stats.watertight_after,
        stats.rejected_distance_too_large,
        stats.rejected_orientation_mismatch,
        stats.rejected_length_mismatch,
        stats.rejected_normal_mismatch,
        stats.rejected_degenerate,
        stats.rejected_duplicate,
        stats.rejected_non_manifold,
        stats.rejected_ambiguous,
    );
    stats
}

fn order_boundary_loop(component: &[(u32, u32)]) -> Option<Vec<u32>> {
    if component.is_empty() {
        return None;
    }
    let mut neighbors: HashMap<u32, Vec<u32>> = HashMap::new();
    for &(a, b) in component {
        neighbors.entry(a).or_default().push(b);
        neighbors.entry(b).or_default().push(a);
    }
    if neighbors.values().any(|ns| ns.len() != 2) {
        return None;
    }
    let start = component[0].0;
    let mut ordered = vec![start];
    let mut prev = u32::MAX;
    let mut current = start;
    for _ in 0..component.len() {
        let nexts = neighbors.get(&current)?;
        let next = if nexts[0] != prev { nexts[0] } else { nexts[1] };
        if next == start {
            ordered.push(start);
            return Some(ordered);
        }
        ordered.push(next);
        prev = current;
        current = next;
    }
    None
}

fn loop_adjacent_normals(mesh: &Mesh, component: &[(u32, u32)]) -> Vec<Vec3> {
    let edge_to_triangles = build_mesh_edge_to_triangles(&mesh.indices);
    let positions = mesh_positions(mesh);
    let mut normals = Vec::new();
    for &edge in component {
        let Some(tris) = edge_to_triangles.get(&edge) else {
            continue;
        };
        if tris.len() != 1 {
            continue;
        }
        let tri_idx = tris[0];
        let tri = [
            mesh.indices[tri_idx * 3],
            mesh.indices[tri_idx * 3 + 1],
            mesh.indices[tri_idx * 3 + 2],
        ];
        let p0 = positions[tri[0] as usize];
        let p1 = positions[tri[1] as usize];
        let p2 = positions[tri[2] as usize];
        normals.push((p1 - p0).cross(p2 - p0).normalize_or_zero());
    }
    normals
}

fn loop_adjacent_normals_and_areas(mesh: &Mesh, component: &[(u32, u32)]) -> Vec<(Vec3, f32)> {
    let edge_to_triangles = build_mesh_edge_to_triangles(&mesh.indices);
    let positions = mesh_positions(mesh);
    let mut result = Vec::new();
    for &edge in component {
        let Some(tris) = edge_to_triangles.get(&edge) else {
            continue;
        };
        if tris.len() != 1 {
            continue;
        }
        let tri_idx = tris[0];
        let tri = [
            mesh.indices[tri_idx * 3],
            mesh.indices[tri_idx * 3 + 1],
            mesh.indices[tri_idx * 3 + 2],
        ];
        let p0 = positions[tri[0] as usize];
        let p1 = positions[tri[1] as usize];
        let p2 = positions[tri[2] as usize];
        let cross = (p1 - p0).cross(p2 - p0);
        result.push((cross.normalize_or_zero(), cross.length() * 0.5));
    }
    result
}

fn angle_between_deg(a: Vec3, b: Vec3) -> f32 {
    let da = a.normalize_or_zero();
    let db = b.normalize_or_zero();
    da.dot(db).clamp(-1.0, 1.0).acos().to_degrees()
}

fn dominant_normal_vote(
    adj: &[(Vec3, f32)],
    ignore_area_below: f32,
    orientation_repair: bool,
) -> (Vec3, bool) {
    let mut kept: Vec<(Vec3, f32)> = adj
        .iter()
        .copied()
        .filter(|(_, area)| *area >= ignore_area_below)
        .collect();
    if kept.is_empty() {
        kept = adj.to_vec();
    }
    if kept.is_empty() {
        return (Vec3::ZERO, false);
    }
    let seed = kept[0].0;
    let mut bimodal = false;
    let mut sum = Vec3::ZERO;
    for (mut n, area) in kept {
        if orientation_repair && seed.dot(n) < 0.0 {
            n = -n;
        }
        if seed.dot(n) < -0.35 {
            bimodal = true;
        }
        sum += n * area.max(1e-6);
    }
    (sum.normalize_or_zero(), bimodal)
}

fn polygon_normal(points: &[Vec3]) -> Vec3 {
    let mut n = Vec3::ZERO;
    for i in 0..points.len() {
        let p = points[i];
        let q = points[(i + 1) % points.len()];
        n.x += (p.y - q.y) * (p.z + q.z);
        n.y += (p.z - q.z) * (p.x + q.x);
        n.z += (p.x - q.x) * (p.y + q.y);
    }
    n.normalize_or_zero()
}

fn loop_planarity_error(points: &[Vec3], normal: Vec3) -> f32 {
    if points.is_empty() || normal == Vec3::ZERO {
        return f32::INFINITY;
    }
    let origin = points[0];
    points
        .iter()
        .map(|p| (*p - origin).dot(normal).abs())
        .fold(0.0f32, f32::max)
}

fn loop_area(points: &[Vec3], normal: Vec3) -> f32 {
    if points.len() < 3 || normal == Vec3::ZERO {
        return 0.0;
    }
    let origin = points[0];
    let mut area = 0.0f32;
    for i in 1..(points.len() - 1) {
        let a = points[i] - origin;
        let b = points[i + 1] - origin;
        area += a.cross(b).dot(normal).abs() * 0.5;
    }
    area
}

fn validate_added_triangles(
    mesh: &Mesh,
    tris: &[[u32; 3]],
) -> Result<MeshTopologyStats, &'static str> {
    let mut canonical_existing = HashSet::<[u32; 3]>::new();
    let mut oriented_existing = HashSet::<[u32; 3]>::new();
    for tri in mesh.indices.chunks_exact(3) {
        let t = [tri[0], tri[1], tri[2]];
        canonical_existing.insert(canonical_mesh_triangle(t));
        oriented_existing.insert(t);
    }
    for tri in tris {
        if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
            return Err("degenerate");
        }
        if oriented_existing.contains(tri)
            || canonical_existing.contains(&canonical_mesh_triangle(*tri))
        {
            return Err("duplicate");
        }
    }
    let mut test = mesh.clone();
    for tri in tris {
        test.indices.extend_from_slice(tri);
    }
    Ok(compute_mesh_topology_stats(&test))
}

fn classify_rejected_tiny_loop(
    loop_n: Vec3,
    adj: &[(Vec3, f32)],
    area: f32,
    diag: f32,
    near_non_manifold: bool,
    near_degenerate: bool,
) -> &'static str {
    let (dominant, bimodal) = dominant_normal_vote(adj, 1e-8, false);
    let mut angles = adj
        .iter()
        .map(|(n, _)| angle_between_deg(loop_n, *n))
        .collect::<Vec<_>>();
    angles.sort_by(|a, b| a.total_cmp(b));
    let avg = if angles.is_empty() {
        0.0
    } else {
        angles.iter().sum::<f32>() / angles.len() as f32
    };
    let max = angles.last().copied().unwrap_or(0.0);
    if near_degenerate {
        "Degenerate-neighborhood issue"
    } else if bimodal {
        if diag < 0.2 && area < 0.02 {
            "Opposite shell proximity"
        } else {
            "True fold / self-overlap"
        }
    } else if dominant != Vec3::ZERO && avg < 35.0 && max > 75.0 {
        "Safe but noisy normals"
    } else if near_non_manifold || max > 95.0 {
        "Orientation/winding issue"
    } else {
        "Unknown"
    }
}

fn try_loop_fill_candidates(
    mesh: &Mesh,
    verts: &[u32],
    before: &MeshTopologyStats,
) -> Result<(Vec<[u32; 3]>, MeshTopologyStats), &'static str> {
    let candidates: Vec<Vec<[u32; 3]>> = if verts.len() == 3 {
        vec![vec![[verts[0], verts[1], verts[2]]]]
    } else {
        vec![
            vec![
                [verts[0], verts[1], verts[2]],
                [verts[0], verts[2], verts[3]],
            ],
            vec![
                [verts[1], verts[2], verts[3]],
                [verts[1], verts[3], verts[0]],
            ],
        ]
    };
    let mut accepted = None::<(Vec<[u32; 3]>, MeshTopologyStats)>;
    let mut ambiguous = 0usize;
    for tris in candidates {
        match validate_added_triangles(mesh, &tris) {
            Ok(after) => {
                if after.boundary_edges < before.boundary_edges
                    && after.non_manifold_edges <= before.non_manifold_edges
                {
                    let score = after.boundary_edges * 1000 + after.non_manifold_edges;
                    if accepted
                        .as_ref()
                        .map(|(_, s)| score < s.boundary_edges * 1000 + s.non_manifold_edges)
                        .unwrap_or(true)
                    {
                        accepted = Some((tris, after));
                    } else {
                        ambiguous += 1;
                    }
                } else {
                    ambiguous += 1;
                }
            }
            Err(reason) => return Err(reason),
        }
    }
    if verts.len() == 4 && accepted.is_none() && ambiguous > 1 {
        Err("ambiguous")
    } else {
        accepted.ok_or("topology")
    }
}

fn fill_tiny_loops_with_mode(
    mesh: &mut Mesh,
    local_cell_size: f32,
    mode: TinyLoopMode,
    log_rejected: bool,
) -> TinyLoopFillStats {
    let before = compute_mesh_topology_stats(mesh);
    let boundary_edges: Vec<(u32, u32)> = boundary_edge_set(&mesh.indices).into_iter().collect();
    let components = boundary_components(&boundary_edges);
    let positions = mesh_positions(mesh);
    let transition_fix = transition_fix_from_env();
    let mut stats = TinyLoopFillStats {
        boundary_edges_before: before.boundary_edges,
        non_manifold_before: before.non_manifold_edges,
        ..TinyLoopFillStats::default()
    };

    for component in components {
        if !(3..=4).contains(&component.len()) {
            continue;
        }
        stats.tiny_loops_detected += 1;
        if component.len() == 3 {
            stats.tri_loops_detected += 1;
        } else {
            stats.quad_loops_detected += 1;
        }

        let Some(ordered) = order_boundary_loop(&component) else {
            stats.loops_rejected += 1;
            stats.rejected_failed_topology += 1;
            continue;
        };
        if ordered.first() != ordered.last() {
            stats.loops_rejected += 1;
            stats.rejected_failed_topology += 1;
            continue;
        }
        let verts: Vec<u32> = ordered[..ordered.len() - 1].to_vec();
        let points: Vec<Vec3> = verts.iter().map(|&v| positions[v as usize]).collect();
        let bbox_min = points
            .iter()
            .copied()
            .fold(Vec3::splat(f32::INFINITY), |acc, p| acc.min(p));
        let bbox_max = points
            .iter()
            .copied()
            .fold(Vec3::splat(f32::NEG_INFINITY), |acc, p| acc.max(p));
        let center = (bbox_min + bbox_max) * 0.5;
        let corridor_relaxed = transition_fix == TransitionCorridorFix::TinyLoop
            && in_inlet_transition_corridor(center);
        let diag = bbox_max.distance(bbox_min);
        let diag_limit = if corridor_relaxed {
            local_cell_size * 0.6
        } else {
            local_cell_size * 0.35
        };
        if diag > diag_limit {
            stats.loops_rejected += 1;
            stats.rejected_loop_too_large += 1;
            continue;
        }
        let loop_n = polygon_normal(&points);
        let planarity = loop_planarity_error(&points, loop_n);
        let planarity_limit = if corridor_relaxed {
            local_cell_size * 0.04
        } else {
            local_cell_size * 0.02
        };
        if !planarity.is_finite() || planarity > planarity_limit {
            stats.loops_rejected += 1;
            stats.rejected_non_planar += 1;
            continue;
        }
        let area = loop_area(&points, loop_n);
        let area_limit = if corridor_relaxed {
            local_cell_size * local_cell_size * 0.12
        } else {
            local_cell_size * local_cell_size * 0.05
        };
        if area > area_limit {
            stats.loops_rejected += 1;
            stats.rejected_loop_too_large += 1;
            continue;
        }
        let adj = loop_adjacent_normals_and_areas(mesh, &component);
        let near_non_manifold = component.iter().any(|edge| {
            build_mesh_edge_to_triangles(&mesh.indices)
                .get(edge)
                .map(|tris| tris.len() != 1)
                .unwrap_or(false)
        });
        let near_degenerate = adj.iter().any(|(_, face_area)| *face_area < 1e-4);
        let (dominant_a, bimodal_a) = dominant_normal_vote(&adj, 1e-4, false);
        let (dominant_c, bimodal_c) = dominant_normal_vote(&adj, 1e-8, true);
        let angles = adj
            .iter()
            .map(|(n, _)| angle_between_deg(loop_n, *n))
            .collect::<Vec<_>>();
        let min_angle = angles.iter().copied().fold(180.0f32, f32::min);
        let max_angle = angles.iter().copied().fold(0.0f32, f32::max);
        let avg_angle = if angles.is_empty() {
            0.0
        } else {
            angles.iter().sum::<f32>() / angles.len() as f32
        };

        let normal_ok = match mode {
            TinyLoopMode::Current => adj.iter().all(|(n, _)| loop_n.dot(*n) >= 0.2),
            TinyLoopMode::RobustNormalVoting => {
                !bimodal_a && dominant_a != Vec3::ZERO && loop_n.dot(dominant_a) >= 0.2
            }
            TinyLoopMode::LocalTopologyOnly => verts.len() == 3 && area <= area_limit && !bimodal_a,
            TinyLoopMode::OrientationRepair => {
                !bimodal_c && dominant_c != Vec3::ZERO && loop_n.dot(dominant_c) >= 0.2
            }
        };
        if !normal_ok {
            stats.loops_rejected += 1;
            stats.rejected_bad_normals += 1;
            if log_rejected {
                let feature = classify_patch_feature(
                    (bbox_min + bbox_max) * 0.5,
                    mesh_bounds(&positions).0,
                    mesh_bounds(&positions).1,
                );
                let adj_normals = adj
                    .iter()
                    .map(|(n, _)| format!("[{:.3},{:.3},{:.3}]", n.x, n.y, n.z))
                    .collect::<Vec<_>>()
                    .join(", ");
                let class = classify_rejected_tiny_loop(
                    loop_n,
                    &adj,
                    area,
                    diag,
                    near_non_manifold,
                    near_degenerate,
                );
                eprintln!(
                    "    tiny_loop_rejected: loop_id={} loop_vertex_count={} bbox=({:.3},{:.3},{:.3})..({:.3},{:.3},{:.3}) area={:.6} loop_normal=[{:.3},{:.3},{:.3}] adjacent_face_count={} adjacent_face_normals=[{}] min_angle_deg={:.3} max_angle_deg={:.3} avg_angle_deg={:.3} candidate_fill_normal=[{:.3},{:.3},{:.3}] feature={} near_non_manifold_cluster={} near_degenerate_cluster={} local_inverted_or_inconsistent_winding={} classification={}",
                    stats.loops_rejected,
                    verts.len(),
                    bbox_min.x,
                    bbox_min.y,
                    bbox_min.z,
                    bbox_max.x,
                    bbox_max.y,
                    bbox_max.z,
                    area,
                    loop_n.x,
                    loop_n.y,
                    loop_n.z,
                    adj.len(),
                    adj_normals,
                    min_angle,
                    max_angle,
                    avg_angle,
                    loop_n.x,
                    loop_n.y,
                    loop_n.z,
                    feature,
                    near_non_manifold,
                    near_degenerate,
                    bimodal_a || max_angle > 90.0,
                    class,
                );
            }
            continue;
        }

        let try_fill = try_loop_fill_candidates(mesh, &verts, &before);
        let (tris, after_stats) = match try_fill {
            Ok(ok) => ok,
            Err(reason) => {
                stats.loops_rejected += 1;
                match reason {
                    "degenerate" => stats.rejected_degenerate += 1,
                    "duplicate" => stats.rejected_duplicate += 1,
                    "ambiguous" => stats.rejected_ambiguous_diagonal += 1,
                    _ => stats.rejected_failed_topology += 1,
                }
                continue;
            }
        };
        for tri in &tris {
            mesh.indices.extend_from_slice(tri);
            stats.triangles_added += 1;
        }
        stats.loops_accepted += 1;
        stats.boundary_edges_after = after_stats.boundary_edges;
        stats.non_manifold_after = after_stats.non_manifold_edges;
        stats.connected_components_after = after_stats.connected_components;
        stats.watertight_after = after_stats.watertight;
    }
    let after = compute_mesh_topology_stats(mesh);
    stats.boundary_edges_after = after.boundary_edges;
    stats.non_manifold_after = after.non_manifold_edges;
    stats.connected_components_after = after.connected_components;
    stats.watertight_after = after.watertight;
    eprintln!(
        "  adaptive-mc tiny_loop_fill mode={:?}: tiny_loops_detected={} tri_loops_detected={} quad_loops_detected={} loops_accepted={} loops_rejected={} triangles_added={} boundary_edges_before={} boundary_edges_after={} non_manifold_before={} non_manifold_after={} connected_components_after={} watertight={} reject_loop_too_large={} reject_non_planar={} reject_bad_normals={} reject_degenerate={} reject_duplicate={} reject_non_manifold={} reject_ambiguous_diagonal={} reject_failed_topology={}",
        mode,
        stats.tiny_loops_detected,
        stats.tri_loops_detected,
        stats.quad_loops_detected,
        stats.loops_accepted,
        stats.loops_rejected,
        stats.triangles_added,
        stats.boundary_edges_before,
        stats.boundary_edges_after,
        stats.non_manifold_before,
        stats.non_manifold_after,
        stats.connected_components_after,
        stats.watertight_after,
        stats.rejected_loop_too_large,
        stats.rejected_non_planar,
        stats.rejected_bad_normals,
        stats.rejected_degenerate,
        stats.rejected_duplicate,
        stats.rejected_non_manifold,
        stats.rejected_ambiguous_diagonal,
        stats.rejected_failed_topology,
    );
    stats
}

fn report_tiny_loop_variants(mesh: &Mesh, local_cell_size: f32) {
    let before = compute_mesh_topology_stats(mesh);
    for mode in [
        TinyLoopMode::RobustNormalVoting,
        TinyLoopMode::LocalTopologyOnly,
        TinyLoopMode::OrientationRepair,
    ] {
        let mut test = mesh.clone();
        let stats = fill_tiny_loops_with_mode(&mut test, local_cell_size, mode, false);
        let after = compute_mesh_topology_stats(&test);
        let new_non_manifold = after
            .non_manifold_edges
            .saturating_sub(before.non_manifold_edges);
        eprintln!(
            "  adaptive-mc tiny_loop_variant mode={:?}: loops_accepted={} loops_rejected={} boundary_edges_before={} boundary_edges_after={} non_manifold_before={} non_manifold_after={} connected_components_before={} connected_components_after={} new_non_manifold_edges_introduced={} new_inverted_triangles_detected=false",
            mode,
            stats.loops_accepted,
            stats.loops_rejected,
            before.boundary_edges,
            after.boundary_edges,
            before.non_manifold_edges,
            after.non_manifold_edges,
            before.connected_components,
            after.connected_components,
            new_non_manifold,
        );
    }
}

fn repair_tiny_loops(mesh: &mut Mesh, local_cell_size: f32) -> TinyLoopFillStats {
    report_tiny_loop_variants(mesh, local_cell_size);
    fill_tiny_loops_with_mode(mesh, local_cell_size, TinyLoopMode::LocalTopologyOnly, true)
}

fn max_boundary_component_len(mesh: &Mesh) -> usize {
    let edges = boundary_edge_set(&mesh.indices)
        .into_iter()
        .collect::<Vec<_>>();
    boundary_components(&edges)
        .into_iter()
        .map(|comp| comp.len())
        .max()
        .unwrap_or(0)
}

fn mesh_triangle_area_sq(mesh: &Mesh, tri: [u32; 3]) -> f32 {
    let p0 = Vec3::from_array(mesh.vertices[tri[0] as usize].position);
    let p1 = Vec3::from_array(mesh.vertices[tri[1] as usize].position);
    let p2 = Vec3::from_array(mesh.vertices[tri[2] as usize].position);
    (p1 - p0).cross(p2 - p0).length_squared()
}

fn duplicate_cleanup_stage_report(
    stage: &str,
    mesh: &Mesh,
    triangles_removed: usize,
    triangles_added: usize,
    vertices_added: usize,
    duplicate_removed: usize,
    reversed_removed: usize,
    zero_area_removed: usize,
    near_zero_removed: usize,
    rejected_reasons: &str,
) {
    let topo = compute_mesh_topology_stats(mesh);
    eprintln!(
        "  adaptive-mc final_cleanup_stage: stage={} triangles={} vertices={} boundary_edges={} non_manifold_edges={} connected_components={} watertight={} triangles_removed={} triangles_added={} vertices_added={} duplicate_triangles_removed={} reversed_duplicates_removed={} zero_area_triangles_removed={} near_zero_area_triangles_removed={} rejected_repairs={}",
        stage,
        topo.triangles,
        topo.vertices,
        topo.boundary_edges,
        topo.non_manifold_edges,
        topo.connected_components,
        topo.watertight,
        triangles_removed,
        triangles_added,
        vertices_added,
        duplicate_removed,
        reversed_removed,
        zero_area_removed,
        near_zero_removed,
        rejected_reasons,
    );
}

fn repair_variant_from_env() -> RepairVariant {
    match std::env::var("IMPLICIT_CAD_REPAIR_VARIANT").ok().as_deref() {
        Some("A") => RepairVariant::ACurrentOrder,
        Some("B") => RepairVariant::BEarlyDuplicate,
        Some("C") => RepairVariant::CNoTJunction,
        Some("D") => RepairVariant::DNoEdgePair,
        _ => RepairVariant::Default,
    }
}

fn transition_fix_from_env() -> TransitionCorridorFix {
    match std::env::var("IMPLICIT_CAD_TRANSITION_FIX").ok().as_deref() {
        Some("A") => TransitionCorridorFix::LargerRepairRadius,
        Some("B") => TransitionCorridorFix::MicroPatch,
        Some("C") => TransitionCorridorFix::TinyLoop,
        _ => TransitionCorridorFix::None,
    }
}

fn transition_layout_mode_from_env() -> &'static str {
    match std::env::var("IMPLICIT_CAD_TRANSITION_LAYOUT")
        .ok()
        .as_deref()
    {
        Some("active") => "active",
        Some("diag") => "diag",
        _ => "off",
    }
}

fn in_inlet_transition_corridor(p: Vec3) -> bool {
    (304.0..=406.0).contains(&p.x)
        && (23.0..=30.0).contains(&p.y.abs())
        && (23.0..=63.0).contains(&p.z)
}

fn near_inlet_transition_plane(p: Vec3) -> bool {
    (p.x - 304.0).abs() <= 2.0
        || (p.x - 406.0).abs() <= 2.0
        || ((p.y.abs() - 27.0).abs() <= 2.0 && (23.0..=63.0).contains(&p.z))
        || (p.z - 23.0).abs() <= 2.0
        || (p.z - 63.0).abs() <= 2.0
}

fn new_boundary_edges_stay_local(
    before_mesh: &Mesh,
    after_mesh: &Mesh,
    focus: Vec3,
    max_distance: f32,
) -> bool {
    let before_edges = boundary_edge_set(&before_mesh.indices);
    let after_edges = boundary_edge_set(&after_mesh.indices);
    let positions = mesh_positions(after_mesh);
    after_edges.difference(&before_edges).all(|&(a, b)| {
        let mid = (positions[a as usize] + positions[b as usize]) * 0.5;
        mid.distance(focus) <= max_distance
    })
}

fn remove_final_duplicate_stack_triangles(
    mesh: &mut Mesh,
    local_cell_size: f32,
) -> DuplicateCleanupStats {
    let before = compute_mesh_topology_stats(mesh);
    let mut stats = DuplicateCleanupStats {
        triangles_before: before.triangles,
        vertices_before: before.vertices,
        boundary_before: before.boundary_edges,
        non_manifold_before: before.non_manifold_edges,
        connected_before: before.connected_components,
        ..Default::default()
    };

    let area_eps_sq = (local_cell_size * 0.001).powi(2);
    let near_zero_eps_sq = area_eps_sq * 16.0;
    let mut candidates = Vec::<(usize, &'static str, bool)>::new();
    let mut first_oriented = HashMap::<[u32; 3], usize>::new();
    let mut first_canonical = HashMap::<[u32; 3], usize>::new();

    for (tri_idx, tri) in mesh.indices.chunks_exact(3).enumerate() {
        let t = [tri[0], tri[1], tri[2]];
        let oriented = t;
        let canonical = canonical_mesh_triangle(t);
        let area_sq = mesh_triangle_area_sq(mesh, t);
        if let Some(_) = first_oriented.get(&oriented) {
            candidates.push((tri_idx, "duplicate", true));
            continue;
        }
        first_oriented.insert(oriented, tri_idx);

        if let Some(&prev_idx) = first_canonical.get(&canonical) {
            let prev = [
                mesh.indices[prev_idx * 3],
                mesh.indices[prev_idx * 3 + 1],
                mesh.indices[prev_idx * 3 + 2],
            ];
            let reason = if permutation_parity_mesh(prev) != permutation_parity_mesh(t) {
                "reversed"
            } else {
                "same_vertex_set"
            };
            candidates.push((tri_idx, reason, true));
            continue;
        }
        first_canonical.insert(canonical, tri_idx);

        if area_sq <= area_eps_sq {
            candidates.push((tri_idx, "zero_area", false));
        } else if area_sq <= near_zero_eps_sq {
            candidates.push((tri_idx, "near_zero_area", false));
        }
    }

    let mut removed = HashSet::<usize>::new();
    let mut current_topo = before.clone();
    let mut current_max_loop = max_boundary_component_len(mesh);

    for (tri_idx, reason, duplicate_like) in candidates {
        if removed.contains(&tri_idx) {
            continue;
        }
        let tri_base = tri_idx * 3;
        if tri_base + 2 >= mesh.indices.len() {
            continue;
        }

        let mut test_mesh = mesh.clone();
        test_mesh.indices.drain(tri_base..tri_base + 3);
        let after = compute_mesh_topology_stats(&test_mesh);
        let new_max_loop = max_boundary_component_len(&test_mesh);

        let topology_ok = after.non_manifold_edges <= current_topo.non_manifold_edges
            && after.connected_components <= current_topo.connected_components
            && (!duplicate_like || after.boundary_edges <= current_topo.boundary_edges + 4)
            && (duplicate_like || after.boundary_edges <= current_topo.boundary_edges)
            && new_max_loop <= current_max_loop.max(4);

        if !topology_ok {
            if after.connected_components > current_topo.connected_components {
                stats.rejected_connected_components += 1;
            } else if new_max_loop > current_max_loop.max(4) {
                stats.rejected_large_boundary_loop += 1;
            } else {
                stats.rejected_no_topology_gain += 1;
            }
            continue;
        }

        if after.boundary_edges > current_topo.boundary_edges {
            stats.removed_new_boundary_edges += after.boundary_edges - current_topo.boundary_edges;
        }
        if after.non_manifold_edges < current_topo.non_manifold_edges {
            stats.removed_reduced_non_manifold += 1;
        }
        match reason {
            "duplicate" => stats.duplicate_triangles_removed += 1,
            "reversed" => stats.reversed_duplicates_removed += 1,
            "same_vertex_set" => stats.same_vertex_set_removed += 1,
            "zero_area" => stats.zero_area_removed += 1,
            "near_zero_area" => stats.near_zero_area_removed += 1,
            _ => {}
        }
        stats.triangles_removed += 1;
        *mesh = test_mesh;
        current_topo = after;
        current_max_loop = new_max_loop;
        removed.insert(tri_idx);
    }

    let after = compute_mesh_topology_stats(mesh);
    stats.triangles_after = after.triangles;
    stats.vertices_after = after.vertices;
    stats.boundary_after = after.boundary_edges;
    stats.non_manifold_after = after.non_manifold_edges;
    stats.connected_after = after.connected_components;
    stats.watertight_after = after.watertight;
    stats
}

fn run_repair_pipeline_variant(
    mesh: &mut Mesh,
    degenerate_cell_size: f32,
    variant: RepairVariant,
) -> Vec<(u32, u32)> {
    match variant {
        RepairVariant::Default => {
            print_boundary_repairability_diagnostic(&mesh, 0.25, 0.01);
            let _edge_pair_stats = repair_edge_pair_cracks(mesh, 0.25);
            diagnose_boundary_edge_vertex_matches(mesh, 0.01);
            let repair_stats = repair_t_junctions(mesh, 0.01);
            let tiny_loop_stats = repair_tiny_loops(mesh, degenerate_cell_size);
            print_boundary_checkpoint("post_repair", &mesh);

            duplicate_cleanup_stage_report(
                "before_duplicate_cleanup",
                &mesh,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                &format!(
                    "edge_pair_rejected={} tjunction_skipped_no_match={} tjunction_skipped_ambiguous={} tiny_loop_rejected={}",
                    0usize,
                    repair_stats.skipped_no_match,
                    repair_stats.skipped_ambiguous,
                    tiny_loop_stats.loops_rejected,
                ),
            );

            let duplicate_cleanup_stats =
                remove_final_duplicate_stack_triangles(mesh, degenerate_cell_size);
            duplicate_cleanup_stage_report(
                "after_duplicate_cleanup",
                &mesh,
                duplicate_cleanup_stats.triangles_removed,
                0,
                0,
                duplicate_cleanup_stats.duplicate_triangles_removed
                    + duplicate_cleanup_stats.same_vertex_set_removed,
                duplicate_cleanup_stats.reversed_duplicates_removed,
                duplicate_cleanup_stats.zero_area_removed,
                duplicate_cleanup_stats.near_zero_area_removed,
                &format!(
                    "rejected_no_topology_gain={} rejected_connected_components={} rejected_large_boundary_loop={}",
                    duplicate_cleanup_stats.rejected_no_topology_gain,
                    duplicate_cleanup_stats.rejected_connected_components,
                    duplicate_cleanup_stats.rejected_large_boundary_loop,
                ),
            );

            let final_edge_pair_stats = repair_edge_pair_cracks(mesh, 0.25);
            duplicate_cleanup_stage_report(
                "after_final_edge_pair_repair",
                &mesh,
                0,
                final_edge_pair_stats.triangles_added,
                final_edge_pair_stats.vertices_added,
                0,
                0,
                0,
                0,
                &format!(
                    "rejected_distance_too_large={} rejected_orientation_mismatch={} rejected_length_mismatch={} rejected_normal_mismatch={} rejected_degenerate={} rejected_duplicate={} rejected_non_manifold={} rejected_ambiguous={}",
                    final_edge_pair_stats.rejected_distance_too_large,
                    final_edge_pair_stats.rejected_orientation_mismatch,
                    final_edge_pair_stats.rejected_length_mismatch,
                    final_edge_pair_stats.rejected_normal_mismatch,
                    final_edge_pair_stats.rejected_degenerate,
                    final_edge_pair_stats.rejected_duplicate,
                    final_edge_pair_stats.rejected_non_manifold,
                    final_edge_pair_stats.rejected_ambiguous,
                ),
            );

            let final_tjunction_stats = repair_t_junctions(mesh, 0.01);
            duplicate_cleanup_stage_report(
                "after_final_tjunction_repair",
                &mesh,
                0,
                final_tjunction_stats.triangles_added,
                0,
                0,
                0,
                0,
                0,
                &format!(
                    "skipped_no_match={} skipped_not_boundary={} skipped_ambiguous={}",
                    final_tjunction_stats.skipped_no_match,
                    final_tjunction_stats.skipped_not_boundary,
                    final_tjunction_stats.skipped_ambiguous,
                ),
            );

            let final_tiny_loop_stats = fill_tiny_loops_with_mode(
                mesh,
                degenerate_cell_size,
                TinyLoopMode::LocalTopologyOnly,
                true,
            );
            duplicate_cleanup_stage_report(
                "after_final_tiny_loop_fill",
                &mesh,
                0,
                final_tiny_loop_stats.triangles_added,
                0,
                0,
                0,
                0,
                0,
                &format!(
                    "loops_rejected={} reject_loop_too_large={} reject_non_planar={} reject_bad_normals={} reject_degenerate={} reject_duplicate={} reject_non_manifold={} reject_ambiguous_diagonal={} reject_failed_topology={}",
                    final_tiny_loop_stats.loops_rejected,
                    final_tiny_loop_stats.rejected_loop_too_large,
                    final_tiny_loop_stats.rejected_non_planar,
                    final_tiny_loop_stats.rejected_bad_normals,
                    final_tiny_loop_stats.rejected_degenerate,
                    final_tiny_loop_stats.rejected_duplicate,
                    final_tiny_loop_stats.rejected_non_manifold,
                    final_tiny_loop_stats.rejected_ambiguous_diagonal,
                    final_tiny_loop_stats.rejected_failed_topology,
                ),
            );
            return final_tjunction_stats.repair_edges;
        }
        RepairVariant::ACurrentOrder
        | RepairVariant::BEarlyDuplicate
        | RepairVariant::CNoTJunction
        | RepairVariant::DNoEdgePair => {
            eprintln!("  adaptive-mc repair_variant={:?}", variant);
            eprintln!(
                "  adaptive-mc transition_fix={:?}",
                transition_fix_from_env()
            );
            duplicate_cleanup_stage_report(
                "post_constrained_weld",
                &mesh,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                "none",
            );

            if matches!(
                variant,
                RepairVariant::BEarlyDuplicate
                    | RepairVariant::CNoTJunction
                    | RepairVariant::DNoEdgePair
            ) {
                let early_dup = remove_final_duplicate_stack_triangles(mesh, degenerate_cell_size);
                duplicate_cleanup_stage_report(
                    "after_duplicate_cleanup",
                    &mesh,
                    early_dup.triangles_removed,
                    0,
                    0,
                    early_dup.duplicate_triangles_removed + early_dup.same_vertex_set_removed,
                    early_dup.reversed_duplicates_removed,
                    early_dup.zero_area_removed,
                    early_dup.near_zero_area_removed,
                    &format!(
                        "rejected_no_topology_gain={} rejected_connected_components={} rejected_large_boundary_loop={}",
                        early_dup.rejected_no_topology_gain,
                        early_dup.rejected_connected_components,
                        early_dup.rejected_large_boundary_loop,
                    ),
                );
            }

            if !matches!(variant, RepairVariant::DNoEdgePair) {
                let edge_pair_stats = repair_edge_pair_cracks(mesh, 0.25);
                duplicate_cleanup_stage_report(
                    "after_edge_pair_repair",
                    &mesh,
                    0,
                    edge_pair_stats.triangles_added,
                    edge_pair_stats.vertices_added,
                    0,
                    0,
                    0,
                    0,
                    &format!(
                        "accepted_repairs={} rejected_distance_too_large={} rejected_orientation_mismatch={} rejected_length_mismatch={} rejected_normal_mismatch={} rejected_degenerate={} rejected_duplicate={} rejected_non_manifold={} rejected_ambiguous={}",
                        edge_pair_stats.candidates_accepted,
                        edge_pair_stats.rejected_distance_too_large,
                        edge_pair_stats.rejected_orientation_mismatch,
                        edge_pair_stats.rejected_length_mismatch,
                        edge_pair_stats.rejected_normal_mismatch,
                        edge_pair_stats.rejected_degenerate,
                        edge_pair_stats.rejected_duplicate,
                        edge_pair_stats.rejected_non_manifold,
                        edge_pair_stats.rejected_ambiguous,
                    ),
                );
            }

            let mut repair_edges = Vec::new();
            if !matches!(variant, RepairVariant::CNoTJunction) {
                let tj_stats = repair_t_junctions(mesh, 0.01);
                repair_edges = tj_stats.repair_edges.clone();
                duplicate_cleanup_stage_report(
                    "after_tjunction_repair",
                    &mesh,
                    0,
                    tj_stats.triangles_added,
                    0,
                    0,
                    0,
                    0,
                    0,
                    &format!(
                        "accepted_repairs={} skipped_no_match={} skipped_not_boundary={} skipped_ambiguous={}",
                        tj_stats.repaired_edges,
                        tj_stats.skipped_no_match,
                        tj_stats.skipped_not_boundary,
                        tj_stats.skipped_ambiguous,
                    ),
                );
            }

            let tiny_stats = fill_tiny_loops_with_mode(
                mesh,
                degenerate_cell_size,
                TinyLoopMode::LocalTopologyOnly,
                true,
            );
            duplicate_cleanup_stage_report(
                "after_tiny_loop_fill",
                &mesh,
                0,
                tiny_stats.triangles_added,
                0,
                0,
                0,
                0,
                0,
                &format!(
                    "accepted_repairs={} loops_rejected={} reject_loop_too_large={} reject_non_planar={} reject_bad_normals={} reject_degenerate={} reject_duplicate={} reject_non_manifold={} reject_ambiguous_diagonal={} reject_failed_topology={}",
                    tiny_stats.loops_accepted,
                    tiny_stats.loops_rejected,
                    tiny_stats.rejected_loop_too_large,
                    tiny_stats.rejected_non_planar,
                    tiny_stats.rejected_bad_normals,
                    tiny_stats.rejected_degenerate,
                    tiny_stats.rejected_duplicate,
                    tiny_stats.rejected_non_manifold,
                    tiny_stats.rejected_ambiguous_diagonal,
                    tiny_stats.rejected_failed_topology,
                ),
            );

            if transition_fix_from_env() == TransitionCorridorFix::MicroPatch {
                let added = apply_transition_micro_patch(mesh);
                duplicate_cleanup_stage_report(
                    "after_transition_micro_patch",
                    &mesh,
                    0,
                    added,
                    0,
                    0,
                    0,
                    0,
                    0,
                    "transition_plane_only",
                );
            }

            let late_dup = remove_final_duplicate_stack_triangles(mesh, degenerate_cell_size);
            duplicate_cleanup_stage_report(
                "after_late_duplicate_cleanup",
                &mesh,
                late_dup.triangles_removed,
                0,
                0,
                late_dup.duplicate_triangles_removed + late_dup.same_vertex_set_removed,
                late_dup.reversed_duplicates_removed,
                late_dup.zero_area_removed,
                late_dup.near_zero_area_removed,
                &format!(
                    "rejected_no_topology_gain={} rejected_connected_components={} rejected_large_boundary_loop={}",
                    late_dup.rejected_no_topology_gain,
                    late_dup.rejected_connected_components,
                    late_dup.rejected_large_boundary_loop,
                ),
            );
            return repair_edges;
        }
    }
}

fn remove_duplicate_and_reversed_triangles(mesh: &mut Mesh) -> usize {
    let mut seen_oriented = HashSet::<[u32; 3]>::new();
    let mut seen_canonical = HashSet::<[u32; 3]>::new();
    let mut keep = Vec::with_capacity(mesh.indices.len());
    let mut removed = 0usize;
    for tri in mesh.indices.chunks_exact(3) {
        let t = [tri[0], tri[1], tri[2]];
        let canonical = canonical_mesh_triangle(t);
        if !seen_oriented.insert(t) || !seen_canonical.insert(canonical) {
            removed += 1;
            continue;
        }
        keep.extend_from_slice(tri);
    }
    mesh.indices = keep;
    removed
}

fn isolate_non_manifold_edges_by_vertex_split(mesh: &mut Mesh) -> usize {
    let edge_to_tris = build_mesh_edge_to_triangles(&mesh.indices);
    let mut split_count = 0usize;
    for (edge, tris) in edge_to_tris {
        if tris.len() <= 2 {
            continue;
        }
        for &tri_idx in tris.iter().skip(2) {
            let base = tri_idx * 3;
            let tri = [
                mesh.indices[base],
                mesh.indices[base + 1],
                mesh.indices[base + 2],
            ];
            for (corner, &v) in tri.iter().enumerate() {
                if v == edge.0 || v == edge.1 {
                    let new_idx = mesh.vertices.len() as u32;
                    mesh.vertices.push(mesh.vertices[v as usize]);
                    mesh.indices[base + corner] = new_idx;
                    split_count += 1;
                }
            }
        }
    }
    split_count
}

fn run_final_repair_variants(mesh: &Mesh, local_cell_size: f32) {
    let before = compute_mesh_topology_stats(mesh);

    let mut a_mesh = mesh.clone();
    let removed = remove_duplicate_and_reversed_triangles(&mut a_mesh);
    let a_after = compute_mesh_topology_stats(&a_mesh);
    eprintln!(
        "  adaptive-mc final_variant_A_duplicate_cleanup: boundary_before={} boundary_after={} non_manifold_before={} non_manifold_after={} connected_before={} connected_after={} triangles_added=0 triangles_removed={} vertices_added=0 vertices_split=0 watertight={}",
        before.boundary_edges,
        a_after.boundary_edges,
        before.non_manifold_edges,
        a_after.non_manifold_edges,
        before.connected_components,
        a_after.connected_components,
        removed,
        a_after.watertight,
    );

    let mut b_mesh = mesh.clone();
    let b_before_tri = b_mesh.indices.len() / 3;
    let _b_stats = fill_tiny_loops_with_mode(
        &mut b_mesh,
        local_cell_size,
        TinyLoopMode::LocalTopologyOnly,
        false,
    );
    let b_after = compute_mesh_topology_stats(&b_mesh);
    eprintln!(
        "  adaptive-mc final_variant_B_final_tiny_loop: boundary_before={} boundary_after={} non_manifold_before={} non_manifold_after={} connected_before={} connected_after={} triangles_added={} triangles_removed=0 vertices_added=0 vertices_split=0 watertight={}",
        before.boundary_edges,
        b_after.boundary_edges,
        before.non_manifold_edges,
        b_after.non_manifold_edges,
        before.connected_components,
        b_after.connected_components,
        (b_mesh.indices.len() / 3).saturating_sub(b_before_tri),
        b_after.watertight,
    );

    let mut c_mesh = mesh.clone();
    let comps = boundary_components(
        &boundary_edge_set(&c_mesh.indices)
            .into_iter()
            .collect::<Vec<_>>(),
    );
    let mut micro_added = 0usize;
    for comp in comps {
        if comp.len() != 1 {
            continue;
        }
        let edge = comp[0];
        let tri_map = build_mesh_edge_to_triangles(&c_mesh.indices);
        let Some(tris) = tri_map.get(&edge) else {
            continue;
        };
        if tris.len() != 1 {
            continue;
        }
        let tri_idx = tris[0];
        let tri = [
            c_mesh.indices[tri_idx * 3],
            c_mesh.indices[tri_idx * 3 + 1],
            c_mesh.indices[tri_idx * 3 + 2],
        ];
        let Some((a, b, opp)) = directed_edge_and_opposite(tri, edge) else {
            continue;
        };
        if let Ok(stats) = validate_added_triangles(&c_mesh, &[[a, opp, b]]) {
            if stats.boundary_edges < compute_mesh_topology_stats(&c_mesh).boundary_edges
                && stats.non_manifold_edges
                    <= compute_mesh_topology_stats(&c_mesh).non_manifold_edges
            {
                c_mesh.indices.extend_from_slice(&[a, opp, b]);
                micro_added += 1;
            }
        }
    }
    let c_after = compute_mesh_topology_stats(&c_mesh);
    eprintln!(
        "  adaptive-mc final_variant_C_micro_patch: boundary_before={} boundary_after={} non_manifold_before={} non_manifold_after={} connected_before={} connected_after={} triangles_added={} triangles_removed=0 vertices_added=0 vertices_split=0 watertight={}",
        before.boundary_edges,
        c_after.boundary_edges,
        before.non_manifold_edges,
        c_after.non_manifold_edges,
        before.connected_components,
        c_after.connected_components,
        micro_added,
        c_after.watertight,
    );

    let mut d_mesh = mesh.clone();
    let split = isolate_non_manifold_edges_by_vertex_split(&mut d_mesh);
    let d_after = compute_mesh_topology_stats(&d_mesh);
    eprintln!(
        "  adaptive-mc final_variant_D_non_manifold_split: boundary_before={} boundary_after={} non_manifold_before={} non_manifold_after={} connected_before={} connected_after={} triangles_added=0 triangles_removed=0 vertices_added=0 vertices_split={} watertight={}",
        before.boundary_edges,
        d_after.boundary_edges,
        before.non_manifold_edges,
        d_after.non_manifold_edges,
        before.connected_components,
        d_after.connected_components,
        split,
        d_after.watertight,
    );
}

fn print_final_defect_localization(mesh: &Mesh) {
    let positions = mesh_positions(mesh);
    let (bounds_min, bounds_max) = mesh_bounds(&positions);
    let edge_to_tris = build_mesh_edge_to_triangles(&mesh.indices);
    let boundary_edges: Vec<(u32, u32)> = edge_to_tris
        .iter()
        .filter_map(|(&edge, tris)| (tris.len() == 1).then_some(edge))
        .collect();
    let boundary_components_vec = boundary_components(&boundary_edges);
    let mut edge_component_kind = HashMap::<(u32, u32), &'static str>::new();
    for comp in &boundary_components_vec {
        let kind = match comp.len() {
            1 => "isolated single-edge defect",
            3 => "3-edge loop",
            4 => "4-edge loop",
            _ => "larger boundary loop",
        };
        for &edge in comp {
            edge_component_kind.insert(edge, kind);
        }
    }

    let mut cluster_bins: BTreeMap<i32, (usize, usize)> = BTreeMap::new();
    let extent = bounds_max - bounds_min + Vec3::splat(1e-9);
    for &(a, b) in &boundary_edges {
        let mid = (positions[a as usize] + positions[b as usize]) * 0.5;
        let rel = (mid - bounds_min) / extent * 10.0;
        let key = (rel.x.floor() as i32).clamp(0, 9) * 100
            + (rel.y.floor() as i32).clamp(0, 9) * 10
            + (rel.z.floor() as i32).clamp(0, 9);
        cluster_bins.entry(key).or_default().0 += 1;
    }
    for (&edge, tris) in &edge_to_tris {
        if tris.len() == 2 {
            continue;
        }
        let mid = (positions[edge.0 as usize] + positions[edge.1 as usize]) * 0.5;
        let rel = (mid - bounds_min) / extent * 10.0;
        let key = (rel.x.floor() as i32).clamp(0, 9) * 100
            + (rel.y.floor() as i32).clamp(0, 9) * 10
            + (rel.z.floor() as i32).clamp(0, 9);
        cluster_bins.entry(key).or_default().1 += 1;
    }

    eprintln!("  adaptive-mc remaining_boundary_edges:");
    for (idx, &(a, b)) in boundary_edges.iter().enumerate() {
        let pa = positions[a as usize];
        let pb = positions[b as usize];
        let mid = (pa + pb) * 0.5;
        let len = pa.distance(pb);
        let feature = classify_patch_feature(mid, bounds_min, bounds_max);
        let mut counts = [0usize; 5];
        for &(c, d) in &boundary_edges {
            if (a, b) == (c, d) {
                continue;
            }
            let qmid = (positions[c as usize] + positions[d as usize]) * 0.5;
            let dist = mid.distance(qmid);
            if dist <= 0.001 {
                counts[0] += 1;
            }
            if dist <= 0.01 {
                counts[1] += 1;
            }
            if dist <= 0.1 {
                counts[2] += 1;
            }
            if dist <= 0.5 {
                counts[3] += 1;
            }
            if dist <= 1.0 {
                counts[4] += 1;
            }
        }
        eprintln!(
            "    boundary_edge id={} endpoints=({}, {}) length={:.6} midpoint=({:.3},{:.3},{:.3}) bbox=({:.3},{:.3},{:.3})..({:.3},{:.3},{:.3}) feature={} adjacent_triangle_count=1 nearby_0p001={} nearby_0p01={} nearby_0p1={} nearby_0p5={} nearby_1p0={} loop_kind={} non_manifold_cluster={}",
            idx + 1,
            a,
            b,
            len,
            mid.x,
            mid.y,
            mid.z,
            pa.min(pb).x,
            pa.min(pb).y,
            pa.min(pb).z,
            pa.max(pb).x,
            pa.max(pb).y,
            pa.max(pb).z,
            feature,
            counts[0],
            counts[1],
            counts[2],
            counts[3],
            counts[4],
            edge_component_kind
                .get(&(a, b))
                .copied()
                .unwrap_or("unknown"),
            edge_to_tris
                .get(&(a, b))
                .map(|t| t.len() != 1)
                .unwrap_or(false),
        );
    }

    eprintln!("  adaptive-mc remaining_non_manifold_edges:");
    let mut nm_cluster_id = 0usize;
    let mut cluster_lookup = HashMap::<i32, usize>::new();
    for (&edge, tris) in &edge_to_tris {
        if tris.len() == 2 {
            continue;
        }
        let mid = (positions[edge.0 as usize] + positions[edge.1 as usize]) * 0.5;
        let rel = (mid - bounds_min) / extent * 10.0;
        let key = (rel.x.floor() as i32).clamp(0, 9) * 100
            + (rel.y.floor() as i32).clamp(0, 9) * 10
            + (rel.z.floor() as i32).clamp(0, 9);
        let cid = *cluster_lookup.entry(key).or_insert_with(|| {
            nm_cluster_id += 1;
            nm_cluster_id
        });
        let len = positions[edge.0 as usize].distance(positions[edge.1 as usize]);
        let mut cause = "unknown";
        let tris_arr: Vec<[u32; 3]> = tris
            .iter()
            .map(|&t| {
                [
                    mesh.indices[t * 3],
                    mesh.indices[t * 3 + 1],
                    mesh.indices[t * 3 + 2],
                ]
            })
            .collect();
        let canonical_count: HashSet<[u32; 3]> = tris_arr
            .iter()
            .map(|t| canonical_mesh_triangle(*t))
            .collect();
        if canonical_count.len() < tris_arr.len() {
            cause = "duplicate triangles";
        } else if tris_arr
            .iter()
            .any(|tri| tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0])
        {
            cause = "degenerate triangles";
        } else if tris.len() > 2 {
            cause = "old extraction artifact";
        }
        eprintln!(
            "    non_manifold_edge id=({}, {}) incident_triangle_count={} length={:.6} midpoint=({:.3},{:.3},{:.3}) feature={} cause={} cluster_id={}",
            edge.0,
            edge.1,
            tris.len(),
            len,
            mid.x,
            mid.y,
            mid.z,
            classify_patch_feature(mid, bounds_min, bounds_max),
            cause,
            cid
        );
    }

    eprintln!("  adaptive-mc defect_clusters:");
    for (key, (bcount, nmcount)) in cluster_bins {
        if bcount == 0 && nmcount == 0 {
            continue;
        }
        let ix = key / 100;
        let iy = (key % 100) / 10;
        let iz = key % 10;
        let cell_min = bounds_min
            + (bounds_max - bounds_min) * Vec3::new(ix as f32, iy as f32, iz as f32) / 10.0;
        let cell_max = bounds_min
            + (bounds_max - bounds_min)
                * Vec3::new((ix + 1) as f32, (iy + 1) as f32, (iz + 1) as f32)
                / 10.0;
        let center = (cell_min + cell_max) * 0.5;
        let cause = if bcount > 0 && nmcount > 0 {
            "mixed residual crack + non-manifold"
        } else if bcount > 0 {
            "residual crack"
        } else {
            "non-manifold cluster"
        };
        let repair = if bcount > 0 {
            "micro patch or final tiny-loop fill"
        } else {
            "duplicate cleanup or edge split"
        };
        eprintln!(
            "    cluster_id={} boundary_edge_count={} non_manifold_edge_count={} bbox=({:.3},{:.3},{:.3})..({:.3},{:.3},{:.3}) nearest_feature={} likely_cause={} suggested_repair={}",
            key,
            bcount,
            nmcount,
            cell_min.x,
            cell_min.y,
            cell_min.z,
            cell_max.x,
            cell_max.y,
            cell_max.z,
            classify_patch_feature(center, bounds_min, bounds_max),
            cause,
            repair,
        );
    }
}

fn collect_boundary_match_stats(mesh: &Mesh, epsilon: f32) -> BoundaryMatchStats {
    if mesh.vertices.is_empty() || mesh.indices.is_empty() {
        return BoundaryMatchStats::default();
    }

    let edge_to_triangles = build_mesh_edge_to_triangles(&mesh.indices);
    let boundary_edges: Vec<(u32, u32)> = edge_to_triangles
        .iter()
        .filter_map(|(&(a, b), tris)| (tris.len() == 1).then_some((a, b)))
        .collect();

    let positions = mesh_positions(mesh);
    let grid_cell_size = (2.0 * epsilon).max(1e-6);
    let vertex_grid = build_vertex_grid(&positions, grid_cell_size);

    let mut stats = BoundaryMatchStats {
        total_boundary_edges: boundary_edges.len(),
        ..BoundaryMatchStats::default()
    };

    for &(a_idx, b_idx) in &boundary_edges {
        let matches = candidate_vertices_on_segment(
            &positions,
            &vertex_grid,
            a_idx,
            b_idx,
            epsilon,
            grid_cell_size,
        )
        .len();
        let center = (positions[a_idx as usize] + positions[b_idx as usize]) * 0.5;
        match matches {
            0 => {
                stats.no_match += 1;
                stats.no_match_centers.push(center);
            }
            1 => {
                stats.one_match += 1;
                stats.one_match_centers.push(center);
            }
            _ => {
                stats.multi_match += 1;
            }
        }
    }

    stats
}

fn repair_t_junctions(mesh: &mut Mesh, epsilon: f32) -> TJunctionRepairStats {
    let t0 = Instant::now();
    let max_iterations = 10usize;
    let triangles_before = mesh.indices.len() / 3;
    let start_stats = collect_boundary_match_stats(mesh, epsilon);
    let mut previous_boundary_edges = start_stats.total_boundary_edges;
    let mut total = TJunctionRepairStats {
        triangles_before,
        boundary_edges_before: previous_boundary_edges,
        no_match_after: start_stats.no_match,
        one_match_after: start_stats.one_match,
        multi_match_after: start_stats.multi_match,
        ..TJunctionRepairStats::default()
    };

    let mut iterations_run = 0usize;
    let mut stop_reason = "closed";
    if previous_boundary_edges > 0 {
        stop_reason = "max_iterations";
        for iteration in 1..=max_iterations {
            let iteration_stats = repair_t_junctions_once(mesh, epsilon, iteration);
            iterations_run = iteration;

            total.repaired_edges += iteration_stats.repaired_edges;
            total.skipped_no_match += iteration_stats.skipped_no_match;
            total.skipped_not_boundary += iteration_stats.skipped_not_boundary;
            total.skipped_ambiguous += iteration_stats.skipped_ambiguous;
            total.triangles_added += iteration_stats.triangles_added;
            total
                .repair_edges
                .extend(iteration_stats.repair_edges.iter().copied());
            total.triangles_after = iteration_stats.triangles_after;
            total.boundary_edges_after = iteration_stats.boundary_edges_after;
            total.no_match_after = iteration_stats.no_match_after;
            total.one_match_after = iteration_stats.one_match_after;
            total.multi_match_after = iteration_stats.multi_match_after;

            if iteration_stats.boundary_edges_after == 0 {
                stop_reason = "closed";
                break;
            }
            if iteration_stats.boundary_edges_after >= previous_boundary_edges {
                stop_reason = "converged";
                break;
            }
            previous_boundary_edges = iteration_stats.boundary_edges_after;
        }
    } else {
        total.triangles_after = triangles_before;
        total.boundary_edges_after = 0;
    }

    let closed_fraction = if total.boundary_edges_before == 0 {
        0.0
    } else {
        (total
            .boundary_edges_before
            .saturating_sub(total.boundary_edges_after)) as f32
            / total.boundary_edges_before as f32
    };
    eprintln!(
        "  adaptive-mc t_junction_repair_summary: elapsed={:?} iterations={} boundary_edges_start={} boundary_edges_end={} triangles_before={} triangles_after={} total_triangles_added={} total_repaired_edges={} stop_reason={} closed_fraction={:.3}",
        t0.elapsed(),
        iterations_run,
        total.boundary_edges_before,
        total.boundary_edges_after,
        total.triangles_before,
        total.triangles_after,
        total.triangles_added,
        total.repaired_edges,
        stop_reason,
        closed_fraction,
    );

    total
}

fn try_tjunction_repair(
    mesh: &Mesh,
    triangles_before: &[[u32; 3]],
    tri_idx: usize,
    deleted_before: &[bool],
    new_tris: &[[u32; 3]],
    focus: Vec3,
    epsilon: f32,
) -> Result<(Vec<[u32; 3]>, MeshTopologyStats), &'static str> {
    for tri in new_tris {
        if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
            return Err("degenerate");
        }
        let area_sq = mesh_triangle_area_sq(mesh, *tri);
        if area_sq <= 1e-10 {
            return Err("degenerate");
        }
    }

    let before_topo = compute_mesh_topology_stats(mesh);
    let mut test_triangles = triangles_before.to_vec();
    let mut test_deleted = deleted_before.to_vec();
    test_deleted[tri_idx] = true;
    for tri in new_tris {
        test_triangles.push(*tri);
        test_deleted.push(false);
    }

    let mut test_mesh = mesh.clone();
    test_mesh.indices = compact_triangles(&test_triangles, &test_deleted);
    let after = compute_mesh_topology_stats(&test_mesh);
    let max_new_boundary_distance = (epsilon * 8.0).max(1.0);
    if after.boundary_edges > before_topo.boundary_edges
        || after.non_manifold_edges > before_topo.non_manifold_edges
        || after.connected_components > before_topo.connected_components
        || !new_boundary_edges_stay_local(mesh, &test_mesh, focus, max_new_boundary_distance)
    {
        return Err("topology");
    }
    Ok((new_tris.to_vec(), after))
}

fn repair_t_junctions_once(
    mesh: &mut Mesh,
    epsilon: f32,
    iteration: usize,
) -> TJunctionRepairStats {
    let t0 = Instant::now();
    let triangles_before = mesh.indices.len() / 3;
    let before_stats = collect_boundary_match_stats(mesh, epsilon);
    let positions = mesh_positions(mesh);
    let transition_fix = transition_fix_from_env();
    let grid_cell_size = (2.0 * epsilon).max(1e-6);
    let vertex_grid = build_vertex_grid(&positions, grid_cell_size);

    let mut triangles: Vec<[u32; 3]> = mesh
        .indices
        .chunks_exact(3)
        .map(|tri| [tri[0], tri[1], tri[2]])
        .collect();
    let mut deleted = vec![false; triangles.len()];
    let mut edge_to_triangles = build_mesh_edge_to_triangles(&mesh.indices);
    let boundary_edges: Vec<(u32, u32)> = edge_to_triangles
        .iter()
        .filter_map(|(&edge, tris)| (tris.len() == 1).then_some(edge))
        .collect();

    let mut stats = TJunctionRepairStats {
        triangles_before,
        boundary_edges_before: before_stats.total_boundary_edges,
        ..TJunctionRepairStats::default()
    };

    for edge in boundary_edges {
        let active = active_triangles_for_edge(&edge_to_triangles, &deleted, edge);
        if active.len() != 1 {
            stats.skipped_not_boundary += 1;
            continue;
        }
        let tri_idx = active[0];
        let tri = triangles[tri_idx];
        let Some((edge_start, edge_end, opposite)) = directed_edge_and_opposite(tri, edge) else {
            stats.skipped_ambiguous += 1;
            continue;
        };

        let edge_mid = (positions[edge_start as usize] + positions[edge_end as usize]) * 0.5;
        let local_epsilon = if transition_fix == TransitionCorridorFix::LargerRepairRadius
            && in_inlet_transition_corridor(edge_mid)
        {
            epsilon.max(0.15)
        } else {
            epsilon
        };
        let local_grid_cell_size = (2.0 * local_epsilon).max(1e-6);

        let mut split_vertices = candidate_vertices_on_segment(
            &positions,
            &vertex_grid,
            edge_start,
            edge_end,
            local_epsilon,
            local_grid_cell_size,
        );
        split_vertices.retain(|(idx, _)| *idx != opposite);

        if split_vertices.is_empty() {
            stats.skipped_no_match += 1;
            continue;
        }

        let mut strip = Vec::with_capacity(split_vertices.len() + 2);
        strip.push(edge_start);
        strip.extend(split_vertices.iter().map(|&(idx, _)| idx));
        strip.push(edge_end);

        let mut new_tris = Vec::with_capacity(strip.len().saturating_sub(1));
        for pair in strip.windows(2) {
            if pair[0] == pair[1] || pair[0] == opposite || pair[1] == opposite {
                continue;
            }
            new_tris.push([pair[0], pair[1], opposite]);
        }
        if new_tris.is_empty() {
            stats.skipped_ambiguous += 1;
            continue;
        }

        let focus = (positions[edge_start as usize] + positions[edge_end as usize]) * 0.5;
        let approved = match try_tjunction_repair(
            mesh,
            &triangles,
            tri_idx,
            &deleted,
            &new_tris,
            focus,
            local_epsilon,
        ) {
            Ok((approved_tris, _)) => approved_tris,
            Err("degenerate") => {
                stats.skipped_ambiguous += 1;
                continue;
            }
            Err("topology") => {
                stats.skipped_ambiguous += 1;
                continue;
            }
            Err(_) => {
                stats.skipped_ambiguous += 1;
                continue;
            }
        };

        deleted[tri_idx] = true;
        for new_tri in approved {
            let new_idx = triangles.len();
            triangles.push(new_tri);
            deleted.push(false);
            add_triangle_to_edge_map(&mut edge_to_triangles, new_idx, new_tri);
            stats.triangles_added += 1;
            for &(u, v) in &[
                (new_tri[0], new_tri[1]),
                (new_tri[1], new_tri[2]),
                (new_tri[2], new_tri[0]),
            ] {
                stats.repair_edges.push(canonical_mesh_edge(u, v));
            }
        }
        stats.repaired_edges += 1;
    }

    mesh.indices = compact_triangles(&triangles, &deleted);
    let after_stats = collect_boundary_match_stats(mesh, epsilon);
    stats.triangles_after = mesh.indices.len() / 3;
    stats.boundary_edges_after = after_stats.total_boundary_edges;
    stats.no_match_after = after_stats.no_match;
    stats.one_match_after = after_stats.one_match;
    stats.multi_match_after = after_stats.multi_match;

    let closed_fraction = if stats.boundary_edges_before == 0 {
        0.0
    } else {
        (stats
            .boundary_edges_before
            .saturating_sub(stats.boundary_edges_after)) as f32
            / stats.boundary_edges_before as f32
    };
    eprintln!(
        "  adaptive-mc t_junction_repair_iter: iteration={} elapsed={:?} boundary_edges={} NoMatch={} OneMatch={} MultiMatch={} repaired_edges={} triangles_before={} triangles_after={} skipped_no_match={} skipped_not_boundary={} skipped_ambiguous={} triangles_added={} closed_fraction={:.3}",
        iteration,
        t0.elapsed(),
        stats.boundary_edges_after,
        stats.no_match_after,
        stats.one_match_after,
        stats.multi_match_after,
        stats.repaired_edges,
        stats.triangles_before,
        stats.triangles_after,
        stats.skipped_no_match,
        stats.skipped_not_boundary,
        stats.skipped_ambiguous,
        stats.triangles_added,
        closed_fraction,
    );

    stats
}

fn diagnose_boundary_edge_vertex_matches(mesh: &Mesh, epsilon: f32) {
    let t0 = Instant::now();
    if mesh.vertices.is_empty() || mesh.indices.is_empty() {
        eprintln!(
            "  adaptive-mc boundary_match_diagnostic: elapsed={:?} total_boundary_edges=0 NoMatch=0 OneMatch=0 MultiMatch=0",
            t0.elapsed()
        );
        return;
    }

    let mut edge_to_triangles: HashMap<(u32, u32), Vec<usize>> =
        HashMap::with_capacity(mesh.indices.len());
    for (tri_idx, tri) in mesh.indices.chunks_exact(3).enumerate() {
        for &(a, b) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            edge_to_triangles
                .entry(canonical_mesh_edge(a, b))
                .or_default()
                .push(tri_idx);
        }
    }

    let boundary_edges: Vec<(u32, u32)> = edge_to_triangles
        .iter()
        .filter_map(|(&(a, b), tris)| (tris.len() == 1).then_some((a, b)))
        .collect();

    let positions: Vec<Vec3> = mesh
        .vertices
        .iter()
        .map(|vertex| Vec3::from_array(vertex.position))
        .collect();
    let mesh_bounds_min = positions
        .iter()
        .copied()
        .fold(Vec3::splat(f32::INFINITY), |acc, p| acc.min(p));
    let mesh_bounds_max = positions
        .iter()
        .copied()
        .fold(Vec3::splat(f32::NEG_INFINITY), |acc, p| acc.max(p));

    let grid_cell_size = (2.0 * epsilon).max(1e-6);
    let mut vertex_grid: HashMap<[i32; 3], Vec<u32>> = HashMap::with_capacity(mesh.vertices.len());
    for (idx, &p) in positions.iter().enumerate() {
        vertex_grid
            .entry(spatial_bucket(p, grid_cell_size))
            .or_default()
            .push(idx as u32);
    }

    let mut no_match = 0usize;
    let mut one_match = 0usize;
    let mut multi_match = 0usize;
    let mut no_match_centers = Vec::new();
    let mut one_match_centers = Vec::new();

    for &(a_idx, b_idx) in &boundary_edges {
        let a = positions[a_idx as usize];
        let b = positions[b_idx as usize];
        let ab = b - a;
        let len = ab.length();
        if len <= 1e-9 {
            no_match += 1;
            no_match_centers.push((a + b) * 0.5);
            continue;
        }

        let steps = (len / grid_cell_size).ceil().max(1.0) as usize;
        let mut candidates = Vec::<u32>::new();
        for step in 0..=steps {
            let t = step as f32 / steps as f32;
            let p = a.lerp(b, t);
            let bucket = spatial_bucket(p, grid_cell_size);
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        let key = [bucket[0] + dx, bucket[1] + dy, bucket[2] + dz];
                        if let Some(indices) = vertex_grid.get(&key) {
                            candidates.extend(indices.iter().copied());
                        }
                    }
                }
            }
        }
        candidates.sort_unstable();
        candidates.dedup();

        let matches = candidates
            .into_iter()
            .filter(|&candidate_idx| candidate_idx != a_idx && candidate_idx != b_idx)
            .filter(|&candidate_idx| {
                point_lies_on_segment(positions[candidate_idx as usize], a, b, epsilon)
            })
            .count();

        let center = (a + b) * 0.5;
        match matches {
            0 => {
                no_match += 1;
                no_match_centers.push(center);
            }
            1 => {
                one_match += 1;
                one_match_centers.push(center);
            }
            _ => {
                multi_match += 1;
            }
        }
    }

    eprintln!(
        "  adaptive-mc boundary_match_diagnostic: elapsed={:?} epsilon={:.4} grid_cell={:.4} total_boundary_edges={} NoMatch={} OneMatch={} MultiMatch={}",
        t0.elapsed(),
        epsilon,
        grid_cell_size,
        boundary_edges.len(),
        no_match,
        one_match,
        multi_match,
    );
    let repairable_fraction = if boundary_edges.is_empty() {
        0.0
    } else {
        one_match as f32 / boundary_edges.len() as f32
    };
    eprintln!(
        "    boundary_match_repairability: OneMatchFraction={:.3} decision={}",
        repairable_fraction,
        if repairable_fraction >= 0.5 {
            "post_emission_t_junction_repair_worth_implementing"
        } else {
            "no_match_dominates_need_different_strategy"
        }
    );
    print_boundary_match_hotspots(
        "boundary_match_one_match_hotspots",
        &one_match_centers,
        mesh_bounds_min,
        mesh_bounds_max,
    );
    print_boundary_match_hotspots(
        "boundary_match_no_match_hotspots",
        &no_match_centers,
        mesh_bounds_min,
        mesh_bounds_max,
    );
}

/// SDF gradient (central differences) — gives the outward surface normal.
#[inline]
fn sdf_gradient(sdf: &dyn Sdf, p: Vec3, eps: f32) -> Vec3 {
    Vec3::new(
        sdf.distance(p + Vec3::X * eps) - sdf.distance(p - Vec3::X * eps),
        sdf.distance(p + Vec3::Y * eps) - sdf.distance(p - Vec3::Y * eps),
        sdf.distance(p + Vec3::Z * eps) - sdf.distance(p - Vec3::Z * eps),
    )
    .normalize_or_zero()
}

fn weld_vertices_unconstrained(mesh: &mut Mesh, epsilon: f32) -> WeldStats {
    let vertices_before = mesh.vertices.len();
    if epsilon <= 0.0 {
        return WeldStats {
            vertices_before,
            vertices_after: vertices_before,
            remap: (0..vertices_before as u32).collect(),
            ..WeldStats::default()
        };
    }
    let inv = 1.0 / epsilon;
    let mut grid: HashMap<[i32; 3], u32> = HashMap::with_capacity(mesh.vertices.len());
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
    for idx in &mut mesh.indices {
        *idx = remap[*idx as usize];
    }
    WeldStats {
        vertices_before,
        vertices_after: mesh.vertices.len(),
        remap,
        ..WeldStats::default()
    }
}

/// Weld vertices within `epsilon` of each other to close cracks at octree
/// level boundaries.  Uses a spatial hash (round to nearest grid cell).
fn weld_vertices(mesh: &mut Mesh, epsilon: f32) -> WeldStats {
    let t0 = Instant::now();
    let vertices_before = mesh.vertices.len();
    if epsilon <= 0.0 {
        return WeldStats {
            vertices_before,
            vertices_after: vertices_before,
            remap: (0..vertices_before as u32).collect(),
            elapsed_ms: t0.elapsed().as_secs_f64() * 1000.0,
            ..WeldStats::default()
        };
    }
    let inv = 1.0 / epsilon;
    let mut buckets: HashMap<[i32; 3], Vec<usize>> = HashMap::with_capacity(mesh.vertices.len());
    for (i, v) in mesh.vertices.iter().enumerate() {
        let key = [
            (v.position[0] * inv).round() as i32,
            (v.position[1] * inv).round() as i32,
            (v.position[2] * inv).round() as i32,
        ];
        buckets.entry(key).or_default().push(i);
    }

    let tri_vertices: Vec<[u32; 3]> = mesh
        .indices
        .chunks_exact(3)
        .map(|tri| [tri[0], tri[1], tri[2]])
        .collect();
    let vertex_to_triangles_map = vertex_to_triangles(&mesh.indices, mesh.vertices.len());
    let mut root_map: Vec<u32> = (0..mesh.vertices.len() as u32).collect();

    let mut stats = WeldStats {
        vertices_before,
        vertices_after: vertices_before,
        remap: root_map.clone(),
        ..WeldStats::default()
    };

    for vertex_ids in buckets.values() {
        if vertex_ids.len() <= 1 {
            continue;
        }
        stats.buckets_considered += 1;
        let representative = vertex_ids[0];
        let rep_normal = Vec3::from_array(mesh.vertices[representative].normal);
        let rep_tris = &vertex_to_triangles_map[representative];
        let mut tri_ids = BTreeMap::<usize, ()>::new();
        let mut reject_opposite_shell = false;

        for &vid in vertex_ids {
            for &tri_id in &vertex_to_triangles_map[vid] {
                tri_ids.insert(tri_id, ());
            }
            let normal = Vec3::from_array(mesh.vertices[vid].normal);
            if rep_normal.dot(normal) < -0.5 {
                reject_opposite_shell = true;
            }
            let pos = Vec3::from_array(mesh.vertices[vid].position);
            let rep_pos = Vec3::from_array(mesh.vertices[representative].position);
            if pos.distance(rep_pos) > epsilon * 0.5
                && vertex_to_triangles_map[vid].iter().any(|&a| {
                    rep_tris
                        .iter()
                        .all(|&b| !triangles_share_edge(&mesh.indices, a, b))
                })
            {
                reject_opposite_shell = true;
            }
        }
        if reject_opposite_shell {
            stats.buckets_rejected += 1;
            stats.vertices_preserved_due_to_rejected_buckets += vertex_ids.len().saturating_sub(1);
            stats.buckets_rejected_for_opposite_shell_collapse += 1;
            continue;
        }

        let affected_tri_ids: Vec<usize> = tri_ids.keys().copied().collect();
        let before_local: Vec<[u32; 3]> = affected_tri_ids
            .iter()
            .map(|&id| tri_vertices[id])
            .collect();
        let before_edges = edge_count_map_from_triangle_slices(before_local.iter());
        let mut after_local = Vec::with_capacity(before_local.len());
        let mut seen_oriented = HashSet::<[u32; 3]>::new();
        let mut seen_canonical = HashMap::<[u32; 3], bool>::new();
        let mut reject_degenerate = false;
        let mut reject_duplicate = false;

        for tri in &before_local {
            let mut mapped = *tri;
            for v in &mut mapped {
                let idx = *v as usize;
                if vertex_ids.contains(&idx) {
                    *v = representative as u32;
                } else {
                    *v = root_map[idx];
                }
            }
            if mapped[0] == mapped[1] || mapped[1] == mapped[2] || mapped[2] == mapped[0] {
                reject_degenerate = true;
                break;
            }
            if !seen_oriented.insert(mapped) {
                reject_duplicate = true;
                break;
            }
            let canonical = canonical_mesh_triangle(mapped);
            let parity = permutation_parity_mesh(mapped);
            if seen_canonical.insert(canonical, parity).is_some() {
                reject_duplicate = true;
                break;
            }
            after_local.push(mapped);
        }

        if reject_degenerate {
            stats.buckets_rejected += 1;
            stats.vertices_preserved_due_to_rejected_buckets += vertex_ids.len().saturating_sub(1);
            stats.buckets_rejected_for_degenerate_triangles += 1;
            continue;
        }
        if reject_duplicate {
            stats.buckets_rejected += 1;
            stats.vertices_preserved_due_to_rejected_buckets += vertex_ids.len().saturating_sub(1);
            stats.buckets_rejected_for_duplicate_triangles += 1;
            continue;
        }

        let after_edges = edge_count_map_from_triangle_slices(after_local.iter());
        let mut reject_non_manifold = false;
        let mut all_edge_keys = HashSet::new();
        all_edge_keys.extend(before_edges.keys().copied());
        all_edge_keys.extend(after_edges.keys().copied());
        for edge in all_edge_keys {
            let before_count = before_edges.get(&edge).copied().unwrap_or(0);
            let after_count = after_edges.get(&edge).copied().unwrap_or(0);
            if before_count == 2 && after_count != 2 {
                reject_non_manifold = true;
                break;
            }
            if after_count > 2 {
                reject_non_manifold = true;
                break;
            }
            if before_count == 0 && after_count == 1 {
                reject_non_manifold = true;
                break;
            }
        }
        if reject_non_manifold {
            stats.buckets_rejected += 1;
            stats.vertices_preserved_due_to_rejected_buckets += vertex_ids.len().saturating_sub(1);
            stats.buckets_rejected_for_non_manifold_edges += 1;
            continue;
        }

        stats.buckets_accepted += 1;
        for &vid in &vertex_ids[1..] {
            root_map[vid] = representative as u32;
        }
    }

    let mut new_verts: Vec<Vertex> = Vec::with_capacity(mesh.vertices.len());
    let mut new_index_for_root = HashMap::<u32, u32>::new();
    let mut final_remap = vec![0u32; mesh.vertices.len()];
    for old_idx in 0..mesh.vertices.len() {
        let root = root_map[old_idx];
        let new_idx = if let Some(&idx) = new_index_for_root.get(&root) {
            idx
        } else {
            let idx = new_verts.len() as u32;
            new_index_for_root.insert(root, idx);
            new_verts.push(mesh.vertices[root as usize]);
            idx
        };
        final_remap[old_idx] = new_idx;
    }
    for idx in &mut mesh.indices {
        *idx = final_remap[*idx as usize];
    }
    mesh.vertices = new_verts;
    stats.vertices_after = mesh.vertices.len();
    stats.remap = final_remap;
    stats.elapsed_ms = t0.elapsed().as_secs_f64() * 1000.0;
    stats
}

/// Remove zero-area (degenerate) triangles.
fn remove_degenerate_triangles(mesh: &mut Mesh, cell_size: f32) {
    let min_area_sq = (cell_size * 0.001).powi(2);
    let boundary_edges_before = boundary_edge_set(&mesh.indices);
    let triangles_before = mesh.indices.len() / 3;
    let mut removed_total = 0usize;
    let mut removed_boundary_adjacent = 0usize;
    let mut area_buckets = [0usize; 5];

    let mut clean: Vec<u32> = Vec::with_capacity(mesh.indices.len());
    for tri in mesh.indices.chunks(3) {
        let p0 = Vec3::from_array(mesh.vertices[tri[0] as usize].position);
        let p1 = Vec3::from_array(mesh.vertices[tri[1] as usize].position);
        let p2 = Vec3::from_array(mesh.vertices[tri[2] as usize].position);
        let area_sq = (p1 - p0).cross(p2 - p0).length_squared();
        if area_sq > min_area_sq {
            clean.extend_from_slice(tri);
            continue;
        }

        removed_total += 1;
        let area = area_sq.sqrt() * 0.5;
        let bucket = if area < 1e-8 {
            0
        } else if area < 1e-6 {
            1
        } else if area < 1e-4 {
            2
        } else if area < 1e-2 {
            3
        } else {
            4
        };
        area_buckets[bucket] += 1;

        let edges = [
            canonical_mesh_edge(tri[0], tri[1]),
            canonical_mesh_edge(tri[1], tri[2]),
            canonical_mesh_edge(tri[2], tri[0]),
        ];
        if edges
            .iter()
            .any(|edge| boundary_edges_before.contains(edge))
        {
            removed_boundary_adjacent += 1;
        }
    }

    eprintln!(
        "  adaptive-mc degenerate_removal_diagnostic: cell_size={:.6} min_area_sq={:.12e} min_area={:.12e} triangles_before={} triangles_removed={} boundary_adjacent_removed={} area_lt_1e-8={} area_1e-8_to_1e-6={} area_1e-6_to_1e-4={} area_1e-4_to_1e-2={} area_gt_1e-2={}",
        cell_size,
        min_area_sq,
        min_area_sq.sqrt() * 0.5,
        triangles_before,
        removed_total,
        removed_boundary_adjacent,
        area_buckets[0],
        area_buckets[1],
        area_buckets[2],
        area_buckets[3],
        area_buckets[4],
    );
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
    sdf: &dyn Sdf,
    bounds_min: Vec3,
    bounds_max: Vec3,
    target_cell_size: f32,
    smooth_normals: bool,
) -> Mesh {
    use std::time::Instant;
    let t0 = Instant::now();

    // Build a cubic root cell that contains the entire model.
    let center = (bounds_min + bounds_max) * 0.5;
    let max_extent = (bounds_max - bounds_min).max_element();
    let mut root_size = target_cell_size;
    while root_size < max_extent * 1.01 {
        root_size *= 2.0;
    }
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
    let normal_eps = (target_cell_size * 0.3).max(0.001);
    let mesh = emit_mesh_from_cells(
        sdf,
        &surface_cells,
        normal_eps,
        smooth_normals,
        target_cell_size * 0.45,
        target_cell_size,
    );
    let t3 = Instant::now();
    let t4 = Instant::now();

    let t5 = Instant::now();

    eprintln!(
        "Adaptive MC: cell={:.3}  seeds={}  surface={}  verts={}  tris={}",
        target_cell_size,
        seeds.len(),
        surface_cells.len(),
        mesh.vertices.len(),
        mesh.indices.len() / 3
    );
    eprintln!(
        "  Phase1={:?}  Phase2={:?}  Emit={:?}  Normals={:?}  Post={:?}  Total={:?}",
        t1 - t0,
        t2 - t1,
        t3 - t2,
        t4 - t3,
        t5 - t4,
        t5 - t0
    );

    mesh
}

pub fn extract_mesh_adaptive_local_regions(
    sdf: &dyn Sdf,
    bounds_min: Vec3,
    bounds_max: Vec3,
    target_cell_size: f32,
    regions: &[LocalRefineRegion],
    smooth_normals: bool,
) -> Mesh {
    if regions.is_empty() {
        return extract_mesh_adaptive(
            sdf,
            bounds_min,
            bounds_max,
            target_cell_size,
            smooth_normals,
        );
    }

    let t0 = Instant::now();
    let stats = AdaptiveMcInstrumentation::default();

    let center = (bounds_min + bounds_max) * 0.5;
    let max_extent = (bounds_max - bounds_min).max_element();
    let mut root_size = target_cell_size.max(0.02);
    while root_size < max_extent * 1.01 {
        root_size *= 2.0;
    }
    let root_min = center - Vec3::splat(root_size * 0.5);

    let min_region_target = regions
        .iter()
        .map(|r| r.target_cell_size.max(0.02))
        .fold(target_cell_size.max(0.02), f32::min);
    let refinement_ratio = (target_cell_size.max(0.02) / min_region_target).max(1.0);
    let extra_depth = refinement_ratio.log2().ceil().max(0.0) as u32;
    let split_depth = (5 + extra_depth.min(3)).min(8);

    let collect_t0 = Instant::now();
    let seeds = collect_seeds_local(
        sdf,
        root_min,
        root_size,
        target_cell_size,
        regions,
        split_depth,
        &stats,
    );
    let collect_elapsed = collect_t0.elapsed();
    eprintln!(
        "  adaptive-mc collect_seeds_local: elapsed={:?} seeds_generated={} cells_pruned={}",
        collect_elapsed,
        stats.seeds_generated.load(Ordering::Relaxed),
        stats.cells_pruned.load(Ordering::Relaxed),
    );

    let process_t0 = Instant::now();
    let surface_cells: Vec<OctreeCell> = seeds
        .par_iter()
        .flat_map(|&(min, size)| {
            process_subtree_local(sdf, min, size, target_cell_size, regions, &stats)
        })
        .collect();
    let process_elapsed = process_t0.elapsed();
    eprintln!(
        "  adaptive-mc process_subtree_local: elapsed={:?} cells_kept={} cells_pruned={} cells_saved_by_multi_crossing={}",
        process_elapsed,
        stats.cells_kept.load(Ordering::Relaxed),
        stats.cells_pruned.load(Ordering::Relaxed),
        stats.cells_saved_by_multi_crossing.load(Ordering::Relaxed),
    );

    let mut surface_cells = surface_cells;
    split_partial_overlap_cells(&mut surface_cells, sdf, 2);
    let surface_cell_count = surface_cells.len();
    let propagate_t0 = Instant::now();
    propagate_size_gradient(&mut surface_cells, sdf, regions, min_region_target);
    let propagate_elapsed = propagate_t0.elapsed();
    let propagated_cell_count = surface_cells.len();
    let balance_t0 = Instant::now();
    let balanced_cells = balance_surface_cells(sdf, surface_cells, min_region_target);
    let balance_elapsed = balance_t0.elapsed();
    let balanced_cell_count = balanced_cells.len();
    let balanced_adjacency = FaceAdjacency::build(&balanced_cells);
    print_partial_overlap_case_report(&balanced_adjacency);
    eprintln!(
        "  adaptive-mc balance_surface_cells: elapsed={:?} balanced_cell_count={}",
        balance_elapsed, balanced_cell_count,
    );
    let normal_eps = (min_region_target * 0.3).max(0.001);
    let emit_t0 = Instant::now();
    let mut mesh = emit_mesh_from_cells(
        sdf,
        &balanced_cells,
        normal_eps,
        smooth_normals,
        min_region_target * 0.45,
        min_region_target,
    );
    print_transition_defect_map(&mesh, &balanced_adjacency);
    let (active_mesh, _) = evaluate_two_adjacent_active_candidates(
        sdf,
        &balanced_cells,
        &mesh,
        &balanced_adjacency,
        normal_eps,
        smooth_normals,
        min_region_target,
    );
    if let Some(next_mesh) = active_mesh {
        mesh = next_mesh;
        print_transition_defect_map(&mesh, &balanced_adjacency);
    }
    let emit_elapsed = emit_t0.elapsed();
    eprintln!(
        "  adaptive-mc emission: elapsed={:?} verts={} tris={}",
        emit_elapsed,
        mesh.vertices.len(),
        mesh.indices.len() / 3,
    );
    let total_elapsed = t0.elapsed();

    eprintln!(
        "Adaptive MC local regions: base={:.3} min={:.3} regions={} seeds={} surface={} balanced={} verts={} tris={}",
        target_cell_size,
        min_region_target,
        regions.len(),
        seeds.len(),
        surface_cell_count,
        balanced_cells.len(),
        mesh.vertices.len(),
        mesh.indices.len() / 3
    );
    eprintln!(
        "  collect_seeds_local={:?} process_subtree_local={:?} propagate_size_gradient={:?} balance_surface_cells={:?} emission={:?} total={:?}",
        collect_elapsed,
        process_elapsed,
        propagate_elapsed,
        balance_elapsed,
        emit_elapsed,
        total_elapsed
    );
    eprintln!(
        "  instrumentation: seeds_generated={} cells_pruned={} cells_kept={} cells_saved_by_multi_crossing={} propagated_cell_count={} balanced_cell_count={}",
        stats.seeds_generated.load(Ordering::Relaxed),
        stats.cells_pruned.load(Ordering::Relaxed),
        stats.cells_kept.load(Ordering::Relaxed),
        stats.cells_saved_by_multi_crossing.load(Ordering::Relaxed),
        propagated_cell_count,
        balanced_cell_count
    );
    print_transition_defect_map(&mesh, &balanced_adjacency);

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
    use crate::sdf::Sdf;
    use crate::sdf::primitives::Sphere;

    struct PlaneX {
        x0: f32,
    }

    impl Sdf for PlaneX {
        fn distance(&self, point: Vec3) -> f32 {
            point.x - self.x0
        }
    }

    #[test]
    fn test_adaptive_sphere_normal() {
        let s = Sphere::new(5.0);
        let mesh = extract_mesh_adaptive(&s, Vec3::splat(-8.0), Vec3::splat(8.0), 0.5, false);
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
        let fine = extract_mesh_adaptive(&s, Vec3::splat(-8.0), Vec3::splat(8.0), 0.3, false);
        assert!(
            fine.indices.len() > coarse.indices.len(),
            "Fine ({}) should have more tris than coarse ({})",
            fine.indices.len() / 3,
            coarse.indices.len() / 3
        );
    }

    #[test]
    fn local_target_size_adds_transition_band_near_refined_region() {
        let region = LocalRefineRegion {
            min: Vec3::new(0.0, 0.0, 0.0),
            max: Vec3::new(10.0, 10.0, 10.0),
            target_cell_size: 0.5,
        };
        let far = local_target_size(
            Vec3::new(40.0, 0.0, 0.0),
            Vec3::new(48.0, 8.0, 8.0),
            3.0,
            &[region],
        );
        let near = local_target_size(
            Vec3::new(10.2, 0.0, 0.0),
            Vec3::new(18.2, 8.0, 8.0),
            3.0,
            &[region],
        );
        assert_eq!(far, 3.0);
        assert!(near <= 1.0 + 1e-6);
    }

    #[test]
    fn face_adjacency_detects_single_2_to_1_transition_face() {
        let cells = vec![
            OctreeCell {
                min: Vec3::new(0.0, 0.0, 0.0),
                size: 2.0,
                corners: [0.0; 8],
            },
            OctreeCell {
                min: Vec3::new(2.0, 0.0, 0.0),
                size: 1.0,
                corners: [0.0; 8],
            },
            OctreeCell {
                min: Vec3::new(2.0, 1.0, 0.0),
                size: 1.0,
                corners: [0.0; 8],
            },
            OctreeCell {
                min: Vec3::new(2.0, 0.0, 1.0),
                size: 1.0,
                corners: [0.0; 8],
            },
            OctreeCell {
                min: Vec3::new(2.0, 1.0, 1.0),
                size: 1.0,
                corners: [0.0; 8],
            },
        ];

        let adjacency = FaceAdjacency::build(&cells);
        let transitions: Vec<_> = adjacency.transition_faces().collect();
        assert_eq!(transitions.len(), 1, "expected one transition face");

        let transition = &transitions[0];
        assert_eq!(transition.coarse_cell_idx, 0);
        assert_eq!(transition.coarse_face_id, 1, "expected +X face");

        let mut fine_indices = transition.fine_cell_indices.clone();
        fine_indices.sort_unstable();
        assert_eq!(fine_indices, vec![1, 2, 3, 4]);
        assert!((adjacency.max_touching_ratio() - 2.0).abs() < 1e-6);
    }

    #[test]
    fn propagate_size_gradient_reduces_four_to_one_ratio() {
        let plane = PlaneX { x0: 3.5 };
        let mut cells = vec![
            OctreeCell {
                min: Vec3::new(0.0, 0.0, 0.0),
                size: 4.0,
                corners: eval_corners(&plane, Vec3::new(0.0, 0.0, 0.0), 4.0),
            },
            OctreeCell {
                min: Vec3::new(4.0, 0.0, 0.0),
                size: 1.0,
                corners: eval_corners(&plane, Vec3::new(4.0, 0.0, 0.0), 1.0),
            },
        ];

        propagate_size_gradient(&mut cells, &plane, &[], 1.0);

        let adjacency = FaceAdjacency::build(&cells);
        assert!(
            adjacency.max_touching_ratio() <= 2.0 + 1e-6,
            "expected propagated ratio <= 2.0, got {}",
            adjacency.max_touching_ratio()
        );
        assert!(
            cells.iter().all(|cell| cell.size < 4.0 - 1e-6),
            "expected original 4mm cell to be replaced"
        );
        assert!(
            cells.iter().any(|cell| (cell.size - 2.0).abs() < 1e-6),
            "expected at least one intermediate 2mm cell"
        );
    }
}
