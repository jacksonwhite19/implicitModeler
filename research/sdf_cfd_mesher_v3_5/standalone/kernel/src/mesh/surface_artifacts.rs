use crate::mesh::sdf_surface::{SdfSurfaceMesh, SurfaceEdgeClassification};
use crate::mesh::surface_features::{SurfaceFeatureGraph, SurfaceFeatureType};
use crate::sdf::Sdf;
use crate::sdf::cfd_query::{GeometryQueryKernel, SdfCfdQueryOptions};
use crate::sdf::procedural_features::{CapRole, ProceduralFeatureQueryOptions, WingSurfaceSide};
use glam::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::sync::Arc;
use std::time::Instant;

pub const SURFACE_ARTIFACT_SCHEMA_VERSION: &str = "surface_artifact_diagnosis.v7";

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct SurfaceArtifactConfig {
    pub severe_normal_angle_degrees: f64,
    pub fold_normal_dot_threshold: f64,
    pub sector_query_tolerance: f64,
    pub feature_match_tolerance: f64,
    pub min_local_thickness: f64,
    pub max_projection_displacement_factor: f64,
    pub suspect_aspect_ratio: f64,
}

impl Default for SurfaceArtifactConfig {
    fn default() -> Self {
        Self {
            severe_normal_angle_degrees: 115.0,
            fold_normal_dot_threshold: -0.25,
            sector_query_tolerance: 6.0,
            feature_match_tolerance: 8.0,
            min_local_thickness: 0.5,
            max_projection_displacement_factor: 5.0,
            suspect_aspect_ratio: 80.0,
        }
    }
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct SurfaceArtifactDiagnostics {
    pub schema_version: String,
    pub mesh_hash: String,
    pub normal_alignment: NormalAlignmentDiagnostics,
    pub fold_detection: FoldDetectionDiagnostics,
    pub self_intersections: SelfIntersectionDiagnostics,
    pub sector_consistency: SectorConsistencyDiagnostics,
    pub projection_branch_consistency: ProjectionBranchDiagnostics,
    pub feature_parameter_consistency: FeatureParameterDiagnostics,
    pub artifact_regions: Vec<ArtifactRegion>,
    pub acceptance: GeometricAcceptance,
    pub timings: ArtifactTiming,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct NormalAlignmentDiagnostics {
    pub triangle_count: usize,
    pub evaluated: usize,
    pub max_angle_degrees: f64,
    pub p95_angle_degrees: f64,
    pub severe_disagreements: usize,
    pub negative_dot_triangles: usize,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct FoldDetectionDiagnostics {
    pub checked_interior_edges: usize,
    pub fold_edges: usize,
    pub unexplained_fold_edges: usize,
    pub worst_adjacent_normal_dot: f64,
    pub metric: String,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct SelfIntersectionDiagnostics {
    pub checked_pairs: usize,
    pub skipped_adjacent_pairs: usize,
    pub intersecting_pairs: usize,
    pub sample_pairs: Vec<[u64; 2]>,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct SectorConsistencyDiagnostics {
    pub queried_vertices: usize,
    pub unsupported_vertices: usize,
    pub side_counts: BTreeMap<String, usize>,
    pub cap_role_counts: BTreeMap<String, usize>,
    pub feature_counts: BTreeMap<String, usize>,
    pub upper_lower_without_cap_triangles: usize,
    pub unexpected_cap_surface_triangles: usize,
    pub feature_distance_violations: usize,
    pub sector_crossing_vertices: usize,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct ProjectionBranchDiagnostics {
    pub vertices_checked: usize,
    pub max_projection_displacement: f64,
    pub displacement_limit_violations: usize,
    pub displacement_over_local_thickness: usize,
    pub notes: Vec<String>,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct FeatureParameterDiagnostics {
    pub curves_checked: usize,
    pub nonmonotonic_curves: usize,
    pub large_jump_curves: usize,
    pub endpoint_mismatch_curves: usize,
    pub curve_results: Vec<FeatureCurveParameterResult>,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct FeatureCurveParameterResult {
    pub curve_id: u64,
    pub name: String,
    pub feature_type: String,
    pub vertex_count: usize,
    pub monotonic: bool,
    pub max_parameter_jump: f64,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct ArtifactRegion {
    pub stable_region_id: String,
    pub triangle_count: usize,
    pub vertex_count: usize,
    pub area: f64,
    pub bbox_min: [f64; 3],
    pub bbox_max: [f64; 3],
    pub centroid: [f64; 3],
    pub nearest_procedural_feature: Option<String>,
    pub surface_sector_composition: BTreeMap<String, usize>,
    pub span_parameter_range: Option<[f64; 2]>,
    pub chord_parameter_range: Option<[f64; 2]>,
    pub max_normal_angle_degrees: f64,
    pub fold_indicators: usize,
    pub self_intersection_pairs: usize,
    pub likely_cause: String,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct GeometricAcceptance {
    pub topology_valid: bool,
    pub self_intersections_zero: bool,
    pub folds_zero: bool,
    pub normal_alignment_accepted: bool,
    pub sector_consistency_accepted: bool,
    pub projection_branch_accepted: bool,
    pub feature_parameters_accepted: bool,
    pub accepted_for_macro_shell_prereview: bool,
    pub failure_reasons: Vec<String>,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct ArtifactTiming {
    pub normal_alignment_ms: f64,
    pub fold_detection_ms: f64,
    pub self_intersection_ms: f64,
    pub sector_validation_ms: f64,
    pub feature_parameter_ms: f64,
    pub region_detection_ms: f64,
    pub total_ms: f64,
}

#[derive(Clone, Debug)]
struct TriangleSignals {
    normal_angle: f64,
    severe_normal: bool,
    sector_inconsistent: bool,
    self_intersection: bool,
    poor_quality: bool,
    fold_indicator: bool,
}

#[derive(Clone, Debug)]
struct VertexSectorSample {
    side: Option<WingSurfaceSide>,
    cap_role: Option<CapRole>,
    sector_id: Option<String>,
    feature_id: Option<String>,
    distance_to_feature: Option<f64>,
    span: Option<f64>,
    chord: Option<f64>,
}

pub fn diagnose_surface_artifacts(
    sdf: Arc<dyn Sdf>,
    mesh: &SdfSurfaceMesh,
    graph: Option<&SurfaceFeatureGraph>,
    config: &SurfaceArtifactConfig,
) -> SurfaceArtifactDiagnostics {
    let total_start = Instant::now();
    let mut timings = ArtifactTiming::default();
    let mut signals = vec![
        TriangleSignals {
            normal_angle: 0.0,
            severe_normal: false,
            sector_inconsistent: false,
            self_intersection: false,
            poor_quality: false,
            fold_indicator: false,
        };
        mesh.triangles.len()
    ];

    let start = Instant::now();
    let normal_alignment =
        normal_alignment_diagnostics(Arc::clone(&sdf), mesh, config, &mut signals);
    timings.normal_alignment_ms = elapsed_ms(start);

    let start = Instant::now();
    let fold_detection = fold_detection_diagnostics(mesh, config, &mut signals);
    timings.fold_detection_ms = elapsed_ms(start);

    let start = Instant::now();
    let self_intersections = self_intersection_diagnostics(mesh, &mut signals);
    timings.self_intersection_ms = elapsed_ms(start);

    let start = Instant::now();
    let (sector_consistency, vertex_samples) =
        sector_consistency_diagnostics(sdf.as_ref(), mesh, config, &mut signals);
    timings.sector_validation_ms = elapsed_ms(start);

    let projection_branch_consistency =
        projection_branch_diagnostics(mesh, config, &vertex_samples);

    let start = Instant::now();
    let feature_parameter_consistency = graph
        .map_or_else(FeatureParameterDiagnostics::default, |graph| {
            feature_parameter_diagnostics(sdf.as_ref(), mesh, graph, config)
        });
    timings.feature_parameter_ms = elapsed_ms(start);

    let start = Instant::now();
    let artifact_regions =
        build_artifact_regions(sdf.as_ref(), mesh, config, &signals, &vertex_samples);
    timings.region_detection_ms = elapsed_ms(start);
    timings.total_ms = elapsed_ms(total_start);

    let acceptance = acceptance(
        mesh,
        &normal_alignment,
        &fold_detection,
        &self_intersections,
        &sector_consistency,
        &projection_branch_consistency,
        &feature_parameter_consistency,
    );

    SurfaceArtifactDiagnostics {
        schema_version: SURFACE_ARTIFACT_SCHEMA_VERSION.to_string(),
        mesh_hash: mesh.mesh_hash(),
        normal_alignment,
        fold_detection,
        self_intersections,
        sector_consistency,
        projection_branch_consistency,
        feature_parameter_consistency,
        artifact_regions,
        acceptance,
        timings,
    }
}

fn normal_alignment_diagnostics(
    sdf: Arc<dyn Sdf>,
    mesh: &SdfSurfaceMesh,
    config: &SurfaceArtifactConfig,
    signals: &mut [TriangleSignals],
) -> NormalAlignmentDiagnostics {
    let kernel = GeometryQueryKernel::from_sdf(sdf);
    let mut angles = Vec::with_capacity(mesh.triangles.len());
    let mut severe = 0;
    let mut negative = 0;
    for (idx, tri) in mesh.triangles.iter().enumerate() {
        let centroid = triangle_centroid(mesh, tri.vertices);
        let query = kernel.query_point(
            [centroid.x as f64, centroid.y as f64, centroid.z as f64],
            SdfCfdQueryOptions {
                distance: true,
                gradient: true,
                ..SdfCfdQueryOptions::distance_only()
            },
        );
        let geom = Vec3::from_array(tri.geometric_normal.map(|v| v as f32)).normalize_or_zero();
        let Some(unit) = query.unit_normal else {
            continue;
        };
        let sdf_normal =
            Vec3::new(unit[0] as f32, unit[1] as f32, unit[2] as f32).normalize_or_zero();
        if geom.length_squared() == 0.0 || sdf_normal.length_squared() == 0.0 {
            continue;
        }
        let dot = geom.dot(sdf_normal).clamp(-1.0, 1.0);
        let angle = dot.acos().to_degrees() as f64;
        if dot < 0.0 {
            negative += 1;
        }
        if angle >= config.severe_normal_angle_degrees {
            severe += 1;
            signals[idx].severe_normal = true;
        }
        signals[idx].normal_angle = angle;
        angles.push(angle);
    }
    angles.sort_by(f64::total_cmp);
    NormalAlignmentDiagnostics {
        triangle_count: mesh.triangles.len(),
        evaluated: angles.len(),
        max_angle_degrees: angles.last().copied().unwrap_or(0.0),
        p95_angle_degrees: percentile(&angles, 0.95),
        severe_disagreements: severe,
        negative_dot_triangles: negative,
    }
}

fn fold_detection_diagnostics(
    mesh: &SdfSurfaceMesh,
    config: &SurfaceArtifactConfig,
    signals: &mut [TriangleSignals],
) -> FoldDetectionDiagnostics {
    let tri_by_id = mesh
        .triangles
        .iter()
        .enumerate()
        .map(|(idx, tri)| (tri.id, idx))
        .collect::<BTreeMap<_, _>>();
    let mut checked = 0;
    let mut fold_edges = 0;
    let mut unexplained = 0;
    let mut worst_dot = 1.0;
    for edge in &mesh.edges {
        if edge.incident_triangles.len() != 2 {
            continue;
        }
        checked += 1;
        let a_idx = tri_by_id[&edge.incident_triangles[0]];
        let b_idx = tri_by_id[&edge.incident_triangles[1]];
        let a = Vec3::from_array(mesh.triangles[a_idx].geometric_normal.map(|v| v as f32))
            .normalize_or_zero();
        let b = Vec3::from_array(mesh.triangles[b_idx].geometric_normal.map(|v| v as f32))
            .normalize_or_zero();
        if a.length_squared() == 0.0 || b.length_squared() == 0.0 {
            continue;
        }
        let dot = a.dot(b).clamp(-1.0, 1.0) as f64;
        worst_dot = f64::min(worst_dot, dot);
        if dot <= config.fold_normal_dot_threshold {
            fold_edges += 1;
            signals[a_idx].fold_indicator = true;
            signals[b_idx].fold_indicator = true;
            if !matches!(
                edge.classification,
                SurfaceEdgeClassification::CandidateSharp
                    | SurfaceEdgeClassification::CandidateProvenanceBoundary
                    | SurfaceEdgeClassification::CandidatePatchBoundary
            ) {
                unexplained += 1;
            }
        }
    }
    FoldDetectionDiagnostics {
        checked_interior_edges: checked,
        fold_edges,
        unexplained_fold_edges: unexplained,
        worst_adjacent_normal_dot: worst_dot,
        metric: "adjacent geometric-normal dot below configured threshold; candidate feature/provenance/patch edges are tracked separately from unexplained folds".to_string(),
    }
}

fn self_intersection_diagnostics(
    mesh: &SdfSurfaceMesh,
    signals: &mut [TriangleSignals],
) -> SelfIntersectionDiagnostics {
    let mut checked = 0;
    let mut skipped = 0;
    let mut hits = 0;
    let mut sample_pairs = Vec::new();
    let aabbs = mesh
        .triangles
        .iter()
        .map(|tri| tri_aabb(mesh, tri.vertices))
        .collect::<Vec<_>>();
    let mut order = (0..mesh.triangles.len()).collect::<Vec<_>>();
    order.sort_by(|a, b| {
        aabbs[*a].0[0]
            .total_cmp(&aabbs[*b].0[0])
            .then_with(|| a.cmp(b))
    });
    for (oi, i) in order.iter().copied().enumerate() {
        for j in order.iter().copied().skip(oi + 1) {
            if aabbs[j].0[0] > aabbs[i].1[0] {
                break;
            }
            if triangles_share_vertex(mesh.triangles[i].vertices, mesh.triangles[j].vertices) {
                skipped += 1;
                continue;
            }
            if !aabb_overlap(aabbs[i], aabbs[j]) {
                continue;
            }
            checked += 1;
            if tri_tri_intersect(mesh, mesh.triangles[i].vertices, mesh.triangles[j].vertices) {
                hits += 1;
                signals[i].self_intersection = true;
                signals[j].self_intersection = true;
                if sample_pairs.len() < 64 {
                    sample_pairs.push([mesh.triangles[i].id, mesh.triangles[j].id]);
                }
            }
        }
    }
    SelfIntersectionDiagnostics {
        checked_pairs: checked,
        skipped_adjacent_pairs: skipped,
        intersecting_pairs: hits,
        sample_pairs,
    }
}

fn sector_consistency_diagnostics(
    sdf: &dyn Sdf,
    mesh: &SdfSurfaceMesh,
    config: &SurfaceArtifactConfig,
    signals: &mut [TriangleSignals],
) -> (SectorConsistencyDiagnostics, Vec<VertexSectorSample>) {
    let options = ProceduralFeatureQueryOptions {
        tolerance: config.sector_query_tolerance,
        ..ProceduralFeatureQueryOptions::diagnostic()
    };
    let mut samples = Vec::with_capacity(mesh.vertices.len());
    let mut side_counts = BTreeMap::new();
    let mut cap_counts = BTreeMap::new();
    let mut feature_counts = BTreeMap::new();
    let mut unsupported = 0;
    let mut feature_distance_violations = 0;
    for vertex in &mesh.vertices {
        let p = Vec3::from_array(vertex.position.map(|v| v as f32));
        let q = sdf.query_procedural_feature(p, &options);
        if q.support_status == crate::sdf::cfd_query::QueryValueStatus::Unsupported {
            unsupported += 1;
        }
        if q.feature_id.is_some()
            && q.distance_to_feature
                .is_some_and(|d| d > config.feature_match_tolerance)
            && q.cap_role
                .is_some_and(|role| !matches!(role, CapRole::None | CapRole::Unknown))
        {
            feature_distance_violations += 1;
        }
        *side_counts.entry(format!("{:?}", q.side)).or_default() += 1;
        *cap_counts.entry(format!("{:?}", q.cap_role)).or_default() += 1;
        *feature_counts
            .entry(q.feature_id.clone().unwrap_or_else(|| "none".to_string()))
            .or_default() += 1;
        samples.push(VertexSectorSample {
            side: q.side,
            cap_role: q.cap_role,
            sector_id: q.normal_sector.map(|s| s.stable_id),
            feature_id: q.feature_id,
            distance_to_feature: q.distance_to_feature,
            span: q.span_parameter,
            chord: q.chord_parameter.or(q.airfoil_parameter),
        });
    }
    let mut upper_lower_without_cap = 0;
    let mut unexpected_cap_surface = 0;
    let mut sector_crossing_vertices = BTreeSet::new();
    for (idx, tri) in mesh.triangles.iter().enumerate() {
        let tri_samples = tri
            .vertices
            .iter()
            .map(|id| &samples[*id as usize])
            .collect::<Vec<_>>();
        let has_upper = tri_samples
            .iter()
            .any(|s| s.side == Some(WingSurfaceSide::Upper));
        let has_lower = tri_samples
            .iter()
            .any(|s| s.side == Some(WingSurfaceSide::Lower));
        let has_side_cap = tri_samples
            .iter()
            .any(|s| s.side == Some(WingSurfaceSide::Cap));
        let has_cap_role = tri_samples.iter().any(|s| {
            s.cap_role
                .is_some_and(|c| !matches!(c, CapRole::None | CapRole::Unknown))
        });
        let has_upper_lower = has_upper && has_lower;
        let has_cap = has_side_cap || has_cap_role;
        let chord_max = tri_samples
            .iter()
            .filter_map(|s| s.chord)
            .fold(0.0, f64::max);
        let chord_min = tri_samples
            .iter()
            .filter_map(|s| s.chord)
            .fold(1.0, f64::min);
        let span_min = tri_samples
            .iter()
            .filter_map(|s| s.span)
            .fold(1.0, f64::min);
        let span_max = tri_samples
            .iter()
            .filter_map(|s| s.span)
            .fold(0.0, f64::max);
        let leading_edge_continuity = chord_max < 0.12;
        let trailing_edge_crossing = chord_min > 0.92;
        let tip_transition_continuity = span_min < 0.03
            || span_max > 0.97
            || tri_samples.iter().any(|s| {
                s.feature_id
                    .as_deref()
                    .is_some_and(|id| id.contains(".tip."))
            });
        if has_upper_lower
            && !has_cap
            && !leading_edge_continuity
            && !tip_transition_continuity
            && trailing_edge_crossing
        {
            if std::env::var_os("SDF_ARTIFACT_DEBUG_SECTORS").is_some() {
                eprintln!(
                    "sector crossing tri={idx} chord_min={chord_min:.6} chord_max={chord_max:.6} samples={tri_samples:?}"
                );
            }
            upper_lower_without_cap += 1;
            signals[idx].sector_inconsistent = true;
            for id in tri.vertices {
                sector_crossing_vertices.insert(id);
            }
        }
        if has_side_cap && !has_cap_role {
            unexpected_cap_surface += 1;
            signals[idx].sector_inconsistent = true;
        }
    }
    (
        SectorConsistencyDiagnostics {
            queried_vertices: mesh.vertices.len(),
            unsupported_vertices: unsupported,
            side_counts,
            cap_role_counts: cap_counts,
            feature_counts,
            upper_lower_without_cap_triangles: upper_lower_without_cap,
            unexpected_cap_surface_triangles: unexpected_cap_surface,
            feature_distance_violations,
            sector_crossing_vertices: sector_crossing_vertices.len(),
        },
        samples,
    )
}

fn projection_branch_diagnostics(
    mesh: &SdfSurfaceMesh,
    config: &SurfaceArtifactConfig,
    samples: &[VertexSectorSample],
) -> ProjectionBranchDiagnostics {
    let mut max_displacement = 0.0;
    let mut limit_violations = 0;
    let mut thickness_violations = 0;
    for (idx, vertex) in mesh.vertices.iter().enumerate() {
        let displacement = vertex
            .projection_status
            .displacement_magnitude
            .unwrap_or(0.0);
        max_displacement = f64::max(max_displacement, displacement);
        let target = vertex
            .local_target_size
            .unwrap_or(mesh.metadata.target_spacing);
        if displacement > config.max_projection_displacement_factor * target {
            limit_violations += 1;
        }
        let near_trailing_cap = samples
            .get(idx)
            .and_then(|s| s.feature_id.as_deref())
            .is_some_and(|id| id.contains("trailing_edge"));
        if near_trailing_cap && displacement > 0.5 * config.min_local_thickness.max(1.0e-9) {
            thickness_violations += 1;
        }
    }
    ProjectionBranchDiagnostics {
        vertices_checked: mesh.vertices.len(),
        max_projection_displacement: max_displacement,
        displacement_limit_violations: limit_violations,
        displacement_over_local_thickness: thickness_violations,
        notes: vec![
            "historical pre-edit positions are unavailable for v3-v6; v7 records final projection displacement and sector assignment for future operation lineage".to_string(),
        ],
    }
}

fn feature_parameter_diagnostics(
    sdf: &dyn Sdf,
    mesh: &SdfSurfaceMesh,
    graph: &SurfaceFeatureGraph,
    config: &SurfaceArtifactConfig,
) -> FeatureParameterDiagnostics {
    let options = ProceduralFeatureQueryOptions {
        tolerance: config.sector_query_tolerance,
        ..ProceduralFeatureQueryOptions::diagnostic()
    };
    let mut results = Vec::new();
    for curve in &graph.curves {
        if curve.ordered_vertices.len() < 2 {
            continue;
        }
        let mut params = Vec::new();
        for id in &curve.ordered_vertices {
            let p = Vec3::from_array(mesh.vertices[*id as usize].position.map(|v| v as f32));
            let q = sdf.query_procedural_feature(p, &options);
            params.push(
                q.span_parameter
                    .or(q.airfoil_parameter)
                    .or(q.chord_parameter),
            );
        }
        let values = params.into_iter().flatten().collect::<Vec<_>>();
        if values.len() < 2 {
            continue;
        }
        let monotonic_up = values.windows(2).all(|w| w[0] <= w[1] + 1.0e-8);
        let monotonic_down = values.windows(2).all(|w| w[0] + 1.0e-8 >= w[1]);
        let max_jump = values
            .windows(2)
            .map(|w| (w[1] - w[0]).abs())
            .fold(0.0, f64::max);
        results.push(FeatureCurveParameterResult {
            curve_id: curve.id,
            name: curve.name.clone(),
            feature_type: format!("{:?}", curve.feature_type),
            vertex_count: values.len(),
            monotonic: monotonic_up || monotonic_down,
            max_parameter_jump: max_jump,
        });
    }
    let nonmonotonic_curves = results.iter().filter(|r| !r.monotonic).count();
    let large_jump_curves = results
        .iter()
        .filter(|r| r.max_parameter_jump > 0.2)
        .count();
    FeatureParameterDiagnostics {
        curves_checked: results.len(),
        nonmonotonic_curves,
        large_jump_curves,
        endpoint_mismatch_curves: graph
            .curves
            .iter()
            .filter(|curve| {
                matches!(
                    curve.feature_type,
                    SurfaceFeatureType::TrailingEdge | SurfaceFeatureType::TipCapSeam
                ) && curve.diagnostics.endpoint_coordinates.len() < 2
            })
            .count(),
        curve_results: results,
    }
}

fn build_artifact_regions(
    sdf: &dyn Sdf,
    mesh: &SdfSurfaceMesh,
    config: &SurfaceArtifactConfig,
    signals: &[TriangleSignals],
    vertex_samples: &[VertexSectorSample],
) -> Vec<ArtifactRegion> {
    let mut suspect = BTreeSet::new();
    for (idx, (tri, signal)) in mesh.triangles.iter().zip(signals).enumerate() {
        if signal.severe_normal
            || signal.sector_inconsistent
            || signal.self_intersection
            || signal.fold_indicator
            || tri.quality.aspect_ratio > config.suspect_aspect_ratio
        {
            suspect.insert(idx);
        }
    }
    if suspect.is_empty() {
        return Vec::new();
    }
    let neighbors = triangle_neighbors(mesh);
    let mut visited = BTreeSet::new();
    let mut regions = Vec::new();
    for start in suspect.iter().copied().collect::<Vec<_>>() {
        if visited.contains(&start) {
            continue;
        }
        let mut queue = VecDeque::from([start]);
        let mut tris = Vec::new();
        visited.insert(start);
        while let Some(idx) = queue.pop_front() {
            tris.push(idx);
            for next in &neighbors[idx] {
                if suspect.contains(next) && visited.insert(*next) {
                    queue.push_back(*next);
                }
            }
        }
        regions.push(region_summary(
            sdf,
            mesh,
            &tris,
            signals,
            vertex_samples,
            regions.len(),
        ));
    }
    regions
}

fn region_summary(
    sdf: &dyn Sdf,
    mesh: &SdfSurfaceMesh,
    tris: &[usize],
    signals: &[TriangleSignals],
    samples: &[VertexSectorSample],
    region_index: usize,
) -> ArtifactRegion {
    let mut vertices = BTreeSet::new();
    let mut area = 0.0;
    let mut bbox_min = [f64::INFINITY; 3];
    let mut bbox_max = [f64::NEG_INFINITY; 3];
    let mut centroid = Vec3::ZERO;
    let mut sector_counts = BTreeMap::new();
    let mut spans = Vec::new();
    let mut chords = Vec::new();
    let mut max_normal_angle = 0.0;
    let mut folds = 0;
    let mut intersections = 0;
    for idx in tris {
        let tri = &mesh.triangles[*idx];
        area += tri.area;
        centroid += triangle_centroid(mesh, tri.vertices) * tri.area as f32;
        max_normal_angle = f64::max(max_normal_angle, signals[*idx].normal_angle);
        if signals[*idx].fold_indicator {
            folds += 1;
        }
        if signals[*idx].self_intersection {
            intersections += 1;
        }
        for id in tri.vertices {
            vertices.insert(id);
            let p = mesh.vertices[id as usize].position;
            for axis in 0..3 {
                bbox_min[axis] = bbox_min[axis].min(p[axis]);
                bbox_max[axis] = bbox_max[axis].max(p[axis]);
            }
            if let Some(sample) = samples.get(id as usize) {
                *sector_counts
                    .entry(
                        sample
                            .sector_id
                            .clone()
                            .unwrap_or_else(|| "unknown".to_string()),
                    )
                    .or_default() += 1;
                if let Some(s) = sample.span {
                    spans.push(s);
                }
                if let Some(c) = sample.chord {
                    chords.push(c);
                }
            }
        }
    }
    if area > 0.0 {
        centroid /= area as f32;
    }
    let query = sdf.query_procedural_feature(
        centroid,
        &ProceduralFeatureQueryOptions {
            tolerance: 12.0,
            ..ProceduralFeatureQueryOptions::diagnostic()
        },
    );
    ArtifactRegion {
        stable_region_id: format!("artifact_region_{region_index:03}"),
        triangle_count: tris.len(),
        vertex_count: vertices.len(),
        area,
        bbox_min,
        bbox_max,
        centroid: [centroid.x as f64, centroid.y as f64, centroid.z as f64],
        nearest_procedural_feature: query.feature_id,
        surface_sector_composition: sector_counts,
        span_parameter_range: min_max(&spans),
        chord_parameter_range: min_max(&chords),
        max_normal_angle_degrees: max_normal_angle,
        fold_indicators: folds,
        self_intersection_pairs: intersections,
        likely_cause: if intersections > 0 {
            "self_intersection".to_string()
        } else if folds > 0 {
            "local_fold_indicator".to_string()
        } else {
            "quality_or_feature_adjacent_visual_artifact".to_string()
        },
    }
}

fn acceptance(
    mesh: &SdfSurfaceMesh,
    normal: &NormalAlignmentDiagnostics,
    folds: &FoldDetectionDiagnostics,
    intersections: &SelfIntersectionDiagnostics,
    sectors: &SectorConsistencyDiagnostics,
    projection: &ProjectionBranchDiagnostics,
    features: &FeatureParameterDiagnostics,
) -> GeometricAcceptance {
    let mut failures = Vec::new();
    let topology_valid = mesh
        .edges
        .iter()
        .all(|edge| edge.incident_triangles.len() == 2)
        && mesh
            .triangles
            .iter()
            .all(|tri| tri.area.is_finite() && tri.area > 0.0);
    if !topology_valid {
        failures.push("topology_or_degenerate_triangle_failure".to_string());
    }
    if intersections.intersecting_pairs != 0 {
        failures.push("self_intersections_detected".to_string());
    }
    if folds.unexplained_fold_edges != 0 {
        failures.push("unexplained_fold_edges_detected".to_string());
    }
    if normal.severe_disagreements != 0 || normal.negative_dot_triangles != 0 {
        failures.push("severe_triangle_sdf_normal_disagreement".to_string());
    }
    if sectors.upper_lower_without_cap_triangles != 0 {
        failures.push("upper_lower_sector_crossing_without_cap".to_string());
    }
    if sectors.feature_distance_violations != 0 {
        failures.push("feature_match_distance_violations".to_string());
    }
    if projection.displacement_limit_violations != 0
        || projection.displacement_over_local_thickness != 0
    {
        failures.push("projection_branch_guard_violation".to_string());
    }
    if features.nonmonotonic_curves != 0 {
        failures.push("nonmonotonic_feature_parameters".to_string());
    }
    GeometricAcceptance {
        topology_valid,
        self_intersections_zero: intersections.intersecting_pairs == 0,
        folds_zero: folds.unexplained_fold_edges == 0,
        normal_alignment_accepted: normal.severe_disagreements == 0
            && normal.negative_dot_triangles == 0,
        sector_consistency_accepted: sectors.upper_lower_without_cap_triangles == 0
            && sectors.feature_distance_violations == 0,
        projection_branch_accepted: projection.displacement_limit_violations == 0
            && projection.displacement_over_local_thickness == 0,
        feature_parameters_accepted: features.nonmonotonic_curves == 0,
        accepted_for_macro_shell_prereview: failures.is_empty(),
        failure_reasons: failures,
    }
}

fn triangle_neighbors(mesh: &SdfSurfaceMesh) -> Vec<BTreeSet<usize>> {
    let tri_by_id = mesh
        .triangles
        .iter()
        .enumerate()
        .map(|(idx, tri)| (tri.id, idx))
        .collect::<BTreeMap<_, _>>();
    let mut out = vec![BTreeSet::new(); mesh.triangles.len()];
    for edge in &mesh.edges {
        for a in &edge.incident_triangles {
            for b in &edge.incident_triangles {
                if a != b {
                    out[tri_by_id[a]].insert(tri_by_id[b]);
                }
            }
        }
    }
    out
}

fn triangle_centroid(mesh: &SdfSurfaceMesh, ids: [u64; 3]) -> Vec3 {
    (position(mesh, ids[0]) + position(mesh, ids[1]) + position(mesh, ids[2])) / 3.0
}

fn position(mesh: &SdfSurfaceMesh, id: u64) -> Vec3 {
    Vec3::from_array(mesh.vertices[id as usize].position.map(|v| v as f32))
}

fn tri_aabb(mesh: &SdfSurfaceMesh, ids: [u64; 3]) -> ([f32; 3], [f32; 3]) {
    let mut min = Vec3::splat(f32::INFINITY);
    let mut max = Vec3::splat(f32::NEG_INFINITY);
    for id in ids {
        let p = position(mesh, id);
        min = min.min(p);
        max = max.max(p);
    }
    ([min.x, min.y, min.z], [max.x, max.y, max.z])
}

fn aabb_overlap(a: ([f32; 3], [f32; 3]), b: ([f32; 3], [f32; 3])) -> bool {
    (0..3).all(|axis| a.0[axis] <= b.1[axis] && b.0[axis] <= a.1[axis])
}

fn triangles_share_vertex(a: [u64; 3], b: [u64; 3]) -> bool {
    a.iter().any(|id| b.contains(id))
}

fn tri_tri_intersect(mesh: &SdfSurfaceMesh, a: [u64; 3], b: [u64; 3]) -> bool {
    let av = [
        position(mesh, a[0]),
        position(mesh, a[1]),
        position(mesh, a[2]),
    ];
    let bv = [
        position(mesh, b[0]),
        position(mesh, b[1]),
        position(mesh, b[2]),
    ];
    for i in 0..3 {
        if segment_intersects_triangle(av[i], av[(i + 1) % 3], bv) {
            return true;
        }
        if segment_intersects_triangle(bv[i], bv[(i + 1) % 3], av) {
            return true;
        }
    }
    false
}

fn segment_intersects_triangle(a: Vec3, b: Vec3, tri: [Vec3; 3]) -> bool {
    let dir = b - a;
    let edge1 = tri[1] - tri[0];
    let edge2 = tri[2] - tri[0];
    let h = dir.cross(edge2);
    let det = edge1.dot(h);
    if det.abs() < 1.0e-7 {
        return false;
    }
    let inv_det = 1.0 / det;
    let s = a - tri[0];
    let u = inv_det * s.dot(h);
    if !(1.0e-7..=1.0 - 1.0e-7).contains(&u) {
        return false;
    }
    let q = s.cross(edge1);
    let v = inv_det * dir.dot(q);
    if v < 1.0e-7 || u + v > 1.0 - 1.0e-7 {
        return false;
    }
    let t = inv_det * edge2.dot(q);
    t > 1.0e-7 && t < 1.0 - 1.0e-7
}

fn min_max(values: &[f64]) -> Option<[f64; 2]> {
    if values.is_empty() {
        return None;
    }
    Some([
        values.iter().copied().fold(f64::INFINITY, f64::min),
        values.iter().copied().fold(f64::NEG_INFINITY, f64::max),
    ])
}

fn percentile(values: &[f64], p: f64) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let idx = ((values.len() - 1) as f64 * p).round() as usize;
    values[idx.min(values.len() - 1)]
}

fn elapsed_ms(start: Instant) -> f64 {
    start.elapsed().as_secs_f64() * 1000.0
}
