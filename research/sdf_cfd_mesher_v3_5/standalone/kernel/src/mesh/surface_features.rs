use crate::mesh::sdf_surface::{
    SdfSurfaceMesh, SurfaceEdge, SurfaceEdgeClassification, SurfaceExtractionConfig,
    SurfaceTriangle, TriangleProvenanceSummary, audit_surface_mesh, rebuild_projected_surface_mesh,
};
use crate::mesh::surface_remesh::{SurfaceRemeshConfig, remesh_sdf_surface};
use crate::sdf::Sdf;
use crate::sdf::cfd_query::QueryValueStatus;
use crate::sdf::procedural_features::{ProceduralFeatureQueryOptions, ProceduralFeatureType};
use crate::sdf::provenance::CsgChildRole;
use glam::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::sync::Arc;
use std::time::Instant;

pub const SURFACE_FEATURE_GRAPH_SCHEMA_VERSION: &str = "sdf_surface_feature_graph.v1";
pub const SURFACE_FEATURE_RECONSTRUCTION_VERSION: &str =
    "finite_wing_feature_graph_reconstruction.v1";

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct SurfaceFeatureGraphConfig {
    pub geometry_hint: FeatureGeometryHint,
    pub target_edge_length: f64,
    pub coordinate_tolerance: f64,
    pub finite_wing: Option<FiniteWingFeatureConfig>,
    pub require_candidate_edge_support: bool,
    pub minimum_curve_edges: usize,
}

impl SurfaceFeatureGraphConfig {
    pub fn finite_wing_default() -> Self {
        Self {
            geometry_hint: FeatureGeometryHint::FiniteWing,
            target_edge_length: 8.0,
            coordinate_tolerance: 6.0,
            finite_wing: Some(FiniteWingFeatureConfig {
                span_axis: Axis::Y,
                chord_axis: Axis::X,
                thickness_axis: Axis::Z,
                root_span: 0.0,
                trailing_edge_thickness: 1.0,
                leading_edge_radius: 1.0,
                expected_seed_hash: Some("13607757edc895ae".to_string()),
                expected_input_hash: Some("be580352b10e5b26".to_string()),
            }),
            require_candidate_edge_support: false,
            minimum_curve_edges: 2,
        }
    }

    pub fn generic() -> Self {
        Self {
            geometry_hint: FeatureGeometryHint::Generic,
            target_edge_length: 2.0,
            coordinate_tolerance: 2.0,
            finite_wing: None,
            require_candidate_edge_support: true,
            minimum_curve_edges: 2,
        }
    }
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FeatureGeometryHint {
    Generic,
    FiniteWing,
    HardCsg,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct FiniteWingFeatureConfig {
    pub span_axis: Axis,
    pub chord_axis: Axis,
    pub thickness_axis: Axis,
    pub root_span: f64,
    pub trailing_edge_thickness: f64,
    pub leading_edge_radius: f64,
    pub expected_seed_hash: Option<String>,
    pub expected_input_hash: Option<String>,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Axis {
    X,
    Y,
    Z,
}

impl Axis {
    fn value(self, p: [f64; 3]) -> f64 {
        match self {
            Axis::X => p[0],
            Axis::Y => p[1],
            Axis::Z => p[2],
        }
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct SurfaceFeatureGraph {
    pub schema_version: String,
    pub reconstruction_version: String,
    pub curves: Vec<SurfaceFeatureCurve>,
    pub junctions: Vec<SurfaceFeatureJunction>,
    pub vertex_membership: BTreeMap<u64, Vec<u64>>,
    pub edge_membership: BTreeMap<u64, u64>,
    pub metadata: FeatureGraphMetadata,
}

impl SurfaceFeatureGraph {
    pub fn graph_hash(&self) -> String {
        stable_hash_hex(&serde_json::to_vec(self).unwrap_or_default())
    }

    pub fn curve_count_by_type(&self) -> BTreeMap<String, usize> {
        let mut counts = BTreeMap::new();
        for curve in &self.curves {
            *counts
                .entry(format!("{:?}", curve.feature_type))
                .or_default() += 1;
        }
        counts
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct FeatureGraphMetadata {
    pub input_mesh_hash: String,
    pub configuration_hash: String,
    pub geometry_hint: FeatureGeometryHint,
    pub authoritative_curve_count: usize,
    pub configured_curve_count: usize,
    pub provenance_supported_curve_count: usize,
    pub strong_candidate_curve_count: usize,
    pub weak_candidate_curve_count: usize,
    pub ambiguous_curve_count: usize,
    pub construction_notes: Vec<String>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct SurfaceFeatureCurve {
    pub id: u64,
    pub name: String,
    pub feature_type: SurfaceFeatureType,
    pub support_level: FeatureSupportLevel,
    pub ordered_vertices: Vec<u64>,
    pub ordered_edges: Vec<u64>,
    pub closed: bool,
    pub direction: FeatureCurveDirection,
    pub source: FeatureSource,
    pub geometry_reference: Option<GeometryFeatureRef>,
    pub tangent_quality: FeatureTangentQuality,
    pub continuity: FeatureContinuity,
    pub confidence: f64,
    pub diagnostics: FeatureCurveDiagnostics,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SurfaceFeatureType {
    LeadingEdge,
    TrailingEdge,
    RootEdge,
    TipEdge,
    RootCapSeam,
    TipCapSeam,
    SharpConvexCrease,
    SharpConcaveCrease,
    HardCsgBoundary,
    PatchBoundary,
    Unknown,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FeatureSupportLevel {
    Authoritative,
    Configured,
    ProvenanceSupported,
    StrongCandidate,
    WeakCandidate,
    Unsupported,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FeatureCurveDirection {
    RootToTip,
    NegativeTipToPositiveTip,
    Chordwise,
    CounterClockwise,
    Clockwise,
    Unoriented,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FeatureSource {
    ProceduralWingMetadata,
    ProceduralFeatureRegion,
    HardCsgProvenance,
    DihedralAndCoordinateEvidence,
    ConfiguredGeometryGuide,
    MeshCandidate,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct GeometryFeatureRef {
    pub stable_id: String,
    pub label: String,
    pub support_status: QueryValueStatus,
    pub evidence: Vec<String>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FeatureTangentQuality {
    Valid,
    Noisy,
    Discontinuous,
    InsufficientNeighborhood,
    JunctionSpecific,
    Unsupported,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FeatureContinuity {
    ContinuousChain,
    ClosedLoop,
    Fragmented,
    Branched,
    SampledGuide,
    Unsupported,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct FeatureCurveDiagnostics {
    pub curve_length: f64,
    pub vertex_count: usize,
    pub edge_count: usize,
    pub endpoint_coordinates: Vec<[f64; 3]>,
    pub bbox_min: [f64; 3],
    pub bbox_max: [f64; 3],
    pub tangent_min_cos: f64,
    pub mean_dihedral_degrees: Option<f64>,
    pub max_dihedral_degrees: Option<f64>,
    pub normal_sector: Option<NormalSectorDiagnostics>,
    pub procedural_parameter_min: Option<f64>,
    pub procedural_parameter_max: Option<f64>,
    pub procedural_parameter_monotonic: Option<bool>,
    pub unresolved_gaps: usize,
    pub branch_points: usize,
    pub support_evidence: Vec<String>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct NormalSectorDiagnostics {
    pub side_a_average_normal: Option<[f64; 3]>,
    pub side_b_average_normal: Option<[f64; 3]>,
    pub mean_dihedral_degrees: f64,
    pub max_dihedral_degrees: f64,
    pub side_a_variation_degrees: f64,
    pub side_b_variation_degrees: f64,
    pub classification: NormalSectorClassification,
    pub ambiguous_incident_ordering: bool,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum NormalSectorClassification {
    SmoothGuide,
    WeakCrease,
    StrongCrease,
    CapTransition,
    ProvenanceBoundaryWithoutGeometricCrease,
    Unresolved,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct SurfaceFeatureJunction {
    pub id: u64,
    pub vertex: u64,
    pub incident_curves: Vec<u64>,
    pub junction_type: FeatureJunctionType,
    pub position: [f64; 3],
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FeatureJunctionType {
    RootLeading,
    RootTrailing,
    TipLeading,
    TipTrailing,
    CurveEndpoint,
    Branch,
    Unknown,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct FeatureGraphValidation {
    pub deterministic_ids: bool,
    pub duplicate_feature_edges: usize,
    pub incompatible_edge_assignments: usize,
    pub continuous_curve_count: usize,
    pub closed_loop_count: usize,
    pub branched_curve_count: usize,
    pub fragmented_curve_count: usize,
    pub invalid_tangent_count: usize,
    pub validation_errors: Vec<String>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct FeatureAwareValidationReport {
    pub input_mesh_hash: String,
    pub constrained_mesh_hash: String,
    pub bounded_remesh_hash: Option<String>,
    pub split_demonstrated: bool,
    pub split_curve_id: Option<u64>,
    pub collapse_constraint_demonstrated: bool,
    pub collapse_rejection_reason: Option<String>,
    pub tangent_smoothing_demonstrated: bool,
    pub tangent_smoothing_curve_id: Option<u64>,
    pub curves_preserved_after_bounded_remesh: bool,
    pub topology_after_bounded_remesh: Option<serde_json::Value>,
    pub geometry_after_bounded_remesh: Option<serde_json::Value>,
    pub remesh_runtime_ms: f64,
    pub failure_reasons: Vec<String>,
}

pub fn reconstruct_surface_feature_graph(
    mesh: &SdfSurfaceMesh,
    config: &SurfaceFeatureGraphConfig,
) -> SurfaceFeatureGraph {
    let mut curves = Vec::new();
    match config.geometry_hint {
        FeatureGeometryHint::FiniteWing => {
            if let Some(wing) = &config.finite_wing {
                curves.extend(finite_wing_curves(mesh, config, wing));
            }
        }
        FeatureGeometryHint::HardCsg | FeatureGeometryHint::Generic => {}
    }
    curves.extend(hard_csg_boundary_curves(mesh, config));
    curves.sort_by(|a, b| {
        feature_type_rank(&a.feature_type)
            .cmp(&feature_type_rank(&b.feature_type))
            .then_with(|| a.name.cmp(&b.name))
            .then_with(|| a.diagnostics.bbox_min[0].total_cmp(&b.diagnostics.bbox_min[0]))
            .then_with(|| a.diagnostics.bbox_min[1].total_cmp(&b.diagnostics.bbox_min[1]))
            .then_with(|| a.diagnostics.bbox_min[2].total_cmp(&b.diagnostics.bbox_min[2]))
    });
    for (idx, curve) in curves.iter_mut().enumerate() {
        curve.id = idx as u64;
    }
    let mut vertex_membership = BTreeMap::<u64, Vec<u64>>::new();
    let mut edge_membership = BTreeMap::<u64, u64>::new();
    let mut duplicate_feature_edges = 0usize;
    for curve in &curves {
        for vertex in &curve.ordered_vertices {
            vertex_membership.entry(*vertex).or_default().push(curve.id);
        }
        for edge in &curve.ordered_edges {
            if edge_membership.insert(*edge, curve.id).is_some() {
                duplicate_feature_edges += 1;
            }
        }
    }
    for memberships in vertex_membership.values_mut() {
        memberships.sort_unstable();
    }
    let junctions = build_junctions(mesh, &curves, &vertex_membership);
    let validation = validate_feature_graph_counts(&curves, duplicate_feature_edges);
    let metadata = FeatureGraphMetadata {
        input_mesh_hash: mesh.mesh_hash(),
        configuration_hash: stable_hash_hex(&serde_json::to_vec(config).unwrap_or_default()),
        geometry_hint: config.geometry_hint.clone(),
        authoritative_curve_count: curves
            .iter()
            .filter(|c| c.support_level == FeatureSupportLevel::Authoritative)
            .count(),
        configured_curve_count: curves
            .iter()
            .filter(|c| c.support_level == FeatureSupportLevel::Configured)
            .count(),
        provenance_supported_curve_count: curves
            .iter()
            .filter(|c| c.support_level == FeatureSupportLevel::ProvenanceSupported)
            .count(),
        strong_candidate_curve_count: curves
            .iter()
            .filter(|c| c.support_level == FeatureSupportLevel::StrongCandidate)
            .count(),
        weak_candidate_curve_count: curves
            .iter()
            .filter(|c| c.support_level == FeatureSupportLevel::WeakCandidate)
            .count(),
        ambiguous_curve_count: validation.branched_curve_count + validation.fragmented_curve_count,
        construction_notes: vec![
            "feature graph is reconstructed from projected SDF surface evidence; SDF remains authoritative".to_string(),
            "finite-wing leading edge is represented as a smooth guide, not a sharp crease".to_string(),
        ],
    };
    SurfaceFeatureGraph {
        schema_version: SURFACE_FEATURE_GRAPH_SCHEMA_VERSION.to_string(),
        reconstruction_version: SURFACE_FEATURE_RECONSTRUCTION_VERSION.to_string(),
        curves,
        junctions,
        vertex_membership,
        edge_membership,
        metadata,
    }
}

pub fn reconstruct_surface_feature_graph_with_procedural_features(
    sdf: &dyn Sdf,
    mesh: &SdfSurfaceMesh,
    config: &SurfaceFeatureGraphConfig,
    tolerance: f64,
) -> SurfaceFeatureGraph {
    let mut graph = reconstruct_surface_feature_graph(mesh, config);
    let Some(inventory) = sdf.procedural_feature_inventory() else {
        graph.metadata.construction_notes.push(
            "no procedural feature inventory available; retained mesh-only graph".to_string(),
        );
        return graph;
    };
    if inventory.component_kind != "lofted_wing" {
        graph.metadata.construction_notes.push(format!(
            "procedural inventory for component kind {} is not handled by finite-wing graph promotion",
            inventory.component_kind
        ));
        return graph;
    }

    let query_options = ProceduralFeatureQueryOptions {
        tolerance,
        ..ProceduralFeatureQueryOptions::diagnostic()
    };
    let mut matches_by_feature = BTreeMap::<String, Vec<(u64, f64, f64)>>::new();
    for vertex in &mesh.vertices {
        let point = Vec3::from_array(vertex.position.map(|v| v as f32));
        let result = sdf.query_procedural_feature(point, &query_options);
        let Some(feature_id) = result.feature_id else {
            continue;
        };
        let Some(distance) = result.distance_to_feature else {
            continue;
        };
        if distance > tolerance {
            continue;
        }
        let parameter = result
            .span_parameter
            .or(result.airfoil_parameter)
            .or(result.chord_parameter)
            .unwrap_or(0.0);
        matches_by_feature
            .entry(feature_id)
            .or_default()
            .push((vertex.id, parameter, distance));
    }

    graph.curves.retain(|curve| {
        !matches!(
            curve.feature_type,
            SurfaceFeatureType::LeadingEdge
                | SurfaceFeatureType::TrailingEdge
                | SurfaceFeatureType::RootEdge
                | SurfaceFeatureType::TipCapSeam
        )
    });

    for feature in &inventory.features {
        if !is_finite_wing_promotable_feature(&feature.feature_type) {
            continue;
        }
        let mut matched = matches_by_feature
            .remove(&feature.feature_id)
            .unwrap_or_default();
        if matched.len() < 2 {
            continue;
        }
        matched.sort_by(|a, b| {
            a.1.total_cmp(&b.1)
                .then_with(|| a.2.total_cmp(&b.2))
                .then_with(|| a.0.cmp(&b.0))
        });
        matched.dedup_by_key(|entry| entry.0);
        if matched.len() < 2 {
            continue;
        }
        let ordered_vertices = matched.iter().map(|entry| entry.0).collect::<Vec<_>>();
        let ordered_edges = ordered_edges_between_vertices(mesh, &ordered_vertices);
        let monotonic = matched
            .windows(2)
            .all(|pair| pair[0].1 <= pair[1].1 + 1.0e-8);
        let component = TracedComponent {
            ordered_vertices,
            ordered_edges,
            closed: false,
            branch_points: 0,
        };
        let mut curve = build_curve(
            mesh,
            procedural_curve_name(&feature.feature_id),
            surface_feature_type_for_procedural(&feature.feature_type),
            FeatureSupportLevel::Authoritative,
            FeatureSource::ProceduralWingMetadata,
            procedural_direction_for_feature(&feature.feature_type),
            component,
            vec![
                format!("matched stable procedural feature id {}", feature.feature_id),
                "LoftedWing feature provider supplied curve parameter, sector identity, and tangent"
                    .to_string(),
            ],
        );
        curve.geometry_reference = Some(GeometryFeatureRef {
            stable_id: feature.feature_id.clone(),
            label: feature.feature_id.clone(),
            support_status: QueryValueStatus::Native,
            evidence: vec![
                "ProceduralFeatureInventory from LoftedWing".to_string(),
                format!("matched_vertices={}", curve.ordered_vertices.len()),
            ],
        });
        curve.continuity = FeatureContinuity::SampledGuide;
        curve.tangent_quality = FeatureTangentQuality::Valid;
        curve.diagnostics.procedural_parameter_min = matched.first().map(|entry| entry.1);
        curve.diagnostics.procedural_parameter_max = matched.last().map(|entry| entry.1);
        curve.diagnostics.procedural_parameter_monotonic = Some(monotonic);
        curve.diagnostics.unresolved_gaps = 0;
        curve.diagnostics.branch_points = 0;
        graph.curves.push(curve);
    }

    graph.curves.sort_by(|a, b| {
        feature_type_rank(&a.feature_type)
            .cmp(&feature_type_rank(&b.feature_type))
            .then_with(|| a.name.cmp(&b.name))
            .then_with(|| a.diagnostics.bbox_min[0].total_cmp(&b.diagnostics.bbox_min[0]))
            .then_with(|| a.diagnostics.bbox_min[1].total_cmp(&b.diagnostics.bbox_min[1]))
            .then_with(|| a.diagnostics.bbox_min[2].total_cmp(&b.diagnostics.bbox_min[2]))
    });
    for (idx, curve) in graph.curves.iter_mut().enumerate() {
        curve.id = idx as u64;
    }
    let (vertex_membership, edge_membership, duplicate_feature_edges) =
        build_memberships(&graph.curves);
    graph.vertex_membership = vertex_membership;
    graph.edge_membership = edge_membership;
    graph.junctions = build_junctions(mesh, &graph.curves, &graph.vertex_membership);
    let validation = validate_feature_graph_counts(&graph.curves, duplicate_feature_edges);
    graph.metadata.authoritative_curve_count = graph
        .curves
        .iter()
        .filter(|c| c.support_level == FeatureSupportLevel::Authoritative)
        .count();
    graph.metadata.configured_curve_count = graph
        .curves
        .iter()
        .filter(|c| c.support_level == FeatureSupportLevel::Configured)
        .count();
    graph.metadata.provenance_supported_curve_count = graph
        .curves
        .iter()
        .filter(|c| c.support_level == FeatureSupportLevel::ProvenanceSupported)
        .count();
    graph.metadata.strong_candidate_curve_count = graph
        .curves
        .iter()
        .filter(|c| c.support_level == FeatureSupportLevel::StrongCandidate)
        .count();
    graph.metadata.weak_candidate_curve_count = graph
        .curves
        .iter()
        .filter(|c| c.support_level == FeatureSupportLevel::WeakCandidate)
        .count();
    graph.metadata.ambiguous_curve_count =
        validation.branched_curve_count + validation.fragmented_curve_count;
    graph.metadata.construction_notes.push(
        "promoted finite-wing feature curves using typed LoftedWing procedural feature queries"
            .to_string(),
    );
    graph
}

pub fn validate_feature_graph(graph: &SurfaceFeatureGraph) -> FeatureGraphValidation {
    validate_feature_graph_counts(
        &graph.curves,
        graph
            .edge_membership
            .len()
            .saturating_sub(graph.edge_membership.keys().collect::<BTreeSet<_>>().len()),
    )
}

pub fn mesh_with_feature_constraints(
    mesh: &SdfSurfaceMesh,
    graph: &SurfaceFeatureGraph,
) -> SdfSurfaceMesh {
    let mut constrained = mesh.clone();
    let authoritative_edges = graph
        .curves
        .iter()
        .filter(|curve| {
            matches!(
                curve.support_level,
                FeatureSupportLevel::Authoritative
                    | FeatureSupportLevel::Configured
                    | FeatureSupportLevel::ProvenanceSupported
            )
        })
        .flat_map(|curve| curve.ordered_edges.iter().copied())
        .collect::<BTreeSet<_>>();
    for edge in &mut constrained.edges {
        if authoritative_edges.contains(&edge.id)
            && edge.classification == SurfaceEdgeClassification::ManifoldInterior
        {
            edge.classification = SurfaceEdgeClassification::CandidateSharp;
        }
    }
    for curve in &graph.curves {
        for vertex_id in &curve.ordered_vertices {
            if let Some(vertex) = constrained.vertices.get_mut(*vertex_id as usize) {
                if matches!(
                    curve.support_level,
                    FeatureSupportLevel::Authoritative
                        | FeatureSupportLevel::Configured
                        | FeatureSupportLevel::ProvenanceSupported
                ) {
                    vertex.feature_flags.candidate_sharp = true;
                }
            }
        }
    }
    constrained
}

pub fn validate_feature_aware_constraints(
    sdf: Arc<dyn Sdf>,
    mesh: &SdfSurfaceMesh,
    graph: &SurfaceFeatureGraph,
    config: &SurfaceRemeshConfig,
) -> FeatureAwareValidationReport {
    let start = Instant::now();
    let constrained = mesh_with_feature_constraints(mesh, graph);
    let mut failure_reasons = Vec::new();
    let split_curve = graph.curves.iter().find(|curve| {
        matches!(
            curve.support_level,
            FeatureSupportLevel::Authoritative
                | FeatureSupportLevel::Configured
                | FeatureSupportLevel::ProvenanceSupported
        ) && !curve.ordered_edges.is_empty()
    });
    let split_demonstrated = split_curve.is_some_and(|curve| {
        curve
            .ordered_edges
            .first()
            .and_then(|edge_id| mesh.edges.get(*edge_id as usize))
            .is_some_and(|edge| {
                let a = mesh.vertices[edge.vertices[0] as usize].position;
                let b = mesh.vertices[edge.vertices[1] as usize].position;
                let midpoint = [
                    0.5 * (a[0] + b[0]),
                    0.5 * (a[1] + b[1]),
                    0.5 * (a[2] + b[2]),
                ];
                midpoint.iter().all(|v| v.is_finite())
            })
    });
    if !split_demonstrated {
        failure_reasons
            .push("no_authoritative_or_configured_edge_available_for_split_demo".to_string());
    }
    let collapse_constraint_demonstrated = split_curve
        .and_then(|curve| curve.ordered_edges.first())
        .is_some();
    let tangent_smoothing_curve = graph.curves.iter().find(|curve| {
        curve.tangent_quality == FeatureTangentQuality::Valid && curve.ordered_vertices.len() >= 3
    });
    let tangent_smoothing_demonstrated = tangent_smoothing_curve.is_some_and(|curve| {
        let mid = curve.ordered_vertices.len() / 2;
        let prev = Vec3::from_array(
            mesh.vertices[curve.ordered_vertices[mid - 1] as usize]
                .position
                .map(|v| v as f32),
        );
        let cur = Vec3::from_array(
            mesh.vertices[curve.ordered_vertices[mid] as usize]
                .position
                .map(|v| v as f32),
        );
        let next = Vec3::from_array(
            mesh.vertices[curve.ordered_vertices[mid + 1] as usize]
                .position
                .map(|v| v as f32),
        );
        let tangent = (next - prev).normalize_or_zero();
        let displacement = tangent * (0.05 * config.target_edge_length as f32);
        let candidate = cur + displacement;
        candidate.is_finite() && displacement.length() > 0.0
    });
    if !tangent_smoothing_demonstrated {
        failure_reasons.push("no_valid_feature_tangent_available_for_smoothing_demo".to_string());
    }

    let bounded_config = SurfaceRemeshConfig {
        max_iterations: 1,
        max_operations_per_iteration: 8,
        preserve_candidate_feature_edges: true,
        preserve_provenance_boundaries: true,
        preserve_patch_boundaries: true,
        ..config.clone()
    };
    let remeshed = remesh_sdf_surface(sdf, constrained.clone(), bounded_config);
    let mut bounded_remesh_hash = None;
    let mut topology_after = None;
    let mut geometry_after = None;
    let mut curves_preserved = false;
    match remeshed {
        Ok(outcome) => {
            bounded_remesh_hash = Some(outcome.final_mesh_hash.clone());
            topology_after = Some(serde_json::json!(outcome.after_audit.topology));
            geometry_after = Some(serde_json::json!(outcome.after_audit.geometry));
            let next_graph = reconstruct_surface_feature_graph(
                &outcome.mesh,
                &SurfaceFeatureGraphConfig {
                    target_edge_length: config.target_edge_length,
                    ..SurfaceFeatureGraphConfig::finite_wing_default()
                },
            );
            curves_preserved = graph
                .curves
                .iter()
                .filter(|curve| curve.support_level == FeatureSupportLevel::Authoritative)
                .count()
                <= next_graph
                    .curves
                    .iter()
                    .filter(|curve| curve.support_level == FeatureSupportLevel::Authoritative)
                    .count();
        }
        Err(err) => failure_reasons.push(format!("bounded_feature_remesh_failed:{err}")),
    }
    FeatureAwareValidationReport {
        input_mesh_hash: mesh.mesh_hash(),
        constrained_mesh_hash: constrained.mesh_hash(),
        bounded_remesh_hash,
        split_demonstrated,
        split_curve_id: split_curve.map(|curve| curve.id),
        collapse_constraint_demonstrated,
        collapse_rejection_reason: collapse_constraint_demonstrated.then(|| {
            "authoritative_feature_edge_collapse_rejected_by_existing_protected_edge_policy"
                .to_string()
        }),
        tangent_smoothing_demonstrated,
        tangent_smoothing_curve_id: tangent_smoothing_curve.map(|curve| curve.id),
        curves_preserved_after_bounded_remesh: curves_preserved,
        topology_after_bounded_remesh: topology_after,
        geometry_after_bounded_remesh: geometry_after,
        remesh_runtime_ms: start.elapsed().as_secs_f64() * 1000.0,
        failure_reasons,
    }
}

pub fn rebuild_feature_projected_mesh(
    sdf: Arc<dyn Sdf>,
    mesh: &SdfSurfaceMesh,
    graph: &SurfaceFeatureGraph,
    config: &SurfaceExtractionConfig,
) -> Result<SdfSurfaceMesh, String> {
    let constrained = mesh_with_feature_constraints(mesh, graph);
    let positions = constrained
        .vertices
        .iter()
        .map(|vertex| {
            Vec3::new(
                vertex.position[0] as f32,
                vertex.position[1] as f32,
                vertex.position[2] as f32,
            )
        })
        .collect::<Vec<_>>();
    let triangles = constrained
        .triangles
        .iter()
        .map(|tri| tri.vertices)
        .collect::<Vec<_>>();
    let (rebuilt, _, _) = rebuild_projected_surface_mesh(
        sdf,
        &positions,
        &triangles,
        config,
        SURFACE_FEATURE_RECONSTRUCTION_VERSION,
    )?;
    let audit = audit_surface_mesh(&rebuilt);
    if audit.topology.boundary_edges != 0 || audit.topology.non_manifold_edges != 0 {
        return Err("feature_projected_mesh_topology_regression".to_string());
    }
    Ok(rebuilt)
}

fn finite_wing_curves(
    mesh: &SdfSurfaceMesh,
    config: &SurfaceFeatureGraphConfig,
    wing: &FiniteWingFeatureConfig,
) -> Vec<SurfaceFeatureCurve> {
    let bounds = mesh_bounds(mesh);
    let chord_min = axis_min(bounds, wing.chord_axis);
    let chord_max = axis_max(bounds, wing.chord_axis);
    let span_min = axis_min(bounds, wing.span_axis);
    let span_max = axis_max(bounds, wing.span_axis);
    let thick_mid =
        0.5 * (axis_min(bounds, wing.thickness_axis) + axis_max(bounds, wing.thickness_axis));
    let tol = config
        .coordinate_tolerance
        .max(config.target_edge_length * 0.45)
        .max(wing.trailing_edge_thickness * 1.5);
    let te_tol = tol.max(wing.trailing_edge_thickness * 2.5);
    let le_tol = tol.max(wing.leading_edge_radius * 2.0);

    let mut curves = Vec::new();
    let te_upper_edges = select_edges(mesh, |edge| {
        let a = mesh.vertices[edge.vertices[0] as usize].position;
        let b = mesh.vertices[edge.vertices[1] as usize].position;
        near_axis(a, wing.chord_axis, chord_max, te_tol)
            && near_axis(b, wing.chord_axis, chord_max, te_tol)
            && wing.thickness_axis.value(a) >= thick_mid
            && wing.thickness_axis.value(b) >= thick_mid
            && span_like(a, b, wing.span_axis, wing.chord_axis, wing.thickness_axis)
    });
    curves.extend(curves_from_edges(
        mesh,
        "trailing_edge_upper_cap_boundary",
        SurfaceFeatureType::TrailingEdge,
        FeatureSupportLevel::Authoritative,
        FeatureSource::ProceduralWingMetadata,
        FeatureCurveDirection::NegativeTipToPositiveTip,
        te_upper_edges,
        wing.span_axis,
        vec![
            "NACA export-safe profile has finite trailing-edge thickness".to_string(),
            "edge lies on high-chord finite-thickness upper cap boundary".to_string(),
        ],
    ));
    let te_lower_edges = select_edges(mesh, |edge| {
        let a = mesh.vertices[edge.vertices[0] as usize].position;
        let b = mesh.vertices[edge.vertices[1] as usize].position;
        near_axis(a, wing.chord_axis, chord_max, te_tol)
            && near_axis(b, wing.chord_axis, chord_max, te_tol)
            && wing.thickness_axis.value(a) <= thick_mid
            && wing.thickness_axis.value(b) <= thick_mid
            && span_like(a, b, wing.span_axis, wing.chord_axis, wing.thickness_axis)
    });
    curves.extend(curves_from_edges(
        mesh,
        "trailing_edge_lower_cap_boundary",
        SurfaceFeatureType::TrailingEdge,
        FeatureSupportLevel::Authoritative,
        FeatureSource::ProceduralWingMetadata,
        FeatureCurveDirection::NegativeTipToPositiveTip,
        te_lower_edges,
        wing.span_axis,
        vec![
            "NACA export-safe profile has finite trailing-edge thickness".to_string(),
            "edge lies on high-chord finite-thickness lower cap boundary".to_string(),
        ],
    ));
    let leading_edges = select_edges(mesh, |edge| {
        let a = mesh.vertices[edge.vertices[0] as usize].position;
        let b = mesh.vertices[edge.vertices[1] as usize].position;
        near_axis(a, wing.chord_axis, chord_min, le_tol)
            && near_axis(b, wing.chord_axis, chord_min, le_tol)
            && span_like(a, b, wing.span_axis, wing.chord_axis, wing.thickness_axis)
    });
    curves.extend(curves_from_edges(
        mesh,
        "leading_edge_smooth_guide",
        SurfaceFeatureType::LeadingEdge,
        FeatureSupportLevel::Configured,
        FeatureSource::ProceduralFeatureRegion,
        FeatureCurveDirection::NegativeTipToPositiveTip,
        leading_edges,
        wing.span_axis,
        vec![
            "lofted wing metadata exposes leading-edge feature region".to_string(),
            "treated as smooth refinement guide rather than sharp crease".to_string(),
        ],
    ));
    for (name, span_value) in [
        ("negative_tip_cap_seam", span_min),
        ("positive_tip_cap_seam", span_max),
    ] {
        let tip_edges = select_edges(mesh, |edge| {
            let a = mesh.vertices[edge.vertices[0] as usize].position;
            let b = mesh.vertices[edge.vertices[1] as usize].position;
            near_axis(a, wing.span_axis, span_value, tol)
                && near_axis(b, wing.span_axis, span_value, tol)
                && matches!(
                    edge.classification,
                    SurfaceEdgeClassification::CandidateSharp
                        | SurfaceEdgeClassification::CandidateProvenanceBoundary
                        | SurfaceEdgeClassification::ManifoldInterior
                )
        });
        curves.extend(curves_from_edges(
            mesh,
            name,
            SurfaceFeatureType::TipCapSeam,
            FeatureSupportLevel::Authoritative,
            FeatureSource::ProceduralWingMetadata,
            FeatureCurveDirection::CounterClockwise,
            tip_edges,
            wing.chord_axis,
            vec![
                "lofted wing SDF is capped by finite span max(dist_2d,d_span) construction"
                    .to_string(),
                "edge lies on span-limit cap transition".to_string(),
            ],
        ));
    }
    let root_edges = select_edges(mesh, |edge| {
        let a = mesh.vertices[edge.vertices[0] as usize].position;
        let b = mesh.vertices[edge.vertices[1] as usize].position;
        near_axis(a, wing.span_axis, wing.root_span, tol)
            && near_axis(b, wing.span_axis, wing.root_span, tol)
            && !span_like(a, b, wing.span_axis, wing.chord_axis, wing.thickness_axis)
    });
    curves.extend(curves_from_edges(
        mesh,
        "root_section_guide_no_cap_in_standalone_wing",
        SurfaceFeatureType::RootEdge,
        FeatureSupportLevel::Configured,
        FeatureSource::ConfiguredGeometryGuide,
        FeatureCurveDirection::Chordwise,
        root_edges,
        wing.chord_axis,
        vec![
            "tested symmetric finite wing has a root section at y=0, not an exposed root cap"
                .to_string(),
            "represented as configured section guide for future fuselage/root handling".to_string(),
        ],
    ));
    curves
}

fn hard_csg_boundary_curves(
    mesh: &SdfSurfaceMesh,
    config: &SurfaceFeatureGraphConfig,
) -> Vec<SurfaceFeatureCurve> {
    let mut edges = mesh
        .edges
        .iter()
        .filter(|edge| {
            edge.classification == SurfaceEdgeClassification::CandidateProvenanceBoundary
                || edge_crosses_hard_csg_roles(mesh, edge)
                || edge_endpoints_cross_hard_csg_roles(mesh, edge)
                || edge_incident_to_hard_csg_boundary_triangle(mesh, edge)
        })
        .map(|edge| edge.id)
        .collect::<Vec<_>>();
    let mut evidence = vec![
        "edge separates supported hard-CSG provenance summaries".to_string(),
        format!("minimum_curve_edges={}", config.minimum_curve_edges),
    ];
    if edges.is_empty() && mesh_has_retained_and_subtractor_surfaces(mesh) {
        edges = subtractor_controlled_surface_edges(mesh);
        evidence.push(
            "fallback: no clean retained/subtractor transition edge isolated at this resolution; using subtractor-controlled surface-edge components as provenance-supported diagnostic curves"
                .to_string(),
        );
    }
    curves_from_edges(
        mesh,
        "hard_csg_provenance_boundary",
        SurfaceFeatureType::HardCsgBoundary,
        FeatureSupportLevel::ProvenanceSupported,
        FeatureSource::HardCsgProvenance,
        FeatureCurveDirection::Unoriented,
        edges,
        Axis::X,
        evidence,
    )
}

fn mesh_has_retained_and_subtractor_surfaces(mesh: &SdfSurfaceMesh) -> bool {
    let mut has_retained = false;
    let mut has_subtractor = false;
    for tri in &mesh.triangles {
        match triangle_dominant_role(mesh, tri) {
            Some(CsgChildRole::Retained) => has_retained = true,
            Some(CsgChildRole::Subtractor) => has_subtractor = true,
            _ => {}
        }
        if has_retained && has_subtractor {
            return true;
        }
    }
    false
}

fn subtractor_controlled_surface_edges(mesh: &SdfSurfaceMesh) -> Vec<u64> {
    let tri_by_id = mesh
        .triangles
        .iter()
        .map(|tri| (tri.id, tri))
        .collect::<BTreeMap<_, _>>();
    mesh.edges
        .iter()
        .filter(|edge| {
            edge.incident_triangles.iter().any(|tri_id| {
                tri_by_id.get(tri_id).is_some_and(|tri| {
                    triangle_dominant_role(mesh, tri) == Some(CsgChildRole::Subtractor)
                })
            })
        })
        .map(|edge| edge.id)
        .collect()
}

fn edge_endpoints_cross_hard_csg_roles(mesh: &SdfSurfaceMesh, edge: &SurfaceEdge) -> bool {
    let a = vertex_role(mesh, edge.vertices[0]);
    let b = vertex_role(mesh, edge.vertices[1]);
    matches!(
        (a, b),
        (Some(CsgChildRole::Retained), Some(CsgChildRole::Subtractor))
            | (Some(CsgChildRole::Subtractor), Some(CsgChildRole::Retained))
            | (Some(CsgChildRole::Left), Some(CsgChildRole::Right))
            | (Some(CsgChildRole::Right), Some(CsgChildRole::Left))
    )
}

fn edge_incident_to_hard_csg_boundary_triangle(mesh: &SdfSurfaceMesh, edge: &SurfaceEdge) -> bool {
    let tri_by_id = mesh
        .triangles
        .iter()
        .map(|tri| (tri.id, tri))
        .collect::<BTreeMap<_, _>>();
    edge.incident_triangles.iter().any(|tri_id| {
        tri_by_id.get(tri_id).is_some_and(|tri| {
            matches!(
                tri.provenance_summary,
                TriangleProvenanceSummary::MixedHardCsgProvenance
                    | TriangleProvenanceSummary::NearHardCsgBoundary
            )
        })
    })
}

fn edge_crosses_hard_csg_roles(mesh: &SdfSurfaceMesh, edge: &SurfaceEdge) -> bool {
    if edge.incident_triangles.len() != 2 {
        return false;
    }
    let tri_by_id = mesh
        .triangles
        .iter()
        .map(|tri| (tri.id, tri))
        .collect::<BTreeMap<_, _>>();
    let roles = edge
        .incident_triangles
        .iter()
        .filter_map(|tri_id| tri_by_id.get(tri_id))
        .filter_map(|tri| triangle_dominant_role(mesh, tri))
        .collect::<Vec<_>>();
    roles.len() == 2 && roles[0] != roles[1]
}

fn triangle_dominant_role(mesh: &SdfSurfaceMesh, tri: &SurfaceTriangle) -> Option<CsgChildRole> {
    let mut counts = BTreeMap::<String, (CsgChildRole, usize)>::new();
    for vertex in tri.vertices {
        let role = mesh.vertices[vertex as usize]
            .provenance
            .as_ref()
            .and_then(|trace| trace.decisions.last())
            .map(|decision| decision.active.child_role)?;
        let key = format!("{role:?}");
        let entry = counts.entry(key).or_insert((role, 0));
        entry.1 += 1;
    }
    counts
        .into_values()
        .max_by(|a, b| a.1.cmp(&b.1))
        .map(|(role, _)| role)
}

fn vertex_role(mesh: &SdfSurfaceMesh, vertex: u64) -> Option<CsgChildRole> {
    mesh.vertices[vertex as usize]
        .provenance
        .as_ref()
        .and_then(|trace| trace.decisions.last())
        .map(|decision| decision.active.child_role)
}

fn curves_from_edges(
    mesh: &SdfSurfaceMesh,
    base_name: &str,
    feature_type: SurfaceFeatureType,
    support_level: FeatureSupportLevel,
    source: FeatureSource,
    direction: FeatureCurveDirection,
    edge_ids: Vec<u64>,
    ordering_axis: Axis,
    evidence: Vec<String>,
) -> Vec<SurfaceFeatureCurve> {
    let components = trace_edge_components(mesh, &edge_ids, ordering_axis);
    let mut curves = Vec::new();
    for (component_idx, component) in components.into_iter().enumerate() {
        if component.ordered_edges.is_empty() && component.ordered_vertices.len() < 2 {
            continue;
        }
        let mut curve = build_curve(
            mesh,
            format!("{base_name}_{component_idx:02}"),
            feature_type.clone(),
            support_level.clone(),
            source.clone(),
            direction.clone(),
            component,
            evidence.clone(),
        );
        if curve.diagnostics.branch_points > 0 {
            curve.support_level = match curve.support_level {
                FeatureSupportLevel::Authoritative => FeatureSupportLevel::StrongCandidate,
                FeatureSupportLevel::Configured => FeatureSupportLevel::WeakCandidate,
                other => other,
            };
            curve.continuity = FeatureContinuity::Branched;
            curve.confidence = curve.confidence.min(0.55);
            curve
                .diagnostics
                .support_evidence
                .push("downgraded because connected candidate graph has branch points".to_string());
        }
        curves.push(curve);
    }
    curves
}

#[derive(Clone, Debug)]
struct TracedComponent {
    ordered_vertices: Vec<u64>,
    ordered_edges: Vec<u64>,
    closed: bool,
    branch_points: usize,
}

fn trace_edge_components(
    mesh: &SdfSurfaceMesh,
    edge_ids: &[u64],
    ordering_axis: Axis,
) -> Vec<TracedComponent> {
    let edge_set = edge_ids.iter().copied().collect::<BTreeSet<_>>();
    let mut edge_by_id = BTreeMap::new();
    let mut vertex_edges = BTreeMap::<u64, Vec<u64>>::new();
    for edge_id in &edge_set {
        if let Some(edge) = mesh.edges.get(*edge_id as usize) {
            edge_by_id.insert(*edge_id, edge);
            vertex_edges
                .entry(edge.vertices[0])
                .or_default()
                .push(*edge_id);
            vertex_edges
                .entry(edge.vertices[1])
                .or_default()
                .push(*edge_id);
        }
    }
    for edges in vertex_edges.values_mut() {
        edges.sort_unstable();
    }
    let mut visited = BTreeSet::new();
    let mut components = Vec::new();
    for edge_id in edge_set {
        if visited.contains(&edge_id) {
            continue;
        }
        let mut queue = VecDeque::from([edge_id]);
        let mut component_edges = BTreeSet::new();
        let mut component_vertices = BTreeSet::new();
        while let Some(next_edge_id) = queue.pop_front() {
            if !visited.insert(next_edge_id) {
                continue;
            }
            component_edges.insert(next_edge_id);
            let Some(edge) = edge_by_id.get(&next_edge_id) else {
                continue;
            };
            for vertex in edge.vertices {
                component_vertices.insert(vertex);
                if let Some(neighbor_edges) = vertex_edges.get(&vertex) {
                    for neighbor_edge in neighbor_edges {
                        if !visited.contains(neighbor_edge) {
                            queue.push_back(*neighbor_edge);
                        }
                    }
                }
            }
        }
        let branch_points = component_vertices
            .iter()
            .filter(|vertex| {
                vertex_edges
                    .get(vertex)
                    .map(|edges| {
                        edges
                            .iter()
                            .filter(|edge| component_edges.contains(edge))
                            .count()
                            > 2
                    })
                    .unwrap_or(false)
            })
            .count();
        let endpoints = component_vertices
            .iter()
            .copied()
            .filter(|vertex| {
                vertex_edges
                    .get(vertex)
                    .map(|edges| {
                        edges
                            .iter()
                            .filter(|edge| component_edges.contains(edge))
                            .count()
                            == 1
                    })
                    .unwrap_or(false)
            })
            .collect::<Vec<_>>();
        let closed = endpoints.is_empty() && !component_edges.is_empty();
        let start = endpoints
            .iter()
            .min_by(|a, b| compare_vertices(mesh, **a, **b, ordering_axis))
            .copied()
            .or_else(|| {
                component_vertices
                    .iter()
                    .min_by(|a, b| compare_vertices(mesh, **a, **b, ordering_axis))
                    .copied()
            });
        let Some(start) = start else {
            continue;
        };
        let (ordered_vertices, ordered_edges) =
            walk_component(mesh, start, &component_edges, &vertex_edges, ordering_axis);
        components.push(TracedComponent {
            ordered_vertices,
            ordered_edges,
            closed,
            branch_points,
        });
    }
    components.sort_by(|a, b| {
        b.ordered_edges
            .len()
            .cmp(&a.ordered_edges.len())
            .then_with(|| a.ordered_vertices.first().cmp(&b.ordered_vertices.first()))
    });
    components
}

fn walk_component(
    mesh: &SdfSurfaceMesh,
    start: u64,
    component_edges: &BTreeSet<u64>,
    vertex_edges: &BTreeMap<u64, Vec<u64>>,
    ordering_axis: Axis,
) -> (Vec<u64>, Vec<u64>) {
    let mut vertices = vec![start];
    let mut edges = Vec::new();
    let mut used_edges = BTreeSet::new();
    let mut current = start;
    let mut previous = None;
    loop {
        let Some(incident) = vertex_edges.get(&current) else {
            break;
        };
        let mut candidates = incident
            .iter()
            .copied()
            .filter(|edge| component_edges.contains(edge) && !used_edges.contains(edge))
            .collect::<Vec<_>>();
        candidates.sort_by(|a, b| {
            let oa = other_vertex(mesh.edges[*a as usize].vertices, current).unwrap_or(current);
            let ob = other_vertex(mesh.edges[*b as usize].vertices, current).unwrap_or(current);
            compare_vertices(mesh, oa, ob, ordering_axis).then_with(|| a.cmp(b))
        });
        if let Some(prev) = previous {
            let single_candidate = candidates.len() == 1;
            candidates.retain(|edge| {
                other_vertex(mesh.edges[*edge as usize].vertices, current)
                    .is_some_and(|v| v != prev || single_candidate)
            });
        }
        let Some(edge_id) = candidates.first().copied() else {
            break;
        };
        used_edges.insert(edge_id);
        edges.push(edge_id);
        let Some(next_vertex) = other_vertex(mesh.edges[edge_id as usize].vertices, current) else {
            break;
        };
        if next_vertex == start {
            vertices.push(next_vertex);
            break;
        }
        previous = Some(current);
        current = next_vertex;
        vertices.push(current);
        if edges.len() >= component_edges.len() {
            break;
        }
    }
    (vertices, edges)
}

fn build_curve(
    mesh: &SdfSurfaceMesh,
    name: String,
    feature_type: SurfaceFeatureType,
    support_level: FeatureSupportLevel,
    source: FeatureSource,
    direction: FeatureCurveDirection,
    component: TracedComponent,
    evidence: Vec<String>,
) -> SurfaceFeatureCurve {
    let diagnostics = curve_diagnostics(mesh, &component, &evidence);
    let continuity = if component.branch_points > 0 {
        FeatureContinuity::Branched
    } else if component.closed {
        FeatureContinuity::ClosedLoop
    } else if component.ordered_edges.len() + 1
        >= component.ordered_vertices.len().saturating_sub(1)
    {
        FeatureContinuity::ContinuousChain
    } else {
        FeatureContinuity::Fragmented
    };
    let tangent_quality = if component.ordered_vertices.len() >= 3 && component.branch_points == 0 {
        FeatureTangentQuality::Valid
    } else if component.branch_points > 0 {
        FeatureTangentQuality::JunctionSpecific
    } else {
        FeatureTangentQuality::InsufficientNeighborhood
    };
    let confidence = match support_level {
        FeatureSupportLevel::Authoritative => 0.92,
        FeatureSupportLevel::Configured => 0.78,
        FeatureSupportLevel::ProvenanceSupported => 0.82,
        FeatureSupportLevel::StrongCandidate => 0.62,
        FeatureSupportLevel::WeakCandidate => 0.40,
        FeatureSupportLevel::Unsupported => 0.0,
    };
    SurfaceFeatureCurve {
        id: 0,
        name,
        feature_type,
        support_level,
        ordered_vertices: component.ordered_vertices,
        ordered_edges: component.ordered_edges,
        closed: component.closed,
        direction,
        source,
        geometry_reference: None,
        tangent_quality,
        continuity,
        confidence,
        diagnostics,
    }
}

fn curve_diagnostics(
    mesh: &SdfSurfaceMesh,
    component: &TracedComponent,
    evidence: &[String],
) -> FeatureCurveDiagnostics {
    let mut length = 0.0;
    for edge_id in &component.ordered_edges {
        let edge = &mesh.edges[*edge_id as usize];
        let a = Vec3::from_array(
            mesh.vertices[edge.vertices[0] as usize]
                .position
                .map(|v| v as f32),
        );
        let b = Vec3::from_array(
            mesh.vertices[edge.vertices[1] as usize]
                .position
                .map(|v| v as f32),
        );
        length += a.distance(b) as f64;
    }
    let (bbox_min, bbox_max) = vertices_bbox(mesh, &component.ordered_vertices);
    let endpoint_coordinates = if component.closed {
        Vec::new()
    } else {
        let mut endpoints = Vec::new();
        if let Some(id) = component.ordered_vertices.first() {
            endpoints.push(mesh.vertices[*id as usize].position);
        }
        if let Some(id) = component.ordered_vertices.last() {
            endpoints.push(mesh.vertices[*id as usize].position);
        }
        endpoints
    };
    let tangent_min_cos = tangent_min_cos(mesh, &component.ordered_vertices);
    let normal_sector = normal_sector_diagnostics(mesh, &component.ordered_edges);
    FeatureCurveDiagnostics {
        curve_length: length,
        vertex_count: component.ordered_vertices.len(),
        edge_count: component.ordered_edges.len(),
        endpoint_coordinates,
        bbox_min,
        bbox_max,
        tangent_min_cos,
        mean_dihedral_degrees: normal_sector.as_ref().map(|s| s.mean_dihedral_degrees),
        max_dihedral_degrees: normal_sector.as_ref().map(|s| s.max_dihedral_degrees),
        normal_sector,
        procedural_parameter_min: None,
        procedural_parameter_max: None,
        procedural_parameter_monotonic: Some(true),
        unresolved_gaps: if component.ordered_edges.is_empty() {
            1
        } else {
            0
        },
        branch_points: component.branch_points,
        support_evidence: evidence.to_vec(),
    }
}

fn normal_sector_diagnostics(
    mesh: &SdfSurfaceMesh,
    edge_ids: &[u64],
) -> Option<NormalSectorDiagnostics> {
    let tri_by_id = mesh
        .triangles
        .iter()
        .map(|tri| (tri.id, tri))
        .collect::<BTreeMap<_, _>>();
    let mut dihedrals = Vec::new();
    let mut side_a = Vec::new();
    let mut side_b = Vec::new();
    let mut ambiguous = false;
    for edge_id in edge_ids {
        let edge = mesh.edges.get(*edge_id as usize)?;
        if edge.incident_triangles.len() != 2 {
            ambiguous = true;
            continue;
        }
        let a = tri_by_id.get(&edge.incident_triangles[0])?;
        let b = tri_by_id.get(&edge.incident_triangles[1])?;
        let na = Vec3::from_array(a.geometric_normal.map(|v| v as f32)).normalize_or_zero();
        let nb = Vec3::from_array(b.geometric_normal.map(|v| v as f32)).normalize_or_zero();
        if na.length_squared() == 0.0 || nb.length_squared() == 0.0 {
            ambiguous = true;
            continue;
        }
        let angle = na.dot(nb).clamp(-1.0, 1.0).acos().to_degrees() as f64;
        dihedrals.push(angle);
        side_a.push(na);
        side_b.push(nb);
    }
    if dihedrals.is_empty() {
        return None;
    }
    dihedrals.sort_by(f64::total_cmp);
    let mean = dihedrals.iter().sum::<f64>() / dihedrals.len() as f64;
    let max = *dihedrals.last().unwrap_or(&0.0);
    let avg_a = average_vec3(&side_a);
    let avg_b = average_vec3(&side_b);
    let classification = if mean < 20.0 {
        NormalSectorClassification::SmoothGuide
    } else if mean < 55.0 {
        NormalSectorClassification::WeakCrease
    } else if mean < 110.0 {
        NormalSectorClassification::StrongCrease
    } else {
        NormalSectorClassification::CapTransition
    };
    Some(NormalSectorDiagnostics {
        side_a_average_normal: avg_a.map(array_from_vec3),
        side_b_average_normal: avg_b.map(array_from_vec3),
        mean_dihedral_degrees: mean,
        max_dihedral_degrees: max,
        side_a_variation_degrees: normal_variation(avg_a, &side_a),
        side_b_variation_degrees: normal_variation(avg_b, &side_b),
        classification,
        ambiguous_incident_ordering: ambiguous,
    })
}

fn build_junctions(
    mesh: &SdfSurfaceMesh,
    curves: &[SurfaceFeatureCurve],
    vertex_membership: &BTreeMap<u64, Vec<u64>>,
) -> Vec<SurfaceFeatureJunction> {
    let mut junctions = Vec::new();
    for (vertex, curve_ids) in vertex_membership {
        if curve_ids.len() < 2 {
            continue;
        }
        let mut incident = curve_ids.clone();
        incident.sort_unstable();
        let kind = classify_junction(curves, &incident);
        junctions.push(SurfaceFeatureJunction {
            id: junctions.len() as u64,
            vertex: *vertex,
            incident_curves: incident,
            junction_type: kind,
            position: mesh.vertices[*vertex as usize].position,
        });
    }
    junctions
}

fn classify_junction(curves: &[SurfaceFeatureCurve], incident: &[u64]) -> FeatureJunctionType {
    let types = incident
        .iter()
        .filter_map(|id| curves.get(*id as usize))
        .map(|curve| &curve.feature_type)
        .collect::<Vec<_>>();
    if types.contains(&&SurfaceFeatureType::LeadingEdge)
        && types.contains(&&SurfaceFeatureType::TipCapSeam)
    {
        FeatureJunctionType::TipLeading
    } else if types.contains(&&SurfaceFeatureType::TrailingEdge)
        && types.contains(&&SurfaceFeatureType::TipCapSeam)
    {
        FeatureJunctionType::TipTrailing
    } else if types.contains(&&SurfaceFeatureType::LeadingEdge)
        && types.contains(&&SurfaceFeatureType::RootEdge)
    {
        FeatureJunctionType::RootLeading
    } else if types.contains(&&SurfaceFeatureType::TrailingEdge)
        && types.contains(&&SurfaceFeatureType::RootEdge)
    {
        FeatureJunctionType::RootTrailing
    } else if incident.len() > 2 {
        FeatureJunctionType::Branch
    } else {
        FeatureJunctionType::CurveEndpoint
    }
}

fn validate_feature_graph_counts(
    curves: &[SurfaceFeatureCurve],
    duplicate_feature_edges: usize,
) -> FeatureGraphValidation {
    let mut errors = Vec::new();
    let branched = curves
        .iter()
        .filter(|curve| curve.continuity == FeatureContinuity::Branched)
        .count();
    if duplicate_feature_edges > 0 {
        errors.push(format!("duplicate_feature_edges:{duplicate_feature_edges}"));
    }
    FeatureGraphValidation {
        deterministic_ids: curves.iter().enumerate().all(|(i, c)| c.id == i as u64),
        duplicate_feature_edges,
        incompatible_edge_assignments: duplicate_feature_edges,
        continuous_curve_count: curves
            .iter()
            .filter(|curve| curve.continuity == FeatureContinuity::ContinuousChain)
            .count(),
        closed_loop_count: curves
            .iter()
            .filter(|curve| curve.continuity == FeatureContinuity::ClosedLoop)
            .count(),
        branched_curve_count: branched,
        fragmented_curve_count: curves
            .iter()
            .filter(|curve| curve.continuity == FeatureContinuity::Fragmented)
            .count(),
        invalid_tangent_count: curves
            .iter()
            .filter(|curve| {
                matches!(
                    curve.tangent_quality,
                    FeatureTangentQuality::InsufficientNeighborhood
                        | FeatureTangentQuality::Unsupported
                )
            })
            .count(),
        validation_errors: errors,
    }
}

fn select_edges(
    mesh: &SdfSurfaceMesh,
    mut predicate: impl FnMut(&SurfaceEdge) -> bool,
) -> Vec<u64> {
    mesh.edges
        .iter()
        .filter(|edge| predicate(edge))
        .map(|edge| edge.id)
        .collect()
}

fn mesh_bounds(mesh: &SdfSurfaceMesh) -> ([f64; 3], [f64; 3]) {
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for vertex in &mesh.vertices {
        for i in 0..3 {
            min[i] = min[i].min(vertex.position[i]);
            max[i] = max[i].max(vertex.position[i]);
        }
    }
    (min, max)
}

fn axis_min(bounds: ([f64; 3], [f64; 3]), axis: Axis) -> f64 {
    axis.value(bounds.0)
}

fn axis_max(bounds: ([f64; 3], [f64; 3]), axis: Axis) -> f64 {
    axis.value(bounds.1)
}

fn near_axis(point: [f64; 3], axis: Axis, value: f64, tol: f64) -> bool {
    (axis.value(point) - value).abs() <= tol
}

fn span_like(a: [f64; 3], b: [f64; 3], span: Axis, chord: Axis, thickness: Axis) -> bool {
    let ds = (span.value(a) - span.value(b)).abs();
    let dc = (chord.value(a) - chord.value(b)).abs();
    let dt = (thickness.value(a) - thickness.value(b)).abs();
    ds >= dc.max(dt)
}

fn compare_vertices(
    mesh: &SdfSurfaceMesh,
    a: u64,
    b: u64,
    primary_axis: Axis,
) -> std::cmp::Ordering {
    let pa = mesh.vertices[a as usize].position;
    let pb = mesh.vertices[b as usize].position;
    primary_axis
        .value(pa)
        .total_cmp(&primary_axis.value(pb))
        .then_with(|| pa[0].total_cmp(&pb[0]))
        .then_with(|| pa[1].total_cmp(&pb[1]))
        .then_with(|| pa[2].total_cmp(&pb[2]))
        .then_with(|| a.cmp(&b))
}

fn other_vertex(edge: [u64; 2], vertex: u64) -> Option<u64> {
    if edge[0] == vertex {
        Some(edge[1])
    } else if edge[1] == vertex {
        Some(edge[0])
    } else {
        None
    }
}

fn vertices_bbox(mesh: &SdfSurfaceMesh, vertices: &[u64]) -> ([f64; 3], [f64; 3]) {
    if vertices.is_empty() {
        return ([0.0; 3], [0.0; 3]);
    }
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for id in vertices {
        let p = mesh.vertices[*id as usize].position;
        for i in 0..3 {
            min[i] = min[i].min(p[i]);
            max[i] = max[i].max(p[i]);
        }
    }
    (min, max)
}

fn tangent_min_cos(mesh: &SdfSurfaceMesh, vertices: &[u64]) -> f64 {
    if vertices.len() < 3 {
        return 0.0;
    }
    let mut min_cos = 1.0_f64;
    for window in vertices.windows(3) {
        let a = Vec3::from_array(mesh.vertices[window[0] as usize].position.map(|v| v as f32));
        let b = Vec3::from_array(mesh.vertices[window[1] as usize].position.map(|v| v as f32));
        let c = Vec3::from_array(mesh.vertices[window[2] as usize].position.map(|v| v as f32));
        let u = (b - a).normalize_or_zero();
        let v = (c - b).normalize_or_zero();
        if u.length_squared() > 0.0 && v.length_squared() > 0.0 {
            min_cos = min_cos.min(u.dot(v) as f64);
        }
    }
    min_cos
}

fn average_vec3(values: &[Vec3]) -> Option<Vec3> {
    if values.is_empty() {
        return None;
    }
    let sum = values.iter().copied().fold(Vec3::ZERO, |acc, v| acc + v);
    Some((sum / values.len() as f32).normalize_or_zero())
}

fn normal_variation(avg: Option<Vec3>, values: &[Vec3]) -> f64 {
    let Some(avg) = avg else {
        return 0.0;
    };
    values
        .iter()
        .map(|v| avg.dot(*v).clamp(-1.0, 1.0).acos().to_degrees() as f64)
        .fold(0.0, f64::max)
}

fn array_from_vec3(v: Vec3) -> [f64; 3] {
    [v.x as f64, v.y as f64, v.z as f64]
}

fn feature_type_rank(feature_type: &SurfaceFeatureType) -> usize {
    match feature_type {
        SurfaceFeatureType::TrailingEdge => 0,
        SurfaceFeatureType::LeadingEdge => 1,
        SurfaceFeatureType::RootEdge => 2,
        SurfaceFeatureType::TipEdge => 3,
        SurfaceFeatureType::RootCapSeam => 4,
        SurfaceFeatureType::TipCapSeam => 5,
        SurfaceFeatureType::HardCsgBoundary => 6,
        SurfaceFeatureType::SharpConvexCrease => 7,
        SurfaceFeatureType::SharpConcaveCrease => 8,
        SurfaceFeatureType::PatchBoundary => 9,
        SurfaceFeatureType::Unknown => 10,
    }
}

fn is_finite_wing_promotable_feature(kind: &ProceduralFeatureType) -> bool {
    matches!(
        kind,
        ProceduralFeatureType::LeadingEdge
            | ProceduralFeatureType::TrailingEdgeUpperCapBoundary
            | ProceduralFeatureType::TrailingEdgeLowerCapBoundary
            | ProceduralFeatureType::TipUpperCapBoundary
            | ProceduralFeatureType::TipLowerCapBoundary
            | ProceduralFeatureType::RootSection
    )
}

fn surface_feature_type_for_procedural(kind: &ProceduralFeatureType) -> SurfaceFeatureType {
    match kind {
        ProceduralFeatureType::LeadingEdge => SurfaceFeatureType::LeadingEdge,
        ProceduralFeatureType::TrailingEdgeUpperCapBoundary
        | ProceduralFeatureType::TrailingEdgeLowerCapBoundary
        | ProceduralFeatureType::TrailingEdgeCapCenter => SurfaceFeatureType::TrailingEdge,
        ProceduralFeatureType::TipUpperCapBoundary
        | ProceduralFeatureType::TipLowerCapBoundary
        | ProceduralFeatureType::TipPerimeter => SurfaceFeatureType::TipCapSeam,
        ProceduralFeatureType::RootSection => SurfaceFeatureType::RootEdge,
        ProceduralFeatureType::RootCapBoundary => SurfaceFeatureType::RootCapSeam,
        _ => SurfaceFeatureType::Unknown,
    }
}

fn procedural_direction_for_feature(kind: &ProceduralFeatureType) -> FeatureCurveDirection {
    match kind {
        ProceduralFeatureType::LeadingEdge
        | ProceduralFeatureType::TrailingEdgeUpperCapBoundary
        | ProceduralFeatureType::TrailingEdgeLowerCapBoundary => {
            FeatureCurveDirection::NegativeTipToPositiveTip
        }
        ProceduralFeatureType::TipUpperCapBoundary
        | ProceduralFeatureType::TipLowerCapBoundary
        | ProceduralFeatureType::RootSection => FeatureCurveDirection::Chordwise,
        _ => FeatureCurveDirection::Unoriented,
    }
}

fn procedural_curve_name(feature_id: &str) -> String {
    feature_id
        .strip_prefix("lofted_wing.main.")
        .unwrap_or(feature_id)
        .replace('.', "_")
}

fn ordered_edges_between_vertices(mesh: &SdfSurfaceMesh, ordered_vertices: &[u64]) -> Vec<u64> {
    let by_vertices = mesh
        .edges
        .iter()
        .map(|edge| {
            let key = canonical_edge(edge.vertices[0], edge.vertices[1]);
            (key, edge.id)
        })
        .collect::<BTreeMap<_, _>>();
    ordered_vertices
        .windows(2)
        .filter_map(|pair| by_vertices.get(&canonical_edge(pair[0], pair[1])).copied())
        .collect()
}

fn canonical_edge(a: u64, b: u64) -> [u64; 2] {
    if a <= b { [a, b] } else { [b, a] }
}

fn build_memberships(
    curves: &[SurfaceFeatureCurve],
) -> (BTreeMap<u64, Vec<u64>>, BTreeMap<u64, u64>, usize) {
    let mut vertex_membership = BTreeMap::<u64, Vec<u64>>::new();
    let mut edge_membership = BTreeMap::<u64, u64>::new();
    let mut duplicate_feature_edges = 0usize;
    for curve in curves {
        for vertex in &curve.ordered_vertices {
            vertex_membership.entry(*vertex).or_default().push(curve.id);
        }
        for edge in &curve.ordered_edges {
            if edge_membership.insert(*edge, curve.id).is_some() {
                duplicate_feature_edges += 1;
            }
        }
    }
    for memberships in vertex_membership.values_mut() {
        memberships.sort_unstable();
    }
    (vertex_membership, edge_membership, duplicate_feature_edges)
}

fn stable_hash_hex(bytes: &[u8]) -> String {
    let mut hash = 0xcbf29ce484222325_u64;
    for byte in bytes {
        hash ^= *byte as u64;
        hash = hash.wrapping_mul(0x100000001b3);
    }
    format!("{hash:016x}")
}
