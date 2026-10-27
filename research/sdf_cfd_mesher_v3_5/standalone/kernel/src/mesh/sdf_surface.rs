use crate::mesh::Mesh;
use crate::mesh::marching_cubes;
use crate::sdf::Sdf;
use crate::sdf::cfd_query::{
    GeometryQueryKernel, QueryValueStatus, SDF_CFD_QUERY_SCHEMA_VERSION, SdfCfdQueryOptions,
};
use crate::sdf::provenance::{CsgChildRole, CsgProvenanceTrace, CsgTraceMode, CsgTraceSupport};
use crate::sdf::query::bounding_points;
use glam::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::sync::Arc;
use std::time::Instant;

pub const SDF_SURFACE_MESH_SCHEMA_VERSION: &str = "sdf_surface_mesh.v1";
pub const SDF_SURFACE_EXTRACTOR_VERSION: &str =
    "uniform_marching_cubes_canonical_grid_edges_projected.v2";

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct SurfaceExtractionConfig {
    pub target_spacing: f64,
    pub min_spacing: f64,
    pub max_spacing: f64,
    pub resolution_override: Option<u32>,
    pub bounds_padding: f64,
    pub weld_tolerance: f64,
    pub projection_tolerance: f64,
    pub projection_max_iterations: usize,
    pub max_projection_displacement_factor: f64,
    pub min_gradient_magnitude: f64,
    pub require_closed: bool,
    pub require_manifold: bool,
    pub sharp_dihedral_degrees: f64,
}

impl Default for SurfaceExtractionConfig {
    fn default() -> Self {
        Self {
            target_spacing: 2.0,
            min_spacing: 0.5,
            max_spacing: 25.0,
            resolution_override: None,
            bounds_padding: 4.0,
            weld_tolerance: 1.0e-4,
            projection_tolerance: 1.0e-4,
            projection_max_iterations: 32,
            max_projection_displacement_factor: 2.5,
            min_gradient_magnitude: 1.0e-8,
            require_closed: true,
            require_manifold: true,
            sharp_dihedral_degrees: 45.0,
        }
    }
}

impl SurfaceExtractionConfig {
    pub fn with_target_spacing(mut self, target_spacing: f64) -> Self {
        self.target_spacing = target_spacing;
        self
    }

    pub fn with_resolution(mut self, resolution: u32) -> Self {
        self.resolution_override = Some(resolution);
        self
    }

    pub fn config_hash(&self) -> String {
        stable_hash_hex(&serde_json::to_vec(self).unwrap_or_default())
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct SdfSurfaceMesh {
    pub schema_version: String,
    pub vertices: Vec<SurfaceVertex>,
    pub triangles: Vec<SurfaceTriangle>,
    pub edges: Vec<SurfaceEdge>,
    pub patches: Vec<SurfacePatch>,
    pub feature_edges: Vec<FeatureEdge>,
    pub metadata: SurfaceMeshMetadata,
}

impl SdfSurfaceMesh {
    pub fn mesh_hash(&self) -> String {
        stable_hash_hex(&serde_json::to_vec(self).unwrap_or_default())
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct SurfaceVertex {
    pub id: u64,
    pub position: [f64; 3],
    pub signed_distance_residual: f64,
    pub raw_gradient: Option<[f64; 3]>,
    pub unit_normal: Option<[f64; 3]>,
    pub projection_status: ProjectionStatus,
    pub provenance: Option<CsgProvenanceTrace>,
    pub provenance_status: QueryValueStatus,
    pub patch_id: Option<u32>,
    pub feature_flags: SurfaceFeatureFlags,
    pub local_target_size: Option<f64>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct ProjectionStatus {
    pub pre_projection_signed_distance: f64,
    pub projected_point: Option<[f64; 3]>,
    pub residual_signed_distance: Option<f64>,
    pub iteration_count: Option<usize>,
    pub converged: bool,
    pub displacement_magnitude: Option<f64>,
    pub final_gradient_magnitude: Option<f64>,
    pub failure_reason: Option<String>,
    pub method: String,
}

#[derive(Clone, Debug, Default, Deserialize, Eq, PartialEq, Serialize)]
pub struct SurfaceFeatureFlags {
    pub candidate_sharp: bool,
    pub provenance_boundary: bool,
    pub patch_boundary: bool,
    pub smooth_csg_unsupported: bool,
    pub uncertain: bool,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct SurfaceTriangle {
    pub id: u64,
    pub vertices: [u64; 3],
    pub geometric_normal: [f64; 3],
    pub area: f64,
    pub patch_id: Option<u32>,
    pub provenance_summary: TriangleProvenanceSummary,
    pub quality: TriangleQuality,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct TriangleQuality {
    pub min_edge_length: f64,
    pub max_edge_length: f64,
    pub min_angle_degrees: f64,
    pub max_angle_degrees: f64,
    pub aspect_ratio: f64,
    pub radius_ratio: f64,
    pub local_size_error: Option<f64>,
    pub normal_angle_to_sdf_degrees: Option<f64>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TriangleProvenanceSummary {
    Consistent {
        key: String,
        support_status: CsgTraceSupport,
    },
    MixedHardCsgProvenance,
    NearHardCsgBoundary,
    IncompleteProvenance,
    UnsupportedSmoothCsgRegion,
    Unknown,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct SurfaceEdge {
    pub id: u64,
    pub vertices: [u64; 2],
    pub incident_triangles: Vec<u64>,
    pub classification: SurfaceEdgeClassification,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SurfaceEdgeClassification {
    ManifoldInterior,
    Boundary,
    NonManifold,
    Duplicate,
    CandidateSharp,
    CandidateProvenanceBoundary,
    CandidatePatchBoundary,
    Uncertain,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct SurfacePatch {
    pub id: u32,
    pub name: String,
    pub source: PatchSource,
    pub triangle_ids: Vec<u64>,
    pub support_status: QueryValueStatus,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum PatchSource {
    GenericWall,
    SdfMetadata,
    HardCsgTerminal,
    UserMapping,
    Unsupported,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct FeatureEdge {
    pub id: u64,
    pub edge_id: u64,
    pub source: FeatureEdgeSource,
    pub support_status: QueryValueStatus,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FeatureEdgeSource {
    DihedralCandidate,
    ProvenanceBoundaryCandidate,
    PatchBoundaryCandidate,
    MetadataCandidate,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct SurfaceMeshMetadata {
    pub surface_mesh_schema_version: String,
    pub query_schema_version: String,
    pub extractor_version: String,
    pub geometry_hash: String,
    pub configuration_hash: String,
    pub mesh_hash_excluding_metadata: String,
    pub bounds_min: [f64; 3],
    pub bounds_max: [f64; 3],
    pub resolution: u32,
    pub target_spacing: f64,
    pub cleanup: CleanupSummary,
}

#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct CleanupSummary {
    pub raw_vertices: usize,
    pub raw_triangles: usize,
    pub welded_vertices_removed: usize,
    pub duplicate_triangles_removed: usize,
    pub degenerate_triangles_removed: usize,
    pub zero_area_triangles_removed: usize,
    pub orientation_flips: usize,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct SurfaceExtractionOutcome {
    pub mesh: SdfSurfaceMesh,
    pub audit: SurfaceMeshAudit,
    pub timings: SurfaceExtractionTimings,
    pub query_counts: SurfaceQueryCounts,
    pub status: SurfaceExtractionStatus,
    pub failure_reasons: Vec<String>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SurfaceExtractionStatus {
    Success,
    SuccessWithWarnings,
    Failed,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct SurfaceExtractionTimings {
    pub geometry_initialization_ms: f64,
    pub sdf_sampling_and_initial_extraction_ms: f64,
    pub vertex_deduplication_ms: f64,
    pub projection_ms: f64,
    pub orientation_and_triangle_quality_ms: f64,
    pub adjacency_ms: f64,
    pub audit_ms: f64,
    pub serialization_ms: f64,
    pub total_ms: f64,
}

#[derive(Clone, Debug, Default, Deserialize, Eq, PartialEq, Serialize)]
pub struct SurfaceQueryCounts {
    pub distance_queries: usize,
    pub gradient_queries: usize,
    pub projection_queries: usize,
    pub provenance_queries: usize,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct SurfaceMeshAudit {
    pub geometry: GeometryAudit,
    pub topology: TopologyAudit,
    pub triangle_quality: TriangleQualityAudit,
    pub provenance: SurfaceProvenanceAudit,
    pub determinism: DeterminismAudit,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct GeometryAudit {
    pub max_abs_sdf_residual: f64,
    pub mean_abs_sdf_residual: f64,
    pub p95_abs_sdf_residual: f64,
    pub max_projection_displacement: f64,
    pub projection_failures: usize,
    pub non_finite_distance_or_gradient: usize,
    pub invalid_normals: usize,
    pub max_sdf_normal_to_triangle_normal_degrees: f64,
    pub mean_sdf_normal_to_triangle_normal_degrees: f64,
    pub bounds_min: [f64; 3],
    pub bounds_max: [f64; 3],
    pub signed_volume: f64,
    pub orientation_consistent: bool,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct TopologyAudit {
    pub vertex_count: usize,
    pub edge_count: usize,
    pub triangle_count: usize,
    pub connected_components: usize,
    pub boundary_edges: usize,
    pub non_manifold_edges: usize,
    pub duplicate_vertices: usize,
    pub duplicate_triangles: usize,
    pub isolated_vertices: usize,
    pub inconsistent_edge_orientation: usize,
    pub zero_area_triangles: usize,
    pub tiny_disconnected_components: usize,
    pub euler_characteristic: i64,
    pub closed_manifold: bool,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct TriangleQualityAudit {
    pub min_area: f64,
    pub max_area: f64,
    pub mean_area: f64,
    pub min_edge_length: f64,
    pub max_edge_length: f64,
    pub min_angle_degrees: f64,
    pub max_angle_degrees: f64,
    pub max_aspect_ratio: f64,
    pub mean_aspect_ratio: f64,
    pub min_radius_ratio: f64,
    pub max_neighbor_size_ratio: f64,
    pub max_normal_angle_jump_degrees: f64,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct SurfaceProvenanceAudit {
    pub vertices_with_complete_hard_csg_traces: usize,
    pub vertices_with_incomplete_traces: usize,
    pub smooth_csg_termination_count: usize,
    pub near_tie_count: usize,
    pub exact_tie_count: usize,
    pub triangles_with_consistent_provenance: usize,
    pub triangles_with_mixed_provenance: usize,
    pub candidate_provenance_boundary_edges: usize,
    pub subtractor_controlled_area: f64,
    pub retained_controlled_area: f64,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct DeterminismAudit {
    pub repeated: bool,
    pub mesh_hash: String,
    pub repeated_mesh_hash: Option<String>,
    pub exact_match: Option<bool>,
}

#[derive(Clone, Debug)]
pub struct SeedSurface {
    pub vertices: Vec<Vec3>,
    pub triangles: Vec<[u64; 3]>,
    pub cleanup: CleanupSummary,
}

pub fn extract_sdf_surface_mesh(
    sdf: Arc<dyn Sdf>,
    config: SurfaceExtractionConfig,
) -> Result<SurfaceExtractionOutcome, String> {
    let total_start = Instant::now();
    let mut timings = SurfaceExtractionTimings::default();
    let geometry_start = Instant::now();
    let kernel = GeometryQueryKernel::from_sdf(Arc::clone(&sdf)).with_projection_settings(
        config.projection_tolerance as f32,
        config.projection_max_iterations,
    );
    timings.geometry_initialization_ms = elapsed_ms(geometry_start);

    let bbox = extraction_bounding_box(sdf.as_ref());
    if !bbox.min.is_finite() || !bbox.max.is_finite() {
        return Err("invalid SDF bounds for surface extraction".to_string());
    }
    let (domain_min, domain_max) = expand_degenerate_bounds(
        bbox.min,
        bbox.max,
        (config.target_spacing.max(config.bounds_padding) as f32).max(1.0),
    );
    if (domain_max - domain_min).max_element() <= 0.0 {
        return Err("invalid SDF bounds for surface extraction".to_string());
    }
    let bounds_min = domain_min - Vec3::splat(config.bounds_padding as f32);
    let bounds_max = domain_max + Vec3::splat(config.bounds_padding as f32);
    let extent = bounds_max - bounds_min;
    let max_extent = extent.max_element().max(config.target_spacing as f32);
    let resolution = config
        .resolution_override
        .unwrap_or_else(|| (max_extent as f64 / config.target_spacing).ceil() as u32)
        .clamp(8, 96);

    let extraction_start = Instant::now();
    let raw_mesh = marching_cubes::extract_mesh_canonical_edges(
        sdf.as_ref(),
        bounds_min,
        bounds_max,
        resolution,
        false,
    );
    timings.sdf_sampling_and_initial_extraction_ms = elapsed_ms(extraction_start);

    let dedup_start = Instant::now();
    let seed = weld_and_clean_seed_mesh(&raw_mesh, &config);
    timings.vertex_deduplication_ms = elapsed_ms(dedup_start);
    if seed.vertices.is_empty() || seed.triangles.is_empty() {
        return Err("surface extraction produced no triangles".to_string());
    }

    let projection_start = Instant::now();
    let (vertices, query_counts) = project_seed_vertices(&kernel, &seed.vertices, &config)?;
    timings.projection_ms = elapsed_ms(projection_start);

    let orient_start = Instant::now();
    let mut cleanup = seed.cleanup;
    let mut triangles = build_oriented_triangles(&vertices, &seed.triangles, &config, &mut cleanup);
    timings.orientation_and_triangle_quality_ms = elapsed_ms(orient_start);
    if triangles.is_empty() {
        return Err("all triangles were removed after projection/orientation".to_string());
    }

    let adjacency_start = Instant::now();
    let (edges, feature_edges, patches) =
        build_edges_and_patches(&mut triangles, &vertices, &config);
    timings.adjacency_ms = elapsed_ms(adjacency_start);

    let audit_start = Instant::now();
    let metadata = SurfaceMeshMetadata {
        surface_mesh_schema_version: SDF_SURFACE_MESH_SCHEMA_VERSION.to_string(),
        query_schema_version: SDF_CFD_QUERY_SCHEMA_VERSION.to_string(),
        extractor_version: SDF_SURFACE_EXTRACTOR_VERSION.to_string(),
        geometry_hash: geometry_hash(sdf.metadata()),
        configuration_hash: config.config_hash(),
        mesh_hash_excluding_metadata: String::new(),
        bounds_min: array_from_vec3(bounds_min),
        bounds_max: array_from_vec3(bounds_max),
        resolution,
        target_spacing: config.target_spacing,
        cleanup,
    };
    let mut mesh = SdfSurfaceMesh {
        schema_version: SDF_SURFACE_MESH_SCHEMA_VERSION.to_string(),
        vertices,
        triangles,
        edges,
        patches,
        feature_edges,
        metadata,
    };
    mesh.metadata.mesh_hash_excluding_metadata = mesh_content_hash(&mesh);
    let mut audit = audit_surface_mesh(&mesh);
    let mut failure_reasons = Vec::new();
    if config.require_closed && audit.topology.boundary_edges > 0 {
        failure_reasons.push("intended closed surface has boundary edges".to_string());
    }
    if config.require_manifold && audit.topology.non_manifold_edges > 0 {
        failure_reasons.push("intended manifold surface has non-manifold edges".to_string());
    }
    if audit.geometry.projection_failures > 0 {
        failure_reasons.push("one or more vertices failed SDF projection".to_string());
    }
    if audit.geometry.max_abs_sdf_residual > config.projection_tolerance * 10.0 {
        failure_reasons
            .push("projected vertex residual exceeds configured tolerance band".to_string());
    }
    let status = if failure_reasons.is_empty() {
        SurfaceExtractionStatus::Success
    } else {
        SurfaceExtractionStatus::Failed
    };
    audit.determinism.mesh_hash = mesh.mesh_hash();
    timings.audit_ms = elapsed_ms(audit_start);
    timings.total_ms = elapsed_ms(total_start);

    Ok(SurfaceExtractionOutcome {
        mesh,
        audit,
        timings,
        query_counts,
        status,
        failure_reasons,
    })
}

pub fn rebuild_projected_surface_mesh(
    sdf: Arc<dyn Sdf>,
    positions: &[Vec3],
    seed_triangles: &[[u64; 3]],
    config: &SurfaceExtractionConfig,
    extractor_version: impl Into<String>,
) -> Result<(SdfSurfaceMesh, SurfaceMeshAudit, SurfaceQueryCounts), String> {
    let kernel = GeometryQueryKernel::from_sdf(Arc::clone(&sdf)).with_projection_settings(
        config.projection_tolerance as f32,
        config.projection_max_iterations,
    );
    let (vertices, query_counts) = project_seed_vertices(&kernel, positions, config)?;
    let mut cleanup = CleanupSummary {
        raw_vertices: positions.len(),
        raw_triangles: seed_triangles.len(),
        ..Default::default()
    };
    let mut triangles = build_oriented_triangles(&vertices, seed_triangles, config, &mut cleanup);
    if triangles.is_empty() {
        return Err("rebuild produced no valid triangles".to_string());
    }
    let (edges, feature_edges, patches) =
        build_edges_and_patches(&mut triangles, &vertices, config);
    let (bounds_min, bounds_max) = bounds_from_positions(positions);
    let metadata = SurfaceMeshMetadata {
        surface_mesh_schema_version: SDF_SURFACE_MESH_SCHEMA_VERSION.to_string(),
        query_schema_version: SDF_CFD_QUERY_SCHEMA_VERSION.to_string(),
        extractor_version: extractor_version.into(),
        geometry_hash: geometry_hash(sdf.metadata()),
        configuration_hash: config.config_hash(),
        mesh_hash_excluding_metadata: String::new(),
        bounds_min: array_from_vec3(bounds_min),
        bounds_max: array_from_vec3(bounds_max),
        resolution: config.resolution_override.unwrap_or(0),
        target_spacing: config.target_spacing,
        cleanup,
    };
    let mut mesh = SdfSurfaceMesh {
        schema_version: SDF_SURFACE_MESH_SCHEMA_VERSION.to_string(),
        vertices,
        triangles,
        edges,
        patches,
        feature_edges,
        metadata,
    };
    mesh.metadata.mesh_hash_excluding_metadata = mesh_content_hash(&mesh);
    let mut audit = audit_surface_mesh(&mesh);
    audit.determinism.mesh_hash = mesh.mesh_hash();
    Ok((mesh, audit, query_counts))
}

/// Stock coordinate/index cleanup only. No SDF queries or acceptance authority.
pub fn weld_and_clean_seed_mesh(raw_mesh: &Mesh, config: &SurfaceExtractionConfig) -> SeedSurface {
    let mut cleanup = CleanupSummary {
        raw_vertices: raw_mesh.vertices.len(),
        raw_triangles: raw_mesh.indices.len() / 3,
        ..Default::default()
    };
    let mut key_to_id: BTreeMap<(i64, i64, i64), u64> = BTreeMap::new();
    let mut vertices = Vec::<Vec3>::new();
    let mut triangles = Vec::<[u64; 3]>::new();
    let mut seen_triangles = BTreeSet::<[u64; 3]>::new();
    let tolerance = config.weld_tolerance.max(1.0e-9) as f32;
    let min_area = (config.min_spacing * config.min_spacing * 1.0e-10).max(1.0e-16);

    for tri in raw_mesh.indices.chunks_exact(3) {
        let mut ids = [0_u64; 3];
        let mut valid = true;
        for (slot, raw_index) in tri.iter().copied().enumerate() {
            let Some(vertex) = raw_mesh.vertices.get(raw_index as usize) else {
                valid = false;
                break;
            };
            let p = Vec3::from_array(vertex.position);
            if !p.is_finite() {
                valid = false;
                break;
            }
            let key = quantized_key(p, tolerance);
            let id = if let Some(id) = key_to_id.get(&key).copied() {
                id
            } else {
                let id = vertices.len() as u64;
                key_to_id.insert(key, id);
                vertices.push(p);
                id
            };
            ids[slot] = id;
        }
        if !valid || ids[0] == ids[1] || ids[1] == ids[2] || ids[2] == ids[0] {
            cleanup.degenerate_triangles_removed += 1;
            continue;
        }
        let p0 = vertices[ids[0] as usize];
        let p1 = vertices[ids[1] as usize];
        let p2 = vertices[ids[2] as usize];
        let area = 0.5 * (p1 - p0).cross(p2 - p0).length() as f64;
        if area <= min_area || !area.is_finite() {
            cleanup.zero_area_triangles_removed += 1;
            continue;
        }
        let mut canonical = ids;
        canonical.sort_unstable();
        if !seen_triangles.insert(canonical) {
            cleanup.duplicate_triangles_removed += 1;
            continue;
        }
        triangles.push(ids);
    }
    cleanup.welded_vertices_removed = cleanup.raw_vertices.saturating_sub(vertices.len());
    SeedSurface {
        vertices,
        triangles,
        cleanup,
    }
}

fn project_seed_vertices(
    kernel: &GeometryQueryKernel,
    seed_vertices: &[Vec3],
    config: &SurfaceExtractionConfig,
) -> Result<(Vec<SurfaceVertex>, SurfaceQueryCounts), String> {
    let mut vertices = Vec::with_capacity(seed_vertices.len());
    let mut query_counts = SurfaceQueryCounts::default();
    let options = SdfCfdQueryOptions {
        distance: true,
        gradient: true,
        projection: true,
        provenance: true,
        hard_csg_provenance: CsgTraceMode::Recursive,
        feature_size: true,
        ..SdfCfdQueryOptions::distance_only()
    };
    let final_options = SdfCfdQueryOptions {
        distance: true,
        gradient: true,
        projection: false,
        provenance: true,
        hard_csg_provenance: CsgTraceMode::Recursive,
        feature_size: true,
        ..SdfCfdQueryOptions::distance_only()
    };
    for (id, point) in seed_vertices.iter().copied().enumerate() {
        let query = kernel.query_point(array_from_vec3(point), options);
        query_counts.distance_queries += 1;
        query_counts.gradient_queries += 1;
        query_counts.projection_queries += 1;
        query_counts.provenance_queries += 1;
        let pre_distance = query.signed_distance.unwrap_or(f64::NAN);
        let Some(projection) = query.projection.clone() else {
            return Err(format!("projection result missing for vertex {id}"));
        };
        if !projection.converged {
            return Err(format!(
                "projection failed for vertex {id}: {}",
                projection
                    .failure_reason
                    .as_deref()
                    .unwrap_or("unknown projection failure")
            ));
        }
        let displacement = projection.displacement_magnitude.unwrap_or(f64::INFINITY);
        if displacement > config.max_projection_displacement_factor * config.target_spacing {
            return Err(format!(
                "projection displacement {displacement:.6} exceeded limit for vertex {id}"
            ));
        }
        let projected = projection
            .projected_point
            .ok_or_else(|| format!("projection produced no point for vertex {id}"))?;
        let final_query = kernel.query_point(projected, final_options);
        query_counts.distance_queries += 1;
        query_counts.gradient_queries += 1;
        query_counts.provenance_queries += 1;
        let gradient_magnitude = final_query.gradient_magnitude;
        let unit_normal = final_query.unit_normal;
        let provenance_status = final_query
            .hard_csg_trace
            .as_ref()
            .map(trace_status_to_query_status)
            .unwrap_or(QueryValueStatus::Unsupported);
        let smooth_csg_unsupported = final_query.hard_csg_trace.as_ref().is_some_and(|trace| {
            trace
                .termination_reason
                .as_deref()
                .unwrap_or_default()
                .contains("unsupported_smooth_csg")
        });
        vertices.push(SurfaceVertex {
            id: id as u64,
            position: projected,
            signed_distance_residual: final_query.signed_distance.unwrap_or(f64::NAN),
            raw_gradient: final_query.raw_gradient,
            unit_normal,
            projection_status: ProjectionStatus {
                pre_projection_signed_distance: pre_distance,
                projected_point: Some(projected),
                residual_signed_distance: final_query.signed_distance,
                iteration_count: projection.iteration_count,
                converged: projection.converged,
                displacement_magnitude: Some(displacement),
                final_gradient_magnitude: gradient_magnitude,
                failure_reason: projection.failure_reason,
                method: projection.method,
            },
            provenance: final_query.hard_csg_trace,
            provenance_status,
            patch_id: Some(0),
            feature_flags: SurfaceFeatureFlags {
                smooth_csg_unsupported,
                uncertain: unit_normal.is_none()
                    || gradient_magnitude.unwrap_or(0.0) <= config.min_gradient_magnitude,
                ..Default::default()
            },
            local_target_size: final_query
                .local_feature_size
                .or(Some(config.target_spacing)),
        });
    }
    Ok((vertices, query_counts))
}

fn build_oriented_triangles(
    vertices: &[SurfaceVertex],
    seed_triangles: &[[u64; 3]],
    config: &SurfaceExtractionConfig,
    cleanup: &mut CleanupSummary,
) -> Vec<SurfaceTriangle> {
    let mut triangles = Vec::with_capacity(seed_triangles.len());
    let mut seen = BTreeSet::<[u64; 3]>::new();
    let min_area = (config.min_spacing * config.min_spacing * 1.0e-10).max(1.0e-16);
    for seed in seed_triangles {
        let mut ids = *seed;
        let p0 = position(vertices, ids[0]);
        let p1 = position(vertices, ids[1]);
        let p2 = position(vertices, ids[2]);
        let mut normal = (p1 - p0).cross(p2 - p0);
        let area = 0.5 * normal.length() as f64;
        if area <= min_area || !area.is_finite() {
            cleanup.degenerate_triangles_removed += 1;
            continue;
        }
        normal = normal.normalize_or_zero();
        let sdf_normal = average_sdf_normal(vertices, ids);
        if sdf_normal.length_squared() > 0.0 && normal.dot(sdf_normal) < 0.0 {
            ids.swap(1, 2);
            normal = -normal;
            cleanup.orientation_flips += 1;
        }
        let mut canonical = ids;
        canonical.sort_unstable();
        if !seen.insert(canonical) {
            cleanup.duplicate_triangles_removed += 1;
            continue;
        }
        let quality = triangle_quality(vertices, ids, normal, config.target_spacing);
        triangles.push(SurfaceTriangle {
            id: triangles.len() as u64,
            vertices: ids,
            geometric_normal: array_from_vec3(normal),
            area,
            patch_id: Some(0),
            provenance_summary: triangle_provenance_summary(vertices, ids),
            quality,
        });
    }
    triangles
}

fn build_edges_and_patches(
    triangles: &mut [SurfaceTriangle],
    vertices: &[SurfaceVertex],
    config: &SurfaceExtractionConfig,
) -> (Vec<SurfaceEdge>, Vec<FeatureEdge>, Vec<SurfacePatch>) {
    let mut edge_map: BTreeMap<(u64, u64), Vec<u64>> = BTreeMap::new();
    for triangle in triangles.iter() {
        for (a, b) in oriented_edges(triangle.vertices) {
            let key = sorted_edge(a, b);
            edge_map.entry(key).or_default().push(triangle.id);
        }
    }
    let tri_by_id: BTreeMap<u64, usize> = triangles
        .iter()
        .enumerate()
        .map(|(idx, tri)| (tri.id, idx))
        .collect();
    let mut edges = Vec::with_capacity(edge_map.len());
    let mut feature_edges = Vec::new();
    let sharp_cos = (config.sharp_dihedral_degrees.to_radians()).cos();
    for ((a, b), incident) in edge_map {
        let mut classification = match incident.len() {
            0 => SurfaceEdgeClassification::Uncertain,
            1 => SurfaceEdgeClassification::Boundary,
            2 => SurfaceEdgeClassification::ManifoldInterior,
            _ => SurfaceEdgeClassification::NonManifold,
        };
        if incident.len() == 2 {
            let t0 = &triangles[tri_by_id[&incident[0]]];
            let t1 = &triangles[tri_by_id[&incident[1]]];
            let n0 = Vec3::from_array([
                t0.geometric_normal[0] as f32,
                t0.geometric_normal[1] as f32,
                t0.geometric_normal[2] as f32,
            ]);
            let n1 = Vec3::from_array([
                t1.geometric_normal[0] as f32,
                t1.geometric_normal[1] as f32,
                t1.geometric_normal[2] as f32,
            ]);
            if t0.provenance_summary != t1.provenance_summary {
                classification = SurfaceEdgeClassification::CandidateProvenanceBoundary;
            } else if n0.dot(n1) < sharp_cos as f32 {
                classification = SurfaceEdgeClassification::CandidateSharp;
            }
        }
        let id = edges.len() as u64;
        if matches!(
            classification,
            SurfaceEdgeClassification::CandidateProvenanceBoundary
                | SurfaceEdgeClassification::CandidateSharp
                | SurfaceEdgeClassification::CandidatePatchBoundary
        ) {
            let source = match classification {
                SurfaceEdgeClassification::CandidateProvenanceBoundary => {
                    FeatureEdgeSource::ProvenanceBoundaryCandidate
                }
                SurfaceEdgeClassification::CandidatePatchBoundary => {
                    FeatureEdgeSource::PatchBoundaryCandidate
                }
                _ => FeatureEdgeSource::DihedralCandidate,
            };
            feature_edges.push(FeatureEdge {
                id: feature_edges.len() as u64,
                edge_id: id,
                source,
                support_status: QueryValueStatus::Heuristic,
            });
        }
        if matches!(
            classification,
            SurfaceEdgeClassification::CandidateProvenanceBoundary
        ) {
            for tri_id in &incident {
                if let Some(idx) = tri_by_id.get(tri_id).copied() {
                    let tri = &mut triangles[idx];
                    tri.provenance_summary = match tri.provenance_summary {
                        TriangleProvenanceSummary::Consistent { .. } => {
                            TriangleProvenanceSummary::NearHardCsgBoundary
                        }
                        ref other => other.clone(),
                    };
                }
            }
        }
        let _ = vertices;
        edges.push(SurfaceEdge {
            id,
            vertices: [a, b],
            incident_triangles: incident,
            classification,
        });
    }
    let patches = vec![SurfacePatch {
        id: 0,
        name: "wall".to_string(),
        source: PatchSource::GenericWall,
        triangle_ids: triangles.iter().map(|tri| tri.id).collect(),
        support_status: QueryValueStatus::Partial,
    }];
    (edges, feature_edges, patches)
}

pub fn audit_surface_mesh(mesh: &SdfSurfaceMesh) -> SurfaceMeshAudit {
    let geometry = geometry_audit(mesh);
    let topology = topology_audit(mesh);
    let triangle_quality = triangle_quality_audit(mesh);
    let provenance = provenance_audit(mesh);
    SurfaceMeshAudit {
        geometry,
        topology,
        triangle_quality,
        provenance,
        determinism: DeterminismAudit {
            repeated: false,
            mesh_hash: mesh.mesh_hash(),
            repeated_mesh_hash: None,
            exact_match: None,
        },
    }
}

pub fn run_determinism_check(
    sdf: Arc<dyn Sdf>,
    config: SurfaceExtractionConfig,
) -> Result<(String, String, bool), String> {
    let first = extract_sdf_surface_mesh(Arc::clone(&sdf), config.clone())?;
    let second = extract_sdf_surface_mesh(sdf, config)?;
    let first_hash = first.mesh.mesh_hash();
    let second_hash = second.mesh.mesh_hash();
    Ok((
        first_hash.clone(),
        second_hash.clone(),
        first_hash == second_hash,
    ))
}

fn geometry_audit(mesh: &SdfSurfaceMesh) -> GeometryAudit {
    let mut residuals: Vec<f64> = mesh
        .vertices
        .iter()
        .map(|v| v.signed_distance_residual.abs())
        .filter(|v| v.is_finite())
        .collect();
    residuals.sort_by(f64::total_cmp);
    let max_abs = residuals.last().copied().unwrap_or(0.0);
    let mean_abs = if residuals.is_empty() {
        0.0
    } else {
        residuals.iter().sum::<f64>() / residuals.len() as f64
    };
    let p95 = percentile_sorted(&residuals, 0.95);
    let max_projection_displacement = mesh
        .vertices
        .iter()
        .filter_map(|v| v.projection_status.displacement_magnitude)
        .fold(0.0, f64::max);
    let projection_failures = mesh
        .vertices
        .iter()
        .filter(|v| !v.projection_status.converged)
        .count();
    let invalid_normals = mesh
        .vertices
        .iter()
        .filter(|v| v.unit_normal.is_none())
        .count();
    let non_finite_distance_or_gradient = mesh
        .vertices
        .iter()
        .filter(|v| {
            !v.signed_distance_residual.is_finite()
                || v.raw_gradient
                    .is_some_and(|g| g.iter().any(|value| !value.is_finite()))
        })
        .count();
    let mut normal_angles = Vec::new();
    for tri in &mesh.triangles {
        if let Some(angle) = tri.quality.normal_angle_to_sdf_degrees {
            normal_angles.push(angle);
        }
    }
    let max_angle = normal_angles.iter().copied().fold(0.0, f64::max);
    let mean_angle = if normal_angles.is_empty() {
        0.0
    } else {
        normal_angles.iter().sum::<f64>() / normal_angles.len() as f64
    };
    let (bounds_min, bounds_max) = surface_bounds(mesh);
    GeometryAudit {
        max_abs_sdf_residual: max_abs,
        mean_abs_sdf_residual: mean_abs,
        p95_abs_sdf_residual: p95,
        max_projection_displacement,
        projection_failures,
        non_finite_distance_or_gradient,
        invalid_normals,
        max_sdf_normal_to_triangle_normal_degrees: max_angle,
        mean_sdf_normal_to_triangle_normal_degrees: mean_angle,
        bounds_min,
        bounds_max,
        signed_volume: signed_volume(mesh),
        orientation_consistent: max_angle <= 90.0,
    }
}

fn topology_audit(mesh: &SdfSurfaceMesh) -> TopologyAudit {
    let boundary_edges = mesh
        .edges
        .iter()
        .filter(|e| e.classification == SurfaceEdgeClassification::Boundary)
        .count();
    let non_manifold_edges = mesh
        .edges
        .iter()
        .filter(|e| e.classification == SurfaceEdgeClassification::NonManifold)
        .count();
    let edge_uses = mesh
        .edges
        .iter()
        .map(|e| e.incident_triangles.len())
        .filter(|count| *count != 2)
        .count();
    let isolated_vertices = isolated_vertex_count(mesh);
    let connected_components = connected_component_count(mesh);
    TopologyAudit {
        vertex_count: mesh.vertices.len(),
        edge_count: mesh.edges.len(),
        triangle_count: mesh.triangles.len(),
        connected_components,
        boundary_edges,
        non_manifold_edges,
        duplicate_vertices: 0,
        duplicate_triangles: mesh.metadata.cleanup.duplicate_triangles_removed,
        isolated_vertices,
        inconsistent_edge_orientation: edge_uses
            .saturating_sub(boundary_edges + non_manifold_edges),
        zero_area_triangles: mesh.metadata.cleanup.zero_area_triangles_removed,
        tiny_disconnected_components: 0,
        euler_characteristic: mesh.vertices.len() as i64 - mesh.edges.len() as i64
            + mesh.triangles.len() as i64,
        closed_manifold: boundary_edges == 0
            && non_manifold_edges == 0
            && isolated_vertices == 0
            && connected_components >= 1,
    }
}

fn triangle_quality_audit(mesh: &SdfSurfaceMesh) -> TriangleQualityAudit {
    if mesh.triangles.is_empty() {
        return TriangleQualityAudit::default();
    }
    let mut min_area = f64::INFINITY;
    let mut max_area: f64 = 0.0;
    let mut sum_area = 0.0;
    let mut min_edge = f64::INFINITY;
    let mut max_edge: f64 = 0.0;
    let mut min_angle = f64::INFINITY;
    let mut max_angle: f64 = 0.0;
    let mut max_aspect: f64 = 0.0;
    let mut sum_aspect = 0.0;
    let mut min_radius_ratio = f64::INFINITY;
    for tri in &mesh.triangles {
        min_area = min_area.min(tri.area);
        max_area = max_area.max(tri.area);
        sum_area += tri.area;
        min_edge = min_edge.min(tri.quality.min_edge_length);
        max_edge = max_edge.max(tri.quality.max_edge_length);
        min_angle = min_angle.min(tri.quality.min_angle_degrees);
        max_angle = max_angle.max(tri.quality.max_angle_degrees);
        max_aspect = max_aspect.max(tri.quality.aspect_ratio);
        sum_aspect += tri.quality.aspect_ratio;
        min_radius_ratio = min_radius_ratio.min(tri.quality.radius_ratio);
    }
    TriangleQualityAudit {
        min_area,
        max_area,
        mean_area: sum_area / mesh.triangles.len() as f64,
        min_edge_length: min_edge,
        max_edge_length: max_edge,
        min_angle_degrees: min_angle,
        max_angle_degrees: max_angle,
        max_aspect_ratio: max_aspect,
        mean_aspect_ratio: sum_aspect / mesh.triangles.len() as f64,
        min_radius_ratio,
        max_neighbor_size_ratio: max_neighbor_size_ratio(mesh),
        max_normal_angle_jump_degrees: max_normal_angle_jump(mesh),
    }
}

fn provenance_audit(mesh: &SdfSurfaceMesh) -> SurfaceProvenanceAudit {
    let mut audit = SurfaceProvenanceAudit::default();
    for vertex in &mesh.vertices {
        if let Some(trace) = &vertex.provenance {
            if trace.complete {
                audit.vertices_with_complete_hard_csg_traces += 1;
            } else {
                audit.vertices_with_incomplete_traces += 1;
            }
            if trace
                .termination_reason
                .as_deref()
                .unwrap_or_default()
                .contains("unsupported_smooth_csg")
            {
                audit.smooth_csg_termination_count += 1;
            }
            for decision in &trace.decisions {
                match decision.decision_state {
                    crate::sdf::provenance::CsgDecisionState::NearTie => audit.near_tie_count += 1,
                    crate::sdf::provenance::CsgDecisionState::ExactTie => {
                        audit.exact_tie_count += 1
                    }
                    _ => {}
                }
            }
        } else {
            audit.vertices_with_incomplete_traces += 1;
        }
    }
    for tri in &mesh.triangles {
        match tri.provenance_summary {
            TriangleProvenanceSummary::Consistent { .. } => {
                audit.triangles_with_consistent_provenance += 1
            }
            TriangleProvenanceSummary::MixedHardCsgProvenance
            | TriangleProvenanceSummary::NearHardCsgBoundary => {
                audit.triangles_with_mixed_provenance += 1
            }
            _ => {}
        }
        if triangle_has_role(mesh, tri, CsgChildRole::Subtractor) {
            audit.subtractor_controlled_area += tri.area;
        }
        if triangle_has_role(mesh, tri, CsgChildRole::Retained) {
            audit.retained_controlled_area += tri.area;
        }
    }
    audit.candidate_provenance_boundary_edges = mesh
        .edges
        .iter()
        .filter(|edge| {
            edge.classification == SurfaceEdgeClassification::CandidateProvenanceBoundary
        })
        .count();
    audit
}

pub fn write_ply(mesh: &SdfSurfaceMesh, path: impl AsRef<std::path::Path>) -> std::io::Result<()> {
    use std::io::Write;
    let mut file = std::fs::File::create(path)?;
    writeln!(file, "ply")?;
    writeln!(file, "format ascii 1.0")?;
    writeln!(
        file,
        "comment visualization artifact only; SDF remains geometry authority"
    )?;
    writeln!(file, "element vertex {}", mesh.vertices.len())?;
    writeln!(file, "property double x")?;
    writeln!(file, "property double y")?;
    writeln!(file, "property double z")?;
    writeln!(file, "element face {}", mesh.triangles.len())?;
    writeln!(file, "property list uchar int vertex_indices")?;
    writeln!(file, "end_header")?;
    for vertex in &mesh.vertices {
        writeln!(
            file,
            "{:.9} {:.9} {:.9}",
            vertex.position[0], vertex.position[1], vertex.position[2]
        )?;
    }
    for tri in &mesh.triangles {
        writeln!(
            file,
            "3 {} {} {}",
            tri.vertices[0], tri.vertices[1], tri.vertices[2]
        )?;
    }
    Ok(())
}

fn triangle_has_role(mesh: &SdfSurfaceMesh, tri: &SurfaceTriangle, role: CsgChildRole) -> bool {
    tri.vertices.iter().all(|id| {
        mesh.vertices[*id as usize]
            .provenance
            .as_ref()
            .and_then(|trace| trace.decisions.last())
            .is_some_and(|decision| decision.active.child_role == role)
    })
}

fn triangle_provenance_summary(
    vertices: &[SurfaceVertex],
    ids: [u64; 3],
) -> TriangleProvenanceSummary {
    let mut keys = BTreeSet::new();
    let mut unsupported_smooth = false;
    let mut incomplete = false;
    let mut support = CsgTraceSupport::Partial;
    for id in ids {
        let Some(trace) = vertices[id as usize].provenance.as_ref() else {
            incomplete = true;
            continue;
        };
        support = trace.support_status;
        if !trace.complete {
            incomplete = true;
        }
        if trace
            .termination_reason
            .as_deref()
            .unwrap_or_default()
            .contains("unsupported_smooth_csg")
        {
            unsupported_smooth = true;
        }
        keys.insert(provenance_key(trace));
    }
    if unsupported_smooth {
        TriangleProvenanceSummary::UnsupportedSmoothCsgRegion
    } else if incomplete {
        TriangleProvenanceSummary::IncompleteProvenance
    } else if keys.len() == 1 {
        TriangleProvenanceSummary::Consistent {
            key: keys.into_iter().next().unwrap_or_default(),
            support_status: support,
        }
    } else if keys.len() > 1 {
        TriangleProvenanceSummary::MixedHardCsgProvenance
    } else {
        TriangleProvenanceSummary::Unknown
    }
}

fn provenance_key(trace: &CsgProvenanceTrace) -> String {
    let terminal = trace
        .terminal_entity
        .as_ref()
        .and_then(|entity| entity.stable_id.clone().or_else(|| entity.label.clone()))
        .unwrap_or_else(|| "unknown_terminal".to_string());
    let role = trace
        .decisions
        .last()
        .map(|decision| format!("{:?}", decision.active.child_role))
        .unwrap_or_else(|| "terminal".to_string());
    format!("{terminal}:{role}")
}

fn trace_status_to_query_status(trace: &CsgProvenanceTrace) -> QueryValueStatus {
    match trace.support_status {
        CsgTraceSupport::Supported => QueryValueStatus::Native,
        CsgTraceSupport::Partial => QueryValueStatus::Partial,
        CsgTraceSupport::Unsupported => QueryValueStatus::Unsupported,
        CsgTraceSupport::Failed => QueryValueStatus::Failed,
        CsgTraceSupport::NotRequested => QueryValueStatus::NotRequested,
    }
}

fn triangle_quality(
    vertices: &[SurfaceVertex],
    ids: [u64; 3],
    geometric_normal: Vec3,
    target_spacing: f64,
) -> TriangleQuality {
    let p = ids.map(|id| position(vertices, id));
    let e = [
        p[0].distance(p[1]) as f64,
        p[1].distance(p[2]) as f64,
        p[2].distance(p[0]) as f64,
    ];
    let area = 0.5 * (p[1] - p[0]).cross(p[2] - p[0]).length() as f64;
    let angles = triangle_angles(e);
    let min_edge = e.iter().copied().fold(f64::INFINITY, f64::min);
    let max_edge = e.iter().copied().fold(0.0, f64::max);
    let altitude = if max_edge > 0.0 {
        (2.0 * area) / max_edge
    } else {
        0.0
    };
    let aspect_ratio = if altitude > 0.0 {
        max_edge / altitude
    } else {
        f64::INFINITY
    };
    let semiperimeter = 0.5 * (e[0] + e[1] + e[2]);
    let inradius = if semiperimeter > 0.0 {
        area / semiperimeter
    } else {
        0.0
    };
    let circumradius = if area > 0.0 {
        e[0] * e[1] * e[2] / (4.0 * area)
    } else {
        f64::INFINITY
    };
    let sdf_normal = average_sdf_normal(vertices, ids);
    let normal_angle_to_sdf = if sdf_normal.length_squared() > 0.0 {
        Some(angle_degrees(geometric_normal, sdf_normal))
    } else {
        None
    };
    TriangleQuality {
        min_edge_length: min_edge,
        max_edge_length: max_edge,
        min_angle_degrees: angles.iter().copied().fold(f64::INFINITY, f64::min),
        max_angle_degrees: angles.iter().copied().fold(0.0, f64::max),
        aspect_ratio,
        radius_ratio: if circumradius.is_finite() && circumradius > 0.0 {
            2.0 * inradius / circumradius
        } else {
            0.0
        },
        local_size_error: Some((max_edge - target_spacing).abs() / target_spacing.max(1.0e-9)),
        normal_angle_to_sdf_degrees: normal_angle_to_sdf,
    }
}

fn triangle_angles(edge_lengths: [f64; 3]) -> [f64; 3] {
    let [a, b, c] = edge_lengths;
    [
        law_of_cosines_angle(b, c, a),
        law_of_cosines_angle(c, a, b),
        law_of_cosines_angle(a, b, c),
    ]
}

fn law_of_cosines_angle(a: f64, b: f64, opposite: f64) -> f64 {
    if a <= 0.0 || b <= 0.0 {
        return 0.0;
    }
    let cos = ((a * a + b * b - opposite * opposite) / (2.0 * a * b)).clamp(-1.0, 1.0);
    cos.acos().to_degrees()
}

fn position(vertices: &[SurfaceVertex], id: u64) -> Vec3 {
    let p = vertices[id as usize].position;
    Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)
}

fn average_sdf_normal(vertices: &[SurfaceVertex], ids: [u64; 3]) -> Vec3 {
    let mut normal = Vec3::ZERO;
    for id in ids {
        if let Some(n) = vertices[id as usize].unit_normal {
            normal += Vec3::new(n[0] as f32, n[1] as f32, n[2] as f32);
        }
    }
    normal.normalize_or_zero()
}

fn quantized_key(p: Vec3, tolerance: f32) -> (i64, i64, i64) {
    (
        (p.x / tolerance).round() as i64,
        (p.y / tolerance).round() as i64,
        (p.z / tolerance).round() as i64,
    )
}

fn expand_degenerate_bounds(mut min: Vec3, mut max: Vec3, half_width: f32) -> (Vec3, Vec3) {
    for axis in 0..3 {
        if max[axis] - min[axis] <= 0.0 {
            let center = (max[axis] + min[axis]) * 0.5;
            min[axis] = center - half_width;
            max[axis] = center + half_width;
        }
    }
    (min, max)
}

fn extraction_bounding_box(sdf: &dyn Sdf) -> crate::sdf::query::BoundingInfo {
    let metadata = sdf.metadata();
    if let Some(bounds) = metadata.support_bounds {
        if bounds.is_valid() && bounds.min.is_finite() && bounds.max.is_finite() {
            let min = bounds.min;
            let max = bounds.max;
            let center = (min + max) * 0.5;
            let size = max - min;
            let corners = [
                Vec3::new(min.x, min.y, min.z),
                Vec3::new(max.x, min.y, min.z),
                Vec3::new(min.x, max.y, min.z),
                Vec3::new(max.x, max.y, min.z),
                Vec3::new(min.x, min.y, max.z),
                Vec3::new(max.x, min.y, max.z),
                Vec3::new(min.x, max.y, max.z),
                Vec3::new(max.x, max.y, max.z),
            ];
            return crate::sdf::query::BoundingInfo {
                min,
                max,
                center,
                size,
                corners,
            };
        }
    }
    bounding_points(sdf)
}

fn sorted_edge(a: u64, b: u64) -> (u64, u64) {
    if a < b { (a, b) } else { (b, a) }
}

fn oriented_edges(ids: [u64; 3]) -> [(u64, u64); 3] {
    [(ids[0], ids[1]), (ids[1], ids[2]), (ids[2], ids[0])]
}

fn connected_component_count(mesh: &SdfSurfaceMesh) -> usize {
    if mesh.triangles.is_empty() {
        return 0;
    }
    let mut tri_adj = vec![Vec::<usize>::new(); mesh.triangles.len()];
    for edge in &mesh.edges {
        for &a in &edge.incident_triangles {
            for &b in &edge.incident_triangles {
                if a != b {
                    tri_adj[a as usize].push(b as usize);
                }
            }
        }
    }
    let mut visited = vec![false; mesh.triangles.len()];
    let mut components = 0;
    for start in 0..mesh.triangles.len() {
        if visited[start] {
            continue;
        }
        components += 1;
        let mut queue = VecDeque::from([start]);
        visited[start] = true;
        while let Some(cur) = queue.pop_front() {
            for &next in &tri_adj[cur] {
                if !visited[next] {
                    visited[next] = true;
                    queue.push_back(next);
                }
            }
        }
    }
    components
}

fn isolated_vertex_count(mesh: &SdfSurfaceMesh) -> usize {
    let mut used = vec![false; mesh.vertices.len()];
    for tri in &mesh.triangles {
        for id in tri.vertices {
            used[id as usize] = true;
        }
    }
    used.iter().filter(|used| !**used).count()
}

fn surface_bounds(mesh: &SdfSurfaceMesh) -> ([f64; 3], [f64; 3]) {
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for vertex in &mesh.vertices {
        for axis in 0..3 {
            min[axis] = min[axis].min(vertex.position[axis]);
            max[axis] = max[axis].max(vertex.position[axis]);
        }
    }
    (min, max)
}

fn bounds_from_positions(positions: &[Vec3]) -> (Vec3, Vec3) {
    let mut min = Vec3::splat(f32::INFINITY);
    let mut max = Vec3::splat(f32::NEG_INFINITY);
    for p in positions {
        min = min.min(*p);
        max = max.max(*p);
    }
    if positions.is_empty() {
        (Vec3::ZERO, Vec3::ZERO)
    } else {
        (min, max)
    }
}

fn signed_volume(mesh: &SdfSurfaceMesh) -> f64 {
    let mut volume = 0.0;
    for tri in &mesh.triangles {
        let a = position(&mesh.vertices, tri.vertices[0]);
        let b = position(&mesh.vertices, tri.vertices[1]);
        let c = position(&mesh.vertices, tri.vertices[2]);
        volume += a.dot(b.cross(c)) as f64;
    }
    volume / 6.0
}

fn max_neighbor_size_ratio(mesh: &SdfSurfaceMesh) -> f64 {
    let tri_by_id: BTreeMap<u64, &SurfaceTriangle> =
        mesh.triangles.iter().map(|tri| (tri.id, tri)).collect();
    let mut max_ratio: f64 = 1.0;
    for edge in &mesh.edges {
        if edge.incident_triangles.len() != 2 {
            continue;
        }
        let a = tri_by_id[&edge.incident_triangles[0]]
            .quality
            .max_edge_length;
        let b = tri_by_id[&edge.incident_triangles[1]]
            .quality
            .max_edge_length;
        let min = a.min(b).max(1.0e-12);
        max_ratio = max_ratio.max(a.max(b) / min);
    }
    max_ratio
}

fn max_normal_angle_jump(mesh: &SdfSurfaceMesh) -> f64 {
    let tri_by_id: BTreeMap<u64, &SurfaceTriangle> =
        mesh.triangles.iter().map(|tri| (tri.id, tri)).collect();
    let mut max_angle: f64 = 0.0;
    for edge in &mesh.edges {
        if edge.incident_triangles.len() != 2 {
            continue;
        }
        let a = Vec3::from_array(
            tri_by_id[&edge.incident_triangles[0]]
                .geometric_normal
                .map(|v| v as f32),
        );
        let b = Vec3::from_array(
            tri_by_id[&edge.incident_triangles[1]]
                .geometric_normal
                .map(|v| v as f32),
        );
        max_angle = max_angle.max(angle_degrees(a, b));
    }
    max_angle
}

fn angle_degrees(a: Vec3, b: Vec3) -> f64 {
    let dot = a
        .normalize_or_zero()
        .dot(b.normalize_or_zero())
        .clamp(-1.0, 1.0);
    dot.acos().to_degrees() as f64
}

fn percentile_sorted(values: &[f64], percentile: f64) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let index = ((values.len().saturating_sub(1)) as f64 * percentile).round() as usize;
    values[index.min(values.len() - 1)]
}

fn array_from_vec3(v: Vec3) -> [f64; 3] {
    [v.x as f64, v.y as f64, v.z as f64]
}

fn stable_hash_hex(bytes: &[u8]) -> String {
    let mut hash = 0xcbf29ce484222325_u64;
    for byte in bytes {
        hash ^= *byte as u64;
        hash = hash.wrapping_mul(0x100000001b3);
    }
    format!("{hash:016x}")
}

fn mesh_content_hash(mesh: &SdfSurfaceMesh) -> String {
    let content = serde_json::json!({
        "vertices": mesh.vertices,
        "triangles": mesh.triangles,
        "edges": mesh.edges,
        "patches": mesh.patches,
        "feature_edges": mesh.feature_edges,
        "schema_version": mesh.schema_version,
    });
    stable_hash_hex(&serde_json::to_vec(&content).unwrap_or_default())
}

fn geometry_hash(metadata: crate::sdf::conditioning::SdfNodeMetadata) -> String {
    stable_hash_hex(format!("{metadata:?}").as_bytes())
}

fn elapsed_ms(start: Instant) -> f64 {
    start.elapsed().as_secs_f64() * 1000.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::booleans::Subtract;
    use crate::sdf::primitives::Sphere;
    use crate::sdf::transforms::Translate;

    #[test]
    fn sphere_surface_is_closed_and_deterministic() {
        let sdf: Arc<dyn Sdf> = Arc::new(Sphere::new(10.0));
        let config = SurfaceExtractionConfig::default()
            .with_target_spacing(2.0)
            .with_resolution(18);
        let first = extract_sdf_surface_mesh(Arc::clone(&sdf), config.clone()).unwrap();
        let second = extract_sdf_surface_mesh(sdf, config).unwrap();
        assert_eq!(first.mesh.mesh_hash(), second.mesh.mesh_hash());
        assert_eq!(first.audit.topology.boundary_edges, 0);
        assert_eq!(first.audit.topology.non_manifold_edges, 0);
        assert!(first.audit.geometry.max_abs_sdf_residual <= 1.0e-3);
    }

    #[test]
    fn subtraction_surface_has_subtractor_controlled_area() {
        let body: Arc<dyn Sdf> = Arc::new(Sphere::new(12.0));
        let tool: Arc<dyn Sdf> = Arc::new(Translate::new(
            Arc::new(Sphere::new(4.0)),
            Vec3::new(3.0, 0.0, 0.0),
        ));
        let sdf: Arc<dyn Sdf> = Arc::new(Subtract::new(body, tool));
        let config = SurfaceExtractionConfig::default()
            .with_target_spacing(2.0)
            .with_resolution(22);
        let outcome = extract_sdf_surface_mesh(sdf, config).unwrap();
        assert_eq!(outcome.audit.topology.boundary_edges, 0);
        assert!(outcome.audit.provenance.subtractor_controlled_area > 0.0);
        assert!(outcome.audit.provenance.retained_controlled_area > 0.0);
    }

    #[test]
    fn single_triangle_edge_adjacency_reports_boundary() {
        let mesh = SdfSurfaceMesh {
            schema_version: SDF_SURFACE_MESH_SCHEMA_VERSION.to_string(),
            vertices: vec![
                test_vertex(0, [0.0, 0.0, 0.0]),
                test_vertex(1, [1.0, 0.0, 0.0]),
                test_vertex(2, [0.0, 1.0, 0.0]),
            ],
            triangles: vec![SurfaceTriangle {
                id: 0,
                vertices: [0, 1, 2],
                geometric_normal: [0.0, 0.0, 1.0],
                area: 0.5,
                patch_id: Some(0),
                provenance_summary: TriangleProvenanceSummary::Unknown,
                quality: TriangleQuality {
                    min_edge_length: 1.0,
                    max_edge_length: 2.0_f64.sqrt(),
                    min_angle_degrees: 45.0,
                    max_angle_degrees: 90.0,
                    aspect_ratio: 2.0,
                    radius_ratio: 0.5,
                    local_size_error: None,
                    normal_angle_to_sdf_degrees: None,
                },
            }],
            edges: Vec::new(),
            patches: Vec::new(),
            feature_edges: Vec::new(),
            metadata: SurfaceMeshMetadata {
                surface_mesh_schema_version: SDF_SURFACE_MESH_SCHEMA_VERSION.to_string(),
                query_schema_version: SDF_CFD_QUERY_SCHEMA_VERSION.to_string(),
                extractor_version: SDF_SURFACE_EXTRACTOR_VERSION.to_string(),
                geometry_hash: "test".to_string(),
                configuration_hash: "test".to_string(),
                mesh_hash_excluding_metadata: "test".to_string(),
                bounds_min: [0.0, 0.0, 0.0],
                bounds_max: [1.0, 1.0, 0.0],
                resolution: 1,
                target_spacing: 1.0,
                cleanup: CleanupSummary::default(),
            },
        };
        let mut triangles = mesh.triangles.clone();
        let (edges, _, _) = build_edges_and_patches(
            &mut triangles,
            &mesh.vertices,
            &SurfaceExtractionConfig::default(),
        );
        assert_eq!(edges.len(), 3);
        assert!(
            edges
                .iter()
                .all(|edge| edge.classification == SurfaceEdgeClassification::Boundary)
        );
    }

    fn test_vertex(id: u64, position: [f64; 3]) -> SurfaceVertex {
        SurfaceVertex {
            id,
            position,
            signed_distance_residual: 0.0,
            raw_gradient: None,
            unit_normal: None,
            projection_status: ProjectionStatus {
                pre_projection_signed_distance: 0.0,
                projected_point: Some(position),
                residual_signed_distance: Some(0.0),
                iteration_count: None,
                converged: true,
                displacement_magnitude: Some(0.0),
                final_gradient_magnitude: None,
                failure_reason: None,
                method: "test".to_string(),
            },
            provenance: None,
            provenance_status: QueryValueStatus::Unsupported,
            patch_id: Some(0),
            feature_flags: SurfaceFeatureFlags::default(),
            local_target_size: Some(1.0),
        }
    }
}
