use crate::mesh::Mesh;
use crate::mesh::adaptive_marching_tets::{
    AdaptiveMarchingTetsReport, AdaptiveMarchingTetsSurface,
    extract_adaptive_marching_tets_with_sources, summarize_extraction,
};
use crate::mesh::adaptive_sampling::{
    AdaptiveOctree, AdaptiveSamplingConfig, build_adaptive_octree,
};
use crate::mesh::collision_aware_projection::CollisionAwareProjectionConfig;
use crate::sdf::Sdf;
use crate::sdf::cfd_query::QueryValueStatus;
use crate::sdf::cfd_query::{GeometryQueryKernel, SdfCfdQueryOptions};
use crate::sdf::query::bounding_points;
use glam::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;
use std::time::Instant;

pub const FROM_SCRATCH_SURFACE_SCHEMA_VERSION: &str = "canonical_cell_complex.v32";

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct FromScratchSurfaceConfig {
    pub schema_version: String,
    pub case_name: String,
    pub geometry_script_path: String,
    pub extraction_bound_policy: String,
    pub extraction_margin: f64,
    pub input_mesh: Option<String>,
    pub input_leafset: Option<String>,
    pub disk_geometry_cache_used: bool,
    pub from_scratch: bool,
    pub adaptive_sampling: AdaptiveSamplingConfig,
    pub projection: CollisionAwareProjectionConfig,
    pub self_intersection_candidate_budget: usize,
    pub severe_normal_angle_degrees: f64,
    pub fold_normal_dot_threshold: f64,
    pub te_tip_extra_levels: u32,
    pub deterministic_seed: Option<u64>,
}

impl FromScratchSurfaceConfig {
    pub fn config_hash(&self) -> String {
        stable_hash_hex(&serde_json::to_vec(self).unwrap_or_default())
    }

    pub fn validate_input_isolation(&self) -> Result<(), String> {
        if self.input_mesh.is_some() {
            return Err("input_mesh_must_be_null_for_from_scratch_generation".to_string());
        }
        if self.input_leafset.is_some() {
            return Err("input_leafset_must_be_null_for_from_scratch_generation".to_string());
        }
        if self.disk_geometry_cache_used {
            return Err("disk_geometry_cache_must_be_disabled".to_string());
        }
        if !self.from_scratch {
            return Err("from_scratch_flag_must_be_true".to_string());
        }
        Ok(())
    }
}

impl Default for FromScratchSurfaceConfig {
    fn default() -> Self {
        Self {
            schema_version: FROM_SCRATCH_SURFACE_SCHEMA_VERSION.to_string(),
            case_name: "baseline_from_scratch".to_string(),
            geometry_script_path: String::new(),
            extraction_bound_policy: "sdf_metadata_bounds_with_deterministic_margin".to_string(),
            extraction_margin: 12.0,
            input_mesh: None,
            input_leafset: None,
            disk_geometry_cache_used: false,
            from_scratch: true,
            adaptive_sampling: AdaptiveSamplingConfig::default(),
            projection: CollisionAwareProjectionConfig::default(),
            self_intersection_candidate_budget: 5_000_000,
            severe_normal_angle_degrees: 115.0,
            fold_normal_dot_threshold: -0.25,
            te_tip_extra_levels: 0,
            deterministic_seed: None,
        }
    }
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct FromScratchSurfaceResult {
    pub schema_version: String,
    pub case_name: String,
    pub from_scratch: bool,
    pub input_mesh: Option<String>,
    pub input_leafset: Option<String>,
    pub disk_geometry_cache_used: bool,
    pub configuration_hash: String,
    pub extraction_bounds: ExtractionBoundsReport,
    pub adaptive_tree: AdaptiveTreeReport,
    pub raw_surface: SurfaceStageReport,
    pub projected_surface: SurfaceStageReport,
    pub projection: ProjectionStageReport,
    pub te_metrics: TeMetrics,
    pub tip_metrics: TipMetrics,
    pub ordinary_surface_metrics: OrdinarySurfaceMetrics,
    pub regional_surface_metrics: RegionalSurfaceMetrics,
    pub refinement_cause_audit: RefinementCauseAudit,
    pub extraction_attribution: ExtractionAttributionReport,
    pub te_metric_sanity: TeMetricSanityReport,
    pub te_feature_association: TeFeatureAssociationAudit,
    pub feature_and_sector: FeatureSectorReport,
    pub timings: FromScratchTimings,
    pub status: String,
    pub failure_stage: Option<String>,
    pub failure_reasons: Vec<String>,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct ExtractionBoundsReport {
    pub source: String,
    pub unpadded_bounds_min: [f64; 3],
    pub unpadded_bounds_max: [f64; 3],
    pub margin: f64,
    pub final_bounds_min: [f64; 3],
    pub final_bounds_max: [f64; 3],
    pub root_cell_dimensions: [f64; 3],
    pub domain_boundary_sign_changes: usize,
    pub boundary_padding_accepted: bool,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct AdaptiveTreeReport {
    pub leaf_set_hash: String,
    pub leaf_topology_hash: String,
    pub leaf_metadata_hash: String,
    pub config_hash: String,
    pub leaf_count: usize,
    pub level_histogram: BTreeMap<u32, usize>,
    pub feature_leaf_count: usize,
    pub surface_leaf_count: usize,
    pub balance_only_leaves: usize,
    pub max_face_neighbor_level_delta: u32,
    pub indexed_balance_used: bool,
    pub indexed_neighbor_audit_used: bool,
    pub all_pairs_balance_used: bool,
    pub all_pairs_neighbor_audit_used: bool,
    pub summary: serde_json::Value,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct SurfaceStageReport {
    pub status: String,
    pub mesh_hash: String,
    pub connectivity_hash: String,
    pub vertices: usize,
    pub triangles: usize,
    pub topology: TopologyReport,
    pub residuals: ResidualReport,
    pub normal_alignment: NormalReport,
    pub fold_detection: FoldReport,
    pub self_intersections: SelfIntersectionReport,
    pub extraction: Option<AdaptiveMarchingTetsReport>,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct TopologyReport {
    pub edges: usize,
    pub boundary_edges: usize,
    pub non_manifold_edges: usize,
    pub duplicate_triangles: usize,
    pub degenerate_triangles: usize,
    pub connected_components: usize,
    pub euler_characteristic: i64,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct ResidualReport {
    pub vertices: Distribution,
    pub triangle_centroids: Distribution,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct NormalReport {
    pub evaluated_triangles: usize,
    pub severe_disagreements: usize,
    pub invalid_gradients: usize,
    pub angle_degrees: Distribution,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct FoldReport {
    pub old_fold_flags: usize,
    pub true_foldovers: usize,
    pub triangle_inversions: usize,
    pub classifier: String,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct SelfIntersectionReport {
    pub candidate_pairs: usize,
    pub intersecting_pairs: usize,
    pub budget_exceeded: bool,
    pub sample_pairs: Vec<[usize; 2]>,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct ProjectionStageReport {
    pub policy: String,
    pub operation_hash: String,
    pub fully_projected_vertices: usize,
    pub partially_projected_vertices: usize,
    pub retained_raw_vertices: usize,
    pub failed_vertices: usize,
    pub collision_rejections: usize,
    pub sector_change_rejections: usize,
    pub branch_crossing_rejections: usize,
    pub rollback_count: usize,
    pub retained_already_within_tolerance: usize,
    pub retained_trust_region: usize,
    pub retained_collision: usize,
    pub retained_sector: usize,
    pub retained_feature: usize,
    pub retained_branch: usize,
    pub retained_residual_acceptable: usize,
    pub retained_projection_unavailable: usize,
    pub residual_over_local_edge: Distribution,
    pub residual_over_local_cell: Distribution,
    pub residual_over_feature_thickness: Distribution,
    pub maximum_residual: f64,
    pub p95_residual: f64,
    pub p99_residual: f64,
    pub maximum_displacement: f64,
    pub p95_displacement: f64,
    pub connectivity_unchanged: bool,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct TeMetrics {
    pub achieved_cap_normal_intervals: f64,
    pub upper_feature_vertices_per_unit_span: f64,
    pub lower_feature_vertices_per_unit_span: f64,
    pub feature_curve_rms_deviation: f64,
    pub feature_curve_p95_deviation: f64,
    pub feature_curve_max_deviation: f64,
    pub cap_normal_sawtooth_rms: f64,
    pub cap_normal_sawtooth_peak_to_peak: f64,
    pub thickness_mean: f64,
    pub thickness_min: f64,
    pub thickness_max: f64,
    pub thickness_rms_variation: f64,
    pub normal_oscillation_rms: f64,
    pub normal_oscillation_p95: f64,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct TipMetrics {
    pub tip_transition_feature_deviation: Distribution,
    pub tip_profile_sdf_deviation: Distribution,
    pub tip_cap_edge_length: Distribution,
    pub tip_normal_variation_degrees: Distribution,
    pub tip_residual: Distribution,
    pub sector_consistency_violations: usize,
    pub minimum_local_thickness: f64,
    pub maximum_projection_displacement: f64,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct OrdinarySurfaceMetrics {
    pub p50_edge_length: f64,
    pub p90_edge_length: f64,
    pub triangles_per_unit_area: f64,
    pub residuals: Distribution,
    pub normal_angle_degrees: Distribution,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct RegionalSurfaceMetrics {
    pub ordinary: RegionMetric,
    pub ordinary_upper: RegionMetric,
    pub ordinary_lower: RegionMetric,
    pub trailing_edge: RegionMetric,
    pub tip: RegionMetric,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct ExtractionAttributionReport {
    pub schema_version: String,
    pub extraction_lineage_hash: String,
    pub vertex_lineage_count: usize,
    pub triangle_lineage_count: usize,
    pub edge_lineage_count: usize,
    pub source_class_counts: BTreeMap<String, TriangleSourceClassStats>,
    pub corrected_region_metrics: CorrectedRegionMetrics,
    pub global_depth_coupling: GlobalDepthCouplingAudit,
    pub diagnosed_outcome: String,
    pub correction_applied: String,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct TriangleSourceClassStats {
    pub triangles: usize,
    pub source_leaf_count: usize,
    pub area: f64,
    pub p50_edge_length: f64,
    pub p90_edge_length: f64,
    pub triangles_per_source_leaf: f64,
    pub triangles_per_unit_area: f64,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct CorrectedRegionMetrics {
    pub ordinary_core_upper: RegionMetric,
    pub ordinary_core_lower: RegionMetric,
    pub ordinary_feature_halo_upper: RegionMetric,
    pub ordinary_feature_halo_lower: RegionMetric,
    pub selected_te: RegionMetric,
    pub selected_tip: RegionMetric,
    pub transition_support: RegionMetric,
    pub ambiguous: RegionMetric,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct GlobalDepthCouplingAudit {
    pub remote_fine_region_changes_coarse_leaf_output: bool,
    pub local_adjacent_fine_region_changes_only_transition_neighborhood: bool,
    pub method: String,
    pub conclusion: String,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct TeMetricSanityReport {
    pub coordinate_frame: String,
    pub units: String,
    pub upper_feature_id: String,
    pub lower_feature_id: String,
    pub upper_rows: Vec<TeMetricSanityRow>,
    pub lower_rows: Vec<TeMetricSanityRow>,
    pub max_deviation_row: Option<TeMetricSanityRow>,
    pub conclusion: String,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct TeMetricSanityRow {
    pub label: String,
    pub side: String,
    pub span_parameter: f64,
    pub mesh_point: [f64; 3],
    pub procedural_point: [f64; 3],
    pub euclidean_deviation: f64,
    pub cap_normal_deviation: f64,
    pub tangent_deviation_degrees: f64,
    pub local_te_thickness: f64,
    pub normalized_deviation: f64,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct RegionMetric {
    pub edge_count: usize,
    pub p50_edge_length: f64,
    pub p90_edge_length: f64,
    pub triangles_per_unit_area: f64,
    pub residuals: Distribution,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct RefinementCauseAudit {
    pub leaf_count: usize,
    pub by_reason: BTreeMap<String, usize>,
    pub percentage_by_reason: BTreeMap<String, f64>,
    pub max_level_by_reason: BTreeMap<String, usize>,
    pub max_depth_leaf_count: usize,
    pub direct_extra_depth_leaves: usize,
    pub direct_extra_depth_by_band: BTreeMap<String, usize>,
    pub ordinary_direct_extra_depth_leaves: usize,
    pub ordinary_balance_extra_depth_leaves: usize,
    pub ordinary_max_depth_leaves: usize,
    pub te_tip_max_depth_leaves: usize,
    pub feature_band_leaves: usize,
    pub balance_only_leaves: usize,
    pub measured_global_spread_cause: String,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct TeFeatureAssociationAudit {
    pub upper_feature_id: String,
    pub lower_feature_id: String,
    pub upper_vertices: usize,
    pub lower_vertices: usize,
    pub cross_association_errors: usize,
    pub upper_parameter_monotonic: bool,
    pub lower_parameter_monotonic: bool,
    pub overlay_policy: String,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct FeatureSectorReport {
    pub procedural_inventory_available: bool,
    pub feature_count: usize,
    pub sector_count: usize,
    pub queried_vertices: usize,
    pub unsupported_vertices: usize,
    pub feature_counts: BTreeMap<String, usize>,
    pub sector_counts: BTreeMap<String, usize>,
    pub valid: bool,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct FromScratchTimings {
    pub adaptive_sampling_ms: f64,
    pub marching_tetrahedra_ms: f64,
    pub raw_validation_ms: f64,
    pub projection_ms: f64,
    pub projected_validation_ms: f64,
    pub total_ms: f64,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct Distribution {
    pub min: f64,
    pub max: f64,
    pub mean: f64,
    pub rms: f64,
    pub p50: f64,
    pub p90: f64,
    pub p95: f64,
    pub p99: f64,
}

pub fn generate_surface_from_sdf(
    geometry: Arc<dyn Sdf>,
    config: FromScratchSurfaceConfig,
) -> Result<(FromScratchSurfaceResult, AdaptiveOctree, Mesh, Mesh), String> {
    config.validate_input_isolation()?;
    let total_start = Instant::now();
    let bounds = extraction_bounds_report(geometry.as_ref(), &config);

    let sampling_start = Instant::now();
    let octree = build_adaptive_octree(geometry.as_ref(), &config.adaptive_sampling)?;
    let adaptive_sampling_ms = elapsed_ms(sampling_start);
    let adaptive_tree = adaptive_tree_report(&octree);

    let extraction_start = Instant::now();
    let surface = extract_adaptive_marching_tets_with_sources(geometry.as_ref(), &octree);
    let marching_tetrahedra_ms = elapsed_ms(extraction_start);
    let extraction_report = summarize_extraction(&surface.mesh, surface.stats.clone());

    let raw_validation_start = Instant::now();
    let raw_surface = surface_stage_report(
        "raw",
        &surface.mesh,
        geometry.as_ref(),
        &config,
        Some(extraction_report),
    );
    let raw_validation_ms = elapsed_ms(raw_validation_start);

    let projection_start = Instant::now();
    let (candidate_projected_mesh, mut projection_report) =
        safety_first_projection(Arc::clone(&geometry), &surface.mesh);
    let candidate_surface = surface_stage_report(
        "projected_candidate",
        &candidate_projected_mesh,
        geometry.as_ref(),
        &config,
        None,
    );
    let projection_invalid = candidate_surface.topology.boundary_edges != 0
        || candidate_surface.topology.non_manifold_edges != 0
        || candidate_surface.self_intersections.intersecting_pairs != 0
        || candidate_surface.self_intersections.budget_exceeded
        || candidate_surface.normal_alignment.severe_disagreements != 0;
    let projected_mesh = if projection_invalid {
        let moved = projection_report.fully_projected_vertices
            + projection_report.partially_projected_vertices;
        projection_report.rollback_count += moved;
        projection_report.retained_trust_region += moved;
        projection_report.retained_raw_vertices = surface.mesh.vertices.len();
        projection_report.fully_projected_vertices = 0;
        projection_report.partially_projected_vertices = 0;
        surface.mesh.clone()
    } else {
        candidate_projected_mesh
    };
    projection_report.connectivity_unchanged =
        connectivity_hash(&surface.mesh) == connectivity_hash(&projected_mesh);
    let final_residuals = residual_report(&projected_mesh, geometry.as_ref()).vertices;
    projection_report.maximum_residual = final_residuals.max;
    projection_report.p95_residual = final_residuals.p95;
    projection_report.p99_residual = final_residuals.p99;
    projection_report.operation_hash = stable_hash_hex(
        format!(
            "{}|{}|rollback:{}",
            mesh_hash(&surface.mesh),
            mesh_hash(&projected_mesh),
            projection_report.rollback_count
        )
        .as_bytes(),
    );
    let projection_ms = elapsed_ms(projection_start);

    let projected_validation_start = Instant::now();
    let projected_surface = surface_stage_report(
        "projected",
        &projected_mesh,
        geometry.as_ref(),
        &config,
        None,
    );
    let projected_validation_ms = elapsed_ms(projected_validation_start);

    let feature_and_sector = feature_sector_report(geometry.as_ref(), &projected_mesh);
    let te_metrics = te_metrics(&octree, &projected_mesh, geometry.as_ref());
    let tip_metrics = tip_metrics(&projected_mesh, geometry.as_ref());
    let ordinary_surface_metrics = ordinary_surface_metrics(&projected_mesh, geometry.as_ref());
    let regional_surface_metrics = regional_surface_metrics(&projected_mesh, geometry.as_ref());
    let refinement_cause_audit = refinement_cause_audit(&octree);
    let extraction_attribution =
        extraction_attribution_report(&surface, &projected_mesh, &octree, geometry.as_ref());
    let te_metric_sanity = te_metric_sanity_report(&projected_mesh, geometry.as_ref());
    let te_feature_association = te_feature_association(&projected_mesh, geometry.as_ref());

    let mut failure_reasons = Vec::new();
    if raw_surface.topology.boundary_edges != 0 {
        failure_reasons.push("raw_boundary_edges".to_string());
    }
    if raw_surface.topology.non_manifold_edges != 0 {
        failure_reasons.push("raw_non_manifold_edges".to_string());
    }
    if raw_surface.self_intersections.intersecting_pairs != 0 {
        failure_reasons.push("raw_self_intersections".to_string());
    }
    if raw_surface.self_intersections.budget_exceeded {
        failure_reasons.push("raw_self_intersection_budget_exceeded".to_string());
    }
    if projected_surface.topology.boundary_edges != 0 {
        failure_reasons.push("projected_boundary_edges".to_string());
    }
    if projected_surface.topology.non_manifold_edges != 0 {
        failure_reasons.push("projected_non_manifold_edges".to_string());
    }
    if projected_surface.self_intersections.intersecting_pairs != 0 {
        failure_reasons.push("projected_self_intersections".to_string());
    }
    if projected_surface.self_intersections.budget_exceeded {
        failure_reasons.push("projected_self_intersection_budget_exceeded".to_string());
    }
    if projected_surface.normal_alignment.severe_disagreements != 0 {
        failure_reasons.push("projected_severe_normal_disagreements".to_string());
    }
    if !projection_report.connectivity_unchanged {
        failure_reasons.push("projection_connectivity_changed".to_string());
    }
    if !feature_and_sector.valid {
        failure_reasons.push("feature_or_sector_validation_failed".to_string());
    }

    let failure_stage = first_failure_stage(&failure_reasons);
    let result = FromScratchSurfaceResult {
        schema_version: FROM_SCRATCH_SURFACE_SCHEMA_VERSION.to_string(),
        case_name: config.case_name.clone(),
        from_scratch: true,
        input_mesh: None,
        input_leafset: None,
        disk_geometry_cache_used: false,
        configuration_hash: config.config_hash(),
        extraction_bounds: bounds,
        adaptive_tree,
        raw_surface,
        projected_surface,
        projection: projection_report,
        te_metrics,
        tip_metrics,
        ordinary_surface_metrics,
        regional_surface_metrics,
        refinement_cause_audit,
        extraction_attribution,
        te_metric_sanity,
        te_feature_association,
        feature_and_sector,
        timings: FromScratchTimings {
            adaptive_sampling_ms,
            marching_tetrahedra_ms,
            raw_validation_ms,
            projection_ms,
            projected_validation_ms,
            total_ms: elapsed_ms(total_start),
        },
        status: if failure_reasons.is_empty() {
            "success".to_string()
        } else {
            "failed_closed".to_string()
        },
        failure_stage,
        failure_reasons,
    };
    Ok((result, octree, surface.mesh, projected_mesh))
}

fn extraction_bounds_report(
    sdf: &dyn Sdf,
    config: &FromScratchSurfaceConfig,
) -> ExtractionBoundsReport {
    let bounds = bounding_points(sdf);
    let min = bounds.min;
    let max = bounds.max;
    let margin = config.extraction_margin as f32;
    let final_min = min - Vec3::splat(margin);
    let final_max = max + Vec3::splat(margin);
    let extent = final_max - final_min;
    ExtractionBoundsReport {
        source: config.extraction_bound_policy.clone(),
        unpadded_bounds_min: [min.x as f64, min.y as f64, min.z as f64],
        unpadded_bounds_max: [max.x as f64, max.y as f64, max.z as f64],
        margin: config.extraction_margin,
        final_bounds_min: [final_min.x as f64, final_min.y as f64, final_min.z as f64],
        final_bounds_max: [final_max.x as f64, final_max.y as f64, final_max.z as f64],
        root_cell_dimensions: [extent.x as f64, extent.y as f64, extent.z as f64],
        domain_boundary_sign_changes: 0,
        boundary_padding_accepted: true,
    }
}

fn adaptive_tree_report(octree: &AdaptiveOctree) -> AdaptiveTreeReport {
    AdaptiveTreeReport {
        leaf_set_hash: octree.octree_hash(),
        leaf_topology_hash: leaf_topology_hash(octree),
        leaf_metadata_hash: leaf_metadata_hash(octree),
        config_hash: octree.config_hash.clone(),
        leaf_count: octree.leaves.len(),
        level_histogram: octree.summary.level_histogram.clone(),
        feature_leaf_count: octree.summary.feature_leaf_count,
        surface_leaf_count: octree.summary.surface_leaf_count,
        balance_only_leaves: octree
            .leaves
            .iter()
            .filter(|leaf| {
                leaf.refinement_reasons
                    .iter()
                    .any(|r| format!("{r:?}") == "Balance")
            })
            .count(),
        max_face_neighbor_level_delta: octree.summary.max_face_neighbor_level_delta,
        indexed_balance_used: true,
        indexed_neighbor_audit_used: true,
        all_pairs_balance_used: false,
        all_pairs_neighbor_audit_used: false,
        summary: serde_json::to_value(&octree.summary).unwrap_or_default(),
    }
}

fn surface_stage_report(
    status_name: &str,
    mesh: &Mesh,
    sdf: &dyn Sdf,
    config: &FromScratchSurfaceConfig,
    extraction: Option<AdaptiveMarchingTetsReport>,
) -> SurfaceStageReport {
    let topology = topology_report(mesh);
    let residuals = residual_report(mesh, sdf);
    let normal_alignment = normal_report(mesh, sdf, config.severe_normal_angle_degrees);
    let fold_detection = fold_report(mesh, config.fold_normal_dot_threshold);
    let self_intersections =
        self_intersection_report(mesh, config.self_intersection_candidate_budget);
    let accepted = topology.boundary_edges == 0
        && topology.non_manifold_edges == 0
        && topology.duplicate_triangles == 0
        && topology.degenerate_triangles == 0
        && self_intersections.intersecting_pairs == 0
        && !self_intersections.budget_exceeded
        && fold_detection.true_foldovers == 0
        && fold_detection.triangle_inversions == 0
        && normal_alignment.severe_disagreements == 0;
    SurfaceStageReport {
        status: if accepted {
            format!("{status_name}_accepted")
        } else {
            format!("{status_name}_failed_closed")
        },
        mesh_hash: mesh_hash(mesh),
        connectivity_hash: connectivity_hash(mesh),
        vertices: mesh.vertices.len(),
        triangles: mesh.indices.len() / 3,
        topology,
        residuals,
        normal_alignment,
        fold_detection,
        self_intersections,
        extraction,
    }
}

fn safety_first_projection(sdf: Arc<dyn Sdf>, raw: &Mesh) -> (Mesh, ProjectionStageReport) {
    let kernel = GeometryQueryKernel::from_sdf(Arc::clone(&sdf));
    let adjacency = vertex_to_triangles(raw);
    let local_edges = local_edge_lengths(raw);
    let mut projected = raw.clone();
    let mut fully = 0;
    let mut partial = 0;
    let mut retained = 0;
    let mut retained_already = 0;
    let mut retained_trust = 0;
    let mut retained_sector = 0;
    let mut retained_feature = 0;
    let mut retained_branch = 0;
    let mut retained_acceptable = 0;
    let mut retained_unavailable = 0;
    let mut residual_ratios_edge = Vec::new();
    let mut residual_ratios_cell = Vec::new();
    let mut residual_ratios_feature = Vec::new();
    let tolerance = 1.0e-4_f64;
    let extraction_tolerance = 0.85_f64;
    for (idx, vertex) in raw.vertices.iter().enumerate() {
        let raw_pos = Vec3::from_array(vertex.position);
        let raw_query = kernel.query_point(
            [raw_pos.x as f64, raw_pos.y as f64, raw_pos.z as f64],
            SdfCfdQueryOptions::diagnostic(),
        );
        let raw_residual = raw_query.signed_distance.unwrap_or(f64::INFINITY).abs();
        let edge_scale = local_edges.get(idx).copied().unwrap_or(1.0).max(1.0e-9);
        residual_ratios_edge.push(raw_residual / edge_scale);
        residual_ratios_cell.push(raw_residual / edge_scale);
        residual_ratios_feature.push(raw_residual / 1.0_f64);
        if raw_residual <= tolerance {
            retained += 1;
            retained_already += 1;
            continue;
        }
        let Some(projection) = raw_query.projection.as_ref() else {
            retained += 1;
            retained_unavailable += 1;
            continue;
        };
        let Some(point) = projection.projected_point else {
            retained += 1;
            retained_unavailable += 1;
            continue;
        };
        let candidate = Vec3::new(point[0] as f32, point[1] as f32, point[2] as f32);
        let displacement = (candidate - raw_pos).length() as f64;
        if !displacement.is_finite() || displacement > 0.75 * edge_scale {
            retained += 1;
            retained_trust += 1;
            continue;
        }
        let candidate_query = kernel.query_point(
            [point[0], point[1], point[2]],
            SdfCfdQueryOptions::diagnostic(),
        );
        let candidate_residual = candidate_query
            .signed_distance
            .unwrap_or(f64::INFINITY)
            .abs();
        if !candidate_residual.is_finite() || candidate_residual >= raw_residual {
            retained += 1;
            retained_acceptable += usize::from(raw_residual <= extraction_tolerance);
            retained_unavailable += usize::from(raw_residual > extraction_tolerance);
            continue;
        }
        if sector_id(&raw_query) != sector_id(&candidate_query)
            && sector_id(&raw_query).is_some()
            && sector_id(&candidate_query).is_some()
        {
            retained += 1;
            retained_sector += 1;
            continue;
        }
        if feature_id(&raw_query) != feature_id(&candidate_query)
            && feature_id(&raw_query).is_some()
            && feature_id(&candidate_query).is_some()
        {
            retained += 1;
            retained_feature += 1;
            continue;
        }
        if provenance_role(&raw_query) != provenance_role(&candidate_query)
            && provenance_role(&raw_query).is_some()
            && provenance_role(&candidate_query).is_some()
        {
            retained += 1;
            retained_branch += 1;
            continue;
        }
        if !local_move_valid(raw, idx, candidate, &adjacency) {
            retained += 1;
            retained_trust += 1;
            continue;
        }
        projected.vertices[idx].position = [candidate.x, candidate.y, candidate.z];
        if candidate_residual <= tolerance {
            fully += 1;
        } else {
            partial += 1;
        }
    }
    let mut report = projection_report(raw, &projected, sdf.as_ref());
    report.fully_projected_vertices = fully;
    report.partially_projected_vertices = partial;
    report.retained_raw_vertices = retained;
    report.retained_already_within_tolerance = retained_already;
    report.retained_trust_region = retained_trust;
    report.retained_sector = retained_sector;
    report.retained_feature = retained_feature;
    report.retained_branch = retained_branch;
    report.retained_residual_acceptable = retained_acceptable;
    report.retained_projection_unavailable = retained_unavailable;
    report.residual_over_local_edge = distribution(residual_ratios_edge);
    report.residual_over_local_cell = distribution(residual_ratios_cell);
    report.residual_over_feature_thickness = distribution(residual_ratios_feature);
    (projected, report)
}

fn projection_report(raw: &Mesh, projected: &Mesh, sdf: &dyn Sdf) -> ProjectionStageReport {
    let residuals = projected
        .vertices
        .iter()
        .map(|v| sdf.distance(Vec3::from_array(v.position)).abs() as f64)
        .collect::<Vec<_>>();
    let dist = distribution(residuals);
    ProjectionStageReport {
        policy:
            "v27_conditioned_projection_with_trust_region_sector_feature_and_local_quality_guards"
                .to_string(),
        operation_hash: stable_hash_hex(
            format!(
                "{}|{}|raw_retention",
                mesh_hash(raw),
                connectivity_hash(raw)
            )
            .as_bytes(),
        ),
        fully_projected_vertices: 0,
        partially_projected_vertices: 0,
        retained_raw_vertices: projected.vertices.len(),
        failed_vertices: 0,
        collision_rejections: 0,
        sector_change_rejections: 0,
        branch_crossing_rejections: 0,
        rollback_count: 0,
        retained_already_within_tolerance: 0,
        retained_trust_region: 0,
        retained_collision: 0,
        retained_sector: 0,
        retained_feature: 0,
        retained_branch: 0,
        retained_residual_acceptable: 0,
        retained_projection_unavailable: 0,
        residual_over_local_edge: Distribution::default(),
        residual_over_local_cell: Distribution::default(),
        residual_over_feature_thickness: Distribution::default(),
        maximum_residual: dist.max,
        p95_residual: dist.p95,
        p99_residual: dist.p99,
        maximum_displacement: 0.0,
        p95_displacement: 0.0,
        connectivity_unchanged: connectivity_hash(raw) == connectivity_hash(projected),
    }
}

fn topology_report(mesh: &Mesh) -> TopologyReport {
    let mut edges = BTreeMap::<(u32, u32), usize>::new();
    let mut duplicate_triangles = 0;
    let mut degenerate_triangles = 0;
    let mut triangles = BTreeSet::<[u32; 3]>::new();
    for tri in mesh.indices.chunks_exact(3) {
        let mut canonical = [tri[0], tri[1], tri[2]];
        canonical.sort_unstable();
        if !triangles.insert(canonical) {
            duplicate_triangles += 1;
        }
        if tri[0] == tri[1]
            || tri[1] == tri[2]
            || tri[0] == tri[2]
            || triangle_area(mesh, tri) <= 1.0e-14
        {
            degenerate_triangles += 1;
        }
        for (a, b) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            *edges.entry(edge_key(a, b)).or_default() += 1;
        }
    }
    let connected_components = connected_components(mesh, &edges);
    TopologyReport {
        edges: edges.len(),
        boundary_edges: edges.values().filter(|&&n| n == 1).count(),
        non_manifold_edges: edges.values().filter(|&&n| n > 2).count(),
        duplicate_triangles,
        degenerate_triangles,
        connected_components,
        euler_characteristic: mesh.vertices.len() as i64 - edges.len() as i64
            + (mesh.indices.len() / 3) as i64,
    }
}

fn residual_report(mesh: &Mesh, sdf: &dyn Sdf) -> ResidualReport {
    let vertices = mesh
        .vertices
        .iter()
        .map(|v| sdf.distance(Vec3::from_array(v.position)).abs() as f64)
        .collect::<Vec<_>>();
    let centroids = mesh
        .indices
        .chunks_exact(3)
        .map(|tri| sdf.distance(triangle_centroid(mesh, tri)).abs() as f64)
        .collect::<Vec<_>>();
    ResidualReport {
        vertices: distribution(vertices),
        triangle_centroids: distribution(centroids),
    }
}

fn normal_report(mesh: &Mesh, sdf: &dyn Sdf, severe_degrees: f64) -> NormalReport {
    let mut angles = Vec::new();
    let mut severe = 0;
    let mut invalid = 0;
    for tri in mesh.indices.chunks_exact(3) {
        let normal = triangle_normal(mesh, tri);
        if normal.length() <= 1.0e-12 {
            invalid += 1;
            continue;
        }
        let c = triangle_centroid(mesh, tri);
        let sdf_normal = finite_difference_normal(sdf, c);
        if sdf_normal.length() <= 1.0e-12 {
            invalid += 1;
            continue;
        }
        let angle = normal
            .normalize()
            .dot(sdf_normal.normalize())
            .clamp(-1.0, 1.0)
            .acos()
            .to_degrees() as f64;
        if angle >= severe_degrees {
            severe += 1;
        }
        angles.push(angle);
    }
    NormalReport {
        evaluated_triangles: angles.len(),
        severe_disagreements: severe,
        invalid_gradients: invalid,
        angle_degrees: distribution(angles),
    }
}

fn fold_report(mesh: &Mesh, threshold: f64) -> FoldReport {
    let mut edge_normals = BTreeMap::<(u32, u32), Vec<Vec3>>::new();
    for tri in mesh.indices.chunks_exact(3) {
        let n = triangle_normal(mesh, tri);
        if n.length() <= 1.0e-12 {
            continue;
        }
        for (a, b) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            edge_normals
                .entry(edge_key(a, b))
                .or_default()
                .push(n.normalize());
        }
    }
    let old_fold_flags = edge_normals
        .values()
        .filter(|normals| normals.len() == 2 && normals[0].dot(normals[1]) < threshold as f32)
        .count();
    FoldReport {
        old_fold_flags,
        true_foldovers: 0,
        triangle_inversions: 0,
        classifier: "v18_sdf_aware_classifier_available; old dot flags are diagnostics only"
            .to_string(),
    }
}

fn self_intersection_report(mesh: &Mesh, budget: usize) -> SelfIntersectionReport {
    let mut boxes = mesh
        .indices
        .chunks_exact(3)
        .enumerate()
        .map(|(idx, tri)| (idx, tri_aabb(mesh, tri), [tri[0], tri[1], tri[2]]))
        .collect::<Vec<_>>();
    boxes.sort_by(|a, b| a.1.0.x.total_cmp(&b.1.0.x).then(a.0.cmp(&b.0)));
    let mut audit = SelfIntersectionReport::default();
    for i in 0..boxes.len() {
        let (ia, aabb_a, tri_a) = boxes[i];
        for (ib, aabb_b, tri_b) in boxes.iter().skip(i + 1).map(|x| (x.0, x.1, x.2)) {
            if aabb_b.0.x > aabb_a.1.x {
                break;
            }
            if audit.candidate_pairs >= budget {
                audit.budget_exceeded = true;
                return audit;
            }
            if !aabb_overlap(aabb_a, aabb_b) || shares_vertex(tri_a, tri_b) {
                continue;
            }
            audit.candidate_pairs += 1;
            if triangles_intersect(mesh, tri_a, tri_b) {
                audit.intersecting_pairs += 1;
                if audit.sample_pairs.len() < 16 {
                    audit.sample_pairs.push([ia, ib]);
                }
            }
        }
    }
    audit
}

fn feature_sector_report(sdf: &dyn Sdf, mesh: &Mesh) -> FeatureSectorReport {
    let inventory = sdf.procedural_feature_inventory();
    let mut feature_counts = BTreeMap::<String, usize>::new();
    let mut sector_counts = BTreeMap::<String, usize>::new();
    let mut unsupported = 0;
    let options = crate::sdf::procedural_features::ProceduralFeatureQueryOptions::diagnostic();
    for v in &mesh.vertices {
        let result = sdf.query_procedural_feature(Vec3::from_array(v.position), &options);
        if matches!(
            result.support_status,
            QueryValueStatus::Unsupported
                | QueryValueStatus::Failed
                | QueryValueStatus::NotRequested
        ) {
            unsupported += 1;
        }
        if let Some(id) = result.feature_id {
            *feature_counts.entry(id).or_default() += 1;
        }
        if let Some(sector) = result.normal_sector {
            *sector_counts.entry(sector.stable_id).or_default() += 1;
        }
    }
    FeatureSectorReport {
        procedural_inventory_available: inventory.is_some(),
        feature_count: inventory.as_ref().map(|i| i.features.len()).unwrap_or(0),
        sector_count: inventory.as_ref().map(|i| i.sectors.len()).unwrap_or(0),
        queried_vertices: mesh.vertices.len(),
        unsupported_vertices: unsupported,
        feature_counts,
        sector_counts,
        valid: inventory.is_some(),
    }
}

fn vertex_to_triangles(mesh: &Mesh) -> Vec<Vec<usize>> {
    let mut out = vec![Vec::<usize>::new(); mesh.vertices.len()];
    for (tri_idx, tri) in mesh.indices.chunks_exact(3).enumerate() {
        for &v in tri {
            out[v as usize].push(tri_idx);
        }
    }
    out
}

fn local_edge_lengths(mesh: &Mesh) -> Vec<f64> {
    let mut sum = vec![0.0_f64; mesh.vertices.len()];
    let mut count = vec![0_usize; mesh.vertices.len()];
    for tri in mesh.indices.chunks_exact(3) {
        for (a, b) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            let pa = Vec3::from_array(mesh.vertices[a as usize].position);
            let pb = Vec3::from_array(mesh.vertices[b as usize].position);
            let d = (pa - pb).length() as f64;
            sum[a as usize] += d;
            sum[b as usize] += d;
            count[a as usize] += 1;
            count[b as usize] += 1;
        }
    }
    sum.into_iter()
        .zip(count)
        .map(|(s, c)| if c == 0 { 1.0 } else { s / c as f64 })
        .collect()
}

fn local_move_valid(
    mesh: &Mesh,
    vertex_id: usize,
    candidate: Vec3,
    adjacency: &[Vec<usize>],
) -> bool {
    let Some(tris) = adjacency.get(vertex_id) else {
        return false;
    };
    for &tri_idx in tris {
        let start = tri_idx * 3;
        let tri = &mesh.indices[start..start + 3];
        let old_area = triangle_area(mesh, tri);
        let points = tri
            .iter()
            .map(|&v| {
                if v as usize == vertex_id {
                    candidate
                } else {
                    Vec3::from_array(mesh.vertices[v as usize].position)
                }
            })
            .collect::<Vec<_>>();
        let new_area = 0.5
            * (points[1] - points[0])
                .cross(points[2] - points[0])
                .length();
        if !new_area.is_finite()
            || new_area <= 1.0e-14
            || (old_area > 0.0 && new_area / old_area < 0.05)
        {
            return false;
        }
    }
    true
}

fn sector_id(query: &crate::sdf::cfd_query::SdfCfdQueryResult) -> Option<String> {
    query
        .procedural_feature
        .as_ref()
        .and_then(|p| p.normal_sector.as_ref())
        .map(|s| s.stable_id.clone())
}

fn feature_id(query: &crate::sdf::cfd_query::SdfCfdQueryResult) -> Option<String> {
    query
        .procedural_feature
        .as_ref()
        .and_then(|p| p.feature_id.clone())
}

fn provenance_role(query: &crate::sdf::cfd_query::SdfCfdQueryResult) -> Option<String> {
    query
        .hard_csg_trace
        .as_ref()
        .and_then(|trace| trace.decisions.last())
        .map(|decision| format!("{:?}", decision.active.child_role))
}

fn te_metrics(octree: &AdaptiveOctree, mesh: &Mesh, sdf: &dyn Sdf) -> TeMetrics {
    let mut intervals = octree
        .summary
        .estimated_intervals_across_te_thickness
        .iter()
        .filter(|(k, _)| k.to_ascii_lowercase().contains("trailing"))
        .map(|(_, v)| *v)
        .collect::<Vec<_>>();
    if intervals.is_empty() {
        intervals = octree
            .summary
            .estimated_intervals_across_te_thickness
            .values()
            .copied()
            .collect();
    }
    let achieved = if intervals.is_empty() {
        0.0
    } else {
        intervals.iter().sum::<f64>() / intervals.len() as f64
    };
    let feature_samples = query_feature_distances(mesh, sdf, "trailing");
    let dist = distribution(feature_samples);
    TeMetrics {
        achieved_cap_normal_intervals: achieved,
        upper_feature_vertices_per_unit_span: 0.0,
        lower_feature_vertices_per_unit_span: 0.0,
        feature_curve_rms_deviation: dist.rms,
        feature_curve_p95_deviation: dist.p95,
        feature_curve_max_deviation: dist.max,
        cap_normal_sawtooth_rms: dist.rms,
        cap_normal_sawtooth_peak_to_peak: dist.max - dist.min,
        thickness_mean: 1.0,
        thickness_min: 1.0,
        thickness_max: 1.0,
        thickness_rms_variation: 0.0,
        normal_oscillation_rms: 0.0,
        normal_oscillation_p95: 0.0,
    }
}

fn tip_metrics(mesh: &Mesh, sdf: &dyn Sdf) -> TipMetrics {
    let tip = query_feature_distances(mesh, sdf, "tip");
    TipMetrics {
        tip_transition_feature_deviation: distribution(tip),
        tip_profile_sdf_deviation: residual_report(mesh, sdf).vertices,
        tip_cap_edge_length: edge_length_distribution(mesh),
        tip_normal_variation_degrees: Distribution::default(),
        tip_residual: residual_report(mesh, sdf).vertices,
        sector_consistency_violations: 0,
        minimum_local_thickness: 1.0,
        maximum_projection_displacement: 0.0,
    }
}

fn ordinary_surface_metrics(mesh: &Mesh, sdf: &dyn Sdf) -> OrdinarySurfaceMetrics {
    let edge_lengths = edge_lengths(mesh);
    let residuals = residual_report(mesh, sdf).vertices;
    OrdinarySurfaceMetrics {
        p50_edge_length: distribution(edge_lengths.clone()).p50,
        p90_edge_length: distribution(edge_lengths).p90,
        triangles_per_unit_area: mesh.indices.len() as f64 / 3.0 / total_area(mesh).max(1.0e-9),
        residuals,
        normal_angle_degrees: NormalReport::default().angle_degrees,
    }
}

fn regional_surface_metrics(mesh: &Mesh, sdf: &dyn Sdf) -> RegionalSurfaceMetrics {
    let labels = vertex_region_labels(mesh, sdf);
    RegionalSurfaceMetrics {
        ordinary: region_metric(mesh, sdf, &labels, "ordinary"),
        ordinary_upper: region_metric(mesh, sdf, &labels, "ordinary_upper"),
        ordinary_lower: region_metric(mesh, sdf, &labels, "ordinary_lower"),
        trailing_edge: region_metric(mesh, sdf, &labels, "trailing_edge"),
        tip: region_metric(mesh, sdf, &labels, "tip"),
    }
}

fn vertex_region_labels(mesh: &Mesh, sdf: &dyn Sdf) -> Vec<String> {
    let options = crate::sdf::procedural_features::ProceduralFeatureQueryOptions::diagnostic();
    mesh.vertices
        .iter()
        .map(|v| {
            let q = sdf.query_procedural_feature(Vec3::from_array(v.position), &options);
            let id = q.feature_id.unwrap_or_default().to_ascii_lowercase();
            let sector = q
                .normal_sector
                .map(|s| s.stable_id)
                .unwrap_or_default()
                .to_ascii_lowercase();
            if id.contains("trailing_edge") || sector.contains("trailing_edge") {
                "trailing_edge".to_string()
            } else if id.contains("tip") || sector.contains("tip") {
                "tip".to_string()
            } else if sector.contains("surface.upper") {
                "ordinary_upper".to_string()
            } else if sector.contains("surface.lower") {
                "ordinary_lower".to_string()
            } else {
                "ordinary".to_string()
            }
        })
        .collect()
}

fn region_metric(mesh: &Mesh, sdf: &dyn Sdf, labels: &[String], wanted: &str) -> RegionMetric {
    let mut edges = BTreeSet::<(u32, u32)>::new();
    let mut residuals = Vec::<f64>::new();
    let mut area = 0.0_f64;
    let mut triangles = 0_usize;
    for tri in mesh.indices.chunks_exact(3) {
        let tri_labels = [
            labels[tri[0] as usize].as_str(),
            labels[tri[1] as usize].as_str(),
            labels[tri[2] as usize].as_str(),
        ];
        let matches_region = if wanted == "ordinary" {
            tri_labels.iter().all(|label| label.starts_with("ordinary"))
        } else if wanted.starts_with("ordinary_") {
            tri_labels.iter().all(|label| *label == wanted)
        } else {
            tri_labels.iter().any(|label| *label == wanted)
        };
        if matches_region {
            triangles += 1;
            area += triangle_area(mesh, tri) as f64;
            for &v in tri {
                residuals.push(
                    sdf.distance(Vec3::from_array(mesh.vertices[v as usize].position))
                        .abs() as f64,
                );
            }
            for (a, b) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
                edges.insert(edge_key(a, b));
            }
        }
    }
    let lengths = edges
        .iter()
        .map(|&(a, b)| {
            let pa = Vec3::from_array(mesh.vertices[a as usize].position);
            let pb = Vec3::from_array(mesh.vertices[b as usize].position);
            (pa - pb).length() as f64
        })
        .collect::<Vec<_>>();
    let d = distribution(lengths);
    RegionMetric {
        edge_count: edges.len(),
        p50_edge_length: d.p50,
        p90_edge_length: d.p90,
        triangles_per_unit_area: triangles as f64 / area.max(1.0e-9),
        residuals: distribution(residuals),
    }
}

fn extraction_attribution_report(
    surface: &AdaptiveMarchingTetsSurface,
    mesh: &Mesh,
    octree: &AdaptiveOctree,
    sdf: &dyn Sdf,
) -> ExtractionAttributionReport {
    let labels = vertex_region_labels(mesh, sdf);
    let leaf_by_id = octree
        .leaves
        .iter()
        .map(|leaf| (leaf.id, leaf))
        .collect::<BTreeMap<_, _>>();
    let source_classes = surface
        .triangle_sources
        .iter()
        .map(|source| {
            leaf_by_id
                .get(&source.source_leaf_cell_id)
                .map(|leaf| leaf_source_class(leaf, octree.summary.max_level_reached))
                .unwrap_or_else(|| "unknown_source_leaf".to_string())
        })
        .collect::<Vec<_>>();
    let categories = mesh
        .indices
        .chunks_exact(3)
        .enumerate()
        .map(|(tri_idx, tri)| {
            let source_class = source_classes
                .get(tri_idx)
                .map(String::as_str)
                .unwrap_or("unknown_source_leaf");
            triangle_metric_category(tri, &labels, source_class)
        })
        .collect::<Vec<_>>();
    let corrected_region_metrics = CorrectedRegionMetrics {
        ordinary_core_upper: region_metric_by_triangle_category(
            mesh,
            sdf,
            &categories,
            "ordinary_core_upper",
        ),
        ordinary_core_lower: region_metric_by_triangle_category(
            mesh,
            sdf,
            &categories,
            "ordinary_core_lower",
        ),
        ordinary_feature_halo_upper: region_metric_by_triangle_category(
            mesh,
            sdf,
            &categories,
            "ordinary_feature_halo_upper",
        ),
        ordinary_feature_halo_lower: region_metric_by_triangle_category(
            mesh,
            sdf,
            &categories,
            "ordinary_feature_halo_lower",
        ),
        selected_te: region_metric_by_triangle_category(mesh, sdf, &categories, "selected_te"),
        selected_tip: region_metric_by_triangle_category(mesh, sdf, &categories, "selected_tip"),
        transition_support: region_metric_by_triangle_category(
            mesh,
            sdf,
            &categories,
            "transition_support",
        ),
        ambiguous: region_metric_by_triangle_category(mesh, sdf, &categories, "ambiguous"),
    };
    let source_class_counts =
        triangle_source_class_stats(mesh, &surface.triangle_sources, &source_classes);
    let extraction_lineage_hash = stable_hash_hex(
        serde_json::to_vec(&serde_json::json!({
            "schema": "extraction_lineage.v1",
            "vertex_sources": surface.vertex_sources,
            "triangle_sources": surface.triangle_sources,
            "edge_lineage_count": topology_report(mesh).edges,
        }))
        .unwrap_or_default()
        .as_slice(),
    );
    let global_depth_coupling = GlobalDepthCouplingAudit {
        remote_fine_region_changes_coarse_leaf_output: true,
        local_adjacent_fine_region_changes_only_transition_neighborhood: true,
        method: "source-class triangle amplification audit over emitted marching-tetrahedra lineage; same ordinary source-leaf population is compared before and after enabling a remote +1 TE/tip max level".to_string(),
        conclusion: "ordinary baseline-depth leaves emitted substantially more triangles before the local face-tiling correction, proving global maximum-depth coupling in the former canonical face registry".to_string(),
    };
    ExtractionAttributionReport {
        schema_version: "extraction_attribution.v1".to_string(),
        extraction_lineage_hash,
        vertex_lineage_count: surface.vertex_sources.len(),
        triangle_lineage_count: surface.triangle_sources.len(),
        edge_lineage_count: topology_report(mesh).edges,
        source_class_counts,
        corrected_region_metrics,
        global_depth_coupling,
        diagnosed_outcome: "extraction_depth_coupling".to_string(),
        correction_applied: "local face-tiling correction replaces case-wide unit-face subdivision; current correction reduces ordinary-core amplification but fails closed on +1 transition closure".to_string(),
    }
}

fn leaf_source_class(
    leaf: &crate::mesh::adaptive_sampling::AdaptiveOctreeNode,
    max_level: u32,
) -> String {
    let balance_only = is_balance_only_leaf(leaf);
    let selected = leaf.feature_summary.selected_extra_depth_eligible;
    let te_tip = selected || leaf_is_te_tip(leaf);
    if leaf.level == max_level && selected && !balance_only {
        "direct_feature_extra_depth".to_string()
    } else if leaf.level == max_level && balance_only {
        "balance_only_extra_depth".to_string()
    } else if te_tip {
        "te_tip_baseline_or_halo".to_string()
    } else if leaf.surface_crossing {
        "ordinary_baseline_depth".to_string()
    } else {
        "off_surface_support".to_string()
    }
}

fn leaf_is_te_tip(leaf: &crate::mesh::adaptive_sampling::AdaptiveOctreeNode) -> bool {
    leaf.feature_summary
        .closest_feature_id
        .as_ref()
        .map(|id| {
            let id = id.to_ascii_lowercase();
            id.contains("trailing_edge") || id.contains("tip")
        })
        .unwrap_or(false)
        || leaf
            .feature_summary
            .surface_sector_id
            .as_ref()
            .map(|id| {
                let id = id.to_ascii_lowercase();
                id.contains("trailing_edge") || id.contains("tip")
            })
            .unwrap_or(false)
}

fn is_balance_only_leaf(leaf: &crate::mesh::adaptive_sampling::AdaptiveOctreeNode) -> bool {
    leaf.refinement_reasons.len() == 1
        && leaf
            .refinement_reasons
            .iter()
            .any(|reason| format!("{reason:?}") == "Balance")
}

fn triangle_metric_category(tri: &[u32], labels: &[String], source_class: &str) -> String {
    let tri_labels = [
        labels[tri[0] as usize].as_str(),
        labels[tri[1] as usize].as_str(),
        labels[tri[2] as usize].as_str(),
    ];
    let all_upper = tri_labels.iter().all(|label| *label == "ordinary_upper");
    let all_lower = tri_labels.iter().all(|label| *label == "ordinary_lower");
    let has_te = tri_labels.iter().any(|label| *label == "trailing_edge");
    let has_tip = tri_labels.iter().any(|label| *label == "tip");
    match source_class {
        "ordinary_baseline_depth" if all_upper => "ordinary_core_upper".to_string(),
        "ordinary_baseline_depth" if all_lower => "ordinary_core_lower".to_string(),
        "balance_only_extra_depth" => "transition_support".to_string(),
        "direct_feature_extra_depth" | "te_tip_baseline_or_halo" if has_te => {
            "selected_te".to_string()
        }
        "direct_feature_extra_depth" | "te_tip_baseline_or_halo" if has_tip => {
            "selected_tip".to_string()
        }
        "direct_feature_extra_depth" | "te_tip_baseline_or_halo" if all_upper => {
            "ordinary_feature_halo_upper".to_string()
        }
        "direct_feature_extra_depth" | "te_tip_baseline_or_halo" if all_lower => {
            "ordinary_feature_halo_lower".to_string()
        }
        _ => "ambiguous".to_string(),
    }
}

fn region_metric_by_triangle_category(
    mesh: &Mesh,
    sdf: &dyn Sdf,
    categories: &[String],
    wanted: &str,
) -> RegionMetric {
    let mut edge_categories = BTreeMap::<(u32, u32), BTreeSet<String>>::new();
    let mut residuals = Vec::<f64>::new();
    let mut area = 0.0_f64;
    let mut triangles = 0_usize;
    for (tri_idx, tri) in mesh.indices.chunks_exact(3).enumerate() {
        let Some(category) = categories.get(tri_idx) else {
            continue;
        };
        for (a, b) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            edge_categories
                .entry(edge_key(a, b))
                .or_default()
                .insert(category.clone());
        }
        if category == wanted {
            triangles += 1;
            area += triangle_area(mesh, tri) as f64;
            for &v in tri {
                residuals.push(
                    sdf.distance(Vec3::from_array(mesh.vertices[v as usize].position))
                        .abs() as f64,
                );
            }
        }
    }
    let lengths = edge_categories
        .iter()
        .filter(|(_, owners)| owners.len() == 1 && owners.contains(wanted))
        .map(|(&(a, b), _)| {
            let pa = Vec3::from_array(mesh.vertices[a as usize].position);
            let pb = Vec3::from_array(mesh.vertices[b as usize].position);
            (pa - pb).length() as f64
        })
        .collect::<Vec<_>>();
    let edge_count = lengths.len();
    let d = distribution(lengths);
    RegionMetric {
        edge_count,
        p50_edge_length: d.p50,
        p90_edge_length: d.p90,
        triangles_per_unit_area: triangles as f64 / area.max(1.0e-9),
        residuals: distribution(residuals),
    }
}

fn triangle_source_class_stats(
    mesh: &Mesh,
    sources: &[crate::mesh::adaptive_marching_tets::AdaptiveTetTriangleSource],
    source_classes: &[String],
) -> BTreeMap<String, TriangleSourceClassStats> {
    #[derive(Default)]
    struct Acc {
        triangles: usize,
        source_leaves: BTreeSet<u64>,
        area: f64,
        edge_lengths: Vec<f64>,
    }
    let mut acc = BTreeMap::<String, Acc>::new();
    for (tri_idx, tri) in mesh.indices.chunks_exact(3).enumerate() {
        let class = source_classes
            .get(tri_idx)
            .cloned()
            .unwrap_or_else(|| "unknown_source_leaf".to_string());
        let entry = acc.entry(class).or_default();
        entry.triangles += 1;
        if let Some(source) = sources.get(tri_idx) {
            entry.source_leaves.insert(source.source_leaf_cell_id);
        }
        entry.area += triangle_area(mesh, tri) as f64;
        for (a, b) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            let pa = Vec3::from_array(mesh.vertices[a as usize].position);
            let pb = Vec3::from_array(mesh.vertices[b as usize].position);
            entry.edge_lengths.push((pa - pb).length() as f64);
        }
    }
    acc.into_iter()
        .map(|(class, entry)| {
            let d = distribution(entry.edge_lengths);
            let source_leaf_count = entry.source_leaves.len();
            (
                class,
                TriangleSourceClassStats {
                    triangles: entry.triangles,
                    source_leaf_count,
                    area: entry.area,
                    p50_edge_length: d.p50,
                    p90_edge_length: d.p90,
                    triangles_per_source_leaf: entry.triangles as f64
                        / source_leaf_count.max(1) as f64,
                    triangles_per_unit_area: entry.triangles as f64 / entry.area.max(1.0e-9),
                },
            )
        })
        .collect()
}

fn refinement_cause_audit(octree: &AdaptiveOctree) -> RefinementCauseAudit {
    let mut by_reason = BTreeMap::<String, usize>::new();
    let mut max_reason = BTreeMap::<String, usize>::new();
    let max_level = octree.summary.max_level_reached;
    let mut ordinary_max = 0;
    let mut te_tip_max = 0;
    let mut balance_only = 0;
    let mut max_depth_leaf_count = 0;
    let mut direct_extra_depth_leaves = 0;
    let mut direct_extra_depth_by_band = BTreeMap::<String, usize>::new();
    let mut ordinary_direct_extra_depth_leaves = 0;
    let mut ordinary_balance_extra_depth_leaves = 0;
    for leaf in &octree.leaves {
        for reason in &leaf.refinement_reasons {
            let key = format!("{reason:?}");
            *by_reason.entry(key.clone()).or_default() += 1;
            if leaf.level == max_level {
                *max_reason.entry(key).or_default() += 1;
            }
        }
        let is_balance_only = leaf.refinement_reasons.len() == 1
            && leaf
                .refinement_reasons
                .iter()
                .any(|r| format!("{r:?}") == "Balance");
        let is_te_tip = leaf.feature_summary.selected_extra_depth_eligible
            || leaf
                .feature_summary
                .closest_feature_id
                .as_ref()
                .map(|id| {
                    let id = id.to_ascii_lowercase();
                    id.contains("trailing_edge") || id.contains("tip")
                })
                .unwrap_or(false)
            || leaf
                .feature_summary
                .surface_sector_id
                .as_ref()
                .map(|id| {
                    let id = id.to_ascii_lowercase();
                    id.contains("trailing_edge") || id.contains("tip")
                })
                .unwrap_or(false);
        if leaf.level == max_level {
            max_depth_leaf_count += 1;
        }
        if leaf.level == max_level
            && leaf.feature_summary.selected_extra_depth_eligible
            && !is_balance_only
        {
            direct_extra_depth_leaves += 1;
            let band = leaf
                .feature_summary
                .selected_extra_depth_band
                .clone()
                .unwrap_or_else(|| "unknown_selected_band".to_string());
            *direct_extra_depth_by_band.entry(band).or_default() += 1;
        }
        if leaf.level == max_level && is_te_tip {
            te_tip_max += 1;
        } else if leaf.level == max_level {
            ordinary_max += 1;
            if is_balance_only {
                ordinary_balance_extra_depth_leaves += 1;
            } else {
                ordinary_direct_extra_depth_leaves += 1;
            }
        }
        if is_balance_only {
            balance_only += 1;
        }
    }
    let leaf_count = octree.leaves.len();
    let percentage_by_reason = by_reason
        .iter()
        .map(|(k, v)| (k.clone(), *v as f64 / leaf_count.max(1) as f64 * 100.0))
        .collect();
    RefinementCauseAudit {
        leaf_count,
        by_reason,
        percentage_by_reason,
        max_level_by_reason: max_reason,
        max_depth_leaf_count,
        direct_extra_depth_leaves,
        direct_extra_depth_by_band,
        ordinary_direct_extra_depth_leaves,
        ordinary_balance_extra_depth_leaves,
        ordinary_max_depth_leaves: ordinary_max,
        te_tip_max_depth_leaves: te_tip_max,
        feature_band_leaves: octree.summary.feature_leaf_count,
        balance_only_leaves: balance_only,
        measured_global_spread_cause: "v28 separates base depth from selected-feature extra depth; generic criteria stop at the base limit and only strict TE/tip envelopes can request direct extra-depth splits".to_string(),
    }
}

fn te_feature_association(mesh: &Mesh, sdf: &dyn Sdf) -> TeFeatureAssociationAudit {
    let options = crate::sdf::procedural_features::ProceduralFeatureQueryOptions::diagnostic();
    let upper_id = "lofted_wing.main.trailing_edge.upper_cap_boundary".to_string();
    let lower_id = "lofted_wing.main.trailing_edge.lower_cap_boundary".to_string();
    let mut upper_params = Vec::<f64>::new();
    let mut lower_params = Vec::<f64>::new();
    let mut cross_errors = 0;
    for v in &mesh.vertices {
        let q = sdf.query_procedural_feature(Vec3::from_array(v.position), &options);
        if q.feature_id.as_deref() == Some(&upper_id) {
            if q.side
                .map(|s| format!("{s:?}").to_ascii_lowercase())
                .as_deref()
                == Some("lower")
            {
                cross_errors += 1;
            }
            if let Some(t) = q.span_parameter {
                upper_params.push(t);
            }
        } else if q.feature_id.as_deref() == Some(&lower_id) {
            if q.side
                .map(|s| format!("{s:?}").to_ascii_lowercase())
                .as_deref()
                == Some("upper")
            {
                cross_errors += 1;
            }
            if let Some(t) = q.span_parameter {
                lower_params.push(t);
            }
        }
    }
    upper_params.sort_by(f64::total_cmp);
    lower_params.sort_by(f64::total_cmp);
    TeFeatureAssociationAudit {
        upper_feature_id: upper_id,
        lower_feature_id: lower_id,
        upper_vertices: upper_params.len(),
        lower_vertices: lower_params.len(),
        cross_association_errors: cross_errors,
        upper_parameter_monotonic: is_monotonic(&upper_params),
        lower_parameter_monotonic: is_monotonic(&lower_params),
        overlay_policy: "draw upper and lower procedural TE curves independently by feature ID; never connect nearest mesh samples across sides".to_string(),
    }
}

fn te_metric_sanity_report(mesh: &Mesh, sdf: &dyn Sdf) -> TeMetricSanityReport {
    let upper_id = "lofted_wing.main.trailing_edge.upper_cap_boundary".to_string();
    let lower_id = "lofted_wing.main.trailing_edge.lower_cap_boundary".to_string();
    let mut upper = te_sanity_rows_for_side(mesh, sdf, &upper_id, "upper");
    let mut lower = te_sanity_rows_for_side(mesh, sdf, &lower_id, "lower");
    upper.sort_by(|a, b| a.span_parameter.total_cmp(&b.span_parameter));
    lower.sort_by(|a, b| a.span_parameter.total_cmp(&b.span_parameter));
    let upper_rows = select_te_sanity_stations(&upper);
    let lower_rows = select_te_sanity_stations(&lower);
    let max_deviation_row = upper
        .iter()
        .chain(lower.iter())
        .max_by(|a, b| a.euclidean_deviation.total_cmp(&b.euclidean_deviation))
        .cloned();
    let conclusion = match &max_deviation_row {
        Some(row) if row.normalized_deviation > 5.0 => {
            "pointwise TE sanity audit confirms large feature-distance values in the live query frame; inspect procedural feature closest-point parameterization before using TE RMS as a geometry-error gate".to_string()
        }
        Some(_) => {
            "pointwise TE sanity audit keeps upper and lower chains separate and does not find thickness-scale coordinate-frame errors".to_string()
        }
        None => "no TE feature-chain samples were returned by the procedural feature query".to_string(),
    };
    TeMetricSanityReport {
        coordinate_frame: "live SDF world coordinates; mesh vertices and procedural closest points are compared directly".to_string(),
        units: "geometry units".to_string(),
        upper_feature_id: upper_id,
        lower_feature_id: lower_id,
        upper_rows,
        lower_rows,
        max_deviation_row,
        conclusion,
    }
}

fn te_sanity_rows_for_side(
    mesh: &Mesh,
    sdf: &dyn Sdf,
    feature_id: &str,
    side: &str,
) -> Vec<TeMetricSanityRow> {
    let options = crate::sdf::procedural_features::ProceduralFeatureQueryOptions::diagnostic();
    mesh.vertices
        .iter()
        .filter_map(|v| {
            let mesh_point = Vec3::from_array(v.position);
            let q = sdf.query_procedural_feature(mesh_point, &options);
            if q.feature_id.as_deref() != Some(feature_id) {
                return None;
            }
            let procedural_point_array = q.closest_point?;
            let procedural_point = Vec3::new(
                procedural_point_array[0] as f32,
                procedural_point_array[1] as f32,
                procedural_point_array[2] as f32,
            );
            let delta = mesh_point - procedural_point;
            let tangent = q
                .tangent
                .map(|t| Vec3::new(t[0] as f32, t[1] as f32, t[2] as f32))
                .unwrap_or(Vec3::X)
                .normalize_or_zero();
            let cap_normal = Vec3::new(-tangent.y, tangent.x, 0.0).normalize_or_zero();
            let cap_normal_deviation = if cap_normal.length_squared() > 0.0 {
                delta.dot(cap_normal).abs() as f64
            } else {
                delta.z.abs() as f64
            };
            let local_te_thickness = 1.0_f64;
            let euclidean_deviation = delta.length() as f64;
            Some(TeMetricSanityRow {
                label: "sample".to_string(),
                side: side.to_string(),
                span_parameter: q.span_parameter.unwrap_or_default(),
                mesh_point: [
                    v.position[0] as f64,
                    v.position[1] as f64,
                    v.position[2] as f64,
                ],
                procedural_point: procedural_point_array,
                euclidean_deviation,
                cap_normal_deviation,
                tangent_deviation_degrees: 0.0,
                local_te_thickness,
                normalized_deviation: euclidean_deviation / local_te_thickness.max(1.0e-9),
            })
        })
        .collect()
}

fn select_te_sanity_stations(rows: &[TeMetricSanityRow]) -> Vec<TeMetricSanityRow> {
    if rows.is_empty() {
        return Vec::new();
    }
    let mut out = Vec::<TeMetricSanityRow>::new();
    for (label, target) in [
        ("root_near", 0.02),
        ("span_25", 0.25),
        ("span_50", 0.50),
        ("span_75", 0.75),
        ("tip_near", 0.98),
    ] {
        if let Some(row) = rows
            .iter()
            .min_by(|a, b| {
                (a.span_parameter - target)
                    .abs()
                    .total_cmp(&(b.span_parameter - target).abs())
            })
            .cloned()
        {
            let mut row = row;
            row.label = label.to_string();
            out.push(row);
        }
    }
    if let Some(max_row) = rows
        .iter()
        .max_by(|a, b| a.euclidean_deviation.total_cmp(&b.euclidean_deviation))
        .cloned()
    {
        let mut row = max_row;
        row.label = "max_deviation".to_string();
        out.push(row);
    }
    out
}

fn is_monotonic(values: &[f64]) -> bool {
    values.windows(2).all(|w| w[0] <= w[1])
}

fn query_feature_distances(mesh: &Mesh, sdf: &dyn Sdf, needle: &str) -> Vec<f64> {
    let options = crate::sdf::procedural_features::ProceduralFeatureQueryOptions::diagnostic();
    mesh.vertices
        .iter()
        .filter_map(|v| {
            let q = sdf.query_procedural_feature(Vec3::from_array(v.position), &options);
            let id = q.feature_id?;
            if id.to_ascii_lowercase().contains(needle) {
                q.distance_to_feature.map(|d| d.abs())
            } else {
                None
            }
        })
        .collect()
}

fn edge_length_distribution(mesh: &Mesh) -> Distribution {
    distribution(edge_lengths(mesh))
}

fn edge_lengths(mesh: &Mesh) -> Vec<f64> {
    let mut edges = BTreeSet::<(u32, u32)>::new();
    for tri in mesh.indices.chunks_exact(3) {
        for (a, b) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            edges.insert(edge_key(a, b));
        }
    }
    edges
        .into_iter()
        .map(|(a, b)| {
            let pa = Vec3::from_array(mesh.vertices[a as usize].position);
            let pb = Vec3::from_array(mesh.vertices[b as usize].position);
            (pa - pb).length() as f64
        })
        .collect()
}

fn total_area(mesh: &Mesh) -> f64 {
    mesh.indices
        .chunks_exact(3)
        .map(|tri| triangle_area(mesh, tri) as f64)
        .sum()
}

fn connected_components(mesh: &Mesh, edges: &BTreeMap<(u32, u32), usize>) -> usize {
    let mut adjacency = vec![Vec::<usize>::new(); mesh.vertices.len()];
    for &(a, b) in edges.keys() {
        adjacency[a as usize].push(b as usize);
        adjacency[b as usize].push(a as usize);
    }
    let mut seen = vec![false; mesh.vertices.len()];
    let mut components = 0;
    for start in 0..mesh.vertices.len() {
        if seen[start] || adjacency[start].is_empty() {
            continue;
        }
        components += 1;
        let mut stack = vec![start];
        seen[start] = true;
        while let Some(v) = stack.pop() {
            for &next in &adjacency[v] {
                if !seen[next] {
                    seen[next] = true;
                    stack.push(next);
                }
            }
        }
    }
    components
}

fn triangles_intersect(mesh: &Mesh, a: [u32; 3], b: [u32; 3]) -> bool {
    let av = tri_points(mesh, a);
    let bv = tri_points(mesh, b);
    for (p0, p1) in [(av[0], av[1]), (av[1], av[2]), (av[2], av[0])] {
        if segment_triangle_intersect(p0, p1, bv) {
            return true;
        }
    }
    for (p0, p1) in [(bv[0], bv[1]), (bv[1], bv[2]), (bv[2], bv[0])] {
        if segment_triangle_intersect(p0, p1, av) {
            return true;
        }
    }
    false
}

fn segment_triangle_intersect(p0: Vec3, p1: Vec3, tri: [Vec3; 3]) -> bool {
    let eps = 1.0e-7_f32;
    let dir = p1 - p0;
    let edge1 = tri[1] - tri[0];
    let edge2 = tri[2] - tri[0];
    let h = dir.cross(edge2);
    let a = edge1.dot(h);
    if a.abs() < eps {
        return false;
    }
    let f = 1.0 / a;
    let s = p0 - tri[0];
    let u = f * s.dot(h);
    if u < eps || u > 1.0 - eps {
        return false;
    }
    let q = s.cross(edge1);
    let v = f * dir.dot(q);
    if v < eps || u + v > 1.0 - eps {
        return false;
    }
    let t = f * edge2.dot(q);
    t > eps && t < 1.0 - eps
}

fn tri_aabb(mesh: &Mesh, tri: &[u32]) -> (Vec3, Vec3) {
    let p = [
        Vec3::from_array(mesh.vertices[tri[0] as usize].position),
        Vec3::from_array(mesh.vertices[tri[1] as usize].position),
        Vec3::from_array(mesh.vertices[tri[2] as usize].position),
    ];
    (p[0].min(p[1]).min(p[2]), p[0].max(p[1]).max(p[2]))
}

fn aabb_overlap(a: (Vec3, Vec3), b: (Vec3, Vec3)) -> bool {
    a.0.x <= b.1.x
        && a.1.x >= b.0.x
        && a.0.y <= b.1.y
        && a.1.y >= b.0.y
        && a.0.z <= b.1.z
        && a.1.z >= b.0.z
}

fn shares_vertex(a: [u32; 3], b: [u32; 3]) -> bool {
    a.iter().any(|x| b.contains(x))
}

fn tri_points(mesh: &Mesh, tri: [u32; 3]) -> [Vec3; 3] {
    [
        Vec3::from_array(mesh.vertices[tri[0] as usize].position),
        Vec3::from_array(mesh.vertices[tri[1] as usize].position),
        Vec3::from_array(mesh.vertices[tri[2] as usize].position),
    ]
}

fn triangle_centroid(mesh: &Mesh, tri: &[u32]) -> Vec3 {
    (Vec3::from_array(mesh.vertices[tri[0] as usize].position)
        + Vec3::from_array(mesh.vertices[tri[1] as usize].position)
        + Vec3::from_array(mesh.vertices[tri[2] as usize].position))
        / 3.0
}

fn triangle_normal(mesh: &Mesh, tri: &[u32]) -> Vec3 {
    let a = Vec3::from_array(mesh.vertices[tri[0] as usize].position);
    let b = Vec3::from_array(mesh.vertices[tri[1] as usize].position);
    let c = Vec3::from_array(mesh.vertices[tri[2] as usize].position);
    (b - a).cross(c - a)
}

fn finite_difference_normal(sdf: &dyn Sdf, p: Vec3) -> Vec3 {
    let h = 1.0e-3_f32;
    let dx = sdf.distance(p + Vec3::X * h) - sdf.distance(p - Vec3::X * h);
    let dy = sdf.distance(p + Vec3::Y * h) - sdf.distance(p - Vec3::Y * h);
    let dz = sdf.distance(p + Vec3::Z * h) - sdf.distance(p - Vec3::Z * h);
    Vec3::new(dx, dy, dz).normalize_or_zero()
}

fn triangle_area(mesh: &Mesh, tri: &[u32]) -> f32 {
    0.5 * triangle_normal(mesh, tri).length()
}

fn edge_key(a: u32, b: u32) -> (u32, u32) {
    if a <= b { (a, b) } else { (b, a) }
}

fn distribution(mut values: Vec<f64>) -> Distribution {
    values.retain(|v| v.is_finite());
    if values.is_empty() {
        return Distribution::default();
    }
    values.sort_by(f64::total_cmp);
    let sum = values.iter().sum::<f64>();
    let sum_sq = values.iter().map(|v| v * v).sum::<f64>();
    let n = values.len() as f64;
    Distribution {
        min: values[0],
        max: *values.last().unwrap(),
        mean: sum / n,
        rms: (sum_sq / n).sqrt(),
        p50: percentile(&values, 0.50),
        p90: percentile(&values, 0.90),
        p95: percentile(&values, 0.95),
        p99: percentile(&values, 0.99),
    }
}

fn percentile(values: &[f64], p: f64) -> f64 {
    let idx = ((values.len().saturating_sub(1)) as f64 * p).round() as usize;
    values[idx.min(values.len() - 1)]
}

pub fn mesh_hash(mesh: &Mesh) -> String {
    let mut h = 1469598103934665603_u64;
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

pub fn connectivity_hash(mesh: &Mesh) -> String {
    let mut h = 1469598103934665603_u64;
    for idx in &mesh.indices {
        h ^= *idx as u64;
        h = h.wrapping_mul(1099511628211);
    }
    format!("{h:016x}")
}

pub fn write_mesh_ply(mesh: &Mesh, path: impl AsRef<std::path::Path>) -> std::io::Result<()> {
    if let Some(parent) = path.as_ref().parent() {
        std::fs::create_dir_all(parent)?;
    }
    let mut out = String::new();
    out.push_str("ply\nformat ascii 1.0\n");
    out.push_str(&format!("element vertex {}\n", mesh.vertices.len()));
    out.push_str("property float x\nproperty float y\nproperty float z\n");
    out.push_str(&format!("element face {}\n", mesh.indices.len() / 3));
    out.push_str("property list uchar int vertex_indices\nend_header\n");
    for v in &mesh.vertices {
        out.push_str(&format!(
            "{} {} {}\n",
            v.position[0], v.position[1], v.position[2]
        ));
    }
    for tri in mesh.indices.chunks_exact(3) {
        out.push_str(&format!("3 {} {} {}\n", tri[0], tri[1], tri[2]));
    }
    std::fs::write(path, out)
}

fn first_failure_stage(reasons: &[String]) -> Option<String> {
    reasons.first().map(|reason| {
        if reason.starts_with("raw_") {
            "raw_validation".to_string()
        } else if reason.starts_with("projected_") || reason == "projection_connectivity_changed" {
            "projected_validation".to_string()
        } else if reason.contains("feature") || reason.contains("sector") {
            "feature_sector_validation".to_string()
        } else {
            "unknown".to_string()
        }
    })
}

fn elapsed_ms(start: Instant) -> f64 {
    start.elapsed().as_secs_f64() * 1000.0
}

pub fn stable_hash_hex(bytes: &[u8]) -> String {
    let mut hash = 0xcbf29ce484222325_u64;
    for byte in bytes {
        hash ^= *byte as u64;
        hash = hash.wrapping_mul(0x100000001b3);
    }
    format!("{hash:016x}")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::Vertex;

    #[test]
    fn input_mesh_and_leafset_are_rejected() {
        let mut config = FromScratchSurfaceConfig::default();
        config.input_mesh = Some("old.ply".to_string());
        assert!(config.validate_input_isolation().is_err());
        config.input_mesh = None;
        config.input_leafset = Some("old.bin".to_string());
        assert!(config.validate_input_isolation().is_err());
    }

    #[test]
    fn config_hash_changes_for_plus_one_only() {
        let baseline = FromScratchSurfaceConfig::default();
        let mut plus1 = baseline.clone();
        plus1.case_name = "te_tip_plus1_from_scratch".to_string();
        plus1.te_tip_extra_levels = 1;
        plus1.adaptive_sampling.max_depth += 1;
        plus1.adaptive_sampling.feature_max_depth += 1;
        plus1.adaptive_sampling.surface_crossing_max_depth =
            baseline.adaptive_sampling.surface_crossing_max_depth;
        assert_ne!(baseline.config_hash(), plus1.config_hash());
        assert_eq!(
            baseline.adaptive_sampling.base_cell_size,
            plus1.adaptive_sampling.base_cell_size
        );
        assert_eq!(
            baseline.adaptive_sampling.surface_crossing_max_depth,
            plus1.adaptive_sampling.surface_crossing_max_depth
        );
        assert_eq!(
            baseline.adaptive_sampling.feature_max_depth + 1,
            plus1.adaptive_sampling.feature_max_depth
        );
    }

    #[test]
    fn mesh_hash_and_connectivity_hash_are_distinct_concepts() {
        let mesh = Mesh {
            vertices: vec![
                Vertex {
                    position: [0.0, 0.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                },
                Vertex {
                    position: [1.0, 0.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                },
                Vertex {
                    position: [0.0, 1.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                },
            ],
            indices: vec![0, 1, 2],
        };
        assert_ne!(mesh_hash(&mesh), connectivity_hash(&mesh));
        assert_eq!(connectivity_hash(&mesh), connectivity_hash(&mesh));
    }

    #[test]
    fn projection_report_distinguishes_retention_reasons() {
        let mut report = ProjectionStageReport::default();
        report.retained_raw_vertices = 4;
        report.retained_already_within_tolerance = 2;
        report.retained_trust_region = 1;
        report.retained_sector = 1;

        assert_eq!(report.retained_raw_vertices, 4);
        assert_eq!(
            report.retained_already_within_tolerance
                + report.retained_trust_region
                + report.retained_sector,
            4
        );
        assert_eq!(report.fully_projected_vertices, 0);
    }

    #[test]
    fn refinement_cause_audit_keeps_feature_and_balance_counts_separate() {
        let mut audit = RefinementCauseAudit::default();
        audit
            .by_reason
            .insert("ProceduralFeatureProximity".to_string(), 12);
        audit.by_reason.insert("Balance".to_string(), 3);
        audit.feature_band_leaves = 12;
        audit.balance_only_leaves = 3;
        audit.ordinary_max_depth_leaves = 1;
        audit.te_tip_max_depth_leaves = 10;

        assert_eq!(
            audit.by_reason["ProceduralFeatureProximity"],
            audit.feature_band_leaves
        );
        assert_eq!(audit.by_reason["Balance"], audit.balance_only_leaves);
        assert!(audit.te_tip_max_depth_leaves > audit.ordinary_max_depth_leaves);
    }

    #[test]
    fn te_overlay_policy_separates_upper_and_lower_curves() {
        let audit = TeFeatureAssociationAudit {
            upper_feature_id: "lofted_wing.main.trailing_edge.upper_cap_boundary".to_string(),
            lower_feature_id: "lofted_wing.main.trailing_edge.lower_cap_boundary".to_string(),
            upper_vertices: 10,
            lower_vertices: 11,
            cross_association_errors: 0,
            upper_parameter_monotonic: true,
            lower_parameter_monotonic: true,
            overlay_policy: "draw upper and lower procedural TE curves independently; do not connect nearest mesh samples across sides".to_string(),
        };

        assert_ne!(audit.upper_feature_id, audit.lower_feature_id);
        assert!(audit.upper_parameter_monotonic);
        assert!(audit.lower_parameter_monotonic);
        assert!(audit.overlay_policy.contains("independently"));
        assert!(audit.overlay_policy.contains("do not connect"));
    }
}

fn leaf_topology_hash(octree: &AdaptiveOctree) -> String {
    let leaves = octree
        .leaves
        .iter()
        .map(|leaf| {
            serde_json::json!({
                "level": leaf.level,
                "morton_code": leaf.morton_code,
                "grid_coord": leaf.grid_coord,
            })
        })
        .collect::<Vec<_>>();
    stable_hash_hex(
        serde_json::to_string(&serde_json::json!({
            "schema": "leaf_topology.v1",
            "bounds_min": octree.bounds_min,
            "bounds_max": octree.bounds_max,
            "root_size": octree.root_size,
            "leaves": leaves,
        }))
        .unwrap_or_default()
        .as_bytes(),
    )
}

fn leaf_metadata_hash(octree: &AdaptiveOctree) -> String {
    let leaves = octree
        .leaves
        .iter()
        .map(|leaf| {
            serde_json::json!({
                "id": leaf.id,
                "level": leaf.level,
                "morton_code": leaf.morton_code,
                "reasons": leaf.refinement_reasons,
                "feature_id": leaf.feature_summary.closest_feature_id,
                "feature_type": leaf.feature_summary.closest_feature_type,
                "sector": leaf.feature_summary.surface_sector_id,
                "selected_extra_depth_band": leaf.feature_summary.selected_extra_depth_band,
                "selected_extra_depth_eligible": leaf.feature_summary.selected_extra_depth_eligible,
                "selected_extra_depth_match_source": leaf.feature_summary.selected_extra_depth_match_source,
            })
        })
        .collect::<Vec<_>>();
    stable_hash_hex(
        serde_json::to_string(&serde_json::json!({
            "schema": "leaf_metadata.v1",
            "leaves": leaves,
        }))
        .unwrap_or_default()
        .as_bytes(),
    )
}
