use crate::mesh::sdf_surface::{
    SdfSurfaceMesh, SurfaceExtractionConfig, SurfaceMeshAudit, SurfaceQueryCounts,
    audit_surface_mesh, rebuild_projected_surface_mesh,
};
use crate::sdf::Sdf;
use crate::sdf::procedural_features::{
    CapRole, ProceduralFeatureQueryOptions, ProceduralFeatureQueryResult, ProceduralFeatureType,
};
use glam::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;
use std::time::Instant;

pub const SURFACE_FEATURE_RESOLUTION_SCHEMA_VERSION: &str = "surface_feature_resolution.v8";
pub const SURFACE_FEATURE_REFINER_VERSION: &str =
    "procedural_feature_band_conformal_edge_subdivision.v1";

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct FeatureResolutionConfig {
    pub global_target_edge_length: f64,
    pub feature_target_edge_length: f64,
    pub feature_band_width: f64,
    pub trailing_edge_thickness: f64,
    pub split_threshold_factor: f64,
    pub max_refinement_passes: usize,
    pub projection_tolerance: f64,
    pub projection_max_iterations: usize,
    pub max_projection_displacement_factor: f64,
    pub feature_snap_tolerance: f64,
    pub refine_trailing_edge: bool,
    pub refine_tip_transitions: bool,
    pub refine_cap_sectors: bool,
}

impl Default for FeatureResolutionConfig {
    fn default() -> Self {
        Self {
            global_target_edge_length: 8.0,
            feature_target_edge_length: 2.0,
            feature_band_width: 8.0,
            trailing_edge_thickness: 1.0,
            split_threshold_factor: 1.15,
            max_refinement_passes: 4,
            projection_tolerance: 5.0e-3,
            projection_max_iterations: 32,
            max_projection_displacement_factor: 5.0,
            feature_snap_tolerance: 1.25,
            refine_trailing_edge: true,
            refine_tip_transitions: true,
            refine_cap_sectors: true,
        }
    }
}

impl FeatureResolutionConfig {
    pub fn surface_extraction_config(&self) -> SurfaceExtractionConfig {
        SurfaceExtractionConfig {
            target_spacing: self.feature_target_edge_length,
            min_spacing: self.feature_target_edge_length * 0.35,
            max_spacing: self.global_target_edge_length * 1.75,
            resolution_override: Some(0),
            bounds_padding: self.global_target_edge_length,
            weld_tolerance: 1.0e-7,
            projection_tolerance: self.projection_tolerance,
            projection_max_iterations: self.projection_max_iterations,
            max_projection_displacement_factor: self.max_projection_displacement_factor,
            min_gradient_magnitude: 1.0e-8,
            require_closed: true,
            require_manifold: true,
            sharp_dihedral_degrees: 45.0,
        }
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct FeatureResolutionOutcome {
    pub schema_version: String,
    pub refiner_version: String,
    pub input_hash: String,
    pub mesh: SdfSurfaceMesh,
    pub mesh_hash: String,
    pub audit: SurfaceMeshAudit,
    pub query_counts: SurfaceQueryCounts,
    pub passes: Vec<FeatureRefinementPass>,
    pub operation_counts: FeatureRefinementCounts,
    pub timings: FeatureResolutionTimings,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct FeatureRefinementPass {
    pub pass_index: usize,
    pub split_edges: usize,
    pub vertices_before: usize,
    pub vertices_after: usize,
    pub triangles_before: usize,
    pub triangles_after: usize,
    pub mesh_hash: String,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct FeatureRefinementCounts {
    pub candidate_edges: usize,
    pub accepted_edge_splits: usize,
    pub feature_snapped_vertices: usize,
    pub midpoint_vertices: usize,
    pub rejected_edges: BTreeMap<String, usize>,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct FeatureResolutionTimings {
    pub sizing_eval_ms: f64,
    pub subdivision_ms: f64,
    pub projection_rebuild_ms: f64,
    pub validation_ms: f64,
    pub total_ms: f64,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct FeatureResolutionMetrics {
    pub schema_version: String,
    pub mesh_hash: String,
    pub finite_trailing_edge_thickness: f64,
    pub regions: Vec<FeatureRegionResolution>,
    pub topology: serde_json::Value,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct FeatureRegionResolution {
    pub region_id: String,
    pub vertex_count: usize,
    pub edge_count: usize,
    pub triangle_count: usize,
    pub local_target_edge_length: f64,
    pub edge_length_min: f64,
    pub edge_length_p05: f64,
    pub edge_length_p50: f64,
    pub edge_length_p90: f64,
    pub edge_length_p95: f64,
    pub edge_length_max: f64,
    pub finite_local_thickness: Option<f64>,
    pub estimated_intervals_across_thickness_p50: Option<f64>,
    pub estimated_intervals_across_thickness_p95: Option<f64>,
    pub min_angle_p05: f64,
    pub min_angle_p50: f64,
    pub aspect_ratio_p95: f64,
    pub feature_deviation_rms: Option<f64>,
    pub feature_deviation_max: Option<f64>,
    pub feature_parameter_spacing_p50: Option<f64>,
    pub feature_parameter_spacing_max: Option<f64>,
    pub normal_oscillation_rms_degrees: Option<f64>,
    pub normal_oscillation_max_degrees: Option<f64>,
    pub roughness_second_difference_rms: Option<f64>,
    pub roughness_second_difference_max: Option<f64>,
    pub projection_displacement_max: f64,
    pub sector_assignment_counts: BTreeMap<String, usize>,
}

#[derive(Clone, Debug)]
struct VertexFeatureSample {
    query: ProceduralFeatureQueryResult,
    target: f64,
    in_refinement_band: bool,
}

pub fn refine_feature_bands(
    sdf: Arc<dyn Sdf>,
    input: &SdfSurfaceMesh,
    config: &FeatureResolutionConfig,
) -> Result<FeatureResolutionOutcome, String> {
    let total_start = Instant::now();
    let mut timings = FeatureResolutionTimings::default();
    let mut counts = FeatureRefinementCounts::default();
    let mut positions = input
        .vertices
        .iter()
        .map(|v| {
            Vec3::new(
                v.position[0] as f32,
                v.position[1] as f32,
                v.position[2] as f32,
            )
        })
        .collect::<Vec<_>>();
    let mut triangles = input
        .triangles
        .iter()
        .map(|t| t.vertices)
        .collect::<Vec<_>>();
    let mut passes = Vec::new();
    let mut total_query_counts = SurfaceQueryCounts::default();
    let options = ProceduralFeatureQueryOptions {
        tolerance: config.feature_band_width,
        ..ProceduralFeatureQueryOptions::diagnostic()
    };

    for pass_index in 0..config.max_refinement_passes {
        let sizing_start = Instant::now();
        let samples = positions
            .iter()
            .map(|p| {
                let query = sdf.query_procedural_feature(*p, &options);
                let target = target_size_for_query(&query, config);
                let in_refinement_band = target < config.global_target_edge_length;
                VertexFeatureSample {
                    query,
                    target,
                    in_refinement_band,
                }
            })
            .collect::<Vec<_>>();
        timings.sizing_eval_ms += elapsed_ms(sizing_start);

        let edges = collect_edges(&triangles);
        let mut split_edges = BTreeMap::<(u64, u64), u64>::new();
        let mut split_positions = Vec::<(u64, Vec3, bool)>::new();
        for (a, b) in edges {
            let pa = positions[a as usize];
            let pb = positions[b as usize];
            let length = (pb - pa).length() as f64;
            let edge_target = samples[a as usize].target.min(samples[b as usize].target);
            let midpoint = (pa + pb) * 0.5;
            let midpoint_query = sdf.query_procedural_feature(midpoint, &options);
            let midpoint_target = target_size_for_query(&midpoint_query, config);
            let target = edge_target.min(midpoint_target);
            let in_band = samples[a as usize].in_refinement_band
                || samples[b as usize].in_refinement_band
                || target < config.global_target_edge_length;
            if !in_band {
                continue;
            }
            counts.candidate_edges += 1;
            if length <= target * config.split_threshold_factor {
                *counts
                    .rejected_edges
                    .entry("inside_target_band".to_string())
                    .or_default() += 1;
                continue;
            }
            let key = sorted_edge(a, b);
            let new_id = (positions.len() + split_positions.len()) as u64;
            let (pos, snapped) = split_position(
                sdf.as_ref(),
                pa,
                pb,
                &samples[a as usize].query,
                &samples[b as usize].query,
                config,
            );
            split_edges.insert(key, new_id);
            split_positions.push((new_id, pos, snapped));
        }
        if split_edges.is_empty() {
            break;
        }

        let subdiv_start = Instant::now();
        let before_vertices = positions.len();
        let before_triangles = triangles.len();
        for (_, pos, snapped) in &split_positions {
            positions.push(*pos);
            if *snapped {
                counts.feature_snapped_vertices += 1;
            } else {
                counts.midpoint_vertices += 1;
            }
        }
        triangles = subdivide_triangles(&triangles, &split_edges);
        counts.accepted_edge_splits += split_edges.len();
        timings.subdivision_ms += elapsed_ms(subdiv_start);

        let rebuild_start = Instant::now();
        let (rebuilt, _audit, query_counts) = rebuild_projected_surface_mesh(
            Arc::clone(&sdf),
            &positions,
            &triangles,
            &config.surface_extraction_config(),
            SURFACE_FEATURE_REFINER_VERSION,
        )?;
        timings.projection_rebuild_ms += elapsed_ms(rebuild_start);
        add_query_counts(&mut total_query_counts, &query_counts);
        positions = rebuilt
            .vertices
            .iter()
            .map(|v| {
                Vec3::new(
                    v.position[0] as f32,
                    v.position[1] as f32,
                    v.position[2] as f32,
                )
            })
            .collect();
        triangles = rebuilt.triangles.iter().map(|t| t.vertices).collect();
        passes.push(FeatureRefinementPass {
            pass_index,
            split_edges: split_edges.len(),
            vertices_before: before_vertices,
            vertices_after: positions.len(),
            triangles_before: before_triangles,
            triangles_after: triangles.len(),
            mesh_hash: rebuilt.mesh_hash(),
        });
    }

    let rebuild_start = Instant::now();
    let (mesh, audit, query_counts) = rebuild_projected_surface_mesh(
        Arc::clone(&sdf),
        &positions,
        &triangles,
        &config.surface_extraction_config(),
        SURFACE_FEATURE_REFINER_VERSION,
    )?;
    timings.projection_rebuild_ms += elapsed_ms(rebuild_start);
    add_query_counts(&mut total_query_counts, &query_counts);
    let validation_start = Instant::now();
    validate_closed_surface(&audit)?;
    timings.validation_ms += elapsed_ms(validation_start);
    timings.total_ms = elapsed_ms(total_start);
    let mesh_hash = mesh.mesh_hash();
    Ok(FeatureResolutionOutcome {
        schema_version: SURFACE_FEATURE_RESOLUTION_SCHEMA_VERSION.to_string(),
        refiner_version: SURFACE_FEATURE_REFINER_VERSION.to_string(),
        input_hash: input.mesh_hash(),
        mesh,
        mesh_hash,
        audit,
        query_counts: total_query_counts,
        passes,
        operation_counts: counts,
        timings,
    })
}

pub fn measure_feature_resolution(
    sdf: &dyn Sdf,
    mesh: &SdfSurfaceMesh,
    config: &FeatureResolutionConfig,
) -> FeatureResolutionMetrics {
    let options = ProceduralFeatureQueryOptions {
        tolerance: config.feature_band_width,
        ..ProceduralFeatureQueryOptions::diagnostic()
    };
    let samples = mesh
        .vertices
        .iter()
        .map(|v| {
            let p = Vec3::new(
                v.position[0] as f32,
                v.position[1] as f32,
                v.position[2] as f32,
            );
            sdf.query_procedural_feature(p, &options)
        })
        .collect::<Vec<_>>();
    let regions = [
        "upper_trailing_edge_cap_boundary",
        "lower_trailing_edge_cap_boundary",
        "trailing_edge_cap_sector",
        "positive_tip_upper_transition",
        "positive_tip_lower_transition",
        "negative_tip_upper_transition",
        "negative_tip_lower_transition",
        "rounded_end_cap_perimeter",
        "root_section_guide",
        "leading_edge_comparison",
    ]
    .into_iter()
    .map(|id| measure_region(id, sdf, mesh, &samples, config))
    .collect::<Vec<_>>();
    FeatureResolutionMetrics {
        schema_version: SURFACE_FEATURE_RESOLUTION_SCHEMA_VERSION.to_string(),
        mesh_hash: mesh.mesh_hash(),
        finite_trailing_edge_thickness: config.trailing_edge_thickness,
        regions,
        topology: serde_json::json!(audit_surface_mesh(mesh).topology),
    }
}

pub fn target_size_for_query(
    query: &ProceduralFeatureQueryResult,
    config: &FeatureResolutionConfig,
) -> f64 {
    let mut target = config.global_target_edge_length;
    let in_distance_band = query
        .distance_to_feature
        .is_some_and(|d| d <= config.feature_band_width);
    let feature_id = query.feature_id.as_deref().unwrap_or_default();
    let cap_role = query.cap_role.unwrap_or(CapRole::Unknown);
    let trailing = matches!(
        query.feature_type,
        ProceduralFeatureType::TrailingEdgeUpperCapBoundary
            | ProceduralFeatureType::TrailingEdgeLowerCapBoundary
            | ProceduralFeatureType::TrailingEdgeCapCenter
            | ProceduralFeatureType::TrailingEdgeCapSector
    ) || feature_id.contains("trailing_edge")
        || cap_role == CapRole::TrailingEdgeCap;
    let tip = matches!(
        query.feature_type,
        ProceduralFeatureType::TipUpperCapBoundary
            | ProceduralFeatureType::TipLowerCapBoundary
            | ProceduralFeatureType::TipPerimeter
            | ProceduralFeatureType::TipCapSector
    ) || feature_id.contains(".tip.")
        || matches!(cap_role, CapRole::TipCapNegative | CapRole::TipCapPositive);
    if config.refine_trailing_edge && trailing && in_distance_band {
        target = target.min(config.feature_target_edge_length);
    }
    if config.refine_tip_transitions && tip && in_distance_band {
        target = target.min(config.feature_target_edge_length);
    }
    if config.refine_cap_sectors
        && matches!(
            cap_role,
            CapRole::TrailingEdgeCap | CapRole::TipCapNegative | CapRole::TipCapPositive
        )
    {
        target = target.min(config.feature_target_edge_length);
    }
    target
}

fn measure_region(
    region_id: &str,
    sdf: &dyn Sdf,
    mesh: &SdfSurfaceMesh,
    samples: &[ProceduralFeatureQueryResult],
    config: &FeatureResolutionConfig,
) -> FeatureRegionResolution {
    let mut vertex_ids = BTreeSet::<u64>::new();
    let mut triangle_ids = BTreeSet::<u64>::new();
    for tri in &mesh.triangles {
        let hit = tri
            .vertices
            .iter()
            .any(|id| sample_in_region(region_id, &samples[*id as usize], config));
        if hit {
            triangle_ids.insert(tri.id);
            for id in tri.vertices {
                vertex_ids.insert(id);
            }
        }
    }
    let mut edges = Vec::<f64>::new();
    for edge in &mesh.edges {
        if vertex_ids.contains(&edge.vertices[0]) && vertex_ids.contains(&edge.vertices[1]) {
            let a = mesh.vertices[edge.vertices[0] as usize].position;
            let b = mesh.vertices[edge.vertices[1] as usize].position;
            edges.push(distance(a, b));
        }
    }
    edges.sort_by(|a, b| a.total_cmp(b));
    let mut angles = Vec::new();
    let mut aspects = Vec::new();
    for id in &triangle_ids {
        if let Some(tri) = mesh.triangles.get(*id as usize) {
            angles.push(tri.quality.min_angle_degrees);
            aspects.push(tri.quality.aspect_ratio);
        }
    }
    angles.sort_by(|a, b| a.total_cmp(b));
    aspects.sort_by(|a, b| a.total_cmp(b));
    let mut deviations = Vec::new();
    let mut params = Vec::new();
    let mut sector_counts = BTreeMap::new();
    let mut projection_max = 0.0_f64;
    let mut normals = Vec::<(f64, [f64; 3])>::new();
    let mut rough_points = Vec::<(f64, Vec3)>::new();
    for id in &vertex_ids {
        let sample = &samples[*id as usize];
        if let Some(d) = sample.distance_to_feature {
            deviations.push(d);
        }
        let param = sample
            .span_parameter
            .or(sample.chord_parameter)
            .or(sample.airfoil_parameter);
        if let Some(p) = param {
            params.push(p);
            if let Some(normal) = mesh.vertices[*id as usize].unit_normal {
                normals.push((p, normal));
            }
            rough_points.push((
                p,
                Vec3::new(
                    mesh.vertices[*id as usize].position[0] as f32,
                    mesh.vertices[*id as usize].position[1] as f32,
                    mesh.vertices[*id as usize].position[2] as f32,
                ),
            ));
        }
        let sector = sample
            .normal_sector
            .as_ref()
            .map(|s| s.stable_id.clone())
            .unwrap_or_else(|| format!("{:?}", sample.side));
        *sector_counts.entry(sector).or_default() += 1;
        projection_max = projection_max.max(
            mesh.vertices[*id as usize]
                .projection_status
                .displacement_magnitude
                .unwrap_or(0.0),
        );
    }
    deviations.sort_by(|a, b| a.total_cmp(b));
    params.sort_by(|a, b| a.total_cmp(b));
    let param_gaps = params.windows(2).map(|w| w[1] - w[0]).collect::<Vec<_>>();
    let (normal_rms, normal_max) = normal_oscillation(&normals);
    let (rough_rms, rough_max) = roughness_second_difference(sdf, &rough_points);
    let p50_edge = percentile(&edges, 0.50);
    let p95_edge = percentile(&edges, 0.95);
    FeatureRegionResolution {
        region_id: region_id.to_string(),
        vertex_count: vertex_ids.len(),
        edge_count: edges.len(),
        triangle_count: triangle_ids.len(),
        local_target_edge_length: if region_id == "leading_edge_comparison"
            || region_id == "root_section_guide"
        {
            config.global_target_edge_length
        } else {
            config.feature_target_edge_length
        },
        edge_length_min: percentile(&edges, 0.0),
        edge_length_p05: percentile(&edges, 0.05),
        edge_length_p50: p50_edge,
        edge_length_p90: percentile(&edges, 0.90),
        edge_length_p95: p95_edge,
        edge_length_max: percentile(&edges, 1.0),
        finite_local_thickness: Some(config.trailing_edge_thickness),
        estimated_intervals_across_thickness_p50: (p50_edge > 0.0)
            .then_some(config.trailing_edge_thickness / p50_edge),
        estimated_intervals_across_thickness_p95: (p95_edge > 0.0)
            .then_some(config.trailing_edge_thickness / p95_edge),
        min_angle_p05: percentile(&angles, 0.05),
        min_angle_p50: percentile(&angles, 0.50),
        aspect_ratio_p95: percentile(&aspects, 0.95),
        feature_deviation_rms: rms(&deviations),
        feature_deviation_max: deviations.last().copied(),
        feature_parameter_spacing_p50: Some(percentile(&param_gaps, 0.50)),
        feature_parameter_spacing_max: param_gaps.iter().copied().max_by(|a, b| a.total_cmp(b)),
        normal_oscillation_rms_degrees: normal_rms,
        normal_oscillation_max_degrees: normal_max,
        roughness_second_difference_rms: rough_rms,
        roughness_second_difference_max: rough_max,
        projection_displacement_max: projection_max,
        sector_assignment_counts: sector_counts,
    }
}

fn sample_in_region(
    region_id: &str,
    sample: &ProceduralFeatureQueryResult,
    config: &FeatureResolutionConfig,
) -> bool {
    let feature_id = sample.feature_id.as_deref().unwrap_or_default();
    let in_band = sample
        .distance_to_feature
        .is_some_and(|d| d <= config.feature_band_width);
    match region_id {
        "upper_trailing_edge_cap_boundary" => {
            in_band && feature_id.contains("trailing_edge.upper_cap_boundary")
        }
        "lower_trailing_edge_cap_boundary" => {
            in_band && feature_id.contains("trailing_edge.lower_cap_boundary")
        }
        "trailing_edge_cap_sector" => sample.cap_role == Some(CapRole::TrailingEdgeCap),
        "positive_tip_upper_transition" => {
            in_band && feature_id.contains("tip.positive.upper_transition")
        }
        "positive_tip_lower_transition" => {
            in_band && feature_id.contains("tip.positive.lower_transition")
        }
        "negative_tip_upper_transition" => {
            in_band && feature_id.contains("tip.negative.upper_transition")
        }
        "negative_tip_lower_transition" => {
            in_band && feature_id.contains("tip.negative.lower_transition")
        }
        "rounded_end_cap_perimeter" => {
            matches!(
                sample.cap_role,
                Some(CapRole::TipCapNegative | CapRole::TipCapPositive)
            ) || feature_id.contains(".tip.")
        }
        "root_section_guide" => in_band && feature_id.contains("root.section"),
        "leading_edge_comparison" => in_band && feature_id.contains("leading_edge"),
        _ => false,
    }
}

fn split_position(
    sdf: &dyn Sdf,
    pa: Vec3,
    pb: Vec3,
    qa: &ProceduralFeatureQueryResult,
    qb: &ProceduralFeatureQueryResult,
    config: &FeatureResolutionConfig,
) -> (Vec3, bool) {
    let midpoint = (pa + pb) * 0.5;
    let same_feature = qa.feature_id.is_some()
        && qa.feature_id == qb.feature_id
        && qa
            .distance_to_feature
            .is_some_and(|d| d <= config.feature_snap_tolerance)
        && qb
            .distance_to_feature
            .is_some_and(|d| d <= config.feature_snap_tolerance);
    if same_feature {
        let feature_id = qa.feature_id.as_deref().unwrap_or_default();
        let param_a = qa
            .span_parameter
            .or(qa.chord_parameter)
            .or(qa.airfoil_parameter);
        let param_b = qb
            .span_parameter
            .or(qb.chord_parameter)
            .or(qb.airfoil_parameter);
        if let (Some(a), Some(b)) = (param_a, param_b) {
            let p = 0.5 * (a + b);
            if let Some(eval) = sdf.evaluate_procedural_feature(feature_id, p) {
                if let Some(point) = eval.closest_point {
                    return (
                        Vec3::new(point[0] as f32, point[1] as f32, point[2] as f32),
                        true,
                    );
                }
            }
        }
    }
    (midpoint, false)
}

fn subdivide_triangles(
    triangles: &[[u64; 3]],
    split_edges: &BTreeMap<(u64, u64), u64>,
) -> Vec<[u64; 3]> {
    let mut out = Vec::with_capacity(triangles.len() + split_edges.len() * 2);
    for tri in triangles {
        let [a, b, c] = *tri;
        let ab = split_edges.get(&sorted_edge(a, b)).copied();
        let bc = split_edges.get(&sorted_edge(b, c)).copied();
        let ca = split_edges.get(&sorted_edge(c, a)).copied();
        match (ab, bc, ca) {
            (None, None, None) => out.push([a, b, c]),
            (Some(m), None, None) => {
                out.push([a, m, c]);
                out.push([m, b, c]);
            }
            (None, Some(m), None) => {
                out.push([b, m, a]);
                out.push([m, c, a]);
            }
            (None, None, Some(m)) => {
                out.push([c, m, b]);
                out.push([m, a, b]);
            }
            (Some(mab), Some(mbc), None) => {
                out.push([b, mbc, mab]);
                out.push([a, mab, c]);
                out.push([mab, mbc, c]);
            }
            (None, Some(mbc), Some(mca)) => {
                out.push([c, mca, mbc]);
                out.push([b, mbc, a]);
                out.push([mbc, mca, a]);
            }
            (Some(mab), None, Some(mca)) => {
                out.push([a, mab, mca]);
                out.push([c, mca, b]);
                out.push([mca, mab, b]);
            }
            (Some(mab), Some(mbc), Some(mca)) => {
                out.push([a, mab, mca]);
                out.push([mab, b, mbc]);
                out.push([mca, mbc, c]);
                out.push([mab, mbc, mca]);
            }
        }
    }
    out
}

fn collect_edges(triangles: &[[u64; 3]]) -> BTreeSet<(u64, u64)> {
    let mut edges = BTreeSet::new();
    for tri in triangles {
        let [a, b, c] = *tri;
        edges.insert(sorted_edge(a, b));
        edges.insert(sorted_edge(b, c));
        edges.insert(sorted_edge(c, a));
    }
    edges
}

fn validate_closed_surface(audit: &SurfaceMeshAudit) -> Result<(), String> {
    if audit.topology.boundary_edges != 0 {
        return Err(format!("boundary edges: {}", audit.topology.boundary_edges));
    }
    if audit.topology.non_manifold_edges != 0 {
        return Err(format!(
            "non-manifold edges: {}",
            audit.topology.non_manifold_edges
        ));
    }
    if audit.topology.duplicate_triangles != 0 || audit.topology.zero_area_triangles != 0 {
        return Err("duplicate or degenerate triangles after feature refinement".to_string());
    }
    if audit.geometry.projection_failures != 0 {
        return Err(format!(
            "projection failures: {}",
            audit.geometry.projection_failures
        ));
    }
    Ok(())
}

fn normal_oscillation(normals: &[(f64, [f64; 3])]) -> (Option<f64>, Option<f64>) {
    if normals.len() < 2 {
        return (None, None);
    }
    let mut sorted = normals.to_vec();
    sorted.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut angles = Vec::new();
    for pair in sorted.windows(2) {
        let a = Vec3::new(
            pair[0].1[0] as f32,
            pair[0].1[1] as f32,
            pair[0].1[2] as f32,
        );
        let b = Vec3::new(
            pair[1].1[0] as f32,
            pair[1].1[1] as f32,
            pair[1].1[2] as f32,
        );
        let dot = a
            .normalize_or_zero()
            .dot(b.normalize_or_zero())
            .clamp(-1.0, 1.0);
        angles.push(dot.acos().to_degrees() as f64);
    }
    (
        rms(&angles),
        angles.into_iter().max_by(|a, b| a.total_cmp(b)),
    )
}

fn roughness_second_difference(
    _sdf: &dyn Sdf,
    points: &[(f64, Vec3)],
) -> (Option<f64>, Option<f64>) {
    if points.len() < 3 {
        return (None, None);
    }
    let mut sorted = points.to_vec();
    sorted.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut values = Vec::new();
    for win in sorted.windows(3) {
        let d = (win[0].1 - win[1].1 * 2.0 + win[2].1).length() as f64;
        values.push(d);
    }
    (
        rms(&values),
        values.into_iter().max_by(|a, b| a.total_cmp(b)),
    )
}

fn rms(values: &[f64]) -> Option<f64> {
    if values.is_empty() {
        return None;
    }
    Some((values.iter().map(|v| v * v).sum::<f64>() / values.len() as f64).sqrt())
}

fn percentile(values: &[f64], p: f64) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let idx = ((values.len() - 1) as f64 * p.clamp(0.0, 1.0)).round() as usize;
    values[idx]
}

fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn sorted_edge(a: u64, b: u64) -> (u64, u64) {
    if a <= b { (a, b) } else { (b, a) }
}

fn add_query_counts(total: &mut SurfaceQueryCounts, next: &SurfaceQueryCounts) {
    total.distance_queries += next.distance_queries;
    total.gradient_queries += next.gradient_queries;
    total.projection_queries += next.projection_queries;
    total.provenance_queries += next.provenance_queries;
}

fn elapsed_ms(start: Instant) -> f64 {
    start.elapsed().as_secs_f64() * 1000.0
}
