use glam::Vec3;
use implicit_cad::mesh::adaptive_mc::extract_mesh_adaptive_local_regions;
use implicit_cad::mesh::adaptive_sampling::{
    AdaptiveOctree, AdaptiveOctreeNode, AdaptiveSamplingConfig, build_adaptive_octree,
};
use implicit_cad::mesh::sdf_surface::{
    SdfSurfaceMesh, SurfaceExtractionConfig, audit_surface_mesh, extract_sdf_surface_mesh,
    rebuild_projected_surface_mesh, write_ply,
};
use implicit_cad::mesh::surface_artifacts::{SurfaceArtifactConfig, diagnose_surface_artifacts};
use implicit_cad::mesh::surface_feature_resolution::{
    FeatureResolutionConfig, FeatureResolutionMetrics, measure_feature_resolution,
};
use implicit_cad::mesh::surface_remesh::{SurfaceRemeshConfig, remesh_sdf_surface};
use implicit_cad::scripting;
use implicit_cad::sdf::Sdf;
use implicit_cad::sdf::query::bounding_points;
use serde::Serialize;
use serde_json::json;
use std::collections::BTreeMap;
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::Arc;
use std::time::Instant;

const REPORT_JSON: &str = "research/sdf_cfd_mesher/reports/adaptive_resolution_policy_v12.json";
const REPORT_MD: &str = "research/sdf_cfd_mesher/reports/adaptive_resolution_policy_v12.md";
const EXP_DIR: &str = "research/sdf_cfd_mesher/experiments/adaptive_resolution_policy_v12";
const MESH_DIR: &str = "research/sdf_cfd_mesher/experiments/adaptive_resolution_policy_v12/meshes";
const OCTREE_DIR: &str =
    "research/sdf_cfd_mesher/experiments/adaptive_resolution_policy_v12/octree";
const SCREENSHOT_DIR: &str =
    "research/sdf_cfd_mesher/experiments/adaptive_resolution_policy_v12/screenshots";
const LOG_DIR: &str = "research/sdf_cfd_mesher/experiments/adaptive_resolution_policy_v12/logs";
const WING_SCRIPT: &str = r#"
wing_with_airfoil_export_safe("2412", 50.0, 50.0, 250.0, 0.0, 0.0, 0.0, 1.0, 1.0, true)
"#;

#[derive(Clone, Debug, Serialize)]
struct UniformGridAudit {
    configuration: SurfaceExtractionConfig,
    grid_dimensions: [u32; 3],
    grid_origin: [f64; 3],
    extraction_bounds_min: [f64; 3],
    extraction_bounds_max: [f64; 3],
    spacing_xyz: [f64; 3],
    effective_cell_size: f64,
    surface_crossing_cell_count: usize,
    total_cell_count: usize,
    total_grid_points: usize,
    seed_mesh_hash: String,
    accepted_uniform_hash: String,
    accepted_uniform_triangles: usize,
}

#[derive(Clone, Debug, Serialize)]
struct RegionDensity {
    region: String,
    leaf_count: usize,
    surface_leaf_count: usize,
    max_level_leaf_count: usize,
    min_leaf_size: f64,
    p50_leaf_size: f64,
    p95_leaf_size: f64,
    estimated_surface_area_proxy: f64,
    density_ratio_vs_uniform: f64,
    requested_te_intervals: f64,
    achieved_te_intervals_p50: Option<f64>,
}

#[derive(Clone, Debug, Serialize)]
struct VariantReport {
    name: String,
    requested_te_intervals: f64,
    configuration_hash: String,
    leaf_set_hash: String,
    output_mesh_hash: String,
    mesh_file_hash: String,
    configuration: AdaptiveSamplingConfig,
    required_depth: serde_json::Value,
    octree_summary: serde_json::Value,
    region_density: Vec<RegionDensity>,
    density_requirements: serde_json::Value,
    direct_extraction: serde_json::Value,
    screenshots: Vec<String>,
    status: String,
    failure_reasons: Vec<String>,
}

fn main() {
    if let Err(err) = run() {
        eprintln!("adaptive resolution policy v12 failed: {err}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    for dir in [
        "research/sdf_cfd_mesher/reports",
        EXP_DIR,
        MESH_DIR,
        OCTREE_DIR,
        SCREENSHOT_DIR,
        LOG_DIR,
    ] {
        fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }

    let total_start = Instant::now();
    let sdf = eval_sdf(WING_SCRIPT)?;
    let uniform_config = finite_wing_extraction();
    let uniform_grid = uniform_grid_audit(sdf.as_ref(), uniform_config.clone())?;

    eprintln!("adaptive_resolution_v12: running accepted uniform reference");
    let seed = extract_sdf_surface_mesh(Arc::clone(&sdf), uniform_config.clone())?.mesh;
    let remeshed = remesh_sdf_surface(Arc::clone(&sdf), seed.clone(), finite_wing_remesh_config())?;
    let uniform_mesh = remeshed.mesh;
    let uniform_path = PathBuf::from(MESH_DIR).join("uniform_reference.ply");
    write_ply(&uniform_mesh, &uniform_path).map_err(|e| e.to_string())?;
    if uniform_mesh.mesh_hash() != "10009dccfa69819e" {
        eprintln!(
            "adaptive_resolution_v12: accepted uniform hash changed: expected 10009dccfa69819e got {}",
            uniform_mesh.mesh_hash()
        );
    }
    let uniform_feature = measure_feature_resolution(
        sdf.as_ref(),
        &uniform_mesh,
        &feature_metrics_config(uniform_grid.effective_cell_size, 1.0),
    );
    let uniform_regions = region_baseline(&uniform_feature);

    let variants = [
        ("adaptive_floor_a", 1.0, 1.0, 7_u32, 2.5_f64, 1.25_f64),
        ("adaptive_floor_b", 2.0, 0.5, 7_u32, 3.5_f64, 1.75_f64),
        ("adaptive_floor_c", 4.0, 0.25, 7_u32, 4.5_f64, 2.25_f64),
    ];

    let mut reports = Vec::<VariantReport>::new();
    let variant_filter = std::env::var("IMPLICIT_CAD_ADAPTIVE_VARIANT_FILTER").ok();
    for (name, requested, feature_target, feature_depth, feature_band, le_band) in variants {
        if variant_filter
            .as_deref()
            .map(|filter| filter != name)
            .unwrap_or(false)
        {
            eprintln!("adaptive_resolution_v12: skipping {name} due to variant filter");
            continue;
        }
        eprintln!("adaptive_resolution_v12: building {name}");
        let config = adaptive_config(
            &uniform_grid,
            requested,
            feature_target,
            feature_depth,
            feature_band,
            le_band,
        );
        let report = run_variant(
            Arc::clone(&sdf),
            name,
            requested,
            config,
            &uniform_grid,
            &uniform_regions,
        )?;
        reports.push(report);
    }

    let render_manifest = write_render_manifest(&uniform_grid, &uniform_mesh, &reports)?;
    let render_status = render_screenshots(&render_manifest);
    attach_screenshot_paths(&mut reports);
    let report = json!({
        "schema_version": "adaptive_resolution_policy.v12",
        "input": {
            "accepted_uniform_reference_hash": "10009dccfa69819e",
            "current_uniform_reference_hash": uniform_mesh.mesh_hash(),
            "wing_script": WING_SCRIPT.trim(),
            "no_stl": true,
            "macro_shell_started": false
        },
        "uniform_extractor_configuration": uniform_grid,
        "correction": {
            "implemented": [
                "typed near-surface resolution floor in src/mesh/adaptive_sampling.rs",
                "NearSurfaceResolutionFloor refinement reason",
                "feature-target clamp removed so A/B/C retain distinct requested TE intervals",
                "v12 runner traces configuration -> sampler -> required depth -> leaf decisions -> leaf-set hash -> output mesh"
            ],
            "accepted_uniform_effective_cell_size": uniform_grid.effective_cell_size,
            "near_surface_floor_policy": "surface-crossing or near-zero leaves refine until leaf size is no coarser than the accepted uniform effective cell size; far empty leaves remain coarse",
            "feature_policy": "procedural feature bands request progressively smaller scalar targets around TE boundaries, TE cap sector, tip transitions, rounded tip, and leading-edge guide"
        },
        "variants": reports,
        "variant_identity": variant_identity(&reports),
        "visualizations": screenshot_paths(),
        "render_manifest": path_for_report(&render_manifest),
        "performance": {
            "total_runtime_ms": elapsed_ms(total_start),
            "render_status": render_status
        },
        "conclusion": conclusion(&reports),
        "recommended_next_task": recommended_next_task(&reports)
    });
    fs::write(
        REPORT_JSON,
        serde_json::to_string_pretty(&report).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    fs::write(REPORT_MD, markdown_summary(&report)).map_err(|e| e.to_string())?;
    write_density_csv(&report)?;
    Ok(())
}

fn run_variant(
    sdf: Arc<dyn Sdf>,
    name: &str,
    requested_te_intervals: f64,
    config: AdaptiveSamplingConfig,
    uniform: &UniformGridAudit,
    uniform_regions: &BTreeMap<String, RegionDensity>,
) -> Result<VariantReport, String> {
    let octree = build_adaptive_octree(sdf.as_ref(), &config)?;
    let leaf_hash = octree.octree_hash();
    let density_regions = density_by_region(&octree, requested_te_intervals, uniform_regions);
    let density_requirements = density_requirements(&density_regions, requested_te_intervals);
    write_octree_diagnostics(name, &octree)?;
    let required_depth = json!({
        "root_size": octree.root_size,
        "uniform_effective_cell_size": uniform.effective_cell_size,
        "near_surface_floor_depth": required_depth(octree.root_size, uniform.effective_cell_size),
        "feature_target_size": config.feature_target_size(),
        "feature_depth": required_depth(octree.root_size, config.feature_target_size() as f64),
        "max_depth": config.max_depth,
        "near_surface_floor_max_depth": config.near_surface_floor_max_depth,
        "feature_max_depth": config.feature_max_depth
    });

    eprintln!("adaptive_resolution_v12: diagnostic direct extraction {name}");
    let direct_start = Instant::now();
    let extraction_config =
        bounded_diagnostic_extraction_config(&config, uniform.effective_cell_size);
    let local_regions = octree.local_refine_regions(&extraction_config);
    let raw = extract_mesh_adaptive_local_regions(
        sdf.as_ref(),
        Vec3::new(
            octree.bounds_min[0] as f32,
            octree.bounds_min[1] as f32,
            octree.bounds_min[2] as f32,
        ),
        Vec3::new(
            octree.bounds_max[0] as f32,
            octree.bounds_max[1] as f32,
            octree.bounds_max[2] as f32,
        ),
        extraction_config.base_cell_size as f32,
        &local_regions,
        false,
    );
    let positions = raw
        .vertices
        .iter()
        .map(|v| Vec3::from_array(v.position))
        .collect::<Vec<_>>();
    let triangles = raw
        .indices
        .chunks_exact(3)
        .map(|tri| [tri[0] as u64, tri[1] as u64, tri[2] as u64])
        .collect::<Vec<_>>();
    let surface_config = adaptive_surface_config(&extraction_config);
    let (mesh, _projected_audit, query_counts) = rebuild_projected_surface_mesh(
        Arc::clone(&sdf),
        &positions,
        &triangles,
        &surface_config,
        "adaptive_resolution_policy_v12_bounded_direct_diagnostic",
    )?;
    let mesh_path = PathBuf::from(MESH_DIR).join(format!("{name}.ply"));
    write_ply(&mesh, &mesh_path).map_err(|e| e.to_string())?;
    let audit = audit_surface_mesh(&mesh);
    let artifacts = diagnose_surface_artifacts(
        Arc::clone(&sdf),
        &mesh,
        None,
        &SurfaceArtifactConfig::default(),
    );
    let mut failure_reasons = Vec::new();
    if audit.topology.boundary_edges > 0 {
        failure_reasons.push("boundary_edges_nonzero".to_string());
    }
    if audit.topology.non_manifold_edges > 0 {
        failure_reasons.push("non_manifold_edges_nonzero".to_string());
    }
    if audit.geometry.projection_failures > 0 {
        failure_reasons.push("projection_failures_nonzero".to_string());
    }
    failure_reasons.extend(artifacts.acceptance.failure_reasons.clone());
    failure_reasons.sort();
    failure_reasons.dedup();
    let status = if failure_reasons.is_empty() {
        "diagnostic_extraction_passed_topology_gates"
    } else {
        "diagnostic_extraction_failed_closed"
    };
    Ok(VariantReport {
        name: name.to_string(),
        requested_te_intervals,
        configuration_hash: config.config_hash(),
        leaf_set_hash: leaf_hash,
        output_mesh_hash: mesh.mesh_hash(),
        mesh_file_hash: file_hash(&mesh_path)?,
        configuration: config,
        required_depth,
        octree_summary: json!(octree.summary),
        region_density: density_regions,
        density_requirements,
        direct_extraction: json!({
            "status": status,
            "runtime_ms": elapsed_ms(direct_start),
            "mesh_path": path_for_report(&mesh_path),
            "bounded_diagnostic_config": extraction_config,
            "local_refine_regions": local_regions.len(),
            "query_counts": query_counts,
            "vertices": mesh.vertices.len(),
            "triangles": mesh.triangles.len(),
            "boundary_edges": audit.topology.boundary_edges,
            "non_manifold_edges": audit.topology.non_manifold_edges,
            "duplicate_triangles": audit.topology.duplicate_triangles,
            "degenerate_triangles": audit.topology.zero_area_triangles,
            "projection_failures": audit.geometry.projection_failures,
            "artifact_acceptance": artifacts.acceptance,
            "failure_reasons": failure_reasons,
            "note": "direct extraction is diagnostic only in v12; it uses the corrected leaf-set regions with a bounded target so this task does not become transition-connectivity work"
        }),
        screenshots: Vec::new(),
        status: status.to_string(),
        failure_reasons,
    })
}

fn adaptive_config(
    uniform: &UniformGridAudit,
    requested_te_intervals: f64,
    feature_target_size: f64,
    feature_depth: u32,
    feature_band_width: f64,
    leading_edge_band_width: f64,
) -> AdaptiveSamplingConfig {
    let floor_depth = required_depth(
        uniform.effective_cell_size * 52.0,
        uniform.effective_cell_size,
    )
    .max(6);
    AdaptiveSamplingConfig {
        min_depth: 2,
        max_depth: feature_depth.max(floor_depth),
        bounds_padding: uniform.configuration.bounds_padding,
        base_cell_size: uniform.effective_cell_size,
        near_surface_floor_cell_size: Some(uniform.effective_cell_size),
        near_surface_floor_band_factor: 1.1,
        near_surface_floor_max_depth: floor_depth,
        max_leaf_cells: 450_000,
        surface_band_factor: 1.1,
        surface_band_max_depth: floor_depth,
        complexity_narrow_band_factor: 0.75,
        complexity_max_depth: floor_depth,
        feature_query_band_factor: 1.2,
        feature_max_depth: feature_depth,
        normal_divergence_degrees: 38.0,
        interpolation_error_tolerance: 0.08,
        normal_interpolation_error_degrees: 32.0,
        required_feature_intervals: requested_te_intervals,
        feature_band_width,
        leading_edge_band_width,
        root_guide_band_width: leading_edge_band_width,
        trailing_edge_thickness: feature_target_size * requested_te_intervals,
        balance_face_neighbors: true,
        max_balance_passes: 5,
    }
}

fn bounded_diagnostic_extraction_config(
    config: &AdaptiveSamplingConfig,
    uniform_cell_size: f64,
) -> AdaptiveSamplingConfig {
    let requested = config.required_feature_intervals.max(1.0);
    let diagnostic_target = if requested <= 1.0 {
        uniform_cell_size
    } else if requested <= 2.0 {
        uniform_cell_size * 0.95
    } else {
        uniform_cell_size * 0.25
    };
    let mut bounded = config.clone();
    bounded.trailing_edge_thickness = diagnostic_target * requested;
    bounded.feature_max_depth = bounded.feature_max_depth.min(8);
    bounded.max_depth = bounded.max_depth.min(8);
    bounded.max_leaf_cells = bounded.max_leaf_cells.min(220_000);
    bounded
}

fn uniform_grid_audit(
    sdf: &dyn Sdf,
    config: SurfaceExtractionConfig,
) -> Result<UniformGridAudit, String> {
    let metadata = sdf.metadata();
    let bbox = if let Some(bounds) = metadata.support_bounds {
        if bounds.is_valid() && bounds.min.is_finite() && bounds.max.is_finite() {
            (bounds.min, bounds.max)
        } else {
            let b = bounding_points(sdf);
            (b.min, b.max)
        }
    } else {
        let b = bounding_points(sdf);
        (b.min, b.max)
    };
    let (domain_min, domain_max) = expand_degenerate_bounds(
        bbox.0,
        bbox.1,
        (config.target_spacing.max(config.bounds_padding) as f32).max(1.0),
    );
    let bounds_min = domain_min - Vec3::splat(config.bounds_padding as f32);
    let bounds_max = domain_max + Vec3::splat(config.bounds_padding as f32);
    let extent = bounds_max - bounds_min;
    let max_extent = extent.max_element().max(config.target_spacing as f32);
    let resolution = config
        .resolution_override
        .unwrap_or_else(|| (max_extent as f64 / config.target_spacing).ceil() as u32)
        .clamp(8, 96);
    let step = extent / resolution as f32;
    let mut values = Vec::<f32>::with_capacity(((resolution + 1).pow(3)) as usize);
    for z in 0..=resolution {
        for y in 0..=resolution {
            for x in 0..=resolution {
                let p =
                    bounds_min + Vec3::new(x as f32 * step.x, y as f32 * step.y, z as f32 * step.z);
                values.push(sdf.distance(p));
            }
        }
    }
    let idx = |x: u32, y: u32, z: u32| -> usize {
        (z * (resolution + 1) * (resolution + 1) + y * (resolution + 1) + x) as usize
    };
    let mut crossings = 0_usize;
    for z in 0..resolution {
        for y in 0..resolution {
            for x in 0..resolution {
                let corners = [
                    values[idx(x, y, z)],
                    values[idx(x + 1, y, z)],
                    values[idx(x, y + 1, z)],
                    values[idx(x + 1, y + 1, z)],
                    values[idx(x, y, z + 1)],
                    values[idx(x + 1, y, z + 1)],
                    values[idx(x, y + 1, z + 1)],
                    values[idx(x + 1, y + 1, z + 1)],
                ];
                let min = corners.iter().copied().fold(f32::INFINITY, f32::min);
                let max = corners.iter().copied().fold(f32::NEG_INFINITY, f32::max);
                if min <= 0.0 && max >= 0.0 {
                    crossings += 1;
                }
            }
        }
    }
    Ok(UniformGridAudit {
        configuration: config,
        grid_dimensions: [resolution, resolution, resolution],
        grid_origin: array_from_vec3(bounds_min),
        extraction_bounds_min: array_from_vec3(bounds_min),
        extraction_bounds_max: array_from_vec3(bounds_max),
        spacing_xyz: array_from_vec3(step),
        effective_cell_size: step.max_element() as f64,
        surface_crossing_cell_count: crossings,
        total_cell_count: resolution as usize * resolution as usize * resolution as usize,
        total_grid_points: (resolution as usize + 1).pow(3),
        seed_mesh_hash: "computed_in_run".to_string(),
        accepted_uniform_hash: "10009dccfa69819e".to_string(),
        accepted_uniform_triangles: 9468,
    })
}

fn density_by_region(
    octree: &AdaptiveOctree,
    requested_te_intervals: f64,
    uniform_regions: &BTreeMap<String, RegionDensity>,
) -> Vec<RegionDensity> {
    let mut buckets = BTreeMap::<String, Vec<&AdaptiveOctreeNode>>::new();
    for leaf in &octree.leaves {
        let region = classify_leaf_region(leaf);
        buckets.entry(region).or_default().push(leaf);
    }
    for required in [
        "ordinary_upper_surface",
        "ordinary_lower_surface",
        "leading_edge",
        "te_upper_boundary",
        "te_lower_boundary",
        "te_cap",
        "tip_transitions",
        "rounded_tip",
    ] {
        buckets.entry(required.to_string()).or_default();
    }
    buckets
        .into_iter()
        .map(|(region, leaves)| {
            let mut sizes = leaves.iter().map(|l| l.size() as f64).collect::<Vec<_>>();
            sizes.sort_by(|a, b| a.total_cmp(b));
            let proxy = leaves
                .iter()
                .filter(|l| l.surface_crossing)
                .map(|l| (l.size() as f64).powi(2))
                .sum::<f64>();
            let uniform_region = uniform_regions.get(&region).cloned();
            let uniform_p50 = uniform_region
                .as_ref()
                .map(|r| r.p50_leaf_size)
                .filter(|v| *v > 0.0)
                .unwrap_or(5.269230842590332);
            let p50 = percentile(&sizes, 0.50);
            let achieved = if region.starts_with("te_") || region == "te_cap" {
                Some(1.0 / p50.max(1.0e-9))
            } else {
                None
            };
            RegionDensity {
                region,
                leaf_count: leaves.len(),
                surface_leaf_count: leaves.iter().filter(|l| l.surface_crossing).count(),
                max_level_leaf_count: leaves
                    .iter()
                    .filter(|l| l.level == octree.summary.max_level_reached)
                    .count(),
                min_leaf_size: sizes.first().copied().unwrap_or(0.0),
                p50_leaf_size: p50,
                p95_leaf_size: percentile(&sizes, 0.95),
                estimated_surface_area_proxy: proxy,
                density_ratio_vs_uniform: if p50 > 0.0 {
                    (uniform_p50 / p50).powi(2)
                } else {
                    0.0
                },
                requested_te_intervals,
                achieved_te_intervals_p50: achieved,
            }
        })
        .collect()
}

fn classify_leaf_region(leaf: &AdaptiveOctreeNode) -> String {
    let feature = leaf
        .feature_summary
        .closest_feature_type
        .as_deref()
        .unwrap_or("");
    let sector = leaf
        .feature_summary
        .surface_sector_id
        .as_deref()
        .unwrap_or("");
    let in_band = leaf.feature_summary.in_feature_band;
    if !in_band && sector.contains("surface.upper") {
        return "ordinary_upper_surface".to_string();
    }
    if !in_band && sector.contains("surface.lower") {
        return "ordinary_lower_surface".to_string();
    }
    if feature.contains("TrailingEdgeUpperCapBoundary") {
        "te_upper_boundary".to_string()
    } else if feature.contains("TrailingEdgeLowerCapBoundary") {
        "te_lower_boundary".to_string()
    } else if feature.contains("TrailingEdgeCapCenter") || sector.contains("trailing_edge.cap") {
        "te_cap".to_string()
    } else if feature.contains("TipUpperCapBoundary") || feature.contains("TipLowerCapBoundary") {
        "tip_transitions".to_string()
    } else if feature.contains("TipPerimeter") || sector.contains("tip_cap") {
        "rounded_tip".to_string()
    } else if in_band && feature.contains("LeadingEdge") {
        "leading_edge".to_string()
    } else if sector.contains("surface.upper") {
        "ordinary_upper_surface".to_string()
    } else if sector.contains("surface.lower") {
        "ordinary_lower_surface".to_string()
    } else {
        "unclassified".to_string()
    }
}

fn region_baseline(metrics: &FeatureResolutionMetrics) -> BTreeMap<String, RegionDensity> {
    let mut out = BTreeMap::new();
    for r in &metrics.regions {
        let key = match r.region_id.as_str() {
            "upper_trailing_edge_cap_boundary" => "te_upper_boundary",
            "lower_trailing_edge_cap_boundary" => "te_lower_boundary",
            "trailing_edge_cap_sector" => "te_cap",
            "positive_tip_upper_transition"
            | "positive_tip_lower_transition"
            | "negative_tip_upper_transition"
            | "negative_tip_lower_transition" => "tip_transitions",
            "rounded_end_cap_perimeter" => "rounded_tip",
            "leading_edge" => "leading_edge",
            _ => r.region_id.as_str(),
        };
        out.insert(
            key.to_string(),
            RegionDensity {
                region: key.to_string(),
                leaf_count: r.edge_count,
                surface_leaf_count: r.triangle_count,
                max_level_leaf_count: 0,
                min_leaf_size: r.edge_length_min,
                p50_leaf_size: r.edge_length_p50,
                p95_leaf_size: r.edge_length_p95,
                estimated_surface_area_proxy: r.triangle_count as f64
                    * r.edge_length_p50.max(1.0e-9).powi(2),
                density_ratio_vs_uniform: 1.0,
                requested_te_intervals: 1.0,
                achieved_te_intervals_p50: r.estimated_intervals_across_thickness_p50,
            },
        );
    }
    out
}

fn density_requirements(
    regions: &[RegionDensity],
    requested_te_intervals: f64,
) -> serde_json::Value {
    let get = |name: &str| {
        regions
            .iter()
            .find(|r| r.region == name)
            .map(|r| r.density_ratio_vs_uniform)
            .unwrap_or(0.0)
    };
    json!({
        "ordinary_upper_no_coarser_than_uniform_proxy": get("ordinary_upper_surface") >= 1.0,
        "ordinary_lower_no_coarser_than_uniform_proxy": get("ordinary_lower_surface") >= 1.0,
        "te_upper_density_vs_uniform": get("te_upper_boundary"),
        "te_lower_density_vs_uniform": get("te_lower_boundary"),
        "te_cap_density_vs_uniform": get("te_cap"),
        "tip_transition_density_vs_uniform": get("tip_transitions"),
        "rounded_tip_density_vs_uniform": get("rounded_tip"),
        "requested_te_intervals": requested_te_intervals,
        "note": "density is octree leaf surface-area proxy before direct extraction; direct transition extractor remains diagnostic"
    })
}

fn feature_metrics_config(global: f64, feature_target: f64) -> FeatureResolutionConfig {
    FeatureResolutionConfig {
        global_target_edge_length: global,
        feature_target_edge_length: feature_target,
        feature_band_width: 8.0,
        trailing_edge_thickness: 1.0,
        refine_trailing_edge: true,
        refine_tip_transitions: true,
        refine_cap_sectors: true,
        ..FeatureResolutionConfig::default()
    }
}

fn finite_wing_extraction() -> SurfaceExtractionConfig {
    SurfaceExtractionConfig {
        target_spacing: 10.0,
        min_spacing: 4.0,
        max_spacing: 16.0,
        bounds_padding: 12.0,
        weld_tolerance: 1.0e-1,
        projection_tolerance: 5.0e-3,
        max_projection_displacement_factor: 5.0,
        sharp_dihedral_degrees: 55.0,
        ..SurfaceExtractionConfig::default().with_resolution(52)
    }
}

fn finite_wing_remesh_config() -> SurfaceRemeshConfig {
    SurfaceRemeshConfig {
        target_edge_length: 8.0,
        split_threshold_factor: 1.4,
        collapse_threshold_factor: 0.3,
        max_iterations: 1,
        max_operations_per_iteration: 40,
        allow_edge_flips: false,
        projection_tolerance: 5.0e-3,
        maximum_projection_displacement_factor: 5.0,
        smoothing_iterations_per_pass: 0,
        minimum_triangle_angle_deg: 0.05,
        maximum_aspect_ratio: 600.0,
        maximum_normal_deviation_deg: 88.0,
        ..SurfaceRemeshConfig::default()
    }
}

fn adaptive_surface_config(config: &AdaptiveSamplingConfig) -> SurfaceExtractionConfig {
    SurfaceExtractionConfig {
        target_spacing: config.base_cell_size,
        min_spacing: config.feature_target_size() as f64,
        max_spacing: config.base_cell_size * 2.0,
        resolution_override: Some(0),
        bounds_padding: config.bounds_padding,
        weld_tolerance: 1.0e-4,
        projection_tolerance: 5.0e-3,
        max_projection_displacement_factor: 8.0,
        sharp_dihedral_degrees: 55.0,
        ..SurfaceExtractionConfig::default()
    }
}

fn write_octree_diagnostics(name: &str, octree: &AdaptiveOctree) -> Result<(), String> {
    let path = PathBuf::from(OCTREE_DIR).join(format!("{name}_leaf_levels.csv"));
    let mut file = fs::File::create(path).map_err(|e| e.to_string())?;
    writeln!(
        file,
        "id,level,morton,x,y,z,size,surface_crossing,feature,sector,reasons"
    )
    .map_err(|e| e.to_string())?;
    for leaf in &octree.leaves {
        let c = leaf.center();
        let reasons = leaf
            .refinement_reasons
            .iter()
            .map(|r| format!("{r:?}"))
            .collect::<Vec<_>>()
            .join("|");
        writeln!(
            file,
            "{},{},{},{:.9},{:.9},{:.9},{:.9},{},{},{},{}",
            leaf.id,
            leaf.level,
            leaf.morton_code,
            c.x,
            c.y,
            c.z,
            leaf.size(),
            leaf.surface_crossing,
            leaf.feature_summary
                .closest_feature_type
                .as_deref()
                .unwrap_or(""),
            leaf.feature_summary
                .surface_sector_id
                .as_deref()
                .unwrap_or(""),
            reasons
        )
        .map_err(|e| e.to_string())?;
    }
    Ok(())
}

fn write_render_manifest(
    uniform_grid: &UniformGridAudit,
    uniform: &SdfSurfaceMesh,
    reports: &[VariantReport],
) -> Result<PathBuf, String> {
    let variants = std::iter::once(json!({
        "name": "uniform_reference",
        "mesh_path": "research/sdf_cfd_mesher/experiments/adaptive_resolution_policy_v12/meshes/uniform_reference.ply",
        "mesh_hash": uniform.mesh_hash(),
        "leaf_set_hash": "uniform_grid",
        "base_surface_size": uniform_grid.effective_cell_size,
        "requested_te_intervals": 0.0,
        "achieved_te_intervals": uniform_grid.effective_cell_size.recip(),
        "triangles": uniform.triangles.len(),
        "vertices": uniform.vertices.len()
    }))
    .chain(reports.iter().map(|r| {
        let achieved = r
            .region_density
            .iter()
            .find(|d| d.region == "te_cap")
            .and_then(|d| d.achieved_te_intervals_p50)
            .unwrap_or(0.0);
        json!({
            "name": r.name,
            "mesh_path": format!("research/sdf_cfd_mesher/experiments/adaptive_resolution_policy_v12/meshes/{}.ply", r.name),
            "mesh_hash": r.output_mesh_hash,
            "leaf_set_hash": r.leaf_set_hash,
            "base_surface_size": uniform_grid.effective_cell_size,
            "requested_te_intervals": r.requested_te_intervals,
            "achieved_te_intervals": achieved,
            "triangles": r.direct_extraction["triangles"].as_u64().unwrap_or(0),
            "vertices": r.direct_extraction["vertices"].as_u64().unwrap_or(0)
        })
    }))
    .collect::<Vec<_>>();
    let path = PathBuf::from(EXP_DIR).join("render_manifest.json");
    fs::write(
        &path,
        serde_json::to_string_pretty(&json!({ "variants": variants }))
            .map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    Ok(path)
}

fn render_screenshots(manifest: &Path) -> serde_json::Value {
    let script =
        PathBuf::from("research/sdf_cfd_mesher/scripts/render_adaptive_resolution_policy_v12.py");
    let output = Command::new("python")
        .arg(script)
        .arg("--manifest")
        .arg(manifest)
        .arg("--out")
        .arg(SCREENSHOT_DIR)
        .output();
    match output {
        Ok(output) => json!({
            "success": output.status.success(),
            "status": output.status.code(),
            "stdout": String::from_utf8_lossy(&output.stdout),
            "stderr": String::from_utf8_lossy(&output.stderr)
        }),
        Err(err) => json!({"success": false, "error": err.to_string()}),
    }
}

fn attach_screenshot_paths(reports: &mut [VariantReport]) {
    let paths = screenshot_paths();
    for report in reports {
        report.screenshots = paths
            .iter()
            .filter(|p| p.contains(&report.name))
            .cloned()
            .collect();
    }
}

fn screenshot_paths() -> Vec<String> {
    let mut paths = fs::read_dir(SCREENSHOT_DIR)
        .ok()
        .into_iter()
        .flat_map(|it| it.filter_map(Result::ok))
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("png"))
        .map(|p| path_for_report(&p))
        .collect::<Vec<_>>();
    paths.sort();
    paths
}

fn variant_identity(reports: &[VariantReport]) -> serde_json::Value {
    let mut config_hashes = BTreeMap::<String, String>::new();
    let mut leaf_hashes = BTreeMap::<String, String>::new();
    let mut mesh_file_hashes = BTreeMap::<String, String>::new();
    for r in reports {
        config_hashes.insert(r.name.clone(), r.configuration_hash.clone());
        leaf_hashes.insert(r.name.clone(), r.leaf_set_hash.clone());
        mesh_file_hashes.insert(r.name.clone(), r.mesh_file_hash.clone());
    }
    json!({
        "configuration_hashes": config_hashes,
        "leaf_set_hashes": leaf_hashes,
        "mesh_file_hashes": mesh_file_hashes,
        "a_b_c_config_distinct": all_values_distinct(&reports.iter().map(|r| r.configuration_hash.clone()).collect::<Vec<_>>()),
        "a_b_c_leaf_sets_distinct": all_values_distinct(&reports.iter().map(|r| r.leaf_set_hash.clone()).collect::<Vec<_>>()),
        "a_b_c_output_files_distinct": all_values_distinct(&reports.iter().map(|r| r.mesh_file_hash.clone()).collect::<Vec<_>>())
    })
}

fn all_values_distinct(values: &[String]) -> bool {
    for i in 0..values.len() {
        for j in (i + 1)..values.len() {
            if values[i] == values[j] {
                return false;
            }
        }
    }
    true
}

fn conclusion(reports: &[VariantReport]) -> serde_json::Value {
    let leaf_distinct = all_values_distinct(
        &reports
            .iter()
            .map(|r| r.leaf_set_hash.clone())
            .collect::<Vec<_>>(),
    );
    let all_failed = reports
        .iter()
        .all(|r| r.status == "diagnostic_extraction_failed_closed");
    json!({
        "resolution_policy_corrected": leaf_distinct,
        "direct_extraction_accepted": !all_failed,
        "primary_result": if leaf_distinct {
            "adaptive A/B/C now produce distinct leaf sets with a near-surface uniform-equivalent floor; direct extraction remains diagnostic until transition connectivity is corrected"
        } else {
            "adaptive variants still collapse to identical leaf sets"
        }
    })
}

fn recommended_next_task(reports: &[VariantReport]) -> serde_json::Value {
    let leaf_distinct = all_values_distinct(
        &reports
            .iter()
            .map(|r| r.leaf_set_hash.clone())
            .collect::<Vec<_>>(),
    );
    if leaf_distinct {
        json!({
            "task": "adaptive extraction transition correction for the corrected v12 leaf sets",
            "reason": "resolution policy now keeps the zero set at least uniform-equivalent and distinguishes A/B/C, but direct adaptive extraction is still only diagnostic and may fail closed on coarse/fine transitions"
        })
    } else {
        json!({
            "task": "adaptive variant-output identity correction",
            "reason": "A/B/C still do not produce distinct leaf-set hashes"
        })
    }
}

fn markdown_summary(report: &serde_json::Value) -> String {
    format!(
        "# Adaptive Resolution Policy v12\n\n\
        Result: implemented a near-surface uniform-equivalent resolution floor and generated distinct adaptive policy diagnostics.\n\n\
        Uniform effective cell size: `{:.6}`.\n\n\
        Uniform surface-crossing cells: `{}`.\n\n\
        Variant identity: config distinct `{}`, leaf-set distinct `{}`, output-file distinct `{}`.\n\n\
        Direct extraction remains diagnostic; transition defects are not corrected in this task.\n\n\
        Screenshots: `{}`.\n\n\
        Recommended next task: `{}`.\n",
        report["uniform_extractor_configuration"]["effective_cell_size"]
            .as_f64()
            .unwrap_or(0.0),
        report["uniform_extractor_configuration"]["surface_crossing_cell_count"]
            .as_u64()
            .unwrap_or(0),
        report["variant_identity"]["a_b_c_config_distinct"]
            .as_bool()
            .unwrap_or(false),
        report["variant_identity"]["a_b_c_leaf_sets_distinct"]
            .as_bool()
            .unwrap_or(false),
        report["variant_identity"]["a_b_c_output_files_distinct"]
            .as_bool()
            .unwrap_or(false),
        SCREENSHOT_DIR,
        report["recommended_next_task"]["task"]
            .as_str()
            .unwrap_or("unknown")
    )
}

fn write_density_csv(report: &serde_json::Value) -> Result<(), String> {
    let path = PathBuf::from(EXP_DIR).join("region_density.csv");
    let mut file = fs::File::create(path).map_err(|e| e.to_string())?;
    writeln!(
        file,
        "variant,region,leaf_count,surface_leaf_count,min_leaf_size,p50_leaf_size,p95_leaf_size,density_ratio_vs_uniform,requested_te_intervals,achieved_te_intervals_p50"
    )
    .map_err(|e| e.to_string())?;
    for v in report["variants"].as_array().unwrap_or(&Vec::new()) {
        for r in v["region_density"].as_array().unwrap_or(&Vec::new()) {
            writeln!(
                file,
                "{},{},{},{},{},{},{},{},{},{}",
                v["name"].as_str().unwrap_or(""),
                r["region"].as_str().unwrap_or(""),
                r["leaf_count"].as_u64().unwrap_or(0),
                r["surface_leaf_count"].as_u64().unwrap_or(0),
                r["min_leaf_size"].as_f64().unwrap_or(0.0),
                r["p50_leaf_size"].as_f64().unwrap_or(0.0),
                r["p95_leaf_size"].as_f64().unwrap_or(0.0),
                r["density_ratio_vs_uniform"].as_f64().unwrap_or(0.0),
                r["requested_te_intervals"].as_f64().unwrap_or(0.0),
                r["achieved_te_intervals_p50"].as_f64().unwrap_or(0.0),
            )
            .map_err(|e| e.to_string())?;
        }
    }
    Ok(())
}

fn required_depth(root_size: f64, target: f64) -> u32 {
    if target <= 0.0 || root_size <= target {
        return 0;
    }
    (root_size / target).log2().ceil() as u32
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

fn percentile(values: &[f64], q: f64) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let idx = ((values.len().saturating_sub(1)) as f64 * q.clamp(0.0, 1.0)).round() as usize;
    values[idx.min(values.len() - 1)]
}

fn eval_sdf(script: &str) -> Result<Arc<dyn Sdf>, String> {
    Ok(scripting::evaluate_script(script)
        .map_err(|e| e.to_string())?
        .into_conditioned_for_backend()
        .sdf)
}

fn array_from_vec3(v: Vec3) -> [f64; 3] {
    [v.x as f64, v.y as f64, v.z as f64]
}

fn file_hash(path: &Path) -> Result<String, String> {
    let bytes = fs::read(path).map_err(|e| e.to_string())?;
    Ok(stable_hash_hex(&bytes))
}

fn stable_hash_hex(bytes: &[u8]) -> String {
    let mut hash = 0xcbf29ce484222325_u64;
    for byte in bytes {
        hash ^= *byte as u64;
        hash = hash.wrapping_mul(0x100000001b3);
    }
    format!("{hash:016x}")
}

fn elapsed_ms(start: Instant) -> f64 {
    start.elapsed().as_secs_f64() * 1000.0
}

fn path_for_report(path: &Path) -> String {
    path.to_string_lossy().replace('\\', "/")
}
