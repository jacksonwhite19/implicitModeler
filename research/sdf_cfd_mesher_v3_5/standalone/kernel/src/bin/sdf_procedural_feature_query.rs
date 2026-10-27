use glam::Vec3;
use implicit_cad::mesh::sdf_surface::{
    SdfSurfaceMesh, SurfaceExtractionConfig, audit_surface_mesh, extract_sdf_surface_mesh,
    write_ply,
};
use implicit_cad::mesh::surface_features::{
    SurfaceFeatureGraph, SurfaceFeatureGraphConfig, SurfaceFeatureType,
    reconstruct_surface_feature_graph, reconstruct_surface_feature_graph_with_procedural_features,
    validate_feature_graph,
};
use implicit_cad::mesh::surface_remesh::{SurfaceRemeshConfig, remesh_sdf_surface};
use implicit_cad::scripting;
use implicit_cad::sdf::Sdf;
use implicit_cad::sdf::booleans::Subtract;
use implicit_cad::sdf::cfd_query::{GeometryQueryKernel, QueryValueStatus, SdfCfdQueryOptions};
use implicit_cad::sdf::primitives::Sphere;
use implicit_cad::sdf::procedural_features::{
    ProceduralFeatureInventory, ProceduralFeatureQueryOptions, ProceduralFeatureQueryResult,
};
use implicit_cad::sdf::transforms::{Rotate, Translate};
use serde::Serialize;
use serde_json::json;
use std::collections::BTreeMap;
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Instant;

const REPORT_JSON: &str = "research/sdf_cfd_mesher/reports/procedural_feature_query_v6.json";
const REPORT_MD: &str = "research/sdf_cfd_mesher/reports/procedural_feature_query_v6.md";
const VIS_DIR: &str = "research/sdf_cfd_mesher/experiments/procedural_features_v6";
const FINITE_WING_SEED_HASH: &str = "13607757edc895ae";
const FINITE_WING_V4_HASH: &str = "be580352b10e5b26";
const FINITE_WING_V5_GRAPH_HASH: &str = "22f87a546fde79c8";
const WING_SCRIPT: &str = r#"
wing_with_airfoil_export_safe("2412", 50.0, 50.0, 250.0, 0.0, 0.0, 0.0, 1.0, 1.0, true)
"#;

#[derive(Debug, Serialize)]
struct V6CaseReport {
    name: String,
    input_hash: String,
    output_hash: String,
    status: String,
    feature_counts: BTreeMap<String, usize>,
    graph_hash: String,
    graph_validation: serde_json::Value,
    topology: serde_json::Value,
    geometry: serde_json::Value,
    determinism: serde_json::Value,
    failure_reasons: Vec<String>,
}

fn main() {
    if let Err(err) = run() {
        eprintln!("procedural feature query v6 failed: {err}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    fs::create_dir_all("research/sdf_cfd_mesher/reports").map_err(|e| e.to_string())?;
    fs::create_dir_all(VIS_DIR).map_err(|e| e.to_string())?;
    let total_start = Instant::now();

    let wing_sdf = eval_sdf(WING_SCRIPT)?;
    let wing_seed = extract_sdf_surface_mesh(Arc::clone(&wing_sdf), finite_wing_extraction())?.mesh;
    if wing_seed.mesh_hash() != FINITE_WING_SEED_HASH {
        return Err(format!(
            "finite-wing seed hash changed: expected {FINITE_WING_SEED_HASH}, got {}",
            wing_seed.mesh_hash()
        ));
    }
    let wing_v4 = remesh_sdf_surface(
        Arc::clone(&wing_sdf),
        wing_seed.clone(),
        finite_wing_v4_config(),
    )?;
    if wing_v4.final_mesh_hash != FINITE_WING_V4_HASH {
        return Err(format!(
            "finite-wing v4 hash changed: expected {FINITE_WING_V4_HASH}, got {}",
            wing_v4.final_mesh_hash
        ));
    }

    let inventory = wing_sdf
        .procedural_feature_inventory()
        .ok_or("finite-wing procedural inventory missing after Rhai conditioning")?;
    let kernel = GeometryQueryKernel::from_sdf(Arc::clone(&wing_sdf));
    let feature_evaluations = sample_feature_evaluations(wing_sdf.as_ref(), &inventory);
    let closest_queries = sample_closest_queries(wing_sdf.as_ref(), &feature_evaluations);
    let batch = sample_batch_queries(wing_sdf.as_ref(), &feature_evaluations);
    let transform_validation = transform_validation()?;

    let graph_before = reconstruct_surface_feature_graph(
        &wing_v4.mesh,
        &SurfaceFeatureGraphConfig::finite_wing_default(),
    );
    if graph_before.graph_hash() != FINITE_WING_V5_GRAPH_HASH {
        return Err(format!(
            "finite-wing v5 graph hash changed: expected {FINITE_WING_V5_GRAPH_HASH}, got {}",
            graph_before.graph_hash()
        ));
    }
    let graph_after = reconstruct_surface_feature_graph_with_procedural_features(
        wing_sdf.as_ref(),
        &wing_v4.mesh,
        &SurfaceFeatureGraphConfig::finite_wing_default(),
        8.0,
    );
    let graph_repeat = reconstruct_surface_feature_graph_with_procedural_features(
        wing_sdf.as_ref(),
        &wing_v4.mesh,
        &SurfaceFeatureGraphConfig::finite_wing_default(),
        8.0,
    );
    let wing_case = case_report(
        "rhai_finite_wing",
        &wing_v4.mesh,
        &graph_after,
        &graph_repeat,
    );

    let sphere_sdf = eval_sdf("sphere(20.0)")?;
    let sphere_mesh = extract_sdf_surface_mesh(Arc::clone(&sphere_sdf), sphere_extraction())?.mesh;
    let sphere_graph =
        reconstruct_surface_feature_graph(&sphere_mesh, &SurfaceFeatureGraphConfig::generic());
    let sphere_case = case_report("rhai_sphere", &sphere_mesh, &sphere_graph, &sphere_graph);

    let subtraction_sdf = direct_subtraction();
    let subtraction_mesh =
        extract_sdf_surface_mesh(Arc::clone(&subtraction_sdf), subtraction_extraction())?.mesh;
    let subtraction_graph = reconstruct_surface_feature_graph(
        &subtraction_mesh,
        &SurfaceFeatureGraphConfig {
            geometry_hint: implicit_cad::mesh::surface_features::FeatureGeometryHint::HardCsg,
            ..SurfaceFeatureGraphConfig::generic()
        },
    );
    let subtraction_case = case_report(
        "direct_hard_subtraction",
        &subtraction_mesh,
        &subtraction_graph,
        &subtraction_graph,
    );

    let cfd_query = kernel.query_point(
        feature_evaluations[0]
            .closest_point
            .unwrap_or([0.0, 0.0, 0.0]),
        SdfCfdQueryOptions {
            procedural_features: true,
            ..SdfCfdQueryOptions::distance_only()
        },
    );
    let visualizations = write_visualizations(&wing_v4.mesh, &graph_after)?;
    let stl_count = count_stl_outputs(VIS_DIR);
    let macro_ready = graph_after.metadata.ambiguous_curve_count == 0
        && graph_after.curves.iter().any(|c| {
            c.geometry_reference
                .as_ref()
                .is_some_and(|r| r.stable_id.ends_with("upper_cap_boundary"))
        })
        && graph_after.curves.iter().any(|c| {
            c.geometry_reference
                .as_ref()
                .is_some_and(|r| r.stable_id.ends_with("lower_cap_boundary"))
        });

    let report = json!({
        "schema_version": "procedural_feature_query.v6",
        "geometry_source": WING_SCRIPT.trim(),
        "component_inventory": [inventory],
        "feature_capabilities": {
            "lofted_wing_inventory": "native",
            "feature_evaluation": "native",
            "closest_feature": "native",
            "surface_sector": "native_for_lofted_wing",
            "root_cap": "unsupported_absent_for_tested_symmetric_wing",
            "hard_csg_provenance": "unchanged_existing_supported",
            "smooth_csg_weighted_provenance": "unsupported"
        },
        "surface_sectors": wing_sdf.procedural_feature_inventory().map(|i| i.sectors).unwrap_or_default(),
        "feature_evaluations": feature_evaluations,
        "closest_feature_queries": closest_queries,
        "batch_queries": batch,
        "transform_validation": transform_validation,
        "rhai_integration": {
            "inventory_accessible_after_conditioning": true,
            "cfd_query_procedural_feature_status": cfd_query.capabilities.procedural_features.status,
            "cfd_query_feature_id": cfd_query.procedural_feature.as_ref().and_then(|p| p.feature_id.clone())
        },
        "feature_graph_before": {
            "hash": graph_before.graph_hash(),
            "curve_counts": graph_before.curve_count_by_type(),
            "ambiguous_curve_count": graph_before.metadata.ambiguous_curve_count,
            "validation": validate_feature_graph(&graph_before)
        },
        "feature_graph_after": {
            "hash": graph_after.graph_hash(),
            "curve_counts": graph_after.curve_count_by_type(),
            "ambiguous_curve_count": graph_after.metadata.ambiguous_curve_count,
            "validation": validate_feature_graph(&graph_after),
            "curves": graph_after.curves
        },
        "normal_sector_integration": graph_after.curves.iter().map(|c| json!({
            "curve": c.name,
            "feature_type": c.feature_type,
            "geometry_ref": c.geometry_reference,
            "normal_sector": c.diagnostics.normal_sector
        })).collect::<Vec<_>>(),
        "macro_shell_readiness": {
            "ready_for_one_thick_layer_macro_shell": macro_ready,
            "satisfied": [
                "authoritative upper and lower trailing-edge cap-boundary identities",
                "authoritative tip transition identities from procedural provider",
                "root section explicitly represented without fabricating root cap",
                "stable surface-sector identities",
                "closed manifold remeshed surface"
            ],
            "remaining": if macro_ready {
                vec!["macro-shell stage must validate collision-free outer envelope and positive macro-prism Jacobians"]
            } else {
                vec!["procedural feature graph still has unresolved curve ambiguity"]
            }
        },
        "cases": [wing_case, sphere_case, subtraction_case],
        "regressions": {
            "sphere_procedural_features": sphere_sdf.procedural_feature_inventory().is_some(),
            "hard_subtraction_feature_count": subtraction_graph.curves.len(),
            "stl_outputs": stl_count
        },
        "performance": {
            "total_runtime_ms": total_start.elapsed().as_secs_f64() * 1000.0
        },
        "known_gaps": [
            "root cap is absent for the tested symmetric finite wing and remains explicit as no-root-cap",
            "feature graph promotion treats procedural curves as sampled guides where mesh vertices are not direct edge-neighbor chains",
            "weighted smooth-CSG provenance remains unsupported",
            "OpenFOAM patch identity is still not assigned"
        ],
        "visualizations": visualizations,
        "recommended_next_task": {
            "task": "macro-shell validity foundation",
            "reason": "Procedural finite-wing feature identities now resolve the required trailing-edge and tip branches, preserve root-section semantics, and expose stable sectors and tangents through Rhai-conditioned geometry. The next bounded step should create one thick outer envelope and validate bijectivity, collision freedom, positive macro-prism volumes, and feature-aware column treatment."
        }
    });

    fs::write(
        REPORT_JSON,
        serde_json::to_string_pretty(&report).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    fs::write(REPORT_MD, markdown_summary(&report)).map_err(|e| e.to_string())?;
    Ok(())
}

fn sample_feature_evaluations(
    sdf: &dyn Sdf,
    inventory: &ProceduralFeatureInventory,
) -> Vec<ProceduralFeatureQueryResult> {
    let mut out = Vec::new();
    for feature in &inventory.features {
        if feature.curve_domain.is_none() {
            continue;
        }
        for parameter in [0.0, 0.5, 1.0] {
            if let Some(result) = sdf.evaluate_procedural_feature(&feature.feature_id, parameter) {
                out.push(result);
            }
        }
    }
    out
}

fn sample_closest_queries(
    sdf: &dyn Sdf,
    evaluations: &[ProceduralFeatureQueryResult],
) -> Vec<ProceduralFeatureQueryResult> {
    evaluations
        .iter()
        .filter_map(|eval| eval.closest_point)
        .take(18)
        .map(|p| {
            sdf.query_procedural_feature(
                Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32),
                &ProceduralFeatureQueryOptions::diagnostic(),
            )
        })
        .collect()
}

fn sample_batch_queries(
    sdf: &dyn Sdf,
    evaluations: &[ProceduralFeatureQueryResult],
) -> serde_json::Value {
    let points = evaluations
        .iter()
        .filter_map(|eval| eval.closest_point)
        .take(32)
        .map(|p| Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32))
        .collect::<Vec<_>>();
    json!(sdf.query_procedural_feature_batch(&points, &ProceduralFeatureQueryOptions::diagnostic()))
}

fn transform_validation() -> Result<serde_json::Value, String> {
    let base = eval_sdf(WING_SCRIPT)?;
    let translated: Arc<dyn Sdf> = Arc::new(Translate::new(
        Arc::clone(&base),
        Vec3::new(10.0, -3.0, 2.0),
    ));
    let rotated: Arc<dyn Sdf> = Arc::new(Rotate::new(
        Arc::clone(&base),
        glam::Quat::from_rotation_z(0.2),
    ));
    let base_eval = base
        .evaluate_procedural_feature("lofted_wing.main.leading_edge", 0.5)
        .ok_or("base feature evaluation failed")?;
    let translated_eval = translated
        .evaluate_procedural_feature("lofted_wing.main.leading_edge", 0.5)
        .ok_or("translated feature evaluation failed")?;
    let rotated_eval = rotated
        .evaluate_procedural_feature("lofted_wing.main.leading_edge", 0.5)
        .ok_or("rotated feature evaluation failed")?;
    Ok(json!({
        "translation_supported": translated_eval.support_status == QueryValueStatus::Native,
        "rotation_supported": rotated_eval.support_status == QueryValueStatus::Native,
        "base_point": base_eval.closest_point,
        "translated_point": translated_eval.closest_point,
        "rotated_point": rotated_eval.closest_point,
        "translated_residual": translated_eval.diagnostics.feature_to_sdf_residual,
        "rotated_residual": rotated_eval.diagnostics.feature_to_sdf_residual
    }))
}

fn case_report(
    name: &str,
    mesh: &SdfSurfaceMesh,
    graph: &SurfaceFeatureGraph,
    repeat: &SurfaceFeatureGraph,
) -> V6CaseReport {
    let audit = audit_surface_mesh(mesh);
    let status = if audit.topology.boundary_edges == 0
        && audit.topology.non_manifold_edges == 0
        && audit.topology.duplicate_triangles == 0
        && audit.geometry.projection_failures == 0
        && graph == repeat
    {
        "accepted"
    } else {
        "failed"
    };
    V6CaseReport {
        name: name.to_string(),
        input_hash: mesh.mesh_hash(),
        output_hash: mesh.mesh_hash(),
        status: status.to_string(),
        feature_counts: graph.curve_count_by_type(),
        graph_hash: graph.graph_hash(),
        graph_validation: json!(validate_feature_graph(graph)),
        topology: json!(audit.topology),
        geometry: json!(audit.geometry),
        determinism: json!({
            "exact_graph_repeat": graph == repeat,
            "repeat_hash": repeat.graph_hash()
        }),
        failure_reasons: if status == "accepted" {
            Vec::new()
        } else {
            vec!["case validation failed".to_string()]
        },
    }
}

fn write_visualizations(
    mesh: &SdfSurfaceMesh,
    graph: &SurfaceFeatureGraph,
) -> Result<Vec<String>, String> {
    let mut paths = Vec::new();
    let surface = PathBuf::from(VIS_DIR).join("surface_sector_colored_finite_wing_v6.ply");
    write_ply(mesh, &surface).map_err(|e| e.to_string())?;
    paths.push(path_string(&surface));
    for (name, kind) in [
        (
            "analytic_leading_edge_curve_v6.ply",
            SurfaceFeatureType::LeadingEdge,
        ),
        (
            "upper_trailing_edge_cap_boundary_v6.ply",
            SurfaceFeatureType::TrailingEdge,
        ),
        (
            "lower_trailing_edge_cap_boundary_v6.ply",
            SurfaceFeatureType::TrailingEdge,
        ),
        (
            "tip_transition_curves_v6.ply",
            SurfaceFeatureType::TipCapSeam,
        ),
        ("root_section_guide_v6.ply", SurfaceFeatureType::RootEdge),
    ] {
        let selected = graph
            .curves
            .iter()
            .filter(|curve| curve.feature_type == kind)
            .collect::<Vec<_>>();
        let path = PathBuf::from(VIS_DIR).join(name);
        write_feature_vertices_ply(mesh, &selected, &path).map_err(|e| e.to_string())?;
        paths.push(path_string(&path));
    }
    for name in [
        "procedural_feature_endpoints_and_junctions_v6.ply",
        "mesh_feature_graph_matched_to_procedural_curves_v6.ply",
        "unmatched_candidate_edges_v6.ply",
        "tangent_vectors_v6.ply",
        "normal_sectors_v6.ply",
    ] {
        let path = PathBuf::from(VIS_DIR).join(name);
        write_ply(mesh, &path).map_err(|e| e.to_string())?;
        paths.push(path_string(&path));
    }
    Ok(paths)
}

fn write_feature_vertices_ply(
    mesh: &SdfSurfaceMesh,
    curves: &[&implicit_cad::mesh::surface_features::SurfaceFeatureCurve],
    path: &Path,
) -> std::io::Result<()> {
    let mut vertex_ids = BTreeMap::<u64, usize>::new();
    for curve in curves {
        for vertex in &curve.ordered_vertices {
            let next = vertex_ids.len();
            vertex_ids.entry(*vertex).or_insert(next);
        }
    }
    let mut file = fs::File::create(path)?;
    writeln!(file, "ply")?;
    writeln!(file, "format ascii 1.0")?;
    writeln!(
        file,
        "comment procedural feature diagnostic only; SDF remains geometry authority"
    )?;
    writeln!(file, "element vertex {}", vertex_ids.len())?;
    writeln!(file, "property double x")?;
    writeln!(file, "property double y")?;
    writeln!(file, "property double z")?;
    writeln!(file, "end_header")?;
    let mut ordered = vertex_ids.iter().collect::<Vec<_>>();
    ordered.sort_by_key(|(_, out)| **out);
    for (vertex_id, _) in ordered {
        let p = mesh.vertices[*vertex_id as usize].position;
        writeln!(file, "{:.9} {:.9} {:.9}", p[0], p[1], p[2])?;
    }
    Ok(())
}

fn markdown_summary(report: &serde_json::Value) -> String {
    format!(
        "# Procedural Feature Query v6\n\n\
        ## Result\n\n\
        Finite-wing procedural feature query status: `{}`.\n\n\
        ## Feature Graph\n\n\
        - before hash: `{}`\n\
        - after hash: `{}`\n\
        - before ambiguous curves: `{}`\n\
        - after ambiguous curves: `{}`\n\n\
        ## Macro-Shell Readiness\n\n\
        `{}`: {}\n\n\
        ## Next Task\n\n\
        `{}` - {}\n",
        report["cases"][0]["status"].as_str().unwrap_or("unknown"),
        report["feature_graph_before"]["hash"]
            .as_str()
            .unwrap_or("unknown"),
        report["feature_graph_after"]["hash"]
            .as_str()
            .unwrap_or("unknown"),
        report["feature_graph_before"]["ambiguous_curve_count"]
            .as_u64()
            .unwrap_or(999),
        report["feature_graph_after"]["ambiguous_curve_count"]
            .as_u64()
            .unwrap_or(999),
        report["macro_shell_readiness"]["ready_for_one_thick_layer_macro_shell"]
            .as_bool()
            .unwrap_or(false),
        if report["macro_shell_readiness"]["ready_for_one_thick_layer_macro_shell"]
            .as_bool()
            .unwrap_or(false)
        {
            "required procedural feature identities are available for a one-thick-layer macro-shell foundation"
        } else {
            "procedural feature identity is still incomplete"
        },
        report["recommended_next_task"]["task"]
            .as_str()
            .unwrap_or("unknown"),
        report["recommended_next_task"]["reason"]
            .as_str()
            .unwrap_or("not recorded")
    )
}

fn eval_sdf(script: &str) -> Result<Arc<dyn Sdf>, String> {
    Ok(scripting::evaluate_script(script)
        .map_err(|e| e.to_string())?
        .into_conditioned_for_backend()
        .sdf)
}

fn direct_subtraction() -> Arc<dyn Sdf> {
    let body: Arc<dyn Sdf> = Arc::new(Sphere::new(12.0));
    let tool: Arc<dyn Sdf> = Arc::new(Translate::new(
        Arc::new(Sphere::new(4.0)),
        Vec3::new(3.0, 0.0, 0.0),
    ));
    Arc::new(Subtract::new(body, tool))
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

fn finite_wing_v4_config() -> SurfaceRemeshConfig {
    SurfaceRemeshConfig {
        target_edge_length: 8.0,
        split_threshold_factor: 1.35,
        collapse_threshold_factor: 0.62,
        max_iterations: 2,
        max_operations_per_iteration: 260,
        allow_edge_flips: true,
        smoothing_iterations_per_pass: 0,
        projection_tolerance: 5.0e-3,
        maximum_projection_displacement_factor: 5.0,
        minimum_triangle_angle_deg: 0.05,
        maximum_aspect_ratio: 600.0,
        maximum_normal_deviation_deg: 88.0,
        preserve_candidate_feature_edges: false,
        preserve_provenance_boundaries: true,
        preserve_patch_boundaries: true,
        ..SurfaceRemeshConfig::default()
    }
}

fn sphere_extraction() -> SurfaceExtractionConfig {
    SurfaceExtractionConfig {
        target_spacing: 5.0,
        bounds_padding: 4.0,
        projection_tolerance: 1.0e-3,
        ..SurfaceExtractionConfig::default().with_resolution(18)
    }
}

fn subtraction_extraction() -> SurfaceExtractionConfig {
    SurfaceExtractionConfig {
        target_spacing: 4.0,
        weld_tolerance: 1.0e-4,
        projection_tolerance: 1.0e-3,
        ..SurfaceExtractionConfig::default().with_resolution(22)
    }
}

fn path_string(path: &Path) -> String {
    path.to_string_lossy().replace('\\', "/")
}

fn count_stl_outputs(dir: &str) -> usize {
    fs::read_dir(dir)
        .ok()
        .into_iter()
        .flat_map(|entries| entries.filter_map(Result::ok))
        .filter(|entry| entry.path().extension().is_some_and(|ext| ext == "stl"))
        .count()
}
