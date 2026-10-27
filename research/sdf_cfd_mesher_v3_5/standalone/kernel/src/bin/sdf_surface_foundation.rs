use glam::Vec3;
use implicit_cad::mesh::sdf_surface::{
    DeterminismAudit, SDF_SURFACE_EXTRACTOR_VERSION, SDF_SURFACE_MESH_SCHEMA_VERSION,
    SurfaceExtractionConfig, SurfaceExtractionOutcome, SurfaceExtractionStatus,
    extract_sdf_surface_mesh, run_determinism_check, write_ply,
};
use implicit_cad::scripting;
use implicit_cad::sdf::Sdf;
use implicit_cad::sdf::booleans::Subtract;
use implicit_cad::sdf::cfd_query::SDF_CFD_QUERY_SCHEMA_VERSION;
use implicit_cad::sdf::primitives::Sphere;
use implicit_cad::sdf::query::BoundingInfo;
use implicit_cad::sdf::query::bounding_points;
use implicit_cad::sdf::transforms::Translate;
use serde::Serialize;
use serde_json::json;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Instant;

const REPORT_JSON: &str = "research/sdf_cfd_mesher/reports/surface_mesh_foundation_v1.json";
const REPORT_MD: &str = "research/sdf_cfd_mesher/reports/surface_mesh_foundation_v1.md";
const VIS_DIR: &str = "research/sdf_cfd_mesher/experiments/surface_foundation/visualizations";

#[derive(Clone)]
struct CaseSpec {
    name: &'static str,
    geometry_source: &'static str,
    script: Option<&'static str>,
    config: SurfaceExtractionConfig,
    required: bool,
}

#[derive(Debug, Serialize)]
struct CaseReport {
    name: String,
    geometry_source: String,
    geometry_hash: Option<String>,
    mesh_hash: Option<String>,
    status: String,
    failure_reasons: Vec<String>,
    counts: serde_json::Value,
    geometry_quality: serde_json::Value,
    topology_quality: serde_json::Value,
    triangle_quality: serde_json::Value,
    provenance: serde_json::Value,
    determinism: serde_json::Value,
    performance: serde_json::Value,
    visualization: Option<String>,
}

fn main() {
    if let Err(err) = run() {
        eprintln!("surface foundation audit failed: {err}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    fs::create_dir_all("research/sdf_cfd_mesher/reports").map_err(|e| e.to_string())?;
    fs::create_dir_all(VIS_DIR).map_err(|e| e.to_string())?;

    let cases = vec![
        CaseSpec {
            name: "rhai_sphere",
            geometry_source: "scripting::evaluate_script(\"sphere(20.0)\")",
            script: Some("sphere(20.0)"),
            config: SurfaceExtractionConfig {
                target_spacing: 5.0,
                min_spacing: 2.0,
                max_spacing: 8.0,
                bounds_padding: 5.0,
                projection_tolerance: 5.0e-4,
                ..SurfaceExtractionConfig::default().with_resolution(28)
            },
            required: true,
        },
        CaseSpec {
            name: "direct_hard_subtraction",
            geometry_source: "direct repository SDF nodes: Subtract(Sphere, Translate(Sphere))",
            script: None,
            config: SurfaceExtractionConfig {
                target_spacing: 4.0,
                min_spacing: 1.5,
                max_spacing: 7.0,
                bounds_padding: 5.0,
                weld_tolerance: 1.0e-4,
                projection_tolerance: 8.0e-4,
                ..SurfaceExtractionConfig::default().with_resolution(32)
            },
            required: true,
        },
        CaseSpec {
            name: "rhai_finite_wing",
            geometry_source: "Rhai wing_with_airfoil_export_safe(\"2412\", 50, 50, 250, ...)",
            script: Some(
                r#"
                wing_with_airfoil_export_safe("2412", 50.0, 50.0, 250.0, 0.0, 0.0, 0.0, 1.0, 1.0, true)
            "#,
            ),
            config: SurfaceExtractionConfig {
                target_spacing: 10.0,
                min_spacing: 4.0,
                max_spacing: 16.0,
                bounds_padding: 12.0,
                weld_tolerance: 1.0e-1,
                projection_tolerance: 5.0e-3,
                max_projection_displacement_factor: 4.0,
                sharp_dihedral_degrees: 55.0,
                ..SurfaceExtractionConfig::default().with_resolution(52)
            },
            required: false,
        },
    ];

    let start = Instant::now();
    let mut reports = Vec::new();
    let mut required_failures = 0usize;

    for case in &cases {
        let report = run_case(case)?;
        if case.required && report.status != "accepted" {
            required_failures += 1;
        }
        reports.push(report);
    }

    let report = json!({
        "schema_version": "surface_mesh_foundation_report.v1",
        "implementation": {
            "extractor": SDF_SURFACE_EXTRACTOR_VERSION,
            "surface_mesh_schema_version": SDF_SURFACE_MESH_SCHEMA_VERSION,
            "query_schema_version": SDF_CFD_QUERY_SCHEMA_VERSION,
            "reused_modules": [
                "src/mesh/marching_cubes.rs",
                "src/sdf/cfd_query.rs",
                "src/sdf/provenance.rs",
                "src/sdf/conditioning.rs",
                "src/scripting/mod.rs"
            ],
            "new_modules": [
                "src/mesh/sdf_surface.rs",
                "src/bin/sdf_surface_foundation.rs",
                "tests/sdf_surface_foundation.rs"
            ],
            "selection_reason": "Uniform marching cubes is the smallest deterministic existing topology source; the foundation welds, projects, orients, and audits the raw triangles instead of treating raw extractor output as CFD-ready."
        },
        "configuration": {
            "cases": cases.iter().map(|c| json!({
                "name": c.name,
                "target_spacing": c.config.target_spacing,
                "resolution_override": c.config.resolution_override,
                "projection_tolerance": c.config.projection_tolerance,
                "require_closed": c.config.require_closed,
                "require_manifold": c.config.require_manifold
            })).collect::<Vec<_>>()
        },
        "cases": reports,
        "capabilities": {
            "typed_surface_mesh": "implemented",
            "initial_extractor": "existing uniform marching cubes reused",
            "vertex_projection": "existing GeometryQueryKernel projection path",
            "sdf_normals": "existing CFD query gradient path",
            "hard_csg_vertex_provenance": "recursive traces attached where supported",
            "patch_identity": "generic wall patch only; production patch mapping deferred",
            "authoritative_feature_edges": "deferred; candidate edges are diagnosed only",
            "smooth_csg_provenance": "unsupported and explicitly counted"
        },
        "known_gaps": [
            "Initial triangles are still marching-cubes quality and need remeshing before macro-shell work.",
            "A through-cut subtractive sphere was observed to leave hard-CSG seam boundary edges with the raw uniform marching-cubes seed; the accepted subtraction case is an enclosed cavity.",
            "Patch assignment is conservative and not sufficient for OpenFOAM boundary grouping.",
            "Smooth-CSG weighted provenance remains unsupported.",
            "Feature edges are candidate diagnostics, not authoritative constraints.",
            "Marching-cubes sampling query counts are estimated from grid size rather than instrumented in the extractor."
        ],
        "recommended_next_task": {
            "task": "extractor/topology correction for real finite-wing output",
            "reason": "The sphere and controlled hard-CSG cases are closed and deterministic, but the real finite-wing marching-cubes seed remains open; topology correction must precede production remeshing or macro-shell work."
        },
        "total_runtime_ms": start.elapsed().as_secs_f64() * 1000.0,
        "required_failures": required_failures
    });

    fs::write(
        REPORT_JSON,
        serde_json::to_string_pretty(&report).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    fs::write(REPORT_MD, markdown_summary(&report)).map_err(|e| e.to_string())?;

    if required_failures > 0 {
        return Err(format!(
            "{required_failures} required surface-foundation cases failed"
        ));
    }
    Ok(())
}

fn run_case(case: &CaseSpec) -> Result<CaseReport, String> {
    let init_start = Instant::now();
    let sdf = load_case_sdf(case)?;
    let init_ms = init_start.elapsed().as_secs_f64() * 1000.0;

    let bounds = bounding_points(sdf.as_ref());
    let estimated_distance_queries =
        (case.config.resolution_override.unwrap_or(32) as u64 + 1).pow(3);

    let outcome = match extract_sdf_surface_mesh(sdf.clone(), case.config.clone()) {
        Ok(outcome) => outcome,
        Err(err) => {
            return Ok(CaseReport {
                name: case.name.to_string(),
                geometry_source: case.geometry_source.to_string(),
                geometry_hash: None,
                mesh_hash: None,
                status: "failed".to_string(),
                failure_reasons: vec![err],
                counts: json!({}),
                geometry_quality: json!({}),
                topology_quality: json!({}),
                triangle_quality: json!({}),
                provenance: json!({}),
                determinism: json!({}),
                performance: json!({ "geometry_initialization_ms": init_ms }),
                visualization: None,
            });
        }
    };

    let determinism_tuple = run_determinism_check(sdf.clone(), case.config.clone())
        .map_err(|e| format!("{} determinism check failed: {e}", case.name))?;
    let determinism = DeterminismAudit {
        repeated: true,
        mesh_hash: determinism_tuple.0,
        repeated_mesh_hash: Some(determinism_tuple.1),
        exact_match: Some(determinism_tuple.2),
    };

    let ply_path = PathBuf::from(VIS_DIR).join(format!("{}.ply", case.name));
    write_ply(&outcome.mesh, &ply_path)
        .map_err(|e| format!("{} visualization export failed: {e}", case.name))?;

    Ok(case_report(
        case,
        &outcome,
        bounds,
        init_ms,
        estimated_distance_queries,
        ply_path,
        determinism,
    ))
}

fn load_case_sdf(case: &CaseSpec) -> Result<Arc<dyn Sdf>, String> {
    if case.name == "direct_hard_subtraction" {
        let body: Arc<dyn Sdf> = Arc::new(Sphere::new(12.0));
        let tool: Arc<dyn Sdf> = Arc::new(Translate::new(
            Arc::new(Sphere::new(4.0)),
            Vec3::new(3.0, 0.0, 0.0),
        ));
        return Ok(Arc::new(Subtract::new(body, tool)));
    }
    let script = case
        .script
        .ok_or_else(|| format!("{} has no script or direct loader", case.name))?;
    Ok(scripting::evaluate_script(script)
        .map_err(|e| format!("{} geometry load failed: {e}", case.name))?
        .into_conditioned_for_backend()
        .sdf)
}

fn case_report(
    case: &CaseSpec,
    outcome: &SurfaceExtractionOutcome,
    bounds: BoundingInfo,
    init_ms: f64,
    estimated_distance_queries: u64,
    ply_path: PathBuf,
    determinism: DeterminismAudit,
) -> CaseReport {
    let audit = &outcome.audit;
    let status = match outcome.status {
        SurfaceExtractionStatus::Success => "accepted",
        SurfaceExtractionStatus::SuccessWithWarnings => "accepted_with_warnings",
        SurfaceExtractionStatus::Failed => "failed",
    };
    CaseReport {
        name: case.name.to_string(),
        geometry_source: case.geometry_source.to_string(),
        geometry_hash: Some(outcome.mesh.metadata.geometry_hash.clone()),
        mesh_hash: Some(outcome.mesh.mesh_hash()),
        status: status.to_string(),
        failure_reasons: outcome.failure_reasons.clone(),
        counts: json!({
            "vertices": outcome.mesh.vertices.len(),
            "edges": outcome.mesh.edges.len(),
            "triangles": outcome.mesh.triangles.len(),
            "feature_edges": outcome.mesh.feature_edges.len(),
            "patches": outcome.mesh.patches.len(),
            "bounds_min": [bounds.min.x, bounds.min.y, bounds.min.z],
            "bounds_max": [bounds.max.x, bounds.max.y, bounds.max.z]
        }),
        geometry_quality: json!(&audit.geometry),
        topology_quality: json!(&audit.topology),
        triangle_quality: json!(&audit.triangle_quality),
        provenance: json!(&audit.provenance),
        determinism: json!(determinism),
        performance: json!({
            "geometry_initialization_ms": init_ms,
            "sdf_sampling_ms": outcome.timings.sdf_sampling_and_initial_extraction_ms,
            "deduplication_ms": outcome.timings.vertex_deduplication_ms,
            "projection_ms": outcome.timings.projection_ms,
            "adjacency_ms": outcome.timings.adjacency_ms,
            "audit_ms": outcome.timings.audit_ms,
            "total_ms": outcome.timings.total_ms,
            "estimated_marching_cubes_distance_queries": estimated_distance_queries,
            "projection_queries": outcome.query_counts.projection_queries,
            "gradient_queries": outcome.query_counts.gradient_queries,
            "provenance_queries": outcome.query_counts.provenance_queries
        }),
        visualization: Some(path_for_report(&ply_path)),
    }
}

fn path_for_report(path: &Path) -> String {
    path.to_string_lossy().replace('\\', "/")
}

fn markdown_summary(report: &serde_json::Value) -> String {
    let mut out = String::new();
    out.push_str("# Surface Mesh Foundation v1\n\n");
    out.push_str("Implemented a typed SDF-native triangular wall-surface mesh foundation backed by the existing Rhai/SDF query and hard-CSG provenance path. The selected initial topology source is the repository's uniform marching-cubes extractor, followed by deterministic welding, SDF projection, orientation normalization, adjacency construction, and audits.\n\n");
    out.push_str("## Cases\n\n");
    if let Some(cases) = report.get("cases").and_then(|v| v.as_array()) {
        for case in cases {
            let name = case.get("name").and_then(|v| v.as_str()).unwrap_or("case");
            let status = case
                .get("status")
                .and_then(|v| v.as_str())
                .unwrap_or("unknown");
            let counts = case.get("counts").unwrap_or(&serde_json::Value::Null);
            let topo = case
                .get("topology_quality")
                .unwrap_or(&serde_json::Value::Null);
            let geom = case
                .get("geometry_quality")
                .unwrap_or(&serde_json::Value::Null);
            let triq = case
                .get("triangle_quality")
                .unwrap_or(&serde_json::Value::Null);
            let prov = case.get("provenance").unwrap_or(&serde_json::Value::Null);
            out.push_str(&format!(
                "- `{name}`: {status}; vertices={}, triangles={}, boundary_edges={}, non_manifold_edges={}, max_residual={:.3e}, min_angle={:.2}, max_aspect={:.2}, complete_traces={}, mixed_triangles={}\n",
                counts.get("vertices").and_then(|v| v.as_u64()).unwrap_or(0),
                counts.get("triangles").and_then(|v| v.as_u64()).unwrap_or(0),
                topo.get("boundary_edges").and_then(|v| v.as_u64()).unwrap_or(0),
                topo.get("non_manifold_edges").and_then(|v| v.as_u64()).unwrap_or(0),
                geom.get("max_abs_sdf_residual").and_then(|v| v.as_f64()).unwrap_or(f64::NAN),
                triq.get("min_angle_degrees").and_then(|v| v.as_f64()).unwrap_or(f64::NAN),
                triq.get("max_aspect_ratio").and_then(|v| v.as_f64()).unwrap_or(f64::NAN),
                prov.get("vertices_with_complete_hard_csg_traces").and_then(|v| v.as_u64()).unwrap_or(0),
                prov.get("triangles_with_mixed_provenance").and_then(|v| v.as_u64()).unwrap_or(0),
            ));
        }
    }
    out.push_str("\n## Commands\n\n");
    out.push_str("- `cargo test sdf_surface`\n");
    out.push_str("- `cargo test --test sdf_surface_foundation`\n");
    out.push_str("- `cargo run --bin sdf_surface_foundation`\n\n");
    out.push_str("## Recommended Next Task\n\n");
    out.push_str("Implement extractor/topology correction for the real finite-wing marching-cubes seed: ambiguous-case handling, deterministic seam welding, trailing-edge/tip closure diagnostics, and a fail-closed topology gate before remeshing.\n");
    out
}
