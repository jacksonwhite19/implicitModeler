use implicit_cad::mesh::adaptive_marching_tets::{
    extract_mesh_adaptive_marching_tets, summarize_extraction,
};
use implicit_cad::mesh::adaptive_sampling::{AdaptiveSamplingConfig, build_adaptive_octree};
use implicit_cad::scripting;
use implicit_cad::sdf::Sdf;
use serde_json::json;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Instant;

const REPORT_JSON: &str = "research/sdf_cfd_mesher/reports/adaptive_marching_tets_v16.json";
const REPORT_MD: &str = "research/sdf_cfd_mesher/reports/adaptive_marching_tets_v16.md";
const EXP_DIR: &str = "research/sdf_cfd_mesher/experiments/adaptive_marching_tets_v16";
const MESH_DIR: &str = "research/sdf_cfd_mesher/experiments/adaptive_marching_tets_v16/meshes";
const WING_SCRIPT: &str =
    r#"wing_with_airfoil_export_safe("2412", 50.0, 50.0, 250.0, 0.0, 0.0, 0.0, 1.0, 1.0, true)"#;

fn main() {
    if let Err(err) = run() {
        eprintln!("adaptive marching tets v16 failed: {err}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    for dir in ["research/sdf_cfd_mesher/reports", EXP_DIR, MESH_DIR] {
        fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }

    let total = Instant::now();
    let sphere = run_case("sphere", eval_sdf("sphere(11.7)")?, small_config(), true)?;
    let hard_subtraction = run_case(
        "hard_subtraction",
        eval_sdf("subtract(sphere(11.7), translate(sphere(3.7), 7.6, 0.0, 0.0))")?,
        small_config(),
        true,
    )?;
    let finite_wing = run_case(
        "finite_wing_current_c",
        eval_sdf(WING_SCRIPT)?,
        current_c_config(),
        false,
    )?;
    let current_c_raw_topology_passed = finite_wing["status"] == "success";
    let post_extraction_artifact_gates_wired = false;
    let all_gates_passed = current_c_raw_topology_passed
        && sphere["status"] == "success"
        && hard_subtraction["status"] == "success"
        && post_extraction_artifact_gates_wired;

    let screenshots = Vec::<String>::new();

    let report = json!({
        "schema_version": "adaptive_marching_tets.v16",
        "architecture": {
            "integer_lattice": "GridPointKey with denominator_level; finest lattice from AdaptiveOctree max level",
            "edge_registry": "TetEdgeKey canonicalizes tetrahedron edge endpoints for shared zero-crossing vertices",
            "face_registry": "finest-lattice CanonicalFaceKey entries shared by incident leaves",
            "face_triangulation": "unit finest-lattice face quads split deterministically with parity diagonal",
            "cell_tetrahedralization": "outward cell boundary triangles plus one deterministic cell center",
            "marching_tetrahedra": "all 16 sign cases handled through 1/2/3 inside count logic",
            "crossing_identity": "zero vertices keyed by canonical TetEdgeKey, not rounded coordinates",
            "old_bridge_path_used": false
        },
        "fixtures": {
            "foundational": {
                "status": "implemented",
                "tests": [
                    "tet_edge_key_canonicalizes_endpoint_order",
                    "face_registry_has_shared_entries_for_balanced_octree",
                    "sphere_marching_tets_extracts_deterministic_surface"
                ]
            }
        },
        "sphere": sphere,
        "hard_subtraction": hard_subtraction,
        "finite_wing_current_c": finite_wing,
        "post_extraction_gates": {
            "projection_stage": "not_wired",
            "self_intersection_audit": "not_wired",
            "fold_audit": "not_wired",
            "feature_sector_attachment": "not_wired",
            "all_required_gates_passed": all_gates_passed
        },
        "depth_study": {
            "te4": {"status": "not_run", "reason": if all_gates_passed { "deferred after current-C architecture proof" } else { "projection/fold/self-intersection gates are not wired for v16" }},
            "te6": {"status": "not_run", "reason": if all_gates_passed { "deferred after current-C architecture proof" } else { "projection/fold/self-intersection gates are not wired for v16" }},
            "te8": {"status": "not_run", "reason": if all_gates_passed { "deferred after current-C architecture proof" } else { "projection/fold/self-intersection gates are not wired for v16" }}
        },
        "performance": {
            "total_runtime_ms": elapsed_ms(total),
            "v15_bridge_timeout_s": 300,
            "per_face_mesh_clone_used": false
        },
        "screenshots": screenshots,
        "visual_review_status": "pending_user_review_after_valid_contact_sheets",
        "recommended_next_task": recommended_next_task(all_gates_passed, current_c_raw_topology_passed, &finite_wing)
    });
    fs::write(
        REPORT_JSON,
        serde_json::to_string_pretty(&report).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    fs::write(REPORT_MD, markdown(&report)).map_err(|e| e.to_string())?;
    Ok(())
}

fn run_case(
    name: &str,
    sdf: Arc<dyn Sdf>,
    config: AdaptiveSamplingConfig,
    write_mesh: bool,
) -> Result<serde_json::Value, String> {
    let start = Instant::now();
    let octree = build_adaptive_octree(sdf.as_ref(), &config)?;
    let leaf_hash = octree.octree_hash();
    let extraction_start = Instant::now();
    let (raw, stats) = extract_mesh_adaptive_marching_tets(sdf.as_ref(), &octree);
    let extraction_ms = elapsed_ms(extraction_start);
    let summary = summarize_extraction(&raw, stats.clone());
    let mesh_path = if write_mesh || name == "finite_wing_current_c" {
        let path = PathBuf::from(MESH_DIR).join(format!("{name}.ply"));
        write_raw_ply(&raw, &path)?;
        Some(path_for_report(&path))
    } else {
        None
    };
    Ok(json!({
        "name": name,
        "leaf_hash": leaf_hash,
        "configuration": config,
        "octree": {
            "leaf_count": octree.leaves.len(),
            "summary": octree.summary,
        },
        "mesh": summary.mesh,
        "stats": summary.stats,
        "topology_by_stage": [
            {
                "stage": "raw_marching_tets",
                "vertices": raw.vertices.len(),
                "triangles": raw.indices.len() / 3,
                "boundary_edges": summary.stats.boundary_edges,
                "non_manifold_edges": summary.stats.non_manifold_edges,
                "duplicate_triangles": summary.stats.duplicate_triangles,
                "degenerate_triangles": summary.stats.degenerate_triangles,
                "hash": summary.mesh.mesh_hash
            }
        ],
        "status": summary.status,
        "failure_reasons": summary.failure_reasons,
        "mesh_path": mesh_path,
        "runtime_ms": elapsed_ms(start),
        "extraction_ms": extraction_ms,
    }))
}

fn small_config() -> AdaptiveSamplingConfig {
    AdaptiveSamplingConfig {
        min_depth: 2,
        max_depth: 3,
        base_cell_size: 6.0,
        bounds_padding: 4.0,
        max_leaf_cells: 16_384,
        balance_face_neighbors: true,
        ..AdaptiveSamplingConfig::default()
    }
}

fn current_c_config() -> AdaptiveSamplingConfig {
    AdaptiveSamplingConfig {
        min_depth: 2,
        max_depth: 7,
        bounds_padding: 12.0,
        base_cell_size: 5.269230842590332,
        near_surface_floor_cell_size: Some(5.269230842590332),
        near_surface_floor_band_factor: 1.1,
        near_surface_floor_max_depth: 6,
        max_leaf_cells: 450_000,
        surface_band_factor: 1.1,
        surface_band_max_depth: 6,
        surface_crossing_max_depth: 7,
        complexity_narrow_band_factor: 0.75,
        complexity_max_depth: 6,
        feature_query_band_factor: 1.2,
        feature_max_depth: 7,
        normal_divergence_degrees: 38.0,
        interpolation_error_tolerance: 0.08,
        normal_interpolation_error_degrees: 32.0,
        required_feature_intervals: 4.0,
        feature_band_width: 4.5,
        leading_edge_band_width: 2.25,
        root_guide_band_width: 2.25,
        trailing_edge_thickness: 1.0,
        balance_face_neighbors: true,
        max_balance_passes: 5,
    }
}

fn eval_sdf(script: &str) -> Result<Arc<dyn Sdf>, String> {
    Ok(scripting::evaluate_script(script)
        .map_err(|err| err.to_string())?
        .into_conditioned_for_backend()
        .sdf)
}

fn write_raw_ply(mesh: &implicit_cad::mesh::Mesh, path: &Path) -> Result<(), String> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
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
    fs::write(path, out).map_err(|e| e.to_string())
}

fn recommended_next_task(
    all_gates_passed: bool,
    current_c_raw_topology_passed: bool,
    finite_wing: &serde_json::Value,
) -> serde_json::Value {
    if all_gates_passed {
        json!({
            "task": "TE-4/6/8 depth study completion",
            "reason": "current C passed with adaptive marching tetrahedra"
        })
    } else if current_c_raw_topology_passed {
        json!({
            "task": "Projection and artifact validation integration for adaptive marching-tets current C",
            "reason": "raw conforming marching-tet topology passed, but projection, self-intersection, fold, and feature-sector gates are not wired"
        })
    } else {
        let reasons = finite_wing["failure_reasons"].clone();
        json!({
            "task": "marching-tetrahedra correction for the first failing current-C stage",
            "reason": reasons
        })
    }
}

fn markdown(report: &serde_json::Value) -> String {
    let case_line = |name: &str, label: &str| {
        let case = &report[name];
        format!(
            "- {}: `{}`; tris `{}`; verts `{}`; boundary `{}`; non-manifold `{}`; degenerate `{}`; hash `{}`\n",
            label,
            case["status"].as_str().unwrap_or("unknown"),
            case["mesh"]["triangles"].as_u64().unwrap_or(0),
            case["mesh"]["vertices"].as_u64().unwrap_or(0),
            case["stats"]["boundary_edges"].as_u64().unwrap_or(0),
            case["stats"]["non_manifold_edges"].as_u64().unwrap_or(0),
            case["stats"]["degenerate_triangles"].as_u64().unwrap_or(0),
            case["mesh"]["mesh_hash"].as_str().unwrap_or("unknown")
        )
    };
    format!(
        "# Adaptive Marching Tetrahedra v16\n\n\
Result: `{}` for current finite-wing C raw extraction.\n\n\
## Architecture\n\n\
The v16 path converts the balanced adaptive octree into a conforming finest-lattice face registry, \
star-tetrahedralizes each valid leaf boundary through the cell center, extracts the zero set with \
marching tetrahedra, and assembles zero-crossing vertices by canonical tetrahedral edge identity. \
It does not use the v13-v15 post-extraction bridge.\n\n\
## Cases\n\n{}{}{}\
## Deferred Gates\n\n\
TE-4, TE-6, and TE-8 were not run in this pass. Projection, fold, self-intersection, and feature-sector stages \
still need to be wired as explicit post-extraction gates before deeper feature refinement or macro-shell work.\n\n\
## Performance\n\n\
Total audit runtime: `{:.3}` ms. Current-C tets: `{}`. Current-C face registry entries: `{}`. \
Per-face whole-mesh cloning: `{}`.\n\n\
Recommended next task: `{}`.\n",
        report["finite_wing_current_c"]["status"]
            .as_str()
            .unwrap_or("unknown"),
        case_line("sphere", "Sphere"),
        case_line("hard_subtraction", "Hard subtraction"),
        case_line("finite_wing_current_c", "Finite wing current C"),
        report["performance"]["total_runtime_ms"]
            .as_f64()
            .unwrap_or(0.0),
        report["finite_wing_current_c"]["stats"]["tetrahedra"]
            .as_u64()
            .unwrap_or(0),
        report["finite_wing_current_c"]["stats"]["face_registry_entries"]
            .as_u64()
            .unwrap_or(0),
        report["performance"]["per_face_mesh_clone_used"]
            .as_bool()
            .unwrap_or(false),
        report["recommended_next_task"]["task"]
            .as_str()
            .unwrap_or("unknown")
    )
}

fn path_for_report(path: &Path) -> String {
    path.to_string_lossy().replace('\\', "/")
}

fn elapsed_ms(start: Instant) -> f64 {
    start.elapsed().as_secs_f64() * 1000.0
}
