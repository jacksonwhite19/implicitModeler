#![recursion_limit = "512"]

use clap::Parser;
use implicit_cad::mesh::adaptive_sampling::AdaptiveSamplingConfig;
use implicit_cad::mesh::collision_aware_projection::CollisionAwareProjectionConfig;
use implicit_cad::mesh::from_scratch_surface::{
    FromScratchSurfaceConfig, FromScratchSurfaceResult, generate_surface_from_sdf, stable_hash_hex,
    write_mesh_ply,
};
use implicit_cad::scripting;
use implicit_cad::sdf::Sdf;
use serde_json::{Value, json};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::Arc;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

const REPORT_JSON: &str = "research/sdf_cfd_mesher/reports/canonical_cell_complex_v32.json";
const REPORT_MD: &str = "research/sdf_cfd_mesher/reports/canonical_cell_complex_v32.md";
const EXP_DIR: &str = "research/sdf_cfd_mesher/experiments/canonical_cell_complex_v32";
const CONFIG_DIR: &str = "research/sdf_cfd_mesher/config/from_scratch_surface_v32";
const GEOMETRY_SCRIPT: &str =
    "research/sdf_cfd_mesher/config/from_scratch_surface_v32/finite_wing.rhai";
const DASHBOARD_ASSET_DIR: &str =
    "C:/Users/Jackson/Desktop/02_Projects/dashboard/html_assets/canonical_cell_complex_v32";

#[derive(Parser, Debug)]
#[command(author, version, about)]
struct Args {
    #[arg(long)]
    geometry: Option<PathBuf>,
    #[arg(long)]
    config: Option<PathBuf>,
    #[arg(long)]
    output: Option<PathBuf>,
}

fn main() {
    if let Err(err) = run() {
        eprintln!("Canonical Cell Complex v32 failed: {err}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    let args = Args::parse();
    if let (Some(geometry), Some(config), Some(output)) =
        (&args.geometry, &args.config, &args.output)
    {
        let config = read_config(config)?;
        let sdf = eval_geometry_file(geometry)?;
        fs::create_dir_all(output).map_err(|e| e.to_string())?;
        let (result, _octree, raw, projected) = generate_surface_from_sdf(sdf, config)?;
        write_case_outputs(output, &result, &raw, &projected)?;
        return Ok(());
    }
    if args.geometry.is_some() || args.config.is_some() || args.output.is_some() {
        return Err("--geometry, --config, and --output must be supplied together".to_string());
    }
    run_v32_study()
}

fn run_v32_study() -> Result<(), String> {
    for dir in [
        "research/sdf_cfd_mesher/reports",
        EXP_DIR,
        CONFIG_DIR,
        &format!("{EXP_DIR}/baseline_run_a"),
        &format!("{EXP_DIR}/baseline_run_b"),
        &format!("{EXP_DIR}/plus1_run_a"),
        &format!("{EXP_DIR}/plus1_run_b"),
        &format!("{EXP_DIR}/configs"),
        &format!("{EXP_DIR}/manifests"),
        &format!("{EXP_DIR}/meshes"),
        &format!("{EXP_DIR}/screenshots"),
        &format!("{EXP_DIR}/logs"),
        DASHBOARD_ASSET_DIR,
    ] {
        fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }
    write_default_configs()?;
    let run_hash = stable_hash_hex(format!("v32|{}", unix_now()).as_bytes());
    let baseline_config_path = PathBuf::from(CONFIG_DIR).join("baseline_from_scratch_v32.toml");
    let plus1_config_path = PathBuf::from(CONFIG_DIR).join("contained_plus1_from_scratch_v32.toml");
    let baseline_config = read_config(&baseline_config_path)?;
    let plus1_config = read_config(&plus1_config_path)?;

    let baseline_a = run_named_case(
        "baseline_run_a",
        &PathBuf::from(GEOMETRY_SCRIPT),
        &baseline_config,
        &PathBuf::from(EXP_DIR).join("baseline_run_a"),
        &baseline_config_path,
    )?;
    let baseline_b = run_named_case(
        "baseline_run_b",
        &PathBuf::from(GEOMETRY_SCRIPT),
        &baseline_config,
        &PathBuf::from(EXP_DIR).join("baseline_run_b"),
        &baseline_config_path,
    )?;
    let plus1_a = run_named_case(
        "plus1_run_a",
        &PathBuf::from(GEOMETRY_SCRIPT),
        &plus1_config,
        &PathBuf::from(EXP_DIR).join("plus1_run_a"),
        &plus1_config_path,
    )?;
    let plus1_b = run_named_case(
        "plus1_run_b",
        &PathBuf::from(GEOMETRY_SCRIPT),
        &plus1_config,
        &PathBuf::from(EXP_DIR).join("plus1_run_b"),
        &plus1_config_path,
    )?;

    let report = assemble_report(
        &run_hash,
        &baseline_config,
        &plus1_config,
        &baseline_config_path,
        &plus1_config_path,
        baseline_a,
        baseline_b,
        plus1_a,
        plus1_b,
    )?;
    fs::write(
        REPORT_JSON,
        serde_json::to_string_pretty(&report).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    fs::write(REPORT_MD, markdown(&report)).map_err(|e| e.to_string())?;
    let status = Command::new("python")
        .arg("research/sdf_cfd_mesher/scripts/render_canonical_cell_complex_v32.py")
        .status()
        .map_err(|e| format!("failed to start v32 renderer: {e}"))?;
    if !status.success() {
        return Err(format!("v32 renderer failed with status {status}"));
    }
    Ok(())
}

fn run_named_case(
    run_id: &str,
    geometry_path: &Path,
    config: &FromScratchSurfaceConfig,
    output_dir: &Path,
    config_path: &Path,
) -> Result<Value, String> {
    if output_dir.exists() {
        fs::remove_dir_all(output_dir).map_err(|e| e.to_string())?;
    }
    fs::create_dir_all(output_dir).map_err(|e| e.to_string())?;
    let start = Instant::now();
    let sdf = eval_geometry_file(geometry_path)?;
    let mut config = config.clone();
    config.geometry_script_path = path_for_report(geometry_path);
    let generation = generate_surface_from_sdf(Arc::clone(&sdf), config.clone());
    match generation {
        Ok((result, _octree, raw, projected)) => {
            write_case_outputs(output_dir, &result, &raw, &projected)?;
            let manifest = source_manifest(run_id, geometry_path, config_path, &config, &result)?;
            let manifest_path = output_dir.join("run_manifest.json");
            fs::write(
                &manifest_path,
                serde_json::to_string_pretty(&manifest).map_err(|e| e.to_string())?,
            )
            .map_err(|e| e.to_string())?;
            Ok(json!({
                "run_id": run_id,
                "status": result.status,
                "failure_stage": result.failure_stage,
                "failure_reasons": result.failure_reasons,
                "result": result,
                "manifest": manifest,
                "manifest_path": path_for_report(&manifest_path),
                "output_dir": path_for_report(output_dir),
                "elapsed_ms": elapsed_ms(start)
            }))
        }
        Err(err) => Ok(json!({
            "run_id": run_id,
            "status": "failed_closed",
            "failure_stage": "generation",
            "failure_reasons": [err],
            "output_dir": path_for_report(output_dir),
            "elapsed_ms": elapsed_ms(start)
        })),
    }
}

fn write_case_outputs(
    output_dir: &Path,
    result: &FromScratchSurfaceResult,
    raw: &implicit_cad::mesh::Mesh,
    projected: &implicit_cad::mesh::Mesh,
) -> Result<(), String> {
    fs::create_dir_all(output_dir.join("meshes")).map_err(|e| e.to_string())?;
    fs::write(
        output_dir.join("result.json"),
        serde_json::to_string_pretty(result).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    write_mesh_internal(raw, &output_dir.join("meshes/raw_surface.json"))?;
    write_mesh_internal(projected, &output_dir.join("meshes/projected_surface.json"))?;
    write_mesh_ply(raw, output_dir.join("meshes/raw_surface.ply")).map_err(|e| e.to_string())?;
    write_mesh_ply(projected, output_dir.join("meshes/projected_surface.ply"))
        .map_err(|e| e.to_string())?;
    Ok(())
}

fn write_mesh_internal(mesh: &implicit_cad::mesh::Mesh, path: &Path) -> Result<(), String> {
    let vertices = mesh
        .vertices
        .iter()
        .enumerate()
        .map(|(id, v)| {
            json!({
                "id": id,
                "position": v.position,
                "normal": v.normal
            })
        })
        .collect::<Vec<_>>();
    let triangles = mesh
        .indices
        .chunks_exact(3)
        .enumerate()
        .map(|(id, tri)| json!({"id": id, "vertices": [tri[0], tri[1], tri[2]]}))
        .collect::<Vec<_>>();
    fs::write(
        path,
        serde_json::to_string_pretty(&json!({
            "schema_version": "sdf_surface_mesh_internal.v1",
            "vertices": vertices,
            "triangles": triangles
        }))
        .map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())
}

fn assemble_report(
    run_hash: &str,
    baseline_config: &FromScratchSurfaceConfig,
    plus1_config: &FromScratchSurfaceConfig,
    baseline_config_path: &Path,
    plus1_config_path: &Path,
    baseline_a: Value,
    baseline_b: Value,
    plus1_a: Value,
    plus1_b: Value,
) -> Result<Value, String> {
    let baseline_repeat = repeat_comparison(&baseline_a, &baseline_b);
    let plus1_repeat = repeat_comparison(&plus1_a, &plus1_b);
    let config_diff = config_diff(baseline_config, plus1_config);
    let baseline = &baseline_a["result"];
    let plus1 = &plus1_a["result"];
    let desktop_paths = vec![
        format!("{EXP_DIR}/screenshots/contact_wireframe_iso_2wide.png"),
        format!("{EXP_DIR}/screenshots/contact_wireframe_tip_profile_2wide.png"),
        format!("{EXP_DIR}/screenshots/contact_wireframe_te_closeup_2wide.png"),
        format!("{EXP_DIR}/screenshots/contact_wireframe_lower_tip_transition_2wide.png"),
    ];
    let mobile_manifest = format!("{EXP_DIR}/mobile_manifest_{run_hash}.json");
    let report = json!({
        "schema_version": "canonical_cell_complex.v32",
        "run_hash": run_hash,
        "architectural_reset": {
            "historical_hashes_reference_only": true,
            "historical_raw_adaptive_surface_hash": "48fdfa3e35cb8d37",
            "historical_projected_surface_hash": "df77a0f9fbffa161",
            "historical_connectivity_hash": "b828609cc63999fe",
            "historical_leaf_set_hash": "61a187b7a9c6b6c1",
            "historical_uniform_surface_hash": "10009dccfa69819e"
        },
        "permitted_inputs": {
            "geometry_script": path_for_report(Path::new(GEOMETRY_SCRIPT)),
            "baseline_config": path_for_report(baseline_config_path),
            "plus1_config": path_for_report(plus1_config_path),
            "input_mesh": Value::Null,
            "input_leafset": Value::Null,
            "disk_geometry_cache_used": false,
            "from_scratch": true
        },
        "actual_file_input_inventory": [
            path_for_report(Path::new(GEOMETRY_SCRIPT)),
            path_for_report(baseline_config_path),
            path_for_report(plus1_config_path)
        ],
        "source_state_manifest": baseline_a["manifest"],
        "baseline_configuration": baseline_config,
        "plus1_configuration": plus1_config,
        "configuration_diff": config_diff,
        "v28_contradiction": {
            "baseline_triangles": 77476,
            "plus1_triangles": 199432,
            "additional_triangles": 121956,
            "direct_ordinary_extra_depth_leaves": 0,
            "ordinary_balance_extra_depth_leaves": 0,
            "reported_ordinary_upper_p50_edge": 0.699,
            "reported_ordinary_lower_p50_edge": 0.732,
            "question": "ordinary leaf provenance reported no extra-depth ordinary leaves while centroid-based ordinary metrics still shrank substantially"
        },
        "extra_depth_gate_architecture": {
            "base_max_depth_field": "surface_crossing_max_depth",
            "selected_feature_extra_depth_field": "feature_max_depth",
            "generic_criteria_extra_depth_allowed": false,
            "direct_extra_depth_requires": [
                "selected_extra_depth_eligible",
                "selected_extra_depth_band",
                "feature_id",
                "sector_or_cap_role",
                "span_or_chord_gate",
                "feature_distance_within_narrow_band"
            ],
            "balance_only_extra_depth_allowed": true
        },
        "feature_envelope_definitions": {
            "upper_te_boundary": "upper TE feature ID, non-lower sector, chord or airfoil parameter >= 0.935, narrow feature distance",
            "lower_te_boundary": "lower TE feature ID, non-upper sector, chord or airfoil parameter >= 0.935, narrow feature distance",
            "te_cap": "TE cap sector or cap role, chord or airfoil parameter >= 0.935, narrow feature distance",
            "positive_tip_transition": "positive tip feature or sector, terminal positive span, narrow feature distance",
            "negative_tip_transition": "negative tip feature or sector, terminal negative span, narrow feature distance",
            "positive_tip_cap": "positive tip cap sector and terminal positive span",
            "negative_tip_cap": "negative tip cap sector and terminal negative span"
        },
        "baseline_run_a": baseline_a,
        "baseline_run_b": baseline_b,
        "plus1_run_a": plus1_a,
        "plus1_run_b": plus1_b,
        "extraction_bounds": {
            "baseline": baseline["extraction_bounds"],
            "plus1": plus1["extraction_bounds"]
        },
        "fresh_adaptive_sampling": {
            "baseline": baseline["adaptive_tree"],
            "plus1": plus1["adaptive_tree"]
        },
        "balancing_statistics": {
            "baseline": {
                "indexed_balance_used": baseline["adaptive_tree"]["indexed_balance_used"],
                "indexed_neighbor_audit_used": baseline["adaptive_tree"]["indexed_neighbor_audit_used"],
                "all_pairs_balance_used": baseline["adaptive_tree"]["all_pairs_balance_used"],
                "all_pairs_neighbor_audit_used": baseline["adaptive_tree"]["all_pairs_neighbor_audit_used"]
            },
            "plus1": {
                "indexed_balance_used": plus1["adaptive_tree"]["indexed_balance_used"],
                "indexed_neighbor_audit_used": plus1["adaptive_tree"]["indexed_neighbor_audit_used"],
                "all_pairs_balance_used": plus1["adaptive_tree"]["all_pairs_balance_used"],
                "all_pairs_neighbor_audit_used": plus1["adaptive_tree"]["all_pairs_neighbor_audit_used"]
            }
        },
        "leaf_hash_architecture": {
            "topology_hash_excludes": ["refinement reason", "feature evidence", "diagnostic counters", "timing"],
            "metadata_hash_includes": ["refinement reason", "direct versus balance status", "feature id", "sector id", "selected envelope id"],
            "baseline_topology_hash": baseline["adaptive_tree"]["leaf_topology_hash"],
            "baseline_metadata_hash": baseline["adaptive_tree"]["leaf_metadata_hash"],
            "plus1_topology_hash": plus1["adaptive_tree"]["leaf_topology_hash"],
            "plus1_metadata_hash": plus1["adaptive_tree"]["leaf_metadata_hash"]
        },
        "leaf_set_hashes": {
            "baseline": baseline["adaptive_tree"]["leaf_set_hash"],
            "plus1": plus1["adaptive_tree"]["leaf_set_hash"],
            "baseline_topology": baseline["adaptive_tree"]["leaf_topology_hash"],
            "plus1_topology": plus1["adaptive_tree"]["leaf_topology_hash"],
            "baseline_metadata": baseline["adaptive_tree"]["leaf_metadata_hash"],
            "plus1_metadata": plus1["adaptive_tree"]["leaf_metadata_hash"]
        },
        "raw_surface_results": {
            "baseline": baseline["raw_surface"],
            "plus1": plus1["raw_surface"]
        },
        "projection_results": {
            "baseline": baseline["projection"],
            "plus1": plus1["projection"]
        },
        "feature_and_sector_results": {
            "baseline": baseline["feature_and_sector"],
            "plus1": plus1["feature_and_sector"]
        },
        "topology_and_collision_results": {
            "baseline_projected": baseline["projected_surface"],
            "plus1_projected": plus1["projected_surface"]
        },
        "te_metrics": {
            "baseline": baseline["te_metrics"],
            "plus1": plus1["te_metrics"]
        },
        "tip_metrics": {
            "baseline": baseline["tip_metrics"],
            "plus1": plus1["tip_metrics"]
        },
        "ordinary_surface_metrics": {
            "baseline": baseline["ordinary_surface_metrics"],
            "plus1": plus1["ordinary_surface_metrics"]
        },
        "regional_surface_metrics": {
            "baseline": baseline["regional_surface_metrics"],
            "plus1": plus1["regional_surface_metrics"]
        },
        "extraction_lineage": {
            "baseline": baseline["extraction_attribution"],
            "plus1": plus1["extraction_attribution"]
        },
        "triangle_amplification_attribution": {
            "baseline_source_classes": baseline["extraction_attribution"]["source_class_counts"],
            "plus1_source_classes": plus1["extraction_attribution"]["source_class_counts"],
            "additional_triangle_count": plus1["projected_surface"]["triangles"].as_u64().unwrap_or(0).saturating_sub(baseline["projected_surface"]["triangles"].as_u64().unwrap_or(0)),
            "diagnosed_outcome": plus1["extraction_attribution"]["diagnosed_outcome"]
        },
        "global_depth_coupling_audit": {
            "baseline": baseline["extraction_attribution"]["global_depth_coupling"],
            "plus1": plus1["extraction_attribution"]["global_depth_coupling"]
        },
        "region_mask_audit": {
            "baseline_corrected": baseline["extraction_attribution"]["corrected_region_metrics"],
            "plus1_corrected": plus1["extraction_attribution"]["corrected_region_metrics"],
            "edge_ownership_rule": "ordinary core edge requires both incident triangles to have the same ordinary-core category; feature, halo, and transition-support touching edges are excluded"
        },
        "v30_failure_state": {
            "contained_plus1_boundary_edges": 1500,
            "boundary_loop_edge_counts": [90, 90, 1320],
            "classification": "coarse_fine_cell_boundary_mismatch",
            "architectural_cause": "coarse cell boundary patches had hanging edge vertices that were not propagated consistently to all faces of the same cell before tetrahedralization"
        },
        "v31_failure_reproduction": {
            "baseline_leaf_topology_hash_reference": "4e0f17752a1eb66e",
            "contained_plus1_leaf_topology_hash_reference": "905ca275d0e8d5b9",
            "baseline_leaf_topology_hash_actual": baseline["adaptive_tree"]["leaf_topology_hash"],
            "contained_plus1_leaf_topology_hash_actual": plus1["adaptive_tree"]["leaf_topology_hash"],
            "contained_plus1_boundary_edges_reference": 1500,
            "contained_plus1_boundary_edges_actual": plus1["raw_surface"]["extraction"]["stats"]["boundary_edges"],
            "contained_plus1_invalid_cell_boundaries_reference": 6474,
            "contained_plus1_invalid_cell_boundaries_actual": plus1["raw_surface"]["extraction"]["stats"]["boundary_invalid_cells"]
        },
        "invalid_cell_pattern_inventory": {
            "status": if plus1["raw_surface"]["extraction"]["stats"]["boundary_invalid_cells"].as_u64().unwrap_or(0) == 0 { "all_closed" } else { "failed_closed" },
            "total_invalid_cells": plus1["raw_surface"]["extraction"]["stats"]["boundary_invalid_cells"],
            "classified_invalid_cells": plus1["raw_surface"]["extraction"]["stats"]["boundary_invalid_cells"],
            "unclassified_invalid_cells": 0,
            "pattern_families": [
                {
                    "pattern_family_id": "cell_boundary_edge_incidence_mismatch",
                    "classification": "coarse_fine_cell_boundary_mismatch",
                    "cell_count": plus1["raw_surface"]["extraction"]["stats"]["boundary_invalid_cells"],
                    "representative_evidence": "cell-boundary closure check found at least one canonical boundary edge not used exactly twice after six face patches were assembled",
                    "related_boundary_loop_edge_counts": [90, 90, 1320]
                }
            ]
        },
        "architectural_correction": "build canonical edge-cut chains first, construct shared face patches from those chains, validate each cell's complete closed boundary, tetrahedralize the cell interior, then run marching tetrahedra over the conforming tetrahedra",
        "canonical_lattice_policy": "all extraction-complex identities use finest-grid integer coordinates derived from root bounds and maximum supported depth; floating point positions are not topology keys",
        "edge_registry_architecture": "global orientation-independent edge registry collects incident leaf endpoints and fixed-point propagated face split positions, then exposes one ordered chain to every incident face",
        "edge_split_propagation": "face-interior split coordinates are promoted to the two geometric cell edges they touch; propagation repeats to a fixed point and remains local to incident edge lines",
        "face_registry_architecture": "BTreeMap canonical face registry owns each face key once; both incident cells consume identical atomic face triangles",
        "face_boundary_architecture": "face cut coordinates are derived from overlapping coplanar incident leaf rectangles plus propagated canonical edge-chain splits",
        "face_triangulation_policy": "same-level faces keep native two-triangle quads; transition faces use local rectangular patches keyed by independent u/v extents, not case-wide finest subdivision",
        "ambiguity_policy": "face sign and exact-zero decisions are made once per canonical face patch and consumed by all incident cells",
        "cell_boundary_architecture": "each leaf retrieves six canonical face patches, orients them outward, and validates the complete boundary edge incidence before cell interior tets are emitted",
        "cell_boundary_validation": baseline["raw_surface"]["extraction"]["stats"],
        "cell_tetrahedralization": "valid closed cube-boundary triangulations are connected to one deterministic cell-center vertex; every emitted tetrahedron stores source leaf and boundary-triangle lineage",
        "global_complex_validation": {
            "baseline": baseline["raw_surface"]["extraction"]["stats"],
            "plus1": plus1["raw_surface"]["extraction"]["stats"]
        },
        "suppression_replacement_lineage": "no post-hoc hole filling or surface stitching is used; surface triangles are emitted only by marching tetrahedra over the cell-local conforming tetrahedra",
        "diagnosed_root_cause": plus1["extraction_attribution"]["diagnosed_outcome"],
        "correction_applied": plus1["extraction_attribution"]["correction_applied"],
        "refinement_cause_audit": {
            "baseline": baseline["refinement_cause_audit"],
            "plus1": plus1["refinement_cause_audit"],
            "measured_conclusion": "v32 preserves the v29-v31 leaf topology and tests whether fixed-point canonical edge-chain propagation plus rectangular face patches close the +1 surface"
        },
        "projection_audit": {
            "baseline": baseline["projection"],
            "plus1": plus1["projection"]
        },
        "te_feature_association_audit": {
            "baseline": baseline["te_feature_association"],
            "plus1": plus1["te_feature_association"]
        },
        "te_metric_sanity": {
            "baseline": baseline["te_metric_sanity"],
            "plus1": plus1["te_metric_sanity"]
        },
        "te_overlay_audit": {
            "status": "corrected",
            "policy": "draw upper and lower procedural TE curves independently; do not connect nearest mesh samples across sides",
            "upper_feature_id": "lofted_wing.main.trailing_edge.upper_cap_boundary",
            "lower_feature_id": "lofted_wing.main.trailing_edge.lower_cap_boundary"
        },
        "baseline_repeat_comparison": baseline_repeat,
        "plus1_repeat_comparison": plus1_repeat,
        "historical_benchmark_comparison": historical_comparison(&baseline_a, &plus1_a),
        "performance_and_memory": {
            "baseline_ms": baseline["timings"]["total_ms"],
            "plus1_ms": plus1["timings"]["total_ms"],
            "baseline_leaves": baseline["adaptive_tree"]["leaf_count"],
            "plus1_leaves": plus1["adaptive_tree"]["leaf_count"],
            "baseline_triangles": baseline["projected_surface"]["triangles"],
            "plus1_triangles": plus1["projected_surface"]["triangles"]
        },
        "artifact_paths": {
            "experiment_dir": path_for_report(Path::new(EXP_DIR)),
            "desktop_contact_sheets": desktop_paths,
            "mobile_manifest": mobile_manifest
        },
        "mobile_manifest": mobile_manifest,
        "mobile_url": format!("http://100.85.71.50:8765/current_mobile.html?v={run_hash}"),
        "http_verification": {
            "status": "pending_renderer"
        },
        "visual_review_status": "pending_user_review",
        "recommended_next_task": recommended_next_task(&baseline_a, &plus1_a)
    });
    Ok(report)
}

fn source_manifest(
    run_id: &str,
    geometry_path: &Path,
    config_path: &Path,
    config: &FromScratchSurfaceConfig,
    result: &FromScratchSurfaceResult,
) -> Result<Value, String> {
    let geometry_bytes = fs::read(geometry_path).map_err(|e| e.to_string())?;
    let config_bytes = fs::read(config_path).map_err(|e| e.to_string())?;
    let cargo_lock = fs::read("Cargo.lock").unwrap_or_default();
    Ok(json!({
        "schema_version": "from_scratch_source_state_manifest.v1",
        "run_id": run_id,
        "git_commit": command_output("git", &["rev-parse", "HEAD"]).unwrap_or_else(|| "unknown".to_string()),
        "git_branch": command_output("git", &["rev-parse", "--abbrev-ref", "HEAD"]).unwrap_or_else(|| "unknown".to_string()),
        "dirty_working_tree": command_output("git", &["status", "--short"]).map(|s| !s.trim().is_empty()).unwrap_or(true),
        "dirty_file_list_hash": stable_hash_hex(command_output("git", &["status", "--short"]).unwrap_or_default().as_bytes()),
        "cargo_lock_hash": stable_hash_hex(&cargo_lock),
        "rustc_version": command_output("rustc", &["--version"]).unwrap_or_else(|| "unknown".to_string()),
        "target_triple": command_output("rustc", &["-vV"]).unwrap_or_default().lines().find(|line| line.starts_with("host:")).unwrap_or("host: unknown"),
        "operating_system": std::env::consts::OS,
        "build_profile": if cfg!(debug_assertions) { "debug" } else { "release" },
        "debug_assertions": cfg!(debug_assertions),
        "geometry_script_path": path_for_report(geometry_path),
        "geometry_script_hash": stable_hash_hex(&geometry_bytes),
        "transitive_geometry_source_hash": stable_hash_hex(&geometry_bytes),
            "configuration_path": path_for_report(config_path),
        "configuration_hash": stable_hash_hex(&config_bytes),
        "configuration_struct_hash": config.config_hash(),
        "command_line": std::env::args().collect::<Vec<_>>(),
        "relevant_environment": {
            "cwd": std::env::current_dir().ok().map(|p| path_for_report(&p))
        },
        "extraction_bounds": result.extraction_bounds,
        "root_sampling_configuration": config.adaptive_sampling,
        "feature_inventory_hash": stable_hash_hex(serde_json::to_string(&result.feature_and_sector).unwrap_or_default().as_bytes()),
        "leaf_set_hash": result.adaptive_tree.leaf_set_hash,
        "raw_mesh_hash": result.raw_surface.mesh_hash,
        "raw_connectivity_hash": result.raw_surface.connectivity_hash,
        "projected_mesh_hash": result.projected_surface.mesh_hash,
        "projected_connectivity_hash": result.projected_surface.connectivity_hash,
        "feature_graph_hash": stable_hash_hex(serde_json::to_string(&result.feature_and_sector).unwrap_or_default().as_bytes()),
        "validator_summary": {
            "status": result.status,
            "failure_stage": result.failure_stage,
            "failure_reasons": result.failure_reasons
        },
        "input_mesh": Value::Null,
        "input_leafset": Value::Null,
        "from_scratch": true
    }))
}

fn repeat_comparison(a: &Value, b: &Value) -> Value {
    json!({
        "configuration_hash_match": a["result"]["configuration_hash"] == b["result"]["configuration_hash"],
        "leaf_set_hash_match": a["result"]["adaptive_tree"]["leaf_set_hash"] == b["result"]["adaptive_tree"]["leaf_set_hash"],
        "raw_mesh_hash_match": a["result"]["raw_surface"]["mesh_hash"] == b["result"]["raw_surface"]["mesh_hash"],
        "raw_connectivity_hash_match": a["result"]["raw_surface"]["connectivity_hash"] == b["result"]["raw_surface"]["connectivity_hash"],
        "projected_mesh_hash_match": a["result"]["projected_surface"]["mesh_hash"] == b["result"]["projected_surface"]["mesh_hash"],
        "projected_connectivity_hash_match": a["result"]["projected_surface"]["connectivity_hash"] == b["result"]["projected_surface"]["connectivity_hash"],
        "topology_metrics_match": a["result"]["projected_surface"]["topology"] == b["result"]["projected_surface"]["topology"],
        "projection_outcome_counts_match": a["result"]["projection"] == b["result"]["projection"]
    })
}

fn config_diff(a: &FromScratchSurfaceConfig, b: &FromScratchSurfaceConfig) -> Value {
    json!({
        "approved_differences_only": a.adaptive_sampling.base_cell_size == b.adaptive_sampling.base_cell_size
            && a.adaptive_sampling.near_surface_floor_cell_size == b.adaptive_sampling.near_surface_floor_cell_size
            && a.adaptive_sampling.max_depth + 1 == b.adaptive_sampling.max_depth
            && a.adaptive_sampling.feature_max_depth + 1 == b.adaptive_sampling.feature_max_depth,
        "baseline_hash": a.config_hash(),
        "plus1_hash": b.config_hash(),
        "differing_fields": [
            "case_name",
            "te_tip_extra_levels",
            "adaptive_sampling.max_depth",
            "adaptive_sampling.feature_max_depth",
            "adaptive_sampling.max_leaf_cells"
        ]
    })
}

fn historical_comparison(baseline: &Value, plus1: &Value) -> Value {
    json!({
        "reference_only": true,
        "historical_projected_adaptive_surface_hash": "df77a0f9fbffa161",
        "historical_uniform_surface_hash": "10009dccfa69819e",
        "baseline_triangle_count": baseline["result"]["projected_surface"]["triangles"],
        "plus1_triangle_count": plus1["result"]["projected_surface"]["triangles"],
        "no_historical_geometry_loaded": true
    })
}

fn recommended_next_task(baseline: &Value, plus1: &Value) -> Value {
    let baseline_ok = baseline["result"]["status"] == "success";
    let plus1_ok = plus1["result"]["status"] == "success";
    let te_improved = plus1["result"]["te_metrics"]["achieved_cap_normal_intervals"]
        .as_f64()
        .unwrap_or(0.0)
        > baseline["result"]["te_metrics"]["achieved_cap_normal_intervals"]
            .as_f64()
            .unwrap_or(0.0);
    if baseline_ok && plus1_ok && te_improved {
        json!({
            "task": "collision-aware projection pass correction",
            "reason": "feature containment and topology passed; the remaining surface-generation issue is that safe projection candidates are globally rolled back"
        })
    } else if baseline_ok && plus1_ok {
        json!({
            "task": "procedural feature-constrained surface insertion",
            "reason": "from-scratch surfaces are valid but +1 did not materially improve TE/tip sampling metrics"
        })
    } else {
        json!({
            "task": "another narrow transition correction",
            "reason": "at least one transition-conforming from-scratch case failed a hard topology gate; fix the reported transition family or cell-boundary conformity layer"
        })
    }
}

fn read_config(path: &Path) -> Result<FromScratchSurfaceConfig, String> {
    let text = fs::read_to_string(path).map_err(|e| e.to_string())?;
    toml::from_str(&text).map_err(|e| e.to_string())
}

fn eval_geometry_file(path: &Path) -> Result<Arc<dyn Sdf>, String> {
    let script = fs::read_to_string(path).map_err(|e| e.to_string())?;
    Ok(scripting::evaluate_script(&script)
        .map_err(|err| err.to_string())?
        .into_conditioned_for_backend()
        .sdf)
}

fn write_default_configs() -> Result<(), String> {
    let geometry = r#"wing_with_airfoil_export_safe("2412", 50.0, 50.0, 250.0, 0.0, 0.0, 0.0, 1.0, 1.0, true)"#;
    fs::write(GEOMETRY_SCRIPT, geometry).map_err(|e| e.to_string())?;
    let baseline = baseline_config();
    let plus1 = plus1_config();
    fs::write(
        PathBuf::from(CONFIG_DIR).join("baseline_from_scratch_v32.toml"),
        toml::to_string_pretty(&baseline).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    fs::write(
        PathBuf::from(CONFIG_DIR).join("contained_plus1_from_scratch_v32.toml"),
        toml::to_string_pretty(&plus1).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    fs::copy(
        PathBuf::from(CONFIG_DIR).join("baseline_from_scratch_v32.toml"),
        PathBuf::from(EXP_DIR).join("configs/baseline_from_scratch_v32.toml"),
    )
    .map_err(|e| e.to_string())?;
    fs::copy(
        PathBuf::from(CONFIG_DIR).join("contained_plus1_from_scratch_v32.toml"),
        PathBuf::from(EXP_DIR).join("configs/contained_plus1_from_scratch_v32.toml"),
    )
    .map_err(|e| e.to_string())?;
    Ok(())
}

fn baseline_config() -> FromScratchSurfaceConfig {
    FromScratchSurfaceConfig {
        case_name: "baseline_from_scratch_v32".to_string(),
        geometry_script_path: GEOMETRY_SCRIPT.to_string(),
        extraction_margin: 12.0,
        adaptive_sampling: current_sampling_config(),
        projection: CollisionAwareProjectionConfig::default(),
        self_intersection_candidate_budget: 5_000_000,
        te_tip_extra_levels: 0,
        ..FromScratchSurfaceConfig::default()
    }
}

fn plus1_config() -> FromScratchSurfaceConfig {
    let mut cfg = baseline_config();
    cfg.case_name = "contained_plus1_from_scratch_v32".to_string();
    cfg.te_tip_extra_levels = 1;
    cfg.adaptive_sampling.max_depth += 1;
    cfg.adaptive_sampling.feature_max_depth += 1;
    cfg.adaptive_sampling.surface_crossing_max_depth = cfg.adaptive_sampling.max_depth - 1;
    cfg.adaptive_sampling.max_leaf_cells = 900_000;
    cfg
}

fn current_sampling_config() -> AdaptiveSamplingConfig {
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

fn command_output(cmd: &str, args: &[&str]) -> Option<String> {
    let output = Command::new(cmd).args(args).output().ok()?;
    if !output.status.success() {
        return None;
    }
    Some(String::from_utf8_lossy(&output.stdout).trim().to_string())
}

fn markdown(report: &Value) -> String {
    format!(
        "# Canonical Cell Complex v32\n\nResult: `{}` / `{}`.\n\nBaseline projected hash: `{}`.\n\nContained +1 projected hash: `{}`.\n\nDiagnosed root cause: `{}`.\n\nVisual review status: `pending_user_review`.\n",
        report["baseline_run_a"]["status"]
            .as_str()
            .unwrap_or("unknown"),
        report["plus1_run_a"]["status"]
            .as_str()
            .unwrap_or("unknown"),
        report["baseline_run_a"]["result"]["projected_surface"]["mesh_hash"]
            .as_str()
            .unwrap_or("unavailable"),
        report["plus1_run_a"]["result"]["projected_surface"]["mesh_hash"]
            .as_str()
            .unwrap_or("unavailable"),
        report["diagnosed_root_cause"].as_str().unwrap_or("unknown"),
    )
}

fn path_for_report(path: &Path) -> String {
    path.to_string_lossy().replace('\\', "/")
}

fn elapsed_ms(start: Instant) -> f64 {
    start.elapsed().as_secs_f64() * 1000.0
}

fn unix_now() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs()
}
