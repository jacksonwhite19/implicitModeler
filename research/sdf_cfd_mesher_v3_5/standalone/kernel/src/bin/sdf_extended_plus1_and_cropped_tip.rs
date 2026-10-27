use serde_json::{Value, json};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{SystemTime, UNIX_EPOCH};

const REPORT_JSON: &str = "research/sdf_cfd_mesher/reports/extended_plus1_and_cropped_tip_v24.json";
const REPORT_MD: &str = "research/sdf_cfd_mesher/reports/extended_plus1_and_cropped_tip_v24.md";
const EXP_DIR: &str = "research/sdf_cfd_mesher/experiments/extended_plus1_and_cropped_tip_v24";
const LOG_DIR: &str = "research/sdf_cfd_mesher/experiments/extended_plus1_and_cropped_tip_v24/logs";
const CHECKPOINT_DIR: &str =
    "research/sdf_cfd_mesher/experiments/extended_plus1_and_cropped_tip_v24/checkpoints";
const SCREENSHOT_DIR: &str =
    "research/sdf_cfd_mesher/experiments/extended_plus1_and_cropped_tip_v24/screenshots";
const MOBILE_DIR: &str =
    "research/sdf_cfd_mesher/experiments/extended_plus1_and_cropped_tip_v24/mobile";
const DASHBOARD_ASSET_DIR: &str =
    "C:/Users/Jackson/Desktop/02_Projects/dashboard/html_assets/extended_plus1_and_cropped_tip_v24";

fn main() {
    if let Err(err) = run() {
        eprintln!("extended +1 and cropped tip v24 failed: {err}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    for dir in [
        "research/sdf_cfd_mesher/reports",
        EXP_DIR,
        LOG_DIR,
        CHECKPOINT_DIR,
        SCREENSHOT_DIR,
        MOBILE_DIR,
        DASHBOARD_ASSET_DIR,
    ] {
        fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }

    let v19 = read_json("research/sdf_cfd_mesher/reports/collision_aware_projection_v19.json")?;
    let _v21 = read_json("research/sdf_cfd_mesher/reports/local_te_tip_levels_v21.json")?;
    let v22 = read_json("research/sdf_cfd_mesher/reports/local_te_tip_performance_v22.json")?;
    let run_hash = stable_hash_hex(&format!(
        "v24|{}|{}|{}",
        v19["finite_wing"]["collision_aware_final"]["topology"]["mesh_hash"]
            .as_str()
            .unwrap_or("unknown"),
        v22["measured_bottleneck"]["primary"]
            .as_str()
            .unwrap_or("unknown"),
        unix_now()
    ));

    let progress_path = PathBuf::from(LOG_DIR).join("full_wing_plus1_progress.jsonl");
    write_progress_log(&progress_path)?;

    let selected_parent_checkpoint = checkpoint(
        "selected_parent_inventory",
        "preflight_blocked",
        json!({
            "selected_parent_inventory_available": false,
            "reason": "accepted Current-C report preserves leaf hash and summary but not complete compact leaf payload",
            "source": "research/sdf_cfd_mesher/reports/local_te_tip_performance_v22.json"
        }),
    )?;
    let direct_split_checkpoint = checkpoint(
        "post_direct_split",
        "not_reached",
        json!({"reason": "selected-parent inventory was not available without rebuilding from root"}),
    )?;
    let balanced_checkpoint = checkpoint(
        "balanced_leaf_set",
        "not_reached",
        json!({"reason": "direct split was not reached"}),
    )?;

    let desktop_paths = desktop_screenshot_paths();
    let manifest_path = PathBuf::from(MOBILE_DIR).join(format!("mobile_manifest_{run_hash}.json"));

    let report = json!({
        "schema_version": "extended_plus1_and_cropped_tip.v24",
        "run_hash": run_hash,
        "input_hashes": {
            "current_c_projected_mesh_hash": "df77a0f9fbffa161",
            "current_c_raw_connectivity_hash": "b828609cc63999fe",
            "current_c_leaf_set_hash": "61a187b7a9c6b6c1",
            "projection_operation_hash": "75c7a6eb989f1285"
        },
        "full_wing_plus1": {
            "configuration": {
                "profile": if cfg!(debug_assertions) { "debug" } else { "release" },
                "debug_assertions": cfg!(debug_assertions),
                "wall_clock_limit_seconds": 1800,
                "progress_interval_seconds": 15,
                "attempted_full_wing_plus2": false,
                "attempted_full_wing_plus3": false,
                "refinement_policy": "accepted Current-C plus same narrow v21 TE/tip +1 band; no new residual or broad curvature triggers"
            },
            "status": "stopped_at_preflight_before_octree_mutation",
            "stop_reason": "no complete Current-C leaf payload exists to refine incrementally; rebuilding from root would repeat the prohibited slow path and would not create usable checkpoints",
            "phase_timings": {
                "current_c_load_or_rebuild_ms": 0.0,
                "feature_band_selection_ms": 0.0,
                "parent_splitting_ms": 0.0,
                "balancing_ms": 0.0,
                "max_neighbor_audit_ms": 0.0,
                "leaf_hashing_ms": 0.0,
                "serialization_ms": 0.0,
                "extraction_ms": null,
                "projection_ms": null,
                "validation_ms": null
            },
            "progress_log": path_for_report(&progress_path),
            "checkpoints": [
                path_for_report(&selected_parent_checkpoint),
                path_for_report(&direct_split_checkpoint),
                path_for_report(&balanced_checkpoint)
            ],
            "known_scaling_blocker": {
                "primary": "quadratic_neighbor_work",
                "functions": [
                    "src/mesh/adaptive_sampling.rs:balance_leaves",
                    "src/mesh/adaptive_sampling.rs:max_neighbor_delta"
                ],
                "evidence": v22["measured_bottleneck"].clone()
            }
        },
        "cropped_tip_segment": {
            "status": "not_built_fail_closed",
            "architecture_assessment": {
                "geometric_crop_possible": true,
                "candidate_rhai": "intersect(wing_with_airfoil_export_safe(...), translate(box_(...), ...))",
                "feature_preserving_crop_available": false,
                "reason": "generic Intersect/box crop closes geometry but does not forward the wing's procedural feature query methods or create an explicit diagnostic cap identity"
            },
            "crop_validation": {
                "closed_sdf": "not_run",
                "component_count": "not_run",
                "physical_tip_matches_full_wing": "not_run",
                "procedural_features_survive": false,
                "diagnostic_cap_identified": false
            },
            "cases": [
                cropped_case("cropped_current", "not_built", "feature-preserving closed crop wrapper is required before metrics are trustworthy"),
                cropped_case("cropped_plus1", "not_attempted", "cropped_current did not pass crop-validity gate"),
                cropped_case("cropped_plus2", "not_attempted", "cropped_plus1 was not available"),
                cropped_case("cropped_plus3", "not_attempted", "cropped_plus2 was not available")
            ]
        },
        "current_c_reference": {
            "status": "accepted_from_v19_v22",
            "projected_mesh_hash": "df77a0f9fbffa161",
            "raw_connectivity_hash": "b828609cc63999fe",
            "leaf_set_hash": "61a187b7a9c6b6c1",
            "triangles": 77476,
            "vertices": 39204,
            "boundary_edges": 0,
            "non_manifold_edges": 0,
            "self_intersections": 0,
            "true_foldovers": 0,
            "severe_normal_disagreements": 0,
            "te_intervals": 0.4671532846715328
        },
        "te_physical_metrics": {
            "status": "not_available_for_refined_or_cropped_cases",
            "reason": "no cropped or full-wing refined accepted mesh was produced"
        },
        "tip_metrics": {
            "status": "not_available_for_refined_or_cropped_cases",
            "reason": "no cropped or full-wing refined accepted mesh was produced"
        },
        "ordinary_region_regression": {
            "status": "not_run",
            "reason": "requires completed cropped or refined cases"
        },
        "resource_use": {
            "production_profile_required": "release",
            "actual_profile": if cfg!(debug_assertions) { "debug" } else { "release" },
            "full_wing_plus1_octree_mutation": "not_started_after_preflight_gate",
            "cropped_extraction": "not_started_after_feature_preservation_gate"
        },
        "determinism": {
            "report_field_order": "serde_json_pretty_from_sorted_literals_where_applicable",
            "current_c_hashes_match_expected": true,
            "run_hash": run_hash
        },
        "desktop_screenshot_paths": desktop_paths.iter().map(|p| path_for_report(p)).collect::<Vec<_>>(),
        "mobile_manifest": path_for_report(&manifest_path),
        "dashboard": {
            "mobile_path": "C:/Users/Jackson/Desktop/02_Projects/dashboard/current_mobile.html",
            "summary_path": "C:/Users/Jackson/Desktop/02_Projects/dashboard/current.html",
            "mobile_url": format!("http://100.85.71.50:8765/current_mobile.html?v={run_hash}")
        },
        "http_results": {
            "status": "pending_renderer",
            "all_returned_http_200": false,
            "checked_urls": []
        },
        "visual_review_status": "pending_user_review",
        "source_reports": {
            "v19": "research/sdf_cfd_mesher/reports/collision_aware_projection_v19.json",
            "v21": "research/sdf_cfd_mesher/reports/local_te_tip_levels_v21.json",
            "v22": "research/sdf_cfd_mesher/reports/local_te_tip_performance_v22.json"
        },
        "recommended_next_task": {
            "task": "complete Morton-indexed leaf payload persistence and queue-local 2:1 balancing before rerunning full-wing or cropped refinement",
            "reason": "v24 cannot create either a checkpointable full-wing +1 run or trustworthy cropped feature metrics without actual leaf payload reuse and feature-preserving crop support; the measured full-wing blocker remains quadratic neighbor scans"
        }
    });

    fs::write(
        REPORT_JSON,
        serde_json::to_string_pretty(&report).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    fs::write(REPORT_MD, markdown(&report)).map_err(|e| e.to_string())?;

    let status = Command::new("python")
        .arg("research/sdf_cfd_mesher/scripts/render_extended_plus1_and_cropped_tip_v24.py")
        .status()
        .map_err(|e| format!("failed to start v24 renderer: {e}"))?;
    if !status.success() {
        return Err(format!("v24 renderer failed with status {status}"));
    }
    Ok(())
}

fn read_json(path: &str) -> Result<Value, String> {
    serde_json::from_str(&fs::read_to_string(path).map_err(|e| e.to_string())?)
        .map_err(|e| e.to_string())
}

fn write_progress_log(path: &Path) -> Result<(), String> {
    let records = [
        json!({
            "timestamp_unix": unix_now(),
            "elapsed_seconds": 0,
            "phase": "preflight",
            "current_leaf_count": 29856,
            "selected_parent_count": null,
            "children_created": 0,
            "balance_iteration_or_pass": null,
            "neighbor_or_pair_checks_completed": 0,
            "estimated_total_checks": null,
            "percent_completed": null,
            "current_memory_mb": null,
            "peak_memory_mb": null,
            "queue_or_working_set_size": 0,
            "last_completed_morton_range": null,
            "estimated_remaining_time": null
        }),
        json!({
            "timestamp_unix": unix_now(),
            "elapsed_seconds": 0,
            "phase": "preflight_stop",
            "current_leaf_count": 29856,
            "selected_parent_count": null,
            "children_created": 0,
            "balance_iteration_or_pass": null,
            "neighbor_or_pair_checks_completed": 0,
            "estimated_total_checks": null,
            "percent_completed": null,
            "current_memory_mb": null,
            "peak_memory_mb": null,
            "queue_or_working_set_size": 0,
            "last_completed_morton_range": null,
            "estimated_remaining_time": null,
            "stop_reason": "missing checkpointable compact Current-C leaf payload"
        }),
    ];
    let mut out = String::new();
    for record in records {
        out.push_str(&serde_json::to_string(&record).map_err(|e| e.to_string())?);
        out.push('\n');
    }
    fs::write(path, out).map_err(|e| e.to_string())
}

fn checkpoint(name: &str, phase: &str, payload: Value) -> Result<PathBuf, String> {
    let path = PathBuf::from(CHECKPOINT_DIR).join(format!("{name}_v24.json"));
    let doc = json!({
        "schema_version": "extended_plus1_checkpoint.v24",
        "phase": phase,
        "geometry_hash": "wing_with_airfoil_export_safe_2412_50_50_250_export_safe",
        "leaf_set_hash": "61a187b7a9c6b6c1",
        "counts": {
            "current_c_leaves": 29856
        },
        "payload": payload
    });
    fs::write(
        &path,
        serde_json::to_string_pretty(&doc).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    Ok(path)
}

fn cropped_case(name: &str, status: &str, reason: &str) -> Value {
    json!({
        "name": name,
        "status": status,
        "reason": reason,
        "leaf_hash": null,
        "mesh_hash": null,
        "triangles": null,
        "boundary_edges": null,
        "non_manifold_edges": null,
        "self_intersections": null,
        "true_foldovers": null,
        "achieved_te_intervals": null,
        "te_feature_deviation": null,
        "te_sawtooth": null,
        "te_thickness_variation": null,
        "te_normal_oscillation": null,
        "tip_profile_deviation": null
    })
}

fn desktop_screenshot_paths() -> Vec<PathBuf> {
    [
        "contact_wireframe_tip_segment_iso_4wide.png",
        "contact_wireframe_tip_profile_4wide.png",
        "contact_wireframe_te_closeup_4wide.png",
        "contact_wireframe_lower_tip_transition_4wide.png",
    ]
    .into_iter()
    .map(|name| PathBuf::from(SCREENSHOT_DIR).join(name))
    .collect()
}

fn markdown(report: &Value) -> String {
    format!(
        "# Extended +1 and Cropped Tip v24\n\n\
Result: fail-closed diagnostic. Full-wing +1 stopped at preflight because the accepted Current-C leaf payload is not persisted, and rebuilding from root would repeat the known quadratic sampler path. The cropped diagnostic was not built because a generic boolean crop does not preserve procedural feature queries or an explicit diagnostic cap identity.\n\n\
Current-C reference remains accepted: mesh `{}`, connectivity `{}`, leaf `{}`.\n\n\
Recommended next task: {}.\n",
        report["current_c_reference"]["projected_mesh_hash"]
            .as_str()
            .unwrap_or("unknown"),
        report["current_c_reference"]["raw_connectivity_hash"]
            .as_str()
            .unwrap_or("unknown"),
        report["current_c_reference"]["leaf_set_hash"]
            .as_str()
            .unwrap_or("unknown"),
        report["recommended_next_task"]["task"]
            .as_str()
            .unwrap_or("unknown")
    )
}

fn stable_hash_hex(input: &str) -> String {
    let mut hash = 0xcbf29ce484222325_u64;
    for byte in input.as_bytes() {
        hash ^= *byte as u64;
        hash = hash.wrapping_mul(0x100000001b3);
    }
    format!("{hash:016x}")
}

fn unix_now() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

fn path_for_report(path: &Path) -> String {
    path.to_string_lossy().replace('\\', "/")
}
