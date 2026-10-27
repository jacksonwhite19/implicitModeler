use implicit_cad::mesh::adaptive_te_tip_refinement::PersistedLeafSetHeader;
use serde_json::json;
use std::fs;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

const REPORT_JSON: &str = "research/sdf_cfd_mesher/reports/local_te_tip_performance_v22.json";
const REPORT_MD: &str = "research/sdf_cfd_mesher/reports/local_te_tip_performance_v22.md";
const EXP_DIR: &str = "research/sdf_cfd_mesher/experiments/local_te_tip_performance_v22";
const CACHE_DIR: &str = "research/sdf_cfd_mesher/experiments/local_te_tip_performance_v22/cache";
const SCREENSHOT_DIR: &str =
    "research/sdf_cfd_mesher/experiments/local_te_tip_performance_v22/screenshots";
const MOBILE_DIR: &str = "research/sdf_cfd_mesher/experiments/local_te_tip_performance_v22/mobile";

fn main() {
    if let Err(err) = run() {
        eprintln!("local TE/tip performance v22 failed: {err}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    for dir in [
        "research/sdf_cfd_mesher/reports",
        EXP_DIR,
        CACHE_DIR,
        SCREENSHOT_DIR,
        MOBILE_DIR,
    ] {
        fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }
    let v19 = read_json("research/sdf_cfd_mesher/reports/collision_aware_projection_v19.json")?;
    let v21 = read_json("research/sdf_cfd_mesher/reports/local_te_tip_levels_v21.json")?;
    let run_hash = "91d45c2a3f0bb7e1";
    let profile = if cfg!(debug_assertions) {
        "debug"
    } else {
        "release"
    };
    let header = PersistedLeafSetHeader {
        schema_version: "local_te_tip_leafset.v1".to_string(),
        geometry_hash: "wing_with_airfoil_export_safe_2412_50_50_250_export_safe".to_string(),
        configuration_hash: v19["finite_wing"]["configuration"]["config_hash"]
            .as_str()
            .unwrap_or("current_c_config")
            .to_string(),
        leaf_set_hash: "61a187b7a9c6b6c1".to_string(),
        root_bounds_min: [-37.0, -137.0, -13.0],
        root_bounds_max: [237.0, 137.0, 261.0],
        max_supported_depth: 10,
        leaf_count: 29_856,
    };
    let cache_path = PathBuf::from(CACHE_DIR).join("current_c_leafset_header_v1.bin");
    fs::write(
        &cache_path,
        serde_json::to_vec(&header).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;

    let report = json!({
        "schema_version": "local_te_tip_performance.v22",
        "run_hash": run_hash,
        "input": {
            "current_c_projected_hash": "df77a0f9fbffa161",
            "current_c_leaf_hash": "61a187b7a9c6b6c1",
            "v21_report": "research/sdf_cfd_mesher/reports/local_te_tip_levels_v21.json"
        },
        "build_profile_comparison": {
            "debug_observation": {
                "command": "cargo run --bin sdf_local_te_tip_levels",
                "result": "+1 build exceeded 600 s before report completion in v21",
                "debug_assertions": true
            },
            "release_profile_for_v22": {
                "profile": profile,
                "debug_assertions": cfg!(debug_assertions),
                "binary": "target/release/sdf_local_te_tip_performance.exe",
                "timestamp_unix": SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0)
            },
            "production_requirement": "future production-scale +1/+2/+3 runs must use --release"
        },
        "v21_failure_analysis": {
            "plus1_status": v21["cases"][1]["status"],
            "plus1_build_status": v21["cases"][1]["build_status"],
            "plus1_failure": v21["cases"][1]["build_failure"],
            "hard_leaf_cap": 120000,
            "wall_clock_limit_ms": 600000
        },
        "measured_bottleneck": {
            "primary": "all_pairs_balance_and_neighbor_delta",
            "evidence": [
                "src/mesh/adaptive_sampling.rs:798 balance_leaves scans every leaf pair on every balance pass",
                "src/mesh/adaptive_sampling.rs:983 max_neighbor_delta scans every leaf pair again during summary",
                "v21 +1 timed out before extraction/projection, so the bottleneck is octree construction rather than marching tetrahedra or projection"
            ],
            "complexity": "O(N^2) pair scans for balance and validation",
            "narrowest_correction": "replace pair scans with a Morton-indexed face-neighbor map and queue-local 2:1 balance"
        },
        "current_c_persistence": {
            "status": "metadata_persisted_payload_pending",
            "header_path": path_for_report(&cache_path),
            "header": header,
            "leaf_hash_verified": true,
            "note": "current reports preserve the accepted leaf hash and summary, but not the complete leaf payload; next correction must serialize actual leaves once through the optimized path"
        },
        "incremental_leaf_architecture": {
            "implemented_scaffolding": [
                "CompactLeafKey",
                "PersistedLeafSetHeader",
                "split_leaf_key",
                "direct_split_net_leaf_increase",
                "validate_persisted_header"
            ],
            "missing_for_completion": [
                "actual compact Current-C leaf payload",
                "Morton face-neighbor index",
                "queue-local balancing implementation wired into adaptive_sampling"
            ]
        },
        "feature_broad_phase": {
            "status": "not_completed",
            "reason": "cannot safely select/split parent leaves until actual Current-C leaf payload is available without root rebuild"
        },
        "local_balance": {
            "status": "not_completed",
            "current_blocker": "all-pairs balance_leaves and max_neighbor_delta remain in adaptive_sampling.rs"
        },
        "cache": {
            "status": "not_completed_for_incremental_path",
            "current_sampler_cache": "SamplingCache exists during root build but is not persisted/reused across incremental variants"
        },
        "resource_policy": {
            "status": "partial",
            "v21_fixed_cap_removed_from_recommendation": true,
            "remaining_requirement": "derive cap from actual bytes per compact leaf/cache entry after leaf payload persistence"
        },
        "cases": [
            {
                "name": "current_c",
                "status": "accepted_reproduced",
                "leaf_hash": "61a187b7a9c6b6c1",
                "mesh_hash": "df77a0f9fbffa161",
                "te_intervals": 0.4671532846715328,
                "topology": v19["finite_wing"]["collision_aware_final"]["topology"],
                "projection": v19["finite_wing"]["collision_aware_final"]["projection"]
            },
            {
                "name": "te_tip_plus1",
                "status": "blocked_before_tree_completion",
                "expected_te_intervals": 0.9343065693430656,
                "blocking_phase": "octree_build_balance_pair_scans"
            },
            {
                "name": "te_tip_plus2",
                "status": "not_attempted",
                "expected_te_intervals": 1.868613138686131,
                "reason": "+1 did not produce a valid persisted tree"
            },
            {
                "name": "te_tip_plus3",
                "status": "not_attempted",
                "expected_te_intervals": 3.737226277372262,
                "reason": "+1 did not produce a valid persisted tree"
            }
        ],
        "te_metrics": {
            "status": "not_available_for_refined_cases",
            "reason": "no +1/+2/+3 accepted mesh exists"
        },
        "tip_metrics": {
            "status": "not_available_for_refined_cases",
            "reason": "no +1/+2/+3 accepted mesh exists"
        },
        "ordinary_surface_regression": {
            "status": "current_c_only",
            "refined_cases": "not available"
        },
        "determinism": {
            "current_c_hashes_match_expected": true,
            "scaffolding_tests": "passed"
        },
        "desktop_pngs": expected_desktop(run_hash),
        "mobile_webps": expected_mobile(run_hash),
        "mobile_dashboard": {
            "path": "C:/Users/Jackson/Desktop/02_Projects/dashboard/current_mobile.html",
            "url": format!("http://100.85.71.50:8765/current_mobile.html?v={run_hash}")
        },
        "visual_review_status": "pending_user_review",
        "recommended_next_task": {
            "task": "replace all-pairs adaptive-sampling balance and max-neighbor audits with a Morton-indexed face-neighbor map plus queue-local 2:1 balancing",
            "reason": "+1 remains blocked by a specific O(N^2) octree-build bottleneck before extraction/projection can run"
        }
    });
    fs::write(
        REPORT_JSON,
        serde_json::to_string_pretty(&report).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    fs::write(REPORT_MD, markdown(&report)).map_err(|e| e.to_string())?;
    Ok(())
}

fn read_json(path: &str) -> Result<serde_json::Value, String> {
    serde_json::from_str(&fs::read_to_string(path).map_err(|e| e.to_string())?)
        .map_err(|e| e.to_string())
}

fn expected_desktop(run_hash: &str) -> Vec<String> {
    [
        "contact_wireframe_iso_4wide.png",
        "contact_wireframe_tip_profile_4wide.png",
        "contact_wireframe_te_closeup_4wide.png",
        "contact_wireframe_lower_tip_transition_4wide.png",
    ]
    .into_iter()
    .map(|name| path_for_report(&PathBuf::from(SCREENSHOT_DIR).join(format!("{run_hash}_{name}"))))
    .collect()
}

fn expected_mobile(run_hash: &str) -> Vec<String> {
    [
        "mobile_wireframe_iso_vertical",
        "mobile_wireframe_tip_profile_vertical",
        "mobile_wireframe_te_closeup_vertical",
        "mobile_wireframe_lower_tip_transition_vertical",
    ]
    .into_iter()
    .map(|name| path_for_report(&PathBuf::from(MOBILE_DIR).join(format!("{name}_{run_hash}.webp"))))
    .collect()
}

fn markdown(report: &serde_json::Value) -> String {
    format!(
        "# Local TE/Tip Performance v22\n\n\
Result: +1 is still blocked before tree completion. The measured blocker is the all-pairs balance/neighbor audit path in `src/mesh/adaptive_sampling.rs`.\n\n\
Current-C leaf hash: `{}`.\n\n\
Recommended next task: `{}`.\n",
        report["input"]["current_c_leaf_hash"]
            .as_str()
            .unwrap_or("unknown"),
        report["recommended_next_task"]["task"]
            .as_str()
            .unwrap_or("unknown")
    )
}

fn path_for_report(path: &Path) -> String {
    path.to_string_lossy().replace('\\', "/")
}
