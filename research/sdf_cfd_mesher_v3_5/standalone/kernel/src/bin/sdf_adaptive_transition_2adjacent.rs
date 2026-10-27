use serde_json::json;
use std::fs;
use std::path::PathBuf;

const V13_REPORT: &str = "research/sdf_cfd_mesher/reports/adaptive_transition_v13.json";
const REPORT_JSON: &str = "research/sdf_cfd_mesher/reports/adaptive_transition_2adjacent_v14.json";
const REPORT_MD: &str = "research/sdf_cfd_mesher/reports/adaptive_transition_2adjacent_v14.md";
const EXP_DIR: &str = "research/sdf_cfd_mesher/experiments/adaptive_transition_2adjacent_v14";
const DEBUG_DIR: &str =
    "research/sdf_cfd_mesher/experiments/adaptive_transition_2adjacent_v14/debug";
const SCREENSHOT_DIR: &str =
    "research/sdf_cfd_mesher/experiments/adaptive_transition_2adjacent_v14/screenshots";

fn main() {
    if let Err(err) = run() {
        eprintln!("adaptive transition 2-adjacent v14 failed: {err}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    for dir in [
        "research/sdf_cfd_mesher/reports",
        EXP_DIR,
        DEBUG_DIR,
        SCREENSHOT_DIR,
    ] {
        fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }

    let v13: serde_json::Value = serde_json::from_str(
        &fs::read_to_string(V13_REPORT).map_err(|e| format!("read v13 report: {e}"))?,
    )
    .map_err(|e| e.to_string())?;
    let finite_v13 = &v13["gate1"]["finite_wing_v12_c"];
    let boundary_after_v13 = finite_v13["boundary_edges_after_transition_layout"]
        .as_u64()
        .unwrap_or(478);
    let non_manifold_after_v13 = finite_v13["non_manifold_edges_after_transition_layout"]
        .as_u64()
        .unwrap_or(0);
    let projection_failures_after_v13 = finite_v13["projection_failures_after_transition_layout"]
        .as_u64()
        .unwrap_or(0);
    let v13_failures = finite_v13["failure_reasons"].clone();

    let current_c_accepted = boundary_after_v13 == 0
        && non_manifold_after_v13 == 0
        && projection_failures_after_v13 == 0
        && !v13_failures
            .as_array()
            .map(|items| {
                items.iter().any(|item| {
                    matches!(
                        item.as_str(),
                        Some("self_intersections_detected")
                            | Some("severe_triangle_sdf_normal_disagreement")
                            | Some("topology_or_degenerate_triangle_failure")
                    )
                })
            })
            .unwrap_or(true);
    let debug_log = "research/sdf_cfd_mesher/experiments/adaptive_transition_2adjacent_v14/debug/current_c_active_rerun.log";
    let log_text = read_text_lossy(debug_log);
    let canonical_inventory_line = canonical_inventory_summary(&log_text);

    let report = json!({
        "schema_version": "adaptive_transition_2adjacent.v14",
        "input": {
            "v12_c_leaf_hash": "61a187b7a9c6b6c1",
            "v12_boundary_edges": 1842,
            "v13_boundary_edges": 478,
            "source_reports": [
                "research/sdf_cfd_mesher/reports/adaptive_extraction_v10.json",
                "research/sdf_cfd_mesher/reports/adaptive_resolution_policy_v12.json",
                V13_REPORT
            ]
        },
        "architecture": {
            "transition_face_key": {
                "implementation": "src/mesh/adaptive_mc.rs::TransitionFaceKey",
                "identity": "axis + finest-grid plane coordinate + finest-grid coarse face origin + coarse level + orientation",
                "floating_point_coordinate_identity": false
            },
            "canonical_coordinates": {
                "implementation": "src/mesh/adaptive_mc.rs::transition_face_key",
                "lattice": "common finest-cell-size integer lattice from FaceAdjacency::finest_cell_size"
            },
            "case_normalization": {
                "implementation": "src/mesh/adaptive_mc.rs::canonical_two_adjacent_layout",
                "supported_masks": {
                    "0011": "TwoAdjacentV0",
                    "1100": "TwoAdjacentV1",
                    "0101": "TwoAdjacentU0",
                    "1010": "TwoAdjacentU1"
                }
            },
            "ownership_policy": {
                "implemented": "canonical inventory creates one deterministic key per supported 2-adjacent face",
                "remaining_gap": "ordinary coarse/fine MC patch suppression and replacement transition polygon emission are not yet integrated",
                "hole_filling": false,
                "post_extraction_feature_splitting": false
            },
            "ambiguity_policy": {
                "implemented": "actual 3x3 face SDF samples are recorded and exact-zero samples count deterministically as crossings",
                "remaining_gap": "asymptotic/face-center decider is not yet wired into emitted transition triangulation"
            },
            "triangulation_policy": {
                "status": "not_integrated",
                "reason": "canonical ownership/data model is present, but transition polygon emission and ordinary patch suppression are still the blocker"
            }
        },
        "case_inventory": {
            "families_targeted": ["2-adjacent"],
            "families_not_implemented": ["2-diagonal", "3-of-4", "edge", "corner", "irregular"],
            "dominant_current_c_family": "2-adjacent",
            "current_c_canonical_inventory_line": canonical_inventory_line,
            "current_c_debug_log": debug_log
        },
        "synthetic_tests": [
            {"name": "canonical_two_adjacent_layouts_cover_rotations_and_reflections", "status": "passed"},
            {"name": "canonical_transition_face_key_is_stable_on_finest_grid", "status": "passed"},
            {"name": "canonical_transition_inventory_owns_supported_two_adjacent_once", "status": "passed"},
            {"name": "exact_zero_face_samples_count_as_deterministic_crossings", "status": "passed"},
            {"name": "partial_overlap_classifier_recognizes_two_adjacent_case", "status": "passed"},
            {"name": "active_transition_layout_splits_near_zero_partial_overlap", "status": "passed"}
        ],
        "real_regression_fixtures": [
            {
                "name": "finite_wing_current_c_v13_dominant_2_adjacent",
                "status": "diagnosed_not_closed",
                "source": "research/sdf_cfd_mesher/reports/adaptive_transition_v13.json",
                "boundary_edges": boundary_after_v13,
                "failure_reasons": v13_failures
            }
        ],
        "finite_wing": {
            "before": {
                "v12_raw_boundary_edges": 1842,
                "v13_fallback_boundary_edges": boundary_after_v13,
                "v13_triangles": finite_v13["triangles_after_transition_layout"]
            },
            "after": {
                "status": if current_c_accepted { "accepted" } else { "failed_closed" },
                "boundary_edges": boundary_after_v13,
                "non_manifold_edges": non_manifold_after_v13,
                "projection_failures": projection_failures_after_v13,
                "failure_reasons": v13_failures
            },
            "topology_by_stage": [
                {
                    "stage": "after_v13_local_uniformization",
                    "boundary_edges": boundary_after_v13,
                    "non_manifold_edges": non_manifold_after_v13,
                    "projection_failures": projection_failures_after_v13,
                    "status": "failed_closed"
                }
            ],
            "remaining_boundary_groups": [
                {
                    "family": "2-adjacent",
                    "status": "remaining",
                    "evidence": "v13 defect map still identifies 2-adjacent as dominant after local uniformization"
                }
            ],
            "remaining_case_families": ["2-adjacent"]
        },
        "sphere": {
            "status": "not_run",
            "reason": "current C did not become fully closed/self-intersection-free"
        },
        "hard_subtraction": {
            "status": "not_run",
            "reason": "current C did not become fully closed/self-intersection-free"
        },
        "projection_stage_findings": {
            "status": "not_isolated_with_new_surface",
            "reason": "canonical 2-adjacent transition polygon emission is not integrated, so no new staged mesh exists beyond v13"
        },
        "performance": {
            "v13_current_c_runtime_ms": v13["performance"]["finite_wing_v12_c_active_transition_runtime_ms"],
            "v14_current_c_rerun_log": debug_log,
            "v14_report_generation": "metadata_plus_active_current_c_rerun_log",
            "transition_face_runtime": "not_measured_until_emission_integrated"
        },
        "screenshots": [],
        "visual_review_status": "pending_user_review_after_valid_current_c_surface",
        "recommended_next_task": {
            "task": "complete canonical 2-adjacent transition polygon emission with ordinary coarse/fine patch suppression",
            "reason": "keys, normalization, ownership inventory, and SDF face sampling are implemented, but current-C cannot improve until the transition entity emits the replacement polygon and suppresses overlapping ordinary MC patches"
        }
    });

    fs::write(
        REPORT_JSON,
        serde_json::to_string_pretty(&report).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    fs::write(REPORT_MD, markdown_summary(&report)).map_err(|e| e.to_string())?;
    fs::write(
        PathBuf::from(SCREENSHOT_DIR).join("index.md"),
        "# Adaptive Transition 2-Adjacent v14 Screenshots\n\nNo final contact sheets were generated because current-C remains failed closed.\n",
    )
    .map_err(|e| e.to_string())?;
    Ok(())
}

fn read_text_lossy(path: &str) -> String {
    match fs::read_to_string(path) {
        Ok(text) => text,
        Err(_) => {
            let Ok(bytes) = fs::read(path) else {
                return String::new();
            };
            if bytes.len() >= 2 && bytes[0] == 0xff && bytes[1] == 0xfe {
                let words: Vec<u16> = bytes[2..]
                    .chunks_exact(2)
                    .map(|chunk| u16::from_le_bytes([chunk[0], chunk[1]]))
                    .collect();
                String::from_utf16_lossy(&words)
            } else {
                String::from_utf8_lossy(&bytes).into_owned()
            }
        }
    }
}

fn canonical_inventory_summary(log_text: &str) -> String {
    let lines: Vec<&str> = log_text.lines().collect();
    let mut best: Option<(u64, String)> = None;

    for (idx, line) in lines.iter().enumerate() {
        if !line.contains("canonical_transition_inventory") {
            continue;
        }

        let mut summary = line.trim().to_string();
        if let Some(next) = lines.get(idx + 1) {
            let trimmed = next.trim();
            if trimmed.contains("duplicate_keys")
                || trimmed.contains("missing_ownership")
                || trimmed.contains("emitted=")
            {
                summary.push(' ');
                summary.push_str(trimmed);
            }
        }

        let discovered = parse_u64_after(&summary, "discovered=").unwrap_or(0);
        let replace = best
            .as_ref()
            .map(|(best_discovered, _)| discovered > *best_discovered)
            .unwrap_or(true);
        if replace {
            best = Some((discovered, summary));
        }
    }

    best.map(|(_, summary)| summary)
        .unwrap_or_else(|| "not_available".to_string())
}

fn parse_u64_after(text: &str, key: &str) -> Option<u64> {
    let start = text.find(key)? + key.len();
    let digits: String = text[start..]
        .chars()
        .take_while(|ch| ch.is_ascii_digit())
        .collect();
    digits.parse().ok()
}

fn markdown_summary(report: &serde_json::Value) -> String {
    format!(
        "# Adaptive Transition 2-Adjacent v14\n\n\
Result: failed closed.\n\n\
Implemented: canonical transition-face key, finest-grid integer identity, deterministic 2-adjacent mask normalization, ownership inventory, and SDF face-sample crossing diagnostics.\n\n\
Current C before/after: v12 `1842` boundary edges; v13 fallback `{}` boundary edges; v14 remains `{}` because replacement transition polygon emission is not integrated.\n\n\
Screenshots: not generated; current-C did not pass topology/artifact gates.\n\n\
Recommended next task: `{}`.\n",
        report["finite_wing"]["before"]["v13_fallback_boundary_edges"]
            .as_u64()
            .unwrap_or(478),
        report["finite_wing"]["after"]["boundary_edges"]
            .as_u64()
            .unwrap_or(478),
        report["recommended_next_task"]["task"]
            .as_str()
            .unwrap_or("unknown")
    )
}
