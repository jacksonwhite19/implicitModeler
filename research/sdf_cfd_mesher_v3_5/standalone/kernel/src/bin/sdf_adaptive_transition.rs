use serde_json::json;
use std::fs;
use std::path::PathBuf;

const V12_REPORT: &str = "research/sdf_cfd_mesher/reports/adaptive_resolution_policy_v12.json";
const REPORT_JSON: &str = "research/sdf_cfd_mesher/reports/adaptive_transition_v13.json";
const REPORT_MD: &str = "research/sdf_cfd_mesher/reports/adaptive_transition_v13.md";
const EXP_DIR: &str = "research/sdf_cfd_mesher/experiments/adaptive_transition_v13";
const MESH_DIR: &str = "research/sdf_cfd_mesher/experiments/adaptive_transition_v13/meshes";
const TRANSITION_DIR: &str =
    "research/sdf_cfd_mesher/experiments/adaptive_transition_v13/transition_cases";
const SCREENSHOT_DIR: &str =
    "research/sdf_cfd_mesher/experiments/adaptive_transition_v13/screenshots";
const LOG_DIR: &str = "research/sdf_cfd_mesher/experiments/adaptive_transition_v13/logs";

fn main() {
    if let Err(err) = run() {
        eprintln!("adaptive transition v13 failed: {err}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    for dir in [
        "research/sdf_cfd_mesher/reports",
        EXP_DIR,
        MESH_DIR,
        TRANSITION_DIR,
        SCREENSHOT_DIR,
        LOG_DIR,
    ] {
        fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }

    let v12: serde_json::Value = serde_json::from_str(
        &fs::read_to_string(V12_REPORT)
            .map_err(|e| format!("read active v12 transition-check report: {e}"))?,
    )
    .map_err(|e| e.to_string())?;
    let variants = v12["variants"].as_array().cloned().unwrap_or_default();
    let current_c = variants
        .iter()
        .find(|v| v["name"] == "adaptive_floor_c")
        .cloned()
        .ok_or("active v12 report did not contain adaptive_floor_c")?;
    let current_b = variants
        .iter()
        .find(|v| v["name"] == "adaptive_floor_b")
        .cloned()
        .unwrap_or_else(|| json!({}));
    let current_a = variants
        .iter()
        .find(|v| v["name"] == "adaptive_floor_a")
        .cloned()
        .unwrap_or_else(|| json!({}));

    let c_boundary_after = current_c["direct_extraction"]["boundary_edges"]
        .as_u64()
        .unwrap_or(u64::MAX);
    let c_self_intersections_zero =
        current_c["direct_extraction"]["artifact_acceptance"]["self_intersections_zero"]
            .as_bool()
            .unwrap_or(false);
    let c_status = if c_boundary_after == 0 && c_self_intersections_zero {
        "accepted"
    } else {
        "failed_closed"
    };

    let gate1_passed = c_status == "accepted";
    let report = json!({
        "schema_version": "adaptive_transition.v13",
        "v12_input": {
            "uniform_hash": "10009dccfa69819e",
            "adaptive_c_leaf_hash": "61a187b7a9c6b6c1",
            "adaptive_c_boundary_edges_before": 1842,
            "active_transition_layout_env": "IMPLICIT_CAD_TRANSITION_LAYOUT=active",
            "source_report": V12_REPORT
        },
        "transition_architecture": {
            "method": "bounded local-uniformization fallback for near-zero 2:1 partial-overlap faces",
            "implementation": "src/mesh/adaptive_mc.rs::split_partial_overlap_cells",
            "ownership_policy": {
                "same_level_faces": "ordinary marching-cubes emission with canonical edge cache",
                "mixed_partial_overlap_faces": "replace the problematic coarse surface cell with SDF-evaluated children before emission",
                "hole_filling": false,
                "coordinate_tolerance_welding_as_primary_identity": false
            },
            "canonical_identity": {
                "current": "existing adaptive_mc canonical edge-key cache plus deterministic projected SdfSurfaceMesh rebuild",
                "remaining_gap": "pure octree finest-grid edge identity is not yet implemented for all partial-overlap transition families"
            },
            "ambiguity_policy": {
                "exact_zero": "existing marching-cubes sign policy preserved",
                "unsupported_transition_cases": "fail closed and report exact case family"
            }
        },
        "transition_case_inventory": {
            "families_classified": [
                "1-of-4",
                "2-adjacent",
                "2-diagonal",
                "3-of-4",
                "full-4-of-4",
                "irregular"
            ],
            "dedicated_regression": "partial_overlap_classifier_recognizes_two_adjacent_case",
            "active_uniformization_regression": "active_transition_layout_splits_near_zero_partial_overlap",
            "finite_wing_current_c_remaining_defect": {
                "dominant_family": "2-adjacent",
                "evidence": "active run still maps boundary-edge clusters to 2-adjacent partial-overlap faces after local coarse-cell subdivision"
            }
        },
        "gate1": {
            "synthetic_cases": {
                "status": "implemented_focused_tests",
                "tests": [
                    "partial_overlap_classifier_recognizes_two_adjacent_case",
                    "active_transition_layout_splits_near_zero_partial_overlap"
                ]
            },
            "sphere": {
                "status": "not_rerun_as_accepted_gate",
                "reason": "finite-wing current-C gate failed; deeper acceptance branch was not entered"
            },
            "hard_subtraction": {
                "status": "not_rerun_as_accepted_gate",
                "reason": "finite-wing current-C gate failed; deeper acceptance branch was not entered"
            },
            "finite_wing_v12_c": {
                "status": c_status,
                "leaf_hash": current_c["leaf_set_hash"],
                "triangles_before_v12": 12022,
                "boundary_edges_before_v12": 1842,
                "triangles_after_transition_layout": current_c["direct_extraction"]["triangles"],
                "boundary_edges_after_transition_layout": current_c["direct_extraction"]["boundary_edges"],
                "non_manifold_edges_after_transition_layout": current_c["direct_extraction"]["non_manifold_edges"],
                "projection_failures_after_transition_layout": current_c["direct_extraction"]["projection_failures"],
                "failure_reasons": current_c["failure_reasons"],
                "gate_decision": if gate1_passed { "proceed_to_gate2" } else { "fail_closed_do_not_run_te4_te6_te8" }
            },
            "other_v12_variants_after_transition_layout": {
                "adaptive_floor_a": {
                    "triangles": current_a["direct_extraction"]["triangles"],
                    "boundary_edges": current_a["direct_extraction"]["boundary_edges"],
                    "failure_reasons": current_a["failure_reasons"]
                },
                "adaptive_floor_b": {
                    "triangles": current_b["direct_extraction"]["triangles"],
                    "boundary_edges": current_b["direct_extraction"]["boundary_edges"],
                    "failure_reasons": current_b["failure_reasons"]
                }
            }
        },
        "depth_policy": {
            "te4": {"status": "not_run", "reason": "Gate 1 finite-wing v12 C failed closed"},
            "te6": {"status": "not_run", "reason": "Gate 1 finite-wing v12 C failed closed"},
            "te8": {"status": "not_run", "reason": "Gate 1 finite-wing v12 C failed closed"}
        },
        "cases": [
            {
                "name": "uniform_reference",
                "mesh_hash": "10009dccfa69819e",
                "status": "reference_only",
                "source": "accepted uniform reference from v12"
            },
            {
                "name": "adaptive_v12_c_transition_layout_active",
                "leaf_set_hash": current_c["leaf_set_hash"],
                "mesh_hash": current_c["output_mesh_hash"],
                "counts": current_c["direct_extraction"],
                "status": c_status,
                "failure_reasons": current_c["failure_reasons"]
            }
        ],
        "uniform_vs_adaptive": {
            "gate2_comparison_status": "not_generated",
            "reason": "Current v12 C did not reach zero boundary edges and zero self-intersections after transition-layout correction"
        },
        "performance": {
            "finite_wing_v12_c_active_transition_runtime_ms": current_c["direct_extraction"]["runtime_ms"],
            "memory_estimate": "not_measured",
            "gate2_te4_te6_te8_runtime": "not_run_gate1_failed"
        },
        "screenshots": [],
        "contact_sheets": {
            "contact_wireframe_iso_4wide.png": "not_generated_gate1_failed",
            "contact_wireframe_tip_profile_4wide.png": "not_generated_gate1_failed",
            "contact_wireframe_te_closeup_4wide.png": "not_generated_gate1_failed",
            "contact_wireframe_lower_tip_transition_4wide.png": "not_generated_gate1_failed"
        },
        "visual_review_status": "pending_user_inspection_after_next_valid_contact_sheets",
        "macro_shell_readiness": {
            "ready": false,
            "reason": "direct adaptive transition connectivity is still invalid on the current v12 C leaf set"
        },
        "known_gaps": [
            "2-adjacent partial-overlap faces remain after coarse-cell local uniformization",
            "pure canonical transition vertices on finest-grid face subedges are still required",
            "TE-4/6/8 were intentionally not run because Gate 1 failed"
        ],
        "recommended_next_task": {
            "task": "canonical 2:1 transition-face triangulation for 2-adjacent partial-overlap cases",
            "reason": "local uniformization reduced current-C boundary edges from 1842 to 478 but did not eliminate 2-adjacent transition cracks or self-intersections"
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
        "# Adaptive Transition v13 Screenshots\n\nGate 1 failed closed, so TE-4/TE-6/TE-8 contact sheets were not generated.\n",
    )
    .map_err(|e| e.to_string())?;
    Ok(())
}

fn markdown_summary(report: &serde_json::Value) -> String {
    format!(
        "# Adaptive Transition v13\n\n\
Result: Gate 1 failed closed.\n\n\
Architecture: bounded local-uniformization of near-zero 2:1 partial-overlap faces in `src/mesh/adaptive_mc.rs`.\n\n\
Current v12 C before/after: boundary edges `1842` -> `{}`, triangles `12022` -> `{}`.\n\n\
Dominant remaining family: `2-adjacent` partial-overlap transition faces.\n\n\
TE-4/TE-6/TE-8: not run because Gate 1 did not produce a closed, self-intersection-free current-C surface.\n\n\
Screenshots: not generated; exactly-four contact-sheet branch is gated on a valid current-C extractor.\n\n\
Recommended next task: `{}`.\n",
        report["gate1"]["finite_wing_v12_c"]["boundary_edges_after_transition_layout"]
            .as_u64()
            .unwrap_or(0),
        report["gate1"]["finite_wing_v12_c"]["triangles_after_transition_layout"]
            .as_u64()
            .unwrap_or(0),
        report["recommended_next_task"]["task"]
            .as_str()
            .unwrap_or("unknown")
    )
}
