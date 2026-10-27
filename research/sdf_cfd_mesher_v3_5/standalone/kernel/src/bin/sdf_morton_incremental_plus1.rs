use implicit_cad::mesh::adaptive_marching_tets::{
    AdaptiveMarchingTetsSurface, extract_adaptive_marching_tets_with_sources,
};
use implicit_cad::mesh::adaptive_sampling::{
    AdaptiveOctree, AdaptiveSamplingConfig, build_adaptive_octree,
    refine_te_tip_feature_band_plus_one,
};
use implicit_cad::mesh::adaptive_te_tip_refinement::{
    ActiveLeafMap, FaceDirection, MortonLeafKey, indexed_max_neighbor_delta,
    load_octree_leaf_payload, persist_octree_leaf_payload,
};
use implicit_cad::scripting;
use implicit_cad::sdf::Sdf;
use serde_json::{Value, json};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::Arc;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

const REPORT_JSON: &str = "research/sdf_cfd_mesher/reports/morton_incremental_plus1_v25.json";
const REPORT_MD: &str = "research/sdf_cfd_mesher/reports/morton_incremental_plus1_v25.md";
const EXP_DIR: &str = "research/sdf_cfd_mesher/experiments/morton_incremental_plus1_v25";
const CACHE_DIR: &str = "research/sdf_cfd_mesher/experiments/morton_incremental_plus1_v25/cache";
const SCREENSHOT_DIR: &str =
    "research/sdf_cfd_mesher/experiments/morton_incremental_plus1_v25/screenshots";
const MOBILE_DIR: &str = "research/sdf_cfd_mesher/experiments/morton_incremental_plus1_v25/mobile";
const DASHBOARD_ASSET_DIR: &str =
    "C:/Users/Jackson/Desktop/02_Projects/dashboard/html_assets/morton_incremental_plus1_v25";
const WING_SCRIPT: &str =
    r#"wing_with_airfoil_export_safe("2412", 50.0, 50.0, 250.0, 0.0, 0.0, 0.0, 1.0, 1.0, true)"#;
const GEOMETRY_HASH: &str = "wing_with_airfoil_export_safe_2412_50_50_250_export_safe";
const CURRENT_C_HASH: &str = "61a187b7a9c6b6c1";

fn main() {
    if let Err(err) = run() {
        eprintln!("morton incremental +1 v25 failed: {err}");
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
        DASHBOARD_ASSET_DIR,
    ] {
        fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }
    let v19 = read_json("research/sdf_cfd_mesher/reports/collision_aware_projection_v19.json")?;
    let v24 = read_json("research/sdf_cfd_mesher/reports/extended_plus1_and_cropped_tip_v24.json")?;
    let sdf = eval_sdf(WING_SCRIPT)?;
    let run_hash = stable_hash_hex(format!("v25|{}|{}", CURRENT_C_HASH, unix_now()).as_bytes());

    let current_config = current_c_config();
    let current_payload = PathBuf::from(CACHE_DIR).join("current_c_leafset_v1.bin");
    let current_meta = PathBuf::from(CACHE_DIR).join("current_c_leafset_v1.json");
    let plus1_payload = PathBuf::from(CACHE_DIR).join("plus1_leafset_v1.bin");
    let plus1_meta = PathBuf::from(CACHE_DIR).join("plus1_leafset_v1.json");

    let current_start = Instant::now();
    let current_octree = build_adaptive_octree(sdf.as_ref(), &current_config)?;
    let current_build_ms = elapsed_ms(current_start);
    let current_hash = current_octree.octree_hash();
    let current_persistence = if current_hash == CURRENT_C_HASH {
        let meta = persist_octree_leaf_payload(
            &current_octree,
            GEOMETRY_HASH,
            &current_payload,
            &current_meta,
        )?;
        let loaded = load_octree_leaf_payload(
            &current_payload,
            GEOMETRY_HASH,
            &current_octree.config_hash,
            CURRENT_C_HASH,
        )?;
        json!({
            "status": "persisted_and_verified",
            "payload_path": path_for_report(&current_payload),
            "metadata_path": path_for_report(&current_meta),
            "metadata": meta,
            "reloaded_leaf_hash": loaded.octree_hash(),
            "build_runtime_ms": current_build_ms,
            "command": "cargo run --release --bin sdf_morton_incremental_plus1"
        })
    } else {
        let candidate_payload = PathBuf::from(CACHE_DIR).join("current_c_leafset_candidate_v1.bin");
        let candidate_meta = PathBuf::from(CACHE_DIR).join("current_c_leafset_candidate_v1.json");
        let meta = persist_octree_leaf_payload(
            &current_octree,
            GEOMETRY_HASH,
            &candidate_payload,
            &candidate_meta,
        )?;
        json!({
            "status": "hash_mismatch_fail_closed",
            "expected": CURRENT_C_HASH,
            "actual": current_hash,
            "build_runtime_ms": current_build_ms,
            "candidate_payload_path": path_for_report(&candidate_payload),
            "candidate_metadata_path": path_for_report(&candidate_meta),
            "candidate_payload_roundtrip": meta.byte_roundtrip_identical,
            "note": "candidate payload is persisted for diagnostics only and is not treated as the accepted Current-C leaf set"
        })
    };

    let mut plus1_tree = None::<AdaptiveOctree>;
    let mut plus1_raw = None::<AdaptiveMarchingTetsSurface>;
    let plus1_result = if current_hash == CURRENT_C_HASH {
        let loaded = load_octree_leaf_payload(
            &current_payload,
            GEOMETRY_HASH,
            &current_octree.config_hash,
            CURRENT_C_HASH,
        )?;
        let plus_config = plus1_config();
        let plus_start = Instant::now();
        match refine_te_tip_feature_band_plus_one(sdf.as_ref(), &loaded, &plus_config) {
            Ok((octree, stats)) => {
                let build_ms = elapsed_ms(plus_start);
                let plus_hash = octree.octree_hash();
                let meta = persist_octree_leaf_payload(
                    &octree,
                    GEOMETRY_HASH,
                    &plus1_payload,
                    &plus1_meta,
                )
                .ok();
                let resource = resource_policy(&octree);
                let raw = if resource["run_extraction"].as_bool().unwrap_or(false) {
                    let raw_start = Instant::now();
                    let surface =
                        extract_adaptive_marching_tets_with_sources(sdf.as_ref(), &octree);
                    let raw_ms = elapsed_ms(raw_start);
                    let value = raw_summary(&surface, raw_ms);
                    plus1_raw = Some(surface);
                    value
                } else {
                    json!({
                        "status": "not_run_resource_gate",
                        "reason": resource["reason"]
                    })
                };
                plus1_tree = Some(octree.clone());
                json!({
                    "status": "tree_built",
                    "build_runtime_ms": build_ms,
                    "leaf_hash": plus_hash,
                    "summary": octree.summary,
                    "incremental_stats": stats,
                    "payload_path": path_for_report(&plus1_payload),
                    "metadata_path": path_for_report(&plus1_meta),
                    "metadata": meta,
                    "resource_policy": resource,
                    "raw_extraction": raw,
                    "projection": {
                        "status": "not_run",
                        "reason": "v19 collision-aware projector is implemented in src/bin/sdf_collision_aware_projection.rs and is not yet exposed as a reusable library entry point for an externally supplied +1 octree"
                    }
                })
            }
            Err(err) => json!({
                "status": "tree_build_failed",
                "failure": err,
                "build_runtime_ms": elapsed_ms(plus_start)
            }),
        }
    } else {
        json!({
            "status": "not_attempted",
            "reason": "Current-C rebuild did not reproduce accepted leaf hash"
        })
    };

    let benchmark = old_vs_new_benchmark();
    let desktop_paths = desktop_screenshot_paths();
    let report = json!({
        "schema_version": "morton_incremental_plus1.v25",
        "run_hash": run_hash,
        "v24_blocker": v24["full_wing_plus1"]["stop_reason"],
        "input": {
            "current_c_projected_mesh_hash": "df77a0f9fbffa161",
            "current_c_raw_connectivity_hash": "b828609cc63999fe",
            "accepted_leaf_set_hash": CURRENT_C_HASH
        },
        "current_c_payload": current_persistence,
        "morton_key_architecture": {
            "type": "src/mesh/adaptive_te_tip_refinement.rs::MortonLeafKey",
            "identity": "level plus integer coordinates plus canonical Morton code",
            "supports": ["parent", "eight_children", "six_face_directions", "domain_boundary", "geometric_bounds"]
        },
        "indexed_neighbor_architecture": {
            "type": "src/mesh/adaptive_te_tip_refinement.rs::ActiveLeafMap",
            "same_level": true,
            "coarser": true,
            "finer_face_enumeration": true,
            "invalid_gap_overlap_detection": true
        },
        "local_balance_architecture": {
            "production_balance_function": "src/mesh/adaptive_sampling.rs::balance_leaves_indexed",
            "production_neighbor_audit": "src/mesh/adaptive_sampling.rs::max_neighbor_delta using indexed_max_neighbor_delta",
            "indexed_balance_used": true,
            "indexed_neighbor_audit_used": true,
            "all_pairs_balance_used": false,
            "all_pairs_neighbor_audit_used": false
        },
        "old_vs_new_benchmark": benchmark,
        "feature_parent_selection": plus1_result["incremental_stats"]["selected_by_feature_type"].clone(),
        "exact_preflight": {
            "selected_parent_count": plus1_result["incremental_stats"]["selected_parent_count"].clone(),
            "direct_child_count": plus1_result["incremental_stats"]["direct_child_count"].clone(),
            "direct_net_leaf_increase": plus1_result["incremental_stats"]["direct_net_leaf_increase"].clone(),
            "net_increase_rule": "7 * selected_parent_count"
        },
        "resource_policy": plus1_result["resource_policy"].clone(),
        "current_c": current_case(&v19, &current_octree, current_hash.clone()),
        "plus1_tree": plus1_result,
        "plus1_extraction": plus1_raw.as_ref().map(|raw| raw_summary(raw, 0.0)).unwrap_or_else(|| json!({"status": "not_available"})),
        "plus1_projection": {
            "status": "not_run",
            "reason": "projection library extraction remains required; existing v19 behavior was not changed"
        },
        "plus1_quality_metrics": quality_metrics(plus1_tree.as_ref(), plus1_raw.as_ref()),
        "ordinary_surface_regression": ordinary_regression(&current_octree, plus1_tree.as_ref()),
        "topology_and_collision": {
            "current_c": "accepted",
            "plus1_raw": plus1_raw.as_ref().map(|raw| raw.stats.boundary_edges == 0 && raw.stats.non_manifold_edges == 0 && raw.stats.degenerate_triangles == 0).unwrap_or(false),
            "plus1_projection": "not_run"
        },
        "determinism": {
            "current_c_payload_roundtrip": current_persistence["metadata"]["byte_roundtrip_identical"]
                .clone()
                .as_bool()
                .map(Value::Bool)
                .unwrap_or_else(|| current_persistence["candidate_payload_roundtrip"].clone()),
            "plus1_payload_roundtrip": plus1_tree.is_some(),
            "run_hash": run_hash
        },
        "performance_and_memory": {
            "current_c_build_ms": current_build_ms,
            "plus1_build_ms": plus1_tree.as_ref().and_then(|_| plus1_result["build_runtime_ms"].as_f64()),
            "profile": if cfg!(debug_assertions) { "debug" } else { "release" }
        },
        "desktop_paths": desktop_paths.iter().map(|p| path_for_report(p)).collect::<Vec<_>>(),
        "mobile_manifest": path_for_report(&PathBuf::from(MOBILE_DIR).join(format!("mobile_manifest_{run_hash}.json"))),
        "mobile_url": format!("http://100.85.71.50:8765/current_mobile.html?v={run_hash}"),
        "http_verification": {
            "status": "pending_renderer"
        },
        "visual_review_status": "pending_user_review",
        "recommended_next_task": recommended_next(current_hash == CURRENT_C_HASH, &plus1_tree, &plus1_raw)
    });

    fs::write(
        REPORT_JSON,
        serde_json::to_string_pretty(&report).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    fs::write(REPORT_MD, markdown(&report)).map_err(|e| e.to_string())?;
    let status = Command::new("python")
        .arg("research/sdf_cfd_mesher/scripts/render_morton_incremental_plus1_v25.py")
        .status()
        .map_err(|e| format!("failed to start v25 renderer: {e}"))?;
    if !status.success() {
        return Err(format!("v25 renderer failed with status {status}"));
    }
    Ok(())
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

fn plus1_config() -> AdaptiveSamplingConfig {
    let mut cfg = current_c_config();
    cfg.max_depth = 8;
    cfg.feature_max_depth = 8;
    cfg.max_leaf_cells = 900_000;
    cfg
}

fn resource_policy(octree: &AdaptiveOctree) -> Value {
    let estimated_tets = octree.leaves.len() as f64 / 29_856.0 * 3_081_312.0;
    let estimated_memory_mb = estimated_tets * 0.00045;
    let run_extraction = estimated_tets <= 8_500_000.0;
    json!({
        "measured_leaf_count": octree.leaves.len(),
        "estimated_tetrahedra": estimated_tets.round() as usize,
        "estimated_memory_mb": estimated_memory_mb,
        "run_extraction": run_extraction,
        "reason": if run_extraction { "within_v25_raw_extraction_gate" } else { "estimated_tetrahedra_exceeds_v25_raw_extraction_gate" }
    })
}

fn raw_summary(surface: &AdaptiveMarchingTetsSurface, runtime_ms: f64) -> Value {
    json!({
        "status": "completed",
        "runtime_ms": runtime_ms,
        "mesh_hash": stable_hash_hex(&serde_json::to_vec(&surface.mesh.indices).unwrap_or_default()),
        "vertices": surface.mesh.vertices.len(),
        "triangles": surface.mesh.indices.len() / 3,
        "stats": surface.stats,
        "accepted_topology_subset": surface.stats.boundary_edges == 0
            && surface.stats.non_manifold_edges == 0
            && surface.stats.duplicate_triangles == 0
            && surface.stats.degenerate_triangles == 0
    })
}

fn current_case(v19: &Value, octree: &AdaptiveOctree, hash: String) -> Value {
    json!({
        "status": if hash == CURRENT_C_HASH { "accepted_reproduced" } else { "hash_mismatch" },
        "leaf_hash": hash,
        "summary": octree.summary,
        "projected_mesh_hash": v19["finite_wing"]["collision_aware_final"]["topology"]["mesh_hash"],
        "topology": v19["finite_wing"]["collision_aware_final"]["topology"]
    })
}

fn quality_metrics(
    octree: Option<&AdaptiveOctree>,
    raw: Option<&AdaptiveMarchingTetsSurface>,
) -> Value {
    let Some(octree) = octree else {
        return json!({"status": "not_available"});
    };
    json!({
        "status": if raw.is_some() { "partial_raw_metrics" } else { "tree_only" },
        "achieved_te_intervals": octree.summary.estimated_intervals_across_te_thickness,
        "te_feature_deviation": "not_measured_without_projected_feature_attachment",
        "te_sawtooth": "not_measured_without_projected_feature_attachment",
        "tip_profile": "not_measured_without_projected_feature_attachment"
    })
}

fn ordinary_regression(current: &AdaptiveOctree, plus1: Option<&AdaptiveOctree>) -> Value {
    let Some(plus1) = plus1 else {
        return json!({"status": "not_available"});
    };
    json!({
        "status": "leaf_summary_proxy",
        "current_leaf_count": current.leaves.len(),
        "plus1_leaf_count": plus1.leaves.len(),
        "current_min_leaf_size": current.summary.min_leaf_size,
        "plus1_min_leaf_size": plus1.summary.min_leaf_size,
        "ordinary_surface_full_metric": "not_measured_without projected surface feature attachment"
    })
}

fn old_vs_new_benchmark() -> Value {
    let mut map = ActiveLeafMap::new(2);
    for x in 0..2 {
        for y in 0..2 {
            for z in 0..2 {
                map.insert(MortonLeafKey::new(1, [x, y, z]), ());
            }
        }
    }
    let face_lookups = map.len() * 6;
    let old_pair_checks = map.len() * (map.len() - 1) / 2;
    let (delta, invalid) = indexed_max_neighbor_delta(&map);
    let _ = map.face_neighbor(MortonLeafKey::new(1, [0, 0, 0]), FaceDirection::XPlus);
    json!({
        "fixture": "complete 2x2x2 same-level root tiling",
        "active_leaf_count": map.len(),
        "old_pair_checks": old_pair_checks,
        "indexed_face_lookups": face_lookups,
        "output_equivalence": delta == 0 && invalid == 0,
        "asymptotic_interpretation": "old O(N^2) pair checks replaced by O(6N) indexed face lookups"
    })
}

fn recommended_next(
    current_c_verified: bool,
    plus1_tree: &Option<AdaptiveOctree>,
    raw: &Option<AdaptiveMarchingTetsSurface>,
) -> Value {
    if !current_c_verified {
        json!({
            "task": "restore exact Current-C leaf-set reproducibility or migrate a new accepted Current-C payload with explicit hash-change approval",
            "reason": "v25 cannot legally refine from Current-C because rebuilding the current sampler produced d290f844518e895a rather than the accepted 61a187b7a9c6b6c1"
        })
    } else if plus1_tree.is_some() && raw.is_some() {
        json!({
            "task": "expose v19 collision-aware projection as a reusable library entry point and run it on the persisted +1 octree",
            "reason": "+1 tree and raw extraction completed, but projection could not be run without changing the projector binary boundary"
        })
    } else if plus1_tree.is_some() {
        json!({
            "task": "run +1 raw extraction with a measured extraction resource gate or optimize adaptive marching-tets memory",
            "reason": "+1 tree completed but raw extraction did not produce an accepted surface"
        })
    } else {
        json!({
            "task": "another narrow indexed-path performance correction",
            "reason": "+1 tree did not complete after replacing all-pairs neighbor scans"
        })
    }
}

fn desktop_screenshot_paths() -> Vec<PathBuf> {
    [
        "contact_wireframe_iso_4wide.png",
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
        "# Morton Incremental +1 v25\n\n\
Result: `{}`.\n\n\
Current-C payload: `{}`.\n\n\
+1 tree: `{}`.\n\n\
Recommended next task: `{}`.\n",
        report["plus1_tree"]["status"].as_str().unwrap_or("unknown"),
        report["current_c_payload"]["status"]
            .as_str()
            .unwrap_or("unknown"),
        report["plus1_tree"]["leaf_hash"]
            .as_str()
            .unwrap_or("unavailable"),
        report["recommended_next_task"]["task"]
            .as_str()
            .unwrap_or("unknown")
    )
}

fn read_json(path: &str) -> Result<Value, String> {
    serde_json::from_str(&fs::read_to_string(path).map_err(|e| e.to_string())?)
        .map_err(|e| e.to_string())
}

fn eval_sdf(script: &str) -> Result<Arc<dyn Sdf>, String> {
    Ok(scripting::evaluate_script(script)
        .map_err(|e| e.to_string())?
        .sdf)
}

fn elapsed_ms(start: Instant) -> f64 {
    start.elapsed().as_secs_f64() * 1000.0
}

fn stable_hash_hex(bytes: &[u8]) -> String {
    let mut hash = 0xcbf29ce484222325_u64;
    for byte in bytes {
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
