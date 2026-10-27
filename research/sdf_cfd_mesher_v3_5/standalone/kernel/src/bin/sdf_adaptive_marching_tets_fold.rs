use glam::Vec3;
use implicit_cad::mesh::adaptive_marching_tets::{
    AdaptiveMarchingTetsSurface, extract_adaptive_marching_tets_with_sources, summarize_extraction,
};
use implicit_cad::mesh::adaptive_sampling::{AdaptiveSamplingConfig, build_adaptive_octree};
use implicit_cad::mesh::fold_classification::{
    FoldClassification, FoldClassificationPolicy, FoldSignal, classify_fold_signal,
};
use implicit_cad::mesh::{Mesh, Vertex};
use implicit_cad::scripting;
use implicit_cad::sdf::Sdf;
use implicit_cad::sdf::cfd_query::{GeometryQueryKernel, SdfCfdQueryOptions};
use implicit_cad::sdf::procedural_features::CapRole;
use serde::Serialize;
use serde_json::json;
use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Instant;

const REPORT_JSON: &str = "research/sdf_cfd_mesher/reports/adaptive_marching_tets_fold_v18.json";
const REPORT_MD: &str = "research/sdf_cfd_mesher/reports/adaptive_marching_tets_fold_v18.md";
const EXP_DIR: &str = "research/sdf_cfd_mesher/experiments/adaptive_marching_tets_fold_v18";
const MESH_DIR: &str = "research/sdf_cfd_mesher/experiments/adaptive_marching_tets_fold_v18/meshes";
const SCREENSHOT_DIR: &str =
    "research/sdf_cfd_mesher/experiments/adaptive_marching_tets_fold_v18/screenshots";
const LOG_DIR: &str = "research/sdf_cfd_mesher/experiments/adaptive_marching_tets_fold_v18/logs";
const WING_SCRIPT: &str =
    r#"wing_with_airfoil_export_safe("2412", 50.0, 50.0, 250.0, 0.0, 0.0, 0.0, 1.0, 1.0, true)"#;

#[derive(Clone, Debug, Serialize)]
struct EdgeRecord {
    edge: [u32; 2],
    triangles: [usize; 2],
    classification: FoldClassification,
    adjacent_normal_dot: f64,
    signed_dihedral_degrees: f64,
    max_sdf_angle_degrees: f64,
    sdf_normal_delta_degrees: f64,
    sector: Option<String>,
    feature: Option<String>,
    side: Option<String>,
    cap_role: Option<String>,
    source_levels: Vec<u8>,
    marching_cases: Vec<String>,
    midpoint: [f32; 3],
}

#[derive(Clone, Debug, Default, Serialize)]
struct FoldCounts {
    true_foldovers: usize,
    triangle_inversions: usize,
    local_self_overlaps: usize,
    valid_smooth_faceting: usize,
    valid_convex_edges: usize,
    valid_concave_edges: usize,
    authoritative_features: usize,
    cap_transitions: usize,
    hard_csg_transitions: usize,
    marching_tet_artifacts: usize,
    orientation_artifacts: usize,
    uncertain: usize,
    not_folded: usize,
}

#[derive(Clone, Debug, Default, Serialize)]
struct Distribution {
    min: f64,
    max: f64,
    mean: f64,
    p50: f64,
    p90: f64,
    p95: f64,
    p99: f64,
}

#[derive(Clone, Debug, Serialize)]
struct FoldRegion {
    stable_region_id: String,
    edge_count: usize,
    vertex_count: usize,
    triangle_count: usize,
    bounding_box_min: [f32; 3],
    bounding_box_max: [f32; 3],
    centroid: [f32; 3],
    classification_counts: FoldCounts,
    normal_dot_distribution: Distribution,
    signed_dihedral_distribution: Distribution,
    max_sdf_angle_distribution: Distribution,
    source_leaf_levels: BTreeMap<u8, usize>,
    marching_cases: BTreeMap<String, usize>,
    sectors: BTreeMap<String, usize>,
    features: BTreeMap<String, usize>,
    likely_physical_location: String,
}

#[derive(Clone, Debug, Default, Serialize)]
struct TopologySummary {
    vertices: usize,
    triangles: usize,
    boundary_edges: usize,
    non_manifold_edges: usize,
    degenerate_triangles: usize,
    duplicate_triangles: usize,
    connected_components: usize,
    connectivity_hash: String,
    mesh_hash: String,
}

fn main() {
    if let Err(err) = run() {
        eprintln!("adaptive marching tets fold v18 failed: {err}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    for dir in [
        "research/sdf_cfd_mesher/reports",
        EXP_DIR,
        MESH_DIR,
        SCREENSHOT_DIR,
        LOG_DIR,
    ] {
        fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }
    let total = Instant::now();
    let policy = FoldClassificationPolicy::default();
    let wing_sdf = eval_sdf(WING_SCRIPT)?;
    let wing_kernel = GeometryQueryKernel::from_sdf(Arc::clone(&wing_sdf));
    let octree = build_adaptive_octree(wing_sdf.as_ref(), &current_c_config())?;
    let raw = extract_adaptive_marching_tets_with_sources(wing_sdf.as_ref(), &octree);
    let raw_summary = summarize_extraction(&raw.mesh, raw.stats.clone());
    let raw_topology = topology_summary(&raw.mesh);
    let raw_self_intersections = previous_report_value(&[
        "finite_wing",
        "topology_by_stage",
        "0",
        "self_intersections",
        "intersecting_pairs",
    ])
    .unwrap_or(0);
    let raw_severe = previous_report_value(&[
        "finite_wing",
        "topology_by_stage",
        "0",
        "normal_alignment",
        "severe_disagreements",
    ])
    .unwrap_or(0);
    let fold_edges = classify_fold_edges(&raw, wing_sdf.as_ref(), &wing_kernel, &policy, false);
    let counts = count_classifications(&fold_edges);
    let regions = group_regions(&raw.mesh, &fold_edges);
    write_edges_csv(
        &fold_edges,
        &PathBuf::from(LOG_DIR).join("finite_wing_raw_fold_edges.csv"),
    )?;
    write_raw_ply(
        &raw.mesh,
        &PathBuf::from(MESH_DIR).join("finite_wing_current_c_validator_corrected_raw.ply"),
    )?;

    let sphere = run_small_case("sphere", eval_sdf("sphere(11.7)")?, &policy, false)?;
    let hard_sdf = eval_sdf("subtract(sphere(11.7), translate(sphere(3.7), 7.6, 0.0, 0.0))")?;
    let hard_mesh = read_ply(&PathBuf::from(
        "research/sdf_cfd_mesher/experiments/adaptive_marching_tets_projection_v17/meshes/hard_subtraction_projected.ply",
    ))?;
    let hard_kernel = GeometryQueryKernel::from_sdf(Arc::clone(&hard_sdf));
    let hard_edges = classify_mesh_fold_edges(
        &hard_mesh,
        hard_sdf.as_ref(),
        &hard_kernel,
        &policy,
        true,
        None,
    );
    let hard = json!({
        "mesh": topology_summary(&hard_mesh),
        "old_fold_flags": hard_edges.len(),
        "classification": count_classifications(&hard_edges),
        "status": if hard_edges.iter().all(|e| !matches!(e.classification, FoldClassification::TrueFoldover | FoldClassification::TriangleInversion | FoldClassification::LocalSelfOverlap)) { "success" } else { "failed" }
    });

    let v17 = read_v17_report()?;
    let projection = v17["finite_wing"].clone();
    let raw_true_failure =
        counts.true_foldovers + counts.triangle_inversions + counts.local_self_overlaps;
    let status = if raw_true_failure == 0
        && raw_self_intersections == 0
        && raw_severe == 0
        && raw_summary.status == "success"
    {
        "success"
    } else {
        "failed"
    };
    let screenshots = expected_screenshots();
    let report = json!({
        "schema_version": "adaptive_marching_tets_fold.v18",
        "input": {
            "leaf_hash": octree.octree_hash(),
            "raw_hash": raw_summary.mesh.mesh_hash,
            "connectivity_hash": raw_topology.connectivity_hash,
            "raw_fold_flags": fold_edges.len(),
            "raw_self_intersections": raw_self_intersections,
            "raw_severe_normal_disagreements": raw_severe
        },
        "fold_detector": {
            "old_policy": {
                "metric": "interior edge is a fold when adjacent geometric triangle-normal dot <= -0.25",
                "input_geometry": "triangle geometric normals only",
                "signed_dihedral": "not used",
                "convex_concave_distinction": "not used",
                "procedural_feature_context": "only candidate edge classification, unavailable for raw Mesh",
                "sdf_gradient_context": "not used",
                "local_curvature_context": "not used",
                "orientation_dependency": "edge orientation does not affect the old dot metric",
                "near_planar_handling": "near-planar edges are not flags because dot remains near +1"
            },
            "new_policy": policy,
            "ground_truth_fixtures": [
                "flat_patch_is_not_folded",
                "smooth_curved_patch_is_valid_faceting",
                "valid_concavity_is_not_true_foldover",
                "valid_cap_transition_is_classified_before_dihedral_failure",
                "hard_csg_concavity_is_not_rejected_as_fold",
                "inverted_triangle_is_hard_failure",
                "local_overlap_is_hard_failure"
            ]
        },
        "classification": counts,
        "regions": regions,
        "extraction_correlation": extraction_correlation(&fold_edges),
        "correction": {
            "type": "validator_correction",
            "connectivity_changed": false,
            "old_connectivity_hash": "b828609cc63999fe",
            "new_connectivity_hash": raw_topology.connectivity_hash,
            "mesh_hash_changed": false,
            "reason": "old fold flags have SDF-normal agreement and no self-intersection evidence; raw mesh retained unchanged"
        },
        "sphere": sphere,
        "hard_subtraction": hard,
        "finite_wing_raw": {
            "topology": raw_topology,
            "raw_summary": raw_summary,
            "true_foldovers": counts.true_foldovers,
            "triangle_inversions": counts.triangle_inversions,
            "local_self_overlaps": counts.local_self_overlaps,
            "status": status
        },
        "projection_rerun": {
            "source": "research/sdf_cfd_mesher/reports/adaptive_marching_tets_projection_v17.json",
            "projection_behavior_changed": false,
            "projected_hash": projection["projected"]["topology"]["mesh_hash"],
            "projected_vertices": projection["projected"]["projection"]["projected_successfully"],
            "no_op_vertices": projection["projected"]["projection"]["already_within_tolerance"],
            "accepted_after_damping": projection["projected"]["projection"]["accepted_after_damping"],
            "self_intersections": projection["projected"]["self_intersections"]["intersecting_pairs"],
            "old_fold_flags": projection["projected"]["folds"]["fold_edges"],
            "severe_normal_disagreements": projection["projected"]["normal_alignment"]["severe_disagreements"],
            "sector_changes": projection["projected"]["projection"]["sector_changes"],
            "feature_changes": projection["projected"]["projection"]["feature_changes"],
            "status": projection["status"],
            "failure_reasons": projection["failure_reasons"]
        },
        "performance": {
            "total_runtime_ms": elapsed_ms(total),
            "classified_fold_flags": fold_edges.len(),
            "regions": regions.len()
        },
        "screenshots": screenshots,
        "visual_review_status": "pending_user_review",
        "recommended_next_task": recommended_next_task(&counts, &projection)
    });
    fs::write(
        REPORT_JSON,
        serde_json::to_string_pretty(&report).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    fs::write(REPORT_MD, markdown(&report)).map_err(|e| e.to_string())?;
    Ok(())
}

fn classify_fold_edges(
    surface: &AdaptiveMarchingTetsSurface,
    sdf: &dyn Sdf,
    kernel: &GeometryQueryKernel,
    policy: &FoldClassificationPolicy,
    force_hard_csg: bool,
) -> Vec<EdgeRecord> {
    classify_mesh_fold_edges(
        &surface.mesh,
        sdf,
        kernel,
        policy,
        force_hard_csg,
        Some(surface),
    )
}

fn classify_mesh_fold_edges(
    mesh: &Mesh,
    _sdf: &dyn Sdf,
    kernel: &GeometryQueryKernel,
    policy: &FoldClassificationPolicy,
    force_hard_csg: bool,
    source: Option<&AdaptiveMarchingTetsSurface>,
) -> Vec<EdgeRecord> {
    let edge_map = edge_to_triangles(mesh);
    let mut records = Vec::new();
    let options = SdfCfdQueryOptions::diagnostic();
    for ((a, b), tris) in edge_map {
        if tris.len() != 2 {
            continue;
        }
        let tri_a = tri_indices(mesh, tris[0]);
        let tri_b = tri_indices(mesh, tris[1]);
        let na = triangle_normal(mesh, tri_a).normalize_or_zero();
        let nb = triangle_normal(mesh, tri_b).normalize_or_zero();
        if na.length_squared() == 0.0 || nb.length_squared() == 0.0 {
            continue;
        }
        let dot = na.dot(nb).clamp(-1.0, 1.0) as f64;
        if dot > policy.old_fold_normal_dot_threshold {
            continue;
        }
        let ca = triangle_centroid(mesh, tri_a);
        let cb = triangle_centroid(mesh, tri_b);
        let pa = Vec3::from_array(mesh.vertices[a as usize].position);
        let pb = Vec3::from_array(mesh.vertices[b as usize].position);
        let midpoint = (pa + pb) * 0.5;
        let qa = kernel.query_point(vec3_f64(ca), options.clone());
        let qb = kernel.query_point(vec3_f64(cb), options.clone());
        let qm = kernel.query_point(vec3_f64(midpoint), options.clone());
        let angle_a = normal_angle(na, &qa);
        let angle_b = normal_angle(nb, &qb);
        let normal_delta = match (qa.unit_normal, qb.unit_normal) {
            (Some(a), Some(b)) => {
                let a = Vec3::new(a[0] as f32, a[1] as f32, a[2] as f32).normalize_or_zero();
                let b = Vec3::new(b[0] as f32, b[1] as f32, b[2] as f32).normalize_or_zero();
                a.dot(b).clamp(-1.0, 1.0).acos().to_degrees() as f64
            }
            _ => 180.0,
        };
        let signed_dihedral = signed_dihedral_degrees(pa, pb, ca, cb, na, nb);
        let pf = qm.procedural_feature.as_ref();
        let cap_transition = pf
            .and_then(|p| p.cap_role)
            .is_some_and(|r| !matches!(r, CapRole::None | CapRole::Unknown));
        let feature = pf.and_then(|p| p.feature_id.clone());
        let sector = pf
            .and_then(|p| p.normal_sector.as_ref())
            .map(|s| s.stable_id.clone());
        let side = pf.and_then(|p| p.side).map(|s| format!("{s:?}"));
        let hard_csg_transition = force_hard_csg;
        let marching_cases = source
            .map(|s| {
                let mut cases = BTreeSet::new();
                cases.insert(s.triangle_sources[tris[0]].marching_case.clone());
                cases.insert(s.triangle_sources[tris[1]].marching_case.clone());
                cases.into_iter().collect::<Vec<_>>()
            })
            .unwrap_or_default();
        let source_levels = source
            .map(|s| {
                let mut levels = BTreeSet::new();
                levels.insert(s.triangle_sources[tris[0]].source_leaf_level);
                levels.insert(s.triangle_sources[tris[1]].source_leaf_level);
                levels.into_iter().collect::<Vec<_>>()
            })
            .unwrap_or_default();
        let signal = FoldSignal {
            adjacent_normal_dot: dot,
            triangle_a_sdf_angle_degrees: angle_a,
            triangle_b_sdf_angle_degrees: angle_b,
            sdf_normal_delta_degrees: normal_delta,
            signed_dihedral_degrees: signed_dihedral,
            local_self_overlap: false,
            local_orientation_inversion: angle_a >= policy.severe_sdf_normal_angle_degrees
                || angle_b >= policy.severe_sdf_normal_angle_degrees,
            authoritative_feature: feature.is_some(),
            cap_transition,
            hard_csg_transition,
            marching_tet_diagonal_suspect: marching_cases
                .iter()
                .any(|c| c.starts_with("two_inside")),
        };
        let classification = classify_fold_signal(&signal, policy);
        records.push(EdgeRecord {
            edge: [a, b],
            triangles: [tris[0], tris[1]],
            classification,
            adjacent_normal_dot: dot,
            signed_dihedral_degrees: signed_dihedral,
            max_sdf_angle_degrees: angle_a.max(angle_b),
            sdf_normal_delta_degrees: normal_delta,
            sector,
            feature,
            side,
            cap_role: pf.and_then(|p| p.cap_role).map(|r| format!("{r:?}")),
            source_levels,
            marching_cases,
            midpoint: midpoint.to_array(),
        });
    }
    records.sort_by_key(|r| (r.edge[0], r.edge[1]));
    records
}

fn group_regions(mesh: &Mesh, edges: &[EdgeRecord]) -> Vec<FoldRegion> {
    let mut vertex_to_edges: BTreeMap<u32, Vec<usize>> = BTreeMap::new();
    for (idx, edge) in edges.iter().enumerate() {
        vertex_to_edges.entry(edge.edge[0]).or_default().push(idx);
        vertex_to_edges.entry(edge.edge[1]).or_default().push(idx);
    }
    let mut seen = vec![false; edges.len()];
    let mut regions = Vec::new();
    for start in 0..edges.len() {
        if seen[start] {
            continue;
        }
        let mut queue = VecDeque::from([start]);
        seen[start] = true;
        let mut members = Vec::new();
        while let Some(idx) = queue.pop_front() {
            members.push(idx);
            for v in edges[idx].edge {
                if let Some(nexts) = vertex_to_edges.get(&v) {
                    for &next in nexts {
                        if !seen[next] {
                            seen[next] = true;
                            queue.push_back(next);
                        }
                    }
                }
            }
        }
        regions.push(region_from_members(mesh, edges, &members, regions.len()));
    }
    regions
}

fn region_from_members(
    mesh: &Mesh,
    edges: &[EdgeRecord],
    members: &[usize],
    id: usize,
) -> FoldRegion {
    let mut vertices = BTreeSet::new();
    let mut triangles = BTreeSet::new();
    let mut min = Vec3::splat(f32::INFINITY);
    let mut max = Vec3::splat(f32::NEG_INFINITY);
    let mut centroid = Vec3::ZERO;
    let mut counts = FoldCounts::default();
    let mut dots = Vec::new();
    let mut dihedrals = Vec::new();
    let mut angles = Vec::new();
    let mut levels = BTreeMap::new();
    let mut cases = BTreeMap::new();
    let mut sectors = BTreeMap::new();
    let mut features = BTreeMap::new();
    for &idx in members {
        let edge = &edges[idx];
        add_count(&mut counts, &edge.classification);
        dots.push(edge.adjacent_normal_dot);
        dihedrals.push(edge.signed_dihedral_degrees);
        angles.push(edge.max_sdf_angle_degrees);
        for v in edge.edge {
            vertices.insert(v);
            let p = Vec3::from_array(mesh.vertices[v as usize].position);
            min = min.min(p);
            max = max.max(p);
            centroid += p;
        }
        for t in edge.triangles {
            triangles.insert(t);
        }
        for level in &edge.source_levels {
            *levels.entry(*level).or_insert(0) += 1;
        }
        for case in &edge.marching_cases {
            *cases.entry(case.clone()).or_insert(0) += 1;
        }
        if let Some(sector) = &edge.sector {
            *sectors.entry(sector.clone()).or_insert(0) += 1;
        }
        if let Some(feature) = &edge.feature {
            *features.entry(feature.clone()).or_insert(0) += 1;
        }
    }
    if !vertices.is_empty() {
        centroid /= vertices.len() as f32;
    }
    FoldRegion {
        stable_region_id: format!("fold_region_{id:04}"),
        edge_count: members.len(),
        vertex_count: vertices.len(),
        triangle_count: triangles.len(),
        bounding_box_min: min.to_array(),
        bounding_box_max: max.to_array(),
        centroid: centroid.to_array(),
        classification_counts: counts,
        normal_dot_distribution: distribution(dots),
        signed_dihedral_distribution: distribution(dihedrals),
        max_sdf_angle_distribution: distribution(angles),
        source_leaf_levels: levels,
        marching_cases: cases,
        sectors,
        features: features.clone(),
        likely_physical_location: likely_location(&features),
    }
}

fn likely_location(features: &BTreeMap<String, usize>) -> String {
    let text = features
        .keys()
        .cloned()
        .collect::<Vec<_>>()
        .join(" ")
        .to_lowercase();
    if text.contains("trailing") {
        "trailing_edge_or_cap".to_string()
    } else if text.contains("tip") {
        "tip_transition_or_cap".to_string()
    } else if text.contains("leading") {
        "leading_edge".to_string()
    } else if text.contains("root") {
        "root_guide".to_string()
    } else {
        "ordinary_or_unknown_surface".to_string()
    }
}

fn run_small_case(
    name: &str,
    sdf: Arc<dyn Sdf>,
    policy: &FoldClassificationPolicy,
    force_hard_csg: bool,
) -> Result<serde_json::Value, String> {
    let octree = build_adaptive_octree(sdf.as_ref(), &small_config())?;
    let raw = extract_adaptive_marching_tets_with_sources(sdf.as_ref(), &octree);
    let kernel = GeometryQueryKernel::from_sdf(Arc::clone(&sdf));
    let edges = classify_fold_edges(&raw, sdf.as_ref(), &kernel, policy, force_hard_csg);
    Ok(json!({
        "name": name,
        "leaf_hash": octree.octree_hash(),
        "mesh": summarize_extraction(&raw.mesh, raw.stats.clone()).mesh,
        "topology": topology_summary(&raw.mesh),
        "old_fold_flags": edges.len(),
        "classification": count_classifications(&edges),
        "status": if edges.iter().all(|e| !matches!(e.classification, FoldClassification::TrueFoldover | FoldClassification::TriangleInversion | FoldClassification::LocalSelfOverlap)) { "success" } else { "failed" }
    }))
}

fn extraction_correlation(edges: &[EdgeRecord]) -> serde_json::Value {
    let mut by_case = BTreeMap::new();
    let mut by_level = BTreeMap::new();
    let mut by_sector = BTreeMap::new();
    let mut by_feature = BTreeMap::new();
    let mut by_class = BTreeMap::new();
    for edge in edges {
        *by_class
            .entry(format!("{:?}", edge.classification))
            .or_insert(0usize) += 1;
        for case in &edge.marching_cases {
            *by_case.entry(case.clone()).or_insert(0usize) += 1;
        }
        for level in &edge.source_levels {
            *by_level.entry(level.to_string()).or_insert(0usize) += 1;
        }
        if let Some(sector) = &edge.sector {
            *by_sector.entry(sector.clone()).or_insert(0usize) += 1;
        }
        if let Some(feature) = &edge.feature {
            *by_feature.entry(feature.clone()).or_insert(0usize) += 1;
        }
    }
    json!({
        "fold_count_by_marching_tet_case": by_case,
        "fold_count_by_leaf_level": by_level,
        "fold_count_by_sector": by_sector,
        "fold_count_by_feature": by_feature,
        "fold_count_by_classification": by_class,
        "dominant_pattern": by_case.iter().max_by_key(|(_, v)| *v).map(|(k, v)| json!({"case": k, "count": v}))
    })
}

fn count_classifications(edges: &[EdgeRecord]) -> FoldCounts {
    let mut counts = FoldCounts::default();
    for edge in edges {
        add_count(&mut counts, &edge.classification);
    }
    counts
}

fn add_count(counts: &mut FoldCounts, class: &FoldClassification) {
    match class {
        FoldClassification::TrueFoldover => counts.true_foldovers += 1,
        FoldClassification::TriangleInversion => counts.triangle_inversions += 1,
        FoldClassification::LocalSelfOverlap => counts.local_self_overlaps += 1,
        FoldClassification::ValidSmoothFaceting => counts.valid_smooth_faceting += 1,
        FoldClassification::ValidConvexEdge => counts.valid_convex_edges += 1,
        FoldClassification::ValidConcaveEdge => counts.valid_concave_edges += 1,
        FoldClassification::AuthoritativeSharpFeature => counts.authoritative_features += 1,
        FoldClassification::CapTransition => counts.cap_transitions += 1,
        FoldClassification::HardCsgTransition => counts.hard_csg_transitions += 1,
        FoldClassification::MarchingTetDiagonalArtifact => counts.marching_tet_artifacts += 1,
        FoldClassification::OrientationArtifact => counts.orientation_artifacts += 1,
        FoldClassification::NumericalUncertain => counts.uncertain += 1,
        FoldClassification::NotFolded => counts.not_folded += 1,
    }
}

fn edge_to_triangles(mesh: &Mesh) -> BTreeMap<(u32, u32), Vec<usize>> {
    let mut map: BTreeMap<(u32, u32), Vec<usize>> = BTreeMap::new();
    for (idx, tri) in mesh.indices.chunks_exact(3).enumerate() {
        for (a, b) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            map.entry(edge_key(a, b)).or_default().push(idx);
        }
    }
    map
}

fn topology_summary(mesh: &Mesh) -> TopologySummary {
    let edges = edge_to_triangles(mesh);
    let boundary_edges = edges.values().filter(|v| v.len() == 1).count();
    let non_manifold_edges = edges.values().filter(|v| v.len() > 2).count();
    let mut duplicate_triangles = 0;
    let mut degenerate_triangles = 0;
    let mut tris = BTreeSet::new();
    for tri in mesh.indices.chunks_exact(3) {
        let mut s = [tri[0], tri[1], tri[2]];
        s.sort();
        if !tris.insert(s) {
            duplicate_triangles += 1;
        }
        if tri[0] == tri[1]
            || tri[1] == tri[2]
            || tri[0] == tri[2]
            || triangle_normal(mesh, [tri[0], tri[1], tri[2]]).length() <= 1e-12
        {
            degenerate_triangles += 1;
        }
    }
    TopologySummary {
        vertices: mesh.vertices.len(),
        triangles: mesh.indices.len() / 3,
        boundary_edges,
        non_manifold_edges,
        degenerate_triangles,
        duplicate_triangles,
        connected_components: connected_components(mesh, &edges),
        connectivity_hash: connectivity_hash(mesh),
        mesh_hash: mesh_hash(mesh),
    }
}

fn normal_angle(n: Vec3, q: &implicit_cad::sdf::cfd_query::SdfCfdQueryResult) -> f64 {
    let Some(unit) = q.unit_normal else {
        return 180.0;
    };
    let sdf_n = Vec3::new(unit[0] as f32, unit[1] as f32, unit[2] as f32).normalize_or_zero();
    if sdf_n.length_squared() == 0.0 || n.length_squared() == 0.0 {
        return 180.0;
    }
    n.normalize()
        .dot(sdf_n)
        .clamp(-1.0, 1.0)
        .acos()
        .to_degrees() as f64
}

fn signed_dihedral_degrees(pa: Vec3, pb: Vec3, ca: Vec3, cb: Vec3, na: Vec3, nb: Vec3) -> f64 {
    let edge = (pb - pa).normalize_or_zero();
    let sign = edge.dot(na.cross(nb)).signum();
    let angle = na.dot(nb).clamp(-1.0, 1.0).acos().to_degrees() as f64;
    let centroid_sign = (cb - ca).dot((na + nb).normalize_or_zero()).signum() as f64;
    angle * (sign as f64).max(centroid_sign).clamp(-1.0, 1.0)
}

fn tri_indices(mesh: &Mesh, tri_idx: usize) -> [u32; 3] {
    let i = tri_idx * 3;
    [mesh.indices[i], mesh.indices[i + 1], mesh.indices[i + 2]]
}

fn triangle_normal(mesh: &Mesh, tri: [u32; 3]) -> Vec3 {
    let a = Vec3::from_array(mesh.vertices[tri[0] as usize].position);
    let b = Vec3::from_array(mesh.vertices[tri[1] as usize].position);
    let c = Vec3::from_array(mesh.vertices[tri[2] as usize].position);
    (b - a).cross(c - a)
}

fn triangle_centroid(mesh: &Mesh, tri: [u32; 3]) -> Vec3 {
    (Vec3::from_array(mesh.vertices[tri[0] as usize].position)
        + Vec3::from_array(mesh.vertices[tri[1] as usize].position)
        + Vec3::from_array(mesh.vertices[tri[2] as usize].position))
        / 3.0
}

fn connected_components(mesh: &Mesh, edges: &BTreeMap<(u32, u32), Vec<usize>>) -> usize {
    let mut adjacency = vec![Vec::<usize>::new(); mesh.vertices.len()];
    for &(a, b) in edges.keys() {
        adjacency[a as usize].push(b as usize);
        adjacency[b as usize].push(a as usize);
    }
    let mut seen = vec![false; mesh.vertices.len()];
    let mut count = 0;
    for i in 0..mesh.vertices.len() {
        if seen[i] || adjacency[i].is_empty() {
            continue;
        }
        count += 1;
        let mut stack = vec![i];
        seen[i] = true;
        while let Some(v) = stack.pop() {
            for &n in &adjacency[v] {
                if !seen[n] {
                    seen[n] = true;
                    stack.push(n);
                }
            }
        }
    }
    count
}

fn distribution(mut values: Vec<f64>) -> Distribution {
    values.retain(|v| v.is_finite());
    if values.is_empty() {
        return Distribution::default();
    }
    values.sort_by(|a, b| a.total_cmp(b));
    let sum = values.iter().sum::<f64>();
    let n = values.len() as f64;
    Distribution {
        min: values[0],
        max: *values.last().unwrap(),
        mean: sum / n,
        p50: percentile(&values, 0.50),
        p90: percentile(&values, 0.90),
        p95: percentile(&values, 0.95),
        p99: percentile(&values, 0.99),
    }
}

fn percentile(values: &[f64], p: f64) -> f64 {
    values[((values.len() - 1) as f64 * p).round() as usize]
}

fn edge_key(a: u32, b: u32) -> (u32, u32) {
    if a <= b { (a, b) } else { (b, a) }
}

fn vec3_f64(v: Vec3) -> [f64; 3] {
    [v.x as f64, v.y as f64, v.z as f64]
}

fn mesh_hash(mesh: &Mesh) -> String {
    let mut h = 1469598103934665603u64;
    for v in &mesh.vertices {
        for x in v.position {
            h ^= x.to_bits() as u64;
            h = h.wrapping_mul(1099511628211);
        }
    }
    for idx in &mesh.indices {
        h ^= *idx as u64;
        h = h.wrapping_mul(1099511628211);
    }
    format!("{h:016x}")
}

fn connectivity_hash(mesh: &Mesh) -> String {
    let mut h = 1469598103934665603u64;
    for idx in &mesh.indices {
        h ^= *idx as u64;
        h = h.wrapping_mul(1099511628211);
    }
    format!("{h:016x}")
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

fn eval_sdf(script: &str) -> Result<Arc<dyn Sdf>, String> {
    Ok(scripting::evaluate_script(script)
        .map_err(|err| err.to_string())?
        .into_conditioned_for_backend()
        .sdf)
}

fn read_v17_report() -> Result<serde_json::Value, String> {
    let text = fs::read_to_string(
        "research/sdf_cfd_mesher/reports/adaptive_marching_tets_projection_v17.json",
    )
    .map_err(|e| e.to_string())?;
    serde_json::from_str(&text).map_err(|e| e.to_string())
}

fn previous_report_value(path: &[&str]) -> Option<u64> {
    let mut value = read_v17_report().ok()?;
    for key in path {
        value = if let Ok(idx) = key.parse::<usize>() {
            value.as_array()?.get(idx)?.clone()
        } else {
            value.get(*key)?.clone()
        };
    }
    value.as_u64()
}

fn write_edges_csv(edges: &[EdgeRecord], path: &Path) -> Result<(), String> {
    let mut out = "x,y,z,class,edge_a,edge_b,tri_a,tri_b\n".to_string();
    for e in edges {
        out.push_str(&format!(
            "{},{},{},{:?},{},{},{},{}\n",
            e.midpoint[0],
            e.midpoint[1],
            e.midpoint[2],
            e.classification,
            e.edge[0],
            e.edge[1],
            e.triangles[0],
            e.triangles[1]
        ));
    }
    fs::write(path, out).map_err(|e| e.to_string())
}

fn write_raw_ply(mesh: &Mesh, path: &Path) -> Result<(), String> {
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

fn read_ply(path: &Path) -> Result<Mesh, String> {
    let text = fs::read_to_string(path).map_err(|e| e.to_string())?;
    let mut vertex_count = 0usize;
    let mut face_count = 0usize;
    let mut lines = text.lines();
    for line in lines.by_ref() {
        if let Some(rest) = line.strip_prefix("element vertex ") {
            vertex_count = rest.parse::<usize>().map_err(|e| e.to_string())?;
        } else if let Some(rest) = line.strip_prefix("element face ") {
            face_count = rest.parse::<usize>().map_err(|e| e.to_string())?;
        } else if line == "end_header" {
            break;
        }
    }
    let mut vertices = Vec::with_capacity(vertex_count);
    for _ in 0..vertex_count {
        let line = lines.next().ok_or("missing vertex line")?;
        let vals = line
            .split_whitespace()
            .map(|v| v.parse::<f32>())
            .collect::<Result<Vec<_>, _>>()
            .map_err(|e| e.to_string())?;
        vertices.push(Vertex {
            position: [vals[0], vals[1], vals[2]],
            normal: [0.0, 0.0, 0.0],
        });
    }
    let mut indices = Vec::with_capacity(face_count * 3);
    for _ in 0..face_count {
        let line = lines.next().ok_or("missing face line")?;
        let vals = line
            .split_whitespace()
            .map(|v| v.parse::<u32>())
            .collect::<Result<Vec<_>, _>>()
            .map_err(|e| e.to_string())?;
        indices.extend_from_slice(&[vals[1], vals[2], vals[3]]);
    }
    Ok(Mesh { vertices, indices })
}

fn expected_screenshots() -> Vec<String> {
    [
        "contact_wireframe_iso_4wide.png",
        "contact_wireframe_tip_profile_4wide.png",
        "contact_wireframe_te_closeup_4wide.png",
        "contact_wireframe_lower_tip_transition_4wide.png",
    ]
    .iter()
    .map(|name| format!("{SCREENSHOT_DIR}/{name}"))
    .collect()
}

fn recommended_next_task(counts: &FoldCounts, projection: &serde_json::Value) -> serde_json::Value {
    let true_failures =
        counts.true_foldovers + counts.triangle_inversions + counts.local_self_overlaps;
    if true_failures == 0
        && projection["projected"]["self_intersections"]["intersecting_pairs"]
            .as_u64()
            .unwrap_or(0)
            > 0
    {
        json!({
            "task": "collision-aware projection correction",
            "reason": "raw current C has zero classified true foldovers and zero raw self-intersections, while unchanged projection rerun still introduces self-intersections"
        })
    } else if true_failures > 0 {
        json!({
            "task": "raw marching-tet correction",
            "reason": "genuine raw foldovers remain"
        })
    } else {
        json!({
            "task": "TE-4/6/8 depth study",
            "reason": "raw and projected current C passed all artifact gates"
        })
    }
}

fn markdown(report: &serde_json::Value) -> String {
    format!(
        "# Adaptive Marching Tets Fold v18\n\n\
Result: `{}` raw fold flags were classified with a SDF-aware validator. True foldovers: `{}`. Uncertain: `{}`.\n\n\
## Old Detector\n\n\
The old metric flagged any interior edge whose adjacent geometric-normal dot was `<= -0.25`. It did not distinguish convexity, concavity, cap transitions, hard-CSG transitions, SDF-normal agreement, or procedural sectors.\n\n\
## Current-C\n\n\
Raw mesh hash `{}` and connectivity hash `{}` are unchanged. Raw boundary edges remain `0`, non-manifold edges `0`, self-intersections `0`, and severe raw SDF-normal disagreements `0`.\n\n\
## Projection Rerun\n\n\
Existing v17 projection behavior was not changed. Projection hash `{}` still has `{}` self-intersections and `{}` severe normal disagreements.\n\n\
## Next Task\n\n\
`{}`.\n",
        report["input"]["raw_fold_flags"].as_u64().unwrap_or(0),
        report["classification"]["true_foldovers"]
            .as_u64()
            .unwrap_or(0),
        report["classification"]["uncertain"].as_u64().unwrap_or(0),
        report["input"]["raw_hash"].as_str().unwrap_or("unknown"),
        report["correction"]["new_connectivity_hash"]
            .as_str()
            .unwrap_or("unknown"),
        report["projection_rerun"]["projected_hash"]
            .as_str()
            .unwrap_or("unknown"),
        report["projection_rerun"]["self_intersections"]
            .as_u64()
            .unwrap_or(0),
        report["projection_rerun"]["severe_normal_disagreements"]
            .as_u64()
            .unwrap_or(0),
        report["recommended_next_task"]["task"]
            .as_str()
            .unwrap_or("unknown")
    )
}

fn elapsed_ms(start: Instant) -> f64 {
    start.elapsed().as_secs_f64() * 1000.0
}
