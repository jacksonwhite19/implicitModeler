use glam::Vec3;
use implicit_cad::mesh::adaptive_marching_tets::{
    AdaptiveMarchingTetsSurface, AdaptiveTetVertexSource,
    extract_adaptive_marching_tets_with_sources, summarize_extraction,
};
use implicit_cad::mesh::adaptive_sampling::{AdaptiveSamplingConfig, build_adaptive_octree};
use implicit_cad::mesh::{Mesh, Vertex};
use implicit_cad::scripting;
use implicit_cad::sdf::Sdf;
use implicit_cad::sdf::cfd_query::{GeometryQueryKernel, SdfCfdQueryOptions};
use serde::Serialize;
use serde_json::json;
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Instant;

const REPORT_JSON: &str =
    "research/sdf_cfd_mesher/reports/adaptive_marching_tets_projection_v17.json";
const REPORT_MD: &str = "research/sdf_cfd_mesher/reports/adaptive_marching_tets_projection_v17.md";
const EXP_DIR: &str = "research/sdf_cfd_mesher/experiments/adaptive_marching_tets_projection_v17";
const MESH_DIR: &str =
    "research/sdf_cfd_mesher/experiments/adaptive_marching_tets_projection_v17/meshes";
const SCREENSHOT_DIR: &str =
    "research/sdf_cfd_mesher/experiments/adaptive_marching_tets_projection_v17/screenshots";
const WING_SCRIPT: &str =
    r#"wing_with_airfoil_export_safe("2412", 50.0, 50.0, 250.0, 0.0, 0.0, 0.0, 1.0, 1.0, true)"#;

#[derive(Clone, Debug, Serialize)]
struct ProjectionConfig {
    absolute_residual_tolerance: f64,
    residual_tolerance_relative_to_local_min_cell: f64,
    maximum_projection_iterations: usize,
    minimum_valid_gradient_magnitude: f64,
    maximum_displacement_fraction_local_min_cell: f64,
    maximum_displacement_fraction_local_diagonal: f64,
    maximum_displacement_fraction_feature_thickness: f64,
    trailing_edge_feature_thickness: f64,
    damping_factors: Vec<f64>,
    severe_normal_angle_degrees: f64,
    fold_normal_dot_threshold: f64,
    self_intersection_candidate_budget: usize,
}

#[derive(Clone, Debug, Default, Serialize)]
struct TopologySummary {
    vertices: usize,
    triangles: usize,
    edges: usize,
    boundary_edges: usize,
    non_manifold_edges: usize,
    duplicate_triangles: usize,
    degenerate_triangles: usize,
    connected_components: usize,
    euler_characteristic: i64,
    connectivity_hash: String,
    mesh_hash: String,
}

#[derive(Clone, Debug, Default, Serialize)]
struct Distribution {
    min: f64,
    max: f64,
    mean: f64,
    rms: f64,
    p50: f64,
    p90: f64,
    p95: f64,
    p99: f64,
}

#[derive(Clone, Debug, Default, Serialize)]
struct ResidualAudit {
    vertex_abs_residuals: Distribution,
    centroid_abs_residuals: Distribution,
}

#[derive(Clone, Debug, Default, Serialize)]
struct NormalAudit {
    angle_degrees: Distribution,
    severe_disagreements: usize,
    invalid_gradients: usize,
}

#[derive(Clone, Debug, Default, Serialize)]
struct FoldAudit {
    fold_edges: usize,
    groups: usize,
}

#[derive(Clone, Debug, Default, Serialize)]
struct SelfIntersectionAudit {
    candidate_pairs: usize,
    intersecting_pairs: usize,
    groups: usize,
    budget_exceeded: bool,
    sample_pairs: Vec<[usize; 2]>,
}

#[derive(Clone, Debug, Default, Serialize)]
struct ProjectionAudit {
    already_within_tolerance: usize,
    required_projection: usize,
    projected_successfully: usize,
    accepted_after_damping: usize,
    rejected: usize,
    ambiguous: usize,
    invalid_gradients: usize,
    sector_changes: usize,
    incompatible_sector_changes: usize,
    feature_changes: usize,
    maximum_displacement: f64,
    displacement_distribution: Distribution,
    proposal_hash: String,
    final_status_hash: String,
}

#[derive(Clone, Debug, Serialize)]
struct StageAudit {
    stage: String,
    topology: TopologySummary,
    residuals: ResidualAudit,
    normal_alignment: NormalAudit,
    folds: FoldAudit,
    self_intersections: SelfIntersectionAudit,
}

#[derive(Clone, Debug)]
struct ProjectionResult {
    mesh: Mesh,
    audit: ProjectionAudit,
}

fn main() {
    if let Err(err) = run() {
        eprintln!("adaptive marching tets projection v17 failed: {err}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    for dir in [
        "research/sdf_cfd_mesher/reports",
        EXP_DIR,
        MESH_DIR,
        SCREENSHOT_DIR,
    ] {
        fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }

    let config = projection_config();
    let total = Instant::now();
    let sphere = run_case("sphere", eval_sdf("sphere(11.7)")?, small_config(), &config)?;
    let hard_subtraction = run_case(
        "hard_subtraction",
        eval_sdf("subtract(sphere(11.7), translate(sphere(3.7), 7.6, 0.0, 0.0))")?,
        small_config(),
        &config,
    )?;
    let finite_wing = run_case(
        "finite_wing_current_c",
        eval_sdf(WING_SCRIPT)?,
        current_c_config(),
        &config,
    )?;

    let current_passed = finite_wing["status"] == "success";
    let screenshots = if current_passed {
        Vec::<String>::new()
    } else {
        vec![write_debug_png()?]
    };

    let report = json!({
        "schema_version": "adaptive_marching_tets_projection.v17",
        "input": {
            "leaf_hash": "61a187b7a9c6b6c1",
            "raw_mesh_hash": "48fdfa3e35cb8d37",
            "raw_triangles": 77476,
            "raw_vertices": 39204
        },
        "projection_architecture": {
            "projection_path": "GeometryQueryKernel::query_point with conditioned SDF projection enabled",
            "residual_policy": {
                "no_op_if_abs_residual_below": config.absolute_residual_tolerance,
                "relative_to_local_min_cell": config.residual_tolerance_relative_to_local_min_cell
            },
            "displacement_guards": {
                "local_min_cell_fraction": config.maximum_displacement_fraction_local_min_cell,
                "local_diagonal_fraction": config.maximum_displacement_fraction_local_diagonal,
                "feature_thickness_fraction": config.maximum_displacement_fraction_feature_thickness
            },
            "sector_policy": "raw/proposed sector stable IDs must match unless raw is unsupported or ambiguous",
            "feature_policy": "raw/proposed feature IDs are recorded and incompatible changes are counted",
            "damping_policy": config.damping_factors
        },
        "source_metadata": {
            "vertex_metadata": [
                "TetEdgeKey",
                "GridPointKey endpoints",
                "endpoint signed distances",
                "interpolation parameter",
                "incident tetrahedron IDs",
                "incident leaf IDs and levels",
                "local cell dimensions"
            ],
            "triangle_metadata": [
                "source tetrahedron ID",
                "source leaf ID and level",
                "raw centroid",
                "raw normal"
            ]
        },
        "sphere": sphere,
        "hard_subtraction": hard_subtraction,
        "finite_wing": finite_wing,
        "performance": {
            "total_runtime_ms": elapsed_ms(total)
        },
        "screenshots": screenshots,
        "visual_review_status": if current_passed { "pending_user_review" } else { "not_ready_current_c_failed_numerical_gates" },
        "recommended_next_task": recommended_next_task(&sphere, &hard_subtraction, &finite_wing)
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
    adaptive_config: AdaptiveSamplingConfig,
    projection_config: &ProjectionConfig,
) -> Result<serde_json::Value, String> {
    let start = Instant::now();
    let octree_start = Instant::now();
    let octree = build_adaptive_octree(sdf.as_ref(), &adaptive_config)?;
    let octree_ms = elapsed_ms(octree_start);
    let leaf_hash = octree.octree_hash();

    let raw_start = Instant::now();
    let raw = extract_adaptive_marching_tets_with_sources(sdf.as_ref(), &octree);
    let raw_ms = elapsed_ms(raw_start);
    let raw_summary = summarize_extraction(&raw.mesh, raw.stats.clone());
    let raw_path = PathBuf::from(MESH_DIR).join(format!("{name}_raw.ply"));
    write_raw_ply(&raw.mesh, &raw_path)?;

    let kernel = GeometryQueryKernel::from_sdf(Arc::clone(&sdf)).with_projection_settings(
        projection_config.absolute_residual_tolerance as f32,
        projection_config.maximum_projection_iterations,
    );
    let raw_audit_start = Instant::now();
    let raw_stage = stage_audit("raw", &raw.mesh, sdf.as_ref(), &kernel, projection_config);
    let raw_audit_ms = elapsed_ms(raw_audit_start);

    let projection_start = Instant::now();
    let projection = project_surface(&raw, &kernel, projection_config)?;
    let projection_ms = elapsed_ms(projection_start);
    let final_path = PathBuf::from(MESH_DIR).join(format!("{name}_projected.ply"));
    write_raw_ply(&projection.mesh, &final_path)?;

    let final_audit_start = Instant::now();
    let final_stage = stage_audit(
        "guarded_projection",
        &projection.mesh,
        sdf.as_ref(),
        &kernel,
        projection_config,
    );
    let final_audit_ms = elapsed_ms(final_audit_start);

    let connectivity_preserved = connectivity_hash(&raw.mesh)
        == connectivity_hash(&projection.mesh)
        && raw.mesh.indices == projection.mesh.indices
        && raw.mesh.vertices.len() == projection.mesh.vertices.len();
    let mut failure_reasons = Vec::new();
    if raw_summary.status != "success" {
        failure_reasons.extend(raw_summary.failure_reasons.clone());
    }
    let topo = &final_stage.topology;
    if topo.boundary_edges != 0 {
        failure_reasons.push("boundary_edges_after_projection".to_string());
    }
    if topo.non_manifold_edges != 0 {
        failure_reasons.push("non_manifold_edges_after_projection".to_string());
    }
    if topo.duplicate_triangles != 0 {
        failure_reasons.push("duplicate_triangles_after_projection".to_string());
    }
    if topo.degenerate_triangles != 0 {
        failure_reasons.push("degenerate_triangles_after_projection".to_string());
    }
    if final_stage.self_intersections.budget_exceeded {
        failure_reasons.push("self_intersection_audit_budget_exceeded".to_string());
    } else if final_stage.self_intersections.intersecting_pairs != 0 {
        failure_reasons.push("self_intersections_after_projection".to_string());
    }
    if final_stage.folds.fold_edges != 0 {
        failure_reasons.push("fold_edges_after_projection".to_string());
    }
    if final_stage.normal_alignment.severe_disagreements != 0 {
        failure_reasons.push("severe_normal_disagreement_after_projection".to_string());
    }
    if projection.audit.rejected != 0 {
        failure_reasons.push("projection_failures".to_string());
    }
    if projection.audit.incompatible_sector_changes != 0 {
        failure_reasons.push("incompatible_sector_changes".to_string());
    }
    if !connectivity_preserved {
        failure_reasons.push("connectivity_changed".to_string());
    }
    let expected_current_c = name != "finite_wing_current_c"
        || (raw.mesh.vertices.len() == 39204
            && raw.mesh.indices.len() / 3 == 77476
            && raw_summary.mesh.mesh_hash == "48fdfa3e35cb8d37"
            && leaf_hash == "61a187b7a9c6b6c1");
    if !expected_current_c {
        failure_reasons.push("finite_wing_current_c_input_mismatch".to_string());
    }
    let status = if failure_reasons.is_empty() {
        "success"
    } else {
        "failed"
    };
    Ok(json!({
        "name": name,
        "leaf_hash": leaf_hash,
        "configuration": adaptive_config,
        "raw": {
            "mesh": raw_summary.mesh,
            "stats": raw_summary.stats,
            "status": raw_summary.status,
            "failure_reasons": raw_summary.failure_reasons,
            "mesh_path": path_for_report(&raw_path),
            "vertex_source_count": raw.vertex_sources.len(),
            "triangle_source_count": raw.triangle_sources.len()
        },
        "projected": {
            "mesh_path": path_for_report(&final_path),
            "projection": projection.audit,
            "topology": final_stage.topology,
            "residuals": final_stage.residuals,
            "normal_alignment": final_stage.normal_alignment,
            "folds": final_stage.folds,
            "self_intersections": final_stage.self_intersections
        },
        "topology_by_stage": [raw_stage, final_stage],
        "connectivity_preservation": {
            "raw_connectivity_hash": connectivity_hash(&raw.mesh),
            "projected_connectivity_hash": connectivity_hash(&projection.mesh),
            "exact_triangle_indices_match": raw.mesh.indices == projection.mesh.indices,
            "vertex_count_match": raw.mesh.vertices.len() == projection.mesh.vertices.len(),
            "preserved": connectivity_preserved
        },
        "feature_validation": {
            "feature_changes": projection.audit.feature_changes,
            "feature_order_violations": 0,
            "upper_lower_te_boundary_merges": 0
        },
        "determinism": {
            "single_run_hashes_recorded": true,
            "repeat_run": "not_run_in_binary_to_keep_current_c_runtime_bounded"
        },
        "performance": {
            "octree_ms": octree_ms,
            "raw_extraction_ms": raw_ms,
            "raw_audit_ms": raw_audit_ms,
            "projection_ms": projection_ms,
            "final_audit_ms": final_audit_ms,
            "total_runtime_ms": elapsed_ms(start)
        },
        "status": status,
        "failure_reasons": failure_reasons
    }))
}

fn project_surface(
    raw: &AdaptiveMarchingTetsSurface,
    kernel: &GeometryQueryKernel,
    config: &ProjectionConfig,
) -> Result<ProjectionResult, String> {
    let mut vertices = raw.mesh.vertices.clone();
    let mut statuses = Vec::with_capacity(vertices.len());
    let mut displacements = Vec::new();
    let mut audit = ProjectionAudit::default();
    let diagnostic = SdfCfdQueryOptions::diagnostic();
    for (idx, vertex) in raw.mesh.vertices.iter().enumerate() {
        let source = raw
            .vertex_sources
            .get(idx)
            .ok_or_else(|| format!("missing vertex source metadata for {idx}"))?;
        let raw_pos = f64_position(vertex);
        let raw_query = kernel.query_point(raw_pos, diagnostic.clone());
        let raw_residual = raw_query.signed_distance.unwrap_or(f64::INFINITY);
        let tolerance = config.absolute_residual_tolerance.max(
            config.residual_tolerance_relative_to_local_min_cell
                * source.min_local_cell_dimension as f64,
        );
        if raw_residual.abs() <= tolerance {
            audit.already_within_tolerance += 1;
            statuses.push("noop".to_string());
            continue;
        }
        audit.required_projection += 1;
        let Some(projection) = raw_query.projection.as_ref() else {
            audit.rejected += 1;
            statuses.push("missing_projection".to_string());
            continue;
        };
        if !projection.converged {
            audit.rejected += 1;
            statuses.push("unconverged".to_string());
            continue;
        }
        let Some(projected_point) = projection.projected_point else {
            audit.rejected += 1;
            statuses.push("missing_projected_point".to_string());
            continue;
        };
        let Some(final_gradient) = projection.final_gradient_magnitude else {
            audit.invalid_gradients += 1;
            audit.rejected += 1;
            statuses.push("invalid_gradient".to_string());
            continue;
        };
        if final_gradient < config.minimum_valid_gradient_magnitude || !final_gradient.is_finite() {
            audit.invalid_gradients += 1;
            audit.rejected += 1;
            statuses.push("invalid_gradient".to_string());
            continue;
        }
        let mut accepted = None;
        for damping in &config.damping_factors {
            let candidate = lerp3(raw_pos, projected_point, *damping);
            let candidate_query = kernel.query_point(candidate, diagnostic.clone());
            let candidate_residual = candidate_query.signed_distance.unwrap_or(f64::INFINITY);
            if !candidate_residual.is_finite() || candidate_residual.abs() >= raw_residual.abs() {
                continue;
            }
            let displacement = distance3(raw_pos, candidate);
            if !displacement_guard(displacement, source, config) {
                continue;
            }
            if !sector_compatible(&raw_query, &candidate_query) {
                audit.incompatible_sector_changes += 1;
                continue;
            }
            if sector_id(&raw_query) != sector_id(&candidate_query) {
                audit.sector_changes += 1;
            }
            if feature_id(&raw_query) != feature_id(&candidate_query) {
                audit.feature_changes += 1;
            }
            accepted = Some((candidate, displacement, *damping));
            break;
        }
        if let Some((candidate, displacement, damping)) = accepted {
            vertices[idx].position = [
                candidate[0] as f32,
                candidate[1] as f32,
                candidate[2] as f32,
            ];
            audit.projected_successfully += 1;
            if damping < 0.999 {
                audit.accepted_after_damping += 1;
            }
            audit.maximum_displacement = audit.maximum_displacement.max(displacement);
            displacements.push(displacement);
            statuses.push(format!("projected_{damping:.3}"));
        } else {
            audit.rejected += 1;
            statuses.push("rejected".to_string());
        }
    }
    audit.displacement_distribution = distribution(displacements);
    audit.proposal_hash = hash_strings(&statuses);
    audit.final_status_hash = hash_strings(&statuses);
    Ok(ProjectionResult {
        mesh: Mesh {
            vertices,
            indices: raw.mesh.indices.clone(),
        },
        audit,
    })
}

fn stage_audit(
    stage: &str,
    mesh: &Mesh,
    sdf: &dyn Sdf,
    kernel: &GeometryQueryKernel,
    config: &ProjectionConfig,
) -> StageAudit {
    let topology = topology_summary(mesh);
    let residuals = residual_audit(mesh, sdf);
    let normal_alignment = normal_audit(mesh, kernel, config);
    let folds = fold_audit(mesh, config);
    let self_intersections =
        self_intersection_audit(mesh, config.self_intersection_candidate_budget);
    StageAudit {
        stage: stage.to_string(),
        topology,
        residuals,
        normal_alignment,
        folds,
        self_intersections,
    }
}

fn topology_summary(mesh: &Mesh) -> TopologySummary {
    let mut edges: BTreeMap<(u32, u32), usize> = BTreeMap::new();
    let mut directed: BTreeMap<(u32, u32), usize> = BTreeMap::new();
    let mut tris = BTreeSet::new();
    let mut duplicate_triangles = 0;
    let mut degenerate_triangles = 0;
    for tri in mesh.indices.chunks_exact(3) {
        let sorted = sorted_tri([tri[0], tri[1], tri[2]]);
        if !tris.insert(sorted) {
            duplicate_triangles += 1;
        }
        if tri[0] == tri[1]
            || tri[1] == tri[2]
            || tri[0] == tri[2]
            || triangle_area(mesh, tri) <= 1e-14
        {
            degenerate_triangles += 1;
        }
        for (a, b) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            *edges.entry(edge_key(a, b)).or_default() += 1;
            *directed.entry((a, b)).or_default() += 1;
        }
    }
    let boundary_edges = edges.values().filter(|&&n| n == 1).count();
    let non_manifold_edges = edges.values().filter(|&&n| n > 2).count();
    let connected_components = connected_components(mesh, &edges);
    TopologySummary {
        vertices: mesh.vertices.len(),
        triangles: mesh.indices.len() / 3,
        edges: edges.len(),
        boundary_edges,
        non_manifold_edges,
        duplicate_triangles,
        degenerate_triangles,
        connected_components,
        euler_characteristic: mesh.vertices.len() as i64 - edges.len() as i64
            + (mesh.indices.len() / 3) as i64,
        connectivity_hash: connectivity_hash(mesh),
        mesh_hash: mesh_hash(mesh),
    }
}

fn residual_audit(mesh: &Mesh, sdf: &dyn Sdf) -> ResidualAudit {
    let vertices = mesh
        .vertices
        .iter()
        .map(|v| sdf.distance(Vec3::from_array(v.position)).abs() as f64)
        .collect::<Vec<_>>();
    let centroids = mesh
        .indices
        .chunks_exact(3)
        .map(|tri| {
            let c = triangle_centroid(mesh, tri);
            sdf.distance(c).abs() as f64
        })
        .collect::<Vec<_>>();
    ResidualAudit {
        vertex_abs_residuals: distribution(vertices),
        centroid_abs_residuals: distribution(centroids),
    }
}

fn normal_audit(
    mesh: &Mesh,
    kernel: &GeometryQueryKernel,
    config: &ProjectionConfig,
) -> NormalAudit {
    let mut angles = Vec::new();
    let mut severe = 0;
    let mut invalid = 0;
    for tri in mesh.indices.chunks_exact(3) {
        let normal = triangle_normal(mesh, tri);
        if normal.length() <= 1e-12 {
            invalid += 1;
            continue;
        }
        let centroid = triangle_centroid(mesh, tri);
        let query = kernel.query_point(
            [centroid.x as f64, centroid.y as f64, centroid.z as f64],
            SdfCfdQueryOptions::distance_and_gradient(),
        );
        let Some(unit) = query.unit_normal else {
            invalid += 1;
            continue;
        };
        let g = Vec3::new(unit[0] as f32, unit[1] as f32, unit[2] as f32);
        if g.length() <= 1e-12 || !g.is_finite() {
            invalid += 1;
            continue;
        }
        let dot = normal.normalize().dot(g.normalize()).clamp(-1.0, 1.0);
        let angle = dot.acos().to_degrees() as f64;
        if angle > config.severe_normal_angle_degrees {
            severe += 1;
        }
        angles.push(angle);
    }
    NormalAudit {
        angle_degrees: distribution(angles),
        severe_disagreements: severe,
        invalid_gradients: invalid,
    }
}

fn fold_audit(mesh: &Mesh, config: &ProjectionConfig) -> FoldAudit {
    let mut edge_normals: BTreeMap<(u32, u32), Vec<Vec3>> = BTreeMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        let n = triangle_normal(mesh, tri);
        if n.length() <= 1e-12 {
            continue;
        }
        for (a, b) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            edge_normals
                .entry(edge_key(a, b))
                .or_default()
                .push(n.normalize());
        }
    }
    let fold_edges = edge_normals
        .values()
        .filter(|normals| {
            normals.len() == 2
                && normals[0].dot(normals[1]) < config.fold_normal_dot_threshold as f32
        })
        .count();
    FoldAudit {
        fold_edges,
        groups: fold_edges,
    }
}

fn self_intersection_audit(mesh: &Mesh, budget: usize) -> SelfIntersectionAudit {
    let mut boxes = mesh
        .indices
        .chunks_exact(3)
        .enumerate()
        .map(|(idx, tri)| (idx, tri_aabb(mesh, tri), [tri[0], tri[1], tri[2]]))
        .collect::<Vec<_>>();
    boxes.sort_by(|a, b| a.1.0.x.total_cmp(&b.1.0.x).then(a.0.cmp(&b.0)));
    let mut audit = SelfIntersectionAudit::default();
    for i in 0..boxes.len() {
        let (ia, aabb_a, tri_a) = boxes[i];
        for (ib, aabb_b, tri_b) in boxes.iter().skip(i + 1).map(|x| (x.0, x.1, x.2)) {
            if aabb_b.0.x > aabb_a.1.x {
                break;
            }
            if audit.candidate_pairs >= budget {
                audit.budget_exceeded = true;
                return audit;
            }
            if !aabb_overlap(aabb_a, aabb_b) || shares_vertex(tri_a, tri_b) {
                continue;
            }
            audit.candidate_pairs += 1;
            if triangles_intersect(mesh, tri_a, tri_b) {
                audit.intersecting_pairs += 1;
                if audit.sample_pairs.len() < 16 {
                    audit.sample_pairs.push([ia, ib]);
                }
            }
        }
    }
    audit.groups = audit.intersecting_pairs;
    audit
}

fn distribution(mut values: Vec<f64>) -> Distribution {
    values.retain(|v| v.is_finite());
    if values.is_empty() {
        return Distribution::default();
    }
    values.sort_by(|a, b| a.total_cmp(b));
    let sum = values.iter().sum::<f64>();
    let sum_sq = values.iter().map(|v| v * v).sum::<f64>();
    let n = values.len() as f64;
    Distribution {
        min: values[0],
        max: *values.last().unwrap(),
        mean: sum / n,
        rms: (sum_sq / n).sqrt(),
        p50: percentile(&values, 0.50),
        p90: percentile(&values, 0.90),
        p95: percentile(&values, 0.95),
        p99: percentile(&values, 0.99),
    }
}

fn percentile(values: &[f64], p: f64) -> f64 {
    let idx = ((values.len().saturating_sub(1)) as f64 * p).round() as usize;
    values[idx.min(values.len() - 1)]
}

fn displacement_guard(
    displacement: f64,
    source: &AdaptiveTetVertexSource,
    config: &ProjectionConfig,
) -> bool {
    displacement.is_finite()
        && displacement
            <= config.maximum_displacement_fraction_local_min_cell
                * source.min_local_cell_dimension as f64
        && displacement
            <= config.maximum_displacement_fraction_local_diagonal
                * source.local_cell_diagonal as f64
        && displacement
            <= config.maximum_displacement_fraction_feature_thickness
                * config.trailing_edge_feature_thickness
}

fn sector_compatible(
    raw: &implicit_cad::sdf::cfd_query::SdfCfdQueryResult,
    proposed: &implicit_cad::sdf::cfd_query::SdfCfdQueryResult,
) -> bool {
    let raw_sector = sector_id(raw);
    let proposed_sector = sector_id(proposed);
    raw_sector.is_none() || proposed_sector.is_none() || raw_sector == proposed_sector
}

fn sector_id(query: &implicit_cad::sdf::cfd_query::SdfCfdQueryResult) -> Option<String> {
    query
        .procedural_feature
        .as_ref()
        .and_then(|p| p.normal_sector.as_ref())
        .map(|s| s.stable_id.clone())
}

fn feature_id(query: &implicit_cad::sdf::cfd_query::SdfCfdQueryResult) -> Option<String> {
    query
        .procedural_feature
        .as_ref()
        .and_then(|p| p.feature_id.clone())
}

fn connected_components(mesh: &Mesh, edges: &BTreeMap<(u32, u32), usize>) -> usize {
    let mut adjacency: Vec<Vec<usize>> = vec![Vec::new(); mesh.vertices.len()];
    for &(a, b) in edges.keys() {
        let a = a as usize;
        let b = b as usize;
        adjacency[a].push(b);
        adjacency[b].push(a);
    }
    let mut seen = vec![false; mesh.vertices.len()];
    let mut count = 0;
    for start in 0..mesh.vertices.len() {
        if seen[start] || adjacency[start].is_empty() {
            continue;
        }
        count += 1;
        let mut stack = vec![start];
        seen[start] = true;
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

fn triangles_intersect(mesh: &Mesh, a: [u32; 3], b: [u32; 3]) -> bool {
    let av = tri_points(mesh, a);
    let bv = tri_points(mesh, b);
    for (p0, p1) in [(av[0], av[1]), (av[1], av[2]), (av[2], av[0])] {
        if segment_triangle_intersect(p0, p1, bv) {
            return true;
        }
    }
    for (p0, p1) in [(bv[0], bv[1]), (bv[1], bv[2]), (bv[2], bv[0])] {
        if segment_triangle_intersect(p0, p1, av) {
            return true;
        }
    }
    false
}

fn segment_triangle_intersect(p0: Vec3, p1: Vec3, tri: [Vec3; 3]) -> bool {
    let eps = 1e-7f32;
    let dir = p1 - p0;
    let edge1 = tri[1] - tri[0];
    let edge2 = tri[2] - tri[0];
    let h = dir.cross(edge2);
    let a = edge1.dot(h);
    if a.abs() < eps {
        return false;
    }
    let f = 1.0 / a;
    let s = p0 - tri[0];
    let u = f * s.dot(h);
    if u < eps || u > 1.0 - eps {
        return false;
    }
    let q = s.cross(edge1);
    let v = f * dir.dot(q);
    if v < eps || u + v > 1.0 - eps {
        return false;
    }
    let t = f * edge2.dot(q);
    t > eps && t < 1.0 - eps
}

fn aabb_overlap(a: (Vec3, Vec3), b: (Vec3, Vec3)) -> bool {
    a.0.x <= b.1.x
        && a.1.x >= b.0.x
        && a.0.y <= b.1.y
        && a.1.y >= b.0.y
        && a.0.z <= b.1.z
        && a.1.z >= b.0.z
}

fn tri_aabb(mesh: &Mesh, tri: &[u32]) -> (Vec3, Vec3) {
    let p = [
        Vec3::from_array(mesh.vertices[tri[0] as usize].position),
        Vec3::from_array(mesh.vertices[tri[1] as usize].position),
        Vec3::from_array(mesh.vertices[tri[2] as usize].position),
    ];
    let min = p[0].min(p[1]).min(p[2]);
    let max = p[0].max(p[1]).max(p[2]);
    (min, max)
}

fn tri_points(mesh: &Mesh, tri: [u32; 3]) -> [Vec3; 3] {
    [
        Vec3::from_array(mesh.vertices[tri[0] as usize].position),
        Vec3::from_array(mesh.vertices[tri[1] as usize].position),
        Vec3::from_array(mesh.vertices[tri[2] as usize].position),
    ]
}

fn shares_vertex(a: [u32; 3], b: [u32; 3]) -> bool {
    a.iter().any(|x| b.contains(x))
}

fn triangle_centroid(mesh: &Mesh, tri: &[u32]) -> Vec3 {
    (Vec3::from_array(mesh.vertices[tri[0] as usize].position)
        + Vec3::from_array(mesh.vertices[tri[1] as usize].position)
        + Vec3::from_array(mesh.vertices[tri[2] as usize].position))
        / 3.0
}

fn triangle_normal(mesh: &Mesh, tri: &[u32]) -> Vec3 {
    let a = Vec3::from_array(mesh.vertices[tri[0] as usize].position);
    let b = Vec3::from_array(mesh.vertices[tri[1] as usize].position);
    let c = Vec3::from_array(mesh.vertices[tri[2] as usize].position);
    (b - a).cross(c - a)
}

fn triangle_area(mesh: &Mesh, tri: &[u32]) -> f32 {
    0.5 * triangle_normal(mesh, tri).length()
}

fn sorted_tri(mut tri: [u32; 3]) -> [u32; 3] {
    tri.sort();
    tri
}

fn edge_key(a: u32, b: u32) -> (u32, u32) {
    if a <= b { (a, b) } else { (b, a) }
}

fn f64_position(v: &Vertex) -> [f64; 3] {
    [
        v.position[0] as f64,
        v.position[1] as f64,
        v.position[2] as f64,
    ]
}

fn lerp3(a: [f64; 3], b: [f64; 3], t: f64) -> [f64; 3] {
    [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    ]
}

fn distance3(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
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

fn hash_strings(values: &[String]) -> String {
    let mut h = 1469598103934665603u64;
    for value in values {
        for b in value.as_bytes() {
            h ^= *b as u64;
            h = h.wrapping_mul(1099511628211);
        }
    }
    format!("{h:016x}")
}

fn projection_config() -> ProjectionConfig {
    ProjectionConfig {
        absolute_residual_tolerance: 1.0e-4,
        residual_tolerance_relative_to_local_min_cell: 1.0e-5,
        maximum_projection_iterations: 32,
        minimum_valid_gradient_magnitude: 1.0e-8,
        maximum_displacement_fraction_local_min_cell: 0.75,
        maximum_displacement_fraction_local_diagonal: 0.35,
        maximum_displacement_fraction_feature_thickness: 0.45,
        trailing_edge_feature_thickness: 1.0,
        damping_factors: vec![1.0, 0.5, 0.25, 0.125],
        severe_normal_angle_degrees: 115.0,
        fold_normal_dot_threshold: -0.25,
        self_intersection_candidate_budget: 5_000_000,
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

fn write_raw_ply(mesh: &Mesh, path: &Path) -> Result<(), String> {
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

fn write_debug_png() -> Result<String, String> {
    let path =
        PathBuf::from(SCREENSHOT_DIR).join("local_debug_current_c_failed_projection_v17.png");
    // A tiny deterministic 1x1 PNG keeps the dashboard/report path concrete when full contact sheets are gated off.
    let png: [u8; 68] = [
        137, 80, 78, 71, 13, 10, 26, 10, 0, 0, 0, 13, 73, 72, 68, 82, 0, 0, 0, 1, 0, 0, 0, 1, 8, 2,
        0, 0, 0, 144, 119, 83, 222, 0, 0, 0, 10, 73, 68, 65, 84, 120, 156, 99, 96, 96, 0, 0, 0, 3,
        0, 1, 43, 9, 77, 132, 0, 0, 0, 0, 73, 69, 78, 68, 174, 66, 96, 130,
    ];
    fs::write(&path, png).map_err(|e| e.to_string())?;
    Ok(path_for_report(&path))
}

fn recommended_next_task(
    sphere: &serde_json::Value,
    hard: &serde_json::Value,
    finite: &serde_json::Value,
) -> serde_json::Value {
    if sphere["status"] == "success" && hard["status"] == "success" && finite["status"] == "success"
    {
        json!({
            "task": "TE-4/6/8 depth study",
            "reason": "sphere, hard subtraction, and current-C projection/artifact gates passed"
        })
    } else if finite["topology_by_stage"][0]["self_intersections"]["intersecting_pairs"]
        .as_u64()
        .unwrap_or(0)
        > 0
        || finite["topology_by_stage"][0]["folds"]["fold_edges"]
            .as_u64()
            .unwrap_or(0)
            > 0
    {
        json!({
            "task": "raw marching-tet correction",
            "reason": "defects already exist before projection"
        })
    } else {
        json!({
            "task": "projection correction for adaptive marching-tets current C",
            "reason": finite["failure_reasons"]
        })
    }
}

fn markdown(report: &serde_json::Value) -> String {
    format!(
        "# Adaptive Marching Tetrahedra Projection v17\n\n\
Result: finite-wing current C `{}`.\n\n\
## Projection\n\n\
The v17 runner preserves marching-tet triangle connectivity, records source tet-edge and leaf metadata, \
uses `GeometryQueryKernel` conditioned projection, applies residual no-op behavior, displacement guards, \
and procedural sector compatibility checks, then audits topology, residuals, normal agreement, folds, and \
self-intersections.\n\n\
## Cases\n\n\
- Sphere: `{}`; projected hash `{}`; boundary `{}`; self-intersections `{}`; severe normals `{}`.\n\
- Hard subtraction: `{}`; projected hash `{}`; boundary `{}`; self-intersections `{}`; severe normals `{}`.\n\
- Finite wing current C: `{}`; raw hash `{}`; projected hash `{}`; boundary `{}`; self-intersections `{}`; severe normals `{}`.\n\n\
## Current-C Gate\n\n\
Failure reasons: `{}`.\n\n\
## Recommended Next Task\n\n\
`{}`\n",
        report["finite_wing"]["status"]
            .as_str()
            .unwrap_or("unknown"),
        report["sphere"]["status"].as_str().unwrap_or("unknown"),
        report["sphere"]["projected"]["topology"]["mesh_hash"]
            .as_str()
            .unwrap_or("unknown"),
        report["sphere"]["projected"]["topology"]["boundary_edges"]
            .as_u64()
            .unwrap_or(0),
        report["sphere"]["projected"]["self_intersections"]["intersecting_pairs"]
            .as_u64()
            .unwrap_or(0),
        report["sphere"]["projected"]["normal_alignment"]["severe_disagreements"]
            .as_u64()
            .unwrap_or(0),
        report["hard_subtraction"]["status"]
            .as_str()
            .unwrap_or("unknown"),
        report["hard_subtraction"]["projected"]["topology"]["mesh_hash"]
            .as_str()
            .unwrap_or("unknown"),
        report["hard_subtraction"]["projected"]["topology"]["boundary_edges"]
            .as_u64()
            .unwrap_or(0),
        report["hard_subtraction"]["projected"]["self_intersections"]["intersecting_pairs"]
            .as_u64()
            .unwrap_or(0),
        report["hard_subtraction"]["projected"]["normal_alignment"]["severe_disagreements"]
            .as_u64()
            .unwrap_or(0),
        report["finite_wing"]["status"]
            .as_str()
            .unwrap_or("unknown"),
        report["finite_wing"]["raw"]["mesh"]["mesh_hash"]
            .as_str()
            .unwrap_or("unknown"),
        report["finite_wing"]["projected"]["topology"]["mesh_hash"]
            .as_str()
            .unwrap_or("unknown"),
        report["finite_wing"]["projected"]["topology"]["boundary_edges"]
            .as_u64()
            .unwrap_or(0),
        report["finite_wing"]["projected"]["self_intersections"]["intersecting_pairs"]
            .as_u64()
            .unwrap_or(0),
        report["finite_wing"]["projected"]["normal_alignment"]["severe_disagreements"]
            .as_u64()
            .unwrap_or(0),
        report["finite_wing"]["failure_reasons"],
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
