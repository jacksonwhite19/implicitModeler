use crate::mesh::sdf_surface::{
    SdfSurfaceMesh, SurfaceEdgeClassification, SurfaceExtractionConfig, SurfaceMeshAudit,
    SurfaceQueryCounts, SurfaceVertex, audit_surface_mesh, rebuild_projected_surface_mesh,
};
use crate::sdf::Sdf;
use crate::sdf::cfd_query::{GeometryQueryKernel, SdfCfdQueryOptions};
use crate::sdf::provenance::CsgTraceMode;
use glam::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::sync::Arc;
use std::time::Instant;

pub const SDF_SURFACE_REMESH_SCHEMA_VERSION: &str = "sdf_surface_remesh.v3";
pub const SDF_SURFACE_REMESH_VERSION: &str = "isotropic_transactional_remesh.v1";

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct SurfaceRemeshConfig {
    pub target_edge_length: f64,
    pub min_edge_length_factor: f64,
    pub max_edge_length_factor: f64,
    pub split_threshold_factor: f64,
    pub collapse_threshold_factor: f64,
    pub max_iterations: usize,
    pub max_operations_per_iteration: usize,
    pub allow_edge_flips: bool,
    pub smoothing_iterations_per_pass: usize,
    pub smoothing_relaxation: f64,
    pub projection_tolerance: f64,
    pub projection_max_iterations: usize,
    pub maximum_projection_displacement_factor: f64,
    pub minimum_gradient_magnitude: f64,
    pub minimum_triangle_angle_deg: f64,
    pub maximum_aspect_ratio: f64,
    pub maximum_normal_deviation_deg: f64,
    pub preserve_candidate_feature_edges: bool,
    pub preserve_provenance_boundaries: bool,
    pub preserve_patch_boundaries: bool,
    pub convergence_tolerance: f64,
}

impl Default for SurfaceRemeshConfig {
    fn default() -> Self {
        Self {
            target_edge_length: 2.0,
            min_edge_length_factor: 0.45,
            max_edge_length_factor: 1.75,
            split_threshold_factor: 1.6,
            collapse_threshold_factor: 0.55,
            max_iterations: 3,
            max_operations_per_iteration: 500,
            allow_edge_flips: true,
            smoothing_iterations_per_pass: 1,
            smoothing_relaxation: 0.35,
            projection_tolerance: 1.0e-4,
            projection_max_iterations: 32,
            maximum_projection_displacement_factor: 1.5,
            minimum_gradient_magnitude: 1.0e-8,
            minimum_triangle_angle_deg: 0.25,
            maximum_aspect_ratio: 250.0,
            maximum_normal_deviation_deg: 85.0,
            preserve_candidate_feature_edges: true,
            preserve_provenance_boundaries: true,
            preserve_patch_boundaries: true,
            convergence_tolerance: 1.0e-3,
        }
    }
}

impl SurfaceRemeshConfig {
    pub fn to_surface_extraction_config(&self) -> SurfaceExtractionConfig {
        SurfaceExtractionConfig {
            target_spacing: self.target_edge_length,
            min_spacing: self.target_edge_length * self.min_edge_length_factor,
            max_spacing: self.target_edge_length * self.max_edge_length_factor,
            resolution_override: Some(0),
            bounds_padding: self.target_edge_length,
            weld_tolerance: 1.0e-7,
            projection_tolerance: self.projection_tolerance,
            projection_max_iterations: self.projection_max_iterations,
            max_projection_displacement_factor: self.maximum_projection_displacement_factor,
            min_gradient_magnitude: self.minimum_gradient_magnitude,
            require_closed: true,
            require_manifold: true,
            sharp_dihedral_degrees: 45.0,
        }
    }
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct RemeshOperationStats {
    pub attempted: usize,
    pub accepted: usize,
    pub rejected: usize,
    pub rejection_reasons: BTreeMap<String, usize>,
    pub average_quality_delta: f64,
    pub max_projection_displacement: f64,
    pub provenance_constraints: usize,
}

impl RemeshOperationStats {
    fn reject(&mut self, reason: impl Into<String>) {
        self.rejected += 1;
        *self.rejection_reasons.entry(reason.into()).or_default() += 1;
    }

    fn accept(&mut self, quality_delta: f64, projection_displacement: f64) {
        self.accepted += 1;
        let n = self.accepted as f64;
        self.average_quality_delta = ((self.average_quality_delta * (n - 1.0)) + quality_delta) / n;
        self.max_projection_displacement = self
            .max_projection_displacement
            .max(projection_displacement);
    }

    fn merge(&mut self, other: &RemeshOperationStats) {
        let accepted_before = self.accepted;
        self.attempted += other.attempted;
        self.rejected += other.rejected;
        self.accepted += other.accepted;
        for (reason, count) in &other.rejection_reasons {
            *self.rejection_reasons.entry(reason.clone()).or_default() += count;
        }
        if self.accepted > 0 {
            self.average_quality_delta = if accepted_before == 0 {
                other.average_quality_delta
            } else if other.accepted == 0 {
                self.average_quality_delta
            } else {
                ((self.average_quality_delta * accepted_before as f64)
                    + (other.average_quality_delta * other.accepted as f64))
                    / self.accepted as f64
            };
        }
        self.max_projection_displacement = self
            .max_projection_displacement
            .max(other.max_projection_displacement);
        self.provenance_constraints += other.provenance_constraints;
    }
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct RemeshOperationSummary {
    pub split: RemeshOperationStats,
    pub collapse: RemeshOperationStats,
    pub flip: RemeshOperationStats,
    pub smooth: RemeshOperationStats,
}

impl RemeshOperationSummary {
    fn total_accepted(&self) -> usize {
        self.split.accepted + self.collapse.accepted + self.flip.accepted + self.smooth.accepted
    }

    fn merge(&mut self, other: &RemeshOperationSummary) {
        self.split.merge(&other.split);
        self.collapse.merge(&other.collapse);
        self.flip.merge(&other.flip);
        self.smooth.merge(&other.smooth);
    }
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct RemeshTimings {
    pub editable_topology_ms: f64,
    pub sizing_field_ms: f64,
    pub split_ms: f64,
    pub collapse_ms: f64,
    pub flip_ms: f64,
    pub smoothing_ms: f64,
    pub projection_and_rebuild_ms: f64,
    pub validation_ms: f64,
    pub serialization_ms: f64,
    pub total_ms: f64,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct RemeshQualitySnapshot {
    pub vertex_count: usize,
    pub edge_count: usize,
    pub triangle_count: usize,
    pub edge_length_min: f64,
    pub edge_length_p05: f64,
    pub edge_length_p50: f64,
    pub edge_length_p95: f64,
    pub edge_length_max: f64,
    pub target_length_mean_abs_error: f64,
    pub target_length_p95_abs_error: f64,
    pub min_angle_min: f64,
    pub min_angle_p05: f64,
    pub min_angle_p50: f64,
    pub max_angle_p95: f64,
    pub max_angle_max: f64,
    pub aspect_ratio_p50: f64,
    pub aspect_ratio_p95: f64,
    pub aspect_ratio_max: f64,
    pub radius_ratio_p05: f64,
    pub radius_ratio_min: f64,
    pub normal_deviation_p95: f64,
    pub normal_deviation_max: f64,
    pub sliver_triangles: usize,
    pub area: f64,
    pub signed_volume: f64,
    pub aggregate_quality_score: f64,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct RemeshIterationReport {
    pub iteration: usize,
    pub operations: RemeshOperationSummary,
    pub before_hash: String,
    pub after_hash: String,
    pub before_quality: RemeshQualitySnapshot,
    pub after_quality: RemeshQualitySnapshot,
    pub topology: crate::mesh::sdf_surface::TopologyAudit,
    pub geometry: crate::mesh::sdf_surface::GeometryAudit,
    pub convergence_note: Option<String>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct SurfaceRemeshOutcome {
    pub schema_version: String,
    pub remesher_version: String,
    pub seed_mesh_hash: String,
    pub final_mesh_hash: String,
    pub seed: SdfSurfaceMesh,
    pub mesh: SdfSurfaceMesh,
    pub before_audit: SurfaceMeshAudit,
    pub after_audit: SurfaceMeshAudit,
    pub before_quality: RemeshQualitySnapshot,
    pub after_quality: RemeshQualitySnapshot,
    pub iterations: Vec<RemeshIterationReport>,
    pub operations: RemeshOperationSummary,
    pub timings: RemeshTimings,
    pub query_counts: SurfaceQueryCounts,
    pub convergence_reason: String,
    pub status: String,
    pub failure_reasons: Vec<String>,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct SurfaceQualityRegionSummary {
    pub target_edge_length: f64,
    pub poor_triangle_count: usize,
    pub total_sliver_count: usize,
    pub regions: Vec<SurfaceQualityRegion>,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct SurfaceQualityRegion {
    pub id: u64,
    pub location_classification: String,
    pub dominant_issue: String,
    pub triangle_count: usize,
    pub vertex_count: usize,
    pub area: f64,
    pub bbox_min: [f64; 3],
    pub bbox_max: [f64; 3],
    pub centroid: [f64; 3],
    pub min_angle_min: f64,
    pub min_angle_p05: f64,
    pub aspect_ratio_p95: f64,
    pub aspect_ratio_max: f64,
    pub radius_ratio_min: f64,
    pub shortest_edge_min: f64,
    pub longest_edge_max: f64,
    pub target_error_p95: f64,
    pub sliver_count: usize,
    pub protected_edge_count: usize,
    pub provenance_composition: BTreeMap<String, usize>,
}

#[derive(Clone, Debug)]
struct EditableVertex {
    position: Vec3,
    unit_normal: Option<Vec3>,
    provenance_key: Option<String>,
    protected_vertex: bool,
}

pub fn analyze_surface_quality_regions(
    mesh: &SdfSurfaceMesh,
    target_edge_length: f64,
) -> SurfaceQualityRegionSummary {
    let poor_triangles = mesh
        .triangles
        .iter()
        .filter(|tri| is_poor_quality_triangle(tri, target_edge_length))
        .map(|tri| tri.id)
        .collect::<BTreeSet<_>>();
    let mut tri_neighbors = BTreeMap::<u64, BTreeSet<u64>>::new();
    let mut protected_edges_by_tri = BTreeMap::<u64, usize>::new();
    for edge in &mesh.edges {
        let protected = !matches!(
            edge.classification,
            SurfaceEdgeClassification::ManifoldInterior
        );
        for tri_id in &edge.incident_triangles {
            if protected {
                *protected_edges_by_tri.entry(*tri_id).or_default() += 1;
            }
            for other in &edge.incident_triangles {
                if tri_id != other {
                    tri_neighbors.entry(*tri_id).or_default().insert(*other);
                }
            }
        }
    }

    let mut visited = BTreeSet::new();
    let mut regions = Vec::new();
    for start in &poor_triangles {
        if visited.contains(start) {
            continue;
        }
        let mut queue = VecDeque::from([*start]);
        let mut tri_ids = Vec::new();
        visited.insert(*start);
        while let Some(tri_id) = queue.pop_front() {
            tri_ids.push(tri_id);
            if let Some(neighbors) = tri_neighbors.get(&tri_id) {
                for neighbor in neighbors {
                    if poor_triangles.contains(neighbor) && visited.insert(*neighbor) {
                        queue.push_back(*neighbor);
                    }
                }
            }
        }
        tri_ids.sort_unstable();
        regions.push(build_quality_region(
            regions.len() as u64,
            mesh,
            &tri_ids,
            &protected_edges_by_tri,
            target_edge_length,
        ));
    }
    SurfaceQualityRegionSummary {
        target_edge_length,
        poor_triangle_count: poor_triangles.len(),
        total_sliver_count: mesh
            .triangles
            .iter()
            .filter(|tri| tri.quality.min_angle_degrees < 5.0 || tri.quality.aspect_ratio > 12.0)
            .count(),
        regions,
    }
}

#[derive(Clone, Debug)]
struct EditableMesh {
    vertices: Vec<EditableVertex>,
    triangles: Vec<[usize; 3]>,
    protected_edges: BTreeSet<(usize, usize)>,
    expected_components: usize,
}

#[derive(Clone, Debug)]
struct EdgeInfo {
    a: usize,
    b: usize,
    length: f64,
    protected: bool,
}

#[derive(Clone, Debug)]
struct ProjectionEdit {
    vertex: EditableVertex,
    displacement: f64,
}

pub fn remesh_sdf_surface(
    sdf: Arc<dyn Sdf>,
    seed: SdfSurfaceMesh,
    config: SurfaceRemeshConfig,
) -> Result<SurfaceRemeshOutcome, String> {
    let total_start = Instant::now();
    let mut timings = RemeshTimings::default();
    let mut total_query_counts = SurfaceQueryCounts::default();
    let before_audit = audit_surface_mesh(&seed);
    validate_seed_audit(&before_audit)?;
    let seed_hash = seed.mesh_hash();
    let before_quality = quality_snapshot(&seed, config.target_edge_length);

    let topo_start = Instant::now();
    let mut editable = EditableMesh::from_surface_mesh(&seed, &config)?;
    timings.editable_topology_ms = elapsed_ms(topo_start);
    let kernel = GeometryQueryKernel::from_sdf(Arc::clone(&sdf)).with_projection_settings(
        config.projection_tolerance as f32,
        config.projection_max_iterations,
    );

    let mut current_mesh = seed.clone();
    let mut current_audit = before_audit.clone();
    let mut current_quality = before_quality.clone();
    let mut seen_hashes = BTreeSet::from([seed_hash.clone()]);
    let mut iterations = Vec::new();
    let mut operations = RemeshOperationSummary::default();
    let mut convergence_reason = "maximum_iterations_reached".to_string();
    let mut failure_reasons = Vec::new();

    for iteration in 0..config.max_iterations {
        let before_hash = current_mesh.mesh_hash();
        let before_iter_quality = current_quality.clone();
        let mut iter_ops = RemeshOperationSummary::default();

        let split_start = Instant::now();
        iter_ops.split = split_pass(&mut editable, &kernel, &config);
        timings.split_ms += elapsed_ms(split_start);

        let collapse_start = Instant::now();
        iter_ops.collapse = collapse_pass(&mut editable, &kernel, &config);
        timings.collapse_ms += elapsed_ms(collapse_start);

        if config.allow_edge_flips {
            let flip_start = Instant::now();
            iter_ops.flip = flip_pass(&mut editable, &config);
            timings.flip_ms += elapsed_ms(flip_start);
        }

        let smooth_start = Instant::now();
        for _ in 0..config.smoothing_iterations_per_pass {
            let smooth_stats = smoothing_pass(&mut editable, &kernel, &config);
            iter_ops.smooth.merge(&smooth_stats);
        }
        timings.smoothing_ms += elapsed_ms(smooth_start);

        operations.merge(&iter_ops);
        let accepted_this_iteration = iter_ops.total_accepted();

        let rebuild_start = Instant::now();
        let (rebuilt, audit, counts) =
            finalize_editable_mesh(Arc::clone(&sdf), &editable, &config)?;
        timings.projection_and_rebuild_ms += elapsed_ms(rebuild_start);
        add_query_counts(&mut total_query_counts, &counts);

        let validation_start = Instant::now();
        validate_seed_audit(&audit)?;
        if audit.topology.connected_components != before_audit.topology.connected_components {
            return Err("remeshing changed connected-component count".to_string());
        }
        timings.validation_ms += elapsed_ms(validation_start);

        let after_hash = rebuilt.mesh_hash();
        let after_quality = quality_snapshot(&rebuilt, config.target_edge_length);
        let convergence_note = if accepted_this_iteration == 0 {
            Some("no_valid_edits_accepted".to_string())
        } else if sizing_band_satisfied(&after_quality, &config) {
            Some("all_edges_inside_sizing_band".to_string())
        } else if quality_improvement(&before_iter_quality, &after_quality).abs()
            < config.convergence_tolerance
        {
            Some("quality_improvement_below_tolerance".to_string())
        } else if seen_hashes.contains(&after_hash) {
            Some("repeated_mesh_hash_detected".to_string())
        } else {
            None
        };

        iterations.push(RemeshIterationReport {
            iteration,
            operations: iter_ops,
            before_hash,
            after_hash: after_hash.clone(),
            before_quality: before_iter_quality,
            after_quality: after_quality.clone(),
            topology: audit.topology.clone(),
            geometry: audit.geometry.clone(),
            convergence_note: convergence_note.clone(),
        });

        current_mesh = rebuilt;
        current_audit = audit;
        current_quality = after_quality;
        editable = EditableMesh::from_surface_mesh(&current_mesh, &config)?;

        if let Some(reason) = convergence_note {
            convergence_reason = reason;
            break;
        }
        seen_hashes.insert(after_hash);
    }

    if convergence_reason == "maximum_iterations_reached" && operations.total_accepted() > 0 {
        failure_reasons.push("iteration limit reached before configured convergence".to_string());
    }
    if !quality_materially_improved(&before_quality, &current_quality) {
        failure_reasons.push("quality distributions did not materially improve".to_string());
    }
    let status = if failure_reasons.is_empty() {
        "success".to_string()
    } else {
        "success_with_warnings".to_string()
    };
    timings.total_ms = elapsed_ms(total_start);

    Ok(SurfaceRemeshOutcome {
        schema_version: SDF_SURFACE_REMESH_SCHEMA_VERSION.to_string(),
        remesher_version: SDF_SURFACE_REMESH_VERSION.to_string(),
        seed_mesh_hash: seed_hash,
        final_mesh_hash: current_mesh.mesh_hash(),
        seed,
        mesh: current_mesh,
        before_audit,
        after_audit: current_audit,
        before_quality,
        after_quality: current_quality,
        iterations,
        operations,
        timings,
        query_counts: total_query_counts,
        convergence_reason,
        status,
        failure_reasons,
    })
}

impl EditableMesh {
    fn from_surface_mesh(
        mesh: &SdfSurfaceMesh,
        config: &SurfaceRemeshConfig,
    ) -> Result<Self, String> {
        let vertices = mesh
            .vertices
            .iter()
            .map(editable_vertex_from_surface)
            .collect::<Vec<_>>();
        let triangles = mesh
            .triangles
            .iter()
            .map(|tri| {
                [
                    tri.vertices[0] as usize,
                    tri.vertices[1] as usize,
                    tri.vertices[2] as usize,
                ]
            })
            .collect::<Vec<_>>();
        let mut protected_edges = BTreeSet::new();
        for edge in &mesh.edges {
            let protected = match edge.classification {
                SurfaceEdgeClassification::CandidateSharp => {
                    config.preserve_candidate_feature_edges
                }
                SurfaceEdgeClassification::CandidateProvenanceBoundary => {
                    config.preserve_provenance_boundaries
                }
                SurfaceEdgeClassification::CandidatePatchBoundary => {
                    config.preserve_patch_boundaries
                }
                _ => false,
            };
            if protected {
                protected_edges.insert(sorted_edge(
                    edge.vertices[0] as usize,
                    edge.vertices[1] as usize,
                ));
            }
        }
        let expected_components = mesh.audit_component_count();
        let editable = Self {
            vertices,
            triangles,
            protected_edges,
            expected_components,
        };
        validate_editable(&editable)?;
        Ok(editable)
    }

    fn edges(&self) -> BTreeMap<(usize, usize), Vec<usize>> {
        let mut edges = BTreeMap::<(usize, usize), Vec<usize>>::new();
        for (tri_idx, tri) in self.triangles.iter().enumerate() {
            for (a, b) in tri_edges(*tri) {
                edges.entry(sorted_edge(a, b)).or_default().push(tri_idx);
            }
        }
        edges
    }

    fn edge_infos(&self) -> Vec<EdgeInfo> {
        self.edges()
            .into_iter()
            .map(|((a, b), _incident)| EdgeInfo {
                a,
                b,
                length: self.vertices[a]
                    .position
                    .distance(self.vertices[b].position) as f64,
                protected: self.protected_edges.contains(&sorted_edge(a, b)),
            })
            .collect()
    }

    fn vertex_neighbors(&self) -> Vec<BTreeSet<usize>> {
        let mut neighbors = vec![BTreeSet::new(); self.vertices.len()];
        for tri in &self.triangles {
            for (a, b) in tri_edges(*tri) {
                neighbors[a].insert(b);
                neighbors[b].insert(a);
            }
        }
        neighbors
    }

    fn compact(&mut self) {
        let mut used = vec![false; self.vertices.len()];
        for tri in &self.triangles {
            for id in tri {
                used[*id] = true;
            }
        }
        let mut remap = vec![usize::MAX; self.vertices.len()];
        let mut vertices = Vec::new();
        for (old, is_used) in used.iter().copied().enumerate() {
            if is_used {
                remap[old] = vertices.len();
                vertices.push(self.vertices[old].clone());
            }
        }
        for tri in &mut self.triangles {
            for id in tri {
                *id = remap[*id];
            }
        }
        let mut protected = BTreeSet::new();
        for (a, b) in &self.protected_edges {
            if remap[*a] != usize::MAX && remap[*b] != usize::MAX && remap[*a] != remap[*b] {
                protected.insert(sorted_edge(remap[*a], remap[*b]));
            }
        }
        self.vertices = vertices;
        self.protected_edges = protected;
    }
}

trait MeshComponentCount {
    fn audit_component_count(&self) -> usize;
}

impl MeshComponentCount for SdfSurfaceMesh {
    fn audit_component_count(&self) -> usize {
        audit_surface_mesh(self).topology.connected_components
    }
}

fn split_pass(
    mesh: &mut EditableMesh,
    kernel: &GeometryQueryKernel,
    config: &SurfaceRemeshConfig,
) -> RemeshOperationStats {
    let mut stats = RemeshOperationStats::default();
    let mut rejected_edges = BTreeSet::new();
    loop {
        if stats.accepted >= config.max_operations_per_iteration
            || stats.attempted >= config.max_operations_per_iteration
        {
            break;
        }
        let mut candidates = mesh
            .edge_infos()
            .into_iter()
            .filter(|edge| edge.length > config.target_edge_length * config.split_threshold_factor)
            .filter(|edge| !rejected_edges.contains(&sorted_edge(edge.a, edge.b)))
            .collect::<Vec<_>>();
        candidates.sort_by(|a, b| {
            b.length
                .total_cmp(&a.length)
                .then_with(|| a.a.cmp(&b.a))
                .then_with(|| a.b.cmp(&b.b))
        });
        let Some(edge) = candidates.into_iter().next() else {
            break;
        };
        stats.attempted += 1;
        let quality_need = edge.length - config.target_edge_length * config.split_threshold_factor;
        match try_split_edge(mesh, kernel, config, edge.a, edge.b) {
            Ok(displacement) => {
                stats.accept(quality_need.max(0.0), displacement);
            }
            Err(reason) => {
                rejected_edges.insert(sorted_edge(edge.a, edge.b));
                stats.reject(reason);
            }
        }
    }
    stats
}

fn try_split_edge(
    mesh: &mut EditableMesh,
    kernel: &GeometryQueryKernel,
    config: &SurfaceRemeshConfig,
    a: usize,
    b: usize,
) -> Result<f64, String> {
    let key = sorted_edge(a, b);
    let edge_map = mesh.edges();
    let incident = edge_map
        .get(&key)
        .ok_or_else(|| "edge_missing".to_string())?;
    if incident.len() != 2 {
        return Err("split_requires_manifold_edge".to_string());
    }
    let midpoint = (mesh.vertices[a].position + mesh.vertices[b].position) * 0.5;
    let projected = project_edit_vertex(kernel, midpoint, config)?;
    let mut candidate = mesh.clone();
    let new_id = candidate.vertices.len();
    candidate.vertices.push(projected.vertex);
    let mut replacements = Vec::new();
    for tri_idx in incident {
        let tri = candidate.triangles[*tri_idx];
        replacements.extend(split_triangle(tri, a, b, new_id)?);
    }
    for (slot, tri_idx) in incident.iter().copied().enumerate() {
        candidate.triangles[tri_idx] = replacements[slot];
    }
    for tri in replacements.into_iter().skip(incident.len()) {
        candidate.triangles.push(tri);
    }
    if mesh.protected_edges.contains(&key) {
        candidate.protected_edges.remove(&key);
        candidate.protected_edges.insert(sorted_edge(a, new_id));
        candidate.protected_edges.insert(sorted_edge(new_id, b));
    }
    validate_editable(&candidate)?;
    *mesh = candidate;
    Ok(projected.displacement)
}

fn collapse_pass(
    mesh: &mut EditableMesh,
    kernel: &GeometryQueryKernel,
    config: &SurfaceRemeshConfig,
) -> RemeshOperationStats {
    let mut stats = RemeshOperationStats::default();
    let mut rejected_edges = BTreeSet::new();
    loop {
        if stats.accepted >= config.max_operations_per_iteration
            || stats.attempted >= config.max_operations_per_iteration
        {
            break;
        }
        let mut candidates = mesh
            .edge_infos()
            .into_iter()
            .filter(|edge| {
                edge.length < config.target_edge_length * config.collapse_threshold_factor
            })
            .filter(|edge| !rejected_edges.contains(&sorted_edge(edge.a, edge.b)))
            .collect::<Vec<_>>();
        candidates.sort_by(|a, b| {
            a.length
                .total_cmp(&b.length)
                .then_with(|| a.a.cmp(&b.a))
                .then_with(|| a.b.cmp(&b.b))
        });
        let Some(edge) = candidates.into_iter().next() else {
            break;
        };
        stats.attempted += 1;
        if edge.protected {
            stats.provenance_constraints += 1;
            rejected_edges.insert(sorted_edge(edge.a, edge.b));
            stats.reject("protected_edge_collapse_rejected");
            continue;
        }
        let quality_need =
            config.target_edge_length * config.collapse_threshold_factor - edge.length;
        match try_collapse_edge(mesh, kernel, config, edge.a, edge.b) {
            Ok(displacement) => {
                stats.accept(quality_need.max(0.0), displacement);
            }
            Err(reason) => {
                if reason.contains("provenance") {
                    stats.provenance_constraints += 1;
                }
                rejected_edges.insert(sorted_edge(edge.a, edge.b));
                stats.reject(reason);
            }
        }
    }
    stats
}

fn try_collapse_edge(
    mesh: &mut EditableMesh,
    kernel: &GeometryQueryKernel,
    config: &SurfaceRemeshConfig,
    a: usize,
    b: usize,
) -> Result<f64, String> {
    let key = sorted_edge(a, b);
    let edge_map = mesh.edges();
    let incident = edge_map
        .get(&key)
        .ok_or_else(|| "edge_missing".to_string())?;
    if incident.len() != 2 {
        return Err("collapse_requires_manifold_edge".to_string());
    }
    if incompatible_provenance(&mesh.vertices[a], &mesh.vertices[b]) {
        return Err("incompatible_provenance_collapse_rejected".to_string());
    }
    if !link_condition(mesh, a, b, incident) {
        return Err("link_condition_rejected".to_string());
    }

    let mut best: Option<(EditableMesh, f64, f64)> = None;
    let midpoint = (mesh.vertices[a].position + mesh.vertices[b].position) * 0.5;
    let projected_midpoint = project_edit_vertex(kernel, midpoint, config)?;
    let candidate_positions = [
        (projected_midpoint.vertex, projected_midpoint.displacement),
        (mesh.vertices[a].clone(), 0.0),
        (mesh.vertices[b].clone(), 0.0),
    ];
    for (target_vertex, displacement) in candidate_positions {
        let mut candidate = mesh.clone();
        candidate.vertices[a] = target_vertex;
        let mut new_triangles = Vec::with_capacity(candidate.triangles.len());
        for tri in &candidate.triangles {
            if tri.contains(&a) && tri.contains(&b) {
                continue;
            }
            let mut next = *tri;
            for id in &mut next {
                if *id == b {
                    *id = a;
                }
            }
            if next[0] == next[1] || next[1] == next[2] || next[2] == next[0] {
                continue;
            }
            new_triangles.push(next);
        }
        candidate.triangles = new_triangles;
        candidate.protected_edges.remove(&key);
        candidate.compact();
        if validate_editable(&candidate).is_err() {
            continue;
        }
        let score = editable_quality_score(&candidate, config.target_edge_length);
        if best
            .as_ref()
            .is_none_or(|(_, best_score, _)| score < *best_score)
        {
            best = Some((candidate, score, displacement));
        }
    }
    let Some((candidate, _, displacement)) = best else {
        return Err("no_valid_collapse_target".to_string());
    };
    *mesh = candidate;
    Ok(displacement)
}

fn flip_pass(mesh: &mut EditableMesh, config: &SurfaceRemeshConfig) -> RemeshOperationStats {
    let mut stats = RemeshOperationStats::default();
    let mut candidates = mesh.edge_infos();
    candidates.sort_by(|a, b| a.a.cmp(&b.a).then_with(|| a.b.cmp(&b.b)));
    for edge in candidates {
        if stats.accepted >= config.max_operations_per_iteration
            || stats.attempted >= config.max_operations_per_iteration
        {
            break;
        }
        stats.attempted += 1;
        if edge.protected {
            stats.reject("protected_edge_flip_rejected");
            continue;
        }
        let before_score = editable_quality_score(mesh, config.target_edge_length);
        match try_flip_edge(mesh, edge.a, edge.b, config) {
            Ok(()) => {
                let after_score = editable_quality_score(mesh, config.target_edge_length);
                stats.accept(before_score - after_score, 0.0);
            }
            Err(reason) => stats.reject(reason),
        }
    }
    stats
}

fn try_flip_edge(
    mesh: &mut EditableMesh,
    a: usize,
    b: usize,
    config: &SurfaceRemeshConfig,
) -> Result<(), String> {
    let key = sorted_edge(a, b);
    let edge_map = mesh.edges();
    let incident = edge_map
        .get(&key)
        .ok_or_else(|| "edge_missing".to_string())?;
    if incident.len() != 2 {
        return Err("flip_requires_two_incident_triangles".to_string());
    }
    let t0 = mesh.triangles[incident[0]];
    let t1 = mesh.triangles[incident[1]];
    let c = opposite_vertex(t0, a, b).ok_or_else(|| "flip_missing_opposite".to_string())?;
    let d = opposite_vertex(t1, a, b).ok_or_else(|| "flip_missing_opposite".to_string())?;
    if c == d || mesh.edges().contains_key(&sorted_edge(c, d)) {
        return Err("flip_duplicate_diagonal_rejected".to_string());
    }
    if incompatible_provenance(&mesh.vertices[c], &mesh.vertices[d]) {
        return Err("incompatible_provenance_flip_rejected".to_string());
    }
    let old_min_angle = local_min_angle(mesh, &[incident[0], incident[1]]);
    let old_normal = triangle_normal_positions(mesh, t0) + triangle_normal_positions(mesh, t1);
    let mut new_a = [c, d, b];
    let mut new_b = [d, c, a];
    let new_normal =
        triangle_normal_positions_for(mesh, new_a) + triangle_normal_positions_for(mesh, new_b);
    if old_normal.length_squared() > 0.0 && new_normal.dot(old_normal) < 0.0 {
        new_a.swap(1, 2);
        new_b.swap(1, 2);
    }
    let mut candidate = mesh.clone();
    candidate.triangles[incident[0]] = new_a;
    candidate.triangles[incident[1]] = new_b;
    validate_editable(&candidate)?;
    let new_min_angle = local_min_angle(&candidate, &[incident[0], incident[1]]);
    if new_min_angle <= old_min_angle + 1.0e-9 {
        return Err("flip_quality_not_improved".to_string());
    }
    let score_before = editable_quality_score(mesh, config.target_edge_length);
    let score_after = editable_quality_score(&candidate, config.target_edge_length);
    if score_after > score_before {
        return Err("flip_aggregate_quality_not_improved".to_string());
    }
    *mesh = candidate;
    Ok(())
}

fn smoothing_pass(
    mesh: &mut EditableMesh,
    kernel: &GeometryQueryKernel,
    config: &SurfaceRemeshConfig,
) -> RemeshOperationStats {
    let mut stats = RemeshOperationStats::default();
    let neighbors = mesh.vertex_neighbors();
    for vertex_id in 0..mesh.vertices.len() {
        if stats.accepted >= config.max_operations_per_iteration
            || stats.attempted >= config.max_operations_per_iteration
        {
            break;
        }
        stats.attempted += 1;
        if mesh.vertices[vertex_id].protected_vertex
            || incident_protected_edge(mesh, vertex_id)
            || neighbors[vertex_id].len() < 3
        {
            stats.reject("protected_or_low_valence_vertex_smoothing_rejected");
            continue;
        }
        let Some(normal) = mesh.vertices[vertex_id].unit_normal else {
            stats.reject("missing_sdf_normal_smoothing_rejected");
            continue;
        };
        let current = mesh.vertices[vertex_id].position;
        let average = neighbors[vertex_id]
            .iter()
            .map(|id| mesh.vertices[*id].position)
            .sum::<Vec3>()
            / neighbors[vertex_id].len() as f32;
        let delta = average - current;
        let tangent_delta = delta - normal * delta.dot(normal);
        if tangent_delta.length() < 1.0e-8 {
            stats.reject("smoothing_displacement_too_small");
            continue;
        }
        let candidate_position = current + tangent_delta * config.smoothing_relaxation as f32;
        let before_score = editable_quality_score(mesh, config.target_edge_length);
        match project_edit_vertex(kernel, candidate_position, config)
            .and_then(|projected| try_move_vertex(mesh, vertex_id, projected, config))
        {
            Ok(displacement) => {
                let after_score = editable_quality_score(mesh, config.target_edge_length);
                stats.accept(before_score - after_score, displacement);
            }
            Err(reason) => stats.reject(reason),
        }
    }
    stats
}

fn try_move_vertex(
    mesh: &mut EditableMesh,
    vertex_id: usize,
    projected: ProjectionEdit,
    config: &SurfaceRemeshConfig,
) -> Result<f64, String> {
    let mut candidate = mesh.clone();
    candidate.vertices[vertex_id] = projected.vertex;
    validate_editable(&candidate)?;
    let before = editable_quality_score(mesh, config.target_edge_length);
    let after = editable_quality_score(&candidate, config.target_edge_length);
    if after > before * 1.02 {
        return Err("smoothing_quality_regression_rejected".to_string());
    }
    *mesh = candidate;
    Ok(projected.displacement)
}

fn finalize_editable_mesh(
    sdf: Arc<dyn Sdf>,
    mesh: &EditableMesh,
    config: &SurfaceRemeshConfig,
) -> Result<(SdfSurfaceMesh, SurfaceMeshAudit, SurfaceQueryCounts), String> {
    validate_editable(mesh)?;
    let positions = mesh.vertices.iter().map(|v| v.position).collect::<Vec<_>>();
    let triangles = mesh
        .triangles
        .iter()
        .map(|tri| [tri[0] as u64, tri[1] as u64, tri[2] as u64])
        .collect::<Vec<_>>();
    let extraction_config = config.to_surface_extraction_config();
    rebuild_projected_surface_mesh(
        sdf,
        &positions,
        &triangles,
        &extraction_config,
        SDF_SURFACE_REMESH_VERSION,
    )
}

fn project_edit_vertex(
    kernel: &GeometryQueryKernel,
    position: Vec3,
    config: &SurfaceRemeshConfig,
) -> Result<ProjectionEdit, String> {
    let options = SdfCfdQueryOptions {
        distance: true,
        gradient: true,
        projection: true,
        provenance: true,
        hard_csg_provenance: CsgTraceMode::Recursive,
        feature_size: true,
        ..SdfCfdQueryOptions::distance_only()
    };
    let final_options = SdfCfdQueryOptions {
        distance: true,
        gradient: true,
        projection: false,
        provenance: true,
        hard_csg_provenance: CsgTraceMode::Recursive,
        feature_size: true,
        ..SdfCfdQueryOptions::distance_only()
    };
    let query = kernel.query_point(array(position), options);
    let projection = query
        .projection
        .ok_or_else(|| "projection_missing".to_string())?;
    if !projection.converged {
        return Err("projection_non_convergence".to_string());
    }
    let projected = projection
        .projected_point
        .ok_or_else(|| "projection_missing_point".to_string())?;
    let displacement = projection.displacement_magnitude.unwrap_or(f64::INFINITY);
    if displacement > config.maximum_projection_displacement_factor * config.target_edge_length {
        return Err("projection_displacement_limit".to_string());
    }
    let final_query = kernel.query_point(projected, final_options);
    let gradient_magnitude = final_query.gradient_magnitude.unwrap_or(0.0);
    if gradient_magnitude <= config.minimum_gradient_magnitude {
        return Err("invalid_gradient_after_projection".to_string());
    }
    Ok(ProjectionEdit {
        vertex: EditableVertex {
            position: Vec3::new(
                projected[0] as f32,
                projected[1] as f32,
                projected[2] as f32,
            ),
            unit_normal: final_query
                .unit_normal
                .map(|n| Vec3::new(n[0] as f32, n[1] as f32, n[2] as f32)),
            provenance_key: final_query.hard_csg_trace.as_ref().map(vertex_trace_key),
            protected_vertex: false,
        },
        displacement,
    })
}

fn validate_seed_audit(audit: &SurfaceMeshAudit) -> Result<(), String> {
    if audit.topology.boundary_edges != 0 {
        return Err("seed_or_result_has_boundary_edges".to_string());
    }
    if audit.topology.non_manifold_edges != 0 {
        return Err("seed_or_result_has_non_manifold_edges".to_string());
    }
    if audit.topology.duplicate_triangles != 0 {
        return Err("seed_or_result_has_duplicate_triangles".to_string());
    }
    if audit.topology.zero_area_triangles != 0 {
        return Err("seed_or_result_has_zero_area_triangles".to_string());
    }
    if audit.geometry.projection_failures != 0 {
        return Err("seed_or_result_has_projection_failures".to_string());
    }
    Ok(())
}

fn validate_editable(mesh: &EditableMesh) -> Result<(), String> {
    if mesh.vertices.is_empty() || mesh.triangles.is_empty() {
        return Err("empty_editable_mesh".to_string());
    }
    let mut seen = BTreeSet::<[usize; 3]>::new();
    for tri in &mesh.triangles {
        if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
            return Err("degenerate_triangle_indices".to_string());
        }
        for id in tri {
            if *id >= mesh.vertices.len() {
                return Err("triangle_references_invalid_vertex".to_string());
            }
        }
        let area = triangle_area_positions(mesh, *tri);
        if area <= 1.0e-12 || !area.is_finite() {
            return Err("degenerate_triangle_area".to_string());
        }
        let mut key = *tri;
        key.sort_unstable();
        if !seen.insert(key) {
            return Err("duplicate_triangle".to_string());
        }
    }
    let edges = mesh.edges();
    if edges.values().any(|incident| incident.len() == 1) {
        return Err("boundary_edge_created".to_string());
    }
    if edges.values().any(|incident| incident.len() > 2) {
        return Err("non_manifold_edge_created".to_string());
    }
    let components = editable_component_count(mesh);
    if components != mesh.expected_components {
        return Err("connected_component_count_changed".to_string());
    }
    Ok(())
}

fn quality_snapshot(mesh: &SdfSurfaceMesh, target: f64) -> RemeshQualitySnapshot {
    let mut edge_lengths = mesh
        .edges
        .iter()
        .map(|edge| {
            let a = surface_position(mesh, edge.vertices[0] as usize);
            let b = surface_position(mesh, edge.vertices[1] as usize);
            a.distance(b) as f64
        })
        .collect::<Vec<_>>();
    edge_lengths.sort_by(f64::total_cmp);
    let mut min_angles = mesh
        .triangles
        .iter()
        .map(|tri| tri.quality.min_angle_degrees)
        .collect::<Vec<_>>();
    min_angles.sort_by(f64::total_cmp);
    let mut max_angles = mesh
        .triangles
        .iter()
        .map(|tri| tri.quality.max_angle_degrees)
        .collect::<Vec<_>>();
    max_angles.sort_by(f64::total_cmp);
    let mut aspects = mesh
        .triangles
        .iter()
        .map(|tri| tri.quality.aspect_ratio)
        .collect::<Vec<_>>();
    aspects.sort_by(f64::total_cmp);
    let mut radius_ratios = mesh
        .triangles
        .iter()
        .map(|tri| tri.quality.radius_ratio)
        .collect::<Vec<_>>();
    radius_ratios.sort_by(f64::total_cmp);
    let mut normal_deviation = mesh
        .triangles
        .iter()
        .filter_map(|tri| tri.quality.normal_angle_to_sdf_degrees)
        .collect::<Vec<_>>();
    normal_deviation.sort_by(f64::total_cmp);
    let mut target_errors = edge_lengths
        .iter()
        .map(|len| (len - target).abs() / target.max(1.0e-12))
        .collect::<Vec<_>>();
    target_errors.sort_by(f64::total_cmp);
    let area = mesh.triangles.iter().map(|tri| tri.area).sum::<f64>();
    let slivers = mesh
        .triangles
        .iter()
        .filter(|tri| tri.quality.min_angle_degrees < 10.0 || tri.quality.aspect_ratio > 10.0)
        .count();
    RemeshQualitySnapshot {
        vertex_count: mesh.vertices.len(),
        edge_count: mesh.edges.len(),
        triangle_count: mesh.triangles.len(),
        edge_length_min: percentile(&edge_lengths, 0.0),
        edge_length_p05: percentile(&edge_lengths, 0.05),
        edge_length_p50: percentile(&edge_lengths, 0.50),
        edge_length_p95: percentile(&edge_lengths, 0.95),
        edge_length_max: percentile(&edge_lengths, 1.0),
        target_length_mean_abs_error: if target_errors.is_empty() {
            0.0
        } else {
            target_errors.iter().sum::<f64>() / target_errors.len() as f64
        },
        target_length_p95_abs_error: percentile(&target_errors, 0.95),
        min_angle_min: percentile(&min_angles, 0.0),
        min_angle_p05: percentile(&min_angles, 0.05),
        min_angle_p50: percentile(&min_angles, 0.50),
        max_angle_p95: percentile(&max_angles, 0.95),
        max_angle_max: percentile(&max_angles, 1.0),
        aspect_ratio_p50: percentile(&aspects, 0.50),
        aspect_ratio_p95: percentile(&aspects, 0.95),
        aspect_ratio_max: percentile(&aspects, 1.0),
        radius_ratio_p05: percentile(&radius_ratios, 0.05),
        radius_ratio_min: percentile(&radius_ratios, 0.0),
        normal_deviation_p95: percentile(&normal_deviation, 0.95),
        normal_deviation_max: percentile(&normal_deviation, 1.0),
        sliver_triangles: slivers,
        area,
        signed_volume: audit_surface_mesh(mesh).geometry.signed_volume,
        aggregate_quality_score: aggregate_quality_score(
            &edge_lengths,
            &min_angles,
            &aspects,
            &normal_deviation,
            target,
        ),
    }
}

fn is_poor_quality_triangle(
    tri: &crate::mesh::sdf_surface::SurfaceTriangle,
    target_edge_length: f64,
) -> bool {
    let edge_error = (tri.quality.max_edge_length - target_edge_length)
        .abs()
        .max((tri.quality.min_edge_length - target_edge_length).abs());
    tri.quality.min_angle_degrees < 5.0
        || tri.quality.aspect_ratio > 12.0
        || tri.quality.radius_ratio < 0.04
        || edge_error > target_edge_length * 0.9
}

fn build_quality_region(
    id: u64,
    mesh: &SdfSurfaceMesh,
    tri_ids: &[u64],
    protected_edges_by_tri: &BTreeMap<u64, usize>,
    target_edge_length: f64,
) -> SurfaceQualityRegion {
    let tri_by_id = mesh
        .triangles
        .iter()
        .map(|tri| (tri.id, tri))
        .collect::<BTreeMap<_, _>>();
    let mut vertices = BTreeSet::new();
    let mut min_angle = Vec::new();
    let mut aspects = Vec::new();
    let mut target_errors = Vec::new();
    let mut protected_edge_count = 0usize;
    let mut provenance_composition = BTreeMap::new();
    let mut area = 0.0;
    let mut sliver_count = 0usize;
    let mut radius_ratio_min = f64::INFINITY;
    let mut shortest_edge_min = f64::INFINITY;
    let mut longest_edge_max = 0.0_f64;
    let mut bbox_min = [f64::INFINITY; 3];
    let mut bbox_max = [f64::NEG_INFINITY; 3];
    let mut centroid_acc = [0.0; 3];
    for tri_id in tri_ids {
        let Some(tri) = tri_by_id.get(tri_id).copied() else {
            continue;
        };
        area += tri.area;
        min_angle.push(tri.quality.min_angle_degrees);
        aspects.push(tri.quality.aspect_ratio);
        target_errors.push((tri.quality.max_edge_length - target_edge_length).abs());
        protected_edge_count += protected_edges_by_tri.get(tri_id).copied().unwrap_or(0);
        radius_ratio_min = radius_ratio_min.min(tri.quality.radius_ratio);
        shortest_edge_min = shortest_edge_min.min(tri.quality.min_edge_length);
        longest_edge_max = longest_edge_max.max(tri.quality.max_edge_length);
        if tri.quality.min_angle_degrees < 5.0 || tri.quality.aspect_ratio > 12.0 {
            sliver_count += 1;
        }
        *provenance_composition
            .entry(format!("{:?}", tri.provenance_summary))
            .or_insert(0) += 1;
        for vertex_id in tri.vertices {
            vertices.insert(vertex_id);
            let p = mesh.vertices[vertex_id as usize].position;
            for axis in 0..3 {
                bbox_min[axis] = bbox_min[axis].min(p[axis]);
                bbox_max[axis] = bbox_max[axis].max(p[axis]);
                centroid_acc[axis] += p[axis] * tri.area / 3.0;
            }
        }
    }
    min_angle.sort_by(f64::total_cmp);
    aspects.sort_by(f64::total_cmp);
    target_errors.sort_by(f64::total_cmp);
    let centroid = if area > 0.0 {
        [
            centroid_acc[0] / area,
            centroid_acc[1] / area,
            centroid_acc[2] / area,
        ]
    } else {
        [
            (bbox_min[0] + bbox_max[0]) * 0.5,
            (bbox_min[1] + bbox_max[1]) * 0.5,
            (bbox_min[2] + bbox_max[2]) * 0.5,
        ]
    };
    let dominant_issue = if percentile(&aspects, 0.95) > 12.0 {
        "high_aspect_ratio"
    } else if percentile(&min_angle, 0.05) < 5.0 {
        "low_minimum_angle"
    } else if percentile(&target_errors, 0.95) > target_edge_length * 0.9 {
        "target_size_error"
    } else {
        "mixed_quality"
    };
    SurfaceQualityRegion {
        id,
        location_classification: classify_aircraft_location(mesh, centroid, bbox_min, bbox_max),
        dominant_issue: dominant_issue.to_string(),
        triangle_count: tri_ids.len(),
        vertex_count: vertices.len(),
        area,
        bbox_min,
        bbox_max,
        centroid,
        min_angle_min: min_angle.first().copied().unwrap_or(0.0),
        min_angle_p05: percentile(&min_angle, 0.05),
        aspect_ratio_p95: percentile(&aspects, 0.95),
        aspect_ratio_max: aspects.last().copied().unwrap_or(0.0),
        radius_ratio_min: if radius_ratio_min.is_finite() {
            radius_ratio_min
        } else {
            0.0
        },
        shortest_edge_min: if shortest_edge_min.is_finite() {
            shortest_edge_min
        } else {
            0.0
        },
        longest_edge_max,
        target_error_p95: percentile(&target_errors, 0.95),
        sliver_count,
        protected_edge_count,
        provenance_composition,
    }
}

fn classify_aircraft_location(
    mesh: &SdfSurfaceMesh,
    centroid: [f64; 3],
    bbox_min: [f64; 3],
    bbox_max: [f64; 3],
) -> String {
    let global_min = mesh.metadata.bounds_min;
    let global_max = mesh.metadata.bounds_max;
    let span = (global_max[1] - global_min[1]).abs().max(1.0);
    let chord = (global_max[0] - global_min[0]).abs().max(1.0);
    let thickness = (global_max[2] - global_min[2]).abs().max(1.0);
    let mut tags = Vec::new();
    if centroid[1] <= global_min[1] + 0.15 * span {
        tags.push("root");
    }
    if centroid[1] >= global_max[1] - 0.15 * span {
        tags.push("wing_tip");
    }
    if centroid[0] <= global_min[0] + 0.18 * chord {
        tags.push("leading_edge");
    }
    if centroid[0] >= global_max[0] - 0.18 * chord {
        tags.push("trailing_edge");
    }
    if bbox_max[2] <= global_min[2] + 0.45 * thickness {
        tags.push("lower_surface");
    } else if bbox_min[2] >= global_max[2] - 0.45 * thickness {
        tags.push("upper_surface");
    }
    if tags.is_empty() {
        "ordinary_or_unknown_surface".to_string()
    } else {
        tags.join("+")
    }
}

fn editable_quality_score(mesh: &EditableMesh, target: f64) -> f64 {
    let mut score = 0.0;
    for edge in mesh.edge_infos() {
        let e = (edge.length - target).abs() / target.max(1.0e-12);
        score += e * e;
    }
    for tri in &mesh.triangles {
        let angles = triangle_angles_positions(mesh, *tri);
        let min_angle = angles.iter().copied().fold(f64::INFINITY, f64::min);
        if min_angle < 30.0 {
            score += (30.0 - min_angle).powi(2) * 0.01;
        }
        let aspect = triangle_aspect_positions(mesh, *tri);
        if aspect > 3.0 {
            score += (aspect - 3.0).powi(2) * 0.1;
        }
    }
    score
}

fn aggregate_quality_score(
    edge_lengths: &[f64],
    min_angles: &[f64],
    aspects: &[f64],
    normal_deviation: &[f64],
    target: f64,
) -> f64 {
    let edge_penalty = if edge_lengths.is_empty() {
        0.0
    } else {
        edge_lengths
            .iter()
            .map(|len| ((len - target) / target.max(1.0e-12)).powi(2))
            .sum::<f64>()
            / edge_lengths.len() as f64
    };
    let angle_penalty = if min_angles.is_empty() {
        0.0
    } else {
        min_angles
            .iter()
            .map(|angle| (30.0 - angle).max(0.0).powi(2))
            .sum::<f64>()
            / min_angles.len() as f64
    };
    let aspect_penalty = if aspects.is_empty() {
        0.0
    } else {
        aspects
            .iter()
            .map(|aspect| (aspect - 3.0).max(0.0).powi(2))
            .sum::<f64>()
            / aspects.len() as f64
    };
    let normal_penalty = if normal_deviation.is_empty() {
        0.0
    } else {
        normal_deviation
            .iter()
            .map(|angle| (angle - 20.0).max(0.0).powi(2))
            .sum::<f64>()
            / normal_deviation.len() as f64
    };
    edge_penalty + 0.01 * angle_penalty + 0.1 * aspect_penalty + 0.01 * normal_penalty
}

fn sizing_band_satisfied(snapshot: &RemeshQualitySnapshot, config: &SurfaceRemeshConfig) -> bool {
    snapshot.edge_length_max <= config.target_edge_length * config.max_edge_length_factor
        && snapshot.edge_length_min >= config.target_edge_length * config.min_edge_length_factor
}

fn quality_improvement(before: &RemeshQualitySnapshot, after: &RemeshQualitySnapshot) -> f64 {
    before.aggregate_quality_score - after.aggregate_quality_score
}

fn quality_materially_improved(
    before: &RemeshQualitySnapshot,
    after: &RemeshQualitySnapshot,
) -> bool {
    after.edge_length_p95 <= before.edge_length_p95
        && after.aspect_ratio_p95 <= before.aspect_ratio_p95
        && after.sliver_triangles <= before.sliver_triangles
        && after.aggregate_quality_score < before.aggregate_quality_score
}

fn split_triangle(
    tri: [usize; 3],
    a: usize,
    b: usize,
    new_id: usize,
) -> Result<Vec<[usize; 3]>, String> {
    let Some(c) = opposite_vertex(tri, a, b) else {
        return Err("split_triangle_missing_edge".to_string());
    };
    if directed_edge_present(tri, a, b) {
        Ok(vec![[a, new_id, c], [new_id, b, c]])
    } else if directed_edge_present(tri, b, a) {
        Ok(vec![[b, new_id, c], [new_id, a, c]])
    } else {
        Err("split_triangle_missing_directed_edge".to_string())
    }
}

fn link_condition(mesh: &EditableMesh, a: usize, b: usize, incident: &[usize]) -> bool {
    let neighbors = mesh.vertex_neighbors();
    let common = neighbors[a]
        .intersection(&neighbors[b])
        .copied()
        .filter(|id| *id != a && *id != b)
        .collect::<BTreeSet<_>>();
    let opposite = incident
        .iter()
        .filter_map(|tri_idx| opposite_vertex(mesh.triangles[*tri_idx], a, b))
        .collect::<BTreeSet<_>>();
    common == opposite && common.len() == 2
}

fn incompatible_provenance(a: &EditableVertex, b: &EditableVertex) -> bool {
    match (&a.provenance_key, &b.provenance_key) {
        (Some(a), Some(b)) => a != b,
        _ => false,
    }
}

fn incident_protected_edge(mesh: &EditableMesh, vertex: usize) -> bool {
    mesh.protected_edges
        .iter()
        .any(|(a, b)| *a == vertex || *b == vertex)
}

fn editable_vertex_from_surface(vertex: &SurfaceVertex) -> EditableVertex {
    EditableVertex {
        position: Vec3::new(
            vertex.position[0] as f32,
            vertex.position[1] as f32,
            vertex.position[2] as f32,
        ),
        unit_normal: vertex
            .unit_normal
            .map(|n| Vec3::new(n[0] as f32, n[1] as f32, n[2] as f32)),
        provenance_key: vertex.provenance.as_ref().map(vertex_trace_key),
        protected_vertex: vertex.feature_flags.candidate_sharp
            || vertex.feature_flags.provenance_boundary
            || vertex.feature_flags.patch_boundary,
    }
}

fn vertex_trace_key(trace: &crate::sdf::provenance::CsgProvenanceTrace) -> String {
    if let Some(decision) = trace.decisions.last() {
        format!(
            "{:?}:{:?}:{:?}:{}",
            decision.operation,
            decision.active.child_role,
            decision.active.entity_kind,
            decision.active.effective_sign
        )
    } else {
        format!("{:?}:{:?}", trace.support_status, trace.termination_reason)
    }
}

fn editable_component_count(mesh: &EditableMesh) -> usize {
    let mut tri_adj = vec![Vec::<usize>::new(); mesh.triangles.len()];
    for incident in mesh.edges().values() {
        for &a in incident {
            for &b in incident {
                if a != b {
                    tri_adj[a].push(b);
                }
            }
        }
    }
    let mut visited = vec![false; mesh.triangles.len()];
    let mut components = 0usize;
    for start in 0..mesh.triangles.len() {
        if visited[start] {
            continue;
        }
        components += 1;
        let mut queue = VecDeque::from([start]);
        visited[start] = true;
        while let Some(cur) = queue.pop_front() {
            for next in &tri_adj[cur] {
                if !visited[*next] {
                    visited[*next] = true;
                    queue.push_back(*next);
                }
            }
        }
    }
    components
}

fn add_query_counts(total: &mut SurfaceQueryCounts, next: &SurfaceQueryCounts) {
    total.distance_queries += next.distance_queries;
    total.gradient_queries += next.gradient_queries;
    total.projection_queries += next.projection_queries;
    total.provenance_queries += next.provenance_queries;
}

fn surface_position(mesh: &SdfSurfaceMesh, id: usize) -> Vec3 {
    let p = mesh.vertices[id].position;
    Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)
}

fn sorted_edge(a: usize, b: usize) -> (usize, usize) {
    if a <= b { (a, b) } else { (b, a) }
}

fn tri_edges(tri: [usize; 3]) -> [(usize, usize); 3] {
    [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])]
}

fn opposite_vertex(tri: [usize; 3], a: usize, b: usize) -> Option<usize> {
    tri.into_iter().find(|id| *id != a && *id != b)
}

fn directed_edge_present(tri: [usize; 3], a: usize, b: usize) -> bool {
    tri_edges(tri).iter().any(|edge| edge.0 == a && edge.1 == b)
}

fn triangle_area_positions(mesh: &EditableMesh, tri: [usize; 3]) -> f64 {
    let a = mesh.vertices[tri[0]].position;
    let b = mesh.vertices[tri[1]].position;
    let c = mesh.vertices[tri[2]].position;
    0.5 * (b - a).cross(c - a).length() as f64
}

fn triangle_normal_positions(mesh: &EditableMesh, tri: [usize; 3]) -> Vec3 {
    triangle_normal_positions_for(mesh, tri)
}

fn triangle_normal_positions_for(mesh: &EditableMesh, tri: [usize; 3]) -> Vec3 {
    let a = mesh.vertices[tri[0]].position;
    let b = mesh.vertices[tri[1]].position;
    let c = mesh.vertices[tri[2]].position;
    (b - a).cross(c - a)
}

fn local_min_angle(mesh: &EditableMesh, tri_ids: &[usize]) -> f64 {
    tri_ids
        .iter()
        .flat_map(|id| triangle_angles_positions(mesh, mesh.triangles[*id]))
        .fold(f64::INFINITY, f64::min)
}

fn triangle_angles_positions(mesh: &EditableMesh, tri: [usize; 3]) -> [f64; 3] {
    let a = mesh.vertices[tri[0]].position;
    let b = mesh.vertices[tri[1]].position;
    let c = mesh.vertices[tri[2]].position;
    let ab = a.distance(b) as f64;
    let bc = b.distance(c) as f64;
    let ca = c.distance(a) as f64;
    [
        law_of_cosines_angle(ab, ca, bc),
        law_of_cosines_angle(ab, bc, ca),
        law_of_cosines_angle(bc, ca, ab),
    ]
}

fn triangle_aspect_positions(mesh: &EditableMesh, tri: [usize; 3]) -> f64 {
    let a = mesh.vertices[tri[0]].position;
    let b = mesh.vertices[tri[1]].position;
    let c = mesh.vertices[tri[2]].position;
    let ab = a.distance(b) as f64;
    let bc = b.distance(c) as f64;
    let ca = c.distance(a) as f64;
    let longest = ab.max(bc).max(ca);
    let area = triangle_area_positions(mesh, tri);
    let altitude = if longest > 1.0e-12 {
        2.0 * area / longest
    } else {
        0.0
    };
    if altitude > 1.0e-12 {
        longest / altitude
    } else {
        f64::INFINITY
    }
}

fn law_of_cosines_angle(a: f64, b: f64, opposite: f64) -> f64 {
    if a <= 0.0 || b <= 0.0 {
        return 0.0;
    }
    ((a * a + b * b - opposite * opposite) / (2.0 * a * b))
        .clamp(-1.0, 1.0)
        .acos()
        .to_degrees()
}

fn percentile(values: &[f64], percentile: f64) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let idx = ((values.len() - 1) as f64 * percentile.clamp(0.0, 1.0)).round() as usize;
    values[idx]
}

fn array(v: Vec3) -> [f64; 3] {
    [v.x as f64, v.y as f64, v.z as f64]
}

fn elapsed_ms(start: Instant) -> f64 {
    start.elapsed().as_secs_f64() * 1000.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::sdf_surface::{SurfaceExtractionConfig, extract_sdf_surface_mesh};
    use crate::sdf::booleans::Subtract;
    use crate::sdf::primitives::Sphere;
    use crate::sdf::transforms::Translate;

    #[test]
    fn split_preserves_closed_sphere_topology() {
        let sdf: Arc<dyn Sdf> = Arc::new(Sphere::new(8.0));
        let seed = extract_sdf_surface_mesh(
            Arc::clone(&sdf),
            SurfaceExtractionConfig {
                target_spacing: 4.0,
                projection_tolerance: 1.0e-3,
                ..SurfaceExtractionConfig::default().with_resolution(14)
            },
        )
        .unwrap()
        .mesh;
        let config = SurfaceRemeshConfig {
            target_edge_length: 1.8,
            split_threshold_factor: 1.25,
            max_iterations: 1,
            max_operations_per_iteration: 25,
            smoothing_iterations_per_pass: 0,
            ..SurfaceRemeshConfig::default()
        };
        let outcome = remesh_sdf_surface(sdf, seed, config).unwrap();
        assert_eq!(outcome.after_audit.topology.boundary_edges, 0);
        assert_eq!(outcome.after_audit.topology.non_manifold_edges, 0);
        assert!(outcome.operations.split.accepted > 0);
    }

    #[test]
    fn collapse_and_flip_do_not_cross_subtraction_provenance() {
        let body: Arc<dyn Sdf> = Arc::new(Sphere::new(12.0));
        let tool: Arc<dyn Sdf> = Arc::new(Translate::new(
            Arc::new(Sphere::new(4.0)),
            Vec3::new(3.0, 0.0, 0.0),
        ));
        let sdf: Arc<dyn Sdf> = Arc::new(Subtract::new(body, tool));
        let seed = extract_sdf_surface_mesh(
            Arc::clone(&sdf),
            SurfaceExtractionConfig {
                target_spacing: 4.0,
                projection_tolerance: 1.0e-3,
                ..SurfaceExtractionConfig::default().with_resolution(20)
            },
        )
        .unwrap()
        .mesh;
        let config = SurfaceRemeshConfig {
            target_edge_length: 3.2,
            max_iterations: 1,
            max_operations_per_iteration: 40,
            ..SurfaceRemeshConfig::default()
        };
        let outcome = remesh_sdf_surface(sdf, seed, config).unwrap();
        assert_eq!(outcome.after_audit.topology.boundary_edges, 0);
        assert_eq!(outcome.after_audit.topology.non_manifold_edges, 0);
        assert!(outcome.after_audit.provenance.subtractor_controlled_area > 0.0);
    }

    #[test]
    fn remeshing_is_deterministic_for_sphere() {
        let sdf: Arc<dyn Sdf> = Arc::new(Sphere::new(8.0));
        let seed = extract_sdf_surface_mesh(
            Arc::clone(&sdf),
            SurfaceExtractionConfig {
                target_spacing: 4.0,
                projection_tolerance: 1.0e-3,
                ..SurfaceExtractionConfig::default().with_resolution(14)
            },
        )
        .unwrap()
        .mesh;
        let config = SurfaceRemeshConfig {
            target_edge_length: 3.0,
            max_iterations: 1,
            max_operations_per_iteration: 30,
            ..SurfaceRemeshConfig::default()
        };
        let a = remesh_sdf_surface(Arc::clone(&sdf), seed.clone(), config.clone()).unwrap();
        let b = remesh_sdf_surface(sdf, seed, config).unwrap();
        assert_eq!(a.final_mesh_hash, b.final_mesh_hash);
        assert_eq!(a.mesh, b.mesh);
    }
}
