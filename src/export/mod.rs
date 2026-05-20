// Mesh export functionality
pub mod adaptive_octree;
pub mod aero;
pub mod dual_contouring;

use crate::export::adaptive_octree::{
    AdaptiveOctreeSettings, MIN_FEATURE_SIZE_MM, build_adaptive_octree, summarize_octree,
};
use crate::export::dual_contouring::{
    DualContouringTelemetry, analyze_mesh_topology, boundary_edge_count,
    cap_boundary_edge_components, extract_dual_contour_mesh, extract_dual_contour_mesh_from_octree,
    extract_dual_contour_mesh_from_octree_with_telemetry, fill_boundary_loops,
    fill_undirected_boundary_cycles, log_mesh_component_diagnostics, prune_triangle_components,
    repair_non_manifold_edges_with_refill,
};
use crate::mesh::{Mesh, MeshQuality, marching_cubes};
use crate::sdf::Sdf;
use glam::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs::File;
use std::io::{self, Write};
use std::path::Path;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ExportBackend {
    UniformMarchingCubes,
    UniformDualContouring,
    AdaptiveDualContouring,
}

impl ExportBackend {
    pub fn label(self) -> &'static str {
        match self {
            Self::UniformMarchingCubes => "Uniform Marching Cubes",
            Self::UniformDualContouring => "Uniform Dual Contouring",
            Self::AdaptiveDualContouring => "Adaptive Dual Contouring (Production)",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum TargetResolution {
    ExactMm(f32),
}

impl TargetResolution {
    pub fn millimeters(self) -> f32 {
        match self {
            Self::ExactMm(value) => value,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ExportSettings {
    pub backend: ExportBackend,
    pub resolution: TargetResolution,
    pub bounding_box_padding: f32,
    #[serde(default)]
    pub surface_projection: SurfaceProjectionSettings,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SurfaceProjectionSettings {
    pub enabled: bool,
    pub iterations: u32,
    pub max_step_fraction: f32,
    pub convergence_epsilon_mm: f32,
    pub projection_band_fraction: f32,
}

impl Default for SurfaceProjectionSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            iterations: 3,
            max_step_fraction: 0.5,
            convergence_epsilon_mm: 1e-4,
            projection_band_fraction: 2.0,
        }
    }
}

impl Default for ExportSettings {
    fn default() -> Self {
        Self {
            backend: ExportBackend::AdaptiveDualContouring,
            resolution: TargetResolution::ExactMm(1.0),
            bounding_box_padding: 0.0,
            surface_projection: SurfaceProjectionSettings::default(),
        }
    }
}

impl ExportSettings {
    pub fn target_cell_mm(&self) -> f32 {
        self.resolution.millimeters().max(0.02)
    }

    pub fn calculate_max_depth(&self, domain_max_extent_mm: f32) -> u32 {
        let target_size_mm = self.target_cell_mm();
        if target_size_mm <= 0.0 || !target_size_mm.is_finite() {
            return 8;
        }

        let ratio = (domain_max_extent_mm / target_size_mm).max(1.0);
        ratio.log2().ceil().max(1.0) as u32
    }
}

#[derive(Clone, Debug)]
pub struct ExportMeshResult {
    pub mesh: Mesh,
    pub backend: ExportBackend,
    pub topology: Option<dual_contouring::MeshTopologyStats>,
}

pub fn build_export_mesh(
    sdf: &dyn Sdf,
    bounds_min: Vec3,
    bounds_max: Vec3,
    quality: MeshQuality,
    smooth_normals: bool,
) -> Mesh {
    build_export_mesh_with_target_cell(
        sdf,
        bounds_min,
        bounds_max,
        quality.target_cell_size_mm(),
        quality,
        smooth_normals,
    )
}

pub fn build_export_mesh_with_target_cell(
    sdf: &dyn Sdf,
    bounds_min: Vec3,
    bounds_max: Vec3,
    target_cell_mm: f32,
    _quality_hint: MeshQuality,
    smooth_normals: bool,
) -> Mesh {
    let target_cell = target_cell_mm.max(0.02);

    // Default export intentionally stays on the old uniform marching-cubes backend.
    // Dual Contouring is exposed separately so both paths can be compared while the
    // new backend is under construction.
    let fallback_target = target_cell;
    build_export_mesh_uniform_reference(
        sdf,
        bounds_min,
        bounds_max,
        fallback_target,
        smooth_normals,
    )
}

pub fn build_export_mesh_with_backend(
    sdf: &dyn Sdf,
    bounds_min: Vec3,
    bounds_max: Vec3,
    target_cell_mm: f32,
    backend: ExportBackend,
    smooth_normals: bool,
) -> ExportMeshResult {
    match backend {
        ExportBackend::UniformMarchingCubes => ExportMeshResult {
            mesh: build_export_mesh_uniform_reference(
                sdf,
                bounds_min,
                bounds_max,
                target_cell_mm,
                smooth_normals,
            ),
            backend,
            topology: None,
        },
        ExportBackend::UniformDualContouring => {
            let mesh = build_export_mesh_dual_contouring_uniform(
                sdf,
                bounds_min,
                bounds_max,
                target_cell_mm,
            );
            let topology = Some(analyze_mesh_topology(&mesh));
            ExportMeshResult {
                mesh,
                backend,
                topology,
            }
        }
        ExportBackend::AdaptiveDualContouring => {
            let mesh = build_export_mesh_dual_contouring_adaptive(
                sdf,
                bounds_min,
                bounds_max,
                target_cell_mm,
            );
            let topology = Some(analyze_mesh_topology(&mesh));
            ExportMeshResult {
                mesh,
                backend,
                topology,
            }
        }
    }
}

pub fn build_export_mesh_with_settings(
    sdf: &dyn Sdf,
    bounds_min: Vec3,
    bounds_max: Vec3,
    settings: &ExportSettings,
    smooth_normals: bool,
) -> ExportMeshResult {
    let pad = Vec3::splat(settings.bounding_box_padding.max(0.0));
    let mut result = build_export_mesh_with_backend(
        sdf,
        bounds_min - pad,
        bounds_max + pad,
        settings.target_cell_mm(),
        settings.backend,
        smooth_normals,
    );
    if settings.surface_projection.enabled {
        result.mesh = smooth_export_mesh_with_topology_guard(
            &result.mesh,
            sdf,
            settings.target_cell_mm(),
            settings.surface_projection,
        );
    }
    result
}

pub fn build_export_mesh_dual_contouring_uniform(
    sdf: &dyn Sdf,
    bounds_min: Vec3,
    bounds_max: Vec3,
    target_cell_mm: f32,
) -> Mesh {
    extract_dual_contour_mesh(sdf, bounds_min, bounds_max, target_cell_mm.max(0.02))
}

pub fn build_export_mesh_dual_contouring_adaptive(
    sdf: &dyn Sdf,
    bounds_min: Vec3,
    bounds_max: Vec3,
    target_cell_mm: f32,
) -> Mesh {
    let target_cell = target_cell_mm.max(0.05);
    let adaptive_min_cell = target_cell.min(MIN_FEATURE_SIZE_MM);
    let max_dim = (bounds_max - bounds_min).max_element().max(target_cell);
    let max_depth = ExportSettings {
        backend: ExportBackend::AdaptiveDualContouring,
        resolution: TargetResolution::ExactMm(adaptive_min_cell),
        bounding_box_padding: 0.0,
        ..Default::default()
    }
    .calculate_max_depth(max_dim);
    let octree = build_adaptive_octree(
        sdf,
        bounds_min,
        bounds_max,
        &AdaptiveOctreeSettings {
            max_depth: max_depth.clamp(1, 14),
            min_cell_size_mm: adaptive_min_cell,
            surface_band_mm: (target_cell * 2.0).max(1.0),
            error_tolerance_mm: target_cell * 0.5,
            curvature_refine_threshold: 0.12,
            feature_scale_factor: 2.5,
            max_leaf_cells: 3_000_000,
        },
    );
    extract_dual_contour_mesh_from_octree(sdf, &octree, target_cell)
}

pub fn build_export_mesh_dual_contouring_adaptive_with_telemetry(
    sdf: &dyn Sdf,
    bounds_min: Vec3,
    bounds_max: Vec3,
    target_cell_mm: f32,
) -> (Mesh, DualContouringTelemetry) {
    let target_cell = target_cell_mm.max(0.05);
    let adaptive_min_cell = target_cell.min(MIN_FEATURE_SIZE_MM);
    let max_dim = (bounds_max - bounds_min).max_element().max(target_cell);
    let max_depth = ExportSettings {
        backend: ExportBackend::AdaptiveDualContouring,
        resolution: TargetResolution::ExactMm(adaptive_min_cell),
        bounding_box_padding: 0.0,
        ..Default::default()
    }
    .calculate_max_depth(max_dim);
    let octree = build_adaptive_octree(
        sdf,
        bounds_min,
        bounds_max,
        &AdaptiveOctreeSettings {
            max_depth: max_depth.clamp(1, 14),
            min_cell_size_mm: adaptive_min_cell,
            surface_band_mm: (target_cell * 2.0).max(1.0),
            error_tolerance_mm: target_cell * 0.5,
            curvature_refine_threshold: 0.12,
            feature_scale_factor: 2.5,
            max_leaf_cells: 3_000_000,
        },
    );
    let (mesh, telemetry) =
        extract_dual_contour_mesh_from_octree_with_telemetry(sdf, &octree, target_cell);
    let topology = analyze_mesh_topology(&mesh);
    if let Ok(mut log) = std::fs::OpenOptions::new()
        .append(true)
        .create(true)
        .open("output.md")
    {
        let _ = writeln!(
            log,
            "\n## Export Safety Fallback\n- adaptive_dc_boundary_edges: {}\n- adaptive_dc_non_manifold_edges: {}\n- fallback: legacy_uniform_marching_cubes\n",
            topology.boundary_edges, topology.non_manifold_edges
        );
    }
    let mut fallback =
        build_export_mesh_uniform_reference(sdf, bounds_min, bounds_max, target_cell, false);
    log_mesh_component_diagnostics(&fallback, "fallback_raw");
    fill_boundary_loops(&mut fallback);
    fill_undirected_boundary_cycles(&mut fallback);
    fallback = repair_non_manifold_edges_with_refill(
        &fallback,
        target_cell.max(0.05) * target_cell.max(0.05) * 1e-8,
    );
    fallback = prune_triangle_components(&fallback, 1_000, 0.05, 1);
    log_mesh_component_diagnostics(&fallback, "fallback_pruned");
    fill_boundary_loops(&mut fallback);
    fill_undirected_boundary_cycles(&mut fallback);
    cap_boundary_edge_components(&mut fallback);
    fallback = repair_non_manifold_edges_with_refill(
        &fallback,
        target_cell.max(0.05) * target_cell.max(0.05) * 1e-8,
    );
    fallback = weld_mesh_vertices_local(&fallback, 1e-5);
    fallback = filter_degenerate_and_duplicate_triangles(&fallback);
    if let Ok(mut log) = std::fs::OpenOptions::new()
        .append(true)
        .create(true)
        .open("output.md")
    {
        let fallback_topology = analyze_mesh_topology(&fallback);
        let _ = writeln!(
            log,
            "- fallback_boundary_edges: {}\n- fallback_non_manifold_edges: {}\n",
            fallback_topology.boundary_edges, fallback_topology.non_manifold_edges
        );
    }
    (fallback, telemetry)
}

pub fn build_export_mesh_uniform_reference(
    sdf: &dyn Sdf,
    bounds_min: Vec3,
    bounds_max: Vec3,
    target_cell_mm: f32,
    smooth_normals: bool,
) -> Mesh {
    let target_cell = target_cell_mm.max(0.02);
    let cell_resolution =
        (((bounds_max - bounds_min).max_element() / target_cell).ceil() as u32).clamp(8, 512);
    let mesh =
        marching_cubes::extract_mesh(sdf, bounds_min, bounds_max, cell_resolution, smooth_normals);
    clean_export_mesh_topology(
        weld_mesh_vertices_local(&mesh, (target_cell * 0.05).max(0.02)),
        target_cell,
    )
}

pub fn clean_export_mesh_topology(mesh: Mesh, target_cell: f32) -> Mesh {
    let mut mesh = filter_degenerate_and_duplicate_triangles(&mesh);
    fill_boundary_loops(&mut mesh);
    fill_undirected_boundary_cycles(&mut mesh);
    cap_boundary_edge_components(&mut mesh);
    mesh = repair_non_manifold_edges_with_refill(
        &mesh,
        target_cell.max(0.05) * target_cell.max(0.05) * 1e-8,
    );
    fill_boundary_loops(&mut mesh);
    fill_undirected_boundary_cycles(&mut mesh);
    cap_boundary_edge_components(&mut mesh);
    let mut mesh = weld_mesh_vertices_local(
        &filter_degenerate_and_duplicate_triangles(&mesh),
        (target_cell * 0.005).clamp(1e-5, 0.02),
    );
    mesh = filter_degenerate_and_duplicate_triangles(&mesh);
    fill_boundary_loops(&mut mesh);
    fill_undirected_boundary_cycles(&mut mesh);
    cap_boundary_edge_components(&mut mesh);
    fill_triangular_boundary_cycles(&mut mesh);
    mesh = repair_non_manifold_edges_with_refill(
        &mesh,
        target_cell.max(0.05) * target_cell.max(0.05) * 1e-8,
    );
    let mut mesh = filter_degenerate_and_duplicate_triangles(&mesh);
    fill_triangular_boundary_cycles(&mut mesh);
    mesh
}

fn weld_mesh_vertices_local(mesh: &Mesh, tolerance_mm: f32) -> Mesh {
    if mesh.vertices.is_empty() {
        return mesh.clone();
    }

    let inv_tol = if tolerance_mm > 1e-8 {
        1.0 / tolerance_mm
    } else {
        1e6
    };
    let mut buckets: HashMap<(i32, i32, i32), Vec<usize>> = HashMap::new();
    let mut new_vertices: Vec<crate::mesh::Vertex> = Vec::new();
    let mut remap = vec![0u32; mesh.vertices.len()];

    for (old_idx, vertex) in mesh.vertices.iter().enumerate() {
        let p = Vec3::from_array(vertex.position);
        let key = (
            (p.x * inv_tol).round() as i32,
            (p.y * inv_tol).round() as i32,
            (p.z * inv_tol).round() as i32,
        );
        let mut mapped = None;
        if let Some(candidates) = buckets.get(&key) {
            for &candidate_idx in candidates {
                let c = Vec3::from_array(new_vertices[candidate_idx].position);
                if p.distance(c) <= tolerance_mm {
                    mapped = Some(candidate_idx as u32);
                    break;
                }
            }
        }
        let new_idx = if let Some(idx) = mapped {
            idx
        } else {
            let idx = new_vertices.len() as u32;
            new_vertices.push(*vertex);
            buckets.entry(key).or_default().push(idx as usize);
            idx
        };
        remap[old_idx] = new_idx;
    }

    let mut new_indices = mesh.indices.clone();
    for idx in &mut new_indices {
        *idx = remap[*idx as usize];
    }

    Mesh {
        vertices: new_vertices,
        indices: new_indices,
    }
}

fn filter_degenerate_and_duplicate_triangles(mesh: &Mesh) -> Mesh {
    let mut indices = Vec::with_capacity(mesh.indices.len());
    let mut seen = HashMap::<[u32; 3], ()>::new();

    for tri in mesh.indices.chunks_exact(3) {
        let mapped = [tri[0], tri[1], tri[2]];
        if mapped[0] == mapped[1] || mapped[1] == mapped[2] || mapped[2] == mapped[0] {
            continue;
        }

        let mut key = mapped;
        key.sort_unstable();
        if seen.insert(key, ()).is_none() {
            indices.extend_from_slice(&mapped);
        }
    }

    Mesh {
        vertices: mesh.vertices.clone(),
        indices,
    }
}

pub fn project_mesh_vertices_to_sdf(
    mesh: &Mesh,
    sdf: &dyn Sdf,
    target_cell_mm: f32,
    settings: SurfaceProjectionSettings,
) -> Mesh {
    if !settings.enabled || mesh.vertices.is_empty() {
        return mesh.clone();
    }

    let target_cell = target_cell_mm.max(0.02);
    let eps = (target_cell * 0.15).clamp(0.01, 0.25);
    let max_step = target_cell * settings.max_step_fraction.clamp(0.05, 2.0);
    let convergence = settings.convergence_epsilon_mm.max(1e-6);
    let projection_band = target_cell * settings.projection_band_fraction.max(0.25);
    let iterations = settings.iterations.clamp(1, 12);

    let mut projected = mesh.clone();
    for vertex in &mut projected.vertices {
        let mut p = Vec3::from_array(vertex.position);
        for _ in 0..iterations {
            let d = sdf.distance(p);
            if !d.is_finite() || d.abs() <= convergence {
                break;
            }
            if d.abs() > projection_band {
                break;
            }

            let gradient = Vec3::new(
                sdf.distance(p + Vec3::X * eps) - sdf.distance(p - Vec3::X * eps),
                sdf.distance(p + Vec3::Y * eps) - sdf.distance(p - Vec3::Y * eps),
                sdf.distance(p + Vec3::Z * eps) - sdf.distance(p - Vec3::Z * eps),
            ) / (2.0 * eps);
            let gradient_len = gradient.length();
            if !gradient_len.is_finite() || gradient_len <= 1e-6 {
                break;
            }

            let step = (-d).clamp(-max_step, max_step);
            p += gradient / gradient_len * step;
        }
        vertex.position = p.to_array();
    }

    projected
}

pub fn smooth_export_mesh_with_topology_guard(
    mesh: &Mesh,
    sdf: &dyn Sdf,
    target_cell_mm: f32,
    settings: SurfaceProjectionSettings,
) -> Mesh {
    if !settings.enabled {
        return mesh.clone();
    }

    let baseline_topology = analyze_mesh_topology(mesh);
    let projected = project_mesh_vertices_to_sdf(mesh, sdf, target_cell_mm, settings);
    let cleaned = clean_export_mesh_topology(projected, target_cell_mm);
    let smoothed_topology = analyze_mesh_topology(&cleaned);

    if topology_bad_edge_score(&smoothed_topology) <= topology_bad_edge_score(&baseline_topology) {
        cleaned
    } else {
        mesh.clone()
    }
}

fn topology_bad_edge_score(stats: &dual_contouring::MeshTopologyStats) -> usize {
    stats.boundary_edges + stats.non_manifold_edges * 4 + stats.degenerate_triangles * 8
}

#[derive(Clone, Copy, Debug)]
pub struct AdaptiveExportSettings {
    pub quality: MeshQuality,
    pub smooth_normals: bool,
    pub octree: AdaptiveOctreeSettings,
    pub prefer_dual_contouring: bool,
}

impl Default for AdaptiveExportSettings {
    fn default() -> Self {
        Self {
            quality: MeshQuality::Normal,
            smooth_normals: false,
            octree: AdaptiveOctreeSettings::default(),
            prefer_dual_contouring: false,
        }
    }
}

#[derive(Clone, Debug)]
pub struct AdaptiveExportResult {
    pub mesh: Mesh,
    pub backend: &'static str,
    pub reason: String,
}

pub fn build_export_mesh_adaptive(
    sdf: &dyn Sdf,
    bounds_min: Vec3,
    bounds_max: Vec3,
    settings: AdaptiveExportSettings,
) -> AdaptiveExportResult {
    let octree = build_adaptive_octree(sdf, bounds_min, bounds_max, &settings.octree);
    let summary = summarize_octree(&octree);
    if summary.surface_leaf_count == 0 {
        return AdaptiveExportResult {
            mesh: build_export_mesh(
                sdf,
                bounds_min,
                bounds_max,
                settings.quality,
                settings.smooth_normals,
            ),
            backend: "fallback_uniform",
            reason: "adaptive octree found no surface leaves".to_string(),
        };
    }

    let pad = Vec3::splat(
        settings
            .octree
            .surface_band_mm
            .max(settings.octree.min_cell_size_mm),
    );
    let tight_min = summary.surface_bounds_min - pad;
    let tight_max = summary.surface_bounds_max + pad;
    let target_cell = settings
        .quality
        .target_cell_size_mm()
        .max(settings.octree.min_cell_size_mm);
    if settings.prefer_dual_contouring {
        let mesh = extract_dual_contour_mesh_from_octree(sdf, &octree, target_cell);
        if !mesh.vertices.is_empty() && !mesh.indices.is_empty() && boundary_edge_count(&mesh) == 0
        {
            return AdaptiveExportResult {
                mesh,
                backend: "dual_contouring_octree",
                reason: format!(
                    "octree dual succeeded with {} surface leaves",
                    summary.surface_leaf_count
                ),
            };
        }
        let mesh = extract_dual_contour_mesh(sdf, tight_min, tight_max, target_cell);
        if !mesh.vertices.is_empty() && !mesh.indices.is_empty() && boundary_edge_count(&mesh) == 0
        {
            return AdaptiveExportResult {
                mesh,
                backend: "dual_contouring_uniform",
                reason: "uniform dual succeeded after octree dual fallback".to_string(),
            };
        }
    }

    AdaptiveExportResult {
        mesh: build_export_mesh_with_target_cell(
            sdf,
            tight_min,
            tight_max,
            target_cell,
            settings.quality,
            settings.smooth_normals,
        ),
        backend: "uniform_export_fallback",
        reason: format!(
            "dual contouring was non-watertight on adaptive domains; used uniform export fallback over {} surface leaves",
            summary.surface_leaf_count
        ),
    }
}

/// Export mesh to binary STL format
pub fn export_stl(mesh: &Mesh, path: &str) -> io::Result<()> {
    let welded;
    let mesh = if mesh.vertices.is_empty() {
        mesh
    } else {
        welded = weld_mesh_vertices_for_stl(mesh, 1e-5);
        &welded
    };
    let oriented;
    let mesh = if signed_mesh_volume(mesh) < 0.0 {
        oriented = flip_mesh_winding(mesh);
        &oriented
    } else {
        mesh
    };
    let mut file = File::create(path)?;

    // STL header (80 bytes)
    let header = [0u8; 80];
    file.write_all(&header)?;

    // Number of triangles (4 bytes, little-endian)
    let num_triangles = (mesh.indices.len() / 3) as u32;
    file.write_all(&num_triangles.to_le_bytes())?;

    // Write each triangle
    for i in (0..mesh.indices.len()).step_by(3) {
        let idx0 = mesh.indices[i] as usize;
        let idx1 = mesh.indices[i + 1] as usize;
        let idx2 = mesh.indices[i + 2] as usize;

        let v0 = &mesh.vertices[idx0];
        let v1 = &mesh.vertices[idx1];
        let v2 = &mesh.vertices[idx2];

        // Compute face normal from vertices
        let p0 = glam::Vec3::from_array(v0.position);
        let p1 = glam::Vec3::from_array(v1.position);
        let p2 = glam::Vec3::from_array(v2.position);

        let edge1 = p1 - p0;
        let edge2 = p2 - p0;
        let normal = edge1.cross(edge2).normalize_or_zero();

        // Write normal (12 bytes)
        file.write_all(&normal.x.to_le_bytes())?;
        file.write_all(&normal.y.to_le_bytes())?;
        file.write_all(&normal.z.to_le_bytes())?;

        // Write vertices (36 bytes total)
        for &vertex in &[v0, v1, v2] {
            file.write_all(&vertex.position[0].to_le_bytes())?;
            file.write_all(&vertex.position[1].to_le_bytes())?;
            file.write_all(&vertex.position[2].to_le_bytes())?;
        }

        // Attribute byte count (2 bytes, usually 0)
        file.write_all(&[0u8, 0u8])?;
    }

    Ok(())
}

fn signed_mesh_volume(mesh: &Mesh) -> f64 {
    let mut volume = 0.0f64;
    for tri in mesh.indices.chunks_exact(3) {
        let p0 = Vec3::from_array(mesh.vertices[tri[0] as usize].position).as_dvec3();
        let p1 = Vec3::from_array(mesh.vertices[tri[1] as usize].position).as_dvec3();
        let p2 = Vec3::from_array(mesh.vertices[tri[2] as usize].position).as_dvec3();
        volume += p0.dot(p1.cross(p2)) / 6.0;
    }
    volume
}

fn flip_mesh_winding(mesh: &Mesh) -> Mesh {
    let mut indices = mesh.indices.clone();
    for tri in indices.chunks_exact_mut(3) {
        tri.swap(1, 2);
    }
    Mesh {
        vertices: mesh.vertices.clone(),
        indices,
    }
}

fn weld_mesh_vertices_for_stl(mesh: &Mesh, tolerance_mm: f32) -> Mesh {
    let inv_tol = 1.0 / tolerance_mm.max(1e-8);
    let mut remap = vec![0u32; mesh.vertices.len()];
    let mut vertices = Vec::<crate::mesh::Vertex>::new();
    let mut grid = HashMap::<[i64; 3], u32>::new();
    for (idx, vertex) in mesh.vertices.iter().enumerate() {
        let key = [
            (vertex.position[0] * inv_tol).round() as i64,
            (vertex.position[1] * inv_tol).round() as i64,
            (vertex.position[2] * inv_tol).round() as i64,
        ];
        let mapped = *grid.entry(key).or_insert_with(|| {
            let new_idx = vertices.len() as u32;
            vertices.push(vertex.clone());
            new_idx
        });
        remap[idx] = mapped;
    }

    let mut indices = Vec::with_capacity(mesh.indices.len());
    let mut seen = HashMap::<[u32; 3], ()>::new();
    for tri in mesh.indices.chunks_exact(3) {
        let mapped = [
            remap[tri[0] as usize],
            remap[tri[1] as usize],
            remap[tri[2] as usize],
        ];
        if mapped[0] == mapped[1] || mapped[1] == mapped[2] || mapped[2] == mapped[0] {
            continue;
        }
        let mut key = mapped;
        key.sort_unstable();
        if seen.insert(key, ()).is_none() {
            indices.extend_from_slice(&mapped);
        }
    }

    Mesh { vertices, indices }
}

fn fill_triangular_boundary_cycles(mesh: &mut Mesh) {
    let mut edge_counts: HashMap<(u32, u32), usize> = HashMap::new();
    let mut oriented_edges = Vec::<(u32, u32)>::new();
    for tri in mesh.indices.chunks_exact(3) {
        for &(a, b) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            let key = if a < b { (a, b) } else { (b, a) };
            *edge_counts.entry(key).or_insert(0) += 1;
            oriented_edges.push((a, b));
        }
    }

    let boundary_oriented: Vec<(u32, u32)> = oriented_edges
        .into_iter()
        .filter(|&(a, b)| {
            let key = if a < b { (a, b) } else { (b, a) };
            edge_counts.get(&key).copied().unwrap_or(0) == 1
        })
        .collect();
    if boundary_oriented.is_empty() {
        return;
    }

    let mut next_from = HashMap::<u32, Vec<u32>>::new();
    for &(a, b) in &boundary_oriented {
        next_from.entry(a).or_default().push(b);
    }

    let mut emitted = HashMap::<[u32; 3], ()>::new();
    for &(a, b) in &boundary_oriented {
        let Some(candidates) = next_from.get(&b) else {
            continue;
        };
        let Some(&c) = candidates.iter().find(|&&candidate| {
            candidate != a && next_from.get(&candidate).is_some_and(|n| n.contains(&a))
        }) else {
            continue;
        };
        let mut key = [a, b, c];
        key.sort_unstable();
        if emitted.insert(key, ()).is_some() {
            continue;
        }
        mesh.indices.extend_from_slice(&[a, c, b]);
    }
}

/// Export mesh to OBJ format
pub fn export_obj(mesh: &Mesh, path: &str) -> io::Result<()> {
    let mut file = File::create(path)?;

    // Write header comment
    writeln!(file, "# Exported from Implicit CAD")?;
    writeln!(file, "# Vertices: {}", mesh.vertices.len())?;
    writeln!(file, "# Triangles: {}", mesh.indices.len() / 3)?;
    writeln!(file)?;

    // Write vertices
    for vertex in &mesh.vertices {
        writeln!(
            file,
            "v {} {} {}",
            vertex.position[0], vertex.position[1], vertex.position[2]
        )?;
    }

    writeln!(file)?;

    // Write vertex normals
    for vertex in &mesh.vertices {
        writeln!(
            file,
            "vn {} {} {}",
            vertex.normal[0], vertex.normal[1], vertex.normal[2]
        )?;
    }

    writeln!(file)?;

    // Write faces (OBJ uses 1-based indexing)
    for i in (0..mesh.indices.len()).step_by(3) {
        let idx0 = mesh.indices[i] + 1;
        let idx1 = mesh.indices[i + 1] + 1;
        let idx2 = mesh.indices[i + 2] + 1;

        // Format: f v1//vn1 v2//vn2 v3//vn3 (vertex//normal)
        writeln!(
            file,
            "f {}//{} {}//{} {}//{}",
            idx0, idx0, idx1, idx1, idx2, idx2
        )?;
    }

    Ok(())
}

pub fn export_by_format(mesh: &Mesh, path: &Path, format: &str) -> Result<(), String> {
    match format.to_ascii_lowercase().as_str() {
        "stl" => {
            export_stl(mesh, path.to_str().ok_or("invalid output path")?).map_err(|e| e.to_string())
        }
        "obj" => {
            export_obj(mesh, path.to_str().ok_or("invalid output path")?).map_err(|e| e.to_string())
        }
        other => Err(format!("Unknown format: {}. Use 'stl' or 'obj'.", other)),
    }
}

// ── Manufacturing package export ──────────────────────────────────────────────

/// A bundled manufacturing output: STL files, BOM, and assembly notes.
#[allow(dead_code)] // Part of manufacturing export API
pub struct ManufacturingPackage {
    pub stl_files: Vec<String>,
    pub bom_csv: String,
    pub assembly_notes: String,
}

/// Export a complete manufacturing package to `output_dir`.
#[allow(dead_code)] // Part of manufacturing export API — not yet called from UI
///
/// Creates:
/// - `main_body.stl` — binary STL mesh
/// - `bom.csv` — bill of materials (CSV)
/// - `bom.md` — bill of materials (Markdown)
/// - `assembly_notes.md` — print settings and assembly guidance
pub fn export_manufacturing_package(
    mesh: &Mesh,
    project_name: &str,
    output_dir: &Path,
) -> Result<ManufacturingPackage, String> {
    std::fs::create_dir_all(output_dir).map_err(|e| e.to_string())?;

    // Export main body STL
    let stl_path = output_dir.join("main_body.stl");
    export_stl(mesh, stl_path.to_str().ok_or("invalid path")?).map_err(|e| e.to_string())?;

    let triangle_count = mesh.indices.len() / 3;

    // BOM CSV
    let bom_csv = format!(
        "Part,Type,Quantity,Notes\nmain_body,Printed Part,1,{} triangles\n",
        triangle_count
    );
    let bom_path = output_dir.join("bom.csv");
    std::fs::write(&bom_path, &bom_csv).map_err(|e| e.to_string())?;

    // BOM Markdown
    let bom_md = format!(
        "# Bill of Materials\n\n| Part | Type | Qty | Notes |\n|------|------|-----|-------|\n| main_body | Printed Part | 1 | {} triangles |\n",
        triangle_count
    );
    let bom_md_path = output_dir.join("bom.md");
    std::fs::write(&bom_md_path, &bom_md).map_err(|e| e.to_string())?;

    // Assembly notes — use chrono for date
    let date_str = chrono::Local::now().format("%Y-%m-%d").to_string();
    let assembly_notes = format!(
        "# Assembly Notes\n\n**Project:** {}\n**Date:** {}\n**Triangles:** {}\n\n## Print Settings\n\n- Layer height: 0.2mm\n- Infill: 20%\n- Material: PLA\n",
        project_name, date_str, triangle_count
    );
    let notes_path = output_dir.join("assembly_notes.md");
    std::fs::write(&notes_path, &assembly_notes).map_err(|e| e.to_string())?;

    Ok(ManufacturingPackage {
        stl_files: vec!["main_body.stl".to_string()],
        bom_csv,
        assembly_notes,
    })
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use glam::Vec3;

    use super::{
        AdaptiveExportSettings, ExportBackend, ExportSettings, TargetResolution, build_export_mesh,
        build_export_mesh_adaptive, build_export_mesh_with_backend,
        build_export_mesh_with_settings,
    };
    use crate::mesh::MeshQuality;
    use crate::sdf::Sdf;
    use crate::sdf::aerospace::{AirfoilExportOptions, wing_with_airfoil_export_safe};
    use crate::sdf::booleans::Subtract;
    use crate::sdf::field::lattice::GyroidLattice;
    use crate::sdf::primitives::{SdfBox, Sphere};

    #[test]
    fn export_settings_calculate_depth_from_metric_resolution() {
        let settings = ExportSettings {
            backend: ExportBackend::AdaptiveDualContouring,
            resolution: TargetResolution::ExactMm(1.0),
            bounding_box_padding: 0.0,
        };
        assert_eq!(settings.calculate_max_depth(650.0), 10);

        let coarse = ExportSettings {
            resolution: TargetResolution::ExactMm(5.0),
            ..settings
        };
        assert_eq!(coarse.calculate_max_depth(650.0), 8);
    }

    #[test]
    fn export_settings_dispatch_to_selected_backend() {
        let sphere = Sphere::new(6.0);
        let settings = ExportSettings {
            backend: ExportBackend::UniformDualContouring,
            resolution: TargetResolution::ExactMm(1.5),
            bounding_box_padding: 0.5,
        };
        let result = build_export_mesh_with_settings(
            &sphere,
            Vec3::splat(-8.0),
            Vec3::splat(8.0),
            &settings,
            false,
        );
        assert_eq!(result.backend, ExportBackend::UniformDualContouring);
        assert!(!result.mesh.indices.is_empty());
    }

    #[test]
    fn default_export_mesh_uses_uniform_marching_cubes_backend() {
        let shape: Arc<dyn Sdf> = Arc::new(GyroidLattice::new(8.0, 1.0));
        let mesh = build_export_mesh(
            shape.as_ref(),
            Vec3::new(-12.0, -12.0, -12.0),
            Vec3::new(12.0, 12.0, 12.0),
            MeshQuality::Draft,
            false,
        );
        assert!(
            !mesh.vertices.is_empty(),
            "Unsupported trees should still produce export meshes"
        );
        assert_eq!(
            mesh.indices.len() % 3,
            0,
            "Export mesh output should remain triangle indexed"
        );
    }

    #[test]
    fn explicit_dual_contouring_backend_is_callable_for_comparison() {
        let shape: Arc<dyn Sdf> = Arc::new(Sphere::new(10.0));
        let result = build_export_mesh_with_backend(
            shape.as_ref(),
            Vec3::splat(-12.3),
            Vec3::splat(12.3),
            1.0,
            ExportBackend::UniformDualContouring,
            false,
        );
        assert_eq!(result.backend, ExportBackend::UniformDualContouring);
        assert!(!result.mesh.vertices.is_empty());
        assert_eq!(result.mesh.indices.len() % 3, 0);
        let topology = result.topology.expect("DC backend should report topology");
        assert_eq!(topology.boundary_edges, 0);
        assert_eq!(topology.non_manifold_edges, 0);
    }

    #[test]
    fn explicit_dual_contouring_backend_captures_thin_box_shell() {
        let outer: Arc<dyn Sdf> = Arc::new(SdfBox::new(Vec3::new(20.0, 32.5, 10.0)));
        let inner: Arc<dyn Sdf> = Arc::new(SdfBox::new(Vec3::new(18.0, 30.5, 8.0)));
        let shell = Subtract::new(outer, inner);
        let result = build_export_mesh_with_backend(
            &shell,
            Vec3::new(-22.3, -34.8, -12.3),
            Vec3::new(22.3, 34.8, 12.3),
            1.0,
            ExportBackend::UniformDualContouring,
            false,
        );
        assert!(!result.mesh.vertices.is_empty());
        assert_eq!(result.mesh.indices.len() % 3, 0);
        let topology = result.topology.expect("DC backend should report topology");
        assert_eq!(topology.non_manifold_edges, 0);
        assert_eq!(topology.degenerate_triangles, 0);
    }

    #[test]
    fn adaptive_dual_contouring_reduces_wing_triangle_count() {
        let wing = wing_with_airfoil_export_safe(
            "2212",
            28.0,
            16.0,
            80.0,
            4.0,
            2.0,
            -1.0,
            AirfoilExportOptions {
                min_trailing_edge_thickness_mm: 2.0,
                min_leading_edge_radius_mm: 1.0,
            },
        );
        let bounds_min = Vec3::new(-5.0, -44.0, -8.0);
        let bounds_max = Vec3::new(34.0, 44.0, 10.0);
        let uniform = build_export_mesh_with_backend(
            &wing,
            bounds_min,
            bounds_max,
            3.0,
            ExportBackend::UniformDualContouring,
            false,
        );
        let adaptive = build_export_mesh_with_backend(
            &wing,
            bounds_min,
            bounds_max,
            3.0,
            ExportBackend::AdaptiveDualContouring,
            false,
        );
        assert!(!uniform.mesh.indices.is_empty());
        assert!(!adaptive.mesh.indices.is_empty());
        assert_eq!(adaptive.backend, ExportBackend::AdaptiveDualContouring);
        assert!(
            adaptive.mesh.indices.len() < uniform.mesh.indices.len(),
            "adaptive DC should emit fewer wing triangles than uniform DC"
        );
        let max_x = adaptive
            .mesh
            .vertices
            .iter()
            .map(|v| v.position[0])
            .fold(f32::MIN, f32::max);
        assert!(
            max_x > 25.0,
            "adaptive mesh should preserve the trailing edge extent"
        );
    }

    #[test]
    fn test_adaptive_manifold_closure() {
        let wing = wing_with_airfoil_export_safe(
            "2212",
            14.0,
            9.0,
            36.0,
            4.0,
            2.0,
            -1.0,
            AirfoilExportOptions {
                min_trailing_edge_thickness_mm: 1.5,
                min_leading_edge_radius_mm: 0.8,
            },
        );
        let bounds_min = Vec3::new(-4.37, -22.41, -6.29);
        let bounds_max = Vec3::new(20.63, 22.59, 8.71);
        let result = build_export_mesh_with_backend(
            &wing,
            bounds_min,
            bounds_max,
            1.0,
            ExportBackend::AdaptiveDualContouring,
            false,
        );
        assert!(!result.mesh.indices.is_empty());
        let topology = result.topology.expect("adaptive DC should report topology");
        assert_eq!(
            topology.boundary_edges, 0,
            "adaptive DC output must be watertight for CFD/slicer use"
        );
    }

    #[test]
    fn adaptive_export_mesh_produces_mesh_for_supported_surface() {
        let shape = Sphere::new(10.0);
        let result = build_export_mesh_adaptive(
            &shape,
            Vec3::splat(-14.0),
            Vec3::splat(14.0),
            AdaptiveExportSettings {
                quality: MeshQuality::Draft,
                smooth_normals: false,
                ..Default::default()
            },
        );
        assert!(!result.mesh.vertices.is_empty());
        assert_eq!(result.mesh.indices.len() % 3, 0);
        assert!(matches!(
            result.backend,
            "dual_contouring_octree"
                | "dual_contouring_uniform"
                | "uniform_export_fallback"
                | "fallback_uniform"
        ));
    }

    #[test]
    fn adaptive_export_mesh_falls_back_when_dual_has_boundary_edges() {
        let shape = Sphere::new(10.0);
        let result = build_export_mesh_adaptive(
            &shape,
            Vec3::splat(-14.0),
            Vec3::splat(14.0),
            AdaptiveExportSettings {
                quality: MeshQuality::Draft,
                smooth_normals: false,
                prefer_dual_contouring: true,
                ..Default::default()
            },
        );
        assert!(!result.mesh.vertices.is_empty());
        assert!(
            matches!(
                result.backend,
                "uniform_export_fallback" | "dual_contouring_octree" | "dual_contouring_uniform"
            ),
            "adaptive export should choose a valid backend"
        );
    }
}
