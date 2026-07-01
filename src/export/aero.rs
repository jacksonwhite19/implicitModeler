use std::collections::{BTreeMap, HashMap, HashSet};
use std::fs;
use std::hash::{DefaultHasher, Hash, Hasher};
use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::Instant;

use glam::Vec3;
use serde::Serialize;

use crate::export::adaptive_octree::AdaptiveOctreeSettings;
use crate::export::dual_contouring::fill_boundary_loops;
use crate::export::{
    AdaptiveExportSettings, build_export_mesh, build_export_mesh_adaptive,
    build_export_mesh_uniform_reference, build_export_mesh_with_target_cell, export_obj,
    export_stl,
};
use crate::mesh::{
    Mesh, MeshQuality, MeshQualityReport, Vertex, adaptive_mc::LocalRefineRegion,
    analyze_mesh_quality,
};
use crate::pipeline::auto_bounds;
use crate::scripting::AeroExportPart;
use crate::sdf::aerospace::{ExtrudedAirfoil, LoftedWing};
use crate::sdf::booleans::Union;
use crate::sdf::{Sdf, sdf_profile_node_visits, sdf_profile_reset, sdf_profile_set_enabled};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum AeroExportMode {
    External,
    ExternalPlusInlets,
    FullFluidBoundary,
}

impl AeroExportMode {
    pub fn parse(value: &str) -> Result<Self, String> {
        match value.trim().to_ascii_lowercase().as_str() {
            "external" => Ok(Self::External),
            "external_plus_inlets" => Ok(Self::ExternalPlusInlets),
            "full_fluid_boundary" => Ok(Self::FullFluidBoundary),
            other => Err(format!(
                "Unknown aero mode '{}'. Use external, external_plus_inlets, or full_fluid_boundary.",
                other
            )),
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::External => "external",
            Self::ExternalPlusInlets => "external_plus_inlets",
            Self::FullFluidBoundary => "full_fluid_boundary",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum AeroRole {
    OuterMoldLine,
    FlowPathInternal,
    InternalStructure,
    ConstructionOnly,
    Ignore,
}

impl AeroRole {
    pub fn parse(value: &str) -> Result<Self, String> {
        match value.trim().to_ascii_lowercase().as_str() {
            "outer_mold_line" => Ok(Self::OuterMoldLine),
            "flow_path_internal" => Ok(Self::FlowPathInternal),
            "internal_structure" => Ok(Self::InternalStructure),
            "construction_only" => Ok(Self::ConstructionOnly),
            "ignore" => Ok(Self::Ignore),
            other => Err(format!(
                "Unknown aero_role '{}'. Use outer_mold_line, flow_path_internal, internal_structure, construction_only, or ignore.",
                other
            )),
        }
    }

    fn included_in_mode(self, mode: AeroExportMode) -> bool {
        match mode {
            AeroExportMode::External => matches!(self, Self::OuterMoldLine),
            AeroExportMode::ExternalPlusInlets | AeroExportMode::FullFluidBoundary => {
                matches!(self, Self::OuterMoldLine | Self::FlowPathInternal)
            }
        }
    }
}

#[derive(Clone, Debug)]
pub struct AeroExportSettings {
    pub mode: AeroExportMode,
    pub quality: MeshQuality,
    pub smooth_normals: bool,
    pub drop_tiny_components: bool,
    pub min_component_triangles: usize,
    pub use_adaptive_extraction: bool,
    pub write_obj_per_patch: bool,
    pub max_patch_preflight_voxels: u64,
    pub adaptive_octree: AdaptiveOctreeSettings,
    pub fast_cfd_mode: bool,
    pub uniform_reference_mode: bool,
    pub uniform_reference_target_cell_mm: f32,
}

impl Default for AeroExportSettings {
    fn default() -> Self {
        Self {
            mode: AeroExportMode::External,
            quality: MeshQuality::Normal,
            smooth_normals: false,
            drop_tiny_components: true,
            min_component_triangles: 8,
            use_adaptive_extraction: true,
            write_obj_per_patch: false,
            max_patch_preflight_voxels: 128_000_000,
            adaptive_octree: AdaptiveOctreeSettings::default(),
            fast_cfd_mode: false,
            uniform_reference_mode: false,
            uniform_reference_target_cell_mm: 1.0,
        }
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct AeroPatchManifest {
    pub name: String,
    pub file: String,
    pub backend: String,
    pub backend_reason: String,
    pub meshing_time_ms: u64,
    pub estimated_uniform_resolution: u32,
    pub estimated_uniform_voxels: u64,
    pub triangle_count: usize,
    pub vertex_count: usize,
    pub bounds_min_mm: [f32; 3],
    pub bounds_max_mm: [f32; 3],
    pub contributors: Vec<String>,
    pub reported_min_feature_size_mm: Option<f32>,
    pub manifold_edge_errors: usize,
    pub boundary_edges: usize,
    pub cleanup_removed_triangles: usize,
    pub cleanup_time_ms: u64,
    pub write_time_ms: u64,
    pub quality_analysis_time_ms: u64,
    pub sdf_query_calls: u64,
    pub avg_sdf_eval_us: f64,
    pub avg_nodes_per_query: f64,
    pub avg_surface_deviation_mm: f32,
    pub max_surface_deviation_mm: f32,
    pub min_edge_length_mm: f32,
    pub max_edge_length_mm: f32,
    pub avg_triangle_aspect_ratio: f32,
    pub max_triangle_aspect_ratio: f32,
    pub mesh_quality: MeshQualityReport,
    pub watertight: bool,
    pub recommended_meshing_notes: Vec<String>,
    pub cache_hit: bool,
}

#[derive(Clone, Debug, Serialize)]
pub struct AeroExportManifest {
    pub mode: String,
    pub units: String,
    pub source_model: String,
    pub patches: Vec<AeroPatchManifest>,
    pub included_parts: Vec<String>,
    pub excluded_parts: Vec<String>,
    pub reachable_internal_parts: Vec<String>,
    pub excluded_sealed_internal_parts: Vec<String>,
    pub warnings: Vec<String>,
}

#[derive(Clone, Debug)]
struct PatchExportStats {
    mesh: Mesh,
    manifest: AeroPatchManifest,
    warnings: Vec<String>,
}

#[derive(Clone, Default)]
struct ReachabilityFilterResult {
    filtered_parts: Vec<AeroExportPart>,
    warnings: Vec<String>,
    reachable_internal_parts: Vec<String>,
    excluded_sealed_internal_parts: Vec<String>,
}

struct CountedSdf {
    inner: Arc<dyn Sdf>,
    query_calls: AtomicU64,
    query_nanos: AtomicU64,
}

impl CountedSdf {
    fn new(inner: Arc<dyn Sdf>) -> Self {
        Self {
            inner,
            query_calls: AtomicU64::new(0),
            query_nanos: AtomicU64::new(0),
        }
    }

    fn snapshot(&self) -> (u64, u64) {
        (
            self.query_calls.load(Ordering::Relaxed),
            self.query_nanos.load(Ordering::Relaxed),
        )
    }
}

impl Sdf for CountedSdf {
    fn distance(&self, point: Vec3) -> f32 {
        let t0 = Instant::now();
        let d = self.inner.distance(point);
        self.query_calls.fetch_add(1, Ordering::Relaxed);
        self.query_nanos
            .fetch_add(t0.elapsed().as_nanos() as u64, Ordering::Relaxed);
        d
    }
}

#[derive(Hash, PartialEq, Eq, Clone, Copy)]
struct AeroPatchCacheKey(u64);

fn aero_patch_cache() -> &'static Mutex<HashMap<AeroPatchCacheKey, PatchExportStats>> {
    static CACHE: OnceLock<Mutex<HashMap<AeroPatchCacheKey, PatchExportStats>>> = OnceLock::new();
    CACHE.get_or_init(|| Mutex::new(HashMap::new()))
}

fn aero_patch_union_cache() -> &'static Mutex<HashMap<AeroPatchCacheKey, Arc<dyn Sdf>>> {
    static CACHE: OnceLock<Mutex<HashMap<AeroPatchCacheKey, Arc<dyn Sdf>>>> = OnceLock::new();
    CACHE.get_or_init(|| Mutex::new(HashMap::new()))
}

fn quantize_vec3(v: Vec3) -> [i32; 3] {
    [
        (v.x * 100.0).round() as i32,
        (v.y * 100.0).round() as i32,
        (v.z * 100.0).round() as i32,
    ]
}

fn sdf_signature(sdf: &dyn Sdf) -> u64 {
    let (bmin, bmax) = auto_bounds(sdf);
    let center = (bmin + bmax) * 0.5;
    let sample_points = [
        bmin,
        bmax,
        center,
        Vec3::new(bmin.x, center.y, center.z),
        Vec3::new(bmax.x, center.y, center.z),
        Vec3::new(center.x, bmin.y, center.z),
        Vec3::new(center.x, bmax.y, center.z),
        Vec3::new(center.x, center.y, bmin.z),
        Vec3::new(center.x, center.y, bmax.z),
    ];
    let mut h = DefaultHasher::new();
    quantize_vec3(bmin).hash(&mut h);
    quantize_vec3(bmax).hash(&mut h);
    for p in sample_points {
        quantize_vec3(p).hash(&mut h);
        ((sdf.distance(p) * 1000.0).round() as i32).hash(&mut h);
    }
    h.finish()
}

fn patch_cache_key(
    patch_name: &str,
    contributors: &[AeroExportPart],
    settings: &AeroExportSettings,
) -> AeroPatchCacheKey {
    let mut h = DefaultHasher::new();
    patch_name.hash(&mut h);
    settings.mode.as_str().hash(&mut h);
    settings.smooth_normals.hash(&mut h);
    settings.drop_tiny_components.hash(&mut h);
    settings.min_component_triangles.hash(&mut h);
    std::mem::discriminant(&settings.quality).hash(&mut h);
    for contributor in contributors {
        contributor.name.hash(&mut h);
        contributor.aero_role.hash(&mut h);
        contributor.patch_name.hash(&mut h);
        contributor.include_in_modes.hash(&mut h);
        contributor.preserve_feature.hash(&mut h);
        contributor.feature_tags.hash(&mut h);
        contributor.reachability_seed.hash(&mut h);
        ((contributor.min_export_scale_mm * 1000.0).round() as i32).hash(&mut h);
        ((contributor.feature_protection_strength * 1000.0).round() as i32).hash(&mut h);
        contributor.priority.hash(&mut h);
        contributor.disable_auto_fast_path.hash(&mut h);
        contributor.refinement_paths.len().hash(&mut h);
        for path in &contributor.refinement_paths {
            path.points_mm.len().hash(&mut h);
            path.radii_mm.len().hash(&mut h);
            for point in &path.points_mm {
                quantize_vec3(Vec3::from_array(*point)).hash(&mut h);
            }
            for radius in &path.radii_mm {
                ((*radius * 1000.0).round() as i32).hash(&mut h);
            }
            ((path.forward_extend_mm * 1000.0).round() as i32).hash(&mut h);
            ((path.aft_extend_mm * 1000.0).round() as i32).hash(&mut h);
            if let Some(target) = path.target_cell_mm {
                ((target * 1000.0).round() as i32).hash(&mut h);
            }
        }
        contributor.refinement_body_sdf.is_some().hash(&mut h);
        contributor.refinement_void_sdf.is_some().hash(&mut h);
        contributor.refinement_path_points_mm.len().hash(&mut h);
        contributor.refinement_path_radii_mm.len().hash(&mut h);
        for point in &contributor.refinement_path_points_mm {
            quantize_vec3(Vec3::from_array(*point)).hash(&mut h);
        }
        for radius in &contributor.refinement_path_radii_mm {
            ((*radius * 1000.0).round() as i32).hash(&mut h);
        }
        ((contributor.refinement_path_forward_extend_mm * 1000.0).round() as i32).hash(&mut h);
        ((contributor.refinement_path_aft_extend_mm * 1000.0).round() as i32).hash(&mut h);
        if let Some(target) = contributor.refinement_path_target_cell_mm {
            ((target * 1000.0).round() as i32).hash(&mut h);
        }
        if let Some(ref sdf) = contributor.refinement_body_sdf {
            sdf_signature(sdf.as_ref()).hash(&mut h);
        }
        if let Some(ref sdf) = contributor.refinement_void_sdf {
            sdf_signature(sdf.as_ref()).hash(&mut h);
        }
        sdf_signature(contributor.sdf.as_ref()).hash(&mut h);
    }
    AeroPatchCacheKey(h.finish())
}

fn union_all(parts: &[Arc<dyn Sdf>]) -> Option<Arc<dyn Sdf>> {
    let mut iter = parts.iter().cloned();
    let first = iter.next()?;
    Some(iter.fold(first, |acc, sdf| Arc::new(Union::new(acc, sdf))))
}

fn explicit_part_bounds(part: &AeroExportPart) -> Option<(Vec3, Vec3)> {
    match (part.bounds_min_mm, part.bounds_max_mm) {
        (Some(min), Some(max)) => Some((Vec3::from_array(min), Vec3::from_array(max))),
        _ => None,
    }
}

fn padded_support_bounds(min: Vec3, max: Vec3) -> (Vec3, Vec3) {
    let span = (max - min).abs().max(Vec3::splat(1.0));
    let pad = span * 0.08 + Vec3::splat(1.0);
    (min - pad, max + pad)
}

fn metadata_part_bounds(part: &AeroExportPart) -> Option<(Vec3, Vec3)> {
    let metadata = part.sdf.metadata();
    let bounds = metadata.support_bounds?;
    bounds
        .is_valid()
        .then(|| padded_support_bounds(bounds.min, bounds.max))
}

fn part_bounds(part: &AeroExportPart) -> (Vec3, Vec3) {
    explicit_part_bounds(part)
        .or_else(|| metadata_part_bounds(part))
        .unwrap_or_else(|| auto_bounds(part.sdf.as_ref()))
}

fn contributor_bounds(contributors: &[AeroExportPart]) -> Option<(Vec3, Vec3)> {
    let mut min = Vec3::splat(f32::MAX);
    let mut max = Vec3::splat(f32::MIN);
    let mut saw_any = false;
    for contributor in contributors {
        let (bmin, bmax) = part_bounds(contributor);
        min = min.min(bmin);
        max = max.max(bmax);
        saw_any = true;
    }
    saw_any.then_some((min, max))
}

fn inferred_min_feature_size_mm(sdf: &dyn Sdf) -> Option<f32> {
    let any = sdf as &dyn std::any::Any;
    if let Some(wing) = any.downcast_ref::<LoftedWing>() {
        return wing.feature_metadata().map(|m| m.min_feature_size_mm);
    }
    if let Some(extruded) = any.downcast_ref::<ExtrudedAirfoil>() {
        return extruded
            .airfoil_feature_metadata()
            .map(|m| m.min_feature_size_mm);
    }
    None
}

fn patch_reported_min_feature_size_mm(contributors: &[AeroExportPart]) -> Option<f32> {
    contributors
        .iter()
        .filter_map(|part| {
            part.min_feature_size_mm
                .or_else(|| inferred_min_feature_size_mm(part.sdf.as_ref()))
        })
        .reduce(f32::min)
}

#[derive(Clone, Copy, Debug)]
struct ScoutGrid {
    nx: usize,
    ny: usize,
    nz: usize,
    cell_size: Vec3,
}

#[derive(Clone, Copy, Debug)]
struct ScoutBlock {
    mesh_min: Vec3,
    mesh_max: Vec3,
    axis: usize,
    core_min_axis: f32,
    core_max_axis: f32,
}

#[derive(Clone, Copy, Debug)]
struct RefineRegion {
    min: Vec3,
    max: Vec3,
    target_cell_mm: f32,
}

fn has_protected_features(contributors: &[AeroExportPart]) -> bool {
    contributors.iter().any(|c| {
        c.preserve_feature || !c.feature_tags.is_empty() || c.feature_protection_strength > 0.0
    })
}

fn build_scout_grid(
    bounds_min: Vec3,
    bounds_max: Vec3,
    target_cell_mm: f32,
    protected: bool,
) -> ScoutGrid {
    let span = (bounds_max - bounds_min)
        .abs()
        .max(Vec3::splat(target_cell_mm.max(0.1)));
    let scout_cell_mm =
        (target_cell_mm * if protected { 4.0 } else { 6.0 }).max(target_cell_mm * 2.0);
    let nx = ((span.x / scout_cell_mm).ceil() as usize).clamp(4, 48);
    let ny = ((span.y / scout_cell_mm).ceil() as usize).clamp(4, 48);
    let nz = ((span.z / scout_cell_mm).ceil() as usize).clamp(3, 32);
    let cell_size = Vec3::new(span.x / nx as f32, span.y / ny as f32, span.z / nz as f32);
    ScoutGrid {
        nx,
        ny,
        nz,
        cell_size,
    }
}

fn scout_corner_index(x: usize, y: usize, z: usize, nx: usize, ny: usize) -> usize {
    x + (nx + 1) * (y + (ny + 1) * z)
}

fn scout_cell_index(x: usize, y: usize, z: usize, nx: usize, ny: usize) -> usize {
    x + nx * (y + ny * z)
}

fn dilate_active_cells(active: &mut [bool], nx: usize, ny: usize, nz: usize, radius: usize) {
    if radius == 0 {
        return;
    }
    let original = active.to_vec();
    for z in 0..nz {
        for y in 0..ny {
            for x in 0..nx {
                let idx = scout_cell_index(x, y, z, nx, ny);
                if !original[idx] {
                    continue;
                }
                let x0 = x.saturating_sub(radius);
                let y0 = y.saturating_sub(radius);
                let z0 = z.saturating_sub(radius);
                let x1 = (x + radius).min(nx - 1);
                let y1 = (y + radius).min(ny - 1);
                let z1 = (z + radius).min(nz - 1);
                for dz in z0..=z1 {
                    for dy in y0..=y1 {
                        for dx in x0..=x1 {
                            active[scout_cell_index(dx, dy, dz, nx, ny)] = true;
                        }
                    }
                }
            }
        }
    }
}

fn scout_surface_macroblocks(
    sdf: &dyn Sdf,
    bounds_min: Vec3,
    bounds_max: Vec3,
    target_cell_mm: f32,
    contributors: &[AeroExportPart],
) -> Vec<ScoutBlock> {
    let protected = has_protected_features(contributors);
    let grid = build_scout_grid(bounds_min, bounds_max, target_cell_mm, protected);
    let ScoutGrid {
        nx,
        ny,
        nz,
        cell_size,
    } = grid;
    let mut corners = vec![0.0f32; (nx + 1) * (ny + 1) * (nz + 1)];
    for z in 0..=nz {
        for y in 0..=ny {
            for x in 0..=nx {
                let p = bounds_min
                    + Vec3::new(
                        x as f32 * cell_size.x,
                        y as f32 * cell_size.y,
                        z as f32 * cell_size.z,
                    );
                corners[scout_corner_index(x, y, z, nx, ny)] = sdf.distance(p);
            }
        }
    }

    let band_mm = (target_cell_mm * if protected { 2.0 } else { 1.5 }).max(2.0);
    let mut active = vec![false; nx * ny * nz];
    for z in 0..nz {
        for y in 0..ny {
            for x in 0..nx {
                let mut d_min = f32::MAX;
                let mut d_max = f32::MIN;
                for &(dx, dy, dz) in &[
                    (0, 0, 0),
                    (1, 0, 0),
                    (0, 1, 0),
                    (1, 1, 0),
                    (0, 0, 1),
                    (1, 0, 1),
                    (0, 1, 1),
                    (1, 1, 1),
                ] {
                    let d = corners[scout_corner_index(x + dx, y + dy, z + dz, nx, ny)];
                    d_min = d_min.min(d);
                    d_max = d_max.max(d);
                }
                let center = bounds_min
                    + Vec3::new(
                        (x as f32 + 0.5) * cell_size.x,
                        (y as f32 + 0.5) * cell_size.y,
                        (z as f32 + 0.5) * cell_size.z,
                    );
                let center_d = sdf.distance(center);
                let active_here = (d_min <= 0.0 && d_max >= 0.0)
                    || d_min.abs() <= band_mm
                    || d_max.abs() <= band_mm
                    || center_d.abs() <= band_mm;
                active[scout_cell_index(x, y, z, nx, ny)] = active_here;
            }
        }
    }

    dilate_active_cells(&mut active, nx, ny, nz, if protected { 1 } else { 0 });

    let mut blocks = Vec::new();
    let eps = Vec3::splat(0.01);
    let span = (bounds_max - bounds_min).abs();
    let primary_axis = if span.x >= span.y { 0usize } else { 1usize };
    let slab_step = if protected { 2usize } else { 3usize };
    let slab_count = if primary_axis == 0 { nx } else { ny };

    for slab_start in (0..slab_count).step_by(slab_step) {
        let slab_end = (slab_start + slab_step).min(slab_count);
        let mut found = false;
        let mut min_i = usize::MAX;
        let mut min_j = usize::MAX;
        let mut max_i = 0usize;
        let mut max_j = 0usize;
        if primary_axis == 0 {
            for z in 0..nz {
                for y in 0..ny {
                    for x in slab_start..slab_end {
                        if active[scout_cell_index(x, y, z, nx, ny)] {
                            found = true;
                            min_i = min_i.min(y);
                            min_j = min_j.min(z);
                            max_i = max_i.max(y + 1);
                            max_j = max_j.max(z + 1);
                        }
                    }
                }
            }
            if !found {
                continue;
            }
            let y0 = min_i.saturating_sub(1);
            let z0 = min_j.saturating_sub(1);
            let y1 = (max_i + 1).min(ny);
            let z1 = (max_j + 1).min(nz);
            let core_min_axis = bounds_min.x + slab_start as f32 * cell_size.x;
            let core_max_axis = bounds_min.x + slab_end as f32 * cell_size.x;
            let mesh_x0 = slab_start.saturating_sub(1);
            let mesh_x1 = (slab_end + 1).min(nx);
            let mesh_min = bounds_min
                + Vec3::new(
                    mesh_x0 as f32 * cell_size.x,
                    y0 as f32 * cell_size.y,
                    z0 as f32 * cell_size.z,
                );
            let mesh_max = bounds_min
                + Vec3::new(
                    mesh_x1 as f32 * cell_size.x,
                    y1 as f32 * cell_size.y,
                    z1 as f32 * cell_size.z,
                );
            blocks.push(ScoutBlock {
                mesh_min: mesh_min - eps,
                mesh_max: mesh_max + eps,
                axis: 0,
                core_min_axis,
                core_max_axis,
            });
        } else {
            for z in 0..nz {
                for x in 0..nx {
                    for y in slab_start..slab_end {
                        if active[scout_cell_index(x, y, z, nx, ny)] {
                            found = true;
                            min_i = min_i.min(x);
                            min_j = min_j.min(z);
                            max_i = max_i.max(x + 1);
                            max_j = max_j.max(z + 1);
                        }
                    }
                }
            }
            if !found {
                continue;
            }
            let x0 = min_i.saturating_sub(1);
            let z0 = min_j.saturating_sub(1);
            let x1 = (max_i + 1).min(nx);
            let z1 = (max_j + 1).min(nz);
            let core_min_axis = bounds_min.y + slab_start as f32 * cell_size.y;
            let core_max_axis = bounds_min.y + slab_end as f32 * cell_size.y;
            let mesh_y0 = slab_start.saturating_sub(1);
            let mesh_y1 = (slab_end + 1).min(ny);
            let mesh_min = bounds_min
                + Vec3::new(
                    x0 as f32 * cell_size.x,
                    mesh_y0 as f32 * cell_size.y,
                    z0 as f32 * cell_size.z,
                );
            let mesh_max = bounds_min
                + Vec3::new(
                    x1 as f32 * cell_size.x,
                    mesh_y1 as f32 * cell_size.y,
                    z1 as f32 * cell_size.z,
                );
            blocks.push(ScoutBlock {
                mesh_min: mesh_min - eps,
                mesh_max: mesh_max + eps,
                axis: 1,
                core_min_axis,
                core_max_axis,
            });
        }
    }
    blocks
}

fn component_along_axis(v: Vec3, axis: usize) -> f32 {
    match axis {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    }
}

fn filter_mesh_to_core_axis_range(mesh: &Mesh, axis: usize, min_axis: f32, max_axis: f32) -> Mesh {
    if mesh.indices.is_empty() {
        return mesh.clone();
    }
    let tol = 0.05f32;
    let mut kept_indices = Vec::with_capacity(mesh.indices.len());
    for tri in mesh.indices.chunks_exact(3) {
        let p0 = Vec3::from_array(mesh.vertices[tri[0] as usize].position);
        let p1 = Vec3::from_array(mesh.vertices[tri[1] as usize].position);
        let p2 = Vec3::from_array(mesh.vertices[tri[2] as usize].position);
        let c = (p0 + p1 + p2) / 3.0;
        let a = component_along_axis(c, axis);
        if a >= min_axis - tol && a <= max_axis + tol {
            kept_indices.extend_from_slice(tri);
        }
    }
    Mesh {
        vertices: mesh.vertices.clone(),
        indices: kept_indices,
    }
}

fn point_in_aabb(point: Vec3, min: Vec3, max: Vec3) -> bool {
    point.x >= min.x
        && point.x <= max.x
        && point.y >= min.y
        && point.y <= max.y
        && point.z >= min.z
        && point.z <= max.z
}

fn filter_mesh_excluding_regions(mesh: &Mesh, regions: &[RefineRegion]) -> Mesh {
    if mesh.indices.is_empty() || regions.is_empty() {
        return mesh.clone();
    }
    let mut kept_indices = Vec::with_capacity(mesh.indices.len());
    for tri in mesh.indices.chunks_exact(3) {
        let p0 = Vec3::from_array(mesh.vertices[tri[0] as usize].position);
        let p1 = Vec3::from_array(mesh.vertices[tri[1] as usize].position);
        let p2 = Vec3::from_array(mesh.vertices[tri[2] as usize].position);
        let center = (p0 + p1 + p2) / 3.0;
        let inside_refine_region = regions
            .iter()
            .any(|region| point_in_aabb(center, region.min, region.max));
        if !inside_refine_region {
            kept_indices.extend_from_slice(tri);
        }
    }
    Mesh {
        vertices: mesh.vertices.clone(),
        indices: kept_indices,
    }
}

fn feature_refine_regions(
    contributors: &[AeroExportPart],
    bounds_min: Vec3,
    bounds_max: Vec3,
    coarse_target_cell_mm: f32,
) -> Vec<RefineRegion> {
    let mut has_leading_edge = false;
    let mut has_trailing_edge = false;
    let mut has_junction = false;
    let mut has_inlet_lip = false;
    let mut has_inlet_region = false;
    let mut has_exhaust_region = false;
    for contributor in contributors {
        for tag in &contributor.feature_tags {
            match tag.as_str() {
                "leading_edge" => has_leading_edge = true,
                "trailing_edge" => has_trailing_edge = true,
                "wing_body_junction" | "tail_body_junction" | "junction" => has_junction = true,
                "inlet_lip" => has_inlet_lip = true,
                "inlet_region" => has_inlet_region = true,
                "exhaust_region" => has_exhaust_region = true,
                _ => {}
            }
        }
    }
    if !(has_leading_edge
        || has_trailing_edge
        || has_junction
        || has_inlet_lip
        || has_inlet_region
        || has_exhaust_region)
    {
        return Vec::new();
    }

    let span = (bounds_max - bounds_min).abs();
    let center = (bounds_min + bounds_max) * 0.5;
    let pad = Vec3::splat((coarse_target_cell_mm * 2.5).max(6.0));
    let half_span_y = span.y * 0.5;
    let mut regions = Vec::new();

    if has_leading_edge {
        let width = (span.x * 0.14).max(50.0);
        if half_span_y > 120.0 {
            let y_band_start = half_span_y * 0.30;
            for sign in [-1.0f32, 1.0f32] {
                let y0 = center.y + sign * y_band_start;
                let y1 = center.y + sign * half_span_y;
                let min_y = y0.min(y1) - pad.y;
                let max_y = y0.max(y1) + pad.y;
                regions.push(RefineRegion {
                    min: Vec3::new(bounds_min.x - pad.x, min_y, bounds_min.z - pad.z),
                    max: Vec3::new(
                        (bounds_min.x + width).min(bounds_max.x) + pad.x,
                        max_y,
                        bounds_max.z + pad.z,
                    ),
                    target_cell_mm: (coarse_target_cell_mm * 0.80).max(3.4),
                });
            }
        } else {
            regions.push(RefineRegion {
                min: Vec3::new(
                    bounds_min.x - pad.x,
                    bounds_min.y - pad.y,
                    bounds_min.z - pad.z,
                ),
                max: Vec3::new(
                    (bounds_min.x + width).min(bounds_max.x) + pad.x,
                    bounds_max.y + pad.y,
                    bounds_max.z + pad.z,
                ),
                target_cell_mm: (coarse_target_cell_mm * 0.80).max(3.4),
            });
        }
    }
    if has_trailing_edge {
        let width = (span.x * 0.12).max(50.0);
        if half_span_y > 120.0 {
            let y_band_start = half_span_y * 0.34;
            for sign in [-1.0f32, 1.0f32] {
                let y0 = center.y + sign * y_band_start;
                let y1 = center.y + sign * half_span_y;
                let min_y = y0.min(y1) - pad.y;
                let max_y = y0.max(y1) + pad.y;
                regions.push(RefineRegion {
                    min: Vec3::new(
                        (bounds_max.x - width).max(bounds_min.x) - pad.x,
                        min_y,
                        bounds_min.z - pad.z,
                    ),
                    max: Vec3::new(bounds_max.x + pad.x, max_y, bounds_max.z + pad.z),
                    target_cell_mm: (coarse_target_cell_mm * 0.74).max(3.1),
                });
            }
        } else {
            regions.push(RefineRegion {
                min: Vec3::new(
                    (bounds_max.x - width).max(bounds_min.x) - pad.x,
                    bounds_min.y - pad.y,
                    bounds_min.z - pad.z,
                ),
                max: Vec3::new(
                    bounds_max.x + pad.x,
                    bounds_max.y + pad.y,
                    bounds_max.z + pad.z,
                ),
                target_cell_mm: (coarse_target_cell_mm * 0.74).max(3.1),
            });
        }
    }
    if has_junction {
        let x_half = (span.x * 0.14).max(50.0);
        let y_half = (span.y * 0.14).max(55.0);
        regions.push(RefineRegion {
            min: Vec3::new(
                center.x - x_half - pad.x,
                center.y - y_half - pad.y,
                bounds_min.z - pad.z,
            ),
            max: Vec3::new(
                center.x + x_half + pad.x,
                center.y + y_half + pad.y,
                bounds_max.z + pad.z,
            ),
            target_cell_mm: (coarse_target_cell_mm * 0.75).max(3.1),
        });
    }
    if has_inlet_lip {
        let x_half = (span.x * 0.10).max(24.0);
        let y_half = (span.y * 0.12).max(18.0);
        let z_half = (span.z * 0.16).max(18.0);
        regions.push(RefineRegion {
            min: Vec3::new(
                center.x - x_half - pad.x,
                center.y - y_half - pad.y,
                center.z - z_half - pad.z,
            ),
            max: Vec3::new(
                center.x + x_half + pad.x,
                center.y + y_half + pad.y,
                center.z + z_half + pad.z,
            ),
            target_cell_mm: (coarse_target_cell_mm * 0.72).max(2.0),
        });
    }
    if has_inlet_region {
        let x0 = bounds_min.x - pad.x;
        let x1 = (bounds_min.x + span.x * 0.28).min(bounds_max.x) + pad.x;
        let y_half = (span.y * 0.18).max(30.0);
        let z0 = center.z - (span.z * 0.05).max(16.0) - pad.z;
        let z1 = bounds_max.z + pad.z;
        regions.push(RefineRegion {
            min: Vec3::new(x0, center.y - y_half - pad.y, z0),
            max: Vec3::new(x1, center.y + y_half + pad.y, z1),
            target_cell_mm: (coarse_target_cell_mm * 0.55).max(1.5),
        });
    }
    if has_exhaust_region {
        let x0 = (bounds_max.x - span.x * 0.18).max(bounds_min.x) - pad.x;
        let x1 = bounds_max.x + pad.x;
        let y_half = (span.y * 0.16).max(28.0);
        let z_half = (span.z * 0.28).max(26.0);
        regions.push(RefineRegion {
            min: Vec3::new(x0, center.y - y_half - pad.y, center.z - z_half - pad.z),
            max: Vec3::new(x1, center.y + y_half + pad.y, center.z + z_half + pad.z),
            target_cell_mm: (coarse_target_cell_mm * 0.55).max(1.5),
        });
    }

    regions
}

fn auto_clearance_refine_regions(
    contributors: &[AeroExportPart],
    bounds_min: Vec3,
    bounds_max: Vec3,
    coarse_target_cell_mm: f32,
) -> Vec<RefineRegion> {
    let Some(body_sdf) = contributors
        .iter()
        .find_map(|c| c.refinement_body_sdf.as_ref())
    else {
        return Vec::new();
    };
    let Some(void_sdf) = contributors
        .iter()
        .find_map(|c| c.refinement_void_sdf.as_ref())
    else {
        return Vec::new();
    };

    let threshold_mm = 4.0f32;
    let near_zero_band_mm = threshold_mm * 1.5f32;
    let span = (bounds_max - bounds_min).abs();
    let nx = 28usize;
    let ny = 28usize;
    let nz = 16usize;
    let step = Vec3::new(
        span.x / (nx as f32 - 1.0),
        span.y / (ny as f32 - 1.0),
        span.z / (nz as f32 - 1.0),
    );
    let idx = |x: usize, y: usize, z: usize| x + nx * (y + ny * z);
    let mut marked = vec![false; nx * ny * nz];

    for z in 0..nz {
        for y in 0..ny {
            for x in 0..nx {
                let p =
                    bounds_min + Vec3::new(x as f32 * step.x, y as f32 * step.y, z as f32 * step.z);
                let db = body_sdf.distance(p).abs();
                let dv = void_sdf.distance(p).abs();
                let implied_clearance = db + dv;
                if db <= near_zero_band_mm
                    && dv <= near_zero_band_mm
                    && implied_clearance <= threshold_mm
                {
                    marked[idx(x, y, z)] = true;
                }
            }
        }
    }

    let mut visited = vec![false; marked.len()];
    let mut regions = Vec::new();
    for z in 0..nz {
        for y in 0..ny {
            for x in 0..nx {
                let start = idx(x, y, z);
                if visited[start] || !marked[start] {
                    continue;
                }
                let mut stack = vec![(x, y, z)];
                visited[start] = true;
                let mut cmin = Vec3::splat(f32::MAX);
                let mut cmax = Vec3::splat(f32::MIN);
                let mut count = 0usize;
                while let Some((cx, cy, cz)) = stack.pop() {
                    count += 1;
                    let p = bounds_min
                        + Vec3::new(cx as f32 * step.x, cy as f32 * step.y, cz as f32 * step.z);
                    cmin = cmin.min(p);
                    cmax = cmax.max(p);
                    for (nxo, nyo, nzo) in [
                        (1isize, 0isize, 0isize),
                        (-1, 0, 0),
                        (0, 1, 0),
                        (0, -1, 0),
                        (0, 0, 1),
                        (0, 0, -1),
                    ] {
                        let xx = cx as isize + nxo;
                        let yy = cy as isize + nyo;
                        let zz = cz as isize + nzo;
                        if xx < 0
                            || yy < 0
                            || zz < 0
                            || xx >= nx as isize
                            || yy >= ny as isize
                            || zz >= nz as isize
                        {
                            continue;
                        }
                        let next = idx(xx as usize, yy as usize, zz as usize);
                        if !visited[next] && marked[next] {
                            visited[next] = true;
                            stack.push((xx as usize, yy as usize, zz as usize));
                        }
                    }
                }
                if count < 2 {
                    continue;
                }
                let pad = Vec3::new(step.x * 1.5, step.y * 1.5, step.z * 1.5);
                regions.push(RefineRegion {
                    min: cmin - pad,
                    max: cmax + pad,
                    target_cell_mm: (coarse_target_cell_mm * 0.5).max(1.25),
                });
            }
        }
    }

    regions
}

fn collect_refinement_path_specs(
    contributors: &[AeroExportPart],
) -> Vec<(Vec<Vec3>, Vec<f32>, f32, f32, f32)> {
    let mut specs = Vec::new();
    for contributor in contributors {
        for path in &contributor.refinement_paths {
            let points: Vec<Vec3> = path
                .points_mm
                .iter()
                .map(|p| Vec3::from_array(*p))
                .collect();
            if points.len() >= 2 && path.radii_mm.len() == points.len() {
                specs.push((
                    points,
                    path.radii_mm.clone(),
                    path.forward_extend_mm.max(0.0),
                    path.aft_extend_mm.max(0.0),
                    path.target_cell_mm.unwrap_or(0.5).max(0.05),
                ));
            }
        }
        if contributor.refinement_path_points_mm.len() >= 2
            && contributor.refinement_path_radii_mm.len()
                == contributor.refinement_path_points_mm.len()
        {
            specs.push((
                contributor
                    .refinement_path_points_mm
                    .iter()
                    .map(|p| Vec3::from_array(*p))
                    .collect(),
                contributor.refinement_path_radii_mm.clone(),
                contributor.refinement_path_forward_extend_mm.max(0.0),
                contributor.refinement_path_aft_extend_mm.max(0.0),
                contributor
                    .refinement_path_target_cell_mm
                    .unwrap_or(0.5)
                    .max(0.05),
            ));
        }
    }
    specs
}

fn path_refine_regions(contributors: &[AeroExportPart]) -> Vec<RefineRegion> {
    let specs = collect_refinement_path_specs(contributors);
    if specs.is_empty() {
        return Vec::new();
    }
    let sample_step_mm = 10.0f32;
    let pad = Vec3::splat(0.5);
    let mut regions = Vec::new();

    for (mut points, radii, forward_extend, aft_extend, target_cell_mm) in specs {
        if forward_extend > 0.0 {
            let dir = (points[1] - points[0]).normalize_or_zero();
            points[0] -= dir * forward_extend;
        }
        if aft_extend > 0.0 {
            let n = points.len();
            let dir = (points[n - 1] - points[n - 2]).normalize_or_zero();
            points[n - 1] += dir * aft_extend;
        }

        let mut sampled_points = vec![points[0]];
        let mut sampled_radii = vec![radii[0]];
        for i in 0..(points.len() - 1) {
            let p0 = points[i];
            let p1 = points[i + 1];
            let r0 = radii[i];
            let r1 = radii[i + 1];
            let len = p0.distance(p1).max(1e-4);
            let subdivisions = (len / sample_step_mm).ceil().max(1.0) as usize;
            for step in 1..=subdivisions {
                let t = step as f32 / subdivisions as f32;
                sampled_points.push(p0.lerp(p1, t));
                sampled_radii.push(r0 + (r1 - r0) * t);
            }
        }

        for i in 0..(sampled_points.len().saturating_sub(1)) {
            let p0 = sampled_points[i];
            let p1 = sampled_points[i + 1];
            let radius = sampled_radii[i].max(sampled_radii[i + 1]).max(0.5);
            let radius_pad = Vec3::splat(radius);
            regions.push(RefineRegion {
                min: p0.min(p1) - radius_pad - pad,
                max: p0.max(p1) + radius_pad + pad,
                target_cell_mm,
            });
        }
    }
    regions
}

fn merge_mesh_into(dst: &mut Mesh, src: &Mesh) {
    if src.vertices.is_empty() || src.indices.is_empty() {
        return;
    }
    let vertex_offset = dst.vertices.len() as u32;
    dst.vertices.extend(src.vertices.iter().copied());
    dst.indices
        .extend(src.indices.iter().map(|idx| idx + vertex_offset));
}

fn to_local_refine_regions(regions: &[RefineRegion]) -> Vec<LocalRefineRegion> {
    regions
        .iter()
        .map(|region| LocalRefineRegion {
            min: region.min,
            max: region.max,
            target_cell_size: region.target_cell_mm,
        })
        .collect()
}

fn filter_reachable_internal_parts(
    parts: &[AeroExportPart],
    mode: AeroExportMode,
) -> Result<ReachabilityFilterResult, String> {
    if !matches!(
        mode,
        AeroExportMode::ExternalPlusInlets | AeroExportMode::FullFluidBoundary
    ) {
        return Ok(ReachabilityFilterResult {
            filtered_parts: parts.to_vec(),
            ..ReachabilityFilterResult::default()
        });
    }

    let included_sdfs: Vec<_> = parts.iter().map(|p| Arc::clone(&p.sdf)).collect();
    let Some(union_sdf) = union_all(&included_sdfs) else {
        return Ok(ReachabilityFilterResult::default());
    };
    let (mut bmin, mut bmax) =
        contributor_bounds(parts).unwrap_or_else(|| auto_bounds(union_sdf.as_ref()));
    let pad = Vec3::splat(5.0);
    bmin -= pad;
    bmax += pad;

    let resolution = 36usize;
    let step = (bmax - bmin) / (resolution as f32 - 1.0);
    let idx = |x: usize, y: usize, z: usize| x + resolution * (y + resolution * z);
    let mut void_cells = vec![false; resolution * resolution * resolution];
    for z in 0..resolution {
        for y in 0..resolution {
            for x in 0..resolution {
                let p = bmin + Vec3::new(x as f32 * step.x, y as f32 * step.y, z as f32 * step.z);
                void_cells[idx(x, y, z)] = union_sdf.distance(p) > 0.0;
            }
        }
    }

    let mut reachable = vec![false; void_cells.len()];
    let mut queue = std::collections::VecDeque::new();
    let enqueue =
        |x: usize,
         y: usize,
         z: usize,
         reachable: &mut [bool],
         queue: &mut std::collections::VecDeque<(usize, usize, usize)>| {
            let i = idx(x, y, z);
            if void_cells[i] && !reachable[i] {
                reachable[i] = true;
                queue.push_back((x, y, z));
            }
        };

    for z in 0..resolution {
        for y in 0..resolution {
            enqueue(0, y, z, &mut reachable, &mut queue);
            enqueue(resolution - 1, y, z, &mut reachable, &mut queue);
        }
    }
    for z in 0..resolution {
        for x in 0..resolution {
            enqueue(x, 0, z, &mut reachable, &mut queue);
            enqueue(x, resolution - 1, z, &mut reachable, &mut queue);
        }
    }
    for y in 0..resolution {
        for x in 0..resolution {
            enqueue(x, y, 0, &mut reachable, &mut queue);
            enqueue(x, y, resolution - 1, &mut reachable, &mut queue);
        }
    }

    for part in parts {
        if part.reachability_seed.is_empty() {
            continue;
        }
        let (pmin, pmax) = part_bounds(part);
        let min_x = (((pmin.x - bmin.x) / step.x).floor().max(0.0)) as usize;
        let min_y = (((pmin.y - bmin.y) / step.y).floor().max(0.0)) as usize;
        let min_z = (((pmin.z - bmin.z) / step.z).floor().max(0.0)) as usize;
        let max_x = (((pmax.x - bmin.x) / step.x)
            .ceil()
            .min((resolution - 1) as f32)) as usize;
        let max_y = (((pmax.y - bmin.y) / step.y)
            .ceil()
            .min((resolution - 1) as f32)) as usize;
        let max_z = (((pmax.z - bmin.z) / step.z)
            .ceil()
            .min((resolution - 1) as f32)) as usize;
        for z in min_z..=max_z {
            for y in min_y..=max_y {
                for x in min_x..=max_x {
                    enqueue(x, y, z, &mut reachable, &mut queue);
                }
            }
        }
    }

    while let Some((x, y, z)) = queue.pop_front() {
        let neighbors = [
            (x.wrapping_sub(1), y, z, x > 0),
            (x + 1, y, z, x + 1 < resolution),
            (x, y.wrapping_sub(1), z, y > 0),
            (x, y + 1, z, y + 1 < resolution),
            (x, y, z.wrapping_sub(1), z > 0),
            (x, y, z + 1, z + 1 < resolution),
        ];
        for &(nx, ny, nz, valid) in &neighbors {
            if valid {
                enqueue(nx, ny, nz, &mut reachable, &mut queue);
            }
        }
    }

    let mut filtered = Vec::new();
    let mut warnings = Vec::new();
    let mut reachable_internal_parts = Vec::new();
    let mut excluded_sealed_internal_parts = Vec::new();
    for part in parts {
        if part.aero_role != "flow_path_internal" {
            filtered.push(part.clone());
            continue;
        }
        let (pmin, pmax) = part_bounds(part);
        let min_x = (((pmin.x - bmin.x) / step.x).floor().max(0.0)) as usize;
        let min_y = (((pmin.y - bmin.y) / step.y).floor().max(0.0)) as usize;
        let min_z = (((pmin.z - bmin.z) / step.z).floor().max(0.0)) as usize;
        let max_x = (((pmax.x - bmin.x) / step.x)
            .ceil()
            .min((resolution - 1) as f32)) as usize;
        let max_y = (((pmax.y - bmin.y) / step.y)
            .ceil()
            .min((resolution - 1) as f32)) as usize;
        let max_z = (((pmax.z - bmin.z) / step.z)
            .ceil()
            .min((resolution - 1) as f32)) as usize;
        let mut touches_reachable_void = false;
        'outer: for z in min_z..=max_z {
            for y in min_y..=max_y {
                for x in min_x..=max_x {
                    if reachable[idx(x, y, z)] {
                        touches_reachable_void = true;
                        break 'outer;
                    }
                }
            }
        }
        if touches_reachable_void {
            reachable_internal_parts.push(part.name.clone());
            filtered.push(part.clone());
        } else {
            excluded_sealed_internal_parts.push(part.name.clone());
            warnings.push(format!(
                "Excluded sealed internal flow patch '{}' because it did not touch reachable fluid",
                part.name
            ));
        }
    }

    Ok(ReachabilityFilterResult {
        filtered_parts: filtered,
        warnings,
        reachable_internal_parts,
        excluded_sealed_internal_parts,
    })
}

fn triangle_area(mesh: &Mesh, tri_index: usize) -> f32 {
    let base = tri_index * 3;
    let i0 = mesh.indices[base] as usize;
    let i1 = mesh.indices[base + 1] as usize;
    let i2 = mesh.indices[base + 2] as usize;
    let p0 = Vec3::from_array(mesh.vertices[i0].position);
    let p1 = Vec3::from_array(mesh.vertices[i1].position);
    let p2 = Vec3::from_array(mesh.vertices[i2].position);
    0.5 * (p1 - p0).cross(p2 - p0).length()
}

fn canonical_edge(a: u32, b: u32) -> (u32, u32) {
    if a < b { (a, b) } else { (b, a) }
}

fn canonical_triangle_indices(a: u32, b: u32, c: u32) -> [u32; 3] {
    let mut tri = [a, b, c];
    tri.sort_unstable();
    tri
}

fn permutation_parity(mut tri: [u32; 3]) -> bool {
    let mut swaps = 0usize;
    for i in 0..3 {
        let mut min_idx = i;
        for j in (i + 1)..3 {
            if tri[j] < tri[min_idx] {
                min_idx = j;
            }
        }
        if min_idx != i {
            tri.swap(i, min_idx);
            swaps += 1;
        }
    }
    swaps % 2 == 0
}

fn classify_patch_feature(
    patch_name: &str,
    center: Vec3,
    bounds_min: Vec3,
    bounds_max: Vec3,
) -> &'static str {
    let extent = (bounds_max - bounds_min).abs();
    let tol = Vec3::new(
        (extent.x * 0.08).max(2.0),
        (extent.y * 0.08).max(2.0),
        (extent.z * 0.08).max(2.0),
    );
    let near_bounds = (center.x - bounds_min.x).abs() <= tol.x
        || (center.x - bounds_max.x).abs() <= tol.x
        || (center.y - bounds_min.y).abs() <= tol.y
        || (center.y - bounds_max.y).abs() <= tol.y
        || (center.z - bounds_min.z).abs() <= tol.z
        || (center.z - bounds_max.z).abs() <= tol.z;
    if near_bounds {
        return "export_bounds_edge";
    }
    let name = patch_name.to_ascii_lowercase();
    if name.contains("wing") {
        if center.x > bounds_min.x + 0.8 * extent.x {
            return "trailing_edge";
        }
        if center.x < bounds_min.x + 0.15 * extent.x {
            return "leading_edge";
        }
        if center.y.abs() < 0.15 * extent.y {
            return "root";
        }
        if center.y.abs() > 0.42 * extent.y {
            return "tip";
        }
    }
    if name.contains("vtail") {
        if center.z > bounds_min.z + 0.8 * extent.z {
            return "tip";
        }
        if center.x > bounds_min.x + 0.7 * extent.x {
            return "trailing_edge";
        }
        if center.x < bounds_min.x + 0.2 * extent.x {
            return "leading_edge";
        }
        if center.y.abs() < 0.2 * extent.y {
            return "blend_junction";
        }
    }
    if name.contains("fuselage") {
        if center.x < bounds_min.x + 0.15 * extent.x {
            return "nose";
        }
        if center.x > bounds_min.x + 0.85 * extent.x {
            return "tail";
        }
        if center.z > bounds_min.z + 0.7 * extent.z {
            return "crown_blend";
        }
    }
    if name.contains("inlet") {
        if center.x < bounds_min.x + 0.2 * extent.x {
            return "inlet_mouth";
        }
        if center.x > bounds_min.x + 0.8 * extent.x {
            return "aft_transition";
        }
    }
    "unknown"
}

fn log_cluster_summary(
    prefix: &str,
    patch_name: &str,
    items: &[(Vec3, f32)],
    bounds_min: Vec3,
    bounds_max: Vec3,
) {
    if items.is_empty() {
        eprintln!("  {} clusters=0", prefix);
        return;
    }
    let extent = (bounds_max - bounds_min).abs() + Vec3::splat(1e-9);
    let mut bins: HashMap<(i32, i32, i32), Vec<(Vec3, f32)>> = HashMap::new();
    for (center, metric) in items {
        let rel = (*center - bounds_min) / extent;
        let key = (
            (rel.x * 10.0).floor().clamp(0.0, 9.0) as i32,
            (rel.y * 10.0).floor().clamp(0.0, 9.0) as i32,
            (rel.z * 10.0).floor().clamp(0.0, 9.0) as i32,
        );
        bins.entry(key).or_default().push((*center, *metric));
    }
    let mut ordered: Vec<_> = bins.into_iter().collect();
    ordered.sort_by(|a, b| b.1.len().cmp(&a.1.len()));
    eprintln!("  {} clusters={}", prefix, ordered.len());
    for (rank, (cell, pts)) in ordered.into_iter().take(5).enumerate() {
        let mut cmin = Vec3::splat(f32::MAX);
        let mut cmax = Vec3::splat(f32::MIN);
        let mut metric_sum = 0.0f32;
        for (center, metric) in &pts {
            cmin = cmin.min(*center);
            cmax = cmax.max(*center);
            metric_sum += *metric;
        }
        let avg_metric = metric_sum / pts.len() as f32;
        let feature =
            classify_patch_feature(patch_name, (cmin + cmax) * 0.5, bounds_min, bounds_max);
        eprintln!(
            "  {} cluster_{} count={} bbox=({:.3},{:.3},{:.3})..({:.3},{:.3},{:.3}) avg_metric={:.6} feature={} cell=[{},{},{}]",
            prefix,
            rank + 1,
            pts.len(),
            cmin.x,
            cmin.y,
            cmin.z,
            cmax.x,
            cmax.y,
            cmax.z,
            avg_metric,
            feature,
            cell.0,
            cell.1,
            cell.2,
        );
    }
}

fn remove_exact_duplicate_triangles(mesh: &Mesh) -> (Mesh, usize) {
    let mut exact_seen: HashSet<[u32; 3]> = HashSet::new();
    let mut kept = Vec::with_capacity(mesh.indices.len());
    let mut removed = 0usize;
    for tri in mesh.indices.chunks_exact(3) {
        let exact = [tri[0], tri[1], tri[2]];
        if !exact_seen.insert(exact) {
            removed += 1;
            continue;
        }
        kept.extend_from_slice(tri);
    }
    (
        Mesh {
            vertices: mesh.vertices.clone(),
            indices: kept,
        },
        removed,
    )
}

fn remove_same_vertex_set_duplicate_triangles(mesh: &Mesh) -> (Mesh, usize, usize) {
    let mut canonical_seen: HashSet<[u32; 3]> = HashSet::new();
    let mut kept = Vec::with_capacity(mesh.indices.len());
    let mut reversed_removed = 0usize;
    let mut same_set_removed = 0usize;
    for tri in mesh.indices.chunks_exact(3) {
        let canonical = canonical_triangle_indices(tri[0], tri[1], tri[2]);
        if !canonical_seen.insert(canonical) {
            same_set_removed += 1;
            reversed_removed += 1;
            continue;
        }
        kept.extend_from_slice(tri);
    }
    (
        Mesh {
            vertices: mesh.vertices.clone(),
            indices: kept,
        },
        reversed_removed,
        same_set_removed,
    )
}

fn print_raw_mesh_topology_diagnostic(mesh: &Mesh, patch_name: &str, weld_tolerance_mm: f32) {
    let bounds = mesh_bounds(mesh);
    let bounds_min = Vec3::from_array(bounds.0);
    let bounds_max = Vec3::from_array(bounds.1);
    let tri_count = mesh.indices.len() / 3;
    let mut edge_counts: HashMap<(u32, u32), usize> = HashMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        for &(a, b) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            *edge_counts.entry(canonical_edge(a, b)).or_insert(0) += 1;
        }
    }
    let mut edges_1 = 0usize;
    let mut edges_2 = 0usize;
    let mut edges_3 = 0usize;
    let mut edges_4 = 0usize;
    let mut edges_gt4 = 0usize;
    let mut nonmanifold_items = Vec::new();
    for (&(a, b), &count) in &edge_counts {
        match count {
            1 => edges_1 += 1,
            2 => edges_2 += 1,
            3 => edges_3 += 1,
            4 => edges_4 += 1,
            _ => edges_gt4 += 1,
        }
        if count != 2 {
            let p0 = Vec3::from_array(mesh.vertices[a as usize].position);
            let p1 = Vec3::from_array(mesh.vertices[b as usize].position);
            nonmanifold_items.push(((p0 + p1) * 0.5, p0.distance(p1)));
        }
    }
    eprintln!(
        "  raw-topology edge_use patch={} tri_count={} edges_used_by_1={} edges_used_by_2={} edges_used_by_3={} edges_used_by_4={} edges_used_by_gt4={}",
        patch_name, tri_count, edges_1, edges_2, edges_3, edges_4, edges_gt4
    );
    log_cluster_summary(
        "raw-topology nonmanifold_edges",
        patch_name,
        &nonmanifold_items,
        bounds_min,
        bounds_max,
    );

    let mut exact_oriented: HashMap<[u32; 3], usize> = HashMap::new();
    let mut by_vertex_set: HashMap<[u32; 3], Vec<[u32; 3]>> = HashMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        let exact = [tri[0], tri[1], tri[2]];
        *exact_oriented.entry(exact).or_insert(0) += 1;
        by_vertex_set
            .entry(canonical_triangle_indices(tri[0], tri[1], tri[2]))
            .or_default()
            .push(exact);
    }
    let exact_duplicate_count = exact_oriented
        .values()
        .map(|count| count.saturating_sub(1))
        .sum::<usize>();
    let same_vertex_set_duplicate_count = by_vertex_set
        .values()
        .map(|tris| tris.len().saturating_sub(1))
        .sum::<usize>();
    let mut reversed_duplicate_count = 0usize;
    for tris in by_vertex_set.values() {
        if tris.len() < 2 {
            continue;
        }
        let even = tris.iter().filter(|tri| permutation_parity(**tri)).count();
        let odd = tris.len() - even;
        if even > 0 && odd > 0 {
            reversed_duplicate_count += even.min(odd);
        }
    }
    let welded = weld_mesh_vertices(mesh, weld_tolerance_mm);
    let mut welded_sets: HashMap<[u32; 3], usize> = HashMap::new();
    for tri in welded.indices.chunks_exact(3) {
        *welded_sets
            .entry(canonical_triangle_indices(tri[0], tri[1], tri[2]))
            .or_insert(0) += 1;
    }
    let near_duplicate_after_weld = welded_sets
        .values()
        .map(|count| count.saturating_sub(1))
        .sum::<usize>();
    let (dedup_exact_mesh, exact_removed) = remove_exact_duplicate_triangles(mesh);
    let (deduped_mesh, reversed_removed, same_set_removed) =
        remove_same_vertex_set_duplicate_triangles(&dedup_exact_mesh);
    let deduped_quality = analyze_mesh_quality(&deduped_mesh);
    eprintln!(
        "  raw-topology duplicates patch={} exact_duplicate_triangles={} reversed_duplicate_triangles={} same_vertex_set_duplicates={} near_duplicate_after_weld={} duplicate_cleanup_exact_removed={} duplicate_cleanup_reversed_removed={} duplicate_cleanup_same_set_removed={} dedup_boundary_edges={} dedup_non_manifold_edges={} dedup_manifold_edge_errors={} dedup_watertight={}",
        patch_name,
        exact_duplicate_count,
        reversed_duplicate_count,
        same_vertex_set_duplicate_count,
        near_duplicate_after_weld,
        exact_removed,
        reversed_removed,
        same_set_removed,
        deduped_quality.boundary_edge_count,
        deduped_quality.non_manifold_edge_count,
        deduped_quality.manifold_edge_error_count,
        deduped_quality.watertight,
    );

    let mut area_lt_1e_12 = 0usize;
    let mut area_1e_12_to_1e_10 = 0usize;
    let mut area_1e_10_to_1e_8 = 0usize;
    let mut area_1e_8_to_1e_6 = 0usize;
    let mut area_1e_6_to_1e_4 = 0usize;
    let mut area_gt_1e_4 = 0usize;
    let mut tiny_triangles = Vec::new();
    for tri_index in 0..tri_count {
        let base = tri_index * 3;
        let i0 = mesh.indices[base] as usize;
        let i1 = mesh.indices[base + 1] as usize;
        let i2 = mesh.indices[base + 2] as usize;
        let p0 = Vec3::from_array(mesh.vertices[i0].position);
        let p1 = Vec3::from_array(mesh.vertices[i1].position);
        let p2 = Vec3::from_array(mesh.vertices[i2].position);
        let area = 0.5 * (p1 - p0).cross(p2 - p0).length();
        match area {
            a if a < 1e-12 => area_lt_1e_12 += 1,
            a if a < 1e-10 => area_1e_12_to_1e_10 += 1,
            a if a < 1e-8 => area_1e_10_to_1e_8 += 1,
            a if a < 1e-6 => area_1e_8_to_1e_6 += 1,
            a if a < 1e-4 => area_1e_6_to_1e_4 += 1,
            _ => area_gt_1e_4 += 1,
        }
        if area < 1e-4 {
            tiny_triangles.push((((p0 + p1 + p2) / 3.0), area));
        }
    }
    eprintln!(
        "  raw-topology triangle_area patch={} lt1e-12={} 1e-12_to_1e-10={} 1e-10_to_1e-8={} 1e-8_to_1e-6={} 1e-6_to_1e-4={} gt1e-4={}",
        patch_name,
        area_lt_1e_12,
        area_1e_12_to_1e_10,
        area_1e_10_to_1e_8,
        area_1e_8_to_1e_6,
        area_1e_6_to_1e_4,
        area_gt_1e_4,
    );
    log_cluster_summary(
        "raw-topology tiny_triangles",
        patch_name,
        &tiny_triangles,
        bounds_min,
        bounds_max,
    );
}

fn print_cleanup_stage(
    label: &str,
    mesh: &Mesh,
    degenerate_removed: usize,
    sliver_removed: usize,
    vertices_welded: usize,
    tiny_component_deleted: bool,
) {
    let quality = analyze_mesh_quality(mesh);
    eprintln!(
        "  aero-cleanup stage={} triangles={} vertices={} boundary_edges={} non_manifold_edges={} connected_components={} degenerate_removed={} sliver_removed={} vertices_welded={} tiny_component_deleted={} watertight={}",
        label,
        mesh.indices.len() / 3,
        mesh.vertices.len(),
        quality.boundary_edge_count,
        quality.manifold_edge_error_count,
        quality.connected_component_count,
        degenerate_removed,
        sliver_removed,
        vertices_welded,
        tiny_component_deleted,
        quality.watertight,
    );
}

fn weld_mesh_vertices(mesh: &Mesh, tolerance_mm: f32) -> Mesh {
    if mesh.vertices.is_empty() {
        return mesh.clone();
    }

    let inv_tol = if tolerance_mm > 1e-8 {
        1.0 / tolerance_mm
    } else {
        1e6
    };
    let mut buckets: HashMap<(i32, i32, i32), Vec<usize>> = HashMap::new();
    let mut new_vertices: Vec<Vertex> = Vec::new();
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

fn remove_degenerate_triangles(mesh: &Mesh, min_area_eps: f32) -> (Mesh, usize) {
    if mesh.indices.is_empty() {
        return (mesh.clone(), 0);
    }
    let mut kept = Vec::with_capacity(mesh.indices.len());
    let mut removed = 0usize;
    for tri in mesh.indices.chunks_exact(3) {
        let p0 = Vec3::from_array(mesh.vertices[tri[0] as usize].position);
        let p1 = Vec3::from_array(mesh.vertices[tri[1] as usize].position);
        let p2 = Vec3::from_array(mesh.vertices[tri[2] as usize].position);
        let area = 0.5 * (p1 - p0).cross(p2 - p0).length();
        if area > min_area_eps {
            kept.extend_from_slice(tri);
        } else {
            removed += 1;
        }
    }
    (
        Mesh {
            vertices: mesh.vertices.clone(),
            indices: kept,
        },
        removed,
    )
}

fn remove_sliver_triangles(mesh: &Mesh, max_edge_ratio: f32, min_area_eps: f32) -> (Mesh, usize) {
    if mesh.indices.is_empty() {
        return (mesh.clone(), 0);
    }
    let mut kept = Vec::with_capacity(mesh.indices.len());
    let mut removed = 0usize;
    for tri in mesh.indices.chunks_exact(3) {
        let p0 = Vec3::from_array(mesh.vertices[tri[0] as usize].position);
        let p1 = Vec3::from_array(mesh.vertices[tri[1] as usize].position);
        let p2 = Vec3::from_array(mesh.vertices[tri[2] as usize].position);
        let e0 = p0.distance(p1);
        let e1 = p1.distance(p2);
        let e2 = p2.distance(p0);
        let max_edge = e0.max(e1).max(e2);
        let min_edge = e0.min(e1).min(e2).max(1e-6);
        let area = 0.5 * (p1 - p0).cross(p2 - p0).length();
        if area > min_area_eps && max_edge / min_edge <= max_edge_ratio {
            kept.extend_from_slice(tri);
        } else {
            removed += 1;
        }
    }
    (
        Mesh {
            vertices: mesh.vertices.clone(),
            indices: kept,
        },
        removed,
    )
}

fn recompute_vertex_normals(mesh: &Mesh) -> Mesh {
    if mesh.vertices.is_empty() || mesh.indices.is_empty() {
        return mesh.clone();
    }
    let mut accum = vec![Vec3::ZERO; mesh.vertices.len()];
    for tri in mesh.indices.chunks_exact(3) {
        let i0 = tri[0] as usize;
        let i1 = tri[1] as usize;
        let i2 = tri[2] as usize;
        let p0 = Vec3::from_array(mesh.vertices[i0].position);
        let p1 = Vec3::from_array(mesh.vertices[i1].position);
        let p2 = Vec3::from_array(mesh.vertices[i2].position);
        let n = (p1 - p0).cross(p2 - p0);
        accum[i0] += n;
        accum[i1] += n;
        accum[i2] += n;
    }
    let mut vertices = mesh.vertices.clone();
    for (vertex, normal) in vertices.iter_mut().zip(accum.iter()) {
        let n = normal.normalize_or_zero();
        if n != Vec3::ZERO {
            vertex.normal = n.to_array();
        }
    }
    Mesh {
        vertices,
        indices: mesh.indices.clone(),
    }
}

fn orient_mesh_outward(mesh: &Mesh) -> Mesh {
    if mesh.vertices.is_empty() || mesh.indices.is_empty() {
        return mesh.clone();
    }

    let mut centroid = Vec3::ZERO;
    for vertex in &mesh.vertices {
        centroid += Vec3::from_array(vertex.position);
    }
    centroid /= mesh.vertices.len() as f32;

    let mut new_indices = mesh.indices.clone();
    for tri in new_indices.chunks_exact_mut(3) {
        let p0 = Vec3::from_array(mesh.vertices[tri[0] as usize].position);
        let p1 = Vec3::from_array(mesh.vertices[tri[1] as usize].position);
        let p2 = Vec3::from_array(mesh.vertices[tri[2] as usize].position);
        let normal = (p1 - p0).cross(p2 - p0);
        let face_center = (p0 + p1 + p2) / 3.0;
        if normal.dot(face_center - centroid) < 0.0 {
            tri.swap(1, 2);
        }
    }

    Mesh {
        vertices: mesh.vertices.clone(),
        indices: new_indices,
    }
}

fn build_triangle_adjacency(mesh: &Mesh, keep_triangle: &[bool]) -> Vec<Vec<usize>> {
    let tri_count = mesh.indices.len() / 3;
    let mut adjacency = vec![Vec::new(); tri_count];
    let mut edge_to_triangles: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for tri in 0..tri_count {
        if !keep_triangle[tri] {
            continue;
        }
        let base = tri * 3;
        for &(a, b) in &[
            (mesh.indices[base], mesh.indices[base + 1]),
            (mesh.indices[base + 1], mesh.indices[base + 2]),
            (mesh.indices[base + 2], mesh.indices[base]),
        ] {
            let key = if a < b { (a, b) } else { (b, a) };
            edge_to_triangles.entry(key).or_default().push(tri);
        }
    }
    for tris in edge_to_triangles.values() {
        for &a in tris {
            for &b in tris {
                if a != b {
                    adjacency[a].push(b);
                }
            }
        }
    }
    for neighbors in &mut adjacency {
        neighbors.sort_unstable();
        neighbors.dedup();
    }
    adjacency
}

fn cleanup_mesh(
    patch_name: &str,
    mesh: &Mesh,
    min_component_triangles: usize,
    drop_tiny_components: bool,
    protected_min_scale_mm: Option<f32>,
) -> (Mesh, usize) {
    if mesh.indices.len() < 3 {
        return (mesh.clone(), 0);
    }

    let global_min_scale = protected_min_scale_mm.unwrap_or(1.0).max(0.05);
    let degenerate_area_eps = (global_min_scale * global_min_scale * 1e-4).max(1e-8);
    let sliver_area_eps = (global_min_scale * global_min_scale * 5e-4).max(5e-8);
    let weld_tol = protected_min_scale_mm
        .map(|s| (s * 0.05).clamp(0.01, 0.05))
        .unwrap_or(0.01);
    print_raw_mesh_topology_diagnostic(mesh, patch_name, weld_tol);
    print_cleanup_stage("input", mesh, 0, 0, 0, false);
    let (mesh, exact_duplicate_removed) = remove_exact_duplicate_triangles(mesh);
    print_cleanup_stage(
        "post_exact_duplicate",
        &mesh,
        exact_duplicate_removed,
        0,
        0,
        false,
    );
    let (mesh, reversed_duplicate_removed, same_set_duplicate_removed) =
        remove_same_vertex_set_duplicate_triangles(&mesh);
    print_cleanup_stage(
        "post_same_vertex_set_duplicate",
        &mesh,
        exact_duplicate_removed + reversed_duplicate_removed + same_set_duplicate_removed,
        0,
        0,
        false,
    );
    let (mesh, degenerate_removed) = remove_degenerate_triangles(&mesh, degenerate_area_eps);
    print_cleanup_stage(
        "post_degenerate",
        &mesh,
        exact_duplicate_removed
            + reversed_duplicate_removed
            + same_set_duplicate_removed
            + degenerate_removed,
        0,
        0,
        false,
    );
    let (mesh, sliver_removed) = remove_sliver_triangles(&mesh, 30.0, sliver_area_eps);
    print_cleanup_stage(
        "post_sliver",
        &mesh,
        exact_duplicate_removed
            + reversed_duplicate_removed
            + same_set_duplicate_removed
            + degenerate_removed,
        sliver_removed,
        0,
        false,
    );
    let tri_count = mesh.indices.len() / 3;
    let mut removed = exact_duplicate_removed
        + reversed_duplicate_removed
        + same_set_duplicate_removed
        + degenerate_removed
        + sliver_removed;
    if tri_count == 0 {
        return (
            Mesh {
                vertices: Vec::new(),
                indices: Vec::new(),
            },
            removed,
        );
    }
    let mut keep_triangle = vec![true; tri_count];

    let mut tiny_component_deleted = false;
    if drop_tiny_components {
        let adjacency = build_triangle_adjacency(&mesh, &keep_triangle);
        let (global_min, global_max) = mesh_bounds(&mesh);
        let global_diag = (Vec3::from_array(global_max) - Vec3::from_array(global_min))
            .length()
            .max(1.0);
        let protected_scale = protected_min_scale_mm
            .unwrap_or(global_diag * 0.01)
            .max(0.05);
        let min_component_area = (global_diag * global_diag * 1e-5)
            .min((protected_scale * protected_scale * 0.25).max(1e-5))
            .max(1e-5);
        let min_component_diag = (global_diag * 0.01).min(protected_scale).max(0.02);

        let mut visited = vec![false; tri_count];
        for start in 0..tri_count {
            if visited[start] || !keep_triangle[start] {
                continue;
            }
            let mut stack = vec![start];
            let mut component = Vec::new();
            visited[start] = true;
            while let Some(tri) = stack.pop() {
                component.push(tri);
                for &next in &adjacency[tri] {
                    if !visited[next] && keep_triangle[next] {
                        visited[next] = true;
                        stack.push(next);
                    }
                }
            }
            let mut area_sum = 0.0f32;
            let mut cmin = Vec3::splat(f32::MAX);
            let mut cmax = Vec3::splat(f32::MIN);
            for &tri in &component {
                area_sum += triangle_area(&mesh, tri);
                let base = tri * 3;
                for offset in 0..3 {
                    let p = Vec3::from_array(
                        mesh.vertices[mesh.indices[base + offset] as usize].position,
                    );
                    cmin = cmin.min(p);
                    cmax = cmax.max(p);
                }
            }
            let diag = (cmax - cmin).length();
            if component.len() < min_component_triangles
                || area_sum < min_component_area
                || diag < min_component_diag
            {
                for tri in component {
                    if keep_triangle[tri] {
                        keep_triangle[tri] = false;
                        removed += 1;
                        tiny_component_deleted = true;
                    }
                }
            }
        }
    }

    let mut new_indices = Vec::with_capacity(mesh.indices.len());
    let mut used_vertices = HashSet::new();
    for tri in 0..tri_count {
        if !keep_triangle[tri] {
            continue;
        }
        for offset in 0..3 {
            let idx = mesh.indices[tri * 3 + offset];
            new_indices.push(idx);
            used_vertices.insert(idx as usize);
        }
    }

    if new_indices.is_empty() {
        return (
            Mesh {
                vertices: Vec::new(),
                indices: Vec::new(),
            },
            removed,
        );
    }

    let mut remap = HashMap::new();
    let mut new_vertices = Vec::with_capacity(used_vertices.len());
    for old_idx in 0..mesh.vertices.len() {
        if used_vertices.contains(&old_idx) {
            let new_idx = new_vertices.len() as u32;
            remap.insert(old_idx as u32, new_idx);
            new_vertices.push(mesh.vertices[old_idx]);
        }
    }
    for idx in &mut new_indices {
        *idx = *remap.get(idx).unwrap_or(idx);
    }

    let cleaned = Mesh {
        vertices: new_vertices,
        indices: new_indices,
    };
    print_cleanup_stage(
        "post_component_filter",
        &cleaned,
        exact_duplicate_removed
            + reversed_duplicate_removed
            + same_set_duplicate_removed
            + degenerate_removed,
        sliver_removed,
        0,
        tiny_component_deleted,
    );
    let pre_weld_vertices = cleaned.vertices.len();
    let cleaned = weld_mesh_vertices(&cleaned, weld_tol);
    let welded_vertices = pre_weld_vertices.saturating_sub(cleaned.vertices.len());
    print_cleanup_stage(
        "post_weld",
        &cleaned,
        exact_duplicate_removed
            + reversed_duplicate_removed
            + same_set_duplicate_removed
            + degenerate_removed,
        sliver_removed,
        welded_vertices,
        tiny_component_deleted,
    );
    let (cleaned, post_weld_removed) = remove_degenerate_triangles(&cleaned, degenerate_area_eps);
    removed += post_weld_removed;
    print_cleanup_stage(
        "post_weld_degenerate",
        &cleaned,
        exact_duplicate_removed
            + reversed_duplicate_removed
            + same_set_duplicate_removed
            + degenerate_removed
            + post_weld_removed,
        sliver_removed,
        welded_vertices,
        tiny_component_deleted,
    );
    let cleaned = orient_mesh_outward(&cleaned);
    print_cleanup_stage(
        "post_orient",
        &cleaned,
        exact_duplicate_removed
            + reversed_duplicate_removed
            + same_set_duplicate_removed
            + degenerate_removed
            + post_weld_removed,
        sliver_removed,
        welded_vertices,
        tiny_component_deleted,
    );
    let cleaned = recompute_vertex_normals(&cleaned);
    print_cleanup_stage(
        "post_normals",
        &cleaned,
        exact_duplicate_removed
            + reversed_duplicate_removed
            + same_set_duplicate_removed
            + degenerate_removed
            + post_weld_removed,
        sliver_removed,
        welded_vertices,
        tiny_component_deleted,
    );

    (cleaned, removed)
}

#[derive(Clone, Copy)]
struct MeshComponentStats {
    center: Vec3,
    size: Vec3,
    area: f32,
}

fn mesh_component_stats(mesh: &Mesh) -> Vec<MeshComponentStats> {
    if mesh.indices.is_empty() {
        return Vec::new();
    }
    let tri_count = mesh.indices.len() / 3;
    let keep_triangle = vec![true; tri_count];
    let adjacency = build_triangle_adjacency(mesh, &keep_triangle);
    let mut visited = vec![false; tri_count];
    let mut components = Vec::new();
    for start in 0..tri_count {
        if visited[start] {
            continue;
        }
        let mut stack = vec![start];
        visited[start] = true;
        let mut area = 0.0f32;
        let mut cmin = Vec3::splat(f32::MAX);
        let mut cmax = Vec3::splat(f32::MIN);
        while let Some(tri) = stack.pop() {
            area += triangle_area(mesh, tri);
            let base = tri * 3;
            for offset in 0..3 {
                let p =
                    Vec3::from_array(mesh.vertices[mesh.indices[base + offset] as usize].position);
                cmin = cmin.min(p);
                cmax = cmax.max(p);
            }
            for &next in &adjacency[tri] {
                if !visited[next] {
                    visited[next] = true;
                    stack.push(next);
                }
            }
        }
        components.push(MeshComponentStats {
            center: (cmin + cmax) * 0.5,
            size: cmax - cmin,
            area,
        });
    }
    components
}

fn duplicate_shell_count(mesh: &Mesh) -> usize {
    let components = mesh_component_stats(mesh);
    let mut duplicates = 0usize;
    for i in 0..components.len() {
        for j in (i + 1)..components.len() {
            let a = components[i];
            let b = components[j];
            let center_close = a.center.distance(b.center) <= 0.25;
            let size_close = (a.size - b.size).abs().max_element() <= 0.25;
            let area_scale = a.area.abs().max(b.area.abs()).max(1e-3);
            let area_close = ((a.area - b.area).abs() / area_scale) <= 0.05;
            if center_close && size_close && area_close {
                duplicates += 1;
            }
        }
    }
    duplicates
}

fn possible_self_intersection_count(mesh: &Mesh, max_triangles: usize) -> usize {
    let tri_count = (mesh.indices.len() / 3).min(max_triangles);
    let mut tri_bounds = Vec::with_capacity(tri_count);
    for tri in 0..tri_count {
        let base = tri * 3;
        let p0 = Vec3::from_array(mesh.vertices[mesh.indices[base] as usize].position);
        let p1 = Vec3::from_array(mesh.vertices[mesh.indices[base + 1] as usize].position);
        let p2 = Vec3::from_array(mesh.vertices[mesh.indices[base + 2] as usize].position);
        tri_bounds.push((
            p0.min(p1).min(p2),
            p0.max(p1).max(p2),
            [
                mesh.indices[base],
                mesh.indices[base + 1],
                mesh.indices[base + 2],
            ],
        ));
    }
    let mut candidates = 0usize;
    for i in 0..tri_bounds.len() {
        for j in (i + 1)..tri_bounds.len() {
            let shares_vertex = tri_bounds[i].2.iter().any(|a| tri_bounds[j].2.contains(a));
            if shares_vertex {
                continue;
            }
            let overlap = tri_bounds[i].0.cmple(tri_bounds[j].1).all()
                && tri_bounds[j].0.cmple(tri_bounds[i].1).all();
            if overlap {
                candidates += 1;
            }
        }
    }
    candidates
}

fn mesh_bounds(mesh: &Mesh) -> ([f32; 3], [f32; 3]) {
    if mesh.vertices.is_empty() {
        return ([0.0; 3], [0.0; 3]);
    }
    let mut min = Vec3::splat(f32::MAX);
    let mut max = Vec3::splat(f32::MIN);
    for vertex in &mesh.vertices {
        let p = Vec3::from_array(vertex.position);
        min = min.min(p);
        max = max.max(p);
    }
    (min.to_array(), max.to_array())
}

fn surface_deviation_stats(mesh: &Mesh, sdf: &dyn Sdf) -> (f32, f32) {
    if mesh.vertices.is_empty() {
        return (0.0, 0.0);
    }
    let mut sum = 0.0f32;
    let mut max_dev = 0.0f32;
    for vertex in &mesh.vertices {
        let p = Vec3::from_array(vertex.position);
        let d = sdf.distance(p).abs();
        sum += d;
        max_dev = max_dev.max(d);
    }
    (sum / mesh.vertices.len() as f32, max_dev)
}

fn patch_adaptive_export_settings(
    contributors: &[AeroExportPart],
    settings: &AeroExportSettings,
    quality: MeshQuality,
    bounds_min: Vec3,
    bounds_max: Vec3,
) -> AdaptiveExportSettings {
    let mut octree = settings.adaptive_octree;
    let patch_span = (bounds_max - bounds_min).abs().max_element().max(1.0);
    let aircraft_scale_min_cell = match quality {
        MeshQuality::Draft => (patch_span / 120.0).max(2.0),
        MeshQuality::Normal => (patch_span / 220.0).max(1.5),
        MeshQuality::Fine => (patch_span / 320.0).max(0.8),
        MeshQuality::Ultra => (patch_span / 420.0).max(0.4),
    };
    octree.min_cell_size_mm = octree.min_cell_size_mm.max(aircraft_scale_min_cell);
    octree.surface_band_mm = octree.surface_band_mm.max(octree.min_cell_size_mm * 1.5);
    let mut preserve_feature = false;
    let mut min_export_scale = f32::MAX;
    let mut feature_strength = 0.0f32;
    let mut has_leading_edge = false;
    let mut has_trailing_edge = false;
    let mut has_junction = false;
    let mut has_inlet_lip = false;
    let mut has_nozzle_exit = false;
    let mut has_fairing_break = false;
    for contributor in contributors {
        if contributor.preserve_feature {
            preserve_feature = true;
        }
        if contributor.min_export_scale_mm > 0.0 {
            min_export_scale = min_export_scale.min(contributor.min_export_scale_mm);
        }
        feature_strength = feature_strength.max(contributor.feature_protection_strength);
        for tag in &contributor.feature_tags {
            match tag.as_str() {
                "leading_edge" => has_leading_edge = true,
                "trailing_edge" => has_trailing_edge = true,
                "wing_body_junction" | "tail_body_junction" | "junction" => has_junction = true,
                "inlet_lip" => has_inlet_lip = true,
                "nozzle_exit" => has_nozzle_exit = true,
                "fairing_break" | "discontinuity" => has_fairing_break = true,
                _ => {}
            }
        }
    }
    if min_export_scale.is_finite() && preserve_feature {
        let protected_floor = min_export_scale.max(0.05);
        if patch_span <= 250.0 {
            octree.min_cell_size_mm = octree.min_cell_size_mm.min(protected_floor);
        } else {
            octree.min_cell_size_mm = octree
                .min_cell_size_mm
                .min((protected_floor * 2.0).max(0.1));
        }
        octree.error_tolerance_mm = octree
            .error_tolerance_mm
            .min((protected_floor * 0.75).max(0.05));
    }
    if preserve_feature {
        let strength = feature_strength.max(1.0);
        octree.curvature_refine_threshold *= (1.0 / (1.0 + 0.4 * strength)).max(0.2);
        octree.surface_band_mm = octree.surface_band_mm.max(octree.min_cell_size_mm * 2.0);
        octree.max_depth = octree.max_depth.saturating_add(strength.round() as u32);
    }
    if has_leading_edge || has_inlet_lip {
        octree.min_cell_size_mm *= 0.75;
        octree.curvature_refine_threshold *= 0.8;
    }
    if has_trailing_edge {
        octree.min_cell_size_mm *= 0.5;
        octree.error_tolerance_mm *= 0.6;
    }
    if has_junction {
        octree.surface_band_mm *= 1.25;
        octree.curvature_refine_threshold *= 0.85;
    }
    if has_nozzle_exit {
        octree.min_cell_size_mm *= 0.7;
    }
    if has_fairing_break {
        octree.curvature_refine_threshold *= 0.75;
    }
    octree.min_cell_size_mm = octree.min_cell_size_mm.max(0.05);
    octree.error_tolerance_mm = octree.error_tolerance_mm.max(0.01);
    AdaptiveExportSettings {
        quality,
        smooth_normals: settings.smooth_normals,
        octree,
        prefer_dual_contouring: settings.use_adaptive_extraction,
    }
}

fn patch_preflight_target_cell_mm(
    contributors: &[AeroExportPart],
    settings: &AeroExportSettings,
    quality: MeshQuality,
    bounds_min: Vec3,
    bounds_max: Vec3,
) -> f32 {
    let patch_span = (bounds_max - bounds_min).abs().max_element().max(1.0);
    let adaptive =
        patch_adaptive_export_settings(contributors, settings, quality, bounds_min, bounds_max);
    let base_hint = match quality {
        MeshQuality::Draft => (patch_span / 100.0).max(6.0),
        MeshQuality::Normal => (patch_span / 150.0).max(4.0),
        MeshQuality::Fine => (patch_span / 220.0).max(2.5),
        MeshQuality::Ultra => (patch_span / 300.0).max(1.5),
    };
    let preserve_feature = contributors.iter().any(|c| c.preserve_feature);
    let mut has_leading_edge = false;
    let mut has_trailing_edge = false;
    let mut has_junction = false;
    let mut has_inlet_lip = false;
    for contributor in contributors {
        for tag in &contributor.feature_tags {
            match tag.as_str() {
                "leading_edge" => has_leading_edge = true,
                "trailing_edge" => has_trailing_edge = true,
                "wing_body_junction" | "tail_body_junction" | "junction" => has_junction = true,
                "inlet_lip" => has_inlet_lip = true,
                _ => {}
            }
        }
    }
    let protected_target = if preserve_feature {
        let mut feature_factor = 0.86f32;
        if has_leading_edge {
            feature_factor = feature_factor.min(0.82);
        }
        if has_trailing_edge {
            feature_factor = feature_factor.min(0.72);
        }
        if has_junction {
            feature_factor = feature_factor.min(0.74);
        }
        if has_inlet_lip {
            feature_factor = feature_factor.min(0.78);
        }
        if (has_leading_edge as u8
            + has_trailing_edge as u8
            + has_junction as u8
            + has_inlet_lip as u8)
            >= 2
        {
            feature_factor *= 0.96;
        }
        let feature_cell = (adaptive.octree.min_cell_size_mm * 3.0).max(base_hint * feature_factor);
        let feature_cap = match quality {
            MeshQuality::Draft => {
                if has_trailing_edge || has_junction || has_leading_edge || has_inlet_lip {
                    5.0
                } else {
                    5.5
                }
            }
            MeshQuality::Normal => 3.5,
            MeshQuality::Fine => 2.0,
            MeshQuality::Ultra => 1.0,
        };
        feature_cell.min(feature_cap)
    } else {
        base_hint.max(adaptive.octree.min_cell_size_mm)
    };
    if preserve_feature {
        base_hint.max(protected_target).max(0.5)
    } else {
        protected_target.max(0.5)
    }
}

fn patch_meshing_notes(
    contributors: &[AeroExportPart],
    manifest: &AeroPatchManifest,
) -> Vec<String> {
    let mut notes = Vec::new();
    let has_internal = contributors
        .iter()
        .any(|c| c.aero_role == "flow_path_internal");
    let has_trailing = contributors
        .iter()
        .flat_map(|c| c.feature_tags.iter())
        .any(|tag| tag == "trailing_edge");
    let has_leading = contributors
        .iter()
        .flat_map(|c| c.feature_tags.iter())
        .any(|tag| tag == "leading_edge");
    let has_junction = contributors
        .iter()
        .flat_map(|c| c.feature_tags.iter())
        .any(|tag| tag == "wing_body_junction" || tag == "tail_body_junction" || tag == "junction");
    let has_inlet = contributors
        .iter()
        .flat_map(|c| c.feature_tags.iter())
        .any(|tag| tag == "inlet_lip" || tag == "nozzle_exit");

    if has_internal {
        notes.push("Internal wetted surface: keep this patch in the fluid region and confirm reachability seeds.".to_string());
    } else {
        notes.push("External wetted wall: use wall boundary conditions and preserve outer mold line fidelity.".to_string());
    }
    if has_trailing {
        notes.push("Thin trailing-edge feature detected: enable edge feature extraction and local refinement.".to_string());
    }
    if has_leading {
        notes.push(
            "Leading-edge feature detected: tighten surface refinement near high curvature."
                .to_string(),
        );
    }
    if has_junction {
        notes.push(
            "Body junction detected: use extra local refinement to avoid junction faceting."
                .to_string(),
        );
    }
    if has_inlet {
        notes.push(
            "Inlet/nozzle lip detected: preserve sharp lips during snappyHexMesh refinement."
                .to_string(),
        );
    }
    if let Some(min_feature) = manifest.reported_min_feature_size_mm {
        let recommended_cell = min_feature * 0.5;
        notes.push(format!(
            "Reported minimum feature size is {:.3} mm; target cell should be <= {:.3} mm to resolve it with two cells across.",
            min_feature, recommended_cell
        ));
    }
    if !manifest.watertight {
        notes.push(
            "Patch is not watertight; inspect and repair before final CFD meshing.".to_string(),
        );
    }
    if manifest.max_triangle_aspect_ratio > 25.0 {
        notes.push("High triangle aspect ratio detected; consider tighter minimum cell size or feature protection.".to_string());
    }
    notes
}

fn should_use_fast_uniform_patch_path(
    contributors: &[AeroExportPart],
    settings: &AeroExportSettings,
    patch_span_mm: f32,
    estimated_uniform_resolution: u32,
) -> bool {
    if contributors.iter().any(|c| c.disable_auto_fast_path) {
        return false;
    }
    if patch_span_mm < 300.0 {
        return false;
    }
    if estimated_uniform_resolution > 220 {
        return false;
    }
    let first_role = match contributors.first() {
        Some(c) => c.aero_role.as_str(),
        None => return false,
    };
    if matches!(
        first_role,
        "internal_structure" | "construction_only" | "ignore"
    ) {
        return false;
    }
    let homogeneous_role = contributors.iter().all(|c| c.aero_role == first_role);
    if !homogeneous_role {
        return false;
    }

    if settings.fast_cfd_mode {
        return true;
    }

    // Large homogeneous external patches are the main pathological case for the
    // slower adaptive path. Allow the scout/narrow-band route automatically for
    // those patches, while keeping internal-flow patches on the more defensive
    // path unless the caller explicitly requests fast CFD mode.
    if first_role != "outer_mold_line" {
        return false;
    }

    patch_span_mm >= 450.0 || estimated_uniform_resolution >= 140
}

fn write_openfoam_helpers(output_dir: &Path, manifest: &AeroExportManifest) -> Result<(), String> {
    let geometry_lines = {
        let mut lines = vec!["geometry".to_string(), "{".to_string()];
        for patch in &manifest.patches {
            lines.push(format!("    {}.stl", patch.name));
            lines.push("    {".to_string());
            lines.push("        type triSurfaceMesh;".to_string());
            lines.push(format!("        name {};", patch.name));
            lines.push("    }".to_string());
        }
        lines.push("}".to_string());
        lines
    };
    fs::write(
        output_dir.join("openfoam_geometry_dict.txt"),
        geometry_lines.join("\n"),
    )
    .map_err(|e| format!("Failed to write OpenFOAM geometry helper: {}", e))?;

    let mut boundary_lines = vec!["# OpenFOAM Boundary Summary".to_string(), "".to_string()];
    for patch in &manifest.patches {
        boundary_lines.push(format!(
            "- `{}`: file `{}`, watertight: {}, notes: {}",
            patch.name,
            patch.file,
            patch.watertight,
            if patch.recommended_meshing_notes.is_empty() {
                "none".to_string()
            } else {
                patch.recommended_meshing_notes.join(" | ")
            }
        ));
    }
    fs::write(
        output_dir.join("openfoam_boundary_summary.md"),
        boundary_lines.join("\n"),
    )
    .map_err(|e| format!("Failed to write OpenFOAM boundary summary: {}", e))?;

    let notes_lines = vec![
        "# snappyHexMesh Notes".to_string(),
        "".to_string(),
        "- Import STL patches from `constant/triSurface/` or point `geometry` at the files in this export folder.".to_string(),
        "- Run `surfaceFeatureExtract` on the generated feature helper config before snappyHexMesh if sharp edges matter.".to_string(),
        "- Start with feature edge extraction enabled for patches with leading/trailing edges or inlet lips.".to_string(),
        "- Tighten local refinement around patches flagged with high aspect ratio or non-watertight warnings.".to_string(),
        "- Confirm internal flow patches are reachable from intended inlet/outlet seeds before solving.".to_string(),
    ];
    fs::write(
        output_dir.join("snappyHexMesh_notes.md"),
        notes_lines.join("\n"),
    )
    .map_err(|e| format!("Failed to write snappyHexMesh notes: {}", e))?;

    let template_root = output_dir.join("openfoam_template");
    let tri_surface_dir = template_root.join("constant").join("triSurface");
    let system_dir = template_root.join("system");
    fs::create_dir_all(&tri_surface_dir)
        .map_err(|e| format!("Failed to create OpenFOAM triSurface template dir: {}", e))?;
    fs::create_dir_all(&system_dir)
        .map_err(|e| format!("Failed to create OpenFOAM system template dir: {}", e))?;

    for patch in &manifest.patches {
        let src = output_dir.join(&patch.file);
        let dst = tri_surface_dir.join(&patch.file);
        fs::copy(&src, &dst).map_err(|e| {
            format!(
                "Failed to populate OpenFOAM triSurface template for '{}': {}",
                patch.name, e
            )
        })?;
    }
    fs::write(
        system_dir.join("geometryDict.snippet"),
        geometry_lines.join("\n"),
    )
    .map_err(|e| format!("Failed to write OpenFOAM geometry snippet: {}", e))?;
    let mut sfe_lines = Vec::new();
    for patch in &manifest.patches {
        sfe_lines.push(format!("{}.stl", patch.name));
        sfe_lines.push("{".to_string());
        sfe_lines.push("    extractionMethod extractFromSurface;".to_string());
        sfe_lines.push("    extractFromSurfaceCoeffs".to_string());
        sfe_lines.push("    {".to_string());
        sfe_lines.push("        includedAngle 150;".to_string());
        sfe_lines.push("    }".to_string());
        sfe_lines.push("    writeObj yes;".to_string());
        sfe_lines.push("}".to_string());
    }
    fs::write(
        system_dir.join("surfaceFeatureExtractDict"),
        sfe_lines.join("\n"),
    )
    .map_err(|e| format!("Failed to write surfaceFeatureExtract helper: {}", e))?;
    fs::write(
        system_dir.join("boundary_conditions.md"),
        boundary_lines.join("\n"),
    )
    .map_err(|e| format!("Failed to write OpenFOAM boundary notes: {}", e))?;

    Ok(())
}

fn write_combined_patch_obj(
    output_dir: &Path,
    patch_meshes: &[(String, Mesh)],
) -> Result<(), String> {
    let mut obj = String::new();
    let mut vertex_offset = 0u32;
    for (name, mesh) in patch_meshes {
        obj.push_str(&format!("o {}\n", name));
        for vertex in &mesh.vertices {
            obj.push_str(&format!(
                "v {} {} {}\n",
                vertex.position[0], vertex.position[1], vertex.position[2]
            ));
        }
        for vertex in &mesh.vertices {
            obj.push_str(&format!(
                "vn {} {} {}\n",
                vertex.normal[0], vertex.normal[1], vertex.normal[2]
            ));
        }
        for tri in mesh.indices.chunks_exact(3) {
            let a = tri[0] + 1 + vertex_offset;
            let b = tri[1] + 1 + vertex_offset;
            let c = tri[2] + 1 + vertex_offset;
            obj.push_str(&format!("f {0}//{0} {1}//{1} {2}//{2}\n", a, b, c));
        }
        vertex_offset += mesh.vertices.len() as u32;
    }
    fs::write(output_dir.join("combined_patches.obj"), obj)
        .map_err(|e| format!("Failed to write combined patch OBJ: {}", e))?;
    Ok(())
}

fn merge_patch_meshes(patch_meshes: &[(String, Mesh)]) -> Mesh {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut vertex_offset = 0u32;
    for (_, mesh) in patch_meshes {
        vertices.extend(mesh.vertices.iter().copied());
        for index in &mesh.indices {
            indices.push(*index + vertex_offset);
        }
        vertex_offset += mesh.vertices.len() as u32;
    }
    Mesh { vertices, indices }
}

fn export_patch(
    patch_name: &str,
    contributors: &[AeroExportPart],
    output_dir: &Path,
    settings: &AeroExportSettings,
) -> Result<PatchExportStats, String> {
    let cache_key = if settings.fast_cfd_mode {
        None
    } else {
        Some(patch_cache_key(patch_name, contributors, settings))
    };
    if let Some(cache_key) = cache_key {
        if let Some(existing) = aero_patch_cache()
            .lock()
            .ok()
            .and_then(|c| c.get(&cache_key).cloned())
        {
            let path = output_dir.join(format!("{patch_name}.stl"));
            if !path.exists() {
                export_stl(
                    &existing.mesh,
                    path.to_str().ok_or("invalid aero export path")?,
                )
                .map_err(|e| format!("Failed to write cached patch '{}': {}", patch_name, e))?;
            }
            if settings.write_obj_per_patch {
                let obj_path = output_dir.join(format!("{patch_name}.obj"));
                if !obj_path.exists() {
                    export_obj(
                        &existing.mesh,
                        obj_path.to_str().ok_or("invalid aero export path")?,
                    )
                    .map_err(|e| {
                        format!("Failed to write cached OBJ patch '{}': {}", patch_name, e)
                    })?;
                }
            }
            let mut cached = existing;
            cached.manifest.cache_hit = true;
            return Ok(cached);
        }
    }

    let patch_sdf = if let Some(cache_key) = cache_key {
        if let Some(existing) = aero_patch_union_cache()
            .lock()
            .ok()
            .and_then(|c| c.get(&cache_key).cloned())
        {
            existing
        } else {
            let sdfs: Vec<_> = contributors.iter().map(|p| Arc::clone(&p.sdf)).collect();
            let unioned = union_all(&sdfs)
                .ok_or_else(|| format!("Patch '{}' had no geometry", patch_name))?;
            if let Ok(mut cache) = aero_patch_union_cache().lock() {
                cache.insert(cache_key, Arc::clone(&unioned));
                if cache.len() > 64 {
                    if let Some(first_key) = cache.keys().next().copied() {
                        cache.remove(&first_key);
                    }
                }
            }
            unioned
        }
    } else {
        let sdfs: Vec<_> = contributors.iter().map(|p| Arc::clone(&p.sdf)).collect();
        union_all(&sdfs).ok_or_else(|| format!("Patch '{}' had no geometry", patch_name))?
    };
    let (bounds_min, bounds_max) =
        contributor_bounds(contributors).unwrap_or_else(|| auto_bounds(patch_sdf.as_ref()));
    let target_cell = patch_preflight_target_cell_mm(
        contributors,
        settings,
        settings.quality,
        bounds_min,
        bounds_max,
    );
    let effective_target_cell = if settings.uniform_reference_mode {
        settings.uniform_reference_target_cell_mm.max(0.02)
    } else {
        target_cell
    };
    let reported_min_feature_size_mm = patch_reported_min_feature_size_mm(contributors);
    let estimated_uniform_resolution =
        (((bounds_max - bounds_min).max_element() / effective_target_cell).ceil() as u32)
            .clamp(8, 4096);
    let estimated_uniform_voxels = estimated_uniform_resolution as u64
        * estimated_uniform_resolution as u64
        * estimated_uniform_resolution as u64;
    eprintln!(
        "[aero] meshing patch '{}' (contributors: {}, bounds: {:.1} x {:.1} x {:.1} mm, target cell: {:.3} mm, est uniform grid: {}^3)",
        patch_name,
        contributors.len(),
        (bounds_max.x - bounds_min.x).abs(),
        (bounds_max.y - bounds_min.y).abs(),
        (bounds_max.z - bounds_min.z).abs(),
        effective_target_cell,
        estimated_uniform_resolution,
    );
    if estimated_uniform_voxels > settings.max_patch_preflight_voxels {
        return Err(format!(
            "Patch '{}' aborted before meshing: estimated equivalent uniform grid is {} voxels, above limit {}. Increase --aero-max-patch-voxels or coarsen aero export settings.",
            patch_name, estimated_uniform_voxels, settings.max_patch_preflight_voxels
        ));
    }
    let t0 = Instant::now();
    let counted_patch_sdf = Arc::new(CountedSdf::new(Arc::clone(&patch_sdf)));
    let counted_patch_ref: &dyn Sdf = counted_patch_sdf.as_ref();
    sdf_profile_reset();
    sdf_profile_set_enabled(true);
    let mut mesh = Mesh {
        vertices: Vec::new(),
        indices: Vec::new(),
    };
    let mut selected_backend = "uniform_export".to_string();
    let mut backend_reason = "uniform export path".to_string();
    let mut fast_patch_mode = false;
    let mesh_build_t0 = Instant::now();
    let patch_span_mm = (bounds_max - bounds_min).abs().max_element();
    if settings.uniform_reference_mode {
        let mut refine_regions =
            feature_refine_regions(contributors, bounds_min, bounds_max, effective_target_cell);
        let auto_regions = auto_clearance_refine_regions(
            contributors,
            bounds_min,
            bounds_max,
            effective_target_cell,
        );
        let path_regions = path_refine_regions(contributors);
        refine_regions.extend(auto_regions.iter().copied());
        refine_regions.extend(path_regions.iter().copied());
        let local_refine_regions = to_local_refine_regions(&refine_regions);
        selected_backend = "uniform_reference_grid".to_string();
        backend_reason = if refine_regions.is_empty() {
            "forced dense uniform grid sampling over full patch bounds".to_string()
        } else {
            format!(
                "forced dense adaptive field sampling with {} local refinement region(s)",
                refine_regions.len()
            )
        };
        if !refine_regions.is_empty() {
            eprintln!(
                "[aero] uniform reference patch '{}': {} refinement region(s)",
                patch_name,
                refine_regions.len()
            );
            if !path_regions.is_empty() {
                let path_specs = collect_refinement_path_specs(contributors);
                eprintln!(
                    "[aero]   path refinement regions: {} segment box(es) from {} analytic path(s), first target={:.3} mm, forward_extend={:.1} mm, aft_extend={:.1} mm",
                    path_regions.len(),
                    path_specs.len(),
                    path_regions[0].target_cell_mm,
                    path_specs.first().map(|s| s.2).unwrap_or(0.0),
                    path_specs.first().map(|s| s.3).unwrap_or(0.0),
                );
            }
            for (i, region) in refine_regions.iter().enumerate() {
                eprintln!(
                    "[aero]   region {}: min=({:.1}, {:.1}, {:.1}) max=({:.1}, {:.1}, {:.1}) target={:.3} mm",
                    i + 1,
                    region.min.x,
                    region.min.y,
                    region.min.z,
                    region.max.x,
                    region.max.y,
                    region.max.z,
                    region.target_cell_mm
                );
            }
        }
        mesh = if local_refine_regions.is_empty() {
            build_export_mesh_uniform_reference(
                counted_patch_ref,
                bounds_min,
                bounds_max,
                effective_target_cell,
                settings.smooth_normals,
            )
        } else {
            crate::mesh::adaptive_mc::extract_mesh_adaptive_local_regions(
                counted_patch_ref,
                bounds_min,
                bounds_max,
                effective_target_cell,
                &local_refine_regions,
                settings.smooth_normals,
            )
        };
    } else if should_use_fast_uniform_patch_path(
        contributors,
        settings,
        patch_span_mm,
        estimated_uniform_resolution,
    ) {
        fast_patch_mode = true;
        let scout_blocks = scout_surface_macroblocks(
            counted_patch_ref,
            bounds_min,
            bounds_max,
            target_cell,
            contributors,
        );
        let scout_block_count = scout_blocks.len();
        eprintln!(
            "[aero] scout patch '{}': {} macroblock(s) from metadata bounds",
            patch_name, scout_block_count
        );
        if scout_blocks.is_empty() {
            selected_backend = "uniform_fast_patch".to_string();
            backend_reason =
                "scout found no active macroblocks; fell back to proven uniform extractor"
                    .to_string();
            mesh = build_export_mesh_with_target_cell(
                counted_patch_ref,
                bounds_min,
                bounds_max,
                effective_target_cell,
                settings.quality,
                settings.smooth_normals,
            );
        } else {
            selected_backend = "uniform_scout_narrow_band".to_string();
            backend_reason = format!(
                "large external OML patch meshed only in {} scout-derived macroblock(s)",
                scout_blocks.len()
            );
            let mut merged = Mesh {
                vertices: Vec::new(),
                indices: Vec::new(),
            };
            for block in scout_blocks {
                let block_mesh = build_export_mesh_with_target_cell(
                    counted_patch_ref,
                    block.mesh_min,
                    block.mesh_max,
                    effective_target_cell,
                    settings.quality,
                    settings.smooth_normals,
                );
                let owned_mesh = filter_mesh_to_core_axis_range(
                    &block_mesh,
                    block.axis,
                    block.core_min_axis,
                    block.core_max_axis,
                );
                merge_mesh_into(&mut merged, &owned_mesh);
            }
            let refine_regions =
                feature_refine_regions(contributors, bounds_min, bounds_max, effective_target_cell);
            if !refine_regions.is_empty() {
                let coarse_only = filter_mesh_excluding_regions(&merged, &refine_regions);
                let mut refined_merged = Mesh {
                    vertices: Vec::new(),
                    indices: Vec::new(),
                };
                merge_mesh_into(&mut refined_merged, &coarse_only);
                for region in &refine_regions {
                    let region_mesh = build_export_mesh_with_target_cell(
                        counted_patch_ref,
                        region.min,
                        region.max,
                        region.target_cell_mm,
                        settings.quality,
                        settings.smooth_normals,
                    );
                    merge_mesh_into(&mut refined_merged, &region_mesh);
                }
                merged = refined_merged;
                backend_reason = format!(
                    "large external OML patch meshed in {} scout block(s) with {} local feature refinement region(s)",
                    scout_block_count,
                    refine_regions.len()
                );
            }
            mesh = weld_mesh_vertices(&merged, (effective_target_cell * 0.05).max(0.02));
            if mesh.vertices.is_empty() || mesh.indices.is_empty() {
                selected_backend = "uniform_fast_patch".to_string();
                backend_reason =
                    "scout narrow-band meshing produced an empty mesh; fell back to uniform extractor"
                        .to_string();
                mesh = build_export_mesh_with_target_cell(
                    counted_patch_ref,
                    bounds_min,
                    bounds_max,
                    effective_target_cell,
                    settings.quality,
                    settings.smooth_normals,
                );
            }
        }
    } else {
        let quality_sequence = match settings.quality {
            MeshQuality::Draft => [
                MeshQuality::Draft,
                MeshQuality::Normal,
                MeshQuality::Fine,
                MeshQuality::Ultra,
            ],
            MeshQuality::Normal => [
                MeshQuality::Normal,
                MeshQuality::Fine,
                MeshQuality::Ultra,
                MeshQuality::Ultra,
            ],
            MeshQuality::Fine => [
                MeshQuality::Fine,
                MeshQuality::Ultra,
                MeshQuality::Ultra,
                MeshQuality::Ultra,
            ],
            MeshQuality::Ultra => [
                MeshQuality::Ultra,
                MeshQuality::Ultra,
                MeshQuality::Ultra,
                MeshQuality::Ultra,
            ],
        };
        for quality in quality_sequence {
            let adaptive_result = if settings.use_adaptive_extraction {
                Some(build_export_mesh_adaptive(
                    counted_patch_ref,
                    bounds_min,
                    bounds_max,
                    patch_adaptive_export_settings(
                        contributors,
                        settings,
                        quality,
                        bounds_min,
                        bounds_max,
                    ),
                ))
            } else {
                None
            };
            mesh = if let Some(result) = adaptive_result {
                selected_backend = result.backend.to_string();
                backend_reason = result.reason;
                result.mesh
            } else {
                selected_backend = "uniform_export".to_string();
                backend_reason = "adaptive extraction disabled".to_string();
                build_export_mesh(
                    counted_patch_ref,
                    bounds_min,
                    bounds_max,
                    quality,
                    settings.smooth_normals,
                )
            };
            if !mesh.vertices.is_empty() && !mesh.indices.is_empty() {
                break;
            }
        }
    }
    let mesh_build_ms = mesh_build_t0.elapsed().as_millis() as u64;
    let protected_min_scale = contributors
        .iter()
        .filter_map(|c| (c.min_export_scale_mm > 0.0).then_some(c.min_export_scale_mm))
        .reduce(f32::min);
    let cleanup_t0 = Instant::now();
    let (mut mesh, cleanup_removed_triangles) = if fast_patch_mode {
        (mesh, 0)
    } else {
        cleanup_mesh(
            patch_name,
            &mesh,
            settings.min_component_triangles,
            settings.drop_tiny_components,
            protected_min_scale,
        )
    };
    let cleanup_ms = cleanup_t0.elapsed().as_millis() as u64;
    if mesh.vertices.is_empty() || mesh.indices.is_empty() {
        sdf_profile_set_enabled(false);
        return Err(format!("Patch '{}' meshed to an empty surface", patch_name));
    }

    let initial_quality = analyze_mesh_quality(&mesh);
    let initial_boundary_edges = initial_quality.boundary_edge_count;
    if !fast_patch_mode && initial_boundary_edges > 0 && initial_boundary_edges <= 128 {
        fill_boundary_loops(&mut mesh);
        let (repaired, _) = cleanup_mesh(
            patch_name,
            &mesh,
            settings.min_component_triangles,
            settings.drop_tiny_components,
            protected_min_scale,
        );
        mesh = repaired;
    }

    let write_t0 = Instant::now();
    let path = output_dir.join(format!("{patch_name}.stl"));
    export_stl(&mesh, path.to_str().ok_or("invalid aero export path")?)
        .map_err(|e| format!("Failed to write patch '{}': {}", patch_name, e))?;
    if settings.write_obj_per_patch {
        let obj_path = output_dir.join(format!("{patch_name}.obj"));
        export_obj(&mesh, obj_path.to_str().ok_or("invalid aero export path")?)
            .map_err(|e| format!("Failed to write OBJ patch '{}': {}", patch_name, e))?;
    }
    let write_ms = write_t0.elapsed().as_millis() as u64;

    let stats_t0 = Instant::now();
    let mesh_quality = analyze_mesh_quality(&mesh);
    let manifold_edge_errors = mesh_quality.manifold_edge_error_count;
    let boundary_edges = mesh_quality.boundary_edge_count;
    let (bounds_min_mm, bounds_max_mm) = mesh_bounds(&mesh);
    let (avg_surface_deviation_mm, max_surface_deviation_mm) = if fast_patch_mode {
        (0.0, 0.0)
    } else {
        surface_deviation_stats(&mesh, patch_sdf.as_ref())
    };
    let stats_ms = stats_t0.elapsed().as_millis() as u64;
    sdf_profile_set_enabled(false);
    let (query_calls, query_nanos) = counted_patch_sdf.snapshot();
    let node_visits = sdf_profile_node_visits();
    let avg_eval_us = if query_calls > 0 {
        query_nanos as f64 / query_calls as f64 / 1_000.0
    } else {
        0.0
    };
    let avg_nodes_per_query = if query_calls > 0 {
        node_visits as f64 / query_calls as f64
    } else {
        0.0
    };
    eprintln!(
        "[aero] profiling patch '{}': queries={}, avg_eval={:.2}us, avg_nodes_per_query={:.2}, mesh={}ms, cleanup={}ms, write={}ms, stats={}ms",
        patch_name,
        query_calls,
        avg_eval_us,
        avg_nodes_per_query,
        mesh_build_ms,
        cleanup_ms,
        write_ms,
        stats_ms,
    );
    let watertight = mesh_quality.watertight;
    let manifest = AeroPatchManifest {
        name: patch_name.to_string(),
        file: format!("{patch_name}.stl"),
        backend: selected_backend,
        backend_reason,
        meshing_time_ms: t0.elapsed().as_millis() as u64,
        estimated_uniform_resolution,
        estimated_uniform_voxels,
        triangle_count: mesh.indices.len() / 3,
        vertex_count: mesh.vertices.len(),
        bounds_min_mm,
        bounds_max_mm,
        contributors: contributors.iter().map(|p| p.name.clone()).collect(),
        reported_min_feature_size_mm,
        manifold_edge_errors,
        boundary_edges,
        cleanup_removed_triangles,
        cleanup_time_ms: cleanup_ms,
        write_time_ms: write_ms,
        quality_analysis_time_ms: stats_ms,
        sdf_query_calls: query_calls,
        avg_sdf_eval_us: avg_eval_us,
        avg_nodes_per_query,
        avg_surface_deviation_mm,
        max_surface_deviation_mm,
        min_edge_length_mm: mesh_quality.min_edge_length_mm,
        max_edge_length_mm: mesh_quality.max_edge_length_mm,
        avg_triangle_aspect_ratio: mesh_quality.avg_triangle_aspect_ratio,
        max_triangle_aspect_ratio: mesh_quality.max_triangle_aspect_ratio,
        mesh_quality,
        watertight,
        recommended_meshing_notes: Vec::new(),
        cache_hit: false,
    };
    let mut manifest = manifest;
    manifest.recommended_meshing_notes = patch_meshing_notes(contributors, &manifest);
    let mut warnings = Vec::new();
    if fast_patch_mode {
        warnings.push(format!(
            "Patch '{}' used the fast external OML export path; heavy cleanup and deviation diagnostics were skipped to keep runtime predictable",
            patch_name
        ));
    }
    if let Some(min_feature) = reported_min_feature_size_mm {
        let recommended_cell = min_feature * 0.5;
        if effective_target_cell > recommended_cell {
            warnings.push(format!(
                "Patch '{}' reports minimum feature size {:.3} mm, but target cell is {:.3} mm; use <= {:.3} mm for two cells across the feature",
                patch_name, min_feature, effective_target_cell, recommended_cell
            ));
        }
    }
    if manifold_edge_errors > 0 {
        warnings.push(format!(
            "Patch '{}' has {} non-manifold/boundary edge issues",
            patch_name, manifold_edge_errors
        ));
    }
    let duplicate_shells = if fast_patch_mode {
        0
    } else {
        duplicate_shell_count(&mesh)
    };
    if duplicate_shells > 0 {
        warnings.push(format!(
            "Patch '{}' has {} possible duplicate shell pair(s)",
            patch_name, duplicate_shells
        ));
    }
    let self_intersections = if fast_patch_mode {
        0
    } else {
        possible_self_intersection_count(&mesh, 512)
    };
    if self_intersections > 0 {
        warnings.push(format!(
            "Patch '{}' has {} possible self-intersection candidate pair(s)",
            patch_name, self_intersections
        ));
    }
    if estimated_uniform_voxels > 64_000_000 {
        warnings.push(format!(
            "Patch '{}' is extremely large for uniform sampling (estimated {} voxels at current target cell); prefer stricter adaptive settings or coarser draft export for iteration",
            patch_name, estimated_uniform_voxels
        ));
    }
    eprintln!(
        "[aero] finished patch '{}' via {} in {} ms ({} triangles)",
        patch_name, manifest.backend, manifest.meshing_time_ms, manifest.triangle_count
    );
    let stats = PatchExportStats {
        mesh,
        manifest,
        warnings,
    };
    if let Some(cache_key) = cache_key {
        if let Ok(mut cache) = aero_patch_cache().lock() {
            cache.insert(cache_key, stats.clone());
            if cache.len() > 64 {
                if let Some(first_key) = cache.keys().next().copied() {
                    cache.remove(&first_key);
                }
            }
        }
    }
    Ok(stats)
}

pub fn export_aero_surfaces(
    source_model: &str,
    parts: &[AeroExportPart],
    output_dir: &Path,
    settings: &AeroExportSettings,
) -> Result<AeroExportManifest, String> {
    let export_t0 = Instant::now();
    if parts.is_empty() {
        return Err(
            "No aero-tagged geometry found. Define an `aero_export` map or function in the script."
                .to_string(),
        );
    }
    fs::create_dir_all(output_dir)
        .map_err(|e| format!("Failed to create aero export directory: {}", e))?;

    let setup_t0 = Instant::now();
    let mut grouped: BTreeMap<String, Vec<AeroExportPart>> = BTreeMap::new();
    let mut included_parts = Vec::new();
    let mut excluded_parts = Vec::new();
    let mut warnings = Vec::new();

    let reachability = filter_reachable_internal_parts(parts, settings.mode)?;
    warnings.extend(reachability.warnings.clone());

    let mut ordered_parts = reachability.filtered_parts.clone();
    ordered_parts.sort_by(|a, b| {
        b.priority
            .cmp(&a.priority)
            .then_with(|| a.patch_name.cmp(&b.patch_name))
            .then_with(|| a.name.cmp(&b.name))
    });

    for part in &ordered_parts {
        let role = AeroRole::parse(&part.aero_role)?;
        let explicitly_included = part.include_in_modes.is_empty()
            || part
                .include_in_modes
                .iter()
                .any(|m| m.eq_ignore_ascii_case(settings.mode.as_str()));
        if role.included_in_mode(settings.mode) && explicitly_included {
            grouped
                .entry(part.patch_name.clone())
                .or_default()
                .push(part.clone());
            included_parts.push(part.name.clone());
        } else {
            excluded_parts.push(part.name.clone());
        }
    }

    if grouped.is_empty() {
        return Err(format!(
            "No aero geometry remained after '{}' filtering.",
            settings.mode.as_str()
        ));
    }
    let setup_ms = setup_t0.elapsed().as_millis() as u64;

    let mut patch_manifests = Vec::new();
    let mut patch_meshes = Vec::new();
    let mut report_lines = vec![
        "# Aero Export Report".to_string(),
        "".to_string(),
        format!("Mode: `{}`", settings.mode.as_str()),
        format!("Source model: `{}`", source_model),
        "".to_string(),
        "## Included Parts".to_string(),
    ];
    if included_parts.is_empty() {
        report_lines.push("- none".to_string());
    } else {
        for name in &included_parts {
            report_lines.push(format!("- {}", name));
        }
    }
    report_lines.push("".to_string());
    report_lines.push("## Excluded Parts".to_string());
    if excluded_parts.is_empty() {
        report_lines.push("- none".to_string());
    } else {
        for name in &excluded_parts {
            report_lines.push(format!("- {}", name));
        }
    }
    if !reachability.reachable_internal_parts.is_empty() {
        report_lines.push("".to_string());
        report_lines.push("## Reachable Internal Flow Parts".to_string());
        for name in &reachability.reachable_internal_parts {
            report_lines.push(format!("- {}", name));
        }
    }
    if !reachability.excluded_sealed_internal_parts.is_empty() {
        report_lines.push("".to_string());
        report_lines.push("## Excluded Sealed Internal Flow Parts".to_string());
        for name in &reachability.excluded_sealed_internal_parts {
            report_lines.push(format!("- {}", name));
        }
    }
    report_lines.push("".to_string());
    report_lines.push("## Patch Summary".to_string());

    let patch_loop_t0 = Instant::now();
    for (patch_name, contributors) in grouped {
        let stats = export_patch(&patch_name, &contributors, output_dir, settings)?;
        report_lines.push(format!(
            "- `{}`: {} triangles, {} vertices, backend: {} ({}), meshing: {}ms, watertight: {}, avg dev: {:.4}mm, max dev: {:.4}mm, edge range: {:.4}-{:.4}mm, aspect avg/max: {:.3}/{:.3}, contributors: {}",
            patch_name,
            stats.manifest.triangle_count,
            stats.manifest.vertex_count,
            stats.manifest.backend,
            stats.manifest.backend_reason,
            stats.manifest.meshing_time_ms,
            stats.manifest.watertight,
            stats.manifest.avg_surface_deviation_mm,
            stats.manifest.max_surface_deviation_mm,
            stats.manifest.min_edge_length_mm,
            stats.manifest.max_edge_length_mm,
            stats.manifest.avg_triangle_aspect_ratio,
            stats.manifest.max_triangle_aspect_ratio,
            stats.manifest.contributors.join(", ")
        ));
        let PatchExportStats {
            mesh,
            manifest,
            warnings: patch_warnings,
        } = stats;
        if !settings.fast_cfd_mode {
            patch_meshes.push((patch_name.clone(), mesh));
        }
        warnings.extend(patch_warnings);
        patch_manifests.push(manifest);
    }
    let patch_loop_ms = patch_loop_t0.elapsed().as_millis() as u64;

    if !warnings.is_empty() {
        report_lines.push("".to_string());
        report_lines.push("## Warnings".to_string());
        for warning in &warnings {
            report_lines.push(format!("- {}", warning));
        }
    }

    let manifest = AeroExportManifest {
        mode: settings.mode.as_str().to_string(),
        units: "mm".to_string(),
        source_model: source_model.to_string(),
        patches: patch_manifests,
        included_parts,
        excluded_parts,
        reachable_internal_parts: reachability.reachable_internal_parts,
        excluded_sealed_internal_parts: reachability.excluded_sealed_internal_parts,
        warnings,
    };

    let manifest_write_t0 = Instant::now();
    fs::write(
        output_dir.join("manifest.json"),
        serde_json::to_string_pretty(&manifest)
            .map_err(|e| format!("Failed to serialize aero manifest: {}", e))?,
    )
    .map_err(|e| format!("Failed to write aero manifest: {}", e))?;

    if !settings.fast_cfd_mode {
        fs::write(
            output_dir.join("patch_summary.json"),
            serde_json::to_string_pretty(&manifest.patches)
                .map_err(|e| format!("Failed to serialize aero patch summary: {}", e))?,
        )
        .map_err(|e| format!("Failed to write aero patch summary: {}", e))?;

        fs::write(output_dir.join("export_report.md"), report_lines.join("\n"))
            .map_err(|e| format!("Failed to write aero export report: {}", e))?;
    }
    let manifest_write_ms = manifest_write_t0.elapsed().as_millis() as u64;

    if settings.fast_cfd_mode {
        eprintln!(
            "[aero] outer timings: setup={}ms, patches={}ms, outputs={}ms, composite=skipped, helpers=skipped, total={}ms",
            setup_ms,
            patch_loop_ms,
            manifest_write_ms,
            export_t0.elapsed().as_millis(),
        );
        return Ok(manifest);
    }

    let composite_t0 = Instant::now();
    let composite_mesh = merge_patch_meshes(&patch_meshes);
    if !composite_mesh.vertices.is_empty() && !composite_mesh.indices.is_empty() {
        export_stl(
            &composite_mesh,
            output_dir
                .join("composite_aero.stl")
                .to_str()
                .ok_or("invalid aero export path")?,
        )
        .map_err(|e| format!("Failed to write composite aero STL: {}", e))?;
    }
    let composite_ms = composite_t0.elapsed().as_millis() as u64;
    let helper_t0 = Instant::now();
    write_combined_patch_obj(output_dir, &patch_meshes)?;
    write_openfoam_helpers(output_dir, &manifest)?;
    let helper_ms = helper_t0.elapsed().as_millis() as u64;
    eprintln!(
        "[aero] outer timings: setup={}ms, patches={}ms, outputs={}ms, composite={}ms, helpers={}ms, total={}ms",
        setup_ms,
        patch_loop_ms,
        manifest_write_ms,
        composite_ms,
        helper_ms,
        export_t0.elapsed().as_millis(),
    );

    Ok(manifest)
}

#[cfg(test)]
mod tests {
    use tempfile::tempdir;

    use super::*;
    use crate::sdf::primitives::SdfBox;

    fn test_part(
        name: &str,
        sdf: Arc<dyn Sdf>,
        aero_role: &str,
        patch_name: &str,
    ) -> AeroExportPart {
        AeroExportPart {
            name: name.into(),
            sdf,
            aero_role: aero_role.into(),
            patch_name: patch_name.into(),
            bounds_min_mm: None,
            bounds_max_mm: None,
            include_in_modes: vec![],
            preserve_feature: false,
            feature_tags: vec![],
            reachability_seed: String::new(),
            min_export_scale_mm: 1.0,
            min_feature_size_mm: None,
            feature_protection_strength: 0.0,
            priority: 0,
            disable_auto_fast_path: false,
            refinement_paths: vec![],
            refinement_body_sdf: None,
            refinement_void_sdf: None,
            refinement_path_points_mm: vec![],
            refinement_path_radii_mm: vec![],
            refinement_path_forward_extend_mm: 0.0,
            refinement_path_aft_extend_mm: 0.0,
            refinement_path_target_cell_mm: None,
        }
    }

    #[test]
    fn aero_mode_filtering_excludes_internal_structure() {
        let parts = vec![
            test_part(
                "oml",
                Arc::new(SdfBox::new(Vec3::splat(20.0))),
                "outer_mold_line",
                "fuselage",
            ),
            test_part(
                "rib",
                Arc::new(SdfBox::new(Vec3::splat(5.0))),
                "internal_structure",
                "rib",
            ),
        ];
        let temp = tempdir().unwrap();
        let settings = AeroExportSettings {
            quality: MeshQuality::Ultra,
            min_component_triangles: 0,
            ..Default::default()
        };
        let manifest = export_aero_surfaces("test_model", &parts, temp.path(), &settings).unwrap();
        assert_eq!(manifest.patches.len(), 1);
        assert_eq!(manifest.patches[0].name, "fuselage");
        assert!(manifest.excluded_parts.iter().any(|p| p == "rib"));
    }

    #[test]
    fn aero_mode_external_plus_inlets_includes_flow_path_internal() {
        let parts = vec![
            AeroExportPart {
                include_in_modes: vec!["external".into(), "external_plus_inlets".into()],
                ..test_part(
                    "aircraft_oml",
                    Arc::new(SdfBox::new(Vec3::new(40.0, 20.0, 10.0))),
                    "outer_mold_line",
                    "aircraft",
                )
            },
            AeroExportPart {
                include_in_modes: vec!["external_plus_inlets".into()],
                ..test_part(
                    "duct_internal",
                    Arc::new(crate::sdf::transforms::Translate::new(
                        Arc::new(SdfBox::new(Vec3::new(10.0, 8.0, 30.0))),
                        Vec3::new(15.0, 0.0, 0.0),
                    )),
                    "flow_path_internal",
                    "duct",
                )
            },
            test_part(
                "internal_rib",
                Arc::new(SdfBox::new(Vec3::splat(5.0))),
                "internal_structure",
                "rib",
            ),
        ];
        let temp = tempdir().unwrap();
        let settings = AeroExportSettings {
            mode: AeroExportMode::ExternalPlusInlets,
            quality: MeshQuality::Ultra,
            min_component_triangles: 0,
            ..Default::default()
        };
        let manifest = export_aero_surfaces("test_model", &parts, temp.path(), &settings).unwrap();
        assert_eq!(manifest.patches.len(), 2);
        assert!(manifest.patches.iter().any(|p| p.name == "aircraft"));
        assert!(manifest.patches.iter().any(|p| p.name == "duct"));
        assert!(manifest.included_parts.iter().any(|p| p == "duct_internal"));
        assert!(manifest.excluded_parts.iter().any(|p| p == "internal_rib"));
    }

    #[test]
    fn aero_patch_export_uses_cache_on_repeat() {
        let parts = vec![AeroExportPart {
            include_in_modes: vec!["external".into()],
            ..test_part(
                "aircraft_oml",
                Arc::new(SdfBox::new(Vec3::new(40.0, 20.0, 10.0))),
                "outer_mold_line",
                "aircraft",
            )
        }];
        let temp = tempdir().unwrap();
        let settings = AeroExportSettings {
            quality: MeshQuality::Ultra,
            min_component_triangles: 0,
            ..Default::default()
        };
        let first = export_aero_surfaces("cached_model", &parts, temp.path(), &settings).unwrap();
        let second = export_aero_surfaces("cached_model", &parts, temp.path(), &settings).unwrap();
        assert!(
            !first.patches[0].cache_hit,
            "first export should build the patch"
        );
        assert!(
            second.patches[0].cache_hit,
            "second identical export should reuse cached patch mesh"
        );
    }

    #[test]
    fn cleanup_mesh_welds_near_duplicate_vertices() {
        let mesh = Mesh {
            vertices: vec![
                crate::mesh::Vertex {
                    position: [0.0, 0.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                },
                crate::mesh::Vertex {
                    position: [1.0, 0.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                },
                crate::mesh::Vertex {
                    position: [0.0, 1.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                },
                crate::mesh::Vertex {
                    position: [1.0 + 5e-5, 0.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                },
            ],
            indices: vec![0, 1, 2, 0, 2, 3],
        };
        let (cleaned, _) = cleanup_mesh("test_patch", &mesh, 0, false, None);
        assert!(
            cleaned.vertices.len() < mesh.vertices.len(),
            "cleanup should weld near-duplicate vertices"
        );
    }

    #[test]
    fn cleanup_mesh_orients_faces_outward() {
        let mesh = Mesh {
            vertices: vec![
                crate::mesh::Vertex {
                    position: [1.0, 1.0, 1.0],
                    normal: [0.0, 0.0, 1.0],
                },
                crate::mesh::Vertex {
                    position: [-1.0, -1.0, 1.0],
                    normal: [0.0, 0.0, 1.0],
                },
                crate::mesh::Vertex {
                    position: [-1.0, 1.0, -1.0],
                    normal: [0.0, 0.0, 1.0],
                },
                crate::mesh::Vertex {
                    position: [1.0, -1.0, -1.0],
                    normal: [0.0, 0.0, 1.0],
                },
            ],
            indices: vec![0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3],
        };
        let (cleaned, _) = cleanup_mesh("test_patch", &mesh, 0, false, None);
        let mut centroid = Vec3::ZERO;
        for vertex in &cleaned.vertices {
            centroid += Vec3::from_array(vertex.position);
        }
        centroid /= cleaned.vertices.len() as f32;

        for tri in cleaned.indices.chunks_exact(3) {
            let p0 = Vec3::from_array(cleaned.vertices[tri[0] as usize].position);
            let p1 = Vec3::from_array(cleaned.vertices[tri[1] as usize].position);
            let p2 = Vec3::from_array(cleaned.vertices[tri[2] as usize].position);
            let normal = (p1 - p0).cross(p2 - p0);
            let face_center = (p0 + p1 + p2) / 3.0;
            assert!(
                normal.dot(face_center - centroid) >= 0.0,
                "face should be outward oriented after cleanup"
            );
        }
    }

    #[test]
    fn cleanup_mesh_drops_tiny_disconnected_shell_by_diagonal() {
        let mesh = Mesh {
            vertices: vec![
                crate::mesh::Vertex {
                    position: [0.0, 0.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                },
                crate::mesh::Vertex {
                    position: [10.0, 0.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                },
                crate::mesh::Vertex {
                    position: [0.0, 10.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                },
                crate::mesh::Vertex {
                    position: [50.0, 50.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                },
                crate::mesh::Vertex {
                    position: [50.02, 50.0, 0.0],
                    normal: [0.0, 0.0, 1.0],
                },
                crate::mesh::Vertex {
                    position: [50.0, 50.02, 0.0],
                    normal: [0.0, 0.0, 1.0],
                },
            ],
            indices: vec![0, 1, 2, 3, 4, 5],
        };
        let (cleaned, removed) = cleanup_mesh("test_patch", &mesh, 0, true, None);
        assert_eq!(removed, 1);
        assert_eq!(cleaned.indices.len(), 3);
    }

    #[test]
    fn patch_adaptive_settings_tighten_for_preserved_features() {
        let contributors = vec![AeroExportPart {
            preserve_feature: true,
            feature_tags: vec!["trailing_edge".into()],
            min_export_scale_mm: 0.4,
            feature_protection_strength: 2.0,
            ..test_part(
                "wing_te",
                Arc::new(SdfBox::new(Vec3::new(40.0, 2.0, 1.0))),
                "outer_mold_line",
                "wing",
            )
        }];
        let settings = AeroExportSettings::default();
        let adaptive = patch_adaptive_export_settings(
            &contributors,
            &settings,
            MeshQuality::Normal,
            Vec3::ZERO,
            Vec3::new(40.0, 2.0, 1.0),
        );
        assert!(adaptive.octree.min_cell_size_mm <= 0.4);
        assert!(adaptive.octree.max_depth > settings.adaptive_octree.max_depth);
        assert!(
            adaptive.octree.curvature_refine_threshold
                < settings.adaptive_octree.curvature_refine_threshold
        );
    }

    #[test]
    fn aero_patch_manifest_records_backend_choice() {
        let parts = vec![AeroExportPart {
            include_in_modes: vec!["external".into()],
            ..test_part(
                "sphere",
                Arc::new(crate::sdf::primitives::Sphere::new(10.0)),
                "outer_mold_line",
                "body",
            )
        }];
        let temp = tempdir().unwrap();
        let manifest = export_aero_surfaces(
            "sphere_test",
            &parts,
            temp.path(),
            &AeroExportSettings::default(),
        )
        .unwrap();
        assert!(!manifest.patches[0].backend.is_empty());
        assert!(manifest.patches[0].max_surface_deviation_mm >= 0.0);
    }

    #[test]
    fn patch_adaptive_settings_respond_to_feature_tags() {
        let contributors = vec![AeroExportPart {
            preserve_feature: true,
            feature_tags: vec!["trailing_edge".into(), "wing_body_junction".into()],
            min_export_scale_mm: 0.4,
            feature_protection_strength: 2.0,
            ..test_part(
                "wing_te",
                Arc::new(SdfBox::new(Vec3::new(40.0, 2.0, 1.0))),
                "outer_mold_line",
                "wing",
            )
        }];
        let settings = AeroExportSettings::default();
        let adaptive = patch_adaptive_export_settings(
            &contributors,
            &settings,
            MeshQuality::Normal,
            Vec3::ZERO,
            Vec3::new(40.0, 2.0, 1.0),
        );
        assert!(adaptive.octree.min_cell_size_mm < settings.adaptive_octree.min_cell_size_mm);
        assert!(adaptive.octree.surface_band_mm >= settings.adaptive_octree.surface_band_mm);
    }

    #[test]
    fn reachability_filter_excludes_sealed_internal_patch() {
        let outer: Arc<dyn Sdf> = Arc::new(SdfBox::new(Vec3::new(40.0, 40.0, 40.0)));
        let sealed_cavity: Arc<dyn Sdf> = Arc::new(crate::sdf::transforms::Translate::new(
            Arc::new(SdfBox::new(Vec3::new(10.0, 10.0, 10.0))),
            Vec3::new(0.0, 0.0, 0.0),
        ));
        let parts = vec![
            AeroExportPart {
                include_in_modes: vec!["external_plus_inlets".into()],
                ..test_part("outer", outer, "outer_mold_line", "outer")
            },
            AeroExportPart {
                include_in_modes: vec!["external_plus_inlets".into()],
                ..test_part("sealed", sealed_cavity, "flow_path_internal", "sealed")
            },
        ];
        let result =
            filter_reachable_internal_parts(&parts, AeroExportMode::ExternalPlusInlets).unwrap();
        assert!(result.filtered_parts.iter().all(|p| p.name != "sealed"));
        assert!(
            result
                .warnings
                .iter()
                .any(|w| w.contains("Excluded sealed internal flow patch"))
        );
        assert!(
            result
                .excluded_sealed_internal_parts
                .iter()
                .any(|p| p == "sealed")
        );
    }

    #[test]
    fn aero_manifest_records_reachability_lists() {
        let parts = vec![
            AeroExportPart {
                include_in_modes: vec!["external_plus_inlets".into()],
                ..test_part(
                    "outer",
                    Arc::new(SdfBox::new(Vec3::new(30.0, 30.0, 30.0))),
                    "outer_mold_line",
                    "outer",
                )
            },
            AeroExportPart {
                include_in_modes: vec!["external_plus_inlets".into()],
                ..test_part(
                    "sealed",
                    Arc::new(crate::sdf::transforms::Translate::new(
                        Arc::new(SdfBox::new(Vec3::new(6.0, 6.0, 6.0))),
                        Vec3::new(0.0, 0.0, 0.0),
                    )),
                    "flow_path_internal",
                    "sealed",
                )
            },
        ];
        let temp = tempdir().unwrap();
        let manifest = export_aero_surfaces(
            "reachability_test",
            &parts,
            temp.path(),
            &AeroExportSettings {
                mode: AeroExportMode::ExternalPlusInlets,
                quality: MeshQuality::Draft,
                min_component_triangles: 0,
                ..Default::default()
            },
        )
        .unwrap();
        assert!(
            manifest
                .excluded_sealed_internal_parts
                .iter()
                .any(|p| p == "sealed")
        );
        assert!(manifest.reachable_internal_parts.is_empty());
    }

    #[test]
    fn priority_orders_patch_contributors_deterministically() {
        let parts = vec![
            AeroExportPart {
                include_in_modes: vec!["external".into()],
                priority: 1,
                ..test_part(
                    "low",
                    Arc::new(SdfBox::new(Vec3::new(20.0, 20.0, 20.0))),
                    "outer_mold_line",
                    "aircraft",
                )
            },
            AeroExportPart {
                include_in_modes: vec!["external".into()],
                priority: 10,
                ..test_part(
                    "high",
                    Arc::new(crate::sdf::transforms::Translate::new(
                        Arc::new(SdfBox::new(Vec3::new(10.0, 10.0, 10.0))),
                        Vec3::new(5.0, 0.0, 0.0),
                    )),
                    "outer_mold_line",
                    "aircraft",
                )
            },
        ];
        let temp = tempdir().unwrap();
        let manifest = export_aero_surfaces(
            "priority_test",
            &parts,
            temp.path(),
            &AeroExportSettings {
                quality: MeshQuality::Draft,
                min_component_triangles: 0,
                ..Default::default()
            },
        )
        .unwrap();
        assert_eq!(
            manifest.included_parts,
            vec!["high".to_string(), "low".to_string()]
        );
        assert_eq!(
            manifest.patches[0].contributors,
            vec!["high".to_string(), "low".to_string()]
        );
    }

    #[test]
    fn aero_export_preflight_rejects_absurd_patch_size() {
        let parts = vec![AeroExportPart {
            include_in_modes: vec!["external".into()],
            ..test_part(
                "huge",
                Arc::new(SdfBox::new(Vec3::new(5000.0, 5000.0, 5000.0))),
                "outer_mold_line",
                "huge",
            )
        }];
        let temp = tempdir().unwrap();
        let err = export_aero_surfaces(
            "huge_model",
            &parts,
            temp.path(),
            &AeroExportSettings {
                quality: MeshQuality::Normal,
                max_patch_preflight_voxels: 1_000_000,
                ..Default::default()
            },
        )
        .unwrap_err();
        assert!(err.contains("aborted before meshing"));
    }
}
