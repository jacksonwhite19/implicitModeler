// Headless mode for batch processing

use crate::analysis::{
    GeometryValidationResult, ValidationSettings, compute_model_properties, validate_geometry,
};
use crate::export::adaptive_octree::AdaptiveOctreeSettings;
use crate::export::aero::{AeroExportMode, AeroExportSettings, export_aero_surfaces};
use crate::mesh::{Mesh, MeshQuality};
use crate::pipeline::compute_sdf_grid_cached_arc;
use crate::scripting;
use crate::scripting::ScriptResult;
use crate::sdf::Sdf;
use glam::Vec3;
use indexmap::IndexMap;
use serde::Serialize;
use std::collections::HashMap;
use std::path::Path;
use std::sync::Arc;
use std::time::Instant;

// ── Metrics structs ───────────────────────────────────────────────────────────

#[derive(Serialize)]
pub struct BoundingBoxMetrics {
    pub min: [f32; 3],
    pub max: [f32; 3],
    pub size: [f32; 3],
}

#[derive(Serialize)]
pub struct ConditioningMetrics {
    pub state: String,
    pub generation: u64,
    pub block_count: usize,
    pub grid_spacing_mm: f32,
    pub interface_band_mm: f32,
    pub confidence: f32,
    pub publication_confidence: f32,
    pub quality_confidence: f32,
    pub anchor_count: usize,
    pub projection_anchor_count: usize,
    pub narrow_band_block_count: usize,
    pub max_iteration_count: usize,
    pub adaptive_region_count: usize,
    pub adaptive_sample_count: usize,
    pub adaptive_narrow_band_region_count: usize,
    pub adaptive_narrow_band_sample_count: usize,
    pub adaptive_min_grid_spacing_mm: Option<f32>,
    pub adaptive_max_grid_spacing_mm: Option<f32>,
    pub adaptive_feature_id_count: usize,
    pub adaptive_requested_feature_id_count: usize,
    pub adaptive_feature_coverage: f32,
    pub adaptive_requested_region_count: usize,
    pub adaptive_skipped_region_count: usize,
    pub adaptive_oversized_region_count: usize,
    pub adaptive_requested_min_grid_spacing_mm: Option<f32>,
    pub block_sample_count: usize,
    pub block_sample_bytes: usize,
    pub adaptive_stored_sample_count: usize,
    pub adaptive_stored_sample_bytes: usize,
    pub adaptive_dense_region_count: usize,
    pub adaptive_sparse_region_count: usize,
    pub adaptive_dense_value_count: usize,
    pub adaptive_sparse_value_count: usize,
    pub total_stored_sample_count: usize,
    pub total_stored_sample_bytes: usize,
    pub estimated_metadata_bytes: usize,
    pub estimated_total_bytes: usize,
}

#[derive(Serialize)]
pub struct MetricsOutput {
    pub schema_version: u32,
    pub volume_mm3: f32,
    pub surface_area_mm2: f32,
    pub bounding_box: BoundingBoxMetrics,
    pub center_of_mass: [f32; 3],
    pub estimated_mass_g: f32,
    pub min_wall_thickness_mm: f32,
    pub min_wall_location: [f32; 3],
    pub component_masses: HashMap<String, f32>,
    pub component_positions: HashMap<String, [f32; 3]>,
    pub script_cg: Option<[f32; 3]>,
    pub geometric_cg: [f32; 3],
    pub evaluation_time_ms: u64,
    pub conditioning_time_ms: Option<u64>,
    pub mesh_extraction_time_ms: u64,
    pub grid_resolution: u32,
    pub dimensions_used: HashMap<String, f64>,
    pub conditioning: Option<ConditioningMetrics>,
    pub validation: GeometryValidationResult,
}

fn quality_from_resolution(resolution: u32) -> MeshQuality {
    match resolution {
        0..=24 => MeshQuality::Draft,
        25..=40 => MeshQuality::Normal,
        41..=56 => MeshQuality::Fine,
        _ => MeshQuality::Ultra,
    }
}

// ── Mesh geometry helpers ─────────────────────────────────────────────────────

fn mesh_bounds(mesh: &Mesh) -> (Vec3, Vec3) {
    if mesh.vertices.is_empty() {
        return (Vec3::ZERO, Vec3::ZERO);
    }
    let mut min = Vec3::splat(f32::MAX);
    let mut max = Vec3::splat(f32::MIN);
    for v in &mesh.vertices {
        let p = Vec3::from(v.position);
        min = min.min(p);
        max = max.max(p);
    }
    (min, max)
}

/// Simplified min-wall-thickness estimate via SDF interior sampling.
/// Samples on a 32³ grid; for interior points, -SDF(p) approximates local clearance to surface.
fn min_wall_thickness(sdf: &dyn Sdf, bbox_min: Vec3, bbox_max: Vec3) -> (f32, Vec3) {
    const STEPS: usize = 32;
    let mut min_thickness = f32::MAX;
    let mut min_location = Vec3::ZERO;

    let step = (bbox_max - bbox_min) / STEPS as f32;

    for ix in 0..STEPS {
        for iy in 0..STEPS {
            for iz in 0..STEPS {
                let p = bbox_min
                    + Vec3::new(
                        ix as f32 * step.x + step.x * 0.5,
                        iy as f32 * step.y + step.y * 0.5,
                        iz as f32 * step.z + step.z * 0.5,
                    );
                let d = sdf.distance(p);
                // Only consider interior points (d < 0)
                if d < 0.0 {
                    let thickness = -d;
                    if thickness < min_thickness {
                        min_thickness = thickness;
                        min_location = p;
                    }
                }
            }
        }
    }

    if min_thickness == f32::MAX {
        (0.0, Vec3::ZERO)
    } else {
        (min_thickness, min_location)
    }
}

// ── Public metrics API ────────────────────────────────────────────────────────

pub fn compute_metrics(
    sdf: &Arc<dyn Sdf>,
    mesh: &Mesh,
    script_result: &ScriptResult,
    resolution: u32,
    eval_time_ms: u64,
    mesh_extraction_time_ms: u64,
    dimensions: &IndexMap<String, f64>,
) -> MetricsOutput {
    // Geometry from sampled field properties so thin-walled/open-shell models do not
    // collapse to zero metrics just because the export mesh is not watertight.
    let (bb_min, bb_max) = mesh_bounds(mesh);
    let span = (bb_max - bb_min).max(Vec3::splat(1.0));
    let pad = span * 0.05 + Vec3::splat(1.0);
    let grid =
        compute_sdf_grid_cached_arc(sdf, bb_min - pad, bb_max + pad, resolution.clamp(16, 48));
    let (volume, surface_area, geometric_cg) = compute_model_properties(&grid);
    let bb_size = bb_max - bb_min;

    // Estimated mass: PLA density ≈ 0.0012 g/mm³
    const PLA_DENSITY: f32 = 0.0012;
    let estimated_mass = volume * PLA_DENSITY;

    // Min wall thickness
    let (min_wall, min_wall_loc) = min_wall_thickness(sdf.as_ref(), bb_min, bb_max);

    // Component masses and positions from script
    let mut component_masses: HashMap<String, f32> = HashMap::new();
    let mut component_positions: HashMap<String, [f32; 3]> = HashMap::new();
    for mp in &script_result.mass_points {
        component_masses.insert(mp.name.clone(), mp.mass_g);
        component_positions.insert(mp.name.clone(), mp.position.to_array());
    }

    // Script CG
    let script_cg = script_result.center_of_gravity().map(|v| v.to_array());

    // Dimensions
    let dimensions_used: HashMap<String, f64> =
        dimensions.iter().map(|(k, v)| (k.clone(), *v)).collect();
    let validation = validate_geometry(
        sdf,
        mesh,
        &ValidationSettings {
            grid_resolution: resolution.clamp(16, 48),
            ..Default::default()
        },
    );
    let conditioning = sdf
        .metadata()
        .conditioned_cache
        .map(|cache| ConditioningMetrics {
            state: format!("{:?}", cache.state),
            generation: cache.generation,
            block_count: cache.block_count,
            grid_spacing_mm: cache.grid_spacing,
            interface_band_mm: cache.interface_band,
            confidence: cache.confidence,
            publication_confidence: cache.publication_confidence,
            quality_confidence: cache.quality_confidence,
            anchor_count: cache.anchor_count,
            projection_anchor_count: cache.projection_anchor_count,
            narrow_band_block_count: cache.narrow_band_block_count,
            max_iteration_count: cache.max_iteration_count,
            adaptive_region_count: cache.adaptive_region_count,
            adaptive_sample_count: cache.adaptive_sample_count,
            adaptive_narrow_band_region_count: cache.adaptive_narrow_band_region_count,
            adaptive_narrow_band_sample_count: cache.adaptive_narrow_band_sample_count,
            adaptive_min_grid_spacing_mm: cache.adaptive_min_grid_spacing,
            adaptive_max_grid_spacing_mm: cache.adaptive_max_grid_spacing,
            adaptive_feature_id_count: cache.adaptive_feature_id_count,
            adaptive_requested_feature_id_count: cache.adaptive_requested_feature_id_count,
            adaptive_feature_coverage: cache.adaptive_feature_coverage,
            adaptive_requested_region_count: cache.adaptive_requested_region_count,
            adaptive_skipped_region_count: cache.adaptive_skipped_region_count,
            adaptive_oversized_region_count: cache.adaptive_oversized_region_count,
            adaptive_requested_min_grid_spacing_mm: cache.adaptive_requested_min_grid_spacing,
            block_sample_count: cache.block_sample_count,
            block_sample_bytes: cache.block_sample_bytes,
            adaptive_stored_sample_count: cache.adaptive_stored_sample_count,
            adaptive_stored_sample_bytes: cache.adaptive_stored_sample_bytes,
            adaptive_dense_region_count: cache.adaptive_dense_region_count,
            adaptive_sparse_region_count: cache.adaptive_sparse_region_count,
            adaptive_dense_value_count: cache.adaptive_dense_value_count,
            adaptive_sparse_value_count: cache.adaptive_sparse_value_count,
            total_stored_sample_count: cache.total_stored_sample_count,
            total_stored_sample_bytes: cache.total_stored_sample_bytes,
            estimated_metadata_bytes: cache.estimated_metadata_bytes,
            estimated_total_bytes: cache.estimated_total_bytes,
        });

    MetricsOutput {
        schema_version: 5,
        volume_mm3: volume,
        surface_area_mm2: surface_area,
        bounding_box: BoundingBoxMetrics {
            min: bb_min.to_array(),
            max: bb_max.to_array(),
            size: bb_size.to_array(),
        },
        center_of_mass: geometric_cg.to_array(),
        estimated_mass_g: estimated_mass,
        min_wall_thickness_mm: min_wall,
        min_wall_location: min_wall_loc.to_array(),
        component_masses,
        component_positions,
        script_cg,
        geometric_cg: geometric_cg.to_array(),
        evaluation_time_ms: eval_time_ms,
        conditioning_time_ms: None,
        mesh_extraction_time_ms,
        grid_resolution: resolution,
        dimensions_used,
        conditioning,
        validation,
    }
}

// ── Headless entry points ─────────────────────────────────────────────────────

pub fn execute_script_headless(
    script_path: &Path,
    output_path: &Path,
    format: &str,
    resolution: u32,
    smooth_normals: bool,
) -> Result<(), String> {
    execute_script_headless_extended(
        script_path,
        Some(output_path),
        format,
        resolution,
        smooth_normals,
        &[],
        None,
        "external",
        0.5,
        1.0,
        6,
        false,
        false,
        false,
        128_000_000,
    )
}

pub fn execute_script_headless_extended(
    script_path: &Path,
    output_path: Option<&Path>,
    format: &str,
    resolution: u32,
    smooth_normals: bool,
    dim_overrides: &[(String, f64)],
    metrics_path: Option<&Path>,
    aero_mode: &str,
    aero_target_error_mm: f32,
    aero_min_cell_mm: f32,
    aero_max_depth: u32,
    aero_write_obj: bool,
    aero_fast_mode: bool,
    aero_uniform_reference: bool,
    aero_max_patch_voxels: u64,
) -> Result<(), String> {
    // Load script and dimensions
    let (script, mut dimensions) = load_script_and_dims(script_path)?;

    // Apply dimension overrides
    for (k, v) in dim_overrides {
        dimensions.insert(k.clone(), *v);
    }

    let is_aero_export = format.eq_ignore_ascii_case("aero");

    if is_aero_export {
        let t0 = Instant::now();
        eprintln!(
            "[aero] starting aero metadata evaluation for {}",
            script_path.display()
        );
        let out_dir = output_path.ok_or("Aero export requires --output <directory>")?;
        let mut aero_parts = scripting::evaluate_aero_export_parts_for_mode(
            &script,
            None,
            None,
            None,
            None,
            &dimensions,
            script_path.parent(),
            None,
            &[],
            aero_mode,
        )?;
        let aero_eval_ms = t0.elapsed().as_millis() as u64;
        eprintln!(
            "[aero] metadata evaluation completed in {} ms ({} aero part(s))",
            aero_eval_ms,
            aero_parts.len()
        );
        let conditioning_t0 = Instant::now();
        scripting::condition_aero_export_parts_for_backend(&mut aero_parts);
        let aero_conditioning_ms = conditioning_t0.elapsed().as_millis() as u64;
        eprintln!(
            "[aero] conditioned {} aero part(s) in {} ms",
            aero_parts.len(),
            aero_conditioning_ms
        );
        let eval_ms = aero_eval_ms;
        let quality = quality_from_resolution(resolution);
        let settings = AeroExportSettings {
            mode: AeroExportMode::parse(aero_mode)?,
            quality,
            smooth_normals,
            write_obj_per_patch: aero_write_obj,
            fast_cfd_mode: aero_fast_mode,
            uniform_reference_mode: aero_uniform_reference,
            uniform_reference_target_cell_mm: aero_min_cell_mm.max(0.05),
            max_patch_preflight_voxels: aero_max_patch_voxels.max(1_000_000),
            adaptive_octree: AdaptiveOctreeSettings {
                error_tolerance_mm: aero_target_error_mm.max(0.01),
                min_cell_size_mm: aero_min_cell_mm.max(0.05),
                max_depth: aero_max_depth.max(1),
                ..Default::default()
            },
            ..Default::default()
        };
        let source_model = script_path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("model");
        eprintln!("[aero] starting patch export for {}", source_model);
        let manifest = export_aero_surfaces(source_model, &aero_parts, out_dir, &settings)?;
        eprintln!(
            "[aero] patch export completed in {} ms",
            t0.elapsed().as_millis()
        );
        println!(
            "Aero export wrote {} patch file(s) to {}",
            manifest.patches.len(),
            out_dir.display()
        );
        if let Some(mpath) = metrics_path {
            let mut script_result = scripting::evaluate_script_full(
                &script,
                None,
                None,
                None,
                None,
                &dimensions,
                script_path.parent(),
                None,
                &[],
            )?;
            let conditioning_t0 = Instant::now();
            script_result.condition_for_backend();
            let conditioning_ms = conditioning_t0.elapsed().as_millis() as u64;
            let (bounds_min, bounds_max) = crate::pipeline::auto_bounds(script_result.sdf.as_ref());
            let metrics_mesh = crate::export::build_export_mesh(
                script_result.sdf.as_ref(),
                bounds_min,
                bounds_max,
                quality,
                smooth_normals,
            );
            let metrics = compute_metrics(
                &script_result.sdf,
                &metrics_mesh,
                &script_result,
                resolution,
                eval_ms,
                0,
                &dimensions,
            );
            let mut metrics = metrics;
            metrics.conditioning_time_ms = Some(conditioning_ms);
            let json = serde_json::to_string_pretty(&metrics)
                .map_err(|e| format!("Metrics serialization failed: {}", e))?;
            std::fs::write(mpath, json)
                .map_err(|e| format!("Failed to write metrics file: {}", e))?;
            println!("Metrics written to {}", mpath.display());
        }
        println!("Evaluation time: {}ms, resolution: {}", eval_ms, resolution);
        return Ok(());
    }

    // Time the evaluation
    let t0 = Instant::now();
    let mut script_result = scripting::evaluate_script_full(
        &script,
        None,
        None,
        None,
        None,
        &dimensions,
        script_path.parent(),
        None,
        &[],
    )?;
    let eval_ms = t0.elapsed().as_millis() as u64;

    let conditioning_t0 = Instant::now();
    script_result.condition_for_backend();
    let conditioning_time_ms = conditioning_t0.elapsed().as_millis() as u64;

    // Compute mesh bounds from bounding points or use defaults
    let (bounds_min, bounds_max) = crate::pipeline::auto_bounds(&*script_result.sdf);

    // Extract mesh
    let mesh_t0 = Instant::now();
    let mesh = crate::export::build_export_mesh(
        script_result.sdf.as_ref(),
        bounds_min,
        bounds_max,
        quality_from_resolution(resolution),
        smooth_normals,
    );
    let mesh_extraction_time_ms = mesh_t0.elapsed().as_millis() as u64;

    // Export mesh if requested
    if let Some(out) = output_path {
        match format.to_lowercase().as_str() {
            "stl" => {
                crate::export::export_by_format(&mesh, out, "stl")
                    .map_err(|e| format!("STL export failed: {}", e))?;
            }
            "obj" => {
                crate::export::export_by_format(&mesh, out, "obj")
                    .map_err(|e| format!("OBJ export failed: {}", e))?;
            }
            "package" => {
                crate::export::export_manufacturing_package(
                    &mesh,
                    script_path
                        .file_stem()
                        .and_then(|s| s.to_str())
                        .unwrap_or("project"),
                    out,
                )?;
            }
            _ => {
                return Err(format!(
                    "Unknown format: {}. Use 'stl', 'obj', 'package', or 'aero'.",
                    format
                ));
            }
        }
        println!(
            "Exported {} ({} vertices, {} triangles)",
            out.display(),
            mesh.vertices.len(),
            mesh.indices.len() / 3
        );
    }

    // Compute and write metrics if requested
    if let Some(mpath) = metrics_path {
        let mut metrics = compute_metrics(
            &script_result.sdf,
            &mesh,
            &script_result,
            resolution,
            eval_ms,
            mesh_extraction_time_ms,
            &dimensions,
        );
        metrics.conditioning_time_ms = Some(conditioning_time_ms);
        let json = serde_json::to_string_pretty(&metrics)
            .map_err(|e| format!("Metrics serialization failed: {}", e))?;
        std::fs::write(mpath, json).map_err(|e| format!("Failed to write metrics file: {}", e))?;
        println!("Metrics written to {}", mpath.display());
    }

    println!("Evaluation time: {}ms, resolution: {}", eval_ms, resolution);
    Ok(())
}

/// Load script text and dimensions from a path.
/// Supports `.icad` project files or raw Rhai scripts.
fn load_script_and_dims(path: &Path) -> Result<(String, IndexMap<String, f64>), String> {
    let ext = path.extension().and_then(|s| s.to_str()).unwrap_or("");

    if ext.eq_ignore_ascii_case("icad") {
        // Try loading as a project file
        match crate::project::Project::load(path) {
            Ok(project) => {
                return Ok((project.script, project.dimensions));
            }
            Err(_) => {
                // Fall through to raw script loading
            }
        }
    }

    // Load as raw Rhai script
    let script =
        std::fs::read_to_string(path).map_err(|e| format!("Failed to read script: {}", e))?;
    Ok((script, IndexMap::new()))
}

pub fn execute_batch(
    input_dir: &Path,
    output_dir: &Path,
    format: &str,
    resolution: u32,
    smooth_normals: bool,
    aero_mode: &str,
    aero_target_error_mm: f32,
    aero_min_cell_mm: f32,
    aero_max_depth: u32,
    aero_write_obj: bool,
    aero_fast_mode: bool,
    aero_uniform_reference: bool,
    aero_max_patch_voxels: u64,
) -> Result<(), String> {
    std::fs::create_dir_all(output_dir)
        .map_err(|e| format!("Failed to create output directory: {}", e))?;

    let entries = std::fs::read_dir(input_dir)
        .map_err(|e| format!("Failed to read input directory: {}", e))?;

    let mut successes = 0;
    let mut failures = 0;

    for entry in entries {
        let entry = entry.map_err(|e| format!("Failed to read directory entry: {}", e))?;
        let path = entry.path();

        if path.extension().and_then(|s| s.to_str()) == Some("rhai") {
            let file_stem = path.file_stem().unwrap();
            let output_name = format!("{}.{}", file_stem.to_str().unwrap(), format);
            let output_path = output_dir.join(output_name);

            print!(
                "Processing {}... ",
                path.file_name().unwrap().to_str().unwrap()
            );
            match execute_script_headless_extended(
                &path,
                Some(&output_path),
                format,
                resolution,
                smooth_normals,
                &[],
                None,
                aero_mode,
                aero_target_error_mm,
                aero_min_cell_mm,
                aero_max_depth,
                aero_write_obj,
                aero_fast_mode,
                aero_uniform_reference,
                aero_max_patch_voxels,
            ) {
                Ok(_) => {
                    successes += 1;
                }
                Err(e) => {
                    println!("FAILED");
                    eprintln!("  Error: {}", e);
                    failures += 1;
                }
            }
        }
    }

    println!("\nBatch processing complete:");
    println!("  Successes: {}", successes);
    println!("  Failures:  {}", failures);

    if failures > 0 {
        Err(format!("{} file(s) failed to process", failures))
    } else {
        Ok(())
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scripting::evaluate_script;
    use tempfile::TempDir;

    #[test]
    fn test_metrics_volume_sphere() {
        // Sphere of radius 10 → analytical volume = 4/3 * π * 10³ ≈ 4188.8 mm³
        let script_result = evaluate_script("sphere(10.0)").unwrap();
        let sdf = &*script_result.sdf;
        let mesh = crate::export::build_export_mesh(
            sdf,
            Vec3::splat(-15.0),
            Vec3::splat(15.0),
            MeshQuality::Ultra,
            false,
        );
        let metrics = compute_metrics(
            &script_result.sdf,
            &mesh,
            &script_result,
            64,
            0,
            0,
            &IndexMap::new(),
        );
        let analytical = (4.0_f32 / 3.0) * std::f32::consts::PI * 1000.0; // 4188.8
        let error_pct = ((metrics.volume_mm3 - analytical) / analytical).abs() * 100.0;
        assert!(
            error_pct < 2.0,
            "Volume error {} should be < 2%, got {}mm3 vs {}mm3",
            error_pct,
            metrics.volume_mm3,
            analytical
        );
    }

    #[test]
    fn test_metrics_has_all_fields() {
        let script_result = evaluate_script("sphere(5.0)")
            .unwrap()
            .into_conditioned_for_backend();
        let sdf = &*script_result.sdf;
        let mesh = crate::export::build_export_mesh(
            sdf,
            Vec3::splat(-10.0),
            Vec3::splat(10.0),
            MeshQuality::Normal,
            false,
        );
        let metrics = compute_metrics(
            &script_result.sdf,
            &mesh,
            &script_result,
            32,
            100,
            0,
            &IndexMap::new(),
        );
        assert_eq!(metrics.schema_version, 5);
        assert!(metrics.volume_mm3 > 0.0);
        assert!(metrics.surface_area_mm2 > 0.0);
        assert_eq!(metrics.grid_resolution, 32);
        assert_eq!(metrics.evaluation_time_ms, 100);
        let conditioning = metrics
            .conditioning
            .as_ref()
            .expect("conditioned script SDF should report cache metrics");
        assert_eq!(conditioning.state, "Ready");
        assert!(conditioning.block_count > 0);
        assert!(conditioning.confidence > 0.0);
        assert!(
            metrics.validation.valid,
            "validation failed: hard_failures={:?}, warnings={:?}",
            metrics.validation.hard_failures, metrics.validation.warnings
        );
    }

    #[test]
    fn test_dim_override_in_metrics() {
        let script = "sphere(wingspan)";
        let mut dims = IndexMap::new();
        dims.insert("wingspan".to_string(), 8.0_f64);
        let result = crate::scripting::evaluate_script_full(
            script,
            None,
            None,
            None,
            None,
            &dims,
            None,
            None,
            &[],
        )
        .unwrap();
        assert!(
            result.sdf.distance(Vec3::new(7.5, 0.0, 0.0)) < 0.0,
            "Sphere of radius 8 should contain point at 7.5"
        );
        let mesh = crate::export::build_export_mesh(
            &*result.sdf,
            Vec3::splat(-12.0),
            Vec3::splat(12.0),
            MeshQuality::Normal,
            false,
        );
        let metrics = compute_metrics(&result.sdf, &mesh, &result, 32, 0, 0, &dims);
        assert_eq!(metrics.dimensions_used.get("wingspan"), Some(&8.0_f64));
    }

    #[test]
    fn test_load_script_and_dims_from_icad() {
        let temp = TempDir::new().unwrap();
        let path = temp.path().join("sample.icad");
        std::fs::write(
            &path,
            r#"{
  "version": "0.1.0",
  "script": "sphere(radius)",
  "resolution": 32,
  "smooth_normals": false,
  "show_wireframe": false,
  "camera_position": [0.0, 0.0, 10.0],
  "camera_target": [0.0, 0.0, 0.0],
  "dimensions": { "radius": 6.5 }
}"#,
        )
        .unwrap();

        let (script, dims) = load_script_and_dims(&path).unwrap();
        assert_eq!(script, "sphere(radius)");
        assert_eq!(dims.get("radius"), Some(&6.5));
    }

    #[test]
    fn test_execute_headless_from_icad_writes_mesh_and_metrics() {
        let temp = TempDir::new().unwrap();
        let project_path = temp.path().join("sample.icad");
        let mesh_path = temp.path().join("sample.stl");
        let metrics_path = temp.path().join("metrics.json");
        std::fs::write(
            &project_path,
            r#"{
  "version": "0.1.0",
  "script": "sphere(radius)",
  "resolution": 32,
  "smooth_normals": false,
  "show_wireframe": false,
  "camera_position": [0.0, 0.0, 10.0],
  "camera_target": [0.0, 0.0, 0.0],
  "dimensions": { "radius": 4.0 }
}"#,
        )
        .unwrap();

        execute_script_headless_extended(
            &project_path,
            Some(&mesh_path),
            "stl",
            24,
            false,
            &[("radius".to_string(), 5.0)],
            Some(&metrics_path),
            "external",
            0.5,
            1.0,
            6,
            false,
            false,
            false,
            128_000_000,
        )
        .unwrap();

        assert!(
            mesh_path.exists(),
            "Headless export should write the mesh file"
        );
        assert!(
            metrics_path.exists(),
            "Headless export should write the metrics file"
        );
        let metrics = std::fs::read_to_string(metrics_path).unwrap();
        assert!(metrics.contains("\"radius\""));
        assert!(metrics.contains("5.0"));
        assert!(metrics.contains("\"conditioning\""));
        assert!(metrics.contains("\"validation\""));
    }

    #[test]
    fn test_execute_headless_package_export() {
        let temp = TempDir::new().unwrap();
        let script_path = temp.path().join("sample.rhai");
        let package_dir = temp.path().join("pkg");
        std::fs::write(&script_path, "sphere(8.0)").unwrap();

        execute_script_headless_extended(
            &script_path,
            Some(&package_dir),
            "package",
            24,
            false,
            &[],
            None,
            "external",
            0.5,
            1.0,
            6,
            false,
            false,
            false,
            128_000_000,
        )
        .unwrap();

        assert!(package_dir.join("main_body.stl").exists());
        assert!(package_dir.join("bom.csv").exists());
        assert!(package_dir.join("assembly_notes.md").exists());
    }

    #[test]
    fn test_execute_headless_aero_export_writes_manifest_and_patch() {
        let temp = TempDir::new().unwrap();
        let script_path = temp.path().join("aircraft.rhai");
        let aero_dir = temp.path().join("aero");
        std::fs::write(
            &script_path,
            r#"
                let fuse = translate(box_(40.0, 30.0, 20.0), 3.0, 2.0, 1.0);
                let rib = translate(box_(5.0, 5.0, 5.0), 0.0, 0.0, 0.0);
                let aero_export = #{
                    parts: [
                        #{
                            name: "fuselage_oml",
                            sdf: fuse,
                            aero_role: "outer_mold_line",
                            patch_name: "fuselage",
                            include_in_modes: ["external"]
                        },
                        #{
                            name: "internal_rib",
                            sdf: rib,
                            aero_role: "internal_structure",
                            patch_name: "rib"
                        }
                    ]
                };
                union(fuse, rib)
            "#,
        )
        .unwrap();

        let script = std::fs::read_to_string(&script_path).unwrap();
        let parts = crate::scripting::evaluate_aero_export_parts(
            &script,
            None,
            None,
            None,
            None,
            &IndexMap::new(),
            script_path.parent(),
            None,
            &[],
        )
        .unwrap();
        let settings = crate::export::aero::AeroExportSettings {
            mode: crate::export::aero::AeroExportMode::External,
            quality: MeshQuality::Ultra,
            smooth_normals: false,
            drop_tiny_components: true,
            min_component_triangles: 0,
            ..Default::default()
        };
        crate::export::aero::export_aero_surfaces("aircraft", &parts, &aero_dir, &settings)
            .unwrap();

        assert!(aero_dir.join("manifest.json").exists());
        assert!(aero_dir.join("patch_summary.json").exists());
        assert!(aero_dir.join("export_report.md").exists());
        assert!(aero_dir.join("openfoam_geometry_dict.txt").exists());
        assert!(aero_dir.join("openfoam_boundary_summary.md").exists());
        assert!(aero_dir.join("snappyHexMesh_notes.md").exists());
        assert!(aero_dir.join("composite_aero.stl").exists());
        assert!(aero_dir.join("combined_patches.obj").exists());
        assert!(
            aero_dir
                .join("openfoam_template")
                .join("system")
                .join("surfaceFeatureExtractDict")
                .exists()
        );
        assert!(
            aero_dir
                .join("openfoam_template")
                .join("constant")
                .join("triSurface")
                .join("fuselage.stl")
                .exists()
        );
        assert!(aero_dir.join("fuselage.stl").exists());
    }

    #[test]
    fn test_execute_headless_aero_export_external_plus_inlets_writes_duct_patch() {
        let temp = TempDir::new().unwrap();
        let script_path = temp.path().join("aircraft_inlet.rhai");
        let aero_dir = temp.path().join("aero_inlets");
        std::fs::write(
            &script_path,
            r#"
                let aircraft = translate(box_(40.0, 30.0, 20.0), 3.0, 2.0, 1.0);
                let duct = translate(box_(10.0, 8.0, 30.0), 15.0, 0.0, 0.0);
                let rib = box_(5.0, 5.0, 5.0);
                let aero_export = #{
                    parts: [
                        #{
                            name: "aircraft_oml",
                            sdf: aircraft,
                            aero_role: "outer_mold_line",
                            patch_name: "aircraft",
                            include_in_modes: ["external", "external_plus_inlets"]
                        },
                        #{
                            name: "duct_internal",
                            sdf: duct,
                            aero_role: "flow_path_internal",
                            patch_name: "duct",
                            include_in_modes: ["external_plus_inlets"]
                        },
                        #{
                            name: "internal_rib",
                            sdf: rib,
                            aero_role: "internal_structure",
                            patch_name: "rib"
                        }
                    ]
                };
                union(aircraft, union(duct, rib))
            "#,
        )
        .unwrap();

        let script = std::fs::read_to_string(&script_path).unwrap();
        let parts = crate::scripting::evaluate_aero_export_parts(
            &script,
            None,
            None,
            None,
            None,
            &IndexMap::new(),
            script_path.parent(),
            None,
            &[],
        )
        .unwrap();
        let settings = crate::export::aero::AeroExportSettings {
            mode: crate::export::aero::AeroExportMode::ExternalPlusInlets,
            quality: MeshQuality::Ultra,
            smooth_normals: false,
            drop_tiny_components: true,
            min_component_triangles: 0,
            ..Default::default()
        };
        let manifest = crate::export::aero::export_aero_surfaces(
            "aircraft_inlet",
            &parts,
            &aero_dir,
            &settings,
        )
        .unwrap();

        assert!(aero_dir.join("manifest.json").exists());
        assert!(aero_dir.join("aircraft.stl").exists());
        assert!(aero_dir.join("duct.stl").exists());
        assert!(manifest.included_parts.iter().any(|p| p == "duct_internal"));
        assert!(manifest.excluded_parts.iter().any(|p| p == "internal_rib"));
    }

    #[test]
    fn test_aero_export_optionally_writes_obj_per_patch() {
        let temp = TempDir::new().unwrap();
        let script_path = temp.path().join("aircraft_obj.rhai");
        let aero_dir = temp.path().join("aero_obj");
        std::fs::write(
            &script_path,
            r#"
                let aircraft = box_(20.0, 10.0, 8.0);
                let aero_export = #{
                    parts: [
                        #{
                            name: "aircraft_oml",
                            sdf: aircraft,
                            aero_role: "outer_mold_line",
                            patch_name: "aircraft"
                        }
                    ]
                };
                aircraft
            "#,
        )
        .unwrap();
        let script = std::fs::read_to_string(&script_path).unwrap();
        let parts = crate::scripting::evaluate_aero_export_parts(
            &script,
            None,
            None,
            None,
            None,
            &IndexMap::new(),
            script_path.parent(),
            None,
            &[],
        )
        .unwrap();
        let settings = crate::export::aero::AeroExportSettings {
            mode: crate::export::aero::AeroExportMode::External,
            quality: MeshQuality::Draft,
            write_obj_per_patch: true,
            min_component_triangles: 0,
            ..Default::default()
        };
        crate::export::aero::export_aero_surfaces("aircraft_obj", &parts, &aero_dir, &settings)
            .unwrap();
        assert!(aero_dir.join("aircraft.stl").exists());
        assert!(aero_dir.join("aircraft.obj").exists());
        assert!(aero_dir.join("patch_summary.json").exists());
    }

    #[test]
    fn test_scratchpad_plane_shell_metrics_are_nonzero() {
        let script = std::fs::read_to_string("scratchpad_plane.rhai").unwrap();
        let result = crate::scripting::evaluate_script_full(
            &script,
            None,
            None,
            None,
            None,
            &IndexMap::new(),
            Some(std::path::Path::new(".")),
            None,
            &[],
        )
        .unwrap();

        let (bounds_min, bounds_max) = crate::pipeline::auto_bounds(result.sdf.as_ref());
        let mesh = crate::export::build_export_mesh(
            result.sdf.as_ref(),
            bounds_min,
            bounds_max,
            MeshQuality::Draft,
            false,
        );
        let metrics = compute_metrics(&result.sdf, &mesh, &result, 24, 0, 0, &IndexMap::new());
        assert!(
            metrics.volume_mm3 > 0.0,
            "shell-aware metrics should report non-zero volume"
        );
        assert!(
            metrics.surface_area_mm2 > 0.0,
            "shell-aware metrics should report non-zero surface area"
        );
        assert!(
            !metrics
                .validation
                .hard_failures
                .iter()
                .any(|f| f == "volume_too_small"),
            "scratchpad plane should no longer fail volume_too_small: {:?}",
            metrics.validation.hard_failures
        );
    }
}
