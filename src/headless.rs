// Headless mode for batch processing

use std::path::Path;
use std::collections::HashMap;
use std::time::Instant;
use glam::Vec3;
use serde::Serialize;
use indexmap::IndexMap;
use crate::sdf::Sdf;
use crate::scripting;
use crate::scripting::ScriptResult;
use crate::mesh::{Mesh, marching_cubes};

// ── Metrics structs ───────────────────────────────────────────────────────────

#[derive(Serialize)]
pub struct BoundingBoxMetrics {
    pub min:  [f32; 3],
    pub max:  [f32; 3],
    pub size: [f32; 3],
}

#[derive(Serialize)]
pub struct MetricsOutput {
    pub schema_version:        u32,
    pub volume_mm3:            f32,
    pub surface_area_mm2:      f32,
    pub bounding_box:          BoundingBoxMetrics,
    pub center_of_mass:        [f32; 3],
    pub estimated_mass_g:      f32,
    pub min_wall_thickness_mm: f32,
    pub min_wall_location:     [f32; 3],
    pub component_masses:      HashMap<String, f32>,
    pub component_positions:   HashMap<String, [f32; 3]>,
    pub script_cg:             Option<[f32; 3]>,
    pub geometric_cg:          [f32; 3],
    pub evaluation_time_ms:    u64,
    pub grid_resolution:       u32,
    pub dimensions_used:       HashMap<String, f64>,
}

// ── Mesh geometry helpers ─────────────────────────────────────────────────────

fn mesh_volume(mesh: &Mesh) -> f32 {
    let mut vol = 0.0_f32;
    for i in (0..mesh.indices.len()).step_by(3) {
        let a = Vec3::from(mesh.vertices[mesh.indices[i]     as usize].position);
        let b = Vec3::from(mesh.vertices[mesh.indices[i + 1] as usize].position);
        let c = Vec3::from(mesh.vertices[mesh.indices[i + 2] as usize].position);
        vol += a.dot(b.cross(c));
    }
    (vol / 6.0).abs()
}

fn mesh_surface_area(mesh: &Mesh) -> f32 {
    let mut area = 0.0_f32;
    for i in (0..mesh.indices.len()).step_by(3) {
        let a = Vec3::from(mesh.vertices[mesh.indices[i]     as usize].position);
        let b = Vec3::from(mesh.vertices[mesh.indices[i + 1] as usize].position);
        let c = Vec3::from(mesh.vertices[mesh.indices[i + 2] as usize].position);
        area += (b - a).cross(c - a).length() * 0.5;
    }
    area
}

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

fn mesh_centroid(mesh: &Mesh) -> Vec3 {
    if mesh.vertices.is_empty() {
        return Vec3::ZERO;
    }
    let sum: Vec3 = mesh.vertices.iter().map(|v| Vec3::from(v.position)).sum();
    sum / mesh.vertices.len() as f32
}

/// Simplified min-wall-thickness estimate via SDF interior sampling.
/// Samples on a 32³ grid; for interior points, -SDF(p) approximates local clearance to surface.
fn min_wall_thickness(sdf: &dyn Sdf, bbox_min: Vec3, bbox_max: Vec3) -> (f32, Vec3) {
    const STEPS: usize = 32;
    let mut min_thickness = f32::MAX;
    let mut min_location  = Vec3::ZERO;

    let step = (bbox_max - bbox_min) / STEPS as f32;

    for ix in 0..STEPS {
        for iy in 0..STEPS {
            for iz in 0..STEPS {
                let p = bbox_min + Vec3::new(
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
                        min_location  = p;
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
    sdf:           &dyn Sdf,
    mesh:          &Mesh,
    script_result: &ScriptResult,
    resolution:    u32,
    eval_time_ms:  u64,
    dimensions:    &IndexMap<String, f64>,
) -> MetricsOutput {
    // Geometry
    let volume       = mesh_volume(mesh);
    let surface_area = mesh_surface_area(mesh);
    let (bb_min, bb_max) = mesh_bounds(mesh);
    let bb_size = bb_max - bb_min;
    let geometric_cg = mesh_centroid(mesh);

    // Estimated mass: PLA density ≈ 0.0012 g/mm³
    const PLA_DENSITY: f32 = 0.0012;
    let estimated_mass = volume * PLA_DENSITY;

    // Min wall thickness
    let (min_wall, min_wall_loc) = min_wall_thickness(sdf, bb_min, bb_max);

    // Component masses and positions from script
    let mut component_masses:    HashMap<String, f32>      = HashMap::new();
    let mut component_positions: HashMap<String, [f32; 3]> = HashMap::new();
    for mp in &script_result.mass_points {
        component_masses.insert(mp.name.clone(), mp.mass_g);
        component_positions.insert(mp.name.clone(), mp.position.to_array());
    }

    // Script CG
    let script_cg = script_result.center_of_gravity().map(|v| v.to_array());

    // Dimensions
    let dimensions_used: HashMap<String, f64> = dimensions
        .iter()
        .map(|(k, v)| (k.clone(), *v))
        .collect();

    MetricsOutput {
        schema_version:        1,
        volume_mm3:            volume,
        surface_area_mm2:      surface_area,
        bounding_box: BoundingBoxMetrics {
            min:  bb_min.to_array(),
            max:  bb_max.to_array(),
            size: bb_size.to_array(),
        },
        center_of_mass:        geometric_cg.to_array(),
        estimated_mass_g:      estimated_mass,
        min_wall_thickness_mm: min_wall,
        min_wall_location:     min_wall_loc.to_array(),
        component_masses,
        component_positions,
        script_cg,
        geometric_cg:          geometric_cg.to_array(),
        evaluation_time_ms:    eval_time_ms,
        grid_resolution:       resolution,
        dimensions_used,
    }
}

// ── Headless entry points ─────────────────────────────────────────────────────

pub fn execute_script_headless(
    script_path:    &Path,
    output_path:    &Path,
    format:         &str,
    resolution:     u32,
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
    )
}

pub fn execute_script_headless_extended(
    script_path:    &Path,
    output_path:    Option<&Path>,
    format:         &str,
    resolution:     u32,
    smooth_normals: bool,
    dim_overrides:  &[(String, f64)],
    metrics_path:   Option<&Path>,
) -> Result<(), String> {
    // Load script and dimensions
    let (script, mut dimensions) = load_script_and_dims(script_path)?;

    // Apply dimension overrides
    for (k, v) in dim_overrides {
        dimensions.insert(k.clone(), *v);
    }

    // Time the evaluation
    let t0 = Instant::now();
    let script_result = scripting::evaluate_script_full(
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

    // Compute mesh bounds from bounding points or use defaults
    let (bounds_min, bounds_max) = auto_bounds(&*script_result.sdf, resolution);

    // Extract mesh
    let mesh = marching_cubes::extract_mesh(
        script_result.sdf.as_ref(),
        bounds_min,
        bounds_max,
        resolution,
        smooth_normals,
    );

    // Export mesh if requested
    if let Some(out) = output_path {
        match format.to_lowercase().as_str() {
            "stl" => {
                crate::export::export_stl(&mesh, out.to_str().unwrap())
                    .map_err(|e| format!("STL export failed: {}", e))?;
            }
            "obj" => {
                crate::export::export_obj(&mesh, out.to_str().unwrap())
                    .map_err(|e| format!("OBJ export failed: {}", e))?;
            }
            _ => {
                return Err(format!("Unknown format: {}. Use 'stl' or 'obj'.", format));
            }
        }
        println!("Exported {} ({} vertices, {} triangles)",
            out.display(),
            mesh.vertices.len(),
            mesh.indices.len() / 3);
    }

    // Compute and write metrics if requested
    if let Some(mpath) = metrics_path {
        let metrics = compute_metrics(
            script_result.sdf.as_ref(),
            &mesh,
            &script_result,
            resolution,
            eval_ms,
            &dimensions,
        );
        let json = serde_json::to_string_pretty(&metrics)
            .map_err(|e| format!("Metrics serialization failed: {}", e))?;
        std::fs::write(mpath, json)
            .map_err(|e| format!("Failed to write metrics file: {}", e))?;
        println!("Metrics written to {}", mpath.display());
    }

    println!("Evaluation time: {}ms, resolution: {}", eval_ms, resolution);
    Ok(())
}

/// Load script text and dimensions from a path.
/// Supports .ntop (project JSON) or raw Rhai scripts.
fn load_script_and_dims(path: &Path) -> Result<(String, IndexMap<String, f64>), String> {
    let ext = path.extension().and_then(|s| s.to_str()).unwrap_or("");

    if ext == "ntop" {
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
    let script = std::fs::read_to_string(path)
        .map_err(|e| format!("Failed to read script: {}", e))?;
    Ok((script, IndexMap::new()))
}

/// Determine good mesh extraction bounds by probing the SDF.
/// Uses a heuristic: expand from origin until SDF is positive everywhere on the shell.
fn auto_bounds(sdf: &dyn Sdf, _resolution: u32) -> (Vec3, Vec3) {
    // Quick sample: find the extent of the geometry
    let probe_radii = [10.0_f32, 25.0, 50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0];
    for &r in &probe_radii {
        // Check corners of the cube at radius r
        let all_outside = [
            Vec3::new( r,  r,  r),
            Vec3::new(-r,  r,  r),
            Vec3::new( r, -r,  r),
            Vec3::new( r,  r, -r),
        ]
        .iter()
        .all(|&p| sdf.distance(p) > 0.0);

        if all_outside {
            return (Vec3::splat(-r), Vec3::splat(r));
        }
    }
    (Vec3::splat(-2000.0), Vec3::splat(2000.0))
}

pub fn execute_batch(
    input_dir:      &Path,
    output_dir:     &Path,
    format:         &str,
    resolution:     u32,
    smooth_normals: bool,
) -> Result<(), String> {
    std::fs::create_dir_all(output_dir)
        .map_err(|e| format!("Failed to create output directory: {}", e))?;

    let entries = std::fs::read_dir(input_dir)
        .map_err(|e| format!("Failed to read input directory: {}", e))?;

    let mut successes = 0;
    let mut failures  = 0;

    for entry in entries {
        let entry = entry.map_err(|e| format!("Failed to read directory entry: {}", e))?;
        let path  = entry.path();

        if path.extension().and_then(|s| s.to_str()) == Some("rhai") {
            let file_stem   = path.file_stem().unwrap();
            let output_name = format!("{}.{}", file_stem.to_str().unwrap(), format);
            let output_path = output_dir.join(output_name);

            print!("Processing {}... ", path.file_name().unwrap().to_str().unwrap());
            match execute_script_headless(&path, &output_path, format, resolution, smooth_normals) {
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

    #[test]
    fn test_metrics_volume_sphere() {
        // Sphere of radius 10 → analytical volume = 4/3 * π * 10³ ≈ 4188.8 mm³
        let script_result = evaluate_script("sphere(10.0)").unwrap();
        let sdf = &*script_result.sdf;
        let mesh = marching_cubes::extract_mesh(sdf, Vec3::splat(-15.0), Vec3::splat(15.0), 64, false);
        let metrics = compute_metrics(sdf, &mesh, &script_result, 64, 0, &IndexMap::new());
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
        let script_result = evaluate_script("sphere(5.0)").unwrap();
        let sdf = &*script_result.sdf;
        let mesh = marching_cubes::extract_mesh(sdf, Vec3::splat(-10.0), Vec3::splat(10.0), 32, false);
        let metrics = compute_metrics(sdf, &mesh, &script_result, 32, 100, &IndexMap::new());
        assert_eq!(metrics.schema_version, 1);
        assert!(metrics.volume_mm3 > 0.0);
        assert!(metrics.surface_area_mm2 > 0.0);
        assert_eq!(metrics.grid_resolution, 32);
        assert_eq!(metrics.evaluation_time_ms, 100);
    }

    #[test]
    fn test_dim_override_in_metrics() {
        let script = "sphere(wingspan)";
        let mut dims = IndexMap::new();
        dims.insert("wingspan".to_string(), 8.0_f64);
        let result = crate::scripting::evaluate_script_full(
            script, None, None, None, None, &dims, None, None, &[]
        ).unwrap();
        assert!(
            result.sdf.distance(Vec3::new(7.5, 0.0, 0.0)) < 0.0,
            "Sphere of radius 8 should contain point at 7.5"
        );
        let mesh = marching_cubes::extract_mesh(&*result.sdf, Vec3::splat(-12.0), Vec3::splat(12.0), 32, false);
        let metrics = compute_metrics(&*result.sdf, &mesh, &result, 32, 0, &dims);
        assert_eq!(metrics.dimensions_used.get("wingspan"), Some(&8.0_f64));
    }
}
