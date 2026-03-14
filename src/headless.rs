// Headless mode for batch processing

use std::path::Path;
use std::sync::Arc;
use glam::Vec3;
use crate::sdf::Sdf;
use crate::scripting;
use crate::mesh::marching_cubes;

pub fn execute_script_headless(
    script_path: &Path,
    output_path: &Path,
    format: &str,
    resolution: u32,
    smooth_normals: bool,
) -> Result<(), String> {
    // Read script file
    let script = std::fs::read_to_string(script_path)
        .map_err(|e| format!("Failed to read script: {}", e))?;

    // Evaluate script
    let sdf = scripting::evaluate_script(&script)?.sdf;

    // Extract mesh
    let bounds_min = Vec3::new(-50.0, -50.0, -50.0);
    let bounds_max = Vec3::new(50.0, 50.0, 50.0);
    let mesh = marching_cubes::extract_mesh(sdf.as_ref(), bounds_min, bounds_max, resolution, smooth_normals);

    // Export based on format
    match format.to_lowercase().as_str() {
        "stl" => {
            crate::export::export_stl(&mesh, output_path.to_str().unwrap())
                .map_err(|e| format!("STL export failed: {}", e))?;
        }
        "obj" => {
            crate::export::export_obj(&mesh, output_path.to_str().unwrap())
                .map_err(|e| format!("OBJ export failed: {}", e))?;
        }
        _ => {
            return Err(format!("Unknown format: {}. Use 'stl' or 'obj'.", format));
        }
    }

    println!("✓ Exported {} ({} vertices, {} triangles)",
             output_path.display(),
             mesh.vertices.len(),
             mesh.indices.len() / 3);

    Ok(())
}

pub fn execute_batch(
    input_dir: &Path,
    output_dir: &Path,
    format: &str,
    resolution: u32,
    smooth_normals: bool,
) -> Result<(), String> {
    // Ensure output directory exists
    std::fs::create_dir_all(output_dir)
        .map_err(|e| format!("Failed to create output directory: {}", e))?;

    // Find all .rhai files
    let entries = std::fs::read_dir(input_dir)
        .map_err(|e| format!("Failed to read input directory: {}", e))?;

    let mut successes = 0;
    let mut failures = 0;
    let mut errors = Vec::new();

    for entry in entries {
        let entry = entry.map_err(|e| format!("Failed to read directory entry: {}", e))?;
        let path = entry.path();

        if path.extension().and_then(|s| s.to_str()) == Some("rhai") {
            let file_stem = path.file_stem().unwrap();
            let output_name = format!("{}.{}", file_stem.to_str().unwrap(), format);
            let output_path = output_dir.join(output_name);

            print!("Processing {}... ", path.file_name().unwrap().to_str().unwrap());
            match execute_script_headless(&path, &output_path, format, resolution, smooth_normals) {
                Ok(_) => {
                    successes += 1;
                }
                Err(e) => {
                    println!("✗ FAILED");
                    eprintln!("  Error: {}", e);
                    errors.push((path.clone(), e));
                    failures += 1;
                }
            }
        }
    }

    println!("\nBatch processing complete:");
    println!("  Successes: {}", successes);
    println!("  Failures: {}", failures);

    if failures > 0 {
        Err(format!("{} file(s) failed to process", failures))
    } else {
        Ok(())
    }
}
