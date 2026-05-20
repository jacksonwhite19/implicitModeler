use std::fs;
use std::path::PathBuf;

use clap::Parser;
use glam::Vec3;
use implicit_cad::gpu::extract_mesh_from_vertex_grid_gpu;
use implicit_cad::scripting::evaluate_aero_export_parts;
use indexmap::IndexMap;

#[derive(Parser, Debug)]
#[command(name = "debug-uniform-reference")]
struct Args {
    #[arg(long)]
    script: PathBuf,
    #[arg(long, default_value = "aircraft_oml")]
    part: String,
    #[arg(long)]
    target_cell_mm: f32,
}

fn main() -> Result<(), String> {
    let args = Args::parse();
    let source = fs::read_to_string(&args.script)
        .map_err(|e| format!("Failed to read script {}: {}", args.script.display(), e))?;
    let dims = IndexMap::new();
    let parts = evaluate_aero_export_parts(
        &source,
        None,
        None,
        None,
        None,
        &dims,
        args.script.parent(),
        None,
        &[],
    )?;
    let part = parts
        .iter()
        .find(|p| p.name == args.part)
        .ok_or_else(|| format!("Part '{}' not found", args.part))?;

    let bounds_min = part
        .bounds_min_mm
        .map(Vec3::from_array)
        .ok_or_else(|| "debug bin expects explicit bounds_min_mm".to_string())?;
    let bounds_max = part
        .bounds_max_mm
        .map(Vec3::from_array)
        .ok_or_else(|| "debug bin expects explicit bounds_max_mm".to_string())?;

    let target_cell = args.target_cell_mm.max(0.02);
    let cell_resolution =
        (((bounds_max - bounds_min).max_element() / target_cell).ceil() as u32).clamp(8, 512);
    let vertex_resolution = cell_resolution + 1;
    let step = (bounds_max - bounds_min) / cell_resolution as f32;
    let total = (vertex_resolution * vertex_resolution * vertex_resolution) as usize;

    let mut min_v = f32::INFINITY;
    let mut max_v = f32::NEG_INFINITY;
    let mut neg = 0usize;
    let mut pos = 0usize;
    let mut zero = 0usize;
    let mut data = Vec::with_capacity(total);

    for idx in 0..total {
        let x = (idx % vertex_resolution as usize) as u32;
        let y = ((idx / vertex_resolution as usize) % vertex_resolution as usize) as u32;
        let z = (idx / (vertex_resolution * vertex_resolution) as usize) as u32;
        let p = bounds_min + Vec3::new(x as f32 * step.x, y as f32 * step.y, z as f32 * step.z);
        let d = part.sdf.distance(p);
        min_v = min_v.min(d);
        max_v = max_v.max(d);
        if d < 0.0 {
            neg += 1;
        } else if d > 0.0 {
            pos += 1;
        } else {
            zero += 1;
        }
        data.push(d);
    }

    let mut sign_change_cells = 0usize;
    let mut all_outside = 0usize;
    let mut all_inside = 0usize;
    let vr = vertex_resolution as usize;
    let cr = cell_resolution as usize;
    for z in 0..cr {
        for y in 0..cr {
            for x in 0..cr {
                let i000 = x + y * vr + z * vr * vr;
                let i100 = i000 + 1;
                let i010 = i000 + vr;
                let i110 = i010 + 1;
                let i001 = i000 + vr * vr;
                let i101 = i001 + 1;
                let i011 = i001 + vr;
                let i111 = i011 + 1;
                let vals = [
                    data[i000], data[i100], data[i110], data[i010], data[i001], data[i101],
                    data[i111], data[i011],
                ];
                let any_neg = vals.iter().any(|v| *v < 0.0);
                let any_pos = vals.iter().any(|v| *v > 0.0);
                if any_neg && any_pos {
                    sign_change_cells += 1;
                } else if any_neg {
                    all_inside += 1;
                } else {
                    all_outside += 1;
                }
            }
        }
    }

    println!("part={}", part.name);
    println!(
        "bounds_min=({:.3},{:.3},{:.3}) bounds_max=({:.3},{:.3},{:.3})",
        bounds_min.x, bounds_min.y, bounds_min.z, bounds_max.x, bounds_max.y, bounds_max.z
    );
    println!(
        "target_cell_mm={:.3} cell_resolution={} vertex_resolution={}",
        target_cell, cell_resolution, vertex_resolution
    );
    println!("step=({:.6},{:.6},{:.6})", step.x, step.y, step.z);
    println!("min_scalar={:.6}", min_v);
    println!("max_scalar={:.6}", max_v);
    println!("neg_vertices={}", neg);
    println!("pos_vertices={}", pos);
    println!("zero_vertices={}", zero);
    println!("sign_change_cells={}", sign_change_cells);
    println!("all_inside_cells={}", all_inside);
    println!("all_outside_cells={}", all_outside);

    match extract_mesh_from_vertex_grid_gpu(bounds_min, bounds_max, cell_resolution, &data, false) {
        Ok(mesh) => {
            println!("extract_ok=true");
            println!("mesh_vertices={}", mesh.vertices.len());
            println!("mesh_indices={}", mesh.indices.len());
            let finite_vertices = mesh
                .vertices
                .iter()
                .filter(|v| {
                    v.position[0].is_finite()
                        && v.position[1].is_finite()
                        && v.position[2].is_finite()
                })
                .count();
            let mut vmin = Vec3::splat(f32::INFINITY);
            let mut vmax = Vec3::splat(f32::NEG_INFINITY);
            for v in &mesh.vertices {
                let p = Vec3::from_array(v.position);
                vmin = vmin.min(p);
                vmax = vmax.max(p);
            }
            let mut degenerate = 0usize;
            let mut min_area = f32::INFINITY;
            let mut max_area = 0.0f32;
            for tri in mesh.indices.chunks_exact(3) {
                let p0 = Vec3::from_array(mesh.vertices[tri[0] as usize].position);
                let p1 = Vec3::from_array(mesh.vertices[tri[1] as usize].position);
                let p2 = Vec3::from_array(mesh.vertices[tri[2] as usize].position);
                let area = 0.5 * (p1 - p0).cross(p2 - p0).length();
                min_area = min_area.min(area);
                max_area = max_area.max(area);
                if area <= 1e-7 {
                    degenerate += 1;
                }
            }
            println!("finite_vertices={}", finite_vertices);
            println!(
                "mesh_bounds_min=({:.6},{:.6},{:.6}) mesh_bounds_max=({:.6},{:.6},{:.6})",
                vmin.x, vmin.y, vmin.z, vmax.x, vmax.y, vmax.z
            );
            println!("triangle_count={}", mesh.indices.len() / 3);
            println!("degenerate_triangles={}", degenerate);
            println!("min_triangle_area={:.9}", min_area);
            println!("max_triangle_area={:.9}", max_area);
            for (i, v) in mesh.vertices.iter().take(6).enumerate() {
                println!(
                    "v{}=({:.6},{:.6},{:.6})",
                    i, v.position[0], v.position[1], v.position[2]
                );
            }
        }
        Err(err) => {
            println!("extract_ok=false");
            println!("extract_error={}", err);
        }
    }

    Ok(())
}
