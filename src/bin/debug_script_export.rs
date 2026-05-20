use std::path::PathBuf;
use std::time::Instant;

use clap::Parser;
use implicit_cad::export::dual_contouring::analyze_mesh_topology;
use implicit_cad::export::{
    SurfaceProjectionSettings, build_export_mesh_uniform_reference, export_stl,
    smooth_export_mesh_with_topology_guard,
};
use implicit_cad::pipeline::auto_bounds;
use implicit_cad::scripting::evaluate_script;

#[derive(Parser, Debug)]
#[command(name = "debug-script-export")]
struct Args {
    #[arg(long)]
    script: PathBuf,

    #[arg(long)]
    output: PathBuf,

    #[arg(long, default_value_t = 4.0)]
    target_cell_mm: f32,

    #[arg(long)]
    smooth_surface: bool,
}

fn main() -> Result<(), String> {
    let args = Args::parse();
    let source =
        std::fs::read_to_string(&args.script).map_err(|e| format!("failed to read script: {e}"))?;
    let eval_start = Instant::now();
    let result = evaluate_script(&source)?;
    let eval_ms = eval_start.elapsed().as_millis();

    let (bounds_min, bounds_max) = auto_bounds(result.sdf.as_ref());
    let mesh_start = Instant::now();
    let mut mesh = build_export_mesh_uniform_reference(
        result.sdf.as_ref(),
        bounds_min,
        bounds_max,
        args.target_cell_mm,
        false,
    );
    if args.smooth_surface {
        mesh = smooth_export_mesh_with_topology_guard(
            &mesh,
            result.sdf.as_ref(),
            args.target_cell_mm,
            SurfaceProjectionSettings {
                enabled: true,
                ..Default::default()
            },
        );
    }
    let mesh_ms = mesh_start.elapsed().as_millis();

    let topology = analyze_mesh_topology(&mesh);
    let write_start = Instant::now();
    export_stl(
        &mesh,
        args.output
            .to_str()
            .ok_or("output path is not valid UTF-8")?,
    )
    .map_err(|e| format!("failed to write STL: {e}"))?;
    let write_ms = write_start.elapsed().as_millis();

    println!("script={}", args.script.display());
    println!("output={}", args.output.display());
    println!("target_cell_mm={:.3}", args.target_cell_mm);
    println!("smooth_surface={}", args.smooth_surface);
    println!(
        "bounds_min=({:.3},{:.3},{:.3}) bounds_max=({:.3},{:.3},{:.3})",
        bounds_min.x, bounds_min.y, bounds_min.z, bounds_max.x, bounds_max.y, bounds_max.z
    );
    println!("eval_ms={eval_ms}");
    println!("mesh_ms={mesh_ms}");
    println!("write_ms={write_ms}");
    println!("vertices={}", mesh.vertices.len());
    println!("triangles={}", mesh.indices.len() / 3);
    println!("boundary_edges={}", topology.boundary_edges);
    println!("non_manifold_edges={}", topology.non_manifold_edges);
    println!("degenerate_triangles={}", topology.degenerate_triangles);
    println!("duplicate_triangles={}", topology.duplicate_triangles);

    Ok(())
}
