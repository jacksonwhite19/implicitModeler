use std::fs;
use std::path::PathBuf;

use clap::Parser;
use glam::Vec3;
use implicit_cad::gpu::{lower_sdf_ir, sample_section_from_grid_gpu, sample_section_gpu, SectionPlane};
use implicit_cad::pipeline::compute_sdf_grid;
use implicit_cad::scripting::evaluate_script;

#[derive(Parser, Debug)]
#[command(name = "sample-section")]
#[command(about = "Sample signed distance on an XZ grid at a fixed Y plane from a Rhai script")]
struct Args {
    #[arg(long)]
    script: PathBuf,
    #[arg(long, default_value = "xz")]
    plane: String,
    #[arg(long, default_value_t = 0.0)]
    coord: f32,
    #[arg(long)]
    amin: f32,
    #[arg(long)]
    amax: f32,
    #[arg(long)]
    bmin: f32,
    #[arg(long)]
    bmax: f32,
    #[arg(long, default_value_t = 401)]
    na: usize,
    #[arg(long, default_value_t = 241)]
    nb: usize,
    #[arg(long)]
    out: PathBuf,
    #[arg(long, default_value = "auto")]
    backend: String,
}

fn main() -> Result<(), String> {
    let args = Args::parse();
    let source = fs::read_to_string(&args.script)
        .map_err(|e| format!("Failed to read script {}: {}", args.script.display(), e))?;
    let result = evaluate_script(&source)?;

    if args.na < 2 || args.nb < 2 {
        return Err("na and nb must be at least 2".into());
    }

    let da = (args.amax - args.amin) / (args.na as f32 - 1.0);
    let db = (args.bmax - args.bmin) / (args.nb as f32 - 1.0);

    let plane = args.plane.to_ascii_lowercase();
    let headers = match plane.as_str() {
        "xz" => ("x", "z"),
        "yz" => ("y", "z"),
        _ => return Err("plane must be either 'xz' or 'yz'".into()),
    };

    let backend = args.backend.to_ascii_lowercase();
    let gpu_plane = match plane.as_str() {
        "xz" => SectionPlane::Xz,
        "yz" => SectionPlane::Yz,
        _ => unreachable!(),
    };
    let maybe_gpu = match backend.as_str() {
        "cpu" => None,
        "gpu" | "auto" => {
            if let Some(ir) = lower_sdf_ir(result.sdf.as_ref()) {
                sample_section_gpu(
                    &ir,
                    gpu_plane,
                    args.coord,
                    args.amin,
                    args.amax,
                    args.bmin,
                    args.bmax,
                    args.na,
                    args.nb,
                ).ok()
            } else {
                let a0 = args.amin.min(args.amax);
                let a1 = args.amin.max(args.amax);
                let b0 = args.bmin.min(args.bmax);
                let b1 = args.bmin.max(args.bmax);
                let pad = 1.0f32;
                let (bounds_min, bounds_max) = match plane.as_str() {
                    "xz" => (
                        Vec3::new(a0 - pad, args.coord - pad, b0 - pad),
                        Vec3::new(a1 + pad, args.coord + pad, b1 + pad),
                    ),
                    "yz" => (
                        Vec3::new(args.coord - pad, a0 - pad, b0 - pad),
                        Vec3::new(args.coord + pad, a1 + pad, b1 + pad),
                    ),
                    _ => unreachable!(),
                };
                let max_span = (bounds_max - bounds_min).max_element().max(1.0);
                let grid_res = (((args.na.max(args.nb) as f32) * 0.75).ceil() as u32 + 8)
                    .clamp(32, 192)
                    .max(((max_span / 2.0).ceil() as u32).clamp(0, 192));
                let grid = compute_sdf_grid(result.sdf.as_ref(), bounds_min, bounds_max, grid_res);
                sample_section_from_grid_gpu(
                    &grid,
                    gpu_plane,
                    args.coord,
                    args.amin,
                    args.amax,
                    args.bmin,
                    args.bmax,
                    args.na,
                    args.nb,
                ).ok()
            }
        }
        _ => return Err("backend must be one of: auto, cpu, gpu".into()),
    };

    let mut csv = String::with_capacity(args.na * args.nb * 24);
    csv.push_str(&format!("{},{},d\n", headers.0, headers.1));
    if let Some(samples) = maybe_gpu {
        for ib in 0..args.nb {
            let b = args.bmin + ib as f32 * db;
            for ia in 0..args.na {
                let a = args.amin + ia as f32 * da;
                let d = samples[ia + ib * args.na];
                csv.push_str(&format!("{a},{b},{d}\n"));
            }
        }
    } else {
        for ib in 0..args.nb {
            let b = args.bmin + ib as f32 * db;
            for ia in 0..args.na {
                let a = args.amin + ia as f32 * da;
                let p = match plane.as_str() {
                    "xz" => Vec3::new(a, args.coord, b),
                    "yz" => Vec3::new(args.coord, a, b),
                    _ => unreachable!(),
                };
                let d = result.sdf.distance(p);
                csv.push_str(&format!("{a},{b},{d}\n"));
            }
        }
    }

    fs::write(&args.out, csv)
        .map_err(|e| format!("Failed to write {}: {}", args.out.display(), e))?;

    println!(
        "saved={} nx={} nz={} y={}",
        args.out.display(),
        args.na,
        args.nb,
        args.coord
    );
    Ok(())
}
