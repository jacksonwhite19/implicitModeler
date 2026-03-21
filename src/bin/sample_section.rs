use std::fs;
use std::path::PathBuf;

use clap::Parser;
use glam::Vec3;
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

    let mut csv = String::with_capacity(args.na * args.nb * 24);
    csv.push_str(&format!("{},{},d\n", headers.0, headers.1));
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
