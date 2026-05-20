use std::fs;
use std::path::PathBuf;

use clap::Parser;
use glam::Vec3;
use implicit_cad::scripting::evaluate_aero_export_parts;
use indexmap::IndexMap;

#[derive(Parser, Debug)]
#[command(name = "sample-aero-part-points")]
struct Args {
    #[arg(long)]
    script: PathBuf,
    #[arg(long, default_value = "aircraft_oml")]
    part: String,
    #[arg(long, default_value = "part")]
    field: String,
    #[arg(long)]
    x: f32,
    #[arg(long)]
    y: f32,
    #[arg(long, num_args = 1..)]
    z: Vec<f32>,
}

fn main() -> Result<(), String> {
    let args = Args::parse();
    let source = fs::read_to_string(&args.script)
        .map_err(|e| format!("Failed to read script {}: {}", args.script.display(), e))?;
    let dimensions = IndexMap::new();
    let parts = evaluate_aero_export_parts(
        &source,
        None,
        None,
        None,
        None,
        &dimensions,
        args.script.parent(),
        None,
        &[],
    )?;
    let part = parts
        .iter()
        .find(|p| p.name == args.part)
        .ok_or_else(|| format!("No aero part named '{}'", args.part))?;
    let sdf: &dyn implicit_cad::sdf::Sdf = match args.field.as_str() {
        "part" => part.sdf.as_ref(),
        "ref_body" => part
            .refinement_body_sdf
            .as_ref()
            .ok_or("Part has no refinement_body_sdf")?
            .as_ref(),
        "ref_void" => part
            .refinement_void_sdf
            .as_ref()
            .ok_or("Part has no refinement_void_sdf")?
            .as_ref(),
        other => {
            return Err(format!(
                "Unknown field '{}'; use part, ref_body, or ref_void",
                other
            ));
        }
    };

    println!("part: {}", part.name);
    println!("field: {}", args.field);
    println!("x: {}", args.x);
    println!("y: {}", args.y);
    for z in args.z {
        let p = Vec3::new(args.x, args.y, z);
        println!("z={:.6} d={:.6}", z, sdf.distance(p));
    }
    Ok(())
}
