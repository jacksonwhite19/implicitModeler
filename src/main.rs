mod app;
mod version_control;
mod materials;
mod sdf;
mod mesh;
mod render;
mod scripting;
mod export;
mod project;
mod headless;
mod components;
mod ui;
mod analysis;
mod geometry_analysis;
mod aero;
mod fea;
mod settings;
mod undo;
mod library;

use clap::Parser;
use std::path::PathBuf;

#[derive(Parser)]
#[command(name = "implicit-cad")]
#[command(about = "Code-first implicit CAD modeler using signed distance fields")]
struct Args {
    /// Run in headless mode (no GUI)
    #[arg(long)]
    headless: bool,

    /// Script file to execute (headless mode)
    #[arg(long, value_name = "FILE")]
    script: Option<PathBuf>,

    /// Output file path (headless mode)
    #[arg(long, value_name = "FILE")]
    output: Option<PathBuf>,

    /// Export format: stl or obj (headless mode)
    #[arg(long, value_name = "FORMAT", default_value = "stl")]
    format: String,

    /// Batch process directory (headless mode)
    #[arg(long, value_name = "DIR")]
    batch: Option<PathBuf>,

    /// Output directory for batch mode
    #[arg(long, value_name = "DIR")]
    batch_output: Option<PathBuf>,

    /// Mesh resolution (grid size)
    #[arg(long, default_value = "32")]
    resolution: u32,

    /// Use smooth normals
    #[arg(long)]
    smooth_normals: bool,

    /// Override a named dimension (repeatable: --dim wingspan=820)
    #[arg(long = "dim", value_name = "NAME=VALUE", action = clap::ArgAction::Append)]
    dim: Vec<String>,

    /// Write metrics JSON to this path
    #[arg(long, value_name = "FILE")]
    output_metrics: Option<PathBuf>,

    /// Mesh quality: draft, normal, fine, ultra
    #[arg(long, default_value = "normal")]
    mesh_quality: String,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Verify CalculiX binary at startup (logs a warning if not found).
    let startup_settings = settings::AppSettings::load();
    fea::calculix::verify_calculix(startup_settings.ccx_path.as_deref());

    let args = Args::parse();

    if args.headless {
        // Parse dimension overrides
        let dim_overrides: Vec<(String, f64)> = args.dim.iter().filter_map(|s| {
            let mut parts = s.splitn(2, '=');
            let name  = parts.next()?.trim().to_string();
            let value = parts.next()?.trim().parse::<f64>().ok()?;
            Some((name, value))
        }).collect();

        // Headless mode
        if let Some(batch_dir) = args.batch {
            let output_dir = args.batch_output.unwrap_or_else(|| PathBuf::from("./output"));
            headless::execute_batch(&batch_dir, &output_dir, &args.format, args.resolution, args.smooth_normals)?;
            Ok(())
        } else if let Some(script_path) = args.script {
            headless::execute_script_headless_extended(
                &script_path,
                args.output.as_deref(),
                &args.format,
                args.resolution,
                args.smooth_normals,
                &dim_overrides,
                args.output_metrics.as_deref(),
            )?;
            Ok(())
        } else {
            eprintln!("Error: --headless requires either --script or --batch");
            std::process::exit(1);
        }
    } else {
        // GUI mode
        use eframe::egui;
        let options = eframe::NativeOptions {
            viewport: egui::ViewportBuilder::default()
                .with_inner_size([1200.0, 800.0]),
            renderer: eframe::Renderer::Wgpu,
            ..Default::default()
        };

        eframe::run_native(
            "Implicit CAD",
            options,
            Box::new(|cc| Ok(Box::new(app::App::new(cc)))),
        )?;
        Ok(())
    }
}
