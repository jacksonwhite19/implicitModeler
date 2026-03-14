mod app;
mod sdf;
mod mesh;
mod render;
mod scripting;
mod export;
mod project;
mod headless;
mod components;
mod node_graph;
mod notebook;
mod ui;
mod analysis;
mod fea;
mod settings;

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
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Verify CalculiX binary at startup (logs a warning if not found).
    let startup_settings = settings::AppSettings::load();
    fea::calculix::verify_calculix(startup_settings.ccx_path.as_deref());

    let args = Args::parse();

    if args.headless {
        // Headless mode
        if let Some(batch_dir) = args.batch {
            let output_dir = args.batch_output.unwrap_or_else(|| PathBuf::from("./output"));
            headless::execute_batch(&batch_dir, &output_dir, &args.format, args.resolution, args.smooth_normals)?;
            Ok(())
        } else if let Some(script_path) = args.script {
            let output_path = args.output.ok_or("--output is required in headless mode")?;
            headless::execute_script_headless(&script_path, &output_path, &args.format, args.resolution, args.smooth_normals)?;
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
