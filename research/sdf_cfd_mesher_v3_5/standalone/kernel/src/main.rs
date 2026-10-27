mod aero;
mod analysis;
mod app;
mod components;
mod export;
mod fea;
mod geometry_analysis;
mod gpu;
mod headless;
mod library;
mod materials;
mod mesh;
mod pipeline;
mod project;
mod render;
mod scripting;
mod sdf;
mod settings;
mod ui;
mod undo;
mod version_control;

use clap::Parser;
use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::thread;
use std::time::{Duration, Instant};

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

    /// Export format: stl, obj, or package (headless mode)
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

    /// Aero export mode for --format aero
    #[arg(long, default_value = "external")]
    aero_mode: String,

    /// Aero adaptive target error in mm
    #[arg(long, default_value = "0.5")]
    aero_target_error_mm: f32,

    /// Aero adaptive minimum cell size in mm
    #[arg(long, default_value = "1.0")]
    aero_min_cell_mm: f32,

    /// Aero adaptive maximum octree depth
    #[arg(long, default_value = "6")]
    aero_max_depth: u32,

    /// Also write OBJ per aero patch
    #[arg(long)]
    aero_write_obj: bool,

    /// Minimal fast CFD export: write only patch files + manifest, skip composite/report/helper outputs
    #[arg(long)]
    aero_fast_mode: bool,

    /// Force dense uniform grid sampling for aero export and bypass scout/adaptive backend selection
    #[arg(long)]
    aero_uniform_reference: bool,

    /// Kill headless aero export if it runs longer than this many seconds
    #[arg(long, default_value = "600")]
    aero_timeout_seconds: u64,

    /// Abort a patch before meshing if its estimated equivalent uniform grid exceeds this many voxels
    #[arg(long, default_value = "128000000")]
    aero_max_patch_voxels: u64,

    #[arg(long, hide = true)]
    aero_worker: bool,
}

fn mesh_quality_resolution(mesh_quality: &str) -> Result<u32, String> {
    match mesh_quality.to_ascii_lowercase().as_str() {
        "draft" => Ok(24),
        "normal" => Ok(32),
        "fine" => Ok(48),
        "ultra" => Ok(64),
        other => Err(format!(
            "Unknown mesh quality '{}'. Use draft, normal, fine, or ultra.",
            other
        )),
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Verify CalculiX binary at startup (logs a warning if not found).
    let startup_settings = settings::AppSettings::load();
    fea::calculix::verify_calculix(startup_settings.ccx_path.as_deref());

    let args = Args::parse();
    let mesh_quality_resolution = mesh_quality_resolution(&args.mesh_quality)?;
    let resolution = if args.resolution == 32 {
        mesh_quality_resolution
    } else {
        args.resolution
    };

    if args.headless
        && args.format.eq_ignore_ascii_case("aero")
        && !args.aero_worker
        && args.script.is_some()
    {
        let exe = std::env::current_exe()?;
        let mut child_args: Vec<_> = std::env::args_os().skip(1).collect();
        child_args.push("--aero-worker".into());
        let mut child = Command::new(exe)
            .args(child_args)
            .stdin(Stdio::null())
            .stdout(Stdio::inherit())
            .stderr(Stdio::inherit())
            .spawn()?;
        let timeout = Duration::from_secs(args.aero_timeout_seconds.max(1));
        let start = Instant::now();
        loop {
            if let Some(status) = child.try_wait()? {
                if status.success() {
                    return Ok(());
                }
                return Err(format!("Headless aero worker exited with status {}", status).into());
            }
            if start.elapsed() >= timeout {
                let _ = child.kill();
                let _ = child.wait();
                return Err(format!(
                    "Headless aero export exceeded {} seconds and was terminated",
                    args.aero_timeout_seconds.max(1)
                )
                .into());
            }
            thread::sleep(Duration::from_millis(250));
        }
    }

    if args.headless {
        // Parse dimension overrides
        let dim_overrides: Vec<(String, f64)> = args
            .dim
            .iter()
            .filter_map(|s| {
                let mut parts = s.splitn(2, '=');
                let name = parts.next()?.trim().to_string();
                let value = parts.next()?.trim().parse::<f64>().ok()?;
                Some((name, value))
            })
            .collect();

        // Headless mode
        if let Some(batch_dir) = args.batch {
            let output_dir = args
                .batch_output
                .unwrap_or_else(|| PathBuf::from("./output"));
            headless::execute_batch(
                &batch_dir,
                &output_dir,
                &args.format,
                resolution,
                args.smooth_normals,
                &args.aero_mode,
                args.aero_target_error_mm,
                args.aero_min_cell_mm,
                args.aero_max_depth,
                args.aero_write_obj,
                args.aero_fast_mode,
                args.aero_uniform_reference,
                args.aero_max_patch_voxels,
            )?;
            Ok(())
        } else if let Some(script_path) = args.script {
            headless::execute_script_headless_extended(
                &script_path,
                args.output.as_deref(),
                &args.format,
                resolution,
                args.smooth_normals,
                &dim_overrides,
                args.output_metrics.as_deref(),
                &args.aero_mode,
                args.aero_target_error_mm,
                args.aero_min_cell_mm,
                args.aero_max_depth,
                args.aero_write_obj,
                args.aero_fast_mode,
                args.aero_uniform_reference,
                args.aero_max_patch_voxels,
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
            viewport: egui::ViewportBuilder::default().with_inner_size([1200.0, 800.0]),
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
