// Async FEA pipeline: mesh → .inp → ccx subprocess → parse .frd → IDW fields.
//
// Run `FEAPipeline::run()` on a background thread.  Progress messages and the
// final result are returned via std::sync::mpsc channels.

use std::io::Write as IoWrite;
use std::path::PathBuf;
use std::sync::Arc;
use glam::Vec3;
use crate::fea::setup::{FEASetup, FEAConfig};
use crate::fea::meshing::{voxel_tet_mesh, TetMesh};
use crate::fea::inp::write_inp;
use crate::fea::frd::parse_frd;
use crate::fea::calculix::find_calculix;
use crate::sdf::Sdf;
use crate::sdf::field::Field;

// ── Result ────────────────────────────────────────────────────────────────────

/// Per-voxel FEA results interpolated onto the SDF grid.
pub struct FEAGridResult {
    pub displacement: Vec<f32>,   // mm
    pub von_mises:    Vec<f32>,   // MPa
    pub bounds_min:   Vec3,
    pub bounds_max:   Vec3,
    pub resolution:   u32,
    pub max_displacement: f32,
    pub max_von_mises:    f32,
}

/// Arc-wrapped field that reads from a pre-computed grid (trilinear interpolation).
pub struct GridField {
    pub data:       Arc<Vec<f32>>,
    pub bounds_min: Vec3,
    pub bounds_max: Vec3,
    pub resolution: u32,
}

impl Field for GridField {
    fn evaluate(&self, p: Vec3) -> f32 {
        let res = self.resolution as usize;
        let span = self.bounds_max - self.bounds_min;
        let t = (p - self.bounds_min) / span;
        let fx = (t.x * (res as f32 - 1.0)).clamp(0.0, res as f32 - 1.001);
        let fy = (t.y * (res as f32 - 1.0)).clamp(0.0, res as f32 - 1.001);
        let fz = (t.z * (res as f32 - 1.0)).clamp(0.0, res as f32 - 1.001);
        let ix = fx as usize; let rx = fx - ix as f32;
        let iy = fy as usize; let ry = fy - iy as f32;
        let iz = fz as usize; let rz = fz - iz as f32;

        let s = |dx: usize, dy: usize, dz: usize| -> f32 {
            let i = (ix+dx) + (iy+dy)*res + (iz+dz)*res*res;
            self.data.get(i).copied().unwrap_or(0.0)
        };

        // trilinear interpolation
        let c00 = s(0,0,0)*(1.0-rx) + s(1,0,0)*rx;
        let c01 = s(0,0,1)*(1.0-rx) + s(1,0,1)*rx;
        let c10 = s(0,1,0)*(1.0-rx) + s(1,1,0)*rx;
        let c11 = s(0,1,1)*(1.0-rx) + s(1,1,1)*rx;
        let c0  = c00*(1.0-ry) + c10*ry;
        let c1  = c01*(1.0-ry) + c11*ry;
        c0*(1.0-rz) + c1*rz
    }
}

// ── Progress messages ─────────────────────────────────────────────────────────

pub enum FEAMessage {
    /// A log line to display in the UI.
    Log(String),
    /// Pipeline completed successfully.
    Done(Box<FEAGridResult>),
    /// Pipeline failed with an error.
    Error(String),
}

// ── Pipeline ──────────────────────────────────────────────────────────────────

pub struct FEAPipeline {
    pub sdf:             Arc<dyn Sdf>,
    pub bounds_min:      Vec3,
    pub bounds_max:      Vec3,
    pub setup:           FEASetup,
    pub config:          FEAConfig,
    pub ccx_override:    Option<String>,
}

impl FEAPipeline {
    /// Run the full pipeline.  Send `FEAMessage` values to `tx` as progress.
    /// Designed to run on a background thread.
    pub fn run(self, tx: std::sync::mpsc::Sender<FEAMessage>) {
        macro_rules! log {
            ($($t:tt)*) => {
                let _ = tx.send(FEAMessage::Log(format!($($t)*)));
            }
        }
        macro_rules! bail {
            ($($t:tt)*) => {{
                let _ = tx.send(FEAMessage::Error(format!($($t)*)));
                return;
            }}
        }

        // ── Step 1: mesh ─────────────────────────────────────────────────────
        log!("Meshing geometry at resolution {}…", self.config.mesh_resolution);
        let mesh = match voxel_tet_mesh(
            self.sdf.as_ref(),
            self.bounds_min,
            self.bounds_max,
            self.config.mesh_resolution,
        ) {
            Ok(m)  => m,
            Err(e) => bail!("Meshing failed: {e}"),
        };
        log!("Mesh: {} nodes, {} elements", mesh.nodes.len(), mesh.elements.len());

        // ── Step 2: write .inp ───────────────────────────────────────────────
        let tmp_dir = match std::env::temp_dir().join("implicit_cad_fea").into_os_string().into_string() {
            Ok(s) => PathBuf::from(s),
            Err(_) => std::env::temp_dir().join("implicit_cad_fea"),
        };
        if let Err(e) = std::fs::create_dir_all(&tmp_dir) {
            bail!("Could not create temp directory: {e}");
        }

        let job_name = "job";
        let inp_path = tmp_dir.join(format!("{job_name}.inp"));

        log!("Writing CalculiX input file…");
        let inp_content = match write_inp(&mesh, &self.setup, &self.config.material, job_name) {
            Ok(s)  => s,
            Err(e) => bail!("Failed to write .inp: {e}"),
        };
        if let Err(e) = std::fs::write(&inp_path, &inp_content) {
            bail!("Failed to save .inp: {e}");
        }

        // ── Step 3: locate and run CalculiX ─────────────────────────────────
        let ccx_bin = match find_calculix(self.ccx_override.as_deref()) {
            Some(p) => p,
            None    => bail!("CalculiX (ccx) not found. Install it or set the path in Settings."),
        };

        log!("Running CalculiX: {}", ccx_bin.display());

        let output = match std::process::Command::new(&ccx_bin)
            .arg(job_name)
            .current_dir(&tmp_dir)
            .stdout(std::process::Stdio::piped())
            .stderr(std::process::Stdio::piped())
            .output()
        {
            Ok(o)  => o,
            Err(e) => bail!("Failed to launch CalculiX: {e}"),
        };

        // Stream stderr lines to log
        let stderr = String::from_utf8_lossy(&output.stderr);
        for line in stderr.lines() {
            log!("ccx: {line}");
        }

        if !output.status.success() {
            // Try to pull the first ERROR line from stderr
            let first_error = stderr.lines()
                .find(|l| l.to_uppercase().contains("ERROR"))
                .unwrap_or("unknown error");
            bail!("CalculiX exited with error: {first_error}");
        }
        log!("CalculiX completed successfully.");

        // ── Step 4: parse .frd ───────────────────────────────────────────────
        let frd_path = tmp_dir.join(format!("{job_name}.frd"));
        let frd_content = match std::fs::read_to_string(&frd_path) {
            Ok(s)  => s,
            Err(e) => bail!("Could not read results file ({}.frd): {e}", job_name),
        };

        log!("Parsing results…");
        let frd = match parse_frd(&frd_content, mesh.nodes.len()) {
            Ok(r)  => r,
            Err(e) => bail!("Failed to parse .frd: {e}"),
        };

        // ── Step 5: IDW interpolation onto SDF grid ──────────────────────────
        log!("Interpolating results onto SDF grid…");
        let res = self.config.mesh_resolution as usize;
        let grid_result = interpolate_idw(&mesh, &frd.displacement, &frd.von_mises,
            self.bounds_min, self.bounds_max, res);

        let max_displacement = grid_result.displacement.iter().cloned().fold(0.0f32, f32::max);
        let max_von_mises    = grid_result.von_mises.iter().cloned().fold(0.0f32, f32::max);

        log!("Max displacement: {:.4} mm", max_displacement);
        log!("Max Von Mises:    {:.2} MPa", max_von_mises);

        let result = FEAGridResult {
            displacement: grid_result.displacement,
            von_mises:    grid_result.von_mises,
            bounds_min:   self.bounds_min,
            bounds_max:   self.bounds_max,
            resolution:   self.config.mesh_resolution,
            max_displacement,
            max_von_mises,
        };

        let _ = tx.send(FEAMessage::Done(Box::new(result)));
    }
}

// ── IDW interpolation ─────────────────────────────────────────────────────────

struct RawGrid { displacement: Vec<f32>, von_mises: Vec<f32> }

/// Inverse-distance-weighted interpolation from mesh nodes onto a regular grid.
fn interpolate_idw(
    mesh:         &TetMesh,
    displacement: &[f32],
    von_mises:    &[f32],
    bounds_min:   Vec3,
    bounds_max:   Vec3,
    resolution:   usize,
) -> RawGrid {
    let total = resolution * resolution * resolution;
    let span  = bounds_max - bounds_min;
    let step  = span / (resolution as f32 - 1.0).max(1.0);
    let k = 8; // nearest neighbours

    // Build a simple flat nearest-node lookup (acceptable for small grids).
    // For performance: we use k=8 nearest neighbours with IDW (power=2).
    let mut disp_out = vec![0.0f32; total];
    let mut vm_out   = vec![0.0f32; total];

    // Use rayon for parallel grid evaluation
    use rayon::prelude::*;
    let nodes = &mesh.nodes;

    let pairs: Vec<(f32, f32)> = (0..total).into_par_iter().map(|idx| {
        let ix = idx % resolution;
        let iy = (idx / resolution) % resolution;
        let iz = idx / (resolution * resolution);
        let p  = bounds_min + Vec3::new(
            ix as f32 * step.x,
            iy as f32 * step.y,
            iz as f32 * step.z,
        );

        // Find k nearest mesh nodes
        let mut dists: Vec<(f32, usize)> = nodes.iter().enumerate()
            .map(|(i, &n)| ((n - p).length_squared(), i))
            .collect();
        dists.sort_unstable_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        let k_actual = k.min(dists.len());

        let mut w_sum  = 0.0f32;
        let mut d_sum  = 0.0f32;
        let mut vm_sum = 0.0f32;

        for &(dist2, ni) in &dists[..k_actual] {
            if dist2 < 1e-10 {
                // Exactly on a node
                return (displacement[ni], von_mises[ni]);
            }
            let w = 1.0 / dist2; // IDW power = 2
            w_sum  += w;
            d_sum  += w * displacement[ni];
            vm_sum += w * von_mises[ni];
        }

        if w_sum > 0.0 {
            (d_sum / w_sum, vm_sum / w_sum)
        } else {
            (0.0, 0.0)
        }
    }).collect();

    for (idx, (d, vm)) in pairs.into_iter().enumerate() {
        disp_out[idx] = d;
        vm_out[idx]   = vm;
    }

    RawGrid { displacement: disp_out, von_mises: vm_out }
}
