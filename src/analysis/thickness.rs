// Wall thickness analysis: for each surface voxel, march inward along the
// surface normal and measure how far the ray travels inside the solid.

use std::sync::Arc;
use glam::Vec3;
use rayon::prelude::*;
use crate::render::SdfGrid;

#[derive(Clone)]
pub struct ThicknessResult {
    pub min_thickness: f32,
    pub min_location:  Vec3,
    /// Same layout as SdfGrid (x-fastest). Each value is the local wall
    /// thickness in world units, or 0.0 for non-surface voxels.
    pub analysis_grid: Arc<Vec<f32>>,
    pub bounds_min:    Vec3,
    pub bounds_max:    Vec3,
    pub resolution:    u32,
}

/// Compute per-voxel wall thickness for the given SDF grid.
///
/// `max_display` caps the maximum thickness stored (voxels whose thickness
/// exceeds this get `max_display`).
pub fn compute_thickness(grid: &SdfGrid, max_display: f32) -> ThicknessResult {
    let res    = grid.resolution as usize;
    let span   = (grid.bounds_max - grid.bounds_min).max_element().max(1e-6);
    let voxel  = span / grid.resolution as f32;
    let step   = voxel * 0.5;

    // Trilinear SDF sample — same logic as the WGSL shader so results agree.
    let sdf_at = |p: Vec3| -> f32 {
        let uvw = (p - grid.bounds_min) / (grid.bounds_max - grid.bounds_min);
        let tc  = uvw * Vec3::splat(res as f32) - Vec3::splat(0.5);
        let i   = tc.floor().as_ivec3();
        let f   = tc.fract();

        let load = |di: i32, dj: i32, dk: i32| -> f32 {
            let ci = (i.x + di).clamp(0, res as i32 - 1) as usize;
            let cj = (i.y + dj).clamp(0, res as i32 - 1) as usize;
            let ck = (i.z + dk).clamp(0, res as i32 - 1) as usize;
            grid.data[ci + cj * res + ck * res * res]
        };

        let v000 = load(0,0,0); let v100 = load(1,0,0);
        let v010 = load(0,1,0); let v110 = load(1,1,0);
        let v001 = load(0,0,1); let v101 = load(1,0,1);
        let v011 = load(0,1,1); let v111 = load(1,1,1);

        let x00 = v000 + (v100 - v000) * f.x;
        let x10 = v010 + (v110 - v010) * f.x;
        let x01 = v001 + (v101 - v001) * f.x;
        let x11 = v011 + (v111 - v011) * f.x;
        let y0  = x00  + (x10  - x00)  * f.y;
        let y1  = x01  + (x11  - x01)  * f.y;
        y0 + (y1 - y0) * f.z
    };

    // Direct grid lookup (no interpolation) for gradient via central differences.
    let load_raw = |ix: i32, iy: i32, iz: i32| -> f32 {
        let ci = ix.clamp(0, res as i32 - 1) as usize;
        let cj = iy.clamp(0, res as i32 - 1) as usize;
        let ck = iz.clamp(0, res as i32 - 1) as usize;
        grid.data[ci + cj * res + ck * res * res]
    };

    let max_steps = ((max_display / step) as usize + 4).max(8);

    let out: Vec<f32> = (0..res * res * res)
        .into_par_iter()
        .map(|idx| {
            let ix = (idx % res) as i32;
            let iy = ((idx / res) % res) as i32;
            let iz = (idx / (res * res)) as i32;

            let sdf_val = grid.data[idx];

            // Only surface voxels — skip clearly interior or exterior.
            if sdf_val.abs() > voxel * 1.5 {
                return 0.0_f32;
            }

            // Gradient (central differences) → inward = -normal.
            let nx = load_raw(ix + 1, iy, iz) - load_raw(ix - 1, iy, iz);
            let ny = load_raw(ix, iy + 1, iz) - load_raw(ix, iy - 1, iz);
            let nz = load_raw(ix, iy, iz + 1) - load_raw(ix, iy, iz - 1);
            let n  = Vec3::new(nx, ny, nz);
            let nl = n.length();
            if nl < 1e-6 { return 0.0_f32; }
            let inward = -(n / nl);  // points into the solid

            // World-space voxel centre.
            let t = Vec3::new(
                (ix as f32 + 0.5) / res as f32,
                (iy as f32 + 0.5) / res as f32,
                (iz as f32 + 0.5) / res as f32,
            );
            let p0 = grid.bounds_min + t * (grid.bounds_max - grid.bounds_min);

            // March inward, tracking distance spent inside the solid (SDF < 0).
            let mut p            = p0;
            let mut dist         = 0.0_f32;
            let mut entered      = sdf_val < 0.0;
            let mut inside_dist  = 0.0_f32;

            for _ in 0..max_steps {
                p    += inward * step;
                dist += step;
                let d = sdf_at(p);

                if !entered {
                    if d < 0.0 { entered = true; }
                } else {
                    if d >= 0.0 {
                        // Exited the other side — inside_dist is the thickness.
                        break;
                    }
                    inside_dist += step;
                }

                if dist > max_display + voxel * 2.0 {
                    inside_dist = max_display;
                    break;
                }
            }

            // If we started inside the solid, add the entry offset.
            if sdf_val < 0.0 {
                inside_dist += step;  // one step was already inside at p0
            }

            inside_dist.clamp(0.0, max_display)
        })
        .collect();

    // Find the minimum non-zero thickness to report.
    let mut min_t   = f32::MAX;
    let mut min_idx = 0usize;
    for (i, &v) in out.iter().enumerate() {
        if v > 0.001 && v < min_t {
            min_t   = v;
            min_idx = i;
        }
    }

    let min_location = if min_t < f32::MAX {
        let ix = (min_idx % res) as f32;
        let iy = ((min_idx / res) % res) as f32;
        let iz = (min_idx / (res * res)) as f32;
        let t = Vec3::new(
            (ix + 0.5) / res as f32,
            (iy + 0.5) / res as f32,
            (iz + 0.5) / res as f32,
        );
        grid.bounds_min + t * (grid.bounds_max - grid.bounds_min)
    } else {
        Vec3::ZERO
    };

    ThicknessResult {
        min_thickness:  if min_t < f32::MAX { min_t } else { 0.0 },
        min_location,
        analysis_grid:  Arc::new(out),
        bounds_min:     grid.bounds_min,
        bounds_max:     grid.bounds_max,
        resolution:     grid.resolution,
    }
}
