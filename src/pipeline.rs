use crate::render::SdfGrid;
use crate::sdf::Sdf;
use glam::Vec3;
use rayon::prelude::*;

pub fn auto_bounds(sdf: &dyn Sdf) -> (Vec3, Vec3) {
    fn scan_bounds(
        sdf: &dyn Sdf,
        min: Vec3,
        max: Vec3,
        step: Vec3,
        phase: Vec3,
    ) -> Option<(Vec3, Vec3)> {
        let nx = ((max.x - min.x) / step.x).floor() as i32 + 1;
        let ny = ((max.y - min.y) / step.y).floor() as i32 + 1;
        let nz = ((max.z - min.z) / step.z).floor() as i32 + 1;
        let total = (nx * ny * nz) as usize;

        let (lo, hi) = (0..total)
            .into_par_iter()
            .map(|idx| {
                let iz = (idx / (nx * ny) as usize) as i32;
                let iy = ((idx / nx as usize) % ny as usize) as i32;
                let ix = (idx % nx as usize) as i32;
                let p = Vec3::new(
                    min.x + (ix as f32 + phase.x) * step.x,
                    min.y + (iy as f32 + phase.y) * step.y,
                    min.z + (iz as f32 + phase.z) * step.z,
                );
                if sdf.distance(p) < 0.0 {
                    (p, p)
                } else {
                    (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN))
                }
            })
            .reduce(
                || (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)),
                |(lo1, hi1), (lo2, hi2)| (lo1.min(lo2), hi1.max(hi2)),
            );

        if lo.x == f32::MAX { None } else { Some((lo, hi)) }
    }

    fn phased_bounds(
        sdf: &dyn Sdf,
        min: Vec3,
        max: Vec3,
        step: Vec3,
        phases: &[Vec3],
    ) -> Option<(Vec3, Vec3)> {
        let mut overall_lo = Vec3::splat(f32::MAX);
        let mut overall_hi = Vec3::splat(f32::MIN);
        let mut found = false;
        for &phase in phases {
            if let Some((lo, hi)) = scan_bounds(sdf, min, max, step, phase) {
                overall_lo = overall_lo.min(lo);
                overall_hi = overall_hi.max(hi);
                found = true;
            }
        }
        if found { Some((overall_lo, overall_hi)) } else { None }
    }

    let phases = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.5, 0.0, 0.0),
        Vec3::new(0.0, 0.5, 0.0),
        Vec3::new(0.0, 0.0, 0.5),
        Vec3::new(0.5, 0.5, 0.5),
    ];

    // Aircraft-oriented broad scan. The previous centerline probe missed translated wings.
    let coarse_min = Vec3::new(-800.0, -800.0, -300.0);
    let coarse_max = Vec3::new(1200.0, 800.0, 300.0);
    let coarse_step = Vec3::new(20.0, 20.0, 10.0);

    let Some((coarse_lo, coarse_hi)) = phased_bounds(sdf, coarse_min, coarse_max, coarse_step, &phases) else {
        return (Vec3::splat(-100.0), Vec3::splat(100.0));
    };

    // Refine around the occupied coarse region so the viewport box is tighter without
    // depending on a single interior seed line that can miss swept wings.
    let refine_min = coarse_lo - coarse_step * 2.0;
    let refine_max = coarse_hi + coarse_step * 2.0;
    let fine_step = Vec3::new(5.0, 5.0, 2.5);
    let (bb_min, bb_max) = phased_bounds(sdf, refine_min, refine_max, fine_step, &phases)
        .unwrap_or((coarse_lo, coarse_hi));

    let span = (bb_max - bb_min).max(Vec3::splat(1.0));
    let pad = span * 0.08 + Vec3::splat(1.0);
    let lo = (bb_min - pad).max(Vec3::new(-1200.0, -1200.0, -500.0));
    let hi = (bb_max + pad).min(Vec3::new( 1400.0,  1200.0,  500.0));
    (lo, hi)
}

pub fn compute_sdf_grid(
    sdf: &dyn Sdf,
    bounds_min: Vec3,
    bounds_max: Vec3,
    res: u32,
) -> SdfGrid {
    let step = (bounds_max - bounds_min) / res as f32;
    let total = (res * res * res) as usize;

    let data: Vec<f32> = (0..total)
        .into_par_iter()
        .map(|idx| {
            let x = (idx % res as usize) as u32;
            let y = ((idx / res as usize) % res as usize) as u32;
            let z = (idx / (res * res) as usize) as u32;
            let p = bounds_min + Vec3::new(
                (x as f32 + 0.5) * step.x,
                (y as f32 + 0.5) * step.y,
                (z as f32 + 0.5) * step.z,
            );
            sdf.distance(p)
        })
        .collect();

    SdfGrid { data, resolution: res, bounds_min, bounds_max }
}

pub fn tight_bounds_from_grid(grid: &SdfGrid) -> Option<(Vec3, Vec3)> {
    let step = (grid.bounds_max - grid.bounds_min) / grid.resolution as f32;
    let res = grid.resolution as usize;
    let mut tight_min = grid.bounds_max;
    let mut tight_max = grid.bounds_min;

    for iz in 0..res {
        for iy in 0..res {
            for ix in 0..res {
                if grid.data[ix + iy * res + iz * res * res] < 0.0 {
                    let p = grid.bounds_min + Vec3::new(
                        (ix as f32 + 0.5) * step.x,
                        (iy as f32 + 0.5) * step.y,
                        (iz as f32 + 0.5) * step.z,
                    );
                    tight_min = tight_min.min(p);
                    tight_max = tight_max.max(p);
                }
            }
        }
    }

    if tight_min.x <= tight_max.x {
        Some((tight_min, tight_max))
    } else {
        None
    }
}
