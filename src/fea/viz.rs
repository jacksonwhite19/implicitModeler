// Pre-computed FEA boundary-condition visualization data.
//
// Sampled once after script evaluation so the display is cheap every frame.
// Each region is represented by a centroid and a small point cloud (interior samples)
// for surface-overlay rendering.  Direction vectors are kept for force/torque arrows.

use glam::Vec3;
use crate::fea::setup::FEASetup;

// ── Per-region display data ───────────────────────────────────────────────────

pub struct RegionViz {
    pub name:     String,
    pub points:   Vec<Vec3>,  // interior sample points for dot-cloud overlay
    pub centroid: Vec3,
}

// ── Full set of BC visualization data ────────────────────────────────────────

pub struct FEAVizData {
    pub fixed_supports:  Vec<RegionViz>,
    pub force_loads:     Vec<(RegionViz, Vec3)>,      // (region, normalized force direction)
    pub force_magnitudes:Vec<f32>,                     // N per force load
    pub pressure_loads:  Vec<RegionViz>,
    pub torque_loads:    Vec<(RegionViz, Vec3)>,       // (region, axis)
    pub motor_thrusts:   Vec<(RegionViz, Vec3)>,       // (region, thrust direction)
    pub gravity:         Option<Vec3>,                  // normalized gravity direction
}

// ── Sampling ──────────────────────────────────────────────────────────────────

/// Pre-compute visualization geometry from `setup` within `bounds_min..bounds_max`.
///
/// Uses a low-resolution 20³ grid; keeps up to `max_pts` interior points per region.
pub fn compute_fea_viz(
    setup:      &FEASetup,
    bounds_min: Vec3,
    bounds_max: Vec3,
) -> FEAVizData {
    const GRID: usize   = 20;
    const MAX_PTS: usize = 150;

    let span = bounds_max - bounds_min;
    let step = span / (GRID as f32 - 1.0).max(1.0);

    // Build flat grid of sample points once.
    let mut grid_pts: Vec<Vec3> = Vec::with_capacity(GRID * GRID * GRID);
    for iz in 0..GRID { for iy in 0..GRID { for ix in 0..GRID {
        grid_pts.push(bounds_min + Vec3::new(
            ix as f32 * step.x,
            iy as f32 * step.y,
            iz as f32 * step.z,
        ));
    }}}

    let sample_region = |sdf: &dyn crate::sdf::Sdf| -> RegionViz {
        let inside: Vec<Vec3> = grid_pts.iter()
            .filter(|&&p| sdf.distance(p) < 0.0)
            .copied()
            .collect();

        // Thin down to MAX_PTS evenly spaced.
        let step_pts = (inside.len() / MAX_PTS).max(1);
        let points: Vec<Vec3> = inside.iter().step_by(step_pts).copied().collect();

        let centroid = if points.is_empty() {
            (bounds_min + bounds_max) * 0.5
        } else {
            points.iter().copied().sum::<Vec3>() / points.len() as f32
        };

        RegionViz { name: String::new(), points, centroid }
    };

    // Fixed supports
    let fixed_supports: Vec<RegionViz> = setup.fixed_supports.iter().map(|r| {
        let mut v = sample_region(r.sdf.as_ref());
        v.name = r.name.clone();
        v
    }).collect();

    // Force loads
    let mut force_loads: Vec<(RegionViz, Vec3)> = Vec::new();
    let mut force_magnitudes: Vec<f32>           = Vec::new();
    for r in &setup.force_loads {
        let mut v = sample_region(r.sdf.as_ref());
        v.name = r.name.clone();
        force_loads.push((v, r.force.normalize_or_zero()));
        force_magnitudes.push(r.force.length());
    }

    // Pressure loads
    let pressure_loads: Vec<RegionViz> = setup.pressure_loads.iter().map(|r| {
        let mut v = sample_region(r.sdf.as_ref());
        v.name = r.name.clone();
        v
    }).collect();

    // Torque loads
    let torque_loads: Vec<(RegionViz, Vec3)> = setup.torque_loads.iter().map(|r| {
        let mut v = sample_region(r.sdf.as_ref());
        v.name = r.name.clone();
        (v, r.axis)
    }).collect();

    // Motor thrusts
    let motor_thrusts: Vec<(RegionViz, Vec3)> = setup.motor_thrusts.iter().map(|r| {
        let mut v = sample_region(r.sdf.as_ref());
        v.name = r.name.clone();
        (v, r.direction)
    }).collect();

    FEAVizData {
        fixed_supports,
        force_loads,
        force_magnitudes,
        pressure_loads,
        torque_loads,
        motor_thrusts,
        gravity: setup.gravity.map(|g| g.normalize_or_zero()),
    }
}
