// Measurement tools: volume, surface area, center of mass, cross-section, point distance.

use std::sync::Arc;
use glam::Vec3;
use rayon::prelude::*;
use crate::render::SdfGrid;
use crate::sdf::Sdf;
use crate::project::Axis;

// ── Data model ────────────────────────────────────────────────────────────────

#[derive(Clone, Default)]
pub struct MeasurementResults {
    pub volume_mm3:       Option<f32>,
    pub surface_area_mm2: Option<f32>,
    pub center_of_mass:   Option<Vec3>,
    pub print_mass_g:     Option<f32>,
    pub cross_sections:   Vec<CrossSectionMeasurement>,
    pub point_distances:  Vec<PointDistanceMeasurement>,
}

#[derive(Clone)]
pub struct CrossSectionMeasurement {
    pub label:    String,
    pub axis:     Axis,
    pub position: f32,
    pub area_mm2: f32,
}

/// How a distance measurement is computed / reported.
#[derive(Clone, PartialEq, Default, Debug)]
pub enum DistanceMeasureKind {
    /// Straight-line 3-D Euclidean distance.
    #[default]
    Distance3D,
    /// Component along the X axis.
    ProjectedX,
    /// Component along the Y axis.
    ProjectedY,
    /// Component along the Z axis.
    ProjectedZ,
    /// Component along a user-defined unit vector.
    CustomVector,
    /// Distance in the XY plane (ignores Z).
    RadialXY,
    /// Distance in the XZ plane (ignores Y).
    RadialXZ,
    /// Distance in the YZ plane (ignores X).
    RadialYZ,
    /// Closest approach from point A to the SDF isosurface.
    MinToSurface,
    /// Acute angle between vector A→B and each cardinal axis (reports in degrees).
    Angle,
}

impl DistanceMeasureKind {
    pub fn label(&self) -> &'static str {
        match self {
            Self::Distance3D    => "3D Distance",
            Self::ProjectedX    => "Projected X",
            Self::ProjectedY    => "Projected Y",
            Self::ProjectedZ    => "Projected Z",
            Self::CustomVector  => "Custom Vector",
            Self::RadialXY      => "Radial (XY plane)",
            Self::RadialXZ      => "Radial (XZ plane)",
            Self::RadialYZ      => "Radial (YZ plane)",
            Self::MinToSurface  => "Min to Surface",
            Self::Angle         => "Angle",
        }
    }

    pub fn all() -> &'static [Self] {
        &[
            Self::Distance3D, Self::ProjectedX, Self::ProjectedY, Self::ProjectedZ,
            Self::CustomVector, Self::RadialXY, Self::RadialXZ, Self::RadialYZ,
            Self::MinToSurface, Self::Angle,
        ]
    }
}

#[derive(Clone)]
pub struct PointDistanceMeasurement {
    pub label:        String,
    pub point_a:      Vec3,
    pub point_b:      Vec3,
    pub kind:         DistanceMeasureKind,
    /// Custom projection vector (unit), used when kind == CustomVector.
    pub custom_vec:   Option<Vec3>,
    // Cached derived values
    pub distance_3d:  f32,
    pub delta:        Vec3,
    /// Primary reported value (depends on kind).
    pub primary_mm:   f32,
    /// Angle of A→B vector with X/Y/Z axes in degrees, when kind == Angle.
    pub angles_deg:   Option<[f32; 3]>,
}

impl PointDistanceMeasurement {
    /// Compute a measurement from two points and a kind.
    pub fn compute(
        label:      String,
        a:          Vec3,
        b:          Vec3,
        kind:       DistanceMeasureKind,
        custom_vec: Option<Vec3>,
        sdf:        Option<&dyn Sdf>,
    ) -> Self {
        let delta    = b - a;
        let dist_3d  = delta.length();

        let (primary, angles) = match &kind {
            DistanceMeasureKind::Distance3D   => (dist_3d, None),
            DistanceMeasureKind::ProjectedX   => (delta.x.abs(), None),
            DistanceMeasureKind::ProjectedY   => (delta.y.abs(), None),
            DistanceMeasureKind::ProjectedZ   => (delta.z.abs(), None),
            DistanceMeasureKind::CustomVector => {
                let v = custom_vec.unwrap_or(Vec3::X).normalize_or(Vec3::X);
                (delta.dot(v).abs(), None)
            }
            DistanceMeasureKind::RadialXY => (Vec3::new(delta.x, delta.y, 0.0).length(), None),
            DistanceMeasureKind::RadialXZ => (Vec3::new(delta.x, 0.0, delta.z).length(), None),
            DistanceMeasureKind::RadialYZ => (Vec3::new(0.0, delta.y, delta.z).length(), None),
            DistanceMeasureKind::MinToSurface => {
                let d = sdf.map(|s| s.distance(a).abs()).unwrap_or(f32::NAN);
                (d, None)
            }
            DistanceMeasureKind::Angle => {
                let dir = if dist_3d > 1e-6 { delta / dist_3d } else { Vec3::X };
                let ax = dir.dot(Vec3::X).clamp(-1.0, 1.0).acos().to_degrees();
                let ay = dir.dot(Vec3::Y).clamp(-1.0, 1.0).acos().to_degrees();
                let az = dir.dot(Vec3::Z).clamp(-1.0, 1.0).acos().to_degrees();
                (ax.min(180.0 - ax), Some([ax, ay, az]))  // smallest angle with X axis as primary
            }
        };

        Self {
            label, point_a: a, point_b: b, kind,
            custom_vec, distance_3d: dist_3d, delta,
            primary_mm: primary, angles_deg: angles,
        }
    }
}

// ── Model properties (volume, surface area, COM) ──────────────────────────────

/// Single parallel pass over the SDF grid: compute volume, surface area, and
/// center of mass simultaneously. Returns (volume_mm³, surface_area_mm², com).
pub fn compute_model_properties(grid: &SdfGrid) -> (f32, f32, Vec3) {
    let res     = grid.resolution as usize;
    let span    = grid.bounds_max - grid.bounds_min;
    let dx      = span.x / res as f32;
    let dy      = span.y / res as f32;
    let dz      = span.z / res as f32;
    let dv      = dx * dy * dz;            // voxel volume
    let voxel_d = (dx * dy * dz).cbrt();   // characteristic voxel size

    let idx = |ix: usize, iy: usize, iz: usize| ix + iy * res + iz * res * res;

    // Partial sums per row (z-major).
    let rows: Vec<(f64, f64, f64, f64, f64)> = (0..res).into_par_iter().map(|iz| {
        let mut vol = 0.0f64;
        let mut sa  = 0.0f64;
        let mut cx  = 0.0f64;
        let mut cy  = 0.0f64;
        let mut cz  = 0.0f64;
        for iy in 0..res {
            for ix in 0..res {
                let d = grid.data[idx(ix, iy, iz)];
                if d < 0.0 {
                    vol += dv as f64;
                    let px = grid.bounds_min.x + (ix as f32 + 0.5) * dx;
                    let py = grid.bounds_min.y + (iy as f32 + 0.5) * dy;
                    let pz = grid.bounds_min.z + (iz as f32 + 0.5) * dz;
                    cx += px as f64 * dv as f64;
                    cy += py as f64 * dv as f64;
                    cz += pz as f64 * dv as f64;
                }
                // Surface area: Cauchy-Crofton on near-surface voxels.
                if d.abs() < voxel_d * 1.5 {
                    // Gradient magnitude via central differences.
                    let gx = {
                        let xa = if ix > 0         { grid.data[idx(ix-1, iy, iz)] } else { d };
                        let xb = if ix < res-1     { grid.data[idx(ix+1, iy, iz)] } else { d };
                        (xb - xa) / (2.0 * dx)
                    };
                    let gy = {
                        let ya = if iy > 0         { grid.data[idx(ix, iy-1, iz)] } else { d };
                        let yb = if iy < res-1     { grid.data[idx(ix, iy+1, iz)] } else { d };
                        (yb - ya) / (2.0 * dy)
                    };
                    let gz = {
                        let za = if iz > 0         { grid.data[idx(ix, iy, iz-1)] } else { d };
                        let zb = if iz < res-1     { grid.data[idx(ix, iy, iz+1)] } else { d };
                        (zb - za) / (2.0 * dz)
                    };
                    let grad_mag = (gx*gx + gy*gy + gz*gz).sqrt();
                    // Integrate |∇φ| δ(φ) dV, approximated as |∇φ| * dV / (voxel_d)
                    sa += (grad_mag * dv as f32 / voxel_d) as f64;
                }
            }
        }
        (vol, sa, cx, cy, cz)
    }).collect();

    let (vol, sa, cx, cy, cz) = rows.iter().fold((0.0f64,0.0f64,0.0f64,0.0f64,0.0f64),
        |a, b| (a.0+b.0, a.1+b.1, a.2+b.2, a.3+b.3, a.4+b.4));

    let com = if vol > 0.0 {
        Vec3::new((cx/vol) as f32, (cy/vol) as f32, (cz/vol) as f32)
    } else {
        Vec3::ZERO
    };

    (vol as f32, sa as f32, com)
}

// ── Cross-section area ────────────────────────────────────────────────────────

/// Sample a 2D slice of the SDF perpendicular to `axis` at `position`,
/// count interior points, and return the estimated area in mm².
/// Resolution: number of sample points per side (e.g. 256).
pub fn measure_cross_section(
    sdf:        Arc<dyn Sdf>,
    axis:       &Axis,
    position:   f32,
    bounds_min: Vec3,
    bounds_max: Vec3,
    resolution: usize,
) -> f32 {
    let res = resolution;
    // Determine the two axes spanning the slice plane.
    let (u_min, u_max, v_min, v_max) = match axis {
        Axis::X => (bounds_min.y, bounds_max.y, bounds_min.z, bounds_max.z),
        Axis::Y => (bounds_min.x, bounds_max.x, bounds_min.z, bounds_max.z),
        Axis::Z => (bounds_min.x, bounds_max.x, bounds_min.y, bounds_max.y),
    };
    let du = (u_max - u_min) / res as f32;
    let dv = (v_max - v_min) / res as f32;
    let cell_area = du * dv;

    let inside: u64 = (0..res).into_par_iter().map(|iv| {
        let mut count = 0u64;
        let v = v_min + (iv as f32 + 0.5) * dv;
        for iu in 0..res {
            let u = u_min + (iu as f32 + 0.5) * du;
            let p = match axis {
                Axis::X => Vec3::new(position, u, v),
                Axis::Y => Vec3::new(u, position, v),
                Axis::Z => Vec3::new(u, v, position),
            };
            if sdf.distance(p) < 0.0 { count += 1; }
        }
        count
    }).sum();

    inside as f32 * cell_area
}

// ── Point distance ────────────────────────────────────────────────────────────

pub fn measure_distance(a: Vec3, b: Vec3) -> f32 {
    (b - a).length()
}

/// Snap a point to the nearest point on the SDF isosurface by gradient descent.
/// Returns the surface point after up to `max_steps` iterations.
pub fn snap_to_surface(sdf: &dyn Sdf, p: Vec3, max_steps: usize) -> Vec3 {
    let eps = 1e-3f32;
    let mut q = p;
    for _ in 0..max_steps {
        let d = sdf.distance(q);
        if d.abs() < eps { break; }
        // Estimate gradient via central differences
        let gx = (sdf.distance(q + Vec3::X * eps) - sdf.distance(q - Vec3::X * eps)) / (2.0 * eps);
        let gy = (sdf.distance(q + Vec3::Y * eps) - sdf.distance(q - Vec3::Y * eps)) / (2.0 * eps);
        let gz = (sdf.distance(q + Vec3::Z * eps) - sdf.distance(q - Vec3::Z * eps)) / (2.0 * eps);
        let grad = Vec3::new(gx, gy, gz);
        let grad_len = grad.length();
        if grad_len < 1e-6 { break; }
        q -= grad / grad_len * d; // move by signed distance along gradient
    }
    q
}

// ── Ray-SDF grid intersection (for viewport picking) ─────────────────────────

/// March a ray through the SDF grid and return the approximate hit point, or None.
/// Uses the grid's trilinear-interpolated SDF for the march.
pub fn ray_march_grid(
    grid:    &SdfGrid,
    origin:  Vec3,
    dir:     Vec3,    // normalised
    max_dist: f32,
) -> Option<Vec3> {
    let mut t = 0.0f32;
    let step_min = (grid.bounds_max - grid.bounds_min).min_element() / grid.resolution as f32;

    for _ in 0..256 {
        let p = origin + dir * t;
        let d = sample_grid(grid, p);
        if d < step_min * 0.5 {
            return Some(p);
        }
        t += d.max(step_min);
        if t > max_dist { break; }
    }
    None
}

/// Trilinear sample of the SDF grid at world-space point `p`.
pub fn sample_grid(grid: &SdfGrid, p: Vec3) -> f32 {
    let res = grid.resolution as usize;
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
    let x00 = v000 + (v100-v000)*f.x; let x10 = v010 + (v110-v010)*f.x;
    let x01 = v001 + (v101-v001)*f.x; let x11 = v011 + (v111-v011)*f.x;
    let y0  = x00  + (x10-x00)*f.y;   let y1  = x01  + (x11-x01)*f.y;
    y0 + (y1-y0)*f.z
}
