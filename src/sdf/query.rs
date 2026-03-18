// Geometric query functions for SDF shapes.
// All functions operate on Arc<dyn Sdf> and return Vec3 positions.

use glam::Vec3;
use rayon::prelude::*;
use crate::sdf::Sdf;

// ── Surface point (ray march + bisection) ────────────────────────────────────

/// Ray march from `origin` in `direction` until SDF sign changes, then bisect.
/// Returns the surface crossing point, or None if not found within max_dist.
pub fn surface_point(sdf: &dyn Sdf, origin: Vec3, direction: Vec3, max_dist: f32) -> Option<Vec3> {
    let dir  = direction.normalize();
    let step = 0.5_f32;
    let n    = (max_dist / step) as usize + 1;

    let mut t      = 0.0_f32;
    let mut prev_d = sdf.distance(origin);

    for _ in 0..n.min(512) {
        t += step.min(prev_d.abs().max(0.05));
        if t > max_dist { break; }
        let p = origin + dir * t;
        let d = sdf.distance(p);
        if prev_d.signum() != d.signum() {
            // Bisect between t-step and t
            let mut lo = t - step.min(prev_d.abs().max(0.05));
            let mut hi = t;
            for _ in 0..16 {
                let mid   = (lo + hi) * 0.5;
                let d_mid = sdf.distance(origin + dir * mid);
                if d_mid.signum() == prev_d.signum() { lo = mid; } else { hi = mid; }
            }
            return Some(origin + dir * ((lo + hi) * 0.5));
        }
        prev_d = d;
    }
    None
}

// ── Closest surface point (gradient descent) ─────────────────────────────────

/// Find the closest point on the SDF isosurface to `query` using gradient descent.
pub fn closest_point(sdf: &dyn Sdf, query: Vec3) -> Vec3 {
    let mut p = query;
    for _ in 0..64 {
        let d = sdf.distance(p);
        if d.abs() < 1e-4 { break; }
        let n = gradient(sdf, p);
        p -= n * d;
    }
    p
}

// ── Furthest point in direction ───────────────────────────────────────────────

/// Find the point on the SDF surface furthest in `direction`.
/// Searches up to 2000mm along the direction axis.
pub fn furthest_point(sdf: &dyn Sdf, direction: Vec3) -> Vec3 {
    let dir = direction.normalize();
    // Find rough extent: march along direction until clearly outside for a while
    let mut t           = 0.0_f32;
    let mut last_inside = Vec3::ZERO;
    let mut found       = false;
    let step            = 2.0_f32;
    for _ in 0..1000 {
        let p = dir * t;
        if sdf.distance(p) < 0.0 {
            last_inside = p;
            found = true;
        } else if found && sdf.distance(p) > 20.0 {
            break; // clearly past the surface
        }
        t += step;
    }
    if !found {
        // Try searching backward from origin
        t = 0.0;
        for _ in 0..1000 {
            let p = -dir * t;
            if sdf.distance(p) < 0.0 {
                last_inside = p;
                found = true;
                break;
            }
            t += step;
        }
    }
    if !found { return Vec3::ZERO; }

    // Refine: binary search from last_inside outward in direction
    let mut lo = 0.0_f32;
    let mut hi = 2.0_f32;
    let base   = last_inside;
    // Extend hi until outside
    while sdf.distance(base + dir * hi) < 0.0 && hi < 2000.0 { hi *= 2.0; }
    for _ in 0..32 {
        let mid = (lo + hi) * 0.5;
        if sdf.distance(base + dir * mid) < 0.0 { lo = mid; } else { hi = mid; }
    }
    base + dir * ((lo + hi) * 0.5)
}

// ── Centroid ──────────────────────────────────────────────────────────────────

/// Geometric centroid on a 32³ grid within bounds. Parallel with rayon.
pub fn centroid_point(sdf: &dyn Sdf, bounds_min: Vec3, bounds_max: Vec3) -> Vec3 {
    let n    = 32_usize;
    let step = (bounds_max - bounds_min) / n as f32;

    let (sum, count) = (0..n * n * n)
        .into_par_iter()
        .map(|idx| {
            let ix = idx % n;
            let iy = (idx / n) % n;
            let iz = idx / (n * n);
            let p = bounds_min + Vec3::new(
                (ix as f32 + 0.5) * step.x,
                (iy as f32 + 0.5) * step.y,
                (iz as f32 + 0.5) * step.z,
            );
            if sdf.distance(p) < 0.0 { (p, 1_u32) } else { (Vec3::ZERO, 0_u32) }
        })
        .reduce(|| (Vec3::ZERO, 0), |(sa, ca), (sb, cb)| (sa + sb, ca + cb));

    if count == 0 { (bounds_min + bounds_max) * 0.5 } else { sum / count as f32 }
}

// ── Bounding info ─────────────────────────────────────────────────────────────

pub struct BoundingInfo {
    pub min:     Vec3,
    pub max:     Vec3,
    pub center:  Vec3,
    pub size:    Vec3,
    pub corners: [Vec3; 8],
}

/// Find tight bounding box by searching furthest points in ±X, ±Y, ±Z.
pub fn bounding_points(sdf: &dyn Sdf) -> BoundingInfo {
    let px = furthest_point(sdf, Vec3::X);
    let nx = furthest_point(sdf, Vec3::NEG_X);
    let py = furthest_point(sdf, Vec3::Y);
    let ny = furthest_point(sdf, Vec3::NEG_Y);
    let pz = furthest_point(sdf, Vec3::Z);
    let nz = furthest_point(sdf, Vec3::NEG_Z);

    let min    = Vec3::new(nx.x, ny.y, nz.z);
    let max    = Vec3::new(px.x, py.y, pz.z);
    let center = (min + max) * 0.5;
    let size   = max - min;
    let corners = [
        Vec3::new(min.x, min.y, min.z),
        Vec3::new(max.x, min.y, min.z),
        Vec3::new(min.x, max.y, min.z),
        Vec3::new(max.x, max.y, min.z),
        Vec3::new(min.x, min.y, max.z),
        Vec3::new(max.x, min.y, max.z),
        Vec3::new(min.x, max.y, max.z),
        Vec3::new(max.x, max.y, max.z),
    ];
    BoundingInfo { min, max, center, size, corners }
}

/// Centroid of the cross-section at `axis_pos` along `axis` (0=X, 1=Y, 2=Z).
pub fn cross_section_centroid(sdf: &dyn Sdf, axis: usize, axis_pos: f32) -> Vec3 {
    // Sample a 64x64 2D grid at the slice
    let n    = 64_usize;
    let bbox = bounding_points(sdf);
    let (u_min, u_max, v_min, v_max) = match axis {
        0 => (bbox.min.y, bbox.max.y, bbox.min.z, bbox.max.z),
        1 => (bbox.min.x, bbox.max.x, bbox.min.z, bbox.max.z),
        _ => (bbox.min.x, bbox.max.x, bbox.min.y, bbox.max.y),
    };
    let du = (u_max - u_min) / n as f32;
    let dv = (v_max - v_min) / n as f32;

    let mut sum   = Vec3::ZERO;
    let mut count = 0_u32;
    for iu in 0..n {
        for iv in 0..n {
            let u = u_min + (iu as f32 + 0.5) * du;
            let v = v_min + (iv as f32 + 0.5) * dv;
            let p = match axis {
                0 => Vec3::new(axis_pos, u, v),
                1 => Vec3::new(u, axis_pos, v),
                _ => Vec3::new(u, v, axis_pos),
            };
            if sdf.distance(p) < 0.0 {
                sum   += p;
                count += 1;
            }
        }
    }
    if count == 0 {
        match axis {
            0 => Vec3::new(axis_pos, (u_min + u_max) * 0.5, (v_min + v_max) * 0.5),
            1 => Vec3::new((u_min + u_max) * 0.5, axis_pos, (v_min + v_max) * 0.5),
            _ => Vec3::new((u_min + u_max) * 0.5, (v_min + v_max) * 0.5, axis_pos),
        }
    } else {
        sum / count as f32
    }
}

// ── Gradient helper (central difference) ─────────────────────────────────────

pub fn gradient(sdf: &dyn Sdf, p: Vec3) -> Vec3 {
    let e = 0.001;
    Vec3::new(
        sdf.distance(p + Vec3::new(e, 0.0, 0.0)) - sdf.distance(p - Vec3::new(e, 0.0, 0.0)),
        sdf.distance(p + Vec3::new(0.0, e, 0.0)) - sdf.distance(p - Vec3::new(0.0, e, 0.0)),
        sdf.distance(p + Vec3::new(0.0, 0.0, e)) - sdf.distance(p - Vec3::new(0.0, 0.0, e)),
    ).normalize_or_zero()
}
