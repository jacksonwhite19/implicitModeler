// Haack series, Von Karman, Tangent Ogive, and Ellipsoid nose/tail cone primitives.

use glam::Vec3;
use crate::sdf::Sdf;

// ── Helper: 2D point-to-segment distance ─────────────────────────────────────

pub fn point_to_segment_2d(px: f32, py: f32, ax: f32, ay: f32, bx: f32, by: f32) -> f32 {
    let dx = bx - ax;
    let dy = by - ay;
    let len_sq = dx * dx + dy * dy;
    if len_sq < 1e-12 {
        let ex = px - ax;
        let ey = py - ay;
        return (ex * ex + ey * ey).sqrt();
    }
    let t = ((px - ax) * dx + (py - ay) * dy) / len_sq;
    let t = t.clamp(0.0, 1.0);
    let cx = ax + t * dx;
    let cy = ay + t * dy;
    let ex = px - cx;
    let ey = py - cy;
    (ex * ex + ey * ey).sqrt()
}

// ── HaackNose ─────────────────────────────────────────────────────────────────

/// Haack series nose cone (Von Karman when c=0, LV-Haack when c=1/3).
#[allow(dead_code)]
pub struct HaackNose {
    pub length: f32,
    pub base_radius: f32,
    pub c_parameter: f32,
    profile: Vec<(f32, f32)>, // (x, r) at 256 points
}

impl HaackNose {
    pub fn new(length: f32, base_radius: f32, c_parameter: f32) -> Self {
        let n = 256usize;
        let mut profile = Vec::with_capacity(n);
        for i in 0..n {
            let x_frac = i as f32 / (n - 1) as f32;
            let x = x_frac * length;
            let theta = (1.0 - 2.0 * x_frac).acos();
            let val = theta - theta.sin() * theta.cos() + c_parameter * theta.sin().powi(3);
            let r = (base_radius / std::f32::consts::PI.sqrt()) * val.max(0.0).sqrt();
            profile.push((x, r));
        }
        Self { length, base_radius, c_parameter, profile }
    }

    pub fn radius_at(&self, x: f32) -> f32 {
        if self.profile.is_empty() { return 0.0; }
        let x_clamped = x.clamp(0.0, self.length);
        // Binary search for the segment
        let x_frac = x_clamped / self.length;
        let idx_f = x_frac * (self.profile.len() - 1) as f32;
        let i0 = (idx_f as usize).min(self.profile.len() - 2);
        let i1 = i0 + 1;
        let (x0, r0) = self.profile[i0];
        let (x1, r1) = self.profile[i1];
        if (x1 - x0).abs() < 1e-9 { return r0; }
        let t = ((x_clamped - x0) / (x1 - x0)).clamp(0.0, 1.0);
        r0 + t * (r1 - r0)
    }
}

impl Sdf for HaackNose {
    fn distance(&self, p: Vec3) -> f32 {
        let r_query = (p.y * p.y + p.z * p.z).sqrt();

        let mut min_dist = f32::MAX;
        for i in 0..self.profile.len() - 1 {
            let (x0, r0) = self.profile[i];
            let (x1, r1) = self.profile[i + 1];
            let dist = point_to_segment_2d(p.x, r_query, x0, r0, x1, r1);
            if dist < min_dist {
                min_dist = dist;
            }
        }

        // Also check endcaps (base disk at x=length)
        // Distance to tip point (0, 0) in 2D
        let tip_dist = ((p.x) * (p.x) + r_query * r_query).sqrt();
        if tip_dist < min_dist { min_dist = tip_dist; }

        // Sign: inside if x in [0, length] and r_query < radius_at(x)
        if p.x >= 0.0 && p.x <= self.length && r_query < self.radius_at(p.x) {
            -min_dist
        } else {
            min_dist
        }
    }
}

// ── HaackTail ─────────────────────────────────────────────────────────────────

/// Haack series tail cone — base_radius at x=0, tip_radius at x=length.
#[allow(dead_code)]
pub struct HaackTail {
    pub length: f32,
    pub base_radius: f32,
    pub tip_radius: f32,
    pub c_parameter: f32,
    profile: Vec<(f32, f32)>,
}

impl HaackTail {
    pub fn new(length: f32, base_radius: f32, tip_radius: f32, c_parameter: f32) -> Self {
        let n = 256usize;
        let mut profile = Vec::with_capacity(n);
        for i in 0..n {
            // x=0 → base, x=length → tip; reverse Haack profile
            let x_frac = i as f32 / (n - 1) as f32;
            let x = x_frac * length;
            // Use reversed x_frac (1 - x_frac) for the Haack formula
            let inv_frac = 1.0 - x_frac;
            let theta = (1.0 - 2.0 * inv_frac).acos();
            let val = theta - theta.sin() * theta.cos() + c_parameter * theta.sin().powi(3);
            let haack_r = (base_radius / std::f32::consts::PI.sqrt()) * val.max(0.0).sqrt();
            // Ensure minimum radius floor is tip_radius
            let r = tip_radius.max(haack_r * (1.0 - x_frac) + tip_radius * x_frac);
            profile.push((x, r));
        }
        Self { length, base_radius, tip_radius, c_parameter, profile }
    }

    pub fn radius_at(&self, x: f32) -> f32 {
        if self.profile.is_empty() { return 0.0; }
        let x_clamped = x.clamp(0.0, self.length);
        let x_frac = x_clamped / self.length;
        let idx_f = x_frac * (self.profile.len() - 1) as f32;
        let i0 = (idx_f as usize).min(self.profile.len() - 2);
        let i1 = i0 + 1;
        let (x0, r0) = self.profile[i0];
        let (x1, r1) = self.profile[i1];
        if (x1 - x0).abs() < 1e-9 { return r0; }
        let t = ((x_clamped - x0) / (x1 - x0)).clamp(0.0, 1.0);
        r0 + t * (r1 - r0)
    }
}

impl Sdf for HaackTail {
    fn distance(&self, p: Vec3) -> f32 {
        let r_query = (p.y * p.y + p.z * p.z).sqrt();

        let mut min_dist = f32::MAX;
        for i in 0..self.profile.len() - 1 {
            let (x0, r0) = self.profile[i];
            let (x1, r1) = self.profile[i + 1];
            let dist = point_to_segment_2d(p.x, r_query, x0, r0, x1, r1);
            if dist < min_dist {
                min_dist = dist;
            }
        }

        if p.x >= 0.0 && p.x <= self.length && r_query < self.radius_at(p.x) {
            -min_dist
        } else {
            min_dist
        }
    }
}

// ── TangentOgive ─────────────────────────────────────────────────────────────

/// Tangent ogive nose cone.
#[allow(dead_code)]
pub struct TangentOgive {
    pub length: f32,
    pub base_radius: f32,
    profile: Vec<(f32, f32)>,
}

impl TangentOgive {
    pub fn new(length: f32, base_radius: f32) -> Self {
        let rho = (base_radius * base_radius + length * length) / (2.0 * base_radius);
        let n = 256usize;
        let mut profile = Vec::with_capacity(n);
        for i in 0..n {
            let x_frac = i as f32 / (n - 1) as f32;
            let x = x_frac * length;
            let r = (rho * rho - (length - x) * (length - x)).max(0.0).sqrt() + base_radius - rho;
            profile.push((x, r.max(0.0)));
        }
        Self { length, base_radius, profile }
    }

    pub fn radius_at(&self, x: f32) -> f32 {
        if self.profile.is_empty() { return 0.0; }
        let x_clamped = x.clamp(0.0, self.length);
        let x_frac = x_clamped / self.length;
        let idx_f = x_frac * (self.profile.len() - 1) as f32;
        let i0 = (idx_f as usize).min(self.profile.len() - 2);
        let i1 = i0 + 1;
        let (x0, r0) = self.profile[i0];
        let (x1, r1) = self.profile[i1];
        if (x1 - x0).abs() < 1e-9 { return r0; }
        let t = ((x_clamped - x0) / (x1 - x0)).clamp(0.0, 1.0);
        r0 + t * (r1 - r0)
    }
}

impl Sdf for TangentOgive {
    fn distance(&self, p: Vec3) -> f32 {
        let r_query = (p.y * p.y + p.z * p.z).sqrt();

        let mut min_dist = f32::MAX;
        for i in 0..self.profile.len() - 1 {
            let (x0, r0) = self.profile[i];
            let (x1, r1) = self.profile[i + 1];
            let dist = point_to_segment_2d(p.x, r_query, x0, r0, x1, r1);
            if dist < min_dist {
                min_dist = dist;
            }
        }

        if p.x >= 0.0 && p.x <= self.length && r_query < self.radius_at(p.x) {
            -min_dist
        } else {
            min_dist
        }
    }
}

// ── EllipsoidNose ─────────────────────────────────────────────────────────────

/// Prolate ellipsoid nose cone (half-ellipsoid, tip at x=0, base at x=length).
pub struct EllipsoidNose {
    pub length: f32,
    pub base_radius: f32,
}

impl EllipsoidNose {
    pub fn new(length: f32, base_radius: f32) -> Self {
        Self { length, base_radius }
    }
}

impl Sdf for EllipsoidNose {
    fn distance(&self, p: Vec3) -> f32 {
        // Half-ellipsoid: only x >= 0 half
        // Approximate ellipsoid SDF using scaled space
        let r = (p.y * p.y + p.z * p.z).sqrt();
        // Map to unit sphere space
        let nx = p.x / self.length;
        let nr = r / self.base_radius;
        // Gradient-corrected ellipsoid SDF approximation
        let d = (nx * nx + nr * nr).sqrt() - 1.0;
        let scale = self.length.min(self.base_radius);

        // Handle the half-nose (only x ∈ [0, length])
        if p.x < 0.0 {
            // Beyond tip: distance to tip point
            let tip_dist = (p.x * p.x + r * r).sqrt();
            return tip_dist;
        }
        if p.x > self.length {
            // Beyond base: distance to base circle
            let dr = (r - self.base_radius).max(0.0);
            let dx = p.x - self.length;
            return (dr * dr + dx * dx).sqrt();
        }

        d * scale
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_von_karman_profile_matches_formula() {
        let nose = HaackNose::new(100.0, 15.0, 0.0);
        for i in 0..10 {
            let x_frac = i as f32 / 9.0;
            let x = x_frac * 100.0;
            let theta = (1.0 - 2.0 * x_frac).acos();
            let expected_r = (15.0 / std::f32::consts::PI.sqrt())
                * (theta - theta.sin() * theta.cos()).sqrt();
            let computed_r = nose.radius_at(x);
            assert!(
                (computed_r - expected_r).abs() < 0.5,
                "r at x={}: expected {:.3}, got {:.3}",
                x,
                expected_r,
                computed_r
            );
        }
    }

    #[test]
    fn test_haack_nose_tip_is_zero() {
        let nose = HaackNose::new(100.0, 15.0, 0.0);
        assert!(nose.radius_at(0.0) < 0.1, "tip radius should be near zero");
    }

    #[test]
    fn test_haack_nose_base_matches() {
        let nose = HaackNose::new(100.0, 15.0, 0.0);
        let r_base = nose.radius_at(100.0);
        assert!((r_base - 15.0).abs() < 0.5, "base radius should match: got {}", r_base);
    }

    #[test]
    fn test_haack_nose_inside_negative() {
        let nose = HaackNose::new(100.0, 15.0, 0.0);
        let d = nose.distance(Vec3::new(50.0, 0.0, 0.0));
        assert!(d < 0.0, "center axis inside nose should be negative, got {}", d);
    }

    #[test]
    fn test_haack_nose_outside_positive() {
        let nose = HaackNose::new(100.0, 15.0, 0.0);
        let d = nose.distance(Vec3::new(50.0, 50.0, 0.0));
        assert!(d > 0.0, "point far outside should be positive, got {}", d);
    }

    #[test]
    fn test_tangent_ogive_inside() {
        let ogive = TangentOgive::new(100.0, 15.0);
        let d = ogive.distance(Vec3::new(50.0, 0.0, 0.0));
        assert!(d < 0.0, "axis inside ogive should be negative, got {}", d);
    }

    #[test]
    fn test_ellipsoid_nose_inside() {
        let en = EllipsoidNose::new(100.0, 15.0);
        let d = en.distance(Vec3::new(50.0, 0.0, 0.0));
        assert!(d < 0.0, "axis inside ellipsoid nose should be negative, got {}", d);
    }
}
