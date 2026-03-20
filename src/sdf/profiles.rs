// 2-D spline cross-section profile — runtime type used by the SDF engine.

use glam::{Vec2, Vec3};
use std::any::Any;
use std::sync::Arc;
use super::Sdf;
use super::aerospace::Section2D;

/// Structural role of a control point within a cross-section profile.
///
/// Used by the longitudinal spine system to locate the points that constraint
/// curves must pass through (e.g. the keel must stay on the keel-line).
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize, Default)]
pub enum PointRole {
    #[default]
    Free,
    /// Deck — topmost structural point. Z-locked by deck_line spine.
    Deck,
    /// Keel — bottommost structural point. Z-locked by keel_line spine.
    Keel,
    /// Chine — widest lateral point. Z and/or Y locked by chine_line spine.
    Chine,
}

/// A closed Catmull-Rom spline cross-section.
///
/// Control points define the shape; the curve passes through each one.
/// Sharp-point indices create C0 corners rather than C1 smooth junctions.
///
/// Implements both [`Section2D`] (for lofting) and [`Sdf`] (as an infinite
/// Y-axis extrusion, useful for standalone preview).
#[derive(Clone)]
pub struct SplineProfile {
    pub control_points: Vec<Vec2>,
    pub sharp_points: Vec<usize>,
    /// Structural role of each control point. Same length as `control_points`.
    pub point_roles: Vec<PointRole>,
}

impl SplineProfile {
    pub fn new(control_points: Vec<Vec2>) -> Self {
        let n = control_points.len();
        Self { control_points, sharp_points: vec![], point_roles: vec![PointRole::Free; n] }
    }

    #[allow(dead_code)] // Part of spline profile construction API
    pub fn with_sharp_points(control_points: Vec<Vec2>, sharp_points: Vec<usize>) -> Self {
        let n = control_points.len();
        Self { control_points, sharp_points, point_roles: vec![PointRole::Free; n] }
    }

    pub fn with_roles(
        control_points: Vec<Vec2>,
        sharp_points: Vec<usize>,
        point_roles: Vec<PointRole>,
    ) -> Self {
        let n = control_points.len();
        let mut roles = point_roles;
        roles.resize(n, PointRole::Free);
        Self { control_points, sharp_points, point_roles: roles }
    }

    /// Position of the first control point with the given role, or `None`.
    pub fn role_pos(&self, role: PointRole) -> Option<Vec2> {
        self.point_roles.iter().enumerate()
            .find(|&(_, r)| *r == role)
            .map(|(i, _)| self.control_points[i])
    }

    /// Unit-circle preset: `n` evenly-spaced points at `radius`.
    pub fn circle(n: usize, radius: f32) -> Self {
        let pts = (0..n).map(|i| {
            let a = std::f32::consts::TAU * i as f32 / n as f32;
            Vec2::new(a.cos() * radius, a.sin() * radius)
        }).collect();
        Self::new(pts)
    }

    /// Ellipse preset.
    pub fn ellipse(n: usize, rx: f32, ry: f32) -> Self {
        let pts = (0..n).map(|i| {
            let a = std::f32::consts::TAU * i as f32 / n as f32;
            Vec2::new(a.cos() * rx, a.sin() * ry)
        }).collect();
        Self::new(pts)
    }

    /// Sample the closed Catmull-Rom spline at `n` uniformly-spaced parameter values.
    /// Returns a polyline of `n` points (first ≠ last; caller closes if needed).
    pub fn sample(&self, n: usize) -> Vec<Vec2> {
        let nc = self.control_points.len();
        if nc == 0 { return vec![]; }
        if nc == 1 { return vec![self.control_points[0]; n]; }
        (0..n).map(|i| {
            let t = nc as f32 * i as f32 / n as f32;
            self.eval_at(t)
        }).collect()
    }

    fn eval_at(&self, t: f32) -> Vec2 {
        let nc = self.control_points.len();
        let seg  = (t.floor() as usize) % nc;
        let u    = t - t.floor();
        let i0   = seg;
        let i1   = (seg + 1) % nc;
        let i2   = (seg + 2) % nc;
        let i_pr = if seg == 0 { nc - 1 } else { seg - 1 };

        let p0 = self.control_points[i_pr];
        let p1 = self.control_points[i0];
        let p2 = self.control_points[i1];
        let p3 = self.control_points[i2];

        // Sharp-point: zero the tangent approaching / leaving the corner.
        let p0 = if self.sharp_points.contains(&i0) { p1 } else { p0 };
        let p3 = if self.sharp_points.contains(&i1) { p2 } else { p3 };

        catmull_rom(p0, p1, p2, p3, u)
    }
}

fn catmull_rom(p0: Vec2, p1: Vec2, p2: Vec2, p3: Vec2, t: f32) -> Vec2 {
    let t2 = t * t;
    let t3 = t2 * t;
    0.5 * ((2.0 * p1)
         + (-p0 + p2) * t
         + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
         + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3)
}

// ── 2-D geometry helpers ────────────────────────────────────────────────────

fn winding_number(poly: &[Vec2], p: Vec2) -> i32 {
    let n = poly.len();
    let mut wn = 0i32;
    for i in 0..n {
        let a = poly[i];
        let b = poly[(i + 1) % n];
        if a.y <= p.y {
            if b.y > p.y && is_left(a, b, p) > 0.0 { wn += 1; }
        } else if b.y <= p.y && is_left(a, b, p) < 0.0 {
            wn -= 1;
        }
    }
    wn
}

#[inline]
fn is_left(a: Vec2, b: Vec2, p: Vec2) -> f32 {
    (b.x - a.x) * (p.y - a.y) - (p.x - a.x) * (b.y - a.y)
}

fn min_dist_sq_to_segment(a: Vec2, b: Vec2, p: Vec2) -> f32 {
    let ab = b - a;
    let ap = p - a;
    let len2 = ab.dot(ab);
    let t = if len2 < 1e-12 { 0.0 } else { (ap.dot(ab) / len2).clamp(0.0, 1.0) };
    (p - (a + ab * t)).length_squared()
}

// ── Section2D impl ───────────────────────────────────────────────────────────

impl Section2D for SplineProfile {
    fn distance_2d(&self, point: Vec2) -> f32 {
        let poly = self.sample(200);
        if poly.len() < 2 { return f32::MAX; }
        let n = poly.len();
        let mut min_d2 = f32::MAX;
        for i in 0..n {
            let d2 = min_dist_sq_to_segment(poly[i], poly[(i + 1) % n], point);
            if d2 < min_d2 { min_d2 = d2; }
        }
        let sign = if winding_number(&poly, point) != 0 { -1.0 } else { 1.0 };
        sign * min_d2.sqrt()
    }

    fn lerp_to(&self, other: &dyn Section2D, t: f32) -> Arc<dyn Section2D> {
        // Resample both to a common count and lerp corresponding vertices.
        let n = 64
            .max(self.control_points.len())
            .max(other.as_any()
                .downcast_ref::<SplineProfile>()
                .map(|o| o.control_points.len())
                .unwrap_or(0));
        // Snap up to next power of two, capped at 256.
        let n = n.next_power_of_two().min(256);

        let self_pts = self.sample(n);
        let other_pts = if let Some(o) = other.as_any().downcast_ref::<SplineProfile>() {
            o.sample(n)
        } else {
            // Can't geometry-lerp; fall back to distance-lerp via default impl.
            return Arc::new(self.clone());
        };

        let lerped: Vec<Vec2> = self_pts.iter().zip(other_pts.iter())
            .map(|(&a, &b)| a.lerp(b, t))
            .collect();

        Arc::new(SplineProfile { control_points: lerped, sharp_points: vec![], point_roles: vec![PointRole::Free; n] })
    }

    fn as_any(&self) -> &dyn Any { self }
}

// ── 3-D SDF impl (infinite Y-extrusion of the XZ-plane profile) ─────────────

impl Sdf for SplineProfile {
    /// Evaluates the profile in the XZ plane, ignoring Y.
    /// Produces an infinite prism running along the Y axis —
    /// useful for standalone preview before lofting.
    fn distance(&self, point: Vec3) -> f32 {
        self.distance_2d(Vec2::new(point.x, point.z))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_circle_profile_inside() {
        let p = SplineProfile::circle(8, 1.0);
        let d = p.distance_2d(Vec2::ZERO);
        assert!(d < 0.0, "origin should be inside unit circle, got {}", d);
    }

    #[test]
    fn test_circle_profile_outside() {
        let p = SplineProfile::circle(8, 1.0);
        let d = p.distance_2d(Vec2::new(2.0, 0.0));
        assert!(d > 0.0, "point outside circle should be positive, got {}", d);
    }

    #[test]
    fn test_lerp_profiles() {
        let a = SplineProfile::circle(8, 1.0);
        let b = SplineProfile::circle(8, 2.0);
        let mid = a.lerp_to(&b, 0.5);
        // Midpoint should be roughly radius 1.5
        let d = mid.distance_2d(Vec2::new(1.5, 0.0));
        assert!(d.abs() < 0.2, "lerp midpoint radius ~1.5, dist={}", d);
    }

    #[test]
    fn test_sdf_3d_extrusion() {
        let p = SplineProfile::circle(8, 1.0);
        // Y=5 is irrelevant — only XZ matters
        assert!(p.distance(Vec3::new(0.0, 5.0, 0.0)) < 0.0, "origin in XZ should be inside");
        assert!(p.distance(Vec3::new(3.0, 0.0, 0.0)) > 0.0, "far XZ point should be outside");
    }
}

// ── RectProfile ───────────────────────────────────────────────────────────────

/// Rectangular 2D cross-section (standard box SDF).
/// Useful for square cable channels and spar caps in sweep operations.
pub struct RectProfile {
    pub half_w: f32,
    pub half_h: f32,
}

impl RectProfile {
    pub fn new(width: f32, height: f32) -> Self {
        Self { half_w: width / 2.0, half_h: height / 2.0 }
    }
}

impl Section2D for RectProfile {
    fn distance_2d(&self, p: Vec2) -> f32 {
        let q = p.abs() - Vec2::new(self.half_w, self.half_h);
        q.max(Vec2::ZERO).length() + q.x.max(q.y).min(0.0)
    }
    fn lerp_to(&self, other: &dyn Section2D, t: f32) -> Arc<dyn Section2D> {
        if let Some(o) = other.as_any().downcast_ref::<RectProfile>() {
            Arc::new(RectProfile {
                half_w: self.half_w + (o.half_w - self.half_w) * t,
                half_h: self.half_h + (o.half_h - self.half_h) * t,
            })
        } else {
            // Cross-type: keep self (no meaningful morphing).
            Arc::new(RectProfile { half_w: self.half_w, half_h: self.half_h })
        }
    }
    fn as_any(&self) -> &dyn Any { self }
}

/// Rounded rectangle 2D cross-section.
pub struct RoundedRectProfile {
    pub half_w: f32,
    pub half_h: f32,
    pub radius: f32,
}

impl RoundedRectProfile {
    pub fn new(width: f32, height: f32, radius: f32) -> Self {
        let half_w = width / 2.0;
        let half_h = height / 2.0;
        let max_r = half_w.min(half_h).max(0.0);
        Self {
            half_w,
            half_h,
            radius: radius.clamp(0.0, max_r),
        }
    }
}

impl Section2D for RoundedRectProfile {
    fn distance_2d(&self, p: Vec2) -> f32 {
        let q = p.abs() - Vec2::new(self.half_w - self.radius, self.half_h - self.radius);
        q.max(Vec2::ZERO).length() + q.x.max(q.y).min(0.0) - self.radius
    }
    fn lerp_to(&self, other: &dyn Section2D, t: f32) -> Arc<dyn Section2D> {
        if let Some(o) = other.as_any().downcast_ref::<RoundedRectProfile>() {
            Arc::new(RoundedRectProfile {
                half_w: self.half_w + (o.half_w - self.half_w) * t,
                half_h: self.half_h + (o.half_h - self.half_h) * t,
                radius: self.radius + (o.radius - self.radius) * t,
            })
        } else {
            Arc::new(RoundedRectProfile {
                half_w: self.half_w,
                half_h: self.half_h,
                radius: self.radius,
            })
        }
    }
    fn as_any(&self) -> &dyn Any { self }
}

// ── NGonProfile ───────────────────────────────────────────────────────────────

/// Regular n-sided polygon 2D cross-section.
/// Useful for hex standoffs and structural tubes in sweep operations.
pub struct NGonProfile {
    pub sides:  u32,
    pub radius: f32,
}

impl NGonProfile {
    pub fn new(sides: u32, radius: f32) -> Self {
        Self { sides: sides.max(3), radius }
    }
}

impl Section2D for NGonProfile {
    /// IQ-style regular polygon SDF.
    fn distance_2d(&self, p: Vec2) -> f32 {
        let n   = self.sides as f32;
        let an  = std::f32::consts::TAU / n;
        let acs = Vec2::new((an / 2.0).cos(), (an / 2.0).sin());
        // Reduce p to the first sector.
        let bn  = p.x.atan2(p.y).rem_euclid(an) - an / 2.0;
        let p2  = Vec2::new(bn.cos(), bn.abs().sin()) * p.length();
        let p2  = p2 - self.radius * acs;
        let p2  = Vec2::new(p2.x, p2.y + (-p2.y).clamp(0.0, self.radius * acs.y));
        p2.length() * p2.x.signum()
    }
    fn lerp_to(&self, other: &dyn Section2D, t: f32) -> Arc<dyn Section2D> {
        if let Some(o) = other.as_any().downcast_ref::<NGonProfile>() {
            // Keep sides of self (morphing side count is undefined).
            Arc::new(NGonProfile {
                sides:  self.sides,
                radius: self.radius + (o.radius - self.radius) * t,
            })
        } else {
            Arc::new(NGonProfile { sides: self.sides, radius: self.radius })
        }
    }
    fn as_any(&self) -> &dyn Any { self }
}
