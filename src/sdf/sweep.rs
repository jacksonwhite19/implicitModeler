// Sweep operation: extrudes a 2D cross-section profile along a 3D path.
//
// Uses the rotation-minimizing (Bishop) frame algorithm for stable orientation,
// with optional linear twist from twist_start to twist_end degrees.

use std::sync::Arc;
use glam::{Vec2, Vec3, Mat3, Quat};
use std::f32::consts::PI;

use crate::sdf::Sdf;
use crate::sdf::aerospace::Section2D;

// ── SweepPath trait ───────────────────────────────────────────────────────────

pub trait SweepPath: Send + Sync {
    /// World-space position at parameter t ∈ [0, 1].
    fn evaluate(&self, t: f32) -> Vec3;
    /// Normalized tangent direction at t.  Computed via finite difference if
    /// the implementor does not have an analytic derivative.
    fn tangent(&self, t: f32) -> Vec3;
    /// Total path length (used for frame-density estimation).
    fn arc_length(&self) -> f32;
}

// ── Shared surface-projection helper ─────────────────────────────────────────

fn gradient(sdf: &dyn Sdf, p: Vec3, eps: f32) -> Vec3 {
    Vec3::new(
        sdf.distance(p + Vec3::X * eps) - sdf.distance(p - Vec3::X * eps),
        sdf.distance(p + Vec3::Y * eps) - sdf.distance(p - Vec3::Y * eps),
        sdf.distance(p + Vec3::Z * eps) - sdf.distance(p - Vec3::Z * eps),
    ) * (0.5 / eps)
}

/// Projects `p` onto the zero-isosurface of `sdf` using 5 Newton steps.
fn closest_surface_point(sdf: &dyn Sdf, p: Vec3) -> Vec3 {
    let eps = 0.001;
    let d = sdf.distance(p);
    let n = gradient(sdf, p, eps).normalize_or_zero();
    let mut q = p - n * d;
    for _ in 0..4 {
        let d2 = sdf.distance(q);
        let n2 = gradient(sdf, q, eps).normalize_or_zero();
        q -= n2 * d2;
    }
    q
}

fn closest_surface_point_with_offset(sdf: &dyn Sdf, p: Vec3, offset: f32) -> Vec3 {
    let cp = closest_surface_point(sdf, p);
    if offset.abs() < 1e-6 {
        return cp;
    }
    let n = gradient(sdf, cp, 0.001).normalize_or_zero();
    cp + n * offset
}

// ── LinePath ──────────────────────────────────────────────────────────────────

/// Straight line from `start` to `end`.
pub struct LinePath {
    pub start: Vec3,
    pub end:   Vec3,
}

impl SweepPath for LinePath {
    fn evaluate(&self, t: f32) -> Vec3 { self.start.lerp(self.end, t) }
    fn tangent(&self, _t: f32) -> Vec3 { (self.end - self.start).normalize_or_zero() }
    fn arc_length(&self) -> f32 { (self.end - self.start).length() }
}

// ── PolylinePath ──────────────────────────────────────────────────────────────

/// Piecewise-linear path through a series of points.
pub struct PolylinePath {
    points:   Vec<Vec3>,
    seg_ends: Vec<f32>,   // cumulative arc length at end of each segment
    total:    f32,
}

impl PolylinePath {
    pub fn new(points: Vec<Vec3>) -> Self {
        let n = points.len();
        let mut seg_ends = Vec::with_capacity(n.saturating_sub(1));
        let mut acc = 0.0f32;
        for i in 1..n {
            acc += (points[i] - points[i - 1]).length();
            seg_ends.push(acc);
        }
        Self { points, seg_ends, total: acc }
    }

    fn seg_and_u(&self, t: f32) -> (usize, f32) {
        if self.points.len() < 2 { return (0, 0.0); }
        let arc = (t * self.total).clamp(0.0, self.total);
        // Find segment whose end >= arc.
        let i = self.seg_ends.partition_point(|&l| l < arc);
        let i = i.min(self.seg_ends.len().saturating_sub(1));
        let start = if i == 0 { 0.0 } else { self.seg_ends[i - 1] };
        let seg_len = self.seg_ends[i] - start;
        let u = if seg_len > 1e-8 { (arc - start) / seg_len } else { 0.0 };
        (i, u.clamp(0.0, 1.0))
    }
}

impl SweepPath for PolylinePath {
    fn evaluate(&self, t: f32) -> Vec3 {
        if self.points.len() < 2 { return Vec3::ZERO; }
        let (i, u) = self.seg_and_u(t);
        self.points[i].lerp(self.points[i + 1], u)
    }
    fn tangent(&self, t: f32) -> Vec3 {
        if self.points.len() < 2 { return Vec3::Z; }
        let (i, _) = self.seg_and_u(t);
        (self.points[i + 1] - self.points[i]).normalize_or_zero()
    }
    fn arc_length(&self) -> f32 { self.total }
}

// ── SplinePath ────────────────────────────────────────────────────────────────

/// Smooth cubic Catmull-Rom spline through the control points.
/// The spline is open (non-looping) — the path starts at the first point and
/// ends at the last.
pub struct SplinePath {
    points: Vec<Vec3>,
    seg_ends: Vec<f32>,
    total: f32,
}

impl SplinePath {
    pub fn new(points: Vec<Vec3>) -> Self {
        // Approximate arc length by sampling each segment at 16 sub-samples.
        let n = points.len();
        if n < 2 {
            return Self { points, seg_ends: vec![], total: 0.0 };
        }
        let segs = n - 1;
        let mut seg_ends = Vec::with_capacity(segs);
        let mut acc = 0.0f32;
        for seg in 0..segs {
            let mut prev = catmull_rom_3d(&points, seg, 0.0);
            for k in 1..=16 {
                let u = k as f32 / 16.0;
                let cur = catmull_rom_3d(&points, seg, u);
                acc += (cur - prev).length();
                prev = cur;
            }
            seg_ends.push(acc);
        }
        Self { points, seg_ends, total: acc }
    }

    fn seg_and_u(&self, t: f32) -> (usize, f32) {
        if self.points.len() < 2 { return (0, 0.0); }
        let arc = (t * self.total).clamp(0.0, self.total);
        let i = self.seg_ends.partition_point(|&l| l < arc);
        let i = i.min(self.seg_ends.len().saturating_sub(1));
        let start = if i == 0 { 0.0 } else { self.seg_ends[i - 1] };
        let seg_len = self.seg_ends[i] - start;
        let u = if seg_len > 1e-8 { (arc - start) / seg_len } else { 0.0 };
        (i, u.clamp(0.0, 1.0))
    }
}

/// Catmull-Rom evaluation at parameter u ∈ [0,1] for segment `seg` in a 3D point array.
/// Clamps ghost points at the ends.
fn catmull_rom_3d(pts: &[Vec3], seg: usize, u: f32) -> Vec3 {
    let n = pts.len();
    let i1 = seg;
    let i2 = (seg + 1).min(n - 1);
    let i0 = if seg == 0 { 0 } else { seg - 1 };
    let i3 = (seg + 2).min(n - 1);
    let p0 = pts[i0];
    let p1 = pts[i1];
    let p2 = pts[i2];
    let p3 = pts[i3];
    let u2 = u * u;
    let u3 = u2 * u;
    0.5 * ((2.0 * p1)
         + (-p0 + p2) * u
         + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * u2
         + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * u3)
}

/// Analytic tangent of the Catmull-Rom spline.
fn catmull_rom_3d_tangent(pts: &[Vec3], seg: usize, u: f32) -> Vec3 {
    let n = pts.len();
    let i1 = seg;
    let i2 = (seg + 1).min(n - 1);
    let i0 = if seg == 0 { 0 } else { seg - 1 };
    let i3 = (seg + 2).min(n - 1);
    let p0 = pts[i0];
    let p1 = pts[i1];
    let p2 = pts[i2];
    let p3 = pts[i3];
    let u2 = u * u;
    0.5 * ((-p0 + p2)
         + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * (2.0 * u)
         + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * (3.0 * u2))
}

impl SweepPath for SplinePath {
    fn evaluate(&self, t: f32) -> Vec3 {
        if self.points.len() < 2 { return Vec3::ZERO; }
        let (seg, u) = self.seg_and_u(t);
        catmull_rom_3d(&self.points, seg, u)
    }
    fn tangent(&self, t: f32) -> Vec3 {
        if self.points.len() < 2 { return Vec3::Z; }
        let (seg, u) = self.seg_and_u(t);
        catmull_rom_3d_tangent(&self.points, seg, u).normalize_or_zero()
    }
    fn arc_length(&self) -> f32 { self.total }
}

// ── SurfaceSpinePath ──────────────────────────────────────────────────────────

/// A path that traces from `start` to `end` while staying on the surface of an SDF.
/// The path is precomputed at construction time as a projected polyline.
pub struct SurfaceSpinePath {
    inner: PolylinePath,
}

impl SurfaceSpinePath {
    pub fn new(surface: Arc<dyn Sdf>, start: Vec3, end: Vec3, steps: usize) -> Self {
        let steps = steps.max(2);
        let mut points = Vec::with_capacity(steps);
        for i in 0..steps {
            let t = i as f32 / (steps - 1) as f32;
            let p = start.lerp(end, t);
            points.push(closest_surface_point(&*surface, p));
        }
        Self { inner: PolylinePath::new(points) }
    }
}

impl SweepPath for SurfaceSpinePath {
    fn evaluate(&self, t: f32) -> Vec3 { self.inner.evaluate(t) }
    fn tangent(&self, t: f32) -> Vec3 { self.inner.tangent(t) }
    fn arc_length(&self) -> f32 { self.inner.arc_length() }
}

/// A smooth guide spline projected onto an SDF surface, then offset along the
/// local outward normal. Useful for dorsal fairings, conformal ducts, and other
/// mold-line-following features that should stand off from the skin.
pub struct ConformalSplinePath {
    inner: PolylinePath,
}

impl ConformalSplinePath {
    pub fn new(surface: Arc<dyn Sdf>, guide_points: Vec<Vec3>, offset: f32, samples: usize) -> Self {
        let guide = SplinePath::new(guide_points);
        let sample_count = samples.max(2);
        let mut points = Vec::with_capacity(sample_count);
        for i in 0..sample_count {
            let t = i as f32 / (sample_count - 1) as f32;
            let p = guide.evaluate(t);
            points.push(closest_surface_point_with_offset(surface.as_ref(), p, offset));
        }
        Self { inner: PolylinePath::new(points) }
    }
}

impl SweepPath for ConformalSplinePath {
    fn evaluate(&self, t: f32) -> Vec3 { self.inner.evaluate(t) }
    fn tangent(&self, t: f32) -> Vec3 { self.inner.tangent(t) }
    fn arc_length(&self) -> f32 { self.inner.arc_length() }
}

// ── Bishop (rotation-minimizing) frame computation ────────────────────────────

/// Hughes-Moller method: given a unit vector `u`, returns a vector perpendicular to it.
fn hughes_moller_perp(u: Vec3) -> Vec3 {
    let abs = u.abs();
    let least = if abs.x <= abs.y && abs.x <= abs.z {
        Vec3::X
    } else if abs.y <= abs.z {
        Vec3::Y
    } else {
        Vec3::Z
    };
    least.cross(u).normalize_or_zero()
}

/// Minimum rotation quaternion that maps `from` onto `to` (both must be unit vectors).
fn min_rotation(from: Vec3, to: Vec3) -> Quat {
    let dot = from.dot(to).clamp(-1.0, 1.0);
    if dot > 0.9999 {
        return Quat::IDENTITY;
    }
    if dot < -0.9999 {
        // 180° rotation around any axis perpendicular to `from`.
        let perp = hughes_moller_perp(from);
        return Quat::from_axis_angle(perp, PI);
    }
    let axis = from.cross(to).normalize_or_zero();
    Quat::from_axis_angle(axis, dot.acos())
}

/// Rotate `(n, b)` around `tangent` by `angle_rad`.
fn apply_twist(n: Vec3, b: Vec3, tangent: Vec3, angle_rad: f32) -> (Vec3, Vec3) {
    if angle_rad.abs() < 1e-6 { return (n, b); }
    let rot = Quat::from_axis_angle(tangent, angle_rad);
    let n2 = (rot * n).normalize_or_zero();
    let b2 = tangent.cross(n2).normalize_or_zero();
    (n2, b2)
}

/// Compute `samples` orthonormal frames along `path` using the rotation-minimizing
/// (Bishop) frame algorithm.  Linear twist from `twist_start` to `twist_end` (degrees)
/// is applied on top.
///
/// Returns `Vec<(position, frame_matrix)>` where each `Mat3` column layout is
/// `[tangent | normal | binormal]`.
pub fn compute_frames(
    path:        &dyn SweepPath,
    samples:     usize,
    twist_start: f32,
    twist_end:   f32,
) -> Vec<(Vec3, Mat3)> {
    let n = samples.max(2);
    let mut frames: Vec<(Vec3, Mat3)> = Vec::with_capacity(n);

    // ── First frame ──────────────────────────────────────────────────────────
    let t0  = path.tangent(0.0).normalize_or_zero();
    let n0  = hughes_moller_perp(t0);
    let b0  = t0.cross(n0).normalize_or_zero();
    let (n0t, b0t) = apply_twist(n0, b0, t0, twist_start.to_radians());
    frames.push((path.evaluate(0.0), Mat3::from_cols(t0, n0t, b0t)));

    // ── Propagate ────────────────────────────────────────────────────────────
    for i in 1..n {
        let s       = i as f32 / (n - 1) as f32;
        let pos     = path.evaluate(s);
        let t_curr  = path.tangent(s).normalize_or_zero();

        let (_, prev_mat) = &frames[i - 1];
        let t_prev = prev_mat.col(0);
        let n_prev = prev_mat.col(1);

        // Rotate previous normal to align with new tangent (Bishop propagation).
        let rot    = min_rotation(t_prev, t_curr);
        let n_curr = (rot * n_prev).normalize_or_zero();
        let b_curr = t_curr.cross(n_curr).normalize_or_zero();

        // Apply accumulated twist (linear interpolation over the whole path).
        let twist_deg = twist_start + (twist_end - twist_start) * s;
        let (n_t, b_t) = apply_twist(n_curr, b_curr, t_curr, twist_deg.to_radians());

        frames.push((pos, Mat3::from_cols(t_curr, n_t, b_t)));
    }

    frames
}

// ── Sweep SDF ─────────────────────────────────────────────────────────────────

/// A 3D body formed by sweeping a 2D cross-section profile along a path.
///
/// Frames are precomputed at construction; distance queries are O(frames) per call.
pub struct Sweep {
    profile:     Arc<dyn Section2D>,
    frames:      Vec<(Vec3, Mat3)>,
    /// Pre-cached axial component of the last and first frame tangents (for endcap test).
    _arc_length: f32,
}

impl Sweep {
    /// Construct a sweep with default 128 frame samples and optional twist (degrees).
    pub fn new(
        profile:     Arc<dyn Section2D>,
        path:        Arc<dyn SweepPath>,
        twist_start: f32,
        twist_end:   f32,
    ) -> Self {
        let arc_length = path.arc_length();
        let samples    = 128.max((arc_length * 4.0) as usize).min(512);
        let frames     = compute_frames(&*path, samples, twist_start, twist_end);
        Self { profile, frames, _arc_length: arc_length }
    }
}

impl Sdf for Sweep {
    fn distance(&self, p: Vec3) -> f32 {
        let n = self.frames.len();
        if n == 0 { return f32::MAX; }

        // Find the nearest frame by 3D distance to frame positions.
        // For most paths this is equivalent to finding the orthogonal projection
        // of p onto the path.
        let mut best_d2 = f32::MAX;
        let mut best_i  = 0usize;
        for (i, (pos, _)) in self.frames.iter().enumerate() {
            let d2 = (*pos - p).length_squared();
            if d2 < best_d2 {
                best_d2 = d2;
                best_i  = i;
            }
        }

        let (frame_pos, frame_mat) = &self.frames[best_i];
        let rel     = p - *frame_pos;
        let axial   = frame_mat.col(0).dot(rel); // along tangent
        let local_2d = Vec2::new(
            frame_mat.col(1).dot(rel),  // normal
            frame_mat.col(2).dot(rel),  // binormal
        );

        let profile_d = self.profile.distance_2d(local_2d);

        // Hemispherical endcaps on open paths.
        if best_i == 0 && axial < 0.0 {
            let overshoot = -axial;
            let pd = profile_d.max(0.0);
            (pd * pd + overshoot * overshoot).sqrt()
        } else if best_i == n - 1 && axial > 0.0 {
            let overshoot = axial;
            let pd = profile_d.max(0.0);
            (pd * pd + overshoot * overshoot).sqrt()
        } else {
            profile_d
        }
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::profiles::SplineProfile;

    /// Circle profile swept along Z-axis should match a cylinder.
    #[test]
    fn circle_sweep_matches_cylinder() {
        // Circle section via SplineProfile::circle
        let profile: Arc<dyn Section2D> = Arc::new(SplineProfile::circle(12, 2.0));
        let path: Arc<dyn SweepPath> = Arc::new(LinePath {
            start: Vec3::ZERO,
            end:   Vec3::new(0.0, 0.0, 10.0),
        });
        let sweep = Sweep::new(profile, path, 0.0, 0.0);

        // Interior point
        let d_inside = sweep.distance(Vec3::new(0.0, 0.0, 5.0));
        assert!(d_inside < 0.0, "center should be inside, got {}", d_inside);

        // Exterior point — outside radius
        let d_outside = sweep.distance(Vec3::new(5.0, 0.0, 5.0));
        assert!(d_outside > 0.0, "outside point should be positive, got {}", d_outside);
    }

    /// SplinePath with 3 collinear points behaves like LinePath.
    #[test]
    fn spline_path_valid_distances() {
        let pts = vec![
            Vec3::ZERO,
            Vec3::new(5.0, 0.0, 0.0),
            Vec3::new(10.0, 0.0, 0.0),
        ];
        let path = SplinePath::new(pts);
        let mid = path.evaluate(0.5);
        assert!((mid.x - 5.0).abs() < 1.0, "midpoint x should be near 5, got {}", mid.x);
    }

    /// Zero twist and full-turn twist both produce valid sweeps.
    #[test]
    fn twist_variants_produce_valid_sdf() {
        let profile: Arc<dyn Section2D> = Arc::new(SplineProfile::circle(8, 1.0));
        let path: Arc<dyn SweepPath> = Arc::new(LinePath {
            start: Vec3::ZERO,
            end:   Vec3::new(0.0, 0.0, 5.0),
        });
        let s0   = Sweep::new(Arc::clone(&profile), Arc::clone(&path) as Arc<dyn SweepPath>, 0.0, 0.0);
        let s360 = Sweep::new(profile, path, 0.0, 360.0);
        // Both should return negative distance at the center axis.
        let q = Vec3::new(0.0, 0.0, 2.5);
        assert!(s0.distance(q) < 0.0);
        assert!(s360.distance(q) < 0.0);
    }

    /// SurfaceSpinePath should stay near the isosurface.
    #[test]
    fn surface_path_stays_near_surface() {
        use crate::sdf::primitives::Sphere;
        let sphere: Arc<dyn Sdf> = Arc::new(Sphere::new(5.0));
        let start  = Vec3::new(0.0, 0.0, 5.0);
        let end    = Vec3::new(5.0, 0.0, 0.0);
        let sp     = SurfaceSpinePath::new(Arc::clone(&sphere), start, end, 32);
        for k in 0..=8 {
            let t = k as f32 / 8.0;
            let p = sp.evaluate(t);
            let d = sphere.distance(p).abs();
            assert!(d < 0.5, "surface path point {} should be near surface, dist={}", k, d);
        }
    }
}
