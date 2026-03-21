// Aerodynamic inlet geometry: NACA flush inlets, EDF buried inlets, inlet lips,
// EDF duct system, and exhaust nozzles.

use glam::{Vec2, Vec3};
use std::sync::Arc;
use crate::sdf::Sdf;
use crate::sdf::booleans::{Intersect, SmoothUnion, Subtract};
use crate::sdf::profiles::{NGonProfile, RectProfile, RoundedRectProfile, SplineProfile};
use crate::sdf::sweep::{PolylinePath, SplinePath, SweepPath, project_point_to_surface_with_offset_and_normal};
use crate::sdf::transforms::Offset;
use crate::sdf::aerospace::Section2D;
use std::any::Any;

// ── Smooth helpers ────────────────────────────────────────────────────────────

/// Smooth maximum — positive when either a or b is large.
pub fn smooth_max(a: f32, b: f32, k: f32) -> f32 {
    0.5 * ((a + b) + ((a - b) * (a - b) + k * k).sqrt())
}

/// Smooth minimum — negative when both a and b are small.
#[allow(dead_code)] // Used internally within inlet geometry builders
pub fn smooth_min(a: f32, b: f32, k: f32) -> f32 {
    -smooth_max(-a, -b, k)
}

// ── Local-frame transform ─────────────────────────────────────────────────────

/// Build an orthonormal frame given a forward direction.
/// Returns (forward, right, up).
fn make_frame(forward: Vec3) -> (Vec3, Vec3, Vec3) {
    let fwd = forward.normalize();
    let world_up = if fwd.dot(Vec3::Z).abs() < 0.9 { Vec3::Z } else { Vec3::Y };
    let right = fwd.cross(world_up).normalize();
    let up = right.cross(fwd).normalize();
    (fwd, right, up)
}

/// Transform a world point into a local frame centered at `origin` with `forward` as +X.
fn to_local(p: Vec3, origin: Vec3, forward: Vec3) -> Vec3 {
    let (fwd, right, up) = make_frame(forward);
    let rel = p - origin;
    Vec3::new(rel.dot(fwd), rel.dot(right), rel.dot(up))
}

// ── NacaInlet ─────────────────────────────────────────────────────────────────

/// NACA submerged flush inlet (scoop-shaped void in the OML).
pub struct NacaInlet {
    pub width: f32,
    pub length: f32,
    pub depth: f32,
    pub ramp_angle_deg: f32,
    pub position: Vec3,
    pub normal: Vec3,
    pub flow_direction: Vec3,
}

impl Sdf for NacaInlet {
    fn distance(&self, p: Vec3) -> f32 {
        // Build local frame: x_axis = -flow_direction (streamwise into inlet),
        // z_axis aligned with normal, y_axis lateral.
        let x_axis = (-self.flow_direction).normalize();
        let z_axis = self.normal.normalize();
        let y_axis = z_axis.cross(x_axis).normalize();

        let rel = p - self.position;
        let lx = rel.dot(x_axis);
        let ly = rel.dot(y_axis);
        let lz = rel.dot(z_axis);

        let ramp_angle = self.ramp_angle_deg.to_radians();

        // x range [0, length]
        let d_x = smooth_max(-lx, lx - self.length, 2.0);

        // Lateral wedge: |y| < (width/2) * (lx/length)
        let x_frac = (lx / self.length).clamp(0.0, 1.0);
        let y_limit = (self.width / 2.0) * x_frac;
        let d_y = ly.abs() - y_limit;

        // Ramp: z < -ramp_angle * lx (negative inside)
        let d_z_ramp = lz + ramp_angle * lx.max(0.0); // negative inside (z going down into surface)

        // Floor: z > -depth
        let d_z_floor = -lz - self.depth;

        // Interior = inside all four constraints
        let d_interior = smooth_max(
            smooth_max(d_x, d_y, 2.0),
            smooth_max(d_z_ramp, d_z_floor, 2.0),
            2.0,
        );
        d_interior
    }
}

// ── InletShape ────────────────────────────────────────────────────────────────

#[derive(Clone)]
pub enum InletShape {
    Circular { diameter: f32 },
    Elliptical { width: f32, height: f32 },
    DShaped { width: f32, height: f32, flat_fraction: f32 },
}

// ── InletLip ─────────────────────────────────────────────────────────────────

/// EDF inlet lip geometry (circular, elliptical, or D-shaped).
pub struct InletLip {
    pub shape: InletShape,
    pub lip_radius: f32,
    pub position: Vec3,
    pub direction: Vec3,
    pub highlight_to_throat: f32,
    pub throat_area_fraction: f32,
}

impl Sdf for InletLip {
    fn distance(&self, p: Vec3) -> f32 {
        let local = to_local(p, self.position, self.direction);
        let lx = local.x; // axial (into inlet)
        let ly = local.y;
        let lz = local.z;

        match &self.shape {
            InletShape::Circular { diameter } => {
                let r_inlet = diameter / 2.0;
                let r_maj = (r_inlet - self.lip_radius).max(0.0);
                let r2 = (ly * ly + lz * lz).sqrt();
                // Torus at x=0 plane
                let torus_d = ((r2 - r_maj).powi(2) + lx.powi(2)).sqrt() - self.lip_radius;
                // Throat cylinder
                let throat_r = r_inlet * self.throat_area_fraction.sqrt();
                let cyl_r = r2 - throat_r;
                let cyl_x_lo = -lx;
                let cyl_x_hi = lx - self.highlight_to_throat;
                let cyl_d = cyl_r.max(cyl_x_lo).max(cyl_x_hi);
                torus_d.min(cyl_d)
            }
            InletShape::Elliptical { width, height } => {
                let r_avg = (width + height) / 4.0;
                let r_maj = (r_avg - self.lip_radius).max(0.0);
                // Scaled radial distance
                let wy = width / 2.0;
                let wz = height / 2.0;
                let r2_scaled = ((ly / wy).powi(2) + (lz / wz).powi(2)).sqrt() * r_avg;
                let torus_d = ((r2_scaled - r_maj).powi(2) + lx.powi(2)).sqrt() - self.lip_radius;
                torus_d
            }
            InletShape::DShaped { width, height, flat_fraction } => {
                let r = width / 2.0;
                let r_maj = (r - self.lip_radius).max(0.0);
                let r2 = (ly * ly + lz * lz).sqrt();
                let torus_d = ((r2 - r_maj).powi(2) + lx.powi(2)).sqrt() - self.lip_radius;
                // Flat bottom cutoff
                let flat_y = -height * (1.0 - flat_fraction) + height / 2.0;
                torus_d.max(flat_y - lz)
            }
        }
    }
}

// ── EdfDuct ───────────────────────────────────────────────────────────────────

/// EDF duct — tapered capsule from inlet to fan to exhaust.
#[allow(dead_code)]
pub struct EdfDuct {
    pub inlet_position: Vec3,
    pub inlet_radius: f32,
    pub fan_position: Vec3,
    pub fan_radius: f32,
    pub exhaust_position: Vec3,
    pub exhaust_radius: f32,
    pub exhaust_direction: Vec3,
    pub control_points: Vec<Vec3>,
}

/// Tapered capsule SDF: cylinder with different radii at each end.
pub fn capsule_sdf_tapered(p: Vec3, a: Vec3, b: Vec3, ra: f32, rb: f32) -> f32 {
    let ab = b - a;
    let ap = p - a;
    let len = ab.length();
    if len < 1e-6 {
        return (p - a).length() - ra;
    }
    let t = (ap.dot(ab) / (len * len)).clamp(0.0, 1.0);
    let closest = a + ab * t;
    let r = ra + (rb - ra) * t;
    (p - closest).length() - r
}

impl Sdf for EdfDuct {
    fn distance(&self, p: Vec3) -> f32 {
        // Section 1: inlet → fan (diffuser)
        let d1 = capsule_sdf_tapered(
            p,
            self.inlet_position,
            self.fan_position,
            self.inlet_radius,
            self.fan_radius,
        );
        // Section 2: fan → exhaust (nozzle)
        let d2 = capsule_sdf_tapered(
            p,
            self.fan_position,
            self.exhaust_position,
            self.fan_radius,
            self.exhaust_radius,
        );
        d1.min(d2)
    }
}

// ── SDuct ─────────────────────────────────────────────────────────────────────

/// S-shaped transition duct from inlet face to fan face.
///
/// The centerline follows a sinusoidal path that offsets from `inlet_center`
/// to `inlet_center + (offset_x, offset_y, 0)` over `length` in the X direction.
/// The tube cross-section tapers linearly from `inlet_r` to `exit_r`.
pub struct SDuct {
    /// Inlet face centre (world space).
    pub inlet_center: Vec3,
    /// Duct flow direction (normalized).
    pub flow_dir: Vec3,
    /// Inlet throat radius (mm).
    pub inlet_r: f32,
    /// Exit face radius (mm).
    pub exit_r: f32,
    /// Total duct length along flow axis (mm).
    pub length: f32,
    /// Lateral (Y) offset of exit centre relative to inlet centre (mm).
    pub offset_y: f32,
    /// Vertical (Z) offset of exit centre relative to inlet centre (mm).
    pub offset_z: f32,
    /// Number of capsule segments used to approximate the curve.
    pub segments: usize,
}

impl SDuct {
    pub fn new(inlet_r: f32, exit_r: f32, length: f32, offset_y: f32, offset_z: f32) -> Self {
        Self {
            inlet_center: Vec3::ZERO,
            flow_dir: Vec3::X,
            inlet_r,
            exit_r,
            length,
            offset_y,
            offset_z,
            segments: 12,
        }
    }
}

impl Sdf for SDuct {
    fn distance(&self, p: Vec3) -> f32 {
        let n = self.segments.max(2);
        let mut dist = f32::MAX;

        for i in 0..n {
            let t0 = i as f32 / n as f32;
            let t1 = (i + 1) as f32 / n as f32;

            // Sinusoidal centerline: smoothly curves from inlet to exit
            let cx0 = self.inlet_center + self.flow_dir * (self.length * t0)
                + Vec3::new(0.0, self.offset_y, self.offset_z) * smoothstep(t0);
            let cx1 = self.inlet_center + self.flow_dir * (self.length * t1)
                + Vec3::new(0.0, self.offset_y, self.offset_z) * smoothstep(t1);

            let r0 = self.inlet_r + (self.exit_r - self.inlet_r) * t0;
            let r1 = self.inlet_r + (self.exit_r - self.inlet_r) * t1;

            dist = dist.min(capsule_sdf_tapered(p, cx0, cx1, r0, r1));
        }
        dist
    }
}

fn smoothstep(t: f32) -> f32 {
    t * t * (3.0 - 2.0 * t)
}

fn smooth_profile_lerp(a: f32, b: f32, t: f32, smoothness: f32) -> f32 {
    let eased = smoothstep(t.clamp(0.0, 1.0));
    let blend = t + (eased - t) * smoothness.clamp(0.0, 1.0);
    a + (b - a) * blend
}

fn radius_at(start_radius: f32, end_radius: f32, t: f32, smoothness: f32) -> f32 {
    smooth_profile_lerp(start_radius, end_radius, t, smoothness).max(1e-4)
}

fn point_distance_sq(path: &dyn SweepPath, p: Vec3, t: f32) -> f32 {
    (path.evaluate(t.clamp(0.0, 1.0)) - p).length_squared()
}

fn refined_closest_t(path: &dyn SweepPath, coarse_ts: &[f32], p: Vec3) -> f32 {
    if coarse_ts.is_empty() {
        return 0.0;
    }
    if coarse_ts.len() == 1 {
        return coarse_ts[0];
    }

    let mut best_i = 0usize;
    let mut best_d2 = f32::INFINITY;
    for (i, &t) in coarse_ts.iter().enumerate() {
        let d2 = point_distance_sq(path, p, t);
        if d2 < best_d2 {
            best_d2 = d2;
            best_i = i;
        }
    }

    let lo_i = best_i.saturating_sub(1);
    let hi_i = (best_i + 1).min(coarse_ts.len() - 1);
    let mut a = coarse_ts[lo_i];
    let mut b = coarse_ts[hi_i];
    if (b - a).abs() < 1e-6 {
        return coarse_ts[best_i];
    }

    let phi = 0.618_033_95_f32;
    let mut c = b - (b - a) * phi;
    let mut d = a + (b - a) * phi;
    let mut fc = point_distance_sq(path, p, c);
    let mut fd = point_distance_sq(path, p, d);

    for _ in 0..28 {
        if fc <= fd {
            b = d;
            d = c;
            fd = fc;
            c = b - (b - a) * phi;
            fc = point_distance_sq(path, p, c);
        } else {
            a = c;
            c = d;
            fc = fd;
            d = a + (b - a) * phi;
            fd = point_distance_sq(path, p, d);
        }
    }

    let mut t = 0.5 * (a + b);
    for _ in 0..6 {
        let h = ((b - a) * 0.1).max(1e-4);
        let t0 = (t - h).clamp(0.0, 1.0);
        let t1 = t;
        let t2 = (t + h).clamp(0.0, 1.0);
        let f0 = point_distance_sq(path, p, t0);
        let f1 = point_distance_sq(path, p, t1);
        let f2 = point_distance_sq(path, p, t2);
        let denom = (t0 - t1) * (t0 - t2) * (t1 - t2);
        if denom.abs() < 1e-10 {
            break;
        }
        let a_quad = (t2 * (f1 - f0) + t1 * (f0 - f2) + t0 * (f2 - f1)) / denom;
        let b_quad = (t2 * t2 * (f0 - f1) + t1 * t1 * (f2 - f0) + t0 * t0 * (f1 - f2)) / denom;
        if a_quad.abs() < 1e-8 {
            break;
        }
        let candidate = (-b_quad / (2.0 * a_quad)).clamp(a, b);
        if (candidate - t).abs() < 1e-5 {
            break;
        }
        t = candidate;
    }

    t.clamp(0.0, 1.0)
}

fn build_coarse_ts(samples: usize) -> Vec<f32> {
    let n = samples.max(8);
    (0..n).map(|i| i as f32 / (n - 1) as f32).collect()
}

struct ExtendedPath {
    base: Arc<dyn SweepPath>,
    start_extension: f32,
    base_length: f32,
    total_length: f32,
    start_pos: Vec3,
    end_pos: Vec3,
    start_tangent: Vec3,
    end_tangent: Vec3,
}

impl ExtendedPath {
    fn new(base: Arc<dyn SweepPath>, start_extension: f32, end_extension: f32) -> Self {
        let base_length = base.arc_length().max(1e-4);
        let start_pos = base.evaluate(0.0);
        let end_pos = base.evaluate(1.0);
        let start_tangent = base.tangent(0.0).normalize_or_zero();
        let end_tangent = base.tangent(1.0).normalize_or_zero();
        let total_length = start_extension.max(0.0) + base_length + end_extension.max(0.0);
        Self {
            base,
            start_extension: start_extension.max(0.0),
            base_length,
            total_length,
            start_pos,
            end_pos,
            start_tangent,
            end_tangent,
        }
    }

    fn base_t_from_s(&self, s: f32) -> f32 {
        ((s - self.start_extension) / self.base_length).clamp(0.0, 1.0)
    }
}

impl SweepPath for ExtendedPath {
    fn evaluate(&self, t: f32) -> Vec3 {
        let s = t.clamp(0.0, 1.0) * self.total_length;
        if s < self.start_extension {
            self.start_pos - self.start_tangent * (self.start_extension - s)
        } else if s > self.start_extension + self.base_length {
            self.end_pos + self.end_tangent * (s - (self.start_extension + self.base_length))
        } else {
            self.base.evaluate(self.base_t_from_s(s))
        }
    }

    fn tangent(&self, t: f32) -> Vec3 {
        let s = t.clamp(0.0, 1.0) * self.total_length;
        if s < self.start_extension {
            self.start_tangent
        } else if s > self.start_extension + self.base_length {
            self.end_tangent
        } else {
            self.base.tangent(self.base_t_from_s(s))
        }
    }

    fn arc_length(&self) -> f32 {
        self.total_length
    }
}

/// A true centerline-distance circular tube field around a smooth path.
/// The closest point is refined on the actual curve parameter instead of
/// blending local sweep segments together.
pub struct SplineTube {
    path: Arc<dyn SweepPath>,
    coarse_ts: Vec<f32>,
    start_radius: f32,
    end_radius: f32,
    smoothness: f32,
}

impl SplineTube {
    pub fn new(
        path: Arc<dyn SweepPath>,
        start_diameter: f32,
        end_diameter: f32,
        samples: usize,
        smoothness: f32,
    ) -> Self {
        Self {
            path,
            coarse_ts: build_coarse_ts(samples),
            start_radius: (start_diameter * 0.5).max(1e-4),
            end_radius: (end_diameter * 0.5).max(1e-4),
            smoothness: smoothness.clamp(0.0, 1.0),
        }
    }

    fn closest_t(&self, p: Vec3) -> f32 {
        refined_closest_t(self.path.as_ref(), &self.coarse_ts, p)
    }

    fn signed_distance_at(&self, p: Vec3, t: f32) -> f32 {
        let center = self.path.evaluate(t);
        let r = radius_at(self.start_radius, self.end_radius, t, self.smoothness);
        (p - center).length() - r
    }
}

impl Sdf for SplineTube {
    fn distance(&self, p: Vec3) -> f32 {
        let t = self.closest_t(p);
        self.signed_distance_at(p, t)
    }
}

/// Hollow circular tube with open inlet/outlet. The inner void is extended
/// linearly beyond the start and end tangents to avoid closed caps.
pub struct HollowSplineTube {
    outer: SplineTube,
    inner: SplineTube,
}

impl HollowSplineTube {
    pub fn new(
        path: Arc<dyn SweepPath>,
        start_inner_diameter: f32,
        end_inner_diameter: f32,
        wall_thickness: f32,
        samples: usize,
        smoothness: f32,
    ) -> Self {
        let wt = wall_thickness.max(1e-4);
        let outer = SplineTube::new(
            Arc::clone(&path),
            start_inner_diameter + wt * 2.0,
            end_inner_diameter + wt * 2.0,
            samples,
            smoothness,
        );
        let start_extension = (start_inner_diameter * 0.6).max(wt * 4.0);
        let end_extension = (end_inner_diameter * 0.6).max(wt * 4.0);
        let inner_path: Arc<dyn SweepPath> = Arc::new(ExtendedPath::new(
            path,
            start_extension,
            end_extension,
        ));
        let inner = SplineTube::new(
            inner_path,
            start_inner_diameter,
            end_inner_diameter,
            samples,
            smoothness,
        );
        Self { outer, inner }
    }
}

impl Sdf for HollowSplineTube {
    fn distance(&self, p: Vec3) -> f32 {
        let outer_d = self.outer.distance(p);
        outer_d.max(-self.inner.distance(p))
    }
}

#[derive(Clone, Copy)]
struct FrameSample {
    t: f32,
    normal: Vec3,
    binormal: Vec3,
}

fn build_frame_samples(path: &dyn SweepPath, samples: usize) -> Vec<FrameSample> {
    use crate::sdf::sweep::compute_frames;

    let n = samples.max(8);
    compute_frames(path, n, 0.0, 0.0)
        .into_iter()
        .enumerate()
        .map(|(i, (_, frame))| FrameSample {
            t: i as f32 / (n - 1) as f32,
            normal: frame.col(1),
            binormal: frame.col(2),
        })
        .collect()
}

fn frame_at(
    samples: &[FrameSample],
    tangent: Vec3,
    t: f32,
) -> (Vec3, Vec3) {
    if samples.is_empty() {
        let (_, n, b) = make_frame(tangent);
        return (n, b);
    }
    if samples.len() == 1 {
        let b = tangent.cross(samples[0].normal).normalize_or_zero();
        let n = b.cross(tangent).normalize_or_zero();
        return if n.length_squared() > 1e-8 && b.length_squared() > 1e-8 {
            (n, b)
        } else {
            let (_, n, b) = make_frame(tangent);
            (n, b)
        };
    }

    let scaled = t.clamp(0.0, 1.0) * (samples.len() as f32 - 1.0);
    let i = scaled.floor() as usize;
    let i0 = i.min(samples.len() - 1);
    let i1 = (i0 + 1).min(samples.len() - 1);
    let a = samples[i0];
    let b = samples[i1];
    let denom = (b.t - a.t).abs().max(1e-6);
    let u = ((t - a.t) / denom).clamp(0.0, 1.0);

    let mut n = a.normal.lerp(b.normal, u).normalize_or_zero();
    let mut bin = a.binormal.lerp(b.binormal, u).normalize_or_zero();
    if n.length_squared() < 1e-8 || bin.length_squared() < 1e-8 {
        let (_, fallback_n, _) = make_frame(tangent);
        n = fallback_n;
    } else {
        n = (n - tangent * tangent.dot(n)).normalize_or_zero();
        bin = (bin - tangent * tangent.dot(bin) - n * n.dot(bin)).normalize_or_zero();
        if n.length_squared() < 1e-8 || bin.length_squared() < 1e-8 {
            let (_, fallback_n, _) = make_frame(tangent);
            n = fallback_n;
        }
    }
    bin = tangent.cross(n).normalize_or_zero();
    if bin.length_squared() < 1e-8 {
        let (_, fallback_n, fallback_b) = make_frame(tangent);
        n = fallback_n;
        bin = fallback_b;
    } else {
        n = bin.cross(tangent).normalize_or_zero();
    }
    (n, bin)
}

fn ellipse_radial_distance(local: glam::Vec2, half_width: f32, half_height: f32) -> f32 {
    let r = local.length();
    if r < 1e-6 {
        return -half_width.min(half_height);
    }
    let dir = local / r;
    let boundary_r = 1.0
        / ((dir.x * dir.x) / (half_width * half_width).max(1e-8)
        + (dir.y * dir.y) / (half_height * half_height).max(1e-8))
            .sqrt();
    r - boundary_r
}

/// Dedicated implicit duct field built around a sampled centerline with a
/// smoothly varying elliptical section.
pub struct VariableDuct {
    path: Arc<dyn SweepPath>,
    coarse_ts: Vec<f32>,
    frame_samples: Vec<FrameSample>,
    inlet_half_width: f32,
    inlet_half_height: f32,
    outlet_half_width: f32,
    outlet_half_height: f32,
    smoothness: f32,
}

impl VariableDuct {
    pub fn new(
        path: Arc<dyn SweepPath>,
        inlet_width: f32,
        inlet_height: f32,
        outlet_width: f32,
        outlet_height: f32,
        samples: usize,
        smoothness: f32,
    ) -> Self {
        Self {
            coarse_ts: build_coarse_ts(samples),
            frame_samples: build_frame_samples(path.as_ref(), samples),
            path,
            inlet_half_width: inlet_width * 0.5,
            inlet_half_height: inlet_height * 0.5,
            outlet_half_width: outlet_width * 0.5,
            outlet_half_height: outlet_height * 0.5,
            smoothness: smoothness.clamp(0.0, 1.0),
        }
    }

    fn closest_t(&self, p: Vec3) -> f32 {
        refined_closest_t(self.path.as_ref(), &self.coarse_ts, p)
    }

    fn half_width_at(&self, t: f32) -> f32 {
        smooth_profile_lerp(self.inlet_half_width, self.outlet_half_width, t, self.smoothness).max(1e-4)
    }

    fn half_height_at(&self, t: f32) -> f32 {
        smooth_profile_lerp(self.inlet_half_height, self.outlet_half_height, t, self.smoothness).max(1e-4)
    }

    fn profile_distance(&self, p: Vec3, t: f32) -> f32 {
        let center = self.path.evaluate(t);
        let tangent = self.path.tangent(t).normalize_or_zero();
        let (normal, binormal) = frame_at(&self.frame_samples, tangent, t);
        let rel = p - center;
        let axial = rel.dot(tangent);
        let local = glam::Vec2::new(rel.dot(normal), rel.dot(binormal));
        let profile_d = ellipse_radial_distance(local, self.half_width_at(t), self.half_height_at(t));

        if t <= 1e-4 && axial < 0.0 {
            let pd = profile_d.max(0.0);
            (pd * pd + axial * axial).sqrt()
        } else if t >= 1.0 - 1e-4 && axial > 0.0 {
            let pd = profile_d.max(0.0);
            (pd * pd + axial * axial).sqrt()
        } else {
            profile_d
        }
    }
}

impl Sdf for VariableDuct {
    fn distance(&self, p: Vec3) -> f32 {
        let t = self.closest_t(p);
        self.profile_distance(p, t)
    }
}

/// Hollow duct body with open inlet and outlet. The inner core extends beyond
/// both ends so subtracting it leaves the duct open instead of capped.
pub struct HollowVariableDuct {
    outer: VariableDuct,
    inner: VariableDuct,
}

impl HollowVariableDuct {
    pub fn new(
        path: Arc<dyn SweepPath>,
        inlet_width: f32,
        inlet_height: f32,
        outlet_width: f32,
        outlet_height: f32,
        wall_thickness: f32,
        samples: usize,
        smoothness: f32,
    ) -> Self {
        let start_extension = inlet_width.max(inlet_height) * 0.75 + wall_thickness * 2.0;
        let end_extension = outlet_width.max(outlet_height) * 0.75 + wall_thickness * 2.0;
        let outer = VariableDuct::new(
            Arc::clone(&path),
            inlet_width + wall_thickness * 2.0,
            inlet_height + wall_thickness * 2.0,
            outlet_width + wall_thickness * 2.0,
            outlet_height + wall_thickness * 2.0,
            samples,
            smoothness,
        );
        let inner_path: Arc<dyn SweepPath> = Arc::new(ExtendedPath::new(
            path,
            start_extension,
            end_extension,
        ));
        let inner = VariableDuct::new(
            inner_path,
            inlet_width,
            inlet_height,
            outlet_width,
            outlet_height,
            samples,
            smoothness,
        );
        Self { outer, inner }
    }
}

impl Sdf for HollowVariableDuct {
    fn distance(&self, p: Vec3) -> f32 {
        self.outer.distance(p).max(-self.inner.distance(p))
    }
}

/// Arbitrary-profile duct using the same refined closest-point-on-centerline
/// evaluation as the circular and elliptical duct kernels.
pub struct ProfileDuct {
    path: Arc<dyn SweepPath>,
    coarse_ts: Vec<f32>,
    frame_samples: Vec<FrameSample>,
    start_profile: Arc<dyn Section2D>,
    end_profile: Arc<dyn Section2D>,
}

impl ProfileDuct {
    pub fn new(
        path: Arc<dyn SweepPath>,
        start_profile: Arc<dyn Section2D>,
        end_profile: Arc<dyn Section2D>,
        samples: usize,
    ) -> Self {
        Self::new_with_frames(
            Arc::clone(&path),
            start_profile,
            end_profile,
            build_frame_samples(path.as_ref(), samples),
            samples,
        )
    }

    pub fn new_with_frames(
        path: Arc<dyn SweepPath>,
        start_profile: Arc<dyn Section2D>,
        end_profile: Arc<dyn Section2D>,
        frame_samples: Vec<FrameSample>,
        samples: usize,
    ) -> Self {
        let coarse_ts = build_coarse_ts(samples);
        Self {
            path,
            coarse_ts,
            frame_samples,
            start_profile,
            end_profile,
        }
    }

    fn closest_t(&self, p: Vec3) -> f32 {
        refined_closest_t(self.path.as_ref(), &self.coarse_ts, p)
    }

    fn profile_distance(&self, p: Vec3, t: f32) -> f32 {
        let center = self.path.evaluate(t);
        let tangent = self.path.tangent(t).normalize_or_zero();
        let (normal, binormal) = frame_at(&self.frame_samples, tangent, t);
        let rel = p - center;
        let axial = rel.dot(tangent);
        let local = glam::Vec2::new(rel.dot(normal), rel.dot(binormal));
        let profile_d = self.start_profile.distance_lerped_2d(self.end_profile.as_ref(), t, local);

        if t <= 1e-4 && axial < 0.0 {
            let pd = profile_d.max(0.0);
            (pd * pd + axial * axial).sqrt()
        } else if t >= 1.0 - 1e-4 && axial > 0.0 {
            let pd = profile_d.max(0.0);
            (pd * pd + axial * axial).sqrt()
        } else {
            profile_d
        }
    }
}

impl Sdf for ProfileDuct {
    fn distance(&self, p: Vec3) -> f32 {
        let t = self.closest_t(p);
        self.profile_distance(p, t)
    }
}

/// Hollow arbitrary-profile duct with explicitly supplied outer and inner
/// start/end profiles so custom inlets can use different inner/outer shapes
/// without relying on generic offset approximations.
pub struct HollowProfileDuct {
    outer: ProfileDuct,
    inner: ProfileDuct,
}

impl HollowProfileDuct {
    pub fn new(
        path: Arc<dyn SweepPath>,
        outer_start: Arc<dyn Section2D>,
        outer_end: Arc<dyn Section2D>,
        inner_start: Arc<dyn Section2D>,
        inner_end: Arc<dyn Section2D>,
        start_extension: f32,
        end_extension: f32,
        samples: usize,
    ) -> Self {
        let outer = ProfileDuct::new(
            Arc::clone(&path),
            outer_start,
            outer_end,
            samples,
        );
        let inner_path: Arc<dyn SweepPath> = Arc::new(ExtendedPath::new(path, start_extension, end_extension));
        let inner = ProfileDuct::new(
            inner_path,
            inner_start,
            inner_end,
            samples,
        );
        Self { outer, inner }
    }
}

impl Sdf for HollowProfileDuct {
    fn distance(&self, p: Vec3) -> f32 {
        self.outer.distance(p).max(-self.inner.distance(p))
    }
}

struct ScaledSection {
    inner: Arc<dyn Section2D>,
    sx: f32,
    sy: f32,
}

impl ScaledSection {
    fn new(inner: Arc<dyn Section2D>, sx: f32, sy: f32) -> Self {
        Self {
            inner,
            sx: sx.max(1e-4),
            sy: sy.max(1e-4),
        }
    }
}

impl Section2D for ScaledSection {
    fn distance_2d(&self, point: Vec2) -> f32 {
        let p = Vec2::new(point.x / self.sx, point.y / self.sy);
        self.inner.distance_2d(p) * self.sx.min(self.sy)
    }

    fn distance_lerped_2d(&self, other: &dyn Section2D, t: f32, point: Vec2) -> f32 {
        self.distance_2d(point) * (1.0 - t) + other.distance_2d(point) * t
    }

    fn lerp_to(&self, other: &dyn Section2D, t: f32) -> Arc<dyn Section2D> {
        Arc::new(ScaledSection::new(self.inner.lerp_to(other, t), self.sx, self.sy))
    }

    fn as_any(&self) -> &dyn Any { self }
}

#[derive(Clone)]
struct TopCapProfile {
    width: f32,
    height: f32,
    radius: f32,
}

impl TopCapProfile {
    fn new(width: f32, height: f32, radius: f32) -> Self {
        Self {
            width: width.max(1e-4),
            height: height.max(1e-4),
            radius: radius.max(0.0),
        }
    }
}

impl Section2D for TopCapProfile {
    fn distance_2d(&self, point: Vec2) -> f32 {
        let rr = RoundedRectProfile::new(self.width, self.height, self.radius);
        let shifted = Vec2::new(point.x, point.y - self.height * 0.5);
        let d_rr = rr.distance_2d(shifted);
        let d_floor = -point.y;
        d_rr.max(d_floor)
    }

    fn distance_lerped_2d(&self, other: &dyn Section2D, t: f32, point: Vec2) -> f32 {
        self.distance_2d(point) * (1.0 - t) + other.distance_2d(point) * t
    }

    fn lerp_to(&self, other: &dyn Section2D, t: f32) -> Arc<dyn Section2D> {
        if t < 0.5 {
            Arc::new(self.clone())
        } else {
            other.lerp_to(other, 0.0)
        }
    }

    fn as_any(&self) -> &dyn Any { self }
}

fn sample_section_outline(section: &dyn Section2D, samples: usize) -> Vec<Vec2> {
    if let Some(profile) = section.as_any().downcast_ref::<SplineProfile>() {
        return profile.sample(samples.max(24));
    }
    if let Some(profile) = section.as_any().downcast_ref::<RectProfile>() {
        return vec![
            Vec2::new(-profile.half_w, -profile.half_h),
            Vec2::new(profile.half_w, -profile.half_h),
            Vec2::new(profile.half_w, profile.half_h),
            Vec2::new(-profile.half_w, profile.half_h),
        ];
    }
    if let Some(profile) = section.as_any().downcast_ref::<RoundedRectProfile>() {
        let n = samples.max(32);
        let mut pts = Vec::with_capacity(n);
        let inner_w = (profile.half_w - profile.radius).max(0.0);
        let inner_h = (profile.half_h - profile.radius).max(0.0);
        for i in 0..n {
            let a = std::f32::consts::TAU * i as f32 / n as f32;
            let sx = a.cos().signum();
            let sy = a.sin().signum();
            pts.push(Vec2::new(
                sx * inner_w + a.cos() * profile.radius,
                sy * inner_h + a.sin() * profile.radius,
            ));
        }
        return pts;
    }
    if let Some(profile) = section.as_any().downcast_ref::<NGonProfile>() {
        return (0..profile.sides).map(|i| {
            let a = std::f32::consts::TAU * i as f32 / profile.sides as f32;
            Vec2::new(a.cos() * profile.radius, a.sin() * profile.radius)
        }).collect();
    }

    let n = samples.max(48);
    let mut pts = Vec::with_capacity(n);
    for i in 0..n {
        let a = std::f32::consts::TAU * i as f32 / n as f32;
        let dir = Vec2::new(a.cos(), a.sin());
        let mut hi = 1.0_f32;
        let mut iters = 0;
        while section.distance_2d(dir * hi) <= 0.0 && iters < 32 {
            hi *= 2.0;
            iters += 1;
        }
        let mut lo = 0.0_f32;
        for _ in 0..32 {
            let mid = 0.5 * (lo + hi);
            if section.distance_2d(dir * mid) <= 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        pts.push(dir * lo);
    }
    pts
}

fn section_lower_extent(section: &dyn Section2D, samples: usize) -> f32 {
    sample_section_outline(section, samples)
        .into_iter()
        .map(|p| -p.y)
        .fold(0.0_f32, f32::max)
}

fn fit_center_outside_offset_surface(
    surface_limit: &dyn Sdf,
    center: Vec3,
    lateral: Vec3,
    upward: Vec3,
    outline: &[Vec2],
    clearance: f32,
) -> Vec3 {
    let mut c = center;
    let l = lateral.normalize_or_zero();
    let u = upward.normalize_or_zero();
    if l.length_squared() < 1e-8 || u.length_squared() < 1e-8 {
        return c;
    }
    for _ in 0..10 {
        let mut min_d = f32::INFINITY;
        for p in outline {
            let wp = c + l * p.x + u * p.y;
            min_d = min_d.min(surface_limit.distance(wp));
        }
        if min_d >= clearance {
            break;
        }
        c += u * (clearance - min_d + 0.02);
    }
    c
}

pub struct ConformalProfileInletParts {
    pub outer_fairing: Arc<dyn Sdf>,
    pub duct_void: Arc<dyn Sdf>,
    pub internal_shell: Arc<dyn Sdf>,
}

pub fn build_conformal_profile_inlet(
    surface: Arc<dyn Sdf>,
    guide_points: Vec<Vec3>,
    duct_path: Arc<dyn SweepPath>,
    outer_start: Arc<dyn Section2D>,
    outer_end: Arc<dyn Section2D>,
    inner_start: Arc<dyn Section2D>,
    inner_end: Arc<dyn Section2D>,
    surface_offset: f32,
    inlet_open_extension: f32,
    outlet_open_extension: f32,
    samples: usize,
) -> ConformalProfileInletParts {
    let sample_count = samples.max(8);
    let surface_limit: Arc<dyn Sdf> = Arc::new(Offset::new(Arc::clone(&surface), surface_offset));
    let guide_spline = PolylinePath::new(guide_points);
    let fairing_t_end = 0.42_f32;
    let fairing_start: Arc<dyn Section2D> = Arc::new(TopCapProfile::new(70.0, 24.0, 6.0));
    let fairing_end: Arc<dyn Section2D> = Arc::new(TopCapProfile::new(20.0, 8.0, 3.0));
    let start_lift = section_lower_extent(fairing_start.as_ref(), sample_count);
    let end_lift = section_lower_extent(fairing_end.as_ref(), sample_count);
    let start_outline = sample_section_outline(fairing_start.as_ref(), sample_count);
    let end_outline = sample_section_outline(fairing_end.as_ref(), sample_count);
    let mut conformal_points = Vec::with_capacity(sample_count);
    let mut fairing_frames = Vec::with_capacity(sample_count);
    for i in 0..sample_count {
        let u = i as f32 / (sample_count - 1) as f32;
        let t = fairing_t_end * u;
        let guide = guide_spline.evaluate(t);
        let (offset_point, normal) =
            project_point_to_surface_with_offset_and_normal(surface.as_ref(), guide, surface_offset);
        let end_fade = 1.0 - smoothstep(u);
        let lift = smooth_profile_lerp(start_lift, end_lift, u, 0.85) * end_fade;
        let mut center = offset_point + normal * lift;

        let tangent = guide_spline.tangent(t).normalize_or_zero();
        let mut upward = (normal - tangent * tangent.dot(normal)).normalize_or_zero();
        if upward.length_squared() < 1e-8 {
            let (_, _, fallback_up) = make_frame(tangent);
            upward = fallback_up;
        }
        let mut lateral = upward.cross(tangent).normalize_or_zero();
        if lateral.length_squared() < 1e-8 {
            let (_, fallback_lateral, fallback_up) = make_frame(tangent);
            lateral = fallback_lateral;
            upward = fallback_up;
        } else {
            upward = tangent.cross(lateral).normalize_or_zero();
        }
        let outline = if u < 0.5 { &start_outline } else { &end_outline };
        center = fit_center_outside_offset_surface(
            surface_limit.as_ref(),
            center,
            lateral,
            upward,
            outline,
            0.2,
        );
        conformal_points.push(center);
        fairing_frames.push(FrameSample {
            t: u,
            normal: lateral,
            binormal: upward,
        });
    }
    let conformal_path: Arc<dyn SweepPath> = Arc::new(PolylinePath::new(conformal_points));
    let fairing_proto: Arc<dyn Sdf> = Arc::new(ProfileDuct::new_with_frames(
        conformal_path,
        Arc::clone(&fairing_start),
        Arc::clone(&fairing_end),
        fairing_frames,
        sample_count,
    ));
    let fairing_blend: Arc<dyn Sdf> = Arc::new(SmoothUnion::new(
        Arc::clone(&surface),
        fairing_proto,
        10.0,
    ));
    let outer_fairing: Arc<dyn Sdf> = Arc::new(Subtract::new(
        fairing_blend,
        Arc::clone(&surface),
    ));

    let internal_shell_proto: Arc<dyn Sdf> = Arc::new(HollowProfileDuct::new(
        Arc::clone(&duct_path),
        outer_start,
        outer_end,
        Arc::clone(&inner_start),
        Arc::clone(&inner_end),
        0.0,
        0.0,
        sample_count,
    ));
    let internal_shell: Arc<dyn Sdf> = Arc::new(Intersect::new(
        internal_shell_proto,
        Arc::clone(&surface_limit),
    ));

    let extended_duct_path: Arc<dyn SweepPath> = Arc::new(ExtendedPath::new(
        duct_path,
        inlet_open_extension.max(0.0),
        outlet_open_extension.max(0.0),
    ));
    let duct_void: Arc<dyn Sdf> = Arc::new(ProfileDuct::new(
        extended_duct_path,
        inner_start,
        inner_end,
        sample_count,
    ));

    ConformalProfileInletParts {
        outer_fairing,
        duct_void,
        internal_shell,
    }
}

// ── BuriedInlet ───────────────────────────────────────────────────────────────

/// A buried inlet: elliptical surface scoop leading to a cylindrical duct throat.
///
/// Centred at the origin. The scoop opening faces +Z (outward normal).
/// The duct runs in the -Z direction for `duct_length` mm.
pub struct BuriedInlet {
    /// Inlet scoop half-width (Y axis, mm).
    pub scoop_width: f32,
    /// Inlet scoop half-length (X axis, mm).
    pub scoop_length: f32,
    /// Throat radius (mm).
    pub throat_r: f32,
    /// Depth from surface to duct entry (mm).
    pub surface_offset: f32,
    /// Duct length below surface (mm).
    pub duct_length: f32,
}

impl BuriedInlet {
    pub fn new(throat_r: f32, duct_length: f32, surface_offset: f32) -> Self {
        Self {
            scoop_width:   throat_r * 1.6,
            scoop_length:  throat_r * 2.2,
            throat_r,
            surface_offset,
            duct_length,
        }
    }
}

impl Sdf for BuriedInlet {
    fn distance(&self, p: Vec3) -> f32 {
        // Scoop: flattened hemi-ellipsoid at z = surface_offset
        let sc = Vec3::new(p.x / self.scoop_length, p.y / self.scoop_width,
                           (p.z - self.surface_offset) / self.throat_r);
        let d_scoop = sc.length() - 1.0;  // normalised sphere (squashed)
        // Only the lower half (z < surface_offset)
        let d_scoop = d_scoop.max(-(p.z - self.surface_offset));

        // Duct: cylinder running from z=surface_offset down to z=surface_offset-duct_length
        let r2 = (p.x * p.x + p.y * p.y).sqrt();
        let z_lo = self.surface_offset - self.duct_length;
        let d_duct_r  = r2 - self.throat_r;
        let d_duct_top = p.z - self.surface_offset;
        let d_duct_bot = z_lo - p.z;
        let d_duct = d_duct_r.max(d_duct_top).max(d_duct_bot);

        d_scoop.min(d_duct)
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::profiles::{RoundedRectProfile, SplineProfile};
    use crate::sdf::sweep::SplinePath;
    use crate::sdf::primitives::Sphere;

    #[test]
    fn test_naca_inlet_finite() {
        let inlet = NacaInlet {
            width: 20.0,
            length: 40.0,
            depth: 5.0,
            ramp_angle_deg: 7.0,
            position: Vec3::ZERO,
            normal: Vec3::Z,
            flow_direction: -Vec3::X,
        };
        let d = inlet.distance(Vec3::new(20.0, 0.0, -2.0));
        assert!(d.is_finite());
    }

    #[test]
    fn test_edf_duct_tapered_capsule() {
        let d = capsule_sdf_tapered(
            Vec3::new(5.0, 0.0, 0.0),
            Vec3::ZERO,
            Vec3::new(10.0, 0.0, 0.0),
            5.0,
            5.0,
        );
        assert!(d < 0.0, "point inside capsule should be negative, got {}", d);
    }

    #[test]
    fn test_edf_duct_outside() {
        let duct = EdfDuct {
            inlet_position: Vec3::ZERO,
            inlet_radius: 20.0,
            fan_position: Vec3::new(100.0, 0.0, 0.0),
            fan_radius: 25.0,
            exhaust_position: Vec3::new(150.0, 0.0, 0.0),
            exhaust_radius: 22.0,
            exhaust_direction: Vec3::X,
            control_points: vec![],
        };
        let d = duct.distance(Vec3::new(50.0, 100.0, 0.0));
        assert!(d > 0.0, "point far outside duct should be positive, got {}", d);
    }

    #[test]
    fn test_circular_inlet_lip() {
        let lip = InletLip {
            shape: InletShape::Circular { diameter: 50.0 },
            lip_radius: 3.0,
            position: Vec3::ZERO,
            direction: Vec3::X,
            highlight_to_throat: 25.0,
            throat_area_fraction: 0.92,
        };
        // Point far away should be positive
        let d = lip.distance(Vec3::new(200.0, 200.0, 200.0));
        assert!(d > 0.0);
    }

    #[test]
    fn test_spline_tube_centerline_is_inside() {
        let path: Arc<dyn SweepPath> = Arc::new(SplinePath::new(vec![
            Vec3::new(0.0, 0.0, 46.0),
            Vec3::new(72.0, 0.0, 58.0),
            Vec3::new(248.0, 0.0, 8.0),
            Vec3::new(430.0, 0.0, 0.0),
        ]));
        let tube = SplineTube::new(path, 94.0, 94.0, 96, 0.95);
        for t in [0.0_f32, 0.2, 0.5, 0.8, 1.0] {
            let p = tube.path.evaluate(t);
            assert!(tube.distance(p) < -40.0, "centerline should be deep inside at t={}", t);
        }
    }

    #[test]
    fn test_hollow_spline_tube_end_is_open() {
        let path: Arc<dyn SweepPath> = Arc::new(SplinePath::new(vec![
            Vec3::new(0.0, 0.0, 46.0),
            Vec3::new(72.0, 0.0, 58.0),
            Vec3::new(248.0, 0.0, 8.0),
            Vec3::new(430.0, 0.0, 0.0),
        ]));
        let tube = HollowSplineTube::new(path, 90.0, 90.0, 2.0, 96, 0.95);
        let near_start_axis = Vec3::new(-20.0, 0.0, 46.0);
        assert!(tube.distance(near_start_axis) > 0.0, "open mouth axis should remain empty");
    }

    #[test]
    fn test_variable_duct_centerline_follows_curve() {
        let path: Arc<dyn SweepPath> = Arc::new(SplinePath::new(vec![
            Vec3::new(0.0, 0.0, 46.0),
            Vec3::new(20.0, 0.0, 52.0),
            Vec3::new(56.0, 0.0, 58.0),
            Vec3::new(108.0, 0.0, 60.0),
            Vec3::new(164.0, 0.0, 48.0),
            Vec3::new(218.0, 0.0, 24.0),
            Vec3::new(266.0, 0.0, 8.0),
            Vec3::new(312.0, 0.0, 0.0),
            Vec3::new(430.0, 0.0, 0.0),
        ]));
        let duct = VariableDuct::new(path, 96.0, 88.0, 90.0, 90.0, 160, 0.98);
        for t in [0.1_f32, 0.25, 0.45, 0.65, 0.85] {
            let p = duct.path.evaluate(t);
            assert!(duct.distance(p) < -38.0, "centerline should stay inside at t={}", t);
        }
    }

    #[test]
    fn test_profile_duct_centerline_follows_curve() {
        let path: Arc<dyn SweepPath> = Arc::new(SplinePath::new(vec![
            Vec3::new(0.0, 0.0, 46.0),
            Vec3::new(20.0, 0.0, 52.0),
            Vec3::new(56.0, 0.0, 58.0),
            Vec3::new(108.0, 0.0, 60.0),
            Vec3::new(164.0, 0.0, 48.0),
            Vec3::new(218.0, 0.0, 24.0),
            Vec3::new(266.0, 0.0, 8.0),
            Vec3::new(312.0, 0.0, 0.0),
            Vec3::new(430.0, 0.0, 0.0),
        ]));
        let start: Arc<dyn Section2D> = Arc::new(RoundedRectProfile::new(96.0, 72.0, 12.0));
        let end: Arc<dyn Section2D> = Arc::new(SplineProfile::circle(16, 45.0));
        let duct = ProfileDuct::new(path, start, end, 160);
        for t in [0.1_f32, 0.25, 0.45, 0.65, 0.85] {
            let p = duct.path.evaluate(t);
            assert!(duct.distance(p) < -30.0, "profile centerline should stay inside at t={}", t);
        }
    }

    #[test]
    fn test_conformal_profile_inlet_builds_three_parts() {
        let surface: Arc<dyn Sdf> = Arc::new(Sphere::new(10.0));
        let duct_path: Arc<dyn SweepPath> = Arc::new(SplinePath::new(vec![
            Vec3::new(0.0, 0.0, 11.5),
            Vec3::new(1.5, 0.0, 10.5),
            Vec3::new(3.5, 0.0, 7.0),
            Vec3::new(6.0, 0.0, 0.0),
        ]));
        let outer_start: Arc<dyn Section2D> = Arc::new(RoundedRectProfile::new(4.0, 3.0, 0.6));
        let outer_end: Arc<dyn Section2D> = Arc::new(SplineProfile::circle(16, 1.4));
        let inner_start: Arc<dyn Section2D> = Arc::new(RoundedRectProfile::new(3.2, 2.2, 0.5));
        let inner_end: Arc<dyn Section2D> = Arc::new(SplineProfile::circle(16, 1.1));
        let parts = build_conformal_profile_inlet(
            surface,
            vec![
                Vec3::new(-2.0, 0.0, 12.0),
                Vec3::new(0.0, 0.0, 12.5),
                Vec3::new(2.0, 0.0, 11.8),
            ],
            duct_path,
            outer_start,
            outer_end,
            inner_start,
            inner_end,
            1.0,
            2.0,
            2.0,
            48,
        );

        assert!(parts.outer_fairing.distance(Vec3::new(0.0, 0.0, 12.5)).is_finite());
        assert!(parts.duct_void.distance(Vec3::new(5.5, 0.0, 0.0)).is_finite());
        assert!(parts.internal_shell.distance(Vec3::new(3.5, 0.0, 7.0)).is_finite());
    }
}
