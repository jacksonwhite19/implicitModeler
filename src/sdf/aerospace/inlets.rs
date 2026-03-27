// Aerodynamic inlet geometry: NACA flush inlets, EDF buried inlets, inlet lips,
// EDF duct system, and exhaust nozzles.

use glam::{Vec2, Vec3};
use std::sync::Arc;
use crate::sdf::Sdf;
use crate::sdf::booleans::{Intersect, Subtract, Union};
use crate::sdf::patterns::Mirror;
use crate::sdf::profiles::{NGonProfile, RectProfile, RoundedRectProfile, SplineProfile};
use crate::sdf::query::gradient as sdf_gradient;
use crate::sdf::sweep::{PolylinePath, SweepPath, project_point_to_surface_with_offset_and_normal};
use crate::sdf::transforms::{Offset, Translate};
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

    fn new_with_frames(
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

/// Arbitrary-profile duct that keeps the section fixed in world YZ
/// orientation while its center moves along the path. This is useful for
/// debugging and for ducts that should preserve a measured handoff section
/// instead of rotating with the path tangent.
pub struct FixedProfileDuct {
    path: Arc<dyn SweepPath>,
    coarse_ts: Vec<f32>,
    start_profile: Arc<dyn Section2D>,
    end_profile: Arc<dyn Section2D>,
    morph_start: f32,
    morph_end: f32,
}

impl FixedProfileDuct {
    pub fn new(
        path: Arc<dyn SweepPath>,
        start_profile: Arc<dyn Section2D>,
        end_profile: Arc<dyn Section2D>,
        samples: usize,
    ) -> Self {
        Self::with_schedule(path, start_profile, end_profile, 0.0, 1.0, samples)
    }

    pub fn with_schedule(
        path: Arc<dyn SweepPath>,
        start_profile: Arc<dyn Section2D>,
        end_profile: Arc<dyn Section2D>,
        morph_start: f32,
        morph_end: f32,
        samples: usize,
    ) -> Self {
        let ms = morph_start.clamp(0.0, 1.0);
        let me = morph_end.clamp(ms, 1.0);
        Self {
            path,
            coarse_ts: build_coarse_ts(samples),
            start_profile,
            end_profile,
            morph_start: ms,
            morph_end: me,
        }
    }

    fn closest_t(&self, p: Vec3) -> f32 {
        refined_closest_t(self.path.as_ref(), &self.coarse_ts, p)
    }

    fn profile_t(&self, t: f32) -> f32 {
        if t <= self.morph_start {
            0.0
        } else if t >= self.morph_end {
            1.0
        } else {
            let span = (self.morph_end - self.morph_start).max(1e-6);
            smoothstep((t - self.morph_start) / span)
        }
    }
}

impl Sdf for FixedProfileDuct {
    fn distance(&self, p: Vec3) -> f32 {
        let t = self.closest_t(p);
        let center = self.path.evaluate(t);
        let tangent = self.path.tangent(t).normalize_or_zero();
        let rel = p - center;
        let axial = rel.dot(tangent);
        let local = Vec2::new(rel.y, rel.z);
        let profile_t = self.profile_t(t);
        let profile_d = self.start_profile.distance_lerped_2d(self.end_profile.as_ref(), profile_t, local);

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

pub struct HollowFixedProfileDuct {
    outer: FixedProfileDuct,
    inner: FixedProfileDuct,
}

impl HollowFixedProfileDuct {
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
        Self::with_schedule(
            path,
            outer_start,
            outer_end,
            inner_start,
            inner_end,
            start_extension,
            end_extension,
            0.0,
            1.0,
            samples,
        )
    }

    pub fn with_schedule(
        path: Arc<dyn SweepPath>,
        outer_start: Arc<dyn Section2D>,
        outer_end: Arc<dyn Section2D>,
        inner_start: Arc<dyn Section2D>,
        inner_end: Arc<dyn Section2D>,
        start_extension: f32,
        end_extension: f32,
        morph_start: f32,
        morph_end: f32,
        samples: usize,
    ) -> Self {
        let outer = FixedProfileDuct::with_schedule(
            Arc::clone(&path),
            outer_start,
            outer_end,
            morph_start,
            morph_end,
            samples,
        );
        let inner_path: Arc<dyn SweepPath> = Arc::new(ExtendedPath::new(path, start_extension, end_extension));
        let inner = FixedProfileDuct::with_schedule(
            inner_path,
            inner_start,
            inner_end,
            morph_start,
            morph_end,
            samples,
        );
        Self { outer, inner }
    }
}

impl Sdf for HollowFixedProfileDuct {
    fn distance(&self, p: Vec3) -> f32 {
        self.outer.distance(p).max(-self.inner.distance(p))
    }
}

struct StraightProfilePatch {
    origin: Vec3,
    forward: Vec3,
    length: f32,
    start_profile: Arc<dyn Section2D>,
    end_profile: Arc<dyn Section2D>,
}

impl StraightProfilePatch {
    fn new(
        start_center: Vec3,
        end_center: Vec3,
        start_profile: Arc<dyn Section2D>,
        end_profile: Arc<dyn Section2D>,
    ) -> Self {
        let delta = end_center - start_center;
        let length = delta.length().max(1e-4);
        let forward = if delta.length_squared() < 1e-8 {
            Vec3::X
        } else {
            delta / length
        };
        Self {
            origin: start_center,
            forward,
            length,
            start_profile,
            end_profile,
        }
    }
}

impl Sdf for StraightProfilePatch {
    fn distance(&self, p: Vec3) -> f32 {
        let local = to_local(p, self.origin, self.forward);
        let x = local.x;
        let yz = Vec2::new(local.y, local.z);
        let t = (x / self.length).clamp(0.0, 1.0);
        let profile_d = self.start_profile.distance_lerped_2d(self.end_profile.as_ref(), t, yz);
        if x < 0.0 {
            let pd = profile_d.max(0.0);
            (pd * pd + x * x).sqrt()
        } else if x > self.length {
            let pd = profile_d.max(0.0);
            let dx = x - self.length;
            (pd * pd + dx * dx).sqrt()
        } else {
            profile_d
        }
    }
}

#[allow(dead_code)]
struct ScaledSection {
    inner: Arc<dyn Section2D>,
    sx: f32,
    sy: f32,
}

struct OffsetSection {
    inner: Arc<dyn Section2D>,
    offset: f32,
}

impl OffsetSection {
    fn new(inner: Arc<dyn Section2D>, offset: f32) -> Self {
        Self { inner, offset }
    }
}

impl Section2D for OffsetSection {
    fn distance_2d(&self, point: Vec2) -> f32 {
        self.inner.distance_2d(point) + self.offset
    }

    fn distance_lerped_2d(&self, other: &dyn Section2D, t: f32, point: Vec2) -> f32 {
        self.distance_2d(point) * (1.0 - t) + other.distance_2d(point) * t
    }

    fn lerp_to(&self, other: &dyn Section2D, t: f32) -> Arc<dyn Section2D> {
        Arc::new(OffsetSection::new(self.inner.lerp_to(other, t), self.offset))
    }

    fn as_any(&self) -> &dyn Any { self }
}

impl ScaledSection {
    #[allow(dead_code)]
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

struct TransposedSection {
    inner: Arc<dyn Section2D>,
}

impl TransposedSection {
    fn new(inner: Arc<dyn Section2D>) -> Self {
        Self { inner }
    }
}

impl Section2D for TransposedSection {
    fn distance_2d(&self, point: Vec2) -> f32 {
        self.inner.distance_2d(Vec2::new(point.y, point.x))
    }

    fn distance_lerped_2d(&self, other: &dyn Section2D, t: f32, point: Vec2) -> f32 {
        self.distance_2d(point) * (1.0 - t) + other.distance_2d(point) * t
    }

    fn lerp_to(&self, other: &dyn Section2D, t: f32) -> Arc<dyn Section2D> {
        let other_inner: Arc<dyn Section2D> = if let Some(o) = other.as_any().downcast_ref::<TransposedSection>() {
            Arc::clone(&o.inner)
        } else {
            other.lerp_to(other, 0.0)
        };
        Arc::new(TransposedSection::new(self.inner.lerp_to(other_inner.as_ref(), t)))
    }

    fn as_any(&self) -> &dyn Any { self }
}

#[derive(Clone)]
struct PolygonSection {
    points: Vec<Vec2>,
}

impl PolygonSection {
    fn new(points: Vec<Vec2>) -> Self {
        Self { points }
    }
}

impl Section2D for PolygonSection {
    fn distance_2d(&self, point: Vec2) -> f32 {
        if self.points.len() < 3 {
            return f32::MAX;
        }
        let n = self.points.len();
        let mut min_d2 = f32::MAX;
        let mut winding = 0i32;
        for i in 0..n {
            let a = self.points[i];
            let b = self.points[(i + 1) % n];
            let ab = b - a;
            let ap = point - a;
            let len2 = ab.dot(ab);
            let t = if len2 < 1e-12 { 0.0 } else { (ap.dot(ab) / len2).clamp(0.0, 1.0) };
            let d2 = (point - (a + ab * t)).length_squared();
            min_d2 = min_d2.min(d2);

            let left = (b.x - a.x) * (point.y - a.y) - (point.x - a.x) * (b.y - a.y);
            if a.y <= point.y {
                if b.y > point.y && left > 0.0 {
                    winding += 1;
                }
            } else if b.y <= point.y && left < 0.0 {
                winding -= 1;
            }
        }
        let sign = if winding != 0 { -1.0 } else { 1.0 };
        sign * min_d2.sqrt()
    }

    fn lerp_to(&self, other: &dyn Section2D, t: f32) -> Arc<dyn Section2D> {
        let self_pts = self.points.clone();
        let other_pts = sample_section_outline(other, self.points.len().max(64));
        let self_pts = if self_pts.len() == other_pts.len() {
            self_pts
        } else {
            sample_section_outline(self, other_pts.len())
        };
        let pts = self_pts.into_iter().zip(other_pts)
            .map(|(a, b)| a.lerp(b, t))
            .collect();
        Arc::new(PolygonSection::new(pts))
    }

    fn as_any(&self) -> &dyn Any { self }
}

#[derive(Clone)]
struct RoundedRectLipProfile {
    half_w: f32,
    half_h: f32,
    radius: f32,
    floor_pts: Vec<Vec2>,
}

impl RoundedRectLipProfile {
    fn floor_y(&self, x: f32) -> f32 {
        if self.floor_pts.is_empty() {
            return -self.half_h;
        }
        let x = x.clamp(-self.half_w, self.half_w);
        for w in self.floor_pts.windows(2) {
            let a = w[0];
            let b = w[1];
            if x >= a.x && x <= b.x {
                let dx = (b.x - a.x).abs().max(1e-6);
                let t = ((x - a.x) / dx).clamp(0.0, 1.0);
                return a.y + (b.y - a.y) * t;
            }
        }
        if x <= self.floor_pts[0].x {
            self.floor_pts[0].y
        } else {
            self.floor_pts[self.floor_pts.len() - 1].y
        }
    }
}

impl Section2D for RoundedRectLipProfile {
    fn distance_2d(&self, point: Vec2) -> f32 {
        let rr = RoundedRectProfile {
            half_w: self.half_w,
            half_h: self.half_h,
            radius: self.radius,
        };
        let d_rr = rr.distance_2d(point);
        let d_floor = self.floor_y(point.x) - point.y;
        d_rr.max(d_floor)
    }

    fn lerp_to(&self, other: &dyn Section2D, t: f32) -> Arc<dyn Section2D> {
        let other_pts = sample_section_outline(other, self.floor_pts.len().max(64));
        let self_pts = sample_section_outline(self, other_pts.len());
        let pts = self_pts.into_iter().zip(other_pts)
            .map(|(a, b)| a.lerp(b, t))
            .collect();
        Arc::new(PolygonSection::new(pts))
    }

    fn as_any(&self) -> &dyn Any { self }
}

fn sample_section_outline(section: &dyn Section2D, samples: usize) -> Vec<Vec2> {
    if let Some(profile) = section.as_any().downcast_ref::<PolygonSection>() {
        return profile.points.clone();
    }
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

fn rounded_rect_params(section: &dyn Section2D) -> Option<(f32, f32, f32)> {
    if let Some(rr) = section.as_any().downcast_ref::<RoundedRectProfile>() {
        return Some((rr.half_w, rr.half_h, rr.radius));
    }
    if let Some(t) = section.as_any().downcast_ref::<TransposedSection>() {
        if let Some(rr) = t.inner.as_any().downcast_ref::<RoundedRectProfile>() {
            return Some((rr.half_w, rr.half_h, rr.radius));
        }
    }
    None
}

fn sample_surface_along_axis(
    surface: &dyn Sdf,
    origin: Vec3,
    axis: Vec3,
    search_half_span: f32,
) -> Option<Vec3> {
    let dir = axis.normalize_or_zero();
    if dir.length_squared() < 1e-8 {
        return None;
    }

    let mut prev_t = search_half_span;
    let mut prev_d = surface.distance(origin + dir * prev_t);
    let steps = 96;
    for i in 1..=steps {
        let t = search_half_span - (2.0 * search_half_span) * (i as f32 / steps as f32);
        let d = surface.distance(origin + dir * t);
        if prev_d.signum() != d.signum() || d.abs() < 1e-4 {
            let mut lo_t = prev_t.min(t);
            let mut hi_t = prev_t.max(t);
            let mut lo_d = surface.distance(origin + dir * lo_t);
            let mut hi_d = surface.distance(origin + dir * hi_t);
            for _ in 0..32 {
                let mid_t = 0.5 * (lo_t + hi_t);
                let mid_d = surface.distance(origin + dir * mid_t);
                if mid_d.abs() < 1e-4 {
                    return Some(origin + dir * mid_t);
                }
                if lo_d.signum() != mid_d.signum() {
                    hi_t = mid_t;
                    hi_d = mid_d;
                } else {
                    lo_t = mid_t;
                    lo_d = mid_d;
                }
            }
            let t_hit = if lo_d.abs() < hi_d.abs() { lo_t } else { hi_t };
            return Some(origin + dir * t_hit);
        }
        prev_t = t;
        prev_d = d;
    }
    None
}

fn build_oml_matched_start_profile(
    surface: &dyn Sdf,
    base_section: &dyn Section2D,
    mouth_center: Vec3,
    lateral: Vec3,
    upward: Vec3,
    face_clearance: f32,
    samples: usize,
) -> Arc<dyn Section2D> {
    let lat = lateral.normalize_or_zero();
    let up = upward.normalize_or_zero();
    if let Some((half_w, half_h, _radius)) = rounded_rect_params(base_section) {
        let n = samples.max(33) | 1;
        let mut floor_pts = Vec::with_capacity(n);
        for i in 0..n {
            let u = i as f32 / (n - 1) as f32;
            let x = -half_w + u * (half_w * 2.0);
            let probe = mouth_center + lat * x;
            let surf_point = sample_surface_along_axis(surface, probe, up, 250.0)
                .unwrap_or_else(|| {
                    project_point_to_surface_with_offset_and_normal(surface, probe, 0.0).0
                });
            let target = surf_point + up * face_clearance.max(0.0);
            let target_y = (target - mouth_center).dot(up);
            floor_pts.push(Vec2::new(x, target_y));
        }

        let top_y = half_h;
        let mut pts = Vec::with_capacity(n + 4);
        pts.push(Vec2::new(-half_w, top_y));
        pts.push(Vec2::new(half_w, top_y));
        pts.push(Vec2::new(half_w, floor_pts[n - 1].y));
        for p in floor_pts.iter().rev() {
            pts.push(*p);
        }
        pts.push(Vec2::new(-half_w, top_y));
        return Arc::new(PolygonSection::new(pts));
    }

    let outline = sample_section_outline(base_section, samples.max(64));
    if outline.len() < 8 || lat.length_squared() < 1e-8 || up.length_squared() < 1e-8 {
        return Arc::new(PolygonSection::new(outline));
    }

    let min_y = outline.iter().map(|p| p.y).fold(f32::INFINITY, f32::min);
    let span_y = outline.iter().map(|p| p.y).fold(f32::NEG_INFINITY, f32::max) - min_y;
    let blend_band = (span_y * 0.18).max(2.0);

    let pts = outline.into_iter().map(|p| {
        if p.y > min_y + blend_band {
            return p;
        }
        let probe = mouth_center + lat * p.x;
        let (surf_point, normal) = project_point_to_surface_with_offset_and_normal(surface, probe, 0.0);
        let target = surf_point + normal * face_clearance.max(0.0);
        let target_y = (target - mouth_center).dot(up);
        let alpha = ((p.y - min_y) / blend_band).clamp(0.0, 1.0);
        Vec2::new(p.x, target_y * (1.0 - alpha) + p.y * alpha)
    }).collect();
    Arc::new(PolygonSection::new(pts))
}

fn build_oml_matched_start_profile_directional(
    surface: &dyn Sdf,
    base_section: &dyn Section2D,
    mouth_center: Vec3,
    lateral: Vec3,
    upward: Vec3,
    inward_world: Vec3,
    face_clearance: f32,
    samples: usize,
) -> Arc<dyn Section2D> {
    let lat = lateral.normalize_or_zero();
    let up = upward.normalize_or_zero();
    let inward_w = inward_world.normalize_or_zero();
    if lat.length_squared() < 1e-8 || up.length_squared() < 1e-8 || inward_w.length_squared() < 1e-8 {
        return build_oml_matched_start_profile(
            surface,
            base_section,
            mouth_center,
            lateral,
            upward,
            face_clearance,
            samples,
        );
    }

    let inward_2d = Vec2::new(inward_w.dot(lat), inward_w.dot(up)).normalize_or_zero();
    if inward_2d.length_squared() < 1e-8 {
        return build_oml_matched_start_profile(
            surface,
            base_section,
            mouth_center,
            lateral,
            upward,
            face_clearance,
            samples,
        );
    }
    let ortho_2d = Vec2::new(-inward_2d.y, inward_2d.x);

    let outline = sample_section_outline(base_section, samples.max(64));
    if outline.len() < 8 {
        return Arc::new(PolygonSection::new(outline));
    }

    let max_side = outline
        .iter()
        .map(|p| p.dot(inward_2d))
        .fold(f32::NEG_INFINITY, f32::max);
    let min_side = outline
        .iter()
        .map(|p| p.dot(inward_2d))
        .fold(f32::INFINITY, f32::min);
    let side_span = (max_side - min_side).max(1.0);
    let blend_band = (side_span * 0.18).max(2.0);
    let min_ortho = outline
        .iter()
        .map(|p| p.dot(ortho_2d))
        .fold(f32::INFINITY, f32::min);
    let max_ortho = outline
        .iter()
        .map(|p| p.dot(ortho_2d))
        .fold(f32::NEG_INFINITY, f32::max);
    let ortho_mid = 0.5 * (min_ortho + max_ortho);
    let ortho_half = (0.5 * (max_ortho - min_ortho)).max(1e-4);

    let pts = outline.into_iter().map(|p| {
        let side = p.dot(inward_2d);
        if side < max_side - blend_band {
            return p;
        }

        let ortho = p.dot(ortho_2d);
        let probe = mouth_center
            + lat * (ortho_2d.x * ortho)
            + up * (ortho_2d.y * ortho);
        let surf_point = sample_surface_along_axis(surface, probe, -inward_w, 250.0)
            .unwrap_or_else(|| project_point_to_surface_with_offset_and_normal(surface, probe, 0.0).0);
        let target = surf_point - inward_w * face_clearance.max(0.0);
        let target_local = Vec2::new(
            (target - mouth_center).dot(lat),
            (target - mouth_center).dot(up),
        );
        let target_side = target_local.dot(inward_2d);
        let alpha = ((max_side - side) / blend_band).clamp(0.0, 1.0);
        let ortho_norm = ((ortho - ortho_mid).abs() / ortho_half).clamp(0.0, 1.0);
        let edge_falloff = 1.0 - smoothstep(((ortho_norm - 0.10) / 0.18).clamp(0.0, 1.0));
        let solved_side = target_side * ((1.0 - alpha) * edge_falloff) + side * (1.0 - (1.0 - alpha) * edge_falloff);
        ortho_2d * ortho + inward_2d * solved_side
    }).collect();

    Arc::new(PolygonSection::new(pts))
}

fn build_matched_inner_start_profile(
    matched_outer: &dyn Section2D,
    outer_section: &dyn Section2D,
    inner_section: &dyn Section2D,
    samples: usize,
) -> Arc<dyn Section2D> {
    let wall_thickness = infer_section_wall_thickness(outer_section, inner_section, samples);
    if wall_thickness > 1e-4 {
        return Arc::new(OffsetSection::new(
            matched_outer.lerp_to(matched_outer, 0.0),
            wall_thickness,
        ));
    }

    if let (Some((outer_half_w, outer_half_h, _)), Some((inner_half_w, inner_half_h, inner_radius))) =
        (rounded_rect_params(outer_section), rounded_rect_params(inner_section))
    {
        if let Some(lip) = matched_outer.as_any().downcast_ref::<RoundedRectLipProfile>() {
            let n = samples.max(17) | 1;
            let mut floor_pts = Vec::with_capacity(n);
            let bottom_thickness = (outer_half_h - inner_half_h).max(0.5);
            let x_scale = if inner_half_w.abs() > 1e-6 {
                outer_half_w / inner_half_w
            } else {
                1.0
            };
            for i in 0..n {
                let u = i as f32 / (n - 1) as f32;
                let x = -inner_half_w + u * (inner_half_w * 2.0);
                let outer_x = (x * x_scale).clamp(-outer_half_w, outer_half_w);
                let y = lip.floor_y(outer_x) + bottom_thickness;
                floor_pts.push(Vec2::new(x, y));
            }
            return Arc::new(RoundedRectLipProfile {
                half_w: inner_half_w,
                half_h: inner_half_h,
                radius: inner_radius.min(inner_half_w).min(inner_half_h),
                floor_pts,
            });
        }
    }

    let n = samples.max(64);
    let matched_outer_pts = sample_section_outline(matched_outer, n);
    let outer_ref_pts = sample_section_outline(outer_section, matched_outer_pts.len().max(n));
    let inner_ref_pts = sample_section_outline(inner_section, outer_ref_pts.len());

    let pts = matched_outer_pts.into_iter()
        .zip(outer_ref_pts.into_iter().zip(inner_ref_pts))
        .map(|(matched, (outer_ref, inner_ref))| {
            let outer_len = outer_ref.length();
            let inner_len = inner_ref.length();
            if outer_len < 1e-6 {
                matched
            } else {
                matched * (inner_len / outer_len)
            }
        })
        .collect();
    Arc::new(PolygonSection::new(pts))
}

fn infer_section_wall_thickness(
    outer_section: &dyn Section2D,
    inner_section: &dyn Section2D,
    samples: usize,
) -> f32 {
    let outer_pts = sample_section_outline(outer_section, samples.max(64));
    let inner_pts = sample_section_outline(inner_section, samples.max(64));
    if outer_pts.is_empty() || inner_pts.is_empty() {
        return 0.0;
    }

    let outer_min_x = outer_pts.iter().map(|p| p.x).fold(f32::INFINITY, f32::min);
    let outer_max_x = outer_pts.iter().map(|p| p.x).fold(f32::NEG_INFINITY, f32::max);
    let outer_min_y = outer_pts.iter().map(|p| p.y).fold(f32::INFINITY, f32::min);
    let outer_max_y = outer_pts.iter().map(|p| p.y).fold(f32::NEG_INFINITY, f32::max);

    let inner_min_x = inner_pts.iter().map(|p| p.x).fold(f32::INFINITY, f32::min);
    let inner_max_x = inner_pts.iter().map(|p| p.x).fold(f32::NEG_INFINITY, f32::max);
    let inner_min_y = inner_pts.iter().map(|p| p.y).fold(f32::INFINITY, f32::min);
    let inner_max_y = inner_pts.iter().map(|p| p.y).fold(f32::NEG_INFINITY, f32::max);

    let tx = ((outer_max_x - outer_min_x) - (inner_max_x - inner_min_x)) * 0.5;
    let ty = ((outer_max_y - outer_min_y) - (inner_max_y - inner_min_y)) * 0.5;

    let mut candidates = Vec::new();
    if tx.is_finite() && tx > 0.0 { candidates.push(tx); }
    if ty.is_finite() && ty > 0.0 { candidates.push(ty); }
    if candidates.is_empty() {
        0.0
    } else {
        candidates.into_iter().fold(f32::INFINITY, f32::min)
    }
}

fn infer_duct_wall_thickness(
    outer_start: &dyn Section2D,
    inner_start: &dyn Section2D,
    outer_end: &dyn Section2D,
    inner_end: &dyn Section2D,
    samples: usize,
) -> f32 {
    let start_t = infer_section_wall_thickness(outer_start, inner_start, samples);
    let end_t = infer_section_wall_thickness(outer_end, inner_end, samples);
    match (start_t > 1e-4, end_t > 1e-4) {
        (true, true) => start_t.min(end_t),
        (true, false) => start_t,
        (false, true) => end_t,
        (false, false) => 0.0,
    }
}

#[allow(dead_code)]
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

pub struct ConformalProfileDuctParts {
    pub outer_body: Arc<dyn Sdf>,
    pub duct_void: Arc<dyn Sdf>,
    pub duct_shell: Arc<dyn Sdf>,
    pub outer_start_profile: Arc<dyn Section2D>,
    pub inner_start_profile: Arc<dyn Section2D>,
    pub mouth_center: Vec3,
}

pub struct DualConformalProfileDuctParts {
    pub outer_body: Arc<dyn Sdf>,
    pub duct_void: Arc<dyn Sdf>,
    pub duct_shell: Arc<dyn Sdf>,
    pub left_outer_start_profile: Arc<dyn Section2D>,
    pub right_outer_start_profile: Arc<dyn Section2D>,
    pub left_inner_start_profile: Arc<dyn Section2D>,
    pub right_inner_start_profile: Arc<dyn Section2D>,
    pub left_mouth_center: Vec3,
    pub right_mouth_center: Vec3,
}

pub fn conformal_rounded_rect_section(
    surface: Arc<dyn Sdf>,
    x: f32,
    center_y: f32,
    search_z: f32,
    width: f32,
    height: f32,
    radius: f32,
    face_clearance: f32,
    samples: usize,
) -> (Arc<dyn Section2D>, f32) {
    let half_h = (height * 0.5).max(1e-4);
    let probe = Vec3::new(x, center_y, search_z);
    let (surface_point, surface_normal) =
        project_point_to_surface_with_offset_and_normal(surface.as_ref(), probe, 0.0);
    let mouth_center =
        surface_point + surface_normal * (half_h + face_clearance.max(0.0) + 0.25);
    let section = build_oml_matched_start_profile(
        surface.as_ref(),
        &RoundedRectProfile::new(width, height, radius),
        mouth_center,
        Vec3::Y,
        Vec3::Z,
        face_clearance,
        samples.max(32),
    );
    (section, mouth_center.z)
}

pub fn conformal_profile_section(
    surface: Arc<dyn Sdf>,
    base_section: Arc<dyn Section2D>,
    anchor: Vec3,
    flow_dir: Vec3,
    face_clearance: f32,
    samples: usize,
) -> (Arc<dyn Section2D>, Vec3) {
    let (surface_point, surface_normal) =
        project_point_to_surface_with_offset_and_normal(surface.as_ref(), anchor, 0.0);
    let mut tangent =
        (flow_dir.normalize_or_zero() - surface_normal * flow_dir.normalize_or_zero().dot(surface_normal))
            .normalize_or_zero();
    if tangent.length_squared() < 1e-8 {
        tangent = Vec3::X;
    }
    let mut lateral = surface_normal.cross(tangent).normalize_or_zero();
    let mut upward = tangent.cross(lateral).normalize_or_zero();
    if lateral.length_squared() < 1e-8 || upward.length_squared() < 1e-8 {
        let (_, fallback_lateral, fallback_up) = make_frame(tangent);
        lateral = fallback_lateral;
        upward = fallback_up;
    }
    let lower = section_lower_extent(base_section.as_ref(), samples.max(32));
    let center = surface_point + upward * (lower + face_clearance.max(0.0) + 0.25);
    let section = build_oml_matched_start_profile(
        surface.as_ref(),
        base_section.as_ref(),
        center,
        lateral,
        upward,
        face_clearance,
        samples.max(32),
    );
    (section, center)
}

fn project_point_to_surface_same_x(surface: &dyn Sdf, x: f32, guess_y: f32, guess_z: f32) -> (Vec3, Vec3) {
    let mut p = Vec3::new(x, guess_y, guess_z);
    for _ in 0..48 {
        let d = surface.distance(p);
        if d.abs() < 1e-4 {
            break;
        }
        let g = sdf_gradient(surface, p);
        let gyz = Vec3::new(0.0, g.y, g.z);
        if gyz.length_squared() < 1e-10 {
            break;
        }
        p -= gyz.normalize() * d;
        p.x = x;
    }
    let mut n = sdf_gradient(surface, p).normalize_or_zero();
    if n.length_squared() < 1e-10 {
        n = Vec3::Z;
    }
    (p, n)
}

pub fn conformal_profile_section_at_x(
    surface: Arc<dyn Sdf>,
    base_section: Arc<dyn Section2D>,
    x: f32,
    guess_y: f32,
    guess_z: f32,
    flow_dir: Vec3,
    face_clearance: f32,
    samples: usize,
) -> (Arc<dyn Section2D>, Vec3) {
    let (surface_point, surface_normal) =
        project_point_to_surface_same_x(surface.as_ref(), x, guess_y, guess_z);
    let flow = flow_dir.normalize_or_zero();
    let mut tangent = (flow - surface_normal * flow.dot(surface_normal)).normalize_or_zero();
    if tangent.length_squared() < 1e-8 {
        tangent = Vec3::X;
    }
    let reference_up = if tangent.dot(Vec3::Z).abs() < 0.95 { Vec3::Z } else { Vec3::Y };
    let mut lateral = reference_up.cross(tangent).normalize_or_zero();
    let mut upward = tangent.cross(lateral).normalize_or_zero();
    if lateral.length_squared() < 1e-8 || upward.length_squared() < 1e-8 {
        let (_, fallback_lateral, fallback_up) = make_frame(tangent);
        lateral = fallback_lateral;
        upward = fallback_up;
    }
    let lower = section_lower_extent(base_section.as_ref(), samples.max(32));
    let inward_world = -surface_normal;
    let inward_along_up = inward_world.dot(upward);
    let center = surface_point - inward_world * (lower + face_clearance.max(0.0) + 0.25);
    let section = if inward_along_up.abs() > 0.75 {
        build_oml_matched_start_profile(
            surface.as_ref(),
            base_section.as_ref(),
            center,
            lateral,
            upward,
            face_clearance,
            samples.max(32),
        )
    } else {
        build_oml_matched_start_profile_directional(
            surface.as_ref(),
            base_section.as_ref(),
            center,
            lateral,
            upward,
            inward_world,
            face_clearance,
            samples.max(32),
        )
    };
    (section, center)
}

struct TranslatedPath {
    inner: Arc<dyn SweepPath>,
    offset: Vec3,
}

impl TranslatedPath {
    fn new(inner: Arc<dyn SweepPath>, offset: Vec3) -> Self {
        Self { inner, offset }
    }
}

impl SweepPath for TranslatedPath {
    fn evaluate(&self, t: f32) -> Vec3 {
        self.inner.evaluate(t) + self.offset
    }

    fn tangent(&self, t: f32) -> Vec3 {
        self.inner.tangent(t)
    }

    fn arc_length(&self) -> f32 {
        self.inner.arc_length()
    }
}

struct MirroredYPath {
    inner: Arc<dyn SweepPath>,
}

impl MirroredYPath {
    fn new(inner: Arc<dyn SweepPath>) -> Self {
        Self { inner }
    }
}

impl SweepPath for MirroredYPath {
    fn evaluate(&self, t: f32) -> Vec3 {
        let p = self.inner.evaluate(t);
        Vec3::new(p.x, -p.y, p.z)
    }

    fn tangent(&self, t: f32) -> Vec3 {
        let v = self.inner.tangent(t);
        Vec3::new(v.x, -v.y, v.z).normalize_or_zero()
    }

    fn arc_length(&self) -> f32 {
        self.inner.arc_length()
    }
}

pub fn build_conformal_profile_duct_at_x(
    surface: Arc<dyn Sdf>,
    outer_base_start: Arc<dyn Section2D>,
    inner_base_start: Arc<dyn Section2D>,
    x: f32,
    guess_y: f32,
    guess_z: f32,
    flow_dir: Vec3,
    face_clearance: f32,
    duct_path: Arc<dyn SweepPath>,
    outer_end: Arc<dyn Section2D>,
    inner_end: Arc<dyn Section2D>,
    start_extension: f32,
    end_extension: f32,
    morph_start: f32,
    morph_end: f32,
    samples: usize,
) -> ConformalProfileDuctParts {
    let sample_count = samples.max(16);
    let wall_thickness = infer_duct_wall_thickness(
        outer_base_start.as_ref(),
        inner_base_start.as_ref(),
        outer_end.as_ref(),
        inner_end.as_ref(),
        sample_count,
    );
    let (outer_start_profile, mouth_center) = conformal_profile_section_at_x(
        Arc::clone(&surface),
        Arc::clone(&outer_base_start),
        x,
        guess_y,
        guess_z,
        flow_dir,
        face_clearance,
        sample_count,
    );
    let inner_start_profile = if wall_thickness > 1e-4 {
        Arc::new(OffsetSection::new(
            Arc::clone(&outer_start_profile),
            wall_thickness,
        )) as Arc<dyn Section2D>
    } else {
        build_matched_inner_start_profile(
            outer_start_profile.as_ref(),
            outer_base_start.as_ref(),
            inner_base_start.as_ref(),
            sample_count,
        )
    };

    let path_offset = mouth_center - duct_path.evaluate(0.0);
    let shifted_path: Arc<dyn SweepPath> = Arc::new(TranslatedPath::new(duct_path, path_offset));
    let outer_body: Arc<dyn Sdf> = Arc::new(FixedProfileDuct::with_schedule(
        Arc::clone(&shifted_path),
        Arc::clone(&outer_start_profile),
        Arc::clone(&outer_end),
        morph_start,
        morph_end,
        sample_count,
    ));
    let duct_void: Arc<dyn Sdf> = if wall_thickness > 1e-4 {
        let extended_outer_body: Arc<dyn Sdf> = Arc::new(FixedProfileDuct::with_schedule(
            Arc::new(ExtendedPath::new(
                Arc::clone(&shifted_path),
                start_extension,
                end_extension,
            )),
            Arc::clone(&outer_start_profile),
            Arc::clone(&outer_end),
            morph_start,
            morph_end,
            sample_count,
        ));
        Arc::new(Offset::new(extended_outer_body, -wall_thickness))
    } else {
        let inner_path: Arc<dyn SweepPath> =
            Arc::new(ExtendedPath::new(Arc::clone(&shifted_path), start_extension, end_extension));
        Arc::new(FixedProfileDuct::with_schedule(
            inner_path,
            Arc::clone(&inner_start_profile),
            inner_end,
            morph_start,
            morph_end,
            sample_count,
        ))
    };
    let duct_shell: Arc<dyn Sdf> = Arc::new(Subtract::new(
        Arc::clone(&outer_body),
        Arc::clone(&duct_void),
    ));

    ConformalProfileDuctParts {
        outer_body,
        duct_void,
        duct_shell,
        outer_start_profile,
        inner_start_profile,
        mouth_center,
    }
}

#[allow(clippy::too_many_arguments)]
pub fn build_dual_conformal_profile_duct_at_x(
    surface: Arc<dyn Sdf>,
    outer_base_start: Arc<dyn Section2D>,
    inner_base_start: Arc<dyn Section2D>,
    left_x: f32,
    left_guess_y: f32,
    left_guess_z: f32,
    right_x: f32,
    right_guess_y: f32,
    right_guess_z: f32,
    flow_dir: Vec3,
    left_path: Arc<dyn SweepPath>,
    right_path: Arc<dyn SweepPath>,
    outer_end: Arc<dyn Section2D>,
    inner_end: Arc<dyn Section2D>,
    face_clearance: f32,
    start_extension: f32,
    end_extension: f32,
    morph_start: f32,
    morph_end: f32,
    samples: usize,
) -> DualConformalProfileDuctParts {
    let left = build_conformal_profile_duct_at_x(
        Arc::clone(&surface),
        Arc::clone(&outer_base_start),
        Arc::clone(&inner_base_start),
        left_x,
        left_guess_y,
        left_guess_z,
        flow_dir,
        face_clearance,
        left_path,
        Arc::clone(&outer_end),
        Arc::clone(&inner_end),
        start_extension,
        end_extension,
        morph_start,
        morph_end,
        samples,
    );
    let right = build_conformal_profile_duct_at_x(
        Arc::clone(&surface),
        outer_base_start,
        inner_base_start,
        right_x,
        right_guess_y,
        right_guess_z,
        flow_dir,
        face_clearance,
        right_path,
        outer_end,
        inner_end,
        start_extension,
        end_extension,
        morph_start,
        morph_end,
        samples,
    );

    let outer_body: Arc<dyn Sdf> = Arc::new(Union::new(
        Arc::clone(&left.outer_body),
        Arc::clone(&right.outer_body),
    ));
    let duct_void: Arc<dyn Sdf> = Arc::new(Union::new(
        Arc::clone(&left.duct_void),
        Arc::clone(&right.duct_void),
    ));
    let duct_shell: Arc<dyn Sdf> = Arc::new(Subtract::new(
        Arc::clone(&outer_body),
        Arc::clone(&duct_void),
    ));

    DualConformalProfileDuctParts {
        outer_body,
        duct_void,
        duct_shell,
        left_outer_start_profile: left.outer_start_profile,
        right_outer_start_profile: right.outer_start_profile,
        left_inner_start_profile: left.inner_start_profile,
        right_inner_start_profile: right.inner_start_profile,
        left_mouth_center: left.mouth_center,
        right_mouth_center: right.mouth_center,
    }
}

#[allow(clippy::too_many_arguments)]
pub fn build_mirrored_dual_conformal_profile_duct_at_x(
    surface: Arc<dyn Sdf>,
    outer_base_start: Arc<dyn Section2D>,
    inner_base_start: Arc<dyn Section2D>,
    x: f32,
    guess_y: f32,
    guess_z: f32,
    flow_dir: Vec3,
    left_path: Arc<dyn SweepPath>,
    outer_end: Arc<dyn Section2D>,
    inner_end: Arc<dyn Section2D>,
    face_clearance: f32,
    start_extension: f32,
    end_extension: f32,
    morph_start: f32,
    morph_end: f32,
    samples: usize,
) -> DualConformalProfileDuctParts {
    let right_path: Arc<dyn SweepPath> = Arc::new(MirroredYPath::new(Arc::clone(&left_path)));
    build_dual_conformal_profile_duct_at_x(
        surface,
        outer_base_start,
        inner_base_start,
        x,
        guess_y,
        guess_z,
        x,
        -guess_y,
        guess_z,
        flow_dir,
        left_path,
        right_path,
        outer_end,
        inner_end,
        face_clearance,
        start_extension,
        end_extension,
        morph_start,
        morph_end,
        samples,
    )
}

fn symmetrize_about_y(body: Arc<dyn Sdf>, y_center: f32) -> Arc<dyn Sdf> {
    let shifted: Arc<dyn Sdf> =
        Arc::new(Translate::new(Arc::clone(&body), Vec3::new(0.0, -y_center, 0.0)));
    let mirrored: Arc<dyn Sdf> = Arc::new(Mirror::new(shifted, Vec3::Y));
    let mirrored_back: Arc<dyn Sdf> =
        Arc::new(Translate::new(mirrored, Vec3::new(0.0, y_center, 0.0)));
    Arc::new(Union::new(body, mirrored_back))
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
    face_clearance: f32,
    inlet_open_extension: f32,
    outlet_open_extension: f32,
    samples: usize,
) -> ConformalProfileInletParts {
    let sample_count = samples.max(8);
    let y_center = if guide_points.is_empty() {
        0.0
    } else {
        guide_points.iter().map(|p| p.y).sum::<f32>() / guide_points.len() as f32
    };
    let centered_about_y = !guide_points.is_empty()
        && guide_points.iter().all(|p| (p.y - y_center).abs() < 1e-4)
        && (duct_path.evaluate(0.0).y - y_center).abs() < 1e-4
        && (duct_path.evaluate(1.0).y - y_center).abs() < 1e-4;
    let guide_path = PolylinePath::new(guide_points);
    let outer_start: Arc<dyn Section2D> = Arc::new(TransposedSection::new(outer_start));
    let outer_end: Arc<dyn Section2D> = Arc::new(TransposedSection::new(outer_end));
    let inner_start: Arc<dyn Section2D> = Arc::new(TransposedSection::new(inner_start));
    let inner_end: Arc<dyn Section2D> = Arc::new(TransposedSection::new(inner_end));

    let outer_start_lower = section_lower_extent(outer_start.as_ref(), sample_count);

    let mut conformal_points = Vec::with_capacity(sample_count);
    let mut fairing_frames = Vec::with_capacity(sample_count);
    for i in 0..sample_count {
        let u = i as f32 / (sample_count - 1) as f32;
        let guide = guide_path.evaluate(u);
        let tangent = guide_path.tangent(u).normalize_or_zero();
        let (offset_point, normal) =
            project_point_to_surface_with_offset_and_normal(surface.as_ref(), guide, surface_offset);

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

        conformal_points.push(offset_point);
        fairing_frames.push(FrameSample {
            t: u,
            normal: lateral,
            binormal: upward,
        });
    }

    let mouth_point = duct_path.evaluate(0.0);
    let (mouth_offset_point, mouth_normal) =
        project_point_to_surface_with_offset_and_normal(surface.as_ref(), mouth_point, 0.0);
    let lip_profile_dims = rounded_rect_params(outer_start.as_ref());
    let mouth_target_center = if let Some((_, half_h, _)) = lip_profile_dims {
        mouth_offset_point + mouth_normal * (half_h + face_clearance.max(0.0) + 0.25)
    } else {
        mouth_offset_point + mouth_normal * (outer_start_lower + face_clearance.max(0.0) + 0.25)
    };
    let mouth_adjust = mouth_target_center - mouth_point;

    let mouth_tangent = duct_path.tangent(0.0).normalize_or_zero();
    let mut mouth_up = (mouth_normal - mouth_tangent * mouth_tangent.dot(mouth_normal)).normalize_or_zero();
    if mouth_up.length_squared() < 1e-8 {
        let (_, _, fallback_up) = make_frame(mouth_tangent);
        mouth_up = fallback_up;
    }
    let mut mouth_lateral = mouth_up.cross(mouth_tangent).normalize_or_zero();
    if mouth_lateral.length_squared() < 1e-8 {
        let (_, fallback_lateral, fallback_up) = make_frame(mouth_tangent);
        mouth_lateral = fallback_lateral;
        mouth_up = fallback_up;
    } else {
        mouth_up = mouth_tangent.cross(mouth_lateral).normalize_or_zero();
    }

    let matched_outer_start = build_oml_matched_start_profile(
        surface.as_ref(),
        outer_start.as_ref(),
        mouth_target_center,
        mouth_lateral,
        mouth_up,
        face_clearance,
        sample_count,
    );
    let matched_inner_start = build_matched_inner_start_profile(
        matched_outer_start.as_ref(),
        outer_start.as_ref(),
        inner_start.as_ref(),
        sample_count,
    );

    let shift_falloff_end = 0.5_f32;
    let mut adjusted_duct_points = Vec::with_capacity(sample_count);
    for i in 0..sample_count {
        let u = i as f32 / (sample_count - 1) as f32;
        let base_point = duct_path.evaluate(u);
        let weight = if u >= shift_falloff_end {
            0.0
        } else {
            1.0 - smoothstep(u / shift_falloff_end)
        };
        adjusted_duct_points.push(base_point + mouth_adjust * weight);
    }
    let adjusted_duct_path: Arc<dyn SweepPath> = Arc::new(PolylinePath::new(adjusted_duct_points.clone()));
    let mouth_transition_t = 0.12_f32;
    let mouth_transition_point = adjusted_duct_path.evaluate(mouth_transition_t);
    let mouth_transition_index =
        ((sample_count as f32 - 1.0) * mouth_transition_t).floor() as usize;
    let mut main_points = Vec::with_capacity(adjusted_duct_points.len() - mouth_transition_index + 1);
    main_points.push(mouth_transition_point);
    main_points.extend(
        adjusted_duct_points
            .iter()
            .skip((mouth_transition_index + 1).min(adjusted_duct_points.len()))
            .copied(),
    );
    if main_points.len() < 2 {
        main_points.push(adjusted_duct_points[adjusted_duct_points.len() - 1]);
    }
    let main_path: Arc<dyn SweepPath> = Arc::new(PolylinePath::new(main_points));

    let mouth_outer_proto: Arc<dyn Sdf> = Arc::new(StraightProfilePatch::new(
        adjusted_duct_points[0],
        mouth_transition_point,
        Arc::clone(&matched_outer_start),
        Arc::clone(&outer_start),
    ));
    let main_outer_proto: Arc<dyn Sdf> = Arc::new(ProfileDuct::new(
        Arc::clone(&main_path),
        Arc::clone(&outer_start),
        Arc::clone(&outer_end),
        sample_count,
    ));
    let outer_body_proto: Arc<dyn Sdf> = Arc::new(Union::new(
        Arc::clone(&mouth_outer_proto),
        main_outer_proto,
    ));
    let mouth_inner_proto: Arc<dyn Sdf> = Arc::new(StraightProfilePatch::new(
        adjusted_duct_points[0],
        mouth_transition_point,
        Arc::clone(&matched_inner_start),
        Arc::clone(&inner_start),
    ));
    let main_inner_proto: Arc<dyn Sdf> = Arc::new(ProfileDuct::new(
        Arc::clone(&main_path),
        Arc::clone(&inner_start),
        Arc::clone(&inner_end),
        sample_count,
    ));
    let duct_void_proto: Arc<dyn Sdf> = Arc::new(Union::new(mouth_inner_proto, main_inner_proto));
    let internal_shell_proto: Arc<dyn Sdf> = Arc::new(Subtract::new(
        Arc::clone(&outer_body_proto),
        Arc::clone(&duct_void_proto),
    ));
    let outer_fairing: Arc<dyn Sdf> = Arc::new(Subtract::new(
        Arc::clone(&mouth_outer_proto),
        Arc::clone(&surface),
    ));
    let internal_shell: Arc<dyn Sdf> = Arc::new(Intersect::new(
        internal_shell_proto,
        Arc::clone(&surface),
    ));

    let extended_duct_path: Arc<dyn SweepPath> = Arc::new(ExtendedPath::new(
        Arc::clone(&main_path),
        inlet_open_extension.max(0.0),
        outlet_open_extension.max(0.0),
    ));
    let extended_main_void: Arc<dyn Sdf> = Arc::new(ProfileDuct::new(
        extended_duct_path,
        Arc::clone(&inner_start),
        Arc::clone(&inner_end),
        sample_count,
    ));
    let duct_void: Arc<dyn Sdf> = Arc::new(Union::new(duct_void_proto, extended_main_void));

    let outer_fairing = if centered_about_y {
        symmetrize_about_y(outer_fairing, y_center)
    } else {
        outer_fairing
    };
    let duct_void = if centered_about_y {
        symmetrize_about_y(duct_void, y_center)
    } else {
        duct_void
    };
    let internal_shell = if centered_about_y {
        symmetrize_about_y(internal_shell, y_center)
    } else {
        internal_shell
    };

    ConformalProfileInletParts { outer_fairing, duct_void, internal_shell }
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
