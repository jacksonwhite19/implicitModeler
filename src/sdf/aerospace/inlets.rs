// Aerodynamic inlet geometry: NACA flush inlets, EDF buried inlets, inlet lips,
// EDF duct system, and exhaust nozzles.

use glam::Vec3;
use crate::sdf::Sdf;

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
}
