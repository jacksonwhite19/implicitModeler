// SDF transform operations

use std::sync::Arc;
use glam::{Vec3, Quat};
use crate::sdf::Sdf;

/// Translate an SDF by an offset
pub struct Translate {
    pub child: Arc<dyn Sdf>,
    pub offset: Vec3,
}

impl Translate {
    pub fn new(child: Arc<dyn Sdf>, offset: Vec3) -> Self {
        Self { child, offset }
    }
}

impl Sdf for Translate {
    fn distance(&self, point: Vec3) -> f32 {
        self.child.distance(point - self.offset)
    }
}

/// Rotate an SDF
pub struct Rotate {
    pub child: Arc<dyn Sdf>,
    pub rotation: Quat,
}

impl Rotate {
    pub fn new(child: Arc<dyn Sdf>, rotation: Quat) -> Self {
        Self { child, rotation }
    }
}

impl Sdf for Rotate {
    fn distance(&self, point: Vec3) -> f32 {
        self.child.distance(self.rotation.inverse() * point)
    }
}

/// Scale an SDF
pub struct Scale {
    pub child: Arc<dyn Sdf>,
    pub scale: Vec3,
}

impl Scale {
    pub fn new(child: Arc<dyn Sdf>, scale: Vec3) -> Self {
        Self { child, scale }
    }
}

impl Sdf for Scale {
    fn distance(&self, point: Vec3) -> f32 {
        let scaled_point = point / self.scale;
        let min_scale = self.scale.x.min(self.scale.y).min(self.scale.z);
        self.child.distance(scaled_point) * min_scale
    }
}

/// Offset (expand/contract) an SDF by a distance
pub struct Offset {
    pub child: Arc<dyn Sdf>,
    pub distance: f32,
}

impl Offset {
    pub fn new(child: Arc<dyn Sdf>, distance: f32) -> Self {
        Self { child, distance }
    }
}

impl Sdf for Offset {
    fn distance(&self, point: Vec3) -> f32 {
        self.child.distance(point) - self.distance
    }
}

/// Twist an SDF around a given axis by an angle proportional to position along that axis.
///
/// A query point at signed distance `d = point · axis` along the axis is rotated by
/// `rate * d` degrees around `axis` before being evaluated against `child`.
///
/// # Approximate SDF
/// The Lipschitz-1 distance guarantee breaks down under strong twist (large `|rate|` or
/// large distances from the axis). Safe for raymarching and marching cubes at moderate
/// deformation; do not rely on the value for precise offset operations.
pub struct Twist {
    pub child: Arc<dyn Sdf>,
    pub axis: Vec3,   // normalised rotation axis
    pub rate: f32,    // degrees per unit length along axis
}

impl Twist {
    /// `axis` is normalised internally; passing a zero vector will produce NaN.
    pub fn new(child: Arc<dyn Sdf>, axis: Vec3, rate: f32) -> Self {
        Self { child, axis: axis.normalize(), rate }
    }
}

impl Sdf for Twist {
    fn distance(&self, point: Vec3) -> f32 {
        // Project point onto axis to obtain the twist parameter.
        let d = point.dot(self.axis);
        let angle_rad = (self.rate * d).to_radians();
        // Un-twist the query point (inverse = negate the angle) before evaluating child.
        let rotation = Quat::from_axis_angle(self.axis, -angle_rad);
        self.child.distance(rotation * point)
    }
}

/// Bend an SDF along a given axis using the standard IQ domain-warp formula.
///
/// The axis defines the straight direction of the unbent shape. A natural bend plane
/// is constructed from `axis` and world-Y (or world-Z when `axis` is near-parallel to Y).
/// At each point the shape is curved in that plane by `curvature * d` radians, where
/// `d = point · axis`.
///
/// # Approximate SDF
/// Same caveat as Twist: Lipschitz-1 is not preserved under high curvature.
/// Safe for raymarching and marching cubes; do not use for precise offsets.
pub struct Bend {
    pub child: Arc<dyn Sdf>,
    pub axis: Vec3,      // normalised bend axis (originally-straight direction)
    pub curvature: f32,  // radians per unit length along axis
}

impl Bend {
    /// `axis` is normalised internally; passing a zero vector will produce NaN.
    pub fn new(child: Arc<dyn Sdf>, axis: Vec3, curvature: f32) -> Self {
        Self { child, axis: axis.normalize(), curvature }
    }
}

impl Sdf for Bend {
    fn distance(&self, point: Vec3) -> f32 {
        let a = self.axis;

        // Build an orthonormal frame: (a, b, c) where b is the "bend into" direction.
        // Choose world-Y as the reference; fall back to world-Z if axis is near-parallel.
        let world_ref = if a.abs().dot(Vec3::Y) < 0.9 { Vec3::Y } else { Vec3::Z };
        let b = a.cross(world_ref).normalize();
        let c = a.cross(b);

        // Decompose point into the local frame.
        let d   = point.dot(a);
        let p_b = point.dot(b);
        let p_c = point.dot(c);

        // IQ bend formula: rotate in the (a, b) plane by curvature * d radians.
        let angle = self.curvature * d;
        let (s, cos_a) = angle.sin_cos();

        let q_a =  cos_a * d   + s     * p_b;
        let q_b = -s     * d   + cos_a * p_b;

        // Reconstruct the warped point in world space and evaluate child.
        self.child.distance(q_a * a + q_b * b + p_c * c)
    }
}

/// Create a shell (hollow) from an SDF
pub struct Shell {
    pub child: Arc<dyn Sdf>,
    pub thickness: f32,
}

impl Shell {
    pub fn new(child: Arc<dyn Sdf>, thickness: f32) -> Self {
        Self { child, thickness }
    }
}

impl Sdf for Shell {
    fn distance(&self, point: Vec3) -> f32 {
        self.child.distance(point).abs() - self.thickness / 2.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::primitives::Sphere;

    #[test]
    fn test_translate() {
        let sphere = Arc::new(Sphere::new(3.0));
        let translated = Translate::new(sphere, Vec3::new(10.0, 0.0, 0.0));

        // The sphere center is now at (10, 0, 0)
        // Point at (10, 0, 0) should be at the center (distance = -3)
        let dist_center = translated.distance(Vec3::new(10.0, 0.0, 0.0));
        assert!((dist_center - (-3.0)).abs() < 0.001, "Center should be inside");

        // Point at (13, 0, 0) should be on the surface (distance = 0)
        let dist_surface = translated.distance(Vec3::new(13.0, 0.0, 0.0));
        assert!(dist_surface.abs() < 0.001, "Point on surface");

        // Point at origin should be outside
        let dist_origin = translated.distance(Vec3::new(0.0, 0.0, 0.0));
        assert!(dist_origin > 0.0, "Origin should be outside translated sphere");
    }

    #[test]
    fn test_rotate() {
        use crate::sdf::primitives::SdfBox;
        use std::f32::consts::PI;

        // Create a box elongated along X axis
        let sdf_box = Arc::new(SdfBox::new(Vec3::new(5.0, 2.0, 1.0)));

        // Rotate 90 degrees around Z axis
        let rotation = Quat::from_rotation_z(PI / 2.0);
        let rotated = Rotate::new(sdf_box, rotation);

        // After rotation, the box is elongated along Y instead of X
        // Point at (0, 5, 0) should be near the surface (was (5, 0, 0) before rotation)
        let dist_y = rotated.distance(Vec3::new(0.0, 5.0, 0.0));
        assert!(dist_y.abs() < 0.01, "Rotated box surface check");

        // Origin should still be inside
        let dist_origin = rotated.distance(Vec3::new(0.0, 0.0, 0.0));
        assert!(dist_origin < 0.0, "Origin should be inside rotated box");
    }

    #[test]
    fn test_twist_zero_rate() {
        // rate=0 → no deformation; should behave identically to the untwisted child.
        let sphere = Arc::new(Sphere::new(3.0));
        let twisted = Twist::new(sphere.clone(), Vec3::Y, 0.0);
        let p = Vec3::new(1.0, 2.0, 1.0);
        let diff = (twisted.distance(p) - sphere.distance(p)).abs();
        assert!(diff < 1e-5, "zero-rate twist should be identity, diff={}", diff);
    }

    #[test]
    fn test_twist_rotates_perpendicular() {
        use crate::sdf::primitives::SdfBox;
        // A box elongated along X with sufficient Y depth to include y=1.
        // At y=0 it is aligned with X; after twisting 90°/unit around Y
        // the box at y=1 should be aligned with Z instead.
        let sdf_box = Arc::new(SdfBox::new(Vec3::new(5.0, 2.0, 0.5)));
        let twisted  = Twist::new(sdf_box, Vec3::Y, 90.0); // 90 deg/unit around Y

        // At y=0 the long axis is X: (4,0,0) should be inside.
        assert!(twisted.distance(Vec3::new(4.0,  0.0, 0.0)) < 0.0,
            "at y=0 long axis is X");
        // At y=1 the box has been twisted -90° back (un-twist) before evaluating.
        // Query (0,1,-4): rotating by -90° around Y maps z=-4 → x=4.
        // Resulting evaluation point (4,1,0) is inside the box (half-extents 5,2,0.5).
        assert!(twisted.distance(Vec3::new(0.0, 1.0, -4.0)) < 0.0,
            "at y=1 long axis should be -Z after 90°/unit twist");
    }

    #[test]
    fn test_bend_zero_curvature() {
        // curvature=0 → no deformation.
        let sphere = Arc::new(Sphere::new(3.0));
        let bent = Bend::new(sphere.clone(), Vec3::X, 0.0);
        let p = Vec3::new(1.5, 0.5, 1.0);
        let diff = (bent.distance(p) - sphere.distance(p)).abs();
        assert!(diff < 1e-5, "zero curvature should be identity, diff={}", diff);
    }

    #[test]
    fn test_bend_curves_shape() {
        use crate::sdf::primitives::Cylinder;
        // A cylinder along Z (half-height large) centred at origin.
        // After bending along X, the cylinder curves in the XY plane.
        // The origin should still be inside regardless of curvature.
        let cyl = Arc::new(Cylinder::new(0.5, 50.0));
        let bent = Bend::new(cyl, Vec3::Z, 0.2); // gentle bend along Z axis
        assert!(bent.distance(Vec3::ZERO) < 0.0, "origin should remain inside bent cylinder");
    }

    #[test]
    fn test_scale() {
        let sphere = Arc::new(Sphere::new(2.0));

        // Scale by 2x in all dimensions
        let scaled = Scale::new(sphere, Vec3::new(2.0, 2.0, 2.0));

        // The scaled sphere has effective radius 4.0
        // Point at (4, 0, 0) should be on the surface
        let dist_surface = scaled.distance(Vec3::new(4.0, 0.0, 0.0));
        assert!(dist_surface.abs() < 0.001, "Scaled sphere surface check");

        // Origin should still be inside
        let dist_origin = scaled.distance(Vec3::new(0.0, 0.0, 0.0));
        assert!(dist_origin < 0.0, "Origin should be inside scaled sphere");
    }
}
