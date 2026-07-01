// SDF primitive shapes

use crate::sdf::Sdf;
use crate::sdf::conditioning::{
    DirtyRegionSource, GeometryEdit, GeometryEditKind, SdfBounds, SdfNodeMetadata,
};
use glam::{Vec3, Vec3Swizzles};

/// A sphere centered at the origin
pub struct Sphere {
    pub radius: f32,
}

impl Sphere {
    pub fn new(radius: f32) -> Self {
        Self { radius }
    }

    pub fn conditioning_bounds_for_radius(radius: f32) -> SdfBounds {
        SdfBounds::from_center_radius(Vec3::ZERO, radius)
    }

    #[allow(dead_code)]
    pub fn conditioning_bounds(&self) -> SdfBounds {
        Self::conditioning_bounds_for_radius(self.radius)
    }

    #[allow(dead_code)]
    pub fn edit_for_radius_change(
        &self,
        feature_id: impl Into<String>,
        new_radius: f32,
        halo: f32,
    ) -> GeometryEdit {
        let support_radius = self.radius.abs().max(new_radius.abs());
        let support_halo = halo.max((new_radius - self.radius).abs());
        GeometryEdit::parameter_changed(
            GeometryEditKind::Parameter,
            DirtyRegionSource::ParameterEdit,
            feature_id,
            Self::conditioning_bounds_for_radius(support_radius),
            self.radius,
            new_radius,
            support_halo,
        )
    }
}

impl Sdf for Sphere {
    fn distance(&self, point: Vec3) -> f32 {
        point.length() - self.radius
    }

    fn metadata(&self) -> SdfNodeMetadata {
        SdfNodeMetadata::new("sphere")
            .with_support_bounds(self.conditioning_bounds())
            .with_f32_parameters([self.radius])
    }
}

/// A box centered at the origin
pub struct SdfBox {
    pub half_extents: Vec3,
}

impl SdfBox {
    pub fn new(half_extents: Vec3) -> Self {
        Self { half_extents }
    }
}

impl Sdf for SdfBox {
    fn distance(&self, point: Vec3) -> f32 {
        let q = point.abs() - self.half_extents;
        q.max(Vec3::ZERO).length() + q.max_element().min(0.0)
    }

    fn metadata(&self) -> SdfNodeMetadata {
        SdfNodeMetadata::new("box")
            .with_support_bounds(SdfBounds::new(
                -self.half_extents.abs(),
                self.half_extents.abs(),
            ))
            .with_f32_parameters([
                self.half_extents.x,
                self.half_extents.y,
                self.half_extents.z,
            ])
    }
}

/// A cylinder centered at the origin, aligned along Z axis
pub struct Cylinder {
    pub radius: f32,
    pub half_height: f32,
}

impl Cylinder {
    pub fn new(radius: f32, half_height: f32) -> Self {
        Self {
            radius,
            half_height,
        }
    }

    pub fn conditioning_bounds_for_dimensions(radius: f32, half_height: f32) -> SdfBounds {
        let radius = radius.abs();
        let half_height = half_height.abs();
        SdfBounds::new(
            Vec3::new(-radius, -radius, -half_height),
            Vec3::new(radius, radius, half_height),
        )
    }

    #[allow(dead_code)]
    pub fn conditioning_bounds(&self) -> SdfBounds {
        Self::conditioning_bounds_for_dimensions(self.radius, self.half_height)
    }

    #[allow(dead_code)]
    pub fn edit_for_radius_change(
        &self,
        feature_id: impl Into<String>,
        new_radius: f32,
        halo: f32,
    ) -> GeometryEdit {
        let support_radius = self.radius.abs().max(new_radius.abs());
        let support_halo = halo.max((new_radius - self.radius).abs());
        GeometryEdit::parameter_changed(
            GeometryEditKind::Parameter,
            DirtyRegionSource::ParameterEdit,
            feature_id,
            Self::conditioning_bounds_for_dimensions(support_radius, self.half_height),
            self.radius,
            new_radius,
            support_halo,
        )
    }

    #[allow(dead_code)]
    pub fn edit_for_half_height_change(
        &self,
        feature_id: impl Into<String>,
        new_half_height: f32,
        halo: f32,
    ) -> GeometryEdit {
        let support_half_height = self.half_height.abs().max(new_half_height.abs());
        let support_halo = halo.max((new_half_height - self.half_height).abs());
        GeometryEdit::parameter_changed(
            GeometryEditKind::Parameter,
            DirtyRegionSource::ParameterEdit,
            feature_id,
            Self::conditioning_bounds_for_dimensions(self.radius, support_half_height),
            self.half_height,
            new_half_height,
            support_halo,
        )
    }
}

impl Sdf for Cylinder {
    fn distance(&self, point: Vec3) -> f32 {
        let d = Vec3::new(
            point.xy().length() - self.radius,
            0.0,
            point.z.abs() - self.half_height,
        );
        d.max(Vec3::ZERO).length() + d.x.max(d.z).min(0.0)
    }

    fn metadata(&self) -> SdfNodeMetadata {
        SdfNodeMetadata::new("cylinder")
            .with_support_bounds(self.conditioning_bounds())
            .with_f32_parameters([self.radius, self.half_height])
    }
}

/// A torus centered at the origin, lying in the XY plane
pub struct Torus {
    pub major_radius: f32, // Distance from origin to tube center
    pub minor_radius: f32, // Tube radius
}

impl Torus {
    pub fn new(major_radius: f32, minor_radius: f32) -> Self {
        Self {
            major_radius,
            minor_radius,
        }
    }
}

impl Sdf for Torus {
    fn distance(&self, point: Vec3) -> f32 {
        let q = Vec3::new(point.xy().length() - self.major_radius, 0.0, point.z);
        q.xz().length() - self.minor_radius
    }

    fn metadata(&self) -> SdfNodeMetadata {
        let radius = self.major_radius.abs() + self.minor_radius.abs();
        SdfNodeMetadata::new("torus")
            .with_support_bounds(SdfBounds::new(
                Vec3::new(-radius, -radius, -self.minor_radius.abs()),
                Vec3::new(radius, radius, self.minor_radius.abs()),
            ))
            .with_f32_parameters([self.major_radius, self.minor_radius])
    }
}

/// A cone centered at the origin, tip at origin, base in XY plane at -height
pub struct Cone {
    pub radius: f32, // Base radius
    pub height: f32, // Height from tip to base
}

impl Cone {
    pub fn new(radius: f32, height: f32) -> Self {
        Self { radius, height }
    }
}

impl Sdf for Cone {
    fn distance(&self, point: Vec3) -> f32 {
        // Cone along Z axis with tip at origin, base at -height
        let q = point.xy().length();
        let h = -point.z; // Height from tip (tip is at z=0, base at z=-height)

        // Slope of the cone
        let tan_angle = self.radius / self.height;

        // Distance to cone surface
        let cone_dist = (q - h * tan_angle) / (1.0 + tan_angle * tan_angle).sqrt();

        // Clamp to cone height
        if h < 0.0 {
            // Above tip
            (q * q + point.z * point.z).sqrt()
        } else if h > self.height {
            // Below base
            ((q - self.radius).powi(2) + (h - self.height).powi(2)).sqrt()
        } else {
            cone_dist
        }
    }

    fn metadata(&self) -> SdfNodeMetadata {
        let radius = self.radius.abs();
        let height = self.height.abs();
        SdfNodeMetadata::new("cone")
            .with_support_bounds(SdfBounds::new(
                Vec3::new(-radius, -radius, -height),
                Vec3::new(radius, radius, 0.0),
            ))
            .with_f32_parameters([self.radius, self.height])
    }
}

/// A tapered capsule segment between two points with linearly varying radius.
pub struct TaperedCapsule {
    pub a: Vec3,
    pub b: Vec3,
    pub radius_a: f32,
    pub radius_b: f32,
}

impl TaperedCapsule {
    pub fn new(a: Vec3, b: Vec3, radius_a: f32, radius_b: f32) -> Self {
        Self {
            a,
            b,
            radius_a,
            radius_b,
        }
    }
}

impl Sdf for TaperedCapsule {
    fn distance(&self, point: Vec3) -> f32 {
        let ab = self.b - self.a;
        let ab_len2 = ab.length_squared();
        if ab_len2 <= 1e-12 {
            return (point - self.a).length() - self.radius_a.max(self.radius_b);
        }
        let t = ((point - self.a).dot(ab) / ab_len2).clamp(0.0, 1.0);
        let p_on_segment = self.a + ab * t;
        let r = self.radius_a + t * (self.radius_b - self.radius_a);
        (point - p_on_segment).length() - r
    }

    fn metadata(&self) -> SdfNodeMetadata {
        let radius = self.radius_a.abs().max(self.radius_b.abs());
        let min = self.a.min(self.b) - Vec3::splat(radius);
        let max = self.a.max(self.b) + Vec3::splat(radius);
        SdfNodeMetadata::new("tapered_capsule")
            .with_support_bounds(SdfBounds::new(min, max))
            .with_f32_parameters([
                self.a.x,
                self.a.y,
                self.a.z,
                self.b.x,
                self.b.y,
                self.b.z,
                self.radius_a,
                self.radius_b,
            ])
    }
}

/// An infinite plane defined by normal and distance from origin
pub struct Plane {
    pub normal: Vec3,  // Plane normal (should be normalized)
    pub distance: f32, // Signed distance from origin
}

impl Plane {
    pub fn new(normal: Vec3, distance: f32) -> Self {
        Self {
            normal: normal.normalize(),
            distance,
        }
    }
}

impl Sdf for Plane {
    fn distance(&self, point: Vec3) -> f32 {
        point.dot(self.normal) - self.distance
    }

    fn metadata(&self) -> SdfNodeMetadata {
        SdfNodeMetadata::new("plane")
            .with_approximate_bounds()
            .with_f32_parameters([self.normal.x, self.normal.y, self.normal.z, self.distance])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sphere() {
        let sphere = Sphere::new(5.0);

        // Point on surface
        assert!((sphere.distance(Vec3::new(5.0, 0.0, 0.0)) - 0.0).abs() < 0.001);

        // Point at origin (inside)
        assert!((sphere.distance(Vec3::new(0.0, 0.0, 0.0)) - (-5.0)).abs() < 0.001);

        // Point outside
        assert!((sphere.distance(Vec3::new(10.0, 0.0, 0.0)) - 5.0).abs() < 0.001);
    }

    #[test]
    fn sphere_emits_radius_dirty_region() {
        let sphere = Sphere::new(3.0);

        let edit = sphere.edit_for_radius_change("nose_radius", 5.0, 0.5);

        assert_eq!(edit.kind, GeometryEditKind::Parameter);
        assert_eq!(edit.feature_id.as_deref(), Some("nose_radius"));
        assert_eq!(edit.dirty_region.source, DirtyRegionSource::ParameterEdit);
        assert_eq!(
            edit.dirty_region.bounds,
            SdfBounds::new(Vec3::splat(-5.0), Vec3::splat(5.0))
        );
        assert_eq!(edit.previous_value, Some(3.0));
        assert_eq!(edit.new_value, Some(5.0));
        assert_eq!(edit.dirty_region.halo, 2.0);
    }

    #[test]
    fn primitive_metadata_reports_support_bounds() {
        let sphere = Sphere::new(3.0);
        let cylinder = Cylinder::new(2.0, 5.0);
        let torus = Torus::new(6.0, 1.5);
        let plane = Plane::new(Vec3::Z, 0.0);

        assert_eq!(
            sphere.metadata().support_bounds,
            Some(SdfBounds::new(Vec3::splat(-3.0), Vec3::splat(3.0)))
        );
        assert_eq!(
            cylinder.metadata().support_bounds,
            Some(SdfBounds::new(
                Vec3::new(-2.0, -2.0, -5.0),
                Vec3::new(2.0, 2.0, 5.0)
            ))
        );
        assert_eq!(
            torus.metadata().support_bounds,
            Some(SdfBounds::new(
                Vec3::new(-7.5, -7.5, -1.5),
                Vec3::new(7.5, 7.5, 1.5)
            ))
        );
        assert!(plane.metadata().support_bounds.is_none());
        assert!(plane.metadata().bounds_are_approximate);
    }

    #[test]
    fn test_box() {
        let sdf_box = SdfBox::new(Vec3::new(2.0, 3.0, 4.0));

        // Point on surface (x face)
        assert!((sdf_box.distance(Vec3::new(2.0, 0.0, 0.0)) - 0.0).abs() < 0.001);

        // Point at origin (inside)
        let dist_origin = sdf_box.distance(Vec3::new(0.0, 0.0, 0.0));
        assert!(dist_origin < 0.0, "Point at origin should be inside");

        // Point outside
        let dist_outside = sdf_box.distance(Vec3::new(5.0, 0.0, 0.0));
        assert!(
            dist_outside > 0.0,
            "Point far outside should have positive distance"
        );
    }

    #[test]
    fn test_cylinder() {
        let cylinder = Cylinder::new(3.0, 5.0); // radius 3, half-height 5 (total height 10)

        // Point on surface (side)
        let dist_side = cylinder.distance(Vec3::new(3.0, 0.0, 0.0));
        assert!(
            dist_side.abs() < 0.001,
            "Point on side surface should have distance ~0"
        );

        // Point at origin (inside)
        let dist_origin = cylinder.distance(Vec3::new(0.0, 0.0, 0.0));
        assert!(dist_origin < 0.0, "Point at origin should be inside");

        // Point outside radially
        let dist_outside = cylinder.distance(Vec3::new(10.0, 0.0, 0.0));
        assert!(
            dist_outside > 0.0,
            "Point far outside should have positive distance"
        );
    }

    #[test]
    fn cylinder_emits_dimension_dirty_regions() {
        let cylinder = Cylinder::new(3.0, 5.0);

        let radius_edit = cylinder.edit_for_radius_change("motor_pod_radius", 4.0, 0.25);
        let height_edit = cylinder.edit_for_half_height_change("motor_pod_length", 7.0, 0.25);

        assert_eq!(radius_edit.kind, GeometryEditKind::Parameter);
        assert_eq!(radius_edit.feature_id.as_deref(), Some("motor_pod_radius"));
        assert_eq!(
            radius_edit.dirty_region.bounds,
            SdfBounds::new(Vec3::new(-4.0, -4.0, -5.0), Vec3::new(4.0, 4.0, 5.0))
        );
        assert_eq!(radius_edit.dirty_region.halo, 1.0);

        assert_eq!(height_edit.kind, GeometryEditKind::Parameter);
        assert_eq!(height_edit.feature_id.as_deref(), Some("motor_pod_length"));
        assert_eq!(
            height_edit.dirty_region.bounds,
            SdfBounds::new(Vec3::new(-3.0, -3.0, -7.0), Vec3::new(3.0, 3.0, 7.0))
        );
        assert_eq!(height_edit.dirty_region.halo, 2.0);
    }

    #[test]
    fn test_torus() {
        let torus = Torus::new(10.0, 3.0); // major radius 10, minor radius 3

        // Point on surface (outer edge in XY plane)
        let dist_outer = torus.distance(Vec3::new(13.0, 0.0, 0.0));
        assert!(
            dist_outer.abs() < 0.001,
            "Point on outer surface should have distance ~0"
        );

        // Point on inner edge in XY plane
        let dist_inner = torus.distance(Vec3::new(7.0, 0.0, 0.0));
        assert!(
            dist_inner.abs() < 0.001,
            "Point on inner surface should have distance ~0"
        );

        // Point inside tube
        let dist_inside = torus.distance(Vec3::new(10.0, 0.0, 0.0));
        assert!(dist_inside < 0.0, "Point at tube center should be inside");

        // Point far outside
        let dist_outside = torus.distance(Vec3::new(20.0, 0.0, 0.0));
        assert!(
            dist_outside > 0.0,
            "Point far outside should have positive distance"
        );
    }

    #[test]
    fn test_cone() {
        let cone = Cone::new(5.0, 10.0); // base radius 5, height 10

        // Point at tip
        let dist_tip = cone.distance(Vec3::new(0.0, 0.0, 0.0));
        assert!(dist_tip.abs() < 0.5, "Point at tip should have distance ~0");

        // Point inside cone
        let dist_inside = cone.distance(Vec3::new(1.0, 0.0, -3.0));
        assert!(
            dist_inside < 0.0,
            "Point inside cone should have negative distance"
        );

        // Point far outside
        let dist_outside = cone.distance(Vec3::new(10.0, 0.0, 0.0));
        assert!(
            dist_outside > 0.0,
            "Point far outside should have positive distance"
        );
    }

    #[test]
    fn test_plane() {
        let plane = Plane::new(Vec3::new(0.0, 0.0, 1.0), 0.0); // XY plane at Z=0

        // Point on plane
        let dist_on = plane.distance(Vec3::new(5.0, 3.0, 0.0));
        assert!(
            dist_on.abs() < 0.001,
            "Point on plane should have distance ~0"
        );

        // Point above plane
        let dist_above = plane.distance(Vec3::new(0.0, 0.0, 5.0));
        assert!(
            (dist_above - 5.0).abs() < 0.001,
            "Point 5 units above should have distance 5"
        );

        // Point below plane
        let dist_below = plane.distance(Vec3::new(0.0, 0.0, -3.0));
        assert!(
            (dist_below - (-3.0)).abs() < 0.001,
            "Point 3 units below should have distance -3"
        );
    }
}
