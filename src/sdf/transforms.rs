// SDF transform operations

use crate::sdf::Sdf;
use crate::sdf::conditioning::{
    DirtyRegion, DirtyRegionSource, GeometryEdit, GeometryEditKind, SdfBounds, SdfFeatureRegion,
    SdfNodeMetadata,
};
use glam::{Quat, Vec2, Vec3};
use std::sync::Arc;

/// Translate an SDF by an offset
pub struct Translate {
    pub child: Arc<dyn Sdf>,
    pub offset: Vec3,
}

impl Translate {
    pub fn new(child: Arc<dyn Sdf>, offset: Vec3) -> Self {
        Self { child, offset }
    }

    #[allow(dead_code)]
    pub fn edit_for_offset_change(
        &self,
        feature_id: impl Into<String>,
        new_offset: Vec3,
        halo: f32,
    ) -> Option<GeometryEdit> {
        let child_bounds = self.child.metadata().support_bounds?;
        let feature_id = feature_id.into();
        let previous_bounds = child_bounds.translated(self.offset);
        let next_bounds = child_bounds.translated(new_offset);
        let mut dirty_region = DirtyRegion::new(
            DirtyRegionSource::Transform,
            previous_bounds.union(&next_bounds),
        )
        .with_halo(halo);
        dirty_region.feature_ids.push(feature_id.clone());
        Some(GeometryEdit {
            kind: GeometryEditKind::Transform,
            feature_id: Some(feature_id),
            dirty_region,
            previous_value: None,
            new_value: None,
        })
    }
}

impl Sdf for Translate {
    fn distance(&self, point: Vec3) -> f32 {
        crate::sdf::sdf_profile_node_visit();
        self.child.distance(point - self.offset)
    }

    fn metadata(&self) -> SdfNodeMetadata {
        let child_metadata = self.child.metadata();
        let child_quality = child_metadata.bound_quality;
        let support_bounds = child_metadata
            .support_bounds
            .as_ref()
            .map(|bounds| bounds.translated(self.offset));
        let mut metadata = SdfNodeMetadata::new("translate")
            .with_dependency("child", child_metadata)
            .inherit_dependency_feature_ids()
            .with_f32_parameters([self.offset.x, self.offset.y, self.offset.z]);
        metadata.set_support_bounds(support_bounds, child_quality);
        metadata.translate_feature_regions(self.offset);
        metadata
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
        crate::sdf::sdf_profile_node_visit();
        self.child.distance(self.rotation.inverse() * point)
    }

    fn metadata(&self) -> SdfNodeMetadata {
        let child_metadata = self.child.metadata();
        let child_quality = child_metadata.bound_quality.conservative_after_operation();
        let support_bounds = child_metadata.support_bounds.as_ref().and_then(|bounds| {
            let rotated_corners: Vec<_> = bounds
                .corners()
                .into_iter()
                .map(|corner| self.rotation * corner)
                .collect();
            SdfBounds::from_points(&rotated_corners)
        });
        let mut metadata = SdfNodeMetadata::new("rotate")
            .with_dependency("child", child_metadata)
            .inherit_dependency_feature_ids()
            .with_f32_parameters([
                self.rotation.x,
                self.rotation.y,
                self.rotation.z,
                self.rotation.w,
            ]);
        metadata.set_support_bounds(support_bounds, child_quality);
        metadata.transform_feature_regions(|point| self.rotation * point);
        metadata
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
        crate::sdf::sdf_profile_node_visit();
        let scaled_point = point / self.scale;
        let min_scale = self.scale.x.min(self.scale.y).min(self.scale.z);
        self.child.distance(scaled_point) * min_scale
    }

    fn metadata(&self) -> SdfNodeMetadata {
        let child_metadata = self.child.metadata();
        let child_quality = child_metadata.bound_quality.conservative_after_operation();
        let support_bounds = child_metadata.support_bounds.as_ref().and_then(|bounds| {
            let scaled_corners: Vec<_> = bounds
                .corners()
                .into_iter()
                .map(|corner| corner * self.scale)
                .collect();
            SdfBounds::from_points(&scaled_corners)
        });
        let mut metadata = SdfNodeMetadata::new("scale")
            .with_dependency("child", child_metadata)
            .inherit_dependency_feature_ids()
            .with_f32_parameters([self.scale.x, self.scale.y, self.scale.z]);
        metadata.set_support_bounds(support_bounds, child_quality);
        metadata.transform_feature_regions(|point| point * self.scale);
        let min_scale = self
            .scale
            .x
            .abs()
            .min(self.scale.y.abs())
            .min(self.scale.z.abs());
        metadata.scale_feature_sizes(min_scale);
        metadata
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

    #[allow(dead_code)]
    pub fn edit_for_distance_change(
        &self,
        feature_id: impl Into<String>,
        new_distance: f32,
        halo: f32,
    ) -> Option<GeometryEdit> {
        let affected_bounds = self.child.metadata().support_bounds?;
        Some(GeometryEdit::offset_distance_changed(
            feature_id,
            affected_bounds,
            self.distance,
            new_distance,
            halo,
        ))
    }
}

impl Sdf for Offset {
    fn distance(&self, point: Vec3) -> f32 {
        crate::sdf::sdf_profile_node_visit();
        self.child.distance(point) - self.distance
    }

    fn metadata(&self) -> SdfNodeMetadata {
        let child_metadata = self.child.metadata();
        let child_quality = child_metadata.bound_quality.conservative_after_operation();
        let support_bounds = child_metadata
            .support_bounds
            .as_ref()
            .map(|bounds| bounds.expanded(self.distance.abs()));
        let mut metadata = SdfNodeMetadata::new("offset")
            .with_dependency("child", child_metadata)
            .inherit_dependency_feature_ids()
            .with_f32_parameters([self.distance])
            .with_dirty_expansion(self.distance.abs());
        metadata.set_support_bounds(support_bounds, child_quality);
        metadata
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
    pub axis: Vec3, // normalised rotation axis
    pub rate: f32,  // degrees per unit length along axis
}

impl Twist {
    /// `axis` is normalised internally; passing a zero vector will produce NaN.
    pub fn new(child: Arc<dyn Sdf>, axis: Vec3, rate: f32) -> Self {
        Self {
            child,
            axis: axis.normalize(),
            rate,
        }
    }
}

fn twist_support_bounds(child_bounds: &SdfBounds, axis: Vec3) -> Option<SdfBounds> {
    if !axis.is_finite() || axis.length_squared() < 1e-8 {
        return None;
    }
    let axis = axis.normalize();
    let mut axial_min = f32::INFINITY;
    let mut axial_max = f32::NEG_INFINITY;
    let mut radial_max = 0.0f32;

    for corner in child_bounds.corners() {
        let axial = corner.dot(axis);
        let radial = corner - axis * axial;
        axial_min = axial_min.min(axial);
        axial_max = axial_max.max(axial);
        radial_max = radial_max.max(radial.length());
    }

    if !axial_min.is_finite() || !axial_max.is_finite() || !radial_max.is_finite() {
        return None;
    }

    let axis_abs_sq = axis * axis;
    let radial_extent = Vec3::new(
        (1.0 - axis_abs_sq.x).max(0.0).sqrt(),
        (1.0 - axis_abs_sq.y).max(0.0).sqrt(),
        (1.0 - axis_abs_sq.z).max(0.0).sqrt(),
    ) * radial_max;
    let axial_a = axis * axial_min;
    let axial_b = axis * axial_max;
    Some(SdfBounds::new(
        axial_a.min(axial_b) - radial_extent,
        axial_a.max(axial_b) + radial_extent,
    ))
}

fn remap_feature_regions_conservative(
    metadata: &mut SdfNodeMetadata,
    mut remap_bounds: impl FnMut(&SdfBounds) -> Option<SdfBounds>,
) {
    metadata.feature_regions = metadata
        .feature_regions
        .iter()
        .filter_map(|region| {
            Some(SdfFeatureRegion::new(
                region.feature_id.clone(),
                remap_bounds(&region.bounds)?,
                region.min_feature_size,
            ))
        })
        .collect();
}

impl Sdf for Twist {
    fn distance(&self, point: Vec3) -> f32 {
        crate::sdf::sdf_profile_node_visit();
        // Project point onto axis to obtain the twist parameter.
        let d = point.dot(self.axis);
        let angle_rad = (self.rate * d).to_radians();
        // Un-twist the query point (inverse = negate the angle) before evaluating child.
        let rotation = Quat::from_axis_angle(self.axis, -angle_rad);
        self.child.distance(rotation * point)
    }

    fn metadata(&self) -> SdfNodeMetadata {
        let child_metadata = self.child.metadata();
        let child_quality = child_metadata.bound_quality.conservative_after_operation();
        let support_bounds = child_metadata
            .support_bounds
            .as_ref()
            .and_then(|bounds| twist_support_bounds(bounds, self.axis));
        let mut metadata = SdfNodeMetadata::new("twist")
            .with_dependency("child", child_metadata)
            .inherit_dependency_feature_ids()
            .with_approximate_bounds()
            .with_f32_parameters([self.axis.x, self.axis.y, self.axis.z, self.rate]);
        metadata.set_support_bounds(support_bounds, child_quality);
        remap_feature_regions_conservative(&mut metadata, |bounds| {
            twist_support_bounds(bounds, self.axis)
        });
        metadata
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
    pub axis: Vec3,     // normalised bend axis (originally-straight direction)
    pub curvature: f32, // radians per unit length along axis
}

impl Bend {
    /// `axis` is normalised internally; passing a zero vector will produce NaN.
    pub fn new(child: Arc<dyn Sdf>, axis: Vec3, curvature: f32) -> Self {
        Self {
            child,
            axis: axis.normalize(),
            curvature,
        }
    }
}

fn bend_frame(axis: Vec3) -> Option<(Vec3, Vec3, Vec3)> {
    if !axis.is_finite() || axis.length_squared() < 1e-8 {
        return None;
    }
    let a = axis.normalize();
    let world_ref = if a.abs().dot(Vec3::Y) < 0.9 {
        Vec3::Y
    } else {
        Vec3::Z
    };
    let b = a.cross(world_ref).normalize_or_zero();
    let c = a.cross(b).normalize_or_zero();
    if b.length_squared() < 1e-8 || c.length_squared() < 1e-8 {
        None
    } else {
        Some((a, b, c))
    }
}

fn bend_support_bounds(child_bounds: &SdfBounds, axis: Vec3, curvature: f32) -> Option<SdfBounds> {
    if curvature.abs() < 1e-8 {
        return Some(child_bounds.clone());
    }
    let (a, b, c) = bend_frame(axis)?;
    let mut c_min = f32::INFINITY;
    let mut c_max = f32::NEG_INFINITY;
    let mut bend_plane_radius = 0.0f32;

    for corner in child_bounds.corners() {
        let qa = corner.dot(a);
        let qb = corner.dot(b);
        let qc = corner.dot(c);
        bend_plane_radius = bend_plane_radius.max(Vec2::new(qa, qb).length());
        c_min = c_min.min(qc);
        c_max = c_max.max(qc);
    }

    if !c_min.is_finite() || !c_max.is_finite() || !bend_plane_radius.is_finite() {
        return None;
    }

    let c_abs_sq = c * c;
    let radial_extent = Vec3::new(
        (1.0 - c_abs_sq.x).max(0.0).sqrt(),
        (1.0 - c_abs_sq.y).max(0.0).sqrt(),
        (1.0 - c_abs_sq.z).max(0.0).sqrt(),
    ) * bend_plane_radius;
    let c_a = c * c_min;
    let c_b = c * c_max;
    Some(SdfBounds::new(
        c_a.min(c_b) - radial_extent,
        c_a.max(c_b) + radial_extent,
    ))
}

impl Sdf for Bend {
    fn distance(&self, point: Vec3) -> f32 {
        crate::sdf::sdf_profile_node_visit();
        let a = self.axis;

        // Build an orthonormal frame: (a, b, c) where b is the "bend into" direction.
        // Choose world-Y as the reference; fall back to world-Z if axis is near-parallel.
        let world_ref = if a.abs().dot(Vec3::Y) < 0.9 {
            Vec3::Y
        } else {
            Vec3::Z
        };
        let b = a.cross(world_ref).normalize();
        let c = a.cross(b);

        // Decompose point into the local frame.
        let d = point.dot(a);
        let p_b = point.dot(b);
        let p_c = point.dot(c);

        // IQ bend formula: rotate in the (a, b) plane by curvature * d radians.
        let angle = self.curvature * d;
        let (s, cos_a) = angle.sin_cos();

        let q_a = cos_a * d + s * p_b;
        let q_b = -s * d + cos_a * p_b;

        // Reconstruct the warped point in world space and evaluate child.
        self.child.distance(q_a * a + q_b * b + p_c * c)
    }

    fn metadata(&self) -> SdfNodeMetadata {
        let child_metadata = self.child.metadata();
        let child_quality = child_metadata.bound_quality.conservative_after_operation();
        let support_bounds = child_metadata
            .support_bounds
            .as_ref()
            .and_then(|bounds| bend_support_bounds(bounds, self.axis, self.curvature));
        let mut metadata = SdfNodeMetadata::new("bend")
            .with_dependency("child", child_metadata)
            .inherit_dependency_feature_ids()
            .with_approximate_bounds()
            .with_f32_parameters([self.axis.x, self.axis.y, self.axis.z, self.curvature]);
        metadata.set_support_bounds(support_bounds, child_quality);
        remap_feature_regions_conservative(&mut metadata, |bounds| {
            bend_support_bounds(bounds, self.axis, self.curvature)
        });
        metadata
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

    #[allow(dead_code)]
    pub fn edit_for_thickness_change(
        &self,
        feature_id: impl Into<String>,
        new_thickness: f32,
        halo: f32,
    ) -> Option<GeometryEdit> {
        let affected_bounds = self.child.metadata().support_bounds?;
        Some(GeometryEdit::shell_thickness_changed(
            feature_id,
            affected_bounds,
            self.thickness,
            new_thickness,
            halo,
        ))
    }
}

impl Sdf for Shell {
    fn distance(&self, point: Vec3) -> f32 {
        crate::sdf::sdf_profile_node_visit();
        self.child.distance(point).abs() - self.thickness / 2.0
    }

    fn metadata(&self) -> SdfNodeMetadata {
        let child_metadata = self.child.metadata();
        let child_quality = child_metadata.bound_quality.conservative_after_operation();
        let support_bounds = child_metadata
            .support_bounds
            .as_ref()
            .map(|bounds| bounds.expanded(self.thickness.abs() * 0.5));
        let mut metadata = SdfNodeMetadata::new("shell")
            .with_dependency("child", child_metadata)
            .inherit_dependency_feature_ids()
            .with_f32_parameters([self.thickness])
            .with_min_feature_size(self.thickness.abs())
            .with_dirty_expansion(self.thickness.abs() * 0.5);
        metadata.set_support_bounds(support_bounds, child_quality);
        metadata
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::conditioning::{DirtyRegionSource, FeatureTaggedSdf, GeometryEditKind};
    use crate::sdf::primitives::Sphere;

    struct FeatureRegionSphere;

    impl Sdf for FeatureRegionSphere {
        fn distance(&self, point: Vec3) -> f32 {
            point.length() - 2.0
        }

        fn metadata(&self) -> SdfNodeMetadata {
            SdfNodeMetadata::new("feature_region_sphere")
                .with_support_bounds(SdfBounds::from_center_radius(Vec3::ZERO, 2.0))
                .with_feature_region(
                    "synthetic.leading_edge",
                    SdfBounds::new(Vec3::new(-2.0, -0.25, -0.25), Vec3::new(-1.5, 0.25, 0.25)),
                    0.5,
                )
        }
    }

    #[test]
    fn test_translate() {
        let sphere = Arc::new(Sphere::new(3.0));
        let translated = Translate::new(sphere, Vec3::new(10.0, 0.0, 0.0));

        // The sphere center is now at (10, 0, 0)
        // Point at (10, 0, 0) should be at the center (distance = -3)
        let dist_center = translated.distance(Vec3::new(10.0, 0.0, 0.0));
        assert!(
            (dist_center - (-3.0)).abs() < 0.001,
            "Center should be inside"
        );

        // Point at (13, 0, 0) should be on the surface (distance = 0)
        let dist_surface = translated.distance(Vec3::new(13.0, 0.0, 0.0));
        assert!(dist_surface.abs() < 0.001, "Point on surface");

        // Point at origin should be outside
        let dist_origin = translated.distance(Vec3::new(0.0, 0.0, 0.0));
        assert!(
            dist_origin > 0.0,
            "Origin should be outside translated sphere"
        );
    }

    #[test]
    fn translate_emits_offset_dirty_region() {
        let sphere = Arc::new(Sphere::new(3.0));
        let translated = Translate::new(sphere, Vec3::new(10.0, 0.0, 0.0));

        let edit = translated
            .edit_for_offset_change("servo_mount_position", Vec3::new(12.0, 2.0, 0.0), 0.5)
            .expect("sphere child should expose finite support bounds");

        assert_eq!(edit.kind, GeometryEditKind::Transform);
        assert_eq!(edit.feature_id.as_deref(), Some("servo_mount_position"));
        assert_eq!(edit.dirty_region.source, DirtyRegionSource::Transform);
        assert_eq!(
            edit.dirty_region.bounds,
            SdfBounds::new(Vec3::new(7.0, -3.0, -3.0), Vec3::new(15.0, 5.0, 3.0))
        );
        assert_eq!(edit.dirty_region.halo, 0.5);
        assert_eq!(edit.previous_value, None);
        assert_eq!(edit.new_value, None);
    }

    #[test]
    fn transform_metadata_uses_live_child_bounds_and_features() {
        let child = Arc::new(FeatureTaggedSdf::new(
            "sensor_pod",
            Arc::new(Sphere::new(2.0)),
        ));
        let translated = Translate::new(child, Vec3::new(5.0, 1.0, -2.0));

        let metadata = translated.metadata();

        assert_eq!(metadata.node_kind, "translate");
        assert_eq!(
            metadata.support_bounds,
            Some(SdfBounds::new(
                Vec3::new(3.0, -1.0, -4.0),
                Vec3::new(7.0, 3.0, 0.0)
            ))
        );
        assert_eq!(metadata.feature_ids, vec!["sensor_pod".to_string()]);
        assert_eq!(metadata.dependencies.len(), 1);
        assert_eq!(metadata.dependencies[0].role, "child");
    }

    #[test]
    fn offset_and_shell_metadata_expand_child_support() {
        let sphere = Arc::new(Sphere::new(3.0));
        let offset = Offset::new(sphere.clone(), 1.5);
        let shell = Shell::new(sphere, 2.0);

        assert_eq!(
            offset.metadata().support_bounds,
            Some(SdfBounds::new(Vec3::splat(-4.5), Vec3::splat(4.5)))
        );
        assert_eq!(
            shell.metadata().support_bounds,
            Some(SdfBounds::new(Vec3::splat(-4.0), Vec3::splat(4.0)))
        );
    }

    #[test]
    fn twist_metadata_reports_conservative_axis_bounds() {
        use crate::sdf::primitives::SdfBox;

        let child = Arc::new(SdfBox::new(Vec3::new(5.0, 2.0, 0.5)));
        let twisted = Twist::new(child, Vec3::Y, 90.0);
        let metadata = twisted.metadata();
        let bounds = metadata
            .support_bounds
            .expect("twist should expose conservative child-derived support bounds");

        assert!(bounds.contains(Vec3::new(0.0, 2.0, 5.0)));
        assert!(bounds.contains(Vec3::new(5.0, -2.0, 0.0)));
        assert!(metadata.bounds_are_approximate);
        assert_eq!(metadata.dependencies.len(), 1);
    }

    #[test]
    fn twist_metadata_preserves_conservative_feature_regions() {
        let twisted = Twist::new(Arc::new(FeatureRegionSphere), Vec3::Y, 90.0);
        let metadata = twisted.metadata();

        assert_eq!(metadata.feature_regions.len(), 1);
        assert_eq!(
            metadata.feature_regions[0].feature_id,
            "synthetic.leading_edge"
        );
        assert!(metadata.feature_regions[0].bounds.is_valid());
        assert_eq!(metadata.feature_regions[0].min_feature_size, 0.5);
    }

    #[test]
    fn bend_metadata_reports_conservative_plane_bounds() {
        use crate::sdf::primitives::SdfBox;

        let child = Arc::new(SdfBox::new(Vec3::new(5.0, 2.0, 1.0)));
        let bent = Bend::new(child, Vec3::X, 0.2);
        let metadata = bent.metadata();
        let bounds = metadata
            .support_bounds
            .expect("bend should expose conservative child-derived support bounds");

        assert!(bounds.contains(Vec3::new(5.0, 0.0, 1.0)));
        assert!(bounds.contains(Vec3::new(0.0, 2.0, 5.0)));
        assert!(metadata.bounds_are_approximate);
        assert_eq!(metadata.dependencies.len(), 1);
    }

    #[test]
    fn bend_metadata_preserves_conservative_feature_regions() {
        let bent = Bend::new(Arc::new(FeatureRegionSphere), Vec3::X, 0.2);
        let metadata = bent.metadata();

        assert_eq!(metadata.feature_regions.len(), 1);
        assert_eq!(
            metadata.feature_regions[0].feature_id,
            "synthetic.leading_edge"
        );
        assert!(metadata.feature_regions[0].bounds.is_valid());
        assert_eq!(metadata.feature_regions[0].min_feature_size, 0.5);
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
        assert!(
            diff < 1e-5,
            "zero-rate twist should be identity, diff={}",
            diff
        );
    }

    #[test]
    fn test_twist_rotates_perpendicular() {
        use crate::sdf::primitives::SdfBox;
        // A box elongated along X with sufficient Y depth to include y=1.
        // At y=0 it is aligned with X; after twisting 90°/unit around Y
        // the box at y=1 should be aligned with Z instead.
        let sdf_box = Arc::new(SdfBox::new(Vec3::new(5.0, 2.0, 0.5)));
        let twisted = Twist::new(sdf_box, Vec3::Y, 90.0); // 90 deg/unit around Y

        // At y=0 the long axis is X: (4,0,0) should be inside.
        assert!(
            twisted.distance(Vec3::new(4.0, 0.0, 0.0)) < 0.0,
            "at y=0 long axis is X"
        );
        // At y=1 the box has been twisted -90° back (un-twist) before evaluating.
        // Query (0,1,-4): rotating by -90° around Y maps z=-4 → x=4.
        // Resulting evaluation point (4,1,0) is inside the box (half-extents 5,2,0.5).
        assert!(
            twisted.distance(Vec3::new(0.0, 1.0, -4.0)) < 0.0,
            "at y=1 long axis should be -Z after 90°/unit twist"
        );
    }

    #[test]
    fn test_bend_zero_curvature() {
        // curvature=0 → no deformation.
        let sphere = Arc::new(Sphere::new(3.0));
        let bent = Bend::new(sphere.clone(), Vec3::X, 0.0);
        let p = Vec3::new(1.5, 0.5, 1.0);
        let diff = (bent.distance(p) - sphere.distance(p)).abs();
        assert!(
            diff < 1e-5,
            "zero curvature should be identity, diff={}",
            diff
        );
    }

    #[test]
    fn test_bend_curves_shape() {
        use crate::sdf::primitives::Cylinder;
        // A cylinder along Z (half-height large) centred at origin.
        // After bending along X, the cylinder curves in the XY plane.
        // The origin should still be inside regardless of curvature.
        let cyl = Arc::new(Cylinder::new(0.5, 50.0));
        let bent = Bend::new(cyl, Vec3::Z, 0.2); // gentle bend along Z axis
        assert!(
            bent.distance(Vec3::ZERO) < 0.0,
            "origin should remain inside bent cylinder"
        );
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

    #[test]
    fn offset_emits_distance_dirty_region() {
        let sphere = Arc::new(Sphere::new(3.0));
        let offset = Offset::new(sphere, 1.5);

        let edit = offset
            .edit_for_distance_change("skin_offset", 2.25, 0.5)
            .expect("sphere child should expose finite support bounds");

        assert_eq!(edit.kind, GeometryEditKind::Offset);
        assert_eq!(edit.feature_id.as_deref(), Some("skin_offset"));
        assert_eq!(edit.dirty_region.source, DirtyRegionSource::Offset);
        assert_eq!(
            edit.dirty_region.bounds,
            SdfBounds::new(Vec3::splat(-3.0), Vec3::splat(3.0))
        );
        assert_eq!(edit.previous_value, Some(1.5));
        assert_eq!(edit.new_value, Some(2.25));
        assert_eq!(edit.dirty_region.halo, 2.25);
    }

    #[test]
    fn shell_emits_thickness_dirty_region() {
        let sphere = Arc::new(Sphere::new(3.0));
        let shell = Shell::new(sphere, 2.0);

        let edit = shell
            .edit_for_thickness_change("printed_shell", 4.0, 0.25)
            .expect("sphere child should expose finite support bounds");

        assert_eq!(edit.kind, GeometryEditKind::Shell);
        assert_eq!(edit.feature_id.as_deref(), Some("printed_shell"));
        assert_eq!(edit.dirty_region.source, DirtyRegionSource::Shell);
        assert_eq!(
            edit.dirty_region.bounds,
            SdfBounds::new(Vec3::splat(-3.0), Vec3::splat(3.0))
        );
        assert_eq!(edit.previous_value, Some(2.0));
        assert_eq!(edit.new_value, Some(4.0));
        assert_eq!(edit.dirty_region.halo, 2.0);
    }
}
