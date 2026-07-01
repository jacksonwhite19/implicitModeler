// SDF boolean operations

use crate::sdf::Sdf;
use crate::sdf::conditioning::{
    BoundQuality, GeometryEdit, SdfBounds, SdfNodeMetadata, combine_bound_quality,
    union_metadata_bounds,
};
use glam::Vec3;
use std::sync::Arc;

/// Union of two SDFs
pub struct Union {
    pub a: Arc<dyn Sdf>,
    pub b: Arc<dyn Sdf>,
}

impl Union {
    pub fn new(a: Arc<dyn Sdf>, b: Arc<dyn Sdf>) -> Self {
        Self { a, b }
    }
}

fn composed_boolean_metadata(
    node_kind: &str,
    dependencies: Vec<(&str, SdfNodeMetadata)>,
    expansion: f32,
) -> SdfNodeMetadata {
    let mut support_bounds =
        union_metadata_bounds(dependencies.iter().map(|(_, metadata)| metadata));
    if expansion > 0.0 {
        support_bounds = support_bounds.map(|bounds| bounds.expanded(expansion));
    }
    let has_unbounded_dependency = dependencies
        .iter()
        .any(|(_, metadata)| metadata.support_bounds.is_none());
    let bound_quality = if support_bounds.is_some() && !has_unbounded_dependency {
        combine_bound_quality(dependencies.iter().map(|(_, metadata)| metadata))
            .conservative_after_operation()
    } else {
        BoundQuality::Unknown
    };
    let mut metadata = SdfNodeMetadata::new(node_kind);
    metadata.set_support_bounds(support_bounds, bound_quality);
    for (role, dependency) in dependencies {
        metadata = metadata.with_dependency(role, dependency);
    }
    metadata
        .with_child_interface_radius(expansion)
        .inherit_dependency_feature_ids()
}

impl Sdf for Union {
    fn distance(&self, point: Vec3) -> f32 {
        crate::sdf::sdf_profile_node_visit();
        self.a.distance(point).min(self.b.distance(point))
    }

    fn metadata(&self) -> SdfNodeMetadata {
        composed_boolean_metadata(
            "union",
            vec![("a", self.a.metadata()), ("b", self.b.metadata())],
            0.0,
        )
    }
}

/// Subtract b from a
pub struct Subtract {
    pub a: Arc<dyn Sdf>,
    pub b: Arc<dyn Sdf>,
}

impl Subtract {
    pub fn new(a: Arc<dyn Sdf>, b: Arc<dyn Sdf>) -> Self {
        Self { a, b }
    }
}

impl Sdf for Subtract {
    fn distance(&self, point: Vec3) -> f32 {
        crate::sdf::sdf_profile_node_visit();
        self.a.distance(point).max(-self.b.distance(point))
    }

    fn metadata(&self) -> SdfNodeMetadata {
        composed_boolean_metadata(
            "subtract",
            vec![("a", self.a.metadata()), ("b", self.b.metadata())],
            0.0,
        )
    }
}

/// Intersection of two SDFs
pub struct Intersect {
    pub a: Arc<dyn Sdf>,
    pub b: Arc<dyn Sdf>,
}

impl Intersect {
    pub fn new(a: Arc<dyn Sdf>, b: Arc<dyn Sdf>) -> Self {
        Self { a, b }
    }
}

impl Sdf for Intersect {
    fn distance(&self, point: Vec3) -> f32 {
        crate::sdf::sdf_profile_node_visit();
        self.a.distance(point).max(self.b.distance(point))
    }

    fn metadata(&self) -> SdfNodeMetadata {
        composed_boolean_metadata(
            "intersect",
            vec![("a", self.a.metadata()), ("b", self.b.metadata())],
            0.0,
        )
    }
}

/// Smooth union of two SDFs with polynomial blending
pub struct SmoothUnion {
    pub a: Arc<dyn Sdf>,
    pub b: Arc<dyn Sdf>,
    pub smoothness: f32,
}

impl SmoothUnion {
    pub fn new(a: Arc<dyn Sdf>, b: Arc<dyn Sdf>, smoothness: f32) -> Self {
        Self { a, b, smoothness }
    }

    #[allow(dead_code)]
    pub fn edit_for_smoothness_change(
        &self,
        feature_id: impl Into<String>,
        interaction_bounds: SdfBounds,
        new_smoothness: f32,
        halo: f32,
    ) -> GeometryEdit {
        GeometryEdit::blend_radius_changed(
            feature_id,
            interaction_bounds,
            self.smoothness,
            new_smoothness,
            halo,
        )
    }

    // Polynomial smooth minimum (IQ formula: mix(b,a,h) - k*h*(1-h))
    fn smin(a: f32, b: f32, k: f32) -> f32 {
        let h = (0.5 + 0.5 * (b - a) / k).clamp(0.0, 1.0);
        b * (1.0 - h) + a * h - k * h * (1.0 - h)
    }
}

impl Sdf for SmoothUnion {
    fn distance(&self, point: Vec3) -> f32 {
        crate::sdf::sdf_profile_node_visit();
        let d1 = self.a.distance(point);
        let d2 = self.b.distance(point);
        Self::smin(d1, d2, self.smoothness)
    }

    fn metadata(&self) -> SdfNodeMetadata {
        composed_boolean_metadata(
            "smooth_union",
            vec![("a", self.a.metadata()), ("b", self.b.metadata())],
            self.smoothness.abs(),
        )
        .with_f32_parameters([self.smoothness])
    }
}

/// Smooth intersection of two SDFs with polynomial blending.
/// Uses IQ's polynomial smooth maximum: smax(a, b, k).
pub struct SmoothIntersect {
    pub a: Arc<dyn Sdf>,
    pub b: Arc<dyn Sdf>,
    pub k: f32,
}

impl SmoothIntersect {
    pub fn new(a: Arc<dyn Sdf>, b: Arc<dyn Sdf>, k: f32) -> Self {
        Self { a, b, k }
    }

    #[allow(dead_code)]
    pub fn edit_for_smoothness_change(
        &self,
        feature_id: impl Into<String>,
        interaction_bounds: SdfBounds,
        new_smoothness: f32,
        halo: f32,
    ) -> GeometryEdit {
        GeometryEdit::blend_radius_changed(
            feature_id,
            interaction_bounds,
            self.k,
            new_smoothness,
            halo,
        )
    }

    // Polynomial smooth maximum (IQ formula: -smin(-a, -b, k))
    fn smax(a: f32, b: f32, k: f32) -> f32 {
        let h = (0.5 - 0.5 * (b - a) / k).clamp(0.0, 1.0);
        b * (1.0 - h) + a * h + k * h * (1.0 - h)
    }
}

impl Sdf for SmoothIntersect {
    fn distance(&self, point: Vec3) -> f32 {
        crate::sdf::sdf_profile_node_visit();
        let d1 = self.a.distance(point);
        let d2 = self.b.distance(point);
        Self::smax(d1, d2, self.k)
    }

    fn metadata(&self) -> SdfNodeMetadata {
        composed_boolean_metadata(
            "smooth_intersect",
            vec![("a", self.a.metadata()), ("b", self.b.metadata())],
            self.k.abs(),
        )
        .with_f32_parameters([self.k])
    }
}

/// Smooth subtraction — removes `tool` from `base` with a smooth chamfer.
/// Uses Inigo Quilez's opSmoothSubtraction formula.
/// k controls the blend radius (larger = softer transition).
pub struct SmoothSubtract {
    pub base: Arc<dyn Sdf>,
    pub tool: Arc<dyn Sdf>,
    pub k: f32,
}

impl SmoothSubtract {
    pub fn new(base: Arc<dyn Sdf>, tool: Arc<dyn Sdf>, k: f32) -> Self {
        Self { base, tool, k }
    }

    #[allow(dead_code)]
    pub fn edit_for_smoothness_change(
        &self,
        feature_id: impl Into<String>,
        interaction_bounds: SdfBounds,
        new_smoothness: f32,
        halo: f32,
    ) -> GeometryEdit {
        GeometryEdit::blend_radius_changed(
            feature_id,
            interaction_bounds,
            self.k,
            new_smoothness,
            halo,
        )
    }
}

impl Sdf for SmoothSubtract {
    fn distance(&self, point: Vec3) -> f32 {
        crate::sdf::sdf_profile_node_visit();
        let d1 = self.tool.distance(point);
        let d2 = self.base.distance(point);
        let h = (0.5 - 0.5 * (d2 + d1) / self.k).clamp(0.0, 1.0);
        // mix(d2, -d1, h) + k*h*(1-h)
        d2 + (-d1 - d2) * h + self.k * h * (1.0 - h)
    }

    fn metadata(&self) -> SdfNodeMetadata {
        composed_boolean_metadata(
            "smooth_subtract",
            vec![
                ("base", self.base.metadata()),
                ("tool", self.tool.metadata()),
            ],
            self.k.abs(),
        )
        .with_f32_parameters([self.k])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::conditioning::{DirtyRegionSource, FeatureTaggedSdf, GeometryEditKind};
    use crate::sdf::primitives::Sphere;
    use crate::sdf::transforms::Translate;

    #[test]
    fn test_union() {
        // Two non-overlapping spheres
        let sphere1 = Arc::new(Sphere::new(2.0));
        let sphere2 = Arc::new(Sphere::new(2.0));

        // We can't directly translate yet, so we'll just verify the union logic
        // Point at origin should be inside sphere1 (distance = -2)
        let union = Union::new(sphere1.clone(), sphere2.clone());

        let dist_origin = union.distance(Vec3::new(0.0, 0.0, 0.0));
        assert!(
            dist_origin < 0.0,
            "Point at origin should be inside at least one sphere"
        );

        // Point far away should be outside both
        let dist_far = union.distance(Vec3::new(10.0, 10.0, 10.0));
        assert!(dist_far > 0.0, "Point far away should be outside");
    }

    #[test]
    fn test_subtract() {
        use crate::sdf::primitives::SdfBox;

        // Box with a sphere subtracted
        let sdf_box = Arc::new(SdfBox::new(Vec3::new(5.0, 5.0, 5.0)));
        let sphere = Arc::new(Sphere::new(3.0));

        let subtract = Subtract::new(sdf_box, sphere);

        // Point at origin should be in the hole (positive distance, since sphere is removed)
        let dist_origin = subtract.distance(Vec3::new(0.0, 0.0, 0.0));
        assert!(
            dist_origin > 0.0,
            "Point at origin should be in the removed sphere (hole)"
        );

        // Point in a corner of the box should still be inside
        let dist_corner = subtract.distance(Vec3::new(4.0, 4.0, 4.0));
        assert!(
            dist_corner < 0.0,
            "Point in corner should still be inside the box"
        );
    }

    #[test]
    fn test_intersect() {
        // Intersect two overlapping spheres
        let sphere1 = Arc::new(Sphere::new(3.0));
        let sphere2 = Arc::new(Sphere::new(3.0));

        let intersect = Intersect::new(sphere1, sphere2);

        // Point at origin should be inside both (distance < 0)
        let dist_origin = intersect.distance(Vec3::new(0.0, 0.0, 0.0));
        assert!(
            dist_origin < 0.0,
            "Point at origin should be inside intersection"
        );

        // Point far away should be outside
        let dist_far = intersect.distance(Vec3::new(10.0, 0.0, 0.0));
        assert!(dist_far > 0.0, "Point far away should be outside");
    }

    #[test]
    fn test_smooth_intersect() {
        // Two spheres of radius 3, both centred at the origin.
        // k = 0.5: the polynomial blend zone width.
        //
        // smax mathematical properties:
        //   1. smax(a,b,k) >= max(a,b) always  →  smooth intersect >= hard intersect
        //   2. Maximum excess above hard max is k/4 (occurs when a == b)
        //   3. When |a-b| >= k the blend is fully clamped and smax == max exactly
        let k = 0.5_f32;
        let max_excess = k / 4.0;

        let make = || {
            let a = Arc::new(Sphere::new(3.0));
            let b = Arc::new(Sphere::new(3.0));
            (a, b)
        };

        // Property 1 & 2: at all sample points, smooth >= hard and excess <= k/4.
        for i in 0..=10 {
            let x = i as f32 * 0.5;
            let p = Vec3::new(x, 0.0, 0.0);
            let (a1, b1) = make();
            let (a2, b2) = make();
            let d_hard = Intersect::new(a1, b1).distance(p);
            let d_smooth = SmoothIntersect::new(a2, b2, k).distance(p);
            assert!(
                d_smooth >= d_hard - 1e-5,
                "smooth >= hard at x={x}: hard={d_hard}, smooth={d_smooth}"
            );
            assert!(
                d_smooth - d_hard <= max_excess + 1e-4,
                "excess <= k/4 at x={x}: excess={}, max_excess={max_excess}",
                d_smooth - d_hard
            );
        }

        // Property 3: at x=10, both sphere distances are ~7; |a-b|=0 < k, so the
        // blend is active. At x=0, both distances are -3 (inside); same story.
        // Check a point where the two input distances differ by more than k so that
        // smooth == hard.  Use two *different* sized spheres for this sub-test.
        {
            let big = Arc::new(Sphere::new(5.0)); // d at x=3: -2
            let small = Arc::new(Sphere::new(1.0)); // d at x=3: +2  → |diff|=4 >> k=0.5
            let big2 = Arc::new(Sphere::new(5.0));
            let small2 = Arc::new(Sphere::new(1.0));
            let p = Vec3::new(3.0, 0.0, 0.0);
            let d_hard = Intersect::new(big, small).distance(p);
            let d_smooth = SmoothIntersect::new(big2, small2, k).distance(p);
            assert!(
                (d_smooth - d_hard).abs() < 1e-4,
                "When inputs differ >> k, smooth should equal hard; hard={d_hard}, smooth={d_smooth}"
            );
        }
    }

    #[test]
    fn smooth_union_emits_blend_radius_dirty_region() {
        let a = Arc::new(Sphere::new(3.0));
        let b = Arc::new(Sphere::new(3.0));
        let blend = SmoothUnion::new(a, b, 2.0);
        let interaction_bounds =
            SdfBounds::new(Vec3::new(-4.0, -3.0, -3.0), Vec3::new(4.0, 3.0, 3.0));

        let edit = blend.edit_for_smoothness_change(
            "wing_root_blend",
            interaction_bounds.clone(),
            4.0,
            1.0,
        );

        assert_eq!(edit.kind, GeometryEditKind::BlendRadius);
        assert_eq!(edit.feature_id.as_deref(), Some("wing_root_blend"));
        assert_eq!(edit.dirty_region.source, DirtyRegionSource::Blend);
        assert_eq!(edit.dirty_region.bounds, interaction_bounds);
        assert_eq!(edit.previous_value, Some(2.0));
        assert_eq!(edit.new_value, Some(4.0));
        assert_eq!(edit.dirty_region.halo, 4.0);
    }

    #[test]
    fn boolean_metadata_exposes_dependencies_and_feature_ids() {
        let left = Arc::new(FeatureTaggedSdf::new(
            "left_wing",
            Arc::new(Sphere::new(2.0)),
        ));
        let right = Arc::new(Translate::new(
            Arc::new(FeatureTaggedSdf::new(
                "right_wing",
                Arc::new(Sphere::new(3.0)),
            )),
            Vec3::new(10.0, 0.0, 0.0),
        ));
        let union = Union::new(left, right);

        let metadata = union.metadata();

        assert_eq!(metadata.node_kind, "union");
        assert_eq!(metadata.dependencies.len(), 2);
        assert_eq!(metadata.dependencies[0].role, "a");
        assert_eq!(metadata.dependencies[1].role, "b");
        assert!(metadata.feature_ids.contains(&"left_wing".to_string()));
        assert!(metadata.feature_ids.contains(&"right_wing".to_string()));
        assert_eq!(
            metadata.support_bounds,
            Some(SdfBounds::new(
                Vec3::new(-2.0, -3.0, -3.0),
                Vec3::new(13.0, 3.0, 3.0)
            ))
        );
    }

    #[test]
    fn smooth_boolean_metadata_expands_support_by_blend_radius() {
        let a = Arc::new(Sphere::new(2.0));
        let b = Arc::new(Translate::new(
            Arc::new(Sphere::new(2.0)),
            Vec3::new(5.0, 0.0, 0.0),
        ));
        let blend = SmoothUnion::new(a, b, 1.5);

        let metadata = blend.metadata();

        assert_eq!(metadata.node_kind, "smooth_union");
        assert_eq!(
            metadata.support_bounds,
            Some(SdfBounds::new(
                Vec3::new(-3.5, -3.5, -3.5),
                Vec3::new(8.5, 3.5, 3.5)
            ))
        );
    }
}
