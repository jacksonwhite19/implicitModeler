// SDF pattern operations: arrays, mirrors, repetition

use crate::sdf::Sdf;
use crate::sdf::conditioning::{SdfBounds, SdfFeatureRegion, SdfNodeMetadata};
use glam::{Quat, Vec3};
use std::sync::Arc;

/// N evenly-spaced copies of a shape offset by a step vector each time.
/// linear_array(shape, 5, 10.0, 0.0, 0.0) → 5 copies at x=0, 10, 20, 30, 40
pub struct LinearArray {
    pub child: Arc<dyn Sdf>,
    pub count: usize,
    pub spacing: Vec3,
}

impl LinearArray {
    pub fn new(child: Arc<dyn Sdf>, count: usize, spacing: Vec3) -> Self {
        Self {
            child,
            count: count.max(1),
            spacing,
        }
    }
}

impl Sdf for LinearArray {
    fn distance(&self, point: Vec3) -> f32 {
        let mut min_dist = f32::MAX;
        for i in 0..self.count {
            let offset = self.spacing * (i as f32);
            let d = self.child.distance(point - offset);
            if d < min_dist {
                min_dist = d;
            }
        }
        min_dist
    }

    fn metadata(&self) -> SdfNodeMetadata {
        let child_metadata = self.child.metadata();
        let child_quality = child_metadata.bound_quality.conservative_after_operation();
        let child_feature_regions = child_metadata.feature_regions.clone();
        let support_bounds = child_metadata.support_bounds.as_ref().map(|bounds| {
            let mut combined = bounds.clone();
            for i in 1..self.count {
                combined = combined.union(&bounds.translated(self.spacing * i as f32));
            }
            combined
        });
        let mut metadata = SdfNodeMetadata::new("linear_array")
            .with_dependency("child", child_metadata)
            .inherit_dependency_feature_ids();
        metadata.set_support_bounds(support_bounds, child_quality);
        metadata.clear_feature_regions();
        for region in child_feature_regions {
            for i in 0..self.count {
                metadata.add_feature_region(region.translated(self.spacing * i as f32));
            }
        }
        metadata
    }
}

/// N copies of a shape arranged evenly around an axis (default Z).
/// polar_array(shape, 6) → 6 copies at 0°, 60°, 120°, 180°, 240°, 300° around Z
pub struct PolarArray {
    pub child: Arc<dyn Sdf>,
    pub count: usize,
    pub axis: Vec3,
}

impl PolarArray {
    pub fn new(child: Arc<dyn Sdf>, count: usize, axis: Vec3) -> Self {
        Self {
            child,
            count: count.max(1),
            axis: axis.normalize(),
        }
    }
}

impl Sdf for PolarArray {
    fn distance(&self, point: Vec3) -> f32 {
        let angle_step = std::f32::consts::TAU / self.count as f32;
        let mut min_dist = f32::MAX;
        for i in 0..self.count {
            let angle = angle_step * (i as f32);
            // Rotate the query point backwards — equivalent to rotating the child forwards
            let q = Quat::from_axis_angle(self.axis, -angle);
            let rotated_point = q * point;
            let d = self.child.distance(rotated_point);
            if d < min_dist {
                min_dist = d;
            }
        }
        min_dist
    }

    fn metadata(&self) -> SdfNodeMetadata {
        let child_metadata = self.child.metadata();
        let child_quality = child_metadata.bound_quality.conservative_after_operation();
        let child_feature_regions = child_metadata.feature_regions.clone();
        let support_bounds = child_metadata.support_bounds.as_ref().and_then(|bounds| {
            let angle_step = std::f32::consts::TAU / self.count as f32;
            let mut combined: Option<SdfBounds> = None;
            for i in 0..self.count {
                let rotation = Quat::from_axis_angle(self.axis, angle_step * i as f32);
                let rotated_corners: Vec<_> = bounds
                    .corners()
                    .into_iter()
                    .map(|corner| rotation * corner)
                    .collect();
                let copy_bounds = SdfBounds::from_points(&rotated_corners)?;
                combined = Some(match combined {
                    Some(existing) => existing.union(&copy_bounds),
                    None => copy_bounds,
                });
            }
            combined
        });
        let mut metadata = SdfNodeMetadata::new("polar_array")
            .with_dependency("child", child_metadata)
            .inherit_dependency_feature_ids();
        metadata.set_support_bounds(support_bounds, child_quality);
        metadata.clear_feature_regions();
        let angle_step = std::f32::consts::TAU / self.count as f32;
        for region in child_feature_regions {
            for i in 0..self.count {
                let rotation = Quat::from_axis_angle(self.axis, angle_step * i as f32);
                if let Some(rotated_region) = region.transformed_bounds(|point| rotation * point) {
                    metadata.add_feature_region(rotated_region);
                }
            }
        }
        metadata
    }
}

/// Reflect a shape across an axis-aligned plane.
/// Mirror combines the original and its reflection — result is symmetric.
/// mirror_x → reflect across YZ plane (negate X)
/// mirror_y → reflect across XZ plane (negate Y)
/// mirror_z → reflect across XY plane (negate Z)
pub struct Mirror {
    pub child: Arc<dyn Sdf>,
    pub normal: Vec3, // Unit normal of the mirror plane (e.g. Vec3::X for YZ plane)
}

impl Mirror {
    pub fn new(child: Arc<dyn Sdf>, normal: Vec3) -> Self {
        Self {
            child,
            normal: normal.normalize(),
        }
    }
}

impl Sdf for Mirror {
    fn distance(&self, point: Vec3) -> f32 {
        // Reflect point across the plane defined by normal through origin
        let reflected = point - 2.0 * point.dot(self.normal) * self.normal;
        let d_original = self.child.distance(point);
        let d_reflected = self.child.distance(reflected);
        d_original.min(d_reflected)
    }

    fn metadata(&self) -> SdfNodeMetadata {
        let child_metadata = self.child.metadata();
        let child_quality = child_metadata.bound_quality.conservative_after_operation();
        let child_feature_regions = child_metadata.feature_regions.clone();
        let support_bounds = child_metadata.support_bounds.as_ref().and_then(|bounds| {
            let mut points = bounds.corners().to_vec();
            points.extend(
                bounds
                    .corners()
                    .into_iter()
                    .map(|corner| corner - 2.0 * corner.dot(self.normal) * self.normal),
            );
            SdfBounds::from_points(&points)
        });
        let mut metadata = SdfNodeMetadata::new("mirror")
            .with_dependency("child", child_metadata)
            .inherit_dependency_feature_ids();
        metadata.set_support_bounds(support_bounds, child_quality);
        metadata.clear_feature_regions();
        for region in child_feature_regions {
            metadata.add_feature_region(region.clone());
            let reflected_region = region
                .transformed_bounds(|point| point - 2.0 * point.dot(self.normal) * self.normal);
            if let Some(reflected_region) = reflected_region {
                metadata.add_feature_region(SdfFeatureRegion::new(
                    reflected_region.feature_id,
                    reflected_region.bounds,
                    reflected_region.min_feature_size,
                ));
            }
        }
        metadata
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::conditioning::{FeatureTaggedSdf, SdfBounds, SdfNodeMetadata};
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
                    "synthetic.edge",
                    SdfBounds::new(Vec3::new(-2.0, -0.25, -0.25), Vec3::new(-1.5, 0.25, 0.25)),
                    0.5,
                )
        }
    }

    #[test]
    fn test_linear_array_creates_copies() {
        let sphere = Arc::new(Sphere { radius: 3.0 });
        let arr = LinearArray::new(sphere, 3, Vec3::new(10.0, 0.0, 0.0));

        // Point near first copy (x=0) should be inside
        assert!(arr.distance(Vec3::new(0.0, 0.0, 0.0)) < 0.0);
        // Point near second copy (x=10) should be inside
        assert!(arr.distance(Vec3::new(10.0, 0.0, 0.0)) < 0.0);
        // Point near third copy (x=20) should be inside
        assert!(arr.distance(Vec3::new(20.0, 0.0, 0.0)) < 0.0);
        // Point far from all copies should be outside
        assert!(arr.distance(Vec3::new(50.0, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn test_linear_array_count_one() {
        let sphere = Arc::new(Sphere { radius: 5.0 });
        let arr = LinearArray::new(sphere.clone(), 1, Vec3::new(10.0, 0.0, 0.0));
        let single = Sphere { radius: 5.0 };
        // count=1 should behave identically to the child
        assert!((arr.distance(Vec3::ZERO) - single.distance(Vec3::ZERO)).abs() < 1e-5);
    }

    #[test]
    fn linear_array_metadata_unions_copy_bounds() {
        let sphere = Arc::new(FeatureTaggedSdf::new("bolt", Arc::new(Sphere::new(2.0))));
        let arr = LinearArray::new(sphere, 3, Vec3::new(10.0, 0.0, 0.0));

        let metadata = arr.metadata();

        assert_eq!(metadata.node_kind, "linear_array");
        assert_eq!(metadata.feature_ids, vec!["bolt".to_string()]);
        assert_eq!(
            metadata.support_bounds,
            Some(SdfBounds::new(
                Vec3::new(-2.0, -2.0, -2.0),
                Vec3::new(22.0, 2.0, 2.0)
            ))
        );
    }

    #[test]
    fn linear_array_metadata_repeats_feature_regions() {
        let arr = LinearArray::new(Arc::new(FeatureRegionSphere), 3, Vec3::new(10.0, 0.0, 0.0));

        let metadata = arr.metadata();

        assert_eq!(metadata.feature_regions.len(), 3);
        assert_eq!(metadata.feature_regions[0].feature_id, "synthetic.edge");
        assert!(
            metadata.feature_regions[2]
                .bounds
                .contains(Vec3::new(18.0, 0.0, 0.0))
        );
    }

    #[test]
    fn test_polar_array_symmetry() {
        let sphere = Arc::new(Sphere { radius: 3.0 });
        // Place sphere offset from axis
        let offset_sphere = Arc::new(crate::sdf::transforms::Translate::new(
            sphere,
            Vec3::new(10.0, 0.0, 0.0),
        ));
        let arr = PolarArray::new(offset_sphere, 4, Vec3::Z);

        // All 4 cardinal points at radius 10 should be inside a copy
        let r = 10.0_f32;
        assert!(arr.distance(Vec3::new(r, 0.0, 0.0)) < 0.0);
        assert!(arr.distance(Vec3::new(-r, 0.0, 0.0)) < 0.0);
        assert!(arr.distance(Vec3::new(0.0, r, 0.0)) < 0.0);
        assert!(arr.distance(Vec3::new(0.0, -r, 0.0)) < 0.0);
    }

    #[test]
    fn polar_array_metadata_repeats_feature_regions() {
        let arr = PolarArray::new(Arc::new(FeatureRegionSphere), 4, Vec3::Z);

        let metadata = arr.metadata();

        assert_eq!(metadata.feature_regions.len(), 4);
        assert_eq!(metadata.feature_regions[0].feature_id, "synthetic.edge");
        assert!(
            metadata
                .feature_regions
                .iter()
                .any(|region| region.bounds.contains(Vec3::new(0.0, -1.75, 0.0)))
        );
    }

    #[test]
    fn test_mirror_x_symmetry() {
        let sphere = Arc::new(Sphere { radius: 3.0 });
        // Place sphere at positive X
        let offset = Arc::new(crate::sdf::transforms::Translate::new(
            sphere,
            Vec3::new(10.0, 0.0, 0.0),
        ));
        let mirrored = Mirror::new(offset, Vec3::X);

        // Original position should be inside
        assert!(mirrored.distance(Vec3::new(10.0, 0.0, 0.0)) < 0.0);
        // Mirror position (negative X) should also be inside
        assert!(mirrored.distance(Vec3::new(-10.0, 0.0, 0.0)) < 0.0);
        // Point far away should be outside
        assert!(mirrored.distance(Vec3::new(0.0, 50.0, 0.0)) > 0.0);
    }

    #[test]
    fn test_mirror_preserves_original() {
        let sphere = Arc::new(Sphere { radius: 5.0 });
        let mirrored = Mirror::new(sphere, Vec3::Y);
        // Origin is equidistant from both — should be same as sphere distance
        let d = mirrored.distance(Vec3::ZERO);
        assert!((d - (-5.0_f32)).abs() < 1e-4);
    }

    #[test]
    fn mirror_metadata_unions_original_and_reflected_bounds() {
        let sphere = Arc::new(Sphere::new(2.0));
        let offset = Arc::new(crate::sdf::transforms::Translate::new(
            sphere,
            Vec3::new(5.0, 0.0, 0.0),
        ));
        let mirrored = Mirror::new(offset, Vec3::X);

        let metadata = mirrored.metadata();

        assert_eq!(metadata.node_kind, "mirror");
        assert_eq!(
            metadata.support_bounds,
            Some(SdfBounds::new(
                Vec3::new(-7.0, -2.0, -2.0),
                Vec3::new(7.0, 2.0, 2.0)
            ))
        );
    }

    #[test]
    fn mirror_metadata_reflects_feature_regions() {
        let mirrored = Mirror::new(Arc::new(FeatureRegionSphere), Vec3::X);

        let metadata = mirrored.metadata();

        assert_eq!(metadata.feature_regions.len(), 2);
        assert_eq!(metadata.feature_regions[0].feature_id, "synthetic.edge");
        assert!(
            metadata
                .feature_regions
                .iter()
                .any(|region| region.bounds.contains(Vec3::new(2.0, 0.0, 0.0)))
        );
    }
}
