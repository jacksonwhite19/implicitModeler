// SDF pattern operations: arrays, mirrors, repetition

use std::sync::Arc;
use glam::{Vec3, Quat};
use crate::sdf::Sdf;

/// N evenly-spaced copies of a shape offset by a step vector each time.
/// linear_array(shape, 5, 10.0, 0.0, 0.0) → 5 copies at x=0, 10, 20, 30, 40
pub struct LinearArray {
    pub child: Arc<dyn Sdf>,
    pub count: usize,
    pub spacing: Vec3,
}

impl LinearArray {
    pub fn new(child: Arc<dyn Sdf>, count: usize, spacing: Vec3) -> Self {
        Self { child, count: count.max(1), spacing }
    }
}

impl Sdf for LinearArray {
    fn distance(&self, point: Vec3) -> f32 {
        let mut min_dist = f32::MAX;
        for i in 0..self.count {
            let offset = self.spacing * (i as f32);
            let d = self.child.distance(point - offset);
            if d < min_dist { min_dist = d; }
        }
        min_dist
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
        Self { child, count: count.max(1), axis: axis.normalize() }
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
            if d < min_dist { min_dist = d; }
        }
        min_dist
    }
}

/// Reflect a shape across an axis-aligned plane.
/// Mirror combines the original and its reflection — result is symmetric.
/// mirror_x → reflect across YZ plane (negate X)
/// mirror_y → reflect across XZ plane (negate Y)
/// mirror_z → reflect across XY plane (negate Z)
pub struct Mirror {
    pub child: Arc<dyn Sdf>,
    pub normal: Vec3,  // Unit normal of the mirror plane (e.g. Vec3::X for YZ plane)
}

impl Mirror {
    pub fn new(child: Arc<dyn Sdf>, normal: Vec3) -> Self {
        Self { child, normal: normal.normalize() }
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
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::primitives::Sphere;

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
    fn test_polar_array_symmetry() {
        let sphere = Arc::new(Sphere { radius: 3.0 });
        // Place sphere offset from axis
        let offset_sphere = Arc::new(crate::sdf::transforms::Translate::new(
            sphere, Vec3::new(10.0, 0.0, 0.0)
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
    fn test_mirror_x_symmetry() {
        let sphere = Arc::new(Sphere { radius: 3.0 });
        // Place sphere at positive X
        let offset = Arc::new(crate::sdf::transforms::Translate::new(
            sphere, Vec3::new(10.0, 0.0, 0.0)
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
}
