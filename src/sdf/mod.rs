// SDF trait and core types

use glam::Vec3;

/// Trait for signed distance field evaluation
pub trait Sdf: Send + Sync {
    /// Returns the signed distance from a point to the surface.
    /// Negative values indicate the point is inside the shape.
    fn distance(&self, point: Vec3) -> f32;
}

pub mod primitives;
pub mod booleans;
pub mod transforms;
pub mod patterns;
pub mod aerospace;
pub mod field;
pub mod profiles;
pub mod spine;
pub mod lattice;

#[cfg(test)]
mod integration_tests {
    use super::*;
    use std::sync::Arc;
    use primitives::{SdfBox, Sphere};
    use booleans::Subtract;
    use transforms::Translate;

    #[test]
    fn test_compound_sdf() {
        // Build a box with a translated sphere subtracted (a box with a hole)
        // Box: 10x10x10 (half-extents 5x5x5)
        // Sphere: radius 3, translated to (3, 0, 0)

        let sdf_box = Arc::new(SdfBox::new(Vec3::new(5.0, 5.0, 5.0)));
        let sphere = Arc::new(Sphere::new(3.0));
        let translated_sphere = Arc::new(Translate::new(sphere, Vec3::new(3.0, 0.0, 0.0)));
        let result = Subtract::new(sdf_box, translated_sphere);

        // Point at (3, 0, 0) - center of the hole
        // Should be inside the removed volume (positive distance)
        let dist_hole_center = result.distance(Vec3::new(3.0, 0.0, 0.0));
        assert!(dist_hole_center > 0.0, "Center of hole should have positive distance");

        // Point at (-4, 0, 0) - inside the box, far from the hole
        // Should be inside the remaining box (negative distance)
        let dist_inside_box = result.distance(Vec3::new(-4.0, 0.0, 0.0));
        assert!(dist_inside_box < 0.0, "Point inside box away from hole should be negative");

        // Point far outside
        let dist_outside = result.distance(Vec3::new(20.0, 20.0, 20.0));
        assert!(dist_outside > 0.0, "Point far outside should be positive");
    }
}
