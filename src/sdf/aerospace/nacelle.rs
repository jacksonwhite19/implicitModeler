// Engine nacelle primitives

use std::sync::Arc;
use glam::Vec3;
use crate::sdf::Sdf;
use crate::sdf::primitives::{Cylinder, Sphere, Cone};
use crate::sdf::booleans::SmoothUnion;
use crate::sdf::transforms::Translate;

/// Create a simple nacelle using composite primitives
///
/// # Arguments
/// * `length` - Total nacelle length
/// * `diameter` - Maximum diameter of nacelle body
/// * `inlet_diameter` - Inlet (front) diameter
/// * `exhaust_diameter` - Exhaust (rear) diameter
///
/// Returns a composite SDF made from cylinders, spheres, and cones with smooth blending
pub fn nacelle_simple(
    length: f32,
    diameter: f32,
    inlet_diameter: f32,
    exhaust_diameter: f32,
) -> Arc<dyn Sdf> {
    let radius = diameter / 2.0;
    let inlet_radius = inlet_diameter / 2.0;
    let exhaust_radius = exhaust_diameter / 2.0;

    // Main cylindrical body
    let body = Arc::new(Cylinder::new(radius, length / 2.0)) as Arc<dyn Sdf>;

    // Inlet: ellipsoidal front using scaled sphere
    let inlet_sphere = Arc::new(Sphere::new(inlet_radius * 1.2)) as Arc<dyn Sdf>;
    let inlet = Arc::new(Translate::new(
        inlet_sphere,
        Vec3::new(-length / 2.0, 0.0, 0.0)
    )) as Arc<dyn Sdf>;

    // Exhaust: tapered cone at rear
    let exhaust_cone = Arc::new(Cone::new(exhaust_radius, length * 0.25)) as Arc<dyn Sdf>;
    let exhaust = Arc::new(Translate::new(
        exhaust_cone,
        Vec3::new(length / 2.0 + length * 0.125, 0.0, 0.0)
    )) as Arc<dyn Sdf>;

    // Smooth blend everything together
    let smoothness = diameter * 0.15;  // 15% of diameter for blend radius

    let nacelle = Arc::new(SmoothUnion::new(body, inlet, smoothness)) as Arc<dyn Sdf>;
    Arc::new(SmoothUnion::new(nacelle, exhaust, smoothness))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_nacelle_creation() {
        let nacelle = nacelle_simple(10.0, 2.0, 1.8, 1.5);

        // Point at center should be inside
        let dist_center = nacelle.distance(Vec3::ZERO);
        assert!(dist_center < 0.0, "Center should be inside nacelle");

        // Point far outside
        let dist_outside = nacelle.distance(Vec3::new(20.0, 20.0, 20.0));
        assert!(dist_outside > 0.0, "Point outside should be positive");
    }

    #[test]
    fn test_nacelle_dimensions() {
        let length = 10.0;
        let diameter = 2.0;
        let nacelle = nacelle_simple(length, diameter, 1.8, 1.5);

        // Verify we can call distance function at various points
        let _ = nacelle.distance(Vec3::ZERO);
        let _ = nacelle.distance(Vec3::new(-length / 2.0, 0.0, 0.0));
        let _ = nacelle.distance(Vec3::new(length / 2.0, 0.0, 0.0));

        // Point far outside should be positive
        let dist_far = nacelle.distance(Vec3::new(50.0, 50.0, 50.0));
        assert!(dist_far > 40.0, "Point far outside should have large positive distance");
    }
}
