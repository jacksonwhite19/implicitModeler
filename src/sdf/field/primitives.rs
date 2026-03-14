// Core field types

use glam::Vec3;
use std::sync::Arc;
use crate::sdf::Sdf;
use super::Field;

/// Constant field - returns the same value everywhere
#[derive(Clone)]
pub struct ConstantField {
    pub value: f32,
}

impl ConstantField {
    pub fn new(value: f32) -> Self {
        Self { value }
    }
}

impl Field for ConstantField {
    fn evaluate(&self, _point: Vec3) -> f32 {
        self.value
    }

    fn bounds(&self) -> Option<(f32, f32)> {
        Some((self.value, self.value))
    }
}

/// SDF field - wraps an SDF and returns its distance value as a field
#[derive(Clone)]
pub struct SdfField {
    pub sdf: Arc<dyn Sdf>,
}

impl SdfField {
    pub fn new(sdf: Arc<dyn Sdf>) -> Self {
        Self { sdf }
    }
}

impl Field for SdfField {
    fn evaluate(&self, point: Vec3) -> f32 {
        self.sdf.distance(point)
    }

    // SDF fields are generally unbounded
    fn bounds(&self) -> Option<(f32, f32)> {
        None
    }
}

/// Position X field - returns the X coordinate
pub struct PositionXField;

impl Field for PositionXField {
    fn evaluate(&self, point: Vec3) -> f32 {
        point.x
    }
}

/// Position Y field - returns the Y coordinate
pub struct PositionYField;

impl Field for PositionYField {
    fn evaluate(&self, point: Vec3) -> f32 {
        point.y
    }
}

/// Position Z field - returns the Z coordinate
pub struct PositionZField;

impl Field for PositionZField {
    fn evaluate(&self, point: Vec3) -> f32 {
        point.z
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::primitives::Sphere;

    #[test]
    fn test_constant_field() {
        let field = ConstantField::new(5.0);

        // Should return same value everywhere
        assert_eq!(field.evaluate(Vec3::ZERO), 5.0);
        assert_eq!(field.evaluate(Vec3::new(10.0, 20.0, 30.0)), 5.0);
        assert_eq!(field.evaluate(Vec3::new(-100.0, 50.0, -25.0)), 5.0);

        // Bounds should be exact
        assert_eq!(field.bounds(), Some((5.0, 5.0)));
    }

    #[test]
    fn test_constant_field_negative() {
        let field = ConstantField::new(-3.5);
        assert_eq!(field.evaluate(Vec3::ZERO), -3.5);
        assert_eq!(field.bounds(), Some((-3.5, -3.5)));
    }

    #[test]
    fn test_sdf_field() {
        let sphere = Arc::new(Sphere::new(10.0));
        let field = SdfField::new(sphere);

        // At origin (center of sphere), distance should be -10
        let dist_center = field.evaluate(Vec3::ZERO);
        assert!((dist_center + 10.0).abs() < 0.01, "Center should be -radius");

        // At radius, distance should be ~0
        let dist_surface = field.evaluate(Vec3::new(10.0, 0.0, 0.0));
        assert!(dist_surface.abs() < 0.01, "Surface should be ~0");

        // Far outside
        let dist_outside = field.evaluate(Vec3::new(20.0, 0.0, 0.0));
        assert!(dist_outside > 0.0, "Outside should be positive");

        // Bounds should be None (unbounded)
        assert_eq!(field.bounds(), None);
    }

    #[test]
    fn test_position_x_field() {
        let field = PositionXField;

        assert_eq!(field.evaluate(Vec3::new(5.0, 0.0, 0.0)), 5.0);
        assert_eq!(field.evaluate(Vec3::new(-3.0, 10.0, 20.0)), -3.0);
        assert_eq!(field.evaluate(Vec3::new(0.0, 0.0, 0.0)), 0.0);
    }

    #[test]
    fn test_position_y_field() {
        let field = PositionYField;

        assert_eq!(field.evaluate(Vec3::new(0.0, 5.0, 0.0)), 5.0);
        assert_eq!(field.evaluate(Vec3::new(10.0, -3.0, 20.0)), -3.0);
        assert_eq!(field.evaluate(Vec3::new(0.0, 0.0, 0.0)), 0.0);
    }

    #[test]
    fn test_position_z_field() {
        let field = PositionZField;

        assert_eq!(field.evaluate(Vec3::new(0.0, 0.0, 5.0)), 5.0);
        assert_eq!(field.evaluate(Vec3::new(10.0, 20.0, -3.0)), -3.0);
        assert_eq!(field.evaluate(Vec3::new(0.0, 0.0, 0.0)), 0.0);
    }

    #[test]
    fn test_position_fields_independence() {
        let fx = PositionXField;
        let fy = PositionYField;
        let fz = PositionZField;

        let point = Vec3::new(1.0, 2.0, 3.0);

        assert_eq!(fx.evaluate(point), 1.0);
        assert_eq!(fy.evaluate(point), 2.0);
        assert_eq!(fz.evaluate(point), 3.0);
    }
}
