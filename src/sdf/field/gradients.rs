// Gradient and spatial variation fields

use glam::Vec3;
use super::Field;

/// Linear gradient field between two points
/// Interpolates from start_value at start_point to end_value at end_point
pub struct GradientField {
    pub start_point: Vec3,
    pub end_point: Vec3,
    pub start_value: f32,
    pub end_value: f32,
}

impl GradientField {
    pub fn new(start_point: Vec3, end_point: Vec3, start_value: f32, end_value: f32) -> Self {
        Self {
            start_point,
            end_point,
            start_value,
            end_value,
        }
    }
}

impl Field for GradientField {
    fn evaluate(&self, point: Vec3) -> f32 {
        let direction = self.end_point - self.start_point;
        let length = direction.length();

        if length < 1e-6 {
            // Start and end are the same point - return start value
            return self.start_value;
        }

        // Project point onto line segment
        let to_point = point - self.start_point;
        let t = to_point.dot(direction) / (length * length);
        let t_clamped = t.clamp(0.0, 1.0);

        // Linear interpolation
        self.start_value + (self.end_value - self.start_value) * t_clamped
    }

    fn bounds(&self) -> Option<(f32, f32)> {
        Some((
            self.start_value.min(self.end_value),
            self.start_value.max(self.end_value),
        ))
    }
}

/// Radial field - interpolates based on distance from a center point
/// Values interpolate from inner_value at inner_radius to outer_value at outer_radius
pub struct RadialField {
    pub center: Vec3,
    pub inner_radius: f32,
    pub outer_radius: f32,
    pub inner_value: f32,
    pub outer_value: f32,
}

impl RadialField {
    pub fn new(
        center: Vec3,
        inner_radius: f32,
        outer_radius: f32,
        inner_value: f32,
        outer_value: f32,
    ) -> Self {
        Self {
            center,
            inner_radius,
            outer_radius,
            inner_value,
            outer_value,
        }
    }
}

impl Field for RadialField {
    fn evaluate(&self, point: Vec3) -> f32 {
        let dist = (point - self.center).length();

        if dist <= self.inner_radius {
            return self.inner_value;
        }
        if dist >= self.outer_radius {
            return self.outer_value;
        }

        // Linear interpolation between inner and outer
        let t = (dist - self.inner_radius) / (self.outer_radius - self.inner_radius);
        self.inner_value + (self.outer_value - self.inner_value) * t
    }

    fn bounds(&self) -> Option<(f32, f32)> {
        Some((
            self.inner_value.min(self.outer_value),
            self.inner_value.max(self.outer_value),
        ))
    }
}

/// Axial radial field - interpolates based on distance from an axis
/// Like RadialField but distance is measured perpendicular to an axis
pub struct AxialRadialField {
    pub axis_point: Vec3,
    pub axis_direction: Vec3, // Should be normalized
    pub inner_radius: f32,
    pub outer_radius: f32,
    pub inner_value: f32,
    pub outer_value: f32,
}

impl AxialRadialField {
    pub fn new(
        axis_point: Vec3,
        axis_direction: Vec3,
        inner_radius: f32,
        outer_radius: f32,
        inner_value: f32,
        outer_value: f32,
    ) -> Self {
        Self {
            axis_point,
            axis_direction: axis_direction.normalize(),
            inner_radius,
            outer_radius,
            inner_value,
            outer_value,
        }
    }
}

impl Field for AxialRadialField {
    fn evaluate(&self, point: Vec3) -> f32 {
        // Project point onto axis
        let to_point = point - self.axis_point;
        let along_axis = to_point.dot(self.axis_direction);
        let on_axis = self.axis_point + self.axis_direction * along_axis;

        // Distance from axis (perpendicular distance)
        let radial_dist = (point - on_axis).length();

        if radial_dist <= self.inner_radius {
            return self.inner_value;
        }
        if radial_dist >= self.outer_radius {
            return self.outer_value;
        }

        // Linear interpolation
        let t = (radial_dist - self.inner_radius) / (self.outer_radius - self.inner_radius);
        self.inner_value + (self.outer_value - self.inner_value) * t
    }

    fn bounds(&self) -> Option<(f32, f32)> {
        Some((
            self.inner_value.min(self.outer_value),
            self.inner_value.max(self.outer_value),
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gradient_field_basic() {
        let field = GradientField::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(10.0, 0.0, 0.0),
            0.0,
            10.0,
        );

        // At start point
        assert_eq!(field.evaluate(Vec3::new(0.0, 0.0, 0.0)), 0.0);

        // At end point
        assert_eq!(field.evaluate(Vec3::new(10.0, 0.0, 0.0)), 10.0);

        // At midpoint
        let mid_val = field.evaluate(Vec3::new(5.0, 0.0, 0.0));
        assert!((mid_val - 5.0).abs() < 0.01);

        // Bounds
        assert_eq!(field.bounds(), Some((0.0, 10.0)));
    }

    #[test]
    fn test_gradient_field_clamping() {
        let field = GradientField::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(10.0, 0.0, 0.0),
            5.0,
            15.0,
        );

        // Before start - should clamp to start value
        assert_eq!(field.evaluate(Vec3::new(-10.0, 0.0, 0.0)), 5.0);

        // After end - should clamp to end value
        assert_eq!(field.evaluate(Vec3::new(20.0, 0.0, 0.0)), 15.0);
    }

    #[test]
    fn test_gradient_field_perpendicular() {
        let field = GradientField::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(10.0, 0.0, 0.0),
            0.0,
            10.0,
        );

        // Points perpendicular to gradient line should project to nearest point on line
        let val = field.evaluate(Vec3::new(5.0, 100.0, 0.0));
        assert!((val - 5.0).abs() < 0.01, "Perpendicular point should project to midpoint");
    }

    #[test]
    fn test_radial_field_spherical() {
        let field = RadialField::new(
            Vec3::ZERO,
            0.0,
            10.0,
            0.0,
            10.0,
        );

        // At center
        assert_eq!(field.evaluate(Vec3::ZERO), 0.0);

        // At outer radius
        let val_outer = field.evaluate(Vec3::new(10.0, 0.0, 0.0));
        assert!((val_outer - 10.0).abs() < 0.01);

        // At half radius
        let val_half = field.evaluate(Vec3::new(5.0, 0.0, 0.0));
        assert!((val_half - 5.0).abs() < 0.01);

        // Radial symmetry - same distance in any direction
        let val_x = field.evaluate(Vec3::new(5.0, 0.0, 0.0));
        let val_y = field.evaluate(Vec3::new(0.0, 5.0, 0.0));
        let val_z = field.evaluate(Vec3::new(0.0, 0.0, 5.0));
        let val_diag = field.evaluate(Vec3::new(5.0 / 3f32.sqrt(), 5.0 / 3f32.sqrt(), 5.0 / 3f32.sqrt()));

        assert!((val_x - val_y).abs() < 0.01);
        assert!((val_y - val_z).abs() < 0.01);
        assert!((val_z - val_diag).abs() < 0.01);
    }

    #[test]
    fn test_radial_field_with_inner_radius() {
        let field = RadialField::new(
            Vec3::ZERO,
            5.0,  // inner radius
            10.0, // outer radius
            2.0,  // inner value
            8.0,  // outer value
        );

        // Inside inner radius - constant
        assert_eq!(field.evaluate(Vec3::ZERO), 2.0);
        assert_eq!(field.evaluate(Vec3::new(3.0, 0.0, 0.0)), 2.0);

        // At inner radius
        assert_eq!(field.evaluate(Vec3::new(5.0, 0.0, 0.0)), 2.0);

        // Between inner and outer - interpolate
        let val_mid = field.evaluate(Vec3::new(7.5, 0.0, 0.0));
        assert!((val_mid - 5.0).abs() < 0.01);

        // Beyond outer radius - constant
        assert_eq!(field.evaluate(Vec3::new(15.0, 0.0, 0.0)), 8.0);

        // Bounds
        assert_eq!(field.bounds(), Some((2.0, 8.0)));
    }

    #[test]
    fn test_radial_field_offset_center() {
        let field = RadialField::new(
            Vec3::new(10.0, 20.0, 30.0),
            0.0,
            5.0,
            0.0,
            5.0,
        );

        // At center
        assert_eq!(field.evaluate(Vec3::new(10.0, 20.0, 30.0)), 0.0);

        // At outer radius from center
        let val = field.evaluate(Vec3::new(15.0, 20.0, 30.0));
        assert!((val - 5.0).abs() < 0.01);
    }

    #[test]
    fn test_axial_radial_field_basic() {
        // Axis along X, starting at origin
        let field = AxialRadialField::new(
            Vec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0), // X axis
            0.0,
            5.0,
            1.0,
            10.0,
        );

        // On the axis - inner value
        assert_eq!(field.evaluate(Vec3::new(10.0, 0.0, 0.0)), 1.0);
        assert_eq!(field.evaluate(Vec3::new(-10.0, 0.0, 0.0)), 1.0);

        // At outer radius from axis
        let val = field.evaluate(Vec3::new(10.0, 5.0, 0.0));
        assert!((val - 10.0).abs() < 0.01);

        // At mid radius from axis
        let val_mid = field.evaluate(Vec3::new(10.0, 2.5, 0.0));
        assert!((val_mid - 5.5).abs() < 0.01);
    }

    #[test]
    fn test_axial_radial_field_cylindrical_symmetry() {
        let field = AxialRadialField::new(
            Vec3::ZERO,
            Vec3::new(0.0, 0.0, 1.0), // Z axis
            0.0,
            5.0,
            0.0,
            10.0,
        );

        // Same distance from axis at different Z positions should have same value
        let val1 = field.evaluate(Vec3::new(3.0, 0.0, 10.0));
        let val2 = field.evaluate(Vec3::new(3.0, 0.0, -5.0));
        let val3 = field.evaluate(Vec3::new(0.0, 3.0, 0.0));

        assert!((val1 - val2).abs() < 0.01);
        assert!((val2 - val3).abs() < 0.01);

        // All should be at 60% of range since distance is 3.0 out of 5.0
        assert!((val1 - 6.0).abs() < 0.01);
    }

    #[test]
    fn test_axial_radial_field_normalization() {
        // Non-normalized axis direction should be automatically normalized
        let field = AxialRadialField::new(
            Vec3::ZERO,
            Vec3::new(3.0, 4.0, 0.0), // Length 5, not normalized
            0.0,
            10.0,
            0.0,
            1.0,
        );

        // Should still work correctly - axis gets normalized internally
        let val = field.evaluate(Vec3::ZERO);
        assert_eq!(val, 0.0);
    }

    #[test]
    fn test_gradient_field_descending() {
        // Test gradient that goes from high to low
        let field = GradientField::new(
            Vec3::ZERO,
            Vec3::new(10.0, 0.0, 0.0),
            10.0,
            0.0,
        );

        assert_eq!(field.evaluate(Vec3::ZERO), 10.0);
        assert_eq!(field.evaluate(Vec3::new(10.0, 0.0, 0.0)), 0.0);
        let mid_val = field.evaluate(Vec3::new(5.0, 0.0, 0.0));
        assert!((mid_val - 5.0).abs() < 0.01);

        // Bounds should still be (min, max) not (start, end)
        assert_eq!(field.bounds(), Some((0.0, 10.0)));
    }
}
