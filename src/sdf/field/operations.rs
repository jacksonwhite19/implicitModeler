// Field-driven SDF operations

use glam::Vec3;
use std::sync::Arc;
use crate::sdf::Sdf;
use super::Field;

/// Variable thickness offset - offsets an SDF by an amount driven by a field
pub struct OffsetByField {
    pub child: Arc<dyn Sdf>,
    pub field: Arc<dyn Field>,
}

impl OffsetByField {
    pub fn new(child: Arc<dyn Sdf>, field: Arc<dyn Field>) -> Self {
        Self { child, field }
    }
}

impl Sdf for OffsetByField {
    fn distance(&self, point: Vec3) -> f32 {
        let base_distance = self.child.distance(point);
        let offset_amount = self.field.evaluate(point);
        base_distance - offset_amount
    }
}

/// Variable thickness shell - creates a hollow shell with thickness controlled by a field
pub struct ShellWithField {
    pub child: Arc<dyn Sdf>,
    pub thickness_field: Arc<dyn Field>,
}

impl ShellWithField {
    pub fn new(child: Arc<dyn Sdf>, thickness_field: Arc<dyn Field>) -> Self {
        Self {
            child,
            thickness_field,
        }
    }
}

impl Sdf for ShellWithField {
    fn distance(&self, point: Vec3) -> f32 {
        let base_distance = self.child.distance(point).abs();
        let thickness = self.thickness_field.evaluate(point);
        base_distance - thickness / 2.0
    }
}

/// Variable blend radius smooth union - blends two SDFs with smoothness controlled by a field
pub struct BlendByField {
    pub a: Arc<dyn Sdf>,
    pub b: Arc<dyn Sdf>,
    pub smoothness_field: Arc<dyn Field>,
}

impl BlendByField {
    pub fn new(a: Arc<dyn Sdf>, b: Arc<dyn Sdf>, smoothness_field: Arc<dyn Field>) -> Self {
        Self {
            a,
            b,
            smoothness_field,
        }
    }

    // Polynomial smooth minimum (same as SmoothUnion)
    fn smin(a: f32, b: f32, k: f32) -> f32 {
        let h = (0.5 + 0.5 * (b - a) / k).clamp(0.0, 1.0);
        b.min(a) - h * (1.0 - h) * k
    }
}

impl Sdf for BlendByField {
    fn distance(&self, point: Vec3) -> f32 {
        let d1 = self.a.distance(point);
        let d2 = self.b.distance(point);
        let k = self.smoothness_field.evaluate(point);
        Self::smin(d1, d2, k)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::primitives::Sphere;
    use crate::sdf::field::primitives::ConstantField;
    use crate::sdf::field::gradients::RadialField;

    #[test]
    fn test_offset_by_field_constant() {
        // With constant field, should match regular Offset behavior
        let sphere = Arc::new(Sphere::new(10.0));
        let offset_field = Arc::new(ConstantField::new(2.0));
        let offset_sdf = OffsetByField::new(sphere, offset_field);

        // At radius 12, original sphere distance is 2.0, after offset -2.0 = 0.0
        let dist = offset_sdf.distance(Vec3::new(12.0, 0.0, 0.0));
        assert!(dist.abs() < 0.01, "Expected ~0, got {}", dist);

        // At center, original is -10.0, after offset -2.0 = -12.0
        let dist_center = offset_sdf.distance(Vec3::ZERO);
        assert!((dist_center + 12.0).abs() < 0.01);
    }

    #[test]
    fn test_offset_by_field_variable() {
        // Variable offset - thicker at center, thinner at edges
        let sphere = Arc::new(Sphere::new(10.0));
        let offset_field = Arc::new(RadialField::new(
            Vec3::ZERO,
            0.0,
            10.0,
            4.0, // thick at center
            1.0, // thin at edge
        ));
        let offset_sdf = OffsetByField::new(sphere, offset_field);

        // At center: offset is 4.0, sphere distance is -10.0, result is -14.0
        let dist_center = offset_sdf.distance(Vec3::ZERO);
        assert!((dist_center + 14.0).abs() < 0.01);

        // At original surface (r=10): offset is 1.0, sphere distance is 0.0, result is -1.0
        let dist_surface = offset_sdf.distance(Vec3::new(10.0, 0.0, 0.0));
        assert!((dist_surface + 1.0).abs() < 0.01);
    }

    #[test]
    fn test_shell_with_field_constant() {
        // With constant thickness, should match regular Shell behavior
        let sphere = Arc::new(Sphere::new(10.0));
        let thickness_field = Arc::new(ConstantField::new(2.0));
        let shell = ShellWithField::new(sphere, thickness_field);

        // On the original surface (r=10), abs(0) - 1 = -1, inside shell wall
        let dist_surface = shell.distance(Vec3::new(10.0, 0.0, 0.0));
        assert!(dist_surface < 0.0, "Should be inside shell wall");

        // At r=9 (inside by 1): abs(-1) - 1 = 0, on inner surface
        let dist_inner = shell.distance(Vec3::new(9.0, 0.0, 0.0));
        assert!(dist_inner.abs() < 0.01, "Expected ~0, got {}", dist_inner);

        // At r=11 (outside by 1): abs(1) - 1 = 0, on outer surface
        let dist_outer = shell.distance(Vec3::new(11.0, 0.0, 0.0));
        assert!(dist_outer.abs() < 0.01, "Expected ~0, got {}", dist_outer);
    }

    #[test]
    fn test_shell_with_field_variable() {
        // Variable thickness - thick at center, thin at edges
        let sphere = Arc::new(Sphere::new(10.0));
        let thickness_field = Arc::new(RadialField::new(
            Vec3::ZERO,
            0.0,
            10.0,
            4.0, // thick at center
            1.0, // thin at edges
        ));
        let shell = ShellWithField::new(sphere, thickness_field);

        // Near center (r=0): thickness is 4.0, abs(-10) - 2 = 8, outside inner surface
        let dist_center = shell.distance(Vec3::ZERO);
        assert!(dist_center > 0.0);

        // At original surface: thickness varies by position
        // Just verify it produces reasonable values
        let dist_surface = shell.distance(Vec3::new(10.0, 0.0, 0.0));
        assert!(dist_surface.is_finite());
    }

    #[test]
    fn test_blend_by_field_constant() {
        // With constant smoothness, should match SmoothUnion behavior
        let sphere_a = Arc::new(Sphere::new(5.0));
        let sphere_b_sdf = Arc::new(Sphere::new(5.0));
        let sphere_b = Arc::new(crate::sdf::transforms::Translate::new(
            sphere_b_sdf,
            Vec3::new(8.0, 0.0, 0.0),
        ));
        let smoothness_field = Arc::new(ConstantField::new(2.0));
        let blend = BlendByField::new(sphere_a, sphere_b, smoothness_field);

        // At origin (center of first sphere): should be negative (inside)
        let dist_center_a = blend.distance(Vec3::ZERO);
        assert!(dist_center_a < 0.0);

        // At midpoint between spheres: should have blending effect
        let dist_mid = blend.distance(Vec3::new(4.0, 0.0, 0.0));
        assert!(dist_mid.is_finite());
    }

    #[test]
    fn test_blend_by_field_variable() {
        // Variable smoothness - more blending at some locations
        let sphere_a = Arc::new(Sphere::new(5.0));
        let sphere_b_sdf = Arc::new(Sphere::new(5.0));
        let sphere_b = Arc::new(crate::sdf::transforms::Translate::new(
            sphere_b_sdf,
            Vec3::new(8.0, 0.0, 0.0),
        ));

        // More smoothness near origin, less smoothness far away
        let smoothness_field = Arc::new(RadialField::new(
            Vec3::ZERO,
            0.0,
            20.0,
            5.0, // high smoothness at center
            0.5, // low smoothness at edges
        ));
        let blend = BlendByField::new(sphere_a, sphere_b, smoothness_field);

        // Just verify it produces valid results
        let dist = blend.distance(Vec3::new(4.0, 0.0, 0.0));
        assert!(dist.is_finite());

        let dist2 = blend.distance(Vec3::ZERO);
        assert!(dist2 < 0.0, "Should be inside first sphere");
    }

    #[test]
    fn test_offset_by_field_expansion_contraction() {
        let sphere = Arc::new(Sphere::new(10.0));

        // Positive offset expands (makes more negative inside)
        let expand_field = Arc::new(ConstantField::new(2.0));
        let expanded = OffsetByField::new(sphere.clone(), expand_field);
        assert!(expanded.distance(Vec3::ZERO) < -10.0);

        // Negative offset contracts (less negative inside)
        let contract_field = Arc::new(ConstantField::new(-2.0));
        let contracted = OffsetByField::new(sphere, contract_field);
        assert!(contracted.distance(Vec3::ZERO) > -10.0);
    }

    #[test]
    fn test_shell_with_field_zero_thickness() {
        let sphere = Arc::new(Sphere::new(10.0));
        let zero_thickness = Arc::new(ConstantField::new(0.0));
        let shell = ShellWithField::new(sphere, zero_thickness);

        // With zero thickness, the shell surface is exactly at abs(distance)
        // At original surface (d=0), abs(0) - 0 = 0
        let dist = shell.distance(Vec3::new(10.0, 0.0, 0.0));
        assert!(dist.abs() < 0.01);

        // Inside original (d=-1), abs(-1) - 0 = 1
        let dist_inside = shell.distance(Vec3::new(9.0, 0.0, 0.0));
        assert!((dist_inside - 1.0).abs() < 0.01);
    }
}
