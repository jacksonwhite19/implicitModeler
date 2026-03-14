// Field arithmetic operations

use glam::Vec3;
use std::sync::Arc;
use super::Field;

/// Add two fields
pub struct FieldAdd {
    pub a: Arc<dyn Field>,
    pub b: Arc<dyn Field>,
}

impl FieldAdd {
    pub fn new(a: Arc<dyn Field>, b: Arc<dyn Field>) -> Self {
        Self { a, b }
    }
}

impl Field for FieldAdd {
    fn evaluate(&self, point: Vec3) -> f32 {
        self.a.evaluate(point) + self.b.evaluate(point)
    }

    fn bounds(&self) -> Option<(f32, f32)> {
        match (self.a.bounds(), self.b.bounds()) {
            (Some((a_min, a_max)), Some((b_min, b_max))) => {
                Some((a_min + b_min, a_max + b_max))
            }
            _ => None,
        }
    }
}

/// Multiply two fields
pub struct FieldMultiply {
    pub a: Arc<dyn Field>,
    pub b: Arc<dyn Field>,
}

impl FieldMultiply {
    pub fn new(a: Arc<dyn Field>, b: Arc<dyn Field>) -> Self {
        Self { a, b }
    }
}

impl Field for FieldMultiply {
    fn evaluate(&self, point: Vec3) -> f32 {
        self.a.evaluate(point) * self.b.evaluate(point)
    }

    fn bounds(&self) -> Option<(f32, f32)> {
        match (self.a.bounds(), self.b.bounds()) {
            (Some((a_min, a_max)), Some((b_min, b_max))) => {
                // For multiplication, need all four combinations
                let products = [
                    a_min * b_min,
                    a_min * b_max,
                    a_max * b_min,
                    a_max * b_max,
                ];
                let min = products.iter().fold(f32::INFINITY, |a, &b| a.min(b));
                let max = products.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));
                Some((min, max))
            }
            _ => None,
        }
    }
}

/// Minimum of two fields
pub struct FieldMin {
    pub a: Arc<dyn Field>,
    pub b: Arc<dyn Field>,
}

impl FieldMin {
    pub fn new(a: Arc<dyn Field>, b: Arc<dyn Field>) -> Self {
        Self { a, b }
    }
}

impl Field for FieldMin {
    fn evaluate(&self, point: Vec3) -> f32 {
        self.a.evaluate(point).min(self.b.evaluate(point))
    }

    fn bounds(&self) -> Option<(f32, f32)> {
        match (self.a.bounds(), self.b.bounds()) {
            (Some((a_min, a_max)), Some((b_min, b_max))) => {
                Some((a_min.min(b_min), a_max.min(b_max)))
            }
            _ => None,
        }
    }
}

/// Maximum of two fields
pub struct FieldMax {
    pub a: Arc<dyn Field>,
    pub b: Arc<dyn Field>,
}

impl FieldMax {
    pub fn new(a: Arc<dyn Field>, b: Arc<dyn Field>) -> Self {
        Self { a, b }
    }
}

impl Field for FieldMax {
    fn evaluate(&self, point: Vec3) -> f32 {
        self.a.evaluate(point).max(self.b.evaluate(point))
    }

    fn bounds(&self) -> Option<(f32, f32)> {
        match (self.a.bounds(), self.b.bounds()) {
            (Some((a_min, a_max)), Some((b_min, b_max))) => {
                Some((a_min.max(b_min), a_max.max(b_max)))
            }
            _ => None,
        }
    }
}

/// Absolute value of a field
pub struct FieldAbs {
    pub child: Arc<dyn Field>,
}

impl FieldAbs {
    pub fn new(child: Arc<dyn Field>) -> Self {
        Self { child }
    }
}

impl Field for FieldAbs {
    fn evaluate(&self, point: Vec3) -> f32 {
        self.child.evaluate(point).abs()
    }

    fn bounds(&self) -> Option<(f32, f32)> {
        self.child.bounds().map(|(min, max)| {
            if min >= 0.0 {
                // All positive - no change
                (min, max)
            } else if max <= 0.0 {
                // All negative - flip and swap
                (max.abs(), min.abs())
            } else {
                // Spans zero - minimum is 0, maximum is larger absolute value
                (0.0, min.abs().max(max))
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::field::primitives::ConstantField;

    #[test]
    fn test_field_add() {
        let a = Arc::new(ConstantField::new(3.0));
        let b = Arc::new(ConstantField::new(7.0));
        let sum = FieldAdd::new(a, b);

        assert_eq!(sum.evaluate(Vec3::ZERO), 10.0);
        assert_eq!(sum.evaluate(Vec3::new(100.0, 200.0, 300.0)), 10.0);
        assert_eq!(sum.bounds(), Some((10.0, 10.0)));
    }

    #[test]
    fn test_field_add_position() {
        use crate::sdf::field::primitives::PositionXField;

        let a = Arc::new(ConstantField::new(5.0));
        let b = Arc::new(PositionXField);
        let sum = FieldAdd::new(a, b);

        // At x=0: 5 + 0 = 5
        assert_eq!(sum.evaluate(Vec3::new(0.0, 0.0, 0.0)), 5.0);

        // At x=10: 5 + 10 = 15
        assert_eq!(sum.evaluate(Vec3::new(10.0, 0.0, 0.0)), 15.0);

        // At x=-3: 5 + (-3) = 2
        assert_eq!(sum.evaluate(Vec3::new(-3.0, 0.0, 0.0)), 2.0);
    }

    #[test]
    fn test_field_multiply() {
        let a = Arc::new(ConstantField::new(4.0));
        let b = Arc::new(ConstantField::new(5.0));
        let product = FieldMultiply::new(a, b);

        assert_eq!(product.evaluate(Vec3::ZERO), 20.0);
        assert_eq!(product.bounds(), Some((20.0, 20.0)));
    }

    #[test]
    fn test_field_multiply_negative() {
        let a = Arc::new(ConstantField::new(-2.0));
        let b = Arc::new(ConstantField::new(3.0));
        let product = FieldMultiply::new(a, b);

        assert_eq!(product.evaluate(Vec3::ZERO), -6.0);
        assert_eq!(product.bounds(), Some((-6.0, -6.0)));
    }

    #[test]
    fn test_field_min() {
        let a = Arc::new(ConstantField::new(3.0));
        let b = Arc::new(ConstantField::new(7.0));
        let min_field = FieldMin::new(a, b);

        assert_eq!(min_field.evaluate(Vec3::ZERO), 3.0);
        assert_eq!(min_field.bounds(), Some((3.0, 3.0)));
    }

    #[test]
    fn test_field_max() {
        let a = Arc::new(ConstantField::new(3.0));
        let b = Arc::new(ConstantField::new(7.0));
        let max_field = FieldMax::new(a, b);

        assert_eq!(max_field.evaluate(Vec3::ZERO), 7.0);
        assert_eq!(max_field.bounds(), Some((7.0, 7.0)));
    }

    #[test]
    fn test_field_abs_positive() {
        let child = Arc::new(ConstantField::new(5.0));
        let abs_field = FieldAbs::new(child);

        assert_eq!(abs_field.evaluate(Vec3::ZERO), 5.0);
        assert_eq!(abs_field.bounds(), Some((5.0, 5.0)));
    }

    #[test]
    fn test_field_abs_negative() {
        let child = Arc::new(ConstantField::new(-5.0));
        let abs_field = FieldAbs::new(child);

        assert_eq!(abs_field.evaluate(Vec3::ZERO), 5.0);
        assert_eq!(abs_field.bounds(), Some((5.0, 5.0)));
    }

    #[test]
    fn test_field_abs_position() {
        use crate::sdf::field::primitives::PositionXField;

        let child = Arc::new(PositionXField);
        let abs_field = FieldAbs::new(child);

        assert_eq!(abs_field.evaluate(Vec3::new(5.0, 0.0, 0.0)), 5.0);
        assert_eq!(abs_field.evaluate(Vec3::new(-5.0, 0.0, 0.0)), 5.0);
        assert_eq!(abs_field.evaluate(Vec3::new(0.0, 0.0, 0.0)), 0.0);
    }

    #[test]
    fn test_field_composition() {
        // Test (a + b) * c
        let a = Arc::new(ConstantField::new(2.0));
        let b = Arc::new(ConstantField::new(3.0));
        let c = Arc::new(ConstantField::new(4.0));

        let sum = Arc::new(FieldAdd::new(a, b));
        let product = FieldMultiply::new(sum, c);

        // (2 + 3) * 4 = 20
        assert_eq!(product.evaluate(Vec3::ZERO), 20.0);
    }

    #[test]
    fn test_complex_field_expression() {
        // Test abs(min(a, b) - c)
        let a = Arc::new(ConstantField::new(10.0));
        let b = Arc::new(ConstantField::new(15.0));
        let c = Arc::new(ConstantField::new(12.0));

        let min_ab = Arc::new(FieldMin::new(a, b));
        let diff = Arc::new(FieldAdd::new(
            min_ab,
            Arc::new(FieldMultiply::new(c, Arc::new(ConstantField::new(-1.0)))),
        ));
        let result = FieldAbs::new(diff);

        // min(10, 15) = 10, 10 - 12 = -2, abs(-2) = 2
        assert_eq!(result.evaluate(Vec3::ZERO), 2.0);
    }
}
