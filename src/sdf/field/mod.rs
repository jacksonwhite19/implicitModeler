// Field trait and field operations module

use glam::Vec3;

/// Trait for scalar field evaluation at any point in 3D space
///
/// Fields represent scalar values that vary across space, distinct from SDFs which
/// represent signed distances to surfaces. Fields can be used to control geometric
/// properties like thickness, blend radius, or lattice density.
pub trait Field: Send + Sync {
    /// Evaluate the field at a given point
    fn evaluate(&self, point: Vec3) -> f32;

    /// Optional: Get approximate bounds for optimization
    /// Returns (min_value, max_value) or None if unbounded
    #[allow(dead_code)] // Part of Field trait API for optimization
    fn bounds(&self) -> Option<(f32, f32)> {
        None
    }
}

pub mod primitives;
pub mod arithmetic;
pub mod gradients;
pub mod operations;
pub mod lattice;
