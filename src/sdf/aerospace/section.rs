// Common trait for 2-D cross-sections used in lofted SDF solids.

use glam::Vec2;
use std::any::Any;
use std::sync::Arc;

/// A 2-D cross-section that can evaluate a signed distance and blend toward another.
///
/// Implement this trait to create custom cross-section shapes that plug into
/// `LoftedWing`, `LoftedFuselage`, or any future lofted primitive.
pub trait Section2D: Send + Sync {
    /// Signed distance from `point` to the cross-section outline (negative = inside).
    fn distance_2d(&self, point: Vec2) -> f32;

    /// Compute the distance of the blended section at `point` **without allocating**.
    ///
    /// This is the method used in the SDF hot path (viewport ray-march, mesh extraction).
    /// The default implementation interpolates the two distances — a valid SDF approximation.
    /// Implementations may override for geometrically exact blending.
    fn distance_lerped_2d(&self, other: &dyn Section2D, t: f32, point: Vec2) -> f32 {
        self.distance_2d(point) * (1.0 - t) + other.distance_2d(point) * t
    }

    /// Materialise a blended section as a heap-allocated object.
    ///
    /// Use this when you need an actual `Section2D` value (e.g. geometry export,
    /// preview). For distance queries, prefer `distance_lerped_2d`.
    fn lerp_to(&self, other: &dyn Section2D, t: f32) -> Arc<dyn Section2D>;

    /// Returns `self` as `&dyn Any`, required for downcasting inside `lerp_to` implementations.
    fn as_any(&self) -> &dyn Any;
}
