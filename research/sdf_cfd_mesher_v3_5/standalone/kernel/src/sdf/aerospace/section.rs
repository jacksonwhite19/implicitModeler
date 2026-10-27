// Common trait for 2-D cross-sections used in lofted SDF solids.

use glam::Vec2;
use std::any::Any;
use std::sync::Arc;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SectionBounds2D {
    pub min: Vec2,
    pub max: Vec2,
}

impl SectionBounds2D {
    pub fn new(min: Vec2, max: Vec2) -> Self {
        Self { min, max }
    }

    pub fn from_points(points: &[Vec2]) -> Option<Self> {
        let mut iter = points.iter().copied();
        let first = iter.next()?;
        if !first.is_finite() {
            return None;
        }
        let mut min = first;
        let mut max = first;
        for point in iter {
            if !point.is_finite() {
                return None;
            }
            min = min.min(point);
            max = max.max(point);
        }
        Some(Self { min, max })
    }

    pub fn expanded(&self, amount: f32) -> Self {
        let amount = amount.max(0.0);
        Self {
            min: self.min - Vec2::splat(amount),
            max: self.max + Vec2::splat(amount),
        }
    }

    pub fn scaled(&self, scale: Vec2) -> Self {
        let corners = [
            self.min * scale,
            Vec2::new(self.max.x, self.min.y) * scale,
            Vec2::new(self.min.x, self.max.y) * scale,
            self.max * scale,
        ];
        Self::from_points(&corners).unwrap_or(*self)
    }

    pub fn transposed(&self) -> Self {
        Self {
            min: Vec2::new(self.min.y, self.min.x),
            max: Vec2::new(self.max.y, self.max.x),
        }
    }

    pub fn union(&self, other: &Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    pub fn corners(&self) -> [Vec2; 4] {
        [
            self.min,
            Vec2::new(self.max.x, self.min.y),
            Vec2::new(self.min.x, self.max.y),
            self.max,
        ]
    }

    pub fn max_extent(&self) -> f32 {
        self.min.length().max(self.max.length()).max(
            Vec2::new(self.min.x, self.max.y)
                .length()
                .max(Vec2::new(self.max.x, self.min.y).length()),
        )
    }
}

/// A 2-D cross-section that can evaluate a signed distance and blend toward another.
///
/// Implement this trait to create custom cross-section shapes that plug into
/// `LoftedWing`, `LoftedFuselage`, or any future lofted primitive.
pub trait Section2D: Send + Sync {
    /// Signed distance from `point` to the cross-section outline (negative = inside).
    fn distance_2d(&self, point: Vec2) -> f32;

    /// Conservative local support bounds for the cross-section.
    ///
    /// This is used by the conditioning backend to turn lofts, sweeps, ducts,
    /// and profile-driven geometry into bounded dirty regions without a later
    /// export-time metadata pass.
    fn bounds_2d(&self) -> Option<SectionBounds2D> {
        None
    }

    /// Stable numeric parameters used by SDF metadata fingerprints.
    ///
    /// Implementations should include enough shape-defining data that a
    /// same-envelope profile edit still changes the parent geometry metadata.
    /// The default falls back to bounds, which is conservative but weaker than
    /// true profile ownership.
    fn metadata_fingerprint_parameters(&self) -> Vec<f32> {
        self.bounds_2d()
            .map(|bounds| vec![bounds.min.x, bounds.min.y, bounds.max.x, bounds.max.y])
            .unwrap_or_default()
    }

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
