// Structural wing primitives built by composing existing SDF operations.
// No new Sdf structs — every function returns an Arc<dyn Sdf> assembled from
// the existing primitive/transform/boolean toolkit.

use glam::{Vec3, Quat};
use std::f32::consts::FRAC_PI_2;
use std::sync::Arc;
use crate::sdf::Sdf;
use crate::sdf::primitives::{SdfBox, Cylinder};
use crate::sdf::transforms::{Translate, Rotate};

/// A thin slab centred at `span_pos` on the Y axis, extending far in X and Z.
///
/// Intersect this with a wing SDF to obtain a rib cross-section at that station.
///
/// * `span_pos`  – absolute Y coordinate of the rib centreline (same units as the wing)
/// * `thickness` – full Y-thickness of the slab
pub fn rib_slab(span_pos: f32, thickness: f32) -> Arc<dyn Sdf> {
    // The slab is a very wide/deep box, thin in Y.
    // 10 000 units in X and Z guarantees it covers any realistic wing planform.
    let slab = Arc::new(SdfBox::new(Vec3::new(10_000.0, thickness * 0.5, 10_000.0)));
    Arc::new(Translate::new(slab, Vec3::new(0.0, span_pos, 0.0)))
}

/// A cylindrical spar running along the full Y span, centred at chord position `chord_pos`.
///
/// The standard `Cylinder` primitive is aligned along Z.
/// Rotate::new applies `rotation.inverse()` to the query point, so storing
/// `Quat::from_rotation_x(-FRAC_PI_2)` causes the inverse (+FRAC_PI_2) to be
/// applied at query time, mapping Y-axis queries onto the Z cylinder correctly.
///
/// * `chord_pos` – absolute X position of the spar centreline (e.g. 0.25 * root_chord)
/// * `radius`    – spar tube radius
pub fn spar_cylinder(chord_pos: f32, radius: f32) -> Arc<dyn Sdf> {
    // Build a very long Z-axis cylinder, then rotate it to lie along Y.
    let cyl = Arc::new(Cylinder::new(radius, 10_000.0));
    // rotation.inverse() = Quat::from_rotation_x(+FRAC_PI_2) transforms the query
    // so that (x, y, z) → (x, −z, y), mapping a Y-axis point onto the Z axis.
    let rotated = Arc::new(Rotate::new(cyl, Quat::from_rotation_x(-FRAC_PI_2)));
    Arc::new(Translate::new(rotated, Vec3::new(chord_pos, 0.0, 0.0)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::Sdf;

    #[test]
    fn test_rib_slab_inside() {
        let slab = rib_slab(5.0, 1.0);
        // Centre of slab should be inside
        assert!(slab.distance(Vec3::new(0.0, 5.0, 0.0)) < 0.0);
        // Half-thickness away should be on the surface
        let d = slab.distance(Vec3::new(0.0, 5.5, 0.0));
        assert!(d.abs() < 0.01, "expected ~0 at slab face, got {}", d);
        // Beyond the slab should be outside
        assert!(slab.distance(Vec3::new(0.0, 7.0, 0.0)) > 0.0);
    }

    #[test]
    fn test_rib_slab_wide() {
        // Slab must cover wide chord/thickness extents
        let slab = rib_slab(0.0, 0.5);
        assert!(slab.distance(Vec3::new(100.0, 0.0, 50.0)) < 0.0,
            "slab should contain large X/Z offsets");
    }

    #[test]
    fn test_spar_cylinder_axis() {
        let spar = spar_cylinder(3.0, 0.5);
        // A point on the Y axis at the chord position should be inside the spar tube
        assert!(spar.distance(Vec3::new(3.0,  0.0, 0.0)) < 0.0);
        assert!(spar.distance(Vec3::new(3.0, 50.0, 0.0)) < 0.0,
            "spar should extend along Y");
        // A point displaced radially beyond the radius should be outside
        assert!(spar.distance(Vec3::new(3.0 + 1.0, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn test_spar_cylinder_not_z_axis() {
        // Verify the spar is NOT still along Z (rotation worked)
        let spar = spar_cylinder(0.0, 0.5);
        // A point far along Z (but on spar axis in XY) should be outside the tube
        assert!(spar.distance(Vec3::new(0.0, 0.0, 5.0)) > 0.0,
            "spar should not extend along Z");
    }
}
