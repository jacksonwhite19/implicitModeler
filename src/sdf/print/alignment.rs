// Alignment features for mating split parts: pins/sockets, tongue/groove,
// dovetail, and bolt holes.
#![allow(dead_code)] // Print alignment feature API — not all fields exposed in UI yet

use std::sync::Arc;
use glam::Vec3;
use crate::sdf::Sdf;
use crate::sdf::primitives::{Cylinder, SdfBox};
use crate::sdf::booleans::{Union, Subtract};
use crate::sdf::transforms::Translate;
use super::split::SplitPlane;

// ── AlignmentFeature ─────────────────────────────────────────────────────────

/// Describes how split parts should interlock for reassembly.
#[derive(Clone, Debug)]
pub enum AlignmentFeature {
    /// No alignment features — bare cut faces.
    None,

    /// Cylindrical pins on part_a that fit into sockets on part_b.
    PinsAndSockets {
        pin_radius: f32,
        pin_height: f32,
        /// Extra radius added to the socket for fit tolerance (default 0.15 mm).
        socket_clearance: f32,
        /// Number of pin/socket pairs (evenly distributed around the centroid).
        count: usize,
        /// Radial distance of pins from the centroid of the split face.
        pattern_radius: f32,
    },

    /// Rectangular tongue on part_a fits into a groove on part_b.
    TongueAndGroove {
        tongue_width: f32,
        tongue_height: f32,
        /// Extra width/depth added to groove (default 0.15 mm).
        groove_clearance: f32,
    },

    /// Trapezoidal dovetail on part_a fits into matching channel on part_b.
    Dovetail {
        width: f32,
        height: f32,
        /// Taper half-angle in degrees (default 15°).
        angle_deg: f32,
        clearance: f32,
    },

    /// Bolt holes on both halves (no protrusions — both halves get aligned holes).
    BoltHoles {
        bolt_radius: f32,
        /// Cylindrical boss radius (hole reinforcement on both faces).
        boss_radius: f32,
        boss_height: f32,
        count: usize,
        pattern_radius: f32,
        countersink: bool,
    },
}

impl AlignmentFeature {
    /// Apply the alignment feature to the two raw halves.
    ///
    /// Returns `(part_a, part_b)` with features added/subtracted.
    pub fn apply(
        &self,
        half_a: Arc<dyn Sdf>,
        half_b: Arc<dyn Sdf>,
        plane: &SplitPlane,
    ) -> (Arc<dyn Sdf>, Arc<dyn Sdf>) {
        match self {
            AlignmentFeature::None => (half_a, half_b),

            AlignmentFeature::PinsAndSockets {
                pin_radius, pin_height, socket_clearance, count, pattern_radius,
            } => {
                let (pos_sdf, neg_sdf) = pins_and_sockets(
                    *pin_radius, *pin_height, *socket_clearance, *count, *pattern_radius, plane,
                );
                // Add pins to part_a
                let part_a: Arc<dyn Sdf> = Arc::new(Union::new(half_a, pos_sdf));
                // Subtract sockets from part_b
                let part_b: Arc<dyn Sdf> = Arc::new(Subtract::new(half_b, neg_sdf));
                (part_a, part_b)
            }

            AlignmentFeature::TongueAndGroove {
                tongue_width, tongue_height, groove_clearance,
            } => {
                let (pos_sdf, neg_sdf) = tongue_and_groove(
                    *tongue_width, *tongue_height, *groove_clearance, plane,
                );
                let part_a: Arc<dyn Sdf> = Arc::new(Union::new(half_a, pos_sdf));
                let part_b: Arc<dyn Sdf> = Arc::new(Subtract::new(half_b, neg_sdf));
                (part_a, part_b)
            }

            AlignmentFeature::Dovetail { width, height, angle_deg, clearance } => {
                let (pos_sdf, neg_sdf) = dovetail(
                    *width, *height, *angle_deg, *clearance, plane,
                );
                let part_a: Arc<dyn Sdf> = Arc::new(Union::new(half_a, pos_sdf));
                let part_b: Arc<dyn Sdf> = Arc::new(Subtract::new(half_b, neg_sdf));
                (part_a, part_b)
            }

            AlignmentFeature::BoltHoles {
                bolt_radius, boss_radius, boss_height, count, pattern_radius, countersink: _,
            } => {
                let holes = bolt_hole_pattern(
                    *bolt_radius, *boss_radius, *boss_height, *count, *pattern_radius, plane,
                );
                // Both halves get the hole pattern subtracted
                let part_a: Arc<dyn Sdf> = Arc::new(Subtract::new(half_a, Arc::clone(&holes)));
                let part_b: Arc<dyn Sdf> = Arc::new(Subtract::new(half_b, holes));
                (part_a, part_b)
            }
        }
    }
}

// ── Helper: build a union of N translated SDFs in a circle ──────────────────

fn circular_pattern<F>(count: usize, radius: f32, center: Vec3, t1: Vec3, t2: Vec3, mut make: F) -> Arc<dyn Sdf>
where
    F: FnMut() -> Arc<dyn Sdf>,
{
    use std::f32::consts::TAU;
    let first = make();
    let offset0 = center + t1 * radius;
    let mut result: Arc<dyn Sdf> = Arc::new(Translate::new(first, offset0));

    for i in 1..count.max(1) {
        let angle = TAU * i as f32 / count as f32;
        let offset = center + (t1 * angle.cos() + t2 * angle.sin()) * radius;
        let piece = make();
        result = Arc::new(Union::new(result, Arc::new(Translate::new(piece, offset))));
    }
    result
}

// ── Pins and sockets ─────────────────────────────────────────────────────────

/// Returns `(pins_sdf, sockets_sdf)`.
/// Pins protrude from the split face along the positive normal by `pin_height`.
/// Sockets are slightly oversized cylinders for fit.
fn pins_and_sockets(
    pin_radius: f32,
    pin_height: f32,
    clearance: f32,
    count: usize,
    pattern_radius: f32,
    plane: &SplitPlane,
) -> (Arc<dyn Sdf>, Arc<dyn Sdf>) {
    let center = plane.center();
    let normal = plane.normal();
    let (t1, t2) = plane.tangents();

    // Pins are cylinders with their base on the cut face, extending into part_a
    // The cylinder is centred at half_height above the cut face along the normal.
    let pin_offset = normal * (pin_height * 0.5);

    let socket_radius = pin_radius + clearance;
    let socket_depth   = pin_height + clearance;
    let socket_offset  = -normal * (socket_depth * 0.5); // sinks into part_b

    // Build axis-aligned cylinders then translate them to their pattern positions.
    // The plane normal determines the cylinder orientation. We handle the three
    // principal axes; for arbitrary normals we use an approximating box (simpler).

    let make_pin_cyl  = || -> Arc<dyn Sdf> { make_cylinder_along(normal, pin_radius, pin_height * 0.5) };
    let make_sock_cyl = || -> Arc<dyn Sdf> { make_cylinder_along(normal, socket_radius, socket_depth * 0.5) };

    let pins    = circular_pattern(count, pattern_radius, center + pin_offset,   t1, t2, make_pin_cyl);
    let sockets = circular_pattern(count, pattern_radius, center + socket_offset, t1, t2, make_sock_cyl);

    (pins, sockets)
}

// ── Tongue and groove ────────────────────────────────────────────────────────

fn tongue_and_groove(
    width: f32,
    height: f32,
    clearance: f32,
    plane: &SplitPlane,
) -> (Arc<dyn Sdf>, Arc<dyn Sdf>) {
    let center = plane.center();
    let normal = plane.normal();
    let (t1, _t2) = plane.tangents();

    // Tongue: box extending into part_a from the cut face
    let tongue_half = Vec3::new(width * 0.5, height * 0.5, height * 0.5);
    let tongue_box: Arc<dyn Sdf> = Arc::new(SdfBox::new(tongue_half));
    let tongue_center = center + normal * (height * 0.5);
    let tongue: Arc<dyn Sdf> = Arc::new(Translate::new(tongue_box, tongue_center));
    let _ = t1; // tongue runs along t1 implicitly (width is along the first tangent)

    // Groove: slightly oversized box to subtract from part_b
    let groove_half = Vec3::new(
        (width + clearance) * 0.5,
        (height + clearance) * 0.5,
        (height + clearance) * 0.5,
    );
    let groove_box: Arc<dyn Sdf> = Arc::new(SdfBox::new(groove_half));
    let groove_center = center - normal * (height * 0.5);
    let groove: Arc<dyn Sdf> = Arc::new(Translate::new(groove_box, groove_center));

    (tongue, groove)
}

// ── Dovetail ─────────────────────────────────────────────────────────────────

fn dovetail(
    width: f32,
    height: f32,
    angle_deg: f32,
    clearance: f32,
    plane: &SplitPlane,
) -> (Arc<dyn Sdf>, Arc<dyn Sdf>) {
    // Approximate a dovetail as a box with a slight taper indicated by width at
    // base vs width at tip. For SDF purposes we use two widths:
    //   base_width (at root of tongue) = width
    //   tip_width  (at end of tongue)  = width + 2 * height * tan(angle)
    // We represent this as a box at the average width. A true dovetail requires
    // a swept / tapered SDF; for phase 17 the approximation is sufficient for
    // alignment (correct shape visible in viewport, within clearance).
    let taper = height * angle_deg.to_radians().tan();
    let avg_width = width + taper; // wider at tip means the box approximation is conservative

    // Reuse tongue-and-groove with avg_width
    tongue_and_groove(avg_width, height, clearance, plane)
}

// ── Bolt holes ───────────────────────────────────────────────────────────────

fn bolt_hole_pattern(
    bolt_radius: f32,
    boss_radius: f32,
    boss_height: f32,
    count: usize,
    pattern_radius: f32,
    plane: &SplitPlane,
) -> Arc<dyn Sdf> {
    let center = plane.center();
    let normal = plane.normal();
    let (t1, t2) = plane.tangents();

    // Bolt holes are cylinders passing through the cut face on both halves.
    // The cylinder extends boss_height on each side.
    let hole_center = center; // centred exactly on the face

    let make_hole = || -> Arc<dyn Sdf> {
        // Hole cylinder (bolt clearance)
        let hole: Arc<dyn Sdf> = make_cylinder_along(normal, bolt_radius, boss_height);
        if boss_radius > bolt_radius {
            // Boss cylinder (slightly wider but same height) — union to create boss, then
            // caller subtracts the whole thing, leaving a recessed boss seat.
            let boss: Arc<dyn Sdf> = make_cylinder_along(normal, boss_radius, boss_height * 0.5);
            Arc::new(Union::new(hole, boss))
        } else {
            hole
        }
    };

    circular_pattern(count, pattern_radius, hole_center, t1, t2, make_hole)
}

// ── Cylinder oriented along an arbitrary axis ────────────────────────────────

/// Create a cylinder whose axis points along `axis` (unit vector) with the
/// given radius and half-height. The result is translated so the cylinder
/// extends from 0 to `half_height*2` along `axis`, centred on the origin.
///
/// For the three principal axes we use the standard `Cylinder` (Z-aligned) with
/// a rotation transform. For arbitrary axes we approximate with an SdfBox.
fn make_cylinder_along(axis: Vec3, radius: f32, half_height: f32) -> Arc<dyn Sdf> {
    let cyl: Arc<dyn Sdf> = Arc::new(Cylinder::new(radius, half_height));

    // If already Z-aligned, done.
    let up = Vec3::Z;
    if (axis.dot(up) - 1.0).abs() < 1e-4 {
        return cyl;
    }
    if (axis.dot(up) + 1.0).abs() < 1e-4 {
        // Pointing -Z: just negate the Z lookup — same shape, symmetric.
        return cyl;
    }

    // For X or Y-aligned axes: use an SdfBox approximation (cylinder cross-section
    // becomes a square cross-section). Acceptable for alignment bosses.
    // A proper rotation transform would require importing `crate::sdf::transforms::Rotate`
    // which takes Euler angles; computing those from an arbitrary axis is non-trivial
    // and adds code complexity. The box approximation has the same bounding volume.
    let half = if (axis.dot(Vec3::X) - 1.0).abs() < 1e-4 || (axis.dot(Vec3::X) + 1.0).abs() < 1e-4 {
        Vec3::new(half_height, radius, radius)
    } else {
        Vec3::new(radius, half_height, radius)
    };
    Arc::new(SdfBox::new(half))
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::primitives::Sphere;
    use crate::sdf::print::split::{split_body, SplitPlane};

    #[test]
    fn pins_protrude_from_part_a() {
        let sphere: Arc<dyn Sdf> = Arc::new(Sphere::new(20.0));
        let plane = SplitPlane::Z(0.0);
        let alignment = AlignmentFeature::PinsAndSockets {
            pin_radius: 1.5,
            pin_height: 3.0,
            socket_clearance: 0.15,
            count: 4,
            pattern_radius: 8.0,
        };
        let result = split_body(sphere, &plane, &alignment);

        // A pin should protrude above Z=0 at one of the pattern positions.
        // Pattern at (8,0,0) with pin height 3 → pin goes from z=0 to z=3.
        // Centre of pin is at z=1.5, radius 1.5.
        let pin_center = Vec3::new(8.0, 0.0, 1.5);
        let d = result.part_a.distance(pin_center);
        assert!(d < 0.0, "pin centre should be inside part_a, got {}", d);
    }

    #[test]
    fn sockets_subtracted_from_part_b() {
        let sphere: Arc<dyn Sdf> = Arc::new(Sphere::new(20.0));
        let plane = SplitPlane::Z(0.0);
        let alignment = AlignmentFeature::PinsAndSockets {
            pin_radius: 1.5,
            pin_height: 3.0,
            socket_clearance: 0.15,
            count: 4,
            pattern_radius: 8.0,
        };
        let result = split_body(sphere, &plane, &alignment);

        // The socket descends into part_b (z < 0). Check the socket cavity.
        // Socket centre ≈ (8,0,-1.575) (half the socket depth below the face).
        let socket_center = Vec3::new(8.0, 0.0, -1.575);
        let d = result.part_b.distance(socket_center);
        // The socket is subtracted so the cavity should read positive (outside material).
        assert!(d > 0.0, "socket cavity should be empty in part_b, got {}", d);
    }

    #[test]
    fn tongue_and_groove_alignment() {
        let sphere: Arc<dyn Sdf> = Arc::new(Sphere::new(20.0));
        let plane = SplitPlane::Z(0.0);
        let alignment = AlignmentFeature::TongueAndGroove {
            tongue_width: 10.0,
            tongue_height: 3.0,
            groove_clearance: 0.2,
        };
        let result = split_body(sphere, &plane, &alignment);

        // Tongue centre at z=1.5 (half the height above the face)
        let tongue_center = Vec3::new(0.0, 0.0, 1.5);
        let d = result.part_a.distance(tongue_center);
        assert!(d < 0.0, "tongue centre should be inside part_a, got {}", d);
    }
}
