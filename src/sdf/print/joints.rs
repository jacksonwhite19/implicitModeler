// Interlocking joint generators for FDM printed assemblies.
#![allow(dead_code)] // Joint geometry API — null() and other helpers not yet called from UI
//
// Each function returns a (add_a, add_b, void_a, void_b) tuple of SDFs. Apply
// to parts as: union(subtract(part_a, void_a), add_a) and similarly for part_b.
//
// Coordinate convention: `axis` is the direction part_a's feature protrudes into
// part_b (or the mating direction). `position` is the centre of the joint
// interface. `length` is the joint extent along the third direction.

use std::sync::Arc;
use glam::Vec3;
use crate::sdf::Sdf;
use crate::sdf::primitives::{Cylinder, SdfBox};
use crate::sdf::booleans::{Union, Subtract};
use crate::sdf::transforms::{Translate, Rotate};
use crate::sdf::aerospace::mechanical::CappedCone;
use super::fasteners::{orient_along, NullSdf};

// ── Joint delta ───────────────────────────────────────────────────────────────

/// Packed (addition, void) delta for one part of a joint.
///
/// Apply with: `union(subtract(part, delta.void), delta.addition)`.
#[derive(Clone)]
pub struct JointDelta {
    pub addition: Arc<dyn Sdf>,
    pub void:     Arc<dyn Sdf>,
}

impl JointDelta {
    pub fn new(addition: Arc<dyn Sdf>, void: Arc<dyn Sdf>) -> Self {
        Self { addition, void }
    }

    /// No-op delta: leaves the part unchanged.
    pub fn null() -> Self {
        Self {
            addition: Arc::new(NullSdf),
            void:     Arc::new(NullSdf),
        }
    }

    /// Apply this delta to a part SDF: `union(subtract(part, void), addition)`.
    pub fn apply(&self, part: Arc<dyn Sdf>) -> Arc<dyn Sdf> {
        Arc::new(Union::new(
            Arc::new(Subtract::new(part, Arc::clone(&self.void))),
            Arc::clone(&self.addition),
        ))
    }
}

// ── Orientation helpers ───────────────────────────────────────────────────────

/// Box with half-extents (hx, hy, hz) centred at `offset` in Z-aligned space.
fn box_at(hx: f32, hy: f32, hz: f32, offset: Vec3) -> Arc<dyn Sdf> {
    Arc::new(Translate::new(
        Arc::new(SdfBox::new(Vec3::new(hx, hy, hz))),
        offset,
    ))
}

/// Rotate a Z-aligned SDF so its Z axis points along `axis`, then translate to `pos`.
fn place_along_axis(child: Arc<dyn Sdf>, pos: Vec3, axis: Vec3) -> Arc<dyn Sdf> {
    Arc::new(Translate::new(orient_along(child, axis), pos))
}

// ── Dovetail joint ────────────────────────────────────────────────────────────

/// Trapezoidal dovetail joint.
///
/// `axis`: direction the male (part_a) feature protrudes toward part_b.
/// `position`: centre of the joint interface.
/// `length`: extent along the sliding direction (perpendicular to both axis and width).
///
/// Returns `(delta_a, delta_b)`.
pub fn dovetail_joint(
    length:    f32,
    width:     f32,
    height:    f32,
    angle_deg: f32,
    clearance: f32,
    position:  Vec3,
    axis:      Vec3,
) -> (JointDelta, JointDelta) {
    // Average width approximation (see alignment.rs dovetail)
    let taper   = height * angle_deg.to_radians().tan();
    let avg_w   = width + taper;

    // Male protrusion on part_a: along +axis from position, half-height = height/2
    let male_z: Arc<dyn Sdf> = box_at(
        avg_w / 2.0,
        length / 2.0,
        height / 2.0,
        Vec3::new(0.0, 0.0, height / 2.0),
    );
    let male = place_along_axis(male_z, position, axis);

    // Female socket on part_b: along -axis from position, with clearance.
    // Build extending in +Z so orient_along(-axis) flips it into -axis direction.
    let cl      = clearance;
    let fem_z: Arc<dyn Sdf> = box_at(
        (avg_w + cl) / 2.0,
        (length + cl) / 2.0,
        (height + cl) / 2.0,
        Vec3::new(0.0, 0.0, (height + cl) / 2.0),
    );
    let female = place_along_axis(fem_z, position, -axis);

    let delta_a = JointDelta::new(male, Arc::new(NullSdf));
    let delta_b = JointDelta::new(Arc::new(NullSdf), female);
    (delta_a, delta_b)
}

// ── Finger joint ──────────────────────────────────────────────────────────────

/// Interlocking rectangular finger joint.
///
/// Odd-indexed fingers go to part_a; even-indexed to part_b. Use an odd `count`
/// for symmetry.  `axis` is the direction the fingers point (into the mating part).
///
/// Returns `(delta_a, delta_b)`.
pub fn finger_joint(
    length:       f32,
    finger_width: f32,
    finger_height: f32,
    count:        usize,
    clearance:    f32,
    position:     Vec3,
    axis:         Vec3,
) -> (JointDelta, JointDelta) {
    let count = count.max(1);
    let total = count as f32 * finger_width;

    // Fingers extend along +axis for finger_height
    let mut add_a: Option<Arc<dyn Sdf>> = None;
    let mut add_b: Option<Arc<dyn Sdf>> = None;
    let mut void_a: Option<Arc<dyn Sdf>> = None;
    let mut void_b: Option<Arc<dyn Sdf>> = None;

    let push = |acc: &mut Option<Arc<dyn Sdf>>, s: Arc<dyn Sdf>| {
        *acc = Some(match acc.take() {
            None    => s,
            Some(r) => Arc::new(Union::new(r, s)),
        });
    };

    for i in 0..count {
        // Centre of finger along the width direction (X in Z-aligned space)
        let x_center = -total / 2.0 + (i as f32 + 0.5) * finger_width;

        // Finger solid (Z-aligned): from z=0 to z=finger_height
        let finger: Arc<dyn Sdf> = Arc::new(Translate::new(
            Arc::new(SdfBox::new(Vec3::new(
                finger_width / 2.0,
                length / 2.0,
                finger_height / 2.0,
            ))),
            Vec3::new(x_center, 0.0, finger_height / 2.0),
        ));
        // Socket (Z-aligned): same but with clearance, going the other way (z < 0)
        let socket: Arc<dyn Sdf> = Arc::new(Translate::new(
            Arc::new(SdfBox::new(Vec3::new(
                (finger_width + clearance) / 2.0,
                (length       + clearance) / 2.0,
                (finger_height + clearance) / 2.0,
            ))),
            Vec3::new(x_center, 0.0, -(finger_height + clearance) / 2.0),
        ));

        let finger_placed = place_along_axis(Arc::clone(&finger), position, axis);
        let socket_placed = place_along_axis(Arc::clone(&socket), position, -axis);

        if i % 2 == 0 {
            // Even finger → part_a gets the protrusion, part_b gets the socket
            push(&mut add_a, finger_placed);
            push(&mut void_b, socket_placed);
        } else {
            // Odd finger → part_b gets the protrusion, part_a gets the socket
            push(&mut add_b, finger_placed);
            push(&mut void_a, socket_placed);
        }
    }

    let wrap = |o: Option<Arc<dyn Sdf>>| -> Arc<dyn Sdf> {
        o.unwrap_or_else(|| Arc::new(NullSdf) as Arc<dyn Sdf>)
    };

    let delta_a = JointDelta::new(wrap(add_a), wrap(void_a));
    let delta_b = JointDelta::new(wrap(add_b), wrap(void_b));
    (delta_a, delta_b)
}

// ── Press-fit connector ────────────────────────────────────────────────────────

/// Cylindrical press-fit pin on part_a → matching socket on part_b with
/// interference fit.
///
/// Returns `(delta_a, delta_b)`.
pub fn press_fit(
    pin_radius:    f32,
    pin_length:    f32,
    socket_depth:  f32,
    interference:  f32,
    position:      Vec3,
    direction:     Vec3,
) -> (JointDelta, JointDelta) {
    // Pin: cylinder from z=0 to z=pin_length along direction
    let pin_z: Arc<dyn Sdf> = Arc::new(Translate::new(
        Arc::new(Cylinder::new(pin_radius, pin_length / 2.0)),
        Vec3::new(0.0, 0.0, pin_length / 2.0),
    ));
    let pin = place_along_axis(pin_z, position, direction);

    // Socket: cylinder going in -direction with interference fit (socket_r = pin_r - interference)
    let socket_r = (pin_radius - interference).max(pin_radius * 0.5);
    let socket_z: Arc<dyn Sdf> = Arc::new(Translate::new(
        Arc::new(Cylinder::new(socket_r, socket_depth / 2.0)),
        Vec3::new(0.0, 0.0, socket_depth / 2.0),
    ));
    let socket = place_along_axis(socket_z, position, -direction);

    let delta_a = JointDelta::new(pin, Arc::new(NullSdf));
    let delta_b = JointDelta::new(Arc::new(NullSdf), socket);
    (delta_a, delta_b)
}

// ── Snap-fit connector ────────────────────────────────────────────────────────

/// Unidirectional snap connector.
///
/// Part_a has a flexible cantilever arm with a wedge tip; part_b has a
/// matching catch ledge.
///
/// Returns `(delta_a, delta_b)`.
pub fn snap_connector(
    width:      f32,
    height:     f32,
    engagement: f32,
    clearance:  f32,
    position:   Vec3,
    direction:  Vec3,
) -> (JointDelta, JointDelta) {
    // Arm: a thin rectangular beam extending from z=0 to z=height along direction
    let arm_thickness = 0.8_f32;
    let arm_z: Arc<dyn Sdf> = Arc::new(Translate::new(
        Arc::new(SdfBox::new(Vec3::new(width / 2.0, arm_thickness / 2.0, height / 2.0))),
        Vec3::new(0.0, 0.0, height / 2.0),
    ));
    // Wedge tip at the end of the arm
    let _tip_z: Arc<dyn Sdf> = Arc::new(Translate::new(
        Arc::new(CappedCone {
            r1: arm_thickness / 2.0 + engagement,  // wide at entry
            r2: arm_thickness / 2.0,                // narrow at tip
            h:  engagement / 2.0,
        }),
        Vec3::new(0.0, 0.0, height + engagement / 2.0),
    ));
    // Note: the CappedCone here is oriented along Z (cylinder-like), but used
    // in the Y direction for the wedge. We approximate with a tapered box instead.
    let wedge_z: Arc<dyn Sdf> = Arc::new(Translate::new(
        Arc::new(SdfBox::new(Vec3::new(
            width / 2.0,
            (arm_thickness + engagement) / 2.0,
            engagement / 2.0,
        ))),
        Vec3::new(0.0, engagement / 4.0, height + engagement / 2.0),
    ));
    let arm_with_wedge: Arc<dyn Sdf> = Arc::new(Union::new(arm_z, wedge_z));
    let arm = place_along_axis(arm_with_wedge, position, direction);

    // Catch ledge on part_b: a small shelf that the wedge snaps past
    let catch_z: Arc<dyn Sdf> = Arc::new(Translate::new(
        Arc::new(SdfBox::new(Vec3::new(
            (width + clearance) / 2.0,
            engagement / 2.0,
            (engagement + 0.5) / 2.0,
        ))),
        Vec3::new(0.0, 0.0, height + (engagement + 0.5) / 2.0),
    ));
    let catch = place_along_axis(catch_z, position, direction);
    // The catch is subtracted from part_b (it's a pocket)
    let catch_void = catch;

    let delta_a = JointDelta::new(arm, Arc::new(NullSdf));
    let delta_b = JointDelta::new(Arc::new(NullSdf), catch_void);
    (delta_a, delta_b)
}

// ── Living hinge strip ────────────────────────────────────────────────────────

/// A continuous thin flexible strip connecting two bodies along a fold line.
///
/// `axis` is the fold line direction.  The strip runs for `width` along `axis`
/// and has a short `length` perpendicular to axis.
///
/// Returns `(delta_a, delta_b)` — both parts get the hinge addition and the
/// hinge occupies the space between them.
pub fn living_hinge_strip(
    width:     f32,
    thickness: f32,
    length:    f32,
    position:  Vec3,
    axis:      Vec3,
) -> (JointDelta, JointDelta) {
    let axis = axis.normalize();

    // Compute a tangent perpendicular to axis (the strip runs along this tangent)
    let perp = {
        let up = if axis.abs().x < 0.9 { Vec3::X } else { Vec3::Y };
        (up - axis * up.dot(axis)).normalize()
    };

    // Thin hinge strip: width along axis, thickness along perp, length spans both parts
    let strip: Arc<dyn Sdf> = Arc::new(Translate::new(
        Arc::new(SdfBox::new(Vec3::new(width / 2.0, thickness / 2.0, length / 2.0))),
        Vec3::ZERO,
    ));
    // Rotate so the strip length runs along perp (perpendicular to axis)
    let q       = glam::Quat::from_rotation_arc(Vec3::Z, perp);
    let strip_r: Arc<dyn Sdf> = Arc::new(Rotate::new(strip, q));
    let strip   = Arc::new(Translate::new(strip_r, position));

    // Reinforcement bosses at each end of the hinge strip
    let end_boss = |side: f32| -> Arc<dyn Sdf> {
        Arc::new(Translate::new(
            Arc::new(Cylinder::new(thickness * 2.0, thickness)),
            position + axis * (width / 2.0 * side),
        ))
    };
    let bosses: Arc<dyn Sdf> = Arc::new(Union::new(end_boss(1.0), end_boss(-1.0)));
    let hinge: Arc<dyn Sdf>  = Arc::new(Union::new(strip, bosses));

    // Both parts share the hinge addition; neither needs a void
    let delta_a = JointDelta::new(Arc::clone(&hinge), Arc::new(NullSdf));
    let delta_b = JointDelta::new(hinge, Arc::new(NullSdf));
    (delta_a, delta_b)
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::primitives::SdfBox;

    #[test]
    fn dovetail_male_protrudes() {
        let (delta_a, _delta_b) = dovetail_joint(
            20.0, 8.0, 5.0, 15.0, 0.15, Vec3::ZERO, Vec3::Z,
        );
        // Male protrudes in +Z from origin. At (0,0,2.5) we should be inside.
        let d = delta_a.addition.distance(Vec3::new(0.0, 0.0, 2.5));
        assert!(d < 0.0, "Male dovetail centre should be solid, got {}", d);
    }

    #[test]
    fn dovetail_clearance_gap() {
        let clearance = 0.15_f32;
        let (_, delta_b) = dovetail_joint(
            20.0, 8.0, 5.0, 15.0, clearance, Vec3::ZERO, Vec3::Z,
        );
        // The female void should extend in -Z from origin. At (0,0,-2.5) we should be inside the void.
        let d = delta_b.void.distance(Vec3::new(0.0, 0.0, -2.5));
        assert!(d < 0.0, "Female socket centre should be inside void, got {}", d);

        // The male (avg_w=width+taper) at (5,0,-2.5) should be OUTSIDE the female void (gap).
        // avg_w = 8 + 5*tan(15°) ≈ 8 + 1.34 = 9.34, so male half_w ≈ 4.67.
        // Female half_w = (9.34 + 0.15)/2 ≈ 4.75.
        // At x=4.8: outside female (x > female_hw), confirming clearance.
        let d_gap = delta_b.void.distance(Vec3::new(4.8, 0.0, -2.5));
        assert!(d_gap > 0.0, "Point at gap width should be outside void, got {}", d_gap);
    }

    #[test]
    fn press_fit_pin_protrudes() {
        let (delta_a, delta_b) = press_fit(2.0, 6.0, 6.0, 0.1, Vec3::ZERO, Vec3::Z);
        // Pin at (0,0,3): inside pin
        assert!(delta_a.addition.distance(Vec3::new(0.0, 0.0, 3.0)) < 0.0);
        // Socket at (0,0,-3): inside socket void
        assert!(delta_b.void.distance(Vec3::new(0.0, 0.0, -3.0)) < 0.0);
    }

    #[test]
    fn finger_joint_alternates() {
        let (delta_a, delta_b) = finger_joint(
            10.0, 3.0, 4.0, 3, 0.1, Vec3::ZERO, Vec3::Z,
        );
        // With 3 fingers: fingers 0,2 go to part_a; finger 1 goes to part_b.
        // Finger 0 centre: x = -total/2 + 0.5*3 = -4.5 + 1.5 = -3.0
        // Finger 1 centre: x = -4.5 + 4.5 = 0.0
        // Finger 2 centre: x = -4.5 + 7.5 = 3.0
        // Part_a addition at (-3, 0, 2): should be solid (finger 0 → part_a)
        assert!(delta_a.addition.distance(Vec3::new(-3.0, 0.0, 2.0)) < 0.0,
            "Part_a finger at x=-3 should be solid");
        // Part_b addition at (0, 0, 2): should be solid (finger 1 → part_b)
        assert!(delta_b.addition.distance(Vec3::new(0.0, 0.0, 2.0)) < 0.0,
            "Part_b finger at x=0 should be solid");
    }

    #[test]
    fn delta_apply_no_op_null() {
        let sphere: Arc<dyn Sdf> = Arc::new(crate::sdf::primitives::Sphere::new(10.0));
        let delta = JointDelta::null();
        let result = delta.apply(Arc::clone(&sphere));
        // NullSdf void has no effect; NullSdf addition has no effect.
        // Result at origin should be same as original sphere (-10).
        let d_orig   = sphere.distance(Vec3::ZERO);
        let d_result = result.distance(Vec3::ZERO);
        assert!((d_orig - d_result).abs() < 0.01, "Null delta should not change sphere");
    }
}
