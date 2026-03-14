// Longitudinal spine constraint system for station-based fuselage lofting.
//
// The user labels cross-section control points with roles (Keel, Deck, Chine)
// and then draws longitudinal curves in the XZ plane that specify how those
// structural points evolve along the fuselage span (X axis).
//
// At evaluation time the cross-section query point is non-uniformly scaled so
// that the Keel, Deck, and Chine points land exactly on their respective curves.

use serde::{Deserialize, Serialize};

// ── AxisCurve ────────────────────────────────────────────────────────────────

/// A piecewise Catmull-Rom spline that maps a span position (X) to a single
/// value (typically Z or Y).  Control points must be supplied in ascending X
/// order.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct AxisCurve {
    /// Sorted control points: each entry is `[x, value]`.
    pub control_points: Vec<[f32; 2]>,
}

impl AxisCurve {
    pub fn new(control_points: Vec<[f32; 2]>) -> Self {
        let mut pts = control_points;
        pts.sort_by(|a, b| a[0].partial_cmp(&b[0]).unwrap_or(std::cmp::Ordering::Equal));
        Self { control_points: pts }
    }

    /// Evaluate the curve at span position `x`.  Returns `None` if there are
    /// fewer than two control points.  Clamps to the first / last value outside
    /// the defined range.
    pub fn eval(&self, x: f32) -> Option<f32> {
        let pts = &self.control_points;
        let n = pts.len();
        if n == 0 { return None; }
        if n == 1 { return Some(pts[0][1]); }

        // Clamp outside range
        if x <= pts[0][0]         { return Some(pts[0][1]); }
        if x >= pts[n - 1][0]     { return Some(pts[n - 1][1]); }

        // Find the surrounding segment
        let seg = pts.windows(2).position(|w| w[1][0] >= x).unwrap_or(n - 2);
        let x0 = pts[seg][0];
        let x1 = pts[seg + 1][0];
        let t = if (x1 - x0).abs() < 1e-10 { 0.0 } else { (x - x0) / (x1 - x0) };

        // Catmull-Rom ghost points at the ends
        let p0 = if seg == 0     { pts[0][1]     } else { pts[seg - 1][1] };
        let p1 = pts[seg][1];
        let p2 = pts[seg + 1][1];
        let p3 = if seg + 2 < n  { pts[seg + 2][1] } else { pts[n - 1][1] };

        Some(catmull_rom_scalar(p0, p1, p2, p3, t))
    }
}

fn catmull_rom_scalar(p0: f32, p1: f32, p2: f32, p3: f32, t: f32) -> f32 {
    let t2 = t * t;
    let t3 = t2 * t;
    0.5 * ((2.0 * p1)
         + (-p0 + p2) * t
         + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
         + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3)
}

// ── Constraint curves ────────────────────────────────────────────────────────

/// Constraints for the Keel and Deck structural lines.
///
/// Both curves live in the XZ plane and supply the Z (vertical) position of the
/// bottommost (keel) and topmost (deck) points of the cross-section at any
/// given span station X.
#[derive(Clone, Debug, Serialize, Deserialize, Default, PartialEq)]
pub struct SpineConstraint {
    /// Z position of the keel (bottommost point) along the span.
    pub keel: Option<AxisCurve>,
    /// Z position of the deck (topmost point) along the span.
    pub deck: Option<AxisCurve>,
}

/// Constraints for the Chine structural line (widest lateral point).
#[derive(Clone, Debug, Serialize, Deserialize, Default, PartialEq)]
pub struct ChineConstraint {
    /// Z position of the chine along the span (vertical locking).
    pub chine_z: Option<AxisCurve>,
    /// Y (lateral half-width) of the chine along the span.
    pub chine_y: Option<AxisCurve>,
}

// ── LongitudinalSplines ───────────────────────────────────────────────────────

/// All longitudinal spine curves for a fuselage.
///
/// These are derived from the cross-section profiles' role-labelled control
/// points and edited in the longitudinal spine editor (XZ plane view).
#[derive(Clone, Debug, Serialize, Deserialize, Default, PartialEq)]
pub struct LongitudinalSplines {
    pub spine: SpineConstraint,
    pub chine: ChineConstraint,
}

impl LongitudinalSplines {
    /// Compute the transform parameters needed to deform a cross-section so
    /// that its Keel, Deck, and Chine points satisfy the longitudinal curves
    /// at span position `x`.
    ///
    /// Returns `SectionTransform` with scale and offset values for the YZ
    /// cross-section query point.  If no relevant curve is active the transform
    /// is identity.
    ///
    /// `ref_keel_z`   — Z of the Keel point in the *reference* (undeformed) profile.
    /// `ref_deck_z`   — Z of the Deck point in the reference profile.
    /// `ref_chine_y`  — Y (lateral) of the Chine point in the reference profile.
    pub fn section_transform(
        &self,
        x: f32,
        ref_keel_z: f32,
        ref_deck_z: f32,
        ref_chine_y: f32,
    ) -> SectionTransform {
        let mut out = SectionTransform::identity();

        let have_keel = self.spine.keel.as_ref().and_then(|c| c.eval(x));
        let have_deck = self.spine.deck.as_ref().and_then(|c| c.eval(x));

        if let (Some(keel_z), Some(deck_z)) = (have_keel, have_deck) {
            let ref_height = (ref_deck_z - ref_keel_z).abs();
            let new_height = (deck_z - keel_z).abs();
            if ref_height > 1e-6 && new_height > 1e-6 {
                let scale_z = new_height / ref_height;
                let ref_center_z = (ref_keel_z + ref_deck_z) * 0.5;
                let new_center_z = (keel_z + deck_z) * 0.5;
                out.scale_z  = scale_z;
                out.offset_z = new_center_z - ref_center_z * scale_z;
            }
        } else if let Some(keel_z) = have_keel {
            // Only keel — shift the whole section vertically
            out.offset_z = keel_z - ref_keel_z;
        } else if let Some(deck_z) = have_deck {
            out.offset_z = deck_z - ref_deck_z;
        }

        if let Some(chine_y) = self.chine.chine_y.as_ref().and_then(|c| c.eval(x)) {
            if ref_chine_y.abs() > 1e-6 {
                out.scale_y = chine_y / ref_chine_y;
            }
        }

        if let Some(chine_z) = self.chine.chine_z.as_ref().and_then(|c| c.eval(x)) {
            // Shift the chine vertically — adjust offset_z to match.
            // This is a rough approximation (assumes chine is near the vertical centre).
            let ref_chine_z = ref_keel_z + (ref_deck_z - ref_keel_z) * 0.5;
            out.offset_z += chine_z - ref_chine_z;
        }

        out
    }
}

// ── SectionTransform ─────────────────────────────────────────────────────────

/// Non-uniform scale + offset applied to the YZ cross-section query point
/// before calling the profile SDF.
///
/// Transforms a world-space 2-D point `p` into profile-local space:
///   `p_local.y = (p.y - offset_y) / scale_y`
///   `p_local.z = (p.z - offset_z) / scale_z`
///
/// `scale_*` = 1.0 and `offset_*` = 0.0 → identity (no constraint applied).
#[derive(Clone, Copy, Debug)]
pub struct SectionTransform {
    pub scale_y:  f32,
    pub scale_z:  f32,
    pub offset_y: f32,
    pub offset_z: f32,
}

impl SectionTransform {
    pub fn identity() -> Self {
        Self { scale_y: 1.0, scale_z: 1.0, offset_y: 0.0, offset_z: 0.0 }
    }

    /// Apply to a 2-D cross-section point `[y, z]`.
    pub fn apply(&self, p: glam::Vec2) -> glam::Vec2 {
        glam::Vec2::new(
            (p.x - self.offset_y) / self.scale_y,
            (p.y - self.offset_z) / self.scale_z,
        )
    }

    /// True when this is effectively the identity transform.
    pub fn is_identity(&self) -> bool {
        (self.scale_y  - 1.0).abs() < 1e-6
            && (self.scale_z  - 1.0).abs() < 1e-6
            && self.offset_y.abs() < 1e-6
            && self.offset_z.abs() < 1e-6
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_axis_curve_linear() {
        let c = AxisCurve::new(vec![[0.0, 0.0], [1.0, 10.0]]);
        let v = c.eval(0.5).unwrap();
        assert!((v - 5.0).abs() < 0.5, "mid-point should be ~5, got {}", v);
    }

    #[test]
    fn test_axis_curve_clamp() {
        let c = AxisCurve::new(vec![[0.0, 3.0], [1.0, 7.0]]);
        assert!((c.eval(-1.0).unwrap() - 3.0).abs() < 1e-4);
        assert!((c.eval(2.0).unwrap()  - 7.0).abs() < 1e-4);
    }

    #[test]
    fn test_section_transform_identity() {
        let splines = LongitudinalSplines::default();
        let t = splines.section_transform(0.5, -1.0, 1.0, 1.0);
        assert!(t.is_identity());
    }

    #[test]
    fn test_section_transform_keel_deck() {
        let mut splines = LongitudinalSplines::default();
        // Keel at z=-2, deck at z=2  (reference: keel=-1, deck=1)
        splines.spine.keel = Some(AxisCurve::new(vec![[0.0, -2.0], [1.0, -2.0]]));
        splines.spine.deck = Some(AxisCurve::new(vec![[0.0,  2.0], [1.0,  2.0]]));
        let t = splines.section_transform(0.5, -1.0, 1.0, 1.0);
        // scale_z should be 2: new_height=4, ref_height=2
        assert!((t.scale_z - 2.0).abs() < 1e-4, "scale_z={}", t.scale_z);
    }
}
