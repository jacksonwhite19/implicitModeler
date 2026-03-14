// Wing lofting with geometric parameters

use glam::{Vec2, Vec3};
use std::sync::Arc;
use crate::sdf::Sdf;
use super::airfoil::get_naca_airfoil;
use super::section::Section2D;

/// Wing section at a specific span location
#[derive(Clone)]
pub struct WingSection {
    pub section: Arc<dyn Section2D>,
    pub position: Vec3,  // Leading edge position
    pub twist: f32,      // Twist angle in radians
}

/// Lofted wing created from cross-section slices
pub struct LoftedWing {
    pub sections: Vec<WingSection>,
}

impl LoftedWing {
    pub fn new(sections: Vec<WingSection>) -> Self {
        Self { sections }
    }
}

impl Sdf for LoftedWing {
    fn distance(&self, point: Vec3) -> f32 {
        if self.sections.len() < 2 {
            return point.length();
        }

        // Span is along Y; cross-section lives in the XZ plane.
        let span_pos = point.y;

        let y_min = self.sections.iter().map(|s| s.position.y).fold(f32::MAX, f32::min);
        let y_max = self.sections.iter().map(|s| s.position.y).fold(f32::MIN, f32::max);

        let d_span = (y_min - span_pos).max(span_pos - y_max);
        let y_clamped = span_pos.clamp(y_min, y_max);

        // Find segment pair that brackets y_clamped
        let n = self.sections.len();
        let mut seg_idx = n - 2;
        let mut t = 1.0f32;
        for i in 0..n - 1 {
            let ya = self.sections[i].position.y;
            let yb = self.sections[i + 1].position.y;
            let span = yb - ya;
            if span.abs() < 1e-6 { continue; }
            let t_raw = (y_clamped - ya) / span;
            if t_raw >= 0.0 && t_raw <= 1.0 {
                seg_idx = i;
                t = t_raw;
                break;
            }
        }

        let sec_a = &self.sections[seg_idx];
        let sec_b = &self.sections[seg_idx + 1];

        // Interpolate leading-edge position and twist at y_clamped
        let lerp_x     = sec_a.position.x + t * (sec_b.position.x - sec_a.position.x);
        let lerp_z     = sec_a.position.z + t * (sec_b.position.z - sec_a.position.z);
        let lerp_twist = sec_a.twist      + t * (sec_b.twist      - sec_a.twist);

        // Cross-section is in XZ plane; X = chord direction, Z = thickness direction
        let local = Vec2::new(point.x - lerp_x, point.z - lerp_z);

        // Twist rotates in XZ plane
        let (sin_t, cos_t) = (-lerp_twist).sin_cos();
        let rotated = Vec2::new(
            local.x * cos_t - local.y * sin_t,
            local.x * sin_t + local.y * cos_t,
        );

        // True lofted cross-section: blend sec_a and sec_b shapes at parameter t.
        // distance_lerped_2d is allocation-free — critical for the realtime hot path.
        let dist_2d = sec_a.section.distance_lerped_2d(&*sec_b.section, t, rotated);

        let outside = Vec2::new(dist_2d.max(0.0), d_span.max(0.0)).length();
        let inside  = dist_2d.max(d_span).min(0.0);
        outside + inside
    }
}

/// Create a parametric wing with full geometric control
///
/// # Arguments
/// * `airfoil_designation` - NACA code (e.g., "2412")
/// * `root_chord` - Chord length at wing root
/// * `tip_chord` - Chord length at wing tip
/// * `span` - Total wing span (tip to tip)
/// * `sweep` - Leading edge sweep angle in degrees (positive = swept back)
/// * `dihedral` - Dihedral angle in degrees (positive = tips higher than root)
/// * `twist` - Tip twist relative to root in degrees (negative = washout)
pub fn wing_with_airfoil(
    airfoil_designation: &str,
    root_chord: f32,
    tip_chord: f32,
    span: f32,
    sweep: f32,
    dihedral: f32,
    twist: f32,
) -> LoftedWing {
    let root_airfoil = get_naca_airfoil(airfoil_designation, root_chord);
    let tip_airfoil  = get_naca_airfoil(airfoil_designation, tip_chord);

    let sweep_rad    = sweep.to_radians();
    let dihedral_rad = dihedral.to_radians();
    let twist_rad    = twist.to_radians();
    let half_span    = span / 2.0;

    let root_section = WingSection {
        section:  root_airfoil,
        position: Vec3::ZERO,
        twist:    0.0,
    };

    // Span along Y; sweep moves leading edge back in X; dihedral lifts tips in Z.
    let tip_x = half_span * sweep_rad.tan();
    let tip_y = half_span * dihedral_rad.cos();
    let tip_z = half_span * dihedral_rad.sin();

    let tip_section = WingSection {
        section:  tip_airfoil,
        position: Vec3::new(tip_x, tip_y, tip_z),
        twist:    twist_rad,
    };

    // Mirror tip in Y for the opposite semi-span
    let neg_tip_section = WingSection {
        section:  tip_section.section.clone(),
        position: Vec3::new(tip_x, -tip_y, tip_z),
        twist:    tip_section.twist,
    };

    // Sections ordered by Y: neg_tip → root → pos_tip
    LoftedWing::new(vec![neg_tip_section, root_section, tip_section])
}

/// Create a wing from pre-built Section2D root and tip cross-sections.
///
/// The chord length is encoded in the sections themselves (e.g. via `Airfoil::chord`).
/// Use this overload when you have custom airfoil data loaded from points or a .dat file.
///
/// # Arguments
/// * `root_section` - Cross-section at the wing root; chord is baked in.
/// * `tip_section`  - Cross-section at the wing tip; chord is baked in.
/// * `span`         - Total wing span (tip to tip)
/// * `sweep`        - Leading edge sweep angle in degrees
/// * `dihedral`     - Dihedral angle in degrees
/// * `twist`        - Tip twist relative to root in degrees
pub fn wing_from_sections(
    root_section: Arc<dyn Section2D>,
    tip_section: Arc<dyn Section2D>,
    span: f32,
    sweep: f32,
    dihedral: f32,
    twist: f32,
) -> LoftedWing {
    let sweep_rad    = sweep.to_radians();
    let dihedral_rad = dihedral.to_radians();
    let twist_rad    = twist.to_radians();
    let half_span    = span / 2.0;

    let root = WingSection {
        section:  root_section,
        position: Vec3::ZERO,
        twist:    0.0,
    };

    let tip_x = half_span * sweep_rad.tan();
    let tip_y = half_span * dihedral_rad.cos();
    let tip_z = half_span * dihedral_rad.sin();

    let tip = WingSection {
        section:  tip_section.clone(),
        position: Vec3::new(tip_x, tip_y, tip_z),
        twist:    twist_rad,
    };

    let neg_tip = WingSection {
        section:  tip_section,
        position: Vec3::new(tip_x, -tip_y, tip_z),
        twist:    twist_rad,
    };

    LoftedWing::new(vec![neg_tip, root, tip])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_straight_wing() {
        let wing = wing_with_airfoil("0012", 10.0, 5.0, 30.0, 0.0, 0.0, 0.0);

        assert_eq!(wing.sections[1].position, Vec3::ZERO);
        assert!((wing.sections[0].position.y + 15.0).abs() < 0.01);
        assert!((wing.sections[2].position.y - 15.0).abs() < 0.01);
        assert!(wing.sections[2].position.x.abs() < 0.01);
        assert!(wing.sections[2].position.z.abs() < 0.01);
    }

    #[test]
    fn test_swept_wing() {
        let wing = wing_with_airfoil("0012", 10.0, 5.0, 30.0, 30.0, 0.0, 0.0);

        let tip = &wing.sections[2];
        assert!(tip.position.x > 0.0, "Swept wing should have positive X offset");

        let expected_x = (30.0_f32.to_radians()).tan() * 15.0;
        assert!((tip.position.x - expected_x).abs() < 0.1);
    }

    #[test]
    fn test_dihedral() {
        let wing = wing_with_airfoil("0012", 10.0, 5.0, 30.0, 0.0, 5.0, 0.0);

        let tip = &wing.sections[2];
        assert!(tip.position.z > 0.0, "Dihedral wing tip should have positive Z offset");

        let expected_z = (5.0_f32.to_radians()).sin() * 15.0;
        assert!((tip.position.z - expected_z).abs() < 0.1);
    }

    #[test]
    fn test_twist() {
        let wing = wing_with_airfoil("0012", 10.0, 5.0, 30.0, 0.0, 0.0, -3.0);

        let tip = &wing.sections[2];
        let expected_twist = (-3.0_f32).to_radians();
        assert!((tip.twist - expected_twist).abs() < 0.01);
    }

    #[test]
    fn test_wing_sdf_evaluation() {
        let wing = wing_with_airfoil("0012", 10.0, 5.0, 30.0, 0.0, 0.0, 0.0);

        let _ = wing.distance(Vec3::new(5.0, 0.0, 0.0));

        let dist_outside = wing.distance(Vec3::new(100.0, 100.0, 100.0));
        assert!(dist_outside > 50.0, "Point far outside wing should have large positive distance");

        let dist_beyond = wing.distance(Vec3::new(5.0, 0.0, 50.0));
        assert!(dist_beyond > 10.0, "Point beyond span should be far outside");
    }

    #[test]
    fn test_lofted_interpolation_uses_both_sections() {
        // Root chord=10, tip chord=5. At mid-span (t=0.5), interpolated chord ≈ 7.5.
        // A point at x=9 (beyond tip chord, within root chord) should be outside
        // at mid-span but inside near the root.
        let wing = wing_with_airfoil("0012", 10.0, 5.0, 30.0, 0.0, 0.0, 0.0);

        let near_root = wing.distance(Vec3::new(9.0, 1.0, 0.0));   // y=1, near root
        let near_tip  = wing.distance(Vec3::new(9.0, 14.0, 0.0));  // y=14, near tip

        // Near root the chord is ~10 so x=9 should be within the airfoil silhouette
        assert!(near_root < near_tip,
            "x=9 should be closer to surface near root (larger chord) than near tip (smaller chord)");
    }
}

#[test]
fn test_wing_interior_points() {
    use crate::sdf::Sdf;
    let wing = wing_with_airfoil("2412", 12.0, 5.0, 40.0, 20.0, 3.0, -4.0);
    // Root section: chord=12, leading edge at (0,0,0), span at y=0
    // X=6 is 50% chord, Z=0 is near center, Y=0 is root
    let d_root = wing.distance(Vec3::new(6.0, 0.0, 0.0));
    println!("root mid-chord (6,0,0): {}", d_root);
    // X=10, Y=0: 83% chord at root — inside
    let d_root2 = wing.distance(Vec3::new(10.0, 0.0, 0.0));
    println!("root 83% chord (10,0,0): {}", d_root2);
    // Span-aligned: Y=10
    let d_mid = wing.distance(Vec3::new(5.0, 10.0, 0.5));
    println!("mid-span (5,10,0.5): {}", d_mid);
    assert!(d_root < 0.0, "root mid-chord should be inside, got {}", d_root);
    assert!(d_root2 < 0.0, "root 83% chord should be inside, got {}", d_root2);
}
