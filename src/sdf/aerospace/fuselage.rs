// Fuselage lofting with cross-section interpolation

use glam::{Vec2, Vec3};
use std::any::Any;
use std::sync::Arc;
use crate::sdf::Sdf;
use crate::sdf::spine::LongitudinalSplines;
use super::section::Section2D;

/// Cross-section shape types for fuselage lofting.
/// Implements `Section2D` so custom shapes can substitute it anywhere.
#[derive(Clone)]
pub enum CrossSection {
    Circle { radius: f32 },
    Ellipse { width: f32, height: f32 },
}

impl CrossSection {
    /// Linearly interpolate toward `other`.
    fn lerp(&self, other: &CrossSection, t: f32) -> CrossSection {
        match (self, other) {
            (CrossSection::Circle { radius: r1 }, CrossSection::Circle { radius: r2 }) => {
                CrossSection::Circle { radius: r1 + (r2 - r1) * t }
            }
            (CrossSection::Ellipse { width: w1, height: h1 },
             CrossSection::Ellipse { width: w2, height: h2 }) => {
                CrossSection::Ellipse {
                    width:  w1 + (w2 - w1) * t,
                    height: h1 + (h2 - h1) * t,
                }
            }
            // Mixed variants: treat Circle as equal-axis Ellipse
            _ => {
                let (w1, h1) = match self {
                    CrossSection::Circle { radius: r } => (*r, *r),
                    CrossSection::Ellipse { width, height } => (*width, *height),
                };
                let (w2, h2) = match other {
                    CrossSection::Circle { radius: r } => (*r, *r),
                    CrossSection::Ellipse { width, height } => (*width, *height),
                };
                CrossSection::Ellipse {
                    width:  w1 + (w2 - w1) * t,
                    height: h1 + (h2 - h1) * t,
                }
            }
        }
    }
}

impl Section2D for CrossSection {
    fn distance_2d(&self, point: Vec2) -> f32 {
        match self {
            CrossSection::Circle { radius } => point.length() - radius,
            CrossSection::Ellipse { width, height } => {
                // Approximate ellipse SDF (exact form is expensive; good enough for MC)
                let p = point / Vec2::new(*width, *height);
                (p.length() - 1.0) * width.min(*height)
            }
        }
    }

    fn lerp_to(&self, other: &dyn Section2D, t: f32) -> Arc<dyn Section2D> {
        if let Some(other_cs) = other.as_any().downcast_ref::<CrossSection>() {
            Arc::new(self.lerp(other_cs, t))
        } else {
            Arc::new(self.clone())
        }
    }

    fn as_any(&self) -> &dyn Any { self }
}

/// Fuselage section at a specific station
#[derive(Clone)]
pub struct FuselageSection {
    cross_section: Arc<dyn Section2D>,
    position: f32,        // Normalized position along fuselage axis [0, 1]
    center_offset: Vec2,  // Offset for asymmetric fuselages
    /// Z of the Keel role-point in the reference profile (None if no Keel labelled).
    ref_keel_z: Option<f32>,
    /// Z of the Deck role-point in the reference profile.
    ref_deck_z: Option<f32>,
    /// Y (lateral) of the Chine role-point in the reference profile.
    ref_chine_y: Option<f32>,
}

/// Lofted fuselage from cross-sections
pub struct LoftedFuselage {
    sections: Vec<FuselageSection>,
    length: f32,
    /// Optional longitudinal spine constraints.  When present, the cross-section
    /// query point is non-uniformly scaled at each X station so that the labelled
    /// Keel/Deck/Chine points follow their respective curves.
    splines: Option<Arc<LongitudinalSplines>>,
}

impl LoftedFuselage {
    pub fn new(sections: Vec<FuselageSection>, length: f32) -> Self {
        Self { sections, length, splines: None }
    }

    /// Build a fuselage from user-supplied (position, Section2D) pairs.
    /// Stations are sorted by position automatically — no pre-sorted order required.
    /// `center_offset` is set to zero for all stations; the full `new()` constructor
    /// remains available for asymmetric fuselages built inside this module.
    pub fn from_stations(mut stations: Vec<(f32, Arc<dyn Section2D>)>, length: f32) -> Self {
        stations.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        let sections = stations
            .into_iter()
            .map(|(pos, cs)| FuselageSection {
                cross_section: cs,
                position: pos,
                center_offset: Vec2::ZERO,
                ref_keel_z: None,
                ref_deck_z: None,
                ref_chine_y: None,
            })
            .collect();
        Self { sections, length, splines: None }
    }

    /// Build a fuselage from (position, Section2D, ref_keel_z, ref_deck_z,
    /// ref_chine_y) tuples and attach a `LongitudinalSplines` constraint.
    pub fn from_stations_with_splines(
        mut stations: Vec<(f32, Arc<dyn Section2D>, Option<f32>, Option<f32>, Option<f32>)>,
        length: f32,
        splines: Arc<LongitudinalSplines>,
    ) -> Self {
        stations.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        let sections = stations
            .into_iter()
            .map(|(pos, cs, keel, deck, chine)| FuselageSection {
                cross_section: cs,
                position: pos,
                center_offset: Vec2::ZERO,
                ref_keel_z:  keel,
                ref_deck_z:  deck,
                ref_chine_y: chine,
            })
            .collect();
        Self { sections, length, splines: Some(splines) }
    }

    /// Find which section pair contains the given normalized position
    fn find_section_pair(&self, normalized_pos: f32) -> (usize, f32) {
        for i in 0..self.sections.len() - 1 {
            let p1 = self.sections[i].position;
            let p2 = self.sections[i + 1].position;

            if normalized_pos >= p1 && normalized_pos <= p2 {
                let t = if (p2 - p1).abs() < 1e-6 {
                    0.0
                } else {
                    (normalized_pos - p1) / (p2 - p1)
                };
                return (i, t.clamp(0.0, 1.0));
            }
        }

        if normalized_pos < self.sections[0].position {
            (0, 0.0)
        } else {
            (self.sections.len() - 2, 1.0)
        }
    }
}

impl Sdf for LoftedFuselage {
    fn distance(&self, point: Vec3) -> f32 {
        if self.sections.is_empty() {
            return point.length();
        }

        // Fuselage axis is along X; normalize to [0, 1]
        let normalized_x = point.x / self.length;

        let axial_dist = if normalized_x < 0.0 {
            -normalized_x * self.length
        } else if normalized_x > 1.0 {
            (normalized_x - 1.0) * self.length
        } else {
            0.0
        };

        let (idx, t) = self.find_section_pair(normalized_x);
        let section1 = &self.sections[idx];
        let section2 = &self.sections[idx + 1];

        let interp_section = section1.cross_section.lerp_to(&*section2.cross_section, t);
        let interp_offset = section1.center_offset * (1.0 - t) + section2.center_offset * t;

        // Interpolate reference role positions for the spine constraint.
        let lerp_opt = |a: Option<f32>, b: Option<f32>| -> Option<f32> {
            match (a, b) { (Some(va), Some(vb)) => Some(va + (vb - va) * t), _ => a.or(b) }
        };
        let ref_keel  = lerp_opt(section1.ref_keel_z,  section2.ref_keel_z);
        let ref_deck  = lerp_opt(section1.ref_deck_z,  section2.ref_deck_z);
        let ref_chine = lerp_opt(section1.ref_chine_y, section2.ref_chine_y);

        let mut p_2d = Vec2::new(point.y, point.z) - interp_offset;

        // Apply longitudinal spine deformation when constraints are present.
        if let Some(splines) = &self.splines {
            let xw = normalized_x * self.length; // world-space X
            let tf = splines.section_transform(
                xw,
                ref_keel.unwrap_or(-1.0),
                ref_deck.unwrap_or( 1.0),
                ref_chine.unwrap_or(1.0),
            );
            if !tf.is_identity() {
                p_2d = tf.apply(p_2d);
            }
        }

        let radial_dist = interp_section.distance_2d(p_2d);

        if axial_dist > 0.0 {
            (radial_dist.max(0.0).powi(2) + axial_dist.powi(2)).sqrt() + radial_dist.min(0.0)
        } else {
            radial_dist
        }
    }
}

/// Create a parametric fuselage with nose and tail shaping
///
/// # Arguments
/// * `length` - Total fuselage length
/// * `max_diameter` - Maximum diameter (at widest point)
/// * `nose_shape` - Nose shape factor: 0.0 = pointed, 1.0 = rounded
/// * `tail_shape` - Tail shape factor: 0.0 = pointed, 1.0 = rounded
pub fn fuselage_parametric(
    length: f32,
    max_diameter: f32,
    nose_shape: f32,
    tail_shape: f32,
) -> LoftedFuselage {
    let max_radius = max_diameter / 2.0;

    let nose_length = length * 0.2;
    let tail_length = length * 0.15;
    let mid_length = length - nose_length - tail_length;

    let cs = |radius: f32| -> Arc<dyn Section2D> {
        Arc::new(CrossSection::Circle { radius })
    };

    let mk = |cs: Arc<dyn Section2D>, pos: f32| FuselageSection {
        cross_section: cs,
        position: pos,
        center_offset: Vec2::ZERO,
        ref_keel_z: None,
        ref_deck_z: None,
        ref_chine_y: None,
    };

    let sections = vec![
        mk(cs(max_radius * (0.05 + 0.2 * nose_shape)), 0.0),
        mk(cs(max_radius * (0.6  + 0.4 * nose_shape)), nose_length / length * 0.5),
        mk(cs(max_radius),                             nose_length / length),
        mk(cs(max_radius),                             (nose_length + mid_length) / length),
        mk(cs(max_radius * (0.7  + 0.2 * tail_shape)), (nose_length + mid_length + tail_length * 0.5) / length),
        mk(cs(max_radius * (0.1  + 0.3 * tail_shape)), 1.0),
    ];

    LoftedFuselage::new(sections, length)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::aerospace::section::Section2D;

    #[test]
    fn test_circle_cross_section() {
        let circle = CrossSection::Circle { radius: 5.0 };

        let dist_center = circle.distance_2d(Vec2::ZERO);
        assert!((dist_center + 5.0).abs() < 0.01, "Center should be -radius");

        let dist_surface = circle.distance_2d(Vec2::new(5.0, 0.0));
        assert!(dist_surface.abs() < 0.01, "Surface point should be ~0");

        let dist_outside = circle.distance_2d(Vec2::new(10.0, 0.0));
        assert!((dist_outside - 5.0).abs() < 0.01, "Outside point should be positive");
    }

    #[test]
    fn test_cross_section_lerp_to() {
        let a = CrossSection::Circle { radius: 4.0 };
        let b = CrossSection::Circle { radius: 8.0 };
        let mid = a.lerp_to(&b, 0.5);
        // At radius 6.0 from center, should be on the surface
        let d = mid.distance_2d(Vec2::new(6.0, 0.0));
        assert!(d.abs() < 0.1, "Lerped circle radius should be 6");
    }

    #[test]
    fn test_fuselage_parametric() {
        let fuselage = fuselage_parametric(100.0, 10.0, 0.5, 0.5);
        assert_eq!(fuselage.sections.len(), 6);
        assert!((fuselage.length - 100.0).abs() < 0.01);
    }

    #[test]
    fn test_fuselage_sdf() {
        let fuselage = fuselage_parametric(100.0, 10.0, 0.5, 0.5);

        let dist_center = fuselage.distance(Vec3::new(50.0, 0.0, 0.0));
        assert!(dist_center < 0.0, "Center point should be inside");

        let dist_outside = fuselage.distance(Vec3::new(50.0, 20.0, 20.0));
        assert!(dist_outside > 0.0, "Point far outside should be positive");

        let dist_beyond = fuselage.distance(Vec3::new(150.0, 0.0, 0.0));
        assert!(dist_beyond > 0.0, "Point beyond fuselage should be positive");
    }
}
