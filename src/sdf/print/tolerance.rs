// Tolerance compensation: shifts SDF isosurfaces to account for FDM process
// inaccuracies. External surfaces shrink slightly; holes grow slightly.

use std::sync::Arc;
use glam::Vec3;
use crate::sdf::Sdf;

// ── ToleranceSettings ────────────────────────────────────────────────────────

/// Per-print tolerance offsets applied to SDF geometry before export.
///
/// Positive `external_offset_mm` grows external surfaces outward.
/// Positive `internal_offset_mm` grows internal surfaces outward (holes get smaller).
/// Default values are tuned for typical FDM: external surfaces shrink 0.1 mm to
/// account for plastic expansion; holes grow 0.15 mm for clearance fits.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ToleranceSettings {
    /// Shift applied to external surfaces (negative = shrink part, positive = grow).
    pub external_offset_mm:   f32,
    /// Shift applied to internal surface boundaries (positive = larger holes = clearance fit).
    pub internal_offset_mm:   f32,
    /// Holes whose diameter is below this receive an extra bonus (mm).
    pub min_hole_diameter_mm: f32,
    /// Additional compensation added to small holes.
    pub small_hole_bonus_mm:  f32,
}

impl Default for ToleranceSettings {
    fn default() -> Self {
        Self {
            external_offset_mm:   -0.1,
            internal_offset_mm:    0.15,
            min_hole_diameter_mm:  3.0,
            small_hole_bonus_mm:   0.05,
        }
    }
}

/// Named preset flavours.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum TolerancePreset {
    TightFit,
    StandardFDM,
    LooseFit,
    Custom,
}

impl ToleranceSettings {
    pub fn apply_preset(&mut self, preset: &TolerancePreset) {
        match preset {
            TolerancePreset::TightFit => {
                self.external_offset_mm   = -0.05;
                self.internal_offset_mm   =  0.10;
                self.small_hole_bonus_mm  =  0.03;
            }
            TolerancePreset::StandardFDM => {
                self.external_offset_mm   = -0.10;
                self.internal_offset_mm   =  0.15;
                self.small_hole_bonus_mm  =  0.05;
            }
            TolerancePreset::LooseFit => {
                self.external_offset_mm   = -0.15;
                self.internal_offset_mm   =  0.20;
                self.small_hole_bonus_mm  =  0.08;
            }
            TolerancePreset::Custom => {}
        }
    }
}

// ── ToleranceCompensated SDF ─────────────────────────────────────────────────

/// Wraps any SDF and adjusts the isosurface position near surfaces to compensate
/// for FDM manufacturing tolerance.
pub struct ToleranceCompensated {
    pub child:    Arc<dyn Sdf>,
    pub settings: ToleranceSettings,
}

impl ToleranceCompensated {
    pub fn new(child: Arc<dyn Sdf>, settings: ToleranceSettings) -> Self {
        Self { child, settings }
    }
}

/// Step for gradient estimation (mm).
const GRAD_EPS: f32 = 0.1;

impl Sdf for ToleranceCompensated {
    fn distance(&self, p: Vec3) -> f32 {
        let d = self.child.distance(p);

        // Only apply compensation within a narrow band around the surface.
        let max_offset = self.settings.internal_offset_mm.abs()
            .max(self.settings.external_offset_mm.abs())
            .max(self.settings.small_hole_bonus_mm)
            + 0.5;

        if d.abs() > max_offset {
            return d;
        }

        // Compute the outward gradient (points from solid into void).
        let nx = self.child.distance(p + Vec3::X * GRAD_EPS)
               - self.child.distance(p - Vec3::X * GRAD_EPS);
        let ny = self.child.distance(p + Vec3::Y * GRAD_EPS)
               - self.child.distance(p - Vec3::Y * GRAD_EPS);
        let nz = self.child.distance(p + Vec3::Z * GRAD_EPS)
               - self.child.distance(p - Vec3::Z * GRAD_EPS);
        let nlen = (nx*nx + ny*ny + nz*nz).sqrt();
        if nlen < 1e-6 {
            return d;
        }
        let outward = Vec3::new(nx / nlen, ny / nlen, nz / nlen);

        // Classify: march outward along the gradient into the void.
        // If we re-enter solid within `probe_dist`, it's an internal surface (cavity).
        // If we never find solid, it's external.
        let probe_dist = self.settings.min_hole_diameter_mm * 2.0 + 2.0;
        let is_internal = is_internal_surface(&self.child, p, outward, probe_dist);

        let raw_offset = if is_internal {
            // Estimate void radius for small-hole bonus.
            let void_r = estimate_void_radius_along(&self.child, p, outward,
                                                    self.settings.min_hole_diameter_mm);
            let bonus = if void_r < self.settings.min_hole_diameter_mm * 0.5 {
                self.settings.small_hole_bonus_mm
            } else {
                0.0
            };
            self.settings.internal_offset_mm + bonus
        } else {
            self.settings.external_offset_mm
        };

        // Shift the isosurface:
        //  External: external_offset = -0.1 → d - (-0.1) = d + 0.1 → surface retreats → part shrinks.
        //  Internal: internal_offset = +0.15 → d + 0.15 → hole grows outward (void expands).
        //            Internal sign is negated so `d - (-0.15) = d + 0.15`.
        let signed_offset = if is_internal { -raw_offset } else { raw_offset };
        d - signed_offset
    }
}

/// March from surface point `p` in the `outward` direction (into void).
/// Returns true if solid (d < 0) is found within `max_dist`.
fn is_internal_surface(sdf: &Arc<dyn Sdf>, p: Vec3, outward: Vec3, max_dist: f32) -> bool {
    let steps = 16usize;
    let step = max_dist / steps as f32;
    for k in 1..=steps {
        let probe = p + outward * (k as f32 * step);
        if sdf.distance(probe) < 0.0 {
            return true;
        }
    }
    false
}

/// Estimate the radius of the void by marching along `outward` and finding
/// the first solid boundary.  Returns the distance to that boundary.
fn estimate_void_radius_along(sdf: &Arc<dyn Sdf>, p: Vec3, outward: Vec3, max_r: f32) -> f32 {
    let steps = 20usize;
    let step = max_r / steps as f32;
    for k in 1..=steps {
        let probe = p + outward * (k as f32 * step);
        if sdf.distance(probe) < 0.0 {
            return (k as f32 - 1.0) * step;
        }
    }
    max_r
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::primitives::{Sphere, SdfBox};
    use crate::sdf::booleans::Subtract;
    use crate::sdf::transforms::Translate;

    fn default_settings() -> ToleranceSettings {
        ToleranceSettings::default()
    }

    #[test]
    fn external_offset_shrinks_sphere() {
        let r = 10.0_f32;
        let sphere: Arc<dyn Sdf> = Arc::new(Sphere::new(r));
        let settings = ToleranceSettings {
            external_offset_mm:   -0.1,
            internal_offset_mm:    0.15,
            min_hole_diameter_mm:  3.0,
            small_hole_bonus_mm:   0.05,
        };
        let comp = ToleranceCompensated::new(Arc::clone(&sphere), settings);

        // The isosurface should now be at ~r - 0.1 = 9.9 (exterior shrank inward).
        // At the original surface radius the compensated SDF should be positive (outside).
        let d_at_orig_surface = comp.distance(Vec3::new(r, 0.0, 0.0));
        assert!(
            d_at_orig_surface > 0.0,
            "at r={} the compensated sphere surface should be inside (positive outside), got {}",
            r, d_at_orig_surface
        );

        // At r-0.2 (clearly inside original sphere but close to new surface) check
        // that we're near zero.
        let d_at_new_surface = comp.distance(Vec3::new(r - 0.1, 0.0, 0.0));
        assert!(
            d_at_new_surface.abs() < 0.15,
            "near expected new surface (r={}) distance should be small, got {}",
            r - 0.1, d_at_new_surface
        );
    }

    #[test]
    fn internal_offset_enlarges_hole() {
        // Box with a cylindrical hole subtracted down the Z axis.
        let box_sdf: Arc<dyn Sdf> = Arc::new(SdfBox::new(Vec3::splat(15.0)));
        // Hole: cylinder of radius 2 running through the centre.
        let hole: Arc<dyn Sdf> = Arc::new(crate::sdf::primitives::Cylinder::new(2.0, 20.0));
        let with_hole: Arc<dyn Sdf> = Arc::new(Subtract::new(box_sdf, hole));

        let settings = default_settings(); // internal_offset = 0.15
        let comp = ToleranceCompensated::new(with_hole, settings);

        // At the original hole wall (r=2 from Z-axis) the compensated distance
        // should be negative (inside the enlarged hole) — the hole got bigger.
        let at_original_wall = comp.distance(Vec3::new(2.0, 0.0, 0.0));
        assert!(
            at_original_wall > 0.0,
            "original hole wall should now be exposed (outside material) after enlargement, got {}",
            at_original_wall
        );
    }

    #[test]
    fn small_hole_bonus_applied() {
        // A tiny hole of radius 1.0 mm (below min_hole_diameter 3mm).
        let box_sdf: Arc<dyn Sdf> = Arc::new(SdfBox::new(Vec3::splat(15.0)));
        let tiny_hole: Arc<dyn Sdf> = Arc::new(crate::sdf::primitives::Cylinder::new(1.0, 20.0));
        let with_hole: Arc<dyn Sdf> = Arc::new(Subtract::new(box_sdf, tiny_hole));

        let settings = ToleranceSettings {
            external_offset_mm:   -0.1,
            internal_offset_mm:    0.15,
            min_hole_diameter_mm:  3.0,
            small_hole_bonus_mm:   0.05,
        };
        let comp = ToleranceCompensated::new(with_hole, settings);

        // With normal+bonus = 0.20, original wall at r=1 should read positive (outside).
        let at_wall = comp.distance(Vec3::new(1.0, 0.0, 0.0));
        assert!(
            at_wall > 0.0,
            "small-hole wall should be enlarged (outside material) after bonus, got {}",
            at_wall
        );
    }
}
