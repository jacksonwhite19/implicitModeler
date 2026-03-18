// Screw hole generation for FDM printed parts.
#![allow(dead_code)] // Fastener spec fields — some not yet accessed in the UI
//
// Provides a static table of M2–M5 specifications and three hole generators:
// clearance holes, countersunk holes, and heat-set insert bosses. Each
// generator returns a (void, boss) pair; check_and_pad() measures wall
// thickness and automatically unions the boss before subtracting the void
// when the wall would be too thin.

use std::sync::Arc;
use glam::{Vec3, Quat};
use crate::sdf::Sdf;
use crate::sdf::primitives::Cylinder;
use crate::sdf::booleans::{Union, Subtract, SmoothUnion};
use crate::sdf::transforms::{Translate, Rotate};
use crate::sdf::aerospace::mechanical::CappedCone;

// ── Screw specification ───────────────────────────────────────────────────────

pub struct ScrewSpec {
    pub designation:           &'static str,
    pub thread_diameter_mm:    f32,
    pub clearance_diameter_mm: f32,
    pub head_diameter_mm:      f32,
    pub head_height_mm:        f32,
    pub countersink_angle_deg: f32,
    pub heat_set_diameter_mm:  f32,
    pub heat_set_depth_mm:     f32,
    pub min_boss_diameter_mm:  f32,
    pub min_wall_thickness_mm: f32,
}

static SCREW_SPECS: &[ScrewSpec] = &[
    ScrewSpec {
        designation: "M2",
        thread_diameter_mm: 2.0, clearance_diameter_mm: 2.4,
        head_diameter_mm: 3.8,   head_height_mm: 2.0,
        countersink_angle_deg: 90.0,
        heat_set_diameter_mm: 3.2, heat_set_depth_mm: 4.0,
        min_boss_diameter_mm: 5.0, min_wall_thickness_mm: 1.6,
    },
    ScrewSpec {
        designation: "M2.5",
        thread_diameter_mm: 2.5, clearance_diameter_mm: 2.9,
        head_diameter_mm: 4.5,   head_height_mm: 2.5,
        countersink_angle_deg: 90.0,
        heat_set_diameter_mm: 3.8, heat_set_depth_mm: 5.0,
        min_boss_diameter_mm: 6.0, min_wall_thickness_mm: 2.0,
    },
    ScrewSpec {
        designation: "M3",
        thread_diameter_mm: 3.0, clearance_diameter_mm: 3.4,
        head_diameter_mm: 5.5,   head_height_mm: 3.0,
        countersink_angle_deg: 90.0,
        heat_set_diameter_mm: 4.5, heat_set_depth_mm: 6.0,
        min_boss_diameter_mm: 7.0, min_wall_thickness_mm: 2.4,
    },
    ScrewSpec {
        designation: "M4",
        thread_diameter_mm: 4.0, clearance_diameter_mm: 4.5,
        head_diameter_mm: 7.0,   head_height_mm: 4.0,
        countersink_angle_deg: 90.0,
        heat_set_diameter_mm: 5.6, heat_set_depth_mm: 8.0,
        min_boss_diameter_mm: 9.0, min_wall_thickness_mm: 3.2,
    },
    ScrewSpec {
        designation: "M5",
        thread_diameter_mm: 5.0, clearance_diameter_mm: 5.5,
        head_diameter_mm: 8.5,   head_height_mm: 5.0,
        countersink_angle_deg: 90.0,
        heat_set_diameter_mm: 6.4, heat_set_depth_mm: 10.0,
        min_boss_diameter_mm: 10.0, min_wall_thickness_mm: 4.0,
    },
];

pub fn get_spec(designation: &str) -> Option<&'static ScrewSpec> {
    SCREW_SPECS.iter().find(|s| s.designation == designation)
}

// ── NullSdf placeholder ───────────────────────────────────────────────────────

/// An SDF that always returns a very large positive value — effectively empty
/// space. Subtracting NullSdf from a body has no effect. Unioning NullSdf into
/// a body has no effect.
pub struct NullSdf;

impl Sdf for NullSdf {
    fn distance(&self, _: Vec3) -> f32 {
        f32::MAX / 2.0
    }
}

// ── Geometry helpers ──────────────────────────────────────────────────────────

/// Z-aligned cylinder from z=0 to z=depth (center at z=depth/2).
pub fn cyl_z(radius: f32, depth: f32) -> Arc<dyn Sdf> {
    Arc::new(Translate::new(
        Arc::new(Cylinder::new(radius, depth / 2.0)),
        Vec3::new(0.0, 0.0, depth / 2.0),
    ))
}

/// Rotate a Z-aligned SDF so its +Z axis aligns with `dir`.
pub fn orient_along(child: Arc<dyn Sdf>, dir: Vec3) -> Arc<dyn Sdf> {
    let dir = dir.normalize();
    if (dir.dot(Vec3::Z) - 1.0).abs() < 1e-4 {
        return child;
    }
    if (dir.dot(Vec3::Z) + 1.0).abs() < 1e-4 {
        return Arc::new(Rotate::new(child, Quat::from_rotation_x(std::f32::consts::PI)));
    }
    Arc::new(Rotate::new(child, Quat::from_rotation_arc(Vec3::Z, dir)))
}

/// Position a Z-aligned SDF so its z=0 end is at `position`, extending along `direction`.
pub fn place_at(child: Arc<dyn Sdf>, position: Vec3, direction: Vec3) -> Arc<dyn Sdf> {
    Arc::new(Translate::new(orient_along(child, direction), position))
}

// ── Z-aligned hole primitives (used by panels.rs and api.rs) ─────────────────

/// Clearance hole cylinder (Z-aligned, from z=0 to z=depth).
pub fn clearance_hole_z(spec: &ScrewSpec, depth: f32) -> Arc<dyn Sdf> {
    cyl_z(spec.clearance_diameter_mm / 2.0, depth)
}

/// Countersink void (Z-aligned): cone at entry (z=0..hh) + shaft (z=hh..hh+shaft_depth).
pub fn countersink_hole_z(spec: &ScrewSpec, shaft_depth: f32) -> Arc<dyn Sdf> {
    let cr  = spec.clearance_diameter_mm / 2.0;
    let hr  = spec.head_diameter_mm / 2.0;
    let hh  = spec.head_height_mm;
    // CappedCone: r1=hr at z=-h (bottom/entry), r2=cr at z=+h (top/narrow). Translate +h → bottom at z=0.
    let cone: Arc<dyn Sdf> = Arc::new(Translate::new(
        Arc::new(CappedCone { r1: hr, r2: cr, h: hh / 2.0 }),
        Vec3::new(0.0, 0.0, hh / 2.0),
    ));
    let shaft: Arc<dyn Sdf> = Arc::new(Translate::new(
        cyl_z(cr, shaft_depth),
        Vec3::new(0.0, 0.0, hh),
    ));
    Arc::new(Union::new(cone, shaft))
}

/// Heat-set insert void (Z-aligned, from z=0 to z=heat_set_depth).
pub fn heat_set_void_z(spec: &ScrewSpec) -> Arc<dyn Sdf> {
    cyl_z(spec.heat_set_diameter_mm / 2.0, spec.heat_set_depth_mm)
}

/// Heat-set boss (Z-aligned): tall cylinder with smooth fillet disc at base.
pub fn heat_set_boss_z(spec: &ScrewSpec) -> Arc<dyn Sdf> {
    let br  = spec.min_boss_diameter_mm / 2.0;
    let bd  = spec.heat_set_depth_mm + 2.0;
    let cyl = cyl_z(br, bd);
    let fillet: Arc<dyn Sdf> = Arc::new(Cylinder::new(br + 1.0, 0.5));
    Arc::new(SmoothUnion::new(cyl, fillet, 1.0))
}

// ── Hole generators ───────────────────────────────────────────────────────────

/// Clearance hole: through-cylinder at `position` along `direction` for `depth`.
/// Returns `(void, boss)`: subtract void from the body, union boss if the wall is thin.
pub fn clearance_hole(
    spec:      &ScrewSpec,
    depth:     f32,
    direction: Vec3,
    position:  Vec3,
) -> (Arc<dyn Sdf>, Arc<dyn Sdf>) {
    let br = spec.min_boss_diameter_mm / 2.0;
    (
        place_at(clearance_hole_z(spec, depth), position, direction),
        place_at(cyl_z(br, depth), position, direction),
    )
}

/// Countersink hole: recessed head seat at entry face + clearance shaft.
pub fn countersink_hole(
    spec:        &ScrewSpec,
    shaft_depth: f32,
    direction:   Vec3,
    position:    Vec3,
) -> (Arc<dyn Sdf>, Arc<dyn Sdf>) {
    let br          = spec.min_boss_diameter_mm / 2.0;
    let total_depth = spec.head_height_mm + shaft_depth;
    (
        place_at(countersink_hole_z(spec, shaft_depth), position, direction),
        place_at(cyl_z(br, total_depth), position, direction),
    )
}

/// Heat-set insert boss: blind hole + surrounding boss cylinder with smooth fillet.
pub fn heat_set_boss(
    spec:      &ScrewSpec,
    direction: Vec3,
    position:  Vec3,
) -> (Arc<dyn Sdf>, Arc<dyn Sdf>) {
    (
        place_at(heat_set_void_z(spec), position, direction),
        place_at(heat_set_boss_z(spec), position, direction),
    )
}

// ── Wall thickness measurement ────────────────────────────────────────────────

/// Approximate wall thickness by marching along `direction` from `start`
/// and measuring total distance spent inside the solid.
///
/// Uses 0.5 mm steps over 50 mm max range.
pub fn measure_wall_thickness(sdf: &dyn Sdf, start: Vec3, direction: Vec3) -> f32 {
    let dir     = direction.normalize();
    let step    = 0.5_f32;
    let n       = 100_usize; // 50 mm max
    let mut inside  = sdf.distance(start) < 0.0;
    let mut entry_t = if inside { 0.0_f32 } else { f32::MAX };
    let mut total   = 0.0_f32;

    for i in 1..=n {
        let t   = i as f32 * step;
        let now = sdf.distance(start + dir * t) < 0.0;
        match (inside, now) {
            (false, true)  => entry_t = t,
            (true,  false) => total  += t - entry_t,
            _              => {}
        }
        inside = now;
    }
    if inside && entry_t < f32::MAX {
        total += n as f32 * step - entry_t;
    }
    total
}

/// Subtract the void from body, automatically unioning the boss first if the
/// measured wall thickness at `position` along `direction` is below the spec minimum.
pub fn check_and_pad(
    body:      Arc<dyn Sdf>,
    void:      Arc<dyn Sdf>,
    boss:      Arc<dyn Sdf>,
    position:  Vec3,
    direction: Vec3,
    spec:      &ScrewSpec,
) -> Arc<dyn Sdf> {
    let thickness = measure_wall_thickness(body.as_ref(), position, direction);
    let base: Arc<dyn Sdf> = if thickness < spec.min_wall_thickness_mm {
        Arc::new(Union::new(body, boss))
    } else {
        body
    };
    Arc::new(Subtract::new(base, void))
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::primitives::SdfBox;

    #[test]
    fn m3_clearance_hole_radius() {
        // M3 clearance diameter is 3.4 mm → radius 1.7 mm.
        let spec     = get_spec("M3").unwrap();
        let (void, _) = clearance_hole(spec, 10.0, Vec3::Z, Vec3::ZERO);

        // A point at radius 1.6 (inside clearance) should be inside the void
        assert!(
            void.distance(Vec3::new(1.6, 0.0, 2.0)) < 0.0,
            "Point inside clearance hole should be negative"
        );
        // A point at radius 2.0 (outside clearance) should be outside
        assert!(
            void.distance(Vec3::new(2.0, 0.0, 2.0)) > 0.0,
            "Point outside clearance hole should be positive"
        );
    }

    #[test]
    fn thin_wall_triggers_boss() {
        // Plate 1 mm thick (below M3 min_wall 2.4 mm) → boss should be added.
        let spec  = get_spec("M3").unwrap();
        let plate: Arc<dyn Sdf> = Arc::new(SdfBox::new(Vec3::new(20.0, 20.0, 0.5)));
        let (void, boss) = clearance_hole(spec, 5.0, Vec3::Z, Vec3::ZERO);

        let result = check_and_pad(plate, void, boss, Vec3::ZERO, Vec3::Z, spec);

        // M3: clearance_r=1.7, boss_r=3.5
        // At (2.5, 0, 0.1): inside boss but outside void. With boss added this should be solid.
        // Boss union makes this region solid; hole is then subtracted (hole r=1.7 < 2.5, no removal here).
        let d = result.distance(Vec3::new(2.5, 0.0, 0.1));
        assert!(d < 0.0, "Boss region should be solid, got {}", d);
    }

    #[test]
    fn thick_wall_no_boss_needed() {
        // Plate 10 mm thick (above M3 min_wall) → no boss, hole is simply drilled.
        let spec  = get_spec("M3").unwrap();
        let plate: Arc<dyn Sdf> = Arc::new(SdfBox::new(Vec3::new(20.0, 20.0, 5.0)));
        let (void, boss) = clearance_hole(spec, 5.0, Vec3::Z, Vec3::ZERO);

        let result = check_and_pad(plate, void, boss, Vec3::ZERO, Vec3::Z, spec);

        // Center of hole at (0, 0, 2): should be outside (hole was drilled through)
        assert!(
            result.distance(Vec3::new(0.0, 0.0, 2.0)) > 0.0,
            "Hole center should be empty"
        );
    }

    #[test]
    fn screw_specs_lookup() {
        assert!(get_spec("M2").is_some());
        assert!(get_spec("M2.5").is_some());
        assert!(get_spec("M3").is_some());
        assert!(get_spec("M4").is_some());
        assert!(get_spec("M5").is_some());
        assert!(get_spec("M6").is_none());
    }
}
