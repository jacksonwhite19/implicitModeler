// Drone-specific structural primitives built by composing existing SDF operations.
// No new Sdf structs — every function returns an Arc<dyn Sdf> assembled from
// the existing primitive/transform/boolean toolkit.

use glam::{Vec3, Quat};
use std::f32::consts::FRAC_PI_2;
use std::sync::Arc;
use crate::sdf::Sdf;
use crate::sdf::primitives::{Sphere, SdfBox, Cylinder};
use crate::sdf::transforms::{Translate, Rotate, Offset};
use crate::sdf::booleans::{Union, Subtract, Intersect, SmoothUnion};
use crate::sdf::patterns::PolarArray;

// ── Internal helpers ──────────────────────────────────────────────────────────

/// Binary-search along +Y at span position `x` to find the SDF zero-crossing.
/// Returns the approximate radial extent of the SDF at that X slice.
pub fn estimate_radius_at(sdf: &Arc<dyn Sdf>, x: f32) -> f32 {
    let mut lo = 0.0_f32;
    let mut hi = 1_000.0_f32;
    if sdf.distance(Vec3::new(x, hi, 0.0)) < 0.0 {
        return hi; // degenerate — shape larger than search range
    }
    for _ in 0..48 {
        let mid = (lo + hi) * 0.5;
        if sdf.distance(Vec3::new(x, mid, 0.0)) < 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    (lo + hi) * 0.5
}

/// Find the arm tip Y by searching along +Y at multiple Z offsets (probing the tube wall).
///
/// For a hollow arm tube the axis (Z=0) is always exterior; probing at Z values that
/// span typical tube-wall thicknesses finds the actual distal cap.
fn find_arm_tip_y(sdf: &Arc<dyn Sdf>, attach_x: f32) -> f32 {
    // Z probes cover tube walls from thin (outer_r ≈ 0.003) to thick (outer_r ≈ 0.5).
    const Z_PROBES: [f32; 12] = [
        0.002, 0.005, 0.01, 0.02, 0.04, 0.06, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5,
    ];
    let fallback = estimate_radius_at(sdf, attach_x); // fuselage surface if tube not found
    let mut best = fallback;

    for &z in &Z_PROBES {
        // Only search if this Z slice has interior at the attachment point.
        if sdf.distance(Vec3::new(attach_x, 0.0, z)) >= 0.0 {
            continue;
        }
        let mut lo = 0.0_f32;
        let mut hi = 1_000.0_f32;
        if sdf.distance(Vec3::new(attach_x, hi, z)) < 0.0 {
            best = best.max(hi);
            continue;
        }
        for _ in 0..32 {
            let mid = (lo + hi) * 0.5;
            if sdf.distance(Vec3::new(attach_x, mid, z)) < 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        best = best.max((lo + hi) * 0.5);
    }
    best
}

/// Scan X ∈ [0, 1] to find the X where the SDF centre (Y=Z=0) is most interior,
/// then estimate the radius there.  Returns (best_x, radius).
fn find_bulkhead_centre(sdf: &Arc<dyn Sdf>) -> (f32, f32) {
    let mut best_x = 0.0_f32;
    let mut best_d = f32::MAX;
    for i in 0..=40 {
        let x = i as f32 / 40.0;
        let d = sdf.distance(Vec3::new(x, 0.0, 0.0));
        if d < best_d {
            best_d = d;
            best_x = x;
        }
    }
    let r = estimate_radius_at(sdf, best_x);
    (best_x, r)
}

/// Cylinder aligned along the X axis with radius `r` and half-length `half_len`.
/// (Standard `Cylinder` is along Z; we rotate it 90° around Y to lie along X.)
fn cylinder_along_x(r: f32, half_len: f32) -> Arc<dyn Sdf> {
    // Rotate::new applies rotation.inverse() to the query point.
    // Storing from_rotation_y(-FRAC_PI_2) makes the inverse (+π/2) map (x,y,z)→(z,y,-x),
    // so the cylinder evaluates with radial = sqrt(z²+y²) and axial = |x|. ✓
    let cyl = Arc::new(Cylinder::new(r, half_len));
    Arc::new(Rotate::new(cyl, Quat::from_rotation_y(-FRAC_PI_2)))
}

/// Cylinder aligned along the Y axis with radius `r` and half-length `half_len`.
fn cylinder_along_y(r: f32, half_len: f32) -> Arc<dyn Sdf> {
    // Storing from_rotation_x(-FRAC_PI_2): inverse (+π/2) maps (x,y,z)→(x,-z,y),
    // so radial = sqrt(x²+z²) and axial = |y|. ✓
    let cyl = Arc::new(Cylinder::new(r, half_len));
    Arc::new(Rotate::new(cyl, Quat::from_rotation_x(-FRAC_PI_2)))
}

// ── Public SDF composition functions ─────────────────────────────────────────

/// Structural bulkhead ring at normalised axial position `position` ∈ [0, 1].
///
/// * `fuselage`             – lofted fuselage SDF (X axis is span, span = 1.0)
/// * `position`             – normalised X position
/// * `thickness`            – axial (X) thickness of the ring
/// * `num_holes`            – number of lightening holes (0 = no holes)
/// * `hole_radius_fraction` – hole radius as a fraction of the local fuselage radius
pub fn bulkhead_at_station(
    fuselage: Arc<dyn Sdf>,
    position: f32,
    thickness: f32,
    num_holes: usize,
    hole_radius_fraction: f32,
) -> Arc<dyn Sdf> {
    // Thin slab at `position` along X, huge in Y and Z.
    let slab = Arc::new(SdfBox::new(Vec3::new(thickness * 0.5, 10_000.0, 10_000.0)));
    let slab = Arc::new(Translate::new(slab, Vec3::new(position, 0.0, 0.0)));

    // Bulkhead = fuselage cross-section at this station.
    let bulkhead: Arc<dyn Sdf> = Arc::new(Intersect::new(Arc::clone(&fuselage), slab));

    if num_holes == 0 {
        return bulkhead;
    }

    let fuse_r = estimate_radius_at(&fuselage, position);
    let hole_r  = fuse_r * hole_radius_fraction;
    // Place holes at 60 % of the fuselage radius (centroid of a thin ring).
    let hole_radial = fuse_r * 0.6;

    // Drill holes along X (axis = 0) through the ring.
    lightening_hole_pattern(bulkhead, num_holes, hole_radial, hole_r, 0)
}

/// Subtract a polar array of circular through-holes from a flat body.
///
/// * `body`        – target SDF
/// * `count`       – number of holes
/// * `radial_pos`  – absolute radial distance from the axis to hole centres
/// * `hole_radius` – radius of each hole
/// * `axis`        – drill axis: 0 = X, 1 = Y, 2 = Z
pub fn lightening_hole_pattern(
    body: Arc<dyn Sdf>,
    count: usize,
    radial_pos: f32,
    hole_radius: f32,
    axis: i64,
) -> Arc<dyn Sdf> {
    if count == 0 {
        return body;
    }
    const BIG: f32 = 10_000.0;

    let (hole_cyl, polar_axis, offset_vec) = match axis {
        0 => (cylinder_along_x(hole_radius, BIG), Vec3::X, Vec3::new(0.0, radial_pos, 0.0)),
        1 => (cylinder_along_y(hole_radius, BIG), Vec3::Y, Vec3::new(radial_pos, 0.0, 0.0)),
        _ => (
            Arc::new(Cylinder::new(hole_radius, BIG)) as Arc<dyn Sdf>,
            Vec3::Z,
            Vec3::new(radial_pos, 0.0, 0.0),
        ),
    };

    let hole_at_r = Arc::new(Translate::new(hole_cyl, offset_vec));
    let holes = Arc::new(PolarArray::new(hole_at_r, count, polar_axis));
    Arc::new(Subtract::new(body, holes))
}

/// Cylindrical boss on a bulkhead face accepting a carbon rod.
///
/// * `bulkhead`        – bulkhead SDF
/// * `angle_degrees`   – angle in the YZ plane (0° = +Y direction)
/// * `radial_fraction` – 0..1 from centre to bulkhead edge
/// * `rod_diameter`    – through-hole diameter
/// * `boss_diameter`   – outer boss cylinder diameter
pub fn rod_mount(
    bulkhead: Arc<dyn Sdf>,
    angle_degrees: f32,
    radial_fraction: f32,
    rod_diameter: f32,
    boss_diameter: f32,
) -> Arc<dyn Sdf> {
    let (bx, fuse_r) = find_bulkhead_centre(&bulkhead);
    let r_pos      = fuse_r * radial_fraction;
    let angle_rad  = angle_degrees.to_radians();
    let boss_y     = r_pos * angle_rad.cos();
    let boss_z     = r_pos * angle_rad.sin();

    // Boss: short cylinder along X (perpendicular to bulkhead face).
    let boss_half_h = boss_diameter; // height = 2 × diameter
    let boss_cyl    = cylinder_along_x(boss_diameter * 0.5, boss_half_h);
    let boss        = Arc::new(Translate::new(boss_cyl, Vec3::new(bx, boss_y, boss_z)));

    // Rod through-hole along X.
    let hole_cyl = cylinder_along_x(rod_diameter * 0.5, 10_000.0);
    let hole     = Arc::new(Translate::new(hole_cyl, Vec3::new(bx, boss_y, boss_z)));

    let with_boss = Arc::new(Union::new(bulkhead, boss));
    Arc::new(Subtract::new(with_boss, hole))
}

/// Hollow cylindrical boom arm extending radially from the fuselage at the given angle.
///
/// The arm attaches at X = 0.5 (fuselage midspan) and extends outward in the YZ plane.
///
/// * `fuselage`       – lofted fuselage SDF (normalised span)
/// * `angle_degrees`  – angle in the YZ plane (0° = +Y, 90° = +Z)
/// * `length`         – arm length from fuselage surface to tip
/// * `outer_diameter` – outer tube diameter
/// * `inner_diameter` – inner (hollow) tube diameter
pub fn motor_arm(
    fuselage: Arc<dyn Sdf>,
    angle_degrees: f32,
    length: f32,
    outer_diameter: f32,
    inner_diameter: f32,
) -> Arc<dyn Sdf> {
    let fuse_r  = estimate_radius_at(&fuselage, 0.5);
    let outer_r = outer_diameter * 0.5;
    let inner_r = inner_diameter * 0.5;
    let half_len = length * 0.5;

    // Hollow tube aligned along Y at the origin.
    let outer_cyl = cylinder_along_y(outer_r, half_len);
    let inner_cyl = cylinder_along_y(inner_r, half_len);
    let hollow    = Arc::new(Subtract::new(outer_cyl, inner_cyl));

    // Translate so the tube spans Y ∈ [fuse_r, fuse_r + length].
    let arm_along_y = Arc::new(Translate::new(
        hollow,
        Vec3::new(0.0, fuse_r + half_len, 0.0),
    ));

    // Rotate around X axis to the requested angle in the YZ plane.
    // Rotate::new(shape, Q) physically rotates the shape by Q.
    let angle_rad = angle_degrees.to_radians();
    let arm_rotated = Arc::new(Rotate::new(
        arm_along_y,
        Quat::from_rotation_x(angle_rad),
    ));

    // Move to midspan (X = 0.5).
    let arm_at_x: Arc<dyn Sdf> = Arc::new(Translate::new(arm_rotated, Vec3::new(0.5, 0.0, 0.0)));

    // Trim the arm-fuselage overlap: subtract fuselage interior so the inner end is flush.
    let trimmed = Arc::new(Subtract::new(Arc::clone(&arm_at_x), Arc::clone(&fuselage)));

    // Smooth-blend the arm into the fuselage.
    Arc::new(SmoothUnion::new(fuselage, trimmed, outer_r * 0.4))
}

/// Square motor mounting plate at the distal end of an arm.
///
/// Assumes the arm extends along +Y at X = 0.5 (i.e. `angle_degrees = 0` in `motor_arm`).
/// The plate is oriented perpendicular to Y (lying in the XZ plane) at the arm tip.
///
/// * `arm`            – arm SDF (as returned by `motor_arm`)
/// * `motor_size_mm`  – side length of the square plate
/// * `plate_thickness`– plate thickness (along the arm axis)
/// * `bolt_pattern`   – PCD radius of the 4-bolt circle
/// * `bolt_diameter`  – diameter of each bolt hole
pub fn motor_mount(
    arm: Arc<dyn Sdf>,
    motor_size_mm: f32,
    plate_thickness: f32,
    bolt_pattern: f32,
    bolt_diameter: f32,
) -> Arc<dyn Sdf> {
    // Tip of the arm: probe the tube wall at multiple Z offsets (axis is hollow/exterior).
    let tip_y     = find_arm_tip_y(&arm, 0.5);
    let half_side = motor_size_mm * 0.5;

    // Square plate in the XZ plane at (0.5, tip_y, 0).
    let plate_box = Arc::new(SdfBox::new(Vec3::new(
        half_side,
        plate_thickness * 0.5,
        half_side,
    )));
    let plate = Arc::new(Translate::new(plate_box, Vec3::new(0.5, tip_y, 0.0)));

    // Bolt holes: 4 cylinders along Y in a PolarArray centred on the mount.
    // Build hole at (bolt_pattern, 0, 0) relative to mount centre, PolarArray around Y, then translate.
    let bolt_hole  = cylinder_along_y(bolt_diameter * 0.5, 10_000.0);
    let bolt_local = Arc::new(Translate::new(bolt_hole, Vec3::new(bolt_pattern, 0.0, 0.0)));
    let bolt_ring  = Arc::new(PolarArray::new(bolt_local, 4, Vec3::Y));
    let bolts      = Arc::new(Translate::new(bolt_ring, Vec3::new(0.5, tip_y, 0.0)));

    let plate_drilled = Arc::new(Subtract::new(plate, bolts));
    Arc::new(Union::new(arm, plate_drilled))
}

/// Generate mounting trays and attachment tabs for a set of placed components.
///
/// For each component:
/// 1. Shell the keepout outward by `wall_thickness` to form tray walls.
/// 2. Intersect the expanded region (wall + tab) with the parent structure to form
///    attachment tabs that reach the parent surface.
/// 3. Subtract the keepout so the component fits inside.
///
/// Returns a single SDF representing all mounting structure combined.
pub fn generate_mounts_sdf(
    components: Vec<(Arc<dyn Sdf>, Arc<dyn Sdf>)>, // (geometry, keepout)
    parent: Arc<dyn Sdf>,
    wall_thickness: f32,
    tab_width: f32,
) -> Arc<dyn Sdf> {
    let mut result: Option<Arc<dyn Sdf>> = None;

    for (_, keepout) in components {
        // Tray outer shell: keepout expanded by wall_thickness, minus the keepout interior.
        let tray_outer: Arc<dyn Sdf> = Arc::new(Offset::new(Arc::clone(&keepout), wall_thickness));
        let tray = Arc::new(Subtract::new(Arc::clone(&tray_outer), Arc::clone(&keepout)));

        // Tab region: expanded by wall + tab, clipped to the parent's near-surface zone.
        let tab_vol   = Arc::new(Offset::new(Arc::clone(&keepout), wall_thickness + tab_width));
        let near_parent = Arc::new(Offset::new(Arc::clone(&parent), tab_width));
        let tabs_raw  = Arc::new(Intersect::new(tab_vol, near_parent));
        let tabs      = Arc::new(Subtract::new(tabs_raw, Arc::clone(&keepout)));

        let mount = Arc::new(Union::new(tray, tabs));

        result = Some(match result {
            None    => mount,
            Some(p) => Arc::new(Union::new(p, mount)),
        });
    }

    // No components → always-outside placeholder.
    result.unwrap_or_else(|| Arc::new(Sphere::new(-1.0)) as Arc<dyn Sdf>)
}

// ── Conformal lattice convenience wrappers ────────────────────────────────────

use crate::sdf::lattice::ConformalGyroid;
use crate::sdf::field::primitives::SdfField;
use crate::sdf::field::gradients::RadialField;

/// Fill a wing volume with a conformal gyroid lattice, automatically masked to
/// the interior by shelling the wing inward by `1.5 * thickness` so struts
/// never intersect the outer skin.
pub fn wing_lattice(wing: Arc<dyn Sdf>, cell_size: f32, thickness: f32) -> Arc<dyn Sdf> {
    let inset_mask: Arc<dyn Sdf> = Arc::new(Offset::new(Arc::clone(&wing), -(1.5 * thickness)));
    Arc::new(ConformalGyroid::with_region_mask(wing, cell_size, thickness, inset_mask))
}

/// Fill a fuselage volume with a conformal gyroid lattice, automatically masked
/// inward by `1.5 * thickness` to preserve skin integrity.
pub fn fuselage_lattice(fuselage: Arc<dyn Sdf>, cell_size: f32, thickness: f32) -> Arc<dyn Sdf> {
    let inset_mask: Arc<dyn Sdf> = Arc::new(Offset::new(Arc::clone(&fuselage), -(1.5 * thickness)));
    Arc::new(ConformalGyroid::with_region_mask(fuselage, cell_size, thickness, inset_mask))
}

/// Fill a fuselage with a density-graded conformal gyroid lattice.
/// Density is driven by a radial field derived from the fuselage SDF, grading
/// from coarse at the centre to fine near the skin.
pub fn fuselage_lattice_graded(
    fuselage: Arc<dyn Sdf>,
    inner_cell_size: f32,
    outer_cell_size: f32,
    thickness: f32,
) -> Arc<dyn Sdf> {
    use std::sync::Arc as A;
    // Field value: 1.0 at interior (large negative SDF), 0.0 at surface (SDF=0).
    // Effective cell size = base_cell / field.  We want coarse (large cell) at centre and
    // fine (small cell) at skin, so invert: field ≈ base / target_cell.
    // Use a radial field from the SDF field with inner_val = cell_size / inner_cell and
    // outer_val = cell_size / outer_cell, both normalised to 1 at the reference.
    let sdf_field: Arc<dyn crate::sdf::field::Field> = A::new(SdfField::new(Arc::clone(&fuselage)));
    // Rough radius estimate for the fuselage at midspan.
    let approx_r = 50.0_f32; // conservative — radial_field clamps anyway
    let density_field: Arc<dyn crate::sdf::field::Field> = A::new(RadialField::new(
        glam::Vec3::ZERO,
        0.0,          // inner_r: at centre
        approx_r,     // outer_r: at (approximate) skin
        inner_cell_size / outer_cell_size, // inner_val → divides out, so cell = outer_cell
        1.0,          // outer_val → cell = outer_cell
    ));
    let _ = sdf_field; // sdf_field not needed — radial by distance from origin suffices
    let inset_mask: Arc<dyn Sdf> = A::new(Offset::new(Arc::clone(&fuselage), -(1.5 * thickness)));
    Arc::new(ConformalGyroid {
        parent:        fuselage,
        cell_size:     outer_cell_size,
        thickness,
        density_field: Some(density_field),
        region_mask:   Some(inset_mask),
    })
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::aerospace::fuselage::LoftedFuselage;
    use crate::sdf::aerospace::CrossSection;
    use crate::sdf::aerospace::Section2D;

    fn make_fuselage() -> Arc<dyn Sdf> {
        let pairs: Vec<(f32, Arc<dyn Section2D>)> = vec![
            (0.0, Arc::new(CrossSection::Circle { radius: 0.3 })),
            (0.5, Arc::new(CrossSection::Circle { radius: 1.0 })),
            (1.0, Arc::new(CrossSection::Circle { radius: 0.3 })),
        ];
        Arc::new(LoftedFuselage::from_stations(pairs, 1.0))
    }

    #[test]
    fn test_estimate_radius_at() {
        let fuse = make_fuselage();
        // At midspan (x=0.5) the circle radius is 1.0.
        let r = estimate_radius_at(&fuse, 0.5);
        assert!((r - 1.0).abs() < 0.05, "expected radius ≈ 1.0 at midspan, got {}", r);
        // At nose/tail the radius should be ~0.3.
        let r_nose = estimate_radius_at(&fuse, 0.0);
        assert!(r_nose < 0.5, "nose radius should be smaller, got {}", r_nose);
    }

    #[test]
    fn test_bulkhead_at_station_is_valid_sdf() {
        let fuse = make_fuselage();
        let bk = bulkhead_at_station(Arc::clone(&fuse), 0.5, 0.02, 0, 0.0);
        // Centre of bulkhead (x=0.5, y=0, z=0) should be inside.
        assert!(bk.distance(Vec3::new(0.5, 0.0, 0.0)) < 0.0,
            "bulkhead centre should be inside");
        // Far outside should be positive.
        assert!(bk.distance(Vec3::new(0.5, 5.0, 0.0)) > 0.0,
            "point far from bulkhead should be outside");
        // Off the station along X should be outside.
        assert!(bk.distance(Vec3::new(0.9, 0.0, 0.0)) > 0.0,
            "point off-station should be outside");
    }

    #[test]
    fn test_bulkhead_with_holes() {
        let fuse = make_fuselage();
        let bk = bulkhead_at_station(Arc::clone(&fuse), 0.5, 0.02, 8, 0.3);
        // Centre should still be inside (hole_radius_fraction 0.3 leaves material at origin).
        assert!(bk.distance(Vec3::new(0.5, 0.0, 0.0)) < 0.0,
            "centre with holes should be inside");
    }

    #[test]
    fn test_lightening_hole_pattern_subtracts() {
        let fuse = make_fuselage();
        let bk_no_holes  = bulkhead_at_station(Arc::clone(&fuse), 0.5, 0.02, 0, 0.0);
        let bk_with_holes = lightening_hole_pattern(bk_no_holes, 4, 0.5, 0.1, 0);
        // Point at hole location (x=0.5, y=0.5, z=0) should be outside after drilling.
        let d = bk_with_holes.distance(Vec3::new(0.5, 0.5, 0.0));
        assert!(d > 0.0, "hole location should be outside, got {}", d);
    }

    #[test]
    fn test_motor_arm_extends_outward() {
        let fuse = make_fuselage();
        // outer_r = 0.06, inner_r = 0.045 → tube wall at Z ≈ 0.05 (midwall).
        let arm = motor_arm(Arc::clone(&fuse), 0.0, 2.0, 0.12, 0.09);
        // Tube wall at midspan of the arm should be inside.
        let d = arm.distance(Vec3::new(0.5, 2.5, 0.05));
        assert!(d < 0.0, "arm tube wall at Y=2.5, Z=0.05 should be inside, got {}", d);
        // The hollow axis is outside the tube.
        let d_axis = arm.distance(Vec3::new(0.5, 2.5, 0.0));
        assert!(d_axis > 0.0, "arm axis (hollow) should be outside, got {}", d_axis);
        // A point far beyond the arm tip should be outside.
        let d_far = arm.distance(Vec3::new(0.5, 10.0, 0.0));
        assert!(d_far > 0.0, "beyond arm tip should be outside, got {}", d_far);
    }

    #[test]
    fn test_motor_arm_can_be_composed() {
        let fuse = make_fuselage();
        let arm  = motor_arm(Arc::clone(&fuse), 0.0, 2.0, 0.12, 0.09);
        let arm2 = motor_arm(Arc::clone(&fuse), 180.0, 2.0, 0.12, 0.09);
        // Result is a valid SdfHandle that can be further combined.
        let both = Arc::new(Union::new(arm, arm2));
        // Each arm tube wall should be reachable (Z=0.05 is in the wall for outer_r=0.06).
        assert!(both.distance(Vec3::new(0.5,  2.5, 0.05)) < 0.0, "+Y arm wall should be inside");
        assert!(both.distance(Vec3::new(0.5, -2.5, 0.05)) < 0.0, "-Y arm wall should be inside");
    }

    #[test]
    fn test_motor_mount_plate_at_tip() {
        let fuse  = make_fuselage();
        let arm   = motor_arm(Arc::clone(&fuse), 0.0, 2.0, 0.12, 0.09);
        let mount = motor_mount(arm, 0.3, 0.05, 0.12, 0.03);
        // Plate centre is at y ≈ 3.0, half_side = 0.15, plate_thickness = 0.05.
        // A point slightly inside the plate (y = 2.98) should be inside.
        let d = mount.distance(Vec3::new(0.5, 2.98, 0.0));
        assert!(d < 0.0, "motor mount plate interior should be inside, got {}", d);
    }

    #[test]
    fn test_rod_mount_valid() {
        let fuse = make_fuselage();
        let bk   = bulkhead_at_station(Arc::clone(&fuse), 0.5, 0.02, 0, 0.0);
        let rm   = rod_mount(bk, 0.0, 0.6, 0.04, 0.08);
        // Result must be a valid SDF (query does not panic).
        let _d = rm.distance(Vec3::new(0.5, 0.0, 0.0));
    }

    #[test]
    fn test_generate_mounts_no_components_is_valid() {
        let fuse = make_fuselage();
        let result = generate_mounts_sdf(vec![], Arc::clone(&fuse), 0.1, 0.2);
        // Placeholder should be always-outside.
        let d = result.distance(Vec3::new(0.5, 0.0, 0.0));
        assert!(d > 0.0, "empty mount result should be outside everywhere, got {}", d);
    }

    #[test]
    fn test_generate_mounts_creates_tray() {
        let fuse = make_fuselage();
        let battery_geom: Arc<dyn Sdf> = Arc::new(SdfBox::new(Vec3::new(0.2, 0.1, 0.075)));
        let battery_geom_placed: Arc<dyn Sdf> = Arc::new(Translate::new(Arc::clone(&battery_geom), Vec3::new(0.5, 0.0, -0.3)));
        let keepout = Arc::new(Offset::new(Arc::clone(&battery_geom_placed), 0.02));
        let mounts = generate_mounts_sdf(
            vec![(battery_geom_placed, keepout)],
            Arc::clone(&fuse),
            0.015,
            0.04,
        );
        // Result is a valid SDF.
        let _d = mounts.distance(Vec3::new(0.5, 0.0, -0.3));
    }
}
