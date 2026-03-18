// Access panel generation for FDM printed enclosures.
#![allow(dead_code)] // Panel result fields — not all consumed by the UI yet
//
// Provides five retention mechanisms (snap fit, screw tabs, friction fit,
// living hinge, magnet boss) and a panel_rect() builder that returns the
// cutout void, removable panel geometry, and parent-side geometry updates.

use std::sync::Arc;
use glam::Vec3;
use crate::sdf::Sdf;
use crate::sdf::primitives::SdfBox;
use crate::sdf::booleans::{Union, Subtract};
use crate::sdf::transforms::{Translate, Rotate};
use super::fasteners::{
    get_spec, cyl_z, orient_along, NullSdf, heat_set_void_z, heat_set_boss_z,
};

// ── Retention mechanism ───────────────────────────────────────────────────────

#[derive(Clone)]
pub enum RetentionMechanism {
    SnapFit {
        clip_count:     usize,
        clip_width:     f32,
        clip_thickness: f32,
        engagement_mm:  f32,
    },
    ScrewTabs {
        screw_designation: String,
        tab_count:         usize,
        tab_thickness:     f32,
    },
    FrictionFit {
        interference_mm: f32,
    },
    LivingHinge {
        hinge_thickness: f32,
        hinge_width:     f32,
    },
    MagnetBoss {
        magnet_diameter: f32,
        magnet_depth:    f32,
        count:           usize,
    },
}

// ── AccessPanel ───────────────────────────────────────────────────────────────

pub struct AccessPanel {
    /// Void to subtract from the parent body (the opening).
    pub cutout:      Arc<dyn Sdf>,
    /// The standalone removable panel geometry.
    pub panel:       Arc<dyn Sdf>,
    /// Geometry to union into the parent (e.g., snap catches, hinge pads).
    pub parent_add:  Arc<dyn Sdf>,
    /// Geometry to subtract from the parent (e.g., magnet holes, heat-set voids).
    pub parent_void: Arc<dyn Sdf>,
    /// The retention type (for documentation / further processing).
    pub retention:   RetentionMechanism,
}

// ── Frame helpers ─────────────────────────────────────────────────────────────

/// Orthonormal tangent frame for the plane perpendicular to `normal`.
/// Returns (t1, t2) where t1 and t2 span the panel surface plane.
pub fn frame(normal: Vec3) -> (Vec3, Vec3) {
    let n  = normal.normalize();
    let up = if n.abs().x < 0.9 { Vec3::X } else { Vec3::Y };
    let t1 = (up - n * up.dot(n)).normalize();
    let t2 = n.cross(t1).normalize();
    (t1, t2)
}

/// Axis-aligned box oriented with: width along t1, height along t2, depth along `normal`.
/// Centered at `center`.
pub fn oriented_box(w: f32, h: f32, d: f32, center: Vec3, normal: Vec3) -> Arc<dyn Sdf> {
    let n        = normal.normalize();
    let (t1, t2) = frame(n);
    let b: Arc<dyn Sdf> = Arc::new(SdfBox::new(Vec3::new(w / 2.0, h / 2.0, d / 2.0)));
    // Rotation matrix: columns are the world-space axes for box X, Y, Z
    let mat = glam::Mat3::from_cols(t1, t2, n);
    let q   = glam::Quat::from_mat3(&mat);
    // Skip rotation if already aligned with the canonical frame
    let rotated: Arc<dyn Sdf> =
        if (n.dot(Vec3::Z) - 1.0).abs() < 1e-4 && (t1.dot(Vec3::X) - 1.0).abs() < 1e-4 {
            b
        } else {
            Arc::new(Rotate::new(b, q))
        };
    Arc::new(Translate::new(rotated, center))
}

// ── Perimeter array ───────────────────────────────────────────────────────────

/// Union `count` copies of `make()` around a circle of radius `r` in the plane
/// perpendicular to `normal`, centered at `center`.  Each item is a Z-aligned
/// SDF that will be oriented along `normal` then translated to its position.
fn perimeter_array(
    count:  usize,
    r:      f32,
    center: Vec3,
    normal: Vec3,
    make:   impl Fn() -> Arc<dyn Sdf>,
) -> Arc<dyn Sdf> {
    use std::f32::consts::TAU;
    let (t1, t2)             = frame(normal);
    let mut result: Option<Arc<dyn Sdf>> = None;

    for i in 0..count.max(1) {
        let a      = TAU * i as f32 / count.max(1) as f32;
        let offset = t1 * a.cos() * r + t2 * a.sin() * r;
        let pos    = center + offset;
        let item   = make();
        let placed = Arc::new(Translate::new(orient_along(item, normal), pos));
        result = Some(match result {
            None    => placed,
            Some(r) => Arc::new(Union::new(r, placed)),
        });
    }
    result.unwrap_or_else(|| Arc::new(NullSdf) as Arc<dyn Sdf>)
}

// ── Snap clip geometry ────────────────────────────────────────────────────────

/// Z-aligned snap clip: cantilever beam + wedge tip.
fn make_clip(clip_width: f32, clip_thickness: f32, engagement: f32) -> Arc<dyn Sdf> {
    let hw  = clip_width / 2.0;
    let ht  = clip_thickness / 2.0;
    let arm = 2.5_f32;
    // Cantilever beam
    let beam: Arc<dyn Sdf> = Arc::new(Translate::new(
        Arc::new(SdfBox::new(Vec3::new(hw, ht, arm / 2.0))),
        Vec3::new(0.0, 0.0, arm / 2.0),
    ));
    // Wedge tip (slightly protruding in Y for snap engagement)
    let tip: Arc<dyn Sdf> = Arc::new(Translate::new(
        Arc::new(SdfBox::new(Vec3::new(hw, ht + engagement / 2.0, engagement / 2.0))),
        Vec3::new(0.0, engagement / 4.0, arm + engagement / 2.0),
    ));
    Arc::new(Union::new(beam, tip))
}

/// Z-aligned snap catch (parent-side ledge that the clip engages).
fn make_catch(clip_width: f32, engagement: f32) -> Arc<dyn Sdf> {
    let catch_h = engagement + 0.3;
    Arc::new(Translate::new(
        Arc::new(SdfBox::new(Vec3::new(
            clip_width / 2.0 + 0.3,
            engagement / 2.0,
            catch_h / 2.0,
        ))),
        Vec3::new(0.0, 0.0, catch_h / 2.0),
    ))
}

// ── panel_rect ────────────────────────────────────────────────────────────────

/// Build an access panel at the given world position.
///
/// `normal` is the outward-facing surface normal of the parent body at the
/// opening site.  Width is measured along t1, height along t2 where (t1, t2,
/// normal) form a right-handed frame.
pub fn panel_rect(
    x:         f32,
    y:         f32,
    z:         f32,
    width:     f32,
    height:    f32,
    thickness: f32,
    normal:    Vec3,
    retention: RetentionMechanism,
) -> AccessPanel {
    let center   = Vec3::new(x, y, z);
    let n        = normal.normalize();
    let (_, t2)  = frame(n);

    // Cutout is slightly oversized for fit clearance
    let cutout = oriented_box(width + 0.3, height + 0.3, thickness + 0.1, center, n);

    match retention {
        // ── Friction fit ──────────────────────────────────────────────────────
        RetentionMechanism::FrictionFit { interference_mm } => {
            let panel = oriented_box(
                width  - interference_mm * 2.0,
                height - interference_mm * 2.0,
                thickness,
                center,
                n,
            );
            AccessPanel {
                cutout, panel,
                parent_add:  Arc::new(NullSdf),
                parent_void: Arc::new(NullSdf),
                retention:   RetentionMechanism::FrictionFit { interference_mm },
            }
        }

        // ── Snap fit ──────────────────────────────────────────────────────────
        RetentionMechanism::SnapFit { clip_count, clip_width, clip_thickness, engagement_mm } => {
            let panel_base = oriented_box(width - 0.2, height - 0.2, thickness, center, n);
            let r          = (width.min(height) / 2.0) * 0.8;
            let cw         = clip_width;
            let ct         = clip_thickness;
            let eng        = engagement_mm;

            // Clips on the panel — extend along +normal from the panel edge
            let clips = perimeter_array(clip_count, r, center, n, || make_clip(cw, ct, eng));
            let panel: Arc<dyn Sdf> = Arc::new(Union::new(panel_base, clips));

            // Catches on the parent cutout edges
            let catches = perimeter_array(clip_count, r, center, n, || make_catch(cw, eng));

            AccessPanel {
                cutout, panel,
                parent_add:  catches,
                parent_void: Arc::new(NullSdf),
                retention:   RetentionMechanism::SnapFit {
                    clip_count, clip_width, clip_thickness, engagement_mm,
                },
            }
        }

        // ── Screw tabs ────────────────────────────────────────────────────────
        RetentionMechanism::ScrewTabs { ref screw_designation, tab_count, tab_thickness } => {
            let desig       = screw_designation.clone();
            let panel_base  = oriented_box(width, height, thickness, center, n);
            // Tabs: small rectangular extensions at perimeter + clearance holes
            let r           = (width.max(height) / 2.0) + 3.0;
            let tab_w       = 8.0_f32;

            // Panel side: tabs with clearance holes
            let panel_tabs: Arc<dyn Sdf> = if let Some(spec) = get_spec(&desig) {
                let depth   = thickness + 1.0;
                let hole_r  = spec.clearance_diameter_mm / 2.0;
                perimeter_array(tab_count, r, center, n, || {
                    // Tab box (Z-aligned, Z=normal direction)
                    let tab: Arc<dyn Sdf> = Arc::new(Translate::new(
                        Arc::new(SdfBox::new(Vec3::new(tab_w / 2.0, tab_thickness / 2.0, thickness / 2.0))),
                        Vec3::new(0.0, 0.0, thickness / 2.0),
                    ));
                    // Clearance hole through the tab
                    let hole = cyl_z(hole_r, depth);
                    Arc::new(Subtract::new(tab, hole))
                })
            } else {
                Arc::new(NullSdf)
            };
            let panel: Arc<dyn Sdf> = Arc::new(Union::new(panel_base, panel_tabs));

            // Parent side: heat-set bosses
            let parent_add: Arc<dyn Sdf> = if let Some(spec) = get_spec(&desig) {
                perimeter_array(tab_count, r, center, n, || heat_set_boss_z(spec))
            } else {
                Arc::new(NullSdf)
            };
            let parent_void: Arc<dyn Sdf> = if let Some(spec) = get_spec(&desig) {
                perimeter_array(tab_count, r, center, n, || heat_set_void_z(spec))
            } else {
                Arc::new(NullSdf)
            };

            AccessPanel {
                cutout, panel, parent_add, parent_void,
                retention: RetentionMechanism::ScrewTabs {
                    screw_designation: desig, tab_count, tab_thickness,
                },
            }
        }

        // ── Living hinge ──────────────────────────────────────────────────────
        RetentionMechanism::LivingHinge { hinge_thickness, hinge_width } => {
            let panel_base = oriented_box(width, height, thickness, center, n);

            // Hinge strip at the "bottom" edge (−t2 side) of the panel
            let hinge_pos: Vec3 = center - t2 * (height / 2.0);
            let hinge_strip     = oriented_box(hinge_width, hinge_thickness, thickness, hinge_pos, n);
            let panel: Arc<dyn Sdf> = Arc::new(Union::new(panel_base, hinge_strip));

            // Parent: reinforced hinge attachment pad on the outer side of the hinge edge
            let pad_pos   = center - t2 * (height / 2.0 + hinge_thickness);
            let parent_add = oriented_box(
                hinge_width,
                hinge_thickness * 2.0,
                thickness,
                pad_pos,
                n,
            );

            AccessPanel {
                cutout, panel,
                parent_add,
                parent_void: Arc::new(NullSdf),
                retention:   RetentionMechanism::LivingHinge { hinge_thickness, hinge_width },
            }
        }

        // ── Magnet boss ───────────────────────────────────────────────────────
        RetentionMechanism::MagnetBoss { magnet_diameter, magnet_depth, count } => {
            let panel_base = oriented_box(width - 0.3, height - 0.3, thickness, center, n);
            let magnet_r   = (magnet_diameter + 0.2) / 2.0;
            let r          = (width.min(height) / 2.0) * 0.7;
            let md         = magnet_depth;
            let mr         = magnet_r;

            // Magnet holes on panel (blind, from the mating face)
            let panel_holes = perimeter_array(count, r, center, n, || cyl_z(mr, md));
            let panel: Arc<dyn Sdf> = Arc::new(Subtract::new(panel_base, panel_holes));

            // Matching magnet holes on parent
            let parent_void = perimeter_array(count, r, center, n, || cyl_z(mr, md));

            AccessPanel {
                cutout, panel,
                parent_add:  Arc::new(NullSdf),
                parent_void,
                retention:   RetentionMechanism::MagnetBoss { magnet_diameter, magnet_depth, count },
            }
        }
    }
}

// ── Drone convenience wrappers ────────────────────────────────────────────────

/// Battery hatch on a fuselage, opening in the +Z (top) direction.
/// `station_x` is the longitudinal centre of the battery bay.
pub fn battery_hatch(
    station_x: f32,
    width:     f32,
    height:    f32,
    retention: RetentionMechanism,
) -> AccessPanel {
    panel_rect(station_x, 0.0, 0.0, width, height, 3.0, Vec3::Z, retention)
}

/// Flight-controller access panel sized for a standard FC footprint.
/// Assumes a 36×36 mm opening to fit 30.5 mm and 20 mm bolt-circle FCs.
pub fn fc_access_panel(station_x: f32, retention: RetentionMechanism) -> AccessPanel {
    panel_rect(station_x, 0.0, 0.0, 36.0, 36.0, 3.0, Vec3::Z, retention)
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn snap_fit_produces_clip_geometry() {
        let ap = panel_rect(
            0.0, 0.0, 0.0, 40.0, 30.0, 3.0, Vec3::Z,
            RetentionMechanism::SnapFit {
                clip_count:     4,
                clip_width:     4.0,
                clip_thickness: 0.8,
                engagement_mm:  0.6,
            },
        );
        // Panel should have clips (non-null parent_add means catches exist on parent)
        let d_catch = ap.parent_add.distance(Vec3::new(13.0, 0.0, 1.0));
        assert!(
            d_catch < f32::MAX / 4.0,
            "Catches should be real geometry, got {}",
            d_catch
        );
        // Panel clips should exist above the panel plane (z>1.5 for a 3mm panel)
        let d_clip = ap.panel.distance(Vec3::new(13.0, 0.0, 4.0));
        assert!(d_clip < 5.0, "Clip should be near panel edge at z=4, got {}", d_clip);
    }

    #[test]
    fn friction_fit_panel_is_inset() {
        let interference = 0.15;
        let ap = panel_rect(
            0.0, 0.0, 0.0, 40.0, 30.0, 3.0, Vec3::Z,
            RetentionMechanism::FrictionFit { interference_mm: interference },
        );
        // Cutout half-width: (40+0.3)/2 = 20.15 — a point just inside should be inside cutout
        assert!(ap.cutout.distance(Vec3::new(19.9, 0.0, 0.0)) < 0.0);
        // Panel half-width: (40 - 2*0.15)/2 = 19.85 — the same point should be outside panel
        assert!(ap.panel.distance(Vec3::new(19.9, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn magnet_boss_creates_holes_on_both_sides() {
        let ap = panel_rect(
            0.0, 0.0, 0.0, 40.0, 30.0, 3.0, Vec3::Z,
            RetentionMechanism::MagnetBoss {
                magnet_diameter: 6.0,
                magnet_depth:    2.5,
                count:           4,
            },
        );
        // parent_void should be non-null
        let d = ap.parent_void.distance(Vec3::new(0.0, 0.0, 1.0));
        assert!(d < f32::MAX / 4.0, "Parent magnet void should be real geometry");
    }
}
