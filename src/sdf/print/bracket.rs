// Automated bracket generation for mounting components onto printed parts.
#![allow(dead_code)] // Bracket geometry API — some variant/fields not yet used in UI
//
// Generates bracket geometry (flat plate, saddle, cantilever, tray) around a
// keepout SDF, then drills heat-set boss and clearance holes at the specified
// mounting hole positions.

use std::sync::Arc;
use glam::Vec3;
use crate::sdf::Sdf;
use crate::sdf::primitives::SdfBox;
use crate::sdf::booleans::{Union, Subtract};
use crate::sdf::transforms::Translate;
use crate::sdf::print::fasteners::{NullSdf, get_spec, clearance_hole, heat_set_boss, check_and_pad};

// ── Minimal local hole type ───────────────────────────────────────────────────

/// Minimal hole info needed for bracket geometry.
/// Avoids importing scripting types to prevent circular dependencies.
#[derive(Clone, Debug)]
pub struct BracketHole {
    pub position:    Vec3,
    pub direction:   Vec3,
    pub designation: String,
}

// ── Bracket type enum ─────────────────────────────────────────────────────────

#[derive(Clone, Debug)]
pub enum BracketFace {
    Top,
    Bottom,
    Front,
    Back,
    Left,
    Right,
}

impl BracketFace {
    /// Returns outward-pointing normal for this face.
    pub fn normal(&self) -> Vec3 {
        match self {
            BracketFace::Top    =>  Vec3::Z,
            BracketFace::Bottom => -Vec3::Z,
            BracketFace::Front  =>  Vec3::X,
            BracketFace::Back   => -Vec3::X,
            BracketFace::Left   => -Vec3::Y,
            BracketFace::Right  =>  Vec3::Y,
        }
    }
}

#[derive(Clone, Debug)]
pub enum BracketType {
    FlatPlate {
        plate_thickness: f32,
        tab_width:       f32,
        tab_extension:   f32,
    },
    Saddle {
        wall_thickness:  f32,
        conform_radius:  f32,
    },
    Cantilever {
        arm_thickness:   f32,
        arm_width:       f32,
        face:            BracketFace,
    },
    FullTray {
        wall_thickness:  f32,
        floor_thickness: f32,
        open_face:       BracketFace,
    },
}

// ── Result type ───────────────────────────────────────────────────────────────

pub struct BracketGeometry {
    pub bracket_body:   Arc<dyn Sdf>,
    pub void_in_parent: Arc<dyn Sdf>,
}

// ── Bracket body generation ───────────────────────────────────────────────────

pub fn generate_bracket_body(
    keepout:      Arc<dyn Sdf>,
    bracket_type: &BracketType,
    holes:        &[BracketHole],
    bmin:         Vec3,
    bmax:         Vec3,
) -> BracketGeometry {
    let midpoint = (bmin + bmax) * 0.5;
    let size = bmax - bmin;

    match bracket_type {
        BracketType::FlatPlate { plate_thickness, tab_width, tab_extension } => {
            let pt = *plate_thickness;
            let te = *tab_extension;

            // Main plate: spans the full X/Y of the component, sits below it.
            let plate_half = Vec3::new(
                size.x / 2.0 + te,
                size.y / 2.0,
                pt / 2.0,
            );
            let plate_center = Vec3::new(midpoint.x, midpoint.y, bmin.z - pt / 2.0);
            let plate: Arc<dyn Sdf> = Arc::new(Translate::new(
                Arc::new(SdfBox::new(plate_half)),
                plate_center,
            ));

            // Add tabs at each hole position.
            let tw = *tab_width;
            let plate_z = bmin.z - pt / 2.0;
            let mut body: Arc<dyn Sdf> = plate;

            for hole in holes {
                let tab_half = Vec3::new(tw / 2.0, size.y / 2.0, pt / 2.0);
                let tab_center = Vec3::new(hole.position.x, midpoint.y, plate_z);
                let tab: Arc<dyn Sdf> = Arc::new(Translate::new(
                    Arc::new(SdfBox::new(tab_half)),
                    tab_center,
                ));
                body = Arc::new(Union::new(body, tab));
            }

            BracketGeometry {
                bracket_body:   body,
                void_in_parent: Arc::new(NullSdf),
            }
        }

        BracketType::Saddle { wall_thickness, .. } => {
            let wt = *wall_thickness;
            // Outer box wrapping the keepout with wall_thickness offset.
            let outer_half = Vec3::new(
                size.x / 2.0 + wt,
                size.y / 2.0 + wt,
                size.z / 2.0 + wt,
            );
            let outer: Arc<dyn Sdf> = Arc::new(Translate::new(
                Arc::new(SdfBox::new(outer_half)),
                midpoint,
            ));
            // Subtract the keepout interior to hollow it out.
            let body: Arc<dyn Sdf> = Arc::new(Subtract::new(outer, keepout));

            BracketGeometry {
                bracket_body:   body,
                void_in_parent: Arc::new(NullSdf),
            }
        }

        BracketType::Cantilever { arm_thickness, arm_width, face } => {
            let at = *arm_thickness;
            let aw = *arm_width;
            let face_normal = face.normal();

            // Determine face center.
            let face_center = midpoint + face_normal * (size * 0.5).dot(face_normal.abs());

            // Arm length: extend 30mm in the face normal direction.
            let arm_length = 30.0_f32;
            let arm_end = face_center + face_normal * arm_length;
            let arm_mid = (face_center + arm_end) * 0.5;

            // Arm oriented along face_normal.
            // Determine arm box half-extents in local space.
            let (half_x, half_y, half_z) = if face_normal.x.abs() > 0.5 {
                (arm_length / 2.0, aw / 2.0, at / 2.0)
            } else if face_normal.y.abs() > 0.5 {
                (aw / 2.0, arm_length / 2.0, at / 2.0)
            } else {
                (aw / 2.0, at / 2.0, arm_length / 2.0)
            };

            let arm: Arc<dyn Sdf> = Arc::new(Translate::new(
                Arc::new(SdfBox::new(Vec3::new(half_x, half_y, half_z))),
                arm_mid,
            ));

            // One beam per hole connecting to attachment.
            let mut body: Arc<dyn Sdf> = arm;
            for hole in holes {
                let beam_mid = (hole.position + arm_end) * 0.5;
                let beam_len = (arm_end - hole.position).length();
                let beam_half = Vec3::new(aw / 2.0, at / 2.0, beam_len / 2.0);
                let beam: Arc<dyn Sdf> = Arc::new(Translate::new(
                    Arc::new(SdfBox::new(beam_half)),
                    beam_mid,
                ));
                body = Arc::new(Union::new(body, beam));
            }

            BracketGeometry {
                bracket_body:   body,
                void_in_parent: Arc::new(NullSdf),
            }
        }

        BracketType::FullTray { wall_thickness, floor_thickness, open_face } => {
            let wt = *wall_thickness;
            let ft = *floor_thickness;
            let open_normal = open_face.normal();

            // Outer box.
            let outer_half = Vec3::new(
                size.x / 2.0 + wt,
                size.y / 2.0 + wt,
                size.z / 2.0 + ft,
            );
            let outer: Arc<dyn Sdf> = Arc::new(Translate::new(
                Arc::new(SdfBox::new(outer_half)),
                midpoint,
            ));

            // Subtract keepout interior.
            let hollowed: Arc<dyn Sdf> = Arc::new(Subtract::new(outer, keepout));

            // Subtract open face: half-space on the open face side.
            // A point p is in the half-space if (p - face_center) · open_normal > 0.
            // We model this as a large offset box on the open side.
            let open_face_center = midpoint + open_normal * (size * 0.5 + Vec3::splat(wt)).dot(open_normal.abs());
            let cut_size = 500.0_f32;
            let cut_half = Vec3::splat(cut_size / 2.0);
            let cut_center = open_face_center + open_normal * (cut_size / 2.0);
            let cut: Arc<dyn Sdf> = Arc::new(Translate::new(
                Arc::new(SdfBox::new(cut_half)),
                cut_center,
            ));

            let body: Arc<dyn Sdf> = Arc::new(Subtract::new(hollowed, cut));

            BracketGeometry {
                bracket_body:   body,
                void_in_parent: Arc::new(NullSdf),
            }
        }
    }
}

// ── Screw hole application ────────────────────────────────────────────────────

/// Apply heat-set bosses to the bracket and clearance holes to the parent.
/// Returns (modified_bracket, modified_parent).
pub fn apply_screw_holes(
    bracket: Arc<dyn Sdf>,
    parent:  Arc<dyn Sdf>,
    holes:   &[BracketHole],
) -> (Arc<dyn Sdf>, Arc<dyn Sdf>) {
    let default_spec = get_spec("M3").unwrap();
    let mut b = bracket;
    let mut p = parent;

    for hole in holes {
        let spec = get_spec(&hole.designation).unwrap_or(default_spec);

        // Heat-set boss on bracket.
        let (void, boss) = heat_set_boss(spec, hole.direction, hole.position);
        b = check_and_pad(b, void, boss, hole.position, hole.direction, spec);

        // Clearance hole through parent.
        let (pvoid, pboss) = clearance_hole(spec, 20.0, hole.direction, hole.position);
        p = check_and_pad(p, pvoid, pboss, hole.position, hole.direction, spec);
    }

    (b, p)
}

// ── Auto bracket ─────────────────────────────────────────────────────────────

/// Generate a complete bracket: body + screw holes in both bracket and parent.
///
/// Returns (modified_parent, bracket_with_bosses).
pub fn auto_bracket(
    keepout:       Arc<dyn Sdf>,
    parent:        Arc<dyn Sdf>,
    holes:         &[BracketHole],
    bracket_type:  &BracketType,
    keepout_min:   Vec3,
    keepout_max:   Vec3,
) -> (Arc<dyn Sdf>, Arc<dyn Sdf>) {
    // If no holes given, generate 4 default M3 holes at corners of the bottom face.
    let default_holes;
    let holes_ref: &[BracketHole] = if holes.is_empty() {
        let mid = (keepout_min + keepout_max) * 0.5;
        let sx  = (keepout_max.x - keepout_min.x) / 2.0 * 0.8;
        let sy  = (keepout_max.y - keepout_min.y) / 2.0 * 0.8;
        let z   = keepout_min.z;
        default_holes = vec![
            BracketHole { position: Vec3::new(mid.x + sx, mid.y + sy, z), direction: Vec3::Z, designation: "M3".into() },
            BracketHole { position: Vec3::new(mid.x - sx, mid.y + sy, z), direction: Vec3::Z, designation: "M3".into() },
            BracketHole { position: Vec3::new(mid.x + sx, mid.y - sy, z), direction: Vec3::Z, designation: "M3".into() },
            BracketHole { position: Vec3::new(mid.x - sx, mid.y - sy, z), direction: Vec3::Z, designation: "M3".into() },
        ];
        &default_holes
    } else {
        holes
    };

    let geo = generate_bracket_body(
        Arc::clone(&keepout),
        bracket_type,
        holes_ref,
        keepout_min,
        keepout_max,
    );

    let (bracket_final, parent_final) = apply_screw_holes(
        geo.bracket_body,
        parent,
        holes_ref,
    );

    (parent_final, bracket_final)
}
