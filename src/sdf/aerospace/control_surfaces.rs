// Control surface geometry for fixed-wing aircraft.
//
// A control surface is carved from the parent wing SDF using half-space Plane SDFs.
// Span direction is Y, chord direction is X, thickness direction is Z.
// span_start and span_end are absolute Y coordinates in world space.
// chord_fraction is the fraction of chord taken by the surface, measured from the trailing edge.

use std::sync::Arc;
use glam::Vec3;
use glam::Quat;
use crate::sdf::Sdf;
use crate::sdf::primitives::{Plane, SdfBox, Cylinder};
use crate::sdf::booleans::{Union, Subtract, Intersect, SmoothUnion};
use crate::sdf::transforms::{Translate, Rotate};

// ── Data model ────────────────────────────────────────────────────────────────

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ControlSurfaceType { Aileron, Elevator, Rudder, Flap, Elevon }

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum HingeType { SimpleGap, Rounded }

#[derive(Clone, Debug)]
pub struct HingeSpec {
    pub hinge_type:   HingeType,
    pub hinge_radius: f32,   // for Rounded, radius of cylindrical hinge line
    pub gap_width:    f32,   // air gap at hinge, default 0.5mm
}

impl HingeSpec {
    pub fn simple_gap(gap_width: f32) -> Self {
        Self { hinge_type: HingeType::SimpleGap, hinge_radius: 0.0, gap_width }
    }
    pub fn rounded(radius: f32, gap_width: f32) -> Self {
        Self { hinge_type: HingeType::Rounded, hinge_radius: radius, gap_width }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum HornSide { Upper, Lower }

#[derive(Clone, Debug)]
pub struct ControlHornSpec {
    pub horn_height:            f32,   // height of horn above hinge in mm
    pub horn_width:             f32,   // width of horn base in mm (span direction)
    pub hole_diameter:          f32,   // clevis pin hole, default 2.0mm
    pub hole_offset:            f32,   // distance from hinge line to hole center
    pub position_span_fraction: f32,   // fraction along the surface span [0, 1]
    pub side:                   HornSide,
}

impl ControlHornSpec {
    pub fn default_upper(horn_height: f32, hole_offset: f32, span_fraction: f32) -> Self {
        Self { horn_height, horn_width: 8.0, hole_diameter: 2.0, hole_offset, position_span_fraction: span_fraction, side: HornSide::Upper }
    }
    pub fn default_lower(horn_height: f32, hole_offset: f32, span_fraction: f32) -> Self {
        Self { horn_height, horn_width: 8.0, hole_diameter: 2.0, hole_offset, position_span_fraction: span_fraction, side: HornSide::Lower }
    }
}

#[derive(Clone, Debug)]
#[allow(dead_code)]
pub struct PushrodSlotSpec {
    pub slot_width:             f32,    // default 3.0mm
    pub slot_length:            f32,    // default 15.0mm
    pub position_span_fraction: f32,
    pub exit_direction:         Vec3,
}

#[derive(Clone, Debug)]
pub struct LinkageSpec {
    pub control_horn: Option<ControlHornSpec>,
    pub pushrod_slot: Option<PushrodSlotSpec>,
}

impl LinkageSpec {
    pub fn none() -> Self { Self { control_horn: None, pushrod_slot: None } }
    pub fn horn(spec: ControlHornSpec) -> Self { Self { control_horn: Some(spec), pushrod_slot: None } }
}

#[allow(dead_code)]
pub struct ControlSurface {
    pub surface_type:   ControlSurfaceType,
    pub parent_wing:    Arc<dyn Sdf>,
    pub span_start:     f32,    // absolute Y position
    pub span_end:       f32,    // absolute Y position
    pub chord_fraction: f32,    // fraction of chord from trailing edge
    pub hinge:          HingeSpec,
    pub linkage:        LinkageSpec,
}

#[allow(dead_code)]
pub struct ControlSurfaceResult {
    pub control_surface: Arc<dyn Sdf>,
    pub modified_parent: Arc<dyn Sdf>,
    pub hinge_line:      (Vec3, Vec3),  // start and end points of hinge line
    pub horn_position:   Option<Vec3>,
}

// ── Geometry helpers ──────────────────────────────────────────────────────────

/// Estimate the X extent of the wing SDF at a given Y position (span station).
/// Returns (x_leading_edge, x_trailing_edge) by scanning along X at z=0.
/// Falls back to (0.0, 100.0) if no material found.
pub fn estimate_chord_x_range(sdf: &dyn Sdf, y: f32) -> (f32, f32) {
    let mut x_min = f32::MAX;
    let mut x_max = f32::MIN;
    let step = 1.0_f32;
    let mut x = -30.0_f32;
    while x <= 400.0 {
        if sdf.distance(Vec3::new(x, y, 0.0)) < 0.0 {
            if x < x_min { x_min = x; }
            if x > x_max { x_max = x; }
        }
        x += step;
    }
    if x_min < x_max { (x_min, x_max) } else { (0.0, 100.0) }
}

/// Estimate the positive Y half-span by marching outward from Y=0 until material ends.
pub fn estimate_half_span(sdf: &dyn Sdf) -> f32 {
    let mut y = 0.0_f32;
    loop {
        y += 5.0;
        if y > 5000.0 { break; }
        // Sample several X positions to check for material
        let has_material = [-20.0_f32, 0.0, 20.0, 50.0, 100.0].iter()
            .any(|&x| sdf.distance(Vec3::new(x, y, 0.0)) < 0.0);
        if !has_material { break; }
    }
    (y - 5.0).max(5.0)
}

// ── Main geometry builder ─────────────────────────────────────────────────────

pub fn build_control_surface(spec: &ControlSurface) -> ControlSurfaceResult {
    let parent    = &spec.parent_wing;
    let y_start   = spec.span_start.min(spec.span_end);
    let y_end     = spec.span_start.max(spec.span_end);
    let y_mid     = (y_start + y_end) * 0.5;
    let span_ext  = (y_end - y_start) * 0.5 + 1.0;  // half-extent + margin

    // Estimate chord extent at midspan to find hinge X position
    let (x_le, x_te) = estimate_chord_x_range(parent.as_ref(), y_mid);
    let chord    = (x_te - x_le).max(1.0);
    let x_hinge  = x_le + chord * (1.0 - spec.chord_fraction);

    // Hinge line endpoints (world space)
    let (x_le_s, x_te_s) = estimate_chord_x_range(parent.as_ref(), y_start);
    let (x_le_e, x_te_e) = estimate_chord_x_range(parent.as_ref(), y_end);
    let chord_s  = (x_te_s - x_le_s).max(1.0);
    let chord_e  = (x_te_e - x_le_e).max(1.0);
    let x_h_s    = x_le_s + chord_s * (1.0 - spec.chord_fraction);
    let x_h_e    = x_le_e + chord_e * (1.0 - spec.chord_fraction);
    let hinge_start = Vec3::new(x_h_s, y_start, 0.0);
    let hinge_end   = Vec3::new(x_h_e, y_end,   0.0);

    // Step 1 & 2 — Raw control surface volume: aft of hinge, within span
    // Aft half-space: x > x_hinge → Plane normal=-X, distance=-x_hinge
    let aft_plane: Arc<dyn Sdf> = Arc::new(Plane { normal: Vec3::NEG_X, distance: -x_hinge });
    // Span lower: y > y_start → Plane normal=-Y, distance=-y_start
    let span_lo:   Arc<dyn Sdf> = Arc::new(Plane { normal: Vec3::NEG_Y, distance: -y_start });
    // Span upper: y < y_end → Plane normal=+Y, distance=y_end
    let span_hi:   Arc<dyn Sdf> = Arc::new(Plane { normal: Vec3::Y,    distance:  y_end   });

    // cs_raw = parent ∩ aft_plane ∩ span_lo ∩ span_hi
    let cs_aft: Arc<dyn Sdf> = Arc::new(Intersect::new(parent.clone(), aft_plane));
    let cs_lo:  Arc<dyn Sdf> = Arc::new(Intersect::new(cs_aft, span_lo));
    let cs_raw: Arc<dyn Sdf> = Arc::new(Intersect::new(cs_lo, span_hi.clone()));

    // Step 3 — Apply hinge geometry
    let gap = spec.hinge.gap_width.max(0.1);

    let (control_surface, modified_parent): (Arc<dyn Sdf>, Arc<dyn Sdf>) = match spec.hinge.hinge_type {
        HingeType::SimpleGap => {
            // Gap slab: thin box of width=gap centered on the hinge plane
            let gap_slab: Arc<dyn Sdf> = Arc::new(Translate::new(
                Arc::new(SdfBox::new(Vec3::new(gap * 0.5, span_ext, 50.0))),
                Vec3::new(x_hinge, y_mid, 0.0),
            ));
            // cs = cs_raw - gap_slab (trim leading edge)
            let cs: Arc<dyn Sdf> = Arc::new(Subtract::new(cs_raw.clone(), gap_slab.clone()));
            // parent_mod = parent - cs_raw - gap_slab
            let parent_cut: Arc<dyn Sdf> = Arc::new(Union::new(cs_raw, gap_slab));
            let par: Arc<dyn Sdf> = Arc::new(Subtract::new(parent.clone(), parent_cut));
            (cs, par)
        }
        HingeType::Rounded => {
            let r = spec.hinge.hinge_radius.max(0.5);
            // Hinge cylinder along Y axis at (x_hinge, y_mid, 0)
            let cyl_z: Arc<dyn Sdf> = Arc::new(Cylinder::new(r, span_ext));
            let cyl_y: Arc<dyn Sdf> = Arc::new(Rotate::new(
                cyl_z,
                Quat::from_rotation_x(std::f32::consts::FRAC_PI_2),
            ));
            let hinge_cyl: Arc<dyn Sdf> = Arc::new(Translate::new(cyl_y, Vec3::new(x_hinge, y_mid, 0.0)));

            // Slightly larger cylinder for the CS leading-edge socket
            let socket_r = r + gap;
            let sock_z: Arc<dyn Sdf> = Arc::new(Cylinder::new(socket_r, span_ext + 1.0));
            let sock_y: Arc<dyn Sdf> = Arc::new(Rotate::new(
                sock_z,
                Quat::from_rotation_x(std::f32::consts::FRAC_PI_2),
            ));
            let socket: Arc<dyn Sdf> = Arc::new(Translate::new(sock_y, Vec3::new(x_hinge, y_mid, 0.0)));

            // cs = cs_raw - socket (concave socket for hinge cylinder)
            let cs: Arc<dyn Sdf> = Arc::new(Subtract::new(cs_raw.clone(), socket));
            // parent_mod = parent - cs_raw (hinge cylinder stays on parent)
            let par_base: Arc<dyn Sdf> = Arc::new(Subtract::new(parent.clone(), cs_raw));
            // Subtract the hinge cylinder from parent (creates the cylinder channel in the wing)
            let par: Arc<dyn Sdf> = Arc::new(Subtract::new(par_base, hinge_cyl));
            (cs, par)
        }
    };

    // Step 4 — Apply control horn
    let mut horn_position: Option<Vec3> = None;
    let mut cs_final = control_surface;

    if let Some(horn_spec) = &spec.linkage.control_horn {
        let y_horn = y_start + horn_spec.position_span_fraction * (y_end - y_start);
        let z_dir  = match horn_spec.side { HornSide::Upper => 1.0_f32, HornSide::Lower => -1.0 };
        let hh = horn_spec.horn_height;
        let hw = horn_spec.horn_width;

        // Horn plate: (2mm thick in X) × (horn_width in Y) × (horn_height in Z)
        // Base at z=0, extends to z = z_dir * horn_height
        let horn_box: Arc<dyn Sdf> = Arc::new(Translate::new(
            Arc::new(SdfBox::new(Vec3::new(1.0, hw * 0.5, hh * 0.5))),
            Vec3::new(x_hinge + 1.0, y_horn, z_dir * hh * 0.5),
        ));

        // Clevis hole through the horn at hole_offset from hinge
        let hole_r = horn_spec.hole_diameter * 0.5;
        let z_hole = z_dir * horn_spec.hole_offset;
        // Hole is a cylinder along X through the horn
        let hole_cyl: Arc<dyn Sdf> = Arc::new(Cylinder::new(hole_r, 5.0));
        // Rotate to X-aligned: rotate around Y by 90°
        let hole_rotated: Arc<dyn Sdf> = Arc::new(Rotate::new(
            hole_cyl,
            Quat::from_rotation_y(std::f32::consts::FRAC_PI_2),
        ));
        let hole: Arc<dyn Sdf> = Arc::new(Translate::new(hole_rotated, Vec3::new(x_hinge + 1.0, y_horn, z_hole)));

        let horn_with_hole: Arc<dyn Sdf> = Arc::new(Subtract::new(horn_box, hole));

        // Smooth union the horn into the CS
        cs_final = Arc::new(SmoothUnion::new(cs_final, horn_with_hole, 1.5));
        horn_position = Some(Vec3::new(x_hinge + 1.0, y_horn, z_hole));
    }

    // Step 5 — Apply pushrod slot to parent
    let mut par_final = modified_parent;

    if let Some(slot_spec) = &spec.linkage.pushrod_slot {
        let y_slot = y_start + slot_spec.position_span_fraction * (y_end - y_start);
        let slot_pos = Vec3::new(x_hinge - 5.0, y_slot, 0.0);
        let sw = slot_spec.slot_width;
        let sl = slot_spec.slot_length;
        let slot_box: Arc<dyn Sdf> = Arc::new(Translate::new(
            Arc::new(SdfBox::new(Vec3::new(sw * 0.5, sl * 0.5, 30.0))),
            slot_pos,
        ));
        par_final = Arc::new(Subtract::new(par_final, slot_box));
    }

    ControlSurfaceResult {
        control_surface: cs_final,
        modified_parent: par_final,
        hinge_line: (hinge_start, hinge_end),
        horn_position,
    }
}

// ── Convenience constructors ──────────────────────────────────────────────────

pub fn aileron(
    wing:           Arc<dyn Sdf>,
    span_start:     f32,
    span_end:       f32,
    chord_fraction: f32,
    hinge:          HingeSpec,
    linkage:        LinkageSpec,
) -> ControlSurfaceResult {
    let spec = ControlSurface {
        surface_type: ControlSurfaceType::Aileron,
        parent_wing: wing,
        span_start, span_end, chord_fraction,
        hinge, linkage,
    };
    build_control_surface(&spec)
}

pub fn elevator(
    stabilizer:     Arc<dyn Sdf>,
    chord_fraction: f32,
    hinge:          HingeSpec,
    linkage:        LinkageSpec,
) -> ControlSurfaceResult {
    let half_span = estimate_half_span(stabilizer.as_ref());
    let spec = ControlSurface {
        surface_type: ControlSurfaceType::Elevator,
        parent_wing: stabilizer,
        span_start: -half_span, span_end: half_span,
        chord_fraction, hinge, linkage,
    };
    build_control_surface(&spec)
}

pub fn rudder(
    fin:            Arc<dyn Sdf>,
    chord_fraction: f32,
    hinge:          HingeSpec,
    linkage:        LinkageSpec,
) -> ControlSurfaceResult {
    // Vertical fin: span along Y from 0 upward (or use half_span estimate)
    let half_span = estimate_half_span(fin.as_ref());
    let spec = ControlSurface {
        surface_type: ControlSurfaceType::Rudder,
        parent_wing: fin,
        span_start: 0.0, span_end: half_span,
        chord_fraction, hinge, linkage,
    };
    build_control_surface(&spec)
}

pub fn flap(
    wing:           Arc<dyn Sdf>,
    span_start:     f32,
    span_end:       f32,
    chord_fraction: f32,
    hinge:          HingeSpec,
) -> ControlSurfaceResult {
    let spec = ControlSurface {
        surface_type: ControlSurfaceType::Flap,
        parent_wing: wing,
        span_start, span_end, chord_fraction,
        hinge, linkage: LinkageSpec::none(),
    };
    build_control_surface(&spec)
}

pub fn elevon(
    wing:           Arc<dyn Sdf>,
    span_start:     f32,
    span_end:       f32,
    chord_fraction: f32,
    hinge:          HingeSpec,
    linkage:        LinkageSpec,
) -> ControlSurfaceResult {
    let spec = ControlSurface {
        surface_type: ControlSurfaceType::Elevon,
        parent_wing: wing,
        span_start, span_end, chord_fraction,
        hinge, linkage,
    };
    build_control_surface(&spec)
}

/// Apply symmetric ailerons to both halves of a full wing.
/// span_start and span_end are normalized [0,1] fractions of the half-span.
/// Returns (pos_aileron, neg_aileron, modified_wing) where pos is the +Y side.
pub fn wing_with_ailerons(
    wing:           Arc<dyn Sdf>,
    span_start:     f32,
    span_end:       f32,
    chord_fraction: f32,
    hinge:          HingeSpec,
    linkage:        LinkageSpec,
) -> (Arc<dyn Sdf>, Arc<dyn Sdf>, Arc<dyn Sdf>) {
    let half_span = estimate_half_span(wing.as_ref());
    let y_s = span_start * half_span;
    let y_e = span_end   * half_span;

    // Positive Y side
    let pos_result = aileron(
        wing.clone(), y_s, y_e, chord_fraction,
        hinge.clone(), linkage.clone(),
    );
    // Negative Y side (mirror)
    let neg_result = aileron(
        pos_result.modified_parent.clone(), -y_e, -y_s, chord_fraction,
        hinge, linkage,
    );

    (pos_result.control_surface, neg_result.control_surface, neg_result.modified_parent)
}
