// Mechanical pattern convenience functions for the scripting API.
//
// These compose existing SDF primitives to produce common fastener and
// machining features: bolt circles, countersinks, counterbores, slots, etc.

use std::sync::Arc;
use glam::{Vec2, Vec3, Vec3Swizzles};

use crate::sdf::Sdf;
use crate::sdf::primitives::{Cylinder, SdfBox, Torus};
use crate::sdf::booleans::{Union, Subtract};
use crate::sdf::transforms::Translate;
use crate::sdf::patterns::PolarArray;

// ── Capped cone (cone frustum) SDF ───────────────────────────────────────────
//
// Tip is along the Z axis. The frustum extends from z = -h (radius r1)
// to z = +h (radius r2). Follows the Inigo Quilez sd_capped_cone formula.

pub struct CappedCone {
    pub r1: f32,   // radius at z = -h (bottom)
    pub r2: f32,   // radius at z = +h (top)
    pub h:  f32,   // half height
}

impl Sdf for CappedCone {
    fn distance(&self, p: Vec3) -> f32 {
        let q  = Vec2::new(p.xy().length(), p.z);
        let k1 = Vec2::new(self.r2, self.h);
        let k2 = Vec2::new(self.r2 - self.r1, 2.0 * self.h);

        let ca = Vec2::new(
            q.x - q.x.min(if q.y < 0.0 { self.r1 } else { self.r2 }),
            q.y.abs() - self.h,
        );
        let t  = ((k1 - q).dot(k2) / k2.dot(k2)).clamp(0.0, 1.0);
        let cb = q - k1 + k2 * t;

        let s = if cb.x < 0.0 && ca.y < 0.0 { -1.0_f32 } else { 1.0_f32 };
        s * ca.dot(ca).min(cb.dot(cb)).sqrt()
    }
}

// ── bolt_circle ───────────────────────────────────────────────────────────────

/// A polar array of vertical through-cylinders centered on a pattern circle.
/// Ready to subtract from a plate or boss.
pub fn bolt_circle(
    hole_radius:    f32,
    pattern_radius: f32,
    count:          usize,
    depth:          f32,
) -> Arc<dyn Sdf> {
    let cyl    = Arc::new(Cylinder::new(hole_radius, depth / 2.0));
    let offset = Arc::new(Translate::new(cyl, Vec3::new(pattern_radius, 0.0, 0.0)));
    Arc::new(PolarArray::new(offset, count, Vec3::Z))
}

// ── bolt_square / bolt_rect ───────────────────────────────────────────────────

/// Four cylinders at the corners of a rectangle, centered at origin.
/// x_spacing and y_spacing are centre-to-centre distances.
/// Common FC pattern: bolt_square(1.5, 30.5, 30.5, 5.0)
pub fn bolt_rect(
    hole_radius: f32,
    x_spacing:   f32,
    y_spacing:   f32,
    depth:       f32,
) -> Arc<dyn Sdf> {
    let corner = |dx: f32, dy: f32| -> Arc<dyn Sdf> {
        Arc::new(Translate::new(
            Arc::new(Cylinder::new(hole_radius, depth / 2.0)),
            Vec3::new(dx, dy, 0.0),
        ))
    };
    let hx = x_spacing / 2.0;
    let hy = y_spacing / 2.0;
    let ab = Arc::new(Union::new(corner(hx, hy),  corner(-hx,  hy)));
    let cd = Arc::new(Union::new(corner(hx, -hy), corner(-hx, -hy)));
    Arc::new(Union::new(ab, cd))
}

/// Alias for bolt_rect — kept because bolt_square is the common name for
/// square FC patterns (e.g. 30.5×30.5).
pub fn bolt_square(
    hole_radius: f32,
    x_spacing:   f32,
    y_spacing:   f32,
    depth:       f32,
) -> Arc<dyn Sdf> {
    bolt_rect(hole_radius, x_spacing, y_spacing, depth)
}

// ── countersink ───────────────────────────────────────────────────────────────

/// A countersunk fastener hole ready to subtract from a plate.
///
/// The shape is:
///   - A cone frustum from radius `head_radius` at z = 0 (top) down to
///     `shaft_radius` at z = -head_depth.
///   - A cylinder of `shaft_radius` from z = -head_depth down to
///     z = -(head_depth + shaft_depth).
pub fn countersink(
    shaft_radius: f32,
    head_radius:  f32,
    head_depth:   f32,
    shaft_depth:  f32,
) -> Arc<dyn Sdf> {
    // Frustum: r1 = shaft_radius at bottom, r2 = head_radius at top, half_h = head_depth/2
    let frustum = Arc::new(Translate::new(
        Arc::new(CappedCone { r1: shaft_radius, r2: head_radius, h: head_depth / 2.0 }),
        Vec3::new(0.0, 0.0, -head_depth / 2.0),
    ));
    let shaft = Arc::new(Translate::new(
        Arc::new(Cylinder::new(shaft_radius, shaft_depth / 2.0)),
        Vec3::new(0.0, 0.0, -head_depth - shaft_depth / 2.0),
    ));
    Arc::new(Union::new(frustum, shaft))
}

// ── counterbore ───────────────────────────────────────────────────────────────

/// A counterbored fastener hole (socket-head cap screw) ready to subtract.
///
/// `bore_radius` × `bore_depth` sits above a `shaft_radius` × `shaft_depth`
/// pilot hole, both co-axial along Z, top of bore at z = 0.
pub fn counterbore(
    shaft_radius: f32,
    bore_radius:  f32,
    bore_depth:   f32,
    shaft_depth:  f32,
) -> Arc<dyn Sdf> {
    let bore = Arc::new(Translate::new(
        Arc::new(Cylinder::new(bore_radius, bore_depth / 2.0)),
        Vec3::new(0.0, 0.0, -bore_depth / 2.0),
    ));
    let shaft = Arc::new(Translate::new(
        Arc::new(Cylinder::new(shaft_radius, shaft_depth / 2.0)),
        Vec3::new(0.0, 0.0, -bore_depth - shaft_depth / 2.0),
    ));
    Arc::new(Union::new(bore, shaft))
}

// ── slot ──────────────────────────────────────────────────────────────────────

/// A rounded slot (stadium profile extruded along Z).
/// Length runs along the X axis, width sets the diameter of the rounded ends.
/// The slot is centered at the origin.
pub fn slot(width: f32, length: f32, depth: f32) -> Arc<dyn Sdf> {
    let inner_len = (length - width).max(0.0);
    // Centre bar: rectangular section between the two caps.
    let bar = Arc::new(SdfBox::new(Vec3::new(
        inner_len / 2.0,
        width / 2.0,
        depth / 2.0,
    )));
    // Two semicircular end caps.
    let cap: Arc<dyn Sdf> = Arc::new(Cylinder::new(width / 2.0, depth / 2.0));
    let cap_l = Arc::new(Translate::new(Arc::clone(&cap), Vec3::new( inner_len / 2.0, 0.0, 0.0)));
    let cap_r = Arc::new(Translate::new(cap,              Vec3::new(-inner_len / 2.0, 0.0, 0.0)));
    Arc::new(Union::new(bar, Arc::new(Union::new(cap_l, cap_r))))
}

// ── chamfer_edge ──────────────────────────────────────────────────────────────

/// Approximate a chamfer on convex edges by shrinking then expanding the body.
///
/// This is an approximation that works best on convex edges.  It is equivalent
/// to `intersect(body, offset(body, -distance))` which contracts the body and
/// bevels sharp convex corners.  Concave edges are unaffected.
pub fn chamfer_edge(body: Arc<dyn Sdf>, distance: f32) -> Arc<dyn Sdf> {
    use crate::sdf::transforms::Offset;
    let contracted = Arc::new(Offset::new(Arc::clone(&body), -distance));
    Arc::new(crate::sdf::booleans::Intersect::new(body, contracted))
}

// ── thread_hole ───────────────────────────────────────────────────────────────

/// Cosmetic threaded hole for visualization only.
///
/// NOT geometrically accurate — the helical geometry is approximated as
/// ring grooves at `pitch` spacing.  Do not use for functional thread
/// generation or any analysis that requires precise thread geometry.
pub fn thread_hole(radius: f32, pitch: f32, depth: f32) -> Arc<dyn Sdf> {
    let hole: Arc<dyn Sdf> = Arc::new(Cylinder::new(radius, depth / 2.0));
    let n_grooves = ((depth / pitch).floor() as usize).max(1);
    let minor_r   = pitch * 0.12;          // groove depth
    let groove_r  = radius + minor_r * 0.5; // torus major radius sits just at the wall

    let mut result = hole;
    for i in 0..n_grooves {
        let z = -depth / 2.0 + (i as f32 + 0.5) * pitch;
        let ring = Arc::new(Translate::new(
            Arc::new(Torus::new(groove_r, minor_r)),
            Vec3::new(0.0, 0.0, z),
        ));
        result = Arc::new(Union::new(result, ring));
    }
    result
}

// ── fc_mount ──────────────────────────────────────────────────────────────────

/// A flight-controller mounting plate with bolt holes at `pattern_mm` square.
///
/// The plate is slightly larger than the bolt pattern.  Ready to union with a
/// frame body or to subtract from a standoff block.
/// Common patterns: 20, 25.5, 30.5 mm.
pub fn fc_mount(pattern_mm: f32, hole_radius: f32, plate_thickness: f32) -> Arc<dyn Sdf> {
    let pad    = hole_radius * 3.0;
    let half   = pattern_mm / 2.0 + pad;
    let plate: Arc<dyn Sdf> = Arc::new(SdfBox::new(Vec3::new(half, half, plate_thickness / 2.0)));
    let holes  = bolt_square(hole_radius, pattern_mm, pattern_mm, plate_thickness * 2.0);
    Arc::new(Subtract::new(plate, holes))
}

// ── motor_mount_pattern ───────────────────────────────────────────────────────

/// Bolt circle sized for common brushless motor mount patterns.
///
/// `motor_size_mm` refers to the motor stator diameter (first two digits of
/// the XXXX size code).  Look-up table for common sizes:
///
/// | Stator Ø (mm) | Common sizes              | Pattern PCD (mm) |
/// |---------------|---------------------------|------------------|
/// | 22            | 2203, 2204, 2205, 2206    | 9                |
/// | 23            | 2306, 2307, 2308          | 12               |
/// | 24            | 2407, 2408                | 12               |
/// | 28            | 2806, 2807, 2808          | 12               |
/// | 31            | 3110, 3115                | 16               |
/// | 35            | 3510, 3511, 3516          | 16               |
/// | 40            | 4014                      | 19               |
/// | 42            | 4215                      | 19               |
///
/// For unlisted sizes the pattern defaults to `motor_size_mm * 0.4`.
pub fn motor_mount_pattern(motor_size_mm: f32, hole_radius: f32, depth: f32) -> Arc<dyn Sdf> {
    let pcd = motor_pcd(motor_size_mm);
    bolt_circle(hole_radius, pcd / 2.0, 4, depth)
}

fn motor_pcd(stator_mm: f32) -> f32 {
    match stator_mm as u32 {
        ..=22  => 9.0,
        23..=24 => 12.0,
        25..=28 => 12.0,
        29..=35 => 16.0,
        36..=42 => 19.0,
        _       => stator_mm * 0.4,
    }
}
