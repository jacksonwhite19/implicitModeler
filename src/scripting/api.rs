// Rhai API registration

use std::sync::Arc;
use rhai::Engine;
use glam::{Vec3, Quat};
use crate::sdf::primitives::{Sphere, SdfBox, Cylinder, Torus, Cone, Plane};
use crate::sdf::booleans::{Union, Subtract, Intersect, SmoothUnion, SmoothSubtract, SmoothIntersect};
use crate::sdf::transforms::{Translate, Rotate, Scale, Offset, Shell, Twist, Bend};
use crate::sdf::patterns::{LinearArray, PolarArray, Mirror};
use crate::sdf::aerospace::{
    Airfoil, get_naca_airfoil, ExtrudedAirfoil, wing_with_airfoil, wing_from_sections,
    fuselage_parametric, nacelle_simple, is_valid_naca_4digit,
    CrossSection, LoftedFuselage, Section2D,
    rib_slab, spar_cylinder,
    bulkhead_at_station, lightening_hole_pattern,
    rod_mount, motor_arm, motor_mount, generate_mounts_sdf,
};
use crate::sdf::field::{
    primitives::{ConstantField, SdfField, PositionXField, PositionYField, PositionZField},
    arithmetic::{FieldAdd, FieldMultiply, FieldMin, FieldMax, FieldAbs},
    gradients::{GradientField, RadialField, AxialRadialField},
    operations::{OffsetByField, ShellWithField, BlendByField},
    lattice::{GyroidLattice, CubicLattice, DiamondLattice, GyroidWithField},
};
use crate::sdf::lattice::{ConformalGyroid, ConformalDiamond, ConformalSchwarzP};
use std::sync::{Mutex, RwLock};
use std::collections::HashMap;
use crate::sdf::profiles::SplineProfile;
use crate::sdf::spine::LongitudinalSplines;
use super::{SdfHandle, FieldHandle, MassPoint, ComponentHandle, SectionHandle, StationHandle};

pub fn register_sdf_functions(engine: &mut Engine) {
    // Register the SdfHandle and FieldHandle types
    engine.register_type::<SdfHandle>();
    engine.register_type::<FieldHandle>();
    engine.register_type::<ComponentHandle>();
    engine.register_type::<SectionHandle>();
    engine.register_type::<StationHandle>();

    // Register all SDF operations
    register_primitives(engine);
    register_booleans(engine);
    register_transforms(engine);
    register_patterns(engine);
    register_math_extras(engine);
    register_aerospace_functions(engine);
    register_field_functions(engine);
    register_lattices(engine);
}

fn register_primitives(engine: &mut Engine) {
    // Sphere
    engine.register_fn("sphere", |radius: f64| {
        SdfHandle(Arc::new(Sphere::new(radius as f32)))
    });

    // Box (using box_ since box is a keyword)
    engine.register_fn("box_", |width: f64, height: f64, depth: f64| {
        let half_extents = Vec3::new(width as f32 / 2.0, height as f32 / 2.0, depth as f32 / 2.0);
        SdfHandle(Arc::new(SdfBox::new(half_extents)))
    });

    // Cylinder
    engine.register_fn("cylinder", |radius: f64, height: f64| {
        SdfHandle(Arc::new(Cylinder::new(radius as f32, height as f32 / 2.0)))
    });

    // Torus
    engine.register_fn("torus", |major_radius: f64, minor_radius: f64| {
        SdfHandle(Arc::new(Torus::new(major_radius as f32, minor_radius as f32)))
    });

    // Cone
    engine.register_fn("cone", |radius: f64, height: f64| {
        SdfHandle(Arc::new(Cone::new(radius as f32, height as f32)))
    });

    // Plane
    engine.register_fn("plane", |nx: f64, ny: f64, nz: f64, distance: f64| {
        let normal = Vec3::new(nx as f32, ny as f32, nz as f32);
        SdfHandle(Arc::new(Plane::new(normal, distance as f32)))
    });
}

fn register_booleans(engine: &mut Engine) {
    // Union
    engine.register_fn("union", |a: SdfHandle, b: SdfHandle| {
        SdfHandle(Arc::new(Union::new(a.0, b.0)))
    });

    // Subtract
    engine.register_fn("subtract", |a: SdfHandle, b: SdfHandle| {
        SdfHandle(Arc::new(Subtract::new(a.0, b.0)))
    });

    // Intersect
    engine.register_fn("intersect", |a: SdfHandle, b: SdfHandle| {
        SdfHandle(Arc::new(Intersect::new(a.0, b.0)))
    });

    // Smooth Union
    engine.register_fn("smooth_union", |a: SdfHandle, b: SdfHandle, smoothness: f64| {
        SdfHandle(Arc::new(SmoothUnion::new(a.0, b.0, smoothness as f32)))
    });

    // Smooth Subtract — removes tool from base with a smooth chamfer/fillet
    // k controls blend radius (larger = softer transition)
    engine.register_fn("smooth_subtract", |base: SdfHandle, tool: SdfHandle, k: f64| {
        SdfHandle(Arc::new(SmoothSubtract::new(base.0, tool.0, k as f32)))
    });

    // Smooth Intersect — intersection with polynomial smooth maximum
    engine.register_fn("smooth_intersect", |a: SdfHandle, b: SdfHandle, k: f64| {
        SdfHandle(Arc::new(SmoothIntersect::new(a.0, b.0, k as f32)))
    });
}

fn register_transforms(engine: &mut Engine) {
    // Translate
    engine.register_fn("translate", |body: SdfHandle, x: f64, y: f64, z: f64| {
        let offset = Vec3::new(x as f32, y as f32, z as f32);
        SdfHandle(Arc::new(Translate::new(body.0, offset)))
    });

    // Rotate (takes degrees, converts to quaternion)
    engine.register_fn("rotate", |body: SdfHandle, rx: f64, ry: f64, rz: f64| {
        let rotation = Quat::from_euler(
            glam::EulerRot::XYZ,
            (rx as f32).to_radians(),
            (ry as f32).to_radians(),
            (rz as f32).to_radians(),
        );
        SdfHandle(Arc::new(Rotate::new(body.0, rotation)))
    });

    // Scale
    engine.register_fn("scale", |body: SdfHandle, sx: f64, sy: f64, sz: f64| {
        let scale = Vec3::new(sx as f32, sy as f32, sz as f32);
        SdfHandle(Arc::new(Scale::new(body.0, scale)))
    });

    // Offset
    engine.register_fn("offset", |body: SdfHandle, distance: f64| {
        SdfHandle(Arc::new(Offset::new(body.0, distance as f32)))
    });

    // Shell
    engine.register_fn("shell", |body: SdfHandle, thickness: f64| {
        SdfHandle(Arc::new(Shell::new(body.0, thickness as f32)))
    });

    // Twist — rotate each cross-section by `rate` degrees per unit along axis (ax, ay, az).
    // Approximate SDF: Lipschitz-1 is not guaranteed under strong twist; safe for
    // raymarching and marching cubes at moderate deformation, not for precise offsets.
    engine.register_fn("twist", |body: SdfHandle, ax: f64, ay: f64, az: f64, rate: f64| {
        let axis = Vec3::new(ax as f32, ay as f32, az as f32);
        SdfHandle(Arc::new(Twist::new(body.0, axis, rate as f32)))
    });

    // Bend — curve the shape along axis (ax, ay, az) by `curvature` radians per unit length.
    // The bend plane is determined from the axis and world-Y (or world-Z near Y).
    // Approximate SDF: same caveat as twist.
    engine.register_fn("bend", |body: SdfHandle, ax: f64, ay: f64, az: f64, curvature: f64| {
        let axis = Vec3::new(ax as f32, ay as f32, az as f32);
        SdfHandle(Arc::new(Bend::new(body.0, axis, curvature as f32)))
    });
}

fn register_patterns(engine: &mut Engine) {
    // Linear array: N evenly-spaced copies offset by (dx, dy, dz) per step
    // e.g. linear_array(cylinder(2.0, 10.0), 5, 10.0, 0.0, 0.0) → 5 cylinders in a row
    engine.register_fn("linear_array", |body: SdfHandle, count: i64, dx: f64, dy: f64, dz: f64| {
        let spacing = Vec3::new(dx as f32, dy as f32, dz as f32);
        SdfHandle(Arc::new(LinearArray::new(body.0, count as usize, spacing)))
    });

    // Polar array: N copies evenly rotated around Z axis
    // e.g. polar_array(cylinder(2.0, 10.0), 6) → 6 cylinders in a circle
    engine.register_fn("polar_array", |body: SdfHandle, count: i64| {
        SdfHandle(Arc::new(PolarArray::new(body.0, count as usize, Vec3::Z)))
    });

    // Polar array with custom axis
    engine.register_fn("polar_array_axis", |body: SdfHandle, count: i64, ax: f64, ay: f64, az: f64| {
        let axis = Vec3::new(ax as f32, ay as f32, az as f32);
        SdfHandle(Arc::new(PolarArray::new(body.0, count as usize, axis)))
    });

    // Mirror across YZ plane (flip X)
    engine.register_fn("mirror_x", |body: SdfHandle| {
        SdfHandle(Arc::new(Mirror::new(body.0, Vec3::X)))
    });

    // Mirror across XZ plane (flip Y)
    engine.register_fn("mirror_y", |body: SdfHandle| {
        SdfHandle(Arc::new(Mirror::new(body.0, Vec3::Y)))
    });

    // Mirror across XY plane (flip Z)
    engine.register_fn("mirror_z", |body: SdfHandle| {
        SdfHandle(Arc::new(Mirror::new(body.0, Vec3::Z)))
    });
}

fn register_math_extras(engine: &mut Engine) {
    // Constants — expose PI, TAU, E as global variables in scripts
    let mut math_module = rhai::Module::new();
    math_module.set_var("PI",  std::f64::consts::PI);
    math_module.set_var("TAU", std::f64::consts::TAU);
    math_module.set_var("E",   std::f64::consts::E);
    engine.register_global_module(math_module.into());

    // Degree/radian helpers (Rhai trig is in radians, but designers think in degrees)
    engine.register_fn("to_rad", |deg: f64| deg.to_radians());
    engine.register_fn("to_deg", |rad: f64| rad.to_degrees());

    // Clamp and lerp
    engine.register_fn("clamp", |v: f64, lo: f64, hi: f64| v.clamp(lo, hi));
    engine.register_fn("lerp",  |a: f64, b: f64, t: f64| a + (b - a) * t);
}

fn register_aerospace_functions(engine: &mut Engine) {
    // NACA airfoil - returns simple extruded airfoil
    engine.register_fn("naca",
        |designation: &str, chord: f64| -> Result<SdfHandle, Box<rhai::EvalAltResult>> {
        if !is_valid_naca_4digit(designation) {
            return Err(format!("Invalid NACA designation '{}': must be exactly 4 digits (e.g. \"0012\", \"2412\")", designation).into());
        }
        let airfoil = get_naca_airfoil(designation, chord as f32);
        let extruded = ExtrudedAirfoil::new(airfoil, 1.0);  // Unit span for simple extrusion
        Ok(SdfHandle(Arc::new(extruded)))
    });

    // Parametric wing with full geometric control
    engine.register_fn("wing_with_airfoil",
        |airfoil: &str, root_chord: f64, tip_chord: f64, span: f64,
         sweep: f64, dihedral: f64, twist: f64| -> Result<SdfHandle, Box<rhai::EvalAltResult>> {
        if !is_valid_naca_4digit(airfoil) {
            return Err(format!("Invalid NACA designation '{}': must be exactly 4 digits (e.g. \"0012\", \"2412\")", airfoil).into());
        }
        let wing = wing_with_airfoil(
            airfoil,
            root_chord as f32,
            tip_chord as f32,
            span as f32,
            sweep as f32,
            dihedral as f32,
            twist as f32
        );
        Ok(SdfHandle(Arc::new(wing)))
    });

    // Blend wrapper - convenient alias for smooth_union
    engine.register_fn("blend", |a: SdfHandle, b: SdfHandle, radius: f64| {
        SdfHandle(Arc::new(SmoothUnion::new(a.0, b.0, radius as f32)))
    });

    // Parametric fuselage
    engine.register_fn("fuselage_parametric",
        |length: f64, diameter: f64, nose: f64, tail: f64| {
        let fuse = fuselage_parametric(
            length as f32,
            diameter as f32,
            nose as f32,
            tail as f32
        );
        SdfHandle(Arc::new(fuse))
    });

    // Simple nacelle primitive
    engine.register_fn("nacelle",
        |length: f64, diameter: f64, inlet: f64, exhaust: f64| {
        let nac = nacelle_simple(
            length as f32,
            diameter as f32,
            inlet as f32,
            exhaust as f32
        );
        SdfHandle(nac)
    });

    // --- Primary fuselage API ---

    // fuselage(stations) — build a lofted fuselage from [position, section] pairs.
    // Positions are normalized [0, 1]. Stations are sorted automatically.
    // The SDF occupies x ∈ [0, 1]; use scale() to set physical length and cross-section size.
    //
    // Example:
    //   let f = fuselage([
    //       [0.0,  circle_section(0.05)],
    //       [0.15, circle_section(0.5)],
    //       [0.8,  circle_section(0.5)],
    //       [1.0,  circle_section(0.08)],
    //   ]);
    //   scale(f, 10.0, 1.0, 1.0)   // → 10 units long, sections unchanged
    engine.register_fn("fuselage",
        |stations: rhai::Array| -> Result<SdfHandle, Box<rhai::EvalAltResult>> {
        if stations.len() < 2 {
            return Err("fuselage requires at least 2 [position, section] pairs".into());
        }
        let mut pairs: Vec<(f32, Arc<dyn Section2D>)> = Vec::with_capacity(stations.len());
        for (i, item) in stations.into_iter().enumerate() {
            let pair = item.try_cast::<rhai::Array>()
                .ok_or_else(|| -> Box<rhai::EvalAltResult> {
                    format!("fuselage: item {} must be [position, section]", i).into()
                })?;
            if pair.len() < 2 {
                return Err(format!(
                    "fuselage: item {} must be [position, section] (got {} element(s))", i, pair.len()
                ).into());
            }
            let pos = pair[0].as_float().map_err(|_| -> Box<rhai::EvalAltResult> {
                format!("fuselage: position in item {} must be a number", i).into()
            })? as f32;
            if !(0.0..=1.0).contains(&pos) {
                return Err(format!(
                    "fuselage: position in item {} ({}) must be in [0, 1]", i, pos
                ).into());
            }
            let section = pair[1].clone().try_cast::<SectionHandle>()
                .ok_or_else(|| -> Box<rhai::EvalAltResult> {
                    format!("fuselage: section in item {} must be a SectionHandle \
                             (use circle_section(), ellipse_section(), airfoil_from_points(), etc.)", i).into()
                })?;
            pairs.push((pos, section.0));
        }
        // from_stations sorts internally, so unsorted input is fine
        Ok(SdfHandle(Arc::new(LoftedFuselage::from_stations(pairs, 1.0))))
    });

    // --- Multi-station fuselage API (legacy — use fuselage() above instead) ---

    // Circular cross-section by radius
    engine.register_fn("circle_section", |radius: f64| {
        SectionHandle(Arc::new(CrossSection::Circle { radius: radius as f32 }))
    });

    // Elliptical cross-section by width and height
    engine.register_fn("ellipse_section", |width: f64, height: f64| {
        SectionHandle(Arc::new(CrossSection::Ellipse {
            width:  width  as f32,
            height: height as f32,
        }))
    });

    // Station at an absolute X position with a cross-section shape
    engine.register_fn("fuselage_station", |position: f64, section: SectionHandle| {
        StationHandle { position: position as f32, section: section.0 }
    });

    // Build a lofted fuselage from an array of StationHandles.
    // Stations must be sorted by ascending position (absolute X coords).
    // Length is derived as the maximum station position.
    engine.register_fn("lofted_fuselage",
        |stations: rhai::Array| -> Result<SdfHandle, Box<rhai::EvalAltResult>> {
        if stations.len() < 2 {
            return Err("lofted_fuselage requires at least 2 stations".into());
        }
        let mut handles: Vec<StationHandle> = Vec::with_capacity(stations.len());
        for (i, item) in stations.into_iter().enumerate() {
            match item.try_cast::<StationHandle>() {
                Some(s) => handles.push(s),
                None => return Err(format!(
                    "lofted_fuselage: item at index {} is not a StationHandle (use fuselage_station())", i
                ).into()),
            }
        }
        for i in 1..handles.len() {
            if handles[i].position <= handles[i - 1].position {
                return Err(format!(
                    "lofted_fuselage: stations must be in strictly ascending order (index {} has position {} <= {})",
                    i, handles[i].position, handles[i - 1].position
                ).into());
            }
        }
        let length = handles.last().unwrap().position;
        if length <= 0.0 {
            return Err("lofted_fuselage: final station position must be > 0".into());
        }
        let pairs: Vec<(f32, Arc<dyn Section2D>)> = handles
            .into_iter()
            .map(|s| (s.position / length, s.section))
            .collect();
        Ok(SdfHandle(Arc::new(LoftedFuselage::from_stations(pairs, length))))
    });

    // --- Custom airfoil input ---

    // Build a Section2D from a Rhai array of [x, y] coordinate pairs (normalized 0-1).
    // `chord` is the physical chord length in model units.
    engine.register_fn("airfoil_from_points",
        |points: rhai::Array, chord: f64| -> Result<SectionHandle, Box<rhai::EvalAltResult>> {
        if points.len() < 3 {
            return Err("airfoil_from_points requires at least 3 coordinate pairs".into());
        }
        let mut pts: Vec<(f32, f32)> = Vec::with_capacity(points.len());
        for (i, item) in points.into_iter().enumerate() {
            let pair = item.try_cast::<rhai::Array>()
                .ok_or_else(|| -> Box<rhai::EvalAltResult> {
                    format!("airfoil_from_points: item at index {} must be an [x, y] array", i).into()
                })?;
            if pair.len() < 2 {
                return Err(format!(
                    "airfoil_from_points: item at index {} needs at least 2 elements", i
                ).into());
            }
            let x = pair[0].as_float().map_err(|_| -> Box<rhai::EvalAltResult> {
                format!("airfoil_from_points: x at index {} must be a number", i).into()
            })? as f32;
            let y = pair[1].as_float().map_err(|_| -> Box<rhai::EvalAltResult> {
                format!("airfoil_from_points: y at index {} must be a number", i).into()
            })? as f32;
            pts.push((x, y));
        }
        Ok(SectionHandle(Arc::new(Airfoil::new(pts, chord as f32))))
    });

    // Load a Selig-format .dat file (two whitespace-separated columns, optional header line).
    // `chord` is the physical chord length in model units.
    engine.register_fn("airfoil_from_dat",
        |path: &str, chord: f64| -> Result<SectionHandle, Box<rhai::EvalAltResult>> {
        let content = std::fs::read_to_string(path)
            .map_err(|e| -> Box<rhai::EvalAltResult> {
                format!("airfoil_from_dat: cannot read '{}': {}", path, e).into()
            })?;
        let mut pts: Vec<(f32, f32)> = Vec::new();
        for line in content.lines() {
            let line = line.trim();
            if line.is_empty() { continue; }
            let mut cols = line.split_whitespace();
            let x_str = match cols.next() { Some(s) => s, None => continue };
            let y_str = match cols.next() { Some(s) => s, None => continue };
            if let (Ok(x), Ok(y)) = (x_str.parse::<f32>(), y_str.parse::<f32>()) {
                pts.push((x, y));
            }
            // Lines that don't parse as floats (headers, comments) are silently skipped.
        }
        if pts.len() < 3 {
            return Err(format!(
                "airfoil_from_dat: '{}' contains fewer than 3 valid coordinate pairs", path
            ).into());
        }
        Ok(SectionHandle(Arc::new(Airfoil::new(pts, chord as f32))))
    });

    // wing_with_airfoil overload: accepts pre-built SectionHandles for root and tip.
    // The chord for each section is baked into the SectionHandle (e.g. via airfoil_from_dat).
    // Arity (6) differs from the NACA-string overload (7), so Rhai dispatches correctly.
    engine.register_fn("wing_with_airfoil",
        |root: SectionHandle, tip: SectionHandle,
         span: f64, sweep: f64, dihedral: f64, twist: f64| -> SdfHandle {
        let wing = wing_from_sections(
            root.0, tip.0,
            span as f32, sweep as f32, dihedral as f32, twist as f32,
        );
        SdfHandle(Arc::new(wing))
    });

    // --- Structural primitives ---

    // rib_at_station(wing, span_pos, thickness) → SdfHandle
    //
    // A rib at the given spanwise Y position, intersected with the wing volume.
    // `span_pos` is in the same coordinate units as the wing (absolute Y).
    // For a wing with half_span H, use span_pos = fraction * H.
    //
    // Example:
    //   let wing = wing_with_airfoil("2412", 12.0, 5.0, 40.0, 0.0, 0.0, 0.0);
    //   let root_rib = rib_at_station(wing, 0.0, 0.5);   // rib at root, 0.5 thick
    //   let mid_rib  = rib_at_station(wing, 8.0, 0.5);   // rib at y=8 (40% of half-span 20)
    engine.register_fn("rib_at_station", |wing: SdfHandle, span_pos: f64, thickness: f64| {
        let slab = rib_slab(span_pos as f32, thickness as f32);
        SdfHandle(Arc::new(Intersect::new(wing.0, slab)))
    });

    // spar(wing, chord_pos, radius) → SdfHandle
    //
    // A spanwise cylindrical spar running the full wing span, at chord position `chord_pos`.
    // `chord_pos` is the absolute X offset of the spar centreline (same units as the wing).
    // For a root chord C, the quarter-chord spar is at chord_pos = 0.25 * C.
    //
    // Example:
    //   let wing = wing_with_airfoil("2412", 12.0, 5.0, 40.0, 0.0, 0.0, 0.0);
    //   let front_spar = spar(wing, 3.0, 0.4);   // 25% chord, r=0.4
    //   let rear_spar  = spar(wing, 9.0, 0.3);   // 75% chord, r=0.3
    engine.register_fn("spar", |wing: SdfHandle, chord_pos: f64, radius: f64| {
        let cyl = spar_cylinder(chord_pos as f32, radius as f32);
        SdfHandle(Arc::new(Intersect::new(wing.0, cyl)))
    });

    register_drone_functions(engine);

    // Conformal lattice convenience wrappers
    engine.register_fn("wing_lattice", |wing: SdfHandle, cell_size: f64, thickness: f64| {
        use crate::sdf::aerospace::structural_drone::wing_lattice;
        SdfHandle(wing_lattice(wing.0, cell_size as f32, thickness as f32))
    });

    engine.register_fn("fuselage_lattice",
        |fuse: SdfHandle, cell_size: f64, thickness: f64| {
        use crate::sdf::aerospace::structural_drone::fuselage_lattice;
        SdfHandle(fuselage_lattice(fuse.0, cell_size as f32, thickness as f32))
    });

    engine.register_fn("fuselage_lattice_graded",
        |fuse: SdfHandle, inner_cell: f64, outer_cell: f64, thickness: f64| {
        use crate::sdf::aerospace::structural_drone::fuselage_lattice_graded;
        SdfHandle(fuselage_lattice_graded(
            fuse.0, inner_cell as f32, outer_cell as f32, thickness as f32,
        ))
    });
}

fn register_drone_functions(engine: &mut Engine) {
    // bulkhead_at_station(fuselage, position, thickness, num_holes, hole_radius_fraction)
    //
    // A structural ring at normalised axial position `position` ∈ [0, 1].
    // If num_holes > 0, lightening holes are drilled at 60 % of the local radius.
    engine.register_fn("bulkhead_at_station",
        |fuselage: SdfHandle, position: f64, thickness: f64,
         num_holes: i64, hole_radius_fraction: f64| {
        SdfHandle(bulkhead_at_station(
            fuselage.0,
            position as f32,
            thickness as f32,
            num_holes.max(0) as usize,
            hole_radius_fraction as f32,
        ))
    });

    // lightening_hole_pattern(body, count, radial_pos, hole_radius, axis)
    //
    // Drills `count` circular holes in a polar array at radius `radial_pos`.
    // axis: 0 = X, 1 = Y, 2 = Z
    engine.register_fn("lightening_hole_pattern",
        |body: SdfHandle, count: i64, radial_pos: f64, hole_radius: f64, axis: i64| {
        SdfHandle(lightening_hole_pattern(
            body.0,
            count.max(0) as usize,
            radial_pos as f32,
            hole_radius as f32,
            axis,
        ))
    });

    // rod_mount(bulkhead, angle_degrees, radial_fraction, rod_diameter, boss_diameter)
    //
    // Adds a cylindrical boss at the given polar position on a bulkhead face,
    // with a through-hole for a carbon rod.
    engine.register_fn("rod_mount",
        |bulkhead: SdfHandle, angle_degrees: f64, radial_fraction: f64,
         rod_diameter: f64, boss_diameter: f64| {
        SdfHandle(rod_mount(
            bulkhead.0,
            angle_degrees as f32,
            radial_fraction as f32,
            rod_diameter as f32,
            boss_diameter as f32,
        ))
    });

    // motor_arm(fuselage, angle_degrees, length, outer_diameter, inner_diameter)
    //
    // A hollow cylindrical boom arm extending radially from the fuselage at midspan (X = 0.5).
    // angle_degrees: 0° = +Y direction in the YZ plane.
    engine.register_fn("motor_arm",
        |fuselage: SdfHandle, angle_degrees: f64, length: f64,
         outer_diameter: f64, inner_diameter: f64| {
        SdfHandle(motor_arm(
            fuselage.0,
            angle_degrees as f32,
            length as f32,
            outer_diameter as f32,
            inner_diameter as f32,
        ))
    });

    // motor_mount(arm, motor_size, plate_thickness, bolt_pattern, bolt_diameter)
    //
    // A square mounting plate at the distal end of the arm (+Y tip at X = 0.5)
    // with 4 bolt holes in a polar array.
    engine.register_fn("motor_mount",
        |arm: SdfHandle, motor_size: f64, plate_thickness: f64,
         bolt_pattern: f64, bolt_diameter: f64| {
        SdfHandle(motor_mount(
            arm.0,
            motor_size as f32,
            plate_thickness as f32,
            bolt_pattern as f32,
            bolt_diameter as f32,
        ))
    });

    // generate_mounts(components, parent, wall_thickness, tab_width)
    //
    // Generates conforming mounting trays + attachment tabs for an array of placed
    // ComponentHandles. Returns a single SDF ready to be unioned into the parent.
    engine.register_fn("generate_mounts",
        |components: rhai::Array, parent: SdfHandle,
         wall_thickness: f64, tab_width: f64|
         -> Result<SdfHandle, Box<rhai::EvalAltResult>> {
        let mut pairs = Vec::new();
        for (i, item) in components.into_iter().enumerate() {
            let comp = item.try_cast::<ComponentHandle>()
                .ok_or_else(|| -> Box<rhai::EvalAltResult> {
                    format!("generate_mounts: item {} must be a ComponentHandle \
                             (use component() / component_named() + place())", i).into()
                })?;
            pairs.push((comp.geometry, comp.keepout));
        }
        Ok(SdfHandle(generate_mounts_sdf(
            pairs,
            parent.0,
            wall_thickness as f32,
            tab_width as f32,
        )))
    });

    // mount_with_bolts(component, parent, wall_thickness, tab_width, bolt_diameter, bolt_count)
    //
    // Wraps generate_mounts for a single component and additionally drills a polar array
    // of bolt holes through the attachment tabs along Z.
    engine.register_fn("mount_with_bolts",
        |comp: ComponentHandle, parent: SdfHandle,
         wall_thickness: f64, tab_width: f64,
         bolt_diameter: f64, bolt_count: i64| -> SdfHandle {
        use crate::sdf::primitives::Cylinder;
        use crate::sdf::patterns::PolarArray;
        use crate::sdf::transforms::Translate;
        use crate::sdf::booleans::Subtract;

        let mount_sdf = generate_mounts_sdf(
            vec![(comp.geometry, comp.keepout)],
            parent.0,
            wall_thickness as f32,
            tab_width as f32,
        );

        // Bolt holes along Z in a ring at radius tab_width/2 from origin.
        let bolt_r = bolt_diameter as f32 / 2.0;
        let bolt_hole = Arc::new(Cylinder::new(bolt_r, 10_000.0));
        let bolt_offset = Arc::new(Translate::new(
            bolt_hole,
            Vec3::new(tab_width as f32 * 0.5, 0.0, 0.0),
        ));
        let bolt_array = Arc::new(PolarArray::new(
            bolt_offset,
            bolt_count.max(0) as usize,
            Vec3::Z,
        ));
        SdfHandle(Arc::new(Subtract::new(mount_sdf, bolt_array)))
    });
}

/// Register the constraint-driven component API.
///
/// Design pattern (from scratchpad):
///   Every component has geometry_sdf + keepout_sdf (geometry offset by clearance).
///   Downstream geometry is built as boolean combinations of these SDFs.
///   Moving a component propagates automatically through the SDF DAG.
pub fn register_component_functions(engine: &mut Engine, collector: Arc<Mutex<Vec<MassPoint>>>) {
    // component(sdf, clearance_margin) — bundle geometry with its keepout zone
    engine.register_fn("component", |sdf: SdfHandle, margin: f64| {
        let keepout = Arc::new(Offset::new(Arc::clone(&sdf.0), margin as f32));
        ComponentHandle {
            geometry: sdf.0,
            keepout: keepout as Arc<dyn crate::sdf::Sdf>,
            mass_g: 0.0,
            name: String::new(),
        }
    });

    // component_mass(sdf, margin, mass_g) — same but with mass for CG
    engine.register_fn("component_mass", |sdf: SdfHandle, margin: f64, mass: f64| {
        let keepout = Arc::new(Offset::new(Arc::clone(&sdf.0), margin as f32));
        ComponentHandle {
            geometry: sdf.0,
            keepout: keepout as Arc<dyn crate::sdf::Sdf>,
            mass_g: mass as f32,
            name: String::new(),
        }
    });

    // component_named(name, sdf, margin, mass_g)
    engine.register_fn("component_named", |name: &str, sdf: SdfHandle, margin: f64, mass: f64| {
        let keepout = Arc::new(Offset::new(Arc::clone(&sdf.0), margin as f32));
        ComponentHandle {
            geometry: sdf.0,
            keepout: keepout as Arc<dyn crate::sdf::Sdf>,
            mass_g: mass as f32,
            name: name.to_string(),
        }
    });

    // place(comp, x, y, z) — translate both geometry and keepout together.
    // If the component carries mass, auto-registers a MassPoint at the placed position.
    {
        let col = Arc::clone(&collector);
        engine.register_fn("place", move |comp: ComponentHandle, x: f64, y: f64, z: f64| {
            let pos = Vec3::new(x as f32, y as f32, z as f32);
            if comp.mass_g > 0.0 {
                col.lock().unwrap().push(MassPoint {
                    name: comp.name.clone(),
                    mass_g: comp.mass_g,
                    position: pos,
                });
            }
            ComponentHandle {
                geometry: Arc::new(Translate::new(comp.geometry, pos)),
                keepout:  Arc::new(Translate::new(comp.keepout,  pos)),
                mass_g: comp.mass_g,
                name: comp.name,
            }
        });
    }

    // geometry(comp) — extract the actual part SDF
    engine.register_fn("geometry", |comp: ComponentHandle| {
        SdfHandle(comp.geometry)
    });

    // keepout(comp) — extract the clearance envelope SDF
    engine.register_fn("keepout", |comp: ComponentHandle| {
        SdfHandle(comp.keepout)
    });

    // mass_g(comp) — read mass value
    engine.register_fn("mass_g", |comp: ComponentHandle| {
        comp.mass_g as f64
    });
}

/// Register mass annotation functions and auto_fuselage.
/// Call this separately from register_sdf_functions, passing the shared collector.
pub fn register_mass_functions(engine: &mut Engine, collector: Arc<Mutex<Vec<MassPoint>>>) {
    // mass_at(mass_g, x, y, z) — declare a point mass at a position
    {
        let col = Arc::clone(&collector);
        engine.register_fn("mass_at", move |mass: f64, x: f64, y: f64, z: f64| {
            col.lock().unwrap().push(MassPoint {
                name: String::new(),
                mass_g: mass as f32,
                position: Vec3::new(x as f32, y as f32, z as f32),
            });
        });
    }

    // mass_named(name, mass_g, x, y, z) — declare a named point mass
    {
        let col = Arc::clone(&collector);
        engine.register_fn("mass_named", move |name: &str, mass: f64, x: f64, y: f64, z: f64| {
            col.lock().unwrap().push(MassPoint {
                name: name.to_string(),
                mass_g: mass as f32,
                position: Vec3::new(x as f32, y as f32, z as f32),
            });
        });
    }

    // auto_fuselage(internal_sdf, skin_thickness) — wrap internal geometry with an outer skin.
    // Equivalent to offset(internal_sdf, skin_thickness) but communicates design intent.
    engine.register_fn("auto_fuselage", |internal: SdfHandle, skin: f64| {
        SdfHandle(Arc::new(Offset::new(internal.0, skin as f32)))
    });
}

fn register_field_functions(engine: &mut Engine) {
    // Field primitives
    engine.register_fn("constant_field", |value: f64| {
        FieldHandle(Arc::new(ConstantField::new(value as f32)))
    });

    engine.register_fn("sdf_as_field", |sdf: SdfHandle| {
        FieldHandle(Arc::new(SdfField::new(sdf.0)))
    });

    engine.register_fn("position_x_field", || {
        FieldHandle(Arc::new(PositionXField))
    });

    engine.register_fn("position_y_field", || {
        FieldHandle(Arc::new(PositionYField))
    });

    engine.register_fn("position_z_field", || {
        FieldHandle(Arc::new(PositionZField))
    });

    // Field arithmetic
    engine.register_fn("add_fields", |a: FieldHandle, b: FieldHandle| {
        FieldHandle(Arc::new(FieldAdd::new(a.0, b.0)))
    });

    engine.register_fn("multiply_fields", |a: FieldHandle, b: FieldHandle| {
        FieldHandle(Arc::new(FieldMultiply::new(a.0, b.0)))
    });

    engine.register_fn("min_fields", |a: FieldHandle, b: FieldHandle| {
        FieldHandle(Arc::new(FieldMin::new(a.0, b.0)))
    });

    engine.register_fn("max_fields", |a: FieldHandle, b: FieldHandle| {
        FieldHandle(Arc::new(FieldMax::new(a.0, b.0)))
    });

    engine.register_fn("abs_field", |field: FieldHandle| {
        FieldHandle(Arc::new(FieldAbs::new(field.0)))
    });

    // Gradient fields
    engine.register_fn("gradient_field",
        |sx: f64, sy: f64, sz: f64, ex: f64, ey: f64, ez: f64,
         start_val: f64, end_val: f64| {
        FieldHandle(Arc::new(GradientField::new(
            Vec3::new(sx as f32, sy as f32, sz as f32),
            Vec3::new(ex as f32, ey as f32, ez as f32),
            start_val as f32,
            end_val as f32,
        )))
    });

    engine.register_fn("radial_field",
        |cx: f64, cy: f64, cz: f64, inner_r: f64, outer_r: f64,
         inner_val: f64, outer_val: f64| {
        FieldHandle(Arc::new(RadialField::new(
            Vec3::new(cx as f32, cy as f32, cz as f32),
            inner_r as f32,
            outer_r as f32,
            inner_val as f32,
            outer_val as f32,
        )))
    });

    engine.register_fn("axial_radial_field",
        |px: f64, py: f64, pz: f64, dx: f64, dy: f64, dz: f64,
         inner_r: f64, outer_r: f64, inner_val: f64, outer_val: f64| {
        FieldHandle(Arc::new(AxialRadialField::new(
            Vec3::new(px as f32, py as f32, pz as f32),
            Vec3::new(dx as f32, dy as f32, dz as f32),
            inner_r as f32,
            outer_r as f32,
            inner_val as f32,
            outer_val as f32,
        )))
    });

    // Field-driven SDF operations (return SdfHandle, not FieldHandle!)
    engine.register_fn("offset_by_field", |sdf: SdfHandle, field: FieldHandle| {
        SdfHandle(Arc::new(OffsetByField::new(sdf.0, field.0)))
    });

    engine.register_fn("shell_with_field", |sdf: SdfHandle, field: FieldHandle| {
        SdfHandle(Arc::new(ShellWithField::new(sdf.0, field.0)))
    });

    engine.register_fn("blend_by_field",
        |a: SdfHandle, b: SdfHandle, field: FieldHandle| {
        SdfHandle(Arc::new(BlendByField::new(a.0, b.0, field.0)))
    });

    // Lattice primitives
    engine.register_fn("gyroid", |cell_size: f64, thickness: f64| {
        SdfHandle(Arc::new(GyroidLattice::new(cell_size as f32, thickness as f32)))
    });

    engine.register_fn("cubic_lattice", |cell_size: f64, strut_radius: f64| {
        SdfHandle(Arc::new(CubicLattice::new(cell_size as f32, strut_radius as f32)))
    });

    engine.register_fn("diamond_lattice", |cell_size: f64, thickness: f64| {
        SdfHandle(Arc::new(DiamondLattice::new(cell_size as f32, thickness as f32)))
    });

    engine.register_fn("gyroid_with_field", |cell_size: f64, field: FieldHandle| {
        SdfHandle(Arc::new(GyroidWithField::new(cell_size as f32, field.0)))
    });
}

fn register_lattices(engine: &mut Engine) {
    // conformal_gyroid — uniform density
    engine.register_fn("conformal_gyroid",
        |parent: SdfHandle, cell_size: f64, thickness: f64| {
        SdfHandle(Arc::new(ConformalGyroid::new(parent.0, cell_size as f32, thickness as f32)))
    });

    // conformal_gyroid_field — spatially varying density
    engine.register_fn("conformal_gyroid_field",
        |parent: SdfHandle, cell_size: f64, thickness: f64, field: FieldHandle| {
        SdfHandle(Arc::new(ConformalGyroid::with_density_field(
            parent.0, cell_size as f32, thickness as f32, field.0,
        )))
    });

    // conformal_gyroid_region — lattice only inside the region mask
    engine.register_fn("conformal_gyroid_region",
        |parent: SdfHandle, cell_size: f64, thickness: f64, region: SdfHandle| {
        SdfHandle(Arc::new(ConformalGyroid::with_region_mask(
            parent.0, cell_size as f32, thickness as f32, region.0,
        )))
    });

    // conformal_diamond
    engine.register_fn("conformal_diamond",
        |parent: SdfHandle, cell_size: f64, thickness: f64| {
        SdfHandle(Arc::new(ConformalDiamond::new(parent.0, cell_size as f32, thickness as f32)))
    });

    // conformal_diamond_field
    engine.register_fn("conformal_diamond_field",
        |parent: SdfHandle, cell_size: f64, thickness: f64, field: FieldHandle| {
        SdfHandle(Arc::new(ConformalDiamond::with_density_field(
            parent.0, cell_size as f32, thickness as f32, field.0,
        )))
    });

    // conformal_schwarz_p
    engine.register_fn("conformal_schwarz_p",
        |parent: SdfHandle, cell_size: f64, thickness: f64| {
        SdfHandle(Arc::new(ConformalSchwarzP::new(parent.0, cell_size as f32, thickness as f32)))
    });
}

// ── Longitudinal spine functions ──────────────────────────────────────────────

/// Register `spline_fuselage(stations, length)` — like `fuselage()` but applies
/// longitudinal spine constraints when the section profiles have role-labelled points.
///
/// The function extracts Keel/Deck/Chine reference positions from each `SectionHandle`
/// (when the underlying section is a `SplineProfile`) and builds a
/// [`LoftedFuselage`] with the supplied [`LongitudinalSplines`].
pub fn register_spine_functions(
    engine: &mut Engine,
    splines: Arc<LongitudinalSplines>,
) {
    use crate::sdf::profiles::{SplineProfile, PointRole};

    engine.register_fn("spline_fuselage",
        move |stations: rhai::Array, length: f64| -> Result<SdfHandle, Box<rhai::EvalAltResult>> {
        if stations.len() < 2 {
            return Err("spline_fuselage requires at least 2 [position, section] pairs".into());
        }
        let len = length as f32;
        let mut tuples: Vec<(f32, Arc<dyn Section2D>, Option<f32>, Option<f32>, Option<f32>)> =
            Vec::with_capacity(stations.len());

        for (i, item) in stations.into_iter().enumerate() {
            let pair = item.try_cast::<rhai::Array>()
                .ok_or_else(|| -> Box<rhai::EvalAltResult> {
                    format!("spline_fuselage: item {} must be [position, section]", i).into()
                })?;
            if pair.len() < 2 {
                return Err(format!("spline_fuselage: item {} needs [position, section]", i).into());
            }
            let pos = pair[0].as_float().map_err(|_| -> Box<rhai::EvalAltResult> {
                format!("spline_fuselage: position in item {} must be a number", i).into()
            })? as f32;
            if !(0.0..=1.0).contains(&pos) {
                return Err(format!(
                    "spline_fuselage: position in item {} ({}) must be in [0, 1]", i, pos
                ).into());
            }
            let section = pair[1].clone().try_cast::<SectionHandle>()
                .ok_or_else(|| -> Box<rhai::EvalAltResult> {
                    format!("spline_fuselage: section in item {} must be a SectionHandle", i).into()
                })?;

            // Extract role positions if the section is a SplineProfile
            let (keel_z, deck_z, chine_y) = if let Some(sp) = section.0
                .as_any()
                .downcast_ref::<SplineProfile>()
            {
                use glam::Vec2;
                let keel  = sp.role_pos(PointRole::Keel).map(|v: Vec2| v.y);
                let deck  = sp.role_pos(PointRole::Deck).map(|v: Vec2| v.y);
                let chine = sp.role_pos(PointRole::Chine).map(|v: Vec2| v.x.abs());
                (keel, deck, chine)
            } else {
                (None, None, None)
            };

            tuples.push((pos, section.0, keel_z, deck_z, chine_y));
        }

        Ok(SdfHandle(Arc::new(LoftedFuselage::from_stations_with_splines(tuples, len, Arc::clone(&splines)))))
    });
}

// ── Profile (spline cross-section) functions ─────────────────────────────────

/// Register `spline(name)` → SdfHandle and `spline_section(name)` → SectionHandle.
///
/// Both functions look up `name` in `profiles`.  If the name is not found a
/// unit-circle default is returned so the script doesn't error.
pub fn register_profile_functions(
    engine: &mut Engine,
    profiles: Arc<RwLock<HashMap<String, SplineProfile>>>,
) {
    // spline(name) → SdfHandle
    // Returns the named profile as an infinite Y-axis extrusion (profile in XZ plane).
    {
        let p = Arc::clone(&profiles);
        engine.register_fn("spline", move |name: &str| -> SdfHandle {
            let guard = p.read().unwrap();
            let profile = guard.get(name)
                .cloned()
                .unwrap_or_else(|| SplineProfile::circle(8, 1.0));
            SdfHandle(Arc::new(profile))
        });
    }

    // spline_section(name) → SectionHandle
    // Returns the named profile as a Section2D for use in fuselage / wing lofting.
    {
        let p = Arc::clone(&profiles);
        engine.register_fn("spline_section", move |name: &str| -> SectionHandle {
            let guard = p.read().unwrap();
            let profile = guard.get(name)
                .cloned()
                .unwrap_or_else(|| SplineProfile::circle(8, 1.0));
            SectionHandle(Arc::new(profile))
        });
    }
}

// ── FEA boundary condition functions ─────────────────────────────────────────

/// Register FEA setup functions.  All calls append to `collector`.
/// If `stress_field` / `displacement_field` are `Some`, `stress_field()` and
/// `displacement_field()` are also available in the script.
pub fn register_fea_functions(
    engine:             &mut Engine,
    collector:          Arc<Mutex<crate::fea::FEASetup>>,
    stress_field:       Option<Arc<dyn crate::sdf::field::Field>>,
    displacement_field: Option<Arc<dyn crate::sdf::field::Field>>,
) {
    use crate::fea::{FEASetup, FEARegion, FEAAxisRegion, FEAForceRegion,
                     FEAPressureRegion, FEATorqueRegion, FEAMotorRegion};

    // fixed_support(region, name)
    {
        let c: Arc<Mutex<FEASetup>> = Arc::clone(&collector);
        engine.register_fn("fixed_support", move |region: SdfHandle, name: &str| {
            c.lock().unwrap().fixed_supports.push(FEARegion {
                name: name.to_string(),
                sdf:  region.0,
            });
        });
    }

    // fixed_axis(region, name, x, y, z)
    {
        let c: Arc<Mutex<FEASetup>> = Arc::clone(&collector);
        engine.register_fn("fixed_axis",
            move |region: SdfHandle, name: &str, x: bool, y: bool, z: bool| {
            c.lock().unwrap().fixed_axes.push(FEAAxisRegion {
                name: name.to_string(),
                sdf:  region.0,
                constrain_x: x,
                constrain_y: y,
                constrain_z: z,
            });
        });
    }

    // force_load(region, name, fx, fy, fz)
    {
        let c: Arc<Mutex<FEASetup>> = Arc::clone(&collector);
        engine.register_fn("force_load",
            move |region: SdfHandle, name: &str, fx: f64, fy: f64, fz: f64| {
            c.lock().unwrap().force_loads.push(FEAForceRegion {
                name:  name.to_string(),
                sdf:   region.0,
                force: Vec3::new(fx as f32, fy as f32, fz as f32),
            });
        });
    }

    // pressure_load(region, name, magnitude)
    {
        let c: Arc<Mutex<FEASetup>> = Arc::clone(&collector);
        engine.register_fn("pressure_load",
            move |region: SdfHandle, name: &str, magnitude: f64| {
            c.lock().unwrap().pressure_loads.push(FEAPressureRegion {
                name:      name.to_string(),
                sdf:       region.0,
                magnitude: magnitude as f32,
            });
        });
    }

    // gravity_load(gx, gy, gz)
    {
        let c: Arc<Mutex<FEASetup>> = Arc::clone(&collector);
        engine.register_fn("gravity_load", move |gx: f64, gy: f64, gz: f64| {
            c.lock().unwrap().gravity = Some(Vec3::new(gx as f32, gy as f32, gz as f32));
        });
    }

    // torque_load(region, name, ax, ay, az, magnitude)
    {
        let c: Arc<Mutex<FEASetup>> = Arc::clone(&collector);
        engine.register_fn("torque_load",
            move |region: SdfHandle, name: &str, ax: f64, ay: f64, az: f64, magnitude: f64| {
            c.lock().unwrap().torque_loads.push(FEATorqueRegion {
                name:      name.to_string(),
                sdf:       region.0,
                axis:      Vec3::new(ax as f32, ay as f32, az as f32).normalize_or_zero(),
                magnitude: magnitude as f32,
            });
        });
    }

    // motor_thrust(region, name, thrust_n, torque_nmm, dir_x, dir_y, dir_z)
    {
        let c: Arc<Mutex<FEASetup>> = Arc::clone(&collector);
        engine.register_fn("motor_thrust",
            move |region: SdfHandle, name: &str, thrust_n: f64, torque_nmm: f64,
                  dx: f64, dy: f64, dz: f64| {
            c.lock().unwrap().motor_thrusts.push(FEAMotorRegion {
                name:       name.to_string(),
                sdf:        region.0,
                thrust_n:   thrust_n   as f32,
                torque_nmm: torque_nmm as f32,
                direction:  Vec3::new(dx as f32, dy as f32, dz as f32).normalize_or_zero(),
            });
        });
    }

    // stress_field() / displacement_field() — available after a successful FEA run
    if let Some(sf) = stress_field {
        engine.register_fn("stress_field", move || FieldHandle(Arc::clone(&sf)));
    } else {
        engine.register_fn("stress_field", || -> FieldHandle {
            FieldHandle(Arc::new(crate::sdf::field::primitives::ConstantField::new(0.0)))
        });
    }

    if let Some(df) = displacement_field {
        engine.register_fn("displacement_field", move || FieldHandle(Arc::clone(&df)));
    } else {
        engine.register_fn("displacement_field", || -> FieldHandle {
            FieldHandle(Arc::new(crate::sdf::field::primitives::ConstantField::new(0.0)))
        });
    }
}
