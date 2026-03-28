// Rhai API registration

use std::sync::Arc;
use rhai::Engine;
use glam::{Vec3, Quat};
use crate::sdf::Sdf;
use crate::sdf::primitives::{Sphere, SdfBox, Cylinder, Torus, Cone, Plane};
use crate::sdf::booleans::{Union, Subtract, Intersect, SmoothUnion, SmoothSubtract, SmoothIntersect};
use crate::sdf::transforms::{Translate, Rotate, Scale, Offset, Shell, Twist, Bend};
use crate::sdf::patterns::{LinearArray, PolarArray, Mirror};
use crate::sdf::aerospace::{
    Airfoil, get_naca_airfoil, ExtrudedAirfoil, wing_with_airfoil, wing_from_sections,
    fuselage_parametric, fuselage_elliptical_parametric, nacelle_simple, is_valid_naca_4digit,
    CrossSection, LoftedFuselage, Section2D,
    rib_slab, spar_cylinder,
    bulkhead_at_station, lightening_hole_pattern,
    rod_mount, motor_arm, motor_mount, generate_mounts_sdf,
    keepout_intersects_plane, bulkhead_with_keepouts, cable_hole_at,
    bolt_circle, bolt_square, bolt_rect,
    countersink, counterbore, slot,
    chamfer_edge, thread_hole,
    fc_mount, motor_mount_pattern,
    VariableDuct, HollowVariableDuct, SplineTube, HollowSplineTube,
    ProfileDuct, HollowProfileDuct, FixedProfileDuct, HollowFixedProfileDuct,
    build_conformal_profile_duct_at_x, build_conformal_profile_inlet,
    build_dual_conformal_profile_duct_at_x, build_mirrored_dual_conformal_profile_duct_at_x,
    conformal_profile_section, conformal_profile_section_at_x,
    conformal_rounded_rect_section,
};
use crate::sdf::field::{
    primitives::{ConstantField, SdfField, PositionXField, PositionYField, PositionZField},
    arithmetic::{FieldAdd, FieldMultiply, FieldMin, FieldMax, FieldAbs},
    gradients::{GradientField, RadialField, AxialRadialField},
    operations::{OffsetByField, ShellWithField, BlendByField},
    lattice::{GyroidLattice, CubicLattice, DiamondLattice, GyroidWithField},
};
use crate::sdf::lattice::{ConformalGyroid, ConformalDiamond, ConformalSchwarzP};
use crate::sdf::print::{SplitPlane, AlignmentFeature, split_body, split_body_multi,
                        ToleranceSettings, ToleranceCompensated};
use crate::sdf::query::bounding_points;
use crate::sdf::print::fasteners::{get_spec, clearance_hole, countersink_hole, heat_set_boss,
                                   check_and_pad};
use crate::sdf::print::panels::{RetentionMechanism, panel_rect,
                                battery_hatch, fc_access_panel};
use crate::sdf::print::joints::JointDelta;
use std::sync::{Mutex, RwLock};
use std::collections::HashMap;
use std::fs;
use crate::sdf::profiles::SplineProfile;
use crate::sdf::spine::LongitudinalSplines;
use super::{SdfHandle, FieldHandle, MassPoint, ComponentHandle, SectionHandle, StationHandle,
            PathHandle, ProfileHandle, MaterialHandle, LayerHandle, LayupConfigHandle,
            HingeHandle, LinkageHandle, PointHandle, RefPointCollector, ReferencePoint, REF_COLORS,
            FlightConditionHandle};

pub fn register_sdf_functions(engine: &mut Engine) {
    // Register the SdfHandle and FieldHandle types
    engine.register_type::<SdfHandle>();
    engine.register_type::<FieldHandle>();
    engine.register_type::<ComponentHandle>();
    engine.register_type::<SectionHandle>();
    engine.register_type::<StationHandle>();
    engine.register_type::<PathHandle>();
    engine.register_type::<ProfileHandle>();
    engine.register_type::<MaterialHandle>();
    engine.register_type::<LayerHandle>();
    engine.register_type::<LayupConfigHandle>();
    engine.register_type::<HingeHandle>();
    engine.register_type::<LinkageHandle>();
    engine.register_type::<PointHandle>();
    register_point_functions(engine);

    // Register all SDF operations
    register_primitives(engine);
    register_booleans(engine);
    register_transforms(engine);
    register_patterns(engine);
    register_math_extras(engine);
    register_aerospace_functions(engine);
    register_mechanical_functions(engine);
    register_sweep_functions(engine);
    register_variable_duct_functions(engine);
    register_field_functions(engine);
    register_lattices(engine);
    register_composite_functions(engine);
    register_print_functions(engine);
    register_tolerance_functions(engine);
    register_fastener_functions(engine);
    register_panel_functions(engine);
    register_joint_functions(engine);
    register_layup_library_functions(engine);
    register_control_surface_functions(engine);
    register_granular_bracket_functions(engine);
    register_placement_functions(engine);
    register_instance_functions(engine);
    super::analysis_api::register_aero_functions(engine);
    super::analysis_api::register_analysis_functions(engine);
    super::analysis_api::register_propulsion_functions(engine);
    crate::scripting::legacy_api::register_legacy_compat_functions(engine);
}

#[allow(dead_code)]
fn analysis_bounds(sdf: &dyn Sdf) -> (Vec3, Vec3) {
    let bbox = bounding_points(sdf);
    let size = (bbox.max - bbox.min).abs();
    let pad = Vec3::new(
        (size.x * 0.1).max(5.0),
        (size.y * 0.1).max(5.0),
        (size.z * 0.1).max(5.0),
    );
    (bbox.min - pad, bbox.max + pad)
}

#[derive(Clone)]
struct FnSdf {
    func: Arc<dyn Fn(Vec3) -> f32 + Send + Sync>,
}

impl Sdf for FnSdf {
    fn distance(&self, point: Vec3) -> f32 {
        (self.func)(point)
    }
}

#[derive(Clone)]
enum StructuralRole {
    Sticky,
    Removable,
}

#[derive(Clone)]
struct BracketMountPoint {
    position: Vec3,
    normal: Vec3,
    tier: i64,
    base_radius: f32,
}

#[derive(Clone)]
struct BracketPart {
    id: String,
    sdf: Arc<dyn Sdf>,
    structural_role: StructuralRole,
    bbox_min: Option<Vec3>,
    bbox_max: Option<Vec3>,
}

#[derive(Clone)]
struct BracketPathResult {
    points: Vec<Vec3>,
    termination_reason: String,
    iterations: usize,
    host_part_id: Option<String>,
    keepout_part_id: Option<String>,
    min_keepout_distance: f32,
    final_host_distance: f32,
}

#[derive(Clone)]
struct BracketConfig {
    grad_eps: f32,
    safe_norm_eps: f32,
    step_size_mm: f32,
    min_step_mm: f32,
    max_step_mm: f32,
    rep_scale_max: f32,
    max_path_iters: usize,
    surface_tol_mm: f32,
    tier2_bridge_thresh_mm: f32,
    dilate_keepout_mm: f32,
    pocket_offset_mm: f32,
    tier1_radius_end_factor: f32,
    tier2_bridge_radius_factor: f32,
    host_blend_k: f32,
    support_density: u32,
    tray_clearance_mm: f32,
    tray_thickness_mm: f32,
}

impl Default for BracketConfig {
    fn default() -> Self {
        Self {
            grad_eps: 1e-3,
            safe_norm_eps: 1e-8,
            step_size_mm: 1.0,
            min_step_mm: 0.1,
            max_step_mm: 5.0,
            rep_scale_max: 10.0,
            max_path_iters: 200,
            surface_tol_mm: 0.05,
            tier2_bridge_thresh_mm: 15.0,
            dilate_keepout_mm: 1.5,
            pocket_offset_mm: 0.2,
            tier1_radius_end_factor: 2.5,
            tier2_bridge_radius_factor: 0.9,
            host_blend_k: 20.0,
            support_density: 3,
            tray_clearance_mm: 0.5,
            tray_thickness_mm: 1.0,
        }
    }
}

fn safe_normalize(v: Vec3, eps: f32) -> Vec3 {
    let len = v.length();
    if len < eps {
        Vec3::ZERO
    } else {
        v / len
    }
}

fn calc_gradient<F>(sdf_func: F, p: Vec3, epsilon: f32) -> Vec3
where
    F: Fn(Vec3) -> f32,
{
    let ex = Vec3::new(epsilon, 0.0, 0.0);
    let ey = Vec3::new(0.0, epsilon, 0.0);
    let ez = Vec3::new(0.0, 0.0, epsilon);

    let dx = (sdf_func(p + ex) - sdf_func(p - ex)) / (2.0 * epsilon);
    let dy = (sdf_func(p + ey) - sdf_func(p - ey)) / (2.0 * epsilon);
    let dz = (sdf_func(p + ez) - sdf_func(p - ez)) / (2.0 * epsilon);
    let g = Vec3::new(dx, dy, dz);
    if g.length() < 1e-6 {
        let dx = (sdf_func(p + ex) - sdf_func(p)) / epsilon;
        let dy = (sdf_func(p + ey) - sdf_func(p)) / epsilon;
        let dz = (sdf_func(p + ez) - sdf_func(p)) / epsilon;
        safe_normalize(Vec3::new(dx, dy, dz), 1e-8)
    } else {
        safe_normalize(g, 1e-8)
    }
}

fn sdf_tapered_capsule_distance(p: Vec3, a: Vec3, b: Vec3, ra: f32, rb: f32) -> f32 {
    let ab = b - a;
    let ab_len2 = ab.length_squared();
    if ab_len2 <= 1e-12 {
        return (p - a).length() - ra.max(rb);
    }
    let t = ((p - a).dot(ab) / ab_len2).clamp(0.0, 1.0);
    let p_on_segment = a + ab * t;
    let r = ra + t * (rb - ra);
    (p - p_on_segment).length() - r
}

fn smin_exp_pair(d1: f32, d2: f32, k: f32) -> f32 {
    if k <= 0.0 {
        return d1.min(d2);
    }
    let a = -k * d1;
    let b = -k * d2;
    let m = a.max(b);
    let s = (a - m).exp() + (b - m).exp();
    -((s + 1e-12).ln() + m) / k
}

fn aggregate_parts(parts: &[BracketPart], role: StructuralRole, p: Vec3) -> (f32, Option<String>) {
    let mut best = f32::INFINITY;
    let mut best_id = None;
    for part in parts {
        let matches = matches!(
            (&part.structural_role, &role),
            (StructuralRole::Sticky, StructuralRole::Sticky)
                | (StructuralRole::Removable, StructuralRole::Removable)
        );
        if !matches {
            continue;
        }
        if let (Some(bmin), Some(bmax)) = (part.bbox_min, part.bbox_max) {
            let q = p.clamp(bmin, bmax);
            let bbox_dist = p.distance(q);
            if bbox_dist > best {
                continue;
            }
        }
        let d = part.sdf.distance(p);
        if d < best {
            best = d;
            best_id = Some(part.id.clone());
        }
    }
    (best, best_id)
}

fn nearest_point_on_polyline(polyline: &[Vec3], query: Vec3) -> Option<Vec3> {
    if polyline.len() < 2 {
        return polyline.first().copied();
    }
    let mut best_p = None;
    let mut best_d2 = f32::INFINITY;
    for seg in polyline.windows(2) {
        let a = seg[0];
        let b = seg[1];
        let ab = b - a;
        let ab_len2 = ab.length_squared();
        if ab_len2 <= 1e-12 {
            let d2 = query.distance_squared(a);
            if d2 < best_d2 {
                best_d2 = d2;
                best_p = Some(a);
            }
            continue;
        }
        let t = ((query - a).dot(ab) / ab_len2).clamp(0.0, 1.0);
        let p = a + ab * t;
        let d2 = query.distance_squared(p);
        if d2 < best_d2 {
            best_d2 = d2;
            best_p = Some(p);
        }
    }
    best_p
}

fn bracket_part(id: impl Into<String>, sdf: Arc<dyn Sdf>, structural_role: StructuralRole) -> BracketPart {
    let bbox = crate::sdf::query::bounding_points(sdf.as_ref());
    BracketPart {
        id: id.into(),
        sdf,
        structural_role,
        bbox_min: Some(bbox.min),
        bbox_max: Some(bbox.max),
    }
}

fn apply_bracket_config_overrides(
    mut config: BracketConfig,
    map: &rhai::Map,
) -> BracketConfig {
    if let Some(v) = map.get("bracket_offset_mm").and_then(|v| v.clone().try_cast::<f64>()) {
        config.pocket_offset_mm = v.clamp(-1.0, 5.0) as f32;
    }
    if let Some(v) = map.get("support_density").and_then(|v| v.clone().try_cast::<i64>()) {
        config.support_density = v.clamp(1, 10) as u32;
    }
    if let Some(v) = map.get("tray_clearance_mm").and_then(|v| v.clone().try_cast::<f64>()) {
        config.tray_clearance_mm = v.clamp(0.0, 5.0) as f32;
    }
    if let Some(v) = map.get("tray_thickness_mm").and_then(|v| v.clone().try_cast::<f64>()) {
        config.tray_thickness_mm = v.clamp(0.5, 8.0) as f32;
    }
    let Some(cfg) = map.get("bracket_config").and_then(|v| v.clone().try_cast::<rhai::Map>()) else {
        return config;
    };
    if let Some(v) = cfg.get("grad_eps").and_then(|v| v.clone().try_cast::<f64>()) {
        config.grad_eps = v.max(1e-5) as f32;
    }
    if let Some(v) = cfg.get("step_size_mm").and_then(|v| v.clone().try_cast::<f64>()) {
        config.step_size_mm = v.max(0.05) as f32;
    }
    if let Some(v) = cfg.get("min_step_mm").and_then(|v| v.clone().try_cast::<f64>()) {
        config.min_step_mm = v.max(0.01) as f32;
    }
    if let Some(v) = cfg.get("max_step_mm").and_then(|v| v.clone().try_cast::<f64>()) {
        config.max_step_mm = v.max(config.min_step_mm as f64) as f32;
    }
    if let Some(v) = cfg.get("rep_scale_max").and_then(|v| v.clone().try_cast::<f64>()) {
        config.rep_scale_max = v.max(0.0) as f32;
    }
    if let Some(v) = cfg.get("max_path_iters").and_then(|v| v.clone().try_cast::<i64>()) {
        config.max_path_iters = v.max(10) as usize;
    }
    if let Some(v) = cfg.get("surface_tol_mm").and_then(|v| v.clone().try_cast::<f64>()) {
        config.surface_tol_mm = v.max(0.005) as f32;
    }
    if let Some(v) = cfg.get("tier2_bridge_thresh_mm").and_then(|v| v.clone().try_cast::<f64>()) {
        config.tier2_bridge_thresh_mm = v.max(0.0) as f32;
    }
    if let Some(v) = cfg.get("dilate_keepout_mm").and_then(|v| v.clone().try_cast::<f64>()) {
        config.dilate_keepout_mm = v.max(0.0) as f32;
    }
    if let Some(v) = cfg.get("pocket_offset_mm").and_then(|v| v.clone().try_cast::<f64>()) {
        config.pocket_offset_mm = v.clamp(-1.0, 5.0) as f32;
    }
    if let Some(v) = cfg.get("tier1_radius_end_factor").and_then(|v| v.clone().try_cast::<f64>()) {
        config.tier1_radius_end_factor = v.max(1.0) as f32;
    }
    if let Some(v) = cfg.get("tier2_bridge_radius_factor").and_then(|v| v.clone().try_cast::<f64>()) {
        config.tier2_bridge_radius_factor = v.max(0.1) as f32;
    }
    if let Some(v) = cfg.get("host_blend_k").and_then(|v| v.clone().try_cast::<f64>()) {
        config.host_blend_k = v.max(0.0) as f32;
    }
    if let Some(v) = cfg.get("support_density").and_then(|v| v.clone().try_cast::<i64>()) {
        config.support_density = v.clamp(1, 10) as u32;
    }
    if let Some(v) = cfg.get("tray_clearance_mm").and_then(|v| v.clone().try_cast::<f64>()) {
        config.tray_clearance_mm = v.clamp(0.0, 5.0) as f32;
    }
    if let Some(v) = cfg.get("tray_thickness_mm").and_then(|v| v.clone().try_cast::<f64>()) {
        config.tray_thickness_mm = v.clamp(0.5, 8.0) as f32;
    }
    config
}

fn effective_support_density(level: u32) -> f32 {
    let t = ((level as f32) - 1.0).clamp(0.0, 9.0) / 9.0;
    1.0 + t.powf(1.6) * 5.0
}

fn map_get_sdf(map: &rhai::Map, key: &str) -> Result<SdfHandle, Box<rhai::EvalAltResult>> {
    map.get(key)
        .and_then(|v| v.clone().try_cast::<SdfHandle>())
        .ok_or_else(|| format!("component map must contain SDF field '{}'", key).into())
}

fn map_get_optional_sdf(map: &rhai::Map, key: &str) -> Option<SdfHandle> {
    map.get(key).and_then(|v| v.clone().try_cast::<SdfHandle>())
}

fn place_component_map_impl(map: &rhai::Map, pos: Vec3) -> rhai::Map {
    let mut out = rhai::Map::new();
    for (key, value) in map.iter() {
        let placed = if let Some(sdf) = value.clone().try_cast::<SdfHandle>() {
            rhai::Dynamic::from(SdfHandle(Arc::new(Translate::new(sdf.0, pos))))
        } else if let Some(point) = value.clone().try_cast::<PointHandle>() {
            rhai::Dynamic::from(PointHandle(point.0 + pos))
        } else {
            value.clone()
        };
        out.insert(key.clone(), placed);
    }
    out.insert("position".into(), rhai::Dynamic::from(PointHandle(pos)));
    out
}

#[allow(dead_code)]
fn parse_axis_string(axis: &str) -> Result<Vec3, Box<rhai::EvalAltResult>> {
    match axis.to_ascii_lowercase().as_str() {
        "x" => Ok(Vec3::X),
        "y" => Ok(Vec3::Y),
        "z" => Ok(Vec3::Z),
        other => Err(format!("Invalid axis '{}': expected x, y, or z", other).into()),
    }
}

#[allow(dead_code)]
fn plane_from_axis(axis: &str, pos: f64) -> Result<SplitPlane, Box<rhai::EvalAltResult>> {
    match axis.to_ascii_lowercase().as_str() {
        "x" => Ok(SplitPlane::X(pos as f32)),
        "y" => Ok(SplitPlane::Y(pos as f32)),
        "z" => Ok(SplitPlane::Z(pos as f32)),
        other => Err(format!("Invalid axis '{}': expected x, y, or z", other).into()),
    }
}

#[allow(dead_code)]
fn tolerance_settings_from_map(settings: rhai::Map) -> ToleranceSettings {
    let mut out = ToleranceSettings::default();

    for (key, value) in settings {
        match key.as_str() {
            "external_offset_mm" => {
                if let Some(v) = value.clone().try_cast::<f64>() {
                    out.external_offset_mm = v as f32;
                }
            }
            "internal_offset_mm" => {
                if let Some(v) = value.clone().try_cast::<f64>() {
                    out.internal_offset_mm = v as f32;
                }
            }
            "min_hole_diameter_mm" => {
                if let Some(v) = value.clone().try_cast::<f64>() {
                    out.min_hole_diameter_mm = v as f32;
                }
            }
            "small_hole_bonus_mm" => {
                if let Some(v) = value.clone().try_cast::<f64>() {
                    out.small_hole_bonus_mm = v as f32;
                }
            }
            _ => {}
        }
    }

    out
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

    // Smooth Subtract ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â removes tool from base with a smooth chamfer/fillet
    // k controls blend radius (larger = softer transition)
    engine.register_fn("smooth_subtract", |base: SdfHandle, tool: SdfHandle, k: f64| {
        SdfHandle(Arc::new(SmoothSubtract::new(base.0, tool.0, k as f32)))
    });

    // Smooth Intersect ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â intersection with polynomial smooth maximum
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

    // Twist ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â rotate each cross-section by `rate` degrees per unit along axis (ax, ay, az).
    // Approximate SDF: Lipschitz-1 is not guaranteed under strong twist; safe for
    // raymarching and marching cubes at moderate deformation, not for precise offsets.
    engine.register_fn("twist", |body: SdfHandle, ax: f64, ay: f64, az: f64, rate: f64| {
        let axis = Vec3::new(ax as f32, ay as f32, az as f32);
        SdfHandle(Arc::new(Twist::new(body.0, axis, rate as f32)))
    });

    // Bend ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â curve the shape along axis (ax, ay, az) by `curvature` radians per unit length.
    // The bend plane is determined from the axis and world-Y (or world-Z near Y).
    // Approximate SDF: same caveat as twist.
    engine.register_fn("bend", |body: SdfHandle, ax: f64, ay: f64, az: f64, curvature: f64| {
        let axis = Vec3::new(ax as f32, ay as f32, az as f32);
        SdfHandle(Arc::new(Bend::new(body.0, axis, curvature as f32)))
    });
}

fn register_patterns(engine: &mut Engine) {
    // Linear array: N evenly-spaced copies offset by (dx, dy, dz) per step
    // e.g. linear_array(cylinder(2.0, 10.0), 5, 10.0, 0.0, 0.0) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ 5 cylinders in a row
    engine.register_fn("linear_array", |body: SdfHandle, count: i64, dx: f64, dy: f64, dz: f64| {
        let spacing = Vec3::new(dx as f32, dy as f32, dz as f32);
        SdfHandle(Arc::new(LinearArray::new(body.0, count as usize, spacing)))
    });

    // Polar array: N copies evenly rotated around Z axis
    // e.g. polar_array(cylinder(2.0, 10.0), 6) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ 6 cylinders in a circle
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

    // full_assembly(half) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â union of body + mirror_y(body) for a symmetric assembly
    engine.register_fn("full_assembly", |body: SdfHandle| {
        let mirrored = Arc::new(Mirror::new(Arc::clone(&body.0), Vec3::Y));
        SdfHandle(Arc::new(Union::new(body.0, mirrored)))
    });

    // mirror_wing(wing, dihedral_deg) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â mirrors wing about Y=0 and applies dihedral to both halves
    engine.register_fn("mirror_wing", |wing: SdfHandle, dihedral_deg: f64| {
        use crate::sdf::transforms::Rotate;
        let dihedral_rot = Quat::from_rotation_x(-(dihedral_deg as f32).to_radians());
        let mirrored_raw = Arc::new(Mirror::new(Arc::clone(&wing.0), Vec3::Y));
        let mirrored_tilted = Arc::new(Rotate::new(mirrored_raw, dihedral_rot));
        let original_rot = Quat::from_rotation_x((dihedral_deg as f32).to_radians());
        let original_tilted = Arc::new(Rotate::new(wing.0, original_rot));
        SdfHandle(Arc::new(Union::new(original_tilted, mirrored_tilted)))
    });

    // Assembly-context aliases for mirror_x/y/z
    engine.register_fn("mirror_assembly_y", |body: SdfHandle| {
        SdfHandle(Arc::new(Mirror::new(body.0, Vec3::Y)))
    });
    engine.register_fn("mirror_assembly_x", |body: SdfHandle| {
        SdfHandle(Arc::new(Mirror::new(body.0, Vec3::X)))
    });
    engine.register_fn("mirror_assembly_z", |body: SdfHandle| {
        SdfHandle(Arc::new(Mirror::new(body.0, Vec3::Z)))
    });
}

fn register_math_extras(engine: &mut Engine) {
    // Constants ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â expose PI, TAU, E as global variables in scripts
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

    engine.register_fn("fuselage_elliptical",
        |length: f64, width: f64, height: f64,
         nose_length: f64, tail_length: f64,
         nose_bluntness: f64, tail_bluntness: f64,
         smoothness: f64| {
        let fuse = fuselage_elliptical_parametric(
            length as f32,
            width as f32,
            height as f32,
            nose_length as f32,
            tail_length as f32,
            nose_bluntness as f32,
            tail_bluntness as f32,
            smoothness as f32,
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

    // fuselage(stations) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â build a lofted fuselage from [position, section] pairs.
    // fuselage(length_mm, stations) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â same, but sets physical length directly (preferred).
    // Positions are normalized [0, 1]. Stations are sorted automatically.
    //
    // Example:
    //   let f = fuselage(600.0, [
    //       [0.0,  circle_section(10.0)],
    //       [0.5,  circle_section(100.0)],
    //       [1.0,  circle_section(12.0)],
    //   ]);  // ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ 600 mm long fuselage
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

    // fuselage(length_mm, stations) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â same as fuselage(stations) but sets physical length directly.
    // Stations still use normalized [0, 1] positions; no separate scale() needed.
    engine.register_fn("fuselage",
        |length_mm: f64, stations: rhai::Array| -> Result<SdfHandle, Box<rhai::EvalAltResult>> {
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
                    format!("fuselage: section in item {} must be a SectionHandle", i).into()
                })?;
            pairs.push((pos, section.0));
        }
        Ok(SdfHandle(Arc::new(LoftedFuselage::from_stations(pairs, length_mm as f32))))
    });

    // --- Multi-station fuselage API (legacy ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â use fuselage() above instead) ---

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

    // Rectangular cross-section by width and height
    engine.register_fn("rect_section", |width: f64, height: f64| {
        SectionHandle(Arc::new(CrossSection::Rect {
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

    engine.register_fn("lofted_fuselage_smooth",
        |stations: rhai::Array, smoothness: f64| -> Result<SdfHandle, Box<rhai::EvalAltResult>> {
        if stations.len() < 2 {
            return Err("lofted_fuselage_smooth requires at least 2 stations".into());
        }
        let mut handles: Vec<StationHandle> = Vec::with_capacity(stations.len());
        for (i, item) in stations.into_iter().enumerate() {
            match item.try_cast::<StationHandle>() {
                Some(s) => handles.push(s),
                None => return Err(format!(
                    "lofted_fuselage_smooth: item at index {} is not a StationHandle (use fuselage_station())", i
                ).into()),
            }
        }
        for i in 1..handles.len() {
            if handles[i].position <= handles[i - 1].position {
                return Err(format!(
                    "lofted_fuselage_smooth: stations must be in strictly ascending order (index {} has position {} <= {})",
                    i, handles[i].position, handles[i - 1].position
                ).into());
            }
        }
        let length = handles.last().unwrap().position;
        if length <= 0.0 {
            return Err("lofted_fuselage_smooth: final station position must be > 0".into());
        }
        let pairs: Vec<(f32, Arc<dyn Section2D>)> = handles
            .into_iter()
            .map(|s| (s.position / length, s.section))
            .collect();
        Ok(SdfHandle(Arc::new(LoftedFuselage::from_stations_smoothed(
            pairs,
            length,
            smoothness as f32,
        ))))
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

    // rib_at_station(wing, span_pos, thickness) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ SdfHandle
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

    // spar(wing, chord_pos, radius) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ SdfHandle
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

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Wing geometry convenience queries ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬
    {
        use crate::sdf::query::bounding_points;

        // wing_span(wing) -> f64 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â Y extent of the wing bounding box
        engine.register_fn("wing_span", |wing: SdfHandle| -> f64 {
            let bi = bounding_points(&*wing.0);
            (bi.max.y - bi.min.y) as f64
        });

        // wing_area(wing) -> f64 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â planform area by trapezoidal integration
        engine.register_fn("wing_area", |wing: SdfHandle| -> f64 {
            let bi = bounding_points(&*wing.0);
            let y_min = bi.min.y;
            let y_max = bi.max.y;
            const N: usize = 50;
            let step = 0.5_f32;
            let chords: Vec<f32> = (0..=N).map(|i| {
                let y = y_min + (y_max - y_min) * i as f32 / N as f32;
                let mut x_max_l = bi.min.x;
                let mut x_min_l = bi.max.x;
                let mut found = false;
                let mut x = bi.min.x;
                while x <= bi.max.x {
                    if wing.0.distance(Vec3::new(x, y, 0.0)) < 0.0 {
                        x_max_l = x_max_l.max(x);
                        x_min_l = x_min_l.min(x);
                        found = true;
                    }
                    x += step;
                }
                if found { (x_max_l - x_min_l).max(0.0) } else { 0.0 }
            }).collect();
            let span = (y_max - y_min) as f64;
            let h = span / N as f64;
            (0..N).map(|i| (chords[i] + chords[i + 1]) as f64 * 0.5 * h).sum()
        });

        // wing_mac(wing) -> f64 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â mean aerodynamic chord
        engine.register_fn("wing_mac", |wing: SdfHandle| -> f64 {
            let bi = bounding_points(&*wing.0);
            let y_min = bi.min.y;
            let y_max = bi.max.y;
            const N: usize = 50;
            let step = 0.5_f32;
            let chords: Vec<f32> = (0..=N).map(|i| {
                let y = y_min + (y_max - y_min) * i as f32 / N as f32;
                let mut xmax = bi.min.x;
                let mut xmin = bi.max.x;
                let mut found = false;
                let mut x = bi.min.x;
                while x <= bi.max.x {
                    if wing.0.distance(Vec3::new(x, y, 0.0)) < 0.0 {
                        xmax = xmax.max(x);
                        xmin = xmin.min(x);
                        found = true;
                    }
                    x += step;
                }
                if found { (xmax - xmin).max(0.0) } else { 0.0 }
            }).collect();
            let span = (y_max - y_min) as f64;
            let h = span / N as f64;
            let numerator: f64 = (0..N).map(|i| {
                let c = (chords[i] + chords[i + 1]) as f64 * 0.5;
                c * c * h
            }).sum();
            let denominator: f64 = (0..N).map(|i| (chords[i] + chords[i + 1]) as f64 * 0.5 * h).sum();
            if denominator > 0.0 { numerator / denominator } else { 0.0 }
        });

        // wing_aspect_ratio(wing) -> f64
        engine.register_fn("wing_aspect_ratio", |wing: SdfHandle| -> f64 {
            let bi = bounding_points(&*wing.0);
            let span = (bi.max.y - bi.min.y) as f64;
            let y_min = bi.min.y;
            let y_max = bi.max.y;
            const N: usize = 30;
            let step = 1.0_f32;
            let chords: Vec<f32> = (0..=N).map(|i| {
                let y = y_min + (y_max - y_min) * i as f32 / N as f32;
                let mut xmax = bi.min.x;
                let mut xmin = bi.max.x;
                let mut found = false;
                let mut x = bi.min.x;
                while x <= bi.max.x {
                    if wing.0.distance(Vec3::new(x, y, 0.0)) < 0.0 {
                        xmax = xmax.max(x);
                        xmin = xmin.min(x);
                        found = true;
                    }
                    x += step;
                }
                if found { (xmax - xmin).max(0.0) } else { 0.0 }
            }).collect();
            let h = span / N as f64;
            let area: f64 = (0..N).map(|i| (chords[i] + chords[i + 1]) as f64 * 0.5 * h).sum();
            if area > 0.0 { span * span / area } else { 0.0 }
        });

        // wing_taper_ratio(wing) -> f64 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â tip_chord / root_chord
        engine.register_fn("wing_taper_ratio", |wing: SdfHandle| -> f64 {
            let bi = bounding_points(&*wing.0);
            let step = 0.5_f32;
            let chord_at = |y: f32| -> f32 {
                let mut xmax = bi.min.x;
                let mut xmin = bi.max.x;
                let mut found = false;
                let mut x = bi.min.x;
                while x <= bi.max.x {
                    if wing.0.distance(Vec3::new(x, y, 0.0)) < 0.0 {
                        xmax = xmax.max(x);
                        xmin = xmin.min(x);
                        found = true;
                    }
                    x += step;
                }
                if found { (xmax - xmin).max(0.0) } else { 0.0 }
            };
            let root_chord = chord_at(bi.min.y + (bi.max.y - bi.min.y) * 0.01);
            let tip_chord  = chord_at(bi.max.y - (bi.max.y - bi.min.y) * 0.01);
            if root_chord > 0.0 { tip_chord as f64 / root_chord as f64 } else { 1.0 }
        });

        // wing_mac_location(wing) -> PointHandle
        engine.register_fn("wing_mac_location", |wing: SdfHandle| -> PointHandle {
            let bi = bounding_points(&*wing.0);
            PointHandle(bi.center)
        });

        // wing_incidence(wing, fuselage) -> f64 degrees
        engine.register_fn("wing_incidence", |wing: SdfHandle, _fuselage: SdfHandle| -> f64 {
            use crate::sdf::query::furthest_point;
            let le = furthest_point(&*wing.0, -Vec3::X);
            let te = furthest_point(&*wing.0,  Vec3::X);
            let chord_vec = te - le;
            (chord_vec.z.atan2(chord_vec.x) as f64).to_degrees()
        });

        // set_wing_incidence(wing, fuselage, degrees) -> SdfHandle
        engine.register_fn("set_wing_incidence",
            |wing: SdfHandle, _fuselage: SdfHandle, degrees: f64| -> SdfHandle {
            use crate::sdf::query::furthest_point;
            use crate::sdf::transforms::Rotate;
            let le = furthest_point(&*wing.0, -Vec3::X);
            let current_inc = {
                let te = furthest_point(&*wing.0, Vec3::X);
                let chord_vec = te - le;
                (chord_vec.z.atan2(chord_vec.x)).to_degrees() as f64
            };
            let delta_rad = (degrees - current_inc) as f32 * std::f32::consts::PI / 180.0;
            let q = Quat::from_rotation_y(delta_rad);
            let to_origin = Arc::new(Translate::new(wing.0, -le));
            let rotated   = Arc::new(Rotate::new(to_origin, q));
            SdfHandle(Arc::new(Translate::new(rotated, le)))
        });
    }

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Deflect control surface ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

    // deflect(surface, angle_deg) -> SdfHandle
    // Rotates a control surface about an estimated hinge line for export/FEA purposes
    engine.register_fn("deflect", |surface: SdfHandle, angle_deg: f64| -> SdfHandle {
        use crate::sdf::query::bounding_points;
        use crate::sdf::transforms::Rotate;
        let bi     = bounding_points(&*surface.0);
        let hinge_x = bi.min.x;
        let pivot   = Vec3::new(
            hinge_x,
            (bi.min.y + bi.max.y) * 0.5,
            (bi.min.z + bi.max.z) * 0.5,
        );
        let q        = Quat::from_rotation_y(-(angle_deg as f32).to_radians());
        let to_pivot = Arc::new(Translate::new(surface.0, -pivot));
        let rotated  = Arc::new(Rotate::new(to_pivot, q));
        SdfHandle(Arc::new(Translate::new(rotated, pivot)))
    });

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Tail volume coefficients ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

    // tail_volume_coefficients(wing, h_tail, v_tail, fuselage) -> Map
    engine.register_fn("tail_volume_coefficients",
        |wing: SdfHandle, h_tail: SdfHandle, v_tail: SdfHandle, fuselage: SdfHandle| -> rhai::Map {
        use crate::sdf::aerospace::stability_geometry::compute_tail_volumes;
        let (coefs, rec) = compute_tail_volumes(&wing.0, &h_tail.0, &v_tail.0, &fuselage.0);
        let mut map = rhai::Map::new();
        map.insert("horizontal_vt".into(),         rhai::Dynamic::from(coefs.horizontal_vt as f64));
        map.insert("vertical_vt".into(),           rhai::Dynamic::from(coefs.vertical_vt as f64));
        map.insert("horizontal_tail_area".into(),  rhai::Dynamic::from(coefs.horizontal_tail_area as f64));
        map.insert("vertical_tail_area".into(),    rhai::Dynamic::from(coefs.vertical_tail_area as f64));
        map.insert("horizontal_moment_arm".into(), rhai::Dynamic::from(coefs.horizontal_moment_arm as f64));
        map.insert("vertical_moment_arm".into(),   rhai::Dynamic::from(coefs.vertical_moment_arm as f64));
        map.insert("wing_area".into(),             rhai::Dynamic::from(coefs.wing_area as f64));
        map.insert("wing_mac".into(),              rhai::Dynamic::from(coefs.wing_mac as f64));
        map.insert("wing_span".into(),             rhai::Dynamic::from(coefs.wing_span as f64));
        map.insert("horizontal_adequate".into(),   rhai::Dynamic::from(rec.horizontal_adequate));
        map.insert("vertical_adequate".into(),     rhai::Dynamic::from(rec.vertical_adequate));
        map.insert("horizontal_message".into(),    rhai::Dynamic::from(rec.horizontal_message));
        map.insert("vertical_message".into(),      rhai::Dynamic::from(rec.vertical_message));
        map
    });

    // size_horizontal_tail(wing, target_vht, moment_arm) -> f64 (required area in mmÃƒâ€šÃ‚Â²)
    engine.register_fn("size_horizontal_tail",
        |wing: SdfHandle, target_vht: f64, moment_arm: f64| -> f64 {
        use crate::sdf::aerospace::stability_geometry::{planform_area, mean_aero_chord};
        let s_w   = planform_area(&wing.0) as f64;
        let mac_w = mean_aero_chord(&wing.0) as f64;
        if moment_arm > 0.0 { target_vht * s_w * mac_w / moment_arm } else { 0.0 }
    });

    // size_vertical_tail(wing, target_vvt, moment_arm) -> f64
    engine.register_fn("size_vertical_tail",
        |wing: SdfHandle, target_vvt: f64, moment_arm: f64| -> f64 {
        use crate::sdf::aerospace::stability_geometry::planform_area;
        use crate::sdf::query::bounding_points;
        let bi  = bounding_points(&*wing.0);
        let b_w = (bi.max.y - bi.min.y) as f64;
        let s_w = planform_area(&wing.0) as f64;
        if moment_arm > 0.0 { target_vvt * s_w * b_w / moment_arm } else { 0.0 }
    });

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Phase 29: Haack / nose-cone primitives ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

    engine.register_fn("von_karman_nose", |length: f64, base_diam: f64| -> SdfHandle {
        use crate::sdf::aerospace::HaackNose;
        SdfHandle(Arc::new(HaackNose::new(length as f32, base_diam as f32 / 2.0, 0.0)))
    });

    engine.register_fn("lv_haack_nose", |length: f64, base_diam: f64| -> SdfHandle {
        use crate::sdf::aerospace::HaackNose;
        SdfHandle(Arc::new(HaackNose::new(length as f32, base_diam as f32 / 2.0, 1.0 / 3.0)))
    });

    engine.register_fn("haack_nose", |length: f64, base_diam: f64, c: f64| -> SdfHandle {
        use crate::sdf::aerospace::HaackNose;
        SdfHandle(Arc::new(HaackNose::new(length as f32, base_diam as f32 / 2.0, c as f32)))
    });

    engine.register_fn("tangent_ogive", |length: f64, base_diam: f64| -> SdfHandle {
        use crate::sdf::aerospace::TangentOgive;
        SdfHandle(Arc::new(TangentOgive::new(length as f32, base_diam as f32 / 2.0)))
    });

    engine.register_fn("ellipsoid_nose", |length: f64, base_diam: f64| -> SdfHandle {
        use crate::sdf::aerospace::EllipsoidNose;
        SdfHandle(Arc::new(EllipsoidNose::new(length as f32, base_diam as f32 / 2.0)))
    });

    engine.register_fn("haack_tail", |length: f64, base_diam: f64, tip_diam: f64, c: f64| -> SdfHandle {
        use crate::sdf::aerospace::HaackTail;
        SdfHandle(Arc::new(HaackTail::new(
            length as f32,
            base_diam as f32 / 2.0,
            tip_diam as f32 / 2.0,
            c as f32,
        )))
    });

    engine.register_fn("von_karman_tail", |length: f64, base_diam: f64, tip_diam: f64| -> SdfHandle {
        use crate::sdf::aerospace::HaackTail;
        SdfHandle(Arc::new(HaackTail::new(
            length as f32,
            base_diam as f32 / 2.0,
            tip_diam as f32 / 2.0,
            0.0,
        )))
    });

    // Convenience composites
    engine.register_fn("nose_fuselage_union", |nose: SdfHandle, fuse: SdfHandle| -> SdfHandle {
        use crate::sdf::query::bounding_points;
        let bbox = bounding_points(&*nose.0);
        let k = bbox.size.y * 0.08;
        SdfHandle(Arc::new(SmoothUnion::new(nose.0, fuse.0, k)))
    });

    engine.register_fn("tail_fuselage_union", |tail: SdfHandle, fuse: SdfHandle| -> SdfHandle {
        use crate::sdf::query::bounding_points;
        let bbox = bounding_points(&*tail.0);
        let k = bbox.size.y * 0.08;
        SdfHandle(Arc::new(SmoothUnion::new(tail.0, fuse.0, k)))
    });

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Phase 29: NACA inlet ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

    engine.register_fn("naca_inlet",
        |width: f64, length: f64, depth: f64,
         px: f64, py: f64, pz: f64,
         nx: f64, ny: f64, nz: f64,
         fdx: f64, fdy: f64, fdz: f64| -> SdfHandle {
        use crate::sdf::aerospace::NacaInlet;
        let normal = Vec3::new(nx as f32, ny as f32, nz as f32).normalize();
        let flow = Vec3::new(fdx as f32, fdy as f32, fdz as f32).normalize();
        SdfHandle(Arc::new(NacaInlet {
            width: width as f32,
            length: length as f32,
            depth: depth as f32,
            ramp_angle_deg: 7.0,
            position: Vec3::new(px as f32, py as f32, pz as f32),
            normal,
            flow_direction: flow,
        }))
    });

    engine.register_fn("naca_inlet_surface",
        |fuse: SdfHandle, width: f64, length: f64, depth: f64,
         axial_pos: f64, circ_angle_deg: f64| -> SdfHandle {
        use crate::sdf::query::bounding_points;
        use crate::sdf::aerospace::NacaInlet;
        let bbox = bounding_points(&*fuse.0);
        let x = bbox.min.x + axial_pos as f32 * bbox.size.x;
        let angle_rad = circ_angle_deg as f32 * std::f32::consts::PI / 180.0;
        let r = bbox.size.y.max(bbox.size.z) / 2.0;
        let pos = Vec3::new(x, r * angle_rad.sin(), r * (-angle_rad.cos()));
        let eps = 0.5_f32;
        let gnx = (fuse.0.distance(pos + Vec3::X * eps) - fuse.0.distance(pos - Vec3::X * eps)) / (2.0 * eps);
        let gny = (fuse.0.distance(pos + Vec3::Y * eps) - fuse.0.distance(pos - Vec3::Y * eps)) / (2.0 * eps);
        let gnz = (fuse.0.distance(pos + Vec3::Z * eps) - fuse.0.distance(pos - Vec3::Z * eps)) / (2.0 * eps);
        let normal = Vec3::new(gnx, gny, gnz).normalize();
        let flow_dir = -Vec3::X;
        SdfHandle(Arc::new(NacaInlet {
            width: width as f32,
            length: length as f32,
            depth: depth as f32,
            ramp_angle_deg: 7.0,
            position: pos,
            normal,
            flow_direction: flow_dir,
        }))
    });

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Phase 29: EDF inlet lips ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

    engine.register_fn("circular_inlet",
        |diam: f64, lip_r: f64, px: f64, py: f64, pz: f64,
         dx: f64, dy: f64, dz: f64| -> SdfHandle {
        use crate::sdf::aerospace::{InletLip, InletShape};
        let direction = Vec3::new(dx as f32, dy as f32, dz as f32).normalize();
        SdfHandle(Arc::new(InletLip {
            shape: InletShape::Circular { diameter: diam as f32 },
            lip_radius: lip_r as f32,
            position: Vec3::new(px as f32, py as f32, pz as f32),
            direction,
            highlight_to_throat: diam as f32 * 0.5,
            throat_area_fraction: 0.92,
        }))
    });

    engine.register_fn("elliptical_inlet",
        |w: f64, h: f64, lip_r: f64, px: f64, py: f64, pz: f64,
         dx: f64, dy: f64, dz: f64| -> SdfHandle {
        use crate::sdf::aerospace::{InletLip, InletShape};
        let direction = Vec3::new(dx as f32, dy as f32, dz as f32).normalize();
        SdfHandle(Arc::new(InletLip {
            shape: InletShape::Elliptical { width: w as f32, height: h as f32 },
            lip_radius: lip_r as f32,
            position: Vec3::new(px as f32, py as f32, pz as f32),
            direction,
            highlight_to_throat: h as f32,
            throat_area_fraction: 0.92,
        }))
    });

    engine.register_fn("dshaped_inlet",
        |w: f64, h: f64, ff: f64, lip_r: f64,
         px: f64, py: f64, pz: f64,
         dx: f64, dy: f64, dz: f64| -> SdfHandle {
        use crate::sdf::aerospace::{InletLip, InletShape};
        let direction = Vec3::new(dx as f32, dy as f32, dz as f32).normalize();
        SdfHandle(Arc::new(InletLip {
            shape: InletShape::DShaped { width: w as f32, height: h as f32, flat_fraction: ff as f32 },
            lip_radius: lip_r as f32,
            position: Vec3::new(px as f32, py as f32, pz as f32),
            direction,
            highlight_to_throat: h as f32,
            throat_area_fraction: 0.92,
        }))
    });

    engine.register_fn("chin_inlet",
        |_fuse: SdfHandle, axial: f64, shape_str: &str, w: f64, h: f64, lip_r: f64| -> SdfHandle {
        use crate::sdf::query::bounding_points;
        use crate::sdf::aerospace::{InletLip, InletShape};
        let bbox = bounding_points(&*_fuse.0);
        let x = bbox.min.x + axial as f32 * bbox.size.x;
        let r = bbox.size.y.max(bbox.size.z) / 2.0;
        // chin = bottom (circumferential angle 180 deg = -Z)
        let pos = Vec3::new(x, 0.0, -r);
        let direction = Vec3::new(0.0, 0.0, 1.0); // pointing into aircraft from below
        let shape = match shape_str {
            "elliptical" => InletShape::Elliptical { width: w as f32, height: h as f32 },
            "dshaped" => InletShape::DShaped { width: w as f32, height: h as f32, flat_fraction: 0.3 },
            _ => InletShape::Circular { diameter: w as f32 },
        };
        SdfHandle(Arc::new(InletLip {
            shape,
            lip_radius: lip_r as f32,
            position: pos,
            direction,
            highlight_to_throat: h as f32,
            throat_area_fraction: 0.92,
        }))
    });

    engine.register_fn("side_inlet",
        |_fuse: SdfHandle, axial: f64, side: &str, shape_str: &str, w: f64, h: f64, lip_r: f64| -> SdfHandle {
        use crate::sdf::query::bounding_points;
        use crate::sdf::aerospace::{InletLip, InletShape};
        let bbox = bounding_points(&*_fuse.0);
        let x = bbox.min.x + axial as f32 * bbox.size.x;
        let r = bbox.size.y.max(bbox.size.z) / 2.0;
        let angle_deg: f32 = match side { "left" => 90.0, "right" => -90.0, _ => 90.0 };
        let angle_rad = angle_deg * std::f32::consts::PI / 180.0;
        let pos = Vec3::new(x, r * angle_rad.sin(), r * (-angle_rad.cos()));
        let shape = match shape_str {
            "elliptical" => InletShape::Elliptical { width: w as f32, height: h as f32 },
            "dshaped" => InletShape::DShaped { width: w as f32, height: h as f32, flat_fraction: 0.3 },
            _ => InletShape::Circular { diameter: w as f32 },
        };
        let direction = Vec3::new(0.0, -angle_rad.sin(), angle_rad.cos()).normalize();
        SdfHandle(Arc::new(InletLip {
            shape,
            lip_radius: lip_r as f32,
            position: pos,
            direction,
            highlight_to_throat: h as f32,
            throat_area_fraction: 0.92,
        }))
    });

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Phase 29: EDF duct ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

    engine.register_fn("edf_duct",
        |inlet: SdfHandle, fan_diam: f64,
         fpx: f64, fpy: f64, fpz: f64,
         epx: f64, epy: f64, epz: f64,
         edx: f64, edy: f64, edz: f64| -> SdfHandle {
        use crate::sdf::query::bounding_points;
        use crate::sdf::aerospace::EdfDuct;
        let bbox = bounding_points(&*inlet.0);
        let inlet_r = bbox.size.y.max(bbox.size.z) / 2.0;
        let fan_r = fan_diam as f32 / 2.0;
        let exhaust_r = fan_r * (0.90_f32).sqrt();
        SdfHandle(Arc::new(EdfDuct {
            inlet_position: bbox.center,
            inlet_radius: inlet_r,
            fan_position: Vec3::new(fpx as f32, fpy as f32, fpz as f32),
            fan_radius: fan_r,
            exhaust_position: Vec3::new(epx as f32, epy as f32, epz as f32),
            exhaust_radius: exhaust_r,
            exhaust_direction: Vec3::new(edx as f32, edy as f32, edz as f32).normalize(),
            control_points: vec![],
        }))
    });

    engine.register_fn("edf_duct_s",
        |inlet: SdfHandle, fan_diam: f64,
         fan_pos: PointHandle, exhaust_pos: PointHandle, exhaust_dir: PointHandle,
         ctrl_pts: rhai::Array| -> SdfHandle {
        use crate::sdf::query::bounding_points;
        use crate::sdf::aerospace::EdfDuct;
        let control_points: Vec<Vec3> = ctrl_pts.iter().filter_map(|pt| {
            let arr = pt.clone().into_array().ok()?;
            if arr.len() >= 3 {
                Some(Vec3::new(
                    arr[0].as_float().ok()? as f32,
                    arr[1].as_float().ok()? as f32,
                    arr[2].as_float().ok()? as f32,
                ))
            } else { None }
        }).collect();
        let bbox = bounding_points(&*inlet.0);
        let inlet_r = bbox.size.y.max(bbox.size.z) / 2.0;
        let fan_r = fan_diam as f32 / 2.0;
        SdfHandle(Arc::new(EdfDuct {
            inlet_position: bbox.center,
            inlet_radius: inlet_r,
            fan_position: fan_pos.0,
            fan_radius: fan_r,
            exhaust_position: exhaust_pos.0,
            exhaust_radius: fan_r * 0.949,
            exhaust_direction: exhaust_dir.0.normalize(),
            control_points,
        }))
    });

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Phase 29: Exhaust nozzle ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

    engine.register_fn("edf_exhaust",
        |_fuse: SdfHandle, pos: PointHandle, dir: PointHandle, diam: f64, lip_r: f64| -> SdfHandle {
        use crate::sdf::aerospace::{InletLip, InletShape};
        SdfHandle(Arc::new(InletLip {
            shape: InletShape::Circular { diameter: diam as f32 },
            lip_radius: lip_r as f32,
            position: pos.0,
            direction: dir.0.normalize(),
            highlight_to_throat: diam as f32 * 0.3,
            throat_area_fraction: 1.0,
        }))
    });

    engine.register_fn("convergent_nozzle",
        |fan_exit_diam: f64, exhaust_diam: f64, length: f64,
         pos: PointHandle, dir: PointHandle| -> SdfHandle {
        use crate::sdf::aerospace::EdfDuct;
        let d = dir.0.normalize();
        SdfHandle(Arc::new(EdfDuct {
            inlet_position: pos.0,
            inlet_radius: fan_exit_diam as f32 / 2.0,
            fan_position: pos.0 + d * length as f32 / 2.0,
            fan_radius: (fan_exit_diam as f32 + exhaust_diam as f32) / 4.0,
            exhaust_position: pos.0 + d * length as f32,
            exhaust_radius: exhaust_diam as f32 / 2.0,
            exhaust_direction: d,
            control_points: vec![],
        }))
    });

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Phase 29: Inlet performance analysis ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

    engine.register_fn("inlet_performance",
        |_fuse: SdfHandle, fc: FlightConditionHandle, fan_diam: f64, _fan_pos: PointHandle| -> rhai::Map {
        use crate::aero::inlet_analysis::compute_inlet_performance;
        let result = compute_inlet_performance(
            fan_diam as f32 * 0.6,   // estimated inlet throat radius
            fan_diam as f32 / 2.0,   // fan face radius
            fan_diam as f32 * 3.0,   // estimated duct length
            15.0,                     // default bend angle
            &fc.0,
        );
        let mut map = rhai::Map::new();
        map.insert("pressure_recovery".into(), rhai::Dynamic::from(result.pressure_recovery as f64));
        map.insert("distortion_dc60".into(), rhai::Dynamic::from(result.distortion_dc60 as f64));
        map.insert("area_ratio".into(), rhai::Dynamic::from(result.area_ratio as f64));
        map.insert("ld_ratio".into(), rhai::Dynamic::from(result.diffuser_length_to_diameter as f64));
        map.insert("estimated_drag_n".into(), rhai::Dynamic::from(result.estimated_inlet_drag_n as f64));
        map
    });

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Phase 29: Surface-conforming access panels ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

    engine.register_fn("surface_panel",
        |parent: SdfHandle, width: f64, height: f64, pos: PointHandle| -> rhai::Array {
        use crate::sdf::booleans::Subtract as SdfSubtract;
        use crate::sdf::transforms::{Rotate as SdfRotate, Translate as SdfTranslate};
        use crate::sdf::primitives::SdfBox as SdfBoxPrim;
        let p = pos.0;
        let eps = 0.5_f32;
        let gnx = (parent.0.distance(p + Vec3::X * eps) - parent.0.distance(p - Vec3::X * eps)) / (2.0 * eps);
        let gny = (parent.0.distance(p + Vec3::Y * eps) - parent.0.distance(p - Vec3::Y * eps)) / (2.0 * eps);
        let gnz = (parent.0.distance(p + Vec3::Z * eps) - parent.0.distance(p - Vec3::Z * eps)) / (2.0 * eps);
        let normal = Vec3::new(gnx, gny, gnz).normalize();

        // Build local frame
        let world_up = if normal.dot(Vec3::Z).abs() < 0.9 { Vec3::Z } else { Vec3::Y };
        let tangent = normal.cross(world_up).normalize();
        let bitangent = normal.cross(tangent).normalize();

        // Build rotation from local to world: cols = tangent, bitangent, normal
        let rot_mat = glam::Mat3::from_cols(tangent, bitangent, normal);
        let quat = Quat::from_mat3(&rot_mat);

        let panel_box: Arc<dyn crate::sdf::Sdf> = Arc::new(SdfBoxPrim::new(
            Vec3::new(height as f32 / 2.0, width as f32 / 2.0, 3.0 / 2.0)
        ));
        let rotated: Arc<dyn crate::sdf::Sdf> = Arc::new(SdfRotate::new(panel_box, quat));
        let positioned: Arc<dyn crate::sdf::Sdf> = Arc::new(SdfTranslate::new(rotated, p));

        let modified_parent = SdfHandle(Arc::new(SdfSubtract::new(parent.0, positioned.clone())));
        let panel_piece = SdfHandle(positioned);
        vec![rhai::Dynamic::from(modified_parent), rhai::Dynamic::from(panel_piece)]
    });

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Simple inlet wrappers ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

    // naca_flush_inlet(width_mm, length_mm, depth_mm, fuse) -> SdfHandle
    // Returns the inlet pocket positioned on the top surface (+Z) of the fuselage.
    // Subtract the result from the fuselage to cut the inlet.
    engine.register_fn("naca_flush_inlet",
        |width: f64, length: f64, depth: f64, fuse: SdfHandle| -> SdfHandle {
        use crate::sdf::aerospace::NacaInlet;
        use crate::sdf::query::bounding_points;
        let bbox  = bounding_points(&*fuse.0);
        // Place at the top-centre of the fuselage, one-third along the body
        let x_pos = bbox.min.x + bbox.size.x * 0.33;
        let z_top = bbox.max.z;
        let pos   = Vec3::new(x_pos, 0.0, z_top);
        let normal = Vec3::Z;
        let flow   = -Vec3::X;
        SdfHandle(Arc::new(NacaInlet {
            width:         width  as f32,
            length:        length as f32,
            depth:         depth  as f32,
            ramp_angle_deg: 7.0,
            position:      pos,
            normal,
            flow_direction: flow,
        }))
    });

    // buried_inlet(throat_radius_mm, duct_length_mm, surface_offset_mm) -> SdfHandle
    // Returns the buried inlet scoop + duct at the origin, opening at +Z.
    engine.register_fn("buried_inlet",
        |throat_r: f64, duct_length: f64, surface_offset: f64| -> SdfHandle {
        use crate::sdf::aerospace::BuriedInlet;
        SdfHandle(Arc::new(BuriedInlet::new(
            throat_r as f32,
            duct_length as f32,
            surface_offset as f32,
        )))
    });

    // s_duct(inlet_r_mm, exit_r_mm, length_mm, offset_y_mm) -> SdfHandle
    // Returns an S-shaped duct starting at the origin, flowing in +X.
    engine.register_fn("s_duct",
        |inlet_r: f64, exit_r: f64, length: f64, offset_y: f64| -> SdfHandle {
        use crate::sdf::aerospace::SDuct;
        SdfHandle(Arc::new(SDuct::new(
            inlet_r as f32,
            exit_r  as f32,
            length  as f32,
            offset_y as f32,
            0.0,
        )))
    });

    engine.register_fn("surface_panel_at_station",
        |fuse: SdfHandle, axial: f64, circ_deg: f64, w: f64, h: f64| -> rhai::Array {
        use crate::sdf::query::bounding_points;
        use crate::sdf::booleans::Subtract as SdfSubtract;
        use crate::sdf::transforms::{Rotate as SdfRotate, Translate as SdfTranslate};
        use crate::sdf::primitives::SdfBox as SdfBoxPrim;
        let bbox = bounding_points(&*fuse.0);
        let x = bbox.min.x + axial as f32 * bbox.size.x;
        let angle = circ_deg as f32 * std::f32::consts::PI / 180.0;
        let r = bbox.size.y.max(bbox.size.z) / 2.0;
        let p = Vec3::new(x, r * angle.sin(), r * (-angle.cos()));

        let eps = 0.5_f32;
        let gnx = (fuse.0.distance(p + Vec3::X * eps) - fuse.0.distance(p - Vec3::X * eps)) / (2.0 * eps);
        let gny = (fuse.0.distance(p + Vec3::Y * eps) - fuse.0.distance(p - Vec3::Y * eps)) / (2.0 * eps);
        let gnz = (fuse.0.distance(p + Vec3::Z * eps) - fuse.0.distance(p - Vec3::Z * eps)) / (2.0 * eps);
        let normal = Vec3::new(gnx, gny, gnz).normalize();

        let world_up = if normal.dot(Vec3::Z).abs() < 0.9 { Vec3::Z } else { Vec3::Y };
        let tangent = normal.cross(world_up).normalize();
        let bitangent = normal.cross(tangent).normalize();
        let rot_mat = glam::Mat3::from_cols(tangent, bitangent, normal);
        let quat = Quat::from_mat3(&rot_mat);

        let panel_box: Arc<dyn crate::sdf::Sdf> = Arc::new(SdfBoxPrim::new(
            Vec3::new(h as f32 / 2.0, w as f32 / 2.0, 3.0 / 2.0)
        ));
        let rotated: Arc<dyn crate::sdf::Sdf> = Arc::new(SdfRotate::new(panel_box, quat));
        let positioned: Arc<dyn crate::sdf::Sdf> = Arc::new(SdfTranslate::new(rotated, p));

        let modified_fuse = SdfHandle(Arc::new(SdfSubtract::new(fuse.0, positioned.clone())));
        let panel_piece = SdfHandle(positioned);
        vec![rhai::Dynamic::from(modified_fuse), rhai::Dynamic::from(panel_piece)]
    });
}

fn register_drone_functions(engine: &mut Engine) {
    // bulkhead_at_station(fuselage, position, thickness, num_holes, hole_radius_fraction)
    //
    // A structural ring at normalised axial position `position` ÃƒÂ¢Ã‹â€ Ã‹â€  [0, 1].
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
    // angle_degrees: 0Ãƒâ€šÃ‚Â° = +Y direction in the YZ plane.
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
pub fn register_component_functions(
    engine: &mut Engine,
    collector: Arc<Mutex<Vec<MassPoint>>>,
    comp_collector: Arc<Mutex<Vec<ComponentHandle>>>,
) {
    fn open_top_tray(length: f32, width: f32, height: f32, wall: f32) -> Arc<dyn crate::sdf::Sdf> {
        let outer = Arc::new(SdfBox::new(Vec3::new(length * 0.5, width * 0.5, height * 0.5)));
        let inner = Arc::new(SdfBox::new(Vec3::new(
            (length - wall * 2.0).max(0.5) * 0.5,
            (width - wall * 2.0).max(0.5) * 0.5,
            (height - wall).max(0.5) * 0.5,
        )));
        let inner = Arc::new(Translate::new(inner, Vec3::new(0.0, 0.0, wall * 0.5)));
        Arc::new(Subtract::new(outer, inner))
    }

    // component(sdf, clearance_margin) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â bundle geometry with its keepout zone
    engine.register_fn("component", |sdf: SdfHandle, margin: f64| {
        let keepout = Arc::new(Offset::new(Arc::clone(&sdf.0), margin as f32));
        ComponentHandle {
            geometry: sdf.0,
            keepout: keepout as Arc<dyn crate::sdf::Sdf>,
            mass_g: 0.0,
            name: String::new(),
        }
    });

    // component_mass(sdf, margin, mass_g) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â same but with mass for CG
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

    // place(comp, x, y, z) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â translate both geometry and keepout together.
    // If the component carries mass, auto-registers a MassPoint at the placed position.
    // Also registers the placed component into the comp_collector for bulkhead_auto.
    {
        let col = Arc::clone(&collector);
        let cc  = Arc::clone(&comp_collector);
        engine.register_fn("place", move |comp: ComponentHandle, x: f64, y: f64, z: f64| {
            let pos = Vec3::new(x as f32, y as f32, z as f32);
            if comp.mass_g > 0.0 {
                col.lock().unwrap().push(MassPoint {
                    name: comp.name.clone(),
                    mass_g: comp.mass_g,
                    position: pos,
                });
            }
            let placed = ComponentHandle {
                geometry: Arc::new(Translate::new(comp.geometry, pos)),
                keepout:  Arc::new(Translate::new(comp.keepout,  pos)),
                mass_g: comp.mass_g,
                name: comp.name,
            };
            cc.lock().unwrap().push(placed.clone());
            placed
        });
    }

    // geometry(comp) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â extract the actual part SDF
    engine.register_fn("place_component",
        |comp_map: rhai::Map, x: f64, y: f64, z: f64| -> rhai::Map {
            place_component_map_impl(&comp_map, Vec3::new(x as f32, y as f32, z as f32))
        }
    );

    engine.register_fn("geometry", |comp: ComponentHandle| {
        SdfHandle(comp.geometry)
    });

    // keepout(comp) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â extract the clearance envelope SDF
    engine.register_fn("keepout", |comp: ComponentHandle| {
        SdfHandle(comp.keepout)
    });

    // mass_g(comp) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â read mass value
    engine.register_fn("mass_g", |comp: ComponentHandle| {
        comp.mass_g as f64
    });

    engine.register_fn("servo_tray", |servo_len: f64, servo_width: f64, servo_height: f64, wall: f64, rail_height: f64| {
        let tray = open_top_tray(servo_len as f32 + wall as f32 * 2.0, servo_width as f32 + wall as f32 * 2.0, servo_height as f32 + rail_height as f32, wall as f32);
        let rail: Arc<dyn crate::sdf::Sdf> = Arc::new(SdfBox::new(Vec3::new(
            servo_len as f32 * 0.45,
            wall as f32 * 0.5,
            rail_height as f32 * 0.5,
        )));
        let left_rail = Arc::new(Translate::new(Arc::clone(&rail), Vec3::new(0.0, servo_width as f32 * 0.5, servo_height as f32 * 0.5)));
        let right_rail = Arc::new(Translate::new(rail, Vec3::new(0.0, -servo_width as f32 * 0.5, servo_height as f32 * 0.5)));
        SdfHandle(Arc::new(Union::new(Arc::new(Union::new(tray, left_rail)), right_rail)))
    });

    engine.register_fn("battery_cradle", |length: f64, width: f64, height: f64, wall: f64, strap_slot_width: f64| {
        let tray = open_top_tray(length as f32 + wall as f32 * 2.0, width as f32 + wall as f32 * 2.0, height as f32 + wall as f32, wall as f32);
        let slot: Arc<dyn crate::sdf::Sdf> = Arc::new(SdfBox::new(Vec3::new(
            strap_slot_width as f32 * 0.5,
            (width as f32 + wall as f32 * 4.0) * 0.5,
            (wall as f32 * 1.4).max(0.5) * 0.5,
        )));
        let slot_a = Arc::new(Translate::new(Arc::clone(&slot), Vec3::new(length as f32 * 0.25, 0.0, 0.0)));
        let slot_b = Arc::new(Translate::new(slot, Vec3::new(-length as f32 * 0.25, 0.0, 0.0)));
        SdfHandle(Arc::new(Subtract::new(Arc::new(Subtract::new(tray, slot_a)), slot_b)))
    });

    engine.register_fn("fc_stack_mount", |width: f64, length: f64, hole_spacing: f64, standoff_height: f64| {
        let base = Arc::new(SdfBox::new(Vec3::new(width as f32 * 0.5, length as f32 * 0.5, 1.0)));
        let post: Arc<dyn crate::sdf::Sdf> = Arc::new(Cylinder::new(2.5, standoff_height as f32 * 0.5));
        let post: Arc<dyn crate::sdf::Sdf> = Arc::new(Translate::new(post, Vec3::new(0.0, 0.0, standoff_height as f32 * 0.5)));
        let hs = hole_spacing as f32 * 0.5;
        let p1 = Arc::new(Translate::new(Arc::clone(&post), Vec3::new( hs,  hs, 0.0)));
        let p2 = Arc::new(Translate::new(Arc::clone(&post), Vec3::new( hs, -hs, 0.0)));
        let p3 = Arc::new(Translate::new(Arc::clone(&post), Vec3::new(-hs,  hs, 0.0)));
        let p4 = Arc::new(Translate::new(post, Vec3::new(-hs, -hs, 0.0)));
        let posts = Arc::new(Union::new(Arc::new(Union::new(p1, p2)), Arc::new(Union::new(p3, p4))));
        SdfHandle(Arc::new(Union::new(base, posts)))
    });

    engine.register_fn("pushrod_guide", |length: f64, outer_diam: f64, inner_diam: f64| {
        let outer = Arc::new(Cylinder::new(outer_diam as f32 * 0.5, length as f32 * 0.5));
        let inner = Arc::new(Cylinder::new(inner_diam as f32 * 0.5, (length as f32 + 1.0) * 0.5));
        let tube = Arc::new(Subtract::new(outer, inner));
        SdfHandle(Arc::new(Rotate::new(tube, Quat::from_rotation_y(std::f32::consts::FRAC_PI_2))))
    });

    engine.register_fn("antenna_mount", |mast_height: f64, base_radius: f64, mast_radius: f64| {
        let base: Arc<dyn crate::sdf::Sdf> = Arc::new(Cylinder::new(base_radius as f32, 1.2));
        let mast: Arc<dyn crate::sdf::Sdf> = Arc::new(Cylinder::new(mast_radius as f32, mast_height as f32 * 0.5));
        let mast = Arc::new(Translate::new(mast, Vec3::new(0.0, 0.0, mast_height as f32 * 0.5 + 1.2)));
        SdfHandle(Arc::new(Union::new(base, mast)))
    });

    engine.register_fn("pushrod_length", |x0: f64, y0: f64, z0: f64, x1: f64, y1: f64, z1: f64| {
        Vec3::new((x1 - x0) as f32, (y1 - y0) as f32, (z1 - z0) as f32).length() as f64
    });

    engine.register_fn("control_throw", |horn_radius_mm: f64, deflection_deg: f64| {
        (horn_radius_mm as f32 * (deflection_deg as f32).to_radians()).abs() as f64
    });
}

/// Register component-aware bulkhead functions that use the live placed-component list.
/// Call this from evaluate_script_full after creating comp_collector.
pub fn register_drone_auto_functions(
    engine: &mut Engine,
    comp_collector: Arc<Mutex<Vec<ComponentHandle>>>,
) {
    use crate::sdf::aerospace::structural_drone::estimate_radius_at;

    // bulkhead_with_components(fuselage, position, thickness, components, margin) -> SdfHandle
    engine.register_fn(
        "bulkhead_with_components",
        |fuselage: SdfHandle, position: f64, thickness: f64,
         components: rhai::Array, margin: f64| {
            let keepouts: Vec<(Arc<dyn crate::sdf::Sdf>, f32)> = components
                .iter()
                .filter_map(|v| v.clone().try_cast::<ComponentHandle>())
                .map(|c| (c.keepout, margin as f32))
                .collect();
            SdfHandle(bulkhead_with_keepouts(
                fuselage.0,
                position as f32,
                thickness as f32,
                &keepouts,
                2.5,
            ))
        },
    );

    // cable_hole(bulkhead, y, z, diameter) -> SdfHandle
    engine.register_fn("cable_hole", |bk: SdfHandle, y: f64, z: f64, diameter: f64| {
        SdfHandle(cable_hole_at(bk.0, y as f32, z as f32, diameter as f32, 2.5))
    });

    // bulkhead_auto(fuselage, position, thickness) -> SdfHandle
    // Uses all components placed so far via place().
    {
        let cc = Arc::clone(&comp_collector);
        engine.register_fn(
            "bulkhead_auto",
            move |fuselage: SdfHandle, position: f64, thickness: f64| {
                let comps = cc.lock().unwrap().clone();
                let fuse_r = estimate_radius_at(&fuselage.0, position as f32);
                let keepouts: Vec<(Arc<dyn crate::sdf::Sdf>, f32)> = comps
                    .iter()
                    .filter(|c| {
                        keepout_intersects_plane(&c.keepout, position as f32, fuse_r)
                    })
                    .map(|c| (Arc::clone(&c.keepout), 1.5_f32))
                    .collect();
                SdfHandle(bulkhead_with_keepouts(
                    fuselage.0,
                    position as f32,
                    thickness as f32,
                    &keepouts,
                    2.5,
                ))
            },
        );
    }

    // auto_bulkheads(fuselage, count, thickness) -> SdfHandle
    // Evenly spaced bulkheads from x=0.1 to x=0.9, each aware of placed components.
    {
        let cc = Arc::clone(&comp_collector);
        engine.register_fn(
            "auto_bulkheads",
            move |fuselage: SdfHandle, count: i64, thickness: f64| {
                let n = count.max(1) as usize;
                let comps = cc.lock().unwrap().clone();
                let fuse_arc = Arc::clone(&fuselage.0);
                let mut result: Option<Arc<dyn crate::sdf::Sdf>> = None;
                for i in 0..n {
                    let pos = if n == 1 {
                        0.5_f32
                    } else {
                        0.1 + (i as f32 / (n - 1) as f32) * 0.8
                    };
                    let fuse_r = estimate_radius_at(&fuse_arc, pos);
                    let keepouts: Vec<(Arc<dyn crate::sdf::Sdf>, f32)> = comps
                        .iter()
                        .filter(|c| {
                            keepout_intersects_plane(&c.keepout, pos, fuse_r)
                        })
                        .map(|c| (Arc::clone(&c.keepout), 1.5_f32))
                        .collect();
                    let bk = bulkhead_with_keepouts(
                        Arc::clone(&fuse_arc),
                        pos,
                        thickness as f32,
                        &keepouts,
                        2.5,
                    );
                    result = Some(match result {
                        None    => bk,
                        Some(r) => Arc::new(crate::sdf::booleans::Union::new(r, bk)),
                    });
                }
                SdfHandle(result.unwrap_or_else(|| {
                    Arc::new(crate::sdf::primitives::Sphere::new(0.001))
                        as Arc<dyn crate::sdf::Sdf>
                }))
            },
        );
    }

    // auto_bulkheads_at(fuselage, positions, thickness) -> SdfHandle
    // Bulkheads at explicitly specified X positions, each aware of placed components.
    {
        let cc = Arc::clone(&comp_collector);
        engine.register_fn(
            "auto_bulkheads_at",
            move |fuselage: SdfHandle, positions: rhai::Array, thickness: f64| {
                let comps = cc.lock().unwrap().clone();
                let fuse_arc = Arc::clone(&fuselage.0);
                let mut result: Option<Arc<dyn crate::sdf::Sdf>> = None;
                for pv in &positions {
                    let pos = pv.as_float().unwrap_or(0.5) as f32;
                    let fuse_r = estimate_radius_at(&fuse_arc, pos);
                    let keepouts: Vec<(Arc<dyn crate::sdf::Sdf>, f32)> = comps
                        .iter()
                        .filter(|c| {
                            keepout_intersects_plane(&c.keepout, pos, fuse_r)
                        })
                        .map(|c| (Arc::clone(&c.keepout), 1.5_f32))
                        .collect();
                    let bk = bulkhead_with_keepouts(
                        Arc::clone(&fuse_arc),
                        pos,
                        thickness as f32,
                        &keepouts,
                        2.5,
                    );
                    result = Some(match result {
                        None    => bk,
                        Some(r) => Arc::new(crate::sdf::booleans::Union::new(r, bk)),
                    });
                }
                SdfHandle(result.unwrap_or_else(|| {
                    Arc::new(crate::sdf::primitives::Sphere::new(0.001))
                        as Arc<dyn crate::sdf::Sdf>
                }))
            },
        );
    }
}

/// Register mass annotation functions and auto_fuselage.
/// Call this separately from register_sdf_functions, passing the shared collector.
pub fn register_mass_functions(engine: &mut Engine, collector: Arc<Mutex<Vec<MassPoint>>>) {
    // mass_at(mass_g, x, y, z) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â declare a point mass at a position
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

    // mass_named(name, mass_g, x, y, z) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â declare a named point mass
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

    // auto_fuselage(internal_sdf, skin_thickness) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â wrap internal geometry with an outer skin.
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
    // conformal_gyroid ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â uniform density
    engine.register_fn("conformal_gyroid",
        |parent: SdfHandle, cell_size: f64, thickness: f64| {
        SdfHandle(Arc::new(ConformalGyroid::new(parent.0, cell_size as f32, thickness as f32)))
    });

    // conformal_gyroid_field ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â spatially varying density
    engine.register_fn("conformal_gyroid_field",
        |parent: SdfHandle, cell_size: f64, thickness: f64, field: FieldHandle| {
        SdfHandle(Arc::new(ConformalGyroid::with_density_field(
            parent.0, cell_size as f32, thickness as f32, field.0,
        )))
    });

    // conformal_gyroid_region ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â lattice only inside the region mask
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

// ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Longitudinal spine functions ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

/// Register `spline_fuselage(stations, length)` ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â like `fuselage()` but applies
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

// ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Profile (spline cross-section) functions ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

/// Register `spline(name)` ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ SdfHandle and `spline_section(name)` ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ SectionHandle.
///
/// Both functions look up `name` in `profiles`.  If the name is not found a
/// unit-circle default is returned so the script doesn't error.
pub fn register_profile_functions(
    engine: &mut Engine,
    profiles: Arc<RwLock<HashMap<String, SplineProfile>>>,
) {
    // spline(name) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ SdfHandle
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

    // spline_section(name) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ SectionHandle
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

// ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ FEA boundary condition functions ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

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

    crate::scripting::legacy_api::register_legacy_fea_compat_functions(engine, Arc::clone(&collector));

    // stress_field() / displacement_field() ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â available after a successful FEA run
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

// ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Mechanical pattern functions ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

fn register_mechanical_functions(engine: &mut Engine) {
    // bolt_circle(hole_radius, pattern_radius, count, depth) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â polar bolt array
    engine.register_fn("bolt_circle",
        |hole_r: f64, pcd_r: f64, count: i64, depth: f64| {
        SdfHandle(bolt_circle(hole_r as f32, pcd_r as f32, count.max(1) as usize, depth as f32))
    });

    // bolt_square(hole_radius, x_spacing, y_spacing, depth) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â 4 corners, square pattern
    engine.register_fn("bolt_square",
        |hole_r: f64, xs: f64, ys: f64, depth: f64| {
        SdfHandle(bolt_square(hole_r as f32, xs as f32, ys as f32, depth as f32))
    });

    // bolt_rect ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â identical to bolt_square but name clarifies non-square patterns
    engine.register_fn("bolt_rect",
        |hole_r: f64, xs: f64, ys: f64, depth: f64| {
        SdfHandle(bolt_rect(hole_r as f32, xs as f32, ys as f32, depth as f32))
    });

    // countersink(shaft_radius, head_radius, head_depth, shaft_depth)
    engine.register_fn("countersink",
        |shaft_r: f64, head_r: f64, head_d: f64, shaft_d: f64| {
        SdfHandle(countersink(shaft_r as f32, head_r as f32, head_d as f32, shaft_d as f32))
    });

    // counterbore(shaft_radius, bore_radius, bore_depth, shaft_depth)
    engine.register_fn("counterbore",
        |shaft_r: f64, bore_r: f64, bore_d: f64, shaft_d: f64| {
        SdfHandle(counterbore(shaft_r as f32, bore_r as f32, bore_d as f32, shaft_d as f32))
    });

    // slot(width, length, depth) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â rounded stadium slot, length along X
    engine.register_fn("slot",
        |width: f64, length: f64, depth: f64| {
        SdfHandle(slot(width as f32, length as f32, depth as f32))
    });

    // chamfer_edge(body, distance) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â approximate convex edge chamfer
    engine.register_fn("chamfer_edge",
        |body: SdfHandle, distance: f64| {
        SdfHandle(chamfer_edge(body.0, distance as f32))
    });

    // thread_hole(radius, pitch, depth) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â cosmetic threaded hole (visual only)
    engine.register_fn("thread_hole",
        |radius: f64, pitch: f64, depth: f64| {
        SdfHandle(thread_hole(radius as f32, pitch as f32, depth as f32))
    });

    // fc_mount(pattern_mm, hole_radius, plate_thickness) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â FC mounting plate
    engine.register_fn("fc_mount",
        |pattern: f64, hole_r: f64, thickness: f64| {
        SdfHandle(fc_mount(pattern as f32, hole_r as f32, thickness as f32))
    });

    // motor_mount_pattern(motor_size_mm, hole_radius, depth)
    // motor_size_mm = stator diameter (e.g. 22 for 2204, 28 for 2806)
    engine.register_fn("motor_mount_pattern",
        |motor_mm: f64, hole_r: f64, depth: f64| {
        SdfHandle(motor_mount_pattern(motor_mm as f32, hole_r as f32, depth as f32))
    });
}

// ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Sweep functions ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

fn register_sweep_functions(engine: &mut Engine) {
    use crate::sdf::sweep::{
        ConformalSplinePath, LinePath, PolylinePath, SplinePath, SurfaceSpinePath, Sweep, SweepPath,
    };
    use crate::sdf::profiles::{SplineProfile, RectProfile, RoundedRectProfile, NGonProfile};

    fn parse_vec3_list(
        fn_name: &str,
        label: &str,
        pts: rhai::Array,
    ) -> Result<Vec<Vec3>, Box<rhai::EvalAltResult>> {
        let mut verts = Vec::with_capacity(pts.len());
        for (i, v) in pts.into_iter().enumerate() {
            let arr = v.clone().try_cast::<rhai::Array>()
                .ok_or_else(|| format!("{}: {} {} must be [x,y,z]", fn_name, label, i))?;
            if arr.len() < 3 {
                return Err(format!("{}: {} {} must have 3 elements", fn_name, label, i).into());
            }
            verts.push(Vec3::new(
                arr[0].clone().try_cast::<f64>().unwrap_or(0.0) as f32,
                arr[1].clone().try_cast::<f64>().unwrap_or(0.0) as f32,
                arr[2].clone().try_cast::<f64>().unwrap_or(0.0) as f32,
            ));
        }
        Ok(verts)
    }

    fn parse_vec3(
        fn_name: &str,
        label: &str,
        arr: rhai::Array,
    ) -> Result<Vec3, Box<rhai::EvalAltResult>> {
        if arr.len() < 3 {
            return Err(format!("{}: {} must be [x,y,z]", fn_name, label).into());
        }
        Ok(Vec3::new(
            arr[0].clone().try_cast::<f64>().unwrap_or(0.0) as f32,
            arr[1].clone().try_cast::<f64>().unwrap_or(0.0) as f32,
            arr[2].clone().try_cast::<f64>().unwrap_or(0.0) as f32,
        ))
    }

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Path constructors ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

    engine.register_fn("line_path",
        |x1: f64, y1: f64, z1: f64, x2: f64, y2: f64, z2: f64| {
        PathHandle(Arc::new(LinePath {
            start: Vec3::new(x1 as f32, y1 as f32, z1 as f32),
            end:   Vec3::new(x2 as f32, y2 as f32, z2 as f32),
        }))
    });

    engine.register_fn("polyline_path",
        |pts: rhai::Array| -> Result<PathHandle, Box<rhai::EvalAltResult>> {
        let mut verts = Vec::with_capacity(pts.len());
        for (i, v) in pts.iter().enumerate() {
            let arr = v.clone().try_cast::<rhai::Array>()
                .ok_or_else(|| format!("polyline_path: item {} must be [x,y,z]", i))?;
            if arr.len() < 3 {
                return Err(format!("polyline_path: item {} must have 3 elements", i).into());
            }
            let x = arr[0].clone().try_cast::<f64>().unwrap_or(0.0) as f32;
            let y = arr[1].clone().try_cast::<f64>().unwrap_or(0.0) as f32;
            let z = arr[2].clone().try_cast::<f64>().unwrap_or(0.0) as f32;
            verts.push(Vec3::new(x, y, z));
        }
        if verts.len() < 2 {
            return Err("polyline_path: need at least 2 points".into());
        }
        Ok(PathHandle(Arc::new(PolylinePath::new(verts))))
    });

    engine.register_fn("spline_path",
        |pts: rhai::Array| -> Result<PathHandle, Box<rhai::EvalAltResult>> {
        let mut verts = Vec::with_capacity(pts.len());
        for (i, v) in pts.iter().enumerate() {
            let arr = v.clone().try_cast::<rhai::Array>()
                .ok_or_else(|| format!("spline_path: item {} must be [x,y,z]", i))?;
            if arr.len() < 3 {
                return Err(format!("spline_path: item {} must have 3 elements", i).into());
            }
            let x = arr[0].clone().try_cast::<f64>().unwrap_or(0.0) as f32;
            let y = arr[1].clone().try_cast::<f64>().unwrap_or(0.0) as f32;
            let z = arr[2].clone().try_cast::<f64>().unwrap_or(0.0) as f32;
            verts.push(Vec3::new(x, y, z));
        }
        if verts.len() < 2 {
            return Err("spline_path: need at least 2 points".into());
        }
        Ok(PathHandle(Arc::new(SplinePath::new(verts))))
    });

    engine.register_fn("surface_path",
        |surface: SdfHandle,
         x1: f64, y1: f64, z1: f64,
         x2: f64, y2: f64, z2: f64,
         steps: i64| {
        PathHandle(Arc::new(SurfaceSpinePath::new(
            surface.0,
            Vec3::new(x1 as f32, y1 as f32, z1 as f32),
            Vec3::new(x2 as f32, y2 as f32, z2 as f32),
            steps.max(2) as usize,
        )))
    });

    engine.register_fn("conformal_spline_path",
        |surface: SdfHandle,
         pts: rhai::Array,
         offset: f64,
         samples: i64| -> Result<PathHandle, Box<rhai::EvalAltResult>> {
        let mut verts = Vec::with_capacity(pts.len());
        for (i, v) in pts.iter().enumerate() {
            let arr = v.clone().try_cast::<rhai::Array>()
                .ok_or_else(|| format!("conformal_spline_path: item {} must be [x,y,z]", i))?;
            if arr.len() < 3 {
                return Err(format!("conformal_spline_path: item {} must have 3 elements", i).into());
            }
            let x = arr[0].clone().try_cast::<f64>().unwrap_or(0.0) as f32;
            let y = arr[1].clone().try_cast::<f64>().unwrap_or(0.0) as f32;
            let z = arr[2].clone().try_cast::<f64>().unwrap_or(0.0) as f32;
            verts.push(Vec3::new(x, y, z));
        }
        if verts.len() < 2 {
            return Err("conformal_spline_path: need at least 2 points".into());
        }
        Ok(PathHandle(Arc::new(ConformalSplinePath::new(
            surface.0,
            verts,
            offset as f32,
            samples.max(2) as usize,
        ))))
    });

    engine.register_fn("conformal_profile_inlet",
        |surface: SdfHandle,
         guide_pts: rhai::Array,
         duct_path: PathHandle,
         outer_start: ProfileHandle,
         outer_end: ProfileHandle,
         inner_start: ProfileHandle,
         inner_end: ProfileHandle,
         surface_offset: f64,
         face_clearance: f64,
         inlet_open_extension: f64,
         outlet_open_extension: f64,
         samples: i64|
         -> Result<rhai::Array, Box<rhai::EvalAltResult>> {
        let verts = parse_vec3_list("conformal_profile_inlet", "guide point", guide_pts)?;
        if verts.len() < 2 {
            return Err("conformal_profile_inlet: need at least 2 guide points".into());
        }
        let parts = build_conformal_profile_inlet(
            surface.0,
            verts,
            duct_path.0,
            outer_start.0,
            outer_end.0,
            inner_start.0,
            inner_end.0,
            surface_offset as f32,
            face_clearance as f32,
            inlet_open_extension as f32,
            outlet_open_extension as f32,
            samples.max(8) as usize,
        );
        Ok(vec![
            rhai::Dynamic::from(SdfHandle(parts.outer_fairing)),
            rhai::Dynamic::from(SdfHandle(parts.duct_void)),
            rhai::Dynamic::from(SdfHandle(parts.internal_shell)),
        ])
    });

    engine.register_fn("conformal_profile_inlet",
        |surface: SdfHandle,
         guide_pts: rhai::Array,
         duct_path: PathHandle,
         outer_start: ProfileHandle,
         outer_end: ProfileHandle,
         inner_start: ProfileHandle,
         inner_end: ProfileHandle,
         surface_offset: f64,
         inlet_open_extension: f64,
         outlet_open_extension: f64,
         samples: i64|
         -> Result<rhai::Array, Box<rhai::EvalAltResult>> {
        let verts = parse_vec3_list("conformal_profile_inlet", "guide point", guide_pts)?;
        if verts.len() < 2 {
            return Err("conformal_profile_inlet: need at least 2 guide points".into());
        }
        let parts = build_conformal_profile_inlet(
            surface.0,
            verts,
            duct_path.0,
            outer_start.0,
            outer_end.0,
            inner_start.0,
            inner_end.0,
            surface_offset as f32,
            surface_offset as f32,
            inlet_open_extension as f32,
            outlet_open_extension as f32,
            samples.max(8) as usize,
        );
        Ok(vec![
            rhai::Dynamic::from(SdfHandle(parts.outer_fairing)),
            rhai::Dynamic::from(SdfHandle(parts.duct_void)),
            rhai::Dynamic::from(SdfHandle(parts.internal_shell)),
        ])
    });

    engine.register_fn("conformal_inlet",
        |surface: SdfHandle,
         guide_pts: rhai::Array,
         outer_w: f64, outer_h: f64,
         inner_w: f64, inner_h: f64,
         outlet_d: f64,
         surface_offset: f64,
         samples: i64,
         outlet_pt: rhai::Array,
         exhaust_pt: rhai::Array|
         -> Result<rhai::Array, Box<rhai::EvalAltResult>> {
        let verts = parse_vec3_list("conformal_inlet", "guide point", guide_pts)?;
        if verts.len() < 2 {
            return Err("conformal_inlet: need at least 2 guide points".into());
        }
        let outlet = parse_vec3("conformal_inlet", "outlet point", outlet_pt)?;
        let exhaust = parse_vec3("conformal_inlet", "exhaust point", exhaust_pt)?;
        let sample_count = samples.max(8) as usize;

        let preview_path = ConformalSplinePath::new(
            Arc::clone(&surface.0),
            verts.clone(),
            surface_offset as f32,
            sample_count,
        );
        let mouth_start = preview_path.evaluate(0.0);
        let mouth_end = preview_path.evaluate(1.0);
        let mouth_tangent = preview_path.tangent(0.0).normalize_or_zero();
        let throat = mouth_end + Vec3::new(
            0.0,
            0.0,
            -((inner_h as f32 * 0.55) + (surface_offset as f32 * 0.45)),
        );
        let transition_ctrl = throat.lerp(outlet, 0.45) + Vec3::new(
            0.0,
            0.0,
            (outlet.z - throat.z) * 0.18,
        );
        let branch_ctrl = mouth_start + mouth_tangent * (inner_w as f32 * 0.18);
        let duct_path: Arc<dyn crate::sdf::sweep::SweepPath> = Arc::new(SplinePath::new(vec![
            mouth_start,
            branch_ctrl,
            mouth_end,
            throat,
            transition_ctrl,
            outlet,
            outlet.lerp(exhaust, 0.45),
            exhaust,
        ]));

        let outer_start: Arc<dyn Section2D> = {
            let mut p = SplineProfile::circle(16, 1.0);
            for pt in &mut p.control_points {
                pt[0] *= outer_w as f32 * 0.5;
                pt[1] *= outer_h as f32 * 0.5;
            }
            Arc::new(p)
        };
        let inner_start: Arc<dyn Section2D> = {
            let mut p = SplineProfile::circle(16, 1.0);
            for pt in &mut p.control_points {
                pt[0] *= inner_w as f32 * 0.5;
                pt[1] *= inner_h as f32 * 0.5;
            }
            Arc::new(p)
        };
        let wall_thickness = (((outer_w - inner_w).max(0.0) + (outer_h - inner_h).max(0.0)) * 0.25) as f32;
        let outer_end: Arc<dyn Section2D> = Arc::new(SplineProfile::circle(
            16,
            outlet_d as f32 * 0.5 + wall_thickness.max(1e-4),
        ));
        let inner_end: Arc<dyn Section2D> = Arc::new(SplineProfile::circle(16, outlet_d as f32 * 0.5));

        let parts = build_conformal_profile_inlet(
            surface.0,
            verts,
            duct_path,
            outer_start,
            outer_end,
            inner_start,
            inner_end,
            surface_offset as f32,
            surface_offset as f32,
            inner_w.max(inner_h) as f32 * 0.35,
            outlet_d as f32 * 0.5,
            sample_count,
        );
        Ok(vec![
            rhai::Dynamic::from(SdfHandle(parts.outer_fairing)),
            rhai::Dynamic::from(SdfHandle(parts.duct_void)),
        ])
    });

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Profile constructors ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

    // circle_profile ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â convenience alias for a SplineProfile circle.
    engine.register_fn("circle_profile", |radius: f64| {
        ProfileHandle(Arc::new(SplineProfile::circle(12, radius as f32)))
    });

    // ellipse_profile ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â elliptical 2D profile.
    engine.register_fn("ellipse_profile", |width: f64, height: f64| {
        // Build an ellipse as a scaled circle spline.
        let mut p = SplineProfile::circle(12, 1.0);
        for pt in &mut p.control_points {
            pt[0] *= width as f32 / 2.0;
            pt[1] *= height as f32 / 2.0;
        }
        ProfileHandle(Arc::new(p))
    });

    // rect_profile ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â rectangular cross-section.
    engine.register_fn("rect_profile", |width: f64, height: f64| {
        ProfileHandle(Arc::new(RectProfile::new(width as f32, height as f32)))
    });

    engine.register_fn("rounded_rect_profile", |width: f64, height: f64, radius: f64| {
        ProfileHandle(Arc::new(RoundedRectProfile::new(width as f32, height as f32, radius as f32)))
    });

    // ngon_profile ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â regular n-gon cross-section.
    engine.register_fn("ngon_profile", |sides: i64, radius: f64| {
        ProfileHandle(Arc::new(NGonProfile::new(sides.max(3) as u32, radius as f32)))
    });

    engine.register_fn("custom_profile",
        |pts: rhai::Array| -> Result<ProfileHandle, Box<rhai::EvalAltResult>> {
        let mut verts = Vec::with_capacity(pts.len());
        for (i, dynv) in pts.into_iter().enumerate() {
            let arr = dynv.clone().try_cast::<rhai::Array>()
                .ok_or_else(|| format!("custom_profile: item {} must be [x,y]", i))?;
            if arr.len() != 2 {
                return Err(format!("custom_profile: item {} must have 2 elements", i).into());
            }
            verts.push(glam::Vec2::new(
                arr[0].clone().cast::<rhai::FLOAT>() as f32,
                arr[1].clone().cast::<rhai::FLOAT>() as f32,
            ));
        }
        if verts.len() < 3 {
            return Err("custom_profile: need at least 3 points".into());
        }
        Ok(ProfileHandle(Arc::new(SplineProfile::new(verts))))
    });

    engine.register_fn("profile_from_csv",
        |path: &str| -> Result<ProfileHandle, Box<rhai::EvalAltResult>> {
        let text = fs::read_to_string(path)
            .map_err(|e| format!("profile_from_csv: failed to read '{}': {}", path, e))?;
        let mut verts = Vec::new();
        for (line_no, line) in text.lines().enumerate() {
            if line_no == 0 {
                continue;
            }
            let trimmed = line.trim();
            if trimmed.is_empty() {
                continue;
            }
            let mut parts = trimmed.split(',');
            let x = parts
                .next()
                .ok_or_else(|| format!("profile_from_csv: line {} missing first column", line_no + 1))?
                .trim()
                .parse::<f32>()
                .map_err(|e| format!("profile_from_csv: line {} invalid first column: {}", line_no + 1, e))?;
            let y = parts
                .next()
                .ok_or_else(|| format!("profile_from_csv: line {} missing second column", line_no + 1))?
                .trim()
                .parse::<f32>()
                .map_err(|e| format!("profile_from_csv: line {} invalid second column: {}", line_no + 1, e))?;
            if verts.last().map(|p: &glam::Vec2| (*p - glam::Vec2::new(x, y)).length_squared() < 1e-12).unwrap_or(false) {
                continue;
            }
            verts.push(glam::Vec2::new(x, y));
        }
        if verts.len() < 3 {
            return Err("profile_from_csv: need at least 3 points".into());
        }
        if (verts[0] - verts[verts.len() - 1]).length_squared() < 1e-12 {
            verts.pop();
        }
        Ok(ProfileHandle(Arc::new(SplineProfile::new(verts))))
    });

    engine.register_fn("section_profile_yz",
        |sdf: SdfHandle,
         x: f64,
         center_y: f64,
         center_z: f64,
         max_radius: f64,
         samples: i64| -> Result<ProfileHandle, Box<rhai::EvalAltResult>> {
        let center = glam::Vec3::new(x as f32, center_y as f32, center_z as f32);
        let radius = max_radius.max(1.0) as f32;
        let n = samples.max(16) as usize;
        let mut pts = Vec::with_capacity(n);
        for i in 0..n {
            let a = std::f32::consts::TAU * i as f32 / n as f32;
            let dir = glam::Vec3::new(0.0, a.cos(), a.sin());
            let hit = crate::sdf::query::surface_point(sdf.0.as_ref(), center, dir, radius)
                .ok_or_else(|| format!("section_profile_yz: no hit at sample {} from center [{:.3},{:.3},{:.3}]", i, center.x, center.y, center.z))?;
            pts.push(glam::Vec2::new(hit.y - center.y, hit.z - center.z));
        }
        Ok(ProfileHandle(Arc::new(SplineProfile::new(pts))))
    });

    engine.register_fn("conformal_rounded_rect_profile",
        |surface: SdfHandle,
         x: f64,
         center_y: f64,
         search_z: f64,
         width: f64,
         height: f64,
         radius: f64,
         face_clearance: f64,
         samples: i64| -> rhai::Array {
            let (profile, center_z) = conformal_rounded_rect_section(
                surface.0,
                x as f32,
                center_y as f32,
                search_z as f32,
                width as f32,
                height as f32,
                radius as f32,
                face_clearance as f32,
                samples.max(16) as usize,
            );
            vec![
                rhai::Dynamic::from(ProfileHandle(profile)),
                rhai::Dynamic::from(center_z as f64),
            ]
        }
    );

    engine.register_fn("conformal_profile",
        |surface: SdfHandle,
         base_profile: ProfileHandle,
         anchor: PointHandle,
         flow_dir: PointHandle,
         face_clearance: f64,
         samples: i64| -> rhai::Array {
            let (profile, center) = conformal_profile_section(
                surface.0,
                base_profile.0,
                anchor.0,
                flow_dir.0,
                face_clearance as f32,
                samples.max(16) as usize,
            );
            vec![
                rhai::Dynamic::from(ProfileHandle(profile)),
                rhai::Dynamic::from(PointHandle(center)),
            ]
        }
    );

    engine.register_fn("conformal_profile_x",
        |surface: SdfHandle,
         base_profile: ProfileHandle,
         x: f64,
         guess_y: f64,
         guess_z: f64,
         flow_dir: PointHandle,
         face_clearance: f64,
         samples: i64| -> rhai::Array {
            let (profile, center) = conformal_profile_section_at_x(
                surface.0,
                base_profile.0,
                x as f32,
                guess_y as f32,
                guess_z as f32,
                flow_dir.0,
                face_clearance as f32,
                samples.max(16) as usize,
            );
            vec![
                rhai::Dynamic::from(ProfileHandle(profile)),
                rhai::Dynamic::from(PointHandle(center)),
            ]
        }
    );

    engine.register_fn("conformal_profile_x",
        |surface: SdfHandle,
         base_profile: ProfileHandle,
         x: f64,
         guess_y: f64,
         guess_z: f64,
         dx: f64, dy: f64, dz: f64,
         face_clearance: f64,
         samples: i64| -> rhai::Array {
            let (profile, center) = conformal_profile_section_at_x(
                surface.0,
                base_profile.0,
                x as f32,
                guess_y as f32,
                guess_z as f32,
                glam::Vec3::new(dx as f32, dy as f32, dz as f32),
                face_clearance as f32,
                samples.max(16) as usize,
            );
            vec![
                rhai::Dynamic::from(ProfileHandle(profile)),
                rhai::Dynamic::from(PointHandle(center)),
            ]
        }
    );

    engine.register_fn("conformal_profile",
        |surface: SdfHandle,
         base_profile: ProfileHandle,
         ax: f64, ay: f64, az: f64,
         dx: f64, dy: f64, dz: f64,
         face_clearance: f64,
         samples: i64| -> rhai::Array {
            let (profile, center) = conformal_profile_section(
                surface.0,
                base_profile.0,
                glam::Vec3::new(ax as f32, ay as f32, az as f32),
                glam::Vec3::new(dx as f32, dy as f32, dz as f32),
                face_clearance as f32,
                samples.max(16) as usize,
            );
            vec![
                rhai::Dynamic::from(ProfileHandle(profile)),
                rhai::Dynamic::from(PointHandle(center)),
            ]
        }
    );

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Sweep constructors ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

    // sweep(profile, path) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â no twist.
    engine.register_fn("sweep",
        |profile: ProfileHandle, path: PathHandle| {
        SdfHandle(Arc::new(Sweep::new(profile.0, path.0, 0.0, 0.0)))
    });

    // sweep_twisted(profile, path, twist_start, twist_end) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â with linear twist.
    engine.register_fn("sweep_twisted",
        |profile: ProfileHandle, path: PathHandle, ts: f64, te: f64| {
        SdfHandle(Arc::new(Sweep::new(profile.0, path.0, ts as f32, te as f32)))
    });

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Drone convenience wrappers ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

    // cable_channel(path, diameter) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â circular hollow for wire routing.
    engine.register_fn("cable_channel",
        |path: PathHandle, diameter: f64| {
        let profile = ProfileHandle(Arc::new(SplineProfile::circle(12, diameter as f32 / 2.0)));
        SdfHandle(Arc::new(Sweep::new(profile.0, path.0, 0.0, 0.0)))
    });

    // carbon_rod(path, outer_d, inner_d) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â hollow carbon tube sweep.
    engine.register_fn("carbon_rod",
        |path: PathHandle, outer_d: f64, inner_d: f64| {
        use crate::sdf::transforms::Shell;
        let outer = SplineProfile::circle(12, outer_d as f32 / 2.0);
        // Approximate shell by using outer profile and relying on caller to
        // subtract inner cylinder.  For a pure SDF shell, we wrap in a Shell.
        let shell_thickness = ((outer_d - inner_d) / 2.0) as f32;
        let outer_sdf = Arc::new(Sweep::new(
            Arc::new(outer) as Arc<dyn Section2D>,
            path.0,
            0.0, 0.0,
        ));
        SdfHandle(Arc::new(Shell::new(outer_sdf, shell_thickness)))
    });

    // control_rod(path, diameter) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â solid rod sweep for pushrods/linkages.
    engine.register_fn("control_rod",
        |path: PathHandle, diameter: f64| {
        let profile = ProfileHandle(Arc::new(SplineProfile::circle(12, diameter as f32 / 2.0)));
        SdfHandle(Arc::new(Sweep::new(profile.0, path.0, 0.0, 0.0)))
    });
}

// ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Mesh import functions ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

fn register_variable_duct_functions(engine: &mut Engine) {
    engine.register_fn("spline_tube_solid",
        |path: PathHandle,
         start_d: f64, end_d: f64,
         samples: i64, smoothness: f64| {
        SdfHandle(Arc::new(SplineTube::new(
            path.0,
            start_d as f32,
            end_d as f32,
            samples.max(8) as usize,
            smoothness as f32,
        )))
    });

    engine.register_fn("spline_tube",
        |path: PathHandle,
         start_inner_d: f64, end_inner_d: f64,
         wall_thickness: f64,
         samples: i64, smoothness: f64| {
        SdfHandle(Arc::new(HollowSplineTube::new(
            path.0,
            start_inner_d as f32,
            end_inner_d as f32,
            wall_thickness as f32,
            samples.max(8) as usize,
            smoothness as f32,
        )))
    });

    engine.register_fn("variable_duct_solid",
        |path: PathHandle,
         inlet_w: f64, inlet_h: f64,
         outlet_w: f64, outlet_h: f64,
         samples: i64, smoothness: f64| {
        SdfHandle(Arc::new(VariableDuct::new(
            path.0,
            inlet_w as f32,
            inlet_h as f32,
            outlet_w as f32,
            outlet_h as f32,
            samples.max(4) as usize,
            smoothness as f32,
        )))
    });

    engine.register_fn("variable_duct",
        |path: PathHandle,
         inlet_w: f64, inlet_h: f64,
         outlet_w: f64, outlet_h: f64,
         wall_thickness: f64,
         samples: i64, smoothness: f64| {
        SdfHandle(Arc::new(HollowVariableDuct::new(
            path.0,
            inlet_w as f32,
            inlet_h as f32,
            outlet_w as f32,
            outlet_h as f32,
            wall_thickness as f32,
            samples.max(4) as usize,
            smoothness as f32,
        )))
    });

    engine.register_fn("variable_duct_circular_outlet",
        |path: PathHandle,
         inlet_w: f64, inlet_h: f64,
         outlet_d: f64,
         wall_thickness: f64,
         samples: i64, smoothness: f64| {
        let od = outlet_d as f32;
        SdfHandle(Arc::new(HollowVariableDuct::new(
            path.0,
            inlet_w as f32,
            inlet_h as f32,
            od,
            od,
            wall_thickness as f32,
            samples.max(4) as usize,
            smoothness as f32,
        )))
    });

    engine.register_fn("profile_duct_solid",
        |path: PathHandle,
         start_profile: ProfileHandle,
         end_profile: ProfileHandle,
         samples: i64| {
        SdfHandle(Arc::new(ProfileDuct::new(
            path.0,
            start_profile.0,
            end_profile.0,
            samples.max(8) as usize,
        )))
    });

    engine.register_fn("profile_duct_fixed_solid",
        |path: PathHandle,
         start_profile: ProfileHandle,
         end_profile: ProfileHandle,
         samples: i64| {
        SdfHandle(Arc::new(FixedProfileDuct::new(
            path.0,
            start_profile.0,
            end_profile.0,
            samples.max(8) as usize,
        )))
    });

    engine.register_fn("profile_duct",
        |path: PathHandle,
         outer_start: ProfileHandle,
         outer_end: ProfileHandle,
         inner_start: ProfileHandle,
         inner_end: ProfileHandle,
         start_extension: f64,
         end_extension: f64,
         samples: i64| {
        SdfHandle(Arc::new(HollowProfileDuct::new(
            path.0,
            outer_start.0,
            outer_end.0,
            inner_start.0,
            inner_end.0,
            start_extension as f32,
            end_extension as f32,
            samples.max(8) as usize,
        )))
    });

    engine.register_fn("profile_duct_fixed",
        |path: PathHandle,
         outer_start: ProfileHandle,
         outer_end: ProfileHandle,
         inner_start: ProfileHandle,
         inner_end: ProfileHandle,
         start_extension: f64,
         end_extension: f64,
         samples: i64| {
        SdfHandle(Arc::new(HollowFixedProfileDuct::new(
            path.0,
            outer_start.0,
            outer_end.0,
            inner_start.0,
            inner_end.0,
            start_extension as f32,
            end_extension as f32,
            samples.max(8) as usize,
        )))
    });

    engine.register_fn("profile_duct_fixed_solid_scheduled",
        |path: PathHandle,
         start_profile: ProfileHandle,
         end_profile: ProfileHandle,
         morph_start: f64,
         morph_end: f64,
         samples: i64| {
        SdfHandle(Arc::new(FixedProfileDuct::with_schedule(
            path.0,
            start_profile.0,
            end_profile.0,
            morph_start as f32,
            morph_end as f32,
            samples.max(8) as usize,
        )))
    });

    engine.register_fn("profile_duct_fixed_scheduled",
        |path: PathHandle,
         outer_start: ProfileHandle,
         outer_end: ProfileHandle,
         inner_start: ProfileHandle,
         inner_end: ProfileHandle,
         start_extension: f64,
         end_extension: f64,
         morph_start: f64,
         morph_end: f64,
         samples: i64| {
        SdfHandle(Arc::new(HollowFixedProfileDuct::with_schedule(
            path.0,
            outer_start.0,
            outer_end.0,
            inner_start.0,
            inner_end.0,
            start_extension as f32,
            end_extension as f32,
            morph_start as f32,
            morph_end as f32,
            samples.max(8) as usize,
        )))
    });

    engine.register_fn("conformal_profile_duct_x",
        |surface: SdfHandle,
         outer_base_start: ProfileHandle,
         inner_base_start: ProfileHandle,
         x: f64,
         guess_y: f64,
         guess_z: f64,
         flow_dir: PointHandle,
         face_clearance: f64,
         duct_path: PathHandle,
         outer_end: ProfileHandle,
         inner_end: ProfileHandle,
         start_extension: f64,
         end_extension: f64,
         morph_start: f64,
         morph_end: f64,
         samples: i64| -> rhai::Array {
            let parts = build_conformal_profile_duct_at_x(
                surface.0,
                outer_base_start.0,
                inner_base_start.0,
                x as f32,
                guess_y as f32,
                guess_z as f32,
                flow_dir.0,
                face_clearance as f32,
                duct_path.0,
                outer_end.0,
                inner_end.0,
                start_extension as f32,
                end_extension as f32,
                morph_start as f32,
                morph_end as f32,
                samples.max(8) as usize,
            );
            vec![
                rhai::Dynamic::from(SdfHandle(parts.outer_body)),
                rhai::Dynamic::from(SdfHandle(parts.duct_void)),
                rhai::Dynamic::from(SdfHandle(parts.duct_shell)),
                rhai::Dynamic::from(ProfileHandle(parts.outer_start_profile)),
                rhai::Dynamic::from(ProfileHandle(parts.inner_start_profile)),
                rhai::Dynamic::from(PointHandle(parts.mouth_center)),
            ]
        }
    );

    engine.register_fn("dual_conformal_profile_duct_x",
        |surface: SdfHandle,
         outer_base_start: ProfileHandle,
         inner_base_start: ProfileHandle,
         left_x: f64,
         left_guess_y: f64,
         left_guess_z: f64,
         right_x: f64,
         right_guess_y: f64,
         right_guess_z: f64,
         flow_dir: PointHandle,
         left_path: PathHandle,
         right_path: PathHandle,
         outer_end: ProfileHandle,
         inner_end: ProfileHandle,
         face_clearance: f64,
         start_extension: f64,
         end_extension: f64,
         morph_start: f64,
         morph_end: f64,
         samples: i64| -> rhai::Array {
            let parts = build_dual_conformal_profile_duct_at_x(
                surface.0,
                outer_base_start.0,
                inner_base_start.0,
                left_x as f32,
                left_guess_y as f32,
                left_guess_z as f32,
                right_x as f32,
                right_guess_y as f32,
                right_guess_z as f32,
                flow_dir.0,
                left_path.0,
                right_path.0,
                outer_end.0,
                inner_end.0,
                face_clearance as f32,
                start_extension as f32,
                end_extension as f32,
                morph_start as f32,
                morph_end as f32,
                samples.max(8) as usize,
            );
            vec![
                rhai::Dynamic::from(SdfHandle(parts.outer_body)),
                rhai::Dynamic::from(SdfHandle(parts.duct_void)),
                rhai::Dynamic::from(SdfHandle(parts.duct_shell)),
                rhai::Dynamic::from(ProfileHandle(parts.left_outer_start_profile)),
                rhai::Dynamic::from(ProfileHandle(parts.right_outer_start_profile)),
                rhai::Dynamic::from(PointHandle(parts.left_mouth_center)),
                rhai::Dynamic::from(PointHandle(parts.right_mouth_center)),
            ]
        }
    );

    engine.register_fn("mirrored_dual_conformal_profile_duct_x",
        |surface: SdfHandle,
         outer_base_start: ProfileHandle,
         inner_base_start: ProfileHandle,
         x: f64,
         guess_y: f64,
         guess_z: f64,
         flow_dir: PointHandle,
         left_path: PathHandle,
         outer_end: ProfileHandle,
         inner_end: ProfileHandle,
         face_clearance: f64,
         start_extension: f64,
         end_extension: f64,
         morph_start: f64,
         morph_end: f64,
         samples: i64| -> rhai::Array {
            let parts = build_mirrored_dual_conformal_profile_duct_at_x(
                surface.0,
                outer_base_start.0,
                inner_base_start.0,
                x as f32,
                guess_y as f32,
                guess_z as f32,
                flow_dir.0,
                left_path.0,
                outer_end.0,
                inner_end.0,
                face_clearance as f32,
                start_extension as f32,
                end_extension as f32,
                morph_start as f32,
                morph_end as f32,
                samples.max(8) as usize,
            );
            vec![
                rhai::Dynamic::from(SdfHandle(parts.outer_body)),
                rhai::Dynamic::from(SdfHandle(parts.duct_void)),
                rhai::Dynamic::from(SdfHandle(parts.duct_shell)),
                rhai::Dynamic::from(ProfileHandle(parts.left_outer_start_profile)),
                rhai::Dynamic::from(ProfileHandle(parts.right_outer_start_profile)),
                rhai::Dynamic::from(PointHandle(parts.left_mouth_center)),
                rhai::Dynamic::from(PointHandle(parts.right_mouth_center)),
            ]
        }
    );
}

pub fn register_mesh_functions(
    engine:      &mut Engine,
    project_dir: Option<std::path::PathBuf>,
    mesh_cache:  Arc<Mutex<super::MeshCache>>,
) {
    use std::path::PathBuf;
    use crate::mesh::{parse_stl, parse_obj};
    use crate::sdf::mesh_import::MeshSdf;

    // Helper: resolve a user-supplied path relative to project_dir, then load +
    // cache-parse the mesh.  Returns Arc<TriangleMesh> or an error string.
    fn resolve_and_load(
        path_str:    &str,
        project_dir: &Option<PathBuf>,
        cache:       &Arc<Mutex<super::MeshCache>>,
    ) -> Result<Arc<crate::mesh::TriangleMesh>, String> {
        // Resolve path: absolute as-is; relative ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ project_dir / path.
        let path: PathBuf = {
            let p = std::path::Path::new(path_str);
            if p.is_absolute() {
                p.to_path_buf()
            } else if let Some(base) = project_dir {
                base.join(p)
            } else {
                p.to_path_buf()
            }
        };

        // Read mtime for cache invalidation.
        let mtime = std::fs::metadata(&path)
            .and_then(|m| m.modified())
            .unwrap_or(std::time::SystemTime::UNIX_EPOCH);

        // Check cache.
        {
            let guard = cache.lock().unwrap();
            if let Some((cached_mtime, mesh)) = guard.get(&path) {
                if *cached_mtime == mtime {
                    return Ok(Arc::clone(mesh));
                }
            }
        }

        // Cache miss ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â read and parse.
        let ext = path.extension()
            .and_then(|e| e.to_str())
            .unwrap_or("")
            .to_ascii_lowercase();

        let mesh = if ext == "obj" {
            let text = std::fs::read_to_string(&path)
                .map_err(|e| format!("import_mesh: cannot read '{}': {}", path.display(), e))?;
            parse_obj(&text)?
        } else {
            // Default: STL (binary or ASCII).
            let data = std::fs::read(&path)
                .map_err(|e| format!("import_mesh: cannot read '{}': {}", path.display(), e))?;
            parse_stl(&data)?
        };

        let mesh = Arc::new(mesh);
        cache.lock().unwrap().insert(path, (mtime, Arc::clone(&mesh)));
        Ok(mesh)
    }

    // import_mesh(path) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â load mesh, return SdfHandle (approximate mode).
    {
        let dir   = project_dir.clone();
        let cache = Arc::clone(&mesh_cache);
        engine.register_fn("import_mesh",
            move |path: &str| -> Result<SdfHandle, Box<rhai::EvalAltResult>> {
            let mesh = resolve_and_load(path, &dir, &cache)
                .map_err(|e: String| -> Box<rhai::EvalAltResult> { e.into() })?;
            Ok(SdfHandle(Arc::new(MeshSdf::new(mesh))))
        });
    }

    // import_mesh_scaled(path, scale) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â load and uniformly scale.
    {
        let dir   = project_dir.clone();
        let cache = Arc::clone(&mesh_cache);
        engine.register_fn("import_mesh_scaled",
            move |path: &str, scale: f64| -> Result<SdfHandle, Box<rhai::EvalAltResult>> {
            let mesh = resolve_and_load(path, &dir, &cache)
                .map_err(|e: String| -> Box<rhai::EvalAltResult> { e.into() })?;
            let s = scale as f32;
            // Build a scaled copy of the mesh vertices.
            let mut scaled = (*mesh).clone();
            for v in &mut scaled.vertices { *v *= s; }
            scaled.bounds_min *= s;
            scaled.bounds_max *= s;
            Ok(SdfHandle(Arc::new(MeshSdf::new(Arc::new(scaled)))))
        });
    }

    // import_mesh_transform(path, tx, ty, tz, rx_deg, ry_deg, rz_deg, scale)
    {
        let dir   = project_dir.clone();
        let cache = Arc::clone(&mesh_cache);
        engine.register_fn("import_mesh_transform",
            move |path: &str,
                  tx: f64, ty: f64, tz: f64,
                  rx: f64, ry: f64, rz: f64,
                  scale: f64| -> Result<SdfHandle, Box<rhai::EvalAltResult>> {
            let mesh = resolve_and_load(path, &dir, &cache)
                .map_err(|e: String| -> Box<rhai::EvalAltResult> { e.into() })?;
            use std::f32::consts::PI;
            let deg2rad = PI / 180.0;
            let rot = glam::Quat::from_euler(
                glam::EulerRot::XYZ,
                rx as f32 * deg2rad,
                ry as f32 * deg2rad,
                rz as f32 * deg2rad,
            );
            let t  = glam::Vec3::new(tx as f32, ty as f32, tz as f32);
            let s  = scale as f32;
            let mut xformed = (*mesh).clone();
            for v in &mut xformed.vertices { *v = rot * (*v * s) + t; }
            // Recompute bounds.
            let mut bmin = glam::Vec3::splat(f32::MAX);
            let mut bmax = glam::Vec3::splat(f32::MIN);
            for &v in &xformed.vertices { bmin = bmin.min(v); bmax = bmax.max(v); }
            xformed.bounds_min = bmin;
            xformed.bounds_max = bmax;
            Ok(SdfHandle(Arc::new(MeshSdf::new(Arc::new(xformed)))))
        });
    }

    // mesh_info(path) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â returns a Rhai map with mesh metadata.
    {
        let dir   = project_dir.clone();
        let cache = Arc::clone(&mesh_cache);
        engine.register_fn("mesh_info",
            move |path: &str| -> Result<rhai::Map, Box<rhai::EvalAltResult>> {
            use crate::mesh::import::validate_mesh;
            let mesh = resolve_and_load(path, &dir, &cache)
                .map_err(|e: String| -> Box<rhai::EvalAltResult> { e.into() })?;
            let v = validate_mesh(&mesh);
            let mut map = rhai::Map::new();
            map.insert("vertex_count".into(),
                rhai::Dynamic::from(mesh.vertex_count() as i64));
            map.insert("triangle_count".into(),
                rhai::Dynamic::from(mesh.triangle_count() as i64));
            map.insert("bounds_min_x".into(), rhai::Dynamic::from(mesh.bounds_min.x as f64));
            map.insert("bounds_min_y".into(), rhai::Dynamic::from(mesh.bounds_min.y as f64));
            map.insert("bounds_min_z".into(), rhai::Dynamic::from(mesh.bounds_min.z as f64));
            map.insert("bounds_max_x".into(), rhai::Dynamic::from(mesh.bounds_max.x as f64));
            map.insert("bounds_max_y".into(), rhai::Dynamic::from(mesh.bounds_max.y as f64));
            map.insert("bounds_max_z".into(), rhai::Dynamic::from(mesh.bounds_max.z as f64));
            map.insert("is_manifold".into(),
                rhai::Dynamic::from(!v.has_non_manifold));
            map.insert("has_open_boundaries".into(),
                rhai::Dynamic::from(v.has_open_boundary));
            Ok(map)
        });
    }
}

// ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Composite layup functions ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

/// Register a secondary hook that collects CompositeLayup references for the
/// Layup Summary panel.  Must be called AFTER register_composite_functions().
pub fn register_composite_collector(
    engine:    &mut Engine,
    collector: Arc<Mutex<Vec<Arc<crate::sdf::aerospace::composite::CompositeLayup>>>>,
) {
    use crate::sdf::aerospace::composite::CompositeSdf;
    // Override composite_layup to also push the layup to the collector.
    engine.register_fn("composite_layup",
        move |parent: SdfHandle, layers: rhai::Array| -> Result<SdfHandle, Box<rhai::EvalAltResult>> {
        use crate::sdf::aerospace::composite::{CompositeLayup, ShellLayer};
        let mut shell_layers: Vec<ShellLayer> = Vec::with_capacity(layers.len());
        for item in layers {
            let lh: LayerHandle = item.try_cast::<LayerHandle>()
                .ok_or_else(|| -> Box<rhai::EvalAltResult> {
                    "composite_layup: array must contain LayerHandles".into()
                })?;
            let layer = Arc::try_unwrap(lh.0).unwrap_or_else(|arc| (*arc).clone());
            shell_layers.push(layer);
        }
        let layup = Arc::new(CompositeLayup::new(parent.0, shell_layers));
        collector.lock().unwrap().push(Arc::clone(&layup));
        Ok(SdfHandle(Arc::new(CompositeSdf::from_arc(layup))))
    });
}

fn register_composite_functions(engine: &mut Engine) {
    use crate::materials::{CompositeMaterial, CompositeMaterialType, PrintedFilament,
                           find_preset};
    use crate::sdf::aerospace::composite::{
        CompositeSdf, CompositeLayup, ShellLayer,
        wing_composite, fuselage_composite, printed_shell,
    };

    // material(name) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ MaterialHandle ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â look up a built-in preset.
    engine.register_fn("material", |name: &str| -> Result<MaterialHandle, Box<rhai::EvalAltResult>> {
        find_preset(name)
            .map(|m| MaterialHandle(Arc::new(m)))
            .ok_or_else(|| format!("material: unknown preset '{}'", name).into())
    });

    // custom_material(name, density, E, G, nu) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ MaterialHandle
    engine.register_fn("custom_material",
        |name: &str, density: f64, e_mpa: f64, g_mpa: f64, nu: f64| -> MaterialHandle {
        MaterialHandle(Arc::new(CompositeMaterial {
            name:                name.to_string(),
            material_type:       CompositeMaterialType::Printed {
                                     filament: PrintedFilament::Custom },
            density_g_cm3:       density as f32,
            elastic_modulus_mpa: e_mpa  as f32,
            shear_modulus_mpa:   g_mpa  as f32,
            poisson_ratio:       nu     as f32,
            color:               [0.7, 0.7, 0.7],
        }))
    });

    // shell_layer(name, material, thickness) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ LayerHandle
    engine.register_fn("shell_layer",
        |name: &str, mat: MaterialHandle, thickness: f64| -> LayerHandle {
        LayerHandle(Arc::new(ShellLayer::new(name, mat.0, thickness as f32)))
    });

    // shell_layer_field(name, material, base_thickness, field) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ LayerHandle
    engine.register_fn("shell_layer_field",
        |name: &str, mat: MaterialHandle, thickness: f64, field: FieldHandle| -> LayerHandle {
        LayerHandle(Arc::new(
            ShellLayer::new(name, mat.0, thickness as f32)
                .with_field(field.0)
        ))
    });

    // core_layer(name, material, thickness, infill) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ LayerHandle
    engine.register_fn("core_layer",
        |name: &str, mat: MaterialHandle, thickness: f64, infill: SdfHandle| -> LayerHandle {
        LayerHandle(Arc::new(
            ShellLayer::new(name, mat.0, thickness as f32)
                .as_core(Some(infill.0))
        ))
    });

    // solid_core_layer(name, material, thickness) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ LayerHandle ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â solid core, no infill.
    engine.register_fn("solid_core_layer",
        |name: &str, mat: MaterialHandle, thickness: f64| -> LayerHandle {
        LayerHandle(Arc::new(
            ShellLayer::new(name, mat.0, thickness as f32)
                .as_core(None)
        ))
    });

    // composite_layup(parent, [layers]) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ SdfHandle
    engine.register_fn("composite_layup",
        |parent: SdfHandle, layers: rhai::Array| -> Result<SdfHandle, Box<rhai::EvalAltResult>> {
        let mut shell_layers: Vec<ShellLayer> = Vec::with_capacity(layers.len());
        for item in layers {
            let lh: LayerHandle = item.try_cast::<LayerHandle>()
                .ok_or_else(|| -> Box<rhai::EvalAltResult> {
                    "composite_layup: array must contain LayerHandles".into()
                })?;
            // Unwrap Arc ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â if this is the only handle we move it, otherwise clone.
            let layer = Arc::try_unwrap(lh.0)
                .unwrap_or_else(|arc| (*arc).clone());
            shell_layers.push(layer);
        }
        Ok(SdfHandle(Arc::new(CompositeSdf::new(CompositeLayup::new(parent.0, shell_layers)))))
    });

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Aerospace convenience wrappers ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

    // wing_composite(wing, outer_plies, core_thickness, inner_plies) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ SdfHandle
    engine.register_fn("wing_composite",
        |wing: SdfHandle, outer: i64, core_t: f64, inner: i64| -> SdfHandle {
        SdfHandle(wing_composite(wing.0, outer as usize, core_t as f32, inner as usize))
    });

    // fuselage_composite(fuse, outer_plies, core_thickness, inner_plies) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ SdfHandle
    engine.register_fn("fuselage_composite",
        |fuse: SdfHandle, outer: i64, core_t: f64, inner: i64| -> SdfHandle {
        SdfHandle(fuselage_composite(fuse.0, outer as usize, core_t as f32, inner as usize))
    });

    // printed_shell(body, thickness, filament) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ SdfHandle
    engine.register_fn("printed_shell",
        |body: SdfHandle, thickness: f64, filament: &str| -> SdfHandle {
        SdfHandle(printed_shell(body.0, thickness as f32, filament))
    });
}

// ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ PlaneHandle / AlignmentHandle (opaque Rhai wrapper types) ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

/// Rhai-visible wrapper for a split plane.
#[derive(Clone)]
pub struct PlaneHandle(pub SplitPlane);

/// Rhai-visible wrapper for an alignment feature.
#[derive(Clone)]
pub struct AlignmentHandle(pub AlignmentFeature);

/// Rhai-visible wrapper for a panel retention mechanism.
#[derive(Clone)]
pub struct RetentionHandle(pub RetentionMechanism);

/// Rhai-visible wrapper for a joint delta (addition + void for one part of a joint).
#[derive(Clone)]
pub struct JointDeltaHandle(pub JointDelta);

fn register_print_functions(engine: &mut Engine) {
    engine.register_type::<PlaneHandle>();
    engine.register_type::<AlignmentHandle>();

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Split plane constructors ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

    engine.register_fn("split_x", |pos: f64| PlaneHandle(SplitPlane::X(pos as f32)));
    engine.register_fn("split_y", |pos: f64| PlaneHandle(SplitPlane::Y(pos as f32)));
    engine.register_fn("split_z", |pos: f64| PlaneHandle(SplitPlane::Z(pos as f32)));
    engine.register_fn("split_plane",
        |nx: f64, ny: f64, nz: f64, dist: f64| {
            PlaneHandle(SplitPlane::Arbitrary {
                normal: glam::Vec3::new(nx as f32, ny as f32, nz as f32),
                distance: dist as f32,
            })
        });

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Alignment feature constructors ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

    engine.register_fn("pins_and_sockets",
        |radius: f64, height: f64, count: i64, pattern_r: f64| {
            AlignmentHandle(AlignmentFeature::PinsAndSockets {
                pin_radius:       radius as f32,
                pin_height:       height as f32,
                socket_clearance: 0.15,
                count:            count as usize,
                pattern_radius:   pattern_r as f32,
            })
        });

    engine.register_fn("tongue_and_groove",
        |width: f64, height: f64| {
            AlignmentHandle(AlignmentFeature::TongueAndGroove {
                tongue_width: width as f32,
                tongue_height: height as f32,
                groove_clearance: 0.15,
            })
        });

    engine.register_fn("dovetail",
        |width: f64, height: f64, angle_deg: f64| {
            AlignmentHandle(AlignmentFeature::Dovetail {
                width:     width as f32,
                height:    height as f32,
                angle_deg: angle_deg as f32,
                clearance: 0.15,
            })
        });

    engine.register_fn("bolt_holes",
        |bolt_r: f64, boss_r: f64, count: i64, pattern_r: f64| {
            AlignmentHandle(AlignmentFeature::BoltHoles {
                bolt_radius:    bolt_r as f32,
                boss_radius:    boss_r as f32,
                boss_height:    3.0,
                count:          count as usize,
                pattern_radius: pattern_r as f32,
                countersink:    false,
            })
        });

    engine.register_fn("bolt_holes_countersunk",
        |bolt_r: f64, boss_r: f64, count: i64, pattern_r: f64| {
            AlignmentHandle(AlignmentFeature::BoltHoles {
                bolt_radius:    bolt_r as f32,
                boss_radius:    boss_r as f32,
                boss_height:    3.0,
                count:          count as usize,
                pattern_radius: pattern_r as f32,
                countersink:    true,
            })
        });

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Split operations ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

    // split(body, plane, alignment) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ [part_a, part_b]
    engine.register_fn("split",
        |body: SdfHandle, plane: PlaneHandle, alignment: AlignmentHandle| -> rhai::Array {
            let result = split_body(body.0, &plane.0, &alignment.0);
            vec![
                rhai::Dynamic::from(SdfHandle(result.part_a)),
                rhai::Dynamic::from(SdfHandle(result.part_b)),
            ]
        });

    // Convenience: split_x/y/z with no alignment ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ [top, bottom]
    engine.register_fn("split_body_x",
        |body: SdfHandle, pos: f64| -> rhai::Array {
            let result = split_body(body.0, &SplitPlane::X(pos as f32), &AlignmentFeature::None);
            vec![rhai::Dynamic::from(SdfHandle(result.part_a)), rhai::Dynamic::from(SdfHandle(result.part_b))]
        });
    engine.register_fn("split_body_y",
        |body: SdfHandle, pos: f64| -> rhai::Array {
            let result = split_body(body.0, &SplitPlane::Y(pos as f32), &AlignmentFeature::None);
            vec![rhai::Dynamic::from(SdfHandle(result.part_a)), rhai::Dynamic::from(SdfHandle(result.part_b))]
        });
    engine.register_fn("split_body_z",
        |body: SdfHandle, pos: f64| -> rhai::Array {
            let result = split_body(body.0, &SplitPlane::Z(pos as f32), &AlignmentFeature::None);
            vec![rhai::Dynamic::from(SdfHandle(result.part_a)), rhai::Dynamic::from(SdfHandle(result.part_b))]
        });

    // split_multi(body, planes_array, alignments_array) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ array of parts
    engine.register_fn("split_multi",
        |body: SdfHandle, planes: rhai::Array, alignments: rhai::Array| -> rhai::Array {
            let pairs: Vec<(SplitPlane, AlignmentFeature)> = planes.iter().zip(alignments.iter())
                .filter_map(|(p, a)| {
                    let ph = p.clone().try_cast::<PlaneHandle>()?;
                    let ah = a.clone().try_cast::<AlignmentHandle>()?;
                    Some((ph.0, ah.0))
                })
                .collect();
            split_body_multi(body.0, &pairs)
                .into_iter()
                .map(|sdf| rhai::Dynamic::from(SdfHandle(sdf)))
                .collect()
        });

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Aerospace convenience wrappers ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

    // split_fuselage(fuse, positions_array) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ array of parts with pin-and-socket
    engine.register_fn("split_fuselage",
        |fuse: SdfHandle, positions: rhai::Array| -> rhai::Array {
            let zs: Vec<f32> = positions.iter()
                .filter_map(|v| v.as_float().ok().map(|f| f as f32))
                .collect();
            let pairs: Vec<(SplitPlane, AlignmentFeature)> = zs.iter().map(|&z| {
                (
                    SplitPlane::Z(z),
                    AlignmentFeature::PinsAndSockets {
                        pin_radius: 2.0,
                        pin_height: 4.0,
                        socket_clearance: 0.15,
                        count: 3,
                        pattern_radius: 8.0,
                    },
                )
            }).collect();
            split_body_multi(fuse.0, &pairs)
                .into_iter()
                .map(|sdf| rhai::Dynamic::from(SdfHandle(sdf)))
                .collect()
        });

    // split_wing(wing, span_fraction) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ [root_half, tip_half] with dovetail
    engine.register_fn("split_wing",
        |wing: SdfHandle, fraction: f64| -> rhai::Array {
            // Assume wing spans Ãƒâ€šÃ‚Â±some range; use Y=0 as default, scaled by fraction.
            // The caller should provide the actual span; here we use a normalized 0-1
            // fraction mapped to Y coordinate (placeholder: split at y = fraction * 100).
            let y_pos = fraction as f32 * 100.0;
            let alignment = AlignmentFeature::Dovetail {
                width: 8.0,
                height: 4.0,
                angle_deg: 15.0,
                clearance: 0.15,
            };
            let result = split_body(wing.0, &SplitPlane::Y(y_pos), &alignment);
            vec![rhai::Dynamic::from(SdfHandle(result.part_b)), rhai::Dynamic::from(SdfHandle(result.part_a))]
        });
}

fn register_tolerance_functions(engine: &mut Engine) {
    // apply_tolerance(body) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ SdfHandle
    // Applies the default StandardFDM tolerance settings.
    // NOTE: apply only to final export geometry, not intermediate shapes used in booleans.
    engine.register_fn("apply_tolerance", |body: SdfHandle| -> SdfHandle {
        SdfHandle(Arc::new(ToleranceCompensated::new(body.0, ToleranceSettings::default())))
    });

    // apply_tolerance_custom(body, external_mm, internal_mm) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ SdfHandle
    engine.register_fn("apply_tolerance_custom",
        |body: SdfHandle, external: f64, internal: f64| -> SdfHandle {
            SdfHandle(Arc::new(ToleranceCompensated::new(body.0, ToleranceSettings {
                external_offset_mm:   external as f32,
                internal_offset_mm:   internal as f32,
                min_hole_diameter_mm: 3.0,
                small_hole_bonus_mm:  0.05,
            })))
        });
}

// ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Fastener functions ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

fn register_fastener_functions(engine: &mut Engine) {
    use glam::Vec3;

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ screw_hole(body, designation, hole_type, depth, x,y,z, dx,dy,dz) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ SdfHandle
    engine.register_fn("screw_hole",
        |body: SdfHandle, designation: &str, hole_type: &str, depth: f64,
         x: f64, y: f64, z: f64, dx: f64, dy: f64, dz: f64|
        -> Result<SdfHandle, Box<rhai::EvalAltResult>>
    {
        let spec = get_spec(designation)
            .ok_or_else(|| -> Box<rhai::EvalAltResult> {
                format!("Unknown screw designation: {}", designation).into()
            })?;
        let pos = Vec3::new(x as f32, y as f32, z as f32);
        let dir = Vec3::new(dx as f32, dy as f32, dz as f32).normalize();

        let (void, boss) = match hole_type {
            "clearance"   => clearance_hole(spec, depth as f32, dir, pos),
            "countersink" => countersink_hole(spec, depth as f32, dir, pos),
            "heat_set"    => heat_set_boss(spec, dir, pos),
            other => return Err(format!("Unknown hole_type: {}  (use clearance/countersink/heat_set)", other).into()),
        };
        Ok(SdfHandle(check_and_pad(body.0, void, boss, pos, dir, spec)))
    });

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ screw_hole_pattern(body, designation, hole_type, depth, pattern, dx,dy,dz) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ SdfHandle
    // pattern is a bolt_circle / bolt_rect SDF used as the void; boss is automatically
    // generated as an expansion of the pattern by the difference between boss_r and hole_r.
    engine.register_fn("screw_hole_pattern",
        |body: SdfHandle, designation: &str, hole_type: &str, _depth: f64,
         pattern: SdfHandle, _dx: f64, _dy: f64, _dz: f64|
        -> Result<SdfHandle, Box<rhai::EvalAltResult>>
    {
        use crate::sdf::transforms::Offset;
        let spec = get_spec(designation)
            .ok_or_else(|| -> Box<rhai::EvalAltResult> {
                format!("Unknown screw designation: {}", designation).into()
            })?;
        let boss_margin = match hole_type {
            "clearance"   => spec.min_boss_diameter_mm / 2.0 - spec.clearance_diameter_mm / 2.0,
            "countersink" => spec.min_boss_diameter_mm / 2.0 - spec.clearance_diameter_mm / 2.0,
            "heat_set"    => spec.min_boss_diameter_mm / 2.0 - spec.heat_set_diameter_mm / 2.0,
            other => return Err(format!("Unknown hole_type: {}", other).into()),
        };
        let void: Arc<dyn crate::sdf::Sdf> = pattern.0;
        let boss: Arc<dyn crate::sdf::Sdf> = Arc::new(Offset::new(Arc::clone(&void), boss_margin));
        // Always apply boss (conservative ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â ensures sufficient wall material)
        let result = Arc::new(crate::sdf::booleans::Subtract::new(
            Arc::new(crate::sdf::booleans::Union::new(body.0, boss)),
            void,
        ));
        Ok(SdfHandle(result))
    });

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ screw_mate(body_a, body_b, desig, type_a, type_b, x,y,z, dx,dy,dz) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ Array
    engine.register_fn("screw_mate",
        |body_a: SdfHandle, body_b: SdfHandle,
         designation: &str, hole_type_a: &str, hole_type_b: &str,
         x: f64, y: f64, z: f64, dx: f64, dy: f64, dz: f64|
        -> Result<rhai::Array, Box<rhai::EvalAltResult>>
    {
        let spec = get_spec(designation)
            .ok_or_else(|| -> Box<rhai::EvalAltResult> {
                format!("Unknown screw designation: {}", designation).into()
            })?;
        let pos  = Vec3::new(x as f32, y as f32, z as f32);
        let dir  = Vec3::new(dx as f32, dy as f32, dz as f32).normalize();
        let through_depth = 20.0_f32;

        let (void_a, boss_a) = match hole_type_a {
            "clearance"   => clearance_hole(spec, through_depth, dir, pos),
            "countersink" => countersink_hole(spec, through_depth, dir, pos),
            "heat_set"    => heat_set_boss(spec, dir, pos),
            other => return Err(format!("Unknown hole_type_a: {}", other).into()),
        };
        let (void_b, boss_b) = match hole_type_b {
            "clearance"   => clearance_hole(spec, through_depth, -dir, pos),
            "countersink" => countersink_hole(spec, through_depth, -dir, pos),
            "heat_set"    => heat_set_boss(spec, -dir, pos),
            other => return Err(format!("Unknown hole_type_b: {}", other).into()),
        };

        let modified_a = check_and_pad(body_a.0, void_a, boss_a, pos,  dir, spec);
        let modified_b = check_and_pad(body_b.0, void_b, boss_b, pos, -dir, spec);

        Ok(vec![
            rhai::Dynamic::from(SdfHandle(modified_a)),
            rhai::Dynamic::from(SdfHandle(modified_b)),
        ])
    });

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ screw_mate_pattern ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â pattern version of screw_mate
    engine.register_fn("screw_mate_pattern",
        |body_a: SdfHandle, body_b: SdfHandle,
         designation: &str, _hole_type_a: &str, _hole_type_b: &str,
         pattern: SdfHandle, _dx: f64, _dy: f64, _dz: f64|
        -> Result<rhai::Array, Box<rhai::EvalAltResult>>
    {
        use crate::sdf::transforms::Offset;
        let spec = get_spec(designation)
            .ok_or_else(|| -> Box<rhai::EvalAltResult> {
                format!("Unknown screw designation: {}", designation).into()
            })?;
        let boss_margin = spec.min_boss_diameter_mm / 2.0 - spec.clearance_diameter_mm / 2.0;
        let void_sdf: Arc<dyn crate::sdf::Sdf> = pattern.0;
        let boss: Arc<dyn crate::sdf::Sdf> = Arc::new(Offset::new(Arc::clone(&void_sdf), boss_margin));

        let result_a = Arc::new(crate::sdf::booleans::Subtract::new(
            Arc::new(crate::sdf::booleans::Union::new(body_a.0, Arc::clone(&boss))),
            Arc::clone(&void_sdf),
        ));
        let result_b = Arc::new(crate::sdf::booleans::Subtract::new(
            Arc::new(crate::sdf::booleans::Union::new(body_b.0, boss)),
            void_sdf,
        ));
        Ok(vec![
            rhai::Dynamic::from(SdfHandle(result_a)),
            rhai::Dynamic::from(SdfHandle(result_b)),
        ])
    });
}

// ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Panel functions ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

fn register_panel_functions(engine: &mut Engine) {
    use glam::Vec3;

    engine.register_type::<RetentionHandle>();

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Retention constructors ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

    // snap_retention(count, clip_width, engagement) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ RetentionHandle
    engine.register_fn("snap_retention",
        |count: i64, clip_width: f64, engagement: f64| -> RetentionHandle {
            RetentionHandle(RetentionMechanism::SnapFit {
                clip_count:     count.max(1) as usize,
                clip_width:     clip_width as f32,
                clip_thickness: 0.8,
                engagement_mm:  engagement as f32,
            })
        });

    // screw_retention(designation, count, tab_thickness) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ RetentionHandle
    engine.register_fn("screw_retention",
        |designation: &str, count: i64, tab_thickness: f64| -> RetentionHandle {
            RetentionHandle(RetentionMechanism::ScrewTabs {
                screw_designation: designation.to_string(),
                tab_count:         count.max(1) as usize,
                tab_thickness:     tab_thickness as f32,
            })
        });

    // friction_retention(interference_mm) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ RetentionHandle
    engine.register_fn("friction_retention",
        |interference: f64| -> RetentionHandle {
            RetentionHandle(RetentionMechanism::FrictionFit {
                interference_mm: interference as f32,
            })
        });

    // hinge_retention(thickness, width) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ RetentionHandle
    engine.register_fn("hinge_retention",
        |thickness: f64, width: f64| -> RetentionHandle {
            RetentionHandle(RetentionMechanism::LivingHinge {
                hinge_thickness: thickness as f32,
                hinge_width:     width as f32,
            })
        });

    // magnet_retention(magnet_diameter, magnet_depth, count) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ RetentionHandle
    engine.register_fn("magnet_retention",
        |magnet_diameter: f64, magnet_depth: f64, count: i64| -> RetentionHandle {
            RetentionHandle(RetentionMechanism::MagnetBoss {
                magnet_diameter: magnet_diameter as f32,
                magnet_depth:    magnet_depth as f32,
                count:           count.max(1) as usize,
            })
        });

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ access_panel(parent, x,y,z, w,h,t, nx,ny,nz, retention) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ [parent, panel]
    engine.register_fn("access_panel",
        |parent: SdfHandle, x: f64, y: f64, z: f64,
         width: f64, height: f64, thickness: f64,
         nx: f64, ny: f64, nz: f64,
         retention: RetentionHandle|
        -> rhai::Array
    {
        let normal = Vec3::new(nx as f32, ny as f32, nz as f32);
        let ap = panel_rect(
            x as f32, y as f32, z as f32,
            width as f32, height as f32, thickness as f32,
            normal,
            retention.0,
        );
        // Apply to parent:
        //   1. Subtract cutout + parent_void
        //   2. Add parent_add
        let cut: Arc<dyn crate::sdf::Sdf> = Arc::new(
            crate::sdf::booleans::Union::new(ap.cutout, ap.parent_void)
        );
        let modified_parent: Arc<dyn crate::sdf::Sdf> = Arc::new(
            crate::sdf::booleans::Union::new(
                Arc::new(crate::sdf::booleans::Subtract::new(parent.0, cut)),
                ap.parent_add,
            )
        );
        vec![
            rhai::Dynamic::from(SdfHandle(modified_parent)),
            rhai::Dynamic::from(SdfHandle(ap.panel)),
        ]
    });

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Drone convenience wrappers ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

    // battery_hatch(fuselage, station_position, width, height, retention) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ [fuselage, hatch]
    engine.register_fn("battery_hatch",
        |fuselage: SdfHandle, station_x: f64, width: f64, height: f64,
         retention: RetentionHandle|
        -> rhai::Array
    {
        let ap = battery_hatch(station_x as f32, width as f32, height as f32, retention.0);
        let cut: Arc<dyn crate::sdf::Sdf> = Arc::new(
            crate::sdf::booleans::Union::new(ap.cutout, ap.parent_void)
        );
        let modified: Arc<dyn crate::sdf::Sdf> = Arc::new(
            crate::sdf::booleans::Union::new(
                Arc::new(crate::sdf::booleans::Subtract::new(fuselage.0, cut)),
                ap.parent_add,
            )
        );
        vec![
            rhai::Dynamic::from(SdfHandle(modified)),
            rhai::Dynamic::from(SdfHandle(ap.panel)),
        ]
    });

    // fc_access_panel(fuselage, station_position, retention) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ [fuselage, panel]
    engine.register_fn("fc_access_panel",
        |fuselage: SdfHandle, station_x: f64, retention: RetentionHandle| -> rhai::Array
    {
        let ap = fc_access_panel(station_x as f32, retention.0);
        let cut: Arc<dyn crate::sdf::Sdf> = Arc::new(
            crate::sdf::booleans::Union::new(ap.cutout, ap.parent_void)
        );
        let modified: Arc<dyn crate::sdf::Sdf> = Arc::new(
            crate::sdf::booleans::Union::new(
                Arc::new(crate::sdf::booleans::Subtract::new(fuselage.0, cut)),
                ap.parent_add,
            )
        );
        vec![
            rhai::Dynamic::from(SdfHandle(modified)),
            rhai::Dynamic::from(SdfHandle(ap.panel)),
        ]
    });
}

// ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Joint functions ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

fn register_joint_functions(engine: &mut Engine) {
    use glam::Vec3;
    use crate::sdf::print::joints::{
        dovetail_joint, finger_joint, press_fit, snap_connector, living_hinge_strip,
    };

    engine.register_type::<JointDeltaHandle>();

    // apply_joint_delta(part, delta) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ SdfHandle
    // Applies union(subtract(part, delta.void), delta.addition)
    engine.register_fn("apply_joint_delta",
        |part: SdfHandle, delta: JointDeltaHandle| -> SdfHandle {
            SdfHandle(delta.0.apply(part.0))
        });

    // dovetail_joint(len, width, height, angle, clearance, px,py,pz, ax,ay,az) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ [delta_a, delta_b]
    engine.register_fn("dovetail_joint",
        |length: f64, width: f64, height: f64, angle_deg: f64, clearance: f64,
         px: f64, py: f64, pz: f64, ax: f64, ay: f64, az: f64|
        -> rhai::Array
    {
        let pos  = Vec3::new(px as f32, py as f32, pz as f32);
        let axis = Vec3::new(ax as f32, ay as f32, az as f32).normalize();
        let (da, db) = dovetail_joint(
            length as f32, width as f32, height as f32,
            angle_deg as f32, clearance as f32, pos, axis,
        );
        vec![
            rhai::Dynamic::from(JointDeltaHandle(da)),
            rhai::Dynamic::from(JointDeltaHandle(db)),
        ]
    });

    // finger_joint(len, finger_width, height, count, clearance, px,py,pz, ax,ay,az) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ [da, db]
    engine.register_fn("finger_joint",
        |length: f64, finger_width: f64, height: f64, count: i64, clearance: f64,
         px: f64, py: f64, pz: f64, ax: f64, ay: f64, az: f64|
        -> rhai::Array
    {
        let pos  = Vec3::new(px as f32, py as f32, pz as f32);
        let axis = Vec3::new(ax as f32, ay as f32, az as f32).normalize();
        let (da, db) = finger_joint(
            length as f32, finger_width as f32, height as f32,
            count.max(1) as usize, clearance as f32, pos, axis,
        );
        vec![
            rhai::Dynamic::from(JointDeltaHandle(da)),
            rhai::Dynamic::from(JointDeltaHandle(db)),
        ]
    });

    // press_fit(pin_radius, pin_length, interference, px,py,pz, dx,dy,dz) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ [da, db]
    engine.register_fn("press_fit",
        |pin_radius: f64, pin_length: f64, interference: f64,
         px: f64, py: f64, pz: f64, dx: f64, dy: f64, dz: f64|
        -> rhai::Array
    {
        let pos = Vec3::new(px as f32, py as f32, pz as f32);
        let dir = Vec3::new(dx as f32, dy as f32, dz as f32).normalize();
        let (da, db) = press_fit(
            pin_radius as f32, pin_length as f32, pin_length as f32,
            interference as f32, pos, dir,
        );
        vec![
            rhai::Dynamic::from(JointDeltaHandle(da)),
            rhai::Dynamic::from(JointDeltaHandle(db)),
        ]
    });

    // snap_connector(width, height, engagement, px,py,pz, dx,dy,dz) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ [da, db]
    engine.register_fn("snap_connector",
        |width: f64, height: f64, engagement: f64,
         px: f64, py: f64, pz: f64, dx: f64, dy: f64, dz: f64|
        -> rhai::Array
    {
        let pos = Vec3::new(px as f32, py as f32, pz as f32);
        let dir = Vec3::new(dx as f32, dy as f32, dz as f32).normalize();
        let (da, db) = snap_connector(
            width as f32, height as f32, engagement as f32, 0.15, pos, dir,
        );
        vec![
            rhai::Dynamic::from(JointDeltaHandle(da)),
            rhai::Dynamic::from(JointDeltaHandle(db)),
        ]
    });

    // living_hinge_strip(width, thickness, length, px,py,pz, ax,ay,az) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ [da, db]
    engine.register_fn("living_hinge_strip",
        |width: f64, thickness: f64, length: f64,
         px: f64, py: f64, pz: f64, ax: f64, ay: f64, az: f64|
        -> rhai::Array
    {
        let pos  = Vec3::new(px as f32, py as f32, pz as f32);
        let axis = Vec3::new(ax as f32, ay as f32, az as f32).normalize();
        let (da, db) = living_hinge_strip(
            width as f32, thickness as f32, length as f32, pos, axis,
        );
        vec![
            rhai::Dynamic::from(JointDeltaHandle(da)),
            rhai::Dynamic::from(JointDeltaHandle(db)),
        ]
    });
}

// ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Part 7: Layup library functions ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬
// Allows library files to define and export named layup configurations
// without binding them to a specific geometry at definition time.

fn register_layup_library_functions(engine: &mut Engine) {
    use crate::sdf::aerospace::composite::{CompositeSdf, CompositeLayup, ShellLayer};

    // composite_layup_config(layers) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ LayupConfigHandle
    // Stores a layup definition for later application. Library components use this
    // to export named layup presets (e.g. fn drone_carbon() { composite_layup_config([...]) }).
    engine.register_fn("composite_layup_config",
        |layers: rhai::Array| -> Result<LayupConfigHandle, Box<rhai::EvalAltResult>> {
        let mut shell_layers: Vec<Arc<ShellLayer>> = Vec::with_capacity(layers.len());
        for item in layers {
            let lh: LayerHandle = item.try_cast::<LayerHandle>()
                .ok_or_else(|| -> Box<rhai::EvalAltResult> {
                    "composite_layup_config: array must contain LayerHandles".into()
                })?;
            shell_layers.push(Arc::clone(&lh.0));
        }
        Ok(LayupConfigHandle(shell_layers))
    });

    // apply_layup(body, layup) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ SdfHandle
    // Applies a stored layup configuration to a body SDF.
    engine.register_fn("apply_layup",
        |body: SdfHandle, layup: LayupConfigHandle| -> SdfHandle {
        let layers: Vec<ShellLayer> = layup.0.iter()
            .map(|arc| (**arc).clone())
            .collect();
        let composite_layup = Arc::new(CompositeLayup::new(body.0, layers));
        SdfHandle(Arc::new(CompositeSdf::from_arc(composite_layup)))
    });
}

fn register_control_surface_functions(engine: &mut Engine) {
    use crate::sdf::aerospace::control_surfaces::{
        HingeSpec, LinkageSpec, ControlHornSpec, PushrodSlotSpec,
        aileron as cs_aileron, elevator as cs_elevator, rudder as cs_rudder,
        flap as cs_flap, elevon as cs_elevon, wing_with_ailerons as cs_wing_with_ailerons,
    };

    // Hinge constructors
    engine.register_fn("simple_gap_hinge", |gap_width: f64| -> HingeHandle {
        HingeHandle(HingeSpec::simple_gap(gap_width as f32))
    });
    engine.register_fn("rounded_hinge", |radius: f64, gap_width: f64| -> HingeHandle {
        HingeHandle(HingeSpec::rounded(radius as f32, gap_width as f32))
    });

    // Linkage constructors
    engine.register_fn("no_linkage", || -> LinkageHandle {
        LinkageHandle(LinkageSpec::none())
    });
    engine.register_fn("control_horn", |height: f64, hole_offset: f64, span_fraction: f64| -> LinkageHandle {
        LinkageHandle(LinkageSpec::horn(ControlHornSpec::default_upper(height as f32, hole_offset as f32, span_fraction as f32)))
    });
    engine.register_fn("control_horn_lower", |height: f64, hole_offset: f64, span_fraction: f64| -> LinkageHandle {
        LinkageHandle(LinkageSpec::horn(ControlHornSpec::default_lower(height as f32, hole_offset as f32, span_fraction as f32)))
    });
    engine.register_fn("pushrod_slot",
        |width: f64, length: f64, span_fraction: f64, dx: f64, dy: f64, dz: f64| -> LinkageHandle {
        LinkageHandle(LinkageSpec {
            control_horn: None,
            pushrod_slot: Some(PushrodSlotSpec {
                slot_width: width as f32,
                slot_length: length as f32,
                position_span_fraction: span_fraction as f32,
                exit_direction: Vec3::new(dx as f32, dy as f32, dz as f32).normalize_or_zero(),
            }),
        })
    });
    engine.register_fn("horn_and_slot",
        |horn_height: f64, hole_offset: f64, slot_span_fraction: f64, dx: f64, dy: f64, dz: f64| -> LinkageHandle {
        LinkageHandle(LinkageSpec {
            control_horn: Some(ControlHornSpec::default_lower(horn_height as f32, hole_offset as f32, slot_span_fraction as f32)),
            pushrod_slot: Some(PushrodSlotSpec {
                slot_width: 3.0,
                slot_length: 15.0,
                position_span_fraction: slot_span_fraction as f32,
                exit_direction: Vec3::new(dx as f32, dy as f32, dz as f32).normalize_or_zero(),
            }),
        })
    });

    // aileron(wing, span_start, span_end, chord_fraction, hinge, linkage) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ [cs_sdf, modified_parent]
    engine.register_fn("aileron",
        |wing: SdfHandle, span_start: f64, span_end: f64, chord_fraction: f64,
         hinge: HingeHandle, linkage: LinkageHandle| -> rhai::Array {
        let result = cs_aileron(wing.0, span_start as f32, span_end as f32, chord_fraction as f32, hinge.0, linkage.0);
        vec![
            rhai::Dynamic::from(SdfHandle(result.control_surface)),
            rhai::Dynamic::from(SdfHandle(result.modified_parent)),
        ]
    });

    // elevator(stab, chord_fraction, hinge, linkage) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ [cs_sdf, modified_parent]
    engine.register_fn("elevator",
        |stab: SdfHandle, chord_fraction: f64, hinge: HingeHandle, linkage: LinkageHandle| -> rhai::Array {
        let result = cs_elevator(stab.0, chord_fraction as f32, hinge.0, linkage.0);
        vec![
            rhai::Dynamic::from(SdfHandle(result.control_surface)),
            rhai::Dynamic::from(SdfHandle(result.modified_parent)),
        ]
    });

    // rudder(fin, chord_fraction, hinge, linkage) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ [cs_sdf, modified_parent]
    engine.register_fn("rudder",
        |fin: SdfHandle, chord_fraction: f64, hinge: HingeHandle, linkage: LinkageHandle| -> rhai::Array {
        let result = cs_rudder(fin.0, chord_fraction as f32, hinge.0, linkage.0);
        vec![
            rhai::Dynamic::from(SdfHandle(result.control_surface)),
            rhai::Dynamic::from(SdfHandle(result.modified_parent)),
        ]
    });

    // flap(wing, span_start, span_end, chord_fraction, hinge) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ [cs_sdf, modified_parent]
    engine.register_fn("flap",
        |wing: SdfHandle, span_start: f64, span_end: f64, chord_fraction: f64, hinge: HingeHandle| -> rhai::Array {
        let result = cs_flap(wing.0, span_start as f32, span_end as f32, chord_fraction as f32, hinge.0);
        vec![
            rhai::Dynamic::from(SdfHandle(result.control_surface)),
            rhai::Dynamic::from(SdfHandle(result.modified_parent)),
        ]
    });

    // elevon(wing, span_start, span_end, chord_fraction, hinge, linkage) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ [cs_sdf, modified_parent]
    engine.register_fn("elevon",
        |wing: SdfHandle, span_start: f64, span_end: f64, chord_fraction: f64,
         hinge: HingeHandle, linkage: LinkageHandle| -> rhai::Array {
        let result = cs_elevon(wing.0, span_start as f32, span_end as f32, chord_fraction as f32, hinge.0, linkage.0);
        vec![
            rhai::Dynamic::from(SdfHandle(result.control_surface)),
            rhai::Dynamic::from(SdfHandle(result.modified_parent)),
        ]
    });

    // wing_with_ailerons(wing, span_start_frac, span_end_frac, chord_fraction) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ [pos_ail, neg_ail, modified_wing]
    engine.register_fn("wing_with_ailerons",
        |wing: SdfHandle, span_start: f64, span_end: f64, chord_fraction: f64| -> rhai::Array {
        let hinge = HingeSpec::rounded(1.5, 0.5);
        let linkage = LinkageSpec::horn(ControlHornSpec::default_lower(15.0, 10.0, 0.5));
        let (pos, neg, modified) = cs_wing_with_ailerons(wing.0, span_start as f32, span_end as f32, chord_fraction as f32, hinge, linkage);
        vec![
            rhai::Dynamic::from(SdfHandle(pos)),
            rhai::Dynamic::from(SdfHandle(neg)),
            rhai::Dynamic::from(SdfHandle(modified)),
        ]
    });

    // wing_with_ailerons_custom(wing, span_start, span_end, chord_fraction, hinge, linkage) ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ [pos, neg, modified]
    engine.register_fn("wing_with_ailerons_custom",
        |wing: SdfHandle, span_start: f64, span_end: f64, chord_fraction: f64,
         hinge: HingeHandle, linkage: LinkageHandle| -> rhai::Array {
        let (pos, neg, modified) = cs_wing_with_ailerons(wing.0, span_start as f32, span_end as f32, chord_fraction as f32, hinge.0, linkage.0);
        vec![
            rhai::Dynamic::from(SdfHandle(pos)),
            rhai::Dynamic::from(SdfHandle(neg)),
            rhai::Dynamic::from(SdfHandle(modified)),
        ]
    });
}

// ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ PointHandle arithmetic and geometric utilities ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

fn register_point_functions(engine: &mut Engine) {
    use crate::sdf::transforms::Translate;

    // Constructor
    engine.register_fn("point", |x: f64, y: f64, z: f64| -> PointHandle {
        PointHandle(glam::Vec3::new(x as f32, y as f32, z as f32))
    });

    // Arithmetic
    engine.register_fn("+", |a: PointHandle, b: PointHandle| -> PointHandle {
        PointHandle(a.0 + b.0)
    });
    engine.register_fn("-", |a: PointHandle, b: PointHandle| -> PointHandle {
        PointHandle(a.0 - b.0)
    });

    // Property getters
    engine.register_get("x", |p: &mut PointHandle| -> f64 { p.0.x as f64 });
    engine.register_get("y", |p: &mut PointHandle| -> f64 { p.0.y as f64 });
    engine.register_get("z", |p: &mut PointHandle| -> f64 { p.0.z as f64 });

    // Distance between two points
    engine.register_fn("dist", |a: PointHandle, b: PointHandle| -> f64 {
        (a.0 - b.0).length() as f64
    });

    // Midpoint
    engine.register_fn("midpoint", |a: PointHandle, b: PointHandle| -> PointHandle {
        PointHandle((a.0 + b.0) * 0.5)
    });

    // Lerp
    engine.register_fn("lerp_point", |a: PointHandle, b: PointHandle, t: f64| -> PointHandle {
        PointHandle(a.0 + (b.0 - a.0) * t as f32)
    });

    // Direction (normalized)
    engine.register_fn("direction", |from: PointHandle, to: PointHandle| -> PointHandle {
        PointHandle((to.0 - from.0).normalize_or_zero())
    });

    // To array [x, y, z]
    engine.register_fn("to_array", |p: PointHandle| -> rhai::Array {
        vec![
            rhai::Dynamic::from(p.0.x as f64),
            rhai::Dynamic::from(p.0.y as f64),
            rhai::Dynamic::from(p.0.z as f64),
        ]
    });

    // Offset by scalar components
    engine.register_fn("offset_point", |p: PointHandle, dx: f64, dy: f64, dz: f64| -> PointHandle {
        PointHandle(p.0 + glam::Vec3::new(dx as f32, dy as f32, dz as f32))
    });

    // Offset along a direction by a scalar distance
    engine.register_fn("offset_along", |p: PointHandle, dir: PointHandle, dist: f64| -> PointHandle {
        PointHandle(p.0 + dir.0.normalize_or_zero() * dist as f32)
    });

    // Offset from `from` toward `toward` by `dist`
    engine.register_fn("offset_toward", |from: PointHandle, toward: PointHandle, dist: f64| -> PointHandle {
        let d = (toward.0 - from.0).normalize_or_zero();
        PointHandle(from.0 + d * dist as f32)
    });

    // Project point onto a plane defined by normal and a point on the plane
    engine.register_fn("project_to_plane",
        |p: PointHandle, normal: PointHandle, plane_pt: PointHandle| -> PointHandle {
            let n = normal.0.normalize_or_zero();
            let v = p.0 - plane_pt.0;
            let projected = p.0 - n * v.dot(n);
            PointHandle(projected)
        }
    );

    // translate_p: translate SDF body to a PointHandle position
    engine.register_fn("translate_p", |body: SdfHandle, pos: PointHandle| -> SdfHandle {
        SdfHandle(std::sync::Arc::new(Translate::new(body.0, pos.0)))
    });

    // place_p: translate ComponentHandle to a PointHandle position
    engine.register_fn("place_p", |comp: ComponentHandle, pos: PointHandle| -> ComponentHandle {
        ComponentHandle {
            geometry: std::sync::Arc::new(Translate::new(comp.geometry, pos.0)),
            keepout:  std::sync::Arc::new(Translate::new(comp.keepout,  pos.0)),
            mass_g:   comp.mass_g,
            name:     comp.name,
        }
    });
}

// ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Geometric query functions (need ref_collector closure capture) ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

pub fn register_query_functions(engine: &mut Engine, ref_collector: RefPointCollector) {
    use crate::sdf::query;
    use crate::sdf::aerospace::control_surfaces::estimate_half_span;

    // surface_point(sdf, ox, oy, oz, dx, dy, dz) -> PointHandle
    engine.register_fn("surface_point",
        |sdf: SdfHandle, ox: f64, oy: f64, oz: f64, dx: f64, dy: f64, dz: f64| -> PointHandle {
            let origin = glam::Vec3::new(ox as f32, oy as f32, oz as f32);
            let dir    = glam::Vec3::new(dx as f32, dy as f32, dz as f32);
            match query::surface_point(sdf.0.as_ref(), origin, dir, 500.0) {
                Some(p) => PointHandle(p),
                None    => PointHandle(origin),
            }
        }
    );

    // surface_point_p(sdf, origin: PointHandle, dir: PointHandle) -> PointHandle
    engine.register_fn("surface_point_p",
        |sdf: SdfHandle, origin: PointHandle, dir: PointHandle| -> PointHandle {
            match query::surface_point(sdf.0.as_ref(), origin.0, dir.0, 500.0) {
                Some(p) => PointHandle(p),
                None    => PointHandle(origin.0),
            }
        }
    );

    // closest_point(sdf, x, y, z) -> PointHandle
    engine.register_fn("closest_point",
        |sdf: SdfHandle, x: f64, y: f64, z: f64| -> PointHandle {
            PointHandle(query::closest_point(sdf.0.as_ref(),
                glam::Vec3::new(x as f32, y as f32, z as f32)))
        }
    );

    // closest_point_p(sdf, query: PointHandle) -> PointHandle
    engine.register_fn("closest_point_p",
        |sdf: SdfHandle, q: PointHandle| -> PointHandle {
            PointHandle(query::closest_point(sdf.0.as_ref(), q.0))
        }
    );

    // sdf_distance_p(sdf, query: PointHandle) -> f64
    engine.register_fn("sdf_distance_p",
        |sdf: SdfHandle, q: PointHandle| -> f64 {
            sdf.0.distance(q.0) as f64
        }
    );

    // sdf_distance(sdf, x, y, z) -> f64
    engine.register_fn("sdf_distance",
        |sdf: SdfHandle, x: f64, y: f64, z: f64| -> f64 {
            sdf.0.distance(glam::Vec3::new(x as f32, y as f32, z as f32)) as f64
        }
    );

    // furthest_point(sdf, dx, dy, dz) -> PointHandle
    engine.register_fn("furthest_point",
        |sdf: SdfHandle, dx: f64, dy: f64, dz: f64| -> PointHandle {
            PointHandle(query::furthest_point(sdf.0.as_ref(),
                glam::Vec3::new(dx as f32, dy as f32, dz as f32)))
        }
    );

    // centroid(sdf) -> PointHandle  (auto-computes bounding box first)
    engine.register_fn("centroid", |sdf: SdfHandle| -> PointHandle {
        let bbox = query::bounding_points(sdf.0.as_ref());
        PointHandle(query::centroid_point(sdf.0.as_ref(), bbox.min, bbox.max))
    });

    // bbox_min / max / center / size
    engine.register_fn("bbox_min", |sdf: SdfHandle| -> PointHandle {
        PointHandle(query::bounding_points(sdf.0.as_ref()).min)
    });
    engine.register_fn("bbox_max", |sdf: SdfHandle| -> PointHandle {
        PointHandle(query::bounding_points(sdf.0.as_ref()).max)
    });
    engine.register_fn("bbox_center", |sdf: SdfHandle| -> PointHandle {
        PointHandle(query::bounding_points(sdf.0.as_ref()).center)
    });
    engine.register_fn("bbox_size", |sdf: SdfHandle| -> PointHandle {
        PointHandle(query::bounding_points(sdf.0.as_ref()).size)
    });

    // cross_section_center(sdf, axis: i64, pos: f64) -> PointHandle
    engine.register_fn("cross_section_center",
        |sdf: SdfHandle, axis: i64, pos: f64| -> PointHandle {
            PointHandle(query::cross_section_centroid(sdf.0.as_ref(), axis as usize, pos as f32))
        }
    );

    // ref_point(name, p) -> PointHandle ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â stores in ref_collector, returns p unchanged
    {
        let rc = std::sync::Arc::clone(&ref_collector);
        engine.register_fn("ref_point", move |name: &str, p: PointHandle| -> PointHandle {
            let mut coll = rc.lock().unwrap();
            let color_idx = coll.len() % REF_COLORS.len();
            coll.push(ReferencePoint {
                name:     name.to_string(),
                position: p.0,
                color:    REF_COLORS[color_idx],
            });
            p
        });
    }

    // get_ref(name) -> PointHandle ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â looks up in ref_collector, returns ZERO if not found
    {
        let rc = std::sync::Arc::clone(&ref_collector);
        engine.register_fn("get_ref", move |name: &str| -> PointHandle {
            let coll = rc.lock().unwrap();
            match coll.iter().find(|rp| rp.name == name) {
                Some(rp) => PointHandle(rp.position),
                None     => PointHandle(glam::Vec3::ZERO),
            }
        });
    }

    // ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Wing-specific queries ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

    // leading_edge(wing, span_fraction) -> PointHandle
    engine.register_fn("leading_edge", |wing: SdfHandle, span_frac: f64| -> PointHandle {
        let half_span = estimate_half_span(wing.0.as_ref());
        let y         = span_frac as f32 * half_span;
        let slice_ctr = query::cross_section_centroid(wing.0.as_ref(), 1, y);
        match query::surface_point(wing.0.as_ref(), slice_ctr, glam::Vec3::NEG_X, 500.0) {
            Some(p) => PointHandle(p),
            None    => PointHandle(query::furthest_point(wing.0.as_ref(), glam::Vec3::NEG_X)),
        }
    });

    // trailing_edge(wing, span_fraction) -> PointHandle
    engine.register_fn("trailing_edge", |wing: SdfHandle, span_frac: f64| -> PointHandle {
        let half_span = estimate_half_span(wing.0.as_ref());
        let y         = span_frac as f32 * half_span;
        let slice_ctr = query::cross_section_centroid(wing.0.as_ref(), 1, y);
        match query::surface_point(wing.0.as_ref(), slice_ctr, glam::Vec3::X, 500.0) {
            Some(p) => PointHandle(p),
            None    => PointHandle(query::furthest_point(wing.0.as_ref(), glam::Vec3::X)),
        }
    });

    // chord_point(wing, span_fraction, chord_fraction) -> PointHandle
    engine.register_fn("chord_point",
        |wing: SdfHandle, span_frac: f64, chord_frac: f64| -> PointHandle {
            let half_span = estimate_half_span(wing.0.as_ref());
            let y         = span_frac as f32 * half_span;
            let slice_ctr = query::cross_section_centroid(wing.0.as_ref(), 1, y);
            let le = match query::surface_point(wing.0.as_ref(), slice_ctr, glam::Vec3::NEG_X, 500.0) {
                Some(p) => p,
                None    => query::furthest_point(wing.0.as_ref(), glam::Vec3::NEG_X),
            };
            let te = match query::surface_point(wing.0.as_ref(), slice_ctr, glam::Vec3::X, 500.0) {
                Some(p) => p,
                None    => query::furthest_point(wing.0.as_ref(), glam::Vec3::X),
            };
            PointHandle(le + (te - le) * chord_frac as f32)
        }
    );

    // wing_tip(wing) -> PointHandle ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â furthest in +Y
    engine.register_fn("wing_tip", |wing: SdfHandle| -> PointHandle {
        PointHandle(query::furthest_point(wing.0.as_ref(), glam::Vec3::Y))
    });

    // wing_root(wing) -> PointHandle ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â cross_section_center at Y=0
    engine.register_fn("wing_root", |wing: SdfHandle| -> PointHandle {
        PointHandle(query::cross_section_centroid(wing.0.as_ref(), 1, 0.0))
    });

    // span_station(wing, span_fraction) -> PointHandle
    engine.register_fn("span_station", |wing: SdfHandle, span_frac: f64| -> PointHandle {
        let half_span = estimate_half_span(wing.0.as_ref());
        let y         = span_frac as f32 * half_span;
        PointHandle(query::cross_section_centroid(wing.0.as_ref(), 1, y))
    });

    // place_at_ref(body, ref_name, dx, dy, dz) -> SdfHandle
    // Places body at the named reference point with additional offsets
    {
        let rc = std::sync::Arc::clone(&ref_collector);
        engine.register_fn("place_at_ref",
            move |body: SdfHandle, ref_name: &str, dx: f64, dy: f64, dz: f64| -> SdfHandle {
            let points = rc.lock().unwrap();
            if let Some(rp) = points.iter().find(|r| r.name == ref_name) {
                let pos = rp.position + glam::Vec3::new(dx as f32, dy as f32, dz as f32);
                SdfHandle(Arc::new(Translate::new(body.0, pos)))
            } else {
                // Ref point not found ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â apply offset only
                SdfHandle(Arc::new(Translate::new(body.0, glam::Vec3::new(dx as f32, dy as f32, dz as f32))))
            }
        });
    }
}

pub fn register_granular_bracket_functions(engine: &mut Engine) {
    fn hidden_sdf_handle() -> SdfHandle {
        SdfHandle(Arc::new(FnSdf { func: Arc::new(|_| 1e6) }))
    }

    engine.register_fn("bracket_offset",
        |mut comp_map: rhai::Map, offset_mm: f64| -> rhai::Map {
            let offset_mm = offset_mm.clamp(-1.0, 5.0);
            let mut cfg = comp_map
                .get("bracket_config")
                .and_then(|v| v.clone().try_cast::<rhai::Map>())
                .unwrap_or_else(rhai::Map::new);
            cfg.insert(
                "pocket_offset_mm".into(),
                rhai::Dynamic::from(offset_mm)
            );
            comp_map.insert("bracket_config".into(), rhai::Dynamic::from(cfg));
            comp_map.insert("bracket_offset_mm".into(), rhai::Dynamic::from(offset_mm));
            comp_map
        }
    );

    engine.register_fn("support_density",
        |mut comp_map: rhai::Map, level: i64| -> rhai::Map {
            let level = level.clamp(1, 10);
            let mut cfg = comp_map
                .get("bracket_config")
                .and_then(|v| v.clone().try_cast::<rhai::Map>())
                .unwrap_or_else(rhai::Map::new);
            cfg.insert(
                "support_density".into(),
                rhai::Dynamic::from(level)
            );
            comp_map.insert("bracket_config".into(), rhai::Dynamic::from(cfg));
            comp_map.insert("support_density".into(), rhai::Dynamic::from(level));
            comp_map
        }
    );

    engine.register_fn("tray_clearance",
        |mut comp_map: rhai::Map, clearance_mm: f64| -> rhai::Map {
            let clearance_mm = clearance_mm.clamp(0.0, 5.0);
            let mut cfg = comp_map
                .get("bracket_config")
                .and_then(|v| v.clone().try_cast::<rhai::Map>())
                .unwrap_or_else(rhai::Map::new);
            cfg.insert("tray_clearance_mm".into(), rhai::Dynamic::from(clearance_mm));
            comp_map.insert("bracket_config".into(), rhai::Dynamic::from(cfg));
            comp_map.insert("tray_clearance_mm".into(), rhai::Dynamic::from(clearance_mm));
            comp_map
        }
    );

    engine.register_fn("tray_thickness",
        |mut comp_map: rhai::Map, thickness_mm: f64| -> rhai::Map {
            let thickness_mm = thickness_mm.clamp(0.5, 8.0);
            let mut cfg = comp_map
                .get("bracket_config")
                .and_then(|v| v.clone().try_cast::<rhai::Map>())
                .unwrap_or_else(rhai::Map::new);
            cfg.insert("tray_thickness_mm".into(), rhai::Dynamic::from(thickness_mm));
            comp_map.insert("bracket_config".into(), rhai::Dynamic::from(cfg));
            comp_map.insert("tray_thickness_mm".into(), rhai::Dynamic::from(thickness_mm));
            comp_map
        }
    );

    engine.register_fn("hide_part",
        |mut value_map: rhai::Map, key: &str| -> rhai::Map {
            if let Some(value) = value_map.get(key).cloned() {
                if value.clone().try_cast::<SdfHandle>().is_some() {
                    value_map.insert(key.into(), rhai::Dynamic::from(hidden_sdf_handle()));
                }
            }
            value_map
        }
    );

    engine.register_fn("hide_parts",
        |mut value_map: rhai::Map, keys: rhai::Array| -> rhai::Map {
            for key in keys {
                if let Some(name) = key.try_cast::<rhai::ImmutableString>() {
                    let key = name.to_string();
                    if let Some(value) = value_map.get(key.as_str()).cloned() {
                        if value.clone().try_cast::<SdfHandle>().is_some() {
                            value_map.insert(key.into(), rhai::Dynamic::from(hidden_sdf_handle()));
                        }
                    }
                }
            }
            value_map
        }
    );

    fn parse_mount_point_map(
        value: &rhai::Dynamic,
        translation: Vec3,
    ) -> Result<BracketMountPoint, Box<rhai::EvalAltResult>> {
        let map = value.clone().try_cast::<rhai::Map>()
            .ok_or_else(|| "mount_points entries must be maps".to_string())?;

        let position = map.get("position")
            .and_then(|v| v.clone().try_cast::<PointHandle>())
            .map(|p| p.0 + translation)
            .ok_or_else(|| "mount point map must contain Point field 'position'".to_string())?;
        let normal = map.get("normal")
            .and_then(|v| v.clone().try_cast::<PointHandle>())
            .map(|p| safe_normalize(p.0, 1e-8))
            .ok_or_else(|| "mount point map must contain Point field 'normal'".to_string())?;
        let tier = map.get("tier")
            .and_then(|v| v.clone().try_cast::<i64>())
            .ok_or_else(|| "mount point map must contain integer field 'tier'".to_string())?;
        if tier != 1 && tier != 2 {
            return Err("mount point tier must be 1 or 2".into());
        }
        let base_radius = map.get("base_radius")
            .and_then(|v| v.clone().try_cast::<f64>())
            .ok_or_else(|| "mount point map must contain numeric field 'base_radius'".to_string())? as f32;
        if base_radius <= 0.0 {
            return Err("mount point base_radius must be positive".into());
        }

        Ok(BracketMountPoint { position, normal, tier, base_radius })
    }

    fn parse_mount_points(
        comp_map: &rhai::Map,
        translation: Vec3,
    ) -> Result<Vec<BracketMountPoint>, Box<rhai::EvalAltResult>> {
        let arr = comp_map.get("mount_points")
            .and_then(|v| v.clone().try_cast::<rhai::Array>())
            .ok_or_else(|| "component map must contain array field 'mount_points'".to_string())?;
        let mut out = Vec::new();
        for item in arr {
            out.push(parse_mount_point_map(&item, translation)?);
        }
        if out.is_empty() {
            return Err("mount_points must contain at least one entry".into());
        }
        Ok(out)
    }

    fn expand_mount_points_for_density(
        points: &[BracketMountPoint],
        density: u32,
    ) -> Vec<BracketMountPoint> {
        if points.is_empty() {
            return Vec::new();
        }
        let mut out = points.to_vec();
        if density <= 1 {
            return out;
        }

        let eff_density = effective_support_density(density);
        let tier1_subdiv = eff_density.floor().max(1.0) as usize - 1;
        let tier2_subdiv = (eff_density.floor() - 2.0).max(0.0) as usize;

        for i in 0..points.len() {
            let a = &points[i];
            let mut neighbors: Vec<(usize, f32)> = Vec::new();
            for j in (i + 1)..points.len() {
                let a = &points[i];
                let b = &points[j];
                if a.tier != b.tier {
                    continue;
                }
                if a.normal.dot(b.normal) < 0.985 {
                    continue;
                }
                let delta = b.position - a.position;
                let same_face = delta.dot(a.normal).abs() <= 1.0;
                if !same_face {
                    continue;
                }
                let tangential_span = (delta - a.normal * delta.dot(a.normal)).length();
                if tangential_span < 6.0 {
                    continue;
                }
                neighbors.push((j, tangential_span));
            }
            neighbors.sort_by(|lhs, rhs| lhs.1.partial_cmp(&rhs.1).unwrap_or(std::cmp::Ordering::Equal));
            let neighbor_limit = if a.tier == 1 {
                if eff_density >= 5.2 { 3 } else if eff_density >= 2.8 { 2 } else { 1 }
            } else if eff_density >= 4.5 {
                2
            } else {
                1
            };
            for (j, _) in neighbors.into_iter().take(neighbor_limit) {
                let b = &points[j];
                let subdiv = if a.tier == 1 { tier1_subdiv } else { tier2_subdiv };
                if subdiv == 0 {
                    continue;
                }
                for k in 1..=subdiv {
                    let t = k as f32 / (subdiv + 1) as f32;
                    out.push(BracketMountPoint {
                        position: a.position.lerp(b.position, t),
                        normal: safe_normalize(a.normal.lerp(b.normal, t), 1e-8),
                        tier: a.tier,
                        base_radius: (a.base_radius + b.base_radius) * 0.5,
                    });
                }
            }
        }

        out
    }

    fn get_host_sdf(parts: &[BracketPart], p: Vec3) -> (f32, Option<String>) {
        aggregate_parts(parts, StructuralRole::Sticky, p)
    }

    fn get_keepout_sdf(parts: &[BracketPart], p: Vec3) -> (f32, Option<String>) {
        aggregate_parts(parts, StructuralRole::Removable, p)
    }

    fn trace_path_from_mount(
        mount: &BracketMountPoint,
        host_parts: &[BracketPart],
        keepout_parts: &[BracketPart],
        config: &BracketConfig,
    ) -> BracketPathResult {
        let initial_offset = (mount.base_radius * 0.1).max(0.5);
        let mut p = mount.position + mount.normal * initial_offset;
        let mut points = vec![mount.position, p];
        let mut termination_reason = "max_iter".to_string();
        let mut last_host_id = None;
        let mut last_keepout_id = None;
        let mut min_keepout_distance = f32::INFINITY;
        let mut final_host_distance = f32::INFINITY;

        for i in 0..config.max_path_iters {
            let (host_dist_here, host_id_here) = get_host_sdf(host_parts, p);
            let (keepout_dist_here, keepout_id_here) = get_keepout_sdf(keepout_parts, p);
            last_host_id = host_id_here.clone();
            last_keepout_id = keepout_id_here.clone();
            min_keepout_distance = min_keepout_distance.min(keepout_dist_here);
            final_host_distance = host_dist_here.abs();

            let host_fn = |q: Vec3| get_host_sdf(host_parts, q).0;
            let keepout_fn = |q: Vec3| get_keepout_sdf(keepout_parts, q).0;

            let attr = -calc_gradient(host_fn, p, config.grad_eps);
            let rep = calc_gradient(keepout_fn, p, config.grad_eps);
            let rep_scale = (1.0 / (keepout_dist_here.abs() + 1e-2)).clamp(0.0, config.rep_scale_max);

            let mut combined = attr + rep * rep_scale;
            if combined.length() < config.safe_norm_eps {
                combined = if attr.length() >= config.safe_norm_eps { attr } else { mount.normal };
            }
            let combined = safe_normalize(combined, config.safe_norm_eps);

            let adaptive_step = (config.step_size_mm * host_dist_here.abs().max(0.5))
                .clamp(config.min_step_mm, config.max_step_mm);
            let mut step = combined * adaptive_step;

            if points.len() >= 3 {
                let prev = points[points.len() - 3];
                if p.distance(prev) < adaptive_step * 0.75 {
                    step *= 0.5;
                }
            }

            if step.length() < config.min_step_mm * 0.1 {
                termination_reason = "stuck_small_step".to_string();
                return BracketPathResult {
                    points,
                    termination_reason,
                    iterations: i + 1,
                    host_part_id: last_host_id,
                    keepout_part_id: last_keepout_id,
                    min_keepout_distance,
                    final_host_distance,
                };
            }

            let p_next = p + step;
            points.push(p_next);

            let (host_dist_next_raw, host_id_next) = get_host_sdf(host_parts, p_next);
            let host_dist_next = host_dist_next_raw.abs();
            final_host_distance = host_dist_next;
            last_host_id = host_id_next.clone();
            if host_dist_next <= config.surface_tol_mm {
                let snapped = if let Some(part) = host_parts.iter().find(|part| Some(&part.id) == host_id_next.as_ref()) {
                    crate::sdf::query::closest_point(part.sdf.as_ref(), p_next)
                } else {
                    p_next
                };
                if points.last().copied() != Some(snapped) {
                    points.push(snapped);
                }
                termination_reason = "hit_host".to_string();
                return BracketPathResult {
                    points,
                    termination_reason,
                    iterations: i + 1,
                    host_part_id: last_host_id,
                    keepout_part_id: last_keepout_id,
                    min_keepout_distance,
                    final_host_distance,
                };
            }

            let (keepout_dist_next, keepout_id_next) = get_keepout_sdf(keepout_parts, p_next);
            last_keepout_id = keepout_id_next;
            min_keepout_distance = min_keepout_distance.min(keepout_dist_next);
            if keepout_dist_next <= -0.5 {
                termination_reason = "violated_keepout".to_string();
                return BracketPathResult {
                    points,
                    termination_reason,
                    iterations: i + 1,
                    host_part_id: last_host_id,
                    keepout_part_id: last_keepout_id,
                    min_keepout_distance,
                    final_host_distance,
                };
            }

            p = p_next;
        }

        BracketPathResult {
            points,
            termination_reason,
            iterations: config.max_path_iters,
            host_part_id: last_host_id,
            keepout_part_id: last_keepout_id,
            min_keepout_distance,
            final_host_distance,
        }
    }

    fn make_capsule_chain(points: &[Vec3], base_radius: f32, radius_end_factor: f32) -> Vec<Arc<dyn Sdf>> {
        if points.len() < 2 {
            return Vec::new();
        }
        let seg_count = points.len() - 1;
        let mut out = Vec::new();
        for (i, seg) in points.windows(2).enumerate() {
            let t0 = i as f32 / seg_count as f32;
            let t1 = (i + 1) as f32 / seg_count as f32;
            let r0 = base_radius + t0 * (base_radius * radius_end_factor - base_radius);
            let r1 = base_radius + t1 * (base_radius * radius_end_factor - base_radius);
            let a = seg[0];
            let b = seg[1];
            let sdf = FnSdf {
                func: Arc::new(move |p: Vec3| sdf_tapered_capsule_distance(p, a, b, r0, r1)),
            };
            out.push(Arc::new(sdf) as Arc<dyn Sdf>);
        }
        out
    }

    fn make_tier1_cover(
        points: &[BracketMountPoint],
        tray_clearance: f32,
        tray_thickness: f32,
    ) -> Vec<Arc<dyn Sdf>> {
        let tier1_points: Vec<&BracketMountPoint> = points.iter().filter(|p| p.tier == 1).collect();
        if tier1_points.is_empty() {
            return Vec::new();
        }
        let mut out = Vec::new();
        let face_radius = (tray_thickness * 0.55).max(0.5);
        let offset_positions: Vec<Vec3> = tier1_points
            .iter()
            .map(|mp| mp.position + mp.normal * tray_clearance)
            .collect();

        let mut connect_pair = |a: Vec3, b: Vec3| {
            let sdf = FnSdf {
                func: Arc::new(move |p: Vec3| sdf_tapered_capsule_distance(p, a, b, face_radius, face_radius)),
            };
            out.push(Arc::new(sdf) as Arc<dyn Sdf>);
        };

        // Connect nearby points on the same support face to make local tray rails.
        for i in 0..tier1_points.len() {
            let a = tier1_points[i];
            let mut neighbors: Vec<(usize, f32)> = Vec::new();
            for j in (i + 1)..tier1_points.len() {
                let b = tier1_points[j];
                if a.normal.dot(b.normal) < 0.985 {
                    continue;
                }
                let delta = b.position - a.position;
                if delta.dot(a.normal).abs() > 1.0 {
                    continue;
                }
                let tangential_span = (delta - a.normal * delta.dot(a.normal)).length();
                if tangential_span < 8.0 {
                    continue;
                }
                neighbors.push((j, tangential_span));
            }
            neighbors.sort_by(|lhs, rhs| lhs.1.partial_cmp(&rhs.1).unwrap_or(std::cmp::Ordering::Equal));
            for (j, _) in neighbors.into_iter().take(2) {
                connect_pair(offset_positions[i], offset_positions[j]);
            }
        }

        // Also force the tier-1 cover to be globally connected by building a minimal
        // spanning network across all tier-1 tray points. This turns per-face strips
        // into a single tray perimeter instead of disconnected local patches.
        if tier1_points.len() >= 2 {
            let mut connected = vec![false; tier1_points.len()];
            connected[0] = true;
            for _ in 1..tier1_points.len() {
                let mut best: Option<(usize, usize, f32)> = None;
                for i in 0..tier1_points.len() {
                    if !connected[i] {
                        continue;
                    }
                    for j in 0..tier1_points.len() {
                        if connected[j] || i == j {
                            continue;
                        }
                        let d = offset_positions[i].distance(offset_positions[j]);
                        match best {
                            Some((_, _, best_d)) if d >= best_d => {}
                            _ => best = Some((i, j, d)),
                        }
                    }
                }
                if let Some((i, j, _)) = best {
                    connect_pair(offset_positions[i], offset_positions[j]);
                    connected[j] = true;
                } else {
                    break;
                }
            }
        }

        for mp in tier1_points {
            let p0 = mp.position + mp.normal * tray_clearance;
            let p1 = mp.position + mp.normal * (tray_clearance + tray_thickness);
            connect_pair(p0, p1);
        }
        out
    }

    fn make_offset_band(seed: Arc<dyn Sdf>, offset_mm: f32, thickness_mm: f32) -> Arc<dyn Sdf> {
        let inner = Arc::new(Offset::new(Arc::clone(&seed), offset_mm));
        let outer = Arc::new(Offset::new(Arc::clone(&seed), offset_mm + thickness_mm));
        Arc::new(Subtract::new(outer, inner))
    }

    fn make_smooth_union(parts: Vec<Arc<dyn Sdf>>, k: f32) -> Arc<dyn Sdf> {
        if parts.is_empty() {
            return Arc::new(FnSdf { func: Arc::new(|_| 1e6) });
        }
        Arc::new(FnSdf {
            func: Arc::new(move |p: Vec3| {
                let mut acc = parts[0].distance(p);
                for sdf in parts.iter().skip(1) {
                    acc = smin_exp_pair(acc, sdf.distance(p), k);
                }
                acc
            }),
        })
    }

    engine.register_fn("mount_point",
        |position: PointHandle, normal: PointHandle, tier: i64, base_radius: f64| -> rhai::Map {
            let mut map = rhai::Map::new();
            map.insert("position".into(), rhai::Dynamic::from(position));
            map.insert("normal".into(), rhai::Dynamic::from(normal));
            map.insert("tier".into(), rhai::Dynamic::from(tier));
            map.insert("base_radius".into(), rhai::Dynamic::from(base_radius));
            map
        }
    );

    engine.register_fn("mount_component_granular",
        |parent: SdfHandle, comp_map: rhai::Map, x: f64, y: f64, z: f64|
         -> Result<rhai::Map, Box<rhai::EvalAltResult>> {
            let pos = Vec3::new(x as f32, y as f32, z as f32);
            let placed = place_component_map_impl(&comp_map, pos);
            let mount_points = expand_mount_points_for_density(
                &parse_mount_points(&comp_map, pos)?,
                apply_bracket_config_overrides(BracketConfig::default(), &comp_map).support_density
            );

            let physical = map_get_sdf(&placed, "physical")?;
            let keepout = map_get_sdf(&placed, "keepout")?;
            let service_keepout = map_get_optional_sdf(&placed, "service_keepout");
            let swept_volume = map_get_optional_sdf(&placed, "swept_volume");
            let fastener_pad_limit = map_get_optional_sdf(&placed, "fastener_pad_limit");
            let fastener_pad_seed = map_get_optional_sdf(&placed, "fastener_pad_seed");
            let fastener_keepout = map_get_optional_sdf(&placed, "fastener_keepout");

            let host_parts = vec![
                bracket_part("host", Arc::clone(&parent.0), StructuralRole::Sticky),
            ];
            let mut keepout_parts = vec![
                bracket_part("keepout", Arc::clone(&keepout.0), StructuralRole::Removable),
            ];
            if let Some(sk) = service_keepout.as_ref() {
                keepout_parts.push(bracket_part("service_keepout", Arc::clone(&sk.0), StructuralRole::Removable));
            }
            if let Some(sw) = swept_volume.as_ref() {
                keepout_parts.push(bracket_part("swept_volume", Arc::clone(&sw.0), StructuralRole::Removable));
            }
            if let Some(fk) = fastener_keepout.as_ref() {
                keepout_parts.push(bracket_part("fastener_keepout", Arc::clone(&fk.0), StructuralRole::Removable));
            }

            let mut config = apply_bracket_config_overrides(BracketConfig::default(), &comp_map);
            let eff_density = effective_support_density(config.support_density);
            config.tier2_bridge_thresh_mm *= 1.0 + 0.10 * (eff_density - 1.0);

            let tier1_cover = if let Some(seed) = map_get_optional_sdf(&placed, "tray_seed") {
                make_offset_band(
                    Arc::clone(&seed.0),
                    config.tray_clearance_mm,
                    config.tray_thickness_mm,
                )
            } else {
                make_smooth_union(
                    make_tier1_cover(
                        &mount_points,
                        config.tray_clearance_mm,
                        config.tray_thickness_mm,
                    ),
                    8.0
                )
            };

            let fastener_pads_raw = fastener_pad_seed
                .as_ref()
                .or(fastener_keepout.as_ref())
                .map(|fk| {
                make_offset_band(
                    Arc::clone(&fk.0),
                    config.tray_clearance_mm,
                    (config.tray_thickness_mm * 1.5).max(config.tray_thickness_mm + 0.5),
                )
            });

            let fastener_pads = if let (Some(raw), Some(limit)) = (fastener_pads_raw.as_ref(), fastener_pad_limit.as_ref()) {
                Some(Arc::new(Intersect::new(Arc::clone(raw), Arc::clone(&limit.0))) as Arc<dyn Sdf>)
            } else {
                fastener_pads_raw
            };

            let mut tier1_capsules = Vec::new();
            let mut tier1_paths = Vec::new();
            let mut tier2_capsules = Vec::new();
            let mut debug_paths = rhai::Array::new();

            for mount in mount_points.iter().filter(|m| m.tier == 1) {
                let path = trace_path_from_mount(mount, &host_parts, &keepout_parts, &config);
                let mut dbg = rhai::Map::new();
                dbg.insert("tier".into(), rhai::Dynamic::from(1_i64));
                dbg.insert("termination_reason".into(), rhai::Dynamic::from(path.termination_reason.clone()));
                dbg.insert("iterations".into(), rhai::Dynamic::from(path.iterations as i64));
                dbg.insert("point_count".into(), rhai::Dynamic::from(path.points.len() as i64));
                dbg.insert("host_part_id".into(), rhai::Dynamic::from(path.host_part_id.clone().unwrap_or_default()));
                dbg.insert("keepout_part_id".into(), rhai::Dynamic::from(path.keepout_part_id.clone().unwrap_or_default()));
                dbg.insert("min_keepout_distance".into(), rhai::Dynamic::from(path.min_keepout_distance as f64));
                dbg.insert("final_host_distance".into(), rhai::Dynamic::from(path.final_host_distance as f64));
                debug_paths.push(rhai::Dynamic::from(dbg));
                if path.points.len() >= 2 {
                    tier1_capsules.extend(make_capsule_chain(&path.points, mount.base_radius, config.tier1_radius_end_factor));
                    tier1_paths.push(path.points);
                }
            }

            for mount in mount_points.iter().filter(|m| m.tier == 2) {
                let path = trace_path_from_mount(mount, &host_parts, &keepout_parts, &config);
                let mut dbg = rhai::Map::new();
                dbg.insert("tier".into(), rhai::Dynamic::from(2_i64));
                dbg.insert("termination_reason".into(), rhai::Dynamic::from(path.termination_reason.clone()));
                dbg.insert("iterations".into(), rhai::Dynamic::from(path.iterations as i64));
                dbg.insert("point_count".into(), rhai::Dynamic::from(path.points.len() as i64));
                dbg.insert("host_part_id".into(), rhai::Dynamic::from(path.host_part_id.clone().unwrap_or_default()));
                dbg.insert("keepout_part_id".into(), rhai::Dynamic::from(path.keepout_part_id.clone().unwrap_or_default()));
                dbg.insert("min_keepout_distance".into(), rhai::Dynamic::from(path.min_keepout_distance as f64));
                dbg.insert("final_host_distance".into(), rhai::Dynamic::from(path.final_host_distance as f64));
                debug_paths.push(rhai::Dynamic::from(dbg));
                let Some(end) = path.points.last().copied() else { continue; };

                let mut best: Option<(f32, Vec3)> = None;
                for tier1_path in &tier1_paths {
                    if let Some(p_tier1) = nearest_point_on_polyline(tier1_path, end) {
                        let d = end.distance(p_tier1);
                        if d < config.tier2_bridge_thresh_mm {
                            match best {
                                Some((best_d, _)) if d >= best_d => {}
                                _ => best = Some((d, p_tier1)),
                            }
                        }
                    }
                }

                if let Some((_, p_tier1)) = best {
                    let a = end;
                    let b = p_tier1;
                    let bridge_radius = (mount.base_radius * config.tier2_bridge_radius_factor).max(0.8);
                    let sdf = FnSdf {
                        func: Arc::new(move |p: Vec3| sdf_tapered_capsule_distance(p, a, b, bridge_radius, bridge_radius)),
                    };
                    tier2_capsules.push(Arc::new(sdf) as Arc<dyn Sdf>);
                }
            }

            let mut all_capsules = Vec::new();
            all_capsules.push(Arc::clone(&tier1_cover));
            all_capsules.extend(tier1_capsules);
            all_capsules.extend(tier2_capsules);
            let bracket_field = make_smooth_union(all_capsules, 2.0);

            let keepout_union = {
                let mut parts = vec![Arc::clone(&keepout.0)];
                if let Some(sk) = service_keepout.as_ref() { parts.push(Arc::clone(&sk.0)); }
                if let Some(sw) = swept_volume.as_ref() { parts.push(Arc::clone(&sw.0)); }
                make_smooth_union(parts, 32.0)
            };

            let dilate_mm = config.dilate_keepout_mm;
            let bracket_after_cut: Arc<dyn Sdf> = Arc::new(FnSdf {
                func: {
                    let bracket_field = Arc::clone(&bracket_field);
                    let keepout_union = Arc::clone(&keepout_union);
                    Arc::new(move |p: Vec3| {
                        let dilated_keepout = keepout_union.distance(p) - dilate_mm;
                        bracket_field.distance(p).max(-dilated_keepout)
                    })
                },
            });

            let bracket_with_fasteners: Arc<dyn Sdf> = if let (Some(fp), Some(fk)) = (fastener_pads.as_ref(), fastener_keepout.as_ref()) {
                Arc::new(FnSdf {
                    func: {
                        let bracket_after_cut = Arc::clone(&bracket_after_cut);
                        let fastener_pads = Arc::clone(fp);
                        let fastener_keepout = Arc::clone(&fk.0);
                        Arc::new(move |p: Vec3| {
                            let pads_cut = fastener_pads.distance(p).max(-fastener_keepout.distance(p));
                            smin_exp_pair(bracket_after_cut.distance(p), pads_cut, 8.0)
                        })
                    },
                })
            } else {
                Arc::clone(&bracket_after_cut)
            };

            let final_blended: Arc<dyn Sdf> = Arc::new(FnSdf {
                func: {
                    let bracket_with_fasteners = Arc::clone(&bracket_with_fasteners);
                    let host = Arc::clone(&parent.0);
                    let host_blend_k = config.host_blend_k;
                    Arc::new(move |p: Vec3| {
                        if host_blend_k <= 0.0 {
                            bracket_with_fasteners.distance(p).min(host.distance(p))
                        } else {
                            smin_exp_pair(bracket_with_fasteners.distance(p), host.distance(p), host_blend_k)
                        }
                    })
                },
            });

            let pocket_offset = config.pocket_offset_mm;
            let final_mount_raw: Arc<dyn Sdf> = Arc::new(FnSdf {
                func: {
                    let final_blended = Arc::clone(&final_blended);
                    let physical: Arc<dyn Sdf> = Arc::clone(&physical.0);
                    Arc::new(move |p: Vec3| {
                        final_blended.distance(p).max(-(physical.distance(p) + pocket_offset))
                    })
                },
            });

            let final_mount: Arc<dyn Sdf> = if let Some(fk) = fastener_keepout.as_ref() {
                Arc::new(FnSdf {
                    func: {
                        let final_mount_raw = Arc::clone(&final_mount_raw);
                        let fastener_keepout = Arc::clone(&fk.0);
                        Arc::new(move |p: Vec3| {
                            final_mount_raw.distance(p).max(-fastener_keepout.distance(p))
                        })
                    },
                })
            } else {
                Arc::clone(&final_mount_raw)
            };

            let mut out = placed;
            out.insert("raw_bracket".into(), rhai::Dynamic::from(SdfHandle(Arc::clone(&bracket_field))));
            out.insert("tray".into(), rhai::Dynamic::from(SdfHandle(Arc::clone(&tier1_cover))));
            if let Some(fp) = fastener_pads {
                out.insert("fastener_pads".into(), rhai::Dynamic::from(SdfHandle(fp)));
            }
            out.insert("cut_bracket".into(), rhai::Dynamic::from(SdfHandle(Arc::clone(&bracket_after_cut))));
            out.insert("fastener_bracket".into(), rhai::Dynamic::from(SdfHandle(Arc::clone(&bracket_with_fasteners))));
            out.insert("blended_bracket".into(), rhai::Dynamic::from(SdfHandle(Arc::clone(&final_blended))));
            out.insert("bracket".into(), rhai::Dynamic::from(SdfHandle(Arc::clone(&final_mount))));
            out.insert("assembly".into(), rhai::Dynamic::from(SdfHandle(final_mount)));
            out.insert("component_physical".into(), rhai::Dynamic::from(physical));
            out.insert("debug_paths".into(), rhai::Dynamic::from(debug_paths));
            let mut debug_summary = rhai::Map::new();
            debug_summary.insert("tier1_count".into(), rhai::Dynamic::from(tier1_paths.len() as i64));
            debug_summary.insert("tier2_count".into(), rhai::Dynamic::from((mount_points.iter().filter(|m| m.tier == 2).count()) as i64));
            debug_summary.insert("dilate_keepout_mm".into(), rhai::Dynamic::from(config.dilate_keepout_mm as f64));
            debug_summary.insert("pocket_offset_mm".into(), rhai::Dynamic::from(config.pocket_offset_mm as f64));
            debug_summary.insert("host_blend_k".into(), rhai::Dynamic::from(config.host_blend_k as f64));
            debug_summary.insert("support_density".into(), rhai::Dynamic::from(config.support_density as i64));
            debug_summary.insert("effective_support_density".into(), rhai::Dynamic::from(effective_support_density(config.support_density) as f64));
            debug_summary.insert("tray_clearance_mm".into(), rhai::Dynamic::from(config.tray_clearance_mm as f64));
            debug_summary.insert("tray_thickness_mm".into(), rhai::Dynamic::from(config.tray_thickness_mm as f64));
            out.insert("debug_summary".into(), rhai::Dynamic::from(debug_summary));
            Ok(out)
        }
    );
}

fn register_placement_functions(engine: &mut Engine) {
    use crate::sdf::query::bounding_points;

    // place_behind(body, anchor, gap) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â places body in -X direction from anchor
    engine.register_fn("place_behind", |body: SdfHandle, anchor: SdfHandle, gap: f64| {
        let anchor_bi = bounding_points(&*anchor.0);
        let body_bi   = bounding_points(&*body.0);
        let body_half_len = (body_bi.max.x - body_bi.min.x) * 0.5;
        let x = anchor_bi.min.x - gap as f32 - body_half_len;
        SdfHandle(Arc::new(Translate::new(body.0, Vec3::new(x, 0.0, 0.0))))
    });

    // place_above(body, anchor, gap) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â places body in +Z direction from anchor
    engine.register_fn("place_above", |body: SdfHandle, anchor: SdfHandle, gap: f64| {
        let anchor_bi = bounding_points(&*anchor.0);
        let body_bi   = bounding_points(&*body.0);
        let body_half = (body_bi.max.z - body_bi.min.z) * 0.5;
        let z = anchor_bi.max.z + gap as f32 + body_half;
        SdfHandle(Arc::new(Translate::new(body.0, Vec3::new(0.0, 0.0, z))))
    });

    // place_below(body, anchor, gap) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â places body in -Z direction from anchor
    engine.register_fn("place_below", |body: SdfHandle, anchor: SdfHandle, gap: f64| {
        let anchor_bi = bounding_points(&*anchor.0);
        let body_bi   = bounding_points(&*body.0);
        let body_half = (body_bi.max.z - body_bi.min.z) * 0.5;
        let z = anchor_bi.min.z - gap as f32 - body_half;
        SdfHandle(Arc::new(Translate::new(body.0, Vec3::new(0.0, 0.0, z))))
    });

    // place_beside(body, anchor, gap) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â places body in +Y direction from anchor
    engine.register_fn("place_beside", |body: SdfHandle, anchor: SdfHandle, gap: f64| {
        let anchor_bi = bounding_points(&*anchor.0);
        let body_bi   = bounding_points(&*body.0);
        let body_half = (body_bi.max.y - body_bi.min.y) * 0.5;
        let y = anchor_bi.max.y + gap as f32 + body_half;
        SdfHandle(Arc::new(Translate::new(body.0, Vec3::new(0.0, y, 0.0))))
    });

    // attach_to_fuselage_station(body, fuselage, station, offset_y, offset_z) -> SdfHandle
    // Places body at the normalized fuselage station with lateral and vertical offsets
    engine.register_fn("attach_to_fuselage_station",
        |body: SdfHandle, fuselage: SdfHandle, station: f64, offset_y: f64, offset_z: f64| {
        let bi = bounding_points(&*fuselage.0);
        let x = bi.min.x + (bi.max.x - bi.min.x) * station as f32;
        let offset = Vec3::new(x, offset_y as f32, offset_z as f32);
        SdfHandle(Arc::new(Translate::new(body.0, offset)))
    });

    // attach_to_trailing_edge(body, wing, span_frac, offset_x, offset_z) -> SdfHandle
    // Uses trailing edge query to find TE position, then places body relative to it
    engine.register_fn("attach_to_trailing_edge",
        |body: SdfHandle, wing: SdfHandle, span_frac: f64, offset_x: f64, offset_z: f64| {
        use crate::sdf::query::furthest_point;
        let bi    = bounding_points(&*wing.0);
        let y_pos = bi.min.y + (bi.max.y - bi.min.y) * span_frac as f32;
        let te    = furthest_point(&*wing.0, Vec3::X);
        let offset = Vec3::new(te.x + offset_x as f32, y_pos, te.z + offset_z as f32);
        SdfHandle(Arc::new(Translate::new(body.0, offset)))
    });
}

// ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ Instancing functions ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬ÃƒÂ¢Ã¢â‚¬ÂÃ¢â€šÂ¬

fn register_instance_functions(engine: &mut Engine) {
    // instance(body, transforms: Array of Maps) -> SdfHandle
    // Each map can have: tx, ty, tz, rx, ry, rz, sx, sy, sz
    engine.register_fn("instance", |body: SdfHandle, transforms: rhai::Array| -> SdfHandle {
        if transforms.is_empty() {
            return body;
        }

        let instances: Vec<Arc<dyn crate::sdf::Sdf>> = transforms.iter().filter_map(|t| {
            let map = t.clone().try_cast::<rhai::Map>()?;
            let get_f = |key: &str| -> f32 {
                map.get(key).and_then(|v| v.clone().as_float().ok()).unwrap_or(0.0) as f32
            };
            let get_f_default = |key: &str, default: f32| -> f32 {
                map.get(key).and_then(|v| v.clone().as_float().ok()).unwrap_or(default as f64) as f32
            };

            let tx = get_f("tx"); let ty = get_f("ty"); let tz = get_f("tz");
            let rx = get_f("rx"); let ry = get_f("ry"); let rz = get_f("rz");
            let sx = get_f_default("sx", 1.0);
            let sy = get_f_default("sy", 1.0);
            let sz = get_f_default("sz", 1.0);

            let mut result: Arc<dyn crate::sdf::Sdf> = Arc::clone(&body.0);

            // Scale
            if (sx - 1.0).abs() > 1e-6 || (sy - 1.0).abs() > 1e-6 || (sz - 1.0).abs() > 1e-6 {
                result = Arc::new(Scale::new(result, Vec3::new(sx, sy, sz)));
            }
            // Rotate
            if rx.abs() > 1e-6 || ry.abs() > 1e-6 || rz.abs() > 1e-6 {
                let q = Quat::from_euler(
                    glam::EulerRot::XYZ,
                    rx.to_radians(), ry.to_radians(), rz.to_radians(),
                );
                result = Arc::new(Rotate::new(result, q));
            }
            // Translate
            if tx.abs() > 1e-6 || ty.abs() > 1e-6 || tz.abs() > 1e-6 {
                result = Arc::new(Translate::new(result, Vec3::new(tx, ty, tz)));
            }
            Some(result)
        }).collect();

        if instances.is_empty() { return body; }

        let union_tree = instances.into_iter().reduce(|a, b| {
            Arc::new(Union::new(a, b)) as Arc<dyn crate::sdf::Sdf>
        }).unwrap();
        SdfHandle(union_tree)
    });

    // instance_grid(body, nx, ny, nz, dx, dy, dz) -> SdfHandle
    engine.register_fn("instance_grid",
        |body: SdfHandle, nx: i64, ny: i64, nz: i64, dx: f64, dy: f64, dz: f64| -> SdfHandle {
        let nx = nx.max(1) as usize;
        let ny = ny.max(1) as usize;
        let nz = nz.max(1) as usize;
        let dx = dx as f32;
        let dy = dy as f32;
        let dz = dz as f32;

        // Center the grid about the origin
        let ox = -(nx as f32 - 1.0) * dx * 0.5;
        let oy = -(ny as f32 - 1.0) * dy * 0.5;
        let oz = -(nz as f32 - 1.0) * dz * 0.5;

        let mut result: Option<Arc<dyn crate::sdf::Sdf>> = None;
        for ix in 0..nx {
            for iy in 0..ny {
                for iz in 0..nz {
                    let pos = Vec3::new(
                        ox + ix as f32 * dx,
                        oy + iy as f32 * dy,
                        oz + iz as f32 * dz,
                    );
                    let inst: Arc<dyn crate::sdf::Sdf> =
                        Arc::new(Translate::new(Arc::clone(&body.0), pos));
                    result = Some(match result {
                        None    => inst,
                        Some(r) => Arc::new(Union::new(r, inst)),
                    });
                }
            }
        }
        SdfHandle(result.unwrap_or(body.0))
    });

    // instance_along_path(body, path, count) -> SdfHandle
    engine.register_fn("instance_along_path",
        |body: SdfHandle, path: PathHandle, count: i64| -> SdfHandle {
        use crate::sdf::sweep::compute_frames;
        let n      = count.max(1) as usize;
        let frames = compute_frames(path.0.as_ref(), n, 0.0, 0.0);

        let mut result: Option<Arc<dyn crate::sdf::Sdf>> = None;
        for (pos, mat) in frames {
            let q        = Quat::from_mat3(&mat);
            let rotated: Arc<dyn crate::sdf::Sdf> = Arc::new(Rotate::new(Arc::clone(&body.0), q));
            let placed:  Arc<dyn crate::sdf::Sdf> = Arc::new(Translate::new(rotated, pos));
            result = Some(match result {
                None    => placed,
                Some(r) => Arc::new(Union::new(r, placed)),
            });
        }
        SdfHandle(result.unwrap_or(body.0))
    });
}
