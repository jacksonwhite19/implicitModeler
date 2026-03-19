use std::sync::{Arc, Mutex};

use glam::Vec3;
use rhai::Engine;

use crate::sdf::Sdf;
use crate::sdf::booleans::Union;
use crate::sdf::booleans::Intersect;
use crate::sdf::print::fasteners::measure_wall_thickness;
use crate::sdf::print::{
    AlignmentFeature, SplitPlane, ToleranceCompensated, ToleranceSettings, split_body,
};
use crate::sdf::query::bounding_points;

use super::{FieldHandle, MotorHandle, PointHandle, PropHandle, PropulsionHandle, SdfHandle, SectionHandle};

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

fn parse_axis_string(axis: &str) -> Result<Vec3, Box<rhai::EvalAltResult>> {
    match axis.to_ascii_lowercase().as_str() {
        "x" => Ok(Vec3::X),
        "y" => Ok(Vec3::Y),
        "z" => Ok(Vec3::Z),
        other => Err(format!("Invalid axis '{}': expected x, y, or z", other).into()),
    }
}

fn plane_from_axis(axis: &str, pos: f64) -> Result<SplitPlane, Box<rhai::EvalAltResult>> {
    match axis.to_ascii_lowercase().as_str() {
        "x" => Ok(SplitPlane::X(pos as f32)),
        "y" => Ok(SplitPlane::Y(pos as f32)),
        "z" => Ok(SplitPlane::Z(pos as f32)),
        other => Err(format!("Invalid axis '{}': expected x, y, or z", other).into()),
    }
}

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

fn resolve_legacy_span_range(_sdf: &dyn Sdf, span_start: f32, span_end: f32, half_span: f32) -> (f32, f32) {
    if span_start.abs() <= 1.0 && span_end.abs() <= 1.0 {
        (span_start * half_span, span_end * half_span)
    } else {
        (span_start, span_end)
    }
}

pub fn register_legacy_compat_functions(engine: &mut Engine) {
    use crate::sdf::aerospace::control_surfaces::{
        ControlHornSpec, HingeSpec, LinkageSpec, aileron as cs_aileron,
        elevator as cs_elevator, elevon as cs_elevon, rudder as cs_rudder,
        estimate_half_span,
    };
    use crate::sdf::aerospace::mechanical::CappedCone;
    use crate::sdf::booleans::Subtract;
    use crate::sdf::field::gradients::{GradientField, RadialField};
    use crate::sdf::field::operations::OffsetByField;
    use crate::sdf::patterns::PolarArray;
    use crate::sdf::primitives::Cylinder;
    use crate::sdf::sweep::{LinePath, Sweep, SweepPath};
    use crate::sdf::transforms::Translate;

    engine.register_fn("tail_cone", |length: f64, diam_start: f64, diam_end: f64| -> SdfHandle {
        SdfHandle(Arc::new(CappedCone {
            r1: diam_start as f32 / 2.0,
            r2: diam_end as f32 / 2.0,
            h: length as f32 / 2.0,
        }))
    });

    engine.register_fn("haack_nose", |length: f64, base_diam: f64| -> SdfHandle {
        use crate::sdf::aerospace::HaackNose;
        SdfHandle(Arc::new(HaackNose::new(length as f32, base_diam as f32 / 2.0, 0.0)))
    });

    engine.register_fn("aileron", |wing: SdfHandle, span_start: f64, span_end: f64, chord_fraction: f64| -> SdfHandle {
        let hinge = HingeSpec::rounded(1.5, 0.5);
        let linkage = LinkageSpec::horn(ControlHornSpec::default_lower(15.0, 10.0, 0.5));
        let half_span = estimate_half_span(wing.0.as_ref());
        let (span_start, span_end) = resolve_legacy_span_range(wing.0.as_ref(), span_start as f32, span_end as f32, half_span);
        let result = cs_aileron(wing.0, span_start, span_end, chord_fraction as f32, hinge, linkage);
        SdfHandle(result.control_surface)
    });
    engine.register_fn("elevon", |wing: SdfHandle, span_start: f64, span_end: f64, chord_fraction: f64| -> SdfHandle {
        let hinge = HingeSpec::rounded(1.5, 0.5);
        let linkage = LinkageSpec::horn(ControlHornSpec::default_lower(15.0, 10.0, 0.5));
        let half_span = estimate_half_span(wing.0.as_ref());
        let (span_start, span_end) = resolve_legacy_span_range(wing.0.as_ref(), span_start as f32, span_end as f32, half_span);
        let result = cs_elevon(wing.0, span_start, span_end, chord_fraction as f32, hinge, linkage);
        SdfHandle(result.control_surface)
    });
    engine.register_fn("elevator", |stab: SdfHandle, _span_frac: f64, chord_fraction: f64| -> SdfHandle {
        let hinge = HingeSpec::rounded(1.5, 0.5);
        let linkage = LinkageSpec::horn(ControlHornSpec::default_lower(15.0, 10.0, 0.5));
        let result = cs_elevator(stab.0, chord_fraction as f32, hinge, linkage);
        SdfHandle(result.control_surface)
    });
    engine.register_fn("rudder", |fin: SdfHandle, _span_frac: f64, chord_fraction: f64| -> SdfHandle {
        let hinge = HingeSpec::rounded(1.5, 0.5);
        let linkage = LinkageSpec::horn(ControlHornSpec::default_lower(15.0, 10.0, 0.5));
        let result = cs_rudder(fin.0, chord_fraction as f32, hinge, linkage);
        SdfHandle(result.control_surface)
    });

    engine.register_fn("rib_slab", |wing: SdfHandle, span_frac: f64, thickness: f64| -> SdfHandle {
        use crate::sdf::aerospace::rib_slab as core_rib_slab;
        let bi = bounding_points(&*wing.0);
        let span = bi.max.y - bi.min.y;
        let span_mm = bi.min.y + span * span_frac as f32;
        let slab = core_rib_slab(span_mm, thickness as f32);
        SdfHandle(Arc::new(Intersect::new(wing.0, slab)))
    });
    engine.register_fn("spar_cylinder", |wing: SdfHandle, chord_pos: f64, radius: f64| -> SdfHandle {
        use crate::sdf::aerospace::spar_cylinder as core_spar;
        let cyl = core_spar(chord_pos as f32, radius as f32);
        SdfHandle(Arc::new(Intersect::new(wing.0, cyl)))
    });
    engine.register_fn("bulkhead_at_station", |fuselage: SdfHandle, pos_mm: f64, thickness: f64| -> SdfHandle {
        use crate::sdf::aerospace::bulkhead_at_station as core_bh;
        let bi = bounding_points(&*fuselage.0);
        let extent_x = (bi.max.x - bi.min.x).max(1e-6);
        let norm_pos = ((pos_mm as f32 - bi.min.x) / extent_x).clamp(0.0, 1.0);
        SdfHandle(core_bh(fuselage.0, norm_pos, thickness as f32, 0, 0.0))
    });
    engine.register_fn("lightening_hole_pattern", |body: SdfHandle, count: i64, radial_pos: f64, hole_radius: f64| -> SdfHandle {
        use crate::sdf::aerospace::lightening_hole_pattern as core_lhp;
        SdfHandle(core_lhp(body.0, count.max(0) as usize, radial_pos as f32, hole_radius as f32, 2))
    });

    engine.register_fn("extrude", |section: SectionHandle, length: f64| -> SdfHandle {
        let half = length as f32 / 2.0;
        let path: Arc<dyn SweepPath> = Arc::new(LinePath {
            start: glam::Vec3::new(0.0, 0.0, -half),
            end: glam::Vec3::new(0.0, 0.0, half),
        });
        SdfHandle(Arc::new(Sweep::new(section.0, path, 0.0, 0.0)))
    });
    engine.register_fn("revolve", |section: SectionHandle, _axis_str: &str, _sweep_deg: f64| -> SdfHandle {
        let path: Arc<dyn SweepPath> = Arc::new(LinePath {
            start: glam::Vec3::new(0.0, 0.0, -0.01),
            end: glam::Vec3::new(0.0, 0.0, 0.01),
        });
        let swept = Arc::new(Sweep::new(Arc::clone(&section.0), path, 0.0, 0.0));
        SdfHandle(Arc::new(PolarArray::new(swept, 24, glam::Vec3::Z)))
    });

    engine.register_fn("heat_set_boss", |outer_r: f64, height: f64, insert_r: f64, insert_depth: f64| -> SdfHandle {
        let boss = Arc::new(Cylinder::new(outer_r as f32, height as f32 / 2.0));
        let hole = Arc::new(Cylinder::new(insert_r as f32, insert_depth as f32 / 2.0));
        let z_off = (height as f32 - insert_depth as f32) / 2.0;
        let hole = Arc::new(Translate::new(hole as Arc<dyn crate::sdf::Sdf>, glam::Vec3::new(0.0, 0.0, z_off)));
        SdfHandle(Arc::new(Subtract::new(boss, hole)))
    });

    engine.register_fn("split_body", |body: SdfHandle, axis_str: &str, pos: f64| -> rhai::Array {
        let plane = match axis_str.to_ascii_lowercase().as_str() {
            "x" => SplitPlane::X(pos as f32),
            "y" => SplitPlane::Y(pos as f32),
            _ => SplitPlane::Z(pos as f32),
        };
        let result = split_body(body.0, &plane, &AlignmentFeature::None);
        vec![
            rhai::Dynamic::from(SdfHandle(result.part_a)),
            rhai::Dynamic::from(SdfHandle(result.part_b)),
        ]
    });
    engine.register_fn("wall_thickness_at", |sdf: SdfHandle, x: f64, y: f64, z: f64, dir: &str| -> Result<f64, Box<rhai::EvalAltResult>> {
        let axis = parse_axis_string(dir)?;
        Ok(measure_wall_thickness(sdf.0.as_ref(), Vec3::new(x as f32, y as f32, z as f32), axis) as f64)
    });
    engine.register_fn("print_overhang_angle", |sdf: SdfHandle, upright: bool| -> rhai::Map {
        let (bounds_min, bounds_max) = analysis_bounds(sdf.0.as_ref());
        let build_dir = if upright { Vec3::Z } else { Vec3::X };
        let resolution = 28u32;
        let step = (bounds_max - bounds_min) / resolution as f32;
        let normal_eps = step.min_element().max(0.5);
        let threshold = 45.0_f32;
        let mut surface_count = 0usize;
        let mut overhang_count = 0usize;
        let mut critical_count = 0usize;
        let mut max_angle_deg = 0.0_f32;
        let mut overhang_area_mm2 = 0.0_f32;

        for z in 0..resolution {
            for y in 0..resolution {
                for x in 0..resolution {
                    let p = bounds_min + Vec3::new(
                        (x as f32 + 0.5) * step.x,
                        (y as f32 + 0.5) * step.y,
                        (z as f32 + 0.5) * step.z,
                    );
                    let d = sdf.0.distance(p);
                    if d.abs() > normal_eps * 1.5 {
                        continue;
                    }

                    let nx = sdf.0.distance(p + Vec3::X * normal_eps) - sdf.0.distance(p - Vec3::X * normal_eps);
                    let ny = sdf.0.distance(p + Vec3::Y * normal_eps) - sdf.0.distance(p - Vec3::Y * normal_eps);
                    let nz = sdf.0.distance(p + Vec3::Z * normal_eps) - sdf.0.distance(p - Vec3::Z * normal_eps);
                    let normal = Vec3::new(nx, ny, nz);
                    let len = normal.length();
                    if len < 1e-6 {
                        continue;
                    }

                    surface_count += 1;
                    let unit_normal = normal / len;
                    let dot = unit_normal.dot(-build_dir).clamp(-1.0, 1.0);
                    let overhang_angle = 180.0_f32 - dot.acos().to_degrees();
                    max_angle_deg = max_angle_deg.max(overhang_angle);
                    if overhang_angle > threshold {
                        overhang_count += 1;
                        overhang_area_mm2 += step.x.abs() * step.y.abs();
                    }
                    if overhang_angle > 90.0 {
                        critical_count += 1;
                    }
                }
            }
        }

        let mut map = rhai::Map::new();
        let surface_count = surface_count as f64;
        let fraction = if surface_count > 0.0 { overhang_count as f64 / surface_count } else { 0.0 };
        let critical_fraction = if surface_count > 0.0 { critical_count as f64 / surface_count } else { 0.0 };
        map.insert("max_angle_deg".into(), rhai::Dynamic::from(max_angle_deg as f64));
        map.insert("fraction_over_45".into(), rhai::Dynamic::from(fraction));
        map.insert("critical_fraction".into(), rhai::Dynamic::from(critical_fraction));
        map.insert("overhang_area_mm2".into(), rhai::Dynamic::from(overhang_area_mm2 as f64));
        map
    });
    engine.register_fn("tolerance_compensate", |body: SdfHandle, settings: rhai::Map| -> SdfHandle {
        let parsed = tolerance_settings_from_map(settings);
        SdfHandle(Arc::new(ToleranceCompensated::new(body.0, parsed)))
    });
    engine.register_fn("add_alignment_features", |body: SdfHandle, axis: &str, pos: f64, n_pins: i64| -> Result<SdfHandle, Box<rhai::EvalAltResult>> {
        let plane = plane_from_axis(axis, pos)?;
        let alignment = AlignmentFeature::PinsAndSockets {
            pin_radius: 2.0,
            pin_height: 8.0,
            socket_clearance: 0.15,
            count: n_pins.max(1) as usize,
            pattern_radius: 12.0,
        };
        let result = split_body(body.0, &plane, &alignment);
        Ok(SdfHandle(Arc::new(Union::new(result.part_a, result.part_b))))
    });
    engine.register_fn("add_alignment_features", |body: SdfHandle, axis: &str, pos: f64, n_pins: f64| -> Result<SdfHandle, Box<rhai::EvalAltResult>> {
        let plane = plane_from_axis(axis, pos)?;
        let alignment = AlignmentFeature::PinsAndSockets {
            pin_radius: 2.0,
            pin_height: 8.0,
            socket_clearance: 0.15,
            count: (n_pins.round() as i64).max(1) as usize,
            pattern_radius: 12.0,
        };
        let result = split_body(body.0, &plane, &alignment);
        Ok(SdfHandle(Arc::new(Union::new(result.part_a, result.part_b))))
    });
    engine.register_fn("alignment_pin", |radius: f64, height: f64| -> SdfHandle {
        SdfHandle(Arc::new(Cylinder::new(radius as f32, height as f32 / 2.0)))
    });
    engine.register_fn("alignment_socket", |radius: f64, height: f64| -> SdfHandle {
        SdfHandle(Arc::new(Cylinder::new(radius as f32, height as f32 / 2.0)))
    });
    engine.register_fn("cross_section_center", |sdf: SdfHandle, axis_str: &str, pos: f64| -> PointHandle {
        let axis = match axis_str.to_ascii_lowercase().as_str() {
            "x" => 0usize,
            "y" => 1,
            _ => 2,
        };
        PointHandle(crate::sdf::query::cross_section_centroid(sdf.0.as_ref(), axis, pos as f32))
    });
    engine.register_fn("radial_field", |cx: f64, cy: f64, cz: f64| -> FieldHandle {
        FieldHandle(Arc::new(RadialField::new(glam::Vec3::new(cx as f32, cy as f32, cz as f32), 0.0, 200.0, 0.0, 1.0)))
    });
    engine.register_fn("offset_by_field", |sdf: SdfHandle, field: FieldHandle, _scale: f64| -> SdfHandle {
        SdfHandle(Arc::new(OffsetByField::new(sdf.0, field.0)))
    });
    engine.register_fn("gradient_field", |dx: f64, dy: f64, dz: f64| -> FieldHandle {
        let d = glam::Vec3::new(dx as f32, dy as f32, dz as f32).normalize_or_zero();
        FieldHandle(Arc::new(GradientField::new(glam::Vec3::ZERO, d * 200.0, 0.0, 1.0)))
    });
    engine.register_fn("gyroid_field", |scale: f64| -> FieldHandle {
        FieldHandle(Arc::new(RadialField::new(glam::Vec3::ZERO, 0.0, scale as f32, 0.0, 1.0)))
    });
    engine.register_fn("recommend_motor_prop", |thrust: f64, cruise_v: f64, max_weight: i64| -> rhai::Array {
        use crate::aero::PropulsionDatabase;
        let db = PropulsionDatabase::new();
        db.recommend_motor_prop(thrust as f32, cruise_v as f32, max_weight as f32)
            .into_iter()
            .map(|r| {
                let mut m = rhai::Map::new();
                m.insert("motor_name".into(), rhai::Dynamic::from(r.motor.name.clone()));
                m.insert("prop_name".into(), rhai::Dynamic::from(r.prop.name.clone()));
                m.insert("cells".into(), rhai::Dynamic::from(r.cells as i64));
                rhai::Dynamic::from(m)
            })
            .collect()
    });
    engine.register_fn("propulsion_setup", |m: MotorHandle, p: PropHandle, cells: i64, cap: i64| -> PropulsionHandle {
        use crate::aero::PropulsionSetup;
        PropulsionHandle(Arc::new(PropulsionSetup {
            motor: (*m.0).clone(),
            prop: (*p.0).clone(),
            battery_cells: cells as u32,
            battery_capacity_mah: cap as f32,
            battery_c_rating: 20.0,
            motor_count: 1,
            efficiency_motor: 0.85,
            efficiency_esc: 0.95,
        }))
    });
    engine.register_fn("propulsion_setup", |m: MotorHandle, p: PropHandle, cells: f64, cap: f64| -> PropulsionHandle {
        use crate::aero::PropulsionSetup;
        PropulsionHandle(Arc::new(PropulsionSetup {
            motor: (*m.0).clone(),
            prop: (*p.0).clone(),
            battery_cells: cells.round().max(1.0) as u32,
            battery_capacity_mah: cap as f32,
            battery_c_rating: 20.0,
            motor_count: 1,
            efficiency_motor: 0.85,
            efficiency_esc: 0.95,
        }))
    });
    engine.register_fn("wing_from_sections", |sections: rhai::Array| -> Result<SdfHandle, Box<rhai::EvalAltResult>> {
        use crate::sdf::aerospace::{get_naca_airfoil, wing_from_sections as core_wfs};
        if sections.len() < 2 {
            return Err("wing_from_sections: need at least 2 sections".into());
        }
        let parse_row = |v: &rhai::Dynamic| -> Option<(f32, f32, String)> {
            let arr = v.clone().try_cast::<rhai::Array>()?;
            let chord = arr.get(1)?.as_float().ok()? as f32;
            let naca = arr.get(2)?.clone().into_string().ok()?;
            Some((0.0, chord, naca))
        };
        let root_row = parse_row(&sections[0])
            .ok_or_else(|| -> Box<rhai::EvalAltResult> { "wing_from_sections: invalid root section".into() })?;
        let tip_row = parse_row(&sections[sections.len() - 1])
            .ok_or_else(|| -> Box<rhai::EvalAltResult> { "wing_from_sections: invalid tip section".into() })?;

        let tip_y = sections[sections.len() - 1]
            .clone()
            .try_cast::<rhai::Array>()
            .and_then(|arr| arr.get(0).and_then(|v| v.as_float().ok()).map(|y| y as f32))
            .unwrap_or(400.0);
        let sweep_off = sections[sections.len() - 1]
            .clone()
            .try_cast::<rhai::Array>()
            .and_then(|arr| arr.get(4).and_then(|v| v.as_float().ok()).map(|s| s as f32))
            .unwrap_or(0.0);
        let sweep_deg = if tip_y > 0.0 { (sweep_off / tip_y).atan().to_degrees() } else { 0.0 };

        let root_sec = get_naca_airfoil(&root_row.2, root_row.1) as Arc<dyn crate::sdf::aerospace::Section2D>;
        let tip_sec = get_naca_airfoil(&tip_row.2, tip_row.1) as Arc<dyn crate::sdf::aerospace::Section2D>;
        let wing = core_wfs(root_sec, tip_sec, tip_y, sweep_deg, 0.0, 0.0);
        Ok(SdfHandle(Arc::new(wing)))
    });
}

pub fn register_legacy_fea_compat_functions(
    engine: &mut Engine,
    collector: Arc<Mutex<crate::fea::FEASetup>>,
) {
    use crate::fea::{FEAForceRegion, FEAMotorRegion, FEAPressureRegion, FEARegion, FEATorqueRegion};
    use crate::sdf::primitives::{SdfBox, Sphere};
    use crate::sdf::transforms::Translate;

    {
        let c: Arc<Mutex<crate::fea::FEASetup>> = Arc::clone(&collector);
        engine.register_fn(
            "fea_fixed_face",
            move |body: SdfHandle, axis: &str, pos: f64| -> Result<SdfHandle, Box<rhai::EvalAltResult>> {
                let region: Arc<dyn Sdf> = match axis.to_ascii_lowercase().as_str() {
                    "x" => Arc::new(Intersect::new(
                        Arc::clone(&body.0),
                        Arc::new(Translate::new(
                            Arc::new(SdfBox::new(Vec3::new(1.0, 10_000.0, 10_000.0))),
                            Vec3::new(pos as f32, 0.0, 0.0),
                        )),
                    )),
                    "y" => Arc::new(Intersect::new(
                        Arc::clone(&body.0),
                        Arc::new(Translate::new(
                            Arc::new(SdfBox::new(Vec3::new(10_000.0, 1.0, 10_000.0))),
                            Vec3::new(0.0, pos as f32, 0.0),
                        )),
                    )),
                    "z" => Arc::new(Intersect::new(
                        Arc::clone(&body.0),
                        Arc::new(Translate::new(
                            Arc::new(SdfBox::new(Vec3::new(10_000.0, 10_000.0, 1.0))),
                            Vec3::new(0.0, 0.0, pos as f32),
                        )),
                    )),
                    other => return Err(format!("Invalid axis '{}': expected x, y, or z", other).into()),
                };
                c.lock().unwrap().fixed_supports.push(FEARegion {
                    name: format!("fixed_face_{}_{}", axis, pos),
                    sdf: region,
                });
                Ok(body)
            },
        );
    }

    {
        let c: Arc<Mutex<crate::fea::FEASetup>> = Arc::clone(&collector);
        engine.register_fn("fea_gravity", move |body: SdfHandle| -> SdfHandle {
            c.lock().unwrap().gravity = Some(Vec3::new(0.0, 0.0, -9.81));
            body
        });
    }

    {
        let c: Arc<Mutex<crate::fea::FEASetup>> = Arc::clone(&collector);
        engine.register_fn(
            "fea_load_point",
            move |x: f64, y: f64, z: f64, fx: f64, fy: f64, fz: f64| -> i64 {
                let region: Arc<dyn Sdf> = Arc::new(Translate::new(
                    Arc::new(Sphere::new(1.5)),
                    Vec3::new(x as f32, y as f32, z as f32),
                ));
                c.lock().unwrap().force_loads.push(FEAForceRegion {
                    name: format!("point_load_{:.1}_{:.1}_{:.1}", x, y, z),
                    sdf: region,
                    force: Vec3::new(fx as f32, fy as f32, fz as f32),
                });
                1
            },
        );
    }

    {
        let c: Arc<Mutex<crate::fea::FEASetup>> = Arc::clone(&collector);
        engine.register_fn(
            "fea_pressure",
            move |body: SdfHandle, axis: &str, pos: f64, pressure: f64| -> Result<SdfHandle, Box<rhai::EvalAltResult>> {
                let region: Arc<dyn Sdf> = match axis.to_ascii_lowercase().as_str() {
                    "x" => Arc::new(Intersect::new(
                        Arc::clone(&body.0),
                        Arc::new(Translate::new(
                            Arc::new(SdfBox::new(Vec3::new(1.0, 10_000.0, 10_000.0))),
                            Vec3::new(pos as f32, 0.0, 0.0),
                        )),
                    )),
                    "y" => Arc::new(Intersect::new(
                        Arc::clone(&body.0),
                        Arc::new(Translate::new(
                            Arc::new(SdfBox::new(Vec3::new(10_000.0, 1.0, 10_000.0))),
                            Vec3::new(0.0, pos as f32, 0.0),
                        )),
                    )),
                    "z" => Arc::new(Intersect::new(
                        Arc::clone(&body.0),
                        Arc::new(Translate::new(
                            Arc::new(SdfBox::new(Vec3::new(10_000.0, 10_000.0, 1.0))),
                            Vec3::new(0.0, 0.0, pos as f32),
                        )),
                    )),
                    other => return Err(format!("Invalid axis '{}': expected x, y, or z", other).into()),
                };
                c.lock().unwrap().pressure_loads.push(FEAPressureRegion {
                    name: format!("pressure_{}_{}", axis, pos),
                    sdf: region,
                    magnitude: pressure as f32,
                });
                Ok(body)
            },
        );
    }

    {
        let c: Arc<Mutex<crate::fea::FEASetup>> = Arc::clone(&collector);
        engine.register_fn(
            "fea_torque",
            move |region: SdfHandle, name: &str, ax: f64, ay: f64, az: f64, magnitude: f64| {
                c.lock().unwrap().torque_loads.push(FEATorqueRegion {
                    name: name.to_string(),
                    sdf: region.0,
                    axis: Vec3::new(ax as f32, ay as f32, az as f32).normalize_or_zero(),
                    magnitude: magnitude as f32,
                });
            },
        );
    }

    {
        let c: Arc<Mutex<crate::fea::FEASetup>> = Arc::clone(&collector);
        engine.register_fn(
            "fea_motor_thrust",
            move |region: SdfHandle, name: &str, thrust_n: f64, torque_nmm: f64, dx: f64, dy: f64, dz: f64| {
                c.lock().unwrap().motor_thrusts.push(FEAMotorRegion {
                    name: name.to_string(),
                    sdf: region.0,
                    thrust_n: thrust_n as f32,
                    torque_nmm: torque_nmm as f32,
                    direction: Vec3::new(dx as f32, dy as f32, dz as f32).normalize_or_zero(),
                });
            },
        );
    }
}
