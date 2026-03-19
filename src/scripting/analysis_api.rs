use std::sync::Arc;

use glam::Vec3;
use rhai::Engine;

use crate::sdf::Sdf;

use super::{
    DragPolarHandle, FlightConditionHandle, MotorHandle, PolarHandle, PropHandle,
    PropulsionHandle, SdfHandle, StabilityResultHandle, TrimResultHandle,
};

pub fn register_aero_functions(engine: &mut Engine) {
    use crate::aero::{FlightCondition, PolarDatabase, solve_lifting_line};

    engine.register_type::<PolarHandle>();
    engine.register_type::<FlightConditionHandle>();
    engine.register_type::<StabilityResultHandle>();
    engine.register_type::<TrimResultHandle>();
    engine.register_type::<DragPolarHandle>();

    engine.register_fn("get_polar", |designation: &str| -> PolarHandle {
        let db = PolarDatabase::new();
        let polar = db
            .get_interpolated(designation, 500_000.0)
            .or_else(|| db.get_interpolated("NACA 0012", 500_000.0))
            .expect("NACA 0012 must exist");
        PolarHandle(Arc::new(polar))
    });

    engine.register_fn("get_polar_re", |designation: &str, reynolds: f64| -> PolarHandle {
        let db = PolarDatabase::new();
        let polar = db
            .get_interpolated(designation, reynolds as f32)
            .or_else(|| db.get_interpolated("NACA 0012", reynolds as f32))
            .expect("NACA 0012 must exist");
        PolarHandle(Arc::new(polar))
    });

    engine.register_fn("cl_at", |p: PolarHandle, alpha: f64| -> f64 { p.0.cl_at(alpha as f32) as f64 });
    engine.register_fn("cd_at", |p: PolarHandle, alpha: f64| -> f64 { p.0.cd_at(alpha as f32) as f64 });
    engine.register_fn("cl_alpha", |p: PolarHandle| -> f64 { p.0.cl_alpha as f64 });
    engine.register_fn("cl_max", |p: PolarHandle| -> f64 { p.0.cl_max as f64 });
    engine.register_fn("alpha_stall", |p: PolarHandle| -> f64 { p.0.alpha_stall_deg as f64 });

    engine.register_fn("flight_condition", |airspeed: f64, altitude: f64, aoa: f64| -> FlightConditionHandle {
        FlightConditionHandle(FlightCondition::new(airspeed as f32, altitude as f32, aoa as f32))
    });
    engine.register_fn("flight_condition_sl", |airspeed: f64, aoa: f64| -> FlightConditionHandle {
        FlightConditionHandle(FlightCondition::new(airspeed as f32, 0.0, aoa as f32))
    });
    engine.register_fn("dynamic_pressure", |fc: FlightConditionHandle| -> f64 { fc.0.dynamic_pressure_pa as f64 });
    engine.register_fn("reynolds", |fc: FlightConditionHandle, chord_mm: f64| -> f64 {
        fc.0.reynolds_for_chord(chord_mm as f32) as f64
    });

    engine.register_fn("run_lifting_line", |wing: SdfHandle, fc: FlightConditionHandle| -> rhai::Map {
        let db = PolarDatabase::new();
        let res = solve_lifting_line(&wing.0, &db, &fc.0, 20);
        let mut map = rhai::Map::new();
        map.insert("cl".into(), rhai::Dynamic::from(res.cl_total as f64));
        map.insert("cd_induced".into(), rhai::Dynamic::from(res.cd_induced as f64));
        map.insert("efficiency".into(), rhai::Dynamic::from(res.oswald_efficiency as f64));
        map.insert("lift_n".into(), rhai::Dynamic::from(res.lift_total_n as f64));
        map.insert("drag_n".into(), rhai::Dynamic::from(res.induced_drag_total_n as f64));
        map.insert("tip_stall".into(), rhai::Dynamic::from(res.tip_stall_risk));
        map.insert("root_stall_first".into(), rhai::Dynamic::from(res.root_stall_first));
        map
    });

    engine.register_fn(
        "run_lifting_line_polar",
        |wing: SdfHandle,
         fc: FlightConditionHandle,
         alpha_start: f64,
         alpha_end: f64,
         alpha_step: f64|
         -> rhai::Array {
            let db = PolarDatabase::new();
            let step = alpha_step.max(0.1) as f32;
            let mut alpha = alpha_start as f32;
            let mut results = rhai::Array::new();
            while alpha <= alpha_end as f32 + 1e-4 {
                let mut fc_i = fc.0.clone();
                fc_i.aoa_deg = alpha;
                let res = solve_lifting_line(&wing.0, &db, &fc_i, 20);
                let mut map = rhai::Map::new();
                map.insert("alpha".into(), rhai::Dynamic::from(alpha as f64));
                map.insert("cl".into(), rhai::Dynamic::from(res.cl_total as f64));
                map.insert("cd_induced".into(), rhai::Dynamic::from(res.cd_induced as f64));
                map.insert("efficiency".into(), rhai::Dynamic::from(res.oswald_efficiency as f64));
                map.insert("lift_n".into(), rhai::Dynamic::from(res.lift_total_n as f64));
                results.push(rhai::Dynamic::from(map));
                alpha += step;
            }
            results
        },
    );

    engine.register_fn(
        "neutral_point",
        |wing: SdfHandle, htail: SdfHandle, fuse: SdfHandle, fc: FlightConditionHandle| -> f64 {
            use crate::aero::{PolarDatabase, compute_neutral_point};
            let db = PolarDatabase::new();
            let np = compute_neutral_point(&wing.0, &htail.0, &fuse.0, &db, &fc.0);
            np.neutral_point_x_mm as f64
        },
    );

    engine.register_fn(
        "static_margin",
        |wing: SdfHandle,
         htail: SdfHandle,
         fuse: SdfHandle,
         fc: FlightConditionHandle,
         cg_x_mm: f64|
         -> rhai::Map {
            use crate::aero::{PolarDatabase, compute_neutral_point, compute_static_margin};
            let db = PolarDatabase::new();
            let np = compute_neutral_point(&wing.0, &htail.0, &fuse.0, &db, &fc.0);
            let wing_bbox = crate::sdf::query::bounding_points(wing.0.as_ref());
            let root_chord = wing_bbox.size.x;
            let mac = (root_chord + root_chord * 0.5) * 0.5;
            let cg = Vec3::new(cg_x_mm as f32, 0.0, 0.0);
            let sm = compute_static_margin(&np, cg, mac);
            let mut map = rhai::Map::new();
            map.insert("neutral_point_x_mm".into(), rhai::Dynamic::from(sm.neutral_point_x_mm as f64));
            map.insert("cg_x_mm".into(), rhai::Dynamic::from(sm.cg_x_mm as f64));
            map.insert("static_margin_mm".into(), rhai::Dynamic::from(sm.static_margin_mm as f64));
            map.insert("static_margin_mac".into(), rhai::Dynamic::from(sm.static_margin_mac as f64));
            map.insert("is_stable".into(), rhai::Dynamic::from(sm.is_stable));
            map.insert("stability_category".into(), rhai::Dynamic::from(sm.stability_category.to_string()));
            map.insert("cg_forward_limit_mm".into(), rhai::Dynamic::from(sm.cg_forward_limit_mm as f64));
            map.insert("cg_aft_limit_mm".into(), rhai::Dynamic::from(sm.cg_aft_limit_mm as f64));
            map.insert("cg_range_mm".into(), rhai::Dynamic::from(sm.cg_range_mm as f64));
            map.insert("pitch_stiffness".into(), rhai::Dynamic::from(sm.pitch_stiffness as f64));
            map
        },
    );

    engine.register_fn(
        "required_cg_range",
        |wing: SdfHandle, htail: SdfHandle, fuse: SdfHandle, fc: FlightConditionHandle| -> rhai::Map {
            use crate::aero::{PolarDatabase, compute_neutral_point, compute_static_margin};
            let db = PolarDatabase::new();
            let np = compute_neutral_point(&wing.0, &htail.0, &fuse.0, &db, &fc.0);
            let wing_bbox = crate::sdf::query::bounding_points(wing.0.as_ref());
            let root_chord = wing_bbox.size.x;
            let mac = (root_chord + root_chord * 0.5) * 0.5;
            let cg = Vec3::new(np.neutral_point_x_mm, 0.0, 0.0);
            let sm = compute_static_margin(&np, cg, mac);
            let mut map = rhai::Map::new();
            map.insert("forward_limit_mm".into(), rhai::Dynamic::from(sm.cg_forward_limit_mm as f64));
            map.insert("aft_limit_mm".into(), rhai::Dynamic::from(sm.cg_aft_limit_mm as f64));
            map.insert("range_mm".into(), rhai::Dynamic::from(sm.cg_range_mm as f64));
            map.insert("neutral_point_mm".into(), rhai::Dynamic::from(np.neutral_point_x_mm as f64));
            map.insert("mac_mm".into(), rhai::Dynamic::from(mac as f64));
            map
        },
    );

    engine.register_fn(
        "trim_analysis",
        |wing: SdfHandle,
         htail: SdfHandle,
         fuse: SdfHandle,
         fc: FlightConditionHandle,
         weight_n: f64|
         -> rhai::Map {
            use crate::aero::{PolarDatabase, compute_neutral_point, compute_static_margin, compute_trim};
            let db = PolarDatabase::new();
            let np = compute_neutral_point(&wing.0, &htail.0, &fuse.0, &db, &fc.0);
            let wing_bbox = crate::sdf::query::bounding_points(wing.0.as_ref());
            let root_chord = wing_bbox.size.x;
            let mac = (root_chord + root_chord * 0.5) * 0.5;
            let cg = Vec3::new(np.neutral_point_x_mm - 0.10 * mac, 0.0, 0.0);
            let sm = compute_static_margin(&np, cg, mac);
            let trim = compute_trim(&np, &sm, &wing.0, &htail.0, &fuse.0, &db, &fc.0, weight_n as f32);
            let mut map = rhai::Map::new();
            map.insert("trim_aoa_deg".into(), rhai::Dynamic::from(trim.trim_aoa_deg as f64));
            map.insert("trim_cl".into(), rhai::Dynamic::from(trim.trim_cl as f64));
            map.insert("trim_airspeed_ms".into(), rhai::Dynamic::from(trim.trim_airspeed_ms as f64));
            map.insert("elevator_deflection_deg".into(), rhai::Dynamic::from(trim.elevator_deflection_deg as f64));
            map.insert("is_trimmed".into(), rhai::Dynamic::from(trim.is_trimmed));
            map.insert("trim_margin_deg".into(), rhai::Dynamic::from(trim.trim_margin_deg as f64));
            map
        },
    );

    engine.register_fn(
        "drag_polar",
        |wing: SdfHandle, fuse: SdfHandle, htail: SdfHandle, vtail: SdfHandle, fc: FlightConditionHandle| -> rhai::Map {
            use crate::aero::{PolarDatabase, compute_drag_polar};
            let db = PolarDatabase::new();
            let result = compute_drag_polar(&wing.0, &fuse.0, &htail.0, &vtail.0, &db, &fc.0, None);
            let mut map = rhai::Map::new();
            map.insert("cd0".into(), rhai::Dynamic::from(result.cd0 as f64));
            map.insert("k".into(), rhai::Dynamic::from(result.k as f64));
            map.insert("cl_best_ld".into(), rhai::Dynamic::from(result.cl_best_ld as f64));
            map.insert("ld_max".into(), rhai::Dynamic::from(result.ld_max as f64));
            map.insert("cd0_wing".into(), rhai::Dynamic::from(result.cd0_breakdown.wing as f64));
            map.insert("cd0_fuselage".into(), rhai::Dynamic::from(result.cd0_breakdown.fuselage as f64));
            map.insert("cd0_htail".into(), rhai::Dynamic::from(result.cd0_breakdown.h_tail as f64));
            map.insert("cd0_vtail".into(), rhai::Dynamic::from(result.cd0_breakdown.v_tail as f64));
            map.insert("best_glide_ms".into(), rhai::Dynamic::from(result.best_glide_airspeed_ms as f64));
            map.insert("best_endurance_ms".into(), rhai::Dynamic::from(result.best_endurance_airspeed_ms as f64));
            map
        },
    );

    engine.register_fn(
        "drag_polar_weighted",
        |wing: SdfHandle,
         fuse: SdfHandle,
         htail: SdfHandle,
         vtail: SdfHandle,
         fc: FlightConditionHandle,
         weight_n: f64|
         -> rhai::Map {
            use crate::aero::{PolarDatabase, compute_drag_polar};
            let db = PolarDatabase::new();
            let result = compute_drag_polar(&wing.0, &fuse.0, &htail.0, &vtail.0, &db, &fc.0, Some(weight_n as f32));
            let mut map = rhai::Map::new();
            map.insert("cd0".into(), rhai::Dynamic::from(result.cd0 as f64));
            map.insert("k".into(), rhai::Dynamic::from(result.k as f64));
            map.insert("cl_best_ld".into(), rhai::Dynamic::from(result.cl_best_ld as f64));
            map.insert("ld_max".into(), rhai::Dynamic::from(result.ld_max as f64));
            map.insert("cd0_wing".into(), rhai::Dynamic::from(result.cd0_breakdown.wing as f64));
            map.insert("cd0_fuselage".into(), rhai::Dynamic::from(result.cd0_breakdown.fuselage as f64));
            map.insert("cd0_htail".into(), rhai::Dynamic::from(result.cd0_breakdown.h_tail as f64));
            map.insert("cd0_vtail".into(), rhai::Dynamic::from(result.cd0_breakdown.v_tail as f64));
            map.insert("best_glide_ms".into(), rhai::Dynamic::from(result.best_glide_airspeed_ms as f64));
            map.insert("best_endurance_ms".into(), rhai::Dynamic::from(result.best_endurance_airspeed_ms as f64));
            map
        },
    );

    engine.register_fn(
        "ld_max",
        |wing: SdfHandle, fuse: SdfHandle, htail: SdfHandle, vtail: SdfHandle, fc: FlightConditionHandle| -> f64 {
            use crate::aero::{PolarDatabase, compute_drag_polar};
            let db = PolarDatabase::new();
            let result = compute_drag_polar(&wing.0, &fuse.0, &htail.0, &vtail.0, &db, &fc.0, None);
            result.ld_max as f64
        },
    );

    engine.register_fn(
        "best_glide_speed",
        |wing: SdfHandle,
         fuse: SdfHandle,
         htail: SdfHandle,
         vtail: SdfHandle,
         fc: FlightConditionHandle,
         weight_n: f64|
         -> f64 {
            use crate::aero::{PolarDatabase, compute_drag_polar};
            let db = PolarDatabase::new();
            let result = compute_drag_polar(&wing.0, &fuse.0, &htail.0, &vtail.0, &db, &fc.0, Some(weight_n as f32));
            result.best_glide_airspeed_ms as f64
        },
    );
}

pub fn register_analysis_functions(engine: &mut Engine) {
    use crate::geometry_analysis::cg_sensitivity::compute_cg_sensitivity;
    use crate::geometry_analysis::interference::check_assembly_interference;
    engine.register_fn("cg_sensitivity", |comps: rhai::Array, np_x: f64, mac: f64| -> rhai::Dynamic {
        let components: Vec<(String, Vec3, f32)> = comps
            .iter()
            .filter_map(|c| {
                let m = c.clone().try_cast::<rhai::Map>()?;
                let name = m.get("name")?.clone().into_string().ok()?;
                let x = m.get("x")?.as_float().ok()? as f32;
                let y = m.get("y")?.as_float().ok()? as f32;
                let z = m.get("z")?.as_float().ok()? as f32;
                let mass = m.get("mass_g")?.as_float().ok()? as f32;
                Some((name, Vec3::new(x, y, z), mass))
            })
            .collect();
        let dimensions = indexmap::IndexMap::new();
        let forward_limit_x = np_x as f32 - 0.25 * mac as f32;
        let result = compute_cg_sensitivity(&components, &dimensions, np_x as f32, mac as f32, forward_limit_x);
        let mut map = rhai::Map::new();
        map.insert("cg_x_mm".into(), rhai::Dynamic::from(result.baseline_cg.x as f64));
        map.insert("cg_y_mm".into(), rhai::Dynamic::from(result.baseline_cg.y as f64));
        map.insert("cg_z_mm".into(), rhai::Dynamic::from(result.baseline_cg.z as f64));
        map.insert("total_mass_g".into(), rhai::Dynamic::from(components.iter().map(|(_, _, mass)| *mass).sum::<f32>() as f64));
        map.insert("static_margin_frac".into(), rhai::Dynamic::from(result.baseline_static_margin_mac as f64));
        let recs: rhai::Array = result
            .recommendations
            .iter()
            .map(|r| rhai::Dynamic::from(r.clone()))
            .collect();
        map.insert("recommendations".into(), rhai::Dynamic::from(recs));
        let comp_arr: rhai::Array = result
            .component_sensitivities
            .iter()
            .map(|c| {
                let mut cm = rhai::Map::new();
                cm.insert("name".into(), rhai::Dynamic::from(c.component_name.clone()));
                cm.insert("mass_g".into(), rhai::Dynamic::from(c.component_mass_g as f64));
                cm.insert("position_x_mm".into(), rhai::Dynamic::from(c.current_position.x as f64));
                cm.insert("position_y_mm".into(), rhai::Dynamic::from(c.current_position.y as f64));
                cm.insert("position_z_mm".into(), rhai::Dynamic::from(c.current_position.z as f64));
                cm.insert("dcg_dx".into(), rhai::Dynamic::from(c.dcg_dx_mm_per_mm as f64));
                cm.insert("dcg_dy".into(), rhai::Dynamic::from(c.dcg_dy_mm_per_mm as f64));
                cm.insert("dcg_dz".into(), rhai::Dynamic::from(c.dcg_dz_mm_per_mm as f64));
                cm.insert("influence_fraction".into(), rhai::Dynamic::from(c.influence_fraction as f64));
                cm.insert("forward_limit_mm".into(), rhai::Dynamic::from(c.forward_limit_mm as f64));
                cm.insert("aft_limit_mm".into(), rhai::Dynamic::from(c.aft_limit_mm as f64));
                rhai::Dynamic::from(cm)
            })
            .collect();
        map.insert("component_sensitivities".into(), rhai::Dynamic::from(comp_arr));
        rhai::Dynamic::from(map)
    });

    engine.register_fn("interference_check", |names: rhai::Array, keepouts: rhai::Array, parent: SdfHandle| -> rhai::Dynamic {
        let components: Vec<(String, Arc<dyn Sdf>, Arc<dyn Sdf>)> = names
            .iter()
            .zip(keepouts.iter())
            .filter_map(|(n, k)| {
                let name = n.clone().into_string().ok()?;
                let keepout = k.clone().try_cast::<SdfHandle>()?.0;
                Some((name, keepout.clone(), keepout))
            })
            .collect();
        let result = check_assembly_interference(&components, Some(parent.0), 12);
        build_interference_map(result)
    });

    engine.register_fn("interference_check_no_parent", |names: rhai::Array, keepouts: rhai::Array| -> rhai::Dynamic {
        let components: Vec<(String, Arc<dyn Sdf>, Arc<dyn Sdf>)> = names
            .iter()
            .zip(keepouts.iter())
            .filter_map(|(n, k)| {
                let name = n.clone().into_string().ok()?;
                let keepout = k.clone().try_cast::<SdfHandle>()?.0;
                Some((name, keepout.clone(), keepout))
            })
            .collect();
        let result = check_assembly_interference(&components, None, 12);
        build_interference_map(result)
    });
}

fn build_interference_map(result: crate::geometry_analysis::InterferenceResult) -> rhai::Dynamic {
    let mut map = rhai::Map::new();
    map.insert("total_interference_count".into(), rhai::Dynamic::from(result.total_interference_count as i64));
    map.insert("has_critical_interference".into(), rhai::Dynamic::from(result.has_critical_interference));
    let pairs_arr: rhai::Array = result
        .pairs
        .iter()
        .map(|p| {
            let mut pm = rhai::Map::new();
            pm.insert("component_a".into(), rhai::Dynamic::from(p.component_a.clone()));
            pm.insert("component_b".into(), rhai::Dynamic::from(p.component_b.clone()));
            pm.insert("volume_mm3".into(), rhai::Dynamic::from(p.interference_volume_mm3 as f64));
            pm.insert("severity".into(), rhai::Dynamic::from(format!("{:?}", p.severity)));
            pm.insert("description".into(), rhai::Dynamic::from(p.description.clone()));
            rhai::Dynamic::from(pm)
        })
        .collect();
    map.insert("pairs".into(), rhai::Dynamic::from(pairs_arr));
    let outside: rhai::Array = result
        .outside_parent
        .iter()
        .map(|n| rhai::Dynamic::from(n.clone()))
        .collect();
    map.insert("outside_parent".into(), rhai::Dynamic::from(outside));
    rhai::Dynamic::from(map)
}

pub fn register_propulsion_functions(engine: &mut Engine) {
    engine.register_type::<MotorHandle>();
    engine.register_type::<PropHandle>();
    engine.register_type::<PropulsionHandle>();

    engine.register_fn("motor", |name: &str| -> Result<MotorHandle, Box<rhai::EvalAltResult>> {
        use crate::aero::PropulsionDatabase;
        let db = PropulsionDatabase::new();
        db.find_motor(name)
            .map(|m| MotorHandle(Arc::new(m.clone())))
            .ok_or_else(|| format!("Motor '{}' not found in database", name).into())
    });

    engine.register_fn("motor_custom", |kv: f64, max_power: f64, max_current: f64, weight: f64, ri: f64| {
        use crate::aero::MotorSpec;
        MotorHandle(Arc::new(MotorSpec {
            name: "Custom".to_string(),
            kv_rpm_per_volt: kv as f32,
            max_power_w: max_power as f32,
            max_current_a: max_current as f32,
            weight_g: weight as f32,
            stator_diameter_mm: 0.0,
            stator_height_mm: 0.0,
            internal_resistance_ohm: ri as f32,
        }))
    });

    engine.register_fn("prop", |name: &str| -> Result<PropHandle, Box<rhai::EvalAltResult>> {
        use crate::aero::PropulsionDatabase;
        let db = PropulsionDatabase::new();
        db.find_prop_by_name(name)
            .map(|p| PropHandle(Arc::new(p.clone())))
            .ok_or_else(|| format!("Prop '{}' not found", name).into())
    });

    engine.register_fn("prop_by_size", |diam: f64, pitch: f64| -> Result<PropHandle, Box<rhai::EvalAltResult>> {
        use crate::aero::PropulsionDatabase;
        let db = PropulsionDatabase::new();
        db.find_prop(diam as f32, pitch as f32)
            .map(|p| PropHandle(Arc::new(p.clone())))
            .ok_or_else(|| "No matching prop found".into())
    });

    engine.register_fn("prop_custom", |diam: f64, pitch: f64, ct: rhai::Array, cp: rhai::Array, j_max: f64| {
        use crate::aero::PropSpec;
        let ct_arr: [f32; 5] = [
            ct.get(0).and_then(|v| v.as_float().ok()).unwrap_or(0.0) as f32,
            ct.get(1).and_then(|v| v.as_float().ok()).unwrap_or(0.0) as f32,
            ct.get(2).and_then(|v| v.as_float().ok()).unwrap_or(0.0) as f32,
            ct.get(3).and_then(|v| v.as_float().ok()).unwrap_or(0.0) as f32,
            ct.get(4).and_then(|v| v.as_float().ok()).unwrap_or(0.0) as f32,
        ];
        let cp_arr: [f32; 5] = [
            cp.get(0).and_then(|v| v.as_float().ok()).unwrap_or(0.001) as f32,
            cp.get(1).and_then(|v| v.as_float().ok()).unwrap_or(0.0) as f32,
            cp.get(2).and_then(|v| v.as_float().ok()).unwrap_or(0.0) as f32,
            cp.get(3).and_then(|v| v.as_float().ok()).unwrap_or(0.0) as f32,
            cp.get(4).and_then(|v| v.as_float().ok()).unwrap_or(0.0) as f32,
        ];
        PropHandle(Arc::new(PropSpec {
            name: format!("Custom {:.0}x{:.0}", diam, pitch / 25.4),
            diameter_mm: diam as f32,
            pitch_mm: pitch as f32,
            blades: 2,
            ct_coeffs: ct_arr,
            cp_coeffs: cp_arr,
            j_max: j_max as f32,
        }))
    });

    engine.register_fn("list_motors", || -> rhai::Array {
        use crate::aero::PropulsionDatabase;
        PropulsionDatabase::new().motors.iter().map(|m| rhai::Dynamic::from(m.name.clone())).collect()
    });

    engine.register_fn("list_props", || -> rhai::Array {
        use crate::aero::PropulsionDatabase;
        PropulsionDatabase::new().props.iter().map(|p| rhai::Dynamic::from(p.name.clone())).collect()
    });

    engine.register_fn("propulsion_setup", |m: MotorHandle, p: PropHandle, cells: i64, cap: f64| {
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

    engine.register_fn("propulsion_setup_full", |m: MotorHandle, p: PropHandle, cells: i64, cap: f64, c: f64, em: f64, ee: f64| {
        use crate::aero::PropulsionSetup;
        PropulsionHandle(Arc::new(PropulsionSetup {
            motor: (*m.0).clone(),
            prop: (*p.0).clone(),
            battery_cells: cells as u32,
            battery_capacity_mah: cap as f32,
            battery_c_rating: c as f32,
            motor_count: 1,
            efficiency_motor: em as f32,
            efficiency_esc: ee as f32,
        }))
    });

    engine.register_fn("propulsion_analysis", |setup: PropulsionHandle, fc: FlightConditionHandle, weight_n: f64| -> rhai::Map {
        use crate::aero::compute_propulsion;
        let result = compute_propulsion(&setup.0, &fc.0, weight_n as f32, None);
        let mut map = rhai::Map::new();
        map.insert("static_thrust_n".into(), rhai::Dynamic::from(result.static_thrust_n as f64));
        map.insert("thrust_to_weight".into(), rhai::Dynamic::from(result.thrust_to_weight as f64));
        map.insert("max_airspeed_ms".into(), rhai::Dynamic::from(result.max_airspeed_ms as f64));
        map.insert("max_airspeed_kmh".into(), rhai::Dynamic::from(result.max_airspeed_kmh as f64));
        map.insert("prop_tip_mach".into(), rhai::Dynamic::from(result.prop_tip_mach as f64));
        map.insert("static_current_a".into(), rhai::Dynamic::from(result.static_current_a as f64));
        map.insert("static_power_w".into(), rhai::Dynamic::from(result.static_power_input_w as f64));
        map.insert("within_motor_limits".into(), rhai::Dynamic::from(result.within_motor_limits));
        map.insert("within_battery_limits".into(), rhai::Dynamic::from(result.within_battery_limits));
        let warnings: rhai::Array = result.warnings.iter().map(|w| rhai::Dynamic::from(w.clone())).collect();
        map.insert("warnings".into(), rhai::Dynamic::from(warnings));
        map
    });

    engine.register_fn("propulsion_thrust_at", |setup: PropulsionHandle, airspeed_ms: f64, fc: FlightConditionHandle| -> f64 {
        use crate::aero::compute_propulsion;
        let result = compute_propulsion(&setup.0, &fc.0, 10.0, None);
        let v = airspeed_ms as f32;
        for i in 1..result.thrust_curve.len() {
            let (v0, t0) = result.thrust_curve[i - 1];
            let (v1, t1) = result.thrust_curve[i];
            if v <= v1 {
                let t = (v - v0) / (v1 - v0).max(1e-6);
                return ((1.0 - t) * t0 + t * t1) as f64;
            }
        }
        0.0
    });

    engine.register_fn(
        "range_endurance",
        |setup: PropulsionHandle,
         wing: SdfHandle,
         fuse: SdfHandle,
         htail: SdfHandle,
         vtail: SdfHandle,
         fc: FlightConditionHandle,
         weight_n: f64|
         -> rhai::Map {
            use crate::aero::{PolarDatabase, compute_drag_polar, compute_propulsion, compute_range_endurance};
            let db = PolarDatabase::new();
            let drag = compute_drag_polar(&wing.0, &fuse.0, &htail.0, &vtail.0, &db, &fc.0, Some(weight_n as f32));
            let prop = compute_propulsion(&setup.0, &fc.0, weight_n as f32, Some(&drag));
            let result = compute_range_endurance(&setup.0, &prop, &drag, &fc.0, weight_n as f32);
            let mut map = rhai::Map::new();
            map.insert("max_endurance_min".into(), rhai::Dynamic::from(result.max_endurance_min as f64));
            map.insert("max_endurance_airspeed_ms".into(), rhai::Dynamic::from(result.max_endurance_airspeed_ms as f64));
            map.insert("max_range_km".into(), rhai::Dynamic::from(result.max_range_km as f64));
            map.insert("max_range_airspeed_ms".into(), rhai::Dynamic::from(result.max_range_airspeed_ms as f64));
            map.insert("cruise_endurance_min".into(), rhai::Dynamic::from(result.cruise_endurance_min as f64));
            map.insert("cruise_range_km".into(), rhai::Dynamic::from(result.cruise_range_km as f64));
            map.insert("battery_energy_wh".into(), rhai::Dynamic::from(result.battery_energy_wh as f64));
            map.insert("specific_energy_wh_per_km".into(), rhai::Dynamic::from(result.specific_energy_wh_per_km as f64));
            map
        },
    );

    engine.register_fn(
        "rate_of_climb",
        |setup: PropulsionHandle,
         wing: SdfHandle,
         fuse: SdfHandle,
         htail: SdfHandle,
         vtail: SdfHandle,
         fc: FlightConditionHandle,
         weight_n: f64|
         -> rhai::Map {
            use crate::aero::{PolarDatabase, compute_drag_polar, compute_propulsion, compute_rate_of_climb};
            let db = PolarDatabase::new();
            let drag = compute_drag_polar(&wing.0, &fuse.0, &htail.0, &vtail.0, &db, &fc.0, Some(weight_n as f32));
            let prop = compute_propulsion(&setup.0, &fc.0, weight_n as f32, Some(&drag));
            let result = compute_rate_of_climb(&prop, &drag, &fc.0, weight_n as f32);
            let mut map = rhai::Map::new();
            map.insert("max_roc_ms".into(), rhai::Dynamic::from(result.max_roc_ms as f64));
            map.insert("max_roc_fpm".into(), rhai::Dynamic::from(result.max_roc_fpm as f64));
            map.insert("best_climb_airspeed_ms".into(), rhai::Dynamic::from(result.best_climb_airspeed_ms as f64));
            map.insert("climb_angle_deg".into(), rhai::Dynamic::from(result.climb_angle_deg as f64));
            map.insert("service_ceiling_m".into(), rhai::Dynamic::from(result.service_ceiling_m as f64));
            map
        },
    );

    engine.register_fn(
        "glide_performance",
        |wing: SdfHandle,
         fuse: SdfHandle,
         htail: SdfHandle,
         vtail: SdfHandle,
         fc: FlightConditionHandle,
         weight_n: f64|
         -> rhai::Map {
            use crate::aero::{PolarDatabase, compute_drag_polar, compute_glide};
            let db = PolarDatabase::new();
            let drag = compute_drag_polar(&wing.0, &fuse.0, &htail.0, &vtail.0, &db, &fc.0, Some(weight_n as f32));
            let result = compute_glide(&drag, &fc.0, weight_n as f32);
            let mut map = rhai::Map::new();
            map.insert("best_glide_ratio".into(), rhai::Dynamic::from(result.best_glide_ratio as f64));
            map.insert("best_glide_airspeed_ms".into(), rhai::Dynamic::from(result.best_glide_airspeed_ms as f64));
            map.insert("best_glide_sink_rate_ms".into(), rhai::Dynamic::from(result.best_glide_sink_rate_ms as f64));
            map.insert("min_sink_rate_ms".into(), rhai::Dynamic::from(result.min_sink_rate_ms as f64));
            map.insert("min_sink_airspeed_ms".into(), rhai::Dynamic::from(result.min_sink_airspeed_ms as f64));
            map.insert("range_from_100m_km".into(), rhai::Dynamic::from(result.range_from_100m_altitude_km as f64));
            map.insert("range_from_500m_km".into(), rhai::Dynamic::from(result.range_from_500m_altitude_km as f64));
            map
        },
    );

    engine.register_fn("recommend_motor_prop", |thrust: f64, cruise_v: f64, max_weight: f64| -> rhai::Array {
        use crate::aero::PropulsionDatabase;
        let db = PropulsionDatabase::new();
        db.recommend_motor_prop(thrust as f32, cruise_v as f32, max_weight as f32)
            .into_iter()
            .map(|r| {
                let mut map = rhai::Map::new();
                map.insert("motor_name".into(), rhai::Dynamic::from(r.motor.name.clone()));
                map.insert("prop_name".into(), rhai::Dynamic::from(r.prop.name.clone()));
                map.insert("cells".into(), rhai::Dynamic::from(r.cells as i64));
                map.insert("static_thrust_n".into(), rhai::Dynamic::from(r.static_thrust_n as f64));
                map.insert("cruise_thrust_n".into(), rhai::Dynamic::from(r.cruise_thrust_n as f64));
                map.insert("cruise_efficiency".into(), rhai::Dynamic::from(r.cruise_efficiency as f64));
                map.insert("estimated_endurance_min".into(), rhai::Dynamic::from(r.estimated_endurance_min as f64));
                map.insert("score".into(), rhai::Dynamic::from(r.score as f64));
                rhai::Dynamic::from(map)
            })
            .collect()
    });
}
