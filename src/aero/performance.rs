// Flight performance: rate of climb, range/endurance, and glide analysis.
#![allow(dead_code)] // Performance result fields — not all displayed in current UI

use crate::aero::flight_condition::FlightCondition;
use crate::aero::drag::DragPolarResult;
use crate::aero::propulsion::{PropulsionSetup, PropulsionResult};

// ── ISA density helper (replicated inline) ────────────────────────────────────

fn isa_density(altitude_m: f32) -> f32 {
    if altitude_m <= 11_000.0 {
        1.225 * (1.0 - 2.2558e-5 * altitude_m).powf(4.2559)
    } else {
        0.3639 * (-1.5769e-4 * (altitude_m - 11_000.0)).exp()
    }
}

// ── Interpolation helper ──────────────────────────────────────────────────────

/// Linear interpolation of a curve at airspeed `v`.
/// Returns the value at the nearest lower point if out of range.
fn interp_curve(curve: &[(f32, f32)], v: f32) -> f32 {
    if curve.is_empty() {
        return 0.0;
    }
    if v <= curve[0].0 {
        return curve[0].1;
    }
    for i in 1..curve.len() {
        let (v0, y0) = curve[i - 1];
        let (v1, y1) = curve[i];
        if v <= v1 {
            let t = (v - v0) / (v1 - v0).max(1e-6);
            return (1.0 - t) * y0 + t * y1;
        }
    }
    curve.last().map(|&(_, y)| y).unwrap_or(0.0)
}

// ── Rate of Climb ─────────────────────────────────────────────────────────────

#[derive(Clone, Debug)]
pub struct ClimbResult {
    pub max_roc_ms: f32,
    pub max_roc_fpm: f32,
    pub best_climb_airspeed_ms: f32,
    pub climb_angle_deg: f32,
    pub service_ceiling_m: f32,
    pub time_to_altitude: Vec<(f32, f32)>,
    pub excess_power_curve: Vec<(f32, f32)>,
}

pub fn compute_rate_of_climb(
    propulsion: &PropulsionResult,
    drag_polar: &DragPolarResult,
    flight: &FlightCondition,
    weight_n: f32,
) -> ClimbResult {
    let rho = flight.air_density_kg_m3;
    let s_ref = drag_polar.s_ref_m2.max(1e-9);

    // Build excess power curve over airspeed range
    let mut excess_power_curve: Vec<(f32, f32)> = Vec::new();
    let mut max_excess = f32::NEG_INFINITY;
    let mut best_v = 0.0_f32;

    for &(v, thrust_n) in &propulsion.thrust_curve {
        if v < 1.0 {
            continue;
        }
        let cl = 2.0 * weight_n / (rho * v * v * s_ref);
        let cd = drag_polar.cd0 + drag_polar.k * cl * cl;
        let drag_force = cd * 0.5 * rho * v * v * s_ref;
        let p_excess = thrust_n * v - drag_force * v;
        excess_power_curve.push((v, p_excess));
        if p_excess > max_excess {
            max_excess = p_excess;
            best_v = v;
        }
    }

    let max_roc_ms = if max_excess > 0.0 && weight_n > 1e-6 {
        max_excess / weight_n
    } else {
        0.0
    };
    let max_roc_fpm = max_roc_ms * 196.85;
    let best_climb_airspeed_ms = best_v;

    // Climb angle at best RoC speed
    let thrust_at_best = interp_curve(&propulsion.thrust_curve, best_v);
    let cl_best = 2.0 * weight_n / (rho * best_v.max(1.0).powi(2) * s_ref);
    let cd_best = drag_polar.cd0 + drag_polar.k * cl_best * cl_best;
    let drag_at_best = cd_best * 0.5 * rho * best_v.max(1.0).powi(2) * s_ref;
    let climb_angle_deg = if weight_n > 1e-6 {
        ((thrust_at_best - drag_at_best) / weight_n)
            .clamp(-1.0, 1.0)
            .asin()
            .to_degrees()
    } else {
        0.0
    };

    // Service ceiling: iterate altitude until RoC < 0.5 m/s
    let rho_sl = 1.225_f32;
    let mut service_ceiling_m = 0.0_f32;
    let mut altitude = 0.0_f32;
    while altitude <= 5000.0 {
        let rho_alt = isa_density(altitude);
        // Scale thrust and drag linearly to altitude density (simplified)
        let thrust_scale = (rho_alt / rho_sl).powf(0.7);
        let drag_scale = rho_alt / rho_sl;
        let thrust_alt = interp_curve(&propulsion.thrust_curve, best_v) * thrust_scale;
        let drag_alt = drag_at_best * drag_scale;
        let roc_alt = ((thrust_alt - drag_alt) * best_v / weight_n.max(1e-6)).max(0.0);
        if roc_alt < 0.5 {
            service_ceiling_m = altitude;
            break;
        }
        service_ceiling_m = altitude;
        altitude += 100.0;
    }

    // Time to altitude (0 to min(service_ceiling, 500) m)
    let max_alt = service_ceiling_m.min(500.0);
    let mut time_to_altitude: Vec<(f32, f32)> = Vec::new();
    let mut cumulative_time = 0.0_f32;
    let mut h = 0.0_f32;
    time_to_altitude.push((0.0, 0.0));
    while h < max_alt - 1e-3 {
        let dh = 50.0_f32.min(max_alt - h);
        let dt = dh / max_roc_ms.max(0.1);
        cumulative_time += dt;
        h += dh;
        time_to_altitude.push((h, cumulative_time));
    }

    ClimbResult {
        max_roc_ms,
        max_roc_fpm,
        best_climb_airspeed_ms,
        climb_angle_deg,
        service_ceiling_m,
        time_to_altitude,
        excess_power_curve,
    }
}

// ── Range and Endurance ───────────────────────────────────────────────────────

#[derive(Clone, Debug)]
pub struct RangeEnduranceResult {
    pub max_endurance_min: f32,
    pub max_endurance_airspeed_ms: f32,
    pub max_range_km: f32,
    pub max_range_airspeed_ms: f32,
    pub cruise_endurance_min: f32,
    pub cruise_range_km: f32,
    pub battery_energy_wh: f32,
    pub usable_energy_wh: f32,
    pub power_required_curve: Vec<(f32, f32)>,
    pub endurance_curve: Vec<(f32, f32)>,
    pub range_curve: Vec<(f32, f32)>,
    pub specific_energy_wh_per_km: f32,
}

pub fn compute_range_endurance(
    setup: &PropulsionSetup,
    propulsion: &PropulsionResult,
    _drag_polar: &DragPolarResult,
    flight: &FlightCondition,
    _weight_n: f32,
) -> RangeEnduranceResult {
    let battery_energy_wh = setup.battery_energy_wh();
    let usable_energy_wh = battery_energy_wh * 0.80;

    let mut power_required_curve: Vec<(f32, f32)> = Vec::new();
    let mut endurance_curve: Vec<(f32, f32)> = Vec::new();
    let mut range_curve: Vec<(f32, f32)> = Vec::new();

    let mut max_endurance_min = 0.0_f32;
    let mut max_endurance_airspeed_ms = 0.0_f32;
    let mut max_range_km = 0.0_f32;
    let mut max_range_airspeed_ms = 0.0_f32;

    for &(v, p_elec) in &propulsion.power_input_curve {
        if v < 5.0 {
            continue; // skip near-stall speeds
        }
        let p_req = p_elec.max(1e-3);
        power_required_curve.push((v, p_req));

        let endurance_min = (usable_energy_wh / p_req) * 60.0;
        let range_km = (usable_energy_wh / p_req) * v * 3.6;

        endurance_curve.push((v, endurance_min));
        range_curve.push((v, range_km));

        if endurance_min > max_endurance_min {
            max_endurance_min = endurance_min;
            max_endurance_airspeed_ms = v;
        }
        if range_km > max_range_km {
            max_range_km = range_km;
            max_range_airspeed_ms = v;
        }
    }

    // Cruise performance at flight condition airspeed
    let cruise_v = flight.airspeed_ms;
    let cruise_p = interp_curve(&propulsion.power_input_curve, cruise_v).max(1e-3);
    let cruise_endurance_min = (usable_energy_wh / cruise_p) * 60.0;
    let cruise_range_km = (usable_energy_wh / cruise_p) * cruise_v * 3.6;

    // Specific energy at cruise
    let specific_energy_wh_per_km = if cruise_v > 1e-3 {
        cruise_p / (cruise_v * 3.6)
    } else {
        0.0
    };

    RangeEnduranceResult {
        max_endurance_min,
        max_endurance_airspeed_ms,
        max_range_km,
        max_range_airspeed_ms,
        cruise_endurance_min,
        cruise_range_km,
        battery_energy_wh,
        usable_energy_wh,
        power_required_curve,
        endurance_curve,
        range_curve,
        specific_energy_wh_per_km,
    }
}

// ── Glide ─────────────────────────────────────────────────────────────────────

#[derive(Clone, Debug)]
pub struct GlideResult {
    pub best_glide_ratio: f32,
    pub best_glide_airspeed_ms: f32,
    pub best_glide_sink_rate_ms: f32,
    pub min_sink_rate_ms: f32,
    pub min_sink_airspeed_ms: f32,
    pub glide_polar: Vec<(f32, f32)>,
    pub range_from_100m_altitude_km: f32,
    pub range_from_500m_altitude_km: f32,
}

pub fn compute_glide(
    drag_polar: &DragPolarResult,
    flight: &FlightCondition,
    weight_n: f32,
) -> GlideResult {
    let rho = flight.air_density_kg_m3;
    let s = drag_polar.s_ref_m2.max(1e-9);
    let cd0 = drag_polar.cd0;
    let k = drag_polar.k;
    let _cl_max = drag_polar.cl_max.max(0.5);

    // Best glide from polar
    let best_glide_ratio = drag_polar.ld_max;
    let best_glide_airspeed_ms = drag_polar.best_glide_airspeed_ms;
    let best_glide_sink_rate_ms = if best_glide_ratio > 1e-6 {
        best_glide_airspeed_ms / best_glide_ratio
    } else {
        best_glide_airspeed_ms
    };

    // Min sink: CL for min sink = sqrt(3 * cd0 / k)
    let cl_minsink = (3.0 * cd0 / k.max(1e-9)).sqrt();
    let cd_minsink = cd0 + k * cl_minsink * cl_minsink;
    let v_minsink = if rho > 1e-9 && s > 1e-9 {
        (2.0 * weight_n / (rho * s * cl_minsink.max(0.01))).sqrt()
    } else {
        0.0
    };
    let min_sink_rate_ms = if cl_minsink > 1e-6 {
        v_minsink * cd_minsink / cl_minsink
    } else {
        v_minsink
    };

    // Stall speed from polar
    let v_stall = drag_polar.v_stall_ms;
    let v_start = (v_stall * 1.05).max(3.0);

    // Glide polar: airspeed vs sink rate
    let mut glide_polar: Vec<(f32, f32)> = Vec::new();
    let mut v = v_start;
    while v <= 30.0 + 1e-4 {
        let cl = 2.0 * weight_n / (rho * v * v * s);
        let cd = cd0 + k * cl * cl;
        let sink = v * cd / cl.max(1e-6);
        glide_polar.push((v, sink));
        v += 0.5;
    }

    // Glide ranges from altitude
    let range_from_100m_altitude_km = best_glide_ratio * 0.1;
    let range_from_500m_altitude_km = best_glide_ratio * 0.5;

    GlideResult {
        best_glide_ratio,
        best_glide_airspeed_ms,
        best_glide_sink_rate_ms,
        min_sink_rate_ms,
        min_sink_airspeed_ms: v_minsink,
        glide_polar,
        range_from_100m_altitude_km,
        range_from_500m_altitude_km,
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aero::flight_condition::FlightCondition;
    use crate::aero::propulsion_db::PropulsionDatabase;
    use crate::aero::drag::{DragPolarResult, CD0Breakdown};

    fn make_drag_polar() -> DragPolarResult {
        DragPolarResult {
            cd0: 0.025,
            cd0_breakdown: CD0Breakdown {
                wing: 0.015,
                fuselage: 0.006,
                h_tail: 0.002,
                v_tail: 0.001,
                interference: 0.001,
                total: 0.025,
            },
            k: 0.045,
            cl_best_ld: 0.75,
            ld_max: 12.0,
            cl_min_drag: 0.0,
            cd_min: 0.025,
            polar_points: vec![],
            best_glide_airspeed_ms: 14.0,
            best_endurance_airspeed_ms: 11.0,
            s_ref_m2: 0.25,
            cl_max: 1.2,
            v_stall_ms: 8.5,
        }
    }

    fn make_setup() -> crate::aero::propulsion::PropulsionSetup {
        let db = PropulsionDatabase::new();
        crate::aero::propulsion::PropulsionSetup {
            motor: db.find_motor("Sunnysky X2212 980KV").unwrap().clone(),
            prop: db.find_prop_by_name("APC 10x4.7E").unwrap().clone(),
            battery_cells: 3,
            battery_capacity_mah: 2200.0,
            battery_c_rating: 20.0,
            motor_count: 1,
            efficiency_motor: 0.85,
            efficiency_esc: 0.95,
        }
    }

    #[test]
    fn endurance_at_best_speed_is_maximum() {
        use crate::aero::propulsion::compute_propulsion;
        let setup = make_setup();
        let fc = FlightCondition::new(15.0, 0.0, 0.0);
        let drag = make_drag_polar();
        let prop = compute_propulsion(&setup, &fc, 10.0, None);
        let result = compute_range_endurance(&setup, &prop, &drag, &fc, 10.0);
        let max_end = result.max_endurance_min;
        let cruise_end = result.cruise_endurance_min;
        assert!(
            max_end >= cruise_end * 0.99,
            "max_end={} cruise_end={}",
            max_end,
            cruise_end
        );
    }

    #[test]
    fn range_at_best_range_speed_is_max() {
        use crate::aero::propulsion::compute_propulsion;
        let setup = make_setup();
        let fc = FlightCondition::new(15.0, 0.0, 0.0);
        let drag = make_drag_polar();
        let prop = compute_propulsion(&setup, &fc, 10.0, None);
        let result = compute_range_endurance(&setup, &prop, &drag, &fc, 10.0);
        assert!(result.max_range_km >= result.cruise_range_km * 0.99);
    }

    #[test]
    fn glide_ratio_positive() {
        let fc = FlightCondition::new(15.0, 0.0, 0.0);
        let drag = make_drag_polar();
        let result = compute_glide(&drag, &fc, 10.0);
        assert!(result.best_glide_ratio > 0.0);
        assert!(result.best_glide_ratio < 50.0);
    }
}
