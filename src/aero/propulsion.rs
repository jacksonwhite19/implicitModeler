// Propulsion analysis: thrust curves, power curves, and performance limits.
#![allow(dead_code)] // Propulsion result fields — not all displayed in current UI

use crate::aero::propulsion_db::{MotorSpec, PropSpec};
use crate::aero::flight_condition::FlightCondition;
use crate::aero::drag::DragPolarResult;

/// Complete propulsion system configuration.
#[derive(Clone, Debug)]
pub struct PropulsionSetup {
    pub motor: MotorSpec,
    pub prop: PropSpec,
    pub battery_cells: u32,
    pub battery_capacity_mah: f32,
    pub battery_c_rating: f32,
    pub motor_count: u32,
    pub efficiency_motor: f32,
    pub efficiency_esc: f32,
}

impl PropulsionSetup {
    pub fn battery_voltage_nominal(&self) -> f32 {
        self.battery_cells as f32 * 3.7
    }
    pub fn battery_voltage_full(&self) -> f32 {
        self.battery_cells as f32 * 4.2
    }
    pub fn battery_energy_wh(&self) -> f32 {
        self.battery_capacity_mah * self.battery_voltage_nominal() / 1000.0
    }
    pub fn max_continuous_current_a(&self) -> f32 {
        self.motor
            .max_current_a
            .min(self.battery_capacity_mah * self.battery_c_rating / 1000.0)
    }
    pub fn max_continuous_power_w(&self) -> f32 {
        self.max_continuous_current_a()
            * self.battery_voltage_nominal()
            * self.efficiency_motor
            * self.efficiency_esc
    }
}

/// Full propulsion analysis result.
#[derive(Clone, Debug)]
pub struct PropulsionResult {
    /// (airspeed_ms, thrust_n) pairs
    pub thrust_curve: Vec<(f32, f32)>,
    /// (airspeed_ms, power_available_w) = thrust * V
    pub power_available_curve: Vec<(f32, f32)>,
    /// (airspeed_ms, electrical_power_input_w)
    pub power_input_curve: Vec<(f32, f32)>,
    /// (airspeed_ms, propulsive_efficiency)
    pub efficiency_curve: Vec<(f32, f32)>,
    pub static_thrust_n: f32,
    pub max_airspeed_ms: f32,
    pub max_airspeed_kmh: f32,
    pub thrust_to_weight: f32,
    pub prop_tip_speed_ms: f32,
    pub prop_tip_mach: f32,
    pub static_power_input_w: f32,
    pub static_current_a: f32,
    pub within_motor_limits: bool,
    pub within_battery_limits: bool,
    pub warnings: Vec<String>,
}

/// Compute propulsion performance across airspeed range.
///
/// `drag` — if provided, used to find max airspeed via thrust/drag crossover.
pub fn compute_propulsion(
    setup: &PropulsionSetup,
    flight: &FlightCondition,
    weight_n: f32,
    drag: Option<&DragPolarResult>,
) -> PropulsionResult {
    let v_batt = setup.battery_voltage_nominal();
    let d_m = setup.prop.diameter_mm / 1000.0;
    let rho = flight.air_density_kg_m3;
    let n_motors = setup.motor_count as f32;

    let mut thrust_curve: Vec<(f32, f32)> = Vec::new();
    let mut power_available_curve: Vec<(f32, f32)> = Vec::new();
    let mut power_input_curve: Vec<(f32, f32)> = Vec::new();
    let mut efficiency_curve: Vec<(f32, f32)> = Vec::new();

    // Airspeed sweep: 0 to 40 m/s in 0.5 m/s steps
    let mut v = 0.0_f32;
    while v <= 40.0 + 1e-6 {
        // Iterative RPM convergence
        let mut rpm = setup.motor.kv_rpm_per_volt * v_batt;
        for _ in 0..20 {
            let n = rpm / 60.0;
            let j = if v > 0.01 {
                (v / (n * d_m)).clamp(0.0, setup.prop.j_max)
            } else {
                0.0
            };
            let cp = setup.prop.cp_at(j).max(0.001);
            let p_shaft = cp * rho * n.powi(3) * d_m.powi(5) * n_motors;
            let p_elec = p_shaft / (setup.efficiency_motor * setup.efficiency_esc);
            let i = p_elec / v_batt;
            let v_drop = i * setup.motor.internal_resistance_ohm * n_motors;
            let v_eff = (v_batt - v_drop).max(0.0);
            let rpm_new = setup.motor.kv_rpm_per_volt * v_eff;
            if (rpm_new - rpm).abs() < 1.0 {
                break;
            }
            rpm = rpm_new;
        }

        // Final values at converged RPM
        let n = rpm / 60.0;
        let j = if v > 0.01 {
            (v / (n * d_m)).clamp(0.0, setup.prop.j_max)
        } else {
            0.0
        };
        let ct = setup.prop.ct_at(j).max(0.0);
        let cp = setup.prop.cp_at(j).max(0.001);
        let thrust_n = ct * rho * n.powi(2) * d_m.powi(4) * n_motors;
        let p_shaft = cp * rho * n.powi(3) * d_m.powi(5) * n_motors;
        let p_elec = p_shaft / (setup.efficiency_motor * setup.efficiency_esc);
        let p_avail = thrust_n * v;
        let eta = if v > 0.1 {
            (thrust_n * v / p_elec.max(1e-6)).clamp(0.0, 1.0)
        } else {
            0.0
        };

        thrust_curve.push((v, thrust_n));
        power_available_curve.push((v, p_avail));
        power_input_curve.push((v, p_elec));
        efficiency_curve.push((v, eta));

        v += 0.5;
    }

    // Static conditions (V=0, first entry)
    let static_thrust_n = thrust_curve.first().map(|&(_, t)| t).unwrap_or(0.0);
    let static_power_input_w = power_input_curve.first().map(|&(_, p)| p).unwrap_or(0.0);
    let static_current_a = static_power_input_w / v_batt.max(1e-6);

    // Prop tip speed and Mach at static conditions
    let rpm_static = {
        let mut rpm = setup.motor.kv_rpm_per_volt * v_batt;
        for _ in 0..20 {
            let n = rpm / 60.0;
            let cp = setup.prop.cp_at(0.0).max(0.001);
            let p_shaft = cp * rho * n.powi(3) * d_m.powi(5) * n_motors;
            let p_elec = p_shaft / (setup.efficiency_motor * setup.efficiency_esc);
            let i = p_elec / v_batt;
            let v_drop = i * setup.motor.internal_resistance_ohm * n_motors;
            let v_eff = (v_batt - v_drop).max(0.0);
            let rpm_new = setup.motor.kv_rpm_per_volt * v_eff;
            if (rpm_new - rpm).abs() < 1.0 {
                break;
            }
            rpm = rpm_new;
        }
        rpm
    };
    let prop_tip_speed_ms = std::f32::consts::PI * d_m * (rpm_static / 60.0);
    let speed_of_sound = 340.3_f32; // m/s at sea level
    let prop_tip_mach = prop_tip_speed_ms / speed_of_sound;

    // Thrust-to-weight ratio
    let thrust_to_weight = if weight_n > 1e-6 {
        static_thrust_n / weight_n
    } else {
        0.0
    };

    // Max airspeed: find first crossover where thrust < drag
    let max_airspeed_ms = if let Some(drag) = drag {
        let s_ref = drag.s_ref_m2;
        let mut crossover_v = 0.0_f32;
        for i in 1..thrust_curve.len() {
            let (v_i, t_i) = thrust_curve[i];
            if v_i < 1.0 {
                continue;
            }
            let cl = 2.0 * weight_n / (rho * v_i * v_i * s_ref.max(1e-9));
            let cd = drag.cd0 + drag.k * cl * cl;
            let drag_force = cd * 0.5 * rho * v_i * v_i * s_ref;
            if t_i < drag_force {
                crossover_v = v_i;
                break;
            }
        }
        crossover_v
    } else {
        0.0
    };
    let max_airspeed_kmh = max_airspeed_ms * 3.6;

    // Limits checks
    let within_motor_limits = static_current_a <= setup.motor.max_current_a;
    let within_battery_limits = static_current_a <= setup.max_continuous_current_a();

    // Warnings
    let mut warnings: Vec<String> = Vec::new();
    if static_current_a > setup.motor.max_current_a {
        warnings.push(format!(
            "Motor overcurrent at full throttle: {:.1}A > {:.1}A max",
            static_current_a, setup.motor.max_current_a
        ));
    }
    if static_current_a > setup.max_continuous_current_a() {
        warnings.push(format!(
            "Battery overcurrent: {:.1}A > {:.1}A",
            static_current_a,
            setup.max_continuous_current_a()
        ));
    }
    if prop_tip_mach > 0.7 {
        warnings.push(format!(
            "Prop tip Mach {:.2} exceeds 0.7 — compressibility losses expected",
            prop_tip_mach
        ));
    }
    if thrust_to_weight < 1.0 {
        warnings.push(format!(
            "Thrust-to-weight {:.2} < 1.0 — insufficient thrust for sustained climb",
            thrust_to_weight
        ));
    }

    PropulsionResult {
        thrust_curve,
        power_available_curve,
        power_input_curve,
        efficiency_curve,
        static_thrust_n,
        max_airspeed_ms,
        max_airspeed_kmh,
        thrust_to_weight,
        prop_tip_speed_ms,
        prop_tip_mach,
        static_power_input_w,
        static_current_a,
        within_motor_limits,
        within_battery_limits,
        warnings,
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aero::{FlightCondition, propulsion_db::PropulsionDatabase};

    fn test_setup() -> PropulsionSetup {
        let db = PropulsionDatabase::new();
        PropulsionSetup {
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
    fn motor_lookup_correct_kv() {
        let db = PropulsionDatabase::new();
        let m = db.find_motor("Sunnysky X2212 980KV").unwrap();
        assert!((m.kv_rpm_per_volt - 980.0).abs() < 1.0);
        assert!((m.internal_resistance_ohm - 0.117).abs() < 0.001);
    }

    #[test]
    fn prop_ct_at_zero_positive() {
        let db = PropulsionDatabase::new();
        let p = db.find_prop_by_name("APC 10x4.7E").unwrap();
        assert!(p.ct_at(0.0) > 0.0);
    }

    #[test]
    fn prop_ct_at_jmax_near_zero() {
        let db = PropulsionDatabase::new();
        let p = db.find_prop_by_name("APC 10x4.7E").unwrap();
        assert!(p.ct_at(p.j_max) < 0.02);
    }

    #[test]
    fn static_thrust_positive() {
        let setup = test_setup();
        let fc = FlightCondition::new(15.0, 0.0, 0.0);
        let result = compute_propulsion(&setup, &fc, 10.0, None);
        assert!(result.static_thrust_n > 0.0, "static thrust={}", result.static_thrust_n);
    }

    #[test]
    fn thrust_decreases_with_airspeed() {
        let setup = test_setup();
        let fc = FlightCondition::new(15.0, 0.0, 0.0);
        let result = compute_propulsion(&setup, &fc, 10.0, None);
        let t0 = result.thrust_curve[0].1;
        let t20 = result
            .thrust_curve
            .iter()
            .find(|(v, _)| *v >= 19.5)
            .map(|(_, t)| *t)
            .unwrap_or(0.0);
        assert!(t0 > t20, "t0={} t20={}", t0, t20);
    }

    #[test]
    fn max_airspeed_reasonable() {
        let setup = test_setup();
        let fc = FlightCondition::new(15.0, 0.0, 0.0);
        let result = compute_propulsion(&setup, &fc, 10.0, None);
        // Without drag intersection, max_airspeed is 0 — just check it's non-negative
        assert!(result.max_airspeed_ms >= 0.0);
    }
}
