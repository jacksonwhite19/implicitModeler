// Propulsion database: motor specs, prop specs, and recommendation engine.
#![allow(dead_code)] // Propulsion data model — not all methods called in current UI

use serde::{Serialize, Deserialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MotorSpec {
    pub name: String,
    pub kv_rpm_per_volt: f32,
    pub max_power_w: f32,
    pub max_current_a: f32,
    pub weight_g: f32,
    pub stator_diameter_mm: f32,
    pub stator_height_mm: f32,
    pub internal_resistance_ohm: f32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PropSpec {
    pub name: String,
    pub diameter_mm: f32,
    pub pitch_mm: f32,
    pub blades: u32,
    pub ct_coeffs: [f32; 5],
    pub cp_coeffs: [f32; 5],
    pub j_max: f32,
}

impl PropSpec {
    pub fn ct_at(&self, j: f32) -> f32 {
        let j = j.clamp(0.0, self.j_max);
        self.ct_coeffs
            .iter()
            .enumerate()
            .map(|(i, &c)| c * j.powi(i as i32))
            .sum::<f32>()
            .max(0.0)
    }

    pub fn cp_at(&self, j: f32) -> f32 {
        let j = j.clamp(0.0, self.j_max);
        self.cp_coeffs
            .iter()
            .enumerate()
            .map(|(i, &c)| c * j.powi(i as i32))
            .sum::<f32>()
            .max(0.001)
    }

    pub fn efficiency_at(&self, j: f32) -> f32 {
        if j < 1e-6 {
            return 0.0;
        }
        (j * self.ct_at(j) / self.cp_at(j)).clamp(0.0, 1.0)
    }
}

#[derive(Clone, Debug)]
pub struct MotorPropRecommendation {
    pub motor: MotorSpec,
    pub prop: PropSpec,
    pub cells: u32,
    pub static_thrust_n: f32,
    pub cruise_thrust_n: f32,
    pub cruise_efficiency: f32,
    pub estimated_endurance_min: f32,
    pub score: f32,
}

pub struct PropulsionDatabase {
    pub motors: Vec<MotorSpec>,
    pub props: Vec<PropSpec>,
}

impl PropulsionDatabase {
    pub fn new() -> Self {
        let motors = vec![
            MotorSpec {
                name: "Sunnysky X2212 980KV".to_string(),
                kv_rpm_per_volt: 980.0,
                max_power_w: 180.0,
                max_current_a: 15.0,
                weight_g: 47.0,
                stator_diameter_mm: 22.0,
                stator_height_mm: 12.0,
                internal_resistance_ohm: 0.117,
            },
            MotorSpec {
                name: "Sunnysky X2212 1400KV".to_string(),
                kv_rpm_per_volt: 1400.0,
                max_power_w: 200.0,
                max_current_a: 20.0,
                weight_g: 47.0,
                stator_diameter_mm: 22.0,
                stator_height_mm: 12.0,
                internal_resistance_ohm: 0.083,
            },
            MotorSpec {
                name: "Sunnysky X2216 1250KV".to_string(),
                kv_rpm_per_volt: 1250.0,
                max_power_w: 250.0,
                max_current_a: 20.0,
                weight_g: 60.0,
                stator_diameter_mm: 22.0,
                stator_height_mm: 16.0,
                internal_resistance_ohm: 0.080,
            },
            MotorSpec {
                name: "Cobra 2217/20 1050KV".to_string(),
                kv_rpm_per_volt: 1050.0,
                max_power_w: 220.0,
                max_current_a: 18.0,
                weight_g: 56.0,
                stator_diameter_mm: 22.0,
                stator_height_mm: 17.0,
                internal_resistance_ohm: 0.095,
            },
            MotorSpec {
                name: "T-Motor AT2317 1400KV".to_string(),
                kv_rpm_per_volt: 1400.0,
                max_power_w: 280.0,
                max_current_a: 22.0,
                weight_g: 55.0,
                stator_diameter_mm: 23.0,
                stator_height_mm: 17.0,
                internal_resistance_ohm: 0.075,
            },
            MotorSpec {
                name: "Sunnysky X2820 920KV".to_string(),
                kv_rpm_per_volt: 920.0,
                max_power_w: 350.0,
                max_current_a: 28.0,
                weight_g: 95.0,
                stator_diameter_mm: 28.0,
                stator_height_mm: 20.0,
                internal_resistance_ohm: 0.062,
            },
            MotorSpec {
                name: "T-Motor AT2820 860KV".to_string(),
                kv_rpm_per_volt: 860.0,
                max_power_w: 400.0,
                max_current_a: 32.0,
                weight_g: 102.0,
                stator_diameter_mm: 28.0,
                stator_height_mm: 20.0,
                internal_resistance_ohm: 0.055,
            },
            MotorSpec {
                name: "Turnigy D2836/8 1100KV".to_string(),
                kv_rpm_per_volt: 1100.0,
                max_power_w: 300.0,
                max_current_a: 24.0,
                weight_g: 72.0,
                stator_diameter_mm: 28.0,
                stator_height_mm: 36.0,
                internal_resistance_ohm: 0.085,
            },
        ];

        let props = vec![
            PropSpec {
                name: "APC 8x4E".to_string(),
                diameter_mm: 203.2,
                pitch_mm: 101.6,
                blades: 2,
                ct_coeffs: [0.1082, -0.1339, -0.0093, 0.0, 0.0],
                cp_coeffs: [0.0398, -0.0306, -0.0081, 0.0, 0.0],
                j_max: 0.95,
            },
            PropSpec {
                name: "APC 9x4.5E".to_string(),
                diameter_mm: 228.6,
                pitch_mm: 114.3,
                blades: 2,
                ct_coeffs: [0.1180, -0.1500, -0.0085, 0.0, 0.0],
                cp_coeffs: [0.0441, -0.0350, -0.0082, 0.0, 0.0],
                j_max: 0.95,
            },
            PropSpec {
                name: "APC 9x6E".to_string(),
                diameter_mm: 228.6,
                pitch_mm: 152.4,
                blades: 2,
                ct_coeffs: [0.1285, -0.1183, -0.0142, 0.0, 0.0],
                cp_coeffs: [0.0520, -0.0385, -0.0092, 0.0, 0.0],
                j_max: 1.25,
            },
            PropSpec {
                name: "APC 10x4.7E".to_string(),
                diameter_mm: 254.0,
                pitch_mm: 119.4,
                blades: 2,
                ct_coeffs: [0.1172, -0.1421, -0.0094, 0.0, 0.0],
                cp_coeffs: [0.0460, -0.0358, -0.0083, 0.0, 0.0],
                j_max: 0.95,
            },
            PropSpec {
                name: "APC 10x7E".to_string(),
                diameter_mm: 254.0,
                pitch_mm: 177.8,
                blades: 2,
                ct_coeffs: [0.1340, -0.1120, -0.0158, 0.0, 0.0],
                cp_coeffs: [0.0558, -0.0398, -0.0098, 0.0, 0.0],
                j_max: 1.30,
            },
            PropSpec {
                name: "APC 11x5.5E".to_string(),
                diameter_mm: 279.4,
                pitch_mm: 139.7,
                blades: 2,
                ct_coeffs: [0.1230, -0.1280, -0.0125, 0.0, 0.0],
                cp_coeffs: [0.0495, -0.0372, -0.0090, 0.0, 0.0],
                j_max: 1.10,
            },
            PropSpec {
                name: "APC 11x7E".to_string(),
                diameter_mm: 279.4,
                pitch_mm: 177.8,
                blades: 2,
                ct_coeffs: [0.1350, -0.1130, -0.0155, 0.0, 0.0],
                cp_coeffs: [0.0562, -0.0402, -0.0095, 0.0, 0.0],
                j_max: 1.30,
            },
            PropSpec {
                name: "APC 12x6E".to_string(),
                diameter_mm: 304.8,
                pitch_mm: 152.4,
                blades: 2,
                ct_coeffs: [0.1285, -0.1230, -0.0140, 0.0, 0.0],
                cp_coeffs: [0.0528, -0.0385, -0.0092, 0.0, 0.0],
                j_max: 1.15,
            },
            PropSpec {
                name: "APC 12x8E".to_string(),
                diameter_mm: 304.8,
                pitch_mm: 203.2,
                blades: 2,
                ct_coeffs: [0.1420, -0.1085, -0.0168, 0.0, 0.0],
                cp_coeffs: [0.0590, -0.0415, -0.0100, 0.0, 0.0],
                j_max: 1.45,
            },
            PropSpec {
                name: "APC 13x6.5E".to_string(),
                diameter_mm: 330.2,
                pitch_mm: 165.1,
                blades: 2,
                ct_coeffs: [0.1295, -0.1245, -0.0138, 0.0, 0.0],
                cp_coeffs: [0.0535, -0.0390, -0.0093, 0.0, 0.0],
                j_max: 1.15,
            },
        ];

        Self { motors, props }
    }

    pub fn find_motor(&self, name: &str) -> Option<&MotorSpec> {
        self.motors.iter().find(|m| m.name == name)
    }

    pub fn find_prop_by_name(&self, name: &str) -> Option<&PropSpec> {
        self.props.iter().find(|p| p.name == name)
    }

    /// Find closest prop by diameter and pitch (mm).
    pub fn find_prop(&self, diameter_mm: f32, pitch_mm: f32) -> Option<&PropSpec> {
        self.props.iter().min_by(|a, b| {
            let da = (a.diameter_mm - diameter_mm).powi(2) + (a.pitch_mm - pitch_mm).powi(2);
            let db = (b.diameter_mm - diameter_mm).powi(2) + (b.pitch_mm - pitch_mm).powi(2);
            da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
        })
    }

    pub fn motors_in_power_range(&self, min_w: f32, max_w: f32) -> Vec<&MotorSpec> {
        self.motors
            .iter()
            .filter(|m| m.max_power_w >= min_w && m.max_power_w <= max_w)
            .collect()
    }

    pub fn recommend_motor_prop(
        &self,
        required_thrust_n: f32,
        cruise_airspeed_ms: f32,
        max_weight_g: f32,
    ) -> Vec<MotorPropRecommendation> {
        let rho = 1.225_f32; // sea level
        let mut candidates: Vec<MotorPropRecommendation> = Vec::new();

        for motor in &self.motors {
            if motor.weight_g > max_weight_g {
                continue;
            }
            for prop in &self.props {
                for cells in 2u32..=6 {
                    let v_batt = cells as f32 * 3.7;
                    let d_m = prop.diameter_mm / 1000.0;

                    // Iterative static thrust (V=0)
                    let mut rpm = motor.kv_rpm_per_volt * v_batt;
                    for _ in 0..10 {
                        let n = rpm / 60.0;
                        let j = 0.0_f32;
                        let cp = prop.cp_at(j).max(0.001);
                        let p_shaft = cp * rho * n.powi(3) * d_m.powi(5);
                        let p_elec = p_shaft / (0.85 * 0.95);
                        let i = p_elec / v_batt;
                        let v_drop = i * motor.internal_resistance_ohm;
                        let v_eff = (v_batt - v_drop).max(0.0);
                        let rpm_new = motor.kv_rpm_per_volt * v_eff;
                        if (rpm_new - rpm).abs() < 1.0 {
                            break;
                        }
                        rpm = rpm_new;
                    }
                    let n_static = rpm / 60.0;
                    let ct_static = prop.ct_at(0.0);
                    let static_thrust_n = ct_static * rho * n_static.powi(2) * d_m.powi(4);

                    // Minimum thrust margin check
                    if static_thrust_n < required_thrust_n * 1.2 {
                        continue;
                    }

                    // Cruise thrust at cruise_airspeed_ms
                    let mut rpm_c = motor.kv_rpm_per_volt * v_batt;
                    for _ in 0..10 {
                        let n = rpm_c / 60.0;
                        let j = if cruise_airspeed_ms > 0.01 {
                            (cruise_airspeed_ms / (n * d_m)).clamp(0.0, prop.j_max)
                        } else {
                            0.0
                        };
                        let cp = prop.cp_at(j).max(0.001);
                        let p_shaft = cp * rho * n.powi(3) * d_m.powi(5);
                        let p_elec = p_shaft / (0.85 * 0.95);
                        let i = p_elec / v_batt;
                        let v_drop = i * motor.internal_resistance_ohm;
                        let v_eff = (v_batt - v_drop).max(0.0);
                        let rpm_new = motor.kv_rpm_per_volt * v_eff;
                        if (rpm_new - rpm_c).abs() < 1.0 {
                            break;
                        }
                        rpm_c = rpm_new;
                    }
                    let n_cruise = rpm_c / 60.0;
                    let j_cruise = if cruise_airspeed_ms > 0.01 {
                        (cruise_airspeed_ms / (n_cruise * d_m)).clamp(0.0, prop.j_max)
                    } else {
                        0.0
                    };
                    let ct_cruise = prop.ct_at(j_cruise);
                    let cruise_thrust_n = ct_cruise * rho * n_cruise.powi(2) * d_m.powi(4);
                    let cruise_efficiency = prop.efficiency_at(j_cruise);

                    // Estimated endurance (rough: 2200 mAh 3S battery, 80% usable)
                    let cap_mah = 2200.0_f32;
                    let energy_wh = cap_mah * v_batt / 1000.0 * 0.80;
                    let cp_c = prop.cp_at(j_cruise).max(0.001);
                    let p_shaft_c = cp_c * rho * n_cruise.powi(3) * d_m.powi(5);
                    let p_elec_c = (p_shaft_c / (0.85 * 0.95)).max(1.0);
                    let estimated_endurance_min = (energy_wh / p_elec_c) * 60.0;

                    // Score
                    let thrust_margin_score =
                        ((static_thrust_n - required_thrust_n) / required_thrust_n).clamp(0.0, 1.0);
                    let weight_score = (1.0 - (motor.weight_g / max_weight_g)).clamp(0.0, 1.0);
                    let score = 0.4 * cruise_efficiency
                        + 0.3 * thrust_margin_score
                        + 0.3 * weight_score;

                    candidates.push(MotorPropRecommendation {
                        motor: motor.clone(),
                        prop: prop.clone(),
                        cells,
                        static_thrust_n,
                        cruise_thrust_n,
                        cruise_efficiency,
                        estimated_endurance_min,
                        score,
                    });
                }
            }
        }

        // Sort by score descending, return top 5
        candidates.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap_or(std::cmp::Ordering::Equal));
        candidates.truncate(5);
        candidates
    }
}
