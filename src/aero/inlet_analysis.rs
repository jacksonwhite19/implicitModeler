// Inlet performance analysis: pressure recovery, distortion, drag estimation.
#![allow(dead_code)] // Inlet analysis result fields — not all displayed in current UI

use super::FlightCondition;

#[derive(Clone, Debug)]
pub struct InletAnalysisResult {
    pub capture_area_ratio: f32,
    pub mass_flow_ratio: f32,
    pub pressure_recovery: f32,
    pub distortion_dc60: f32,
    pub diffuser_length_to_diameter: f32,
    pub area_ratio: f32,
    pub estimated_inlet_drag_n: f32,
    pub warnings: Vec<String>,
}

/// Compute inlet aerodynamic performance using Sovran-Klomp correlations.
///
/// * `inlet_radius` — inlet throat radius in mm
/// * `fan_radius` — fan face radius in mm
/// * `duct_length` — diffuser length in mm
/// * `bend_angle_deg` — duct bend angle (0 for straight duct)
/// * `flight` — flight condition
pub fn compute_inlet_performance(
    inlet_radius: f32,
    fan_radius: f32,
    duct_length: f32,
    bend_angle_deg: f32,
    flight: &FlightCondition,
) -> InletAnalysisResult {
    let inlet_area = std::f32::consts::PI * (inlet_radius * 0.001).powi(2); // m²
    let fan_area = std::f32::consts::PI * (fan_radius * 0.001).powi(2);
    let area_ratio = fan_area / inlet_area.max(1e-6);
    let duct_length_m = duct_length * 0.001;
    let diameter_m = 2.0 * inlet_radius * 0.001;
    let l_over_d = duct_length_m / diameter_m.max(1e-6);

    // Sovran-Klomp pressure recovery correlation
    let eta_p = (1.0
        - 0.035 * (area_ratio - 1.0).max(0.0).powf(1.3) * l_over_d.powf(-0.27))
    .clamp(0.80, 1.0);

    // DC60 distortion (S-duct contribution from bend angle)
    let dc60 = 0.03 * (bend_angle_deg / 30.0).powi(2) * (area_ratio - 1.0).max(0.0);

    // Mass flow ratio (slight overcapture typical at cruise)
    let capture_area = inlet_area * 1.05;
    let mass_flow_ratio = inlet_area / capture_area;

    // Additive drag (momentum deficit)
    let rho = flight.air_density_kg_m3;
    let v = flight.airspeed_ms;
    let mdot = rho * v * inlet_area;
    let v_inlet = v * inlet_area / fan_area.max(1e-6); // continuity
    let drag = mdot * (v_inlet - v);

    let mut warnings = Vec::new();
    if eta_p < 0.95 {
        warnings.push(format!(
            "Pressure recovery {:.3} < 0.95 — consider longer diffuser",
            eta_p
        ));
    }
    if dc60 > 0.15 {
        warnings.push(format!(
            "DC60 distortion {:.3} > 0.15 — fan stall risk",
            dc60
        ));
    }
    if l_over_d < 2.0 {
        warnings.push(format!(
            "Diffuser L/D {:.1} < 2.0 — separation risk",
            l_over_d
        ));
    }
    if area_ratio > 1.5 {
        warnings.push(format!(
            "Area ratio {:.2} > 1.5 — possible diffuser stall",
            area_ratio
        ));
    }

    InletAnalysisResult {
        capture_area_ratio: capture_area / inlet_area,
        mass_flow_ratio,
        pressure_recovery: eta_p,
        distortion_dc60: dc60,
        diffuser_length_to_diameter: l_over_d,
        area_ratio,
        estimated_inlet_drag_n: drag,
        warnings,
    }
}
