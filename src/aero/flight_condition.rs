// Flight condition: ISA atmosphere and aerodynamic parameters.
#![allow(dead_code)] // Flight condition fields — not all consumed in current analysis paths

/// Dynamic flight state with ISA atmosphere lookup.
#[derive(Clone, Debug)]
pub struct FlightCondition {
    pub airspeed_ms:        f32,
    pub air_density_kg_m3:  f32,
    pub altitude_m:         f32,
    pub aoa_deg:            f32,
    pub sideslip_deg:       f32,
    /// q = 0.5 * rho * V²  (Pa).
    pub dynamic_pressure_pa: f32,
    /// ρ·V/μ where μ = 1.789e-5 Pa·s.
    pub reynolds_per_meter:  f32,
}

impl FlightCondition {
    /// Construct from airspeed, altitude and angle of attack.
    pub fn new(airspeed_ms: f32, altitude_m: f32, aoa_deg: f32) -> Self {
        let rho = Self::isa_density(altitude_m);
        let mu  = 1.789e-5_f32;
        Self {
            airspeed_ms,
            air_density_kg_m3:   rho,
            altitude_m,
            aoa_deg,
            sideslip_deg:        0.0,
            dynamic_pressure_pa: 0.5 * rho * airspeed_ms * airspeed_ms,
            reynolds_per_meter:  rho * airspeed_ms / mu,
        }
    }

    /// Reynolds number for a given chord length in millimetres.
    pub fn reynolds_for_chord(&self, chord_mm: f32) -> f32 {
        self.reynolds_per_meter * chord_mm / 1000.0
    }

    /// ISA standard atmosphere density (kg/m³).
    ///
    /// Below 11 000 m (troposphere): ρ = 1.225 · (1 − 2.2558×10⁻⁵ · h)^4.2559
    /// Above 11 000 m (lower stratosphere): ρ = 0.3639 · exp(−1.5769×10⁻⁴ · (h − 11000))
    fn isa_density(altitude_m: f32) -> f32 {
        if altitude_m <= 11_000.0 {
            1.225 * (1.0 - 2.2558e-5 * altitude_m).powf(4.2559)
        } else {
            0.3639 * (-1.5769e-4 * (altitude_m - 11_000.0)).exp()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sea_level_density() {
        let fc = FlightCondition::new(50.0, 0.0, 5.0);
        assert!((fc.air_density_kg_m3 - 1.225).abs() < 0.01);
    }

    #[test]
    fn test_dynamic_pressure() {
        let fc = FlightCondition::new(100.0, 0.0, 0.0);
        let expected_q = 0.5 * 1.225 * 100.0 * 100.0;
        assert!((fc.dynamic_pressure_pa - expected_q).abs() < 1.0);
    }

    #[test]
    fn test_altitude_density_decrease() {
        let fc_lo = FlightCondition::new(50.0, 0.0, 0.0);
        let fc_hi = FlightCondition::new(50.0, 5000.0, 0.0);
        assert!(fc_hi.air_density_kg_m3 < fc_lo.air_density_kg_m3);
    }

    #[test]
    fn test_reynolds_for_chord() {
        let fc = FlightCondition::new(50.0, 0.0, 0.0);
        let re = fc.reynolds_for_chord(100.0);  // 100 mm chord
        // Re = rho*V*c/mu = 1.225 * 50 * 0.1 / 1.789e-5 ≈ 342 000
        assert!(re > 300_000.0 && re < 400_000.0, "Re = {re}");
    }
}
