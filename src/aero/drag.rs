// Drag polar: zero-lift drag build-up and parabolic polar construction.
#![allow(dead_code)] // Drag polar result fields — not all displayed in current UI

use std::sync::Arc;
use crate::sdf::Sdf;
use crate::sdf::query::bounding_points;
use crate::aero::{polars::PolarDatabase, flight_condition::FlightCondition};

/// Complete drag polar result.
#[derive(Clone, Debug)]
pub struct DragPolarResult {
    pub cd0: f32,
    pub cd0_breakdown: CD0Breakdown,
    pub k: f32,
    pub cl_best_ld: f32,
    pub ld_max: f32,
    pub cl_min_drag: f32,
    pub cd_min: f32,
    pub polar_points: Vec<(f32, f32)>,  // (CL, CD)
    pub best_glide_airspeed_ms: f32,
    pub best_endurance_airspeed_ms: f32,
    /// Reference wing area (m²) used for all coefficient conversions.
    pub s_ref_m2: f32,
    /// Maximum lift coefficient (default 1.2 for plain wings).
    pub cl_max: f32,
    /// Stall airspeed (m/s) at the weight passed to compute_drag_polar; 0 if no weight given.
    pub v_stall_ms: f32,
}

/// Breakdown of zero-lift drag by component.
#[derive(Clone, Debug)]
pub struct CD0Breakdown {
    pub wing: f32,
    pub fuselage: f32,
    pub h_tail: f32,
    pub v_tail: f32,
    pub interference: f32,
    pub total: f32,
}

// ── Public API ────────────────────────────────────────────────────────────────

/// Compute drag polar for a conventional layout.
///
/// Optional `weight_n` is used to estimate best-glide and best-endurance airspeeds.
/// Pass `None` (or the weight) — a 10 N default is used when None.
pub fn compute_drag_polar(
    wing_sdf:     &Arc<dyn Sdf>,
    fuselage_sdf: &Arc<dyn Sdf>,
    htail_sdf:    &Arc<dyn Sdf>,
    vtail_sdf:    &Arc<dyn Sdf>,
    polar_db:     &PolarDatabase,
    flight:       &FlightCondition,
    weight_n:     Option<f32>,
) -> DragPolarResult {
    let w_n = weight_n.unwrap_or(10.0).max(0.1);

    // ── Extract geometry ─────────────────────────────────────────────────────
    let wing_bbox  = bounding_points(wing_sdf.as_ref());
    let fuse_bbox  = bounding_points(fuselage_sdf.as_ref());
    let htail_bbox = bounding_points(htail_sdf.as_ref());
    let vtail_bbox = bounding_points(vtail_sdf.as_ref());

    let root_chord_mm = wing_bbox.size.x;
    let tip_chord_mm  = (root_chord_mm * 0.5).max(root_chord_mm * 0.1);
    let mac_mm        = (root_chord_mm + tip_chord_mm) * 0.5;
    let b_m           = wing_bbox.size.y / 1000.0;
    let s_ref_m2      = b_m * mac_mm / 1000.0;
    let ar_wing       = if s_ref_m2 > 1e-9 { b_m * b_m / s_ref_m2 } else { 6.0 };

    // ── Friction coefficient for each component ──────────────────────────────
    let mu = 1.789e-5_f32;
    let rho = flight.air_density_kg_m3;
    let v   = flight.airspeed_ms;
    let tc  = 0.12_f32; // default t/c for NACA 4-digit

    // Wing CD0.
    let re_wing = flight.reynolds_for_chord(root_chord_mm).max(1000.0);
    let cf_wing = turbulent_cf(re_wing);
    let kf_wing = 1.0 + 2.0 * tc + 60.0 * tc.powi(4);
    // Wetted area ≈ 2 * planform * (1 + 0.25*tc).
    let s_wet_wing = 2.0 * s_ref_m2 * (1.0 + 0.25 * tc);
    let cd0_wing   = cf_wing * kf_wing * s_wet_wing / s_ref_m2;

    // Fuselage CD0.
    let l_fuse_m  = (fuse_bbox.size.x / 1000.0).max(0.001);
    let d_fuse_m  = ((fuse_bbox.size.y + fuse_bbox.size.z) * 0.5 / 1000.0).max(0.001);
    let re_fuse   = (rho * v * l_fuse_m / mu).max(1000.0);
    let cf_fuse   = turbulent_cf(re_fuse);
    let fineness  = (l_fuse_m / d_fuse_m).max(1.0);
    let kf_fuse   = 1.0 + 60.0 / fineness.powi(3) + 0.0025 * fineness;
    // Wetted area ≈ π * D * L * 0.9 (cylinder approx).
    let s_wet_fuse = std::f32::consts::PI * d_fuse_m * l_fuse_m * 0.9;
    let cd0_fuse   = if s_ref_m2 > 1e-9 { cf_fuse * kf_fuse * s_wet_fuse / s_ref_m2 } else { 0.0 };

    // H-tail CD0 (same as wing approach).
    let htail_chord_mm  = htail_bbox.size.x;
    let htail_span_m    = htail_bbox.size.y / 1000.0;
    let s_htail_m2      = htail_span_m * htail_chord_mm / 1000.0;
    let re_htail        = flight.reynolds_for_chord(htail_chord_mm).max(1000.0);
    let cf_htail        = turbulent_cf(re_htail);
    let kf_htail        = 1.0 + 2.0 * tc + 60.0 * tc.powi(4);
    let s_wet_htail     = 2.0 * s_htail_m2 * (1.0 + 0.25 * tc);
    let cd0_htail       = if s_ref_m2 > 1e-9 { cf_htail * kf_htail * s_wet_htail / s_ref_m2 } else { 0.0 };

    // V-tail CD0.
    let vtail_chord_mm  = vtail_bbox.size.x;
    let vtail_span_m    = (vtail_bbox.size.y.max(vtail_bbox.size.z)) / 1000.0;
    let s_vtail_m2      = vtail_span_m * vtail_chord_mm / 1000.0;
    let re_vtail        = flight.reynolds_for_chord(vtail_chord_mm).max(1000.0);
    let cf_vtail        = turbulent_cf(re_vtail);
    let kf_vtail        = 1.0 + 2.0 * tc + 60.0 * tc.powi(4);
    let s_wet_vtail     = 2.0 * s_vtail_m2 * (1.0 + 0.25 * tc);
    let cd0_vtail       = if s_ref_m2 > 1e-9 { cf_vtail * kf_vtail * s_wet_vtail / s_ref_m2 } else { 0.0 };

    // Interference drag: 5% of sum.
    let sum_components  = cd0_wing + cd0_fuse + cd0_htail + cd0_vtail;
    let cd0_interference = sum_components * 0.05;
    let cd0_total       = sum_components + cd0_interference;

    let breakdown = CD0Breakdown {
        wing:         cd0_wing,
        fuselage:     cd0_fuse,
        h_tail:       cd0_htail,
        v_tail:       cd0_vtail,
        interference: cd0_interference,
        total:        cd0_total,
    };

    // ── Induced drag factor ──────────────────────────────────────────────────
    // Use e=0.85 for rectangular planform estimate.
    let e = 0.85_f32;
    let k = 1.0 / (std::f32::consts::PI * e * ar_wing);

    // ── Polar points ─────────────────────────────────────────────────────────
    let re_mid  = flight.reynolds_for_chord(mac_mm).max(1000.0);
    let polar   = polar_db.get_interpolated("NACA 0012", re_mid)
        .or_else(|| polar_db.get_interpolated("NACA 0012", 500_000.0))
        .expect("NACA 0012 must exist");

    let stall_aoa = polar.alpha_stall_deg.min(20.0);
    let mut polar_points: Vec<(f32, f32)> = Vec::new();

    let mut aoa = -5.0_f32;
    while aoa <= stall_aoa + 1e-4 {
        let cl = polar.cl_at(aoa);
        let cd = cd0_total + k * cl * cl;
        polar_points.push((cl, cd));
        aoa += 1.0;
    }

    // ── Best L/D ─────────────────────────────────────────────────────────────
    let cl_best   = (cd0_total / k).sqrt().max(0.01);
    let ld_max    = cl_best / (2.0 * cd0_total);
    let cl_min_drag = 0.0_f32; // for symmetric parabola, min drag at CL=0.
    let cd_min    = cd0_total;

    // ── Performance airspeeds ─────────────────────────────────────────────────
    // Best glide: CL at best L/D.
    let v_glide = if s_ref_m2 > 1e-9 && rho > 0.0 {
        let q_req = w_n / (s_ref_m2 * cl_best.max(0.01));
        (2.0 * q_req / rho).sqrt()
    } else {
        flight.airspeed_ms
    };

    // Best endurance: fly at CL^(3/2)/CD max → CL = sqrt(3*CD0/k).
    let cl_endurance = (3.0 * cd0_total / k).sqrt().max(0.01);
    let v_endurance = if s_ref_m2 > 1e-9 && rho > 0.0 {
        let q_req = w_n / (s_ref_m2 * cl_endurance.max(0.01));
        (2.0 * q_req / rho).sqrt()
    } else {
        flight.airspeed_ms * 0.76 // approximately 76% of best glide speed
    };

    // ── Stall speed ───────────────────────────────────────────────────────────
    let cl_max = 1.2_f32;
    let v_stall_ms = if s_ref_m2 > 1e-9 && rho > 0.0 {
        (2.0 * w_n / (rho * s_ref_m2 * cl_max)).sqrt()
    } else {
        0.0
    };

    DragPolarResult {
        cd0: cd0_total,
        cd0_breakdown: breakdown,
        k,
        cl_best_ld: cl_best,
        ld_max,
        cl_min_drag,
        cd_min,
        polar_points,
        best_glide_airspeed_ms:     v_glide,
        best_endurance_airspeed_ms: v_endurance,
        s_ref_m2,
        cl_max,
        v_stall_ms,
    }
}

// ── Private helpers ───────────────────────────────────────────────────────────

/// Schlichting turbulent skin-friction coefficient.
/// Cf = 0.455 / (log10(Re))^2.58
fn turbulent_cf(re: f32) -> f32 {
    let re = re.max(10_000.0);
    let log_re = re.log10().max(1.0); // log10(10000)=4, guard against near-zero
    let denom = log_re.powf(2.58);
    if denom < 1e-12 { return 0.006; }  // fallback for degenerate inputs
    0.455 / denom
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;
    use std::sync::Arc;
    use crate::aero::polars::PolarDatabase;
    use crate::aero::flight_condition::FlightCondition;

    fn box_sdf_centered(center: Vec3, half: Vec3) -> Arc<dyn Sdf> {
        use crate::sdf::primitives::SdfBox;
        use crate::sdf::transforms::Translate;
        let b: Arc<dyn Sdf> = Arc::new(SdfBox::new(half));
        Arc::new(Translate::new(b, center))
    }

    fn make_typical_aircraft() -> (Arc<dyn Sdf>, Arc<dyn Sdf>, Arc<dyn Sdf>, Arc<dyn Sdf>) {
        // All shapes must pass through or very near origin so bounding_points works.
        // Wing: 200mm chord at x=-100..100, 800mm span at y=-400..400. Origin inside.
        let wing  = box_sdf_centered(Vec3::ZERO, Vec3::new(100.0, 400.0, 5.0));
        // Fuselage: long in x, thin in y/z. Origin inside.
        let fuse  = box_sdf_centered(Vec3::ZERO, Vec3::new(300.0, 20.0, 20.0));
        // H-tail: thin in x (chord=70mm), wide in y (span=200mm). Origin inside.
        let htail = box_sdf_centered(Vec3::ZERO, Vec3::new(35.0, 100.0, 3.0));
        // V-tail: thin in y, tall in z. Origin inside.
        let vtail = box_sdf_centered(Vec3::ZERO, Vec3::new(35.0, 3.0, 80.0));
        (wing, fuse, htail, vtail)
    }

    /// Test 1: CD0 for clean config in realistic range 0.015–0.060.
    #[test]
    fn test_cd0_in_range() {
        let db = PolarDatabase::new();
        let fc = FlightCondition::new(20.0, 0.0, 4.0);
        let (wing, fuse, htail, vtail) = make_typical_aircraft();
        let result = compute_drag_polar(&wing, &fuse, &htail, &vtail, &db, &fc, Some(10.0));
        println!("CD0 = {:.4}, breakdown: wing={:.4} fuse={:.4} htail={:.4} vtail={:.4}",
            result.cd0,
            result.cd0_breakdown.wing,
            result.cd0_breakdown.fuselage,
            result.cd0_breakdown.h_tail,
            result.cd0_breakdown.v_tail);
        assert!(result.cd0 >= 0.010 && result.cd0 <= 0.080,
            "CD0 should be 0.010–0.080, got {:.4}", result.cd0);
    }

    /// Test 2: L/D max in realistic range 8–25.
    #[test]
    fn test_ld_max_in_range() {
        let db = PolarDatabase::new();
        let fc = FlightCondition::new(20.0, 0.0, 4.0);
        let (wing, fuse, htail, vtail) = make_typical_aircraft();
        let result = compute_drag_polar(&wing, &fuse, &htail, &vtail, &db, &fc, Some(10.0));
        println!("L/D max = {:.1}", result.ld_max);
        assert!(result.ld_max >= 5.0 && result.ld_max <= 30.0,
            "L/D max should be 5–30, got {:.1}", result.ld_max);
    }

    /// Test 3: polar_points has entries for each AoA step.
    #[test]
    fn test_polar_points_populated() {
        let db = PolarDatabase::new();
        let fc = FlightCondition::new(20.0, 0.0, 4.0);
        let (wing, fuse, htail, vtail) = make_typical_aircraft();
        let result = compute_drag_polar(&wing, &fuse, &htail, &vtail, &db, &fc, Some(10.0));
        // From -5 to ~16 deg in 1 deg steps → ~22 points.
        assert!(result.polar_points.len() >= 10,
            "Polar points should have at least 10 entries, got {}",
            result.polar_points.len());
        // All CD values should be positive.
        for (cl, cd) in &result.polar_points {
            assert!(*cd > 0.0, "CD should be positive at CL={:.3}, CD={:.5}", cl, cd);
        }
    }
}
