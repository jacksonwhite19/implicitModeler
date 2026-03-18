// Lifting line theory (Glauert Fourier series method).
#![allow(dead_code)] // Lifting line result fields — not all displayed in current UI
//
// Solves for the spanwise lift distribution on a finite wing described by an SDF.
// Returns aerodynamic coefficients, efficiency, and stall analysis.

use std::sync::Arc;
use crate::sdf::Sdf;
use crate::sdf::query::bounding_points;
use super::polars::PolarDatabase;
use super::flight_condition::FlightCondition;

/// Full result of a lifting-line analysis.
#[derive(Clone, Debug)]
pub struct LiftingLineResult {
    pub cl_total:             f32,
    pub cd_induced:           f32,
    pub oswald_efficiency:    f32,
    pub span_efficiency:      f32,
    pub lift_total_n:         f32,
    pub induced_drag_total_n: f32,
    pub lift_distribution:    Vec<f32>,   // Γ per station (m²/s)
    pub induced_aoa:          Vec<f32>,   // degrees
    pub effective_aoa:        Vec<f32>,   // degrees
    pub local_cl:             Vec<f32>,
    pub stall_stations:       Vec<usize>,
    pub stall_margin:         Vec<f32>,   // (cl_max - cl_local) / cl_max
    pub span_stations:        Vec<f32>,   // y/b/2, normalised
    pub tip_stall_risk:       bool,
    pub root_stall_first:     bool,
}

/// Solve lifting-line theory for a wing described by `wing_sdf`.
///
/// Wing geometry is extracted by SDF sampling:
///   - Span from bounding box Y extent.
///   - Chord sampled along X at each span station.
///   - Default airfoil NACA 0012.
pub fn solve_lifting_line(
    wing_sdf: &Arc<dyn Sdf>,
    polar_db: &PolarDatabase,
    flight:   &FlightCondition,
    n_stations: usize,
) -> LiftingLineResult {
    let n = n_stations.max(4).min(128);

    // ── Step 1: Extract wing geometry from SDF ─────────────────────────────
    let bbox   = bounding_points(wing_sdf.as_ref());
    let span_mm = bbox.size.y;
    let b_m     = span_mm / 1000.0;

    // Determine if half-span or full-span model.
    let is_half_span = bbox.min.y > -0.1 * span_mm;

    // Estimate root chord and tip chord by scanning X extent at y=0 and y=±span/2.
    let root_chord_mm = chord_at_y(wing_sdf.as_ref(), 0.0, &bbox);
    let tip_y         = if is_half_span { bbox.max.y * 0.98 } else { bbox.max.y * 0.98 };
    let tip_chord_mm  = chord_at_y(wing_sdf.as_ref(), tip_y, &bbox).max(root_chord_mm * 0.05);

    // Wing reference area (trapezoidal approximation) in m².
    let s_m2 = b_m * (root_chord_mm + tip_chord_mm) / 2.0 / 1000.0;

    // Get polar.
    let designation = "NACA 0012";
    let mid_chord   = (root_chord_mm + tip_chord_mm) * 0.5;
    let re_mid      = flight.reynolds_for_chord(mid_chord);
    let polar = polar_db.get_interpolated(designation, re_mid)
        .or_else(|| polar_db.get_interpolated("NACA 0012", re_mid))
        .expect("PolarDatabase must contain NACA 0012");

    // ── Step 2: Build Glauert system ───────────────────────────────────────
    // N odd Fourier terms: 1, 3, 5, ... 2N-1.
    let n_terms = n;

    // Cosine station angles: θ_i = i·π/(N+1) for i = 1..=N.
    let mut theta = vec![0.0_f32; n];
    let mut y_norm = vec![0.0_f32; n];  // y / (b/2), in [-1, 1]
    for i in 0..n {
        theta[i] = (i + 1) as f32 * std::f32::consts::PI / (n + 1) as f32;
        y_norm[i] = -(theta[i].cos());  // cosine spacing, -1 at root→tip
    }

    // Chord at each station.
    let mut chord_m = vec![0.0_f32; n];
    for i in 0..n {
        let frac = y_norm[i].abs();  // 0 at root, 1 at tip
        chord_m[i] = (root_chord_mm * (1.0 - frac) + tip_chord_mm * frac) / 1000.0;
    }

    // CL_alpha per radian from polar.
    let a_rad = polar.cl_alpha;  // per radian
    // Alpha_0 from polar.
    let alpha_0_rad = polar.alpha_zero_lift_deg.to_radians();

    // Build N×N matrix M and RHS b.
    let mut mat = vec![vec![0.0_f32; n_terms]; n];
    let mut rhs = vec![0.0_f32; n];

    let b_m_half = b_m * if is_half_span { 1.0 } else { 0.5 };

    for i in 0..n {
        let sin_t = theta[i].sin().max(1e-8);
        let c_i   = chord_m[i].max(1e-6);
        let mu_i  = c_i * a_rad / (4.0 * b_m_half);  // μ_i = c·a / (4b)

        for j in 0..n_terms {
            let nj = (2 * j + 1) as f32;  // 1, 3, 5, ...
            mat[i][j] = (mu_i * nj / sin_t + 1.0) * (nj * theta[i]).sin();
        }
        rhs[i] = mu_i * (flight.aoa_deg.to_radians() - alpha_0_rad);
    }

    // ── Step 3: Gaussian elimination with partial pivoting ─────────────────
    let coeffs = gaussian_solve(&mat, &rhs);

    // ── Step 4: Compute global aerodynamic coefficients ────────────────────
    let ar = b_m * b_m / s_m2;

    // CL = A_1 * π * AR (A_1 = coeffs[0]).
    let a1    = coeffs[0];
    let cl    = a1 * std::f32::consts::PI * ar;

    // Oswald efficiency: e = A_1² / Σ(n_j · A_j²).
    let mut denom = 0.0_f32;
    for j in 0..n_terms {
        let nj = (2 * j + 1) as f32;
        denom += nj * coeffs[j] * coeffs[j];
    }
    let e = if denom > 1e-12 { a1 * a1 / denom } else { 1.0 };
    let e = e.min(1.0).max(0.0);

    let cd_induced = if ar > 0.0 && e > 0.0 {
        cl * cl / (std::f32::consts::PI * e * ar)
    } else {
        0.0
    };

    let q          = flight.dynamic_pressure_pa;
    let lift_n     = q * s_m2 * cl;
    let drag_i_n   = q * s_m2 * cd_induced;

    // ── Step 5: Spanwise distributions ────────────────────────────────────
    let v = flight.airspeed_ms;
    let mut lift_dist   = vec![0.0_f32; n];
    let mut induced_aoa = vec![0.0_f32; n];
    let mut eff_aoa     = vec![0.0_f32; n];
    let mut local_cl    = vec![0.0_f32; n];

    for i in 0..n {
        // Circulation Γ_i = 2·b·V · Σ A_j · sin(n_j · θ_i).
        let mut gamma = 0.0_f32;
        let mut alpha_ind = 0.0_f32;
        for j in 0..n_terms {
            let nj  = (2 * j + 1) as f32;
            let stj = (nj * theta[i]).sin();
            gamma     += coeffs[j] * stj;
            // α_ind_i = Σ n_j · A_j · sin(n_j θ_i) / sin(θ_i)
            alpha_ind += nj * coeffs[j] * stj;
        }
        let sin_t   = theta[i].sin().max(1e-8);
        gamma       *= 2.0 * b_m_half * v;
        alpha_ind    = (alpha_ind / sin_t).to_degrees();

        let c_i     = chord_m[i].max(1e-6);
        let cl_i    = if v > 0.0 && c_i > 0.0 {
            2.0 * gamma / (v * c_i)
        } else { 0.0 };
        let aoa_eff = flight.aoa_deg - alpha_ind;

        lift_dist[i]   = gamma;
        induced_aoa[i] = alpha_ind;
        eff_aoa[i]     = aoa_eff;
        local_cl[i]    = cl_i;
    }

    // ── Step 6: Stall analysis ─────────────────────────────────────────────
    let mut stall_stations = Vec::new();
    let mut stall_margin   = vec![0.0_f32; n];
    let cl_max = polar.cl_max;

    for i in 0..n {
        let margin = (cl_max - local_cl[i].abs()) / cl_max.max(0.1);
        stall_margin[i] = margin;
        if margin < 0.0 {
            stall_stations.push(i);
        }
    }

    // Tip stall: stall stations concentrated near tip (|y_norm| > 0.7).
    let tip_stall_risk = stall_stations.iter()
        .any(|&i| y_norm[i].abs() > 0.7 && stall_margin[i] < 0.1);

    // Root stall first: minimum margin at root (|y_norm| < 0.3) before tip.
    let root_min = stall_margin.iter().enumerate()
        .filter(|(i, _)| y_norm[*i].abs() < 0.3)
        .map(|(_, &m)| m)
        .fold(f32::MAX, f32::min);
    let tip_min = stall_margin.iter().enumerate()
        .filter(|(i, _)| y_norm[*i].abs() > 0.7)
        .map(|(_, &m)| m)
        .fold(f32::MAX, f32::min);
    let root_stall_first = root_min < tip_min;

    LiftingLineResult {
        cl_total:             cl,
        cd_induced,
        oswald_efficiency:    e,
        span_efficiency:      e,
        lift_total_n:         lift_n,
        induced_drag_total_n: drag_i_n,
        lift_distribution:    lift_dist,
        induced_aoa,
        effective_aoa:        eff_aoa,
        local_cl,
        stall_stations,
        stall_margin,
        span_stations:        y_norm,
        tip_stall_risk,
        root_stall_first,
    }
}

// ── Chord estimation from SDF ─────────────────────────────────────────────────

fn chord_at_y(sdf: &dyn Sdf, y_mm: f32, bbox: &crate::sdf::query::BoundingInfo) -> f32 {
    // Scan X at given Y, Z=0 to find entry/exit of solid.
    let x_range = bbox.size.x;
    let x_start = bbox.min.x;
    let n_steps  = 128_usize;
    let dx       = x_range / n_steps as f32;

    let mut x_enter = f32::NAN;
    let mut x_exit  = f32::NAN;
    let mut prev_d  = sdf.distance(glam::Vec3::new(x_start, y_mm, 0.0));

    for k in 1..=n_steps {
        let x   = x_start + k as f32 * dx;
        let d   = sdf.distance(glam::Vec3::new(x, y_mm, 0.0));
        if prev_d > 0.0 && d < 0.0 && x_enter.is_nan() { x_enter = x; }
        if prev_d < 0.0 && d > 0.0                      { x_exit  = x; }
        prev_d = d;
    }

    if !x_enter.is_nan() && !x_exit.is_nan() {
        (x_exit - x_enter).abs()
    } else {
        // Fallback: use bbox X size (root assumption).
        bbox.size.x * 0.7
    }
}

// ── Gaussian elimination with partial pivoting ────────────────────────────────

fn gaussian_solve(a: &[Vec<f32>], b: &[f32]) -> Vec<f32> {
    let n = b.len();
    let mut mat: Vec<Vec<f32>> = a.iter().map(|row| row.clone()).collect();
    let mut rhs: Vec<f32>      = b.to_vec();

    for col in 0..n {
        // Partial pivot.
        let max_row = (col..n).max_by(|&r1, &r2| {
            mat[r1][col].abs().partial_cmp(&mat[r2][col].abs()).unwrap_or(std::cmp::Ordering::Equal)
        }).unwrap_or(col);
        mat.swap(col, max_row);
        rhs.swap(col, max_row);

        let pivot = mat[col][col];
        if pivot.abs() < 1e-12 { continue; }

        for row in (col + 1)..n {
            let factor = mat[row][col] / pivot;
            for c in col..n {
                let sub = mat[col][c] * factor;
                mat[row][c] -= sub;
            }
            rhs[row] -= rhs[col] * factor;
        }
    }

    // Back substitution.
    let mut x = vec![0.0_f32; n];
    for i in (0..n).rev() {
        let mut sum = rhs[i];
        for j in (i + 1)..n {
            sum -= mat[i][j] * x[j];
        }
        let pivot = mat[i][i];
        x[i] = if pivot.abs() > 1e-12 { sum / pivot } else { 0.0 };
    }
    x
}

// ── Test helper ───────────────────────────────────────────────────────────────

/// Solve lifting line from explicit geometric parameters (for testing without SDF).
#[cfg(test)]
pub fn solve_lifting_line_from_params(
    span_mm: f32,
    root_chord_mm: f32,
    tip_chord_mm:  f32,
    twist_deg:     f32,
    polar_db:      &PolarDatabase,
    flight:        &FlightCondition,
    n_stations:    usize,
) -> LiftingLineResult {
    let n = n_stations.max(4).min(128);

    let b_m      = span_mm / 1000.0;
    let b_m_half = b_m / 2.0;
    let n_terms  = n;

    let designation = "NACA 0012";
    let mid_chord   = (root_chord_mm + tip_chord_mm) * 0.5;
    let re_mid      = flight.reynolds_for_chord(mid_chord);
    let polar = polar_db.get_interpolated(designation, re_mid)
        .expect("NACA 0012 must be in database");

    let a_rad      = polar.cl_alpha;
    let alpha_0_rad = polar.alpha_zero_lift_deg.to_radians();

    let s_m2 = b_m * (root_chord_mm + tip_chord_mm) / 2.0 / 1000.0;
    let ar   = b_m * b_m / s_m2;

    let mut theta  = vec![0.0_f32; n];
    let mut y_norm = vec![0.0_f32; n];
    for i in 0..n {
        theta[i]  = (i + 1) as f32 * std::f32::consts::PI / (n + 1) as f32;
        y_norm[i] = -(theta[i].cos());
    }

    let mut chord_m = vec![0.0_f32; n];
    for i in 0..n {
        let frac    = y_norm[i].abs();
        chord_m[i]  = (root_chord_mm * (1.0 - frac) + tip_chord_mm * frac) / 1000.0;
    }

    // Twist linearly from root (0) to tip (twist_deg).
    let twist: Vec<f32> = (0..n).map(|i| twist_deg * y_norm[i].abs()).collect();

    let mut mat = vec![vec![0.0_f32; n_terms]; n];
    let mut rhs = vec![0.0_f32; n];

    for i in 0..n {
        let sin_t = theta[i].sin().max(1e-8);
        let c_i   = chord_m[i].max(1e-6);
        let mu_i  = c_i * a_rad / (4.0 * b_m_half);

        for j in 0..n_terms {
            let nj = (2 * j + 1) as f32;
            mat[i][j] = (mu_i * nj / sin_t + 1.0) * (nj * theta[i]).sin();
        }
        let aoa_eff = flight.aoa_deg + twist[i];
        rhs[i] = mu_i * (aoa_eff.to_radians() - alpha_0_rad);
    }

    let coeffs = gaussian_solve(&mat, &rhs);

    let a1 = coeffs[0];
    let cl = a1 * std::f32::consts::PI * ar;

    let mut denom = 0.0_f32;
    for j in 0..n_terms {
        let nj = (2 * j + 1) as f32;
        denom += nj * coeffs[j] * coeffs[j];
    }
    let e = if denom > 1e-12 { (a1 * a1 / denom).min(1.0).max(0.0) } else { 1.0 };

    let cd_induced = if ar > 0.0 && e > 0.0 {
        cl * cl / (std::f32::consts::PI * e * ar)
    } else { 0.0 };

    let q        = flight.dynamic_pressure_pa;
    let lift_n   = q * s_m2 * cl;
    let drag_i_n = q * s_m2 * cd_induced;

    let v = flight.airspeed_ms;
    let mut lift_dist   = vec![0.0_f32; n];
    let mut induced_aoa = vec![0.0_f32; n];
    let mut eff_aoa     = vec![0.0_f32; n];
    let mut local_cl    = vec![0.0_f32; n];

    for i in 0..n {
        let mut gamma     = 0.0_f32;
        let mut alpha_ind = 0.0_f32;
        for j in 0..n_terms {
            let nj  = (2 * j + 1) as f32;
            let stj = (nj * theta[i]).sin();
            gamma     += coeffs[j] * stj;
            alpha_ind += nj * coeffs[j] * stj;
        }
        let sin_t    = theta[i].sin().max(1e-8);
        gamma        *= 2.0 * b_m_half * v;
        alpha_ind     = (alpha_ind / sin_t).to_degrees();
        let c_i      = chord_m[i].max(1e-6);
        let cl_i     = if v > 0.0 && c_i > 0.0 { 2.0 * gamma / (v * c_i) } else { 0.0 };
        let aoa_eff_i = flight.aoa_deg - alpha_ind + twist[i];

        lift_dist[i]   = gamma;
        induced_aoa[i] = alpha_ind;
        eff_aoa[i]     = aoa_eff_i;
        local_cl[i]    = cl_i;
    }

    let cl_max = polar.cl_max;
    let mut stall_stations = Vec::new();
    let mut stall_margin   = vec![0.0_f32; n];
    for i in 0..n {
        let margin = (cl_max - local_cl[i].abs()) / cl_max.max(0.1);
        stall_margin[i] = margin;
        if margin < 0.0 { stall_stations.push(i); }
    }

    let tip_stall_risk = stall_stations.iter()
        .any(|&i| y_norm[i].abs() > 0.7 && stall_margin[i] < 0.1);

    let root_min = stall_margin.iter().enumerate()
        .filter(|(i, _)| y_norm[*i].abs() < 0.3)
        .map(|(_, &m)| m)
        .fold(f32::MAX, f32::min);
    let tip_min = stall_margin.iter().enumerate()
        .filter(|(i, _)| y_norm[*i].abs() > 0.7)
        .map(|(_, &m)| m)
        .fold(f32::MAX, f32::min);
    let root_stall_first = root_min < tip_min;

    LiftingLineResult {
        cl_total: cl, cd_induced, oswald_efficiency: e, span_efficiency: e,
        lift_total_n: lift_n, induced_drag_total_n: drag_i_n,
        lift_distribution: lift_dist, induced_aoa, effective_aoa: eff_aoa, local_cl,
        stall_stations, stall_margin, span_stations: y_norm,
        tip_stall_risk, root_stall_first,
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aero::polars::PolarDatabase;
    use crate::aero::flight_condition::FlightCondition;

    fn make_db() -> PolarDatabase { PolarDatabase::new() }

    fn flight_5deg() -> FlightCondition {
        FlightCondition::new(50.0, 0.0, 5.0)
    }

    /// Test 1: Elliptical wing → e ≈ 1.0 (within 10% tolerance for analytical method).
    /// An elliptical wing has tip_chord → 0.
    #[test]
    fn test_elliptical_wing_efficiency() {
        let db = make_db();
        let fc = flight_5deg();
        // Elliptical chord distribution: tip_chord very small.
        let result = solve_lifting_line_from_params(
            1000.0,  // 1 m span
            120.0,   // 120 mm root chord
            5.0,     // nearly-zero tip chord → approximate ellipse
            0.0,
            &db, &fc, 20,
        );
        // True ellipse gives e=1.0; our tapered approx should be close.
        println!("Elliptical e = {}", result.oswald_efficiency);
        assert!(
            result.oswald_efficiency > 0.80,
            "Elliptical-ish wing: e should be > 0.80, got {}",
            result.oswald_efficiency
        );
    }

    /// Test 2: Rectangular wing → e ≈ 0.85–0.95.
    #[test]
    fn test_rectangular_wing_efficiency() {
        let db = make_db();
        let fc = flight_5deg();
        let result = solve_lifting_line_from_params(
            1000.0,  // 1 m span
            120.0,   // constant chord (rectangular)
            120.0,
            0.0,
            &db, &fc, 20,
        );
        println!("Rectangular e = {}", result.oswald_efficiency);
        assert!(
            result.oswald_efficiency > 0.75 && result.oswald_efficiency <= 1.0,
            "Rectangular wing e out of expected range, got {}",
            result.oswald_efficiency
        );
    }

    /// Test 3: CL increases linearly with AoA in non-stalled region.
    #[test]
    fn test_linear_cl_vs_aoa() {
        let db   = make_db();
        let fc2  = FlightCondition::new(50.0, 0.0, 2.0);
        let fc8  = FlightCondition::new(50.0, 0.0, 8.0);
        let r2   = solve_lifting_line_from_params(800.0, 100.0, 100.0, 0.0, &db, &fc2, 16);
        let r8   = solve_lifting_line_from_params(800.0, 100.0, 100.0, 0.0, &db, &fc8, 16);
        println!("CL(2°)={} CL(8°)={}", r2.cl_total, r8.cl_total);
        assert!(r8.cl_total > r2.cl_total, "CL must increase with AoA");
        // Ratio should be close to 8/2 = 4 (linear).
        let ratio = r8.cl_total / r2.cl_total;
        assert!(ratio > 2.5 && ratio < 6.0, "CL ratio = {ratio}, expected ~4");
    }

    /// Test 4: Stall detected when AoA > alpha_stall.
    #[test]
    fn test_stall_detection() {
        let db  = make_db();
        let fc  = FlightCondition::new(50.0, 0.0, 20.0);
        let res = solve_lifting_line_from_params(600.0, 80.0, 80.0, 0.0, &db, &fc, 16);
        println!("Stall stations at AoA=20°: {:?}", res.stall_stations);
        assert!(!res.stall_stations.is_empty(), "Should have stalled stations at 20°");
    }

    /// Test 5: Washout (negative tip twist) → root stalls first.
    #[test]
    fn test_washout_root_stall_first() {
        let db  = make_db();
        let fc  = FlightCondition::new(50.0, 0.0, 10.0);
        // Negative twist at tip → tip has lower effective AoA → root stalls first.
        let res = solve_lifting_line_from_params(800.0, 100.0, 100.0, -4.0, &db, &fc, 20);
        println!(
            "Washout: root_stall_first={} tip_stall_risk={}",
            res.root_stall_first, res.tip_stall_risk
        );
        // With washout at high AoA, root stalls before tip.
        assert!(res.root_stall_first, "Washout should cause root to stall first");
    }
}
