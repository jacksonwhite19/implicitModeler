// Analytical NACA polar generation using thin airfoil theory with realistic corrections.
//
// Reference geometry:
//   Alpha range: -20° to +20° at 0.5° increments (81 points).
//   CL = CL_alpha * (alpha - alpha_0)  in linear region, with soft stall rollover.
//   CD = CD0 + CL² / (pi * AR_eff)    parabolic drag polar (AR_eff = 8 approximation).
//   CM = -0.25 * camber_fraction       quarter-chord moment.

use super::polars::AirfoilPolar;

/// Parse a NACA designation string and return (max_camber_frac, camber_pos, thickness_frac).
///
/// Handles:
///   - 4-digit: NACA MPXX  → camber=M/100, pos=P/10, t=XX/100
///   - 5-digit: NACA LPQXX → camber approx
///   - 6-series: NACA 63-412, 64-412 → treated as cambered thin profiles
fn parse_naca(designation: &str) -> (f32, f32, f32) {
    // Strip "NACA " prefix and any whitespace.
    let s = designation
        .to_uppercase()
        .replace("NACA ", "")
        .replace("NACA", "")
        .trim()
        .to_string();

    // 6-series like "63-412" or "64-412"
    if s.contains('-') {
        let parts: Vec<&str> = s.split('-').collect();
        if parts.len() == 2 {
            let digits = parts[1];
            if digits.len() >= 3 {
                let camber: f32 = digits[0..1].parse().unwrap_or(0) as f32 / 10.0;
                let thickness: f32 = digits[digits.len()-2..].parse().unwrap_or(12) as f32 / 100.0;
                return (camber * 0.01, 0.4, thickness);
            }
        }
        return (0.0, 0.0, 0.12);
    }

    // 5-digit (230XX)
    if s.len() == 5 {
        let t: f32 = s[3..5].parse().unwrap_or(12) as f32 / 100.0;
        let camber_line: u32 = s[0..1].parse().unwrap_or(2);
        // 230-series: design CL ≈ 0.3, ideal AoA ≈ 1.5°
        let camber_frac = (camber_line as f32) * 0.006;
        return (camber_frac, 0.3, t);
    }

    // 4-digit (MPXX)
    if s.len() == 4 {
        let m: f32 = s[0..1].parse().unwrap_or(0) as f32 / 100.0;
        let p: f32 = s[1..2].parse().unwrap_or(0) as f32 / 10.0;
        let t: f32 = s[2..4].parse().unwrap_or(12) as f32 / 100.0;
        return (m, p, t);
    }

    (0.0, 0.0, 0.12)
}

/// Generate an analytical NACA polar for any designation and Reynolds number.
///
/// Uses thin airfoil theory with empirical stall and drag corrections.
pub fn generate_naca_polar(designation: &str, reynolds: f32) -> AirfoilPolar {
    let (camber, _camber_pos, thickness) = parse_naca(designation);

    // Thin airfoil CL_alpha base (2π per radian = 0.10966 per degree).
    let cl_alpha_per_deg_base = 2.0 * std::f32::consts::PI / 180.0;

    // Correct for thickness: Prandtl-Glauert-ish viscous correction (small for thin airfoils).
    // Approximate: cl_alpha ≈ 2π * (1 + 0.77*t) (Abbott & von Doenhoff).
    let cl_alpha_per_deg = cl_alpha_per_deg_base * (1.0 + 0.77 * thickness);

    // Zero-lift angle (degrees).  For symmetric: 0.  For cambered: ≈ -camber * 4 * 10.
    // More precisely: alpha_0 ≈ -2 * camber / (camber_pos + 0.01) * ... simplified:
    let alpha_0 = if camber > 1e-4 {
        // Thin airfoil: alpha_0 = -2 * max_camber / camber_position (rough)
        -(camber * 40.0)  // ≈ −camber_frac * 40 degrees
    } else {
        0.0_f32
    };

    // Stall angle above linear region.
    let alpha_stall = if camber > 1e-4 {
        // Cambered profiles stall at higher total alpha but lower from zero-lift.
        14.0 + (thickness - 0.12) * 30.0  // thicker = higher stall
    } else {
        // Symmetric
        12.0 + (thickness - 0.09) * 40.0
    };
    let alpha_stall = alpha_stall.clamp(10.0, 20.0);

    // CD0: parasite drag. Depends on thickness and Reynolds number.
    // Laminar Re correction: CD0 ~ 1/sqrt(Re) * 2.66 (flat plate approx).
    let re_factor = (500_000.0_f32 / reynolds.max(50_000.0)).sqrt().min(3.0);
    let cd0_base  = 0.006 + thickness * 0.04;   // baseline
    let cd0       = cd0_base * re_factor.powf(0.4);

    // AR_eff for induced drag in 2D (use AR=8 approximation for section polar).
    let pi_ar_eff = std::f32::consts::PI * 8.0;

    // Build table.
    let n_points = 81_usize;
    let alpha_start = -20.0_f32;
    let d_alpha     =  0.5_f32;

    let mut alpha_vec = Vec::with_capacity(n_points);
    let mut cl_vec    = Vec::with_capacity(n_points);
    let mut cd_vec    = Vec::with_capacity(n_points);
    let mut cm_vec    = Vec::with_capacity(n_points);

    for i in 0..n_points {
        let alpha = alpha_start + i as f32 * d_alpha;
        alpha_vec.push(alpha);

        // Linear CL.
        let cl_linear = cl_alpha_per_deg * (alpha - alpha_0);

        // Stall model: smooth rollover using tanh blending.
        // Below stall: linear.  Above stall: transition to plateau then drop.
        let cl_max_lin = cl_alpha_per_deg * (alpha_stall - alpha_0);

        let cl = if alpha <= alpha_stall {
            cl_linear
        } else {
            let da = alpha - alpha_stall;
            // Soft stall: Cl drops with progressive separation.
            let drop_rate = 0.04;  // CL drop per degree post-stall
            let cl_post   = cl_max_lin - da * drop_rate * cl_max_lin.abs();
            cl_post.max(-cl_max_lin.abs() * 0.3)
        };
        // Mirror negative stall symmetrically.
        let alpha_stall_neg = -alpha_stall;
        let cl = if alpha < alpha_stall_neg {
            let da  = alpha_stall_neg - alpha;
            let cl_min_lin = cl_alpha_per_deg * (alpha_stall_neg - alpha_0);
            let drop_rate  = 0.04;
            let cl_post    = cl_min_lin + da * drop_rate * cl_min_lin.abs();
            cl_post.min(-cl_min_lin.abs() * 0.3).max(cl)
        } else {
            cl
        };
        cl_vec.push(cl);

        // CD: parabolic polar with high-alpha spike.
        let cd_induced  = cl * cl / pi_ar_eff;
        let alpha_rad   = alpha.to_radians();
        // Post-stall form drag increase.
        let cd_form     = if alpha.abs() > alpha_stall {
            let da = (alpha.abs() - alpha_stall) * 0.07;
            da * da
        } else {
            0.0
        };
        let cd = (cd0 + cd_induced + cd_form).max(0.005);
        cd_vec.push(cd);

        // CM: constant ≈ -0.25 * camber (thin airfoil theory).
        let cm_base = -0.25 * camber * 4.0;
        // Small variation with alpha (∂Cm/∂α ≈ 0).
        let _ = alpha_rad;
        cm_vec.push(cm_base);
    }

    AirfoilPolar::from_data(designation.to_string(), reynolds, alpha_vec, cl_vec, cd_vec, cm_vec)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_symmetric_zero_lift() {
        let p = generate_naca_polar("NACA 0012", 500_000.0);
        // Symmetric airfoil: zero lift at alpha=0.
        assert!((p.cl_at(0.0)).abs() < 0.05, "CL at 0° should be ~0 for symmetric");
        assert!((p.alpha_zero_lift_deg).abs() < 0.5);
    }

    #[test]
    fn test_cambered_negative_alpha0() {
        let p = generate_naca_polar("NACA 4412", 500_000.0);
        // 4412 has significant camber, alpha_0 should be negative.
        assert!(p.alpha_zero_lift_deg < -1.0, "NACA 4412 alpha_0 should be negative");
    }

    #[test]
    fn test_stall_detected() {
        let p = generate_naca_polar("NACA 0012", 500_000.0);
        assert!(!p.is_stalled(8.0));
        assert!(p.is_stalled(20.0));
    }

    #[test]
    fn test_cd_positive() {
        let p = generate_naca_polar("NACA 2412", 200_000.0);
        for &cd in &p.cd { assert!(cd > 0.0, "CD must be positive"); }
    }
}
