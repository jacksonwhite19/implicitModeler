// Airfoil polar data structures and interpolation
#![allow(dead_code)] // Aero data model — not all fields displayed in current UI

use std::collections::HashMap;
use std::path::Path;

/// Complete aerodynamic polar for one airfoil at one Reynolds number.
#[derive(Clone, Debug)]
pub struct AirfoilPolar {
    pub designation:       String,
    pub reynolds_number:   f32,
    /// Angle of attack in degrees, ascending order.
    pub alpha_deg:         Vec<f32>,
    pub cl:                Vec<f32>,
    pub cd:                Vec<f32>,
    pub cm:                Vec<f32>,
    pub cl_max:            f32,
    pub alpha_stall_deg:   f32,
    pub alpha_zero_lift_deg: f32,
    /// Lift-curve slope per radian.
    pub cl_alpha:          f32,
    /// Lift-curve slope per degree.
    pub cl_alpha_per_deg:  f32,
}

impl AirfoilPolar {
    /// Create a polar from raw tabular data, computing derived quantities automatically.
    pub fn from_data(
        designation: String,
        reynolds: f32,
        alpha: Vec<f32>,
        cl: Vec<f32>,
        cd: Vec<f32>,
        cm: Vec<f32>,
    ) -> Self {
        assert_eq!(alpha.len(), cl.len());
        assert_eq!(alpha.len(), cd.len());
        assert_eq!(alpha.len(), cm.len());

        // Find CL_max and stall angle (first local maximum or largest CL).
        let (cl_max, alpha_stall_deg) = Self::find_stall(&alpha, &cl);

        // Alpha at zero lift: interpolate CL=0 crossing.
        let alpha_zero_lift_deg = Self::find_zero_lift(&alpha, &cl);

        // CL_alpha: linear regression in -5..+5 deg range.
        let cl_alpha_per_deg = Self::compute_cl_alpha(&alpha, &cl);
        let cl_alpha = cl_alpha_per_deg * (180.0 / std::f32::consts::PI);

        Self {
            designation,
            reynolds_number: reynolds,
            alpha_deg: alpha,
            cl,
            cd,
            cm,
            cl_max,
            alpha_stall_deg,
            alpha_zero_lift_deg,
            cl_alpha,
            cl_alpha_per_deg,
        }
    }

    /// CL at given alpha (degrees). Linearly interpolated; clamped at bounds.
    pub fn cl_at(&self, alpha_deg: f32) -> f32 {
        Self::interp(&self.alpha_deg, &self.cl, alpha_deg)
    }

    /// CD at given alpha (degrees). Linearly interpolated; clamped at bounds.
    pub fn cd_at(&self, alpha_deg: f32) -> f32 {
        Self::interp(&self.alpha_deg, &self.cd, alpha_deg)
    }

    /// CM at given alpha (degrees). Linearly interpolated; clamped at bounds.
    pub fn cm_at(&self, alpha_deg: f32) -> f32 {
        Self::interp(&self.alpha_deg, &self.cm, alpha_deg)
    }

    /// Returns true when operating beyond alpha_stall (CL < CL_max).
    pub fn is_stalled(&self, alpha_deg: f32) -> bool {
        alpha_deg > self.alpha_stall_deg
    }

    // ── Private helpers ──────────────────────────────────────────────────────

    fn interp(xs: &[f32], ys: &[f32], x: f32) -> f32 {
        if xs.is_empty() { return 0.0; }
        if x <= xs[0]  { return ys[0]; }
        if x >= xs[xs.len() - 1] { return ys[ys.len() - 1]; }
        // Binary search for bracket.
        let idx = xs.partition_point(|&v| v <= x);
        let i   = idx.saturating_sub(1);
        let j   = idx.min(xs.len() - 1);
        if i == j { return ys[i]; }
        let t = (x - xs[i]) / (xs[j] - xs[i]);
        ys[i] * (1.0 - t) + ys[j] * t
    }

    fn find_stall(alpha: &[f32], cl: &[f32]) -> (f32, f32) {
        if cl.is_empty() { return (1.0, 15.0); }
        let mut best_i = 0;
        for (i, &v) in cl.iter().enumerate() {
            if v > cl[best_i] { best_i = i; }
        }
        (cl[best_i], alpha[best_i])
    }

    fn find_zero_lift(alpha: &[f32], cl: &[f32]) -> f32 {
        // Scan for sign change.
        for i in 0..cl.len().saturating_sub(1) {
            if cl[i] <= 0.0 && cl[i + 1] >= 0.0 {
                let t = -cl[i] / (cl[i + 1] - cl[i]);
                return alpha[i] + t * (alpha[i + 1] - alpha[i]);
            }
        }
        // Fallback: first negative alpha that hits zero.
        *alpha.first().unwrap_or(&0.0)
    }

    fn compute_cl_alpha(alpha: &[f32], cl: &[f32]) -> f32 {
        // Use points in -5..+5 deg range.
        let pairs: Vec<(f32, f32)> = alpha.iter().zip(cl.iter())
            .filter(|(a, _)| **a >= -5.0 && **a <= 5.0)
            .map(|(a, c)| (*a, *c))
            .collect();
        if pairs.len() < 2 { return 0.1097; } // 2π/deg
        let n   = pairs.len() as f32;
        let sx  = pairs.iter().map(|(a, _)| a).sum::<f32>();
        let sy  = pairs.iter().map(|(_, c)| c).sum::<f32>();
        let sxx = pairs.iter().map(|(a, _)| a * a).sum::<f32>();
        let sxy = pairs.iter().map(|(a, c)| a * c).sum::<f32>();
        let denom = n * sxx - sx * sx;
        if denom.abs() < 1e-10 { return 0.1097; }
        (n * sxy - sx * sy) / denom
    }
}

// ── Polar database ────────────────────────────────────────────────────────────

/// Collection of airfoil polars keyed by (designation, reynolds_number_rounded).
pub struct PolarDatabase {
    pub polars:      HashMap<(String, u32), AirfoilPolar>,
    pub user_polars: HashMap<String, AirfoilPolar>,
}

impl PolarDatabase {
    /// Initialise with all analytically-generated standard polars.
    pub fn new() -> Self {
        use super::polar_data::generate_naca_polar;

        let profiles = &[
            "NACA 0006", "NACA 0009", "NACA 0012", "NACA 0015", "NACA 0018",
            "NACA 2412", "NACA 2415",
            "NACA 4412", "NACA 4415",
            "NACA 23012", "NACA 23015",
            "NACA 63-412", "NACA 64-412",
        ];
        let reynolds_list = [200_000_f32, 500_000_f32];

        let mut polars = HashMap::new();
        for &desig in profiles {
            for &re in &reynolds_list {
                let polar = generate_naca_polar(desig, re);
                let key   = (desig.to_string(), Self::round_re(re));
                polars.insert(key, polar);
            }
        }

        Self { polars, user_polars: HashMap::new() }
    }

    /// Retrieve a stored polar at exact (rounded) Reynolds number.
    pub fn get(&self, designation: &str, reynolds: f32) -> Option<&AirfoilPolar> {
        let key = (designation.to_string(), Self::round_re(reynolds));
        self.polars.get(&key)
            .or_else(|| self.user_polars.get(designation))
    }

    /// Retrieve or interpolate between stored Reynolds numbers.
    /// Falls back to nearest stored if out of range.
    pub fn get_interpolated(&self, designation: &str, reynolds: f32) -> Option<AirfoilPolar> {
        // Collect all stored polars for this designation.
        let mut candidates: Vec<(&AirfoilPolar, f32)> = self.polars.iter()
            .filter(|((d, _), _)| d == designation)
            .map(|(_, p)| (p, p.reynolds_number))
            .collect();
        candidates.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());

        if candidates.is_empty() { return None; }
        if candidates.len() == 1 { return Some(candidates[0].0.clone()); }

        // Check exact match.
        if let Some((p, _)) = candidates.iter().find(|(_, r)| (*r - reynolds).abs() < 1000.0) {
            return Some((*p).clone());
        }

        // Find bracket.
        let lo_idx = candidates.partition_point(|(_, r)| *r < reynolds).saturating_sub(1);
        let hi_idx = (lo_idx + 1).min(candidates.len() - 1);
        if lo_idx == hi_idx { return Some(candidates[lo_idx].0.clone()); }

        let (p_lo, re_lo) = candidates[lo_idx];
        let (p_hi, re_hi) = candidates[hi_idx];
        let t = (reynolds - re_lo) / (re_hi - re_lo);

        // Interpolate table values.
        let n = p_lo.alpha_deg.len();
        let mut cl_interp = vec![0.0_f32; n];
        let mut cd_interp = vec![0.0_f32; n];
        let mut cm_interp = vec![0.0_f32; n];
        for i in 0..n {
            // Use lo table alpha; hi values are interpolated onto same alpha grid.
            let a = p_lo.alpha_deg[i];
            cl_interp[i] = p_lo.cl[i] * (1.0 - t) + p_hi.cl_at(a) * t;
            cd_interp[i] = p_lo.cd[i] * (1.0 - t) + p_hi.cd_at(a) * t;
            cm_interp[i] = p_lo.cm[i] * (1.0 - t) + p_hi.cm_at(a) * t;
        }

        Some(AirfoilPolar::from_data(
            designation.to_string(),
            reynolds,
            p_lo.alpha_deg.clone(),
            cl_interp,
            cd_interp,
            cm_interp,
        ))
    }

    /// Parse a simple CSV user polar: columns alpha, CL, CD, CM.
    pub fn load_user_polar(path: &Path) -> Result<AirfoilPolar, String> {
        let content = std::fs::read_to_string(path)
            .map_err(|e| format!("Cannot read polar file: {e}"))?;

        let designation = path.file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("user")
            .to_string();

        let mut alpha = Vec::new();
        let mut cl    = Vec::new();
        let mut cd    = Vec::new();
        let mut cm    = Vec::new();

        for line in content.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') { continue; }
            let cols: Vec<f32> = line.split(',')
                .filter_map(|s| s.trim().parse().ok())
                .collect();
            if cols.len() >= 3 {
                alpha.push(cols[0]);
                cl.push(cols[1]);
                cd.push(cols[2]);
                cm.push(if cols.len() >= 4 { cols[3] } else { 0.0 });
            }
        }

        if alpha.is_empty() {
            return Err("No valid data rows found in polar file".to_string());
        }

        Ok(AirfoilPolar::from_data(designation, 0.0, alpha, cl, cd, cm))
    }

    pub fn register_user_polar(&mut self, name: String, polar: AirfoilPolar) {
        self.user_polars.insert(name, polar);
    }

    fn round_re(re: f32) -> u32 {
        // Round to nearest 50k for keying purposes.
        ((re / 50_000.0).round() as u32) * 50_000
    }
}

impl Default for PolarDatabase {
    fn default() -> Self { Self::new() }
}
