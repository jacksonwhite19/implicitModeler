// CG sensitivity analysis — analytical partial derivatives of CG with respect
// to component positions and dimension parameters.
#![allow(dead_code)] // Analysis result fields — not all displayed in the current UI

use glam::Vec3;
use indexmap::IndexMap;

#[derive(Clone, Debug)]
pub struct ComponentCgSensitivity {
    pub component_name: String,
    pub component_mass_g: f32,
    pub current_position: Vec3,
    pub dcg_dx_mm_per_mm: f32,
    pub dcg_dy_mm_per_mm: f32,
    pub dcg_dz_mm_per_mm: f32,
    pub influence_fraction: f32,
    pub forward_limit_mm: f32,
    pub aft_limit_mm: f32,
}

#[derive(Clone, Debug)]
pub struct DimensionCgSensitivity {
    pub dimension_name: String,
    pub current_value: f64,
    pub dcg_x_per_unit: f32,
    pub dcg_z_per_unit: f32,
    pub dstatic_margin_per_unit: f32,
    pub sensitivity_rating: SensitivityRating,
}

#[derive(Clone, Debug, PartialEq)]
pub enum SensitivityRating {
    Low,      // < 0.5mm per unit
    Medium,   // 0.5 - 2mm
    High,     // 2 - 5mm
    Critical, // > 5mm
}

impl SensitivityRating {
    pub fn from_magnitude(mm_per_unit: f32) -> Self {
        let v = mm_per_unit.abs();
        if v < 0.5 {
            SensitivityRating::Low
        } else if v < 2.0 {
            SensitivityRating::Medium
        } else if v < 5.0 {
            SensitivityRating::High
        } else {
            SensitivityRating::Critical
        }
    }
}

#[derive(Clone, Debug)]
pub struct CgEnvelope {
    pub forward_limit_x_mm: f32,
    pub aft_limit_x_mm: f32,
    pub current_x_mm: f32,
    pub margin_to_forward_limit_mm: f32,
    pub margin_to_aft_limit_mm: f32,
    /// 0% = at forward limit, 100% = at aft limit
    pub percent_through_envelope: f32,
}

#[derive(Clone, Debug)]
pub struct CgSensitivityResult {
    pub baseline_cg: Vec3,
    pub baseline_static_margin_mac: f32,
    pub component_sensitivities: Vec<ComponentCgSensitivity>,
    pub dimension_sensitivities: Vec<DimensionCgSensitivity>,
    pub cg_envelope: CgEnvelope,
    pub recommendations: Vec<String>,
}

/// Compute CG sensitivity for a set of components with known positions and masses.
///
/// - `components`: `(name, position, mass_g)` tuples
/// - `dimensions`: named dimensions from the script (heuristic analysis only)
/// - `neutral_point_x_mm`: aerodynamic neutral point position (aft CG limit)
/// - `wing_mac`: mean aerodynamic chord in mm
/// - `forward_limit_x_mm`: forward CG limit (typically NP − 0.25·MAC)
pub fn compute_cg_sensitivity(
    components: &[(String, Vec3, f32)],
    dimensions: &IndexMap<String, f64>,
    neutral_point_x_mm: f32,
    wing_mac: f32,
    forward_limit_x_mm: f32,
) -> CgSensitivityResult {
    let total_mass: f32 = components.iter().map(|(_, _, m)| m).sum();

    // Degenerate case — no mass.
    if total_mass <= 0.0 || components.is_empty() {
        return CgSensitivityResult {
            baseline_cg: Vec3::ZERO,
            baseline_static_margin_mac: 0.0,
            component_sensitivities: vec![],
            dimension_sensitivities: vec![],
            cg_envelope: CgEnvelope {
                forward_limit_x_mm,
                aft_limit_x_mm: neutral_point_x_mm,
                current_x_mm: 0.0,
                margin_to_forward_limit_mm: 0.0,
                margin_to_aft_limit_mm: neutral_point_x_mm,
                percent_through_envelope: 0.0,
            },
            recommendations: vec!["No mass points defined. Declare components with mass_g to enable CG analysis.".to_string()],
        };
    }

    // Baseline CG
    let baseline_cg = {
        let sum_mx: f32 = components.iter().map(|(_, p, m)| p.x * m).sum();
        let sum_my: f32 = components.iter().map(|(_, p, m)| p.y * m).sum();
        let sum_mz: f32 = components.iter().map(|(_, p, m)| p.z * m).sum();
        Vec3::new(sum_mx / total_mass, sum_my / total_mass, sum_mz / total_mass)
    };

    // Static margin: (NP - CG) / MAC. Positive = stable.
    let baseline_static_margin_mac = if wing_mac > 0.0 {
        (neutral_point_x_mm - baseline_cg.x) / wing_mac
    } else {
        0.0
    };

    // ── Component sensitivities ──────────────────────────────────────────────
    let component_sensitivities: Vec<ComponentCgSensitivity> = components
        .iter()
        .map(|(name, pos, mass)| {
            // Analytical: dCG/dx_i = m_i / M_total  (same for y, z)
            let dcg_dx = mass / total_mass;

            // Sum of moments from all OTHER components
            let sum_other_x: f32 = components
                .iter()
                .filter(|(n, _, _)| n != name)
                .map(|(_, p, m)| p.x * m)
                .sum();

            // CG = (comp_x * mass + sum_other) / total_mass = forward_limit
            // → comp_x = (limit * total_mass - sum_other) / mass
            let (fwd_comp_x, aft_comp_x) = if *mass > 1e-9 {
                let fwd = (forward_limit_x_mm * total_mass - sum_other_x) / mass;
                let aft = (neutral_point_x_mm * total_mass - sum_other_x) / mass;
                (fwd, aft)
            } else {
                (pos.x, pos.x)
            };

            ComponentCgSensitivity {
                component_name: name.clone(),
                component_mass_g: *mass,
                current_position: *pos,
                dcg_dx_mm_per_mm: dcg_dx,
                dcg_dy_mm_per_mm: dcg_dx, // same formula by symmetry
                dcg_dz_mm_per_mm: dcg_dx,
                influence_fraction: mass / total_mass,
                forward_limit_mm: fwd_comp_x,
                aft_limit_mm: aft_comp_x,
            }
        })
        .collect();

    // ── Dimension sensitivities (heuristic) ──────────────────────────────────
    let dimension_sensitivities: Vec<DimensionCgSensitivity> = dimensions
        .iter()
        .map(|(name, &value)| {
            let dcg_x = estimate_dimension_cg_sensitivity(name, value as f32, components, total_mass, baseline_cg.x);
            let rating = SensitivityRating::from_magnitude(dcg_x);

            // Rough estimate of static margin sensitivity: −dcg_x / MAC
            let dsm = if wing_mac > 0.0 { -dcg_x / wing_mac } else { 0.0 };

            DimensionCgSensitivity {
                dimension_name: name.clone(),
                current_value: value,
                dcg_x_per_unit: dcg_x,
                dcg_z_per_unit: 0.0, // not estimated without geometry model
                dstatic_margin_per_unit: dsm,
                sensitivity_rating: rating,
            }
        })
        .collect();

    // ── CG envelope ─────────────────────────────────────────────────────────
    let envelope_span = (neutral_point_x_mm - forward_limit_x_mm).abs().max(1e-9);
    let pct = ((baseline_cg.x - forward_limit_x_mm) / envelope_span * 100.0).clamp(0.0, 100.0);

    let cg_envelope = CgEnvelope {
        forward_limit_x_mm,
        aft_limit_x_mm: neutral_point_x_mm,
        current_x_mm: baseline_cg.x,
        margin_to_forward_limit_mm: (baseline_cg.x - forward_limit_x_mm).max(0.0),
        margin_to_aft_limit_mm: (neutral_point_x_mm - baseline_cg.x).max(0.0),
        percent_through_envelope: pct,
    };

    // ── Recommendations ──────────────────────────────────────────────────────
    let mut recommendations = Vec::new();

    for dim in &dimension_sensitivities {
        if dim.sensitivity_rating == SensitivityRating::Critical {
            recommendations.push(format!(
                "Dimension '{}' has critical CG influence ({:.1}mm/unit). Consider constraining this dimension.",
                dim.dimension_name, dim.dcg_x_per_unit
            ));
        }
    }

    if pct < 20.0 {
        recommendations.push(format!(
            "CG is very forward ({:.0}% through envelope). Consider moving heavy components aft.",
            pct
        ));
    } else if pct > 80.0 {
        recommendations.push(format!(
            "CG is very aft ({:.0}% through envelope). Stability margin is small.",
            pct
        ));
    }

    for comp in &component_sensitivities {
        if comp.influence_fraction > 0.40 {
            recommendations.push(format!(
                "Component '{}' ({:.0}% of total mass) dominates CG. Its placement is critical.",
                comp.component_name,
                comp.influence_fraction * 100.0
            ));
        }
    }

    CgSensitivityResult {
        baseline_cg,
        baseline_static_margin_mac,
        component_sensitivities,
        dimension_sensitivities,
        cg_envelope,
        recommendations,
    }
}

/// Heuristic estimate of how much a named dimension shifts the CG in X (mm per unit change).
/// This is intentionally conservative and name-based; exact computation would require
/// re-running the script (not available in this module).
fn estimate_dimension_cg_sensitivity(
    dim_name: &str,
    _value: f32,
    components: &[(String, Vec3, f32)],
    total_mass: f32,
    _baseline_cg_x: f32,
) -> f32 {
    let name_lower = dim_name.to_lowercase();

    // Estimate fraction of total mass that moves with this dimension
    let moving_mass_fraction: f32 = if name_lower.contains("fuse") || name_lower.contains("length") {
        // Fuselage length changes push aft components
        // Approximate: tail components (aft half) move with fuselage stretch
        let aft_mass: f32 = components.iter()
            .filter(|(_, p, _)| p.x > _baseline_cg_x)
            .map(|(_, _, m)| *m)
            .sum();
        if total_mass > 1e-9 { aft_mass / total_mass } else { 0.3 }
    } else if name_lower.contains("battery") || name_lower.contains("bat_") || name_lower.contains("_bat") {
        // Battery position: assume battery is ~30% of total mass moving 1:1
        0.30
    } else if name_lower.contains("motor") || name_lower.contains("arm") {
        0.10
    } else if name_lower.contains("x") || name_lower.contains("pos") || name_lower.contains("offset") {
        // Direct position parameter — affects some component
        0.15
    } else {
        // Default: negligible direct CG effect
        0.05
    };

    // dcg_x ≈ moving_mass_fraction mm per mm of dimension change
    moving_mass_fraction
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_component_sensitivity_analytical() {
        // Two components: 100g at x=0, 100g at x=100
        // CG at x=50. Moving first component by 10mm shifts CG by 5mm (50% of mass * 10mm)
        let components = vec![
            ("front".to_string(), Vec3::new(0.0, 0.0, 0.0), 100.0f32),
            ("rear".to_string(), Vec3::new(100.0, 0.0, 0.0), 100.0f32),
        ];
        let dims = IndexMap::new();
        let result = compute_cg_sensitivity(&components, &dims, 80.0, 30.0, 72.5);
        let front_sens = result.component_sensitivities.iter()
            .find(|c| c.component_name == "front").unwrap();
        assert!(
            (front_sens.dcg_dx_mm_per_mm - 0.5).abs() < 0.01,
            "Expected dcg_dx = 0.5, got {}",
            front_sens.dcg_dx_mm_per_mm
        );
    }

    #[test]
    fn test_cg_baseline_computation() {
        let components = vec![
            ("a".to_string(), Vec3::new(20.0, 0.0, 0.0), 200.0f32),
            ("b".to_string(), Vec3::new(80.0, 0.0, 0.0), 200.0f32),
        ];
        let dims = IndexMap::new();
        let result = compute_cg_sensitivity(&components, &dims, 60.0, 20.0, 55.0);
        assert!(
            (result.baseline_cg.x - 50.0).abs() < 0.1,
            "Expected CG at x=50, got {}",
            result.baseline_cg.x
        );
    }

    #[test]
    fn test_cg_envelope_forward_of_np() {
        // CG at 40mm, NP at 60mm, mac=20 → forward_limit=55, aft_limit=60
        let components = vec![
            ("a".to_string(), Vec3::new(40.0, 0.0, 0.0), 100.0f32),
        ];
        let dims = IndexMap::new();
        let result = compute_cg_sensitivity(&components, &dims, 60.0, 20.0, 55.0);
        assert!(
            result.cg_envelope.current_x_mm < result.cg_envelope.aft_limit_x_mm,
            "CG should be forward of NP"
        );
        assert!(
            result.cg_envelope.margin_to_aft_limit_mm > 0.0,
            "Should have positive margin to aft limit"
        );
    }

    #[test]
    fn test_empty_components() {
        let result = compute_cg_sensitivity(&[], &IndexMap::new(), 100.0, 30.0, 92.5);
        assert!(result.component_sensitivities.is_empty());
        assert!(!result.recommendations.is_empty());
    }

    #[test]
    fn test_dominant_component_recommendation() {
        let components = vec![
            ("battery".to_string(), Vec3::new(50.0, 0.0, 0.0), 450.0f32),
            ("frame".to_string(),   Vec3::new(50.0, 0.0, 0.0),  50.0f32),
        ];
        let result = compute_cg_sensitivity(&components, &IndexMap::new(), 80.0, 30.0, 72.5);
        let has_dominant_rec = result.recommendations.iter()
            .any(|r| r.contains("battery") && r.contains("dominates"));
        assert!(has_dominant_rec, "Should warn about dominant battery component");
    }
}
