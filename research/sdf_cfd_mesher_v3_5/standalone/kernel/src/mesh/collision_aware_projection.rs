use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct CollisionAwareProjectionConfig {
    pub residual_tolerance: f64,
    pub extraction_accuracy_fallback_tolerance: f64,
    pub maximum_iterations: usize,
    pub line_search_reduction_factor: f64,
    pub minimum_line_search_fraction: f64,
    pub maximum_displacement_fraction_local_edge: f64,
    pub maximum_displacement_fraction_local_cell: f64,
    pub maximum_displacement_fraction_feature_size: f64,
    pub maximum_displacement_fraction_sector_thickness: f64,
    pub maximum_displacement_fraction_competing_sector: f64,
    pub absolute_displacement_limit: Option<f64>,
    pub minimum_gradient_magnitude: f64,
    pub minimum_triangle_area_ratio: f64,
    pub maximum_allowed_normal_deviation_degrees: f64,
}

impl Default for CollisionAwareProjectionConfig {
    fn default() -> Self {
        Self {
            residual_tolerance: 1.0e-4,
            extraction_accuracy_fallback_tolerance: 0.85,
            maximum_iterations: 32,
            line_search_reduction_factor: 0.5,
            minimum_line_search_fraction: 0.03125,
            maximum_displacement_fraction_local_edge: 0.45,
            maximum_displacement_fraction_local_cell: 0.35,
            maximum_displacement_fraction_feature_size: 0.25,
            maximum_displacement_fraction_sector_thickness: 0.20,
            maximum_displacement_fraction_competing_sector: 0.35,
            absolute_displacement_limit: Some(0.45),
            minimum_gradient_magnitude: 1.0e-6,
            minimum_triangle_area_ratio: 0.08,
            maximum_allowed_normal_deviation_degrees: 115.0,
        }
    }
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct ProjectionScaleAvailability {
    pub absolute_limit: Option<f64>,
    pub local_edge_length: Option<f64>,
    pub local_cell_size: Option<f64>,
    pub feature_size: Option<f64>,
    pub sector_thickness: Option<f64>,
    pub competing_sector_distance: Option<f64>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct TrustRegionResult {
    pub maximum_displacement: f64,
    pub limiting_scale: String,
    pub unavailable_scales: Vec<String>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ProjectionOutcome {
    FullyProjected,
    PartiallyProjected,
    RetainedRaw,
    Failed,
}

#[derive(Clone, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ProjectionRejectionReason {
    InvalidResidual,
    InvalidGradient,
    TrustRegionExceeded,
    IncompatibleSector,
    IncompatibleFeature,
    IncompatibleProvenance,
    BranchCrossing,
    LocalTriangleInvalid,
    LocalCollision,
    GlobalCollisionRollback,
    SevereNormalDisagreementRollback,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct ProjectionCandidateRecord {
    pub candidate_source: String,
    pub residual: f64,
    pub displacement: f64,
    pub gradient_magnitude: Option<f64>,
    pub sector_id: Option<String>,
    pub feature_id: Option<String>,
    pub provenance_role: Option<String>,
    pub accepted_fraction: Option<f64>,
    pub rejection_reasons: Vec<ProjectionRejectionReason>,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct SurfaceProjectionRecord {
    pub vertex_id: u32,
    pub expected_sector: Option<String>,
    pub expected_feature: Option<String>,
    pub expected_hard_csg_provenance: Option<String>,
    pub candidates: Vec<ProjectionCandidateRecord>,
    pub selected_candidate_index: Option<usize>,
    pub final_residual: f64,
    pub total_displacement: f64,
    pub outcome: Option<ProjectionOutcome>,
    pub rejection_reasons: Vec<ProjectionRejectionReason>,
}

pub fn trust_region_limit(
    config: &CollisionAwareProjectionConfig,
    scales: &ProjectionScaleAvailability,
) -> TrustRegionResult {
    let mut candidates = Vec::<(String, f64)>::new();
    let mut unavailable = Vec::<String>::new();

    if let Some(v) = config.absolute_displacement_limit {
        if v.is_finite() && v >= 0.0 {
            candidates.push(("absolute".to_string(), v));
        }
    } else {
        unavailable.push("absolute".to_string());
    }
    push_scaled(
        &mut candidates,
        &mut unavailable,
        "local_edge_length",
        scales.local_edge_length,
        config.maximum_displacement_fraction_local_edge,
    );
    push_scaled(
        &mut candidates,
        &mut unavailable,
        "local_cell_size",
        scales.local_cell_size,
        config.maximum_displacement_fraction_local_cell,
    );
    push_scaled(
        &mut candidates,
        &mut unavailable,
        "feature_size",
        scales.feature_size,
        config.maximum_displacement_fraction_feature_size,
    );
    push_scaled(
        &mut candidates,
        &mut unavailable,
        "sector_thickness",
        scales.sector_thickness,
        config.maximum_displacement_fraction_sector_thickness,
    );
    push_scaled(
        &mut candidates,
        &mut unavailable,
        "competing_sector_distance",
        scales.competing_sector_distance,
        config.maximum_displacement_fraction_competing_sector,
    );

    let (limiting_scale, maximum_displacement) = candidates
        .into_iter()
        .filter(|(_, v)| v.is_finite() && *v >= 0.0)
        .min_by(|a, b| a.1.total_cmp(&b.1).then(a.0.cmp(&b.0)))
        .unwrap_or_else(|| ("unbounded_no_available_scale".to_string(), f64::INFINITY));

    TrustRegionResult {
        maximum_displacement,
        limiting_scale,
        unavailable_scales: unavailable,
    }
}

fn push_scaled(
    candidates: &mut Vec<(String, f64)>,
    unavailable: &mut Vec<String>,
    name: &str,
    value: Option<f64>,
    factor: f64,
) {
    match value {
        Some(v) if v.is_finite() && v >= 0.0 && factor.is_finite() && factor >= 0.0 => {
            candidates.push((name.to_string(), v * factor));
        }
        _ => unavailable.push(name.to_string()),
    }
}

pub fn deterministic_line_search_fractions(config: &CollisionAwareProjectionConfig) -> Vec<f64> {
    let mut out = Vec::new();
    let mut f = 1.0_f64;
    let reduction = config.line_search_reduction_factor;
    let min = config.minimum_line_search_fraction;
    if !(0.0..1.0).contains(&reduction) || min <= 0.0 {
        return vec![1.0];
    }
    while f + f64::EPSILON >= min {
        out.push(f);
        f *= reduction;
    }
    if out.last().is_none_or(|last| (*last - min).abs() > 1.0e-12) {
        out.push(min);
    }
    out
}

pub fn sector_transition_allowed(
    expected: Option<&str>,
    proposed: Option<&str>,
    allowed_junction_sectors: &[String],
) -> bool {
    match (expected, proposed) {
        (None, _) | (_, None) => true,
        (Some(a), Some(b)) if a == b => true,
        (Some(_), Some(b)) => allowed_junction_sectors.iter().any(|s| s == b),
    }
}

pub fn feature_transition_allowed(expected: Option<&str>, proposed: Option<&str>) -> bool {
    match (expected, proposed) {
        (None, _) | (_, None) => true,
        (Some(a), Some(b)) => a == b,
    }
}

pub fn provenance_transition_allowed(expected: Option<&str>, proposed: Option<&str>) -> bool {
    match (expected, proposed) {
        (None, _) | (_, None) => true,
        (Some(a), Some(b)) => a == b || a == "branch_boundary" || b == "branch_boundary",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trust_region_uses_most_restrictive_available_scale() {
        let config = CollisionAwareProjectionConfig::default();
        let scales = ProjectionScaleAvailability {
            absolute_limit: Some(10.0),
            local_edge_length: Some(2.0),
            local_cell_size: Some(3.0),
            feature_size: Some(1.0),
            sector_thickness: Some(0.5),
            competing_sector_distance: Some(9.0),
        };
        let result = trust_region_limit(&config, &scales);
        assert_eq!(result.limiting_scale, "sector_thickness");
        assert!((result.maximum_displacement - 0.1).abs() < 1.0e-12);
    }

    #[test]
    fn trust_region_records_unavailable_scales_without_zeroing_limit() {
        let config = CollisionAwareProjectionConfig {
            absolute_displacement_limit: None,
            ..CollisionAwareProjectionConfig::default()
        };
        let scales = ProjectionScaleAvailability {
            local_cell_size: Some(2.0),
            ..ProjectionScaleAvailability::default()
        };
        let result = trust_region_limit(&config, &scales);
        assert_eq!(result.limiting_scale, "local_cell_size");
        assert!(
            result
                .unavailable_scales
                .contains(&"feature_size".to_string())
        );
    }

    #[test]
    fn line_search_is_deterministic() {
        let config = CollisionAwareProjectionConfig {
            line_search_reduction_factor: 0.5,
            minimum_line_search_fraction: 0.125,
            ..CollisionAwareProjectionConfig::default()
        };
        assert_eq!(
            deterministic_line_search_fractions(&config),
            vec![1.0, 0.5, 0.25, 0.125]
        );
    }

    #[test]
    fn sector_rules_reject_upper_to_lower_without_junction() {
        assert!(!sector_transition_allowed(
            Some("lofted_wing.main.surface.upper"),
            Some("lofted_wing.main.surface.lower"),
            &[]
        ));
        assert!(sector_transition_allowed(
            Some("lofted_wing.main.surface.upper"),
            Some("lofted_wing.main.surface.lower"),
            &["lofted_wing.main.surface.lower".to_string()]
        ));
    }

    #[test]
    fn feature_rules_preserve_ids() {
        assert!(feature_transition_allowed(Some("a"), Some("a")));
        assert!(!feature_transition_allowed(Some("a"), Some("b")));
        assert!(feature_transition_allowed(None, Some("b")));
    }

    #[test]
    fn provenance_rules_preserve_retained_subtractor_roles() {
        assert!(provenance_transition_allowed(
            Some("retained"),
            Some("retained")
        ));
        assert!(!provenance_transition_allowed(
            Some("retained"),
            Some("subtractor")
        ));
        assert!(provenance_transition_allowed(
            Some("branch_boundary"),
            Some("subtractor")
        ));
    }
}
