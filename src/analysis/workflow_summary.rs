use crate::analysis::print_analysis::{PrintAnalysisResult, PrintAnalysisSettings};
use crate::analysis::thickness::ThicknessResult;
use crate::aero::{DragPolarResult, StaticMarginResult, TrimResult};
use crate::geometry_analysis::CgSensitivityResult;
use crate::scripting::MassPoint;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SummaryStatus {
    Pass,
    Warning,
    Fail,
}

impl SummaryStatus {
    pub fn label(self) -> &'static str {
        match self {
            SummaryStatus::Pass => "Pass",
            SummaryStatus::Warning => "Warning",
            SummaryStatus::Fail => "Fail",
        }
    }
}

#[derive(Clone, Debug)]
pub struct ManufacturingSummary {
    pub status: SummaryStatus,
    pub min_wall_mm: Option<f32>,
    pub overhang_area_mm2: Option<f32>,
    pub critical_overhang_area_mm2: Option<f32>,
    pub issue_errors: usize,
    pub issue_warnings: usize,
    pub notes: Vec<String>,
}

#[derive(Clone, Debug)]
pub struct FlightSummary {
    pub status: SummaryStatus,
    pub total_mass_g: f32,
    pub cg_x_mm: Option<f32>,
    pub static_margin_mac: Option<f32>,
    pub best_glide_ratio: Option<f32>,
    pub best_glide_speed_ms: Option<f32>,
    pub trim_aoa_deg: Option<f32>,
    pub notes: Vec<String>,
}

pub fn summarize_manufacturing(
    thickness: Option<&ThicknessResult>,
    print: Option<&PrintAnalysisResult>,
    settings: &PrintAnalysisSettings,
) -> ManufacturingSummary {
    let mut status = SummaryStatus::Pass;
    let mut notes = Vec::new();

    let min_wall_mm = thickness.map(|t| t.min_thickness);
    if let Some(min_wall) = min_wall_mm {
        if min_wall < settings.min_wall_thickness {
            status = SummaryStatus::Fail;
            notes.push(format!(
                "Minimum wall {:.2} mm is below the {:.2} mm target.",
                min_wall, settings.min_wall_thickness
            ));
        }
    } else {
        status = SummaryStatus::Warning;
        notes.push("Wall-thickness analysis has not been run yet.".into());
    }

    let (overhang_area_mm2, critical_overhang_area_mm2, issue_errors, issue_warnings) =
        if let Some(result) = print {
            let errors = result.features.issues.iter()
                .filter(|i| matches!(i.severity, crate::analysis::print_analysis::IssueSeverity::Error))
                .count();
            let warnings = result.features.issues.iter()
                .filter(|i| matches!(i.severity, crate::analysis::print_analysis::IssueSeverity::Warning))
                .count();

            if result.overhang.critical_overhang_area_mm2 > 0.0 || errors > 0 {
                status = SummaryStatus::Fail;
            } else if result.overhang.overhang_area_mm2 > 0.0 || warnings > 0 {
                if status != SummaryStatus::Fail {
                    status = SummaryStatus::Warning;
                }
            }

            if result.overhang.overhang_area_mm2 > 0.0 {
                notes.push(format!(
                    "Overhang area {:.0} mm² with {:.0} mm³ estimated support.",
                    result.overhang.overhang_area_mm2,
                    result.overhang.support_volume_estimate_mm3
                ));
            }
            if errors > 0 || warnings > 0 {
                notes.push(format!("Detected {} errors and {} warnings.", errors, warnings));
            }

            (
                Some(result.overhang.overhang_area_mm2),
                Some(result.overhang.critical_overhang_area_mm2),
                errors,
                warnings,
            )
        } else {
            if status == SummaryStatus::Pass {
                status = SummaryStatus::Warning;
            }
            notes.push("Print analysis has not been run yet.".into());
            (None, None, 0, 0)
        };

    ManufacturingSummary {
        status,
        min_wall_mm,
        overhang_area_mm2,
        critical_overhang_area_mm2,
        issue_errors,
        issue_warnings,
        notes,
    }
}

pub fn summarize_flight(
    mass_points: &[MassPoint],
    cg_x_mm: Option<f32>,
    static_margin: Option<&StaticMarginResult>,
    trim: Option<&TrimResult>,
    drag_polar: Option<&DragPolarResult>,
    cg_sensitivity: Option<&CgSensitivityResult>,
) -> FlightSummary {
    let total_mass_g: f32 = mass_points.iter().map(|m| m.mass_g).sum();
    let mut status = if mass_points.is_empty() {
        SummaryStatus::Warning
    } else {
        SummaryStatus::Pass
    };
    let mut notes = Vec::new();

    if mass_points.is_empty() {
        notes.push("No mass points are present, so CG-based checks are incomplete.".into());
    }

    let static_margin_mac = static_margin.map(|sm| sm.static_margin_mac);
    if let Some(sm) = static_margin {
        use crate::aero::StabilityCategory::*;
        match sm.stability_category {
            Unstable | Neutral => {
                status = SummaryStatus::Fail;
                notes.push(format!("Static margin is {}.", sm.stability_category));
            }
            Marginal => {
                if status != SummaryStatus::Fail {
                    status = SummaryStatus::Warning;
                }
                notes.push("Static margin is marginal.".into());
            }
            Stable | VeryStable => {}
        }
    } else {
        if status == SummaryStatus::Pass {
            status = SummaryStatus::Warning;
        }
        notes.push("Flight stability analysis has not been run yet.".into());
    }

    if let Some(trim) = trim {
        if !trim.is_trimmed && status != SummaryStatus::Fail {
            status = SummaryStatus::Warning;
            notes.push("Trim solution is approximate.".into());
        }
    }

    if let Some(cg) = cg_sensitivity {
        if !cg.recommendations.is_empty() {
            notes.push(cg.recommendations[0].clone());
        }
    }

    FlightSummary {
        status,
        total_mass_g,
        cg_x_mm,
        static_margin_mac,
        best_glide_ratio: drag_polar.map(|d| d.ld_max),
        best_glide_speed_ms: drag_polar.map(|d| d.best_glide_airspeed_ms),
        trim_aoa_deg: trim.map(|t| t.trim_aoa_deg),
        notes,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analysis::print_analysis::{FeatureDetectionResult, OrientationResult, OverhangResult, PrintAnalysisResult, SurfacePoint};
    use crate::aero::{StaticMarginResult, StabilityCategory};
    use glam::Vec3;

    #[test]
    fn manufacturing_summary_fails_on_thin_walls() {
        let thickness = ThicknessResult {
            min_thickness: 0.5,
            min_location: Vec3::ZERO,
            analysis_grid: std::sync::Arc::new(vec![0.5]),
            bounds_min: Vec3::ZERO,
            bounds_max: Vec3::ONE,
            resolution: 1,
        };
        let summary = summarize_manufacturing(
            Some(&thickness),
            None,
            &PrintAnalysisSettings::default(),
        );
        assert_eq!(summary.status, SummaryStatus::Fail);
    }

    #[test]
    fn flight_summary_warns_without_analysis() {
        let summary = summarize_flight(&[], None, None, None, None, None);
        assert_eq!(summary.status, SummaryStatus::Warning);
    }

    #[test]
    fn flight_summary_fails_when_unstable() {
        let sm = StaticMarginResult {
            cg_x_mm: 100.0,
            neutral_point_x_mm: 90.0,
            static_margin_mm: -10.0,
            static_margin_mac: -0.1,
            stability_category: StabilityCategory::Unstable,
            is_stable: false,
            cg_forward_limit_mm: 70.0,
            cg_aft_limit_mm: 85.0,
            cg_range_mm: 15.0,
            pitch_stiffness: -0.1,
        };
        let summary = summarize_flight(&[], Some(100.0), Some(&sm), None, None, None);
        assert_eq!(summary.status, SummaryStatus::Fail);
    }

    #[test]
    fn manufacturing_summary_warns_for_print_issues() {
        let print = PrintAnalysisResult {
            surface_points: Vec::<SurfacePoint>::new(),
            overhang: OverhangResult {
                overhang_grid: vec![],
                overhang_area_mm2: 10.0,
                critical_overhang_area_mm2: 0.0,
                support_volume_estimate_mm3: 100.0,
                resolution: 1,
                bounds_min: Vec3::ZERO,
                bounds_max: Vec3::ONE,
            },
            orientation: OrientationResult { candidates: vec![], recommended: 0 },
            features: FeatureDetectionResult { issues: vec![], issue_grid: vec![] },
        };
        let thickness = ThicknessResult {
            min_thickness: 2.0,
            min_location: Vec3::ZERO,
            analysis_grid: std::sync::Arc::new(vec![2.0]),
            bounds_min: Vec3::ZERO,
            bounds_max: Vec3::ONE,
            resolution: 1,
        };
        let summary = summarize_manufacturing(Some(&thickness), Some(&print), &PrintAnalysisSettings::default());
        assert_eq!(summary.status, SummaryStatus::Warning);
    }
}
