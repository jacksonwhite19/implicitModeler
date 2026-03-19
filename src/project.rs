// Project file management

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::io;
use std::path::Path;
use indexmap::IndexMap;
use crate::ui::spline_editor::SplineEditorState;
use crate::sdf::spine::LongitudinalSplines;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ManufacturingPreset {
    Foamboard,
    LwPlaShell,
    CarbonTubeSpar,
    BalsaHybrid,
    MoldedShell,
}

impl ManufacturingPreset {
    pub fn label(&self) -> &'static str {
        match self {
            Self::Foamboard => "Foamboard",
            Self::LwPlaShell => "LW-PLA Shell",
            Self::CarbonTubeSpar => "Carbon Tube Spar",
            Self::BalsaHybrid => "Balsa Hybrid",
            Self::MoldedShell => "Molded Shell",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum AssemblyConstraintFormula {
    CopyOffset {
        source: String,
        scale: f64,
        offset: f64,
    },
    AverageOffset {
        sources: Vec<String>,
        offset: f64,
    },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AssemblyConstraint {
    pub label: String,
    pub driven: String,
    pub formula: AssemblyConstraintFormula,
    #[serde(default = "default_true")]
    pub enabled: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DesignVariant {
    pub name: String,
    #[serde(default)]
    pub description: String,
    #[serde(skip_serializing_if = "IndexMap::is_empty", default)]
    pub dimensions: IndexMap<String, f64>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct WorkflowConfig {
    pub template_id: String,
    pub vehicle_type: String,
    pub manufacturing_preset: ManufacturingPreset,
    #[serde(skip_serializing_if = "IndexMap::is_empty", default)]
    pub parameter_groups: IndexMap<String, Vec<String>>,
    #[serde(skip_serializing_if = "Vec::is_empty", default)]
    pub assembly_constraints: Vec<AssemblyConstraint>,
    #[serde(skip_serializing_if = "Vec::is_empty", default)]
    pub variants: Vec<DesignVariant>,
}

fn default_true() -> bool { true }

pub fn apply_assembly_constraints(
    dimensions: &mut IndexMap<String, f64>,
    constraints: &[AssemblyConstraint],
) -> usize {
    let mut applied = 0usize;

    for constraint in constraints.iter().filter(|c| c.enabled) {
        let value = match &constraint.formula {
            AssemblyConstraintFormula::CopyOffset { source, scale, offset } => {
                dimensions.get(source).map(|v| v * scale + offset)
            }
            AssemblyConstraintFormula::AverageOffset { sources, offset } => {
                let values: Option<Vec<f64>> = sources.iter()
                    .map(|name| dimensions.get(name).copied())
                    .collect();
                values.and_then(|vals| {
                    if vals.is_empty() {
                        None
                    } else {
                        Some(vals.iter().sum::<f64>() / vals.len() as f64 + offset)
                    }
                })
            }
        };

        if let Some(value) = value {
            dimensions.insert(constraint.driven.clone(), value);
            applied += 1;
        }
    }

    applied
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Axis { X, Y, Z }

impl Axis {
    pub fn to_index(&self) -> u32 {
        match self { Axis::X => 0, Axis::Y => 1, Axis::Z => 2 }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SectionPlane {
    pub axis:     Axis,
    pub position: f32,
    pub enabled:  bool,
    /// When false, keeps coord >= position. When true, keeps coord <= position (flipped).
    #[serde(default)]
    pub flip:     bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SectionView {
    pub plane_a: Option<SectionPlane>,
    pub plane_b: Option<SectionPlane>,
}

impl Default for SectionView {
    fn default() -> Self {
        Self {
            plane_a: Some(SectionPlane { axis: Axis::X, position: 0.0, enabled: false, flip: false }),
            plane_b: None,
        }
    }
}

#[derive(Serialize, Deserialize, Clone)]
pub struct Project {
    pub version: String,
    pub script: String,
    pub resolution: u32,
    pub smooth_normals: bool,
    pub show_wireframe: bool,
    pub camera_position: [f32; 3],
    pub camera_target: [f32; 3],
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timestamp: Option<String>,
    /// Named spline cross-section profiles edited via the spline editor.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub profiles: Option<HashMap<String, SplineEditorState>>,
    /// Longitudinal spine constraints (keel/deck/chine curves).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub splines: Option<LongitudinalSplines>,
    /// Section view clipping planes.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub section_view: Option<SectionView>,
    /// FEA solver configuration (material, resolution).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fea_config: Option<crate::fea::FEAConfig>,
    /// Named dimensions injected as Rhai constants.
    #[serde(skip_serializing_if = "IndexMap::is_empty", default)]
    pub dimensions: IndexMap<String, f64>,
    /// Print analysis settings (build direction, printer preset, thresholds).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub print_analysis_settings: Option<crate::analysis::print_analysis::PrintAnalysisSettings>,
    /// Tolerance compensation settings for FDM export.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tolerance_settings: Option<crate::sdf::print::ToleranceSettings>,
    /// Workflow metadata for template-driven aircraft projects.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub workflow_config: Option<WorkflowConfig>,
    /// Version control state (optional for backward compatibility).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version_control: Option<crate::version_control::VersionControlState>,
}

impl Project {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        script: String,
        resolution: u32,
        smooth_normals: bool,
        show_wireframe: bool,
        camera_position: [f32; 3],
        camera_target: [f32; 3],
        profiles: Option<HashMap<String, SplineEditorState>>,
        splines: Option<LongitudinalSplines>,
        section_view: Option<SectionView>,
        fea_config: Option<crate::fea::FEAConfig>,
        dimensions: IndexMap<String, f64>,
        print_analysis_settings: Option<crate::analysis::print_analysis::PrintAnalysisSettings>,
        tolerance_settings: Option<crate::sdf::print::ToleranceSettings>,
        workflow_config: Option<WorkflowConfig>,
    ) -> Self {
        Self {
            version: env!("CARGO_PKG_VERSION").to_string(),
            script,
            resolution,
            smooth_normals,
            show_wireframe,
            camera_position,
            camera_target,
            timestamp: Some(chrono::Local::now().to_rfc3339()),
            profiles,
            splines,
            section_view,
            fea_config,
            dimensions,
            print_analysis_settings,
            tolerance_settings,
            workflow_config,
            version_control: None,
        }
    }

    pub fn save(&self, path: impl AsRef<Path>) -> io::Result<()> {
        let json = serde_json::to_string_pretty(self)?;
        fs::write(path, json)?;
        Ok(())
    }

    pub fn load(path: impl AsRef<Path>) -> io::Result<Self> {
        let json = fs::read_to_string(path)?;
        let project: Project = serde_json::from_str(&json)?;
        Ok(project)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn apply_copy_offset_constraint_updates_driven_dimension() {
        let mut dims = IndexMap::new();
        dims.insert("wing_mount_x".to_string(), 120.0);
        let constraints = vec![AssemblyConstraint {
            label: "Servo follows wing".to_string(),
            driven: "servo_x".to_string(),
            formula: AssemblyConstraintFormula::CopyOffset {
                source: "wing_mount_x".to_string(),
                scale: 1.0,
                offset: 50.0,
            },
            enabled: true,
        }];

        let count = apply_assembly_constraints(&mut dims, &constraints);
        assert_eq!(count, 1);
        assert_eq!(dims.get("servo_x"), Some(&170.0));
    }

    #[test]
    fn apply_average_constraint_uses_multiple_sources() {
        let mut dims = IndexMap::new();
        dims.insert("wing_mount_x".to_string(), 120.0);
        dims.insert("tail_mount_x".to_string(), 480.0);
        let constraints = vec![AssemblyConstraint {
            label: "Split near center".to_string(),
            driven: "split_station_x".to_string(),
            formula: AssemblyConstraintFormula::AverageOffset {
                sources: vec!["wing_mount_x".to_string(), "tail_mount_x".to_string()],
                offset: -10.0,
            },
            enabled: true,
        }];

        apply_assembly_constraints(&mut dims, &constraints);
        assert_eq!(dims.get("split_station_x"), Some(&290.0));
    }
}
