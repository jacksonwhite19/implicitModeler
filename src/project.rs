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
