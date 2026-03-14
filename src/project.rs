// Project file management

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::io;
use std::path::Path;
use crate::node_graph::NodeGraph;
use crate::notebook::Notebook;
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
    #[serde(skip_serializing_if = "Option::is_none")]
    pub node_graph: Option<NodeGraph>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub notebook: Option<Notebook>,
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
        node_graph: Option<NodeGraph>,
        notebook: Option<Notebook>,
        profiles: Option<HashMap<String, SplineEditorState>>,
        splines: Option<LongitudinalSplines>,
        section_view: Option<SectionView>,
        fea_config: Option<crate::fea::FEAConfig>,
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
            node_graph,
            notebook,
            profiles,
            splines,
            section_view,
            fea_config,
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
