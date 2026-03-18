// Version control system for CAD projects — git-style commit/branch model.

pub mod operations;

use std::collections::HashMap;
use std::hash::{Hash, Hasher, DefaultHasher};
use serde::{Serialize, Deserialize};
use indexmap::IndexMap;
use crate::ui::spline_editor::SplineEditorState;
use crate::sdf::spine::LongitudinalSplines;
use crate::undo::AppState;

// ── CommitId ──────────────────────────────────────────────────────────────────

#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct CommitId(pub String);

impl CommitId {
    /// Generate a deterministic ID from timestamp + message + script + dims.
    pub fn generate(timestamp: &str, message: &str, script: &str, dims: &str) -> Self {
        let mut h = DefaultHasher::new();
        timestamp.hash(&mut h);
        message.hash(&mut h);
        script.hash(&mut h);
        dims.hash(&mut h);
        CommitId(format!("{:016x}", h.finish()))
    }

    pub fn short(&self) -> &str {
        &self.0[..self.0.len().min(8)]
    }
}

// ── ProjectState — the snapshot embedded in a commit ─────────────────────────

#[derive(Clone, Serialize, Deserialize)]
pub struct ProjectState {
    pub script_text: String,
    pub profiles: HashMap<String, SplineEditorState>,
    pub dimensions: IndexMap<String, f64>,
    pub longitudinal_splines: LongitudinalSplines,
}

impl ProjectState {
    pub fn from_app_state(app: &AppState) -> Self {
        Self {
            script_text: app.script_text.clone(),
            profiles: app.profiles.clone(),
            dimensions: app.dimensions.clone(),
            longitudinal_splines: app.splines.clone(),
        }
    }

    pub fn apply_to_app_state(&self, app: &mut AppState) {
        app.script_text = self.script_text.clone();
        app.profiles = self.profiles.clone();
        app.dimensions = self.dimensions.clone();
        app.splines = self.longitudinal_splines.clone();
    }

    /// Quick equality check that avoids comparing profiles (no PartialEq on SplineEditorState).
    #[allow(dead_code)] // Available for future snapshot comparison
    pub fn is_same_as(&self, app: &AppState) -> bool {
        self.script_text == app.script_text
            && self.dimensions == app.dimensions
            && self.longitudinal_splines == app.splines
    }
}

// ── Commit ────────────────────────────────────────────────────────────────────

#[derive(Clone, Serialize, Deserialize)]
pub struct Commit {
    pub id: CommitId,
    pub parent_ids: Vec<CommitId>,
    pub author: String,
    pub timestamp: chrono::DateTime<chrono::Utc>,
    pub message: String,
    pub state: ProjectState,
    pub thumbnail: Option<Vec<u8>>,
}

// ── Branch ────────────────────────────────────────────────────────────────────

#[derive(Clone, Serialize, Deserialize)]
pub struct Branch {
    pub name: String,
    pub head_commit_id: CommitId,
    pub created_from: CommitId,
    pub created_at: chrono::DateTime<chrono::Utc>,
    pub description: String,
}

// ── VersionControlState ───────────────────────────────────────────────────────

#[derive(Clone, Serialize, Deserialize)]
pub struct VersionControlState {
    pub commits: HashMap<CommitId, Commit>,
    pub branches: HashMap<String, Branch>,
    pub current_branch: String,
    pub head_commit_id: Option<CommitId>,
    pub detached_head: bool,
    pub working_changes: bool,
}

impl VersionControlState {
    /// Create fresh state with a root commit and "main" branch.
    pub fn new_with_root(app_state: &AppState) -> Self {
        let timestamp = chrono::Utc::now();
        let ts_str = timestamp.to_rfc3339();
        let dims_str = format!("{:?}", app_state.dimensions);
        let root_id = CommitId::generate(&ts_str, "Initial commit", &app_state.script_text, &dims_str);

        let root_commit = Commit {
            id: root_id.clone(),
            parent_ids: Vec::new(),
            author: String::from("User"),
            timestamp,
            message: String::from("Initial commit"),
            state: ProjectState::from_app_state(app_state),
            thumbnail: None,
        };

        let main_branch = Branch {
            name: String::from("main"),
            head_commit_id: root_id.clone(),
            created_from: root_id.clone(),
            created_at: timestamp,
            description: String::from("Main development branch"),
        };

        let mut commits = HashMap::new();
        commits.insert(root_id.clone(), root_commit);

        let mut branches = HashMap::new();
        branches.insert(String::from("main"), main_branch);

        Self {
            commits,
            branches,
            current_branch: String::from("main"),
            head_commit_id: Some(root_id),
            detached_head: false,
            working_changes: false,
        }
    }
}
