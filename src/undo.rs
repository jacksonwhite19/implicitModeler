// App-wide command-pattern undo/redo system.
//
// AppState holds all user-editable fields.  Commands implement apply() + undo()
// against AppState.  UndoHistory owns the past/future stacks.

use std::collections::HashMap;
use std::time::{Duration, Instant};
use indexmap::IndexMap;

use crate::ui::spline_editor::SplineEditorState;
use crate::sdf::spine::LongitudinalSplines;
use crate::fea::setup::FEASetup;

// ── User-editable application state ──────────────────────────────────────────

pub struct AppState {
    pub script_text:    String,
    pub profiles:       HashMap<String, SplineEditorState>,
    pub splines:        LongitudinalSplines,
    pub fea_setup:      FEASetup,
    pub active_profile: Option<String>,
    /// Named dimensions — injected as global constants into Rhai before eval.
    pub dimensions:     IndexMap<String, f64>,
}

impl AppState {
    pub fn new(script_text: String) -> Self {
        Self {
            script_text,
            profiles:       HashMap::new(),
            splines:        LongitudinalSplines::default(),
            fea_setup:      FEASetup::default(),
            active_profile: None,
            dimensions:     IndexMap::new(),
        }
    }
}

// ── Command trait ─────────────────────────────────────────────────────────────

pub trait Command: Send + Sync {
    fn apply(&self, state: &mut AppState);
    fn undo(&self, state: &mut AppState);
    fn description(&self) -> &str;
    /// Allow downcast for coalescing.
    fn as_any(&self) -> &dyn std::any::Any;
    /// Try to coalesce `newer` into self (absorbing it).  Return true if coalesced.
    fn try_coalesce(&mut self, _newer: &dyn Command) -> bool { false }
}

// ── History entry ─────────────────────────────────────────────────────────────

struct UndoEntry {
    cmd:  Box<dyn Command>,
    time: Instant,
}

// ── UndoHistory ───────────────────────────────────────────────────────────────

pub struct UndoHistory {
    past:             Vec<UndoEntry>,
    future:           Vec<Box<dyn Command>>,
    pub max_history:  usize,
    /// Description of the last undo or redo action for status bar display.
    pub last_action:  Option<(String, Instant)>,  // (text, when)
}

impl Default for UndoHistory {
    fn default() -> Self {
        Self {
            past:        Vec::new(),
            future:      Vec::new(),
            max_history: 200,
            last_action: None,
        }
    }
}

impl UndoHistory {
    /// Execute a command: apply it, push to past, clear future.
    /// Consecutive same-type commands within 800 ms are coalesced.
    pub fn execute(&mut self, cmd: Box<dyn Command>, state: &mut AppState) {
        cmd.apply(state);
        let now = Instant::now();

        // Try coalescing with the top of the stack.
        if let Some(top) = self.past.last_mut() {
            if now.duration_since(top.time) < Duration::from_millis(800) {
                if top.cmd.try_coalesce(cmd.as_ref()) {
                    top.time = now;
                    self.future.clear();
                    return;
                }
            }
        }

        self.future.clear();
        self.past.push(UndoEntry { cmd, time: now });

        if self.past.len() > self.max_history {
            self.past.remove(0);
        }
    }

    pub fn undo(&mut self, state: &mut AppState) {
        if let Some(entry) = self.past.pop() {
            let desc = entry.cmd.description().to_owned();
            entry.cmd.undo(state);
            self.future.push(entry.cmd);
            self.last_action = Some((format!("Undone: {}", desc), Instant::now()));
        }
    }

    pub fn redo(&mut self, state: &mut AppState) {
        if let Some(cmd) = self.future.pop() {
            let desc = cmd.description().to_owned();
            cmd.apply(state);
            let now = Instant::now();
            self.past.push(UndoEntry { cmd, time: now });
            self.last_action = Some((format!("Redone: {}", desc), Instant::now()));
        }
    }

    pub fn clear(&mut self) {
        self.past.clear();
        self.future.clear();
    }

    /// Push a command that has already been applied to state (e.g. by egui TextEdit).
    /// Handles coalescing the same as execute() but skips the apply() call.
    pub fn push_executed(&mut self, cmd: Box<dyn Command>) {
        let now = Instant::now();

        if let Some(top) = self.past.last_mut() {
            if now.duration_since(top.time) < Duration::from_millis(800) {
                if top.cmd.try_coalesce(cmd.as_ref()) {
                    top.time = now;
                    self.future.clear();
                    return;
                }
            }
        }

        self.future.clear();
        self.past.push(UndoEntry { cmd, time: now });
        if self.past.len() > self.max_history {
            self.past.remove(0);
        }
    }

    pub fn can_undo(&self) -> bool { !self.past.is_empty() }
    pub fn can_redo(&self) -> bool { !self.future.is_empty() }

    pub fn undo_description(&self) -> Option<&str> {
        self.past.last().map(|e| e.cmd.description())
    }
    pub fn redo_description(&self) -> Option<&str> {
        self.future.last().map(|c| c.description())
    }

    /// Iterator over past command descriptions, most-recent first (up to n).
    pub fn past_descriptions(&self, n: usize) -> impl Iterator<Item = &str> {
        self.past.iter().rev().take(n).map(|e| e.cmd.description())
    }
}

// ── ScriptTextCommand ─────────────────────────────────────────────────────────

pub struct ScriptTextCommand {
    pub before: String,
    pub after:  String,
    pub desc:   String,
}

impl ScriptTextCommand {
    pub fn new(before: String, after: String) -> Self {
        Self { before, after, desc: "Edit script".to_owned() }
    }
}

impl Command for ScriptTextCommand {
    fn apply(&self, state: &mut AppState) { state.script_text = self.after.clone(); }
    fn undo(&self,  state: &mut AppState) { state.script_text = self.before.clone(); }
    fn description(&self) -> &str { &self.desc }
    fn as_any(&self) -> &dyn std::any::Any { self }
    fn try_coalesce(&mut self, newer: &dyn Command) -> bool {
        if let Some(n) = newer.as_any().downcast_ref::<Self>() {
            self.after = n.after.clone();
            true
        } else { false }
    }
}

// ── SplineShapeResetCommand (covers all spline edits via full-state snapshot) ──

pub struct SplineShapeResetCommand {
    pub profile_name: String,
    pub before:       SplineEditorState,
    pub after:        SplineEditorState,
    pub desc:         String,
}

impl Command for SplineShapeResetCommand {
    fn apply(&self, state: &mut AppState) {
        state.profiles.insert(self.profile_name.clone(), self.after.clone());
    }
    fn undo(&self, state: &mut AppState) {
        state.profiles.insert(self.profile_name.clone(), self.before.clone());
    }
    fn description(&self) -> &str { &self.desc }
    fn as_any(&self) -> &dyn std::any::Any { self }
}

// ── LongitudinalSpineEditCommand ──────────────────────────────────────────────

pub struct LongitudinalSpineEditCommand {
    pub before: LongitudinalSplines,
    pub after:  LongitudinalSplines,
    pub desc:   String,
}

impl LongitudinalSpineEditCommand {
    pub fn new(before: LongitudinalSplines, after: LongitudinalSplines) -> Self {
        Self { before, after, desc: "Edit spine".to_owned() }
    }
}

impl Command for LongitudinalSpineEditCommand {
    fn apply(&self, state: &mut AppState) { state.splines = self.after.clone(); }
    fn undo(&self,  state: &mut AppState) { state.splines = self.before.clone(); }
    fn description(&self) -> &str { &self.desc }
    fn as_any(&self) -> &dyn std::any::Any { self }
    fn try_coalesce(&mut self, newer: &dyn Command) -> bool {
        if let Some(n) = newer.as_any().downcast_ref::<Self>() {
            self.after = n.after.clone();
            true
        } else { false }
    }
}

// ── RenameCommand ─────────────────────────────────────────────────────────────

pub struct RenameCommand {
    pub old_name: String,
    pub new_name: String,
    /// Full before-script (since rename does a find-replace across whole text).
    pub script_before: String,
    pub script_after:  String,
}

impl Command for RenameCommand {
    fn apply(&self, state: &mut AppState) {
        state.script_text = self.script_after.clone();
        if let Some(profile) = state.profiles.remove(&self.old_name) {
            state.profiles.insert(self.new_name.clone(), profile);
        }
        if state.active_profile.as_deref() == Some(&self.old_name) {
            state.active_profile = Some(self.new_name.clone());
        }
    }
    fn undo(&self, state: &mut AppState) {
        state.script_text = self.script_before.clone();
        if let Some(profile) = state.profiles.remove(&self.new_name) {
            state.profiles.insert(self.old_name.clone(), profile);
        }
        if state.active_profile.as_deref() == Some(&self.new_name) {
            state.active_profile = Some(self.old_name.clone());
        }
    }
    fn description(&self) -> &str { "Rename" }
    fn as_any(&self) -> &dyn std::any::Any { self }
}

// ── DimensionEditCommand ──────────────────────────────────────────────────────

/// Covers both value edits and name renames for a single dimension entry.
/// Coalesces value changes to the same name within 800ms (slider drag).
pub struct DimensionEditCommand {
    pub before_name:  String,
    pub before_value: f64,
    pub after_name:   String,
    pub after_value:  f64,
    /// Insertion index (preserved across undo so order is stable).
    pub index:        usize,
}

impl DimensionEditCommand {
    pub fn new_value(name: String, before: f64, after: f64, index: usize) -> Self {
        Self { before_name: name.clone(), before_value: before,
               after_name: name, after_value: after, index }
    }
    pub fn new_rename(old: String, val: f64, new: String, index: usize) -> Self {
        Self { before_name: old, before_value: val, after_name: new, after_value: val, index }
    }
}

impl Command for DimensionEditCommand {
    fn apply(&self, state: &mut AppState) {
        state.dimensions.swap_remove(&self.before_name);
        // Re-insert preserving approximate position.
        let target = self.index.min(state.dimensions.len());
        state.dimensions.shift_insert(target, self.after_name.clone(), self.after_value);
    }
    fn undo(&self, state: &mut AppState) {
        state.dimensions.swap_remove(&self.after_name);
        let target = self.index.min(state.dimensions.len());
        state.dimensions.shift_insert(target, self.before_name.clone(), self.before_value);
    }
    fn description(&self) -> &str { "Edit dimension" }
    fn as_any(&self) -> &dyn std::any::Any { self }
    fn try_coalesce(&mut self, newer: &dyn Command) -> bool {
        if let Some(n) = newer.as_any().downcast_ref::<Self>() {
            // Only coalesce pure-value changes to the same dimension.
            if self.after_name == n.before_name && self.after_name == n.after_name {
                self.after_value = n.after_value;
                return true;
            }
        }
        false
    }
}

// ── DimensionDeleteCommand ────────────────────────────────────────────────────

pub struct DimensionDeleteCommand {
    pub name:  String,
    pub value: f64,
    pub index: usize,
}

impl Command for DimensionDeleteCommand {
    fn apply(&self, state: &mut AppState) {
        state.dimensions.swap_remove(&self.name);
    }
    fn undo(&self, state: &mut AppState) {
        let target = self.index.min(state.dimensions.len());
        state.dimensions.shift_insert(target, self.name.clone(), self.value);
    }
    fn description(&self) -> &str { "Delete dimension" }
    fn as_any(&self) -> &dyn std::any::Any { self }
}
