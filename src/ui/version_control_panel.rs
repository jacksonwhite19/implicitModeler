// Version control UI panel — floating Window with Branches / History / Changes tabs.

use eframe::egui::{self, Color32, RichText, Ui};
use crate::version_control::{VersionControlState, CommitId, ProjectState};
use crate::version_control::operations::{
    commit, create_branch, checkout_branch, checkout_commit, discard_changes,
    delete_branch, merge, get_commit_graph,
    MergeResult, MergeConflict,
};
use crate::undo::AppState;

// ── Tab enum ──────────────────────────────────────────────────────────────────

#[derive(PartialEq, Clone, Copy, Default)]
pub enum VCTab {
    Branches,
    #[default]
    History,
    Changes,
}

// ── Conflict resolution ───────────────────────────────────────────────────────

#[derive(Clone)]
#[allow(dead_code)] // Part of merge conflict resolution API
pub enum ConflictResolution {
    UseCurrent,
    UseSource,
    Manual(Vec<String>),
}

// ── VCPanelState ──────────────────────────────────────────────────────────────

#[allow(dead_code)] // VC panel state — some fields used only in planned merge/checkout UI
pub struct VCPanelState {
    pub active_tab: VCTab,
    pub selected_commit: Option<CommitId>,
    pub selected_branch: Option<String>,
    pub merge_source: Option<String>,
    pub conflict: Option<MergeConflict>,
    pub conflict_resolutions: Vec<ConflictResolution>,
    pub dim_resolutions: Vec<bool>,         // true = use_current per dimension conflict
    pub history_limit: usize,
    pub pending_checkout_branch: Option<String>,
    pub pending_checkout_commit: Option<CommitId>,
    pub pending_merge_source: Option<String>,
    pub commit_dialog_open: bool,
    pub commit_msg_suggestion: String,
}

impl Default for VCPanelState {
    fn default() -> Self {
        Self {
            active_tab: VCTab::History,
            selected_commit: None,
            selected_branch: None,
            merge_source: None,
            conflict: None,
            conflict_resolutions: Vec::new(),
            dim_resolutions: Vec::new(),
            history_limit: 100,
            pending_checkout_branch: None,
            pending_checkout_commit: None,
            pending_merge_source: None,
            commit_dialog_open: false,
            commit_msg_suggestion: String::new(),
        }
    }
}

// ── Suggest commit message ────────────────────────────────────────────────────

pub fn suggest_commit_message(current: &AppState, head_state: &ProjectState) -> String {
    let script_changed = current.script_text != head_state.script_text;
    let dims_changed: Vec<&str> = current.dimensions.iter()
        .filter(|(k, v)| head_state.dimensions.get(*k) != Some(v))
        .map(|(k, _)| k.as_str())
        .collect();
    let profiles_changed = current.profiles.len() != head_state.profiles.len()
        || current.profiles.keys().any(|k| {
            let a = serde_json::to_string(head_state.profiles.get(k).unwrap_or(&Default::default())).unwrap_or_default();
            let b = serde_json::to_string(current.profiles.get(k).unwrap_or(&Default::default())).unwrap_or_default();
            a != b
        });

    match (script_changed, dims_changed.len(), profiles_changed) {
        (false, 0, false) => "No changes".to_string(),
        (false, n, false) if n > 0 => format!("Update {}", dims_changed.join(", ")),
        (true, 0, false) => "Update script".to_string(),
        (true, n, false) if n > 0 => format!("Update script and {}", dims_changed.join(", ")),
        (_, _, true) => "Edit profiles".to_string(),
        _ => "Update".to_string(),
    }
}

// ── Diff display helper ───────────────────────────────────────────────────────

struct DiffLine {
    kind: DiffKind,
    content: String,
}

enum DiffKind {
    Same,
    Added,
    Removed,
}

fn compute_diff_display(before: &str, after: &str, context: usize) -> Vec<DiffLine> {
    let before_lines: Vec<&str> = before.lines().collect();
    let after_lines: Vec<&str> = after.lines().collect();

    let n = before_lines.len();
    let m = after_lines.len();

    // Build LCS table.
    let mut dp = vec![vec![0usize; m + 1]; n + 1];
    for i in (0..n).rev() {
        for j in (0..m).rev() {
            dp[i][j] = if before_lines[i] == after_lines[j] {
                dp[i + 1][j + 1] + 1
            } else {
                dp[i + 1][j].max(dp[i][j + 1])
            };
        }
    }

    // Walk LCS to get diff.
    let mut raw: Vec<DiffLine> = Vec::new();
    let mut i = 0;
    let mut j = 0;
    while i < n || j < m {
        if i < n && j < m && before_lines[i] == after_lines[j] {
            raw.push(DiffLine { kind: DiffKind::Same, content: before_lines[i].to_string() });
            i += 1; j += 1;
        } else if j < m && (i >= n || dp[i][j + 1] >= dp[i + 1][j]) {
            raw.push(DiffLine { kind: DiffKind::Added, content: after_lines[j].to_string() });
            j += 1;
        } else if i < n {
            raw.push(DiffLine { kind: DiffKind::Removed, content: before_lines[i].to_string() });
            i += 1;
        }
    }

    // Filter to only show context lines around changes.
    let mut show = vec![false; raw.len()];
    for (idx, dl) in raw.iter().enumerate() {
        if !matches!(dl.kind, DiffKind::Same) {
            let start = idx.saturating_sub(context);
            let end = (idx + context + 1).min(raw.len());
            for k in start..end {
                show[k] = true;
            }
        }
    }

    raw.into_iter().zip(show.into_iter())
        .filter(|(_, s)| *s)
        .map(|(d, _)| d)
        .collect()
}

// ── Main panel entry point ────────────────────────────────────────────────────

pub fn show_vc_panel(
    ctx: &egui::Context,
    open: &mut bool,
    vc: &mut VersionControlState,
    state: &mut AppState,
    panel_state: &mut VCPanelState,
    commit_message: &mut String,
    new_branch_name: &mut String,
    new_branch_desc: &mut String,
    new_branch_dialog: &mut bool,
    discard_confirm: &mut bool,
    needs_eval: &mut bool,
) {
    // ── Conflict resolution modal — shown on top of everything ───────────────
    if panel_state.conflict.is_some() {
        show_conflict_modal(ctx, vc, state, panel_state, needs_eval);
        // Don't show main panel while conflict modal is open
        return;
    }

    egui::Window::new("Version Control")
        .open(open)
        .default_width(480.0)
        .min_height(300.0)
        .resizable(true)
        .show(ctx, |ui| {
            // ── Tab bar ───────────────────────────────────────────────────────
            ui.horizontal(|ui| {
                ui.selectable_value(&mut panel_state.active_tab, VCTab::Branches, "Branches");
                ui.selectable_value(&mut panel_state.active_tab, VCTab::History,  "History");
                ui.selectable_value(&mut panel_state.active_tab, VCTab::Changes,  "Changes");
            });
            ui.separator();

            match panel_state.active_tab {
                VCTab::Branches => show_branches_tab(
                    ui, vc, state, panel_state,
                    new_branch_name, new_branch_desc, new_branch_dialog,
                    needs_eval,
                ),
                VCTab::History => show_history_tab(
                    ui, vc, state, panel_state, needs_eval,
                ),
                VCTab::Changes => show_changes_tab(
                    ui, vc, state, panel_state,
                    commit_message, discard_confirm, needs_eval,
                ),
            }
        });
}

// ── Branches tab ──────────────────────────────────────────────────────────────

fn show_branches_tab(
    ui: &mut Ui,
    vc: &mut VersionControlState,
    state: &mut AppState,
    panel_state: &mut VCPanelState,
    new_branch_name: &mut String,
    new_branch_desc: &mut String,
    new_branch_dialog: &mut bool,
    needs_eval: &mut bool,
) {
    ui.horizontal(|ui| {
        ui.heading("Branches");
        if ui.button("+ New Branch").clicked() {
            *new_branch_dialog = true;
        }
    });

    // New branch dialog
    if *new_branch_dialog {
        ui.group(|ui| {
            ui.label("Branch name:");
            ui.text_edit_singleline(new_branch_name);
            ui.label("Description (optional):");
            ui.text_edit_singleline(new_branch_desc);
            ui.horizontal(|ui| {
                let can_create = !new_branch_name.trim().is_empty();
                if ui.add_enabled(can_create, egui::Button::new("Create")).clicked() {
                    let name = new_branch_name.trim().to_string();
                    let desc = new_branch_desc.trim().to_string();
                    match create_branch(vc, name, desc) {
                        Ok(_) => {
                            *new_branch_name = String::new();
                            *new_branch_desc = String::new();
                            *new_branch_dialog = false;
                        }
                        Err(e) => {
                            // Show error (could improve this with a status field)
                            ui.colored_label(Color32::RED, e);
                        }
                    }
                }
                if ui.button("Cancel").clicked() {
                    *new_branch_dialog = false;
                }
            });
        });
        ui.separator();
    }

    // Branch list
    let branch_names: Vec<String> = vc.branches.keys().cloned().collect();
    let current_branch = vc.current_branch.clone();

    // Compute commit counts for display
    let mut checkout_target: Option<String> = None;
    let mut delete_target: Option<String> = None;
    let mut merge_target: Option<String> = None;

    egui::ScrollArea::vertical().max_height(300.0).show(ui, |ui| {
        for name in &branch_names {
            let branch = &vc.branches[name];
            let is_current = *name == current_branch;
            let head_commit = vc.commits.get(&branch.head_commit_id);
            let last_time = head_commit.map(|c| c.timestamp.format("%Y-%m-%d %H:%M").to_string())
                .unwrap_or_else(|| "unknown".to_string());

            ui.group(|ui| {
                ui.horizontal(|ui| {
                    if is_current {
                        ui.colored_label(Color32::from_rgb(100, 150, 255),
                            RichText::new(format!("● {}", name)).strong());
                    } else {
                        ui.label(RichText::new(format!("  {}", name)));
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(RichText::new(&last_time).small().weak());
                    });
                });

                if !branch.description.is_empty() {
                    ui.label(RichText::new(&branch.description).small().weak());
                }

                if !is_current {
                    ui.horizontal(|ui| {
                        if ui.small_button("Checkout").clicked() {
                            checkout_target = Some(name.clone());
                        }
                        if ui.small_button("Merge into current").clicked() {
                            merge_target = Some(name.clone());
                        }
                        if *name != "main" {
                            if ui.small_button(RichText::new("Delete").color(Color32::RED)).clicked() {
                                delete_target = Some(name.clone());
                            }
                        }
                    });
                } else {
                    ui.label(RichText::new("(current)").small().color(Color32::from_rgb(100, 150, 255)));
                }
            });
        }
    });

    // Process deferred actions
    if let Some(target) = checkout_target {
        match checkout_branch(vc, state, &target) {
            Ok(_) => { *needs_eval = true; }
            Err(e) => {
                // Store error for display next frame
                panel_state.selected_branch = Some(e);
            }
        }
    }
    if let Some(target) = delete_target {
        let _ = delete_branch(vc, &target);
    }
    if let Some(target) = merge_target {
        match merge(vc, state, &target) {
            Ok(MergeResult::Success) => { *needs_eval = true; }
            Ok(MergeResult::Conflict(conflict)) => {
                panel_state.conflict_resolutions = conflict.script_conflicts.iter()
                    .map(|_| ConflictResolution::UseCurrent)
                    .collect();
                panel_state.dim_resolutions = conflict.dimension_conflicts.iter()
                    .map(|_| true)
                    .collect();
                panel_state.conflict = Some(conflict);
            }
            Err(_) => {}
        }
    }

    // Show checkout error if stored
    if let Some(ref err) = panel_state.selected_branch {
        if err.starts_with("Cannot checkout") {
            ui.separator();
            ui.colored_label(Color32::from_rgb(255, 180, 50), err);
        }
    }
}

// ── History tab ───────────────────────────────────────────────────────────────

fn show_history_tab(
    ui: &mut Ui,
    vc: &mut VersionControlState,
    state: &mut AppState,
    panel_state: &mut VCPanelState,
    needs_eval: &mut bool,
) {
    let graph = get_commit_graph(vc);
    let total = graph.len();
    let limit = panel_state.history_limit;

    ui.horizontal(|ui| {
        ui.heading("History");
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.label(RichText::new(format!("{} commits", total)).small().weak());
        });
    });

    egui::ScrollArea::vertical().max_height(400.0).show(ui, |ui| {
        let display_count = graph.len().min(limit);
        for (idx, node) in graph.iter().take(display_count).enumerate() {
            let is_last = idx + 1 == display_count;

            ui.horizontal(|ui| {
                // Graph line column (fixed width)
                ui.set_min_width(20.0);
                let (rect, _) = ui.allocate_exact_size(
                    egui::vec2(20.0, 28.0),
                    egui::Sense::hover(),
                );
                let painter = ui.painter_at(rect);
                let cx = rect.center().x;
                let top = rect.min.y;
                let bot = rect.max.y;
                let mid = rect.center().y;

                // Vertical line (connect to next commit)
                if !is_last {
                    painter.line_segment(
                        [egui::pos2(cx, mid), egui::pos2(cx, bot)],
                        egui::Stroke::new(1.5, Color32::GRAY),
                    );
                }
                if idx > 0 {
                    painter.line_segment(
                        [egui::pos2(cx, top), egui::pos2(cx, mid)],
                        egui::Stroke::new(1.5, Color32::GRAY),
                    );
                }

                // Dot
                let radius = if node.is_head { 6.0 } else { 4.0 };
                let dot_color = if node.is_head {
                    Color32::from_rgb(100, 200, 100)
                } else if node.is_current_branch_head {
                    Color32::from_rgb(100, 150, 255)
                } else {
                    Color32::GRAY
                };
                painter.circle_filled(egui::pos2(cx, mid), radius, dot_color);
                if node.parent_ids.len() > 1 {
                    // Merge commit — draw a second line
                    painter.circle_stroke(egui::pos2(cx, mid), radius + 2.0,
                        egui::Stroke::new(1.0, Color32::from_rgb(200, 150, 50)));
                }

                // Commit info column
                ui.vertical(|ui| {
                    ui.horizontal(|ui| {
                        // Short hash badge
                        ui.label(RichText::new(&node.short_id).monospace().small()
                            .color(Color32::from_rgb(150, 150, 150)));

                        // Branch labels
                        for branch_name in &node.branch_names {
                            let color = if *branch_name == vc.current_branch {
                                Color32::from_rgb(100, 150, 255)
                            } else {
                                Color32::from_rgb(150, 200, 100)
                            };
                            ui.label(RichText::new(format!("[{}]", branch_name))
                                .small().color(color).strong());
                        }

                        if node.is_head {
                            ui.label(RichText::new("HEAD").small()
                                .color(Color32::from_rgb(100, 200, 100)).strong());
                        }

                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if !node.is_head {
                                if ui.small_button("Checkout").clicked() {
                                    panel_state.pending_checkout_commit = Some(node.commit_id.clone());
                                }
                            }
                            let ts = node.timestamp.format("%m-%d %H:%M").to_string();
                            ui.label(RichText::new(ts).small().weak());
                        });
                    });

                    // Message
                    let msg_text = RichText::new(&node.message).small();
                    let msg_text = if node.is_head { msg_text.strong() } else { msg_text };
                    ui.label(msg_text);
                });
            });
        }

        if total > limit {
            if ui.button(format!("Load more ({} remaining)", total - limit)).clicked() {
                panel_state.history_limit += 100;
            }
        }
    });

    // Process deferred checkout
    if let Some(commit_id) = panel_state.pending_checkout_commit.take() {
        match checkout_commit(vc, state, &commit_id) {
            Ok(_) => { *needs_eval = true; }
            Err(e) => {
                ui.colored_label(Color32::from_rgb(255, 180, 50), e);
            }
        }
    }
}

// ── Changes tab ───────────────────────────────────────────────────────────────

fn show_changes_tab(
    ui: &mut Ui,
    vc: &mut VersionControlState,
    state: &mut AppState,
    panel_state: &mut VCPanelState,
    commit_message: &mut String,
    discard_confirm: &mut bool,
    needs_eval: &mut bool,
) {
    let head_state = vc.head_commit_id.as_ref()
        .and_then(|id| vc.commits.get(id))
        .map(|c| c.state.clone());

    let has_changes = vc.working_changes;

    if !has_changes {
        ui.vertical_centered(|ui| {
            ui.add_space(20.0);
            ui.label(RichText::new("✓ Working tree clean").color(Color32::from_rgb(100, 200, 100)));
            ui.label(RichText::new("No uncommitted changes").weak().small());
        });
        return;
    }

    // Top action bar
    ui.horizontal(|ui| {
        if ui.button("Commit…").clicked() {
            if let Some(ref hs) = head_state {
                panel_state.commit_msg_suggestion = suggest_commit_message(state, hs);
                if commit_message.is_empty() {
                    *commit_message = panel_state.commit_msg_suggestion.clone();
                }
            }
            panel_state.commit_dialog_open = true;
        }

        if *discard_confirm {
            ui.colored_label(Color32::RED, "Discard all changes? ");
            if ui.button("Yes, discard").clicked() {
                discard_changes(vc, state);
                *discard_confirm = false;
                *needs_eval = true;
            }
            if ui.button("Cancel").clicked() {
                *discard_confirm = false;
            }
        } else {
            if ui.button(RichText::new("Discard All").color(Color32::RED)).clicked() {
                *discard_confirm = true;
            }
        }
    });

    // Commit dialog
    if panel_state.commit_dialog_open {
        ui.group(|ui| {
            ui.label("Commit message:");
            ui.text_edit_multiline(commit_message);
            ui.horizontal(|ui| {
                let can_commit = !commit_message.trim().is_empty();
                if ui.add_enabled(can_commit, egui::Button::new("Commit")).clicked() {
                    let msg = commit_message.clone();
                    commit(vc, state, msg, "User".to_string());
                    *commit_message = String::new();
                    panel_state.commit_dialog_open = false;
                }
                if ui.button("Cancel").clicked() {
                    panel_state.commit_dialog_open = false;
                }
            });
        });
        ui.separator();
    }

    if let Some(ref head) = head_state {
        egui::ScrollArea::vertical().max_height(350.0).show(ui, |ui| {
            // Script changes section
            if state.script_text != head.script_text {
                ui.collapsing("Script Changes", |ui| {
                    let diff_lines = compute_diff_display(&head.script_text, &state.script_text, 3);
                    egui::ScrollArea::vertical().max_height(200.0).id_salt("script_diff").show(ui, |ui| {
                        for dl in &diff_lines {
                            let (text, color) = match dl.kind {
                                DiffKind::Added   => (format!("+ {}", dl.content), Color32::from_rgb(80, 200, 80)),
                                DiffKind::Removed => (format!("- {}", dl.content), Color32::from_rgb(200, 80, 80)),
                                DiffKind::Same    => (format!("  {}", dl.content), Color32::GRAY),
                            };
                            ui.label(RichText::new(text).monospace().small().color(color));
                        }
                    });
                });
            }

            // Dimension changes section
            let changed_dims: Vec<(&str, f64, Option<f64>)> = state.dimensions.iter()
                .filter_map(|(k, &v)| {
                    let prev = head.dimensions.get(k).copied();
                    if prev != Some(v) {
                        Some((k.as_str(), v, prev))
                    } else {
                        None
                    }
                })
                .collect();

            // Also check for deleted dims
            let deleted_dims: Vec<(&str, f64)> = head.dimensions.iter()
                .filter(|(k, _)| !state.dimensions.contains_key(*k))
                .map(|(k, &v)| (k.as_str(), v))
                .collect();

            if !changed_dims.is_empty() || !deleted_dims.is_empty() {
                ui.collapsing("Dimension Changes", |ui| {
                    egui::Grid::new("dim_changes").striped(true).show(ui, |ui| {
                        ui.label(RichText::new("Name").strong());
                        ui.label(RichText::new("Before").strong());
                        ui.label(RichText::new("After").strong());
                        ui.end_row();

                        for (name, new_val, old_val) in &changed_dims {
                            ui.label(*name);
                            match old_val {
                                Some(v) => ui.label(RichText::new(format!("{:.3}", v)).color(Color32::from_rgb(200, 80, 80))),
                                None => ui.label(RichText::new("(new)").weak()),
                            };
                            ui.label(RichText::new(format!("{:.3}", new_val)).color(Color32::from_rgb(80, 200, 80)));
                            ui.end_row();
                        }
                        for (name, old_val) in &deleted_dims {
                            ui.label(*name);
                            ui.label(RichText::new(format!("{:.3}", old_val)).color(Color32::from_rgb(200, 80, 80)));
                            ui.label(RichText::new("(deleted)").weak());
                            ui.end_row();
                        }
                    });
                });
            }

            // Profile changes section
            let profiles_json_current = serde_json::to_string(&state.profiles).unwrap_or_default();
            let profiles_json_head = serde_json::to_string(&head.profiles).unwrap_or_default();
            if profiles_json_current != profiles_json_head {
                ui.collapsing("Profile Changes", |ui| {
                    for k in state.profiles.keys() {
                        let in_head = head.profiles.contains_key(k);
                        let same = head.profiles.get(k)
                            .map(|p| serde_json::to_string(p).unwrap_or_default())
                            == state.profiles.get(k).map(|p| serde_json::to_string(p).unwrap_or_default());
                        if !in_head {
                            ui.colored_label(Color32::from_rgb(80, 200, 80), format!("+ {}", k));
                        } else if !same {
                            ui.colored_label(Color32::YELLOW, format!("~ {}", k));
                        }
                    }
                    for k in head.profiles.keys() {
                        if !state.profiles.contains_key(k) {
                            ui.colored_label(Color32::from_rgb(200, 80, 80), format!("- {}", k));
                        }
                    }
                });
            }
        });
    }
}

// ── Conflict resolution modal ─────────────────────────────────────────────────

fn show_conflict_modal(
    ctx: &egui::Context,
    vc: &mut VersionControlState,
    state: &mut AppState,
    panel_state: &mut VCPanelState,
    needs_eval: &mut bool,
) {
    let mut finish_merge = false;
    let mut cancel_merge = false;

    egui::Window::new("Merge Conflicts")
        .collapsible(false)
        .resizable(true)
        .default_width(600.0)
        .show(ctx, |ui| {
            ui.colored_label(Color32::from_rgb(255, 180, 50),
                "⚠ Merge conflicts detected. Resolve all before finishing.");
            ui.separator();

            if let Some(ref conflict) = panel_state.conflict {
                egui::ScrollArea::vertical().max_height(400.0).show(ui, |ui| {
                    // Script conflicts
                    if !conflict.script_conflicts.is_empty() {
                        ui.heading("Script Conflicts");
                        for (i, sc) in conflict.script_conflicts.iter().enumerate() {
                            ui.group(|ui| {
                                ui.label(format!("Lines {}–{}", sc.line_start, sc.line_end));

                                ui.columns(3, |cols| {
                                    cols[0].label(RichText::new("Base").strong());
                                    for l in &sc.base_lines {
                                        cols[0].label(RichText::new(l).monospace().small());
                                    }

                                    cols[1].label(RichText::new("Current").strong()
                                        .color(Color32::from_rgb(80, 150, 255)));
                                    for l in &sc.current_lines {
                                        cols[1].label(RichText::new(l).monospace().small()
                                            .color(Color32::from_rgb(80, 150, 255)));
                                    }

                                    cols[2].label(RichText::new("Source").strong()
                                        .color(Color32::from_rgb(80, 200, 80)));
                                    for l in &sc.source_lines {
                                        cols[2].label(RichText::new(l).monospace().small()
                                            .color(Color32::from_rgb(80, 200, 80)));
                                    }
                                });

                                if i < panel_state.conflict_resolutions.len() {
                                    ui.horizontal(|ui| {
                                        let is_current = matches!(
                                            panel_state.conflict_resolutions[i],
                                            ConflictResolution::UseCurrent
                                        );
                                        let is_source = matches!(
                                            panel_state.conflict_resolutions[i],
                                            ConflictResolution::UseSource
                                        );
                                        if ui.selectable_label(is_current, "Use Current").clicked() {
                                            panel_state.conflict_resolutions[i] = ConflictResolution::UseCurrent;
                                        }
                                        if ui.selectable_label(is_source, "Use Source").clicked() {
                                            panel_state.conflict_resolutions[i] = ConflictResolution::UseSource;
                                        }
                                    });
                                }
                            });
                        }
                    }

                    // Dimension conflicts
                    if !conflict.dimension_conflicts.is_empty() {
                        ui.heading("Dimension Conflicts");
                        egui::Grid::new("dim_conflicts").striped(true).show(ui, |ui| {
                            ui.label(RichText::new("Name").strong());
                            ui.label(RichText::new("Current").strong());
                            ui.label(RichText::new("Source").strong());
                            ui.label(RichText::new("Use Current?").strong());
                            ui.end_row();

                            for (i, (name, current_val, source_val)) in conflict.dimension_conflicts.iter().enumerate() {
                                ui.label(name);
                                ui.colored_label(Color32::from_rgb(80, 150, 255), format!("{:.3}", current_val));
                                ui.colored_label(Color32::from_rgb(80, 200, 80), format!("{:.3}", source_val));
                                if i < panel_state.dim_resolutions.len() {
                                    ui.checkbox(&mut panel_state.dim_resolutions[i], "");
                                }
                                ui.end_row();
                            }
                        });
                    }

                    // Profile conflicts
                    if !conflict.profile_conflicts.is_empty() {
                        ui.heading("Profile Conflicts");
                        for name in &conflict.profile_conflicts {
                            ui.label(format!("  Profile '{}' modified in both branches", name));
                            ui.label(RichText::new("  (current version will be kept)").weak().small());
                        }
                    }
                });
            }

            ui.separator();
            let all_resolved = panel_state.conflict.as_ref()
                .map(|c| c.script_conflicts.len() == panel_state.conflict_resolutions.len())
                .unwrap_or(false);

            ui.horizontal(|ui| {
                if ui.add_enabled(all_resolved, egui::Button::new("Finish Merge")).clicked() {
                    finish_merge = true;
                }
                if ui.button("Cancel").clicked() {
                    cancel_merge = true;
                }
            });
        });

    if cancel_merge {
        panel_state.conflict = None;
        panel_state.conflict_resolutions.clear();
        panel_state.dim_resolutions.clear();
    }

    if finish_merge {
        apply_conflict_resolutions(vc, state, panel_state, needs_eval);
    }
}

fn apply_conflict_resolutions(
    vc: &mut VersionControlState,
    state: &mut AppState,
    panel_state: &mut VCPanelState,
    needs_eval: &mut bool,
) {
    if let Some(conflict) = panel_state.conflict.take() {
        // Apply script conflict resolutions
        let mut script_lines: Vec<String> = state.script_text.lines().map(|s| s.to_string()).collect();

        for (i, resolution) in panel_state.conflict_resolutions.iter().enumerate() {
            if i < conflict.script_conflicts.len() {
                let sc = &conflict.script_conflicts[i];
                let replacement = match resolution {
                    ConflictResolution::UseCurrent => sc.current_lines.clone(),
                    ConflictResolution::UseSource  => sc.source_lines.clone(),
                    ConflictResolution::Manual(lines) => lines.clone(),
                };
                // Replace lines in range
                let end = sc.line_end.min(script_lines.len().saturating_sub(1));
                if sc.line_start <= end && sc.line_start < script_lines.len() {
                    script_lines.splice(sc.line_start..=end, replacement);
                }
            }
        }
        state.script_text = script_lines.join("\n");

        // Apply dimension conflict resolutions
        for (i, (name, _current_val, source_val)) in conflict.dimension_conflicts.iter().enumerate() {
            let use_current = panel_state.dim_resolutions.get(i).copied().unwrap_or(true);
            if !use_current {
                state.dimensions.insert(name.clone(), *source_val);
            }
        }

        // Create merge commit
        let timestamp = chrono::Utc::now();
        let ts_str = timestamp.to_rfc3339();
        let dims_str = format!("{:?}", state.dimensions);
        let merge_msg = "Merge (resolved conflicts)".to_string();
        let merge_id = crate::version_control::CommitId::generate(
            &ts_str, &merge_msg, &state.script_text, &dims_str,
        );

        let parent_ids = vc.head_commit_id.iter().cloned().collect();
        let merge_commit = crate::version_control::Commit {
            id: merge_id.clone(),
            parent_ids,
            author: "User".to_string(),
            timestamp,
            message: merge_msg,
            state: crate::version_control::ProjectState::from_app_state(state),
            thumbnail: None,
        };

        vc.commits.insert(merge_id.clone(), merge_commit);
        if let Some(branch) = vc.branches.get_mut(&vc.current_branch) {
            branch.head_commit_id = merge_id.clone();
        }
        vc.head_commit_id = Some(merge_id);
        vc.working_changes = false;

        panel_state.conflict_resolutions.clear();
        panel_state.dim_resolutions.clear();
        *needs_eval = true;
    }
}
