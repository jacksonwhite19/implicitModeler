// Project tree panel — hierarchical view of all named entities in the script.

use std::collections::HashMap;
use eframe::egui::{self, Color32, RichText};
use crate::scripting::MassPoint;
use crate::fea::FEASetup;
use crate::sdf::spine::LongitudinalSplines;

// ── Tree node types ───────────────────────────────────────────────────────────

/// A stable ID used to preserve collapse-state and selection across rebuilds.
pub type NodeId = String;

/// A node in the project tree.
pub enum TreeNode {
    /// A collapsible group with children.
    Group {
        id:       NodeId,
        label:    String,
        icon:     &'static str,
        color:    Color32,
        children: Vec<TreeNode>,
    },
    /// A leaf node with an optional script name (used for rename / jump).
    Leaf {
        id:          NodeId,
        label:       String,
        icon:        &'static str,
        color:       Color32,
        /// The quoted name as it appears in the script (None if not applicable).
        script_name: Option<String>,
    },
}

impl TreeNode {
    pub fn id(&self) -> &str {
        match self {
            TreeNode::Group { id, .. } => id,
            TreeNode::Leaf  { id, .. } => id,
        }
    }
}

// ── Rename popup state ────────────────────────────────────────────────────────

pub struct RenameState {
    pub node_id:    NodeId,
    pub old_name:   String,
    pub buffer:     String,
    /// true when we're waiting for user to confirm >3 occurrences
    pub needs_confirm: bool,
    pub occurrence_count: usize,
}

// ── ProjectTree (persistent state) ───────────────────────────────────────────

/// All state for the project tree panel that persists across frames.
#[derive(Default)]
pub struct ProjectTree {
    /// Top-level tree nodes (rebuilt after each eval).
    pub nodes: Vec<TreeNode>,
    /// Collapse open/closed per node id. `true` = open.
    pub collapse: HashMap<NodeId, bool>,
    /// Currently selected node id.
    pub selected: Option<NodeId>,
    /// Pending rename operation.
    pub rename: Option<RenameState>,
    /// Whether the tree has ever been populated (to show empty state).
    pub has_been_populated: bool,
}

impl ProjectTree {
    /// Rebuild tree from the latest script results.
    ///
    /// Previous collapse state and selection are preserved by matching node IDs.
    pub fn rebuild(
        &mut self,
        mass_points:   &[MassPoint],
        fea_setup:     &FEASetup,
        profile_names: &[String],
        splines:       &LongitudinalSplines,
        ref_points:    &[crate::scripting::ReferencePoint],
    ) {
        let new_nodes = build_tree(mass_points, fea_setup, profile_names, splines, ref_points);
        // Mark any new nodes as open by default
        mark_defaults(&new_nodes, &mut self.collapse);
        // Check if the previously selected node still exists
        if let Some(ref sel) = self.selected {
            if !node_id_exists(&new_nodes, sel) {
                self.selected = None;
            }
        }
        self.nodes = new_nodes;
        let populated = !mass_points.is_empty()
            || !fea_setup.is_empty()
            || !profile_names.is_empty()
            || splines.spine.keel.is_some()
            || splines.spine.deck.is_some()
            || splines.chine.chine_y.is_some()
            || !ref_points.is_empty();
        if populated {
            self.has_been_populated = true;
        }
    }

    /// Clear the tree (on script error or empty script, but keep populated flag).
    #[allow(dead_code)] // Available for error recovery
    pub fn clear(&mut self) {
        self.nodes.clear();
        self.selected = None;
    }
}

// ── Tree building ─────────────────────────────────────────────────────────────

fn build_tree(
    mass_points:   &[MassPoint],
    fea_setup:     &FEASetup,
    profile_names: &[String],
    splines:       &LongitudinalSplines,
    ref_points:    &[crate::scripting::ReferencePoint],
) -> Vec<TreeNode> {
    let mut roots: Vec<TreeNode> = Vec::new();

    // ── Components (mass points) ──────────────────────────────────────────────
    if !mass_points.is_empty() {
        let children: Vec<TreeNode> = mass_points.iter().map(|mp| {
            TreeNode::Leaf {
                id:          format!("component:{}", mp.name),
                label:       format!("{} ({:.0} g)", mp.name, mp.mass_g),
                icon:        "📦",
                color:       Color32::from_rgb(120, 200, 255),
                script_name: Some(mp.name.clone()),
            }
        }).collect();
        roots.push(TreeNode::Group {
            id:       "group:components".into(),
            label:    format!("Components ({})", children.len()),
            icon:     "📦",
            color:    Color32::from_rgb(120, 200, 255),
            children,
        });
    }

    // ── Spline profiles ───────────────────────────────────────────────────────
    if !profile_names.is_empty() {
        let mut sorted = profile_names.to_vec();
        sorted.sort();
        let children: Vec<TreeNode> = sorted.iter().map(|name| {
            TreeNode::Leaf {
                id:          format!("profile:{}", name),
                label:       name.clone(),
                icon:        "〰",
                color:       Color32::from_rgb(160, 255, 160),
                script_name: Some(name.clone()),
            }
        }).collect();
        roots.push(TreeNode::Group {
            id:       "group:profiles".into(),
            label:    format!("Profiles ({})", children.len()),
            icon:     "〰",
            color:    Color32::from_rgb(160, 255, 160),
            children,
        });
    }

    // ── Longitudinal spine ────────────────────────────────────────────────────
    let has_spine = splines.spine.keel.is_some()
        || splines.spine.deck.is_some()
        || splines.chine.chine_y.is_some();
    if has_spine {
        let mut children = Vec::new();
        if splines.spine.keel.is_some() {
            children.push(TreeNode::Leaf {
                id:          "spine:keel".into(),
                label:       "Keel Line".into(),
                icon:        "⎯",
                color:       Color32::from_rgb(200, 160, 80),
                script_name: None,
            });
        }
        if splines.spine.deck.is_some() {
            children.push(TreeNode::Leaf {
                id:          "spine:deck".into(),
                label:       "Deck Line".into(),
                icon:        "⎯",
                color:       Color32::from_rgb(200, 160, 80),
                script_name: None,
            });
        }
        if splines.chine.chine_y.is_some() {
            children.push(TreeNode::Leaf {
                id:          "spine:chine".into(),
                label:       "Chine Line".into(),
                icon:        "⎯",
                color:       Color32::from_rgb(200, 160, 80),
                script_name: None,
            });
        }
        roots.push(TreeNode::Group {
            id:       "group:spine".into(),
            label:    "Longitudinal Spine".into(),
            icon:     "⎯",
            color:    Color32::from_rgb(200, 160, 80),
            children,
        });
    }

    // ── FEA conditions ────────────────────────────────────────────────────────
    if !fea_setup.is_empty() {
        let mut children = Vec::new();
        for r in &fea_setup.fixed_supports {
            children.push(TreeNode::Leaf {
                id:          format!("fea:fixed:{}", r.name),
                label:       format!("Fixed — {}", r.name),
                icon:        "🔒",
                color:       Color32::from_rgb(80, 120, 255),
                script_name: Some(r.name.clone()),
            });
        }
        for r in &fea_setup.fixed_axes {
            children.push(TreeNode::Leaf {
                id:          format!("fea:fixed_axis:{}", r.name),
                label:       format!("Fixed Axis — {}", r.name),
                icon:        "🔒",
                color:       Color32::from_rgb(80, 120, 255),
                script_name: Some(r.name.clone()),
            });
        }
        for r in &fea_setup.force_loads {
            children.push(TreeNode::Leaf {
                id:          format!("fea:force:{}", r.name),
                label:       format!("Force — {}", r.name),
                icon:        "↗",
                color:       Color32::from_rgb(255, 220, 60),
                script_name: Some(r.name.clone()),
            });
        }
        for r in &fea_setup.pressure_loads {
            children.push(TreeNode::Leaf {
                id:          format!("fea:pressure:{}", r.name),
                label:       format!("Pressure — {}", r.name),
                icon:        "⬤",
                color:       Color32::from_rgb(255, 160, 60),
                script_name: Some(r.name.clone()),
            });
        }
        for r in &fea_setup.torque_loads {
            children.push(TreeNode::Leaf {
                id:          format!("fea:torque:{}", r.name),
                label:       format!("Torque — {}", r.name),
                icon:        "↺",
                color:       Color32::from_rgb(255, 160, 200),
                script_name: Some(r.name.clone()),
            });
        }
        for r in &fea_setup.motor_thrusts {
            children.push(TreeNode::Leaf {
                id:          format!("fea:motor:{}", r.name),
                label:       format!("Motor — {}", r.name),
                icon:        "⚙",
                color:       Color32::from_rgb(180, 255, 180),
                script_name: Some(r.name.clone()),
            });
        }
        if fea_setup.gravity.is_some() {
            children.push(TreeNode::Leaf {
                id:          "fea:gravity".into(),
                label:       "Gravity".into(),
                icon:        "↓",
                color:       Color32::from_rgb(200, 200, 200),
                script_name: None,
            });
        }
        roots.push(TreeNode::Group {
            id:       "group:fea".into(),
            label:    format!("FEA Conditions ({})", children.len()),
            icon:     "🔬",
            color:    Color32::from_rgb(200, 200, 200),
            children,
        });
    }

    // ── Reference points ─────────────────────────────────────────────────────
    if !ref_points.is_empty() {
        let children: Vec<TreeNode> = ref_points.iter().map(|rp| {
            let color = Color32::from_rgb(
                (rp.color[0] * 255.0) as u8,
                (rp.color[1] * 255.0) as u8,
                (rp.color[2] * 255.0) as u8,
            );
            TreeNode::Leaf {
                id:          format!("refpt_{}", rp.name),
                label:       format!("{} ({:.1}, {:.1}, {:.1})", rp.name, rp.position.x, rp.position.y, rp.position.z),
                icon:        "◉",
                color,
                script_name: None,
            }
        }).collect();
        roots.push(TreeNode::Group {
            id:       "ref_points".to_string(),
            label:    format!("Reference Points ({})", children.len()),
            icon:     "◎",
            color:    Color32::from_rgb(200, 180, 100),
            children,
        });
    }

    roots
}

/// Recursively mark any new node ID as open (true) if not yet in the collapse map.
fn mark_defaults(nodes: &[TreeNode], collapse: &mut HashMap<NodeId, bool>) {
    for node in nodes {
        match node {
            TreeNode::Group { id, children, .. } => {
                collapse.entry(id.clone()).or_insert(true);
                mark_defaults(children, collapse);
            }
            TreeNode::Leaf { id, .. } => {
                collapse.entry(id.clone()).or_insert(false);
            }
        }
    }
}

/// Returns true if any node in the tree has this id.
fn node_id_exists(nodes: &[TreeNode], id: &str) -> bool {
    for node in nodes {
        if node.id() == id { return true; }
        if let TreeNode::Group { children, .. } = node {
            if node_id_exists(children, id) { return true; }
        }
    }
    false
}

// ── Rendering ─────────────────────────────────────────────────────────────────

/// Result of showing the project tree: what the host should do next.
pub struct TreeInteraction {
    /// If Some, jump the editor cursor to show this script name.
    pub jump_to_name: Option<String>,
    /// If Some, the user wants to rename this node: (node_id, old_name).
    pub start_rename: Option<(NodeId, String)>,
    /// If Some, a rename was confirmed: (old_name, new_name).
    pub confirmed_rename: Option<(String, String)>,
}

/// Render the project tree panel into `ui`.
///
/// Returns a `TreeInteraction` describing any click or rename action.
pub fn show_project_tree(
    ui:   &mut egui::Ui,
    tree: &mut ProjectTree,
) -> TreeInteraction {
    let mut interaction = TreeInteraction { jump_to_name: None, start_rename: None, confirmed_rename: None };

    // ── Rename popup (if active) ──────────────────────────────────────────────
    let mut confirm_rename: Option<(String, String, String)> = None; // (old, new, id)
    let mut cancel_rename = false;
    if let Some(ref mut rs) = tree.rename {
        egui::Window::new("Rename")
            .collapsible(false)
            .resizable(false)
            .default_width(240.0)
            .show(ui.ctx(), |ui| {
                ui.label(format!("Rename \"{}\":", rs.old_name));
                let response = ui.text_edit_singleline(&mut rs.buffer);
                if response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                    cancel_rename = true;
                }
                ui.horizontal(|ui| {
                    let ok_clicked = ui.button("OK").clicked()
                        || (response.lost_focus()
                            && ui.input(|i| i.key_pressed(egui::Key::Enter)));
                    if ok_clicked && !rs.buffer.is_empty() && rs.buffer != rs.old_name {
                        confirm_rename = Some((rs.old_name.clone(), rs.buffer.clone(), rs.node_id.clone()));
                    }
                    if ui.button("Cancel").clicked() {
                        cancel_rename = true;
                    }
                });
                if rs.needs_confirm {
                    ui.separator();
                    ui.colored_label(
                        Color32::YELLOW,
                        format!("⚠ Name appears {} times. All will be replaced.", rs.occurrence_count),
                    );
                }
            });
    }
    if cancel_rename {
        tree.rename = None;
    }
    if let Some((old, new, _id)) = confirm_rename {
        interaction.confirmed_rename = Some((old, new));
        tree.rename = None;
    }

    // ── Empty state ───────────────────────────────────────────────────────────
    if tree.nodes.is_empty() {
        ui.add_space(8.0);
        ui.label(
            RichText::new(if tree.has_been_populated {
                "No geometry — fix script errors\nto repopulate the tree."
            } else {
                "No geometry yet — run your\nscript to populate the\nproject tree."
            })
            .italics()
            .color(Color32::GRAY),
        );
        return interaction;
    }

    // ── Tree nodes ────────────────────────────────────────────────────────────
    for node in &tree.nodes {
        show_node(ui, node, &mut tree.collapse, &mut tree.selected, &mut interaction);
    }

    interaction
}

fn show_node(
    ui:          &mut egui::Ui,
    node:        &TreeNode,
    collapse:    &mut HashMap<NodeId, bool>,
    selected:    &mut Option<NodeId>,
    interaction: &mut TreeInteraction,
) {
    match node {
        TreeNode::Group { id, label, icon, color, children } => {
            let is_open = *collapse.get(id).unwrap_or(&true);
            let header = egui::collapsing_header::CollapsingState::load_with_default_open(
                ui.ctx(),
                egui::Id::new(id),
                is_open,
            );
            header.show_header(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.colored_label(*color, *icon);
                    ui.label(label.as_str());
                });
            }).body(|ui| {
                for child in children {
                    show_node(ui, child, collapse, selected, interaction);
                }
            });
            // Sync collapse state back to our map.
            let updated = egui::collapsing_header::CollapsingState::load_with_default_open(
                ui.ctx(),
                egui::Id::new(id),
                is_open,
            );
            collapse.insert(id.clone(), updated.is_open());
        }
        TreeNode::Leaf { id, label, icon, color, script_name } => {
            let is_selected = selected.as_deref() == Some(id.as_str());
            let bg = if is_selected {
                Color32::from_rgba_unmultiplied(100, 160, 255, 60)
            } else {
                Color32::TRANSPARENT
            };

            let row = egui::Frame::none()
                .fill(bg)
                .inner_margin(egui::Margin::symmetric(4.0, 2.0))
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.add_space(16.0); // indent
                        ui.colored_label(*color, *icon);
                        ui.label(label.as_str());
                    });
                });

            let resp = ui.interact(
                row.response.rect,
                egui::Id::new(id).with("click"),
                egui::Sense::click(),
            );

            // Left-click: select + jump
            if resp.clicked() {
                *selected = Some(id.clone());
                if let Some(name) = script_name {
                    interaction.jump_to_name = Some(name.clone());
                }
            }

            // Right-click: rename popup
            if resp.secondary_clicked() {
                if let Some(name) = script_name {
                    interaction.start_rename = Some((id.clone(), name.clone()));
                }
            }

            // Hover tooltip
            resp.on_hover_text(label.as_str());
        }
    }
}

// ── Script utilities ──────────────────────────────────────────────────────────

/// Find the character offset of the first occurrence of `name` (as a quoted string
/// or bare identifier) in `script`, returning (char_offset, line_number_0indexed).
pub fn find_name_in_script(script: &str, name: &str) -> Option<(usize, usize)> {
    // Prefer the quoted form (component_named / fea_region calls)
    let quoted = format!("\"{}\"", name);
    let pos = script.find(&quoted).or_else(|| script.find(name))?;
    let line = script[..pos].chars().filter(|&c| c == '\n').count();
    Some((pos, line))
}

/// Count occurrences of the quoted name in the script.
pub fn count_occurrences(script: &str, name: &str) -> usize {
    let quoted = format!("\"{}\"", name);
    let mut count = 0;
    let mut pos = 0;
    while let Some(found) = script[pos..].find(&quoted) {
        count += 1;
        pos += found + quoted.len();
    }
    count
}

/// Replace all occurrences of `old_name` (quoted) with `new_name` (quoted).
pub fn rename_in_script(script: &str, old_name: &str, new_name: &str) -> String {
    let old_quoted = format!("\"{}\"", old_name);
    let new_quoted = format!("\"{}\"", new_name);
    script.replace(&old_quoted, &new_quoted)
}
