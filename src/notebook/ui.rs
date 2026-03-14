// nTop-style notebook panel UI

use eframe::egui;
use super::types::*;

const ALL_CATEGORIES: &[(&str, &[NbBlockKind])] = &[
    ("Primitives", &[
        NbBlockKind::Sphere, NbBlockKind::Box_, NbBlockKind::Cylinder,
        NbBlockKind::Torus, NbBlockKind::Cone,
    ]),
    ("Booleans", &[
        NbBlockKind::Union, NbBlockKind::Subtract, NbBlockKind::Intersect,
        NbBlockKind::SmoothUnion, NbBlockKind::SmoothSubtract,
    ]),
    ("Transforms", &[
        NbBlockKind::Translate, NbBlockKind::Rotate, NbBlockKind::Scale,
        NbBlockKind::Offset, NbBlockKind::Shell,
    ]),
    ("Patterns", &[
        NbBlockKind::LinearArray, NbBlockKind::PolarArray,
        NbBlockKind::MirrorX, NbBlockKind::MirrorY, NbBlockKind::MirrorZ,
    ]),
    ("Aerospace", &[
        NbBlockKind::Fuselage, NbBlockKind::Nacelle,
        NbBlockKind::Wing, NbBlockKind::NacaAirfoil,
    ]),
];

/// Human-readable input slot names for 2-input blocks.
fn input_names(kind: NbBlockKind) -> (&'static str, &'static str) {
    match kind {
        NbBlockKind::Subtract | NbBlockKind::SmoothSubtract => ("Body", "Tool"),
        NbBlockKind::Intersect => ("A", "B"),
        _ => ("A", "B"),
    }
}

/// Draw the notebook panel. Returns true if anything changed (triggers re-evaluation).
pub fn show_notebook_panel(ui: &mut egui::Ui, notebook: &mut Notebook) -> bool {
    let mut changed = false;

    // ── Persistent state: which block is expanded ────────────────────────────
    let state_id = ui.id().with("nb_expanded");
    let mut expanded_id: Option<u64> = ui.ctx()
        .data_mut(|d| d.get_temp::<Option<u64>>(state_id))
        .flatten();

    // ── Toolbar ──────────────────────────────────────────────────────────────
    ui.horizontal(|ui| {
        ui.menu_button("➕ Add Block", |ui| {
            for &(cat, kinds) in ALL_CATEGORIES {
                ui.menu_button(cat, |ui| {
                    for &kind in kinds {
                        if ui.button(kind.display_name()).clicked() {
                            let id = notebook.add_block(kind);
                            notebook.preview_id = Some(id);
                            expanded_id = Some(id);
                            changed = true;
                            ui.close_menu();
                        }
                    }
                });
            }
        });

        if ui.small_button("── Section").clicked() {
            notebook.add_block(NbBlockKind::Section);
            changed = true;
        }

        if !notebook.blocks.is_empty() {
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let n_ops = notebook.blocks.iter().filter(|b| b.kind != NbBlockKind::Section).count();
                ui.label(
                    egui::RichText::new(format!("{} ops", n_ops))
                        .color(egui::Color32::from_gray(100))
                        .size(11.0)
                );
            });
        }
    });

    if notebook.blocks.is_empty() {
        ui.separator();
        ui.add_space(12.0);
        ui.vertical_centered(|ui| {
            ui.label(egui::RichText::new("Empty notebook").color(egui::Color32::from_gray(90)).italics());
            ui.label(egui::RichText::new("Use ➕ Add Block to start").color(egui::Color32::from_gray(75)).size(11.0));
        });
        ui.ctx().data_mut(|d| d.insert_temp(state_id, expanded_id));
        return changed;
    }

    ui.add_space(2.0);

    // Snapshot IDs & labels before mutable borrow
    let block_ids: Vec<u64> = notebook.blocks.iter().map(|b| b.id).collect();
    let block_labels: Vec<String> = (0..block_ids.len())
        .map(|i| notebook.block_label(i))
        .collect();
    let preview_id = notebook.preview_id;

    let mut to_delete: Option<u64> = None;
    let mut new_preview: Option<Option<u64>> = None;
    let mut toggle_expand: Option<u64> = None;

    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            for (i, &block_id) in block_ids.iter().enumerate() {
                let kind = notebook.blocks.iter().find(|b| b.id == block_id).unwrap().kind;

                // ── Section divider ──────────────────────────────────────────
                if kind == NbBlockKind::Section {
                    ui.add_space(6.0);
                    ui.horizontal(|ui| {
                        let avail = ui.available_width();
                        // Left rule
                        let r1 = ui.allocate_rect(
                            egui::Rect::from_min_size(ui.cursor().min, egui::vec2(12.0, 1.0)),
                            egui::Sense::hover(),
                        );
                        ui.painter().hline(
                            r1.rect.x_range(),
                            r1.rect.center().y,
                            egui::Stroke::new(1.0, egui::Color32::from_gray(55)),
                        );

                        // Editable label — inline text edit for section name
                        if let Some(block) = notebook.blocks.iter_mut().find(|b| b.id == block_id) {
                            if let Some(param) = block.string_params.first_mut() {
                                let te = egui::TextEdit::singleline(&mut param.value)
                                    .font(egui::TextStyle::Small)
                                    .desired_width(avail - 60.0)
                                    .frame(false)
                                    .text_color(egui::Color32::from_gray(140));
                                if ui.add(te).changed() {
                                    changed = true;
                                }
                            }
                        }

                        // Right rule + delete
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if ui.add(egui::Button::new(
                                egui::RichText::new("✕").color(egui::Color32::from_gray(100))
                            ).frame(false)).clicked() {
                                to_delete = Some(block_id);
                            }
                            let remaining = ui.available_width().max(4.0);
                            let r2 = ui.allocate_rect(
                                egui::Rect::from_min_size(ui.cursor().min, egui::vec2(remaining, 1.0)),
                                egui::Sense::hover(),
                            );
                            ui.painter().hline(
                                r2.rect.x_range(),
                                r2.rect.center().y,
                                egui::Stroke::new(1.0, egui::Color32::from_gray(55)),
                            );
                        });
                    });
                    ui.add_space(4.0);
                    continue;
                }

                // ── Regular block row ─────────────────────────────────────────
                let is_preview  = preview_id == Some(block_id);
                let is_expanded = expanded_id == Some(block_id);
                let [r, g, b_c] = kind.header_color();
                let accent = egui::Color32::from_rgb(r, g, b_c);
                let inputs = kind.sdf_inputs();

                // Gather prev-block labels available for input selectors
                let prev_entries: Vec<(u64, String)> = block_ids[..i]
                    .iter()
                    .zip(block_labels[..i].iter())
                    .filter(|(bid, _)| {
                        // Skip Section blocks in the input selector
                        notebook.blocks.iter().find(|b| b.id == **bid)
                            .map(|b| b.kind != NbBlockKind::Section)
                            .unwrap_or(false)
                    })
                    .map(|(&bid, lbl)| (bid, lbl.clone()))
                    .collect();

                let row_bg = if is_expanded {
                    egui::Color32::from_gray(32)
                } else {
                    egui::Color32::TRANSPARENT
                };

                // Draw the row
                let row_resp = egui::Frame::none()
                    .fill(row_bg)
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            // Colored left accent bar
                            let (rect, _) = ui.allocate_exact_size(
                                egui::vec2(3.0, 22.0), egui::Sense::hover(),
                            );
                            ui.painter().rect_filled(rect, 0.0, accent);
                            ui.add_space(5.0);

                            // Preview dot
                            let dot = if is_preview { "●" } else { "○" };
                            let dot_resp = ui.add(egui::Button::new(
                                egui::RichText::new(dot)
                                    .color(if is_preview {
                                        egui::Color32::from_rgb(100, 180, 255)
                                    } else {
                                        egui::Color32::from_gray(80)
                                    })
                                    .size(12.0)
                            ).frame(false));
                            if dot_resp.clicked() {
                                new_preview = Some(if is_preview { None } else { Some(block_id) });
                            }
                            dot_resp.on_hover_text(if is_preview {
                                "Previewing this block"
                            } else {
                                "Set as preview"
                            });

                            // Block label (kind name + index)
                            let label_text = egui::RichText::new(
                                format!("{} #{}", kind.display_name(), i + 1)
                            )
                            .color(if is_expanded {
                                egui::Color32::WHITE
                            } else {
                                egui::Color32::from_gray(190)
                            })
                            .size(12.5);

                            let lbl_resp = ui.add(
                                egui::Label::new(label_text).sense(egui::Sense::click())
                            );
                            if lbl_resp.clicked() {
                                toggle_expand = Some(block_id);
                            }

                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                // Delete button
                                if ui.add(egui::Button::new(
                                    egui::RichText::new("✕").color(egui::Color32::from_gray(100))
                                ).frame(false)).clicked() {
                                    to_delete = Some(block_id);
                                }

                                // Expand toggle (only for blocks with params or 2 inputs)
                                let has_body = !notebook.blocks.iter()
                                    .find(|b| b.id == block_id)
                                    .map(|b| b.float_params.is_empty() && b.string_params.is_empty())
                                    .unwrap_or(true);
                                if has_body {
                                    let chevron = if is_expanded { "▼" } else { "▶" };
                                    if ui.add(egui::Button::new(
                                        egui::RichText::new(chevron)
                                            .color(egui::Color32::from_gray(120))
                                            .size(10.0)
                                    ).frame(false)).clicked() {
                                        toggle_expand = Some(block_id);
                                    }
                                }
                            });
                        });
                    });

                // Click anywhere on the row also toggles expand
                if row_resp.response.clicked() && !matches!(toggle_expand, Some(id) if id == block_id) {
                    toggle_expand = Some(block_id);
                }

                // ── Expanded body: params ─────────────────────────────────────
                if is_expanded {
                    egui::Frame::none()
                        .fill(egui::Color32::from_gray(25))
                        .inner_margin(egui::Margin { left: 16.0, right: 8.0, top: 4.0, bottom: 6.0 })
                        .show(ui, |ui| {
                            ui.set_min_width(ui.available_width());
                            let block = notebook.blocks.iter_mut()
                                .find(|b| b.id == block_id).unwrap();

                            // Float params
                            for param in &mut block.float_params {
                                let speed = drag_speed(&param.label);
                                ui.horizontal(|ui| {
                                    ui.add_sized([72.0, 0.0], egui::Label::new(
                                        egui::RichText::new(&param.label)
                                            .color(egui::Color32::from_gray(150))
                                            .size(11.0)
                                    ));
                                    changed |= ui.add(
                                        egui::DragValue::new(&mut param.value).speed(speed)
                                    ).changed();
                                });
                            }

                            // String params
                            for param in &mut block.string_params {
                                ui.horizontal(|ui| {
                                    ui.add_sized([72.0, 0.0], egui::Label::new(
                                        egui::RichText::new(&param.label)
                                            .color(egui::Color32::from_gray(150))
                                            .size(11.0)
                                    ));
                                    changed |= ui.add(
                                        egui::TextEdit::singleline(&mut param.value)
                                            .desired_width(100.0)
                                    ).changed();
                                });
                            }
                        });
                }

                // ── Input connections (always visible for 2-input blocks) ──────
                if inputs == 2 {
                    let (name_a, name_b) = input_names(kind);
                    egui::Frame::none()
                        .fill(egui::Color32::from_gray(22))
                        .inner_margin(egui::Margin { left: 24.0, right: 8.0, top: 2.0, bottom: 3.0 })
                        .show(ui, |ui| {
                            ui.set_min_width(ui.available_width());
                            let block = notebook.blocks.iter_mut()
                                .find(|b| b.id == block_id).unwrap();
                            changed |= show_input_row(ui, name_a, &prev_entries, &mut block.input_a, block_id, "a");
                            changed |= show_input_row(ui, name_b, &prev_entries, &mut block.input_b, block_id, "b");
                        });
                } else if inputs == 1 && is_expanded {
                    // For single-input blocks, show the connection when expanded
                    egui::Frame::none()
                        .fill(egui::Color32::from_gray(22))
                        .inner_margin(egui::Margin { left: 24.0, right: 8.0, top: 2.0, bottom: 3.0 })
                        .show(ui, |ui| {
                            ui.set_min_width(ui.available_width());
                            let block = notebook.blocks.iter_mut()
                                .find(|b| b.id == block_id).unwrap();
                            changed |= show_input_row(ui, "Input", &prev_entries, &mut block.input_a, block_id, "a");
                        });
                }

                ui.add_space(1.0);
            }
        });

    // ── Apply deferred mutations ─────────────────────────────────────────────
    if let Some(pid_opt) = new_preview {
        notebook.preview_id = pid_opt;
        changed = true;
    }
    if let Some(id) = to_delete {
        if expanded_id == Some(id) { expanded_id = None; }
        notebook.remove_block(id);
        changed = true;
    }
    if let Some(id) = toggle_expand {
        expanded_id = if expanded_id == Some(id) { None } else { Some(id) };
    }

    ui.ctx().data_mut(|d| d.insert_temp(state_id, expanded_id));
    changed
}

/// Compact inline input-reference row: "Body  ← Box #2 ▾"
fn show_input_row(
    ui: &mut egui::Ui,
    slot_name: &str,
    prev_entries: &[(u64, String)],
    input_ref: &mut NbInputRef,
    block_id: u64,
    slot_tag: &str,
) -> bool {
    let mut changed = false;

    let selected_text = match input_ref {
        NbInputRef::Previous => prev_entries
            .last()
            .map(|(_, l)| format!("← {}", l))
            .unwrap_or_else(|| "← (none)".to_string()),
        NbInputRef::BlockId(id) => prev_entries
            .iter()
            .find(|(bid, _)| bid == id)
            .map(|(_, l)| format!("← {}", l))
            .unwrap_or_else(|| "← ⚠ missing".to_string()),
    };

    ui.horizontal(|ui| {
        // Connector glyph
        ui.label(
            egui::RichText::new("├─")
                .color(egui::Color32::from_gray(60))
                .size(11.0)
        );
        // Slot name
        ui.add_sized([36.0, 0.0], egui::Label::new(
            egui::RichText::new(slot_name)
                .color(egui::Color32::from_gray(130))
                .size(11.0)
        ));
        // Dropdown
        egui::ComboBox::from_id_salt(egui::Id::new(("nb_in", block_id, slot_tag)))
            .width(130.0)
            .selected_text(&selected_text)
            .show_ui(ui, |ui| {
                if let Some((_, last_lbl)) = prev_entries.last() {
                    let is_sel = *input_ref == NbInputRef::Previous;
                    if ui.selectable_label(is_sel, format!("← {}", last_lbl)).clicked() && !is_sel {
                        *input_ref = NbInputRef::Previous;
                        changed = true;
                    }
                }
                for (bid, blbl) in prev_entries.iter().rev() {
                    let is_sel = *input_ref == NbInputRef::BlockId(*bid);
                    if ui.selectable_label(is_sel, format!("← {}", blbl)).clicked() && !is_sel {
                        *input_ref = NbInputRef::BlockId(*bid);
                        changed = true;
                    }
                }
            });
    });

    changed
}

/// Drag speed hint per parameter label.
fn drag_speed(label: &str) -> f64 {
    match label {
        "nose" | "tail" | "inlet" | "exhaust" => 0.005,
        "sx" | "sy" | "sz" => 0.01,
        "thickness" | "k" | "amount" | "minor_r" => 0.05,
        "count" => 0.1,
        _ => 0.5,
    }
}
