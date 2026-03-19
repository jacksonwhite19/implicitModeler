// In-app function reference panel.

use eframe::egui::{self, Context, RichText, ScrollArea, TextEdit, Frame, Color32, Stroke};
use crate::ui::help_data::{FUNCTION_DOCS, CATEGORIES, FunctionStatus, function_status};
use crate::ui::help_search::HelpSearchState;

/// Map category name to a display color.
pub fn category_color(category: &str) -> Color32 {
    match category {
        "Primitives"            => Color32::from_rgb(100, 149, 237),
        "Booleans"              => Color32::from_rgb(255, 165,   0),
        "Transforms"            => Color32::from_rgb( 50, 205,  50),
        "Assembly"              => Color32::from_rgb(135, 206, 235),
        "Math"                  => Color32::from_rgb(200, 200, 200),
        "Wing and Airfoil"      => Color32::from_rgb(220,  20,  60),
        "Wing Measurements"     => Color32::from_rgb(255, 100, 100),
        "Control Surfaces"      => Color32::from_rgb(255, 140,   0),
        "Nose and Tail"         => Color32::from_rgb(210, 105,  30),
        "Inlets"                => Color32::from_rgb(160,  32, 240),
        "Structural"            => Color32::from_rgb(112, 128, 144),
        "Components"            => Color32::from_rgb(  0, 128, 128),
        "Fasteners"             => Color32::from_rgb(169, 169, 169),
        "Fuselage and Sections" => Color32::from_rgb(205,  92,  92),
        "Fields"                => Color32::from_rgb( 64, 224, 208),
        "Lattice"               => Color32::from_rgb( 32, 178, 170),
        "Splines and Sweeps"    => Color32::from_rgb( 72, 209, 204),
        "FEA and Loads"         => Color32::from_rgb(255, 215,   0),
        "Mesh Import"           => Color32::from_rgb(152, 251, 152),
        "Composite Layup"       => Color32::from_rgb(147, 112, 219),
        "Print and Split"       => Color32::from_rgb(176, 196, 222),
        "Joints and Panels"     => Color32::from_rgb(188, 143, 143),
        "Aerodynamic Analysis"  => Color32::from_rgb(100, 149, 237),
        "Geometry Analysis"     => Color32::from_rgb( 70, 130, 180),
        "Points and Queries"    => Color32::from_rgb(144, 238, 144),
        "Placement"             => Color32::from_rgb(240, 230, 140),
        "Instancing"            => Color32::from_rgb(255, 228, 196),
        "Propulsion"            => Color32::from_rgb(255, 215,   0),
        _                       => Color32::from_rgb(150, 150, 150),
    }
}

fn status_color(status: FunctionStatus) -> Color32 {
    match status {
        FunctionStatus::Stable => Color32::from_rgb(70, 160, 90),
        FunctionStatus::Experimental => Color32::from_rgb(210, 150, 60),
        FunctionStatus::Legacy => Color32::from_rgb(170, 95, 95),
    }
}

pub fn show_help_panel(
    ctx: &Context,
    open: &mut bool,
    state: &mut HelpSearchState,
    script_text: &mut String,
    cursor_byte: Option<usize>,
    _search_focused: bool,
) {
    if !*open { return; }

    let mut close = false;

    egui::Window::new("Function Reference")
        .open(open)
        .resizable(true)
        .default_size([480.0, 700.0])
        .show(ctx, |ui| {
            // ── Search bar ──────────────────────────────────────────────────
            ui.horizontal(|ui| {
                let te = TextEdit::singleline(&mut state.query)
                    .hint_text("Search functions…")
                    .desired_width(ui.available_width() - 8.0);
                let resp = ui.add(te);
                if resp.changed() {
                    state.update_results();
                    state.show_recent = false;
                }
                if resp.gained_focus() && state.query.is_empty() {
                    state.show_recent = !state.recent_searches.is_empty();
                }
            });

            // Recent searches dropdown
            if state.show_recent && !state.recent_searches.is_empty() {
                Frame::none()
                    .fill(ui.visuals().extreme_bg_color)
                    .show(ui, |ui| {
                        ui.set_max_height(120.0);
                        ScrollArea::vertical().show(ui, |ui| {
                            let recents = state.recent_searches.clone();
                            for (i, recent) in recents.iter().enumerate() {
                                ui.horizontal(|ui| {
                                    if ui.small_button(recent).clicked() {
                                        state.query = recent.clone();
                                        state.update_results();
                                        state.show_recent = false;
                                    }
                                    if ui.small_button("x").clicked() {
                                        state.recent_searches.remove(i);
                                    }
                                });
                            }
                            if ui.small_button("Clear history").clicked() {
                                state.recent_searches.clear();
                                state.show_recent = false;
                            }
                        });
                    });
            }

            // Keyboard navigation
            let escape_pressed = ui.input(|i| i.key_pressed(egui::Key::Escape));
            let up_pressed     = ui.input(|i| i.key_pressed(egui::Key::ArrowUp));
            let down_pressed   = ui.input(|i| i.key_pressed(egui::Key::ArrowDown));

            if up_pressed   { state.move_selection(-1); }
            if down_pressed { state.move_selection(1); }
            if escape_pressed {
                if !state.query.is_empty() {
                    state.query.clear();
                    state.update_results();
                } else {
                    close = true;
                }
            }

            // ── Category chips ───────────────────────────────────────────────
            ui.add_space(4.0);
            egui::ScrollArea::horizontal().id_salt("cat_chips").show(ui, |ui| {
                ui.horizontal(|ui| {
                    let all_active = state.selected_category.is_none();
                    let all_btn = if all_active {
                        ui.add(egui::Button::new(RichText::new("All").color(Color32::WHITE))
                            .fill(Color32::from_rgb(80, 80, 200)))
                    } else {
                        ui.button("All")
                    };
                    if all_btn.clicked() {
                        state.selected_category = None;
                        state.update_results();
                    }

                    for &cat in CATEGORIES {
                        let active = state.selected_category.as_deref() == Some(cat);
                        let col = category_color(cat);
                        let btn = if active {
                            ui.add(egui::Button::new(RichText::new(cat).color(Color32::WHITE).small())
                                .fill(col))
                        } else {
                            ui.add(egui::Button::new(RichText::new(cat).small()))
                        };
                        if btn.clicked() {
                            state.selected_category = if active { None } else { Some(cat.to_string()) };
                            state.update_results();
                        }
                    }
                });
            });

            // ── Results count ────────────────────────────────────────────────
            ui.add_space(2.0);
            ui.label(RichText::new(format!(
                "Showing {} of {} functions",
                state.results.len(),
                FUNCTION_DOCS.len()
            ))
            .small()
            .color(Color32::GRAY));
            ui.separator();

            // ── Empty state ──────────────────────────────────────────────────
            if state.results.is_empty() {
                ui.add_space(20.0);
                ui.centered_and_justified(|ui| {
                    if state.query.is_empty() {
                        ui.label("No functions available.");
                    } else {
                        ui.label(format!("No functions match \"{}\"", state.query));
                    }
                });
                return;
            }

            // ── Results list ─────────────────────────────────────────────────
            let scroll_to = state.scroll_to.take();
            ScrollArea::vertical()
                .id_salt("help_results")
                .auto_shrink([false, false])
                .show(ui, |ui| {
                    let results = state.results.clone();
                    for (list_idx, &doc_idx) in results.iter().enumerate() {
                        let doc = &FUNCTION_DOCS[doc_idx];
                        let status = function_status(doc);
                        let is_expanded = state.selected_function == Some(doc_idx);
                        let col = category_color(doc.category);
                        let status_col = status_color(status);

                        let row_resp = Frame::none()
                            .stroke(if is_expanded {
                                Stroke::new(1.0, col)
                            } else {
                                Stroke::NONE
                            })
                            .inner_margin(6.0)
                            .show(ui, |ui| {
                                // ── Collapsed header ────────────────────────
                                ui.horizontal(|ui| {
                                    ui.label(RichText::new(doc.name)
                                        .monospace()
                                        .strong()
                                        .color(col));
                                    ui.label(
                                        RichText::new(format!("[{}]", status.label()))
                                            .small()
                                            .color(status_col),
                                    );
                                    ui.with_layout(
                                        egui::Layout::right_to_left(egui::Align::Center),
                                        |ui| {
                                            ui.add(
                                                egui::Button::new(
                                                    RichText::new(doc.category).small().color(col),
                                                )
                                                .frame(false),
                                            );
                                        },
                                    );
                                });
                                ui.label(
                                    RichText::new(doc.signature)
                                        .monospace()
                                        .small()
                                        .color(Color32::GRAY),
                                );
                                ui.label(RichText::new(doc.description).small());

                                // ── Expanded detail ──────────────────────────
                                if is_expanded {
                                    ui.add_space(4.0);

                                    // Signature block
                                    Frame::none()
                                        .fill(Color32::from_rgb(30, 30, 40))
                                        .inner_margin(6.0)
                                        .show(ui, |ui| {
                                            ui.label(
                                                RichText::new(doc.signature)
                                                    .monospace()
                                                    .color(Color32::LIGHT_GRAY),
                                            );
                                        });

                                    ui.add_space(4.0);
                                    ui.horizontal(|ui| {
                                        ui.label(RichText::new("Returns:").strong().small());
                                        ui.label(
                                            RichText::new(doc.returns).monospace().small(),
                                        );
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label(RichText::new("Status:").strong().small());
                                        ui.label(
                                            RichText::new(status.label()).small().color(status_col),
                                        );
                                    });

                                    // Example block
                                    ui.add_space(4.0);
                                    ui.label(RichText::new("Example:").strong().small());
                                    Frame::none()
                                        .fill(Color32::from_rgb(25, 35, 25))
                                        .inner_margin(6.0)
                                        .show(ui, |ui| {
                                            ui.label(
                                                RichText::new(doc.example)
                                                    .monospace()
                                                    .small()
                                                    .color(Color32::from_rgb(144, 238, 144)),
                                            );
                                        });

                                    // Notes
                                    if let Some(notes) = doc.notes {
                                        ui.add_space(4.0);
                                        Frame::none()
                                            .fill(Color32::from_rgb(60, 55, 20))
                                            .inner_margin(6.0)
                                            .show(ui, |ui| {
                                                ui.horizontal(|ui| {
                                                    ui.label("i");
                                                    ui.label(RichText::new(notes).small());
                                                });
                                            });
                                    }
                                    if let Some(status_note) = status.note() {
                                        ui.add_space(4.0);
                                        Frame::none()
                                            .fill(Color32::from_rgb(45, 40, 28))
                                            .inner_margin(6.0)
                                            .show(ui, |ui| {
                                                ui.label(
                                                    RichText::new(status_note)
                                                        .small()
                                                        .color(status_col),
                                                );
                                            });
                                    }

                                    // Tags
                                    ui.add_space(4.0);
                                    ui.horizontal_wrapped(|ui| {
                                        for &tag in doc.tags {
                                            ui.label(
                                                RichText::new(tag)
                                                    .small()
                                                    .color(Color32::GRAY),
                                            );
                                        }
                                    });

                                    // Action buttons
                                    ui.add_space(4.0);
                                    ui.horizontal(|ui| {
                                        if ui.button("Copy Example").clicked() {
                                            ctx.copy_text(doc.example.to_string());
                                        }
                                        if ui.button("Insert at Cursor").clicked() {
                                            let insert_pos =
                                                cursor_byte.unwrap_or(script_text.len());
                                            let safe_pos = insert_pos.min(script_text.len());
                                            script_text.insert_str(safe_pos, doc.example);
                                        }
                                    });
                                }
                            });

                        // Scroll into view
                        if scroll_to == Some(list_idx) {
                            row_resp.response.scroll_to_me(None);
                        }

                        // Click to expand/collapse
                        if row_resp
                            .response
                            .interact(egui::Sense::click())
                            .clicked()
                        {
                            if is_expanded {
                                state.selected_function = None;
                            } else {
                                if !state.query.is_empty() {
                                    state.push_recent(&state.query.clone());
                                }
                                state.selected_function = Some(doc_idx);
                                state.scroll_to = Some(list_idx);
                            }
                        }

                        ui.separator();
                    }
                });
        });

    if close {
        *open = false;
    }
}
