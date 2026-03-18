// Library panel — shows all project library components with search, thumbnails, and insert.

use eframe::egui::{self, Color32, RichText, TextureHandle, ColorImage, TextureOptions};
use crate::library::{LibraryManager, ThumbnailState};
use crate::library::thumbnail::{THUMB_W, THUMB_H};

pub struct LibraryPanelState {
    pub open:            bool,
    pub search:          String,
    pub selected:        Option<usize>,
    /// Cached egui textures, indexed parallel to manager.components.
    pub textures:        Vec<Option<TextureHandle>>,
    /// Which component file is being edited (Some = editing that file, None = main script).
    pub editing_library: Option<std::path::PathBuf>,
}

impl Default for LibraryPanelState {
    fn default() -> Self {
        Self {
            open: false,
            search: String::new(),
            selected: None,
            textures: Vec::new(),
            editing_library: None,
        }
    }
}

impl LibraryPanelState {
    /// Ensure texture cache has the right length and upload any ready thumbnails.
    fn sync_textures(&mut self, ctx: &egui::Context, manager: &mut LibraryManager) {
        // Grow/shrink texture vec to match components
        self.textures.resize_with(manager.components.len(), || None);

        for (i, comp) in manager.components.iter_mut().enumerate() {
            // Poll background generation
            comp.poll_thumbnail();

            if comp.thumbnail_state == ThumbnailState::Ready {
                if let Some(pixels) = comp.thumbnail_pixels.take() {
                    let image = ColorImage::from_rgba_unmultiplied([THUMB_W, THUMB_H], &pixels);
                    let tex   = ctx.load_texture(
                        format!("lib_thumb_{}", comp.module_name()),
                        image,
                        TextureOptions::default(),
                    );
                    self.textures[i] = Some(tex);
                    // Mark as done — pixels consumed, texture uploaded
                    comp.thumbnail_state = ThumbnailState::Failed; // reuse Failed as "uploaded"
                }
            }
        }
    }
}

/// Draw the library panel in a collapsible section of the left panel.
/// Returns `Some(insert_text)` if the user clicked Insert on a component,
/// or `Some("__NEW_COMPONENT__")` if the user clicked New Component.
pub fn show_library_panel(
    ui:      &mut egui::Ui,
    ctx:     &egui::Context,
    state:   &mut LibraryPanelState,
    manager: &mut LibraryManager,
) -> Option<String> {
    let mut insert_text: Option<String> = None;

    state.sync_textures(ctx, manager);

    // Kick off thumbnail generation for visible components that haven't been generated yet
    if state.open {
        for comp in manager.components.iter_mut() {
            if comp.thumbnail_state == ThumbnailState::NotGenerated {
                let source = comp.source.clone();
                let preview_call = comp.metadata.preview_fn.clone()
                    .or_else(|| comp.exported_functions.first().map(|sig| {
                        let args = sig.params.iter().map(|_| "0.0").collect::<Vec<_>>().join(", ");
                        format!("{}({})", sig.name, args)
                    }));

                if let Some(pfn) = preview_call {
                    let (tx, rx) = std::sync::mpsc::channel();
                    comp.thumbnail_state    = ThumbnailState::Generating;
                    comp.thumbnail_receiver = Some(rx);

                    rayon::spawn(move || {
                        // Evaluate component source directly (functions defined inline)
                        // then call the preview function
                        let full_script = format!("{}\n{}", source, pfn);
                        let result = match crate::scripting::evaluate_script(&full_script) {
                            Ok(r) => {
                                let pixels = crate::library::thumbnail::render_thumbnail(r.sdf);
                                Ok(pixels)
                            }
                            Err(e) => Err(e),
                        };
                        let _ = tx.send(result);
                    });
                } else {
                    comp.thumbnail_state = ThumbnailState::Failed;
                }
            }
        }
    }

    let header = ui.collapsing("Library", |ui| {
        state.open = true;

        // Search bar
        ui.horizontal(|ui| {
            ui.label("Search:");
            ui.add(egui::TextEdit::singleline(&mut state.search)
                .hint_text("Search components...")
                .desired_width(f32::INFINITY));
        });

        // New component button
        if ui.button("+ New Component").clicked() {
            insert_text = Some("__NEW_COMPONENT__".to_string());
        }

        ui.separator();

        let search_lower = state.search.to_lowercase();
        let filtered: Vec<usize> = manager.components.iter().enumerate()
            .filter(|(_, c)| {
                if search_lower.is_empty() { return true; }
                let name_match = c.name.to_lowercase().contains(&search_lower);
                let desc_match = c.metadata.description.as_deref()
                    .map(|d| d.to_lowercase().contains(&search_lower))
                    .unwrap_or(false);
                let tag_match  = c.metadata.tags.iter()
                    .any(|t| t.to_lowercase().contains(&search_lower));
                name_match || desc_match || tag_match
            })
            .map(|(i, _)| i)
            .collect();

        if filtered.is_empty() {
            ui.label(RichText::new("No components found").color(Color32::GRAY).italics());
            return;
        }

        for &idx in &filtered {
            let comp     = &manager.components[idx];
            let tex      = state.textures.get(idx).and_then(|t| t.as_ref());
            let selected = state.selected == Some(idx);

            let frame = egui::Frame::none()
                .inner_margin(egui::Margin::same(6.0))
                .stroke(egui::Stroke::new(
                    if selected { 1.5 } else { 0.5 },
                    if selected { Color32::from_rgb(80, 140, 220) } else { Color32::from_gray(60) },
                ))
                .rounding(4.0);

            let clicked = frame.show(ui, |ui| {
                ui.horizontal(|ui| {
                    // Thumbnail or placeholder
                    let thumb_size = egui::Vec2::splat(48.0);
                    if let Some(tex) = tex {
                        ui.image((tex.id(), thumb_size));
                    } else {
                        match comp.thumbnail_state {
                            ThumbnailState::Generating => {
                                let spinner = egui::Spinner::new();
                                ui.add_sized(thumb_size, spinner);
                            }
                            ThumbnailState::Failed | ThumbnailState::NotGenerated => {
                                let (rect, _) = ui.allocate_exact_size(thumb_size, egui::Sense::hover());
                                ui.painter().rect_filled(rect, 4.0, Color32::from_gray(50));
                                ui.painter().text(
                                    rect.center(), egui::Align2::CENTER_CENTER,
                                    "[ ]", egui::FontId::proportional(12.0), Color32::from_gray(120),
                                );
                            }
                            ThumbnailState::Ready => {}
                        }
                    }

                    ui.vertical(|ui| {
                        ui.label(RichText::new(&comp.name).strong());
                        if let Some(desc) = &comp.metadata.description {
                            if !desc.is_empty() {
                                ui.label(RichText::new(desc).small().color(Color32::GRAY));
                            }
                        }
                        // Tags
                        if !comp.metadata.tags.is_empty() {
                            ui.horizontal_wrapped(|ui| {
                                for tag in &comp.metadata.tags {
                                    ui.label(
                                        RichText::new(tag)
                                            .small()
                                            .background_color(Color32::from_rgb(40, 60, 90))
                                            .color(Color32::from_rgb(140, 180, 220))
                                    );
                                }
                            });
                        }
                        // Error indicator
                        if comp.has_error {
                            ui.label(RichText::new("! Error").small().color(Color32::RED));
                        }
                    });
                });
            }).response.interact(egui::Sense::click()).clicked();

            if clicked {
                state.selected = if selected { None } else { Some(idx) };
            }

            // Expanded detail view
            if selected {
                let comp = &manager.components[idx];
                ui.indent("lib_detail", |ui| {
                    if let Some(desc) = &comp.metadata.description {
                        if !desc.is_empty() {
                            ui.label(desc.as_str());
                        }
                    }

                    ui.label(RichText::new("Functions:").small().color(Color32::GRAY));
                    for sig in &comp.exported_functions {
                        let sig_str = format!("{}({})", sig.name, sig.params.join(", "));
                        ui.horizontal(|ui| {
                            ui.label(RichText::new(&sig_str).code().small());
                            if let Some(desc) = &sig.description {
                                ui.label(RichText::new(format!("-- {}", desc)).small().color(Color32::GRAY));
                            }
                        });
                    }

                    ui.horizontal(|ui| {
                        // Insert button
                        if ui.button("Insert").clicked() {
                            let mod_name = comp.module_name();
                            let text = if let Some(first_fn) = comp.exported_functions.first() {
                                let args = first_fn.params.iter().map(|_| "0.0").collect::<Vec<_>>().join(", ");
                                let binding_name = mod_name.replace('-', "_");
                                format!("let my_{} = {}::{}({});\n", binding_name, mod_name, first_fn.name, args)
                            } else {
                                format!("// {}: no exported functions\n", mod_name)
                            };
                            insert_text = Some(text);
                        }

                        // Edit button
                        if ui.button("Edit").clicked() {
                            state.editing_library = Some(comp.file_path.clone());
                        }
                    });
                });
                ui.add_space(4.0);
            }
        }
    });

    // Track open state from header click
    if header.header_response.clicked() {
        // egui collapsing manages its own open state; we just sync ours
    }
    if !header.openness.eq(&1.0) && header.openness.eq(&0.0) {
        state.open = false;
    }

    insert_text
}

/// Build the list of referenced library components from the script text.
/// Returns (module_name, Vec<line_numbers>) pairs.
#[allow(dead_code)] // Available for library reference highlighting
pub fn find_library_references(script: &str, manager: &LibraryManager) -> Vec<(String, Vec<usize>)> {
    let mut refs = Vec::new();
    for comp in &manager.components {
        let mod_name = comp.module_name();
        let prefix   = format!("{}::", mod_name);
        if script.contains(&prefix) {
            let lines: Vec<usize> = script.lines().enumerate()
                .filter(|(_, line)| line.contains(&prefix))
                .map(|(i, _)| i)
                .collect();
            refs.push((mod_name, lines));
        }
    }
    refs
}
