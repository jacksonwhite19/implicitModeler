// New Project Wizard UI
#![allow(dead_code)] // Not yet wired into the main UI

use eframe::egui;
use std::collections::HashMap;
use super::templates::{get_templates, instantiate_project, TemplateInstance};

pub struct WizardState {
    pub open:         bool,
    pub step:         usize,    // 0 = template, 1 = params, 2 = location
    pub selected:     usize,    // template index
    pub params:       HashMap<String, f64>,
    pub project_name: String,
    pub project_path: String,
}

impl Default for WizardState {
    fn default() -> Self {
        Self {
            open:         false,
            step:         0,
            selected:     0,
            params:       HashMap::new(),
            project_name: "Untitled".to_string(),
            project_path: String::new(),
        }
    }
}

/// Show the wizard window.
///
/// Returns `Some((script, project_name, project_path))` when the user clicks Create,
/// or `None` otherwise.
pub fn show_wizard(
    ui_ctx: &egui::Context,
    state: &mut WizardState,
) -> Option<(TemplateInstance, String, String)> {
    if !state.open {
        return None;
    }

    let mut result = None;

    egui::Window::new("New Project")
        .collapsible(false)
        .resizable(true)
        .min_width(600.0)
        .show(ui_ctx, |ui| {
            // Step indicator
            ui.horizontal(|ui| {
                for (i, label) in ["1. Template", "2. Parameters", "3. Location"].iter().enumerate() {
                    if i == state.step {
                        ui.strong(*label);
                    } else {
                        ui.label(*label);
                    }
                    if i < 2 {
                        ui.label(" › ");
                    }
                }
            });
            ui.separator();

            match state.step {
                0 => {
                    ui.label("Select a project template:");
                    ui.add_space(8.0);
                    let templates = get_templates();
                    egui::Grid::new("template_grid")
                        .num_columns(3)
                        .spacing([12.0, 12.0])
                        .show(ui, |ui| {
                            for (i, tmpl) in templates.iter().enumerate() {
                                let selected = state.selected == i;
                                let stroke_width = if selected { 2.0 } else { 1.0 };
                                let stroke_color = if selected {
                                    egui::Color32::from_rgb(100, 150, 255)
                                } else {
                                    egui::Color32::GRAY
                                };
                                let frame = egui::Frame::default()
                                    .stroke(egui::Stroke::new(stroke_width, stroke_color));
                                frame.show(ui, |ui| {
                                    ui.set_width(160.0);
                                    ui.vertical(|ui| {
                                        ui.strong(tmpl.name);
                                        ui.label(egui::RichText::new(tmpl.description).small());
                                        ui.horizontal(|ui| {
                                            for tag in tmpl.tags {
                                                ui.label(
                                                    egui::RichText::new(*tag)
                                                        .small()
                                                        .color(egui::Color32::LIGHT_BLUE),
                                                );
                                            }
                                        });
                                    });
                                });
                                if ui.interact(
                                    ui.min_rect(),
                                    egui::Id::new(("tmpl", i)),
                                    egui::Sense::click(),
                                )
                                .clicked()
                                {
                                    state.selected = i;
                                    state.params.clear();
                                    for p in templates[i].params {
                                        state.params.insert(p.name.to_string(), p.default);
                                    }
                                }
                                if (i + 1) % 3 == 0 {
                                    ui.end_row();
                                }
                            }
                        });
                    ui.add_space(8.0);
                    if ui.button("Next →").clicked() {
                        let templates = get_templates();
                        state.params.clear();
                        for p in templates[state.selected].params {
                            state.params.entry(p.name.to_string()).or_insert(p.default);
                        }
                        state.step = 1;
                    }
                }
                1 => {
                    let templates = get_templates();
                    let tmpl = &templates[state.selected];
                    ui.label(format!("Configure '{}' parameters:", tmpl.name));
                    ui.add_space(8.0);
                    if tmpl.params.is_empty() {
                        ui.label("No parameters for this template.");
                    } else {
                        egui::Grid::new("params_grid").num_columns(2).show(ui, |ui| {
                            for param in tmpl.params {
                                ui.label(param.label);
                                let val = state
                                    .params
                                    .entry(param.name.to_string())
                                    .or_insert(param.default);
                                ui.add(
                                    egui::DragValue::new(val)
                                        .range(param.min..=param.max)
                                        .speed(1.0),
                                );
                                ui.end_row();
                            }
                        });
                    }
                    ui.add_space(8.0);
                    ui.horizontal(|ui| {
                        if ui.button("← Back").clicked() {
                            state.step = 0;
                        }
                        if ui.button("Next →").clicked() {
                            state.step = 2;
                        }
                    });
                }
                2 => {
                    ui.label("Project details:");
                    ui.add_space(8.0);
                    egui::Grid::new("location_grid").num_columns(2).show(ui, |ui| {
                        ui.label("Project name:");
                        ui.text_edit_singleline(&mut state.project_name);
                        ui.end_row();
                        ui.label("Save location:");
                        ui.text_edit_singleline(&mut state.project_path);
                        ui.end_row();
                    });
                    ui.add_space(8.0);
                    ui.horizontal(|ui| {
                        if ui.button("← Back").clicked() {
                            state.step = 1;
                        }
                        if ui.button("✓ Create Project").clicked() {
                            let templates = get_templates();
                            let instance = instantiate_project(&templates[state.selected], &state.params);
                            result = Some((
                                instance,
                                state.project_name.clone(),
                                state.project_path.clone(),
                            ));
                            state.open = false;
                        }
                    });
                }
                _ => {}
            }
        });

    result
}
