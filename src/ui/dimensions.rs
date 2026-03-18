// Named dimensions panel.
//
// Shows a table of (name, value) pairs below the project tree.
// Dimension names are injected into Rhai as global constants before each eval.
// Editing any value sets `pending_eval = true` so the script re-runs next frame.
//
// Phase 31: also shows detected script variables as live-manipulation sliders.

use eframe::egui::{self, Color32, RichText, Ui};
use std::collections::HashMap;

use crate::undo::{AppState, DimensionEditCommand, DimensionDeleteCommand, UndoHistory,
                  ScriptTextCommand};
use crate::ui::script_variable_detector::{
    detect_script_variables, rewrite_literal, extract_literal_at,
    infer_unit, DetectedVariable, DetectionType,
};
use crate::scripting;

// ── Per-row editing state ─────────────────────────────────────────────────────

#[derive(Default)]
pub struct DimensionsState {
    /// Which row is being name-edited (index).
    pub editing_name:  Option<usize>,
    /// Buffer for the name being typed.
    pub name_buf:      String,

    /// Which row is in slider mode (index).  `None` = text edit.
    pub slider_row:    Option<usize>,
    /// (min, max) for the active slider row.
    pub slider_range:  (f64, f64),

    /// Pending value before a slider drag started (for undo on release).
    pub drag_start:    Option<f64>,

    /// Whether the "set range" popup is open for a slider row.
    pub range_popup:   Option<usize>,
    pub range_min_buf: String,
    pub range_max_buf: String,

    // Phase 31 — script variable live-manipulation sliders
    /// Cached detected variables; refreshed each time script re-runs.
    pub detected_variables:     Vec<DetectedVariable>,
    /// Slider ranges keyed by variable name or "line:col" for inline literals.
    pub variable_slider_ranges: HashMap<String, (f64, f64)>,
    /// Original value at drag start for undo coalescing: (key, original_value, script_before).
    pub variable_drag_origin:   Option<(String, f64, String)>,
}

// ── Main panel function ───────────────────────────────────────────────────────

/// Draw the dimensions panel.  Returns `true` if any dimension changed and
/// a script re-evaluation should be triggered on the next frame.
///
/// Also draws the Phase 31 "Script Variables" section for live literal manipulation.
/// When a script variable slider moves it directly rewrites `state.script_text` and
/// returns `true` so the caller triggers re-eval.
pub fn show_dimensions_panel(
    ui:      &mut Ui,
    state:   &mut AppState,
    ds:      &mut DimensionsState,
    history: &mut UndoHistory,
) -> bool {
    let mut changed = false;

    ui.add_space(4.0);
    ui.separator();
    ui.add_space(2.0);

    ui.horizontal(|ui| {
        ui.label(RichText::new("Dimensions").strong().size(12.0));
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui.small_button("+").clicked() {
                // Find a unique default name.
                let mut idx = state.dimensions.len() + 1;
                let name = loop {
                    let n = format!("dim_{}", idx);
                    if !state.dimensions.contains_key(&n) { break n; }
                    idx += 1;
                };
                let insert_idx = state.dimensions.len();
                history.execute(Box::new(DimensionEditCommand {
                    before_name:  name.clone(),
                    before_value: 0.0,
                    after_name:   name,
                    after_value:  0.0,
                    index:        insert_idx,
                }), state);
                changed = true;
            }
        });
    });

    // ── Named dimensions grid ─────────────────────────────────────────────────
    // (Script Variables section always follows regardless of whether named dims exist)
    let keys: Vec<String> = state.dimensions.keys().cloned().collect();
    if keys.is_empty() {
        ui.label(RichText::new("No dimensions — press + to add").color(Color32::GRAY).size(11.0));
    }

    let mut delete_index: Option<usize> = None;

    if !keys.is_empty() { egui::Grid::new("dims_grid")
        .num_columns(3)
        .spacing([4.0, 2.0])
        .striped(true)
        .show(ui, |ui| {
            for (row, name) in keys.iter().enumerate() {
                let Some(&value) = state.dimensions.get(name.as_str()) else { ui.end_row(); continue; };

                // ── Name cell ────────────────────────────────────────────────
                if ds.editing_name == Some(row) {
                    let resp = ui.add(
                        egui::TextEdit::singleline(&mut ds.name_buf)
                            .desired_width(80.0)
                            .font(egui::TextStyle::Monospace),
                    );
                    if resp.lost_focus() || ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                        let new_name = ds.name_buf.trim().to_string();
                        // Validate: non-empty, valid identifier, not a duplicate.
                        let is_valid = is_valid_identifier(&new_name)
                            && (new_name == *name || !state.dimensions.contains_key(&new_name));
                        if is_valid && new_name != *name {
                            history.execute(Box::new(DimensionEditCommand::new_rename(
                                name.clone(), value, new_name, row,
                            )), state);
                            changed = true;
                        }
                        ds.editing_name = None;
                    }
                } else {
                    let label_resp = ui.label(
                        RichText::new(name.as_str())
                            .monospace()
                            .color(Color32::from_rgb(180, 220, 255)),
                    );
                    if label_resp.double_clicked() {
                        ds.editing_name = Some(row);
                        ds.name_buf     = name.clone();
                    }
                    label_resp.on_hover_text("Double-click to rename");
                }

                // ── Value cell ───────────────────────────────────────────────
                if ds.slider_row == Some(row) {
                    let (lo, hi) = ds.slider_range;
                    let mut v = value;
                    let slider = egui::Slider::new(&mut v, lo..=hi)
                        .clamping(egui::SliderClamping::Never)
                        .min_decimals(0)
                        .max_decimals(4);
                    let resp = ui.add(slider);

                    if resp.drag_started() {
                        ds.drag_start = Some(value);
                    }

                    if v != value {
                        if let Some(slot) = state.dimensions.get_mut(name.as_str()) {
                            *slot = v;
                            changed = true;
                        }
                    }

                    if resp.drag_stopped() {
                        if let Some(before) = ds.drag_start.take() {
                            if before != v {
                                history.push_executed(Box::new(DimensionEditCommand::new_value(
                                    name.clone(), before, v, row,
                                )));
                            }
                        }
                    }

                    // Right-click to set range.
                    if resp.secondary_clicked() {
                        ds.range_popup   = Some(row);
                        ds.range_min_buf = format!("{}", lo);
                        ds.range_max_buf = format!("{}", hi);
                    }
                } else {
                    // Text edit for value.
                    let mut buf = format!("{}", value);
                    let resp = ui.add(
                        egui::TextEdit::singleline(&mut buf)
                            .desired_width(80.0)
                            .font(egui::TextStyle::Monospace),
                    );
                    if resp.lost_focus() || ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                        if let Ok(new_val) = buf.trim().parse::<f64>() {
                            if new_val != value {
                                history.execute(Box::new(DimensionEditCommand::new_value(
                                    name.clone(), value, new_val, row,
                                )), state);
                                changed = true;
                            }
                        }
                    }
                    if resp.double_clicked() {
                        ds.slider_row  = Some(row);
                        ds.slider_range = (value - value.abs().max(1.0), value + value.abs().max(1.0));
                    }
                    resp.on_hover_text("Double-click for slider mode");
                }

                // ── Delete button ─────────────────────────────────────────────
                if ui.small_button("×").on_hover_text("Delete dimension").clicked() {
                    delete_index = Some(row);
                }

                ui.end_row();
            }
        }); // end grid

    // Handle delete after grid (can't mutate state inside the closure above).
    if let Some(row) = delete_index {
        if let Some(name) = keys.get(row) {
            if let Some(&value) = state.dimensions.get(name.as_str()) {
                history.execute(Box::new(DimensionDeleteCommand {
                    name:  name.clone(),
                    value,
                    index: row,
                }), state);
                if ds.slider_row == Some(row) { ds.slider_row = None; }
                if ds.editing_name == Some(row) { ds.editing_name = None; }
                changed = true;
            }
        }
    }
    } // end if !keys.is_empty()

    // ── Slider range popup ────────────────────────────────────────────────────
    if let Some(popup_row) = ds.range_popup {
        let popup_id = ui.id().with("dim_range_popup");
        let mut open = true;
        egui::Window::new("Set slider range")
            .id(popup_id)
            .collapsible(false)
            .resizable(false)
            .open(&mut open)
            .show(ui.ctx(), |ui| {
                ui.horizontal(|ui| {
                    ui.label("Min:");
                    ui.add(egui::TextEdit::singleline(&mut ds.range_min_buf).desired_width(60.0));
                });
                ui.horizontal(|ui| {
                    ui.label("Max:");
                    ui.add(egui::TextEdit::singleline(&mut ds.range_max_buf).desired_width(60.0));
                });
                if ui.button("Apply").clicked() {
                    if let (Ok(lo), Ok(hi)) = (
                        ds.range_min_buf.trim().parse::<f64>(),
                        ds.range_max_buf.trim().parse::<f64>(),
                    ) {
                        if lo < hi {
                            ds.slider_range = (lo, hi);
                            ds.slider_row   = Some(popup_row);
                        }
                    }
                    ds.range_popup = None;
                }
            });
        if !open { ds.range_popup = None; }
    }

    // ── Phase 31: Script Variables ────────────────────────────────────────────
    changed |= show_script_variables_section(ui, state, ds, history);

    changed
}

// ── Script variable live-manipulation panel ────────────────────────────────────
//
// Grouped by code section (# === Name === delimiters), collapsible.
// Each variable shows: name label + value text-edit (top row), then a full-width slider.

fn show_script_variables_section(
    ui:      &mut Ui,
    state:   &mut AppState,
    ds:      &mut DimensionsState,
    history: &mut UndoHistory,
) -> bool {
    if ds.detected_variables.is_empty() {
        return false;
    }

    ui.add_space(4.0);
    ui.separator();

    // Parse sections and group variables.
    let cells = scripting::parse_cells(&state.script_text);
    let vars: Vec<DetectedVariable> = ds.detected_variables.clone();
    let mut changed = false;

    for cell in &cells {
        let section_vars: Vec<&DetectedVariable> = vars.iter()
            .filter(|v| v.line >= cell.start_line && v.line <= cell.end_line)
            .collect();

        if section_vars.is_empty() {
            continue;
        }

        // Single cell (no delimiters) → plain header; multi-cell → collapsing per section.
        if cells.len() == 1 {
            ui.label(RichText::new("Script Variables").strong().size(12.0));
            ui.add_space(2.0);
            for var in &section_vars {
                if render_sv_row(ui, state, ds, history, var) {
                    changed = true;
                }
                ui.add_space(2.0);
            }
        } else {
            egui::CollapsingHeader::new(
                RichText::new(format!("⚡ {}", cell.name)).strong().size(11.0),
            )
            .default_open(true)
            .id_salt(format!("sv_sec_{}", cell.id))
            .show(ui, |ui| {
                for var in section_vars {
                    if render_sv_row(ui, state, ds, history, var) {
                        changed = true;
                    }
                    ui.add_space(2.0);
                }
            });
        }
    }

    changed
}

/// Convert a snake_case identifier to a human-readable Title Case label.
/// Strips pure unit abbreviation suffixes (_deg, _mm, _ms, _mah, _n, _g)
/// since those are already shown as the unit annotation.
fn prettify_name(name: &str) -> String {
    // Only strip abbreviation suffixes, not semantic words like _chord, _length, etc.
    const UNIT_SUFFIXES: &[&str] = &["_deg", "_mm", "_ms", "_mah", "_n", "_g"];
    let lower = name.to_lowercase();
    let base = UNIT_SUFFIXES.iter()
        .find(|&&s| lower.ends_with(s))
        .map(|s| &name[..name.len() - s.len()])
        .unwrap_or(name);

    base.split('_')
        .filter(|s| !s.is_empty())
        .map(|word| {
            let mut chars = word.chars();
            match chars.next() {
                None => String::new(),
                Some(first) => {
                    let upper: String = first.to_uppercase().collect();
                    upper + chars.as_str()
                }
            }
        })
        .collect::<Vec<_>>()
        .join(" ")
}

/// Count decimal places in a numeric literal string.
fn count_decimal_places_str(lit: &str) -> usize {
    if let Some(dot_pos) = lit.find('.') {
        let after_dot = &lit[dot_pos + 1..];
        after_dot.chars().take_while(|c| c.is_ascii_digit()).count()
    } else {
        0
    }
}

/// Render one variable row: [name label | value text-edit] then [slider].
fn render_sv_row(
    ui:      &mut Ui,
    state:   &mut AppState,
    ds:      &mut DimensionsState,
    history: &mut UndoHistory,
    var:     &DetectedVariable,
) -> bool {
    let mut changed = false;
    let key = make_var_key(var);

    // Auto-range.
    let (min, max) = *ds.variable_slider_ranges.entry(key.clone()).or_insert_with(|| {
        if var.value > 0.0      { (var.value * 0.1, var.value * 3.0) }
        else if var.value < 0.0 { (var.value * 3.0, var.value * 0.1) }
        else                    { (-100.0, 100.0) }
    });

    // Display label.
    let unit = infer_unit(&var.name);
    let display_name = match &var.detection_type {
        DetectionType::LetBinding { variable_name } => {
            let pretty = prettify_name(variable_name);
            if unit.is_empty() { pretty }
            else { format!("{} ({})", pretty, unit) }
        }
        DetectionType::InlineLiteral { parent_function } => {
            parent_function.as_deref()
                .map(|f| format!("{} arg", prettify_name(f)))
                .unwrap_or_else(|| format!("Line {} arg", var.line + 1))
        }
    };

    let has_conflict = matches!(&var.detection_type, DetectionType::LetBinding { variable_name }
        if state.dimensions.contains_key(variable_name.as_str()));
    let label_color = if has_conflict { Color32::YELLOW } else { Color32::from_rgb(160, 210, 160) };

    let orig_lit = extract_literal_at(&state.script_text, var.line, var.col_start, var.col_end);
    let decimals = count_decimal_places_str(&orig_lit).max(1);

    // ── Top row: name + value text-edit ──────────────────────────────────────
    let new_val_from_te = ui.horizontal(|ui| -> Option<f64> {
        let lbl = ui.label(RichText::new(&display_name).color(label_color).size(11.0));
        if has_conflict {
            lbl.on_hover_text(format!("'{}' also exists as a Named Dimension", var.name));
        } else {
            lbl.on_hover_text(&var.context);
        }

        // Push the text edit to the right edge.
        let te_width = 68.0_f32;
        let space = ui.available_width() - te_width - 2.0;
        if space > 0.0 { ui.add_space(space); }

        let mut buf = format!("{:.prec$}", var.value, prec = decimals);
        let te = ui.add(
            egui::TextEdit::singleline(&mut buf)
                .desired_width(te_width)
                .font(egui::TextStyle::Monospace),
        );
        if te.lost_focus() {
            if let Ok(v) = buf.trim().parse::<f64>() {
                if (v - var.value).abs() > 1e-12 {
                    return Some(v);
                }
            }
        }
        None
    }).inner;

    if let Some(new_val) = new_val_from_te {
        let new_script = rewrite_literal(
            &state.script_text, var.line, var.col_start, var.col_end, new_val, &orig_lit,
        );
        let before = state.script_text.clone();
        state.script_text = new_script;
        ds.detected_variables = detect_script_variables(&state.script_text);
        let mut cmd = ScriptTextCommand::new(before, state.script_text.clone());
        cmd.desc = format!("Edit '{}'", var.name);
        history.push_executed(Box::new(cmd));
        changed = true;
    }

    // ── Slider row ─────────────────────────────────────────────────────────
    let mut val = var.value;
    let slider_resp = ui.add(
        egui::Slider::new(&mut val, min..=max)
            .clamping(egui::SliderClamping::Never)
            .min_decimals(1)
            .max_decimals(4),
    );

    if slider_resp.drag_started() {
        ds.variable_drag_origin = Some((key.clone(), var.value, state.script_text.clone()));
    }

    if slider_resp.changed() {
        // Rewrite the literal in the editor text so the number stays in sync,
        // but do NOT signal re-eval yet — wait until drag_stopped to avoid
        // triggering an expensive SDF recompute on every mouse-move frame.
        let orig2 = extract_literal_at(&state.script_text, var.line, var.col_start, var.col_end);
        let new_script = rewrite_literal(
            &state.script_text, var.line, var.col_start, var.col_end, val, &orig2,
        );
        state.script_text = new_script;
        ds.detected_variables = detect_script_variables(&state.script_text);
        // changed stays false here — re-eval fires on drag_stopped below.
    }

    if slider_resp.drag_stopped() {
        // Now signal re-eval with the final value.
        changed = true;
        if let Some((_, _, script_before)) = ds.variable_drag_origin.take() {
            let script_after = state.script_text.clone();
            if script_before != script_after {
                let mut cmd = ScriptTextCommand::new(script_before, script_after);
                cmd.desc = format!("Drag '{}'", var.name);
                history.push_executed(Box::new(cmd));
            }
        }
    }

    // ── Right-click context menu ────────────────────────────────────────────
    slider_resp.context_menu(|ui| {
        if let DetectionType::LetBinding { variable_name } = &var.detection_type {
            if ui.button("Promote to Named Dimension").clicked() {
                promote_to_dimension(state, var);
                ds.detected_variables = detect_script_variables(&state.script_text);
                changed = true;
                ui.close_menu();
            }
            let _ = variable_name;
        }
        if matches!(&var.detection_type, DetectionType::InlineLiteral { .. }) {
            if ui.button("Convert to let variable").clicked() {
                convert_to_variable(state, var);
                ds.detected_variables = detect_script_variables(&state.script_text);
                changed = true;
                ui.close_menu();
            }
        }
        ui.menu_button("Set Range...", |ui| {
            let entry = ds.variable_slider_ranges.entry(key.clone()).or_insert((min, max));
            ui.horizontal(|ui| {
                ui.label("Min:");
                let mut min_buf = format!("{}", entry.0);
                if ui.add(egui::TextEdit::singleline(&mut min_buf).desired_width(60.0)).lost_focus() {
                    if let Ok(v) = min_buf.trim().parse::<f64>() { entry.0 = v; }
                }
            });
            ui.horizontal(|ui| {
                ui.label("Max:");
                let mut max_buf = format!("{}", entry.1);
                if ui.add(egui::TextEdit::singleline(&mut max_buf).desired_width(60.0)).lost_focus() {
                    if let Ok(v) = max_buf.trim().parse::<f64>() { entry.1 = v; }
                }
            });
        });
    });

    changed
}

/// Build the HashMap key for a detected variable.
fn make_var_key(var: &DetectedVariable) -> String {
    match &var.detection_type {
        DetectionType::LetBinding { variable_name } => variable_name.clone(),
        DetectionType::InlineLiteral { .. } => format!("{}:{}", var.line, var.col_start),
    }
}

/// Promote a let-binding variable to a named dimension by removing its line from
/// the script and inserting its value into state.dimensions.
fn promote_to_dimension(state: &mut AppState, var: &DetectedVariable) {
    if let DetectionType::LetBinding { variable_name } = &var.detection_type {
        let lines: Vec<&str> = state.script_text.lines().collect();
        let new_script: String = lines
            .iter()
            .enumerate()
            .filter(|(i, _)| *i != var.line)
            .map(|(_, l)| *l)
            .collect::<Vec<_>>()
            .join("\n");
        state.script_text = new_script;
        state.dimensions.insert(variable_name.clone(), var.value);
    }
}

/// Convert an inline literal to a let-binding variable at the top of the script.
fn convert_to_variable(state: &mut AppState, var: &DetectedVariable) {
    let new_var_name = match &var.detection_type {
        DetectionType::InlineLiteral { parent_function } => parent_function
            .as_ref()
            .map(|f| format!("{}_arg", f))
            .unwrap_or_else(|| format!("val_line_{}", var.line + 1)),
        _ => return,
    };

    // Replace the literal at its position with the new variable name.
    let orig_lit = extract_literal_at(&state.script_text, var.line, var.col_start, var.col_end);
    let lines: Vec<&str> = state.script_text.lines().collect();
    if var.line >= lines.len() {
        return;
    }
    let target_line = lines[var.line];
    let char_indices: Vec<(usize, char)> = target_line.char_indices().collect();
    let char_count = char_indices.len();
    let byte_start = if var.col_start < char_count { char_indices[var.col_start].0 } else { target_line.len() };
    let byte_end = if var.col_end < char_count { char_indices[var.col_end].0 } else { target_line.len() };

    let mut new_line = target_line.to_string();
    if byte_start <= byte_end && byte_end <= new_line.len() {
        new_line.replace_range(byte_start..byte_end, &new_var_name);
    }

    let mut result_lines: Vec<String> = lines.iter().map(|l| l.to_string()).collect();
    result_lines[var.line] = new_line;
    let body = result_lines.join("\n");

    // Prepend the let binding.
    let decimal_places = orig_lit.find('.').map(|p| {
        orig_lit[p+1..].chars().take_while(|c| c.is_ascii_digit()).count()
    }).unwrap_or(1).max(1);
    let let_line = format!("let {} = {:.prec$};\n", new_var_name, var.value, prec = decimal_places);
    state.script_text = format!("{}{}", let_line, body);
}

// ── Helpers ───────────────────────────────────────────────────────────────────

fn is_valid_identifier(s: &str) -> bool {
    if s.is_empty() { return false; }
    let mut chars = s.chars();
    let first = chars.next().unwrap();
    if !first.is_alphabetic() && first != '_' { return false; }
    chars.all(|c| c.is_alphanumeric() || c == '_')
}
