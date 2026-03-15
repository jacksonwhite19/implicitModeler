// Autocomplete popup and signature tooltip for the script editor.

use eframe::egui::{self, Color32};

// ── Completion catalogue ──────────────────────────────────────────────────────

pub struct CompletionItem {
    pub name:      &'static str,
    pub signature: &'static str,
    pub color:     Color32,
}

macro_rules! item {
    ($name:expr, $sig:expr, $color:expr) => {
        CompletionItem { name: $name, signature: $sig, color: $color }
    };
}

const C_PRIM: Color32 = Color32::from_rgb(80,  200, 220);
const C_BOOL: Color32 = Color32::from_rgb(220, 140,  60);
const C_XFRM: Color32 = Color32::from_rgb(220, 200,  80);
const C_PAT:  Color32 = Color32::from_rgb(120, 210, 120);
const C_AERO: Color32 = Color32::from_rgb(220, 130, 150);
const C_FLD:  Color32 = Color32::from_rgb(100, 160, 220);
const C_COMP: Color32 = Color32::from_rgb(220, 160, 120);
const C_FEA:  Color32 = Color32::from_rgb(220,  80,  80);
const C_MATH: Color32 = Color32::from_rgb(220, 200,  80);

pub static ALL_COMPLETIONS: &[CompletionItem] = &[
    // Primitives
    item!("sphere",          "(radius)",                                C_PRIM),
    item!("box_",            "(width, height, depth)",                  C_PRIM),
    item!("cylinder",        "(radius, height)",                        C_PRIM),
    item!("torus",           "(major_radius, minor_radius)",            C_PRIM),
    item!("cone",            "(radius, height)",                        C_PRIM),
    item!("plane",           "(nx, ny, nz, distance)",                  C_PRIM),
    item!("gyroid",          "(cell_size, thickness)",                  C_PRIM),
    item!("cubic_lattice",   "(cell_size, strut_radius)",               C_PRIM),
    item!("diamond_lattice", "(cell_size, thickness)",                  C_PRIM),
    // Booleans
    item!("union",           "(a, b)",                                  C_BOOL),
    item!("subtract",        "(a, b)",                                  C_BOOL),
    item!("intersect",       "(a, b)",                                  C_BOOL),
    item!("smooth_union",    "(a, b, smoothness)",                      C_BOOL),
    item!("smooth_subtract", "(base, tool, k)",                         C_BOOL),
    item!("smooth_intersect","(a, b, k)",                               C_BOOL),
    item!("blend",           "(a, b, radius)",                          C_BOOL),
    // Transforms
    item!("translate",       "(body, x, y, z)",                        C_XFRM),
    item!("rotate",          "(body, rx_deg, ry_deg, rz_deg)",         C_XFRM),
    item!("scale",           "(body, sx, sy, sz)",                     C_XFRM),
    item!("offset",          "(body, distance)",                       C_XFRM),
    item!("shell",           "(body, thickness)",                      C_XFRM),
    item!("twist",           "(body, ax, ay, az, rate)",               C_XFRM),
    item!("bend",            "(body, ax, ay, az, curvature)",          C_XFRM),
    // Patterns
    item!("linear_array",    "(body, count, dx, dy, dz)",              C_PAT),
    item!("polar_array",     "(body, count)",                          C_PAT),
    item!("polar_array_axis","(body, count, ax, ay, az)",              C_PAT),
    item!("mirror_x",        "(body)",                                  C_PAT),
    item!("mirror_y",        "(body)",                                  C_PAT),
    item!("mirror_z",        "(body)",                                  C_PAT),
    // Aerospace
    item!("naca",            "(digits, chord, span, sweep_deg)",       C_AERO),
    item!("wing_with_airfoil","(airfoil, chord_root, chord_tip, span, sweep_deg)", C_AERO),
    item!("fuselage",        "(length, radius)",                       C_AERO),
    item!("fuselage_parametric","(length, nose_r, mid_r, tail_r, nose_l, tail_l)", C_AERO),
    item!("nacelle",         "(length, inlet_r, max_r, outlet_r)",     C_AERO),
    item!("bulkhead_at_station","(fuselage, station, thickness)",      C_AERO),
    item!("rib_at_station",  "(wing, span_pos, thickness)",            C_AERO),
    item!("spar",            "(wing, chord_pos, radius)",              C_AERO),
    item!("rod_mount",       "(x, y, z, length, radius)",              C_AERO),
    item!("motor_arm",       "(x, y, z, length, radius)",              C_AERO),
    item!("motor_mount",     "(x, y, z, radius, height)",              C_AERO),
    item!("conformal_gyroid","(body, cell_size, thickness)",           C_AERO),
    item!("conformal_diamond","(body, cell_size, thickness)",          C_AERO),
    item!("conformal_schwarz_p","(body, cell_size, thickness)",        C_AERO),
    item!("wing_lattice",    "(wing, cell_size, thickness)",           C_AERO),
    item!("fuselage_lattice","(fuselage, cell_size, thickness)",       C_AERO),
    item!("fuselage_lattice_graded","(fuselage, cell_min, cell_max, thick)", C_AERO),
    item!("circle_section",  "(radius)",                               C_AERO),
    item!("ellipse_section", "(width, height)",                        C_AERO),
    item!("fuselage_station","(position, section)",                    C_AERO),
    item!("lofted_fuselage", "(stations)",                             C_AERO),
    item!("spline",          "(name)",                                 C_AERO),
    item!("spline_section",  "(name)",                                 C_AERO),
    item!("auto_fuselage",   "(internal, skin_thickness)",             C_AERO),
    // Fields
    item!("constant_field",  "(value)",                                C_FLD),
    item!("sdf_as_field",    "(sdf)",                                  C_FLD),
    item!("position_x_field","()",                                     C_FLD),
    item!("position_y_field","()",                                     C_FLD),
    item!("position_z_field","()",                                     C_FLD),
    item!("add_fields",      "(a, b)",                                 C_FLD),
    item!("multiply_fields", "(a, b)",                                 C_FLD),
    item!("min_fields",      "(a, b)",                                 C_FLD),
    item!("max_fields",      "(a, b)",                                 C_FLD),
    item!("abs_field",       "(field)",                                C_FLD),
    item!("gradient_field",  "(sdf, scale)",                           C_FLD),
    item!("radial_field",    "(cx, cy, cz, inner_r, outer_r)",        C_FLD),
    item!("axial_radial_field","(cx, cy, cz, ax, ay, az, inner_r, outer_r)", C_FLD),
    item!("offset_by_field", "(sdf, field)",                          C_FLD),
    item!("shell_with_field","(sdf, field)",                          C_FLD),
    item!("blend_by_field",  "(a, b, field)",                         C_FLD),
    item!("gyroid_with_field","(cell_size, field)",                   C_FLD),
    item!("stress_field",    "()",                                     C_FLD),
    item!("displacement_field","()",                                   C_FLD),
    // Components / mass
    item!("component",       "(sdf, margin)",                         C_COMP),
    item!("component_mass",  "(sdf, margin, mass_g)",                 C_COMP),
    item!("component_named", "(name, sdf, margin, mass_g)",           C_COMP),
    item!("place",           "(comp, x, y, z)",                       C_COMP),
    item!("geometry",        "(comp)",                                 C_COMP),
    item!("keepout",         "(comp)",                                 C_COMP),
    item!("mass_g",          "(comp)",                                 C_COMP),
    item!("mass_at",         "(mass_g, x, y, z)",                     C_COMP),
    item!("mass_named",      "(name, mass_g, x, y, z)",               C_COMP),
    item!("generate_mounts", "(body, positions)",                     C_COMP),
    item!("mount_with_bolts","(body, positions, bolt_r, head_r, head_h)", C_COMP),
    // FEA
    item!("fixed_support",   "(region, name)",                        C_FEA),
    item!("fixed_axis",      "(region, name, ax, ay, az)",            C_FEA),
    item!("force_load",      "(region, name, fx, fy, fz)",            C_FEA),
    item!("pressure_load",   "(region, name, pressure)",              C_FEA),
    item!("gravity_load",    "(gx, gy, gz)",                          C_FEA),
    item!("torque_load",     "(region, name, tx, ty, tz)",            C_FEA),
    item!("motor_thrust",    "(region, name, fx, fy, fz, tx, ty, tz)", C_FEA),
    // Math helpers
    item!("to_rad",          "(degrees)",                             C_MATH),
    item!("to_deg",          "(radians)",                             C_MATH),
    item!("clamp",           "(value, min, max)",                     C_MATH),
    item!("lerp",            "(a, b, t)",                             C_MATH),
];

// ── State ─────────────────────────────────────────────────────────────────────

#[derive(Default)]
pub struct AutocompleteState {
    pub visible:        bool,
    /// Indices into ALL_COMPLETIONS for current matches.
    pub match_indices:  Vec<usize>,
    pub selected_index: usize,
    /// Screen position to show the popup at (just below cursor line).
    pub anchor_pos:     egui::Pos2,
    /// Char index in the script text where the current token starts.
    pub token_start:    usize,
    /// Current token text (prefix being typed).
    pub token:          String,
}

/// Pending action returned by show_autocomplete.
pub enum AutocompleteAction {
    None,
    Confirm(usize),   // ALL_COMPLETIONS index to apply
    Dismiss,
}

// ── Update ────────────────────────────────────────────────────────────────────

/// Call this after the TextEdit renders, passing the current cursor char offset.
/// Updates `state` with fresh completions / visibility.
pub fn update_completions(
    state:      &mut AutocompleteState,
    text:       &str,
    cursor:     usize,
    anchor_pos: egui::Pos2,
) {
    // Extract the word immediately before (and including) cursor.
    let before = &text[..cursor.min(text.len())];
    let token_start = before
        .rfind(|c: char| !c.is_alphanumeric() && c != '_')
        .map(|p| p + before[p..].chars().next().map(|c| c.len_utf8()).unwrap_or(1))
        .unwrap_or(0);
    let token = &before[token_start..];

    // Only trigger for ≥ 2 non-digit-starting characters.
    if token.len() < 2 || token.starts_with(|c: char| c.is_ascii_digit()) {
        state.visible = false;
        return;
    }

    let matches: Vec<usize> = ALL_COMPLETIONS
        .iter()
        .enumerate()
        .filter(|(_, item)| item.name.starts_with(token))
        .map(|(i, _)| i)
        .collect();

    if matches.is_empty() {
        state.visible = false;
        return;
    }

    // Keep selected index stable when match list doesn't change much.
    if state.token != token {
        state.selected_index = 0;
    }
    state.visible        = true;
    state.match_indices  = matches;
    state.selected_index = state.selected_index.min(state.match_indices.len().saturating_sub(1));
    state.anchor_pos     = anchor_pos;
    state.token_start    = token_start;
    state.token          = token.to_owned();
}

// ── Render popup ──────────────────────────────────────────────────────────────

/// Render the autocomplete popup.  Returns the action the caller should take.
pub fn show_autocomplete(
    ctx:   &egui::Context,
    state: &mut AutocompleteState,
) -> AutocompleteAction {
    if !state.visible || state.match_indices.is_empty() {
        return AutocompleteAction::None;
    }

    let max_show   = 8usize;
    let n          = state.match_indices.len().min(max_show);
    let row_height = 18.0f32;
    let popup_w    = 340.0f32;

    let mut action = AutocompleteAction::None;

    egui::Window::new("##autocomplete")
        .title_bar(false)
        .resizable(false)
        .fixed_pos(state.anchor_pos)
        .fixed_size([popup_w, n as f32 * row_height + 4.0])
        .frame(egui::Frame::popup(&ctx.style()))
        .order(egui::Order::Foreground)
        .show(ctx, |ui| {
            ui.spacing_mut().item_spacing.y = 0.0;
            for (list_i, &global_i) in state.match_indices.iter().enumerate().take(max_show) {
                let item  = &ALL_COMPLETIONS[global_i];
                let sel   = list_i == state.selected_index;
                let (rect, resp) = ui.allocate_exact_size(
                    egui::vec2(popup_w - 8.0, row_height),
                    egui::Sense::click(),
                );
                if sel {
                    ui.painter().rect_filled(rect, 2.0, egui::Color32::from_white_alpha(20));
                }
                ui.painter().text(
                    rect.left_center() + egui::vec2(4.0, 0.0),
                    egui::Align2::LEFT_CENTER,
                    item.name,
                    egui::FontId::monospace(13.0),
                    item.color,
                );
                let sig_x = rect.left() + 4.0 + item.name.len() as f32 * 7.8;
                ui.painter().text(
                    egui::pos2(sig_x, rect.center().y),
                    egui::Align2::LEFT_CENTER,
                    item.signature,
                    egui::FontId::monospace(11.0),
                    egui::Color32::from_rgb(140, 140, 140),
                );
                if resp.clicked() {
                    state.selected_index = list_i;
                    action = AutocompleteAction::Confirm(global_i);
                }
            }
            if state.match_indices.len() > max_show {
                ui.label(egui::RichText::new(
                    format!("  … {} more", state.match_indices.len() - max_show)
                ).color(egui::Color32::GRAY).size(11.0));
            }
        });

    action
}

// ── Apply completion ──────────────────────────────────────────────────────────

/// Replace `token_start..cursor` in `text` with the chosen completion name,
/// append `(` if not already there, return new cursor position.
pub fn apply_completion(
    text:        &mut String,
    token_start: usize,
    cursor:      usize,
    item:        &CompletionItem,
) -> usize {
    let name = item.name;
    let end   = cursor.min(text.len());

    // Replace token with full name.
    text.replace_range(token_start..end, name);

    let after_name = token_start + name.len();

    // Insert `(` if the character after is not already `(`.
    let next_char = text[after_name..].chars().next();
    if next_char != Some('(') {
        text.insert(after_name, '(');
        // Place cursor inside the parens.
        after_name + 1
    } else {
        // Already has paren — place cursor inside.
        after_name + 1
    }
}

// ── Signature tooltip ─────────────────────────────────────────────────────────

/// If the cursor is inside a known function call's parentheses, show a small
/// signature tooltip above the cursor.  `anchor_pos` is just above the cursor.
pub fn show_signature_tooltip(
    ctx:        &egui::Context,
    text:       &str,
    cursor:     usize,
    anchor_pos: egui::Pos2,
) {
    let (fn_name, param_idx) = match current_call_site(text, cursor) {
        Some(v) => v,
        None    => return,
    };

    let item = ALL_COMPLETIONS.iter().find(|it| it.name == fn_name);
    let item = match item { Some(it) => it, None => return };

    // Build formatted text: bold current parameter.
    let params_str = item.signature.trim_matches(|c| c == '(' || c == ')');
    let params: Vec<&str> = params_str.split(',').map(str::trim).collect();

    egui::Window::new("##sig_tip")
        .title_bar(false)
        .resizable(false)
        .fixed_pos(anchor_pos)
        .frame(egui::Frame::popup(&ctx.style()))
        .order(egui::Order::Tooltip)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 0.0;
                ui.label(egui::RichText::new(fn_name)
                    .color(item.color)
                    .monospace()
                    .size(13.0));
                ui.label(egui::RichText::new("(").monospace().color(egui::Color32::GRAY).size(13.0));
                for (i, p) in params.iter().enumerate() {
                    if i > 0 {
                        ui.label(egui::RichText::new(", ").monospace()
                            .color(egui::Color32::GRAY).size(13.0));
                    }
                    if i == param_idx {
                        ui.label(egui::RichText::new(*p).monospace()
                            .color(egui::Color32::WHITE)
                            .strong()
                            .size(13.0));
                    } else {
                        ui.label(egui::RichText::new(*p).monospace()
                            .color(egui::Color32::from_rgb(180, 180, 180))
                            .size(13.0));
                    }
                }
                ui.label(egui::RichText::new(")").monospace().color(egui::Color32::GRAY).size(13.0));
            });
        });
}

/// Scan backward from `cursor` to find if we are inside a function call's
/// parentheses.  Returns (function_name, parameter_index_0based) or None.
fn current_call_site(text: &str, cursor: usize) -> Option<(&str, usize)> {
    let before = &text[..cursor.min(text.len())];

    // Find the matching open-paren by tracking depth.
    let mut depth  = 0i32;
    let mut commas = 0usize;
    let mut open_pos: Option<usize> = None;

    for (byte_pos, ch) in before.char_indices().rev() {
        match ch {
            ')' => depth += 1,
            '(' => {
                if depth == 0 {
                    open_pos = Some(byte_pos);
                    break;
                }
                depth -= 1;
            }
            ',' if depth == 0 => commas += 1,
            '\n' if depth == 0 => return None,  // crossed a line — bail
            _ => {}
        }
    }

    let open = open_pos?;
    // What's immediately before the `(`?
    let before_paren = &before[..open].trim_end();
    let fn_start = before_paren
        .rfind(|c: char| !c.is_alphanumeric() && c != '_')
        .map(|p| p + 1)
        .unwrap_or(0);
    let fn_name = &before_paren[fn_start..];
    if fn_name.is_empty() { return None; }

    Some((fn_name, commas))
}
