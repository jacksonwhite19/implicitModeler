Good. Go with Option B — LayoutJob-based custom rendering, no new dependencies. It's more work but you avoid adding a crate that may not be maintained and you get full control over highlighting rules that match your exact Rhai API.

```
Implement syntax highlighting and autocomplete for the script editor. Replace the plain egui::TextEdit with a custom editor widget that renders highlighted text using egui::text::LayoutJob while preserving all existing editor behavior including tab indent, cursor position, selection, and scroll state.

Part 1 — Syntax highlighter. Add src/ui/syntax.rs. Implement highlight_script(source: &str) -> egui::text::LayoutJob that tokenizes the source string and assigns colors to each token range.

Token categories and colors (use egui::Color32 values appropriate for a dark background):

Rhai keywords (let, if, else, for, while, fn, return, true, false) — soft purple, approximately Color32::from_rgb(180, 120, 220)

Registered SDF primitive functions (sphere, box_, cylinder, torus, cone, plane) — bright cyan, Color32::from_rgb(80, 200, 220)

Registered boolean functions (union, subtract, intersect, smooth_union, smooth_subtract, smooth_intersect, blend) — orange, Color32::from_rgb(220, 140, 60)

Registered transform functions (translate, rotate, scale, offset, shell, twist, bend) — yellow, Color32::from_rgb(220, 200, 80)

Registered pattern functions (linear_array, polar_array, polar_array_axis, mirror_x, mirror_y, mirror_z) — light green, Color32::from_rgb(120, 210, 120)

Aerospace functions (naca, wing_with_airfoil, fuselage, station, fuselage_parametric, nacelle, bulkhead_at_station, rod_mount, motor_arm, motor_mount, rib_at_station, spar, conformal_gyroid, conformal_diamond, conformal_schwarz_p, wing_lattice, fuselage_lattice, fuselage_lattice_graded) — warm pink, Color32::from_rgb(220, 130, 150)

Field functions (constant_field, sdf_as_field, position_x_field, position_y_field, position_z_field, add_fields, multiply_fields, min_fields, max_fields, abs_field, gradient_field, radial_field, axial_radial_field, offset_by_field, shell_with_field, blend_by_field, gyroid_with_field, stress_field, displacement_field) — light blue, Color32::from_rgb(100, 160, 220)

Component and mass functions (component, component_mass, component_named, place, geometry, keepout, mass_g, mass_at, mass_named, auto_fuselage, generate_mounts, mount_with_bolts) — salmon, Color32::from_rgb(220, 160, 120)

FEA functions (fixed_support, fixed_axis, force_load, pressure_load, gravity_load, torque_load, motor_thrust) — red-orange, Color32::from_rgb(220, 80, 80)

String literals (content between double quotes) — green, Color32::from_rgb(140, 200, 100)

Numeric literals (integers and floats) — light orange, Color32::from_rgb(210, 170, 100)

Line comments (from // to end of line) — grey, Color32::from_rgb(120, 120, 120)

Identifiers (anything else that is not punctuation) — default text color, Color32::from_rgb(210, 210, 210)

Punctuation and operators — slightly dimmed default, Color32::from_rgb(160, 160, 160)

Implement the tokenizer as a simple character-by-character scanner, not a full parser. It does not need to be 100% correct in all edge cases — it needs to be fast (runs every frame) and correct for common patterns. Specifically: scan for // and color to end of line, scan for " and color to closing ", match known keyword and function name strings at word boundaries, match digit-starting sequences as numbers, everything else is identifier or punctuation.

The LayoutJob must use the same monospace font as the existing editor (egui::TextStyle::Monospace) and the same font size. Line endings must be preserved exactly.

Part 2 — Custom editor widget. Replace the egui::TextEdit block in src/app.rs with a new function render_script_editor(ui: &mut egui::Ui, state: &mut ScriptEditorState) -> egui::Response in src/ui/script_editor.rs.

The widget works as follows: render the highlighted text as a non-interactive egui::Label using the LayoutJob from Part 1, positioned behind a transparent egui::TextEdit of the same size that handles all keyboard and mouse input. The TextEdit remains the actual input handler — it captures keystrokes, manages cursor, selection, clipboard, undo/redo. The Label renders the highlighted version on top using the same text content. Use ui.put() to stack them at the same Rect. The TextEdit background must be set to Color32::TRANSPARENT so the highlighted label shows through. This is the standard egui approach for syntax-highlighted editors.

Add ScriptEditorState to App:
pub struct ScriptEditorState {
    pub text: String,
    pub highlight_cache: Option<(u64, egui::text::LayoutJob)>,
}

Cache the LayoutJob result keyed by a hash of the source text. Only re-tokenize when the text changes. This ensures highlighting does not re-run every frame when the script is unchanged.

Part 3 — Autocomplete. Add src/ui/autocomplete.rs. Implement a completion system that shows a small floating popup below the cursor when the user is typing a function name.

Maintain a master list of all 61 registered Rhai functions with their full signatures as strings. Group them by category matching the highlighter categories above.

Trigger autocomplete when: the character just typed is a letter or underscore, the current token (word under cursor, scanning backward to the nearest non-identifier character) is at least 2 characters long, and at least one registered function name starts with that prefix.

Show the autocomplete popup as a small egui::Window positioned below the current cursor line. Display up to 8 matching completions. Each entry shows the function name in its category color followed by the parameter signature in grey. The currently selected completion is highlighted with a subtle background. Navigate with arrow keys, confirm with Tab or Enter, dismiss with Escape or by clicking elsewhere.

When a completion is confirmed, replace the current token in the script text with the full function name and insert a ( after it if not already present. Place the cursor inside the parentheses.

Add autocomplete_state: AutocompleteState to App:
pub struct AutocompleteState {
    pub visible: bool,
    pub completions: Vec<CompletionItem>,
    pub selected_index: usize,
    pub anchor_pos: egui::Pos2,
}

Part 4 — Function signature tooltip. When the cursor is inside a function call parentheses (the current line contains an open paren with a known function name before it and the cursor is between the parens), show a small tooltip above the cursor displaying the full function signature. Bold the current parameter based on comma count before the cursor. This is the standard signature help behavior found in VS Code and similar editors. Implement as a separate non-interactive egui::Window with a dark background and a single line of formatted text.

Part 5 — Preserve all existing behavior. The new editor must preserve: tab key inserts four spaces, Ctrl+Z and Ctrl+Y for undo/redo (handled by egui::TextEdit natively), Ctrl+A to select all, click to place cursor, click and drag to select, scroll to follow cursor after script evaluation jumps to a line from the project tree. Run the full test suite after implementation to confirm no regressions in script evaluation.
```