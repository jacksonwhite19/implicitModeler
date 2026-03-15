// Syntax highlighter for the Rhai script editor.
// Returns an egui::text::LayoutJob with colored token spans.

use eframe::egui::{self, Color32};
use egui::text::{LayoutJob, TextFormat};

// ── Token category colours ────────────────────────────────────────────────────
const C_KEYWORD:    Color32 = Color32::from_rgb(180, 120, 220);
const C_PRIMITIVE:  Color32 = Color32::from_rgb(80,  200, 220);
const C_BOOLEAN:    Color32 = Color32::from_rgb(220, 140,  60);
const C_TRANSFORM:  Color32 = Color32::from_rgb(220, 200,  80);
const C_PATTERN:    Color32 = Color32::from_rgb(120, 210, 120);
const C_AEROSPACE:  Color32 = Color32::from_rgb(220, 130, 150);
const C_FIELD:      Color32 = Color32::from_rgb(100, 160, 220);
const C_COMPONENT:  Color32 = Color32::from_rgb(220, 160, 120);
const C_FEA:        Color32 = Color32::from_rgb(220,  80,  80);
const C_STRING:     Color32 = Color32::from_rgb(140, 200, 100);
const C_NUMBER:     Color32 = Color32::from_rgb(210, 170, 100);
const C_COMMENT:    Color32 = Color32::from_rgb(120, 120, 120);
const C_IDENT:      Color32 = Color32::from_rgb(210, 210, 210);
const C_PUNCT:      Color32 = Color32::from_rgb(160, 160, 160);

// ── Function/keyword name tables ──────────────────────────────────────────────

const KEYWORDS: &[&str] = &[
    "let", "if", "else", "for", "while", "fn", "return", "true", "false",
    "break", "continue", "loop", "in", "switch", "do",
];

const PRIMITIVES: &[&str] = &[
    "sphere", "box_", "cylinder", "torus", "cone", "plane",
    "gyroid", "cubic_lattice", "diamond_lattice",
];

const BOOLEANS: &[&str] = &[
    "union", "subtract", "intersect",
    "smooth_union", "smooth_subtract", "smooth_intersect", "blend",
];

const TRANSFORMS: &[&str] = &[
    "translate", "rotate", "scale", "offset", "shell", "twist", "bend",
];

const PATTERNS: &[&str] = &[
    "linear_array", "polar_array", "polar_array_axis",
    "mirror_x", "mirror_y", "mirror_z",
];

const AEROSPACE: &[&str] = &[
    "naca", "wing_with_airfoil", "fuselage", "fuselage_parametric",
    "fuselage_station", "lofted_fuselage", "nacelle",
    "bulkhead_at_station", "lightening_hole_pattern",
    "rod_mount", "motor_arm", "motor_mount",
    "rib_at_station", "spar",
    "conformal_gyroid", "conformal_gyroid_field", "conformal_gyroid_region",
    "conformal_diamond", "conformal_diamond_field",
    "conformal_schwarz_p",
    "wing_lattice", "fuselage_lattice", "fuselage_lattice_graded",
    "circle_section", "ellipse_section",
    "airfoil_from_points", "airfoil_from_dat",
    "spline_fuselage", "spline", "spline_section",
    "auto_fuselage",
];

const FIELDS: &[&str] = &[
    "constant_field", "sdf_as_field",
    "position_x_field", "position_y_field", "position_z_field",
    "add_fields", "multiply_fields", "min_fields", "max_fields", "abs_field",
    "gradient_field", "radial_field", "axial_radial_field",
    "offset_by_field", "shell_with_field", "blend_by_field",
    "gyroid_with_field",
    "stress_field", "displacement_field",
];

const COMPONENTS: &[&str] = &[
    "component", "component_mass", "component_named",
    "place", "geometry", "keepout",
    "mass_g", "mass_at", "mass_named",
    "generate_mounts", "mount_with_bolts",
];

const FEA_FNS: &[&str] = &[
    "fixed_support", "fixed_axis",
    "force_load", "pressure_load", "gravity_load",
    "torque_load", "motor_thrust",
];

// Math helpers registered in api.rs
const MATH_FNS: &[&str] = &[
    "to_rad", "to_deg", "clamp", "lerp",
];

// ── Main entry point ──────────────────────────────────────────────────────────

/// Tokenise `text` and return a coloured `LayoutJob` suitable for use inside
/// a `TextEdit::layouter` closure.  Does not set `wrap.max_width`; the caller
/// should set that to `wrap_width`.
pub fn highlight_script(text: &str, font_id: egui::FontId) -> LayoutJob {
    let mut job = LayoutJob::default();

    // Collect (byte_offset, char) pairs so we can slice `text` correctly.
    let chars: Vec<(usize, char)> = text.char_indices().collect();
    let n = chars.len();
    let mut i = 0;

    while i < n {
        let (byte_pos, ch) = chars[i];

        // ── Line comment  //…\n ──────────────────────────────────────────────
        if ch == '/' && i + 1 < n && chars[i + 1].1 == '/' {
            let start = byte_pos;
            while i < n && chars[i].1 != '\n' { i += 1; }
            let end = if i < n { chars[i].0 } else { text.len() };
            append(&mut job, &text[start..end], C_COMMENT, &font_id);
            continue;
        }

        // ── String literal  "…" ──────────────────────────────────────────────
        if ch == '"' {
            let start = byte_pos;
            i += 1;
            while i < n {
                let c = chars[i].1;
                if c == '\\' { i += 2; continue; }   // skip escape
                if c == '"'  { i += 1; break; }
                i += 1;
            }
            let end = if i < n { chars[i].0 } else { text.len() };
            append(&mut job, &text[start..end], C_STRING, &font_id);
            continue;
        }

        // ── Numeric literal ───────────────────────────────────────────────────
        if ch.is_ascii_digit() {
            let start = byte_pos;
            while i < n {
                let c = chars[i].1;
                if c.is_ascii_alphanumeric() || c == '.' || c == '_' { i += 1; }
                else { break; }
            }
            let end = if i < n { chars[i].0 } else { text.len() };
            append(&mut job, &text[start..end], C_NUMBER, &font_id);
            continue;
        }

        // ── Identifier / keyword / function name ──────────────────────────────
        if ch.is_alphabetic() || ch == '_' {
            let start = byte_pos;
            while i < n {
                let c = chars[i].1;
                if c.is_alphanumeric() || c == '_' { i += 1; }
                else { break; }
            }
            let end = if i < n { chars[i].0 } else { text.len() };
            let word = &text[start..end];
            let color = word_color(word);
            append(&mut job, word, color, &font_id);
            continue;
        }

        // ── Punctuation / operator / whitespace ───────────────────────────────
        // Collect a run of non-word, non-string-start, non-comment-start chars
        // (each gets the same colour so we can batch them for efficiency).
        let start = byte_pos;
        while i < n {
            let (_, c) = chars[i];
            // Stop before: identifier start, digit, string, comment
            if c.is_alphabetic() || c == '_' || c.is_ascii_digit() || c == '"' { break; }
            if c == '/' && i + 1 < n && chars[i + 1].1 == '/' { break; }
            i += 1;
        }
        let end = if i < n { chars[i].0 } else { text.len() };
        if end > start {
            append(&mut job, &text[start..end], C_PUNCT, &font_id);
        }
    }

    job
}

// ── Helpers ───────────────────────────────────────────────────────────────────

fn append(job: &mut LayoutJob, text: &str, color: Color32, font_id: &egui::FontId) {
    job.append(text, 0.0, TextFormat {
        font_id: font_id.clone(),
        color,
        ..Default::default()
    });
}

fn word_color(word: &str) -> Color32 {
    if KEYWORDS.contains(&word)   { return C_KEYWORD;   }
    if PRIMITIVES.contains(&word) { return C_PRIMITIVE;  }
    if BOOLEANS.contains(&word)   { return C_BOOLEAN;    }
    if TRANSFORMS.contains(&word) { return C_TRANSFORM;  }
    if PATTERNS.contains(&word)   { return C_PATTERN;    }
    if AEROSPACE.contains(&word)  { return C_AEROSPACE;  }
    if FIELDS.contains(&word)     { return C_FIELD;      }
    if COMPONENTS.contains(&word) { return C_COMPONENT;  }
    if FEA_FNS.contains(&word)    { return C_FEA;        }
    if MATH_FNS.contains(&word)   { return C_TRANSFORM;  } // treat as transform-ish
    C_IDENT
}
