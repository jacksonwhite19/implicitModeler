// Project templates for the New Project wizard
#![allow(dead_code)] // Templates are used by project_wizard which is not yet wired into the main UI

use std::collections::HashMap;

pub struct ProjectTemplate {
    pub id:          &'static str,
    pub name:        &'static str,
    pub description: &'static str,
    pub tags:        &'static [&'static str],
    pub script:      &'static str,   // Rhai script with {param} placeholders
    pub params:      &'static [TemplateParam],
}

pub struct TemplateParam {
    pub name:    &'static str,
    pub label:   &'static str,
    pub default: f64,
    pub min:     f64,
    pub max:     f64,
}

// ── Template scripts ──────────────────────────────────────────────────────────

static BLANK_SCRIPT: &str = r#"// Blank Project
sphere(10.0)"#;

static FIXED_WING_SCRIPT: &str = r#"// Fixed Wing Template
let wingspan = {wingspan};
let fuse_len = {fuselage_length};
let wing = wing_with_airfoil("2412", wingspan * 0.22, wingspan * 0.15, wingspan / 2.0, 3.0, 1.5, -1.0);
let full_wing = mirror_y(wing);
let fuse = fuselage([
    [0.0,  circle_section(0.08 * fuse_len * 0.01)],
    [0.15, circle_section(0.35 * fuse_len * 0.01)],
    [0.55, circle_section(0.38 * fuse_len * 0.01)],
    [0.85, circle_section(0.28 * fuse_len * 0.01)],
    [1.0,  circle_section(0.08 * fuse_len * 0.01)],
]);
// Aerodynamic Analysis — uncomment after adding h_tail and v_tail surfaces
//
// let fc = flight_condition_sl(18.0, 4.0);  // 18 m/s cruise, 4 deg AoA
// let ll = run_lifting_line(full_wing, fc);
// let stability = static_margin(full_wing, h_tail, v_tail, fuse, fc);
// let polar = drag_polar(full_wing, fuse, h_tail, v_tail, fc);
// let trim = trim_analysis(full_wing, h_tail, fuse, fc, 8.0 * 9.81);  // 8N weight
//
// Key results: ll.cl_total, stability.static_margin_mac, polar.ld_max, trim.trim_aoa_deg
union(fuse, full_wing)"#;

static FLYING_WING_SCRIPT: &str = r#"// Flying Wing Template
let wingspan = {wingspan};
let wing = wing_with_airfoil("2412", wingspan * 0.20, wingspan * 0.10, wingspan / 2.0, {sweep_angle}, 0.0, 0.0);
mirror_y(wing)"#;

static QUADCOPTER_SCRIPT: &str = r#"// Quadcopter Template
let fuse = fuselage([
    [0.0, circle_section(30.0)],
    [0.5, circle_section(50.0)],
    [1.0, circle_section(25.0)],
]);
let arm_len = {arm_length};
let arm_d = {arm_diameter};
let arm0   = motor_arm(fuse,   0.0, arm_len, arm_d * 1.2, arm_d * 0.8);
let arm90  = motor_arm(fuse,  90.0, arm_len, arm_d * 1.2, arm_d * 0.8);
let arm180 = motor_arm(fuse, 180.0, arm_len, arm_d * 1.2, arm_d * 0.8);
let arm270 = motor_arm(fuse, 270.0, arm_len, arm_d * 1.2, arm_d * 0.8);
union(union(union(union(fuse, arm0), arm90), arm180), arm270)"#;

static STRUCTURAL_SCRIPT: &str = r#"// Structural Part Template
let w = {width};
let h = {height};
let d = {depth};
box_(w, h, d)"#;

// ── Template parameter definitions ───────────────────────────────────────────

static FIXED_WING_PARAMS: &[TemplateParam] = &[
    TemplateParam { name: "wingspan",         label: "Wingspan (mm)",        default: 800.0, min: 100.0, max: 5000.0 },
    TemplateParam { name: "fuselage_length",  label: "Fuselage length (mm)", default: 600.0, min: 100.0, max: 3000.0 },
];

static FLYING_WING_PARAMS: &[TemplateParam] = &[
    TemplateParam { name: "wingspan",    label: "Wingspan (mm)",    default: 700.0, min: 100.0, max: 5000.0 },
    TemplateParam { name: "sweep_angle", label: "Sweep angle (°)",  default: 30.0,  min: 0.0,   max: 60.0  },
];

static QUADCOPTER_PARAMS: &[TemplateParam] = &[
    TemplateParam { name: "arm_length",   label: "Arm length (mm)",   default: 150.0, min: 50.0,  max: 500.0 },
    TemplateParam { name: "arm_diameter", label: "Arm diameter (mm)", default: 12.0,  min: 4.0,   max: 50.0  },
];

static STRUCTURAL_PARAMS: &[TemplateParam] = &[
    TemplateParam { name: "width",  label: "Width (mm)",  default: 50.0, min: 1.0, max: 1000.0 },
    TemplateParam { name: "height", label: "Height (mm)", default: 30.0, min: 1.0, max: 1000.0 },
    TemplateParam { name: "depth",  label: "Depth (mm)",  default: 20.0, min: 1.0, max: 1000.0 },
];

// ── Template list ─────────────────────────────────────────────────────────────

static TEMPLATES: &[ProjectTemplate] = &[
    ProjectTemplate {
        id:          "blank",
        name:        "Blank",
        description: "Empty project with a placeholder sphere",
        tags:        &["general"],
        script:      BLANK_SCRIPT,
        params:      &[],
    },
    ProjectTemplate {
        id:          "fixed_wing",
        name:        "Fixed Wing",
        description: "Conventional fixed-wing aircraft with fuselage",
        tags:        &["aerospace", "aircraft"],
        script:      FIXED_WING_SCRIPT,
        params:      FIXED_WING_PARAMS,
    },
    ProjectTemplate {
        id:          "flying_wing",
        name:        "Flying Wing",
        description: "Tailless flying wing with configurable sweep",
        tags:        &["aerospace", "aircraft"],
        script:      FLYING_WING_SCRIPT,
        params:      FLYING_WING_PARAMS,
    },
    ProjectTemplate {
        id:          "quadcopter",
        name:        "Quadcopter",
        description: "Four-arm multirotor frame",
        tags:        &["drone", "multirotor"],
        script:      QUADCOPTER_SCRIPT,
        params:      QUADCOPTER_PARAMS,
    },
    ProjectTemplate {
        id:          "structural_part",
        name:        "Structural Part",
        description: "Simple rectangular structural part",
        tags:        &["mechanical", "structural"],
        script:      STRUCTURAL_SCRIPT,
        params:      STRUCTURAL_PARAMS,
    },
];

pub fn get_templates() -> &'static [ProjectTemplate] {
    TEMPLATES
}

/// Instantiate a template by replacing {param} placeholders with parameter values.
pub fn instantiate(template: &ProjectTemplate, params: &HashMap<String, f64>) -> String {
    let mut out = template.script.to_string();
    for (k, v) in params {
        out = out.replace(&format!("{{{}}}", k), &format!("{:.6}", v));
    }
    out
}
