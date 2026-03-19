// Project templates for the New Project wizard
#![allow(dead_code)] // Templates are used by project_wizard which is not yet wired into the main UI

use std::collections::HashMap;
use indexmap::IndexMap;

use crate::project::{
    AssemblyConstraint, AssemblyConstraintFormula, DesignVariant, ManufacturingPreset, WorkflowConfig,
};

pub struct ProjectTemplate {
    pub id:          &'static str,
    pub name:        &'static str,
    pub description: &'static str,
    pub tags:        &'static [&'static str],
    pub script:      &'static str,
    pub params:      &'static [TemplateParam],
    pub workflow:    Option<TemplateWorkflow>,
}

pub struct TemplateParam {
    pub name:    &'static str,
    pub label:   &'static str,
    pub default: f64,
    pub min:     f64,
    pub max:     f64,
}

#[derive(Clone)]
pub struct TemplateWorkflow {
    pub vehicle_type: &'static str,
    pub manufacturing_preset: ManufacturingPreset,
}

pub struct TemplateInstance {
    pub script:                  String,
    pub dimensions:              IndexMap<String, f64>,
    pub workflow_config:         Option<WorkflowConfig>,
    pub print_analysis_settings: Option<crate::analysis::print_analysis::PrintAnalysisSettings>,
    pub tolerance_settings:      Option<crate::sdf::print::ToleranceSettings>,
}

// ── Template scripts ──────────────────────────────────────────────────────────

static BLANK_SCRIPT: &str = r#"// Blank Project
sphere(10.0)"#;

static FIXED_WING_SCRIPT: &str = r#"// Canonical fixed-wing aircraft template.
// Top-level values come from the Dimensions panel / project config.

let half_span = wingspan / 2.0;
let wing_z = wing_mount_z;
let tail_z = tail_mount_z;

let half_wing = wing_with_airfoil("2412", root_chord, tip_chord, half_span, sweep_deg, dihedral_deg, washout_deg);
let full_wing = translate(union(half_wing, mirror_y(half_wing)), wing_mount_x, 0.0, wing_z);

let htail_half = wing_with_airfoil("0010", htail_root_chord, htail_tip_chord, htail_span / 2.0, htail_sweep_deg, 0.0, 0.0);
let htail = translate(union(htail_half, mirror_y(htail_half)), tail_mount_x, 0.0, tail_z);

let vtail = rotate(
    wing_with_airfoil("0010", vtail_root_chord, vtail_tip_chord, vtail_span, vtail_sweep_deg, 0.0, 0.0),
    90.0, 0.0, 0.0
);
let vtail = translate(vtail, tail_mount_x, 0.0, tail_z + vtail_span * 0.35);

let fuse = shell(fuselage([
    [0.0,  circle_section(nose_radius)],
    [0.12, ellipse_section(fuse_width * 0.42, fuse_height * 0.38)],
    [0.35, ellipse_section(fuse_width * 0.50, fuse_height * 0.48)],
    [0.62, ellipse_section(fuse_width * 0.50, fuse_height * 0.46)],
    [0.82, ellipse_section(fuse_width * 0.32, fuse_height * 0.28)],
    [1.0,  circle_section(tail_radius)],
]), shell_thickness);
let hatch_parts = battery_hatch(fuse, battery_x, battery_len * 0.72, battery_width + 8.0, snap_retention(2, 12.0, 1.2));
let fuse = hatch_parts[0];
let battery_hatch_panel = hatch_parts[1];
let fc_panel_parts = fc_access_panel(fuse, fc_x, friction_retention(0.3));
let fuse = fc_panel_parts[0];
let fc_access_cover = fc_panel_parts[1];

let battery = component_named("battery", box_(battery_len, battery_width, battery_height), clearance_margin, battery_mass_g);
let battery = place(battery, battery_x, 0.0, battery_z);
let servo_l = component_named("servo_l", box_(servo_len, servo_width, servo_height), clearance_margin, servo_mass_g);
let servo_l = place(servo_l, servo_x,  wing_servo_y, wing_z - 6.0);
let servo_r = component_named("servo_r", box_(servo_len, servo_width, servo_height), clearance_margin, servo_mass_g);
let servo_r = place(servo_r, servo_x, -wing_servo_y, wing_z - 6.0);
let fc = component_named("flight_controller", box_(fc_len, fc_width, fc_height), clearance_margin, fc_mass_g);
let fc = place(fc, fc_x, 0.0, fc_z);
let rx = component_named("receiver", box_(rx_len, rx_width, rx_height), clearance_margin, rx_mass_g);
let rx = place(rx, rx_x, 0.0, rx_z);

let battery_mount = translate(battery_cradle(battery_len, battery_width, battery_height, shell_thickness, strap_slot_width), battery_x, 0.0, battery_z);
let fc_mount = translate(fc_stack_mount(fc_width, fc_len, fc_hole_spacing, fc_standoff_height), fc_x, 0.0, fc_z - fc_height * 0.5);
let servo_tray_l = translate(servo_tray(servo_len, servo_width, servo_height, 1.5, 4.0), servo_x,  wing_servo_y, wing_z - 8.0);
let servo_tray_r = translate(servo_tray(servo_len, servo_width, servo_height, 1.5, 4.0), servo_x, -wing_servo_y, wing_z - 8.0);
let antenna_l = translate(antenna_mount(antenna_mast_height, antenna_base_radius, antenna_mast_radius), rx_x + antenna_offset_x, antenna_y, rx_z);
let antenna_r = translate(antenna_mount(antenna_mast_height, antenna_base_radius, antenna_mast_radius), rx_x + antenna_offset_x, -antenna_y, rx_z);

let wing_main_spar = translate(rotate(pushrod_guide(wingspan * 0.55, spar_outer_diam, spar_inner_diam), 90.0, 0.0, 0.0), wing_mount_x + root_chord * spar_chord_fraction, 0.0, wing_z);
let fuse_bulkhead_a = bulkhead_at_station(fuse, battery_x - battery_len * 0.6, 2.2);
let fuse_bulkhead_b = bulkhead_at_station(fuse, wing_mount_x, 2.2);
let fuse_bulkhead_c = bulkhead_at_station(fuse, tail_mount_x - 30.0, 2.2);

let aileron_parts = wing_with_ailerons(half_wing, 0.58, 0.93, aileron_chord_fraction);
let aileron_pair = union(aileron_parts[0], aileron_parts[1]);
let full_wing = union(full_wing, translate(aileron_parts[2], wing_mount_x, 0.0, wing_z));
let elevator_cs = elevator(htail_half, 0.5, elevator_chord_fraction);
let rudder_cs = rudder(vtail, 0.5, rudder_chord_fraction);

let pushrod_l = translate(pushrod_guide(pushrod_length(servo_x, wing_servo_y, wing_z - 6.0, wing_mount_x + root_chord * 0.75, wing_servo_y, wing_z), 3.0, 1.6), servo_x, wing_servo_y, wing_z - 6.0);
let pushrod_r = translate(pushrod_guide(pushrod_length(servo_x, -wing_servo_y, wing_z - 6.0, wing_mount_x + root_chord * 0.75, -wing_servo_y, wing_z), 3.0, 1.6), servo_x, -wing_servo_y, wing_z - 6.0);

let airframe = union(union(union(union(union(fuse, full_wing), htail), vtail), aileron_pair), union(elevator_cs, rudder_cs));
let structure = union(union(union(union(wing_main_spar, fuse_bulkhead_a), fuse_bulkhead_b), fuse_bulkhead_c), union(battery_mount, fc_mount));
let internals = union(union(union(servo_tray_l, servo_tray_r), union(pushrod_l, pushrod_r)), union(antenna_l, antenna_r));
let service_parts = union(battery_hatch_panel, fc_access_cover);
let printable = union(union(union(airframe, structure), internals), service_parts);

let joined = add_alignment_features(printable, "x", split_station_x, split_pin_count);

let fc = flight_condition_sl(cruise_speed_ms, cruise_aoa_deg);
let propulsion = propulsion_setup(motor_custom(motor_kv, motor_power_w, motor_current_a, motor_mass_g, motor_resistance_ohm), prop_by_size(prop_diameter_mm, prop_pitch_mm), battery_cells, battery_capacity_mah);
let _perf = propulsion_analysis(propulsion, fc, design_weight_n);
let _glide = glide_performance(half_wing, fuse, htail, vtail, fc, design_weight_n);
let _roc = rate_of_climb(propulsion, half_wing, fuse, htail, vtail, fc, design_weight_n);

joined"#;

static FLYING_WING_SCRIPT: &str = r#"// Flying Wing Template
let wing = wing_with_airfoil("2412", wingspan * 0.20, wingspan * 0.10, wingspan / 2.0, sweep_angle, 0.0, 0.0);
mirror_y(wing)"#;

static QUADCOPTER_SCRIPT: &str = r#"// Quadcopter Template
let fuse = fuselage([
    [0.0, circle_section(30.0)],
    [0.5, circle_section(50.0)],
    [1.0, circle_section(25.0)],
]);
let arm_len = arm_length;
let arm_d = arm_diameter;
let arm0   = motor_arm(fuse,   0.0, arm_len, arm_d * 1.2, arm_d * 0.8);
let arm90  = motor_arm(fuse,  90.0, arm_len, arm_d * 1.2, arm_d * 0.8);
let arm180 = motor_arm(fuse, 180.0, arm_len, arm_d * 1.2, arm_d * 0.8);
let arm270 = motor_arm(fuse, 270.0, arm_len, arm_d * 1.2, arm_d * 0.8);
union(union(union(union(fuse, arm0), arm90), arm180), arm270)"#;

static STRUCTURAL_SCRIPT: &str = r#"// Structural Part Template
let w = width;
let h = height;
let d = depth;
box_(w, h, d)"#;

// ── Template parameter definitions ───────────────────────────────────────────

static FIXED_WING_PARAMS: &[TemplateParam] = &[
    TemplateParam { name: "wingspan",         label: "Wingspan (mm)",        default: 800.0, min: 100.0, max: 5000.0 },
    TemplateParam { name: "root_chord",       label: "Root chord (mm)",      default: 180.0, min: 50.0,  max: 1000.0 },
    TemplateParam { name: "tip_chord",        label: "Tip chord (mm)",       default: 110.0, min: 30.0,  max: 800.0 },
    TemplateParam { name: "fuse_width",       label: "Fuselage width (mm)",  default: 70.0,  min: 20.0,  max: 300.0 },
    TemplateParam { name: "fuse_height",      label: "Fuselage height (mm)", default: 85.0,  min: 20.0,  max: 300.0 },
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
        workflow:    None,
    },
    ProjectTemplate {
        id:          "fixed_wing",
        name:        "Fixed Wing",
        description: "Conventional fixed-wing aircraft with fuselage",
        tags:        &["aerospace", "aircraft"],
        script:      FIXED_WING_SCRIPT,
        params:      FIXED_WING_PARAMS,
        workflow:    Some(TemplateWorkflow {
            vehicle_type: "fixed_wing",
            manufacturing_preset: ManufacturingPreset::CarbonTubeSpar,
        }),
    },
    ProjectTemplate {
        id:          "flying_wing",
        name:        "Flying Wing",
        description: "Tailless flying wing with configurable sweep",
        tags:        &["aerospace", "aircraft"],
        script:      FLYING_WING_SCRIPT,
        params:      FLYING_WING_PARAMS,
        workflow:    Some(TemplateWorkflow {
            vehicle_type: "flying_wing",
            manufacturing_preset: ManufacturingPreset::Foamboard,
        }),
    },
    ProjectTemplate {
        id:          "quadcopter",
        name:        "Quadcopter",
        description: "Four-arm multirotor frame",
        tags:        &["drone", "multirotor"],
        script:      QUADCOPTER_SCRIPT,
        params:      QUADCOPTER_PARAMS,
        workflow:    None,
    },
    ProjectTemplate {
        id:          "structural_part",
        name:        "Structural Part",
        description: "Simple rectangular structural part",
        tags:        &["mechanical", "structural"],
        script:      STRUCTURAL_SCRIPT,
        params:      STRUCTURAL_PARAMS,
        workflow:    None,
    },
];

pub fn get_templates() -> &'static [ProjectTemplate] {
    TEMPLATES
}

pub fn preset_dimension_defaults(preset: &ManufacturingPreset) -> IndexMap<String, f64> {
    let mut dims = IndexMap::new();
    match preset {
        ManufacturingPreset::Foamboard => {
            dims.insert("shell_thickness".into(), 4.0);
            dims.insert("spar_outer_diam".into(), 8.0);
            dims.insert("spar_inner_diam".into(), 6.0);
            dims.insert("clearance_margin".into(), 3.0);
        }
        ManufacturingPreset::LwPlaShell => {
            dims.insert("shell_thickness".into(), 1.2);
            dims.insert("spar_outer_diam".into(), 10.0);
            dims.insert("spar_inner_diam".into(), 8.0);
            dims.insert("clearance_margin".into(), 2.0);
        }
        ManufacturingPreset::CarbonTubeSpar => {
            dims.insert("shell_thickness".into(), 1.6);
            dims.insert("spar_outer_diam".into(), 10.0);
            dims.insert("spar_inner_diam".into(), 8.0);
            dims.insert("clearance_margin".into(), 2.0);
        }
        ManufacturingPreset::BalsaHybrid => {
            dims.insert("shell_thickness".into(), 2.0);
            dims.insert("spar_outer_diam".into(), 9.0);
            dims.insert("spar_inner_diam".into(), 7.0);
            dims.insert("clearance_margin".into(), 1.5);
        }
        ManufacturingPreset::MoldedShell => {
            dims.insert("shell_thickness".into(), 1.4);
            dims.insert("spar_outer_diam".into(), 8.0);
            dims.insert("spar_inner_diam".into(), 6.0);
            dims.insert("clearance_margin".into(), 1.5);
        }
    }
    dims
}

fn fixed_wing_dimension_defaults() -> IndexMap<String, f64> {
    let mut dims = IndexMap::new();
    let defaults = [
        ("nose_radius", 16.0),
        ("tail_radius", 10.0),
        ("sweep_deg", 8.0),
        ("dihedral_deg", 3.0),
        ("washout_deg", -1.5),
        ("wing_mount_x", 115.0),
        ("wing_mount_z", 18.0),
        ("tail_mount_x", 470.0),
        ("tail_mount_z", 34.0),
        ("htail_span", 280.0),
        ("htail_root_chord", 82.0),
        ("htail_tip_chord", 54.0),
        ("htail_sweep_deg", 12.0),
        ("vtail_span", 130.0),
        ("vtail_root_chord", 95.0),
        ("vtail_tip_chord", 48.0),
        ("vtail_sweep_deg", 18.0),
        ("aileron_chord_fraction", 0.24),
        ("elevator_chord_fraction", 0.32),
        ("rudder_chord_fraction", 0.34),
        ("battery_len", 105.0),
        ("battery_width", 36.0),
        ("battery_height", 28.0),
        ("battery_x", 70.0),
        ("battery_z", -2.0),
        ("battery_mass_g", 180.0),
        ("servo_len", 23.0),
        ("servo_width", 12.0),
        ("servo_height", 24.0),
        ("servo_x", 165.0),
        ("wing_servo_y", 210.0),
        ("servo_mass_g", 14.0),
        ("fc_len", 36.0),
        ("fc_width", 36.0),
        ("fc_height", 8.0),
        ("fc_x", 132.0),
        ("fc_z", 8.0),
        ("fc_mass_g", 12.0),
        ("fc_hole_spacing", 30.5),
        ("fc_standoff_height", 12.0),
        ("rx_len", 24.0),
        ("rx_width", 14.0),
        ("rx_height", 7.0),
        ("rx_x", 160.0),
        ("rx_z", 12.0),
        ("rx_mass_g", 7.0),
        ("antenna_y", 28.0),
        ("antenna_offset_x", 8.0),
        ("antenna_mast_height", 85.0),
        ("antenna_base_radius", 4.0),
        ("antenna_mast_radius", 1.2),
        ("strap_slot_width", 18.0),
        ("split_station_x", 245.0),
        ("split_pin_count", 4.0),
        ("spar_chord_fraction", 0.27),
        ("cruise_speed_ms", 18.0),
        ("cruise_aoa_deg", 4.0),
        ("motor_kv", 1100.0),
        ("motor_power_w", 350.0),
        ("motor_current_a", 32.0),
        ("motor_mass_g", 70.0),
        ("motor_resistance_ohm", 0.05),
        ("prop_diameter_mm", 229.0),
        ("prop_pitch_mm", 119.0),
        ("battery_cells", 3.0),
        ("battery_capacity_mah", 2200.0),
        ("design_weight_n", 11.0),
    ];
    for (name, value) in defaults {
        dims.insert(name.into(), value);
    }
    dims
}

fn fixed_wing_parameter_groups() -> IndexMap<String, Vec<String>> {
    let mut groups = IndexMap::new();
    groups.insert("Airframe".into(), vec![
        "wingspan".into(), "root_chord".into(), "tip_chord".into(), "fuse_width".into(), "fuse_height".into(),
    ]);
    groups.insert("Tail".into(), vec![
        "htail_span".into(), "htail_root_chord".into(), "htail_tip_chord".into(), "vtail_span".into(),
    ]);
    groups.insert("Internals".into(), vec![
        "battery_len".into(), "battery_width".into(), "battery_height".into(), "servo_len".into(), "servo_width".into(),
        "rx_len".into(), "rx_width".into(), "rx_height".into(),
    ]);
    groups.insert("Manufacturing".into(), vec![
        "shell_thickness".into(), "spar_outer_diam".into(), "spar_inner_diam".into(), "split_station_x".into(),
    ]);
    groups
}

fn fixed_wing_assembly_constraints() -> Vec<AssemblyConstraint> {
    vec![
        AssemblyConstraint {
            label: "Battery follows wing mount".into(),
            driven: "battery_x".into(),
            formula: AssemblyConstraintFormula::CopyOffset {
                source: "wing_mount_x".into(),
                scale: 1.0,
                offset: -45.0,
            },
            enabled: true,
        },
        AssemblyConstraint {
            label: "Flight controller follows wing mount".into(),
            driven: "fc_x".into(),
            formula: AssemblyConstraintFormula::CopyOffset {
                source: "wing_mount_x".into(),
                scale: 1.0,
                offset: 17.0,
            },
            enabled: true,
        },
        AssemblyConstraint {
            label: "Receiver follows flight controller".into(),
            driven: "rx_x".into(),
            formula: AssemblyConstraintFormula::CopyOffset {
                source: "fc_x".into(),
                scale: 1.0,
                offset: 28.0,
            },
            enabled: true,
        },
        AssemblyConstraint {
            label: "Servos follow wing mount".into(),
            driven: "servo_x".into(),
            formula: AssemblyConstraintFormula::CopyOffset {
                source: "wing_mount_x".into(),
                scale: 1.0,
                offset: 50.0,
            },
            enabled: true,
        },
        AssemblyConstraint {
            label: "Tail follows wing mount".into(),
            driven: "tail_mount_x".into(),
            formula: AssemblyConstraintFormula::CopyOffset {
                source: "wing_mount_x".into(),
                scale: 1.0,
                offset: 355.0,
            },
            enabled: true,
        },
        AssemblyConstraint {
            label: "Split station stays between wing and tail".into(),
            driven: "split_station_x".into(),
            formula: AssemblyConstraintFormula::AverageOffset {
                sources: vec!["wing_mount_x".into(), "tail_mount_x".into()],
                offset: -47.5,
            },
            enabled: true,
        },
    ]
}

fn default_variants(dimensions: &IndexMap<String, f64>) -> Vec<DesignVariant> {
    vec![DesignVariant {
        name: "Baseline".into(),
        description: "Default reference configuration".into(),
        dimensions: dimensions.clone(),
    }]
}

pub fn preset_print_analysis_settings(
    preset: &ManufacturingPreset,
) -> crate::analysis::print_analysis::PrintAnalysisSettings {
    let mut print_settings = crate::analysis::print_analysis::PrintAnalysisSettings::default();
    match preset {
        ManufacturingPreset::Foamboard => {
            print_settings.min_wall_thickness = 3.0;
            print_settings.min_feature_size = 2.0;
        }
        ManufacturingPreset::LwPlaShell | ManufacturingPreset::CarbonTubeSpar => {
            print_settings.min_wall_thickness = 0.8;
            print_settings.min_feature_size = 0.6;
        }
        ManufacturingPreset::BalsaHybrid => {
            print_settings.min_wall_thickness = 1.2;
            print_settings.min_feature_size = 0.8;
        }
        ManufacturingPreset::MoldedShell => {
            print_settings.min_wall_thickness = 1.0;
            print_settings.min_feature_size = 0.5;
        }
    }
    print_settings
}

pub fn preset_tolerance_settings(
    preset: &ManufacturingPreset,
) -> crate::sdf::print::ToleranceSettings {
    let mut tolerance_settings = crate::sdf::print::ToleranceSettings::default();
    if !matches!(preset, ManufacturingPreset::Foamboard | ManufacturingPreset::MoldedShell) {
        tolerance_settings.apply_preset(&crate::sdf::print::TolerancePreset::StandardFDM);
    }
    tolerance_settings
}

fn workflow_defaults(workflow: &TemplateWorkflow) -> TemplateInstance {
    let mut dimensions = preset_dimension_defaults(&workflow.manufacturing_preset);
    let assembly_constraints = if workflow.vehicle_type == "fixed_wing" {
        fixed_wing_assembly_constraints()
    } else {
        Vec::new()
    };
    if workflow.vehicle_type == "fixed_wing" {
        dimensions.extend(fixed_wing_dimension_defaults());
        crate::project::apply_assembly_constraints(&mut dimensions, &assembly_constraints);
    }

    let variants = default_variants(&dimensions);

    TemplateInstance {
        script: String::new(),
        dimensions,
        workflow_config: Some(WorkflowConfig {
            template_id: String::new(),
            vehicle_type: workflow.vehicle_type.to_string(),
            manufacturing_preset: workflow.manufacturing_preset.clone(),
            parameter_groups: if workflow.vehicle_type == "fixed_wing" {
                fixed_wing_parameter_groups()
            } else {
                IndexMap::new()
            },
            assembly_constraints,
            variants,
        }),
        print_analysis_settings: Some(preset_print_analysis_settings(&workflow.manufacturing_preset)),
        tolerance_settings: Some(preset_tolerance_settings(&workflow.manufacturing_preset)),
    }
}

pub fn instantiate_project(template: &ProjectTemplate, params: &HashMap<String, f64>) -> TemplateInstance {
    let mut instance = if let Some(workflow) = &template.workflow {
        workflow_defaults(workflow)
    } else {
        TemplateInstance {
            script: String::new(),
            dimensions: IndexMap::new(),
            workflow_config: None,
            print_analysis_settings: None,
            tolerance_settings: None,
        }
    };

    instance.script = template.script.to_string();
    for (k, v) in params {
        instance.dimensions.insert(k.clone(), *v);
    }
    if let Some(cfg) = &mut instance.workflow_config {
        cfg.template_id = template.id.to_string();
    }
    instance
}

#[allow(dead_code)]
pub fn instantiate(template: &ProjectTemplate, params: &HashMap<String, f64>) -> String {
    instantiate_project(template, params).script
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_wing_template_exposes_project_dimensions() {
        let tmpl = get_templates().iter().find(|t| t.id == "fixed_wing").unwrap();
        let mut params = HashMap::new();
        params.insert("wingspan".to_string(), 800.0);
        params.insert("root_chord".to_string(), 180.0);
        params.insert("tip_chord".to_string(), 110.0);
        params.insert("fuse_width".to_string(), 70.0);
        params.insert("fuse_height".to_string(), 85.0);

        let instance = instantiate_project(tmpl, &params);
        assert!(instance.script.contains("battery_cradle"));
        assert_eq!(instance.dimensions.get("wingspan"), Some(&800.0));
        assert!(instance.dimensions.contains_key("shell_thickness"));
        assert!(instance.workflow_config.is_some());
        assert!(!instance.workflow_config.as_ref().unwrap().assembly_constraints.is_empty());
        assert_eq!(instance.workflow_config.as_ref().unwrap().variants.len(), 1);
    }

    #[test]
    fn fixed_wing_template_evaluates() {
        let tmpl = get_templates().iter().find(|t| t.id == "fixed_wing").unwrap();
        let mut params = HashMap::new();
        params.insert("wingspan".to_string(), 800.0);
        params.insert("root_chord".to_string(), 180.0);
        params.insert("tip_chord".to_string(), 110.0);
        params.insert("fuse_width".to_string(), 70.0);
        params.insert("fuse_height".to_string(), 85.0);

        let instance = instantiate_project(tmpl, &params);
        let result = crate::scripting::evaluate_script_full(
            &instance.script,
            None,
            None,
            None,
            None,
            &instance.dimensions,
            None,
            None,
            &[],
        );
        assert!(result.is_ok(), "Fixed-wing template should evaluate: {:?}", result.err());
    }
}
