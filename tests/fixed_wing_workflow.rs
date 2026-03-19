use implicit_cad::export::build_export_mesh;
use implicit_cad::headless::compute_metrics;
use implicit_cad::headless::execute_script_headless_extended;
use implicit_cad::pipeline::auto_bounds;
use implicit_cad::project::Project;
use implicit_cad::scripting::evaluate_script_full;
use implicit_cad::ui::templates::{get_templates, instantiate_project};
use std::collections::HashMap;
use tempfile::TempDir;

fn fixed_wing_instance(wingspan: f64) -> implicit_cad::ui::templates::TemplateInstance {
    let template = get_templates().iter().find(|t| t.id == "fixed_wing").unwrap();
    let mut params = HashMap::new();
    params.insert("wingspan".to_string(), wingspan);
    params.insert("root_chord".to_string(), 180.0);
    params.insert("tip_chord".to_string(), 110.0);
    params.insert("fuse_width".to_string(), 70.0);
    params.insert("fuse_height".to_string(), 85.0);
    instantiate_project(template, &params)
}

#[test]
fn fixed_wing_complete_800mm_workflow_generates_mesh() {
    let instance = fixed_wing_instance(800.0);
    let result = evaluate_script_full(
        &instance.script,
        None,
        None,
        None,
        None,
        &instance.dimensions,
        None,
        None,
        &[],
    )
    .expect("fixed-wing template should evaluate");

    let (bounds_min, bounds_max) = auto_bounds(result.sdf.as_ref());
    let mesh = build_export_mesh(result.sdf.as_ref(), bounds_min, bounds_max, 32, false);
    let mass_names: Vec<_> = result.mass_points.iter().map(|m| m.name.as_str()).collect();

    assert!(!mesh.vertices.is_empty(), "fixed-wing workflow should produce a mesh");
    assert!(mass_names.contains(&"battery"));
    assert!(mass_names.contains(&"servo_l"));
    assert!(mass_names.contains(&"servo_r"));
    assert!(mass_names.contains(&"flight_controller"));
    assert!(mass_names.contains(&"receiver"));
}

#[test]
fn fixed_wing_dimension_change_regenerates_expected_span() {
    let small = fixed_wing_instance(800.0);
    let large = fixed_wing_instance(920.0);

    let eval_small = evaluate_script_full(
        &small.script,
        None,
        None,
        None,
        None,
        &small.dimensions,
        None,
        None,
        &[],
    )
    .unwrap();
    let eval_large = evaluate_script_full(
        &large.script,
        None,
        None,
        None,
        None,
        &large.dimensions,
        None,
        None,
        &[],
    )
    .unwrap();

    let (small_min, small_max) = auto_bounds(eval_small.sdf.as_ref());
    let small_mesh = build_export_mesh(eval_small.sdf.as_ref(), small_min, small_max, 24, false);
    let small_metrics = compute_metrics(eval_small.sdf.as_ref(), &small_mesh, &eval_small, 24, 0, &small.dimensions);

    let (large_min, large_max) = auto_bounds(eval_large.sdf.as_ref());
    let large_mesh = build_export_mesh(eval_large.sdf.as_ref(), large_min, large_max, 24, false);
    let large_metrics = compute_metrics(eval_large.sdf.as_ref(), &large_mesh, &eval_large, 24, 0, &large.dimensions);

    assert!(
        (large_metrics.volume_mm3 - small_metrics.volume_mm3).abs() > 1_000.0,
        "changing wingspan should materially change the aircraft volume"
    );
}

#[test]
fn fixed_wing_project_round_trips_to_icad_and_headless_export() {
    let temp = TempDir::new().unwrap();
    let instance = fixed_wing_instance(800.0);
    let project_path = temp.path().join("fixed_wing.icad");
    let mesh_path = temp.path().join("fixed_wing.stl");
    let metrics_path = temp.path().join("fixed_wing_metrics.json");

    let project = Project::new(
        instance.script.clone(),
        32,
        false,
        false,
        [0.0, 0.0, 600.0],
        [0.0, 0.0, 0.0],
        None,
        None,
        None,
        None,
        instance.dimensions.clone(),
        instance.print_analysis_settings.clone(),
        instance.tolerance_settings.clone(),
        instance.workflow_config.clone(),
    );
    project.save(&project_path).unwrap();

    execute_script_headless_extended(
        &project_path,
        Some(&mesh_path),
        "stl",
        32,
        false,
        &[],
        Some(&metrics_path),
    )
    .unwrap();

    assert!(mesh_path.exists(), "headless fixed-wing export should write STL");
    assert!(metrics_path.exists(), "headless fixed-wing export should write metrics");
    let metrics = std::fs::read_to_string(metrics_path).unwrap();
    assert!(metrics.contains("\"wingspan\""));
    assert!(metrics.contains("800.0"));
}

#[test]
fn fixed_wing_package_export_writes_manufacturing_artifacts() {
    let temp = TempDir::new().unwrap();
    let instance = fixed_wing_instance(800.0);
    let project_path = temp.path().join("fixed_wing.icad");
    let package_dir = temp.path().join("package");

    let project = Project::new(
        instance.script,
        32,
        false,
        false,
        [0.0, 0.0, 600.0],
        [0.0, 0.0, 0.0],
        None,
        None,
        None,
        None,
        instance.dimensions,
        instance.print_analysis_settings,
        instance.tolerance_settings,
        instance.workflow_config,
    );
    project.save(&project_path).unwrap();

    execute_script_headless_extended(
        &project_path,
        Some(&package_dir),
        "package",
        32,
        false,
        &[],
        None,
    )
    .unwrap();

    assert!(package_dir.join("main_body.stl").exists());
    assert!(package_dir.join("bom.csv").exists());
    assert!(package_dir.join("assembly_notes.md").exists());
}
