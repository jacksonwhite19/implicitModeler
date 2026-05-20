use std::fs;

use glam::Vec3;
use implicit_cad::export::aero::{AeroExportMode, AeroExportSettings, export_aero_surfaces};
use implicit_cad::export::build_export_mesh_uniform_reference;
use implicit_cad::headless::compute_metrics;
use implicit_cad::mesh::{MeshQuality, analyze_mesh_quality};
use implicit_cad::scripting::{evaluate_aero_export_parts, evaluate_script};
use implicit_cad::sdf::primitives::Sphere;
use tempfile::TempDir;

#[test]
fn uniform_reference_sphere_is_closed() {
    let sphere = Sphere::new(10.0);
    let mesh = build_export_mesh_uniform_reference(
        &sphere,
        Vec3::splat(-12.0),
        Vec3::splat(12.0),
        MeshQuality::Draft.target_cell_size_mm(),
        false,
    );
    let quality = analyze_mesh_quality(&mesh);
    assert!(!mesh.vertices.is_empty());
    assert!(
        quality.watertight,
        "sphere should be a watertight reference mesh: {quality:?}"
    );
    assert_eq!(quality.boundary_edge_count, 0);
    assert_eq!(quality.non_manifold_edge_count, 0);
}

#[test]
fn headless_metrics_include_mesh_quality_and_extraction_time() {
    let script = evaluate_script("sphere(8.0)").unwrap();
    let mesh = build_export_mesh_uniform_reference(
        script.sdf.as_ref(),
        Vec3::splat(-10.0),
        Vec3::splat(10.0),
        1.0,
        false,
    );
    let metrics = compute_metrics(
        &script.sdf,
        &mesh,
        &script,
        24,
        5,
        7,
        &indexmap::IndexMap::new(),
    );
    assert_eq!(metrics.mesh_extraction_time_ms, 7);
    assert!(metrics.validation.metrics.mesh_quality.watertight);
    assert_eq!(
        metrics.validation.metrics.mesh_quality.boundary_edge_count,
        0
    );
}

#[test]
fn aero_uniform_reference_manifest_reports_shared_quality_metrics() {
    let temp = TempDir::new().unwrap();
    let script_path = temp.path().join("body.rhai");
    fs::write(
        &script_path,
        r#"
            let body = sphere(10.0);
            let aero_export = #{
                parts: [
                    #{
                        name: "body_oml",
                        sdf: body,
                        aero_role: "outer_mold_line",
                        patch_name: "body",
                        include_in_modes: ["external"]
                    }
                ]
            };
            body
        "#,
    )
    .unwrap();
    let script = fs::read_to_string(&script_path).unwrap();
    let parts = evaluate_aero_export_parts(
        &script,
        None,
        None,
        None,
        None,
        &indexmap::IndexMap::new(),
        script_path.parent(),
        None,
        &[],
    )
    .unwrap();
    let manifest = export_aero_surfaces(
        "body",
        &parts,
        temp.path(),
        &AeroExportSettings {
            mode: AeroExportMode::External,
            quality: MeshQuality::Draft,
            uniform_reference_mode: true,
            uniform_reference_target_cell_mm: 1.0,
            min_component_triangles: 0,
            ..Default::default()
        },
    )
    .unwrap();
    let patch = &manifest.patches[0];
    assert_eq!(patch.backend, "uniform_reference_grid");
    assert_eq!(patch.mesh_quality.boundary_edge_count, patch.boundary_edges);
    assert_eq!(
        patch.mesh_quality.manifold_edge_error_count,
        patch.manifold_edge_errors
    );
}
