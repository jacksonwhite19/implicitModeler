// Integration tests for field-driven operations

use implicit_cad::scripting::evaluate_script;
use glam::Vec3;

#[test]
fn test_constant_field_script() {
    let script = r#"
        let sphere = sphere(10.0);
        let thickness = constant_field(2.0);
        offset_by_field(sphere, thickness)
    "#;

    let result = evaluate_script(script);
    assert!(result.is_ok(), "Constant field script should succeed");

    if let Ok(r) = result { let sdf = r.sdf;
        // Should behave like regular offset
        let dist = sdf.distance(Vec3::new(12.0, 0.0, 0.0));
        assert!(dist.abs() < 0.1, "Distance should be ~0 at radius 12");
    }
}

#[test]
fn test_radial_field_script() {
    let script = r#"
        let sphere = sphere(20.0);
        let thickness = radial_field(0.0, 0.0, 0.0, 0.0, 20.0, 4.0, 1.0);
        shell_with_field(sphere, thickness)
    "#;

    let result = evaluate_script(script);
    assert!(result.is_ok(), "Radial field script should succeed");

    if let Ok(r) = result { let sdf = r.sdf;
        // Verify it produces valid distance values
        let dist = sdf.distance(Vec3::ZERO);
        assert!(dist.is_finite());
    }
}

#[test]
fn test_field_arithmetic_script() {
    let script = r#"
        let base = constant_field(2.0);
        let variation = multiply_fields(position_x_field(), constant_field(0.1));
        let combined = add_fields(base, variation);
        offset_by_field(sphere(10.0), combined)
    "#;

    let result = evaluate_script(script);
    assert!(result.is_ok(), "Field arithmetic script should succeed");
}

#[test]
fn test_gradient_field_script() {
    let script = r#"
        let field = gradient_field(0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 1.0, 5.0);
        offset_by_field(sphere(10.0), field)
    "#;

    let result = evaluate_script(script);
    assert!(result.is_ok(), "Gradient field script should succeed");
}

#[test]
fn test_gyroid_lattice_script() {
    let script = r#"
        gyroid(5.0, 0.8)
    "#;

    let result = evaluate_script(script);
    assert!(result.is_ok(), "Gyroid lattice script should succeed");

    if let Ok(r) = result { let sdf = r.sdf;
        // Verify lattice produces finite values
        let dist = sdf.distance(Vec3::new(5.0, 5.0, 5.0));
        assert!(dist.is_finite());
    }
}

#[test]
fn test_cubic_lattice_script() {
    let script = r#"
        cubic_lattice(10.0, 0.5)
    "#;

    let result = evaluate_script(script);
    assert!(result.is_ok(), "Cubic lattice script should succeed");

    if let Ok(r) = result { let sdf = r.sdf;
        let dist = sdf.distance(Vec3::ZERO);
        assert!(dist.is_finite());
    }
}

#[test]
fn test_diamond_lattice_script() {
    let script = r#"
        diamond_lattice(10.0, 1.0)
    "#;

    let result = evaluate_script(script);
    assert!(result.is_ok(), "Diamond lattice script should succeed");
}

#[test]
fn test_gyroid_with_field_script() {
    let script = r#"
        let density = radial_field(0.0, 0.0, 0.0, 0.0, 30.0, 1.5, 0.3);
        gyroid_with_field(5.0, density)
    "#;

    let result = evaluate_script(script);
    assert!(result.is_ok(), "Gyroid with field script should succeed");
}

#[test]
fn test_lattice_in_box_script() {
    let script = r#"
        let lattice = gyroid(5.0, 0.8);
        let box = box_(40.0, 40.0, 40.0);
        intersect(box, lattice)
    "#;

    let result = evaluate_script(script);
    assert!(result.is_ok(), "Lattice in box script should succeed");

    if let Ok(r) = result { let sdf = r.sdf;
        // Inside box and near lattice structure
        let dist = sdf.distance(Vec3::ZERO);
        assert!(dist.is_finite());

        // Far outside box
        let dist_outside = sdf.distance(Vec3::new(100.0, 100.0, 100.0));
        assert!(dist_outside > 0.0);
    }
}

#[test]
fn test_blend_by_field_script() {
    let script = r#"
        let s1 = sphere(10.0);
        let s2 = translate(sphere(10.0), 15.0, 0.0, 0.0);
        let smoothness = constant_field(3.0);
        blend_by_field(s1, s2, smoothness)
    "#;

    let result = evaluate_script(script);
    assert!(result.is_ok(), "Blend by field script should succeed");
}

#[test]
fn test_complex_field_composition() {
    let script = r#"
        // Complex field: max(abs(position_x), constant)
        let pos_x = position_x_field();
        let abs_x = abs_field(pos_x);
        let min_thickness = constant_field(0.5);
        let thickness = max_fields(abs_x, min_thickness);

        shell_with_field(sphere(20.0), thickness)
    "#;

    let result = evaluate_script(script);
    assert!(result.is_ok(), "Complex field composition should succeed");
}

#[test]
fn test_variable_thickness_example() {
    // Test the actual example file
    let script = std::fs::read_to_string("examples/variable_thickness_shell.rhai")
        .expect("Should be able to read variable_thickness_shell.rhai");

    let result = evaluate_script(&script);
    assert!(result.is_ok(), "Variable thickness example should succeed");
}

#[test]
fn test_field_arithmetic_example() {
    // Test the actual example file
    let script = std::fs::read_to_string("examples/field_arithmetic.rhai")
        .expect("Should be able to read field_arithmetic.rhai");

    let result = evaluate_script(&script);
    assert!(result.is_ok(), "Field arithmetic example should succeed");
}

#[test]
fn test_gyroid_lattice_example() {
    // Test the actual example file
    let script = std::fs::read_to_string("examples/gyroid_lattice.rhai")
        .expect("Should be able to read gyroid_lattice.rhai");

    let result = evaluate_script(&script);
    assert!(result.is_ok(), "Gyroid lattice example should succeed");
}

#[test]
fn test_conformal_lattice_example() {
    // Test the actual example file
    let script = std::fs::read_to_string("examples/conformal_lattice.rhai")
        .expect("Should be able to read conformal_lattice.rhai");

    let result = evaluate_script(&script);
    assert!(result.is_ok(), "Conformal lattice example should succeed");
}
