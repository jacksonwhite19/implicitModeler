// Integration tests for Phase 11: Pattern Operations

use implicit_cad::scripting;
use implicit_cad::sdf::Sdf;
use glam::Vec3;
use std::sync::Arc;

fn eval(script: &str) -> Arc<dyn Sdf> {
    scripting::evaluate_script(script).expect("Script should evaluate successfully").sdf
}

// ─── linear_array ────────────────────────────────────────────────────────────

#[test]
fn test_linear_array_x() {
    let sdf = eval("linear_array(sphere(3.0), 4, 10.0, 0.0, 0.0)");
    // Centers at x = 0, 10, 20, 30
    for x in [0.0_f32, 10.0, 20.0, 30.0] {
        assert!(sdf.distance(Vec3::new(x, 0.0, 0.0)) < 0.0,
            "should be inside copy at x={}", x);
    }
    assert!(sdf.distance(Vec3::new(50.0, 0.0, 0.0)) > 0.0,
        "should be outside past last copy");
}

#[test]
fn test_linear_array_y() {
    let sdf = eval("linear_array(sphere(2.0), 3, 0.0, 8.0, 0.0)");
    for y in [0.0_f32, 8.0, 16.0] {
        assert!(sdf.distance(Vec3::new(0.0, y, 0.0)) < 0.0,
            "should be inside copy at y={}", y);
    }
}

#[test]
fn test_linear_array_count_one() {
    let arr = eval("linear_array(sphere(5.0), 1, 20.0, 0.0, 0.0)");
    let single = eval("sphere(5.0)");
    let p = Vec3::new(0.0, 3.0, 0.0);
    assert!((arr.distance(p) - single.distance(p)).abs() < 1e-4,
        "count=1 should match single shape");
}

// ─── polar_array ─────────────────────────────────────────────────────────────

#[test]
fn test_polar_array_6_around_z() {
    // 6 boxes offset from Z axis — all 6 positions should be inside a copy
    let sdf = eval("
        let b = translate(box_(3.0, 3.0, 6.0), 12.0, 0.0, 0.0);
        polar_array(b, 6)
    ");

    let r = 12.0_f32;
    let angles = [0.0_f32, 60.0, 120.0, 180.0, 240.0, 300.0];
    for deg in angles {
        let rad = deg.to_radians();
        let p = Vec3::new(r * rad.cos(), r * rad.sin(), 0.0);
        assert!(sdf.distance(p) < 0.0,
            "should be inside copy at {}°", deg);
    }
}

#[test]
fn test_polar_array_symmetry() {
    // A single offset sphere polar-arrayed by 4 should be symmetric
    let sdf = eval("
        let s = translate(sphere(3.0), 8.0, 0.0, 0.0);
        polar_array(s, 4)
    ");
    let r = 8.0_f32;
    let d0 = sdf.distance(Vec3::new(r, 0.0, 0.0));
    let d1 = sdf.distance(Vec3::new(-r, 0.0, 0.0));
    let d2 = sdf.distance(Vec3::new(0.0, r, 0.0));
    let d3 = sdf.distance(Vec3::new(0.0, -r, 0.0));
    assert!((d0 - d1).abs() < 1e-4);
    assert!((d0 - d2).abs() < 1e-4);
    assert!((d0 - d3).abs() < 1e-4);
}

// ─── mirror ──────────────────────────────────────────────────────────────────

#[test]
fn test_mirror_x() {
    let sdf = eval("mirror_x(translate(sphere(4.0), 10.0, 0.0, 0.0))");
    // Original at +X
    assert!(sdf.distance(Vec3::new(10.0, 0.0, 0.0)) < 0.0);
    // Mirror at -X
    assert!(sdf.distance(Vec3::new(-10.0, 0.0, 0.0)) < 0.0);
    // Far away
    assert!(sdf.distance(Vec3::new(0.0, 30.0, 0.0)) > 0.0);
}

#[test]
fn test_mirror_y() {
    let sdf = eval("mirror_y(translate(sphere(4.0), 0.0, 10.0, 0.0))");
    assert!(sdf.distance(Vec3::new(0.0, 10.0, 0.0)) < 0.0);
    assert!(sdf.distance(Vec3::new(0.0, -10.0, 0.0)) < 0.0);
}

#[test]
fn test_mirror_z() {
    let sdf = eval("mirror_z(translate(sphere(4.0), 0.0, 0.0, 10.0))");
    assert!(sdf.distance(Vec3::new(0.0, 0.0, 10.0)) < 0.0);
    assert!(sdf.distance(Vec3::new(0.0, 0.0, -10.0)) < 0.0);
}

// ─── math extras ─────────────────────────────────────────────────────────────

#[test]
fn test_pi_constant() {
    let sdf = eval("sphere(PI * 3.0)");
    // PI * 3 ≈ 9.42, center should be inside
    assert!(sdf.distance(Vec3::ZERO) < 0.0);
}

#[test]
fn test_to_rad_to_deg() {
    let sdf = eval("
        let angle_deg = 90.0;
        let angle_rad = to_rad(angle_deg);
        sphere(angle_rad * 2.0)
    ");
    assert!(sdf.distance(Vec3::ZERO) < 0.0);
}

#[test]
fn test_clamp_lerp() {
    // clamp(15, 0, 10) = 10 → sphere(10)
    let sdf = eval("sphere(clamp(15.0, 0.0, 10.0))");
    assert!(sdf.distance(Vec3::new(9.0, 0.0, 0.0)) < 0.0);
    assert!(sdf.distance(Vec3::new(11.0, 0.0, 0.0)) > 0.0);

    // lerp(0, 20, 0.5) = 10 → sphere(10)
    let sdf2 = eval("sphere(lerp(0.0, 20.0, 0.5))");
    assert!(sdf2.distance(Vec3::new(9.0, 0.0, 0.0)) < 0.0);
    assert!(sdf2.distance(Vec3::new(11.0, 0.0, 0.0)) > 0.0);
}

// ─── complex compositions ─────────────────────────────────────────────────────

#[test]
fn test_pattern_combined_with_boolean() {
    // Plate with a polar array of holes
    let sdf = eval("
        let plate = cylinder(20.0, 5.0);
        let hole = translate(cylinder(2.0, 6.0), 12.0, 0.0, 0.0);
        let holes = polar_array(hole, 6);
        subtract(plate, holes)
    ");
    // Center of plate should be inside
    assert!(sdf.distance(Vec3::ZERO) < 0.0);
    // A hole position should be outside (removed)
    assert!(sdf.distance(Vec3::new(12.0, 0.0, 0.0)) > 0.0);
}

#[test]
fn test_user_function_with_pattern() {
    let sdf = eval("
        fn spoke(angle_deg) {
            let c = translate(cylinder(1.5, 18.0), 9.0, 0.0, 0.0);
            rotate(c, 0.0, 0.0, angle_deg)
        }
        let hub = cylinder(4.0, 10.0);
        let s0 = spoke(0.0);
        let s1 = spoke(60.0);
        let s2 = spoke(120.0);
        let wheel = union(hub, union(s0, union(s1, s2)));
        wheel
    ");
    assert!(sdf.distance(Vec3::ZERO) < 0.0);
}

#[test]
fn test_example_parametric_bracket() {
    let script = std::fs::read_to_string("examples/parametric_bracket.rhai")
        .expect("parametric_bracket.rhai should exist");
    let result = scripting::evaluate_script(&script);
    assert!(result.is_ok(), "parametric_bracket.rhai failed: {:?}", result.err());
}

#[test]
fn test_example_symmetric_wing() {
    let script = std::fs::read_to_string("examples/symmetric_wing.rhai")
        .expect("symmetric_wing.rhai should exist");
    let result = scripting::evaluate_script(&script);
    assert!(result.is_ok(), "symmetric_wing.rhai failed: {:?}", result.err());
}
