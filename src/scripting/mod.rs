// Rhai scripting engine

use std::sync::{Arc, Mutex, RwLock};
use std::collections::HashMap;
use rhai::Engine;
use glam::Vec3;
use crate::sdf::Sdf;
use crate::sdf::field::Field;
use crate::sdf::aerospace::Section2D;
use crate::sdf::profiles::SplineProfile;
use crate::sdf::spine::LongitudinalSplines;
use crate::fea::FEASetup;

pub mod api;

/// Wrapper for SDF objects in Rhai scripts
#[derive(Clone)]
pub struct SdfHandle(pub Arc<dyn Sdf>);

/// Wrapper for Field objects in Rhai scripts
#[derive(Clone)]
pub struct FieldHandle(pub Arc<dyn Field>);

/// Handle to a 2-D cross-section (circle, ellipse, airfoil, etc.)
/// Used as input to `fuselage_station()` in scripts.
#[derive(Clone)]
pub struct SectionHandle(pub Arc<dyn Section2D>);

/// Handle to a positioned fuselage station.
/// Created by `fuselage_station(position, section)` in scripts.
#[derive(Clone)]
pub struct StationHandle {
    pub position: f32,
    pub section: Arc<dyn Section2D>,
}

/// An aircraft component: carries both its geometry SDF and a keepout (clearance) SDF.
/// Create with `component(sdf, margin)`, place with `place(comp, x, y, z)`,
/// extract with `geometry(comp)` or `keepout(comp)`.
#[derive(Clone)]
pub struct ComponentHandle {
    pub geometry: Arc<dyn Sdf>,
    pub keepout: Arc<dyn Sdf>,
    pub mass_g: f32,
    pub name: String,
}

/// A point mass declared in a script via `mass_at()` or `mass_named()`
#[derive(Clone, Debug)]
pub struct MassPoint {
    pub name: String,
    pub mass_g: f32,
    pub position: Vec3,
}

/// Result of evaluating a script — includes the SDF, declared mass points, and FEA conditions.
pub struct ScriptResult {
    pub sdf:         Arc<dyn Sdf>,
    pub mass_points: Vec<MassPoint>,
    pub fea_setup:   FEASetup,
}

impl ScriptResult {
    /// Compute the overall center of gravity from declared mass points.
    /// Returns None if no mass points were declared.
    pub fn center_of_gravity(&self) -> Option<Vec3> {
        let total_mass: f32 = self.mass_points.iter().map(|m| m.mass_g).sum();
        if total_mass <= 0.0 {
            return None;
        }
        let weighted: Vec3 = self.mass_points.iter()
            .map(|m| m.position * m.mass_g)
            .sum();
        Some(weighted / total_mass)
    }

    pub fn total_mass_g(&self) -> f32 {
        self.mass_points.iter().map(|m| m.mass_g).sum()
    }
}

/// Evaluate a script and return the resulting SDF plus any declared mass points.
///
/// Convenience wrapper — no profile, spine, or FEA field access.
pub fn evaluate_script(source: &str) -> Result<ScriptResult, String> {
    evaluate_script_full(source, None, None, None, None)
}

/// Convenience wrapper used by the app with profiles and spine but without FEA fields.
pub fn evaluate_script_with_profiles(
    source: &str,
    profiles: Option<Arc<RwLock<HashMap<String, SplineProfile>>>>,
    splines: Option<Arc<LongitudinalSplines>>,
) -> Result<ScriptResult, String> {
    evaluate_script_full(source, profiles, splines, None, None)
}

/// Full evaluator.
///
/// - `profiles`: `spline(name)` and `spline_section(name)` resolve named profiles.
/// - `splines`: `spline_fuselage(stations, length)` applies longitudinal spine constraints.
/// - `stress_field`: `stress_field()` returns a FieldHandle backed by this field.
/// - `displacement_field`: `displacement_field()` returns a FieldHandle backed by this field.
pub fn evaluate_script_full(
    source:             &str,
    profiles:           Option<Arc<RwLock<HashMap<String, SplineProfile>>>>,
    splines:            Option<Arc<LongitudinalSplines>>,
    stress_field:       Option<Arc<dyn Field>>,
    displacement_field: Option<Arc<dyn Field>>,
) -> Result<ScriptResult, String> {
    let mass_collector: Arc<Mutex<Vec<MassPoint>>>  = Arc::new(Mutex::new(Vec::new()));
    let fea_collector:  Arc<Mutex<FEASetup>>        = Arc::new(Mutex::new(FEASetup::default()));

    let mut engine = Engine::new();
    api::register_sdf_functions(&mut engine);
    api::register_component_functions(&mut engine, Arc::clone(&mass_collector));
    api::register_mass_functions(&mut engine, Arc::clone(&mass_collector));
    api::register_fea_functions(&mut engine, Arc::clone(&fea_collector), stress_field, displacement_field);
    if let Some(p) = profiles {
        api::register_profile_functions(&mut engine, p);
    }
    if let Some(s) = splines {
        api::register_spine_functions(&mut engine, s);
    }

    match engine.eval::<SdfHandle>(source) {
        Ok(handle) => {
            let mass_points = mass_collector.lock().unwrap().clone();
            let fea_setup   = std::mem::take(&mut *fea_collector.lock().unwrap());
            Ok(ScriptResult { sdf: handle.0, mass_points, fea_setup })
        }
        Err(e) => {
            let msg = e.to_string();
            if msg.contains("() (expecting") || msg.contains("output type incorrect") {
                Err("Script must end with an SDF expression (no semicolon on the last line).\nExample: the last line should be  `auto_fuselage(internals, 3.0)`  not  `auto_fuselage(internals, 3.0);`".to_string())
            } else {
                Err(format!("Script error: {}", e))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_evaluate_simple_sphere() {
        let script = "sphere(5.0)";
        let result = evaluate_script(script);
        assert!(result.is_ok(), "Simple sphere script should succeed");
    }

    #[test]
    fn test_evaluate_compound() {
        let script = r#"
            let base = box_(40.0, 20.0, 10.0);
            let hole = cylinder(4.0, 12.0);
            let hole = translate(hole, 10.0, 5.0, 0.0);
            let part = subtract(base, hole);
            part
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "Compound script should succeed");

        if let Ok(r) = result {
            let dist = r.sdf.distance(Vec3::new(0.0, 0.0, 0.0));
            assert!(dist < 0.0, "Origin should be inside the box");
        }
    }

    #[test]
    fn test_evaluate_error() {
        let script = "invalid_function()";
        let result = evaluate_script(script);
        assert!(result.is_err(), "Invalid script should return error");
    }

    #[test]
    fn test_mass_points_collected() {
        let script = r#"
            mass_at(150.0, 0.0, 0.0, 0.0);
            mass_at(25.0, 30.0, 0.0, 0.0);
            sphere(5.0)
        "#;
        let result = evaluate_script(script).unwrap();
        assert_eq!(result.mass_points.len(), 2);
        assert!((result.total_mass_g() - 175.0).abs() < 0.01);
        let cg = result.center_of_gravity().unwrap();
        // CG x = (150*0 + 25*30) / 175 = 750/175 ≈ 4.286
        assert!((cg.x - 4.286).abs() < 0.01, "CG x = {}", cg.x);
    }

    #[test]
    fn test_wing_with_airfoil_script() {
        let script = r#"
            let wing = wing_with_airfoil("2412", 12.0, 5.0, 40.0, 20.0, 3.0, -4.0);
            wing
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "wing_with_airfoil script should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        // Mid-chord of root section should be inside
        let d = sdf.distance(Vec3::new(6.0, 0.0, 0.0));
        assert!(d < 0.0, "Root mid-chord should be inside the wing, got {}", d);
    }

    #[test]
    fn test_fuselage_normalized() {
        // Unit fuselage: x ∈ [0,1]. Stations given out of order → auto-sorted.
        let script = r#"
            fuselage([
                [1.0,  circle_section(0.05)],
                [0.0,  circle_section(0.05)],
                [0.15, circle_section(0.5)],
                [0.85, circle_section(0.5)],
            ])
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "fuselage() should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        // Mid-body (x=0.5) should be inside
        let d = sdf.distance(Vec3::new(0.5, 0.0, 0.0));
        assert!(d < 0.0, "mid-body should be inside, got {}", d);
        // Far outside should be positive
        let d_out = sdf.distance(Vec3::new(0.5, 5.0, 0.0));
        assert!(d_out > 0.0, "far outside should be positive, got {}", d_out);
    }

    #[test]
    fn test_fuselage_out_of_range_error() {
        let script = r#"
            fuselage([[0.0, circle_section(0.5)], [2.0, circle_section(0.1)]])
        "#;
        let result = evaluate_script(script);
        assert!(result.is_err(), "position > 1.0 should return an error");
    }

    #[test]
    fn test_lofted_fuselage_script() {
        // Multi-station fuselage: nose at x=0 (r=0.2), body at x=3 (r=1), tail at x=8 (r=0.3)
        let script = r#"
            let nose  = fuselage_station(0.0, circle_section(0.2));
            let body  = fuselage_station(3.0, circle_section(1.0));
            let tail  = fuselage_station(8.0, ellipse_section(0.5, 0.3));
            lofted_fuselage([nose, body, tail])
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "lofted_fuselage script should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        // Point at mid-body (x=4, y=0, z=0) should be inside
        let d = sdf.distance(Vec3::new(4.0, 0.0, 0.0));
        assert!(d < 0.0, "Mid-body should be inside, got {}", d);
        // Point far above mid-body should be outside
        let d_out = sdf.distance(Vec3::new(4.0, 5.0, 0.0));
        assert!(d_out > 0.0, "Point far above should be outside, got {}", d_out);
    }

    #[test]
    fn test_lofted_fuselage_unsorted_error() {
        // lofted_fuselage() still validates order (legacy API)
        let script = r#"
            let a = fuselage_station(5.0, circle_section(1.0));
            let b = fuselage_station(2.0, circle_section(0.5));
            lofted_fuselage([a, b])
        "#;
        let result = evaluate_script(script);
        assert!(result.is_err(), "lofted_fuselage with unsorted stations should return an error");
    }

    #[test]
    fn test_lofted_fuselage_too_few_error() {
        let script = r#"
            let a = fuselage_station(0.0, circle_section(1.0));
            lofted_fuselage([a])
        "#;
        let result = evaluate_script(script);
        assert!(result.is_err(), "Fewer than 2 stations should return an error");
    }

    // --- Structural primitive integration tests ---

    #[test]
    fn test_rib_at_station() {
        // Root rib (y=0) on a 2412 wing; root chord 12, half-span 20.
        // Interior point at (6, 0, 0) should be inside the rib.
        // Point at (6, 5, 0) is outside the rib thickness (0.5) but still in the wing —
        // the intersection should exclude it.
        let script = r#"
            let wing = wing_with_airfoil("2412", 12.0, 5.0, 40.0, 0.0, 0.0, 0.0);
            rib_at_station(wing, 0.0, 0.5)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "rib_at_station should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        let d_inside = sdf.distance(Vec3::new(6.0, 0.0, 0.0));
        assert!(d_inside < 0.0, "root rib centre should be inside, got {}", d_inside);
        let d_outside_span = sdf.distance(Vec3::new(6.0, 5.0, 0.0));
        assert!(d_outside_span > 0.0, "point 5 units from rib should be outside, got {}", d_outside_span);
    }

    #[test]
    fn test_spar() {
        // Front spar at 25% of root chord (x=3), radius 0.4.
        // Point on spar axis at root should be inside; point far off-axis should be outside.
        let script = r#"
            let wing = wing_with_airfoil("2412", 12.0, 5.0, 40.0, 0.0, 0.0, 0.0);
            spar(wing, 3.0, 0.4)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "spar should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        let d_on_axis = sdf.distance(Vec3::new(3.0, 0.0, 0.0));
        assert!(d_on_axis < 0.0, "spar axis at root should be inside, got {}", d_on_axis);
        let d_off_axis = sdf.distance(Vec3::new(3.0, 0.0, 5.0));
        assert!(d_off_axis > 0.0, "point well above spar should be outside, got {}", d_off_axis);
    }

    #[test]
    fn test_two_ribs_union() {
        // Two ribs at different stations unioned together.
        let script = r#"
            let wing = wing_with_airfoil("0012", 10.0, 5.0, 30.0, 0.0, 0.0, 0.0);
            let rib1 = rib_at_station(wing, 0.0, 0.4);
            let rib2 = rib_at_station(wing, 6.0, 0.4);
            union(rib1, rib2)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "union of two ribs should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        // Both rib centres should be inside the union
        assert!(sdf.distance(Vec3::new(5.0, 0.0, 0.0)) < 0.0, "rib1 centre should be inside");
        assert!(sdf.distance(Vec3::new(5.0, 6.0, 0.0)) < 0.0, "rib2 centre should be inside");
        // Gap between ribs should be outside
        assert!(sdf.distance(Vec3::new(5.0, 3.0, 0.0)) > 0.0, "gap between ribs should be outside");
    }

    #[test]
    fn test_rib_spar_composition() {
        // Full structural assembly: two ribs + front spar, all unioned.
        let script = r#"
            let wing = wing_with_airfoil("0012", 10.0, 5.0, 30.0, 0.0, 0.0, 0.0);
            let rib1     = rib_at_station(wing, 0.0, 0.4);
            let rib2     = rib_at_station(wing, 6.0, 0.4);
            let front_sp = spar(wing, 2.5, 0.35);
            let structure = union(union(rib1, rib2), front_sp);
            structure
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "rib+spar assembly should succeed: {:?}", result.err());
        // The result must be a valid SDF (can be further composed)
        let sdf = result.unwrap().sdf;
        // At least one of the three parts must register as interior at a known-interior point
        let d = sdf.distance(Vec3::new(2.5, 0.0, 0.0));
        assert!(d < 0.0, "spar axis at root should be inside the assembly, got {}", d);
    }

    #[test]
    fn test_airfoil_from_points_script() {
        // Minimal diamond-shaped airfoil: 4 pts that form a closed loop
        let script = r#"
            let pts = [
                [0.0, 0.0],
                [0.5, 0.1],
                [1.0, 0.0],
                [0.5, -0.1],
                [0.0, 0.0],
            ];
            let af = airfoil_from_points(pts, 10.0);
            let wing = wing_with_airfoil(af, af, 20.0, 0.0, 0.0, 0.0);
            wing
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "airfoil_from_points wing should succeed: {:?}", result.err());
    }

    #[test]
    fn test_airfoil_from_points_too_few_error() {
        let script = r#"
            let pts = [[0.0, 0.0], [1.0, 0.0]];
            airfoil_from_points(pts, 5.0)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_err(), "Fewer than 3 points should return an error");
    }

    #[test]
    fn test_auto_fuselage() {
        let script = r#"
            let internals = box_(8.0, 4.0, 3.0);
            auto_fuselage(internals, 2.0)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "auto_fuselage should succeed");
        // Outer skin should be larger than inner box
        let sdf = result.unwrap().sdf;
        // box_ half-extent = 4.0, skin = 2.0 → outer surface at x=6.0
        // At x=5.5 we should be inside the skin
        let dist = sdf.distance(Vec3::new(5.5, 0.0, 0.0));
        assert!(dist < 0.0, "Point inside skin should be interior, got {}", dist);
    }

    // ── Drone structural primitive integration tests ───────────────────────────

    /// Helper script fragment that builds a normalised test fuselage.
    fn fuse_script() -> &'static str {
        r#"let fuse = fuselage([
            [0.0, circle_section(0.3)],
            [0.5, circle_section(1.0)],
            [1.0, circle_section(0.3)],
        ]);"#
    }

    #[test]
    fn test_bulkhead_at_station_script() {
        let script = format!(r#"
            {}
            bulkhead_at_station(fuse, 0.5, 0.02, 0, 0.0)
        "#, fuse_script());
        let result = evaluate_script(&script);
        assert!(result.is_ok(), "bulkhead_at_station should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        // Centre of bulkhead at x=0.5 must be inside.
        assert!(sdf.distance(Vec3::new(0.5, 0.0, 0.0)) < 0.0);
        // Off-station must be outside.
        assert!(sdf.distance(Vec3::new(0.9, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn test_bulkhead_with_holes_script() {
        let script = format!(r#"
            {}
            bulkhead_at_station(fuse, 0.5, 0.02, 6, 0.3)
        "#, fuse_script());
        let result = evaluate_script(&script);
        assert!(result.is_ok(), "bulkhead_at_station with holes should succeed: {:?}", result.err());
    }

    #[test]
    fn test_lightening_hole_pattern_script() {
        let script = format!(r#"
            {}
            let bk = bulkhead_at_station(fuse, 0.5, 0.02, 0, 0.0);
            lightening_hole_pattern(bk, 4, 0.5, 0.1, 0)
        "#, fuse_script());
        let result = evaluate_script(&script);
        assert!(result.is_ok(), "lightening_hole_pattern should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        // Hole centre at (0.5, 0.5, 0) should be outside after drilling.
        assert!(sdf.distance(Vec3::new(0.5, 0.5, 0.0)) > 0.0,
            "hole location should be outside");
    }

    #[test]
    fn test_rod_mount_script() {
        let script = format!(r#"
            {}
            let bk = bulkhead_at_station(fuse, 0.5, 0.02, 0, 0.0);
            rod_mount(bk, 0.0, 0.6, 0.04, 0.08)
        "#, fuse_script());
        let result = evaluate_script(&script);
        assert!(result.is_ok(), "rod_mount should succeed: {:?}", result.err());
    }

    #[test]
    fn test_motor_arm_script() {
        let script = format!(r#"
            {}
            motor_arm(fuse, 0.0, 2.0, 0.12, 0.09)
        "#, fuse_script());
        let result = evaluate_script(&script);
        assert!(result.is_ok(), "motor_arm should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        // Tube wall at midspan: outer_r=0.06, Z=0.05 is in the wall (0.045..0.06).
        assert!(sdf.distance(Vec3::new(0.5, 2.5, 0.05)) < 0.0,
            "arm tube wall should be inside");
        // Hollow axis should be outside the tube.
        assert!(sdf.distance(Vec3::new(0.5, 2.5, 0.0)) > 0.0,
            "arm hollow axis should be outside");
        // Far beyond arm tip should be outside.
        assert!(sdf.distance(Vec3::new(0.5, 10.0, 0.0)) > 0.0,
            "beyond arm tip should be outside");
    }

    #[test]
    fn test_motor_mount_script() {
        let script = format!(r#"
            {}
            let arm = motor_arm(fuse, 0.0, 2.0, 0.12, 0.09);
            motor_mount(arm, 0.3, 0.05, 0.12, 0.03)
        "#, fuse_script());
        let result = evaluate_script(&script);
        assert!(result.is_ok(), "motor_mount should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        // Plate at y ≈ 3.0, half_side=0.15, thickness=0.05. Point at y=2.98 should be inside.
        assert!(sdf.distance(Vec3::new(0.5, 2.98, 0.0)) < 0.0,
            "motor mount plate interior should be inside");
    }

    #[test]
    fn test_generate_mounts_script() {
        let script = format!(r#"
            {}
            let battery = component_named("battery", box_(0.4, 0.2, 0.15), 0.02, 50.0);
            let battery_placed = place(battery, 0.5, 0.0, -0.3);
            generate_mounts([battery_placed], fuse, 0.015, 0.04)
        "#, fuse_script());
        let result = evaluate_script(&script);
        assert!(result.is_ok(), "generate_mounts should succeed: {:?}", result.err());
    }

    #[test]
    fn test_mount_with_bolts_script() {
        let script = format!(r#"
            {}
            let esc = component(box_(0.1, 0.05, 0.02), 0.01);
            let esc_placed = place(esc, 0.5, 0.0, -0.2);
            mount_with_bolts(esc_placed, fuse, 0.01, 0.03, 0.02, 4)
        "#, fuse_script());
        let result = evaluate_script(&script);
        assert!(result.is_ok(), "mount_with_bolts should succeed: {:?}", result.err());
    }

    #[test]
    fn test_drone_full_example_script() {
        // Full workflow: fuselage + two bulkheads + two arms + motor mount + battery tray.
        let script = r#"
            let fuse = fuselage([
                [0.0, circle_section(0.3)],
                [0.5, circle_section(1.0)],
                [1.0, circle_section(0.3)],
            ]);
            let frame_a = bulkhead_at_station(fuse, 0.3, 0.02, 8, 0.6);
            let frame_b = bulkhead_at_station(fuse, 0.6, 0.02, 8, 0.6);
            let arm_0   = motor_arm(fuse, 0.0, 2.0, 0.12, 0.09);
            let arm_90  = motor_arm(fuse, 90.0, 2.0, 0.12, 0.09);
            let mount_0 = motor_mount(arm_0, 0.3, 0.05, 0.12, 0.03);
            let battery = component_named("battery", box_(0.4, 0.2, 0.15), 0.02, 180.0);
            let battery_placed = place(battery, 0.5, 0.0, -0.3);
            let battery_mount  = generate_mounts([battery_placed], fuse, 0.015, 0.04);
            union(union(union(union(union(fuse, frame_a), frame_b), arm_90), mount_0), battery_mount)
        "#;
        let result = evaluate_script(script);
        assert!(result.is_ok(), "full drone script should succeed: {:?}", result.err());
        let sdf = result.unwrap().sdf;
        // Fuselage mid-body must be inside the combined geometry.
        assert!(sdf.distance(Vec3::new(0.5, 0.0, 0.0)) < 0.0,
            "fuselage mid-body should be inside");
    }
}
