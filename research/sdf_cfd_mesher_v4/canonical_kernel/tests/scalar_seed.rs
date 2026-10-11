use glam::Vec3;
use implicit_cad::{mesh::marching_cubes::{extract_mesh_canonical_edges,
    extract_scalar_seed_canonical_edges}, sdf::{Sdf, primitives::Sphere,
    transforms::{Scale, Translate}, booleans::SmoothUnion,
    conditioning::{SdfBounds, SdfNodeMetadata}}};
use serde_json::{json, Value};
use std::sync::{Arc, atomic::{AtomicU64, Ordering}};
use v4_canonical_kernel::Registry;
use implicit_cad::mesh::sdf_surface::{weld_and_clean_seed_mesh, SurfaceExtractionConfig};

struct Counted { field: Arc<dyn Sdf>, calls: AtomicU64 }
impl Sdf for Counted {
    fn distance(&self, p: Vec3) -> f32 {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.field.distance(p)
    }
}

#[test]
fn scalar_seed_preserves_stock_positions_indices_and_normal_bearing_entry() {
    let sphere: Arc<dyn Sdf> = Arc::new(Sphere::new(1.));
    let ellipsoid: Arc<dyn Sdf> = Arc::new(Scale::new(sphere.clone(), Vec3::new(1.5, 1., 0.75)));
    let blend: Arc<dyn Sdf> = Arc::new(SmoothUnion::new(ellipsoid.clone(),
        Arc::new(Translate::new(sphere.clone(), Vec3::new(0.4, 0.5, 0.))), 0.2));
    for field in [sphere, ellipsoid, blend] {
        let counted = Counted { field, calls: AtomicU64::new(0) };
        let stock = extract_mesh_canonical_edges(&counted, Vec3::splat(-2.), Vec3::splat(2.), 12, false);
        assert_eq!(counted.calls.swap(0, Ordering::Relaxed), 7 * 13_u64.pow(3));
        let scalar = extract_scalar_seed_canonical_edges(&counted, Vec3::splat(-2.), Vec3::splat(2.), 12);
        assert_eq!(counted.calls.load(Ordering::Relaxed), 13_u64.pow(3));
        assert_eq!(stock.indices, scalar.indices);
        assert_eq!(stock.vertices.len(), scalar.vertices.len());
        for (a, b) in stock.vertices.iter().zip(&scalar.vertices) {
            assert_eq!(a.position.map(f32::to_bits), b.position.map(f32::to_bits));
            assert_eq!(b.normal, [0.; 3]);
        }
        assert!(stock.vertices.iter().any(|v| v.normal != [0.; 3]));
        let controls=SurfaceExtractionConfig::default();
        let a=weld_and_clean_seed_mesh(&stock,&controls);
        let b=weld_and_clean_seed_mesh(&scalar,&controls);
        assert_eq!(a.vertices,b.vertices);
        assert_eq!(a.triangles,b.triangles);
        assert_eq!(a.cleanup,b.cleanup);
        assert_eq!(counted.calls.load(Ordering::Relaxed),13_u64.pow(3));
    }
}

struct Fault { value: f32 }
impl Sdf for Fault {
    fn distance(&self, _: Vec3) -> f32 { self.value }
    fn metadata(&self) -> SdfNodeMetadata {
        SdfNodeMetadata::new("test_field").with_support_bounds(SdfBounds::from_center_radius(Vec3::ZERO, 1.))
    }
}

fn diagnostic(error: &v4_canonical_kernel::Failure) -> Value {
    serde_json::from_str::<Value>(&error.detail).unwrap()["seed_diagnostics"].clone()
}

#[test]
fn failures_retain_exact_attempt_counts_without_accepting_proposal() {
    for (value, code) in [(f32::NAN, "NonfiniteFieldResult"), (1., "InvalidSurfaceSeedOutput")] {
        let mut registry = Registry::default();
        let h = registry.admit(Arc::new(Fault { value }), json!({}), 1.).unwrap();
        let error = registry.get(h).unwrap().surface_seed(0.1, 0.3, 729, 30.).unwrap_err();
        assert_eq!(error.code, code);
        let d = diagnostic(&error);
        assert_eq!(d["scalar_grid_attempts"], 729);
        assert_eq!(d["scalar_field_evaluations"], 729);
        assert_eq!(d["conditioning_attempts"], 0);
        assert_eq!(d["normal_stencil_attempts"], 0);
        assert_eq!(d["projection_attempts"], 0);
        assert_eq!(d["geometry_acceptance"], false);
    }
}

#[test]
fn query_preflight_and_deadline_failures_are_distinct_and_counted() {
    let mut registry = Registry::default();
    let h = registry.admit(Arc::new(Sphere::new(1.)), json!({}), 1.).unwrap();
    let g = registry.get(h).unwrap();
    let budget = g.surface_seed(0.1, 0.3, 728, 30.).unwrap_err();
    assert_eq!(budget.code, "QueryBudgetExceeded");
    assert_eq!(diagnostic(&budget)["scalar_grid_attempts"], 0);
    assert_eq!(diagnostic(&budget)["requested_scalar_grid_attempts"], 729);
    let timeout = g.surface_seed(0.1, 0.3, 729, 1e-30).unwrap_err();
    assert_eq!(timeout.code, "HostTimeout");
    assert_eq!(diagnostic(&timeout)["scalar_grid_attempts"], 729);
    assert_eq!(diagnostic(&timeout)["scalar_field_evaluations"], 0);
    assert_eq!(diagnostic(&timeout)["timed_out"], true);
    assert_eq!(g.query(&[[0.; 3]]).unwrap(), vec![-1.]);
}

#[test]
fn scalar_seed_exact_budget_and_live_generic_source_binding() {
    for source in ["sphere(1.0)", "translate(scale(sphere(1.0), 1.5, 1.0, 0.75), 0.2, -0.3, 0.4)"] {
        let mut registry = Registry::default();
        let h = registry.load_rhai(source, 0.001).unwrap();
        let g = registry.get(h).unwrap();
        let p = g.surface_seed(0.0001, 0.0005, 729, 30.).unwrap();
        let d = &p["data"];
        assert_eq!(p["geometry_identity"], g.description["geometry_identity"]);
        assert_eq!(d["scalar_evaluations"], 729);
        assert_eq!(d["stage_counts"]["scalar_grid_attempts"], 729);
        assert_eq!(d["producer"], "V4StockScalarConstructionSeed/1");
        assert_eq!(d["seed_projection"], "NotRun");
        assert!(!d["coordinates_m"].as_array().unwrap().is_empty());
        assert!(!d["triangles"].as_array().unwrap().is_empty());
    }
}
