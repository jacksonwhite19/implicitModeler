use glam::Vec3;
use implicit_cad::sdf::{Sdf, primitives::Sphere};
use serde_json::json;
use std::sync::Arc;
use v4_canonical_kernel::{MAX_BATCH, Registry};

#[test]
fn handle_lifetime_and_no_reuse() {
    let s: Arc<dyn Sdf> = Arc::new(Sphere::new(1.));
    let weak = Arc::downgrade(&s);
    let mut r = Registry::default();
    let h = r.admit(s, json!({"test":"sphere"}), 1.).unwrap();
    assert!(weak.upgrade().is_some());
    r.release(h).unwrap();
    assert!(weak.upgrade().is_none());
    assert_eq!(r.release(h).unwrap_err().code, "InvalidOrReleasedHandle");
    let next = r.admit(Arc::new(Sphere::new(2.)), json!({}), 1.).unwrap();
    assert_ne!(h, next);
    assert!(r.get(h).is_err());
}

#[test]
fn batch_input_and_atomic_output() {
    let mut r = Registry::default();
    let h = r.admit(Arc::new(Sphere::new(1.)), json!({}), 1.).unwrap();
    assert_eq!(r.get(h).unwrap().query(&[]).unwrap(), Vec::<f64>::new());
    assert_eq!(
        r.get(h)
            .unwrap()
            .query(&[[f64::NAN, 0., 0.]])
            .unwrap_err()
            .code,
        "NonfiniteCoordinate"
    );
    assert_eq!(
        r.get(h)
            .unwrap()
            .query(&[[f64::MAX, 0., 0.]])
            .unwrap_err()
            .code,
        "CoordinateOutOfKernelRange"
    );
    assert_eq!(
        r.get(h)
            .unwrap()
            .query(&vec![[0.; 3]; MAX_BATCH + 1])
            .unwrap_err()
            .code,
        "BatchTooLarge"
    );
    assert!(
        r.dispatch(json!({"op":"query","handle":h,"points_m":[[0,0]]}))
            .is_err()
    );
    assert!(
        r.dispatch(json!({"op":"query","handle":h,"points_m":null}))
            .is_err()
    );
    assert_eq!(r.get(h).unwrap().query(&[[0.; 3]]).unwrap(), vec![-1.]);
}

struct Fault(bool);
impl Sdf for Fault {
    fn distance(&self, _: Vec3) -> f32 {
        if self.0 {
            panic!("injected kernel failure")
        } else {
            f32::NAN
        }
    }
    fn evaluate_procedural_feature(
        &self,
        _: &str,
        _: f64,
    ) -> Option<implicit_cad::sdf::procedural_features::ProceduralFeatureQueryResult> {
        Some(
            implicit_cad::sdf::procedural_features::ProceduralFeatureQueryResult::unsupported(
                Vec3::splat(f32::NAN),
                "injected nonfinite feature result",
            ),
        )
    }
}
#[test]
fn kernel_failure_is_contained() {
    let mut r = Registry::default();
    let h = r
        .admit(Arc::new(Fault(false)), json!({"fault":true}), 1.)
        .unwrap();
    assert_eq!(
        r.get(h).unwrap().query(&[[0.; 3]]).unwrap_err().code,
        "NonfiniteFieldResult"
    );
    let h = r
        .admit(Arc::new(Fault(true)), json!({"fault":true}), 1.)
        .unwrap();
    let result = r.guarded(json!({"op":"query","handle":h,"points_m":[[0,0,0]]}));
    assert_eq!(result["error"]["code"], "KernelPanic");
    assert!(r.get(h).is_err());
}

#[test]
fn unit_conversion_and_capabilities() {
    let mut r = Registry::default();
    assert!(r.admit(Arc::new(Sphere::new(1.)), json!({}), 0.).is_err());
    let h = r
        .admit(Arc::new(Sphere::new(40.)), json!({}), 0.001)
        .unwrap();
    assert_eq!(
        r.get(h)
            .unwrap()
            .query(&[[0., 0., 0.], [0.04, 0., 0.]])
            .unwrap(),
        vec![-0.04, 0.]
    );
    assert_eq!(
        r.get(h).unwrap().description["capabilities"]["exact_distance"],
        "not_declared"
    );
    assert_eq!(
        r.get(h).unwrap().description["capabilities"]["projection"],
        "unsupported"
    );
}

#[test]
fn constructor_errors_and_cache_isolation() {
    let mut r = Registry::default();
    assert_eq!(
        r.load_rhai("not_a_shape()", 1.).unwrap_err().code,
        "ScriptConstructionFailed"
    );
    assert_eq!(
        r.load_rhai("sphere(1.0)", 1.).unwrap_err().code,
        "FreshHostRequired"
    );
    assert!(r.dispatch(json!({"op":"something_else"})).is_err());
}

struct BadBounds;
impl Sdf for BadBounds {
    fn distance(&self, _: Vec3) -> f32 {
        0.
    }
    fn metadata(&self) -> implicit_cad::sdf::conditioning::SdfNodeMetadata {
        use implicit_cad::sdf::conditioning::{SdfBounds, SdfNodeMetadata};
        SdfNodeMetadata::new("fault_injection").with_support_bounds(SdfBounds {
            min: Vec3::splat(f32::NAN),
            max: Vec3::ONE,
        })
    }
}
#[test]
fn invalid_bounds_fail_admission() {
    let mut r = Registry::default();
    assert_eq!(
        r.admit(Arc::new(BadBounds), json!({}), 1.)
            .unwrap_err()
            .code,
        "InvalidKernelBounds"
    );
}

#[test]
fn numerical_query_safety_and_zero_direction() {
    let mut r = Registry::default();
    let h = r.admit(Arc::new(Sphere::new(1.)), json!({}), 1.).unwrap();
    for op in ["numerical_project", "numerical_normal"] {
        let v = r
            .dispatch(json!({"op":op,"handle":h,"points_m":[[0,0,0]]}))
            .unwrap();
        assert_eq!(v["data"]["results"][0]["normal_status"], "zero_direction");
        assert_eq!(v["data"]["results"][0]["signed_field_at_return_m"], -1.);
        assert!(
            r.dispatch(json!({"op":op,"handle":h,"points_m":[[0,0]]}))
                .is_err()
        );
    }
    let g = r.get(h).unwrap();
    assert_eq!(
        g.numerical(&[[f64::NAN, 0., 0.]], true).unwrap_err().code,
        "NonfiniteCoordinate"
    );
    assert_eq!(
        g.numerical(&[[f64::MAX, 0., 0.]], true).unwrap_err().code,
        "CoordinateOutOfKernelRange"
    );
    assert_eq!(
        g.numerical(&vec![[0.; 3]; MAX_BATCH + 1], false)
            .unwrap_err()
            .code,
        "BatchTooLarge"
    );
    assert_eq!(
        g.evaluate_features(&[v4_canonical_kernel::queries::FeatureEvaluation {
            feature_id: "unknown".into(),
            parameter: f64::NAN
        }])
        .unwrap_err()
        .code,
        "InvalidFeatureParameter"
    );
    let h = r.admit(Arc::new(Fault(false)), json!({}), 1.).unwrap();
    assert_eq!(
        r.get(h)
            .unwrap()
            .numerical(&[[0.; 3]], true)
            .unwrap_err()
            .code,
        "NonfiniteQueryResult"
    );
    assert_eq!(r.dispatch(json!({"op":"feature_evaluate","handle":h,"requests":[{"feature_id":"fault","parameter":0.}]})).unwrap_err().code,"NonfiniteQueryResult");
    let h = r.admit(Arc::new(Fault(true)), json!({}), 1.).unwrap();
    assert_eq!(
        r.guarded(json!({"op":"numerical_project","handle":h,"points_m":[[0,0,0]]}))["error"]["code"],
        "KernelPanic"
    );
    assert!(r.get(h).is_err());
}
