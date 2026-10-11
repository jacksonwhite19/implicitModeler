//! Direct-call oracle. Retained constructors/points only, no geometry algorithms.
use glam::Vec3;
use implicit_cad::sdf::{procedural_features::ProceduralFeatureQueryOptions, query};
use serde_json::{Value, json};
use std::{fs, time::Instant};
use v4_canonical_kernel::Registry;

fn main() {
    let args: Vec<_> = std::env::args().collect();
    let input: Value = serde_json::from_str(&fs::read_to_string(&args[1]).unwrap()).unwrap();
    let case = input["cases"]
        .as_array()
        .unwrap()
        .iter()
        .find(|c| c["name"] == args[2])
        .unwrap();
    let s = case["metres_per_model_unit"].as_f64().unwrap();
    let mut r = Registry::default();
    let start = Instant::now();
    let h = r.load_rhai(case["script"].as_str().unwrap(), s).unwrap();
    let load_seconds = start.elapsed().as_secs_f64();
    let g = r.get(h).unwrap();
    assert_eq!(g.description, case["description"]);
    let sdf = g.canonical.as_ref();
    let mut points: Vec<[f64; 3]> = serde_json::from_value(case["points_m"].clone()).unwrap();
    let inventory = sdf.procedural_feature_inventory();
    let mut evaluations = vec![];
    let mut feature_points = vec![];
    if let Some(i) = &inventory {
        for f in &i.features {
            for parameter in [0., 0.25, 0.5, 0.75, 1.] {
                let value = sdf.evaluate_procedural_feature(&f.feature_id, parameter);
                if let Some(p) = value.as_ref().and_then(|v| v.closest_point) {
                    feature_points.push(p.map(|v| v * s));
                }
                evaluations
                    .push(json!({"feature_id":f.feature_id,"parameter":parameter,"model":value}));
            }
        }
    }
    let feature_point_count = feature_points.len();
    if args[2] == "plain_wing" {
        let retained: Value = serde_json::from_str(&fs::read_to_string(&args[3]).unwrap()).unwrap();
        for row in retained["polygon_midpoints"].as_array().unwrap() {
            let p: [f64; 3] = serde_json::from_value(row["midpoint_model"].clone()).unwrap();
            points.push(p.map(|v| v * s));
        }
    }
    feature_points.extend(&points);
    let qs: Vec<_> = points
        .iter()
        .map(|p| Vec3::from_array(p.map(|v| (v / s) as f32)))
        .collect();
    let t = Instant::now();
    let scalar: Vec<_> = qs.iter().map(|q| f64::from(sdf.distance(*q)) * s).collect();
    let scalar_seconds = t.elapsed().as_secs_f64();
    let t = Instant::now();
    let normal: Vec<_> = qs
        .iter()
        .map(|q| query::gradient(sdf, *q).to_array().map(f64::from))
        .collect();
    let normal_seconds = t.elapsed().as_secs_f64();
    let t = Instant::now();
    let projected: Vec<_> = qs
        .iter()
        .map(|q| {
            let p = query::closest_point(sdf, *q);
            json!({"returned_point_model_f32":p.to_array().map(f64::from),
            "returned_point_m":p.to_array().map(|v|f64::from(v)*s),
            "normal_direction":query::gradient(sdf,p).to_array().map(f64::from),
            "signed_field_at_return_m":f64::from(sdf.distance(p))*s})
        })
        .collect();
    let project_seconds = t.elapsed().as_secs_f64();
    let fqs: Vec<_> = feature_points
        .iter()
        .map(|p| Vec3::from_array(p.map(|v| (v / s) as f32)))
        .collect();
    let batch =
        sdf.query_procedural_feature_batch(&fqs, &ProceduralFeatureQueryOptions::diagnostic());
    // Repeated work supplies a non-single-call timing, without changing controls.
    let t = Instant::now();
    for _ in 0..20 {
        for q in &qs {
            std::hint::black_box(query::closest_point(sdf, *q));
        }
    }
    let repeated_projection_seconds = t.elapsed().as_secs_f64();
    println!(
        "{}",
        json!({"description":g.description,"points_m":points,"scalar":scalar,"normal":normal,
        "projected":projected,"inventory":inventory,"evaluations":evaluations,"feature_points_m":feature_points,
        "feature_point_count":feature_point_count,"batch":batch,"load_seconds":load_seconds,
        "scalar_seconds":scalar_seconds,"normal_seconds":normal_seconds,"project_seconds":project_seconds,
        "repeated_projection_count":20*qs.len(),"repeated_projection_seconds":repeated_projection_seconds})
    );
}
