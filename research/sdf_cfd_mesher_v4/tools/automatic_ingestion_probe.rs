//! Audit only: calls existing canonical, feature and query implementations.
use glam::Vec3;
use implicit_cad::sdf::{aerospace::{wing::LoftedWing,airfoil::Airfoil}, procedural_features::ProceduralFeatureQueryOptions, query};
use serde_json::{Value,json};
use std::{fs,time::Instant};
use v4_canonical_kernel::Registry;

fn main() {
    let input:Value=serde_json::from_str(&fs::read_to_string(std::env::args().nth(1).unwrap()).unwrap()).unwrap();
    let case=input["cases"].as_array().unwrap().iter().find(|c|c["name"]=="plain_wing").unwrap();
    let start=Instant::now();let mut registry=Registry::default();
    let handle=registry.load_rhai(case["script"].as_str().unwrap(),case["metres_per_model_unit"].as_f64().unwrap()).unwrap();
    let load_seconds=start.elapsed().as_secs_f64();let g=registry.get(handle).unwrap();
    assert_eq!(g.description,case["description"]);
    let sdf=g.canonical.as_ref();let inventory=sdf.procedural_feature_inventory().unwrap();
    let points:Vec<[f64;3]>=serde_json::from_value(case["points_m"].clone()).unwrap();
    let direct:Vec<f64>=serde_json::from_value(case["direct_m"].clone()).unwrap();
    let values=g.query(&points).unwrap();assert!(values.iter().zip(&direct).all(|(a,b)|a.to_bits()==b.to_bits()));
    let mut evaluations=Vec::new();let mut eval_points=Vec::new();
    for feature in &inventory.features {for parameter in [0.,0.25,0.5,0.75,1.] {
        let value=sdf.evaluate_procedural_feature(&feature.feature_id,parameter);
        if let Some(p)=value.as_ref().and_then(|v|v.closest_point) {eval_points.push(Vec3::from_array(p.map(|v|v as f32)));}
        evaluations.push(json!({"feature":feature.feature_id,"parameter":parameter,"result":value}));
    }}
    let options=ProceduralFeatureQueryOptions::diagnostic();
    let t=Instant::now();let batch=sdf.query_procedural_feature_batch(&eval_points,&options);let feature_seconds=t.elapsed().as_secs_f64();
    let repeat=sdf.query_procedural_feature_batch(&eval_points,&options);assert_eq!(batch.results,repeat.results);
    let t=Instant::now();let projections:Vec<_>=points.iter().map(|p| {
        let q=Vec3::from_array(p.map(|v|(v/g.metres_per_model_unit) as f32));
        let projected=query::closest_point(sdf,q);let normal=query::gradient(sdf,projected);
        json!({"input_model":q.to_array(),"projected_model":projected.to_array(),
            "residual_model":sdf.distance(projected),"normal":normal.to_array(),
            "finite":projected.is_finite()&&normal.is_finite(),"zero_normal":normal.length_squared()==0.})
    }).collect();let projection_seconds=t.elapsed().as_secs_f64();
    // Inspect stored polygon coordinates; no new airfoil or surface construction.
    let wing=sdf.as_any().downcast_ref::<LoftedWing>().unwrap();let section=&wing.sections[wing.sections.len()/2];
    assert_eq!(section.twist,0.);let airfoil=section.section.as_any().downcast_ref::<Airfoil>().unwrap();
    let mut polygon_probes=Vec::new();
    for (i,a) in airfoil.points.iter().enumerate() {
        let b=airfoil.points[(i+1)%airfoil.points.len()];
        let q=section.position+Vec3::new((a.0+b.0)*0.5*airfoil.chord,0.,(a.1+b.1)*0.5*airfoil.chord);
        let r=sdf.query_procedural_feature(q,&options);
        polygon_probes.push(json!({"edge":i,"section_a":a,"section_b":b,"midpoint_model":q.to_array(),
            "scalar_model":sdf.distance(q),"result":r}));
    }
    let t=Instant::now();let mut checksum=0.;for _ in 0..100 {for v in g.query(&points).unwrap(){checksum+=v;}}
    let scalar_seconds=t.elapsed().as_secs_f64();
    println!("{}",json!({"geometry_identity":g.description["geometry_identity"],"description":g.description,
        "units":"model mm; bridge m","precision":"canonical binary32","conditioning_invoked":false,
        "load_seconds":load_seconds,"scalar_count":values.len(),"scalar_bits_equal":true,
        "inventory":inventory,"evaluations":evaluations,"feature_batch":batch,"repeat_results_equal":true,
        "feature_seconds":feature_seconds,"query_options":options,"projection_seconds":projection_seconds,
        "projections":projections,"stored_section_points":airfoil.points.len(),"section_chord_model":airfoil.chord,
        "section_positions":wing.sections.iter().map(|s|s.position.to_array()).collect::<Vec<_>>(),
        "polygon_midpoints":polygon_probes,"timed_scalar_queries":100*points.len(),"timed_scalar_seconds":scalar_seconds,
        "checksum":checksum,"total_seconds":start.elapsed().as_secs_f64(),"no_mesh_generated":true}));
}
