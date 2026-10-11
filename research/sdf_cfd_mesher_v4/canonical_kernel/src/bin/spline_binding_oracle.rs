//! Bounded direct-storage/query oracle. Calls existing kernel methods only.
use glam::{DVec2, Vec3};
use implicit_cad::sdf::{Sdf, aerospace::{Airfoil, LoftedWing, naca_spline::NacaSpline}};
use serde_json::{Value, json};
use v4_canonical_kernel::Registry;

fn main() {
    let args: Vec<_> = std::env::args().collect();
    let script = std::fs::read_to_string(&args[1]).unwrap();
    let points: Vec<[f64;3]> = serde_json::from_slice(&std::fs::read(&args[2]).unwrap()).unwrap();
    let mut registry = Registry::default();
    let handle = registry.load_rhai(&script,1.).unwrap();
    let g = registry.get(handle).unwrap();
    let packet = g.stored_spline().unwrap();
    let transported: Value = serde_json::from_slice(&serde_json::to_vec(&packet).unwrap()).unwrap();
    assert_eq!(packet, transported);
    let wing = g.canonical.as_any().downcast_ref::<LoftedWing>().unwrap();
    let mut controls=0;let mut placements=0;
    for (i,s) in wing.sections.iter().enumerate() {
        let a=s.section.as_any().downcast_ref::<Airfoil>().unwrap();
        let out=&transported["data"]["sections"][i];
        for (v,b) in s.position.to_array().iter().zip(out["position_bits"].as_array().unwrap()) {
            assert_eq!(v.to_bits() as u64,b.as_u64().unwrap());placements+=1;
        }
        assert_eq!(a.chord.to_bits() as u64,out["chord_bits"].as_u64().unwrap());
        assert_eq!(s.twist.to_bits() as u64,out["twist_bits"].as_u64().unwrap());placements+=2;
        let spline=a.spline().unwrap();
        let reloaded:NacaSpline=serde_json::from_value(out["spline"].clone()).unwrap();
        assert_eq!(spline.as_ref(),&reloaded);
        for (j,span) in spline.spans.iter().enumerate() { for k in 0..4 { for d in 0..2 {
            assert_eq!(span.control[k][d].to_bits(),out["control_bits"][j][k][d].as_u64().unwrap());
            controls+=1;
        }}}
    }
    let a=wing.sections[0].section.as_any().downcast_ref::<Airfoil>().unwrap();
    let spline=a.spline().unwrap();
    let reloaded:NacaSpline=serde_json::from_value(transported["data"]["sections"][0]["spline"].clone()).unwrap();
    let mut rows=Vec::new();
    for p in points {
        let q=DVec2::new(p[0],p[2]);
        let direct=spline.closest_skin_point(q).unwrap();
        let exported=reloaded.closest_skin_point(q).unwrap();
        assert_eq!(serde_json::to_value(&direct).unwrap(),serde_json::to_value(&exported).unwrap());
        rows.push(json!({"point":p,"scalar":wing.distance(Vec3::from_array(p.map(|x|x as f32))),
            "direct_skin":direct,"exported_skin_identical":true}));
    }
    println!("{}",json!({"geometry_identity":g.description["geometry_identity"],
        "spline_identity":packet["spline_identity"],"coefficient_bit_comparisons":controls,
        "placement_bit_comparisons":placements,"all_exact":true,"queries":rows}));
}
