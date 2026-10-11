//! Regression fixtures exercise the shared extractor; production has no fixture dispatch.
use glam::Vec3;
use implicit_cad::{mesh::{Mesh, marching_cubes::{extract_scalar_seed_canonical_edges,
    extract_scalar_seed_canonical_edges_unfiltered},
    sdf_surface::{weld_and_clean_seed_mesh, SurfaceExtractionConfig}}, sdf::Sdf};
use serde_json::{json, Value};
use std::{collections::BTreeMap, sync::{Arc, atomic::{AtomicU64, Ordering}}};
use v4_canonical_kernel::Registry;

struct Counted { field: Arc<dyn Sdf>, calls: AtomicU64 }
impl Sdf for Counted {
    fn distance(&self,p:Vec3)->f32 { self.calls.fetch_add(1,Ordering::Relaxed);self.field.distance(p) }
}
fn topology(vertices: usize, triangles: &[[u32;3]]) -> Value {
    let mut edges=BTreeMap::<(u32,u32),(usize,i32)>::new();
    for t in triangles {
        assert!(t.iter().all(|i| (*i as usize)<vertices));
        assert!(t[0]!=t[1] && t[1]!=t[2] && t[2]!=t[0]);
        for (a,b) in [(t[0],t[1]),(t[1],t[2]),(t[2],t[0])] {
            let v=edges.entry((a.min(b),a.max(b))).or_default();
            v.0+=1;v.1+=if a<b {1} else {-1};
        }
    }
    json!({"vertices":vertices,"triangles":triangles.len(),"edges":edges.len(),
        "boundary_edges":edges.values().filter(|v|v.0==1).count(),
        "nonmanifold_edges":edges.values().filter(|v|v.0>2).count(),
        "orientation_failures":edges.values().filter(|v|v.1!=0).count(),
        "euler":vertices as i64-edges.len() as i64+triangles.len() as i64})
}
fn tris(mesh:&Mesh)->Vec<[u32;3]> { mesh.indices.chunks_exact(3).map(|t|[t[0],t[1],t[2]]).collect() }
fn assert_closed(t:&Value) {
    assert_eq!(t["boundary_edges"],0);assert_eq!(t["nonmanifold_edges"],0);
    assert_eq!(t["orientation_failures"],0);assert_eq!(t["euler"],2);
}
fn failure_case()->Value {
    let p=std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../results/random_shape_smoke_20261010/CASE.json");
    serde_json::from_slice(&std::fs::read(p).unwrap()).unwrap()
}

#[test]
fn unchanged_failed_case_localizes_cleanup_damage_and_preserves_connectivity() {
    let case=failure_case();let scale=case["geometry"]["metres_per_model_unit"].as_f64().unwrap();
    let controls=&case["general_sdf"]["seed"];
    let spacing=controls["spacing_m"].as_f64().unwrap();let padding=controls["padding_m"].as_f64().unwrap();
    let mut registry=Registry::default();
    let h=registry.load_rhai(case["geometry"]["rhai"].as_str().unwrap(),scale).unwrap();
    let g=registry.get(h).unwrap();let bounds=g.canonical.metadata().support_bounds.unwrap();
    let low=bounds.min-Vec3::splat((padding/scale) as f32);
    let high=bounds.max+Vec3::splat((padding/scale) as f32);
    let res=(f64::from((high-low).max_element())*scale/spacing).ceil().max(8.) as u32;
    assert_eq!(res,50);
    let counted=Counted{field:g.canonical.clone(),calls:AtomicU64::new(0)};
    let raw=extract_scalar_seed_canonical_edges_unfiltered(&counted,low,high,res);
    assert_eq!(counted.calls.swap(0,Ordering::Relaxed),51_u64.pow(3));
    let legacy=extract_scalar_seed_canonical_edges(&counted,low,high,res);
    assert_eq!(counted.calls.load(Ordering::Relaxed),51_u64.pow(3));
    assert_eq!(raw.vertices.len(),legacy.vertices.len());
    for (a,b) in raw.vertices.iter().zip(&legacy.vertices) {
        assert_eq!(a.position.map(f32::to_bits),b.position.map(f32::to_bits));assert_eq!(a.normal,[0.;3]);
    }
    let raw_t=tris(&raw);let raw_topology=topology(raw.vertices.len(),&raw_t);
    assert_closed(&raw_topology);
    let legacy_topology=topology(legacy.vertices.len(),&tris(&legacy));
    assert!(legacy_topology["boundary_edges"].as_u64().unwrap()>0);
    let cleaned=weld_and_clean_seed_mesh(&legacy,&SurfaceExtractionConfig::default());
    let cleaned_t:Vec<[u32;3]>=cleaned.triangles.iter().map(|t|t.map(|i|u32::try_from(i).unwrap())).collect();
    let cleaned_topology=topology(cleaned.vertices.len(),&cleaned_t);
    assert_eq!(cleaned_topology["boundary_edges"],21);assert_eq!(cleaned_topology["euler"],-5);
    // Every deleted legacy triangle in this fixture has nonzero finite area.
    let retained:std::collections::BTreeSet<_>=tris(&legacy).into_iter().collect();
    let deleted:Vec<_>=raw_t.iter().filter(|t|!retained.contains(*t)).collect();
    assert_eq!(deleted.len(),9);
    let areas:Vec<_>=deleted.iter().map(|t| {
        let p=t.map(|i|Vec3::from_array(raw.vertices[i as usize].position));
        f64::from((p[1]-p[0]).cross(p[2]-p[0]).length()*0.5)
    }).collect();
    assert!(areas.iter().all(|a|a.is_finite() && *a>0.));
    let raw_cleaned=weld_and_clean_seed_mesh(&raw,&SurfaceExtractionConfig::default());
    let raw_cleaned_t:Vec<[u32;3]>=raw_cleaned.triangles.iter().map(|t|t.map(|i|u32::try_from(i).unwrap())).collect();
    let raw_cleaned_topology=topology(raw_cleaned.vertices.len(),&raw_cleaned_t);
    println!("SEED_TOPOLOGY_DIAGNOSIS={}",json!({"unfiltered":raw_topology,"legacy_small_area_filter":legacy_topology,
        "legacy_filtered_then_welded":cleaned_topology,"unfiltered_then_legacy_welded":raw_cleaned_topology,
        "deleted_nonzero_areas_model_units_squared":areas,"legacy_cleanup":cleaned.cleanup}));
    let p=g.surface_seed(padding,spacing,controls["maximum_queries"].as_u64().unwrap(),
        controls["timeout_seconds"].as_f64().unwrap()).unwrap();
    let d=&p["data"];let triangles:Vec<[u32;3]>=serde_json::from_value(d["triangles"].clone()).unwrap();
    assert_eq!(triangles,raw_t);assert_closed(&topology(d["coordinates_m"].as_array().unwrap().len(),&triangles));
    assert_eq!(d["stock_cleanup"],"NotRun");assert_eq!(d["small_area_deletion"],"NotRun");
    assert_eq!(d["approximate_coordinate_weld"],"NotRun");assert_eq!(d["continuous_acceptance"],false);
    assert_eq!(d["scalar_evaluations"],132651);
}

#[test]
fn shared_native_proposal_retains_closed_connectivity_across_smooth_sources() {
    for source in ["sphere(2.0)","translate(scale(sphere(1.0), 3.1, 2.2, 1.7), 0.17, -0.13, 0.21)",
        "smooth_union(sphere(2.0), translate(scale(sphere(1.0), 2.5, 1.7, 1.4), 1.4, 0.3, -0.2), 0.5)"] {
        let mut registry=Registry::default();let h=registry.load_rhai(source,0.001).unwrap();
        let p=registry.get(h).unwrap().surface_seed(0.0001,0.0001,1000000,30.).unwrap();
        let d=&p["data"];let triangles:Vec<[u32;3]>=serde_json::from_value(d["triangles"].clone()).unwrap();
        assert_closed(&topology(d["coordinates_m"].as_array().unwrap().len(),&triangles));
        assert_eq!(d["seed_geometry_audit"],"NotRun");
        assert_eq!(d["stage_counts"]["normal_stencil_attempts"],0);
    }
}
