use implicit_cad::sdf::aerospace::{Airfoil, LoftedWing};
use serde_json::{Value, json};
use v4_canonical_kernel::Registry;

#[test]
fn stored_bits_match_actual_canonical_sections() {
    let cases: Value = serde_json::from_str(include_str!("../../results/generic_canonical_bridge/DIRECT.json")).unwrap();
    let case = cases["cases"].as_array().unwrap().iter().find(|c| c["name"] == "plain_wing").unwrap();
    let mut registry = Registry::default();
    let loaded = registry.dispatch(json!({"op":"load_rhai", "script":case["script"],
        "metres_per_model_unit":case["metres_per_model_unit"]})).unwrap();
    let handle = loaded["handle"].as_u64().unwrap();
    let packet = registry.dispatch(json!({"op":"stored_topology","handle":handle})).unwrap();
    let object = registry.get(handle).unwrap();
    assert_eq!(packet["geometry_identity"], object.description["geometry_identity"]);
    assert_eq!(packet["kernel_source_sha256"], v4_canonical_kernel::SOURCE_SHA256);
    // Source identity is intentionally part of geometry identity. A rebuild
    // must not impersonate a historical executable. Retained geometry data
    // still have to match exactly after removing only those identity fields.
    let mut current = object.description.clone();
    let mut retained = case["description"].clone();
    if current["kernel_source_sha256"] != retained["kernel_source_sha256"] {
        assert_ne!(current["geometry_identity"], retained["geometry_identity"]);
    }
    for key in ["geometry_identity", "kernel_source_sha256"] {
        current.as_object_mut().unwrap().remove(key);
        retained.as_object_mut().unwrap().remove(key);
    }
    assert_eq!(current, retained);
    let wing = object.canonical.as_any().downcast_ref::<LoftedWing>().unwrap();
    let mut point_bits = 0;
    for (index, section) in wing.sections.iter().enumerate() {
        let exported = &packet["data"]["sections"][index];
        let airfoil = section.section.as_any().downcast_ref::<Airfoil>().unwrap();
        assert_eq!(exported["source_section_index"], index);
        assert_eq!(exported["chord_bits"], airfoil.chord.to_bits());
        assert_eq!(exported["position_bits"], json!(section.position.to_array().map(f32::to_bits)));
        assert_eq!(exported["twist_bits"], section.twist.to_bits());
        for (i, (x, z)) in airfoil.points.iter().enumerate() {
            assert_eq!(exported["point_bits"][i], json!([x.to_bits(), z.to_bits()]));
            point_bits += 2;
        }
    }
    println!("point_bits_equal={point_bits}, sections={}", wing.sections.len());
    assert_eq!(packet, registry.dispatch(json!({"op":"stored_topology","handle":handle})).unwrap());
    registry.release(handle).unwrap();
    assert_eq!(registry.dispatch(json!({"op":"stored_topology","handle":handle})).unwrap_err().code,"InvalidOrReleasedHandle");
    assert!(registry.dispatch(json!({"op":"stored_topology","handle":"wrong"})).is_err());
}

#[test]
fn unrelated_and_transformed_representations_not_approximated() {
    let cases: Value = serde_json::from_str(include_str!("../../results/generic_canonical_bridge/DIRECT.json")).unwrap();
    for name in ["sphere", "transformed_sphere", "scaled_sphere"] {
        let case = cases["cases"].as_array().unwrap().iter().find(|c| c["name"] == name).unwrap();
        let mut registry = Registry::default();
        let loaded = registry.dispatch(json!({"op":"load_rhai", "script":case["script"],
            "metres_per_model_unit":case["metres_per_model_unit"]})).unwrap();
        let result = registry.dispatch(json!({"op":"stored_topology","handle":loaded["handle"]})).unwrap();
        assert_eq!(result["data"]["status"],"unsupported");
        assert_eq!(result["data"]["reason"],"StoredPolygonSectionLoftRequired");
    }
}
