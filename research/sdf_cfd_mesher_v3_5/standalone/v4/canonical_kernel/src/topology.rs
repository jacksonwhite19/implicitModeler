//! Stored representation export only; no profile construction or meshing.
use crate::{Geometry, Result, SOURCE_SHA256, hash_json};
use implicit_cad::sdf::aerospace::{Airfoil, LoftedWing};
use implicit_cad::sdf::aerospace::airfoil::AirfoilRepresentation;
use serde_json::{Value, json};

impl Geometry {
    pub fn stored_topology(&self) -> Result<Value> {
        let data = if let Some(wing) = self.canonical.as_any().downcast_ref::<LoftedWing>() {
            let mut sections = Vec::new();
            let mut unsupported = None;
            for (index, section) in wing.sections.iter().enumerate() {
                if let Some(profile) = section.section.as_any().downcast_ref::<Airfoil>() {
                    if !matches!(profile.representation, AirfoilRepresentation::Polygon) {
                        unsupported = Some("SectionRepresentationNotPolygon");
                        break;
                    }
                    if !section.position.is_finite() || !section.twist.is_finite()
                        || !profile.chord.is_finite()
                        || profile.points.iter().any(|p| !p.0.is_finite() || !p.1.is_finite()) {
                        unsupported = Some("NonfiniteStoredTopology");
                        break;
                    }
                    sections.push(json!({"source_section_index":index,
                        "point_bits":profile.points.iter().map(|p|[p.0.to_bits(),p.1.to_bits()]).collect::<Vec<_>>(),
                        "chord_bits":profile.chord.to_bits(),
                        "position_bits":section.position.to_array().map(f32::to_bits),
                        "twist_bits":section.twist.to_bits()}));
                } else {
                    unsupported = Some("SectionStoredTopologyUnavailable");
                    break;
                }
            }
            if let Some(reason) = unsupported {
                json!({"status":"unsupported", "reason":reason})
            } else {
                json!({"status":"available", "representation":"StoredPolygonSectionLoft/1",
                    "sections":sections, "section_coordinates":"normalized x,z",
                    "placement_coordinates":"model units; span along y",
                    "frame":"unwrapped canonical object; no external transform admitted",
                    "closure":"explicit stored endpoint repeat; checked by consumer",
                    "kernel_feature_ids":self.description["stored_metadata"]["feature_ids"],
                    "provenance":"LoftedWing.sections -> Airfoil.points/chord; original f32 bits",
                    "uniform_extrusion":"not inferred by export; consumer must check stored values"})
            }
        } else {
            json!({"status":"unsupported", "reason":"StoredPolygonSectionLoftRequired",
                "scope":"Other objects and external transform wrappers are not approximated"})
        };
        let payload = json!({"geometry_identity":self.description["geometry_identity"],
            "kernel_source_sha256":SOURCE_SHA256,"metres_per_model_unit":self.metres_per_model_unit,
            "data":data});
        Ok(json!({"schema":"V4CanonicalQueries/1", "operation":"stored_topology",
            "geometry_identity":self.description["geometry_identity"],
            "kernel_source_sha256":SOURCE_SHA256,"metres_per_model_unit":self.metres_per_model_unit,
            "topology_identity":hash_json(&payload),"data":payload["data"],
            "evidence":{"source_bits":true,"canonical_equivalence_certificate":false,
                "qualified_surface":false}}))
    }
}
