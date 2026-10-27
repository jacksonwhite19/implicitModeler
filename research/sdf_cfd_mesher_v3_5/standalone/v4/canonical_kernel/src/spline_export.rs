//! Owned export of actual storage, matching the SF2 geometry_probe source layout.
//! No reconstruction, normalization, preview fit, or geometry operation.
use crate::{Failure, Geometry, Result, SOURCE_SHA256, hash_json};
use implicit_cad::sdf::aerospace::{Airfoil, LoftedWing};
use serde_json::{Value, json};

impl Geometry {
    pub fn stored_spline(&self) -> Result<Value> {
        let data = if let Some(wing) = self.canonical.as_any().downcast_ref::<LoftedWing>() {
            let mut sections = Vec::new();
            let mut reason = None;
            for (index, s) in wing.sections.iter().enumerate() {
                let Some(a) = s.section.as_any().downcast_ref::<Airfoil>() else {
                    reason = Some("StoredAirfoilRequired"); break;
                };
                let Some(spline) = a.spline() else {
                    reason = Some("StoredSmoothSplineRequired"); break;
                };
                if !a.chord.is_finite() || !s.twist.is_finite() || !s.position.is_finite()
                    || spline.spans.iter().any(|span| span.control.iter().flatten().any(|x| !x.is_finite())) {
                    return Err(Failure::new("NonfiniteStoredSpline", "stored_spline", index));
                }
                // Operational protocol bound; no resampling fallback.
                if sections.len() >= 16 || spline.spans.len() > 4096 {
                    return Err(Failure::new("SplineExportResourceLimit", "stored_spline", "16 sections / 4096 spans"));
                }
                sections.push(json!({"source_section_index":index,
                    "position":s.position.to_array(),"chord":a.chord,"twist":s.twist,
                    "position_bits":s.position.to_array().map(f32::to_bits),
                    "chord_bits":a.chord.to_bits(),"twist_bits":s.twist.to_bits(),
                    "spline":spline.as_ref(),
                    "control_bits":spline.spans.iter().map(|span|span.control.map(|p|p.map(f64::to_bits))).collect::<Vec<_>>() }));
            }
            if let Some(reason) = reason { json!({"status":"unsupported","reason":reason}) }
            else { json!({"status":"available","representation":"StoredSmoothSplineSectionLoft/1",
                "sections":sections,"inventory":self.canonical.procedural_feature_inventory(),
                "coordinates":"spline controls normalized x,z; placements/chord in model units",
                "frame":"unwrapped canonical object; no external transform",
                "precision":"stored binary64 cubic coefficients; binary32 chord/placement/twist",
                "preview_geometry_used":false,
                "provenance":"LoftedWing.sections -> Airfoil::spline(); SF2 geometry_probe export mechanism"}) }
        } else { json!({"status":"unsupported","reason":"UnwrappedStoredSectionLoftRequired"}) };
        let payload = json!({"geometry_identity":self.description["geometry_identity"],
            "kernel_source_sha256":SOURCE_SHA256,"metres_per_model_unit":self.metres_per_model_unit,
            "data":data});
        let result = json!({"schema":"V4CanonicalQueries/1","operation":"stored_spline",
            "geometry_identity":self.description["geometry_identity"],"kernel_source_sha256":SOURCE_SHA256,
            "metres_per_model_unit":self.metres_per_model_unit,"spline_identity":hash_json(&payload),
            "data":payload["data"],"evidence":{"stored_source_export":true,"surface_qualified":false}});
        if serde_json::to_vec(&result).unwrap().len() + 64 > crate::MAX_LINE {
            return Err(Failure::new("SplineExportResourceLimit", "stored_spline", "response exceeds existing MAX_LINE"));
        }
        Ok(result)
    }
}
