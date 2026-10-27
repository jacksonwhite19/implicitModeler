//! Transport only. All geometry operations remain on the owned canonical Sdf.
use crate::{Failure, Geometry, MAX_BATCH, Result, SOURCE_SHA256};
use glam::Vec3;
use implicit_cad::sdf::{
    procedural_features::{ProceduralFeatureQueryOptions, ProceduralFeatureQueryResult},
    query,
};
use serde::Deserialize;
use serde_json::{Value, json};

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FeatureEvaluation {
    pub feature_id: String,
    pub parameter: f64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FeatureOptions {
    pub closest_feature: bool,
    pub all_features_within_tolerance: bool,
    pub surface_sector: bool,
    pub local_parameters: bool,
    pub tolerance_m: f64,
}

impl Geometry {
    fn failure(&self, code: &str, detail: impl ToString) -> Failure {
        Failure {
            code: code.into(),
            stage: "canonical_query".into(),
            geometry_identity: self.description["geometry_identity"]
                .as_str()
                .map(str::to_owned),
            detail: detail.to_string(),
        }
    }
    fn finite(&self, values: impl IntoIterator<Item = f64>) -> Result<()> {
        if values.into_iter().all(f64::is_finite) {
            Ok(())
        } else {
            Err(self.failure("NonfiniteQueryResult", "kernel result or unit conversion"))
        }
    }
    fn check_count(&self, n: usize) -> Result<()> {
        if n > MAX_BATCH {
            Err(self.failure("BatchTooLarge", n))
        } else {
            Ok(())
        }
    }
    fn model_points(&self, points: &[[f64; 3]]) -> Result<Vec<Vec3>> {
        self.check_count(points.len())?;
        points
            .iter()
            .enumerate()
            .map(|(i, p)| {
                if !p.iter().all(|v| v.is_finite()) {
                    return Err(self.failure("NonfiniteCoordinate", i));
                }
                let q = Vec3::from_array(p.map(|v| (v / self.metres_per_model_unit) as f32));
                if !q.is_finite() {
                    return Err(self.failure("CoordinateOutOfKernelRange", i));
                }
                Ok(q)
            })
            .collect()
    }
    fn envelope(&self, operation: &str, data: Value) -> Value {
        json!({"schema":"V4CanonicalQueries/1", "geometry_identity":self.description["geometry_identity"],
            "kernel_source_sha256":SOURCE_SHA256,"operation":operation,
            "metres_per_model_unit":self.metres_per_model_unit,"data":data,
            "evidence":{"continuous_certificate":false,"qualified_branch_query":false,
                "feature_coverage":"unestablished","geometry_source":"ScriptResult::canonical_sdf or caller-admitted canonical object"}})
    }
    pub fn regional_expression(&self) -> Result<Value> {
        // A caller-admitted custom Sdf can self-report arbitrary metadata. Only
        // the existing Rhai construction route is admitted for this export.
        if !self.rhai_expression_issuer {
            return Err(self.failure("UnsupportedExpressionIssuer", "Rhai canonical object required"));
        }
        let expression = crate::regional_expression(&self.canonical.metadata(), &mut 65536, 0)?;
        Ok(self.envelope("regional_expression", json!({
            "schema":"V4SourceExpression/1", "expression":expression,
            "semantics":"Real extension of stored binary32 coefficients; binary32 evaluation error is a separate obligation",
            "continuous_acceptance":false
        })))
    }
    pub fn discover(&self) -> Result<Value> {
        let inventory = self.inventory()?;
        Ok(self.envelope("discover",json!({
            "operations":{"scalar":"available", "numerical_project":"available_unqualified",
                "numerical_normal":"available_unqualified", "feature_inventory":"callable_optional",
                "feature_evaluate":"callable_optional", "feature_query":"callable_may_return_unsupported"},
            "inventory":inventory["data"],
            "provenance":{"numerical_project":"src/sdf/query.rs::closest_point",
                "numerical_normal":"src/sdf/query.rs::gradient",
                "features":"src/sdf/mod.rs::Sdf procedural methods"},
            "semantics":{"projection":"at most 64 steps; abs(field)<1e-4 model-unit break; no convergence status or minimum-distance guarantee",
                "normal":"normalized central finite differences at 0.001 model-unit offsets; zero is not a successful normal",
                "default_feature_options_model":ProceduralFeatureQueryOptions::diagnostic(),
                "legacy_descriptor":"immutable admission/identity record; discover describes added query transport, not stronger geometry guarantees"}})))
    }
    pub fn numerical(&self, points: &[[f64; 3]], project: bool) -> Result<Value> {
        let start = std::time::Instant::now();
        let qs = self.model_points(points)?;
        let mut rows = Vec::with_capacity(qs.len());
        for (p, q) in points.iter().zip(qs) {
            let result = if project {
                query::closest_point(self.canonical.as_ref(), q)
            } else {
                q
            };
            let normal = query::gradient(self.canonical.as_ref(), result);
            let residual = self.canonical.distance(result);
            let result_m = result
                .to_array()
                .map(|v| f64::from(v) * self.metres_per_model_unit);
            let displacement = std::array::from_fn::<_, 3, _>(|i| result_m[i] - p[i]);
            let residual_m = f64::from(residual) * self.metres_per_model_unit;
            self.finite(
                result_m
                    .into_iter()
                    .chain(displacement)
                    .chain(normal.to_array().map(f64::from))
                    .chain([f64::from(residual), residual_m]),
            )?;
            rows.push(json!({"requested_point_m":p,"input_model_f32":q.to_array().map(f64::from),
                "returned_point_model_f32":result.to_array().map(f64::from),"returned_point_m":result_m,
                "displacement_m":displacement,"normal_direction":normal.to_array().map(f64::from),
                "normal_status":if normal.length_squared()==0. {"zero_direction"} else {"numerical_direction"},
                "signed_field_at_return_model_f32":f64::from(residual),"signed_field_at_return_m":residual_m,
                "convergence_status":"not_provided_by_kernel", "branch_selection":"not_provided"}));
        }
        Ok(self.envelope(
            if project {
                "numerical_project"
            } else {
                "numerical_normal"
            },
            json!({"results":rows,"elapsed_seconds":start.elapsed().as_secs_f64()}),
        ))
    }
    pub fn inventory(&self) -> Result<Value> {
        let raw = self.canonical.procedural_feature_inventory();
        if let Some(i) = &raw {
            for f in &i.features {
                self.finite(
                    f.curve_domain
                        .into_iter()
                        .flatten()
                        .chain(f.finite_trailing_edge_thickness),
                )?;
            }
            for j in &i.junctions {
                self.finite(j.span_parameter)?;
            }
        }
        let mut si = raw.clone();
        if let Some(i) = &mut si {
            for f in &mut i.features {
                if let Some(v) = &mut f.finite_trailing_edge_thickness {
                    *v *= self.metres_per_model_unit;
                    self.finite([*v])?;
                }
            }
        }
        Ok(self.envelope("feature_inventory",json!({"model":raw,"si":si,
            "scope":"inventory only; not certified smooth sheets or complete ownership",
            "conversion":"finite_trailing_edge_thickness is a length; IDs and curve/span parameters unchanged"})))
    }
    fn feature_si(
        &self,
        raw: &ProceduralFeatureQueryResult,
    ) -> Result<ProceduralFeatureQueryResult> {
        let mut r = raw.clone();
        // Validate BEFORE serde serialization (nonfinite serde floats otherwise become null).
        self.finite(
            r.query_position
                .into_iter()
                .chain(r.closest_point.into_iter().flatten())
                .chain(r.tangent.into_iter().flatten())
                .chain(r.distance_to_feature)
                .chain(r.span_parameter)
                .chain(r.chord_parameter)
                .chain(r.airfoil_parameter)
                .chain(r.diagnostics.query_tolerance)
                .chain(r.diagnostics.feature_to_sdf_residual),
        )?;
        for m in &r.all_matches {
            self.finite(
                m.closest_point
                    .into_iter()
                    .chain(m.tangent.into_iter().flatten())
                    .chain([m.distance])
                    .chain(m.span_parameter),
            )?;
        }
        let s = self.metres_per_model_unit;
        r.query_position = r.query_position.map(|v| v * s);
        r.closest_point = r.closest_point.map(|p| p.map(|v| v * s));
        r.distance_to_feature = r.distance_to_feature.map(|v| v * s);
        r.diagnostics.query_tolerance = r.diagnostics.query_tolerance.map(|v| v * s);
        r.diagnostics.feature_to_sdf_residual =
            r.diagnostics.feature_to_sdf_residual.map(|v| v * s);
        for m in &mut r.all_matches {
            m.closest_point = m.closest_point.map(|v| v * s);
            m.distance *= s;
        }
        self.finite(
            r.query_position
                .into_iter()
                .chain(r.closest_point.into_iter().flatten())
                .chain(r.distance_to_feature)
                .chain(r.diagnostics.query_tolerance)
                .chain(r.diagnostics.feature_to_sdf_residual),
        )?;
        for m in &r.all_matches {
            self.finite(m.closest_point.into_iter().chain([m.distance]))?;
        }
        Ok(r)
    }
    pub fn evaluate_features(&self, requests: &[FeatureEvaluation]) -> Result<Value> {
        self.check_count(requests.len())?;
        let mut rows = Vec::with_capacity(requests.len());
        for r in requests {
            if !r.parameter.is_finite() {
                return Err(self.failure("InvalidFeatureParameter", &r.feature_id));
            }
            let raw = self
                .canonical
                .evaluate_procedural_feature(&r.feature_id, r.parameter);
            let si = raw.as_ref().map(|v| self.feature_si(v)).transpose()?;
            rows.push(json!({"feature_id":r.feature_id,"parameter":r.parameter,
                "status":if raw.is_some(){"returned"}else{"unsupported_or_unknown_feature"},"model":raw,"si":si}));
        }
        Ok(self.envelope("feature_evaluate", json!({"results":rows})))
    }
    pub fn classify_features(
        &self,
        points: &[[f64; 3]],
        options: Option<FeatureOptions>,
    ) -> Result<Value> {
        let qs = self.model_points(points)?;
        let options = match options {
            None => ProceduralFeatureQueryOptions::diagnostic(),
            Some(o) => {
                let tolerance = o.tolerance_m / self.metres_per_model_unit;
                if !tolerance.is_finite() || tolerance < 0. {
                    return Err(self.failure("InvalidFeatureTolerance", o.tolerance_m));
                }
                ProceduralFeatureQueryOptions {
                    closest_feature: o.closest_feature,
                    all_features_within_tolerance: o.all_features_within_tolerance,
                    surface_sector: o.surface_sector,
                    local_parameters: o.local_parameters,
                    tolerance,
                }
            }
        };
        let raw = self.canonical.query_procedural_feature_batch(&qs, &options);
        let si = raw
            .results
            .iter()
            .map(|r| self.feature_si(r))
            .collect::<Result<Vec<_>>>()?;
        Ok(self.envelope("feature_query",json!({"options_model":options,"model":raw,
            "results_si":si,"selection":"kernel candidates and ambiguity preserved; not certified ownership"})))
    }
}
