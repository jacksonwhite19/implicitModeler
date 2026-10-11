//! Generic owned connection to existing canonical objects. No shape construction.
use glam::Vec3;
use implicit_cad::{
    scripting,
    sdf::{Sdf, conditioning::SdfNodeMetadata},
};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    collections::HashMap,
    io::{BufRead, Write},
    panic::{AssertUnwindSafe, catch_unwind},
    sync::Arc,
};

pub const SOURCE_SHA256: &str = env!("V4_KERNEL_SOURCE_SHA256");
pub const MAX_BATCH: usize = 65536;
pub const MAX_LINE: usize = 8 * 1024 * 1024;
pub mod queries;
pub mod topology;
pub mod spline_export;
pub mod surface_seed;

#[derive(Debug, Serialize)]
pub struct Failure {
    pub code: String,
    pub stage: String,
    pub geometry_identity: Option<String>,
    pub detail: String,
}
impl Failure {
    pub fn new(code: &str, stage: &str, detail: impl ToString) -> Self {
        Self {
            code: code.into(),
            stage: stage.into(),
            geometry_identity: None,
            detail: detail.to_string(),
        }
    }
}
type Result<T> = std::result::Result<T, Failure>;

pub fn hash_json(v: &Value) -> String {
    format!("{:x}", Sha256::digest(serde_json::to_vec(v).unwrap()))
}

#[cfg(test)]
mod regional_controls {
    use super::*;
    use implicit_cad::sdf::conditioning::metadata_fingerprint_from_f32s;
    use implicit_cad::sdf::primitives::Sphere;

    #[test]
    fn exact_parameter_capture_preserves_fingerprint_and_clone() {
        let values = [0.0_f32, -0.0, 0.3, f32::MIN_POSITIVE];
        let m = SdfNodeMetadata::new("control").with_f32_parameters(values);
        assert_eq!(m.parameter_bits, values.map(f32::to_bits));
        assert_eq!(m.parameter_fingerprint, metadata_fingerprint_from_f32s(values));
        assert_eq!(m, m.clone());
        let replaced = m.clone().with_parameter_fingerprint(7);
        assert_eq!(replaced.parameter_fingerprint, 7);
        assert!(replaced.parameter_bits.is_empty());
        assert!(SdfNodeMetadata::new("control").parameter_bits.is_empty());
    }

    #[test]
    fn caller_metadata_cannot_forge_rhai_issuer() {
        let mut registry = Registry::default();
        let handle = registry.admit(Arc::new(Sphere::new(1.0)),
            json!({"kind":"rhai_utf8","script":"sphere(1.0)"}), 1.0).unwrap();
        let error = registry.dispatch(json!({"op":"regional_expression","handle":handle})).unwrap_err();
        assert_eq!(error.code, "UnsupportedExpressionIssuer");
    }

    #[test]
    fn native_rhai_export_is_live_and_unit_bound() {
        let mut registry = Registry::default();
        let handle = registry.load_rhai("translate(box_(2.0, 4.0, 6.0), 3.0, 4.0, 5.0)", 0.001).unwrap();
        let packet = registry.dispatch(json!({"op":"regional_expression","handle":handle})).unwrap();
        assert_eq!(packet["geometry_identity"], registry.get(handle).unwrap().description["geometry_identity"]);
        assert_eq!(packet["kernel_source_sha256"], SOURCE_SHA256);
        assert_eq!(packet["metres_per_model_unit"], 0.001);
        let expression = &packet["data"]["expression"];
        assert_eq!(expression["parameters_bits"], json!([3.0_f32.to_bits(),4.0_f32.to_bits(),5.0_f32.to_bits()]));
        assert_eq!(expression["children"]["child"]["parameters_bits"], json!([1.0_f32.to_bits(),2.0_f32.to_bits(),3.0_f32.to_bits()]));
        assert_eq!(packet["data"]["continuous_acceptance"], false);
        registry.release(handle).unwrap();
        assert_eq!(registry.dispatch(json!({"op":"regional_expression","handle":handle})).unwrap_err().code,"InvalidOrReleasedHandle");
    }

    #[test]
    fn unsupported_or_invalid_expression_is_not_exported() {
        for (script, code) in [("rotate(sphere(1.0), 0.0, 0.0, 10.0)","UnsupportedSourceBoundOperation"),
            ("smooth_union(sphere(1.0), sphere(2.0), 0.0)","InvalidSourceBoundParameter")] {
            let mut registry = Registry::default();
            let handle = registry.load_rhai(script, 1.0).unwrap();
            assert_eq!(registry.dispatch(json!({"op":"regional_expression","handle":handle})).unwrap_err().code, code);
        }
    }

    #[test]
    fn malformed_metadata_and_expression_budget_reject() {
        let metadata = Sphere::new(1.0).metadata();
        assert_eq!(regional_expression(&metadata, &mut 0, 0).unwrap_err().code,"SourceExpressionBudgetExceeded");
        assert_eq!(regional_expression(&metadata, &mut 8, 129).unwrap_err().code,"SourceExpressionBudgetExceeded");
        let mut wrong = metadata.clone(); wrong.parameter_bits.clear();
        assert_eq!(regional_expression(&wrong,&mut 8,0).unwrap_err().code,"SourceExpressionArityMismatch");
        let wrong = SdfNodeMetadata::new("translate").with_f32_parameters([1.0,2.0,3.0])
            .with_dependency("wrong",metadata);
        assert_eq!(regional_expression(&wrong,&mut 8,0).unwrap_err().code,"SourceExpressionRoleMismatch");
    }
}

fn metadata(m: &SdfNodeMetadata) -> Result<Value> {
    if m.conditioned_cache.is_some() {
        return Err(Failure::new(
            "ConditionedGeometryRejected",
            "admission",
            "canonical object required",
        ));
    }
    let bounds = m
        .support_bounds
        .as_ref()
        .map(|b| [b.min.to_array(), b.max.to_array()]);
    if bounds.as_ref().is_some_and(|b| {
        !b.iter().flatten().all(|v| v.is_finite()) || (0..3).any(|i| b[0][i] > b[1][i])
    }) {
        return Err(Failure::new(
            "InvalidKernelBounds",
            "admission",
            &m.node_kind,
        ));
    }
    let dependencies = m
        .dependencies
        .iter()
        .map(|d| Ok(json!({"role":d.role,"metadata":metadata(&d.metadata)?})))
        .collect::<Result<Vec<_>>>()?;
    Ok(
        json!({"node_kind":m.node_kind,"bounds_model":bounds,"bound_quality":format!("{:?}",m.bound_quality),
        "parameter_fingerprint":m.parameter_fingerprint.to_string(),"dependencies":dependencies,
        "feature_ids":m.feature_ids,"conditioned_cache_present":false}),
    )
}

// Query-only export from the actual canonical object's stored metadata.
// No script parser, parameter inference, or new distance implementation.
fn regional_expression(m: &SdfNodeMetadata, budget: &mut usize, depth: usize) -> Result<Value> {
    let fail = |code: &str| Failure::new(code, "regional_expression", &m.node_kind);
    if *budget == 0 || depth > 128 { return Err(fail("SourceExpressionBudgetExceeded")); }
    *budget -= 1;
    if m.conditioned_cache.is_some() { return Err(fail("ConditionedGeometryRejected")); }
    let (count, roles): (usize, &[&str]) = match m.node_kind.as_str() {
        "sphere" => (1, &[]), "box" => (3, &[]),
        "cylinder" | "torus" => (2, &[]),
        "translate" | "scale" => (3, &["child"]), "offset" => (1, &["child"]),
        "union" | "intersect" | "subtract" => (0, &["a", "b"]),
        "smooth_union" | "smooth_intersect" => (1, &["a", "b"]),
        "smooth_subtract" => (1, &["base", "tool"]),
        _ => return Err(fail("UnsupportedSourceBoundOperation")),
    };
    if m.parameter_bits.len() != count || m.dependencies.len() != roles.len() {
        return Err(fail("SourceExpressionArityMismatch"));
    }
    let positive = matches!(m.node_kind.as_str(), "sphere" | "box" | "cylinder" | "torus" |
        "scale" | "smooth_union" | "smooth_intersect" | "smooth_subtract");
    if m.parameter_bits.iter().any(|b| {
        let v = f32::from_bits(*b); !v.is_finite() || (positive && v <= 0.0)
    }) { return Err(fail("InvalidSourceBoundParameter")); }
    let mut children = serde_json::Map::new();
    for role in roles {
        let matches: Vec<_> = m.dependencies.iter().filter(|d| d.role == *role).collect();
        if matches.len() != 1 { return Err(fail("SourceExpressionRoleMismatch")); }
        children.insert((*role).into(), regional_expression(&matches[0].metadata, budget, depth + 1)?);
    }
    Ok(json!({"kind":m.node_kind,"parameters_bits":m.parameter_bits,"children":children}))
}

pub struct Geometry {
    rhai_expression_issuer: bool,
    pub canonical: Arc<dyn Sdf>,
    pub metres_per_model_unit: f64,
    pub description: Value,
}
impl Geometry {
    pub fn query(&self, points: &[[f64; 3]]) -> Result<Vec<f64>> {
        let id = self.description["geometry_identity"].as_str().unwrap();
        let fail = |code: &str, detail: String| Failure {
            code: code.into(),
            stage: "scalar_query".into(),
            geometry_identity: Some(id.into()),
            detail,
        };
        if points.len() > MAX_BATCH {
            return Err(fail("BatchTooLarge", points.len().to_string()));
        }
        let mut values = Vec::with_capacity(points.len());
        for (i, p) in points.iter().enumerate() {
            if !p.iter().all(|x| x.is_finite()) {
                return Err(fail("NonfiniteCoordinate", i.to_string()));
            }
            let q = Vec3::from_array(p.map(|x| (x / self.metres_per_model_unit) as f32));
            if !q.is_finite() {
                return Err(fail("CoordinateOutOfKernelRange", i.to_string()));
            }
            let d = f64::from(self.canonical.distance(q)) * self.metres_per_model_unit;
            if !d.is_finite() {
                return Err(fail("NonfiniteFieldResult", i.to_string()));
            }
            values.push(d);
        }
        Ok(values)
    }
}

#[derive(Default)]
pub struct Registry {
    objects: HashMap<u64, Geometry>,
    next: u64,
    rhai_attempted: bool,
}
impl Registry {
    /// Caller owns construction. Admission takes one Arc ownership reference.
    pub fn admit(
        &mut self,
        canonical: Arc<dyn Sdf>,
        construction: Value,
        scale: f64,
    ) -> Result<u64> {
        if !scale.is_finite() || scale <= 0.0 {
            return Err(Failure::new("InvalidUnitScale", "admission", scale));
        }
        let meta = metadata(&canonical.metadata())?;
        let inventory = canonical.procedural_feature_inventory();
        let mut description = json!({"schema":"V4CanonicalKernel/1","kernel_source_sha256":SOURCE_SHA256,
            "construction":construction,"stored_metadata":meta,"metres_per_model_unit":scale,
            "metres_per_model_unit_bits":format!("{:016x}",scale.to_bits()),
            "coordinate_rule":"q_model_f32 = f32(q_m_f64 / metres_per_model_unit)",
            "field_rule":"phi_m_f64 = f64(canonical.distance(q_model_f32)) * metres_per_model_unit",
            "sign":"negative_inside_positive_outside","precision":"canonical binary32; bridge binary64 conversion only",
            "capabilities":{"signed_scalar":"supported","exact_distance":"not_declared",
                "approximate_distance":"error_bound_not_declared","projection":"unsupported",
                "derivatives":"unsupported","curvature_bounds":"unsupported",
                "feature_query":"unsupported","feature_inventory":if inventory.is_some(){"available_unqualified"}else{"unsupported"},
                "continuous_certificates":"unsupported","bounds":if meta["bounds_model"].is_null(){"caller_region_required"}else{"kernel_metadata"}},
            "feature_inventory":inventory});
        let identity = hash_json(&description);
        description["geometry_identity"] = json!(identity);
        self.next = self
            .next
            .checked_add(1)
            .ok_or_else(|| Failure::new("HandleSpaceExhausted", "admission", "restart host"))?;
        self.objects.insert(
            self.next,
            Geometry {
                rhai_expression_issuer: false,
                canonical,
                metres_per_model_unit: scale,
                description,
            },
        );
        Ok(self.next)
    }
    pub fn get(&self, handle: u64) -> Result<&Geometry> {
        self.objects
            .get(&handle)
            .ok_or_else(|| Failure::new("InvalidOrReleasedHandle", "handle_lookup", handle))
    }
    pub fn release(&mut self, handle: u64) -> Result<()> {
        self.objects
            .remove(&handle)
            .map(|_| ())
            .ok_or_else(|| Failure::new("InvalidOrReleasedHandle", "release", handle))
    }
    pub fn load_rhai(&mut self, script: &str, scale: f64) -> Result<u64> {
        if self.rhai_attempted {
            return Err(Failure::new(
                "FreshHostRequired",
                "construction",
                "one Rhai construction attempt per host prevents existing quantized helper-cache aliases",
            ));
        }
        self.rhai_attempted = true;
        let result = scripting::evaluate_script(script)
            .map_err(|e| Failure::new("ScriptConstructionFailed", "construction", e))?;
        let handle = self.admit(result.canonical_sdf, json!({"kind":"rhai_utf8","script":script,
            "script_sha256":format!("{:x}",Sha256::digest(script.as_bytes())),
            "external_project_context":"none; standalone script only","source":"ScriptResult::canonical_sdf"}), scale)?;
        let geometry = self.objects.get_mut(&handle).ok_or_else(||
            Failure::new("InvalidOrReleasedHandle", "construction", handle))?;
        geometry.rhai_expression_issuer = true;
        Ok(handle)
    }
    pub fn dispatch(&mut self, request: Value) -> Result<Value> {
        #[derive(Deserialize)]
        #[serde(tag = "op", rename_all = "snake_case", deny_unknown_fields)]
        enum Request {
            LoadRhai {
                script: String,
                metres_per_model_unit: f64,
            },
            Describe {
                handle: u64,
            },
            Query {
                handle: u64,
                points_m: Vec<[f64; 3]>,
            },
            SurfaceSeed {
                handle: u64,
                padding_m: f64,
                spacing_m: f64,
                maximum_queries: u64,
                timeout_seconds: f64,
            },
            Discover {
                handle: u64,
            },
            RegionalExpression {
                handle: u64,
            },
            StoredTopology {
                handle: u64,
            },
            StoredSpline {
                handle: u64,
            },
            NumericalProject {
                handle: u64,
                points_m: Vec<[f64; 3]>,
            },
            NumericalNormal {
                handle: u64,
                points_m: Vec<[f64; 3]>,
            },
            FeatureInventory {
                handle: u64,
            },
            FeatureEvaluate {
                handle: u64,
                requests: Vec<queries::FeatureEvaluation>,
            },
            FeatureQuery {
                handle: u64,
                points_m: Vec<[f64; 3]>,
                options: Option<queries::FeatureOptions>,
            },
            Release {
                handle: u64,
            },
        }
        let request: Request = serde_json::from_value(request)
            .map_err(|e| Failure::new("InvalidRequest", "protocol", e))?;
        match request {
            Request::LoadRhai {
                script,
                metres_per_model_unit,
            } => {
                let t = std::time::Instant::now();
                let h = self.load_rhai(&script, metres_per_model_unit)?;
                Ok(
                    json!({"handle":h,"description":self.get(h)?.description,"load_seconds":t.elapsed().as_secs_f64()}),
                )
            }
            Request::Describe { handle } => Ok(self.get(handle)?.description.clone()),
            Request::SurfaceSeed { handle, padding_m, spacing_m, maximum_queries, timeout_seconds } =>
                self.get(handle)?.surface_seed(padding_m, spacing_m, maximum_queries, timeout_seconds),
            Request::Discover { handle } => self.get(handle)?.discover(),
            Request::RegionalExpression { handle } => self.get(handle)?.regional_expression(),
            Request::StoredTopology { handle } => self.get(handle)?.stored_topology(),
            Request::StoredSpline { handle } => self.get(handle)?.stored_spline(),
            Request::NumericalProject { handle, points_m } => {
                self.get(handle)?.numerical(&points_m, true)
            }
            Request::NumericalNormal { handle, points_m } => {
                self.get(handle)?.numerical(&points_m, false)
            }
            Request::FeatureInventory { handle } => self.get(handle)?.inventory(),
            Request::FeatureEvaluate { handle, requests } => {
                self.get(handle)?.evaluate_features(&requests)
            }
            Request::FeatureQuery {
                handle,
                points_m,
                options,
            } => self.get(handle)?.classify_features(&points_m, options),
            Request::Query { handle, points_m } => {
                let t = std::time::Instant::now();
                let values = self.get(handle)?.query(&points_m)?;
                Ok(json!({"values_m":values,"query_seconds":t.elapsed().as_secs_f64()}))
            }
            Request::Release { handle } => {
                self.release(handle)?;
                Ok(json!({"released":handle}))
            }
        }
    }
    pub fn guarded(&mut self, request: Value) -> Value {
        match catch_unwind(AssertUnwindSafe(|| self.dispatch(request))) {
            Ok(Ok(value)) => json!({"ok":true,"result":value}),
            Ok(Err(error)) => json!({"ok":false,"error":error}),
            Err(_) => {
                self.objects.clear();
                json!({"ok":false,"error":Failure::new("KernelPanic", "native_host", "all handles invalidated; restart host")})
            }
        }
    }
}

/// Sequential protocol owner: no registry mutation or concurrent query races.
/// No unsafe ABI, pointers, or Rust exceptions cross the process boundary.
pub fn serve(mut registry: Registry, initial_handle: Option<u64>) -> std::io::Result<()> {
    let stdin = std::io::stdin();
    let mut input = stdin.lock();
    let stdout = std::io::stdout();
    let mut output = stdout.lock();
    writeln!(
        output,
        "{}",
        json!({"ready":"V4CanonicalKernel/1","initial_handle":initial_handle,"kernel_source_sha256":SOURCE_SHA256})
    )?;
    output.flush()?;
    loop {
        let mut line = Vec::new();
        let n =
            std::io::Read::take(&mut input, (MAX_LINE + 1) as u64).read_until(b'\n', &mut line)?;
        if n == 0 {
            break;
        }
        if n > MAX_LINE {
            writeln!(
                output,
                "{}",
                json!({"ok":false,"error":Failure::new("RequestTooLarge","protocol",MAX_LINE)})
            )?;
            output.flush()?;
            break;
        }
        let answer = match serde_json::from_slice(&line) {
            Ok(value) => registry.guarded(value),
            Err(e) => json!({"ok":false,"error":Failure::new("InvalidJson","protocol",e)}),
        };
        let encoded = serde_json::to_vec(&answer)?;
        if encoded.len() + 1 > MAX_LINE {
            writeln!(
                output,
                "{}",
                json!({"ok":false,"error":Failure::new("ResponseTooLarge","protocol",MAX_LINE)})
            )?;
        } else {
            output.write_all(&encoded)?;
            output.write_all(b"\n")?;
        }
        output.flush()?;
    }
    Ok(())
}
