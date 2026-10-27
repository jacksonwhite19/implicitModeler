//! Scalar-only stock marching-cubes proposal. Final surface acceptance is external.
use crate::{Failure, Geometry, MAX_LINE, Result, SOURCE_SHA256};
use glam::Vec3;
use implicit_cad::{mesh::marching_cubes::extract_scalar_seed_canonical_edges_unfiltered,
    sdf::{Sdf, conditioning::SdfNodeMetadata}};
use serde_json::{Value, json};
use std::{sync::atomic::{AtomicBool, AtomicU64, Ordering}, time::Instant};

// The same source Arc owns every sample. No alternate field or geometry parser.
struct SampleOwner {
    sdf: std::sync::Arc<dyn Sdf>,
    count: AtomicU64,
    evaluations: AtomicU64,
    nonfinite: AtomicBool,
    timed_out: AtomicBool,
    budget_exceeded: AtomicBool,
    maximum_queries: u64,
    start: Instant,
    timeout: f64,
}
impl Sdf for SampleOwner {
    fn distance(&self, point: Vec3) -> f32 {
        let attempt = self.count.fetch_add(1, Ordering::Relaxed);
        if attempt >= self.maximum_queries {
            self.budget_exceeded.store(true, Ordering::Relaxed);
            return f32::NAN;
        }
        if self.start.elapsed().as_secs_f64() > self.timeout {
            self.timed_out.store(true, Ordering::Relaxed);
            return f32::NAN;
        }
        self.evaluations.fetch_add(1, Ordering::Relaxed);
        let value = self.sdf.distance(point);
        if !value.is_finite() { self.nonfinite.store(true, Ordering::Relaxed); }
        value
    }
    fn metadata(&self) -> SdfNodeMetadata { self.sdf.metadata() }
}

fn diagnostics(stage: &str, requested: Option<u64>, sampler: Option<&SampleOwner>) -> Value {
    json!({"schema":"V4ScalarSeedDiagnostics/1", "stage":stage,
        "requested_scalar_grid_attempts":requested,
        "scalar_grid_attempts":sampler.map_or(0, |s| s.count.load(Ordering::Relaxed)),
        "scalar_field_evaluations":sampler.map_or(0, |s| s.evaluations.load(Ordering::Relaxed)),
        "conditioning_attempts":0,"normal_stencil_attempts":0,"projection_attempts":0,
        "budget_exceeded":sampler.is_some_and(|s| s.budget_exceeded.load(Ordering::Relaxed)),
        "timed_out":sampler.is_some_and(|s| s.timed_out.load(Ordering::Relaxed)),
        "nonfinite_field":sampler.is_some_and(|s| s.nonfinite.load(Ordering::Relaxed)),
        "geometry_acceptance":false})
}

impl Geometry {
    pub fn surface_seed(&self, padding_m: f64, spacing_m: f64,
                        maximum_queries: u64, timeout_seconds: f64) -> Result<Value> {
        self.surface_seed_controls(padding_m, Some(spacing_m), None, maximum_queries, timeout_seconds)
    }
    fn seed_plan_data(&self, padding_m:f64, spacing_m:Option<f64>, cells_across:Option<u32>,
                      maximum_queries:u64, timeout_seconds:f64) -> Result<(Value,Vec3,Vec3)> {
        let fail=|code:&str,message:&str| Failure {code:code.into(),stage:"surface_seed".into(),
            geometry_identity:self.description["geometry_identity"].as_str().map(str::to_owned),
            detail:json!({"message":message,"seed_diagnostics":diagnostics("controls_and_bounds",None,None)}).to_string()};
        let bounds=self.canonical.metadata().support_bounds
            .ok_or_else(||fail("StockSeedBoundsUnavailable","No native support box; no fallback search"))?;
        if !bounds.is_valid() || !padding_m.is_finite() || padding_m<=0. {
            return Err(fail("InvalidSurfaceSeedControls","Finite native bounds and positive padding required"));
        }
        let padding=(padding_m/self.metres_per_model_unit) as f32;
        let low=bounds.min-Vec3::splat(padding);let high=bounds.max+Vec3::splat(padding);
        let plan=crate::seed_admission::plan(low,high,self.metres_per_model_unit,padding_m,spacing_m,cells_across,maximum_queries,timeout_seconds)
            .map_err(|message|fail("InvalidSurfaceSeedControls",&message))?;
        Ok((plan,low,high))
    }
    pub fn surface_seed_plan(&self,padding_m:f64,spacing_m:Option<f64>,cells_across:Option<u32>,
                             maximum_queries:u64,timeout_seconds:f64)->Result<Value> {
        let (plan,_,_)=self.seed_plan_data(padding_m,spacing_m,cells_across,maximum_queries,timeout_seconds)?;
        Ok(json!({"schema":"V4CanonicalQueries/1","operation":"surface_seed_plan",
            "geometry_identity":self.description["geometry_identity"],"kernel_source_sha256":SOURCE_SHA256,"data":plan}))
    }
    pub fn surface_seed_controls(&self,padding_m:f64,spacing_m:Option<f64>,cells_across:Option<u32>,
                                 maximum_queries:u64,timeout_seconds:f64)->Result<Value> {
        let (plan,low,high)=self.seed_plan_data(padding_m,spacing_m,cells_across,maximum_queries,timeout_seconds)?;
        let grid_points=plan["predicted_grid_samples"].as_u64();
        let fail=|code:&str,message:String,data:Value| Failure {code:code.into(),stage:"surface_seed".into(),
            geometry_identity:self.description["geometry_identity"].as_str().map(str::to_owned),
            detail:json!({"message":message,"seed_plan":plan,"seed_diagnostics":data}).to_string()};
        if plan["admitted"] != true {
            return Err(fail(plan["reason"].as_str().unwrap(),"Requested grid refused before allocation or scalar evaluation; change explicit controls/budget, no automatic coarsening".into(),diagnostics("grid_preflight",grid_points,None)));
        }
        let grid_points=grid_points.unwrap();let resolution=plan["resolution"].as_u64().unwrap() as u32;
        let extent=high-low;
        let sampler = SampleOwner { sdf:self.canonical.clone(), count:AtomicU64::new(0),
            evaluations:AtomicU64::new(0), nonfinite:AtomicBool::new(false),
            timed_out:AtomicBool::new(false), budget_exceeded:AtomicBool::new(false),
            maximum_queries, start:Instant::now(), timeout:timeout_seconds };
        // Same stock lookup tables, interpolation and canonical edge cache.
        // Preserve connectivity: neither small-area deletion nor approximate
        // coordinate welding is a topology-preserving simplification.
        // Source projection/labels/orientation already belong to labelled_seed.
        let mesh = extract_scalar_seed_canonical_edges_unfiltered(&sampler, low, high, resolution);
        let stage_counts = diagnostics("scalar_extraction_complete", Some(grid_points), Some(&sampler));
        if sampler.timed_out.load(Ordering::Relaxed) || sampler.start.elapsed().as_secs_f64() > timeout_seconds {
            return Err(fail("HostTimeout", "Scalar seed exceeded its explicit deadline; no proposal accepted".into(), stage_counts));
        }
        if sampler.nonfinite.load(Ordering::Relaxed) {
            return Err(fail("NonfiniteFieldResult", "Scalar seed field sample is nonfinite".into(), stage_counts));
        }
        if sampler.budget_exceeded.load(Ordering::Relaxed) {
            return Err(fail("QueryBudgetExceeded", "Scalar seed exhausted its explicit query cap; no proposal accepted".into(), stage_counts));
        }
        let calls = sampler.count.load(Ordering::Relaxed);
        let evaluations = sampler.evaluations.load(Ordering::Relaxed);
        if calls != grid_points || evaluations != grid_points {
            return Err(fail("StockSeedQueryAccountingChanged", format!("expected {grid_points}, attempted {calls}, evaluated {evaluations}"), stage_counts));
        }
        if mesh.vertices.is_empty() || mesh.indices.is_empty() || mesh.indices.len() % 3 != 0
            || mesh.vertices.iter().any(|v| !v.position.iter().all(|x| x.is_finite()))
            || mesh.indices.iter().any(|i| *i as usize >= mesh.vertices.len()) {
            return Err(fail("InvalidSurfaceSeedOutput", "Nonempty finite indexed triangles required".into(), stage_counts));
        }
        let coordinates: Vec<[f64; 3]> = mesh.vertices.iter()
            .map(|v| v.position.map(|x| f64::from(x) * self.metres_per_model_unit)).collect();
        let triangles: Vec<[u32; 3]> = mesh.indices.chunks_exact(3)
            .map(|t| [t[0],t[1],t[2]]).collect();
        let packet = json!({"schema":"V4CanonicalQueries/1", "operation":"surface_seed",
            "geometry_identity":self.description["geometry_identity"],"kernel_source_sha256":SOURCE_SHA256,
            "data":{"route":"CanonicalMarchingCubesSeed/1", "producer":"V4StockScalarConstructionSeed/1",
                "continuous_acceptance":false,"volume_refinement":false,"seed_projection":"NotRun",
                "seed_geometry_audit":"NotRun","full_grid_normals":"NotRun",
                "handoff_owner":"general_sdf_binding.labelled_seed: source projection, labels and orientation; shared MMGS and final acceptance",
                "padding_m_requested":padding_m,"spacing_m_requested":spacing_m,"cells_across_requested":cells_across,"seed_plan":plan,
                "bounds_m_realized":[low.to_array().map(|v| f64::from(v)*self.metres_per_model_unit), high.to_array().map(|v| f64::from(v)*self.metres_per_model_unit)],
                "grid_step_m":(extent/resolution as f32).to_array().map(|v| f64::from(v)*self.metres_per_model_unit),
                "resolution":resolution,"grid_points":grid_points,"grid_scalar_evaluations":grid_points,
                "scalar_evaluations":evaluations,"stage_counts":stage_counts,
                "stock_cleanup":"NotRun",
                "connectivity":"CanonicalGridEdgeIndicesPreserved/1",
                "small_area_deletion":"NotRun","approximate_coordinate_weld":"NotRun",
                "extractor":"implicit_cad::mesh::marching_cubes::extract_scalar_seed_canonical_edges_unfiltered",
                "native_seconds":sampler.start.elapsed().as_secs_f64(),"coordinates_m":coordinates,"triangles":triangles}});
        if serde_json::to_vec(&json!({"ok":true,"result":&packet})).unwrap().len() + 1 > MAX_LINE {
            return Err(fail("ResponseTooLarge", "Scalar seed exceeds existing 8 MiB protocol limit".into(),
                diagnostics("packet_serialization", Some(grid_points), Some(&sampler))));
        }
        if sampler.start.elapsed().as_secs_f64() > timeout_seconds {
            sampler.timed_out.store(true, Ordering::Relaxed);
            return Err(fail("HostTimeout", "Scalar seed cleanup/serialization exceeded its deadline".into(),
                diagnostics("packet_serialization", Some(grid_points), Some(&sampler))));
        }
        Ok(packet)
    }
}
