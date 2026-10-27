use crate::sdf::Sdf;
use crate::sdf::conditioning::{
    BoundQuality, ConditionedGeometryKernel, SdfBounds, SdfFeatureRegion, SdfNodeMetadata,
};
use crate::sdf::procedural_features::{
    ProceduralFeatureBatchResult, ProceduralFeatureInventory, ProceduralFeatureQueryOptions,
    ProceduralFeatureQueryResult,
};
use crate::sdf::provenance::{
    CsgProvenanceTrace, CsgTiePolicy, CsgTraceMode, CsgTraceSupport, SdfProvenanceOptions,
};
use glam::Vec3;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use std::time::Instant;

pub const SDF_CFD_QUERY_SCHEMA_VERSION: &str = "sdf_cfd_query.v1";

const DEFAULT_GRADIENT_STEP: f32 = 1.0e-3;
const DEFAULT_PROJECTION_TOLERANCE: f32 = 1.0e-4;
const DEFAULT_PROJECTION_MAX_ITERATIONS: usize = 32;
const GRADIENT_NORM_EPSILON: f64 = 1.0e-8;

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum QueryValueStatus {
    Native,
    Derived,
    Heuristic,
    Partial,
    Unsupported,
    Failed,
    NotRequested,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct FieldCapability {
    pub status: QueryValueStatus,
    pub implementation_source: String,
    pub limitations: Vec<String>,
    pub numerical_method: Option<String>,
    pub expected_cost: String,
    pub deterministic: bool,
    pub test_coverage: String,
}

impl FieldCapability {
    pub fn new(
        status: QueryValueStatus,
        implementation_source: impl Into<String>,
        expected_cost: impl Into<String>,
    ) -> Self {
        Self {
            status,
            implementation_source: implementation_source.into(),
            limitations: Vec::new(),
            numerical_method: None,
            expected_cost: expected_cost.into(),
            deterministic: true,
            test_coverage: "covered_by_sdf_cfd_query_tests".to_string(),
        }
    }

    pub fn with_limitation(mut self, limitation: impl Into<String>) -> Self {
        self.limitations.push(limitation.into());
        self
    }

    pub fn with_method(mut self, method: impl Into<String>) -> Self {
        self.numerical_method = Some(method.into());
        self
    }

    pub fn unsupported(reason: impl Into<String>) -> Self {
        Self::new(QueryValueStatus::Unsupported, "not_available", "none").with_limitation(reason)
    }

    pub fn not_requested() -> Self {
        Self::new(
            QueryValueStatus::NotRequested,
            "caller_request_mask",
            "none",
        )
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct QueryFieldCapabilities {
    pub position: FieldCapability,
    pub signed_distance: FieldCapability,
    pub raw_gradient: FieldCapability,
    pub unit_normal: FieldCapability,
    pub gradient_magnitude: FieldCapability,
    pub projection: FieldCapability,
    pub curvature: FieldCapability,
    pub nearest_entity: FieldCapability,
    pub competing_entity: FieldCapability,
    pub patch: FieldCapability,
    pub region: FieldCapability,
    pub feature: FieldCapability,
    pub provenance: FieldCapability,
    pub hard_csg_provenance: FieldCapability,
    pub procedural_features: FieldCapability,
    pub local_coordinates: FieldCapability,
    pub local_feature_size: FieldCapability,
    pub opposing_surface_distance: FieldCapability,
    pub suggested_mesh_size: FieldCapability,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub struct SdfCfdQueryOptions {
    pub distance: bool,
    pub gradient: bool,
    pub projection: bool,
    pub provenance: bool,
    pub hard_csg_provenance: CsgTraceMode,
    pub procedural_features: bool,
    pub csg_tie_policy: CsgTiePolicy,
    pub curvature: bool,
    pub competing_entity: bool,
    pub feature_size: bool,
    pub mesh_sizing: bool,
    pub opposing_surface_distance: bool,
}

impl SdfCfdQueryOptions {
    pub fn distance_only() -> Self {
        Self {
            distance: true,
            gradient: false,
            projection: false,
            provenance: false,
            hard_csg_provenance: CsgTraceMode::None,
            procedural_features: false,
            csg_tie_policy: CsgTiePolicy::default(),
            curvature: false,
            competing_entity: false,
            feature_size: false,
            mesh_sizing: false,
            opposing_surface_distance: false,
        }
    }

    pub fn distance_and_gradient() -> Self {
        Self {
            gradient: true,
            ..Self::distance_only()
        }
    }

    pub fn diagnostic() -> Self {
        Self {
            distance: true,
            gradient: true,
            projection: true,
            provenance: true,
            hard_csg_provenance: CsgTraceMode::Recursive,
            procedural_features: true,
            csg_tie_policy: CsgTiePolicy::default(),
            curvature: true,
            competing_entity: true,
            feature_size: true,
            mesh_sizing: true,
            opposing_surface_distance: true,
        }
    }
}

impl Default for SdfCfdQueryOptions {
    fn default() -> Self {
        Self::distance_and_gradient()
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct SurfaceProjection {
    pub projected_point: Option<[f64; 3]>,
    pub residual_signed_distance: Option<f64>,
    pub iteration_count: Option<usize>,
    pub converged: bool,
    pub displacement_magnitude: Option<f64>,
    pub final_gradient_magnitude: Option<f64>,
    pub failure_reason: Option<String>,
    pub method: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum GeometryEntityKind {
    Component,
    Primitive,
    CsgNode,
    ScriptedObject,
    Region,
    RootSdf,
    Unknown,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct GeometryEntityRef {
    pub stable_id: Option<String>,
    pub label: Option<String>,
    pub kind: GeometryEntityKind,
    pub source: String,
    pub support: QueryValueStatus,
    pub confidence: f64,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct PatchRef {
    pub stable_id: Option<String>,
    pub name: Option<String>,
    pub openfoam_role: Option<String>,
    pub source: String,
    pub support: QueryValueStatus,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct RegionRef {
    pub stable_id: Option<String>,
    pub name: Option<String>,
    pub source: String,
    pub support: QueryValueStatus,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FeatureKind {
    SmoothSurface,
    SharpConvexEdge,
    SharpConcaveEdge,
    Corner,
    TrailingEdge,
    LeadingEdge,
    Blend,
    InletLip,
    PatchBoundary,
    Uncertain,
    Unsupported,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct FeatureClassification {
    pub kind: FeatureKind,
    pub feature_ids: Vec<String>,
    pub source: String,
    pub support: QueryValueStatus,
    pub confidence: f64,
    pub notes: Vec<String>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct CurvatureEstimate {
    pub mean_curvature: Option<f64>,
    pub principal_min: Option<f64>,
    pub principal_max: Option<f64>,
    pub method: String,
    pub step: f64,
    pub support: QueryValueStatus,
    pub confidence: f64,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct BoundsSummary {
    pub min: [f64; 3],
    pub max: [f64; 3],
    pub quality: String,
    pub approximate: bool,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct FeatureRegionSummary {
    pub feature_id: String,
    pub bounds: BoundsSummary,
    pub min_feature_size: f64,
    pub contains_query: bool,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct GeometryProvenance {
    pub support: QueryValueStatus,
    pub root_node_kind: Option<String>,
    pub root_feature_ids: Vec<String>,
    pub dependency_roles: Vec<String>,
    pub feature_regions: Vec<FeatureRegionSummary>,
    pub source: String,
    pub notes: Vec<String>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct QueryConfidence {
    pub overall: f64,
    pub distance: f64,
    pub gradient: f64,
    pub projection: f64,
    pub provenance: f64,
    pub reasons: Vec<String>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct QueryDiagnostics {
    pub success: bool,
    pub errors: Vec<String>,
    pub warnings: Vec<String>,
    pub finite_position: bool,
    pub finite_distance: Option<bool>,
    pub finite_gradient: Option<bool>,
    pub gradient_norm_epsilon: f64,
    pub used_conditioned_kernel: bool,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct SdfCfdQueryResult {
    pub schema_version: String,
    pub position: [f64; 3],
    pub signed_distance: Option<f64>,
    pub raw_gradient: Option<[f64; 3]>,
    pub unit_normal: Option<[f64; 3]>,
    pub gradient_magnitude: Option<f64>,
    pub projection: Option<SurfaceProjection>,
    pub curvature: Option<CurvatureEstimate>,
    pub nearest_entity: Option<GeometryEntityRef>,
    pub competing_entity: Option<GeometryEntityRef>,
    pub patch: Option<PatchRef>,
    pub region: Option<RegionRef>,
    pub feature: FeatureClassification,
    pub provenance: GeometryProvenance,
    pub hard_csg_trace: Option<CsgProvenanceTrace>,
    pub procedural_feature: Option<ProceduralFeatureQueryResult>,
    pub local_coordinates: Option<[f64; 3]>,
    pub local_feature_size: Option<f64>,
    pub opposing_surface_distance: Option<f64>,
    pub suggested_mesh_size: Option<f64>,
    pub confidence: QueryConfidence,
    pub capabilities: QueryFieldCapabilities,
    pub diagnostics: QueryDiagnostics,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct SdfCfdBatchSummary {
    pub input_count: usize,
    pub success_count: usize,
    pub failure_count: usize,
    pub elapsed_ms: f64,
    pub geometry_initialized_once: bool,
    pub options: SdfCfdQueryOptions,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct SdfCfdBatchQueryResult {
    pub schema_version: String,
    pub results: Vec<SdfCfdQueryResult>,
    pub summary: SdfCfdBatchSummary,
}

#[derive(Clone)]
pub struct GeometryQueryKernel {
    sdf: Arc<dyn Sdf>,
    conditioned: Option<ConditionedGeometryKernel>,
    metadata: SdfNodeMetadata,
    gradient_step: f32,
    projection_tolerance: f32,
    projection_max_iterations: usize,
}

impl GeometryQueryKernel {
    pub fn from_sdf(sdf: Arc<dyn Sdf>) -> Self {
        let metadata = sdf.metadata();
        let conditioned = ConditionedGeometryKernel::from_runtime_metadata(Arc::clone(&sdf));
        Self {
            sdf,
            conditioned,
            metadata,
            gradient_step: DEFAULT_GRADIENT_STEP,
            projection_tolerance: DEFAULT_PROJECTION_TOLERANCE,
            projection_max_iterations: DEFAULT_PROJECTION_MAX_ITERATIONS,
        }
    }

    pub fn with_gradient_step(mut self, step: f32) -> Self {
        if step.is_finite() && step > 0.0 {
            self.gradient_step = step;
        }
        self
    }

    pub fn with_projection_settings(mut self, tolerance: f32, max_iterations: usize) -> Self {
        if tolerance.is_finite() && tolerance > 0.0 {
            self.projection_tolerance = tolerance;
        }
        self.projection_max_iterations = max_iterations.max(1);
        self
    }

    pub fn metadata(&self) -> &SdfNodeMetadata {
        &self.metadata
    }

    pub fn has_conditioned_kernel(&self) -> bool {
        self.conditioned.is_some()
    }

    pub fn procedural_feature_inventory(&self) -> Option<ProceduralFeatureInventory> {
        self.sdf.procedural_feature_inventory()
    }

    pub fn evaluate_procedural_feature(
        &self,
        feature_id: &str,
        parameter: f64,
    ) -> Option<ProceduralFeatureQueryResult> {
        self.sdf.evaluate_procedural_feature(feature_id, parameter)
    }

    pub fn query_procedural_features(
        &self,
        position: [f64; 3],
        options: ProceduralFeatureQueryOptions,
    ) -> ProceduralFeatureQueryResult {
        self.sdf
            .query_procedural_feature(vec3_from_f64(position), &options)
    }

    pub fn query_procedural_features_batch(
        &self,
        positions: &[[f64; 3]],
        options: ProceduralFeatureQueryOptions,
    ) -> ProceduralFeatureBatchResult {
        let points = positions
            .iter()
            .copied()
            .map(vec3_from_f64)
            .collect::<Vec<_>>();
        self.sdf.query_procedural_feature_batch(&points, &options)
    }

    pub fn query_point(
        &self,
        position: [f64; 3],
        options: SdfCfdQueryOptions,
    ) -> SdfCfdQueryResult {
        let mut diagnostics = QueryDiagnostics {
            success: true,
            errors: Vec::new(),
            warnings: Vec::new(),
            finite_position: position.iter().all(|v| v.is_finite()),
            finite_distance: None,
            finite_gradient: None,
            gradient_norm_epsilon: GRADIENT_NORM_EPSILON,
            used_conditioned_kernel: self.conditioned.is_some(),
        };

        let point = vec3_from_f64(position);
        if !diagnostics.finite_position {
            diagnostics.success = false;
            diagnostics
                .errors
                .push("query position contains a non-finite coordinate".to_string());
        }

        let provenance_options = SdfProvenanceOptions {
            trace_mode: options.hard_csg_provenance,
            tie_policy: options.csg_tie_policy,
        };
        let sdf_sample =
            if diagnostics.finite_position && options.distance && provenance_options.requested() {
                Some(self.sdf.sample_with_provenance(point, &provenance_options))
            } else {
                None
            };

        let signed_distance = if options.distance && diagnostics.finite_position {
            let value = sdf_sample
                .as_ref()
                .map(|sample| sample.distance)
                .unwrap_or_else(|| self.sdf.distance(point));
            diagnostics.finite_distance = Some(value.is_finite());
            if value.is_finite() {
                Some(value as f64)
            } else {
                diagnostics.success = false;
                diagnostics
                    .errors
                    .push("SDF distance evaluation returned a non-finite value".to_string());
                None
            }
        } else {
            None
        };

        let raw_gradient_vec = if options.gradient && diagnostics.finite_position {
            let value = self.raw_gradient(point);
            let finite = value.is_finite();
            diagnostics.finite_gradient = Some(finite);
            if finite {
                Some(value)
            } else {
                diagnostics.success = false;
                diagnostics
                    .errors
                    .push("gradient evaluation returned a non-finite vector".to_string());
                None
            }
        } else {
            None
        };

        let gradient_magnitude = raw_gradient_vec.map(|g| g.length() as f64);
        let unit_normal = raw_gradient_vec.and_then(|g| {
            let norm = g.length() as f64;
            if norm > GRADIENT_NORM_EPSILON {
                Some(array_from_vec3(g / norm as f32))
            } else {
                diagnostics
                    .warnings
                    .push("gradient magnitude is too small to normalize".to_string());
                None
            }
        });
        let raw_gradient = raw_gradient_vec.map(array_from_vec3);

        let projection = if options.projection && diagnostics.finite_position {
            Some(self.project(point))
        } else {
            None
        };

        let curvature = if options.curvature && diagnostics.finite_position {
            self.curvature(point)
        } else {
            None
        };

        let feature = if options.provenance {
            self.classify_feature(point, signed_distance)
        } else {
            FeatureClassification {
                kind: FeatureKind::Unsupported,
                feature_ids: Vec::new(),
                source: "caller_request_mask".to_string(),
                support: QueryValueStatus::NotRequested,
                confidence: 0.0,
                notes: Vec::new(),
            }
        };

        let provenance = if options.provenance {
            self.provenance(point)
        } else {
            GeometryProvenance {
                support: QueryValueStatus::NotRequested,
                root_node_kind: None,
                root_feature_ids: Vec::new(),
                dependency_roles: Vec::new(),
                feature_regions: Vec::new(),
                source: "caller_request_mask".to_string(),
                notes: Vec::new(),
            }
        };
        let hard_csg_trace = if provenance_options.requested() {
            sdf_sample.and_then(|sample| sample.provenance)
        } else {
            None
        };
        let procedural_feature = if options.procedural_features && diagnostics.finite_position {
            Some(
                self.sdf
                    .query_procedural_feature(point, &ProceduralFeatureQueryOptions::diagnostic()),
            )
        } else {
            None
        };

        let local_feature_size = if options.feature_size && diagnostics.finite_position {
            self.conditioned
                .as_ref()
                .and_then(|kernel| kernel.local_feature_size(point))
                .map(|v| v as f64)
        } else {
            None
        };

        let opposing_surface_distance = if options.opposing_surface_distance
            && diagnostics.finite_position
            && unit_normal.is_some()
        {
            let direction = vec3_from_f64(unit_normal.unwrap());
            self.conditioned.as_ref().and_then(|kernel| {
                kernel
                    .wall_thickness(point, direction, 10_000.0, self.projection_tolerance)
                    .map(|v| v as f64)
            })
        } else {
            None
        };

        let capabilities = self.field_capabilities(
            options,
            raw_gradient.is_some(),
            projection.as_ref().is_some_and(|p| p.converged),
            curvature.is_some(),
            local_feature_size.is_some(),
            opposing_surface_distance.is_some(),
            hard_csg_trace.as_ref(),
            procedural_feature.as_ref(),
        );
        let confidence = query_confidence(
            &diagnostics,
            signed_distance,
            gradient_magnitude,
            projection.as_ref(),
            &provenance,
        );

        SdfCfdQueryResult {
            schema_version: SDF_CFD_QUERY_SCHEMA_VERSION.to_string(),
            position,
            signed_distance,
            raw_gradient,
            unit_normal,
            gradient_magnitude,
            projection,
            curvature,
            nearest_entity: None,
            competing_entity: None,
            patch: None,
            region: None,
            feature,
            provenance,
            hard_csg_trace,
            procedural_feature,
            local_coordinates: None,
            local_feature_size,
            opposing_surface_distance,
            suggested_mesh_size: None,
            confidence,
            capabilities,
            diagnostics,
        }
    }

    pub fn query_batch(
        &self,
        positions: &[[f64; 3]],
        options: SdfCfdQueryOptions,
    ) -> SdfCfdBatchQueryResult {
        let start = Instant::now();
        let results: Vec<_> = positions
            .iter()
            .map(|position| self.query_point(*position, options))
            .collect();
        let success_count = results
            .iter()
            .filter(|result| result.diagnostics.success)
            .count();
        SdfCfdBatchQueryResult {
            schema_version: SDF_CFD_QUERY_SCHEMA_VERSION.to_string(),
            summary: SdfCfdBatchSummary {
                input_count: positions.len(),
                success_count,
                failure_count: positions.len().saturating_sub(success_count),
                elapsed_ms: start.elapsed().as_secs_f64() * 1000.0,
                geometry_initialized_once: true,
                options,
            },
            results,
        }
    }

    fn raw_gradient(&self, point: Vec3) -> Vec3 {
        if let Some(conditioned) = &self.conditioned {
            conditioned.gradient(point, self.gradient_step)
        } else {
            finite_difference_gradient(self.sdf.as_ref(), point, self.gradient_step)
        }
    }

    fn project(&self, point: Vec3) -> SurfaceProjection {
        let Some(conditioned) = &self.conditioned else {
            return SurfaceProjection {
                projected_point: None,
                residual_signed_distance: None,
                iteration_count: None,
                converged: false,
                displacement_magnitude: None,
                final_gradient_magnitude: None,
                failure_reason: Some(
                    "projection requires conditionable metadata and no conditioned kernel is available"
                        .to_string(),
                ),
                method: "ConditionedGeometryKernel::project_to_surface".to_string(),
            };
        };

        match conditioned.project_to_surface(
            point,
            self.projection_tolerance,
            self.projection_max_iterations,
        ) {
            Some(projected) => {
                let residual = self.sdf.distance(projected) as f64;
                let final_gradient = self.raw_gradient(projected).length() as f64;
                SurfaceProjection {
                    projected_point: Some(array_from_vec3(projected)),
                    residual_signed_distance: Some(residual),
                    iteration_count: None,
                    converged: residual.is_finite()
                        && residual.abs() <= self.projection_tolerance as f64,
                    displacement_magnitude: Some((projected - point).length() as f64),
                    final_gradient_magnitude: Some(final_gradient),
                    failure_reason: None,
                    method: "ConditionedGeometryKernel::project_to_surface; existing API does not expose iteration count".to_string(),
                }
            }
            None => SurfaceProjection {
                projected_point: None,
                residual_signed_distance: None,
                iteration_count: None,
                converged: false,
                displacement_magnitude: None,
                final_gradient_magnitude: None,
                failure_reason: Some("existing projection path did not converge".to_string()),
                method: "ConditionedGeometryKernel::project_to_surface".to_string(),
            },
        }
    }

    fn curvature(&self, point: Vec3) -> Option<CurvatureEstimate> {
        let conditioned = self.conditioned.as_ref()?;
        let principal = conditioned.principal_curvature(point, self.gradient_step);
        Some(CurvatureEstimate {
            mean_curvature: Some(conditioned.curvature(point, self.gradient_step) as f64),
            principal_min: Some(principal.minimum as f64),
            principal_max: Some(principal.maximum as f64),
            method: "finite_difference_normal_variation_on_conditioned_kernel".to_string(),
            step: self.gradient_step as f64,
            support: QueryValueStatus::Derived,
            confidence: 0.55,
        })
    }

    fn provenance(&self, point: Vec3) -> GeometryProvenance {
        GeometryProvenance {
            support: QueryValueStatus::Partial,
            root_node_kind: Some(self.metadata.node_kind.clone()),
            root_feature_ids: self.metadata.feature_ids.clone(),
            dependency_roles: self
                .metadata
                .dependencies
                .iter()
                .map(|dependency| dependency.role.clone())
                .collect(),
            feature_regions: self
                .metadata
                .feature_regions
                .iter()
                .map(|region| feature_region_summary(region, point, &self.metadata))
                .collect(),
            source: "SdfNodeMetadata from existing Rust/Rhai SDF graph".to_string(),
            notes: vec![
                "root/dependency metadata is available, but active child provenance is not returned by distance evaluation".to_string(),
                "smooth blend provenance requires weighted multi-source support in a future evaluator extension".to_string(),
            ],
        }
    }

    fn classify_feature(&self, point: Vec3, signed_distance: Option<f64>) -> FeatureClassification {
        let containing: Vec<_> = self
            .metadata
            .feature_regions
            .iter()
            .filter(|region| region.bounds.contains(point))
            .map(|region| region.feature_id.clone())
            .collect();

        if containing.is_empty() {
            let near_surface = signed_distance.is_some_and(|d| d.abs() <= 1.0);
            return FeatureClassification {
                kind: if near_surface {
                    FeatureKind::SmoothSurface
                } else {
                    FeatureKind::Uncertain
                },
                feature_ids: Vec::new(),
                source: "metadata_feature_region_absence".to_string(),
                support: QueryValueStatus::Heuristic,
                confidence: if near_surface { 0.35 } else { 0.15 },
                notes: vec![
                    "no feature region contains the query point; this is not proof of smoothness"
                        .to_string(),
                ],
            };
        }

        let kind = classify_feature_ids(&containing);
        FeatureClassification {
            kind,
            feature_ids: containing,
            source: "SdfNodeMetadata.feature_regions".to_string(),
            support: QueryValueStatus::Partial,
            confidence: 0.55,
            notes: vec![
                "classification is metadata-name based; sharp convex/concave side normals are not yet available".to_string(),
            ],
        }
    }

    fn field_capabilities(
        &self,
        options: SdfCfdQueryOptions,
        gradient_ok: bool,
        projection_ok: bool,
        curvature_ok: bool,
        feature_size_ok: bool,
        opposing_wall_ok: bool,
        hard_csg_trace: Option<&CsgProvenanceTrace>,
        procedural_feature: Option<&ProceduralFeatureQueryResult>,
    ) -> QueryFieldCapabilities {
        let conditioned_available = self.conditioned.is_some();
        QueryFieldCapabilities {
            position: FieldCapability::new(QueryValueStatus::Native, "caller_input", "none"),
            signed_distance: if options.distance {
                FieldCapability::new(
                    QueryValueStatus::Native,
                    "Arc<dyn Sdf>::distance via existing Rust/Rhai evaluator",
                    "one_sdf_evaluation",
                )
            } else {
                FieldCapability::not_requested()
            },
            raw_gradient: if options.gradient {
                let status = if gradient_ok {
                    QueryValueStatus::Derived
                } else {
                    QueryValueStatus::Failed
                };
                FieldCapability::new(
                    status,
                    "ConditionedGeometryKernel::gradient or canonical finite differences",
                    "six_sdf_evaluations_or_conditioned_samples",
                )
                .with_method("central finite difference")
            } else {
                FieldCapability::not_requested()
            },
            unit_normal: if options.gradient {
                FieldCapability::new(
                    if gradient_ok {
                        QueryValueStatus::Derived
                    } else {
                        QueryValueStatus::Failed
                    },
                    "normalized raw_gradient",
                    "gradient_cost",
                )
                .with_limitation("undefined when gradient magnitude is near zero")
            } else {
                FieldCapability::not_requested()
            },
            gradient_magnitude: if options.gradient {
                FieldCapability::new(
                    if gradient_ok {
                        QueryValueStatus::Derived
                    } else {
                        QueryValueStatus::Failed
                    },
                    "norm(raw_gradient)",
                    "gradient_cost",
                )
            } else {
                FieldCapability::not_requested()
            },
            projection: if options.projection {
                if projection_ok {
                    FieldCapability::new(
                        QueryValueStatus::Derived,
                        "ConditionedGeometryKernel::project_to_surface",
                        "iterative_projection",
                    )
                    .with_limitation("existing projection API does not expose iteration count")
                } else if conditioned_available {
                    FieldCapability::new(
                        QueryValueStatus::Failed,
                        "ConditionedGeometryKernel::project_to_surface",
                        "iterative_projection",
                    )
                } else {
                    FieldCapability::unsupported("projection requires conditionable metadata")
                }
            } else {
                FieldCapability::not_requested()
            },
            curvature: if options.curvature {
                if curvature_ok {
                    FieldCapability::new(
                        QueryValueStatus::Derived,
                        "ConditionedGeometryKernel::curvature/principal_curvature",
                        "multiple_gradient_samples",
                    )
                    .with_method("finite difference of surface normals")
                } else {
                    FieldCapability::unsupported("curvature requires conditioned geometry kernel")
                }
            } else {
                FieldCapability::not_requested()
            },
            nearest_entity: FieldCapability::unsupported(
                "distance evaluation returns only scalar distance, not active component provenance",
            ),
            competing_entity: if options.competing_entity {
                FieldCapability::unsupported(
                    "current min/max CSG evaluation does not expose second-nearest contributor",
                )
            } else {
                FieldCapability::not_requested()
            },
            patch: FieldCapability::unsupported(
                "OpenFOAM boundary patch identity is not preserved in general SDF evaluation",
            ),
            region: FieldCapability::unsupported(
                "material/flow-region identity is not represented by the current SDF trait",
            ),
            feature: if options.provenance {
                FieldCapability::new(
                    QueryValueStatus::Partial,
                    "SdfNodeMetadata.feature_regions",
                    "metadata_scan",
                )
                .with_limitation(
                    "feature side normals and sharp convex/concave identity are unavailable",
                )
            } else {
                FieldCapability::not_requested()
            },
            provenance: if options.provenance {
                FieldCapability::new(
                    QueryValueStatus::Partial,
                    "SdfNodeMetadata root/dependency/feature summaries",
                    "metadata_access",
                )
                .with_limitation("active distance contributor is not available")
            } else {
                FieldCapability::not_requested()
            },
            hard_csg_provenance: if options.hard_csg_provenance != CsgTraceMode::None {
                match hard_csg_trace.map(|trace| trace.support_status) {
                    Some(CsgTraceSupport::Supported) => FieldCapability::new(
                        QueryValueStatus::Native,
                        "Sdf::sample_with_provenance hard-CSG overrides",
                        "one_recursive_boolean_sample",
                    ),
                    Some(CsgTraceSupport::Partial) => FieldCapability::new(
                        QueryValueStatus::Partial,
                        "Sdf::sample_with_provenance hard-CSG overrides",
                        "one_recursive_boolean_sample",
                    )
                    .with_limitation("terminal entity metadata is partial"),
                    Some(CsgTraceSupport::Unsupported) => FieldCapability::new(
                        QueryValueStatus::Unsupported,
                        "Sdf::sample_with_provenance",
                        "one_recursive_boolean_sample",
                    )
                    .with_limitation("trace terminated at unsupported smooth or wrapper node"),
                    Some(CsgTraceSupport::Failed) => FieldCapability::new(
                        QueryValueStatus::Failed,
                        "Sdf::sample_with_provenance",
                        "one_recursive_boolean_sample",
                    ),
                    Some(CsgTraceSupport::NotRequested) | None => FieldCapability::new(
                        QueryValueStatus::Unsupported,
                        "Sdf::sample_with_provenance",
                        "one_recursive_boolean_sample",
                    )
                    .with_limitation("geometry has no supported hard-CSG decision at this point"),
                }
            } else {
                FieldCapability::not_requested()
            },
            procedural_features: if options.procedural_features {
                match procedural_feature.map(|result| result.support_status) {
                    Some(QueryValueStatus::Native) => FieldCapability::new(
                        QueryValueStatus::Native,
                        "Sdf::query_procedural_feature",
                        "one_procedural_feature_query",
                    ),
                    Some(QueryValueStatus::Partial) => FieldCapability::new(
                        QueryValueStatus::Partial,
                        "Sdf::query_procedural_feature",
                        "one_procedural_feature_query",
                    ),
                    Some(QueryValueStatus::Failed) => FieldCapability::new(
                        QueryValueStatus::Failed,
                        "Sdf::query_procedural_feature",
                        "one_procedural_feature_query",
                    ),
                    _ => FieldCapability::unsupported(
                        "geometry does not expose procedural feature metadata",
                    ),
                }
            } else {
                FieldCapability::not_requested()
            },
            local_coordinates: FieldCapability::unsupported(
                "component-local coordinates are not exposed by the Sdf trait or metadata",
            ),
            local_feature_size: if options.feature_size {
                if feature_size_ok {
                    FieldCapability::new(
                        QueryValueStatus::Partial,
                        "ConditionedGeometryKernel::local_feature_size",
                        "metadata_scan",
                    )
                } else {
                    FieldCapability::unsupported("local feature size requires conditioned metadata")
                }
            } else {
                FieldCapability::not_requested()
            },
            opposing_surface_distance: if options.opposing_surface_distance {
                if opposing_wall_ok {
                    FieldCapability::new(
                        QueryValueStatus::Heuristic,
                        "ConditionedGeometryKernel::wall_thickness",
                        "two_ray_intersections",
                    )
                    .with_limitation("not a true nearest competing component query")
                } else {
                    FieldCapability::unsupported(
                        "opposing surface query needs a valid normal and conditioned kernel",
                    )
                }
            } else {
                FieldCapability::not_requested()
            },
            suggested_mesh_size: if options.mesh_sizing {
                FieldCapability::unsupported(
                    "CFD mesh sizing policy has not been defined in Phase 0",
                )
            } else {
                FieldCapability::not_requested()
            },
        }
    }
}

fn finite_difference_gradient(sdf: &dyn Sdf, point: Vec3, step: f32) -> Vec3 {
    let h = step.max(1.0e-6);
    Vec3::new(
        (sdf.distance(point + Vec3::X * h) - sdf.distance(point - Vec3::X * h)) / (2.0 * h),
        (sdf.distance(point + Vec3::Y * h) - sdf.distance(point - Vec3::Y * h)) / (2.0 * h),
        (sdf.distance(point + Vec3::Z * h) - sdf.distance(point - Vec3::Z * h)) / (2.0 * h),
    )
}

fn vec3_from_f64(position: [f64; 3]) -> Vec3 {
    Vec3::new(position[0] as f32, position[1] as f32, position[2] as f32)
}

fn array_from_vec3(value: Vec3) -> [f64; 3] {
    [value.x as f64, value.y as f64, value.z as f64]
}

fn bounds_summary(bounds: &SdfBounds, metadata: &SdfNodeMetadata) -> BoundsSummary {
    BoundsSummary {
        min: array_from_vec3(bounds.min),
        max: array_from_vec3(bounds.max),
        quality: bound_quality_label(metadata.bound_quality).to_string(),
        approximate: metadata.bounds_are_approximate,
    }
}

fn feature_region_summary(
    region: &SdfFeatureRegion,
    point: Vec3,
    metadata: &SdfNodeMetadata,
) -> FeatureRegionSummary {
    FeatureRegionSummary {
        feature_id: region.feature_id.clone(),
        bounds: bounds_summary(&region.bounds, metadata),
        min_feature_size: region.min_feature_size as f64,
        contains_query: region.bounds.contains(point),
    }
}

fn bound_quality_label(quality: BoundQuality) -> &'static str {
    match quality {
        BoundQuality::Exact => "exact",
        BoundQuality::Conservative => "conservative",
        BoundQuality::Estimated => "estimated",
        BoundQuality::Unknown => "unknown",
    }
}

fn classify_feature_ids(feature_ids: &[String]) -> FeatureKind {
    let joined = feature_ids.join(" ").to_ascii_lowercase();
    if joined.contains("leading") || joined.contains(".le") {
        FeatureKind::LeadingEdge
    } else if joined.contains("trailing") || joined.contains(".te") {
        FeatureKind::TrailingEdge
    } else if joined.contains("inlet") && (joined.contains("lip") || joined.contains("mouth")) {
        FeatureKind::InletLip
    } else if joined.contains("blend") || joined.contains("junction") {
        FeatureKind::Blend
    } else {
        FeatureKind::Uncertain
    }
}

fn query_confidence(
    diagnostics: &QueryDiagnostics,
    signed_distance: Option<f64>,
    gradient_magnitude: Option<f64>,
    projection: Option<&SurfaceProjection>,
    provenance: &GeometryProvenance,
) -> QueryConfidence {
    let distance_conf: f64 = if signed_distance.is_some() { 1.0 } else { 0.0 };
    let gradient_conf = gradient_magnitude
        .map(|norm| {
            if norm > GRADIENT_NORM_EPSILON {
                0.85
            } else {
                0.1
            }
        })
        .unwrap_or(0.0);
    let projection_conf = projection
        .map(|projection| if projection.converged { 0.8 } else { 0.0 })
        .unwrap_or(0.0);
    let provenance_conf = match provenance.support {
        QueryValueStatus::Partial => 0.35,
        QueryValueStatus::Native => 1.0,
        _ => 0.0,
    };
    let mut reasons = diagnostics.errors.clone();
    reasons.extend(diagnostics.warnings.clone());
    if provenance.support == QueryValueStatus::Partial {
        reasons.push("provenance is metadata-only and lacks active child identity".to_string());
    }
    let overall = if diagnostics.success {
        distance_conf
            .max(gradient_conf)
            .max(projection_conf)
            .max(0.5)
    } else {
        0.0
    };
    QueryConfidence {
        overall,
        distance: distance_conf,
        gradient: gradient_conf,
        projection: projection_conf,
        provenance: provenance_conf,
        reasons,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::booleans::Union;
    use crate::sdf::primitives::Sphere;
    use crate::sdf::transforms::Translate;

    fn sphere_kernel() -> GeometryQueryKernel {
        GeometryQueryKernel::from_sdf(Arc::new(Sphere::new(5.0)))
    }

    #[test]
    fn distance_only_respects_request_mask() {
        let result =
            sphere_kernel().query_point([6.0, 0.0, 0.0], SdfCfdQueryOptions::distance_only());
        assert_eq!(result.signed_distance.unwrap(), 1.0);
        assert!(result.raw_gradient.is_none());
        assert_eq!(
            result.capabilities.raw_gradient.status,
            QueryValueStatus::NotRequested
        );
    }

    #[test]
    fn gradient_and_unit_normal_are_populated() {
        let result = sphere_kernel()
            .query_point([6.0, 0.0, 0.0], SdfCfdQueryOptions::distance_and_gradient());
        let gradient = result.raw_gradient.expect("gradient should be populated");
        assert!((gradient[0] - 1.0).abs() < 1.0e-3, "{gradient:?}");
        assert!(gradient[1].abs() < 1.0e-3);
        assert!(gradient[2].abs() < 1.0e-3);
        assert!((result.gradient_magnitude.unwrap() - 1.0).abs() < 1.0e-3);
        assert_eq!(result.unit_normal.unwrap()[0], 1.0);
    }

    #[test]
    fn invalid_gradient_does_not_fabricate_normal() {
        let result = sphere_kernel()
            .query_point([0.0, 0.0, 0.0], SdfCfdQueryOptions::distance_and_gradient());
        assert!(result.gradient_magnitude.unwrap() <= GRADIENT_NORM_EPSILON);
        assert!(result.unit_normal.is_none());
        assert!(!result.diagnostics.warnings.is_empty());
    }

    #[test]
    fn projection_success_is_reported() {
        let result = sphere_kernel().query_point([6.0, 0.0, 0.0], SdfCfdQueryOptions::diagnostic());
        let projection = result.projection.expect("projection should be requested");
        assert!(projection.converged, "{projection:?}");
        assert!(projection.residual_signed_distance.unwrap().abs() <= 1.0e-3);
    }

    #[test]
    fn projection_failure_is_represented() {
        let kernel = sphere_kernel().with_projection_settings(1.0e-9, 1);
        let result = kernel.query_point([100.0, 0.0, 0.0], SdfCfdQueryOptions::diagnostic());
        let projection = result.projection.expect("projection should be requested");
        if !projection.converged {
            assert!(projection.failure_reason.is_some());
        }
    }

    #[test]
    fn capability_metadata_is_explicit() {
        let result = sphere_kernel().query_point([6.0, 0.0, 0.0], SdfCfdQueryOptions::diagnostic());
        assert_eq!(
            result.capabilities.patch.status,
            QueryValueStatus::Unsupported
        );
        assert_eq!(
            result.capabilities.provenance.status,
            QueryValueStatus::Partial
        );
        assert_eq!(
            result.capabilities.suggested_mesh_size.status,
            QueryValueStatus::Unsupported
        );
    }

    #[test]
    fn result_serializes_with_schema_version() {
        let result = sphere_kernel().query_point([6.0, 0.0, 0.0], SdfCfdQueryOptions::diagnostic());
        let json = serde_json::to_string(&result).expect("query result should serialize");
        assert!(json.contains(SDF_CFD_QUERY_SCHEMA_VERSION));
        let decoded: SdfCfdQueryResult =
            serde_json::from_str(&json).expect("query result should deserialize");
        assert_eq!(decoded.schema_version, SDF_CFD_QUERY_SCHEMA_VERSION);
    }

    #[test]
    fn batch_preserves_order_and_summarizes_failures() {
        let points = [[6.0, 0.0, 0.0], [0.0, 0.0, 0.0], [7.0, 0.0, 0.0]];
        let batch =
            sphere_kernel().query_batch(&points, SdfCfdQueryOptions::distance_and_gradient());
        assert_eq!(batch.results.len(), points.len());
        assert_eq!(batch.results[0].position, points[0]);
        assert_eq!(batch.results[2].position, points[2]);
        assert_eq!(batch.summary.input_count, points.len());
        assert!(batch.summary.geometry_initialized_once);
    }

    #[test]
    fn distance_only_does_not_request_hard_csg_trace() {
        let left = Arc::new(Translate::new(
            Arc::new(Sphere::new(1.0)),
            Vec3::new(-5.0, 0.0, 0.0),
        ));
        let right = Arc::new(Translate::new(
            Arc::new(Sphere::new(1.0)),
            Vec3::new(5.0, 0.0, 0.0),
        ));
        let kernel = GeometryQueryKernel::from_sdf(Arc::new(Union::new(left, right)));
        let result = kernel.query_point([-5.0, 0.0, 0.0], SdfCfdQueryOptions::distance_only());
        assert!(result.hard_csg_trace.is_none());
        assert_eq!(
            result.capabilities.hard_csg_provenance.status,
            QueryValueStatus::NotRequested
        );
    }

    #[test]
    fn diagnostic_query_reports_hard_csg_trace() {
        let left = Arc::new(Translate::new(
            Arc::new(Sphere::new(1.0)),
            Vec3::new(-5.0, 0.0, 0.0),
        ));
        let right = Arc::new(Translate::new(
            Arc::new(Sphere::new(1.0)),
            Vec3::new(5.0, 0.0, 0.0),
        ));
        let kernel = GeometryQueryKernel::from_sdf(Arc::new(Union::new(left, right)));
        let result = kernel.query_point([-5.0, 0.0, 0.0], SdfCfdQueryOptions::diagnostic());
        let trace = result.hard_csg_trace.expect("hard CSG trace should exist");
        assert_eq!(trace.decisions.len(), 1);
        assert_eq!(
            result.capabilities.hard_csg_provenance.status,
            QueryValueStatus::Partial
        );
    }
}
