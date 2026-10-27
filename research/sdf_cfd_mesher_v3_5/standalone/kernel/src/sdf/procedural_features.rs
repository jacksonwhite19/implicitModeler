use crate::sdf::cfd_query::QueryValueStatus;
use glam::Vec3;
use serde::{Deserialize, Serialize};
use std::time::Instant;

pub const PROCEDURAL_FEATURE_SCHEMA_VERSION: &str = "procedural_feature_query.v1";

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct ProceduralFeatureQueryOptions {
    pub closest_feature: bool,
    pub all_features_within_tolerance: bool,
    pub surface_sector: bool,
    pub local_parameters: bool,
    pub tolerance: f64,
}

impl ProceduralFeatureQueryOptions {
    pub fn diagnostic() -> Self {
        Self {
            closest_feature: true,
            all_features_within_tolerance: true,
            surface_sector: true,
            local_parameters: true,
            tolerance: 6.0,
        }
    }
}

impl Default for ProceduralFeatureQueryOptions {
    fn default() -> Self {
        Self::diagnostic()
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct ProceduralFeatureBatchResult {
    pub schema_version: String,
    pub results: Vec<ProceduralFeatureQueryResult>,
    pub summary: ProceduralFeatureBatchSummary,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct ProceduralFeatureBatchSummary {
    pub input_count: usize,
    pub success_count: usize,
    pub failure_count: usize,
    pub elapsed_ms: f64,
    pub ordering_preserved: bool,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct ProceduralFeatureInventory {
    pub schema_version: String,
    pub component_id: String,
    pub component_kind: String,
    pub features: Vec<ProceduralFeatureDefinition>,
    pub sectors: Vec<SurfaceSectorDefinition>,
    pub junctions: Vec<ProceduralFeatureJunction>,
    pub capabilities: ProceduralFeatureCapabilities,
}

impl ProceduralFeatureInventory {
    pub fn unsupported(component_kind: impl Into<String>, reason: impl Into<String>) -> Self {
        Self {
            schema_version: PROCEDURAL_FEATURE_SCHEMA_VERSION.to_string(),
            component_id: "unsupported".to_string(),
            component_kind: component_kind.into(),
            features: Vec::new(),
            sectors: Vec::new(),
            junctions: Vec::new(),
            capabilities: ProceduralFeatureCapabilities {
                support_status: QueryValueStatus::Unsupported,
                inventory: QueryValueStatus::Unsupported,
                feature_evaluation: QueryValueStatus::Unsupported,
                closest_feature: QueryValueStatus::Unsupported,
                surface_sector: QueryValueStatus::Unsupported,
                local_parameters: QueryValueStatus::Unsupported,
                transform_propagation: QueryValueStatus::Unsupported,
                limitations: vec![reason.into()],
            },
        }
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct ProceduralFeatureDefinition {
    pub feature_id: String,
    pub feature_type: ProceduralFeatureType,
    pub support_status: QueryValueStatus,
    pub curve_domain: Option<[f64; 2]>,
    pub closed: bool,
    pub endpoint_roles: Vec<FeatureEndpointRole>,
    pub connected_feature_ids: Vec<String>,
    pub source: String,
    pub available_parameters: Vec<String>,
    pub tangent_available: bool,
    pub normal_sector_available: bool,
    pub known_singularities: Vec<String>,
    pub cap_exists: Option<bool>,
    pub finite_trailing_edge_thickness: Option<f64>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct SurfaceSectorDefinition {
    pub sector_id: String,
    pub sector_type: ProceduralFeatureType,
    pub support_status: QueryValueStatus,
    pub adjacent_feature_ids: Vec<String>,
    pub notes: Vec<String>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct ProceduralFeatureJunction {
    pub junction_id: String,
    pub role: FeatureEndpointRole,
    pub connected_feature_ids: Vec<String>,
    pub span_parameter: Option<f64>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct ProceduralFeatureCapabilities {
    pub support_status: QueryValueStatus,
    pub inventory: QueryValueStatus,
    pub feature_evaluation: QueryValueStatus,
    pub closest_feature: QueryValueStatus,
    pub surface_sector: QueryValueStatus,
    pub local_parameters: QueryValueStatus,
    pub transform_propagation: QueryValueStatus,
    pub limitations: Vec<String>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct ProceduralFeatureQueryResult {
    pub schema_version: String,
    pub query_position: [f64; 3],
    pub component_id: Option<String>,
    pub feature_id: Option<String>,
    pub feature_type: ProceduralFeatureType,
    pub support_status: QueryValueStatus,
    pub closest_point: Option<[f64; 3]>,
    pub distance_to_feature: Option<f64>,
    pub tangent: Option<[f64; 3]>,
    pub normal_sector: Option<SurfaceSectorId>,
    pub span_parameter: Option<f64>,
    pub chord_parameter: Option<f64>,
    pub airfoil_parameter: Option<f64>,
    pub side: Option<WingSurfaceSide>,
    pub cap_role: Option<CapRole>,
    pub endpoint_role: Option<FeatureEndpointRole>,
    pub connected_feature_ids: Vec<String>,
    pub all_matches: Vec<ProceduralFeatureMatch>,
    pub diagnostics: ProceduralFeatureDiagnostics,
}

impl ProceduralFeatureQueryResult {
    pub fn unsupported(position: Vec3, reason: impl Into<String>) -> Self {
        Self {
            schema_version: PROCEDURAL_FEATURE_SCHEMA_VERSION.to_string(),
            query_position: vec3_array(position),
            component_id: None,
            feature_id: None,
            feature_type: ProceduralFeatureType::Unknown,
            support_status: QueryValueStatus::Unsupported,
            closest_point: None,
            distance_to_feature: None,
            tangent: None,
            normal_sector: None,
            span_parameter: None,
            chord_parameter: None,
            airfoil_parameter: None,
            side: None,
            cap_role: None,
            endpoint_role: None,
            connected_feature_ids: Vec::new(),
            all_matches: Vec::new(),
            diagnostics: ProceduralFeatureDiagnostics {
                success: false,
                warnings: Vec::new(),
                errors: vec![reason.into()],
                ambiguity: None,
                query_tolerance: None,
                method: "unsupported".to_string(),
                feature_to_sdf_residual: None,
            },
        }
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct ProceduralFeatureMatch {
    pub feature_id: String,
    pub feature_type: ProceduralFeatureType,
    pub closest_point: [f64; 3],
    pub distance: f64,
    pub tangent: Option<[f64; 3]>,
    pub span_parameter: Option<f64>,
    pub endpoint_role: Option<FeatureEndpointRole>,
    pub support_status: QueryValueStatus,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct ProceduralFeatureDiagnostics {
    pub success: bool,
    pub warnings: Vec<String>,
    pub errors: Vec<String>,
    pub ambiguity: Option<String>,
    pub query_tolerance: Option<f64>,
    pub method: String,
    pub feature_to_sdf_residual: Option<f64>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ProceduralFeatureType {
    LeadingEdge,
    TrailingEdgeUpperCapBoundary,
    TrailingEdgeLowerCapBoundary,
    TrailingEdgeCapCenter,
    TipUpperCapBoundary,
    TipLowerCapBoundary,
    TipPerimeter,
    RootSection,
    RootCapBoundary,
    UpperSurfaceSector,
    LowerSurfaceSector,
    TrailingEdgeCapSector,
    TipCapSector,
    Unknown,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct SurfaceSectorId {
    pub stable_id: String,
    pub support_status: QueryValueStatus,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum WingSurfaceSide {
    Upper,
    Lower,
    Cap,
    Unknown,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CapRole {
    TrailingEdgeCap,
    TipCapNegative,
    TipCapPositive,
    RootCap,
    None,
    Unknown,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FeatureEndpointRole {
    NegativeTip,
    PositiveTip,
    Root,
    LeadingTip,
    TrailingUpperTip,
    TrailingLowerTip,
    LeadingRoot,
    TrailingUpperRoot,
    TrailingLowerRoot,
    ClosedLoop,
    Interior,
    Unknown,
}

pub trait ProceduralFeatureProvider {
    fn procedural_feature_inventory(&self) -> Option<ProceduralFeatureInventory> {
        None
    }

    fn evaluate_procedural_feature(
        &self,
        _feature_id: &str,
        _parameter: f64,
    ) -> Option<ProceduralFeatureQueryResult> {
        None
    }

    fn query_procedural_feature(
        &self,
        point: Vec3,
        _options: &ProceduralFeatureQueryOptions,
    ) -> ProceduralFeatureQueryResult {
        ProceduralFeatureQueryResult::unsupported(
            point,
            "geometry does not implement procedural feature queries",
        )
    }

    fn query_procedural_feature_batch(
        &self,
        points: &[Vec3],
        options: &ProceduralFeatureQueryOptions,
    ) -> ProceduralFeatureBatchResult {
        let start = Instant::now();
        let results = points
            .iter()
            .map(|point| self.query_procedural_feature(*point, options))
            .collect::<Vec<_>>();
        let success_count = results
            .iter()
            .filter(|result| result.support_status != QueryValueStatus::Unsupported)
            .count();
        ProceduralFeatureBatchResult {
            schema_version: PROCEDURAL_FEATURE_SCHEMA_VERSION.to_string(),
            summary: ProceduralFeatureBatchSummary {
                input_count: points.len(),
                success_count,
                failure_count: points.len().saturating_sub(success_count),
                elapsed_ms: start.elapsed().as_secs_f64() * 1000.0,
                ordering_preserved: true,
            },
            results,
        }
    }
}

pub(crate) fn vec3_array(point: Vec3) -> [f64; 3] {
    [point.x as f64, point.y as f64, point.z as f64]
}

pub(crate) fn array_vec3(point: [f64; 3]) -> Vec3 {
    Vec3::new(point[0] as f32, point[1] as f32, point[2] as f32)
}
