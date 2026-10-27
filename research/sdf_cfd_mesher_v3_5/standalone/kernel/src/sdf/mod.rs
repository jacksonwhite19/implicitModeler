// SDF trait and core types

use glam::Vec3;
use std::any::Any;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use crate::sdf::procedural_features::{
    PROCEDURAL_FEATURE_SCHEMA_VERSION, ProceduralFeatureBatchResult, ProceduralFeatureBatchSummary,
    ProceduralFeatureInventory, ProceduralFeatureQueryOptions, ProceduralFeatureQueryResult,
};
use crate::sdf::provenance::{CsgProvenanceTrace, SdfProvenanceOptions, SdfSample};

/// Trait for signed distance field evaluation
pub trait Sdf: Any + Send + Sync {
    /// Returns the signed distance from a point to the surface.
    /// Negative values indicate the point is inside the shape.
    fn distance(&self, point: Vec3) -> f32;

    /// Returns live graph metadata used by the conditioning backend.
    ///
    /// Implementations should expose conservative support bounds and dependency
    /// roles when they are known. The default keeps older or analytic nodes
    /// usable while marking their bounds as unavailable for local conditioning.
    fn metadata(&self) -> crate::sdf::conditioning::SdfNodeMetadata {
        crate::sdf::conditioning::SdfNodeMetadata::unknown()
    }

    /// Returns the scalar distance plus optional hard-CSG provenance.
    ///
    /// The default implementation preserves the normal `distance()` path and only
    /// attaches terminal metadata when provenance is explicitly requested. Hard
    /// CSG and selected wrappers override this to build recursive traces.
    fn sample_with_provenance(&self, point: Vec3, options: &SdfProvenanceOptions) -> SdfSample {
        let distance = self.distance(point);
        let provenance = options
            .requested()
            .then(|| CsgProvenanceTrace::terminal(self.metadata()));
        SdfSample {
            distance,
            provenance,
        }
    }

    /// Returns procedural feature inventory when a primitive exposes stable
    /// analytic feature and sector metadata. The default keeps existing SDF
    /// nodes usable while making unsupported geometry explicit.
    fn procedural_feature_inventory(&self) -> Option<ProceduralFeatureInventory> {
        None
    }

    /// Evaluates a named procedural feature curve at a normalized parameter.
    fn evaluate_procedural_feature(
        &self,
        _feature_id: &str,
        _parameter: f64,
    ) -> Option<ProceduralFeatureQueryResult> {
        None
    }

    /// Queries procedural features near a point. This is separate from hard-CSG
    /// provenance and is only supported by primitives that own procedural
    /// feature definitions.
    fn query_procedural_feature(
        &self,
        point: Vec3,
        _options: &ProceduralFeatureQueryOptions,
    ) -> ProceduralFeatureQueryResult {
        ProceduralFeatureQueryResult::unsupported(
            point,
            "geometry does not expose procedural feature metadata",
        )
    }

    fn query_procedural_feature_batch(
        &self,
        points: &[Vec3],
        options: &ProceduralFeatureQueryOptions,
    ) -> ProceduralFeatureBatchResult {
        let start = std::time::Instant::now();
        let results = points
            .iter()
            .map(|point| self.query_procedural_feature(*point, options))
            .collect::<Vec<_>>();
        let success_count = results
            .iter()
            .filter(|result| {
                result.support_status != crate::sdf::cfd_query::QueryValueStatus::Unsupported
            })
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

impl dyn Sdf {
    pub fn as_any(&self) -> &dyn Any {
        self
    }
}

static SDF_PROFILE_ENABLED: AtomicBool = AtomicBool::new(false);
static SDF_PROFILE_NODE_VISITS: AtomicU64 = AtomicU64::new(0);

pub fn sdf_profile_set_enabled(enabled: bool) {
    SDF_PROFILE_ENABLED.store(enabled, Ordering::Relaxed);
}

pub fn sdf_profile_reset() {
    SDF_PROFILE_NODE_VISITS.store(0, Ordering::Relaxed);
}

pub fn sdf_profile_node_visit() {
    if SDF_PROFILE_ENABLED.load(Ordering::Relaxed) {
        SDF_PROFILE_NODE_VISITS.fetch_add(1, Ordering::Relaxed);
    }
}

pub fn sdf_profile_node_visits() -> u64 {
    SDF_PROFILE_NODE_VISITS.load(Ordering::Relaxed)
}

pub mod aerospace;
pub mod booleans;
pub mod cfd_query;
pub mod conditioning;
pub mod field;
pub mod lattice;
pub mod mesh_import;
pub mod patterns;
pub mod primitives;
pub mod print;
pub mod procedural_features;
pub mod profiles;
pub mod provenance;
pub mod query;
pub mod spine;
pub mod sweep;
pub mod transforms;

#[cfg(test)]
mod integration_tests {
    use super::*;
    use booleans::Subtract;
    use primitives::{SdfBox, Sphere};
    use std::sync::Arc;
    use transforms::Translate;

    #[test]
    fn test_compound_sdf() {
        // Build a box with a translated sphere subtracted (a box with a hole)
        // Box: 10x10x10 (half-extents 5x5x5)
        // Sphere: radius 3, translated to (3, 0, 0)

        let sdf_box = Arc::new(SdfBox::new(Vec3::new(5.0, 5.0, 5.0)));
        let sphere = Arc::new(Sphere::new(3.0));
        let translated_sphere = Arc::new(Translate::new(sphere, Vec3::new(3.0, 0.0, 0.0)));
        let result = Subtract::new(sdf_box, translated_sphere);

        // Point at (3, 0, 0) - center of the hole
        // Should be inside the removed volume (positive distance)
        let dist_hole_center = result.distance(Vec3::new(3.0, 0.0, 0.0));
        assert!(
            dist_hole_center > 0.0,
            "Center of hole should have positive distance"
        );

        // Point at (-4, 0, 0) - inside the box, far from the hole
        // Should be inside the remaining box (negative distance)
        let dist_inside_box = result.distance(Vec3::new(-4.0, 0.0, 0.0));
        assert!(
            dist_inside_box < 0.0,
            "Point inside box away from hole should be negative"
        );

        // Point far outside
        let dist_outside = result.distance(Vec3::new(20.0, 20.0, 20.0));
        assert!(dist_outside > 0.0, "Point far outside should be positive");
    }
}
