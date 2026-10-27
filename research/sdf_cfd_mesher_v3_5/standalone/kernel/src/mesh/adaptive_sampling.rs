use crate::mesh::adaptive_mc::{LocalRefineRegion, extract_mesh_adaptive_local_regions};
use crate::mesh::adaptive_te_tip_refinement::{
    ActiveLeafMap, FaceDirection, FaceNeighborRelation, MortonLeafKey, indexed_max_neighbor_delta,
};
use crate::mesh::sdf_surface::{
    SdfSurfaceMesh, SurfaceExtractionConfig, SurfaceExtractionOutcome, SurfaceExtractionStatus,
    SurfaceExtractionTimings, rebuild_projected_surface_mesh,
};
use crate::mesh::surface_feature_resolution::{
    FeatureResolutionConfig, measure_feature_resolution,
};
use crate::sdf::Sdf;
use crate::sdf::procedural_features::{
    CapRole, ProceduralFeatureQueryOptions, ProceduralFeatureType,
};
use crate::sdf::query::bounding_points;
use glam::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::sync::Arc;
use std::time::Instant;

pub const ADAPTIVE_SAMPLING_SCHEMA_VERSION: &str = "adaptive_sampling.v10";
pub const ADAPTIVE_SAMPLING_ENGINE_VERSION: &str = "sdf_feature_aware_balanced_octree_sampler.v1";

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct AdaptiveSamplingConfig {
    pub min_depth: u32,
    pub max_depth: u32,
    pub bounds_padding: f64,
    pub base_cell_size: f64,
    pub near_surface_floor_cell_size: Option<f64>,
    pub near_surface_floor_band_factor: f64,
    pub near_surface_floor_max_depth: u32,
    pub max_leaf_cells: usize,
    pub surface_band_factor: f64,
    pub surface_band_max_depth: u32,
    pub surface_crossing_max_depth: u32,
    pub complexity_narrow_band_factor: f64,
    pub complexity_max_depth: u32,
    pub feature_query_band_factor: f64,
    pub feature_max_depth: u32,
    pub normal_divergence_degrees: f64,
    pub interpolation_error_tolerance: f64,
    pub normal_interpolation_error_degrees: f64,
    pub required_feature_intervals: f64,
    pub feature_band_width: f64,
    pub leading_edge_band_width: f64,
    pub root_guide_band_width: f64,
    pub trailing_edge_thickness: f64,
    pub balance_face_neighbors: bool,
    pub max_balance_passes: usize,
}

impl Default for AdaptiveSamplingConfig {
    fn default() -> Self {
        Self {
            min_depth: 4,
            max_depth: 6,
            bounds_padding: 12.0,
            base_cell_size: 8.0,
            near_surface_floor_cell_size: None,
            near_surface_floor_band_factor: 1.15,
            near_surface_floor_max_depth: 6,
            max_leaf_cells: 80_000,
            surface_band_factor: 1.75,
            surface_band_max_depth: 5,
            surface_crossing_max_depth: 6,
            complexity_narrow_band_factor: 1.25,
            complexity_max_depth: 6,
            feature_query_band_factor: 2.0,
            feature_max_depth: 6,
            normal_divergence_degrees: 22.5,
            interpolation_error_tolerance: 0.35,
            normal_interpolation_error_degrees: 18.0,
            required_feature_intervals: 4.0,
            feature_band_width: 8.0,
            leading_edge_band_width: 6.0,
            root_guide_band_width: 6.0,
            trailing_edge_thickness: 1.0,
            balance_face_neighbors: true,
            max_balance_passes: 8,
        }
    }
}

impl AdaptiveSamplingConfig {
    pub fn config_hash(&self) -> String {
        stable_hash_hex(&serde_json::to_vec(self).unwrap_or_default())
    }

    pub fn feature_target_size(&self) -> f32 {
        (self.trailing_edge_thickness / self.required_feature_intervals.max(1.0)) as f32
    }
}

#[derive(Clone, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum RefinementReason {
    MinimumDepth,
    SurfaceCrossing,
    SurfaceBand,
    NearSurfaceResolutionFloor,
    NormalDivergence,
    FeatureThickness,
    ProceduralFeatureProximity,
    FieldInterpolationError,
    NormalInterpolationError,
    Balance,
    MaxDepthReached,
    LeafBudgetReached,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct AdaptiveCellSample {
    pub position: [f64; 3],
    pub signed_distance: f64,
    pub normal: Option<[f64; 3]>,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct AdaptiveFeatureSummary {
    pub closest_feature_id: Option<String>,
    pub closest_feature_type: Option<String>,
    pub closest_feature_distance: Option<f64>,
    pub surface_sector_id: Option<String>,
    pub span_parameter: Option<f64>,
    pub chord_parameter: Option<f64>,
    pub airfoil_parameter: Option<f64>,
    pub local_thickness: Option<f64>,
    pub in_feature_band: bool,
    pub selected_extra_depth_band: Option<String>,
    pub selected_extra_depth_eligible: bool,
    pub selected_extra_depth_band_width: Option<f64>,
    pub selected_extra_depth_match_source: Option<String>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct AdaptiveOctreeNode {
    pub id: u64,
    pub morton_code: u64,
    pub level: u32,
    pub grid_coord: [u32; 3],
    pub bounds_min: [f64; 3],
    pub bounds_max: [f64; 3],
    pub refinement_reasons: Vec<RefinementReason>,
    pub samples: Vec<AdaptiveCellSample>,
    pub feature_summary: AdaptiveFeatureSummary,
    pub local_estimated_thickness: Option<f64>,
    pub local_estimated_curvature: Option<f64>,
    pub normal_divergence_degrees: f64,
    pub field_interpolation_error: f64,
    pub normal_interpolation_error_degrees: f64,
    pub surface_crossing: bool,
}

impl AdaptiveOctreeNode {
    pub fn center(&self) -> Vec3 {
        let min = vec3_from_array(self.bounds_min);
        let max = vec3_from_array(self.bounds_max);
        (min + max) * 0.5
    }

    pub fn size(&self) -> f32 {
        (vec3_from_array(self.bounds_max) - vec3_from_array(self.bounds_min)).max_element()
    }
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct AdaptiveOctreeSummary {
    pub leaf_count: usize,
    pub surface_leaf_count: usize,
    pub feature_leaf_count: usize,
    pub max_level_reached: u32,
    pub level_histogram: BTreeMap<u32, usize>,
    pub reason_histogram: BTreeMap<String, usize>,
    pub balance_passes: usize,
    pub inserted_balancing_cells: usize,
    pub max_face_neighbor_level_delta: u32,
    pub local_refine_region_count: usize,
    pub surface_floor_region_count: usize,
    pub feature_refine_region_count: usize,
    pub min_leaf_size: f64,
    pub max_leaf_size: f64,
    pub estimated_intervals_across_te_thickness: BTreeMap<String, f64>,
    pub cache: AdaptiveSamplingCacheStats,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct AdaptiveSamplingCacheStats {
    pub sdf_distance_hits: usize,
    pub sdf_distance_misses: usize,
    pub procedural_feature_hits: usize,
    pub procedural_feature_misses: usize,
    pub gradient_estimates: usize,
    pub approximate_queries_avoided: usize,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct IncrementalRefinementStats {
    pub selected_parent_count: usize,
    pub direct_child_count: usize,
    pub direct_net_leaf_increase: usize,
    pub balance_events: usize,
    pub balance_child_insertions: usize,
    pub final_leaf_count: usize,
    pub final_leaf_hash: String,
    pub final_max_neighbor_delta: u32,
    pub indexed_balance_used: bool,
    pub indexed_neighbor_audit_used: bool,
    pub all_pairs_balance_used: bool,
    pub all_pairs_neighbor_audit_used: bool,
    pub selected_by_feature_type: BTreeMap<String, usize>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct AdaptiveOctree {
    pub schema_version: String,
    pub engine_version: String,
    pub config_hash: String,
    pub bounds_min: [f64; 3],
    pub bounds_max: [f64; 3],
    pub root_size: f64,
    pub leaves: Vec<AdaptiveOctreeNode>,
    pub summary: AdaptiveOctreeSummary,
}

impl AdaptiveOctree {
    pub fn octree_hash(&self) -> String {
        stable_hash_hex(&serde_json::to_vec(self).unwrap_or_default())
    }

    pub fn local_refine_regions(&self, config: &AdaptiveSamplingConfig) -> Vec<LocalRefineRegion> {
        let mut grouped = BTreeMap::<String, (Vec3, Vec3)>::new();
        let feature_target = config.feature_target_size().max(0.05);
        let mut surface_floor_bounds: Option<(Vec3, Vec3)> = None;
        let surface_floor_target = config
            .near_surface_floor_cell_size
            .unwrap_or(config.base_cell_size)
            .max(0.02) as f32;
        for leaf in &self.leaves {
            if leaf.surface_crossing
                || leaf
                    .refinement_reasons
                    .contains(&RefinementReason::NearSurfaceResolutionFloor)
                || leaf
                    .refinement_reasons
                    .contains(&RefinementReason::SurfaceBand)
            {
                let min = vec3_from_array(leaf.bounds_min);
                let max = vec3_from_array(leaf.bounds_max);
                match surface_floor_bounds.as_mut() {
                    Some((lo, hi)) => {
                        *lo = lo.min(min);
                        *hi = hi.max(max);
                    }
                    None => surface_floor_bounds = Some((min, max)),
                }
            }
            if leaf.feature_summary.in_feature_band
                || leaf
                    .refinement_reasons
                    .contains(&RefinementReason::FeatureThickness)
                || leaf
                    .refinement_reasons
                    .contains(&RefinementReason::ProceduralFeatureProximity)
            {
                let key = leaf
                    .feature_summary
                    .closest_feature_id
                    .clone()
                    .or_else(|| leaf.feature_summary.surface_sector_id.clone())
                    .or_else(|| leaf.feature_summary.closest_feature_type.clone())
                    .unwrap_or_else(|| "feature_band".to_string());
                let min = vec3_from_array(leaf.bounds_min);
                let max = vec3_from_array(leaf.bounds_max);
                grouped
                    .entry(key)
                    .and_modify(|(lo, hi)| {
                        *lo = lo.min(min);
                        *hi = hi.max(max);
                    })
                    .or_insert((min, max));
            }
        }
        let pad = config.base_cell_size as f32 * 0.25;
        let mut regions = grouped
            .into_iter()
            .map(|(_, (min, max))| LocalRefineRegion {
                min: min - Vec3::splat(pad),
                max: max + Vec3::splat(pad),
                target_cell_size: feature_target,
            })
            .collect::<Vec<_>>();
        if let Some((min, max)) = surface_floor_bounds {
            regions.push(LocalRefineRegion {
                min: min - Vec3::splat(pad),
                max: max + Vec3::splat(pad),
                target_cell_size: surface_floor_target,
            });
        }
        regions.sort_by(|a, b| {
            tuple_key(a.min)
                .cmp(&tuple_key(b.min))
                .then(tuple_key(a.max).cmp(&tuple_key(b.max)))
        });
        regions
    }
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct AdaptiveSamplingTimings {
    pub geometry_bounds_ms: f64,
    pub octree_refinement_ms: f64,
    pub balancing_ms: f64,
    pub extraction_ms: f64,
    pub projection_rebuild_ms: f64,
    pub audit_ms: f64,
    pub total_ms: f64,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct AdaptiveSurfaceExtractionOutcome {
    pub octree: AdaptiveOctree,
    pub surface: SurfaceExtractionOutcome,
    pub adaptive_timings: AdaptiveSamplingTimings,
    pub local_refine_regions: usize,
}

#[derive(Clone, Debug)]
struct WorkCell {
    min: Vec3,
    max: Vec3,
    level: u32,
    morton_code: u64,
    grid_coord: [u32; 3],
    balance_inserted: bool,
    inherited_extra_depth_band: Option<String>,
    inherited_extra_depth_band_width: Option<f64>,
    inherited_extra_depth_match_source: Option<String>,
}

#[derive(Clone, Debug)]
struct CachedDistance {
    distance: f32,
}

#[derive(Default)]
struct SamplingCache {
    distances: BTreeMap<[i64; 3], CachedDistance>,
    features: BTreeMap<[i64; 3], AdaptiveFeatureSummary>,
    stats: AdaptiveSamplingCacheStats,
}

impl SamplingCache {
    fn distance(&mut self, sdf: &dyn Sdf, point: Vec3) -> f32 {
        let key = point_cache_key(point);
        if let Some(value) = self.distances.get(&key) {
            self.stats.sdf_distance_hits += 1;
            return value.distance;
        }
        self.stats.sdf_distance_misses += 1;
        let distance = sdf.distance(point);
        self.distances.insert(key, CachedDistance { distance });
        distance
    }

    fn feature_summary(
        &mut self,
        sdf: &dyn Sdf,
        point: Vec3,
        config: &AdaptiveSamplingConfig,
    ) -> AdaptiveFeatureSummary {
        let key = point_cache_key(point);
        if let Some(value) = self.features.get(&key) {
            self.stats.procedural_feature_hits += 1;
            return value.clone();
        }
        self.stats.procedural_feature_misses += 1;
        let value = sample_feature_summary_uncached(sdf, point, config);
        self.features.insert(key, value.clone());
        value
    }

    fn stats(&self) -> AdaptiveSamplingCacheStats {
        let mut stats = self.stats.clone();
        stats.approximate_queries_avoided = stats.sdf_distance_hits + stats.procedural_feature_hits;
        stats
    }
}

pub fn build_adaptive_octree(
    sdf: &dyn Sdf,
    config: &AdaptiveSamplingConfig,
) -> Result<AdaptiveOctree, String> {
    let bounds_start = Instant::now();
    let (domain_min, _domain_max, root_size) = adaptive_domain(sdf, config)?;
    let _bounds_ms = elapsed_ms(bounds_start);
    let mut leaves = Vec::<AdaptiveOctreeNode>::new();
    let mut stack = vec![WorkCell {
        min: domain_min,
        max: domain_min + Vec3::splat(root_size),
        level: 0,
        morton_code: 0,
        grid_coord: [0, 0, 0],
        balance_inserted: false,
        inherited_extra_depth_band: None,
        inherited_extra_depth_band_width: None,
        inherited_extra_depth_match_source: None,
    }];
    let mut budget_exhausted = false;
    let mut cache = SamplingCache::default();
    while let Some(cell) = stack.pop() {
        let (mut node, split) = evaluate_cell(sdf, &cell, config, &mut cache);
        if cell.balance_inserted && !node.refinement_reasons.contains(&RefinementReason::Balance) {
            node.refinement_reasons.push(RefinementReason::Balance);
        }
        if split && leaves.len() + stack.len() + 8 < config.max_leaf_cells {
            let center = (cell.min + cell.max) * 0.5;
            for child in (0_u8..8).rev() {
                let dx = child & 1;
                let dy = (child >> 1) & 1;
                let dz = (child >> 2) & 1;
                let child_min = Vec3::new(
                    if dx == 0 { cell.min.x } else { center.x },
                    if dy == 0 { cell.min.y } else { center.y },
                    if dz == 0 { cell.min.z } else { center.z },
                );
                let child_max = Vec3::new(
                    if dx == 0 { center.x } else { cell.max.x },
                    if dy == 0 { center.y } else { cell.max.y },
                    if dz == 0 { center.z } else { cell.max.z },
                );
                let inherited_band = node
                    .feature_summary
                    .selected_extra_depth_band
                    .clone()
                    .or_else(|| cell.inherited_extra_depth_band.clone());
                let inherited_width = node
                    .feature_summary
                    .selected_extra_depth_band_width
                    .or(cell.inherited_extra_depth_band_width);
                let inherited_source = node
                    .feature_summary
                    .selected_extra_depth_match_source
                    .clone()
                    .or_else(|| cell.inherited_extra_depth_match_source.clone());
                stack.push(WorkCell {
                    min: child_min,
                    max: child_max,
                    level: cell.level + 1,
                    morton_code: (cell.morton_code << 3) | child as u64,
                    grid_coord: [
                        cell.grid_coord[0] * 2 + dx as u32,
                        cell.grid_coord[1] * 2 + dy as u32,
                        cell.grid_coord[2] * 2 + dz as u32,
                    ],
                    balance_inserted: cell.balance_inserted,
                    inherited_extra_depth_band: inherited_band.clone(),
                    inherited_extra_depth_band_width: inherited_width,
                    inherited_extra_depth_match_source: inherited_source.clone(),
                });
            }
        } else {
            if split {
                budget_exhausted = true;
                node.refinement_reasons
                    .push(RefinementReason::LeafBudgetReached);
            }
            leaves.push(node);
        }
    }
    leaves = sort_and_renumber(leaves);
    let (leaves, balance_passes, inserted) = if config.balance_face_neighbors {
        balance_leaves(sdf, leaves, config, &mut cache)
    } else {
        (leaves, 0, 0)
    };
    let mut octree = AdaptiveOctree {
        schema_version: ADAPTIVE_SAMPLING_SCHEMA_VERSION.to_string(),
        engine_version: ADAPTIVE_SAMPLING_ENGINE_VERSION.to_string(),
        config_hash: config.config_hash(),
        bounds_min: array_from_vec3(domain_min),
        bounds_max: array_from_vec3(domain_min + Vec3::splat(root_size)),
        root_size: root_size as f64,
        leaves,
        summary: AdaptiveOctreeSummary::default(),
    };
    octree.summary = summarize_octree(
        &octree.leaves,
        balance_passes,
        inserted,
        budget_exhausted,
        cache.stats(),
    );
    Ok(octree)
}

pub fn refine_te_tip_feature_band_plus_one(
    sdf: &dyn Sdf,
    base: &AdaptiveOctree,
    config: &AdaptiveSamplingConfig,
) -> Result<(AdaptiveOctree, IncrementalRefinementStats), String> {
    let base_max_level = base.summary.max_level_reached;
    let mut cache = SamplingCache::default();
    let mut selected = BTreeSet::<MortonLeafKey>::new();
    let mut selected_by_feature_type = BTreeMap::<String, usize>::new();
    for leaf in &base.leaves {
        if leaf.level != base_max_level || leaf.level >= config.max_depth {
            continue;
        }
        if !leaf.feature_summary.selected_extra_depth_eligible || !is_te_tip_leaf(leaf) {
            continue;
        }
        selected.insert(node_key(leaf));
        let key = leaf
            .feature_summary
            .selected_extra_depth_band
            .clone()
            .or_else(|| leaf.feature_summary.closest_feature_type.clone())
            .or_else(|| leaf.feature_summary.surface_sector_id.clone())
            .unwrap_or_else(|| "unknown".to_string());
        *selected_by_feature_type.entry(key).or_default() += 1;
    }
    if selected.is_empty() {
        return Err("no_te_tip_feature_band_parents_selected".to_string());
    }
    let mut direct = Vec::<AdaptiveOctreeNode>::new();
    let mut direct_children = 0_usize;
    for leaf in &base.leaves {
        if selected.contains(&node_key(leaf)) {
            direct_children += 8;
            let min = vec3_from_array(leaf.bounds_min);
            let max = vec3_from_array(leaf.bounds_max);
            let center = (min + max) * 0.5;
            for child in 0_u8..8 {
                let dx = child & 1;
                let dy = (child >> 1) & 1;
                let dz = (child >> 2) & 1;
                let child_min = Vec3::new(
                    if dx == 0 { min.x } else { center.x },
                    if dy == 0 { min.y } else { center.y },
                    if dz == 0 { min.z } else { center.z },
                );
                let child_max = Vec3::new(
                    if dx == 0 { center.x } else { max.x },
                    if dy == 0 { center.y } else { max.y },
                    if dz == 0 { center.z } else { max.z },
                );
                let work = WorkCell {
                    min: child_min,
                    max: child_max,
                    level: leaf.level + 1,
                    morton_code: (leaf.morton_code << 3) | child as u64,
                    grid_coord: [
                        leaf.grid_coord[0] * 2 + dx as u32,
                        leaf.grid_coord[1] * 2 + dy as u32,
                        leaf.grid_coord[2] * 2 + dz as u32,
                    ],
                    balance_inserted: false,
                    inherited_extra_depth_band: leaf
                        .feature_summary
                        .selected_extra_depth_band
                        .clone(),
                    inherited_extra_depth_band_width: leaf
                        .feature_summary
                        .selected_extra_depth_band_width,
                    inherited_extra_depth_match_source: leaf
                        .feature_summary
                        .selected_extra_depth_match_source
                        .clone(),
                };
                let (mut node, _) = evaluate_cell(sdf, &work, config, &mut cache);
                node.refinement_reasons
                    .push(RefinementReason::ProceduralFeatureProximity);
                direct.push(node);
            }
        } else {
            direct.push(leaf.clone());
        }
    }
    let direct_net_leaf_increase = selected.len().saturating_mul(7);
    let direct = sort_and_renumber(direct);
    let (leaves, balance_events, balance_child_insertions) = if config.balance_face_neighbors {
        balance_leaves_indexed(sdf, direct, config, &mut cache)
    } else {
        (direct, 0, 0)
    };
    let mut octree = AdaptiveOctree {
        schema_version: ADAPTIVE_SAMPLING_SCHEMA_VERSION.to_string(),
        engine_version: ADAPTIVE_SAMPLING_ENGINE_VERSION.to_string(),
        config_hash: config.config_hash(),
        bounds_min: base.bounds_min,
        bounds_max: base.bounds_max,
        root_size: base.root_size,
        leaves,
        summary: AdaptiveOctreeSummary::default(),
    };
    octree.summary = summarize_octree(
        &octree.leaves,
        balance_events,
        balance_child_insertions,
        false,
        cache.stats(),
    );
    let final_leaf_hash = octree.octree_hash();
    let stats = IncrementalRefinementStats {
        selected_parent_count: selected.len(),
        direct_child_count: direct_children,
        direct_net_leaf_increase,
        balance_events,
        balance_child_insertions,
        final_leaf_count: octree.leaves.len(),
        final_leaf_hash,
        final_max_neighbor_delta: octree.summary.max_face_neighbor_level_delta,
        indexed_balance_used: true,
        indexed_neighbor_audit_used: true,
        all_pairs_balance_used: false,
        all_pairs_neighbor_audit_used: false,
        selected_by_feature_type,
    };
    Ok((octree, stats))
}

fn is_te_tip_leaf(leaf: &AdaptiveOctreeNode) -> bool {
    let feature = leaf
        .feature_summary
        .closest_feature_type
        .as_deref()
        .unwrap_or_default();
    let sector = leaf
        .feature_summary
        .surface_sector_id
        .as_deref()
        .unwrap_or_default();
    feature.contains("TrailingEdge")
        || feature.contains("Tip")
        || sector.contains("trailing")
        || sector.contains("tip")
        || sector.contains("TrailingEdge")
        || sector.contains("Tip")
}

pub fn extract_adaptive_sdf_surface_mesh(
    sdf: Arc<dyn Sdf>,
    adaptive_config: AdaptiveSamplingConfig,
    surface_config: SurfaceExtractionConfig,
) -> Result<AdaptiveSurfaceExtractionOutcome, String> {
    let total_start = Instant::now();
    let mut timings = AdaptiveSamplingTimings::default();
    let octree_start = Instant::now();
    let octree = build_adaptive_octree(sdf.as_ref(), &adaptive_config)?;
    timings.octree_refinement_ms = elapsed_ms(octree_start);

    let regions = octree.local_refine_regions(&adaptive_config);
    let extraction_start = Instant::now();
    let raw = extract_mesh_adaptive_local_regions(
        sdf.as_ref(),
        vec3_from_array(octree.bounds_min),
        vec3_from_array(octree.bounds_max),
        adaptive_config.base_cell_size as f32,
        &regions,
        false,
    );
    timings.extraction_ms = elapsed_ms(extraction_start);

    let positions = raw
        .vertices
        .iter()
        .map(|v| Vec3::from_array(v.position))
        .collect::<Vec<_>>();
    let triangles = raw
        .indices
        .chunks_exact(3)
        .map(|tri| [tri[0] as u64, tri[1] as u64, tri[2] as u64])
        .collect::<Vec<_>>();

    let rebuild_start = Instant::now();
    let (mesh, audit, query_counts) = rebuild_projected_surface_mesh(
        Arc::clone(&sdf),
        &positions,
        &triangles,
        &surface_config,
        format!("{ADAPTIVE_SAMPLING_ENGINE_VERSION}+adaptive_mc_local_regions"),
    )?;
    timings.projection_rebuild_ms = elapsed_ms(rebuild_start);

    let audit_start = Instant::now();
    let mut failure_reasons = Vec::new();
    if surface_config.require_closed && audit.topology.boundary_edges > 0 {
        failure_reasons.push("intended closed surface has boundary edges".to_string());
    }
    if surface_config.require_manifold && audit.topology.non_manifold_edges > 0 {
        failure_reasons.push("intended manifold surface has non-manifold edges".to_string());
    }
    if audit.geometry.projection_failures > 0 {
        failure_reasons.push("one or more vertices failed SDF projection".to_string());
    }
    let status = if failure_reasons.is_empty() {
        SurfaceExtractionStatus::Success
    } else {
        SurfaceExtractionStatus::Failed
    };
    timings.audit_ms = elapsed_ms(audit_start);
    timings.total_ms = elapsed_ms(total_start);
    let surface = SurfaceExtractionOutcome {
        mesh,
        audit,
        timings: SurfaceExtractionTimings {
            geometry_initialization_ms: 0.0,
            sdf_sampling_and_initial_extraction_ms: timings.extraction_ms,
            vertex_deduplication_ms: 0.0,
            projection_ms: timings.projection_rebuild_ms,
            orientation_and_triangle_quality_ms: 0.0,
            adjacency_ms: 0.0,
            audit_ms: timings.audit_ms,
            serialization_ms: 0.0,
            total_ms: timings.total_ms,
        },
        query_counts: query_counts.clone(),
        status,
        failure_reasons,
    };
    Ok(AdaptiveSurfaceExtractionOutcome {
        octree,
        surface,
        adaptive_timings: timings,
        local_refine_regions: regions.len(),
    })
}

pub fn adaptive_feature_resolution_metrics(
    sdf: &dyn Sdf,
    mesh: &SdfSurfaceMesh,
    config: &AdaptiveSamplingConfig,
) -> serde_json::Value {
    let feature_config = FeatureResolutionConfig {
        global_target_edge_length: config.base_cell_size,
        feature_target_edge_length: config.trailing_edge_thickness
            / config.required_feature_intervals.max(1.0),
        feature_band_width: config.feature_band_width,
        trailing_edge_thickness: config.trailing_edge_thickness,
        refine_trailing_edge: true,
        refine_tip_transitions: true,
        refine_cap_sectors: true,
        ..FeatureResolutionConfig::default()
    };
    serde_json::to_value(measure_feature_resolution(sdf, mesh, &feature_config))
        .unwrap_or_else(|_| serde_json::json!({ "error": "metric_serialization_failed" }))
}

fn adaptive_domain(
    sdf: &dyn Sdf,
    config: &AdaptiveSamplingConfig,
) -> Result<(Vec3, Vec3, f32), String> {
    let metadata = sdf.metadata();
    let mut min = metadata
        .support_bounds
        .as_ref()
        .map(|b| b.min)
        .unwrap_or(Vec3::splat(f32::INFINITY));
    let mut max = metadata
        .support_bounds
        .as_ref()
        .map(|b| b.max)
        .unwrap_or(Vec3::splat(f32::NEG_INFINITY));
    if !min.is_finite() || !max.is_finite() {
        let points = bounding_points(sdf);
        min = points.min;
        max = points.max;
    }
    if !min.is_finite() || !max.is_finite() {
        return Err("adaptive sampling received non-finite bounds".to_string());
    }
    let pad = config.bounds_padding as f32;
    min -= Vec3::splat(pad);
    max += Vec3::splat(pad);
    let center = (min + max) * 0.5;
    let mut root_size = (max - min).max_element().max(config.base_cell_size as f32);
    let base = config.base_cell_size.max(1.0e-3) as f32;
    let min_root = base * 2.0_f32.powi(config.min_depth as i32);
    root_size = root_size.max(min_root);
    let root_min = center - Vec3::splat(root_size * 0.5);
    Ok((root_min, root_min + Vec3::splat(root_size), root_size))
}

fn evaluate_cell(
    sdf: &dyn Sdf,
    cell: &WorkCell,
    config: &AdaptiveSamplingConfig,
    cache: &mut SamplingCache,
) -> (AdaptiveOctreeNode, bool) {
    let corners = cell_corners(cell.min, cell.max);
    let center = (cell.min + cell.max) * 0.5;
    let face_centers = [
        Vec3::new(cell.min.x, center.y, center.z),
        Vec3::new(cell.max.x, center.y, center.z),
        Vec3::new(center.x, cell.min.y, center.z),
        Vec3::new(center.x, cell.max.y, center.z),
        Vec3::new(center.x, center.y, cell.min.z),
        Vec3::new(center.x, center.y, cell.max.z),
    ];
    let mut probe_points = Vec::with_capacity(15);
    probe_points.extend(corners);
    probe_points.extend(face_centers);
    probe_points.push(center);

    let eps = ((cell.max - cell.min).max_element() * 0.25).max(1.0e-3);
    let mut samples = Vec::with_capacity(probe_points.len());
    let mut min_d = f32::INFINITY;
    let mut max_d = f32::NEG_INFINITY;
    let mut normals = Vec::new();
    for p in &probe_points {
        let d = cache.distance(sdf, *p);
        min_d = min_d.min(d);
        max_d = max_d.max(d);
        let n = estimate_normal_cached(sdf, *p, eps, cache);
        cache.stats.gradient_estimates += 1;
        if n.length_squared() > 0.0 {
            normals.push(n);
        }
        samples.push(AdaptiveCellSample {
            position: array_from_vec3(*p),
            signed_distance: d as f64,
            normal: (n.length_squared() > 0.0).then(|| array_from_vec3(n)),
        });
    }
    let surface_crossing = min_d <= 0.0 && max_d >= 0.0;
    let cell_size_f32 = (cell.max - cell.min).max_element();
    let cell_diagonal = (cell.max - cell.min).length();
    let min_abs_distance = samples
        .iter()
        .map(|s| s.signed_distance.abs())
        .fold(f64::INFINITY, f64::min) as f32;
    let near_surface =
        min_abs_distance <= cell_diagonal * config.surface_band_factor.max(0.0) as f32;
    let complexity_near_surface = surface_crossing
        || min_abs_distance <= cell_diagonal * config.complexity_narrow_band_factor.max(0.0) as f32;
    let feature_query_allowed = surface_crossing
        || min_abs_distance <= cell_diagonal * config.feature_query_band_factor.max(0.0) as f32
        || cell.level < config.min_depth;
    let normal_divergence = max_normal_angle(&normals);
    let center_distance = samples.last().map(|s| s.signed_distance).unwrap_or(0.0);
    let corner_avg = samples
        .iter()
        .take(8)
        .map(|s| s.signed_distance)
        .sum::<f64>()
        / 8.0;
    let field_interpolation_error =
        (center_distance - corner_avg).abs() / (cell_size_f32 as f64).max(1.0e-9);
    let center_normal = normals.last().copied().unwrap_or(Vec3::ZERO);
    let normal_avg = average_normal(&normals[..normals.len().min(8)]);
    let normal_interp_error = angle_degrees(center_normal, normal_avg);
    let mut feature_summary = if feature_query_allowed {
        cache.feature_summary(sdf, center, config)
    } else {
        AdaptiveFeatureSummary::default()
    };
    if cell.level > config.surface_crossing_max_depth {
        if let Some(band) = &cell.inherited_extra_depth_band {
            feature_summary.selected_extra_depth_eligible = true;
            feature_summary.selected_extra_depth_band = Some(band.clone());
            feature_summary.selected_extra_depth_band_width = cell.inherited_extra_depth_band_width;
            feature_summary.selected_extra_depth_match_source = cell
                .inherited_extra_depth_match_source
                .clone()
                .or_else(|| Some("inherited_from_selected_parent".to_string()));
            feature_summary.in_feature_band = true;
            feature_summary.local_thickness = Some(config.trailing_edge_thickness);
        }
    }
    let thickness = feature_summary.local_thickness;
    let cell_size = (cell.max - cell.min).max_element() as f64;

    let mut reasons = BTreeSet::new();
    if cell.level < config.min_depth {
        reasons.insert(RefinementReason::MinimumDepth);
    }
    if surface_crossing && cell.level < config.surface_crossing_max_depth {
        reasons.insert(RefinementReason::SurfaceCrossing);
    }
    if near_surface && cell.level < config.surface_band_max_depth {
        reasons.insert(RefinementReason::SurfaceBand);
    }
    if let Some(floor) = config.near_surface_floor_cell_size {
        let floor_near_surface = surface_crossing
            || min_abs_distance
                <= cell_diagonal * config.near_surface_floor_band_factor.max(0.0) as f32;
        if floor_near_surface
            && cell.level < config.near_surface_floor_max_depth
            && cell_size > floor.max(1.0e-6)
        {
            reasons.insert(RefinementReason::NearSurfaceResolutionFloor);
        }
    }
    if complexity_near_surface
        && cell.level < config.complexity_max_depth
        && normal_divergence > config.normal_divergence_degrees
    {
        reasons.insert(RefinementReason::NormalDivergence);
    }
    let feature_limit = feature_refinement_limit(config, &feature_summary);
    if feature_summary.in_feature_band && cell.level < feature_limit {
        if let Some(thickness) = thickness {
            if cell_size > thickness / config.required_feature_intervals.max(1.0) {
                reasons.insert(RefinementReason::FeatureThickness);
            }
        }
    }
    if feature_summary.in_feature_band && cell.level < feature_limit {
        reasons.insert(RefinementReason::ProceduralFeatureProximity);
    }
    if complexity_near_surface
        && cell.level < config.complexity_max_depth
        && field_interpolation_error > config.interpolation_error_tolerance
    {
        reasons.insert(RefinementReason::FieldInterpolationError);
    }
    if complexity_near_surface
        && cell.level < config.complexity_max_depth
        && normal_interp_error > config.normal_interpolation_error_degrees
    {
        reasons.insert(RefinementReason::NormalInterpolationError);
    }
    if cell.level < config.min_depth {
        reasons.insert(RefinementReason::MinimumDepth);
    }
    let wants_split = !reasons.is_empty();
    if cell.level >= config.max_depth {
        reasons.insert(RefinementReason::MaxDepthReached);
    }
    let can_split = cell.level < config.max_depth;
    let node = AdaptiveOctreeNode {
        id: 0,
        morton_code: cell.morton_code,
        level: cell.level,
        grid_coord: cell.grid_coord,
        bounds_min: array_from_vec3(cell.min),
        bounds_max: array_from_vec3(cell.max),
        refinement_reasons: reasons.into_iter().collect(),
        samples,
        feature_summary,
        local_estimated_thickness: thickness,
        local_estimated_curvature: Some(normal_divergence / cell_size.max(1.0e-9)),
        normal_divergence_degrees: normal_divergence,
        field_interpolation_error,
        normal_interpolation_error_degrees: normal_interp_error,
        surface_crossing,
    };
    (node, wants_split && can_split)
}

fn sample_feature_summary_uncached(
    sdf: &dyn Sdf,
    point: Vec3,
    config: &AdaptiveSamplingConfig,
) -> AdaptiveFeatureSummary {
    let options = ProceduralFeatureQueryOptions {
        tolerance: config
            .feature_band_width
            .max(config.leading_edge_band_width)
            .max(config.root_guide_band_width),
        ..ProceduralFeatureQueryOptions::diagnostic()
    };
    let query = sdf.query_procedural_feature(point, &options);
    let mut summary = AdaptiveFeatureSummary::default();
    summary.closest_feature_id = query.feature_id.clone();
    summary.closest_feature_type = Some(format!("{:?}", query.feature_type));
    summary.closest_feature_distance = query.distance_to_feature;
    summary.surface_sector_id = query.normal_sector.as_ref().map(|s| s.stable_id.clone());
    summary.span_parameter = query.span_parameter;
    summary.chord_parameter = query.chord_parameter;
    summary.airfoil_parameter = query.airfoil_parameter;
    let feature_band = match query.feature_type {
        ProceduralFeatureType::LeadingEdge => config.leading_edge_band_width,
        ProceduralFeatureType::RootSection => config.root_guide_band_width,
        ProceduralFeatureType::TrailingEdgeUpperCapBoundary
        | ProceduralFeatureType::TrailingEdgeLowerCapBoundary
        | ProceduralFeatureType::TrailingEdgeCapCenter
        | ProceduralFeatureType::TipUpperCapBoundary
        | ProceduralFeatureType::TipLowerCapBoundary
        | ProceduralFeatureType::TipPerimeter => config.feature_band_width,
        _ => config.feature_band_width,
    };
    summary.in_feature_band = query
        .distance_to_feature
        .map(|d| d <= feature_band)
        .unwrap_or(false);
    let te_or_tip = matches!(
        query.feature_type,
        ProceduralFeatureType::TrailingEdgeUpperCapBoundary
            | ProceduralFeatureType::TrailingEdgeLowerCapBoundary
            | ProceduralFeatureType::TrailingEdgeCapCenter
            | ProceduralFeatureType::TipUpperCapBoundary
            | ProceduralFeatureType::TipLowerCapBoundary
            | ProceduralFeatureType::TipPerimeter
    ) || matches!(
        query.cap_role,
        Some(CapRole::TrailingEdgeCap | CapRole::TipCapNegative | CapRole::TipCapPositive)
    );
    if te_or_tip && summary.in_feature_band {
        summary.local_thickness = Some(config.trailing_edge_thickness);
    }
    if let Some(envelope) = selected_extra_depth_envelope(&query, config) {
        summary.selected_extra_depth_eligible = true;
        summary.selected_extra_depth_band = Some(envelope.band_id);
        summary.selected_extra_depth_band_width = Some(envelope.band_width);
        summary.selected_extra_depth_match_source = Some(envelope.match_source);
        summary.in_feature_band = true;
        summary.local_thickness = Some(config.trailing_edge_thickness);
    }
    summary
}

#[derive(Clone, Debug)]
struct SelectedExtraDepthEnvelope {
    band_id: String,
    band_width: f64,
    match_source: String,
}

fn feature_refinement_limit(
    config: &AdaptiveSamplingConfig,
    summary: &AdaptiveFeatureSummary,
) -> u32 {
    if summary.selected_extra_depth_eligible {
        config.feature_max_depth
    } else {
        config
            .feature_max_depth
            .min(config.surface_crossing_max_depth)
    }
}

fn selected_extra_depth_envelope(
    query: &crate::sdf::procedural_features::ProceduralFeatureQueryResult,
    config: &AdaptiveSamplingConfig,
) -> Option<SelectedExtraDepthEnvelope> {
    let feature_id = query.feature_id.as_deref().unwrap_or_default();
    let sector_id = query
        .normal_sector
        .as_ref()
        .map(|s| s.stable_id.as_str())
        .unwrap_or_default();
    let distance = query.distance_to_feature?;
    let span = query.span_parameter.unwrap_or(0.5);
    let chord = query.chord_parameter.unwrap_or(0.0);
    let airfoil = query.airfoil_parameter.unwrap_or(0.5);
    let band_width = selected_extra_band_width(config);
    if distance > band_width {
        return None;
    }

    let te_chord_ok = chord >= 0.935 || airfoil >= 0.935;
    match query.feature_type {
        ProceduralFeatureType::TrailingEdgeUpperCapBoundary => {
            if feature_id.ends_with(".trailing_edge.upper_cap_boundary")
                && te_chord_ok
                && !sector_id.contains(".lower")
            {
                return Some(extra_envelope(
                    "upper_te_boundary",
                    band_width,
                    "feature_id+sector+chord",
                ));
            }
        }
        ProceduralFeatureType::TrailingEdgeLowerCapBoundary => {
            if feature_id.ends_with(".trailing_edge.lower_cap_boundary")
                && te_chord_ok
                && !sector_id.contains(".upper")
            {
                return Some(extra_envelope(
                    "lower_te_boundary",
                    band_width,
                    "feature_id+sector+chord",
                ));
            }
        }
        ProceduralFeatureType::TrailingEdgeCapCenter
        | ProceduralFeatureType::TrailingEdgeCapSector => {
            if te_chord_ok
                && (sector_id.contains("trailing_edge.cap_sector")
                    || matches!(query.cap_role, Some(CapRole::TrailingEdgeCap)))
            {
                return Some(extra_envelope("te_cap", band_width, "sector+chord"));
            }
        }
        ProceduralFeatureType::TipUpperCapBoundary
        | ProceduralFeatureType::TipLowerCapBoundary
        | ProceduralFeatureType::TipPerimeter => {
            let near_negative = span <= 0.035 || feature_id.contains(".tip.negative.");
            let near_positive = span >= 0.965 || feature_id.contains(".tip.positive.");
            let tip_sector = sector_id.contains(".tip.")
                || matches!(
                    query.cap_role,
                    Some(CapRole::TipCapNegative | CapRole::TipCapPositive)
                );
            if near_negative && tip_sector {
                let side = if feature_id.contains(".upper") {
                    "upper"
                } else if feature_id.contains(".lower") {
                    "lower"
                } else {
                    "cap"
                };
                return Some(extra_envelope(
                    &format!("negative_tip_{side}_transition"),
                    band_width,
                    "feature_id+span+sector",
                ));
            }
            if near_positive && tip_sector {
                let side = if feature_id.contains(".upper") {
                    "upper"
                } else if feature_id.contains(".lower") {
                    "lower"
                } else {
                    "cap"
                };
                return Some(extra_envelope(
                    &format!("positive_tip_{side}_transition"),
                    band_width,
                    "feature_id+span+sector",
                ));
            }
        }
        _ => {}
    }

    if sector_id.contains("tip.positive.cap_sector") && span >= 0.965 && distance <= band_width {
        return Some(extra_envelope(
            "positive_tip_cap",
            band_width,
            "sector+span",
        ));
    }
    if sector_id.contains("tip.negative.cap_sector") && span <= 0.035 && distance <= band_width {
        return Some(extra_envelope(
            "negative_tip_cap",
            band_width,
            "sector+span",
        ));
    }
    None
}

fn selected_extra_band_width(config: &AdaptiveSamplingConfig) -> f64 {
    (config.base_cell_size * 0.46)
        .min(config.feature_band_width * 0.58)
        .max(config.trailing_edge_thickness * 1.15)
}

fn extra_envelope(
    band_id: impl Into<String>,
    band_width: f64,
    match_source: impl Into<String>,
) -> SelectedExtraDepthEnvelope {
    SelectedExtraDepthEnvelope {
        band_id: band_id.into(),
        band_width,
        match_source: match_source.into(),
    }
}

fn sample_feature_summary(
    sdf: &dyn Sdf,
    point: Vec3,
    config: &AdaptiveSamplingConfig,
) -> AdaptiveFeatureSummary {
    sample_feature_summary_uncached(sdf, point, config)
}

fn balance_leaves(
    sdf: &dyn Sdf,
    leaves: Vec<AdaptiveOctreeNode>,
    config: &AdaptiveSamplingConfig,
    cache: &mut SamplingCache,
) -> (Vec<AdaptiveOctreeNode>, usize, usize) {
    balance_leaves_indexed(sdf, leaves, config, cache)
}

fn balance_leaves_indexed(
    sdf: &dyn Sdf,
    leaves: Vec<AdaptiveOctreeNode>,
    config: &AdaptiveSamplingConfig,
    cache: &mut SamplingCache,
) -> (Vec<AdaptiveOctreeNode>, usize, usize) {
    let mut map = ActiveLeafMap::from_items(
        config.max_depth,
        leaves.into_iter().map(|leaf| (node_key(&leaf), leaf)),
    );
    let mut queue = VecDeque::<MortonLeafKey>::new();
    for key in map.keys() {
        queue.push_back(key);
    }
    let mut split_events = 0;
    let mut inserted = 0;
    let mut work_items = 0_usize;
    let max_work_items = config.max_leaf_cells.saturating_mul(128).max(1024);
    while let Some(key) = queue.pop_front() {
        work_items += 1;
        if work_items > max_work_items {
            break;
        }
        if !map.contains_key(&key) {
            continue;
        }
        let mut split_key = None;
        for direction in FaceDirection::ALL {
            match map.face_neighbor(key, direction) {
                FaceNeighborRelation::Coarser(coarse) if key.level > coarse.level + 1 => {
                    split_key = Some(coarse);
                    break;
                }
                FaceNeighborRelation::Finer(finer)
                | FaceNeighborRelation::InvalidOverlap(finer) => {
                    if finer.iter().any(|other| other.level > key.level + 1) {
                        split_key = Some(key);
                        break;
                    }
                }
                _ => {}
            }
        }
        let Some(to_split) = split_key else {
            continue;
        };
        if to_split.level >= config.max_depth || !map.contains_key(&to_split) {
            continue;
        }
        let Some(leaf) = map.remove(&to_split) else {
            continue;
        };
        split_events += 1;
        inserted += 8;
        let mut affected = BTreeSet::<MortonLeafKey>::new();
        for child in split_leaf_for_balance(sdf, &leaf, config, cache) {
            let child_key = node_key(&child);
            map.insert(child_key, child);
            affected.insert(child_key);
        }
        affected.insert(to_split);
        let seeds = affected.iter().copied().collect::<Vec<_>>();
        for seed in seeds {
            for direction in FaceDirection::ALL {
                collect_neighbor_keys(&map, seed, direction, &mut affected);
            }
        }
        for affected_key in affected {
            queue.push_back(affected_key);
        }
    }
    (
        sort_and_renumber(map.values().collect()),
        split_events,
        inserted,
    )
}

fn collect_neighbor_keys(
    map: &ActiveLeafMap<AdaptiveOctreeNode>,
    key: MortonLeafKey,
    direction: FaceDirection,
    out: &mut BTreeSet<MortonLeafKey>,
) {
    match map.face_neighbor(key, direction) {
        FaceNeighborRelation::SameLevel(k) | FaceNeighborRelation::Coarser(k) => {
            out.insert(k);
        }
        FaceNeighborRelation::Finer(keys) | FaceNeighborRelation::InvalidOverlap(keys) => {
            out.extend(keys);
        }
        FaceNeighborRelation::DomainBoundary | FaceNeighborRelation::InvalidGap => {}
    }
}

fn split_leaf_for_balance(
    sdf: &dyn Sdf,
    leaf: &AdaptiveOctreeNode,
    config: &AdaptiveSamplingConfig,
    cache: &mut SamplingCache,
) -> Vec<AdaptiveOctreeNode> {
    let min = vec3_from_array(leaf.bounds_min);
    let max = vec3_from_array(leaf.bounds_max);
    let center = (min + max) * 0.5;
    let mut out = Vec::with_capacity(8);
    for child in 0_u8..8 {
        let dx = child & 1;
        let dy = (child >> 1) & 1;
        let dz = (child >> 2) & 1;
        let child_min = Vec3::new(
            if dx == 0 { min.x } else { center.x },
            if dy == 0 { min.y } else { center.y },
            if dz == 0 { min.z } else { center.z },
        );
        let child_max = Vec3::new(
            if dx == 0 { center.x } else { max.x },
            if dy == 0 { center.y } else { max.y },
            if dz == 0 { center.z } else { max.z },
        );
        let work = WorkCell {
            min: child_min,
            max: child_max,
            level: leaf.level + 1,
            morton_code: (leaf.morton_code << 3) | child as u64,
            grid_coord: [
                leaf.grid_coord[0] * 2 + dx as u32,
                leaf.grid_coord[1] * 2 + dy as u32,
                leaf.grid_coord[2] * 2 + dz as u32,
            ],
            balance_inserted: true,
            inherited_extra_depth_band: leaf.feature_summary.selected_extra_depth_band.clone(),
            inherited_extra_depth_band_width: leaf.feature_summary.selected_extra_depth_band_width,
            inherited_extra_depth_match_source: leaf
                .feature_summary
                .selected_extra_depth_match_source
                .clone(),
        };
        let (mut node, _) = evaluate_cell(sdf, &work, config, cache);
        node.refinement_reasons.push(RefinementReason::Balance);
        out.push(node);
    }
    out
}

fn summarize_octree(
    leaves: &[AdaptiveOctreeNode],
    balance_passes: usize,
    inserted_balancing_cells: usize,
    budget_exhausted: bool,
    cache_stats: AdaptiveSamplingCacheStats,
) -> AdaptiveOctreeSummary {
    let mut summary = AdaptiveOctreeSummary {
        leaf_count: leaves.len(),
        surface_leaf_count: leaves.iter().filter(|n| n.surface_crossing).count(),
        feature_leaf_count: leaves
            .iter()
            .filter(|n| n.feature_summary.in_feature_band)
            .count(),
        max_level_reached: leaves.iter().map(|n| n.level).max().unwrap_or(0),
        balance_passes,
        inserted_balancing_cells,
        max_face_neighbor_level_delta: max_neighbor_delta(leaves),
        local_refine_region_count: leaves
            .iter()
            .filter(|n| n.feature_summary.in_feature_band)
            .count(),
        surface_floor_region_count: leaves
            .iter()
            .filter(|n| {
                n.surface_crossing
                    || n.refinement_reasons
                        .contains(&RefinementReason::NearSurfaceResolutionFloor)
            })
            .count(),
        feature_refine_region_count: leaves
            .iter()
            .filter(|n| {
                n.feature_summary.in_feature_band
                    || n.refinement_reasons
                        .contains(&RefinementReason::FeatureThickness)
                    || n.refinement_reasons
                        .contains(&RefinementReason::ProceduralFeatureProximity)
            })
            .count(),
        min_leaf_size: leaves
            .iter()
            .map(|n| n.size() as f64)
            .fold(f64::INFINITY, f64::min),
        max_leaf_size: leaves.iter().map(|n| n.size() as f64).fold(0.0, f64::max),
        cache: cache_stats,
        ..AdaptiveOctreeSummary::default()
    };
    for leaf in leaves {
        *summary.level_histogram.entry(leaf.level).or_default() += 1;
        for reason in &leaf.refinement_reasons {
            *summary
                .reason_histogram
                .entry(format!("{reason:?}"))
                .or_default() += 1;
        }
    }
    if budget_exhausted {
        summary
            .reason_histogram
            .insert("LeafBudgetReached".to_string(), 1);
    }
    let mut by_kind = BTreeMap::<String, Vec<f64>>::new();
    for leaf in leaves {
        let Some(thickness) = leaf.local_estimated_thickness else {
            continue;
        };
        let key = leaf
            .feature_summary
            .closest_feature_type
            .clone()
            .unwrap_or_else(|| "unknown".to_string());
        by_kind
            .entry(key)
            .or_default()
            .push(thickness / leaf.size() as f64);
    }
    for (kind, mut values) in by_kind {
        values.sort_by(|a, b| a.total_cmp(b));
        summary
            .estimated_intervals_across_te_thickness
            .insert(kind, percentile(&values, 0.50));
    }
    summary
}

fn sort_and_renumber(mut leaves: Vec<AdaptiveOctreeNode>) -> Vec<AdaptiveOctreeNode> {
    leaves.sort_by(|a, b| {
        a.level
            .cmp(&b.level)
            .then(a.morton_code.cmp(&b.morton_code))
            .then(a.grid_coord.cmp(&b.grid_coord))
    });
    for (idx, leaf) in leaves.iter_mut().enumerate() {
        leaf.id = idx as u64;
    }
    leaves
}

fn max_neighbor_delta(leaves: &[AdaptiveOctreeNode]) -> u32 {
    let max_depth = leaves.iter().map(|leaf| leaf.level).max().unwrap_or(0);
    let map = ActiveLeafMap::from_items(
        max_depth,
        leaves.iter().cloned().map(|leaf| (node_key(&leaf), leaf)),
    );
    indexed_max_neighbor_delta(&map).0
}

fn node_key(leaf: &AdaptiveOctreeNode) -> MortonLeafKey {
    MortonLeafKey::new(leaf.level, leaf.grid_coord)
}

#[allow(dead_code)]
fn face_adjacent(a: &AdaptiveOctreeNode, b: &AdaptiveOctreeNode) -> bool {
    bounds_face_adjacent(
        vec3_from_array(a.bounds_min),
        vec3_from_array(a.bounds_max),
        vec3_from_array(b.bounds_min),
        vec3_from_array(b.bounds_max),
    )
}

fn bounds_face_adjacent(a_min: Vec3, a_max: Vec3, b_min: Vec3, b_max: Vec3) -> bool {
    let eps = 1.0e-5_f32;
    for axis in 0..3 {
        let touch = (component(a_max, axis) - component(b_min, axis)).abs() <= eps
            || (component(b_max, axis) - component(a_min, axis)).abs() <= eps;
        if !touch {
            continue;
        }
        let mut overlap = true;
        for other in 0..3 {
            if other == axis {
                continue;
            }
            let lo = component(a_min, other).max(component(b_min, other));
            let hi = component(a_max, other).min(component(b_max, other));
            if hi - lo <= eps {
                overlap = false;
                break;
            }
        }
        if overlap {
            return true;
        }
    }
    false
}

fn cell_corners(min: Vec3, max: Vec3) -> [Vec3; 8] {
    [
        Vec3::new(min.x, min.y, min.z),
        Vec3::new(max.x, min.y, min.z),
        Vec3::new(min.x, max.y, min.z),
        Vec3::new(max.x, max.y, min.z),
        Vec3::new(min.x, min.y, max.z),
        Vec3::new(max.x, min.y, max.z),
        Vec3::new(min.x, max.y, max.z),
        Vec3::new(max.x, max.y, max.z),
    ]
}

fn estimate_normal(sdf: &dyn Sdf, p: Vec3, eps: f32) -> Vec3 {
    Vec3::new(
        sdf.distance(p + Vec3::X * eps) - sdf.distance(p - Vec3::X * eps),
        sdf.distance(p + Vec3::Y * eps) - sdf.distance(p - Vec3::Y * eps),
        sdf.distance(p + Vec3::Z * eps) - sdf.distance(p - Vec3::Z * eps),
    )
    .normalize_or_zero()
}

fn estimate_normal_cached(sdf: &dyn Sdf, p: Vec3, eps: f32, cache: &mut SamplingCache) -> Vec3 {
    Vec3::new(
        cache.distance(sdf, p + Vec3::X * eps) - cache.distance(sdf, p - Vec3::X * eps),
        cache.distance(sdf, p + Vec3::Y * eps) - cache.distance(sdf, p - Vec3::Y * eps),
        cache.distance(sdf, p + Vec3::Z * eps) - cache.distance(sdf, p - Vec3::Z * eps),
    )
    .normalize_or_zero()
}

fn max_normal_angle(normals: &[Vec3]) -> f64 {
    let mut max_angle = 0.0;
    for i in 0..normals.len() {
        for j in (i + 1)..normals.len() {
            max_angle = f64::max(max_angle, angle_degrees(normals[i], normals[j]));
        }
    }
    max_angle
}

fn average_normal(normals: &[Vec3]) -> Vec3 {
    let mut sum = Vec3::ZERO;
    for normal in normals {
        sum += *normal;
    }
    sum.normalize_or_zero()
}

fn angle_degrees(a: Vec3, b: Vec3) -> f64 {
    if a.length_squared() == 0.0 || b.length_squared() == 0.0 {
        return 0.0;
    }
    a.normalize()
        .dot(b.normalize())
        .clamp(-1.0, 1.0)
        .acos()
        .to_degrees() as f64
}

fn component(v: Vec3, axis: usize) -> f32 {
    match axis {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    }
}

fn tuple_key(v: Vec3) -> (i64, i64, i64) {
    (
        (v.x as f64 * 1.0e6).round() as i64,
        (v.y as f64 * 1.0e6).round() as i64,
        (v.z as f64 * 1.0e6).round() as i64,
    )
}

fn point_cache_key(v: Vec3) -> [i64; 3] {
    [
        (v.x as f64 * 1.0e6).round() as i64,
        (v.y as f64 * 1.0e6).round() as i64,
        (v.z as f64 * 1.0e6).round() as i64,
    ]
}

fn array_from_vec3(v: Vec3) -> [f64; 3] {
    [v.x as f64, v.y as f64, v.z as f64]
}

fn vec3_from_array(v: [f64; 3]) -> Vec3 {
    Vec3::new(v[0] as f32, v[1] as f32, v[2] as f32)
}

fn percentile(values: &[f64], q: f64) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let idx = ((values.len().saturating_sub(1)) as f64 * q.clamp(0.0, 1.0)).round() as usize;
    values[idx.min(values.len() - 1)]
}

fn stable_hash_hex(bytes: &[u8]) -> String {
    let mut hash = 0xcbf29ce484222325_u64;
    for byte in bytes {
        hash ^= *byte as u64;
        hash = hash.wrapping_mul(0x100000001b3);
    }
    format!("{hash:016x}")
}

fn elapsed_ms(start: Instant) -> f64 {
    start.elapsed().as_secs_f64() * 1000.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::cfd_query::QueryValueStatus;
    use crate::sdf::primitives::Sphere;
    use crate::sdf::procedural_features::{
        ProceduralFeatureDiagnostics, ProceduralFeatureQueryResult, SurfaceSectorId,
        WingSurfaceSide,
    };

    #[test]
    fn adaptive_octree_is_deterministic_for_sphere() {
        let sphere = Sphere::new(4.0);
        let config = AdaptiveSamplingConfig {
            min_depth: 2,
            max_depth: 3,
            bounds_padding: 2.0,
            base_cell_size: 2.0,
            max_leaf_cells: 10_000,
            ..AdaptiveSamplingConfig::default()
        };
        let a = build_adaptive_octree(&sphere, &config).unwrap();
        let b = build_adaptive_octree(&sphere, &config).unwrap();
        assert_eq!(a.octree_hash(), b.octree_hash());
        assert!(a.summary.leaf_count > 0);
        assert!(a.summary.max_face_neighbor_level_delta <= 1);
    }

    #[test]
    fn feature_thickness_policy_reaches_requested_target_size() {
        let config = AdaptiveSamplingConfig {
            trailing_edge_thickness: 1.0,
            required_feature_intervals: 4.0,
            ..AdaptiveSamplingConfig::default()
        };
        assert!((config.feature_target_size() - 0.25).abs() < 1.0e-6);
    }

    #[test]
    fn generic_feature_summary_cannot_access_extra_depth() {
        let config = AdaptiveSamplingConfig {
            surface_crossing_max_depth: 7,
            feature_max_depth: 8,
            ..AdaptiveSamplingConfig::default()
        };
        let summary = AdaptiveFeatureSummary {
            in_feature_band: true,
            selected_extra_depth_eligible: false,
            ..AdaptiveFeatureSummary::default()
        };

        assert_eq!(feature_refinement_limit(&config, &summary), 7);
    }

    #[test]
    fn selected_te_envelope_can_access_extra_depth() {
        let config = AdaptiveSamplingConfig {
            base_cell_size: 5.269230842590332,
            feature_band_width: 4.5,
            trailing_edge_thickness: 1.0,
            surface_crossing_max_depth: 7,
            feature_max_depth: 8,
            ..AdaptiveSamplingConfig::default()
        };
        let query = test_query(
            "lofted_wing.main.trailing_edge.upper_cap_boundary",
            ProceduralFeatureType::TrailingEdgeUpperCapBoundary,
            "lofted_wing.main.surface.upper",
            1.0,
            Some(0.5),
            Some(0.98),
            None,
            Some(CapRole::TrailingEdgeCap),
        );
        let envelope = selected_extra_depth_envelope(&query, &config).unwrap();
        let summary = AdaptiveFeatureSummary {
            in_feature_band: true,
            selected_extra_depth_eligible: true,
            selected_extra_depth_band: Some(envelope.band_id),
            ..AdaptiveFeatureSummary::default()
        };

        assert_eq!(feature_refinement_limit(&config, &summary), 8);
        assert_eq!(
            summary.selected_extra_depth_band.as_deref(),
            Some("upper_te_boundary")
        );
    }

    #[test]
    fn selected_envelope_rejects_broad_midspan_ordinary_surface() {
        let config = AdaptiveSamplingConfig {
            base_cell_size: 5.269230842590332,
            feature_band_width: 4.5,
            trailing_edge_thickness: 1.0,
            ..AdaptiveSamplingConfig::default()
        };
        let query = test_query(
            "lofted_wing.main.leading_edge",
            ProceduralFeatureType::LeadingEdge,
            "lofted_wing.main.surface.upper",
            0.5,
            Some(0.5),
            Some(0.5),
            None,
            Some(CapRole::None),
        );

        assert!(selected_extra_depth_envelope(&query, &config).is_none());
    }

    fn test_query(
        feature_id: &str,
        feature_type: ProceduralFeatureType,
        sector_id: &str,
        distance: f64,
        span: Option<f64>,
        chord: Option<f64>,
        airfoil: Option<f64>,
        cap_role: Option<CapRole>,
    ) -> ProceduralFeatureQueryResult {
        ProceduralFeatureQueryResult {
            schema_version: "test".to_string(),
            query_position: [0.0, 0.0, 0.0],
            component_id: Some("lofted_wing.main".to_string()),
            feature_id: Some(feature_id.to_string()),
            feature_type,
            support_status: QueryValueStatus::Native,
            closest_point: Some([0.0, 0.0, 0.0]),
            distance_to_feature: Some(distance),
            tangent: None,
            normal_sector: Some(SurfaceSectorId {
                stable_id: sector_id.to_string(),
                support_status: QueryValueStatus::Native,
            }),
            span_parameter: span,
            chord_parameter: chord,
            airfoil_parameter: airfoil,
            side: Some(WingSurfaceSide::Upper),
            cap_role,
            endpoint_role: None,
            connected_feature_ids: Vec::new(),
            all_matches: Vec::new(),
            diagnostics: ProceduralFeatureDiagnostics {
                success: true,
                warnings: Vec::new(),
                errors: Vec::new(),
                ambiguity: None,
                query_tolerance: Some(1.0),
                method: "test".to_string(),
                feature_to_sdf_residual: None,
            },
        }
    }
}
