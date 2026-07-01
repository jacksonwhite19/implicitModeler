#![allow(dead_code)]

use crate::sdf::Sdf;
use glam::Vec3;
use rayon::prelude::*;
use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, mpsc};
use std::thread;
use std::time::Instant;

#[derive(Clone, Debug, PartialEq)]
pub struct SdfBounds {
    pub min: Vec3,
    pub max: Vec3,
}

impl SdfBounds {
    pub fn new(min: Vec3, max: Vec3) -> Self {
        Self { min, max }
    }

    pub fn from_center_radius(center: Vec3, radius: f32) -> Self {
        let radius = radius.abs();
        Self {
            min: center - Vec3::splat(radius),
            max: center + Vec3::splat(radius),
        }
    }

    pub fn from_points(points: &[Vec3]) -> Option<Self> {
        let mut iter = points.iter().copied();
        let first = iter.next()?;
        if !first.is_finite() {
            return None;
        }
        let mut min = first;
        let mut max = first;
        for point in iter {
            if !point.is_finite() {
                return None;
            }
            min = min.min(point);
            max = max.max(point);
        }
        Some(Self { min, max })
    }

    pub fn expanded(&self, amount: f32) -> Self {
        Self {
            min: self.min - Vec3::splat(amount),
            max: self.max + Vec3::splat(amount),
        }
    }

    pub fn translated(&self, offset: Vec3) -> Self {
        Self {
            min: self.min + offset,
            max: self.max + offset,
        }
    }

    pub fn union(&self, other: &Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    pub fn contains(&self, point: Vec3) -> bool {
        point.x >= self.min.x
            && point.y >= self.min.y
            && point.z >= self.min.z
            && point.x <= self.max.x
            && point.y <= self.max.y
            && point.z <= self.max.z
    }

    pub fn contains_bounds(&self, other: &Self) -> bool {
        self.contains(other.min) && self.contains(other.max)
    }

    pub fn intersects(&self, other: &Self) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }

    pub fn intersection(&self, other: &Self) -> Option<Self> {
        if !self.intersects(other) {
            return None;
        }
        Some(Self {
            min: self.min.max(other.min),
            max: self.max.min(other.max),
        })
    }

    pub fn clamped_to(&self, other: &Self) -> Self {
        Self {
            min: self.min.max(other.min),
            max: self.max.min(other.max),
        }
    }

    pub fn size(&self) -> Vec3 {
        self.max - self.min
    }

    pub fn is_valid(&self) -> bool {
        self.min.x <= self.max.x
            && self.min.y <= self.max.y
            && self.min.z <= self.max.z
            && self.min.is_finite()
            && self.max.is_finite()
    }

    pub fn corners(&self) -> [Vec3; 8] {
        [
            Vec3::new(self.min.x, self.min.y, self.min.z),
            Vec3::new(self.max.x, self.min.y, self.min.z),
            Vec3::new(self.min.x, self.max.y, self.min.z),
            Vec3::new(self.max.x, self.max.y, self.min.z),
            Vec3::new(self.min.x, self.min.y, self.max.z),
            Vec3::new(self.max.x, self.min.y, self.max.z),
            Vec3::new(self.min.x, self.max.y, self.max.z),
            Vec3::new(self.max.x, self.max.y, self.max.z),
        ]
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct SdfNodeDependency {
    pub role: String,
    pub metadata: SdfNodeMetadata,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SdfFeatureRegion {
    pub feature_id: String,
    pub bounds: SdfBounds,
    pub min_feature_size: f32,
}

impl SdfFeatureRegion {
    pub fn new(feature_id: impl Into<String>, bounds: SdfBounds, min_feature_size: f32) -> Self {
        Self {
            feature_id: feature_id.into(),
            bounds,
            min_feature_size,
        }
    }

    pub fn recommended_grid_spacing(
        &self,
        samples_per_feature: f32,
        min_spacing: f32,
    ) -> Option<f32> {
        if !self.bounds.is_valid()
            || !self.min_feature_size.is_finite()
            || self.min_feature_size <= 0.0
        {
            return None;
        }
        Some((self.min_feature_size / samples_per_feature.max(1.0)).max(min_spacing.max(1.0e-4)))
    }

    pub fn translated(&self, offset: Vec3) -> Self {
        Self {
            feature_id: self.feature_id.clone(),
            bounds: self.bounds.translated(offset),
            min_feature_size: self.min_feature_size,
        }
    }

    pub fn transformed_bounds(&self, transform: impl Fn(Vec3) -> Vec3) -> Option<Self> {
        let transformed_corners: Vec<_> =
            self.bounds.corners().into_iter().map(transform).collect();
        Some(Self {
            feature_id: self.feature_id.clone(),
            bounds: SdfBounds::from_points(&transformed_corners)?,
            min_feature_size: self.min_feature_size,
        })
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ConditionedCacheMetadata {
    pub state: ConditionedCacheState,
    pub generation: u64,
    pub block_count: usize,
    pub grid_spacing: f32,
    pub interface_band: f32,
    pub confidence: f32,
    pub publication_confidence: f32,
    pub quality_confidence: f32,
    pub anchor_count: usize,
    pub projection_anchor_count: usize,
    pub narrow_band_block_count: usize,
    pub max_iteration_count: usize,
    pub adaptive_region_count: usize,
    pub adaptive_sample_count: usize,
    pub adaptive_narrow_band_region_count: usize,
    pub adaptive_narrow_band_sample_count: usize,
    pub adaptive_min_grid_spacing: Option<f32>,
    pub adaptive_max_grid_spacing: Option<f32>,
    pub adaptive_feature_id_count: usize,
    pub adaptive_requested_feature_id_count: usize,
    pub adaptive_feature_coverage: f32,
    pub adaptive_requested_region_count: usize,
    pub adaptive_skipped_region_count: usize,
    pub adaptive_oversized_region_count: usize,
    pub adaptive_requested_min_grid_spacing: Option<f32>,
    pub block_sample_count: usize,
    pub block_sample_bytes: usize,
    pub surface_refinement_block_count: usize,
    pub surface_refinement_cell_count: usize,
    pub surface_refinement_sample_bytes: usize,
    pub surface_refinement_distance_cell_count: usize,
    pub surface_refinement_distance_sample_bytes: usize,
    pub surface_refinement_point_count: usize,
    pub surface_refinement_point_bytes: usize,
    pub adaptive_stored_sample_count: usize,
    pub adaptive_stored_sample_bytes: usize,
    pub adaptive_dense_region_count: usize,
    pub adaptive_sparse_region_count: usize,
    pub adaptive_dense_value_count: usize,
    pub adaptive_sparse_value_count: usize,
    pub total_stored_sample_count: usize,
    pub total_stored_sample_bytes: usize,
    pub estimated_metadata_bytes: usize,
    pub estimated_total_bytes: usize,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SdfOperationMetadata {
    pub dirty_expansion: f32,
    pub interface_radius: f32,
    pub child_change_affects_siblings: bool,
}

impl Default for SdfOperationMetadata {
    fn default() -> Self {
        Self {
            dirty_expansion: 0.0,
            interface_radius: 0.0,
            child_change_affects_siblings: false,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BoundQuality {
    Exact,
    Conservative,
    Estimated,
    Unknown,
}

impl BoundQuality {
    pub fn is_conditionable(self) -> bool {
        !matches!(self, Self::Unknown)
    }

    pub fn is_approximate(self) -> bool {
        !matches!(self, Self::Exact)
    }

    pub fn conservative_after_operation(self) -> Self {
        match self {
            Self::Unknown => Self::Unknown,
            Self::Estimated => Self::Estimated,
            Self::Exact | Self::Conservative => Self::Conservative,
        }
    }

    pub fn combine_with(self, other: Self) -> Self {
        match (self, other) {
            (Self::Unknown, _) | (_, Self::Unknown) => Self::Unknown,
            (Self::Estimated, _) | (_, Self::Estimated) => Self::Estimated,
            (Self::Conservative, _) | (_, Self::Conservative) => Self::Conservative,
            (Self::Exact, Self::Exact) => Self::Exact,
        }
    }

    pub fn combine_all(values: impl IntoIterator<Item = Self>) -> Self {
        values
            .into_iter()
            .reduce(|acc, quality| acc.combine_with(quality))
            .unwrap_or(Self::Unknown)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct SdfNodeMetadata {
    pub node_kind: String,
    pub support_bounds: Option<SdfBounds>,
    pub feature_ids: Vec<String>,
    pub feature_regions: Vec<SdfFeatureRegion>,
    pub dependencies: Vec<SdfNodeDependency>,
    pub min_feature_size: Option<f32>,
    pub bounds_are_approximate: bool,
    pub bound_quality: BoundQuality,
    pub parameter_fingerprint: u64,
    pub operation: SdfOperationMetadata,
    pub conditioned_cache: Option<ConditionedCacheMetadata>,
}

impl SdfNodeMetadata {
    pub fn new(node_kind: impl Into<String>) -> Self {
        Self {
            node_kind: node_kind.into(),
            support_bounds: None,
            feature_ids: Vec::new(),
            feature_regions: Vec::new(),
            dependencies: Vec::new(),
            min_feature_size: None,
            bounds_are_approximate: false,
            bound_quality: BoundQuality::Unknown,
            parameter_fingerprint: 0,
            operation: SdfOperationMetadata::default(),
            conditioned_cache: None,
        }
    }

    pub fn unknown() -> Self {
        Self::new("unknown").with_approximate_bounds()
    }

    pub fn with_support_bounds(mut self, bounds: SdfBounds) -> Self {
        let quality = if self.bounds_are_approximate {
            BoundQuality::Conservative
        } else {
            BoundQuality::Exact
        };
        self.set_support_bounds(Some(bounds), quality);
        self
    }

    pub fn with_conservative_bounds(mut self, bounds: SdfBounds) -> Self {
        self.set_support_bounds(Some(bounds), BoundQuality::Conservative);
        self
    }

    pub fn with_estimated_bounds(mut self, bounds: SdfBounds) -> Self {
        self.set_support_bounds(Some(bounds), BoundQuality::Estimated);
        self
    }

    pub fn with_approximate_bounds(mut self) -> Self {
        self.bounds_are_approximate = true;
        self.bound_quality = match (self.support_bounds.is_some(), self.bound_quality) {
            (false, _) => BoundQuality::Unknown,
            (true, BoundQuality::Exact) => BoundQuality::Conservative,
            (true, quality) => quality,
        };
        self
    }

    pub fn with_bound_quality(mut self, quality: BoundQuality) -> Self {
        if self.support_bounds.is_none() {
            self.bound_quality = BoundQuality::Unknown;
            self.bounds_are_approximate = true;
        } else {
            self.bound_quality = quality;
            self.bounds_are_approximate = quality.is_approximate();
        }
        self
    }

    pub fn set_support_bounds(&mut self, bounds: Option<SdfBounds>, quality: BoundQuality) {
        self.support_bounds = bounds;
        if self.support_bounds.is_some() {
            self.bound_quality = quality;
            self.bounds_are_approximate = quality.is_approximate();
        } else {
            self.bound_quality = BoundQuality::Unknown;
            self.bounds_are_approximate = true;
        }
    }

    pub fn is_conditionable(&self) -> bool {
        self.support_bounds
            .as_ref()
            .is_some_and(SdfBounds::is_valid)
            && self.bound_quality.is_conditionable()
    }

    pub fn with_feature_id(mut self, feature_id: impl Into<String>) -> Self {
        self.add_feature_id(feature_id);
        self
    }

    pub fn add_feature_id(&mut self, feature_id: impl Into<String>) {
        let feature_id = feature_id.into();
        if !self.feature_ids.contains(&feature_id) {
            self.feature_ids.push(feature_id);
        }
    }

    pub fn with_feature_region(
        mut self,
        feature_id: impl Into<String>,
        bounds: SdfBounds,
        min_feature_size: f32,
    ) -> Self {
        self.add_feature_region(SdfFeatureRegion::new(feature_id, bounds, min_feature_size));
        self
    }

    pub fn add_feature_region(&mut self, region: SdfFeatureRegion) {
        if !region.bounds.is_valid()
            || !region.min_feature_size.is_finite()
            || region.min_feature_size <= 0.0
        {
            return;
        }
        self.merge_min_feature_size(region.min_feature_size);
        self.add_feature_id(region.feature_id.clone());
        self.feature_regions.push(region);
    }

    pub fn clear_feature_regions(&mut self) {
        self.feature_regions.clear();
    }

    pub fn translate_feature_regions(&mut self, offset: Vec3) {
        self.feature_regions = self
            .feature_regions
            .iter()
            .map(|region| region.translated(offset))
            .collect();
    }

    pub fn transform_feature_regions(&mut self, transform: impl Fn(Vec3) -> Vec3 + Copy) {
        self.feature_regions = self
            .feature_regions
            .iter()
            .filter_map(|region| region.transformed_bounds(transform))
            .collect();
    }

    pub fn scale_feature_sizes(&mut self, factor: f32) {
        if !factor.is_finite() || factor <= 0.0 {
            return;
        }
        self.min_feature_size = self.min_feature_size.map(|size| size * factor);
        for region in &mut self.feature_regions {
            region.min_feature_size *= factor;
        }
    }

    pub fn with_min_feature_size(mut self, size: f32) -> Self {
        self.merge_min_feature_size(size);
        self
    }

    pub fn merge_min_feature_size(&mut self, size: f32) {
        if !size.is_finite() || size <= 0.0 {
            return;
        }
        self.min_feature_size = Some(match self.min_feature_size {
            Some(existing) => existing.min(size),
            None => size,
        });
    }

    pub fn recommended_conditioning_spacing(
        &self,
        samples_per_feature: f32,
        min_spacing: f32,
    ) -> Option<f32> {
        let samples_per_feature = samples_per_feature.max(1.0);
        let min_spacing = min_spacing.max(1.0e-4);
        self.min_feature_size
            .filter(|size| size.is_finite() && *size > 0.0)
            .map(|size| (size / samples_per_feature).max(min_spacing))
    }

    pub fn with_parameter_fingerprint(mut self, fingerprint: u64) -> Self {
        self.parameter_fingerprint = fingerprint;
        self
    }

    pub fn with_f32_parameters(mut self, values: impl IntoIterator<Item = f32>) -> Self {
        self.parameter_fingerprint = metadata_fingerprint_from_f32s(values);
        self
    }

    pub fn with_dirty_expansion(mut self, amount: f32) -> Self {
        self.operation.dirty_expansion = self.operation.dirty_expansion.max(amount.abs());
        self
    }

    pub fn with_child_interface_radius(mut self, radius: f32) -> Self {
        self.operation.interface_radius = self.operation.interface_radius.max(radius.abs());
        self.operation.child_change_affects_siblings = true;
        self
    }

    pub fn with_child_change_affects_siblings(mut self) -> Self {
        self.operation.child_change_affects_siblings = true;
        self
    }

    pub fn with_dependency(mut self, role: impl Into<String>, metadata: SdfNodeMetadata) -> Self {
        if let Some(min_feature_size) = metadata.min_feature_size {
            self.merge_min_feature_size(min_feature_size);
        }
        for region in &metadata.feature_regions {
            self.add_feature_region(region.clone());
        }
        self.dependencies.push(SdfNodeDependency {
            role: role.into(),
            metadata,
        });
        self
    }

    pub fn inherit_dependency_feature_ids(mut self) -> Self {
        let inherited: Vec<_> = self
            .dependencies
            .iter()
            .flat_map(|dependency| dependency.metadata.feature_ids.iter().cloned())
            .collect();
        for feature_id in inherited {
            self.add_feature_id(feature_id);
        }
        self
    }

    pub fn support_change_bounds(&self, next: &Self) -> Option<SdfBounds> {
        if !self.is_conditionable() || !next.is_conditionable() {
            return None;
        }
        Some(
            self.support_bounds
                .as_ref()?
                .union(next.support_bounds.as_ref()?),
        )
    }

    pub fn localized_change_bounds(&self, next: &Self) -> Option<SdfBounds> {
        if !self.is_conditionable() || !next.is_conditionable() {
            return None;
        }

        if self.node_kind != next.node_kind || self.dependencies.len() != next.dependencies.len() {
            return self.support_change_bounds(next);
        }

        let dependency_roles_match = self
            .dependencies
            .iter()
            .zip(next.dependencies.iter())
            .all(|(previous, next)| previous.role == next.role);
        if !dependency_roles_match {
            return self.support_change_bounds(next);
        }

        let dependency_changed = self
            .dependencies
            .iter()
            .zip(next.dependencies.iter())
            .any(|(previous, next)| previous.metadata != next.metadata);

        if dependency_changed && metadata_node_stores_child_bounds_in_local_space(&self.node_kind) {
            return self.support_change_bounds(next);
        }

        let mut child_bounds: Option<SdfBounds> = None;
        for (index, (previous, next_dependency)) in self
            .dependencies
            .iter()
            .zip(next.dependencies.iter())
            .enumerate()
            .filter(|(_, (previous, next))| previous.metadata != next.metadata)
        {
            let Some(local_child_bounds) = previous
                .metadata
                .localized_change_bounds(&next_dependency.metadata)
            else {
                return self.support_change_bounds(next);
            };
            let Some(propagated_bounds) =
                self.propagate_child_change_bounds(next, index, local_child_bounds)
            else {
                return self.support_change_bounds(next);
            };
            child_bounds = Some(match child_bounds {
                Some(acc) => acc.union(&propagated_bounds),
                None => propagated_bounds,
            });
        }

        if child_bounds.is_some() {
            return child_bounds;
        }

        let own_metadata_changed = self.support_bounds != next.support_bounds
            || self.feature_ids != next.feature_ids
            || self.bounds_are_approximate != next.bounds_are_approximate
            || self.bound_quality != next.bound_quality
            || self.min_feature_size != next.min_feature_size
            || self.feature_regions != next.feature_regions
            || self.parameter_fingerprint != next.parameter_fingerprint
            || self.operation != next.operation;

        if own_metadata_changed {
            self.own_change_bounds(next)
                .or_else(|| self.support_change_bounds(next))
        } else {
            None
        }
    }

    fn own_change_bounds(&self, next: &Self) -> Option<SdfBounds> {
        if self.node_kind != next.node_kind {
            return None;
        }

        if metadata_node_uses_smooth_child_interface(&self.node_kind) {
            return self.smooth_child_interface_change_bounds(next);
        }

        None
    }

    fn smooth_child_interface_change_bounds(&self, next: &Self) -> Option<SdfBounds> {
        let radius = self
            .operation
            .interface_radius
            .max(next.operation.interface_radius)
            .max(self.operation.dirty_expansion)
            .max(next.operation.dirty_expansion);
        let previous = expanded_child_interface_bounds(self, radius)?;
        let next = expanded_child_interface_bounds(next, radius)?;
        Some(previous.union(&next))
    }

    fn propagate_child_change_bounds(
        &self,
        next: &Self,
        changed_child_index: usize,
        child_bounds: SdfBounds,
    ) -> Option<SdfBounds> {
        let dirty_expansion = self
            .operation
            .dirty_expansion
            .max(next.operation.dirty_expansion);
        let interface_radius = self
            .operation
            .interface_radius
            .max(next.operation.interface_radius);
        let mut propagated = child_bounds.expanded(dirty_expansion);

        let affects_siblings = self.operation.child_change_affects_siblings
            || next.operation.child_change_affects_siblings;
        if !affects_siblings {
            return Some(propagated);
        }

        let interaction_bounds = propagated.expanded(interface_radius);
        for (index, (previous_sibling, next_sibling)) in self
            .dependencies
            .iter()
            .zip(next.dependencies.iter())
            .enumerate()
        {
            if index == changed_child_index {
                continue;
            }

            let sibling_bounds = match (
                previous_sibling.metadata.support_bounds.as_ref(),
                next_sibling.metadata.support_bounds.as_ref(),
            ) {
                (Some(previous), Some(next)) => previous.union(next),
                (Some(bounds), None) | (None, Some(bounds)) => bounds.clone(),
                (None, None) => return None,
            }
            .expanded(interface_radius);

            if let Some(interface_bounds) = interaction_bounds.intersection(&sibling_bounds) {
                propagated = propagated.union(&interface_bounds);
            }
        }

        Some(propagated)
    }
}

fn metadata_node_stores_child_bounds_in_local_space(node_kind: &str) -> bool {
    matches!(
        node_kind,
        "translate" | "rotate" | "scale" | "twist" | "bend"
    )
}

fn metadata_node_uses_smooth_child_interface(node_kind: &str) -> bool {
    matches!(
        node_kind,
        "smooth_union" | "smooth_intersect" | "smooth_subtract"
    )
}

fn expanded_child_interface_bounds(metadata: &SdfNodeMetadata, radius: f32) -> Option<SdfBounds> {
    let mut dependencies = metadata.dependencies.iter();
    let first = dependencies.next()?.metadata.support_bounds.as_ref()?;
    let second = dependencies.next()?.metadata.support_bounds.as_ref()?;
    let radius = radius.abs().max(0.0);
    first
        .expanded(radius)
        .intersection(&second.expanded(radius))
}

const METADATA_FINGERPRINT_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
const METADATA_FINGERPRINT_PRIME: u64 = 0x0000_0100_0000_01b3;

pub fn metadata_fingerprint_from_f32s(values: impl IntoIterator<Item = f32>) -> u64 {
    let mut fingerprint = METADATA_FINGERPRINT_OFFSET;
    for value in values {
        fingerprint ^= value.to_bits() as u64;
        fingerprint = fingerprint.wrapping_mul(METADATA_FINGERPRINT_PRIME);
    }
    fingerprint
}

pub fn union_metadata_bounds<'a>(
    metadata: impl IntoIterator<Item = &'a SdfNodeMetadata>,
) -> Option<SdfBounds> {
    metadata
        .into_iter()
        .filter_map(|metadata| metadata.support_bounds.clone())
        .reduce(|acc, bounds| acc.union(&bounds))
}

pub fn combine_bound_quality<'a>(
    metadata: impl IntoIterator<Item = &'a SdfNodeMetadata>,
) -> BoundQuality {
    BoundQuality::combine_all(metadata.into_iter().map(|metadata| {
        if metadata.support_bounds.is_some() {
            metadata.bound_quality
        } else {
            BoundQuality::Unknown
        }
    }))
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum MetadataAuditIssueKind {
    MissingSupportBounds,
    InvalidSupportBounds,
    UnknownBoundQuality,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MetadataAuditIssue {
    pub path: String,
    pub node_kind: String,
    pub issue: MetadataAuditIssueKind,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MetadataAuditReport {
    pub issues: Vec<MetadataAuditIssue>,
}

impl MetadataAuditReport {
    pub fn is_clean(&self) -> bool {
        self.issues.is_empty()
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct MetadataCoverageReport {
    pub total_node_count: usize,
    pub conditionable_node_count: usize,
    pub unconditionable_node_count: usize,
    pub dependency_edge_count: usize,
    pub dependency_role_counts: HashMap<String, usize>,
    pub node_kind_counts: HashMap<String, usize>,
    pub feature_id_reference_count: usize,
    pub unique_feature_id_count: usize,
    pub feature_region_count: usize,
    pub nodes_with_min_feature_size: usize,
    pub nodes_with_parameter_fingerprint: usize,
    pub child_interface_node_count: usize,
    pub smooth_interface_node_count: usize,
    pub approximate_bound_node_count: usize,
    pub conservative_bound_node_count: usize,
    pub estimated_bound_node_count: usize,
}

impl MetadataCoverageReport {
    pub fn is_conditioning_ready(&self) -> bool {
        self.total_node_count > 0 && self.unconditionable_node_count == 0
    }

    pub fn has_locality_hints(&self) -> bool {
        self.feature_region_count > 0
            || self.unique_feature_id_count > 0
            || self.nodes_with_parameter_fingerprint > 0
            || self.child_interface_node_count > 0
    }

    pub fn node_kind_count(&self, node_kind: &str) -> usize {
        self.node_kind_counts.get(node_kind).copied().unwrap_or(0)
    }

    pub fn dependency_role_count(&self, role: &str) -> usize {
        self.dependency_role_counts.get(role).copied().unwrap_or(0)
    }
}

pub fn audit_conditionable_geometry(metadata: &SdfNodeMetadata) -> MetadataAuditReport {
    let mut issues = Vec::new();
    audit_metadata_node(metadata, "root", &mut issues);
    MetadataAuditReport { issues }
}

pub fn summarize_conditioning_metadata(metadata: &SdfNodeMetadata) -> MetadataCoverageReport {
    let mut report = MetadataCoverageReport::default();
    let mut unique_feature_ids = HashSet::new();
    summarize_metadata_node(metadata, &mut report, &mut unique_feature_ids);
    report.unique_feature_id_count = unique_feature_ids.len();
    report
}

fn summarize_metadata_node(
    metadata: &SdfNodeMetadata,
    report: &mut MetadataCoverageReport,
    unique_feature_ids: &mut HashSet<String>,
) {
    report.total_node_count += 1;
    if metadata.is_conditionable() {
        report.conditionable_node_count += 1;
    } else {
        report.unconditionable_node_count += 1;
    }
    *report
        .node_kind_counts
        .entry(metadata.node_kind.clone())
        .or_default() += 1;
    report.feature_id_reference_count += metadata.feature_ids.len();
    unique_feature_ids.extend(metadata.feature_ids.iter().cloned());
    report.feature_region_count += metadata.feature_regions.len();
    if metadata.min_feature_size.is_some() {
        report.nodes_with_min_feature_size += 1;
    }
    if metadata.parameter_fingerprint != 0 {
        report.nodes_with_parameter_fingerprint += 1;
    }
    if metadata.operation.child_change_affects_siblings {
        report.child_interface_node_count += 1;
    }
    if metadata.operation.interface_radius > 0.0 {
        report.smooth_interface_node_count += 1;
    }
    if metadata.bounds_are_approximate {
        report.approximate_bound_node_count += 1;
    }
    match metadata.bound_quality {
        BoundQuality::Conservative => report.conservative_bound_node_count += 1,
        BoundQuality::Estimated => report.estimated_bound_node_count += 1,
        BoundQuality::Exact | BoundQuality::Unknown => {}
    }

    report.dependency_edge_count += metadata.dependencies.len();
    for dependency in &metadata.dependencies {
        *report
            .dependency_role_counts
            .entry(dependency.role.clone())
            .or_default() += 1;
        summarize_metadata_node(&dependency.metadata, report, unique_feature_ids);
    }
}

pub fn assert_conditionable_geometry(
    metadata: &SdfNodeMetadata,
) -> Result<(), MetadataAuditReport> {
    let report = audit_conditionable_geometry(metadata);
    if report.is_clean() {
        Ok(())
    } else {
        Err(report)
    }
}

fn audit_metadata_node(
    metadata: &SdfNodeMetadata,
    path: &str,
    issues: &mut Vec<MetadataAuditIssue>,
) {
    match metadata.support_bounds.as_ref() {
        Some(bounds) if !bounds.is_valid() => issues.push(MetadataAuditIssue {
            path: path.to_string(),
            node_kind: metadata.node_kind.clone(),
            issue: MetadataAuditIssueKind::InvalidSupportBounds,
        }),
        Some(_) if !metadata.bound_quality.is_conditionable() => issues.push(MetadataAuditIssue {
            path: path.to_string(),
            node_kind: metadata.node_kind.clone(),
            issue: MetadataAuditIssueKind::UnknownBoundQuality,
        }),
        Some(_) => {}
        None => issues.push(MetadataAuditIssue {
            path: path.to_string(),
            node_kind: metadata.node_kind.clone(),
            issue: MetadataAuditIssueKind::MissingSupportBounds,
        }),
    }

    for (index, dependency) in metadata.dependencies.iter().enumerate() {
        let child_path = format!("{path}/{}[{index}]", dependency.role);
        audit_metadata_node(&dependency.metadata, &child_path, issues);
    }
}

pub struct FeatureTaggedSdf {
    pub feature_id: String,
    pub child: Arc<dyn Sdf>,
}

impl FeatureTaggedSdf {
    pub fn new(feature_id: impl Into<String>, child: Arc<dyn Sdf>) -> Self {
        Self {
            feature_id: feature_id.into(),
            child,
        }
    }
}

impl Sdf for FeatureTaggedSdf {
    fn distance(&self, point: Vec3) -> f32 {
        self.child.distance(point)
    }

    fn metadata(&self) -> SdfNodeMetadata {
        let child_metadata = self.child.metadata();
        let support_bounds = child_metadata.support_bounds.clone();
        let mut metadata = SdfNodeMetadata::new("feature")
            .with_dependency("child", child_metadata)
            .with_feature_id(self.feature_id.clone())
            .inherit_dependency_feature_ids();
        metadata.set_support_bounds(
            support_bounds,
            metadata.dependencies[0].metadata.bound_quality,
        );
        metadata
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GeometryEditKind {
    FullRebuild,
    Parameter,
    Feature,
    BlendRadius,
    Boolean,
    Shell,
    Offset,
    Transform,
}

#[derive(Clone, Debug, PartialEq)]
pub struct GeometryEdit {
    pub kind: GeometryEditKind,
    pub feature_id: Option<String>,
    pub dirty_region: DirtyRegion,
    pub previous_value: Option<f32>,
    pub new_value: Option<f32>,
}

impl GeometryEdit {
    pub fn parameter_changed(
        kind: GeometryEditKind,
        source: DirtyRegionSource,
        feature_id: impl Into<String>,
        affected_bounds: SdfBounds,
        previous_value: f32,
        new_value: f32,
        halo: f32,
    ) -> Self {
        let feature_id = feature_id.into();
        let mut dirty_region = DirtyRegion::new(source, affected_bounds).with_halo(halo);
        dirty_region.feature_ids.push(feature_id.clone());
        Self {
            kind,
            feature_id: Some(feature_id),
            dirty_region,
            previous_value: Some(previous_value),
            new_value: Some(new_value),
        }
    }

    pub fn blend_radius_changed(
        feature_id: impl Into<String>,
        affected_bounds: SdfBounds,
        previous_radius: f32,
        new_radius: f32,
        halo: f32,
    ) -> Self {
        let feature_id = feature_id.into();
        let mut dirty_region = DirtyRegion::new(DirtyRegionSource::Blend, affected_bounds)
            .with_halo(halo.max(previous_radius).max(new_radius));
        dirty_region.feature_ids.push(feature_id.clone());
        Self {
            kind: GeometryEditKind::BlendRadius,
            feature_id: Some(feature_id),
            dirty_region,
            previous_value: Some(previous_radius),
            new_value: Some(new_radius),
        }
    }

    pub fn offset_distance_changed(
        feature_id: impl Into<String>,
        affected_bounds: SdfBounds,
        previous_distance: f32,
        new_distance: f32,
        halo: f32,
    ) -> Self {
        let support_halo = halo
            .max(previous_distance.abs())
            .max(new_distance.abs())
            .max((new_distance - previous_distance).abs());
        Self::parameter_changed(
            GeometryEditKind::Offset,
            DirtyRegionSource::Offset,
            feature_id,
            affected_bounds,
            previous_distance,
            new_distance,
            support_halo,
        )
    }

    pub fn shell_thickness_changed(
        feature_id: impl Into<String>,
        affected_bounds: SdfBounds,
        previous_thickness: f32,
        new_thickness: f32,
        halo: f32,
    ) -> Self {
        let previous_half = previous_thickness.abs() * 0.5;
        let new_half = new_thickness.abs() * 0.5;
        let support_halo = halo
            .max(previous_half)
            .max(new_half)
            .max((new_half - previous_half).abs());
        Self::parameter_changed(
            GeometryEditKind::Shell,
            DirtyRegionSource::Shell,
            feature_id,
            affected_bounds,
            previous_thickness,
            new_thickness,
            support_halo,
        )
    }

    pub fn metadata_changed(
        feature_id: Option<String>,
        previous_metadata: &SdfNodeMetadata,
        next_metadata: &SdfNodeMetadata,
        halo: f32,
    ) -> Option<Self> {
        let affected_bounds = previous_metadata
            .localized_change_bounds(next_metadata)
            .or_else(|| previous_metadata.support_change_bounds(next_metadata))?;
        let mut dirty_region =
            DirtyRegion::new(DirtyRegionSource::FeatureEdit, affected_bounds).with_halo(halo);
        if let Some(spacing) = recommended_grid_spacing_for_metadata_change(
            previous_metadata,
            next_metadata,
            ConditioningPolicy::DEFAULT_ADAPTIVE_SAMPLES_PER_FEATURE,
            ConditioningPolicy::DEFAULT_ADAPTIVE_MIN_GRID_SPACING,
        ) {
            dirty_region = dirty_region.with_recommended_grid_spacing(spacing);
        }
        {
            let mut push_unique = |feature_id: String| {
                if !dirty_region.feature_ids.contains(&feature_id) {
                    dirty_region.feature_ids.push(feature_id);
                }
            };

            if let Some(feature_id) = feature_id.clone() {
                push_unique(feature_id);
            }
            for feature_id in previous_metadata
                .feature_ids
                .iter()
                .chain(next_metadata.feature_ids.iter())
                .cloned()
            {
                push_unique(feature_id);
            }
        }

        Some(Self {
            kind: GeometryEditKind::Feature,
            feature_id,
            dirty_region,
            previous_value: None,
            new_value: None,
        })
    }
}

fn recommended_grid_spacing_for_metadata_change(
    previous_metadata: &SdfNodeMetadata,
    next_metadata: &SdfNodeMetadata,
    samples_per_feature: f32,
    min_spacing: f32,
) -> Option<f32> {
    [
        previous_metadata.min_feature_size,
        next_metadata.min_feature_size,
    ]
    .into_iter()
    .flatten()
    .filter(|size| size.is_finite() && *size > 0.0)
    .reduce(f32::min)
    .map(|size| (size / samples_per_feature.max(1.0)).max(min_spacing.max(1.0e-4)))
}

#[derive(Clone, Debug, PartialEq)]
pub struct GeometryEditTransaction {
    pub edits: Vec<GeometryEdit>,
}

impl GeometryEditTransaction {
    pub fn new(edits: Vec<GeometryEdit>) -> Self {
        assert!(
            !edits.is_empty(),
            "conditioning edit transactions must contain at least one edit"
        );
        Self { edits }
    }

    pub fn single(edit: GeometryEdit) -> Self {
        Self::new(vec![edit])
    }

    pub fn dirty_regions(&self) -> Vec<DirtyRegion> {
        self.edits
            .iter()
            .map(|edit| edit.dirty_region.clone())
            .collect()
    }
}

impl From<GeometryEdit> for GeometryEditTransaction {
    fn from(edit: GeometryEdit) -> Self {
        Self::single(edit)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DirtyRegionSource {
    FullGeometry,
    ParameterEdit,
    FeatureEdit,
    Boolean,
    Blend,
    Shell,
    Offset,
    Transform,
    Unknown,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DirtyRegion {
    pub source: DirtyRegionSource,
    pub feature_ids: Vec<String>,
    pub bounds: SdfBounds,
    pub recommended_grid_spacing: Option<f32>,
    pub halo: f32,
    pub topology_change_expected: bool,
}

impl DirtyRegion {
    pub fn new(source: DirtyRegionSource, bounds: SdfBounds) -> Self {
        Self {
            source,
            feature_ids: Vec::new(),
            bounds,
            recommended_grid_spacing: None,
            halo: 0.0,
            topology_change_expected: false,
        }
    }

    pub fn with_halo(mut self, halo: f32) -> Self {
        self.halo = halo.max(0.0);
        self
    }

    pub fn with_recommended_grid_spacing(mut self, spacing: f32) -> Self {
        if spacing.is_finite() && spacing > 0.0 {
            self.recommended_grid_spacing = Some(spacing);
        }
        self
    }

    pub fn update_bounds(&self) -> SdfBounds {
        self.bounds.expanded(self.halo)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConditionedCacheState {
    Ready,
    Partial,
    Unavailable,
    Rejected,
    Experimental,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConditioningUpdateMode {
    LocalIncremental,
    FullRebuild,
    DirectAnalytic,
    NotRun,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InsideOutside {
    Inside,
    OnSurface,
    Outside,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LocalValidationMode {
    LocalCacheQuality,
    FullComparison,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum AdaptiveRefreshScope {
    FullRebuild,
    LocalEdit,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayIntersection {
    pub point: Vec3,
    pub distance: f32,
    pub normal: Vec3,
    pub iterations: usize,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PrincipalCurvature {
    pub minimum: f32,
    pub maximum: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CrossSectionStats {
    pub axis: usize,
    pub position: f32,
    pub area: f32,
    pub centroid: Vec3,
    pub sample_count: usize,
    pub inside_sample_count: usize,
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct BlockIndex {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl BlockIndex {
    pub fn new(x: i32, y: i32, z: i32) -> Self {
        Self { x, y, z }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ConditionedBlock {
    pub index: BlockIndex,
    pub bounds: SdfBounds,
    pub generation: u64,
    pub state: ConditionedCacheState,
    pub samples: Vec<f32>,
    surface_refinement: Option<SurfaceRefinement>,
    pub min_distance: f32,
    pub max_distance: f32,
    pub reconditioning_anchor_count: usize,
    pub reconditioning_projection_anchor_count: usize,
    pub reconditioning_iteration_count: usize,
    pub max_reconditioning_change: f32,
    pub reconditioning_narrow_band_sample_count: usize,
    pub reconditioning_used_narrow_band: bool,
}

impl ConditionedBlock {
    fn surface_refined_distance(&self, point: Vec3) -> Option<f32> {
        self.surface_refinement
            .as_ref()
            .and_then(|refinement| refinement.interpolated_distance(point))
    }

    fn surface_refined_reinitialized_distance(
        &self,
        point: Vec3,
        sign_hint: Option<f32>,
    ) -> Option<f32> {
        self.surface_refinement.as_ref().and_then(|refinement| {
            refinement.reinitialized_distance_with_sign_hint(point, sign_hint)
        })
    }

    pub fn interpolated_distance(&self, point: Vec3, spacing: f32) -> Option<f32> {
        if spacing <= 0.0 || !self.bounds.contains(point) || self.samples.is_empty() {
            return None;
        }

        if let Some(distance) = self.surface_refined_distance(point) {
            return Some(distance);
        }

        let size = self.bounds.size();
        let nx = (size.x / spacing).ceil().max(0.0) as usize;
        let ny = (size.y / spacing).ceil().max(0.0) as usize;
        let nz = (size.z / spacing).ceil().max(0.0) as usize;
        let expected_sample_count = (nx + 1) * (ny + 1) * (nz + 1);
        if self.samples.len() != expected_sample_count {
            return None;
        }

        let local = (point - self.bounds.min) / spacing;
        let fx = local.x.clamp(0.0, nx as f32);
        let fy = local.y.clamp(0.0, ny as f32);
        let fz = local.z.clamp(0.0, nz as f32);
        let ix0 = fx.floor() as usize;
        let iy0 = fy.floor() as usize;
        let iz0 = fz.floor() as usize;
        let ix1 = (ix0 + 1).min(nx);
        let iy1 = (iy0 + 1).min(ny);
        let iz1 = (iz0 + 1).min(nz);
        let tx = if ix0 == ix1 { 0.0 } else { fx - ix0 as f32 };
        let ty = if iy0 == iy1 { 0.0 } else { fy - iy0 as f32 };
        let tz = if iz0 == iz1 { 0.0 } else { fz - iz0 as f32 };
        let sample = |ix: usize, iy: usize, iz: usize| -> f32 {
            self.samples[((ix * (ny + 1) + iy) * (nz + 1)) + iz]
        };

        let c00 = lerp(sample(ix0, iy0, iz0), sample(ix1, iy0, iz0), tx);
        let c10 = lerp(sample(ix0, iy1, iz0), sample(ix1, iy1, iz0), tx);
        let c01 = lerp(sample(ix0, iy0, iz1), sample(ix1, iy0, iz1), tx);
        let c11 = lerp(sample(ix0, iy1, iz1), sample(ix1, iy1, iz1), tx);
        let c0 = lerp(c00, c10, ty);
        let c1 = lerp(c01, c11, ty);
        Some(lerp(c0, c1, tz))
    }
}

#[derive(Clone, Debug, PartialEq)]
struct SurfaceRefinement {
    bounds: SdfBounds,
    spacing: f32,
    nx: usize,
    ny: usize,
    nz: usize,
    cells: HashMap<usize, [f32; 8]>,
    distance_cells: HashMap<usize, [f32; 8]>,
    surface_points: Option<SurfacePointCloud>,
}

impl SurfaceRefinement {
    fn interpolated_distance(&self, point: Vec3) -> Option<f32> {
        self.interpolated_distance_from_cells(point, &self.cells)
    }

    fn reinitialized_distance(&self, point: Vec3) -> Option<f32> {
        self.interpolated_distance_from_cells(point, &self.distance_cells)
    }

    fn reinitialized_distance_with_sign_hint(
        &self,
        point: Vec3,
        sign_hint: Option<f32>,
    ) -> Option<f32> {
        let distance = self.reinitialized_distance(point)?;
        let Some(sign_hint) = sign_hint else {
            return Some(distance);
        };
        if !sign_hint.is_finite() || sign_hint.abs() <= f32::EPSILON {
            return Some(distance);
        }
        Some(sign_hint.signum() * distance.abs())
    }

    fn interpolated_distance_from_cells(
        &self,
        point: Vec3,
        cells: &HashMap<usize, [f32; 8]>,
    ) -> Option<f32> {
        if self.spacing <= 0.0 || cells.is_empty() || !self.bounds.contains(point) {
            return None;
        }
        let local = (point - self.bounds.min) / self.spacing;
        let ix = local.x.floor() as usize;
        let iy = local.y.floor() as usize;
        let iz = local.z.floor() as usize;
        if ix >= self.nx || iy >= self.ny || iz >= self.nz {
            return None;
        }
        let samples = cells.get(&surface_refinement_cell_index(ix, iy, iz, self.ny, self.nz))?;
        let tx = (local.x - ix as f32).clamp(0.0, 1.0);
        let ty = (local.y - iy as f32).clamp(0.0, 1.0);
        let tz = (local.z - iz as f32).clamp(0.0, 1.0);

        let c00 = lerp(samples[0], samples[1], tx);
        let c10 = lerp(samples[2], samples[3], tx);
        let c01 = lerp(samples[4], samples[5], tx);
        let c11 = lerp(samples[6], samples[7], tx);
        let c0 = lerp(c00, c10, ty);
        let c1 = lerp(c01, c11, ty);
        Some(lerp(c0, c1, tz))
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ConditioningUpdateSummary {
    pub generation: u64,
    pub update_mode: ConditioningUpdateMode,
    pub cache_state: ConditionedCacheState,
    pub dirty_regions: Vec<DirtyRegion>,
    pub invalidated_block_count: usize,
    pub regenerated_block_count: usize,
    pub full_rebuild_used: bool,
    pub reconditioning: ReconditioningStats,
    pub adaptive: AdaptiveConditioningStats,
    pub diagnostics: ConditioningDiagnostics,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ReconditioningStats {
    pub block_count: usize,
    pub reconditioned_block_count: usize,
    pub narrow_band_block_count: usize,
    pub narrow_band_sample_count: usize,
    pub anchor_count: usize,
    pub projection_anchor_count: usize,
    pub max_iteration_count: usize,
    pub max_change: f32,
}

impl ReconditioningStats {
    fn from_blocks(blocks: &[ConditionedBlock]) -> Self {
        let mut stats = Self {
            block_count: blocks.len(),
            ..Self::default()
        };
        for block in blocks {
            stats.anchor_count += block.reconditioning_anchor_count;
            stats.projection_anchor_count += block.reconditioning_projection_anchor_count;
            stats.max_iteration_count = stats
                .max_iteration_count
                .max(block.reconditioning_iteration_count);
            stats.max_change = stats.max_change.max(block.max_reconditioning_change);
            if block.reconditioning_iteration_count > 0 {
                stats.reconditioned_block_count += 1;
            }
            if block.reconditioning_used_narrow_band {
                stats.narrow_band_block_count += 1;
                stats.narrow_band_sample_count += block.reconditioning_narrow_band_sample_count;
            }
        }
        stats
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct AdaptiveConditioningStats {
    pub region_count: usize,
    pub sample_count: usize,
    pub narrow_band_region_count: usize,
    pub narrow_band_sample_count: usize,
    pub anchor_count: usize,
    pub projection_anchor_count: usize,
    pub max_iteration_count: usize,
    pub min_grid_spacing: Option<f32>,
    pub max_grid_spacing: Option<f32>,
    pub feature_id_count: usize,
    pub requested_feature_id_count: usize,
    pub requested_region_count: usize,
    pub skipped_region_count: usize,
    pub oversized_region_count: usize,
    pub requested_min_grid_spacing: Option<f32>,
}

impl AdaptiveConditioningStats {
    fn from_regions(regions: &[AdaptiveConditionedRegion]) -> Self {
        let mut stats = Self::default();
        let mut feature_ids = HashSet::new();
        stats.region_count = regions.len();
        for region in regions {
            stats.sample_count += region.region.report.sample_count;
            stats.anchor_count += region.region.report.anchor_count;
            stats.projection_anchor_count += region.region.report.projection_anchor_count;
            stats.max_iteration_count = stats
                .max_iteration_count
                .max(region.region.report.iteration_count);
            if region.region.report.used_narrow_band {
                stats.narrow_band_region_count += 1;
                stats.narrow_band_sample_count += region.region.report.narrow_band_sample_count;
            }
            stats.min_grid_spacing = Some(
                stats
                    .min_grid_spacing
                    .map_or(region.spacing, |spacing| spacing.min(region.spacing)),
            );
            stats.max_grid_spacing = Some(
                stats
                    .max_grid_spacing
                    .map_or(region.spacing, |spacing| spacing.max(region.spacing)),
            );
            feature_ids.extend(region.feature_ids.iter().cloned());
        }
        stats.feature_id_count = feature_ids.len();
        stats
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ConditionedStorageStats {
    pub block_count: usize,
    pub block_sample_count: usize,
    pub block_sample_bytes: usize,
    pub surface_refinement_block_count: usize,
    pub surface_refinement_cell_count: usize,
    pub surface_refinement_sample_bytes: usize,
    pub surface_refinement_distance_cell_count: usize,
    pub surface_refinement_distance_sample_bytes: usize,
    pub surface_refinement_point_count: usize,
    pub surface_refinement_point_bytes: usize,
    pub adaptive_region_count: usize,
    pub adaptive_logical_sample_count: usize,
    pub adaptive_stored_sample_count: usize,
    pub adaptive_stored_sample_bytes: usize,
    pub adaptive_dense_region_count: usize,
    pub adaptive_sparse_region_count: usize,
    pub adaptive_dense_value_count: usize,
    pub adaptive_sparse_value_count: usize,
    pub total_stored_sample_count: usize,
    pub total_stored_sample_bytes: usize,
    pub estimated_metadata_bytes: usize,
    pub estimated_total_bytes: usize,
}

#[derive(Clone, Debug, PartialEq)]
struct AdaptiveConditionedRegion {
    source: DirtyRegionSource,
    feature_ids: Vec<String>,
    bounds: SdfBounds,
    spacing: f32,
    generation: u64,
    min_feature_size: Option<f32>,
    region: ReconditionedSampleRegion,
}

#[derive(Clone, Debug, PartialEq)]
struct AdaptiveRegionRequest {
    source: DirtyRegionSource,
    feature_ids: Vec<String>,
    update_bounds: SdfBounds,
    requested_spacing: f32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum SurfaceRefinementMode {
    SurfaceOnly,
    SurfaceAndDistanceBand,
}

impl AdaptiveConditionedRegion {
    fn sample_conditioned(&self, point: Vec3) -> Option<f32> {
        if !self.bounds.contains(point) {
            return None;
        }
        self.region.sample_conditioned(point, self.spacing)
    }
}

impl ConditionedStorageStats {
    fn from_model(model: &ConditionedGeometryModel) -> Self {
        let block_sample_count: usize =
            model.blocks.values().map(|block| block.samples.len()).sum();
        let surface_refinement_block_count = model
            .blocks
            .values()
            .filter(|block| block.surface_refinement.is_some())
            .count();
        let surface_refinement_cell_count: usize = model
            .blocks
            .values()
            .filter_map(|block| block.surface_refinement.as_ref())
            .map(|refinement| refinement.cells.len())
            .sum();
        let surface_refinement_distance_cell_count: usize = model
            .blocks
            .values()
            .filter_map(|block| block.surface_refinement.as_ref())
            .map(|refinement| refinement.distance_cells.len())
            .sum();
        let surface_refinement_point_count: usize = model
            .blocks
            .values()
            .filter_map(|block| block.surface_refinement.as_ref())
            .filter_map(|refinement| refinement.surface_points.as_ref())
            .map(SurfacePointCloud::point_count)
            .sum();
        let mut stats = Self {
            block_count: model.blocks.len(),
            block_sample_count,
            block_sample_bytes: block_sample_count * std::mem::size_of::<f32>(),
            surface_refinement_block_count,
            surface_refinement_cell_count,
            surface_refinement_sample_bytes: surface_refinement_cell_count
                * 8
                * std::mem::size_of::<f32>(),
            surface_refinement_distance_cell_count,
            surface_refinement_distance_sample_bytes: surface_refinement_distance_cell_count
                * 8
                * std::mem::size_of::<f32>(),
            surface_refinement_point_count,
            surface_refinement_point_bytes: surface_refinement_point_count
                * 3
                * std::mem::size_of::<f32>(),
            adaptive_region_count: model.adaptive_regions.len(),
            ..Self::default()
        };

        for region in &model.adaptive_regions {
            stats.adaptive_logical_sample_count += region.region.report.sample_count;
            let raw_count = region.region.raw_samples.stored_value_count();
            let conditioned_count = region.region.samples.stored_value_count();
            let stored_count = raw_count + conditioned_count;
            stats.adaptive_stored_sample_count += stored_count;

            let raw_is_sparse = region.region.raw_samples.is_sparse();
            let conditioned_is_sparse = region.region.samples.is_sparse();
            if raw_is_sparse || conditioned_is_sparse {
                stats.adaptive_sparse_region_count += 1;
            } else {
                stats.adaptive_dense_region_count += 1;
            }

            if raw_is_sparse {
                stats.adaptive_sparse_value_count += raw_count;
            } else {
                stats.adaptive_dense_value_count += raw_count;
            }
            if conditioned_is_sparse {
                stats.adaptive_sparse_value_count += conditioned_count;
            } else {
                stats.adaptive_dense_value_count += conditioned_count;
            }
        }

        stats.adaptive_stored_sample_bytes =
            stats.adaptive_stored_sample_count * std::mem::size_of::<f32>();
        stats.total_stored_sample_count =
            stats.block_sample_count + stats.adaptive_stored_sample_count;
        stats.total_stored_sample_bytes = stats.block_sample_bytes
            + stats.surface_refinement_sample_bytes
            + stats.surface_refinement_distance_sample_bytes
            + stats.surface_refinement_point_bytes
            + stats.adaptive_stored_sample_bytes;

        let block_metadata_bytes = model.blocks.len() * std::mem::size_of::<ConditionedBlock>();
        let adaptive_metadata_bytes =
            model.adaptive_regions.len() * std::mem::size_of::<AdaptiveConditionedRegion>();
        let surface_refinement_entry_overhead =
            stats.surface_refinement_cell_count * std::mem::size_of::<usize>();
        let surface_refinement_distance_entry_overhead =
            stats.surface_refinement_distance_cell_count * std::mem::size_of::<usize>();
        let sparse_entry_overhead =
            stats.adaptive_sparse_value_count * (std::mem::size_of::<usize>() * 2);
        stats.estimated_metadata_bytes = block_metadata_bytes
            + adaptive_metadata_bytes
            + surface_refinement_entry_overhead
            + surface_refinement_distance_entry_overhead
            + sparse_entry_overhead;
        stats.estimated_total_bytes =
            stats.total_stored_sample_bytes + stats.estimated_metadata_bytes;
        stats
    }
}

fn prioritize_adaptive_region_requests(
    requests: Vec<AdaptiveRegionRequest>,
) -> Vec<AdaptiveRegionRequest> {
    let mut seen_feature_keys = HashSet::new();
    let mut first_per_feature = Vec::new();
    let mut remaining = Vec::new();

    for (index, request) in requests.into_iter().enumerate() {
        let feature_key = request
            .feature_ids
            .first()
            .cloned()
            .unwrap_or_else(|| format!("__unowned_request_{index}"));
        if seen_feature_keys.insert(feature_key) {
            first_per_feature.push(request);
        } else {
            remaining.push(request);
        }
    }

    first_per_feature.extend(remaining);
    first_per_feature
}

fn uncovered_adaptive_feature_id_count(
    requests: &[AdaptiveRegionRequest],
    active_feature_ids: &HashSet<String>,
    current_feature_ids: &HashSet<String>,
) -> usize {
    let mut remaining = HashSet::new();
    for request in requests {
        for feature_id in &request.feature_ids {
            if !active_feature_ids.contains(feature_id) && !current_feature_ids.contains(feature_id)
            {
                remaining.insert(feature_id.clone());
            }
        }
    }
    remaining.len()
}

fn initial_adaptive_region_order(a: &DirtyRegion, b: &DirtyRegion) -> std::cmp::Ordering {
    a.recommended_grid_spacing
        .unwrap_or(f32::INFINITY)
        .partial_cmp(&b.recommended_grid_spacing.unwrap_or(f32::INFINITY))
        .unwrap_or(std::cmp::Ordering::Equal)
        .then_with(|| {
            bounds_volume(&a.bounds)
                .partial_cmp(&bounds_volume(&b.bounds))
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .then_with(|| format!("{:?}", a.source).cmp(&format!("{:?}", b.source)))
}

fn bounds_volume(bounds: &SdfBounds) -> f32 {
    let size = bounds.size();
    size.x.max(0.0) * size.y.max(0.0) * size.z.max(0.0)
}

#[derive(Clone)]
pub struct ConditionedGeometryModel {
    domain: SdfBounds,
    policy: ConditioningPolicy,
    block_size: f32,
    generation: u64,
    blocks: HashMap<BlockIndex, ConditionedBlock>,
    adaptive_regions: Vec<AdaptiveConditionedRegion>,
    adaptive_requested_region_count: usize,
    adaptive_skipped_region_count: usize,
    adaptive_oversized_region_count: usize,
    adaptive_requested_min_grid_spacing: Option<f32>,
    adaptive_requested_feature_ids: HashSet<String>,
    dirty_history: Vec<DirtyRegion>,
    last_diagnostics: Option<ConditioningDiagnostics>,
    last_rejected_diagnostics: Option<ConditioningDiagnostics>,
}

#[derive(Clone)]
pub struct ConditionedGeometryKernel {
    canonical: Arc<dyn Sdf>,
    conditioning: ConditionedGeometryModel,
}

impl ConditionedGeometryKernel {
    pub const DEFAULT_RUNTIME_MAX_INITIAL_SAMPLES: usize = 350_000;

    pub fn new(
        canonical: Arc<dyn Sdf>,
        domain: SdfBounds,
        block_size: f32,
        policy: ConditioningPolicy,
    ) -> Self {
        let mut conditioning = ConditionedGeometryModel::new(domain, block_size, policy);
        conditioning.initialize_from_canonical(canonical.as_ref());
        Self {
            canonical,
            conditioning,
        }
    }

    pub fn from_conditionable_metadata(
        canonical: Arc<dyn Sdf>,
        block_size: f32,
        policy: ConditioningPolicy,
        domain_halo: f32,
    ) -> Option<Self> {
        let metadata = canonical.metadata();
        if !metadata.is_conditionable() {
            return None;
        }
        let domain = metadata.support_bounds?.expanded(
            domain_halo
                .max(policy.interface_band)
                .max(policy.grid_spacing),
        );
        Some(Self::new(canonical, domain, block_size, policy))
    }

    pub fn from_runtime_metadata(canonical: Arc<dyn Sdf>) -> Option<Self> {
        Self::from_runtime_metadata_with_sample_budget(
            canonical,
            Self::DEFAULT_RUNTIME_MAX_INITIAL_SAMPLES,
        )
    }

    pub fn from_runtime_metadata_with_sample_budget(
        canonical: Arc<dyn Sdf>,
        max_initial_samples: usize,
    ) -> Option<Self> {
        let metadata = canonical.metadata();
        if !metadata.is_conditionable() {
            return None;
        }
        let support_bounds = metadata.support_bounds.as_ref()?;
        let policy = runtime_conditioning_policy_for_bounds(support_bounds, max_initial_samples);
        let block_size = (policy.grid_spacing * 16.0).max(policy.grid_spacing);
        let domain_halo = runtime_conditioning_domain_halo(&policy);
        Self::from_conditionable_metadata(canonical, block_size, policy, domain_halo)
    }

    pub fn canonical(&self) -> &Arc<dyn Sdf> {
        &self.canonical
    }

    pub fn conditioning(&self) -> &ConditionedGeometryModel {
        &self.conditioning
    }

    pub fn storage_stats(&self) -> ConditionedStorageStats {
        self.conditioning.storage_stats()
    }

    pub fn surface_refinement_bounds(&self) -> Vec<SdfBounds> {
        self.conditioning.surface_refinement_bounds()
    }

    pub fn raw_surface_refined_reinitialized_distance(&self, point: Vec3) -> Option<f32> {
        let distance = self
            .conditioning
            .surface_refined_reinitialized_distance(point)?;
        let canonical = self.canonical_distance(point);
        if !canonical.is_finite() || canonical.abs() <= f32::EPSILON {
            return Some(distance);
        }
        Some(canonical.signum() * distance.abs())
    }

    pub fn conditioning_mut(&mut self) -> &mut ConditionedGeometryModel {
        &mut self.conditioning
    }

    pub fn advance_to_canonical_after_metadata_change(
        &self,
        next_canonical: Arc<dyn Sdf>,
    ) -> Option<Self> {
        let next_metadata = next_canonical.metadata();
        if !next_metadata.is_conditionable() {
            return None;
        }

        let previous_metadata = self.canonical.metadata();
        let halo = runtime_conditioning_domain_halo(&self.conditioning.policy);
        let Some(edit) = GeometryEdit::metadata_changed(
            Some("canonical_graph".to_string()),
            &previous_metadata,
            &next_metadata,
            halo,
        ) else {
            if !Arc::ptr_eq(&self.canonical, &next_canonical) {
                return Self::from_runtime_metadata(next_canonical);
            }
            let mut updated = self.clone();
            updated.canonical = next_canonical;
            return Some(updated);
        };

        if !self
            .conditioning
            .domain()
            .contains_bounds(&edit.dirty_region.update_bounds())
        {
            return Self::from_runtime_metadata(next_canonical);
        }

        let mut updated = self.clone();
        updated.replace_canonical_with_edit(next_canonical, edit);
        Some(updated)
    }

    pub fn canonical_distance(&self, point: Vec3) -> f32 {
        self.canonical.distance(point)
    }

    pub fn distance(&self, point: Vec3) -> f32 {
        self.conditioned_distance(point)
            .unwrap_or_else(|| self.canonical_distance(point))
    }

    pub fn raw_conditioned_distance(&self, point: Vec3) -> Option<f32> {
        self.conditioning.conditioned_distance(point)
    }

    pub fn raw_surface_refined_distance(&self, point: Vec3) -> Option<f32> {
        self.conditioning.surface_refined_distance(point)
    }

    pub fn conditioned_distance(&self, point: Vec3) -> Option<f32> {
        let conditioned = self.raw_conditioned_distance(point)?;
        let canonical = self.canonical_distance(point);
        Some(published_fidelity_preserving_conditioned_sample(
            canonical,
            conditioned,
            &self.conditioning.policy,
        ))
    }

    pub fn gradient(&self, point: Vec3, step: f32) -> Vec3 {
        if let Some(gradient) = self.reinitialized_band_gradient(point, step) {
            if gradient.length_squared() > f32::EPSILON {
                return gradient;
            }
        }
        let h = step.max(1.0e-4);
        Vec3::new(
            self.distance(point + Vec3::X * h) - self.distance(point - Vec3::X * h),
            self.distance(point + Vec3::Y * h) - self.distance(point - Vec3::Y * h),
            self.distance(point + Vec3::Z * h) - self.distance(point - Vec3::Z * h),
        ) / (2.0 * h)
    }

    fn reinitialized_band_gradient(&self, point: Vec3, step: f32) -> Option<Vec3> {
        let h = step
            .max(1.0e-4)
            .min(SURFACE_REFINEMENT_DISTANCE_TARGET_SPACING.max(1.0e-4));
        let dx = self.raw_surface_refined_reinitialized_distance(point + Vec3::X * h)?
            - self.raw_surface_refined_reinitialized_distance(point - Vec3::X * h)?;
        let dy = self.raw_surface_refined_reinitialized_distance(point + Vec3::Y * h)?
            - self.raw_surface_refined_reinitialized_distance(point - Vec3::Y * h)?;
        let dz = self.raw_surface_refined_reinitialized_distance(point + Vec3::Z * h)?
            - self.raw_surface_refined_reinitialized_distance(point - Vec3::Z * h)?;
        let gradient = Vec3::new(dx, dy, dz) / (2.0 * h);
        gradient.is_finite().then_some(gradient)
    }

    pub fn surface_normal(&self, point: Vec3, step: f32) -> Vec3 {
        self.gradient(point, step).normalize_or_zero()
    }

    pub fn inside_outside(&self, point: Vec3, tolerance: f32) -> InsideOutside {
        let distance = self.distance(point);
        let tolerance = tolerance.max(0.0);
        if distance < -tolerance {
            InsideOutside::Inside
        } else if distance > tolerance {
            InsideOutside::Outside
        } else {
            InsideOutside::OnSurface
        }
    }

    pub fn closest_surface_point(
        &self,
        query: Vec3,
        tolerance: f32,
        max_iterations: usize,
    ) -> Option<Vec3> {
        self.project_to_surface(query, tolerance, max_iterations)
    }

    pub fn project_to_surface(
        &self,
        query: Vec3,
        tolerance: f32,
        max_iterations: usize,
    ) -> Option<Vec3> {
        let original_query = query;
        let mut point = query;
        let tolerance = tolerance.max(1.0e-6);
        for _ in 0..max_iterations.max(1) {
            let distance = self
                .raw_surface_refined_reinitialized_distance(point)
                .unwrap_or_else(|| self.distance(point));
            if distance.abs() <= tolerance {
                return Some(point);
            }
            let normal =
                self.surface_normal(point, self.conditioning.effective_grid_spacing(point));
            if normal.length_squared() <= f32::EPSILON {
                return None;
            }
            point -= normal * distance;
        }

        if self.distance(point).abs() <= tolerance {
            Some(point)
        } else {
            self.project_to_canonical_surface(original_query, tolerance, max_iterations)
        }
    }

    fn project_to_canonical_surface(
        &self,
        query: Vec3,
        tolerance: f32,
        max_iterations: usize,
    ) -> Option<Vec3> {
        let mut point = query;
        for _ in 0..max_iterations.max(1) {
            let distance = self.canonical_distance(point);
            if distance.abs() <= tolerance {
                return Some(point);
            }
            let normal = finite_difference_gradient(
                self.canonical.as_ref(),
                point,
                self.conditioning.policy.grid_spacing,
            )
            .normalize_or_zero();
            if normal.length_squared() <= f32::EPSILON {
                return None;
            }
            point -= normal * distance;
        }

        if self.canonical_distance(point).abs() <= tolerance {
            Some(point)
        } else {
            None
        }
    }

    pub fn ray_intersection(
        &self,
        origin: Vec3,
        direction: Vec3,
        max_distance: f32,
        tolerance: f32,
        max_steps: usize,
    ) -> Option<RayIntersection> {
        let dir = direction.normalize_or_zero();
        if dir.length_squared() <= f32::EPSILON || max_distance <= 0.0 {
            return None;
        }

        let tolerance = tolerance.max(1.0e-6);
        let mut t = 0.0_f32;
        let mut previous_d = self.distance(origin);
        if previous_d.abs() <= tolerance {
            let point = origin;
            return Some(RayIntersection {
                point,
                distance: 0.0,
                normal: self.surface_normal(point, self.conditioning.effective_grid_spacing(point)),
                iterations: 0,
            });
        }

        for step_index in 0..max_steps.max(1) {
            let step = previous_d.abs().max(tolerance).min(max_distance - t);
            if step <= 0.0 {
                break;
            }
            let previous_t = t;
            t += step;
            if t > max_distance {
                break;
            }

            let point = origin + dir * t;
            let distance = self.distance(point);
            if distance.abs() <= tolerance {
                return Some(RayIntersection {
                    point,
                    distance: t,
                    normal: self
                        .surface_normal(point, self.conditioning.effective_grid_spacing(point)),
                    iterations: step_index + 1,
                });
            }

            if sign_with_tolerance(previous_d, tolerance)
                != sign_with_tolerance(distance, tolerance)
            {
                let hit_t =
                    self.refine_ray_interval(origin, dir, previous_t, t, previous_d, tolerance);
                let hit_point = origin + dir * hit_t;
                return Some(RayIntersection {
                    point: hit_point,
                    distance: hit_t,
                    normal: self.surface_normal(
                        hit_point,
                        self.conditioning.effective_grid_spacing(hit_point),
                    ),
                    iterations: step_index + 1,
                });
            }

            previous_d = distance;
        }

        None
    }

    pub fn local_feature_size(&self, point: Vec3) -> Option<f32> {
        let metadata = self.canonical.metadata();
        let mut best = metadata.min_feature_size;
        for region in &metadata.feature_regions {
            if region.bounds.contains(point) {
                best = Some(best.map_or(region.min_feature_size, |existing| {
                    existing.min(region.min_feature_size)
                }));
            }
        }
        best.or_else(|| {
            Some(
                self.conditioning
                    .effective_grid_spacing(point)
                    .max(self.conditioning.policy.grid_spacing)
                    * 2.0,
            )
        })
    }

    pub fn wall_thickness(
        &self,
        point: Vec3,
        direction: Vec3,
        max_distance: f32,
        tolerance: f32,
    ) -> Option<f32> {
        let dir = direction.normalize_or_zero();
        if dir.length_squared() <= f32::EPSILON || max_distance <= 0.0 {
            return None;
        }
        let tolerance = tolerance.max(self.conditioning.effective_grid_spacing(point));
        let offset = dir * tolerance * 1.5;
        let forward = self.ray_intersection(point + offset, dir, max_distance, tolerance, 128);
        let backward = self.ray_intersection(point - offset, -dir, max_distance, tolerance, 128);
        match (forward, backward) {
            (Some(a), Some(b)) => Some(a.distance + b.distance + tolerance * 3.0),
            (Some(hit), None) | (None, Some(hit)) => Some(hit.distance + tolerance * 1.5),
            (None, None) => None,
        }
    }

    pub fn curvature(&self, point: Vec3, step: f32) -> f32 {
        let h = step
            .max(self.conditioning.effective_grid_spacing(point))
            .max(1.0e-4);
        let nx1 = self.surface_normal(point + Vec3::X * h, h);
        let nx0 = self.surface_normal(point - Vec3::X * h, h);
        let ny1 = self.surface_normal(point + Vec3::Y * h, h);
        let ny0 = self.surface_normal(point - Vec3::Y * h, h);
        let nz1 = self.surface_normal(point + Vec3::Z * h, h);
        let nz0 = self.surface_normal(point - Vec3::Z * h, h);
        ((nx1.x - nx0.x) + (ny1.y - ny0.y) + (nz1.z - nz0.z)) / (2.0 * h)
    }

    pub fn principal_curvature(&self, point: Vec3, step: f32) -> PrincipalCurvature {
        let h = step
            .max(self.conditioning.effective_grid_spacing(point))
            .max(1.0e-4);
        let normal = self.surface_normal(point, h);
        if normal.length_squared() <= f32::EPSILON {
            return PrincipalCurvature {
                minimum: 0.0,
                maximum: 0.0,
            };
        }
        let reference = if normal.abs().dot(Vec3::Y) < 0.9 {
            Vec3::Y
        } else {
            Vec3::X
        };
        let tangent_a = normal.cross(reference).normalize_or_zero();
        let tangent_b = normal.cross(tangent_a).normalize_or_zero();
        let directional_curvature = |tangent: Vec3| -> f32 {
            if tangent.length_squared() <= f32::EPSILON {
                return 0.0;
            }
            let n1 = self.surface_normal(point + tangent * h, h);
            let n0 = self.surface_normal(point - tangent * h, h);
            ((n1 - n0).dot(tangent) / (2.0 * h)).abs()
        };
        let ka = directional_curvature(tangent_a);
        let kb = directional_curvature(tangent_b);
        PrincipalCurvature {
            minimum: ka.min(kb),
            maximum: ka.max(kb),
        }
    }

    pub fn cross_section(
        &self,
        axis: usize,
        position: f32,
        resolution: usize,
    ) -> CrossSectionStats {
        let axis = axis.min(2);
        let resolution = resolution.clamp(4, 512);
        let bounds = self.query_bounds();
        let (u_min, u_max, v_min, v_max) = match axis {
            0 => (bounds.min.y, bounds.max.y, bounds.min.z, bounds.max.z),
            1 => (bounds.min.x, bounds.max.x, bounds.min.z, bounds.max.z),
            _ => (bounds.min.x, bounds.max.x, bounds.min.y, bounds.max.y),
        };
        let du = (u_max - u_min) / resolution as f32;
        let dv = (v_max - v_min) / resolution as f32;
        let mut centroid_sum = Vec3::ZERO;
        let mut inside_sample_count = 0_usize;
        for iu in 0..resolution {
            for iv in 0..resolution {
                let u = u_min + (iu as f32 + 0.5) * du;
                let v = v_min + (iv as f32 + 0.5) * dv;
                let point = match axis {
                    0 => Vec3::new(position, u, v),
                    1 => Vec3::new(u, position, v),
                    _ => Vec3::new(u, v, position),
                };
                if self.distance(point) < 0.0 {
                    centroid_sum += point;
                    inside_sample_count += 1;
                }
            }
        }
        let area = inside_sample_count as f32 * du.abs() * dv.abs();
        CrossSectionStats {
            axis,
            position,
            area,
            centroid: if inside_sample_count > 0 {
                centroid_sum / inside_sample_count as f32
            } else {
                match axis {
                    0 => Vec3::new(position, (u_min + u_max) * 0.5, (v_min + v_max) * 0.5),
                    1 => Vec3::new((u_min + u_max) * 0.5, position, (v_min + v_max) * 0.5),
                    _ => Vec3::new((u_min + u_max) * 0.5, (v_min + v_max) * 0.5, position),
                }
            },
            sample_count: resolution * resolution,
            inside_sample_count,
        }
    }

    pub fn volume(&self, resolution: usize) -> f32 {
        self.sample_volume_and_area(resolution).0
    }

    pub fn wetted_area(&self, resolution: usize) -> f32 {
        self.sample_volume_and_area(resolution).1
    }

    pub fn frontal_area(&self, direction: Vec3, resolution: usize) -> f32 {
        let dir = direction.normalize_or_zero();
        if dir.length_squared() <= f32::EPSILON {
            return 0.0;
        }
        let axis = dominant_axis(dir);
        let resolution = resolution.clamp(4, 512);
        let bounds = self.query_bounds();
        let (u_min, u_max, v_min, v_max, max_distance) = match axis {
            0 => (
                bounds.min.y,
                bounds.max.y,
                bounds.min.z,
                bounds.max.z,
                bounds.size().x,
            ),
            1 => (
                bounds.min.x,
                bounds.max.x,
                bounds.min.z,
                bounds.max.z,
                bounds.size().y,
            ),
            _ => (
                bounds.min.x,
                bounds.max.x,
                bounds.min.y,
                bounds.max.y,
                bounds.size().z,
            ),
        };
        let du = (u_max - u_min) / resolution as f32;
        let dv = (v_max - v_min) / resolution as f32;
        let start_axis = if dir[axis] >= 0.0 {
            axis_component(bounds.min, axis) - self.conditioning.policy.interface_band
        } else {
            axis_component(bounds.max, axis) + self.conditioning.policy.interface_band
        };
        let mut hits = 0_usize;
        for iu in 0..resolution {
            for iv in 0..resolution {
                let u = u_min + (iu as f32 + 0.5) * du;
                let v = v_min + (iv as f32 + 0.5) * dv;
                let origin = match axis {
                    0 => Vec3::new(start_axis, u, v),
                    1 => Vec3::new(u, start_axis, v),
                    _ => Vec3::new(u, v, start_axis),
                };
                if self
                    .ray_intersection(
                        origin,
                        dir,
                        max_distance + self.conditioning.policy.interface_band * 2.0,
                        self.conditioning.policy.grid_spacing,
                        128,
                    )
                    .is_some()
                {
                    hits += 1;
                }
            }
        }
        hits as f32 * du.abs() * dv.abs()
    }

    fn query_bounds(&self) -> SdfBounds {
        self.canonical
            .metadata()
            .support_bounds
            .filter(SdfBounds::is_valid)
            .unwrap_or_else(|| self.conditioning.domain().clone())
    }

    fn sample_volume_and_area(&self, resolution: usize) -> (f32, f32) {
        let resolution = resolution.clamp(4, 160);
        let bounds = self.query_bounds();
        let size = bounds.size();
        if !size.is_finite() || size.min_element() <= 0.0 {
            return (0.0, 0.0);
        }
        let dx = size.x / resolution as f32;
        let dy = size.y / resolution as f32;
        let dz = size.z / resolution as f32;
        let sample_count = resolution * resolution * resolution;
        let inside: Vec<bool> = (0..sample_count)
            .into_par_iter()
            .map(|idx| {
                let ix = idx % resolution;
                let iy = (idx / resolution) % resolution;
                let iz = idx / (resolution * resolution);
                let point = bounds.min
                    + Vec3::new(
                        (ix as f32 + 0.5) * dx,
                        (iy as f32 + 0.5) * dy,
                        (iz as f32 + 0.5) * dz,
                    );
                self.distance(point) < 0.0
            })
            .collect();
        let index =
            |ix: usize, iy: usize, iz: usize| -> usize { (iz * resolution + iy) * resolution + ix };
        let mut inside_count = 0_usize;
        let mut area = 0.0_f32;
        for iz in 0..resolution {
            for iy in 0..resolution {
                for ix in 0..resolution {
                    if !inside[index(ix, iy, iz)] {
                        continue;
                    }
                    inside_count += 1;
                    if ix + 1 == resolution || !inside[index(ix + 1, iy, iz)] {
                        area += dy * dz;
                    }
                    if ix == 0 || !inside[index(ix - 1, iy, iz)] {
                        area += dy * dz;
                    }
                    if iy + 1 == resolution || !inside[index(ix, iy + 1, iz)] {
                        area += dx * dz;
                    }
                    if iy == 0 || !inside[index(ix, iy - 1, iz)] {
                        area += dx * dz;
                    }
                    if iz + 1 == resolution || !inside[index(ix, iy, iz + 1)] {
                        area += dx * dy;
                    }
                    if iz == 0 || !inside[index(ix, iy, iz - 1)] {
                        area += dx * dy;
                    }
                }
            }
        }
        (inside_count as f32 * dx * dy * dz, area)
    }

    fn refine_ray_interval(
        &self,
        origin: Vec3,
        direction: Vec3,
        mut lo: f32,
        mut hi: f32,
        lo_distance: f32,
        tolerance: f32,
    ) -> f32 {
        let lo_sign = sign_with_tolerance(lo_distance, tolerance);
        for _ in 0..32 {
            let mid = (lo + hi) * 0.5;
            let mid_distance = self.distance(origin + direction * mid);
            if mid_distance.abs() <= tolerance {
                return mid;
            }
            if sign_with_tolerance(mid_distance, tolerance) == lo_sign {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        (lo + hi) * 0.5
    }

    pub fn rebuild_conditioned_cache(&mut self) -> ConditioningUpdateSummary {
        self.conditioning
            .initialize_from_canonical(self.canonical.as_ref())
    }

    pub fn replace_canonical_with_edit(
        &mut self,
        next_canonical: Arc<dyn Sdf>,
        edit: GeometryEdit,
    ) -> ConditioningUpdateSummary {
        self.replace_canonical_with_edits(next_canonical, GeometryEditTransaction::single(edit))
    }

    pub fn replace_canonical_with_edits(
        &mut self,
        next_canonical: Arc<dyn Sdf>,
        transaction: GeometryEditTransaction,
    ) -> ConditioningUpdateSummary {
        let summary = self.conditioning.apply_edits_and_recondition(
            self.canonical.as_ref(),
            next_canonical.as_ref(),
            transaction,
        );
        self.canonical = next_canonical;
        summary
    }
}

impl Sdf for ConditionedGeometryKernel {
    fn distance(&self, point: Vec3) -> f32 {
        self.distance(point)
    }

    fn metadata(&self) -> SdfNodeMetadata {
        let mut metadata = self.canonical.metadata();
        let blocks: Vec<ConditionedBlock> = self.conditioning.blocks.values().cloned().collect();
        let reconditioning = ReconditioningStats::from_blocks(&blocks);
        let adaptive = self.conditioning.adaptive_stats();
        let storage = self.conditioning.storage_stats();
        let (state, confidence, publication_confidence, quality_confidence) = self
            .conditioning
            .last_diagnostics()
            .map(|diagnostics| {
                (
                    diagnostics.cache_state,
                    diagnostics.confidence,
                    diagnostics.publication_confidence,
                    diagnostics.quality_confidence,
                )
            })
            .unwrap_or((ConditionedCacheState::Unavailable, 0.0, 0.0, 0.0));
        metadata.conditioned_cache = Some(ConditionedCacheMetadata {
            state,
            generation: self.conditioning.generation(),
            block_count: self.conditioning.block_count(),
            grid_spacing: self.conditioning.policy.grid_spacing,
            interface_band: self.conditioning.policy.interface_band,
            confidence,
            publication_confidence,
            quality_confidence,
            anchor_count: reconditioning.anchor_count,
            projection_anchor_count: reconditioning.projection_anchor_count,
            narrow_band_block_count: reconditioning.narrow_band_block_count,
            max_iteration_count: reconditioning.max_iteration_count,
            adaptive_region_count: adaptive.region_count,
            adaptive_sample_count: adaptive.sample_count,
            adaptive_narrow_band_region_count: adaptive.narrow_band_region_count,
            adaptive_narrow_band_sample_count: adaptive.narrow_band_sample_count,
            adaptive_min_grid_spacing: adaptive.min_grid_spacing,
            adaptive_max_grid_spacing: adaptive.max_grid_spacing,
            adaptive_feature_id_count: adaptive.feature_id_count,
            adaptive_requested_feature_id_count: adaptive.requested_feature_id_count,
            adaptive_feature_coverage: self.conditioning.adaptive_feature_coverage(),
            adaptive_requested_region_count: adaptive.requested_region_count,
            adaptive_skipped_region_count: adaptive.skipped_region_count,
            adaptive_oversized_region_count: adaptive.oversized_region_count,
            adaptive_requested_min_grid_spacing: adaptive.requested_min_grid_spacing,
            block_sample_count: storage.block_sample_count,
            block_sample_bytes: storage.block_sample_bytes,
            surface_refinement_block_count: storage.surface_refinement_block_count,
            surface_refinement_cell_count: storage.surface_refinement_cell_count,
            surface_refinement_sample_bytes: storage.surface_refinement_sample_bytes,
            surface_refinement_distance_cell_count: storage.surface_refinement_distance_cell_count,
            surface_refinement_distance_sample_bytes: storage
                .surface_refinement_distance_sample_bytes,
            surface_refinement_point_count: storage.surface_refinement_point_count,
            surface_refinement_point_bytes: storage.surface_refinement_point_bytes,
            adaptive_stored_sample_count: storage.adaptive_stored_sample_count,
            adaptive_stored_sample_bytes: storage.adaptive_stored_sample_bytes,
            adaptive_dense_region_count: storage.adaptive_dense_region_count,
            adaptive_sparse_region_count: storage.adaptive_sparse_region_count,
            adaptive_dense_value_count: storage.adaptive_dense_value_count,
            adaptive_sparse_value_count: storage.adaptive_sparse_value_count,
            total_stored_sample_count: storage.total_stored_sample_count,
            total_stored_sample_bytes: storage.total_stored_sample_bytes,
            estimated_metadata_bytes: storage.estimated_metadata_bytes,
            estimated_total_bytes: storage.estimated_total_bytes,
        });
        metadata
    }
}

pub fn condition_sdf_for_backend_with_sample_budget(
    canonical: Arc<dyn Sdf>,
    max_initial_samples: usize,
) -> Arc<dyn Sdf> {
    if conditioned_kernel_ref(&canonical).is_some() {
        return canonical;
    }
    match ConditionedGeometryKernel::from_runtime_metadata_with_sample_budget(
        Arc::clone(&canonical),
        max_initial_samples,
    ) {
        Some(kernel) => Arc::new(kernel),
        None => canonical,
    }
}

pub fn condition_sdf_for_backend(canonical: Arc<dyn Sdf>) -> Arc<dyn Sdf> {
    condition_sdf_for_backend_with_sample_budget(
        canonical,
        ConditionedGeometryKernel::DEFAULT_RUNTIME_MAX_INITIAL_SAMPLES,
    )
}

pub fn conditioned_kernel_ref(sdf: &Arc<dyn Sdf>) -> Option<&ConditionedGeometryKernel> {
    sdf.as_ref()
        .as_any()
        .downcast_ref::<ConditionedGeometryKernel>()
}

pub fn condition_sdf_after_backend_edit(
    previous: Option<&Arc<dyn Sdf>>,
    next_canonical: Arc<dyn Sdf>,
) -> Arc<dyn Sdf> {
    condition_sdf_after_backend_edit_with_sample_budget(
        previous,
        next_canonical,
        ConditionedGeometryKernel::DEFAULT_RUNTIME_MAX_INITIAL_SAMPLES,
    )
}

pub fn condition_sdf_after_backend_edit_with_sample_budget(
    previous: Option<&Arc<dyn Sdf>>,
    next_canonical: Arc<dyn Sdf>,
    max_initial_samples: usize,
) -> Arc<dyn Sdf> {
    if let Some(previous_kernel) = previous.and_then(conditioned_kernel_ref) {
        if let Some(next_kernel) =
            previous_kernel.advance_to_canonical_after_metadata_change(Arc::clone(&next_canonical))
        {
            return Arc::new(next_kernel);
        }
    }

    condition_sdf_for_backend_with_sample_budget(next_canonical, max_initial_samples)
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct ConditioningJobId(pub u64);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConditioningJobState {
    Queued,
    Running,
    Ready,
    Failed,
    Superseded,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BackgroundConditioningSchedulerConfig {
    pub max_initial_samples: usize,
    pub coalesce_queued_jobs: bool,
}

impl Default for BackgroundConditioningSchedulerConfig {
    fn default() -> Self {
        Self {
            max_initial_samples: ConditionedGeometryKernel::DEFAULT_RUNTIME_MAX_INITIAL_SAMPLES,
            coalesce_queued_jobs: true,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ConditioningJobSummary {
    pub id: ConditioningJobId,
    pub state: ConditioningJobState,
    pub node_kind: String,
    pub parameter_fingerprint: u64,
    pub conditionable: bool,
    pub cache_state: ConditionedCacheState,
    pub update_mode: ConditioningUpdateMode,
    pub generation: Option<u64>,
    pub elapsed_ms: Option<u128>,
    pub error: Option<String>,
}

impl ConditioningJobSummary {
    fn from_metadata(
        id: ConditioningJobId,
        state: ConditioningJobState,
        metadata: &SdfNodeMetadata,
    ) -> Self {
        Self {
            id,
            state,
            node_kind: metadata.node_kind.clone(),
            parameter_fingerprint: metadata.parameter_fingerprint,
            conditionable: metadata.is_conditionable(),
            cache_state: ConditionedCacheState::Unavailable,
            update_mode: ConditioningUpdateMode::NotRun,
            generation: None,
            elapsed_ms: None,
            error: None,
        }
    }

    fn ready_from_sdf(id: ConditioningJobId, sdf: &Arc<dyn Sdf>, elapsed_ms: u128) -> Self {
        let metadata = sdf.metadata();
        let cache = metadata.conditioned_cache.as_ref();
        let conditionable = cache.is_some() || metadata.is_conditionable();
        let cache_state = cache.map_or(ConditionedCacheState::Unavailable, |cache| cache.state);
        let update_mode = conditioned_kernel_ref(sdf)
            .and_then(|kernel| {
                kernel
                    .conditioning()
                    .last_diagnostics()
                    .map(|diagnostics| diagnostics.update_mode)
            })
            .unwrap_or(if cache.is_some() {
                ConditioningUpdateMode::FullRebuild
            } else {
                ConditioningUpdateMode::DirectAnalytic
            });
        let generation = cache.map(|cache| cache.generation);
        Self {
            id,
            state: ConditioningJobState::Ready,
            node_kind: metadata.node_kind,
            parameter_fingerprint: metadata.parameter_fingerprint,
            conditionable,
            cache_state,
            update_mode,
            generation,
            elapsed_ms: Some(elapsed_ms),
            error: None,
        }
    }

    fn with_state(mut self, state: ConditioningJobState) -> Self {
        self.state = state;
        self
    }

    fn failed(mut self, error: impl Into<String>) -> Self {
        self.state = ConditioningJobState::Failed;
        self.error = Some(error.into());
        self
    }
}

#[derive(Clone)]
pub struct BackgroundConditioningResult {
    pub id: ConditioningJobId,
    pub sdf: Arc<dyn Sdf>,
    pub summary: ConditioningJobSummary,
}

#[derive(Default)]
struct BackgroundConditioningSchedulerState {
    summaries: HashMap<ConditioningJobId, ConditioningJobSummary>,
    latest_ready: Option<BackgroundConditioningResult>,
}

enum BackgroundConditioningCommand {
    Enqueue {
        id: ConditioningJobId,
        previous: Option<Arc<dyn Sdf>>,
        canonical: Arc<dyn Sdf>,
    },
    Shutdown,
}

pub struct BackgroundConditioningScheduler {
    next_job_id: AtomicU64,
    sender: mpsc::Sender<BackgroundConditioningCommand>,
    state: Arc<Mutex<BackgroundConditioningSchedulerState>>,
    worker: Option<thread::JoinHandle<()>>,
}

impl BackgroundConditioningScheduler {
    pub fn new() -> Self {
        Self::with_config(BackgroundConditioningSchedulerConfig::default())
    }

    pub fn with_config(config: BackgroundConditioningSchedulerConfig) -> Self {
        let (sender, receiver) = mpsc::channel();
        let state = Arc::new(Mutex::new(BackgroundConditioningSchedulerState::default()));
        let worker_state = Arc::clone(&state);
        let worker = thread::spawn(move || {
            run_background_conditioning_worker(config, receiver, worker_state);
        });

        Self {
            next_job_id: AtomicU64::new(1),
            sender,
            state,
            worker: Some(worker),
        }
    }

    pub fn enqueue_rebuild(&self, canonical: Arc<dyn Sdf>) -> ConditioningJobId {
        self.enqueue_after_edit(None, canonical)
    }

    pub fn enqueue_after_edit(
        &self,
        previous: Option<Arc<dyn Sdf>>,
        canonical: Arc<dyn Sdf>,
    ) -> ConditioningJobId {
        let id = ConditioningJobId(self.next_job_id.fetch_add(1, Ordering::Relaxed));
        let metadata = canonical.metadata();
        self.store_summary(ConditioningJobSummary::from_metadata(
            id,
            ConditioningJobState::Queued,
            &metadata,
        ));

        if self
            .sender
            .send(BackgroundConditioningCommand::Enqueue {
                id,
                previous,
                canonical,
            })
            .is_err()
        {
            self.update_summary(id, |summary| {
                summary.failed("background conditioning worker is not available")
            });
        }

        id
    }

    pub fn job_summary(&self, id: ConditioningJobId) -> Option<ConditioningJobSummary> {
        self.state.lock().ok()?.summaries.get(&id).cloned()
    }

    pub fn job_summaries(&self) -> Vec<ConditioningJobSummary> {
        let Ok(state) = self.state.lock() else {
            return Vec::new();
        };
        let mut summaries: Vec<_> = state.summaries.values().cloned().collect();
        summaries.sort_by_key(|summary| summary.id.0);
        summaries
    }

    pub fn latest_ready(&self) -> Option<BackgroundConditioningResult> {
        self.state.lock().ok()?.latest_ready.clone()
    }

    pub fn take_latest_ready(&self) -> Option<BackgroundConditioningResult> {
        self.state.lock().ok()?.latest_ready.take()
    }

    fn store_summary(&self, summary: ConditioningJobSummary) {
        if let Ok(mut state) = self.state.lock() {
            state.summaries.insert(summary.id, summary);
        }
    }

    fn update_summary(
        &self,
        id: ConditioningJobId,
        update: impl FnOnce(ConditioningJobSummary) -> ConditioningJobSummary,
    ) {
        if let Ok(mut state) = self.state.lock() {
            if let Some(summary) = state.summaries.remove(&id) {
                state.summaries.insert(id, update(summary));
            }
        }
    }
}

impl Default for BackgroundConditioningScheduler {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for BackgroundConditioningScheduler {
    fn drop(&mut self) {
        let _ = self.sender.send(BackgroundConditioningCommand::Shutdown);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

fn run_background_conditioning_worker(
    config: BackgroundConditioningSchedulerConfig,
    receiver: mpsc::Receiver<BackgroundConditioningCommand>,
    state: Arc<Mutex<BackgroundConditioningSchedulerState>>,
) {
    while let Ok(command) = receiver.recv() {
        let (mut id, mut previous, mut canonical) = match command {
            BackgroundConditioningCommand::Enqueue {
                id,
                previous,
                canonical,
            } => (id, previous, canonical),
            BackgroundConditioningCommand::Shutdown => return,
        };

        if config.coalesce_queued_jobs {
            loop {
                match receiver.try_recv() {
                    Ok(BackgroundConditioningCommand::Enqueue {
                        id: next_id,
                        previous: next_previous,
                        canonical: next_canonical,
                    }) => {
                        mark_background_conditioning_job(
                            &state,
                            id,
                            ConditioningJobState::Superseded,
                        );
                        id = next_id;
                        previous = next_previous;
                        canonical = next_canonical;
                    }
                    Ok(BackgroundConditioningCommand::Shutdown) => return,
                    Err(mpsc::TryRecvError::Empty) => break,
                    Err(mpsc::TryRecvError::Disconnected) => break,
                }
            }
        }

        mark_background_conditioning_job(&state, id, ConditioningJobState::Running);
        let start = Instant::now();
        let conditioned = condition_sdf_after_backend_edit_with_sample_budget(
            previous.as_ref(),
            canonical,
            config.max_initial_samples,
        );
        let summary =
            ConditioningJobSummary::ready_from_sdf(id, &conditioned, start.elapsed().as_millis());
        if let Ok(mut state) = state.lock() {
            state.summaries.insert(id, summary.clone());
            state.latest_ready = Some(BackgroundConditioningResult {
                id,
                sdf: conditioned,
                summary,
            });
        }
    }
}

fn mark_background_conditioning_job(
    state: &Arc<Mutex<BackgroundConditioningSchedulerState>>,
    id: ConditioningJobId,
    next_state: ConditioningJobState,
) {
    if let Ok(mut state) = state.lock() {
        if let Some(summary) = state.summaries.get_mut(&id) {
            summary.state = next_state;
        }
    }
}

impl ConditionedGeometryModel {
    pub fn new(domain: SdfBounds, block_size: f32, policy: ConditioningPolicy) -> Self {
        assert!(domain.is_valid(), "conditioned model domain must be valid");
        assert!(
            block_size > 0.0,
            "conditioned model block size must be positive"
        );
        assert!(
            policy.grid_spacing > 0.0,
            "conditioned model grid spacing must be positive"
        );
        assert!(
            policy.max_region_sample_count > 0,
            "conditioned model region sample budget must be positive"
        );
        Self {
            domain,
            policy,
            block_size,
            generation: 0,
            blocks: HashMap::new(),
            adaptive_regions: Vec::new(),
            adaptive_requested_region_count: 0,
            adaptive_skipped_region_count: 0,
            adaptive_oversized_region_count: 0,
            adaptive_requested_min_grid_spacing: None,
            adaptive_requested_feature_ids: HashSet::new(),
            dirty_history: Vec::new(),
            last_diagnostics: None,
            last_rejected_diagnostics: None,
        }
    }

    pub fn domain(&self) -> &SdfBounds {
        &self.domain
    }

    pub fn generation(&self) -> u64 {
        self.generation
    }

    pub fn block_count(&self) -> usize {
        self.blocks.len()
    }

    pub fn adaptive_region_count(&self) -> usize {
        self.adaptive_regions.len()
    }

    pub fn storage_stats(&self) -> ConditionedStorageStats {
        ConditionedStorageStats::from_model(self)
    }

    pub fn surface_refinement_bounds(&self) -> Vec<SdfBounds> {
        self.blocks
            .values()
            .filter(|block| block.surface_refinement.is_some())
            .map(|block| block.bounds.clone())
            .collect()
    }

    pub fn adaptive_stats(&self) -> AdaptiveConditioningStats {
        let mut stats = AdaptiveConditioningStats::from_regions(&self.adaptive_regions);
        stats.requested_region_count = self.adaptive_requested_region_count;
        stats.skipped_region_count = self.adaptive_skipped_region_count;
        stats.oversized_region_count = self.adaptive_oversized_region_count;
        stats.requested_min_grid_spacing = self.adaptive_requested_min_grid_spacing;
        stats.requested_feature_id_count = self.adaptive_requested_feature_ids.len();
        stats
    }

    pub fn adaptive_feature_coverage(&self) -> f32 {
        if self.adaptive_requested_feature_ids.is_empty() {
            return 1.0;
        }
        let covered: HashSet<_> = self
            .adaptive_regions
            .iter()
            .flat_map(|region| region.feature_ids.iter().cloned())
            .collect();
        let covered_count = self
            .adaptive_requested_feature_ids
            .iter()
            .filter(|feature_id| covered.contains(*feature_id))
            .count();
        covered_count as f32 / self.adaptive_requested_feature_ids.len() as f32
    }

    pub fn conditioned_distance(&self, point: Vec3) -> Option<f32> {
        if !self.domain.contains(point) {
            return None;
        }
        let index = self.clamped_block_index_for_point(point);
        let block = self.blocks.get(&index);
        if let Some(surface_refined_distance) =
            block.and_then(|block| block.surface_refined_distance(point))
        {
            return Some(surface_refined_distance);
        }
        let block_distance = self
            .blocks
            .get(&index)
            .and_then(|block| block.interpolated_distance(point, self.policy.grid_spacing));
        if let Some(adaptive_distance) = self.adaptive_conditioned_distance(point) {
            if let Some(block_distance) = block_distance {
                if adaptive_conflicts_with_block_near_surface(
                    adaptive_distance,
                    block_distance,
                    &self.policy,
                ) {
                    return Some(block_distance);
                }
            }
            return Some(adaptive_distance);
        }
        block_distance
    }

    pub fn surface_refined_distance(&self, point: Vec3) -> Option<f32> {
        if !self.domain.contains(point) {
            return None;
        }
        let index = self.clamped_block_index_for_point(point);
        self.blocks
            .get(&index)
            .and_then(|block| block.surface_refined_distance(point))
    }

    pub fn surface_refined_reinitialized_distance(&self, point: Vec3) -> Option<f32> {
        if !self.domain.contains(point) {
            return None;
        }
        let index = self.clamped_block_index_for_point(point);
        let block = self.blocks.get(&index)?;
        let sign_hint = block.interpolated_distance(point, self.policy.grid_spacing);
        block.surface_refined_reinitialized_distance(point, sign_hint)
    }

    fn adaptive_conditioned_distance(&self, point: Vec3) -> Option<f32> {
        self.adaptive_regions
            .iter()
            .filter(|region| region.bounds.contains(point))
            .filter_map(|region| {
                region
                    .sample_conditioned(point)
                    .filter(|distance| distance.is_finite())
                    .map(|distance| (region.spacing, distance))
            })
            .min_by(|(a_spacing, _), (b_spacing, _)| {
                a_spacing
                    .partial_cmp(b_spacing)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(_, distance)| distance)
    }

    pub fn effective_grid_spacing(&self, point: Vec3) -> f32 {
        self.adaptive_regions
            .iter()
            .filter(|region| region.bounds.contains(point))
            .map(|region| region.spacing)
            .fold(self.policy.grid_spacing, |spacing, candidate| {
                spacing.min(candidate)
            })
    }

    pub fn conditioned_gradient(&self, point: Vec3, step: f32) -> Option<Vec3> {
        let h = step.max(1.0e-4);
        let dx = self.conditioned_distance(point + Vec3::X * h)?
            - self.conditioned_distance(point - Vec3::X * h)?;
        let dy = self.conditioned_distance(point + Vec3::Y * h)?
            - self.conditioned_distance(point - Vec3::Y * h)?;
        let dz = self.conditioned_distance(point + Vec3::Z * h)?
            - self.conditioned_distance(point - Vec3::Z * h)?;
        Some(Vec3::new(dx, dy, dz) / (2.0 * h))
    }

    pub fn diagnose_cache_quality(
        &self,
        canonical: &dyn Sdf,
    ) -> ConditionedCacheQualityDiagnostics {
        let mut grid_point_count = 0_usize;
        let mut near_interface_sample_count = 0_usize;
        let mut gradient_sample_count = 0_usize;
        let mut projection_sample_count = 0_usize;
        let mut sign_mismatch_count_near_interface = 0_usize;
        let mut max_surface_residual = 0.0_f32;
        let mut max_projection_residual = 0.0_f32;
        let mut distance_errors = Vec::new();
        let mut gradient_errors = Vec::new();
        let sign_tolerance = self.policy.max_interface_error.max(1.0e-6);
        let surface_residual_band = self.policy.grid_spacing * 0.5;

        for point in sample_points(&self.domain, self.policy.grid_spacing) {
            let Some(conditioned_d) = self.conditioned_distance(point) else {
                continue;
            };
            grid_point_count += 1;
            let canonical_d = canonical.distance(point);
            let near_interface = canonical_d.abs() <= self.policy.interface_band
                || conditioned_d.abs() <= self.policy.interface_band;
            if !near_interface {
                continue;
            }

            near_interface_sample_count += 1;
            distance_errors.push((conditioned_d - canonical_d).abs());
            if sign_with_tolerance(canonical_d, sign_tolerance)
                != sign_with_tolerance(conditioned_d, sign_tolerance)
            {
                sign_mismatch_count_near_interface += 1;
            }
            if conditioned_d.abs() <= surface_residual_band {
                max_surface_residual = max_surface_residual.max(canonical_d.abs());
            }
            if let Some(gradient) =
                self.conditioned_gradient(point, self.effective_grid_spacing(point))
            {
                gradient_sample_count += 1;
                let gradient_length = gradient.length();
                gradient_errors.push((gradient_length - 1.0).abs());
                let normal = gradient.normalize_or_zero();
                if normal.length_squared() > f32::EPSILON {
                    let projected = point - normal * conditioned_d;
                    max_projection_residual =
                        max_projection_residual.max(canonical.distance(projected).abs());
                    projection_sample_count += 1;
                }
            }
        }

        let distance_abs_error_p50 = percentile(&mut distance_errors.clone(), 50.0);
        let distance_abs_error_p95 = percentile(&mut distance_errors.clone(), 95.0);
        let distance_abs_error_max = distance_errors
            .iter()
            .copied()
            .fold(0.0_f32, |acc, value| acc.max(value));
        let gradient_abs_error_p50 = percentile(&mut gradient_errors.clone(), 50.0);
        let gradient_abs_error_p95 = percentile(&mut gradient_errors.clone(), 95.0);
        let gradient_abs_error_max = gradient_errors
            .iter()
            .copied()
            .fold(0.0_f32, |acc, value| acc.max(value));

        let mut confidence = 1.0_f32;
        if sign_mismatch_count_near_interface > 0 {
            confidence = 0.0;
        } else if gradient_abs_error_p95 > self.policy.max_gradient_abs_error_p95 {
            confidence =
                (self.policy.max_gradient_abs_error_p95 / gradient_abs_error_p95).clamp(0.0, 1.0);
        }
        if max_surface_residual > self.policy.max_interface_error {
            confidence = confidence
                .min((self.policy.max_interface_error / max_surface_residual).clamp(0.0, 1.0));
        }

        ConditionedCacheQualityDiagnostics {
            grid_point_count,
            near_interface_sample_count,
            gradient_sample_count,
            projection_sample_count,
            sign_mismatch_count_near_interface,
            max_surface_residual,
            max_projection_residual,
            distance_abs_error_p50,
            distance_abs_error_p95,
            distance_abs_error_max,
            gradient_abs_error_p50,
            gradient_abs_error_p95,
            gradient_abs_error_max,
            adaptive_region_count: self.adaptive_region_count(),
            adaptive_sample_count: self.adaptive_stats().sample_count,
            adaptive_min_grid_spacing: self.adaptive_stats().min_grid_spacing,
            adaptive_feature_id_count: self.adaptive_stats().feature_id_count,
            adaptive_requested_feature_id_count: self.adaptive_stats().requested_feature_id_count,
            adaptive_feature_coverage: self.adaptive_feature_coverage(),
            confidence,
        }
    }

    fn diagnose_local_update_cache_quality(
        &self,
        after: &dyn Sdf,
        dirty_regions: &[DirtyRegion],
        affected_indexes: &[BlockIndex],
    ) -> ConditioningDiagnostics {
        #[derive(Default)]
        struct LocalQualityAccumulator {
            grid_point_count: usize,
            dirty_sample_count: usize,
            conditioned_sample_count: usize,
            changed_near_interface_count: usize,
            max_surface_residual: f32,
            sign_mismatch_count_near_interface: usize,
            gradient_errors: Vec<f32>,
        }

        impl LocalQualityAccumulator {
            fn merge(mut self, other: Self) -> Self {
                self.grid_point_count += other.grid_point_count;
                self.dirty_sample_count += other.dirty_sample_count;
                self.conditioned_sample_count += other.conditioned_sample_count;
                self.changed_near_interface_count += other.changed_near_interface_count;
                self.max_surface_residual =
                    self.max_surface_residual.max(other.max_surface_residual);
                self.sign_mismatch_count_near_interface += other.sign_mismatch_count_near_interface;
                self.gradient_errors.extend(other.gradient_errors);
                self
            }
        }

        let update_bounds: Vec<_> = dirty_regions
            .iter()
            .map(DirtyRegion::update_bounds)
            .collect();
        let sign_tolerance = self
            .policy
            .max_interface_error
            .max(self.policy.grid_spacing * 0.75)
            .max(1.0e-6);
        let surface_residual_band = self.policy.grid_spacing * 0.5;

        let quality = affected_indexes
            .par_iter()
            .filter_map(|index| self.blocks.get(index))
            .map(|block| {
                let mut quality = LocalQualityAccumulator::default();
                for point in sample_points(&block.bounds, self.policy.grid_spacing) {
                    quality.grid_point_count += 1;
                    if dirty_regions
                        .iter()
                        .any(|dirty_region| dirty_region.bounds.contains(point))
                    {
                        quality.dirty_sample_count += 1;
                    }
                    if update_bounds
                        .iter()
                        .any(|update_bounds| update_bounds.contains(point))
                    {
                        quality.conditioned_sample_count += 1;
                    }

                    let Some(conditioned_d) =
                        block.interpolated_distance(point, self.policy.grid_spacing)
                    else {
                        continue;
                    };
                    let canonical_d = after.distance(point);
                    let near_interface = canonical_d.abs() <= self.policy.interface_band
                        || conditioned_d.abs() <= self.policy.interface_band;
                    if !near_interface {
                        continue;
                    }

                    if strong_sign_mismatch(canonical_d, conditioned_d, sign_tolerance) {
                        quality.sign_mismatch_count_near_interface += 1;
                    }
                    if (canonical_d - conditioned_d).abs() > sign_tolerance {
                        quality.changed_near_interface_count += 1;
                    }
                    if conditioned_d.abs() <= surface_residual_band {
                        quality.max_surface_residual =
                            quality.max_surface_residual.max(canonical_d.abs());
                    }
                    if let Some(gradient) =
                        self.conditioned_gradient(point, self.policy.grid_spacing)
                    {
                        quality
                            .gradient_errors
                            .push((gradient.length() - 1.0).abs());
                    }
                }
                quality
            })
            .reduce(
                LocalQualityAccumulator::default,
                LocalQualityAccumulator::merge,
            );

        let halo_sample_count = quality
            .conditioned_sample_count
            .saturating_sub(quality.dirty_sample_count);
        let gradient_abs_error_p50 = percentile(&mut quality.gradient_errors.clone(), 50.0);
        let gradient_abs_error_p95 = percentile(&mut quality.gradient_errors.clone(), 95.0);
        let gradient_abs_error_max = quality
            .gradient_errors
            .iter()
            .copied()
            .fold(0.0_f32, |acc, value| acc.max(value));

        let mut quality_confidence = 1.0_f32;
        if quality.sign_mismatch_count_near_interface > 0 {
            quality_confidence = 0.0;
        } else if gradient_abs_error_p95 > self.policy.max_gradient_abs_error_p95 {
            quality_confidence =
                (self.policy.max_gradient_abs_error_p95 / gradient_abs_error_p95).clamp(0.0, 1.0);
        }
        if quality.max_surface_residual > self.policy.max_interface_error {
            quality_confidence = quality_confidence.min(
                (self.policy.max_interface_error / quality.max_surface_residual).clamp(0.0, 1.0),
            );
        }

        let publication_confidence: f32 = if quality.sign_mismatch_count_near_interface == 0
            && quality.conditioned_sample_count > 0
        {
            1.0
        } else {
            0.0
        };

        let cache_state = if publication_confidence >= 1.0 {
            ConditionedCacheState::Ready
        } else {
            ConditionedCacheState::Rejected
        };

        ConditioningDiagnostics {
            cache_state,
            update_mode: ConditioningUpdateMode::LocalIncremental,
            validated_against_full: false,
            grid_point_count: quality.grid_point_count,
            dirty_sample_count: quality.dirty_sample_count,
            halo_sample_count,
            conditioned_sample_count: quality.conditioned_sample_count,
            changed_near_interface_count: quality.changed_near_interface_count,
            max_abs_error_inside_update: quality.max_surface_residual,
            max_abs_error_near_interface: quality.max_surface_residual,
            sign_mismatch_count_near_interface: quality.sign_mismatch_count_near_interface,
            unexpected_sign_flip_count_outside_update: 0,
            gradient_abs_error_p50,
            gradient_abs_error_p95,
            gradient_abs_error_max,
            confidence: publication_confidence,
            publication_confidence,
            quality_confidence,
        }
    }

    pub fn dirty_history(&self) -> &[DirtyRegion] {
        &self.dirty_history
    }

    pub fn last_diagnostics(&self) -> Option<&ConditioningDiagnostics> {
        self.last_diagnostics.as_ref()
    }

    pub fn last_rejected_diagnostics(&self) -> Option<&ConditioningDiagnostics> {
        self.last_rejected_diagnostics.as_ref()
    }

    pub fn initialize_from_canonical(&mut self, sdf: &dyn Sdf) -> ConditioningUpdateSummary {
        self.generation += 1;
        self.blocks.clear();
        self.adaptive_regions.clear();
        self.last_rejected_diagnostics = None;
        let indexes = self.all_block_indexes();
        let regenerated_blocks =
            self.regenerate_block_regions(sdf, &indexes, ConditionedCacheState::Ready);
        let reconditioning = ReconditioningStats::from_blocks(&regenerated_blocks);
        for block in regenerated_blocks {
            self.blocks.insert(block.index, block);
        }
        let seed_regions = self.initial_adaptive_dirty_regions(&sdf.metadata());
        let adaptive =
            self.refresh_adaptive_regions(sdf, &seed_regions, AdaptiveRefreshScope::FullRebuild);
        let diagnostics = ConditioningDiagnostics::full_rebuild(self.blocks.len());
        self.last_diagnostics = Some(diagnostics.clone());
        ConditioningUpdateSummary {
            generation: self.generation,
            update_mode: ConditioningUpdateMode::FullRebuild,
            cache_state: ConditionedCacheState::Ready,
            dirty_regions: Vec::new(),
            invalidated_block_count: self.blocks.len(),
            regenerated_block_count: self.blocks.len(),
            full_rebuild_used: true,
            reconditioning,
            adaptive,
            diagnostics,
        }
    }

    pub fn apply_edit_and_recondition(
        &mut self,
        before: &dyn Sdf,
        after: &dyn Sdf,
        edit: GeometryEdit,
    ) -> ConditioningUpdateSummary {
        self.apply_edits_and_recondition(before, after, GeometryEditTransaction::single(edit))
    }

    pub fn apply_edits_and_recondition(
        &mut self,
        before: &dyn Sdf,
        after: &dyn Sdf,
        transaction: GeometryEditTransaction,
    ) -> ConditioningUpdateSummary {
        let dirty_regions = transaction.dirty_regions();
        self.dirty_history.extend(dirty_regions.iter().cloned());
        let affected_indexes = self.block_indexes_intersecting_dirty_regions(&dirty_regions);

        if self.should_skip_local_update_for_affected_blocks(&affected_indexes) {
            let diagnostics = ConditioningDiagnostics::full_rebuild(self.blocks.len());
            return self.apply_full_rebuild_after_rejected_local(
                after,
                dirty_regions,
                affected_indexes.len(),
                diagnostics,
                true,
            );
        }

        match self.policy.local_validation_mode {
            LocalValidationMode::FullComparison => {
                let diagnostics = compare_local_update_regions_to_full(
                    before,
                    after,
                    &self.domain,
                    &dirty_regions,
                    &self.policy,
                );
                if diagnostics.accepted(&self.policy) {
                    self.apply_local_regeneration(
                        after,
                        dirty_regions,
                        affected_indexes,
                        diagnostics,
                    )
                } else {
                    self.apply_full_rebuild_after_rejected_local(
                        after,
                        dirty_regions,
                        affected_indexes.len(),
                        diagnostics,
                        true,
                    )
                }
            }
            LocalValidationMode::LocalCacheQuality => {
                self.generation += 1;
                let regenerated_blocks = self.regenerate_block_regions_with_refinement(
                    after,
                    &affected_indexes,
                    ConditionedCacheState::Ready,
                    SurfaceRefinementMode::SurfaceOnly,
                );
                let reconditioning = ReconditioningStats::from_blocks(&regenerated_blocks);
                for block in regenerated_blocks {
                    self.blocks.insert(block.index, block);
                }
                let diagnostics = self.diagnose_local_update_cache_quality(
                    after,
                    &dirty_regions,
                    &affected_indexes,
                );
                if diagnostics.accepted(&self.policy) {
                    let adaptive = self.refresh_adaptive_regions(
                        after,
                        &dirty_regions,
                        AdaptiveRefreshScope::LocalEdit,
                    );
                    self.last_rejected_diagnostics = None;
                    self.last_diagnostics = Some(diagnostics.clone());
                    ConditioningUpdateSummary {
                        generation: self.generation,
                        update_mode: ConditioningUpdateMode::LocalIncremental,
                        cache_state: diagnostics.cache_state,
                        dirty_regions,
                        invalidated_block_count: affected_indexes.len(),
                        regenerated_block_count: affected_indexes.len(),
                        full_rebuild_used: false,
                        reconditioning,
                        adaptive,
                        diagnostics,
                    }
                } else {
                    self.apply_full_rebuild_after_rejected_local(
                        after,
                        dirty_regions,
                        affected_indexes.len(),
                        diagnostics,
                        false,
                    )
                }
            }
        }
    }

    fn apply_local_regeneration(
        &mut self,
        after: &dyn Sdf,
        dirty_regions: Vec<DirtyRegion>,
        affected_indexes: Vec<BlockIndex>,
        diagnostics: ConditioningDiagnostics,
    ) -> ConditioningUpdateSummary {
        self.generation += 1;
        let regenerated_blocks = self.regenerate_block_regions_with_refinement(
            after,
            &affected_indexes,
            ConditionedCacheState::Ready,
            SurfaceRefinementMode::SurfaceOnly,
        );
        let reconditioning = ReconditioningStats::from_blocks(&regenerated_blocks);
        for block in regenerated_blocks {
            self.blocks.insert(block.index, block);
        }
        let adaptive =
            self.refresh_adaptive_regions(after, &dirty_regions, AdaptiveRefreshScope::LocalEdit);
        self.last_rejected_diagnostics = None;
        self.last_diagnostics = Some(diagnostics.clone());
        ConditioningUpdateSummary {
            generation: self.generation,
            update_mode: ConditioningUpdateMode::LocalIncremental,
            cache_state: diagnostics.cache_state,
            dirty_regions,
            invalidated_block_count: affected_indexes.len(),
            regenerated_block_count: affected_indexes.len(),
            full_rebuild_used: false,
            reconditioning,
            adaptive,
            diagnostics,
        }
    }

    fn apply_full_rebuild_after_rejected_local(
        &mut self,
        after: &dyn Sdf,
        dirty_regions: Vec<DirtyRegion>,
        invalidated_block_count: usize,
        diagnostics: ConditioningDiagnostics,
        increment_generation: bool,
    ) -> ConditioningUpdateSummary {
        if increment_generation {
            self.generation += 1;
        }
        self.blocks.clear();
        let indexes = self.all_block_indexes();
        let regenerated_blocks = self.regenerate_block_regions_with_refinement(
            after,
            &indexes,
            ConditionedCacheState::Ready,
            SurfaceRefinementMode::SurfaceOnly,
        );
        let reconditioning = ReconditioningStats::from_blocks(&regenerated_blocks);
        for block in regenerated_blocks {
            self.blocks.insert(block.index, block);
        }
        let seed_regions = self.initial_adaptive_dirty_regions(&after.metadata());
        let adaptive =
            self.refresh_adaptive_regions(after, &seed_regions, AdaptiveRefreshScope::FullRebuild);
        self.last_rejected_diagnostics = (diagnostics.update_mode
            == ConditioningUpdateMode::LocalIncremental)
            .then(|| diagnostics.clone());
        self.last_diagnostics = Some(ConditioningDiagnostics::full_rebuild(self.blocks.len()));
        ConditioningUpdateSummary {
            generation: self.generation,
            update_mode: ConditioningUpdateMode::FullRebuild,
            cache_state: ConditionedCacheState::Ready,
            dirty_regions,
            invalidated_block_count,
            regenerated_block_count: indexes.len(),
            full_rebuild_used: true,
            reconditioning,
            adaptive,
            diagnostics,
        }
    }

    fn initial_adaptive_dirty_regions(&self, metadata: &SdfNodeMetadata) -> Vec<DirtyRegion> {
        if !self.policy.adaptive_enabled {
            return Vec::new();
        }
        let mut regions = Vec::new();
        self.collect_initial_adaptive_dirty_regions(metadata, &mut regions);
        regions.sort_by(initial_adaptive_region_order);
        regions
    }

    fn initial_adaptive_candidate_limit(&self) -> usize {
        self.policy.adaptive_max_regions
    }

    fn collect_initial_adaptive_dirty_regions(
        &self,
        metadata: &SdfNodeMetadata,
        regions: &mut Vec<DirtyRegion>,
    ) {
        if regions.len() >= self.initial_adaptive_candidate_limit() {
            return;
        }

        if !metadata.feature_regions.is_empty() {
            for feature_region in &metadata.feature_regions {
                if regions.len() >= self.initial_adaptive_candidate_limit() {
                    return;
                }
                let Some(spacing) = feature_region.recommended_grid_spacing(
                    self.policy.adaptive_samples_per_feature,
                    self.policy.adaptive_min_grid_spacing,
                ) else {
                    continue;
                };
                if !self.policy.should_use_adaptive_spacing(spacing) {
                    continue;
                }
                let Some(bounds) = feature_region.bounds.intersection(&self.domain) else {
                    continue;
                };
                let mut dirty_region =
                    DirtyRegion::new(DirtyRegionSource::FullGeometry, bounds).with_halo(spacing);
                dirty_region
                    .feature_ids
                    .push(feature_region.feature_id.clone());
                dirty_region.recommended_grid_spacing = Some(spacing);
                regions.push(dirty_region);
            }
            return;
        }

        if metadata_node_stores_child_bounds_in_local_space(&metadata.node_kind) {
            return;
        }

        for dependency in &metadata.dependencies {
            self.collect_initial_adaptive_dirty_regions(&dependency.metadata, regions);
            if regions.len() >= self.initial_adaptive_candidate_limit() {
                return;
            }
        }

        let Some(spacing) = metadata.recommended_conditioning_spacing(
            self.policy.adaptive_samples_per_feature,
            self.policy.adaptive_min_grid_spacing,
        ) else {
            return;
        };
        if !self.policy.should_use_adaptive_spacing(spacing) {
            return;
        }
        let Some(bounds) = metadata.support_bounds.clone() else {
            return;
        };
        let Some(bounds) = bounds.intersection(&self.domain) else {
            return;
        };

        let mut dirty_region =
            DirtyRegion::new(DirtyRegionSource::FullGeometry, bounds).with_halo(spacing);
        dirty_region.feature_ids = metadata.feature_ids.clone();
        dirty_region.recommended_grid_spacing = Some(spacing);
        regions.push(dirty_region);
    }

    fn initial_smooth_interface_dirty_region(
        &self,
        metadata: &SdfNodeMetadata,
    ) -> Option<DirtyRegion> {
        let radius = metadata
            .operation
            .interface_radius
            .max(metadata.operation.dirty_expansion)
            .max(self.policy.grid_spacing);
        let bounds =
            expanded_child_interface_bounds(metadata, radius)?.intersection(&self.domain)?;
        if !bounds.is_valid() {
            return None;
        }

        let interface_feature_size = metadata
            .operation
            .interface_radius
            .max(metadata.min_feature_size.unwrap_or(0.0))
            .max(self.policy.grid_spacing * 0.5);
        let base_spacing = (interface_feature_size
            / (self.policy.adaptive_samples_per_feature.max(1.0) * 2.0))
            .max(self.policy.adaptive_min_grid_spacing)
            .min(self.policy.grid_spacing * 0.5);
        let spacing = self.fit_initial_interface_spacing_to_budget(&bounds, base_spacing)?;
        if !self.policy.should_use_adaptive_spacing(spacing) {
            return None;
        }

        let mut dirty_region =
            DirtyRegion::new(DirtyRegionSource::Blend, bounds).with_halo(spacing * 2.0);
        dirty_region.recommended_grid_spacing = Some(spacing);
        dirty_region
            .feature_ids
            .push(format!("{}.smooth_interface", metadata.node_kind));
        for feature_id in metadata.feature_ids.iter().take(4) {
            if !dirty_region.feature_ids.contains(feature_id) {
                dirty_region.feature_ids.push(feature_id.clone());
            }
        }
        Some(dirty_region)
    }

    fn fit_initial_interface_spacing_to_budget(
        &self,
        bounds: &SdfBounds,
        requested_spacing: f32,
    ) -> Option<f32> {
        if !bounds.is_valid() || !requested_spacing.is_finite() || requested_spacing <= 0.0 {
            return None;
        }
        let target_sample_count = self
            .policy
            .adaptive_max_region_sample_count
            .min(self.policy.adaptive_max_sparse_scan_sample_count)
            .min(20_000)
            .max(512);
        let max_spacing = (self.policy.grid_spacing * 0.75).max(requested_spacing);
        let mut spacing = requested_spacing.max(self.policy.adaptive_min_grid_spacing);
        for _ in 0..24 {
            let update_bounds = bounds.expanded(spacing * 2.0).clamped_to(&self.domain);
            if update_bounds.is_valid()
                && bounded_sample_count(&update_bounds, spacing) <= target_sample_count
            {
                return Some(spacing);
            }
            let next_spacing = (spacing * 1.35).min(max_spacing);
            if next_spacing <= spacing * 1.0001 {
                break;
            }
            spacing = next_spacing;
        }
        let update_bounds = bounds.expanded(max_spacing * 2.0).clamped_to(&self.domain);
        if update_bounds.is_valid()
            && bounded_sample_count(&update_bounds, max_spacing) <= target_sample_count
        {
            Some(max_spacing)
        } else {
            None
        }
    }

    fn refresh_adaptive_regions(
        &mut self,
        sdf: &dyn Sdf,
        dirty_regions: &[DirtyRegion],
        scope: AdaptiveRefreshScope,
    ) -> AdaptiveConditioningStats {
        if !self.policy.adaptive_enabled || dirty_regions.is_empty() {
            self.adaptive_requested_region_count = 0;
            self.adaptive_skipped_region_count = 0;
            self.adaptive_oversized_region_count = 0;
            self.adaptive_requested_min_grid_spacing = None;
            self.adaptive_requested_feature_ids.clear();
            return self.adaptive_stats();
        }

        self.adaptive_requested_region_count = 0;
        self.adaptive_skipped_region_count = 0;
        self.adaptive_oversized_region_count = 0;
        self.adaptive_requested_min_grid_spacing = None;
        self.adaptive_requested_feature_ids.clear();

        let update_bounds: Vec<_> = dirty_regions
            .iter()
            .filter_map(|dirty_region| {
                self.adaptive_refresh_bounds_for_dirty_region(dirty_region, scope)
            })
            .collect();
        if update_bounds.is_empty() {
            return self.adaptive_stats();
        }

        self.adaptive_regions.retain(|region| {
            !update_bounds
                .iter()
                .any(|bounds| region.bounds.intersects(bounds))
        });

        let mut requests = Vec::new();
        for dirty_region in dirty_regions {
            let Some(spacing) = self.policy.adaptive_spacing_for_dirty_region(dirty_region) else {
                continue;
            };
            self.adaptive_requested_region_count += 1;
            self.adaptive_requested_min_grid_spacing = Some(
                self.adaptive_requested_min_grid_spacing
                    .map_or(spacing, |min| min.min(spacing)),
            );
            self.adaptive_requested_feature_ids
                .extend(dirty_region.feature_ids.iter().cloned());
            let Some(update_bounds) =
                self.adaptive_refresh_bounds_for_dirty_region(dirty_region, scope)
            else {
                self.adaptive_skipped_region_count += 1;
                continue;
            };
            requests.push(AdaptiveRegionRequest {
                source: dirty_region.source,
                feature_ids: dirty_region.feature_ids.clone(),
                update_bounds,
                requested_spacing: spacing,
            });
        }

        let requests = prioritize_adaptive_region_requests(requests);

        for (request_index, request) in requests.iter().enumerate() {
            if self.adaptive_regions.len() >= self.policy.adaptive_max_regions {
                self.adaptive_skipped_region_count += 1;
                continue;
            }
            let active_feature_ids: HashSet<_> = self
                .adaptive_regions
                .iter()
                .flat_map(|region| region.feature_ids.iter().cloned())
                .collect();
            let current_feature_ids: HashSet<_> = request.feature_ids.iter().cloned().collect();
            let remaining_uncovered_feature_count = uncovered_adaptive_feature_id_count(
                &requests[request_index + 1..],
                &active_feature_ids,
                &current_feature_ids,
            );
            let available_region_count = self
                .policy
                .adaptive_max_regions
                .saturating_sub(self.adaptive_regions.len());
            let reserved_region_count =
                remaining_uncovered_feature_count.min(available_region_count.saturating_sub(1));
            let request_region_budget = available_region_count
                .saturating_sub(reserved_region_count)
                .max(1);
            let Some((spacing, region_chunks)) = self.adaptive_chunks_for_request(
                &request.update_bounds,
                request.requested_spacing,
                request_region_budget,
            ) else {
                self.adaptive_skipped_region_count += 1;
                self.adaptive_oversized_region_count += 1;
                continue;
            };
            if region_chunks.is_empty() {
                self.adaptive_skipped_region_count += 1;
                self.adaptive_oversized_region_count += 1;
                continue;
            }

            for region_bounds in region_chunks {
                let mut adaptive_policy = self.policy.clone();
                adaptive_policy.grid_spacing = spacing;
                adaptive_policy.interface_band = (spacing * 2.5)
                    .max(self.policy.max_interface_error)
                    .min(self.policy.interface_band.max(spacing * 2.5));
                adaptive_policy.max_region_sample_count =
                    self.policy.adaptive_max_region_sample_count;
                let region = ReconditionedSampleRegion::from_sdf(
                    sdf,
                    region_bounds.clone(),
                    &adaptive_policy,
                );
                self.adaptive_regions.push(AdaptiveConditionedRegion {
                    source: request.source,
                    feature_ids: request.feature_ids.clone(),
                    bounds: region_bounds,
                    spacing,
                    generation: self.generation,
                    min_feature_size: Some(
                        request.requested_spacing
                            * self.policy.adaptive_samples_per_feature.max(1.0),
                    ),
                    region,
                });
            }
        }

        self.adaptive_stats()
    }

    fn adaptive_refresh_bounds_for_dirty_region(
        &self,
        dirty_region: &DirtyRegion,
        scope: AdaptiveRefreshScope,
    ) -> Option<SdfBounds> {
        let bounds = match scope {
            AdaptiveRefreshScope::FullRebuild => dirty_region.update_bounds(),
            AdaptiveRefreshScope::LocalEdit => {
                let spacing = self
                    .policy
                    .adaptive_spacing_for_dirty_region(dirty_region)?;
                let adaptive_halo = spacing * 4.5;
                dirty_region.bounds.expanded(adaptive_halo)
            }
        };
        bounds.intersection(&self.domain)
    }

    fn adaptive_sample_count_for_bounds(&self, bounds: &SdfBounds, spacing: f32) -> usize {
        bounded_sample_count(bounds, spacing)
    }

    fn adaptive_chunks_for_request(
        &self,
        update_bounds: &SdfBounds,
        requested_spacing: f32,
        max_region_count: usize,
    ) -> Option<(f32, Vec<SdfBounds>)> {
        if max_region_count == 0 || !requested_spacing.is_finite() || requested_spacing <= 0.0 {
            return None;
        }

        let mut spacing = requested_spacing.max(self.policy.adaptive_min_grid_spacing);
        let max_spacing = self.policy.grid_spacing * 0.995;
        for _ in 0..32 {
            if !self.policy.should_use_adaptive_spacing(spacing) || spacing > max_spacing {
                return None;
            }

            let region_bounds = sample_bounds_for_bounds(update_bounds, spacing)
                .expanded(spacing)
                .clamped_to(&self.domain);
            if region_bounds.is_valid() {
                let region_sample_count =
                    self.adaptive_sample_count_for_bounds(&region_bounds, spacing);
                if region_sample_count <= self.policy.adaptive_max_region_sample_count {
                    return Some((spacing, vec![region_bounds]));
                }
                if region_sample_count <= self.policy.adaptive_max_sparse_scan_sample_count {
                    return Some((spacing, vec![region_bounds]));
                }

                let chunks = self.split_adaptive_region_bounds_to_budget(
                    region_bounds,
                    spacing,
                    max_region_count,
                );
                if !chunks.is_empty() {
                    return Some((spacing, chunks));
                }
            }

            let next_spacing = (spacing * 1.25).min(max_spacing);
            if next_spacing <= spacing * 1.0001 {
                break;
            }
            spacing = next_spacing;
        }

        None
    }

    fn split_adaptive_region_bounds_to_budget(
        &self,
        bounds: SdfBounds,
        spacing: f32,
        max_region_count: usize,
    ) -> Vec<SdfBounds> {
        if max_region_count == 0 {
            return Vec::new();
        }
        let budget = self.policy.adaptive_max_region_sample_count;
        if self.adaptive_sample_count_for_bounds(&bounds, spacing) <= budget {
            return vec![bounds];
        }

        let mut pending = vec![bounds];
        let mut accepted = Vec::new();
        while let Some(bounds) = pending.pop() {
            if self.adaptive_sample_count_for_bounds(&bounds, spacing) <= budget {
                accepted.push(bounds);
                if accepted.len() + pending.len() > max_region_count {
                    return Vec::new();
                }
                continue;
            }
            if accepted.len() + pending.len() + 2 > max_region_count {
                return Vec::new();
            }
            let size = bounds.size();
            let axis = if size.x >= size.y && size.x >= size.z {
                0
            } else if size.y >= size.z {
                1
            } else {
                2
            };
            let axis_length = match axis {
                0 => size.x,
                1 => size.y,
                _ => size.z,
            };
            if axis_length <= spacing * 2.0 {
                return Vec::new();
            }

            let mid = match axis {
                0 => (bounds.min.x + bounds.max.x) * 0.5,
                1 => (bounds.min.y + bounds.max.y) * 0.5,
                _ => (bounds.min.z + bounds.max.z) * 0.5,
            };
            let mut lower_max = bounds.max;
            let mut upper_min = bounds.min;
            match axis {
                0 => {
                    lower_max.x = mid;
                    upper_min.x = mid;
                }
                1 => {
                    lower_max.y = mid;
                    upper_min.y = mid;
                }
                _ => {
                    lower_max.z = mid;
                    upper_min.z = mid;
                }
            }
            pending.push(SdfBounds::new(upper_min, bounds.max));
            pending.push(SdfBounds::new(bounds.min, lower_max));
        }

        accepted
    }

    pub fn block_indexes_intersecting(&self, bounds: &SdfBounds) -> Vec<BlockIndex> {
        let clipped = bounds.clamped_to(&self.domain);
        if !clipped.is_valid() || !clipped.intersects(&self.domain) {
            return Vec::new();
        }
        let min = self.block_index_for_point(clipped.min);
        let max = self.block_index_for_point(clipped.max);
        let max_counts = self.block_counts();
        let max_x = max_counts.x - 1;
        let max_y = max_counts.y - 1;
        let max_z = max_counts.z - 1;
        let mut indexes = Vec::new();
        for x in min.x.clamp(0, max_x)..=max.x.clamp(0, max_x) {
            for y in min.y.clamp(0, max_y)..=max.y.clamp(0, max_y) {
                for z in min.z.clamp(0, max_z)..=max.z.clamp(0, max_z) {
                    indexes.push(BlockIndex::new(x, y, z));
                }
            }
        }
        indexes
    }

    fn block_indexes_intersecting_dirty_regions(
        &self,
        dirty_regions: &[DirtyRegion],
    ) -> Vec<BlockIndex> {
        let mut unique_indexes = HashSet::new();
        for dirty_region in dirty_regions {
            unique_indexes.extend(self.block_indexes_intersecting(&dirty_region.update_bounds()));
        }
        let mut indexes: Vec<_> = unique_indexes.into_iter().collect();
        indexes.sort_by_key(|index| (index.x, index.y, index.z));
        indexes
    }

    fn should_skip_local_update_for_affected_blocks(
        &self,
        affected_indexes: &[BlockIndex],
    ) -> bool {
        if self.policy.allow_partial_local_updates || affected_indexes.is_empty() {
            return false;
        }

        let counts = self.block_counts();
        let total_block_count = (counts.x as usize)
            .saturating_mul(counts.y as usize)
            .saturating_mul(counts.z as usize)
            .max(1);
        let affected_fraction = affected_indexes.len() as f32 / total_block_count as f32;
        let max_local_fraction = self.policy.max_local_update_block_fraction.clamp(0.0, 1.0);
        affected_fraction >= max_local_fraction
    }

    fn all_block_indexes(&self) -> Vec<BlockIndex> {
        let counts = self.block_counts();
        let mut indexes = Vec::new();
        for x in 0..counts.x {
            for y in 0..counts.y {
                for z in 0..counts.z {
                    indexes.push(BlockIndex::new(x, y, z));
                }
            }
        }
        indexes
    }

    fn block_counts(&self) -> BlockIndex {
        let size = self.domain.size();
        BlockIndex::new(
            (size.x / self.block_size).ceil().max(1.0) as i32,
            (size.y / self.block_size).ceil().max(1.0) as i32,
            (size.z / self.block_size).ceil().max(1.0) as i32,
        )
    }

    fn block_index_for_point(&self, point: Vec3) -> BlockIndex {
        let local = (point - self.domain.min) / self.block_size;
        BlockIndex::new(
            local.x.floor() as i32,
            local.y.floor() as i32,
            local.z.floor() as i32,
        )
    }

    fn clamped_block_index_for_point(&self, point: Vec3) -> BlockIndex {
        let index = self.block_index_for_point(point);
        let counts = self.block_counts();
        BlockIndex::new(
            index.x.clamp(0, counts.x - 1),
            index.y.clamp(0, counts.y - 1),
            index.z.clamp(0, counts.z - 1),
        )
    }

    fn block_bounds(&self, index: BlockIndex) -> SdfBounds {
        let min = self.domain.min
            + Vec3::new(index.x as f32, index.y as f32, index.z as f32) * self.block_size;
        let max = (min + Vec3::splat(self.block_size)).min(self.domain.max);
        SdfBounds::new(min, max)
    }

    fn regenerate_block(
        &self,
        sdf: &dyn Sdf,
        index: BlockIndex,
        state: ConditionedCacheState,
    ) -> ConditionedBlock {
        self.regenerate_block_with_refinement(
            sdf,
            index,
            state,
            SurfaceRefinementMode::SurfaceAndDistanceBand,
        )
    }

    fn regenerate_block_with_refinement(
        &self,
        sdf: &dyn Sdf,
        index: BlockIndex,
        state: ConditionedCacheState,
        surface_refinement_mode: SurfaceRefinementMode,
    ) -> ConditionedBlock {
        let region_bounds = self
            .reconditioning_region_bounds(&[index])
            .expect("single regenerated block must produce region bounds");
        let region = ReconditionedSampleRegion::from_sdf(sdf, region_bounds, &self.policy);
        self.extract_block_from_region(sdf, index, state, &region, surface_refinement_mode)
    }

    fn regenerate_block_regions(
        &self,
        sdf: &dyn Sdf,
        indexes: &[BlockIndex],
        state: ConditionedCacheState,
    ) -> Vec<ConditionedBlock> {
        self.regenerate_block_regions_with_refinement(
            sdf,
            indexes,
            state,
            SurfaceRefinementMode::SurfaceAndDistanceBand,
        )
    }

    fn regenerate_block_regions_with_refinement(
        &self,
        sdf: &dyn Sdf,
        indexes: &[BlockIndex],
        state: ConditionedCacheState,
        surface_refinement_mode: SurfaceRefinementMode,
    ) -> Vec<ConditionedBlock> {
        let components = self.budgeted_block_components(indexes);
        let mut blocks: Vec<ConditionedBlock> = components
            .into_par_iter()
            .flat_map(|component| {
                let Some(region_bounds) = self.reconditioning_region_bounds(&component) else {
                    return Vec::new();
                };
                let region = ReconditionedSampleRegion::from_sdf(sdf, region_bounds, &self.policy);
                component
                    .into_iter()
                    .map(|index| {
                        self.extract_block_from_region(
                            sdf,
                            index,
                            state,
                            &region,
                            surface_refinement_mode,
                        )
                    })
                    .collect::<Vec<_>>()
            })
            .collect();
        blocks.sort_by_key(|block| (block.index.x, block.index.y, block.index.z));
        blocks
    }

    fn budgeted_block_components(&self, indexes: &[BlockIndex]) -> Vec<Vec<BlockIndex>> {
        let mut components = Vec::new();
        for component in connected_block_components(indexes) {
            self.split_component_to_region_budget(component, &mut components);
        }
        components.sort_by_key(|component| {
            component
                .first()
                .map(|index| (index.x, index.y, index.z))
                .unwrap_or((i32::MAX, i32::MAX, i32::MAX))
        });
        components
    }

    fn split_component_to_region_budget(
        &self,
        component: Vec<BlockIndex>,
        output: &mut Vec<Vec<BlockIndex>>,
    ) {
        if component.is_empty() {
            return;
        }
        let sample_count = self.reconditioning_region_sample_count(&component);
        let max_blocks_per_region = self.policy.max_blocks_per_region.max(1);
        if component.len() == 1
            || (component.len() <= max_blocks_per_region
                && sample_count <= self.policy.max_region_sample_count)
        {
            output.push(component);
            return;
        }

        let axis = widest_block_axis(&component);
        let mut sorted = component;
        sorted.sort_by_key(|index| match axis {
            0 => (index.x, index.y, index.z),
            1 => (index.y, index.x, index.z),
            _ => (index.z, index.x, index.y),
        });
        let mid = sorted.len() / 2;
        let upper = sorted.split_off(mid);
        for split in [sorted, upper] {
            for connected in connected_block_components(&split) {
                self.split_component_to_region_budget(connected, output);
            }
        }
    }

    fn reconditioning_region_bounds(&self, indexes: &[BlockIndex]) -> Option<SdfBounds> {
        let ghost_halo = self.policy.grid_spacing;
        indexes
            .iter()
            .copied()
            .map(|index| {
                sample_bounds_for_bounds(&self.block_bounds(index), self.policy.grid_spacing)
            })
            .reduce(|acc, bounds| acc.union(&bounds))
            .map(|bounds| bounds.expanded(ghost_halo))
    }

    fn reconditioning_region_sample_count(&self, indexes: &[BlockIndex]) -> usize {
        self.reconditioning_region_bounds(indexes)
            .map(|bounds| {
                let (nx, ny, nz) = grid_cell_counts(&bounds, self.policy.grid_spacing);
                (nx + 1) * (ny + 1) * (nz + 1)
            })
            .unwrap_or(0)
    }

    fn extract_block_from_region(
        &self,
        sdf: &dyn Sdf,
        index: BlockIndex,
        state: ConditionedCacheState,
        region: &ReconditionedSampleRegion,
        surface_refinement_mode: SurfaceRefinementMode,
    ) -> ConditionedBlock {
        let bounds = self.block_bounds(index);
        let (nx, ny, nz) = grid_cell_counts(&bounds, self.policy.grid_spacing);
        let sample_count = (nx + 1) * (ny + 1) * (nz + 1);
        let mut samples = Vec::with_capacity(sample_count);
        let mut raw_grid_samples = Vec::with_capacity(sample_count);
        let mut max_reconditioning_change = 0.0_f32;
        for ix in 0..=nx {
            for iy in 0..=ny {
                for iz in 0..=nz {
                    let point = bounds.min
                        + Vec3::new(ix as f32, iy as f32, iz as f32) * self.policy.grid_spacing;
                    let conditioned = region
                        .sample_conditioned(point, self.policy.grid_spacing)
                        .unwrap_or_else(|| sdf.distance(point));
                    let raw = region
                        .sample_raw(point, self.policy.grid_spacing)
                        .unwrap_or_else(|| sdf.distance(point));
                    let conditioned = cache_fidelity_preserving_conditioned_sample(
                        raw,
                        conditioned,
                        &self.policy,
                    );
                    max_reconditioning_change =
                        max_reconditioning_change.max((conditioned - raw).abs());
                    raw_grid_samples.push(raw);
                    samples.push(conditioned);
                }
            }
        }
        let min_distance = samples
            .iter()
            .copied()
            .fold(f32::INFINITY, |acc, value| acc.min(value));
        let max_distance = samples
            .iter()
            .copied()
            .fold(f32::NEG_INFINITY, |acc, value| acc.max(value));
        ConditionedBlock {
            index,
            bounds: bounds.clone(),
            generation: self.generation,
            state,
            samples,
            surface_refinement: build_surface_refinement_for_block(
                sdf,
                &bounds,
                &raw_grid_samples,
                self.policy.grid_spacing,
                surface_refinement_mode == SurfaceRefinementMode::SurfaceAndDistanceBand,
            ),
            min_distance,
            max_distance,
            reconditioning_anchor_count: region.report.anchor_count,
            reconditioning_projection_anchor_count: region.report.projection_anchor_count,
            reconditioning_iteration_count: region.report.iteration_count,
            max_reconditioning_change,
            reconditioning_narrow_band_sample_count: region.report.narrow_band_sample_count,
            reconditioning_used_narrow_band: region.report.used_narrow_band,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ReconditioningReport {
    pub sample_count: usize,
    pub narrow_band_sample_count: usize,
    pub anchor_count: usize,
    pub projection_anchor_count: usize,
    pub iteration_count: usize,
    pub max_change: f32,
    pub used_fast_sweeping: bool,
    pub used_narrow_band: bool,
}

#[derive(Clone, Debug, PartialEq)]
struct ReconditionedSampleRegion {
    bounds: SdfBounds,
    raw_samples: RegionSamples,
    samples: RegionSamples,
    report: ReconditioningReport,
}

impl ReconditionedSampleRegion {
    fn from_sdf(sdf: &dyn Sdf, bounds: SdfBounds, policy: &ConditioningPolicy) -> Self {
        let (nx, ny, nz) = grid_cell_counts(&bounds, policy.grid_spacing);
        let sample_count = (nx + 1) * (ny + 1) * (nz + 1);
        if sample_count > policy.max_region_sample_count {
            return Self::from_sdf_narrow_band(sdf, bounds, policy);
        }

        let raw_samples: Vec<f32> = sample_points(&bounds, policy.grid_spacing)
            .into_par_iter()
            .map(|point| sdf.distance(point))
            .collect();
        let (samples, mut report) = recondition_signed_distance_samples(
            &bounds,
            &raw_samples,
            policy.grid_spacing,
            policy.interface_band,
        );
        let samples = fidelity_preserving_dense_samples(&raw_samples, samples, policy);
        report.max_change = max_dense_sample_change(&raw_samples, &samples);
        Self {
            bounds,
            raw_samples: RegionSamples::Dense(raw_samples),
            samples: RegionSamples::Dense(samples),
            report,
        }
    }

    fn from_sdf_narrow_band(sdf: &dyn Sdf, bounds: SdfBounds, policy: &ConditioningPolicy) -> Self {
        let shape = SampleGridShape::new(&bounds, policy.grid_spacing);
        let raw_samples = sample_narrow_band_raw_values(
            sdf,
            &shape,
            policy.interface_band.max(policy.grid_spacing) + policy.grid_spacing * 2.0,
            2,
        );
        let (samples, mut report) = recondition_narrow_band_signed_distance_samples(
            &shape,
            &raw_samples,
            policy.grid_spacing,
            policy.interface_band,
        );
        let samples = fidelity_preserving_sparse_samples(&raw_samples, samples, policy);
        report.max_change = max_sparse_sample_change(&raw_samples, &samples);

        Self {
            bounds,
            raw_samples: RegionSamples::Sparse(raw_samples),
            samples: RegionSamples::Sparse(samples),
            report,
        }
    }

    fn sample_conditioned(&self, point: Vec3, spacing: f32) -> Option<f32> {
        self.sample_values(point, spacing, &self.samples)
    }

    fn sample_raw(&self, point: Vec3, spacing: f32) -> Option<f32> {
        self.sample_values(point, spacing, &self.raw_samples)
    }

    fn sample_values(&self, point: Vec3, spacing: f32, values: &RegionSamples) -> Option<f32> {
        if spacing <= 0.0 || values.is_empty() {
            return None;
        }
        let (nx, ny, nz) = grid_cell_counts(&self.bounds, spacing);
        let expected_sample_count = (nx + 1) * (ny + 1) * (nz + 1);
        if matches!(values, RegionSamples::Dense(samples) if samples.len() != expected_sample_count)
        {
            return None;
        }

        let grid_max = self.bounds.min + Vec3::new(nx as f32, ny as f32, nz as f32) * spacing;
        if point.x < self.bounds.min.x
            || point.y < self.bounds.min.y
            || point.z < self.bounds.min.z
            || point.x > grid_max.x
            || point.y > grid_max.y
            || point.z > grid_max.z
        {
            return None;
        }

        let local = (point - self.bounds.min) / spacing;
        let fx = local.x.clamp(0.0, nx as f32);
        let fy = local.y.clamp(0.0, ny as f32);
        let fz = local.z.clamp(0.0, nz as f32);
        let ix0 = fx.floor() as usize;
        let iy0 = fy.floor() as usize;
        let iz0 = fz.floor() as usize;
        let ix1 = (ix0 + 1).min(nx);
        let iy1 = (iy0 + 1).min(ny);
        let iz1 = (iz0 + 1).min(nz);
        let tx = if ix0 == ix1 { 0.0 } else { fx - ix0 as f32 };
        let ty = if iy0 == iy1 { 0.0 } else { fy - iy0 as f32 };
        let tz = if iz0 == iz1 { 0.0 } else { fz - iz0 as f32 };
        let sample = |ix: usize, iy: usize, iz: usize| -> Option<f32> {
            values.get(sample_grid_index(ix, iy, iz, ny, nz))
        };

        let c00 = lerp(sample(ix0, iy0, iz0)?, sample(ix1, iy0, iz0)?, tx);
        let c10 = lerp(sample(ix0, iy1, iz0)?, sample(ix1, iy1, iz0)?, tx);
        let c01 = lerp(sample(ix0, iy0, iz1)?, sample(ix1, iy0, iz1)?, tx);
        let c11 = lerp(sample(ix0, iy1, iz1)?, sample(ix1, iy1, iz1)?, tx);
        let c0 = lerp(c00, c10, ty);
        let c1 = lerp(c01, c11, ty);
        Some(lerp(c0, c1, tz))
    }
}

#[derive(Clone, Debug, PartialEq)]
enum RegionSamples {
    Dense(Vec<f32>),
    Sparse(HashMap<usize, f32>),
}

impl RegionSamples {
    fn is_empty(&self) -> bool {
        match self {
            Self::Dense(samples) => samples.is_empty(),
            Self::Sparse(samples) => samples.is_empty(),
        }
    }

    fn is_sparse(&self) -> bool {
        matches!(self, Self::Sparse(_))
    }

    fn stored_value_count(&self) -> usize {
        match self {
            Self::Dense(samples) => samples.len(),
            Self::Sparse(samples) => samples.len(),
        }
    }

    fn get(&self, index: usize) -> Option<f32> {
        match self {
            Self::Dense(samples) => samples.get(index).copied(),
            Self::Sparse(samples) => samples.get(&index).copied(),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ConditioningPolicy {
    pub grid_spacing: f32,
    pub interface_band: f32,
    pub max_gradient_abs_error_p95: f32,
    pub max_interface_error: f32,
    pub max_region_sample_count: usize,
    pub max_blocks_per_region: usize,
    pub local_validation_mode: LocalValidationMode,
    pub adaptive_enabled: bool,
    pub adaptive_samples_per_feature: f32,
    pub adaptive_min_grid_spacing: f32,
    pub adaptive_max_region_sample_count: usize,
    pub adaptive_max_sparse_scan_sample_count: usize,
    pub adaptive_max_regions: usize,
    pub allow_partial_local_updates: bool,
    pub max_local_update_block_fraction: f32,
}

impl Default for ConditioningPolicy {
    fn default() -> Self {
        Self {
            grid_spacing: 1.0,
            interface_band: 2.5,
            max_gradient_abs_error_p95: 0.15,
            max_interface_error: 1.0e-4,
            max_region_sample_count: Self::DEFAULT_MAX_REGION_SAMPLE_COUNT,
            max_blocks_per_region: Self::DEFAULT_MAX_BLOCKS_PER_REGION,
            local_validation_mode: LocalValidationMode::LocalCacheQuality,
            adaptive_enabled: true,
            adaptive_samples_per_feature: Self::DEFAULT_ADAPTIVE_SAMPLES_PER_FEATURE,
            adaptive_min_grid_spacing: Self::DEFAULT_ADAPTIVE_MIN_GRID_SPACING,
            adaptive_max_region_sample_count: Self::DEFAULT_ADAPTIVE_MAX_REGION_SAMPLE_COUNT,
            adaptive_max_sparse_scan_sample_count:
                Self::DEFAULT_ADAPTIVE_MAX_SPARSE_SCAN_SAMPLE_COUNT,
            adaptive_max_regions: Self::DEFAULT_ADAPTIVE_MAX_REGIONS,
            allow_partial_local_updates: true,
            max_local_update_block_fraction: 0.65,
        }
    }
}

impl ConditioningPolicy {
    pub const DEFAULT_MAX_REGION_SAMPLE_COUNT: usize = 262_144;
    pub const DEFAULT_MAX_BLOCKS_PER_REGION: usize = 8;
    pub const DEFAULT_ADAPTIVE_SAMPLES_PER_FEATURE: f32 = 2.0;
    pub const DEFAULT_ADAPTIVE_MIN_GRID_SPACING: f32 = 0.25;
    pub const DEFAULT_ADAPTIVE_MAX_REGION_SAMPLE_COUNT: usize = 65_536;
    pub const DEFAULT_ADAPTIVE_MAX_SPARSE_SCAN_SAMPLE_COUNT: usize = 1_000_000;
    pub const DEFAULT_ADAPTIVE_MAX_REGIONS: usize = 8;

    pub fn adaptive_spacing_for_dirty_region(&self, dirty_region: &DirtyRegion) -> Option<f32> {
        let spacing = dirty_region.recommended_grid_spacing?;
        if self.should_use_adaptive_spacing(spacing) {
            Some(spacing.max(self.adaptive_min_grid_spacing.max(1.0e-4)))
        } else {
            None
        }
    }

    pub fn should_use_adaptive_spacing(&self, spacing: f32) -> bool {
        self.adaptive_enabled
            && spacing.is_finite()
            && spacing > 0.0
            && spacing < self.grid_spacing * 0.999
            && self.adaptive_max_region_sample_count > 0
            && self.adaptive_max_regions > 0
    }
}

fn runtime_conditioning_policy_for_bounds(
    support_bounds: &SdfBounds,
    max_initial_samples: usize,
) -> ConditioningPolicy {
    let mut policy = ConditioningPolicy::default();
    let max_initial_samples = max_initial_samples.max(1);
    let mut spacing = policy.grid_spacing.max(1.0e-3);

    for _ in 0..96 {
        policy.grid_spacing = spacing;
        policy.interface_band = spacing * 2.5;
        let domain = support_bounds.expanded(runtime_conditioning_domain_halo(&policy));
        if bounded_sample_count(&domain, spacing) <= max_initial_samples {
            break;
        }
        spacing *= 1.25;
    }

    policy.max_region_sample_count = policy
        .max_region_sample_count
        .min(max_initial_samples.max(64));
    policy.max_blocks_per_region = 1;
    policy.adaptive_max_sparse_scan_sample_count = policy
        .adaptive_max_sparse_scan_sample_count
        .min((max_initial_samples / 4).max(policy.adaptive_max_region_sample_count));
    policy.allow_partial_local_updates = false;
    policy.max_local_update_block_fraction = 0.25;
    policy
}

fn runtime_conditioning_domain_halo(policy: &ConditioningPolicy) -> f32 {
    policy.interface_band.max(policy.grid_spacing) + policy.grid_spacing * 2.0
}

fn bounded_sample_count(bounds: &SdfBounds, spacing: f32) -> usize {
    if !bounds.is_valid() || spacing <= 0.0 || !spacing.is_finite() {
        return usize::MAX;
    }
    let (nx, ny, nz) = grid_cell_counts(bounds, spacing);
    nx.saturating_add(1)
        .saturating_mul(ny.saturating_add(1))
        .saturating_mul(nz.saturating_add(1))
}

#[derive(Clone, Debug, PartialEq)]
pub struct ConditioningDiagnostics {
    pub cache_state: ConditionedCacheState,
    pub update_mode: ConditioningUpdateMode,
    pub validated_against_full: bool,
    pub grid_point_count: usize,
    pub dirty_sample_count: usize,
    pub halo_sample_count: usize,
    pub conditioned_sample_count: usize,
    pub changed_near_interface_count: usize,
    pub max_abs_error_inside_update: f32,
    pub max_abs_error_near_interface: f32,
    pub sign_mismatch_count_near_interface: usize,
    pub unexpected_sign_flip_count_outside_update: usize,
    pub gradient_abs_error_p50: f32,
    pub gradient_abs_error_p95: f32,
    pub gradient_abs_error_max: f32,
    pub confidence: f32,
    pub publication_confidence: f32,
    pub quality_confidence: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ConditionedCacheQualityDiagnostics {
    pub grid_point_count: usize,
    pub near_interface_sample_count: usize,
    pub gradient_sample_count: usize,
    pub projection_sample_count: usize,
    pub sign_mismatch_count_near_interface: usize,
    pub max_surface_residual: f32,
    pub max_projection_residual: f32,
    pub distance_abs_error_p50: f32,
    pub distance_abs_error_p95: f32,
    pub distance_abs_error_max: f32,
    pub gradient_abs_error_p50: f32,
    pub gradient_abs_error_p95: f32,
    pub gradient_abs_error_max: f32,
    pub adaptive_region_count: usize,
    pub adaptive_sample_count: usize,
    pub adaptive_min_grid_spacing: Option<f32>,
    pub adaptive_feature_id_count: usize,
    pub adaptive_requested_feature_id_count: usize,
    pub adaptive_feature_coverage: f32,
    pub confidence: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ConditionedEquivalenceDiagnostics {
    pub grid_point_count: usize,
    pub compared_sample_count: usize,
    pub near_interface_sample_count: usize,
    pub local_cache_miss_count: usize,
    pub full_cache_miss_count: usize,
    pub sign_mismatch_count_near_interface: usize,
    pub distance_delta_p50: f32,
    pub distance_delta_p95: f32,
    pub distance_delta_max: f32,
    pub gradient_delta_p50: f32,
    pub gradient_delta_p95: f32,
    pub gradient_delta_max: f32,
    pub spacing: f32,
    pub interface_band: f32,
    pub confidence: f32,
}

impl ConditionedEquivalenceDiagnostics {
    pub fn accepted(&self, max_distance_delta_p95: f32, max_gradient_delta_p95: f32) -> bool {
        self.compared_sample_count > 0
            && self.sign_mismatch_count_near_interface == 0
            && self.distance_delta_p95 <= max_distance_delta_p95
            && self.gradient_delta_p95 <= max_gradient_delta_p95
    }
}

impl ConditioningDiagnostics {
    pub fn accepted(&self, policy: &ConditioningPolicy) -> bool {
        if self.sign_mismatch_count_near_interface != 0 {
            return false;
        }

        if self.validated_against_full {
            self.cache_state == ConditionedCacheState::Ready
                && self.max_abs_error_near_interface <= policy.max_interface_error
                && self.gradient_abs_error_p95 <= policy.max_gradient_abs_error_p95
                && self.unexpected_sign_flip_count_outside_update == 0
        } else if policy.allow_partial_local_updates {
            self.confidence > 0.0
        } else {
            self.cache_state == ConditionedCacheState::Ready
        }
    }

    pub fn full_rebuild(block_count: usize) -> Self {
        Self {
            cache_state: ConditionedCacheState::Ready,
            update_mode: ConditioningUpdateMode::FullRebuild,
            validated_against_full: true,
            grid_point_count: 0,
            dirty_sample_count: 0,
            halo_sample_count: 0,
            conditioned_sample_count: 0,
            changed_near_interface_count: 0,
            max_abs_error_inside_update: 0.0,
            max_abs_error_near_interface: 0.0,
            sign_mismatch_count_near_interface: 0,
            unexpected_sign_flip_count_outside_update: 0,
            gradient_abs_error_p50: 0.0,
            gradient_abs_error_p95: 0.0,
            gradient_abs_error_max: 0.0,
            confidence: if block_count > 0 { 1.0 } else { 0.0 },
            publication_confidence: if block_count > 0 { 1.0 } else { 0.0 },
            quality_confidence: if block_count > 0 { 1.0 } else { 0.0 },
        }
    }
}

/// Compare a local dirty-region update against a full recomputation.
///
/// This is a backend diagnostic harness, not a reinitialization solver. It
/// establishes the kernel contract for local update coverage, interface drift,
/// sign consistency, and gradient quality before production conditioning is
/// added.
pub fn compare_local_update_to_full(
    before: &dyn Sdf,
    after: &dyn Sdf,
    domain: &SdfBounds,
    dirty_region: &DirtyRegion,
    policy: &ConditioningPolicy,
) -> ConditioningDiagnostics {
    compare_local_update_regions_to_full(
        before,
        after,
        domain,
        std::slice::from_ref(dirty_region),
        policy,
    )
}

pub fn compare_local_update_regions_to_full(
    before: &dyn Sdf,
    after: &dyn Sdf,
    domain: &SdfBounds,
    dirty_regions: &[DirtyRegion],
    policy: &ConditioningPolicy,
) -> ConditioningDiagnostics {
    assert!(
        domain.is_valid(),
        "conditioning domain bounds must be valid"
    );
    assert!(
        !dirty_regions.is_empty(),
        "conditioning requires at least one dirty region"
    );
    for dirty_region in dirty_regions {
        assert!(
            dirty_region.bounds.is_valid(),
            "dirty-region bounds must be valid"
        );
    }
    assert!(
        policy.grid_spacing > 0.0,
        "conditioning grid spacing must be positive"
    );

    let update_bounds: Vec<_> = dirty_regions
        .iter()
        .map(DirtyRegion::update_bounds)
        .collect();
    let mut grid_point_count = 0_usize;
    let mut dirty_sample_count = 0_usize;
    let mut conditioned_sample_count = 0_usize;
    let mut changed_near_interface_count = 0_usize;
    let mut max_abs_error_inside_update = 0.0_f32;
    let mut max_abs_error_near_interface = 0.0_f32;
    let mut sign_mismatch_count_near_interface = 0_usize;
    let mut unexpected_sign_flip_count_outside_update = 0_usize;
    let mut gradient_errors = Vec::new();

    for point in sample_points(domain, policy.grid_spacing) {
        grid_point_count += 1;
        let before_d = before.distance(point);
        let full_after_d = after.distance(point);
        let inside_dirty = dirty_regions
            .iter()
            .any(|dirty_region| dirty_region.bounds.contains(point));
        let inside_update = update_bounds
            .iter()
            .any(|update_bounds| update_bounds.contains(point));
        let local_after_d = if inside_update {
            full_after_d
        } else {
            before_d
        };

        if inside_dirty {
            dirty_sample_count += 1;
        }
        if inside_update {
            conditioned_sample_count += 1;
            max_abs_error_inside_update =
                max_abs_error_inside_update.max((local_after_d - full_after_d).abs());
        }

        let near_interface = full_after_d.abs() <= policy.interface_band
            || local_after_d.abs() <= policy.interface_band;
        if near_interface {
            max_abs_error_near_interface =
                max_abs_error_near_interface.max((local_after_d - full_after_d).abs());
            if sign(full_after_d) != sign(local_after_d) {
                sign_mismatch_count_near_interface += 1;
            }
            if (full_after_d - before_d).abs() > 1.0e-6 {
                changed_near_interface_count += 1;
            }
            gradient_errors.push(
                (finite_difference_gradient(after, point, policy.grid_spacing).length() - 1.0)
                    .abs(),
            );
        }

        if !inside_update && sign(full_after_d) != sign(before_d) {
            unexpected_sign_flip_count_outside_update += 1;
        }
    }

    let halo_sample_count = conditioned_sample_count.saturating_sub(dirty_sample_count);
    let gradient_abs_error_p50 = percentile(&mut gradient_errors.clone(), 50.0);
    let gradient_abs_error_p95 = percentile(&mut gradient_errors.clone(), 95.0);
    let gradient_abs_error_max = gradient_errors
        .iter()
        .copied()
        .fold(0.0_f32, |acc, value| acc.max(value));
    let mut confidence = 1.0_f32;
    if sign_mismatch_count_near_interface > 0 || unexpected_sign_flip_count_outside_update > 0 {
        confidence = 0.0;
    } else if gradient_abs_error_p95 > policy.max_gradient_abs_error_p95 {
        confidence = (policy.max_gradient_abs_error_p95 / gradient_abs_error_p95).clamp(0.0, 1.0);
    }
    if max_abs_error_near_interface > policy.max_interface_error {
        confidence = confidence
            .min((policy.max_interface_error / max_abs_error_near_interface).clamp(0.0, 1.0));
    }

    let cache_state = if confidence >= 1.0 {
        ConditionedCacheState::Ready
    } else if confidence > 0.0 {
        ConditionedCacheState::Partial
    } else {
        ConditionedCacheState::Rejected
    };

    ConditioningDiagnostics {
        cache_state,
        update_mode: ConditioningUpdateMode::LocalIncremental,
        validated_against_full: true,
        grid_point_count,
        dirty_sample_count,
        halo_sample_count,
        conditioned_sample_count,
        changed_near_interface_count,
        max_abs_error_inside_update,
        max_abs_error_near_interface,
        sign_mismatch_count_near_interface,
        unexpected_sign_flip_count_outside_update,
        gradient_abs_error_p50,
        gradient_abs_error_p95,
        gradient_abs_error_max,
        confidence,
        publication_confidence: confidence,
        quality_confidence: confidence,
    }
}

pub fn diagnose_conditioned_equivalence(
    local: &ConditionedGeometryKernel,
    full: &ConditionedGeometryKernel,
    bounds: Option<&SdfBounds>,
    spacing: Option<f32>,
) -> ConditionedEquivalenceDiagnostics {
    let comparison_bounds = bounds
        .cloned()
        .or_else(|| {
            local
                .conditioning
                .domain()
                .intersection(full.conditioning.domain())
        })
        .unwrap_or_else(|| local.conditioning.domain().clone());
    let spacing = spacing
        .filter(|spacing| spacing.is_finite() && *spacing > 0.0)
        .unwrap_or_else(|| {
            local
                .conditioning
                .policy
                .grid_spacing
                .max(full.conditioning.policy.grid_spacing)
                .max(1.0e-4)
        });
    let interface_band = local
        .conditioning
        .policy
        .interface_band
        .max(full.conditioning.policy.interface_band)
        .max(spacing);
    let sign_tolerance = spacing.max(1.0e-6) * 0.75;
    let mut grid_point_count = 0_usize;
    let mut compared_sample_count = 0_usize;
    let mut near_interface_sample_count = 0_usize;
    let mut local_cache_miss_count = 0_usize;
    let mut full_cache_miss_count = 0_usize;
    let mut sign_mismatch_count_near_interface = 0_usize;
    let mut distance_deltas = Vec::new();
    let mut gradient_deltas = Vec::new();

    for point in sample_points(&comparison_bounds, spacing) {
        grid_point_count += 1;
        if local.conditioned_distance(point).is_none() {
            local_cache_miss_count += 1;
        }
        if full.conditioned_distance(point).is_none() {
            full_cache_miss_count += 1;
        }

        let local_distance = local.distance(point);
        let full_distance = full.distance(point);
        if !local_distance.is_finite() || !full_distance.is_finite() {
            continue;
        }

        compared_sample_count += 1;
        let near_interface =
            local_distance.abs() <= interface_band || full_distance.abs() <= interface_band;
        if near_interface {
            near_interface_sample_count += 1;
            if strong_sign_mismatch(local_distance, full_distance, sign_tolerance) {
                sign_mismatch_count_near_interface += 1;
            }
            distance_deltas.push((local_distance - full_distance).abs());
            let local_gradient = local.gradient(point, spacing);
            let full_gradient = full.gradient(point, spacing);
            if local_gradient.is_finite() && full_gradient.is_finite() {
                gradient_deltas.push((local_gradient - full_gradient).length());
            }
        }
    }

    let distance_delta_p50 = percentile(&mut distance_deltas.clone(), 50.0);
    let distance_delta_p95 = percentile(&mut distance_deltas.clone(), 95.0);
    let distance_delta_max = distance_deltas
        .iter()
        .copied()
        .fold(0.0_f32, |acc, value| acc.max(value));
    let gradient_delta_p50 = percentile(&mut gradient_deltas.clone(), 50.0);
    let gradient_delta_p95 = percentile(&mut gradient_deltas.clone(), 95.0);
    let gradient_delta_max = gradient_deltas
        .iter()
        .copied()
        .fold(0.0_f32, |acc, value| acc.max(value));

    let max_distance_delta_p95 = spacing * 1.5;
    let max_gradient_delta_p95 = 0.35;
    let mut confidence: f32 = if compared_sample_count > 0 { 1.0 } else { 0.0 };
    if sign_mismatch_count_near_interface > 0 {
        confidence = 0.0;
    }
    if distance_delta_p95 > max_distance_delta_p95 {
        confidence = confidence.min((max_distance_delta_p95 / distance_delta_p95).clamp(0.0, 1.0));
    }
    if gradient_delta_p95 > max_gradient_delta_p95 {
        confidence = confidence.min((max_gradient_delta_p95 / gradient_delta_p95).clamp(0.0, 1.0));
    }

    ConditionedEquivalenceDiagnostics {
        grid_point_count,
        compared_sample_count,
        near_interface_sample_count,
        local_cache_miss_count,
        full_cache_miss_count,
        sign_mismatch_count_near_interface,
        distance_delta_p50,
        distance_delta_p95,
        distance_delta_max,
        gradient_delta_p50,
        gradient_delta_p95,
        gradient_delta_max,
        spacing,
        interface_band,
        confidence,
    }
}

#[derive(Clone, Debug, PartialEq)]
struct SampleGridShape {
    bounds: SdfBounds,
    spacing: f32,
    nx: usize,
    ny: usize,
    nz: usize,
    sample_count: usize,
}

impl SampleGridShape {
    fn new(bounds: &SdfBounds, spacing: f32) -> Self {
        let (nx, ny, nz) = grid_cell_counts(bounds, spacing);
        Self {
            bounds: bounds.clone(),
            spacing,
            nx,
            ny,
            nz,
            sample_count: (nx + 1) * (ny + 1) * (nz + 1),
        }
    }

    fn index(&self, ix: usize, iy: usize, iz: usize) -> usize {
        sample_grid_index(ix, iy, iz, self.ny, self.nz)
    }

    fn coords(&self, index: usize) -> (usize, usize, usize) {
        let yz_count = (self.ny + 1) * (self.nz + 1);
        let ix = index / yz_count;
        let rem = index % yz_count;
        let iy = rem / (self.nz + 1);
        let iz = rem % (self.nz + 1);
        (ix, iy, iz)
    }

    fn point(&self, ix: usize, iy: usize, iz: usize) -> Vec3 {
        self.bounds.min + Vec3::new(ix as f32, iy as f32, iz as f32) * self.spacing
    }

    fn point_for_index(&self, index: usize) -> Vec3 {
        let (ix, iy, iz) = self.coords(index);
        self.point(ix, iy, iz)
    }

    fn axis_neighbors(&self, index: usize) -> Vec<usize> {
        let (ix, iy, iz) = self.coords(index);
        let mut neighbors = Vec::with_capacity(6);
        if ix > 0 {
            neighbors.push(self.index(ix - 1, iy, iz));
        }
        if ix < self.nx {
            neighbors.push(self.index(ix + 1, iy, iz));
        }
        if iy > 0 {
            neighbors.push(self.index(ix, iy - 1, iz));
        }
        if iy < self.ny {
            neighbors.push(self.index(ix, iy + 1, iz));
        }
        if iz > 0 {
            neighbors.push(self.index(ix, iy, iz - 1));
        }
        if iz < self.nz {
            neighbors.push(self.index(ix, iy, iz + 1));
        }
        neighbors
    }
}

fn sample_narrow_band_raw_values(
    sdf: &dyn Sdf,
    shape: &SampleGridShape,
    band_radius: f32,
    halo_cells: usize,
) -> HashMap<usize, f32> {
    let mut active = HashSet::new();
    let mut raw_samples = HashMap::new();

    for ix in 0..=shape.nx {
        for iy in 0..=shape.ny {
            for iz in 0..=shape.nz {
                let index = shape.index(ix, iy, iz);
                let value = sdf.distance(shape.point(ix, iy, iz));
                if value.is_finite() && value.abs() <= band_radius {
                    insert_sparse_sample(sdf, shape, index, &mut active, &mut raw_samples);
                }

                if ix < shape.nx {
                    let neighbor = shape.index(ix + 1, iy, iz);
                    let neighbor_value = sdf.distance(shape.point(ix + 1, iy, iz));
                    if crosses_interface(value, neighbor_value) {
                        insert_sparse_sample(sdf, shape, index, &mut active, &mut raw_samples);
                        insert_sparse_sample(sdf, shape, neighbor, &mut active, &mut raw_samples);
                    }
                }
                if iy < shape.ny {
                    let neighbor = shape.index(ix, iy + 1, iz);
                    let neighbor_value = sdf.distance(shape.point(ix, iy + 1, iz));
                    if crosses_interface(value, neighbor_value) {
                        insert_sparse_sample(sdf, shape, index, &mut active, &mut raw_samples);
                        insert_sparse_sample(sdf, shape, neighbor, &mut active, &mut raw_samples);
                    }
                }
                if iz < shape.nz {
                    let neighbor = shape.index(ix, iy, iz + 1);
                    let neighbor_value = sdf.distance(shape.point(ix, iy, iz + 1));
                    if crosses_interface(value, neighbor_value) {
                        insert_sparse_sample(sdf, shape, index, &mut active, &mut raw_samples);
                        insert_sparse_sample(sdf, shape, neighbor, &mut active, &mut raw_samples);
                    }
                }
            }
        }
    }

    for _ in 0..halo_cells {
        let current: Vec<_> = active.iter().copied().collect();
        for index in current {
            for neighbor in shape.axis_neighbors(index) {
                insert_sparse_sample(sdf, shape, neighbor, &mut active, &mut raw_samples);
            }
        }
    }

    raw_samples
}

fn insert_sparse_sample(
    sdf: &dyn Sdf,
    shape: &SampleGridShape,
    index: usize,
    active: &mut HashSet<usize>,
    raw_samples: &mut HashMap<usize, f32>,
) {
    active.insert(index);
    raw_samples
        .entry(index)
        .or_insert_with(|| sdf.distance(shape.point_for_index(index)));
}

fn crosses_interface(a: f32, b: f32) -> bool {
    if !a.is_finite() || !b.is_finite() {
        return false;
    }
    let a_sign = sign(a);
    let b_sign = sign(b);
    a_sign == 0 || b_sign == 0 || a_sign != b_sign
}

fn recondition_narrow_band_signed_distance_samples(
    shape: &SampleGridShape,
    raw_samples: &HashMap<usize, f32>,
    spacing: f32,
    interface_band: f32,
) -> (HashMap<usize, f32>, ReconditioningReport) {
    let mut distances: HashMap<usize, f32> = raw_samples
        .keys()
        .copied()
        .map(|index| (index, f32::INFINITY))
        .collect();
    let mut fixed = HashSet::new();
    let mut anchor_count = 0_usize;
    let mut projection_anchor_count = 0_usize;
    let anchor_band = interface_band.max(spacing);

    let mut set_anchor = |index: usize, distance: f32| -> bool {
        if !raw_samples.contains_key(&index) || !distance.is_finite() || distance > anchor_band {
            return false;
        }
        let distance = distance.max(0.0);
        let inserted = fixed.insert(index);
        if inserted {
            anchor_count += 1;
        }
        let entry = distances.entry(index).or_insert(f32::INFINITY);
        *entry = (*entry).min(distance);
        inserted
    };

    for (&index, &value) in raw_samples {
        if value.is_finite() && value.abs() <= f32::EPSILON {
            set_anchor(index, 0.0);
        }
    }

    let active_indices: Vec<_> = raw_samples.keys().copied().collect();
    for &index in &active_indices {
        if let Some(distance) =
            projection_corrected_sparse_distance(shape, raw_samples, index, anchor_band)
        {
            if set_anchor(index, distance) {
                projection_anchor_count += 1;
            }
        }
    }

    for &index in &active_indices {
        let (ix, iy, iz) = shape.coords(index);
        if ix < shape.nx {
            let neighbor = shape.index(ix + 1, iy, iz);
            add_interface_edge_anchor_from_values(
                index,
                neighbor,
                raw_samples.get(&index).copied(),
                raw_samples.get(&neighbor).copied(),
                spacing,
                &mut set_anchor,
            );
        }
        if iy < shape.ny {
            let neighbor = shape.index(ix, iy + 1, iz);
            add_interface_edge_anchor_from_values(
                index,
                neighbor,
                raw_samples.get(&index).copied(),
                raw_samples.get(&neighbor).copied(),
                spacing,
                &mut set_anchor,
            );
        }
        if iz < shape.nz {
            let neighbor = shape.index(ix, iy, iz + 1);
            add_interface_edge_anchor_from_values(
                index,
                neighbor,
                raw_samples.get(&index).copied(),
                raw_samples.get(&neighbor).copied(),
                spacing,
                &mut set_anchor,
            );
        }
    }

    if anchor_count == 0 {
        return (
            raw_samples.clone(),
            ReconditioningReport {
                sample_count: shape.sample_count,
                narrow_band_sample_count: raw_samples.len(),
                anchor_count,
                projection_anchor_count,
                iteration_count: 0,
                max_change: 0.0,
                used_fast_sweeping: false,
                used_narrow_band: true,
            },
        );
    }

    let max_iterations = ((shape.nx + shape.ny + shape.nz).max(1) * 2).clamp(8, 96);
    let tolerance = spacing * 1.0e-4;
    let mut iteration_count = 0_usize;
    let mut last_max_change = 0.0_f32;
    let orders = [
        (true, true, true),
        (false, true, true),
        (true, false, true),
        (false, false, true),
        (true, true, false),
        (false, true, false),
        (true, false, false),
        (false, false, false),
    ];
    let ordered_sweeps: Vec<Vec<usize>> = orders
        .into_iter()
        .map(|(x_forward, y_forward, z_forward)| {
            let mut ordered = active_indices.clone();
            ordered.sort_by_key(|index| {
                let (ix, iy, iz) = shape.coords(*index);
                (
                    sweep_coord(ix, shape.nx, x_forward),
                    sweep_coord(iy, shape.ny, y_forward),
                    sweep_coord(iz, shape.nz, z_forward),
                )
            });
            ordered
        })
        .collect();

    for iteration in 0..max_iterations {
        let mut max_change = 0.0_f32;
        for ordered in &ordered_sweeps {
            for &index in ordered {
                if fixed.contains(&index) {
                    continue;
                }
                let (ix, iy, iz) = shape.coords(index);
                let candidate =
                    fast_sweeping_update_sparse_second_order(&distances, shape, ix, iy, iz);
                let current = distances.get(&index).copied().unwrap_or(f32::INFINITY);
                if candidate.is_finite() && candidate < current {
                    let change = (current - candidate).abs();
                    max_change = max_change.max(change);
                    distances.insert(index, candidate);
                }
            }
        }
        iteration_count = iteration + 1;
        last_max_change = max_change;
        if max_change <= tolerance {
            break;
        }
    }

    let samples = raw_samples
        .iter()
        .map(|(&index, &raw)| {
            let distance = distances.get(&index).copied().unwrap_or(f32::INFINITY);
            let sample = if !raw.is_finite() || !distance.is_finite() {
                raw
            } else {
                match sign(raw) {
                    -1 => -distance,
                    1 => distance,
                    _ => 0.0,
                }
            };
            (index, sample)
        })
        .collect();

    (
        samples,
        ReconditioningReport {
            sample_count: shape.sample_count,
            narrow_band_sample_count: raw_samples.len(),
            anchor_count,
            projection_anchor_count,
            iteration_count,
            max_change: last_max_change,
            used_fast_sweeping: true,
            used_narrow_band: true,
        },
    )
}

fn sweep_coord(coord: usize, max: usize, forward: bool) -> usize {
    if forward { coord } else { max - coord }
}

pub fn recondition_signed_distance_samples(
    bounds: &SdfBounds,
    raw_samples: &[f32],
    spacing: f32,
    interface_band: f32,
) -> (Vec<f32>, ReconditioningReport) {
    assert!(bounds.is_valid(), "reconditioning bounds must be valid");
    assert!(spacing > 0.0, "reconditioning spacing must be positive");

    let (nx, ny, nz) = grid_cell_counts(bounds, spacing);
    let sample_count = (nx + 1) * (ny + 1) * (nz + 1);
    assert_eq!(
        raw_samples.len(),
        sample_count,
        "raw sample count must match the reconditioning grid"
    );

    let mut distances = vec![f32::INFINITY; sample_count];
    let mut fixed = vec![false; sample_count];
    let mut anchor_count = 0_usize;
    let mut projection_anchor_count = 0_usize;
    let anchor_band = interface_band.max(spacing);

    let mut set_anchor = |index: usize, distance: f32| -> bool {
        if !distance.is_finite() || distance > anchor_band {
            return false;
        }
        let distance = distance.max(0.0);
        let inserted = !fixed[index];
        if inserted {
            anchor_count += 1;
        }
        fixed[index] = true;
        distances[index] = distances[index].min(distance);
        inserted
    };

    for ix in 0..=nx {
        for iy in 0..=ny {
            for iz in 0..=nz {
                let index = sample_grid_index(ix, iy, iz, ny, nz);
                let value = raw_samples[index];
                if value.is_finite() && value.abs() <= f32::EPSILON {
                    set_anchor(index, 0.0);
                }
            }
        }
    }

    for ix in 0..=nx {
        for iy in 0..=ny {
            for iz in 0..=nz {
                let index = sample_grid_index(ix, iy, iz, ny, nz);
                if let Some(distance) = projection_corrected_dense_distance(
                    raw_samples,
                    index,
                    ix,
                    iy,
                    iz,
                    nx,
                    ny,
                    nz,
                    spacing,
                    anchor_band,
                ) {
                    if set_anchor(index, distance) {
                        projection_anchor_count += 1;
                    }
                }
            }
        }
    }

    for ix in 0..=nx {
        for iy in 0..=ny {
            for iz in 0..=nz {
                let index = sample_grid_index(ix, iy, iz, ny, nz);
                if ix < nx {
                    add_interface_edge_anchor(
                        index,
                        sample_grid_index(ix + 1, iy, iz, ny, nz),
                        raw_samples,
                        spacing,
                        &mut set_anchor,
                    );
                }
                if iy < ny {
                    add_interface_edge_anchor(
                        index,
                        sample_grid_index(ix, iy + 1, iz, ny, nz),
                        raw_samples,
                        spacing,
                        &mut set_anchor,
                    );
                }
                if iz < nz {
                    add_interface_edge_anchor(
                        index,
                        sample_grid_index(ix, iy, iz + 1, ny, nz),
                        raw_samples,
                        spacing,
                        &mut set_anchor,
                    );
                }
            }
        }
    }

    if anchor_count == 0 {
        return (
            raw_samples.to_vec(),
            ReconditioningReport {
                sample_count,
                narrow_band_sample_count: sample_count,
                anchor_count,
                projection_anchor_count,
                iteration_count: 0,
                max_change: 0.0,
                used_fast_sweeping: false,
                used_narrow_band: false,
            },
        );
    }

    let max_iterations = ((nx + ny + nz).max(1) * 2).clamp(8, 96);
    let tolerance = spacing * 1.0e-4;
    let mut iteration_count = 0_usize;
    let mut last_max_change = 0.0_f32;
    let orders = [
        (true, true, true),
        (false, true, true),
        (true, false, true),
        (false, false, true),
        (true, true, false),
        (false, true, false),
        (true, false, false),
        (false, false, false),
    ];

    for iteration in 0..max_iterations {
        let mut max_change = 0.0_f32;
        for (x_forward, y_forward, z_forward) in orders {
            let xs = sweep_indices(nx, x_forward);
            let ys = sweep_indices(ny, y_forward);
            let zs = sweep_indices(nz, z_forward);
            for &ix in &xs {
                for &iy in &ys {
                    for &iz in &zs {
                        let index = sample_grid_index(ix, iy, iz, ny, nz);
                        if fixed[index] {
                            continue;
                        }
                        let candidate =
                            fast_sweeping_update(&distances, ix, iy, iz, nx, ny, nz, spacing);
                        if candidate.is_finite() && candidate < distances[index] {
                            let change = (distances[index] - candidate).abs();
                            max_change = max_change.max(change);
                            distances[index] = candidate;
                        }
                    }
                }
            }
        }
        iteration_count = iteration + 1;
        last_max_change = max_change;
        if max_change <= tolerance {
            break;
        }
    }

    let samples = raw_samples
        .iter()
        .copied()
        .zip(distances)
        .map(|(raw, distance)| {
            if !raw.is_finite() || !distance.is_finite() {
                return raw;
            }
            match sign(raw) {
                -1 => -distance,
                1 => distance,
                _ => 0.0,
            }
        })
        .collect();

    (
        samples,
        ReconditioningReport {
            sample_count,
            narrow_band_sample_count: sample_count,
            anchor_count,
            projection_anchor_count,
            iteration_count,
            max_change: last_max_change,
            used_fast_sweeping: true,
            used_narrow_band: false,
        },
    )
}

fn add_interface_edge_anchor(
    a_index: usize,
    b_index: usize,
    raw_samples: &[f32],
    spacing: f32,
    set_anchor: &mut impl FnMut(usize, f32) -> bool,
) {
    add_interface_edge_anchor_from_values(
        a_index,
        b_index,
        raw_samples.get(a_index).copied(),
        raw_samples.get(b_index).copied(),
        spacing,
        set_anchor,
    );
}

fn add_interface_edge_anchor_from_values(
    a_index: usize,
    b_index: usize,
    a: Option<f32>,
    b: Option<f32>,
    spacing: f32,
    set_anchor: &mut impl FnMut(usize, f32) -> bool,
) {
    let (Some(a), Some(b)) = (a, b) else {
        return;
    };
    if !a.is_finite() || !b.is_finite() {
        return;
    }
    let a_sign = sign(a);
    let b_sign = sign(b);
    if a_sign == b_sign && a_sign != 0 {
        return;
    }
    let denom = a.abs() + b.abs();
    if denom <= f32::EPSILON {
        set_anchor(a_index, 0.0);
        set_anchor(b_index, 0.0);
        return;
    }
    set_anchor(a_index, spacing * a.abs() / denom);
    set_anchor(b_index, spacing * b.abs() / denom);
}

fn projection_corrected_dense_distance(
    raw_samples: &[f32],
    index: usize,
    ix: usize,
    iy: usize,
    iz: usize,
    nx: usize,
    ny: usize,
    nz: usize,
    spacing: f32,
    anchor_band: f32,
) -> Option<f32> {
    let value = *raw_samples.get(index)?;
    if !value.is_finite() || value.abs() > anchor_band {
        return None;
    }
    let gradient = Vec3::new(
        dense_axis_derivative(raw_samples, ix, iy, iz, nx, ny, nz, spacing, 0)?,
        dense_axis_derivative(raw_samples, ix, iy, iz, nx, ny, nz, spacing, 1)?,
        dense_axis_derivative(raw_samples, ix, iy, iz, nx, ny, nz, spacing, 2)?,
    );
    let gradient_length = gradient.length();
    if !gradient_length.is_finite() || gradient_length <= 1.0e-6 {
        return None;
    }
    Some((value.abs() / gradient_length).min(anchor_band))
}

fn dense_axis_derivative(
    raw_samples: &[f32],
    ix: usize,
    iy: usize,
    iz: usize,
    nx: usize,
    ny: usize,
    nz: usize,
    spacing: f32,
    axis: usize,
) -> Option<f32> {
    let sample = |x: usize, y: usize, z: usize| {
        raw_samples
            .get(sample_grid_index(x, y, z, ny, nz))
            .copied()
            .filter(|value| value.is_finite())
    };

    match axis {
        0 if ix > 0 && ix < nx => {
            Some((sample(ix + 1, iy, iz)? - sample(ix - 1, iy, iz)?) / (2.0 * spacing))
        }
        0 if ix < nx => Some((sample(ix + 1, iy, iz)? - sample(ix, iy, iz)?) / spacing),
        0 if ix > 0 => Some((sample(ix, iy, iz)? - sample(ix - 1, iy, iz)?) / spacing),
        1 if iy > 0 && iy < ny => {
            Some((sample(ix, iy + 1, iz)? - sample(ix, iy - 1, iz)?) / (2.0 * spacing))
        }
        1 if iy < ny => Some((sample(ix, iy + 1, iz)? - sample(ix, iy, iz)?) / spacing),
        1 if iy > 0 => Some((sample(ix, iy, iz)? - sample(ix, iy - 1, iz)?) / spacing),
        2 if iz > 0 && iz < nz => {
            Some((sample(ix, iy, iz + 1)? - sample(ix, iy, iz - 1)?) / (2.0 * spacing))
        }
        2 if iz < nz => Some((sample(ix, iy, iz + 1)? - sample(ix, iy, iz)?) / spacing),
        2 if iz > 0 => Some((sample(ix, iy, iz)? - sample(ix, iy, iz - 1)?) / spacing),
        _ => None,
    }
}

fn projection_corrected_sparse_distance(
    shape: &SampleGridShape,
    raw_samples: &HashMap<usize, f32>,
    index: usize,
    anchor_band: f32,
) -> Option<f32> {
    let value = *raw_samples.get(&index)?;
    if !value.is_finite() || value.abs() > anchor_band {
        return None;
    }
    let (ix, iy, iz) = shape.coords(index);
    let gradient = Vec3::new(
        sparse_axis_derivative(shape, raw_samples, ix, iy, iz, 0)?,
        sparse_axis_derivative(shape, raw_samples, ix, iy, iz, 1)?,
        sparse_axis_derivative(shape, raw_samples, ix, iy, iz, 2)?,
    );
    let gradient_length = gradient.length();
    if !gradient_length.is_finite() || gradient_length <= 1.0e-6 {
        return None;
    }
    Some((value.abs() / gradient_length).min(anchor_band))
}

fn sparse_axis_derivative(
    shape: &SampleGridShape,
    raw_samples: &HashMap<usize, f32>,
    ix: usize,
    iy: usize,
    iz: usize,
    axis: usize,
) -> Option<f32> {
    let sample = |x: usize, y: usize, z: usize| {
        raw_samples
            .get(&shape.index(x, y, z))
            .copied()
            .filter(|value| value.is_finite())
    };

    match axis {
        0 if ix > 0 && ix < shape.nx => {
            Some((sample(ix + 1, iy, iz)? - sample(ix - 1, iy, iz)?) / (2.0 * shape.spacing))
        }
        0 if ix < shape.nx => Some((sample(ix + 1, iy, iz)? - sample(ix, iy, iz)?) / shape.spacing),
        0 if ix > 0 => Some((sample(ix, iy, iz)? - sample(ix - 1, iy, iz)?) / shape.spacing),
        1 if iy > 0 && iy < shape.ny => {
            Some((sample(ix, iy + 1, iz)? - sample(ix, iy - 1, iz)?) / (2.0 * shape.spacing))
        }
        1 if iy < shape.ny => Some((sample(ix, iy + 1, iz)? - sample(ix, iy, iz)?) / shape.spacing),
        1 if iy > 0 => Some((sample(ix, iy, iz)? - sample(ix, iy - 1, iz)?) / shape.spacing),
        2 if iz > 0 && iz < shape.nz => {
            Some((sample(ix, iy, iz + 1)? - sample(ix, iy, iz - 1)?) / (2.0 * shape.spacing))
        }
        2 if iz < shape.nz => Some((sample(ix, iy, iz + 1)? - sample(ix, iy, iz)?) / shape.spacing),
        2 if iz > 0 => Some((sample(ix, iy, iz)? - sample(ix, iy, iz - 1)?) / shape.spacing),
        _ => None,
    }
}

fn fast_sweeping_update_sparse(
    distances: &HashMap<usize, f32>,
    shape: &SampleGridShape,
    ix: usize,
    iy: usize,
    iz: usize,
) -> f32 {
    let mut neighbors = Vec::with_capacity(3);

    let mut axis_min = f32::INFINITY;
    if ix > 0 {
        axis_min = axis_min.min(
            distances
                .get(&shape.index(ix - 1, iy, iz))
                .copied()
                .unwrap_or(f32::INFINITY),
        );
    }
    if ix < shape.nx {
        axis_min = axis_min.min(
            distances
                .get(&shape.index(ix + 1, iy, iz))
                .copied()
                .unwrap_or(f32::INFINITY),
        );
    }
    if axis_min.is_finite() {
        neighbors.push(axis_min);
    }

    axis_min = f32::INFINITY;
    if iy > 0 {
        axis_min = axis_min.min(
            distances
                .get(&shape.index(ix, iy - 1, iz))
                .copied()
                .unwrap_or(f32::INFINITY),
        );
    }
    if iy < shape.ny {
        axis_min = axis_min.min(
            distances
                .get(&shape.index(ix, iy + 1, iz))
                .copied()
                .unwrap_or(f32::INFINITY),
        );
    }
    if axis_min.is_finite() {
        neighbors.push(axis_min);
    }

    axis_min = f32::INFINITY;
    if iz > 0 {
        axis_min = axis_min.min(
            distances
                .get(&shape.index(ix, iy, iz - 1))
                .copied()
                .unwrap_or(f32::INFINITY),
        );
    }
    if iz < shape.nz {
        axis_min = axis_min.min(
            distances
                .get(&shape.index(ix, iy, iz + 1))
                .copied()
                .unwrap_or(f32::INFINITY),
        );
    }
    if axis_min.is_finite() {
        neighbors.push(axis_min);
    }

    solve_eikonal_update(&mut neighbors, shape.spacing)
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct EikonalStencil {
    value: f32,
    spacing: f32,
}

fn fast_sweeping_update_sparse_second_order(
    distances: &HashMap<usize, f32>,
    shape: &SampleGridShape,
    ix: usize,
    iy: usize,
    iz: usize,
) -> f32 {
    let mut stencils = Vec::with_capacity(3);
    if let Some(stencil) = second_order_sparse_axis_stencil(distances, shape, ix, iy, iz, 0) {
        stencils.push(stencil);
    }
    if let Some(stencil) = second_order_sparse_axis_stencil(distances, shape, ix, iy, iz, 1) {
        stencils.push(stencil);
    }
    if let Some(stencil) = second_order_sparse_axis_stencil(distances, shape, ix, iy, iz, 2) {
        stencils.push(stencil);
    }

    solve_eikonal_update_weighted(&mut stencils)
}

fn second_order_sparse_axis_stencil(
    distances: &HashMap<usize, f32>,
    shape: &SampleGridShape,
    ix: usize,
    iy: usize,
    iz: usize,
    axis: usize,
) -> Option<EikonalStencil> {
    let negative = sparse_direction_stencil(distances, shape, ix, iy, iz, axis, -1);
    let positive = sparse_direction_stencil(distances, shape, ix, iy, iz, axis, 1);
    match (negative, positive) {
        (Some(negative), Some(positive)) => {
            if negative.value <= positive.value {
                Some(negative)
            } else {
                Some(positive)
            }
        }
        (Some(stencil), None) | (None, Some(stencil)) => Some(stencil),
        (None, None) => None,
    }
}

fn sparse_direction_stencil(
    distances: &HashMap<usize, f32>,
    shape: &SampleGridShape,
    ix: usize,
    iy: usize,
    iz: usize,
    axis: usize,
    direction: isize,
) -> Option<EikonalStencil> {
    let first = sparse_offset_distance(distances, shape, ix, iy, iz, axis, direction)?;
    let second = sparse_offset_distance(distances, shape, ix, iy, iz, axis, direction * 2);
    if let Some(second) = second.filter(|second| *second <= first) {
        let value = (4.0 * first - second) / 3.0;
        if value.is_finite() {
            return Some(EikonalStencil {
                value,
                spacing: shape.spacing * (2.0 / 3.0),
            });
        }
    }
    Some(EikonalStencil {
        value: first,
        spacing: shape.spacing,
    })
}

fn sparse_offset_distance(
    distances: &HashMap<usize, f32>,
    shape: &SampleGridShape,
    ix: usize,
    iy: usize,
    iz: usize,
    axis: usize,
    offset: isize,
) -> Option<f32> {
    let mut x = ix as isize;
    let mut y = iy as isize;
    let mut z = iz as isize;
    match axis {
        0 => x += offset,
        1 => y += offset,
        2 => z += offset,
        _ => return None,
    }
    if x < 0
        || y < 0
        || z < 0
        || x > shape.nx as isize
        || y > shape.ny as isize
        || z > shape.nz as isize
    {
        return None;
    }
    distances
        .get(&shape.index(x as usize, y as usize, z as usize))
        .copied()
        .filter(|distance| distance.is_finite())
}

fn solve_eikonal_update_weighted(stencils: &mut Vec<EikonalStencil>) -> f32 {
    if stencils.is_empty() {
        return f32::INFINITY;
    }
    stencils.sort_by(|a, b| a.value.total_cmp(&b.value));
    let mut best = f32::INFINITY;
    for count in 1..=stencils.len() {
        let active = &stencils[..count];
        let mut a = 0.0_f32;
        let mut b = 0.0_f32;
        let mut c = -1.0_f32;
        for stencil in active {
            if stencil.spacing <= 0.0 || !stencil.spacing.is_finite() {
                continue;
            }
            let weight = 1.0 / (stencil.spacing * stencil.spacing);
            a += weight;
            b += stencil.value * weight;
            c += stencil.value * stencil.value * weight;
        }
        if a <= 0.0 {
            continue;
        }
        let discriminant = (b * b - a * c).max(0.0);
        let candidate = (b + discriminant.sqrt()) / a;
        let next_value = stencils.get(count).map(|stencil| stencil.value);
        if next_value.is_none_or(|value| candidate <= value) {
            best = candidate;
            break;
        }
        best = candidate;
    }
    best
}

fn fast_sweeping_update(
    distances: &[f32],
    ix: usize,
    iy: usize,
    iz: usize,
    nx: usize,
    ny: usize,
    nz: usize,
    spacing: f32,
) -> f32 {
    let mut neighbors = Vec::with_capacity(3);

    let mut axis_min = f32::INFINITY;
    if ix > 0 {
        axis_min = axis_min.min(distances[sample_grid_index(ix - 1, iy, iz, ny, nz)]);
    }
    if ix < nx {
        axis_min = axis_min.min(distances[sample_grid_index(ix + 1, iy, iz, ny, nz)]);
    }
    if axis_min.is_finite() {
        neighbors.push(axis_min);
    }

    axis_min = f32::INFINITY;
    if iy > 0 {
        axis_min = axis_min.min(distances[sample_grid_index(ix, iy - 1, iz, ny, nz)]);
    }
    if iy < ny {
        axis_min = axis_min.min(distances[sample_grid_index(ix, iy + 1, iz, ny, nz)]);
    }
    if axis_min.is_finite() {
        neighbors.push(axis_min);
    }

    axis_min = f32::INFINITY;
    if iz > 0 {
        axis_min = axis_min.min(distances[sample_grid_index(ix, iy, iz - 1, ny, nz)]);
    }
    if iz < nz {
        axis_min = axis_min.min(distances[sample_grid_index(ix, iy, iz + 1, ny, nz)]);
    }
    if axis_min.is_finite() {
        neighbors.push(axis_min);
    }

    solve_eikonal_update(&mut neighbors, spacing)
}

fn solve_eikonal_update(neighbors: &mut Vec<f32>, spacing: f32) -> f32 {
    if neighbors.is_empty() {
        return f32::INFINITY;
    }
    neighbors.sort_by(|a, b| a.total_cmp(b));
    let mut value = neighbors[0] + spacing;

    if neighbors.len() >= 2 && value > neighbors[1] {
        let a = neighbors[0];
        let b = neighbors[1];
        let discriminant = (2.0 * spacing * spacing - (a - b).powi(2)).max(0.0);
        value = (a + b + discriminant.sqrt()) * 0.5;
    }

    if neighbors.len() >= 3 && value > neighbors[2] {
        let a = neighbors[0];
        let b = neighbors[1];
        let c = neighbors[2];
        let discriminant =
            (3.0 * spacing * spacing - (a - b).powi(2) - (a - c).powi(2) - (b - c).powi(2))
                .max(0.0);
        value = (a + b + c + discriminant.sqrt()) / 3.0;
    }

    value
}

fn sweep_indices(max: usize, forward: bool) -> Vec<usize> {
    if forward {
        (0..=max).collect()
    } else {
        (0..=max).rev().collect()
    }
}

pub fn sample_points(bounds: &SdfBounds, spacing: f32) -> Vec<Vec3> {
    let (nx, ny, nz) = grid_cell_counts(bounds, spacing);
    let mut points = Vec::with_capacity((nx + 1) * (ny + 1) * (nz + 1));
    for ix in 0..=nx {
        for iy in 0..=ny {
            for iz in 0..=nz {
                points.push(bounds.min + Vec3::new(ix as f32, iy as f32, iz as f32) * spacing);
            }
        }
    }
    points
}

fn sample_bounds_for_bounds(bounds: &SdfBounds, spacing: f32) -> SdfBounds {
    let (nx, ny, nz) = grid_cell_counts(bounds, spacing);
    SdfBounds::new(
        bounds.min,
        bounds.min + Vec3::new(nx as f32, ny as f32, nz as f32) * spacing,
    )
}

fn connected_block_components(indexes: &[BlockIndex]) -> Vec<Vec<BlockIndex>> {
    let mut remaining: HashSet<_> = indexes.iter().copied().collect();
    let mut components = Vec::new();

    while let Some(seed) = remaining.iter().next().copied() {
        remaining.remove(&seed);
        let mut stack = vec![seed];
        let mut component = Vec::new();

        while let Some(index) = stack.pop() {
            component.push(index);
            for neighbor in block_axis_neighbors(index) {
                if remaining.remove(&neighbor) {
                    stack.push(neighbor);
                }
            }
        }

        component.sort_by_key(|index| (index.x, index.y, index.z));
        components.push(component);
    }

    components.sort_by_key(|component| {
        component
            .first()
            .map(|index| (index.x, index.y, index.z))
            .unwrap_or((i32::MAX, i32::MAX, i32::MAX))
    });
    components
}

fn widest_block_axis(indexes: &[BlockIndex]) -> usize {
    let mut min = BlockIndex::new(i32::MAX, i32::MAX, i32::MAX);
    let mut max = BlockIndex::new(i32::MIN, i32::MIN, i32::MIN);
    for index in indexes {
        min.x = min.x.min(index.x);
        min.y = min.y.min(index.y);
        min.z = min.z.min(index.z);
        max.x = max.x.max(index.x);
        max.y = max.y.max(index.y);
        max.z = max.z.max(index.z);
    }
    let span_x = max.x - min.x;
    let span_y = max.y - min.y;
    let span_z = max.z - min.z;
    if span_x >= span_y && span_x >= span_z {
        0
    } else if span_y >= span_z {
        1
    } else {
        2
    }
}

fn block_axis_neighbors(index: BlockIndex) -> [BlockIndex; 6] {
    [
        BlockIndex::new(index.x - 1, index.y, index.z),
        BlockIndex::new(index.x + 1, index.y, index.z),
        BlockIndex::new(index.x, index.y - 1, index.z),
        BlockIndex::new(index.x, index.y + 1, index.z),
        BlockIndex::new(index.x, index.y, index.z - 1),
        BlockIndex::new(index.x, index.y, index.z + 1),
    ]
}

fn grid_cell_counts(bounds: &SdfBounds, spacing: f32) -> (usize, usize, usize) {
    let size = bounds.max - bounds.min;
    (
        (size.x / spacing).ceil().max(0.0) as usize,
        (size.y / spacing).ceil().max(0.0) as usize,
        (size.z / spacing).ceil().max(0.0) as usize,
    )
}

fn sample_grid_index(ix: usize, iy: usize, iz: usize, ny: usize, nz: usize) -> usize {
    ((ix * (ny + 1) + iy) * (nz + 1)) + iz
}

const SURFACE_REFINEMENT_MIN_COARSE_SPACING: f32 = 5.0;
const SURFACE_REFINEMENT_SURFACE_TARGET_SPACING: f32 = 1.5;
const SURFACE_REFINEMENT_DISTANCE_TARGET_SPACING: f32 = 1.18;
const SURFACE_REFINEMENT_MAX_SURFACE_CELLS_PER_BLOCK: usize = 1_200_000;
const SURFACE_REFINEMENT_MAX_DISTANCE_CELLS_PER_BLOCK: usize = 9_000_000;
const SURFACE_REFINEMENT_CORE_DISTANCE_BAND: f32 = 6.0;
const SURFACE_REFINEMENT_DISTANCE_BAND: f32 = 10.0;
const SURFACE_REFINEMENT_PRUNE_BAND_FACTOR: f32 = 1.0;
const SURFACE_REFINEMENT_STORE_BAND_FACTOR: f32 = 2.0;

fn surface_refinement_cell_index(ix: usize, iy: usize, iz: usize, ny: usize, nz: usize) -> usize {
    ((ix * ny + iy) * nz) + iz
}

fn surface_refinement_subdivisions(coarse_spacing: f32, target_spacing: f32) -> Option<usize> {
    if !coarse_spacing.is_finite() || coarse_spacing < SURFACE_REFINEMENT_MIN_COARSE_SPACING {
        return None;
    }
    let target_spacing = target_spacing.max(1.0e-4);
    Some((coarse_spacing / target_spacing).ceil().max(1.0) as usize)
}

fn build_surface_refinement_for_block(
    sdf: &dyn Sdf,
    bounds: &SdfBounds,
    raw_grid_samples: &[f32],
    coarse_spacing: f32,
    include_distance_band: bool,
) -> Option<SurfaceRefinement> {
    let target_spacing = if include_distance_band {
        SURFACE_REFINEMENT_DISTANCE_TARGET_SPACING
    } else {
        SURFACE_REFINEMENT_SURFACE_TARGET_SPACING
    };
    let subdivisions = surface_refinement_subdivisions(coarse_spacing, target_spacing)?;
    if !bounds.is_valid() || raw_grid_samples.is_empty() {
        return None;
    }

    let (coarse_nx, coarse_ny, coarse_nz) = grid_cell_counts(bounds, coarse_spacing);
    let expected_sample_count = (coarse_nx + 1) * (coarse_ny + 1) * (coarse_nz + 1);
    if raw_grid_samples.len() != expected_sample_count {
        return None;
    }

    let fine_spacing = coarse_spacing / subdivisions as f32;
    let fine_nx = coarse_nx * subdivisions;
    let fine_ny = coarse_ny * subdivisions;
    let fine_nz = coarse_nz * subdivisions;
    if fine_nx == 0 || fine_ny == 0 || fine_nz == 0 {
        return None;
    }

    let mut node_cache = HashMap::new();
    let cells = collect_surface_refinement_cells(
        sdf,
        bounds,
        raw_grid_samples,
        coarse_spacing,
        fine_spacing,
        subdivisions,
        0.0,
        SURFACE_REFINEMENT_MAX_SURFACE_CELLS_PER_BLOCK,
        &mut node_cache,
    );
    if cells.is_empty() {
        return None;
    }
    let (distance_cells, surface_points) = if include_distance_band {
        let mut distance_source_cells = collect_surface_refinement_cells(
            sdf,
            bounds,
            raw_grid_samples,
            coarse_spacing,
            fine_spacing,
            subdivisions,
            SURFACE_REFINEMENT_CORE_DISTANCE_BAND.min(SURFACE_REFINEMENT_DISTANCE_BAND),
            SURFACE_REFINEMENT_MAX_DISTANCE_CELLS_PER_BLOCK,
            &mut node_cache,
        );
        if SURFACE_REFINEMENT_DISTANCE_BAND > SURFACE_REFINEMENT_CORE_DISTANCE_BAND
            && distance_source_cells.len() < SURFACE_REFINEMENT_MAX_DISTANCE_CELLS_PER_BLOCK
        {
            let outer_cells = collect_surface_refinement_cells(
                sdf,
                bounds,
                raw_grid_samples,
                coarse_spacing,
                fine_spacing,
                subdivisions,
                SURFACE_REFINEMENT_DISTANCE_BAND,
                SURFACE_REFINEMENT_MAX_DISTANCE_CELLS_PER_BLOCK,
                &mut node_cache,
            );
            for (cell_index, values) in outer_cells {
                distance_source_cells.entry(cell_index).or_insert(values);
            }
        }
        let distance_source_cells = if distance_source_cells.is_empty() {
            &cells
        } else {
            &distance_source_cells
        };
        let surface_points = SurfacePointCloud::from_cells(
            distance_source_cells,
            bounds,
            fine_spacing,
            fine_ny,
            fine_nz,
        );
        let distance_cells = reinitialize_surface_refinement_cells(
            distance_source_cells,
            bounds,
            fine_spacing,
            fine_nx,
            fine_ny,
            fine_nz,
            surface_points.as_ref(),
        );
        (distance_cells, surface_points)
    } else {
        (HashMap::new(), None)
    };

    Some(SurfaceRefinement {
        bounds: bounds.clone(),
        spacing: fine_spacing,
        nx: fine_nx,
        ny: fine_ny,
        nz: fine_nz,
        cells,
        distance_cells,
        surface_points,
    })
}

#[allow(clippy::too_many_arguments)]
fn collect_surface_refinement_cells(
    sdf: &dyn Sdf,
    bounds: &SdfBounds,
    raw_grid_samples: &[f32],
    coarse_spacing: f32,
    fine_spacing: f32,
    subdivisions: usize,
    base_band: f32,
    max_cells: usize,
    node_cache: &mut HashMap<usize, f32>,
) -> HashMap<usize, [f32; 8]> {
    let (coarse_nx, coarse_ny, coarse_nz) = grid_cell_counts(bounds, coarse_spacing);
    let fine_ny = coarse_ny * subdivisions;
    let fine_nz = coarse_nz * subdivisions;
    let coarse_cell_diagonal = Vec3::splat(coarse_spacing).length();
    let coarse_band = base_band + coarse_cell_diagonal * SURFACE_REFINEMENT_PRUNE_BAND_FACTOR;
    let mut cells = HashMap::new();

    for ix in 0..coarse_nx {
        for iy in 0..coarse_ny {
            for iz in 0..coarse_nz {
                let Some(corners) =
                    coarse_cell_corner_values(raw_grid_samples, ix, iy, iz, coarse_ny, coarse_nz)
                else {
                    continue;
                };
                if !surface_refinement_candidate(&corners, coarse_band) {
                    continue;
                }

                refine_surface_cell_range(
                    sdf,
                    bounds,
                    fine_spacing,
                    fine_ny,
                    fine_nz,
                    ix * subdivisions,
                    (ix + 1) * subdivisions,
                    iy * subdivisions,
                    (iy + 1) * subdivisions,
                    iz * subdivisions,
                    (iz + 1) * subdivisions,
                    node_cache,
                    &mut cells,
                    base_band,
                    max_cells,
                );
                if cells.len() >= max_cells {
                    break;
                }
            }
            if cells.len() >= max_cells {
                break;
            }
        }
        if cells.len() >= max_cells {
            break;
        }
    }
    cells
}

fn coarse_cell_corner_values(
    samples: &[f32],
    ix: usize,
    iy: usize,
    iz: usize,
    ny: usize,
    nz: usize,
) -> Option<[f32; 8]> {
    Some([
        *samples.get(sample_grid_index(ix, iy, iz, ny, nz))?,
        *samples.get(sample_grid_index(ix + 1, iy, iz, ny, nz))?,
        *samples.get(sample_grid_index(ix, iy + 1, iz, ny, nz))?,
        *samples.get(sample_grid_index(ix + 1, iy + 1, iz, ny, nz))?,
        *samples.get(sample_grid_index(ix, iy, iz + 1, ny, nz))?,
        *samples.get(sample_grid_index(ix + 1, iy, iz + 1, ny, nz))?,
        *samples.get(sample_grid_index(ix, iy + 1, iz + 1, ny, nz))?,
        *samples.get(sample_grid_index(ix + 1, iy + 1, iz + 1, ny, nz))?,
    ])
}

#[allow(clippy::too_many_arguments)]
fn refine_surface_cell_range(
    sdf: &dyn Sdf,
    bounds: &SdfBounds,
    spacing: f32,
    ny: usize,
    nz: usize,
    ix0: usize,
    ix1: usize,
    iy0: usize,
    iy1: usize,
    iz0: usize,
    iz1: usize,
    node_cache: &mut HashMap<usize, f32>,
    cells: &mut HashMap<usize, [f32; 8]>,
    base_band: f32,
    max_cells: usize,
) {
    if cells.len() >= max_cells || ix0 >= ix1 || iy0 >= iy1 || iz0 >= iz1 {
        return;
    }

    let values = fine_range_corner_values(
        sdf, bounds, spacing, ny, nz, ix0, ix1, iy0, iy1, iz0, iz1, node_cache,
    );
    let cell_size = Vec3::new(
        (ix1 - ix0) as f32 * spacing,
        (iy1 - iy0) as f32 * spacing,
        (iz1 - iz0) as f32 * spacing,
    );
    let diagonal = cell_size.length();
    if !surface_refinement_candidate(
        &values,
        base_band + diagonal * SURFACE_REFINEMENT_PRUNE_BAND_FACTOR,
    ) {
        return;
    }

    let terminal = ix1 - ix0 == 1 && iy1 - iy0 == 1 && iz1 - iz0 == 1;
    if terminal {
        if surface_refinement_candidate(
            &values,
            base_band + diagonal * SURFACE_REFINEMENT_STORE_BAND_FACTOR,
        ) {
            cells.insert(surface_refinement_cell_index(ix0, iy0, iz0, ny, nz), values);
        }
        return;
    }

    let x_splits = split_cell_range(ix0, ix1);
    let y_splits = split_cell_range(iy0, iy1);
    let z_splits = split_cell_range(iz0, iz1);
    for (cx0, cx1) in x_splits {
        for (cy0, cy1) in y_splits {
            for (cz0, cz1) in z_splits {
                refine_surface_cell_range(
                    sdf, bounds, spacing, ny, nz, cx0, cx1, cy0, cy1, cz0, cz1, node_cache, cells,
                    base_band, max_cells,
                );
                if cells.len() >= max_cells {
                    return;
                }
            }
        }
    }
}

fn split_cell_range(start: usize, end: usize) -> [(usize, usize); 2] {
    if end - start <= 1 {
        [(start, end), (end, end)]
    } else {
        let mid = start + (end - start) / 2;
        [(start, mid), (mid, end)]
    }
}

#[allow(clippy::too_many_arguments)]
fn fine_range_corner_values(
    sdf: &dyn Sdf,
    bounds: &SdfBounds,
    spacing: f32,
    ny: usize,
    nz: usize,
    ix0: usize,
    ix1: usize,
    iy0: usize,
    iy1: usize,
    iz0: usize,
    iz1: usize,
    node_cache: &mut HashMap<usize, f32>,
) -> [f32; 8] {
    [
        fine_node_sample(sdf, bounds, spacing, ny, nz, ix0, iy0, iz0, node_cache),
        fine_node_sample(sdf, bounds, spacing, ny, nz, ix1, iy0, iz0, node_cache),
        fine_node_sample(sdf, bounds, spacing, ny, nz, ix0, iy1, iz0, node_cache),
        fine_node_sample(sdf, bounds, spacing, ny, nz, ix1, iy1, iz0, node_cache),
        fine_node_sample(sdf, bounds, spacing, ny, nz, ix0, iy0, iz1, node_cache),
        fine_node_sample(sdf, bounds, spacing, ny, nz, ix1, iy0, iz1, node_cache),
        fine_node_sample(sdf, bounds, spacing, ny, nz, ix0, iy1, iz1, node_cache),
        fine_node_sample(sdf, bounds, spacing, ny, nz, ix1, iy1, iz1, node_cache),
    ]
}

fn fine_node_sample(
    sdf: &dyn Sdf,
    bounds: &SdfBounds,
    spacing: f32,
    ny: usize,
    nz: usize,
    ix: usize,
    iy: usize,
    iz: usize,
    node_cache: &mut HashMap<usize, f32>,
) -> f32 {
    let index = sample_grid_index(ix, iy, iz, ny, nz);
    if let Some(value) = node_cache.get(&index).copied() {
        return value;
    }
    let point = bounds.min + Vec3::new(ix as f32, iy as f32, iz as f32) * spacing;
    let value = sdf.distance(point);
    node_cache.insert(index, value);
    value
}

fn reinitialize_surface_refinement_cells(
    cells: &HashMap<usize, [f32; 8]>,
    bounds: &SdfBounds,
    spacing: f32,
    nx: usize,
    ny: usize,
    nz: usize,
    surface_points: Option<&SurfacePointCloud>,
) -> HashMap<usize, [f32; 8]> {
    let Some(surface_points) = surface_points else {
        return cells.clone();
    };
    let node_values = surface_refinement_node_values(cells, ny, nz);
    if node_values.is_empty() || spacing <= 0.0 || !spacing.is_finite() {
        return cells.clone();
    }

    let shape = SampleGridShape {
        bounds: bounds.clone(),
        spacing,
        nx,
        ny,
        nz,
        sample_count: (nx + 1) * (ny + 1) * (nz + 1),
    };
    let node_distances =
        redistance_surface_refinement_band(&shape, &node_values, surface_points, spacing);

    cells
        .iter()
        .map(|(&cell_index, raw_values)| {
            let (ix, iy, iz) = surface_refinement_cell_coords(cell_index, ny, nz);
            let mut signed_distances = [0.0_f32; 8];
            for (corner_index, offset) in SURFACE_REFINEMENT_CORNER_OFFSETS.iter().enumerate() {
                let nx = ix + offset[0];
                let nyi = iy + offset[1];
                let nzi = iz + offset[2];
                let node_index = sample_grid_index(nx, nyi, nzi, ny, nz);
                let raw = raw_values[corner_index];
                let distance = node_distances
                    .get(&node_index)
                    .copied()
                    .filter(|distance| distance.is_finite());
                signed_distances[corner_index] = match (raw.is_finite(), distance) {
                    (true, Some(distance)) => match sign(raw) {
                        -1 => -distance,
                        1 => distance,
                        _ => 0.0,
                    },
                    _ => raw,
                };
            }
            (cell_index, signed_distances)
        })
        .collect()
}

fn redistance_surface_refinement_band(
    shape: &SampleGridShape,
    raw_samples: &HashMap<usize, f32>,
    surface_points: &SurfacePointCloud,
    spacing: f32,
) -> HashMap<usize, f32> {
    let mut distances: HashMap<usize, f32> = raw_samples
        .keys()
        .copied()
        .map(|index| (index, f32::INFINITY))
        .collect();
    let mut fixed = HashSet::new();
    let active_indices: Vec<_> = raw_samples.keys().copied().collect();
    let anchor_band = (spacing * 2.25)
        .max(spacing + 1.0e-4)
        .min(SURFACE_REFINEMENT_CORE_DISTANCE_BAND);

    let mut set_anchor = |index: usize, distance: f32| -> bool {
        if !raw_samples.contains_key(&index) || !distance.is_finite() || distance > anchor_band {
            return false;
        }
        let distance = distance.max(0.0);
        let entry = distances.entry(index).or_insert(f32::INFINITY);
        let improved = distance < *entry;
        if improved {
            *entry = distance;
        }
        let inserted = fixed.insert(index);
        inserted || improved
    };

    for (&index, &value) in raw_samples {
        if value.is_finite() && value.abs() <= f32::EPSILON {
            set_anchor(index, 0.0);
        }
    }

    let surface_anchor_search = anchor_band + spacing;
    for &index in &active_indices {
        let point = shape.point_for_index(index);
        if let Some(distance) = surface_points.nearest_distance(point, surface_anchor_search) {
            set_anchor(index, distance);
        }
    }

    for &index in &active_indices {
        if let Some(distance) =
            projection_corrected_sparse_distance(shape, raw_samples, index, anchor_band)
        {
            set_anchor(index, distance);
        }
    }

    for &index in &active_indices {
        let (ix, iy, iz) = shape.coords(index);
        if ix < shape.nx {
            let neighbor = shape.index(ix + 1, iy, iz);
            add_interface_edge_anchor_from_values(
                index,
                neighbor,
                raw_samples.get(&index).copied(),
                raw_samples.get(&neighbor).copied(),
                spacing,
                &mut set_anchor,
            );
        }
        if iy < shape.ny {
            let neighbor = shape.index(ix, iy + 1, iz);
            add_interface_edge_anchor_from_values(
                index,
                neighbor,
                raw_samples.get(&index).copied(),
                raw_samples.get(&neighbor).copied(),
                spacing,
                &mut set_anchor,
            );
        }
        if iz < shape.nz {
            let neighbor = shape.index(ix, iy, iz + 1);
            add_interface_edge_anchor_from_values(
                index,
                neighbor,
                raw_samples.get(&index).copied(),
                raw_samples.get(&neighbor).copied(),
                spacing,
                &mut set_anchor,
            );
        }
    }

    if fixed.is_empty() {
        return raw_samples
            .iter()
            .filter_map(|(&index, &value)| value.is_finite().then_some((index, value.abs())))
            .collect();
    }

    let max_iterations = ((shape.nx + shape.ny + shape.nz).max(1) * 2).clamp(8, 128);
    let tolerance = spacing * 1.0e-4;
    let orders = [
        (true, true, true),
        (false, true, true),
        (true, false, true),
        (false, false, true),
        (true, true, false),
        (false, true, false),
        (true, false, false),
        (false, false, false),
    ];
    let ordered_sweeps: Vec<Vec<usize>> = orders
        .into_iter()
        .map(|(x_forward, y_forward, z_forward)| {
            let mut ordered = active_indices.clone();
            ordered.sort_by_key(|index| {
                let (ix, iy, iz) = shape.coords(*index);
                (
                    sweep_coord(ix, shape.nx, x_forward),
                    sweep_coord(iy, shape.ny, y_forward),
                    sweep_coord(iz, shape.nz, z_forward),
                )
            });
            ordered
        })
        .collect();

    for _ in 0..max_iterations {
        let mut max_change = 0.0_f32;
        for ordered in &ordered_sweeps {
            for &index in ordered {
                if fixed.contains(&index) {
                    continue;
                }
                let (ix, iy, iz) = shape.coords(index);
                let candidate = fast_sweeping_update_sparse(&distances, shape, ix, iy, iz);
                let current = distances.get(&index).copied().unwrap_or(f32::INFINITY);
                if candidate.is_finite() && candidate < current {
                    let change = (current - candidate).abs();
                    max_change = max_change.max(change);
                    distances.insert(index, candidate);
                }
            }
        }
        if max_change <= tolerance {
            break;
        }
    }

    distances
}

fn surface_refinement_node_values(
    cells: &HashMap<usize, [f32; 8]>,
    ny: usize,
    nz: usize,
) -> HashMap<usize, f32> {
    let mut values = HashMap::new();
    for (&cell_index, cell_values) in cells {
        let (ix, iy, iz) = surface_refinement_cell_coords(cell_index, ny, nz);
        for (corner_index, offset) in SURFACE_REFINEMENT_CORNER_OFFSETS.iter().enumerate() {
            let nx = ix + offset[0];
            let nyi = iy + offset[1];
            let nzi = iz + offset[2];
            values
                .entry(sample_grid_index(nx, nyi, nzi, ny, nz))
                .or_insert(cell_values[corner_index]);
        }
    }
    values
}

fn surface_refinement_node_gradient(
    values: &HashMap<usize, f32>,
    ix: usize,
    iy: usize,
    iz: usize,
    spacing: f32,
    ny: usize,
    nz: usize,
) -> Option<Vec3> {
    if spacing <= 0.0 || !spacing.is_finite() {
        return None;
    }
    let current = *values.get(&sample_grid_index(ix, iy, iz, ny, nz))?;
    let dx = surface_refinement_axis_derivative(values, ix, iy, iz, 0, spacing, ny, nz, current)?;
    let dy = surface_refinement_axis_derivative(values, ix, iy, iz, 1, spacing, ny, nz, current)?;
    let dz = surface_refinement_axis_derivative(values, ix, iy, iz, 2, spacing, ny, nz, current)?;
    let gradient = Vec3::new(dx, dy, dz);
    gradient.is_finite().then_some(gradient)
}

#[allow(clippy::too_many_arguments)]
fn surface_refinement_axis_derivative(
    values: &HashMap<usize, f32>,
    ix: usize,
    iy: usize,
    iz: usize,
    axis: usize,
    spacing: f32,
    ny: usize,
    nz: usize,
    current: f32,
) -> Option<f32> {
    let sample = |delta: isize| -> Option<f32> {
        let mut x = ix as isize;
        let mut y = iy as isize;
        let mut z = iz as isize;
        match axis {
            0 => x += delta,
            1 => y += delta,
            2 => z += delta,
            _ => return None,
        }
        if x < 0 || y < 0 || z < 0 {
            return None;
        }
        values
            .get(&sample_grid_index(
                x as usize, y as usize, z as usize, ny, nz,
            ))
            .copied()
            .filter(|value| value.is_finite())
    };

    let negative = sample(-1);
    let positive = sample(1);
    match (negative, positive) {
        (Some(negative), Some(positive)) => Some((positive - negative) / (2.0 * spacing)),
        (None, Some(positive)) if current.is_finite() => Some((positive - current) / spacing),
        (Some(negative), None) if current.is_finite() => Some((current - negative) / spacing),
        _ => None,
    }
}

fn surface_refinement_gradient_corrected_distance(
    raw: f32,
    gradient: Vec3,
    max_distance: f32,
) -> Option<f32> {
    let gradient_length = gradient.length();
    if !raw.is_finite()
        || !gradient_length.is_finite()
        || gradient_length <= 0.15
        || max_distance <= 0.0
    {
        return None;
    }
    let distance = raw.abs() / gradient_length;
    (distance.is_finite() && distance <= max_distance).then_some(distance)
}

fn surface_refinement_corrected_distance(
    point_distance: Option<f32>,
    gradient_distance: Option<f32>,
    spacing: f32,
) -> Option<f32> {
    match (point_distance, gradient_distance) {
        (Some(point_distance), Some(gradient_distance)) => {
            if gradient_distance <= point_distance {
                let lower_bound = (point_distance - spacing * 2.0).max(0.0);
                Some(gradient_distance.max(lower_bound))
            } else {
                Some(point_distance)
            }
        }
        (Some(point_distance), None) => Some(point_distance),
        (None, Some(gradient_distance)) => Some(gradient_distance),
        (None, None) => None,
    }
}

fn surface_refinement_has_crossing(values: &[f32; 8]) -> bool {
    if values.iter().any(|value| !value.is_finite()) {
        return false;
    }
    let mut min_value = f32::INFINITY;
    let mut max_value = f32::NEG_INFINITY;
    for &value in values {
        min_value = min_value.min(value);
        max_value = max_value.max(value);
    }
    min_value <= 0.0 && max_value >= 0.0
}

const SURFACE_REFINEMENT_CORNER_OFFSETS: [[usize; 3]; 8] = [
    [0, 0, 0],
    [1, 0, 0],
    [0, 1, 0],
    [1, 1, 0],
    [0, 0, 1],
    [1, 0, 1],
    [0, 1, 1],
    [1, 1, 1],
];

const SURFACE_REFINEMENT_EDGE_CORNERS: [[usize; 2]; 12] = [
    [0, 1],
    [2, 3],
    [4, 5],
    [6, 7],
    [0, 2],
    [1, 3],
    [4, 6],
    [5, 7],
    [0, 4],
    [1, 5],
    [2, 6],
    [3, 7],
];

fn surface_refinement_cell_coords(index: usize, ny: usize, nz: usize) -> (usize, usize, usize) {
    let yz_count = ny * nz;
    let ix = index / yz_count;
    let rem = index % yz_count;
    let iy = rem / nz;
    let iz = rem % nz;
    (ix, iy, iz)
}

fn surface_refinement_corner_point(
    bounds: &SdfBounds,
    spacing: f32,
    ix: usize,
    iy: usize,
    iz: usize,
    corner_index: usize,
) -> Vec3 {
    let offset = SURFACE_REFINEMENT_CORNER_OFFSETS[corner_index];
    bounds.min
        + Vec3::new(
            (ix + offset[0]) as f32,
            (iy + offset[1]) as f32,
            (iz + offset[2]) as f32,
        ) * spacing
}

#[derive(Clone, Debug, PartialEq)]
struct SurfacePointCloud {
    cell_size: f32,
    buckets: HashMap<(i32, i32, i32), Vec<Vec3>>,
}

impl SurfacePointCloud {
    fn from_cells(
        cells: &HashMap<usize, [f32; 8]>,
        bounds: &SdfBounds,
        spacing: f32,
        ny: usize,
        nz: usize,
    ) -> Option<Self> {
        let cell_size = (spacing * 4.0).max(4.0);
        let mut cloud = Self {
            cell_size,
            buckets: HashMap::new(),
        };
        let mut inserted = HashSet::new();
        for (&cell_index, values) in cells {
            let (ix, iy, iz) = surface_refinement_cell_coords(cell_index, ny, nz);
            for edge in SURFACE_REFINEMENT_EDGE_CORNERS {
                let a = values[edge[0]];
                let b = values[edge[1]];
                if !a.is_finite() || !b.is_finite() || !crosses_zero_sample(a, b) {
                    continue;
                }
                let pa = surface_refinement_corner_point(bounds, spacing, ix, iy, iz, edge[0]);
                let pb = surface_refinement_corner_point(bounds, spacing, ix, iy, iz, edge[1]);
                let denom = a.abs() + b.abs();
                let t = if denom <= f32::EPSILON {
                    0.5
                } else {
                    (a.abs() / denom).clamp(0.0, 1.0)
                };
                let point = pa.lerp(pb, t);
                let key = quantized_surface_point_key(point);
                if inserted.insert(key) {
                    cloud.insert(point);
                }
            }
        }
        (!cloud.buckets.is_empty()).then_some(cloud)
    }

    fn insert(&mut self, point: Vec3) {
        self.buckets
            .entry(self.bucket_key(point))
            .or_default()
            .push(point);
    }

    fn point_count(&self) -> usize {
        self.buckets.values().map(Vec::len).sum()
    }

    fn nearest_distance(&self, point: Vec3, max_distance: f32) -> Option<f32> {
        if max_distance <= 0.0 || !max_distance.is_finite() {
            return None;
        }
        let center = self.bucket_key(point);
        let radius = (max_distance / self.cell_size).ceil().max(1.0) as i32;
        let mut best = max_distance;
        let mut found = false;
        for dx in -radius..=radius {
            for dy in -radius..=radius {
                for dz in -radius..=radius {
                    let key = (center.0 + dx, center.1 + dy, center.2 + dz);
                    let Some(points) = self.buckets.get(&key) else {
                        continue;
                    };
                    for candidate in points {
                        let distance = point.distance(*candidate);
                        if distance < best {
                            best = distance;
                            found = true;
                        }
                    }
                }
            }
        }
        found.then_some(best)
    }

    fn bucket_key(&self, point: Vec3) -> (i32, i32, i32) {
        (
            (point.x / self.cell_size).floor() as i32,
            (point.y / self.cell_size).floor() as i32,
            (point.z / self.cell_size).floor() as i32,
        )
    }
}

fn crosses_zero_sample(a: f32, b: f32) -> bool {
    a == 0.0 || b == 0.0 || (a < 0.0 && b > 0.0) || (a > 0.0 && b < 0.0)
}

fn quantized_surface_point_key(point: Vec3) -> (i32, i32, i32) {
    (
        (point.x * 1000.0).round() as i32,
        (point.y * 1000.0).round() as i32,
        (point.z * 1000.0).round() as i32,
    )
}

fn surface_refinement_candidate(values: &[f32; 8], band: f32) -> bool {
    if values.iter().any(|value| !value.is_finite()) {
        return false;
    }
    let mut min_value = f32::INFINITY;
    let mut max_value = f32::NEG_INFINITY;
    let mut min_abs = f32::INFINITY;
    for &value in values {
        min_value = min_value.min(value);
        max_value = max_value.max(value);
        min_abs = min_abs.min(value.abs());
    }
    min_value <= 0.0 && max_value >= 0.0 || min_abs <= band.max(0.0)
}

pub fn finite_difference_gradient(sdf: &dyn Sdf, point: Vec3, step: f32) -> Vec3 {
    let h = step.max(1.0e-4);
    Vec3::new(
        sdf.distance(point + Vec3::X * h) - sdf.distance(point - Vec3::X * h),
        sdf.distance(point + Vec3::Y * h) - sdf.distance(point - Vec3::Y * h),
        sdf.distance(point + Vec3::Z * h) - sdf.distance(point - Vec3::Z * h),
    ) / (2.0 * h)
}

fn percentile(values: &mut [f32], percentile: f32) -> f32 {
    if values.is_empty() {
        return 0.0;
    }
    values.sort_by(|a, b| a.total_cmp(b));
    let index = ((percentile / 100.0) * (values.len().saturating_sub(1)) as f32).round() as usize;
    values[index]
}

fn sign(value: f32) -> i8 {
    if value < 0.0 {
        -1
    } else if value > 0.0 {
        1
    } else {
        0
    }
}

fn sign_with_tolerance(value: f32, tolerance: f32) -> i8 {
    let tolerance = tolerance.max(0.0);
    if value < -tolerance {
        -1
    } else if value > tolerance {
        1
    } else {
        0
    }
}

fn canonical_sign_mismatch(conditioned: f32, canonical: f32) -> bool {
    (canonical < 0.0 && conditioned > 0.0) || (canonical > 0.0 && conditioned < 0.0)
}

fn cache_fidelity_preserving_conditioned_sample(
    canonical: f32,
    conditioned: f32,
    policy: &ConditioningPolicy,
) -> f32 {
    if !canonical.is_finite() || !conditioned.is_finite() {
        return canonical;
    }

    let surface_lock_band = policy.max_interface_error.max(policy.grid_spacing * 1.0e-4);
    if canonical.abs() <= surface_lock_band || conditioned.abs() <= surface_lock_band {
        return canonical;
    }

    if canonical_sign_mismatch(conditioned, canonical) {
        return canonical;
    }

    conditioned
}

fn published_fidelity_preserving_conditioned_sample(
    canonical: f32,
    conditioned: f32,
    policy: &ConditioningPolicy,
) -> f32 {
    if !canonical.is_finite() || !conditioned.is_finite() {
        return canonical;
    }

    let fidelity_band = published_fidelity_band(policy);
    if canonical.abs() <= fidelity_band || conditioned.abs() <= cache_fidelity_band(policy) {
        return canonical;
    }

    if canonical_sign_mismatch(conditioned, canonical) {
        return canonical;
    }

    let transition_width = policy.grid_spacing.max(1.0e-4);
    let transition_end = fidelity_band + transition_width;
    if canonical.abs() <= transition_end {
        let t = ((canonical.abs() - fidelity_band) / transition_width).clamp(0.0, 1.0);
        let smooth_t = t * t * (3.0 - 2.0 * t);
        return lerp(canonical, conditioned, smooth_t);
    }

    conditioned
}

fn cache_fidelity_band(policy: &ConditioningPolicy) -> f32 {
    policy.interface_band.max(policy.grid_spacing * 2.0)
}

fn published_fidelity_band(policy: &ConditioningPolicy) -> f32 {
    let cache_band = cache_fidelity_band(policy);
    if policy.grid_spacing < 5.0 {
        return cache_band;
    }
    cache_band.max(policy.grid_spacing * 7.0)
}

fn adaptive_conflicts_with_block_near_surface(
    adaptive: f32,
    block: f32,
    policy: &ConditioningPolicy,
) -> bool {
    if !adaptive.is_finite() || !block.is_finite() {
        return false;
    }
    let interface_band = policy.interface_band.max(policy.grid_spacing);
    let near_surface = adaptive.abs() <= interface_band || block.abs() <= interface_band;
    near_surface && canonical_sign_mismatch(adaptive, block)
}

fn fidelity_preserving_dense_samples(
    raw_samples: &[f32],
    mut conditioned_samples: Vec<f32>,
    policy: &ConditioningPolicy,
) -> Vec<f32> {
    for (raw, conditioned) in raw_samples
        .iter()
        .copied()
        .zip(conditioned_samples.iter_mut())
    {
        *conditioned = cache_fidelity_preserving_conditioned_sample(raw, *conditioned, policy);
    }
    conditioned_samples
}

fn fidelity_preserving_sparse_samples(
    raw_samples: &HashMap<usize, f32>,
    mut conditioned_samples: HashMap<usize, f32>,
    policy: &ConditioningPolicy,
) -> HashMap<usize, f32> {
    for (&index, &raw) in raw_samples {
        if let Some(conditioned) = conditioned_samples.get_mut(&index) {
            *conditioned = cache_fidelity_preserving_conditioned_sample(raw, *conditioned, policy);
        }
    }
    conditioned_samples
}

fn max_dense_sample_change(raw_samples: &[f32], conditioned_samples: &[f32]) -> f32 {
    raw_samples
        .iter()
        .copied()
        .zip(conditioned_samples.iter().copied())
        .filter(|(raw, conditioned)| raw.is_finite() && conditioned.is_finite())
        .map(|(raw, conditioned)| (conditioned - raw).abs())
        .fold(0.0_f32, f32::max)
}

fn max_sparse_sample_change(
    raw_samples: &HashMap<usize, f32>,
    conditioned_samples: &HashMap<usize, f32>,
) -> f32 {
    raw_samples
        .iter()
        .filter_map(|(index, raw)| {
            conditioned_samples
                .get(index)
                .copied()
                .map(|conditioned| (*raw, conditioned))
        })
        .filter(|(raw, conditioned)| raw.is_finite() && conditioned.is_finite())
        .map(|(raw, conditioned)| (conditioned - raw).abs())
        .fold(0.0_f32, f32::max)
}

fn strong_sign_mismatch(a: f32, b: f32, tolerance: f32) -> bool {
    let a_sign = sign_with_tolerance(a, tolerance);
    let b_sign = sign_with_tolerance(b, tolerance);
    a_sign != 0 && b_sign != 0 && a_sign != b_sign
}

fn dominant_axis(direction: Vec3) -> usize {
    let abs = direction.abs();
    if abs.x >= abs.y && abs.x >= abs.z {
        0
    } else if abs.y >= abs.z {
        1
    } else {
        2
    }
}

fn axis_component(point: Vec3, axis: usize) -> f32 {
    match axis.min(2) {
        0 => point.x,
        1 => point.y,
        _ => point.z,
    }
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::booleans::{SmoothUnion, Union};
    use crate::sdf::primitives::{SdfBox, Sphere};
    use crate::sdf::transforms::Translate;
    use std::sync::{Arc, Mutex};
    use std::time::{Duration, Instant};

    struct ScaledSdf {
        child: Arc<dyn Sdf>,
        scale: f32,
    }

    impl ScaledSdf {
        fn new(child: Arc<dyn Sdf>, scale: f32) -> Self {
            Self { child, scale }
        }
    }

    impl Sdf for ScaledSdf {
        fn distance(&self, point: Vec3) -> f32 {
            self.child.distance(point) * self.scale
        }

        fn metadata(&self) -> SdfNodeMetadata {
            self.child.metadata()
        }
    }

    struct ScaledPlane {
        scale: f32,
    }

    impl Sdf for ScaledPlane {
        fn distance(&self, point: Vec3) -> f32 {
            point.x * self.scale
        }
    }

    struct FixedMetadataPlane {
        offset: f32,
    }

    impl Sdf for FixedMetadataPlane {
        fn distance(&self, point: Vec3) -> f32 {
            point.x - self.offset
        }

        fn metadata(&self) -> SdfNodeMetadata {
            SdfNodeMetadata::new("fixed_metadata_plane")
                .with_support_bounds(SdfBounds::new(Vec3::splat(-4.0), Vec3::splat(4.0)))
        }
    }

    struct FeatureSizedSphere {
        radius: f32,
        min_feature_size: f32,
    }

    impl Sdf for FeatureSizedSphere {
        fn distance(&self, point: Vec3) -> f32 {
            point.length() - self.radius
        }

        fn metadata(&self) -> SdfNodeMetadata {
            SdfNodeMetadata::new("feature_sized_sphere")
                .with_support_bounds(SdfBounds::from_center_radius(Vec3::ZERO, self.radius))
                .with_min_feature_size(self.min_feature_size)
                .with_f32_parameters([self.radius, self.min_feature_size])
        }
    }

    struct SlowSphere {
        radius: f32,
        sleep_once: Mutex<bool>,
        delay: Duration,
    }

    impl SlowSphere {
        fn new(radius: f32, delay: Duration) -> Self {
            Self {
                radius,
                sleep_once: Mutex::new(false),
                delay,
            }
        }
    }

    impl Sdf for SlowSphere {
        fn distance(&self, point: Vec3) -> f32 {
            if let Ok(mut slept) = self.sleep_once.lock() {
                if !*slept {
                    *slept = true;
                    std::thread::sleep(self.delay);
                }
            }
            point.length() - self.radius
        }

        fn metadata(&self) -> SdfNodeMetadata {
            SdfNodeMetadata::new("slow_sphere")
                .with_support_bounds(SdfBounds::from_center_radius(Vec3::ZERO, self.radius))
                .with_f32_parameters([self.radius])
        }
    }

    fn unit_bounds_metadata(fingerprint: u64) -> SdfNodeMetadata {
        SdfNodeMetadata::new("same_envelope_profile")
            .with_support_bounds(SdfBounds::new(Vec3::splat(-1.0), Vec3::splat(1.0)))
            .with_parameter_fingerprint(fingerprint)
    }

    fn feature_dirty_region(
        feature_id: &str,
        bounds: SdfBounds,
        halo: f32,
        recommended_grid_spacing: f32,
    ) -> DirtyRegion {
        let mut dirty_region = DirtyRegion::new(DirtyRegionSource::FeatureEdit, bounds)
            .with_halo(halo)
            .with_recommended_grid_spacing(recommended_grid_spacing);
        dirty_region.feature_ids.push(feature_id.to_string());
        dirty_region
    }

    fn wait_for_background_job(
        scheduler: &BackgroundConditioningScheduler,
        id: ConditioningJobId,
    ) -> ConditioningJobSummary {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            if let Some(summary) = scheduler.job_summary(id) {
                if matches!(
                    summary.state,
                    ConditioningJobState::Ready
                        | ConditioningJobState::Failed
                        | ConditioningJobState::Superseded
                ) {
                    return summary;
                }
            }
            assert!(
                Instant::now() < deadline,
                "background conditioning job {id:?} did not finish"
            );
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    #[test]
    fn metadata_changed_emits_union_dirty_region() {
        let previous = SdfNodeMetadata::new("profile_duct")
            .with_support_bounds(SdfBounds::new(Vec3::ZERO, Vec3::new(10.0, 2.0, 2.0)))
            .with_feature_id("left_inlet");
        let next = SdfNodeMetadata::new("profile_duct")
            .with_support_bounds(SdfBounds::new(
                Vec3::new(-1.0, -3.0, -2.0),
                Vec3::new(12.0, 3.0, 3.0),
            ))
            .with_feature_id("left_inlet")
            .with_feature_id("duct_path");

        let edit = GeometryEdit::metadata_changed(
            Some("left_inlet_profile".to_string()),
            &previous,
            &next,
            0.75,
        )
        .expect("bounded metadata changes should emit dirty regions");

        assert_eq!(edit.kind, GeometryEditKind::Feature);
        assert_eq!(edit.feature_id.as_deref(), Some("left_inlet_profile"));
        assert_eq!(edit.dirty_region.source, DirtyRegionSource::FeatureEdit);
        assert_eq!(
            edit.dirty_region.bounds,
            SdfBounds::new(Vec3::new(-1.0, -3.0, -2.0), Vec3::new(12.0, 3.0, 3.0))
        );
        assert_eq!(edit.dirty_region.halo, 0.75);
        assert_eq!(
            edit.dirty_region.feature_ids,
            vec![
                "left_inlet_profile".to_string(),
                "left_inlet".to_string(),
                "duct_path".to_string(),
            ]
        );
    }

    #[test]
    fn metadata_changed_localizes_dirty_region_through_boolean_tree() {
        let left_previous: Arc<dyn Sdf> = Arc::new(Translate::new(
            Arc::new(Sphere::new(3.0)),
            Vec3::new(-20.0, 0.0, 0.0),
        ));
        let right_previous: Arc<dyn Sdf> = Arc::new(Translate::new(
            Arc::new(Sphere::new(3.0)),
            Vec3::new(20.0, 0.0, 0.0),
        ));
        let previous: Arc<dyn Sdf> = Arc::new(Union::new(left_previous, right_previous));

        let left_next: Arc<dyn Sdf> = Arc::new(Translate::new(
            Arc::new(Sphere::new(3.0)),
            Vec3::new(-20.0, 0.0, 0.0),
        ));
        let right_next: Arc<dyn Sdf> = Arc::new(Translate::new(
            Arc::new(Sphere::new(4.0)),
            Vec3::new(20.0, 0.0, 0.0),
        ));
        let next: Arc<dyn Sdf> = Arc::new(Union::new(left_next, right_next));

        let edit = GeometryEdit::metadata_changed(
            Some("right_sphere_radius".to_string()),
            &previous.metadata(),
            &next.metadata(),
            0.5,
        )
        .expect("bounded metadata changes should emit a dirty region");

        assert!(edit.dirty_region.bounds.min.x > 15.0);
        assert!(edit.dirty_region.bounds.max.x < 25.0);
        assert!(
            !edit
                .dirty_region
                .bounds
                .contains(Vec3::new(-20.0, 0.0, 0.0))
        );
        assert!(edit.dirty_region.bounds.contains(Vec3::new(20.0, 0.0, 0.0)));
    }

    #[test]
    fn parameter_fingerprint_marks_same_envelope_shape_change_dirty() {
        let previous = unit_bounds_metadata(11);
        let next = unit_bounds_metadata(12);

        let edit = GeometryEdit::metadata_changed(
            Some("profile_control_points".to_string()),
            &previous,
            &next,
            0.25,
        )
        .expect("fingerprint-only parameter changes should emit a dirty region");

        assert_eq!(
            edit.dirty_region.bounds,
            SdfBounds::new(Vec3::splat(-1.0), Vec3::splat(1.0))
        );
        assert_eq!(edit.dirty_region.halo, 0.25);
    }

    #[test]
    fn min_feature_size_propagates_through_dependencies_and_dirty_regions() {
        let previous = SdfNodeMetadata::new("wing_shell")
            .with_support_bounds(SdfBounds::new(Vec3::splat(-2.0), Vec3::splat(2.0)))
            .with_dependency(
                "child",
                SdfNodeMetadata::new("airfoil")
                    .with_support_bounds(SdfBounds::new(Vec3::splat(-1.0), Vec3::splat(1.0)))
                    .with_min_feature_size(1.0),
            );
        let next = SdfNodeMetadata::new("wing_shell")
            .with_support_bounds(SdfBounds::new(Vec3::splat(-2.0), Vec3::splat(2.0)))
            .with_dependency(
                "child",
                SdfNodeMetadata::new("airfoil")
                    .with_support_bounds(SdfBounds::new(Vec3::splat(-1.0), Vec3::splat(1.0)))
                    .with_min_feature_size(0.5),
            )
            .with_parameter_fingerprint(12);

        let edit = GeometryEdit::metadata_changed(
            Some("trailing_edge".to_string()),
            &previous,
            &next,
            0.5,
        )
        .expect("feature-sized metadata changes should emit a dirty region");

        assert_eq!(previous.min_feature_size, Some(1.0));
        assert_eq!(next.min_feature_size, Some(0.5));
        assert_eq!(edit.dirty_region.recommended_grid_spacing, Some(0.25));
    }

    #[test]
    fn smooth_boolean_child_change_marks_sibling_interface_band_dirty() {
        let right_previous: Arc<dyn Sdf> = Arc::new(Translate::new(
            Arc::new(Sphere::new(2.0)),
            Vec3::new(4.0, 0.0, 0.0),
        ));
        let right_next: Arc<dyn Sdf> = Arc::new(Translate::new(
            Arc::new(Sphere::new(2.0)),
            Vec3::new(4.0, 0.0, 0.0),
        ));
        let previous: Arc<dyn Sdf> = Arc::new(SmoothUnion::new(
            Arc::new(Sphere::new(2.0)),
            right_previous,
            1.0,
        ));
        let next: Arc<dyn Sdf> = Arc::new(SmoothUnion::new(
            Arc::new(Sphere::new(2.2)),
            right_next,
            1.0,
        ));

        let edit = GeometryEdit::metadata_changed(
            Some("fuselage_wing_blend".to_string()),
            &previous.metadata(),
            &next.metadata(),
            0.5,
        )
        .expect("smooth boolean child changes should emit an interface-aware dirty region");

        assert!(
            edit.dirty_region.bounds.max.x > 3.0,
            "dirty region should extend beyond the changed child into the sibling blend interface"
        );
        assert!(
            edit.dirty_region.bounds.max.x < 3.4,
            "dirty region should stay localized to the blend interaction band"
        );
    }

    #[test]
    fn smooth_boolean_radius_change_marks_child_interface_dirty() {
        let previous: Arc<dyn Sdf> = Arc::new(SmoothUnion::new(
            Arc::new(Sphere::new(3.0)),
            Arc::new(Translate::new(
                Arc::new(Sphere::new(3.0)),
                Vec3::new(5.0, 0.0, 0.0),
            )),
            1.0,
        ));
        let next: Arc<dyn Sdf> = Arc::new(SmoothUnion::new(
            Arc::new(Sphere::new(3.0)),
            Arc::new(Translate::new(
                Arc::new(Sphere::new(3.0)),
                Vec3::new(5.0, 0.0, 0.0),
            )),
            2.0,
        ));

        let edit = GeometryEdit::metadata_changed(
            Some("center_blend".to_string()),
            &previous.metadata(),
            &next.metadata(),
            0.5,
        )
        .expect("smoothness-only metadata changes should emit a dirty region");

        assert!(
            edit.dirty_region.bounds.min.x > -3.1,
            "dirty region should not start at the full left sphere support"
        );
        assert!(
            edit.dirty_region.bounds.max.x < 8.1,
            "dirty region should not extend to the full right sphere support"
        );
        assert!(
            edit.dirty_region.bounds.contains(Vec3::new(2.5, 0.0, 0.0)),
            "dirty region should cover the child interface"
        );
    }

    #[test]
    fn backend_edit_helper_advances_existing_conditioned_cache() {
        let previous = condition_sdf_for_backend(Arc::new(Sphere::new(5.0)));
        assert!(
            conditioned_kernel_ref(&previous)
                .expect("sphere should be runtime conditionable")
                .conditioning()
                .dirty_history()
                .is_empty()
        );

        let next = condition_sdf_after_backend_edit(Some(&previous), Arc::new(Sphere::new(4.5)));
        let kernel = conditioned_kernel_ref(&next).expect("updated sphere should stay conditioned");

        assert_eq!(kernel.conditioning().dirty_history().len(), 1);
        assert!(matches!(
            kernel
                .conditioning()
                .last_diagnostics()
                .unwrap()
                .cache_state,
            ConditionedCacheState::Ready | ConditionedCacheState::Partial
        ));
    }

    #[test]
    fn backend_edit_helper_rebuilds_when_new_graph_has_unchanged_metadata() {
        let previous = condition_sdf_for_backend(Arc::new(FixedMetadataPlane { offset: 0.0 }));
        let next = condition_sdf_after_backend_edit(
            Some(&previous),
            Arc::new(FixedMetadataPlane { offset: 1.5 }),
        );
        let kernel = conditioned_kernel_ref(&next).expect("bounded plane should stay conditioned");
        let point = Vec3::new(2.0, 0.0, 0.0);

        assert!(
            (kernel.distance(point) - 0.5).abs() < 0.25,
            "new graph with unchanged metadata must not reuse stale conditioned samples"
        );
    }

    #[test]
    fn metadata_changed_without_bounds_requires_fallback() {
        let previous = SdfNodeMetadata::new("profile_duct");
        let next = SdfNodeMetadata::new("profile_duct")
            .with_support_bounds(SdfBounds::new(Vec3::ZERO, Vec3::ONE));

        assert!(GeometryEdit::metadata_changed(None, &previous, &next, 0.5).is_none());
        assert!(previous.support_change_bounds(&next).is_none());
    }

    #[test]
    fn metadata_changed_with_unknown_quality_requires_fallback() {
        let previous = SdfNodeMetadata::new("profile_duct")
            .with_support_bounds(SdfBounds::new(Vec3::ZERO, Vec3::ONE))
            .with_bound_quality(BoundQuality::Unknown);
        let next = SdfNodeMetadata::new("profile_duct")
            .with_support_bounds(SdfBounds::new(Vec3::ZERO, Vec3::splat(2.0)));

        assert!(GeometryEdit::metadata_changed(None, &previous, &next, 0.5).is_none());
        assert!(previous.support_change_bounds(&next).is_none());
    }

    #[test]
    fn support_bound_quality_distinguishes_exact_conservative_and_estimated() {
        let exact = SdfNodeMetadata::new("sphere")
            .with_support_bounds(SdfBounds::new(Vec3::splat(-1.0), Vec3::splat(1.0)));
        let conservative = SdfNodeMetadata::new("sweep")
            .with_support_bounds(SdfBounds::new(Vec3::ZERO, Vec3::ONE))
            .with_approximate_bounds();
        let estimated = SdfNodeMetadata::new("mesh")
            .with_estimated_bounds(SdfBounds::new(Vec3::ZERO, Vec3::ONE));
        let unknown = SdfNodeMetadata::unknown();

        assert_eq!(exact.bound_quality, BoundQuality::Exact);
        assert!(exact.is_conditionable());
        assert_eq!(conservative.bound_quality, BoundQuality::Conservative);
        assert!(conservative.is_conditionable());
        assert_eq!(estimated.bound_quality, BoundQuality::Estimated);
        assert!(estimated.is_conditionable());
        assert_eq!(unknown.bound_quality, BoundQuality::Unknown);
        assert!(!unknown.is_conditionable());
    }

    #[test]
    fn conditionable_metadata_audit_accepts_bounded_feature_tree() {
        let tagged = FeatureTaggedSdf::new("wing_root", Arc::new(Sphere::new(3.0)));
        let report = audit_conditionable_geometry(&tagged.metadata());

        assert!(report.is_clean(), "{report:?}");
        assert!(assert_conditionable_geometry(&tagged.metadata()).is_ok());
    }

    #[test]
    fn conditionable_metadata_audit_reports_unknown_child() {
        let metadata = SdfNodeMetadata::new("feature")
            .with_support_bounds(SdfBounds::new(Vec3::splat(-1.0), Vec3::splat(1.0)))
            .with_dependency("child", SdfNodeMetadata::unknown());

        let report = audit_conditionable_geometry(&metadata);

        assert_eq!(report.issues.len(), 1);
        assert_eq!(report.issues[0].path, "root/child[0]");
        assert_eq!(
            report.issues[0].issue,
            MetadataAuditIssueKind::MissingSupportBounds
        );
        assert!(assert_conditionable_geometry(&metadata).is_err());
    }

    #[test]
    fn conditionable_metadata_audit_reports_unknown_bound_quality() {
        let metadata = SdfNodeMetadata::new("bounded_unknown")
            .with_support_bounds(SdfBounds::new(Vec3::ZERO, Vec3::ONE))
            .with_bound_quality(BoundQuality::Unknown);

        let report = audit_conditionable_geometry(&metadata);

        assert_eq!(report.issues.len(), 1);
        assert_eq!(
            report.issues[0].issue,
            MetadataAuditIssueKind::UnknownBoundQuality
        );
    }

    #[test]
    fn conditionable_metadata_audit_reports_invalid_bounds() {
        let metadata = SdfNodeMetadata::new("bad_node")
            .with_support_bounds(SdfBounds::new(Vec3::ONE, Vec3::ZERO));

        let report = audit_conditionable_geometry(&metadata);

        assert_eq!(report.issues.len(), 1);
        assert_eq!(
            report.issues[0].issue,
            MetadataAuditIssueKind::InvalidSupportBounds
        );
    }

    #[test]
    fn metadata_coverage_report_counts_locality_signals() {
        let tagged = FeatureTaggedSdf::new("wing.root", Arc::new(Sphere::new(3.0)));
        let blend = SmoothUnion::new(
            Arc::new(tagged),
            Arc::new(Translate::new(
                Arc::new(Sphere::new(2.0)),
                Vec3::new(4.0, 0.0, 0.0),
            )),
            1.5,
        );

        let coverage = summarize_conditioning_metadata(&blend.metadata());

        assert!(coverage.is_conditioning_ready(), "{coverage:?}");
        assert!(coverage.has_locality_hints(), "{coverage:?}");
        assert_eq!(coverage.node_kind_count("smooth_union"), 1);
        assert_eq!(coverage.node_kind_count("feature"), 1);
        assert_eq!(coverage.dependency_role_count("a"), 1);
        assert_eq!(coverage.dependency_role_count("b"), 1);
        assert!(coverage.total_node_count >= 5);
        assert!(coverage.unique_feature_id_count >= 1);
        assert!(coverage.nodes_with_parameter_fingerprint >= 3);
        assert_eq!(coverage.child_interface_node_count, 1);
        assert_eq!(coverage.smooth_interface_node_count, 1);
    }

    #[test]
    fn fast_sweeping_reconditions_scaled_plane_samples() {
        let bounds = SdfBounds::new(Vec3::splat(-4.0), Vec3::splat(4.0));
        let spacing = 1.0;
        let raw_samples: Vec<_> = sample_points(&bounds, spacing)
            .into_iter()
            .map(|point| point.x * 2.0)
            .collect();

        let (conditioned, report) =
            recondition_signed_distance_samples(&bounds, &raw_samples, spacing, 2.0);
        let (_, ny, nz) = grid_cell_counts(&bounds, spacing);
        let positive = sample_grid_index(7, 4, 4, ny, nz);
        let negative = sample_grid_index(1, 4, 4, ny, nz);

        assert!(report.used_fast_sweeping);
        assert!(report.anchor_count > 0);
        assert!(report.iteration_count > 0);
        assert_eq!(raw_samples[positive], 6.0);
        assert!((conditioned[positive] - 3.0).abs() < 0.05);
        assert_eq!(raw_samples[negative], -6.0);
        assert!((conditioned[negative] + 3.0).abs() < 0.05);
    }

    #[test]
    fn projection_anchors_recondition_same_sign_near_interface_samples() {
        let bounds = SdfBounds::new(Vec3::new(0.5, -1.0, -1.0), Vec3::new(3.5, 1.0, 1.0));
        let spacing = 1.0;
        let raw_samples: Vec<_> = sample_points(&bounds, spacing)
            .into_iter()
            .map(|point| point.x * 2.0)
            .collect();

        let (conditioned, report) =
            recondition_signed_distance_samples(&bounds, &raw_samples, spacing, 8.0);
        let (_, ny, nz) = grid_cell_counts(&bounds, spacing);
        let first = sample_grid_index(0, 1, 1, ny, nz);
        let last = sample_grid_index(3, 1, 1, ny, nz);

        assert_eq!(report.projection_anchor_count, raw_samples.len());
        assert_eq!(raw_samples[first], 1.0);
        assert!((conditioned[first] - 0.5).abs() < 1.0e-6);
        assert_eq!(raw_samples[last], 7.0);
        assert!((conditioned[last] - 3.5).abs() < 1.0e-6);
    }

    #[test]
    fn conditioned_model_reconditions_scaled_sdf_cache() {
        let scaled = ScaledSdf::new(Arc::new(Sphere::new(5.0)), 2.0);
        let mut model = ConditionedGeometryModel::new(
            SdfBounds::new(Vec3::splat(-8.0), Vec3::splat(8.0)),
            16.0,
            ConditioningPolicy::default(),
        );

        let summary = model.initialize_from_canonical(&scaled);
        let point = Vec3::new(7.0, 0.0, 0.0);
        let raw = scaled.distance(point);
        let conditioned = model
            .conditioned_distance(point)
            .expect("point should be inside the conditioned cache");

        assert_eq!(summary.update_mode, ConditioningUpdateMode::FullRebuild);
        assert_eq!(
            summary.reconditioning.block_count,
            summary.regenerated_block_count
        );
        assert!(summary.reconditioning.anchor_count > 0);
        assert!(summary.reconditioning.projection_anchor_count > 0);
        assert!(summary.reconditioning.max_iteration_count > 0);
        assert!((raw - 4.0).abs() < 1.0e-6);
        assert!(
            (conditioned - 2.0).abs() < 0.25,
            "expected fast-swept distance near 2.0, got {conditioned}"
        );
        assert!(
            model
                .blocks
                .values()
                .any(|block| block.reconditioning_anchor_count > 0
                    && block.reconditioning_iteration_count > 0)
        );
    }

    #[test]
    fn local_region_reconditions_same_sign_neighbor_block() {
        let plane = ScaledPlane { scale: 2.0 };
        let model = ConditionedGeometryModel::new(
            SdfBounds::new(Vec3::splat(-8.0), Vec3::splat(8.0)),
            4.0,
            ConditioningPolicy::default(),
        );
        let interface_block = BlockIndex::new(2, 2, 2);
        let same_sign_block = BlockIndex::new(3, 2, 2);
        let point = Vec3::new(7.0, 1.0, 1.0);
        let isolated =
            model.regenerate_block(&plane, same_sign_block, ConditionedCacheState::Ready);
        let isolated_distance = isolated
            .interpolated_distance(point, model.policy.grid_spacing)
            .expect("point should be covered by the isolated block");

        let region_blocks = model.regenerate_block_regions(
            &plane,
            &[interface_block, same_sign_block],
            ConditionedCacheState::Ready,
        );
        let shared_region_block = region_blocks
            .iter()
            .find(|block| block.index == same_sign_block)
            .expect("same-sign block should be regenerated from the shared region");
        let shared_region_distance = shared_region_block
            .interpolated_distance(point, model.policy.grid_spacing)
            .expect("point should be covered by the shared-region block");

        assert_eq!(isolated.reconditioning_anchor_count, 0);
        assert!((isolated_distance - 14.0).abs() < 1.0e-6);
        assert!(shared_region_block.reconditioning_anchor_count > 0);
        assert!(
            (shared_region_distance - 7.0).abs() < 0.15,
            "expected shared-region fast sweeping near 7.0, got {shared_region_distance}"
        );
    }

    #[test]
    fn full_rebuild_reconditions_same_sign_blocks_across_region() {
        let plane = ScaledPlane { scale: 2.0 };
        let mut model = ConditionedGeometryModel::new(
            SdfBounds::new(Vec3::splat(-8.0), Vec3::splat(8.0)),
            4.0,
            ConditioningPolicy::default(),
        );
        let point = Vec3::new(7.0, 1.0, 1.0);

        let summary = model.initialize_from_canonical(&plane);
        let conditioned = model
            .conditioned_distance(point)
            .expect("point should be inside the initialized conditioned cache");
        let same_sign_block = model
            .blocks
            .get(&BlockIndex::new(3, 2, 2))
            .expect("same-sign block should exist");

        assert_eq!(summary.update_mode, ConditioningUpdateMode::FullRebuild);
        assert_eq!(
            summary.reconditioning.block_count,
            summary.regenerated_block_count
        );
        assert!(summary.reconditioning.reconditioned_block_count > 0);
        assert!(!same_sign_block.samples.is_empty());
        assert!(same_sign_block.reconditioning_anchor_count > 0);
        assert!(
            (conditioned - 7.0).abs() < 0.15,
            "expected full-region fast sweeping near 7.0, got {conditioned}"
        );
    }

    #[test]
    fn connected_block_components_keep_disjoint_regions_separate() {
        let components = connected_block_components(&[
            BlockIndex::new(0, 0, 0),
            BlockIndex::new(1, 0, 0),
            BlockIndex::new(8, 0, 0),
        ]);

        assert_eq!(components.len(), 2);
        assert_eq!(
            components[0],
            vec![BlockIndex::new(0, 0, 0), BlockIndex::new(1, 0, 0)]
        );
        assert_eq!(components[1], vec![BlockIndex::new(8, 0, 0)]);
    }

    #[test]
    fn budgeted_block_components_split_oversized_regions() {
        let policy = ConditioningPolicy {
            grid_spacing: 1.0,
            interface_band: 2.5,
            max_gradient_abs_error_p95: 0.15,
            max_interface_error: 1.0e-4,
            max_region_sample_count: 400,
            local_validation_mode: LocalValidationMode::LocalCacheQuality,
            ..ConditioningPolicy::default()
        };
        let model = ConditionedGeometryModel::new(
            SdfBounds::new(Vec3::splat(-8.0), Vec3::splat(8.0)),
            4.0,
            policy,
        );
        let indexes = vec![
            BlockIndex::new(0, 2, 2),
            BlockIndex::new(1, 2, 2),
            BlockIndex::new(2, 2, 2),
            BlockIndex::new(3, 2, 2),
        ];

        let components = model.budgeted_block_components(&indexes);
        let covered_block_count: usize = components.iter().map(Vec::len).sum();

        assert!(components.len() > 1);
        assert_eq!(covered_block_count, indexes.len());
        for component in components {
            let sample_count = model.reconditioning_region_sample_count(&component);
            assert!(
                component.len() == 1 || sample_count <= model.policy.max_region_sample_count,
                "component {component:?} used {sample_count} samples"
            );
        }
    }

    #[test]
    fn oversized_region_uses_sparse_narrow_band_reconditioning() {
        let policy = ConditioningPolicy {
            grid_spacing: 1.0,
            interface_band: 2.5,
            max_gradient_abs_error_p95: 0.15,
            max_interface_error: 1.0e-4,
            max_region_sample_count: 100,
            local_validation_mode: LocalValidationMode::LocalCacheQuality,
            ..ConditioningPolicy::default()
        };
        let region = ReconditionedSampleRegion::from_sdf(
            &ScaledPlane { scale: 2.0 },
            SdfBounds::new(Vec3::splat(-8.0), Vec3::splat(8.0)),
            &policy,
        );
        let near_interface = Vec3::new(2.0, 0.0, 0.0);
        let far_from_band = Vec3::new(7.0, 0.0, 0.0);

        let conditioned = region
            .sample_conditioned(near_interface, policy.grid_spacing)
            .expect("near-interface narrow-band sample should be stored");

        assert!(region.report.used_narrow_band);
        assert!(region.report.used_fast_sweeping);
        assert!(region.report.sample_count > policy.max_region_sample_count);
        assert!(region.report.narrow_band_sample_count < region.report.sample_count);
        assert!((conditioned - 2.0).abs() < 0.15);
        assert_eq!(
            region.sample_conditioned(far_from_band, policy.grid_spacing),
            None
        );
    }

    #[test]
    fn oversized_single_block_reports_narrow_band_cache_stats() {
        let policy = ConditioningPolicy {
            grid_spacing: 1.0,
            interface_band: 2.5,
            max_gradient_abs_error_p95: 0.15,
            max_interface_error: 1.0e-4,
            max_region_sample_count: 100,
            local_validation_mode: LocalValidationMode::LocalCacheQuality,
            ..ConditioningPolicy::default()
        };
        let mut model = ConditionedGeometryModel::new(
            SdfBounds::new(Vec3::splat(-8.0), Vec3::splat(8.0)),
            16.0,
            policy,
        );

        let summary = model.initialize_from_canonical(&ScaledPlane { scale: 2.0 });
        let near_interface = model
            .conditioned_distance(Vec3::new(2.0, 0.0, 0.0))
            .expect("near-interface point should be inside the cache");
        let far_from_band = model
            .conditioned_distance(Vec3::new(7.0, 0.0, 0.0))
            .expect("far point should fall back to canonical samples inside the block");

        assert_eq!(summary.regenerated_block_count, 1);
        assert_eq!(summary.reconditioning.narrow_band_block_count, 1);
        assert!(summary.reconditioning.narrow_band_sample_count > 0);
        assert!(summary.reconditioning.narrow_band_sample_count < 17 * 17 * 17);
        assert!((near_interface - 2.0).abs() < 0.15);
        assert!((far_from_band - 14.0).abs() < 1.0e-6);
    }

    #[test]
    fn local_update_matches_full_when_dirty_region_covers_edit() {
        let before = Sphere::new(10.0);
        let after = Sphere::new(11.0);
        let domain = SdfBounds::new(Vec3::splat(-16.0), Vec3::splat(16.0));
        let dirty = DirtyRegion::new(
            DirtyRegionSource::ParameterEdit,
            SdfBounds::new(Vec3::splat(-12.0), Vec3::splat(12.0)),
        )
        .with_halo(2.0);
        let policy = ConditioningPolicy {
            grid_spacing: 1.0,
            interface_band: 2.0,
            max_gradient_abs_error_p95: 0.2,
            max_interface_error: 1.0e-6,
            max_region_sample_count: ConditioningPolicy::DEFAULT_MAX_REGION_SAMPLE_COUNT,
            local_validation_mode: LocalValidationMode::FullComparison,
            ..ConditioningPolicy::default()
        };

        let diagnostics = compare_local_update_to_full(&before, &after, &domain, &dirty, &policy);

        assert!(diagnostics.accepted(&policy), "{diagnostics:?}");
        assert!(diagnostics.conditioned_sample_count < diagnostics.grid_point_count);
        assert!(diagnostics.changed_near_interface_count > 0);
        assert_eq!(diagnostics.max_abs_error_inside_update, 0.0);
        assert_eq!(diagnostics.max_abs_error_near_interface, 0.0);
        assert_eq!(diagnostics.sign_mismatch_count_near_interface, 0);
        assert_eq!(diagnostics.unexpected_sign_flip_count_outside_update, 0);
    }

    #[test]
    fn local_update_rejects_undercovered_dirty_region() {
        let before = Sphere::new(10.0);
        let after = Sphere::new(12.0);
        let domain = SdfBounds::new(Vec3::splat(-16.0), Vec3::splat(16.0));
        let dirty = DirtyRegion::new(
            DirtyRegionSource::ParameterEdit,
            SdfBounds::new(Vec3::splat(-4.0), Vec3::splat(4.0)),
        )
        .with_halo(1.0);
        let policy = ConditioningPolicy {
            grid_spacing: 1.0,
            interface_band: 2.0,
            max_gradient_abs_error_p95: 0.2,
            max_interface_error: 1.0e-6,
            max_region_sample_count: ConditioningPolicy::DEFAULT_MAX_REGION_SAMPLE_COUNT,
            local_validation_mode: LocalValidationMode::FullComparison,
            ..ConditioningPolicy::default()
        };

        let diagnostics = compare_local_update_to_full(&before, &after, &domain, &dirty, &policy);

        assert_eq!(diagnostics.cache_state, ConditionedCacheState::Rejected);
        assert!(!diagnostics.accepted(&policy));
        assert!(diagnostics.max_abs_error_near_interface > policy.max_interface_error);
        assert!(diagnostics.unexpected_sign_flip_count_outside_update > 0);
    }

    #[test]
    fn box_corner_edit_reports_finite_gradient_metrics() {
        let before = SdfBox::new(Vec3::new(8.0, 6.0, 4.0));
        let after = SdfBox::new(Vec3::new(9.0, 7.0, 5.0));
        let domain = SdfBounds::new(Vec3::splat(-14.0), Vec3::splat(14.0));
        let dirty = DirtyRegion::new(
            DirtyRegionSource::FeatureEdit,
            SdfBounds::new(Vec3::splat(-10.0), Vec3::splat(10.0)),
        )
        .with_halo(2.0);
        let policy = ConditioningPolicy {
            grid_spacing: 1.0,
            interface_band: 2.0,
            max_gradient_abs_error_p95: 0.5,
            max_interface_error: 1.0e-6,
            max_region_sample_count: ConditioningPolicy::DEFAULT_MAX_REGION_SAMPLE_COUNT,
            local_validation_mode: LocalValidationMode::FullComparison,
            ..ConditioningPolicy::default()
        };

        let diagnostics = compare_local_update_to_full(&before, &after, &domain, &dirty, &policy);

        assert!(diagnostics.accepted(&policy), "{diagnostics:?}");
        assert!(diagnostics.gradient_abs_error_p50.is_finite());
        assert!(diagnostics.gradient_abs_error_p95.is_finite());
        assert!(diagnostics.gradient_abs_error_max.is_finite());
    }

    #[test]
    fn conditioned_model_initializes_all_blocks() {
        let sphere = Sphere::new(10.0);
        let mut model = ConditionedGeometryModel::new(
            SdfBounds::new(Vec3::splat(-16.0), Vec3::splat(16.0)),
            8.0,
            ConditioningPolicy::default(),
        );

        let summary = model.initialize_from_canonical(&sphere);

        assert_eq!(model.generation(), 1);
        assert_eq!(model.block_count(), 64);
        assert_eq!(summary.update_mode, ConditioningUpdateMode::FullRebuild);
        assert_eq!(summary.regenerated_block_count, 64);
        assert_eq!(summary.cache_state, ConditionedCacheState::Ready);
    }

    #[test]
    fn conditioned_storage_stats_account_for_dense_blocks() {
        let sphere = Sphere::new(10.0);
        let mut model = ConditionedGeometryModel::new(
            SdfBounds::new(Vec3::splat(-16.0), Vec3::splat(16.0)),
            8.0,
            ConditioningPolicy::default(),
        );
        model.initialize_from_canonical(&sphere);

        let stats = model.storage_stats();

        assert_eq!(stats.block_count, model.block_count());
        assert_eq!(stats.adaptive_region_count, 0);
        assert!(stats.block_sample_count > 0);
        assert_eq!(
            stats.block_sample_bytes,
            stats.block_sample_count * std::mem::size_of::<f32>()
        );
        assert_eq!(stats.total_stored_sample_count, stats.block_sample_count);
        assert_eq!(stats.total_stored_sample_bytes, stats.block_sample_bytes);
        assert!(stats.estimated_metadata_bytes > 0);
        assert!(stats.estimated_total_bytes >= stats.total_stored_sample_bytes);
    }

    #[test]
    fn conditioned_model_interpolates_cached_distance() {
        let sphere = Sphere::new(10.0);
        let mut model = ConditionedGeometryModel::new(
            SdfBounds::new(Vec3::splat(-16.0), Vec3::splat(16.0)),
            8.0,
            ConditioningPolicy::default(),
        );
        model.initialize_from_canonical(&sphere);

        let cached = model
            .conditioned_distance(Vec3::new(9.5, 0.0, 0.0))
            .expect("point should be inside the conditioned domain");

        assert!((cached - -0.5).abs() < 1.0e-6);
        assert_eq!(model.conditioned_distance(Vec3::splat(20.0)), None);
    }

    #[test]
    fn feature_tagged_sdf_exposes_feature_ownership_metadata() {
        let tagged = FeatureTaggedSdf::new("wing_root", Arc::new(Sphere::new(3.0)));

        let metadata = tagged.metadata();

        assert_eq!(metadata.node_kind, "feature");
        assert_eq!(metadata.feature_ids, vec!["wing_root".to_string()]);
        assert_eq!(
            metadata.support_bounds,
            Some(SdfBounds::new(Vec3::splat(-3.0), Vec3::splat(3.0)))
        );
        assert_eq!(metadata.dependencies.len(), 1);
        assert_eq!(metadata.dependencies[0].role, "child");
        assert_eq!(tagged.distance(Vec3::new(3.0, 0.0, 0.0)), 0.0);
    }

    #[test]
    fn conditioned_kernel_initializes_live_cache_state() {
        let kernel = ConditionedGeometryKernel::new(
            Arc::new(Sphere::new(10.0)),
            SdfBounds::new(Vec3::splat(-16.0), Vec3::splat(16.0)),
            8.0,
            ConditioningPolicy::default(),
        );

        assert_eq!(kernel.conditioning().generation(), 1);
        assert_eq!(kernel.conditioning().block_count(), 64);
        assert!((kernel.distance(Vec3::new(10.0, 0.0, 0.0))).abs() < 1.0e-6);
        let metadata = kernel.metadata();
        assert_eq!(metadata.node_kind, "sphere");
        let cache = metadata
            .conditioned_cache
            .expect("conditioned kernel metadata should expose live cache state");
        assert_eq!(cache.state, ConditionedCacheState::Ready);
        assert_eq!(cache.generation, 1);
        assert_eq!(cache.block_count, 64);
        assert_eq!(cache.grid_spacing, 1.0);
        assert!(cache.anchor_count > 0);
        assert!(
            (kernel
                .conditioned_distance(Vec3::new(10.0, 0.0, 0.0))
                .unwrap())
            .abs()
                < 1.0e-6
        );
    }

    #[test]
    fn initial_feature_sized_metadata_seeds_adaptive_region() {
        let kernel = ConditionedGeometryKernel::new(
            Arc::new(FeatureSizedSphere {
                radius: 1.0,
                min_feature_size: 0.5,
            }),
            SdfBounds::new(Vec3::splat(-4.0), Vec3::splat(4.0)),
            4.0,
            ConditioningPolicy::default(),
        );
        let metadata = kernel.metadata();
        let cache = metadata
            .conditioned_cache
            .expect("conditioned kernel metadata should expose adaptive cache state");

        assert!(kernel.conditioning().adaptive_region_count() > 0);
        assert_eq!(
            cache.adaptive_region_count,
            kernel.conditioning().adaptive_region_count()
        );
        assert_eq!(cache.adaptive_min_grid_spacing, Some(0.25));
        assert!(kernel.conditioning().effective_grid_spacing(Vec3::ZERO) <= 0.25);
    }

    #[test]
    fn local_edit_with_recommended_spacing_creates_adaptive_overlay() {
        let plane = ScaledPlane { scale: 2.0 };
        let mut model = ConditionedGeometryModel::new(
            SdfBounds::new(Vec3::splat(-4.0), Vec3::splat(4.0)),
            4.0,
            ConditioningPolicy::default(),
        );
        model.initialize_from_canonical(&plane);
        let edit = GeometryEdit {
            kind: GeometryEditKind::Feature,
            feature_id: Some("thin_trailing_edge".to_string()),
            dirty_region: feature_dirty_region(
                "thin_trailing_edge",
                SdfBounds::new(Vec3::splat(-1.0), Vec3::splat(1.0)),
                0.25,
                0.25,
            ),
            previous_value: None,
            new_value: None,
        };

        let summary = model.apply_edit_and_recondition(&plane, &plane, edit);

        assert_eq!(
            summary.update_mode,
            ConditioningUpdateMode::LocalIncremental
        );
        assert_eq!(summary.adaptive.region_count, 1);
        assert_eq!(summary.diagnostics.confidence, 1.0);
        assert_eq!(summary.diagnostics.publication_confidence, 1.0);
        assert!(summary.diagnostics.quality_confidence.is_finite());
        assert!((0.0..=1.0).contains(&summary.diagnostics.quality_confidence));
        assert_eq!(model.adaptive_region_count(), 1);
        assert_eq!(model.effective_grid_spacing(Vec3::ZERO), 0.25);
        assert!(
            model
                .conditioned_distance(Vec3::new(1.0, 0.0, 0.0))
                .is_some()
        );
    }

    #[test]
    fn local_adaptive_refresh_uses_feature_bounds_not_block_halo() {
        let plane = ScaledPlane { scale: 2.0 };
        let mut model = ConditionedGeometryModel::new(
            SdfBounds::new(Vec3::splat(-8.0), Vec3::splat(8.0)),
            8.0,
            ConditioningPolicy::default(),
        );
        model.initialize_from_canonical(&plane);
        let dirty_region = feature_dirty_region(
            "thin_local_edge",
            SdfBounds::new(Vec3::splat(-1.0), Vec3::splat(1.0)),
            4.0,
            0.25,
        );
        let update_size = dirty_region.update_bounds().size();
        let edit = GeometryEdit {
            kind: GeometryEditKind::Feature,
            feature_id: Some("thin_local_edge".to_string()),
            dirty_region,
            previous_value: None,
            new_value: None,
        };

        let summary = model.apply_edit_and_recondition(&plane, &plane, edit);

        assert_eq!(summary.adaptive.region_count, 1);
        let adaptive_size = model.adaptive_regions[0].bounds.size();
        assert!(adaptive_size.x < update_size.x * 0.5);
        assert!(adaptive_size.y < update_size.y * 0.5);
        assert!(adaptive_size.z < update_size.z * 0.5);
    }

    #[test]
    fn adaptive_overlays_are_replaced_for_intersecting_dirty_regions() {
        let plane = ScaledPlane { scale: 2.0 };
        let mut model = ConditionedGeometryModel::new(
            SdfBounds::new(Vec3::splat(-4.0), Vec3::splat(4.0)),
            4.0,
            ConditioningPolicy::default(),
        );
        model.initialize_from_canonical(&plane);
        for feature_id in ["first_edit", "second_edit"] {
            let edit = GeometryEdit {
                kind: GeometryEditKind::Feature,
                feature_id: Some(feature_id.to_string()),
                dirty_region: feature_dirty_region(
                    feature_id,
                    SdfBounds::new(Vec3::splat(-1.0), Vec3::splat(1.0)),
                    0.25,
                    0.25,
                ),
                previous_value: None,
                new_value: None,
            };
            model.apply_edit_and_recondition(&plane, &plane, edit);
        }

        assert_eq!(model.adaptive_region_count(), 1);
        assert_eq!(model.adaptive_stats().feature_id_count, 1);
        assert_eq!(model.adaptive_stats().requested_feature_id_count, 1);
        assert_eq!(model.adaptive_feature_coverage(), 1.0);
    }

    #[test]
    fn adaptive_allocator_reserves_budget_for_uncovered_feature_ids() {
        let plane = ScaledPlane { scale: 2.0 };
        let policy = ConditioningPolicy {
            adaptive_max_regions: 2,
            adaptive_max_region_sample_count: 2_048,
            ..ConditioningPolicy::default()
        };
        let mut model = ConditionedGeometryModel::new(
            SdfBounds::new(Vec3::splat(-8.0), Vec3::splat(8.0)),
            8.0,
            policy,
        );
        model.initialize_from_canonical(&plane);

        let broad_fine_edit = GeometryEdit {
            kind: GeometryEditKind::Feature,
            feature_id: Some("broad_fine_edge".to_string()),
            dirty_region: feature_dirty_region(
                "broad_fine_edge",
                SdfBounds::new(Vec3::splat(-4.0), Vec3::splat(4.0)),
                0.25,
                0.25,
            ),
            previous_value: None,
            new_value: None,
        };
        let second_feature_edit = GeometryEdit {
            kind: GeometryEditKind::Feature,
            feature_id: Some("second_edge".to_string()),
            dirty_region: feature_dirty_region(
                "second_edge",
                SdfBounds::new(Vec3::new(4.0, -1.0, -1.0), Vec3::new(6.0, 1.0, 1.0)),
                0.5,
                0.25,
            ),
            previous_value: None,
            new_value: None,
        };

        let summary = model.apply_edits_and_recondition(
            &plane,
            &plane,
            GeometryEditTransaction::new(vec![broad_fine_edit, second_feature_edit]),
        );

        assert_eq!(summary.adaptive.region_count, 2);
        assert_eq!(summary.adaptive.feature_id_count, 2);
        assert_eq!(summary.adaptive.requested_feature_id_count, 2);
        assert_eq!(model.adaptive_feature_coverage(), 1.0);
        assert!(model.adaptive_regions.iter().any(|region| {
            region.feature_ids == ["broad_fine_edge".to_string()]
                && region.region.report.used_narrow_band
        }));
    }

    #[test]
    fn oversized_adaptive_overlay_uses_sparse_narrow_band_when_scan_is_bounded() {
        let plane = ScaledPlane { scale: 2.0 };
        let policy = ConditioningPolicy {
            adaptive_max_region_sample_count: 64,
            ..ConditioningPolicy::default()
        };
        let mut model = ConditionedGeometryModel::new(
            SdfBounds::new(Vec3::splat(-8.0), Vec3::splat(8.0)),
            8.0,
            policy,
        );
        model.initialize_from_canonical(&plane);
        let edit = GeometryEdit {
            kind: GeometryEditKind::Feature,
            feature_id: Some("broad_fine_edit".to_string()),
            dirty_region: feature_dirty_region(
                "broad_fine_edit",
                SdfBounds::new(Vec3::splat(-4.0), Vec3::splat(4.0)),
                1.0,
                0.25,
            ),
            previous_value: None,
            new_value: None,
        };

        let summary = model.apply_edit_and_recondition(&plane, &plane, edit);

        assert_eq!(
            summary.update_mode,
            ConditioningUpdateMode::LocalIncremental
        );
        assert_eq!(summary.adaptive.region_count, 1);
        assert_eq!(summary.adaptive.narrow_band_region_count, 1);
        assert!(summary.adaptive.narrow_band_sample_count > 0);
        assert!(summary.adaptive.narrow_band_sample_count < summary.adaptive.sample_count);
        assert_eq!(summary.adaptive.requested_region_count, 1);
        assert_eq!(summary.adaptive.skipped_region_count, 0);
        assert_eq!(summary.adaptive.oversized_region_count, 0);
        assert_eq!(summary.adaptive.requested_min_grid_spacing, Some(0.25));
        assert_eq!(summary.adaptive.requested_feature_id_count, 1);
        assert_eq!(model.adaptive_feature_coverage(), 1.0);
        assert_eq!(model.adaptive_region_count(), 1);
        assert!(
            model
                .conditioned_distance(Vec3::new(2.0, 0.0, 0.0))
                .is_some()
        );
    }

    #[test]
    fn oversized_adaptive_overlay_is_skipped_when_sparse_scan_is_too_broad() {
        let plane = ScaledPlane { scale: 2.0 };
        let policy = ConditioningPolicy {
            adaptive_max_region_sample_count: 64,
            adaptive_max_sparse_scan_sample_count: 128,
            ..ConditioningPolicy::default()
        };
        let mut model = ConditionedGeometryModel::new(
            SdfBounds::new(Vec3::splat(-8.0), Vec3::splat(8.0)),
            8.0,
            policy,
        );
        model.initialize_from_canonical(&plane);
        let edit = GeometryEdit {
            kind: GeometryEditKind::Feature,
            feature_id: Some("broad_fine_edit".to_string()),
            dirty_region: feature_dirty_region(
                "broad_fine_edit",
                SdfBounds::new(Vec3::splat(-4.0), Vec3::splat(4.0)),
                1.0,
                0.25,
            ),
            previous_value: None,
            new_value: None,
        };

        let summary = model.apply_edit_and_recondition(&plane, &plane, edit);

        assert_eq!(summary.adaptive.region_count, 0);
        assert_eq!(summary.adaptive.requested_region_count, 1);
        assert_eq!(summary.adaptive.skipped_region_count, 1);
        assert_eq!(summary.adaptive.oversized_region_count, 1);
        assert_eq!(summary.adaptive.requested_feature_id_count, 1);
        assert_eq!(model.adaptive_feature_coverage(), 0.0);
    }

    #[test]
    fn conditioned_kernel_can_initialize_from_conditionable_metadata() {
        let kernel = ConditionedGeometryKernel::from_conditionable_metadata(
            Arc::new(Sphere::new(5.0)),
            4.0,
            ConditioningPolicy::default(),
            2.0,
        )
        .expect("sphere metadata should provide conditionable bounds");

        assert!(
            kernel
                .conditioning()
                .domain()
                .contains(Vec3::new(5.0, 0.0, 0.0))
        );
        assert!(kernel.conditioning().block_count() > 0);
        assert!(
            (kernel
                .conditioned_distance(Vec3::new(5.0, 0.0, 0.0))
                .unwrap())
            .abs()
                < 0.75
        );
        assert!(
            (kernel
                .project_to_surface(Vec3::new(6.0, 0.0, 0.0), 1.0e-3, 12)
                .expect("projection should converge on bounded sphere")
                .x
                - 5.0)
                .abs()
                < 0.75
        );
    }

    #[test]
    fn conditioned_kernel_rejects_unknown_metadata_domain() {
        let kernel = ConditionedGeometryKernel::from_conditionable_metadata(
            Arc::new(ScaledPlane { scale: 2.0 }),
            4.0,
            ConditioningPolicy::default(),
            2.0,
        );

        assert!(kernel.is_none());
    }

    #[test]
    fn runtime_conditioning_policy_scales_to_sample_budget() {
        let support_bounds = SdfBounds::new(Vec3::ZERO, Vec3::new(2000.0, 400.0, 200.0));

        let policy = runtime_conditioning_policy_for_bounds(&support_bounds, 50_000);
        let domain = support_bounds.expanded(runtime_conditioning_domain_halo(&policy));

        assert!(policy.grid_spacing > 1.0);
        assert!(bounded_sample_count(&domain, policy.grid_spacing) <= 50_000);
        assert_eq!(policy.max_blocks_per_region, 1);
        assert!(!policy.allow_partial_local_updates);
        assert_eq!(policy.max_local_update_block_fraction, 0.25);
    }

    #[test]
    fn backend_conditioning_wraps_conditionable_runtime_sdf() {
        let raw: Arc<dyn Sdf> = Arc::new(ScaledSdf::new(Arc::new(Sphere::new(10.0)), 2.0));

        let conditioned = condition_sdf_for_backend(raw);

        let cached_distance = conditioned.distance(Vec3::new(7.0, 0.0, 0.0));
        assert!((cached_distance - -3.0).abs() < 0.25);
    }

    #[test]
    fn backend_conditioning_leaves_unbounded_runtime_sdf_evaluable() {
        let raw: Arc<dyn Sdf> = Arc::new(ScaledPlane { scale: 2.0 });

        let conditioned = condition_sdf_for_backend(raw);

        assert_eq!(conditioned.distance(Vec3::new(2.0, 0.0, 0.0)), 4.0);
    }

    #[test]
    fn background_scheduler_publishes_conditioned_kernel_result() {
        let scheduler = BackgroundConditioningScheduler::new();
        let id = scheduler.enqueue_rebuild(Arc::new(Sphere::new(5.0)));

        let summary = wait_for_background_job(&scheduler, id);

        assert_eq!(summary.state, ConditioningJobState::Ready);
        assert_eq!(summary.cache_state, ConditionedCacheState::Ready);
        assert_eq!(summary.update_mode, ConditioningUpdateMode::FullRebuild);
        assert_eq!(summary.generation, Some(1));
        assert!(summary.conditionable);

        let result = scheduler
            .take_latest_ready()
            .expect("ready job should publish latest conditioned result");
        assert_eq!(result.id, id);
        assert!(conditioned_kernel_ref(&result.sdf).is_some());
        assert!((result.sdf.distance(Vec3::new(5.0, 0.0, 0.0))).abs() < 0.75);
        assert!(scheduler.take_latest_ready().is_none());
    }

    #[test]
    fn background_scheduler_falls_back_when_edit_cache_is_not_ready() {
        let previous = condition_sdf_for_backend(Arc::new(Sphere::new(5.0)));
        let scheduler = BackgroundConditioningScheduler::new();
        let id = scheduler.enqueue_after_edit(Some(previous), Arc::new(Sphere::new(4.5)));

        let summary = wait_for_background_job(&scheduler, id);

        assert_eq!(summary.state, ConditioningJobState::Ready);
        assert_eq!(summary.update_mode, ConditioningUpdateMode::FullRebuild);
        assert_eq!(summary.cache_state, ConditionedCacheState::Ready);
        assert_eq!(summary.generation, Some(2));
        let result = scheduler
            .take_latest_ready()
            .expect("edit-aware job should publish a ready result");
        let kernel = conditioned_kernel_ref(&result.sdf)
            .expect("edit-aware background conditioning should return a conditioned kernel");
        assert_eq!(kernel.conditioning().generation(), 2);
        assert_eq!(kernel.conditioning().dirty_history().len(), 1);
    }

    #[test]
    fn background_scheduler_returns_direct_analytic_for_unconditionable_sdf() {
        let scheduler = BackgroundConditioningScheduler::new();
        let id = scheduler.enqueue_rebuild(Arc::new(ScaledPlane { scale: 2.0 }));

        let summary = wait_for_background_job(&scheduler, id);

        assert_eq!(summary.state, ConditioningJobState::Ready);
        assert_eq!(summary.cache_state, ConditionedCacheState::Unavailable);
        assert_eq!(summary.update_mode, ConditioningUpdateMode::DirectAnalytic);
        assert_eq!(summary.generation, None);
        assert!(!summary.conditionable);

        let result = scheduler
            .take_latest_ready()
            .expect("direct analytic job should still publish a usable result");
        assert!(conditioned_kernel_ref(&result.sdf).is_none());
        assert_eq!(result.sdf.distance(Vec3::new(2.0, 0.0, 0.0)), 4.0);
    }

    #[test]
    fn background_scheduler_coalesces_queued_edit_bursts_to_latest_job() {
        let scheduler =
            BackgroundConditioningScheduler::with_config(BackgroundConditioningSchedulerConfig {
                max_initial_samples: 2_000,
                coalesce_queued_jobs: true,
            });
        let first =
            scheduler.enqueue_rebuild(Arc::new(SlowSphere::new(5.0, Duration::from_millis(120))));
        let mut burst_ids = Vec::new();
        for radius in [5.1, 5.2, 5.3, 5.4, 5.5] {
            burst_ids.push(scheduler.enqueue_rebuild(Arc::new(Sphere::new(radius))));
        }
        let latest = *burst_ids.last().expect("burst should enqueue jobs");

        let latest_summary = wait_for_background_job(&scheduler, latest);

        assert_eq!(latest_summary.state, ConditioningJobState::Ready);
        let mut superseded_count = 0;
        let mut ready_non_latest_count = 0;
        for id in std::iter::once(first).chain(burst_ids[..burst_ids.len() - 1].iter().copied()) {
            let summary = wait_for_background_job(&scheduler, id);
            match summary.state {
                ConditioningJobState::Superseded => superseded_count += 1,
                ConditioningJobState::Ready => ready_non_latest_count += 1,
                state => panic!("burst job {id:?} reached unexpected state {state:?}"),
            }
        }
        assert!(
            superseded_count >= burst_ids.len() - 1,
            "at least the intermediate burst jobs should be superseded"
        );
        assert!(
            ready_non_latest_count <= 1,
            "only a job already running before the burst may finish before the latest job"
        );
        let result = scheduler
            .take_latest_ready()
            .expect("latest burst job should publish a result");
        assert_eq!(result.id, latest);
        let kernel = conditioned_kernel_ref(&result.sdf)
            .expect("latest burst result should be a conditioned kernel");
        assert!(
            (kernel.canonical_distance(Vec3::new(5.5, 0.0, 0.0))).abs() < 1.0e-6,
            "latest queued radius should be the published conditioned result"
        );
    }

    #[test]
    fn conditioned_kernel_distance_uses_conditioned_cache_by_default() {
        let kernel = ConditionedGeometryKernel::new(
            Arc::new(ScaledPlane { scale: 2.0 }),
            SdfBounds::new(Vec3::splat(-8.0), Vec3::splat(8.0)),
            4.0,
            ConditioningPolicy::default(),
        );
        let point = Vec3::new(7.0, 1.0, 1.0);
        let outside_domain = Vec3::new(12.0, 1.0, 1.0);
        let sdf_view: &dyn Sdf = &kernel;

        assert!((kernel.canonical_distance(point) - 14.0).abs() < 1.0e-6);
        assert!(
            (kernel.conditioned_distance(point).unwrap() - 7.0).abs() < 0.15,
            "expected conditioned cache near 7.0"
        );
        assert!(
            (kernel.distance(point) - 7.0).abs() < 0.15,
            "expected kernel distance to use conditioned cache"
        );
        assert!(
            (sdf_view.distance(point) - 7.0).abs() < 0.15,
            "expected Sdf trait distance to use conditioned cache"
        );
        assert_eq!(kernel.conditioned_distance(outside_domain), None);
        assert!((kernel.distance(outside_domain) - 24.0).abs() < 1.0e-6);
    }

    #[test]
    fn conditioned_kernel_runtime_distance_preserves_near_surface_canonical_detail() {
        let kernel = ConditionedGeometryKernel::new(
            Arc::new(ScaledPlane { scale: 0.5 }),
            SdfBounds::new(Vec3::splat(-8.0), Vec3::splat(8.0)),
            4.0,
            ConditioningPolicy::default(),
        );
        let point = Vec3::new(1.0, 1.0, 1.0);
        let canonical = kernel.canonical_distance(point);
        let raw_conditioned = kernel
            .raw_conditioned_distance(point)
            .expect("point should be inside the conditioned cache");
        let published_conditioned = kernel
            .conditioned_distance(point)
            .expect("point should be inside the published conditioned field");

        assert!((canonical - 0.5).abs() < 1.0e-6);
        assert!(
            (raw_conditioned - canonical).abs() > 0.1,
            "raw cache is allowed to recondition near-surface samples before publication"
        );
        assert!(
            (published_conditioned - canonical).abs() < 1.0e-6,
            "published conditioned distance should preserve canonical near-surface detail"
        );
        assert!(
            (kernel.distance(point) - canonical).abs() < 1.0e-6,
            "runtime distance should not publish a coarser near-surface cache value"
        );
    }

    #[test]
    fn cache_quality_diagnostics_reports_conditioned_scaled_plane_quality() {
        let kernel = ConditionedGeometryKernel::new(
            Arc::new(ScaledPlane { scale: 2.0 }),
            SdfBounds::new(Vec3::splat(-8.0), Vec3::splat(8.0)),
            4.0,
            ConditioningPolicy::default(),
        );

        let diagnostics = kernel
            .conditioning()
            .diagnose_cache_quality(kernel.canonical().as_ref());
        let gradient = kernel
            .conditioning()
            .conditioned_gradient(Vec3::new(7.0, 1.0, 1.0), 1.0)
            .expect("interior conditioned point should have a gradient");

        assert!(diagnostics.grid_point_count > 0);
        assert!(diagnostics.near_interface_sample_count > 0);
        assert!(diagnostics.gradient_sample_count > 0);
        assert!(diagnostics.projection_sample_count > 0);
        assert_eq!(diagnostics.sign_mismatch_count_near_interface, 0);
        assert!(diagnostics.max_surface_residual <= 1.0e-6);
        assert!(diagnostics.max_projection_residual <= 1.0e-6);
        assert!(diagnostics.distance_abs_error_max > 0.0);
        assert!(diagnostics.gradient_abs_error_p95 < 0.05, "{diagnostics:?}");
        assert!(diagnostics.confidence > 0.99, "{diagnostics:?}");
        assert!((gradient.length() - 1.0).abs() < 0.05);
    }

    #[test]
    fn kernel_projection_uses_conditioned_distance_and_normal() {
        let kernel = ConditionedGeometryKernel::new(
            Arc::new(ScaledPlane { scale: 2.0 }),
            SdfBounds::new(Vec3::splat(-8.0), Vec3::splat(8.0)),
            4.0,
            ConditioningPolicy::default(),
        );
        let query = Vec3::new(7.0, 1.0, 1.0);

        let normal = kernel.surface_normal(query, 1.0);
        let projected = kernel
            .project_to_surface(query, 1.0e-4, 8)
            .expect("conditioned projection should reach the plane");

        assert!(normal.dot(Vec3::X) > 0.99, "normal was {normal:?}");
        assert!(
            projected.x.abs() < 0.15,
            "expected projection near x=0, got {projected:?}"
        );
        assert!(
            kernel.distance(projected).abs() < 0.15,
            "projected point should be near the conditioned surface"
        );
        assert!(
            (kernel.canonical_distance(query) - 14.0).abs() < 1.0e-6,
            "canonical field remains scaled for debugging"
        );
    }

    #[test]
    fn conditioned_kernel_exposes_first_class_query_methods() {
        let kernel = ConditionedGeometryKernel::new(
            Arc::new(ScaledPlane { scale: 2.0 }),
            SdfBounds::new(Vec3::splat(-8.0), Vec3::splat(8.0)),
            4.0,
            ConditioningPolicy::default(),
        );

        let hit = kernel
            .ray_intersection(Vec3::new(7.0, 1.0, 1.0), Vec3::NEG_X, 20.0, 1.0e-4, 64)
            .expect("conditioned ray query should hit the plane");
        let closest = kernel
            .closest_surface_point(Vec3::new(7.0, 1.0, 1.0), 1.0e-4, 8)
            .expect("closest point query should project onto the plane");

        assert_eq!(
            kernel.inside_outside(Vec3::new(-1.0, 0.0, 0.0), 1.0e-4),
            InsideOutside::Inside
        );
        assert_eq!(
            kernel.inside_outside(Vec3::new(1.0, 0.0, 0.0), 1.0e-4),
            InsideOutside::Outside
        );
        assert_eq!(
            kernel.inside_outside(Vec3::ZERO, 1.0e-4),
            InsideOutside::OnSurface
        );
        assert!(
            hit.point.x.abs() < 0.15,
            "expected ray hit near x=0, got {hit:?}"
        );
        assert!((hit.distance - 7.0).abs() < 0.15);
        assert!(hit.normal.dot(Vec3::X) > 0.99);
        assert!(closest.x.abs() < 0.15);
    }

    #[test]
    fn conditioned_kernel_replaces_canonical_with_local_transaction() {
        let policy = ConditioningPolicy {
            grid_spacing: 1.0,
            interface_band: 2.0,
            max_gradient_abs_error_p95: 0.2,
            max_interface_error: 1.0e-6,
            max_region_sample_count: ConditioningPolicy::DEFAULT_MAX_REGION_SAMPLE_COUNT,
            local_validation_mode: LocalValidationMode::LocalCacheQuality,
            ..ConditioningPolicy::default()
        };
        let before_sphere = Sphere::new(10.0);
        let edit = before_sphere.edit_for_radius_change("sphere_radius", 11.0, 2.0);
        let mut kernel = ConditionedGeometryKernel::new(
            Arc::new(before_sphere),
            SdfBounds::new(Vec3::splat(-16.0), Vec3::splat(16.0)),
            8.0,
            policy,
        );

        let summary = kernel.replace_canonical_with_edit(Arc::new(Sphere::new(11.0)), edit);

        assert_eq!(
            summary.update_mode,
            ConditioningUpdateMode::LocalIncremental,
            "{:?}",
            summary.diagnostics
        );
        assert!(!summary.diagnostics.validated_against_full);
        assert!(!summary.full_rebuild_used);
        assert_eq!(kernel.conditioning().generation(), 2);
        assert_eq!(kernel.conditioning().dirty_history().len(), 1);
        assert!((kernel.distance(Vec3::new(11.0, 0.0, 0.0))).abs() < 1.0e-6);
        assert!(
            (kernel
                .conditioned_distance(Vec3::new(11.0, 0.0, 0.0))
                .unwrap())
            .abs()
                < 1.0e-6
        );
    }

    #[test]
    fn conditioned_kernel_exposes_geometry_service_queries() {
        let kernel = ConditionedGeometryKernel::new(
            Arc::new(Sphere::new(5.0)),
            SdfBounds::new(Vec3::splat(-6.0), Vec3::splat(6.0)),
            6.0,
            ConditioningPolicy {
                grid_spacing: 0.75,
                interface_band: 1.5,
                ..ConditioningPolicy::default()
            },
        );

        let surface = Vec3::new(5.0, 0.0, 0.0);
        let projected = kernel
            .project_to_surface(Vec3::new(5.8, 0.0, 0.0), 0.1, 32)
            .expect("projection should converge on sphere");
        let thickness = kernel
            .wall_thickness(Vec3::ZERO, Vec3::X, 20.0, 0.25)
            .expect("sphere center should report wall thickness across diameter");
        let cross_section = kernel.cross_section(0, 0.0, 64);
        let volume = kernel.volume(48);
        let wetted_area = kernel.wetted_area(48);
        let frontal_area = kernel.frontal_area(Vec3::X, 64);
        let curvature = kernel.curvature(surface, 0.5).abs();
        let principal = kernel.principal_curvature(surface, 0.5);

        assert!((projected.x - 5.0).abs() < 0.25);
        assert!((thickness - 10.0).abs() < 2.0, "thickness={thickness}");
        assert!(
            (cross_section.area - std::f32::consts::PI * 25.0).abs()
                / (std::f32::consts::PI * 25.0)
                < 0.20,
            "cross_section={cross_section:?}"
        );
        assert!(
            (volume - (4.0 / 3.0) * std::f32::consts::PI * 125.0).abs()
                / ((4.0 / 3.0) * std::f32::consts::PI * 125.0)
                < 0.25,
            "volume={volume}"
        );
        assert!(wetted_area > 0.0);
        assert!(
            (frontal_area - std::f32::consts::PI * 25.0).abs() / (std::f32::consts::PI * 25.0)
                < 0.30,
            "frontal_area={frontal_area}"
        );
        assert!(curvature.is_finite());
        assert!(principal.minimum.is_finite());
        assert!(principal.maximum.is_finite());
        assert!(kernel.local_feature_size(surface).is_some());
    }

    #[test]
    fn broad_strict_edit_skips_local_attempt() {
        let policy = ConditioningPolicy {
            grid_spacing: 1.0,
            interface_band: 2.0,
            max_gradient_abs_error_p95: 0.2,
            max_interface_error: 1.0e-6,
            max_region_sample_count: ConditioningPolicy::DEFAULT_MAX_REGION_SAMPLE_COUNT,
            local_validation_mode: LocalValidationMode::LocalCacheQuality,
            allow_partial_local_updates: false,
            max_local_update_block_fraction: 0.5,
            ..ConditioningPolicy::default()
        };
        let before_sphere = Sphere::new(10.0);
        let edit = before_sphere.edit_for_radius_change("sphere_radius", 11.0, 2.0);
        let mut kernel = ConditionedGeometryKernel::new(
            Arc::new(before_sphere),
            SdfBounds::new(Vec3::splat(-16.0), Vec3::splat(16.0)),
            8.0,
            policy,
        );
        let total_blocks = kernel.conditioning().block_count();

        let summary = kernel.replace_canonical_with_edit(Arc::new(Sphere::new(11.0)), edit);

        assert_eq!(summary.update_mode, ConditioningUpdateMode::FullRebuild);
        assert_eq!(
            summary.diagnostics.update_mode,
            ConditioningUpdateMode::FullRebuild
        );
        assert!(summary.full_rebuild_used);
        assert_eq!(summary.invalidated_block_count, total_blocks);
        assert_eq!(summary.regenerated_block_count, total_blocks);
        assert_eq!(summary.reconditioning.block_count, total_blocks);
        assert_eq!(kernel.conditioning().generation(), 2);
    }

    #[test]
    fn blend_radius_edit_reconditions_only_affected_blocks() {
        let left = Arc::new(Translate::new(
            Arc::new(Sphere::new(7.0)),
            Vec3::new(-5.0, 0.0, 0.0),
        ));
        let right = Arc::new(Translate::new(
            Arc::new(Sphere::new(7.0)),
            Vec3::new(5.0, 0.0, 0.0),
        ));
        let before = SmoothUnion::new(left.clone(), right.clone(), 2.0);
        let after = SmoothUnion::new(left, right, 4.0);
        let policy = ConditioningPolicy {
            grid_spacing: 1.0,
            interface_band: 2.0,
            max_gradient_abs_error_p95: 1.0,
            max_interface_error: 1.0e-5,
            max_region_sample_count: ConditioningPolicy::DEFAULT_MAX_REGION_SAMPLE_COUNT,
            local_validation_mode: LocalValidationMode::LocalCacheQuality,
            ..ConditioningPolicy::default()
        };
        let mut model = ConditionedGeometryModel::new(
            SdfBounds::new(Vec3::splat(-20.0), Vec3::splat(20.0)),
            8.0,
            policy,
        );
        model.initialize_from_canonical(&before);
        let total_blocks = model.block_count();
        let edit = before.edit_for_smoothness_change(
            "center_blend",
            SdfBounds::new(Vec3::new(-8.0, -8.0, -8.0), Vec3::new(8.0, 8.0, 8.0)),
            4.0,
            2.0,
        );

        let summary = model.apply_edits_and_recondition(
            &before,
            &after,
            GeometryEditTransaction::single(edit),
        );

        assert_eq!(
            summary.update_mode,
            ConditioningUpdateMode::LocalIncremental,
            "{:?}",
            summary.diagnostics
        );
        assert!(!summary.diagnostics.validated_against_full);
        assert!(!summary.full_rebuild_used);
        assert!(summary.regenerated_block_count > 0);
        assert!(summary.regenerated_block_count < total_blocks);
        assert_eq!(
            summary.reconditioning.block_count,
            summary.regenerated_block_count
        );
        assert_ne!(summary.cache_state, ConditionedCacheState::Rejected);
        assert_eq!(model.generation(), 2);
        assert_eq!(model.dirty_history().len(), 1);
    }

    #[test]
    fn edit_transaction_reconditions_disjoint_dirty_regions_once() {
        let before_left = Arc::new(Translate::new(
            Arc::new(Sphere::new(4.0)),
            Vec3::new(-10.0, 0.0, 0.0),
        ));
        let before_right = Arc::new(Translate::new(
            Arc::new(Sphere::new(4.0)),
            Vec3::new(10.0, 0.0, 0.0),
        ));
        let after_left = Arc::new(Translate::new(
            Arc::new(Sphere::new(5.0)),
            Vec3::new(-10.0, 0.0, 0.0),
        ));
        let after_right = Arc::new(Translate::new(
            Arc::new(Sphere::new(5.5)),
            Vec3::new(10.0, 0.0, 0.0),
        ));
        let before = Union::new(before_left, before_right);
        let after = Union::new(after_left, after_right);
        let policy = ConditioningPolicy {
            grid_spacing: 1.5,
            interface_band: 2.0,
            max_gradient_abs_error_p95: 0.25,
            max_interface_error: 1.0e-6,
            max_region_sample_count: ConditioningPolicy::DEFAULT_MAX_REGION_SAMPLE_COUNT,
            local_validation_mode: LocalValidationMode::LocalCacheQuality,
            ..ConditioningPolicy::default()
        };
        let mut model = ConditionedGeometryModel::new(
            SdfBounds::new(Vec3::splat(-24.0), Vec3::splat(24.0)),
            8.0,
            policy,
        );
        model.initialize_from_canonical(&before);
        let total_blocks = model.block_count();
        let left_edit = GeometryEdit::parameter_changed(
            GeometryEditKind::Feature,
            DirtyRegionSource::FeatureEdit,
            "left_radius",
            SdfBounds::new(Vec3::new(-16.0, -6.0, -6.0), Vec3::new(-4.0, 6.0, 6.0)),
            4.0,
            5.0,
            1.5,
        );
        let right_edit = GeometryEdit::parameter_changed(
            GeometryEditKind::Feature,
            DirtyRegionSource::FeatureEdit,
            "right_radius",
            SdfBounds::new(Vec3::new(4.0, -7.0, -7.0), Vec3::new(16.0, 7.0, 7.0)),
            4.0,
            5.5,
            1.5,
        );

        let summary = model.apply_edits_and_recondition(
            &before,
            &after,
            GeometryEditTransaction::new(vec![left_edit, right_edit]),
        );

        assert_eq!(
            summary.update_mode,
            ConditioningUpdateMode::LocalIncremental,
            "{:?}",
            summary.diagnostics
        );
        assert!(!summary.diagnostics.validated_against_full);
        assert!(!summary.full_rebuild_used);
        assert_eq!(summary.dirty_regions.len(), 2);
        assert_eq!(model.dirty_history().len(), 2);
        assert!(summary.regenerated_block_count > 0);
        assert!(summary.regenerated_block_count < total_blocks);
        assert_eq!(
            summary.invalidated_block_count,
            summary.regenerated_block_count
        );
        assert_ne!(summary.cache_state, ConditionedCacheState::Rejected);
    }

    #[test]
    fn undercovered_live_edit_falls_back_to_full_rebuild() {
        let before = Sphere::new(10.0);
        let after = Sphere::new(12.0);
        let policy = ConditioningPolicy {
            grid_spacing: 1.0,
            interface_band: 2.0,
            max_gradient_abs_error_p95: 0.2,
            max_interface_error: 1.0e-6,
            max_region_sample_count: ConditioningPolicy::DEFAULT_MAX_REGION_SAMPLE_COUNT,
            local_validation_mode: LocalValidationMode::FullComparison,
            ..ConditioningPolicy::default()
        };
        let mut model = ConditionedGeometryModel::new(
            SdfBounds::new(Vec3::splat(-16.0), Vec3::splat(16.0)),
            8.0,
            policy,
        );
        model.initialize_from_canonical(&before);
        let edit = GeometryEdit {
            kind: GeometryEditKind::Parameter,
            feature_id: Some("sphere_radius".to_string()),
            dirty_region: DirtyRegion::new(
                DirtyRegionSource::ParameterEdit,
                SdfBounds::new(Vec3::splat(-4.0), Vec3::splat(4.0)),
            )
            .with_halo(1.0),
            previous_value: Some(10.0),
            new_value: Some(12.0),
        };

        let summary = model.apply_edit_and_recondition(&before, &after, edit);

        assert_eq!(summary.update_mode, ConditioningUpdateMode::FullRebuild);
        assert!(summary.full_rebuild_used);
        assert_eq!(summary.cache_state, ConditionedCacheState::Ready);
        assert_eq!(summary.regenerated_block_count, model.block_count());
        assert_eq!(
            summary.diagnostics.cache_state,
            ConditionedCacheState::Rejected
        );
        assert!(summary.diagnostics.validated_against_full);
        assert_eq!(model.generation(), 2);
    }
}
