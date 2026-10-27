use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::io::{Cursor, Read, Write};
use std::path::Path;

use crate::mesh::adaptive_sampling::{AdaptiveOctree, AdaptiveOctreeNode};

pub const LEAFSET_PAYLOAD_MAGIC: &[u8; 8] = b"AMTLFv1\0";
pub const LEAFSET_PAYLOAD_SCHEMA_VERSION: &str = "adaptive_marching_tets_leafset_payload.v1";

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct FeatureDepthRequest {
    pub requested_intervals: f64,
    pub verified_thickness: f64,
    pub root_size: f64,
    pub current_feature_level: u32,
    pub required_level: u32,
    pub achieved_cell_size: f64,
    pub achieved_intervals: f64,
    pub capped_by_max_depth: bool,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct FeatureResourceEstimate {
    pub baseline_feature_leaves: usize,
    pub baseline_total_leaves: usize,
    pub requested_level: u32,
    pub current_feature_level: u32,
    pub estimated_feature_leaves: usize,
    pub estimated_total_leaves: usize,
    pub estimated_surface_triangles: usize,
    pub estimated_tetrahedra: usize,
    pub estimated_memory_mb: f64,
    pub estimated_runtime_ms: f64,
    pub within_leaf_budget: bool,
    pub within_memory_budget: bool,
    pub within_runtime_budget: bool,
    pub status: String,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct ResourceBudget {
    pub max_leaves: usize,
    pub max_memory_mb: f64,
    pub max_runtime_ms: f64,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct IncrementalLevelPolicy {
    pub current_feature_level: u32,
    pub additional_levels: u32,
    pub target_feature_level: u32,
    pub current_intervals: f64,
    pub expected_intervals: f64,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct IncrementalResourceGate {
    pub measured_leaves: usize,
    pub measured_feature_leaves: usize,
    pub estimated_tetrahedra: usize,
    pub estimated_runtime_ms: f64,
    pub max_leaves: usize,
    pub max_tetrahedra: usize,
    pub max_runtime_ms: f64,
    pub status: String,
    pub failure_reasons: Vec<String>,
}

#[derive(Clone, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
pub struct CompactLeafKey {
    pub level: u32,
    pub morton_code: u64,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct PersistedLeafSetHeader {
    pub schema_version: String,
    pub geometry_hash: String,
    pub configuration_hash: String,
    pub leaf_set_hash: String,
    pub root_bounds_min: [f64; 3],
    pub root_bounds_max: [f64; 3],
    pub max_supported_depth: u32,
    pub leaf_count: usize,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct PersistedLeafSetMetadata {
    pub schema_version: String,
    pub magic: String,
    pub geometry_hash: String,
    pub sampler_configuration_hash: String,
    pub accepted_leaf_set_hash: String,
    pub calculated_leaf_set_hash: String,
    pub root_bounds_min: [f64; 3],
    pub root_bounds_max: [f64; 3],
    pub root_cell_dimensions: [f64; 3],
    pub maximum_supported_depth: u32,
    pub active_leaf_count: usize,
    pub level_histogram: BTreeMap<u32, usize>,
    pub refinement_reason_counts: BTreeMap<String, usize>,
    pub payload_bytes: usize,
    pub byte_roundtrip_identical: bool,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, Hash, Ord, PartialEq, PartialOrd, Serialize)]
pub struct MortonLeafKey {
    pub level: u32,
    pub coord: [u32; 3],
    pub morton_code: u64,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, Hash, Ord, PartialEq, PartialOrd, Serialize)]
pub enum FaceDirection {
    XMinus,
    XPlus,
    YMinus,
    YPlus,
    ZMinus,
    ZPlus,
}

impl FaceDirection {
    pub const ALL: [FaceDirection; 6] = [
        FaceDirection::XMinus,
        FaceDirection::XPlus,
        FaceDirection::YMinus,
        FaceDirection::YPlus,
        FaceDirection::ZMinus,
        FaceDirection::ZPlus,
    ];

    pub fn axis(self) -> usize {
        match self {
            FaceDirection::XMinus | FaceDirection::XPlus => 0,
            FaceDirection::YMinus | FaceDirection::YPlus => 1,
            FaceDirection::ZMinus | FaceDirection::ZPlus => 2,
        }
    }

    pub fn positive(self) -> bool {
        matches!(
            self,
            FaceDirection::XPlus | FaceDirection::YPlus | FaceDirection::ZPlus
        )
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub enum FaceNeighborRelation {
    DomainBoundary,
    SameLevel(MortonLeafKey),
    Coarser(MortonLeafKey),
    Finer(Vec<MortonLeafKey>),
    InvalidGap,
    InvalidOverlap(Vec<MortonLeafKey>),
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct IndexedBalanceStats {
    pub indexed_balance_used: bool,
    pub indexed_neighbor_audit_used: bool,
    pub all_pairs_balance_used: bool,
    pub all_pairs_neighbor_audit_used: bool,
    pub queue_pushes: usize,
    pub queue_pops: usize,
    pub stale_entries: usize,
    pub unique_active_leaves_examined: usize,
    pub direct_selected_parent_splits: usize,
    pub balance_only_splits: usize,
    pub max_queue_size: usize,
    pub final_max_face_neighbor_level_delta: u32,
    pub invalid_face_relations: usize,
}

pub fn persist_octree_leaf_payload(
    octree: &AdaptiveOctree,
    geometry_hash: &str,
    payload_path: &Path,
    metadata_path: &Path,
) -> Result<PersistedLeafSetMetadata, String> {
    let mut canonical = octree.clone();
    canonical.leaves.sort_by(|a, b| {
        a.level
            .cmp(&b.level)
            .then(a.morton_code.cmp(&b.morton_code))
            .then(a.grid_coord.cmp(&b.grid_coord))
    });
    for (idx, leaf) in canonical.leaves.iter_mut().enumerate() {
        leaf.id = idx as u64;
    }
    validate_octree_leaf_keys(&canonical.leaves)?;
    let payload = encode_leaf_payload(&canonical, geometry_hash)?;
    fs::write(payload_path, &payload).map_err(|e| e.to_string())?;
    let decoded = load_octree_leaf_payload(
        payload_path,
        geometry_hash,
        &octree.config_hash,
        &octree.octree_hash(),
    )?;
    let roundtrip = encode_leaf_payload(&decoded, geometry_hash)?;
    let metadata = leaf_payload_metadata(
        &canonical,
        geometry_hash,
        payload.len(),
        payload == roundtrip,
    );
    fs::write(
        metadata_path,
        serde_json::to_string_pretty(&metadata).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    Ok(metadata)
}

pub fn load_octree_leaf_payload(
    payload_path: &Path,
    expected_geometry_hash: &str,
    expected_config_hash: &str,
    expected_leaf_hash: &str,
) -> Result<AdaptiveOctree, String> {
    let bytes = fs::read(payload_path).map_err(|e| e.to_string())?;
    let (geometry_hash, octree) = decode_leaf_payload(&bytes)?;
    if geometry_hash != expected_geometry_hash {
        return Err("geometry_hash_mismatch".to_string());
    }
    if octree.config_hash != expected_config_hash {
        return Err("sampler_configuration_hash_mismatch".to_string());
    }
    validate_octree_leaf_keys(&octree.leaves)?;
    let calculated = octree.octree_hash();
    if calculated != expected_leaf_hash {
        return Err(format!(
            "leaf_set_hash_mismatch expected={expected_leaf_hash} calculated={calculated}"
        ));
    }
    Ok(octree)
}

pub fn validate_octree_leaf_keys(leaves: &[AdaptiveOctreeNode]) -> Result<(), String> {
    let keys = leaves
        .iter()
        .map(|leaf| node_key_from_parts(leaf.level, leaf.grid_coord))
        .collect::<Vec<_>>();
    validate_active_keys(&keys)
}

fn encode_leaf_payload(octree: &AdaptiveOctree, geometry_hash: &str) -> Result<Vec<u8>, String> {
    let mut out = Vec::new();
    out.extend_from_slice(LEAFSET_PAYLOAD_MAGIC);
    write_string(&mut out, LEAFSET_PAYLOAD_SCHEMA_VERSION)?;
    write_string(&mut out, geometry_hash)?;
    let body = bincode::serialize(octree).map_err(|e| e.to_string())?;
    write_u64(&mut out, body.len() as u64)?;
    out.extend_from_slice(&body);
    Ok(out)
}

fn decode_leaf_payload(bytes: &[u8]) -> Result<(String, AdaptiveOctree), String> {
    let mut cursor = Cursor::new(bytes);
    let mut magic = [0_u8; 8];
    cursor.read_exact(&mut magic).map_err(|e| e.to_string())?;
    if &magic != LEAFSET_PAYLOAD_MAGIC {
        return Err("leaf_payload_magic_mismatch".to_string());
    }
    let schema = read_string(&mut cursor)?;
    if schema != LEAFSET_PAYLOAD_SCHEMA_VERSION {
        return Err("leaf_payload_schema_mismatch".to_string());
    }
    let geometry_hash = read_string(&mut cursor)?;
    let len = read_u64(&mut cursor)? as usize;
    let mut body = vec![0_u8; len];
    cursor.read_exact(&mut body).map_err(|e| e.to_string())?;
    let octree = bincode::deserialize(&body).map_err(|e| e.to_string())?;
    Ok((geometry_hash, octree))
}

fn leaf_payload_metadata(
    octree: &AdaptiveOctree,
    geometry_hash: &str,
    payload_bytes: usize,
    byte_roundtrip_identical: bool,
) -> PersistedLeafSetMetadata {
    let mut reason_counts = BTreeMap::<String, usize>::new();
    for leaf in &octree.leaves {
        for reason in &leaf.refinement_reasons {
            *reason_counts.entry(format!("{reason:?}")).or_default() += 1;
        }
    }
    PersistedLeafSetMetadata {
        schema_version: LEAFSET_PAYLOAD_SCHEMA_VERSION.to_string(),
        magic: String::from_utf8_lossy(LEAFSET_PAYLOAD_MAGIC).to_string(),
        geometry_hash: geometry_hash.to_string(),
        sampler_configuration_hash: octree.config_hash.clone(),
        accepted_leaf_set_hash: octree.octree_hash(),
        calculated_leaf_set_hash: octree.octree_hash(),
        root_bounds_min: octree.bounds_min,
        root_bounds_max: octree.bounds_max,
        root_cell_dimensions: [octree.root_size, octree.root_size, octree.root_size],
        maximum_supported_depth: octree.summary.max_level_reached,
        active_leaf_count: octree.leaves.len(),
        level_histogram: octree.summary.level_histogram.clone(),
        refinement_reason_counts: reason_counts,
        payload_bytes,
        byte_roundtrip_identical,
    }
}

fn write_string(out: &mut Vec<u8>, value: &str) -> Result<(), String> {
    write_u64(out, value.len() as u64)?;
    out.write_all(value.as_bytes()).map_err(|e| e.to_string())
}

fn read_string(cursor: &mut Cursor<&[u8]>) -> Result<String, String> {
    let len = read_u64(cursor)? as usize;
    let mut bytes = vec![0_u8; len];
    cursor.read_exact(&mut bytes).map_err(|e| e.to_string())?;
    String::from_utf8(bytes).map_err(|e| e.to_string())
}

fn write_u64(out: &mut Vec<u8>, value: u64) -> Result<(), String> {
    out.write_all(&value.to_le_bytes())
        .map_err(|e| e.to_string())
}

fn read_u64(cursor: &mut Cursor<&[u8]>) -> Result<u64, String> {
    let mut bytes = [0_u8; 8];
    cursor.read_exact(&mut bytes).map_err(|e| e.to_string())?;
    Ok(u64::from_le_bytes(bytes))
}

fn node_key_from_parts(level: u32, coord: [u32; 3]) -> MortonLeafKey {
    MortonLeafKey::new(level, coord)
}

#[derive(Clone, Debug)]
pub struct ActiveLeafMap<T> {
    max_depth: u32,
    leaves: BTreeMap<MortonLeafKey, T>,
}

impl MortonLeafKey {
    pub fn new(level: u32, coord: [u32; 3]) -> Self {
        Self {
            level,
            coord,
            morton_code: encode_morton(coord),
        }
    }

    pub fn parent(self) -> Option<Self> {
        (self.level > 0).then(|| {
            Self::new(
                self.level - 1,
                [self.coord[0] >> 1, self.coord[1] >> 1, self.coord[2] >> 1],
            )
        })
    }

    pub fn children(self) -> [Self; 8] {
        std::array::from_fn(|child| {
            let dx = (child & 1) as u32;
            let dy = ((child >> 1) & 1) as u32;
            let dz = ((child >> 2) & 1) as u32;
            Self::new(
                self.level + 1,
                [
                    self.coord[0] * 2 + dx,
                    self.coord[1] * 2 + dy,
                    self.coord[2] * 2 + dz,
                ],
            )
        })
    }

    pub fn same_level_neighbor(self, direction: FaceDirection) -> Option<Self> {
        let axis = direction.axis();
        let extent = 1_u32.checked_shl(self.level).unwrap_or(0);
        let mut coord = self.coord;
        if direction.positive() {
            if coord[axis] + 1 >= extent {
                return None;
            }
            coord[axis] += 1;
        } else {
            if coord[axis] == 0 {
                return None;
            }
            coord[axis] -= 1;
        }
        Some(Self::new(self.level, coord))
    }

    pub fn bounds(self, root_min: [f64; 3], root_size: f64) -> ([f64; 3], [f64; 3]) {
        let denom = 2.0_f64.powi(self.level as i32);
        let cell = root_size / denom;
        let min = [
            root_min[0] + self.coord[0] as f64 * cell,
            root_min[1] + self.coord[1] as f64 * cell,
            root_min[2] + self.coord[2] as f64 * cell,
        ];
        let max = [min[0] + cell, min[1] + cell, min[2] + cell];
        (min, max)
    }
}

impl<T> ActiveLeafMap<T> {
    pub fn new(max_depth: u32) -> Self {
        Self {
            max_depth,
            leaves: BTreeMap::new(),
        }
    }

    pub fn from_items(max_depth: u32, items: impl IntoIterator<Item = (MortonLeafKey, T)>) -> Self {
        let mut out = Self::new(max_depth);
        for (key, value) in items {
            out.insert(key, value);
        }
        out
    }

    pub fn insert(&mut self, key: MortonLeafKey, value: T) -> Option<T> {
        self.leaves.insert(key, value)
    }

    pub fn remove(&mut self, key: &MortonLeafKey) -> Option<T> {
        self.leaves.remove(key)
    }

    pub fn contains_key(&self, key: &MortonLeafKey) -> bool {
        self.leaves.contains_key(key)
    }

    pub fn get(&self, key: &MortonLeafKey) -> Option<&T> {
        self.leaves.get(key)
    }

    pub fn get_mut(&mut self, key: &MortonLeafKey) -> Option<&mut T> {
        self.leaves.get_mut(key)
    }

    pub fn keys(&self) -> impl Iterator<Item = MortonLeafKey> + '_ {
        self.leaves.keys().copied()
    }

    pub fn values(self) -> impl Iterator<Item = T> {
        self.leaves.into_values()
    }

    pub fn len(&self) -> usize {
        self.leaves.len()
    }

    pub fn face_neighbor(
        &self,
        key: MortonLeafKey,
        direction: FaceDirection,
    ) -> FaceNeighborRelation {
        let Some(same) = key.same_level_neighbor(direction) else {
            return FaceNeighborRelation::DomainBoundary;
        };
        if self.contains_key(&same) {
            return FaceNeighborRelation::SameLevel(same);
        }
        let mut coarse = same;
        while let Some(parent) = coarse.parent() {
            if self.contains_key(&parent) {
                return FaceNeighborRelation::Coarser(parent);
            }
            coarse = parent;
        }
        let finer = self.finer_face_neighbors(key, direction);
        if finer.is_empty() {
            FaceNeighborRelation::InvalidGap
        } else if face_finer_coverage_complete(key, direction, &finer) {
            FaceNeighborRelation::Finer(finer)
        } else {
            FaceNeighborRelation::InvalidOverlap(finer)
        }
    }

    pub fn finer_face_neighbors(
        &self,
        key: MortonLeafKey,
        direction: FaceDirection,
    ) -> Vec<MortonLeafKey> {
        let axis = direction.axis();
        let mut out = BTreeSet::<MortonLeafKey>::new();
        for level in key.level + 1..=self.max_depth {
            let scale = 1_u32 << (level - key.level);
            let mut ranges = [
                key.coord[0] * scale..(key.coord[0] + 1) * scale,
                key.coord[1] * scale..(key.coord[1] + 1) * scale,
                key.coord[2] * scale..(key.coord[2] + 1) * scale,
            ];
            let normal = if direction.positive() {
                (key.coord[axis] + 1) * scale
            } else {
                key.coord[axis].saturating_mul(scale).saturating_sub(1)
            };
            let extent = 1_u32 << level;
            if normal >= extent {
                continue;
            }
            ranges[axis] = normal..normal + 1;
            for x in ranges[0].clone() {
                for y in ranges[1].clone() {
                    for z in ranges[2].clone() {
                        let candidate = MortonLeafKey::new(level, [x, y, z]);
                        if self.contains_key(&candidate) {
                            out.insert(candidate);
                        }
                    }
                }
            }
        }
        out.into_iter().collect()
    }
}

pub fn encode_morton(coord: [u32; 3]) -> u64 {
    let mut code = 0_u64;
    for bit in 0..21 {
        code |= ((coord[0] as u64 >> bit) & 1) << (3 * bit);
        code |= ((coord[1] as u64 >> bit) & 1) << (3 * bit + 1);
        code |= ((coord[2] as u64 >> bit) & 1) << (3 * bit + 2);
    }
    code
}

pub fn validate_active_keys(keys: &[MortonLeafKey]) -> Result<(), String> {
    let mut seen = BTreeSet::new();
    for key in keys {
        if !seen.insert(*key) {
            return Err("duplicate_leaf_key".to_string());
        }
    }
    let key_set = seen;
    for key in keys {
        let mut parent = key.parent();
        while let Some(p) = parent {
            if key_set.contains(&p) {
                return Err("ancestor_descendant_conflict".to_string());
            }
            parent = p.parent();
        }
    }
    Ok(())
}

pub fn indexed_max_neighbor_delta<T>(map: &ActiveLeafMap<T>) -> (u32, usize) {
    let mut max_delta = 0_u32;
    let mut invalid = 0_usize;
    for key in map.keys() {
        for direction in FaceDirection::ALL {
            match map.face_neighbor(key, direction) {
                FaceNeighborRelation::DomainBoundary => {}
                FaceNeighborRelation::SameLevel(other) => {
                    max_delta = max_delta.max(key.level.abs_diff(other.level));
                }
                FaceNeighborRelation::Coarser(other) => {
                    max_delta = max_delta.max(key.level.abs_diff(other.level));
                }
                FaceNeighborRelation::Finer(others) => {
                    for other in others {
                        max_delta = max_delta.max(key.level.abs_diff(other.level));
                    }
                }
                FaceNeighborRelation::InvalidGap | FaceNeighborRelation::InvalidOverlap(_) => {
                    invalid += 1;
                }
            }
        }
    }
    (max_delta, invalid)
}

fn face_finer_coverage_complete(
    key: MortonLeafKey,
    direction: FaceDirection,
    finer: &[MortonLeafKey],
) -> bool {
    let Some(min_level) = finer.iter().map(|k| k.level).min() else {
        return false;
    };
    let axis = direction.axis();
    let scale = 1_u32 << (min_level - key.level);
    let expected = scale.saturating_mul(scale);
    let mut covered = BTreeSet::new();
    for f in finer.iter().filter(|f| f.level == min_level) {
        let mut uv = Vec::with_capacity(2);
        for a in 0..3 {
            if a != axis {
                uv.push(f.coord[a] - key.coord[a] * scale);
            }
        }
        if uv.len() == 2 {
            covered.insert((uv[0], uv[1]));
        }
    }
    covered.len() == expected as usize
}

pub fn split_leaf_key(parent: &CompactLeafKey) -> Vec<CompactLeafKey> {
    (0_u64..8)
        .map(|child| CompactLeafKey {
            level: parent.level + 1,
            morton_code: (parent.morton_code << 3) | child,
        })
        .collect()
}

pub fn direct_split_net_leaf_increase(parent_count: usize) -> usize {
    parent_count.saturating_mul(7)
}

pub fn validate_persisted_header(
    header: &PersistedLeafSetHeader,
    expected_schema: &str,
    expected_geometry_hash: &str,
    expected_config_hash: &str,
    expected_leaf_hash: &str,
) -> Result<(), String> {
    if header.schema_version != expected_schema {
        return Err("schema_version_mismatch".to_string());
    }
    if header.geometry_hash != expected_geometry_hash {
        return Err("geometry_hash_mismatch".to_string());
    }
    if header.configuration_hash != expected_config_hash {
        return Err("configuration_hash_mismatch".to_string());
    }
    if header.leaf_set_hash != expected_leaf_hash {
        return Err("leaf_set_hash_mismatch".to_string());
    }
    Ok(())
}

impl Default for ResourceBudget {
    fn default() -> Self {
        Self {
            max_leaves: 650_000,
            max_memory_mb: 24_000.0,
            max_runtime_ms: 1_800_000.0,
        }
    }
}

pub fn incremental_level_policy(
    current_feature_level: u32,
    additional_levels: u32,
    current_intervals: f64,
) -> IncrementalLevelPolicy {
    let target_feature_level = current_feature_level.saturating_add(additional_levels);
    let expected_intervals = current_intervals * 2.0_f64.powi(additional_levels as i32);
    IncrementalLevelPolicy {
        current_feature_level,
        additional_levels,
        target_feature_level,
        current_intervals,
        expected_intervals,
    }
}

pub fn incremental_resource_gate(
    measured_leaves: usize,
    measured_feature_leaves: usize,
    baseline_leaves: usize,
    baseline_tetrahedra: usize,
    baseline_runtime_ms: f64,
    max_leaves: usize,
    max_tetrahedra: usize,
    max_runtime_ms: f64,
) -> IncrementalResourceGate {
    let leaf_ratio = measured_leaves as f64 / baseline_leaves.max(1) as f64;
    let estimated_tetrahedra = (baseline_tetrahedra as f64 * leaf_ratio).ceil() as usize;
    let estimated_runtime_ms = baseline_runtime_ms * leaf_ratio.powf(1.08);
    let mut failure_reasons = Vec::new();
    if measured_leaves > max_leaves {
        failure_reasons.push("measured_leaf_count_exceeds_hard_limit".to_string());
    }
    if estimated_tetrahedra > max_tetrahedra {
        failure_reasons.push("estimated_tetrahedra_exceeds_hard_limit".to_string());
    }
    if estimated_runtime_ms > max_runtime_ms {
        failure_reasons.push("estimated_runtime_exceeds_hard_limit".to_string());
    }
    let status = if failure_reasons.is_empty() {
        "run_allowed"
    } else {
        "resource_limited"
    }
    .to_string();
    IncrementalResourceGate {
        measured_leaves,
        measured_feature_leaves,
        estimated_tetrahedra,
        estimated_runtime_ms,
        max_leaves,
        max_tetrahedra,
        max_runtime_ms,
        status,
        failure_reasons,
    }
}

pub fn required_feature_depth(
    root_size: f64,
    verified_thickness: f64,
    requested_intervals: f64,
    current_feature_level: u32,
    max_depth: u32,
) -> FeatureDepthRequest {
    let target_size = verified_thickness / requested_intervals.max(1.0);
    let required = if target_size <= 0.0 || !target_size.is_finite() {
        current_feature_level
    } else {
        (root_size / target_size).log2().ceil().max(0.0) as u32
    };
    let required_level = required.min(max_depth);
    let achieved_cell_size = root_size / 2.0_f64.powi(required_level as i32);
    FeatureDepthRequest {
        requested_intervals,
        verified_thickness,
        root_size,
        current_feature_level,
        required_level,
        achieved_cell_size,
        achieved_intervals: verified_thickness / achieved_cell_size.max(1.0e-12),
        capped_by_max_depth: required > max_depth,
    }
}

pub fn estimate_feature_resources(
    baseline_feature_leaves: usize,
    baseline_total_leaves: usize,
    baseline_triangles: usize,
    baseline_tetrahedra: usize,
    baseline_runtime_ms: f64,
    current_feature_level: u32,
    requested_level: u32,
    budget: &ResourceBudget,
) -> FeatureResourceEstimate {
    let delta = requested_level.saturating_sub(current_feature_level);
    let growth = 8usize.saturating_pow(delta.min(8));
    let estimated_feature_leaves = baseline_feature_leaves.saturating_mul(growth);
    let estimated_total_leaves = baseline_total_leaves
        .saturating_sub(baseline_feature_leaves)
        .saturating_add(estimated_feature_leaves);
    let leaf_ratio = estimated_total_leaves as f64 / baseline_total_leaves.max(1) as f64;
    let estimated_surface_triangles = (baseline_triangles as f64 * leaf_ratio).ceil() as usize;
    let estimated_tetrahedra = (baseline_tetrahedra as f64 * leaf_ratio).ceil() as usize;
    let estimated_memory_mb =
        estimated_tetrahedra as f64 * 0.00045 + estimated_surface_triangles as f64 * 0.00012;
    let estimated_runtime_ms = baseline_runtime_ms * leaf_ratio.powf(1.08);
    let within_leaf_budget = estimated_total_leaves <= budget.max_leaves;
    let within_memory_budget = estimated_memory_mb <= budget.max_memory_mb;
    let within_runtime_budget = estimated_runtime_ms <= budget.max_runtime_ms;
    let status = if within_leaf_budget && within_memory_budget && within_runtime_budget {
        "run_allowed"
    } else {
        "resource_limited"
    }
    .to_string();
    FeatureResourceEstimate {
        baseline_feature_leaves,
        baseline_total_leaves,
        requested_level,
        current_feature_level,
        estimated_feature_leaves,
        estimated_total_leaves,
        estimated_surface_triangles,
        estimated_tetrahedra,
        estimated_memory_mb,
        estimated_runtime_ms,
        within_leaf_budget,
        within_memory_budget,
        within_runtime_budget,
        status,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::adaptive_sampling::RefinementReason;

    #[test]
    fn te4_depth_uses_cap_normal_target_size() {
        let d = required_feature_depth(274.0, 1.0, 4.0, 7, 12);
        assert_eq!(d.required_level, 11);
        assert!(d.achieved_intervals >= 4.0);
    }

    #[test]
    fn te6_quantizes_to_same_level_as_te4_for_current_root_size() {
        let d = required_feature_depth(274.0, 1.0, 6.0, 7, 12);
        assert_eq!(d.required_level, 11);
        assert!(d.achieved_intervals >= 6.0);
    }

    #[test]
    fn te8_requires_deeper_level() {
        let d = required_feature_depth(274.0, 1.0, 8.0, 7, 12);
        assert_eq!(d.required_level, 12);
        assert!(d.achieved_intervals >= 8.0);
    }

    #[test]
    fn depth_reports_maximum_depth_cap() {
        let d = required_feature_depth(274.0, 1.0, 8.0, 7, 10);
        assert_eq!(d.required_level, 10);
        assert!(d.capped_by_max_depth);
    }

    #[test]
    fn resource_estimate_fails_large_depth_jump() {
        let estimate = estimate_feature_resources(
            2872,
            29856,
            77476,
            3_081_312,
            857_503.0,
            7,
            11,
            &ResourceBudget::default(),
        );
        assert_eq!(estimate.status, "resource_limited");
        assert!(!estimate.within_leaf_budget);
    }

    #[test]
    fn plus_levels_double_intervals_per_level() {
        let p = incremental_level_policy(7, 3, 0.4671532846715328);
        assert_eq!(p.target_feature_level, 10);
        assert!((p.expected_intervals - 3.7372262773722624).abs() < 1.0e-12);
    }

    #[test]
    fn incremental_gate_reports_measured_leaf_limit() {
        let gate = incremental_resource_gate(
            700_001,
            50_000,
            29_856,
            3_081_312,
            857_503.0,
            700_000,
            20_000_000,
            1_800_000.0,
        );
        assert_eq!(gate.status, "resource_limited");
        assert!(
            gate.failure_reasons
                .iter()
                .any(|r| r == "measured_leaf_count_exceeds_hard_limit")
        );
    }

    #[test]
    fn incremental_gate_allows_small_growth() {
        let gate = incremental_resource_gate(
            40_000,
            5_000,
            29_856,
            3_081_312,
            857_503.0,
            700_000,
            20_000_000,
            1_800_000.0,
        );
        assert_eq!(gate.status, "run_allowed");
    }

    #[test]
    fn parent_split_creates_eight_deterministic_children() {
        let parent = CompactLeafKey {
            level: 7,
            morton_code: 0x1234,
        };
        let children = split_leaf_key(&parent);
        assert_eq!(children.len(), 8);
        assert_eq!(
            children[0],
            CompactLeafKey {
                level: 8,
                morton_code: 0x91a0
            }
        );
        assert_eq!(
            children[7],
            CompactLeafKey {
                level: 8,
                morton_code: 0x91a7
            }
        );
        assert_eq!(direct_split_net_leaf_increase(3), 21);
    }

    #[test]
    fn persisted_header_rejects_mismatches() {
        let header = PersistedLeafSetHeader {
            schema_version: "local_te_tip_leafset.v1".to_string(),
            geometry_hash: "g".to_string(),
            configuration_hash: "c".to_string(),
            leaf_set_hash: "61a187b7a9c6b6c1".to_string(),
            root_bounds_min: [0.0; 3],
            root_bounds_max: [1.0; 3],
            max_supported_depth: 10,
            leaf_count: 29_856,
        };
        assert!(
            validate_persisted_header(
                &header,
                "local_te_tip_leafset.v1",
                "g",
                "c",
                "61a187b7a9c6b6c1"
            )
            .is_ok()
        );
        assert_eq!(
            validate_persisted_header(&header, "bad", "g", "c", "61a187b7a9c6b6c1").unwrap_err(),
            "schema_version_mismatch"
        );
    }

    #[test]
    fn morton_leaf_parent_and_children_are_exact() {
        let key = MortonLeafKey::new(3, [2, 5, 6]);
        assert_eq!(key.parent().unwrap(), MortonLeafKey::new(2, [1, 2, 3]));
        let children = key.children();
        assert_eq!(children[0], MortonLeafKey::new(4, [4, 10, 12]));
        assert_eq!(children[7], MortonLeafKey::new(4, [5, 11, 13]));
        let mut sorted = children.to_vec();
        sorted.sort();
        sorted.dedup();
        assert_eq!(sorted.len(), 8);
    }

    #[test]
    fn same_level_neighbor_and_domain_boundary_are_indexed() {
        let mut map = ActiveLeafMap::new(2);
        let a = MortonLeafKey::new(1, [0, 0, 0]);
        let b = MortonLeafKey::new(1, [1, 0, 0]);
        map.insert(a, ());
        map.insert(b, ());
        assert_eq!(
            map.face_neighbor(a, FaceDirection::XPlus),
            FaceNeighborRelation::SameLevel(b)
        );
        assert_eq!(
            map.face_neighbor(a, FaceDirection::XMinus),
            FaceNeighborRelation::DomainBoundary
        );
    }

    #[test]
    fn coarser_neighbor_lookup_walks_parent_chain() {
        let mut map = ActiveLeafMap::new(3);
        let fine = MortonLeafKey::new(2, [1, 0, 0]);
        let coarse = MortonLeafKey::new(1, [1, 0, 0]);
        map.insert(fine, ());
        map.insert(coarse, ());
        assert_eq!(
            map.face_neighbor(fine, FaceDirection::XPlus),
            FaceNeighborRelation::Coarser(coarse)
        );
    }

    #[test]
    fn finer_face_enumeration_returns_four_children() {
        let mut map = ActiveLeafMap::new(2);
        let coarse = MortonLeafKey::new(1, [0, 0, 0]);
        map.insert(coarse, ());
        let expected = [
            MortonLeafKey::new(2, [2, 0, 0]),
            MortonLeafKey::new(2, [2, 0, 1]),
            MortonLeafKey::new(2, [2, 1, 0]),
            MortonLeafKey::new(2, [2, 1, 1]),
        ];
        for key in expected {
            map.insert(key, ());
        }
        match map.face_neighbor(coarse, FaceDirection::XPlus) {
            FaceNeighborRelation::Finer(mut got) => {
                got.sort();
                assert_eq!(got, expected);
            }
            other => panic!("unexpected relation {other:?}"),
        }
    }

    #[test]
    fn active_key_validation_rejects_duplicates_and_ancestors() {
        let a = MortonLeafKey::new(1, [0, 0, 0]);
        assert_eq!(
            validate_active_keys(&[a, a]).unwrap_err(),
            "duplicate_leaf_key"
        );
        let child = a.children()[0];
        assert_eq!(
            validate_active_keys(&[a, child]).unwrap_err(),
            "ancestor_descendant_conflict"
        );
    }

    #[test]
    fn leaf_payload_roundtrip_is_byte_identical() {
        let temp = tempfile::tempdir().expect("tempdir");
        let payload = temp.path().join("leafset.bin");
        let meta = temp.path().join("leafset.json");
        let octree = tiny_octree();
        let hash = octree.octree_hash();
        let metadata =
            persist_octree_leaf_payload(&octree, "test_geometry", &payload, &meta).unwrap();
        assert!(metadata.byte_roundtrip_identical);
        let loaded =
            load_octree_leaf_payload(&payload, "test_geometry", &octree.config_hash, &hash)
                .unwrap();
        assert_eq!(loaded.octree_hash(), hash);
        let before = std::fs::read(&payload).unwrap();
        persist_octree_leaf_payload(&loaded, "test_geometry", &payload, &meta).unwrap();
        let after = std::fs::read(&payload).unwrap();
        assert_eq!(before, after);
        assert_eq!(
            load_octree_leaf_payload(&payload, "bad_geometry", &octree.config_hash, &hash)
                .unwrap_err(),
            "geometry_hash_mismatch"
        );
    }

    fn tiny_octree() -> AdaptiveOctree {
        let mut leaves = Vec::new();
        for (id, coord) in [[0, 0, 0], [1, 0, 0]].into_iter().enumerate() {
            leaves.push(AdaptiveOctreeNode {
                id: id as u64,
                morton_code: MortonLeafKey::new(1, coord).morton_code,
                level: 1,
                grid_coord: coord,
                bounds_min: [coord[0] as f64, coord[1] as f64, coord[2] as f64],
                bounds_max: [
                    coord[0] as f64 + 1.0,
                    coord[1] as f64 + 1.0,
                    coord[2] as f64 + 1.0,
                ],
                refinement_reasons: vec![RefinementReason::MinimumDepth],
                samples: Vec::new(),
                feature_summary: Default::default(),
                local_estimated_thickness: None,
                local_estimated_curvature: None,
                normal_divergence_degrees: 0.0,
                field_interpolation_error: 0.0,
                normal_interpolation_error_degrees: 0.0,
                surface_crossing: false,
            });
        }
        let mut level_histogram = BTreeMap::new();
        level_histogram.insert(1, 2);
        AdaptiveOctree {
            schema_version: "test".to_string(),
            engine_version: "test".to_string(),
            config_hash: "config".to_string(),
            bounds_min: [0.0, 0.0, 0.0],
            bounds_max: [2.0, 2.0, 2.0],
            root_size: 2.0,
            leaves,
            summary: crate::mesh::adaptive_sampling::AdaptiveOctreeSummary {
                leaf_count: 2,
                max_level_reached: 1,
                level_histogram,
                ..Default::default()
            },
        }
    }
}
