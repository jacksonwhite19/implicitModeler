use crate::sdf::conditioning::SdfNodeMetadata;
use serde::{Deserialize, Serialize};

pub const DEFAULT_CSG_TIE_ABSOLUTE_TOLERANCE: f64 = 1.0e-7;
pub const DEFAULT_CSG_TIE_RELATIVE_TOLERANCE: f64 = 1.0e-6;
pub const DEFAULT_CSG_TIE_SCALE_FLOOR: f64 = 1.0;

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CsgTraceMode {
    None,
    Immediate,
    Recursive,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub struct CsgTiePolicy {
    pub absolute_tolerance: f64,
    pub relative_tolerance: f64,
    pub scale_floor: f64,
}

impl Default for CsgTiePolicy {
    fn default() -> Self {
        Self {
            absolute_tolerance: DEFAULT_CSG_TIE_ABSOLUTE_TOLERANCE,
            relative_tolerance: DEFAULT_CSG_TIE_RELATIVE_TOLERANCE,
            scale_floor: DEFAULT_CSG_TIE_SCALE_FLOOR,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub struct SdfProvenanceOptions {
    pub trace_mode: CsgTraceMode,
    pub tie_policy: CsgTiePolicy,
}

impl SdfProvenanceOptions {
    pub fn none() -> Self {
        Self {
            trace_mode: CsgTraceMode::None,
            tie_policy: CsgTiePolicy::default(),
        }
    }

    pub fn immediate() -> Self {
        Self {
            trace_mode: CsgTraceMode::Immediate,
            tie_policy: CsgTiePolicy::default(),
        }
    }

    pub fn recursive() -> Self {
        Self {
            trace_mode: CsgTraceMode::Recursive,
            tie_policy: CsgTiePolicy::default(),
        }
    }

    pub fn requested(self) -> bool {
        self.trace_mode != CsgTraceMode::None
    }
}

impl Default for SdfProvenanceOptions {
    fn default() -> Self {
        Self::none()
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CsgTraceSupport {
    Supported,
    Partial,
    Unsupported,
    Failed,
    NotRequested,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum HardCsgOperation {
    Union,
    Intersection,
    Subtraction,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CsgChildRole {
    Left,
    Right,
    Retained,
    Subtractor,
    Child,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CsgDecisionState {
    Unique,
    NearTie,
    ExactTie,
    Unsupported,
    Failed,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CsgSelectedSide {
    Left,
    Right,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct CsgEntityRef {
    pub stable_id: Option<String>,
    pub label: Option<String>,
    pub entity_kind: Option<String>,
    pub metadata_source: Option<String>,
    pub support_status: CsgTraceSupport,
}

impl CsgEntityRef {
    pub fn from_metadata(metadata: &SdfNodeMetadata) -> Self {
        let stable_id = if metadata.feature_ids.len() == 1 {
            metadata.feature_ids.first().cloned()
        } else {
            None
        };
        Self {
            stable_id,
            label: Some(metadata.node_kind.clone()),
            entity_kind: Some(metadata.node_kind.clone()),
            metadata_source: Some("SdfNodeMetadata".to_string()),
            support_status: if metadata.node_kind == "unknown" {
                CsgTraceSupport::Partial
            } else {
                CsgTraceSupport::Partial
            },
        }
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct ActiveCsgContributor {
    pub node_id: Option<String>,
    pub node_label: Option<String>,
    pub entity_kind: Option<String>,
    pub child_role: CsgChildRole,
    pub effective_sign: f64,
    pub raw_child_distance: f64,
    pub effective_child_distance: f64,
    pub metadata_source: Option<String>,
    pub support_status: CsgTraceSupport,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct HardCsgDecision {
    pub operation: HardCsgOperation,
    pub result_distance: f64,
    pub active: ActiveCsgContributor,
    pub competing: Option<ActiveCsgContributor>,
    pub decision_state: CsgDecisionState,
    pub absolute_margin: f64,
    pub relative_margin: f64,
    pub tie_threshold: f64,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct CsgProvenanceTrace {
    pub root: Option<CsgEntityRef>,
    pub decisions: Vec<HardCsgDecision>,
    pub terminal_entity: Option<CsgEntityRef>,
    pub complete: bool,
    pub support_status: CsgTraceSupport,
    pub termination_reason: Option<String>,
}

impl CsgProvenanceTrace {
    pub fn not_requested() -> Self {
        Self {
            root: None,
            decisions: Vec::new(),
            terminal_entity: None,
            complete: false,
            support_status: CsgTraceSupport::NotRequested,
            termination_reason: Some("hard CSG provenance was not requested".to_string()),
        }
    }

    pub fn terminal(metadata: SdfNodeMetadata) -> Self {
        let entity = CsgEntityRef::from_metadata(&metadata);
        Self {
            root: Some(entity.clone()),
            decisions: Vec::new(),
            terminal_entity: Some(entity),
            complete: true,
            support_status: CsgTraceSupport::Partial,
            termination_reason: None,
        }
    }

    pub fn unsupported(metadata: SdfNodeMetadata, reason: impl Into<String>) -> Self {
        let entity = CsgEntityRef::from_metadata(&metadata);
        Self {
            root: Some(entity.clone()),
            decisions: Vec::new(),
            terminal_entity: Some(entity),
            complete: false,
            support_status: CsgTraceSupport::Unsupported,
            termination_reason: Some(reason.into()),
        }
    }
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct SdfSample {
    pub distance: f32,
    pub provenance: Option<CsgProvenanceTrace>,
}

pub fn contributor_from_metadata(
    metadata: &SdfNodeMetadata,
    child_role: CsgChildRole,
    effective_sign: f64,
    raw_child_distance: f32,
    effective_child_distance: f32,
) -> ActiveCsgContributor {
    ActiveCsgContributor {
        node_id: if metadata.feature_ids.len() == 1 {
            metadata.feature_ids.first().cloned()
        } else {
            None
        },
        node_label: Some(metadata.node_kind.clone()),
        entity_kind: Some(metadata.node_kind.clone()),
        child_role,
        effective_sign,
        raw_child_distance: raw_child_distance as f64,
        effective_child_distance: effective_child_distance as f64,
        metadata_source: Some("SdfNodeMetadata".to_string()),
        support_status: CsgTraceSupport::Partial,
    }
}

pub fn decision_state(
    left_effective: f32,
    right_effective: f32,
    policy: CsgTiePolicy,
) -> (CsgDecisionState, f64, f64, f64) {
    if !left_effective.is_finite() || !right_effective.is_finite() {
        return (CsgDecisionState::Failed, f64::NAN, f64::NAN, f64::NAN);
    }
    let left = left_effective as f64;
    let right = right_effective as f64;
    let absolute_margin = (left - right).abs();
    let scale = left.abs().max(right.abs()).max(policy.scale_floor.max(0.0));
    let threshold = policy.absolute_tolerance.max(0.0) + policy.relative_tolerance.max(0.0) * scale;
    let relative_margin = if scale > 0.0 {
        absolute_margin / scale
    } else {
        0.0
    };
    let state = if left_effective == right_effective {
        CsgDecisionState::ExactTie
    } else if absolute_margin <= threshold {
        CsgDecisionState::NearTie
    } else {
        CsgDecisionState::Unique
    };
    (state, absolute_margin, relative_margin, threshold)
}

pub fn selected_side_from_result(
    left_effective: f32,
    right_effective: f32,
    result: f32,
) -> CsgSelectedSide {
    let result_bits = result.to_bits();
    let left_matches = left_effective.to_bits() == result_bits;
    let right_matches = right_effective.to_bits() == result_bits;
    match (left_matches, right_matches) {
        (true, false) => CsgSelectedSide::Left,
        (false, true) => CsgSelectedSide::Right,
        (true, true) => CsgSelectedSide::Left,
        (false, false) => {
            if left_effective == result {
                CsgSelectedSide::Left
            } else {
                CsgSelectedSide::Right
            }
        }
    }
}
