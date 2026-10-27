use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FoldClassification {
    TrueFoldover,
    TriangleInversion,
    LocalSelfOverlap,
    ValidSmoothFaceting,
    ValidConvexEdge,
    ValidConcaveEdge,
    AuthoritativeSharpFeature,
    CapTransition,
    HardCsgTransition,
    MarchingTetDiagonalArtifact,
    OrientationArtifact,
    NumericalUncertain,
    NotFolded,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct FoldSignal {
    pub adjacent_normal_dot: f64,
    pub triangle_a_sdf_angle_degrees: f64,
    pub triangle_b_sdf_angle_degrees: f64,
    pub sdf_normal_delta_degrees: f64,
    pub signed_dihedral_degrees: f64,
    pub local_self_overlap: bool,
    pub local_orientation_inversion: bool,
    pub authoritative_feature: bool,
    pub cap_transition: bool,
    pub hard_csg_transition: bool,
    pub marching_tet_diagonal_suspect: bool,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct FoldClassificationPolicy {
    pub old_fold_normal_dot_threshold: f64,
    pub severe_sdf_normal_angle_degrees: f64,
    pub valid_sdf_normal_angle_degrees: f64,
    pub high_sdf_normal_delta_degrees: f64,
}

impl Default for FoldClassificationPolicy {
    fn default() -> Self {
        Self {
            old_fold_normal_dot_threshold: -0.25,
            severe_sdf_normal_angle_degrees: 115.0,
            valid_sdf_normal_angle_degrees: 90.0,
            high_sdf_normal_delta_degrees: 75.0,
        }
    }
}

pub fn classify_fold_signal(
    signal: &FoldSignal,
    policy: &FoldClassificationPolicy,
) -> FoldClassification {
    if signal.local_self_overlap {
        return FoldClassification::LocalSelfOverlap;
    }
    if signal.local_orientation_inversion
        || signal.triangle_a_sdf_angle_degrees >= policy.severe_sdf_normal_angle_degrees
        || signal.triangle_b_sdf_angle_degrees >= policy.severe_sdf_normal_angle_degrees
    {
        return FoldClassification::TriangleInversion;
    }
    if signal.hard_csg_transition {
        return FoldClassification::HardCsgTransition;
    }
    if signal.cap_transition {
        return FoldClassification::CapTransition;
    }
    if signal.authoritative_feature {
        return FoldClassification::AuthoritativeSharpFeature;
    }
    if signal.adjacent_normal_dot > policy.old_fold_normal_dot_threshold {
        return FoldClassification::NotFolded;
    }
    let a_ok = signal.triangle_a_sdf_angle_degrees < policy.valid_sdf_normal_angle_degrees;
    let b_ok = signal.triangle_b_sdf_angle_degrees < policy.valid_sdf_normal_angle_degrees;
    if a_ok && b_ok {
        if signal.sdf_normal_delta_degrees < policy.high_sdf_normal_delta_degrees {
            return FoldClassification::ValidSmoothFaceting;
        }
        if signal.signed_dihedral_degrees < 0.0 {
            FoldClassification::ValidConcaveEdge
        } else {
            FoldClassification::ValidConvexEdge
        }
    } else if signal.marching_tet_diagonal_suspect {
        FoldClassification::MarchingTetDiagonalArtifact
    } else {
        FoldClassification::NumericalUncertain
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn base(dot: f64) -> FoldSignal {
        FoldSignal {
            adjacent_normal_dot: dot,
            triangle_a_sdf_angle_degrees: 5.0,
            triangle_b_sdf_angle_degrees: 6.0,
            sdf_normal_delta_degrees: 12.0,
            signed_dihedral_degrees: 0.0,
            local_self_overlap: false,
            local_orientation_inversion: false,
            authoritative_feature: false,
            cap_transition: false,
            hard_csg_transition: false,
            marching_tet_diagonal_suspect: false,
        }
    }

    #[test]
    fn flat_patch_is_not_folded() {
        assert_eq!(
            classify_fold_signal(&base(0.99), &FoldClassificationPolicy::default()),
            FoldClassification::NotFolded
        );
    }

    #[test]
    fn smooth_curved_patch_is_valid_faceting() {
        assert_eq!(
            classify_fold_signal(&base(-0.5), &FoldClassificationPolicy::default()),
            FoldClassification::ValidSmoothFaceting
        );
    }

    #[test]
    fn valid_concavity_is_not_true_foldover() {
        let mut signal = base(-0.7);
        signal.sdf_normal_delta_degrees = 100.0;
        signal.signed_dihedral_degrees = -55.0;
        assert_eq!(
            classify_fold_signal(&signal, &FoldClassificationPolicy::default()),
            FoldClassification::ValidConcaveEdge
        );
    }

    #[test]
    fn valid_cap_transition_is_classified_before_dihedral_failure() {
        let mut signal = base(-0.9);
        signal.cap_transition = true;
        assert_eq!(
            classify_fold_signal(&signal, &FoldClassificationPolicy::default()),
            FoldClassification::CapTransition
        );
    }

    #[test]
    fn hard_csg_concavity_is_not_rejected_as_fold() {
        let mut signal = base(-0.9);
        signal.hard_csg_transition = true;
        assert_eq!(
            classify_fold_signal(&signal, &FoldClassificationPolicy::default()),
            FoldClassification::HardCsgTransition
        );
    }

    #[test]
    fn inverted_triangle_is_hard_failure() {
        let mut signal = base(-0.1);
        signal.triangle_a_sdf_angle_degrees = 140.0;
        assert_eq!(
            classify_fold_signal(&signal, &FoldClassificationPolicy::default()),
            FoldClassification::TriangleInversion
        );
    }

    #[test]
    fn local_overlap_is_hard_failure() {
        let mut signal = base(0.4);
        signal.local_self_overlap = true;
        assert_eq!(
            classify_fold_signal(&signal, &FoldClassificationPolicy::default()),
            FoldClassification::LocalSelfOverlap
        );
    }
}
