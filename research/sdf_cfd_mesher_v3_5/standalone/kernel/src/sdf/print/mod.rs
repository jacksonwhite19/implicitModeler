// Split-body, alignment features, fasteners, panels, and joints for printed parts.

pub mod alignment;
pub mod bracket;
pub mod fasteners;
pub mod joints;
pub mod panels;
pub mod split;
pub mod tolerance;

pub use alignment::AlignmentFeature;
pub use split::{SplitFitResult, SplitPlane, split_body, split_body_multi, verify_split_fit};
pub use tolerance::{ToleranceCompensated, TolerancePreset, ToleranceSettings};
