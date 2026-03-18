// Split-body, alignment features, fasteners, panels, and joints for printed parts.

pub mod split;
pub mod alignment;
pub mod tolerance;
pub mod fasteners;
pub mod panels;
pub mod joints;
pub mod bracket;

pub use split::{SplitPlane, SplitFitResult, split_body, split_body_multi, verify_split_fit};
pub use alignment::AlignmentFeature;
pub use tolerance::{ToleranceSettings, TolerancePreset, ToleranceCompensated};
