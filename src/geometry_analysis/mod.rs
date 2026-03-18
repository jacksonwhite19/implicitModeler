// Geometry analysis modules that have no dependency on `render` or `project`,
// making them usable from both the library (`lib.rs`) and the binary (`main.rs`).

pub mod cg_sensitivity;
pub mod interference;

pub use cg_sensitivity::{
    CgSensitivityResult, compute_cg_sensitivity,
};
pub use interference::{
    InterferenceResult, InterferenceSeverity,
};
