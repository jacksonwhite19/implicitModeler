pub mod aero;
pub mod hole_detection;
pub mod measurements;
pub mod print_analysis;
pub mod thickness;
pub mod validation;
pub mod workflow_summary;
pub use measurements::{
    CrossSectionMeasurement, DistanceMeasureKind, MeasurementResults, PointDistanceMeasurement,
    compute_model_properties, measure_cross_section, ray_march_grid, snap_to_surface,
};
pub use validation::{GeometryValidationResult, ValidationSettings, validate_geometry};
// CG sensitivity and interference live in `geometry_analysis` (lib.rs-accessible).
pub use crate::geometry_analysis::{
    CgSensitivityResult, InterferenceResult, InterferenceSeverity, compute_cg_sensitivity,
};
