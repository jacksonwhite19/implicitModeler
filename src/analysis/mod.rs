pub mod thickness;
pub mod measurements;
pub mod print_analysis;
pub mod hole_detection;
pub mod aero;
pub use measurements::{MeasurementResults, CrossSectionMeasurement, PointDistanceMeasurement,
                       DistanceMeasureKind, compute_model_properties, measure_cross_section, ray_march_grid, snap_to_surface};
// CG sensitivity and interference live in `geometry_analysis` (lib.rs-accessible).
pub use crate::geometry_analysis::{
    CgSensitivityResult, compute_cg_sensitivity,
    InterferenceResult, InterferenceSeverity,
};
