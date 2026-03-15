pub mod thickness;
pub mod measurements;
pub use measurements::{MeasurementResults, CrossSectionMeasurement, PointDistanceMeasurement,
                       DistanceMeasureKind, compute_model_properties, measure_cross_section,
                       measure_distance, ray_march_grid, snap_to_surface};
