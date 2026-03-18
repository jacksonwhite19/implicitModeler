// Aerodynamic analysis module: polar data, flight conditions, lifting line theory.
// This module is in the library crate so it can be referenced from scripting.

pub mod polars;
pub mod inlet_analysis;
pub mod polar_data;
pub mod flight_condition;
pub mod lifting_line;
pub mod stability;
pub mod drag;
pub mod propulsion_db;
pub mod propulsion;
pub mod performance;

pub use polars::{AirfoilPolar, PolarDatabase};
pub use flight_condition::FlightCondition;
pub use lifting_line::{LiftingLineResult, solve_lifting_line};
pub use stability::{NeutralPointResult, StaticMarginResult, StabilityCategory, TrimResult,
                    compute_neutral_point, compute_static_margin, compute_trim};
pub use drag::{DragPolarResult, compute_drag_polar};
pub use propulsion_db::{MotorSpec, PropSpec, PropulsionDatabase};
pub use propulsion::{PropulsionSetup, compute_propulsion};
pub use performance::{compute_rate_of_climb, compute_range_endurance, compute_glide};
