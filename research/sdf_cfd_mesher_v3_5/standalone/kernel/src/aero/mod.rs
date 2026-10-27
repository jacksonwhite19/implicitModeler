// Aerodynamic analysis module: polar data, flight conditions, lifting line theory.
// This module is in the library crate so it can be referenced from scripting.

pub mod drag;
pub mod flight_condition;
pub mod inlet_analysis;
pub mod lifting_line;
pub mod performance;
pub mod polar_data;
pub mod polars;
pub mod propulsion;
pub mod propulsion_db;
pub mod stability;

pub use drag::{DragPolarResult, compute_drag_polar};
pub use flight_condition::FlightCondition;
pub use lifting_line::{LiftingLineResult, solve_lifting_line};
pub use performance::{compute_glide, compute_range_endurance, compute_rate_of_climb};
pub use polars::{AirfoilPolar, PolarDatabase};
pub use propulsion::{PropulsionSetup, compute_propulsion};
pub use propulsion_db::{MotorSpec, PropSpec, PropulsionDatabase};
pub use stability::{
    NeutralPointResult, StabilityCategory, StaticMarginResult, TrimResult, compute_neutral_point,
    compute_static_margin, compute_trim,
};
