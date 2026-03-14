// Aerospace primitives module

pub mod section;
pub mod airfoil;
pub mod wing;
pub mod fuselage;
pub mod nacelle;
pub mod structural;
pub mod structural_drone;

// Re-export key types
pub use section::Section2D;
pub use airfoil::{Airfoil, ExtrudedAirfoil, get_naca_airfoil, is_valid_naca_4digit};
pub use wing::{wing_with_airfoil, wing_from_sections};
pub use fuselage::{fuselage_parametric, CrossSection, LoftedFuselage};
pub use nacelle::nacelle_simple;
pub use structural::{rib_slab, spar_cylinder};
pub use structural_drone::{
    estimate_radius_at,
    bulkhead_at_station, lightening_hole_pattern,
    rod_mount, motor_arm, motor_mount, generate_mounts_sdf,
    wing_lattice, fuselage_lattice, fuselage_lattice_graded,
};

// Convenience blend wrapper using existing SmoothUnion
use std::sync::Arc;
use crate::sdf::Sdf;
use crate::sdf::booleans::SmoothUnion;

/// Blend two SDFs with a specified radius
/// This is a convenience wrapper around SmoothUnion
pub fn blend(a: Arc<dyn Sdf>, b: Arc<dyn Sdf>, radius: f32) -> Arc<dyn Sdf> {
    Arc::new(SmoothUnion::new(a, b, radius))
}
