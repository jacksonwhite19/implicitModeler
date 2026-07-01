// Aerospace primitives module

pub mod airfoil;
pub mod composite;
pub mod control_surfaces;
pub mod fuselage;
pub mod inlets;
pub mod mechanical;
pub mod nacelle;
pub mod nose_tail;
pub mod section;
pub mod stability_geometry;
pub mod structural;
pub mod structural_drone;
pub mod wing;

// Re-export key types
pub use airfoil::{
    Airfoil, AirfoilExportOptions, AirfoilFeatureMetadata, ExtrudedAirfoil, get_naca_airfoil,
    get_naca_airfoil_export_safe, is_valid_naca_4digit,
};
pub use fuselage::{
    CrossSection, LoftedFuselage, fuselage_elliptical_parametric, fuselage_parametric,
};
pub use mechanical::{
    bolt_circle, bolt_rect, bolt_square, chamfer_edge, counterbore, countersink, fc_mount,
    motor_mount_pattern, slot, thread_hole,
};
pub use nacelle::nacelle_simple;
pub use section::{Section2D, SectionBounds2D};
pub use structural::{rib_slab, spar_cylinder};
pub use structural_drone::{
    bulkhead_at_station, bulkhead_with_keepouts, cable_hole_at, generate_mounts_sdf,
    keepout_intersects_plane, lightening_hole_pattern, motor_arm, motor_mount, rod_mount,
};
pub use wing::{LoftedWing, wing_from_sections, wing_with_airfoil, wing_with_airfoil_export_safe};

pub use inlets::{
    BuriedInlet, EdfDuct, FixedProfileDuct, HollowFixedProfileDuct, HollowProfileDuct,
    HollowSplineTube, HollowVariableDuct, InletLip, InletShape, NacaInlet, ProfileDuct, SDuct,
    SplineTube, VariableDuct, build_conformal_profile_duct_at_x, build_conformal_profile_inlet,
    build_dual_conformal_profile_duct_at_x, build_mirrored_dual_conformal_profile_duct_at_x,
    conformal_profile_section, conformal_profile_section_at_x, conformal_rounded_rect_section,
};
pub use nose_tail::{EllipsoidNose, HaackNose, HaackTail, TangentOgive};
