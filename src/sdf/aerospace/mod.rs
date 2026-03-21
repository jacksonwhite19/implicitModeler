// Aerospace primitives module

pub mod section;
pub mod airfoil;
pub mod wing;
pub mod fuselage;
pub mod nacelle;
pub mod structural;
pub mod structural_drone;
pub mod mechanical;
pub mod composite;
pub mod control_surfaces;
pub mod stability_geometry;
pub mod nose_tail;
pub mod inlets;

// Re-export key types
pub use section::Section2D;
pub use airfoil::{Airfoil, ExtrudedAirfoil, get_naca_airfoil, is_valid_naca_4digit};
pub use wing::{wing_with_airfoil, wing_from_sections};
pub use fuselage::{fuselage_parametric, fuselage_elliptical_parametric, CrossSection, LoftedFuselage};
pub use nacelle::nacelle_simple;
pub use structural::{rib_slab, spar_cylinder};
pub use structural_drone::{
    bulkhead_at_station, lightening_hole_pattern,
    rod_mount, motor_arm, motor_mount, generate_mounts_sdf,
    keepout_intersects_plane, bulkhead_with_keepouts, cable_hole_at,
};
pub use mechanical::{
    bolt_circle, bolt_square, bolt_rect,
    countersink, counterbore, slot,
    chamfer_edge, thread_hole,
    fc_mount, motor_mount_pattern,
};


pub use nose_tail::{HaackNose, HaackTail, TangentOgive, EllipsoidNose};
pub use inlets::{
    NacaInlet, InletLip, InletShape, EdfDuct, SDuct, BuriedInlet,
    VariableDuct, HollowVariableDuct, SplineTube, HollowSplineTube,
    ProfileDuct, HollowProfileDuct,
    build_conformal_profile_inlet,
};
