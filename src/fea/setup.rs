// FEA boundary condition and material data structures.

use std::sync::Arc;
use glam::Vec3;
use serde::{Serialize, Deserialize};
use crate::sdf::Sdf;

// ── Region types (runtime only — contain Arc<dyn Sdf>, not serialisable) ──────

#[derive(Clone)]
pub struct FEARegion {
    pub name: String,
    pub sdf:  Arc<dyn Sdf>,
}

#[derive(Clone)]
pub struct FEAAxisRegion {
    pub name:         String,
    pub sdf:          Arc<dyn Sdf>,
    pub constrain_x:  bool,
    pub constrain_y:  bool,
    pub constrain_z:  bool,
}

#[derive(Clone)]
pub struct FEAForceRegion {
    pub name:  String,
    pub sdf:   Arc<dyn Sdf>,
    pub force: Vec3,   // Newtons
}

#[derive(Clone)]
pub struct FEAPressureRegion {
    pub name:      String,
    pub sdf:       Arc<dyn Sdf>,
    pub magnitude: f32,  // Pa, positive = inward
}

#[derive(Clone)]
pub struct FEATorqueRegion {
    pub name:      String,
    pub sdf:       Arc<dyn Sdf>,
    pub axis:      Vec3,
    pub magnitude: f32,  // N·mm
}

#[derive(Clone)]
pub struct FEAMotorRegion {
    pub name:      String,
    pub sdf:       Arc<dyn Sdf>,
    pub thrust_n:  f32,
    pub torque_nmm: f32,
    pub direction: Vec3,
}

/// All FEA boundary conditions collected from one script execution.
#[derive(Default, Clone)]
pub struct FEASetup {
    pub fixed_supports:  Vec<FEARegion>,
    pub fixed_axes:      Vec<FEAAxisRegion>,
    pub force_loads:     Vec<FEAForceRegion>,
    pub pressure_loads:  Vec<FEAPressureRegion>,
    pub gravity:         Option<Vec3>,
    pub torque_loads:    Vec<FEATorqueRegion>,
    pub motor_thrusts:   Vec<FEAMotorRegion>,
}

impl FEASetup {
    pub fn is_empty(&self) -> bool {
        self.fixed_supports.is_empty()
            && self.fixed_axes.is_empty()
            && self.force_loads.is_empty()
            && self.pressure_loads.is_empty()
            && self.gravity.is_none()
            && self.torque_loads.is_empty()
            && self.motor_thrusts.is_empty()
    }
}

// ── Material presets ──────────────────────────────────────────────────────────

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum MaterialPreset { PLA, PETG, ABS, CarbonFiber, Custom }

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MaterialProperties {
    pub preset:                    MaterialPreset,
    pub youngs_modulus_mpa:        f32,
    pub poisson_ratio:             f32,
    pub density_tonne_per_mm3:     f32,
    pub yield_strength_mpa:        f32,
}

impl MaterialProperties {
    pub fn from_preset(preset: MaterialPreset) -> Self {
        match preset {
            MaterialPreset::PLA => Self {
                preset: MaterialPreset::PLA,
                youngs_modulus_mpa: 2100.0, poisson_ratio: 0.35,
                density_tonne_per_mm3: 1.24e-9, yield_strength_mpa: 50.0,
            },
            MaterialPreset::PETG => Self {
                preset: MaterialPreset::PETG,
                youngs_modulus_mpa: 2100.0, poisson_ratio: 0.38,
                density_tonne_per_mm3: 1.27e-9, yield_strength_mpa: 45.0,
            },
            MaterialPreset::ABS => Self {
                preset: MaterialPreset::ABS,
                youngs_modulus_mpa: 2300.0, poisson_ratio: 0.35,
                density_tonne_per_mm3: 1.05e-9, yield_strength_mpa: 40.0,
            },
            MaterialPreset::CarbonFiber => Self {
                preset: MaterialPreset::CarbonFiber,
                youngs_modulus_mpa: 70000.0, poisson_ratio: 0.10,
                density_tonne_per_mm3: 1.60e-9, yield_strength_mpa: 600.0,
            },
            MaterialPreset::Custom => Self {
                preset: MaterialPreset::Custom,
                youngs_modulus_mpa: 2100.0, poisson_ratio: 0.35,
                density_tonne_per_mm3: 1.24e-9, yield_strength_mpa: 50.0,
            },
        }
    }
}

impl Default for MaterialProperties {
    fn default() -> Self { Self::from_preset(MaterialPreset::PLA) }
}

/// FEA user configuration — persisted in the project file.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FEAConfig {
    pub material:         MaterialProperties,
    /// Grid resolution used for voxel tet meshing.
    pub mesh_resolution:  u32,
    /// Show FEA condition overlays in the viewport.
    pub show_conditions:  bool,
}

impl Default for FEAConfig {
    fn default() -> Self {
        Self { material: MaterialProperties::default(), mesh_resolution: 32, show_conditions: true }
    }
}
