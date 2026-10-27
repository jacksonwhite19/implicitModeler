// FEA module: voxel meshing → CalculiX → result fields.

pub mod calculix;
pub mod frd;
pub mod inp;
pub mod meshing;
pub mod pipeline;
pub mod setup;
pub mod viz;

pub use pipeline::{FEAGridResult, FEAMessage, FEAPipeline, GridField};
pub use setup::{
    FEAAxisRegion, FEAConfig, FEAForceRegion, FEAMotorRegion, FEAPressureRegion, FEARegion,
    FEASetup, FEATorqueRegion, MaterialPreset, MaterialProperties,
};
pub use viz::{FEAVizData, compute_fea_viz};
