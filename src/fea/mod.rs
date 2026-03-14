// FEA module: voxel meshing → CalculiX → result fields.

pub mod setup;
pub mod meshing;
pub mod inp;
pub mod frd;
pub mod calculix;
pub mod pipeline;
pub mod viz;

pub use setup::{FEASetup, FEAConfig, FEARegion, FEAAxisRegion, FEAForceRegion,
                FEAPressureRegion, FEATorqueRegion, FEAMotorRegion,
                MaterialPreset, MaterialProperties};
pub use meshing::TetMesh;
pub use pipeline::{FEAPipeline, FEAGridResult, FEAMessage, GridField};
pub use viz::{FEAVizData, RegionViz, compute_fea_viz};
