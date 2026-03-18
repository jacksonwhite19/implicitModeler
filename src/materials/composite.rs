// Composite material definitions and preset library.
#![allow(dead_code)] // Material data model — fields will be used as materials UI is built out

/// Class of composite or structural material.
#[derive(Clone, Debug)]
pub enum CompositeMaterialType {
    /// Unidirectional carbon fiber prepreg.
    CarbonFiberUD    { ply_thickness_mm: f32 },
    /// Woven carbon fiber fabric (plain or twill).
    CarbonFiberWoven { ply_thickness_mm: f32 },
    /// Fiberglass fabric (plain weave).
    Fiberglass       { ply_thickness_mm: f32 },
    /// Structural foam core.
    Foam             { closed_cell: bool },
    /// FDM printed shell.
    Printed          { filament: PrintedFilament },
}

#[derive(Clone, Debug, PartialEq)]
pub enum PrintedFilament { PLA, PETG, ABS, Nylon, Custom }

/// A composite or structural material with mechanical properties.
#[derive(Clone, Debug)]
pub struct CompositeMaterial {
    pub name:                String,
    pub material_type:       CompositeMaterialType,
    /// Density g/cm³.
    pub density_g_cm3:       f32,
    /// Young's modulus in the fibre direction (MPa).
    pub elastic_modulus_mpa: f32,
    /// In-plane shear modulus (MPa).
    pub shear_modulus_mpa:   f32,
    /// Poisson ratio.
    pub poisson_ratio:       f32,
    /// sRGB colour used for viewport visualisation [0..1].
    pub color:               [f32; 3],
}

impl CompositeMaterial {
    /// Ply thickness in mm, if applicable; None for foam / printed materials.
    pub fn ply_thickness_mm(&self) -> Option<f32> {
        match &self.material_type {
            CompositeMaterialType::CarbonFiberUD    { ply_thickness_mm } => Some(*ply_thickness_mm),
            CompositeMaterialType::CarbonFiberWoven { ply_thickness_mm } => Some(*ply_thickness_mm),
            CompositeMaterialType::Fiberglass       { ply_thickness_mm } => Some(*ply_thickness_mm),
            _ => None,
        }
    }
}

// ── Built-in preset library ───────────────────────────────────────────────────

/// All built-in material presets indexed by name.
pub fn preset_library() -> Vec<CompositeMaterial> {
    vec![
        CompositeMaterial {
            name:                "CarbonUD_200gsm".into(),
            material_type:       CompositeMaterialType::CarbonFiberUD { ply_thickness_mm: 0.125 },
            density_g_cm3:       1.55,
            elastic_modulus_mpa: 135_000.0,
            shear_modulus_mpa:   5_000.0,
            poisson_ratio:       0.28,
            color:               [0.08, 0.08, 0.10],
        },
        CompositeMaterial {
            name:                "CarbonWoven_200gsm".into(),
            material_type:       CompositeMaterialType::CarbonFiberWoven { ply_thickness_mm: 0.20 },
            density_g_cm3:       1.55,
            elastic_modulus_mpa: 70_000.0,
            shear_modulus_mpa:   5_000.0,
            poisson_ratio:       0.10,
            color:               [0.12, 0.12, 0.16],
        },
        CompositeMaterial {
            name:                "Fiberglass_200gsm".into(),
            material_type:       CompositeMaterialType::Fiberglass { ply_thickness_mm: 0.22 },
            density_g_cm3:       1.80,
            elastic_modulus_mpa: 17_000.0,
            shear_modulus_mpa:   2_800.0,
            poisson_ratio:       0.30,
            color:               [0.85, 0.82, 0.65],
        },
        CompositeMaterial {
            name:                "Rohacell31".into(),
            material_type:       CompositeMaterialType::Foam { closed_cell: true },
            density_g_cm3:       0.031,
            elastic_modulus_mpa: 36.0,
            shear_modulus_mpa:   13.0,
            poisson_ratio:       0.35,
            color:               [0.92, 0.88, 0.78],
        },
        CompositeMaterial {
            name:                "Rohacell51".into(),
            material_type:       CompositeMaterialType::Foam { closed_cell: true },
            density_g_cm3:       0.052,
            elastic_modulus_mpa: 70.0,
            shear_modulus_mpa:   22.0,
            poisson_ratio:       0.35,
            color:               [0.90, 0.85, 0.72],
        },
        CompositeMaterial {
            name:                "DivinycellH60".into(),
            material_type:       CompositeMaterialType::Foam { closed_cell: true },
            density_g_cm3:       0.060,
            elastic_modulus_mpa: 55.0,
            shear_modulus_mpa:   22.0,
            poisson_ratio:       0.32,
            color:               [0.88, 0.84, 0.70],
        },
        CompositeMaterial {
            name:                "PLA".into(),
            material_type:       CompositeMaterialType::Printed { filament: PrintedFilament::PLA },
            density_g_cm3:       1.24,
            elastic_modulus_mpa: 3_500.0,
            shear_modulus_mpa:   1_300.0,
            poisson_ratio:       0.36,
            color:               [0.96, 0.96, 0.96],
        },
        CompositeMaterial {
            name:                "PETG".into(),
            material_type:       CompositeMaterialType::Printed { filament: PrintedFilament::PETG },
            density_g_cm3:       1.27,
            elastic_modulus_mpa: 2_800.0,
            shear_modulus_mpa:   1_000.0,
            poisson_ratio:       0.38,
            color:               [0.75, 0.90, 0.98],
        },
    ]
}

/// Look up a preset material by name (case-insensitive).
pub fn find_preset(name: &str) -> Option<CompositeMaterial> {
    let lower = name.to_ascii_lowercase();
    preset_library().into_iter().find(|m| m.name.to_ascii_lowercase() == lower)
}
