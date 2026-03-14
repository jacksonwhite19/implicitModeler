// Notebook data structures

use serde::{Deserialize, Serialize};

/// Every block kind maps to one Rhai call.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub enum NbBlockKind {
    // Primitives (0 SDF inputs)
    Sphere, Box_, Cylinder, Torus, Cone,
    // Booleans (2 SDF inputs)
    Union, Subtract, Intersect, SmoothUnion, SmoothSubtract,
    // Transforms (1 SDF input)
    Translate, Rotate, Scale, Offset, Shell,
    // Patterns (1 SDF input)
    LinearArray, PolarArray, MirrorX, MirrorY, MirrorZ,
    // Aerospace (0 SDF inputs)
    Fuselage, Nacelle, Wing, NacaAirfoil,
    // Structure (no SDF — just a visual divider with a label)
    Section,
}

impl NbBlockKind {
    pub fn display_name(self) -> &'static str {
        match self {
            Self::Sphere       => "Sphere",
            Self::Box_         => "Box",
            Self::Cylinder     => "Cylinder",
            Self::Torus        => "Torus",
            Self::Cone         => "Cone",
            Self::Union        => "Union",
            Self::Subtract     => "Subtract",
            Self::Intersect    => "Intersect",
            Self::SmoothUnion  => "Smooth Union",
            Self::SmoothSubtract => "Smooth Subtract",
            Self::Translate    => "Translate",
            Self::Rotate       => "Rotate",
            Self::Scale        => "Scale",
            Self::Offset       => "Offset",
            Self::Shell        => "Shell",
            Self::LinearArray  => "Linear Array",
            Self::PolarArray   => "Polar Array",
            Self::MirrorX      => "Mirror X",
            Self::MirrorY      => "Mirror Y",
            Self::MirrorZ      => "Mirror Z",
            Self::Fuselage     => "Fuselage",
            Self::Nacelle      => "Nacelle",
            Self::Wing         => "Wing",
            Self::NacaAirfoil  => "NACA Airfoil",
            Self::Section      => "── Section",
        }
    }

    /// How many SDF inputs this block takes (0, 1, or 2).
    pub fn sdf_inputs(self) -> usize {
        match self {
            Self::Sphere | Self::Box_ | Self::Cylinder | Self::Torus | Self::Cone => 0,
            Self::Fuselage | Self::Nacelle | Self::Wing | Self::NacaAirfoil => 0,
            Self::Section => 0,
            Self::Union | Self::Subtract | Self::Intersect
            | Self::SmoothUnion | Self::SmoothSubtract => 2,
            _ => 1,
        }
    }

    /// Float parameter definitions: (label, default_value).
    pub fn float_params(self) -> &'static [(&'static str, f32)] {
        match self {
            Self::Sphere      => &[("radius", 10.0)],
            Self::Box_        => &[("width", 20.0), ("height", 15.0), ("depth", 10.0)],
            Self::Cylinder    => &[("radius", 5.0), ("height", 20.0)],
            Self::Torus       => &[("major_r", 10.0), ("minor_r", 3.0)],
            Self::Cone        => &[("radius", 8.0), ("height", 15.0)],
            Self::Union | Self::Subtract | Self::Intersect => &[],
            Self::SmoothUnion | Self::SmoothSubtract => &[("k", 3.0)],
            Self::Translate   => &[("x", 0.0), ("y", 0.0), ("z", 0.0)],
            Self::Rotate      => &[("rx", 0.0), ("ry", 0.0), ("rz", 0.0)],
            Self::Scale       => &[("sx", 1.0), ("sy", 1.0), ("sz", 1.0)],
            Self::Offset      => &[("amount", 2.0)],
            Self::Shell       => &[("thickness", 1.5)],
            Self::LinearArray => &[("count", 3.0), ("dx", 10.0), ("dy", 0.0), ("dz", 0.0)],
            Self::PolarArray  => &[("count", 6.0)],
            Self::MirrorX | Self::MirrorY | Self::MirrorZ => &[],
            Self::Fuselage    => &[("length", 120.0), ("diameter", 40.0), ("nose", 0.7), ("tail", 0.5)],
            Self::Nacelle     => &[("length", 40.0), ("diameter", 15.0), ("inlet", 0.8), ("exhaust", 0.7)],
            Self::NacaAirfoil => &[("chord", 80.0)],
            Self::Wing        => &[
                ("root_chord", 80.0), ("tip_chord", 40.0), ("span", 200.0),
                ("sweep", 20.0), ("dihedral", 3.0), ("twist", -2.0),
            ],
            Self::Section => &[],
        }
    }

    /// String parameter definitions: (label, default_value).
    pub fn string_params(self) -> &'static [(&'static str, &'static str)] {
        match self {
            Self::NacaAirfoil => &[("designation", "2412")],
            Self::Wing        => &[("airfoil", "2412")],
            Self::Section     => &[("label", "Section")],
            _ => &[],
        }
    }

    /// Header bar color [R, G, B].
    pub fn header_color(self) -> [u8; 3] {
        match self {
            Self::Sphere | Self::Box_ | Self::Cylinder | Self::Torus | Self::Cone
                => [70, 130, 180],
            Self::Union | Self::Subtract | Self::Intersect | Self::SmoothUnion | Self::SmoothSubtract
                => [160, 82, 45],
            Self::Translate | Self::Rotate | Self::Scale | Self::Offset | Self::Shell
                => [60, 140, 80],
            Self::LinearArray | Self::PolarArray | Self::MirrorX | Self::MirrorY | Self::MirrorZ
                => [100, 100, 160],
            Self::Fuselage | Self::Nacelle | Self::Wing | Self::NacaAirfoil
                => [120, 60, 160],
            Self::Section => [55, 55, 55],
        }
    }

    pub fn category(self) -> &'static str {
        match self {
            Self::Sphere | Self::Box_ | Self::Cylinder | Self::Torus | Self::Cone => "Primitives",
            Self::Union | Self::Subtract | Self::Intersect | Self::SmoothUnion | Self::SmoothSubtract => "Booleans",
            Self::Translate | Self::Rotate | Self::Scale | Self::Offset | Self::Shell => "Transforms",
            Self::LinearArray | Self::PolarArray | Self::MirrorX | Self::MirrorY | Self::MirrorZ => "Patterns",
            Self::Fuselage | Self::Nacelle | Self::Wing | Self::NacaAirfoil => "Aerospace",
            Self::Section => "Structure",
        }
    }
}

/// Where a block sources one of its SDF inputs.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum NbInputRef {
    /// Output of the block immediately above.
    Previous,
    /// Output of a specific earlier block identified by its stable ID.
    BlockId(u64),
}

impl Default for NbInputRef {
    fn default() -> Self { NbInputRef::Previous }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NbFloatParam {
    pub label: String,
    pub value: f32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NbStringParam {
    pub label: String,
    pub value: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NbBlock {
    pub id: u64,
    pub kind: NbBlockKind,
    pub float_params: Vec<NbFloatParam>,
    pub string_params: Vec<NbStringParam>,
    /// Primary SDF input (most blocks).
    pub input_a: NbInputRef,
    /// Secondary SDF input (boolean/dual-input blocks only).
    pub input_b: NbInputRef,
}

impl NbBlock {
    pub fn new(id: u64, kind: NbBlockKind) -> Self {
        let float_params = kind.float_params().iter()
            .map(|&(label, value)| NbFloatParam { label: label.to_string(), value })
            .collect();
        let string_params = kind.string_params().iter()
            .map(|&(label, value)| NbStringParam { label: label.to_string(), value: value.to_string() })
            .collect();
        Self {
            id,
            kind,
            float_params,
            string_params,
            input_a: NbInputRef::Previous,
            input_b: NbInputRef::Previous,
        }
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Notebook {
    pub blocks: Vec<NbBlock>,
    next_id: u64,
    /// Which block is currently being previewed (None = last block).
    pub preview_id: Option<u64>,
}

impl Notebook {
    pub fn add_block(&mut self, kind: NbBlockKind) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        self.blocks.push(NbBlock::new(id, kind));
        id
    }

    pub fn remove_block(&mut self, id: u64) {
        self.blocks.retain(|b| b.id != id);
        if self.preview_id == Some(id) {
            self.preview_id = None;
        }
    }

    /// Index of the preview block (falls back to last block).
    pub fn preview_index(&self) -> usize {
        if let Some(pid) = self.preview_id {
            if let Some(i) = self.blocks.iter().position(|b| b.id == pid) {
                return i;
            }
        }
        self.blocks.len().saturating_sub(1)
    }

    /// Human-readable label for the block at index, shown in input dropdowns.
    pub fn block_label(&self, index: usize) -> String {
        if let Some(b) = self.blocks.get(index) {
            format!("{} #{}", b.kind.display_name(), index + 1)
        } else {
            format!("Block #{}", index + 1)
        }
    }
}
