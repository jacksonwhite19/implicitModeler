// Node graph data structures

use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, Serialize, Deserialize)]
pub struct NodeId(pub u64);

/// Every node kind maps to one Rhai function call (or the Output sentinel).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub enum NodeKind {
    // Primitives
    Sphere,
    Box_,
    Cylinder,
    Torus,
    Cone,
    // Booleans
    Union,
    Subtract,
    Intersect,
    SmoothUnion,
    // Transforms
    Translate,
    Rotate,
    Scale,
    Offset,
    Shell,
    // Aerospace
    Fuselage,
    Nacelle,
    NacaAirfoil,
    Wing,
    // Special
    Output,
}

impl NodeKind {
    pub fn display_name(self) -> &'static str {
        match self {
            Self::Sphere      => "Sphere",
            Self::Box_        => "Box",
            Self::Cylinder    => "Cylinder",
            Self::Torus       => "Torus",
            Self::Cone        => "Cone",
            Self::Union       => "Union",
            Self::Subtract    => "Subtract",
            Self::Intersect   => "Intersect",
            Self::SmoothUnion => "Smooth Union",
            Self::Translate   => "Translate",
            Self::Rotate      => "Rotate",
            Self::Scale       => "Scale",
            Self::Offset      => "Offset",
            Self::Shell       => "Shell",
            Self::Fuselage    => "Fuselage",
            Self::Nacelle     => "Nacelle",
            Self::NacaAirfoil => "NACA Airfoil",
            Self::Wing        => "Wing",
            Self::Output      => "Output",
        }
    }

    pub fn rhai_fn(self) -> Option<&'static str> {
        match self {
            Self::Sphere      => Some("sphere"),
            Self::Box_        => Some("box_"),
            Self::Cylinder    => Some("cylinder"),
            Self::Torus       => Some("torus"),
            Self::Cone        => Some("cone"),
            Self::Union       => Some("union"),
            Self::Subtract    => Some("subtract"),
            Self::Intersect   => Some("intersect"),
            Self::SmoothUnion => Some("smooth_union"),
            Self::Translate   => Some("translate"),
            Self::Rotate      => Some("rotate"),
            Self::Scale       => Some("scale"),
            Self::Offset      => Some("offset"),
            Self::Shell       => Some("shell"),
            Self::Fuselage    => Some("fuselage_parametric"),
            Self::Nacelle     => Some("nacelle"),
            Self::NacaAirfoil => Some("naca"),
            Self::Wing        => Some("wing_with_airfoil"),
            Self::Output      => None,
        }
    }

    pub fn sdf_input_count(self) -> usize {
        match self {
            Self::Sphere | Self::Box_ | Self::Cylinder | Self::Torus | Self::Cone => 0,
            Self::Union | Self::Subtract | Self::Intersect | Self::SmoothUnion => 2,
            Self::Translate | Self::Rotate | Self::Scale | Self::Offset | Self::Shell => 1,
            Self::Fuselage | Self::Nacelle | Self::NacaAirfoil | Self::Wing => 0,
            Self::Output => 1,
        }
    }

    pub fn string_params(self) -> &'static [(&'static str, &'static str)] {
        match self {
            Self::NacaAirfoil => &[("designation", "2412")],
            Self::Wing        => &[("airfoil", "2412")],
            _ => &[],
        }
    }

    pub fn float_params(self) -> &'static [(&'static str, f32)] {
        match self {
            Self::Sphere      => &[("radius", 10.0)],
            Self::Box_        => &[("width", 20.0), ("height", 15.0), ("depth", 10.0)],
            Self::Cylinder    => &[("radius", 5.0), ("height", 20.0)],
            Self::Torus       => &[("major_r", 10.0), ("minor_r", 3.0)],
            Self::Cone        => &[("radius", 8.0), ("height", 15.0)],
            Self::Union       => &[],
            Self::Subtract    => &[],
            Self::Intersect   => &[],
            Self::SmoothUnion => &[("smoothness", 2.0)],
            Self::Translate   => &[("x", 0.0), ("y", 0.0), ("z", 0.0)],
            Self::Rotate      => &[("rx", 0.0), ("ry", 0.0), ("rz", 0.0)],
            Self::Scale       => &[("sx", 1.0), ("sy", 1.0), ("sz", 1.0)],
            Self::Offset      => &[("amount", 1.0)],
            Self::Shell       => &[("thickness", 1.0)],
            Self::Fuselage    => &[("length", 60.0), ("diameter", 8.0), ("nose", 0.7), ("tail", 0.5)],
            Self::Nacelle     => &[("length", 12.0), ("diameter", 3.0), ("inlet", 0.8), ("exhaust", 0.7)],
            Self::NacaAirfoil => &[("chord", 12.0)],
            Self::Wing        => &[
                ("root_chord", 12.0), ("tip_chord", 6.0), ("span", 25.0),
                ("sweep", 20.0), ("dihedral", 5.0), ("twist", -2.0),
            ],
            Self::Output => &[],
        }
    }

    pub fn sdf_input_labels(self) -> &'static [&'static str] {
        match self {
            Self::Union | Self::Subtract | Self::Intersect | Self::SmoothUnion => &["A", "B"],
            Self::Translate | Self::Rotate | Self::Scale | Self::Offset | Self::Shell => &["shape"],
            Self::Output => &["result"],
            _ => &[],
        }
    }

    pub fn header_color(self) -> [u8; 3] {
        match self {
            Self::Sphere | Self::Box_ | Self::Cylinder | Self::Torus | Self::Cone
                => [70, 130, 180],
            Self::Union | Self::Subtract | Self::Intersect | Self::SmoothUnion
                => [160, 82, 45],
            Self::Translate | Self::Rotate | Self::Scale | Self::Offset | Self::Shell
                => [60, 140, 80],
            Self::Fuselage | Self::Nacelle | Self::NacaAirfoil | Self::Wing
                => [120, 60, 160],
            Self::Output
                => [200, 170, 30],
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SdfInput {
    pub label: String,
    pub connection: Option<NodeId>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StringParam {
    pub label: String,
    pub value: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FloatParam {
    pub label: String,
    pub value: f32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GraphNode {
    pub id: NodeId,
    pub kind: NodeKind,
    pub pos: [f32; 2],
    pub sdf_inputs: Vec<SdfInput>,
    pub string_params: Vec<StringParam>,
    pub float_params: Vec<FloatParam>,
}

impl GraphNode {
    pub fn new(id: NodeId, kind: NodeKind, pos: [f32; 2]) -> Self {
        let sdf_inputs = kind.sdf_input_labels().iter()
            .map(|&l| SdfInput { label: l.to_string(), connection: None })
            .collect();
        let string_params = kind.string_params().iter()
            .map(|&(l, d)| StringParam { label: l.to_string(), value: d.to_string() })
            .collect();
        let float_params = kind.float_params().iter()
            .map(|&(l, d)| FloatParam { label: l.to_string(), value: d })
            .collect();
        Self { id, kind, pos, sdf_inputs, string_params, float_params }
    }

    pub fn body_height(&self) -> f32 {
        let rows = self.sdf_inputs.len().max(1)
            + self.string_params.len()
            + self.float_params.len();
        22.0 + rows as f32 * 24.0 + 10.0
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct NodeGraph {
    pub nodes: Vec<GraphNode>,
    next_id: u64,
    pub output_node_id: Option<NodeId>,
}

impl NodeGraph {
    pub fn with_output() -> Self {
        let mut g = Self::default();
        let id = g.add_node(NodeKind::Output, [400.0, 300.0]);
        g.output_node_id = Some(id);
        g
    }

    pub fn add_node(&mut self, kind: NodeKind, pos: [f32; 2]) -> NodeId {
        let id = NodeId(self.next_id);
        self.next_id += 1;
        if kind == NodeKind::Output {
            self.output_node_id = Some(id);
        }
        self.nodes.push(GraphNode::new(id, kind, pos));
        id
    }

    pub fn remove_node(&mut self, id: NodeId) {
        for node in &mut self.nodes {
            for pin in &mut node.sdf_inputs {
                if pin.connection == Some(id) {
                    pin.connection = None;
                }
            }
        }
        self.nodes.retain(|n| n.id != id);
        if self.output_node_id == Some(id) {
            self.output_node_id = None;
        }
    }

    pub fn connect(&mut self, from: NodeId, to_node: NodeId, to_pin: usize) {
        if let Some(node) = self.nodes.iter_mut().find(|n| n.id == to_node) {
            if let Some(pin) = node.sdf_inputs.get_mut(to_pin) {
                pin.connection = Some(from);
            }
        }
    }

    pub fn disconnect(&mut self, to_node: NodeId, to_pin: usize) {
        if let Some(node) = self.nodes.iter_mut().find(|n| n.id == to_node) {
            if let Some(pin) = node.sdf_inputs.get_mut(to_pin) {
                pin.connection = None;
            }
        }
    }

    pub fn node(&self, id: NodeId) -> Option<&GraphNode> {
        self.nodes.iter().find(|n| n.id == id)
    }

    pub fn node_mut(&mut self, id: NodeId) -> Option<&mut GraphNode> {
        self.nodes.iter_mut().find(|n| n.id == id)
    }
}
