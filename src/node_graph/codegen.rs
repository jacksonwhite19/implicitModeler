// Graph → Rhai script code generation

use std::collections::HashSet;
use super::types::{NodeGraph, NodeId, NodeKind};

#[derive(Debug, PartialEq)]
pub enum CodegenError {
    NoOutput,
    Cycle,
    DisconnectedInput { node_kind: &'static str, pin: &'static str },
}

impl std::fmt::Display for CodegenError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoOutput => write!(f, "No Output node in graph"),
            Self::Cycle => write!(f, "Cycle detected in graph"),
            Self::DisconnectedInput { node_kind, pin }
                => write!(f, "Node '{}' has unconnected input '{}'", node_kind, pin),
        }
    }
}

pub fn graph_to_rhai(graph: &NodeGraph) -> Result<String, CodegenError> {
    let output_id = graph.output_node_id.ok_or(CodegenError::NoOutput)?;
    let output_node = graph.node(output_id).ok_or(CodegenError::NoOutput)?;

    // The result var is what feeds the Output node's input
    let source_id = output_node.sdf_inputs.first()
        .and_then(|p| p.connection)
        .ok_or(CodegenError::NoOutput)?;

    // Topological sort (DFS post-order, explicit stack)
    let mut order: Vec<NodeId> = Vec::new();
    let mut visited: HashSet<u64> = HashSet::new();
    let mut visiting: HashSet<u64> = HashSet::new();

    topo_sort(graph, source_id, &mut order, &mut visited, &mut visiting)?;

    // Emit one let binding per node in topo order
    let mut lines: Vec<String> = Vec::new();
    for &nid in &order {
        if let Some(node) = graph.node(nid) {
            if node.kind == NodeKind::Output {
                continue;
            }
            let var = var_name(nid);
            let fn_name = node.kind.rhai_fn().unwrap();

            // Build argument list: SDF inputs, then string params, then float params
            let mut args: Vec<String> = Vec::new();

            for (i, pin) in node.sdf_inputs.iter().enumerate() {
                let conn = pin.connection.ok_or(CodegenError::DisconnectedInput {
                    node_kind: node.kind.display_name(),
                    pin: node.kind.sdf_input_labels().get(i).copied().unwrap_or("input"),
                })?;
                args.push(var_name(conn));
            }

            for param in &node.string_params {
                args.push(format!("\"{}\"", param.value));
            }

            for param in &node.float_params {
                // Always emit with at least one decimal place so Rhai parses as float
                if param.value.fract() == 0.0 {
                    args.push(format!("{:.1}", param.value));
                } else {
                    args.push(format!("{}", param.value));
                }
            }

            lines.push(format!("let {} = {}({});", var, fn_name, args.join(", ")));
        }
    }

    let result_var = var_name(source_id);
    lines.push(result_var);
    Ok(lines.join("\n"))
}

fn var_name(id: NodeId) -> String {
    format!("n{}", id.0)
}

fn topo_sort(
    graph: &NodeGraph,
    id: NodeId,
    order: &mut Vec<NodeId>,
    visited: &mut HashSet<u64>,
    visiting: &mut HashSet<u64>,
) -> Result<(), CodegenError> {
    if visited.contains(&id.0) {
        return Ok(());
    }
    if !visiting.insert(id.0) {
        return Err(CodegenError::Cycle);
    }

    if let Some(node) = graph.node(id) {
        // Recurse into all connected SDF inputs
        let connections: Vec<NodeId> = node.sdf_inputs.iter()
            .filter_map(|p| p.connection)
            .collect();
        for dep in connections {
            topo_sort(graph, dep, order, visited, visiting)?;
        }
    }

    visiting.remove(&id.0);
    visited.insert(id.0);
    order.push(id);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::node_graph::types::{NodeGraph, NodeKind};

    #[test]
    fn test_sphere_only() {
        let mut g = NodeGraph::with_output();
        let s = g.add_node(NodeKind::Sphere, [100.0, 100.0]);
        let out_id = g.output_node_id.unwrap();
        g.connect(s, out_id, 0);

        let script = graph_to_rhai(&g).unwrap();
        assert!(script.contains("sphere(10.0)"), "script: {}", script);
        assert!(script.ends_with(&format!("n{}", s.0)));
    }

    #[test]
    fn test_translate_sphere() {
        let mut g = NodeGraph::with_output();
        let s = g.add_node(NodeKind::Sphere, [50.0, 50.0]);
        let t = g.add_node(NodeKind::Translate, [200.0, 50.0]);
        g.connect(s, t, 0);
        let out_id = g.output_node_id.unwrap();
        g.connect(t, out_id, 0);

        let script = graph_to_rhai(&g).unwrap();
        // sphere must come before translate in the script
        let sphere_pos = script.find("sphere").unwrap();
        let translate_pos = script.find("translate").unwrap();
        assert!(sphere_pos < translate_pos);
    }

    #[test]
    fn test_wing_node_codegen() {
        let mut g = NodeGraph::with_output();
        let w = g.add_node(NodeKind::Wing, [100.0, 100.0]);
        let out_id = g.output_node_id.unwrap();
        g.connect(w, out_id, 0);

        let script = graph_to_rhai(&g).unwrap();
        // String param "2412" should be quoted, floats after
        assert!(script.contains("wing_with_airfoil(\"2412\""), "script: {}", script);
        assert!(script.contains("12.0"), "should have root_chord: {}", script);
    }

    #[test]
    fn test_fuselage_node_codegen() {
        let mut g = NodeGraph::with_output();
        let f = g.add_node(NodeKind::Fuselage, [100.0, 100.0]);
        let out_id = g.output_node_id.unwrap();
        g.connect(f, out_id, 0);

        let script = graph_to_rhai(&g).unwrap();
        assert!(script.contains("fuselage_parametric(60.0, 8.0"), "script: {}", script);
    }

    #[test]
    fn test_no_output() {
        let g = NodeGraph::default();
        assert_eq!(graph_to_rhai(&g), Err(CodegenError::NoOutput));
    }

    #[test]
    fn test_no_connection_to_output() {
        let g = NodeGraph::with_output();
        // Output exists but nothing connected to it
        assert_eq!(graph_to_rhai(&g), Err(CodegenError::NoOutput));
    }
}
