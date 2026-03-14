// Node graph canvas UI using egui Painter

use eframe::egui;
use egui::{Color32, Pos2, Rect, Stroke, Vec2};
use super::types::{GraphNode, NodeGraph, NodeId, NodeKind};

const NODE_WIDTH: f32 = 160.0;
const PIN_RADIUS: f32 = 6.0;
const HEADER_HEIGHT: f32 = 24.0;
const ROW_HEIGHT: f32 = 22.0;
const CANVAS_GRID: f32 = 40.0;

/// Persistent UI state for the graph canvas (not serialized — view state only)
#[derive(Debug)]
pub struct GraphUiState {
    pub pan: Vec2,
    pub zoom: f32,
    pub selected_node: Option<NodeId>,
    pub dragging_node: Option<NodeId>,
    pub drag_offset: Vec2,
    pub wire_drag_from: Option<NodeId>,
    pub pending_context_pos: Option<Pos2>,
    /// Snapshot requested because a drag just ended
    drag_just_ended: bool,
}

impl Default for GraphUiState {
    fn default() -> Self {
        Self {
            pan: Vec2::ZERO,
            zoom: 1.0,
            selected_node: None,
            dragging_node: None,
            drag_offset: Vec2::ZERO,
            wire_drag_from: None,
            pending_context_pos: None,
            drag_just_ended: false,
        }
    }
}

fn canvas_to_screen(canvas_pos: [f32; 2], state: &GraphUiState, origin: Pos2) -> Pos2 {
    Pos2::new(
        origin.x + (canvas_pos[0] + state.pan.x) * state.zoom,
        origin.y + (canvas_pos[1] + state.pan.y) * state.zoom,
    )
}

fn screen_to_canvas(screen_pos: Pos2, state: &GraphUiState, origin: Pos2) -> [f32; 2] {
    [
        (screen_pos.x - origin.x) / state.zoom - state.pan.x,
        (screen_pos.y - origin.y) / state.zoom - state.pan.y,
    ]
}

fn node_screen_pos(node: &GraphNode, state: &GraphUiState, origin: Pos2) -> Pos2 {
    canvas_to_screen(node.pos, state, origin)
}

fn node_screen_rect(node: &GraphNode, state: &GraphUiState, origin: Pos2) -> Rect {
    let pos = node_screen_pos(node, state, origin);
    Rect::from_min_size(pos, Vec2::new(NODE_WIDTH * state.zoom, node.body_height() * state.zoom))
}

fn output_pin_pos(node: &GraphNode, state: &GraphUiState, origin: Pos2) -> Pos2 {
    let r = node_screen_rect(node, state, origin);
    Pos2::new(r.right(), r.top() + HEADER_HEIGHT * state.zoom * 0.5)
}

fn input_pin_pos(node: &GraphNode, pin_index: usize, state: &GraphUiState, origin: Pos2) -> Pos2 {
    let r = node_screen_rect(node, state, origin);
    let y = r.top() + (HEADER_HEIGHT + ROW_HEIGHT * 0.5 + ROW_HEIGHT * pin_index as f32) * state.zoom;
    Pos2::new(r.left(), y)
}

/// Returns `(should_run, push_history)`.
/// `push_history` is true whenever a structural change (add/delete/connect/drag-end)
/// occurred that should be snapshotted for undo.
pub fn show_graph_panel(
    ui: &mut egui::Ui,
    graph: &mut NodeGraph,
    state: &mut GraphUiState,
    on_run: &mut bool,
) -> bool {   // returns push_history
    let mut push_history = false;

    // Toolbar
    ui.horizontal(|ui| {
        if ui.button("▶ Run").clicked() {
            *on_run = true;
        }
        ui.separator();
        ui.label(format!("{} nodes", graph.nodes.len()));
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.small("Right-click canvas to add nodes • Del to delete selected");
        });
    });
    ui.separator();

    // Canvas
    let available = ui.available_rect_before_wrap();
    let (canvas_resp, painter) = ui.allocate_painter(available.size(), egui::Sense::click_and_drag());
    let origin = canvas_resp.rect.min;

    draw_grid(&painter, canvas_resp.rect, state);

    // Pan (middle-drag or Alt+left-drag)
    if canvas_resp.dragged_by(egui::PointerButton::Middle) {
        state.pan += canvas_resp.drag_delta() / state.zoom;
    }

    // Zoom
    let scroll_delta = ui.input(|i| i.raw_scroll_delta.y);
    if scroll_delta != 0.0 && canvas_resp.hovered() {
        let factor = if scroll_delta > 0.0 { 1.1_f32 } else { 1.0 / 1.1 };
        state.zoom = (state.zoom * factor).clamp(0.3, 3.0);
    }

    // Right-click on canvas background → add node menu
    if canvas_resp.secondary_clicked() {
        if let Some(mouse) = canvas_resp.interact_pointer_pos() {
            let cp = screen_to_canvas(mouse, state, origin);
            state.pending_context_pos = Some(Pos2::new(cp[0], cp[1]));
        }
    }

    // Wires
    draw_wires(&painter, graph, state, origin);

    // Wire being dragged
    if let Some(from_id) = state.wire_drag_from {
        if let Some(from_node) = graph.node(from_id) {
            let from_pos = output_pin_pos(from_node, state, origin);
            if let Some(mouse) = canvas_resp.interact_pointer_pos() {
                draw_bezier(&painter, from_pos, mouse,
                    Color32::from_rgba_premultiplied(255, 200, 50, 180), 2.0);
            }
        }
        if canvas_resp.drag_stopped() {
            state.wire_drag_from = None;
        }
    }

    // Delete selected node with Delete key
    if ui.input(|i| i.key_pressed(egui::Key::Delete)) {
        if let Some(sel) = state.selected_node {
            push_history = true;
            graph.remove_node(sel);
            state.selected_node = None;
        }
    }

    // --- Collect node interactions ---
    let mut node_drag_started: Option<(NodeId, Vec2)> = None;
    let mut wire_drag_started: Option<NodeId> = None;
    let mut wire_connected: Option<(NodeId, NodeId, usize)> = None;
    let mut node_to_delete: Option<NodeId> = None;

    let node_ids: Vec<NodeId> = graph.nodes.iter().map(|n| n.id).collect();
    for &nid in &node_ids {
        if let Some(node) = graph.node(nid) {
            let rect = node_screen_rect(node, state, origin);
            let node_resp = ui.interact(rect,
                egui::Id::new(("node", nid.0)), egui::Sense::click_and_drag());

            if node_resp.drag_started() && state.wire_drag_from.is_none() {
                if let Some(mouse) = node_resp.interact_pointer_pos() {
                    let offset = Vec2::new(
                        (mouse.x - rect.left()) / state.zoom,
                        (mouse.y - rect.top()) / state.zoom,
                    );
                    node_drag_started = Some((nid, offset));
                }
            }
            if node_resp.clicked() {
                state.selected_node = Some(nid);
            }

            node_resp.context_menu(|ui| {
                if ui.button("Delete node").clicked() {
                    node_to_delete = Some(nid);
                    ui.close_menu();
                }
            });

            draw_node(&painter, node, state, origin, state.selected_node == Some(nid));

            // Output pin → start wire drag
            if node.kind != NodeKind::Output {
                let out_pos = output_pin_pos(node, state, origin);
                let pin_rect = Rect::from_center_size(out_pos,
                    Vec2::splat(PIN_RADIUS * 2.5 * state.zoom));
                let pin_resp = ui.interact(pin_rect,
                    egui::Id::new(("out_pin", nid.0)), egui::Sense::drag());
                if pin_resp.drag_started() {
                    wire_drag_started = Some(nid);
                }
            }

            // Input pins → receive wire
            for (pin_idx, _pin) in node.sdf_inputs.iter().enumerate() {
                let in_pos = input_pin_pos(node, pin_idx, state, origin);
                let pin_rect = Rect::from_center_size(in_pos,
                    Vec2::splat(PIN_RADIUS * 2.5 * state.zoom));
                let pin_resp = ui.interact(pin_rect,
                    egui::Id::new(("in_pin", nid.0, pin_idx)), egui::Sense::click());

                if pin_resp.clicked() {
                    if let Some(from) = state.wire_drag_from {
                        wire_connected = Some((from, nid, pin_idx));
                        state.wire_drag_from = None;
                    }
                }
                if pin_resp.hovered() && canvas_resp.drag_stopped() {
                    if let Some(from) = state.wire_drag_from {
                        wire_connected = Some((from, nid, pin_idx));
                        state.wire_drag_from = None;
                    }
                }
            }
        }
    }

    // Apply node drag
    if let Some((drag_id, offset)) = node_drag_started {
        state.dragging_node = Some(drag_id);
        state.drag_offset = offset;
        state.drag_just_ended = false;
    }
    if let Some(drag_id) = state.dragging_node {
        if canvas_resp.dragged() || ui.input(|i| i.pointer.any_down()) {
            if let Some(mouse) = ui.input(|i| i.pointer.hover_pos()) {
                let canvas_pos = screen_to_canvas(
                    Pos2::new(mouse.x - state.drag_offset.x * state.zoom,
                              mouse.y - state.drag_offset.y * state.zoom),
                    state, origin,
                );
                if let Some(node) = graph.node_mut(drag_id) {
                    node.pos = canvas_pos;
                }
            }
        } else {
            // Drag just released — push a history snapshot
            state.dragging_node = None;
            push_history = true;
        }
    }

    // Apply wire drag start
    if let Some(from_id) = wire_drag_started {
        state.wire_drag_from = Some(from_id);
    }

    // Apply wire connection → push history
    if let Some((from, to, pin)) = wire_connected {
        graph.connect(from, to, pin);
        push_history = true;
    }

    // Delete node → push history
    if let Some(del_id) = node_to_delete {
        if state.selected_node == Some(del_id) {
            state.selected_node = None;
        }
        graph.remove_node(del_id);
        push_history = true;
    }

    // --- Param widgets (second pass to avoid borrow conflicts) ---
    let node_ids2: Vec<NodeId> = graph.nodes.iter().map(|n| n.id).collect();
    for &nid in &node_ids2 {
        let node_idx = match graph.nodes.iter().position(|n| n.id == nid) {
            Some(i) => i,
            None => continue,
        };

        let (base, sdf_input_count, str_count, float_count) = {
            let node = &graph.nodes[node_idx];
            (node_screen_pos(node, state, origin),
             node.sdf_inputs.len(),
             node.string_params.len(),
             node.float_params.len())
        };
        let input_start_y = base.y + HEADER_HEIGHT * state.zoom;

        // String params (TextEdit)
        for si in 0..str_count {
            let row_y = input_start_y
                + (sdf_input_count as f32 * ROW_HEIGHT + si as f32 * ROW_HEIGHT) * state.zoom;
            let edit_rect = Rect::from_min_size(
                Pos2::new(base.x + NODE_WIDTH * 0.38 * state.zoom, row_y + 3.0 * state.zoom),
                Vec2::new(NODE_WIDTH * 0.57 * state.zoom, ROW_HEIGHT * state.zoom - 6.0),
            );
            let value = &mut graph.nodes[node_idx].string_params[si].value;
            let resp = ui.put(edit_rect, egui::TextEdit::singleline(value)
                .font(egui::TextStyle::Monospace)
                .desired_width(f32::INFINITY));
            if resp.lost_focus() && resp.changed() {
                push_history = true;
            }
        }

        // Float params (DragValue) — push history when drag ends
        for pi in 0..float_count {
            let row_y = input_start_y
                + ((sdf_input_count + str_count) as f32 * ROW_HEIGHT
                   + pi as f32 * ROW_HEIGHT) * state.zoom;
            let drag_rect = Rect::from_min_size(
                Pos2::new(base.x + NODE_WIDTH * 0.45 * state.zoom, row_y + 3.0 * state.zoom),
                Vec2::new(NODE_WIDTH * 0.5 * state.zoom, ROW_HEIGHT * state.zoom - 6.0),
            );
            let value = &mut graph.nodes[node_idx].float_params[pi].value;
            let resp = ui.put(drag_rect,
                egui::DragValue::new(value).speed(0.1).max_decimals(2));
            // lost_focus fires when DragValue drag ends
            if resp.lost_focus() {
                push_history = true;
            }
        }
    }

    // "Add node" context menu
    if let Some(canvas_pos) = state.pending_context_pos {
        let pos = [canvas_pos.x, canvas_pos.y];
        let screen = canvas_to_screen([canvas_pos.x, canvas_pos.y], state, origin);
        let mut open = true;
        egui::Window::new("Add Node")
            .id(egui::Id::new("add_node_menu"))
            .fixed_pos(screen)
            .collapsible(false)
            .resizable(false)
            .title_bar(false)
            .open(&mut open)
            .show(ui.ctx(), |ui| {
                if draw_add_node_menu(ui, graph, pos) {
                    push_history = true;
                    state.pending_context_pos = None;
                }
            });
        if !open || ui.input(|i| i.key_pressed(egui::Key::Escape)) {
            state.pending_context_pos = None;
        }
    }

    push_history
}

fn draw_add_node_menu(ui: &mut egui::Ui, graph: &mut NodeGraph, pos: [f32; 2]) -> bool {
    let mut chosen = false;

    ui.label(egui::RichText::new("Add Node").strong());
    ui.separator();

    ui.label("Primitives");
    for kind in [NodeKind::Sphere, NodeKind::Box_, NodeKind::Cylinder,
                 NodeKind::Torus, NodeKind::Cone] {
        if ui.button(kind.display_name()).clicked() {
            graph.add_node(kind, pos);
            chosen = true;
        }
    }
    ui.separator();
    ui.label("Booleans");
    for kind in [NodeKind::Union, NodeKind::Subtract, NodeKind::Intersect,
                 NodeKind::SmoothUnion] {
        if ui.button(kind.display_name()).clicked() {
            graph.add_node(kind, pos);
            chosen = true;
        }
    }
    ui.separator();
    ui.label("Transforms");
    for kind in [NodeKind::Translate, NodeKind::Rotate, NodeKind::Scale,
                 NodeKind::Offset, NodeKind::Shell] {
        if ui.button(kind.display_name()).clicked() {
            graph.add_node(kind, pos);
            chosen = true;
        }
    }
    ui.separator();
    ui.label("Aerospace");
    for kind in [NodeKind::Fuselage, NodeKind::Nacelle,
                 NodeKind::NacaAirfoil, NodeKind::Wing] {
        if ui.button(kind.display_name()).clicked() {
            graph.add_node(kind, pos);
            chosen = true;
        }
    }

    chosen
}

fn draw_grid(painter: &egui::Painter, rect: Rect, state: &GraphUiState) {
    painter.rect_filled(rect, 0.0, Color32::from_gray(28));

    let grid = CANVAS_GRID * state.zoom;
    let origin_offset = Vec2::new(
        state.pan.x * state.zoom % grid,
        state.pan.y * state.zoom % grid,
    );

    let stroke = Stroke::new(1.0, Color32::from_rgba_premultiplied(60, 60, 60, 128));
    let stroke_major = Stroke::new(1.0, Color32::from_rgba_premultiplied(80, 80, 80, 180));

    let mut x = rect.left() + origin_offset.x;
    let mut col = 0i32;
    while x < rect.right() {
        let s = if col % 5 == 0 { stroke_major } else { stroke };
        painter.line_segment([Pos2::new(x, rect.top()), Pos2::new(x, rect.bottom())], s);
        x += grid;
        col += 1;
    }
    let mut y = rect.top() + origin_offset.y;
    let mut row = 0i32;
    while y < rect.bottom() {
        let s = if row % 5 == 0 { stroke_major } else { stroke };
        painter.line_segment([Pos2::new(rect.left(), y), Pos2::new(rect.right(), y)], s);
        y += grid;
        row += 1;
    }
}

fn draw_node(
    painter: &egui::Painter,
    node: &GraphNode,
    state: &GraphUiState,
    origin: Pos2,
    selected: bool,
) {
    let rect = node_screen_rect(node, state, origin);
    let [r, g, b] = node.kind.header_color();
    let header_color = Color32::from_rgb(r, g, b);
    let body_color = Color32::from_rgb(45, 45, 48);
    let border_color = if selected {
        Color32::from_rgb(255, 220, 80)
    } else {
        Color32::from_rgb(80, 80, 85)
    };
    let rounding = 4.0 * state.zoom;

    painter.rect_filled(rect, rounding, body_color);
    let header_rect = Rect::from_min_size(rect.min,
        Vec2::new(rect.width(), HEADER_HEIGHT * state.zoom));
    painter.rect_filled(header_rect, rounding, header_color);
    painter.rect_stroke(rect, rounding, Stroke::new(1.5, border_color));
    painter.text(
        header_rect.center(),
        egui::Align2::CENTER_CENTER,
        node.kind.display_name(),
        egui::FontId::proportional(11.0 * state.zoom.min(1.3)),
        Color32::WHITE,
    );

    let row_start_y = rect.top() + HEADER_HEIGHT * state.zoom;

    // SDF input pins + labels
    for (i, pin) in node.sdf_inputs.iter().enumerate() {
        let pin_pos = input_pin_pos(node, i, state, origin);
        let row_y = row_start_y + (i as f32 * ROW_HEIGHT + ROW_HEIGHT * 0.5) * state.zoom;
        let connected = pin.connection.is_some();
        let pin_fill = if connected { Color32::from_rgb(100, 200, 100) }
                       else { Color32::from_gray(80) };
        painter.circle_filled(pin_pos, PIN_RADIUS * state.zoom, pin_fill);
        painter.circle_stroke(pin_pos, PIN_RADIUS * state.zoom,
            Stroke::new(1.0, Color32::from_gray(180)));
        painter.text(
            Pos2::new(rect.left() + (PIN_RADIUS + 4.0) * state.zoom, row_y),
            egui::Align2::LEFT_CENTER,
            &pin.label,
            egui::FontId::proportional(10.0 * state.zoom.min(1.2)),
            Color32::from_gray(190),
        );
    }

    // Output pin
    if node.kind != NodeKind::Output {
        let out_pos = output_pin_pos(node, state, origin);
        painter.circle_filled(out_pos, PIN_RADIUS * state.zoom,
            Color32::from_rgb(180, 120, 60));
        painter.circle_stroke(out_pos, PIN_RADIUS * state.zoom,
            Stroke::new(1.0, Color32::from_gray(200)));
    }

    // String param labels
    let str_start_y = row_start_y + node.sdf_inputs.len() as f32 * ROW_HEIGHT * state.zoom;
    for (si, param) in node.string_params.iter().enumerate() {
        let y = str_start_y + (si as f32 * ROW_HEIGHT + ROW_HEIGHT * 0.5) * state.zoom;
        painter.text(
            Pos2::new(rect.left() + 8.0 * state.zoom, y),
            egui::Align2::LEFT_CENTER,
            &param.label,
            egui::FontId::proportional(10.0 * state.zoom.min(1.2)),
            Color32::from_gray(200),
        );
    }

    // Float param labels
    let param_start_y = str_start_y + node.string_params.len() as f32 * ROW_HEIGHT * state.zoom;
    for (pi, param) in node.float_params.iter().enumerate() {
        let y = param_start_y + (pi as f32 * ROW_HEIGHT + ROW_HEIGHT * 0.5) * state.zoom;
        painter.text(
            Pos2::new(rect.left() + 8.0 * state.zoom, y),
            egui::Align2::LEFT_CENTER,
            &param.label,
            egui::FontId::proportional(10.0 * state.zoom.min(1.2)),
            Color32::from_gray(200),
        );
    }
}

fn draw_wires(painter: &egui::Painter, graph: &NodeGraph,
              state: &GraphUiState, origin: Pos2) {
    for node in &graph.nodes {
        for (pin_idx, pin) in node.sdf_inputs.iter().enumerate() {
            if let Some(from_id) = pin.connection {
                if let Some(from_node) = graph.node(from_id) {
                    let from_pos = output_pin_pos(from_node, state, origin);
                    let to_pos = input_pin_pos(node, pin_idx, state, origin);
                    draw_bezier(painter, from_pos, to_pos,
                        Color32::from_rgb(100, 200, 100), 2.0);
                }
            }
        }
    }
}

fn draw_bezier(painter: &egui::Painter, from: Pos2, to: Pos2,
               color: Color32, width: f32) {
    let dx = (to.x - from.x).abs().max(80.0) * 0.5;
    let c1 = Pos2::new(from.x + dx, from.y);
    let c2 = Pos2::new(to.x - dx, to.y);
    const SEGMENTS: usize = 20;
    let mut pts: Vec<Pos2> = Vec::with_capacity(SEGMENTS + 1);
    for i in 0..=SEGMENTS {
        let t = i as f32 / SEGMENTS as f32;
        let t2 = t * t; let t3 = t2 * t;
        let mt = 1.0 - t; let mt2 = mt * mt; let mt3 = mt2 * mt;
        pts.push(Pos2::new(
            mt3*from.x + 3.0*mt2*t*c1.x + 3.0*mt*t2*c2.x + t3*to.x,
            mt3*from.y + 3.0*mt2*t*c1.y + 3.0*mt*t2*c2.y + t3*to.y,
        ));
    }
    for w in pts.windows(2) {
        painter.line_segment([w[0], w[1]], Stroke::new(width, color));
    }
}
