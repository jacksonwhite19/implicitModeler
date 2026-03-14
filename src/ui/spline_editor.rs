// Spline cross-section editor — egui canvas + data model

use serde::{Deserialize, Serialize};
use eframe::egui;
use crate::sdf::profiles::SplineProfile;

// ── Data model ───────────────────────────────────────────────────────────────

#[derive(Clone, Serialize, Deserialize, PartialEq, Debug, Default)]
pub enum SymmetryMode {
    #[default]
    None,
    /// Mirror across the vertical Y axis — left ↔ right (flips X).
    SymY,
    /// Mirror across the horizontal X axis — top ↔ bottom (flips Y).
    SymX,
    /// N-fold rotational symmetry. N must divide the point count evenly.
    Radial(u32),
}

#[derive(Clone, Serialize, Deserialize, PartialEq, Debug)]
pub enum PresetKind { Circle, Ellipse, Stadium, Hexagon, Diamond }

// Re-export so callers don't need to import sdf::profiles separately.
pub use crate::sdf::profiles::PointRole;

/// Serialisable editor state for a single spline cross-section profile.
///
/// Convert to a runtime [`SplineProfile`] via [`SplineEditorState::to_profile`].
#[derive(Clone, Serialize, Deserialize)]
pub struct SplineEditorState {
    pub control_points: Vec<[f32; 2]>,
    pub symmetry: SymmetryMode,
    /// Indices of "corner" (C0) control points.
    pub sharp_points: Vec<usize>,
    /// Structural role of each control point (same length as control_points).
    pub point_roles: Vec<PointRole>,

    // ── Preset parameters ────────────────────────────────────────────────
    pub preset_kind: Option<PresetKind>,
    pub circle_r:     f32,
    pub ellipse_rx:   f32,
    pub ellipse_ry:   f32,
    pub stadium_len:  f32,  // half-length of straight section
    pub stadium_cap:  f32,  // cap radius
    pub hexagon_r:    f32,  // circumradius
    pub diamond_w:    f32,  // half-width  (X)
    pub diamond_h:    f32,  // half-height (Y)

    /// UI-only: index of the point currently being dragged.
    #[serde(skip)]
    pub dragging_point: Option<usize>,
    /// UI-only: index of the point currently selected.
    #[serde(skip)]
    pub selected_point: Option<usize>,
}

impl Default for SplineEditorState {
    fn default() -> Self {
        let sp = SplineProfile::circle(8, 1.0);
        let n = sp.control_points.len();
        Self {
            control_points: sp.control_points.iter().map(|v| [v.x, v.y]).collect(),
            point_roles:    vec![PointRole::Free; n],
            symmetry: SymmetryMode::None,
            sharp_points: vec![],
            preset_kind:  Some(PresetKind::Circle),
            circle_r:     1.0,
            ellipse_rx:   1.5,
            ellipse_ry:   0.8,
            stadium_len:  1.2,
            stadium_cap:  0.8,
            hexagon_r:    1.0,
            diamond_w:    1.0,
            diamond_h:    1.2,
            dragging_point: None,
            selected_point: None,
        }
    }
}

impl SplineEditorState {
    /// Build a runtime [`SplineProfile`] from this editor state.
    pub fn to_profile(&self) -> SplineProfile {
        use glam::Vec2;
        SplineProfile::with_roles(
            self.control_points.iter().map(|&[x, y]| Vec2::new(x, y)).collect(),
            self.sharp_points.clone(),
            self.point_roles.clone(),
        )
    }

    // ── Preset application ────────────────────────────────────────────────

    pub fn apply_circle(&mut self) {
        let sp = SplineProfile::circle(8, self.circle_r);
        self.control_points = sp.control_points.iter().map(|v| [v.x, v.y]).collect();
        self.point_roles = vec![PointRole::Free; self.control_points.len()];
        self.sharp_points.clear();
        self.selected_point = None;
        self.preset_kind = Some(PresetKind::Circle);
    }

    pub fn apply_ellipse(&mut self) {
        let sp = SplineProfile::ellipse(8, self.ellipse_rx, self.ellipse_ry);
        self.control_points = sp.control_points.iter().map(|v| [v.x, v.y]).collect();
        self.point_roles = vec![PointRole::Free; self.control_points.len()];
        self.sharp_points.clear();
        self.selected_point = None;
        self.preset_kind = Some(PresetKind::Ellipse);
    }

    pub fn apply_stadium(&mut self) {
        use std::f32::consts::PI;
        let cx = self.stadium_len;
        let r  = self.stadium_cap;
        let mut pts: Vec<[f32; 2]> = Vec::new();
        for i in 0..=5 {
            let a = PI * 0.5 + PI * i as f32 / 5.0;
            pts.push([-cx + a.cos() * r, a.sin() * r]);
        }
        for i in 0..=5 {
            let a = -PI * 0.5 + PI * i as f32 / 5.0;
            pts.push([cx + a.cos() * r, a.sin() * r]);
        }
        pts.dedup_by(|a, b| (a[0] - b[0]).abs() < 1e-5 && (a[1] - b[1]).abs() < 1e-5);
        self.control_points = pts;
        self.point_roles = vec![PointRole::Free; self.control_points.len()];
        self.sharp_points.clear();
        self.selected_point = None;
        self.preset_kind = Some(PresetKind::Stadium);
    }

    pub fn apply_hexagon(&mut self) {
        use std::f32::consts::{PI, TAU};
        let r = self.hexagon_r;
        self.control_points = (0..6).map(|i| {
            let a = TAU * i as f32 / 6.0 + PI / 6.0;
            [a.cos() * r, a.sin() * r]
        }).collect();
        self.point_roles = vec![PointRole::Free; 6];
        self.sharp_points = (0..6).collect();
        self.selected_point = None;
        self.preset_kind = Some(PresetKind::Hexagon);
    }

    pub fn apply_diamond(&mut self) {
        self.control_points = vec![
            [0.0,           self.diamond_h],
            [self.diamond_w, 0.0],
            [0.0,          -self.diamond_h],
            [-self.diamond_w, 0.0],
        ];
        self.point_roles = vec![PointRole::Free; 4];
        self.sharp_points = vec![0, 1, 2, 3];
        self.selected_point = None;
        self.preset_kind = Some(PresetKind::Diamond);
    }

    // ── Symmetry enforcement ─────────────────────────────────────────────

    /// After moving `moved_idx`, update mirror/rotated copies to match.
    pub fn apply_symmetry(&mut self, moved_idx: usize) {
        let n = self.control_points.len();
        if n < 2 { return; }
        match self.symmetry.clone() {
            SymmetryMode::None => {}
            SymmetryMode::SymY => {
                let [x, y] = self.control_points[moved_idx];
                let target = [-x, y];
                if let Some(idx) = self.nearest_other(moved_idx, target) {
                    self.control_points[idx] = target;
                }
            }
            SymmetryMode::SymX => {
                let [x, y] = self.control_points[moved_idx];
                let target = [x, -y];
                if let Some(idx) = self.nearest_other(moved_idx, target) {
                    self.control_points[idx] = target;
                }
            }
            SymmetryMode::Radial(k) => {
                let k = k as usize;
                if k < 2 || n % k != 0 { return; }
                let sector_size = n / k;
                let offset = moved_idx % sector_size;
                let [x, y] = self.control_points[moved_idx];
                for s in 1..k {
                    let a = std::f32::consts::TAU * s as f32 / k as f32;
                    let (sin_a, cos_a) = a.sin_cos();
                    let rx = x * cos_a - y * sin_a;
                    let ry = x * sin_a + y * cos_a;
                    let target = (s * sector_size + offset) % n;
                    self.control_points[target] = [rx, ry];
                }
            }
        }
    }

    fn nearest_other(&self, skip: usize, pos: [f32; 2]) -> Option<usize> {
        self.control_points.iter().enumerate()
            .filter(|(i, _)| *i != skip)
            .min_by(|(_, a), (_, b)| {
                let da = (a[0] - pos[0]).hypot(a[1] - pos[1]);
                let db = (b[0] - pos[0]).hypot(b[1] - pos[1]);
                da.partial_cmp(&db).unwrap()
            })
            .map(|(i, _)| i)
    }
}

// ── Canvas UI ────────────────────────────────────────────────────────────────

/// Draw the spline editor in the given `ui` region.
/// Returns `true` when any state change occurred (triggers a grid rebuild).
pub fn show_spline_editor(
    ui: &mut egui::Ui,
    state: &mut SplineEditorState,
    profile_name: &str,
) -> bool {
    let mut changed = false;

    // ── Preset buttons ────────────────────────────────────────────────────

    ui.horizontal_wrapped(|ui| {
        ui.label("Presets:");
        if ui.small_button("Circle").clicked()  { state.apply_circle();  changed = true; }
        if ui.small_button("Ellipse").clicked() { state.apply_ellipse(); changed = true; }
        if ui.small_button("Stadium").clicked() { state.apply_stadium(); changed = true; }
        if ui.small_button("Hexagon").clicked() { state.apply_hexagon(); changed = true; }
        if ui.small_button("Diamond").clicked() { state.apply_diamond(); changed = true; }
    });

    // ── Preset parameter controls ─────────────────────────────────────────

    match state.preset_kind.clone() {
        Some(PresetKind::Circle) => {
            ui.horizontal(|ui| {
                ui.label("r:");
                if ui.add(egui::DragValue::new(&mut state.circle_r)
                    .range(0.1f32..=50.0).speed(0.05).suffix(" u"))
                    .changed()
                { state.apply_circle(); changed = true; }
            });
        }
        Some(PresetKind::Ellipse) => {
            ui.horizontal(|ui| {
                ui.label("rx:");
                if ui.add(egui::DragValue::new(&mut state.ellipse_rx)
                    .range(0.1f32..=50.0).speed(0.05).suffix(" u"))
                    .changed()
                { state.apply_ellipse(); changed = true; }
                ui.label("ry:");
                if ui.add(egui::DragValue::new(&mut state.ellipse_ry)
                    .range(0.1f32..=50.0).speed(0.05).suffix(" u"))
                    .changed()
                { state.apply_ellipse(); changed = true; }
            });
        }
        Some(PresetKind::Stadium) => {
            ui.horizontal(|ui| {
                ui.label("len:");
                if ui.add(egui::DragValue::new(&mut state.stadium_len)
                    .range(0.1f32..=50.0).speed(0.05).suffix(" u"))
                    .changed()
                { state.apply_stadium(); changed = true; }
                ui.label("cap r:");
                if ui.add(egui::DragValue::new(&mut state.stadium_cap)
                    .range(0.1f32..=50.0).speed(0.05).suffix(" u"))
                    .changed()
                { state.apply_stadium(); changed = true; }
            });
        }
        Some(PresetKind::Hexagon) => {
            ui.horizontal(|ui| {
                ui.label("circumradius:");
                if ui.add(egui::DragValue::new(&mut state.hexagon_r)
                    .range(0.1f32..=50.0).speed(0.05).suffix(" u"))
                    .changed()
                { state.apply_hexagon(); changed = true; }
                // Also show edge length read-only: edge = r
                ui.label(egui::RichText::new(
                    format!("  edge ≈ {:.3} u", state.hexagon_r)
                ).weak().small());
            });
        }
        Some(PresetKind::Diamond) => {
            ui.horizontal(|ui| {
                ui.label("w:");
                if ui.add(egui::DragValue::new(&mut state.diamond_w)
                    .range(0.1f32..=50.0).speed(0.05).suffix(" u"))
                    .changed()
                { state.apply_diamond(); changed = true; }
                ui.label("h:");
                if ui.add(egui::DragValue::new(&mut state.diamond_h)
                    .range(0.1f32..=50.0).speed(0.05).suffix(" u"))
                    .changed()
                { state.apply_diamond(); changed = true; }
            });
        }
        None => {}
    }

    // ── Symmetry ──────────────────────────────────────────────────────────

    ui.horizontal(|ui| {
        ui.label("Sym:");
        let mut sym = state.symmetry.clone();
        if ui.selectable_label(sym == SymmetryMode::None, "None").clicked() {
            sym = SymmetryMode::None;
        }
        if ui.selectable_label(sym == SymmetryMode::SymY, "Y (left↔right)")
            .on_hover_text("Mirror across vertical Y axis")
            .clicked()
        { sym = SymmetryMode::SymY; }
        if ui.selectable_label(sym == SymmetryMode::SymX, "X (top↔bottom)")
            .on_hover_text("Mirror across horizontal X axis")
            .clicked()
        { sym = SymmetryMode::SymX; }
        if ui.selectable_label(matches!(sym, SymmetryMode::Radial(_)), "×4 Radial").clicked() {
            sym = SymmetryMode::Radial(4);
        }
        if sym != state.symmetry { state.symmetry = sym; }
    });

    ui.label(
        egui::RichText::new(format!("{}  |  {} pts", profile_name, state.control_points.len()))
            .weak().small(),
    );
    ui.separator();

    // ── Canvas ───────────────────────────────────────────────────────────

    let canvas_height = (ui.available_height() - 60.0).max(200.0);
    let canvas_size = egui::Vec2::new(ui.available_width(), canvas_height);
    let (response, painter) = ui.allocate_painter(canvas_size, egui::Sense::click_and_drag());
    let canvas_rect = response.rect;
    let center = canvas_rect.center();

    // Auto-scale: fit the profile's bounding box into ~70% of the canvas,
    // with a minimum scale so an empty/tiny profile is still usable.
    let scale = {
        let (mut max_abs_x, mut max_abs_y) = (0.1f32, 0.1f32);
        for &[x, y] in &state.control_points {
            if x.abs() > max_abs_x { max_abs_x = x.abs(); }
            if y.abs() > max_abs_y { max_abs_y = y.abs(); }
        }
        // Add 30% padding around the profile extent
        let extent_x = max_abs_x * 1.3;
        let extent_y = max_abs_y * 1.3;
        let sx = canvas_rect.width()  * 0.5 / extent_x;
        let sy = canvas_rect.height() * 0.5 / extent_y;
        sx.min(sy).max(10.0)
    };

    // Background
    painter.rect_filled(canvas_rect, 4.0, egui::Color32::from_rgb(28, 30, 34));

    // Grid + axis tick labels
    let grid_col  = egui::Color32::from_rgb(45, 48, 52);
    let tick_col  = egui::Color32::from_rgb(100, 105, 120);
    let label_col = egui::Color32::from_rgb(110, 115, 135);
    let font      = egui::FontId::proportional(10.0);

    // Grid spacing: every 0.5 model units.
    // Label every whole unit that falls on an even grid line.
    let half_step = scale * 0.5;          // pixels per 0.5 model unit
    let full_step = scale;                // pixels per 1.0 model unit

    // Determine integer range visible on screen
    let half_w_units = (canvas_rect.width()  * 0.5 / full_step).ceil() as i32 + 1;
    let half_h_units = (canvas_rect.height() * 0.5 / full_step).ceil() as i32 + 1;

    for i in -half_w_units * 2..=half_w_units * 2 {
        let x = center.x + i as f32 * half_step;
        if x < canvas_rect.left() || x > canvas_rect.right() { continue; }
        let bright = i % 2 == 0; // whole-unit lines slightly brighter
        let col = if bright { egui::Color32::from_rgb(55, 58, 65) } else { grid_col };
        painter.line_segment(
            [egui::Pos2::new(x, canvas_rect.top()), egui::Pos2::new(x, canvas_rect.bottom())],
            egui::Stroke::new(if bright { 1.2 } else { 0.8 }, col),
        );
        // X-axis tick label at whole units (skip 0 — drawn separately)
        if bright && i != 0 {
            let val = i / 2;
            painter.text(
                egui::Pos2::new(x + 2.0, center.y + 3.0),
                egui::Align2::LEFT_TOP,
                format!("{}", val),
                font.clone(),
                label_col,
            );
        }
    }

    for i in -half_h_units * 2..=half_h_units * 2 {
        let y = center.y + i as f32 * half_step;
        if y < canvas_rect.top() || y > canvas_rect.bottom() { continue; }
        let bright = i % 2 == 0;
        let col = if bright { egui::Color32::from_rgb(55, 58, 65) } else { grid_col };
        painter.line_segment(
            [egui::Pos2::new(canvas_rect.left(), y), egui::Pos2::new(canvas_rect.right(), y)],
            egui::Stroke::new(if bright { 1.2 } else { 0.8 }, col),
        );
        // Y-axis tick label at whole units (skip 0, note i negated because canvas Y is down)
        if bright && i != 0 {
            let val = -i / 2;
            painter.text(
                egui::Pos2::new(center.x + 3.0, y - 11.0),
                egui::Align2::LEFT_TOP,
                format!("{}", val),
                font.clone(),
                label_col,
            );
        }
    }

    // Axes (drawn on top of grid)
    let ax_col = egui::Color32::from_rgb(80, 88, 110);
    painter.line_segment(
        [egui::Pos2::new(canvas_rect.left(), center.y), egui::Pos2::new(canvas_rect.right(), center.y)],
        egui::Stroke::new(1.5, ax_col),
    );
    painter.line_segment(
        [egui::Pos2::new(center.x, canvas_rect.top()), egui::Pos2::new(center.x, canvas_rect.bottom())],
        egui::Stroke::new(1.5, ax_col),
    );

    // Origin "0" label
    painter.text(
        egui::Pos2::new(center.x + 3.0, center.y + 3.0),
        egui::Align2::LEFT_TOP,
        "0",
        font.clone(),
        tick_col,
    );

    // Axis labels in corners
    painter.text(
        egui::Pos2::new(canvas_rect.right() - 14.0, center.y - 14.0),
        egui::Align2::RIGHT_TOP,
        "X",
        egui::FontId::proportional(11.0),
        tick_col,
    );
    painter.text(
        egui::Pos2::new(center.x + 5.0, canvas_rect.top() + 2.0),
        egui::Align2::LEFT_TOP,
        "Y",
        egui::FontId::proportional(11.0),
        tick_col,
    );

    // Coordinate transforms
    let m2c = |[x, y]: [f32; 2]| -> egui::Pos2 {
        egui::Pos2::new(center.x + x * scale, center.y - y * scale)
    };
    let c2m = |pos: egui::Pos2| -> [f32; 2] {
        [(pos.x - center.x) / scale, -(pos.y - center.y) / scale]
    };

    // Spline curve
    let profile = state.to_profile();
    let sampled = profile.sample(200);
    if sampled.len() >= 2 {
        let poly: Vec<egui::Pos2> = sampled.iter().map(|v| m2c([v.x, v.y])).collect();
        painter.add(egui::Shape::Path(egui::epaint::PathShape {
            points: poly.clone(),
            closed: true,
            fill: egui::Color32::from_rgba_unmultiplied(70, 115, 175, 45),
            stroke: egui::epaint::PathStroke::NONE,
        }));
        painter.add(egui::Shape::Path(egui::epaint::PathShape {
            points: poly,
            closed: true,
            fill: egui::Color32::TRANSPARENT,
            stroke: egui::epaint::PathStroke::new(2.0, egui::Color32::from_rgb(90, 155, 220)),
        }));
    }

    // ── Interactions ─────────────────────────────────────────────────────

    let handle_r = 7.0f32;
    let pointer = response.interact_pointer_pos();
    let n = state.control_points.len();

    let hovered = pointer.and_then(|ptr| {
        state.control_points.iter().enumerate().find(|(_, pt)| {
            let cp = m2c(**pt);
            (cp.x - ptr.x).hypot(cp.y - ptr.y) < handle_r + 5.0
        }).map(|(i, _)| i)
    });

    if response.drag_started() {
        if let Some(idx) = hovered {
            state.dragging_point = Some(idx);
            state.selected_point = Some(idx);
            state.preset_kind = None; // free-hand edit clears preset tracking
        }
    }

    if response.dragged() {
        if let (Some(di), Some(ptr)) = (state.dragging_point, pointer) {
            if canvas_rect.contains(ptr) {
                state.control_points[di] = c2m(ptr);
                state.apply_symmetry(di);
                changed = true;
            }
        }
    }

    if response.drag_stopped() {
        state.dragging_point = None;
    }

    // Click on empty area → insert new point
    if response.clicked() && hovered.is_none() {
        if let Some(ptr) = pointer {
            if canvas_rect.contains(ptr) {
                let new_pt = c2m(ptr);
                let ins = find_insert_index(&state.control_points, new_pt, &m2c);
                state.control_points.insert(ins, new_pt);
                state.point_roles.insert(ins, PointRole::Free);
                for idx in state.sharp_points.iter_mut() { if *idx >= ins { *idx += 1; } }
                state.selected_point = Some(ins);
                state.preset_kind = None;
                changed = true;
            }
        }
    }

    // Right-click on handle → context menu (role assignment + delete)
    if response.secondary_clicked() {
        if let Some(idx) = hovered {
            state.selected_point = Some(idx);
        }
    }
    if let Some(sel) = state.selected_point {
        if sel < state.control_points.len() {
            response.context_menu(|ui| {
                ui.label(egui::RichText::new(format!("Point {}", sel)).strong());
                ui.separator();
                ui.label("Role:");
                for role in [PointRole::Free, PointRole::Keel, PointRole::Deck, PointRole::Chine] {
                    let name = match role {
                        PointRole::Free  => "Free",
                        PointRole::Keel  => "Keel (red)",
                        PointRole::Deck  => "Deck (blue)",
                        PointRole::Chine => "Chine (yellow)",
                    };
                    let current = state.point_roles.get(sel).copied().unwrap_or(PointRole::Free) == role;
                    if ui.selectable_label(current, name).clicked() {
                        if sel < state.point_roles.len() {
                            state.point_roles[sel] = role;
                        }
                        changed = true;
                        ui.close_menu();
                    }
                }
                ui.separator();
                let can_delete = state.control_points.len() > 3;
                if ui.add_enabled(can_delete, egui::Button::new("Delete point")).clicked() {
                    state.control_points.remove(sel);
                    state.point_roles.remove(sel);
                    state.sharp_points.retain(|&i| i != sel);
                    for i in state.sharp_points.iter_mut() { if *i > sel { *i -= 1; } }
                    state.selected_point = None;
                    state.preset_kind = None;
                    changed = true;
                    ui.close_menu();
                }
            });
        }
    }

    // ── Draw handles ─────────────────────────────────────────────────────

    for (i, &pt) in state.control_points.iter().enumerate() {
        let cp = m2c(pt);
        let selected = state.selected_point == Some(i);
        let dragging = state.dragging_point == Some(i);
        let sharp    = state.sharp_points.contains(&i);
        let hov      = hovered == Some(i);
        let role     = state.point_roles.get(i).copied().unwrap_or(PointRole::Free);

        let color = if selected || dragging {
            egui::Color32::from_rgb(255, 210, 60)
        } else if hov {
            egui::Color32::from_rgb(200, 230, 255)
        } else {
            match role {
                PointRole::Keel  => egui::Color32::from_rgb(220, 80,  80),
                PointRole::Deck  => egui::Color32::from_rgb(80,  140, 230),
                PointRole::Chine => egui::Color32::from_rgb(220, 190, 60),
                PointRole::Free  => if sharp {
                    egui::Color32::from_rgb(210, 120, 80)
                } else {
                    egui::Color32::from_rgb(160, 190, 230)
                },
            }
        };

        if sharp {
            painter.rect_filled(
                egui::Rect::from_center_size(cp, egui::Vec2::splat(handle_r * 2.0)),
                2.0, color,
            );
        } else {
            painter.circle_filled(cp, handle_r, color);
            painter.circle_stroke(cp, handle_r, egui::Stroke::new(1.0, egui::Color32::from_rgb(60, 70, 90)));
        }

        if n <= 20 {
            painter.text(
                egui::Pos2::new(cp.x + handle_r + 3.0, cp.y - handle_r),
                egui::Align2::LEFT_TOP,
                format!("{}", i),
                egui::FontId::proportional(9.0),
                egui::Color32::from_rgb(140, 148, 170),
            );
        }
    }

    // ── Bounding-box dimension overlay ───────────────────────────────────
    if state.control_points.len() >= 2 {
        let (mut min_x, mut max_x, mut min_y, mut max_y) =
            (f32::MAX, f32::MIN, f32::MAX, f32::MIN);
        for &[x, y] in &state.control_points {
            if x < min_x { min_x = x; } if x > max_x { max_x = x; }
            if y < min_y { min_y = y; } if y > max_y { max_y = y; }
        }
        let width  = max_x - min_x;
        let height = max_y - min_y;
        let dim_col  = egui::Color32::from_rgba_unmultiplied(130, 185, 240, 140);
        let dim_font = egui::FontId::proportional(9.5);
        let tick = 4.0f32;
        let pad  = (max_x - min_x).max(max_y - min_y) * 0.18 + 0.25;

        // Horizontal dimension: below profile
        let y_dim = min_y - pad;
        let pl = m2c([min_x, y_dim]);
        let pr = m2c([max_x, y_dim]);
        painter.line_segment([pl, pr], egui::Stroke::new(1.0, dim_col));
        for px in [pl.x, pr.x] {
            painter.line_segment(
                [egui::Pos2::new(px, pl.y - tick), egui::Pos2::new(px, pl.y + tick)],
                egui::Stroke::new(1.0, dim_col));
        }
        painter.text(egui::Pos2::new((pl.x + pr.x) * 0.5, pl.y - tick - 2.0),
            egui::Align2::CENTER_BOTTOM,
            format!("W  {:.3} u", width), dim_font.clone(), dim_col);

        // Vertical dimension: right of profile
        let x_dim = max_x + pad;
        let pb = m2c([x_dim, min_y]);
        let pt = m2c([x_dim, max_y]);
        painter.line_segment([pb, pt], egui::Stroke::new(1.0, dim_col));
        for py in [pb.y, pt.y] {
            painter.line_segment(
                [egui::Pos2::new(pb.x - tick, py), egui::Pos2::new(pb.x + tick, py)],
                egui::Stroke::new(1.0, dim_col));
        }
        painter.text(egui::Pos2::new(pb.x + tick + 3.0, (pb.y + pt.y) * 0.5),
            egui::Align2::LEFT_CENTER,
            format!("H  {:.3} u", height), dim_font.clone(), dim_col);
    }

    // ── Selected-point properties ─────────────────────────────────────────

    ui.separator();
    if let Some(sel) = state.selected_point {
        if sel < state.control_points.len() {
            let [mut px, mut py] = state.control_points[sel];
            let role = state.point_roles.get(sel).copied().unwrap_or(PointRole::Free);
            let role_label = match role {
                PointRole::Free  => egui::RichText::new("Free").weak(),
                PointRole::Keel  => egui::RichText::new("Keel").color(egui::Color32::from_rgb(220, 80, 80)),
                PointRole::Deck  => egui::RichText::new("Deck").color(egui::Color32::from_rgb(80, 140, 230)),
                PointRole::Chine => egui::RichText::new("Chine").color(egui::Color32::from_rgb(220, 190, 60)),
            };

            ui.horizontal(|ui| {
                ui.label(egui::RichText::new(format!("Pt {}", sel)).strong());
                ui.label("X:");
                let xr = ui.add(egui::DragValue::new(&mut px).speed(0.005).suffix(" u"));
                ui.label("Y:");
                let yr = ui.add(egui::DragValue::new(&mut py).speed(0.005).suffix(" u"));
                if xr.changed() || yr.changed() {
                    state.control_points[sel] = [px, py];
                    state.apply_symmetry(sel);
                    state.preset_kind = None;
                    changed = true;
                }
                ui.label(role_label);
                ui.label(egui::RichText::new("(right-click to set role)").weak().small());
            });

            ui.horizontal(|ui| {
                let mut sharp = state.sharp_points.contains(&sel);
                if ui.checkbox(&mut sharp, "Sharp corner").changed() {
                    if sharp { state.sharp_points.push(sel); }
                    else     { state.sharp_points.retain(|&i| i != sel); }
                    changed = true;
                }
                if ui.small_button("Delete").clicked() && n > 3 {
                    state.control_points.remove(sel);
                    if sel < state.point_roles.len() { state.point_roles.remove(sel); }
                    state.sharp_points.retain(|&i| i != sel);
                    for i in state.sharp_points.iter_mut() { if *i > sel { *i -= 1; } }
                    state.selected_point = None;
                    changed = true;
                }
            });

            // Distance to adjacent control points
            let nc = state.control_points.len();
            let prev_idx = if sel == 0 { nc - 1 } else { sel - 1 };
            let next_idx = (sel + 1) % nc;
            let [ax, ay] = state.control_points[prev_idx];
            let [bx, by] = state.control_points[next_idx];
            let d_prev = ((px - ax).powi(2) + (py - ay).powi(2)).sqrt();
            let d_next = ((bx - px).powi(2) + (by - py).powi(2)).sqrt();
            let dim_col = egui::Color32::from_rgb(120, 130, 160);
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new(
                    format!("← Pt{}  ΔX {:.3}  ΔY {:.3}  d {:.3}",
                        prev_idx, (px-ax).abs(), (py-ay).abs(), d_prev)
                ).small().color(dim_col));
                ui.separator();
                ui.label(egui::RichText::new(
                    format!("Pt{} →  ΔX {:.3}  ΔY {:.3}  d {:.3}",
                        next_idx, (bx-px).abs(), (by-py).abs(), d_next)
                ).small().color(dim_col));
            });
        }
    } else {
        ui.label(
            egui::RichText::new("Click handle to select  |  Click canvas to add  |  Right-click handle for role/delete")
                .weak().small(),
        );
    }

    changed
}

// ── Helpers ───────────────────────────────────────────────────────────────────

fn find_insert_index(
    pts: &[[f32; 2]],
    new_pt: [f32; 2],
    m2c: &impl Fn([f32; 2]) -> egui::Pos2,
) -> usize {
    let n = pts.len();
    if n == 0 { return 0; }
    let np = m2c(new_pt);
    (0..n)
        .min_by(|&i, &j| {
            let di = point_to_seg_dist(np, m2c(pts[i]), m2c(pts[(i + 1) % n]));
            let dj = point_to_seg_dist(np, m2c(pts[j]), m2c(pts[(j + 1) % n]));
            di.partial_cmp(&dj).unwrap()
        })
        .map(|i| i + 1)
        .unwrap_or(0)
}

fn point_to_seg_dist(p: egui::Pos2, a: egui::Pos2, b: egui::Pos2) -> f32 {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let len2 = dx * dx + dy * dy;
    if len2 < 1e-12 { return (p - a).length(); }
    let t = ((p.x - a.x) * dx + (p.y - a.y) * dy / len2).clamp(0.0, 1.0);
    (p - egui::Pos2::new(a.x + t * dx, a.y + t * dy)).length()
}
