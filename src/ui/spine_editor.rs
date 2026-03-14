// Longitudinal spine editor — egui canvas for editing AxisCurve control points
// in the XZ plane (X = span, Z = height).

use eframe::egui;
use crate::sdf::spine::{AxisCurve, LongitudinalSplines};

// ── Target curve selector ─────────────────────────────────────────────────────

/// Which curve is currently being edited in the spine editor.
#[derive(Clone, Copy, PartialEq, Eq, Debug, serde::Serialize, serde::Deserialize)]
pub enum SpineEditorTarget {
    Keel,
    Deck,
    ChineZ,
    ChineY,
}

impl SpineEditorTarget {
    pub fn label(self) -> &'static str {
        match self {
            Self::Keel   => "Keel",
            Self::Deck   => "Deck",
            Self::ChineZ => "Chine Z",
            Self::ChineY => "Chine Y",
        }
    }

    pub fn color(self) -> egui::Color32 {
        match self {
            Self::Keel   => egui::Color32::from_rgb(220, 80,  80),
            Self::Deck   => egui::Color32::from_rgb(80,  140, 230),
            Self::ChineZ => egui::Color32::from_rgb(220, 190, 60),
            Self::ChineY => egui::Color32::from_rgb(80,  200, 130),
        }
    }
}

// ── Spine editor state ────────────────────────────────────────────────────────

#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct SpineEditorState {
    pub active_target: SpineEditorTarget,
    #[serde(skip)]
    pub dragging_point: Option<usize>,
    #[serde(skip)]
    pub selected_point: Option<usize>,
}

impl Default for SpineEditorState {
    fn default() -> Self {
        Self {
            active_target:  SpineEditorTarget::Keel,
            dragging_point: None,
            selected_point: None,
        }
    }
}

// ── Public entry point ────────────────────────────────────────────────────────

/// Draw the longitudinal spine editor in the given `ui` region.
///
/// `splines` is mutated in-place.  Returns `true` when any curve was changed.
/// `fuselage_length` is used to set the X domain of the canvas.
pub fn show_spine_editor(
    ui: &mut egui::Ui,
    state: &mut SpineEditorState,
    splines: &mut LongitudinalSplines,
    fuselage_length: f32,
) -> bool {
    let mut changed = false;

    // ── Curve selector ─────────────────────────────────────────────────────
    ui.horizontal(|ui| {
        ui.label("Edit:");
        for target in [
            SpineEditorTarget::Keel,
            SpineEditorTarget::Deck,
            SpineEditorTarget::ChineZ,
            SpineEditorTarget::ChineY,
        ] {
            let active = state.active_target == target;
            let btn = egui::RichText::new(target.label())
                .color(if active { target.color() } else { egui::Color32::GRAY });
            if ui.selectable_label(active, btn).clicked() {
                state.active_target = target;
                state.selected_point = None;
                state.dragging_point = None;
            }
        }
    });

    let curve = get_curve_mut(splines, state.active_target);
    let color  = state.active_target.color();

    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(state.active_target.label()).color(color).strong());
        let pt_count = curve.as_ref().map(|c| c.control_points.len()).unwrap_or(0);
        ui.label(egui::RichText::new(format!("{} pts", pt_count)).weak().small());
        if ui.small_button("Clear").clicked() {
            *curve = None;
            state.selected_point = None;
            changed = true;
        }
    });

    ui.separator();

    // ── Canvas ─────────────────────────────────────────────────────────────
    let canvas_height = (ui.available_height() - 40.0).max(180.0);
    let canvas_size = egui::Vec2::new(ui.available_width(), canvas_height);
    let (response, painter) = ui.allocate_painter(canvas_size, egui::Sense::click_and_drag());
    let rect = response.rect;

    // X domain: [0, fuselage_length].  Y domain: determined by curve extents.
    let (y_min, y_max) = curve_y_range(curve.as_ref(), fuselage_length);
    let margin = 0.15;
    let y_range = (y_max - y_min).abs().max(0.5);
    let y_lo = y_min - y_range * margin;
    let y_hi = y_max + y_range * margin;

    // Convert model ↔ canvas
    let m2c = |mx: f32, my: f32| -> egui::Pos2 {
        let px = rect.left() + (mx / fuselage_length.max(0.001)) * rect.width();
        let py = rect.bottom() - ((my - y_lo) / (y_hi - y_lo)) * rect.height();
        egui::Pos2::new(px, py)
    };
    let c2m = |pos: egui::Pos2| -> [f32; 2] {
        let mx = ((pos.x - rect.left()) / rect.width()) * fuselage_length;
        let my = y_lo + ((rect.bottom() - pos.y) / rect.height()) * (y_hi - y_lo);
        [mx, my]
    };

    // Background
    painter.rect_filled(rect, 4.0, egui::Color32::from_rgb(28, 30, 34));

    // Grid lines
    draw_spine_grid(&painter, rect, fuselage_length, y_lo, y_hi, &m2c);

    // Draw all curves (dimmed), then active on top
    for (other_target, other_curve) in all_curves(splines) {
        if other_target == state.active_target { continue; }
        if let Some(c) = other_curve {
            draw_curve(&painter, c, other_target.color().linear_multiply(0.3), fuselage_length, &m2c);
        }
    }
    if let Some(c) = get_curve(splines, state.active_target) {
        draw_curve(&painter, c, color, fuselage_length, &m2c);
    }

    // ── Interactions ───────────────────────────────────────────────────────
    let handle_r = 7.0f32;
    let pointer = response.interact_pointer_pos();

    let curve_pts: Vec<[f32; 2]> = get_curve(splines, state.active_target)
        .map(|c| c.control_points.clone())
        .unwrap_or_default();

    let hovered = pointer.and_then(|ptr| {
        curve_pts.iter().enumerate().find(|(_, pt)| {
            let cp = m2c(pt[0], pt[1]);
            (cp.x - ptr.x).hypot(cp.y - ptr.y) < handle_r + 5.0
        }).map(|(i, _)| i)
    });

    if response.drag_started() {
        if let Some(idx) = hovered {
            state.dragging_point = Some(idx);
            state.selected_point = Some(idx);
        }
    }

    if response.dragged() {
        if let (Some(di), Some(ptr)) = (state.dragging_point, pointer) {
            if rect.contains(ptr) {
                let [new_x, new_y] = c2m(ptr);
                let new_x = new_x.clamp(0.0, fuselage_length);
                let c = get_or_insert_curve(splines, state.active_target);
                if di < c.control_points.len() {
                    c.control_points[di] = [new_x, new_y];
                    // Keep sorted by X
                    c.control_points.sort_by(|a, b| a[0].partial_cmp(&b[0]).unwrap_or(std::cmp::Ordering::Equal));
                    // Update dragging index (sort may have moved it)
                    if let Some(new_idx) = c.control_points.iter().position(|&[x, y]| {
                        (x - new_x).abs() < 1e-4 && (y - new_y).abs() < 1e-4
                    }) {
                        state.dragging_point = Some(new_idx);
                        state.selected_point = Some(new_idx);
                    }
                    changed = true;
                }
            }
        }
    }

    if response.drag_stopped() {
        state.dragging_point = None;
    }

    // Click on empty area → insert new point
    if response.clicked() && hovered.is_none() {
        if let Some(ptr) = pointer {
            if rect.contains(ptr) {
                let [new_x, new_y] = c2m(ptr);
                let new_x = new_x.clamp(0.0, fuselage_length);
                let c = get_or_insert_curve(splines, state.active_target);
                c.control_points.push([new_x, new_y]);
                c.control_points.sort_by(|a, b| a[0].partial_cmp(&b[0]).unwrap_or(std::cmp::Ordering::Equal));
                state.selected_point = c.control_points.iter().position(|&[x, y]| {
                    (x - new_x).abs() < 1e-4 && (y - new_y).abs() < 1e-4
                });
                changed = true;
            }
        }
    }

    // Right-click → delete hovered point
    if response.secondary_clicked() {
        if let Some(idx) = hovered {
            let c = get_or_insert_curve(splines, state.active_target);
            if c.control_points.len() > 1 {
                c.control_points.remove(idx);
                if c.control_points.is_empty() {
                    *get_curve_mut(splines, state.active_target) = None;
                }
                if state.selected_point == Some(idx) { state.selected_point = None; }
                changed = true;
            }
        }
    }

    // ── Draw handles ───────────────────────────────────────────────────────
    let curve_pts: Vec<[f32; 2]> = get_curve(splines, state.active_target)
        .map(|c| c.control_points.clone())
        .unwrap_or_default();

    for (i, &[x, y]) in curve_pts.iter().enumerate() {
        let cp = m2c(x, y);
        let sel = state.selected_point == Some(i);
        let drag = state.dragging_point == Some(i);
        let hov = hovered == Some(i);

        let pt_color = if sel || drag {
            egui::Color32::from_rgb(255, 210, 60)
        } else if hov {
            egui::Color32::from_rgb(200, 230, 255)
        } else {
            color
        };

        painter.circle_filled(cp, handle_r, pt_color);
        painter.circle_stroke(cp, handle_r, egui::Stroke::new(1.0, egui::Color32::from_rgb(40, 45, 55)));
    }

    // ── Segment distance annotations on canvas ────────────────────────────
    {
        let ann_col  = egui::Color32::from_rgba_unmultiplied(160, 160, 190, 170);
        let ann_font = egui::FontId::proportional(8.5);
        let tick     = 4.0f32;
        for w in curve_pts.windows(2) {
            let [x0, y0] = w[0];
            let [x1, y1] = w[1];
            let dx = (x1 - x0).abs();
            let dy = (y1 - y0).abs();
            // Horizontal dimension leader (mid-height of segment)
            let mid_y = (y0 + y1) * 0.5;
            let pa = m2c(x0, mid_y);
            let pb = m2c(x1, mid_y);
            painter.line_segment([pa, pb], egui::Stroke::new(0.8, ann_col));
            painter.line_segment([egui::Pos2::new(pa.x, pa.y - tick), egui::Pos2::new(pa.x, pa.y + tick)], egui::Stroke::new(0.8, ann_col));
            painter.line_segment([egui::Pos2::new(pb.x, pb.y - tick), egui::Pos2::new(pb.x, pb.y + tick)], egui::Stroke::new(0.8, ann_col));
            let mid_canvas = egui::Pos2::new((pa.x + pb.x) * 0.5, pa.y - 11.0);
            painter.text(mid_canvas, egui::Align2::CENTER_BOTTOM,
                format!("ΔX {:.3}", dx), ann_font.clone(), ann_col);
            // Vertical dimension leader (mid-X of segment)
            if dy > 1e-4 {
                let mid_x = (x0 + x1) * 0.5;
                let pc = m2c(mid_x, y0);
                let pd = m2c(mid_x, y1);
                painter.line_segment([pc, pd], egui::Stroke::new(0.8, ann_col));
                painter.line_segment([egui::Pos2::new(pc.x - tick, pc.y), egui::Pos2::new(pc.x + tick, pc.y)], egui::Stroke::new(0.8, ann_col));
                painter.line_segment([egui::Pos2::new(pd.x - tick, pd.y), egui::Pos2::new(pd.x + tick, pd.y)], egui::Stroke::new(0.8, ann_col));
                let mid_v = egui::Pos2::new(pc.x + 5.0, (pc.y + pd.y) * 0.5);
                painter.text(mid_v, egui::Align2::LEFT_CENTER,
                    format!("ΔY {:.3}", dy), ann_font.clone(), ann_col);
            }
        }
    }

    // ── Selected-point info ────────────────────────────────────────────────
    ui.separator();
    if let Some(sel) = state.selected_point {
        let c_pts: Vec<[f32; 2]> = get_curve(splines, state.active_target)
            .map(|c| c.control_points.clone())
            .unwrap_or_default();
        if sel < c_pts.len() {
            let [cur_x, cur_y] = c_pts[sel];
            let mut x_val = cur_x;
            let mut y_val = cur_y;
            let n_pts = c_pts.len();

            ui.horizontal(|ui| {
                ui.label(egui::RichText::new(format!("Pt {}", sel)).strong());
                ui.label("X:");
                let xr = ui.add(egui::DragValue::new(&mut x_val)
                    .range(0.0..=fuselage_length).speed(0.005).suffix(" u"));
                ui.label("Y:");
                let yr = ui.add(egui::DragValue::new(&mut y_val)
                    .speed(0.005).suffix(" u"));

                if xr.changed() || yr.changed() {
                    let x_clamped = x_val.clamp(0.0, fuselage_length);
                    let c = get_or_insert_curve(splines, state.active_target);
                    c.control_points[sel] = [x_clamped, y_val];
                    c.control_points.sort_by(|a, b| {
                        a[0].partial_cmp(&b[0]).unwrap_or(std::cmp::Ordering::Equal)
                    });
                    // Re-find index after sort
                    state.selected_point = c.control_points.iter().position(|p| {
                        (p[0] - x_clamped).abs() < 1e-4 && (p[1] - y_val).abs() < 1e-4
                    });
                    changed = true;
                }

                if ui.small_button("Delete").clicked() && n_pts > 1 {
                    let c = get_or_insert_curve(splines, state.active_target);
                    c.control_points.remove(sel);
                    state.selected_point = None;
                    changed = true;
                }
            });

            // Distance to neighbours
            let dim_col = egui::Color32::from_rgb(130, 140, 165);
            if sel > 0 {
                let [px, py] = c_pts[sel - 1];
                let dx = (cur_x - px).abs();
                let dy = (cur_y - py).abs();
                let d  = ((cur_x - px).powi(2) + (cur_y - py).powi(2)).sqrt();
                ui.label(egui::RichText::new(
                    format!("← prev  ΔX {:.3}  ΔY {:.3}  dist {:.3}", dx, dy, d)
                ).small().color(dim_col));
            }
            if sel + 1 < n_pts {
                let [nx, ny] = c_pts[sel + 1];
                let dx = (nx - cur_x).abs();
                let dy = (ny - cur_y).abs();
                let d  = ((nx - cur_x).powi(2) + (ny - cur_y).powi(2)).sqrt();
                ui.label(egui::RichText::new(
                    format!("→ next  ΔX {:.3}  ΔY {:.3}  dist {:.3}", dx, dy, d)
                ).small().color(dim_col));
            }
        }
    } else {
        ui.label(
            egui::RichText::new("Click canvas to add  |  Drag to move  |  Right-click to delete")
                .weak().small(),
        );
    }

    changed
}

// ── Curve drawing helpers ─────────────────────────────────────────────────────

fn draw_curve(
    painter: &egui::Painter,
    curve: &AxisCurve,
    color: egui::Color32,
    fuselage_length: f32,
    m2c: &impl Fn(f32, f32) -> egui::Pos2,
) {
    let pts = &curve.control_points;
    if pts.len() < 2 { return; }

    let samples = 120;
    let x0 = pts[0][0];
    let x1 = pts[pts.len() - 1][0];
    let polyline: Vec<egui::Pos2> = (0..=samples).filter_map(|i| {
        let x = x0 + (x1 - x0) * i as f32 / samples as f32;
        let y = curve.eval(x)?;
        Some(m2c(x, y))
    }).collect();

    for w in polyline.windows(2) {
        painter.line_segment([w[0], w[1]], egui::Stroke::new(2.0, color));
    }

    // Dashed extension to fuselage ends
    let dash_col = color.linear_multiply(0.4);
    let p_start = m2c(0.0, curve.eval(x0).unwrap_or(0.0));
    let p_x0    = m2c(x0, curve.eval(x0).unwrap_or(0.0));
    painter.line_segment([p_start, p_x0], egui::Stroke::new(1.0, dash_col));

    let p_end = m2c(fuselage_length, curve.eval(x1).unwrap_or(0.0));
    let p_x1  = m2c(x1, curve.eval(x1).unwrap_or(0.0));
    painter.line_segment([p_x1, p_end], egui::Stroke::new(1.0, dash_col));
}

fn draw_spine_grid(
    painter: &egui::Painter,
    rect: egui::Rect,
    fuselage_length: f32,
    y_lo: f32,
    y_hi: f32,
    m2c: &impl Fn(f32, f32) -> egui::Pos2,
) {
    let grid_col  = egui::Color32::from_rgb(45, 48, 52);
    let label_col = egui::Color32::from_rgb(110, 115, 135);
    let font      = egui::FontId::proportional(9.0);

    // Vertical grid lines at integer X (span) positions
    let n_x = fuselage_length.ceil() as i32;
    for xi in 0..=n_x {
        let x = xi as f32;
        let p0 = m2c(x, y_lo);
        let p1 = m2c(x, y_hi);
        painter.line_segment([p0, p1], egui::Stroke::new(0.8, grid_col));
        painter.text(egui::Pos2::new(p0.x + 2.0, rect.bottom() - 12.0),
            egui::Align2::LEFT_BOTTOM, format!("{}", xi), font.clone(), label_col);
    }

    // Horizontal grid lines at round Y values
    let y_step = nice_step((y_hi - y_lo) / 6.0);
    let y_start = (y_lo / y_step).floor() * y_step;
    let mut y = y_start;
    while y <= y_hi + y_step * 0.1 {
        let p0 = m2c(0.0, y);
        let p1 = m2c(fuselage_length, y);
        let on_zero = y.abs() < y_step * 0.1;
        let col = if on_zero { egui::Color32::from_rgb(70, 75, 85) } else { grid_col };
        painter.line_segment([p0, p1], egui::Stroke::new(if on_zero { 1.2 } else { 0.8 }, col));
        painter.text(egui::Pos2::new(rect.left() + 3.0, p0.y - 10.0),
            egui::Align2::LEFT_TOP, format!("{:.1}", y), font.clone(), label_col);
        y += y_step;
    }
}

fn nice_step(raw: f32) -> f32 {
    let e = raw.log10().floor();
    let b = 10_f32.powf(e);
    let f = raw / b;
    let s = if f < 1.5 { 1.0 } else if f < 3.5 { 2.0 } else if f < 7.5 { 5.0 } else { 10.0 };
    (s * b).max(0.01)
}

fn curve_y_range(curve: Option<&AxisCurve>, fuselage_length: f32) -> (f32, f32) {
    if let Some(c) = curve {
        if !c.control_points.is_empty() {
            let (mut lo, mut hi) = (f32::MAX, f32::MIN);
            for &[_, y] in &c.control_points {
                if y < lo { lo = y; }
                if y > hi { hi = y; }
            }
            if (hi - lo).abs() < 1e-4 { lo -= 1.0; hi += 1.0; }
            return (lo, hi);
        }
    }
    // Default domain
    let half = fuselage_length * 0.15;
    (-half, half)
}

// ── Curve access helpers ──────────────────────────────────────────────────────

fn get_curve<'a>(splines: &'a LongitudinalSplines, target: SpineEditorTarget) -> Option<&'a AxisCurve> {
    match target {
        SpineEditorTarget::Keel   => splines.spine.keel.as_ref(),
        SpineEditorTarget::Deck   => splines.spine.deck.as_ref(),
        SpineEditorTarget::ChineZ => splines.chine.chine_z.as_ref(),
        SpineEditorTarget::ChineY => splines.chine.chine_y.as_ref(),
    }
}

fn get_curve_mut<'a>(splines: &'a mut LongitudinalSplines, target: SpineEditorTarget) -> &'a mut Option<AxisCurve> {
    match target {
        SpineEditorTarget::Keel   => &mut splines.spine.keel,
        SpineEditorTarget::Deck   => &mut splines.spine.deck,
        SpineEditorTarget::ChineZ => &mut splines.chine.chine_z,
        SpineEditorTarget::ChineY => &mut splines.chine.chine_y,
    }
}

fn get_or_insert_curve<'a>(splines: &'a mut LongitudinalSplines, target: SpineEditorTarget) -> &'a mut AxisCurve {
    let slot = get_curve_mut(splines, target);
    if slot.is_none() {
        *slot = Some(AxisCurve::new(vec![]));
    }
    slot.as_mut().unwrap()
}

fn all_curves(splines: &LongitudinalSplines) -> Vec<(SpineEditorTarget, Option<&AxisCurve>)> {
    vec![
        (SpineEditorTarget::Keel,   splines.spine.keel.as_ref()),
        (SpineEditorTarget::Deck,   splines.spine.deck.as_ref()),
        (SpineEditorTarget::ChineZ, splines.chine.chine_z.as_ref()),
        (SpineEditorTarget::ChineY, splines.chine.chine_y.as_ref()),
    ]
}
