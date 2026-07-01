use eframe::{egui, egui_wgpu, wgpu};
use glam::Vec3;
use implicit_cad::mesh::{Mesh, MeshQuality, Vertex, parse_stl};
use implicit_cad::render::{
    AxesRenderer, Camera, GridRenderer, RaymarchRenderer, SdfGrid, SectionUniforms, StandardView,
    ThicknessUniforms, WireframeRenderer,
};
use implicit_cad::scripting::{self, MeshCache};
use implicit_cad::sdf::Sdf;
use implicit_cad::sdf::booleans::Union;
use indexmap::IndexMap;
use serde_json::Value;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, mpsc};
use std::time::{Instant, SystemTime};
use wgpu::util::DeviceExt;

#[derive(Clone)]
struct LoadStats {
    source: String,
    artifact_path: Option<PathBuf>,
    bounds_source: String,
    total_ms: u128,
    eval_ms: u128,
    bounds_ms: u128,
    grid_ms: u128,
    mesh_ms: u128,
    grid_resolution: u32,
    triangle_count: usize,
    bounds_min: Vec3,
    bounds_max: Vec3,
}

struct LoadSuccess {
    path: PathBuf,
    grid: Option<Arc<SdfGrid>>,
    mesh: Option<Arc<Mesh>>,
    stats: LoadStats,
}

struct LoadFailure {
    path: PathBuf,
    message: String,
}

type LoadResponse = Result<LoadSuccess, LoadFailure>;

#[derive(Clone)]
struct ModelEntry {
    label: String,
    result_path: PathBuf,
    script_path: Option<PathBuf>,
    stl_path: Option<PathBuf>,
    status: String,
    triangle_count: Option<usize>,
}

pub struct ViewerApp {
    pending_path: Option<PathBuf>,
    current_path: Option<PathBuf>,
    watch_target_file: Option<PathBuf>,
    watch_target_modified: Option<SystemTime>,
    optimizer_dir: Option<PathBuf>,
    models: Vec<ModelEntry>,
    model_filter: String,
    show_models: bool,
    grid: Option<Arc<SdfGrid>>,
    mesh: Option<Arc<Mesh>>,
    mesh_revision: u64,
    camera: Camera,
    receiver: Option<mpsc::Receiver<LoadResponse>>,
    loading: bool,
    status: String,
    error: Option<String>,
    stats: Option<LoadStats>,
    grid_resolution: u32,
    preview_cell_mm: f32,
    smooth_normals: bool,
    show_wireframe: bool,
    prefer_live_sdf: bool,
    grid_only_preview: bool,
    last_mouse_pos: Option<egui::Pos2>,
}

impl ViewerApp {
    pub fn new(path: Option<PathBuf>) -> Self {
        let optimizer_dir = discover_optimizer_dir();
        let models = optimizer_dir
            .as_ref()
            .map(|dir| discover_optimizer_models(dir))
            .unwrap_or_default();
        let status = if models.is_empty() {
            "Open a Rhai SDF/OML script, optimizer result, or STL.".to_string()
        } else {
            format!("Select one of {} optimizer models.", models.len())
        };

        Self {
            pending_path: path,
            current_path: None,
            watch_target_file: None,
            watch_target_modified: None,
            optimizer_dir,
            models,
            model_filter: String::new(),
            show_models: true,
            grid: None,
            mesh: None,
            mesh_revision: 0,
            camera: Camera::new(16.0 / 9.0),
            receiver: None,
            loading: false,
            status,
            error: None,
            stats: None,
            grid_resolution: 64,
            preview_cell_mm: 8.0,
            smooth_normals: false,
            show_wireframe: false,
            prefer_live_sdf: false,
            grid_only_preview: false,
            last_mouse_pos: None,
        }
    }

    pub fn quick_sdf_preview(mut self) -> Self {
        self.prefer_live_sdf = true;
        self.grid_only_preview = true;
        self.grid_resolution = 48;
        self.preview_cell_mm = 10.0;
        self
    }

    pub fn with_watch_target_file(mut self, path: PathBuf) -> Self {
        self.watch_target_file = Some(path);
        self.show_models = false;
        self
    }

    pub fn set_target_path(&mut self, path: Option<PathBuf>) {
        let Some(path) = path else {
            self.pending_path = None;
            return;
        };

        let target_key = normalized_path_key(&path);
        let current_key = self.current_path.as_ref().map(|p| normalized_path_key(p));
        let pending_key = self.pending_path.as_ref().map(|p| normalized_path_key(p));

        if current_key.as_deref() == Some(target_key.as_str())
            || pending_key.as_deref() == Some(target_key.as_str())
        {
            return;
        }

        self.pending_path = Some(path);
    }

    fn poll_watch_target(&mut self) {
        let Some(path) = self.watch_target_file.clone() else {
            return;
        };
        let Ok(metadata) = fs::metadata(&path) else {
            return;
        };
        let modified = metadata.modified().ok();
        if modified.is_some() && modified == self.watch_target_modified {
            return;
        }
        self.watch_target_modified = modified;
        let Ok(text) = fs::read_to_string(&path) else {
            return;
        };
        let target = text.trim();
        if target.is_empty() {
            return;
        }
        self.set_target_path(Some(PathBuf::from(target)));
    }

    pub fn embedded_ui(&mut self, ui: &mut egui::Ui) {
        if let Some(path) = self.pending_path.take() {
            self.start_load(path);
        }
        self.receive_loads();

        ui.vertical(|ui| {
            ui.horizontal_wrapped(|ui| {
                if ui
                    .add_enabled(
                        self.current_path.is_some() && !self.loading,
                        egui::Button::new("Reload"),
                    )
                    .clicked()
                {
                    if let Some(path) = self.current_path.clone() {
                        self.start_load(path);
                    }
                }
                ui.separator();
                ui.label("Grid");
                ui.add(
                    egui::DragValue::new(&mut self.grid_resolution)
                        .range(32..=192)
                        .speed(8),
                );
                ui.label("Cell mm");
                ui.add(
                    egui::DragValue::new(&mut self.preview_cell_mm)
                        .range(0.5..=50.0)
                        .speed(0.5),
                );
                ui.checkbox(&mut self.smooth_normals, "Smooth");
                ui.checkbox(&mut self.show_wireframe, "Wire");
                ui.separator();
                if ui.button("Frame").clicked() {
                    if let Some(stats) = &self.stats {
                        let bounds_min = stats.bounds_min;
                        let bounds_max = stats.bounds_max;
                        self.frame_bounds(bounds_min, bounds_max);
                    }
                }
                if ui.button("Iso").clicked() {
                    self.camera.snap_to_view(StandardView::Isometric);
                }
                if ui.button("Side").clicked() {
                    self.camera.snap_to_view(StandardView::Left);
                }
                if ui.button("Top").clicked() {
                    self.camera.snap_to_view(StandardView::Top);
                }
                if self.loading {
                    ui.spinner();
                }
                ui.label(&self.status);
            });

            ui.separator();
            self.viewport(ui);
        });

        if self.loading {
            ui.ctx()
                .request_repaint_after(std::time::Duration::from_millis(50));
        }
    }

    fn start_load(&mut self, path: PathBuf) {
        let (tx, rx) = mpsc::channel();
        let grid_resolution = self.grid_resolution.clamp(32, 192);
        let preview_cell_mm = self.preview_cell_mm.max(0.5);
        let smooth_normals = self.smooth_normals;
        let prefer_live_sdf = self.prefer_live_sdf;
        let grid_only_preview = self.grid_only_preview;
        self.loading = true;
        self.error = None;
        self.status = format!("Loading {}...", display_name(&path));
        self.receiver = Some(rx);

        std::thread::spawn(move || {
            let result = if prefer_live_sdf {
                load_preview_with_options(
                    &path,
                    grid_resolution,
                    preview_cell_mm,
                    smooth_normals,
                    true,
                    grid_only_preview,
                )
            } else {
                load_preview(&path, grid_resolution, preview_cell_mm, smooth_normals)
            }
            .map_err(|message| LoadFailure {
                path: path.clone(),
                message,
            });
            let _ = tx.send(result);
        });
    }

    fn receive_loads(&mut self) {
        let Some(rx) = &self.receiver else {
            return;
        };
        let Ok(response) = rx.try_recv() else {
            return;
        };
        self.receiver = None;
        self.loading = false;

        match response {
            Ok(success) => {
                self.frame_bounds(success.stats.bounds_min, success.stats.bounds_max);
                self.status = format!(
                    "{} loaded: {}, {} tris, total {} ms",
                    display_name(&success.path),
                    success.stats.source,
                    success.stats.triangle_count,
                    success.stats.total_ms
                );
                self.current_path = Some(success.path);
                self.grid = success.grid;
                self.mesh = success.mesh;
                self.mesh_revision = self.mesh_revision.wrapping_add(1);
                self.stats = Some(success.stats);
            }
            Err(failure) => {
                self.status = format!("Failed to load {}", display_name(&failure.path));
                self.error = Some(failure.message);
            }
        }
    }

    fn top_bar(&mut self, ctx: &egui::Context) {
        egui::TopBottomPanel::top("viewer_toolbar").show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.heading("SDF Viewer");
                if self.watch_target_file.is_some() {
                    ui.colored_label(egui::Color32::from_rgb(130, 210, 255), "Dashboard linked");
                }
                ui.separator();
                ui.checkbox(&mut self.show_models, "Models");

                if ui.button("Open").clicked() {
                    let mut dialog = rfd::FileDialog::new()
                        .add_filter("Viewer inputs", &["rhai", "json", "stl"]);
                    if let Some(dir) = &self.optimizer_dir {
                        dialog = dialog.set_directory(dir);
                    }
                    if let Some(path) = dialog.pick_file() {
                        self.start_load(path);
                    }
                }

                if ui.button("Refresh").clicked() {
                    self.refresh_models();
                }

                if ui
                    .add_enabled(
                        self.current_path.is_some() && !self.loading,
                        egui::Button::new("Reload"),
                    )
                    .clicked()
                {
                    if let Some(path) = self.current_path.clone() {
                        self.start_load(path);
                    }
                }

                ui.separator();
                ui.label("Grid");
                ui.add(
                    egui::DragValue::new(&mut self.grid_resolution)
                        .range(32..=192)
                        .speed(8),
                );
                ui.label("Cell mm");
                ui.add(
                    egui::DragValue::new(&mut self.preview_cell_mm)
                        .range(0.5..=50.0)
                        .speed(0.5),
                );
                ui.checkbox(&mut self.smooth_normals, "Smooth");
                ui.checkbox(&mut self.show_wireframe, "Wire");

                ui.separator();
                if ui.button("Frame").clicked() {
                    if let Some(stats) = &self.stats {
                        let bounds_min = stats.bounds_min;
                        let bounds_max = stats.bounds_max;
                        self.frame_bounds(bounds_min, bounds_max);
                    }
                }
                if ui.button("Iso").clicked() {
                    self.camera.snap_to_view(StandardView::Isometric);
                }
                if ui.button("Side").clicked() {
                    self.camera.snap_to_view(StandardView::Left);
                }
                if ui.button("Top").clicked() {
                    self.camera.snap_to_view(StandardView::Top);
                }
                if ui.button("Reset").clicked() {
                    self.camera.reset();
                }

                if self.loading {
                    ui.spinner();
                }
                ui.label(&self.status);
            });
        });
    }

    fn refresh_models(&mut self) {
        self.optimizer_dir = discover_optimizer_dir();
        self.models = self
            .optimizer_dir
            .as_ref()
            .map(|dir| discover_optimizer_models(dir))
            .unwrap_or_default();
        self.status = format!("Found {} optimizer models.", self.models.len());
    }

    fn model_panel(&mut self, ctx: &egui::Context) {
        if !self.show_models {
            return;
        }

        egui::SidePanel::left("optimizer_model_browser")
            .resizable(true)
            .default_width(340.0)
            .width_range(260.0..=520.0)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.heading("Models");
                    if ui.button("Refresh").clicked() {
                        self.refresh_models();
                    }
                });
                ui.add(
                    egui::TextEdit::singleline(&mut self.model_filter)
                        .hint_text("Filter optimizer outputs"),
                );
                if let Some(dir) = &self.optimizer_dir {
                    ui.small(dir.display().to_string());
                }
                ui.separator();

                let filter = self.model_filter.trim().to_ascii_lowercase();
                let visible: Vec<ModelEntry> = self
                    .models
                    .iter()
                    .filter(|entry| {
                        filter.is_empty()
                            || entry.label.to_ascii_lowercase().contains(&filter)
                            || entry.status.to_ascii_lowercase().contains(&filter)
                    })
                    .cloned()
                    .collect();

                egui::ScrollArea::vertical().show(ui, |ui| {
                    for entry in visible {
                        ui.horizontal(|ui| {
                            let selected = self.current_path.as_ref().is_some_and(|path| {
                                normalized_path_key(path)
                                    == normalized_path_key(entry.result_path.as_path())
                                    || entry.stl_path.as_ref().is_some_and(|stl_path| {
                                        normalized_path_key(path) == normalized_path_key(stl_path)
                                    })
                                    || entry.script_path.as_ref().is_some_and(|script_path| {
                                        normalized_path_key(path)
                                            == normalized_path_key(script_path.as_path())
                                    })
                            });
                            if ui
                                .selectable_label(selected, short_model_label(&entry.label))
                                .on_hover_text(entry.result_path.display().to_string())
                                .clicked()
                            {
                                self.start_load(entry.result_path.clone());
                            }
                        });
                        ui.horizontal_wrapped(|ui| {
                            ui.small(&entry.status);
                            if let Some(tris) = entry.triangle_count {
                                ui.small(format!("{} tris", format_count(tris)));
                            }
                            if let Some(stl_path) = &entry.stl_path {
                                if !stl_path.exists() {
                                    ui.small("missing STL");
                                }
                            }
                        });
                        ui.separator();
                    }
                });
            });
    }

    fn frame_bounds(&mut self, bounds_min: Vec3, bounds_max: Vec3) {
        self.camera.frame_bounds(bounds_min, bounds_max);
        let diagonal = (bounds_max - bounds_min).length().max(1.0);
        self.camera.near = (diagonal * 0.0005).max(0.05);
        self.camera.far = diagonal * 8.0;
    }

    fn stats_panel(&mut self, ctx: &egui::Context) {
        egui::TopBottomPanel::bottom("viewer_stats").show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                if let Some(path) = &self.current_path {
                    ui.label(path.display().to_string());
                }
                if let Some(stats) = &self.stats {
                    ui.separator();
                    ui.label(&stats.source);
                    ui.separator();
                    ui.label(format!("total {} ms", stats.total_ms));
                    if stats.eval_ms > 0 {
                        ui.label(format!("eval {} ms", stats.eval_ms));
                    }
                    if stats.bounds_ms > 0 {
                        ui.label(format!("bounds {} ms", stats.bounds_ms));
                    }
                    if stats.grid_ms > 0 {
                        ui.label(format!("grid {} ms", stats.grid_ms));
                    }
                    if stats.grid_resolution > 0 {
                        ui.label(format!("grid {}^3", stats.grid_resolution));
                    }
                    if stats.mesh_ms > 0 {
                        ui.label(format!("mesh {} ms", stats.mesh_ms));
                    }
                    ui.label(format!("{} triangles", stats.triangle_count));
                    ui.label(format!("bounds: {}", stats.bounds_source));
                    if let Some(artifact_path) = &stats.artifact_path {
                        ui.label(artifact_path.display().to_string());
                    }
                }
            });
        });
    }

    fn viewport(&mut self, ui: &mut egui::Ui) {
        let (rect, response) =
            ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());

        if response.dragged_by(egui::PointerButton::Primary) {
            if let (Some(last), Some(current)) = (self.last_mouse_pos, response.hover_pos()) {
                let delta = current - last;
                self.camera.orbit(delta.x, -delta.y);
            }
        }

        if response.dragged_by(egui::PointerButton::Secondary)
            || (response.dragged_by(egui::PointerButton::Primary)
                && ui.input(|i| i.modifiers.shift))
        {
            if let (Some(last), Some(current)) = (self.last_mouse_pos, response.hover_pos()) {
                let delta = current - last;
                self.camera.pan(delta.x, -delta.y);
            }
        }

        self.last_mouse_pos = response.hover_pos();

        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll.abs() > 0.01 {
                self.camera.zoom(scroll * 0.1);
            }
        }

        if rect.width() > 0.0 && rect.height() > 0.0 {
            self.camera.aspect = rect.width() / rect.height();
        }

        let painter = ui.painter().with_clip_rect(rect);
        painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(16, 18, 20));

        if self.grid.is_none() && self.mesh.is_none() {
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "Open a .rhai, optimizer result .json, or .stl",
                egui::FontId::proportional(16.0),
                egui::Color32::from_rgb(170, 176, 184),
            );
        }

        let cb = ViewerPaint {
            camera: self.camera.clone(),
            grid: self.grid.clone(),
            mesh: self.mesh.clone(),
            mesh_revision: self.mesh_revision,
            show_wireframe: self.show_wireframe,
            viewport_rect: rect,
        };
        ui.painter()
            .add(egui_wgpu::Callback::new_paint_callback(rect, cb));

        if let Some(err) = &self.error {
            let box_rect = egui::Rect::from_min_size(
                rect.min + egui::vec2(16.0, 16.0),
                egui::vec2((rect.width() - 32.0).min(760.0), 120.0),
            );
            painter.rect_filled(box_rect, 6.0, egui::Color32::from_rgb(68, 28, 28));
            painter.text(
                box_rect.min + egui::vec2(12.0, 12.0),
                egui::Align2::LEFT_TOP,
                err,
                egui::FontId::monospace(12.0),
                egui::Color32::from_rgb(255, 220, 220),
            );
        }
    }
}

impl eframe::App for ViewerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        apply_viewer_style(ctx);
        self.poll_watch_target();
        if let Some(path) = self.pending_path.take() {
            self.start_load(path);
        }
        self.receive_loads();
        self.top_bar(ctx);
        self.model_panel(ctx);
        self.stats_panel(ctx);
        egui::CentralPanel::default().show(ctx, |ui| {
            self.viewport(ui);
        });

        if self.loading || self.watch_target_file.is_some() {
            ctx.request_repaint_after(std::time::Duration::from_millis(250));
        }
    }
}

fn apply_viewer_style(ctx: &egui::Context) {
    let mut visuals = egui::Visuals::dark();
    visuals.panel_fill = egui::Color32::from_rgb(17, 20, 24);
    visuals.window_fill = egui::Color32::from_rgb(20, 24, 28);
    visuals.faint_bg_color = egui::Color32::from_rgb(25, 30, 36);
    visuals.extreme_bg_color = egui::Color32::from_rgb(10, 12, 15);
    visuals.selection.bg_fill = egui::Color32::from_rgb(48, 96, 130);
    visuals.selection.stroke = egui::Stroke::new(1.0, egui::Color32::from_rgb(135, 205, 255));
    visuals.widgets.inactive.bg_fill = egui::Color32::from_rgb(28, 34, 40);
    visuals.widgets.hovered.bg_fill = egui::Color32::from_rgb(38, 48, 58);
    visuals.widgets.active.bg_fill = egui::Color32::from_rgb(48, 66, 82);
    ctx.set_visuals(visuals);

    let mut style = (*ctx.style()).clone();
    style.spacing.item_spacing = egui::vec2(8.0, 6.0);
    style.spacing.button_padding = egui::vec2(10.0, 5.0);
    style.spacing.interact_size = egui::vec2(40.0, 24.0);
    ctx.set_style(style);
}

#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct MeshUniforms {
    view_proj: [[f32; 4]; 4],
    light_dir: [f32; 3],
    _padding: f32,
}

struct OffscreenTarget {
    size: [u32; 2],
    color_view: wgpu::TextureView,
    depth_view: wgpu::TextureView,
    bind_group: wgpu::BindGroup,
}

struct DepthMeshRenderer {
    mesh_pipeline: wgpu::RenderPipeline,
    blit_pipeline: wgpu::RenderPipeline,
    uniform_buffer: wgpu::Buffer,
    uniform_bind_group: wgpu::BindGroup,
    texture_bind_group_layout: wgpu::BindGroupLayout,
    sampler: wgpu::Sampler,
    target: Option<OffscreenTarget>,
    vertex_buffer: Option<wgpu::Buffer>,
    index_buffer: Option<wgpu::Buffer>,
    num_indices: u32,
}

impl DepthMeshRenderer {
    fn new(device: &wgpu::Device, output_format: wgpu::TextureFormat) -> Self {
        let mesh_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("SDF Viewer Depth Mesh Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("../../shaders/mesh.wgsl").into()),
        });
        let blit_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("SDF Viewer Mesh Blit Shader"),
            source: wgpu::ShaderSource::Wgsl(
                r#"
struct VertexOut {
    @builtin(position) position: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) vertex_index: u32) -> VertexOut {
    var positions = array<vec2<f32>, 3>(
        vec2<f32>(-1.0, -1.0),
        vec2<f32>(3.0, -1.0),
        vec2<f32>(-1.0, 3.0)
    );
    let pos = positions[vertex_index];
    var out: VertexOut;
    out.position = vec4<f32>(pos, 0.0, 1.0);
    out.uv = vec2<f32>((pos.x + 1.0) * 0.5, (1.0 - pos.y) * 0.5);
    return out;
}

@group(0) @binding(0)
var mesh_texture: texture_2d<f32>;
@group(0) @binding(1)
var mesh_sampler: sampler;

@fragment
fn fs_main(in: VertexOut) -> @location(0) vec4<f32> {
    return textureSample(mesh_texture, mesh_sampler, in.uv);
}
"#
                .into(),
            ),
        });

        let uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("SDF Viewer Depth Mesh Uniforms"),
            size: std::mem::size_of::<MeshUniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let uniform_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("SDF Viewer Depth Mesh Uniform Layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            });
        let uniform_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("SDF Viewer Depth Mesh Uniform Bind Group"),
            layout: &uniform_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buffer.as_entire_binding(),
            }],
        });
        let mesh_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("SDF Viewer Depth Mesh Pipeline Layout"),
            bind_group_layouts: &[&uniform_bind_group_layout],
            push_constant_ranges: &[],
        });
        let mesh_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("SDF Viewer Depth Mesh Pipeline"),
            layout: Some(&mesh_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &mesh_shader,
                entry_point: "vs_main",
                buffers: &[wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &[
                        wgpu::VertexAttribute {
                            offset: 0,
                            shader_location: 0,
                            format: wgpu::VertexFormat::Float32x3,
                        },
                        wgpu::VertexAttribute {
                            offset: std::mem::size_of::<[f32; 3]>() as wgpu::BufferAddress,
                            shader_location: 1,
                            format: wgpu::VertexFormat::Float32x3,
                        },
                    ],
                }],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &mesh_shader,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format: wgpu::TextureFormat::Rgba8Unorm,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                polygon_mode: wgpu::PolygonMode::Fill,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::LessEqual,
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        let texture_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("SDF Viewer Mesh Texture Layout"),
                entries: &[
                    wgpu::BindGroupLayoutEntry {
                        binding: 0,
                        visibility: wgpu::ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Texture {
                            sample_type: wgpu::TextureSampleType::Float { filterable: true },
                            view_dimension: wgpu::TextureViewDimension::D2,
                            multisampled: false,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 1,
                        visibility: wgpu::ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                        count: None,
                    },
                ],
            });
        let blit_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("SDF Viewer Mesh Blit Pipeline Layout"),
            bind_group_layouts: &[&texture_bind_group_layout],
            push_constant_ranges: &[],
        });
        let blit_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("SDF Viewer Mesh Blit Pipeline"),
            layout: Some(&blit_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &blit_shader,
                entry_point: "vs_main",
                buffers: &[],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &blit_shader,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format: output_format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState::default(),
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("SDF Viewer Mesh Texture Sampler"),
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });

        Self {
            mesh_pipeline,
            blit_pipeline,
            uniform_buffer,
            uniform_bind_group,
            texture_bind_group_layout,
            sampler,
            target: None,
            vertex_buffer: None,
            index_buffer: None,
            num_indices: 0,
        }
    }

    fn upload_mesh(&mut self, device: &wgpu::Device, mesh: &Mesh) {
        if mesh.vertices.is_empty() {
            self.vertex_buffer = None;
            self.index_buffer = None;
            self.num_indices = 0;
            return;
        }

        self.vertex_buffer = Some(
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("SDF Viewer Depth Mesh Vertex Buffer"),
                contents: bytemuck::cast_slice(&mesh.vertices),
                usage: wgpu::BufferUsages::VERTEX,
            }),
        );
        self.index_buffer = Some(
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("SDF Viewer Depth Mesh Index Buffer"),
                contents: bytemuck::cast_slice(&mesh.indices),
                usage: wgpu::BufferUsages::INDEX,
            }),
        );
        self.num_indices = mesh.indices.len() as u32;
    }

    fn update_uniforms(&self, queue: &wgpu::Queue, camera: &Camera) {
        let uniforms = MeshUniforms {
            view_proj: camera.view_projection().to_cols_array_2d(),
            light_dir: glam::Vec3::new(0.5, -0.3, -0.8).normalize().to_array(),
            _padding: 0.0,
        };
        queue.write_buffer(&self.uniform_buffer, 0, bytemuck::cast_slice(&[uniforms]));
    }

    fn render_to_offscreen(
        &mut self,
        device: &wgpu::Device,
        encoder: &mut wgpu::CommandEncoder,
        size: [u32; 2],
    ) {
        if self.num_indices == 0 {
            return;
        }
        self.ensure_target(device, size);
        let Some(target) = &self.target else {
            return;
        };

        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("SDF Viewer Depth Mesh Offscreen Pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &target.color_view,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &target.depth_view,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Discard,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
        });
        pass.set_pipeline(&self.mesh_pipeline);
        pass.set_bind_group(0, &self.uniform_bind_group, &[]);
        if let (Some(vb), Some(ib)) = (&self.vertex_buffer, &self.index_buffer) {
            pass.set_vertex_buffer(0, vb.slice(..));
            pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..self.num_indices, 0, 0..1);
        }
    }

    fn ensure_target(&mut self, device: &wgpu::Device, size: [u32; 2]) {
        let size = [size[0].max(1), size[1].max(1)];
        if self
            .target
            .as_ref()
            .is_some_and(|target| target.size == size)
        {
            return;
        }
        let color = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("SDF Viewer Depth Mesh Color Target"),
            size: wgpu::Extent3d {
                width: size[0],
                height: size[1],
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8Unorm,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        });
        let depth = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("SDF Viewer Depth Mesh Depth Target"),
            size: wgpu::Extent3d {
                width: size[0],
                height: size[1],
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth32Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let color_view = color.create_view(&Default::default());
        let depth_view = depth.create_view(&Default::default());
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("SDF Viewer Mesh Texture Bind Group"),
            layout: &self.texture_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&color_view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&self.sampler),
                },
            ],
        });
        self.target = Some(OffscreenTarget {
            size,
            color_view,
            depth_view,
            bind_group,
        });
    }

    fn render(&self, render_pass: &mut wgpu::RenderPass<'static>) {
        let Some(target) = &self.target else {
            return;
        };
        render_pass.set_pipeline(&self.blit_pipeline);
        render_pass.set_bind_group(0, &target.bind_group, &[]);
        render_pass.draw(0..3, 0..1);
    }
}

struct ViewerPaint {
    camera: Camera,
    grid: Option<Arc<SdfGrid>>,
    mesh: Option<Arc<Mesh>>,
    mesh_revision: u64,
    show_wireframe: bool,
    viewport_rect: egui::Rect,
}

#[derive(Default)]
struct ViewerUploadState {
    mesh_revision: u64,
}

impl egui_wgpu::CallbackTrait for ViewerPaint {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        screen_descriptor: &egui_wgpu::ScreenDescriptor,
        egui_encoder: &mut wgpu::CommandEncoder,
        callback_resources: &mut egui_wgpu::CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        let fmt = wgpu::TextureFormat::Bgra8Unorm;
        if callback_resources.get::<GridRenderer>().is_none() {
            callback_resources.insert(GridRenderer::new(device, fmt));
        }
        if callback_resources.get::<AxesRenderer>().is_none() {
            callback_resources.insert(AxesRenderer::new(device, fmt));
        }
        if callback_resources.get::<WireframeRenderer>().is_none() {
            callback_resources.insert(WireframeRenderer::new(device, fmt));
        }
        if callback_resources.get::<RaymarchRenderer>().is_none() {
            callback_resources.insert(RaymarchRenderer::new(device, fmt));
        }
        if callback_resources.get::<DepthMeshRenderer>().is_none() {
            callback_resources.insert(DepthMeshRenderer::new(device, fmt));
        }
        if callback_resources.get::<ViewerUploadState>().is_none() {
            callback_resources.insert(ViewerUploadState::default());
        }

        let upload_needed = {
            let state: &ViewerUploadState = callback_resources.get().unwrap();
            state.mesh_revision != self.mesh_revision
        };
        if upload_needed {
            if let Some(mesh) = &self.mesh {
                let depth_mesh: &mut DepthMeshRenderer = callback_resources.get_mut().unwrap();
                depth_mesh.upload_mesh(device, mesh);
                let wireframe: &mut WireframeRenderer = callback_resources.get_mut().unwrap();
                wireframe.upload_mesh(device, mesh);
            }
            let state: &mut ViewerUploadState = callback_resources.get_mut().unwrap();
            state.mesh_revision = self.mesh_revision;
        }

        let depth_mesh: &mut DepthMeshRenderer = callback_resources.get_mut().unwrap();
        depth_mesh.update_uniforms(queue, &self.camera);

        let grid_renderer: &GridRenderer = callback_resources.get().unwrap();
        grid_renderer.update_uniforms(queue, &self.camera);

        let axes_renderer: &AxesRenderer = callback_resources.get().unwrap();
        axes_renderer.update_uniforms(queue, &self.camera);

        let wireframe: &WireframeRenderer = callback_resources.get().unwrap();
        wireframe.update_uniforms(queue, &self.camera);

        let ppp = screen_descriptor.pixels_per_point;
        let viewport_offset = [
            self.viewport_rect.min.x * ppp,
            self.viewport_rect.min.y * ppp,
        ];
        let viewport_size = [
            self.viewport_rect.width() * ppp,
            self.viewport_rect.height() * ppp,
        ];

        let raymarch: &mut RaymarchRenderer = callback_resources.get_mut().unwrap();
        if let Some(grid) = &self.grid {
            let needs_upload = raymarch
                .last_grid
                .as_ref()
                .map(|last| !Arc::ptr_eq(last, grid))
                .unwrap_or(true);
            if needs_upload {
                raymarch.upload_grid(device, queue, grid);
            }
        }
        raymarch.update_uniforms(
            queue,
            &self.camera,
            viewport_offset,
            viewport_size,
            &SectionUniforms::default(),
            &ThicknessUniforms::default(),
        );

        if self.grid.is_none() && self.mesh.is_some() {
            let offscreen_size = [
                (self.viewport_rect.width() * ppp).ceil().max(1.0) as u32,
                (self.viewport_rect.height() * ppp).ceil().max(1.0) as u32,
            ];
            let depth_mesh: &mut DepthMeshRenderer = callback_resources.get_mut().unwrap();
            depth_mesh.render_to_offscreen(device, egui_encoder, offscreen_size);
        }

        Vec::new()
    }

    fn paint(
        &self,
        _info: egui::PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        callback_resources: &egui_wgpu::CallbackResources,
    ) {
        let grid_renderer: &GridRenderer = callback_resources.get().unwrap();
        let axes_renderer: &AxesRenderer = callback_resources.get().unwrap();
        grid_renderer.render(render_pass);

        if self.grid.is_some() {
            let raymarch: &RaymarchRenderer = callback_resources.get().unwrap();
            raymarch.render(render_pass);
        } else if self.mesh.is_some() {
            let depth_mesh: &DepthMeshRenderer = callback_resources.get().unwrap();
            depth_mesh.render(render_pass);
        }

        if self.show_wireframe && self.mesh.is_some() {
            let wireframe: &WireframeRenderer = callback_resources.get().unwrap();
            wireframe.render(render_pass);
        }
        axes_renderer.render(render_pass);
    }
}

fn load_preview(
    path: &Path,
    grid_resolution: u32,
    preview_cell_mm: f32,
    smooth_normals: bool,
) -> Result<LoadSuccess, String> {
    load_preview_with_options(
        path,
        grid_resolution,
        preview_cell_mm,
        smooth_normals,
        false,
        false,
    )
}

fn load_preview_with_options(
    path: &Path,
    grid_resolution: u32,
    preview_cell_mm: f32,
    smooth_normals: bool,
    prefer_live_sdf: bool,
    grid_only_preview: bool,
) -> Result<LoadSuccess, String> {
    let total_start = Instant::now();
    let extension = path
        .extension()
        .and_then(|ext| ext.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();

    if extension == "stl" {
        return load_stl_preview(
            path,
            None,
            "STL mesh".to_string(),
            "STL bounds".to_string(),
            total_start,
        );
    }

    if extension == "json" {
        let result_json = read_json(path)?;
        return load_optimizer_result_preview(path, result_json, total_start);
    }

    if extension == "rhai" && !prefer_live_sdf {
        if let Some((result_path, result_json)) = find_optimizer_result_for_script(path) {
            if let Ok(success) =
                load_optimizer_result_preview(&result_path, result_json, total_start)
            {
                return Ok(LoadSuccess {
                    path: path.to_path_buf(),
                    ..success
                });
            }
        }
    }

    load_live_sdf_preview(
        path,
        grid_resolution,
        preview_cell_mm,
        smooth_normals,
        grid_only_preview,
        total_start,
    )
}

fn load_optimizer_result_preview(
    result_path: &Path,
    result_json: Value,
    total_start: Instant,
) -> Result<LoadSuccess, String> {
    let stl_path = result_stl_path(&result_json, result_path)
        .ok_or_else(|| format!("No summary.stl_path found in {}", result_path.display()))?;
    let bounds = result_bounds(&result_json);
    let bounds_source = if bounds.is_some() {
        format!("optimizer result {}", display_name(result_path))
    } else {
        "STL bounds".to_string()
    };
    let mut success = load_stl_preview(
        &stl_path,
        bounds,
        format!("optimizer STL artifact ({})", display_name(result_path)),
        bounds_source,
        total_start,
    )?;
    success.path = result_path.to_path_buf();
    Ok(success)
}

fn load_stl_preview(
    path: &Path,
    override_bounds: Option<(Vec3, Vec3)>,
    source: String,
    bounds_source: String,
    total_start: Instant,
) -> Result<LoadSuccess, String> {
    let mesh_start = Instant::now();
    let data = fs::read(path).map_err(|e| format!("Failed to read {}: {}", path.display(), e))?;
    let tri_mesh =
        parse_stl(&data).map_err(|e| format!("Failed to parse {}: {}", path.display(), e))?;
    let (mesh, mesh_bounds_min, mesh_bounds_max) = mesh_from_triangle_mesh(&tri_mesh);
    let mesh_ms = mesh_start.elapsed().as_millis();
    let (bounds_min, bounds_max) = override_bounds.unwrap_or((mesh_bounds_min, mesh_bounds_max));
    let triangle_count = mesh.indices.len() / 3;

    let stats = LoadStats {
        source,
        artifact_path: Some(path.to_path_buf()),
        bounds_source,
        total_ms: total_start.elapsed().as_millis(),
        eval_ms: 0,
        bounds_ms: 0,
        grid_ms: 0,
        mesh_ms,
        grid_resolution: 0,
        triangle_count,
        bounds_min,
        bounds_max,
    };

    Ok(LoadSuccess {
        path: path.to_path_buf(),
        grid: None,
        mesh: Some(Arc::new(mesh)),
        stats,
    })
}

fn load_live_sdf_preview(
    path: &Path,
    grid_resolution: u32,
    preview_cell_mm: f32,
    smooth_normals: bool,
    grid_only_preview: bool,
    total_start: Instant,
) -> Result<LoadSuccess, String> {
    let script = fs::read_to_string(path)
        .map_err(|e| format!("Failed to read {}: {}", path.display(), e))?;
    let script = with_preview_fallbacks(script);
    let project_dir = path.parent();
    let mesh_cache: Arc<Mutex<MeshCache>> = Arc::new(Mutex::new(MeshCache::new()));

    let eval_start = Instant::now();
    let (sdf, source) = match scripting::evaluate_script_full(
        &script,
        None,
        None,
        None,
        None,
        &IndexMap::new(),
        project_dir,
        Some(Arc::clone(&mesh_cache)),
        &[],
    ) {
        Ok(mut result) => {
            result.condition_for_backend();
            (result.sdf, "final SDF expression".to_string())
        }
        Err(primary_err) => {
            let mut parts = scripting::evaluate_aero_export_parts(
                &script,
                None,
                None,
                None,
                None,
                &IndexMap::new(),
                project_dir,
                Some(mesh_cache),
                &[],
            )
            .map_err(|secondary_err| {
                format!(
                    "{}\n\nAero export fallback also failed:\n{}",
                    primary_err, secondary_err
                )
            })?;
            scripting::condition_aero_export_parts_for_backend(&mut parts);
            let oml_parts: Vec<_> = parts
                .into_iter()
                .filter(|part| part.aero_role == "outer_mold_line")
                .collect();
            if oml_parts.is_empty() {
                return Err(
                    "No outer_mold_line parts found in aero_export.parts for preview.".to_string(),
                );
            }
            let part_bounds = combined_part_bounds(&oml_parts);
            let sdf = union_sdfs(oml_parts.iter().map(|part| Arc::clone(&part.sdf)).collect())
                .ok_or("No previewable OML SDF parts found.")?;
            let names = oml_parts
                .iter()
                .map(|part| part.name.as_str())
                .collect::<Vec<_>>()
                .join(", ");
            let source = format!("aero_export OML: {}", names);
            return load_sdf_from_evaluated(
                path,
                sdf,
                source,
                part_bounds,
                eval_start.elapsed().as_millis(),
                grid_resolution,
                preview_cell_mm,
                smooth_normals,
                grid_only_preview,
                total_start,
            );
        }
    };
    let eval_ms = eval_start.elapsed().as_millis();

    load_sdf_from_evaluated(
        path,
        sdf,
        source,
        None,
        eval_ms,
        grid_resolution,
        preview_cell_mm,
        smooth_normals,
        grid_only_preview,
        total_start,
    )
}

fn with_preview_fallbacks(script: String) -> String {
    if script.contains("faired_inlet_cap(") && !script.contains("fn faired_inlet_cap") {
        format!(
            r#"
fn faired_inlet_cap(center, normal, span, flow, length, width, height, embed_depth) {{
    let cap = box_(length + embed_depth, width, height);
    let shift = length * 0.35 - embed_depth * 0.5;
    translate(
        cap,
        center[0] + flow[0] * shift,
        center[1] + flow[1] * shift,
        center[2] + flow[2] * shift
    )
}}

{script}
"#
        )
    } else {
        script
    }
}

fn load_sdf_from_evaluated(
    path: &Path,
    sdf: Arc<dyn Sdf>,
    source: String,
    part_bounds: Option<(Vec3, Vec3)>,
    eval_ms: u128,
    grid_resolution: u32,
    preview_cell_mm: f32,
    smooth_normals: bool,
    grid_only_preview: bool,
    total_start: Instant,
) -> Result<LoadSuccess, String> {
    let bounds_start = Instant::now();
    let (bounds_min, bounds_max, bounds_source) = if let Some((min, max)) = part_bounds {
        (min, max, "aero_export part bounds".to_string())
    } else if let Some((result_path, result_json)) = find_optimizer_result_for_script(path) {
        if let Some((min, max)) = result_bounds(&result_json) {
            (
                min,
                max,
                format!("optimizer result {}", display_name(&result_path)),
            )
        } else {
            let bounds = implicit_cad::pipeline::auto_bounds(sdf.as_ref());
            (bounds.0, bounds.1, "sampled SDF auto_bounds".to_string())
        }
    } else {
        let bounds = implicit_cad::pipeline::auto_bounds(sdf.as_ref());
        (bounds.0, bounds.1, "sampled SDF auto_bounds".to_string())
    };
    let bounds_ms = bounds_start.elapsed().as_millis();

    let grid_start = Instant::now();
    let grid = Arc::new(implicit_cad::pipeline::compute_sdf_grid(
        sdf.as_ref(),
        bounds_min,
        bounds_max,
        grid_resolution,
    ));
    let grid_ms = grid_start.elapsed().as_millis();

    let mesh_start = Instant::now();
    let mesh = if grid_only_preview {
        None
    } else {
        Some(Arc::new(
            implicit_cad::export::build_export_mesh_with_target_cell(
                sdf.as_ref(),
                bounds_min,
                bounds_max,
                preview_cell_mm,
                MeshQuality::Draft,
                smooth_normals,
            ),
        ))
    };
    let mesh_ms = if mesh.is_some() {
        mesh_start.elapsed().as_millis()
    } else {
        0
    };
    let triangle_count = mesh
        .as_ref()
        .map(|mesh| mesh.indices.len() / 3)
        .unwrap_or(0);

    let stats = LoadStats {
        source,
        artifact_path: None,
        bounds_source,
        total_ms: total_start.elapsed().as_millis(),
        eval_ms,
        bounds_ms,
        grid_ms,
        mesh_ms,
        grid_resolution,
        triangle_count,
        bounds_min,
        bounds_max,
    };

    Ok(LoadSuccess {
        path: path.to_path_buf(),
        grid: Some(grid),
        mesh,
        stats,
    })
}

fn mesh_from_triangle_mesh(tri_mesh: &implicit_cad::mesh::TriangleMesh) -> (Mesh, Vec3, Vec3) {
    let mut vertices = Vec::with_capacity(tri_mesh.triangles.len() * 3);
    let mut indices = Vec::with_capacity(tri_mesh.triangles.len() * 3);

    for (tri_idx, &[a, b, c]) in tri_mesh.triangles.iter().enumerate() {
        let normal = tri_mesh
            .normals
            .get(tri_idx)
            .copied()
            .unwrap_or(Vec3::Y)
            .to_array();
        for source_index in [a, b, c] {
            let position = tri_mesh.vertices[source_index as usize].to_array();
            indices.push(vertices.len() as u32);
            vertices.push(Vertex { position, normal });
        }
    }

    (
        Mesh { vertices, indices },
        tri_mesh.bounds_min,
        tri_mesh.bounds_max,
    )
}

fn combined_part_bounds(parts: &[scripting::AeroExportPart]) -> Option<(Vec3, Vec3)> {
    let mut min = Vec3::splat(f32::INFINITY);
    let mut max = Vec3::splat(f32::NEG_INFINITY);
    let mut found = false;

    for part in parts {
        let Some(part_min) = part.bounds_min_mm else {
            return None;
        };
        let Some(part_max) = part.bounds_max_mm else {
            return None;
        };
        min = min.min(Vec3::from_array(part_min));
        max = max.max(Vec3::from_array(part_max));
        found = true;
    }

    found.then_some((min, max))
}

fn read_json(path: &Path) -> Result<Value, String> {
    let text = fs::read_to_string(path)
        .map_err(|e| format!("Failed to read {}: {}", path.display(), e))?;
    serde_json::from_str(&text).map_err(|e| format!("Failed to parse {}: {}", path.display(), e))
}

fn find_optimizer_result_for_script(script_path: &Path) -> Option<(PathBuf, Value)> {
    let metadata = read_sibling_metadata(script_path);
    let feature = metadata
        .as_ref()
        .and_then(|meta| meta.get("feature"))
        .and_then(Value::as_str)
        .map(str::to_owned);
    let generated_script = metadata
        .as_ref()
        .and_then(|meta| meta.get("generated_script"))
        .and_then(Value::as_str)
        .map(str::to_owned);

    let experiment_dir = script_path.parent()?.parent()?;
    let logs_dir = experiment_dir.join("logs");
    let entries = fs::read_dir(logs_dir).ok()?;
    let script_key = normalized_path_key(script_path);
    let script_name = script_path
        .file_name()
        .and_then(|s| s.to_str())
        .unwrap_or("");

    let mut fallback: Option<(PathBuf, Value, i32)> = None;
    for entry in entries.flatten() {
        let path = entry.path();
        if path.extension().and_then(|s| s.to_str()) != Some("json") {
            continue;
        }
        let file_name = path.file_name().and_then(|s| s.to_str()).unwrap_or("");
        if !file_name.ends_with("_optimizer_export_result.json") {
            continue;
        }
        let Ok(json) = read_json(&path) else {
            continue;
        };

        let mut score = 0;
        if result_script_paths(&json)
            .iter()
            .any(|candidate| normalized_path_key(Path::new(candidate)) == script_key)
        {
            score = 100;
        } else if result_script_paths(&json)
            .iter()
            .any(|candidate| candidate.ends_with(script_name))
        {
            score = 90;
        } else if let Some(generated_script) = &generated_script {
            if result_script_paths(&json)
                .iter()
                .any(|candidate| candidate.ends_with(generated_script))
            {
                score = 80;
            }
        }

        if score == 0 {
            if let Some(feature) = &feature {
                let feature_key = feature
                    .strip_prefix("aircraft_oml_")
                    .unwrap_or(feature)
                    .replace("_faired_cap_native_frame", "");
                if file_name.contains(&feature_key) {
                    score = 50;
                }
            }
        }

        if score >= 90 {
            return Some((path, json));
        }
        if score > fallback.as_ref().map(|(_, _, s)| *s).unwrap_or(0) {
            fallback = Some((path, json, score));
        }
    }

    fallback.map(|(path, json, _)| (path, json))
}

fn read_sibling_metadata(script_path: &Path) -> Option<Value> {
    let metadata_path = PathBuf::from(format!("{}.metadata.json", script_path.display()));
    read_json(&metadata_path).ok()
}

fn result_script_paths(json: &Value) -> Vec<String> {
    let mut paths = Vec::new();
    for pointer in [
        "/summary/sdf_script_source",
        "/preset_definition/sdf_script_source",
        "/summary/source_script",
    ] {
        if let Some(path) = json.pointer(pointer).and_then(Value::as_str) {
            paths.push(path.to_string());
        }
    }
    paths
}

fn result_stl_path(json: &Value, result_path: &Path) -> Option<PathBuf> {
    let stl = json.pointer("/summary/stl_path")?.as_str()?;
    let path = PathBuf::from(stl);
    if path.is_absolute() {
        Some(path)
    } else {
        Some(result_path.parent()?.join(path))
    }
}

fn result_bounds(json: &Value) -> Option<(Vec3, Vec3)> {
    let min = json_array3(json.pointer("/summary/bbox_min")?)?;
    let max = json_array3(json.pointer("/summary/bbox_max")?)?;
    Some((Vec3::from_array(min), Vec3::from_array(max)))
}

fn json_array3(value: &Value) -> Option<[f32; 3]> {
    let array = value.as_array()?;
    if array.len() != 3 {
        return None;
    }
    Some([
        array[0].as_f64()? as f32,
        array[1].as_f64()? as f32,
        array[2].as_f64()? as f32,
    ])
}

fn normalized_path_key(path: &Path) -> String {
    path.canonicalize()
        .unwrap_or_else(|_| path.to_path_buf())
        .to_string_lossy()
        .replace('\\', "/")
        .to_ascii_lowercase()
}

fn discover_optimizer_dir() -> Option<PathBuf> {
    let current_dir = std::env::current_dir().ok()?;
    for ancestor in current_dir.ancestors() {
        let candidate = ancestor
            .join("dual_contouring")
            .join("direct_sparse_sdf_mc_experiment");
        if candidate.join("logs").is_dir() {
            return Some(candidate);
        }
        let candidate = ancestor.join("direct_sparse_sdf_mc_experiment");
        if candidate.join("logs").is_dir() {
            return Some(candidate);
        }
    }
    None
}

fn discover_optimizer_models(experiment_dir: &Path) -> Vec<ModelEntry> {
    let logs_dir = experiment_dir.join("logs");
    let Ok(entries) = fs::read_dir(logs_dir) else {
        return Vec::new();
    };

    let mut models = Vec::new();
    for entry in entries.flatten() {
        let result_path = entry.path();
        let file_name = result_path
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("");
        if result_path.extension().and_then(|s| s.to_str()) != Some("json")
            || !file_name.ends_with("_optimizer_export_result.json")
        {
            continue;
        }
        let Ok(json) = read_json(&result_path) else {
            continue;
        };
        let label = clean_model_label(file_name);
        let script_path = result_script_paths(&json)
            .into_iter()
            .next()
            .map(PathBuf::from);
        let stl_path = result_stl_path(&json, &result_path);
        let status = json
            .get("status")
            .and_then(Value::as_str)
            .unwrap_or("unknown")
            .to_string();
        let triangle_count = json
            .pointer("/summary/triangle_count")
            .and_then(Value::as_u64)
            .map(|value| value as usize);

        models.push(ModelEntry {
            label,
            result_path,
            script_path,
            stl_path,
            status,
            triangle_count,
        });
    }

    models.sort_by(|a, b| {
        a.label
            .to_ascii_lowercase()
            .cmp(&b.label.to_ascii_lowercase())
    });
    models
}

fn clean_model_label(file_name: &str) -> String {
    file_name
        .strip_suffix("_optimizer_export_result.json")
        .unwrap_or(file_name)
        .replace('_', " ")
}

fn short_model_label(label: &str) -> String {
    const MAX_LEN: usize = 42;
    if label.len() <= MAX_LEN {
        label.to_string()
    } else {
        format!("{}...", &label[..MAX_LEN])
    }
}

fn format_count(value: usize) -> String {
    let text = value.to_string();
    let mut out = String::new();
    for (idx, ch) in text.chars().rev().enumerate() {
        if idx > 0 && idx % 3 == 0 {
            out.push(',');
        }
        out.push(ch);
    }
    out.chars().rev().collect()
}

fn union_sdfs(mut sdfs: Vec<Arc<dyn Sdf>>) -> Option<Arc<dyn Sdf>> {
    let first = sdfs.pop()?;
    Some(sdfs.into_iter().fold(first, |acc, sdf| {
        Arc::new(Union::new(acc, sdf)) as Arc<dyn Sdf>
    }))
}

fn display_name(path: &Path) -> String {
    path.file_name()
        .and_then(|s| s.to_str())
        .map(|s| s.to_string())
        .unwrap_or_else(|| path.display().to_string())
}

#[allow(dead_code)]
fn main() -> eframe::Result<()> {
    let mut args = std::env::args_os().skip(1);
    let first_arg = args.next();
    if first_arg.as_deref() == Some(std::ffi::OsStr::new("--bench-load"))
        || first_arg.as_deref() == Some(std::ffi::OsStr::new("--bench-load-live-sdf"))
    {
        let live_sdf = first_arg.as_deref() == Some(std::ffi::OsStr::new("--bench-load-live-sdf"));
        let Some(path) = args.next().map(PathBuf::from) else {
            eprintln!("Usage: sdf_viewer --bench-load <rhai|json|stl>");
            eprintln!("   or: sdf_viewer --bench-load-live-sdf <rhai>");
            std::process::exit(2);
        };
        let result = if live_sdf {
            load_preview_with_options(&path, 48, 10.0, false, true, true)
        } else {
            load_preview(&path, 96, 5.0, false)
        };
        match result {
            Ok(success) => {
                println!("path={}", path.display());
                println!("source={}", success.stats.source);
                println!(
                    "artifact={}",
                    success
                        .stats
                        .artifact_path
                        .as_ref()
                        .map(|p| p.display().to_string())
                        .unwrap_or_else(|| "none".to_string())
                );
                println!("bounds_source={}", success.stats.bounds_source);
                println!("total_ms={}", success.stats.total_ms);
                println!("eval_ms={}", success.stats.eval_ms);
                println!("bounds_ms={}", success.stats.bounds_ms);
                println!("grid_ms={}", success.stats.grid_ms);
                println!("mesh_ms={}", success.stats.mesh_ms);
                println!("triangles={}", success.stats.triangle_count);
            }
            Err(message) => {
                eprintln!("{}", message);
                std::process::exit(1);
            }
        }
        return Ok(());
    }

    if first_arg.as_deref() == Some(std::ffi::OsStr::new("--watch-target")) {
        let Some(watch_file) = args.next().map(PathBuf::from) else {
            eprintln!("Usage: sdf_viewer --watch-target <target-file>");
            std::process::exit(2);
        };
        let initial_path = fs::read_to_string(&watch_file)
            .ok()
            .map(|text| text.trim().to_string())
            .filter(|text| !text.is_empty())
            .map(PathBuf::from);
        let options = eframe::NativeOptions {
            viewport: egui::ViewportBuilder::default()
                .with_title("Fast SDF Viewer")
                .with_inner_size([1400.0, 900.0]),
            renderer: eframe::Renderer::Wgpu,
            ..Default::default()
        };
        return eframe::run_native(
            "Fast SDF Viewer",
            options,
            Box::new(move |_cc| {
                Ok(Box::new(
                    ViewerApp::new(initial_path)
                        .quick_sdf_preview()
                        .with_watch_target_file(watch_file),
                ))
            }),
        );
    }

    let initial_path = first_arg.map(PathBuf::from);
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1200.0, 800.0]),
        renderer: eframe::Renderer::Wgpu,
        ..Default::default()
    };

    eframe::run_native(
        "Fast SDF Viewer",
        options,
        Box::new(move |_cc| Ok(Box::new(ViewerApp::new(initial_path)))),
    )
}
