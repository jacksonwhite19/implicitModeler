use eframe::egui;
use rusqlite::{Connection, Row};
use serde_json::Value;
use serde_json::json;
use std::cmp::Ordering;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Child, Command};
use std::time::{Duration, Instant};

#[derive(Clone, Default)]
struct CandidateRow {
    candidate_id: String,
    variant_id: String,
    campaign_id: String,
    generation: i64,
    inferred_generation: String,
    status: String,
    evaluation_id: Option<String>,
    evaluation_status: Option<String>,
    notes: Option<String>,
    design_variables: Value,
    rough_score: Option<f64>,
    best_ld: Option<f64>,
    best_alpha: Option<f64>,
    cl: Option<f64>,
    cd: Option<f64>,
    cm: Option<f64>,
    cl_alpha_per_deg: Option<f64>,
    cm_alpha_per_deg: Option<f64>,
    alpha_points: Vec<AlphaPoint>,
    score_components: Vec<NamedValue>,
    detractors: Vec<NamedValue>,
    cells: Option<f64>,
    aircraft_faces: Option<f64>,
    max_non_ortho: Option<f64>,
    max_skewness: Option<f64>,
    yplus_p50: Option<f64>,
    yplus_p95: Option<f64>,
    yplus_p99: Option<f64>,
    yplus_max: Option<f64>,
    failure_count: usize,
    viewer_target: Option<PathBuf>,
    lineage: Vec<LineageRow>,
    module_attempts: Vec<ModuleAttemptRow>,
    failures: Vec<FailureRow>,
    runner_state: Option<RunnerStateRow>,
}

#[derive(Clone, Default)]
struct AlphaPoint {
    alpha_deg: f64,
    cl: Option<f64>,
    cd: Option<f64>,
    cm: Option<f64>,
    ld: Option<f64>,
    case_path: Option<String>,
}

#[derive(Clone, Default)]
struct NamedValue {
    name: String,
    value: f64,
}

#[derive(Clone, Default)]
struct LineageRow {
    parent_ids: Vec<String>,
    operator: String,
    reason: String,
    mutation_summary: Value,
}

#[derive(Clone, Default)]
struct ModuleAttemptRow {
    module_name: String,
    status: String,
    runtime_seconds: Option<f64>,
    metrics: Value,
    metadata: Value,
    warnings: Value,
}

#[derive(Clone, Default)]
struct FailureRow {
    category: String,
    stage: String,
    severity: String,
    message: String,
}

#[derive(Clone, Default)]
struct RunnerStateRow {
    state: String,
    stage: String,
    active_module: Option<String>,
    progress: Option<f64>,
    priority: Option<i64>,
    updated_at: String,
    reason: Option<String>,
    message: Option<String>,
    metadata: Value,
}

#[derive(Clone, Default)]
struct OptimizerRunRow {
    optimizer_run_id: String,
    optimizer_name: String,
    status: String,
    objective: Value,
    settings: Value,
    summary: Value,
    created_at: String,
    started_at: Option<String>,
    finished_at: Option<String>,
}

#[derive(Clone, Default)]
struct OptimizerIterationRow {
    optimizer_iteration_id: String,
    optimizer_run_id: String,
    iteration_index: i64,
    status: String,
    proposed_candidate_ids: Vec<String>,
    parent_candidate_ids: Vec<String>,
    strategy: String,
    rationale: String,
    optimizer_state: Value,
    created_at: String,
    completed_at: Option<String>,
}

#[derive(Default)]
struct DashboardData {
    campaigns: Vec<String>,
    optimizer_runs: Vec<OptimizerRunRow>,
    optimizer_iterations: Vec<OptimizerIterationRow>,
    candidates: Vec<CandidateRow>,
    load_error: Option<String>,
    loaded_at: Option<Instant>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum DashboardTab {
    Overview,
    Candidate,
    Scoring,
    Curves,
    Lineage,
    Controls,
}

struct DashboardApp {
    repo_root: PathBuf,
    workspace: PathBuf,
    db_path: PathBuf,
    viewer_exe: PathBuf,
    viewer_watch_file: PathBuf,
    viewer_child: Option<Child>,
    data: DashboardData,
    selected_candidate_id: Option<String>,
    viewer_auto_sync: bool,
    filter: String,
    active_tab: DashboardTab,
    auto_refresh: bool,
    status: String,
    last_auto_refresh: Instant,
}

impl DashboardApp {
    fn new(workspace: PathBuf) -> Self {
        let repo_root = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
        let db_path = if workspace.extension().and_then(|s| s.to_str()) == Some("db") {
            workspace.clone()
        } else {
            workspace.join("optimizer.db")
        };
        let workspace_dir = db_path
            .parent()
            .map(Path::to_path_buf)
            .unwrap_or_else(|| workspace.clone());
        let mut app = Self {
            repo_root,
            workspace: workspace_dir,
            db_path,
            viewer_exe: PathBuf::from("target")
                .join("release")
                .join("sdf_viewer.exe"),
            viewer_watch_file: PathBuf::new(),
            viewer_child: None,
            data: DashboardData::default(),
            selected_candidate_id: None,
            viewer_auto_sync: true,
            filter: String::new(),
            active_tab: DashboardTab::Overview,
            auto_refresh: true,
            status: String::new(),
            last_auto_refresh: Instant::now(),
        };
        app.viewer_watch_file = app.workspace.join("dashboard_sdf_viewer_target.txt");
        app.refresh();
        app
    }

    fn refresh(&mut self) {
        self.data = load_dashboard_data(&self.workspace, &self.db_path, &self.repo_root);
        if let Some(error) = &self.data.load_error {
            self.status = format!("Load failed: {error}");
        } else {
            self.status = format!("Loaded {} candidates", self.data.candidates.len());
            if self.selected_candidate_id.is_none() {
                self.selected_candidate_id = self
                    .data
                    .candidates
                    .first()
                    .map(|candidate| candidate.candidate_id.clone());
            }
        }
        self.last_auto_refresh = Instant::now();
    }

    fn selected_candidate(&self) -> Option<&CandidateRow> {
        let id = self.selected_candidate_id.as_ref()?;
        self.data
            .candidates
            .iter()
            .find(|candidate| &candidate.candidate_id == id)
    }

    fn visible_candidates(&self) -> Vec<&CandidateRow> {
        let filter = self.filter.trim().to_ascii_lowercase();
        self.data
            .candidates
            .iter()
            .filter(|candidate| {
                filter.is_empty()
                    || candidate.variant_id.to_ascii_lowercase().contains(&filter)
                    || candidate
                        .candidate_id
                        .to_ascii_lowercase()
                        .contains(&filter)
                    || candidate.status.to_ascii_lowercase().contains(&filter)
                    || candidate.runner_state.as_ref().is_some_and(|state| {
                        state.state.to_ascii_lowercase().contains(&filter)
                            || state.stage.to_ascii_lowercase().contains(&filter)
                            || state
                                .active_module
                                .as_deref()
                                .unwrap_or("")
                                .to_ascii_lowercase()
                                .contains(&filter)
                    })
            })
            .collect()
    }

    fn select_candidate(&mut self, candidate_id: &str) {
        if self
            .selected_candidate_id
            .as_ref()
            .is_some_and(|current| current == candidate_id)
        {
            return;
        }
        self.selected_candidate_id = Some(candidate_id.to_string());
        if self.viewer_auto_sync {
            if let Some(candidate) = self.selected_candidate().cloned() {
                self.open_or_update_viewer(&candidate);
            }
        }
    }

    fn open_or_update_viewer(&mut self, candidate: &CandidateRow) {
        let Some(target) = &candidate.viewer_target else {
            self.status = "No SDF preview target found for selected candidate".to_string();
            return;
        };
        let viewer_exe = self.repo_root.join(&self.viewer_exe);
        if !viewer_exe.exists() {
            self.status = format!("SDF viewer executable not found: {}", viewer_exe.display());
            return;
        }

        if let Some(parent) = self.viewer_watch_file.parent() {
            if let Err(err) = fs::create_dir_all(parent) {
                self.status = format!("Failed to create viewer control directory: {err}");
                return;
            }
        }
        if let Err(err) = fs::write(&self.viewer_watch_file, target.display().to_string()) {
            self.status = format!("Failed to update SDF viewer target: {err}");
            return;
        }

        let viewer_running = match self.viewer_child.as_mut() {
            Some(child) => match child.try_wait() {
                Ok(Some(_)) => false,
                Ok(None) => true,
                Err(_) => false,
            },
            None => false,
        };

        if viewer_running {
            self.status = format!("Sent {} to running SDF viewer", candidate.variant_id);
            return;
        }

        match Command::new(&viewer_exe)
            .arg("--watch-target")
            .arg(&self.viewer_watch_file)
            .current_dir(&self.repo_root)
            .spawn()
        {
            Ok(child) => {
                self.status = format!(
                    "Opened SDF viewer session pid {} for {}",
                    child.id(),
                    candidate.variant_id
                );
                self.viewer_child = Some(child);
            }
            Err(err) => {
                self.status = format!("Failed to open SDF viewer: {err}");
            }
        }
    }
}

impl eframe::App for DashboardApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        apply_dashboard_style(ctx);
        if self.auto_refresh && self.last_auto_refresh.elapsed() > Duration::from_secs(10) {
            self.refresh();
        }

        egui::TopBottomPanel::top("top_bar").show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.heading("Aircraft Optimizer");
                if ui.button("Refresh").clicked() {
                    self.refresh();
                }
                ui.checkbox(&mut self.auto_refresh, "Auto");
                ui.checkbox(&mut self.viewer_auto_sync, "Sync Viewer");
                ui.separator();
                tab_button(ui, &mut self.active_tab, DashboardTab::Overview, "Overview");
                tab_button(
                    ui,
                    &mut self.active_tab,
                    DashboardTab::Candidate,
                    "Candidate",
                );
                tab_button(ui, &mut self.active_tab, DashboardTab::Scoring, "Scoring");
                tab_button(ui, &mut self.active_tab, DashboardTab::Curves, "Curves");
                tab_button(ui, &mut self.active_tab, DashboardTab::Lineage, "History");
                tab_button(ui, &mut self.active_tab, DashboardTab::Controls, "Controls");
                ui.separator();
                ui.label(&self.status);
            });
        });

        egui::SidePanel::left("candidate_list")
            .resizable(true)
            .default_width(620.0)
            .width_range(480.0..=900.0)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Filter");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.filter)
                            .desired_width(260.0)
                            .hint_text("candidate, status, id"),
                    );
                });
                ui.small(format!("DB: {}", self.db_path.display()));
                ui.separator();
                candidate_table(ui, self);
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            if let Some(error) = &self.data.load_error {
                ui.colored_label(egui::Color32::from_rgb(255, 130, 130), error);
                return;
            }
            if let Some(candidate) = self.selected_candidate().cloned() {
                match self.active_tab {
                    DashboardTab::Overview => overview_tab(ui, self),
                    DashboardTab::Candidate => {
                        ui.columns(2, |columns| {
                            candidate_detail(&mut columns[0], self, &candidate);
                            preview_panel(&mut columns[1], self, &candidate);
                        });
                    }
                    DashboardTab::Scoring => scoring_tab(ui, self),
                    DashboardTab::Curves => curves_tab(ui, &candidate),
                    DashboardTab::Lineage => lineage_tab(ui, self),
                    DashboardTab::Controls => controls_tab(ui, self),
                }
            } else {
                ui.centered_and_justified(|ui| {
                    ui.label("Select a candidate.");
                });
            }
        });

        if self.auto_refresh {
            ctx.request_repaint_after(Duration::from_secs(10));
        }
    }
}

fn apply_dashboard_style(ctx: &egui::Context) {
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

fn tab_button(ui: &mut egui::Ui, active: &mut DashboardTab, tab: DashboardTab, label: &str) {
    if ui.selectable_label(*active == tab, label).clicked() {
        *active = tab;
    }
}

fn candidate_table(ui: &mut egui::Ui, app: &mut DashboardApp) {
    let rows: Vec<CandidateRow> = app.visible_candidates().into_iter().cloned().collect();
    egui::ScrollArea::vertical().show(ui, |ui| {
        egui::Grid::new("candidate_grid")
            .num_columns(8)
            .striped(true)
            .min_col_width(52.0)
            .show(ui, |ui| {
                ui.strong("Candidate");
                ui.strong("Gen");
                ui.strong("Runner");
                ui.strong("Score");
                ui.strong("L/D");
                ui.strong("CL");
                ui.strong("CD");
                ui.strong("CM");
                ui.end_row();

                for candidate in rows {
                    let selected = app
                        .selected_candidate_id
                        .as_ref()
                        .is_some_and(|id| id == &candidate.candidate_id);
                    let label = if selected {
                        format!("> {}", candidate.variant_id)
                    } else {
                        candidate.variant_id.clone()
                    };
                    if ui.selectable_label(selected, label).clicked() {
                        app.select_candidate(&candidate.candidate_id);
                    }
                    ui.label(&candidate.inferred_generation);
                    runner_status_label(ui, &candidate);
                    ui.label(fmt(candidate.rough_score));
                    ui.label(fmt(candidate.best_ld));
                    ui.label(fmt(candidate.cl));
                    ui.label(fmt(candidate.cd));
                    ui.label(fmt(candidate.cm));
                    ui.end_row();
                }
            });
    });
}

fn candidate_detail(ui: &mut egui::Ui, app: &mut DashboardApp, candidate: &CandidateRow) {
    ui.horizontal_wrapped(|ui| {
        ui.heading(&candidate.variant_id);
        ui.label(&candidate.candidate_id);
    });
    if let Some(notes) = &candidate.notes {
        ui.small(notes);
    }
    if let Some(state) = &candidate.runner_state {
        ui.horizontal_wrapped(|ui| {
            ui.strong("Runner:");
            runner_status_label(ui, candidate);
            if let Some(active_module) = &state.active_module {
                ui.small(format!("module: {active_module}"));
            }
            if let Some(progress) = state.progress {
                ui.small(format!("progress: {:.0}%", progress * 100.0));
            }
            if let Some(priority) = state.priority {
                ui.small(format!("priority: {priority}"));
            }
            ui.small(format!("updated: {}", state.updated_at));
        });
        if let Some(message) = &state.message {
            ui.small(message);
        }
        if !state.metadata.is_null() {
            ui.collapsing("runner metadata", |ui| json_view(ui, &state.metadata));
        }
    }

    ui.horizontal_wrapped(|ui| {
        metric_box(ui, "DB gen", candidate.generation.to_string());
        metric_box(ui, "Score", fmt(candidate.rough_score));
        metric_box(ui, "L/D", fmt(candidate.best_ld));
        metric_box(ui, "CL", fmt(candidate.cl));
        metric_box(ui, "CD", fmt(candidate.cd));
        metric_box(ui, "CM", fmt(candidate.cm));
        metric_box(ui, "Alpha", fmt(candidate.best_alpha));
    });
    ui.horizontal_wrapped(|ui| {
        ui.small(format!("Campaign: {}", candidate.campaign_id));
        if let Some(evaluation_id) = &candidate.evaluation_id {
            ui.small(format!("Evaluation: {evaluation_id}"));
        }
        if let Some(status) = &candidate.evaluation_status {
            ui.small(format!("Evaluation status: {status}"));
        }
    });

    ui.horizontal_wrapped(|ui| {
        metric_box(ui, "Cells", fmt0(candidate.cells));
        metric_box(ui, "Aircraft faces", fmt0(candidate.aircraft_faces));
        metric_box(ui, "Max non-ortho", fmt(candidate.max_non_ortho));
        metric_box(ui, "Max skew", fmt(candidate.max_skewness));
        metric_box(ui, "y+ p95", fmt(candidate.yplus_p95));
        metric_box(ui, "y+ max", fmt(candidate.yplus_max));
    });

    ui.horizontal_wrapped(|ui| {
        ui.strong("SDF preview target:");
        if let Some(path) = &candidate.viewer_target {
            ui.small(path.display().to_string());
        } else {
            ui.small("No viewer target found.");
        }
    });
    if ui
        .add_enabled(
            candidate.viewer_target.is_some(),
            egui::Button::new("Open / Update SDF Viewer"),
        )
        .clicked()
    {
        app.open_or_update_viewer(candidate);
    }

    ui.separator();
    candidate_curve_strip(ui, candidate);
    ui.separator();
    egui::ScrollArea::vertical().show(ui, |ui| {
        ui.collapsing("Generation / Lineage", |ui| {
            if candidate.lineage.is_empty() {
                ui.label("No lineage records.");
            }
            for lineage in &candidate.lineage {
                ui.label(format!("Operator: {}", lineage.operator));
                ui.label(format!("Reason: {}", lineage.reason));
                ui.label(format!("Parents: {}", lineage.parent_ids.join(", ")));
                json_view(ui, &lineage.mutation_summary);
                ui.separator();
            }
        });

        ui.collapsing("Measured Data", |ui| {
            measured_table(ui, candidate);
        });

        ui.collapsing("Alpha Sweep Points", |ui| {
            alpha_point_table(ui, candidate);
        });

        ui.collapsing("Score Components", |ui| {
            named_value_table(ui, "component", &candidate.score_components);
            ui.separator();
            named_value_table(ui, "detractor", &candidate.detractors);
        });

        ui.collapsing("Design Variables", |ui| {
            json_view(ui, &candidate.design_variables);
        });

        ui.collapsing("Module Attempts", |ui| {
            for attempt in &candidate.module_attempts {
                ui.horizontal_wrapped(|ui| {
                    ui.strong(&attempt.module_name);
                    status_label(ui, &attempt.status, 0);
                    if let Some(runtime) = attempt.runtime_seconds {
                        ui.small(format!("{} s", fmt(Some(runtime))));
                    }
                });
                if attempt.warnings != Value::Null {
                    ui.small(format!("warnings: {}", attempt.warnings));
                }
                ui.collapsing("metrics", |ui| json_view(ui, &attempt.metrics));
                ui.collapsing("metadata", |ui| json_view(ui, &attempt.metadata));
                ui.separator();
            }
        });

        ui.collapsing("Failures", |ui| {
            if candidate.failures.is_empty() {
                ui.label("No failures.");
            }
            for failure in &candidate.failures {
                ui.colored_label(
                    egui::Color32::from_rgb(255, 150, 120),
                    format!(
                        "{} / {} / {}",
                        failure.category, failure.stage, failure.severity
                    ),
                );
                ui.label(&failure.message);
                ui.separator();
            }
        });
    });
}

fn preview_panel(ui: &mut egui::Ui, app: &mut DashboardApp, candidate: &CandidateRow) {
    ui.heading("SDF Preview");
    ui.label("Fast preview uses a linked standalone SDF viewer session.");
    ui.small("When Sync Viewer is enabled, selecting a candidate updates the running viewer.");
    ui.separator();
    if ui
        .add_enabled(
            candidate.viewer_target.is_some(),
            egui::Button::new("Open / Update SDF Viewer"),
        )
        .clicked()
    {
        app.open_or_update_viewer(candidate);
    }
    ui.separator();
    ui.strong("Target");
    if let Some(path) = &candidate.viewer_target {
        ui.monospace(path.display().to_string());
    } else {
        ui.label("No Rhai/SDF preview target found.");
    }
    ui.separator();
    candidate_curve_strip(ui, candidate);
}

fn candidate_curve_strip(ui: &mut egui::Ui, candidate: &CandidateRow) {
    if candidate.alpha_points.is_empty() {
        ui.label("No alpha sweep plots recorded for this candidate.");
        return;
    }
    ui.heading("Generated Aero Curves");
    ui.columns(2, |columns| {
        line_plot(
            &mut columns[0],
            "L/D vs alpha",
            &candidate.alpha_points,
            CurveMetric::Ld,
        );
        line_plot(
            &mut columns[1],
            "CL vs alpha",
            &candidate.alpha_points,
            CurveMetric::Cl,
        );
    });
    ui.columns(2, |columns| {
        line_plot(
            &mut columns[0],
            "CD vs alpha",
            &candidate.alpha_points,
            CurveMetric::Cd,
        );
        line_plot(
            &mut columns[1],
            "CM vs alpha",
            &candidate.alpha_points,
            CurveMetric::Cm,
        );
    });
}

fn overview_tab(ui: &mut egui::Ui, app: &mut DashboardApp) {
    ui.heading("Campaign Overview");
    ui.horizontal_wrapped(|ui| {
        metric_box(ui, "Candidates", app.data.candidates.len().to_string());
        metric_box(
            ui,
            "Evaluated",
            app.data
                .candidates
                .iter()
                .filter(|c| c.status == "evaluated")
                .count()
                .to_string(),
        );
        metric_box(
            ui,
            "Failures",
            app.data
                .candidates
                .iter()
                .map(|c| c.failure_count)
                .sum::<usize>()
                .to_string(),
        );
        metric_box(
            ui,
            "Best score",
            fmt(app
                .data
                .candidates
                .iter()
                .filter_map(|c| c.rough_score)
                .reduce(f64::max)),
        );
        metric_box(
            ui,
            "Best L/D",
            fmt(app
                .data
                .candidates
                .iter()
                .filter_map(|c| c.best_ld)
                .reduce(f64::max)),
        );
    });

    ui.separator();
    ui.columns(2, |columns| {
        columns[0].heading("Score vs L/D");
        scatter_plot(
            &mut columns[0],
            &app.data.candidates,
            PlotMetric::BestLd,
            PlotMetric::Score,
            "L/D",
            "Score",
        );
        columns[1].heading("Drag vs Lift");
        scatter_plot(
            &mut columns[1],
            &app.data.candidates,
            PlotMetric::Cd,
            PlotMetric::Cl,
            "CD",
            "CL",
        );
    });

    ui.separator();
    ui.heading("Best Results");
    ranked_table(ui, &app.data.candidates, 8);
    ui.separator();
    next_up_panel(ui, app);
}

fn scoring_tab(ui: &mut egui::Ui, app: &mut DashboardApp) {
    ui.heading("Scoring");
    ui.columns(2, |columns| {
        columns[0].heading("Score vs Moment");
        scatter_plot(
            &mut columns[0],
            &app.data.candidates,
            PlotMetric::Cm,
            PlotMetric::Score,
            "CM",
            "Score",
        );
        columns[1].heading("Score vs Mesh Quality");
        scatter_plot(
            &mut columns[1],
            &app.data.candidates,
            PlotMetric::MaxNonOrtho,
            PlotMetric::Score,
            "Max non-ortho",
            "Score",
        );
    });

    ui.separator();
    if let Some(candidate) = app.selected_candidate() {
        ui.heading(format!(
            "Selected scoring breakdown: {}",
            candidate.variant_id
        ));
        ui.columns(2, |columns| {
            named_bar_chart(
                &mut columns[0],
                "Components",
                &candidate.score_components,
                false,
            );
            named_bar_chart(&mut columns[1], "Detractors", &candidate.detractors, true);
        });
    }
}

fn curves_tab(ui: &mut egui::Ui, candidate: &CandidateRow) {
    ui.heading(format!("Alpha Sweep Curves: {}", candidate.variant_id));
    if candidate.alpha_points.is_empty() {
        ui.label("No alpha sweep points recorded for this candidate.");
        return;
    }

    ui.columns(2, |columns| {
        line_plot(
            &mut columns[0],
            "CL vs alpha",
            &candidate.alpha_points,
            CurveMetric::Cl,
        );
        line_plot(
            &mut columns[1],
            "CD vs alpha",
            &candidate.alpha_points,
            CurveMetric::Cd,
        );
    });
    ui.columns(2, |columns| {
        line_plot(
            &mut columns[0],
            "L/D vs alpha",
            &candidate.alpha_points,
            CurveMetric::Ld,
        );
        line_plot(
            &mut columns[1],
            "CM vs alpha",
            &candidate.alpha_points,
            CurveMetric::Cm,
        );
    });
    ui.separator();
    alpha_point_table(ui, candidate);
}

fn lineage_tab(ui: &mut egui::Ui, app: &mut DashboardApp) {
    ui.horizontal_wrapped(|ui| {
        ui.heading("Run History");
        ui.small("Candidate state, parentage, score, and pending optimizer proposals.");
    });
    run_history_summary(ui, app);
    ui.separator();
    run_history_table(ui, app);
    ui.separator();
    ui.collapsing("Lineage Map", |ui| {
        ui.small(
            "Rows are grouped by recorded generation. Lines show parent links when available.",
        );
        generation_tree(ui, app);
    });
    ui.separator();
    next_up_panel(ui, app);
}

fn run_history_summary(ui: &mut egui::Ui, app: &DashboardApp) {
    let ran = app
        .data
        .candidates
        .iter()
        .filter(|candidate| {
            matches!(
                candidate_history_state(candidate).as_str(),
                "ran" | "scored" | "promoted" | "rejected"
            )
        })
        .count();
    let running = app
        .data
        .candidates
        .iter()
        .filter(|candidate| {
            matches!(
                candidate_history_state(candidate).as_str(),
                "running" | "meshing" | "cfd_running"
            )
        })
        .count();
    let failed = app
        .data
        .candidates
        .iter()
        .filter(|candidate| candidate_history_state(candidate) == "failed")
        .count();
    let pending_candidates = app
        .data
        .candidates
        .iter()
        .filter(|candidate| {
            matches!(
                candidate_history_state(candidate).as_str(),
                "pending" | "queued"
            )
        })
        .count();
    let pending_iterations = app
        .data
        .optimizer_iterations
        .iter()
        .filter(|iteration| iteration.status != "complete")
        .count();

    ui.horizontal_wrapped(|ui| {
        metric_box(ui, "Ran", ran.to_string());
        metric_box(ui, "Running", running.to_string());
        metric_box(ui, "Pending candidates", pending_candidates.to_string());
        metric_box(ui, "Pending iterations", pending_iterations.to_string());
        metric_box(ui, "Failed", failed.to_string());
    });
}

fn run_history_table(ui: &mut egui::Ui, app: &mut DashboardApp) {
    let mut rows = app.data.candidates.clone();
    rows.sort_by(|a, b| {
        a.generation
            .cmp(&b.generation)
            .then_with(|| candidate_history_rank(a).cmp(&candidate_history_rank(b)))
            .then_with(|| {
                b.rough_score
                    .partial_cmp(&a.rough_score)
                    .unwrap_or(Ordering::Equal)
            })
            .then_with(|| a.variant_id.cmp(&b.variant_id))
    });

    egui::ScrollArea::vertical()
        .max_height(380.0)
        .show(ui, |ui| {
            egui::Grid::new("run_history_grid")
                .num_columns(11)
                .striped(true)
                .min_col_width(58.0)
                .show(ui, |ui| {
                    ui.strong("State");
                    ui.strong("Stage");
                    ui.strong("Gen");
                    ui.strong("Candidate");
                    ui.strong("Parents");
                    ui.strong("Score");
                    ui.strong("L/D");
                    ui.strong("Alpha");
                    ui.strong("Mesh");
                    ui.strong("y+ p95");
                    ui.strong("Notes");
                    ui.end_row();

                    for candidate in rows {
                        let selected = app
                            .selected_candidate_id
                            .as_ref()
                            .is_some_and(|id| id == &candidate.candidate_id);
                        history_state_label(ui, &candidate);
                        ui.label(candidate_runner_stage(&candidate));
                        ui.label(&candidate.inferred_generation);
                        if ui
                            .selectable_label(selected, short_label(&candidate.variant_id))
                            .on_hover_text(&candidate.candidate_id)
                            .clicked()
                        {
                            app.select_candidate(&candidate.candidate_id);
                        }
                        ui.small(parent_summary(&candidate, &app.data.candidates));
                        ui.label(fmt(candidate.rough_score));
                        ui.label(fmt(candidate.best_ld));
                        ui.label(fmt(candidate.best_alpha));
                        ui.label(format!(
                            "{} / {}",
                            fmt0(candidate.cells),
                            fmt(candidate.max_non_ortho)
                        ));
                        ui.label(fmt(candidate.yplus_p95));
                        ui.small(history_note(&candidate));
                        ui.end_row();
                    }
                });
        });
}

fn controls_tab(ui: &mut egui::Ui, app: &mut DashboardApp) {
    ui.heading("Run Controls");
    ui.label("These controls create traceable request files in the workspace. A runner/controller can consume them later.");
    ui.separator();

    ui.horizontal_wrapped(|ui| {
        if ui.button("Request Start").clicked() {
            app.write_control_request("start_campaign", None);
        }
        if ui.button("Request Pause").clicked() {
            app.write_control_request("pause_campaign", None);
        }
        if ui.button("Request Stop").clicked() {
            app.write_control_request("stop_campaign", None);
        }
        if ui.button("Request Generate Next").clicked() {
            app.write_control_request("generate_next_candidates", None);
        }
    });

    if let Some(candidate) = app.selected_candidate().cloned() {
        ui.horizontal_wrapped(|ui| {
            ui.strong("Selected:");
            ui.label(&candidate.variant_id);
            if ui.button("Promote").clicked() {
                app.write_control_request("promote_candidate", Some(&candidate));
            }
            if ui.button("Rerun Rough").clicked() {
                app.write_control_request("rerun_rough_scoring", Some(&candidate));
            }
            if ui.button("Rerun Final").clicked() {
                app.write_control_request("rerun_final_scoring", Some(&candidate));
            }
        });
    }

    ui.separator();
    next_up_panel(ui, app);
    ui.separator();
    optimizer_runs_panel(ui, app);
}

impl DashboardApp {
    fn write_control_request(&mut self, action: &str, candidate: Option<&CandidateRow>) {
        let dir = self.workspace.join("dashboard_control_requests");
        if let Err(err) = fs::create_dir_all(&dir) {
            self.status = format!("Failed to create control request directory: {err}");
            return;
        }
        let timestamp = chrono::Utc::now().format("%Y%m%dT%H%M%SZ").to_string();
        let path = dir.join(format!("{timestamp}_{action}.json"));
        let payload = json!({
            "action": action,
            "created_at": chrono::Utc::now().to_rfc3339(),
            "workspace": self.workspace,
            "selected_candidate": candidate.map(|candidate| json!({
                "candidate_id": candidate.candidate_id,
                "variant_id": candidate.variant_id,
                "rough_score": candidate.rough_score,
                "best_ld": candidate.best_ld,
            })),
            "status": "requested",
            "source": "optimizer_dashboard",
            "note": "Dashboard request only. A runner/controller must consume this file."
        });
        match serde_json::to_string_pretty(&payload)
            .map_err(|err| err.to_string())
            .and_then(|text| fs::write(&path, text).map_err(|err| err.to_string()))
        {
            Ok(()) => self.status = format!("Wrote control request {}", path.display()),
            Err(err) => self.status = format!("Failed to write control request: {err}"),
        }
    }
}

fn measured_table(ui: &mut egui::Ui, candidate: &CandidateRow) {
    egui::Grid::new("measured_data_grid")
        .num_columns(2)
        .striped(true)
        .show(ui, |ui| {
            for (name, value) in [
                ("best alpha", fmt(candidate.best_alpha)),
                ("CL", fmt(candidate.cl)),
                ("CD", fmt(candidate.cd)),
                ("CM", fmt(candidate.cm)),
                ("CL / deg", fmt(candidate.cl_alpha_per_deg)),
                ("CM / deg", fmt(candidate.cm_alpha_per_deg)),
                ("cells", fmt0(candidate.cells)),
                ("aircraft faces", fmt0(candidate.aircraft_faces)),
                ("max non-orthogonality", fmt(candidate.max_non_ortho)),
                ("max skewness", fmt(candidate.max_skewness)),
                ("y+ p50", fmt(candidate.yplus_p50)),
                ("y+ p95", fmt(candidate.yplus_p95)),
                ("y+ p99", fmt(candidate.yplus_p99)),
                ("y+ max", fmt(candidate.yplus_max)),
            ] {
                ui.label(name);
                ui.monospace(value);
                ui.end_row();
            }
        });
}

fn alpha_point_table(ui: &mut egui::Ui, candidate: &CandidateRow) {
    egui::Grid::new(format!("alpha_points_{}", candidate.candidate_id))
        .num_columns(6)
        .striped(true)
        .show(ui, |ui| {
            ui.strong("alpha");
            ui.strong("CL");
            ui.strong("CD");
            ui.strong("CM");
            ui.strong("L/D");
            ui.strong("case");
            ui.end_row();
            for point in &candidate.alpha_points {
                ui.label(format!("{:.1}", point.alpha_deg));
                ui.label(fmt(point.cl));
                ui.label(fmt(point.cd));
                ui.label(fmt(point.cm));
                ui.label(fmt(point.ld));
                ui.small(point.case_path.as_deref().unwrap_or(""));
                ui.end_row();
            }
        });
}

fn named_value_table(ui: &mut egui::Ui, label: &str, values: &[NamedValue]) {
    if values.is_empty() {
        ui.label(format!("No {label} values."));
        return;
    }
    egui::Grid::new(format!("{}_table", label))
        .num_columns(2)
        .striped(true)
        .show(ui, |ui| {
            ui.strong(label);
            ui.strong("value");
            ui.end_row();
            for value in values {
                ui.label(&value.name);
                ui.label(format!("{:.3}", value.value));
                ui.end_row();
            }
        });
}

fn ranked_table(ui: &mut egui::Ui, candidates: &[CandidateRow], limit: usize) {
    egui::Grid::new("ranked_candidates")
        .num_columns(7)
        .striped(true)
        .show(ui, |ui| {
            ui.strong("rank");
            ui.strong("candidate");
            ui.strong("gen");
            ui.strong("score");
            ui.strong("L/D");
            ui.strong("CL");
            ui.strong("CD");
            ui.end_row();
            for (idx, candidate) in candidates.iter().take(limit).enumerate() {
                ui.label((idx + 1).to_string());
                ui.label(&candidate.variant_id);
                ui.label(&candidate.inferred_generation);
                ui.label(fmt(candidate.rough_score));
                ui.label(fmt(candidate.best_ld));
                ui.label(fmt(candidate.cl));
                ui.label(fmt(candidate.cd));
                ui.end_row();
            }
        });
}

fn next_up_panel(ui: &mut egui::Ui, app: &DashboardApp) {
    ui.heading("Next / Proposed");
    let proposed: Vec<_> = app
        .data
        .optimizer_iterations
        .iter()
        .filter(|iteration| iteration.status != "complete")
        .collect();
    if proposed.is_empty() {
        ui.label("No pending optimizer iteration proposals in the current database.");
        ui.small("When generated candidates are recorded before evaluation, they will appear here from optimizer_iterations.proposed_candidate_ids_json.");
    } else {
        for iteration in proposed {
            ui.group(|ui| {
                ui.horizontal_wrapped(|ui| {
                    ui.strong(format!("Iteration {}", iteration.iteration_index));
                    status_label(ui, &iteration.status, 0);
                    ui.small(&iteration.strategy);
                });
                ui.small(format!(
                    "Iteration ID: {}",
                    iteration.optimizer_iteration_id
                ));
                ui.label(&iteration.rationale);
                ui.small(format!("Run: {}", iteration.optimizer_run_id));
                ui.small(format!("Created: {}", iteration.created_at));
                if let Some(completed_at) = &iteration.completed_at {
                    ui.small(format!("Completed: {completed_at}"));
                }
                if !iteration.parent_candidate_ids.is_empty() {
                    ui.small(format!(
                        "Parents: {}",
                        iteration.parent_candidate_ids.join(", ")
                    ));
                }
                ui.label("Proposed candidates:");
                for id in &iteration.proposed_candidate_ids {
                    let label = app
                        .data
                        .candidates
                        .iter()
                        .find(|candidate| &candidate.candidate_id == id)
                        .map(|candidate| candidate.variant_id.as_str())
                        .unwrap_or(id);
                    ui.small(label);
                }
                ui.collapsing("optimizer state", |ui| {
                    json_view(ui, &iteration.optimizer_state)
                });
            });
        }
    }
}

fn optimizer_runs_panel(ui: &mut egui::Ui, app: &DashboardApp) {
    ui.heading("Optimizer Runs");
    if app.data.optimizer_runs.is_empty() {
        ui.label("No optimizer run records.");
        return;
    }
    for run in &app.data.optimizer_runs {
        ui.group(|ui| {
            ui.horizontal_wrapped(|ui| {
                ui.strong(&run.optimizer_name);
                status_label(ui, &run.status, 0);
                ui.small(&run.optimizer_run_id);
            });
            ui.small(format!("Created: {}", run.created_at));
            if let Some(started_at) = &run.started_at {
                ui.small(format!("Started: {started_at}"));
            }
            if let Some(finished_at) = &run.finished_at {
                ui.small(format!("Finished: {finished_at}"));
            }
            ui.collapsing("objective", |ui| json_view(ui, &run.objective));
            ui.collapsing("settings", |ui| json_view(ui, &run.settings));
            ui.collapsing("summary", |ui| json_view(ui, &run.summary));
        });
    }
}

#[derive(Clone, Copy)]
enum PlotMetric {
    Score,
    BestLd,
    Cl,
    Cd,
    Cm,
    MaxNonOrtho,
}

fn metric_for(candidate: &CandidateRow, metric: PlotMetric) -> Option<f64> {
    match metric {
        PlotMetric::Score => candidate.rough_score,
        PlotMetric::BestLd => candidate.best_ld,
        PlotMetric::Cl => candidate.cl,
        PlotMetric::Cd => candidate.cd,
        PlotMetric::Cm => candidate.cm,
        PlotMetric::MaxNonOrtho => candidate.max_non_ortho,
    }
}

fn scatter_plot(
    ui: &mut egui::Ui,
    candidates: &[CandidateRow],
    x_metric: PlotMetric,
    y_metric: PlotMetric,
    x_label: &str,
    y_label: &str,
) {
    let points: Vec<_> = candidates
        .iter()
        .filter_map(|candidate| {
            Some((
                candidate,
                metric_for(candidate, x_metric)?,
                metric_for(candidate, y_metric)?,
            ))
        })
        .collect();
    let desired = egui::vec2(ui.available_width().max(260.0), 260.0);
    let (rect, response) = ui.allocate_exact_size(desired, egui::Sense::hover());
    let painter = ui.painter_at(rect);
    draw_plot_frame(&painter, rect, x_label, y_label);
    if points.is_empty() {
        painter.text(
            rect.center(),
            egui::Align2::CENTER_CENTER,
            "No data",
            egui::FontId::proportional(14.0),
            egui::Color32::from_rgb(170, 176, 184),
        );
        return;
    }
    let (x_min, x_max) = range(points.iter().map(|(_, x, _)| *x));
    let (y_min, y_max) = range(points.iter().map(|(_, _, y)| *y));
    draw_plot_tick_labels(&painter, rect, x_min, x_max, y_min, y_max);
    for (candidate, x, y) in points {
        let pos = plot_pos(rect, x, y, x_min, x_max, y_min, y_max);
        let color = generation_color(&candidate.inferred_generation);
        draw_point_marker(&painter, pos, color, &candidate.inferred_generation, 5.0);
        if response
            .hover_pos()
            .is_some_and(|hover| hover.distance(pos) < 8.0)
        {
            painter.text(
                pos + egui::vec2(6.0, -6.0),
                egui::Align2::LEFT_BOTTOM,
                &candidate.variant_id,
                egui::FontId::monospace(11.0),
                egui::Color32::WHITE,
            );
        }
    }
    draw_plot_legend(&painter, rect);
}

#[derive(Clone, Copy)]
enum CurveMetric {
    Cl,
    Cd,
    Cm,
    Ld,
}

fn curve_value(point: &AlphaPoint, metric: CurveMetric) -> Option<f64> {
    match metric {
        CurveMetric::Cl => point.cl,
        CurveMetric::Cd => point.cd,
        CurveMetric::Cm => point.cm,
        CurveMetric::Ld => point.ld,
    }
}

fn line_plot(ui: &mut egui::Ui, title: &str, points: &[AlphaPoint], metric: CurveMetric) {
    ui.heading(title);
    let values: Vec<_> = points
        .iter()
        .filter_map(|point| Some((point.alpha_deg, curve_value(point, metric)?)))
        .collect();
    let desired = egui::vec2(ui.available_width().max(260.0), 220.0);
    let (rect, _) = ui.allocate_exact_size(desired, egui::Sense::hover());
    let painter = ui.painter_at(rect);
    draw_plot_frame(&painter, rect, "alpha", "");
    if values.is_empty() {
        return;
    }
    let (x_min, x_max) = range(values.iter().map(|(x, _)| *x));
    let (y_min, y_max) = range(values.iter().map(|(_, y)| *y));
    draw_plot_tick_labels(&painter, rect, x_min, x_max, y_min, y_max);
    let screen: Vec<_> = values
        .iter()
        .map(|(x, y)| plot_pos(rect, *x, *y, x_min, x_max, y_min, y_max))
        .collect();
    for pair in screen.windows(2) {
        painter.line_segment(
            [pair[0], pair[1]],
            egui::Stroke::new(2.0, egui::Color32::from_rgb(120, 190, 255)),
        );
    }
    for pos in screen {
        painter.circle_filled(pos, 4.0, egui::Color32::from_rgb(120, 190, 255));
        painter.circle_stroke(
            pos,
            4.0,
            egui::Stroke::new(1.0, egui::Color32::from_rgb(235, 245, 255)),
        );
    }
}

fn named_bar_chart(ui: &mut egui::Ui, title: &str, values: &[NamedValue], detractor: bool) {
    ui.heading(title);
    if values.is_empty() {
        ui.label("No values.");
        return;
    }
    let desired = egui::vec2(ui.available_width().max(260.0), 260.0);
    let (rect, _) = ui.allocate_exact_size(desired, egui::Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(18, 22, 26));
    let max_value = values
        .iter()
        .map(|v| v.value.abs())
        .fold(0.0, f64::max)
        .max(1.0);
    let row_h = (rect.height() / values.len().max(1) as f32).min(30.0);
    for (idx, value) in values.iter().enumerate() {
        let y = rect.top() + 8.0 + idx as f32 * row_h;
        let width = ((value.value.abs() / max_value) as f32) * (rect.width() - 150.0).max(40.0);
        let color = if detractor {
            egui::Color32::from_rgb(230, 120, 100)
        } else {
            egui::Color32::from_rgb(120, 210, 150)
        };
        painter.text(
            egui::pos2(rect.left() + 6.0, y + row_h * 0.5),
            egui::Align2::LEFT_CENTER,
            short_label(&value.name),
            egui::FontId::monospace(10.0),
            egui::Color32::from_rgb(215, 220, 226),
        );
        let bar = egui::Rect::from_min_size(
            egui::pos2(rect.left() + 120.0, y + 5.0),
            egui::vec2(width, (row_h - 10.0).max(4.0)),
        );
        painter.rect_filled(bar, 2.0, color);
        painter.text(
            egui::pos2(bar.right() + 4.0, y + row_h * 0.5),
            egui::Align2::LEFT_CENTER,
            format!("{:.2}", value.value),
            egui::FontId::monospace(10.0),
            egui::Color32::WHITE,
        );
    }
}

fn generation_tree(ui: &mut egui::Ui, app: &mut DashboardApp) {
    let desired = egui::vec2(ui.available_width().max(600.0), 420.0);
    let (rect, response) = ui.allocate_exact_size(desired, egui::Sense::click());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(16, 18, 20));
    if app.data.candidates.is_empty() {
        return;
    }

    let mut rows = app.data.candidates.clone();
    rows.sort_by(|a, b| {
        a.generation
            .cmp(&b.generation)
            .then_with(|| a.variant_id.cmp(&b.variant_id))
    });
    let generations: Vec<i64> = {
        let mut values: Vec<_> = rows.iter().map(|c| c.generation).collect();
        values.sort_unstable();
        values.dedup();
        values
    };
    let col_w = (rect.width() - 40.0) / generations.len().max(1) as f32;
    let mut positions: Vec<(String, egui::Pos2, egui::Rect)> = Vec::new();

    for (col_idx, generation) in generations.iter().enumerate() {
        let gen_rows: Vec<_> = rows
            .iter()
            .filter(|candidate| candidate.generation == *generation)
            .collect();
        painter.text(
            egui::pos2(
                rect.left() + 20.0 + col_idx as f32 * col_w,
                rect.top() + 14.0,
            ),
            egui::Align2::LEFT_CENTER,
            format!("gen {generation}"),
            egui::FontId::monospace(12.0),
            egui::Color32::from_rgb(190, 200, 210),
        );
        for (row_idx, candidate) in gen_rows.iter().enumerate() {
            let x = rect.left() + 20.0 + col_idx as f32 * col_w;
            let y = rect.top() + 48.0 + row_idx as f32 * 42.0;
            let node_rect = egui::Rect::from_min_size(
                egui::pos2(x, y),
                egui::vec2((col_w - 18.0).max(100.0), 30.0),
            );
            positions.push((
                candidate.candidate_id.clone(),
                node_rect.center(),
                node_rect,
            ));
        }
    }

    for candidate in &rows {
        let Some((_, child_pos, _)) = positions
            .iter()
            .find(|(id, _, _)| id == &candidate.candidate_id)
        else {
            continue;
        };
        for lineage in &candidate.lineage {
            for parent_id in &lineage.parent_ids {
                if let Some((_, parent_pos, _)) =
                    positions.iter().find(|(id, _, _)| id == parent_id)
                {
                    painter.line_segment(
                        [
                            *parent_pos + egui::vec2(50.0, 0.0),
                            *child_pos - egui::vec2(50.0, 0.0),
                        ],
                        egui::Stroke::new(1.0, egui::Color32::from_rgb(95, 115, 135)),
                    );
                }
            }
        }
    }

    let click_pos = response.interact_pointer_pos();
    for candidate in &rows {
        let Some((_, _, node_rect)) = positions
            .iter()
            .find(|(id, _, _)| id == &candidate.candidate_id)
        else {
            continue;
        };
        let selected = app
            .selected_candidate_id
            .as_ref()
            .is_some_and(|id| id == &candidate.candidate_id);
        let color = if selected {
            egui::Color32::from_rgb(55, 90, 115)
        } else {
            egui::Color32::from_rgb(30, 38, 46)
        };
        painter.rect_filled(*node_rect, 4.0, color);
        painter.rect_stroke(
            *node_rect,
            4.0,
            egui::Stroke::new(1.0, generation_color(&candidate.inferred_generation)),
        );
        painter.text(
            node_rect.center(),
            egui::Align2::CENTER_CENTER,
            format!(
                "{}  {}",
                short_label(&candidate.variant_id),
                fmt(candidate.rough_score)
            ),
            egui::FontId::monospace(10.0),
            egui::Color32::WHITE,
        );
        if response.clicked() && click_pos.is_some_and(|pos| node_rect.contains(pos)) {
            app.select_candidate(&candidate.candidate_id);
        }
    }
}

fn draw_plot_frame(painter: &egui::Painter, rect: egui::Rect, x_label: &str, y_label: &str) {
    painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(18, 22, 26));
    let plot = plot_rect(rect);
    let grid_color = egui::Color32::from_rgb(38, 48, 58);
    for idx in 0..=4 {
        let t = idx as f32 / 4.0;
        let x = plot.left() + plot.width() * t;
        painter.line_segment(
            [egui::pos2(x, plot.top()), egui::pos2(x, plot.bottom())],
            egui::Stroke::new(1.0, grid_color),
        );
        let y = plot.top() + plot.height() * t;
        painter.line_segment(
            [egui::pos2(plot.left(), y), egui::pos2(plot.right(), y)],
            egui::Stroke::new(1.0, grid_color),
        );
    }
    painter.rect_stroke(
        plot,
        0.0,
        egui::Stroke::new(1.0, egui::Color32::from_rgb(65, 75, 85)),
    );
    painter.text(
        egui::pos2(plot.center().x, rect.bottom() - 10.0),
        egui::Align2::CENTER_BOTTOM,
        x_label,
        egui::FontId::monospace(11.0),
        egui::Color32::from_rgb(180, 188, 196),
    );
    painter.text(
        egui::pos2(rect.left() + 8.0, plot.top()),
        egui::Align2::LEFT_TOP,
        y_label,
        egui::FontId::monospace(11.0),
        egui::Color32::from_rgb(180, 188, 196),
    );
}

fn draw_plot_tick_labels(
    painter: &egui::Painter,
    rect: egui::Rect,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
) {
    let plot = plot_rect(rect);
    for idx in 0..=4 {
        let t = idx as f64 / 4.0;
        let x_value = x_min + (x_max - x_min) * t;
        let x = plot.left() + plot.width() * t as f32;
        painter.text(
            egui::pos2(x, plot.bottom() + 4.0),
            egui::Align2::CENTER_TOP,
            compact_tick(x_value),
            egui::FontId::monospace(9.0),
            egui::Color32::from_rgb(150, 160, 170),
        );

        let y_value = y_max - (y_max - y_min) * t;
        let y = plot.top() + plot.height() * t as f32;
        painter.text(
            egui::pos2(plot.left() - 6.0, y),
            egui::Align2::RIGHT_CENTER,
            compact_tick(y_value),
            egui::FontId::monospace(9.0),
            egui::Color32::from_rgb(150, 160, 170),
        );
    }
}

fn draw_point_marker(
    painter: &egui::Painter,
    pos: egui::Pos2,
    color: egui::Color32,
    generation: &str,
    size: f32,
) {
    let stroke = egui::Stroke::new(1.25, egui::Color32::from_rgb(245, 248, 252));
    if generation.contains("gen2") {
        let points = [
            pos + egui::vec2(0.0, -size),
            pos + egui::vec2(size, 0.0),
            pos + egui::vec2(0.0, size),
            pos + egui::vec2(-size, 0.0),
        ];
        painter.add(egui::Shape::convex_polygon(points.to_vec(), color, stroke));
    } else if generation.contains("gen1") {
        let rect = egui::Rect::from_center_size(pos, egui::vec2(size * 1.7, size * 1.7));
        painter.rect_filled(rect, 1.0, color);
        painter.rect_stroke(rect, 1.0, stroke);
    } else {
        painter.circle_filled(pos, size, color);
        painter.circle_stroke(pos, size, stroke);
    }
}

fn draw_plot_legend(painter: &egui::Painter, rect: egui::Rect) {
    let origin = rect.right_top() + egui::vec2(-120.0, 10.0);
    for (idx, (label, gen_label)) in [("gen0", "gen0"), ("gen1", "gen1"), ("gen2", "gen2")]
        .iter()
        .enumerate()
    {
        let y = origin.y + idx as f32 * 16.0;
        let pos = egui::pos2(origin.x, y + 7.0);
        draw_point_marker(painter, pos, generation_color(gen_label), gen_label, 4.0);
        painter.text(
            egui::pos2(origin.x + 10.0, y + 7.0),
            egui::Align2::LEFT_CENTER,
            *label,
            egui::FontId::monospace(9.0),
            egui::Color32::from_rgb(180, 188, 196),
        );
    }
}

fn plot_rect(rect: egui::Rect) -> egui::Rect {
    egui::Rect::from_min_max(
        rect.min + egui::vec2(42.0, 18.0),
        rect.max - egui::vec2(18.0, 32.0),
    )
}

fn plot_pos(
    rect: egui::Rect,
    x: f64,
    y: f64,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
) -> egui::Pos2 {
    let plot = plot_rect(rect);
    let tx = if (x_max - x_min).abs() < f64::EPSILON {
        0.5
    } else {
        (x - x_min) / (x_max - x_min)
    };
    let ty = if (y_max - y_min).abs() < f64::EPSILON {
        0.5
    } else {
        (y - y_min) / (y_max - y_min)
    };
    egui::pos2(
        plot.left() + (tx as f32) * plot.width(),
        plot.bottom() - (ty as f32) * plot.height(),
    )
}

fn range(values: impl Iterator<Item = f64>) -> (f64, f64) {
    let mut min = f64::INFINITY;
    let mut max = f64::NEG_INFINITY;
    for value in values {
        min = min.min(value);
        max = max.max(value);
    }
    if !min.is_finite() || !max.is_finite() {
        return (0.0, 1.0);
    }
    if (max - min).abs() < f64::EPSILON {
        (min - 1.0, max + 1.0)
    } else {
        let pad = (max - min) * 0.08;
        (min - pad, max + pad)
    }
}

fn generation_color(label: &str) -> egui::Color32 {
    if label.contains("gen2") {
        egui::Color32::from_rgb(130, 210, 255)
    } else if label.contains("gen1") {
        egui::Color32::from_rgb(155, 220, 150)
    } else if label.contains("gen0") {
        egui::Color32::from_rgb(235, 190, 110)
    } else {
        egui::Color32::from_rgb(190, 190, 205)
    }
}

fn short_label(value: &str) -> String {
    const MAX: usize = 28;
    if value.chars().count() <= MAX {
        value.to_string()
    } else {
        let prefix: String = value.chars().take(MAX - 1).collect();
        format!("{prefix}...")
    }
}

fn compact_tick(value: f64) -> String {
    let abs = value.abs();
    if abs >= 1000.0 {
        format!("{value:.0}")
    } else if abs >= 10.0 {
        format!("{value:.1}")
    } else {
        format!("{value:.2}")
    }
}

fn metric_box(ui: &mut egui::Ui, label: &str, value: String) {
    egui::Frame::none()
        .fill(egui::Color32::from_rgb(28, 34, 40))
        .inner_margin(egui::Margin::symmetric(8.0, 6.0))
        .show(ui, |ui| {
            ui.vertical(|ui| {
                ui.small(label);
                ui.strong(if value.is_empty() {
                    "-".to_string()
                } else {
                    value
                });
            });
        });
}

fn status_label(ui: &mut egui::Ui, status: &str, failure_count: usize) {
    let color = if failure_count > 0 || status == "failed" {
        egui::Color32::from_rgb(255, 150, 120)
    } else if status == "success" || status == "complete" || status == "evaluated" {
        egui::Color32::from_rgb(130, 220, 150)
    } else {
        egui::Color32::from_rgb(220, 190, 110)
    };
    if failure_count > 0 {
        ui.colored_label(color, format!("{status} ({failure_count})"));
    } else {
        ui.colored_label(color, status);
    }
}

fn runner_status_label(ui: &mut egui::Ui, candidate: &CandidateRow) {
    if let Some(state) = &candidate.runner_state {
        let color = runner_state_color(&state.state);
        ui.horizontal(|ui| {
            let (rect, _) = ui.allocate_exact_size(egui::vec2(9.0, 9.0), egui::Sense::hover());
            ui.painter().circle_filled(rect.center(), 4.0, color);
            ui.colored_label(color, format!("{} / {}", state.state, state.stage));
        });
    } else {
        status_label(ui, &candidate.status, candidate.failure_count);
    }
}

fn candidate_history_state(candidate: &CandidateRow) -> String {
    if let Some(state) = &candidate.runner_state {
        return state.state.clone();
    }
    let status = candidate.status.to_ascii_lowercase();
    if candidate.failure_count > 0 || status.contains("fail") || status.contains("error") {
        "failed".to_string()
    } else if status.contains("running")
        || status.contains("active")
        || status.contains("in_progress")
        || status.contains("started")
    {
        "running".to_string()
    } else if status == "success"
        || status == "complete"
        || status == "completed"
        || status == "evaluated"
        || candidate.rough_score.is_some()
        || candidate.best_ld.is_some()
    {
        "ran".to_string()
    } else {
        "pending".to_string()
    }
}

fn candidate_history_rank(candidate: &CandidateRow) -> i32 {
    match candidate_history_state(candidate).as_str() {
        "running" | "meshing" | "cfd_running" => 0,
        "queued" | "pending" => 1,
        "failed" => 2,
        "promoted" => 3,
        "scored" => 4,
        "rejected" => 5,
        _ => 6,
    }
}

fn history_state_label(ui: &mut egui::Ui, candidate: &CandidateRow) {
    let state = candidate_history_state(candidate);
    let color = runner_state_color(&state);
    ui.horizontal(|ui| {
        let (rect, _) = ui.allocate_exact_size(egui::vec2(9.0, 9.0), egui::Sense::hover());
        ui.painter().circle_filled(rect.center(), 4.0, color);
        ui.colored_label(color, state);
    });
}

fn runner_state_color(state: &str) -> egui::Color32 {
    match state {
        "promoted" => egui::Color32::from_rgb(130, 220, 150),
        "scored" | "ran" => egui::Color32::from_rgb(155, 220, 150),
        "running" | "meshing" | "cfd_running" => egui::Color32::from_rgb(130, 210, 255),
        "queued" | "pending" => egui::Color32::from_rgb(220, 190, 110),
        "rejected" => egui::Color32::from_rgb(180, 185, 195),
        "failed" => egui::Color32::from_rgb(255, 150, 120),
        _ => egui::Color32::from_rgb(190, 190, 205),
    }
}

fn parent_summary(candidate: &CandidateRow, all_candidates: &[CandidateRow]) -> String {
    let mut labels = Vec::new();
    for lineage in &candidate.lineage {
        for parent_id in &lineage.parent_ids {
            let label = all_candidates
                .iter()
                .find(|candidate| &candidate.candidate_id == parent_id)
                .map(|candidate| short_label(&candidate.variant_id))
                .unwrap_or_else(|| short_label(parent_id));
            if !labels.contains(&label) {
                labels.push(label);
            }
        }
    }
    if labels.is_empty() {
        "-".to_string()
    } else {
        labels.join(", ")
    }
}

fn candidate_runner_stage(candidate: &CandidateRow) -> String {
    candidate
        .runner_state
        .as_ref()
        .map(|state| {
            if let Some(progress) = state.progress {
                format!("{} ({:.0}%)", state.stage, progress * 100.0)
            } else {
                state.stage.clone()
            }
        })
        .unwrap_or_else(|| "-".to_string())
}

fn history_note(candidate: &CandidateRow) -> String {
    if let Some(state) = &candidate.runner_state {
        if let Some(message) = &state.message {
            return message.clone();
        }
        if let Some(reason) = &state.reason {
            return reason.clone();
        }
    }
    if candidate.failure_count > 0 {
        return format!("{} failure(s)", candidate.failure_count);
    }
    if candidate.alpha_points.is_empty() {
        return "no alpha sweep".to_string();
    }
    if candidate.yplus_p95.is_some() {
        return "rough CFD + y+".to_string();
    }
    if candidate.cells.is_some() {
        return "rough CFD".to_string();
    }
    if candidate.viewer_target.is_some() {
        return "geometry available".to_string();
    }
    "-".to_string()
}

fn json_view(ui: &mut egui::Ui, value: &Value) {
    let text = serde_json::to_string_pretty(value).unwrap_or_else(|_| value.to_string());
    let mut display = text;
    ui.add(
        egui::TextEdit::multiline(&mut display)
            .font(egui::TextStyle::Monospace)
            .desired_width(f32::INFINITY)
            .desired_rows(8)
            .interactive(false),
    );
}

fn load_dashboard_data(workspace: &Path, db_path: &Path, repo_root: &Path) -> DashboardData {
    let mut data = DashboardData {
        ..Default::default()
    };
    let connection = match Connection::open(db_path) {
        Ok(connection) => connection,
        Err(err) => {
            data.load_error = Some(format!("Failed to open {}: {err}", db_path.display()));
            return data;
        }
    };

    data.campaigns = load_campaigns(&connection);
    data.optimizer_runs = load_optimizer_runs(&connection);
    data.optimizer_iterations = load_optimizer_iterations(&connection);
    data.candidates = load_candidates(&connection, workspace, repo_root);
    data.candidates.sort_by(|a, b| {
        b.rough_score
            .partial_cmp(&a.rough_score)
            .unwrap_or(Ordering::Equal)
    });
    data.loaded_at = Some(Instant::now());
    data
}

fn load_optimizer_runs(connection: &Connection) -> Vec<OptimizerRunRow> {
    let mut stmt = match connection.prepare(
        r#"
        SELECT optimizer_run_id, optimizer_name, status, objective_json, settings_json,
               summary_json, created_at, started_at, finished_at
        FROM optimizer_runs
        ORDER BY created_at DESC, rowid DESC
        "#,
    ) {
        Ok(stmt) => stmt,
        Err(_) => return Vec::new(),
    };
    stmt.query_map([], |row| {
        let objective: Option<String> = row.get(3)?;
        let settings: Option<String> = row.get(4)?;
        let summary: Option<String> = row.get(5)?;
        Ok(OptimizerRunRow {
            optimizer_run_id: row.get(0)?,
            optimizer_name: row.get(1)?,
            status: row.get(2)?,
            objective: objective.as_deref().map(parse_json).unwrap_or(Value::Null),
            settings: settings.as_deref().map(parse_json).unwrap_or(Value::Null),
            summary: summary.as_deref().map(parse_json).unwrap_or(Value::Null),
            created_at: row.get(6)?,
            started_at: row.get(7)?,
            finished_at: row.get(8)?,
        })
    })
    .map(|rows| rows.flatten().collect())
    .unwrap_or_default()
}

fn load_optimizer_iterations(connection: &Connection) -> Vec<OptimizerIterationRow> {
    let mut stmt = match connection.prepare(
        r#"
        SELECT optimizer_iteration_id, optimizer_run_id, iteration_index, status,
               proposed_candidate_ids_json, parent_candidate_ids_json, strategy,
               rationale, optimizer_state_json, created_at, completed_at
        FROM optimizer_iterations
        ORDER BY iteration_index, created_at, rowid
        "#,
    ) {
        Ok(stmt) => stmt,
        Err(_) => return Vec::new(),
    };
    stmt.query_map([], |row| {
        let proposed: Option<String> = row.get(4)?;
        let parents: Option<String> = row.get(5)?;
        let state: Option<String> = row.get(8)?;
        Ok(OptimizerIterationRow {
            optimizer_iteration_id: row.get(0)?,
            optimizer_run_id: row.get(1)?,
            iteration_index: row.get(2)?,
            status: row.get(3)?,
            proposed_candidate_ids: json_string_list(proposed.as_deref()),
            parent_candidate_ids: json_string_list(parents.as_deref()),
            strategy: row.get(6)?,
            rationale: row.get(7)?,
            optimizer_state: state.as_deref().map(parse_json).unwrap_or(Value::Null),
            created_at: row.get(9)?,
            completed_at: row.get(10)?,
        })
    })
    .map(|rows| rows.flatten().collect())
    .unwrap_or_default()
}

fn load_campaigns(connection: &Connection) -> Vec<String> {
    let mut stmt = match connection.prepare(
        "SELECT name, status, created_at FROM campaigns ORDER BY created_at DESC, rowid DESC",
    ) {
        Ok(stmt) => stmt,
        Err(_) => return Vec::new(),
    };
    stmt.query_map([], |row| {
        Ok(format!(
            "{} ({}, {})",
            row.get::<_, String>(0)?,
            row.get::<_, String>(1)?,
            row.get::<_, String>(2)?
        ))
    })
    .map(|rows| rows.flatten().collect())
    .unwrap_or_default()
}

fn load_candidates(
    connection: &Connection,
    workspace: &Path,
    repo_root: &Path,
) -> Vec<CandidateRow> {
    let mut stmt = match connection.prepare(
        r#"
        SELECT c.candidate_id, c.campaign_id, c.generation, c.status, c.notes,
               c.design_variables_json, e.evaluation_id, e.status AS evaluation_status
        FROM candidates c
        LEFT JOIN evaluations e ON e.candidate_id = c.candidate_id
        ORDER BY c.created_at, c.rowid
        "#,
    ) {
        Ok(stmt) => stmt,
        Err(_) => return Vec::new(),
    };

    let rows = stmt
        .query_map([], |row| {
            candidate_from_row(connection, workspace, repo_root, row)
        })
        .map(|rows| rows.flatten().collect())
        .unwrap_or_default();
    rows
}

fn candidate_from_row(
    connection: &Connection,
    workspace: &Path,
    repo_root: &Path,
    row: &Row,
) -> rusqlite::Result<CandidateRow> {
    let candidate_id: String = row.get(0)?;
    let campaign_id: String = row.get(1)?;
    let generation: i64 = row.get(2)?;
    let status: String = row.get(3)?;
    let notes: Option<String> = row.get(4)?;
    let design_variables_json: String = row.get(5)?;
    let evaluation_id: Option<String> = row.get(6)?;
    let evaluation_status: Option<String> = row.get(7)?;
    let lineage = load_lineage(connection, &candidate_id);
    let variant_id = notes
        .as_deref()
        .and_then(extract_variant_id)
        .or_else(|| {
            lineage.iter().find_map(|item| {
                item.mutation_summary
                    .get("variant_id")?
                    .as_str()
                    .map(str::to_string)
            })
        })
        .unwrap_or_else(|| candidate_id.clone());
    let module_attempts = load_module_attempts(connection, &candidate_id);
    let failures = load_failures(connection, &candidate_id);
    let runner_state = load_runner_state(connection, &candidate_id);
    let rough_metrics = latest_module(&module_attempts, "rough_cfd_scoring")
        .map(|attempt| &attempt.metrics)
        .unwrap_or(&Value::Null);
    let rough_metadata = latest_module(&module_attempts, "rough_cfd_scoring")
        .map(|attempt| &attempt.metadata)
        .unwrap_or(&Value::Null);
    let smoke_metrics = latest_module(&module_attempts, "openfoam_smoke_validation")
        .map(|attempt| &attempt.metrics)
        .unwrap_or(&Value::Null);
    let steady_metrics = latest_module(&module_attempts, "openfoam_steady_validation")
        .map(|attempt| &attempt.metrics)
        .unwrap_or(&Value::Null);
    let best_usable = rough_metadata
        .pointer("/alpha_sweep_aggregate/best_usable")
        .unwrap_or(&Value::Null);
    let alpha_points = alpha_points_from_metadata(rough_metadata);

    Ok(CandidateRow {
        candidate_id: candidate_id.clone(),
        variant_id: variant_id.clone(),
        campaign_id,
        generation,
        inferred_generation: infer_generation_label(&variant_id, generation),
        status,
        evaluation_id,
        evaluation_status,
        notes,
        design_variables: parse_json(&design_variables_json),
        rough_score: metric_value(rough_metrics, "score.rough_total"),
        best_ld: metric_value(rough_metrics, "score.rough_best_usable_ld"),
        best_alpha: best_usable.get("alpha_deg").and_then(Value::as_f64),
        cl: best_usable.get("cl").and_then(Value::as_f64),
        cd: best_usable.get("cd").and_then(Value::as_f64),
        cm: best_usable.get("cm").and_then(Value::as_f64),
        cl_alpha_per_deg: metric_value(rough_metrics, "score.rough_cl_alpha_per_deg"),
        cm_alpha_per_deg: metric_value(rough_metrics, "score.rough_cm_alpha_per_deg"),
        alpha_points,
        score_components: named_values_from_object(rough_metadata.pointer("/component_values")),
        detractors: named_values_from_object(rough_metadata.pointer("/detractor_values")),
        cells: metric_value(smoke_metrics, "openfoam.cells"),
        aircraft_faces: metric_value(smoke_metrics, "openfoam.aircraft_faces"),
        max_non_ortho: metric_value(smoke_metrics, "openfoam.max_non_orthogonality"),
        max_skewness: metric_value(smoke_metrics, "openfoam.max_skewness"),
        yplus_p50: metric_value(steady_metrics, "openfoam_steady.yplus_p50"),
        yplus_p95: metric_value(steady_metrics, "openfoam_steady.yplus_p95"),
        yplus_p99: metric_value(steady_metrics, "openfoam_steady.yplus_p99"),
        yplus_max: metric_value(steady_metrics, "openfoam_steady.yplus_max"),
        failure_count: failures.len(),
        viewer_target: find_viewer_target(
            connection,
            workspace,
            repo_root,
            &candidate_id,
            &variant_id,
        ),
        lineage,
        module_attempts,
        failures,
        runner_state,
    })
}

fn load_runner_state(connection: &Connection, candidate_id: &str) -> Option<RunnerStateRow> {
    let mut stmt = connection
        .prepare(
            r#"
            SELECT state, stage, active_module, progress, priority, updated_at,
                   reason, message, metadata_json
            FROM candidate_runner_states
            WHERE candidate_id = ?
            "#,
        )
        .ok()?;
    stmt.query_row([candidate_id], |row| {
        let metadata: Option<String> = row.get(8)?;
        Ok(RunnerStateRow {
            state: row.get(0)?,
            stage: row.get(1)?,
            active_module: row.get(2)?,
            progress: row.get(3)?,
            priority: row.get(4)?,
            updated_at: row.get(5)?,
            reason: row.get(6)?,
            message: row.get(7)?,
            metadata: metadata.as_deref().map(parse_json).unwrap_or(Value::Null),
        })
    })
    .ok()
}

fn alpha_points_from_metadata(metadata: &Value) -> Vec<AlphaPoint> {
    metadata
        .pointer("/alpha_sweep_aggregate/points")
        .and_then(Value::as_array)
        .map(|points| {
            points
                .iter()
                .filter_map(|point| {
                    Some(AlphaPoint {
                        alpha_deg: point.get("alpha_deg")?.as_f64()?,
                        cl: point.get("cl").and_then(Value::as_f64),
                        cd: point.get("cd").and_then(Value::as_f64),
                        cm: point.get("cm").and_then(Value::as_f64),
                        ld: point.get("ld").and_then(Value::as_f64),
                        case_path: point
                            .get("rans_case")
                            .and_then(Value::as_str)
                            .map(str::to_string),
                    })
                })
                .collect()
        })
        .unwrap_or_default()
}

fn named_values_from_object(value: Option<&Value>) -> Vec<NamedValue> {
    let mut values: Vec<_> = value
        .and_then(Value::as_object)
        .map(|object| {
            object
                .iter()
                .filter_map(|(name, value)| {
                    Some(NamedValue {
                        name: name.clone(),
                        value: value.as_f64()?,
                    })
                })
                .collect()
        })
        .unwrap_or_default();
    values.sort_by(|a: &NamedValue, b: &NamedValue| a.name.cmp(&b.name));
    values
}

fn json_string_list(text: Option<&str>) -> Vec<String> {
    text.map(parse_json)
        .and_then(|value| {
            value.as_array().map(|items| {
                items
                    .iter()
                    .filter_map(Value::as_str)
                    .map(str::to_string)
                    .collect()
            })
        })
        .unwrap_or_default()
}

fn load_lineage(connection: &Connection, candidate_id: &str) -> Vec<LineageRow> {
    let mut stmt = match connection.prepare(
        r#"
        SELECT parent_candidate_ids_json, operator, reason, mutation_summary_json
        FROM candidate_lineage
        WHERE child_candidate_id = ?
        ORDER BY created_at
        "#,
    ) {
        Ok(stmt) => stmt,
        Err(_) => return Vec::new(),
    };
    stmt.query_map([candidate_id], |row| {
        let parents: String = row.get(0)?;
        let mutation: Option<String> = row.get(3)?;
        Ok(LineageRow {
            parent_ids: parse_json(&parents)
                .as_array()
                .map(|items| {
                    items
                        .iter()
                        .filter_map(Value::as_str)
                        .map(str::to_string)
                        .collect()
                })
                .unwrap_or_default(),
            operator: row.get(1)?,
            reason: row.get(2)?,
            mutation_summary: mutation.as_deref().map(parse_json).unwrap_or(Value::Null),
        })
    })
    .map(|rows| rows.flatten().collect())
    .unwrap_or_default()
}

fn load_module_attempts(connection: &Connection, candidate_id: &str) -> Vec<ModuleAttemptRow> {
    let mut stmt = match connection.prepare(
        r#"
        SELECT module_name, status, runtime_seconds, metrics_json, metadata_json, warnings_json
        FROM module_attempts
        WHERE candidate_id = ?
        ORDER BY started_at, rowid
        "#,
    ) {
        Ok(stmt) => stmt,
        Err(_) => return Vec::new(),
    };
    stmt.query_map([candidate_id], |row| {
        let metrics: Option<String> = row.get(3)?;
        let metadata: Option<String> = row.get(4)?;
        let warnings: Option<String> = row.get(5)?;
        Ok(ModuleAttemptRow {
            module_name: row.get(0)?,
            status: row.get(1)?,
            runtime_seconds: row.get(2)?,
            metrics: metrics.as_deref().map(parse_json).unwrap_or(Value::Null),
            metadata: metadata.as_deref().map(parse_json).unwrap_or(Value::Null),
            warnings: warnings.as_deref().map(parse_json).unwrap_or(Value::Null),
        })
    })
    .map(|rows| rows.flatten().collect())
    .unwrap_or_default()
}

fn load_failures(connection: &Connection, candidate_id: &str) -> Vec<FailureRow> {
    let mut stmt = match connection.prepare(
        r#"
        SELECT category, stage, severity, message
        FROM failures
        WHERE candidate_id = ?
        ORDER BY created_at
        "#,
    ) {
        Ok(stmt) => stmt,
        Err(_) => return Vec::new(),
    };
    stmt.query_map([candidate_id], |row| {
        Ok(FailureRow {
            category: row.get(0)?,
            stage: row.get(1)?,
            severity: row.get(2)?,
            message: row.get(3)?,
        })
    })
    .map(|rows| rows.flatten().collect())
    .unwrap_or_default()
}

fn find_viewer_target(
    connection: &Connection,
    workspace: &Path,
    repo_root: &Path,
    candidate_id: &str,
    variant_id: &str,
) -> Option<PathBuf> {
    if let Some(path) = find_rhai_preview_target(repo_root, variant_id) {
        return Some(path);
    }

    if let Some(path) =
        find_geometry_provider_script(connection, workspace, repo_root, candidate_id)
    {
        return Some(path);
    }

    for artifact_type in [
        "generated_rhai",
        "sdf_rhai",
        "rhai",
        "geometry_script",
        "export_result_json",
        "oml_stl",
        "stl",
        "openfoam_stl",
    ] {
        let mut stmt = connection
            .prepare(
                r#"
                SELECT path, source_original_path
                FROM artifacts
                WHERE candidate_id = ? AND artifact_type = ?
                ORDER BY created_at DESC, rowid DESC
                "#,
            )
            .ok()?;
        let rows = stmt
            .query_map([candidate_id, artifact_type], |row| {
                Ok((
                    row.get::<_, Option<String>>(0)?,
                    row.get::<_, Option<String>>(1)?,
                ))
            })
            .ok()?;
        for item in rows.flatten() {
            for value in [item.0, item.1].into_iter().flatten() {
                let path = resolve_path(&value, workspace, repo_root);
                if path.exists() {
                    return Some(path);
                }
            }
        }
    }

    let result = repo_root
        .join("dual_contouring")
        .join("direct_sparse_sdf_mc_experiment")
        .join("logs")
        .join(format!("{variant_id}_optimizer_export_result.json"));
    if result.exists() {
        return Some(result);
    }

    let stl_dir = repo_root
        .join("dual_contouring")
        .join("direct_sparse_sdf_mc_experiment")
        .join("stl");
    std::fs::read_dir(stl_dir)
        .ok()?
        .flatten()
        .map(|entry| entry.path())
        .find(|path| {
            path.file_name()
                .and_then(|s| s.to_str())
                .is_some_and(|name| name.contains(variant_id) && name.ends_with(".stl"))
        })
}

fn find_geometry_provider_script(
    connection: &Connection,
    workspace: &Path,
    repo_root: &Path,
    candidate_id: &str,
) -> Option<PathBuf> {
    let mut stmt = connection
        .prepare(
            r#"
            SELECT script_path
            FROM geometry_provider_results
            WHERE candidate_id = ? AND script_path IS NOT NULL
            ORDER BY created_at DESC, rowid DESC
            "#,
        )
        .ok()?;
    let rows = stmt
        .query_map([candidate_id], |row| row.get::<_, Option<String>>(0))
        .ok()?;
    for value in rows.flatten().flatten() {
        let path = resolve_path(&value, workspace, repo_root);
        if path.exists() {
            return Some(path);
        }
    }
    None
}

fn find_rhai_preview_target(repo_root: &Path, variant_id: &str) -> Option<PathBuf> {
    let scratch_dir = repo_root
        .join("dual_contouring")
        .join("direct_sparse_sdf_mc_experiment")
        .join("scratch");
    let direct = scratch_dir.join(format!(
        "aircraft_oml_{variant_id}_faired_cap_native_frame.rhai"
    ));
    if direct.exists() {
        return Some(direct);
    }

    std::fs::read_dir(&scratch_dir)
        .ok()?
        .flatten()
        .map(|entry| entry.path())
        .filter(|path| path.extension().and_then(|s| s.to_str()) == Some("rhai"))
        .find(|path| {
            path.file_name()
                .and_then(|s| s.to_str())
                .is_some_and(|name| name.contains(variant_id))
        })
}

fn latest_module<'a>(
    attempts: &'a [ModuleAttemptRow],
    module_name: &str,
) -> Option<&'a ModuleAttemptRow> {
    attempts
        .iter()
        .rev()
        .find(|attempt| attempt.module_name == module_name)
}

fn metric_value(metrics: &Value, key: &str) -> Option<f64> {
    metrics.get(key)?.get("value")?.as_f64()
}

fn parse_json(text: &str) -> Value {
    serde_json::from_str(text).unwrap_or(Value::Null)
}

fn extract_variant_id(notes: &str) -> Option<String> {
    let marker = " for ";
    let start = notes.find(marker)? + marker.len();
    let rest = &notes[start..];
    let end = rest.find('.')?;
    Some(rest[..end].to_string())
}

fn infer_generation_label(variant_id: &str, generation: i64) -> String {
    if variant_id.starts_with("opg2_") {
        "gen2".to_string()
    } else if variant_id.starts_with("opg1_") {
        "gen1".to_string()
    } else if variant_id.starts_with("shv") {
        "gen0".to_string()
    } else {
        format!("db {generation}")
    }
}

fn resolve_path(value: &str, workspace: &Path, repo_root: &Path) -> PathBuf {
    let path = PathBuf::from(value);
    if path.is_absolute() {
        path
    } else if value.starts_with("..") {
        workspace.join(path).components().collect()
    } else {
        repo_root.join(path)
    }
}

fn fmt(value: Option<f64>) -> String {
    value.map(|v| format!("{v:.3}")).unwrap_or_default()
}

fn fmt0(value: Option<f64>) -> String {
    value
        .map(|v| format!("{}", v.round() as i64))
        .unwrap_or_default()
}

fn parse_arg_value(args: &[String], name: &str) -> Option<PathBuf> {
    args.windows(2)
        .find(|pair| pair[0] == name)
        .map(|pair| PathBuf::from(&pair[1]))
}

fn print_help() {
    println!(
        "Usage: optimizer_dashboard --workspace <workspace-or-optimizer.db>\n\nDefault workspace: aircraft_optimizer_platform/runs/pilot_loop_gen2_3_snappy_firstpass_import"
    );
}

fn main() -> eframe::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    if args.iter().any(|arg| arg == "-h" || arg == "--help") {
        print_help();
        return Ok(());
    }

    let workspace = parse_arg_value(&args, "--workspace").unwrap_or_else(|| {
        PathBuf::from("aircraft_optimizer_platform")
            .join("runs")
            .join("pilot_loop_gen2_3_snappy_firstpass_import")
    });
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1450.0, 900.0]),
        renderer: eframe::Renderer::Wgpu,
        ..Default::default()
    };

    eframe::run_native(
        "Aircraft Optimizer Dashboard",
        options,
        Box::new(move |_cc| Ok(Box::new(DashboardApp::new(workspace)))),
    )
}
