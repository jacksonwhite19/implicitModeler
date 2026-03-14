// Visual node graph editor — Phase 13

pub mod types;
pub mod codegen;
pub mod ui;

pub use types::*;
pub use ui::{show_graph_panel, GraphUiState};

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum EditorMode {
    #[default]
    Script,
    Notebook,
    Graph,  // kept for project file compatibility but not shown as a tab
}
