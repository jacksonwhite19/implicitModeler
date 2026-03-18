// UI sub-modules

pub mod examples;

pub mod spline_editor;

pub mod spine_editor;

pub mod project_tree;

pub mod syntax;
pub mod autocomplete;
pub use autocomplete::AutocompleteState;

pub mod dimensions;
pub use dimensions::DimensionsState;

pub mod script_variable_detector;
pub use script_variable_detector::DetectionType;

pub mod library_panel;

pub mod templates;
pub mod project_wizard;

pub mod version_control_panel;
pub use version_control_panel::VCPanelState;

pub mod help_data;
pub mod help_search;
pub mod help_panel;
pub use help_search::HelpSearchState;
