// UI sub-modules

pub mod examples;

pub mod spline_editor;

pub mod spine_editor;

pub mod project_tree;

pub mod autocomplete;
pub mod syntax;
pub use autocomplete::AutocompleteState;

pub mod dimensions;
pub use dimensions::DimensionsState;

pub mod script_variable_detector;
pub use script_variable_detector::DetectionType;

pub mod library_panel;

pub mod project_wizard;
pub mod templates;

pub mod version_control_panel;
pub use version_control_panel::VCPanelState;

pub mod help_data;
pub mod help_panel;
pub mod help_search;
pub use help_search::HelpSearchState;
