// Project-local component library system.
//
// Library components live in `lib/` alongside the `.icad` project file.
// Each `.rhai` file in `lib/` defines a named module automatically available
// in the main script via `component_name::function_name(args)` syntax.

pub mod metadata;
pub mod manager;
pub mod thumbnail;

pub use manager::{LibraryManager, ThumbnailState};
