// Library component manager — scans lib/ directory, parses metadata, manages thumbnail state.

use std::path::{Path, PathBuf};
use std::sync::mpsc::Receiver;
use crate::library::metadata::{ComponentMetadata, FunctionSignature, extract_function_signatures};

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ThumbnailState {
    NotGenerated,
    Generating,
    Ready,
    Failed,
}

#[allow(dead_code)] // Library component data model — error_message not yet displayed in UI
pub struct LibraryComponent {
    pub name:               String,
    pub file_path:          PathBuf,
    pub metadata:           ComponentMetadata,
    pub source:             String,
    pub exported_functions: Vec<FunctionSignature>,
    pub thumbnail_state:    ThumbnailState,
    /// Raw RGBA pixels (128x128x4) once generated.
    pub thumbnail_pixels:   Option<Vec<u8>>,
    /// Channel receiver for background thumbnail generation.
    pub thumbnail_receiver: Option<Receiver<Result<Vec<u8>, String>>>,
    /// True if the component had a parse/eval error.
    pub has_error:          bool,
    pub error_message:      Option<String>,
}

impl LibraryComponent {
    fn from_file(path: PathBuf) -> Option<Self> {
        let source = std::fs::read_to_string(&path).ok()?;
        let stem   = path.file_stem()?.to_string_lossy().to_string();
        let meta   = ComponentMetadata::parse(&source);
        let name   = meta.name.clone().unwrap_or_else(|| stem.replace('_', " ").replace('-', " "));
        let fns    = extract_function_signatures(&source);
        Some(LibraryComponent {
            name,
            file_path: path,
            metadata: meta,
            source,
            exported_functions: fns,
            thumbnail_state: ThumbnailState::NotGenerated,
            thumbnail_pixels: None,
            thumbnail_receiver: None,
            has_error: false,
            error_message: None,
        })
    }

    /// The module name used in Rhai: the file stem (e.g. "servo_mount").
    pub fn module_name(&self) -> String {
        self.file_path
            .file_stem()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_else(|| self.name.replace(' ', "_").to_lowercase())
    }

    /// Poll the thumbnail receiver; returns true if state changed.
    pub fn poll_thumbnail(&mut self) -> bool {
        if self.thumbnail_state != ThumbnailState::Generating {
            return false;
        }
        let receiver = match self.thumbnail_receiver.as_ref() {
            Some(r) => r,
            None    => { self.thumbnail_state = ThumbnailState::Failed; return true; }
        };
        use std::sync::mpsc::TryRecvError;
        match receiver.try_recv() {
            Ok(Ok(pixels)) => {
                self.thumbnail_pixels  = Some(pixels);
                self.thumbnail_state   = ThumbnailState::Ready;
                self.thumbnail_receiver = None;
                true
            }
            Ok(Err(e)) => {
                eprintln!("Thumbnail generation failed: {}", e);
                self.thumbnail_state   = ThumbnailState::Failed;
                self.thumbnail_receiver = None;
                true
            }
            Err(TryRecvError::Empty)        => false,
            Err(TryRecvError::Disconnected) => {
                self.thumbnail_state   = ThumbnailState::Failed;
                self.thumbnail_receiver = None;
                true
            }
        }
    }
}

pub struct LibraryManager {
    pub lib_dir:    PathBuf,
    pub components: Vec<LibraryComponent>,
}

impl LibraryManager {
    pub fn new(lib_dir: PathBuf) -> Self {
        LibraryManager { lib_dir, components: Vec::new() }
    }

    /// Scan lib_dir and reload all .rhai files.
    pub fn scan(&mut self) {
        let dir = &self.lib_dir;
        if !dir.exists() {
            self.components.clear();
            return;
        }
        let entries = match std::fs::read_dir(dir) {
            Ok(e) => e,
            Err(_) => { self.components.clear(); return; }
        };

        let mut new_components = Vec::new();
        for entry in entries.flatten() {
            let path = entry.path();
            if path.extension().and_then(|e| e.to_str()) == Some("rhai") {
                if let Some(comp) = LibraryComponent::from_file(path) {
                    new_components.push(comp);
                }
            }
        }
        // Sort by name for stable ordering
        new_components.sort_by(|a, b| a.name.cmp(&b.name));
        self.components = new_components;
    }

    /// Reload a single component file (after editing).
    #[allow(dead_code)] // Available for hot-reload in file-watcher integration
    pub fn reload_component(&mut self, file_path: &Path) {
        if let Some(idx) = self.components.iter().position(|c| c.file_path == file_path) {
            if let Some(comp) = LibraryComponent::from_file(file_path.to_path_buf()) {
                self.components[idx] = comp;
            }
        }
    }

    /// Returns (module_name, source) pairs for all valid components, for passing to the Rhai engine.
    pub fn module_sources(&self) -> Vec<(String, String)> {
        self.components.iter()
            .filter(|c| !c.has_error)
            .map(|c| (c.module_name(), c.source.clone()))
            .collect()
    }

    /// Create a new library component file with a template and return its path.
    pub fn create_new_component(&mut self, name: &str) -> std::io::Result<PathBuf> {
        let stem = name.to_lowercase().replace(' ', "_");
        let path = self.lib_dir.join(format!("{}.rhai", stem));
        let template = format!(
            "//! name: {}\n//! description: \n//! author: \n//! version: 1.0\n//! tags: \n//! preview_fn: build()\n\nfn build() {{\n    sphere(5.0)\n}}\n",
            name
        );
        if !self.lib_dir.exists() {
            std::fs::create_dir_all(&self.lib_dir)?;
        }
        std::fs::write(&path, &template)?;
        self.scan();
        Ok(path)
    }
}
