// Application-level settings — persisted across sessions in the user's config dir.
//
// The settings file is stored at:
//   <config_dir>/implicit-cad/settings.json
// where <config_dir> is the platform config directory
// (e.g. %APPDATA% on Windows, ~/.config on Linux).

use serde::{Deserialize, Serialize};
use std::path::PathBuf;

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct AppSettings {
    /// Optional path to the CalculiX `ccx` binary.
    /// When `None`, the bundled binary and system PATH are searched automatically.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ccx_path: Option<String>,
}

impl AppSettings {
    fn settings_path() -> PathBuf {
        // Use the executable's directory as a fallback when the OS config dir is unavailable.
        let base = std::env::var("APPDATA")
            .map(PathBuf::from)
            .or_else(|_| std::env::var("HOME").map(|h| PathBuf::from(h).join(".config")))
            .unwrap_or_else(|_| std::env::current_exe()
                .ok()
                .and_then(|p| p.parent().map(|d| d.to_path_buf()))
                .unwrap_or_else(|| PathBuf::from(".")));
        base.join("implicit-cad").join("settings.json")
    }

    pub fn load() -> Self {
        let path = Self::settings_path();
        if let Ok(data) = std::fs::read_to_string(&path) {
            serde_json::from_str(&data).unwrap_or_default()
        } else {
            Self::default()
        }
    }

    pub fn save(&self) {
        let path = Self::settings_path();
        if let Some(parent) = path.parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        if let Ok(json) = serde_json::to_string_pretty(self) {
            let _ = std::fs::write(&path, json);
        }
    }
}
