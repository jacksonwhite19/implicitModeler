// CalculiX binary locator and settings.
//
// Looks for the bundled `ccx` binary relative to the executable.  A settings
// override can point to any installed copy.

use std::path::{Path, PathBuf};

/// Find the CalculiX binary.
///
/// Search order:
///   1. `override_path` if provided and the file exists.
///   2. `assets/calculix/ccx[.exe]` relative to the running executable.
///   3. `ccx` on the system PATH (via `which`/`where`).
pub fn find_calculix(override_path: Option<&str>) -> Option<PathBuf> {
    // 1. User override
    if let Some(p) = override_path {
        let pb = PathBuf::from(p);
        if pb.exists() {
            return Some(pb);
        }
    }

    // 2. Bundled binary next to executable
    if let Ok(exe) = std::env::current_exe() {
        if let Some(exe_dir) = exe.parent() {
            let bundled = bundled_path(exe_dir);
            if bundled.exists() {
                return Some(bundled);
            }
        }
    }

    // 3. System PATH
    find_on_path()
}

fn bundled_path(exe_dir: &Path) -> PathBuf {
    let binary = if cfg!(windows) { "ccx.exe" } else { "ccx" };
    exe_dir.join("assets").join("calculix").join(binary)
}

fn find_on_path() -> Option<PathBuf> {
    let binary = if cfg!(windows) { "ccx.exe" } else { "ccx" };
    if let Ok(path_var) = std::env::var("PATH") {
        let sep = if cfg!(windows) { ';' } else { ':' };
        for dir in path_var.split(sep) {
            let candidate = Path::new(dir).join(binary);
            if candidate.exists() {
                return Some(candidate);
            }
        }
    }
    None
}

/// Verify the binary is present and log a warning if not.
/// Called once at startup.
pub fn verify_calculix(override_path: Option<&str>) {
    match find_calculix(override_path) {
        Some(p) => eprintln!("[FEA] CalculiX found: {}", p.display()),
        None    => eprintln!(
            "[FEA] WARNING: CalculiX (ccx) not found. \
             Place it in assets/calculix/ or set a path override in settings."
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_find_nonexistent_override() {
        // A path that does not exist should not be returned as override
        let result = find_calculix(Some("/nonexistent/path/ccx"));
        // May find system ccx or bundled, but should not return the nonexistent override
        if let Some(p) = result {
            assert_ne!(p.to_str().unwrap(), "/nonexistent/path/ccx");
        }
    }
}
