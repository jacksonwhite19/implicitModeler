use std::fs;
use std::path::PathBuf;
use std::time::Instant;

fn main() -> Result<(), String> {
    let script_path = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("usage: current-root-check <script.rhai>")?,
    );
    let source = fs::read_to_string(&script_path).map_err(|e| format!("read script: {e}"))?;
    let started = Instant::now();
    let execution = match implicit_cad::scripting::evaluate_script(&source) {
        Ok(_) => "SUCCESS".to_string(),
        Err(e) => format!("ERROR: {e}"),
    };
    let elapsed_seconds = started.elapsed().as_secs_f64();
    println!(
        "{}",
        serde_json::json!({
            "execution": execution,
            "elapsed_seconds": elapsed_seconds
        })
    );
    Ok(())
}
