//! Opt-in viewport timing. CPU durations are not GPU timestamp durations.
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Mutex, OnceLock};
use std::time::Instant;

static CONDITIONED_GRID_STARTED: AtomicU64 = AtomicU64::new(0);
pub fn latest_conditioned_grid_started() -> u64 {
    CONDITIONED_GRID_STARTED.load(Ordering::Relaxed)
}

pub fn enabled() -> bool {
    static ENABLED: OnceLock<bool> = OnceLock::new();
    *ENABLED.get_or_init(|| std::env::var_os("IMPLICIT_VIEWPORT_TIMING").is_some())
}

fn events() -> &'static Mutex<Vec<(String, f64)>> {
    static EVENTS: OnceLock<Mutex<Vec<(String, f64)>>> = OnceLock::new();
    EVENTS.get_or_init(|| Mutex::new(Vec::new()))
}
pub fn record(stage: &str, ms: f64) {
    if !enabled() {
        return;
    }
    if let Ok(mut e) = events().lock() {
        if e.len() < 100_000 {
            e.push((stage.to_owned(), ms));
        }
    }
}

/// Process-relative lifecycle events for scheduling qualification.
pub fn lifecycle(stage: &str, revision: u64) {
    if !enabled() {
        return;
    }
    if stage == "conditioned_grid_started_worker" {
        CONDITIONED_GRID_STARTED.store(revision, Ordering::Relaxed);
    }
    static START: OnceLock<Instant> = OnceLock::new();
    let ms = START.get_or_init(Instant::now).elapsed().as_secs_f64() * 1000.;
    eprintln!(
        "PREVIEW_EVENT {}",
        serde_json::json!({"stage":stage,"revision":revision,"at_ms":ms,"thread":format!("{:?}",std::thread::current().id())})
    );
}
pub fn flush() {
    let batch = events()
        .lock()
        .map(|mut e| std::mem::take(&mut *e))
        .unwrap_or_default();
    for (stage, ms) in batch {
        eprintln!(
            "VIEWPORT_TIMING {}",
            serde_json::json!({"stage":stage,"ms":ms})
        );
    }
}

pub struct Span(&'static str, Instant);
impl Span {
    pub fn new(name: &'static str) -> Self {
        Self(name, Instant::now())
    }
}
impl Drop for Span {
    fn drop(&mut self) {
        record(self.0, self.1.elapsed().as_secs_f64() * 1000.0);
    }
}
