//! Opt-in bounded aggregate diagnostics, never geometry authority.
use serde::Serialize;
use std::collections::BTreeMap;
use std::sync::{
    Mutex,
    atomic::{AtomicBool, AtomicUsize, Ordering},
};
use std::time::Instant;
static ENABLED: AtomicBool = AtomicBool::new(false);
static DETAIL_LEFT: AtomicUsize = AtomicUsize::new(0);
static DATA: Mutex<BTreeMap<&'static str, Metric>> = Mutex::new(BTreeMap::new());
#[derive(Default, Clone, Serialize)]
pub struct Metric {
    pub count: u64,
    pub ns: u64,
}
pub fn enabled() -> bool {
    ENABLED.load(Ordering::Relaxed)
}
pub fn reset(enable: bool) {
    DATA.lock().unwrap().clear();
    DETAIL_LEFT.store(if enable { 16 } else { 0 }, Ordering::Relaxed);
    ENABLED.store(enable, Ordering::Relaxed);
}
pub(super) fn detailed() -> bool {
    DETAIL_LEFT
        .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |v| v.checked_sub(1))
        .is_ok()
}
pub fn snapshot() -> BTreeMap<&'static str, Metric> {
    DATA.lock().unwrap().clone()
}
pub fn record(name: &'static str, count: u64, ns: u64) {
    let mut data = DATA.lock().unwrap();
    let m = data.entry(name).or_default();
    m.count += count;
    m.ns += ns;
}
pub fn start() -> Option<Instant> {
    enabled().then(Instant::now)
}
pub fn end(name: &'static str, t: Option<Instant>) {
    if let Some(t) = t {
        record(name, 1, t.elapsed().as_nanos() as u64);
    }
}
#[derive(Default)]
pub(super) struct Query {
    pub seeds: u64,
    pub updates: u64,
    pub ties: u64,
    pub nodes: u64,
    pub roots: u64,
    pub splits: u64,
    pub contains_visits: u64,
    pub crossings: u64,
    pub bisects: u64,
    pub vectors: u64,
    pub grows: u64,
}
impl Query {
    pub fn publish(&self) {
        for (name, n) in [
            ("seed_evaluations", self.seeds),
            ("candidate_updates", self.updates),
            ("ties", self.ties),
            ("solver_nodes", self.nodes),
            ("root_solver_calls", self.roots),
            ("subdivisions", self.splits),
            ("contains_span_visits", self.contains_visits),
            ("crossing_intervals", self.crossings),
            ("bisection_iterations", self.bisects),
            ("temporary_vectors", self.vectors),
            ("vector_growth_events", self.grows),
        ] {
            record(name, n, 0);
        }
    }
}
