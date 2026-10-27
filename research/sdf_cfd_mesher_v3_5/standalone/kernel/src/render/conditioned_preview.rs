//! Bounded, revision-tagged conditioning and sampled-preview scheduling.
//! Geometry functions are unchanged; no sampling occurs while holding the mailbox lock.
use super::{SdfGrid, telemetry};
use super::viewport_grid::{ViewportGridSpec,ViewportSdfGrid};
use crate::sdf::{Sdf, conditioning::*};
use std::sync::{Arc, Condvar, Mutex};
use std::time::Instant;

/// The App's existing script job counter, also advanced on model clearing.
pub type ModelRevision = u64;

pub struct PreviewRequest {
    pub revision: ModelRevision,
    pub previous: Option<Arc<dyn Sdf>>,
    pub canonical: Arc<dyn Sdf>,
    pub spec: ViewportGridSpec,
    pub build_grid: bool,
    /// Used only when a retained direct preview switches to sampled display.
    pub already_conditioned: bool,
}

pub struct CompletedConditionedPreview {
    pub revision: ModelRevision,
    pub sdf: Arc<dyn Sdf>,
    pub grid: Option<Arc<ViewportSdfGrid>>,
    pub summary: ConditioningJobSummary,
    pub grid_ms: f64,
}

pub struct PreviewResponse {
    pub revision: ModelRevision,
    pub outcome: Result<CompletedConditionedPreview, String>,
}

#[derive(Default)]
struct Mailbox {
    current: ModelRevision,
    stopped: bool,
    pending: Option<PreviewRequest>,
    completed: Option<PreviewResponse>,
    summary: Option<ConditioningJobSummary>,
    active: Option<(ModelRevision, ConstructionCancellation)>,
}
type Shared = Arc<(Mutex<Mailbox>, Condvar)>;

pub struct ConditionedPreviewWorker {
    shared: Shared,
}

fn is_current(shared: &Shared, revision: ModelRevision) -> bool {
    let state = shared.0.lock().unwrap_or_else(|e| e.into_inner());
    !state.stopped && state.current == revision
}

/// Exact former UI grid operation. Also used by the numerical-equivalence probe.
pub fn build_conditioned_grid(sdf: &Arc<dyn Sdf>, resolution: u32) -> Arc<SdfGrid> {
    let (lo, hi) = crate::pipeline::auto_bounds(sdf.as_ref());
    crate::pipeline::compute_viewport_sdf_grid_cached_arc(sdf, lo, hi, resolution)
}

// Restrict the measured optimization to deterministic spline wings and their
// translation wrappers. Other/custom fields retain their existing query path.
fn exact_coarse_reuse_candidate(mut sdf: &dyn Sdf) -> bool {
    while let Some(t) = sdf.as_any().downcast_ref::<crate::sdf::transforms::Translate>() {
        sdf = t.child.as_ref();
    }
    let Some(wing) = sdf.as_any().downcast_ref::<crate::sdf::aerospace::wing::LoftedWing>() else { return false; };
    !wing.sections.is_empty() && wing.sections.iter().all(|section| {
        section.section.as_any().downcast_ref::<crate::sdf::aerospace::airfoil::Airfoil>()
            .is_some_and(|airfoil| airfoil.spline().is_some())
    })
}

fn build(request: PreviewRequest, shared: &Shared) -> Result<CompletedConditionedPreview, String> {
    let revision = request.revision;
    telemetry::lifecycle("conditioning_started", revision);
    let start = Instant::now();
    let sdf = if request.already_conditioned {
        request.canonical
    } else {
        // Same function, previous-SDF snapshot, and default budget as the original scheduler.
        let control=shared.0.lock().unwrap_or_else(|e|e.into_inner()).active.as_ref().expect("active request").1.clone();
        let result=condition_sdf_after_backend_edit_cancellable(
            request.previous.as_ref(),
            request.canonical,
            BackgroundConditioningSchedulerConfig::default().max_initial_samples,
            control,
        );
        if result.is_err() { telemetry::lifecycle("conditioning_cancelled_ended",revision); }
        result.map_err(|_|"conditioning cooperatively cancelled".to_owned())?
    };
    let summary = ConditioningJobSummary::ready_from_sdf(
        ConditioningJobId(revision),
        &sdf,
        start.elapsed().as_millis(),
    );
    telemetry::lifecycle("conditioning_finished", revision);
    if !is_current(shared, revision) {
        telemetry::lifecycle("stale_before_grid", revision);
        return Err("superseded before grid generation".into());
    }
    let start = Instant::now();
    let grid = if request.build_grid {
        telemetry::lifecycle("conditioned_grid_started_worker", revision);
        let (lo,hi)=crate::pipeline::auto_bounds(sdf.as_ref());
        let grid = super::viewport_grid::sample_cached(&sdf,lo,hi,request.spec);
        telemetry::lifecycle("conditioned_grid_finished", revision);
        if grid.data.iter().any(|x| !x.is_finite()) {
            return Err("conditioned preview contains nonfinite field samples".into());
        }
        Some(grid)
    } else {
        None
    };
    Ok(CompletedConditionedPreview {
        revision,
        sdf,
        grid,
        summary,
        grid_ms: start.elapsed().as_secs_f64() * 1000.,
    })
}

impl ConditionedPreviewWorker {
    pub fn new() -> Self {
        Self::with_builder(build)
    }

    fn with_builder(
        builder: impl Fn(PreviewRequest, &Shared) -> Result<CompletedConditionedPreview, String>
        + Send
        + 'static,
    ) -> Self {
        let shared: Shared = Arc::new((Mutex::new(Mailbox::default()), Condvar::new()));
        let worker = shared.clone();
        // Intentionally detached: closing the App never joins an expensive field query.
        // Drop stops publication and pending work; an active numerical call may finish.
        std::thread::Builder::new()
            .name("conditioned-preview".into())
            .spawn(move || {
                loop {
                    let request = {
                        let mut state = worker.0.lock().unwrap_or_else(|e| e.into_inner());
                        while !state.stopped && state.pending.is_none() {
                            state = worker.1.wait(state).unwrap_or_else(|e| e.into_inner());
                        }
                        if state.stopped {
                            return;
                        }
                        let request = state.pending.take().unwrap();
                        let revision=request.revision;
                        let mut control=ConstructionCancellation::default();
                        if exact_coarse_reuse_candidate(request.canonical.as_ref()) {
                            control=control.with_exact_coarse_reuse();
                            eprintln!("CONSTRUCTION_EXACT_COARSE_REUSE revision={revision}");
                        }
                        let control=control.with_observer(move |phase| {
                            eprintln!("CONDITIONING_CANCEL_PHASE revision={revision} phase={phase}");
                            telemetry::lifecycle("conditioning_cancel_observed",revision);
                        });
                        state.active=Some((revision,control));
                        if let Some(summary) = &mut state.summary {
                            summary.state = ConditioningJobState::Running;
                        }
                        request
                    };
                    let revision = request.revision;
                    if !is_current(&worker, revision) {
                        worker.0.lock().unwrap_or_else(|e|e.into_inner()).active=None;
                        continue;
                    }
                    let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        builder(request, &worker)
                    }))
                    .unwrap_or_else(|payload| {
                        let message = payload
                            .downcast_ref::<String>()
                            .cloned()
                            .or_else(|| payload.downcast_ref::<&str>().map(|s| s.to_string()))
                            .unwrap_or_else(|| "non-string panic".into());
                        Err(format!("conditioned preview worker panicked: {message}"))
                    });
                    if let Err(error) = &outcome {
                        eprintln!("CONDITIONED_PREVIEW_RESULT revision={revision}: {error}");
                    }
                    let mut state = worker.0.lock().unwrap_or_else(|e| e.into_inner());
                    state.active=None;
                    if state.stopped || state.current != revision {
                        telemetry::lifecycle("stale_result_discarded", revision);
                        continue;
                    }
                    match &outcome {
                        Ok(result) => state.summary = Some(result.summary.clone()),
                        Err(error) => {
                            if let Some(summary) = &mut state.summary {
                                summary.state = ConditioningJobState::Failed;
                                summary.error = Some(error.clone());
                            }
                        }
                    }
                    state.completed = Some(PreviewResponse { revision, outcome });
                    telemetry::lifecycle("conditioned_result_ready", revision);
                }
            })
            .expect("spawn conditioned-preview worker");
        Self { shared }
    }

    /// Called at evaluation start, including empty/new model transitions.
    pub fn invalidate(&self, revision: ModelRevision) {
        let mut state = self.shared.0.lock().unwrap_or_else(|e| e.into_inner());
        assert!(revision > state.current, "model revisions must increase");
        state.current = revision;
        if let Some((active,control))=&state.active {
            telemetry::lifecycle("conditioning_cancel_requested",*active);
            control.cancel();
        }
        if let Some(old) = state.pending.take() {
            telemetry::lifecycle("pending_coalesced", old.revision);
        }
        if let Some(old) = state.completed.take() {
            telemetry::lifecycle("stale_completed_discarded", old.revision);
        }
        state.summary = None;
    }

    pub fn enqueue(&self, request: PreviewRequest) {
        let summary = ConditioningJobSummary::from_metadata(
            ConditioningJobId(request.revision),
            ConditioningJobState::Queued,
            &request.canonical.metadata(),
        );
        let mut state = self.shared.0.lock().unwrap_or_else(|e| e.into_inner());
        if state.stopped || state.current != request.revision {
            return;
        }
        if let Some(old) = state.pending.replace(request) {
            telemetry::lifecycle("pending_coalesced", old.revision);
        }
        state.summary = Some(summary);
        self.shared.1.notify_one();
    }

    pub fn take_ready(&self) -> Option<PreviewResponse> {
        self.shared
            .0
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .completed
            .take()
    }
    pub fn job_summary(&self, id: ConditioningJobId) -> Option<ConditioningJobSummary> {
        self.shared
            .0
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .summary
            .clone()
            .filter(|s| s.id == id)
    }
}
impl Drop for ConditionedPreviewWorker {
    fn drop(&mut self) {
        let mut state = self.shared.0.lock().unwrap_or_else(|e| e.into_inner());
        state.stopped = true;
        if let Some((active,control))=&state.active {
            telemetry::lifecycle("conditioning_cancel_requested_shutdown",*active);
            control.cancel();
        }
        state.pending = None;
        state.completed = None;
        self.shared.1.notify_one();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;
    use std::time::Duration;

    fn request(revision: u64) -> PreviewRequest {
        PreviewRequest {
            revision,
            previous: None,
            canonical: Arc::new(crate::sdf::primitives::Sphere::new(2.)),
            spec: ViewportGridSpec::cube(8),
            build_grid: false,
            already_conditioned: true,
        }
    }
    fn completed(request: PreviewRequest) -> CompletedConditionedPreview {
        CompletedConditionedPreview {
            revision: request.revision,
            summary: ConditioningJobSummary::ready_from_sdf(
                ConditioningJobId(request.revision),
                &request.canonical,
                0,
            ),
            sdf: request.canonical,
            grid: None,
            grid_ms: 0.,
        }
    }
    fn ready(worker: &ConditionedPreviewWorker) -> PreviewResponse {
        let t = Instant::now();
        loop {
            if let Some(result) = worker.take_ready() {
                return result;
            }
            assert!(t.elapsed() < Duration::from_secs(5));
            std::thread::yield_now();
        }
    }
    #[test]
    fn rectangular_mode_switches_and_stale_grid_cannot_publish() {
        let (started_tx,started_rx)=mpsc::channel();let (release_tx,release_rx)=mpsc::channel();
        let worker=ConditionedPreviewWorker::with_builder(move|r,_|{
            if r.revision==2 {started_tx.send(()).unwrap();release_rx.recv().unwrap();}
            let spec=r.spec;
            let grid=Arc::new(super::super::viewport_grid::sample_cpu(r.canonical.as_ref(),glam::Vec3::splat(-3.),glam::Vec3::splat(3.),spec));
            let mut result=completed(r);result.grid=Some(grid);Ok(result)
        });
        let enqueue=|rev,spec|{worker.invalidate(rev);let mut r=request(rev);r.spec=spec;worker.enqueue(r);};
        enqueue(1,ViewportGridSpec::cube(96));assert_eq!(ready(&worker).outcome.unwrap().grid.unwrap().spec,ViewportGridSpec::cube(96));
        enqueue(2,ViewportGridSpec::EXPERIMENTAL_WING);started_rx.recv().unwrap();
        enqueue(3,ViewportGridSpec::cube(192));release_tx.send(()).unwrap();
        let latest=ready(&worker).outcome.unwrap();assert_eq!(latest.revision,3);assert_eq!(latest.grid.unwrap().spec,ViewportGridSpec::cube(192));
        for (rev,spec)in[(4,ViewportGridSpec::EXPERIMENTAL_WING),(5,ViewportGridSpec::EXPERIMENTAL_WING),(6,ViewportGridSpec::cube(96))]{
            enqueue(rev,spec);let r=ready(&worker).outcome.unwrap();assert_eq!(r.revision,rev);assert_eq!(r.grid.unwrap().spec,spec);
        }
        worker.invalidate(7);assert!(worker.take_ready().is_none()); // File > New, no request.
        let mut r=request(8);r.spec=ViewportGridSpec::EXPERIMENTAL_WING;worker.invalidate(8);worker.enqueue(r);
        assert_eq!(ready(&worker).outcome.unwrap().grid.unwrap().spec,ViewportGridSpec::EXPERIMENTAL_WING);
    }
    #[test]
    fn active_stale_and_pending_are_bounded_and_latest_wins() {
        let (started_tx, started_rx) = mpsc::channel();
        let (release_tx, release_rx) = mpsc::channel();
        let worker = ConditionedPreviewWorker::with_builder(move |r, _| {
            started_tx.send(r.revision).unwrap();
            if r.revision == 1 {
                release_rx.recv().unwrap();
            }
            Ok(completed(r))
        });
        worker.invalidate(1);
        worker.enqueue(request(1));
        assert_eq!(started_rx.recv_timeout(Duration::from_secs(5)).unwrap(), 1);
        for revision in 2..=50 {
            worker.invalidate(revision);
            worker.enqueue(request(revision));
            let state = worker.shared.0.lock().unwrap();
            assert_eq!(state.pending.as_ref().unwrap().revision, revision);
            assert!(state.completed.is_none());
        }
        release_tx.send(()).unwrap();
        assert_eq!(started_rx.recv_timeout(Duration::from_secs(5)).unwrap(), 50);
        assert_eq!(ready(&worker).outcome.unwrap().revision, 50);
        assert!(started_rx.try_recv().is_err());
    }
    #[test]
    fn completed_old_result_is_removed_on_new_or_empty_model() {
        let worker = ConditionedPreviewWorker::with_builder(|r, _| Ok(completed(r)));
        worker.invalidate(1);
        worker.enqueue(request(1));
        let t = Instant::now();
        while worker.shared.0.lock().unwrap().completed.is_none() {
            assert!(t.elapsed() < Duration::from_secs(5));
            std::thread::yield_now();
        }
        worker.invalidate(2); // Empty/New invalidates even without a replacement request.
        assert!(worker.take_ready().is_none());
        worker.enqueue(request(1)); // Late canonical result cannot queue old work.
        assert!(worker.shared.0.lock().unwrap().pending.is_none());
    }
    #[test]
    fn closing_does_not_wait_or_publish_into_reopened_worker() {
        let (started_tx, started_rx) = mpsc::channel();
        let (release_tx, release_rx) = mpsc::channel();
        let old = ConditionedPreviewWorker::with_builder(move |r, _| {
            started_tx.send(()).unwrap();
            release_rx.recv().unwrap();
            Ok(completed(r))
        });
        old.invalidate(1);
        old.enqueue(request(1));
        started_rx.recv().unwrap();
        let old_shared = old.shared.clone();
        // Drop must return while the builder is still blocked, with no timing tolerance.
        drop(old);
        let new = ConditionedPreviewWorker::with_builder(|r, _| Ok(completed(r)));
        new.invalidate(1);
        new.enqueue(request(1));
        assert_eq!(ready(&new).revision, 1);
        release_tx.send(()).unwrap();
        assert!(old_shared.0.lock().unwrap().stopped);
        assert!(new.take_ready().is_none());
    }
    #[test]
    fn errors_and_panics_are_delivered_and_worker_survives() {
        let worker = ConditionedPreviewWorker::with_builder(|r, _| match r.revision {
            1 => Err("injected error".into()),
            2 => panic!("injected panic"),
            _ => Ok(completed(r)),
        });
        for revision in 1..=3 {
            worker.invalidate(revision);
            worker.enqueue(request(revision));
            let response = ready(&worker);
            assert_eq!(response.revision, revision);
            assert_eq!(response.outcome.is_ok(), revision == 3);
        }
    }

    #[test]
    fn cooperative_construction_cancels_before_grid_and_next_request_runs() {
        for phase in ["before_build","coarse_sampling","adaptive_start","before_publish"] {
            let (entered_tx,entered_rx)=mpsc::channel();
            let (release_tx,release_rx)=mpsc::channel();
            let release=Arc::new(Mutex::new(release_rx));
            let grids=Arc::new(Mutex::new(Vec::new()));let tracked=grids.clone();
            let worker=ConditionedPreviewWorker::with_builder(move |r,shared| {
                let control=shared.0.lock().unwrap().active.as_ref().unwrap().1.clone();
                let control=if r.revision==1 {
                    let tx=entered_tx.clone();let release=release.clone();
                    let once=std::sync::atomic::AtomicBool::new(false);
                    control.with_checkpoint_hook(move |p| {
                        if p==phase&&!once.swap(true,std::sync::atomic::Ordering::Relaxed) {
                            tx.send(()).unwrap();release.lock().unwrap().recv().unwrap();
                        }
                    })
                } else { control };
                condition_sdf_after_backend_edit_cancellable(None,r.canonical.clone(),350_000,control)
                    .map_err(|_|"cancelled".to_string())?;
                tracked.lock().unwrap().push(r.revision); // Would start grid only on success.
                Ok(completed(r))
            });
            worker.invalidate(1);worker.enqueue(request(1));
            entered_rx.recv_timeout(Duration::from_secs(5)).unwrap();
            worker.invalidate(2);worker.enqueue(request(2));
            release_tx.send(()).unwrap();
            assert_eq!(ready(&worker).outcome.unwrap().revision,2);
            assert_eq!(*grids.lock().unwrap(),vec![2]);
        }
    }

    #[test]
    fn shutdown_and_empty_model_cancel_active_signal_and_stale_errors() {
        for shutdown in [false,true] {
            let (tx,rx)=mpsc::channel();let (end_tx,end_rx)=mpsc::channel();
            let worker=ConditionedPreviewWorker::with_builder(move |_,shared| {
                let token=shared.0.lock().unwrap().active.as_ref().unwrap().1.clone();
                tx.send(token.clone()).unwrap();
                // A construction entry returns Cancelled once the same signal is cancelled.
                loop {
                    let result=condition_sdf_after_backend_edit_cancellable(None,Arc::new(crate::sdf::primitives::Sphere::new(1.)),350_000,token.clone());
                    if result.is_err(){break;}
                    std::thread::yield_now();
                }
                end_tx.send(()).unwrap();Err("error concurrent with cancellation".into())
            });
            worker.invalidate(1);worker.enqueue(request(1));rx.recv_timeout(Duration::from_secs(5)).unwrap();
            let shared=worker.shared.clone();
            if shutdown {drop(worker);} else {worker.invalidate(2);drop(worker);}
            end_rx.recv_timeout(Duration::from_secs(5)).unwrap();
            assert!(shared.0.lock().unwrap().completed.is_none());
        }
    }
}
