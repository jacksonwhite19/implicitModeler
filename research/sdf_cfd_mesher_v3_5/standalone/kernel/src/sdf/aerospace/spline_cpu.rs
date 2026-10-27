//! Exact derived CPU setup. Canonical public spline storage stays authoritative.
use super::NacaSpline;
use glam::DVec2;
use std::{
    cell::RefCell,
    sync::{
        Arc, Mutex, OnceLock,
        atomic::{AtomicU8, Ordering},
    },
};
static MODE: OnceLock<AtomicU8> = OnceLock::new();
pub fn mode() -> u8 {
    MODE.get_or_init(|| {
        AtomicU8::new(
            std::env::var("IMPLICIT_SPLINE_CPU_MODE")
                .ok()
                .and_then(|v| v.parse().ok())
                .filter(|v| *v <= 4)
                .unwrap_or(4),
        )
    })
    .load(Ordering::Relaxed)
}
/// Research A/B switch. Set only while no geometry evaluations are active.
pub fn set_mode(mode: u8) {
    self::mode();
    MODE.get().unwrap().store(mode, Ordering::Relaxed);
}
type Pair = (Arc<NacaSpline>, Arc<NacaSpline>, bool);
thread_local! {static PAIR:RefCell<Option<Pair>>=const {RefCell::new(None)};}
pub fn profiles_equal(a: &Arc<NacaSpline>, b: &Arc<NacaSpline>) -> bool {
    if Arc::ptr_eq(a, b) {
        return true;
    }
    if a.spans.len() > 4096 || b.spans.len() > 4096 {
        return a == b;
    }
    PAIR.with(|cache| {
        let mut cache = cache.borrow_mut();
        if let Some((x, y, equal)) = cache.as_ref() {
            if (Arc::ptr_eq(a, x) && Arc::ptr_eq(b, y)) || (Arc::ptr_eq(a, y) && Arc::ptr_eq(b, x))
            {
                return *equal;
            }
        }
        if super::profile::enabled() {
            super::profile::record("actual_value_comparisons", 1, 0);
        }
        let equal = a == b;
        // Retain both immutable allocations. Arc::make_mut must detach before
        // any safe mutation; address recycling cannot cause a false cache hit.
        *cache = Some((a.clone(), b.clone(), equal));
        equal
    })
}

pub struct Prepared {
    pub(super) source: Arc<NacaSpline>,
    pub(super) seeds: Vec<[DVec2; 3]>,
    pub(super) bounds: Vec<(DVec2, DVec2)>,
    pub setup_ms: f64,
}
impl Prepared {
    pub fn new(source: Arc<NacaSpline>) -> Option<Self> {
        if source.spans.is_empty()
            || source.spans.len() > 4096
            || source
                .spans
                .iter()
                .any(|s| s.control.iter().flatten().any(|v| !v.is_finite()))
        {
            return None;
        }
        let t = std::time::Instant::now();
        let seeds = source
            .spans
            .iter()
            .map(|s| [0., 0.5, 1.].map(|t| s.evaluate(t).0))
            .collect();
        let bounds = source.spans.iter().map(|s| s.bounds()).collect();
        Some(Self {
            source,
            seeds,
            bounds,
            setup_ms: t.elapsed().as_secs_f64() * 1000.,
        })
    }
    pub fn bytes(&self) -> usize {
        self.seeds.capacity() * std::mem::size_of::<[DVec2; 3]>()
            + self.bounds.capacity() * std::mem::size_of::<(DVec2, DVec2)>()
    }
    pub fn closest_point(
        &self,
        q: DVec2,
    ) -> Result<super::SectionClosestPoint, super::SectionQueryError> {
        self.source
            .closest_prepared(q, super::QUERY_BUDGET, true, Some(self))
    }
}
static PREPARED: Mutex<Vec<Arc<Prepared>>> = Mutex::new(Vec::new());
thread_local! {static HOT:RefCell<Vec<Arc<Prepared>>>=const {RefCell::new(Vec::new())};}
pub fn prepared(source: &Arc<NacaSpline>) -> Option<Arc<Prepared>> {
    HOT.with(|hot| {
        let mut hot = hot.borrow_mut();
        if let Some(p) = hot.iter().find(|p| Arc::ptr_eq(&p.source, source)) {
            return Some(p.clone());
        }
        let mut shared = PREPARED.lock().unwrap_or_else(|e| e.into_inner());
        let p = if let Some(p) = shared.iter().find(|p| Arc::ptr_eq(&p.source, source)) {
            p.clone()
        } else {
            let p = Arc::new(Prepared::new(source.clone())?);
            if shared.len() == 4 {
                shared.remove(0);
            }
            shared.push(p.clone());
            p
        };
        if hot.len() == 2 {
            hot.remove(0);
        }
        hot.push(p.clone());
        Some(p)
    })
}
pub fn closest_shared(
    source: &Arc<NacaSpline>,
    q: DVec2,
) -> Result<super::SectionClosestPoint, super::SectionQueryError> {
    match prepared(source) {
        Some(p) => p.closest_point(q),
        None => source.closest_point(q),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn pair_identity_and_mutation_are_safe() {
        let a = Arc::new(NacaSpline {
            spans: vec![super::super::CubicSpan {
                control: [[0., 0.]; 4],
            }],
        });
        let mut b = Arc::new((*a).clone());
        assert!(!Arc::ptr_eq(&a, &b));
        assert!(profiles_equal(&a, &b));
        Arc::make_mut(&mut b).spans[0].control[0][0] = 1.;
        assert!(!profiles_equal(&a, &b));
        assert!(profiles_equal(&b, &b));
        let mut nan = (*a).clone();
        nan.spans[0].control[0][0] = f64::NAN;
        let nan = Arc::new(nan);
        let other = Arc::new((*nan).clone());
        assert!(profiles_equal(&nan, &nan));
        assert!(!profiles_equal(&nan, &other));
    }
    #[test]
    fn prepared_sharing_mutation_and_serialization() {
        let mut a = Arc::new(NacaSpline::new(0., 0., 0.12, [1., 0.00126], [1., -0.00126]).unwrap());
        let before = serde_json::to_string(a.as_ref()).unwrap();
        let p = prepared(&a).unwrap();
        let q = prepared(&a).unwrap();
        assert!(Arc::ptr_eq(&p, &q));
        assert_eq!(before, serde_json::to_string(a.as_ref()).unwrap());
        Arc::make_mut(&mut a).spans[0].control[0][1] += 0.001;
        let fresh = prepared(&a).unwrap();
        assert!(!Arc::ptr_eq(&p, &fresh));
        assert_eq!(p.seeds[0][0].y.to_bits(), 0.00126f64.to_bits());
        assert_eq!(
            fresh.seeds[0][0].y.to_bits(),
            a.spans[0].evaluate(0.).0.y.to_bits()
        );
    }
}
