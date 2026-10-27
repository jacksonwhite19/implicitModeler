//! Viewport-only scalar storage. Production SdfGrid/GridInfo remain cubic.
//! X-fastest: x + y*Nx + z*Nx*Ny. Samples are at lo + (xyz + 0.5)*spacing.
use crate::sdf::Sdf;
use glam::Vec3;
use rayon::prelude::*;
use std::sync::{Arc, Mutex, OnceLock, Weak};

#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub struct ViewportGridSpec([u32; 3]);
impl ViewportGridSpec {
    pub const EXPERIMENTAL_WING: Self = Self([256, 192, 128]);
    pub fn new(dimensions: [u32; 3]) -> Result<Self, String> {
        if dimensions.contains(&0) {
            return Err("zero viewport dimension".into());
        }
        let count = dimensions
            .iter()
            .try_fold(1u32, |a, &b| a.checked_mul(b))
            .ok_or("viewport cell count overflow")?;
        (count as usize)
            .checked_mul(4)
            .filter(|&n| n <= isize::MAX as usize)
            .ok_or("viewport byte count overflow")?;
        Ok(Self(dimensions))
    }
    pub fn cube(n: u32) -> Self {
        Self::new([n; 3]).expect("valid viewport cube")
    }
    pub fn dimensions(self) -> [u32; 3] {
        self.0
    }
    pub fn cell_count(self) -> usize {
        self.0.iter().map(|&n| n as usize).product()
    }
    pub fn cubic_resolution(self) -> Option<u32> {
        (self.0[0] == self.0[1] && self.0[1] == self.0[2]).then_some(self.0[0])
    }
    pub fn index(self, x: u32, y: u32, z: u32) -> usize {
        assert!(x < self.0[0] && y < self.0[1] && z < self.0[2]);
        x as usize
            + y as usize * self.0[0] as usize
            + z as usize * self.0[0] as usize * self.0[1] as usize
    }
    pub fn coordinates(self, i: usize) -> [u32; 3] {
        assert!(i < self.cell_count());
        let [nx, ny, _] = self.0.map(|n| n as usize);
        [
            (i % nx) as u32,
            (i / nx % ny) as u32,
            (i / (nx * ny)) as u32,
        ]
    }
    pub fn spacing(self, lo: Vec3, hi: Vec3) -> Vec3 {
        (hi - lo) / Vec3::from_array(self.0.map(|n| n as f32))
    }
    pub fn point(self, i: usize, lo: Vec3, hi: Vec3) -> Vec3 {
        let [x, y, z] = self.coordinates(i);
        let d = self.spacing(lo, hi);
        lo + Vec3::new(
            (x as f32 + 0.5) * d.x,
            (y as f32 + 0.5) * d.y,
            (z as f32 + 0.5) * d.z,
        )
    }
}
impl std::fmt::Display for ViewportGridSpec {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}×{}×{}", self.0[0], self.0[1], self.0[2])
    }
}

pub struct ViewportSdfGrid {
    pub data: Vec<f32>,
    pub spec: ViewportGridSpec,
    pub bounds_min: Vec3,
    pub bounds_max: Vec3,
}
impl ViewportSdfGrid {
    pub fn spacing(&self) -> Vec3 {
        self.spec.spacing(self.bounds_min, self.bounds_max)
    }
    pub fn voxel_size(&self) -> f32 {
        let span = (self.bounds_max - self.bounds_min).max(Vec3::ONE);
        // Retain the exact old scalar operation for cubic previews.
        if let Some(n) = self.spec.cubic_resolution() {
            span.max_element() / n as f32
        } else {
            (span / Vec3::from_array(self.spec.0.map(|n| n as f32))).max_element()
        }
    }
    pub fn to_cubic(&self) -> Option<super::SdfGrid> {
        Some(super::SdfGrid {
            data: self.data.clone(),
            resolution: self.spec.cubic_resolution()?,
            bounds_min: self.bounds_min,
            bounds_max: self.bounds_max,
        })
    }
}
impl From<super::SdfGrid> for ViewportSdfGrid {
    fn from(g: super::SdfGrid) -> Self {
        Self {
            data: g.data,
            spec: ViewportGridSpec::cube(g.resolution),
            bounds_min: g.bounds_min,
            bounds_max: g.bounds_max,
        }
    }
}
pub fn sample_cpu(sdf: &dyn Sdf, lo: Vec3, hi: Vec3, spec: ViewportGridSpec) -> ViewportSdfGrid {
    let _timing = super::telemetry::Span::new("cpu_grid_sampling");
    let data = (0..spec.cell_count())
        .into_par_iter()
        .map(|i| sdf.distance(spec.point(i, lo, hi)))
        .collect();
    ViewportSdfGrid {
        data,
        spec,
        bounds_min: lo,
        bounds_max: hi,
    }
}

#[derive(Hash, PartialEq, Eq, Clone, Copy)]
struct CacheKey {
    id: usize,
    spec: ViewportGridSpec,
    lo: [u32; 3],
    hi: [u32; 3],
    wing: bool,
    conditioned: bool,
    cpu: bool,
}
fn key(sdf: &Arc<dyn Sdf>, lo: Vec3, hi: Vec3, spec: ViewportGridSpec) -> CacheKey {
    CacheKey {
        id: Arc::as_ptr(sdf) as *const () as usize,
        spec,
        lo: lo.to_array().map(f32::to_bits),
        hi: hi.to_array().map(f32::to_bits),
        wing: std::env::var("IMPLICIT_VIEWPORT_WING_GPU").as_deref() != Ok("0"),
        conditioned: std::env::var("IMPLICIT_VIEWPORT_CONDITIONED_GPU").as_deref() == Ok("1"),
        cpu: std::env::var("IMPLICIT_VIEWPORT_FORCE_CPU").as_deref() == Ok("1"),
    }
}
struct Entry {
    key: CacheKey,
    owner: Weak<dyn Sdf>,
    grid: Arc<ViewportSdfGrid>,
}
fn cache() -> &'static Mutex<Vec<Entry>> {
    static CACHE: OnceLock<Mutex<Vec<Entry>>> = OnceLock::new();
    CACHE.get_or_init(|| Mutex::new(Vec::new()))
}

pub fn sample_cached(
    sdf: &Arc<dyn Sdf>,
    lo: Vec3,
    hi: Vec3,
    spec: ViewportGridSpec,
) -> Arc<ViewportSdfGrid> {
    let k = key(sdf, lo, hi, spec);
    if let Ok(mut c) = cache().lock() {
        c.retain(|e| e.owner.strong_count() > 0);
        if let Some(e) = c
            .iter()
            .find(|e| e.key == k && e.owner.upgrade().is_some_and(|s| Arc::ptr_eq(&s, sdf)))
        {
            return e.grid.clone();
        }
    }
    let attempt = std::panic::catch_unwind(std::panic::AssertUnwindSafe(
        || -> Result<ViewportSdfGrid, String> {
            if k.cpu {
                return Err("forced CPU viewport evaluation".into());
            }
            if k.conditioned {
                if let Some(kernel) = crate::sdf::conditioning::conditioned_kernel_ref(sdf) {
                    let t = std::time::Instant::now();
                    let ir = crate::gpu::conditioned::ConditionedViewportIr::lower(kernel)?;
                    super::telemetry::record(
                        "conditioned_lower_and_snapshot",
                        t.elapsed().as_secs_f64() * 1000.,
                    );
                    return ir.viewport_grid(lo, hi, spec);
                }
            }
            let t = std::time::Instant::now();
            let ir = if k.wing {
                crate::gpu::wing::lower_viewport_sdf_ir(sdf.as_ref())
            } else {
                crate::gpu::lower_sdf_ir(sdf.as_ref())
            };
            super::telemetry::record("viewport_lower_sdf_ir", t.elapsed().as_secs_f64() * 1000.);
            let ir = ir.ok_or("unsupported viewport field")?;
            if crate::gpu::wing::has_wing(&ir) {
                return crate::gpu::wing::viewport_grid(&ir, lo, hi, spec);
            }
            if let Some(n) = spec.cubic_resolution() {
                return crate::gpu::compute_sdf_grid_gpu(&ir, lo, hi, n).map(Into::into);
            }
            // Generic IR retains the existing point evaluator in bounded coordinate batches.
            let mut data = Vec::with_capacity(spec.cell_count());
            for start in (0..spec.cell_count()).step_by(65536) {
                let points: Vec<_> = (start..(start + 65536).min(spec.cell_count()))
                    .map(|i| spec.point(i, lo, hi))
                    .collect();
                data.extend(crate::gpu::sample_points_gpu(&ir, &points)?);
            }
            Ok(ViewportSdfGrid {
                data,
                spec,
                bounds_min: lo,
                bounds_max: hi,
            })
        },
    ));
    let grid = match attempt {
        Ok(Ok(g)) if g.data.len() == spec.cell_count() && g.data.iter().all(|v| v.is_finite()) => {
            eprintln!("VIEWPORT_FIELD_GPU dimensions={spec}");
            g
        }
        other => {
            let reason = match other {
                Ok(Err(e)) => e,
                Ok(Ok(_)) => "invalid GPU samples".into(),
                Err(_) => "GPU operation panicked".into(),
            };
            eprintln!("VIEWPORT_FIELD_CPU_FALLBACK dimensions={spec}: {reason}");
            sample_cpu(sdf.as_ref(), lo, hi, spec)
        }
    };
    let grid = Arc::new(grid);
    if let Ok(mut c) = cache().lock() {
        c.retain(|e| e.owner.strong_count() > 0);
        if c.len() >= 2 {
            c.remove(0);
        }
        c.push(Entry {
            key: k,
            owner: Arc::downgrade(sdf),
            grid: grid.clone(),
        });
    }
    grid
}

// Viewport versions of existing measurement formulas. Cubic analysis stays unchanged.
pub fn compute_model_properties(grid: &ViewportSdfGrid) -> (f32, f32, Vec3) {
    let [nx, ny, nz] = grid.spec.dimensions().map(|n| n as usize);
    let span = grid.bounds_max - grid.bounds_min;
    let dx = span.x / nx as f32;
    let dy = span.y / ny as f32;
    let dz = span.z / nz as f32;
    let dv = dx * dy * dz; // voxel volume
    let voxel_d = (dx * dy * dz).cbrt(); // characteristic voxel size

    let idx = |ix: usize, iy: usize, iz: usize| ix + iy * nx + iz * nx * ny;

    // Partial sums per row (z-major).
    let rows: Vec<(f64, f64, f64, f64, f64)> = (0..nz)
        .into_par_iter()
        .map(|iz| {
            let mut vol = 0.0f64;
            let mut sa = 0.0f64;
            let mut cx = 0.0f64;
            let mut cy = 0.0f64;
            let mut cz = 0.0f64;
            for iy in 0..ny {
                for ix in 0..nx {
                    let d = grid.data[idx(ix, iy, iz)];
                    if d < 0.0 {
                        vol += dv as f64;
                        let px = grid.bounds_min.x + (ix as f32 + 0.5) * dx;
                        let py = grid.bounds_min.y + (iy as f32 + 0.5) * dy;
                        let pz = grid.bounds_min.z + (iz as f32 + 0.5) * dz;
                        cx += px as f64 * dv as f64;
                        cy += py as f64 * dv as f64;
                        cz += pz as f64 * dv as f64;
                    }
                    // Surface area: Cauchy-Crofton on near-surface voxels.
                    if d.abs() < voxel_d * 1.5 {
                        // Gradient magnitude via central differences.
                        let gx = {
                            let xa = if ix > 0 {
                                grid.data[idx(ix - 1, iy, iz)]
                            } else {
                                d
                            };
                            let xb = if ix < nx - 1 {
                                grid.data[idx(ix + 1, iy, iz)]
                            } else {
                                d
                            };
                            (xb - xa) / (2.0 * dx)
                        };
                        let gy = {
                            let ya = if iy > 0 {
                                grid.data[idx(ix, iy - 1, iz)]
                            } else {
                                d
                            };
                            let yb = if iy < ny - 1 {
                                grid.data[idx(ix, iy + 1, iz)]
                            } else {
                                d
                            };
                            (yb - ya) / (2.0 * dy)
                        };
                        let gz = {
                            let za = if iz > 0 {
                                grid.data[idx(ix, iy, iz - 1)]
                            } else {
                                d
                            };
                            let zb = if iz < nz - 1 {
                                grid.data[idx(ix, iy, iz + 1)]
                            } else {
                                d
                            };
                            (zb - za) / (2.0 * dz)
                        };
                        let grad_mag = (gx * gx + gy * gy + gz * gz).sqrt();
                        // Integrate |∇φ| δ(φ) dV, approximated as |∇φ| * dV / (voxel_d)
                        sa += (grad_mag * dv as f32 / voxel_d) as f64;
                    }
                }
            }
            (vol, sa, cx, cy, cz)
        })
        .collect();

    let (vol, sa, cx, cy, cz) = rows
        .iter()
        .fold((0.0f64, 0.0f64, 0.0f64, 0.0f64, 0.0f64), |a, b| {
            (a.0 + b.0, a.1 + b.1, a.2 + b.2, a.3 + b.3, a.4 + b.4)
        });

    let com = if vol > 0.0 {
        Vec3::new((cx / vol) as f32, (cy / vol) as f32, (cz / vol) as f32)
    } else {
        Vec3::ZERO
    };

    (vol as f32, sa as f32, com)
}

pub fn ray_march_grid(
    grid: &ViewportSdfGrid,
    origin: Vec3,
    dir: Vec3, // normalised
    max_dist: f32,
) -> Option<Vec3> {
    let mut t = 0.0f32;
    let step_min = grid.spacing().min_element();

    for _ in 0..256 {
        let p = origin + dir * t;
        let d = sample_grid(grid, p);
        if d < step_min * 0.5 {
            return Some(p);
        }
        t += d.max(step_min);
        if t > max_dist {
            break;
        }
    }
    None
}

/// Trilinear sample of the SDF grid at world-space point `p`.
pub fn sample_grid(grid: &ViewportSdfGrid, p: Vec3) -> f32 {
    let [nx, ny, nz] = grid.spec.dimensions();
    let uvw = (p - grid.bounds_min) / (grid.bounds_max - grid.bounds_min);
    let tc = uvw * Vec3::new(nx as f32, ny as f32, nz as f32) - Vec3::splat(0.5);
    let i = tc.floor().as_ivec3();
    let f = tc.fract();

    let load = |di: i32, dj: i32, dk: i32| -> f32 {
        let ci = (i.x + di).clamp(0, nx as i32 - 1) as usize;
        let cj = (i.y + dj).clamp(0, ny as i32 - 1) as usize;
        let ck = (i.z + dk).clamp(0, nz as i32 - 1) as usize;
        grid.data[ci + cj * nx as usize + ck * nx as usize * ny as usize]
    };

    let v000 = load(0, 0, 0);
    let v100 = load(1, 0, 0);
    let v010 = load(0, 1, 0);
    let v110 = load(1, 1, 0);
    let v001 = load(0, 0, 1);
    let v101 = load(1, 0, 1);
    let v011 = load(0, 1, 1);
    let v111 = load(1, 1, 1);
    let x00 = v000 + (v100 - v000) * f.x;
    let x10 = v010 + (v110 - v010) * f.x;
    let x01 = v001 + (v101 - v001) * f.x;
    let x11 = v011 + (v111 - v011) * f.x;
    let y0 = x00 + (x10 - x00) * f.y;
    let y1 = x01 + (x11 - x01) * f.y;
    y0 + (y1 - y0) * f.z
}

pub fn tight_bounds_from_grid(grid: &ViewportSdfGrid) -> Option<(Vec3, Vec3)> {
    let mut lo = grid.bounds_max;
    let mut hi = grid.bounds_min;
    for (i, &v) in grid.data.iter().enumerate() {
        if v < 0. {
            let p = grid.spec.point(i, grid.bounds_min, grid.bounds_max);
            lo = lo.min(p);
            hi = hi.max(p);
        }
    }
    (lo.x <= hi.x).then_some((lo, hi))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::primitives::Sphere;
    #[test]
    fn dimensions_counts_strides_and_centres() {
        assert!(ViewportGridSpec::new([0, 2, 3]).is_err());
        assert!(ViewportGridSpec::new([u32::MAX, 2, 2]).is_err());
        let s = ViewportGridSpec::new([2, 3, 4]).unwrap();
        assert_eq!(s.cell_count(), 24);
        assert_eq!(s.index(1, 0, 0), 1);
        assert_eq!(s.index(0, 1, 0), 2);
        assert_eq!(s.index(0, 0, 1), 6);
        let lo = Vec3::new(-1., -3., -6.);
        let hi = -lo;
        assert_eq!(s.point(0, lo, hi), Vec3::new(-0.5, -2., -4.5));
        assert_eq!(s.point(23, lo, hi), Vec3::new(0.5, 2., 4.5));
        for i in 0..24 {
            let [x, y, z] = s.coordinates(i);
            assert_eq!(s.index(x, y, z), i);
        }
    }
    #[test]
    fn cpu_sampling_order_and_cubic_measurement_identity() {
        let sdf = Sphere::new(2.);
        let lo = Vec3::splat(-3.);
        let hi = -lo;
        for spec in [
            ViewportGridSpec::new([3, 5, 7]).unwrap(),
            ViewportGridSpec::cube(8),
        ] {
            let g = sample_cpu(&sdf, lo, hi, spec);
            for (i, v) in g.data.iter().enumerate() {
                assert_eq!(v.to_bits(), sdf.distance(spec.point(i, lo, hi)).to_bits());
            }
            if let Some(c) = g.to_cubic() {
                let a = compute_model_properties(&g);
                let b = crate::analysis::compute_model_properties(&c);
                assert_eq!(a, b);
                assert_eq!(g.voxel_size(), 6. / 8.);
                for p in [Vec3::ZERO, Vec3::ONE, lo, hi] {
                    assert_eq!(
                        sample_grid(&g, p).to_bits(),
                        crate::analysis::measurements::sample_grid(&c, p).to_bits()
                    );
                }
            }
        }
    }
    #[test]
    fn rectangular_volume_and_interpolation() {
        let spec = ViewportGridSpec::new([2, 3, 4]).unwrap();
        let lo = Vec3::ZERO;
        let hi = Vec3::new(2., 6., 12.);
        let mut g = ViewportSdfGrid {
            data: vec![-1.; 24],
            spec,
            bounds_min: lo,
            bounds_max: hi,
        };
        assert_eq!(compute_model_properties(&g).0, 144.);
        assert!(g.to_cubic().is_none());
        assert_eq!(g.voxel_size(), 3.);
        g.data = (0..24)
            .map(|i| spec.point(i, lo, hi).dot(Vec3::new(1., 2., 3.)))
            .collect();
        for i in 0..24 {
            assert_eq!(sample_grid(&g, spec.point(i, lo, hi)), g.data[i]);
        }
    }
    #[test]
    fn cache_keys_include_each_dimension_exact_bounds_and_owner() {
        let a: Arc<dyn Sdf> = Arc::new(Sphere::new(2.));
        let b: Arc<dyn Sdf> = Arc::new(Sphere::new(2.));
        let lo = Vec3::ZERO;
        let hi = Vec3::ONE;
        let s = ViewportGridSpec::new([96, 192, 128]).unwrap();
        let k = key(&a, lo, hi, s);
        assert!(k == key(&a, lo, hi, s));
        assert!(k != key(&b, lo, hi, s));
        for d in [
            [192, 96, 128],
            [96, 193, 128],
            [96, 192, 129],
            [97, 192, 128],
        ] {
            assert!(k != key(&a, lo, hi, ViewportGridSpec::new(d).unwrap()));
        }
        assert!(k != key(&a, lo, hi + Vec3::X * 0.0001, s));
    }
    #[test]
    fn gpu_partial_workgroups_and_cubic_regression() {
        let w = crate::sdf::aerospace::wing_with_airfoil("0012", 60., 30., 120., 5., 0., 0.);
        let ir = crate::gpu::wing::lower_viewport_sdf_ir(&w).unwrap();
        let lo = Vec3::new(-1., -62., -5.);
        let hi = Vec3::new(65., 62., 5.);
        let spec = ViewportGridSpec::new([7, 9, 5]).unwrap();
        let gpu = crate::gpu::wing::viewport_grid(&ir, lo, hi, spec).unwrap();
        let cpu = sample_cpu(&w, lo, hi, spec);
        assert_eq!(gpu.data.len(), 315);
        for (a, b) in cpu.data.iter().zip(&gpu.data) {
            assert!((a - b).abs() < 1e-4);
        }
        let old = crate::gpu::wing::grid(&ir, lo, hi, 8).unwrap();
        let new = crate::gpu::wing::viewport_grid(&ir, lo, hi, ViewportGridSpec::cube(8)).unwrap();
        assert_eq!(
            old.data.iter().map(|v| v.to_bits()).collect::<Vec<_>>(),
            new.data.iter().map(|v| v.to_bits()).collect::<Vec<_>>()
        );
    }
}

#[cfg(test)]
mod cache_tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};
    struct Counted(AtomicUsize);
    impl Sdf for Counted {
        fn distance(&self, p: Vec3) -> f32 {
            self.0.fetch_add(1, Ordering::Relaxed);
            p.length() - 1.
        }
    }
    #[test]
    fn repeated_cache_hits_and_dimension_changes_miss() {
        let concrete = Arc::new(Counted(AtomicUsize::new(0)));
        let sdf: Arc<dyn Sdf> = concrete.clone();
        let lo = Vec3::splat(-2.);
        let hi = -lo;
        let a = ViewportGridSpec::new([3, 4, 5]).unwrap();
        let b = ViewportGridSpec::new([4, 3, 5]).unwrap();
        let ga = sample_cached(&sdf, lo, hi, a);
        let repeat = sample_cached(&sdf, lo, hi, a);
        assert!(Arc::ptr_eq(&ga, &repeat));
        assert_eq!(concrete.0.load(Ordering::Relaxed), 60);
        let gb = sample_cached(&sdf, lo, hi, b);
        assert_eq!(gb.spec, b);
        assert!(!Arc::ptr_eq(&ga, &gb));
        assert_eq!(concrete.0.load(Ordering::Relaxed), 120);
        let moved = sample_cached(&sdf, lo + Vec3::X * 0.001, hi, b);
        assert_ne!(moved.bounds_min, gb.bounds_min);
        assert_eq!(concrete.0.load(Ordering::Relaxed), 180);
    }
}
