
# IMPLICIT BRACKET ENGINE (Granular) — DETAILED SPECIFICATION

Below is a detailed, implementation-ready specification for your IMPLICIT BRACKET ENGINE (Granular). It extends the original tasks with explicit SDF conventions, types, safe utilities, stable numerics, configuration parameters, test cases, diagnostics, and optimization notes. This is intended to be used as a definitive prompt for a coding agent or an engineering implementation.
all previous bracket/support work is outdated and should be removed. this is the new source of truth. do not try to blend the old bracket code with new code 
---

## Overview & Conventions

- SDF sign convention: All signed-distance functions return negative inside geometry, zero on surface, and positive outside.
  - Boolean semantics reminder: `min(d1, d2)` = union (closest material). `max(d1, d2)` = intersection/subtraction behavior depends on usage; see Phase 5 specifics.
- Units: All distances are in millimeters (mm) by default.
- Types:
  - `Vec3` — 3D vector with float components `x, y, z`.
  - `SDF` — callable `f(p: Vec3) -> float` returning signed distance.
  - `Part` — base object in scene (see Phase 1).
- Numerical tolerances:
  - `EPS = 1e-8` default for normalization.
  - Central difference epsilon default `GRAD_EPS = 1e-3` mm (configurable).

---

## Phase 1: Data Model & Core Metadata

### Data types (recommend Python dataclasses for clarity)

    from dataclasses import dataclass
    from typing import Callable, Optional, List

    @dataclass
    class Vec3:
        x: float
        y: float
        z: float

    @dataclass
    class MountPoint:
        position: Vec3
        normal: Vec3               # expected outward surface normal on host at mount
        tier: int                  # 1 or 2
        base_radius: float         # mm

    @dataclass
    class Part:
        id: str
        sdf: Callable[[Vec3], float]   # signed distance function for the part
        structural_role: str           # 'STICKY' | 'REMOVABLE' | 'NEUTRAL'
        bbox_min: Optional[Vec3] = None
        bbox_max: Optional[Vec3] = None
        metadata: dict = None
        mount_points: List[MountPoint] = None

### Requirements / tasks

- Task 1.1: `structural_role` property
  - Allowed values: `'STICKY'`, `'REMOVABLE'`, `'NEUTRAL'`.
  - Validate at construction or ingestion.

- Task 1.2: `MountPoint` object
  - `position`: world-space `Vec3`.
  - `normal`: unit `Vec3`. Validate with `safe_normalize`.
  - `tier`: integer, must be 1 or 2.
  - `base_radius`: positive float in mm.

- Task 1.3: `get_host_sdf(p: Vec3) -> (float, Optional[str])`
  - Aggregate SDF of all parts where `structural_role == 'STICKY'`.
  - Return `(dist, nearest_part_id)`.
  - Aggregation: `min(dist_i)` across relevant parts (union).
  - Optimize by testing part AABBs before evaluating SDFs.

- Task 1.4: `get_keepout_sdf(p: Vec3) -> (float, Optional[str])`
  - Aggregate SDF of all parts where `structural_role == 'REMOVABLE'`.
  - Return `(dist, nearest_part_id)`.
  - Aggregation: `min(dist_i)` (union).

---

## Phase 2: Vector Math & SDF Primitives

### Safety helpers (language-agnostic)

- `safe_normalize(v: Vec3, eps=1e-8) -> Vec3` — returns zero vector if magnitude < `eps`.
- `clamp(x, lo, hi)`.
- `vec_len(v)`, `vec_add`, `vec_sub`, `vec_mul_scalar`.

### Task 2.1: `calc_gradient(sdf_func, p, epsilon=1e-3) -> Vec3`

- Central difference:
  - `gx = (f(p + ex) - f(p - ex)) / (2*epsilon)`, similarly for `gy`, `gz`.
- Return normalized gradient (direction of increasing SDF).
- Fallback: If central-difference gradient magnitude below threshold, try forward/backward difference or return zero vector.
- Example usage: `grad = calc_gradient(lambda q: host_sdf(q)[0], p)`

### Task 2.2: `sdf_tapered_capsule(p, a, b, ra, rb) -> float`

- Inputs: `a: Vec3`, `b: Vec3`, `ra, rb: float` in mm.
- Algorithm:
  - `ab = b - a`
  - `t = clamp(dot(p - a, ab) / dot(ab, ab), 0, 1)` (if `dot(ab, ab) == 0`, degenerate)
  - `p_on_segment = a + ab * t`
  - `r = ra + t * (rb - ra)`
  - Return `length(p - p_on_segment) - r`
- Degenerate case: if `a == b`, return sphere SDF at `a` with radius `ra` or `max(ra, rb)`.

### Task 2.3: `smin_exp(d1, d2, k=32.0) -> float`

- Numerically-stable subtract-max trick:
    
    import math

    def smin_exp(d1, d2, k=32.0, eps=1e-12):
        if k <= 0:
            return min(d1, d2)
        a = -k * d1
        b = -k * d2
        m = max(a, b)
        s = math.exp(a - m) + math.exp(b - m)
        return - (math.log(s + eps) + m) / k

- For large `k`, `smin_exp` approximates `min(d1, d2)`; for small `k`, it blends smoothly.

---

## Phase 3: Gradient-Following Pathfinder

### Overview

- Start at the mountpoint (offset slightly outwards) and iteratively move according to an attraction vector (to host wall) and a repulsion vector (away from keepout). Trace a path composed of sample points.

- Diagnostics: record step history, gradients, `dist_to_host`, `dist_to_keepout`, termination reason.

### Task 3.1: Attraction vector

- `Attraction_Vec(p) = -calc_gradient(lambda q: host_sdf(q)[0], p)`  
  - Points in direction of decreasing SDF (toward host surface).

### Task 3.2: Repulsion vector

- `Repulsion_Vec(p) = calc_gradient(lambda q: keepout_sdf(q)[0], p)`  
  - Points away from removable obstacles.

### Task 3.3: Steered Stepper — `get_next_step(p) -> Vec3`

- Inputs:
  - `p: Vec3`
  - Config: `step_size_mm` (default `1.0`), `min_step_mm` (`0.1`), `max_step_mm` (`5.0`), `rep_scale_max` (`10.0`).
- Compute:
  - `attr = Attraction_Vec(p)`
  - `rep = Repulsion_Vec(p)`
  - `d_keepout = keepout_sdf(p)[0]`
  - `rep_scale = clamp(1.0 / (abs(d_keepout) + 1e-2), 0.0, rep_scale_max)`
  - `combined = attr + rep * rep_scale`
  - `combined = safe_normalize(combined)` (fallbacks: prefer `attr` if `combined` is zero)
  - `dist_to_host = abs(host_sdf(p)[0])`
  - `adaptive_step = clamp(step_size_mm * max(0.5, dist_to_host), min_step_mm, max_step_mm)`
  - `step = combined * adaptive_step`
- Guards:
  - Avoid divide-by-zero, clamp scales.
  - Detect oscillation / repeated back-and-forth and apply damping or reduce `adaptive_step`.

### Task 3.4: Path Tracer — `trace_path_from_mount(mount: MountPoint, config) -> PathResult`

- Inputs:
  - `start_p = mount.position + mount.normal * initial_offset_mm` where `initial_offset_mm = max(0.5, mount.base_radius * 0.1)`
  - `max_iterations = 200` (configurable)
  - `tol_surface = 0.05` mm (termination threshold)
  - `min_distance_keepout = 0.1` mm (clearance)
- Loop:
  - For i in `1..max_iterations`:
    - `step = get_next_step(p)`
    - If `vec_len(step) < min_step_mm * 0.1`: terminate as `stuck_small_step`
    - `p_next = p + step`
    - Append `p_next` to path
    - If `abs(get_host_sdf(p_next)[0]) <= tol_surface`: terminate `hit_host`
    - If `get_keepout_sdf(p_next)[0] <= -0.5`: terminate `violated_keepout` (penetrated keepout)
  - If iterations exhausted: terminate `max_iter`
- Return a `PathResult`:
  - fields: `points: List[Vec3]`, `termination_reason: str`, `iterations: int`, `diagnostics: dict`

---

## Phase 4: Tier 1 & Tier 2 Generation

### Task 4.1: Tier 1 Capsule Generation

- For each Tier 1 mount point:
  - Use path points from Phase 3.
  - For consecutive points `p_i -> p_{i+1}`, generate tapered capsule segments:
    - segment endpoints `(a, b) = (p_i, p_{i+1})`
    - radii: compute linear interpolation along path:
      - At mount: `r_start = base_radius`
      - At far end: `r_end = base_radius * radius_end_factor` where `radius_end_factor = 2.5` (configurable)
  - Create `sdf_tapered_capsule` SDFs for each segment.
  - Aggregate into `tier1_sdf_union(p)` by `min` across segments.
- Edge cases: If path has fewer than 2 points, do not generate.

### Task 4.2: Tier 2 Search

- For each Tier 2 mount point:
  - Sample `dist_to_tier1 = tier1_sdf_union(mount.position)` (and optionally along small offsets).
  - Use `tier2_search_radius = 15.0` mm default.

### Task 4.3: Tier 2 Bridge

- If `dist_to_tier1 < tier2_search_radius`:
  - Find nearest point on Tier 1 path (nearest on segments).
  - Generate a single tapered capsule bridging the Tier 2 trace end to nearest Tier 1 point.
  - Bridge radius configurable: `bridge_radius = base_radius * bridge_radius_factor`.
- Task 4.4: Tier 2 Pruning
  - If `dist_to_tier1 >= tier2_search_radius` return null; do not generate geometry.

---

## Phase 5: Assembly & Final Booleans

### Task 5.1: Initial Bracket Union

- Combine all Tier 1 and Tier 2 capsule SDFs into `bracket_field(p)` using `smin_exp` with `k=2.0` (smooth union).
- Implement pairwise reduction or an n-ary stable reduction.

### Task 5.2: Clearance Dilation

- Dilation offset: `dilate_mm = +1.5` mm (configurable).
- Given convention, dilated keepout SDF:
  - `dilated_keepout_sdf(p) = get_keepout_sdf(p)[0] - dilate_mm`
- This effectively grows the keepout region by `dilate_mm`.

### Task 5.3: The Hard Cut (air-gap for removable parts)

- Enforce hard cut so bracket never occupies dilated keepout:
  - `bracket_after_cut(p) = max(bracket_field(p), -dilated_keepout_sdf(p))`
- Rationale:
  - For points inside dilated keepout, `dilated_keepout_sdf` is negative; `-dilated_keepout_sdf` becomes positive, `max` forces bracket SDF positive (air) there.

### Task 5.4: Organic Wall Integration (fillets to host)

- Blend bracket into host to form organic filleted transitions:
  - `final_blended(p) = smin_exp(bracket_after_cut(p), host_sdf(p)[0], k=5.0)`
- Higher `k` yields sharper blends; adjust per desired fillet size.

### Task 5.5: Pocketing (hardware mounting hole)

- If `component_body_sdf` available for the hardware (mount), pocket out clearance:
  - `pocketed(p) = max(final_blended(p), -(component_body_sdf(p) + pocket_offset_mm))`
  - Typical `pocket_offset_mm = 0.2` mm.
- Validate pocket depth and tolerance.

---

## Configuration Parameters & Tuning

- Global numeric defaults:
  - `GRAD_EPS = 1e-3` mm
  - `SAFE_NORM_EPS = 1e-8`
  - `STEP_SIZE_MM = 1.0`
  - `MIN_STEP_MM = 0.1`
  - `MAX_STEP_MM = 5.0`
  - `REP_SCALE_MAX = 10.0`
  - `MAX_PATH_ITERS = 200`
  - `SURFACE_TOL_MM = 0.05`
  - `TIER2_BRIDGE_THRESH_MM = 15.0`
  - `DILATE_KEEPOUT_MM = 1.5`
  - `POCKET_OFFSET_MM = 0.2`
- Expose these parameters via a `Config` object to tune per-run or per-part.

---

## Numerical Safety Helpers & Diagnostics

- `safe_normalize(v, eps=SAFE_NORM_EPS) -> Vec3`
- `smin_exp` stable implementation using subtract-max trick (see above).
- Diagnostics to capture per-path:
  - `iteration_count`, `termination_reason` (`'hit_host'`, `'violated_keepout'`, `'stuck_small_step'`, `'max_iter'`)
  - arrays of `dist_to_host`, `dist_to_keepout`, gradient magnitudes, step magnitudes
  - nearest part IDs for host/keepout per sample
- Failure modes to detect:
  - Stuck oscillation on ridge: repeated back-and-forth steps
  - Immediate keepout violation: mountpoint inside removable geometry
  - No host hit within `MAX_PATH_ITERS`
  - NaN or Inf in SDF or gradients

---

## Performance, Caching, & Spatial Acceleration

- Avoid evaluating all parts per SDF call:
  - Build BVH or spatial hash on part bounding boxes to limit candidate parts.
  - Optionally quantize position and memoize SDF results (LRU cache).
- Parallelize per-mountpoint traces (embarrassingly parallel).
- Cache gradient computations when possible (reuse neighbor samples).

---

## Unit Tests & Validation Suite

- SDF & gradient tests:
  - Plane SDF: `f(p) = p.z - z0` → gradient constant `(0,0,1)`.
  - `sdf_tapered_capsule` on-axis: verify distances at several `t`.
  - Degenerate capsule `a==b` behaves like a sphere.
- `smin_exp` tests:
  - For very large `k` (e.g., 1e6), `smin_exp(d1,d2,k)` ≈ `min(d1,d2)`.
  - No NaNs or Infs for extreme values.
- Path tracer tests:
  - Simple scene: host = plane at z=0 (STICKY); keepout = cylinder above plane (REMOVABLE); mount at (0,0,10). Path should terminate `hit_host` without penetrating keepout.
  - Entry inside keepout: mountpoint inside removable geometry → expect `violated_keepout` or outward push handling.
- Integration tests:
  - Full pipeline on a simple mount scenario: verify final SDF has pocket and clearance.
- Performance test:
  - Measure SDF evaluations per path; test BVH reduces candidate counts significantly.

---

## Example Function Signatures

- `get_host_sdf(p: Vec3) -> Tuple[float, Optional[str]]`
- `get_keepout_sdf(p: Vec3) -> Tuple[float, Optional[str]]`
- `calc_gradient(sdf_func: Callable[[Vec3], float], p: Vec3, epsilon: float = 1e-3) -> Vec3`
- `sdf_tapered_capsule(p: Vec3, a: Vec3, b: Vec3, ra: float, rb: float) -> float`
- `smin_exp(d1: float, d2: float, k: float = 32.0) -> float`
- `get_next_step(p: Vec3, config: Config) -> Tuple[Vec3, DebugInfo]`
- `trace_path_from_mount(mount: MountPoint, config: Config) -> PathResult`

---

## Example Code Snippets (Compact)

- Safe normalize and gradient (conceptual):

    import math
    from typing import Callable

    def vec_len(v): return math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
    def vec_add(a, b): return Vec3(a.x+b.x, a.y+b.y, a.z+b.z)
    def vec_sub(a, b): return Vec3(a.x-b.x, a.y-b.y, a.z-b.z)
    def vec_scale(v, s): return Vec3(v.x*s, v.y*s, v.z*s)

    def safe_normalize(v, eps=1e-8):
        n = vec_len(v)
        if n < eps:
            return Vec3(0.0, 0.0, 0.0)
        return vec_scale(v, 1.0/n)

    def calc_gradient(sdf_func: Callable[[Vec3], float], p: Vec3, eps: float = 1e-3):
        ex = Vec3(eps, 0.0, 0.0)
        ey = Vec3(0.0, eps, 0.0)
        ez = Vec3(0.0, 0.0, eps)
        dx = sdf_func(vec_add(p, ex)) - sdf_func(vec_sub(p, ex))
        dy = sdf_func(vec_add(p, ey)) - sdf_func(vec_sub(p, ey))
        dz = sdf_func(vec_add(p, ez)) - sdf_func(vec_sub(p, ez))
        g = Vec3(dx * 0.5/eps, dy * 0.5/eps, dz * 0.5/eps)
        return safe_normalize(g)

- Stable `smin_exp`:

    import math

    def smin_exp(d1, d2, k=32.0, eps=1e-12):
        if k <= 0:
            return min(d1, d2)
        a = -k * d1
        b = -k * d2
        m = max(a, b)
        s = math.exp(a - m) + math.exp(b - m)
        return - (math.log(s + eps) + m) / k

- Tapered capsule sketch:

    def sdf_tapered_capsule(p, a, b, ra, rb):
        ab = vec_sub(b, a)
        ab_len2 = dot(ab, ab)
        if ab_len2 == 0.0:
            # degenerate -> sphere at a
            return vec_len(vec_sub(p, a)) - max(ra, rb)
        t = clamp(dot(vec_sub(p, a), ab) / ab_len2, 0.0, 1.0)
        p_on_seg = vec_add(a, vec_scale(ab, t))
        r = ra + t * (rb - ra)
        return vec_len(vec_sub(p, p_on_seg)) - r

---

## Diagnostics & Failure Modes to Report

- Stuck in saddle/ridge: repeated oscillatory steps — record and apply damping or reduce step size automatically.
- Immediate

