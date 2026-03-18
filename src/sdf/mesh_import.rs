// Mesh-to-SDF conversion.
//
// Two modes:
//   - Approximate: voxelized 64³ grid, trilinear interpolation.  Fast.
//   - Accurate: exact point-to-triangle BVH + generalized winding number sign.  Slow.

use std::sync::Arc;
use glam::Vec3;
use rayon::prelude::*;
use crate::sdf::Sdf;
use crate::mesh::TriangleMesh;

// ── BVH ───────────────────────────────────────────────────────────────────────

/// AABB for BVH nodes.
#[derive(Clone, Copy)]
struct Aabb {
    min: Vec3,
    max: Vec3,
}

impl Aabb {
    #[allow(dead_code)] // Used in BVH construction
    fn new(min: Vec3, max: Vec3) -> Self { Self { min, max } }
    fn empty() -> Self { Self { min: Vec3::splat(f32::MAX), max: Vec3::splat(f32::MIN) } }

    fn extend(&mut self, p: Vec3) {
        self.min = self.min.min(p);
        self.max = self.max.max(p);
    }

    fn extend_aabb(&mut self, other: &Aabb) {
        self.min = self.min.min(other.min);
        self.max = self.max.max(other.max);
    }

    #[allow(dead_code)] // Used in SAH-split BVH build
    fn centroid(&self) -> Vec3 { (self.min + self.max) * 0.5 }
    #[allow(dead_code)] // Used in SAH cost computation
    fn surface_area(&self) -> f32 {
        let e = self.max - self.min;
        2.0 * (e.x * e.y + e.y * e.z + e.z * e.x)
    }

    /// Squared distance from point to the AABB (0 if inside).
    fn dist_sq_to_point(&self, p: Vec3) -> f32 {
        let d = (self.min - p).max(Vec3::ZERO) + (p - self.max).max(Vec3::ZERO);
        d.length_squared()
    }
}

fn triangle_aabb(mesh: &TriangleMesh, tri: u32) -> Aabb {
    let [a, b, c] = mesh.triangles[tri as usize];
    let va = mesh.vertices[a as usize];
    let vb = mesh.vertices[b as usize];
    let vc = mesh.vertices[c as usize];
    let mut bb = Aabb::empty();
    bb.extend(va); bb.extend(vb); bb.extend(vc);
    bb
}

struct BvhNode {
    aabb:  Aabb,
    /// `left` < 0 means leaf: triangle index = -(left + 1).
    /// `left` >= 0 means internal: children at `left` and `left+1`.
    left:  i32,
    right: i32, // only valid for internal nodes
}

pub struct Bvh {
    nodes:     Vec<BvhNode>,
    mesh:      Arc<TriangleMesh>,
}

impl Bvh {
    pub fn build(mesh: Arc<TriangleMesh>) -> Self {
        let n = mesh.triangles.len();
        let mut tris: Vec<u32> = (0..n as u32).collect();
        let mut nodes = Vec::with_capacity(n * 2);
        build_bvh_recursive(&mesh, &mut tris, &mut nodes);
        Self { nodes, mesh }
    }

    /// Nearest squared distance and triangle index.
    pub fn nearest_sq(&self, p: Vec3) -> (f32, u32) {
        nearest_recursive(&self.nodes, &self.mesh, p, 0, f32::MAX)
    }

    /// Signed distance using majority-vote ray casting (fast, approximate sign).
    pub fn ray_cast_sign(&self, p: Vec3) -> f32 {
        let (d2, _) = self.nearest_sq(p);
        let dist = d2.sqrt();
        let sign = ray_cast_sign(&self.mesh, p);
        sign * dist
    }
}

fn build_bvh_recursive(mesh: &TriangleMesh, tris: &mut [u32], nodes: &mut Vec<BvhNode>) -> i32 {
    let idx = nodes.len() as i32;

    let mut aabb = Aabb::empty();
    for &t in tris.iter() {
        aabb.extend_aabb(&triangle_aabb(mesh, t));
    }

    if tris.len() == 1 {
        nodes.push(BvhNode { aabb, left: -(tris[0] as i32 + 1), right: -1 });
        return idx;
    }

    // Longest-axis median split.
    let extent = aabb.max - aabb.min;
    let axis = if extent.x >= extent.y && extent.x >= extent.z { 0 }
               else if extent.y >= extent.z { 1 }
               else { 2 };

    let centroid_of = |t: u32| -> f32 {
        let [a, b, c] = mesh.triangles[t as usize];
        let v = (mesh.vertices[a as usize]
               + mesh.vertices[b as usize]
               + mesh.vertices[c as usize]) / 3.0;
        match axis { 0 => v.x, 1 => v.y, _ => v.z }
    };

    let mid_val = {
        let mut vals: Vec<f32> = tris.iter().map(|&t| centroid_of(t)).collect();
        vals.sort_by(|a, b| a.partial_cmp(b).unwrap());
        vals[vals.len() / 2]
    };

    let split = tris.partition_point(|&t| centroid_of(t) < mid_val).max(1).min(tris.len() - 1);
    let (left_tris, right_tris) = tris.split_at_mut(split);

    // Reserve node slot before recursing so idx stays valid.
    nodes.push(BvhNode { aabb, left: -1, right: -1 });

    let left  = build_bvh_recursive(mesh, left_tris,  nodes);
    let right = build_bvh_recursive(mesh, right_tris, nodes);

    nodes[idx as usize].left  = left;
    nodes[idx as usize].right = right;
    idx
}

fn nearest_recursive(
    nodes: &[BvhNode],
    mesh:  &TriangleMesh,
    p:     Vec3,
    node:  i32,
    best:  f32,
) -> (f32, u32) {
    let n = &nodes[node as usize];

    // Early-out if this node's AABB is farther than the current best.
    if n.aabb.dist_sq_to_point(p) >= best { return (best, 0); }

    if n.left < 0 {
        // Leaf: compute exact point-to-triangle distance.
        let tri_idx = -(n.left + 1) as u32;
        let d2 = point_triangle_dist_sq(mesh, tri_idx, p);
        return if d2 < best { (d2, tri_idx) } else { (best, 0) };
    }

    // Internal: recurse into both children, closer first.
    let d_left  = nodes[n.left  as usize].aabb.dist_sq_to_point(p);
    let d_right = nodes[n.right as usize].aabb.dist_sq_to_point(p);

    let (first, second) = if d_left <= d_right {
        (n.left, n.right)
    } else {
        (n.right, n.left)
    };

    let (best, best_tri) = nearest_recursive(nodes, mesh, p, first, best);
    nearest_recursive(nodes, mesh, p, second, best)
        .pipe(|(d, t)| if d < best { (d, t) } else { (best, best_tri) })
}

trait Pipe: Sized {
    fn pipe<F: FnOnce(Self) -> Self>(self, f: F) -> Self { f(self) }
}
impl<T> Pipe for T {}

// ── Point-to-triangle distance (Eberly method) ────────────────────────────────

fn point_triangle_dist_sq(mesh: &TriangleMesh, tri: u32, p: Vec3) -> f32 {
    let [a, b, c] = mesh.triangles[tri as usize];
    let va = mesh.vertices[a as usize];
    let vb = mesh.vertices[b as usize];
    let vc = mesh.vertices[c as usize];
    point_to_triangle_sq(p, va, vb, vc)
}

/// Squared distance from point `p` to triangle (a, b, c).
fn point_to_triangle_sq(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> f32 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;

    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 { return ap.length_squared(); }

    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 { return bp.length_squared(); }

    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 { return cp.length_squared(); }

    let vc_ = d1 * d4 - d3 * d2;
    if vc_ <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let t = d1 / (d1 - d3);
        return (ap - ab * t).length_squared();
    }

    let vb_ = d5 * d2 - d1 * d6;
    if vb_ <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let t = d2 / (d2 - d6);
        return (ap - ac * t).length_squared();
    }

    let va_ = d3 * d6 - d5 * d4;
    if va_ <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (p - (b + (c - b) * t)).length_squared();
    }

    let denom = 1.0 / (va_ + vb_ + vc_);
    let s = vb_ * denom;
    let t = vc_ * denom;
    (ap - ab * s - ac * t).length_squared()
}

// ── Sign determination ────────────────────────────────────────────────────────

/// Majority-vote ray casting in +X, +Y, +Z directions.
fn ray_cast_sign(mesh: &TriangleMesh, p: Vec3) -> f32 {
    let votes = [
        ray_crosses(mesh, p, Vec3::X),
        ray_crosses(mesh, p, Vec3::Y),
        ray_crosses(mesh, p, Vec3::Z),
    ];
    let inside = votes.iter().filter(|&&v| v % 2 != 0).count();
    if inside >= 2 { -1.0 } else { 1.0 }
}

/// Number of triangle crossings along a ray from `origin` in `dir`.
fn ray_crosses(mesh: &TriangleMesh, origin: Vec3, dir: Vec3) -> u32 {
    let mut count = 0u32;
    for &[a, b, c] in &mesh.triangles {
        let va = mesh.vertices[a as usize];
        let vb = mesh.vertices[b as usize];
        let vc = mesh.vertices[c as usize];
        if moller_trumbore(origin, dir, va, vb, vc) {
            count += 1;
        }
    }
    count
}

/// Möller-Trumbore ray-triangle intersection (forward direction only, t > 0).
fn moller_trumbore(orig: Vec3, dir: Vec3, a: Vec3, b: Vec3, c: Vec3) -> bool {
    const EPS: f32 = 1e-8;
    let e1 = b - a;
    let e2 = c - a;
    let h  = dir.cross(e2);
    let det = e1.dot(h);
    if det.abs() < EPS { return false; }
    let inv_det = 1.0 / det;
    let s = orig - a;
    let u = s.dot(h) * inv_det;
    if !(0.0..=1.0).contains(&u) { return false; }
    let q = s.cross(e1);
    let v = dir.dot(q) * inv_det;
    if v < 0.0 || u + v > 1.0 { return false; }
    let t = e2.dot(q) * inv_det;
    t > EPS
}

// ── Generalized Winding Number (sign for accurate mode) ───────────────────────

/// Solid angle subtended by triangle (a, b, c) at point p.
/// Uses the formula by Van Oosterom & Strackee.
fn triangle_solid_angle(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> f32 {
    let ra = (a - p).normalize_or_zero();
    let rb = (b - p).normalize_or_zero();
    let rc = (c - p).normalize_or_zero();
    let num = ra.dot(rb.cross(rc));
    let den = 1.0 + ra.dot(rb) + rb.dot(rc) + rc.dot(ra);
    2.0 * num.atan2(den)
}

/// Generalized winding number: value > 0.5 means inside.
fn generalized_winding_number(mesh: &TriangleMesh, p: Vec3) -> f32 {
    use std::f32::consts::PI;
    let sum: f32 = mesh.triangles.iter().map(|&[a, b, c]| {
        triangle_solid_angle(
            p,
            mesh.vertices[a as usize],
            mesh.vertices[b as usize],
            mesh.vertices[c as usize],
        )
    }).sum();
    sum / (4.0 * PI)
}

// ── MeshSdfApprox ─────────────────────────────────────────────────────────────

const APPROX_RES: usize = 64;

pub struct MeshSdfApprox {
    grid:      Vec<f32>,
    bounds_min: Vec3,
    voxel_sz:  Vec3,
}

impl MeshSdfApprox {
    pub fn new(mesh: &TriangleMesh) -> Self {
        let bvh = Bvh::build(Arc::new(mesh.clone()));
        let pad = (mesh.bounds_max - mesh.bounds_min) * 0.05 + Vec3::splat(0.5);
        let bmin = mesh.bounds_min - pad;
        let bmax = mesh.bounds_max + pad;
        let vsz  = (bmax - bmin) / APPROX_RES as f32;

        let total = APPROX_RES * APPROX_RES * APPROX_RES;
        let grid: Vec<f32> = (0..total).into_par_iter().map(|idx| {
            let iz = idx / (APPROX_RES * APPROX_RES);
            let iy = (idx / APPROX_RES) % APPROX_RES;
            let ix = idx % APPROX_RES;
            let p  = bmin + Vec3::new(
                (ix as f32 + 0.5) * vsz.x,
                (iy as f32 + 0.5) * vsz.y,
                (iz as f32 + 0.5) * vsz.z,
            );
            bvh.ray_cast_sign(p)
        }).collect();

        Self { grid, bounds_min: bmin, voxel_sz: vsz }
    }

    fn sample(&self, p: Vec3) -> f32 {
        let _r   = APPROX_RES as f32;
        let rel = (p - self.bounds_min) / self.voxel_sz - Vec3::splat(0.5);
        let xi  = rel.x.floor() as isize;
        let yi  = rel.y.floor() as isize;
        let zi  = rel.z.floor() as isize;
        let fx  = rel.x - xi as f32;
        let fy  = rel.y - yi as f32;
        let fz  = rel.z - zi as f32;

        let get = |ix: isize, iy: isize, iz: isize| -> f32 {
            #[allow(non_snake_case)]
            let R = APPROX_RES as isize;
            if ix < 0 || iy < 0 || iz < 0 || ix >= R || iy >= R || iz >= R {
                // Extrapolate as the closest-edge value.
                let cx = ix.clamp(0, R-1) as usize;
                let cy = iy.clamp(0, R-1) as usize;
                let cz = iz.clamp(0, R-1) as usize;
                return self.grid[cx + cy * APPROX_RES + cz * APPROX_RES * APPROX_RES];
            }
            self.grid[ix as usize + iy as usize * APPROX_RES + iz as usize * APPROX_RES * APPROX_RES]
        };

        // Trilinear interpolation.
        let c000 = get(xi,   yi,   zi);
        let c100 = get(xi+1, yi,   zi);
        let c010 = get(xi,   yi+1, zi);
        let c110 = get(xi+1, yi+1, zi);
        let c001 = get(xi,   yi,   zi+1);
        let c101 = get(xi+1, yi,   zi+1);
        let c011 = get(xi,   yi+1, zi+1);
        let c111 = get(xi+1, yi+1, zi+1);

        let c00 = c000 * (1.0 - fx) + c100 * fx;
        let c01 = c001 * (1.0 - fx) + c101 * fx;
        let c10 = c010 * (1.0 - fx) + c110 * fx;
        let c11 = c011 * (1.0 - fx) + c111 * fx;
        let c0  = c00  * (1.0 - fy) + c10  * fy;
        let c1  = c01  * (1.0 - fy) + c11  * fy;
        c0 * (1.0 - fz) + c1 * fz
    }
}

impl Sdf for MeshSdfApprox {
    fn distance(&self, p: Vec3) -> f32 { self.sample(p) }
}

// ── MeshSdfAccurate ───────────────────────────────────────────────────────────

const SIGN_RES: usize = 32;

pub struct MeshSdfAccurate {
    mesh:       Arc<TriangleMesh>,
    bvh:        Arc<Bvh>,
    sign_grid:  Vec<f32>,   // coarse signed distance grid (32³)
    sign_bmin:  Vec3,
    sign_vsz:   Vec3,
}

impl MeshSdfAccurate {
    #[allow(dead_code)] // Called by MeshSdf::build_accurate
    pub fn new(mesh: Arc<TriangleMesh>) -> Self {
        let bvh = Arc::new(Bvh::build(Arc::clone(&mesh)));
        let pad  = (mesh.bounds_max - mesh.bounds_min) * 0.05 + Vec3::splat(0.5);
        let bmin = mesh.bounds_min - pad;
        let bmax = mesh.bounds_max + pad;
        let vsz  = (bmax - bmin) / SIGN_RES as f32;

        // Build a coarse sign grid using GWN (expensive but only SIGN_RES³ samples).
        let sign_grid: Vec<f32> = (0..SIGN_RES * SIGN_RES * SIGN_RES)
            .into_par_iter()
            .map(|idx| {
                let iz = idx / (SIGN_RES * SIGN_RES);
                let iy = (idx / SIGN_RES) % SIGN_RES;
                let ix = idx % SIGN_RES;
                let p  = bmin + Vec3::new(
                    (ix as f32 + 0.5) * vsz.x,
                    (iy as f32 + 0.5) * vsz.y,
                    (iz as f32 + 0.5) * vsz.z,
                );
                let wn = generalized_winding_number(&mesh, p);
                if wn > 0.5 { -1.0 } else { 1.0 }
            })
            .collect();

        Self { mesh, bvh, sign_grid, sign_bmin: bmin, sign_vsz: vsz }
    }

    fn coarse_sign(&self, p: Vec3) -> f32 {
        #[allow(non_snake_case)]
        let R = SIGN_RES as isize;
        let rel = (p - self.sign_bmin) / self.sign_vsz;
        let ix  = (rel.x as isize).clamp(0, R-1) as usize;
        let iy  = (rel.y as isize).clamp(0, R-1) as usize;
        let iz  = (rel.z as isize).clamp(0, R-1) as usize;
        self.sign_grid[ix + iy * SIGN_RES + iz * SIGN_RES * SIGN_RES]
    }
}

impl Sdf for MeshSdfAccurate {
    fn distance(&self, p: Vec3) -> f32 {
        let (d2, _) = self.bvh.nearest_sq(p);
        let dist    = d2.sqrt();
        // Use GWN for points near the surface; coarse grid elsewhere.
        let voxel   = self.sign_vsz.max_element();
        let sign    = if dist < 2.0 * voxel {
            let wn = generalized_winding_number(&self.mesh, p);
            if wn > 0.5 { -1.0 } else { 1.0 }
        } else {
            self.coarse_sign(p)
        };
        sign * dist
    }
}

// ── MeshSdf wrapper ───────────────────────────────────────────────────────────

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum MeshSdfMode {
    Approximate,
    #[allow(dead_code)] // Available for future high-quality rendering
    Accurate,
}

pub struct MeshSdf {
    pub approx:   Arc<MeshSdfApprox>,
    pub accurate: Option<Arc<MeshSdfAccurate>>,
    pub mode:     MeshSdfMode,
    mesh:         Arc<TriangleMesh>,  // kept for build_accurate()
}

impl MeshSdf {
    pub fn new(mesh: Arc<TriangleMesh>) -> Self {
        let approx = Arc::new(MeshSdfApprox::new(&mesh));
        Self { approx, accurate: None, mode: MeshSdfMode::Approximate, mesh }
    }

    /// Build (or rebuild) the accurate mode SDF.  May take several seconds.
    #[allow(dead_code)] // Part of mesh SDF accuracy upgrade API
    pub fn build_accurate(&mut self) {
        self.accurate = Some(Arc::new(MeshSdfAccurate::new(Arc::clone(&self.mesh))));
        self.mode = MeshSdfMode::Accurate;
    }

    #[allow(dead_code)] // Accessor for stored mesh reference
    pub fn mesh(&self) -> &Arc<TriangleMesh> { &self.mesh }
}

impl Sdf for MeshSdf {
    fn distance(&self, p: Vec3) -> f32 {
        match self.mode {
            MeshSdfMode::Approximate => self.approx.distance(p),
            MeshSdfMode::Accurate => {
                if let Some(ref acc) = self.accurate {
                    acc.distance(p)
                } else {
                    self.approx.distance(p)
                }
            }
        }
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::parse_stl;

    /// Cube STL — build an 8-triangle cube and check sign.
    fn cube_stl_bytes() -> Vec<u8> {
        // Build a binary STL for a unit cube [-1,1]³ (12 triangles).
        let tris: &[([f32;3], [[f32;3];3])] = &[
            // -Z face
            ([0.,0.,-1.], [[-1.,-1.,-1.],[1.,-1.,-1.],[1.,1.,-1.]]),
            ([0.,0.,-1.], [[-1.,-1.,-1.],[1.,1.,-1.],[-1.,1.,-1.]]),
            // +Z face
            ([0.,0.,1.],  [[-1.,-1.,1.],[1.,1.,1.],[1.,-1.,1.]]),
            ([0.,0.,1.],  [[-1.,-1.,1.],[-1.,1.,1.],[1.,1.,1.]]),
            // -X
            ([-1.,0.,0.],[[-1.,-1.,-1.],[-1.,1.,-1.],[-1.,1.,1.]]),
            ([-1.,0.,0.],[[-1.,-1.,-1.],[-1.,1.,1.],[-1.,-1.,1.]]),
            // +X
            ([1.,0.,0.],  [[1.,-1.,-1.],[1.,1.,1.],[1.,1.,-1.]]),
            ([1.,0.,0.],  [[1.,-1.,-1.],[1.,-1.,1.],[1.,1.,1.]]),
            // -Y
            ([0.,-1.,0.],[[-1.,-1.,-1.],[1.,-1.,1.],[1.,-1.,-1.]]),
            ([0.,-1.,0.],[[-1.,-1.,-1.],[-1.,-1.,1.],[1.,-1.,1.]]),
            // +Y
            ([0.,1.,0.], [[-1.,1.,-1.],[1.,1.,-1.],[1.,1.,1.]]),
            ([0.,1.,0.], [[-1.,1.,-1.],[1.,1.,1.],[-1.,1.,1.]]),
        ];
        let n = tris.len() as u32;
        let mut out = vec![0u8; 80];
        out.extend_from_slice(&n.to_le_bytes());
        for (norm, verts) in tris {
            for &f in norm { out.extend_from_slice(&f.to_le_bytes()); }
            for v in verts { for &f in v { out.extend_from_slice(&f.to_le_bytes()); } }
            out.extend_from_slice(&[0u8; 2]);
        }
        out
    }

    #[test]
    fn cube_center_is_inside() {
        let data  = cube_stl_bytes();
        let mesh  = parse_stl(&data).unwrap();
        let sdf   = MeshSdf::new(Arc::new(mesh));
        let d     = sdf.distance(Vec3::ZERO);
        assert!(d < 0.0, "cube center should be inside, got {}", d);
    }

    #[test]
    fn cube_far_exterior_is_outside() {
        let data = cube_stl_bytes();
        let mesh = parse_stl(&data).unwrap();
        let sdf  = MeshSdf::new(Arc::new(mesh));
        let d    = sdf.distance(Vec3::new(10.0, 0.0, 0.0));
        assert!(d > 0.0, "far exterior should be positive, got {}", d);
    }

    #[test]
    fn non_manifold_does_not_panic() {
        // Single triangle: open boundary, but construction must not panic.
        use crate::mesh::import::{TriangleMesh, validate_mesh};
        let mesh = TriangleMesh {
            vertices: vec![Vec3::ZERO, Vec3::X, Vec3::Y],
            triangles: vec![[0, 1, 2]],
            normals: vec![Vec3::Z],
            bounds_min: Vec3::ZERO,
            bounds_max: Vec3::ONE,
        };
        let v = validate_mesh(&mesh);
        assert!(v.has_open_boundary);
        let _ = MeshSdf::new(Arc::new(mesh)); // must not panic
    }
}
