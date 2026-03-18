// STL and OBJ mesh parsers.
//
// Returns a TriangleMesh with vertices, indexed triangles, per-triangle normals,
// and axis-aligned bounding box.  Non-manifold and open-boundary meshes are
// accepted with warnings in MeshValidationResult.

use std::collections::HashMap;
use glam::Vec3;

// ── TriangleMesh ──────────────────────────────────────────────────────────────

#[derive(Clone, Debug)]
#[allow(dead_code)]
pub struct TriangleMesh {
    pub vertices:   Vec<Vec3>,
    pub triangles:  Vec<[u32; 3]>,
    /// Per-triangle face normals (unit, or zero for degenerate triangles).
    pub normals:    Vec<Vec3>,
    pub bounds_min: Vec3,
    pub bounds_max: Vec3,
}

impl TriangleMesh {
    pub fn vertex_count(&self)   -> usize { self.vertices.len() }
    pub fn triangle_count(&self) -> usize { self.triangles.len() }

    /// Triangle centroid.
    #[allow(dead_code)] // Used in hole detection analysis
    pub fn triangle_centroid(&self, tri_idx: usize) -> Vec3 {
        let [a, b, c] = self.triangles[tri_idx];
        (self.vertices[a as usize]
             + self.vertices[b as usize]
             + self.vertices[c as usize]) / 3.0
    }
}

// ── Validation ────────────────────────────────────────────────────────────────

#[derive(Clone, Debug, Default)]
#[allow(dead_code)]
pub struct MeshValidationResult {
    pub degenerate_count:    u32,
    pub has_degenerate:      bool,
    pub has_non_manifold:    bool,
    pub has_open_boundary:   bool,
}

pub fn validate_mesh(mesh: &TriangleMesh) -> MeshValidationResult {
    let mut edge_count: HashMap<(u32, u32), u32> = HashMap::new();
    let mut degenerate = 0u32;

    for (i, &[a, b, c]) in mesh.triangles.iter().enumerate() {
        // Check for degenerate (zero-area) triangles.
        let va = mesh.vertices[a as usize];
        let vb = mesh.vertices[b as usize];
        let vc = mesh.vertices[c as usize];
        let area2 = (vb - va).cross(vc - va).length_squared();
        if area2 < 1e-16 { degenerate += 1; }

        // Count directed half-edges for manifold / open-boundary check.
        for (ea, eb) in [(a, b), (b, c), (c, a)] {
            let key = if ea < eb { (ea, eb) } else { (eb, ea) };
            *edge_count.entry(key).or_insert(0) += 1;
            let _ = i; // silence unused
        }
    }

    let mut non_manifold  = false;
    let mut open_boundary = false;
    for &count in edge_count.values() {
        if count > 2 { non_manifold  = true; }
        if count < 2 { open_boundary = true; }
    }

    MeshValidationResult {
        degenerate_count:  degenerate,
        has_degenerate:    degenerate > 0,
        has_non_manifold:  non_manifold,
        has_open_boundary: open_boundary,
    }
}

// ── Internal helpers ──────────────────────────────────────────────────────────

fn compute_bounds(vertices: &[Vec3]) -> (Vec3, Vec3) {
    let mut mn = Vec3::splat(f32::MAX);
    let mut mx = Vec3::splat(f32::MIN);
    for &v in vertices {
        mn = mn.min(v);
        mx = mx.max(v);
    }
    (mn, mx)
}

fn compute_normals(vertices: &[Vec3], triangles: &[[u32; 3]]) -> Vec<Vec3> {
    triangles.iter().map(|&[a, b, c]| {
        let va = vertices[a as usize];
        let vb = vertices[b as usize];
        let vc = vertices[c as usize];
        (vb - va).cross(vc - va).normalize_or_zero()
    }).collect()
}

fn build_mesh(vertices: Vec<Vec3>, triangles: Vec<[u32; 3]>) -> TriangleMesh {
    let (bounds_min, bounds_max) = compute_bounds(&vertices);
    let normals = compute_normals(&vertices, &triangles);
    TriangleMesh { vertices, triangles, normals, bounds_min, bounds_max }
}

// ── Binary STL parser ─────────────────────────────────────────────────────────

pub fn parse_stl(data: &[u8]) -> Result<TriangleMesh, String> {
    // Detect ASCII vs binary: ASCII starts with "solid " (after trimming whitespace).
    // A reliable heuristic is: try binary first (check length), fall back to ASCII.
    if is_binary_stl(data) {
        parse_binary_stl(data)
    } else {
        let text = std::str::from_utf8(data)
            .map_err(|e| format!("STL: UTF-8 decode failed: {}", e))?;
        parse_ascii_stl(text)
    }
}

fn is_binary_stl(data: &[u8]) -> bool {
    if data.len() < 84 { return false; }
    // ASCII STL must start with "solid" (possibly with BOM / whitespace).
    let head = std::str::from_utf8(&data[..80]).unwrap_or("");
    if head.trim_start().starts_with("solid") {
        // Still might be binary — check if expected binary size matches.
        let tri_count = u32::from_le_bytes([data[80], data[81], data[82], data[83]]) as usize;
        let expected  = 84 + tri_count * 50;
        if expected == data.len() { return true; }
        return false; // it is ASCII
    }
    true
}

fn parse_binary_stl(data: &[u8]) -> Result<TriangleMesh, String> {
    if data.len() < 84 {
        return Err("STL binary: too short".to_string());
    }
    let tri_count = u32::from_le_bytes([data[80], data[81], data[82], data[83]]) as usize;
    let expected  = 84 + tri_count * 50;
    if data.len() < expected {
        return Err(format!(
            "STL binary: expected {} bytes for {} triangles, got {}",
            expected, tri_count, data.len()
        ));
    }

    let mut vertices  = Vec::with_capacity(tri_count * 3);
    let mut triangles = Vec::with_capacity(tri_count);

    for i in 0..tri_count {
        let base = 84 + i * 50;
        // Skip normal (12 bytes), read 3 vertices (9 × 4 bytes).
        let mut v = [Vec3::ZERO; 3];
        for (j, vi) in v.iter_mut().enumerate() {
            let off = base + 12 + j * 12;
            let x = f32::from_le_bytes([data[off],   data[off+1], data[off+2],  data[off+3]]);
            let y = f32::from_le_bytes([data[off+4], data[off+5], data[off+6],  data[off+7]]);
            let z = f32::from_le_bytes([data[off+8], data[off+9], data[off+10], data[off+11]]);
            *vi = Vec3::new(x, y, z);
        }
        let base_idx = vertices.len() as u32;
        vertices.push(v[0]);
        vertices.push(v[1]);
        vertices.push(v[2]);
        triangles.push([base_idx, base_idx + 1, base_idx + 2]);
    }

    Ok(build_mesh(vertices, triangles))
}

fn parse_ascii_stl(text: &str) -> Result<TriangleMesh, String> {
    let mut vertices  = Vec::new();
    let mut triangles = Vec::new();
    let mut tri_verts = Vec::<Vec3>::new();

    for line in text.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with("vertex ") {
            let parts: Vec<&str> = trimmed.split_whitespace().collect();
            if parts.len() >= 4 {
                let x: f32 = parts[1].parse().unwrap_or(0.0);
                let y: f32 = parts[2].parse().unwrap_or(0.0);
                let z: f32 = parts[3].parse().unwrap_or(0.0);
                tri_verts.push(Vec3::new(x, y, z));
            }
        } else if trimmed.starts_with("endfacet") {
            if tri_verts.len() == 3 {
                let base = vertices.len() as u32;
                vertices.push(tri_verts[0]);
                vertices.push(tri_verts[1]);
                vertices.push(tri_verts[2]);
                triangles.push([base, base + 1, base + 2]);
            }
            tri_verts.clear();
        }
    }

    if triangles.is_empty() {
        return Err("STL ASCII: no triangles found".to_string());
    }
    Ok(build_mesh(vertices, triangles))
}

// ── OBJ parser ────────────────────────────────────────────────────────────────

pub fn parse_obj(text: &str) -> Result<TriangleMesh, String> {
    let mut vertices  = Vec::<Vec3>::new();
    let mut triangles = Vec::<[u32; 3]>::new();

    for line in text.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with('#') || trimmed.is_empty() { continue; }

        let parts: Vec<&str> = trimmed.split_whitespace().collect();
        if parts.is_empty() { continue; }

        match parts[0] {
            "v" if parts.len() >= 4 => {
                let x: f32 = parts[1].parse().unwrap_or(0.0);
                let y: f32 = parts[2].parse().unwrap_or(0.0);
                let z: f32 = parts[3].parse().unwrap_or(0.0);
                vertices.push(Vec3::new(x, y, z));
            }
            "f" if parts.len() >= 4 => {
                // Parse face indices (1-based, optional /vt/vn suffixes).
                let idx: Vec<u32> = parts[1..].iter().map(|s| {
                    let face_idx = s.split('/').next().unwrap_or("0");
                    let i: i32 = face_idx.parse().unwrap_or(1);
                    // Negative indices count from end.
                    if i < 0 { (vertices.len() as i32 + i) as u32 }
                    else { (i - 1) as u32 }
                }).collect();
                // Fan triangulation: (0,1,2), (0,2,3), ...
                for k in 1..idx.len().saturating_sub(1) {
                    triangles.push([idx[0], idx[k], idx[k + 1]]);
                }
            }
            _ => {}
        }
    }

    if vertices.is_empty() {
        return Err("OBJ: no vertices found".to_string());
    }
    if triangles.is_empty() {
        return Err("OBJ: no faces found".to_string());
    }
    Ok(build_mesh(vertices, triangles))
}

// ── Tests ──────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_ascii_stl_cube() {
        let stl = b"solid cube
  facet normal 0 0 -1
    outer loop
      vertex -1 -1 -1
      vertex  1 -1 -1
      vertex  1  1 -1
    endloop
  endfacet
  facet normal 0 0 -1
    outer loop
      vertex -1 -1 -1
      vertex  1  1 -1
      vertex -1  1 -1
    endloop
  endfacet
endsolid cube";
        let mesh = parse_stl(stl).unwrap();
        assert_eq!(mesh.triangle_count(), 2);
        assert_eq!(mesh.vertex_count(), 6);
    }

    #[test]
    fn parse_obj_quad_triangulation() {
        let obj = "v -1 -1  0\nv  1 -1  0\nv  1  1  0\nv -1  1  0\nf 1 2 3 4\n";
        let mesh = parse_obj(obj).unwrap();
        assert_eq!(mesh.triangle_count(), 2);
    }

    #[test]
    fn validation_detects_open_boundary() {
        // Single triangle — all 3 edges appear only once → open boundary.
        let mesh = TriangleMesh {
            vertices: vec![
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            ],
            triangles: vec![[0, 1, 2]],
            normals: vec![Vec3::Z],
            bounds_min: Vec3::ZERO,
            bounds_max: Vec3::new(1.0, 1.0, 0.0),
        };
        let r = validate_mesh(&mesh);
        assert!(r.has_open_boundary);
    }
}
