// Voxel-based tetrahedral meshing from an SDF.
//
// Samples the SDF on a regular grid and applies the Freudenthal 6-tet
// decomposition of each grid cube, retaining only tets whose four nodes are
// all strictly inside the solid (SDF < 0).  The result is a valid C3D4
// volumetric tet mesh suitable for CalculiX linear-static analysis.
//
// Node indices in the output mesh are 1-based (CalculiX convention).

use glam::Vec3;
use crate::sdf::Sdf;
use rayon::prelude::*;

pub struct TetMesh {
    /// World-space positions of all nodes (0-indexed internally).
    pub nodes:    Vec<Vec3>,
    /// Four 0-based node indices per element.
    pub elements: Vec<[usize; 4]>,
}

// Freudenthal 6-tet decomposition of a unit cube.
// Corner encoding: bit0=dx, bit1=dy, bit2=dz  →  (000..111) = corners 0..7.
const FREUDENTHAL: [[usize; 4]; 6] = [
    [0, 1, 3, 7],
    [0, 3, 2, 7],
    [0, 4, 5, 7],
    [0, 5, 1, 7],
    [0, 6, 7, 4],
    [0, 2, 7, 6],
];

/// Build a tetrahedral mesh from `sdf` within `bounds_min`..`bounds_max` using
/// a `resolution × resolution × resolution` node grid.
///
/// Returns `Err` if the mesh is empty or has too few nodes.
pub fn voxel_tet_mesh(
    sdf:         &dyn Sdf,
    bounds_min:  Vec3,
    bounds_max:  Vec3,
    resolution:  u32,
) -> Result<TetMesh, String> {
    let res   = resolution as usize;
    let span  = bounds_max - bounds_min;
    let step  = span / (res as f32 - 1.0).max(1.0);

    // Evaluate SDF in parallel on all grid nodes.
    let total = res * res * res;
    let distances: Vec<f32> = (0..total)
        .into_par_iter()
        .map(|idx| {
            let ix = idx % res;
            let iy = (idx / res) % res;
            let iz = idx / (res * res);
            let p  = bounds_min + Vec3::new(
                ix as f32 * step.x,
                iy as f32 * step.y,
                iz as f32 * step.z,
            );
            sdf.distance(p)
        })
        .collect();

    let nidx = |ix: usize, iy: usize, iz: usize| ix + iy * res + iz * res * res;

    let mut elements: Vec<[usize; 4]> = Vec::new();
    for iz in 0..res - 1 {
        for iy in 0..res - 1 {
            for ix in 0..res - 1 {
                let corners: [usize; 8] = [
                    nidx(ix,   iy,   iz  ),
                    nidx(ix+1, iy,   iz  ),
                    nidx(ix,   iy+1, iz  ),
                    nidx(ix+1, iy+1, iz  ),
                    nidx(ix,   iy,   iz+1),
                    nidx(ix+1, iy,   iz+1),
                    nidx(ix,   iy+1, iz+1),
                    nidx(ix+1, iy+1, iz+1),
                ];
                for pat in &FREUDENTHAL {
                    let tet = [corners[pat[0]], corners[pat[1]], corners[pat[2]], corners[pat[3]]];
                    if tet.iter().all(|&ni| distances[ni] < 0.0) {
                        elements.push(tet);
                    }
                }
            }
        }
    }

    if elements.is_empty() {
        return Err(
            "No solid elements found. Check that the SDF has interior volume \
             and increase mesh resolution.".into()
        );
    }

    // Compact: drop unreferenced nodes, build remapping.
    let mut used = vec![false; total];
    for tet in &elements { for &ni in tet { used[ni] = true; } }

    let mut old_to_new = vec![usize::MAX; total];
    let mut nodes: Vec<Vec3> = Vec::new();
    for (i, &u) in used.iter().enumerate() {
        if u {
            let ix = i % res;
            let iy = (i / res) % res;
            let iz = i / (res * res);
            old_to_new[i] = nodes.len();
            nodes.push(bounds_min + Vec3::new(
                ix as f32 * step.x,
                iy as f32 * step.y,
                iz as f32 * step.z,
            ));
        }
    }

    let elements: Vec<[usize; 4]> = elements.iter()
        .map(|t| [old_to_new[t[0]], old_to_new[t[1]], old_to_new[t[2]], old_to_new[t[3]]])
        .collect();

    if nodes.len() < 4 {
        return Err(format!("Too few nodes ({}) — increase mesh resolution.", nodes.len()));
    }

    // Validate: check for degenerate tets (all 4 nodes identical).
    let degenerate = elements.iter().filter(|t| {
        t[0] == t[1] || t[0] == t[2] || t[0] == t[3] || t[1] == t[2] || t[1] == t[3] || t[2] == t[3]
    }).count();
    if degenerate > 0 {
        return Err(format!(
            "{degenerate} degenerate tetrahedra found — mesh is not suitable for FEA."
        ));
    }

    Ok(TetMesh { nodes, elements })
}
