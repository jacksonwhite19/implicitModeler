// Assembly interference checking — voxel-based overlap volume estimation
// between component keepout volumes.
#![allow(dead_code)] // Analysis result fields — not all displayed in the current UI

use glam::Vec3;
use std::sync::Arc;
use crate::sdf::Sdf;
use crate::sdf::query::bounding_points;
use rayon::prelude::*;

#[derive(Clone, Debug, PartialEq)]
pub enum InterferenceSeverity {
    Negligible, // < 1mm³
    Minor,      // 1-100mm³
    Moderate,   // 100-1000mm³
    Critical,   // > 1000mm³
}

#[derive(Clone, Debug)]
pub struct InterferencePair {
    pub component_a: String,
    pub component_b: String,
    pub interference_volume_mm3: f32,
    pub interference_centroid: Vec3,
    pub severity: InterferenceSeverity,
    pub description: String,
}

#[derive(Clone, Debug)]
pub struct InterferenceResult {
    pub pairs: Vec<InterferencePair>,
    pub total_interference_count: usize,
    pub has_critical_interference: bool,
    /// Component names whose keepout volume extends outside the parent boundary.
    pub outside_parent: Vec<String>,
}

/// Check all component pairs for keepout volume overlap.
///
/// - `components`: `(name, keepout_sdf, geometry_sdf)` — keepout is used for overlap tests.
/// - `parent_sdf`: optional outer boundary; components outside it are flagged.
/// - `resolution`: voxel grid resolution per axis (e.g. 12 → 12³ voxels per overlapping bbox).
pub fn check_assembly_interference(
    components: &[(String, Arc<dyn Sdf>, Arc<dyn Sdf>)],
    parent_sdf: Option<Arc<dyn Sdf>>,
    resolution: usize,
) -> InterferenceResult {
    let n = components.len();

    // Pre-compute bounding boxes for all keepout volumes.
    let bboxes: Vec<_> = components
        .iter()
        .map(|(_, keepout, _)| bounding_points(&**keepout))
        .collect();

    // Check all unique pairs in parallel.
    let pairs: Vec<InterferencePair> = (0..n)
        .into_par_iter()
        .flat_map(|i| {
            let bboxes_ref = &bboxes;
            let components_ref = &components;
            let res = resolution;
            (i + 1..n)
                .into_par_iter()
                .filter_map(move |j| {
                    let bi = &bboxes_ref[i];
                    let bj = &bboxes_ref[j];

                    // Axis-aligned bounding box pre-filter.
                    if bi.max.x < bj.min.x || bj.max.x < bi.min.x { return None; }
                    if bi.max.y < bj.min.y || bj.max.y < bi.min.y { return None; }
                    if bi.max.z < bj.min.z || bj.max.z < bi.min.z { return None; }

                    // Intersection bounding box.
                    let overlap_min = Vec3::new(
                        bi.min.x.max(bj.min.x),
                        bi.min.y.max(bj.min.y),
                        bi.min.z.max(bj.min.z),
                    );
                    let overlap_max = Vec3::new(
                        bi.max.x.min(bj.max.x),
                        bi.max.y.min(bj.max.y),
                        bi.max.z.min(bj.max.z),
                    );

                    // Sanity: bbox must have positive volume.
                    if overlap_max.x <= overlap_min.x
                        || overlap_max.y <= overlap_min.y
                        || overlap_max.z <= overlap_min.z
                    {
                        return None;
                    }

                    let res_f = res as f32;
                    let span = overlap_max - overlap_min;
                    let dx = span / res_f;
                    let voxel_vol = dx.x * dx.y * dx.z;

                    let sdf_a = &components_ref[i].1;
                    let sdf_b = &components_ref[j].1;

                    let mut vol = 0.0f32;
                    let mut centroid_sum = Vec3::ZERO;
                    let mut count = 0u32;

                    for ix in 0..res {
                        for iy in 0..res {
                            for iz in 0..res {
                                let p = overlap_min + Vec3::new(
                                    (ix as f32 + 0.5) * dx.x,
                                    (iy as f32 + 0.5) * dx.y,
                                    (iz as f32 + 0.5) * dx.z,
                                );
                                if sdf_a.distance(p) < 0.0 && sdf_b.distance(p) < 0.0 {
                                    vol += voxel_vol;
                                    centroid_sum += p;
                                    count += 1;
                                }
                            }
                        }
                    }

                    if vol < 0.1 {
                        return None;
                    }

                    let severity = if vol < 1.0 {
                        InterferenceSeverity::Negligible
                    } else if vol < 100.0 {
                        InterferenceSeverity::Minor
                    } else if vol < 1000.0 {
                        InterferenceSeverity::Moderate
                    } else {
                        InterferenceSeverity::Critical
                    };

                    let centroid = if count > 0 {
                        centroid_sum / count as f32
                    } else {
                        (overlap_min + overlap_max) * 0.5
                    };

                    let name_a = &components_ref[i].0;
                    let name_b = &components_ref[j].0;

                    Some(InterferencePair {
                        component_a: name_a.clone(),
                        component_b: name_b.clone(),
                        interference_volume_mm3: vol,
                        interference_centroid: centroid,
                        severity: severity.clone(),
                        description: format!(
                            "{:.1}mm³ overlap between {} and {}",
                            vol, name_a, name_b
                        ),
                    })
                })
                .collect::<Vec<_>>()
        })
        .collect();

    // Check containment within parent boundary.
    let outside_parent: Vec<String> = if let Some(ref parent) = parent_sdf {
        components
            .iter()
            .zip(bboxes.iter())
            .filter_map(|((name, _, _), bbox)| {
                // Any corner of the keepout bbox outside parent by > 5mm → flag it.
                let outside = bbox.corners.iter().any(|&corner| parent.distance(corner) > 5.0);
                if outside { Some(name.clone()) } else { None }
            })
            .collect()
    } else {
        vec![]
    };

    let total = pairs.len();
    let critical = pairs.iter().any(|p| p.severity == InterferenceSeverity::Critical);

    // Sort by descending interference volume.
    let mut sorted_pairs = pairs;
    sorted_pairs.sort_by(|a, b| {
        b.interference_volume_mm3
            .partial_cmp(&a.interference_volume_mm3)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    InterferenceResult {
        pairs: sorted_pairs,
        total_interference_count: total,
        has_critical_interference: critical,
        outside_parent,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::primitives::Sphere;
    use crate::sdf::transforms::Translate;

    #[test]
    fn test_non_overlapping_spheres_no_interference() {
        let s1: Arc<dyn Sdf> = Arc::new(Sphere::new(10.0));
        let s2: Arc<dyn Sdf> = Arc::new(Translate::new(
            Arc::new(Sphere::new(10.0)),
            Vec3::new(50.0, 0.0, 0.0),
        ));
        let components = vec![
            ("a".to_string(), s1.clone(), s1),
            ("b".to_string(), s2.clone(), s2),
        ];
        let result = check_assembly_interference(&components, None, 8);
        assert_eq!(
            result.total_interference_count, 0,
            "Non-overlapping spheres should not interfere"
        );
    }

    #[test]
    fn test_overlapping_spheres_have_interference() {
        // Two spheres of radius 10 centered at 0 and 5 — clearly overlapping
        let s1: Arc<dyn Sdf> = Arc::new(Sphere::new(10.0));
        let s2: Arc<dyn Sdf> = Arc::new(Translate::new(
            Arc::new(Sphere::new(10.0)),
            Vec3::new(5.0, 0.0, 0.0),
        ));
        let components = vec![
            ("a".to_string(), s1.clone(), s1),
            ("b".to_string(), s2.clone(), s2),
        ];
        let result = check_assembly_interference(&components, None, 8);
        assert!(
            result.total_interference_count > 0,
            "Overlapping spheres should interfere"
        );
        assert!(result.pairs[0].interference_volume_mm3 > 0.0);
    }

    #[test]
    fn test_outside_parent_detected() {
        let parent: Arc<dyn Sdf> = Arc::new(Sphere::new(20.0));
        // Component far outside parent
        let comp: Arc<dyn Sdf> = Arc::new(Translate::new(
            Arc::new(Sphere::new(5.0)),
            Vec3::new(100.0, 0.0, 0.0),
        ));
        let components = vec![("outside".to_string(), comp.clone(), comp)];
        let result = check_assembly_interference(&components, Some(parent), 8);
        assert!(result.outside_parent.contains(&"outside".to_string()));
    }

    #[test]
    fn test_empty_components() {
        let result = check_assembly_interference(&[], None, 8);
        assert_eq!(result.total_interference_count, 0);
        assert!(!result.has_critical_interference);
        assert!(result.outside_parent.is_empty());
    }
}
