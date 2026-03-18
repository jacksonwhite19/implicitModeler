// Split-body operation: divide an SDF into two halves along a plane and add
// alignment features so the printed parts mate correctly.

use std::sync::Arc;
use glam::Vec3;
use crate::sdf::Sdf;
use crate::sdf::booleans::Intersect;
use super::alignment::AlignmentFeature;

// ── SplitPlane ───────────────────────────────────────────────────────────────

/// A plane that divides space into two halves.
///
/// "Positive" side: `signed_distance(p) > 0`.
/// "Negative" side: `signed_distance(p) < 0`.
#[derive(Clone, Debug)]
pub enum SplitPlane {
    /// Cut perpendicular to X at the given position.
    X(f32),
    /// Cut perpendicular to Y at the given position.
    Y(f32),
    /// Cut perpendicular to Z at the given position.
    Z(f32),
    /// Arbitrary plane: `normal·p - distance = 0`.  `normal` need not be unit.
    Arbitrary { normal: Vec3, distance: f32 },
}

impl SplitPlane {
    /// Signed distance from `point` to the plane (positive on the "top" side).
    #[allow(dead_code)] // Part of split plane geometry API
    pub fn signed_distance(&self, point: Vec3) -> f32 {
        match self {
            SplitPlane::X(d) => point.x - d,
            SplitPlane::Y(d) => point.y - d,
            SplitPlane::Z(d) => point.z - d,
            SplitPlane::Arbitrary { normal, distance } => {
                let n = normal.normalize();
                point.dot(n) - distance
            }
        }
    }

    /// Outward normal of the "positive" half-space (unit vector).
    #[allow(dead_code)] // Part of split plane geometry API
    pub fn normal(&self) -> Vec3 {
        match self {
            SplitPlane::X(_)                    => Vec3::X,
            SplitPlane::Y(_)                    => Vec3::Y,
            SplitPlane::Z(_)                    => Vec3::Z,
            SplitPlane::Arbitrary { normal, .. } => normal.normalize(),
        }
    }

    /// Center point on the plane (origin projected onto it).
    pub fn center(&self) -> Vec3 {
        match self {
            SplitPlane::X(d)                    => Vec3::new(*d, 0.0, 0.0),
            SplitPlane::Y(d)                    => Vec3::new(0.0, *d, 0.0),
            SplitPlane::Z(d)                    => Vec3::new(0.0, 0.0, *d),
            SplitPlane::Arbitrary { normal, distance } => {
                normal.normalize() * distance
            }
        }
    }

    /// Project `point` onto the plane surface.
    #[allow(dead_code)] // Part of split plane geometry API
    pub fn project(&self, point: Vec3) -> Vec3 {
        let n = self.normal();
        point - n * self.signed_distance(point)
    }

    /// An SDF representing the positive half-space (inside where SD > 0, i.e. use
    /// negated Plane so that distance < 0 means inside the half-space).
    pub fn positive_half_sdf(&self) -> Arc<dyn Sdf> {
        // Plane SDF: point.dot(normal) - d  → positive on normal side
        // Intersect wants negative = inside, so we negate: -(point.dot(n) - d)
        let n = self.normal();
        let d = match self {
            SplitPlane::X(v) => *v,
            SplitPlane::Y(v) => *v,
            SplitPlane::Z(v) => *v,
            SplitPlane::Arbitrary { distance, .. } => *distance,
        };
        // Negate: inside = point on positive side of plane → use -plane
        Arc::new(NegatedPlane { normal: n, distance: d })
    }

    /// An SDF representing the negative half-space.
    pub fn negative_half_sdf(&self) -> Arc<dyn Sdf> {
        let n = self.normal();
        let d = match self {
            SplitPlane::X(v) => *v,
            SplitPlane::Y(v) => *v,
            SplitPlane::Z(v) => *v,
            SplitPlane::Arbitrary { distance, .. } => *distance,
        };
        Arc::new(NegatedPlane { normal: -n, distance: -d })
    }

    /// Two unit vectors lying in the plane (for placing features at offsets).
    pub fn tangents(&self) -> (Vec3, Vec3) {
        let n = self.normal();
        // Pick a tangent not parallel to n
        let up = if n.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
        let t1 = n.cross(up).normalize();
        let t2 = n.cross(t1).normalize();
        (t1, t2)
    }
}

// Half-space SDF: distance < 0 when the point is on the positive side of the plane.
struct NegatedPlane {
    normal: Vec3,
    distance: f32,
}

impl Sdf for NegatedPlane {
    fn distance(&self, p: Vec3) -> f32 {
        // Plane SDF: positive on the normal side.
        // We want "inside" (negative) when p is on the normal side, so negate.
        -(p.dot(self.normal) - self.distance)
    }
}

// ── SplitResult ──────────────────────────────────────────────────────────────

/// The two halves produced by `split_body`.
pub struct SplitResult {
    /// Part on the positive side of the split plane, with alignment protrusions.
    pub part_a: Arc<dyn Sdf>,
    /// Part on the negative side of the split plane, with mating sockets/cavities.
    pub part_b: Arc<dyn Sdf>,
    pub plane: SplitPlane,
}

// ── split_body ───────────────────────────────────────────────────────────────

/// Split `body` at `plane`, adding `alignment` features so the parts can be
/// reassembled after printing.
pub fn split_body(
    body: Arc<dyn Sdf>,
    plane: &SplitPlane,
    alignment: &AlignmentFeature,
) -> SplitResult {
    // Carve the two halves
    let half_a_raw: Arc<dyn Sdf> = Arc::new(Intersect::new(
        Arc::clone(&body),
        plane.positive_half_sdf(),
    ));
    let half_b_raw: Arc<dyn Sdf> = Arc::new(Intersect::new(
        Arc::clone(&body),
        plane.negative_half_sdf(),
    ));

    // Generate alignment features
    let (part_a, part_b) = alignment.apply(half_a_raw, half_b_raw, plane);

    SplitResult { part_a, part_b, plane: plane.clone() }
}

/// Split `body` at multiple planes in sequence, returning one part per region
/// in order from most-negative to most-positive.
pub fn split_body_multi(
    body: Arc<dyn Sdf>,
    planes: &[(SplitPlane, AlignmentFeature)],
) -> Vec<Arc<dyn Sdf>> {
    if planes.is_empty() {
        return vec![body];
    }

    // Sort planes by their position along the plane normal (ascending).
    // For a plane with normal n at position p, the scalar is center·n.
    let mut sorted: Vec<&(SplitPlane, AlignmentFeature)> = planes.iter().collect();
    sorted.sort_by(|a, b| {
        let pa = a.0.center().dot(a.0.normal());
        let pb = b.0.center().dot(b.0.normal());
        pa.partial_cmp(&pb).unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut parts: Vec<Arc<dyn Sdf>> = Vec::new();
    let mut remainder = Arc::clone(&body);

    for (i, (plane, alignment)) in sorted.iter().enumerate() {
        let result = split_body(Arc::clone(&remainder), plane, alignment);
        parts.push(result.part_b);   // negative side = "earlier" piece
        remainder = result.part_a;    // positive side continues to next split
        let _ = i;
    }
    parts.push(remainder); // final positive-most piece
    parts
}

// ── SplitFitResult ───────────────────────────────────────────────────────────

pub struct SplitFitResult {
    pub fits: bool,
    pub interference_volume_mm3: f32,
    pub gap_volume_mm3: f32,
    pub warnings: Vec<String>,
}

/// Verify that the two halves in `result` mate correctly by sampling a 2D grid
/// on the split plane and checking for interference or gaps.
///
/// Uses a 32×32 sample grid on the plane, ±half_size from the plane centroid.
pub fn verify_split_fit(result: &SplitResult, half_size: f32) -> SplitFitResult {
    const N: usize = 32;
    let plane = &result.plane;
    let center = plane.center();
    let (t1, t2) = plane.tangents();
    let normal = plane.normal();

    let step = 2.0 * half_size / (N as f32);
    let voxel_vol = step * step * step;

    let mut interference = 0u32;
    let mut gap = 0u32;

    // Sample a thin slab around the cut plane
    for i in 0..N {
        for j in 0..N {
            let u = -half_size + (i as f32 + 0.5) * step;
            let v = -half_size + (j as f32 + 0.5) * step;
            let p_plane = center + t1 * u + t2 * v;

            // Check just above and below the plane
            for &side in &[-0.5f32, 0.5f32] {
                let p = p_plane + normal * (side * step);
                let da = result.part_a.distance(p);
                let db = result.part_b.distance(p);

                let in_a = da < 0.0;
                let in_b = db < 0.0;

                if in_a && in_b {
                    interference += 1;
                }
            }

            // Check exactly on the plane for gaps (neither part claims the boundary)
            let da = result.part_a.distance(p_plane);
            let db = result.part_b.distance(p_plane);
            if da > 0.5 * step && db > 0.5 * step {
                gap += 1;
            }
        }
    }

    let interference_volume = interference as f32 * voxel_vol;
    let gap_volume = gap as f32 * voxel_vol;

    let mut warnings = Vec::new();
    if interference_volume > 0.5 {
        warnings.push(format!(
            "Interference volume {:.2} mm³ > 0.5 mm³ — parts may not assemble without post-processing",
            interference_volume
        ));
    }
    if gap_volume > half_size * half_size * 0.05 {
        warnings.push(format!(
            "Gap volume {:.2} mm³ suggests poor face contact on the split plane",
            gap_volume
        ));
    }

    SplitFitResult {
        fits: warnings.is_empty(),
        interference_volume_mm3: interference_volume,
        gap_volume_mm3: gap_volume,
        warnings,
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::primitives::Sphere;

    #[test]
    fn split_z_sphere_top_inside() {
        let sphere: Arc<dyn Sdf> = Arc::new(Sphere::new(10.0));
        let plane = SplitPlane::Z(0.0);
        let result = split_body(sphere, &plane, &AlignmentFeature::None);

        // Top half: Z=+5 should be inside (negative SDF)
        let d = result.part_a.distance(Vec3::new(0.0, 0.0, 5.0));
        assert!(d < 0.0, "Z=+5 should be inside top half, got {}", d);

        // Top half: Z=-5 should be outside (positive SDF)
        let d = result.part_a.distance(Vec3::new(0.0, 0.0, -5.0));
        assert!(d > 0.0, "Z=-5 should be outside top half, got {}", d);
    }

    #[test]
    fn split_z_sphere_bottom_inside() {
        let sphere: Arc<dyn Sdf> = Arc::new(Sphere::new(10.0));
        let plane = SplitPlane::Z(0.0);
        let result = split_body(sphere, &plane, &AlignmentFeature::None);

        // Bottom half: Z=-5 should be inside
        let d = result.part_b.distance(Vec3::new(0.0, 0.0, -5.0));
        assert!(d < 0.0, "Z=-5 should be inside bottom half, got {}", d);

        // Bottom half: Z=+5 should be outside
        let d = result.part_b.distance(Vec3::new(0.0, 0.0, 5.0));
        assert!(d > 0.0, "Z=+5 should be outside bottom half, got {}", d);
    }

    #[test]
    fn split_multi_produces_three_parts() {
        let sphere: Arc<dyn Sdf> = Arc::new(Sphere::new(15.0));
        let planes = vec![
            (SplitPlane::Z(-4.0), AlignmentFeature::None),
            (SplitPlane::Z(4.0),  AlignmentFeature::None),
        ];
        let parts = split_body_multi(sphere, &planes);
        assert_eq!(parts.len(), 3, "two splits should produce three parts");

        // Bottom piece: inside at z=-10, outside at z=0 and z=+10
        assert!(parts[0].distance(Vec3::new(0.0, 0.0, -10.0)) < 0.0);
        assert!(parts[0].distance(Vec3::new(0.0, 0.0,   0.0)) > 0.0);

        // Middle piece: inside at z=0
        assert!(parts[1].distance(Vec3::new(0.0, 0.0,  0.0)) < 0.0);
        assert!(parts[1].distance(Vec3::new(0.0, 0.0, 10.0)) > 0.0);

        // Top piece: inside at z=+10
        assert!(parts[2].distance(Vec3::new(0.0, 0.0, 10.0)) < 0.0);
        assert!(parts[2].distance(Vec3::new(0.0, 0.0,  0.0)) > 0.0);
    }

    #[test]
    fn verify_fit_passes_for_clean_split() {
        let sphere: Arc<dyn Sdf> = Arc::new(Sphere::new(10.0));
        let plane = SplitPlane::Z(0.0);
        let result = split_body(sphere, &plane, &AlignmentFeature::None);
        let fit = verify_split_fit(&result, 12.0);
        assert!(
            fit.interference_volume_mm3 < 0.5,
            "clean split should have minimal interference, got {} mm³",
            fit.interference_volume_mm3
        );
    }
}
