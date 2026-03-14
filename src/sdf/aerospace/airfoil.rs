// NACA airfoil generation and 2D->3D SDF conversion

use glam::{Vec2, Vec3};
use lazy_static::lazy_static;
use std::any::Any;
use std::collections::HashMap;
use std::sync::{Arc, RwLock};
use crate::sdf::Sdf;
use super::section::Section2D;

/// Airfoil profile defined by 2D coordinates
#[derive(Clone)]
pub struct Airfoil {
    /// Normalized coordinates (x, y) where x ∈ [0,1], y ∈ [-0.5, 0.5]
    pub points: Vec<(f32, f32)>,
    pub chord: f32,
}

impl Airfoil {
    pub fn new(points: Vec<(f32, f32)>, chord: f32) -> Self {
        Self { points, chord }
    }

    /// Compute the distance to the lerped polyline inline — no allocation, no intermediate struct.
    /// This is the inner hot-path called by `Section2D::distance_lerped_2d`.
    pub(super) fn lerped_distance_inline(&self, other: &Airfoil, t: f32, point: Vec2) -> f32 {
        let n = self.points.len().min(other.points.len());
        if n < 2 { return point.length(); }

        let lerp_chord = self.chord + (other.chord - self.chord) * t;
        let p = Vec2::new(point.x / lerp_chord, point.y / lerp_chord);
        let s = 1.0 - t;

        // Closest-point pass
        let mut min_dist_sq = f32::MAX;
        for i in 0..n - 1 {
            let (ax0, ay0) = self.points[i];
            let (bx0, by0) = other.points[i];
            let (ax1, ay1) = self.points[i + 1];
            let (bx1, by1) = other.points[i + 1];
            let p1 = Vec2::new(ax0 * s + bx0 * t, ay0 * s + by0 * t);
            let p2 = Vec2::new(ax1 * s + bx1 * t, ay1 * s + by1 * t);
            let edge = p2 - p1;
            let elen_sq = edge.length_squared();
            if elen_sq < 1e-10 {
                min_dist_sq = min_dist_sq.min((p - p1).length_squared());
                continue;
            }
            let tp = ((p - p1).dot(edge) / elen_sq).clamp(0.0, 1.0);
            min_dist_sq = min_dist_sq.min((p - (p1 + edge * tp)).length_squared());
        }

        let dist = min_dist_sq.sqrt() * lerp_chord;

        // Winding-number pass
        let mut crossings = 0u32;
        for i in 0..n - 1 {
            let (ax0, ay0) = self.points[i];
            let (bx0, by0) = other.points[i];
            let (ax1, ay1) = self.points[i + 1];
            let (by1) = other.points[i + 1].1;
            let y0 = ay0 * s + by0 * t;
            let y1 = ay1 * s + by1 * t;
            if (y0 <= p.y && y1 > p.y) || (y1 <= p.y && y0 > p.y) {
                let tc = (p.y - y0) / (y1 - y0);
                let x0 = (self.points[i].0) * s + other.points[i].0 * t;
                let x1 = (self.points[i + 1].0) * s + other.points[i + 1].0 * t;
                if p.x < x0 + tc * (x1 - x0) {
                    crossings += 1;
                }
            }
        }

        if crossings % 2 == 1 { -dist } else { dist }
    }

    /// Blend linearly with `other` by `t`, resampling both polylines to the same point count.
    pub fn lerp(&self, other: &Airfoil, t: f32) -> Airfoil {
        let n = self.points.len().min(other.points.len());
        let a = resample_polyline(&self.points, n);
        let b = resample_polyline(&other.points, n);
        let points = a.iter().zip(b.iter())
            .map(|(&(ax, ay), &(bx, by))| (ax + (bx - ax) * t, ay + (by - ay) * t))
            .collect();
        let chord = self.chord + (other.chord - self.chord) * t;
        Airfoil::new(points, chord)
    }
}

impl Section2D for Airfoil {
    fn distance_2d(&self, point: Vec2) -> f32 {
        if self.points.is_empty() {
            return point.length();
        }

        // Normalize point to [0,1] coordinate system
        let p = Vec2::new(point.x / self.chord, point.y / self.chord);

        // Find closest point on polyline
        let mut min_dist_sq = f32::MAX;
        for i in 0..self.points.len() - 1 {
            let p1 = Vec2::new(self.points[i].0, self.points[i].1);
            let p2 = Vec2::new(self.points[i + 1].0, self.points[i + 1].1);

            let edge = p2 - p1;
            let edge_len_sq = edge.length_squared();
            if edge_len_sq < 1e-10 {
                min_dist_sq = min_dist_sq.min((p - p1).length_squared());
                continue;
            }

            let t_proj = ((p - p1).dot(edge) / edge_len_sq).clamp(0.0, 1.0);
            let dist_sq = (p - (p1 + edge * t_proj)).length_squared();
            min_dist_sq = min_dist_sq.min(dist_sq);
        }

        let dist = min_dist_sq.sqrt() * self.chord;

        // Determine sign via ray-casting (winding number)
        let mut crossings = 0;
        for i in 0..self.points.len() - 1 {
            let p1 = Vec2::new(self.points[i].0, self.points[i].1);
            let p2 = Vec2::new(self.points[i + 1].0, self.points[i + 1].1);
            if (p1.y <= p.y && p2.y > p.y) || (p1.y > p.y && p2.y <= p.y) {
                let t_cross = (p.y - p1.y) / (p2.y - p1.y);
                if p.x < p1.x + t_cross * (p2.x - p1.x) {
                    crossings += 1;
                }
            }
        }

        if crossings % 2 == 1 { -dist } else { dist }
    }

    /// Allocation-free hot-path version: lerps both polylines inline without creating
    /// an intermediate `Airfoil`. Called millions of times during ray-march / MC.
    fn distance_lerped_2d(&self, other: &dyn Section2D, t: f32, point: Vec2) -> f32 {
        if let Some(other_af) = other.as_any().downcast_ref::<Airfoil>() {
            self.lerped_distance_inline(other_af, t, point)
        } else {
            // Heterogeneous fallback: blend distances
            self.distance_2d(point) * (1.0 - t) + other.distance_2d(point) * t
        }
    }

    fn lerp_to(&self, other: &dyn Section2D, t: f32) -> Arc<dyn Section2D> {
        if let Some(other_airfoil) = other.as_any().downcast_ref::<Airfoil>() {
            Arc::new(self.lerp(other_airfoil, t))
        } else {
            Arc::new(self.clone())
        }
    }

    fn as_any(&self) -> &dyn Any { self }
}

/// Resample a polyline to exactly `n` points by arc-length parameterisation.
/// When `pts.len() == n` the original slice is returned as-is (no allocation beyond copy).
fn resample_polyline(pts: &[(f32, f32)], n: usize) -> Vec<(f32, f32)> {
    if pts.len() == n {
        return pts.to_vec();
    }
    if n == 0 || pts.is_empty() {
        return Vec::new();
    }

    // Build cumulative arc-length table
    let mut arc = Vec::with_capacity(pts.len());
    arc.push(0.0f32);
    for i in 1..pts.len() {
        let dx = pts[i].0 - pts[i - 1].0;
        let dy = pts[i].1 - pts[i - 1].1;
        arc.push(arc[i - 1] + (dx * dx + dy * dy).sqrt());
    }
    let total = *arc.last().unwrap();
    if total < 1e-10 {
        return vec![(pts[0].0, pts[0].1); n];
    }

    let denom = (n - 1).max(1) as f32;
    (0..n)
        .map(|i| {
            let s = total * i as f32 / denom;
            let j = arc.partition_point(|&a| a < s).saturating_sub(1).min(pts.len() - 2);
            let seg_len = arc[j + 1] - arc[j];
            let t = if seg_len > 1e-10 { (s - arc[j]) / seg_len } else { 0.0 };
            (
                pts[j].0 + t * (pts[j + 1].0 - pts[j].0),
                pts[j].1 + t * (pts[j + 1].1 - pts[j].1),
            )
        })
        .collect()
}

/// Generate NACA 4-digit airfoil coordinates
///
/// # Arguments
/// * `m` - Maximum camber as fraction of chord (0-0.1 typical)
/// * `p` - Location of maximum camber as fraction of chord (0-1)
/// * `t` - Maximum thickness as fraction of chord (0-0.3 typical)
/// * `n_points` - Number of points to generate (50-100 recommended)
///
/// Returns vector of (x, y) coordinates from trailing edge (1, 0) around leading edge to trailing edge
pub fn naca_4digit(m: f32, p: f32, t: f32, n_points: usize) -> Vec<(f32, f32)> {
    let mut points = Vec::with_capacity(n_points * 2);

    // Generate x coordinates using cosine spacing (better resolution at leading edge)
    let mut x_coords = Vec::with_capacity(n_points);
    for i in 0..n_points {
        let beta = std::f32::consts::PI * (i as f32) / ((n_points - 1) as f32);
        let x = 0.5 * (1.0 - beta.cos());
        x_coords.push(x);
    }

    // Calculate thickness distribution.
    // Use -0.1015 (open TE variant) so the trailing edge has finite thickness
    // (~0.252% of chord), then clamp to a floor so even tapered wingtips render cleanly.
    let thickness = |x: f32| {
        let yt = 5.0 * t * (
            0.2969 * x.sqrt()
            - 0.1260 * x
            - 0.3516 * x.powi(2)
            + 0.2843 * x.powi(3)
            - 0.1015 * x.powi(4)  // -0.1015 = open TE; small but finite thickness at x=1
        );
        // Minimum half-thickness of 0.6% chord so the TE is always above one voxel
        // at Fine/Ultra quality (0.05/0.02 unit cells) for typical chord lengths.
        yt.max(0.006)
    };

    // Calculate mean camber line and its derivative
    let camber = |x: f32| {
        if p < 0.0001 {
            0.0
        } else if x < p {
            (m / p.powi(2)) * (2.0 * p * x - x.powi(2))
        } else {
            (m / (1.0 - p).powi(2)) * ((1.0 - 2.0 * p) + 2.0 * p * x - x.powi(2))
        }
    };

    let camber_deriv = |x: f32| {
        if p < 0.0001 {
            0.0
        } else if x < p {
            (2.0 * m / p.powi(2)) * (p - x)
        } else {
            (2.0 * m / (1.0 - p).powi(2)) * (p - x)
        }
    };

    // Generate upper surface points (from leading edge to trailing edge)
    for i in (0..n_points).rev() {
        let x = x_coords[i];
        let yt = thickness(x);
        let yc = camber(x);
        let dyc_dx = camber_deriv(x);

        let theta = dyc_dx.atan();
        let xu = x - yt * theta.sin();
        let yu = yc + yt * theta.cos();

        points.push((xu, yu));
    }

    // Generate lower surface points (from trailing edge to leading edge)
    for i in 1..n_points {  // Skip first point to avoid duplication
        let x = x_coords[i];
        let yt = thickness(x);
        let yc = camber(x);
        let dyc_dx = camber_deriv(x);

        let theta = dyc_dx.atan();
        let xl = x + yt * theta.sin();
        let yl = yc - yt * theta.cos();

        points.push((xl, yl));
    }

    // Close the polyline: connect lower TE back to upper TE.
    // This is critical for correct winding-number inside/outside tests.
    if let Some(&first) = points.first() {
        points.push(first);
    }

    points
}

/// Returns true if the string is a valid NACA 4-digit designation.
pub fn is_valid_naca_4digit(designation: &str) -> bool {
    designation.len() == 4 && designation.chars().all(|c| c.is_ascii_digit())
}

/// Parse NACA 4-digit designation string
///
/// # Example
/// "2412" → (m=0.02, p=0.4, t=0.12)
fn parse_naca_4digit(designation: &str) -> Option<(f32, f32, f32)> {
    if designation.len() != 4 {
        return None;
    }

    let digits: Vec<char> = designation.chars().collect();

    let m = digits[0].to_digit(10)? as f32 / 100.0;  // Max camber in hundredths
    let p = digits[1].to_digit(10)? as f32 / 10.0;    // Position in tenths
    let t_tens = digits[2].to_digit(10)? as f32;
    let t_ones = digits[3].to_digit(10)? as f32;
    let t = (t_tens * 10.0 + t_ones) / 100.0;         // Thickness in hundredths

    Some((m, p, t))
}

// Airfoil cache for performance
lazy_static! {
    static ref AIRFOIL_CACHE: RwLock<HashMap<String, Arc<Airfoil>>> =
        RwLock::new(HashMap::new());
}

/// Get or generate a NACA 4-digit airfoil with caching
///
/// # Arguments
/// * `designation` - NACA 4-digit code (e.g., "2412", "0012")
/// * `chord` - Chord length in model units
///
/// Returns cached or newly generated airfoil
pub fn get_naca_airfoil(designation: &str, chord: f32) -> Arc<Airfoil> {
    let key = format!("{}_{:.2}", designation, chord);

    {
        let cache = AIRFOIL_CACHE.read().unwrap();
        if let Some(airfoil) = cache.get(&key) {
            return Arc::clone(airfoil);
        }
    }

    let (m, p, t) = match parse_naca_4digit(designation) {
        Some(v) => v,
        None => (0.0, 0.0, 0.12), // NACA 0012 fallback
    };

    let points = naca_4digit(m, p, t, 100);
    let airfoil = Arc::new(Airfoil::new(points, chord));

    {
        let mut cache = AIRFOIL_CACHE.write().unwrap();
        cache.insert(key, Arc::clone(&airfoil));
    }

    airfoil
}

/// Extruded airfoil - simple 3D SDF from 2D airfoil profile
pub struct ExtrudedAirfoil {
    airfoil: Arc<Airfoil>,
    half_span: f32,
}

impl ExtrudedAirfoil {
    pub fn new(airfoil: Arc<Airfoil>, half_span: f32) -> Self {
        Self { airfoil, half_span }
    }
}

impl Sdf for ExtrudedAirfoil {
    fn distance(&self, point: Vec3) -> f32 {
        let p_2d = Vec2::new(point.x, point.y);
        let d_profile = self.airfoil.distance_2d(p_2d);

        let d_span = point.z.abs() - self.half_span;

        let outside_profile = d_profile.max(0.0);
        let outside_span = d_span.max(0.0);
        let combined_outside = (outside_profile * outside_profile + outside_span * outside_span).sqrt();

        let inside_dist = d_profile.max(d_span).min(0.0);

        combined_outside + inside_dist
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::aerospace::section::Section2D;

    #[test]
    fn test_parse_naca_4digit() {
        let (m, p, t) = parse_naca_4digit("2412").unwrap();
        assert!((m - 0.02).abs() < 0.001, "Max camber should be 0.02");
        assert!((p - 0.4).abs() < 0.001, "Position should be 0.4");
        assert!((t - 0.12).abs() < 0.001, "Thickness should be 0.12");
    }

    #[test]
    fn test_naca_0012_symmetric() {
        let points = naca_4digit(0.0, 0.0, 0.12, 50);

        let first_point = points.first().unwrap();
        assert!((first_point.0 - 1.0).abs() < 0.05, "First point should be near trailing edge x=1");

        let last_point = points.last().unwrap();
        assert!((last_point.0 - 1.0).abs() < 0.05, "Last point should be near trailing edge x=1");

        // Trailing edge should have small but non-zero thickness (open TE formula)
        assert!(first_point.1.abs() < 0.05, "Trailing edge upper should be near y=0");
        assert!(last_point.1.abs() < 0.05, "Trailing edge lower should be near y=0");

        let mut leading_edge = &points[0];
        for point in &points {
            if point.0 < leading_edge.0 {
                leading_edge = point;
            }
        }
        assert!(leading_edge.0 < 0.05, "Leading edge should be near x=0");
    }

    #[test]
    fn test_naca_2412_cambered() {
        let points = naca_4digit(0.02, 0.4, 0.12, 50);

        let mid = points.len() / 2;
        let upper = points[mid / 2];
        let lower = points[mid + mid / 2];

        assert!(upper.1 > lower.1.abs(), "Cambered airfoil should be asymmetric");
    }

    #[test]
    fn test_airfoil_cache() {
        let airfoil1 = get_naca_airfoil("2412", 10.0);
        let airfoil2 = get_naca_airfoil("2412", 10.0);
        assert!(Arc::ptr_eq(&airfoil1, &airfoil2), "Cache should return same Arc");
    }

    #[test]
    fn test_extruded_airfoil_sdf() {
        let airfoil = get_naca_airfoil("0012", 10.0);
        let extruded = ExtrudedAirfoil::new(airfoil, 5.0);

        let _ = extruded.distance(Vec3::new(5.0, 0.0, 0.0));

        let dist_outside = extruded.distance(Vec3::new(50.0, 50.0, 50.0));
        assert!(dist_outside > 5.0, "Point far outside should have positive distance");

        let dist_beyond_span = extruded.distance(Vec3::new(5.0, 0.0, 20.0));
        assert!(dist_beyond_span > 5.0, "Point beyond span should be far outside");
    }

    #[test]
    fn test_airfoil_lerp_same_designation() {
        // Lerp between two NACA 0012 airfoils of different chord should give mid chord
        let a = get_naca_airfoil("0012", 10.0);
        let b = get_naca_airfoil("0012", 6.0);
        let mid = a.lerp(&b, 0.5);
        assert!((mid.chord - 8.0).abs() < 0.01, "Lerped chord should be 8");
        assert_eq!(mid.points.len(), a.points.len(), "Point count should match");
    }

    #[test]
    fn test_airfoil_lerp_to_trait() {
        let a = get_naca_airfoil("0012", 10.0);
        let b = get_naca_airfoil("2412", 8.0);
        let mid = a.lerp_to(b.as_ref(), 0.5);
        // Should produce a valid section at an intermediate point inside the airfoil
        let d = mid.distance_2d(Vec2::new(5.0, 0.0));
        assert!(d < 0.0, "Mid-chord centerline should be inside interpolated airfoil");
    }

    #[test]
    fn test_resample_polyline_same_count() {
        let pts: Vec<(f32, f32)> = (0..10).map(|i| (i as f32, 0.0)).collect();
        let resampled = resample_polyline(&pts, 10);
        assert_eq!(resampled.len(), 10);
        for (a, b) in pts.iter().zip(resampled.iter()) {
            assert!((a.0 - b.0).abs() < 1e-6);
        }
    }

    #[test]
    fn test_resample_polyline_different_count() {
        // 5 points on unit line, resample to 9
        let pts: Vec<(f32, f32)> = (0..5).map(|i| (i as f32 / 4.0, 0.0)).collect();
        let resampled = resample_polyline(&pts, 9);
        assert_eq!(resampled.len(), 9);
        // Endpoints should be preserved
        assert!((resampled[0].0 - 0.0).abs() < 1e-5);
        assert!((resampled[8].0 - 1.0).abs() < 1e-5);
    }
}
