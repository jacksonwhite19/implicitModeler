// Multi-shell composite layup SDF.
//
// A CompositeLayup wraps a parent SDF and an ordered list of ShellLayers
// (outermost → innermost).  The CompositeSdf distance() returns the outer
// mold-line surface (parent offset by total wall thickness).
//
// Key helpers:
//   layer_at(p)       → which layer index contains point p (None if outside)
//   layer_sdf(i)      → SDF of just that layer's solid shell region

use std::sync::Arc;
use glam::Vec3;
use crate::sdf::Sdf;
use crate::sdf::transforms::Offset;
use crate::sdf::booleans::{Intersect};
use crate::materials::CompositeMaterial;

// ── Complement SDF (negate distance) ─────────────────────────────────────────

#[allow(dead_code)] // Used in layer_sdf which is part of the layup analysis API
struct Complement(Arc<dyn Sdf>);
impl Sdf for Complement {
    fn distance(&self, p: Vec3) -> f32 { -self.0.distance(p) }
}

// ── ShellLayer ────────────────────────────────────────────────────────────────

pub struct ShellLayer {
    pub name:         String,
    pub material:     Arc<CompositeMaterial>,
    /// Base thickness in mm (may be overridden spatially by `thickness_field`).
    pub thickness:  f32,
    /// If Some, evaluated at each query point to get per-point thickness.
    pub thickness_field: Option<Arc<dyn crate::sdf::field::Field>>,
    /// True for foam/lattice core layers.
    pub is_core:    bool,
    /// Optional lattice infill SDF for core layers.
    pub core_infill: Option<Arc<dyn Sdf>>,
}

impl Clone for ShellLayer {
    fn clone(&self) -> Self {
        Self {
            name:            self.name.clone(),
            material:        Arc::clone(&self.material),
            thickness:       self.thickness,
            thickness_field: self.thickness_field.clone(),
            is_core:         self.is_core,
            core_infill:     self.core_infill.clone(),
        }
    }
}

impl ShellLayer {
    pub fn new(name: &str, material: Arc<CompositeMaterial>, thickness: f32) -> Self {
        Self {
            name: name.to_string(),
            material,
            thickness,
            thickness_field: None,
            is_core: false,
            core_infill: None,
        }
    }

    pub fn with_field(mut self, field: Arc<dyn crate::sdf::field::Field>) -> Self {
        self.thickness_field = Some(field);
        self
    }

    pub fn as_core(mut self, infill: Option<Arc<dyn Sdf>>) -> Self {
        self.is_core = true;
        self.core_infill = infill;
        self
    }

    /// Effective thickness at a given point.
    #[allow(dead_code)] // Part of layup analysis API
    pub fn thickness_at(&self, p: Vec3) -> f32 {
        if let Some(ref f) = self.thickness_field {
            f.evaluate(p).max(0.0)
        } else {
            self.thickness
        }
    }

    /// Areal weight in g/m² for solid laminate layers.
    #[allow(dead_code)] // Part of layup analysis API
    pub fn areal_weight_g_m2(&self) -> f32 {
        self.material.density_g_cm3 * self.thickness * 1000.0   // g/cm³ × mm × 10 = g/m²
    }
}

// ── CompositeLayup ────────────────────────────────────────────────────────────

pub struct CompositeLayup {
    /// Ordered outermost → innermost.
    pub layers: Vec<ShellLayer>,
    /// The geometry being shelled (the inner mold line).
    pub parent: Arc<dyn Sdf>,
}

impl CompositeLayup {
    pub fn new(parent: Arc<dyn Sdf>, layers: Vec<ShellLayer>) -> Self {
        Self { layers, parent }
    }

    /// Total wall thickness = sum of all layer thicknesses (at a point, using
    /// the base thickness for non-field layers).
    pub fn total_thickness(&self) -> f32 {
        self.layers.iter().map(|l| l.thickness).sum()
    }

    /// Depth of a query point above the parent surface.
    /// Equals parent.distance(p): positive in the shell/exterior region,
    /// zero on the inner mold line, negative inside the parent cavity.
    fn wall_depth(&self, p: Vec3) -> f32 {
        self.parent.distance(p)
    }

    /// Returns the index (0-based, outermost = 0) of the layer containing `p`.
    /// Returns None if the point is inside the parent cavity or outside the OML.
    pub fn layer_at(&self, p: Vec3) -> Option<usize> {
        let depth = self.wall_depth(p);
        if depth < 0.0 { return None; }   // inside the parent (cavity)

        let total = self.total_thickness();
        if depth > total { return None; }   // beyond the outer mold line

        // Walk layers from outside in.
        // depth 0..t0 = layer 0, t0..(t0+t1) = layer 1, etc.
        let mut outer_cursor = total;
        for (i, layer) in self.layers.iter().enumerate() {
            let t = layer.thickness;
            let inner_edge = outer_cursor - t;
            // Layer i occupies depth range [inner_edge, outer_cursor]
            // depth measured from inner mold line (parent surface)
            // We need to invert: outer mold line is at depth=total,
            // layer 0 is outermost = highest depth values.
            if depth > inner_edge {
                return Some(i);
            }
            outer_cursor = inner_edge;
        }
        None
    }

    /// Returns an SDF representing only the solid region of layer `index`.
    #[allow(dead_code)] // Part of layup analysis API
    pub fn layer_sdf(&self, index: usize) -> Arc<dyn Sdf> {
        // `depth` of inner edge of layer `index` above parent surface:
        //   inner_offset = sum of thicknesses of layers AFTER `index` (closer to parent)
        // `outer_offset` = inner_offset + layer[index].thickness
        let inner_offset: f32 = self.layers[index + 1..].iter().map(|l| l.thickness).sum();
        let outer_offset = inner_offset + self.layers[index].thickness;

        // Outer boundary: offset(parent, outer_offset)
        let outer_sdf: Arc<dyn Sdf> = Arc::new(Offset::new(Arc::clone(&self.parent), outer_offset));
        // Inner boundary (complement of offset(parent, inner_offset))
        let inner_sdf: Arc<dyn Sdf> = if inner_offset > 1e-6 {
            Arc::new(Complement(Arc::new(Offset::new(Arc::clone(&self.parent), inner_offset))))
        } else {
            // Inner boundary is the parent itself (negated).
            Arc::new(Complement(Arc::clone(&self.parent)))
        };

        let layer_region: Arc<dyn Sdf> = Arc::new(Intersect::new(outer_sdf, inner_sdf));

        // Apply core infill if present.
        if self.layers[index].is_core {
            if let Some(ref infill) = self.layers[index].core_infill {
                return Arc::new(Intersect::new(layer_region, Arc::clone(infill)));
            }
        }
        layer_region
    }
}

// ── CompositeSdf ──────────────────────────────────────────────────────────────

pub struct CompositeSdf {
    #[allow(dead_code)] // Accessed by layer_at/layer_sdf analysis methods
    pub layup:      Arc<CompositeLayup>,
    /// Cached outer mold line SDF = Offset(parent, total_thickness).
    outer_mold_sdf: Arc<dyn Sdf>,
}

impl CompositeSdf {
    pub fn new(layup: CompositeLayup) -> Self {
        let total = layup.total_thickness();
        let outer: Arc<dyn Sdf> = if total > 1e-6 {
            Arc::new(Offset::new(Arc::clone(&layup.parent), total))
        } else {
            Arc::clone(&layup.parent)
        };
        Self { layup: Arc::new(layup), outer_mold_sdf: outer }
    }

    /// Construct from an already-Arc'd layup (used by the scripting collector).
    pub fn from_arc(layup: Arc<CompositeLayup>) -> Self {
        let total = layup.total_thickness();
        let outer: Arc<dyn Sdf> = if total > 1e-6 {
            Arc::new(Offset::new(Arc::clone(&layup.parent), total))
        } else {
            Arc::clone(&layup.parent)
        };
        Self { layup, outer_mold_sdf: outer }
    }

    #[allow(dead_code)] // Part of layup analysis API
    pub fn layer_at(&self, p: Vec3) -> Option<usize> {
        self.layup.layer_at(p)
    }

    #[allow(dead_code)] // Part of layup analysis API
    pub fn layer_sdf(&self, index: usize) -> Arc<dyn Sdf> {
        self.layup.layer_sdf(index)
    }
}

impl Sdf for CompositeSdf {
    fn distance(&self, p: Vec3) -> f32 {
        self.outer_mold_sdf.distance(p)
    }
}

// ── Convenience constructors ──────────────────────────────────────────────────

/// Standard sandwich wing: N outer plies / foam core / N inner plies.
pub fn wing_composite(
    wing:            Arc<dyn Sdf>,
    outer_plies:     usize,
    core_thickness:  f32,
    inner_plies:     usize,
) -> Arc<dyn Sdf> {
    use crate::materials::find_preset;
    let carbon = Arc::new(find_preset("CarbonWoven_200gsm")
        .expect("CarbonWoven_200gsm preset missing"));
    let foam   = Arc::new(find_preset("Rohacell51")
        .expect("Rohacell51 preset missing"));
    let ply_t  = carbon.ply_thickness_mm().unwrap_or(0.20);

    let mut layers = Vec::new();
    for i in 0..outer_plies {
        layers.push(ShellLayer::new(
            &format!("outer_ply_{}", i + 1), Arc::clone(&carbon), ply_t));
    }
    layers.push(ShellLayer::new("core", Arc::clone(&foam), core_thickness).as_core(None));
    for i in 0..inner_plies {
        layers.push(ShellLayer::new(
            &format!("inner_ply_{}", i + 1), Arc::clone(&carbon), ply_t));
    }
    Arc::new(CompositeSdf::new(CompositeLayup::new(wing, layers)))
}

/// Standard sandwich fuselage.
pub fn fuselage_composite(
    fuselage:        Arc<dyn Sdf>,
    outer_plies:     usize,
    core_thickness:  f32,
    inner_plies:     usize,
) -> Arc<dyn Sdf> {
    // Identical construction — reuse wing helper (same sandwich pattern).
    wing_composite(fuselage, outer_plies, core_thickness, inner_plies)
}

/// Single-material printed shell.
pub fn printed_shell(
    body:       Arc<dyn Sdf>,
    thickness:  f32,
    filament:   &str,
) -> Arc<dyn Sdf> {
    use crate::materials::find_preset;
    let mat_name = match filament.to_ascii_lowercase().as_str() {
        "petg" => "PETG",
        "abs"  => "ABS",
        _      => "PLA",
    };
    let mat = Arc::new(find_preset(mat_name).unwrap_or_else(|| {
        find_preset("PLA").expect("PLA preset missing")
    }));
    let layers = vec![ShellLayer::new("shell", mat, thickness)];
    Arc::new(CompositeSdf::new(CompositeLayup::new(body, layers)))
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::primitives::Sphere;
    use crate::materials::find_preset;

    fn sphere_r10() -> Arc<dyn Sdf> {
        Arc::new(Sphere::new(10.0))
    }

    #[test]
    fn composite_outer_surface_at_correct_offset() {
        // Sphere radius 10, single shell layer 2mm → outer surface at radius 12.
        let mat     = Arc::new(find_preset("PLA").unwrap());
        let layers  = vec![ShellLayer::new("shell", mat, 2.0)];
        let layup   = CompositeLayup::new(sphere_r10(), layers);
        let csdf    = CompositeSdf::new(layup);

        // Point on the expected outer surface r=12.
        let d = csdf.distance(Vec3::new(12.0, 0.0, 0.0));
        assert!(d.abs() < 0.01, "outer surface should be at r=12, got d={}", d);

        // Point inside the parent (r<10) should be negative.
        let d_in = csdf.distance(Vec3::new(5.0, 0.0, 0.0));
        assert!(d_in < 0.0, "inside parent should be negative, got {}", d_in);
    }

    #[test]
    fn layer_at_three_layer_sandwich() {
        // Sphere r=10, layers: 1mm outer, 4mm core, 1mm inner.
        // Total = 6mm. Outer mold line at r=16.
        let carbon  = Arc::new(find_preset("CarbonWoven_200gsm").unwrap());
        let foam    = Arc::new(find_preset("Rohacell51").unwrap());
        let layers  = vec![
            ShellLayer::new("outer", Arc::clone(&carbon), 1.0),
            ShellLayer::new("core",  Arc::clone(&foam),   4.0).as_core(None),
            ShellLayer::new("inner", Arc::clone(&carbon), 1.0),
        ];
        let layup = CompositeLayup::new(sphere_r10(), layers);

        // r=10.5 → depth=0.5 → inner layer (index 2)
        let li = layup.layer_at(Vec3::new(10.5, 0.0, 0.0));
        assert_eq!(li, Some(2), "r=10.5 should be inner layer (2), got {:?}", li);

        // r=12.0 → depth=2.0 → core (index 1)
        let lc = layup.layer_at(Vec3::new(12.0, 0.0, 0.0));
        assert_eq!(lc, Some(1), "r=12.0 should be core layer (1), got {:?}", lc);

        // r=15.5 → depth=5.5 → outer layer (index 0)
        let lo = layup.layer_at(Vec3::new(15.5, 0.0, 0.0));
        assert_eq!(lo, Some(0), "r=15.5 should be outer layer (0), got {:?}", lo);

        // r=5 → inside parent → None
        assert_eq!(layup.layer_at(Vec3::new(5.0, 0.0, 0.0)), None);
        // r=17 → outside OML → None
        assert_eq!(layup.layer_at(Vec3::new(17.0, 0.0, 0.0)), None);
    }

    #[test]
    fn wing_composite_valid_sdf() {
        let wing = sphere_r10();
        let c = wing_composite(wing, 2, 6.0, 2);
        // Should be outside r=10 for surface (has shell)
        let d = c.distance(Vec3::new(10.5, 0.0, 0.0));
        assert!(d < 0.0, "inside composite shell should be negative, got {}", d);
        let d_out = c.distance(Vec3::new(30.0, 0.0, 0.0));
        assert!(d_out > 0.0, "far outside should be positive, got {}", d_out);
    }

    #[test]
    fn core_layer_with_infill() {
        use crate::sdf::lattice::ConformalGyroid;
        let sphere = sphere_r10();
        let infill: Arc<dyn Sdf> = Arc::new(ConformalGyroid::new(
            Arc::clone(&sphere), 4.0, 0.8,
        ));
        let mat  = Arc::new(find_preset("Rohacell51").unwrap());
        let core = ShellLayer::new("core", mat, 4.0).as_core(Some(infill));
        let carbon = Arc::new(find_preset("CarbonWoven_200gsm").unwrap());
        let layers = vec![
            ShellLayer::new("outer", Arc::clone(&carbon), 1.0),
            core,
        ];
        let csdf = CompositeSdf::new(CompositeLayup::new(sphere_r10(), layers));
        // Must not panic; should produce finite distances.
        let d = csdf.distance(Vec3::new(12.0, 0.0, 0.0));
        assert!(d.is_finite(), "core infill SDF should return finite distance");
    }
}
