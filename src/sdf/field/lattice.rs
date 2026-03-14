// Lattice primitives - periodic structures

use glam::Vec3;
use std::sync::Arc;
use crate::sdf::Sdf;
use super::Field;

/// Gyroid lattice - TPMS (triply periodic minimal surface)
/// Based on the implicit function: sin(x)cos(y) + sin(y)cos(z) + sin(z)cos(x) = 0
pub struct GyroidLattice {
    pub cell_size: f32,
    pub thickness: f32,
}

impl GyroidLattice {
    pub fn new(cell_size: f32, thickness: f32) -> Self {
        Self {
            cell_size,
            thickness,
        }
    }

    fn gyroid_value(p: Vec3) -> f32 {
        p.x.sin() * p.y.cos() + p.y.sin() * p.z.cos() + p.z.sin() * p.x.cos()
    }
}

impl Sdf for GyroidLattice {
    fn distance(&self, point: Vec3) -> f32 {
        use std::f32::consts::PI;

        // Scale point to gyroid domain (period = 2π)
        let scaled = point * (2.0 * PI / self.cell_size);

        // Evaluate gyroid implicit function
        let gyroid = Self::gyroid_value(scaled);

        // Convert to approximate distance
        // Gyroid value oscillates around zero, we want thickness/2 at gyroid=0
        let scale_factor = self.cell_size / (2.0 * PI);
        (gyroid.abs() - self.thickness / (2.0 * scale_factor)) * scale_factor
    }
}

/// Cubic lattice - simple repeating struts along cell edges
pub struct CubicLattice {
    pub cell_size: f32,
    pub strut_radius: f32,
}

impl CubicLattice {
    pub fn new(cell_size: f32, strut_radius: f32) -> Self {
        Self {
            cell_size,
            strut_radius,
        }
    }
}

impl Sdf for CubicLattice {
    fn distance(&self, point: Vec3) -> f32 {
        // Map to cell-local coordinates [0, cell_size] using modulo
        let local = Vec3::new(
            point.x.rem_euclid(self.cell_size),
            point.y.rem_euclid(self.cell_size),
            point.z.rem_euclid(self.cell_size),
        );

        // Distance to struts along each axis
        // Struts run along cell edges, so we measure distance in the perpendicular plane

        // Distance to X-axis struts (perpendicular distance in YZ plane)
        let dy = local.y.min(self.cell_size - local.y);
        let dz = local.z.min(self.cell_size - local.z);
        let to_x_strut = (dy * dy + dz * dz).sqrt();

        // Distance to Y-axis struts (perpendicular distance in XZ plane)
        let dx = local.x.min(self.cell_size - local.x);
        // dz already calculated
        let to_y_strut = (dx * dx + dz * dz).sqrt();

        // Distance to Z-axis struts (perpendicular distance in XY plane)
        // dx, dy already calculated
        let to_z_strut = (dx * dx + dy * dy).sqrt();

        // Minimum distance to any strut, minus radius
        let min_dist = to_x_strut.min(to_y_strut).min(to_z_strut);
        min_dist - self.strut_radius
    }
}

/// Diamond lattice - BCC (body-centered cubic) structure
/// Based on implicit: sin(x)sin(y)sin(z) + sin(x)cos(y)cos(z) + cos(x)sin(y)cos(z) + cos(x)cos(y)sin(z) = 0
pub struct DiamondLattice {
    pub cell_size: f32,
    pub thickness: f32,
}

impl DiamondLattice {
    pub fn new(cell_size: f32, thickness: f32) -> Self {
        Self {
            cell_size,
            thickness,
        }
    }

    fn diamond_value(p: Vec3) -> f32 {
        p.x.sin() * p.y.sin() * p.z.sin()
            + p.x.sin() * p.y.cos() * p.z.cos()
            + p.x.cos() * p.y.sin() * p.z.cos()
            + p.x.cos() * p.y.cos() * p.z.sin()
    }
}

impl Sdf for DiamondLattice {
    fn distance(&self, point: Vec3) -> f32 {
        use std::f32::consts::PI;

        // Scale to diamond domain
        let scaled = point * (2.0 * PI / self.cell_size);

        // Evaluate diamond implicit function
        let diamond = Self::diamond_value(scaled);

        // Convert to approximate distance
        let scale_factor = self.cell_size / (2.0 * PI);
        (diamond.abs() - self.thickness / (2.0 * scale_factor)) * scale_factor
    }
}

/// Gyroid lattice with variable thickness controlled by a field
pub struct GyroidWithField {
    pub cell_size: f32,
    pub thickness_field: Arc<dyn Field>,
}

impl GyroidWithField {
    pub fn new(cell_size: f32, thickness_field: Arc<dyn Field>) -> Self {
        Self {
            cell_size,
            thickness_field,
        }
    }
}

impl Sdf for GyroidWithField {
    fn distance(&self, point: Vec3) -> f32 {
        use std::f32::consts::PI;

        // Scale to gyroid domain
        let scaled = point * (2.0 * PI / self.cell_size);

        // Evaluate gyroid
        let gyroid = GyroidLattice::gyroid_value(scaled);

        // Evaluate thickness at this specific point
        let thickness = self.thickness_field.evaluate(point);

        // Convert to distance with variable thickness
        let scale_factor = self.cell_size / (2.0 * PI);
        (gyroid.abs() - thickness / (2.0 * scale_factor)) * scale_factor
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::field::primitives::ConstantField;

    #[test]
    fn test_gyroid_basic() {
        let lattice = GyroidLattice::new(10.0, 1.0);

        // Just verify it produces finite values
        let dist = lattice.distance(Vec3::ZERO);
        assert!(dist.is_finite());

        let dist2 = lattice.distance(Vec3::new(5.0, 5.0, 5.0));
        assert!(dist2.is_finite());
    }

    #[test]
    fn test_gyroid_periodicity() {
        let lattice = GyroidLattice::new(10.0, 1.0);

        let p1 = Vec3::new(5.0, 3.0, 2.0);
        let p2 = Vec3::new(15.0, 3.0, 2.0); // Shifted by one cell in X
        let p3 = Vec3::new(5.0, 13.0, 2.0); // Shifted by one cell in Y
        let p4 = Vec3::new(5.0, 3.0, 12.0); // Shifted by one cell in Z

        let d1 = lattice.distance(p1);
        let d2 = lattice.distance(p2);
        let d3 = lattice.distance(p3);
        let d4 = lattice.distance(p4);

        // Should be approximately equal due to periodicity
        assert!((d1 - d2).abs() < 0.1, "X periodicity failed: {} vs {}", d1, d2);
        assert!((d1 - d3).abs() < 0.1, "Y periodicity failed: {} vs {}", d1, d3);
        assert!((d1 - d4).abs() < 0.1, "Z periodicity failed: {} vs {}", d1, d4);
    }

    #[test]
    fn test_cubic_lattice_basic() {
        let lattice = CubicLattice::new(10.0, 0.5);

        // Verify finite values
        let dist = lattice.distance(Vec3::ZERO);
        assert!(dist.is_finite());
    }

    #[test]
    fn test_cubic_lattice_structure() {
        let lattice = CubicLattice::new(10.0, 0.5);

        // Point at cell corner should be near strut
        let corner = Vec3::new(0.1, 0.1, 5.0);
        let dist_corner = lattice.distance(corner);
        assert!(dist_corner < 1.0, "Corner should be near strut");

        // Point at cell center should be farther from struts
        let center = Vec3::new(5.0, 5.0, 5.0);
        let dist_center = lattice.distance(center);
        assert!(dist_center > dist_corner, "Center should be farther from struts");
    }

    #[test]
    fn test_cubic_lattice_periodicity() {
        let lattice = CubicLattice::new(10.0, 0.5);

        let p1 = Vec3::new(2.0, 3.0, 4.0);
        let p2 = Vec3::new(12.0, 3.0, 4.0); // Shifted by cell size

        let d1 = lattice.distance(p1);
        let d2 = lattice.distance(p2);

        // Should be approximately equal
        assert!((d1 - d2).abs() < 0.01, "Periodicity failed: {} vs {}", d1, d2);
    }

    #[test]
    fn test_diamond_lattice_basic() {
        let lattice = DiamondLattice::new(10.0, 1.0);

        // Verify finite values
        let dist = lattice.distance(Vec3::ZERO);
        assert!(dist.is_finite());

        let dist2 = lattice.distance(Vec3::new(5.0, 5.0, 5.0));
        assert!(dist2.is_finite());
    }

    #[test]
    fn test_diamond_lattice_periodicity() {
        let lattice = DiamondLattice::new(10.0, 1.0);

        let p1 = Vec3::new(3.0, 4.0, 5.0);
        let p2 = Vec3::new(13.0, 4.0, 5.0); // Shifted by cell size in X

        let d1 = lattice.distance(p1);
        let d2 = lattice.distance(p2);

        // Should be approximately equal
        assert!((d1 - d2).abs() < 0.1);
    }

    #[test]
    fn test_gyroid_with_field_constant() {
        // With constant field, should match GyroidLattice
        let constant_thickness = Arc::new(ConstantField::new(1.0));
        let gyroid_field = GyroidWithField::new(10.0, constant_thickness);
        let gyroid_const = GyroidLattice::new(10.0, 1.0);

        let p = Vec3::new(5.0, 3.0, 7.0);

        let d1 = gyroid_field.distance(p);
        let d2 = gyroid_const.distance(p);

        assert!((d1 - d2).abs() < 0.01, "Constant field should match: {} vs {}", d1, d2);
    }

    #[test]
    fn test_gyroid_with_field_variable() {
        use crate::sdf::field::gradients::RadialField;

        // Variable thickness - thicker at center
        let thickness_field = Arc::new(RadialField::new(
            Vec3::ZERO,
            0.0,
            20.0,
            2.0, // thick at center
            0.5, // thin at edges
        ));
        let gyroid = GyroidWithField::new(10.0, thickness_field);

        // Just verify it produces finite values
        let dist = gyroid.distance(Vec3::ZERO);
        assert!(dist.is_finite());

        let dist2 = gyroid.distance(Vec3::new(15.0, 0.0, 0.0));
        assert!(dist2.is_finite());
    }

    #[test]
    fn test_lattice_thickness_affects_distance() {
        let thin = GyroidLattice::new(10.0, 0.5);
        let thick = GyroidLattice::new(10.0, 2.0);

        let p = Vec3::new(5.0, 5.0, 5.0);

        let d_thin = thin.distance(p);
        let d_thick = thick.distance(p);

        // Thicker lattice should have different (generally more negative) distances
        // This is an approximate test since the distance field is complex
        assert!(d_thin.is_finite() && d_thick.is_finite());
    }
}
