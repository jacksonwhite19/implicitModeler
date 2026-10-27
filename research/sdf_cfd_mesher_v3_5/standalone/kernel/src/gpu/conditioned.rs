//! Viewport-only dense conditioned field. Not part of production SdfIr lowering.
use super::*;
use crate::sdf::conditioning::{ConditionedGeometryKernel, DenseConditionedViewportSnapshot};

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub(super) struct Tile {
    origin_spacing: [f32; 4],
    guard_min: [f32; 4],
    guard_max: [f32; 4],
    dimensions_offset: [u32; 4],
}
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub(super) struct Config {
    domain_min_block_size: [f32; 4],
    domain_max: [f32; 4],
    counts: [u32; 4],
    bands: [f32; 4],
}
pub struct ConditionedViewportIr {
    pub canonical: SdfIr,
    pub generation: u64,
    pub(super) tiles: Vec<Tile>,
    pub(super) values: Vec<f32>,
    pub(super) config: Config,
}
impl ConditionedViewportIr {
    pub fn lower(kernel: &ConditionedGeometryKernel) -> Result<Self, String> {
        let canonical = super::wing::lower_viewport_sdf_ir(kernel.canonical().as_ref())
            .ok_or("canonical child is not viewport GPU lowerable")?;
        if !super::wing::has_wing(&canonical) {
            return Err("conditioned GPU qualification is limited to spline-wing children".into());
        }
        let snapshot = kernel.conditioning().dense_viewport_snapshot()?;
        Self::from_snapshot(canonical, snapshot)
    }
    fn from_snapshot(
        canonical: SdfIr,
        s: DenseConditionedViewportSnapshot,
    ) -> Result<Self, String> {
        let mut tiles = Vec::new();
        let mut values = Vec::new();
        let adaptive = s.adaptive.len();
        for (index, tile) in s.blocks.iter().chain(&s.adaptive).enumerate() {
            let offset =
                u32::try_from(values.len()).map_err(|_| "conditioned sample address overflow")?;
            tiles.push(Tile {
                origin_spacing: [tile.origin.x, tile.origin.y, tile.origin.z, tile.spacing],
                guard_min: [
                    tile.bounds.min.x,
                    tile.bounds.min.y,
                    tile.bounds.min.z,
                    if index >= s.blocks.len() { 1. } else { 0. },
                ],
                guard_max: [tile.bounds.max.x, tile.bounds.max.y, tile.bounds.max.z, 0.],
                dimensions_offset: [
                    tile.dimensions[0],
                    tile.dimensions[1],
                    tile.dimensions[2],
                    offset,
                ],
            });
            values.extend_from_slice(&tile.samples);
        }
        Ok(Self {
            canonical,
            generation: s.generation,
            tiles,
            values,
            config: Config {
                domain_min_block_size: [
                    s.domain.min.x,
                    s.domain.min.y,
                    s.domain.min.z,
                    s.block_size,
                ],
                domain_max: [s.domain.max.x, s.domain.max.y, s.domain.max.z, 0.],
                counts: [
                    s.block_counts[0],
                    s.block_counts[1],
                    s.block_counts[2],
                    adaptive as u32,
                ],
                bands: s.bands,
            },
        })
    }
    pub fn data_bytes(&self) -> usize {
        self.values.len() * 4
            + self.tiles.len() * std::mem::size_of::<Tile>()
            + std::mem::size_of::<Config>()
    }
    fn shader(&self, shader: String) -> String {
        format!(
            "{}\n{}",
            include_str!("../../shaders/conditioned_field.wgsl"),
            shader.replacen("fn sdf_eval(p:", "fn canonical_eval(p:", 1)
        )
    }
    pub fn grid(&self, lo: Vec3, hi: Vec3, res: u32) -> Result<SdfGrid, String> {
        let p = GridParams {
            bounds_min: lo.to_array(),
            bounds_max: hi.to_array(),
            resolution: res,
            _pad: 0,
        };
        let data = super::wing::run(
            &self.canonical,
            self.shader(super::build_grid_shader_offset(&self.canonical, true)),
            bytemuck::bytes_of(&p),
            None,
            (res as usize).pow(3),
            [(res + 3) / 4; 3],
            Some(self),
        )?;
        Ok(SdfGrid {
            data,
            resolution: res,
            bounds_min: lo,
            bounds_max: hi,
        })
    }
    pub fn points(&self, points: &[Vec3]) -> Result<Vec<f32>, String> {
        if points.is_empty() {
            return Ok(vec![]);
        }
        let p = PointSampleParams {
            count: points.len() as u32,
            _pad0: 0,
            _pad1: 0,
            _pad2: 0,
        };
        let input: Vec<_> = points.iter().map(|p| [p.x, p.y, p.z, 0.]).collect();
        super::wing::run(
            &self.canonical,
            self.shader(super::build_points_shader(&self.canonical)),
            bytemuck::bytes_of(&p),
            Some(&input),
            points.len(),
            [(points.len() as u32 + 63) / 64, 1, 1],
            Some(self),
        )
    }
    pub fn viewport_grid(&self,lo:Vec3,hi:Vec3,spec:crate::render::viewport_grid::ViewportGridSpec)->Result<crate::render::viewport_grid::ViewportSdfGrid,String>{
        let params=super::ViewportGridParams::new(lo,hi,spec);
        let data=super::wing::run(&self.canonical,self.shader(super::build_viewport_grid_shader(&self.canonical)),bytemuck::bytes_of(&params),None,spec.cell_count(),spec.dimensions().map(|n|n.div_ceil(4)),Some(self))?;
        Ok(crate::render::viewport_grid::ViewportSdfGrid{data,spec,bounds_min:lo,bounds_max:hi})
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf::conditioning::SdfBounds;
    use crate::sdf::conditioning::{ConditionedGeometryKernel, ConditioningPolicy};
    use std::sync::Arc;
    #[test]
    fn nonwing_kernel_is_explicitly_unsupported() {
        let k = ConditionedGeometryKernel::new(
            Arc::new(Sphere::new(2.)),
            SdfBounds::new(Vec3::splat(-3.), Vec3::splat(3.)),
            6.,
            ConditioningPolicy {
                adaptive_enabled: false,
                ..Default::default()
            },
        );
        assert!(ConditionedViewportIr::lower(&k).is_err());
    }
    #[test]
    fn conditioned_point_subset_matches_existing_gpu_rule() {
        let w = Arc::new(crate::sdf::aerospace::wing_with_airfoil(
            "0012", 60., 30., 120., 5., 0., 0.,
        ));
        let k = ConditionedGeometryKernel::new(
            w,
            SdfBounds::new(Vec3::splat(-2.), Vec3::splat(2.)),
            4.,
            ConditioningPolicy {
                grid_spacing: 2.,
                adaptive_enabled: false,
                ..Default::default()
            },
        );
        let ir = ConditionedViewportIr::lower(&k).unwrap();
        assert!(super::super::lower_sdf_ir(&k).is_none());
        let p = [
            Vec3::ZERO,
            Vec3::splat(1.),
            Vec3::splat(-2.),
            Vec3::new(30., 0., 3.),
        ];
        let gpu = ir.points(&p).expect("GPU conditioned point evaluation");
        for (p, g) in p.iter().zip(gpu) {
            assert!((k.distance(*p) - g).abs() < 1e-4);
        }
    }
}
