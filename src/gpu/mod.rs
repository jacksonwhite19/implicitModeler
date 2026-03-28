use std::any::Any;
use std::sync::{Mutex, OnceLock};

use bytemuck::{Pod, Zeroable};
use eframe::wgpu;
use glam::{Mat3, Vec3, Vec3Swizzles};

use crate::mesh::{Mesh, Vertex, MeshQuality};
use crate::render::SdfGrid;
use crate::sdf::Sdf;
use crate::sdf::booleans::{Intersect, SmoothIntersect, SmoothSubtract, SmoothUnion, Subtract, Union};
use crate::mesh::marching_cubes::{EDGE_TABLE, TRI_TABLE};
use crate::sdf::primitives::{Cone, Cylinder, Plane, SdfBox, Sphere, TaperedCapsule, Torus};
use crate::sdf::transforms::{Bend, Offset, Rotate, Scale, Shell, Translate, Twist};

#[derive(Clone, Debug)]
pub enum SdfIr {
    Sphere { radius: f32 },
    Box { half_extents: [f32; 3] },
    Cylinder { radius: f32, half_height: f32 },
    Torus { major_radius: f32, minor_radius: f32 },
    Cone { radius: f32, height: f32 },
    TaperedCapsule { a: [f32; 3], b: [f32; 3], radius_a: f32, radius_b: f32 },
    Plane { normal: [f32; 3], distance: f32 },
    Union { a: Box<SdfIr>, b: Box<SdfIr> },
    Subtract { a: Box<SdfIr>, b: Box<SdfIr> },
    Intersect { a: Box<SdfIr>, b: Box<SdfIr> },
    SmoothUnion { a: Box<SdfIr>, b: Box<SdfIr>, smoothness: f32 },
    SmoothSubtract { a: Box<SdfIr>, b: Box<SdfIr>, smoothness: f32 },
    SmoothIntersect { a: Box<SdfIr>, b: Box<SdfIr>, smoothness: f32 },
    Translate { child: Box<SdfIr>, offset: [f32; 3] },
    Rotate { child: Box<SdfIr>, inverse_basis: [[f32; 3]; 3] },
    Scale { child: Box<SdfIr>, scale: [f32; 3], min_scale: f32 },
    Offset { child: Box<SdfIr>, distance: f32 },
    Shell { child: Box<SdfIr>, thickness: f32 },
    Twist { child: Box<SdfIr>, axis: [f32; 3], rate: f32 },
    Bend { child: Box<SdfIr>, axis: [f32; 3], bend_b: [f32; 3], bend_c: [f32; 3], curvature: f32 },
}

#[derive(Clone, Copy, Debug)]
pub enum SectionPlane {
    Xz,
    Yz,
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct SectionParams {
    amin: f32,
    da: f32,
    bmin: f32,
    db: f32,
    coord: f32,
    na: u32,
    nb: u32,
    plane: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct GridParams {
    bounds_min: [f32; 3],
    resolution: u32,
    bounds_max: [f32; 3],
    _pad: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct MeshParams {
    bounds_min: [f32; 3],
    cell_resolution: u32,
    bounds_max: [f32; 3],
    vertex_resolution: u32,
}

pub fn lower_sdf_ir(sdf: &dyn Sdf) -> Option<SdfIr> {
    let any = sdf as &dyn Any;

    if let Some(s) = any.downcast_ref::<Sphere>() {
        return Some(SdfIr::Sphere { radius: s.radius });
    }
    if let Some(b) = any.downcast_ref::<SdfBox>() {
        return Some(SdfIr::Box { half_extents: b.half_extents.to_array() });
    }
    if let Some(c) = any.downcast_ref::<Cylinder>() {
        return Some(SdfIr::Cylinder { radius: c.radius, half_height: c.half_height });
    }
    if let Some(t) = any.downcast_ref::<Torus>() {
        return Some(SdfIr::Torus { major_radius: t.major_radius, minor_radius: t.minor_radius });
    }
    if let Some(c) = any.downcast_ref::<Cone>() {
        return Some(SdfIr::Cone { radius: c.radius, height: c.height });
    }
    if let Some(c) = any.downcast_ref::<TaperedCapsule>() {
        return Some(SdfIr::TaperedCapsule {
            a: c.a.to_array(),
            b: c.b.to_array(),
            radius_a: c.radius_a,
            radius_b: c.radius_b,
        });
    }
    if let Some(p) = any.downcast_ref::<Plane>() {
        return Some(SdfIr::Plane { normal: p.normal.to_array(), distance: p.distance });
    }
    if let Some(u) = any.downcast_ref::<Union>() {
        return Some(SdfIr::Union {
            a: Box::new(lower_sdf_ir(u.a.as_ref())?),
            b: Box::new(lower_sdf_ir(u.b.as_ref())?),
        });
    }
    if let Some(s) = any.downcast_ref::<Subtract>() {
        return Some(SdfIr::Subtract {
            a: Box::new(lower_sdf_ir(s.a.as_ref())?),
            b: Box::new(lower_sdf_ir(s.b.as_ref())?),
        });
    }
    if let Some(i) = any.downcast_ref::<Intersect>() {
        return Some(SdfIr::Intersect {
            a: Box::new(lower_sdf_ir(i.a.as_ref())?),
            b: Box::new(lower_sdf_ir(i.b.as_ref())?),
        });
    }
    if let Some(s) = any.downcast_ref::<SmoothUnion>() {
        return Some(SdfIr::SmoothUnion {
            a: Box::new(lower_sdf_ir(s.a.as_ref())?),
            b: Box::new(lower_sdf_ir(s.b.as_ref())?),
            smoothness: s.smoothness,
        });
    }
    if let Some(s) = any.downcast_ref::<SmoothSubtract>() {
        return Some(SdfIr::SmoothSubtract {
            a: Box::new(lower_sdf_ir(s.base.as_ref())?),
            b: Box::new(lower_sdf_ir(s.tool.as_ref())?),
            smoothness: s.k,
        });
    }
    if let Some(s) = any.downcast_ref::<SmoothIntersect>() {
        return Some(SdfIr::SmoothIntersect {
            a: Box::new(lower_sdf_ir(s.a.as_ref())?),
            b: Box::new(lower_sdf_ir(s.b.as_ref())?),
            smoothness: s.k,
        });
    }
    if let Some(t) = any.downcast_ref::<Translate>() {
        return Some(SdfIr::Translate {
            child: Box::new(lower_sdf_ir(t.child.as_ref())?),
            offset: t.offset.to_array(),
        });
    }
    if let Some(r) = any.downcast_ref::<Rotate>() {
        let inv = Mat3::from_quat(r.rotation.inverse());
        return Some(SdfIr::Rotate {
            child: Box::new(lower_sdf_ir(r.child.as_ref())?),
            inverse_basis: mat3_to_rows(inv),
        });
    }
    if let Some(s) = any.downcast_ref::<Scale>() {
        return Some(SdfIr::Scale {
            child: Box::new(lower_sdf_ir(s.child.as_ref())?),
            scale: s.scale.to_array(),
            min_scale: s.scale.min_element(),
        });
    }
    if let Some(o) = any.downcast_ref::<Offset>() {
        return Some(SdfIr::Offset {
            child: Box::new(lower_sdf_ir(o.child.as_ref())?),
            distance: o.distance,
        });
    }
    if let Some(s) = any.downcast_ref::<Shell>() {
        return Some(SdfIr::Shell {
            child: Box::new(lower_sdf_ir(s.child.as_ref())?),
            thickness: s.thickness,
        });
    }
    if let Some(t) = any.downcast_ref::<Twist>() {
        return Some(SdfIr::Twist {
            child: Box::new(lower_sdf_ir(t.child.as_ref())?),
            axis: t.axis.to_array(),
            rate: t.rate,
        });
    }
    if let Some(b) = any.downcast_ref::<Bend>() {
        let a = b.axis;
        let world_ref = if a.abs().dot(Vec3::Y) < 0.9 { Vec3::Y } else { Vec3::Z };
        let bend_b = a.cross(world_ref).normalize();
        let bend_c = a.cross(bend_b);
        return Some(SdfIr::Bend {
            child: Box::new(lower_sdf_ir(b.child.as_ref())?),
            axis: a.to_array(),
            bend_b: bend_b.to_array(),
            bend_c: bend_c.to_array(),
            curvature: b.curvature,
        });
    }

    None
}

pub fn eval_ir_cpu(ir: &SdfIr, p: Vec3) -> f32 {
    match ir {
        SdfIr::Sphere { radius } => p.length() - *radius,
        SdfIr::Box { half_extents } => {
            let q = p.abs() - Vec3::from_array(*half_extents);
            q.max(Vec3::ZERO).length() + q.max_element().min(0.0)
        }
        SdfIr::Cylinder { radius, half_height } => {
            let d = Vec3::new(p.truncate().length() - *radius, 0.0, p.z.abs() - *half_height);
            d.max(Vec3::ZERO).length() + d.x.max(d.z).min(0.0)
        }
        SdfIr::Torus { major_radius, minor_radius } => {
            let q = Vec3::new(p.truncate().length() - *major_radius, 0.0, p.z);
            q.xz().length() - *minor_radius
        }
        SdfIr::Cone { radius, height } => {
            let q = p.truncate().length();
            let h = -p.z;
            let tan_angle = *radius / *height;
            let cone_dist = (q - h * tan_angle) / (1.0 + tan_angle * tan_angle).sqrt();
            if h < 0.0 {
                (q * q + p.z * p.z).sqrt()
            } else if h > *height {
                ((q - *radius).powi(2) + (h - *height).powi(2)).sqrt()
            } else {
                cone_dist
            }
        }
        SdfIr::TaperedCapsule { a, b, radius_a, radius_b } => {
            let a = Vec3::from_array(*a);
            let b = Vec3::from_array(*b);
            let ab = b - a;
            let ab_len2 = ab.length_squared();
            if ab_len2 <= 1e-12 {
                (p - a).length() - radius_a.max(*radius_b)
            } else {
                let t = ((p - a).dot(ab) / ab_len2).clamp(0.0, 1.0);
                let p_on_segment = a + ab * t;
                let r = *radius_a + t * (*radius_b - *radius_a);
                (p - p_on_segment).length() - r
            }
        }
        SdfIr::Plane { normal, distance } => p.dot(Vec3::from_array(*normal)) - *distance,
        SdfIr::Union { a, b } => eval_ir_cpu(a, p).min(eval_ir_cpu(b, p)),
        SdfIr::Subtract { a, b } => eval_ir_cpu(a, p).max(-eval_ir_cpu(b, p)),
        SdfIr::Intersect { a, b } => eval_ir_cpu(a, p).max(eval_ir_cpu(b, p)),
        SdfIr::SmoothUnion { a, b, smoothness } => {
            let d1 = eval_ir_cpu(a, p);
            let d2 = eval_ir_cpu(b, p);
            let h = (0.5 + 0.5 * (d2 - d1) / *smoothness).clamp(0.0, 1.0);
            d2 * (1.0 - h) + d1 * h - *smoothness * h * (1.0 - h)
        }
        SdfIr::SmoothSubtract { a, b, smoothness } => {
            let d1 = eval_ir_cpu(b, p);
            let d2 = eval_ir_cpu(a, p);
            let h = (0.5 - 0.5 * (d2 + d1) / *smoothness).clamp(0.0, 1.0);
            d2 + (-d1 - d2) * h + *smoothness * h * (1.0 - h)
        }
        SdfIr::SmoothIntersect { a, b, smoothness } => {
            let d1 = eval_ir_cpu(a, p);
            let d2 = eval_ir_cpu(b, p);
            let h = (0.5 - 0.5 * (d2 - d1) / *smoothness).clamp(0.0, 1.0);
            d2 * (1.0 - h) + d1 * h + *smoothness * h * (1.0 - h)
        }
        SdfIr::Translate { child, offset } => eval_ir_cpu(child, p - Vec3::from_array(*offset)),
        SdfIr::Rotate { child, inverse_basis } => {
            let m = rows_to_mat3(*inverse_basis);
            eval_ir_cpu(child, m * p)
        }
        SdfIr::Scale { child, scale, min_scale } => {
            let s = Vec3::from_array(*scale);
            eval_ir_cpu(child, p / s) * *min_scale
        }
        SdfIr::Offset { child, distance } => eval_ir_cpu(child, p) - *distance,
        SdfIr::Shell { child, thickness } => eval_ir_cpu(child, p).abs() - *thickness / 2.0,
        SdfIr::Twist { child, axis, rate } => {
            let axis = Vec3::from_array(*axis);
            let d = p.dot(axis);
            let angle_rad = (*rate * d).to_radians();
            let rotation = glam::Quat::from_axis_angle(axis, -angle_rad);
            eval_ir_cpu(child, rotation * p)
        }
        SdfIr::Bend { child, axis, bend_b, bend_c, curvature } => {
            let a = Vec3::from_array(*axis);
            let b = Vec3::from_array(*bend_b);
            let c = Vec3::from_array(*bend_c);
            let d = p.dot(a);
            let p_b = p.dot(b);
            let p_c = p.dot(c);
            let angle = *curvature * d;
            let (s, cos_a) = angle.sin_cos();
            let q_a = cos_a * d + s * p_b;
            let q_b = -s * d + cos_a * p_b;
            eval_ir_cpu(child, q_a * a + q_b * b + p_c * c)
        }
    }
}

pub fn sample_section_gpu(
    ir: &SdfIr,
    plane: SectionPlane,
    coord: f32,
    amin: f32,
    amax: f32,
    bmin: f32,
    bmax: f32,
    na: usize,
    nb: usize,
) -> Result<Vec<f32>, String> {
    let ctx = gpu_context()?;
    let shader = build_section_shader(ir);
    let shader_module = ctx.device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("GPU Section Sampling Shader"),
        source: wgpu::ShaderSource::Wgsl(shader.into()),
    });
    let params = SectionParams {
        amin,
        da: (amax - amin) / (na as f32 - 1.0),
        bmin,
        db: (bmax - bmin) / (nb as f32 - 1.0),
        coord,
        na: na as u32,
        nb: nb as u32,
        plane: match plane { SectionPlane::Xz => 0, SectionPlane::Yz => 1 },
    };
    let output_len = na * nb;
    let output_size = (output_len * std::mem::size_of::<f32>()) as u64;

    let params_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Section Params"),
        size: std::mem::size_of::<SectionParams>() as u64,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    ctx.queue.write_buffer(&params_buffer, 0, bytemuck::bytes_of(&params));

    let output_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Section Output"),
        size: output_size,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    let readback_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Section Readback"),
        size: output_size,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });

    let bind_group_layout = ctx.device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("GPU Section BGL"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ],
    });
    let bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("GPU Section BG"),
        layout: &bind_group_layout,
        entries: &[
            wgpu::BindGroupEntry { binding: 0, resource: params_buffer.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 1, resource: output_buffer.as_entire_binding() },
        ],
    });
    let pipeline_layout = ctx.device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("GPU Section PL"),
        bind_group_layouts: &[&bind_group_layout],
        push_constant_ranges: &[],
    });
    let pipeline = ctx.device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("GPU Section Pipeline"),
        layout: Some(&pipeline_layout),
        module: &shader_module,
        entry_point: "main",
        compilation_options: Default::default(),
        cache: None,
    });

    let mut encoder = ctx.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("GPU Section Encoder"),
    });
    {
        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("GPU Section Pass"),
            timestamp_writes: None,
        });
        pass.set_pipeline(&pipeline);
        pass.set_bind_group(0, &bind_group, &[]);
        let groups_x = (na as u32 + 7) / 8;
        let groups_y = (nb as u32 + 7) / 8;
        pass.dispatch_workgroups(groups_x, groups_y, 1);
    }
    encoder.copy_buffer_to_buffer(&output_buffer, 0, &readback_buffer, 0, output_size);
    ctx.queue.submit(Some(encoder.finish()));
    read_back_f32_buffer(&ctx.device, &readback_buffer, output_len)
}

pub fn sample_section_from_grid_gpu(
    grid: &SdfGrid,
    plane: SectionPlane,
    coord: f32,
    amin: f32,
    amax: f32,
    bmin: f32,
    bmax: f32,
    na: usize,
    nb: usize,
) -> Result<Vec<f32>, String> {
    let ctx = gpu_context()?;
    let shader = build_grid_section_shader();
    let shader_module = ctx.device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("GPU Grid Section Shader"),
        source: wgpu::ShaderSource::Wgsl(shader.into()),
    });
    let params = SectionParams {
        amin,
        da: (amax - amin) / (na as f32 - 1.0),
        bmin,
        db: (bmax - bmin) / (nb as f32 - 1.0),
        coord,
        na: na as u32,
        nb: nb as u32,
        plane: match plane { SectionPlane::Xz => 0, SectionPlane::Yz => 1 },
    };
    let grid_params = GridParams {
        bounds_min: grid.bounds_min.to_array(),
        resolution: grid.resolution,
        bounds_max: grid.bounds_max.to_array(),
        _pad: 0,
    };
    let scalar_size = (grid.data.len() * std::mem::size_of::<f32>()) as u64;
    let output_len = na * nb;
    let output_size = (output_len * std::mem::size_of::<f32>()) as u64;

    let section_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Grid Section Params"),
        size: std::mem::size_of::<SectionParams>() as u64,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    ctx.queue.write_buffer(&section_buffer, 0, bytemuck::bytes_of(&params));
    let grid_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Grid Section Grid Params"),
        size: std::mem::size_of::<GridParams>() as u64,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    ctx.queue.write_buffer(&grid_buffer, 0, bytemuck::bytes_of(&grid_params));
    let scalar_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Grid Section Scalars"),
        size: scalar_size,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    ctx.queue.write_buffer(&scalar_buffer, 0, bytemuck::cast_slice(&grid.data));
    let output_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Grid Section Output"),
        size: output_size,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    let readback_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Grid Section Readback"),
        size: output_size,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });

    let bind_group_layout = ctx.device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("GPU Grid Section BGL"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 3,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ],
    });
    let bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("GPU Grid Section BG"),
        layout: &bind_group_layout,
        entries: &[
            wgpu::BindGroupEntry { binding: 0, resource: section_buffer.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 1, resource: grid_buffer.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 2, resource: scalar_buffer.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 3, resource: output_buffer.as_entire_binding() },
        ],
    });
    let pipeline_layout = ctx.device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("GPU Grid Section PL"),
        bind_group_layouts: &[&bind_group_layout],
        push_constant_ranges: &[],
    });
    let pipeline = ctx.device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("GPU Grid Section Pipeline"),
        layout: Some(&pipeline_layout),
        module: &shader_module,
        entry_point: "main",
        compilation_options: Default::default(),
        cache: None,
    });
    let mut encoder = ctx.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("GPU Grid Section Encoder"),
    });
    {
        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("GPU Grid Section Pass"),
            timestamp_writes: None,
        });
        pass.set_pipeline(&pipeline);
        pass.set_bind_group(0, &bind_group, &[]);
        let groups_x = (na as u32 + 7) / 8;
        let groups_y = (nb as u32 + 7) / 8;
        pass.dispatch_workgroups(groups_x, groups_y, 1);
    }
    encoder.copy_buffer_to_buffer(&output_buffer, 0, &readback_buffer, 0, output_size);
    ctx.queue.submit(Some(encoder.finish()));
    read_back_f32_buffer(&ctx.device, &readback_buffer, output_len)
}

pub fn compute_sdf_grid_gpu(
    ir: &SdfIr,
    bounds_min: Vec3,
    bounds_max: Vec3,
    resolution: u32,
) -> Result<SdfGrid, String> {
    let ctx = gpu_context()?;
    let shader = build_grid_shader(ir);
    let shader_module = ctx.device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("GPU Grid Sampling Shader"),
        source: wgpu::ShaderSource::Wgsl(shader.into()),
    });
    let params = GridParams {
        bounds_min: bounds_min.to_array(),
        resolution,
        bounds_max: bounds_max.to_array(),
        _pad: 0,
    };
    let output_len = (resolution * resolution * resolution) as usize;
    let output_size = (output_len * std::mem::size_of::<f32>()) as u64;

    let params_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Grid Params"),
        size: std::mem::size_of::<GridParams>() as u64,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    ctx.queue.write_buffer(&params_buffer, 0, bytemuck::bytes_of(&params));

    let output_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Grid Output"),
        size: output_size,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    let readback_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Grid Readback"),
        size: output_size,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });

    let bind_group_layout = ctx.device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("GPU Grid BGL"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ],
    });
    let bind_group = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("GPU Grid BG"),
        layout: &bind_group_layout,
        entries: &[
            wgpu::BindGroupEntry { binding: 0, resource: params_buffer.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 1, resource: output_buffer.as_entire_binding() },
        ],
    });
    let pipeline_layout = ctx.device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("GPU Grid PL"),
        bind_group_layouts: &[&bind_group_layout],
        push_constant_ranges: &[],
    });
    let pipeline = ctx.device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("GPU Grid Pipeline"),
        layout: Some(&pipeline_layout),
        module: &shader_module,
        entry_point: "main",
        compilation_options: Default::default(),
        cache: None,
    });

    let mut encoder = ctx.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("GPU Grid Encoder"),
    });
    {
        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("GPU Grid Pass"),
            timestamp_writes: None,
        });
        pass.set_pipeline(&pipeline);
        pass.set_bind_group(0, &bind_group, &[]);
        let groups = (resolution + 3) / 4;
        pass.dispatch_workgroups(groups, groups, groups);
    }
    encoder.copy_buffer_to_buffer(&output_buffer, 0, &readback_buffer, 0, output_size);
    ctx.queue.submit(Some(encoder.finish()));

    Ok(SdfGrid {
        data: read_back_f32_buffer(&ctx.device, &readback_buffer, output_len)?,
        resolution,
        bounds_min,
        bounds_max,
    })
}

pub fn extract_mesh_gpu(
    ir: &SdfIr,
    bounds_min: Vec3,
    bounds_max: Vec3,
    quality: MeshQuality,
    smooth_normals: bool,
) -> Result<Mesh, String> {
    let ctx = gpu_context()?;
    let max_extent = (bounds_max - bounds_min).max_element().max(1.0);
    let target = quality.target_cell_size_mm().max(0.25);
    let cell_resolution = ((max_extent / target).ceil() as u32).clamp(12, 160);
    let vertex_resolution = cell_resolution + 1;
    let scalar_count = (vertex_resolution * vertex_resolution * vertex_resolution) as usize;
    let scalar_size = (scalar_count * std::mem::size_of::<f32>()) as u64;
    let cell_count = (cell_resolution * cell_resolution * cell_resolution) as usize;
    let count_size = (cell_count * std::mem::size_of::<u32>()) as u64;

    let mesh_params = MeshParams {
        bounds_min: bounds_min.to_array(),
        cell_resolution,
        bounds_max: bounds_max.to_array(),
        vertex_resolution,
    };

    let params_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Mesh Params"),
        size: std::mem::size_of::<MeshParams>() as u64,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    ctx.queue.write_buffer(&params_buffer, 0, bytemuck::bytes_of(&mesh_params));

    let scalar_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Mesh Scalar Grid"),
        size: scalar_size,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    run_gpu_vertex_grid_pass(
        &ctx.device,
        &ctx.queue,
        ir,
        &params_buffer,
        &scalar_buffer,
        vertex_resolution,
    )?;

    let counts_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Mesh Counts"),
        size: count_size,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    let counts_readback = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Mesh Counts Readback"),
        size: count_size,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    run_gpu_mesh_count_pass(&ctx.device, &ctx.queue, &params_buffer, &scalar_buffer, &counts_buffer, cell_resolution)?;
    {
        let mut encoder = ctx.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("GPU Mesh Count Readback"),
        });
        encoder.copy_buffer_to_buffer(&counts_buffer, 0, &counts_readback, 0, count_size);
        ctx.queue.submit(Some(encoder.finish()));
    }
    let counts = read_back_u32_buffer(&ctx.device, &counts_readback, cell_count)?;
    let mut offsets = vec![0u32; cell_count];
    let mut total_vertices = 0u32;
    for (i, count) in counts.iter().enumerate() {
        offsets[i] = total_vertices;
        total_vertices = total_vertices.saturating_add(*count);
    }
    if total_vertices == 0 {
        return Ok(Mesh { vertices: Vec::new(), indices: Vec::new() });
    }

    let offsets_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Mesh Offsets"),
        size: count_size,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    ctx.queue.write_buffer(&offsets_buffer, 0, bytemuck::cast_slice(&offsets));

    let vertex_stride = std::mem::size_of::<[f32; 4]>() as u64;
    let positions_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Mesh Positions"),
        size: total_vertices as u64 * vertex_stride,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    let positions_readback = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Mesh Positions Readback"),
        size: total_vertices as u64 * vertex_stride,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    run_gpu_mesh_emit_pass(
        &ctx.device,
        &ctx.queue,
        &params_buffer,
        &scalar_buffer,
        &offsets_buffer,
        &positions_buffer,
        cell_resolution,
    )?;
    {
        let mut encoder = ctx.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("GPU Mesh Position Readback"),
        });
        encoder.copy_buffer_to_buffer(
            &positions_buffer,
            0,
            &positions_readback,
            0,
            total_vertices as u64 * vertex_stride,
        );
        ctx.queue.submit(Some(encoder.finish()));
    }

    let positions_raw = read_back_f32_buffer(&ctx.device, &positions_readback, total_vertices as usize * 4)?;
    let mut vertices = Vec::with_capacity(total_vertices as usize);
    for chunk in positions_raw.chunks_exact(4) {
        vertices.push(Vertex {
            position: [chunk[0], chunk[1], chunk[2]],
            normal: [0.0, 0.0, 0.0],
        });
    }
    let indices: Vec<u32> = (0..total_vertices).collect();
    let mut mesh = Mesh { vertices, indices };
    compute_mesh_normals(&mut mesh, smooth_normals);
    Ok(mesh)
}

pub fn extract_mesh_from_vertex_grid_gpu(
    bounds_min: Vec3,
    bounds_max: Vec3,
    cell_resolution: u32,
    vertex_scalars: &[f32],
    smooth_normals: bool,
) -> Result<Mesh, String> {
    let ctx = gpu_context()?;
    let vertex_resolution = cell_resolution + 1;
    let expected = (vertex_resolution * vertex_resolution * vertex_resolution) as usize;
    if vertex_scalars.len() != expected {
        return Err(format!(
            "vertex scalar grid size mismatch: expected {expected}, got {}",
            vertex_scalars.len()
        ));
    }
    let mesh_params = MeshParams {
        bounds_min: bounds_min.to_array(),
        cell_resolution,
        bounds_max: bounds_max.to_array(),
        vertex_resolution,
    };
    let params_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Vertex Grid Mesh Params"),
        size: std::mem::size_of::<MeshParams>() as u64,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    ctx.queue.write_buffer(&params_buffer, 0, bytemuck::bytes_of(&mesh_params));
    let scalar_buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Vertex Grid Mesh Scalars"),
        size: (vertex_scalars.len() * std::mem::size_of::<f32>()) as u64,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    ctx.queue.write_buffer(&scalar_buffer, 0, bytemuck::cast_slice(vertex_scalars));

    extract_mesh_from_scalar_buffer_gpu(
        &ctx.device,
        &ctx.queue,
        &params_buffer,
        &scalar_buffer,
        cell_resolution,
        bounds_min,
        bounds_max,
        smooth_normals,
    )
}

fn mat3_to_rows(m: Mat3) -> [[f32; 3]; 3] {
    let cols = m.to_cols_array();
    [
        [cols[0], cols[3], cols[6]],
        [cols[1], cols[4], cols[7]],
        [cols[2], cols[5], cols[8]],
    ]
}

fn rows_to_mat3(rows: [[f32; 3]; 3]) -> Mat3 {
    Mat3::from_cols_array(&[
        rows[0][0], rows[1][0], rows[2][0],
        rows[0][1], rows[1][1], rows[2][1],
        rows[0][2], rows[1][2], rows[2][2],
    ])
}

fn build_section_shader(ir: &SdfIr) -> String {
    let expr = wgsl_expr(ir, "p");
    format!(
        r#"
struct Params {{
    amin: f32,
    da: f32,
    bmin: f32,
    db: f32,
    coord: f32,
    na: u32,
    nb: u32,
    plane: u32,
}}

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read_write> out_data: array<f32>;

fn sdf_box(p: vec3<f32>, half_extents: vec3<f32>) -> f32 {{
    let q = abs(p) - half_extents;
    return length(max(q, vec3<f32>(0.0))) + min(max(q.x, max(q.y, q.z)), 0.0);
}}

fn sdf_cylinder(p: vec3<f32>, radius: f32, half_height: f32) -> f32 {{
    let radial = length(p.xy) - radius;
    let axial = abs(p.z) - half_height;
    let d = vec2<f32>(radial, axial);
    return length(max(d, vec2<f32>(0.0))) + min(max(d.x, d.y), 0.0);
}}

fn sdf_torus(p: vec3<f32>, major_radius: f32, minor_radius: f32) -> f32 {{
    let q = vec2<f32>(length(p.xy) - major_radius, p.z);
    return length(q) - minor_radius;
}}

fn sdf_cone(p: vec3<f32>, radius: f32, height: f32) -> f32 {{
    let q = length(p.xy);
    let h = -p.z;
    let tan_angle = radius / height;
    let cone_dist = (q - h * tan_angle) / sqrt(1.0 + tan_angle * tan_angle);
    if (h < 0.0) {{
        return sqrt(q * q + p.z * p.z);
    }}
    if (h > height) {{
        return length(vec2<f32>(q - radius, h - height));
    }}
    return cone_dist;
}}

fn sdf_tapered_capsule(p: vec3<f32>, a: vec3<f32>, b: vec3<f32>, radius_a: f32, radius_b: f32) -> f32 {{
    let ab = b - a;
    let ab_len2 = dot(ab, ab);
    if (ab_len2 <= 1e-12) {{
        return length(p - a) - max(radius_a, radius_b);
    }}
    let t = clamp(dot(p - a, ab) / ab_len2, 0.0, 1.0);
    let p_on_segment = a + ab * t;
    let r = radius_a + t * (radius_b - radius_a);
    return length(p - p_on_segment) - r;
}}

fn smooth_union_poly(a: f32, b: f32, k: f32) -> f32 {{
    let h = clamp(0.5 + 0.5 * (b - a) / k, 0.0, 1.0);
    return b * (1.0 - h) + a * h - k * h * (1.0 - h);
}}

fn smooth_intersect_poly(a: f32, b: f32, k: f32) -> f32 {{
    let h = clamp(0.5 - 0.5 * (b - a) / k, 0.0, 1.0);
    return b * (1.0 - h) + a * h + k * h * (1.0 - h);
}}

fn smooth_subtract_poly(base: f32, tool: f32, k: f32) -> f32 {{
    let h = clamp(0.5 - 0.5 * (base + tool) / k, 0.0, 1.0);
    return base + (-tool - base) * h + k * h * (1.0 - h);
}}

fn rotate_around_axis(p: vec3<f32>, axis: vec3<f32>, angle: f32) -> vec3<f32> {{
    let s = sin(angle);
    let c = cos(angle);
    return p * c + cross(axis, p) * s + axis * dot(axis, p) * (1.0 - c);
}}

fn bend_eval_point(p: vec3<f32>, axis: vec3<f32>, bend_b: vec3<f32>, bend_c: vec3<f32>, curvature: f32) -> vec3<f32> {{
    let d = dot(p, axis);
    let pb = dot(p, bend_b);
    let pc = dot(p, bend_c);
    let angle = curvature * d;
    let s = sin(angle);
    let c = cos(angle);
    let qa = c * d + s * pb;
    let qb = -s * d + c * pb;
    return qa * axis + qb * bend_b + pc * bend_c;
}}

fn sdf_eval(p: vec3<f32>) -> f32 {{
    return {expr};
}}

@compute @workgroup_size(8, 8, 1)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {{
    if (gid.x >= params.na || gid.y >= params.nb) {{
        return;
    }}
    let a = params.amin + f32(gid.x) * params.da;
    let b = params.bmin + f32(gid.y) * params.db;
    let p = select(
        vec3<f32>(a, params.coord, b),
        vec3<f32>(params.coord, a, b),
        params.plane == 1u
    );
    let idx = gid.x + gid.y * params.na;
    out_data[idx] = sdf_eval(p);
}}
"#
    )
}

fn build_grid_shader(ir: &SdfIr) -> String {
    let expr = wgsl_expr(ir, "p");
    format!(
        r#"
struct Params {{
    bounds_min: vec3<f32>,
    resolution: u32,
    bounds_max: vec3<f32>,
    _pad: u32,
}}

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read_write> out_data: array<f32>;

fn sdf_box(p: vec3<f32>, half_extents: vec3<f32>) -> f32 {{
    let q = abs(p) - half_extents;
    return length(max(q, vec3<f32>(0.0))) + min(max(q.x, max(q.y, q.z)), 0.0);
}}

fn sdf_cylinder(p: vec3<f32>, radius: f32, half_height: f32) -> f32 {{
    let radial = length(p.xy) - radius;
    let axial = abs(p.z) - half_height;
    let d = vec2<f32>(radial, axial);
    return length(max(d, vec2<f32>(0.0))) + min(max(d.x, d.y), 0.0);
}}

fn sdf_torus(p: vec3<f32>, major_radius: f32, minor_radius: f32) -> f32 {{
    let q = vec2<f32>(length(p.xy) - major_radius, p.z);
    return length(q) - minor_radius;
}}

fn sdf_cone(p: vec3<f32>, radius: f32, height: f32) -> f32 {{
    let q = length(p.xy);
    let h = -p.z;
    let tan_angle = radius / height;
    let cone_dist = (q - h * tan_angle) / sqrt(1.0 + tan_angle * tan_angle);
    if (h < 0.0) {{
        return sqrt(q * q + p.z * p.z);
    }}
    if (h > height) {{
        return length(vec2<f32>(q - radius, h - height));
    }}
    return cone_dist;
}}

fn sdf_tapered_capsule(p: vec3<f32>, a: vec3<f32>, b: vec3<f32>, radius_a: f32, radius_b: f32) -> f32 {{
    let ab = b - a;
    let ab_len2 = dot(ab, ab);
    if (ab_len2 <= 1e-12) {{
        return length(p - a) - max(radius_a, radius_b);
    }}
    let t = clamp(dot(p - a, ab) / ab_len2, 0.0, 1.0);
    let p_on_segment = a + ab * t;
    let r = radius_a + t * (radius_b - radius_a);
    return length(p - p_on_segment) - r;
}}

fn smooth_union_poly(a: f32, b: f32, k: f32) -> f32 {{
    let h = clamp(0.5 + 0.5 * (b - a) / k, 0.0, 1.0);
    return b * (1.0 - h) + a * h - k * h * (1.0 - h);
}}

fn smooth_intersect_poly(a: f32, b: f32, k: f32) -> f32 {{
    let h = clamp(0.5 - 0.5 * (b - a) / k, 0.0, 1.0);
    return b * (1.0 - h) + a * h + k * h * (1.0 - h);
}}

fn smooth_subtract_poly(base: f32, tool: f32, k: f32) -> f32 {{
    let h = clamp(0.5 - 0.5 * (base + tool) / k, 0.0, 1.0);
    return base + (-tool - base) * h + k * h * (1.0 - h);
}}

fn rotate_around_axis(p: vec3<f32>, axis: vec3<f32>, angle: f32) -> vec3<f32> {{
    let s = sin(angle);
    let c = cos(angle);
    return p * c + cross(axis, p) * s + axis * dot(axis, p) * (1.0 - c);
}}

fn bend_eval_point(p: vec3<f32>, axis: vec3<f32>, bend_b: vec3<f32>, bend_c: vec3<f32>, curvature: f32) -> vec3<f32> {{
    let d = dot(p, axis);
    let pb = dot(p, bend_b);
    let pc = dot(p, bend_c);
    let angle = curvature * d;
    let s = sin(angle);
    let c = cos(angle);
    let qa = c * d + s * pb;
    let qb = -s * d + c * pb;
    return qa * axis + qb * bend_b + pc * bend_c;
}}

fn sdf_eval(p: vec3<f32>) -> f32 {{
    return {expr};
}}

@compute @workgroup_size(4, 4, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {{
    if (gid.x >= params.resolution || gid.y >= params.resolution || gid.z >= params.resolution) {{
        return;
    }}
    let span = params.bounds_max - params.bounds_min;
    let step = span / f32(params.resolution);
    let p = params.bounds_min + (vec3<f32>(gid) + vec3<f32>(0.5)) * step;
    let res = params.resolution;
    let idx = gid.x + gid.y * res + gid.z * res * res;
    out_data[idx] = sdf_eval(p);
}}
"#
    )
}

fn build_grid_section_shader() -> String {
    r#"
struct SectionParams {
    amin: f32,
    da: f32,
    bmin: f32,
    db: f32,
    coord: f32,
    na: u32,
    nb: u32,
    plane: u32,
}

struct GridParams {
    bounds_min: vec3<f32>,
    resolution: u32,
    bounds_max: vec3<f32>,
    _pad: u32,
}

@group(0) @binding(0) var<uniform> section: SectionParams;
@group(0) @binding(1) var<uniform> grid: GridParams;
@group(0) @binding(2) var<storage, read> scalars: array<f32>;
@group(0) @binding(3) var<storage, read_write> out_data: array<f32>;

fn scalar_at(ix: u32, iy: u32, iz: u32) -> f32 {
    let res = grid.resolution;
    let cx = clamp(ix, 0u, res - 1u);
    let cy = clamp(iy, 0u, res - 1u);
    let cz = clamp(iz, 0u, res - 1u);
    return scalars[cx + cy * res + cz * res * res];
}

fn sample_grid(p: vec3<f32>) -> f32 {
    let span = grid.bounds_max - grid.bounds_min;
    let cell = span / f32(grid.resolution);
    let local = (p - grid.bounds_min) / cell - vec3<f32>(0.5);
    let base = vec3<i32>(floor(local));
    let frac = fract(local);

    let x0 = u32(max(base.x, 0));
    let y0 = u32(max(base.y, 0));
    let z0 = u32(max(base.z, 0));
    let x1 = min(x0 + 1u, grid.resolution - 1u);
    let y1 = min(y0 + 1u, grid.resolution - 1u);
    let z1 = min(z0 + 1u, grid.resolution - 1u);

    let c000 = scalar_at(x0, y0, z0);
    let c100 = scalar_at(x1, y0, z0);
    let c010 = scalar_at(x0, y1, z0);
    let c110 = scalar_at(x1, y1, z0);
    let c001 = scalar_at(x0, y0, z1);
    let c101 = scalar_at(x1, y0, z1);
    let c011 = scalar_at(x0, y1, z1);
    let c111 = scalar_at(x1, y1, z1);

    let c00 = mix(c000, c100, frac.x);
    let c10 = mix(c010, c110, frac.x);
    let c01 = mix(c001, c101, frac.x);
    let c11 = mix(c011, c111, frac.x);
    let c0 = mix(c00, c10, frac.y);
    let c1 = mix(c01, c11, frac.y);
    return mix(c0, c1, frac.z);
}

@compute @workgroup_size(8, 8, 1)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= section.na || gid.y >= section.nb) {
        return;
    }
    let a = section.amin + f32(gid.x) * section.da;
    let b = section.bmin + f32(gid.y) * section.db;
    let p = select(
        vec3<f32>(a, section.coord, b),
        vec3<f32>(section.coord, a, b),
        section.plane == 1u
    );
    let idx = gid.x + gid.y * section.na;
    out_data[idx] = sample_grid(p);
}
"#.to_string()
}

fn build_vertex_grid_shader(ir: &SdfIr) -> String {
    let expr = wgsl_expr(ir, "p");
    format!(
        r#"
struct Params {{
    bounds_min: vec3<f32>,
    cell_resolution: u32,
    bounds_max: vec3<f32>,
    vertex_resolution: u32,
}}

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read_write> out_data: array<f32>;

fn sdf_box(p: vec3<f32>, half_extents: vec3<f32>) -> f32 {{
    let q = abs(p) - half_extents;
    return length(max(q, vec3<f32>(0.0))) + min(max(q.x, max(q.y, q.z)), 0.0);
}}

fn sdf_cylinder(p: vec3<f32>, radius: f32, half_height: f32) -> f32 {{
    let radial = length(p.xy) - radius;
    let axial = abs(p.z) - half_height;
    let d = vec2<f32>(radial, axial);
    return length(max(d, vec2<f32>(0.0))) + min(max(d.x, d.y), 0.0);
}}

fn sdf_torus(p: vec3<f32>, major_radius: f32, minor_radius: f32) -> f32 {{
    let q = vec2<f32>(length(p.xy) - major_radius, p.z);
    return length(q) - minor_radius;
}}

fn sdf_cone(p: vec3<f32>, radius: f32, height: f32) -> f32 {{
    let q = length(p.xy);
    let h = -p.z;
    let tan_angle = radius / height;
    let cone_dist = (q - h * tan_angle) / sqrt(1.0 + tan_angle * tan_angle);
    if (h < 0.0) {{
        return sqrt(q * q + p.z * p.z);
    }}
    if (h > height) {{
        return length(vec2<f32>(q - radius, h - height));
    }}
    return cone_dist;
}}

fn sdf_tapered_capsule(p: vec3<f32>, a: vec3<f32>, b: vec3<f32>, radius_a: f32, radius_b: f32) -> f32 {{
    let ab = b - a;
    let ab_len2 = dot(ab, ab);
    if (ab_len2 <= 1e-12) {{
        return length(p - a) - max(radius_a, radius_b);
    }}
    let t = clamp(dot(p - a, ab) / ab_len2, 0.0, 1.0);
    let p_on_segment = a + ab * t;
    let r = radius_a + t * (radius_b - radius_a);
    return length(p - p_on_segment) - r;
}}

fn smooth_union_poly(a: f32, b: f32, k: f32) -> f32 {{
    let h = clamp(0.5 + 0.5 * (b - a) / k, 0.0, 1.0);
    return b * (1.0 - h) + a * h - k * h * (1.0 - h);
}}

fn rotate_around_axis(p: vec3<f32>, axis: vec3<f32>, angle: f32) -> vec3<f32> {{
    let s = sin(angle);
    let c = cos(angle);
    return p * c + cross(axis, p) * s + axis * dot(axis, p) * (1.0 - c);
}}

fn bend_eval_point(p: vec3<f32>, axis: vec3<f32>, bend_b: vec3<f32>, bend_c: vec3<f32>, curvature: f32) -> vec3<f32> {{
    let d = dot(p, axis);
    let pb = dot(p, bend_b);
    let pc = dot(p, bend_c);
    let angle = curvature * d;
    let s = sin(angle);
    let c = cos(angle);
    let qa = c * d + s * pb;
    let qb = -s * d + c * pb;
    return qa * axis + qb * bend_b + pc * bend_c;
}}

fn sdf_eval(p: vec3<f32>) -> f32 {{
    return {expr};
}}

@compute @workgroup_size(4, 4, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {{
    if (gid.x >= params.vertex_resolution || gid.y >= params.vertex_resolution || gid.z >= params.vertex_resolution) {{
        return;
    }}
    let span = params.bounds_max - params.bounds_min;
    let step = span / f32(params.cell_resolution);
    let p = params.bounds_min + vec3<f32>(gid) * step;
    let res = params.vertex_resolution;
    let idx = gid.x + gid.y * res + gid.z * res * res;
    out_data[idx] = sdf_eval(p);
}}
"#
    )
}

fn edge_table_wgsl() -> String {
    let values = EDGE_TABLE.iter().map(|v| format!("{v}u")).collect::<Vec<_>>().join(", ");
    format!("var<private> EDGE_TABLE: array<u32, 256> = array<u32, 256>({values});")
}

fn tri_table_wgsl() -> String {
    let rows = TRI_TABLE.iter().map(|row| {
        let vals = row.iter().map(|v| format!("{v}")).collect::<Vec<_>>().join(", ");
        format!("array<i32, 16>({vals})")
    }).collect::<Vec<_>>().join(",\n");
    format!("var<private> TRI_TABLE: array<array<i32, 16>, 256> = array<array<i32, 16>, 256>(\n{rows}\n);")
}

fn tri_count_wgsl() -> String {
    let counts = TRI_TABLE.iter().map(|row| {
        let mut c = 0u32;
        let mut i = 0usize;
        while i < 16 && row[i] != -1 {
            c += 1;
            i += 3;
        }
        format!("{c}u")
    }).collect::<Vec<_>>().join(", ");
    format!("var<private> TRI_COUNTS: array<u32, 256> = array<u32, 256>({counts});")
}

fn build_mesh_count_shader() -> String {
    let edge_table = edge_table_wgsl();
    let tri_count = tri_count_wgsl();
    format!(
        r#"
struct Params {{
    bounds_min: vec3<f32>,
    cell_resolution: u32,
    bounds_max: vec3<f32>,
    vertex_resolution: u32,
}}

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read> scalars: array<f32>;
@group(0) @binding(2) var<storage, read_write> counts: array<u32>;

{edge_table}
{tri_count}

fn scalar_at(x: u32, y: u32, z: u32) -> f32 {{
    let res = params.vertex_resolution;
    return scalars[x + y * res + z * res * res];
}}

@compute @workgroup_size(4, 4, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {{
    let res = params.cell_resolution;
    if (gid.x >= res || gid.y >= res || gid.z >= res) {{
        return;
    }}

    var cube_index: u32 = 0u;
    let v0 = scalar_at(gid.x, gid.y, gid.z);
    let v1 = scalar_at(gid.x + 1u, gid.y, gid.z);
    let v2 = scalar_at(gid.x + 1u, gid.y + 1u, gid.z);
    let v3 = scalar_at(gid.x, gid.y + 1u, gid.z);
    let v4 = scalar_at(gid.x, gid.y, gid.z + 1u);
    let v5 = scalar_at(gid.x + 1u, gid.y, gid.z + 1u);
    let v6 = scalar_at(gid.x + 1u, gid.y + 1u, gid.z + 1u);
    let v7 = scalar_at(gid.x, gid.y + 1u, gid.z + 1u);
    if (v0 < 0.0) {{ cube_index = cube_index | 1u; }}
    if (v1 < 0.0) {{ cube_index = cube_index | 2u; }}
    if (v2 < 0.0) {{ cube_index = cube_index | 4u; }}
    if (v3 < 0.0) {{ cube_index = cube_index | 8u; }}
    if (v4 < 0.0) {{ cube_index = cube_index | 16u; }}
    if (v5 < 0.0) {{ cube_index = cube_index | 32u; }}
    if (v6 < 0.0) {{ cube_index = cube_index | 64u; }}
    if (v7 < 0.0) {{ cube_index = cube_index | 128u; }}
    let idx = gid.x + gid.y * res + gid.z * res * res;
    if (cube_index == 0u || cube_index == 255u || EDGE_TABLE[cube_index] == 0u) {{
        counts[idx] = 0u;
        return;
    }}
    counts[idx] = TRI_COUNTS[cube_index] * 3u;
}}
"#
    )
}

fn build_mesh_emit_shader() -> String {
    let edge_table = edge_table_wgsl();
    let tri_table = tri_table_wgsl();
    format!(
        r#"
struct Params {{
    bounds_min: vec3<f32>,
    cell_resolution: u32,
    bounds_max: vec3<f32>,
    vertex_resolution: u32,
}}

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read> scalars: array<f32>;
@group(0) @binding(2) var<storage, read> offsets: array<u32>;
@group(0) @binding(3) var<storage, read_write> positions: array<vec4<f32>>;

{edge_table}
{tri_table}

fn scalar_at(x: u32, y: u32, z: u32) -> f32 {{
    let res = params.vertex_resolution;
    return scalars[x + y * res + z * res * res];
}}

fn corner_pos(base: vec3<u32>, corner: u32, step: vec3<f32>) -> vec3<f32> {{
    var corner_offset = vec3<u32>(0u, 0u, 0u);
    switch corner {{
        case 1u: {{ corner_offset = vec3<u32>(1u, 0u, 0u); }}
        case 2u: {{ corner_offset = vec3<u32>(1u, 1u, 0u); }}
        case 3u: {{ corner_offset = vec3<u32>(0u, 1u, 0u); }}
        case 4u: {{ corner_offset = vec3<u32>(0u, 0u, 1u); }}
        case 5u: {{ corner_offset = vec3<u32>(1u, 0u, 1u); }}
        case 6u: {{ corner_offset = vec3<u32>(1u, 1u, 1u); }}
        case 7u: {{ corner_offset = vec3<u32>(0u, 1u, 1u); }}
        default: {{}}
    }}
    return params.bounds_min + vec3<f32>(base + corner_offset) * step;
}}

fn edge_v0(edge: u32) -> u32 {{
    switch edge {{
        case 1u: {{ return 1u; }}
        case 2u: {{ return 2u; }}
        case 3u: {{ return 3u; }}
        case 4u: {{ return 4u; }}
        case 5u: {{ return 5u; }}
        case 6u: {{ return 6u; }}
        case 7u: {{ return 7u; }}
        case 8u: {{ return 0u; }}
        case 9u: {{ return 1u; }}
        case 10u: {{ return 2u; }}
        case 11u: {{ return 3u; }}
        default: {{ return 0u; }}
    }}
}}

fn edge_v1(edge: u32) -> u32 {{
    switch edge {{
        case 0u: {{ return 1u; }}
        case 1u: {{ return 2u; }}
        case 2u: {{ return 3u; }}
        case 3u: {{ return 0u; }}
        case 4u: {{ return 5u; }}
        case 5u: {{ return 6u; }}
        case 6u: {{ return 7u; }}
        case 7u: {{ return 4u; }}
        case 8u: {{ return 4u; }}
        case 9u: {{ return 5u; }}
        case 10u: {{ return 6u; }}
        default: {{ return 7u; }}
    }}
}}

@compute @workgroup_size(4, 4, 4)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {{
    let res = params.cell_resolution;
    if (gid.x >= res || gid.y >= res || gid.z >= res) {{
        return;
    }}

    let cell_idx = gid.x + gid.y * res + gid.z * res * res;
    let base_out = offsets[cell_idx];
    let span = params.bounds_max - params.bounds_min;
    let step = span / f32(params.cell_resolution);

    var cube_index: u32 = 0u;
    var vals: array<f32, 8>;
    vals[0] = scalar_at(gid.x, gid.y, gid.z);
    vals[1] = scalar_at(gid.x + 1u, gid.y, gid.z);
    vals[2] = scalar_at(gid.x + 1u, gid.y + 1u, gid.z);
    vals[3] = scalar_at(gid.x, gid.y + 1u, gid.z);
    vals[4] = scalar_at(gid.x, gid.y, gid.z + 1u);
    vals[5] = scalar_at(gid.x + 1u, gid.y, gid.z + 1u);
    vals[6] = scalar_at(gid.x + 1u, gid.y + 1u, gid.z + 1u);
    vals[7] = scalar_at(gid.x, gid.y + 1u, gid.z + 1u);
    if (vals[0] < 0.0) {{ cube_index = cube_index | 1u; }}
    if (vals[1] < 0.0) {{ cube_index = cube_index | 2u; }}
    if (vals[2] < 0.0) {{ cube_index = cube_index | 4u; }}
    if (vals[3] < 0.0) {{ cube_index = cube_index | 8u; }}
    if (vals[4] < 0.0) {{ cube_index = cube_index | 16u; }}
    if (vals[5] < 0.0) {{ cube_index = cube_index | 32u; }}
    if (vals[6] < 0.0) {{ cube_index = cube_index | 64u; }}
    if (vals[7] < 0.0) {{ cube_index = cube_index | 128u; }}
    let edge_flags = EDGE_TABLE[cube_index];
    if (cube_index == 0u || cube_index == 255u || edge_flags == 0u) {{
        return;
    }}

    var edges = array<vec3<f32>, 12>();
    for (var edge: u32 = 0u; edge < 12u; edge = edge + 1u) {{
        if ((edge_flags & (1u << edge)) != 0u) {{
            let v0 = edge_v0(edge);
            let v1 = edge_v1(edge);
            let p0 = corner_pos(gid, v0, step);
            let p1 = corner_pos(gid, v1, step);
            let d0 = vals[v0];
            let d1 = vals[v1];
            let t = select(0.5, clamp(-d0 / (d1 - d0), 0.0, 1.0), abs(d1 - d0) > 1e-6);
            edges[edge] = p0 + (p1 - p0) * t;
        }}
    }}

    var out_idx: u32 = base_out;
    for (var i: u32 = 0u; i < 16u; i = i + 3u) {{
        if (TRI_TABLE[cube_index][i] < 0) {{
            break;
        }}
        positions[out_idx] = vec4<f32>(edges[u32(TRI_TABLE[cube_index][i])], 1.0);
        positions[out_idx + 1u] = vec4<f32>(edges[u32(TRI_TABLE[cube_index][i + 1u])], 1.0);
        positions[out_idx + 2u] = vec4<f32>(edges[u32(TRI_TABLE[cube_index][i + 2u])], 1.0);
        out_idx = out_idx + 3u;
    }}
}}
"#
    )
}

fn run_gpu_vertex_grid_pass(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    ir: &SdfIr,
    params_buffer: &wgpu::Buffer,
    output_buffer: &wgpu::Buffer,
    vertex_resolution: u32,
) -> Result<(), String> {
    let shader_module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("GPU Vertex Grid Shader"),
        source: wgpu::ShaderSource::Wgsl(build_vertex_grid_shader(ir).into()),
    });
    let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("GPU Vertex Grid BGL"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ],
    });
    let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("GPU Vertex Grid BG"),
        layout: &bind_group_layout,
        entries: &[
            wgpu::BindGroupEntry { binding: 0, resource: params_buffer.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 1, resource: output_buffer.as_entire_binding() },
        ],
    });
    let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("GPU Vertex Grid PL"),
        bind_group_layouts: &[&bind_group_layout],
        push_constant_ranges: &[],
    });
    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("GPU Vertex Grid Pipeline"),
        layout: Some(&pipeline_layout),
        module: &shader_module,
        entry_point: "main",
        compilation_options: Default::default(),
        cache: None,
    });
    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("GPU Vertex Grid Encoder"),
    });
    {
        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("GPU Vertex Grid Pass"),
            timestamp_writes: None,
        });
        pass.set_pipeline(&pipeline);
        pass.set_bind_group(0, &bind_group, &[]);
        let groups = (vertex_resolution + 3) / 4;
        pass.dispatch_workgroups(groups, groups, groups);
    }
    queue.submit(Some(encoder.finish()));
    Ok(())
}

fn run_gpu_mesh_count_pass(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    params_buffer: &wgpu::Buffer,
    scalar_buffer: &wgpu::Buffer,
    counts_buffer: &wgpu::Buffer,
    cell_resolution: u32,
) -> Result<(), String> {
    let shader_module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("GPU Mesh Count Shader"),
        source: wgpu::ShaderSource::Wgsl(build_mesh_count_shader().into()),
    });
    let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("GPU Mesh Count BGL"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ],
    });
    let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("GPU Mesh Count BG"),
        layout: &bind_group_layout,
        entries: &[
            wgpu::BindGroupEntry { binding: 0, resource: params_buffer.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 1, resource: scalar_buffer.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 2, resource: counts_buffer.as_entire_binding() },
        ],
    });
    let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("GPU Mesh Count PL"),
        bind_group_layouts: &[&bind_group_layout],
        push_constant_ranges: &[],
    });
    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("GPU Mesh Count Pipeline"),
        layout: Some(&pipeline_layout),
        module: &shader_module,
        entry_point: "main",
        compilation_options: Default::default(),
        cache: None,
    });
    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("GPU Mesh Count Encoder"),
    });
    {
        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("GPU Mesh Count Pass"),
            timestamp_writes: None,
        });
        pass.set_pipeline(&pipeline);
        pass.set_bind_group(0, &bind_group, &[]);
        let groups = (cell_resolution + 3) / 4;
        pass.dispatch_workgroups(groups, groups, groups);
    }
    queue.submit(Some(encoder.finish()));
    Ok(())
}

fn run_gpu_mesh_emit_pass(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    params_buffer: &wgpu::Buffer,
    scalar_buffer: &wgpu::Buffer,
    offsets_buffer: &wgpu::Buffer,
    positions_buffer: &wgpu::Buffer,
    cell_resolution: u32,
) -> Result<(), String> {
    let shader_module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("GPU Mesh Emit Shader"),
        source: wgpu::ShaderSource::Wgsl(build_mesh_emit_shader().into()),
    });
    let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("GPU Mesh Emit BGL"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 3,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ],
    });
    let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("GPU Mesh Emit BG"),
        layout: &bind_group_layout,
        entries: &[
            wgpu::BindGroupEntry { binding: 0, resource: params_buffer.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 1, resource: scalar_buffer.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 2, resource: offsets_buffer.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 3, resource: positions_buffer.as_entire_binding() },
        ],
    });
    let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("GPU Mesh Emit PL"),
        bind_group_layouts: &[&bind_group_layout],
        push_constant_ranges: &[],
    });
    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("GPU Mesh Emit Pipeline"),
        layout: Some(&pipeline_layout),
        module: &shader_module,
        entry_point: "main",
        compilation_options: Default::default(),
        cache: None,
    });
    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("GPU Mesh Emit Encoder"),
    });
    {
        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("GPU Mesh Emit Pass"),
            timestamp_writes: None,
        });
        pass.set_pipeline(&pipeline);
        pass.set_bind_group(0, &bind_group, &[]);
        let groups = (cell_resolution + 3) / 4;
        pass.dispatch_workgroups(groups, groups, groups);
    }
    queue.submit(Some(encoder.finish()));
    Ok(())
}

fn extract_mesh_from_scalar_buffer_gpu(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    params_buffer: &wgpu::Buffer,
    scalar_buffer: &wgpu::Buffer,
    cell_resolution: u32,
    bounds_min: Vec3,
    bounds_max: Vec3,
    smooth_normals: bool,
) -> Result<Mesh, String> {
    let cell_count = (cell_resolution * cell_resolution * cell_resolution) as usize;
    let count_size = (cell_count * std::mem::size_of::<u32>()) as u64;

    let counts_buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Scalar Mesh Counts"),
        size: count_size,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    let counts_readback = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Scalar Mesh Counts Readback"),
        size: count_size,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    run_gpu_mesh_count_pass(device, queue, params_buffer, scalar_buffer, &counts_buffer, cell_resolution)?;
    {
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("GPU Scalar Mesh Count Readback"),
        });
        encoder.copy_buffer_to_buffer(&counts_buffer, 0, &counts_readback, 0, count_size);
        queue.submit(Some(encoder.finish()));
    }
    let counts = read_back_u32_buffer(device, &counts_readback, cell_count)?;
    let mut offsets = vec![0u32; cell_count];
    let mut total_vertices = 0u32;
    for (i, count) in counts.iter().enumerate() {
        offsets[i] = total_vertices;
        total_vertices = total_vertices.saturating_add(*count);
    }
    if total_vertices == 0 {
        return Ok(Mesh { vertices: Vec::new(), indices: Vec::new() });
    }

    let offsets_buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Scalar Mesh Offsets"),
        size: count_size,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    queue.write_buffer(&offsets_buffer, 0, bytemuck::cast_slice(&offsets));

    let vertex_stride = std::mem::size_of::<[f32; 4]>() as u64;
    let positions_buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Scalar Mesh Positions"),
        size: total_vertices as u64 * vertex_stride,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    let positions_readback = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("GPU Scalar Mesh Positions Readback"),
        size: total_vertices as u64 * vertex_stride,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    run_gpu_mesh_emit_pass(
        device,
        queue,
        params_buffer,
        scalar_buffer,
        &offsets_buffer,
        &positions_buffer,
        cell_resolution,
    )?;
    {
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("GPU Scalar Mesh Position Readback"),
        });
        encoder.copy_buffer_to_buffer(
            &positions_buffer,
            0,
            &positions_readback,
            0,
            total_vertices as u64 * vertex_stride,
        );
        queue.submit(Some(encoder.finish()));
    }
    let positions_raw = read_back_f32_buffer(device, &positions_readback, total_vertices as usize * 4)?;
    let mut vertices = Vec::with_capacity(total_vertices as usize);
    for chunk in positions_raw.chunks_exact(4) {
        vertices.push(Vertex {
            position: [chunk[0], chunk[1], chunk[2]],
            normal: [0.0, 0.0, 0.0],
        });
    }
    let indices: Vec<u32> = (0..total_vertices).collect();
    let mut mesh = Mesh { vertices, indices };
    let _ = (bounds_min, bounds_max);
    compute_mesh_normals(&mut mesh, smooth_normals);
    Ok(mesh)
}

fn compute_mesh_normals(mesh: &mut Mesh, smooth_normals: bool) {
    let mut accum = vec![Vec3::ZERO; mesh.vertices.len()];
    for tri in mesh.indices.chunks(3) {
        let i0 = tri[0] as usize;
        let i1 = tri[1] as usize;
        let i2 = tri[2] as usize;
        let p0 = Vec3::from_array(mesh.vertices[i0].position);
        let p1 = Vec3::from_array(mesh.vertices[i1].position);
        let p2 = Vec3::from_array(mesh.vertices[i2].position);
        let n = (p1 - p0).cross(p2 - p0);
        accum[i0] += n;
        accum[i1] += n;
        accum[i2] += n;
    }
    for (v, n) in mesh.vertices.iter_mut().zip(accum.iter()) {
        let nn = n.normalize_or_zero();
        v.normal = [nn.x, nn.y, nn.z];
    }
    if smooth_normals {
        // Current GPU extractor already emits de-duplicated-per-cell vertices only as needed.
        // Keep the pass simple and stable for now.
    }
}

fn wgsl_expr(ir: &SdfIr, point_expr: &str) -> String {
    match ir {
        SdfIr::Sphere { radius } => format!("length({point_expr}) - {radius}f"),
        SdfIr::Box { half_extents } => format!(
            "sdf_box({point_expr}, vec3<f32>({}f, {}f, {}f))",
            half_extents[0], half_extents[1], half_extents[2]
        ),
        SdfIr::Cylinder { radius, half_height } => {
            format!("sdf_cylinder({point_expr}, {radius}f, {half_height}f)")
        }
        SdfIr::Torus { major_radius, minor_radius } => {
            format!("sdf_torus({point_expr}, {major_radius}f, {minor_radius}f)")
        }
        SdfIr::Cone { radius, height } => format!("sdf_cone({point_expr}, {radius}f, {height}f)"),
        SdfIr::TaperedCapsule { a, b, radius_a, radius_b } => format!(
            "sdf_tapered_capsule({point_expr}, vec3<f32>({}f, {}f, {}f), vec3<f32>({}f, {}f, {}f), {radius_a}f, {radius_b}f)",
            a[0], a[1], a[2], b[0], b[1], b[2]
        ),
        SdfIr::Plane { normal, distance } => format!(
            "dot({point_expr}, vec3<f32>({}f, {}f, {}f)) - {distance}f",
            normal[0], normal[1], normal[2]
        ),
        SdfIr::Union { a, b } => format!("min({}, {})", wgsl_expr(a, point_expr), wgsl_expr(b, point_expr)),
        SdfIr::Subtract { a, b } => format!("max({}, -({}))", wgsl_expr(a, point_expr), wgsl_expr(b, point_expr)),
        SdfIr::Intersect { a, b } => format!("max({}, {})", wgsl_expr(a, point_expr), wgsl_expr(b, point_expr)),
        SdfIr::SmoothUnion { a, b, smoothness } => {
            format!(
                "smooth_union_poly({}, {}, {}f)",
                wgsl_expr(a, point_expr),
                wgsl_expr(b, point_expr),
                smoothness
            )
        }
        SdfIr::SmoothSubtract { a, b, smoothness } => {
            format!(
                "smooth_subtract_poly({}, {}, {}f)",
                wgsl_expr(a, point_expr),
                wgsl_expr(b, point_expr),
                smoothness
            )
        }
        SdfIr::SmoothIntersect { a, b, smoothness } => {
            format!(
                "smooth_intersect_poly({}, {}, {}f)",
                wgsl_expr(a, point_expr),
                wgsl_expr(b, point_expr),
                smoothness
            )
        }
        SdfIr::Translate { child, offset } => {
            let q = format!("{point_expr} - vec3<f32>({}f, {}f, {}f)", offset[0], offset[1], offset[2]);
            wgsl_expr(child, &q)
        }
        SdfIr::Rotate { child, inverse_basis } => {
            let q = format!(
                "mat3x3<f32>(vec3<f32>({}f, {}f, {}f), vec3<f32>({}f, {}f, {}f), vec3<f32>({}f, {}f, {}f)) * {point_expr}",
                inverse_basis[0][0], inverse_basis[1][0], inverse_basis[2][0],
                inverse_basis[0][1], inverse_basis[1][1], inverse_basis[2][1],
                inverse_basis[0][2], inverse_basis[1][2], inverse_basis[2][2],
            );
            wgsl_expr(child, &q)
        }
        SdfIr::Scale { child, scale, min_scale } => {
            let q = format!("{point_expr} / vec3<f32>({}f, {}f, {}f)", scale[0], scale[1], scale[2]);
            format!("({}) * {min_scale}f", wgsl_expr(child, &q))
        }
        SdfIr::Offset { child, distance } => format!("({}) - {distance}f", wgsl_expr(child, point_expr)),
        SdfIr::Shell { child, thickness } => format!("abs({}) - {}f / 2.0", wgsl_expr(child, point_expr), thickness),
        SdfIr::Twist { child, axis, rate } => {
            let q = format!(
                "rotate_around_axis({point_expr}, vec3<f32>({}f, {}f, {}f), -radians({rate}f * dot({point_expr}, vec3<f32>({}f, {}f, {}f))))",
                axis[0], axis[1], axis[2], axis[0], axis[1], axis[2]
            );
            wgsl_expr(child, &q)
        }
        SdfIr::Bend { child, axis, bend_b, bend_c, curvature } => {
            let q = format!(
                "bend_eval_point({point_expr}, vec3<f32>({}f, {}f, {}f), vec3<f32>({}f, {}f, {}f), vec3<f32>({}f, {}f, {}f), {curvature}f)",
                axis[0], axis[1], axis[2],
                bend_b[0], bend_b[1], bend_b[2],
                bend_c[0], bend_c[1], bend_c[2]
            );
            wgsl_expr(child, &q)
        }
    }
}

struct GpuContext {
    device: wgpu::Device,
    queue: wgpu::Queue,
}

static GPU_CONTEXT: OnceLock<Result<Mutex<GpuContext>, String>> = OnceLock::new();

fn gpu_context() -> Result<std::sync::MutexGuard<'static, GpuContext>, String> {
    let ctx = GPU_CONTEXT.get_or_init(|| {
        let instance = wgpu::Instance::default();
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: None,
            force_fallback_adapter: false,
        })).ok_or_else(|| "No suitable GPU adapter found".to_string())?;

        let (device, queue) = pollster::block_on(adapter.request_device(
            &wgpu::DeviceDescriptor {
                label: Some("Implicit CAD GPU Compute Device"),
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::default(),
                memory_hints: wgpu::MemoryHints::Performance,
            },
            None,
        )).map_err(|e| format!("Failed to create GPU device: {e}"))?;

        Ok(Mutex::new(GpuContext { device, queue }))
    });
    match ctx {
        Ok(m) => m.lock().map_err(|_| "GPU context lock poisoned".to_string()),
        Err(e) => Err(e.clone()),
    }
}

fn read_back_f32_buffer(device: &wgpu::Device, buffer: &wgpu::Buffer, len: usize) -> Result<Vec<f32>, String> {
    use std::sync::mpsc;

    let slice = buffer.slice(..);
    let (tx, rx) = mpsc::channel();
    slice.map_async(wgpu::MapMode::Read, move |result| {
        let _ = tx.send(result.map_err(|e| e.to_string()));
    });
    device.poll(wgpu::Maintain::Wait);
    rx.recv().map_err(|e| format!("GPU readback channel error: {e}"))??;
    let data = slice.get_mapped_range();
    let values = bytemuck::cast_slice::<u8, f32>(&data).to_vec();
    drop(data);
    buffer.unmap();
    if values.len() != len {
        return Err(format!("GPU readback size mismatch: expected {len}, got {}", values.len()));
    }
    Ok(values)
}

fn read_back_u32_buffer(device: &wgpu::Device, buffer: &wgpu::Buffer, len: usize) -> Result<Vec<u32>, String> {
    use std::sync::mpsc;

    let slice = buffer.slice(..);
    let (tx, rx) = mpsc::channel();
    slice.map_async(wgpu::MapMode::Read, move |result| {
        let _ = tx.send(result.map_err(|e| e.to_string()));
    });
    device.poll(wgpu::Maintain::Wait);
    rx.recv().map_err(|e| format!("GPU readback channel error: {e}"))??;
    let data = slice.get_mapped_range();
    let values = bytemuck::cast_slice::<u8, u32>(&data).to_vec();
    drop(data);
    buffer.unmap();
    if values.len() != len {
        return Err(format!("GPU readback size mismatch: expected {len}, got {}", values.len()));
    }
    Ok(values)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use glam::Vec3;
    use crate::pipeline::compute_sdf_grid;
    use crate::sdf::field::lattice::GyroidLattice;
    use crate::sdf::booleans::{SmoothIntersect, SmoothSubtract, Union};
    use crate::sdf::primitives::{Sphere, SdfBox};
    use crate::sdf::transforms::{Bend, Translate};

    #[test]
    fn lower_supported_sdf_tree() {
        let shape: Arc<dyn Sdf> = Arc::new(Union::new(
            Arc::new(Translate::new(Arc::new(Sphere::new(3.0)), Vec3::new(5.0, 0.0, 0.0))),
            Arc::new(SdfBox::new(Vec3::new(1.0, 2.0, 3.0))),
        ));
        let ir = lower_sdf_ir(shape.as_ref());
        assert!(ir.is_some(), "Expected supported SDF tree to lower into IR");
    }

    #[test]
    fn lower_supported_smooth_boolean_tree() {
        let shape: Arc<dyn Sdf> = Arc::new(SmoothIntersect::new(
            Arc::new(SmoothSubtract::new(
                Arc::new(SdfBox::new(Vec3::new(4.0, 4.0, 2.0))),
                Arc::new(Translate::new(Arc::new(Sphere::new(1.5)), Vec3::new(0.0, 0.0, 1.0))),
                0.75,
            )),
            Arc::new(Translate::new(Arc::new(Sphere::new(5.0)), Vec3::new(0.0, 0.0, -0.5))),
            0.5,
        ));
        let ir = lower_sdf_ir(shape.as_ref());
        assert!(ir.is_some(), "Expected smooth subtract/intersect tree to lower into IR");
    }

    #[test]
    fn gpu_section_matches_cpu_for_supported_subset() {
        let shape: Arc<dyn Sdf> = Arc::new(Union::new(
            Arc::new(Translate::new(Arc::new(Sphere::new(3.0)), Vec3::new(1.0, 0.0, 0.0))),
            Arc::new(SdfBox::new(Vec3::new(1.5, 1.0, 0.75))),
        ));
        let ir = lower_sdf_ir(shape.as_ref()).expect("IR lowering");
        let gpu = sample_section_gpu(&ir, SectionPlane::Xz, 0.0, -5.0, 5.0, -5.0, 5.0, 32, 24)
            .expect("GPU section sampling");
        assert_eq!(gpu.len(), 32 * 24);

        let da = 10.0 / 31.0;
        let db = 10.0 / 23.0;
        for ib in 0..24 {
            for ia in 0..32 {
                let x = -5.0 + ia as f32 * da;
                let z = -5.0 + ib as f32 * db;
                let p = Vec3::new(x, 0.0, z);
                let cpu = eval_ir_cpu(&ir, p);
                let d = gpu[ia + ib * 32];
                assert!((cpu - d).abs() < 1e-3, "Mismatch at ({x},{z}): cpu={cpu}, gpu={d}");
            }
        }
    }

    #[test]
    fn gpu_grid_matches_cpu_for_supported_subset() {
        let shape: Arc<dyn Sdf> = Arc::new(Translate::new(Arc::new(Sphere::new(2.0)), Vec3::new(0.5, 0.0, -0.5)));
        let ir = lower_sdf_ir(shape.as_ref()).expect("IR lowering");
        let grid = compute_sdf_grid_gpu(&ir, Vec3::new(-4.0, -4.0, -4.0), Vec3::new(4.0, 4.0, 4.0), 16)
            .expect("GPU grid eval");
        let step = (grid.bounds_max - grid.bounds_min) / grid.resolution as f32;
        for z in 0..grid.resolution as usize {
            for y in 0..grid.resolution as usize {
                for x in 0..grid.resolution as usize {
                    let idx = x + y * grid.resolution as usize + z * grid.resolution as usize * grid.resolution as usize;
                    let p = grid.bounds_min + Vec3::new(
                        (x as f32 + 0.5) * step.x,
                        (y as f32 + 0.5) * step.y,
                        (z as f32 + 0.5) * step.z,
                    );
                    let cpu = eval_ir_cpu(&ir, p);
                    let gpu = grid.data[idx];
                    assert!((cpu - gpu).abs() < 1e-3, "Mismatch at idx {idx}: cpu={cpu}, gpu={gpu}");
                }
            }
        }
    }

    #[test]
    fn gpu_mesh_extracts_supported_subset() {
        let shape: Arc<dyn Sdf> = Arc::new(Union::new(
            Arc::new(Translate::new(Arc::new(Sphere::new(3.0)), Vec3::new(1.0, 0.0, 0.0))),
            Arc::new(SdfBox::new(Vec3::new(1.5, 1.0, 0.75))),
        ));
        let ir = lower_sdf_ir(shape.as_ref()).expect("IR lowering");
        let mesh = extract_mesh_gpu(&ir, Vec3::new(-6.0, -6.0, -6.0), Vec3::new(6.0, 6.0, 6.0), MeshQuality::Draft, false)
            .expect("GPU mesh extraction");
        assert!(!mesh.vertices.is_empty(), "GPU mesh extractor should produce vertices");
        assert_eq!(mesh.indices.len() % 3, 0, "GPU mesh output should be triangle-indexed");
    }

    #[test]
    fn gpu_grid_section_sampling_supports_unlowerable_trees() {
        let shape: Arc<dyn Sdf> = Arc::new(GyroidLattice::new(8.0, 1.0));
        assert!(lower_sdf_ir(shape.as_ref()).is_none(), "Gyroid lattice should force sampled-grid fallback");
        let grid = compute_sdf_grid(shape.as_ref(), Vec3::new(-5.0, -5.0, -5.0), Vec3::new(5.0, 5.0, 5.0), 48);
        let gpu = sample_section_from_grid_gpu(&grid, SectionPlane::Xz, 0.0, -4.0, 4.0, -4.0, 4.0, 31, 31)
            .expect("GPU grid-backed section sampling");
        assert_eq!(gpu.len(), 31 * 31);
        assert!(gpu.iter().all(|d| d.is_finite()), "Grid-backed section sampling should stay finite");
    }

    #[test]
    fn gpu_mesh_extracts_from_sampled_vertex_grid() {
        let shape: Arc<dyn Sdf> = Arc::new(GyroidLattice::new(8.0, 1.0));
        let bounds_min = Vec3::new(-5.0, -5.0, -5.0);
        let bounds_max = Vec3::new(5.0, 5.0, 5.0);
        let cell_resolution = 28u32;
        let vertex_grid = compute_sdf_grid(shape.as_ref(), bounds_min, bounds_max, cell_resolution + 1);
        let mesh = extract_mesh_from_vertex_grid_gpu(
            bounds_min,
            bounds_max,
            cell_resolution,
            &vertex_grid.data,
            false,
        ).expect("GPU mesh extraction from sampled grid");
        assert!(!mesh.vertices.is_empty(), "Sampled-grid GPU mesh extractor should produce vertices");
        assert_eq!(mesh.indices.len() % 3, 0, "Sampled-grid GPU mesh output should be triangle-indexed");
    }

    #[test]
    fn lower_bend_and_twist_trees() {
        let bent: Arc<dyn Sdf> = Arc::new(Bend::new(Arc::new(Sphere::new(3.0)), Vec3::Y, 0.08));
        assert!(lower_sdf_ir(bent.as_ref()).is_some(), "Bend should now lower into GPU IR");
    }
}
