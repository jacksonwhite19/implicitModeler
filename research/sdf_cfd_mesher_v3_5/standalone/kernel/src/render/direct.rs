//! Bounded, opt-in viewport derivative. The canonical Sdf remains authoritative.
use super::{Camera, SectionUniforms, telemetry};
use crate::gpu::SdfIr;
use crate::sdf::Sdf;
use eframe::wgpu;
use glam::{Mat3, Vec3};
use std::sync::Arc;

#[derive(Debug, PartialEq)]
pub enum DirectCapability {
    DirectSafe,
    FallbackRequired(String),
}

pub fn capability(ir: &SdfIr) -> DirectCapability {
    fn visit(ir: &SdfIr, depth: usize, count: &mut usize) -> Result<(), String> {
        *count += 1;
        if depth > 48 || *count > 256 {
            return Err("graph exceeds 48 levels / 256 nodes".into());
        }
        let invalid = || Err("invalid or nonfinite primitive/transform parameters".into());
        match ir {
            SdfIr::Sphere { radius } => {
                if radius.is_finite() && *radius > 0.0 {
                    Ok(())
                } else {
                    invalid()
                }
            }
            SdfIr::Box { half_extents } => {
                if half_extents.iter().all(|v| v.is_finite() && *v > 0.0) {
                    Ok(())
                } else {
                    invalid()
                }
            }
            SdfIr::Union { a, b } | SdfIr::Intersect { a, b } | SdfIr::Subtract { a, b } => {
                visit(a, depth + 1, count)?;
                visit(b, depth + 1, count)
            }
            SdfIr::Translate { child, offset } => {
                if !offset.iter().all(|x| x.is_finite()) {
                    return invalid();
                }
                visit(child, depth + 1, count)
            }
            SdfIr::Rotate {
                child,
                inverse_basis,
            } => {
                let m = Mat3::from_cols_array_2d(inverse_basis).transpose();
                let err = (m.transpose() * m - Mat3::IDENTITY)
                    .to_cols_array()
                    .iter()
                    .map(|x| x.abs())
                    .fold(0.0_f32, f32::max);
                if !m.is_finite() || err > 1e-5 || (m.determinant() - 1.0).abs() > 1e-5 {
                    return invalid();
                }
                visit(child, depth + 1, count)
            }
            SdfIr::Scale {
                child,
                scale,
                min_scale,
            } => {
                if !scale[0].is_finite()
                    || scale[0] <= 0.0
                    || scale[0] != scale[1]
                    || scale[0] != scale[2]
                    || *min_scale != scale[0]
                {
                    return Err("only positive uniform scale is qualified".into());
                }
                visit(child, depth + 1, count)
            }
            _ => Err("operation lowers to GPU but is not qualified for direct tracing".into()),
        }
    }
    match visit(ir, 0, &mut 0) {
        Ok(()) => DirectCapability::DirectSafe,
        Err(e) => DirectCapability::FallbackRequired(e),
    }
}

/// One function per node prevents transform-expression precedence errors and
/// exponential point-expression expansion. Leaves reuse the existing WGSL emitter.
pub fn field_source(ir: &SdfIr) -> Result<String, String> {
    if let DirectCapability::FallbackRequired(e) = capability(ir) {
        return Err(e);
    }
    fn emit(ir: &SdfIr, out: &mut String, next: &mut usize) -> usize {
        let id = *next;
        *next += 1;
        let v = |a: &[f32; 3]| format!("vec3<f32>({:?}, {:?}, {:?})", a[0], a[1], a[2]);
        let expr = match ir {
            SdfIr::Sphere { .. } | SdfIr::Box { .. } => crate::gpu::wgsl_expr(ir, "p"),
            SdfIr::Union { a, b } | SdfIr::Intersect { a, b } | SdfIr::Subtract { a, b } => {
                let a = emit(a, out, next);
                let b = emit(b, out, next);
                match ir {
                    SdfIr::Union { .. } => format!("min(node_{a}(p),node_{b}(p))"),
                    SdfIr::Intersect { .. } => format!("max(node_{a}(p),node_{b}(p))"),
                    _ => format!("max(node_{a}(p),-node_{b}(p))"),
                }
            }
            SdfIr::Translate { child, offset } => {
                let c = emit(child, out, next);
                format!("node_{c}(p - {})", v(offset))
            }
            SdfIr::Rotate {
                child,
                inverse_basis: m,
            } => {
                let c = emit(child, out, next);
                format!(
                    "node_{c}(mat3x3<f32>({},{},{}) * p)",
                    v(&[m[0][0], m[1][0], m[2][0]]),
                    v(&[m[0][1], m[1][1], m[2][1]]),
                    v(&[m[0][2], m[1][2], m[2][2]])
                )
            }
            SdfIr::Scale { child, scale, .. } => {
                let c = emit(child, out, next);
                format!("node_{c}(p / {:?}) * {:?}", scale[0], scale[0])
            }
            _ => unreachable!("capability checked"),
        };
        out.push_str(&format!(
            "fn node_{id}(p:vec3<f32>)->f32 {{ return {expr}; }}\n"
        ));
        id
    }
    let mut out = String::from(
        "fn sdf_box(p:vec3<f32>,h:vec3<f32>)->f32 { let q=abs(p)-h; return length(max(q,vec3<f32>(0.0)))+min(max(q.x,max(q.y,q.z)),0.0); }\n",
    );
    let root = emit(ir, &mut out, &mut 0);
    out.push_str(&format!(
        "fn sdf_eval(p:vec3<f32>)->f32 {{ return node_{root}(p); }}\n"
    ));
    if out.len() > 128 * 1024 {
        return Err("generated field exceeds 128 KiB".into());
    }
    Ok(out)
}

pub struct DirectScene {
    pub field: String,
    pub bounds_min: Vec3,
    pub bounds_max: Vec3,
    pub canonical: Arc<dyn Sdf>,
    pub failure: std::sync::Mutex<Option<String>>,
}
impl DirectScene {
    pub fn from_sdf(sdf: Arc<dyn Sdf>, min: Vec3, max: Vec3) -> Result<Self, String> {
        let _timer = telemetry::Span::new("direct_lower_and_generate");
        let ir = crate::gpu::lower_sdf_ir(sdf.as_ref())
            .ok_or("custom/domain-specific SDF cannot lower")?;
        if !min.is_finite() || !max.is_finite() || (max - min).min_element() <= 0.0 {
            return Err("invalid bounds".into());
        }
        Ok(Self {
            field: field_source(&ir)?,
            bounds_min: min,
            bounds_max: max,
            canonical: sdf,
            failure: std::sync::Mutex::new(None),
        })
    }
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct DirectUniforms {
    eye: [f32; 4],
    lo: [f32; 4],
    hi: [f32; 4],
    right: [f32; 4],
    up: [f32; 4],
    forward: [f32; 4],
    viewport: [f32; 4],
    section_a: [f32; 4],
    section_b: [f32; 4],
}
impl DirectUniforms {
    pub fn new(
        camera: &Camera,
        min: Vec3,
        max: Vec3,
        offset: [f32; 2],
        size: [f32; 2],
        section: &SectionUniforms,
    ) -> Self {
        let view = camera.view_matrix();
        let r = view.row(0);
        let u = view.row(1);
        let f = -view.row(2);
        let epsilon = ((max - min).max_element() * 1e-6).max(1e-5);
        Self {
            eye: [camera.eye.x, camera.eye.y, camera.eye.z, epsilon],
            lo: [min.x, min.y, min.z, 0.0],
            hi: [max.x, max.y, max.z, 0.0],
            right: [r.x, r.y, r.z, camera.aspect],
            up: [u.x, u.y, u.z, (camera.fov * 0.5).tan()],
            forward: [f.x, f.y, f.z, 0.0],
            viewport: [offset[0], offset[1], size[0], size[1]],
            section_a: [
                section.plane_a_enabled as u32 as f32,
                section.plane_a_axis as f32,
                section.plane_a_position,
                section.plane_a_flip as u32 as f32,
            ],
            section_b: [
                section.plane_b_enabled as u32 as f32,
                section.plane_b_axis as f32,
                section.plane_b_position,
                section.plane_b_flip as u32 as f32,
            ],
        }
    }
}

pub struct DirectSdfRenderer {
    pipeline: wgpu::RenderPipeline,
    uniform: wgpu::Buffer,
    binding: wgpu::BindGroup,
    pub scene: Arc<DirectScene>,
}
impl DirectSdfRenderer {
    pub fn new(
        device: &wgpu::Device,
        format: wgpu::TextureFormat,
        scene: Arc<DirectScene>,
        diagnostic: bool,
    ) -> Result<Self, String> {
        let _timing = telemetry::Span::new("direct_shader_pipeline_compile");
        device.push_error_scope(wgpu::ErrorFilter::Validation);
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Direct SDF shader"),
            source: wgpu::ShaderSource::Wgsl(
                format!(
                    "{}\n{}",
                    scene.field,
                    include_str!("../../shaders/direct_sdf.wgsl")
                )
                .into(),
            ),
        });
        let uniform = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Direct SDF uniforms"),
            size: std::mem::size_of::<DirectUniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: None,
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let binding = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform.as_entire_binding(),
            }],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[&layout],
            push_constant_ranges: &[],
        });
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Direct SDF viewport"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                buffers: &[],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: if diagnostic { "fs_probe" } else { "fs_main" },
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: if diagnostic {
                        None
                    } else {
                        Some(wgpu::BlendState::ALPHA_BLENDING)
                    },
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: Default::default(),
            depth_stencil: None,
            multisample: Default::default(),
            multiview: None,
            cache: None,
        });
        if let Some(e) = pollster::block_on(device.pop_error_scope()) {
            return Err(e.to_string());
        }
        Ok(Self {
            pipeline,
            uniform,
            binding,
            scene,
        })
    }
    pub fn update(
        &self,
        queue: &wgpu::Queue,
        camera: &Camera,
        offset: [f32; 2],
        size: [f32; 2],
        section: &SectionUniforms,
    ) {
        queue.write_buffer(
            &self.uniform,
            0,
            bytemuck::bytes_of(&DirectUniforms::new(
                camera,
                self.scene.bounds_min,
                self.scene.bounds_max,
                offset,
                size,
                section,
            )),
        );
    }
    pub fn render(&self, pass: &mut wgpu::RenderPass<'static>) {
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.binding, &[]);
        pass.draw(0..3, 0..1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn direct_capability_is_stricter_than_gpu_lowering() {
        for source in [
            "shell(sphere(2.0),0.1)",
            "offset(sphere(2.0),1.0)",
            "scale(sphere(2.0),1.0,2.0,1.0)",
            "scale(sphere(2.0),-1.0,-1.0,-1.0)",
            "smooth_union(sphere(2.0),sphere(3.0),0.2)",
            "cylinder(1.0,2.0)",
            "twist(sphere(2.0),0.0,0.0,1.0,0.5)",
            "bend(sphere(2.0),0.0,0.0,1.0,0.1)",
        ] {
            let s = crate::scripting::evaluate_script(source).unwrap();
            let ir = crate::gpu::lower_sdf_ir(s.canonical_sdf.as_ref()).unwrap();
            assert!(
                matches!(capability(&ir), DirectCapability::FallbackRequired(_)),
                "{source}"
            );
        }
    }
    #[test]
    fn deep_graph_fails_closed() {
        let mut ir = SdfIr::Sphere { radius: 1.0 };
        for _ in 0..50 {
            ir = SdfIr::Translate {
                child: Box::new(ir),
                offset: [0.0; 3],
            };
        }
        assert!(field_source(&ir).is_err());
    }
    #[test]
    fn invalid_values_and_custom_nodes_fail_closed() {
        struct Custom;
        impl Sdf for Custom {
            fn distance(&self, p: Vec3) -> f32 {
                p.length() - 1.0
            }
        }
        assert!(
            DirectScene::from_sdf(Arc::new(Custom), Vec3::splat(-2.0), Vec3::splat(2.0)).is_err()
        );
        for radius in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            assert!(field_source(&SdfIr::Sphere { radius }).is_err());
        }
    }
}
