// Sphere-tracing renderer: uploads a 3D SDF texture and ray-marches it on the GPU.

use std::sync::Arc;
use eframe::wgpu;
use glam::Vec3;
use super::camera::Camera;

/// Flat section-view data passed to the shader via the uniform buffer.
#[derive(Copy, Clone)]
pub struct SectionUniforms {
    pub plane_a_enabled:  bool,
    pub plane_a_axis:     u32,
    pub plane_a_position: f32,
    pub plane_a_flip:     bool,
    pub plane_b_enabled:  bool,
    pub plane_b_axis:     u32,
    pub plane_b_position: f32,
    pub plane_b_flip:     bool,
}

impl Default for SectionUniforms {
    fn default() -> Self {
        Self {
            plane_a_enabled: false, plane_a_axis: 0, plane_a_position: 0.0, plane_a_flip: false,
            plane_b_enabled: false, plane_b_axis: 0, plane_b_position: 0.0, plane_b_flip: false,
        }
    }
}

/// Thickness / stress overlay parameters for the shader.
#[derive(Copy, Clone)]
pub struct ThicknessUniforms {
    pub enabled:      bool,
    pub min_display:  f32,
    pub max_display:  f32,
    /// When `true`, the colour map is inverted (red = high, green = low).
    /// Used for stress overlays where red = at/above yield, green = safe.
    pub invert:       bool,
}

impl Default for ThicknessUniforms {
    fn default() -> Self {
        Self { enabled: false, min_display: 0.0, max_display: 10.0, invert: false }
    }
}

#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct Uniforms {
    // Ray generation: camera basis + FOV (avoids projection-matrix convention issues)
    camera_pos:      [f32; 3],
    voxel_size:      f32,
    bounds_min:      [f32; 3],
    _pad1:           f32,
    bounds_max:      [f32; 3],
    _pad2:           f32,
    viewport_offset: [f32; 2],
    viewport_size:   [f32; 2],
    camera_right:    [f32; 3],
    tan_half_fov:    f32,
    camera_up:       [f32; 3],
    aspect:          f32,
    camera_fwd:      [f32; 3],
    _pad3:           f32,
    // Section view — 8 × u32/f32 = 32 bytes  (total so far: 144 B)
    section_a_enabled: u32,
    section_a_axis:    u32,
    section_a_pos:     f32,
    section_a_flip:    u32,
    section_b_enabled: u32,
    section_b_axis:    u32,
    section_b_pos:     f32,
    section_b_flip:    u32,
    // Thickness / stress overlay — 4 × u32/f32 = 16 bytes  (total: 160 B, 16-B aligned)
    thickness_enabled: u32,
    thickness_min:     f32,
    thickness_max:     f32,
    overlay_invert:    u32,
}

/// CPU-side SDF grid, produced after script evaluation.
pub struct SdfGrid {
    /// Flat f32 values, index = x + y*res + z*res*res (X fastest).
    pub data:       Vec<f32>,
    pub resolution: u32,
    pub bounds_min: Vec3,
    pub bounds_max: Vec3,
}

impl SdfGrid {
    /// Clone all grid data into an owned struct (for background threads).
    pub fn clone_data(&self) -> SdfGrid {
        SdfGrid {
            data:       self.data.clone(),
            resolution: self.resolution,
            bounds_min: self.bounds_min,
            bounds_max: self.bounds_max,
        }
    }
}

pub struct RaymarchRenderer {
    pipeline:               wgpu::RenderPipeline,
    uniform_buffer:         wgpu::Buffer,
    bind_group_layout:      wgpu::BindGroupLayout,
    bind_group:             Option<wgpu::BindGroup>,
    /// The last-uploaded grid, kept to skip redundant uploads.
    pub last_grid:          Option<Arc<SdfGrid>>,
    /// Current SDF texture view (set after each grid upload).
    sdf_view:               Option<Arc<wgpu::TextureView>>,
    /// Dummy 1×1×1 R32Float texture used when no thickness data is available.
    dummy_thickness_view:   Arc<wgpu::TextureView>,
    /// Current thickness texture view, or None if no analysis has been run.
    thickness_view:         Option<Arc<wgpu::TextureView>>,
}

fn make_bind_group_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label:   Some("Raymarch BGL"),
        entries: &[
            // binding 0: uniform buffer
            wgpu::BindGroupLayoutEntry {
                binding:    0,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty:                 wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size:   None,
                },
                count: None,
            },
            // binding 1: SDF 3D texture
            wgpu::BindGroupLayoutEntry {
                binding:    1,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type:    wgpu::TextureSampleType::Float { filterable: false },
                    view_dimension: wgpu::TextureViewDimension::D3,
                    multisampled:   false,
                },
                count: None,
            },
            // binding 2: thickness 3D texture
            wgpu::BindGroupLayoutEntry {
                binding:    2,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type:    wgpu::TextureSampleType::Float { filterable: false },
                    view_dimension: wgpu::TextureViewDimension::D3,
                    multisampled:   false,
                },
                count: None,
            },
        ],
    })
}


impl RaymarchRenderer {
    pub fn new(device: &wgpu::Device, surface_format: wgpu::TextureFormat) -> Self {
        // Note: `new` has no queue — supply a temporary one-shot queue just for the
        // dummy texture. The dummy is tiny (1 float) so this is negligible.
        // Actually we can't do that. Use device.create_texture + write_texture pattern.
        let dummy_thickness_view = {
            let tex = device.create_texture(&wgpu::TextureDescriptor {
                label:           Some("Dummy Thickness Texture"),
                size:            wgpu::Extent3d { width: 1, height: 1, depth_or_array_layers: 1 },
                mip_level_count: 1,
                sample_count:    1,
                dimension:       wgpu::TextureDimension::D3,
                format:          wgpu::TextureFormat::R32Float,
                usage:           wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
                view_formats:    &[],
            });
            Arc::new(tex.create_view(&wgpu::TextureViewDescriptor {
                dimension: Some(wgpu::TextureViewDimension::D3),
                ..Default::default()
            }))
        };

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label:  Some("Raymarch Shader"),
            source: wgpu::ShaderSource::Wgsl(
                include_str!("../../shaders/raymarch.wgsl").into(),
            ),
        });

        let uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label:              Some("Raymarch Uniforms"),
            size:               std::mem::size_of::<Uniforms>() as u64,
            usage:              wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let bind_group_layout = make_bind_group_layout(device);

        let pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label:                Some("Raymarch Pipeline Layout"),
                bind_group_layouts:   &[&bind_group_layout],
                push_constant_ranges: &[],
            });

        let pipeline =
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label:  Some("Raymarch Pipeline"),
                layout: Some(&pipeline_layout),
                vertex: wgpu::VertexState {
                    module:               &shader,
                    entry_point:          "vs_main",
                    buffers:              &[],
                    compilation_options:  Default::default(),
                },
                fragment: Some(wgpu::FragmentState {
                    module:      &shader,
                    entry_point: "fs_main",
                    targets: &[Some(wgpu::ColorTargetState {
                        format:     surface_format,
                        blend:      Some(wgpu::BlendState::ALPHA_BLENDING),
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                    compilation_options: Default::default(),
                }),
                primitive: wgpu::PrimitiveState {
                    topology:           wgpu::PrimitiveTopology::TriangleList,
                    cull_mode:          None,
                    ..Default::default()
                },
                depth_stencil: None,
                multisample:   wgpu::MultisampleState::default(),
                multiview:     None,
                cache:         None,
            });

        Self {
            pipeline,
            uniform_buffer,
            bind_group_layout,
            bind_group: None,
            last_grid:  None,
            sdf_view:               None,
            dummy_thickness_view,
            thickness_view:         None,
        }
    }

    /// Upload a new SDF grid as a 3D texture and rebuild the bind group.
    pub fn upload_grid(
        &mut self,
        device: &wgpu::Device,
        queue:  &wgpu::Queue,
        grid:   &Arc<SdfGrid>,
    ) {
        let res = grid.resolution;

        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("SDF 3D Texture"),
            size: wgpu::Extent3d {
                width:                 res,
                height:                res,
                depth_or_array_layers: res,
            },
            mip_level_count: 1,
            sample_count:    1,
            dimension:       wgpu::TextureDimension::D3,
            format:          wgpu::TextureFormat::R32Float,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });

        queue.write_texture(
            texture.as_image_copy(),
            bytemuck::cast_slice(&grid.data),
            wgpu::ImageDataLayout {
                offset:         0,
                bytes_per_row:  Some(res * 4),
                rows_per_image: Some(res),
            },
            wgpu::Extent3d {
                width:                 res,
                height:                res,
                depth_or_array_layers: res,
            },
        );

        let view = Arc::new(texture.create_view(&wgpu::TextureViewDescriptor {
            dimension: Some(wgpu::TextureViewDimension::D3),
            ..Default::default()
        }));

        self.sdf_view  = Some(Arc::clone(&view));
        self.last_grid = Some(Arc::clone(grid));
        self.rebuild_bind_group(device);
    }

    /// Upload thickness analysis data as a 3D texture and rebuild the bind group.
    pub fn upload_thickness(
        &mut self,
        device:     &wgpu::Device,
        queue:      &wgpu::Queue,
        data:       &[f32],
        resolution: u32,
    ) {
        let res = resolution;
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("Thickness 3D Texture"),
            size: wgpu::Extent3d {
                width:                 res,
                height:                res,
                depth_or_array_layers: res,
            },
            mip_level_count: 1,
            sample_count:    1,
            dimension:       wgpu::TextureDimension::D3,
            format:          wgpu::TextureFormat::R32Float,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats:    &[],
        });

        queue.write_texture(
            texture.as_image_copy(),
            bytemuck::cast_slice(data),
            wgpu::ImageDataLayout {
                offset:         0,
                bytes_per_row:  Some(res * 4),
                rows_per_image: Some(res),
            },
            wgpu::Extent3d {
                width:                 res,
                height:                res,
                depth_or_array_layers: res,
            },
        );

        let view = Arc::new(texture.create_view(&wgpu::TextureViewDescriptor {
            dimension: Some(wgpu::TextureViewDimension::D3),
            ..Default::default()
        }));
        self.thickness_view = Some(view);
        self.rebuild_bind_group(device);
    }

    /// Clear the thickness overlay (revert to dummy texture).
    pub fn clear_thickness(&mut self, device: &wgpu::Device) {
        self.thickness_view = None;
        self.rebuild_bind_group(device);
    }

    fn rebuild_bind_group(&mut self, device: &wgpu::Device) {
        let Some(ref sdf_view) = self.sdf_view else { return };
        let thickness_view = self.thickness_view.as_ref().unwrap_or(&self.dummy_thickness_view);

        self.bind_group = Some(device.create_bind_group(&wgpu::BindGroupDescriptor {
            label:   Some("Raymarch Bind Group"),
            layout:  &self.bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding:  0,
                    resource: self.uniform_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding:  1,
                    resource: wgpu::BindingResource::TextureView(sdf_view),
                },
                wgpu::BindGroupEntry {
                    binding:  2,
                    resource: wgpu::BindingResource::TextureView(thickness_view),
                },
            ],
        }));
    }

    pub fn update_uniforms(
        &self,
        queue:           &wgpu::Queue,
        camera:          &Camera,
        viewport_offset: [f32; 2],
        viewport_size:   [f32; 2],
        section:         &SectionUniforms,
        thickness:       &ThicknessUniforms,
    ) {
        let Some(ref grid) = self.last_grid else { return };

        let span = (grid.bounds_max - grid.bounds_min)
            .max(Vec3::splat(1.0));
        let voxel_size = (span.x.max(span.y).max(span.z)) / grid.resolution as f32;

        // Extract camera basis vectors from the view matrix rows.
        // look_at_rh rows: [right, up, -forward, translation_row]
        let view = camera.view_matrix();
        let r0 = view.row(0);
        let r1 = view.row(1);
        let r2 = view.row(2);
        let camera_right = [r0.x, r0.y, r0.z];
        let camera_up    = [r1.x, r1.y, r1.z];
        let camera_fwd   = [-r2.x, -r2.y, -r2.z]; // row2 = -forward

        let u = Uniforms {
            camera_pos:      camera.eye.to_array(),
            voxel_size,
            bounds_min:      grid.bounds_min.to_array(),
            _pad1:           0.0,
            bounds_max:      grid.bounds_max.to_array(),
            _pad2:           0.0,
            viewport_offset,
            viewport_size,
            camera_right,
            tan_half_fov:    (camera.fov * 0.5).tan(),
            camera_up,
            aspect:          camera.aspect,
            camera_fwd,
            _pad3:           0.0,
            section_a_enabled: section.plane_a_enabled as u32,
            section_a_axis:    section.plane_a_axis,
            section_a_pos:     section.plane_a_position,
            section_a_flip:    section.plane_a_flip as u32,
            section_b_enabled: section.plane_b_enabled as u32,
            section_b_axis:    section.plane_b_axis,
            section_b_pos:     section.plane_b_position,
            section_b_flip:    section.plane_b_flip as u32,
            thickness_enabled: thickness.enabled as u32,
            thickness_min:     thickness.min_display,
            thickness_max:     thickness.max_display,
            overlay_invert:    thickness.invert as u32,
        };

        queue.write_buffer(&self.uniform_buffer, 0, bytemuck::cast_slice(&[u]));
    }

    pub fn render(&self, render_pass: &mut wgpu::RenderPass<'static>) {
        let Some(ref bind_group) = self.bind_group else { return };
        render_pass.set_pipeline(&self.pipeline);
        render_pass.set_bind_group(0, bind_group, &[]);
        render_pass.draw(0..3, 0..1);
    }
}
