//! Viewport-only shared-spline loft evaluation. No polygon substitution.
use super::*;
use crate::sdf::aerospace::naca_spline::{CubicSpan, NacaSpline};
use crate::sdf::aerospace::{Airfoil, LoftedWing};
use std::sync::Arc;
use wgpu::util::DeviceExt;

#[derive(Clone, Debug)]
pub struct SplineWing {
    pub spline: Arc<NacaSpline>,
    /// Leading edge xyz, chord. Twist is limited to zero in this qualification.
    pub sections: [[f32; 4]; 3],
}
impl SplineWing {
    fn lower(w: &LoftedWing) -> Option<Self> {
        if w.sections.len() != 3
            || w.sections
                .iter()
                .any(|s| s.twist != 0.0 || !s.position.is_finite())
        {
            return None;
        }
        let mut sections = [[0.; 4]; 3];
        let mut spline: Option<Arc<NacaSpline>> = None;
        for (i, s) in w.sections.iter().enumerate() {
            let a = s.section.as_any().downcast_ref::<Airfoil>()?;
            if !a.chord.is_finite() || a.chord <= 0. {
                return None;
            }
            let curve = a.spline()?;
            if let Some(old) = &spline {
                if old.as_ref() != curve.as_ref() {
                    return None;
                }
            }
            spline = Some(curve.clone());
            sections[i] = [s.position.x, s.position.y, s.position.z, a.chord];
        }
        if sections.windows(2).any(|p| p[1][1] - p[0][1] < 1e-6) {
            return None;
        }
        let spline = spline?;
        if spline.spans.len() < 2
            || spline.spans.len() > 4096
            || spline.spans.iter().any(|s| {
                s.control
                    .iter()
                    .flatten()
                    .any(|v| !v.is_finite() || v.abs() > f32::MAX as f64)
            })
        {
            return None;
        }
        Some(Self { spline, sections })
    }
    pub fn expression(&self, p: &str) -> String {
        let v = |s: [f32; 4]| format!("vec4<f32>({:?},{:?},{:?},{:?})", s[0], s[1], s[2], s[3]);
        format!(
            "spline_wing({p},{},{},{})",
            v(self.sections[0]),
            v(self.sections[1]),
            v(self.sections[2])
        )
    }
    pub fn distance(&self, p: Vec3) -> f32 {
        let y = p.y.clamp(self.sections[0][1], self.sections[2][1]);
        let i = if y <= self.sections[1][1] { 0 } else { 1 };
        let a = self.sections[i];
        let b = self.sections[i + 1];
        let t = (y - a[1]) / (b[1] - a[1]);
        let c = a[3] + t * (b[3] - a[3]);
        let q = glam::Vec2::new(
            p.x - (a[0] + t * (b[0] - a[0])),
            p.z - (a[2] + t * (b[2] - a[2])),
        );
        let d = self
            .spline
            .closest_point(q.as_dvec2() / c as f64)
            .map_or(f32::NAN, |v| (v.signed_distance * c as f64) as f32);
        let ds = (self.sections[0][1] - p.y).max(p.y - self.sections[2][1]);
        if !d.is_finite() {
            return d;
        }
        glam::Vec2::new(d.max(0.), ds.max(0.)).length() + d.max(ds).min(0.)
    }
}

/// Deliberately separate from production lower_sdf_ir: exports keep their old route.
pub fn lower_viewport_sdf_ir(sdf: &dyn Sdf) -> Option<SdfIr> {
    if let Some(ir) = super::lower_sdf_ir(sdf) {
        return Some(ir);
    }
    let any = sdf as &dyn Any;
    if let Some(w) = any.downcast_ref::<LoftedWing>() {
        return SplineWing::lower(w).map(SdfIr::SplineWing);
    }
    if let Some(t) = any.downcast_ref::<Translate>() {
        return Some(SdfIr::Translate {
            child: Box::new(lower_viewport_sdf_ir(t.child.as_ref())?),
            offset: t.offset.to_array(),
        });
    }
    if let Some(t) = any.downcast_ref::<Rotate>() {
        return Some(SdfIr::Rotate {
            child: Box::new(lower_viewport_sdf_ir(t.child.as_ref())?),
            inverse_basis: mat3_to_rows(Mat3::from_quat(t.rotation.inverse())),
        });
    }
    if let Some(t) = any.downcast_ref::<Scale>() {
        return Some(SdfIr::Scale {
            child: Box::new(lower_viewport_sdf_ir(t.child.as_ref())?),
            scale: t.scale.to_array(),
            min_scale: t.scale.min_element(),
        });
    }
    if let Some(t) = any.downcast_ref::<Union>() {
        return Some(SdfIr::Union {
            a: Box::new(lower_viewport_sdf_ir(t.a.as_ref())?),
            b: Box::new(lower_viewport_sdf_ir(t.b.as_ref())?),
        });
    }
    if let Some(t) = any.downcast_ref::<Intersect>() {
        return Some(SdfIr::Intersect {
            a: Box::new(lower_viewport_sdf_ir(t.a.as_ref())?),
            b: Box::new(lower_viewport_sdf_ir(t.b.as_ref())?),
        });
    }
    if let Some(t) = any.downcast_ref::<Subtract>() {
        return Some(SdfIr::Subtract {
            a: Box::new(lower_viewport_sdf_ir(t.a.as_ref())?),
            b: Box::new(lower_viewport_sdf_ir(t.b.as_ref())?),
        });
    }
    None
}
fn curves<'a>(ir: &'a SdfIr, out: &mut Vec<&'a Arc<NacaSpline>>) {
    match ir {
        SdfIr::SplineWing(w) => out.push(&w.spline),
        SdfIr::Union { a, b }
        | SdfIr::Intersect { a, b }
        | SdfIr::Subtract { a, b }
        | SdfIr::SmoothUnion { a, b, .. }
        | SdfIr::SmoothIntersect { a, b, .. }
        | SdfIr::SmoothSubtract { a, b, .. } => {
            curves(a, out);
            curves(b, out)
        }
        SdfIr::Translate { child, .. }
        | SdfIr::Rotate { child, .. }
        | SdfIr::Scale { child, .. }
        | SdfIr::Offset { child, .. }
        | SdfIr::Shell { child, .. }
        | SdfIr::Twist { child, .. }
        | SdfIr::Bend { child, .. } => curves(child, out),
        _ => {}
    }
}
pub fn has_wing(ir: &SdfIr) -> bool {
    let mut c = vec![];
    curves(ir, &mut c);
    !c.is_empty()
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Node {
    bounds: [f32; 4],
    meta: [u32; 4],
    ab: [f32; 4],
    cd: [f32; 4],
}
fn nodes(ir: &SdfIr) -> Result<Vec<Node>, String> {
    let mut c = vec![];
    curves(ir, &mut c);
    let s = c.first().ok_or("no spline")?;
    if c.iter().any(|x| x.as_ref() != s.as_ref()) {
        return Err("different spline profiles require CPU fallback".into());
    }
    fn build(spans: &[CubicSpan], out: &mut Vec<Node>) {
        let id = out.len();
        out.push(Node::zeroed());
        let mut lo = [f32::INFINITY; 2];
        let mut hi = [f32::NEG_INFINITY; 2];
        for s in spans {
            for p in s.control {
                for k in 0..2 {
                    lo[k] = lo[k].min(p[k] as f32);
                    hi[k] = hi[k].max(p[k] as f32)
                }
            }
        }
        out[id].bounds = [lo[0], lo[1], hi[0], hi[1]];
        if spans.len() == 1 {
            let p = spans[0].control.map(|p| p.map(|x| x as f32));
            out[id].meta[1] = 1;
            out[id].ab = [p[0][0], p[0][1], p[1][0], p[1][1]];
            out[id].cd = [p[2][0], p[2][1], p[3][0], p[3][1]];
        } else {
            let m = spans.len() / 2;
            build(&spans[..m], out);
            build(&spans[m..], out)
        }
        out[id].meta[0] = out.len() as u32;
    }
    let mut out = vec![];
    build(&s.spans, &mut out);
    // Explicit closing trailing face, not a rounded or blended skin segment.
    let a = s.spans.last().unwrap().control[3];
    let b = s.spans[0].control[0];
    out[0].ab = [a[0] as f32, a[1] as f32, b[0] as f32, b[1] as f32];
    Ok(out)
}

pub(super) fn run(
    ir: &SdfIr,
    shader: String,
    params: &[u8],
    points: Option<&[[f32; 4]]>,
    len: usize,
    groups: [u32; 3],
    conditioned: Option<&super::conditioned::ConditionedViewportIr>,
) -> Result<Vec<f32>, String> {
    let t = std::time::Instant::now();
    let ctx = super::gpu_context()?;
    crate::render::telemetry::record("wing_device_acquire", t.elapsed().as_secs_f64() * 1000.);
    let t = std::time::Instant::now();
    let data = nodes(ir)?;
    crate::render::telemetry::record("wing_bvh_build", t.elapsed().as_secs_f64() * 1000.);
    let shader = format!(
        "{}\n{}",
        include_str!("../../shaders/spline_wing.wgsl"),
        shader
    );
    ctx.device.push_error_scope(wgpu::ErrorFilter::Validation);
    let module_start = std::time::Instant::now();
    let module = ctx
        .device
        .create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Viewport spline wing"),
            source: wgpu::ShaderSource::Wgsl(shader.into()),
        });
    crate::render::telemetry::record(
        "wing_shader_module",
        module_start.elapsed().as_secs_f64() * 1000.,
    );
    let pipeline_start = std::time::Instant::now();
    let pipeline = ctx
        .device
        .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("Viewport spline wing"),
            layout: None,
            module: &module,
            entry_point: "main",
            compilation_options: Default::default(),
            cache: None,
        });
    if let Some(e) = pollster::block_on(ctx.device.pop_error_scope()) {
        return Err(e.to_string());
    }
    crate::render::telemetry::record(
        "wing_pipeline_compile",
        pipeline_start.elapsed().as_secs_f64() * 1000.,
    );
    crate::render::telemetry::record(
        "wing_shader_pipeline_and_bvh",
        t.elapsed().as_secs_f64() * 1000.,
    );
    let buffer = |label, bytes: &[u8], usage| {
        ctx.device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(label),
                contents: bytes,
                usage,
            })
    };
    let upload_start = std::time::Instant::now();
    let conditioned_buffers = conditioned.map(|c| {
        [
            buffer(
                "Conditioned tiles",
                bytemuck::cast_slice(&c.tiles),
                wgpu::BufferUsages::STORAGE,
            ),
            buffer(
                "Conditioned values",
                bytemuck::cast_slice(&c.values),
                wgpu::BufferUsages::STORAGE,
            ),
            buffer(
                "Conditioned config",
                bytemuck::bytes_of(&c.config),
                wgpu::BufferUsages::UNIFORM,
            ),
        ]
    });
    if conditioned.is_some() {
        crate::render::telemetry::record(
            "conditioned_data_buffer_creation_upload",
            upload_start.elapsed().as_secs_f64() * 1000.,
        );
    }
    let parameters = buffer(
        "Wing query params",
        params,
        wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
    );
    let curves = buffer(
        "Canonical cubic BVH",
        bytemuck::cast_slice(&data),
        wgpu::BufferUsages::STORAGE,
    );
    let input = points.map(|p| {
        buffer(
            "Query points",
            bytemuck::cast_slice(p),
            wgpu::BufferUsages::STORAGE,
        )
    });
    let size = (len * 4) as u64;
    let output = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("Wing field"),
        size,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    let failed = buffer(
        "Wing query failure flag",
        &[0u8; 4],
        wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
    );
    let staging = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("Wing readback"),
        size: size + 4,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let mut entries = vec![
        wgpu::BindGroupEntry {
            binding: 0,
            resource: parameters.as_entire_binding(),
        },
        wgpu::BindGroupEntry {
            binding: 3,
            resource: curves.as_entire_binding(),
        },
        wgpu::BindGroupEntry {
            binding: 4,
            resource: failed.as_entire_binding(),
        },
    ];
    if let Some(buffers) = &conditioned_buffers {
        for (i, buffer) in buffers.iter().enumerate() {
            entries.push(wgpu::BindGroupEntry {
                binding: 5 + i as u32,
                resource: buffer.as_entire_binding(),
            });
        }
    }
    if let Some(input) = &input {
        entries.push(wgpu::BindGroupEntry {
            binding: 1,
            resource: input.as_entire_binding(),
        });
    }
    entries.push(wgpu::BindGroupEntry {
        binding: if input.is_some() { 2 } else { 1 },
        resource: output.as_entire_binding(),
    });
    let bind = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: None,
        layout: &pipeline.get_bind_group_layout(0),
        entries: &entries,
    });
    let mut submit_ms = 0.;
    let mut completion_ms = 0.;
    // Bound each wing grid dispatch to four Z planes to avoid long watchdog jobs.
    let chunks = if input.is_some() { 1 } else { groups[2] };
    for z in 0..chunks {
        if input.is_none() {
            ctx.queue
                .write_buffer(&parameters, 28, bytemuck::bytes_of(&(z * 4)));
        }
        let t = std::time::Instant::now();
        let mut enc = ctx.device.create_command_encoder(&Default::default());
        {
            let mut pass = enc.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: None,
                timestamp_writes: None,
            });
            pass.set_pipeline(&pipeline);
            pass.set_bind_group(0, &bind, &[]);
            pass.dispatch_workgroups(groups[0], groups[1], 1);
        }
        ctx.queue.submit(Some(enc.finish()));
        submit_ms += t.elapsed().as_secs_f64() * 1000.;
        let t = std::time::Instant::now();
        ctx.device.poll(wgpu::Maintain::Wait);
        completion_ms += t.elapsed().as_secs_f64() * 1000.;
    }
    let t = std::time::Instant::now();
    let mut enc = ctx.device.create_command_encoder(&Default::default());
    enc.copy_buffer_to_buffer(&output, 0, &staging, 0, size);
    enc.copy_buffer_to_buffer(&failed, 0, &staging, size, 4);
    ctx.queue.submit(Some(enc.finish()));
    submit_ms += t.elapsed().as_secs_f64() * 1000.;
    let t = std::time::Instant::now();
    ctx.device.poll(wgpu::Maintain::Wait);
    completion_ms += t.elapsed().as_secs_f64() * 1000.;
    crate::render::telemetry::record("wing_dispatch_submit", submit_ms);
    crate::render::telemetry::record("wing_gpu_completion_including_copy", completion_ms);
    let t = std::time::Instant::now();
    let mut result = super::read_back_f32_buffer(&ctx.device, &staging, len + 1)?;
    crate::render::telemetry::record("wing_map_and_cpu_copy", t.elapsed().as_secs_f64() * 1000.);
    let flags = result.pop().unwrap().to_bits();
    if flags != 0 || result.iter().any(|v| !v.is_finite()) {
        return Err(format!(
            "spline query unresolved (flags {flags}); CPU fallback required"
        ));
    }
    Ok(result)
}
pub fn grid(ir: &SdfIr, lo: Vec3, hi: Vec3, res: u32) -> Result<SdfGrid, String> {
    // This API remains cubic for existing callers.
    let p = GridParams {
        bounds_min: lo.to_array(),
        bounds_max: hi.to_array(),
        resolution: res,
        _pad: 0,
    };
    let data = run(
        ir,
        super::build_grid_shader_offset(ir, true),
        bytemuck::bytes_of(&p),
        None,
        (res as usize).pow(3),
        [(res + 3) / 4; 3],
        None,
    )?;
    Ok(SdfGrid {
        data,
        resolution: res,
        bounds_min: lo,
        bounds_max: hi,
    })
}
pub fn points(ir: &SdfIr, points: &[Vec3]) -> Result<Vec<f32>, String> {
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
    run(
        ir,
        super::build_points_shader(ir),
        bytemuck::bytes_of(&p),
        Some(&input),
        points.len(),
        [(points.len() as u32 + 63) / 64, 1, 1],
        None,
    )
}
pub fn viewport_grid(ir:&SdfIr,lo:Vec3,hi:Vec3,spec:crate::render::viewport_grid::ViewportGridSpec)->Result<crate::render::viewport_grid::ViewportSdfGrid,String>{
    let params=super::ViewportGridParams::new(lo,hi,spec);
    let data=run(ir,super::build_viewport_grid_shader(ir),bytemuck::bytes_of(&params),None,spec.cell_count(),spec.dimensions().map(|n|n.div_ceil(4)),None)?;
    Ok(crate::render::viewport_grid::ViewportSdfGrid{data,spec,bounds_min:lo,bounds_max:hi})
}

#[cfg(test)]
mod tests {
    use super::*;
    fn model() -> LoftedWing {
        crate::sdf::aerospace::wing_with_airfoil("0012", 60., 30., 120., 5., 0., 0.)
    }
    #[test]
    fn viewport_only_and_unsupported_cases() {
        let mut w = model();
        assert!(super::super::lower_sdf_ir(&w).is_none());
        let ir = lower_viewport_sdf_ir(&w).unwrap();
        assert!(matches!(
            crate::render::direct::capability(&ir),
            crate::render::direct::DirectCapability::FallbackRequired(_)
        ));
        w.sections[1].twist = 0.01;
        assert!(lower_viewport_sdf_ir(&w).is_none());
        let mut w = model();
        w.sections[1].section = crate::sdf::aerospace::get_naca_airfoil("2412", 60.);
        assert!(lower_viewport_sdf_ir(&w).is_none());
        let mut w = model();
        w.sections[1].section = Arc::new(Airfoil::new(vec![(0., 0.), (1., 0.), (0., 0.)], 60.));
        assert!(lower_viewport_sdf_ir(&w).is_none());
    }
    #[test]
    fn wing_points_match_cpu_existing_point_rule() {
        let w = model();
        let ir = lower_viewport_sdf_ir(&w).unwrap();
        let p = [
            Vec3::new(15., 0., 0.),
            Vec3::new(0., 0., 0.001),
            Vec3::new(60., 0., 0.),
            Vec3::new(35.25, 60., 0.),
            Vec3::new(20., 30., 3.),
            Vec3::new(20., 30., -3.),
        ];
        let g = points(&ir, &p).unwrap();
        for (p, g) in p.into_iter().zip(g) {
            let c = w.distance(p);
            assert!((c - g).abs() < 1e-4, "{p:?}: CPU {c} GPU {g}");
        }
    }
    #[test]
    fn unresolved_query_is_an_explicit_error() {
        let mut ir = lower_viewport_sdf_ir(&model()).unwrap();
        let SdfIr::SplineWing(w) = &mut ir else {
            unreachable!()
        };
        let s = Arc::make_mut(&mut w.spline);
        let first = s.spans[0].control[0];
        s.spans.last_mut().unwrap().control[3] = first;
        assert!(points(&ir, &[Vec3::ZERO]).unwrap_err().contains("flags 1"));
    }
}
