// XYZ axis indicator shader

struct Uniforms {
    rotation: mat4x4<f32>,  // Only rotation part of view matrix
};

@group(0) @binding(0)
var<uniform> uniforms: Uniforms;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) color: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec3<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;

    // Apply rotation to the axis direction
    let rotated = uniforms.rotation * vec4<f32>(in.position, 0.0);

    // Project to screen space with fixed orthographic projection
    // Position in bottom-left corner of viewport
    let scale = 0.15;  // Size of axes
    let offset = vec2<f32>(-0.85, -0.85);  // Position in corner

    let screen_pos = vec2<f32>(rotated.x, rotated.z) * scale + offset;
    out.clip_position = vec4<f32>(screen_pos, 0.0, 1.0);
    out.color = in.color;

    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    return vec4<f32>(in.color, 1.0);
}
