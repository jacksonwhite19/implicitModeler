// Vertex shader

struct Uniforms {
    view_proj: mat4x4<f32>,
    light_dir: vec3<f32>,
};

@group(0) @binding(0)
var<uniform> uniforms: Uniforms;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) world_normal: vec3<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    out.clip_position = uniforms.view_proj * vec4<f32>(in.position, 1.0);
    out.world_normal = in.normal;
    return out;
}

// Fragment shader

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    // Material color (light gray-blue)
    let base_color = vec3<f32>(0.75, 0.78, 0.82);

    // Normalize normal
    let normal = normalize(in.world_normal);

    // Key light (main directional)
    let key_light_dir = normalize(-uniforms.light_dir);
    let key_diffuse = max(dot(normal, key_light_dir), 0.0);

    // Fill light (softer, from opposite side)
    let fill_light_dir = normalize(vec3<f32>(-0.3, 0.5, 0.6));
    let fill_diffuse = max(dot(normal, fill_light_dir), 0.0) * 0.3;

    // Rim light (from behind/above)
    let rim_light_dir = normalize(vec3<f32>(0.0, -0.3, 0.8));
    let rim_diffuse = max(dot(normal, rim_light_dir), 0.0) * 0.2;

    // Combine lighting
    let ambient = 0.35;
    let diffuse = key_diffuse + fill_diffuse + rim_diffuse;
    let lighting = ambient + (1.0 - ambient) * diffuse;

    let color = base_color * lighting;
    return vec4<f32>(color, 1.0);
}
