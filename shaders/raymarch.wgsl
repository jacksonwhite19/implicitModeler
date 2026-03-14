// Sphere-tracing renderer against a precomputed 3D SDF texture.
// Uses textureLoad + manual trilinear interpolation (avoids R32Float filterability issues).
// Ray generation uses camera basis vectors to avoid projection-matrix convention issues.

struct Uniforms {
    camera_pos:      vec3<f32>,
    voxel_size:      f32,
    bounds_min:      vec3<f32>,
    _pad1:           f32,
    bounds_max:      vec3<f32>,
    _pad2:           f32,
    viewport_offset: vec2<f32>,
    viewport_size:   vec2<f32>,
    camera_right:    vec3<f32>,
    tan_half_fov:    f32,
    camera_up:       vec3<f32>,
    aspect:          f32,
    camera_fwd:      vec3<f32>,
    _pad3:           f32,
    // Section view (offsets 112-143, 32 B)
    section_a_enabled: u32,
    section_a_axis:    u32,
    section_a_pos:     f32,
    section_a_flip:    u32,
    section_b_enabled: u32,
    section_b_axis:    u32,
    section_b_pos:     f32,
    section_b_flip:    u32,
    // Thickness overlay (offsets 144-159, 16 B)
    thickness_enabled: u32,
    thickness_min:     f32,
    thickness_max:     f32,
    overlay_invert:    u32,  // 1 = invert colour map (red=high, green=low) for stress
};

@group(0) @binding(0) var<uniform> uni: Uniforms;
@group(0) @binding(1) var sdf_tex:       texture_3d<f32>;
@group(0) @binding(2) var thickness_tex: texture_3d<f32>;

fn load_tex(tex: texture_3d<f32>, c: vec3<i32>, dim: vec3<i32>) -> f32 {
    let cc = clamp(c, vec3<i32>(0), dim - vec3<i32>(1));
    return textureLoad(tex, cc, 0).r;
}

fn sample_3d(tex: texture_3d<f32>, p: vec3<f32>) -> f32 {
    let dim3 = vec3<i32>(textureDimensions(tex));
    let dim  = vec3<f32>(dim3);
    let uvw  = (p - uni.bounds_min) / (uni.bounds_max - uni.bounds_min);
    let tc   = uvw * dim - 0.5;
    let i    = vec3<i32>(floor(tc));
    let f    = fract(tc);

    let v000 = load_tex(tex, i + vec3(0,0,0), dim3);
    let v100 = load_tex(tex, i + vec3(1,0,0), dim3);
    let v010 = load_tex(tex, i + vec3(0,1,0), dim3);
    let v110 = load_tex(tex, i + vec3(1,1,0), dim3);
    let v001 = load_tex(tex, i + vec3(0,0,1), dim3);
    let v101 = load_tex(tex, i + vec3(1,0,1), dim3);
    let v011 = load_tex(tex, i + vec3(0,1,1), dim3);
    let v111 = load_tex(tex, i + vec3(1,1,1), dim3);

    let x00 = mix(v000, v100, f.x);
    let x10 = mix(v010, v110, f.x);
    let x01 = mix(v001, v101, f.x);
    let x11 = mix(v011, v111, f.x);
    return mix(mix(x00, x10, f.y), mix(x01, x11, f.y), f.z);
}

fn sample_sdf(p: vec3<f32>) -> f32 {
    return sample_3d(sdf_tex, p);
}

fn calc_normal(p: vec3<f32>) -> vec3<f32> {
    let e = uni.voxel_size * 0.5;
    return normalize(vec3<f32>(
        sample_sdf(p + vec3(e, 0.0, 0.0)) - sample_sdf(p - vec3(e, 0.0, 0.0)),
        sample_sdf(p + vec3(0.0, e, 0.0)) - sample_sdf(p - vec3(0.0, e, 0.0)),
        sample_sdf(p + vec3(0.0, 0.0, e)) - sample_sdf(p - vec3(0.0, 0.0, e)),
    ));
}

// Returns the world-space coordinate for the given axis (0=X, 1=Y, 2=Z).
fn axis_coord(p: vec3<f32>, axis: u32) -> f32 {
    if axis == 0u { return p.x; }
    if axis == 1u { return p.y; }
    return p.z;
}

// Map a normalised thickness value [0..1] to a colour.
// 0 (thin/critical) → red, 0.5 → yellow, 1 (thick/OK) → green.
fn thickness_color(t: f32) -> vec3<f32> {
    let r = clamp(1.0 - t * 2.0 + 1.0, 0.0, 1.0) * clamp(2.0 - t * 2.0, 0.0, 1.0);
    let g = clamp(t * 2.0, 0.0, 1.0);
    let b = 0.0;
    // Simpler piecewise: red→yellow in [0,0.5], yellow→green in [0.5,1]
    if t < 0.5 {
        return vec3<f32>(1.0, t * 2.0, 0.0);
    } else {
        return vec3<f32>(1.0 - (t - 0.5) * 2.0, 1.0, 0.0);
    }
}

struct VOut { @builtin(position) pos: vec4<f32> };

@vertex
fn vs_main(@builtin(vertex_index) vid: u32) -> VOut {
    var xy = array<vec2<f32>, 3>(
        vec2<f32>(-1.0, -1.0),
        vec2<f32>( 3.0, -1.0),
        vec2<f32>(-1.0,  3.0),
    );
    return VOut(vec4<f32>(xy[vid], 0.0, 1.0));
}

@fragment
fn fs_main(@builtin(position) frag_coord: vec4<f32>) -> @location(0) vec4<f32> {
    // Map pixel to NDC in [-1, 1] for the 3D viewport sub-region.
    let local = frag_coord.xy - uni.viewport_offset;
    let ndc_x =        local.x / uni.viewport_size.x * 2.0 - 1.0;
    let ndc_y = 1.0 - local.y / uni.viewport_size.y * 2.0;

    // Build ray from camera basis vectors — avoids any projection-matrix depth convention.
    let ro = uni.camera_pos;
    let rd = normalize(
          uni.camera_fwd
        + uni.camera_right * (ndc_x * uni.aspect * uni.tan_half_fov)
        + uni.camera_up    * (ndc_y * uni.tan_half_fov)
    );

    // AABB entry/exit
    let inv_rd = 1.0 / rd;
    let t0 = (uni.bounds_min - ro) * inv_rd;
    let t1 = (uni.bounds_max - ro) * inv_rd;
    var tmin = max(max(min(t0.x, t1.x), min(t0.y, t1.y)), min(t0.z, t1.z));
    var tmax = min(min(max(t0.x, t1.x), max(t0.y, t1.y)), max(t0.z, t1.z));
    if tmax < 0.0 || tmin > tmax { return vec4<f32>(0.0, 0.0, 0.0, 0.0); }

    // Section view — clip the ray to the kept half-space BEFORE marching.
    // flip=0: kept side is coord >= pos.  flip=1: kept side is coord <= pos.
    //
    // Two cases per plane:
    //   (A) Ray starts on clipped side → advance tmin to the plane, then check
    //       if SDF is negative at the plane entry — if so, the cap face is here.
    //   (B) Ray moves from kept toward clipped → reduce tmax to the plane;
    //       after the march, check if SDF is negative there for the cap face.
    //
    // In case (A) we return the cap colour immediately (the cap is the first
    // visible surface from the clipped side).  In case (B) the cap is detected
    // in the !hit branch below.
    var t_cap        = 1e30f;  // tmax was clipped here; check SDF after march
    var march_from   = tmin;   // start of safe marching range (inside kept region)
    var march_anchor = max(tmin - uni.voxel_size * 0.1, 0.0); // known-outside point for bisection

    if uni.section_a_enabled != 0u {
        let ro_a   = axis_coord(ro, uni.section_a_axis);
        let rd_a   = axis_coord(rd, uni.section_a_axis);
        let flip_a = uni.section_a_flip != 0u;

        if abs(rd_a) > 1e-6 {
            let t_plane = (uni.section_a_pos - ro_a) / rd_a;
            // Ray moves toward clipped side when:
            //   flip=0: coord decreasing → rd_a < 0
            //   flip=1: coord increasing → rd_a > 0
            let toward_clipped = select(rd_a < 0.0, rd_a > 0.0, flip_a);
            if toward_clipped {
                // Case B: ray exits through the section plane.
                if t_plane < tmax { tmax = t_plane; t_cap = t_plane; }
            } else {
                // Case A: ray enters from clipped side; advance tmin.
                if t_plane > tmin {
                    tmin = t_plane;
                    march_from   = t_plane;
                    march_anchor = t_plane;  // SDF >= 0 here (checked below)
                    // If the SDF is clearly inside the object at the section plane
                    // (more than half a voxel deep), this is the cap face.
                    // Pixels near the surface boundary fall through to the march so
                    // the edge shows correct surface shading instead of cap bleed.
                    if sample_sdf(ro + rd * t_plane) < -uni.voxel_size * 0.5 {
                        return vec4<f32>(0.85, 0.72, 0.55, 1.0);
                    }
                }
            }
        } else {
            // Parallel ray: discard if entirely on the clipped side.
            let on_clipped = select(ro_a < uni.section_a_pos, ro_a > uni.section_a_pos, flip_a);
            if on_clipped { return vec4<f32>(0.0, 0.0, 0.0, 0.0); }
        }
    }

    if uni.section_b_enabled != 0u {
        let ro_b   = axis_coord(ro, uni.section_b_axis);
        let rd_b   = axis_coord(rd, uni.section_b_axis);
        let flip_b = uni.section_b_flip != 0u;

        if abs(rd_b) > 1e-6 {
            let t_plane = (uni.section_b_pos - ro_b) / rd_b;
            let toward_clipped = select(rd_b < 0.0, rd_b > 0.0, flip_b);
            if toward_clipped {
                if t_plane < tmax { tmax = t_plane; t_cap = min(t_cap, t_plane); }
            } else {
                if t_plane > tmin {
                    tmin = t_plane;
                    march_from   = t_plane;
                    march_anchor = t_plane;
                    if sample_sdf(ro + rd * t_plane) < -uni.voxel_size * 0.5 {
                        return vec4<f32>(0.85, 0.72, 0.55, 1.0);
                    }
                }
            }
        } else {
            let on_clipped = select(ro_b < uni.section_b_pos, ro_b > uni.section_b_pos, flip_b);
            if on_clipped { return vec4<f32>(0.0, 0.0, 0.0, 0.0); }
        }
    }

    if tmax < 0.0 || tmin > tmax { return vec4<f32>(0.0, 0.0, 0.0, 0.0); }

    let min_step = uni.voxel_size * 0.1;

    // Sphere trace. Start at march_from (which is inside the kept region and
    // has SDF >= 0). march_anchor is a confirmed-outside point for bisection.
    var t      = march_from;
    var t_prev = march_anchor;
    var hit    = false;
    for (var i = 0; i < 256; i++) {
        let d = sample_sdf(ro + rd * t);
        if d < 0.0 { hit = true; break; }
        t_prev = t;
        t += max(d, min_step);
        if t > tmax + min_step { break; }
    }

    if !hit {
        // Case B cap: the ray was stopped by a section plane and the SDF is
        // clearly inside the object there → shade the exposed cross-section face.
        if t_cap < 1e29 && sample_sdf(ro + rd * t_cap) < -uni.voxel_size * 0.5 {
            return vec4<f32>(0.85, 0.72, 0.55, 1.0);
        }
        return vec4<f32>(0.0, 0.0, 0.0, 0.0);
    }

    // Bisect between t_prev (outside, d≥0) and t (inside, d<0) for sharp edges.
    var t_lo = t_prev;
    var t_hi = t;
    for (var b = 0; b < 8; b++) {
        let t_mid = (t_lo + t_hi) * 0.5;
        if sample_sdf(ro + rd * t_mid) < 0.0 { t_hi = t_mid; } else { t_lo = t_mid; }
    }
    t = t_hi;

    let p = ro + rd * t;
    let n = normalize(calc_normal(p));

    // Base surface colour — may be overridden by thickness overlay.
    var base = vec3<f32>(0.75, 0.78, 0.82);

    // Thickness / stress overlay: sample the texture and apply a colour map.
    if uni.thickness_enabled != 0u {
        let th = sample_3d(thickness_tex, p);
        if th > 0.001 {
            let range = uni.thickness_max - uni.thickness_min;
            var t_norm = clamp((th - uni.thickness_min) / max(range, 0.001), 0.0, 1.0);
            if uni.overlay_invert != 0u { t_norm = 1.0 - t_norm; }
            base = thickness_color(t_norm);
        }
    }

    let key  = max(dot(n, normalize(vec3<f32>( 0.5, -0.3,  0.8))), 0.0);
    let fill = max(dot(n, normalize(vec3<f32>(-0.3,  0.5,  0.6))), 0.0) * 0.3;
    let rim  = max(dot(n, normalize(vec3<f32>( 0.0, -0.3,  0.8))), 0.0) * 0.2;
    let lit  = 0.35 + 0.65 * (key + fill + rim);

    return vec4<f32>(base * lit, 1.0);
}
