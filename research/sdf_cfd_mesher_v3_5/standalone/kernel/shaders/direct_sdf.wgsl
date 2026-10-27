// Viewport-only conservative tracer. Field functions are prepended from SdfIr.
struct Uniforms {
    eye:vec4<f32>, lo:vec4<f32>, hi:vec4<f32>, right:vec4<f32>, up:vec4<f32>, forward:vec4<f32>,
    viewport:vec4<f32>, section_a:vec4<f32>, section_b:vec4<f32>,
}
@group(0) @binding(0) var<uniform> u:Uniforms;
fn plane_value(p:vec3<f32>,s:vec4<f32>)->f32 {
    return select(s.z-p[u32(s.y)],p[u32(s.y)]-s.z,s.w>0.5);
}
fn field(p:vec3<f32>)->f32 {
    var d=sdf_eval(p);
    if u.section_a.x>0.5 {d=max(d,plane_value(p,u.section_a));}
    if u.section_b.x>0.5 {d=max(d,plane_value(p,u.section_b));}
    return d;
}
fn ray(pixel:vec2<f32>)->vec3<f32> {
    let ndc=(pixel-u.viewport.xy)/u.viewport.zw*2.0-1.0;
    return normalize(u.forward.xyz+u.right.xyz*(ndc.x*u.right.w*u.up.w)-u.up.xyz*(ndc.y*u.up.w));
}
// (hit distance, scalar residual, iterations, status: 0 miss, 1 hit, 2 budget exhausted)
fn trace(rd:vec3<f32>)->vec4<f32> {
    var entry=0.0; var exit=1e30;
    for(var axis=0u;axis<3u;axis++) {
        if abs(rd[axis])<1e-10 {
            if u.eye[axis]<u.lo[axis] || u.eye[axis]>u.hi[axis] {return vec4<f32>(0.0);}
        } else {
            let a=(u.lo[axis]-u.eye[axis])/rd[axis];
            let b=(u.hi[axis]-u.eye[axis])/rd[axis];
            entry=max(entry,min(a,b)); exit=min(exit,max(a,b));
        }
    }
    if entry>exit {return vec4<f32>(0.0);}
    var t=entry;
    for(var i=0u;i<4096u;i++) {
        let d=field(u.eye.xyz+rd*t);
        if abs(d)<=u.eye.w {return vec4<f32>(t,d,f32(i),1.0);}
        t+=abs(d)*0.8;
        if t>exit {return vec4<f32>(0.0);}
    }
    return vec4<f32>(t,field(u.eye.xyz+rd*t),4096.0,2.0);
}
@vertex fn vs_main(@builtin(vertex_index) id:u32)->@builtin(position) vec4<f32> {
    var xy=array<vec2<f32>,3>(vec2<f32>(-1.0,-1.0),vec2<f32>(3.0,-1.0),vec2<f32>(-1.0,3.0));
    return vec4<f32>(xy[id],0.0,1.0);
}
@fragment fn fs_probe(@builtin(position) pixel:vec4<f32>)->@location(0) vec4<f32> {return trace(ray(pixel.xy));}
@fragment fn fs_main(@builtin(position) pixel:vec4<f32>)->@location(0) vec4<f32> {
    let rd=ray(pixel.xy);let hit=trace(rd);
    if hit.w==0.0 {return vec4<f32>(0.0);}
    // Visible failure marker, never silently hide an unresolved ray.
    if hit.w==2.0 {return vec4<f32>(1.0,0.0,1.0,1.0);}
    let p=u.eye.xyz+rd*hit.x;let e=u.eye.w*2.0;
    let g=vec3<f32>(field(p+vec3<f32>(e,0.0,0.0))-field(p-vec3<f32>(e,0.0,0.0)),field(p+vec3<f32>(0.0,e,0.0))-field(p-vec3<f32>(0.0,e,0.0)),field(p+vec3<f32>(0.0,0.0,e))-field(p-vec3<f32>(0.0,0.0,e)));
    let n=g/max(length(g),1e-20);
    var base=vec3<f32>(0.75,0.78,0.82);
    if (u.section_a.x>0.5 && abs(plane_value(p,u.section_a))<e) || (u.section_b.x>0.5 && abs(plane_value(p,u.section_b))<e) {base=vec3<f32>(0.85,0.72,0.55);}
    let key=max(dot(n,normalize(vec3<f32>(0.5,-0.3,0.8))),0.0);
    let fill=max(dot(n,normalize(vec3<f32>(-0.3,0.5,0.6))),0.0)*0.3;
    let rim=max(dot(n,normalize(vec3<f32>(0.0,-0.3,0.8))),0.0)*0.2;
    return vec4<f32>(base*(0.35+0.65*(key+fill+rim)),1.0);
}
