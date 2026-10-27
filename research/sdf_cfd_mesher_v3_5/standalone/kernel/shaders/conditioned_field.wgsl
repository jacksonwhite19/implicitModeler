struct ConditionedTile {
    origin_spacing:vec4<f32>, guard_min:vec4<f32>, guard_max:vec4<f32>, dimensions_offset:vec4<u32>,
}
struct ConditionedConfig {
    domain_min_block_size:vec4<f32>, domain_max:vec4<f32>, counts:vec4<u32>, bands:vec4<f32>,
}
@group(0) @binding(5) var<storage,read> conditioned_tiles:array<ConditionedTile>;
@group(0) @binding(6) var<storage,read> conditioned_values:array<f32>;
@group(0) @binding(7) var<uniform> conditioned_config:ConditionedConfig;
fn conditioned_lerp(a:f32,b:f32,t:f32)->f32 { return a+(b-a)*t; }
fn conditioned_value(tile:ConditionedTile,i:vec3<u32>)->f32 {
    return conditioned_values[tile.dimensions_offset.w+(i.x*(tile.dimensions_offset.y+1u)+i.y)*(tile.dimensions_offset.z+1u)+i.z];
}
// x=sample, y=presence. Missing data selects the existing canonical fallback.
fn conditioned_tile_sample(id:u32,p:vec3<f32>)->vec2<f32> {
    let tile=conditioned_tiles[id];let dims=tile.dimensions_offset.xyz;
    let origin=tile.origin_spacing.xyz;let spacing=tile.origin_spacing.w;
    if any(p<tile.guard_min.xyz)||any(p>tile.guard_max.xyz) {return vec2<f32>(0.0);}
    if tile.guard_min.w>0.0 && (any(p<origin)||any(p>origin+vec3<f32>(dims)*spacing)) {return vec2<f32>(0.0);}
    let f=clamp((p-origin)/spacing,vec3<f32>(0.0),vec3<f32>(dims));
    let a=vec3<u32>(floor(f));let b=min(a+vec3<u32>(1u),dims);let t=select(f-vec3<f32>(a),vec3<f32>(0.0),a==b);
    let c00=conditioned_lerp(conditioned_value(tile,vec3<u32>(a.x,a.y,a.z)),conditioned_value(tile,vec3<u32>(b.x,a.y,a.z)),t.x);
    let c10=conditioned_lerp(conditioned_value(tile,vec3<u32>(a.x,b.y,a.z)),conditioned_value(tile,vec3<u32>(b.x,b.y,a.z)),t.x);
    let c01=conditioned_lerp(conditioned_value(tile,vec3<u32>(a.x,a.y,b.z)),conditioned_value(tile,vec3<u32>(b.x,a.y,b.z)),t.x);
    let c11=conditioned_lerp(conditioned_value(tile,vec3<u32>(a.x,b.y,b.z)),conditioned_value(tile,vec3<u32>(b.x,b.y,b.z)),t.x);
    return vec2<f32>(conditioned_lerp(conditioned_lerp(c00,c10,t.y),conditioned_lerp(c01,c11,t.y),t.z),1.0);
}
fn conditioned_sign_mismatch(a:f32,b:f32)->bool {return (a<0.0&&b>0.0)||(a>0.0&&b<0.0);}
fn sdf_eval(p:vec3<f32>)->f32 {
    let canonical=canonical_eval(p);
    if any(p<conditioned_config.domain_min_block_size.xyz)||any(p>conditioned_config.domain_max.xyz) {return canonical;}
    let counts=conditioned_config.counts.xyz;
    let b=vec3<u32>(clamp(floor((p-conditioned_config.domain_min_block_size.xyz)/conditioned_config.domain_min_block_size.w),vec3<f32>(0.0),vec3<f32>(counts-vec3<u32>(1u))));
    let block=conditioned_tile_sample((b.x*counts.y+b.y)*counts.z+b.z,p);
    var chosen=block;var best_spacing=3.402823e38;
    let block_count=counts.x*counts.y*counts.z;
    for(var i=0u;i<conditioned_config.counts.w;i+=1u) {
        let id=block_count+i;let s=conditioned_tile_sample(id,p);let spacing=conditioned_tiles[id].origin_spacing.w;
        if s.y>0.0 && spacing<best_spacing { chosen=s;best_spacing=spacing; }
    }
    if chosen.y==0.0 {return canonical;}
    let bands=conditioned_config.bands;
    if block.y>0.0 && (abs(chosen.x)<=bands.w||abs(block.x)<=bands.w)&&conditioned_sign_mismatch(chosen.x,block.x) {chosen=block;}
    let conditioned=chosen.x;
    if abs(canonical)<=bands.x||abs(conditioned)<=bands.y||conditioned_sign_mismatch(conditioned,canonical) {return canonical;}
    if abs(canonical)<=bands.x+bands.z {
        let t=clamp((abs(canonical)-bands.x)/bands.z,0.0,1.0);
        return conditioned_lerp(canonical,conditioned,t*t*(3.0-2.0*t));
    }
    return conditioned;
}
