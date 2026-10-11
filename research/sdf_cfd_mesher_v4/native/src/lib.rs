//! Read-only batch ABI around qualified V3 section mathematics.
//! Uniform extrusion is the production wing.rs composition in binary64.
#[path = "../frozen/cached_spline.rs"]
mod cached_spline;
use cached_spline::{NacaSpline,SpanCache,QueryStats,SectionBranch,QUERY_BUDGET};
use glam::DVec2;
use std::sync::{OnceLock,Mutex};
use std::collections::HashMap;
struct Geometry { section:NacaSpline, cache:SpanCache, lower:f64, upper:f64 }
static GEOMETRY:OnceLock<Geometry>=OnceLock::new();
static MEMO:OnceLock<Mutex<HashMap<(u64,u64),[f64;8]>>>=OnceLock::new();

#[unsafe(no_mangle)]
pub unsafe extern "C" fn v4_load(data:*const u8,n:usize,lower:f64,upper:f64)->i32 {
    let Ok(section)=serde_json::from_slice::<NacaSpline>(unsafe{std::slice::from_raw_parts(data,n)}) else{return 1};
    if section.spans.is_empty() || !lower.is_finite() || !upper.is_finite() || lower>=upper{return 2}
    let cache=SpanCache::new(&section);
    if GEOMETRY.set(Geometry{section,cache,lower,upper}).is_err(){return 3}
    MEMO.get_or_init(||Mutex::new(HashMap::new()));0
}
#[unsafe(no_mangle)]
pub extern "C" fn v4_clear_memo(){if let Some(m)=MEMO.get(){m.lock().unwrap().clear();}}

fn section_query(g:&Geometry,x:f64,z:f64)->Result<[f64;8],i32>{
    let p=DVec2::new(x,z);
    let q=g.section.closest_cached(p,QUERY_BUDGET,true,&g.cache,true,&mut QueryStats::new(g.section.spans.len())).map_err(|_|2)?;
    // Exact distance derivative away from the boundary; the frozen source
    // normal is the one-sided selection inside its own distance error bound.
    let ambiguous=q.signed_distance.abs()<=q.distance_error_bound;
    let gradient=if ambiguous{DVec2::from_array(q.outward_normal)}else{(p-DVec2::from_array(q.point))/q.signed_distance};
    let branch=if q.branch==SectionBranch::TrailingFace{2.0}else if q.branch==SectionBranch::Upper{0.0}else{1.0};
    let curvature=if q.branch==SectionBranch::TrailingFace{0.0}else{
        // Identical existing V3 bridge curvature expression.
        let u=if q.branch==SectionBranch::Upper{-q.parameter}else{q.parameter};
        let a=(u+1.0)*0.5*g.section.spans.len() as f64;let i=(a.floor() as usize).min(g.section.spans.len()-1);let t=a-i as f64;
        let span=&g.section.spans[i];let p=span.control.map(DVec2::from_array);let (_,d)=span.evaluate(t);let dd=(p[2]-p[1]*2.0+p[0])*(6.0*(1.0-t))+(p[3]-p[2]*2.0+p[1])*(6.0*t);
        (d.x*dd.y-d.y*dd.x)/d.length().powi(3)
    };
    Ok([q.signed_distance,gradient.x,gradient.y,q.distance_error_bound,curvature,branch,ambiguous as u8 as f64,q.visited_nodes as f64])
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn v4_query(points:*const f64,n:usize,out:*mut f64)->i32{
    let Some(g)=GEOMETRY.get()else{return 1};
    let points=unsafe{std::slice::from_raw_parts(points,n*3)};
    let output=unsafe{std::slice::from_raw_parts_mut(out,n*10)};
    let mut memo=MEMO.get().unwrap().lock().unwrap();
    for (i,p) in points.chunks_exact(3).enumerate(){
        if !p.iter().all(|x|x.is_finite()){return 4}
        let key=(p[0].to_bits(),p[2].to_bits());
        let q=if let Some(v)=memo.get(&key){*v}else{let v=match section_query(g,p[0],p[2]){Ok(v)=>v,Err(c)=>return c};memo.insert(key,v);v};
        let ds=(g.lower-p[1]).max(p[1]-g.upper);
        let span_gradient=if g.lower-p[1]>=p[1]-g.upper{-1.0}else{1.0};
        let d2=q[0];let outside=DVec2::new(d2.max(0.0),ds.max(0.0)).length();
        let inside=d2.max(ds).min(0.0);let phi=outside+inside;
        let (a,b)=if outside>0.0{(d2.max(0.0)/outside,ds.max(0.0)/outside)}else if d2>=ds{(1.0,0.0)}else{(0.0,1.0)};
        let values=[phi,a*q[1],b*span_gradient,a*q[2],q[3],q[4],q[5],q[6],(d2==ds) as u8 as f64,q[7]];
        output[i*10..i*10+10].copy_from_slice(&values);
    }0
}
