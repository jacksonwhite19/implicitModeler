//! FFI only: calls the production NacaSpline, with coefficients exported by
//! the live Rhai constructor. No independent shape or distance implementation.
mod cached_spline;
use cached_spline::{NacaSpline,SectionBranch,SpanCache,QueryStats,QUERY_BUDGET};
use glam::DVec2;
use std::sync::OnceLock;
static SECTION:OnceLock<NacaSpline>=OnceLock::new();
static CACHE:OnceLock<SpanCache>=OnceLock::new();
#[unsafe(no_mangle)] pub unsafe extern "C" fn spline_load(data:*const u8,n:usize)->i32 {
    let Ok(s)=serde_json::from_slice::<NacaSpline>(unsafe{std::slice::from_raw_parts(data,n)}) else{return 1};
    if s.spans.is_empty(){return 2}let cache=SpanCache::new(&s);if SECTION.set(s).is_err(){return 3}if CACHE.set(cache).is_err(){return 4}0
}
#[unsafe(no_mangle)] pub unsafe extern "C" fn spline_query_mode(x:f64,z:f64,fast:i32,out:*mut f64,diag:*mut u64,mask:*mut u8)->i32 {
    let Some(s)=SECTION.get() else{return 1};let mut forced_cache;let cache=if fast==2{forced_cache=CACHE.get().unwrap().clone();forced_cache.valid=false;&forced_cache}else{CACHE.get().unwrap()};let mut st=QueryStats::new(s.spans.len());let Ok(q)=s.closest_cached(DVec2::new(x,z),QUERY_BUDGET,false,cache,fast!=0,&mut st) else{return 2};
    if !diag.is_null(){let values=[s.spans.len()as u64,st.exact_spans as u64,st.rejected as u64,st.refined_roots as u64,st.winner as u64,st.nodes as u64,st.fallback as u64];unsafe{std::ptr::copy_nonoverlapping(values.as_ptr(),diag,7);}}
    if !mask.is_null(){unsafe{std::ptr::copy_nonoverlapping(st.retained.as_ptr(),mask,st.retained.len());}}
    let u=if q.branch==SectionBranch::Upper{-q.parameter}else{q.parameter};let a=(u+1.0)*0.5*s.spans.len() as f64;let i=(a.floor() as usize).min(s.spans.len()-1);let t=a-i as f64;
    let span=&s.spans[i];let p=span.control.map(DVec2::from_array);let (_,d)=span.evaluate(t);let dd=(p[2]-p[1]*2.0+p[0])*(6.0*(1.0-t))+(p[3]-p[2]*2.0+p[1])*(6.0*t);
    let curvature=(d.x*dd.y-d.y*dd.x)/d.length().powi(3);
    let values=[q.point[0],q.point[1],q.outward_normal[0],q.outward_normal[1],u,curvature,q.signed_distance,q.distance_error_bound];
    unsafe{std::ptr::copy_nonoverlapping(values.as_ptr(),out,values.len());}0
}

#[unsafe(no_mangle)] pub unsafe extern "C" fn spline_query(x:f64,z:f64,out:*mut f64)->i32{unsafe{spline_query_mode(x,z,1,out,std::ptr::null_mut(),std::ptr::null_mut())}}
