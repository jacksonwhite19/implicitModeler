//! FFI only: calls the production NacaSpline, with coefficients exported by
//! the live Rhai constructor. No independent shape or distance implementation.
use implicit_cad::sdf::aerospace::naca_spline::{NacaSpline,SectionBranch};
use glam::DVec2;
use std::sync::OnceLock;
static SECTION:OnceLock<NacaSpline>=OnceLock::new();
#[unsafe(no_mangle)] pub unsafe extern "C" fn spline_load(data:*const u8,n:usize)->i32 {
    let Ok(s)=serde_json::from_slice::<NacaSpline>(unsafe{std::slice::from_raw_parts(data,n)}) else{return 1};
    if s.spans.is_empty(){return 2}if SECTION.set(s).is_err(){return 3}0
}
#[unsafe(no_mangle)] pub unsafe extern "C" fn spline_query(x:f64,z:f64,out:*mut f64)->i32 {
    let Some(s)=SECTION.get() else{return 1};let Ok(q)=s.closest_skin_point(DVec2::new(x,z)) else{return 2};
    let u=if q.branch==SectionBranch::Upper{-q.parameter}else{q.parameter};let a=(u+1.0)*0.5*s.spans.len() as f64;let i=(a.floor() as usize).min(s.spans.len()-1);let t=a-i as f64;
    let span=&s.spans[i];let p=span.control.map(DVec2::from_array);let (_,d)=span.evaluate(t);let dd=(p[2]-p[1]*2.0+p[0])*(6.0*(1.0-t))+(p[3]-p[2]*2.0+p[1])*(6.0*t);
    let curvature=(d.x*dd.y-d.y*dd.x)/d.length().powi(3);
    let values=[q.point[0],q.point[1],q.outward_normal[0],q.outward_normal[1],u,curvature,q.signed_distance,q.distance_error_bound];
    unsafe{std::ptr::copy_nonoverlapping(values.as_ptr(),out,values.len());}0
}
