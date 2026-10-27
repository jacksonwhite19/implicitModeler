//! Pure grid arithmetic shared by native preflight and extraction. No scalar calls.
use glam::Vec3;
use serde_json::{Value, json};
use crate::MAX_LINE;

pub fn plan(low: Vec3, high: Vec3, scale: f64, padding_m: f64,
            spacing: Option<f64>, cells: Option<u32>, budget: u64, timeout: f64) -> std::result::Result<Value, String> {
    if !scale.is_finite() || scale <= 0. || !padding_m.is_finite() || padding_m <= 0.
        || budget == 0 || !timeout.is_finite() || timeout <= 0. || spacing.is_some() == cells.is_some()
        || spacing.is_some_and(|x| !x.is_finite() || x <= 0.) || cells == Some(0) {
        return Err("Exactly one positive spacing_m or cells_across, positive explicit resources and SI scale required".into());
    }
    let extent = high-low;
    if !low.is_finite() || !high.is_finite() || !extent.is_finite() || extent.min_element() <= 0. {
        return Err("Finite nondegenerate canonical binary32 padded support box required".into());
    }
    let lengths=extent.to_array().map(|v| f64::from(v)*scale);
    let longest=f64::from(extent.max_element())*scale;
    let effective=spacing.unwrap_or_else(|| longest/f64::from(cells.unwrap()));
    if !longest.is_finite() || !lengths.iter().all(|v| v.is_finite() && *v>0.) || !effective.is_finite() || effective<=0. {
        return Err("SI bounds or derived spacing overflow/underflow".into());
    }
    let requested=(longest/effective).ceil().max(8.);
    let resolution=if requested.is_finite() && requested < f64::from(u32::MAX) {Some(requested as u32)} else {None};
    let points=resolution.and_then(|r| u64::from(r).checked_add(1)).and_then(|n| n.checked_pow(3));
    let cubes=resolution.and_then(|r| u64::from(r).checked_pow(3));
    // Existing extractor uses u32 grid/cube products and u32 vertex indices.
    let edges=resolution.and_then(|r| u64::from(r).checked_add(1).and_then(|n| n.checked_mul(n)).and_then(|n| n.checked_mul(u64::from(r))).and_then(|n| n.checked_mul(3)));
    let triangles=cubes.and_then(|n| n.checked_mul(5));
    let grid_bytes=points.and_then(|n| n.checked_mul(std::mem::size_of::<f32>() as u64));
    let arithmetic_safe=points.is_some_and(|n| n<=u64::from(u32::MAX) && usize::try_from(n).is_ok())
        && cubes.is_some_and(|n| n<=u64::from(u32::MAX))
        && edges.is_some_and(|n| n<=u64::from(u32::MAX));
    let reason=if !arithmetic_safe {"ExtractorIndexArithmeticLimit"}
        else if points.unwrap()>budget {"QueryBudgetExceeded"} else {"ADMITTED_CONSTRUCTION_ONLY"};
    let grid_step=resolution.map(|r| (extent/r as f32).to_array().map(|v| f64::from(v)*scale));
    let steps_safe=grid_step.is_some_and(|s| s.iter().all(|v| v.is_finite() && *v>0.));
    let reason=if arithmetic_safe && !steps_safe {"CoordinateOutOfKernelRange"} else {reason};
    // Deliberately loose output estimate, not an exact allocation/serialized-size guarantee.
    let output_estimate=edges.and_then(|n| n.checked_mul(128)).and_then(|n| triangles.and_then(|t|t.checked_mul(40)).and_then(|t|n.checked_add(t))).and_then(|n|n.checked_add(16384));
    Ok(json!({"schema":"V4SeedAdmission/1", "metres_per_model_unit":scale,
        "bounds_role":"Conservative canonical support metadata plus requested padding; not tight geometric AABB",
        "bounds_m_realized":[low.to_array().map(|v| f64::from(v)*scale),high.to_array().map(|v| f64::from(v)*scale)],
        "axis_lengths_m":lengths,"longest_padded_extent_m":longest,"padding_m_requested":padding_m,
        "spacing_m_requested":spacing,"cells_across_requested":cells,"spacing_m_effective":effective,
        "construction_floor_cells":8,"resolution":resolution,"grid_step_m":grid_step,"predicted_grid_samples":points,
        "maximum_queries":budget,"timeout_seconds":timeout,"protocol_limit_bytes":MAX_LINE,
        "grid_scalar_storage_bytes":grid_bytes,"grid_scalar_storage_scope":"Only f32 scalar Vec; excludes vertices, indices, edge HashMap/cache, source, JSON and transport copies. Not peak memory.",
        "maximum_grid_edges":edges,"maximum_lookup_triangles":triangles,
        "loose_output_estimate_bytes":output_estimate,"protocol_risk":output_estimate.is_none_or(|n|n>MAX_LINE as u64),
        "protocol_risk_scope":"Worst-case grid edge/triangle counts with loose JSON byte allowances; actual response remains capped. Risk is not measured output or automatic refusal.",
        "work_estimate":"Predicted scalar samples only; no calibrated runtime estimate. Existing owner deadlines remain authoritative.",
        "admitted":reason=="ADMITTED_CONSTRUCTION_ONLY","reason":reason,"scalar_grid_attempts":0,"scalar_field_evaluations":0,
        "geometry_acceptance":false}))
}

#[cfg(test)]
mod tests {
    use super::*;
    fn p(scale:f64, spacing:Option<f64>, cells:Option<u32>, budget:u64)->Value {
        plan(Vec3::ZERO,Vec3::new(10.,2.,1.),scale,scale*0.1,spacing,cells,budget,30.).unwrap()
    }
    #[test] fn relative_dimensions_and_long_thin_steps() {
        let a=p(0.001,None,Some(96),1_000_000);let b=p(1.,None,Some(96),1_000_000);
        assert_eq!(a["resolution"],b["resolution"]);assert_eq!(a["predicted_grid_samples"],b["predicted_grid_samples"]);
        assert!(a["grid_step_m"][0].as_f64().unwrap()>a["grid_step_m"][1].as_f64().unwrap());
        assert_ne!(p(0.001,Some(0.001),None,1_000_000)["resolution"],p(1.,Some(0.001),None,1_000_000)["resolution"]);
    }
    #[test] fn exact_budget_and_above_old_limit() {
        for r in [8_u32,95,96,97,146] {
            let n=u64::from(r+1).pow(3);
            let a=p(1.,None,Some(r),n); assert_eq!(a["resolution"],r);assert_eq!(a["predicted_grid_samples"],n);assert_eq!(a["admitted"],true);
            let b=p(1.,None,Some(r),n-1);assert_eq!(b["admitted"],false);assert_eq!(b["reason"],"QueryBudgetExceeded");assert_eq!(b["scalar_field_evaluations"],0);
        }
    }
    #[test] fn invalid_settings_and_checked_extractor_products() {
        for (s,c) in [(None,None),(Some(1.),Some(10)),(Some(f64::NAN),None),(Some(0.),None),(None,Some(0))] {
            assert!(plan(Vec3::ZERO,Vec3::ONE,1.,0.1,s,c,1000,30.).is_err());
        }
        for r in [2000,u32::MAX] {let a=p(1.,None,Some(r),u64::MAX);assert_eq!(a["admitted"],false);assert_eq!(a["reason"],"ExtractorIndexArithmeticLimit");}
        assert!(p(1.,Some(1e-300),None,u64::MAX)["resolution"].is_null());
        assert_eq!(p(1.,None,Some(1),1000)["resolution"],8);
    }
}
