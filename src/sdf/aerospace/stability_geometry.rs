use std::sync::Arc;
use glam::Vec3;
use crate::sdf::Sdf;
use crate::sdf::query::bounding_points;

pub struct TailVolumeCoefficients {
    pub horizontal_vt:         f32,
    pub vertical_vt:           f32,
    pub horizontal_tail_area:  f32,
    pub vertical_tail_area:    f32,
    pub horizontal_moment_arm: f32,
    pub vertical_moment_arm:   f32,
    pub wing_area:             f32,
    pub wing_mac:              f32,
    pub wing_span:             f32,
}

pub struct TailSizingRecommendation {
    pub horizontal_adequate: bool,
    pub vertical_adequate:   bool,
    pub horizontal_message:  String,
    pub vertical_message:    String,
}

/// Compute planform area of an SDF by scanning its XZ footprint (span along Y axis).
pub fn planform_area(sdf: &Arc<dyn Sdf>) -> f32 {
    let bi = bounding_points(&**sdf);
    const N: usize = 40;
    let step = 1.0_f32;
    let mut area = 0.0_f32;
    let h = (bi.max.y - bi.min.y) / N as f32;
    let chords: Vec<f32> = (0..=N).map(|i| {
        let y = bi.min.y + h * i as f32;
        let mut xmax = bi.min.x;
        let mut xmin = bi.max.x;
        let mut found = false;
        let mut x = bi.min.x;
        while x <= bi.max.x {
            if sdf.distance(Vec3::new(x, y, 0.0)) < 0.0 {
                xmax = xmax.max(x);
                xmin = xmin.min(x);
                found = true;
            }
            x += step;
        }
        if found { (xmax - xmin).max(0.0) } else { 0.0 }
    }).collect();
    for i in 0..N {
        area += (chords[i] + chords[i + 1]) * 0.5 * h;
    }
    area
}

/// Mean aerodynamic chord
pub fn mean_aero_chord(sdf: &Arc<dyn Sdf>) -> f32 {
    let bi = bounding_points(&**sdf);
    const N: usize = 40;
    let step = 1.0_f32;
    let y_min = bi.min.y;
    let y_max = bi.max.y;
    let chords: Vec<f32> = (0..=N).map(|i| {
        let y = y_min + (y_max - y_min) * i as f32 / N as f32;
        let mut xmax = bi.min.x;
        let mut xmin = bi.max.x;
        let mut found = false;
        let mut x = bi.min.x;
        while x <= bi.max.x {
            if sdf.distance(Vec3::new(x, y, 0.0)) < 0.0 {
                xmax = xmax.max(x);
                xmin = xmin.min(x);
                found = true;
            }
            x += step;
        }
        if found { (xmax - xmin).max(0.0) } else { 0.0 }
    }).collect();
    let h = (y_max - y_min) / N as f32;
    let num: f32 = (0..N).map(|i| { let c = (chords[i] + chords[i + 1]) * 0.5; c * c * h }).sum();
    let den: f32 = (0..N).map(|i| (chords[i] + chords[i + 1]) * 0.5 * h).sum();
    if den > 0.0 { num / den } else { 0.0 }
}

pub fn compute_tail_volumes(
    wing:     &Arc<dyn Sdf>,
    h_tail:   &Arc<dyn Sdf>,
    v_tail:   &Arc<dyn Sdf>,
    _fuselage: &Arc<dyn Sdf>,
) -> (TailVolumeCoefficients, TailSizingRecommendation) {
    let wing_bi   = bounding_points(&**wing);
    let h_tail_bi = bounding_points(&**h_tail);
    let v_tail_bi = bounding_points(&**v_tail);

    let s_w   = planform_area(wing);
    let mac_w = mean_aero_chord(wing);
    let b_w   = wing_bi.size.y;

    let s_ht   = planform_area(h_tail);
    let mac_ht = mean_aero_chord(h_tail);

    let s_vt   = planform_area(v_tail);
    let mac_vt = mean_aero_chord(v_tail);

    // Moment arms: distance from wing MAC quarter-chord to tail AC
    let wing_qc_x   = wing_bi.min.x + mac_w * 0.25;
    let h_tail_ac_x = h_tail_bi.min.x + mac_ht * 0.25;
    let l_ht        = (h_tail_ac_x - wing_qc_x).abs();

    let v_tail_ac_x = v_tail_bi.min.x + mac_vt * 0.25;
    let l_vt        = (v_tail_ac_x - wing_qc_x).abs();

    let vht = if s_w > 0.0 && mac_w > 0.0 { (s_ht * l_ht) / (s_w * mac_w) } else { 0.0 };
    let vvt = if s_w > 0.0 && b_w > 0.0  { (s_vt * l_vt) / (s_w * b_w) }   else { 0.0 };

    let h_adequate = vht >= 0.30;
    let v_adequate = vvt >= 0.02;

    let h_msg = if vht >= 0.40 {
        format!("VHT={:.3} — excellent horizontal stability", vht)
    } else if vht >= 0.30 {
        format!("VHT={:.3} — adequate horizontal stability", vht)
    } else if vht >= 0.24 {
        format!("VHT={:.3} — marginal, consider increasing H-tail area or moment arm", vht)
    } else {
        format!("VHT={:.3} — inadequate, increase H-tail area to {:.0}mm²",
            vht, 0.30 * s_w * mac_w / l_ht.max(1.0))
    };

    let v_msg = if vvt >= 0.04 {
        format!("VVT={:.3} — excellent directional stability", vvt)
    } else if vvt >= 0.02 {
        format!("VVT={:.3} — adequate directional stability", vvt)
    } else if vvt >= 0.016 {
        format!("VVT={:.3} — marginal, consider increasing V-tail", vvt)
    } else {
        format!("VVT={:.3} — inadequate, increase V-tail area to {:.0}mm²",
            vvt, 0.02 * s_w * b_w / l_vt.max(1.0))
    };

    let coefs = TailVolumeCoefficients {
        horizontal_vt:         vht,
        vertical_vt:           vvt,
        horizontal_tail_area:  s_ht,
        vertical_tail_area:    s_vt,
        horizontal_moment_arm: l_ht,
        vertical_moment_arm:   l_vt,
        wing_area:             s_w,
        wing_mac:              mac_w,
        wing_span:             b_w,
    };
    let rec = TailSizingRecommendation {
        horizontal_adequate: h_adequate,
        vertical_adequate:   v_adequate,
        horizontal_message:  h_msg,
        vertical_message:    v_msg,
    };
    (coefs, rec)
}
