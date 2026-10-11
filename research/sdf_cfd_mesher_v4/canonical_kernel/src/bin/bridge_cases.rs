//! Caller-owned integration cases. Constructors and parameters live here, never in meshing.
use glam::Vec3;
use implicit_cad::{
    scripting,
    sdf::{
        Sdf,
        aerospace::{Airfoil, LoftedWing},
        primitives::TaperedCapsule,
    },
};
use serde_json::{Value, json};
use std::{sync::Arc, time::Instant};
use v4_canonical_kernel::{Registry, SOURCE_SHA256};

fn capsule() -> (Arc<dyn Sdf>, Value) {
    let a = Vec3::new(0., 0., -0.04);
    let b = Vec3::new(0., 0., 0.04);
    let r = 0.04_f32;
    (
        Arc::new(TaperedCapsule::new(a, b, r, r)),
        json!({"kind":"rust_constructor",
        "symbol":"implicit_cad::sdf::primitives::TaperedCapsule::new",
        "a_f32":a.to_array(),"b_f32":b.to_array(),"radius_a_f32":r,"radius_b_f32":r,
        "historical_definition":"radius=0.04 m, half_length=0.04 m",
        "authority_scope":"binary32 kernel test object; not historical binary64 capsule identity"}),
    )
}

fn points() -> Vec<[f64; 3]> {
    let mut p = vec![
        [0., 0., 0.],
        [0.04, 0., 0.],
        [0., 0., 0.08],
        [0., 0., -0.08],
        [0.02, 0.01, 0.03],
        [0.05, 0.01, 0.06],
        [-0.04, 0., 0.],
        [0., 0., 0.12],
        [2., 2., 2.],
        [-2., -2., -2.],
        [0.6096, 0., 0.],
        [1.2192, 0., 0.],
        [1.2192, 0., 0.00127],
        [1.2192, 0., -0.00127],
        [0.374083984375, 0., 0.07315968322753906],
        [0.374083984375, 0., -0.07315968322753906],
        [0.6096, 0.841248, 0.],
        [0.6096, -0.841248, 0.],
        [1.2192, 0.841248, 0.00127],
        [0., 0.841248, 0.],
        [0., -0.841248, 0.],
        [-0.0005, 0., 0.],
        [1.2197, 0., 0.],
    ];
    for x in [-1., 0., 1.] {
        for y in [-1., 0., 1.] {
            for z in [-1., 0., 1.] {
                p.push([x, y, z]);
            }
        }
    }
    p
}

fn stored_inputs(sdf: &dyn Sdf) -> Value {
    if let Some(w) = sdf.as_any().downcast_ref::<LoftedWing>() {
        return json!({"sections":w.sections.iter().map(|s| {
            let a=s.section.as_any().downcast_ref::<Airfoil>().unwrap();
            json!({"position_f32":s.position.to_array(),"position_bits":s.position.to_array().map(f32::to_bits),
                "twist_f32_radians":s.twist,"chord_f32":a.chord,"chord_bits":a.chord.to_bits(),
                "section_points_f32_bits":a.points.iter().map(|p|[p.0.to_bits(),p.1.to_bits()]).collect::<Vec<_>>()})
        }).collect::<Vec<_>>()});
    }
    json!({"stored_metadata":"see generic recursive parameter fingerprints; caller constructor record and source bytes retained"})
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = std::env::args().collect::<Vec<_>>();
    if args.get(1).is_some_and(|s| s == "--serve-capsule") {
        let (sdf, p) = capsule();
        let mut r = Registry::default();
        let h = r.admit(sdf, p, 1.).map_err(|e| format!("{e:?}"))?;
        return Ok(v4_canonical_kernel::serve(r, Some(h))?);
    }
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../..")
        .canonicalize()?;
    let script = std::fs::read_to_string(
        root.join("research/sdf_cfd_mesher_v4/tools/ames_wingtip_builtin_probe.rhai"),
    )?;
    let cases = vec![
        ("capsule", None, 1.),
        ("plain_wing", Some(script.clone()), 0.001),
        ("sphere", Some("sphere(40.0)".into()), 0.001),
        (
            "transformed_sphere",
            Some("translate(rotate(sphere(40.0), 0.0, 10.0, 0.0), 10.0, 20.0, 30.0)".into()),
            0.001,
        ),
        (
            "scaled_sphere",
            Some("scale(sphere(40.0), 2.0, 1.0, 0.5)".into()),
            0.001,
        ),
    ];
    let mut rows = vec![];
    for (name, script, scale) in cases {
        let t = Instant::now();
        let (sdf, construction) = if let Some(ref s) = script {
            (
                scripting::evaluate_script(s)?.canonical_sdf,
                json!({"kind":"rhai_utf8","script":s,
                "script_sha256":format!("{:x}",sha2::Sha256::digest(s.as_bytes())),
                "external_project_context":"none; standalone script only","source":"ScriptResult::canonical_sdf"}),
            )
        } else {
            capsule()
        };
        let load = t.elapsed().as_secs_f64();
        let stored = stored_inputs(sdf.as_ref());
        let mut p = points();
        if let Some(bounds) = sdf.metadata().support_bounds {
            for point in bounds.corners() {
                p.push(point.to_array().map(|v| f64::from(v) * scale));
            }
            let mid = (bounds.min + bounds.max) * 0.5;
            for axis in 0..3 {
                for side in [bounds.min, bounds.max] {
                    let mut point = mid;
                    point[axis] = side[axis];
                    p.push(point.to_array().map(|v| f64::from(v) * scale));
                }
            }
        }
        let direct: Vec<f64> = p
            .iter()
            .map(|p| {
                f64::from(sdf.distance(Vec3::from_array(p.map(|x| (x / scale) as f32)))) * scale
            })
            .collect();
        let mut r = Registry::default();
        let h = r
            .admit(Arc::clone(&sdf), construction, scale)
            .map_err(|e| format!("{e:?}"))?;
        let bridge = r.get(h).unwrap().query(&p).unwrap();
        assert!(
            direct
                .iter()
                .zip(&bridge)
                .all(|(a, b)| a.to_bits() == b.to_bits())
        );
        let repeat = 8192;
        let t = Instant::now();
        let mut sink = 0.;
        for _ in 0..repeat {
            for p in &p {
                sink += f64::from(sdf.distance(Vec3::from_array(p.map(|x| (x / scale) as f32))))
                    * scale;
            }
        }
        std::hint::black_box(sink);
        let direct_seconds = t.elapsed().as_secs_f64();
        let t = Instant::now();
        for _ in 0..repeat {
            std::hint::black_box(r.get(h).unwrap().query(&p).unwrap());
        }
        let bridge_seconds = t.elapsed().as_secs_f64();
        rows.push(
            json!({"name":name,"script":script,"metres_per_model_unit":scale,
            "points_m":p,"direct_m":direct,"native_bridge_m":bridge,"all_bits_equal":true,
            "description":r.get(h).unwrap().description,"actual_stored_inputs":stored,
            "load_seconds":load,"timed_query_count":repeat*p.len(),"direct_seconds":direct_seconds,
            "native_bridge_seconds":bridge_seconds}),
        );
        r.release(h).unwrap();
        assert!(r.get(h).is_err());
    }
    let output = json!({"schema":"V4CanonicalDirectComparison/1","kernel_source_sha256":SOURCE_SHA256,"cases":rows});
    std::fs::write(&args[1], serde_json::to_string_pretty(&output)? + "\n")?;
    Ok(())
}

use sha2::Digest;
