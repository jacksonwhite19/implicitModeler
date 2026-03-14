// Generate Rhai script from a Notebook

use std::collections::HashMap;
use super::types::*;

/// Generate a Rhai script from the notebook, evaluating only up to (and including)
/// the preview block. Returns a script string ready to pass to evaluate_script().
pub fn notebook_to_rhai(notebook: &Notebook) -> String {
    if notebook.blocks.is_empty() {
        return "sphere(0.001)".to_string();
    }

    let preview_idx = notebook.preview_index();

    let mut lines = Vec::new();
    // Maps block id → variable name, built as we iterate.
    let mut id_to_var: HashMap<u64, String> = HashMap::new();
    // Ordered variable names (parallel to iterated blocks).
    let mut ordered_vars: Vec<String> = Vec::new();

    for (i, block) in notebook.blocks.iter().enumerate().take(preview_idx + 1) {
        // Section blocks are visual-only dividers — skip code generation.
        if block.kind == NbBlockKind::Section {
            let _ = i;
            continue;
        }

        let var = format!("_b{}", block.id);

        // Resolve "previous" to the var of the block directly above.
        let prev_var = ordered_vars
            .last()
            .map(|s| s.as_str())
            .unwrap_or("sphere(0.001)");

        let input_a = resolve(&block.input_a, prev_var, &id_to_var);
        let input_b = resolve(&block.input_b, prev_var, &id_to_var);

        let expr = block_expr(block, &input_a, &input_b);
        lines.push(format!("let {} = {};", var, expr));

        id_to_var.insert(block.id, var.clone());
        ordered_vars.push(var);
        let _ = i;
    }

    // Last line: return the preview block's variable.
    if let Some(last_var) = ordered_vars.last() {
        lines.push(last_var.clone());
    } else {
        lines.push("sphere(0.001)".to_string());
    }

    lines.join("\n")
}

fn resolve(r: &NbInputRef, prev_var: &str, id_to_var: &HashMap<u64, String>) -> String {
    match r {
        NbInputRef::Previous => prev_var.to_string(),
        NbInputRef::BlockId(id) => id_to_var
            .get(id)
            .cloned()
            .unwrap_or_else(|| prev_var.to_string()),
    }
}

/// Format an f32 for Rhai — always include decimal point so Rhai treats it as float.
fn ff(v: f32) -> String {
    if v.fract() == 0.0 && v.abs() < 1_000_000.0 {
        format!("{:.1}", v)
    } else {
        format!("{}", v)
    }
}

fn fv(params: &[NbFloatParam], i: usize) -> f32 {
    params.get(i).map(|p| p.value).unwrap_or(0.0)
}

fn sv(params: &[NbStringParam], i: usize) -> String {
    params
        .get(i)
        .map(|p| format!("\"{}\"", p.value))
        .unwrap_or_else(|| "\"\"".to_string())
}

fn block_expr(block: &NbBlock, a: &str, b: &str) -> String {
    let fp = &block.float_params;
    let sp = &block.string_params;

    match block.kind {
        NbBlockKind::Sphere    => format!("sphere({})", ff(fv(fp, 0))),
        NbBlockKind::Box_      => format!("box_({}, {}, {})", ff(fv(fp,0)), ff(fv(fp,1)), ff(fv(fp,2))),
        NbBlockKind::Cylinder  => format!("cylinder({}, {})", ff(fv(fp,0)), ff(fv(fp,1))),
        NbBlockKind::Torus     => format!("torus({}, {})", ff(fv(fp,0)), ff(fv(fp,1))),
        NbBlockKind::Cone      => format!("cone({}, {})", ff(fv(fp,0)), ff(fv(fp,1))),

        NbBlockKind::Union          => format!("union({}, {})", a, b),
        NbBlockKind::Subtract       => format!("subtract({}, {})", a, b),
        NbBlockKind::Intersect      => format!("intersect({}, {})", a, b),
        NbBlockKind::SmoothUnion    => format!("smooth_union({}, {}, {})", a, b, ff(fv(fp,0))),
        NbBlockKind::SmoothSubtract => format!("smooth_subtract({}, {}, {})", a, b, ff(fv(fp,0))),

        NbBlockKind::Translate => format!("translate({}, {}, {}, {})", a, ff(fv(fp,0)), ff(fv(fp,1)), ff(fv(fp,2))),
        NbBlockKind::Rotate    => format!("rotate({}, {}, {}, {})", a, ff(fv(fp,0)), ff(fv(fp,1)), ff(fv(fp,2))),
        NbBlockKind::Scale     => format!("scale({}, {}, {}, {})", a, ff(fv(fp,0)), ff(fv(fp,1)), ff(fv(fp,2))),
        NbBlockKind::Offset    => format!("offset({}, {})", a, ff(fv(fp,0))),
        NbBlockKind::Shell     => format!("shell({}, {})", a, ff(fv(fp,0))),

        NbBlockKind::LinearArray => format!("linear_array({}, {}, {}, {}, {})",
            a, fv(fp,0) as i64, ff(fv(fp,1)), ff(fv(fp,2)), ff(fv(fp,3))),
        NbBlockKind::PolarArray  => format!("polar_array({}, {})", a, fv(fp,0) as i64),
        NbBlockKind::MirrorX     => format!("mirror_x({})", a),
        NbBlockKind::MirrorY     => format!("mirror_y({})", a),
        NbBlockKind::MirrorZ     => format!("mirror_z({})", a),

        NbBlockKind::Fuselage    => format!("fuselage_parametric({}, {}, {}, {})",
            ff(fv(fp,0)), ff(fv(fp,1)), ff(fv(fp,2)), ff(fv(fp,3))),
        NbBlockKind::Nacelle     => format!("nacelle({}, {}, {}, {})",
            ff(fv(fp,0)), ff(fv(fp,1)), ff(fv(fp,2)), ff(fv(fp,3))),
        NbBlockKind::NacaAirfoil => format!("naca({}, {})", sv(sp, 0), ff(fv(fp,0))),
        NbBlockKind::Wing        => format!("wing_with_airfoil({}, {}, {}, {}, {}, {}, {})",
            sv(sp,0), ff(fv(fp,0)), ff(fv(fp,1)), ff(fv(fp,2)),
            ff(fv(fp,3)), ff(fv(fp,4)), ff(fv(fp,5))),
        // Section is a visual-only divider; codegen skips it before calling block_expr.
        NbBlockKind::Section => "sphere(0.001)".to_string(),
    }
}
