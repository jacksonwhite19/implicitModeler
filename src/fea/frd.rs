// CalculiX .frd result file parser.
//
// Extracts nodal displacement and Von Mises stress from ASCII .frd output.
// The .frd format stores values in 12-character wide fields (Fortran E12.5),
// which can run together without whitespace separators.

use std::collections::HashMap;

/// Parsed FEA results: displacement magnitude and Von Mises stress per node (0-indexed).
pub struct FrdResult {
    /// Displacement magnitude [mm] at each mesh node (index = node 0-based).
    pub displacement: Vec<f32>,
    /// Von Mises stress [MPa] at each mesh node.
    pub von_mises: Vec<f32>,
}

/// Parse a CalculiX .frd file and return per-node results.
/// `num_nodes` must match the node count in the mesh (used to pre-allocate).
pub fn parse_frd(content: &str, num_nodes: usize) -> Result<FrdResult, String> {
    let mut displacements: HashMap<usize, [f32; 3]> = HashMap::new();
    let mut stresses:      HashMap<usize, [f32; 6]> = HashMap::new();

    #[derive(Clone, Copy, PartialEq)]
    enum Block { None, Disp, Stress }

    let mut block = Block::None;

    for line in content.lines() {
        // Block header: "    3" prefix followed by result name
        if line.starts_with("    3") || line.starts_with("   3 ") {
            let upper = line.to_uppercase();
            if upper.contains("DISP") {
                block = Block::Disp;
            } else if upper.contains("STRESS") || upper.contains("S  ") || upper.contains("STRESS") {
                block = Block::Stress;
            } else {
                block = Block::None;
            }
            continue;
        }

        // End of block
        if line.starts_with(" -2") || line.starts_with(" -3") {
            block = Block::None;
            continue;
        }

        // Skip component descriptor lines
        if line.starts_with("-4") || line.starts_with("-5") {
            continue;
        }

        // Value record: " -1" prefix
        if line.starts_with(" -1") && block != Block::None {
            // Fields: " -1" (3 chars), node_id (12 chars), then 12-char value fields
            let data = &line[3..];
            if data.len() < 12 { continue; }

            let node_id: usize = match data[..12].trim().parse::<i64>() {
                Ok(n) if n >= 1 => (n - 1) as usize, // convert to 0-based
                _ => continue,
            };

            let vals = parse_packed_floats(&data[12..]);

            match block {
                Block::Disp if vals.len() >= 3 => {
                    displacements.insert(node_id, [vals[0], vals[1], vals[2]]);
                }
                Block::Stress if vals.len() >= 6 => {
                    stresses.insert(node_id, [vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]]);
                }
                _ => {}
            }
        }
    }

    if displacements.is_empty() && stresses.is_empty() {
        return Err("No results found in .frd file. CalculiX may have failed.".into());
    }

    let mut displacement_out = vec![0.0f32; num_nodes];
    let mut von_mises_out    = vec![0.0f32; num_nodes];

    for (node_id, uvw) in &displacements {
        if *node_id < num_nodes {
            displacement_out[*node_id] = (uvw[0]*uvw[0] + uvw[1]*uvw[1] + uvw[2]*uvw[2]).sqrt();
        }
    }

    for (node_id, s) in &stresses {
        if *node_id < num_nodes {
            von_mises_out[*node_id] = von_mises(s[0], s[1], s[2], s[3], s[4], s[5]);
        }
    }

    Ok(FrdResult { displacement: displacement_out, von_mises: von_mises_out })
}

/// Parse packed 12-character float fields from a value record tail.
/// Handles the case where values run together (no whitespace) in Fortran E12.5 format.
fn parse_packed_floats(s: &str) -> Vec<f32> {
    let mut vals = Vec::new();
    let bytes = s.as_bytes();
    let mut i = 0;
    while i + 12 <= bytes.len() {
        let field = &s[i..i + 12];
        if let Ok(v) = field.trim().parse::<f32>() {
            vals.push(v);
        }
        i += 12;
    }
    vals
}

/// Von Mises stress from 6 stress tensor components [sxx, syy, szz, sxy, syz, szx].
fn von_mises(sxx: f32, syy: f32, szz: f32, sxy: f32, syz: f32, szx: f32) -> f32 {
    let d1 = sxx - syy;
    let d2 = syy - szz;
    let d3 = szz - sxx;
    (0.5 * (d1*d1 + d2*d2 + d3*d3 + 6.0*(sxy*sxy + syz*syz + szx*szx))).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_von_mises_uniaxial() {
        // Pure uniaxial stress: sxx = 100, all others 0 → VM = 100
        let vm = von_mises(100.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        assert!((vm - 100.0).abs() < 0.01, "VM = {vm}");
    }

    #[test]
    fn test_von_mises_hydrostatic() {
        // Hydrostatic: sxx = syy = szz = p, no shear → VM = 0
        let vm = von_mises(100.0, 100.0, 100.0, 0.0, 0.0, 0.0);
        assert!(vm < 0.01, "VM = {vm}");
    }

    #[test]
    fn test_parse_packed_floats() {
        // Typical CalculiX output: 3 values packed together
        let s = " 1.23456E+02-2.34567E+01 3.45678E-02";
        let vals = parse_packed_floats(s);
        assert_eq!(vals.len(), 3);
        assert!((vals[0] - 123.456).abs() < 0.01);
        assert!((vals[1] - (-23.4567)).abs() < 0.01);
        assert!((vals[2] - 0.0345678).abs() < 0.001);
    }

    #[test]
    fn test_parse_frd_minimal() {
        // Minimal synthetic .frd with displacement only.
        // Use a raw string to preserve leading whitespace (the 4-space indentation
        // of CalculiX block headers must not be stripped).
        let frd = concat!(
            "    1UDATE              2026-03-14\n",
            "    3                   100DISP        4    1\n",
            "-4  U           4    1\n",
            "-4  U1          1    2    1    0\n",
            "-4  U2          1    2    2    0\n",
            "-4  U3          1    2    3    0\n",
            "-4  ALL         0    0    0    0    1\n",
            " -1           1 3.00000E+00 4.00000E+00 0.00000E+00\n",
            " -2\n",
            " -3\n",
        );
        let result = parse_frd(frd, 2).unwrap();
        // Node 0: displacement = sqrt(9+16+0) = 5.0
        assert!((result.displacement[0] - 5.0).abs() < 0.01,
            "disp = {}", result.displacement[0]);
    }
}
