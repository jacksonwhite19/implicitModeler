// Script variable auto-detection for Bret Victor-style live manipulation.
//
// Detects numeric literals in:
//   Pass 1: `let name = <number>;` at top-level scope (LetBinding)
//   Pass 2: function call arguments like `foo(200.0, 100.0)` (InlineLiteral)

#[derive(Clone, Debug, PartialEq)]
pub enum DetectionType {
    LetBinding { variable_name: String },
    InlineLiteral { parent_function: Option<String> },
}

#[derive(Clone, Debug)]
pub struct DetectedVariable {
    pub name: String,        // variable name or auto label
    pub value: f64,
    pub line: usize,         // 0-indexed
    pub col_start: usize,    // char offset on that line where literal starts
    pub col_end: usize,      // char offset where literal ends (exclusive)
    pub detection_type: DetectionType,
    pub context: String,     // snippet for display
}

/// Detect all numeric variables/literals in the script source.
pub fn detect_script_variables(source: &str) -> Vec<DetectedVariable> {
    let mut results = Vec::new();

    // Pass 1: let-bindings at top-level scope
    let mut brace_depth: i32 = 0;
    for (line_idx, raw_line) in source.lines().enumerate() {
        // Update brace depth for this line BEFORE processing (so lines inside blocks are skipped)
        // We count braces on previous lines cumulatively; handle this by scanning as we go.
        let trimmed = raw_line.trim();

        // Only process let-bindings at depth 0
        if brace_depth == 0 && trimmed.starts_with("let ") {
            if let Some(var) = parse_let_binding(raw_line, line_idx) {
                results.push(var);
            }
        }

        // Update brace depth based on this line's content
        for ch in raw_line.chars() {
            match ch {
                '{' => brace_depth += 1,
                '}' => brace_depth = (brace_depth - 1).max(0),
                _ => {}
            }
        }
    }

    // Pass 2: inline literals in function calls
    let inline = detect_inline_literals(source, &results);
    results.extend(inline);

    results
}

/// Parse a single `let name = <number>;` line.
fn parse_let_binding(raw_line: &str, line_idx: usize) -> Option<DetectedVariable> {
    let chars: Vec<char> = raw_line.chars().collect();
    let mut pos = 0;

    // Skip leading whitespace
    while pos < chars.len() && chars[pos].is_whitespace() {
        pos += 1;
    }

    // Expect "let "
    let let_kw = "let ";
    if pos + let_kw.len() > chars.len() {
        return None;
    }
    let prefix: String = chars[pos..pos + let_kw.len()].iter().collect();
    if prefix != let_kw {
        return None;
    }
    pos += let_kw.len();

    // Skip optional "mut "
    let mut_kw = "mut ";
    if pos + mut_kw.len() <= chars.len() {
        let maybe_mut: String = chars[pos..pos + mut_kw.len()].iter().collect();
        if maybe_mut == mut_kw {
            pos += mut_kw.len();
        }
    }

    // Extract identifier
    let ident_start = pos;
    while pos < chars.len() && (chars[pos].is_alphanumeric() || chars[pos] == '_') {
        pos += 1;
    }
    if pos == ident_start {
        return None;
    }
    let variable_name: String = chars[ident_start..pos].iter().collect();
    // Must start with letter or underscore (not digit)
    if !chars[ident_start].is_alphabetic() && chars[ident_start] != '_' {
        return None;
    }

    // Skip whitespace
    while pos < chars.len() && chars[pos].is_whitespace() {
        pos += 1;
    }

    // Expect '='
    if pos >= chars.len() || chars[pos] != '=' {
        return None;
    }
    pos += 1;

    // Reject compound assignments (==, +=, -=, etc.)
    // We already consumed one '='; if next char is '=' it was '=='
    if pos < chars.len() && chars[pos] == '=' {
        return None;
    }

    // Skip whitespace
    while pos < chars.len() && chars[pos].is_whitespace() {
        pos += 1;
    }

    // Try to parse a number
    let col_start = pos;
    let rest: String = chars[pos..].iter().collect();
    let (value, char_len) = try_parse_number(&rest)?;
    let col_end = col_start + char_len;

    // Sanity check: value must be finite
    if !value.is_finite() {
        return None;
    }

    let context = raw_line.trim().to_string();

    Some(DetectedVariable {
        name: variable_name.clone(),
        value,
        line: line_idx,
        col_start,
        col_end,
        detection_type: DetectionType::LetBinding { variable_name },
        context,
    })
}

/// Detect numeric literals inside function call arguments.
fn detect_inline_literals(source: &str, let_bindings: &[DetectedVariable]) -> Vec<DetectedVariable> {
    let mut results = Vec::new();
    let lines: Vec<&str> = source.lines().collect();

    // Build a set of (line, col_start) for let-binding literals so we can skip them.
    let let_positions: std::collections::HashSet<(usize, usize)> =
        let_bindings.iter().map(|v| (v.line, v.col_start)).collect();

    // Walk the full source character by character tracking state.
    // We care about positions right after '(' or ',' (with optional whitespace).
    let mut line_idx: usize = 0;
    let mut col_idx: usize = 0; // char column on current line
    let chars: Vec<char> = source.chars().collect();
    let total = chars.len();
    let mut i = 0;

    // For tracking function names: we need a small buffer of recent identifier characters.
    // We'll track "last identifier before '('"
    // Simple approach: scan the raw line backward from '(' position.

    // Track call depth and argument index for labeling.
    // Stack: (function_name, arg_index)
    let mut call_stack: Vec<(Option<String>, usize)> = Vec::new();

    while i < total {
        let ch = chars[i];

        if ch == '\n' {
            line_idx += 1;
            col_idx = 0;
            i += 1;
            continue;
        }

        if ch == '(' {
            // Find the function name by scanning backward on this line.
            let raw_line = if line_idx < lines.len() { lines[line_idx] } else { "" };
            let fn_name = find_function_name_before(raw_line, col_idx);
            call_stack.push((fn_name, 0));
            col_idx += 1;
            i += 1;
            continue;
        }

        if ch == ')' {
            call_stack.pop();
            col_idx += 1;
            i += 1;
            continue;
        }

        if ch == ',' {
            if let Some(top) = call_stack.last_mut() {
                top.1 += 1;
            }
            col_idx += 1;
            i += 1;
            continue;
        }

        // After '(' or ',', skip whitespace then try to parse a number.
        // We detect a "potential argument position" by checking if we just passed a '(' or ','.
        // Instead of tracking state explicitly, check character by character:
        // A number at argument position means: the character before (ignoring whitespace) is '(' or ','.
        // This is complex to do in a forward scan — use a simpler approach:
        // just try to parse a number whenever we're inside a call (call_stack non-empty)
        // and the current position looks like the start of a standalone number.

        if !call_stack.is_empty() {
            // Check if this could be a number argument start
            if ch.is_ascii_digit() || (ch == '-' && i + 1 < total && chars[i + 1].is_ascii_digit()) {
                // Check that previous non-whitespace char is '(' or ','
                if is_after_arg_separator(&chars, i) {
                    // Don't overlap with let-bindings
                    if !let_positions.contains(&(line_idx, col_idx)) {
                        let rest: String = chars[i..].iter().collect();
                        if let Some((value, char_len)) = try_parse_number(&rest) {
                            if value.is_finite() {
                                let (fn_name, arg_idx) = call_stack.last()
                                    .map(|(f, a)| (f.clone(), *a))
                                    .unwrap_or((None, 0));

                                let name = match &fn_name {
                                    Some(f) => format!("{} arg {}", f, arg_idx + 1),
                                    None => format!("line {} col {}", line_idx + 1, col_idx + 1),
                                };

                                let raw_line = if line_idx < lines.len() { lines[line_idx] } else { "" };
                                results.push(DetectedVariable {
                                    name,
                                    value,
                                    line: line_idx,
                                    col_start: col_idx,
                                    col_end: col_idx + char_len,
                                    detection_type: DetectionType::InlineLiteral {
                                        parent_function: fn_name,
                                    },
                                    context: raw_line.trim().to_string(),
                                });

                                // Advance past the literal
                                for _ in 0..char_len {
                                    if i < total {
                                        if chars[i] == '\n' {
                                            line_idx += 1;
                                            col_idx = 0;
                                        } else {
                                            col_idx += 1;
                                        }
                                        i += 1;
                                    }
                                }
                                continue;
                            }
                        }
                    }
                }
            }
        }

        col_idx += 1;
        i += 1;
    }

    results
}

/// Check whether position `i` in `chars` is right after a `(` or `,` (with optional whitespace).
fn is_after_arg_separator(chars: &[char], i: usize) -> bool {
    if i == 0 {
        return false;
    }
    let mut j = i as isize - 1;
    while j >= 0 && chars[j as usize].is_whitespace() && chars[j as usize] != '\n' {
        j -= 1;
    }
    if j < 0 {
        return false;
    }
    matches!(chars[j as usize], '(' | ',')
}

/// Find the identifier immediately before `(` at `col` on `line`.
fn find_function_name_before(line: &str, col: usize) -> Option<String> {
    let chars: Vec<char> = line.chars().collect();
    // col is the position of '(' — scan backward
    if col == 0 {
        return None;
    }
    let mut j = col as isize - 1;
    // Skip whitespace
    while j >= 0 && chars[j as usize].is_whitespace() {
        j -= 1;
    }
    if j < 0 {
        return None;
    }
    // Check if this char is identifier-like
    if !chars[j as usize].is_alphanumeric() && chars[j as usize] != '_' {
        return None;
    }
    // Scan to start of identifier
    let end = j as usize + 1;
    while j >= 0 && (chars[j as usize].is_alphanumeric() || chars[j as usize] == '_') {
        j -= 1;
    }
    let start = (j + 1) as usize;
    let name: String = chars[start..end].iter().collect();
    if name.is_empty() {
        None
    } else {
        Some(name)
    }
}

/// Try to parse a numeric literal from the start of `s`.
/// Returns `(value, char_length_consumed)` or `None`.
pub fn try_parse_number(s: &str) -> Option<(f64, usize)> {
    let chars: Vec<char> = s.chars().collect();
    let mut len = 0;

    // Optional leading minus
    if chars.get(0) == Some(&'-') {
        len += 1;
    }

    // Must have at least one digit after possible minus
    if len >= chars.len() || !chars[len].is_ascii_digit() {
        return None;
    }

    // Integer digits
    while len < chars.len() && chars[len].is_ascii_digit() {
        len += 1;
    }

    // Decimal part
    if len < chars.len() && chars[len] == '.' {
        len += 1;
        while len < chars.len() && chars[len].is_ascii_digit() {
            len += 1;
        }
    }

    // Exponent part
    if len < chars.len() && (chars[len] == 'e' || chars[len] == 'E') {
        len += 1;
        if len < chars.len() && (chars[len] == '+' || chars[len] == '-') {
            len += 1;
        }
        while len < chars.len() && chars[len].is_ascii_digit() {
            len += 1;
        }
    }

    if len == 0 || (len == 1 && chars[0] == '-') {
        return None;
    }

    let substr: String = chars[..len].iter().collect();
    substr.parse::<f64>().ok().map(|v| (v, len))
}

// ── Literal rewriting ─────────────────────────────────────────────────────────

/// Rewrite the literal at (line, col_start..col_end) with `new_value`,
/// preserving the decimal-place count from `original_literal`.
pub fn rewrite_literal(
    source: &str,
    line: usize,
    col_start: usize,
    col_end: usize,
    new_value: f64,
    original_literal: &str,
) -> String {
    let lines: Vec<&str> = source.lines().collect();
    if line >= lines.len() {
        return source.to_string();
    }

    let target_line = lines[line];
    let decimal_places = count_decimal_places(original_literal);
    let formatted = format!("{:.prec$}", new_value, prec = decimal_places.max(1));

    let char_indices: Vec<(usize, char)> = target_line.char_indices().collect();
    let char_count = char_indices.len();

    let byte_start = if col_start < char_count {
        char_indices[col_start].0
    } else {
        target_line.len()
    };
    let byte_end = if col_end < char_count {
        char_indices[col_end].0
    } else {
        target_line.len()
    };

    let mut new_line = target_line.to_string();
    if byte_start <= byte_end && byte_end <= new_line.len() {
        new_line.replace_range(byte_start..byte_end, &formatted);
    }

    let mut result_lines: Vec<String> = lines.iter().map(|l| l.to_string()).collect();
    result_lines[line] = new_line;
    let joined = result_lines.join("\n");

    // Preserve trailing newline if original had one
    if source.ends_with('\n') && !joined.ends_with('\n') {
        format!("{}\n", joined)
    } else {
        joined
    }
}

fn count_decimal_places(literal: &str) -> usize {
    if let Some(dot_pos) = literal.find('.') {
        let after_dot = &literal[dot_pos + 1..];
        // Stop at 'e' or 'E'
        after_dot.chars().take_while(|c| c.is_ascii_digit()).count()
    } else {
        0
    }
}

/// Extract the literal string at the given line/col range.
pub fn extract_literal_at(source: &str, line: usize, col_start: usize, col_end: usize) -> String {
    let lines: Vec<&str> = source.lines().collect();
    if line >= lines.len() {
        return "0.0".to_string();
    }
    let chars: Vec<char> = lines[line].chars().collect();
    if col_start >= chars.len() {
        return "0.0".to_string();
    }
    let end = col_end.min(chars.len());
    chars[col_start..end].iter().collect()
}

// ── Unit inference ────────────────────────────────────────────────────────────

pub fn infer_unit(name: &str) -> &'static str {
    let lower = name.to_lowercase();
    if lower.ends_with("_mm")
        || lower.ends_with("_length")
        || lower.ends_with("_width")
        || lower.ends_with("_height")
        || lower.ends_with("_radius")
        || lower.ends_with("_diameter")
        || lower.ends_with("_span")
        || lower.ends_with("_chord")
    {
        return "mm";
    }
    if lower.ends_with("_deg")
        || lower.ends_with("_angle")
        || lower.ends_with("_sweep")
        || lower.ends_with("_dihedral")
    {
        return "°";
    }
    if lower.ends_with("_ms") {
        return "m/s";
    }
    if lower.ends_with("_n") {
        return "N";
    }
    if lower.ends_with("_g") {
        return "g";
    }
    if lower.ends_with("_mah") {
        return "mAh";
    }
    ""
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detect_let_binding() {
        let source = "let wingspan = 800.0;\nlet chord = 150.0;";
        let vars = detect_script_variables(source);
        let wing = vars.iter().find(|v| v.name == "wingspan").expect("should detect wingspan");
        assert_eq!(wing.value, 800.0);
        assert_eq!(wing.line, 0);
        assert!(matches!(wing.detection_type, DetectionType::LetBinding { .. }));
    }

    #[test]
    fn test_detect_let_binding_column_position() {
        let source = "let wingspan = 800.0;";
        let vars = detect_script_variables(source);
        let wing = vars.iter().find(|v| v.name == "wingspan").unwrap();
        // "let wingspan = " is 15 chars, so col_start = 15
        assert_eq!(wing.col_start, 15, "col_start should point to the literal");
        // "800.0" is 5 chars
        assert_eq!(wing.col_end, 20);
    }

    #[test]
    fn test_detect_inline_literal() {
        let source = "let w = wing_with_airfoil(\"2412\", 200.0, 100.0);";
        let vars = detect_script_variables(source);
        // Should detect 200.0 and 100.0 as InlineLiterals
        let inline: Vec<_> = vars
            .iter()
            .filter(|v| matches!(v.detection_type, DetectionType::InlineLiteral { .. }))
            .collect();
        assert!(inline.len() >= 1, "should detect inline literals");
    }

    #[test]
    fn test_rewrite_literal() {
        let source = "let wingspan = 800.0;\nlet chord = 150.0;";
        let result = rewrite_literal(source, 0, 15, 20, 900.0, "800.0");
        assert!(result.contains("900.0"), "should contain new value");
        assert!(result.contains("150.0"), "should not change other lines");
        assert!(!result.contains("800.0"), "should replace old value");
    }

    #[test]
    fn test_rewrite_preserves_decimal_places() {
        let source = "let x = 1.500;";
        let result = rewrite_literal(source, 0, 8, 13, 2.0, "1.500");
        assert!(result.contains("2.000"), "should preserve 3 decimal places");
    }

    #[test]
    fn test_skip_fn_block() {
        let source = "fn helper() {\n    let x = 5.0;\n}\nlet y = 10.0;";
        let vars = detect_script_variables(source);
        assert!(vars.iter().any(|v| v.name == "y"), "top-level y should be detected");
        assert!(!vars.iter().any(|v| v.name == "x"), "x inside fn block should not be detected");
    }

    #[test]
    fn test_try_parse_number() {
        assert_eq!(try_parse_number("123.45 rest"), Some((123.45, 6)));
        assert_eq!(try_parse_number("-5.0 rest"), Some((-5.0, 4)));
        assert_eq!(try_parse_number("1e3"), Some((1000.0, 3)));
        assert_eq!(try_parse_number("abc"), None);
    }
}
