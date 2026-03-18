// Improved Rhai error formatting with context and suggestions

pub struct FormattedError {
    pub line:          Option<usize>,
    pub column:        Option<usize>,
    pub message:       String,
    pub context_lines: Vec<(usize, String, bool)>, // (line_no, text, is_error_line)
    pub suggestion:    Option<String>,
}

/// Known function names for "did you mean?" suggestions.
static KNOWN_FUNCTIONS: &[&str] = &[
    "sphere", "box_", "cylinder", "torus", "cone", "plane",
    "union", "subtract", "intersect", "smooth_union", "smooth_subtract",
    "translate", "rotate", "scale", "offset", "shell",
    "mirror_x", "mirror_y", "mirror_z",
    "linear_array", "polar_array",
    "fuselage", "lofted_fuselage", "fuselage_station",
    "wing_with_airfoil", "wing_from_sections",
    "aileron", "elevator", "rudder", "flap", "elevon",
    "component", "component_named", "place", "geometry", "keepout",
    "mass_at", "mass_named",
    "spar", "rib_at_station", "bulkhead_at_station",
    "bulkhead_with_components", "bulkhead_auto", "auto_bulkheads",
    "point", "ref_point", "get_ref", "offset_point",
    "auto_bracket", "auto_bracket_flat", "auto_bracket_detect",
    "sweep", "cable_channel", "carbon_rod",
    "material", "shell_layer", "composite_layup_config", "apply_layup",
    "import_mesh",
];

pub fn format_script_error(source: &str, error_msg: &str) -> FormattedError {
    let (line, column) = parse_position(error_msg);
    let context_lines = build_context(source, line);
    let (message, suggestion) = categorize_error(error_msg);
    FormattedError { line, column, message, context_lines, suggestion }
}

/// Parse line and column from Rhai error message strings.
/// Looks for patterns like "line 5", "(line 5, position 3)", "at position (5, 3)".
fn parse_position(msg: &str) -> (Option<usize>, Option<usize>) {
    let mut line: Option<usize> = None;
    let mut column: Option<usize> = None;

    // Look for "line X" pattern
    if let Some(idx) = msg.find("line ") {
        let rest = &msg[idx + 5..];
        let num_str: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
        if !num_str.is_empty() {
            line = num_str.parse().ok();
        }
    }

    // Look for "position X" or ", X)" after "line N"
    if let Some(idx) = msg.find("position ") {
        let rest = &msg[idx + 9..];
        let num_str: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
        if !num_str.is_empty() {
            column = num_str.parse().ok();
        }
    } else if line.is_some() {
        // Try ", N)" pattern after "line N"
        if let Some(line_idx) = msg.find("line ") {
            let after_line = &msg[line_idx + 5..];
            // Skip the line number digits
            let rest: &str = after_line.trim_start_matches(|c: char| c.is_ascii_digit());
            // Look for ", N)" — comma then space then digits
            if rest.starts_with(", ") {
                let after_comma = &rest[2..];
                let num_str: String = after_comma.chars().take_while(|c| c.is_ascii_digit()).collect();
                if !num_str.is_empty() {
                    column = num_str.parse().ok();
                }
            }
        }
    }

    (line, column)
}

/// Build context lines around the error line.
fn build_context(source: &str, line: Option<usize>) -> Vec<(usize, String, bool)> {
    let Some(error_line) = line else { return vec![]; };
    let lines: Vec<&str> = source.lines().collect();
    let total = lines.len();
    if total == 0 { return vec![]; }

    let start = if error_line > 2 { error_line - 2 } else { 1 };
    let end = (error_line + 2).min(total);

    let mut result = Vec::new();
    for ln in start..=end {
        if ln >= 1 && ln <= total {
            let text = lines[ln - 1].to_string();
            let is_error = ln == error_line;
            result.push((ln, text, is_error));
        }
    }
    result
}

/// Categorize the error and produce a human-readable message + optional suggestion.
fn categorize_error(msg: &str) -> (String, Option<String>) {
    if msg.contains("Function not found:") {
        // Extract the function name from the error
        let fn_name = extract_function_name(msg);
        let suggestions = find_close_matches(&fn_name, KNOWN_FUNCTIONS, 3);
        let message = if suggestions.is_empty() {
            format!("Unknown function '{}'. Check the API reference for available functions.", fn_name)
        } else {
            format!("Unknown function '{}'. Did you mean: {}?",
                fn_name,
                suggestions.join(", "))
        };
        (message, None)
    } else if msg.contains("cannot be cast to") || msg.contains("mismatched types") {
        (format!("Type mismatch: {}. Check that you are passing the correct handle type — \
            SdfHandle, FieldHandle, PathHandle, and ComponentHandle are not interchangeable.", msg),
         None)
    } else if msg.contains("Variable not found") || msg.contains("undefined variable") {
        let var_name = extract_variable_name(msg);
        (format!("Variable '{}' not found. If this is a dimension, check the Dimensions panel. \
            If a library module, check the lib/ directory.", var_name),
         None)
    } else if msg.contains("Script must end with") {
        (msg.to_string(),
         Some("Make sure the last line is an SDF expression without a semicolon.".to_string()))
    } else if msg.contains("Parse error") || msg.contains("Syntax error") {
        (format!("Syntax error: {}. Common causes: missing closing parenthesis, extra comma, \
            or unclosed string literal.", msg),
         None)
    } else if msg.contains("expecting") && !msg.contains("Script must end") {
        (format!("Syntax error: {}. Common causes: missing closing parenthesis, extra comma, \
            or unclosed string literal.", msg),
         None)
    } else {
        (msg.to_string(), None)
    }
}

/// Extract function name from "Function not found: name (...)" pattern.
fn extract_function_name(msg: &str) -> String {
    if let Some(idx) = msg.find("Function not found:") {
        let rest = msg[idx + 19..].trim();
        // The name may be quoted or followed by " ("
        let name: String = if rest.starts_with('\'') || rest.starts_with('"') {
            rest.chars().skip(1).take_while(|&c| c != '\'' && c != '"').collect()
        } else {
            rest.chars().take_while(|c| c.is_alphanumeric() || *c == '_').collect()
        };
        if name.is_empty() { rest.split_whitespace().next().unwrap_or("unknown").to_string() } else { name }
    } else {
        "unknown".to_string()
    }
}

/// Extract variable name from "Variable not found: name" pattern.
fn extract_variable_name(msg: &str) -> String {
    for prefix in &["Variable not found:", "undefined variable"] {
        if let Some(idx) = msg.find(prefix) {
            let rest = msg[idx + prefix.len()..].trim();
            let name: String = if rest.starts_with('\'') || rest.starts_with('"') {
                rest.chars().skip(1).take_while(|&c| c != '\'' && c != '"').collect()
            } else {
                rest.chars().take_while(|c| c.is_alphanumeric() || *c == '_').collect()
            };
            if !name.is_empty() { return name; }
        }
    }
    "unknown".to_string()
}

/// Find up to `n` closest matches using Levenshtein distance.
fn find_close_matches<'a>(query: &str, candidates: &[&'a str], n: usize) -> Vec<&'a str> {
    let query_lower = query.to_lowercase();
    let mut scored: Vec<(usize, &str)> = candidates
        .iter()
        .map(|&c| (levenshtein_distance(&query_lower, c), c))
        .collect();
    scored.sort_by_key(|&(d, _)| d);
    // Only return suggestions that are reasonably close
    scored.into_iter()
        .filter(|&(d, _)| d <= 5)
        .take(n)
        .map(|(_, name)| name)
        .collect()
}

/// Standard Levenshtein distance between two strings.
pub fn levenshtein_distance(a: &str, b: &str) -> usize {
    let a: Vec<char> = a.chars().collect();
    let b: Vec<char> = b.chars().collect();
    let m = a.len();
    let n = b.len();

    let mut dp = vec![vec![0usize; n + 1]; m + 1];

    for i in 0..=m { dp[i][0] = i; }
    for j in 0..=n { dp[0][j] = j; }

    for i in 1..=m {
        for j in 1..=n {
            let cost = if a[i - 1] == b[j - 1] { 0 } else { 1 };
            dp[i][j] = (dp[i - 1][j] + 1)
                .min(dp[i][j - 1] + 1)
                .min(dp[i - 1][j - 1] + cost);
        }
    }
    dp[m][n]
}

/// Produce a concise human-readable error message for a cell error.
/// Strips Rhai boilerplate prefixes for readability in the inline error display.
pub fn format_cell_error(msg: &str) -> String {
    // Strip common Rhai prefixes.
    let stripped = msg
        .trim_start_matches("Script error:")
        .trim_start_matches("Rhai:")
        .trim();
    // Delegate to categorize for better messaging.
    let (message, _) = categorize_error(stripped);
    message
}

/// Build a human-readable error string with context and suggestion.
pub fn build_error_string(e: &FormattedError) -> String {
    let mut out = String::new();

    // Header with position
    match (e.line, e.column) {
        (Some(l), Some(c)) => out.push_str(&format!("[line {}, col {}] {}\n", l, c, e.message)),
        (Some(l), None)    => out.push_str(&format!("[line {}] {}\n", l, e.message)),
        _                  => out.push_str(&format!("{}\n", e.message)),
    }

    // Context lines
    if !e.context_lines.is_empty() {
        out.push('\n');
        for &(ln, ref text, is_error) in &e.context_lines {
            if is_error {
                out.push_str(&format!("> {:>4} | {}\n", ln, text));
                // Pointer line
                if let Some(col) = e.column {
                    let spaces = " ".repeat(col.saturating_sub(1) + 8); // align under the char
                    out.push_str(&format!("{}^ error here\n", spaces));
                } else {
                    out.push_str("       ^\n");
                }
            } else {
                out.push_str(&format!("  {:>4} | {}\n", ln, text));
            }
        }
    }

    // Suggestion
    if let Some(ref suggestion) = e.suggestion {
        out.push('\n');
        out.push_str(&format!("Suggestion: {}\n", suggestion));
    }

    out.trim_end().to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_position_line_only() {
        let (l, c) = parse_position("error at line 5 somewhere");
        assert_eq!(l, Some(5));
        assert!(c.is_none());
    }

    #[test]
    fn test_parse_position_with_column() {
        let (l, c) = parse_position("error (line 3, position 7)");
        assert_eq!(l, Some(3));
        assert_eq!(c, Some(7));
    }

    #[test]
    fn test_levenshtein() {
        assert_eq!(levenshtein_distance("sphere", "sphre"), 1);
        assert_eq!(levenshtein_distance("union", "union"), 0);
        assert_eq!(levenshtein_distance("abc", "xyz"), 3);
    }

    #[test]
    fn test_find_close_matches() {
        let matches = find_close_matches("sphre", KNOWN_FUNCTIONS, 3);
        assert!(matches.contains(&"sphere"), "Should suggest sphere for sphre");
    }

    #[test]
    fn test_categorize_function_not_found() {
        let (msg, _) = categorize_error("Function not found: my_func (...)");
        assert!(msg.contains("my_func"));
    }
}
