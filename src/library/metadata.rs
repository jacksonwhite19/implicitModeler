// Library component metadata parsed from //! doc comment blocks.

#[derive(Default, Clone, Debug)]
pub struct ComponentMetadata {
    pub name:        Option<String>,
    pub description: Option<String>,
    pub author:      Option<String>,
    pub version:     Option<String>,
    pub tags:        Vec<String>,
    pub preview_fn:  Option<String>,
}

impl ComponentMetadata {
    /// Parse leading `//!` lines. Stop at the first non-`//!` line.
    pub fn parse(source: &str) -> Self {
        let mut meta = ComponentMetadata::default();
        for line in source.lines() {
            let trimmed = line.trim();
            let rest = match trimmed.strip_prefix("//!") {
                Some(r) => r.trim(),
                None    => break,
            };
            if let Some(v) = rest.strip_prefix("name:")        { meta.name        = Some(v.trim().to_string()); }
            else if let Some(v) = rest.strip_prefix("description:") { meta.description = Some(v.trim().to_string()); }
            else if let Some(v) = rest.strip_prefix("author:")  { meta.author      = Some(v.trim().to_string()); }
            else if let Some(v) = rest.strip_prefix("version:") { meta.version     = Some(v.trim().to_string()); }
            else if let Some(v) = rest.strip_prefix("tags:")    {
                meta.tags = v.split(',').map(|s| s.trim().to_string())
                             .filter(|s| !s.is_empty()).collect();
            }
            else if let Some(v) = rest.strip_prefix("preview_fn:") { meta.preview_fn = Some(v.trim().to_string()); }
        }
        meta
    }
}

/// Extract `fn` declaration signatures by scanning source text for `fn name(params)` patterns.
#[derive(Clone, Debug)]
pub struct FunctionSignature {
    pub name:        String,
    pub params:      Vec<String>,
    pub description: Option<String>,
}

pub fn extract_function_signatures(source: &str) -> Vec<FunctionSignature> {
    let mut sigs = Vec::new();
    let mut prev_doc: Option<String> = None;
    for line in source.lines() {
        let trimmed = line.trim();
        // Collect `// description` directly above a fn
        if let Some(doc) = trimmed.strip_prefix("//") {
            prev_doc = Some(doc.trim().to_string());
            continue;
        }
        if trimmed.starts_with("fn ") {
            if let Some(paren_open) = trimmed.find('(') {
                if let Some(paren_close) = trimmed.find(')') {
                    let name = trimmed[3..paren_open].trim().to_string();
                    let params_str = &trimmed[paren_open + 1..paren_close];
                    let params: Vec<String> = if params_str.trim().is_empty() {
                        Vec::new()
                    } else {
                        params_str.split(',').map(|p| p.trim().to_string())
                                  .filter(|p| !p.is_empty()).collect()
                    };
                    sigs.push(FunctionSignature {
                        name,
                        params,
                        description: prev_doc.take(),
                    });
                    continue;
                }
            }
        }
        // Any other non-empty non-comment line resets the doc accumulator
        if !trimmed.is_empty() && !trimmed.starts_with("//") {
            prev_doc = None;
        }
    }
    sigs
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_full_block() {
        let src = "//! name: Servo Mount\n//! description: Parametric servo\n//! tags: servo, rc\nfn build(w, l) { }";
        let m = ComponentMetadata::parse(src);
        assert_eq!(m.name.as_deref(), Some("Servo Mount"));
        assert_eq!(m.description.as_deref(), Some("Parametric servo"));
        assert_eq!(m.tags, vec!["servo", "rc"]);
        assert!(m.preview_fn.is_none());
    }

    #[test]
    fn parse_empty_source() {
        let m = ComponentMetadata::parse("fn build() {}");
        assert!(m.name.is_none());
    }

    #[test]
    fn extract_sigs() {
        let src = "// Create the geometry\nfn build(w, h, d) {}\nfn keepout(w, h) {}";
        let sigs = extract_function_signatures(src);
        assert_eq!(sigs.len(), 2);
        assert_eq!(sigs[0].name, "build");
        assert_eq!(sigs[0].params, vec!["w", "h", "d"]);
        assert_eq!(sigs[0].description.as_deref(), Some("Create the geometry"));
        assert_eq!(sigs[1].name, "keepout");
        assert_eq!(sigs[1].params, vec!["w", "h"]);
    }
}
