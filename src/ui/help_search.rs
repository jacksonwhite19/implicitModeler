// Help panel search state and filtering logic.

use crate::ui::help_data::{FUNCTION_DOCS, FunctionDoc};

pub struct HelpSearchState {
    pub query: String,
    pub selected_category: Option<String>,
    pub results: Vec<usize>,              // indices into FUNCTION_DOCS
    pub selected_function: Option<usize>, // index of currently expanded entry
    pub scroll_to: Option<usize>,
    pub recent_searches: Vec<String>,     // max 10, newest first
    pub show_recent: bool,
}

impl Default for HelpSearchState {
    fn default() -> Self { Self::new() }
}

impl HelpSearchState {
    pub fn new() -> Self {
        let mut s = Self {
            query: String::new(),
            selected_category: None,
            results: Vec::new(),
            selected_function: None,
            scroll_to: None,
            recent_searches: Vec::new(),
            show_recent: false,
        };
        s.update_results();
        s
    }

    pub fn update_results(&mut self) {
        let q = self.query.to_lowercase();

        if q.is_empty() && self.selected_category.is_none() {
            // All functions sorted by category then name
            let mut all: Vec<usize> = (0..FUNCTION_DOCS.len()).collect();
            all.sort_by(|&a, &b| {
                let da = &FUNCTION_DOCS[a];
                let db = &FUNCTION_DOCS[b];
                da.category.cmp(db.category).then(da.name.cmp(db.name))
            });
            self.results = all;
            return;
        }

        let mut scored: Vec<(usize, i32)> = FUNCTION_DOCS
            .iter()
            .enumerate()
            .filter_map(|(i, doc)| {
                // Category filter
                if let Some(ref cat) = self.selected_category {
                    if doc.category != cat { return None; }
                }

                if q.is_empty() {
                    return Some((i, 0));
                }

                let name_lower = doc.name.to_lowercase();
                let desc_lower = doc.description.to_lowercase();

                let mut score = 0i32;

                if name_lower.starts_with(&q) {
                    score = score.max(100);
                } else if name_lower.contains(&q) {
                    score = score.max(80);
                }

                for &tag in doc.tags {
                    let tag_lower = tag.to_lowercase();
                    if tag_lower == q {
                        score = score.max(70);
                    } else if tag_lower.contains(&q) {
                        score = score.max(40);
                    }
                }

                if desc_lower.contains(&q) {
                    score = score.max(50);
                }

                if score > 0 { Some((i, score)) } else { None }
            })
            .collect();

        scored.sort_by(|a, b| b.1.cmp(&a.1).then(
            FUNCTION_DOCS[a.0].name.cmp(FUNCTION_DOCS[b.0].name)
        ));

        self.results = scored.into_iter().map(|(i, _)| i).collect();
    }

    pub fn push_recent(&mut self, query: &str) {
        if query.trim().is_empty() { return; }
        self.recent_searches.retain(|s| s != query);
        self.recent_searches.insert(0, query.to_string());
        self.recent_searches.truncate(10);
    }

    pub fn move_selection(&mut self, delta: i32) {
        if self.results.is_empty() { return; }
        let n = self.results.len() as i32;
        let current = self.selected_function
            .and_then(|sel| self.results.iter().position(|&r| r == sel))
            .map(|p| p as i32)
            .unwrap_or(-1);
        let next = (current + delta).clamp(0, n - 1) as usize;
        self.selected_function = Some(self.results[next]);
        self.scroll_to = Some(next);
    }
}

// Suppress unused import warning when FUNCTION_DOCS is empty stub.
const _: fn() = || {
    let _: &FunctionDoc;
};
