// Component registry for discovery and loading

use super::library::ComponentDef;
use std::collections::HashMap;
use std::fs;
use std::io;
use std::path::Path;

/// In-memory registry of available components
pub struct ComponentRegistry {
    components: HashMap<String, ComponentDef>,
    categories: HashMap<String, Vec<String>>,
}

impl ComponentRegistry {
    /// Create a new empty registry
    pub fn new() -> Self {
        Self {
            components: HashMap::new(),
            categories: HashMap::new(),
        }
    }

    /// Load all components from a directory
    pub fn load_from_directory(&mut self, dir: impl AsRef<Path>) -> Result<usize, io::Error> {
        let dir = dir.as_ref();

        if !dir.exists() {
            // Directory doesn't exist, not an error (just no components to load)
            return Ok(0);
        }

        let mut loaded_count = 0;

        // Recursively walk directory
        self.load_from_directory_recursive(dir, &mut loaded_count)?;

        Ok(loaded_count)
    }

    fn load_from_directory_recursive(&mut self, dir: &Path, count: &mut usize) -> Result<(), io::Error> {
        for entry in fs::read_dir(dir)? {
            let entry = entry?;
            let path = entry.path();

            if path.is_dir() {
                // Recurse into subdirectories
                self.load_from_directory_recursive(&path, count)?;
            } else if path.extension().and_then(|s| s.to_str()) == Some("json") {
                // Try to load JSON file as component
                match self.load_component_file(&path) {
                    Ok(component) => {
                        self.register(component);
                        *count += 1;
                    }
                    Err(e) => {
                        eprintln!("Warning: Failed to load component from {}: {}", path.display(), e);
                    }
                }
            }
        }

        Ok(())
    }

    fn load_component_file(&self, path: &Path) -> Result<ComponentDef, io::Error> {
        let json = fs::read_to_string(path)?;
        let component: ComponentDef = serde_json::from_str(&json)
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
        Ok(component)
    }

    /// Register a component in the registry
    pub fn register(&mut self, component: ComponentDef) {
        let name = component.name.clone();
        let category = component.category.clone();

        // Add to category index
        self.categories.entry(category.clone())
            .or_insert_with(Vec::new)
            .push(name.clone());

        // Add to components map
        self.components.insert(name, component);
    }

    /// Get a component by name
    pub fn get(&self, name: &str) -> Option<&ComponentDef> {
        self.components.get(name)
    }

    /// List all category names
    pub fn list_categories(&self) -> Vec<String> {
        let mut categories: Vec<String> = self.categories.keys().cloned().collect();
        categories.sort();
        categories
    }

    /// List components in a category
    pub fn list_by_category(&self, category: &str) -> Vec<&ComponentDef> {
        self.categories.get(category)
            .map(|names| {
                names.iter()
                    .filter_map(|n| self.components.get(n))
                    .collect()
            })
            .unwrap_or_default()
    }

    /// Get total number of registered components
    pub fn count(&self) -> usize {
        self.components.len()
    }
}

impl Default for ComponentRegistry {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::components::library::{ParamType, ParamValue, ParameterDef};
    use std::collections::HashMap;
    use tempfile::TempDir;

    #[test]
    fn test_register_and_get() {
        let mut registry = ComponentRegistry::new();

        let component = ComponentDef {
            name: "test_component".to_string(),
            category: "test".to_string(),
            description: "Test component".to_string(),
            parameters: HashMap::new(),
            script_template: "sphere(10.0)".to_string(),
        };

        registry.register(component.clone());

        assert_eq!(registry.count(), 1);
        assert!(registry.get("test_component").is_some());
        assert_eq!(registry.get("test_component").unwrap().name, "test_component");
    }

    #[test]
    fn test_categories() {
        let mut registry = ComponentRegistry::new();

        let component1 = ComponentDef {
            name: "comp1".to_string(),
            category: "aerospace".to_string(),
            description: "Test".to_string(),
            parameters: HashMap::new(),
            script_template: "test()".to_string(),
        };

        let component2 = ComponentDef {
            name: "comp2".to_string(),
            category: "primitives".to_string(),
            description: "Test".to_string(),
            parameters: HashMap::new(),
            script_template: "test()".to_string(),
        };

        registry.register(component1);
        registry.register(component2);

        let categories = registry.list_categories();
        assert_eq!(categories.len(), 2);
        assert!(categories.contains(&"aerospace".to_string()));
        assert!(categories.contains(&"primitives".to_string()));

        let aerospace_components = registry.list_by_category("aerospace");
        assert_eq!(aerospace_components.len(), 1);
        assert_eq!(aerospace_components[0].name, "comp1");
    }

    #[test]
    fn test_load_from_directory_missing() {
        let mut registry = ComponentRegistry::new();

        // Should not error if directory doesn't exist
        let result = registry.load_from_directory("/nonexistent/path");
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), 0);
    }

    #[test]
    fn test_load_from_directory() {
        let temp_dir = TempDir::new().unwrap();
        let component_dir = temp_dir.path().join("components");
        fs::create_dir(&component_dir).unwrap();

        // Create a test component JSON file
        let mut params = HashMap::new();
        params.insert("radius".to_string(), ParameterDef {
            param_type: ParamType::Float,
            default: ParamValue::Float(10.0),
            min: Some(1.0),
            max: Some(100.0),
            description: None,
        });

        let component = ComponentDef {
            name: "test_sphere".to_string(),
            category: "primitives".to_string(),
            description: "Test sphere".to_string(),
            parameters: params,
            script_template: "sphere(#{radius})".to_string(),
        };

        let json = serde_json::to_string_pretty(&component).unwrap();
        fs::write(component_dir.join("test_sphere.json"), json).unwrap();

        // Load components
        let mut registry = ComponentRegistry::new();
        let count = registry.load_from_directory(&component_dir).unwrap();

        assert_eq!(count, 1);
        assert_eq!(registry.count(), 1);
        assert!(registry.get("test_sphere").is_some());
    }
}
