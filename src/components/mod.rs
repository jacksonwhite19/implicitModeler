// Component system for reusable parametric blocks

pub mod library;
pub mod registry;

pub use library::{ComponentDef, ParamValue};
pub use registry::ComponentRegistry;

use std::collections::HashMap;

/// Runtime component instance with specific parameter values
pub struct ComponentInstance {
    pub component_def: ComponentDef,
    pub param_values: HashMap<String, ParamValue>,
}

impl ComponentInstance {
    /// Create a new component instance with default parameter values
    pub fn new(component_def: ComponentDef) -> Self {
        let mut param_values = HashMap::new();
        for (name, def) in &component_def.parameters {
            param_values.insert(name.clone(), def.default.clone());
        }

        Self {
            component_def,
            param_values,
        }
    }

    /// Create a component instance with custom parameter values
    #[allow(dead_code)] // Part of component instance API
    pub fn with_params(component_def: ComponentDef, param_values: HashMap<String, ParamValue>) -> Self {
        Self {
            component_def,
            param_values,
        }
    }

    /// Set a parameter value
    #[allow(dead_code)] // Part of component instance API
    pub fn set_param(&mut self, name: String, value: ParamValue) {
        self.param_values.insert(name, value);
    }

    /// Validate all parameter values against definitions
    pub fn validate(&self) -> Result<(), String> {
        for (name, value) in &self.param_values {
            let def = self.component_def.parameters.get(name)
                .ok_or(format!("Unknown parameter: {}", name))?;

            def.validate(value)?;
        }
        Ok(())
    }

    /// Generate Rhai script by substituting parameters (no nesting yet)
    pub fn generate_script(&self) -> Result<String, String> {
        // Validate first
        self.validate()?;

        let mut script = self.component_def.script_template.clone();

        // Substitute parameters #{param_name} → value
        for (param_name, param_value) in &self.param_values {
            let placeholder = format!("#{{{}}}",  param_name);
            let value_str = param_value.to_script_string();
            script = script.replace(&placeholder, &value_str);
        }

        // Validate all placeholders were resolved
        if script.contains("#{") {
            return Err("Unresolved parameters in template".to_string());
        }

        Ok(script)
    }

    /// Generate script with nested component expansion (Step 8)
    #[allow(dead_code)] // Part of component nesting API
    pub fn generate_script_with_nesting(&self, registry: &ComponentRegistry) -> Result<String, String> {
        // Validate first
        self.validate()?;

        let mut script = self.component_def.script_template.clone();

        // Expand nested components @{component_name(...)}
        script = self.expand_nested_components(script, registry, 0)?;

        // Substitute parameters #{param_name} → value
        for (param_name, param_value) in &self.param_values {
            let placeholder = format!("#{{{}}}",  param_name);
            let value_str = param_value.to_script_string();
            script = script.replace(&placeholder, &value_str);
        }

        // Validate all placeholders were resolved
        if script.contains("#{") {
            return Err("Unresolved parameters in template".to_string());
        }
        if script.contains("@{") {
            return Err("Unresolved component references in template".to_string());
        }

        Ok(script)
    }

    /// Expand nested component references recursively
    fn expand_nested_components(&self, mut script: String, registry: &ComponentRegistry, depth: usize) -> Result<String, String> {
        const MAX_DEPTH: usize = 10;
        if depth >= MAX_DEPTH {
            return Err("Maximum nesting depth exceeded (possible cycle)".to_string());
        }

        // Find and expand all @{component_name} references
        while let Some(start_idx) = script.find("@{") {
            let remaining = &script[start_idx..];

            // Find matching closing brace
            if let Some(end_offset) = remaining.find('}') {
                let reference = &remaining[2..end_offset]; // Extract content between @{ and }
                let component_name = reference.trim();

                // Look up component in registry
                let component_def = registry.get(component_name)
                    .ok_or(format!("Component not found: {}", component_name))?;

                // Create instance with default parameters
                let nested_instance = ComponentInstance::new(component_def.clone());

                // Recursively expand the nested component
                let expanded_script = nested_instance.expand_nested_components(
                    component_def.script_template.clone(),
                    registry,
                    depth + 1
                )?;

                // Substitute nested component's parameters
                let mut nested_script = expanded_script;
                for (param_name, param_value) in &nested_instance.param_values {
                    let placeholder = format!("#{{{}}}",  param_name);
                    let value_str = param_value.to_script_string();
                    nested_script = nested_script.replace(&placeholder, &value_str);
                }

                // Replace the @{...} reference with the expanded script
                let full_reference = format!("@{{{}}}", component_name);
                script = script.replace(&full_reference, &nested_script);
            } else {
                return Err(format!("Unclosed @{{ reference at position {}", start_idx));
            }
        }

        Ok(script)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_component_instance_default_params() {
        let mut params = HashMap::new();
        params.insert("radius".to_string(), ParameterDef {
            param_type: ParamType::Float,
            default: ParamValue::Float(10.0),
            min: Some(1.0),
            max: Some(100.0),
            description: None,
        });

        let component_def = ComponentDef {
            name: "test_sphere".to_string(),
            category: "primitives".to_string(),
            description: "Test sphere".to_string(),
            parameters: params,
            script_template: "sphere(#{radius})".to_string(),
        };

        let instance = ComponentInstance::new(component_def);

        // Should have default value
        assert_eq!(instance.param_values.get("radius"), Some(&ParamValue::Float(10.0)));
    }

    #[test]
    fn test_generate_script_simple() {
        let mut params = HashMap::new();
        params.insert("radius".to_string(), ParameterDef {
            param_type: ParamType::Float,
            default: ParamValue::Float(10.0),
            min: None,
            max: None,
            description: None,
        });

        let component_def = ComponentDef {
            name: "test_sphere".to_string(),
            category: "primitives".to_string(),
            description: "Test sphere".to_string(),
            parameters: params,
            script_template: "sphere(#{radius})".to_string(),
        };

        let instance = ComponentInstance::new(component_def);
        let script = instance.generate_script().unwrap();

        assert_eq!(script, "sphere(10.0)");
    }

    #[test]
    fn test_generate_script_multiple_params() {
        let mut params = HashMap::new();
        params.insert("length".to_string(), ParameterDef {
            param_type: ParamType::Float,
            default: ParamValue::Float(60.0),
            min: None,
            max: None,
            description: None,
        });
        params.insert("diameter".to_string(), ParameterDef {
            param_type: ParamType::Float,
            default: ParamValue::Float(8.0),
            min: None,
            max: None,
            description: None,
        });

        let component_def = ComponentDef {
            name: "test_fuselage".to_string(),
            category: "aerospace".to_string(),
            description: "Test fuselage".to_string(),
            parameters: params,
            script_template: "fuselage_parametric(#{length}, #{diameter}, 0.7, 0.5)".to_string(),
        };

        let instance = ComponentInstance::new(component_def);
        let script = instance.generate_script().unwrap();

        assert_eq!(script, "fuselage_parametric(60.0, 8.0, 0.7, 0.5)");
    }

    #[test]
    fn test_validation_error() {
        let mut params = HashMap::new();
        params.insert("value".to_string(), ParameterDef {
            param_type: ParamType::Float,
            default: ParamValue::Float(10.0),
            min: Some(0.0),
            max: Some(20.0),
            description: None,
        });

        let component_def = ComponentDef {
            name: "test".to_string(),
            category: "test".to_string(),
            description: "Test".to_string(),
            parameters: params,
            script_template: "test(#{value})".to_string(),
        };

        let mut instance = ComponentInstance::new(component_def);
        instance.set_param("value".to_string(), ParamValue::Float(100.0));

        assert!(instance.generate_script().is_err());
    }
}
