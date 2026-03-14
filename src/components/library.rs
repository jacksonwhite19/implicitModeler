// Component library data structures

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Parameter type enumeration
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub enum ParamType {
    Float,
    Int,
    Bool,
    String,
}

/// Parameter value enumeration
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub enum ParamValue {
    Float(f64),
    Int(i64),
    Bool(bool),
    String(String),
}

impl ParamValue {
    /// Convert parameter value to Rhai script string
    pub fn to_script_string(&self) -> String {
        match self {
            ParamValue::Float(v) => {
                // Rhai requires floats to have a decimal point — "60" is INT, "60.0" is FLOAT
                if v.fract() == 0.0 {
                    format!("{:.1}", v)
                } else {
                    v.to_string()
                }
            }
            ParamValue::Int(v) => v.to_string(),
            ParamValue::Bool(v) => v.to_string(),
            ParamValue::String(v) => format!("\"{}\"", v),
        }
    }

    /// Get the parameter type of this value
    pub fn param_type(&self) -> ParamType {
        match self {
            ParamValue::Float(_) => ParamType::Float,
            ParamValue::Int(_) => ParamType::Int,
            ParamValue::Bool(_) => ParamType::Bool,
            ParamValue::String(_) => ParamType::String,
        }
    }
}

/// Parameter definition with metadata and validation rules
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ParameterDef {
    pub param_type: ParamType,
    pub default: ParamValue,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}

impl ParameterDef {
    /// Validate a parameter value against this definition
    pub fn validate(&self, value: &ParamValue) -> Result<(), String> {
        // Type check
        if value.param_type() != self.param_type {
            return Err(format!("Type mismatch: expected {:?}, got {:?}",
                             self.param_type, value.param_type()));
        }

        // Range validation for numeric types
        match value {
            ParamValue::Float(v) => {
                if let Some(min) = self.min {
                    if *v < min {
                        return Err(format!("Value {} is below minimum {}", v, min));
                    }
                }
                if let Some(max) = self.max {
                    if *v > max {
                        return Err(format!("Value {} is above maximum {}", v, max));
                    }
                }
            }
            ParamValue::Int(v) => {
                if let Some(min) = self.min {
                    if (*v as f64) < min {
                        return Err(format!("Value {} is below minimum {}", v, min));
                    }
                }
                if let Some(max) = self.max {
                    if (*v as f64) > max {
                        return Err(format!("Value {} is above maximum {}", v, max));
                    }
                }
            }
            _ => {}
        }

        Ok(())
    }
}

/// Component definition with parameters and script template
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ComponentDef {
    pub name: String,
    pub category: String,
    pub description: String,
    #[serde(default)]
    pub parameters: HashMap<String, ParameterDef>,
    pub script_template: String,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_param_value_to_script_string() {
        assert_eq!(ParamValue::Float(5.5).to_script_string(), "5.5");
        assert_eq!(ParamValue::Int(42).to_script_string(), "42");
        assert_eq!(ParamValue::Bool(true).to_script_string(), "true");
        assert_eq!(ParamValue::String("test".to_string()).to_script_string(), "\"test\"");
    }

    #[test]
    fn test_parameter_validation_type() {
        let param_def = ParameterDef {
            param_type: ParamType::Float,
            default: ParamValue::Float(10.0),
            min: Some(0.0),
            max: Some(100.0),
            description: None,
        };

        assert!(param_def.validate(&ParamValue::Float(50.0)).is_ok());
        assert!(param_def.validate(&ParamValue::Int(50)).is_err());
    }

    #[test]
    fn test_parameter_validation_range() {
        let param_def = ParameterDef {
            param_type: ParamType::Float,
            default: ParamValue::Float(10.0),
            min: Some(5.0),
            max: Some(15.0),
            description: None,
        };

        assert!(param_def.validate(&ParamValue::Float(10.0)).is_ok());
        assert!(param_def.validate(&ParamValue::Float(3.0)).is_err());
        assert!(param_def.validate(&ParamValue::Float(20.0)).is_err());
    }

    #[test]
    fn test_component_def_serialization() {
        let mut params = HashMap::new();
        params.insert("length".to_string(), ParameterDef {
            param_type: ParamType::Float,
            default: ParamValue::Float(60.0),
            min: Some(30.0),
            max: Some(120.0),
            description: Some("Total length".to_string()),
        });

        let component = ComponentDef {
            name: "test_component".to_string(),
            category: "test".to_string(),
            description: "Test component".to_string(),
            parameters: params,
            script_template: "sphere(#{length})".to_string(),
        };

        // Test JSON round-trip
        let json = serde_json::to_string_pretty(&component).unwrap();
        let deserialized: ComponentDef = serde_json::from_str(&json).unwrap();

        assert_eq!(component.name, deserialized.name);
        assert_eq!(component.category, deserialized.category);
        assert_eq!(component.parameters.len(), deserialized.parameters.len());
    }
}
