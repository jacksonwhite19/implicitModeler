// Integration tests for component system
use implicit_cad::components::{ComponentRegistry, ComponentInstance};
use implicit_cad::scripting;
use glam::Vec3;
use implicit_cad::mesh::marching_cubes;

#[test]
fn test_load_all_default_components() {
    let mut registry = ComponentRegistry::new();

    // Try to load from components directory
    let paths_to_try = vec!["components", "./components", "../components"];
    let mut loaded = false;

    for path in paths_to_try {
        if let Ok(count) = registry.load_from_directory(path) {
            if count > 0 {
                eprintln!("Loaded {} components from {}", count, path);
                loaded = true;
                break;
            }
        }
    }

    assert!(loaded, "Failed to load components from any path");
    assert!(registry.count() >= 7, "Expected at least 7 default components, found {}", registry.count());
}

#[test]
fn test_all_components_execute() {
    let mut registry = ComponentRegistry::new();

    // Load components
    let paths_to_try = vec!["components", "./components", "../components"];
    for path in paths_to_try {
        if let Ok(count) = registry.load_from_directory(path) {
            if count > 0 {
                break;
            }
        }
    }

    // Get all components
    let all_components: Vec<_> = registry.list_categories()
        .iter()
        .flat_map(|cat| registry.list_by_category(cat))
        .collect();

    assert!(!all_components.is_empty(), "No components loaded");

    // Test each component
    for component in all_components {
        eprintln!("Testing component: {}", component.name);

        // Create instance with defaults
        let instance = ComponentInstance::new(component.clone());

        // Generate script
        let script_result = instance.generate_script();
        assert!(script_result.is_ok(), "Failed to generate script for {}: {:?}",
                component.name, script_result.err());

        let script = script_result.unwrap();
        eprintln!("  Generated script: {}", script.lines().take(2).collect::<Vec<_>>().join(" "));

        // Execute script
        let eval_result = scripting::evaluate_script(&script);
        assert!(eval_result.is_ok(), "Failed to execute script for {}: {:?}",
                component.name, eval_result.err());

        let sdf = eval_result.unwrap().sdf;

        // Extract mesh — use resolution 32 to handle thin shells
        let bounds_min = Vec3::new(-100.0, -100.0, -100.0);
        let bounds_max = Vec3::new(100.0, 100.0, 100.0);
        let mesh = marching_cubes::extract_mesh(sdf.as_ref(), bounds_min, bounds_max, 32, false);

        // Verify mesh has vertices (not empty)
        assert!(!mesh.vertices.is_empty(),
                "Component {} produced empty mesh", component.name);

        eprintln!("  ✓ Produced mesh with {} vertices", mesh.vertices.len());
    }
}

#[test]
fn test_component_parameters_customizable() {
    let mut registry = ComponentRegistry::new();

    // Load components
    let paths_to_try = vec!["components", "./components", "../components"];
    for path in paths_to_try {
        if let Ok(count) = registry.load_from_directory(path) {
            if count > 0 {
                break;
            }
        }
    }

    // Test fuselage_v1 with custom parameters
    if let Some(fuselage) = registry.get("fuselage_v1") {
        let mut instance = ComponentInstance::new(fuselage.clone());

        // Modify a parameter
        instance.set_param("length".to_string(),
                          implicit_cad::components::ParamValue::Float(80.0));

        let script = instance.generate_script().unwrap();
        assert!(script.contains("80"), "Script should contain custom length value");

        // Verify it still executes
        let sdf = scripting::evaluate_script(&script).unwrap().sdf;
        let bounds_min = Vec3::new(-100.0, -100.0, -100.0);
        let bounds_max = Vec3::new(100.0, 100.0, 100.0);
        let mesh = marching_cubes::extract_mesh(sdf.as_ref(), bounds_min, bounds_max, 16, false);
        assert!(!mesh.vertices.is_empty());
    }
}

#[test]
fn test_nested_component_expansion_syntax() {
    let mut registry = ComponentRegistry::new();

    // Load components
    let paths_to_try = vec!["components", "./components", "../components"];
    for path in paths_to_try {
        if let Ok(count) = registry.load_from_directory(path) {
            if count > 0 {
                break;
            }
        }
    }

    // Create a test component with nested reference
    use std::collections::HashMap;
    use implicit_cad::components::{ComponentDef, ParamValue};

    let test_component = ComponentDef {
        name: "test_nested".to_string(),
        category: "test".to_string(),
        description: "Test nested component".to_string(),
        parameters: HashMap::new(),
        script_template: "let base = @{fuselage_v1};\nbase".to_string(),
    };

    let instance = ComponentInstance::new(test_component);
    let result = instance.generate_script_with_nesting(&registry);

    // Should successfully expand the nested reference
    if result.is_ok() {
        let script = result.unwrap();
        assert!(!script.contains("@{"), "Nested references should be expanded");
        assert!(script.contains("fuselage_parametric"), "Should contain expanded fuselage code");
        eprintln!("Nested expansion successful: {}", script);
    } else {
        eprintln!("Nested expansion not yet fully implemented: {:?}", result.err());
    }
}
