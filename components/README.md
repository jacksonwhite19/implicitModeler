# Component Library Documentation

## Overview

The component library system allows you to create reusable, parametric geometric building blocks. Components are defined as JSON files with parameter definitions and Rhai script templates.

## Component JSON Format

Each component is a JSON file with the following structure:

```json
{
  "name": "component_name",
  "category": "category_name",
  "description": "Human-readable description",
  "parameters": {
    "param_name": {
      "param_type": "Float | Int | Bool | String",
      "default": {"Float": 10.0} | {"Int": 5} | {"Bool": true} | {"String": "value"},
      "min": 0.0,
      "max": 100.0,
      "description": "Parameter description"
    }
  },
  "script_template": "Rhai script with #{param_name} placeholders"
}
```

### Fields

- **name**: Unique identifier for the component (lowercase, underscores)
- **category**: Organizational category (aerospace, primitives, field_ops, etc.)
- **description**: Brief description shown in UI tooltips
- **parameters**: Map of parameter definitions (can be empty)
- **script_template**: Rhai script with parameter placeholders

## Parameter Types

### Float
```json
{
  "param_type": "Float",
  "default": {"Float": 10.0},
  "min": 1.0,
  "max": 100.0,
  "description": "Numeric parameter with decimal precision"
}
```

### Int
```json
{
  "param_type": "Int",
  "default": {"Int": 5},
  "min": 0.0,
  "max": 10.0,
  "description": "Integer parameter"
}
```

### Bool
```json
{
  "param_type": "Bool",
  "default": {"Bool": true},
  "description": "Boolean toggle"
}
```

### String
```json
{
  "param_type": "String",
  "default": {"String": "default_value"},
  "description": "Text parameter"
}
```

## Template Syntax

### Parameter Substitution

Use `#{param_name}` to reference parameters:

```rhai
let fuselage = fuselage_parametric(#{length}, #{diameter}, 0.7, 0.5);
fuselage
```

When instantiated with parameters, this becomes:

```rhai
let fuselage = fuselage_parametric(60, 8, 0.7, 0.5);
fuselage
```

### Complete Statements Required

Templates must be valid, complete Rhai statements. End with a return value:

**Good:**
```rhai
let result = sphere(#{radius});
result
```

**Bad:**
```rhai
sphere(#{radius})  // Missing assignment and return
```

### Nested Components (Composition)

Reference other components with `@{component_name}`:

```rhai
let body = @{fuselage_v1};
let wing = @{wing_swept};
let aircraft = union(body, wing);
aircraft
```

Nested references are recursively expanded before parameter substitution.

## Creating Custom Components

### Method 1: Save from UI

1. Write your script in the editor
2. Use `#{param_name}` for values you want to parameterize
3. Click "📦 Save as Component..."
4. Fill in name, category, and description
5. Component is saved to `components/{category}/{name}.json`
6. Edit the JSON file to add parameter definitions

### Method 2: Manual JSON Creation

1. Create a new `.json` file in `components/{category}/`
2. Follow the JSON format above
3. Define parameters with types, defaults, and ranges
4. Write template with `#{param}` placeholders
5. Restart app to load new component

## Example: Simple Sphere Component

```json
{
  "name": "custom_sphere",
  "category": "primitives",
  "description": "Customizable sphere",
  "parameters": {
    "radius": {
      "param_type": "Float",
      "default": {"Float": 10.0},
      "min": 1.0,
      "max": 50.0,
      "description": "Sphere radius"
    }
  },
  "script_template": "let s = sphere(#{radius});\ns"
}
```

## Example: Multi-Parameter Component

```json
{
  "name": "fuselage_v1",
  "category": "aerospace",
  "description": "Parametric fuselage with elliptical cross-section",
  "parameters": {
    "length": {
      "param_type": "Float",
      "default": {"Float": 60.0},
      "min": 30.0,
      "max": 120.0,
      "description": "Total fuselage length"
    },
    "diameter": {
      "param_type": "Float",
      "default": {"Float": 8.0},
      "min": 2.0,
      "max": 20.0,
      "description": "Maximum diameter"
    },
    "nose_ratio": {
      "param_type": "Float",
      "default": {"Float": 0.7},
      "min": 0.3,
      "max": 1.0,
      "description": "Nose taper ratio"
    },
    "tail_ratio": {
      "param_type": "Float",
      "default": {"Float": 0.5},
      "min": 0.3,
      "max": 1.0,
      "description": "Tail taper ratio"
    }
  },
  "script_template": "let fuselage = fuselage_parametric(#{length}, #{diameter}, #{nose_ratio}, #{tail_ratio});\nfuselage"
}
```

## Example: Nested Component Assembly

```json
{
  "name": "simple_aircraft",
  "category": "aerospace",
  "description": "Basic aircraft assembly",
  "parameters": {},
  "script_template": "let body = @{fuselage_v1};\nlet wing = @{wing_swept};\nlet engine = @{nacelle_turbofan};\nlet aircraft = union(body, union(wing, engine));\naircraft"
}
```

## Best Practices

1. **Naming**: Use lowercase with underscores (e.g., `wing_swept`, `fuselage_v1`)
2. **Categories**: Group related components (aerospace, primitives, mechanical, etc.)
3. **Descriptions**: Write clear, concise descriptions for UI tooltips
4. **Parameter Ranges**: Set sensible min/max bounds for UI sliders
5. **Default Values**: Choose defaults that produce visible, valid geometry
6. **Complete Scripts**: Always end templates with a return value
7. **Testing**: Test component in UI before sharing

## Directory Structure

```
components/
├── aerospace/
│   ├── fuselage_v1.json
│   ├── wing_swept.json
│   └── nacelle_turbofan.json
├── primitives/
│   ├── rounded_box.json
│   └── chamfered_cylinder.json
├── field_ops/
│   ├── gyroid_shell.json
│   └── radial_taper.json
└── custom/
    └── (your components)
```

## Troubleshooting

### Component Not Loading
- Check JSON syntax with a validator
- Verify file has `.json` extension
- Check console output for parsing errors
- Ensure `param_type` matches `default` type

### Script Syntax Error
- Template must be complete Rhai statement
- End with variable name or explicit value
- Use `let result = ...;\nresult` pattern

### Parameter Not Substituting
- Verify exact spelling in `#{param_name}`
- Parameter must be defined in `parameters` map
- Check that placeholder uses `#{}` not `@{}`

### Nested Component Not Found
- Referenced component must exist in registry
- Use exact component name: `@{component_name}`
- Check console for "Component not found" errors

## Advanced Features

### Cycle Detection
Maximum nesting depth is 10 levels to prevent infinite recursion.

### Parameter Validation
- Float/Int ranges enforced (min/max)
- Type mismatches detected
- Unknown parameters rejected

### Script Generation
1. Nested components expanded recursively
2. Parameters substituted with values
3. Result validated for unresolved placeholders

## API Reference

### ComponentInstance

```rust
// Create with defaults
let instance = ComponentInstance::new(component_def);

// Set custom parameter
instance.set_param("radius".to_string(), ParamValue::Float(15.0));

// Generate script (no nesting)
let script = instance.generate_script()?;

// Generate script with nested expansion
let script = instance.generate_script_with_nesting(&registry)?;
```

### ComponentRegistry

```rust
// Load components from directory
let mut registry = ComponentRegistry::new();
registry.load_from_directory("components")?;

// Get component by name
if let Some(component) = registry.get("fuselage_v1") {
    // Use component...
}

// List categories
for category in registry.list_categories() {
    // Browse category...
}

// Get components in category
let aerospace = registry.list_by_category("aerospace");
```

## License

Components are MIT licensed. Feel free to share and modify!
