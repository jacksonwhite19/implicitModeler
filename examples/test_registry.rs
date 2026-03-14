use implicit_cad::components::ComponentRegistry;

fn main() {
    let mut registry = ComponentRegistry::new();
    
    println!("Current directory: {:?}", std::env::current_dir());
    
    match registry.load_from_directory("components") {
        Ok(count) => {
            println!("✓ Loaded {} components", count);
            
            for category in registry.list_categories() {
                println!("\nCategory: {}", category);
                for comp in registry.list_by_category(&category) {
                    println!("  - {} ({})", comp.name, comp.description);
                }
            }
        }
        Err(e) => {
            println!("✗ Error: {}", e);
        }
    }
}
