use std::path::Path;

fn main() {
    println!("Current directory: {:?}", std::env::current_dir());
    
    let paths = vec!["components", "./components", "../components"];
    
    for p in paths {
        println!("\nChecking: {}", p);
        if Path::new(p).exists() {
            println!("  ✓ Exists");
            if Path::new(p).is_dir() {
                println!("  ✓ Is directory");
                match std::fs::read_dir(p) {
                    Ok(entries) => {
                        let count: usize = entries.count();
                        println!("  ✓ Contains {} entries", count);
                    }
                    Err(e) => println!("  ✗ Error reading: {}", e),
                }
            }
        } else {
            println!("  ✗ Does not exist");
        }
    }
}
