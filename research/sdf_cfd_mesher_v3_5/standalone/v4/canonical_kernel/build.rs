use sha2::{Digest, Sha256};
use std::{fs, path::Path};

fn walk(p: &Path, paths: &mut Vec<std::path::PathBuf>) {
    for entry in fs::read_dir(p).unwrap() {
        let p = entry.unwrap().path();
        if p.is_dir() {
            walk(&p, paths);
        } else if p.extension().is_some_and(|s| s == "rs" || s == "wgsl") {
            paths.push(p);
        }
    }
}

fn main() {
    let root = Path::new("../../kernel").canonicalize().unwrap();
    println!("cargo:rerun-if-changed={}", root.join("src").display());
    println!("cargo:rerun-if-changed={}", root.join("shaders").display());
    let mut paths = vec![root.join("Cargo.toml"), root.join("Cargo.lock")];
    walk(&root.join("src"), &mut paths);
    walk(&root.join("shaders"), &mut paths);
    paths.sort();
    let mut h = Sha256::new();
    for p in paths {
        println!("cargo:rerun-if-changed={}", p.display());
        let name = p
            .strip_prefix(&root)
            .unwrap()
            .to_string_lossy()
            .replace('\\', "/");
        let bytes = fs::read(&p).unwrap();
        h.update(name.as_bytes());
        h.update([0]);
        h.update((bytes.len() as u64).to_le_bytes());
        h.update(bytes);
    }
    // Bind this research wrapper as well as the unchanged compiled production dependency.
    let mut host_paths=vec![Path::new("Cargo.toml").to_path_buf(),Path::new("Cargo.lock").to_path_buf(),Path::new("build.rs").to_path_buf()];
    walk(Path::new("src"), &mut host_paths); host_paths.sort();
    for p in host_paths {
        println!("cargo:rerun-if-changed={}",p.display());
        let bytes=fs::read(&p).unwrap(); h.update(b"research_native172/");
        h.update(p.to_string_lossy().replace('\\',"/").as_bytes());h.update([0]);
        h.update((bytes.len() as u64).to_le_bytes());h.update(bytes);
    }
    println!("cargo:rustc-env=V4_KERNEL_SOURCE_SHA256={:x}", h.finalize());
}
