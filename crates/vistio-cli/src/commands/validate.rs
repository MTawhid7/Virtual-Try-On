//! `vistio validate` command implementation.

/// Validate a mesh or config.
pub fn validate(path: &str) -> Result<(), Box<dyn std::error::Error>> {
    println!("Vistio Validator");
    println!("────────────────");
    println!();

    if path.ends_with(".toml") {
        println!("Validating config: {path}");
        let content = std::fs::read_to_string(path)?;
        let _config: vistio_solver::SolverConfig = toml::from_str(&content)?;
        println!("✅ Config is valid.");
    } else if path.ends_with(".json") {
        println!("Validating mesh: {path}");
        let content = std::fs::read_to_string(path)?;
        let mesh: vistio_mesh::TriangleMesh = serde_json::from_str(&content)?;
        match mesh.validate() {
            Ok(()) => println!("✅ Mesh is valid ({} verts, {} tris).", mesh.vertex_count(), mesh.triangle_count()),
            Err(e) => println!("❌ Mesh validation failed: {e}"),
        }
    } else {
        println!("Unsupported file format. Use .toml (config) or .json (mesh).");
    }

    Ok(())
}
