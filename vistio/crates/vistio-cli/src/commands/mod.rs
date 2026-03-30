//! CLI command implementations.
//!
//! Each command lives in its own submodule for maintainability.

mod benchmark;
mod inspect;
mod validate;
mod visualize;

pub use benchmark::benchmark;
pub use inspect::inspect;
pub use validate::validate;
pub use visualize::visualize;

/// Run a simulation from config file.
pub fn simulate(config_path: &str) -> Result<(), Box<dyn std::error::Error>> {
    println!("Vistio Simulation");
    println!("─────────────────");
    println!("Config: {config_path}");
    println!();
    println!("Note: TOML config loading is a Tier-1 feature.");
    println!("Use `vistio benchmark` to run procedural scenarios now.");
    Ok(())
}
