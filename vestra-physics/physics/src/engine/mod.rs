// physics/src/engine/mod.rs
pub mod config;
pub mod state;
pub mod simulation;

pub use config::PhysicsConfig;
pub use state::PhysicsState;
pub use simulation::Simulation;