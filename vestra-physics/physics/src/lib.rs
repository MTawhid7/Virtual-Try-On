// physics/src/lib.rs
pub mod collision;
pub mod engine;
pub mod systems;
pub mod utils;

use engine::Simulation;
use utils::profiler::Profiler;
use wasm_bindgen::prelude::*;

// Re-export profiler WASM functions for direct access
// pub use utils::profiler::{profiler_get_report, profiler_reset, profiler_set_enabled};

#[cfg(feature = "parallel")]
pub use wasm_bindgen_rayon::init_thread_pool;

/// The main WASM entry point for the physics engine.
/// Wraps the internal Rust Simulation struct and exposes a JS-friendly API.
#[wasm_bindgen]
pub struct PhysicsEngine {
    sim: Simulation,
}

#[wasm_bindgen]
impl PhysicsEngine {
    #[wasm_bindgen(constructor)]
    pub fn new(
        garment_pos: Vec<f32>,
        garment_indices: Vec<u32>,
        garment_uvs: Vec<f32>,
        collider_pos: Vec<f32>,
        collider_normals: Vec<f32>,
        collider_indices: Vec<u32>,
        collider_smoothing: usize,
        collider_inflation: f32,
        scale_factor: f32,
    ) -> PhysicsEngine {
        utils::set_panic_hook();

        PhysicsEngine {
            sim: Simulation::new(
                garment_pos,
                garment_indices,
                garment_uvs,
                collider_pos,
                collider_normals,
                collider_indices,
                collider_smoothing,
                collider_inflation,
                scale_factor,
            ),
        }
    }

    pub fn step(&mut self, dt: f32) {
        self.sim.step(dt);
    }

    /// Updates the collider mesh with new vertex positions (e.g. for animation).
    /// PROFILING: This involves re-processing the mesh (smoothing) and rebuilding the spatial hash.
    pub fn update_collider(&mut self, positions: Vec<f32>) {
        self.sim.update_collider(&positions);
    }

    /// Returns a pointer to the positions buffer.
    /// Layout: [x, y, z, w, x, y, z, w...] (Stride = 4 floats)
    pub fn get_positions_ptr(&self) -> *const f32 {
        self.sim.state.positions[0].as_ref().as_ptr()
    }

    /// Returns a pointer to the normals buffer.
    /// Layout: [x, y, z, w, x, y, z, w...] (Stride = 4 floats)
    pub fn get_normals_ptr(&self) -> *const f32 {
        self.sim.state.normals[0].as_ref().as_ptr()
    }

    // --- Interaction Methods ---

    pub fn set_interaction(&mut self, index: usize, x: f32, y: f32, z: f32) {
        self.sim.mouse.grab(index, glam::Vec3::new(x, y, z));
    }

    pub fn update_interaction(&mut self, x: f32, y: f32, z: f32) {
        self.sim.mouse.update_target(glam::Vec3::new(x, y, z));
    }

    pub fn end_interaction(&mut self) {
        self.sim.mouse.release();
    }

    // --- Profiling Methods ---

    /// Returns the profiling report as a JSON string.
    /// Contains timing data for all measured categories.
    pub fn get_profile_report(&self) -> String {
        Profiler::get_report_json()
    }

    /// Resets all profiling statistics.
    pub fn reset_profiler(&self) {
        Profiler::reset();
    }

    /// Enables or disables profiling.
    /// When disabled, profiling calls have near-zero overhead.
    pub fn set_profiler_enabled(&self, enabled: bool) {
        Profiler::set_enabled(enabled);
    }

    /// Returns the particle count for diagnostic purposes.
    pub fn get_particle_count(&self) -> usize {
        self.sim.state.count
    }

    /// Returns the number of substeps per frame.
    pub fn get_substeps(&self) -> usize {
        self.sim.config.substeps
    }

    /// Returns the number of solver iterations per substep.
    pub fn get_solver_iterations(&self) -> usize {
        self.sim.config.solver_iterations
    }

    // --- Tuning Methods ---

    pub fn set_substeps(&mut self, substeps: usize) {
        self.sim.config.substeps = substeps;
    }

    pub fn set_solver_iterations(&mut self, iterations: usize) {
        self.sim.config.solver_iterations = iterations;
    }

    pub fn set_self_collision_enabled(&mut self, enabled: bool) {
        self.sim.config.self_collision_enabled = enabled;
    }
}
