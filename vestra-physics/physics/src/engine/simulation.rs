// physics/src/engine/simulation.rs

use crate::collision::self_collision::SelfCollisionConfig;
use crate::collision::{CollisionResolver, MeshCollider, SelfCollision};
use crate::engine::{PhysicsConfig, PhysicsState};
use crate::systems::constraints::MouseConstraint;
use crate::systems::dynamics::{Integrator, Solver};
use crate::systems::forces::Aerodynamics;
use crate::utils::normals;
use crate::utils::profiler::{ProfileCategory, Profiler};

/// The core physics simulation state and logic container.
/// Holds all subsystems (solver, collider, aerodynamics, etc.) and orchestrates the time step.
///
/// OPTIMIZATION: Uses SIMD-accelerated constraint solving.
pub struct Simulation {
    /// The raw particle data (Position, Velocity, Mass).
    pub state: PhysicsState,
    /// Configuration parameters for the solver.
    pub config: PhysicsConfig,
    /// The static collision geometry (e.g., the Mannequin).
    pub collider: MeshCollider,
    /// Handles collision detection and response.
    pub resolver: CollisionResolver,
    /// Solves internal constraints (Distance, Bending, etc.).
    pub solver: Solver,
    /// Calculates external wind/drag forces.
    pub aerodynamics: Aerodynamics,
    /// Handles user interaction (Mouse dragging).
    pub mouse: MouseConstraint,
    /// Handles cloth-on-cloth self-collision.
    pub self_collision: SelfCollision,
    /// Substep counter for reduced-frequency self-collision.
    substep_counter: u32,
    /// Frame counter for lazy updates (e.g. normals).
    frame_count: u32,
}

impl Simulation {
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
    ) -> Self {
        let state = PhysicsState::new(&garment_pos, &garment_indices, &garment_uvs);
        let particle_count = state.count;

        let config = PhysicsConfig::default();

        let collider = MeshCollider::new(
            collider_pos,
            collider_normals,
            collider_indices,
            collider_smoothing,
            collider_inflation,
        );

        let resolver = CollisionResolver::new(particle_count);
        let aerodynamics = Aerodynamics::new(particle_count);

        let solver = Solver::new(&state, scale_factor, config.distance_compliance);
        let mouse = MouseConstraint::new();

        let self_collision_config = SelfCollisionConfig {
            thickness: config.self_collision_thickness,
            stiffness: config.self_collision_stiffness,
            frequency: config.self_collision_frequency,
            max_pairs: 10000,
        };
        let self_collision = SelfCollision::new(&state, self_collision_config);

        Self {
            state,
            config,
            collider,
            resolver,
            solver,
            aerodynamics,
            mouse,
            self_collision,
            substep_counter: 0,
            frame_count: 0,
        }
    }

    /// Advances the simulation by `dt` seconds.
    /// Uses fixed sub-stepping with SIMD-accelerated constraint solving.
    ///
    /// PROFILING: Each phase is instrumented for performance analysis.
    pub fn step(&mut self, dt: f32) {
        Profiler::begin_frame();

        // Use fixed substeps from config (no adaptive)
        let sdt = dt / self.config.substeps as f32;

        // Broad-phase collision detection (once per frame)
        Profiler::start(ProfileCategory::BroadPhase);
        self.resolver.broad_phase(&self.state, &mut self.collider);
        Profiler::end(ProfileCategory::BroadPhase);

        // Narrow-phase collision detection (Once per frame)
        Profiler::start(ProfileCategory::NarrowPhase);
        self.resolver
            .narrow_phase(&mut self.state, &self.collider, &self.config, sdt);
        Profiler::end(ProfileCategory::NarrowPhase);

        // External forces (aerodynamics) - Decimated (Once per frame)
        Profiler::start(ProfileCategory::Aerodynamics);
        let forces = self.aerodynamics.apply(&self.state, &self.config, sdt);
        Profiler::end(ProfileCategory::Aerodynamics);

        for _ in 0..self.config.substeps {
            // Integration (updates positions based on velocity and forces)
            Profiler::start(ProfileCategory::Integration);
            Integrator::integrate(&mut self.state, &self.config, forces, sdt);
            Profiler::end(ProfileCategory::Integration);

            // Mouse interaction
            Profiler::start(ProfileCategory::MouseConstraint);
            self.mouse.solve(&mut self.state, sdt);
            Profiler::end(ProfileCategory::MouseConstraint);

            // SIMD-accelerated constraint solving
            Profiler::start(ProfileCategory::Constraints);
            self.solver
                .solve(&mut self.state, &self.resolver, &self.config, sdt);
            Profiler::end(ProfileCategory::Constraints);

            // Self-collision at reduced frequency for performance
            if self.config.self_collision_enabled {
                let freq = self.self_collision.config.frequency as u32;
                if freq == 0 || self.substep_counter % freq == 0 {
                    Profiler::start(ProfileCategory::SelfCollision);
                    self.self_collision.solve(&mut self.state);
                    Profiler::end(ProfileCategory::SelfCollision);
                }
            }
            self.substep_counter = self.substep_counter.wrapping_add(1);
        }

        // Compute vertex normals in WASM
        Profiler::start(ProfileCategory::Normals);
        normals::compute_vertex_normals(
            &self.state.positions,
            &self.state.indices,
            &mut self.state.normals,
        );
        Profiler::end(ProfileCategory::Normals);

        self.frame_count = self.frame_count.wrapping_add(1);

        Profiler::end_frame();
    }

    pub fn update_collider(&mut self, positions: &[f32]) {
        self.collider.update(positions);
    }
}
