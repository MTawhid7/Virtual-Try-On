// physics/sr/engine/config.rs

use glam::Vec3;

/// Global configuration for the physics simulation.
/// Controls solver quality/speed trade-offs and physical properties like gravity and stiffness.
#[derive(Clone, Debug)]
pub struct PhysicsConfig {
    // --- Simulation Quality ---
    /// Number of sub-steps per frame. Higher = More stable, Slower.
    pub substeps: usize,
    /// Number of solver iterations per sub-step. Higher = Stiffer constraints.
    pub solver_iterations: usize,
    /// Relaxation parameter (Successive Over-Relaxation). 1.0 = Gauss-Seidel. <1.0 = More stable.
    pub spectral_radius: f32,

    // --- Environment ---
    pub gravity: Vec3,
    pub wind: Vec3,
    /// Global Velocity Damping (0.0 to 1.0). Simulates air resistance.
    pub damping: f32,

    // --- Material Properties ---
    pub drag_coeff: f32,
    pub lift_coeff: f32,
    #[allow(dead_code)]
    pub density: f32,

    // --- Constraint Stiffness ---
    /// Compliance (inverse stiffness) for distance constraints.
    /// 0.0 = Infinite stiffness (jittery). Small value (e.g. 1e-5) = Stable.
    pub distance_compliance: f32,
    /// Resistance to area change (Shearing). Very low for cloth.
    pub area_compliance: f32,
    /// Distance between cloth layers or cloth/body.
    pub contact_thickness: f32,
    pub static_friction: f32,
    pub dynamic_friction: f32,
    pub collision_stiffness: f32,

    // --- Self-Collision ---
    /// Enable cloth self-collision detection
    pub self_collision_enabled: bool,
    /// Minimum separation between cloth layers (meters)
    pub self_collision_thickness: f32,
    /// Self-collision repulsion strength (0.0 - 1.0)
    pub self_collision_stiffness: f32,
    /// Solve self-collision every N substeps (performance optimization)
    pub self_collision_frequency: u8,
}

impl PhysicsConfig {
    pub fn default() -> Self {
        Self {
            // OPTIMIZED: 4 substeps x 4 iterations = 16 solves/frame (High Performance)
            // Reduced from 8 to restore FPS. "Soft Physics" handles stability.
            substeps: 8,
            solver_iterations: 8,

            // DISABLED: Chebyshev acceleration causes jitter in tight collisions.
            // 0.2 = Minimal acceleration (Safe). avoiding erratic updates in tight spots.
            spectral_radius: 0.5,

            gravity: Vec3::new(0.0, -9.81, 0.0),
            wind: Vec3::new(0.0, 0.0, 0.0),
            // Increased damping to kill "push-pull" vibrations (0.95 dissipates 5% energy/frame)
            damping: 0.99,

            // Reduced drag so the cloth falls naturally instead of floating
            drag_coeff: 0.5,
            lift_coeff: 0.05,
            density: 1.0,

            // STIFF: 1.0e-6 makes it very rigid (Denim/Leather).
            // Removes almost all rubbery feel.
            distance_compliance: 1.0e-7,
            area_compliance: 2.0e-4,

            contact_thickness: 0.005,
            static_friction: 0.3,
            dynamic_friction: 0.2,

            // SOFTENED: 0.5 reduces violent "kick-back" from body.
            collision_stiffness: 0.75,

            // Self-Collision: Enabled by default with balanced settings
            self_collision_enabled: true,
            self_collision_thickness: 0.005, // 5mm
            self_collision_stiffness: 0.5,
            self_collision_frequency: 2, // Every other substep
        }
    }
}
