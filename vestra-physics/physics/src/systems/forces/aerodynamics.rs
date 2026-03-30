// physics/src/system/forces/aerodynamics.rs

use glam::Vec3;
use crate::engine::state::PhysicsState;
use crate::engine::config::PhysicsConfig;

/// Calculates aerodynamic forces (Lift and Drag) acting on the garment.
/// Uses a simplified triangle-based model assuming a uniform wind vector.
pub struct Aerodynamics {
    force_buffer: Vec<Vec3>,
}

impl Aerodynamics {
    pub fn new(particle_count: usize) -> Self {
        Self {
            // Eagerly allocate the entire buffer to prevent runtime resizing
            force_buffer: vec![Vec3::ZERO; particle_count],
        }
    }

    /// Computes lift and drag for every triangle and distributes the force to vertices.
    /// - Uses the relative velocity between the wind and the triangle's surface.
    /// - `Drag`: Resistance parallel to airflow.
    /// - `Lift`: Force perpendicular to airflow (Bernoulli principle).
    pub fn apply(&mut self, state: &PhysicsState, config: &PhysicsConfig, dt: f32) -> &Vec<Vec3> {
        // Zero out the buffer using a fast fill instead of clearing/pushing
        self.force_buffer.fill(Vec3::ZERO);

        let num_triangles = state.indices.len() / 3;
        for i in 0..num_triangles {
            let idx0 = state.indices[i * 3] as usize;
            let idx1 = state.indices[i * 3 + 1] as usize;
            let idx2 = state.indices[i * 3 + 2] as usize;

            let p0 = state.positions[idx0].truncate();
            let p1 = state.positions[idx1].truncate();
            let p2 = state.positions[idx2].truncate();

            let v0 = (p0 - state.prev_positions[idx0].truncate()) / dt;
            let v1 = (p1 - state.prev_positions[idx1].truncate()) / dt;
            let v2 = (p2 - state.prev_positions[idx2].truncate()) / dt;
            let tri_vel = (v0 + v1 + v2) / 3.0;

            let rel_vel: Vec3 = tri_vel - config.wind;
            let vel_sq = rel_vel.length_squared();

            if vel_sq < 1e-6 { continue; }

            let edge1 = p1 - p0;
            let edge2 = p2 - p0;
            let cross = edge1.cross(edge2);
            let area_x2 = cross.length();

            if area_x2 < 1e-6 { continue; }

            let area = area_x2 * 0.5;
            let normal = cross / area_x2;

            let v_dot_n = rel_vel.dot(normal);
            let v_normal = normal * v_dot_n;
            let v_tangent = rel_vel - v_normal;

            let f_drag = -0.5 * config.drag_coeff * area * v_normal.length() * v_normal;
            let f_lift = -0.5 * config.lift_coeff * area * v_tangent.length() * v_tangent;

            let total_force = f_drag + f_lift;

            let force_per_vert = total_force / 3.0;
            self.force_buffer[idx0] += force_per_vert;
            self.force_buffer[idx1] += force_per_vert;
            self.force_buffer[idx2] += force_per_vert;
        }

        &self.force_buffer
    }
}