// physics/src/collision/resolver/mod.rs
mod broad;
mod narrow;

use super::collider::MeshCollider;
use crate::engine::config::PhysicsConfig;
use crate::engine::state::PhysicsState;
use glam::{Vec3, Vec4};

/// Represents a single point of contact between a particle and a collider.
#[derive(Clone, Copy)]
pub struct Contact {
    pub particle_index: usize,
    pub normal: Vec3,
    pub surface_point: Vec3,
}

/// Manages collision detection and resolution.
/// Stores contact constraints and solver buffers to avoid per-frame allocations.
pub struct CollisionResolver {
    pub(crate) contacts: Vec<Contact>,

    // Spatial Hashing Buffers
    #[allow(dead_code)]
    pub(crate) query_buffer: Vec<usize>,
    pub(crate) candidate_indices: Vec<usize>,
    pub(crate) candidate_offsets: Vec<usize>,
    pub(crate) candidate_counts: Vec<usize>,
}

impl CollisionResolver {
    pub fn new(particle_count: usize) -> Self {
        let estimated_candidates = particle_count * 100;
        Self {
            contacts: Vec::with_capacity(particle_count),
            query_buffer: Vec::with_capacity(256),
            candidate_indices: Vec::with_capacity(estimated_candidates),
            candidate_offsets: vec![0; particle_count],
            candidate_counts: vec![0; particle_count],
        }
    }

    pub fn broad_phase(&mut self, state: &PhysicsState, collider: &mut MeshCollider) {
        broad::perform_broad_phase(self, state, collider);
    }

    pub fn narrow_phase(
        &mut self,
        state: &mut PhysicsState,
        collider: &MeshCollider,
        config: &PhysicsConfig,
        dt: f32,
    ) {
        narrow::perform_narrow_phase(self, state, collider, config, dt);
    }

    // FIX: Removed 'omega' parameter. Collisions are NOT accelerated.
    pub fn resolve_contacts(&self, state: &mut PhysicsState, config: &PhysicsConfig, _dt: f32) {
        for contact in &self.contacts {
            let i = contact.particle_index;
            let pos_v4 = state.positions[i];
            let pos = pos_v4.truncate();
            let normal = contact.normal;
            let surface_point = contact.surface_point;

            let vec = pos - surface_point;
            let projection = vec.dot(normal);

            if projection < config.contact_thickness {
                let penetration = config.contact_thickness - projection;
                let stiffness = if projection < 0.0 {
                    1.0
                } else {
                    config.collision_stiffness
                };

                // FIX: No omega multiplication here.
                let correction = normal * (penetration * stiffness);
                state.positions[i] += Vec4::from((correction, 0.0));

                let prev_v4 = state.prev_positions[i];
                let current_pos_v3 = state.positions[i].truncate();
                let prev_pos_v3 = prev_v4.truncate();

                let velocity = current_pos_v3 - prev_pos_v3;
                let vn_mag = velocity.dot(normal);
                let vn = normal * vn_mag;
                let vt = velocity - vn;
                let vt_len = vt.length();

                let mut friction_factor = 0.0;
                if vt_len > 1e-9 {
                    if vt_len < penetration * config.static_friction {
                        friction_factor = 1.0;
                    } else {
                        let max_slide = penetration * config.dynamic_friction;
                        friction_factor = max_slide / vt_len;
                        if friction_factor > 1.0 {
                            friction_factor = 1.0;
                        }
                    }
                }

                let new_vt = vt * (1.0 - friction_factor);
                let new_vn = if vn_mag < 0.0 { Vec3::ZERO } else { vn };

                let total_correction = new_vn + new_vt;
                state.prev_positions[i] = state.positions[i] - Vec4::from((total_correction, 0.0));
            }
        }
    }
}
