// physics-core/src/lib.rs
pub mod utils;
pub mod data;        // <--- Changed to pub mod
pub mod constraints; // <--- Changed to pub mod
pub mod collider;    // <--- Changed to pub mod

use wasm_bindgen::prelude::*;
use data::PhysicsData;
use constraints::distance::DistanceConstraint;
use constraints::bending::BendingConstraint;
use constraints::tether::TetherConstraint;
use collider::sdf::SDFCollider;

#[wasm_bindgen]
pub fn init_hooks() {
    console_error_panic_hook::set_once();
}

#[wasm_bindgen]
pub struct Simulation {
    data: PhysicsData,
    distance_constraint: DistanceConstraint,
    bending_constraint: BendingConstraint,
    tether_constraint: Option<TetherConstraint>,
    sdf_collider: SDFCollider,
}

#[wasm_bindgen]
impl Simulation {
    #[wasm_bindgen(constructor)]
    pub fn new(positions: Vec<f32>, indices: Vec<u32>) -> Simulation {
        let data = PhysicsData::new(positions, indices);
        let distance_constraint = DistanceConstraint::new(&data);
        let bending_constraint = BendingConstraint::new(&data);
        let sdf_collider = SDFCollider::new();

        Simulation {
            data,
            distance_constraint,
            bending_constraint,
            tether_constraint: None,
            sdf_collider,
        }
    }

    // NEW: Pin vertices near the top to act like a hanger
    pub fn pin_collar(&mut self, threshold: f32) {
        let mut max_y = f32::MIN;
        // 1. Find the highest point
        for i in 0..self.data.count {
            let y = self.data.positions[i * 3 + 1];
            if y > max_y { max_y = y; }
        }

        // 2. Pin everything within 'threshold' of the top
        let cutoff = max_y - threshold;
        let mut pinned_count = 0;

        for i in 0..self.data.count {
            let y = self.data.positions[i * 3 + 1];
            if y > cutoff {
                self.data.inv_mass[i] = 0.0; // Infinite mass = Pinned
                pinned_count += 1;
            }
        }
        crate::console_log!("[Rust] Pinned {} vertices at collar (Y > {:.2})", pinned_count, cutoff);
    }

    // NEW: Call this from JS after pin_collar()
    pub fn init_tethers(&mut self) {
        self.tether_constraint = Some(TetherConstraint::new(&self.data));
    }

    pub fn step(&mut self, dt: f32) {
        let gravity = -9.81;
        let drag = 0.98;
        let substeps = 25;
        let sdt = dt / substeps as f32;

        for _ in 0..substeps {
            // Integrate
            for i in 0..self.data.count {
                if self.data.inv_mass[i] == 0.0 { continue; }
                let idx = i * 3;
                let px = self.data.positions[idx];
                let py = self.data.positions[idx+1];
                let pz = self.data.positions[idx+2];
                let prev_x = self.data.prev_positions[idx];
                let prev_y = self.data.prev_positions[idx+1];
                let prev_z = self.data.prev_positions[idx+2];

                let vel_x = (px - prev_x) * drag;
                let mut vel_y = (py - prev_y) * drag;
                let vel_z = (pz - prev_z) * drag;

                vel_y += gravity * sdt * sdt;

                self.data.prev_positions[idx] = px;
                self.data.prev_positions[idx+1] = py;
                self.data.prev_positions[idx+2] = pz;
                self.data.positions[idx] += vel_x;
                self.data.positions[idx+1] += vel_y;
                self.data.positions[idx+2] += vel_z;
                // DEBUG: Visualize Velocity
                let speed_sq = vel_x*vel_x + vel_y*vel_y + vel_z*vel_z;
                if speed_sq > 0.001 {
                    self.data.debug_data[i] = 2.0; // Blue for moving
                } else {
                    self.data.debug_data[i] = 0.0;
                }
            }
            // 2. Solve Constraints
            self.distance_constraint.solve(&mut self.data, sdt);
            self.bending_constraint.solve(&mut self.data, sdt);

            // Solve Tethers if they exist
            if let Some(tether) = &self.tether_constraint {
                tether.solve(&mut self.data);
            }

            // 3. Solve Collision
            self.sdf_collider.solve(&mut self.data, sdt);
        }
    }

    pub fn get_pos_ptr(&self) -> *const f32 { self.data.positions.as_ptr() }
    pub fn get_debug_ptr(&self) -> *const f32 { self.data.debug_data.as_ptr() }
    pub fn get_count(&self) -> usize { self.data.count }

    pub fn get_sdf_config(&self) -> Vec<f32> {
        let s = &self.sdf_collider;
        vec![
            // Spine (0-6)
            s.spine_radius,
            s.spine_base[0], s.spine_base[1], s.spine_base[2],
            s.spine_top[0], s.spine_top[1], s.spine_top[2],
            // Shoulders (7-13)
            s.shoulder_radius,
            s.shoulder_left[0], s.shoulder_left[1], s.shoulder_left[2],
            s.shoulder_right[0], s.shoulder_right[1], s.shoulder_right[2],
        ]
    }
}