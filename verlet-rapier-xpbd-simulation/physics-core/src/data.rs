// physics-core/src/data.rs

pub struct PhysicsData {
    pub count: usize,
    pub positions: Vec<f32>,
    pub prev_positions: Vec<f32>,
    pub inv_mass: Vec<f32>,
    pub indices: Vec<u32>,

    // NEW: Debug flags (0.0 = None, 1.0 = Collision)
    pub debug_data: Vec<f32>,
}

impl PhysicsData {
    pub fn new(positions: Vec<f32>, indices: Vec<u32>) -> Self {
        let count = positions.len() / 3;
        let prev_positions = positions.clone();
        let inv_mass = vec![1.0; count];

        // Initialize debug data with 0.0
        let debug_data = vec![0.0; count];

        let mut min_y = f32::MAX;
        let mut max_y = f32::MIN;
        for i in 0..count {
            let y = positions[i * 3 + 1];
            if y < min_y { min_y = y; }
            if y > max_y { max_y = y; }
        }
        crate::console_log!("[Rust] Mesh Loaded. Verts: {}. Y-Range: [{:.2}, {:.2}]", count, min_y, max_y);

        PhysicsData {
            count,
            positions,
            prev_positions,
            inv_mass,
            indices,
            debug_data,
        }
    }
}