// physics-core/src/collider/spatial_hash.rs
use crate::data::PhysicsData;

pub struct SpatialHash {
    cell_size: f32,
    table_size: usize,
    table: Vec<i32>, // Stores the 'head' particle index for each cell
    next: Vec<i32>,  // Stores the 'next' particle index in the chain
    query_ids: Vec<usize>, // Temp buffer for neighbors
}

impl SpatialHash {
    pub fn new(count: usize, cell_size: f32) -> Self {
        // Table size should be ~2x particle count and Prime to reduce collisions
        let table_size = next_prime(count * 2);

        SpatialHash {
            cell_size,
            table_size,
            table: vec![-1; table_size],
            next: vec![-1; count],
            query_ids: Vec::with_capacity(64),
        }
    }

    pub fn solve(&mut self, data: &mut PhysicsData) {
        let thickness = 0.005; // 0.5cm separation
        let stiffness = 0.1;   // Soft repulsion

        // 1. Clear Table
        self.table.fill(-1);
        self.next.fill(-1);

        // 2. Build Hash Grid
        for i in 0..data.count {
            let idx = i * 3;
            let h = self.hash(
                data.positions[idx],
                data.positions[idx+1],
                data.positions[idx+2]
            );

            // Insert into linked list:
            // Point current particle to what was previously at head
            self.next[i] = self.table[h];
            // Set current particle as new head
            self.table[h] = i as i32;
        }

        // 3. Solve Collisions
        for i in 0..data.count {
            if data.inv_mass[i] == 0.0 { continue; }

            let idx_a = i * 3;
            let px = data.positions[idx_a];
            let py = data.positions[idx_a+1];
            let pz = data.positions[idx_a+2];

            // Find neighbors
            self.query(px, py, pz, thickness);

            for &j in &self.query_ids {
                if i == j { continue; }

                let idx_b = j * 3;
                let dx = px - data.positions[idx_b];
                let dy = py - data.positions[idx_b+1];
                let dz = pz - data.positions[idx_b+2];

                let dist_sq = dx*dx + dy*dy + dz*dz;

                if dist_sq < thickness * thickness && dist_sq > 1e-6 {
                    let dist = dist_sq.sqrt();
                    let correction = (thickness - dist) * 0.5 * stiffness;

                    let nx = dx / dist;
                    let ny = dy / dist;
                    let nz = dz / dist;

                    let w1 = data.inv_mass[i];
                    let w2 = data.inv_mass[j];
                    let w_sum = w1 + w2;

                    if w_sum > 0.0 {
                        let s = correction / w_sum;

                        if w1 > 0.0 {
                            data.positions[idx_a] += nx * s * w1;
                            data.positions[idx_a+1] += ny * s * w1;
                            data.positions[idx_a+2] += nz * s * w1;
                        }
                        if w2 > 0.0 {
                            data.positions[idx_b] -= nx * s * w2;
                            data.positions[idx_b+1] -= ny * s * w2;
                            data.positions[idx_b+2] -= nz * s * w2;
                        }
                    }
                }
            }
        }
    }

    fn hash(&self, x: f32, y: f32, z: f32) -> usize {
        let xi = (x / self.cell_size).floor() as i32;
        let yi = (y / self.cell_size).floor() as i32;
        let zi = (z / self.cell_size).floor() as i32;

        // Large primes for hashing
        let h = (xi.wrapping_mul(73856093)) ^
                (yi.wrapping_mul(19349663)) ^
                (zi.wrapping_mul(83492791));

        (h.abs() as usize) % self.table_size
    }

    fn query(&mut self, x: f32, y: f32, z: f32, r: f32) {
        self.query_ids.clear();

        let x0 = ((x - r) / self.cell_size).floor() as i32;
        let x1 = ((x + r) / self.cell_size).floor() as i32;
        let y0 = ((y - r) / self.cell_size).floor() as i32;
        let y1 = ((y + r) / self.cell_size).floor() as i32;
        let z0 = ((z - r) / self.cell_size).floor() as i32;
        let z1 = ((z + r) / self.cell_size).floor() as i32;

        for xi in x0..=x1 {
            for yi in y0..=y1 {
                for zi in z0..=z1 {
                    let h = (xi.wrapping_mul(73856093) ^
                             yi.wrapping_mul(19349663) ^
                             zi.wrapping_mul(83492791)).abs() as usize % self.table_size;

                    let mut id = self.table[h];
                    while id != -1 {
                        self.query_ids.push(id as usize);
                        id = self.next[id as usize];
                    }
                }
            }
        }
    }
}

fn next_prime(mut n: usize) -> usize {
    loop {
        let mut is_prime = true;
        let limit = (n as f64).sqrt() as usize;
        for i in 2..=limit {
            if n % i == 0 {
                is_prime = false;
                break;
            }
        }
        if is_prime { return n; }
        n += 1;
    }
}