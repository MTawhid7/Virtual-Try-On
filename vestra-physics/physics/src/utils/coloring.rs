// physics/src/utils/coloring.rs

/// Organizes constraints into batches such that no two constraints in the same batch
/// share a particle. This allows for parallel solving (if multi-threaded) or
/// simply ensures stable sequential solving order.
///
/// Uses a greedy graph coloring algorithm with a CSR (Compressed Sparse Row) adjacency structure.
pub fn color_constraints(
    constraints: &[[usize; 2]],
    particle_count: usize
) -> (Vec<usize>, Vec<usize>) {
    // 1. Build Adjacency (CSR Format)
    let mut degree = vec![0usize; particle_count];
    for &[p1, p2] in constraints {
        degree[p1] += 1;
        degree[p2] += 1;
    }

    let mut offset = vec![0usize; particle_count + 1];
    for i in 0..particle_count {
        offset[i + 1] = offset[i] + degree[i];
    }

    let mut adj = vec![0usize; offset[particle_count]];
    let mut counter = offset.clone();
    for (i, &[p1, p2]) in constraints.iter().enumerate() {
        adj[counter[p1]] = i;
        counter[p1] += 1;
        adj[counter[p2]] = i;
        counter[p2] += 1;
    }

    // 2. Greedy Coloring with bitmask for speed
    let mut constraint_colors: Vec<Option<usize>> = vec![None; constraints.len()];
    let mut batch_indices: Vec<Vec<usize>> = Vec::new();

    for i in 0..constraints.len() {
        let [p1, p2] = constraints[i];
        let mut used_colors = 0u64;

        // Find used colors among neighbors
        for &c_idx in &adj[offset[p1]..offset[p1+1]] {
            if let Some(c) = constraint_colors[c_idx] {
                // FIX: Explicitly use u64 literal for bitwise operation
                used_colors |= 1u64 << c;
            }
        }
        for &c_idx in &adj[offset[p2]..offset[p2+1]] {
            if let Some(c) = constraint_colors[c_idx] {
                // FIX: Explicitly use u64 literal for bitwise operation
                used_colors |= 1u64 << c;
            }
        }

        // Find the first unset bit (the first available color)
        let color = (!used_colors).trailing_zeros() as usize;
        constraint_colors[i] = Some(color);

        if color >= batch_indices.len() {
            batch_indices.resize(color + 1, Vec::new());
        }
        batch_indices[color].push(i);
    }

    flatten_batches(batch_indices, constraints.len())
}

pub fn color_constraints_3(
    constraints: &[[usize; 3]],
    particle_count: usize
) -> (Vec<usize>, Vec<usize>) {
    let mut degree = vec![0usize; particle_count];
    for &[p1, p2, p3] in constraints {
        degree[p1] += 1;
        degree[p2] += 1;
        degree[p3] += 1;
    }

    let mut offset = vec![0usize; particle_count + 1];
    for i in 0..particle_count {
        offset[i + 1] = offset[i] + degree[i];
    }

    let mut adj = vec![0usize; offset[particle_count]];
    let mut counter = offset.clone();
    for (i, &[p1, p2, p3]) in constraints.iter().enumerate() {
        adj[counter[p1]] = i; counter[p1] += 1;
        adj[counter[p2]] = i; counter[p2] += 1;
        adj[counter[p3]] = i; counter[p3] += 1;
    }

    let mut constraint_colors: Vec<Option<usize>> = vec![None; constraints.len()];
    let mut batch_indices: Vec<Vec<usize>> = Vec::new();

    for i in 0..constraints.len() {
        let [p1, p2, p3] = constraints[i];
        let mut used_colors = 0u64;

        for &c_idx in &adj[offset[p1]..offset[p1+1]] {
            if let Some(c) = constraint_colors[c_idx] { used_colors |= 1u64 << c; }
        }
        for &c_idx in &adj[offset[p2]..offset[p2+1]] {
            if let Some(c) = constraint_colors[c_idx] { used_colors |= 1u64 << c; }
        }
        for &c_idx in &adj[offset[p3]..offset[p3+1]] {
            if let Some(c) = constraint_colors[c_idx] { used_colors |= 1u64 << c; }
        }

        let color = (!used_colors).trailing_zeros() as usize;
        constraint_colors[i] = Some(color);

        if color >= batch_indices.len() {
            batch_indices.resize(color + 1, Vec::new());
        }
        batch_indices[color].push(i);
    }

    flatten_batches(batch_indices, constraints.len())
}

fn flatten_batches(batch_indices: Vec<Vec<usize>>, total_count: usize) -> (Vec<usize>, Vec<usize>) {
    let mut sorted_indices = Vec::with_capacity(total_count);
    let mut batch_offsets = Vec::new();
    let mut current_offset = 0;

    for batch in batch_indices {
        batch_offsets.push(current_offset);
        sorted_indices.extend_from_slice(&batch);
        current_offset += batch.len();
    }
    batch_offsets.push(current_offset);

    (sorted_indices, batch_offsets)
}