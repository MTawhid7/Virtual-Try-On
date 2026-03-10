//! `vistio inspect` command implementation.

use vistio_debug::snapshot::StateSnapshot;

/// Inspect a state snapshot.
pub fn inspect(path: &str) -> Result<(), Box<dyn std::error::Error>> {
    println!("Vistio Snapshot Inspector");
    println!("────────────────────────");
    println!();

    let data = std::fs::read(path)?;
    let snapshot = StateSnapshot::from_bytes(&data)
        .map_err(|e| format!("Failed to read snapshot: {e}"))?;

    println!("Timestep:     {}", snapshot.timestep);
    println!("Sim time:     {:.4}s", snapshot.sim_time);
    println!("Vertices:     {}", snapshot.vertex_count);
    println!("Pos entries:  {}", snapshot.positions.len());
    println!("Vel entries:  {}", snapshot.velocities.len());

    // Quick stats
    if !snapshot.positions.is_empty() {
        let min_y = snapshot.positions.iter()
            .enumerate()
            .filter(|(i, _)| i % 3 == 1) // Y components
            .map(|(_, v)| *v)
            .fold(f32::INFINITY, f32::min);
        let max_y = snapshot.positions.iter()
            .enumerate()
            .filter(|(i, _)| i % 3 == 1)
            .map(|(_, v)| *v)
            .fold(f32::NEG_INFINITY, f32::max);
        println!("Y range:      [{:.4}, {:.4}]", min_y, max_y);
    }

    Ok(())
}
