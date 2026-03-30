//! Benchmark metrics — data collected during a benchmark run.

use serde::{Deserialize, Serialize};

/// Metrics collected from a benchmark scenario run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BenchmarkMetrics {
    /// Scenario name.
    pub scenario: String,
    /// Total wall-clock time (seconds).
    pub total_wall_time: f64,
    /// Number of timesteps executed.
    pub timesteps: u32,
    /// Average wall-clock time per timestep (seconds).
    pub avg_step_time: f64,
    /// Minimum step time.
    pub min_step_time: f64,
    /// Maximum step time.
    pub max_step_time: f64,
    /// Final kinetic energy (should approach zero for stable drape).
    pub final_kinetic_energy: f64,
    /// Maximum vertex displacement from initial position.
    pub max_displacement: f32,
    /// Average solver iterations per step (0 for stub).
    pub avg_iterations: f32,
    /// Vertex count.
    pub vertex_count: usize,
    /// Triangle count.
    pub triangle_count: usize,
    pub drape_coefficient: f32,
}

impl BenchmarkMetrics {
    /// Format as a CSV row (header + data).
    pub fn to_csv_header() -> String {
        "scenario,vertex_count,triangle_count,timesteps,total_wall_time_s,avg_step_ms,min_step_ms,max_step_ms,final_ke,max_displacement,avg_iterations,drape_coefficient_pct".to_string()
    }

    /// Format this metrics instance as a CSV data row.
    pub fn to_csv_row(&self) -> String {
        format!(
            "{},{},{},{},{:.6},{:.4},{:.4},{:.4},{:.6e},{:.6},{:.1},{:.2}",
            self.scenario,
            self.vertex_count,
            self.triangle_count,
            self.timesteps,
            self.total_wall_time,
            self.avg_step_time * 1000.0,
            self.min_step_time * 1000.0,
            self.max_step_time * 1000.0,
            self.final_kinetic_energy,
            self.max_displacement,
            self.avg_iterations,
            self.drape_coefficient,
        )
    }

    /// Format multiple metrics as a complete CSV string.
    pub fn to_csv(metrics: &[BenchmarkMetrics]) -> String {
        let mut csv = Self::to_csv_header();
        for m in metrics {
            csv.push('\n');
            csv.push_str(&m.to_csv_row());
        }
        csv
    }
}
