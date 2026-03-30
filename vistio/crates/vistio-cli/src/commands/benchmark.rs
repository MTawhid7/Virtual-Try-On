//! `vistio benchmark` command implementation.

use vistio_bench::metrics::BenchmarkMetrics;
use vistio_bench::runner::BenchmarkRunner;
use vistio_bench::scenarios::{Scenario, ScenarioKind};
use vistio_material::MaterialDatabase;
use vistio_solver::pd_solver::ProjectiveDynamicsSolver;

/// Run benchmark suite.
pub fn benchmark(
    scenario_name: &str,
    output_path: Option<&str>,
    material_name: Option<&str>,
) -> Result<(), Box<dyn std::error::Error>> {
    println!("Vistio Benchmark Suite");
    println!("══════════════════════");
    println!();

    // Look up material from database if specified
    let material_props = if let Some(name) = material_name {
        let db = MaterialDatabase::with_defaults();
        let props = db.get(name).ok_or_else(|| {
            let available: Vec<&str> = db.names();
            format!(
                "Unknown material: '{name}'. Available: {}",
                available.join(", ")
            )
        })?;
        println!("Material: {name}");
        println!();
        Some(props.clone())
    } else {
        None
    };

    let scenarios: Vec<ScenarioKind> = if scenario_name == "all" {
        ScenarioKind::all().to_vec()
    } else {
        let kind = match scenario_name {
            "hanging_sheet" => ScenarioKind::HangingSheet,
            "sphere_drape" => ScenarioKind::SphereDrape,
            "cusick_drape" => ScenarioKind::CusickDrape,
            "cantilever_bending" => ScenarioKind::CantileverBending,
            "uniaxial_stretch" => ScenarioKind::UniaxialStretch,
            "self_fold" => ScenarioKind::SelfFold,
            other => {
                eprintln!("Unknown scenario: {other}");
                eprintln!("Available: hanging_sheet, sphere_drape, cusick_drape, cantilever_bending, uniaxial_stretch, self_fold, all");
                return Err("Unknown scenario".into());
            }
        };
        vec![kind]
    };

    let mut all_metrics = Vec::new();
    let mut solver = ProjectiveDynamicsSolver::new();

    for &kind in &scenarios {
        let mut scenario = Scenario::from_kind(kind);

        // Apply material if specified
        if let Some(ref props) = material_props {
            scenario = scenario.with_material(props.clone());
        }

        println!("Running: {} ({} verts, {} tris, {} steps)",
            kind.name(),
            scenario.garment.vertex_count(),
            scenario.garment.triangle_count(),
            scenario.timesteps,
        );

        let metrics = BenchmarkRunner::run(&scenario, &mut solver)
            .map_err(|e| format!("Benchmark failed: {e}"))?;

        println!("  Wall time:     {:.3}s", metrics.total_wall_time);
        println!("  Avg step:      {:.3}ms", metrics.avg_step_time * 1000.0);
        println!("  Final KE:      {:.6e}", metrics.final_kinetic_energy);
        println!("  Max displace:  {:.4}m", metrics.max_displacement);
        println!();

        all_metrics.push(metrics);
    }

    // Output CSV
    if let Some(path) = output_path {
        let csv = BenchmarkMetrics::to_csv(&all_metrics);
        std::fs::write(path, &csv)?;
        println!("Results written to: {path}");
    } else {
        // Print to stdout
        println!("CSV Output:");
        println!("{}", BenchmarkMetrics::to_csv(&all_metrics));
    }

    Ok(())
}
