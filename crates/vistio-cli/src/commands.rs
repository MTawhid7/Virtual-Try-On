//! CLI command implementations.

use vistio_bench::metrics::BenchmarkMetrics;
use vistio_bench::runner::BenchmarkRunner;
use vistio_bench::scenarios::{Scenario, ScenarioKind};
use vistio_debug::snapshot::StateSnapshot;
use vistio_material::MaterialDatabase;
use vistio_solver::pd_solver::ProjectiveDynamicsSolver;

/// Run a simulation from config file.
pub fn simulate(config_path: &str) -> Result<(), Box<dyn std::error::Error>> {
    println!("Vistio Simulation");
    println!("─────────────────");
    println!("Config: {config_path}");
    println!();
    println!("Note: TOML config loading is a Tier-1 feature.");
    println!("Use `vistio benchmark` to run procedural scenarios now.");
    Ok(())
}

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

/// Run a simulation and stream to the Rerun viewer for live inspection.
pub fn visualize(
    scenario_name: &str,
    material_name: Option<&str>,
    _output_path: &str, // Kept for CLI compat; Rerun streams directly
) -> Result<(), Box<dyn std::error::Error>> {
    println!("Vistio Visual Simulation (Rerun)");
    println!("════════════════════════════════");
    println!();

    let kind = match scenario_name {
        "hanging_sheet" => vistio_bench::scenarios::ScenarioKind::HangingSheet,
        "sphere_drape" => vistio_bench::scenarios::ScenarioKind::SphereDrape,
        "cusick_drape" => vistio_bench::scenarios::ScenarioKind::CusickDrape,
        "cantilever_bending" => vistio_bench::scenarios::ScenarioKind::CantileverBending,
        "uniaxial_stretch" => vistio_bench::scenarios::ScenarioKind::UniaxialStretch,
        "self_fold" => vistio_bench::scenarios::ScenarioKind::SelfFold,
        other => {
            eprintln!("Unknown scenario: {other}");
            return Err("Unknown scenario".into());
        }
    };

    let mut scenario = vistio_bench::scenarios::Scenario::from_kind(kind);

    // Apply material if specified
    let material_label = if let Some(name) = material_name {
        let db = MaterialDatabase::with_defaults();
        let props = db.get(name).ok_or_else(|| {
            format!("Unknown material: '{name}'")
        })?;
        scenario = scenario.with_material(props.clone());
        name.to_string()
    } else {
        "default".to_string()
    };

    println!("Scenario:  {}", scenario_name);
    println!("Material:  {}", material_label);
    println!("Frames:    {}", scenario.timesteps);
    println!("Viewer:    Rerun (spawning...)");
    println!();

    // Launch Bevy Viewer
    println!("Initializing Bevy Viewer (PBR)...");

    // We don't need full SimulationState init here anymore, as the Bevy viewer
    // builds its own runner. We just launch the viewer with the scenario.
    vistio_viewer::launch_viewer(scenario).map_err(|e| format!("Viewer error: {}", e))?;

    Ok(())
}

/// Validate a mesh or config.
pub fn validate(path: &str) -> Result<(), Box<dyn std::error::Error>> {
    println!("Vistio Validator");
    println!("────────────────");
    println!();

    if path.ends_with(".toml") {
        println!("Validating config: {path}");
        let content = std::fs::read_to_string(path)?;
        let _config: vistio_solver::SolverConfig = toml::from_str(&content)?;
        println!("✅ Config is valid.");
    } else if path.ends_with(".json") {
        println!("Validating mesh: {path}");
        let content = std::fs::read_to_string(path)?;
        let mesh: vistio_mesh::TriangleMesh = serde_json::from_str(&content)?;
        match mesh.validate() {
            Ok(()) => println!("✅ Mesh is valid ({} verts, {} tris).", mesh.vertex_count(), mesh.triangle_count()),
            Err(e) => println!("❌ Mesh validation failed: {e}"),
        }
    } else {
        println!("Unsupported file format. Use .toml (config) or .json (mesh).");
    }

    Ok(())
}
