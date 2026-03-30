//! `vistio visualize` command implementation.

use vistio_material::MaterialDatabase;

/// Run a simulation and stream to the Bevy viewer for live inspection.
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
    println!("Viewer:    Bevy PBR (spawning...)");
    println!();

    // Launch Bevy Viewer
    vistio_viewer::launch_viewer(scenario).map_err(|e| format!("Viewer error: {}", e))?;

    Ok(())
}
