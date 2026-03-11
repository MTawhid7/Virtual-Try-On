//! Vistio High-Fidelity Viewer using Bevy.
//!
//! Provides a physically based rendering environment for Vistio cloth simulations.
//!
//! ## Module Structure
//!
//! - `resources` — Bevy ECS resources (SimRunner, SceneData) and components (ClothMesh)
//! - `ipc_handler` — IPC collision handler bridging viewer to contact system
//! - `systems` — Bevy systems (simulate_cloth, setup_scene, compute_smooth_normals)

mod resources;
mod ipc_handler;
mod systems;

use bevy::prelude::*;
use bevy_panorbit_camera::PanOrbitCameraPlugin;

use vistio_bench::scenarios::Scenario;
use vistio_solver::pd_solver::ProjectiveDynamicsSolver;
use vistio_solver::state::SimulationState;
use vistio_solver::strategy::SolverStrategy;
use vistio_mesh::topology::Topology;
use vistio_material::CoRotationalModel;
use vistio_contact::{CollisionPipeline, SpatialHash, VertexTriangleTest, ProjectionContactResponse};

use resources::{SimRunner, SceneData};
use systems::{simulate_cloth, setup_scene};

/// Launch the Bevy viewer for a given scenario.
pub fn launch_viewer(scenario: Scenario) -> Result<(), Box<dyn std::error::Error>> {
    println!("Initializing Bevy Viewer (PBR)...");

    // Initialize solver
    let topology = Topology::build(&scenario.garment);
    let mut solver = ProjectiveDynamicsSolver::new();

    let vertex_mass: f32 = if let Some(ref properties) = scenario.material {
        if properties.is_anisotropic() {
            // Anisotropic: Tier 3/4 with Discrete Shells + anisotropic co-rotational model
            let model = Box::new(vistio_material::AnisotropicCoRotationalModel::from_properties(properties));
            if scenario.config.ipc_enabled {
                solver.init_with_material_tier4(
                    &scenario.garment,
                    &topology,
                    &scenario.config,
                    properties,
                    model,
                    &scenario.pinned,
                ).map_err(|e| format!("Solver init failed: {e}"))?;
            } else {
                solver.init_with_material_tier3(
                    &scenario.garment,
                    &topology,
                    &scenario.config,
                    properties,
                    model,
                    &scenario.pinned,
                ).map_err(|e| format!("Solver init failed: {e}"))?;
            }
        } else {
            // Isotropic: Tier 2/4 with dihedral bending + co-rotational model
            let model = Box::new(CoRotationalModel::new());
            if scenario.config.ipc_enabled {
                solver.init_with_material_tier4(
                    &scenario.garment,
                    &topology,
                    &scenario.config,
                    properties,
                    model,
                    &scenario.pinned,
                ).map_err(|e| format!("Solver init failed: {e}"))?;
            } else {
                solver.init_with_material(
                    &scenario.garment,
                    &topology,
                    &scenario.config,
                    properties,
                    model,
                    &scenario.pinned,
                ).map_err(|e| format!("Solver init failed: {e}"))?;
            }
        }

        let total_area: f32 = {
            use bevy::prelude::Vec3;
            let mesh = &scenario.garment;
            (0..mesh.triangle_count()).map(|t| {
                let idx_base = t * 3;
                let i0 = mesh.indices[idx_base] as usize;
                let i1 = mesh.indices[idx_base + 1] as usize;
                let i2 = mesh.indices[idx_base + 2] as usize;
                let p0 = Vec3::new(mesh.pos_x[i0], mesh.pos_y[i0], mesh.pos_z[i0]);
                let p1 = Vec3::new(mesh.pos_x[i1], mesh.pos_y[i1], mesh.pos_z[i1]);
                let p2 = Vec3::new(mesh.pos_x[i2], mesh.pos_y[i2], mesh.pos_z[i2]);
                0.5 * (p1 - p0).cross(p2 - p0).length()
            }).sum()
        };
        properties.mass_per_vertex(scenario.garment.vertex_count(), total_area)
    } else {
        solver.init(&scenario.garment, &topology, &scenario.config, &scenario.pinned)
            .map_err(|e| format!("Solver init failed: {e}"))?;
        scenario.vertex_mass
    };

    let mut state = SimulationState::from_mesh_with_masses(
        &scenario.garment,
        solver.lumped_masses(),
        &scenario.pinned,
    ).map_err(|e| format!("State init failed: {e}"))?;

    let is_ipc = scenario.config.ipc_enabled;
    let broad_phase: Box<dyn vistio_contact::broad::BroadPhase + Send + Sync> = if is_ipc {
        Box::new(vistio_contact::bvh::BvhBroadPhase::new(&scenario.garment))
    } else {
        Box::new(SpatialHash::new(0.05))
    };

    let thickness = if is_ipc { scenario.config.barrier_d_hat * 1.5 } else { 0.0 };

    let mut pipeline = CollisionPipeline::new(
        broad_phase,
        Box::new(VertexTriangleTest),
        Box::new(ProjectionContactResponse),
        scenario.garment.clone(),
        thickness,
        0.0,
    );

    if is_ipc {
        pipeline = pipeline.with_self_collision(&topology, 1).with_ipc(true);
    }

    let ground_height: f32;
    match scenario.kind {
        vistio_bench::scenarios::ScenarioKind::SphereDrape => {
            ground_height = -0.3;
            pipeline = pipeline
                .with_ground(-0.3)
                .with_sphere(vistio_math::Vec3::new(0.0, 0.0, 0.0), 0.3);
        },
        vistio_bench::scenarios::ScenarioKind::CusickDrape => {
            ground_height = -0.3; // Give it room to fall
            pipeline = pipeline
                .with_ground(-0.3)
                .with_cylinder(0.0, 0.0, 0.3, 0.09); // Base at y=-0.3, radius 0.09m, height 0.6m (top at y=0.3)
        },
        vistio_bench::scenarios::ScenarioKind::CantileverBending => {
            ground_height = 0.0; // Floor at y=0.0
            pipeline = pipeline
                .with_ground(0.0)
                .with_box(-0.1, 0.1, 0.0, 0.499, -0.2, 0.0);
        },
        _ => {
            ground_height = -0.3;
            pipeline = pipeline.with_ground(-0.3);
        }
    }

    // Wire ground height into the solver's internal constraint enforcement
    state.ground_height = Some(ground_height);

    // Pre-compute kappa once at init
    let kappa = vistio_contact::barrier::estimate_initial_kappa(
        scenario.config.barrier_kappa,
        vertex_mass,
        scenario.config.gravity[1].abs(),
        scenario.config.barrier_d_hat,
    );

    let runner = SimRunner {
        solver,
        state,
        dt: scenario.dt,
        current_step: 0,
        indices: scenario.garment.indices.clone(),
        collision: Some(pipeline),
        config: scenario.config.clone(),
        mesh: scenario.garment.clone(),
        kappa,
        accumulated_time: 0.0,
    };

    let scene_data = SceneData {
        body: scenario.body.clone(),
        ground_height,
        scenario_kind: scenario.kind,
    };

    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: format!("Vistio Viewer - {}", scenario.kind.name()),
            resolution: (1280., 720.).into(),
            ..default()
        }),
        ..default()
    }));
    app.add_plugins(PanOrbitCameraPlugin);

    app.insert_resource(runner);
    app.insert_resource(scene_data);
    app.insert_resource(ClearColor(Color::srgb(0.05, 0.05, 0.08))); // Dark background

    // Setup scene
    app.add_systems(Startup, setup_scene);

    // Update simulation
    app.add_systems(Update, simulate_cloth);

    app.run();

    Ok(())
}
