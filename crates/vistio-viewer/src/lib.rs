//! Vistio High-Fidelity Viewer using Bevy.
//!
//! Provides a physically based rendering environment for Vistio cloth simulations.



use bevy::prelude::*;
use bevy::render::mesh::Indices;
use bevy::render::render_resource::PrimitiveTopology;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};

use vistio_bench::scenarios::Scenario;
use vistio_solver::pd_solver::ProjectiveDynamicsSolver;
use vistio_solver::state::SimulationState;
use vistio_solver::strategy::SolverStrategy;
use vistio_mesh::topology::Topology;
use vistio_material::CoRotationalModel;
use vistio_contact::{CollisionPipeline, SpatialHash, VertexTriangleTest, ProjectionContactResponse};

/// System resource holding the Vistio simulation state.
#[derive(Resource)]
struct SimRunner {
    solver: ProjectiveDynamicsSolver,
    state: SimulationState,
    dt: f32,
    current_step: u32,
    indices: Vec<u32>, // Flat array of [i0, i1, i2, ...]
    collision: Option<CollisionPipeline>,
    config: vistio_solver::config::SolverConfig,
    mesh: vistio_mesh::TriangleMesh,
    /// Pre-computed barrier stiffness (computed once at init, not every frame)
    kappa: f32,
    /// Accumulated wall-clock time for fixed-timestep accumulation.
    /// Ensures deterministic step count regardless of frame rate.
    accumulated_time: f32,
}

#[derive(Resource)]
struct SceneData {
    body: Option<vistio_mesh::TriangleMesh>,
    ground_height: f32,
    scenario_kind: vistio_bench::scenarios::ScenarioKind,
}

/// Component to tag the Bevy cloth entity.
#[derive(Component)]
struct ClothMesh;

/// Launch the Bevy viewer for a given scenario.
pub fn launch_viewer(scenario: Scenario) -> Result<(), Box<dyn std::error::Error>> {
    println!("Initializing Bevy Viewer (PBR)...");

    // Initialize solver
    let topology = Topology::build(&scenario.garment);
    let mut solver = ProjectiveDynamicsSolver::new();

    let vertex_mass: f32 = if let Some(ref properties) = scenario.material {
        let model = Box::new(CoRotationalModel::new());
        solver.init_with_material(
            &scenario.garment,
            &topology,
            &scenario.config,
            properties,
            model,
            &scenario.pinned,
        ).map_err(|e| format!("Solver init failed: {e}"))?;

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

fn compute_smooth_normals(
    pos_x: &[f32],
    pos_y: &[f32],
    pos_z: &[f32],
    indices: &[u32],
) -> Vec<[f32; 3]> {
    let n = pos_x.len();
    let mut normals = vec![[0.0_f32; 3]; n];

    for chunk in indices.chunks_exact(3) {
        let i0 = chunk[0] as usize;
        let i1 = chunk[1] as usize;
        let i2 = chunk[2] as usize;

        let p0 = Vec3::new(pos_x[i0], pos_y[i0], pos_z[i0]);
        let p1 = Vec3::new(pos_x[i1], pos_y[i1], pos_z[i1]);
        let p2 = Vec3::new(pos_x[i2], pos_y[i2], pos_z[i2]);

        // Triangle normal (area weighted)
        let cross = (p1 - p0).cross(p2 - p0);

        normals[i0][0] += cross.x;
        normals[i0][1] += cross.y;
        normals[i0][2] += cross.z;
        normals[i1][0] += cross.x;
        normals[i1][1] += cross.y;
        normals[i1][2] += cross.z;
        normals[i2][0] += cross.x;
        normals[i2][1] += cross.y;
        normals[i2][2] += cross.z;
    }

    // Normalize
    for normal in &mut normals {
        let len_sq = normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2];
        if len_sq > 1e-12 {
            let inv_len = 1.0 / len_sq.sqrt();
            normal[0] *= inv_len;
            normal[1] *= inv_len;
            normal[2] *= inv_len;
        } else {
            // Fallback: point up
            normal[0] = 0.0;
            normal[1] = 1.0;
            normal[2] = 0.0;
        }
    }

    normals
}

struct ViewerIpcHandler<'a> {
    pipeline: &'a mut CollisionPipeline,
    d_hat: f32,
    kappa: f32,
    mesh: &'a vistio_mesh::TriangleMesh,
}

impl<'a> vistio_solver::pd_solver::IpcCollisionHandler for ViewerIpcHandler<'a> {
    fn detect_contacts(&mut self, px: &[f32], py: &[f32], pz: &[f32]) -> vistio_solver::pd_solver::IpcBarrierForces {
        self.pipeline.detect_ipc_contacts(px, py, pz, self.d_hat, self.kappa)
    }

    fn compute_ccd_step(&mut self, prev_x: &[f32], prev_y: &[f32], prev_z: &[f32], new_x: &[f32], new_y: &[f32], new_z: &[f32]) -> f32 {
        self.pipeline.compute_ccd_step(&self.mesh.indices, prev_x, prev_y, prev_z, new_x, new_y, new_z)
    }

    fn set_d_hat(&mut self, d_hat: f32) {
        self.d_hat = d_hat;
    }
}

fn simulate_cloth(
    mut runner: ResMut<SimRunner>,
    scene_data: Res<SceneData>,
    time: Res<Time>,
    mut meshes: ResMut<Assets<Mesh>>,
    query: Query<(Entity, &Handle<Mesh>), With<ClothMesh>>,
) {
    let dt = runner.dt;

    // Fixed-timestep accumulation: run physics steps at a fixed rate
    // regardless of Bevy's variable frame rate. This ensures determinism.
    runner.accumulated_time += time.delta_seconds();
    let max_steps_per_frame = 3; // Cap to prevent spiral of death
    let mut steps_this_frame = 0;

    while runner.accumulated_time >= dt && steps_this_frame < max_steps_per_frame {
        runner.accumulated_time -= dt;
        steps_this_frame += 1;

    if scene_data.scenario_kind == vistio_bench::scenarios::ScenarioKind::UniaxialStretch {
        // Stop pulling after 120 frames to hit exactly 50% strain without infinite tearing
        if runner.current_step < 120 {
            let stretch_amount = 0.25 / 120.0;
            for i in 0..runner.state.vertex_count {
                if runner.state.pos_x[i] > 0.249 {
                    runner.state.pos_x[i] += stretch_amount;
                    runner.state.vel_x[i] = stretch_amount / dt;
                }
            }
        }
    }

    // Borrow parts independently
    let SimRunner { ref mut solver, ref mut state, ref mut collision, ref config, ref mesh, kappa, .. } = *runner;

    let _result = if config.ipc_enabled {
        if let Some(ref mut pipeline) = collision {
            let mut handler = ViewerIpcHandler {
                pipeline,
                d_hat: config.barrier_d_hat,
                kappa,
                mesh,
            };
            match solver.step_with_ipc(state, dt, &mut handler) {
                Ok(res) => res,
                Err(e) => {
                    eprintln!("Simulation error: {}", e);
                    return;
                }
            }
        } else {
            let mut empty_handler = vistio_solver::pd_solver::EmptyIpcHandler;
            match solver.step_with_ipc(state, dt, &mut empty_handler) {
                Ok(res) => res,
                Err(e) => {
                    eprintln!("Simulation error: {}", e);
                    return;
                }
            }
        }
    } else {
        match solver.step(state, dt) {
            Ok(res) => res,
            Err(e) => {
                eprintln!("Simulation error: {}", e);
                return;
            }
        }
    };

    // Post-solve: IPC barriers handle all contact enforcement.
    // No post-solve hard projection needed — it would inject energy and
    // create oscillation by overriding barrier-computed positions.

    runner.current_step += 1;
    } // end fixed-timestep loop

    let n = runner.state.vertex_count;
    let mut positions = Vec::with_capacity(n);
    let mut uvs = Vec::with_capacity(n); // We just pass dummy UVs for now, or we can use the mesh UVs if we stored them

    for i in 0..n {
        positions.push([runner.state.pos_x[i], runner.state.pos_y[i], runner.state.pos_z[i]]);
        uvs.push([0.0_f32, 0.0_f32]); // Dummy UVs for StandardMaterial
    }

    let normals = compute_smooth_normals(
        &runner.state.pos_x,
        &runner.state.pos_y,
        &runner.state.pos_z,
        &runner.indices,
    );

    // Update Bevy mesh
    if let Ok((_entity, mesh_handle)) = query.get_single() {
        if let Some(mesh) = meshes.get_mut(mesh_handle) {
            mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
            mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
            // Updating UVs is technically not needed every frame if they are static,
            // but for simplicity we keep it here.
        }
    }
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    runner: Res<SimRunner>,
    scene_data: Res<SceneData>,
) {
    // 1. Setup Initial Mesh
    let n = runner.state.vertex_count;
    let mut positions = Vec::with_capacity(n);
    let mut uvs = Vec::with_capacity(n);

    for i in 0..n {
        positions.push([runner.state.pos_x[i], runner.state.pos_y[i], runner.state.pos_z[i]]);
        // Approximate UVs based on index since we didn't store original UVs in SimulationState
        // (For the real checkerboard we should pass the actual UVs, but StandardMaterial doesn't strictly need them unless mapped)
        uvs.push([(i % 20) as f32 / 20.0, (i / 20) as f32 / 20.0]);
    }

    let normals = compute_smooth_normals(
        &runner.state.pos_x,
        &runner.state.pos_y,
        &runner.state.pos_z,
        &runner.indices,
    );

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        Default::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    mesh.insert_indices(Indices::U32(runner.indices.clone()));

    // Fabric Material - Double-sided PBR
    let cloth_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.8, 0.2, 0.2), // Deep red cloth
        perceptual_roughness: 0.9, // Fabric is rough
        metallic: 0.05,
        double_sided: true, // Crucial for cloth!
        cull_mode: None,    // Render both sides
        ..default()
    });

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(mesh),
            material: cloth_material,
            ..default()
        },
        bevy::render::view::NoFrustumCulling,
        ClothMesh,
    ));

    // 2. Setup Ground Plane
    let ground_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.2, 0.2, 0.25),
        perceptual_roughness: 0.8,
        ..default()
    });

    let gh = scene_data.ground_height;
    commands.spawn(PbrBundle {
        mesh: meshes.add(Cuboid::new(4.0, 0.05, 4.0)),
        material: ground_material,
        transform: Transform::from_xyz(0.0, gh - 0.025, 0.0), // top face at Y = ground_height
        ..default()
    });

    // 2.5. Setup Body (e.g. Sphere) if present
    if let Some(ref body) = scene_data.body {
        let n_body = body.vertex_count();
        let mut body_positions = Vec::with_capacity(n_body);
        let mut body_uvs = Vec::with_capacity(n_body);

        for i in 0..n_body {
            body_positions.push([body.pos_x[i], body.pos_y[i], body.pos_z[i]]);
            body_uvs.push([0.0_f32, 0.0_f32]);
        }

        let body_normals = compute_smooth_normals(&body.pos_x, &body.pos_y, &body.pos_z, &body.indices);

        let mut body_mesh = Mesh::new(
            PrimitiveTopology::TriangleList,
            Default::default(),
        );
        body_mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, body_positions);
        body_mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, body_normals);
        body_mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, body_uvs);
        body_mesh.insert_indices(Indices::U32(body.indices.clone()));

        let body_material = materials.add(StandardMaterial {
            base_color: Color::srgb(0.7, 0.7, 0.7),
            perceptual_roughness: 0.6,
            ..default()
        });

        commands.spawn(PbrBundle {
            mesh: meshes.add(body_mesh),
            material: body_material,
            ..default()
        });
    }

    // 2.6 Setup Cylinder/Box for specific scenarios (Visual Geometry)
    match scene_data.scenario_kind {
        vistio_bench::scenarios::ScenarioKind::CusickDrape => {
            let pedestal_material = materials.add(StandardMaterial {
                base_color: Color::srgb(0.5, 0.5, 0.55),
                perceptual_roughness: 0.7,
                ..default()
            });
            // Cylinder: radius 0.09, height 0.6. Centered at origin of its local transform.
            // Our collision cylinder starts at y=-0.3 and goes up by height 0.6 (top at y=0.3).
            // Bevy's cylinder is centered at 0. So we place it at y = -0.3 + (0.6 / 2) = 0.0.
            commands.spawn(PbrBundle {
                mesh: meshes.add(Cylinder::new(0.09, 0.6)),
                material: pedestal_material,
                transform: Transform::from_xyz(0.0, 0.0, 0.0),
                ..default()
            });
        },
        vistio_bench::scenarios::ScenarioKind::CantileverBending => {
            let ledge_material = materials.add(StandardMaterial {
                base_color: Color::srgb(0.6, 0.4, 0.2), // Wood-like
                perceptual_roughness: 0.9,
                ..default()
            });
            // Box is min_x=-0.1, max_x=0.1, min_y=0.0, max_y=0.5, min_z=-0.2, max_z=0.0
            // Size: (0.2, 0.5, 0.2)
            // Center is (0.0, 0.25, -0.1)
            commands.spawn(PbrBundle {
                mesh: meshes.add(Cuboid::new(0.2, 0.5, 0.2)),
                material: ledge_material,
                transform: Transform::from_xyz(0.0, 0.25, -0.1),
                ..default()
            });
        },
        _ => {}
    }

    // 3. Setup Directional Light (Sun/Key Light) with Shadows
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(4.0, 8.0, 4.0)
            .looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    // 4. Setup Ambient Light (Fill Light)
    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 100.0,
    });

    // 5. Setup Camera
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0.0, 1.5, 3.5).looking_at(Vec3::new(0.0, 0.5, 0.0), Vec3::Y),
            ..default()
        },
        PanOrbitCamera {
            focus: Vec3::new(0.0, 0.5, 0.0),
            radius: Some(3.5),
            ..default()
        },
    ));
}
