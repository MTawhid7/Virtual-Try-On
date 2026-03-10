//! Bevy systems for the Vistio viewer: simulation update and scene setup.

use bevy::prelude::*;
use bevy::render::mesh::Indices;
use bevy::render::render_resource::PrimitiveTopology;

use crate::resources::{SimRunner, SceneData, ClothMesh};
use crate::ipc_handler::ViewerIpcHandler;

// ─── Smooth Normals ──────────────────────────────────────────

pub(crate) fn compute_smooth_normals(
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

// ─── Simulation System ───────────────────────────────────────

pub(crate) fn simulate_cloth(
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

    // Performance watchdog: abort physics loop if it takes >30ms.
    // This prevents macOS "Application Not Responding" hangs when running
    // unoptimized debug builds (where physics takes seconds per step).
    let loop_start = std::time::Instant::now();

    while runner.accumulated_time >= dt && steps_this_frame < max_steps_per_frame {
        if steps_this_frame > 0 && loop_start.elapsed().as_millis() > 30 {
            // Unoptimized debug build detected: yield to renderer to keep app responsive.
            break;
        }

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
        use vistio_solver::strategy::SolverStrategy;
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

// ─── Scene Setup System ──────────────────────────────────────

pub(crate) fn setup_scene(
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
        bevy_panorbit_camera::PanOrbitCamera {
            focus: Vec3::new(0.0, 0.5, 0.0),
            radius: Some(3.5),
            ..default()
        },
    ));
}
