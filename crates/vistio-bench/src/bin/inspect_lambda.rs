use vistio_bench::scenarios::{Scenario, ScenarioKind};
use vistio_solver::pd_solver::ProjectiveDynamicsSolver;
use vistio_solver::state::SimulationState;
use vistio_contact::collision_pipeline::CollisionPipeline;

fn main() {
    let mut scenario = Scenario::sphere_drape();
    let is_ipc = scenario.config.ipc_enabled;
    let broad_phase = Box::new(vistio_contact::bvh::BvhBroadPhase::new(&scenario.garment));
    let mut pipeline = CollisionPipeline::new(broad_phase, Box::new(vistio_contact::narrow::VertexTriangleTest), Box::new(vistio_contact::response::ProjectionContactResponse), scenario.garment.clone(), 0.01 * 1.5, 0.0)
        .with_ground(-0.3)
        .with_sphere(vistio_math::Vec3::new(0.0, 0.0, 0.0), 0.3)
        .with_ipc(true);
        
    let topology = vistio_mesh::topology::Topology::build(&scenario.garment);
    let mut solver = ProjectiveDynamicsSolver::new();
    solver.init(&scenario.garment, &topology, &scenario.config, &scenario.pinned).unwrap();
    
    let mut state = SimulationState::from_mesh_with_masses(&scenario.garment, solver.lumped_masses(), &scenario.pinned).unwrap();
    state.ground_height = Some(-0.3);
    let initial_y: Vec<f32> = state.pos_y.clone();

    for step_idx in 0..50 {
        let mut handler = vistio_solver::pd_solver::DefaultIpcHandler {
            d_hat: 0.01,
            kappa: 0.0,
            mesh: &scenario.garment,
        };
        // We will just step manually so we can inspect state
        let _ = solver.step_with_ipc(&mut state, scenario.dt, &mut handler).unwrap();
        
        let max_d = (0..state.vertex_count).map(|i| {
            let dx = state.pos_x[i] - scenario.garment.pos_x[i];
            let dy = state.pos_y[i] - initial_y[i];
            let dz = state.pos_z[i] - scenario.garment.pos_z[i];
            (dx*dx + dy*dy + dz*dz).sqrt()
        }).fold(0.0f32, f32::max);
        
        let max_lam = state.al_lambda_y.iter().map(|&x| x.abs()).fold(0.0f32, f32::max);
        let max_vel = state.vel_y.iter().map(|&x| x.abs()).fold(0.0f32, f32::max);
        
        println!("Step {}: max_d={:.2}, max_lam={:.2}, max_vy={:.2}", step_idx, max_d, max_lam, max_vel);
    }
}
