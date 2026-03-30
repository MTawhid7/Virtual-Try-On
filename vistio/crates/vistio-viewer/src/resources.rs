//! Bevy ECS resources and components for the Vistio viewer.

use bevy::prelude::*;
use vistio_contact::CollisionPipeline;
use vistio_solver::pd_solver::ProjectiveDynamicsSolver;
use vistio_solver::state::SimulationState;

/// System resource holding the Vistio simulation state.
#[derive(Resource)]
pub(crate) struct SimRunner {
    pub solver: ProjectiveDynamicsSolver,
    pub state: SimulationState,
    pub dt: f32,
    pub current_step: u32,
    pub indices: Vec<u32>, // Flat array of [i0, i1, i2, ...]
    pub collision: Option<CollisionPipeline>,
    pub config: vistio_solver::config::SolverConfig,
    pub mesh: vistio_mesh::TriangleMesh,
    /// Pre-computed barrier stiffness (computed once at init, not every frame)
    pub kappa: f32,
    /// Accumulated wall-clock time for fixed-timestep accumulation.
    /// Ensures deterministic step count regardless of frame rate.
    pub accumulated_time: f32,
}

#[derive(Resource)]
pub(crate) struct SceneData {
    pub body: Option<vistio_mesh::TriangleMesh>,
    pub ground_height: f32,
    pub scenario_kind: vistio_bench::scenarios::ScenarioKind,
}

/// Component to tag the Bevy cloth entity.
#[derive(Component)]
pub(crate) struct ClothMesh;
