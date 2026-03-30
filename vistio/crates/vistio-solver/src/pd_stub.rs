//! Projective Dynamics stub solver.
//!
//! This is a Tier-0 placeholder that implements the full solver interface
//! but skips the local step (energy projection). It runs the prediction
//! and velocity update phases correctly, producing valid but gravity-only
//! motion. This validates the pipeline wiring before the real PD solver
//! is implemented in Tier 1.

use std::time::Instant;

use vistio_mesh::TriangleMesh;
use vistio_mesh::topology::Topology;
use vistio_types::VistioResult;

use crate::config::SolverConfig;
use crate::state::SimulationState;
use crate::strategy::{SolverStrategy, StepResult};

/// Stub Projective Dynamics solver.
///
/// Performs correct inertial prediction (gravity) but skips the
/// local-global iteration loop. Positions follow a pure ballistic
/// trajectory modified only by damping and pinning constraints.
///
/// This stub exists to:
/// 1. Validate the pipeline (mesh → state → solver → output)
/// 2. Verify the SolverStrategy trait API is well-designed
/// 3. Provide a baseline for benchmark comparisons
pub struct ProjectiveDynamicsStub {
    config: SolverConfig,
    initialized: bool,
}

impl ProjectiveDynamicsStub {
    /// Creates a new stub solver.
    pub fn new() -> Self {
        Self {
            config: SolverConfig::default(),
            initialized: false,
        }
    }
}

impl Default for ProjectiveDynamicsStub {
    fn default() -> Self {
        Self::new()
    }
}

impl SolverStrategy for ProjectiveDynamicsStub {
    fn init(
        &mut self,
        _mesh: &TriangleMesh,
        _topology: &Topology,
        config: &SolverConfig,
        _pinned: &[bool],
    ) -> VistioResult<()> {
        self.config = config.clone();
        self.initialized = true;
        Ok(())
    }

    fn step(
        &mut self,
        state: &mut SimulationState,
        dt: f32,
    ) -> VistioResult<StepResult> {
        let start = Instant::now();

        if !self.initialized {
            return Err(vistio_types::VistioError::InvalidConfig(
                "Solver not initialized. Call init() first.".into(),
            ));
        }

        // 1. Save previous positions
        state.save_previous();

        // 2. Predict: p_pred = p + dt*v + dt²*g
        state.predict(dt, self.config.gravity);

        // 3. Accept predictions as final positions (stub: skip local-global)
        //    In the real PD solver, this is where the iterative solve happens:
        //    for iter in 0..max_iterations {
        //        local_step();   // Project each element to its constraint set
        //        global_step();  // Solve linear system for optimal positions
        //    }
        for i in 0..state.vertex_count {
            state.pos_x[i] = state.pred_x[i];
            state.pos_y[i] = state.pred_y[i];
            state.pos_z[i] = state.pred_z[i];
        }

        // 4. Update velocities from position change
        state.update_velocities(dt);

        // 5. Apply damping
        state.damp_velocities(self.config.damping);

        let wall_time = start.elapsed().as_secs_f64();

        Ok(StepResult {
            iterations: 0, // Stub: no iterations
            final_residual: 0.0,
            converged: true,
            wall_time,
        })
    }

    fn name(&self) -> &str {
        "projective_dynamics_stub"
    }
}
