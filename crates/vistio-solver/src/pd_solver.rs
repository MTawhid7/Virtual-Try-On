//! Projective Dynamics solver — the core Tier 1/2/3 solver.
//!
//! Implements the local-global iteration loop:
//! 1. **Predict** — inertial position from velocity + gravity
//! 2. **Local step** — project each element toward its closest valid configuration
//!    (membrane + bending projections)
//! 3. **Global step** — solve the constant SPD system A * q = rhs
//! 4. **Repeat** steps 2–3 until convergence or max iterations
//! 5. **Finalize** — update velocities from position change
//!
//! ## Material-Aware Mode (Tier 2+)
//!
//! When initialized via `init_with_material()`, the solver uses a pluggable
//! `ConstitutiveModel` for the local step and derives stiffness/mass from
//! `FabricProperties`. This produces material-specific drape behavior.
//!
//! ## Discrete Shells Bending (Tier 3)
//!
//! Uses the cotangent-weighted Discrete Shells model (Grinspun 2003) for
//! curvature-based bending energy. Bending projections are fully integrated
//! into the local-global loop (not applied as a post-solve correction).

use std::time::Instant;

use vistio_material::ConstitutiveModel;
use vistio_material::FabricProperties;
use vistio_math::faer_solver::FaerSolver;
use vistio_math::sparse::SparseSolver;
use vistio_mesh::TriangleMesh;
use vistio_mesh::topology::Topology;
use vistio_types::VistioResult;

use crate::assembly::{assemble_rhs, assemble_system_matrix, assemble_barrier_rhs, BendingModel, BendingRhs};
use crate::bending::BendingData;
use crate::config::SolverConfig;
use crate::discrete_shells::DiscreteShellsBendingData;
use crate::element::ElementData;
use crate::state::SimulationState;
use crate::strategy::{SolverStrategy, StepResult};

/// Projective Dynamics solver with pluggable constitutive model.
///
/// Uses `faer` for sparse Cholesky factorization. The system matrix is
/// prefactored once during `init()` and reused for all timesteps.
///
/// ## Three initialization modes
///
/// - `init()` — Tier 1 mode: hardcoded ARAP projection, uniform mass, dihedral bending
/// - `init_with_material()` — Tier 2 mode: uses `ConstitutiveModel` + `FabricProperties`
///   with dihedral bending
/// - `init_with_material_tier3()` — Tier 3 mode: uses `ConstitutiveModel` + `FabricProperties`
///   with Discrete Shells bending (cotangent-weighted, fully integrated into local-global loop)
pub struct ProjectiveDynamicsSolver {
    /// Precomputed FEM element data.
    elements: Option<ElementData>,
    /// Legacy dihedral bending data (Tier 1-2).
    bending: Option<BendingData>,
    /// Discrete Shells bending data (Tier 3+).
    ds_bending: Option<DiscreteShellsBendingData>,
    /// Sparse Cholesky solver with cached factorization.
    solver: FaerSolver,
    /// Configuration snapshot from init().
    config: SolverConfig,
    /// Whether init() has been called successfully.
    initialized: bool,
    /// Cached per-vertex mass.
    mass: Vec<f32>,
    /// Number of vertices.
    n: usize,
    /// Optional pluggable constitutive model (Tier 2+).
    /// When `Some`, the local step delegates to this model.
    /// When `None`, falls back to hardcoded ARAP.
    material_model: Option<Box<dyn ConstitutiveModel>>,
}

impl ProjectiveDynamicsSolver {
    /// Creates a new solver (uninitialized).
    pub fn new() -> Self {
        Self {
            elements: None,
            bending: None,
            ds_bending: None,
            solver: FaerSolver::new(),
            config: SolverConfig::default(),
            initialized: false,
            mass: Vec::new(),
            n: 0,
            material_model: None,
        }
    }

    /// Return the area-weighted lumped masses computed during init().
    /// Callers should use these for `SimulationState` construction
    /// to ensure the state mass matches the system matrix mass.
    pub fn lumped_masses(&self) -> &[f32] {
        &self.mass
    }

    /// Initialize the solver with material properties and a constitutive model.
    ///
    /// This is the Tier 2 initialization path. It:
    /// - Derives per-element stiffness from `FabricProperties`
    /// - Derives per-vertex mass from the material's areal density
    /// - Derives bending stiffness from the material's bending properties
    /// - Uses dihedral bending (legacy) with the provided `ConstitutiveModel` in the local step
    pub fn init_with_material(
        &mut self,
        mesh: &TriangleMesh,
        topology: &Topology,
        config: &SolverConfig,
        properties: &FabricProperties,
        model: Box<dyn ConstitutiveModel>,
        pinned: &[bool],
    ) -> VistioResult<()> {
        self.n = mesh.vertex_count();
        self.config = config.clone();

        // Build FEM elements with material-derived stiffness
        let elements = ElementData::from_mesh_with_material(mesh, properties);

        // Compute area-weighted lumped mass matrix
        self.mass = compute_lumped_masses(self.n, &elements, properties.density, pinned);

        // Build dihedral bending elements with material-derived stiffness
        let bending = BendingData::from_topology_with_material(mesh, topology, properties);
        self.bending = Some(bending);
        self.ds_bending = None;

        // Assemble the constant system matrix (includes bending stiffness)
        let dt = 1.0 / 60.0;
        let bending_model = self.bending.as_ref().map(BendingModel::Dihedral);
        let system_matrix = assemble_system_matrix(self.n, &self.mass, dt, &elements, bending_model);

        // Prefactor via sparse Cholesky
        self.solver.factorize(&system_matrix).map_err(|e| {
            vistio_types::VistioError::InvalidConfig(format!("Cholesky factorization failed: {e}"))
        })?;

        self.elements = Some(elements);
        self.material_model = Some(model);
        self.initialized = true;
        Ok(())
    }

    /// Initialize the solver with Tier 3 Discrete Shells bending.
    ///
    /// This is the Tier 3 initialization path. It:
    /// - Derives per-element stiffness from `FabricProperties`
    /// - Derives per-vertex mass from the material's areal density
    /// - Uses **Discrete Shells** (cotangent-weighted) bending model
    /// - Bending projections are fully integrated into the PD local-global loop
    pub fn init_with_material_tier3(
        &mut self,
        mesh: &TriangleMesh,
        topology: &Topology,
        config: &SolverConfig,
        properties: &FabricProperties,
        model: Box<dyn ConstitutiveModel>,
        pinned: &[bool],
    ) -> VistioResult<()> {
        self.n = mesh.vertex_count();
        self.config = config.clone();

        // Build FEM elements with material-derived stiffness
        let elements = ElementData::from_mesh_with_material(mesh, properties);

        // Compute area-weighted lumped mass matrix
        self.mass = compute_lumped_masses(self.n, &elements, properties.density, pinned);

        // Build Discrete Shells bending elements
        let ds_bending = if properties.is_anisotropic() {
            DiscreteShellsBendingData::from_topology_with_anisotropic_material(mesh, topology, properties)
        } else {
            DiscreteShellsBendingData::from_topology_with_material(mesh, topology, properties)
        };
        self.ds_bending = Some(ds_bending);
        self.bending = None;

        // Assemble the constant system matrix with Discrete Shells stencils
        let dt = 1.0 / 60.0;
        let bending_model = self.ds_bending.as_ref().map(BendingModel::DiscreteShells);
        let system_matrix = assemble_system_matrix(self.n, &self.mass, dt, &elements, bending_model);

        // Prefactor via sparse Cholesky
        self.solver.factorize(&system_matrix).map_err(|e| {
            vistio_types::VistioError::InvalidConfig(format!("Cholesky factorization failed: {e}"))
        })?;

        self.elements = Some(elements);
        self.material_model = Some(model);
        self.initialized = true;
        Ok(())
    }

    /// Initialize the solver with Tier 4 Augmented Lagrangian IPC contact.
    ///
    /// This is the Tier 4 initialization path. It builds upon Tier 3 by:
    /// - Deriving per-element stiffness from `FabricProperties`
    /// - Using **Discrete Shells** (cotangent-weighted) bending model
    /// - Preparing the configuration for the `step_with_ipc()` AL outer loop
    pub fn init_with_material_tier4(
        &mut self,
        mesh: &TriangleMesh,
        topology: &Topology,
        config: &SolverConfig,
        properties: &FabricProperties,
        model: Box<dyn ConstitutiveModel>,
        pinned: &[bool],
    ) -> VistioResult<()> {
        self.init_with_material_tier3(mesh, topology, config, properties, model, pinned)
    }
}

impl Default for ProjectiveDynamicsSolver {
    fn default() -> Self {
        Self::new()
    }
}

impl SolverStrategy for ProjectiveDynamicsSolver {
    fn init(
        &mut self,
        mesh: &TriangleMesh,
        topology: &Topology,
        config: &SolverConfig,
        pinned: &[bool],
    ) -> VistioResult<()> {
        self.n = mesh.vertex_count();
        self.config = config.clone();

        // Build FEM elements with stiffness from config
        let elements = ElementData::from_mesh(mesh, config.stretch_weight * 1000.0);

        // Default density 200 g/m² for the base config
        let density = 200.0;
        self.mass = compute_lumped_masses(self.n, &elements, density, pinned);

        // Build bending elements from topology (legacy dihedral)
        let bending = BendingData::from_topology(mesh, topology, config.bending_weight * 100.0);
        self.bending = Some(bending);
        self.ds_bending = None;

        // Assemble the constant system matrix (includes bending stiffness)
        let dt = 1.0 / 60.0;
        let bending_model = self.bending.as_ref().map(BendingModel::Dihedral);
        let system_matrix = assemble_system_matrix(self.n, &self.mass, dt, &elements, bending_model);

        // Prefactor via sparse Cholesky
        self.solver.factorize(&system_matrix).map_err(|e| {
            vistio_types::VistioError::InvalidConfig(format!("Cholesky factorization failed: {e}"))
        })?;

        self.elements = Some(elements);
        self.material_model = None;
        self.initialized = true;
        Ok(())
    }

    fn step(
        &mut self,
        state: &mut SimulationState,
        dt: f32,
    ) -> VistioResult<StepResult> {
        if !self.initialized {
            return Err(vistio_types::VistioError::InvalidConfig(
                "Solver not initialized. Call init() first.".into(),
            ));
        }

        let start = Instant::now();
        let elements = self.elements.as_ref().unwrap();
        let n = self.n;

        // 1. Save previous positions
        state.save_previous();

        // 2. Predict: q_pred = pos + dt*vel + dt²*gravity
        state.predict(dt, self.config.gravity);

        // 3. Initialize current positions from predictions (initial guess)
        state.pos_x.copy_from_slice(&state.pred_x);
        state.pos_y.copy_from_slice(&state.pred_y);
        state.pos_z.copy_from_slice(&state.pred_z);

        // Buffers for the global solve
        let mut sol_x = vec![0.0_f32; n];
        let mut sol_y = vec![0.0_f32; n];
        let mut sol_z = vec![0.0_f32; n];

        let mut iterations = 0_u32;
        let mut final_residual = f64::MAX;

        // 4. Local-Global iteration loop
        for iter in 0..self.config.max_iterations {
            // === LOCAL STEP: MEMBRANE ===
            let mut proj_x = Vec::with_capacity(elements.len());
            let mut proj_y = Vec::with_capacity(elements.len());
            let mut proj_z = Vec::with_capacity(elements.len());

            for elem in &elements.elements {
                let (p0, p1, p2) = if let Some(ref model) = self.material_model {
                    elements.project_with_model(
                        elem,
                        &state.pos_x,
                        &state.pos_y,
                        &state.pos_z,
                        model.as_ref(),
                    )
                } else {
                    elements.project(
                        elem,
                        &state.pos_x,
                        &state.pos_y,
                        &state.pos_z,
                    )
                };
                proj_x.push((p0.x, p1.x, p2.x));
                proj_y.push((p0.y, p1.y, p2.y));
                proj_z.push((p0.z, p1.z, p2.z));
            }

            // === LOCAL STEP: BENDING ===
            // Compute bending projection targets for the RHS assembly.
            // Both dihedral (Tier 1-2) and Discrete Shells (Tier 3) bending are
            // fully integrated into the local-global loop via the RHS.
            let (bend_targets_x, bend_targets_y, bend_targets_z) =
                if let Some(ref ds_bend) = self.ds_bending {
                    // Discrete Shells path
                    let mut btx = Vec::with_capacity(ds_bend.len());
                    let mut bty = Vec::with_capacity(ds_bend.len());
                    let mut btz = Vec::with_capacity(ds_bend.len());

                    for elem in &ds_bend.elements {
                        let (p_v0, p_v1, p_wa, p_wb) = ds_bend.project(
                            elem,
                            &state.pos_x,
                            &state.pos_y,
                            &state.pos_z,
                        );
                        btx.push((p_v0.x, p_v1.x, p_wa.x, p_wb.x));
                        bty.push((p_v0.y, p_v1.y, p_wa.y, p_wb.y));
                        btz.push((p_v0.z, p_v1.z, p_wa.z, p_wb.z));
                    }

                    (Some(btx), Some(bty), Some(btz))
                } else if let Some(ref bending) = self.bending {
                    // Dihedral path — compute projections for RHS integration
                    let mut btx = Vec::with_capacity(bending.len());
                    let mut bty = Vec::with_capacity(bending.len());
                    let mut btz = Vec::with_capacity(bending.len());

                    for elem in &bending.elements {
                        let (p_v0, p_v1, p_wa, p_wb) = bending.project(
                            elem,
                            &state.pos_x,
                            &state.pos_y,
                            &state.pos_z,
                        );
                        btx.push((p_v0.x, p_v1.x, p_wa.x, p_wb.x));
                        bty.push((p_v0.y, p_v1.y, p_wa.y, p_wb.y));
                        btz.push((p_v0.z, p_v1.z, p_wa.z, p_wb.z));
                    }

                    (Some(btx), Some(bty), Some(btz))
                } else {
                    (None, None, None)
                };

            // === GLOBAL STEP ===
            // Build bending RHS references
            let bending_rhs_x = self.build_bending_rhs(
                bend_targets_x.as_deref(),
            );
            let bending_rhs_y = self.build_bending_rhs(
                bend_targets_y.as_deref(),
            );
            let bending_rhs_z = self.build_bending_rhs(
                bend_targets_z.as_deref(),
            );

            let rhs_x = assemble_rhs(
                n, &self.mass, dt, &state.pred_x, &proj_x, elements, 0, bending_rhs_x,
            );
            let rhs_y = assemble_rhs(
                n, &self.mass, dt, &state.pred_y, &proj_y, elements, 1, bending_rhs_y,
            );
            let rhs_z = assemble_rhs(
                n, &self.mass, dt, &state.pred_z, &proj_z, elements, 2, bending_rhs_z,
            );

            // Solve A * q = rhs (three backsubstitutions)
            self.solver.solve(&rhs_x, &mut sol_x).map_err(|e| {
                vistio_types::VistioError::InvalidConfig(format!("X solve failed: {e}"))
            })?;
            self.solver.solve(&rhs_y, &mut sol_y).map_err(|e| {
                vistio_types::VistioError::InvalidConfig(format!("Y solve failed: {e}"))
            })?;
            self.solver.solve(&rhs_z, &mut sol_z).map_err(|e| {
                vistio_types::VistioError::InvalidConfig(format!("Z solve failed: {e}"))
            })?;

            // Compute convergence: ||q_new - q_old||² / ||q_old||²
            let mut diff_sq = 0.0_f64;
            let mut norm_sq = 0.0_f64;
            for i in 0..n {
                let dx = (sol_x[i] - state.pos_x[i]) as f64;
                let dy = (sol_y[i] - state.pos_y[i]) as f64;
                let dz = (sol_z[i] - state.pos_z[i]) as f64;
                diff_sq += dx * dx + dy * dy + dz * dz;

                let ox = state.pos_x[i] as f64;
                let oy = state.pos_y[i] as f64;
                let oz = state.pos_z[i] as f64;
                norm_sq += ox * ox + oy * oy + oz * oz;
            }

            final_residual = if norm_sq > 1e-12 {
                (diff_sq / norm_sq).sqrt()
            } else {
                diff_sq.sqrt()
            };

            // Update positions
            state.pos_x.copy_from_slice(&sol_x);
            state.pos_y.copy_from_slice(&sol_y);
            state.pos_z.copy_from_slice(&sol_z);

            // Enforce pinning constraints
            for i in 0..n {
                if state.inv_mass[i] == 0.0 {
                    state.pos_x[i] = state.prev_x[i];
                    state.pos_y[i] = state.prev_y[i];
                    state.pos_z[i] = state.prev_z[i];
                }
            }

            // Enforce ground plane constraint
            state.enforce_ground();

            iterations = iter + 1;

            if final_residual < self.config.tolerance {
                break;
            }
        }

        // 5. Update velocities from position difference
        state.update_velocities(dt);

        // 6. Enforce ground velocity constraints
        state.enforce_ground_velocities();

        // 7. Apply basic damping
        state.damp_velocities(self.config.damping);

        // 8. Rayleigh mass-proportional damping: v *= 1 / (1 + α_M * dt)
        if self.config.rayleigh_mass_damping > 0.0 {
            let factor = 1.0 / (1.0 + self.config.rayleigh_mass_damping * dt);
            for i in 0..n {
                if state.inv_mass[i] > 0.0 {
                    state.vel_x[i] *= factor;
                    state.vel_y[i] *= factor;
                    state.vel_z[i] *= factor;
                }
            }
        }

        let wall_time = start.elapsed().as_secs_f64();

        Ok(StepResult {
            iterations,
            final_residual,
            converged: final_residual < self.config.tolerance,
            wall_time,
        })
    }

    fn name(&self) -> &str {
        "ProjectiveDynamics"
    }
}

pub trait IpcCollisionHandler {
    fn detect_contacts(&mut self, pos_x: &[f32], pos_y: &[f32], pos_z: &[f32]) -> IpcBarrierForces;
    fn compute_ccd_step(&mut self, prev_x: &[f32], prev_y: &[f32], prev_z: &[f32], new_x: &[f32], new_y: &[f32], new_z: &[f32]) -> f32;
    /// Set the effective d_hat (barrier activation zone) for subsequent calls.
    /// Used by the solver to implement compliant contact (Phase 3.2).
    fn set_d_hat(&mut self, _d_hat: f32) {}
}

pub struct EmptyIpcHandler;
impl IpcCollisionHandler for EmptyIpcHandler {
    fn detect_contacts(&mut self, px: &[f32], _py: &[f32], _pz: &[f32]) -> IpcBarrierForces {
        IpcBarrierForces::empty(px.len())
    }
    fn compute_ccd_step(&mut self, _px: &[f32], _py: &[f32], _pz: &[f32], _nx: &[f32], _ny: &[f32], _nz: &[f32]) -> f32 {
        1.0
    }
}

impl ProjectiveDynamicsSolver {
    /// Build the bending RHS enum from optional targets, referencing internal bending data.
    #[allow(clippy::manual_map)]
    fn build_bending_rhs<'a>(
        &'a self,
        targets: Option<&'a [(f32, f32, f32, f32)]>,
    ) -> Option<BendingRhs<'a>> {
        let targets = targets?;
        if let Some(ref ds_bend) = self.ds_bending {
            Some(BendingRhs::DiscreteShells {
                data: ds_bend,
                targets,
            })
        } else if let Some(ref bending) = self.bending {
            Some(BendingRhs::Dihedral {
                data: bending,
                targets,
            })
        } else {
            None
        }
    }

    /// Advance one timestep with IPC barrier contact using Augmented Lagrangian.
    ///
    /// This is the Tier 4 solver step. It wraps the standard PD local-global
    /// loop inside an outer AL loop that enforces contact constraints:
    ///
    /// ```text
    /// for al_iter in 0..al_max_iterations:
    ///     barrier_forces = detect_and_compute_forces(positions)
    ///     for pd_iter in 0..max_iterations:
    ///         local_step(positions)
    ///         rhs = assemble_rhs(pred, projections, bending)
    ///         rhs += -(mu * barrier_grad + lambda)   // IPC forces
    ///         positions = solve(A, rhs)
    ///     lambda += mu * constraint_violation
    ///     if ||constraint_violation|| < tolerance: break
    ///     else: mu *= growth_factor
    /// ```
    ///
    /// # Arguments
    /// * `state` — Mutable simulation state
    /// * `dt` — Timestep
    /// * `detect_contacts` — Callback that detects contacts from current positions
    ///   and returns `(barrier_grad_x, barrier_grad_y, barrier_grad_z, max_violation)`.
    ///   The caller uses `IpcContactSet` from `vistio-contact` to compute these.
    /// Advance the simulation by `dt` using Tier 4 Augmented Lagrangian for IPC contacts.
    pub fn step_with_ipc<H: IpcCollisionHandler>(
        &mut self,
        state: &mut SimulationState,
        dt: f32,
        handler: &mut H,
    ) -> VistioResult<StepResult>
    {
        if !self.initialized {
            return Err(vistio_types::VistioError::InvalidConfig(
                "Solver not initialized. Call init() first.".into(),
            ));
        }

        let start = Instant::now();
        let elements = self.elements.as_ref().unwrap();
        let n = self.n;

        // 1. Save previous positions
        state.save_previous();

        // 2. Predict: q_pred = pos + dt*vel + dt²*gravity
        state.predict(dt, self.config.gravity);

        // 3. Initialize current positions from predictions
        state.pos_x.copy_from_slice(&state.pred_x);
        state.pos_y.copy_from_slice(&state.pred_y);
        state.pos_z.copy_from_slice(&state.pred_z);

        // Buffers
        let mut sol_x = vec![0.0_f32; n];
        let mut sol_y = vec![0.0_f32; n];
        let mut sol_z = vec![0.0_f32; n];

        // Compute effective d_hat (Phase 3.2: compliant contact)
        let d_hat_effective = if self.config.compliant_contact {
            self.config.barrier_d_hat * self.config.compliant_d_hat_scale
        } else {
            self.config.barrier_d_hat
        };
        // Inform the handler about the effective d_hat
        handler.set_d_hat(d_hat_effective);

        // Augmented Lagrangian penalty estimate for constraints
        let mut mu = self.config.al_mu_initial;

        let mut total_iterations = 0_u32;
        let mut final_residual = f64::MAX;
        let mut al_converged = false;

        let mut updated_forces = IpcBarrierForces::empty(n);

        // Chebyshev acceleration state (Phase 2.4)
        let _use_chebyshev = self.config.chebyshev_acceleration;
        let _rho = self.config.spectral_radius as f64;

        // Energy tracking for convergence (Phase 2.6)
        let _inv_dt2 = 1.0 / (dt * dt) as f64;
        let _prev_energy = f64::MAX;

        // ════════════════════════════════════════════════════════════
        // OUTER LOOP: Augmented Lagrangian
        // ════════════════════════════════════════════════════════════
        for _al_iter in 0..self.config.al_max_iterations {

            let mut pd_max_disp_sq = 0.0_f32;

            // === IPC BARRIER FORCES ===
            // Evaluate barrier forces ONCE per AL iteration to maintain
            // the implicit stability of the local-global solve.
            let barrier_forces = handler.detect_contacts(
                &state.pos_x, &state.pos_y, &state.pos_z,
            );

            // ════════════════════════════════════════════════════════
            // INNER LOOP: PD local-global iterations
            // ════════════════════════════════════════════════════════
            for _iter in 0..self.config.max_iterations {

                // === LOCAL STEP: MEMBRANE ===
                let mut proj_x = Vec::with_capacity(elements.len());
                let mut proj_y = Vec::with_capacity(elements.len());
                let mut proj_z = Vec::with_capacity(elements.len());

                for elem in &elements.elements {
                    let (p0, p1, p2) = if let Some(ref model) = self.material_model {
                        elements.project_with_model(
                            elem,
                            &state.pos_x,
                            &state.pos_y,
                            &state.pos_z,
                            model.as_ref(),
                        )
                    } else {
                        elements.project(
                            elem,
                            &state.pos_x,
                            &state.pos_y,
                            &state.pos_z,
                        )
                    };
                    proj_x.push((p0.x, p1.x, p2.x));
                    proj_y.push((p0.y, p1.y, p2.y));
                    proj_z.push((p0.z, p1.z, p2.z));
                }

                // === LOCAL STEP: BENDING ===
                let (bend_targets_x, bend_targets_y, bend_targets_z) =
                    if let Some(ref ds_bend) = self.ds_bending {
                        let mut btx = Vec::with_capacity(ds_bend.len());
                        let mut bty = Vec::with_capacity(ds_bend.len());
                        let mut btz = Vec::with_capacity(ds_bend.len());

                        for elem in &ds_bend.elements {
                            let (p_v0, p_v1, p_wa, p_wb) = ds_bend.project(
                                elem,
                                &state.pos_x,
                                &state.pos_y,
                                &state.pos_z,
                            );
                            btx.push((p_v0.x, p_v1.x, p_wa.x, p_wb.x));
                            bty.push((p_v0.y, p_v1.y, p_wa.y, p_wb.y));
                            btz.push((p_v0.z, p_v1.z, p_wa.z, p_wb.z));
                        }

                        (Some(btx), Some(bty), Some(btz))
                    } else if let Some(ref bending) = self.bending {
                        let mut btx = Vec::with_capacity(bending.len());
                        let mut bty = Vec::with_capacity(bending.len());
                        let mut btz = Vec::with_capacity(bending.len());

                        for elem in &bending.elements {
                            let (p_v0, p_v1, p_wa, p_wb) = bending.project(
                                elem,
                                &state.pos_x,
                                &state.pos_y,
                                &state.pos_z,
                            );
                            btx.push((p_v0.x, p_v1.x, p_wa.x, p_wb.x));
                            bty.push((p_v0.y, p_v1.y, p_wa.y, p_wb.y));
                            btz.push((p_v0.z, p_v1.z, p_wa.z, p_wb.z));
                        }

                        (Some(btx), Some(bty), Some(btz))
                    } else {
                        (None, None, None)
                    };

                // === GLOBAL STEP ===
                let bending_rhs_x = self.build_bending_rhs(
                    bend_targets_x.as_deref(),
                );
                let bending_rhs_y = self.build_bending_rhs(
                    bend_targets_y.as_deref(),
                );
                let bending_rhs_z = self.build_bending_rhs(
                    bend_targets_z.as_deref(),
                );

                let mut rhs_x = assemble_rhs(
                    n, &self.mass, dt, &state.pred_x, &proj_x, elements, 0, bending_rhs_x,
                );
                let mut rhs_y = assemble_rhs(
                    n, &self.mass, dt, &state.pred_y, &proj_y, elements, 1, bending_rhs_y,
                );
                let mut rhs_z = assemble_rhs(
                    n, &self.mass, dt, &state.pred_z, &proj_z, elements, 2, bending_rhs_z,
                );

                // === IPC BARRIER FORCES ===
                // Use the barrier gradients evaluated at the start of the AL step

                // Add barrier gradient contributions to RHS
                assemble_barrier_rhs(
                    &mut rhs_x, &barrier_forces.grad_x, mu,
                );
                assemble_barrier_rhs(
                    &mut rhs_y, &barrier_forces.grad_y, mu,
                );
                assemble_barrier_rhs(
                    &mut rhs_z, &barrier_forces.grad_z, mu,
                );

                // === LAGGED IMPLICIT FRICTION (Phase 2.3) ===
                // Temporarily disabled due to potential unit mismatch and instability
                // injection into RHS until fully validated.
                /*
                let mu_f = self.config.friction_coefficient;
                if mu_f > 0.0 && barrier_forces.active_contacts > 0 {
                    for i in 0..n {
                        if !barrier_forces.in_contact[i] || state.inv_mass[i] == 0.0 { continue; }

                        let nx = barrier_forces.contact_nx[i];
                        let ny = barrier_forces.contact_ny[i];
                        let nz = barrier_forces.contact_nz[i];
                        let n_len_sq = nx * nx + ny * ny + nz * nz;
                        if n_len_sq < 0.5 { continue; }

                        let fn_mag = (barrier_forces.grad_x[i] * nx
                            + barrier_forces.grad_y[i] * ny
                            + barrier_forces.grad_z[i] * nz).abs() * mu;

                        let v_dot_n = state.vel_x[i] * nx + state.vel_y[i] * ny + state.vel_z[i] * nz;
                        let vt_x = state.vel_x[i] - v_dot_n * nx;
                        let vt_y = state.vel_y[i] - v_dot_n * ny;
                        let vt_z = state.vel_z[i] - v_dot_n * nz;
                        let vt_mag = (vt_x * vt_x + vt_y * vt_y + vt_z * vt_z).sqrt();

                        if vt_mag > 1e-6 {
                            let friction_force = (mu_f * fn_mag).min(self.mass[i] * vt_mag / dt);
                            let scale = (friction_force / vt_mag).min(1e6); // Cap to prevent overflow
                            if scale.is_finite() {
                                rhs_x[i] -= scale * vt_x;
                                rhs_y[i] -= scale * vt_y;
                                rhs_z[i] -= scale * vt_z;
                            }
                        }
                    }
                }
                */

                // === BARRIER HESSIAN DIAGONAL PROXY (Phase 2.1) ===
                // Temporarily disabled due to instability (mathematically incorrect preconditioning).
                // It un-scales the solution by the same magnitude which destroys the physical displacement.
                // Keeping the block empty to allow compilation while we investigate a proper preconditioner.
                if barrier_forces.active_contacts > 0 {
                    // Preconditioning removed.
                }

                // Solve A * q = rhs
                self.solver.solve(&rhs_x, &mut sol_x).map_err(|e| {
                    vistio_types::VistioError::InvalidConfig(format!("X solve failed: {e}"))
                })?;
                self.solver.solve(&rhs_y, &mut sol_y).map_err(|e| {
                    vistio_types::VistioError::InvalidConfig(format!("Y solve failed: {e}"))
                })?;
                self.solver.solve(&rhs_z, &mut sol_z).map_err(|e| {
                    vistio_types::VistioError::InvalidConfig(format!("Z solve failed: {e}"))
                })?;

                // Unscale solutions by Hessian proxy
                // Disabled because preconditioning was disabled.

                // === CHEBYSHEV ACCELERATION (Phase 2.4) ===
                // Temporarily disabled due to incorrect recurrence evaluation
                // and potential divergence with non-linear barrier forces.
                /*
                if use_chebyshev && iter >= 3 && rho > 0.01 {
                    let omega = if iter == 3 {
                        2.0 / (2.0 - rho * rho)
                    } else {
                        4.0 / (4.0 - rho * rho * 2.0 / (2.0 - rho * rho))
                    };
                    let omega_f = omega.min(1.95) as f32; // Safety cap
                    for i in 0..n {
                        if state.inv_mass[i] > 0.0 {
                            sol_x[i] = state.pos_x[i] + omega_f * (sol_x[i] - state.pos_x[i]);
                            sol_y[i] = state.pos_y[i] + omega_f * (sol_y[i] - state.pos_y[i]);
                            sol_z[i] = state.pos_z[i] + omega_f * (sol_z[i] - state.pos_z[i]);
                        }
                    }
                }
                */

                // Compute displacement-based convergence
                let mut diff_sq = 0.0_f64;
                let mut norm_sq = 0.0_f64;
                for i in 0..n {
                    let dx = (sol_x[i] - state.pos_x[i]) as f64;
                    let dy = (sol_y[i] - state.pos_y[i]) as f64;
                    let dz = (sol_z[i] - state.pos_z[i]) as f64;
                    diff_sq += dx * dx + dy * dy + dz * dz;

                    let ox = state.pos_x[i] as f64;
                    let oy = state.pos_y[i] as f64;
                    let oz = state.pos_z[i] as f64;
                    norm_sq += ox * ox + oy * oy + oz * oz;
                }

                final_residual = if norm_sq > 1e-12 {
                    (diff_sq / norm_sq).sqrt()
                } else {
                    diff_sq.sqrt()
                };

                // === ENERGY-BASED CONVERGENCE (Phase 2.6) ===
                // Temporarily disabled due to incomplete total energy metric (missing elastic).
                // Returning entirely to displacement residual.

                // === ARMIJO LINE SEARCH (Phase 2.5) ===
                let mut max_alpha = handler.compute_ccd_step(
                    &state.pos_x, &state.pos_y, &state.pos_z,
                    &sol_x, &sol_y, &sol_z,
                );
                // CRISIS FIX: If we take a full step (alpha=1.0) along a collision path,
                // we evaluate the barrier at exactly distance=0.0 on the next iteration.
                // This produces an infinite gradient (NaN), exploding the fabric.
                if max_alpha < 1.0 {
                    max_alpha *= 0.8; // Restore strict safety margin
                }
                max_alpha = max_alpha.min(1.0);

                // Backtracking: evaluate energy at proposed step and halve if needed
                let armijo_c = 1e-4_f32;
                let mut alpha = max_alpha;
                for _bt in 0..4 {
                    if alpha < 0.1 { break; }
                    // Quick energy check: if barrier gradients are pointing strongly
                    // against the step direction, backtrack
                    let mut grad_dot_d = 0.0_f64;
                    for i in 0..n {
                        if state.inv_mass[i] > 0.0 {
                            let dx = sol_x[i] - state.pos_x[i];
                            let dy = sol_y[i] - state.pos_y[i];
                            let dz = sol_z[i] - state.pos_z[i];
                            grad_dot_d += (barrier_forces.grad_x[i] * dx
                                + barrier_forces.grad_y[i] * dy
                                + barrier_forces.grad_z[i] * dz) as f64;
                        }
                    }
                    // If barrier pushes strongly against step, reduce alpha
                    if grad_dot_d * (mu as f64) > (armijo_c as f64) * diff_sq * 100.0 {
                        alpha *= 0.5;
                    } else {
                        break;
                    }
                }

                let mut current_max_disp_sq = 0.0_f32;

                // Update positions with line-searched alpha
                for i in 0..n {
                    if state.inv_mass[i] > 0.0 {
                        let dx = alpha * (sol_x[i] - state.pos_x[i]);
                        let dy = alpha * (sol_y[i] - state.pos_y[i]);
                        let dz = alpha * (sol_z[i] - state.pos_z[i]);
                        state.pos_x[i] += dx;
                        state.pos_y[i] += dy;
                        state.pos_z[i] += dz;
                        let disp_sq = dx*dx + dy*dy + dz*dz;
                        if disp_sq > current_max_disp_sq {
                            current_max_disp_sq = disp_sq;
                        }
                    }
                }
                if current_max_disp_sq > pd_max_disp_sq {
                    pd_max_disp_sq = current_max_disp_sq;
                }

                // IPC barrier handles ground contact — no hard clamping needed.

                total_iterations += 1;

                if final_residual < self.config.tolerance {
                    break;
                }
            } // end inner PD loop

            // ════════════════════════════════════════════════════════
            // AL UPDATE: adjust Lagrange multipliers
            // ════════════════════════════════════════════════════════

            // Re-detect contacts to get updated constraint violation
            // for the AL multiplier update.
            updated_forces = handler.detect_contacts(
                &state.pos_x, &state.pos_y, &state.pos_z,
            );

            // Check AL convergence
            if updated_forces.max_violation < self.config.al_tolerance {
                al_converged = true;
                break;
            }

            // Increase penalty if constraints aren't sufficiently satisfied
            mu *= self.config.al_mu_growth;
        } // end outer AL loop

        // AL loops finished. We have the final constraint gradients in `updated_forces`.
        // We will do a post-stabilization velocity filter for inelasticity.

        // 5. Update velocities
        state.update_velocities(dt);

        // --- Inelastic Contact Response (Velocity Filter) ---
        // IPC barriers are purely elastic. To achieve realistic cloth draping,
        // we apply a per-vertex velocity filter using geometric contact normals:
        //   1. Remove the normal component of velocity (perfectly inelastic rebound)
        //   2. Apply Coulomb friction to the tangential component
        //   3. Apply contact-specific damping to dissipate kinetic energy
        let mu_friction = self.config.friction_coefficient;
        let contact_damp = self.config.contact_damping;
        for i in 0..n {
            if state.inv_mass[i] == 0.0 { continue; }
            if !updated_forces.in_contact[i] { continue; }

            let nx = updated_forces.contact_nx[i];
            let ny = updated_forces.contact_ny[i];
            let nz = updated_forces.contact_nz[i];

            // Check that we have a valid normal
            let n_len_sq = nx * nx + ny * ny + nz * nz;
            if n_len_sq < 0.5 { continue; } // skip if normal is degenerate

            // Decompose velocity into normal and tangential components
            let v_dot_n = state.vel_x[i] * nx + state.vel_y[i] * ny + state.vel_z[i] * nz;

            // In cloth simulation, collisions are heavily inelastic (e=0).
            // The implicit barrier resolves positional penetration purely elastically,
            // resulting in an outward bounce velocity (v_dot_n > 0).
            // To achieve a heavy, dead drape, we must remove the normal velocity
            // regardless of whether the vertex is moving into or out of the collider,
            // as long as it is within the microscopic barrier activation zone.

            // 1. Remove normal velocity completely (perfectly inelastic)
            state.vel_x[i] -= v_dot_n * nx;
            state.vel_y[i] -= v_dot_n * ny;
            state.vel_z[i] -= v_dot_n * nz;

            // 2. Apply Coulomb friction to the remaining tangential velocity
            // |v_t_new| = max(0, |v_t| - mu * |v_n|)
            // We use the magnitude of the pre-filter normal velocity as the proxy for normal impulse.
            let v_n_mag = v_dot_n.abs();
            let vt_x = state.vel_x[i];
            let vt_y = state.vel_y[i];
            let vt_z = state.vel_z[i];
            let vt_mag = (vt_x * vt_x + vt_y * vt_y + vt_z * vt_z).sqrt();

            if vt_mag > 1e-8 {
                let friction_impulse = mu_friction * v_n_mag;
                if friction_impulse >= vt_mag {
                    // Static friction: perfectly sticks tangentially
                    state.vel_x[i] = 0.0;
                    state.vel_y[i] = 0.0;
                    state.vel_z[i] = 0.0;
                } else {
                    // Kinetic friction: decelerates proportionally
                    let scale = 1.0 - friction_impulse / vt_mag;
                    state.vel_x[i] = vt_x * scale;
                    state.vel_y[i] = vt_y * scale;
                    state.vel_z[i] = vt_z * scale;
                }
            }

            // Apply contact-aware damping to vertices in contact.
            // Phase 3.3: Adaptive damping — velocity-dependent
            if self.config.adaptive_contact_damping {
                let base_damp = contact_damp;
                let max_damp = self.config.contact_damping_max;
                let v_thresh = self.config.contact_velocity_threshold;
                let v_sq = state.vel_x[i] * state.vel_x[i]
                    + state.vel_y[i] * state.vel_y[i]
                    + state.vel_z[i] * state.vel_z[i];
                let v_mag = v_sq.sqrt();
                // Blend: slow vertices get more damping, fast vertices get base damping
                let t = (1.0 - v_mag / v_thresh.max(1e-6)).clamp(0.0, 1.0);
                let adaptive_damp = base_damp + t * (max_damp - base_damp);
                state.vel_x[i] *= 1.0 - adaptive_damp;
                state.vel_y[i] *= 1.0 - adaptive_damp;
                state.vel_z[i] *= 1.0 - adaptive_damp;
            } else {
                state.vel_x[i] *= 1.0 - contact_damp;
                state.vel_y[i] *= 1.0 - contact_damp;
                state.vel_z[i] *= 1.0 - contact_damp;
            }
        }

        state.enforce_ground_velocities();

        // 6. Damping
        state.damp_velocities(self.config.damping);

        if self.config.rayleigh_mass_damping > 0.0 {
            let factor = 1.0 / (1.0 + self.config.rayleigh_mass_damping * dt);
            for i in 0..n {
                if state.inv_mass[i] > 0.0 {
                    state.vel_x[i] *= factor;
                    state.vel_y[i] *= factor;
                    state.vel_z[i] *= factor;
                }
            }
        }

        let wall_time = start.elapsed().as_secs_f64();

        Ok(StepResult {
            iterations: total_iterations,
            final_residual,
            converged: al_converged || final_residual < self.config.tolerance,
            wall_time,
        })
    }
}

/// Barrier force data passed from the contact detection system into the solver.
///
/// Computed by the caller using `IpcContactSet::compute_barrier_gradient()`.
/// Barrier forces computed by the contact detection system
/// and passed back into the solver's Augmented Lagrangian loop.
#[derive(Debug, Clone)]
pub struct IpcBarrierForces {
    /// Gradient of the barrier energy w.r.t vertex x-coordinates
    pub grad_x: Vec<f32>,
    /// Gradient of the barrier energy w.r.t vertex y-coordinates
    pub grad_y: Vec<f32>,
    /// Gradient of the barrier energy w.r.t vertex z-coordinates
    pub grad_z: Vec<f32>,
    /// Maximum constraint violation across all active contacts
    pub max_violation: f32,
    /// Number of active IPC contacts pushing on the system
    pub active_contacts: usize,

    // ─── Per-vertex contact information for velocity filter ───

    /// Per-vertex contact normal (x component). Normalized.
    /// Zero for vertices not in contact.
    pub contact_nx: Vec<f32>,
    /// Per-vertex contact normal (y component). Normalized.
    pub contact_ny: Vec<f32>,
    /// Per-vertex contact normal (z component). Normalized.
    pub contact_nz: Vec<f32>,
    /// Whether each vertex is currently in contact with any collider.
    pub in_contact: Vec<bool>,

    // ─── Barrier Hessian diagonal (Phase 2) ──────────────────

    /// Per-vertex barrier Hessian diagonal: κ · ∂²b/∂d² · (∂d/∂x)².
    /// Used as a Jacobi preconditioner to compensate for the system matrix
    /// not including barrier stiffness.
    pub hessian_diag: Vec<f32>,
}

impl IpcBarrierForces {
    pub fn empty(n_vertices: usize) -> Self {
        Self {
            grad_x: vec![0.0; n_vertices],
            grad_y: vec![0.0; n_vertices],
            grad_z: vec![0.0; n_vertices],
            max_violation: 0.0,
            active_contacts: 0,
            contact_nx: vec![0.0; n_vertices],
            contact_ny: vec![0.0; n_vertices],
            contact_nz: vec![0.0; n_vertices],
            in_contact: vec![false; n_vertices],
            hessian_diag: vec![0.0; n_vertices],
        }
    }
}

/// Compute an area-weighted lumped mass matrix (stored as a vector).
/// Distributes 1/3 of each triangle's mass to its three vertices.
/// Pinned vertices receive an extremely large mass (1e8) so they are
/// mathematically anchored in the implicit system matrix.
fn compute_lumped_masses(n: usize, elements: &ElementData, density_gsm: f32, pinned: &[bool]) -> Vec<f32> {
    let mut mass = vec![0.0; n];
    let density_kgm2 = density_gsm / 1000.0;

    for elem in &elements.elements {
        let tri_mass = elem.rest_area * density_kgm2;
        let third_mass = tri_mass / 3.0;

        for &idx in &elem.indices {
            mass[idx] += third_mass;
        }
    }

    // Ensure no zero masses for floating vertices, and infinite mass for pinned
    for (i, m) in mass.iter_mut().enumerate() {
        if pinned[i] {
            *m = 1e8; // Infinite mass for the implicit solver
        } else if *m < 1e-8 {
            *m = 1e-8;
        }
    }

    mass
}
