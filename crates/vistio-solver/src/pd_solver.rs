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

        // Augmented Lagrangian state
        let mut lambda_x = vec![0.0_f32; n];
        let mut lambda_y = vec![0.0_f32; n];
        let mut lambda_z = vec![0.0_f32; n];
        let mut mu = self.config.al_mu_initial;

        let mut total_iterations = 0_u32;
        let mut final_residual = f64::MAX;
        let mut al_converged = false;

        let mut updated_forces = IpcBarrierForces::empty(n);

        // ════════════════════════════════════════════════════════════
        // OUTER LOOP: Augmented Lagrangian
        // ════════════════════════════════════════════════════════════
        for _al_iter in 0..self.config.al_max_iterations {
            // Detect contacts and compute barrier gradient from current positions
            let barrier_forces_init = handler.detect_contacts(
                &state.pos_x, &state.pos_y, &state.pos_z,
            );
            let barrier_forces = barrier_forces_init;

            let mut pd_max_disp_sq = 0.0_f32;

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
                // Add barrier gradient + AL multiplier contributions to RHS
                assemble_barrier_rhs(
                    &mut rhs_x, &barrier_forces.grad_x, &lambda_x, mu,
                );
                assemble_barrier_rhs(
                    &mut rhs_y, &barrier_forces.grad_y, &lambda_y, mu,
                );
                assemble_barrier_rhs(
                    &mut rhs_z, &barrier_forces.grad_z, &lambda_z, mu,
                );

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

                // Compute convergence
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

                // === AL LINE SEARCH & CCD STEP SIZE ===
                // Instead of jumping directly to sol, treat sol - pos as descent direction.
                let mut max_alpha = handler.compute_ccd_step(
                    &state.pos_x, &state.pos_y, &state.pos_z,
                    &sol_x, &sol_y, &sol_z,
                );
                if max_alpha < 1.0 { max_alpha *= 0.8; } // Safe margin
                max_alpha = max_alpha.min(1.0);


                let mut current_max_disp_sq = 0.0_f32;

                // Update positions with CCD limiting
                for i in 0..n {
                    if state.inv_mass[i] > 0.0 {
                        let dx = max_alpha * (sol_x[i] - state.pos_x[i]);
                        let dy = max_alpha * (sol_y[i] - state.pos_y[i]);
                        let dz = max_alpha * (sol_z[i] - state.pos_z[i]);
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

                state.enforce_ground();

                total_iterations += 1;

                if final_residual < self.config.tolerance {
                    break;
                }
            } // end inner PD loop

            // ════════════════════════════════════════════════════════
            // AL UPDATE: adjust Lagrange multipliers
            // ════════════════════════════════════════════════════════

            // Re-detect contacts to get updated constraint violation
            if pd_max_disp_sq < 1e-12 {
                // Resting contact optimization: if positions barely moved during the PD loop
                // (e.g., resting on the floor), re-detecting contacts is redundant.
                updated_forces = barrier_forces;
            } else {
                updated_forces = handler.detect_contacts(
                    &state.pos_x, &state.pos_y, &state.pos_z,
                );
            }

            // Update λ ← λ + μ · ∇barrier (the constraint gradient)
            for i in 0..n {
                lambda_x[i] += mu * updated_forces.grad_x[i];
                lambda_y[i] += mu * updated_forces.grad_y[i];
                lambda_z[i] += mu * updated_forces.grad_z[i];
            }

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

            // Only filter if the vertex is moving INTO the contact surface (v_dot_n < 0
            // means moving against the outward normal)
            if v_dot_n < 0.0 {
                // Remove the normal velocity component (perfectly inelastic)
                state.vel_x[i] -= v_dot_n * nx;
                state.vel_y[i] -= v_dot_n * ny;
                state.vel_z[i] -= v_dot_n * nz;

                // Coulomb friction: clamp tangential velocity
                // |f_t| ≤ μ · |f_n|, which translates to: |v_t_new| ≤ |v_t| - μ·|v_n|
                let vt_x = state.vel_x[i] - (state.vel_x[i] * nx + state.vel_y[i] * ny + state.vel_z[i] * nz) * nx;
                let vt_y = state.vel_y[i] - (state.vel_x[i] * nx + state.vel_y[i] * ny + state.vel_z[i] * nz) * ny;
                let vt_z = state.vel_z[i] - (state.vel_x[i] * nx + state.vel_y[i] * ny + state.vel_z[i] * nz) * nz;
                let vt_mag = (vt_x * vt_x + vt_y * vt_y + vt_z * vt_z).sqrt();
                let v_n_mag = (-v_dot_n).abs();

                if vt_mag > 1e-8 {
                    // Friction reduces tangential velocity by μ·|v_n|
                    let friction_impulse = mu_friction * v_n_mag;
                    if friction_impulse >= vt_mag {
                        // Static friction: tangential velocity goes to zero
                        state.vel_x[i] -= vt_x;
                        state.vel_y[i] -= vt_y;
                        state.vel_z[i] -= vt_z;
                    } else {
                        // Kinetic friction: reduce tangential velocity proportionally
                        let scale = 1.0 - friction_impulse / vt_mag;
                        state.vel_x[i] = vt_x * scale;
                        state.vel_y[i] = vt_y * scale;
                        state.vel_z[i] = vt_z * scale;
                    }
                }
            }

            // Apply contact-aware damping to vertices in contact.
            // This dissipates remaining kinetic energy for settling.
            state.vel_x[i] *= 1.0 - contact_damp;
            state.vel_y[i] *= 1.0 - contact_damp;
            state.vel_z[i] *= 1.0 - contact_damp;
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
