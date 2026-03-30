//! Projective Dynamics solver — the core Tier 1/2/3/4 solver.
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
//!
//! ## Module Structure
//!
//! - `mod.rs` — Solver struct, initialization methods, `build_bending_rhs()`
//! - `step.rs` — `SolverStrategy` trait impl (`init`, `step`, `name`)
//! - `ipc.rs` — IPC types and traits (`IpcCollisionHandler`, `IpcBarrierForces`)
//! - `al_step.rs` — `step_with_ipc()` (Tier 4 Augmented Lagrangian + PD inner loop)
//! - `velocity_filter.rs` — Post-solve contact velocity filtering and damping
//! - `mass.rs` — Lumped mass computation

mod mass;
pub mod ipc;
mod step;
mod al_step;
mod velocity_filter;

pub use ipc::{IpcCollisionHandler, IpcBarrierForces, EmptyIpcHandler};

use vistio_material::ConstitutiveModel;
use vistio_material::FabricProperties;
use vistio_math::faer_solver::FaerSolver;
use vistio_math::sparse::SparseSolver;
use vistio_mesh::TriangleMesh;
use vistio_mesh::topology::Topology;
use vistio_types::VistioResult;

use crate::assembly::{assemble_system_matrix, BendingModel, BendingRhs};
use crate::bending::BendingData;
use crate::config::SolverConfig;
use crate::discrete_shells::DiscreteShellsBendingData;
use crate::element::ElementData;

use mass::compute_lumped_masses;

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
    pub(crate) elements: Option<ElementData>,
    /// Legacy dihedral bending data (Tier 1-2).
    pub(crate) bending: Option<BendingData>,
    /// Discrete Shells bending data (Tier 3+).
    pub(crate) ds_bending: Option<DiscreteShellsBendingData>,
    /// Sparse Cholesky solver with cached factorization.
    pub(crate) solver: FaerSolver,
    /// Configuration snapshot from init().
    pub(crate) config: SolverConfig,
    /// Whether init() has been called successfully.
    pub(crate) initialized: bool,
    /// Cached per-vertex mass.
    pub(crate) mass: Vec<f32>,
    /// Number of vertices.
    pub(crate) n: usize,
    /// Optional pluggable constitutive model (Tier 2+).
    /// When `Some`, the local step delegates to this model.
    /// When `None`, falls back to hardcoded ARAP.
    pub(crate) material_model: Option<Box<dyn ConstitutiveModel>>,
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

impl ProjectiveDynamicsSolver {
    /// Build the bending RHS enum from optional targets, referencing internal bending data.
    #[allow(clippy::manual_map)]
    pub(crate) fn build_bending_rhs<'a>(
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
}
