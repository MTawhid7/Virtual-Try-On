//! Benchmark scenarios — procedural mesh + pinning + config for each test case.
//!
//! Scenarios are organized by type:
//! - `sheets` — Flat sheet scenarios (hanging_sheet, uniaxial_stretch)
//! - `drapes` — Drape scenarios (sphere_drape, cusick_drape)
//! - `folds` — Folding/bending scenarios (cantilever_bending, self_fold)

mod sheets;
mod drapes;
mod folds;

use serde::{Deserialize, Serialize};

use vistio_material::FabricProperties;
use vistio_mesh::TriangleMesh;
use vistio_solver::SolverConfig;

/// Which benchmark scenario to run.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ScenarioKind {
    /// Cloth pinned at top edge, hanging under gravity.
    HangingSheet,
    /// Cloth draped over a sphere.
    SphereDrape,
    CusickDrape,
    CantileverBending,
    UniaxialStretch,
    /// Cloth dropped diagonally onto ground, folding onto itself.
    SelfFold,
}

impl ScenarioKind {
    /// Returns all scenario kinds.
    pub fn all() -> &'static [ScenarioKind] {
        &[
            ScenarioKind::HangingSheet,
            ScenarioKind::SphereDrape,
            ScenarioKind::CusickDrape,
            ScenarioKind::CantileverBending,
            ScenarioKind::UniaxialStretch,
            ScenarioKind::SelfFold,
        ]
    }

    /// Returns a human-readable name.
    pub fn name(&self) -> &'static str {
        match self {
            ScenarioKind::HangingSheet => "hanging_sheet",
            ScenarioKind::SphereDrape => "sphere_drape",
            ScenarioKind::CusickDrape => "cusick_drape",
            ScenarioKind::CantileverBending => "cantilever_bending",
            ScenarioKind::UniaxialStretch => "uniaxial_stretch",
            ScenarioKind::SelfFold => "self_fold",
        }
    }
}

/// A fully specified benchmark scenario.
pub struct Scenario {
    /// Scenario type.
    pub kind: ScenarioKind,
    /// Garment mesh.
    pub garment: TriangleMesh,
    /// Optional obstacle mesh (sphere for drape scenario).
    pub body: Option<TriangleMesh>,
    /// Per-vertex pinning.
    pub pinned: Vec<bool>,
    /// Solver configuration.
    pub config: SolverConfig,
    /// Number of timesteps to simulate.
    pub timesteps: u32,
    /// Timestep size (seconds).
    pub dt: f32,
    /// Per-vertex mass (kg). Used when `material` is `None`.
    pub vertex_mass: f32,
    /// Optional material properties. When set, the runner uses
    /// `init_with_material()` for material-aware simulation.
    pub material: Option<FabricProperties>,
}

impl Scenario {
    pub fn from_kind(kind: ScenarioKind) -> Self {
        match kind {
            ScenarioKind::HangingSheet => Self::hanging_sheet(),
            ScenarioKind::SphereDrape => Self::sphere_drape(),
            ScenarioKind::CusickDrape => Self::cusick_drape(),
            ScenarioKind::CantileverBending => Self::cantilever_bending(),
            ScenarioKind::UniaxialStretch => Self::uniaxial_stretch(),
            ScenarioKind::SelfFold => Self::self_fold(),
        }
    }

    /// Set a material for this scenario, enabling material-aware simulation.
    ///
    /// When set, the runner uses `init_with_material()` instead of `init()`,
    /// deriving mass and stiffness from the material properties.
    pub fn with_material(mut self, properties: FabricProperties) -> Self {
        self.material = Some(properties);
        self
    }
}
