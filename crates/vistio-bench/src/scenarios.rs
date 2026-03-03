//! Benchmark scenarios — procedural mesh + pinning + config for each test case.
//!
//! Two canonical scenarios for regression testing:
//! 1. **Hanging sheet** — Cloth pinned at top edge, drapes under gravity
//! 2. **Sphere drape** — Cloth falls onto a sphere
//!
//! Self-collision testing is deferred to Tier 4 (IPC barrier contact).

use serde::{Deserialize, Serialize};

use vistio_material::FabricProperties;
use vistio_mesh::generators::{quad_grid, uv_sphere};
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
    /// Create the hanging sheet scenario.
    ///
    /// A 1m × 1m cloth at 20×20 resolution, pinned along the top edge,
    /// hanging under gravity for 2 seconds at 60fps.
    pub fn hanging_sheet() -> Self {
        let cols = 20;
        let rows = 20;
        let mut garment = quad_grid(cols, rows, 1.0, 1.0);
        let n = garment.vertex_count();
        let verts_x = cols + 1;

        // Rotate to XZ plane and elevate: Y = 1.0
        for i in 0..n {
            garment.pos_z[i] = garment.pos_y[i];
            garment.pos_y[i] = 1.0;
        }

        // Pin the top row (now edge along Z)
        let mut pinned = vec![false; n];
        for item in pinned.iter_mut().take(verts_x) {
            *item = true;
        }

        Self {
            kind: ScenarioKind::HangingSheet,
            garment,
            body: None,
            pinned,
            config: SolverConfig::default(),
            timesteps: 600, // 10 seconds at 60fps
            dt: 1.0 / 60.0,
            vertex_mass: 0.002, // ~200g/m² cloth, distributed across 441 vertices
            material: None,
        }
    }

    /// Create the sphere drape scenario.
    ///
    /// A 1.5m × 1.5m cloth at 20×20 resolution falls onto
    /// a sphere of radius 0.3m centered at origin.
    pub fn sphere_drape() -> Self {
        let mut garment = quad_grid(20, 20, 1.5, 1.5);
        let n = garment.vertex_count();

        // Rotate to XZ plane and elevate: Y = 1.0
        for i in 0..n {
            garment.pos_z[i] = garment.pos_y[i];
            garment.pos_y[i] = 1.0;
        }

        let body = uv_sphere(0.3, 16, 32);

        let config = SolverConfig {
            ipc_enabled: true, // Use IPC guarantees
            ..Default::default()
        };

        Self {
            kind: ScenarioKind::SphereDrape,
            garment,
            body: Some(body),
            pinned: vec![false; n], // Nothing pinned — free fall
            config,
            timesteps: 180, // 3 seconds
            dt: 1.0 / 60.0,
            vertex_mass: 0.002,
            material: None,
        }
    }

    /// Create a scenario by kind.
    pub fn cusick_drape() -> Self {
        let mut garment = vistio_mesh::generators::circular_grid(0.30, 64, 15);
        let n = garment.vertex_count();
        for i in 0..n {
            garment.pos_z[i] = garment.pos_y[i];
            garment.pos_y[i] = 0.4;
        }
        Self {
            kind: ScenarioKind::CusickDrape,
            garment,
            body: None,
            pinned: vec![false; n],
            config: SolverConfig::default(),
            timesteps: 300,
            dt: 1.0 / 60.0,
            vertex_mass: 0.002,
            material: None,
        }
    }

    pub fn cantilever_bending() -> Self {
        let cols = 10;
        let rows = 40;
        let mut garment = vistio_mesh::generators::quad_grid(cols, rows, 0.10, 0.40);
        let n = garment.vertex_count();
        for i in 0..n {
            garment.pos_z[i] = garment.pos_y[i] + 0.20; // z: [0.0, 0.40]
            garment.pos_y[i] = 0.5; // Starts resting on the shelf at y=0.5
        }
        let mut pinned = vec![false; n];
        // Pin the first three rows (which are situated on the shelf)
        for i in 0..3 * (cols + 1) {
            pinned[i] = true;
        }
        Self {
            kind: ScenarioKind::CantileverBending,
            garment,
            body: None,
            pinned,
            config: SolverConfig::default(),
            timesteps: 300,
            dt: 1.0 / 60.0,
            vertex_mass: 0.001,
            material: None,
        }
    }

    pub fn uniaxial_stretch() -> Self {
        let cols = 15;
        let rows = 15;
        let mut garment = vistio_mesh::generators::quad_grid(cols, rows, 0.50, 0.50);
        let n = garment.vertex_count();
        for i in 0..n {
            garment.pos_z[i] = garment.pos_y[i];
            garment.pos_y[i] = 1.0;
        }
        let mut pinned = vec![false; n];
        for (i, p) in pinned.iter_mut().enumerate() {
            if garment.pos_x[i] < -0.249 { *p = true; }
            if garment.pos_x[i] >  0.249 { *p = true; }
        }
        Self {
            kind: ScenarioKind::UniaxialStretch,
            garment,
            body: None,
            pinned,
            config: SolverConfig::default(),
            timesteps: 120,
            dt: 1.0 / 60.0,
            vertex_mass: 0.0005,
            material: None,
        }
    }

    /// Create the self-fold scenario.
    ///
    /// Cloth dropped diagonally onto the ground, folding onto itself.
    pub fn self_fold() -> Self {
        let mut garment = quad_grid(20, 20, 1.0, 1.0);
        let n = garment.vertex_count();

        // Rotate at an angle and elevate
        for i in 0..n {
            let cx = garment.pos_x[i] - 0.5;
            let cy = garment.pos_y[i] - 0.5;
            garment.pos_z[i] = cx * 0.707 - cy * 0.707;
            garment.pos_y[i] = cx * 0.707 + cy * 0.707 + 1.0;
        }

        let config = SolverConfig {
            ipc_enabled: true, // Use IPC for self-collision
            ..Default::default()
        };

        Self {
            kind: ScenarioKind::SelfFold,
            garment,
            body: None,
            pinned: vec![false; n],
            config,
            timesteps: 300,
            dt: 1.0 / 60.0,
            vertex_mass: 0.002,
            material: None,
        }
    }

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
