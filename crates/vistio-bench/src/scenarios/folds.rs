//! Folding and bending benchmark scenarios.

use vistio_mesh::generators::quad_grid;
use vistio_solver::SolverConfig;

use super::{Scenario, ScenarioKind};

impl Scenario {
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
        for item in pinned.iter_mut().take(3 * (cols + 1)) {
            *item = true;
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

    /// Create the self-fold scenario.
    ///
    /// A long, narrow ribbon of cloth dropped vertically onto the ground,
    /// buckling and folding onto itself multiple times.
    pub fn self_fold() -> Self {
        // 0.2m x 2.0m ribbon, 10x50 resolution
        let mut garment = quad_grid(10, 50, 0.2, 2.0);
        let n = garment.vertex_count();

        // Elevate the ribbon and add a slight angle to encourage consistent buckling
        for i in 0..n {
            let x = garment.pos_x[i];
            let y = garment.pos_y[i]; // [-1.0, 1.0]

            // ~10 degree tilt around Z axis
            let c = 0.9848;
            let s = 0.1736;

            let rx = x * c - y * s;
            let ry = x * s + y * c;

            garment.pos_x[i] = rx;
            garment.pos_y[i] = ry + 1.2; // Elevate so the lowest point is around 0.2m above ground
            garment.pos_z[i] = 0.0;
        }

        let config = SolverConfig {
            ipc_enabled: false, // Deferred self-collision to GPU Compute (Tier 5)
            ..Default::default()
        };

        Self {
            kind: ScenarioKind::SelfFold,
            garment,
            body: None,
            pinned: vec![false; n],
            config,
            timesteps: 400, // Longer to give it time to stack
            dt: 1.0 / 60.0,
            vertex_mass: 0.00014, // ~200g/m^2 cloth for 561 vertices
            material: Some(vistio_material::FabricProperties::silk_charmeuse()),
        }
    }
}
