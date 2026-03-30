//! Sheet-type benchmark scenarios.

use vistio_mesh::generators::quad_grid;
use vistio_solver::SolverConfig;

use super::{Scenario, ScenarioKind};

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
}
