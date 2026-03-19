//! Folding and bending benchmark scenarios.

use vistio_mesh::generators::quad_grid;
use vistio_solver::SolverConfig;

use super::{Scenario, ScenarioKind};

impl Scenario {
    pub fn cantilever_bending() -> Self {
        // ASTM D1388 cantilever bending test geometry:
        // - Narrow strip: 2.5cm wide × 25cm long
        // - 40% (10cm) supported on the box, 60% (15cm) overhang
        // - Silk charmeuse for visible droop
        let cols = 6;  // width (narrow strip)
        let rows = 30; // length
        let mut garment = vistio_mesh::generators::quad_grid(cols, rows, 0.025, 0.25);
        let n = garment.vertex_count();
        for i in 0..n {
            // Map Y (originally [−0.125, +0.125]) to Z as the strip direction.
            // Support zone: Z ∈ [−0.10, 0.0], Overhang: Z ∈ [0.0, +0.15]
            // The box ledge ends at Z = 0.0.
            garment.pos_z[i] = garment.pos_y[i] + 0.025;
            // Place slightly above box top (y=0.499 → box top, cloth at 0.501)
            garment.pos_y[i] = 0.501;
        }
        let mut pinned = vec![false; n];
        // Pin only the vertices in the support zone (z <= 0.0, i.e., on the box).
        for (i, p) in pinned.iter_mut().enumerate().take(n) {
            if garment.pos_z[i] <= 0.0 {
                *p = true;
            }
        }
        Self {
            kind: ScenarioKind::CantileverBending,
            garment,
            body: None,
            pinned,
            config: SolverConfig {
                ipc_enabled: true,
                barrier_d_hat: 1e-4,
                barrier_kappa: 0.0,
                al_max_iterations: 10,
                ..Default::default()
            },
            timesteps: 300,
            dt: 1.0 / 60.0,
            vertex_mass: 0.001,
            material: Some(vistio_material::FabricProperties::silk_charmeuse()),
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
