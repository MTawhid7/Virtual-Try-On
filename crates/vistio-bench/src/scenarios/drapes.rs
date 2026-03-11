//! Drape-type benchmark scenarios.

use vistio_mesh::generators::{quad_grid, uv_sphere};
use vistio_solver::SolverConfig;

use super::{Scenario, ScenarioKind};

impl Scenario {
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
            ipc_enabled: true,
            // 1e-4 gives ~1cm activation zone for close-contact draping.
            // CCD line search prevents barrier overshoot during free-fall.
            barrier_d_hat: 1e-4,
            barrier_kappa: 0.0, // adaptive estimation
            al_max_iterations: 15,
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
            material: Some(vistio_material::FabricProperties::cotton_twill()),
        }
    }

    /// Create the Cusick drape scenario.
    ///
    /// ISO 9073-9 Cusick drape test: circular specimen of radius 15cm
    /// drops onto a 9cm-radius cylinder pedestal.
    pub fn cusick_drape() -> Self {
        // Higher resolution for realistic fold formation around cylinder edge.
        // 48×15 gives enough elements to buckle and form radial pleats.
        let mut garment = vistio_mesh::generators::circular_grid(0.15, 48, 15);
        let n = garment.vertex_count();
        for i in 0..n {
            garment.pos_z[i] = garment.pos_y[i];
            garment.pos_y[i] = 0.5;
        }
        let config = SolverConfig {
            ipc_enabled: true,
            // Larger barrier zone for cylinder contact stability
            barrier_d_hat: 1e-4,
            barrier_kappa: 0.0,
            al_max_iterations: 15,
            ..Default::default()
        };

        Self {
            kind: ScenarioKind::CusickDrape,
            garment,
            body: None,
            pinned: vec![false; n],
            config,
            timesteps: 300,
            dt: 1.0 / 60.0,
            vertex_mass: 0.001,
            material: Some(vistio_material::FabricProperties::jersey_knit()),
        }
    }
}
