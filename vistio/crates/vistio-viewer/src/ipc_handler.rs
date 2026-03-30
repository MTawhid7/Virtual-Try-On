//! IPC collision handler for the Bevy viewer.

use vistio_contact::CollisionPipeline;

pub(crate) struct ViewerIpcHandler<'a> {
    pub pipeline: &'a mut CollisionPipeline,
    pub d_hat: f32,
    pub kappa: f32,
    pub mesh: &'a vistio_mesh::TriangleMesh,
}

impl<'a> vistio_solver::pd_solver::IpcCollisionHandler for ViewerIpcHandler<'a> {
    fn detect_contacts(&mut self, px: &[f32], py: &[f32], pz: &[f32]) -> vistio_solver::pd_solver::IpcBarrierForces {
        self.pipeline.detect_ipc_contacts(px, py, pz, self.d_hat, self.kappa)
    }

    fn compute_ccd_step(&mut self, prev_x: &[f32], prev_y: &[f32], prev_z: &[f32], new_x: &[f32], new_y: &[f32], new_z: &[f32], padding: f32, alphas: &mut [f32]) -> f32 {
        self.pipeline.compute_ccd_step(&self.mesh.indices, prev_x, prev_y, prev_z, new_x, new_y, new_z, padding, alphas)
    }

    fn set_d_hat(&mut self, d_hat: f32) {
        self.d_hat = d_hat;
    }

    fn project_positions(&mut self, pos_x: &mut [f32], pos_y: &mut [f32], pos_z: &mut [f32]) {
        self.pipeline.project_positions(pos_x, pos_y, pos_z);
    }
}
