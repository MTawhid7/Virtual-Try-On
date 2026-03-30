//! Renderer trait and HeadlessRenderer stub.
//!
//! The renderer is called once per frame (or per N-frames) to present
//! the current mesh state. The headless renderer discards all frames,
//! serving as a no-op for benchmarks and CI.

use vistio_mesh::TriangleMesh;
use vistio_types::VistioResult;

/// A single render frame.
pub struct RenderFrame {
    /// Timestep this frame corresponds to.
    pub timestep: u32,
    /// Current vertex positions (borrowed from SimulationState).
    pub pos_x: Vec<f32>,
    pub pos_y: Vec<f32>,
    pub pos_z: Vec<f32>,
}

impl RenderFrame {
    /// Create a frame from position SoA buffers.
    pub fn from_positions(
        timestep: u32,
        pos_x: &[f32],
        pos_y: &[f32],
        pos_z: &[f32],
    ) -> Self {
        Self {
            timestep,
            pos_x: pos_x.to_vec(),
            pos_y: pos_y.to_vec(),
            pos_z: pos_z.to_vec(),
        }
    }
}

/// Trait for rendering simulation output.
///
/// # Implementations
/// - [`HeadlessRenderer`] — Discards frames (benchmarks, CI)
/// - `WgpuRenderer` — Real-time visualization (Tier 2)
/// - `GltfExporter` — Export frames as glTF sequences (Tier 2)
pub trait Renderer: Send {
    /// Initialize the renderer with the mesh topology.
    fn init(&mut self, mesh: &TriangleMesh) -> VistioResult<()>;

    /// Submit a frame for rendering.
    fn submit_frame(&mut self, frame: &RenderFrame) -> VistioResult<()>;

    /// Finalize rendering (flush buffers, close files, etc.).
    fn finalize(&mut self) -> VistioResult<()>;

    /// Returns the renderer name.
    fn name(&self) -> &str;

    /// Returns the number of frames submitted.
    fn frame_count(&self) -> u32;
}

/// Headless renderer — discards all frames.
///
/// Used for benchmarks and CI where no visual output is needed.
pub struct HeadlessRenderer {
    frames: u32,
}

impl HeadlessRenderer {
    /// Creates a new headless renderer.
    pub fn new() -> Self {
        Self { frames: 0 }
    }
}

impl Default for HeadlessRenderer {
    fn default() -> Self {
        Self::new()
    }
}

impl Renderer for HeadlessRenderer {
    fn init(&mut self, _mesh: &TriangleMesh) -> VistioResult<()> {
        Ok(())
    }

    fn submit_frame(&mut self, _frame: &RenderFrame) -> VistioResult<()> {
        self.frames += 1;
        Ok(())
    }

    fn finalize(&mut self) -> VistioResult<()> {
        Ok(())
    }

    fn name(&self) -> &str {
        "headless"
    }

    fn frame_count(&self) -> u32 {
        self.frames
    }
}
