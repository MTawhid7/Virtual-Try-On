//! GPU compute backend trait and CPU fallback.
//!
//! The [`GpuBackend`] trait defines the interface for dispatching
//! parallel compute workloads. The [`CpuFallback`] implementation
//! executes sequentially on CPU, serving as a reference for correctness.

use crate::buffers::ComputeBuffer;
use vistio_types::{VistioError, VistioResult};

/// Trait for GPU/CPU compute backends.
///
/// The solver dispatches parallel workloads through this trait,
/// enabling transparent GPU acceleration. All methods accept
/// `ComputeBuffer` references.
///
/// # Implementations
/// - [`CpuFallback`] — Sequential CPU reference (always available)
/// - `WgpuBackend` — GPU compute via wgpu (Tier 1)
pub trait GpuBackend: Send {
    /// Initialize the backend. Called once at startup.
    fn init(&mut self) -> VistioResult<()>;

    /// Returns the backend name (e.g., "cpu_fallback", "wgpu_metal").
    fn name(&self) -> &str;

    /// Apply a per-element operation: `out[i] = a[i] + scale * b[i]`.
    ///
    /// This is the fundamental AXPY operation used by the solver
    /// for position/velocity updates.
    fn axpy(
        &self,
        a: &ComputeBuffer,
        b: &ComputeBuffer,
        scale: f32,
        out: &mut ComputeBuffer,
    ) -> VistioResult<()>;

    /// Compute the dot product of two buffers.
    fn dot(&self, a: &ComputeBuffer, b: &ComputeBuffer) -> VistioResult<f64>;

    /// Fill a buffer with a constant value.
    fn fill(&self, buffer: &mut ComputeBuffer, value: f32);

    /// Returns true if the backend supports GPU compute.
    fn is_gpu(&self) -> bool;
}

/// CPU fallback backend — sequential reference implementation.
///
/// Always available, used for:
/// - Platforms without GPU support
/// - Correctness validation (GPU results should match CPU)
/// - Small meshes where GPU overhead isn't worthwhile
pub struct CpuFallback {
    initialized: bool,
}

impl CpuFallback {
    /// Creates a new CPU fallback backend.
    pub fn new() -> Self {
        Self { initialized: false }
    }
}

impl Default for CpuFallback {
    fn default() -> Self {
        Self::new()
    }
}

impl GpuBackend for CpuFallback {
    fn init(&mut self) -> VistioResult<()> {
        self.initialized = true;
        Ok(())
    }

    fn name(&self) -> &str {
        "cpu_fallback"
    }

    fn axpy(
        &self,
        a: &ComputeBuffer,
        b: &ComputeBuffer,
        scale: f32,
        out: &mut ComputeBuffer,
    ) -> VistioResult<()> {
        if a.len() != b.len() || a.len() != out.len() {
            return Err(VistioError::Gpu(
                "AXPY buffer length mismatch".into(),
            ));
        }

        let a_data = a.as_slice();
        let b_data = b.as_slice();
        let out_data = out.as_mut_slice();

        for i in 0..a.len() {
            out_data[i] = a_data[i] + scale * b_data[i];
        }

        Ok(())
    }

    fn dot(&self, a: &ComputeBuffer, b: &ComputeBuffer) -> VistioResult<f64> {
        if a.len() != b.len() {
            return Err(VistioError::Gpu(
                "Dot product buffer length mismatch".into(),
            ));
        }

        let a_data = a.as_slice();
        let b_data = b.as_slice();
        let mut sum = 0.0f64;
        for i in 0..a.len() {
            sum += a_data[i] as f64 * b_data[i] as f64;
        }

        Ok(sum)
    }

    fn fill(&self, buffer: &mut ComputeBuffer, value: f32) {
        for x in buffer.as_mut_slice() {
            *x = value;
        }
    }

    fn is_gpu(&self) -> bool {
        false
    }
}
