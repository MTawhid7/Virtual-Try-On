//! GPU/CPU compute buffer abstraction.
//!
//! Provides a unified buffer type that can represent either
//! CPU-side data or GPU-managed storage.

/// A compute buffer that can hold data on CPU or GPU.
///
/// In the CPU fallback, this is simply a `Vec<f32>`.
/// In the GPU backend, this wraps a `wgpu::Buffer` handle.
#[derive(Debug, Clone)]
pub struct ComputeBuffer {
    /// CPU-side data.
    data: Vec<f32>,
    /// Number of elements (not bytes).
    len: usize,
}

impl ComputeBuffer {
    /// Creates a new buffer filled with zeros.
    pub fn zeros(len: usize) -> Self {
        Self {
            data: vec![0.0; len],
            len,
        }
    }

    /// Creates a buffer from existing data.
    pub fn from_data(data: Vec<f32>) -> Self {
        let len = data.len();
        Self { data, len }
    }

    /// Returns the number of elements.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Returns true if the buffer is empty.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns a slice of the CPU-side data.
    pub fn as_slice(&self) -> &[f32] {
        &self.data
    }

    /// Returns a mutable slice of the CPU-side data.
    pub fn as_mut_slice(&mut self) -> &mut [f32] {
        &mut self.data
    }

    /// Copies data from a slice into the buffer.
    pub fn copy_from_slice(&mut self, src: &[f32]) {
        self.data[..src.len()].copy_from_slice(src);
    }
}
