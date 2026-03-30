//! Integration tests for vistio-gpu.

use vistio_gpu::backend::{CpuFallback, GpuBackend};
use vistio_gpu::buffers::ComputeBuffer;

// ─── Buffer Tests ─────────────────────────────────────────────

#[test]
fn buffer_zeros() {
    let buf = ComputeBuffer::zeros(100);
    assert_eq!(buf.len(), 100);
    assert!(buf.as_slice().iter().all(|&x| x == 0.0));
}

#[test]
fn buffer_from_data() {
    let buf = ComputeBuffer::from_data(vec![1.0, 2.0, 3.0]);
    assert_eq!(buf.len(), 3);
    assert_eq!(buf.as_slice(), &[1.0, 2.0, 3.0]);
}

#[test]
fn buffer_copy() {
    let mut buf = ComputeBuffer::zeros(3);
    buf.copy_from_slice(&[7.0, 8.0, 9.0]);
    assert_eq!(buf.as_slice(), &[7.0, 8.0, 9.0]);
}

// ─── CpuFallback Tests ───────────────────────────────────────

#[test]
fn cpu_init() {
    let mut backend = CpuFallback::new();
    assert!(backend.init().is_ok());
    assert_eq!(backend.name(), "cpu_fallback");
    assert!(!backend.is_gpu());
}

#[test]
fn cpu_axpy() {
    let backend = CpuFallback::new();
    let a = ComputeBuffer::from_data(vec![1.0, 2.0, 3.0]);
    let b = ComputeBuffer::from_data(vec![4.0, 5.0, 6.0]);
    let mut out = ComputeBuffer::zeros(3);

    backend.axpy(&a, &b, 2.0, &mut out).unwrap();
    // out = a + 2*b = [1+8, 2+10, 3+12] = [9, 12, 15]
    assert_eq!(out.as_slice(), &[9.0, 12.0, 15.0]);
}

#[test]
fn cpu_axpy_length_mismatch() {
    let backend = CpuFallback::new();
    let a = ComputeBuffer::from_data(vec![1.0, 2.0]);
    let b = ComputeBuffer::from_data(vec![4.0, 5.0, 6.0]);
    let mut out = ComputeBuffer::zeros(3);

    assert!(backend.axpy(&a, &b, 1.0, &mut out).is_err());
}

#[test]
fn cpu_dot() {
    let backend = CpuFallback::new();
    let a = ComputeBuffer::from_data(vec![1.0, 2.0, 3.0]);
    let b = ComputeBuffer::from_data(vec![4.0, 5.0, 6.0]);

    let result = backend.dot(&a, &b).unwrap();
    // 1*4 + 2*5 + 3*6 = 32
    assert!((result - 32.0).abs() < 1e-10);
}

#[test]
fn cpu_fill() {
    let backend = CpuFallback::new();
    let mut buf = ComputeBuffer::zeros(5);
    backend.fill(&mut buf, std::f32::consts::PI);
    assert!(buf.as_slice().iter().all(|&x| (x - std::f32::consts::PI).abs() < 1e-6));
}
