//! 3×2 matrix type for deformation gradients in codimensional (shell) simulation.
//!
//! In cloth simulation, triangles are 2D manifolds embedded in 3D space.
//! The deformation gradient F is a 3×2 matrix mapping from the 2D reference
//! configuration to the 3D deformed configuration.

use glam::Vec3;
use serde::{Deserialize, Serialize};

/// A 3×2 column-major matrix.
///
/// Used to represent the deformation gradient F for shell elements.
/// Columns represent the deformed edge vectors of a triangle
/// mapped through the inverse of the rest-state edge vectors.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Mat3x2 {
    /// First column (3 components).
    pub col0: Vec3,
    /// Second column (3 components).
    pub col1: Vec3,
}

impl Mat3x2 {
    /// Creates a new 3×2 matrix from two column vectors.
    #[inline]
    pub fn from_cols(col0: Vec3, col1: Vec3) -> Self {
        Self { col0, col1 }
    }

    /// The zero matrix.
    pub const ZERO: Self = Self {
        col0: Vec3::ZERO,
        col1: Vec3::ZERO,
    };

    /// Identity-like matrix (first two columns of 3×3 identity).
    pub const IDENTITY: Self = Self {
        col0: Vec3::X,
        col1: Vec3::Y,
    };

    /// Compute F^T * F (a 2×2 matrix), returned as [a, b; b, d].
    ///
    /// This is the right Cauchy-Green deformation tensor C = F^T F,
    /// which measures strain independent of rotation.
    #[inline]
    pub fn ftf(&self) -> [f32; 4] {
        let a = self.col0.dot(self.col0);
        let b = self.col0.dot(self.col1);
        let d = self.col1.dot(self.col1);

        if !a.is_finite() || !b.is_finite() || !d.is_finite() {
            return [0.0, 0.0, 0.0, 0.0];
        }

        [a, b, b, d]
    }

    /// Frobenius norm squared: ||F||_F^2 = trace(F^T F).
    #[inline]
    pub fn frobenius_norm_sq(&self) -> f32 {
        self.col0.length_squared() + self.col1.length_squared()
    }

    /// Multiply by a 2×2 matrix (column-major [a, b, c, d]):
    /// result = self * [[a, c], [b, d]]
    #[inline]
    pub fn mul_mat2(&self, m: [f32; 4]) -> Self {
        Self {
            col0: self.col0 * m[0] + self.col1 * m[1],
            col1: self.col0 * m[2] + self.col1 * m[3],
        }
    }
}

impl std::ops::Sub for Mat3x2 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self {
            col0: self.col0 - rhs.col0,
            col1: self.col1 - rhs.col1,
        }
    }
}

impl std::ops::Mul<f32> for Mat3x2 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: f32) -> Self {
        Self {
            col0: self.col0 * rhs,
            col1: self.col1 * rhs,
        }
    }
}
