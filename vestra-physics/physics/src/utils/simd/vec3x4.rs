// physics/src/utils/simd/vec3x4.rs

//! SIMD-accelerated 3D vector (4 Vec3s packed in SoA layout).
//! X components in lanes 0-3, Y in lanes 0-3, Z in lanes 0-3.

#![allow(dead_code)]

use super::f32x4::F32x4;

/// SIMD-accelerated 3D vector (4 Vec3s packed in SoA layout).
/// X components in lanes 0-3, Y in lanes 0-3, Z in lanes 0-3.
#[derive(Clone, Copy)]
pub struct Vec3x4 {
    pub x: F32x4,
    pub y: F32x4,
    pub z: F32x4,
}

impl Vec3x4 {
    /// Create from 4 glam Vec4s (uses xyz, ignores w).
    #[inline(always)]
    pub fn from_vec4s(v0: glam::Vec4, v1: glam::Vec4, v2: glam::Vec4, v3: glam::Vec4) -> Self {
        Self {
            x: F32x4::new(v0.x, v1.x, v2.x, v3.x),
            y: F32x4::new(v0.y, v1.y, v2.y, v3.y),
            z: F32x4::new(v0.z, v1.z, v2.z, v3.z),
        }
    }

    /// Splat a single Vec3 to all lanes.
    #[inline(always)]
    pub fn splat(v: glam::Vec3) -> Self {
        Self {
            x: F32x4::splat(v.x),
            y: F32x4::splat(v.y),
            z: F32x4::splat(v.z),
        }
    }

    /// Element-wise subtraction.
    #[inline(always)]
    pub fn sub(self, rhs: Self) -> Self {
        Self {
            x: self.x.sub(rhs.x),
            y: self.y.sub(rhs.y),
            z: self.z.sub(rhs.z),
        }
    }

    /// Element-wise addition.
    #[inline(always)]
    pub fn add(self, rhs: Self) -> Self {
        Self {
            x: self.x.add(rhs.x),
            y: self.y.add(rhs.y),
            z: self.z.add(rhs.z),
        }
    }

    /// Multiply all components by a scalar F32x4.
    #[inline(always)]
    pub fn mul_scalar(self, s: F32x4) -> Self {
        Self {
            x: self.x.mul(s),
            y: self.y.mul(s),
            z: self.z.mul(s),
        }
    }

    /// Divide all components by a scalar F32x4.
    #[inline(always)]
    pub fn div_scalar(self, s: F32x4) -> Self {
        Self {
            x: self.x.div(s),
            y: self.y.div(s),
            z: self.z.div(s),
        }
    }

    /// Compute length squared for each of the 4 vectors.
    #[inline(always)]
    pub fn length_squared(self) -> F32x4 {
        self.x.mul(self.x)
            .add(self.y.mul(self.y))
            .add(self.z.mul(self.z))
    }

    /// Compute length for each of the 4 vectors.
    #[inline(always)]
    pub fn length(self) -> F32x4 {
        self.length_squared().sqrt()
    }

    /// Normalize each of the 4 vectors (safe, clamps minimum length).
    #[inline(always)]
    pub fn normalize_safe(self) -> Self {
        let len = self.length();
        let safe_len = len.max(F32x4::splat(1e-8));
        self.div_scalar(safe_len)
    }

    /// Extract lane as glam Vec4 (w=0).
    #[inline(always)]
    pub fn extract_lane0(self) -> glam::Vec4 {
        glam::Vec4::new(self.x.lane0(), self.y.lane0(), self.z.lane0(), 0.0)
    }

    #[inline(always)]
    pub fn extract_lane1(self) -> glam::Vec4 {
        glam::Vec4::new(self.x.lane1(), self.y.lane1(), self.z.lane1(), 0.0)
    }

    #[inline(always)]
    pub fn extract_lane2(self) -> glam::Vec4 {
        glam::Vec4::new(self.x.lane2(), self.y.lane2(), self.z.lane2(), 0.0)
    }

    #[inline(always)]
    pub fn extract_lane3(self) -> glam::Vec4 {
        glam::Vec4::new(self.x.lane3(), self.y.lane3(), self.z.lane3(), 0.0)
    }

    /// Extract lane by index.
    #[inline(always)]
    pub fn extract_lane(self, i: usize) -> glam::Vec4 {
        match i {
            0 => self.extract_lane0(),
            1 => self.extract_lane1(),
            2 => self.extract_lane2(),
            _ => self.extract_lane3(),
        }
    }
}
