//! Strongly-typed identifiers for simulation entities.
//!
//! Newtype wrappers prevent accidental mixing of particle indices
//! with triangle indices or material indices.

use serde::{Deserialize, Serialize};

/// Index into the particle (vertex) arrays.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ParticleId(pub u32);

/// Index into the triangle array.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct TriangleId(pub u32);

/// Index into the material database.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct MaterialId(pub u16);

impl ParticleId {
    /// Returns the raw index as `usize` for array indexing.
    #[inline]
    pub fn index(self) -> usize {
        self.0 as usize
    }
}

impl TriangleId {
    #[inline]
    pub fn index(self) -> usize {
        self.0 as usize
    }
}

impl MaterialId {
    #[inline]
    pub fn index(self) -> usize {
        self.0 as usize
    }
}

impl From<u32> for ParticleId {
    fn from(val: u32) -> Self {
        Self(val)
    }
}

impl From<u32> for TriangleId {
    fn from(val: u32) -> Self {
        Self(val)
    }
}

impl From<u16> for MaterialId {
    fn from(val: u16) -> Self {
        Self(val)
    }
}
