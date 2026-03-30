//! Physical fabric properties.
//!
//! These parameters correspond to measurable quantities from the
//! Kawabata Evaluation System (KES) and map directly to simulation
//! constitutive model coefficients.

use serde::{Deserialize, Serialize};

/// Physical properties of a fabric material.
///
/// Parameters are based on the Kawabata Evaluation System (KES)
/// and map directly to simulation constitutive model coefficients:
///
/// | KES Instrument | Property | Field |
/// |---|---|---|
/// | KES-FB1 (Tensile) | Extension stiffness | `stretch_stiffness_warp`, `stretch_stiffness_weft` |
/// | KES-FB1 (Shear) | Shear stiffness | `shear_stiffness` |
/// | KES-FB2 (Bending) | Bending rigidity | `bending_stiffness_warp`, `bending_stiffness_weft` |
/// | KES-FB3 (Compression) | Thickness | `thickness` |
/// | KES-FB4 (Surface) | Friction | `friction` |
/// | Weight | Areal density | `density` |
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FabricProperties {
    /// Human-readable name (e.g., "Cotton Twill 200gsm").
    pub name: String,

    /// Areal density in g/m² (grams per square meter).
    /// Controls vertex mass. Heavier fabrics drape differently.
    pub density: f32,

    /// Stretch stiffness along the warp (vertical/lengthwise) direction.
    /// Higher = more resistance to stretching. Range: 0.0–1.0 (normalized).
    pub stretch_stiffness_warp: f32,

    /// Stretch stiffness along the weft (horizontal/crosswise) direction.
    pub stretch_stiffness_weft: f32,

    /// Shear stiffness (resistance to parallelogram deformation).
    pub shear_stiffness: f32,

    /// Bending stiffness along the warp direction.
    /// Controls fold sharpness. Low = flowing, High = stiff.
    pub bending_stiffness_warp: f32,

    /// Bending stiffness along the weft direction.
    pub bending_stiffness_weft: f32,

    /// Fabric thickness in meters. Used for contact separation.
    pub thickness: f32,

    /// Coulomb friction coefficient (0.0–1.0).
    pub friction: f32,

    /// Damping coefficient (0.0–1.0). Higher = more energy dissipation.
    pub damping: f32,
}

impl FabricProperties {
    /// Returns the average stretch stiffness (isotropic approximation).
    pub fn avg_stretch_stiffness(&self) -> f32 {
        (self.stretch_stiffness_warp + self.stretch_stiffness_weft) / 2.0
    }

    /// Returns the average bending stiffness (isotropic approximation).
    pub fn avg_bending_stiffness(&self) -> f32 {
        (self.bending_stiffness_warp + self.bending_stiffness_weft) / 2.0
    }

    /// Returns the mass per vertex for a mesh with `n` vertices
    /// covering `total_area` square meters.
    pub fn mass_per_vertex(&self, n: usize, total_area: f32) -> f32 {
        // density is g/m², convert to kg/m² then distribute
        (self.density / 1000.0) * total_area / n as f32
    }

    /// Returns true if the fabric has anisotropic stretch stiffness (warp ≠ weft).
    ///
    /// A ratio difference > 5% is considered anisotropic.
    pub fn is_anisotropic(&self) -> bool {
        let ratio = if self.stretch_stiffness_weft > 1e-8 {
            self.stretch_stiffness_warp / self.stretch_stiffness_weft
        } else {
            1.0
        };
        (ratio - 1.0).abs() > 0.05
    }

    /// Returns the ratio of warp to weft stretch stiffness.
    ///
    /// Values > 1.0 mean warp is stiffer; < 1.0 means weft is stiffer.
    pub fn warp_weft_ratio(&self) -> f32 {
        if self.stretch_stiffness_weft > 1e-8 {
            self.stretch_stiffness_warp / self.stretch_stiffness_weft
        } else {
            1.0
        }
    }
}
