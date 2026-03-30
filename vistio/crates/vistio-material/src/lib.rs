//! # vistio-material
//!
//! Constitutive model abstraction and material database.
//!
//! ## Design
//!
//! The [`ConstitutiveModel`] trait defines the interface for computing
//! elastic energy projections from deformation gradients. Different
//! implementors (linear elastic, co-rotational, anisotropic) can be
//! swapped at runtime via the strategy pattern.
//!
//! The [`MaterialDatabase`] stores named fabric presets with physically
//! meaningful parameters derived from Kawabata Evaluation System (KES)
//! measurements.

pub mod anisotropic;
pub mod corotational;
pub mod database;
pub mod isotropic;
pub mod orthotropic;
pub mod properties;
pub mod traits;

pub use anisotropic::AnisotropicCoRotationalModel;
pub use corotational::CoRotationalModel;
pub use database::MaterialDatabase;
pub use isotropic::IsotropicLinearModel;
pub use orthotropic::OrthotropicLinearModel;
pub use properties::FabricProperties;
pub use traits::ConstitutiveModel;
