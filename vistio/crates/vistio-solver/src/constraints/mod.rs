//! Constraint modules for the Projective Dynamics solver.
//!
//! Groups the FEM element data, system matrix assembly, and bending
//! constraint models into a single logically organized module.
//!
//! - `element` — FEM triangle elements with co-rotational/ARAP projections
//! - `assembly` — System matrix and RHS vector construction
//! - `bending` — Dihedral bending constraints (Tier 1-2)
//! - `discrete_shells` — Cotangent-weighted Discrete Shells bending (Tier 3+)

pub mod element;
pub mod assembly;
pub mod bending;
pub mod discrete_shells;
