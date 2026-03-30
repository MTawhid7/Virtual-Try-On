// physics/src/collision/spatial/mod.rs

pub mod static_grid;
pub mod dynamic;

pub use static_grid::StaticSpatialHash;
pub use dynamic::HierarchicalSpatialHash;