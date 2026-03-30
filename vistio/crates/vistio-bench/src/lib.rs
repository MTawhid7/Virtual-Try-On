//! # vistio-bench
//!
//! Benchmark suite for the Vistio simulation engine.
//!
//! Provides 3 procedural benchmark scenarios, metric collection,
//! and CSV/JSON export for regression tracking.

pub mod metrics;
pub mod runner;
pub mod scenarios;

pub use metrics::BenchmarkMetrics;
pub use runner::BenchmarkRunner;
pub use scenarios::{Scenario, ScenarioKind};
