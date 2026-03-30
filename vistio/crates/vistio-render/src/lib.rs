//! # vistio-render
//!
//! Pluggable rendering abstraction for Vistio.
//!
//! Provides a `Renderer` trait with a `HeadlessRenderer` stub,
//! a `JsonFrameExporter` for JSON output, and a `RerunSink` for
//! live visual simulation testing via the Rerun viewer.

pub mod json_exporter;
pub mod renderer;
pub use json_exporter::JsonFrameExporter;
pub use renderer::{HeadlessRenderer, RenderFrame, Renderer};
