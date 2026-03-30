//! JSON frame exporter — writes per-frame mesh data for visual inspection.
//!
//! Implements the `Renderer` trait. Captures vertex positions and triangle
//! indices at each frame, then serializes the entire animation to a JSON
//! file on `finalize()`. The output can be loaded by the companion
//! HTML/Three.js viewer for live visual simulation testing.

use serde::Serialize;
use vistio_mesh::TriangleMesh;
use vistio_types::VistioResult;

use crate::renderer::{RenderFrame, Renderer};

/// A single frame of captured mesh data.
#[derive(Serialize)]
struct FrameData {
    timestep: u32,
    positions: Vec<f32>, // Interleaved [x0,y0,z0, x1,y1,z1, ...]
}

/// Complete animation data for JSON export.
#[derive(Serialize)]
struct AnimationData {
    vertex_count: usize,
    triangle_count: usize,
    indices: Vec<u32>,
    frames: Vec<FrameData>,
}

/// Exports simulation frames to a JSON file for visual inspection.
///
/// Usage:
/// ```text
/// let mut exporter = JsonFrameExporter::new("output.json");
/// exporter.init(&mesh)?;
/// // ... run simulation, calling submit_frame() each step ...
/// exporter.finalize()?; // Writes the JSON file
/// ```
pub struct JsonFrameExporter {
    output_path: String,
    indices: Vec<u32>,
    vertex_count: usize,
    triangle_count: usize,
    frames: Vec<FrameData>,
}

impl JsonFrameExporter {
    /// Creates a new exporter that will write to the given path.
    pub fn new(output_path: &str) -> Self {
        Self {
            output_path: output_path.to_string(),
            indices: Vec::new(),
            vertex_count: 0,
            triangle_count: 0,
            frames: Vec::new(),
        }
    }
}

impl Renderer for JsonFrameExporter {
    fn init(&mut self, mesh: &TriangleMesh) -> VistioResult<()> {
        self.vertex_count = mesh.vertex_count();
        self.triangle_count = mesh.triangle_count();
        self.indices = mesh.indices.clone();
        Ok(())
    }

    fn submit_frame(&mut self, frame: &RenderFrame) -> VistioResult<()> {
        let n = frame.pos_x.len();
        let mut positions = Vec::with_capacity(n * 3);
        for i in 0..n {
            positions.push(frame.pos_x[i]);
            positions.push(frame.pos_y[i]);
            positions.push(frame.pos_z[i]);
        }
        self.frames.push(FrameData {
            timestep: frame.timestep,
            positions,
        });
        Ok(())
    }

    fn finalize(&mut self) -> VistioResult<()> {
        let data = AnimationData {
            vertex_count: self.vertex_count,
            triangle_count: self.triangle_count,
            indices: self.indices.clone(),
            frames: std::mem::take(&mut self.frames),
        };
        let json = serde_json::to_string(&data).map_err(|e| {
            vistio_types::VistioError::Serialization(format!("JSON serialization failed: {e}"))
        })?;
        std::fs::write(&self.output_path, json)?;
        Ok(())
    }

    fn name(&self) -> &str {
        "json_exporter"
    }

    fn frame_count(&self) -> u32 {
        self.frames.len() as u32
    }
}
