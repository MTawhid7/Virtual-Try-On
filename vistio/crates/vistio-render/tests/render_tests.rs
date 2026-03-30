//! Integration tests for vistio-render.

use vistio_mesh::generators::quad_grid;
use vistio_render::renderer::{HeadlessRenderer, RenderFrame, Renderer};

#[test]
fn headless_init() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let mut renderer = HeadlessRenderer::new();
    renderer.init(&mesh).unwrap();
    assert_eq!(renderer.name(), "headless");
    assert_eq!(renderer.frame_count(), 0);
}

#[test]
fn headless_submit_frames() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let mut renderer = HeadlessRenderer::new();
    renderer.init(&mesh).unwrap();

    let frame = RenderFrame::from_positions(
        0,
        &mesh.pos_x,
        &mesh.pos_y,
        &mesh.pos_z,
    );
    renderer.submit_frame(&frame).unwrap();
    renderer.submit_frame(&frame).unwrap();
    assert_eq!(renderer.frame_count(), 2);
}

#[test]
fn headless_finalize() {
    let mut renderer = HeadlessRenderer::new();
    renderer.finalize().unwrap();
}

#[test]
fn render_frame_from_positions() {
    let frame = RenderFrame::from_positions(
        42,
        &[1.0, 2.0, 3.0],
        &[4.0, 5.0, 6.0],
        &[7.0, 8.0, 9.0],
    );
    assert_eq!(frame.timestep, 42);
    assert_eq!(frame.pos_x.len(), 3);
}
