"""
Live visualization module using Taichi's GUI.
"""

import time
import numpy as np

from simulation.core.config import SimConfig
from simulation.core.engine import SimulationEngine
from simulation.core.state import ParticleState


def visualize_simulation(
    engine: SimulationEngine,
    state: ParticleState,
    config: SimConfig,
    sphere_center=None,
    sphere_radius=None,
    body_mesh_path=None,
) -> None:
    """Launch Taichi GUI window and run the simulation live."""
    import taichi as ti
    
    print("\n  [Live Visualizer] Initializing Taichi Window (1024x1024)...")
    window = ti.ui.Window("Garment Simulation - Live", (1024, 1024))
    canvas = window.get_canvas()
    canvas.set_background_color((0.15, 0.15, 0.18))
    scene = window.get_scene()
    camera = ti.ui.Camera()
    
    # Initialize camera position looking at the scene based on center
    camera.position(0, 2.5, 4)
    camera.lookat(0, 1.0, 0)
    
    frame = 0
    # Create the sphere mesh upfront if we need to render it
    sphere_v = None
    sphere_f = None
    if sphere_center is not None and sphere_radius is not None:
        try:
            import trimesh
            sphere = trimesh.creation.icosphere(subdivisions=3, radius=sphere_radius)
            sphere.apply_translation(sphere_center)
            sphere_v = ti.Vector.field(3, dtype=ti.f32, shape=len(sphere.vertices))
            sphere_v.from_numpy(sphere.vertices.astype(np.float32))
            sphere_f = ti.field(dtype=ti.i32, shape=sphere.faces.size)
            sphere_f.from_numpy(sphere.faces.flatten().astype(np.int32))
        except ImportError:
            print("  [Warning] trimesh not installed, sphere proxy won't be drawn.")
            
    if body_mesh_path is not None:
        try:
            import trimesh
            body = trimesh.load(body_mesh_path, force="mesh")
            sphere_v = ti.Vector.field(3, dtype=ti.f32, shape=len(body.vertices))
            sphere_v.from_numpy(body.vertices.astype(np.float32))
            sphere_f = ti.field(dtype=ti.i32, shape=body.faces.size)
            sphere_f.from_numpy(body.faces.flatten().astype(np.int32))
        except Exception as e:
            print(f"  [Warning] Could not load body mesh for visualizer: {e}")

    # Mesh rendering requires flattened indices array for faces
    indices = None
    if state.faces is not None and len(state.faces) > 0:
        indices = ti.field(dtype=ti.i32, shape=state.faces.size)
        indices.from_numpy(state.faces.flatten().astype(np.int32))

    print("  [Live Visualizer] Running. Hold RMB to rotate, W/A/S/D to pan.")
    
    start_time = time.perf_counter()
    while window.running and frame < config.total_frames:
        engine.step_frame(state)
        frame += 1

        camera.track_user_inputs(window, movement_speed=0.03, hold_key=ti.ui.RMB)
        scene.set_camera(camera)
        
        scene.ambient_light((0.4, 0.4, 0.4))
        scene.point_light(pos=(2, 5, 2), color=(1.0, 1.0, 1.0))
        
        # Draw cloth
        if indices is not None:
             scene.mesh(state.positions, indices=indices, color=(0.3, 0.7, 0.8), two_sided=True)
        else:
             scene.particles(state.positions, radius=0.015, color=(0.3, 0.7, 0.8))
             
        # Draw sphere if present
        if sphere_v is not None and sphere_f is not None:
             scene.mesh(sphere_v, indices=sphere_f, color=(0.8, 0.3, 0.3), two_sided=True)
             
        canvas.scene(scene)
        print(f"\r  Frame {frame}/{config.total_frames}", end="", flush=True)
        window.show()
    print()
    elapsed = time.perf_counter() - start_time
    print(f"\n  Simulation Complete. Elapsed: {elapsed:.3f}s ({elapsed / config.total_frames * 1000:.1f}ms/frame)")
