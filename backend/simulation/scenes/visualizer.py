"""
Live visualization module using Taichi's GUI.

Features:
    - Renders cloth mesh + body/sphere collider in real-time
    - Stitch line rendering (green lines between stitch pairs)
    - Pause/Step controls: SPACE to pause, RIGHT ARROW to step one frame
    - Per-frame diagnostics printed to console
    - Camera: hold RMB to rotate, W/A/S/D to pan
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
    stitch_pairs=None,
) -> None:
    """
    Launch Taichi GUI window and run the simulation live.

    Args:
        engine: Configured SimulationEngine (with solver, collider, self_collider).
        state: Initialized ParticleState with positions loaded.
        config: SimConfig for the scene.
        sphere_center: Optional (3,) array for sphere collider rendering.
        sphere_radius: Optional float for sphere collider rendering.
        body_mesh_path: Optional path to body GLB for rendering.
        stitch_pairs: Optional (S, 2) int32 array of stitch vertex index pairs.
                      If provided, green lines are drawn between stitched particles.
    """
    import taichi as ti
    import os

    # Resolve paths before GUI initialization (macOS GUI init can change CWD)
    if body_mesh_path is not None:
        body_mesh_path = os.path.abspath(body_mesh_path)

    print("\n  [Live Visualizer] Initializing Taichi Window (1024x1024)...")
    window = ti.ui.Window("Garment Simulation - Live", (1024, 1024))
    canvas = window.get_canvas()
    canvas.set_background_color((0.12, 0.12, 0.15))
    scene = window.get_scene()
    camera = ti.ui.Camera()

    # Camera: positioned to see the torso region
    camera.position(0.0, 1.2, 1.5)
    camera.lookat(0.0, 1.0, 0.18)

    frame = 0
    paused = False
    step_one = False

    # --- Collider mesh (sphere or body) ---
    collider_v = None
    collider_f = None

    if sphere_center is not None and sphere_radius is not None:
        try:
            import trimesh
            sphere = trimesh.creation.icosphere(subdivisions=3, radius=sphere_radius)
            sphere.apply_translation(sphere_center)
            collider_v = ti.Vector.field(3, dtype=ti.f32, shape=len(sphere.vertices))
            collider_v.from_numpy(sphere.vertices.astype(np.float32))
            collider_f = ti.field(dtype=ti.i32, shape=sphere.faces.size)
            collider_f.from_numpy(sphere.faces.flatten().astype(np.int32))
        except ImportError:
            print("  [Warning] trimesh not installed, sphere proxy won't be drawn.")

    if body_mesh_path is not None:
        try:
            import trimesh
            body = trimesh.load(body_mesh_path, force="mesh")
            collider_v = ti.Vector.field(3, dtype=ti.f32, shape=len(body.vertices))
            collider_v.from_numpy(body.vertices.astype(np.float32))
            collider_f = ti.field(dtype=ti.i32, shape=body.faces.size)
            collider_f.from_numpy(body.faces.flatten().astype(np.int32))
        except Exception as e:
            print(f"  [Warning] Could not load body mesh for visualizer: {e}")

    # --- Cloth mesh indices ---
    indices = None
    if state.faces is not None and len(state.faces) > 0:
        indices = ti.field(dtype=ti.i32, shape=state.faces.size)
        indices.from_numpy(state.faces.flatten().astype(np.int32))

    # --- Stitch line rendering ---
    stitch_line_field = None
    n_stitch_pairs = 0
    if stitch_pairs is not None and len(stitch_pairs) > 0:
        n_stitch_pairs = len(stitch_pairs)
        # Each stitch pair needs 2 vertices for a line segment
        stitch_line_field = ti.Vector.field(3, dtype=ti.f32, shape=n_stitch_pairs * 2)
        print(f"  [Live Visualizer] Rendering {n_stitch_pairs} stitch lines (green)")

    print("  [Live Visualizer] Controls:")
    print("    RMB + drag = rotate camera")
    print("    W/A/S/D    = pan camera")
    print("    SPACE       = pause / resume")
    print("    RIGHT ARROW = step one frame (while paused)")
    print(f"  [Live Visualizer] Running {config.total_frames} frames...")
    print()

    start_time = time.perf_counter()

    while window.running and frame < config.total_frames:
        # --- Input handling ---
        # Check for pause toggle (Space key)
        if window.get_event(ti.ui.PRESS):
            if window.event.key == ti.ui.SPACE:
                paused = not paused
                status = "PAUSED" if paused else "RUNNING"
                print(f"\r  [{status}] Frame {frame}/{config.total_frames}          ")
            elif window.event.key == ti.ui.RIGHT:
                if paused:
                    step_one = True

        # --- Simulation step ---
        if not paused or step_one:
            engine.step_frame(state)
            frame += 1
            step_one = False

            # Print frame diagnostics
            positions_np = state.get_positions_numpy()
            min_y = float(np.min(positions_np[:, 1]))
            max_y = float(np.max(positions_np[:, 1]))
            velocities_np = state.get_velocities_numpy()
            mean_speed = float(np.mean(np.linalg.norm(velocities_np, axis=1)))

            diag = f"  Frame {frame:3d}/{config.total_frames} | Y:[{min_y:.3f}, {max_y:.3f}] | speed:{mean_speed:.3f} m/s"

            # Stitch gap diagnostic
            if stitch_pairs is not None and len(stitch_pairs) > 0:
                pa = positions_np[stitch_pairs[:, 0]]
                pb = positions_np[stitch_pairs[:, 1]]
                gaps = np.linalg.norm(pa - pb, axis=1)
                max_gap = float(np.max(gaps))
                mean_gap = float(np.mean(gaps))
                diag += f" | stitch gap: max={max_gap*100:.1f}cm mean={mean_gap*100:.1f}cm"

            print(f"\r{diag}", end="", flush=True)

        # --- Update stitch line positions ---
        if stitch_line_field is not None and stitch_pairs is not None:
            pos_np = state.get_positions_numpy()
            line_verts = np.zeros((n_stitch_pairs * 2, 3), dtype=np.float32)
            line_verts[0::2] = pos_np[stitch_pairs[:, 0]]
            line_verts[1::2] = pos_np[stitch_pairs[:, 1]]
            stitch_line_field.from_numpy(line_verts)

        # --- Rendering ---
        camera.track_user_inputs(window, movement_speed=0.03, hold_key=ti.ui.RMB)
        scene.set_camera(camera)

        scene.ambient_light((0.4, 0.4, 0.4))
        scene.point_light(pos=(2, 5, 2), color=(1.0, 1.0, 1.0))
        scene.point_light(pos=(-2, 3, -1), color=(0.5, 0.5, 0.6))

        # Draw cloth mesh
        if indices is not None:
            scene.mesh(state.positions, indices=indices,
                       color=(0.3, 0.75, 0.85), two_sided=True)
        else:
            scene.particles(state.positions, radius=0.015,
                            color=(0.3, 0.75, 0.85))

        # Draw collider (body or sphere)
        if collider_v is not None and collider_f is not None:
            scene.mesh(collider_v, indices=collider_f,
                       color=(0.65, 0.55, 0.50), two_sided=True)

        # Draw stitch lines (green)
        if stitch_line_field is not None:
            scene.lines(stitch_line_field, width=3.0,
                        color=(0.2, 1.0, 0.3))

        canvas.scene(scene)
        window.show()

    print()
    elapsed = time.perf_counter() - start_time
    print(f"\n  Simulation Complete. {frame} frames in {elapsed:.3f}s ({elapsed / max(frame, 1) * 1000:.1f}ms/frame)")
