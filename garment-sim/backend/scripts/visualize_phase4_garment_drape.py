"""
Phase 4 visual verification — Garment Drape on Body.

Exports three GLB files to backend/storage/:

    phase4_garment_initial.glb   — garment panels in starting position (no sim)
    phase4_garment_draped.glb    — garment after 300 frames draping on body
    phase4_body.glb              — body mesh alone (for overlay in Blender)

Run from backend/:
    python -m scripts.visualize_phase4_garment_drape

What to check:
    1. phase4_garment_initial.glb
       - Front panel (lighter) should be at Z≈+0.12, back panel at Z≈-0.12
       - Both panels in the torso region (Y=0.65–1.35m)
       - Two flat rectangles separated by ~24cm in Z

    2. phase4_garment_draped.glb
       - Panels should wrap around the body (no longer flat)
       - Both panels visible — front curves forward, back curves backward
       - Side seams (left/right edges) should be closed (near zero gap)
       - No spikes or explosive instability

    3. Overlay in Blender:
       - Import both phase4_body.glb and phase4_garment_draped.glb
       - Garment should sit on the torso surface, not penetrate or float above
"""

import sys
import time
from pathlib import Path

import numpy as np
import trimesh

# Ensure backend/ is on path when run as `python -m scripts.visualize_phase4_garment_drape`
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import simulation  # noqa: F401 — must import first to init Taichi

from simulation.collision import BodyCollider, ClothSelfCollider
from simulation.constraints import build_constraints
from simulation.core.config import SimConfig
from simulation.core.engine import SimulationEngine, compute_vertex_normals
from simulation.core.state import ParticleState
from simulation.export import write_glb
from simulation.materials import FABRIC_PRESETS
from simulation.mesh.grid import compute_area_weighted_inv_masses
from simulation.mesh.panel_builder import build_garment_mesh
from simulation.solver.xpbd import XPBDSolver


_BODY_GLB = Path("data/bodies/mannequin_physics.glb")
_TANK_TOP_JSON = Path("data/patterns/tank_top.json")
_OUT_DIR = Path("storage")

TOTAL_FRAMES = 300
RESOLUTION = 20


def export_body_mesh(out_path: Path) -> None:
    """Load mannequin_physics.glb and re-export as a standalone GLB."""
    scene = trimesh.load(str(_BODY_GLB))
    meshes = list(scene.geometry.values()) if hasattr(scene, "geometry") else [scene]
    combined = trimesh.util.concatenate(meshes)
    body_scene = trimesh.Scene(geometry={"body": combined})
    out_path.parent.mkdir(parents=True, exist_ok=True)
    body_scene.export(file_obj=str(out_path), file_type="glb")
    print(f"  Body mesh → {out_path}")


def main() -> None:
    _OUT_DIR.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("Phase 4 Visual Verification — Garment Drape on Body")
    print("=" * 60)

    # ------------------------------------------------------------------ #
    # 1. Export initial garment position (no simulation)
    # ------------------------------------------------------------------ #
    print("\n[1/4] Building garment mesh...")
    garment = build_garment_mesh(_TANK_TOP_JSON, resolution=RESOLUTION)
    n_particles = garment.positions.shape[0]
    n_stitches = garment.stitch_pairs.shape[0]
    print(f"      Panels:       {garment.panel_ids}")
    print(f"      Particles:    {n_particles}")
    print(f"      Triangles:    {garment.faces.shape[0]}")
    print(f"      Stitch pairs: {n_stitches}")

    normals_init = compute_vertex_normals(garment.positions, garment.faces)
    out_initial = _OUT_DIR / "phase4_garment_initial.glb"
    write_glb(garment.positions, garment.faces, normals_init, uvs=garment.uvs, path=out_initial)
    print(f"\n      Initial garment → {out_initial}")

    # Print initial bounding box to verify placement
    bb_min = garment.positions.min(axis=0)
    bb_max = garment.positions.max(axis=0)
    print(f"      Initial bbox: X=[{bb_min[0]:.3f}, {bb_max[0]:.3f}]  "
          f"Y=[{bb_min[1]:.3f}, {bb_max[1]:.3f}]  Z=[{bb_min[2]:.3f}, {bb_max[2]:.3f}]")

    # ------------------------------------------------------------------ #
    # 2. Export body mesh
    # ------------------------------------------------------------------ #
    print("\n[2/4] Exporting body mesh...")
    out_body = _OUT_DIR / "phase4_body.glb"
    export_body_mesh(out_body)

    # ------------------------------------------------------------------ #
    # 3. Run simulation
    # ------------------------------------------------------------------ #
    print(f"\n[3/4] Running simulation ({TOTAL_FRAMES} frames)...")
    fabric = FABRIC_PRESETS["cotton"]

    config = SimConfig(
        total_frames=TOTAL_FRAMES,
        substeps=15,
        solver_iterations=8,
        damping=fabric.damping,
        max_particles=n_particles + 50,
        collision_thickness=0.008,
        friction_coefficient=fabric.friction,
        air_drag=0.3,
    )

    collider = BodyCollider.from_glb(str(_BODY_GLB), target_height=1.75, decimate_target=5000)
    print(f"      Body triangles: {collider.spatial_hash.n_triangles}")

    inv_masses = compute_area_weighted_inv_masses(
        garment.positions, garment.faces, fabric.density
    )

    constraints = build_constraints(
        positions=garment.positions,
        edges=garment.edges,
        faces=garment.faces,
        stitch_pairs=garment.stitch_pairs if n_stitches > 0 else None,
        max_stitches=n_stitches + 10,
    )

    state = ParticleState(config)
    state.load_from_numpy(
        garment.positions,
        faces=garment.faces,
        edges=garment.edges,
        inv_masses=inv_masses,
    )

    solver = XPBDSolver(
        constraints=constraints,
        stretch_compliance=fabric.stretch_compliance,
        bend_compliance=fabric.bend_compliance,
        stitch_compliance=1e-6,
        max_stretch=fabric.max_stretch,
        max_compress=fabric.max_compress,
        stretch_damping=fabric.stretch_damping,
        bend_damping=fabric.bend_damping,
    )

    engine = SimulationEngine(config, solver=solver)
    engine.collider = collider
    engine.self_collider = ClothSelfCollider.from_mesh(
        faces_np=garment.faces,
        positions_np=garment.positions,
        thickness=config.self_collision_thickness,
    )

    t0 = time.perf_counter()
    engine.run(
        state,
        progress_callback=lambda f, t: print(f"\r      Frame {f}/{t}", end="", flush=True),
    )
    elapsed = time.perf_counter() - t0
    print(f"\n      Done in {elapsed:.1f}s ({elapsed / TOTAL_FRAMES * 1000:.0f}ms/frame)")

    # ------------------------------------------------------------------ #
    # 4. Export draped result + stats
    # ------------------------------------------------------------------ #
    print("\n[4/4] Exporting draped garment...")
    final_pos = state.get_positions_numpy()
    normals_final = compute_vertex_normals(final_pos, garment.faces)
    out_draped = _OUT_DIR / "phase4_garment_draped.glb"
    write_glb(final_pos, garment.faces, normals_final, uvs=garment.uvs, path=out_draped)
    print(f"      Draped garment → {out_draped}")

    # Stats
    bb_min = final_pos.min(axis=0)
    bb_max = final_pos.max(axis=0)
    print(f"      Final bbox: X=[{bb_min[0]:.3f}, {bb_max[0]:.3f}]  "
          f"Y=[{bb_min[1]:.3f}, {bb_max[1]:.3f}]  Z=[{bb_min[2]:.3f}, {bb_max[2]:.3f}]")

    velocities = state.get_velocities_numpy()
    mean_speed = float(np.mean(np.linalg.norm(velocities, axis=1)))
    print(f"      Mean speed: {mean_speed:.4f} m/s")

    if n_stitches > 0:
        pa = final_pos[garment.stitch_pairs[:, 0]]
        pb = final_pos[garment.stitch_pairs[:, 1]]
        gaps = np.linalg.norm(pa - pb, axis=1)
        print(f"      Seam gap — max: {gaps.max() * 100:.2f}cm, mean: {gaps.mean() * 100:.2f}cm")

    edge_lengths = np.linalg.norm(
        final_pos[garment.edges[:, 1]] - final_pos[garment.edges[:, 0]], axis=1
    )
    rest_lengths = np.linalg.norm(
        garment.positions[garment.edges[:, 1]] - garment.positions[garment.edges[:, 0]], axis=1
    )
    mean_stretch = float(np.mean(np.abs(edge_lengths / rest_lengths - 1.0)))
    print(f"      Mean stretch: {mean_stretch:.4%}")

    print("\n" + "=" * 60)
    print("Output files:")
    print(f"  {out_initial}")
    print(f"  {out_body}")
    print(f"  {out_draped}")
    print("\nVerification checklist:")
    print("  [ ] phase4_garment_initial.glb — two flat panels at Y=0.65–1.35m, Z=±0.12m")
    print("  [ ] phase4_garment_draped.glb  — panels curved around body torso, seams closed")
    print("  [ ] Blender overlay: garment sits ON body surface (no float, no penetration)")
    print("  [ ] No explosive spikes or instability in draped result")


if __name__ == "__main__":
    main()
