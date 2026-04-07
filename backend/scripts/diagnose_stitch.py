"""
Diagnostic script: examine stitch closure and edge stretch distribution
after garment drape simulation. Exports per-frame snapshots.
"""
import numpy as np
from simulation.mesh.panel_builder import build_garment_mesh
from simulation.mesh.grid import compute_area_weighted_inv_masses
from simulation.materials import FABRIC_PRESETS
from simulation.constraints import build_constraints
from simulation.core.config import SimConfig
from simulation.core.state import ParticleState
from simulation.core.engine import SimulationEngine, compute_vertex_normals
from simulation.collision import BodyCollider
from simulation.solver.xpbd import XPBDSolver
from simulation.export import write_glb
import time

_BODY = "data/bodies/mannequin_physics.glb"
_PATTERN = "data/patterns/tank_top.json"

fabric = FABRIC_PRESETS["cotton"]
garment = build_garment_mesh(_PATTERN, resolution=20)
n = garment.positions.shape[0]
n_st = garment.stitch_pairs.shape[0]

print(f"Particles: {n}, Stitch pairs: {n_st}")
print(f"Front panel range: X=[{garment.positions[:n//2,0].min():.3f}, {garment.positions[:n//2,0].max():.3f}], "
      f"Z=[{garment.positions[:n//2,2].min():.3f}, {garment.positions[:n//2,2].max():.3f}]")
print(f"Back panel range:  X=[{garment.positions[n//2:,0].min():.3f}, {garment.positions[n//2:,0].max():.3f}], "
      f"Z=[{garment.positions[n//2:,2].min():.3f}, {garment.positions[n//2:,2].max():.3f}]")

# Show stitch pair initial distances
pa_init = garment.positions[garment.stitch_pairs[:, 0]]
pb_init = garment.positions[garment.stitch_pairs[:, 1]]
init_gaps = np.linalg.norm(pa_init - pb_init, axis=1)
print(f"\nInitial stitch gaps: min={init_gaps.min()*100:.1f}cm, "
      f"max={init_gaps.max()*100:.1f}cm, mean={init_gaps.mean()*100:.1f}cm")

# Show stitch pair positions
print("\nStitch pair endpoints (first 5):")
for i in range(min(5, n_st)):
    a, b = garment.stitch_pairs[i]
    pA = garment.positions[a]
    pB = garment.positions[b]
    d = np.linalg.norm(pA - pB)
    print(f"  [{a:3d}] ({pA[0]:+.3f}, {pA[1]:.3f}, {pA[2]:.3f}) <-> "
          f"[{b:3d}] ({pB[0]:+.3f}, {pB[1]:.3f}, {pB[2]:.3f}) gap={d*100:.1f}cm")

# Run short sim and check convergence at intervals
config = SimConfig(
    total_frames=480,
    substeps=15,
    solver_iterations=12,
    damping=fabric.damping,
    max_particles=1000,
    collision_thickness=0.008,
    friction_coefficient=fabric.friction,
    air_drag=0.3,
)

inv_masses = compute_area_weighted_inv_masses(garment.positions, garment.faces, fabric.density)
constraints = build_constraints(
    positions=garment.positions, edges=garment.edges, faces=garment.faces,
    stitch_pairs=garment.stitch_pairs, max_stitches=n_st + 10,
)

state = ParticleState(config)
state.load_from_numpy(garment.positions, faces=garment.faces, edges=garment.edges, inv_masses=inv_masses)

solver = XPBDSolver(
    constraints=constraints,
    stretch_compliance=fabric.stretch_compliance,
    bend_compliance=fabric.bend_compliance,
    stitch_compliance=1e-8,
    max_stretch=fabric.max_stretch,
    max_compress=fabric.max_compress,
    stretch_damping=fabric.stretch_damping,
    bend_damping=fabric.bend_damping,
)

engine = SimulationEngine(config, solver=solver)
engine.collider = BodyCollider.from_glb(_BODY, decimate_target=5000)

solver.initialize(state, config)

print(f"\nRunning simulation ({config.total_frames} frames)...")
print(f"{'Frame':>6s}  {'MeanGap':>8s}  {'MaxGap':>8s}  {'MeanSpd':>8s}  {'MaxStretch':>10s}  {'MinY':>6s}")

import pathlib
pathlib.Path("storage/frames").mkdir(parents=True, exist_ok=True)

start = time.perf_counter()
for frame in range(config.total_frames):
    engine.step_frame(state)

    if (frame + 1) % 60 == 0 or frame == 0:
        pos = state.get_positions_numpy()
        vel = state.get_velocities_numpy()

        # Stitch gaps
        pa = pos[garment.stitch_pairs[:, 0]]
        pb = pos[garment.stitch_pairs[:, 1]]
        gaps = np.linalg.norm(pa - pb, axis=1)

        # Edge stretch
        el = np.linalg.norm(pos[garment.edges[:, 1]] - pos[garment.edges[:, 0]], axis=1)
        rl = np.linalg.norm(garment.positions[garment.edges[:, 1]] - garment.positions[garment.edges[:, 0]], axis=1)
        stretch = np.abs(el / rl - 1.0)

        mean_spd = np.mean(np.linalg.norm(vel, axis=1))
        min_y = np.min(pos[:, 1])

        print(f"{frame+1:6d}  {gaps.mean()*100:7.2f}cm  {gaps.max()*100:7.2f}cm  "
              f"{mean_spd:7.3f}  {stretch.max()*100:9.2f}%  {min_y:6.3f}")

        # Export snapshot
        normals = compute_vertex_normals(pos, garment.faces)
        write_glb(pos, garment.faces, normals, uvs=garment.uvs,
                  path=f"storage/frames/garment_f{frame+1:04d}.glb")

elapsed = time.perf_counter() - start
print(f"\nDone in {elapsed:.1f}s")

# Final detailed analysis
pos = state.get_positions_numpy()
pa = pos[garment.stitch_pairs[:, 0]]
pb = pos[garment.stitch_pairs[:, 1]]
gaps = np.linalg.norm(pa - pb, axis=1)
print(f"\nFinal stitch gaps distribution:")
for pct in [0, 25, 50, 75, 90, 95, 100]:
    v = np.percentile(gaps, pct) * 100
    print(f"  {pct:3d}th percentile: {v:.2f} cm")

# Find worst stretch edges
el = np.linalg.norm(pos[garment.edges[:, 1]] - pos[garment.edges[:, 0]], axis=1)
rl = np.linalg.norm(garment.positions[garment.edges[:, 1]] - garment.positions[garment.edges[:, 0]], axis=1)
stretch = np.abs(el / rl - 1.0)
worst5 = np.argsort(stretch)[-5:]
print(f"\nTop 5 worst-stretched edges:")
for idx in reversed(worst5):
    a, b = garment.edges[idx]
    print(f"  edge [{a:3d}-{b:3d}]: stretch={stretch[idx]*100:.1f}%, "
          f"current={el[idx]*100:.2f}cm, rest={rl[idx]*100:.2f}cm")
