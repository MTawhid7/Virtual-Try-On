"""
Trace the sew phase of a GarmentCode simulation.

Runs only the sew phase (default 240 frames) and prints a gap table at
regular checkpoints.  Exports one GLB snapshot per checkpoint so you can
step through the closure sequence in the frontend viewer.

This script answers:
  - Do sleeve cap gaps reduce at all, or does the solver stall immediately?
  - Which seams close fastest vs slowest?
  - At what frame does the back panel penetrate the body?
  - What does the garment look like mid-sew?

Usage (run from backend/ with virtualenv active):
    python -m scripts.debug_gc_sew_trace
    python -m scripts.debug_gc_sew_trace --frames 120 --checkpoints 6
    python -m scripts.debug_gc_sew_trace --pattern data/patterns/garmentcode/hoody_mean.json

Output: storage/debug_sew_f????.glb  (one file per checkpoint)
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path


def run_trace(
    pattern_path: str = "data/patterns/garmentcode/shirt_mean.json",
    mesh_resolution: float = 1.5,
    body_z_offset: float = 0.131,
    sew_frames: int = 240,
    n_checkpoints: int = 6,
    output_dir: str = "storage",
) -> None:
    import collections

    import numpy as np
    import trimesh

    from simulation.collision import BodyCollider
    from simulation.constraints import build_constraints
    from simulation.constraints.attachment import AttachmentConstraints
    from simulation.core.config import SimConfig
    from simulation.core.engine import SimulationEngine, compute_vertex_normals
    from simulation.core.state import ParticleState
    from simulation.export.gltf_writer import write_glb_with_body
    from simulation.materials import FABRIC_PRESETS
    from simulation.mesh.gc_mesh_adapter import build_garment_mesh_gc, build_gc_attachment_constraints, prewrap_panels_to_body, calibrate_garment_y, resolve_initial_penetrations
    from simulation.mesh.grid import compute_area_weighted_inv_masses
    from simulation.solver.xpbd import XPBDSolver

    print(f"\n{'='*72}")
    print("GC Sew Phase Trace")
    print(f"{'='*72}")
    print(f"  Pattern:      {pattern_path}")
    print(f"  Sew frames:   {sew_frames}")
    print(f"  Checkpoints:  {n_checkpoints}")

    # --- Body collider ---
    fabric = FABRIC_PRESETS["cotton"]
    collider = BodyCollider.from_glb(
        "data/bodies/mannequin_physics.glb",
        target_height=1.75,
        decimate_target=5000,
    )

    # --- Config — mirrors garment_drape.py exactly, but total_frames=sew_frames ---
    config = SimConfig(
        total_frames=sew_frames,
        substeps=8,
        solver_iterations=16,
        sew_solver_iterations=32,
        damping=fabric.damping,
        max_particles=50_000,
        collision_thickness=0.007,
        sew_collision_thickness=0.012,
        friction_coefficient=fabric.friction,
        air_drag=0.3,
        sew_frames=sew_frames,          # run only the sew phase
        sew_gravity_fraction=0.15,
        sew_stitch_compliance=1e-10,
        drape_stitch_compliance=1e-8,
        transition_frames=0,            # no transition — we stop at end of sew
        sew_ramp_frames=120,
        sew_initial_compliance=1.0,
        enable_self_collision=False,
    )

    # --- Garment mesh ---
    gm = build_garment_mesh_gc(
        pattern_path,
        mesh_resolution=mesh_resolution,
        body_z_offset=body_z_offset,
    )
    # Calibrate vertical placement then 3D-prewrap panels onto body surface.
    calibrate_garment_y(gm, "data/bodies/mannequin_profile.json")
    prewrap_panels_to_body(gm, clearance=0.008)
    n_corr = resolve_initial_penetrations(gm, clearance=0.008)
    if n_corr:
        print(f"  Penetration resolver: corrected {n_corr} inside-body vertices")
    n_particles = gm.positions.shape[0]
    config.max_particles = max(n_particles + 200, 1000)
    n_stitches = gm.stitch_pairs.shape[0]
    offsets = gm.panel_offsets + [n_particles]

    # --- Physics setup ---
    inv_masses = compute_area_weighted_inv_masses(
        gm.positions, gm.faces, fabric.density,
        max_inv_mass=config.max_inv_mass,
    )
    constraints = build_constraints(
        positions=gm.positions,
        edges=gm.edges,
        faces=gm.faces,
        stitch_pairs=gm.stitch_pairs if n_stitches > 0 else None,
        max_stitches=n_stitches + 10,
    )
    # --- Attachment constraints (sew phase anchors) ---
    attach_indices, attach_targets = build_gc_attachment_constraints(gm)
    if len(attach_indices) > 0:
        attach_constraints = AttachmentConstraints(max_attachments=len(attach_indices) + 50)
        attach_constraints.initialize(attach_indices, attach_targets)
        constraints.attachment = attach_constraints
        print(f"  Attachment constraints: {len(attach_indices)} vertices pinned (sew phase only)")
    else:
        print("  Attachment constraints: none (no vertices selected)")

    state = ParticleState(config)
    state.load_from_numpy(
        gm.positions, faces=gm.faces, edges=gm.edges, inv_masses=inv_masses
    )

    ref_dt = 1.0 / 60.0 / 4.0
    substep_dt = config.dt / config.substeps
    scale = (substep_dt / ref_dt) ** 2

    solver = XPBDSolver(
        constraints=constraints,
        stretch_compliance=fabric.stretch_compliance * scale,
        bend_compliance=fabric.bend_compliance * scale,
        stitch_compliance=config.sew_stitch_compliance,
        attachment_compliance=1e-4,
        max_stretch=fabric.max_stretch,
        max_compress=fabric.max_compress,
        stretch_damping=fabric.stretch_damping,
        bend_damping=fabric.bend_damping,
    )
    engine = SimulationEngine(config, solver=solver)
    engine.collider = collider
    solver.initialize(state, config)

    # --- Seam label grouping ---
    seam_to_indices: dict = collections.OrderedDict()
    if gm.stitch_seam_ids:
        for pair_idx, label in enumerate(gm.stitch_seam_ids):
            seam_to_indices.setdefault(label, []).append(pair_idx)
    else:
        seam_to_indices["all"] = list(range(n_stitches))

    # --- Checkpoint frames ---
    checkpoint_set = set([0])
    for i in range(1, n_checkpoints + 1):
        checkpoint_set.add(int(round(sew_frames * i / n_checkpoints)))
    # --- Body mesh for GLB export ---
    body_mesh = trimesh.load("data/bodies/mannequin_physics.glb", force="mesh")
    body_verts = np.array(body_mesh.vertices, dtype=np.float32)
    body_faces_arr = np.array(body_mesh.faces, dtype=np.int32)

    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # --- Helper: print gap table for current state ---
    def print_gap_table(frame_num: int, pos_np: "np.ndarray") -> None:
        pa_all = pos_np[gm.stitch_pairs[:, 0]]
        pb_all = pos_np[gm.stitch_pairs[:, 1]]
        all_gaps = np.linalg.norm(pa_all - pb_all, axis=1)

        # Count vertices inside the body interior Z range
        body_z_back, body_z_front = 0.034, 0.279
        in_body = int(np.sum(
            (pos_np[:, 2] > body_z_back) & (pos_np[:, 2] < body_z_front) &
            (pos_np[:, 1] > 0.5) & (pos_np[:, 1] < 1.8)
        ))

        # Also check specifically the back-torso panel for penetration
        back_panels = [k for k, pid in enumerate(gm.panel_ids) if "btorso" in pid.lower() or "back" in pid.lower()]
        back_inside = 0
        for k in back_panels:
            ps = pos_np[offsets[k]:offsets[k + 1]]
            back_inside += int(np.sum((ps[:, 2] > body_z_back) & (ps[:, 2] < body_z_front)))

        print(
            f"\n  ── Frame {frame_num:4d} / {sew_frames} ─────────────────────────────────────────"
        )
        print(
            f"     Overall gap: mean={all_gaps.mean()*100:.1f}cm  max={all_gaps.max()*100:.1f}cm  "
            f"body_interior_verts={in_body}  back_panel_inside={back_inside}"
        )
        for label, indices in seam_to_indices.items():
            sg = all_gaps[np.array(indices)]
            flag = "❌" if sg.max() > 0.05 else "✅"
            print(
                f"     {flag} {label:<50}  "
                f"max={sg.max()*100:5.1f}cm  mean={sg.mean()*100:5.1f}cm  n={len(indices)}"
            )

    # --- Helper: export snapshot GLB ---
    def export_snapshot(frame_num: int, pos_np: "np.ndarray") -> Path:
        normals = compute_vertex_normals(pos_np, gm.faces)
        out = out_dir / f"debug_sew_f{frame_num:04d}.glb"
        write_glb_with_body(
            pos_np, gm.faces,
            body_verts, body_faces_arr,
            cloth_normals=normals,
            cloth_uvs=gm.uvs,
            path=str(out),
        )
        return out

    # -----------------------------------------------------------------------
    # Initial state (frame 0)
    # -----------------------------------------------------------------------
    print(f"\n  Particles: {n_particles}  |  Seams: {len(seam_to_indices)}  |  Pairs: {n_stitches}")

    init_pos = state.get_positions_numpy()
    print_gap_table(0, init_pos)
    out = export_snapshot(0, init_pos)
    print(f"  💾 → {out}")

    # -----------------------------------------------------------------------
    # Sew phase loop
    # -----------------------------------------------------------------------
    start_t = time.perf_counter()
    snapshots: list[tuple[int, "np.ndarray"]] = []

    for frame in range(sew_frames):
        engine.step_frame(state, frame=frame)

        if (frame + 1) in checkpoint_set:
            pos_np = state.get_positions_numpy()
            print_gap_table(frame + 1, pos_np)
            out = export_snapshot(frame + 1, pos_np)
            print(f"  💾 → {out}")
            snapshots.append((frame + 1, pos_np.copy()))

    elapsed = time.perf_counter() - start_t
    print(f"\n  Sew phase complete in {elapsed:.1f}s ({elapsed/sew_frames*1000:.1f}ms/frame)")

    # -----------------------------------------------------------------------
    # Seam closure summary table
    # -----------------------------------------------------------------------
    final_pos = state.get_positions_numpy()
    init_gaps = np.linalg.norm(
        init_pos[gm.stitch_pairs[:, 0]] - init_pos[gm.stitch_pairs[:, 1]], axis=1
    )
    final_gaps = np.linalg.norm(
        final_pos[gm.stitch_pairs[:, 0]] - final_pos[gm.stitch_pairs[:, 1]], axis=1
    )

    print(f"\n  {'─'*70}")
    print("  Seam closure summary  (initial → final)")
    print(f"  {'─'*70}")
    print(f"  {'Seam':<52}  {'Initial':>8}  {'Final':>8}  {'Closed%':>8}  {'Status'}")
    print(f"  {'─'*52}  {'─'*8}  {'─'*8}  {'─'*8}  {'─'*6}")

    for label, indices in seam_to_indices.items():
        idx_arr = np.array(indices)
        i_gap = float(init_gaps[idx_arr].mean())
        f_gap = float(final_gaps[idx_arr].mean())
        closed_pct = 100.0 * (1.0 - f_gap / max(i_gap, 1e-6))
        status = "✅" if f_gap < 0.05 else "❌"
        print(
            f"  {label:<52}  {i_gap*100:>7.1f}cm  {f_gap*100:>7.1f}cm  "
            f"{closed_pct:>7.1f}%  {status}"
        )

    print(f"\n  GLB snapshots written to {out_dir}/:")
    print(f"    debug_sew_f0000.glb  (initial)")
    for f, _ in snapshots:
        print(f"    debug_sew_f{f:04d}.glb")
    print(f"\n  Load each in the viewer (http://localhost:3000) to step through the sew sequence.")
    print()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pattern", default="data/patterns/garmentcode/shirt_mean.json",
        help="GarmentCode spec JSON",
    )
    parser.add_argument(
        "--z-offset", type=float, default=0.131, dest="z_offset",
        help="Body Z offset in metres (default: 0.131)",
    )
    parser.add_argument(
        "--res", type=float, default=1.5,
        help="Mesh resolution in cm (default: 1.5)",
    )
    parser.add_argument(
        "--frames", type=int, default=240,
        help="Number of sew-phase frames to run (default: 240)",
    )
    parser.add_argument(
        "--checkpoints", type=int, default=6,
        help="Number of gap-table snapshots to print and export (default: 6)",
    )
    parser.add_argument(
        "--output-dir", default="storage", dest="output_dir",
        help="Directory for snapshot GLBs (default: storage)",
    )
    args = parser.parse_args()

    run_trace(
        pattern_path=args.pattern,
        mesh_resolution=args.res,
        body_z_offset=args.z_offset,
        sew_frames=args.frames,
        n_checkpoints=args.checkpoints,
        output_dir=args.output_dir,
    )


if __name__ == "__main__":
    main()
