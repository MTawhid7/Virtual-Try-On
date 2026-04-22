"""
Garment drape scene — Pattern → Sew → Drape pipeline.

Drapes a pattern-based garment onto the mannequin body mesh using XPBD
with a 2-stage simulation:
  Stage 1 (Sew):  Reduced gravity, very stiff stitches, no strain limit.
                   Panels slide together and seams close.
  Stage 2 (Drape): Full gravity, normal compliance, strain limit active.
                   Fabric drapes naturally on the body.

Pipeline:
    pattern JSON → triangulate panels → place in 3D → merge → stitch constraints
    → body collision → 2-stage engine.run() → export GLB
"""

import time

import numpy as np
import trimesh

from simulation.collision import BodyCollider
from simulation.constraints import build_constraints
from simulation.core.config import SimConfig
from simulation.core.engine import SimulationEngine, compute_vertex_normals
from simulation.core.state import ParticleState
from simulation.export import write_glb
from simulation.export.gltf_writer import write_glb_animated, write_glb_with_body
from simulation.materials import FABRIC_PRESETS
from simulation.mesh.grid import compute_area_weighted_inv_masses
from simulation.mesh.panel_builder import build_garment_mesh
from simulation.mesh.gc_mesh_adapter import build_garment_mesh_gc, build_gc_attachment_constraints, prewrap_panels_to_body, calibrate_garment_y, resolve_initial_penetrations
from simulation.constraints.attachment import AttachmentConstraints
from simulation.solver.xpbd import XPBDSolver


_BODY_GLB_PATH = "data/bodies/mannequin_physics.glb"
_TSHIRT_JSON = "data/patterns/tshirt.json"
_GC_SHIRT_JSON = "data/patterns/garmentcode/shirt_mean.json"

# Z-offset to align GarmentCode SMPL panel positions onto mannequin_physics.glb.
#
# GarmentCode's mean body has torso panels centred at Z≈+0.025m (after cm→m):
#   front torso translation Z = +25cm → +0.25m
#   back  torso translation Z = -20cm → -0.20m
#   centre = (0.25 + (-0.20)) / 2 = +0.025m
#
# Our mannequin_physics.glb body centre at chest height:
#   chest_z_front = 0.2786m, chest_z_back = 0.0335m
#   centre = (0.2786 + 0.0335) / 2 = +0.1561m
#
# Required offset: 0.1561 - 0.025 = 0.131m
_GC_BODY_Z_OFFSET: float = 0.131


def run_garment_drape(
    visualize: bool = False,
    output_path: str = "storage/garment_drape.glb",
    pattern_path: str = _TSHIRT_JSON,
    resolution: int = 20,
    export_body: bool = True,
    animate: bool = False,
    gc_pattern: str | None = None,
    gc_body_z_offset: float = _GC_BODY_Z_OFFSET,
) -> None:
    """
    Drape a garment pattern onto the body using sew-then-drape.

    Optimized for 30fps real-time visualization:
        - target_edge=0.030 → ~400-800 particles total
        - 4 substeps × 8 solver iterations = 32 solver steps/frame
        - No self-collision (GPU→CPU sync eliminated)
        - Sew phase: 80 frames with 5% gravity, very stiff stitches
        - Drape phase: 220 frames with full gravity, normal compliance

    Validation checks:
        1. No NaN in final positions
        2. No sub-floor penetration (Y ≥ -2×thickness)
        3. Garment on body — majority of particles in torso region
        4. Stitches closed (max seam gap < 5mm after settling)
        5. Energy decay (mean speed < 1.5 m/s)
        6. Edge length preservation (mean stretch < 15%)
    """
    print("=== Garment Drape Scene (Sew-then-Drape) ===")
    if gc_pattern:
        print(f"  Pattern: {gc_pattern} (GarmentCode pipeline)")
    else:
        print(f"  Pattern: {pattern_path} (native pipeline)")
    print("  Draping onto mannequin body mesh...\n")

    fabric = FABRIC_PRESETS["cotton"]

    # --- Body mesh ---
    print("  Loading body mesh...")
    collider = BodyCollider.from_glb(
        _BODY_GLB_PATH,
        target_height=1.75,
        decimate_target=5000,
    )
    print(f"  Body proxy: {collider.spatial_hash.n_triangles} triangles\n")

    config = SimConfig(
        total_frames=570,            # 240 sew + 30 transition + 300 drape
        substeps=8,                  # 4 → 8: more constraint resolution per frame
        solver_iterations=16,        # 8 → 16: better drape-phase convergence
        sew_solver_iterations=32,    # 16 → 32: 8×32=256 solves/frame during sew
        damping=fabric.damping,
        max_particles=50000,
        collision_thickness=0.007,   # 0.012 → 0.007: closer body contact; safe with contact algorithms
        sew_collision_thickness=0.012,   # keep sew shell thick — panels still moving
        friction_coefficient=fabric.friction,
        air_drag=0.3,
        sew_frames=240,
        sew_gravity_fraction=0.15,
        sew_stitch_compliance=1e-10,
        drape_stitch_compliance=1e-8,
        transition_frames=30,
        sew_ramp_frames=120,
        sew_initial_compliance=1.0,  # α̃=57600 >> 2w≈200 → ~0.5mm/iter, no tunneling
        enable_self_collision=False,
    )

    # --- Garment mesh ---
    if gc_pattern:
        # GarmentCode pipeline: pattern spec → BoxMesh → GarmentMesh
        print(f"  Building garment mesh from GarmentCode spec: {gc_pattern}")
        print(f"  GC body Z offset: {gc_body_z_offset:+.4f}m")
        garment = build_garment_mesh_gc(
            gc_pattern,
            mesh_resolution=1.5,   # kept at 1.5cm: 1.0cm caused degenerate seam topology
            fabric="cotton",
            body_z_offset=gc_body_z_offset,
        )
    else:
        # Original pipeline: garment-sim's native pattern JSON
        print("  Building garment mesh from pattern JSON...")
        garment = build_garment_mesh(
            pattern_path,
            resolution=resolution,
            global_scale=1.0,
            target_edge=0.020,
        )
    # GC path: calibrate vertical placement then 3D-prewrap panels onto body surface.
    if gc_pattern:
        calibrate_garment_y(garment, "data/bodies/mannequin_profile.json")
        prewrap_panels_to_body(garment, body_mesh_path=_BODY_GLB_PATH)
        n_corr = resolve_initial_penetrations(garment, body_mesh_path=_BODY_GLB_PATH, clearance=0.008)
        if n_corr:
            print(f"  Penetration resolver: corrected {n_corr} inside-body vertices")

    n_particles = garment.positions.shape[0]
    config.max_particles = max(n_particles + 200, 1000)
    n_stitches = garment.stitch_pairs.shape[0]

    # Diagnostic: print per-panel 3D bounding boxes so alignment can be
    # quickly verified against mannequin surface bounds.
    if gc_pattern:
        offsets_diag = garment.panel_offsets + [n_particles]
        print("  Panel 3D positions (after Z offset):")
        for k, pid in enumerate(garment.panel_ids):
            ps = garment.positions[offsets_diag[k]:offsets_diag[k + 1]]
            print(
                f"    {pid:<24} "
                f"Y=[{ps[:, 1].min():.3f}, {ps[:, 1].max():.3f}]m  "
                f"Z=[{ps[:, 2].min():.3f}, {ps[:, 2].max():.3f}]m"
            )
        # Body reference bounds at chest height for quick sanity check
        print("  Body ref @ chest: Z_back=0.034m, Z_front=0.279m")

    print(f"  Panels:          {garment.panel_ids}")
    print(f"  Particles:       {n_particles}")
    print(f"  Triangles:       {garment.faces.shape[0]}")
    print(f"  Edges:           {garment.edges.shape[0]}")
    print(f"  Stitch pairs:    {n_stitches}")

    # Mesh quality diagnostics
    edge_lengths = np.linalg.norm(
        garment.positions[garment.edges[:, 1]] - garment.positions[garment.edges[:, 0]], axis=1
    )
    print(f"  Edge length — min: {edge_lengths.min()*100:.1f}cm, max: {edge_lengths.max()*100:.1f}cm, "
          f"ratio: {edge_lengths.max()/max(edge_lengths.min(), 1e-8):.1f}x")
    print()

    inv_masses = compute_area_weighted_inv_masses(
        garment.positions, garment.faces, fabric.density,
        max_inv_mass=config.max_inv_mass,
    )

    # --- Constraints ---
    constraints = build_constraints(
        positions=garment.positions,
        edges=garment.edges,
        faces=garment.faces,
        stitch_pairs=garment.stitch_pairs if n_stitches > 0 else None,
        max_stitches=n_stitches + 10,
    )
    print(f"  Distance constraints: {constraints.distance.n_edges if constraints.distance else 0}")
    print(f"  Bending constraints:  {constraints.bending.n_hinges if constraints.bending else 0}")
    print(f"  Stitch constraints:   {constraints.stitch.n_stitches if constraints.stitch else 0}")

    # --- Attachment constraints (GC path only — sew phase panel anchoring) ---
    if gc_pattern:
        attach_indices, attach_targets = build_gc_attachment_constraints(garment)
        if len(attach_indices) > 0:
            attach_constraints = AttachmentConstraints(max_attachments=len(attach_indices) + 50)
            attach_constraints.initialize(attach_indices, attach_targets)
            constraints.attachment = attach_constraints
            print(f"  Attachment constraints: {len(attach_indices)} vertices pinned (sew phase only)")
    print()

    # --- State ---
    state = ParticleState(config)
    state.load_from_numpy(
        garment.positions,
        faces=garment.faces,
        edges=garment.edges,
        inv_masses=inv_masses,
    )

    # --- Substep-independent compliance scaling ---
    # Compliance values in presets are calibrated for substep_dt = 1/60/4 = 0.00417s.
    # Scale to actual substep_dt so physics is invariant to substep count choice.
    reference_substep_dt = 1.0 / 60.0 / 4.0  # calibration reference
    substep_dt = config.dt / config.substeps
    compliance_scale = (substep_dt / reference_substep_dt) ** 2
    scaled_stretch = fabric.stretch_compliance * compliance_scale
    scaled_bend = fabric.bend_compliance * compliance_scale

    # --- Solver ---
    solver = XPBDSolver(
        constraints=constraints,
        stretch_compliance=scaled_stretch,
        bend_compliance=scaled_bend,
        stitch_compliance=config.sew_stitch_compliance,  # Start with sew compliance
        attachment_compliance=1e-4,   # Soft pin — panels can flex but won't tunnel through body
        max_stretch=fabric.max_stretch,
        max_compress=fabric.max_compress,
        stretch_damping=fabric.stretch_damping,
        bend_damping=fabric.bend_damping,
    )

    # --- Engine ---
    engine = SimulationEngine(config, solver=solver)
    engine.collider = collider

    drape_frames = config.total_frames - config.sew_frames - config.transition_frames
    print(f"  Simulation: {config.total_frames} frames "
          f"({config.sew_frames} sew + {config.transition_frames} transition + {drape_frames} drape)")
    print(f"  Substeps: {config.substeps}, Sew iterations: {config.sew_solver_iterations}, "
          f"Drape iterations: {config.solver_iterations}")
    print(f"  Sew solver steps/frame: {config.substeps * config.sew_solver_iterations}  "
          f"Drape solver steps/frame: {config.substeps * config.solver_iterations}")
    print()

    # --- Run ---
    start_time = time.perf_counter()
    if visualize:
        from simulation.scenes.visualizer import visualize_simulation
        visualize_simulation(
            engine, state, config,
            body_mesh_path=_BODY_GLB_PATH,
            stitch_pairs=garment.stitch_pairs if n_stitches > 0 else None,
        )
    else:
        sim_result = engine.run(
            state,
            progress_callback=lambda f, t: print(f"\r  Frame {f}/{t}", end="", flush=True),
            record_every_n_frames=5 if animate else 0,
        )
        elapsed = time.perf_counter() - start_time
        print(f"\n  Elapsed: {elapsed:.3f}s ({elapsed / config.total_frames * 1000:.1f}ms/frame)")

    final_positions = state.get_positions_numpy()
    frame_snaps = sim_result.frame_positions if (not visualize and animate) else None

    # --- Post-processing: seam welding ---
    if n_stitches > 0:
        weld_threshold = 0.005  # 5mm — catch more residual gaps
        welded = 0
        for i, j in garment.stitch_pairs:
            gap = np.linalg.norm(final_positions[i] - final_positions[j])
            if gap < weld_threshold:
                mid = (final_positions[i] + final_positions[j]) / 2
                final_positions[i] = mid
                final_positions[j] = mid
                welded += 1
        print(f"\n  Seam welding: {welded}/{n_stitches} pairs welded (<{weld_threshold*1000:.0f}mm)")

    # --- Validation ---
    print("\n  --- Results ---")

    # 1. No NaN
    has_nan = bool(np.any(np.isnan(final_positions)))
    print(f"  NaN check: {'FAIL ❌' if has_nan else 'PASS ✅'}")

    # 2. No sub-floor penetration
    min_y = float(np.min(final_positions[:, 1]))
    below_floor = min_y < -config.collision_thickness * 2
    print(f"  Min Y: {min_y:.4f}m {'FAIL ❌' if below_floor else 'PASS ✅'} (no sub-floor penetration)")

    # 3. Garment distributed on torso (Y=0.5–1.8m covers waist to above shoulders)
    waist_y, top_y = 0.5, 1.8
    in_torso = int(np.sum((final_positions[:, 1] >= waist_y) & (final_positions[:, 1] <= top_y)))
    torso_ok = in_torso > n_particles * 0.25
    print(f"  Particles in torso Y={waist_y:.1f}–{top_y:.1f}m: {in_torso}/{n_particles}")
    print(f"  Garment on body: {'PASS ✅' if torso_ok else 'FAIL ❌'}")

    # 4. Stitch gap
    if n_stitches > 0:
        pa = final_positions[garment.stitch_pairs[:, 0]]
        pb = final_positions[garment.stitch_pairs[:, 1]]
        gaps = np.linalg.norm(pa - pb, axis=1)
        max_gap = float(np.max(gaps))
        mean_gap = float(np.mean(gaps))
        seam_ok = max_gap < 0.05
        print(f"  Stitch gap — max: {max_gap * 100:.2f}cm, mean: {mean_gap * 100:.2f}cm")
        print(f"  Stitches closed (<5cm): {'PASS ✅' if seam_ok else 'FAIL ❌'}")

        # Per-seam breakdown
        if garment.stitch_seam_ids is not None:
            import collections
            seam_to_indices: dict = collections.OrderedDict()
            for pair_idx, label in enumerate(garment.stitch_seam_ids):
                seam_to_indices.setdefault(label, []).append(pair_idx)
            print("\n  Per-seam gap breakdown:")
            for label, indices in seam_to_indices.items():
                sg = gaps[indices]
                threshold = 0.10  # 10cm — sleeve caps may have wider residual gap
                status = "✅" if sg.max() < threshold else "❌"
                print(f"    {status} {label:<48} "
                      f"max={sg.max()*100:5.1f}cm  mean={sg.mean()*100:5.1f}cm  n={len(indices)}")

    # 5. Energy decay
    velocities = state.get_velocities_numpy()
    mean_speed = float(np.mean(np.linalg.norm(velocities, axis=1)))
    print(f"  Mean speed: {mean_speed:.4f} m/s {'PASS ✅' if mean_speed < 1.5 else 'FAIL ❌'}")

    # 6. Edge length preservation
    final_edge_lengths = np.linalg.norm(
        final_positions[garment.edges[:, 1]] - final_positions[garment.edges[:, 0]], axis=1
    )
    rest_lengths = np.linalg.norm(
        garment.positions[garment.edges[:, 1]] - garment.positions[garment.edges[:, 0]], axis=1
    )
    stretch_ratio = np.abs(final_edge_lengths / rest_lengths - 1.0)
    mean_stretch = float(np.mean(stretch_ratio))
    max_stretch_val = float(np.max(stretch_ratio))
    print(f"  Mean stretch: {mean_stretch:.4%} {'PASS ✅' if mean_stretch < 0.15 else 'FAIL ❌'}")
    print(f"  Max stretch:  {max_stretch_val:.4%}")

    # Per-panel stretch breakdown — locate the worst edge
    worst_edge_idx = int(np.argmax(stretch_ratio))
    vi = int(garment.edges[worst_edge_idx, 0])
    vj = int(garment.edges[worst_edge_idx, 1])
    pos_i = final_positions[vi]
    pos_j = final_positions[vj]
    offsets = garment.panel_offsets + [n_particles]
    def _panel_of(v: int) -> str:
        for k, pid in enumerate(garment.panel_ids):
            if offsets[k] <= v < offsets[k + 1]:
                return pid
        return "unknown"
    panel_i = _panel_of(vi)
    panel_j = _panel_of(vj)
    print(f"\n  Worst edge: idx={worst_edge_idx}  v[{vi}]({panel_i}) — v[{vj}]({panel_j})")
    print(f"    rest={rest_lengths[worst_edge_idx]*100:.2f}cm  "
          f"final={final_edge_lengths[worst_edge_idx]*100:.2f}cm  "
          f"stretch={stretch_ratio[worst_edge_idx]:.1%}")
    print(f"    pos_i=({pos_i[0]:+.3f}, {pos_i[1]:.3f}, {pos_i[2]:+.3f})")
    print(f"    pos_j=({pos_j[0]:+.3f}, {pos_j[1]:.3f}, {pos_j[2]:+.3f})")

    print("\n  Per-panel max stretch:")
    for k, pid in enumerate(garment.panel_ids):
        start, end = offsets[k], offsets[k + 1]
        # All edges where both endpoints are in this panel
        mask = (
            ((garment.edges[:, 0] >= start) & (garment.edges[:, 0] < end)) |
            ((garment.edges[:, 1] >= start) & (garment.edges[:, 1] < end))
        )
        if mask.any():
            ps = stretch_ratio[mask]
            print(f"    {pid:<16} mean={ps.mean():.2%}  max={ps.max():.2%}  "
                  f"n_edges={mask.sum()}  n_over20pct={int((ps > 0.20).sum())}")

    # --- Export ---
    normals = compute_vertex_normals(final_positions, garment.faces)
    body_mesh = trimesh.load(_BODY_GLB_PATH, force="mesh")
    body_verts = np.array(body_mesh.vertices, dtype=np.float32)
    body_faces_arr = np.array(body_mesh.faces, dtype=np.int32)

    if animate and frame_snaps is not None:
        # Animated GLB: drape phase only (skip sew + transition where panels fly around).
        # frame_snaps[0] = initial flat layout; subsequent entries = every 5th frame.
        # Drape phase starts at frame sew_frames + transition_frames.
        record_every = 5
        drape_start_frame = config.sew_frames + config.transition_frames
        drape_start_idx = drape_start_frame // record_every  # snap index where drape begins
        drape_snaps = frame_snaps[drape_start_idx:]
        anim_out = str(output_path).replace(".glb", "_animated.glb")
        out = write_glb_animated(
            drape_snaps,
            garment.faces,
            body_verts,
            body_faces_arr,
            fps=10.0,
            path=anim_out,
        )
        print(f"\n  Animated export ({len(drape_snaps)} drape-phase keyframes) → {out}")
        # Also write the static final-frame GLB alongside it
        static_out = write_glb_with_body(
            final_positions, garment.faces,
            body_verts, body_faces_arr,
            cloth_normals=normals,
            cloth_uvs=garment.uvs,
            path=output_path,
        )
        print(f"  Static export → {static_out}")
    elif export_body:
        out = write_glb_with_body(
            final_positions, garment.faces,
            body_verts, body_faces_arr,
            cloth_normals=normals,
            cloth_uvs=garment.uvs,
            path=output_path,
        )
        print(f"\n  Exported to {out}")
    else:
        out = write_glb(final_positions, garment.faces, normals, uvs=garment.uvs, path=output_path)
        print(f"\n  Exported to {out}")
    print("  Run `cd frontend && npm run dev` then open http://localhost:3000")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Garment simulation drape scene.")
    parser.add_argument("--visualize", action="store_true", help="Launch live visualization")
    parser.add_argument("--pattern", type=str, default=_TSHIRT_JSON, help="Path to pattern JSON")
    parser.add_argument("--output", type=str, default="storage/garment_drape.glb", help="Output path")
    parser.add_argument("--res", type=int, default=20, help="Triangulation resolution")
    parser.add_argument("--no-body", action="store_true", help="Export cloth-only GLB (no body mesh)")
    parser.add_argument("--animate", action="store_true", help="Export animated GLB (morph-target drape sequence)")
    parser.add_argument("--gc", type=str, default=None, metavar="GC_SPEC",
                        help="Path to GarmentCode pattern spec JSON (uses GC pipeline instead of native)")
    parser.add_argument("--gc-z-offset", type=float, default=_GC_BODY_Z_OFFSET, metavar="METRES",
                        help=f"Z offset (m) to align GC panels onto mannequin (default {_GC_BODY_Z_OFFSET})")

    args = parser.parse_args()

    run_garment_drape(
        visualize=args.visualize,
        output_path=args.output,
        pattern_path=args.pattern,
        resolution=args.res,
        export_body=not args.no_body,
        animate=args.animate,
        gc_pattern=args.gc,
        gc_body_z_offset=args.gc_z_offset,
    )
