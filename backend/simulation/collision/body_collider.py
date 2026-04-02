"""
Body mesh collision orchestrator — Sprint 2 Layer 3a-Extended.

Loads a GLB body mesh, rescales to target height, optionally decimates to
a physics proxy, computes smooth vertex normals, and builds a static spatial
hash for O(1) candidate triangle lookup.

Interface matches SphereCollider so the engine doesn't need to know which
collider is active:
    collider.resolve(state, config)

This file may use `from __future__ import annotations` (no @ti.kernel here —
kernels live in resolver.py).
"""

from __future__ import annotations

import logging
import warnings
from pathlib import Path

import numpy as np
import trimesh

from simulation.collision.resolver import resolve_body_collision
from simulation.collision.spatial_hash import StaticSpatialHash
from simulation.core.config import SimConfig
from simulation.core.state import ParticleState

log = logging.getLogger(__name__)


class BodyCollider:
    """
    Body mesh collision resolver.

    Loads a GLB body mesh, preprocesses it into a physics proxy, builds a
    static spatial hash, and exposes a resolve() interface matching SphereCollider.

    Usage:
        collider = BodyCollider.from_glb("data/bodies/male_body.glb")
        engine.collider = collider
        # Engine calls collider.resolve(state, config) inside the solver loop
    """

    def __init__(self, spatial_hash: StaticSpatialHash) -> None:
        self.spatial_hash = spatial_hash

    @classmethod
    def from_glb(
        cls,
        path: str | Path,
        target_height: float = 1.75,
        decimate_target: int | None = 5000,
        cell_size: float | None = None,
        max_entries: int = 500000,
    ) -> BodyCollider:
        """
        Factory: load GLB → rescale → decimate → normals → build spatial hash.

        Preprocessing pipeline:
            1. Load mesh via trimesh (force='mesh')
            2. Rescale: scale = target_height / (max_y - min_y), shift feet to Y=0
            3. Decimate: simplify_quadric_decimation (warn+skip if unavailable)
            4. Post-process: remove degenerate triangles, recompute normals
            5. Auto cell_size: 1.5× mean edge length (if not provided)
            6. Build StaticSpatialHash

        The body mesh (male_body.glb) is in centimeters. Rescaling converts to meters.

        Args:
            path: Path to the body GLB file.
            target_height: Target body height in meters after rescaling.
            decimate_target: Target face count for physics proxy. None = skip decimation.
            cell_size: Hash grid cell size (meters). Auto-computed if None.
            max_entries: Max spatial hash entries (raise if warning about overflow).

        Returns:
            Ready-to-use BodyCollider with spatial hash built.
        """
        path = Path(path)
        log.info(f"BodyCollider: loading {path}")

        # 1. Load mesh
        mesh = trimesh.load(str(path), force="mesh")
        log.info(f"  Loaded: {len(mesh.vertices)} vertices, {len(mesh.faces)} faces")

        # 2. Rescale to target height, feet at Y=0
        bounds = mesh.bounds  # [[min_x, min_y, min_z], [max_x, max_y, max_z]]
        min_y = bounds[0][1]
        max_y = bounds[1][1]
        height = max_y - min_y
        if height < 1e-6:
            raise ValueError(f"Body mesh height is near-zero ({height}). Check the mesh.")

        scale = target_height / height
        mesh.apply_scale(scale)
        # Shift so feet (min_y after scale) are at Y=0
        min_y_scaled = mesh.bounds[0][1]
        mesh.apply_translation([0.0, -min_y_scaled, 0.0])
        log.info(
            f"  Rescaled: scale={scale:.5f}, height={target_height}m, "
            f"Y range: {mesh.bounds[0][1]:.3f}–{mesh.bounds[1][1]:.3f}m"
        )

        # 3. Decimate to physics proxy
        if decimate_target is not None and len(mesh.faces) > decimate_target:
            try:
                mesh = mesh.simplify_quadric_decimation(face_count=decimate_target)
                log.info(f"  Decimated: {len(mesh.faces)} faces")
            except Exception as e:
                warnings.warn(
                    f"Decimation failed ({e}). Using full mesh as physics proxy "
                    f"({len(mesh.faces)} faces). This may be slow."
                )

        # 4. Post-process: remove degenerate triangles + fix normals
        areas = mesh.area_faces
        valid_mask = areas > 1e-10
        n_degenerate = int((~valid_mask).sum())
        if n_degenerate > 0:
            warnings.warn(
                f"Removed {n_degenerate} degenerate triangles from decimated mesh."
            )
            mesh.update_faces(valid_mask)
            mesh.remove_unreferenced_vertices()

        mesh.fix_normals()  # Recompute/fix after decimation

        # Log quality metrics
        edge_lengths = np.linalg.norm(
            mesh.vertices[mesh.edges_unique[:, 1]] - mesh.vertices[mesh.edges_unique[:, 0]],
            axis=1,
        )
        log.info(
            f"  Physics proxy: {len(mesh.faces)} faces, "
            f"edge length: min={edge_lengths.min():.4f} "
            f"avg={edge_lengths.mean():.4f} "
            f"max={edge_lengths.max():.4f} m"
        )

        # 5. Auto cell_size: 1.5× mean edge length
        if cell_size is None:
            cell_size = float(1.5 * edge_lengths.mean())
            log.info(f"  Auto cell_size: {cell_size:.4f} m")

        # Vertex normals for smooth normal interpolation (Vestra "smoothed normal trick")
        vertex_normals = np.array(mesh.vertex_normals, dtype=np.float32)

        # 6. Build spatial hash
        n_tris = len(mesh.faces)
        sh = StaticSpatialHash(
            cell_size=cell_size,
            table_size=65536,
            max_triangles=max(n_tris + 1000, 25000),
            max_entries=max_entries,
        )
        sh.build(
            vertices=np.array(mesh.vertices, dtype=np.float32),
            faces=np.array(mesh.faces, dtype=np.int32),
            vertex_normals=vertex_normals,
        )
        log.info(f"  Spatial hash built: cell_size={cell_size:.4f}m, {n_tris} triangles")

        return cls(sh)

    def resolve(self, state: ParticleState, config: SimConfig) -> None:
        """
        Resolve body mesh collisions for all particles.

        Called inside the XPBD solver iteration loop by the engine (interleaved
        collision — the Vestra pattern).

        Delegates to the resolve_body_collision Taichi kernel, passing all
        spatial hash fields and collision parameters from config.
        """
        sh = self.spatial_hash
        resolve_body_collision(
            state.positions,
            state.predicted,
            state.inv_mass,
            state.n_particles,
            sh.cell_start,
            sh.cell_count,
            sh.entries,
            sh.tri_v0,
            sh.tri_v1,
            sh.tri_v2,
            sh.tri_n0,
            sh.tri_n1,
            sh.tri_n2,
            sh.n_triangles,
            sh.cell_size,
            sh.table_size,
            config.collision_thickness,
            config.friction_coefficient,
        )
