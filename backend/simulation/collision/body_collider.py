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
        Factory: uses smart_process to obtain a clean physics proxy, then builds spatial hash.
        """
        path = Path(path)
        log.info(f"BodyCollider: initializing from {path}")

        # 1. Obtain clean physics proxy.
        # If the path is already a pre-processed proxy (stem ends with '_physics'),
        # load it directly — running smart_process on it would create a
        # double-suffixed '_physics_physics.glb' output. Otherwise, run the
        # preprocessing pipeline to produce/validate the proxy.
        if path.stem.endswith("_physics"):
            log.info(f"  Path is already a physics proxy — loading directly.")
            mesh = trimesh.load(str(path), force="mesh")
        else:
            try:
                from scripts.process_body import smart_process
                physics_path = smart_process(path, target_height=target_height, decimate_target=decimate_target or 5000)
                log.info(f"  Using processed physics proxy: {physics_path}")
                mesh = trimesh.load(str(physics_path), force="mesh")
            except ImportError:
                log.warning("Could not import smart_process pipeline. Loading raw mesh.")
                mesh = trimesh.load(str(path), force="mesh")
            
        log.info(f"  Loaded: {len(mesh.vertices)} vertices, {len(mesh.faces)} faces")

        # 2. Quality metrics and cell size
        edge_lengths = np.linalg.norm(
            mesh.vertices[mesh.edges_unique[:, 1]] - mesh.vertices[mesh.edges_unique[:, 0]],
            axis=1,
        )
        log.info(
            f"  Physics proxy edge length: min={edge_lengths.min():.4f} "
            f"avg={edge_lengths.mean():.4f} "
            f"max={edge_lengths.max():.4f} m"
        )

        if cell_size is None:
            cell_size = float(1.5 * edge_lengths.mean())
            log.info(f"  Auto cell_size: {cell_size:.4f} m")

        # Vertex normals for smooth normal interpolation (Vestra "smoothed normal trick")
        vertex_normals = np.array(mesh.vertex_normals, dtype=np.float32)

        # 6. Build spatial hash
        n_tris = len(mesh.faces)
        sh = StaticSpatialHash(
            cell_size=cell_size,
            table_size=262144,
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
