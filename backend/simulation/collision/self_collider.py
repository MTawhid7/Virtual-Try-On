"""
Cloth self-collision orchestrator — Sprint 2 Fabric Realism Track C.

Builds a dynamic spatial hash each substep from current cloth triangle centroids,
then resolves particle-triangle self-collisions via the resolve_self_collision kernel.

Interface matches BodyCollider:
    self_collider.resolve(state, config)

This file may use `from __future__ import annotations` (no @ti.kernel here —
kernels live in self_resolver.py).
"""

from __future__ import annotations

import numpy as np
import taichi as ti
from numpy.typing import NDArray

from simulation.collision.self_resolver import resolve_self_collision
from simulation.core.state import ParticleState

# Hash function primes — must match self_resolver.py and spatial_hash.py
_P1 = 73856093
_P2 = 19349663
_P3 = 83492791


class ClothSelfCollider:
    """
    Cloth self-collision resolver with dynamic per-substep spatial hash.

    The hash maps triangle centroids to cells. Each substep:
        1. Current positions are read from the GPU
        2. Triangle centroids are hashed (vectorised numpy, ~0.1ms for 7k faces)
        3. resolve_self_collision kernel runs on GPU

    1-ring exclusion (skip triangles sharing the query vertex) is enforced
    inside the kernel — no pre-computed CSR needed for vertex-equality exclusion.

    Usage:
        sc = ClothSelfCollider.from_mesh(faces_np, positions_np, thickness=0.004)
        engine.self_collider = sc
    """

    def __init__(
        self,
        n_particles: int,
        n_faces: int,
        faces_np: NDArray[np.int32],
        cell_size: float,
        thickness: float,
        table_size: int = 65536,
        max_entries: int = 200000,
    ) -> None:
        self.n_particles = n_particles
        self.n_faces = n_faces
        self.cell_size = cell_size
        self.thickness = thickness
        self.table_size = table_size
        self.max_entries = max_entries

        # Cached numpy face array — avoids GPU round-trip each substep
        self._faces_np = faces_np.astype(np.int32)

        # Dynamic hash fields (overwritten each substep from numpy)
        self.cell_start = ti.field(dtype=ti.i32, shape=table_size)
        self.cell_count = ti.field(dtype=ti.i32, shape=table_size)
        self.entries = ti.field(dtype=ti.i32, shape=max_entries)

        # Face topology on GPU (static — cloth topology doesn't change)
        self.faces_field = ti.Vector.field(3, dtype=ti.i32, shape=n_faces)
        self.faces_field.from_numpy(faces_np.astype(np.int32))

    @classmethod
    def from_mesh(
        cls,
        faces_np: NDArray[np.int32],
        positions_np: NDArray[np.float32],
        thickness: float = 0.004,
        cell_size: float | None = None,
    ) -> ClothSelfCollider:
        """
        Factory: create a ClothSelfCollider from cloth mesh topology and rest positions.

        Args:
            faces_np:     (F, 3) int32 triangle indices.
            positions_np: (N, 3) float32 rest vertex positions.
            thickness:    Self-collision push-out distance in meters (default 4mm).
            cell_size:    Hash cell size. Defaults to 1.5× mean edge length.
        """
        if cell_size is None:
            v0 = positions_np[faces_np[:, 0]]
            v1 = positions_np[faces_np[:, 1]]
            v2 = positions_np[faces_np[:, 2]]
            e01 = np.linalg.norm(v1 - v0, axis=1)
            e12 = np.linalg.norm(v2 - v1, axis=1)
            e20 = np.linalg.norm(v0 - v2, axis=1)
            avg_edge = float(np.mean(np.concatenate([e01, e12, e20])))
            cell_size = 1.5 * avg_edge

        return cls(
            n_particles=len(positions_np),
            n_faces=len(faces_np),
            faces_np=faces_np,
            cell_size=cell_size,
            thickness=thickness,
        )

    def rebuild_hash(self, positions_np: NDArray[np.float32]) -> None:
        """
        Rebuild the dynamic spatial hash from current cloth positions.

        Each triangle contributes one entry keyed by its centroid cell.
        The 27-cell search in the kernel compensates for triangles near cell
        boundaries — no AABB expansion needed.

        Fully vectorised numpy: ~0.1ms for 7k faces.
        """
        fnp = self._faces_np
        v0 = positions_np[fnp[:, 0]]
        v1 = positions_np[fnp[:, 1]]
        v2 = positions_np[fnp[:, 2]]

        cent = (v0 + v1 + v2) / 3.0
        cs = self.cell_size
        ts = self.table_size

        ix = np.floor(cent[:, 0] / cs).astype(np.int64)
        iy = np.floor(cent[:, 1] / cs).astype(np.int64)
        iz = np.floor(cent[:, 2] / cs).astype(np.int64)

        # Vectorised hash matching the Taichi kernel (i32 wraparound via bitmask)
        h = (
            ((ix * _P1) & 0xFFFFFFFF)
            ^ ((iy * _P2) & 0xFFFFFFFF)
            ^ ((iz * _P3) & 0xFFFFFFFF)
        ).astype(np.int64)
        h[h > 0x7FFFFFFF] -= 0x100000000
        h = (h % ts).astype(np.int32)

        # Counting sort: argsort groups face indices by their hash cell
        sort_idx = np.argsort(h, kind="stable").astype(np.int32)
        h_sorted = h[sort_idx]

        count_np = np.bincount(h_sorted, minlength=ts).astype(np.int32)
        start_np = np.zeros(ts, dtype=np.int32)
        start_np[1:] = np.cumsum(count_np[:-1])

        entries_np = np.zeros(self.max_entries, dtype=np.int32)
        n = self.n_faces
        entries_np[:n] = sort_idx

        self.cell_count.from_numpy(count_np)
        self.cell_start.from_numpy(start_np)
        self.entries.from_numpy(entries_np)

    def update_hash(self, state: ParticleState) -> None:
        """
        Rebuild the spatial hash from current cloth positions.

        Called ONCE per substep by the engine, before the solver iteration
        loop. Separating hash rebuild from kernel dispatch avoids 8× GPU→CPU
        syncs per substep (one per solver iteration), which caused both
        massive performance overhead and cascade instability.
        """
        positions_np = state.positions.to_numpy()[: state.n_particles]
        self.rebuild_hash(positions_np)

    def resolve(self, state: ParticleState) -> None:
        """
        Resolve self-collisions using the pre-built spatial hash.

        Called inside the XPBD solver iteration loop by the engine (after
        body collision). Hash must already be up-to-date via update_hash().
        Normal-only correction: no tangential/friction component.
        """
        resolve_self_collision(
            state.positions,
            state.inv_mass,
            state.n_particles,
            self.faces_field,
            self.n_faces,
            self.cell_start,
            self.cell_count,
            self.entries,
            self.table_size,
            self.cell_size,
            self.thickness,
        )
