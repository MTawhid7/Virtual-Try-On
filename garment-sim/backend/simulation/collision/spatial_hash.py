"""
Static spatial hash for body mesh triangles.

Built once at initialization for the body proxy mesh. Maps 3D cells to lists
of triangle indices — O(1) cell lookup, then ~3-8 candidate triangles per cell.

Build strategy: NumPy counting sort (two-pass). No dynamic allocation in kernels.
Query: @ti.func methods for device-side access from the resolver kernel.

IMPORTANT: Do NOT add `from __future__ import annotations` to this file.
Taichi JIT cannot resolve string annotations on .template() kernel parameters.
"""

import warnings

import numpy as np
import taichi as ti
from numpy.typing import NDArray


# Hash function primes (Teschner et al. 2003)
_P1 = 73856093
_P2 = 19349663
_P3 = 83492791


@ti.data_oriented
class StaticSpatialHash:
    """
    Spatial hash for static triangle meshes (body proxy).

    Each cell stores indices of triangles whose AABB overlaps the cell.
    Query a world-space position → get (start, count) into the entries array →
    iterate candidate triangles.

    Cell size ≈ 1.5× average triangle edge length gives ~3-8 candidates per cell.

    We search 27 cells (3×3×3 neighborhood) rather than just the query cell.
    This handles particles near cell boundaries where the closest triangle
    might be in an adjacent cell. Correctness over micro-optimization.
    """

    def __init__(
        self,
        cell_size: float,
        table_size: int = 65536,
        max_triangles: int = 25000,
        max_entries: int = 200000,
    ) -> None:
        """
        Args:
            cell_size: Hash grid cell size in meters (≈ 1.5× avg edge length).
            table_size: Hash table size. Should be a power of 2 for good XOR distribution.
            max_triangles: Maximum number of body triangles.
            max_entries: Maximum total (cell, triangle) entries across all cells.
        """
        self.cell_size = cell_size
        self.table_size = table_size
        self.max_triangles = max_triangles
        self.max_entries = max_entries
        self.n_triangles = 0

        # --- Taichi fields ---
        # Hash table: cell_start[h] = start index into entries array for hash h
        #             cell_count[h] = number of triangles with hash h
        self.cell_start = ti.field(dtype=ti.i32, shape=table_size)
        self.cell_count = ti.field(dtype=ti.i32, shape=table_size)
        # Flat triangle-index array, sorted by hash cell
        self.entries = ti.field(dtype=ti.i32, shape=max_entries)

        # Body triangle geometry (pre-stored for kernel access — avoids NumPy in kernel)
        self.tri_v0 = ti.Vector.field(3, dtype=ti.f32, shape=max_triangles)
        self.tri_v1 = ti.Vector.field(3, dtype=ti.f32, shape=max_triangles)
        self.tri_v2 = ti.Vector.field(3, dtype=ti.f32, shape=max_triangles)
        self.tri_n0 = ti.Vector.field(3, dtype=ti.f32, shape=max_triangles)
        self.tri_n1 = ti.Vector.field(3, dtype=ti.f32, shape=max_triangles)
        self.tri_n2 = ti.Vector.field(3, dtype=ti.f32, shape=max_triangles)

    # ------------------------------------------------------------------
    # Build (NumPy — runs once, no GPU needed)
    # ------------------------------------------------------------------

    def build(
        self,
        vertices: NDArray,
        faces: NDArray,
        vertex_normals: NDArray,
    ) -> None:
        """
        Insert all body triangles into the spatial hash.

        For each triangle:
            1. Compute AABB (min/max of its 3 vertices)
            2. Find all cells that AABB overlaps
            3. Insert triangle index into each overlapping cell

        Two-pass counting sort:
            Pass 1: Count entries per cell → compute prefix sums (cell_start)
            Pass 2: Write triangle indices into entries array at correct offsets

        Args:
            vertices: (V, 3) float32 body vertex positions.
            faces: (F, 3) int32 triangle indices.
            vertex_normals: (V, 3) float32 vertex normals.
        """
        n_tris = len(faces)
        if n_tris > self.max_triangles:
            warnings.warn(
                f"Body mesh has {n_tris} triangles, but max_triangles={self.max_triangles}. "
                f"Clamping — some triangles will not be in the hash."
            )
            n_tris = self.max_triangles

        self.n_triangles = n_tris
        cell_size = self.cell_size
        table_size = self.table_size

        # Collect all (hash, tri_index) pairs
        pairs = []  # list of (hash_value, tri_index)

        for tri_idx in range(n_tris):
            i0, i1, i2 = faces[tri_idx]
            v0, v1, v2 = vertices[i0], vertices[i1], vertices[i2]

            aabb_min = np.minimum(np.minimum(v0, v1), v2)
            aabb_max = np.maximum(np.maximum(v0, v1), v2)

            # Cell range this AABB spans
            ix_min = int(np.floor(aabb_min[0] / cell_size))
            iy_min = int(np.floor(aabb_min[1] / cell_size))
            iz_min = int(np.floor(aabb_min[2] / cell_size))
            ix_max = int(np.floor(aabb_max[0] / cell_size))
            iy_max = int(np.floor(aabb_max[1] / cell_size))
            iz_max = int(np.floor(aabb_max[2] / cell_size))

            for ix in range(ix_min, ix_max + 1):
                for iy in range(iy_min, iy_max + 1):
                    for iz in range(iz_min, iz_max + 1):
                        h = _hash_cell_np(ix, iy, iz, table_size)
                        pairs.append((h, tri_idx))

        if len(pairs) > self.max_entries:
            warnings.warn(
                f"Spatial hash has {len(pairs)} entries, but max_entries={self.max_entries}. "
                f"Increase max_entries to avoid missing triangles."
            )
            pairs = pairs[: self.max_entries]

        if len(pairs) == 0:
            return

        # --- Counting sort (Pass 1: histogram) ---
        count_np = np.zeros(table_size, dtype=np.int32)
        for h, _ in pairs:
            count_np[h] += 1

        # Prefix sum → cell_start
        start_np = np.zeros(table_size, dtype=np.int32)
        start_np[1:] = np.cumsum(count_np[:-1])

        # --- Pass 2: write entries ---
        entries_np = np.zeros(self.max_entries, dtype=np.int32)
        write_ptr = start_np.copy()
        for h, tri_idx in pairs:
            entries_np[write_ptr[h]] = tri_idx
            write_ptr[h] += 1

        # --- Upload to Taichi fields ---
        self.cell_count.from_numpy(count_np)
        self.cell_start.from_numpy(start_np)
        self.entries.from_numpy(entries_np)

        # --- Upload triangle geometry ---
        v0_arr = np.zeros((self.max_triangles, 3), dtype=np.float32)
        v1_arr = np.zeros((self.max_triangles, 3), dtype=np.float32)
        v2_arr = np.zeros((self.max_triangles, 3), dtype=np.float32)
        n0_arr = np.zeros((self.max_triangles, 3), dtype=np.float32)
        n1_arr = np.zeros((self.max_triangles, 3), dtype=np.float32)
        n2_arr = np.zeros((self.max_triangles, 3), dtype=np.float32)

        for tri_idx in range(n_tris):
            i0, i1, i2 = faces[tri_idx]
            v0_arr[tri_idx] = vertices[i0]
            v1_arr[tri_idx] = vertices[i1]
            v2_arr[tri_idx] = vertices[i2]
            n0_arr[tri_idx] = vertex_normals[i0]
            n1_arr[tri_idx] = vertex_normals[i1]
            n2_arr[tri_idx] = vertex_normals[i2]

        self.tri_v0.from_numpy(v0_arr)
        self.tri_v1.from_numpy(v1_arr)
        self.tri_v2.from_numpy(v2_arr)
        self.tri_n0.from_numpy(n0_arr)
        self.tri_n1.from_numpy(n1_arr)
        self.tri_n2.from_numpy(n2_arr)

    # ------------------------------------------------------------------
    # Device-side query methods (@ti.func — called from resolver kernel)
    # ------------------------------------------------------------------

    @ti.func
    def hash_cell(self, ix: ti.i32, iy: ti.i32, iz: ti.i32) -> ti.i32:
        """Spatial hash function (Teschner et al. 2003). Returns index into cell_start/cell_count."""
        h = (ix * _P1) ^ (iy * _P2) ^ (iz * _P3)
        # Ensure non-negative modulo
        h = h % self.table_size
        if h < 0:
            h += self.table_size
        return h

    @ti.func
    def cell_indices(self, pos: ti.math.vec3) -> ti.math.vec3:
        """Convert world position to integer cell indices (ix, iy, iz)."""
        ix = ti.cast(ti.floor(pos.x / self.cell_size), ti.i32)
        iy = ti.cast(ti.floor(pos.y / self.cell_size), ti.i32)
        iz = ti.cast(ti.floor(pos.z / self.cell_size), ti.i32)
        return ti.math.vec3(ti.cast(ix, ti.f32), ti.cast(iy, ti.f32), ti.cast(iz, ti.f32))


# ------------------------------------------------------------------
# NumPy-side hash helper (for build() and tests)
# ------------------------------------------------------------------

def _hash_cell_np(ix: int, iy: int, iz: int, table_size: int) -> int:
    """
    Python-side hash for build() and unit tests. Must match @ti.func hash_cell().

    Simulates ti.i32 wraparound arithmetic via bitmask — Python's unlimited integers
    would give different XOR/modulo results when multiplication overflows 32 bits
    (e.g. ix=32: 32 * 73856093 = 2.36e9 > i32 max of 2.15e9, wraps in Taichi).

    Steps:
        1. Mask each term to 32-bit unsigned (same bit pattern as ti.i32 after overflow)
        2. XOR the three uint32 values → uint32 result
        3. Convert to signed i32 range (same modulo semantics as Taichi's if h<0 branch)
        4. Take modulo to get a valid table index (Python % always returns non-negative)
    """
    # Step 1+2: mask to 32-bit unsigned before XOR (simulates ti.i32 overflow)
    h = ((ix * _P1) & 0xFFFFFFFF) ^ ((iy * _P2) & 0xFFFFFFFF) ^ ((iz * _P3) & 0xFFFFFFFF)
    # Step 3: uint32 → signed i32 (>0x7FFFFFFF wraps to negative in i32)
    if h > 0x7FFFFFFF:
        h -= 0x100000000
    # Step 4: modulo. Python % always non-negative; Taichi uses `if h<0: h+=table_size`
    h = h % table_size
    return h
