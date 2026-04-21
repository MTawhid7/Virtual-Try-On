"""
Grid mesh generator — creates a flat NxN cloth grid for testing.

Produces positions, edges (for distance constraints), and triangle faces
(for bending constraints and rendering). The grid lies in the XZ plane
at a given Y height, centered at the origin.

This is the simplest mesh for validating the physics pipeline before
moving to pattern triangulation in Sprint 2.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


@dataclass
class GridMesh:
    """Result of grid generation."""

    positions: NDArray[np.float32]   # (N, 3) vertex positions
    faces: NDArray[np.int32]         # (F, 3) triangle indices
    edges: NDArray[np.int32]         # (E, 2) unique edge pairs
    resolution: tuple[int, int]      # (rows, cols) grid dimensions


def compute_area_weighted_inv_masses(
    positions: NDArray[np.float32],
    faces: NDArray[np.int32],
    density: float,
    max_inv_mass: float = 5000.0,
) -> NDArray[np.float32]:
    """
    Compute per-vertex inverse masses using the lumped-mass FEM approach.

    Each triangle distributes its area equally to its three vertices (area/3).
    The vertex mass is then density × vertex_area. This gives physically correct
    inertia scaling: finer meshes produce lighter particles, not the same 1 kg
    per particle that uniform inv_mass=1.0 produces.

    Args:
        positions: (N, 3) vertex positions.
        faces:     (F, 3) triangle vertex indices.
        density:   Fabric surface density in kg/m².

    Returns:
        (N,) array of inverse masses (1/mass per vertex), float32.
    """
    n_verts = positions.shape[0]
    vertex_area = np.zeros(n_verts, dtype=np.float64)

    v0 = positions[faces[:, 0]]
    v1 = positions[faces[:, 1]]
    v2 = positions[faces[:, 2]]
    tri_areas = np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1) * 0.5

    np.add.at(vertex_area, faces[:, 0], tri_areas / 3.0)
    np.add.at(vertex_area, faces[:, 1], tri_areas / 3.0)
    np.add.at(vertex_area, faces[:, 2], tri_areas / 3.0)

    vertex_area = np.maximum(vertex_area, 1e-12)  # guard against degenerate verts
    inv_mass = 1.0 / (density * vertex_area)
    return np.minimum(inv_mass, max_inv_mass).astype(np.float32)


def generate_grid(
    width: float = 1.0,
    height: float = 1.0,
    cols: int = 10,
    rows: int = 10,
    center: tuple[float, float, float] = (0.0, 2.0, 0.0),
) -> GridMesh:
    """
    Generate a flat rectangular cloth grid in the XZ plane.

    The grid is centered at `center` with the given width (X) and height (Z).
    Uses alternating diagonal triangulation (checkerboard) to avoid
    structural anisotropy — the same fix used in Vistio (Feb 24 worklog).

    Args:
        width:  Grid extent in X direction (meters).
        height: Grid extent in Z direction (meters).
        cols:   Number of vertices along X.
        rows:   Number of vertices along Z.
        center: 3D position of the grid center.

    Returns:
        GridMesh with positions, faces, edges, and resolution.
    """
    if cols < 2 or rows < 2:
        raise ValueError(f"Grid must be at least 2×2, got {cols}×{rows}")

    # --- Generate vertex positions ---
    n_verts = rows * cols
    positions = np.zeros((n_verts, 3), dtype=np.float32)

    for r in range(rows):
        for c in range(cols):
            idx = r * cols + c
            # X: left to right, Y: constant height, Z: front to back
            positions[idx, 0] = center[0] + (c / (cols - 1) - 0.5) * width
            positions[idx, 1] = center[1]
            positions[idx, 2] = center[2] + (r / (rows - 1) - 0.5) * height

    # --- Generate triangles (checkerboard pattern) ---
    faces_list: list[tuple[int, int, int]] = []

    for r in range(rows - 1):
        for c in range(cols - 1):
            # Four corners of the quad
            tl = r * cols + c            # top-left
            tr = r * cols + (c + 1)      # top-right
            bl = (r + 1) * cols + c      # bottom-left
            br = (r + 1) * cols + (c + 1)  # bottom-right

            # Alternate diagonal direction to prevent structural anisotropy
            if (r + c) % 2 == 0:
                # Diagonal: top-left to bottom-right
                faces_list.append((tl, bl, br))
                faces_list.append((tl, br, tr))
            else:
                # Diagonal: top-right to bottom-left
                faces_list.append((tl, bl, tr))
                faces_list.append((bl, br, tr))

    faces = np.array(faces_list, dtype=np.int32)

    # --- Extract unique edges ---
    edge_set: set[tuple[int, int]] = set()
    for f in faces_list:
        for i in range(3):
            a, b = f[i], f[(i + 1) % 3]
            edge = (min(a, b), max(a, b))
            edge_set.add(edge)

    # Add shear (cross-diagonal) edges for in-plane resistance.
    # Each quad has one diagonal from the triangulation; add the other diagonal
    # to prevent quads from freely collapsing into parallelograms.
    for r in range(rows - 1):
        for c in range(cols - 1):
            tl = r * cols + c
            tr = r * cols + (c + 1)
            bl = (r + 1) * cols + c
            br = (r + 1) * cols + (c + 1)
            if (r + c) % 2 == 0:
                # triangulation uses tl-br diagonal; add cross-diagonal tr-bl
                edge_set.add((min(tr, bl), max(tr, bl)))
            else:
                # triangulation uses tr-bl diagonal; add cross-diagonal tl-br
                edge_set.add((min(tl, br), max(tl, br)))

    edges = np.array(sorted(edge_set), dtype=np.int32)

    return GridMesh(
        positions=positions,
        faces=faces,
        edges=edges,
        resolution=(rows, cols),
    )
