"""
glTF/GLB export — writes simulation results to binary glTF (.glb) files.

Uses trimesh to construct a mesh from positions, faces, and normals, then
exports as a single binary .glb file. This is the standard export format
for the simulation engine — .glb files can be loaded in Blender, three.js,
any glTF-compatible viewer, and our own React Three Fiber frontend.

Design decisions:
  - Cloth-only export: the GLB contains only the simulation output mesh.
    Collision geometry (sphere, body) is excluded — use the `-v` flag for
    live visualization with collider geometry.
  - Stateless: normals are pre-computed by `compute_vertex_normals()` in
    engine.py and passed in. The writer has no physics knowledge.
  - UVs are optional: Sprint 1 grids don't have meaningful UVs. The
    interface is ready for Sprint 2 pattern panels.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import trimesh
from numpy.typing import NDArray


def write_glb(
    positions: NDArray[np.float32],
    faces: NDArray[np.int32],
    normals: NDArray[np.float32] | None = None,
    uvs: NDArray[np.float32] | None = None,
    path: str | Path = "output.glb",
) -> Path:
    """
    Export a mesh to a binary glTF (.glb) file.

    Args:
        positions: (N, 3) vertex positions.
        faces:     (F, 3) triangle vertex indices.
        normals:   (N, 3) vertex normals. If None, trimesh will compute them.
        uvs:       (N, 2) UV texture coordinates. Optional.
        path:      Output file path. Parent directories are created if needed.

    Returns:
        Resolved Path to the written .glb file.

    Raises:
        ValueError: If positions or faces have invalid shapes.
    """
    path = Path(path)

    # --- Input validation ---
    if positions.ndim != 2 or positions.shape[1] != 3:
        raise ValueError(
            f"positions must be (N, 3), got {positions.shape}"
        )
    if faces.ndim != 2 or faces.shape[1] != 3:
        raise ValueError(
            f"faces must be (F, 3), got {faces.shape}"
        )
    if len(faces) == 0:
        raise ValueError("Cannot export mesh with zero faces.")

    # --- Build trimesh Mesh ---
    # Trimesh accepts vertex_normals as a construction parameter.
    # If normals are provided, we pass them to avoid recomputation.
    kwargs: dict = {
        "vertices": positions.astype(np.float64),
        "faces": faces.astype(np.int64),
        "process": False,  # Don't merge/reorder vertices — preserve our indexing
    }

    if normals is not None:
        if normals.shape != positions.shape:
            raise ValueError(
                f"normals shape {normals.shape} must match positions shape {positions.shape}"
            )
        kwargs["vertex_normals"] = normals.astype(np.float64)

    mesh = trimesh.Trimesh(**kwargs)

    # --- Attach UVs as visual ---
    if uvs is not None:
        if uvs.shape[0] != positions.shape[0] or uvs.shape[1] != 2:
            raise ValueError(
                f"uvs must be (N, 2), got {uvs.shape}"
            )
        # trimesh stores UVs in the visual property
        mesh.visual = trimesh.visual.TextureVisuals(uv=uvs.astype(np.float64))

    # --- Wrap in a Scene for proper glTF export ---
    scene = trimesh.Scene(geometry={"cloth": mesh})

    # --- Ensure output directory exists ---
    path.parent.mkdir(parents=True, exist_ok=True)

    # --- Export ---
    scene.export(file_obj=str(path), file_type="glb")

    return path.resolve()
