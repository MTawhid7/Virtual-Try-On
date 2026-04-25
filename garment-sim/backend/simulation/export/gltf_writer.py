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
  - Animated export (write_glb_animated): uses raw glTF 2.0 binary with
    morph targets + animated weight clip. No extra dependencies beyond
    Python builtins (json, struct). K keyframes → K morph targets; at
    time T[k] weight[k]=1, enabling smooth LINEAR interpolation between
    simulation frames in Three.js AnimationMixer.
"""

from __future__ import annotations

import json
import struct
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


def write_glb_with_body(
    cloth_positions: NDArray[np.float32],
    cloth_faces: NDArray[np.int32],
    body_positions: NDArray[np.float32],
    body_faces: NDArray[np.int32],
    cloth_normals: NDArray[np.float32] | None = None,
    cloth_uvs: NDArray[np.float32] | None = None,
    path: str | Path = "output.glb",
) -> Path:
    """
    Export cloth + body as two named meshes in a single GLB.

    The React Three Fiber viewer detects mesh names:
      "cloth" → applies fabric material
      "body"  → applies skin material

    Args:
        cloth_positions: (N, 3) cloth vertex positions.
        cloth_faces:     (F, 3) cloth triangle indices.
        body_positions:  (M, 3) body vertex positions.
        body_faces:      (G, 3) body triangle indices.
        cloth_normals:   (N, 3) cloth vertex normals. Optional.
        cloth_uvs:       (N, 2) cloth UV coordinates. Optional.
        path:            Output file path.

    Returns:
        Resolved Path to the written .glb file.
    """
    path = Path(path)

    # Build cloth mesh
    cloth_kwargs: dict = {
        "vertices": cloth_positions.astype(np.float64),
        "faces": cloth_faces.astype(np.int64),
        "process": False,
    }
    if cloth_normals is not None:
        cloth_kwargs["vertex_normals"] = cloth_normals.astype(np.float64)
    cloth_mesh = trimesh.Trimesh(**cloth_kwargs)
    if cloth_uvs is not None:
        cloth_mesh.visual = trimesh.visual.TextureVisuals(
            uv=cloth_uvs.astype(np.float64)
        )

    # Build body mesh
    body_mesh = trimesh.Trimesh(
        vertices=body_positions.astype(np.float64),
        faces=body_faces.astype(np.int64),
        process=False,
    )

    scene = trimesh.Scene(geometry={"cloth": cloth_mesh, "body": body_mesh})

    path.parent.mkdir(parents=True, exist_ok=True)
    scene.export(file_obj=str(path), file_type="glb")

    return path.resolve()


def write_glb_animated(
    frame_positions: list[NDArray[np.float32]],
    cloth_faces: NDArray[np.int32],
    body_positions: NDArray[np.float32],
    body_faces: NDArray[np.int32],
    fps: float = 6.0,
    path: str | Path = "output_animated.glb",
) -> Path:
    """
    Export an animated cloth simulation as a single GLB using glTF 2.0 morph targets.

    The cloth mesh is animated via K morph targets (one per keyframe). At time
    T[k] = k/fps, weight[k]=1 and all other weights=0. LINEAR interpolation in
    Three.js AnimationMixer smoothly blends between adjacent simulation frames.

    The body mesh is embedded as a static named mesh ("body"). The cloth mesh is
    named "cloth" — the R3F viewer applies fabric/skin materials by these names.

    glTF binary is written directly (no extra dependencies beyond json + struct)
    to avoid trimesh's lack of morph-target animation support.

    Args:
        frame_positions: K snapshots of cloth vertex positions, each (N, 3).
                         frame_positions[0] is the initial panel layout (base pose).
        cloth_faces:     (F, 3) cloth triangle indices (constant across frames).
        body_positions:  (M, 3) body vertex positions (static).
        body_faces:      (G, 3) body triangle indices.
        fps:             Keyframe playback rate. Default 6.0 means each captured
                         frame occupies 1/6 s — at record_every_n_frames=5 from a
                         30fps sim, real time is compressed to ~6s of animation.
        path:            Output .glb file path.

    Returns:
        Resolved Path to the written file.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    K = len(frame_positions)
    if K < 2:
        raise ValueError(f"Need at least 2 frames for animation, got {K}")

    base_pos = frame_positions[0].astype(np.float32)   # (N, 3) — initial panel layout
    N = base_pos.shape[0]
    F = cloth_faces.shape[0]
    M = body_positions.shape[0]
    G = body_faces.shape[0]

    # -------------------------------------------------------------------------
    # Build the binary buffer.  Each section is 4-byte aligned.
    # -------------------------------------------------------------------------
    buf = bytearray()
    buffer_views: list[dict] = []
    accessors: list[dict] = []

    ARRAY_BUFFER   = 34962
    ELEMENT_ARRAY  = 34963
    FLOAT          = 5126
    UNSIGNED_INT   = 5125

    def _pad4(data: bytes) -> bytes:
        r = len(data) % 4
        return data + b"\x00" * ((4 - r) % 4)

    def _add(data: bytes, target: int | None = None) -> int:
        """Append data (padded) to buf, register a bufferView, return its index."""
        padded = _pad4(data)
        bv: dict = {"buffer": 0, "byteOffset": len(buf), "byteLength": len(padded)}
        if target is not None:
            bv["target"] = target
        buf.extend(padded)
        buffer_views.append(bv)
        return len(buffer_views) - 1

    def _acc(bv_idx: int, comp_type: int, count: int, typ: str,
             min_vals: list | None = None, max_vals: list | None = None) -> int:
        acc: dict = {
            "bufferView": bv_idx,
            "componentType": comp_type,
            "count": count,
            "type": typ,
        }
        if min_vals is not None:
            acc["min"] = min_vals
        if max_vals is not None:
            acc["max"] = max_vals
        accessors.append(acc)
        return len(accessors) - 1

    # --- Cloth base positions ---
    bv = _add(base_pos.tobytes(), ARRAY_BUFFER)
    acc_cloth_pos = _acc(bv, FLOAT, N, "VEC3",
                         base_pos.min(0).tolist(), base_pos.max(0).tolist())

    # --- Cloth face indices (uint32) ---
    idx_data = cloth_faces.astype(np.uint32).flatten().tobytes()
    bv = _add(idx_data, ELEMENT_ARRAY)
    acc_cloth_idx = _acc(bv, UNSIGNED_INT, F * 3, "SCALAR")

    # --- Morph target position deltas (one per keyframe) ---
    morph_acc: list[int] = []
    for k, frame_pos in enumerate(frame_positions):
        delta = (frame_pos.astype(np.float32) - base_pos)
        bv = _add(delta.tobytes(), ARRAY_BUFFER)
        a = _acc(bv, FLOAT, N, "VEC3",
                 delta.min(0).tolist(), delta.max(0).tolist())
        morph_acc.append(a)

    # --- Body positions ---
    body_pos32 = body_positions.astype(np.float32)
    bv = _add(body_pos32.tobytes(), ARRAY_BUFFER)
    acc_body_pos = _acc(bv, FLOAT, M, "VEC3",
                        body_pos32.min(0).tolist(), body_pos32.max(0).tolist())

    # --- Body face indices (uint32) ---
    bv = _add(body_faces.astype(np.uint32).flatten().tobytes(), ELEMENT_ARRAY)
    acc_body_idx = _acc(bv, UNSIGNED_INT, G * 3, "SCALAR")

    # --- Animation times: [0, 1/fps, 2/fps, ..., (K-1)/fps] ---
    times = np.linspace(0.0, (K - 1) / fps, K, dtype=np.float32)
    bv = _add(times.tobytes())
    acc_times = _acc(bv, FLOAT, K, "SCALAR", [0.0], [float(times[-1])])

    # --- Animation weights: K×K identity matrix (flattened row-major).
    #     At time T[k]: weight[k]=1, all others=0. LINEAR interpolation
    #     between keyframes gives smooth blending. ---
    weights = np.eye(K, dtype=np.float32).flatten()
    bv = _add(weights.tobytes())
    acc_weights = _acc(bv, FLOAT, K * K, "SCALAR")

    # -------------------------------------------------------------------------
    # Build glTF JSON
    # -------------------------------------------------------------------------
    gltf = {
        "asset": {"version": "2.0", "generator": "garment-sim"},
        "scene": 0,
        "scenes": [{"nodes": [0, 1]}],
        "nodes": [
            {"mesh": 0, "name": "cloth"},
            {"mesh": 1, "name": "body"},
        ],
        "meshes": [
            {
                "name": "cloth",
                "primitives": [{
                    "attributes": {"POSITION": acc_cloth_pos},
                    "indices": acc_cloth_idx,
                    "targets": [{"POSITION": a} for a in morph_acc],
                    "mode": 4,  # TRIANGLES
                }],
                "weights": [0.0] * K,
            },
            {
                "name": "body",
                "primitives": [{
                    "attributes": {"POSITION": acc_body_pos},
                    "indices": acc_body_idx,
                    "mode": 4,
                }],
            },
        ],
        "animations": [{
            "name": "Drape",
            "channels": [{
                "sampler": 0,
                "target": {"node": 0, "path": "weights"},
            }],
            "samplers": [{
                "input": acc_times,
                "output": acc_weights,
                "interpolation": "LINEAR",
            }],
        }],
        "accessors": accessors,
        "bufferViews": buffer_views,
        "buffers": [{"byteLength": len(buf)}],
    }

    # -------------------------------------------------------------------------
    # Pack GLB: header + JSON chunk + BIN chunk
    # -------------------------------------------------------------------------
    json_bytes = json.dumps(gltf, separators=(",", ":")).encode("utf-8")
    # JSON chunk must be padded to 4-byte boundary with spaces (0x20)
    r = len(json_bytes) % 4
    if r:
        json_bytes += b" " * (4 - r)

    bin_bytes = bytes(buf)

    total_len = 12 + 8 + len(json_bytes) + 8 + len(bin_bytes)
    header     = struct.pack("<III", 0x46546C67, 2, total_len)
    json_chunk = struct.pack("<II", len(json_bytes), 0x4E4F534A) + json_bytes
    bin_chunk  = struct.pack("<II", len(bin_bytes),  0x004E4942) + bin_bytes

    with open(path, "wb") as f:
        f.write(header + json_chunk + bin_chunk)

    return path.resolve()
