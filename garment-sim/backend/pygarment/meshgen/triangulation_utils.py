"""
    Triangulation utilities for panel mesh generation.

    VENDORED & REWRITTEN: Original GarmentCode used CGAL's
    Mesh_2_Constrained_Delaunay_triangulation_2 for constrained Delaunay
    triangulation. This version uses the 'triangle' library (Shewchuk's
    Triangle) which provides equivalent CDT + refinement without the
    CGAL C++ dependency.

    The `triangle` library's 'p' (PSLG) flag automatically handles
    inside/outside domain detection — replacing CGAL's mark_domain()
    and FaceInfo2 machinery entirely.

    Original: pygarment/meshgen/triangulation_utils.py (GarmentCode)
"""

import numpy as np
import triangle as tr


def get_edge_vert_ids(edges):
    """
    Returns a list of index pairs of edge vertices into their corresponding
    panel.panel_vertices defining the border of the panel.

    Input:
        * edges (list): All edges of a panel
    Output:
        * zipped_array (ndarray): ndarray of start and end indices of edge
          vertices into panel.vertices defining the line segments of the
          panel edges (e.g. [[0,1],[1,2],[2,3],...,[19,20],[20,0]])
    """
    zipped_array = np.empty((0, 2))
    for edge in edges:
        edge_verts_ids = edge.vertex_range
        rolled_list = np.roll(edge_verts_ids, 1, axis=0)
        zipped_array_edge = np.stack((rolled_list, edge_verts_ids), axis=1)[1:]
        zipped_array = np.concatenate((zipped_array, zipped_array_edge), axis=0)

    return zipped_array.astype(int)


def gen_panel_mesh_triangle(points, edge_verts_ids, mesh_resolution,
                            min_angle=30.0):
    """
    Triangulate a 2D panel using the 'triangle' library's constrained
    Delaunay triangulation with Ruppert refinement.

    This replaces the original CGAL pipeline:
        1. CGAL CDT + refine_Delaunay_mesh_2(criteria)
        2. mark_domain() for inside/outside detection
        3. get_keep_vertices() to remove spurious boundary points
        4. Second CDT pass to clean up

    The 'triangle' library handles all of this with the 'p' (PSLG) flag,
    which automatically constrains to the polygon interior.

    Args:
        points:          (N, 2) list/array of boundary vertices
        edge_verts_ids:  (E, 2) array of segment index pairs defining
                         the polygon boundary as a PSLG
        mesh_resolution: target edge length (in the same units as points,
                         typically cm for GarmentCode)
        min_angle:       minimum triangle angle in degrees (default 30°,
                         equivalent to CGAL's 0.125 criterion)

    Returns:
        (new_vertices, faces):
            new_vertices — list of [x, y] vertex positions (boundary +
                           interior Steiner points)
            faces        — list of [v0, v1, v2] triangle face indices
    """
    points_arr = np.array(points, dtype=np.float64)
    segments = np.array(edge_verts_ids, dtype=np.int32)

    # Target area for equilateral triangle with the given edge length.
    # Factor 1.43 matches GarmentCode's CGAL criteria:
    #   refine_Delaunay_mesh_2(cdt, criteria(0.125, 1.43 * resolution))
    target_area = (np.sqrt(3) / 4) * (1.43 * mesh_resolution) ** 2

    # Build input for the triangle library
    data = {
        'vertices': points_arr,
        'segments': segments,
    }

    # Triangle options:
    #   p  = Triangulate a Planar Straight Line Graph (auto inside/outside)
    #   q  = Quality mesh with minimum angle constraint
    #   a  = Maximum triangle area constraint
    #   Y  = Prohibit insertion of Steiner points on boundary segments
    #        (critical: preserves the original boundary vertices for stitch
    #         matching — equivalent to GarmentCode's get_keep_vertices())
    opts = f'pq{min_angle:.0f}a{target_area:.8f}Y'

    result = tr.triangulate(data, opts)

    new_vertices = result['vertices'].tolist()
    faces = result['triangles'].tolist()

    return new_vertices, faces


def is_manifold(face_v_ids, points, tol=1e-2):
    """Check if the 2D mesh is manifold — all face triangles are proper
    triangles (no degenerate slivers).

    This function is unchanged from the original GarmentCode.
    """
    face_v_ids = np.asarray(face_v_ids)
    points = np.asarray(points)

    faces = points[face_v_ids]
    face_side_1 = np.linalg.norm(faces[:, 0] - faces[:, 1], axis=1)
    face_side_2 = np.linalg.norm(faces[:, 1] - faces[:, 2], axis=1)
    face_side_3 = np.linalg.norm(faces[:, 0] - faces[:, 2], axis=1)
    side_lengths = np.stack([face_side_1, face_side_2, face_side_3], axis=-1)

    return bool(np.all(
        side_lengths.sum(axis=1) > 2 * side_lengths.max(axis=1) + tol
    ))
