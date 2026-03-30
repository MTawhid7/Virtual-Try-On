//! Mesh topology queries.
//!
//! Builds adjacency data structures from the triangle index buffer,
//! enabling efficient neighbor queries needed by the solver
//! (vertex-to-triangle, edge adjacency, etc.).

use std::collections::{HashMap, HashSet};

use crate::mesh::TriangleMesh;

/// Precomputed topology information for a triangle mesh.
///
/// Built once when a mesh is loaded (or after remeshing).
/// Provides O(1) adjacency queries used by:
/// - Bending constraint generation (edge neighbors)
/// - Self-collision exclusion zones (1-ring neighborhood)
/// - Normal computation (vertex-to-triangle fan)
#[derive(Debug, Clone)]
pub struct Topology {
    /// For each vertex, the list of triangles that contain it.
    pub vertex_triangles: Vec<Vec<u32>>,

    /// Unique edges as `(v_min, v_max)` pairs.
    pub edges: Vec<[u32; 2]>,

    /// For each edge, the one or two adjacent triangles.
    /// Boundary edges have exactly 1 adjacent triangle.
    pub edge_triangles: Vec<Vec<u32>>,

    /// Interior edges that have exactly 2 adjacent triangles.
    /// These are the edges where bending constraints are applied.
    /// Stored as `(edge_index, tri_a, tri_b)`.
    pub interior_edges: Vec<InteriorEdge>,
}

/// An interior (non-boundary) edge with its two adjacent triangles.
///
/// Used for bending constraint generation: the dihedral angle
/// between tri_a and tri_b across this edge defines the bending energy.
#[derive(Debug, Clone, Copy)]
pub struct InteriorEdge {
    /// Index of vertex A of the shared edge.
    pub v0: u32,
    /// Index of vertex B of the shared edge.
    pub v1: u32,
    /// The "wing" vertex of triangle A (not on the edge).
    pub wing_a: u32,
    /// The "wing" vertex of triangle B (not on the edge).
    pub wing_b: u32,
    /// Index of adjacent triangle A.
    pub tri_a: u32,
    /// Index of adjacent triangle B.
    pub tri_b: u32,
}

impl Topology {
    /// Build topology from a triangle mesh.
    pub fn build(mesh: &TriangleMesh) -> Self {
        let vertex_count = mesh.vertex_count();
        let tri_count = mesh.triangle_count();

        // Build vertex → triangle adjacency
        let mut vertex_triangles: Vec<Vec<u32>> = vec![Vec::new(); vertex_count];
        for t in 0..tri_count {
            let [a, b, c] = mesh.triangle(t);
            vertex_triangles[a as usize].push(t as u32);
            vertex_triangles[b as usize].push(t as u32);
            vertex_triangles[c as usize].push(t as u32);
        }

        // Build edge → triangle map
        // Key: (min_vertex, max_vertex) to canonicalize edge direction
        let mut edge_map: HashMap<(u32, u32), Vec<u32>> = HashMap::new();

        for t in 0..tri_count {
            let [a, b, c] = mesh.triangle(t);
            let tri_edges = [(a, b), (b, c), (c, a)];
            for (v0, v1) in tri_edges {
                let key = if v0 < v1 { (v0, v1) } else { (v1, v0) };
                edge_map.entry(key).or_default().push(t as u32);
            }
        }

        let mut edges: Vec<[u32; 2]> = Vec::with_capacity(edge_map.len());
        let mut edge_triangles: Vec<Vec<u32>> = Vec::with_capacity(edge_map.len());
        let mut interior_edges: Vec<InteriorEdge> = Vec::new();

        for (&(v0, v1), tris) in &edge_map {
            let edge_idx = edges.len();
            edges.push([v0, v1]);
            edge_triangles.push(tris.clone());

            // Interior edge: exactly 2 adjacent triangles
            if tris.len() == 2 {
                let tri_a = tris[0];
                let tri_b = tris[1];

                // Find wing vertices (the vertex in each tri not on the shared edge)
                let wing_a = find_wing_vertex(mesh, tri_a, v0, v1);
                let wing_b = find_wing_vertex(mesh, tri_b, v0, v1);

                interior_edges.push(InteriorEdge {
                    v0,
                    v1,
                    wing_a,
                    wing_b,
                    tri_a,
                    tri_b,
                });
            }
            let _ = edge_idx; // Used for indexing into edges/edge_triangles
        }

        Self {
            vertex_triangles,
            edges,
            edge_triangles,
            interior_edges,
        }
    }

    /// Returns the 1-ring vertex neighborhood of vertex `v`.
    ///
    /// These are all vertices connected to `v` by an edge.
    /// Used for self-collision exclusion zones.
    pub fn one_ring(&self, v: u32, mesh: &TriangleMesh) -> HashSet<u32> {
        let mut neighbors = HashSet::new();
        for &tri in &self.vertex_triangles[v as usize] {
            let [a, b, c] = mesh.triangle(tri as usize);
            if a != v {
                neighbors.insert(a);
            }
            if b != v {
                neighbors.insert(b);
            }
            if c != v {
                neighbors.insert(c);
            }
        }
        neighbors
    }

    /// Returns the number of boundary edges (edges with only 1 adjacent triangle).
    pub fn boundary_edge_count(&self) -> usize {
        self.edge_triangles
            .iter()
            .filter(|tris| tris.len() == 1)
            .count()
    }

    /// Returns true if the mesh is closed (no boundary edges).
    pub fn is_closed(&self) -> bool {
        self.boundary_edge_count() == 0
    }
}

/// Find the vertex in triangle `tri` that is not v0 or v1 (the "wing" vertex).
fn find_wing_vertex(mesh: &TriangleMesh, tri: u32, v0: u32, v1: u32) -> u32 {
    let [a, b, c] = mesh.triangle(tri as usize);
    if a != v0 && a != v1 {
        a
    } else if b != v0 && b != v1 {
        b
    } else {
        c
    }
}

