//! Topology-based collision exclusion.
//!
//! Prevents false positives from adjacent vertices by excluding
//! vertices within an n-ring neighborhood from collision detection.

use std::collections::HashSet;

use vistio_mesh::TriangleMesh;
use vistio_mesh::topology::Topology;

/// N-ring topology exclusion for self-collision filtering.
///
/// Pre-computes for each vertex the set of vertices within `depth`
/// topological rings. Any pair where both vertices are in each other's
/// exclusion set is skipped during collision detection.
///
/// Typical depth: 2 (excludes immediate neighbors and their neighbors).
pub struct TopologyExclusion {
    /// For each vertex, the set of excluded vertex indices.
    excluded: Vec<HashSet<u32>>,
}

impl TopologyExclusion {
    /// Build exclusion zones with `depth`-ring neighborhood.
    ///
    /// Uses iterative BFS through the vertex-triangle adjacency.
    pub fn new(mesh: &TriangleMesh, topology: &Topology, depth: usize) -> Self {
        let n = mesh.vertex_count();
        let mut excluded: Vec<HashSet<u32>> = vec![HashSet::new(); n];

        for (v, current_exclusion) in excluded.iter_mut().enumerate() {
            let mut current: HashSet<u32> = HashSet::new();
            current.insert(v as u32);

            for _ in 0..depth {
                let mut next = current.clone();
                for &vertex in &current {
                    let ring = topology.one_ring(vertex, mesh);
                    next.extend(ring);
                }
                current = next;
            }

            *current_exclusion = current;
        }

        Self { excluded }
    }

    /// Returns true if vertices i and j should be excluded from collision testing.
    #[inline]
    pub fn should_exclude(&self, i: usize, j: usize) -> bool {
        self.excluded[i].contains(&(j as u32))
    }

    /// Returns the exclusion set for a given vertex.
    pub fn exclusion_set(&self, v: usize) -> &HashSet<u32> {
        &self.excluded[v]
    }

    /// Total number of vertices.
    pub fn vertex_count(&self) -> usize {
        self.excluded.len()
    }
}
