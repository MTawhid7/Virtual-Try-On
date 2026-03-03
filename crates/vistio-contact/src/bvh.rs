//! AABB-based Bounding Volume Hierarchy for broad phase collision detection.
//!
//! Provides O(N log N) broad phase queries for both cloth-body and
//! self-collision scenarios. Each leaf node wraps a single triangle,
//! inflated by a margin (thickness) for conservative detection.
//!
//! The BVH is rebuilt from scratch each frame using top-down
//! median-split construction.

use vistio_mesh::TriangleMesh;
use vistio_types::VistioResult;

use crate::broad::{BroadPhase, CandidatePair};

/// Axis-Aligned Bounding Box.
#[derive(Debug, Clone, Copy)]
pub struct Aabb {
    pub min_x: f32,
    pub min_y: f32,
    pub min_z: f32,
    pub max_x: f32,
    pub max_y: f32,
    pub max_z: f32,
}

impl Aabb {
    /// Create an AABB from three vertex positions, inflated by a margin.
    #[allow(clippy::too_many_arguments)]
    pub fn from_triangle(
        ax: f32, ay: f32, az: f32,
        bx: f32, by: f32, bz: f32,
        cx: f32, cy: f32, cz: f32,
        margin: f32,
    ) -> Self {
        Self {
            min_x: ax.min(bx).min(cx) - margin,
            min_y: ay.min(by).min(cy) - margin,
            min_z: az.min(bz).min(cz) - margin,
            max_x: ax.max(bx).max(cx) + margin,
            max_y: ay.max(by).max(cy) + margin,
            max_z: az.max(bz).max(cz) + margin,
        }
    }

    /// Create an AABB enclosing two child AABBs.
    pub fn merge(a: &Aabb, b: &Aabb) -> Self {
        Self {
            min_x: a.min_x.min(b.min_x),
            min_y: a.min_y.min(b.min_y),
            min_z: a.min_z.min(b.min_z),
            max_x: a.max_x.max(b.max_x),
            max_y: a.max_y.max(b.max_y),
            max_z: a.max_z.max(b.max_z),
        }
    }

    /// Test if two AABBs overlap.
    pub fn overlaps(&self, other: &Aabb) -> bool {
        self.min_x <= other.max_x && self.max_x >= other.min_x
            && self.min_y <= other.max_y && self.max_y >= other.min_y
            && self.min_z <= other.max_z && self.max_z >= other.min_z
    }

    /// Return the centroid of this AABB.
    pub fn centroid(&self) -> [f32; 3] {
        [
            (self.min_x + self.max_x) * 0.5,
            (self.min_y + self.max_y) * 0.5,
            (self.min_z + self.max_z) * 0.5,
        ]
    }

    /// Return the longest axis (0=X, 1=Y, 2=Z).
    pub fn longest_axis(&self) -> usize {
        let dx = self.max_x - self.min_x;
        let dy = self.max_y - self.min_y;
        let dz = self.max_z - self.min_z;
        if dx >= dy && dx >= dz {
            0
        } else if dy >= dz {
            1
        } else {
            2
        }
    }
}

/// A node in the BVH tree.
#[derive(Debug, Clone)]
pub(crate) enum BvhNode {
    /// Leaf node containing a single triangle index.
    Leaf {
        aabb: Aabb,
        triangle_idx: usize,
    },
    /// Internal node with two children.
    Internal {
        aabb: Aabb,
        left: usize,  // Index into BvhTree::nodes
        right: usize, // Index into BvhTree::nodes
    },
}

impl BvhNode {
    fn aabb(&self) -> &Aabb {
        match self {
            BvhNode::Leaf { aabb, .. } => aabb,
            BvhNode::Internal { aabb, .. } => aabb,
        }
    }
}

/// A complete BVH tree built over triangle primitives.
#[derive(Debug, Clone)]
pub(crate) struct BvhTree {
    pub(crate) nodes: Vec<BvhNode>,
    pub(crate) root: usize,
}

impl BvhTree {
    /// Build a BVH from per-triangle AABBs using top-down median split.
    pub(crate) fn build(leaf_aabbs: &[Aabb]) -> Option<Self> {
        if leaf_aabbs.is_empty() {
            return None;
        }

        let mut nodes = Vec::with_capacity(2 * leaf_aabbs.len());
        let mut indices: Vec<usize> = (0..leaf_aabbs.len()).collect();
        let root = Self::build_recursive(&mut nodes, leaf_aabbs, &mut indices);
        Some(Self { nodes, root })
    }

    fn build_recursive(
        nodes: &mut Vec<BvhNode>,
        leaf_aabbs: &[Aabb],
        indices: &mut [usize],
    ) -> usize {
        if indices.len() == 1 {
            let idx = indices[0];
            let node_idx = nodes.len();
            nodes.push(BvhNode::Leaf {
                aabb: leaf_aabbs[idx],
                triangle_idx: idx,
            });
            return node_idx;
        }

        // Compute encompassing AABB
        let mut bounds = leaf_aabbs[indices[0]];
        for &i in indices.iter().skip(1) {
            bounds = Aabb::merge(&bounds, &leaf_aabbs[i]);
        }

        // Split along longest axis at median
        let axis = bounds.longest_axis();
        indices.sort_by(|&a, &b| {
            let ca = leaf_aabbs[a].centroid()[axis];
            let cb = leaf_aabbs[b].centroid()[axis];
            ca.partial_cmp(&cb).unwrap_or(std::cmp::Ordering::Equal)
        });

        let mid = indices.len() / 2;
        let (left_indices, right_indices) = indices.split_at_mut(mid);

        let left = Self::build_recursive(nodes, leaf_aabbs, left_indices);
        let right = Self::build_recursive(nodes, leaf_aabbs, right_indices);

        // Recompute bounds from children (more precise than accumulated bounds)
        let merged = Aabb::merge(nodes[left].aabb(), nodes[right].aabb());

        let node_idx = nodes.len();
        nodes.push(BvhNode::Internal {
            aabb: merged,
            left,
            right,
        });
        node_idx
    }

    /// Query all pairs of overlapping leaf AABBs within the same tree (self-collision).
    fn query_self_pairs(&self) -> Vec<(usize, usize)> {
        let mut pairs = Vec::new();
        if self.nodes.is_empty() {
            return pairs;
        }
        self.query_self_recursive(self.root, self.root, &mut pairs);
        pairs
    }

    fn query_self_recursive(
        &self,
        node_a: usize,
        node_b: usize,
        pairs: &mut Vec<(usize, usize)>,
    ) {
        // Don't test a node against itself unless it's internal
        if !self.nodes[node_a].aabb().overlaps(self.nodes[node_b].aabb()) {
            return;
        }

        match (&self.nodes[node_a], &self.nodes[node_b]) {
            // Both leaves
            (
                BvhNode::Leaf { triangle_idx: ta, .. },
                BvhNode::Leaf { triangle_idx: tb, .. },
            ) => {
                if ta < tb {
                    pairs.push((*ta, *tb));
                }
            }
            // A is internal, B is leaf
            (BvhNode::Internal { left, right, .. }, BvhNode::Leaf { .. }) => {
                self.query_self_recursive(*left, node_b, pairs);
                self.query_self_recursive(*right, node_b, pairs);
            }
            // A is leaf, B is internal
            (BvhNode::Leaf { .. }, BvhNode::Internal { left, right, .. }) => {
                self.query_self_recursive(node_a, *left, pairs);
                self.query_self_recursive(node_a, *right, pairs);
            }
            // Both internal
            (
                BvhNode::Internal { left: la, right: ra, .. },
                BvhNode::Internal { left: lb, right: rb, .. },
            ) => {
                if node_a == node_b {
                    // Same subtree — recurse into children
                    let la = *la;
                    let ra = *ra;
                    self.query_self_recursive(la, la, pairs);
                    self.query_self_recursive(la, ra, pairs);
                    self.query_self_recursive(ra, ra, pairs);
                } else {
                    let la = *la;
                    let ra = *ra;
                    let lb = *lb;
                    let rb = *rb;
                    self.query_self_recursive(la, lb, pairs);
                    self.query_self_recursive(la, rb, pairs);
                    self.query_self_recursive(ra, lb, pairs);
                    self.query_self_recursive(ra, rb, pairs);
                }
            }
        }
    }

    /// Refit the existing BVH tree with new leaf AABBs.
    /// This is O(N) instead of O(N log N) for full rebuild.
    fn refit(&mut self, leaf_aabbs: &[Aabb]) {
        if self.nodes.is_empty() { return; }
        self.refit_recursive(self.root, leaf_aabbs);
    }

    fn refit_recursive(&mut self, node_idx: usize, leaf_aabbs: &[Aabb]) -> Aabb {
        match self.nodes[node_idx] {
            BvhNode::Leaf { triangle_idx, .. } => {
                let new_aabb = leaf_aabbs[triangle_idx];
                self.nodes[node_idx] = BvhNode::Leaf { aabb: new_aabb, triangle_idx };
                new_aabb
            }
            BvhNode::Internal { left, right, .. } => {
                let left_aabb = self.refit_recursive(left, leaf_aabbs);
                let right_aabb = self.refit_recursive(right, leaf_aabbs);
                let merged = Aabb::merge(&left_aabb, &right_aabb);
                self.nodes[node_idx] = BvhNode::Internal { aabb: merged, left, right };
                merged
            }
        }
    }
}

// ─── BroadPhase Implementation ────────────────────────────────

/// BVH-based broad phase for collision detection.
///
/// Implements the `BroadPhase` trait. Builds a BVH over triangles
/// each frame and queries for overlapping triangle pairs.
pub struct BvhBroadPhase {
    /// Reference mesh topology (triangle indices).
    mesh_indices: Vec<u32>,
    /// Number of triangles.
    tri_count: usize,
    /// Cached BVH tree (rebuilt or refit each update).
    tree: Option<BvhTree>,
    /// Cached candidate pairs from last query.
    pairs: Vec<CandidatePair>,
    /// Track how long since last full O(N log N) rebuild.
    frames_since_rebuild: usize,
}

impl BvhBroadPhase {
    /// Create a new BVH broad phase from a reference mesh.
    pub fn new(mesh: &TriangleMesh) -> Self {
        Self {
            mesh_indices: mesh.indices.clone(),
            tri_count: mesh.triangle_count(),
            tree: None,
            pairs: Vec::new(),
            frames_since_rebuild: 0,
        }
    }
}

impl BroadPhase for BvhBroadPhase {
    fn update(&mut self, pos_x: &[f32], pos_y: &[f32], pos_z: &[f32], thickness: f32) -> VistioResult<()> {
        // Build per-triangle AABBs
        let mut leaf_aabbs = Vec::with_capacity(self.tri_count);
        for t in 0..self.tri_count {
            let base = t * 3;
            let i0 = self.mesh_indices[base] as usize;
            let i1 = self.mesh_indices[base + 1] as usize;
            let i2 = self.mesh_indices[base + 2] as usize;

            leaf_aabbs.push(Aabb::from_triangle(
                pos_x[i0], pos_y[i0], pos_z[i0],
                pos_x[i1], pos_y[i1], pos_z[i1],
                pos_x[i2], pos_y[i2], pos_z[i2],
                thickness,
            ));
        }

        self.frames_since_rebuild += 1;
        if self.tree.is_none() || self.frames_since_rebuild >= 10 {
            self.tree = BvhTree::build(&leaf_aabbs);
            self.frames_since_rebuild = 0;
        } else {
            self.tree.as_mut().unwrap().refit(&leaf_aabbs);
        }

        // Query self-collision pairs (triangle vs triangle)
        self.pairs.clear();
        if let Some(ref tree) = self.tree {
            let tri_pairs = tree.query_self_pairs();
            for (ta, tb) in tri_pairs {
                // Emit candidate pairs as vertex pairs from the two triangles.
                // For each pair of triangles (ta, tb), we emit candidate pairs
                // of their vertices for narrow-phase testing.
                let base_a = ta * 3;
                let base_b = tb * 3;
                let verts_a = [
                    self.mesh_indices[base_a],
                    self.mesh_indices[base_a + 1],
                    self.mesh_indices[base_a + 2],
                ];
                let verts_b = [
                    self.mesh_indices[base_b],
                    self.mesh_indices[base_b + 1],
                    self.mesh_indices[base_b + 2],
                ];

                // Check that triangles don't share vertices
                let shares_vertex = verts_a.iter().any(|va| verts_b.contains(va));
                if shares_vertex {
                    continue;
                }

                // Emit vertex pairs for narrow-phase testing
                for &va in &verts_a {
                    for &vb in &verts_b {
                        if va != vb {
                            self.pairs.push(CandidatePair {
                                a: va,
                                b: vb,
                                is_self: true,
                            });
                        }
                    }
                }
            }
        }

        Ok(())
    }

    fn query_pairs(&self) -> Vec<CandidatePair> {
        self.pairs.clone()
    }

    fn query_triangle_pairs(&self) -> Vec<crate::broad::CandidateTrianglePair> {
        let mut result = Vec::new();
        if let Some(ref tree) = self.tree {
            let tri_pairs = tree.query_self_pairs();
            for (ta, tb) in tri_pairs {
                result.push(crate::broad::CandidateTrianglePair {
                    ta: ta as u32,
                    tb: tb as u32,
                    is_self: true,
                });
            }
        }
        result
    }

    fn name(&self) -> &str {
        "bvh_broad_phase"
    }
}
