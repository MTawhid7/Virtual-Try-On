"""
Bending constraint — XPBD dihedral angle preservation (isometric bending).

For each internal edge shared by two triangles (v0, v1, v2, v3), we compute
the dihedral angle θ between the two triangle normals. The constraint is:
    C(x) = θ - θ₀

where θ₀ is the rest angle (the angle at initialization — typically flat = π
for a grid mesh).

The gradient computation uses the cotangent-based formulation for efficiency:
each of the 4 vertices gets a gradient contribution based on the triangle
normals and their areas.

Reference: Bergou et al. "A Quadratic Bending Model for Inextensible Surfaces"
and the XPBD formulation from Müller et al. 2020.
"""

import taichi as ti
import numpy as np
from numpy.typing import NDArray


def find_adjacent_triangle_pairs(
    faces: NDArray[np.int32],
) -> NDArray[np.int32]:
    """
    Find all pairs of adjacent triangles sharing an edge.

    Returns an (M, 4) array where each row is [v0, v1, v2, v3]:
        - v0, v1: the shared edge vertices
        - v2: the opposite vertex in the first triangle
        - v3: the opposite vertex in the second triangle

    This is the standard "hinge" data structure for dihedral bending.
    """
    # Build edge → triangle map
    edge_to_tris: dict[tuple[int, int], list[tuple[int, int]]] = {}

    for fi, face in enumerate(faces):
        for k in range(3):
            a, b = int(face[k]), int(face[(k + 1) % 3])
            edge = (min(a, b), max(a, b))
            # Store (triangle_index, opposite_vertex)
            opposite = int(face[(k + 2) % 3])
            if edge not in edge_to_tris:
                edge_to_tris[edge] = []
            edge_to_tris[edge].append((fi, opposite))

    # Find shared edges (exactly 2 triangles)
    hinges: list[tuple[int, int, int, int]] = []
    for (a, b), tri_list in edge_to_tris.items():
        if len(tri_list) == 2:
            _, opp0 = tri_list[0]
            _, opp1 = tri_list[1]
            hinges.append((a, b, opp0, opp1))

    if not hinges:
        return np.zeros((0, 4), dtype=np.int32)

    return np.array(hinges, dtype=np.int32)


@ti.data_oriented
class BendingConstraints:
    """
    Stores and projects bending (dihedral angle) constraints.

    Uses the discrete dihedral angle between adjacent triangle pairs.
    Isometric bending: the rest angle is captured from the initial configuration.
    """

    def __init__(self, max_hinges: int) -> None:
        self.max_hinges = max_hinges
        self.n_hinges: int = 0

        # Hinge topology: [shared_v0, shared_v1, opposite_v2, opposite_v3]
        self.hinge_v0 = ti.field(dtype=ti.i32, shape=max_hinges)  # shared edge
        self.hinge_v1 = ti.field(dtype=ti.i32, shape=max_hinges)  # shared edge
        self.hinge_v2 = ti.field(dtype=ti.i32, shape=max_hinges)  # opposite in tri 1
        self.hinge_v3 = ti.field(dtype=ti.i32, shape=max_hinges)  # opposite in tri 2

        # Rest dihedral angle
        self.rest_angle = ti.field(dtype=ti.f32, shape=max_hinges)

        # XPBD Lagrange multipliers
        self.lambdas = ti.field(dtype=ti.f32, shape=max_hinges)

    def initialize(
        self,
        faces: NDArray[np.int32],
        positions: NDArray[np.float32],
    ) -> None:
        """
        Build bending constraints from mesh faces and initial positions.

        Finds all internal edges (shared by 2 triangles), computes the
        initial dihedral angle as the rest angle.
        """
        hinges = find_adjacent_triangle_pairs(faces)
        n = hinges.shape[0]

        if n > self.max_hinges:
            raise ValueError(
                f"Hinge count {n} exceeds max_hinges {self.max_hinges}."
            )

        self.n_hinges = n

        if n == 0:
            return

        # Load hinge indices
        pad = lambda arr: np.pad(arr, (0, self.max_hinges - n))
        self.hinge_v0.from_numpy(pad(hinges[:, 0].astype(np.int32)))
        self.hinge_v1.from_numpy(pad(hinges[:, 1].astype(np.int32)))
        self.hinge_v2.from_numpy(pad(hinges[:, 2].astype(np.int32)))
        self.hinge_v3.from_numpy(pad(hinges[:, 3].astype(np.int32)))

        # Compute rest angles from initial positions
        rest_angles = np.zeros(self.max_hinges, dtype=np.float32)
        for h in range(n):
            v0, v1, v2, v3 = hinges[h]
            rest_angles[h] = self._compute_dihedral_angle_numpy(
                positions[v0], positions[v1], positions[v2], positions[v3]
            )
        self.rest_angle.from_numpy(rest_angles)

        # Zero Lagrange multipliers
        self.lambdas.from_numpy(np.zeros(self.max_hinges, dtype=np.float32))

    @staticmethod
    def _compute_dihedral_angle_numpy(
        p0: NDArray, p1: NDArray, p2: NDArray, p3: NDArray
    ) -> float:
        """Compute dihedral angle between two triangles sharing edge (p0, p1)."""
        edge = p1 - p0
        n1 = np.cross(edge, p2 - p0)
        n2 = np.cross(edge, p3 - p0)

        n1_len = np.linalg.norm(n1)
        n2_len = np.linalg.norm(n2)

        if n1_len < 1e-10 or n2_len < 1e-10:
            return 0.0

        n1 = n1 / n1_len
        n2 = n2 / n2_len

        cos_angle = float(np.clip(np.dot(n1, n2), -1.0, 1.0))
        sin_angle = float(np.dot(np.cross(n1, n2), edge / np.linalg.norm(edge)))

        return float(np.arctan2(sin_angle, cos_angle))

    def reset_lambdas(self) -> None:
        """Reset Lagrange multipliers at the start of each substep."""
        self.lambdas.fill(0.0)

    @ti.func
    def _dihedral_angle(
        self,
        p0: ti.math.vec3,
        p1: ti.math.vec3,
        p2: ti.math.vec3,
        p3: ti.math.vec3,
    ) -> ti.f32:
        """Compute dihedral angle between two triangles sharing edge (p0, p1)."""
        edge = p1 - p0
        n1 = (edge).cross(p2 - p0)
        n2 = (edge).cross(p3 - p0)

        n1_len = n1.norm()
        n2_len = n2.norm()

        angle = ti.f32(0.0)
        if n1_len > 1e-10 and n2_len > 1e-10:
            n1_hat = n1 / n1_len
            n2_hat = n2 / n2_len

            cos_a = ti.math.clamp(n1_hat.dot(n2_hat), -1.0, 1.0)
            edge_hat = edge / ti.max(edge.norm(), 1e-10)
            sin_a = n1_hat.cross(n2_hat).dot(edge_hat)

            angle = ti.atan2(sin_a, cos_a)

        return angle

    @ti.kernel
    def project(
        self,
        positions: ti.template(),
        inv_mass: ti.template(),
        n_hinges: ti.i32,
        compliance: ti.f32,
        dt: ti.f32,
    ):
        """
        Project all bending constraints using XPBD.

        Uses a finite-difference gradient approximation for the dihedral angle
        with respect to the 4 hinge vertices. This is simpler and more robust
        than the analytical gradient, at the cost of 4 extra angle evaluations
        per hinge.

        XPBD update:
            α̃ = compliance / dt²
            C = θ - θ₀
            Δλ = -(C + α̃·λ) / (Σ wᵢ·|∇Cᵢ|² + α̃)
        """
        alpha_tilde = compliance / (dt * dt)
        eps = ti.f32(1e-4)  # Finite difference step

        for h in range(n_hinges):
            i0 = self.hinge_v0[h]
            i1 = self.hinge_v1[h]
            i2 = self.hinge_v2[h]
            i3 = self.hinge_v3[h]

            p0 = positions[i0]
            p1 = positions[i1]
            p2 = positions[i2]
            p3 = positions[i3]

            # Current dihedral angle
            theta = self._dihedral_angle(p0, p1, p2, p3)
            C = theta - self.rest_angle[h]

            # Wrap angle difference to [-π, π]
            while C > ti.math.pi:
                C -= 2.0 * ti.math.pi
            while C < -ti.math.pi:
                C += 2.0 * ti.math.pi

            # Skip small constraint violations
            if ti.abs(C) < 1e-8:
                continue

            # Compute gradients via finite differences for each vertex
            # Gradient for vertex i: ∇C_i ≈ (θ(x_i + ε·e_k) - θ(x_i - ε·e_k)) / (2ε)
            grad0 = ti.math.vec3(0.0)
            grad1 = ti.math.vec3(0.0)
            grad2 = ti.math.vec3(0.0)
            grad3 = ti.math.vec3(0.0)

            for axis in ti.static(range(3)):
                delta = ti.math.vec3(0.0)
                delta[axis] = eps

                # Gradient for p0
                t_plus = self._dihedral_angle(p0 + delta, p1, p2, p3)
                t_minus = self._dihedral_angle(p0 - delta, p1, p2, p3)
                grad0[axis] = (t_plus - t_minus) / (2.0 * eps)

                # Gradient for p1
                t_plus = self._dihedral_angle(p0, p1 + delta, p2, p3)
                t_minus = self._dihedral_angle(p0, p1 - delta, p2, p3)
                grad1[axis] = (t_plus - t_minus) / (2.0 * eps)

                # Gradient for p2
                t_plus = self._dihedral_angle(p0, p1, p2 + delta, p3)
                t_minus = self._dihedral_angle(p0, p1, p2 - delta, p3)
                grad2[axis] = (t_plus - t_minus) / (2.0 * eps)

                # Gradient for p3
                t_plus = self._dihedral_angle(p0, p1, p2, p3 + delta)
                t_minus = self._dihedral_angle(p0, p1, p2, p3 - delta)
                grad3[axis] = (t_plus - t_minus) / (2.0 * eps)

            # Weighted gradient norm squared: Σ w_i * |∇C_i|²
            w0 = inv_mass[i0]
            w1 = inv_mass[i1]
            w2 = inv_mass[i2]
            w3 = inv_mass[i3]

            w_grad_sq = (
                w0 * grad0.norm_sqr()
                + w1 * grad1.norm_sqr()
                + w2 * grad2.norm_sqr()
                + w3 * grad3.norm_sqr()
            )

            if w_grad_sq < 1e-12:
                continue

            # XPBD update
            denom = w_grad_sq + alpha_tilde
            delta_lambda = -(C + alpha_tilde * self.lambdas[h]) / denom
            self.lambdas[h] += delta_lambda

            # Apply corrections
            positions[i0] += w0 * grad0 * delta_lambda
            positions[i1] += w1 * grad1 * delta_lambda
            positions[i2] += w2 * grad2 * delta_lambda
            positions[i3] += w3 * grad3 * delta_lambda
