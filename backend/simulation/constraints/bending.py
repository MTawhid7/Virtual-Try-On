"""
Bending constraint — XPBD dihedral angle preservation (isometric bending).

For each internal edge shared by two triangles (v0, v1, v2, v3), we compute
the dihedral angle θ between the two triangle normals. The constraint is:
    C(x) = θ - θ₀

where θ₀ is the rest angle (the angle at initialization — typically 0 rad
for a flat grid mesh, since e × (p2 - p0) and e × (p3 - p0) point the same
direction when the mesh is planar).

Gradients are computed analytically using the cotangent-weighted formula
(Bergou 2006 / Grinspun 2003). For hinge (p0, p1, p2, p3) with shared edge
e = p1 - p0 and unscaled triangle normals n1 = e × (p2 - p0),
n2 = e × (p3 - p0):

    ∂θ/∂p2 =  |e| / |n1|² * n1
    ∂θ/∂p3 = -|e| / |n2|² * n2
    ∂θ/∂p0 = +dot(p2-p1, e)/(|e||n1|²)*n1  +  dot(p3-p1, e)/(|e||n2|²)*n2
    ∂θ/∂p1 = -dot(p2-p0, e)/(|e||n1|²)*n1  -  dot(p3-p0, e)/(|e||n2|²)*n2

This replaces the previous finite-difference approximation (24 extra angle
evaluations per hinge per iteration) with O(1) cross products + dot products,
eliminating O(eps²) gradient noise and ~8× reducing per-hinge compute cost.

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

    @ti.kernel
    def apply_bend_damping(
        self,
        positions: ti.template(),
        velocities: ti.template(),
        inv_mass: ti.template(),
        n_hinges: ti.i32,
        damping_coeff: ti.f32,
    ):
        """
        Damp the angular velocity component of each hinge (Rayleigh-style).

        Computes the analytical bending gradient ∇θ for each hinge (same formula
        as the project kernel — Track A), then damps the constraint-velocity
        component C_dot = ∇θ · v along the gradient direction.

        Applied once per substep AFTER integrator.update(), before the next predict.
        Requires Track A analytical gradients (correct sign convention in place).
        """
        for h in range(n_hinges):
            i0 = self.hinge_v0[h]
            i1 = self.hinge_v1[h]
            i2 = self.hinge_v2[h]
            i3 = self.hinge_v3[h]

            p0 = positions[i0]
            p1 = positions[i1]
            p2 = positions[i2]
            p3 = positions[i3]

            e  = p1 - p0
            n1 = e.cross(p2 - p0)
            n2 = e.cross(p3 - p0)

            len_e  = e.norm()
            len_n1 = n1.norm()
            len_n2 = n2.norm()

            if len_e < 1e-8 or len_n1 < 1e-8 or len_n2 < 1e-8:
                continue

            inv_n1sq  = ti.f32(1.0) / (len_n1 * len_n1)
            inv_n2sq  = ti.f32(1.0) / (len_n2 * len_n2)
            inv_len_e = ti.f32(1.0) / len_e

            # Analytical gradient (same convention as project kernel)
            grad2 = -len_e * inv_n1sq * n1
            grad3 =  len_e * inv_n2sq * n2
            grad0 = (-(p2 - p1).dot(e) * inv_n1sq * inv_len_e) * n1 \
                  + ( (p3 - p1).dot(e) * inv_n2sq * inv_len_e) * n2
            grad1 = ( (p2 - p0).dot(e) * inv_n1sq * inv_len_e) * n1 \
                  + (-(p3 - p0).dot(e) * inv_n2sq * inv_len_e) * n2

            w0 = inv_mass[i0]
            w1 = inv_mass[i1]
            w2 = inv_mass[i2]
            w3 = inv_mass[i3]

            # C_dot = ∇θ · v  (rate of dihedral angle change)
            C_dot = (
                grad0.dot(velocities[i0])
                + grad1.dot(velocities[i1])
                + grad2.dot(velocities[i2])
                + grad3.dot(velocities[i3])
            )

            w_grad_sq = (
                w0 * grad0.norm_sqr()
                + w1 * grad1.norm_sqr()
                + w2 * grad2.norm_sqr()
                + w3 * grad3.norm_sqr()
            )

            if w_grad_sq < 1e-12:
                continue

            # Damping impulse: reduce the angular velocity by damping_coeff fraction
            delta = -damping_coeff * C_dot / w_grad_sq
            velocities[i0] += w0 * grad0 * delta
            velocities[i1] += w1 * grad1 * delta
            velocities[i2] += w2 * grad2 * delta
            velocities[i3] += w3 * grad3 * delta

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
        Project all bending constraints using XPBD with analytical gradients.

        Uses the closed-form cotangent-weighted gradient of the dihedral angle
        (Bergou 2006 / Grinspun 2003). For hinge (p0, p1, p2, p3):
            e  = p1 - p0  (shared edge)
            n1 = e × (p2 - p0)  (unscaled normal of triangle 1)
            n2 = e × (p3 - p0)  (unscaled normal of triangle 2)

            ∂θ/∂p2 =  |e| / |n1|² * n1
            ∂θ/∂p3 = -|e| / |n2|² * n2
            ∂θ/∂p0 = +dot(p2-p1, e)/(|e||n1|²)*n1 + dot(p3-p1, e)/(|e||n2|²)*n2
            ∂θ/∂p1 = -dot(p2-p0, e)/(|e||n1|²)*n1 - dot(p3-p0, e)/(|e||n2|²)*n2

        XPBD update:
            α̃ = compliance / dt²
            C = θ - θ₀
            Δλ = -(C + α̃·λ) / (Σ wᵢ·|∇Cᵢ|² + α̃)
        """
        alpha_tilde = compliance / (dt * dt)

        for h in range(n_hinges):
            i0 = self.hinge_v0[h]
            i1 = self.hinge_v1[h]
            i2 = self.hinge_v2[h]
            i3 = self.hinge_v3[h]

            p0 = positions[i0]
            p1 = positions[i1]
            p2 = positions[i2]
            p3 = positions[i3]

            e  = p1 - p0                   # shared edge (unscaled)
            n1 = e.cross(p2 - p0)          # unscaled normal of triangle 1
            n2 = e.cross(p3 - p0)          # unscaled normal of triangle 2

            len_e  = e.norm()
            len_n1 = n1.norm()
            len_n2 = n2.norm()

            if len_e < 1e-8 or len_n1 < 1e-8 or len_n2 < 1e-8:
                continue

            # Current dihedral angle (atan2 form for full [-π, π] range)
            n1_hat = n1 / len_n1
            n2_hat = n2 / len_n2
            e_hat  = e  / len_e
            cos_a  = ti.math.clamp(n1_hat.dot(n2_hat), -1.0, 1.0)
            sin_a  = n1_hat.cross(n2_hat).dot(e_hat)
            theta  = ti.atan2(sin_a, cos_a)

            C = theta - self.rest_angle[h]
            # Wrap to [-π, π]
            while C >  ti.math.pi: C -= 2.0 * ti.math.pi
            while C < -ti.math.pi: C += 2.0 * ti.math.pi

            if ti.abs(C) < 1e-8:
                continue

            # Analytical cotangent-weighted gradients (Bergou 2006 / Grinspun 2003).
            # Our convention: n1 = e × (p2-p0), which points OPPOSITE to the paper's
            # outward normal (p2-p0) × e.  All n1-related terms are therefore negated
            # relative to the published formula; n2 = e × (p3-p0) is also negated.
            # This produces grad2 = -|e|/|n1|² n1, grad3 = +|e|/|n2|² n2 and the
            # corresponding edge-vertex corrections below (verified against FD).
            inv_n1sq = ti.f32(1.0) / (len_n1 * len_n1)
            inv_n2sq = ti.f32(1.0) / (len_n2 * len_n2)
            inv_len_e = ti.f32(1.0) / len_e

            grad2 = -len_e * inv_n1sq * n1
            grad3 =  len_e * inv_n2sq * n2
            grad0 = (-(p2 - p1).dot(e) * inv_n1sq * inv_len_e) * n1 \
                  + ( (p3 - p1).dot(e) * inv_n2sq * inv_len_e) * n2
            grad1 = ( (p2 - p0).dot(e) * inv_n1sq * inv_len_e) * n1 \
                  + (-(p3 - p0).dot(e) * inv_n2sq * inv_len_e) * n2

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

            # XPBD Lagrange multiplier update
            denom = w_grad_sq + alpha_tilde
            delta_lambda = -(C + alpha_tilde * self.lambdas[h]) / denom
            self.lambdas[h] += delta_lambda

            # Apply position corrections
            positions[i0] += w0 * grad0 * delta_lambda
            positions[i1] += w1 * grad1 * delta_lambda
            positions[i2] += w2 * grad2 * delta_lambda
            positions[i3] += w3 * grad3 * delta_lambda
