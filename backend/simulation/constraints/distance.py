"""
Distance constraint — XPBD edge-length preservation.

Each edge (i, j) has a rest length L₀. The constraint function is:
    C(x) = |x_i - x_j| - L₀

The XPBD projection computes:
    Δλ = -(C + α̃·λ) / (w_i + w_j + α̃)
    Δx_i = +w_i · n̂ · Δλ
    Δx_j = -w_j · n̂ · Δλ

where n̂ is the unit vector from x_j to x_i, and α̃ = α/dt².
"""

import taichi as ti
import numpy as np
from numpy.typing import NDArray


@ti.data_oriented
class DistanceConstraints:
    """
    Stores and projects distance constraints for all edges in the mesh.

    Pre-allocated Taichi fields hold edge pairs, rest lengths, and
    Lagrange multipliers. The projection kernel runs on all edges in parallel.
    """

    def __init__(self, max_edges: int) -> None:
        self.max_edges = max_edges
        self.n_edges: int = 0

        # Edge topology: vertex index pairs
        self.edge_v0 = ti.field(dtype=ti.i32, shape=max_edges)
        self.edge_v1 = ti.field(dtype=ti.i32, shape=max_edges)

        # Rest lengths (computed at initialization)
        self.rest_length = ti.field(dtype=ti.f32, shape=max_edges)

        # XPBD Lagrange multipliers (reset each substep)
        self.lambdas = ti.field(dtype=ti.f32, shape=max_edges)

    def initialize(
        self,
        edges: NDArray[np.int32],
        positions: NDArray[np.float32],
    ) -> None:
        """
        Build distance constraints from edge list and initial positions.

        Args:
            edges: (E, 2) array of vertex index pairs.
            positions: (N, 3) initial vertex positions for computing rest lengths.
        """
        n = edges.shape[0]
        if n > self.max_edges:
            raise ValueError(
                f"Edge count {n} exceeds max_edges {self.max_edges}."
            )

        self.n_edges = n

        # Load edge indices
        v0_padded = np.zeros(self.max_edges, dtype=np.int32)
        v1_padded = np.zeros(self.max_edges, dtype=np.int32)
        v0_padded[:n] = edges[:, 0]
        v1_padded[:n] = edges[:, 1]
        self.edge_v0.from_numpy(v0_padded)
        self.edge_v1.from_numpy(v1_padded)

        # Compute rest lengths from initial positions
        rest = np.zeros(self.max_edges, dtype=np.float32)
        p0 = positions[edges[:, 0]]  # (E, 3)
        p1 = positions[edges[:, 1]]  # (E, 3)
        rest[:n] = np.linalg.norm(p1 - p0, axis=1).astype(np.float32)
        self.rest_length.from_numpy(rest)

        # Zero Lagrange multipliers
        self.lambdas.from_numpy(np.zeros(self.max_edges, dtype=np.float32))

    def reset_lambdas(self) -> None:
        """Reset Lagrange multipliers at the start of each substep."""
        self.lambdas.fill(0.0)

    @ti.kernel
    def project(
        self,
        positions: ti.template(),
        inv_mass: ti.template(),
        n_edges: ti.i32,
        compliance: ti.f32,
        dt: ti.f32,
    ):
        """
        Project all distance constraints using XPBD.

        Standard XPBD update:
            α̃ = compliance / dt²
            C = |x_i - x_j| - L₀
            Δλ = -(C + α̃·λ) / (w_i + w_j + α̃)
            x_i += w_i · n̂ · Δλ
            x_j -= w_j · n̂ · Δλ
        """
        alpha_tilde = compliance / (dt * dt)

        for e in range(n_edges):
            i = self.edge_v0[e]
            j = self.edge_v1[e]

            wi = inv_mass[i]
            wj = inv_mass[j]

            # Skip if both vertices are pinned
            if wi + wj < 1e-12:
                continue

            diff = positions[i] - positions[j]
            dist = diff.norm()

            # Avoid division by zero for degenerate edges
            if dist < 1e-10:
                continue

            n_hat = diff / dist
            C = dist - self.rest_length[e]

            # XPBD Lagrange multiplier update
            denom = wi + wj + alpha_tilde
            delta_lambda = -(C + alpha_tilde * self.lambdas[e]) / denom
            self.lambdas[e] += delta_lambda

            # Apply corrections
            correction = n_hat * delta_lambda
            positions[i] += wi * correction
            positions[j] -= wj * correction
