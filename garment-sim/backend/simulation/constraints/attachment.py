"""
Attachment constraints — soft XPBD positional pins for garment vertices.

Pulls selected vertices toward target world-space positions with configurable
compliance.  Used during the sew phase ONLY to prevent panels from tunneling
through the body.

Each attachment is a single-particle positional constraint with 3 independent
scalar DOFs (one per axis):

    C_k(x) = x_i[k] - target[k]     (k = 0, 1, 2)

XPBD update per component:
    α̃  = compliance / dt²
    Δλ_k = -(C_k + α̃·λ_k) / (w_i + α̃)
    x_i[k] += w_i · Δλ_k

With compliance = 1e-4 the constraint is soft enough that seam-closure forces
can slide panels laterally while still preventing Z-axis tunneling through the
body during rapid sew-phase movement.

IMPORTANT: Do NOT add `from __future__ import annotations` to this file.
Taichi JIT cannot resolve string annotations on .template() kernel parameters.
"""

import taichi as ti
import numpy as np
from numpy.typing import NDArray


@ti.data_oriented
class AttachmentConstraints:
    """
    Soft XPBD positional pin constraints for garment vertices.

    Pre-allocated Taichi fields hold vertex indices, target positions, and
    per-component Lagrange multipliers.  The projection kernel runs on all
    attachments in parallel.

    Typical usage (sew phase only):
        ac = AttachmentConstraints(max_attachments=500)
        ac.initialize(vertex_indices, target_positions)

        # Inside substep loop (sew phase):
        ac.reset_lambdas()
        ac.project(state.positions, state.inv_mass, ac.n_attachments, 1e-4, dt)
    """

    def __init__(self, max_attachments: int = 2000) -> None:
        self.max_attachments = max_attachments
        self.n_attachments: int = 0

        # Topology: one vertex index per attachment
        self.attach_vertex = ti.field(dtype=ti.i32, shape=max_attachments)

        # Target world-space positions
        self.target_pos = ti.Vector.field(3, dtype=ti.f32, shape=max_attachments)

        # XPBD Lagrange multipliers — one 3-vector per attachment (reset each substep)
        self.lambdas = ti.Vector.field(3, dtype=ti.f32, shape=max_attachments)

    def initialize(
        self,
        vertex_indices: NDArray,    # (A,) int32 — global vertex indices to pin
        target_positions: NDArray,  # (A, 3) float32 — world-space pin targets
    ) -> None:
        """
        Load attachment definitions.

        Args:
            vertex_indices:  (A,) int32 — global vertex index for each pin.
            target_positions: (A, 3) float32 — target world-space position.
        """
        n = len(vertex_indices)
        if n > self.max_attachments:
            raise ValueError(
                f"Attachment count {n} exceeds max_attachments {self.max_attachments}."
            )
        self.n_attachments = n

        v_pad = np.zeros(self.max_attachments, dtype=np.int32)
        v_pad[:n] = vertex_indices
        self.attach_vertex.from_numpy(v_pad)

        p_pad = np.zeros((self.max_attachments, 3), dtype=np.float32)
        p_pad[:n] = target_positions
        self.target_pos.from_numpy(p_pad)

        self.lambdas.fill(0.0)

    def reset_lambdas(self) -> None:
        """Reset Lagrange multipliers at the start of each substep."""
        self.lambdas.fill(0.0)

    @ti.kernel
    def project(
        self,
        positions: ti.template(),
        inv_mass: ti.template(),
        n_attachments: ti.i32,
        compliance: ti.f32,
        dt: ti.f32,
    ):
        """
        Project attachment constraints: pull pinned vertices toward their targets.

        For each attachment a:
          i       = attach_vertex[a]
          C       = positions[i] - target_pos[a]   (3-vector)
          α̃       = compliance / dt²
          Δλ      = -(C + α̃·λ) / (w_i + α̃)
          λ      += Δλ
          pos[i] += w_i · Δλ

        Pinned particles (inv_mass == 0) are skipped.
        """
        alpha_tilde = compliance / (dt * dt)

        for a in range(n_attachments):
            i = self.attach_vertex[a]
            wi = inv_mass[i]

            # Skip pinned particles
            if wi < 1e-12:
                continue

            C = positions[i] - self.target_pos[a]

            denom = wi + alpha_tilde
            d_lambda = -(C + alpha_tilde * self.lambdas[a]) / denom
            self.lambdas[a] += d_lambda

            positions[i] += wi * d_lambda
