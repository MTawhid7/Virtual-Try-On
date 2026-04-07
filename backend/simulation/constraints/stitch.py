"""
Stitch constraint — zero rest-length distance constraints that pull
corresponding seam vertices on adjacent garment panels together.

Each stitch is a pair of global vertex indices (i, j) across two panels.
The constraint function is:
    C(x) = |x_i - x_j|   (zero rest length — pulls to coincidence)

XPBD projection:
    Δλ = -(C + α̃·λ) / (w_i + w_j + α̃)
    Δx_i = +w_i · n̂ · Δλ
    Δx_j = -w_j · n̂ · Δλ

where n̂ = (x_i - x_j) / |x_i - x_j|.

Compliance α controls how aggressively stitches close:
  - 1e-6: stiff — closes a 0.24m gap in ~60–80 frames
  - 1e-4: soft — useful for debugging or gradual closure

IMPORTANT: Do NOT add `from __future__ import annotations` to this file.
Taichi JIT cannot resolve string annotations on .template() kernel parameters.
"""

import taichi as ti
import numpy as np
from numpy.typing import NDArray


@ti.data_oriented
class StitchConstraints:
    """
    Zero rest-length distance constraints connecting paired seam vertices.

    Pre-allocated Taichi fields hold the vertex index pairs and Lagrange
    multipliers. The projection kernel runs on all stitch pairs in parallel.
    """

    def __init__(self, max_stitches: int = 10_000) -> None:
        self.max_stitches = max_stitches
        self.n_stitches: int = 0

        # Stitch topology: global vertex index pairs
        self.stitch_v0 = ti.field(dtype=ti.i32, shape=max_stitches)
        self.stitch_v1 = ti.field(dtype=ti.i32, shape=max_stitches)

        # XPBD Lagrange multipliers (reset each substep)
        self.lambdas = ti.field(dtype=ti.f32, shape=max_stitches)

    def initialize(self, stitch_pairs: NDArray[np.int32]) -> None:
        """
        Load stitch vertex pairs.

        Args:
            stitch_pairs: (S, 2) array of global vertex index pairs.
                          stitch_pairs[k] = [i, j] means vertices i and j
                          are stitched together (zero rest-length).
        """
        n = stitch_pairs.shape[0]
        if n > self.max_stitches:
            raise ValueError(
                f"Stitch count {n} exceeds max_stitches {self.max_stitches}."
            )

        self.n_stitches = n

        v0_padded = np.zeros(self.max_stitches, dtype=np.int32)
        v1_padded = np.zeros(self.max_stitches, dtype=np.int32)
        v0_padded[:n] = stitch_pairs[:, 0]
        v1_padded[:n] = stitch_pairs[:, 1]
        self.stitch_v0.from_numpy(v0_padded)
        self.stitch_v1.from_numpy(v1_padded)

        self.lambdas.from_numpy(np.zeros(self.max_stitches, dtype=np.float32))

    def reset_lambdas(self) -> None:
        """Reset Lagrange multipliers at the start of each substep."""
        self.lambdas.fill(0.0)

    @ti.kernel
    def project(
        self,
        positions: ti.template(),
        inv_mass: ti.template(),
        n_stitches: ti.i32,
        compliance: ti.f32,
        dt: ti.f32,
    ):
        """
        Project stitch constraints: pull paired vertices toward each other.

        For each stitch pair (i, j):
          - C = |x_i - x_j|  (zero rest length)
          - if C < 1e-8: skip (vertices already coincident, no direction)
          - XPBD: Δλ = -(C + α̃·λ) / (w_i + w_j + α̃)
          - Δx_i = +w_i · n̂ · Δλ
          - Δx_j = -w_j · n̂ · Δλ
        """
        alpha_tilde = compliance / (dt * dt)

        for k in range(n_stitches):
            i = self.stitch_v0[k]
            j = self.stitch_v1[k]

            wi = inv_mass[i]
            wj = inv_mass[j]

            # Skip if both particles are pinned
            w_sum = wi + wj
            if w_sum < 1e-8:
                continue

            delta = positions[i] - positions[j]
            dist = delta.norm()

            # Skip if already coincident (avoids division by zero)
            if dist < 1e-8:
                continue

            n_hat = delta / dist

            # C = dist (zero rest length)
            C = dist

            # XPBD update
            d_lambda = -(C + alpha_tilde * self.lambdas[k]) / (w_sum + alpha_tilde)
            self.lambdas[k] += d_lambda

            positions[i] += wi * n_hat * d_lambda
            positions[j] -= wj * n_hat * d_lambda
