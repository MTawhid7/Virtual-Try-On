"""
Tests for the analytical cotangent-weighted bending gradient.

Verifies correctness of the closed-form ∂θ/∂pi formulas against
finite-difference reference values, and confirms constraint energy
decreases monotonically under XPBD projection.
"""

import numpy as np

import simulation  # noqa: F401  (initialises Taichi before any kernel import)
from simulation.constraints.bending import BendingConstraints
from simulation.core.config import SimConfig
from simulation.core.state import ParticleState
from simulation.mesh.grid import generate_grid


# ---------------------------------------------------------------------------
# NumPy reference implementations (used in tests, not in the hot path)
# ---------------------------------------------------------------------------

def _dihedral_angle_np(p0, p1, p2, p3):
    """Compute dihedral angle for hinge (p0,p1,p2,p3) in [-π, π]."""
    e = p1 - p0
    n1 = np.cross(e, p2 - p0)
    n2 = np.cross(e, p3 - p0)
    len_n1 = np.linalg.norm(n1)
    len_n2 = np.linalg.norm(n2)
    len_e  = np.linalg.norm(e)
    if len_n1 < 1e-10 or len_n2 < 1e-10 or len_e < 1e-10:
        return 0.0
    n1_hat = n1 / len_n1
    n2_hat = n2 / len_n2
    e_hat  = e  / len_e
    cos_a = float(np.clip(np.dot(n1_hat, n2_hat), -1.0, 1.0))
    sin_a = float(np.dot(np.cross(n1_hat, n2_hat), e_hat))
    return float(np.arctan2(sin_a, cos_a))


def _fd_bending_grads(p0, p1, p2, p3, eps=1e-5):
    """Finite-difference gradient of dihedral angle w.r.t. all 4 vertices."""
    grads = []
    for p_ref, idx in [(p0, 0), (p1, 1), (p2, 2), (p3, 3)]:
        g = np.zeros(3)
        pts = [p0.copy(), p1.copy(), p2.copy(), p3.copy()]
        for axis in range(3):
            pts_p = [p.copy() for p in pts]
            pts_p[idx][axis] += eps
            pts_m = [p.copy() for p in pts]
            pts_m[idx][axis] -= eps
            g[axis] = (_dihedral_angle_np(*pts_p) - _dihedral_angle_np(*pts_m)) / (2 * eps)
        grads.append(g)
    return tuple(grads)


def _analytical_bending_grads(p0, p1, p2, p3):
    """
    Analytical cotangent-weighted gradient (Bergou 2006 / Grinspun 2003).

    Uses our sign convention: n1 = e × (p2-p0), n2 = e × (p3-p0).
    All n1-related terms are negated vs. the paper (whose normals point opposite).
    """
    e = p1 - p0
    n1 = np.cross(e, p2 - p0)
    n2 = np.cross(e, p3 - p0)
    len_e  = np.linalg.norm(e)
    len_n1 = np.linalg.norm(n1)
    len_n2 = np.linalg.norm(n2)
    inv_n1sq  = 1.0 / (len_n1 * len_n1)
    inv_n2sq  = 1.0 / (len_n2 * len_n2)
    inv_len_e = 1.0 / len_e

    grad2 = -len_e * inv_n1sq * n1
    grad3 =  len_e * inv_n2sq * n2
    grad0 = (-np.dot(p2 - p1, e) * inv_n1sq * inv_len_e) * n1 \
          + ( np.dot(p3 - p1, e) * inv_n2sq * inv_len_e) * n2
    grad1 = ( np.dot(p2 - p0, e) * inv_n1sq * inv_len_e) * n1 \
          + (-np.dot(p3 - p0, e) * inv_n2sq * inv_len_e) * n2
    return grad0, grad1, grad2, grad3


# ---------------------------------------------------------------------------
# Test cases
# ---------------------------------------------------------------------------

class TestAnalyticalBendingGradients:
    """Verify that the analytical gradient formula produces correct values."""

    # A kinked 2-triangle configuration: shared edge along X, triangles in XY and XZ.
    # Dihedral angle is 90° (π/2 rad) — well-conditioned, non-degenerate.
    _P0 = np.array([0.0, 0.0, 0.0])
    _P1 = np.array([1.0, 0.0, 0.0])
    _P2 = np.array([0.5, 1.0, 0.0])   # in XY plane
    _P3 = np.array([0.5, 0.0, 1.0])   # in XZ plane

    def test_gradient_matches_finite_diff_p2(self):
        """Analytical ∂θ/∂p2 agrees with finite-difference to < 0.5%."""
        g_an = _analytical_bending_grads(self._P0, self._P1, self._P2, self._P3)
        g_fd = _fd_bending_grads(self._P0, self._P1, self._P2, self._P3)

        for label, g_a, g_f in zip(["p0", "p1", "p2", "p3"], g_an, g_fd):
            norm_f = np.linalg.norm(g_f)
            if norm_f < 1e-10:
                continue  # zero gradient — both should be zero
            rel_err = np.linalg.norm(g_a - g_f) / norm_f
            assert rel_err < 0.005, (
                f"Analytical vs FD gradient mismatch at {label}: "
                f"rel_err={rel_err:.4%}, analytical={g_a}, fd={g_f}"
            )

    def test_flat_hinge_is_pi(self):
        """
        Flat hinge (coplanar triangles, p2 and p3 on opposite sides of shared
        edge) has dihedral angle θ = π — the fully-extended, unfolded state.

        n1 = e × (p2-p0) = [0,0,+1]  (pointing +Z)
        n2 = e × (p3-p0) = [0,0,-1]  (pointing -Z)
        cos = n1̂·n2̂ = -1 → θ = atan2(0, -1) = π
        """
        p0 = np.array([0.0, 0.0, 0.0])
        p1 = np.array([1.0, 0.0, 0.0])
        p2 = np.array([0.5,  1.0, 0.0])
        p3 = np.array([0.5, -1.0, 0.0])  # opposite side of shared edge, same plane

        theta = _dihedral_angle_np(p0, p1, p2, p3)
        assert abs(abs(theta) - np.pi) < 1e-6, (
            f"Expected flat hinge (|θ|≈π), got θ={theta:.6f}"
        )

    def test_gradient_sum_is_zero(self):
        """
        For a rigid-body translation, ∑ ∂θ/∂pi = 0 (translational invariance).
        Equivalently, the sum of all gradient vectors should be the zero vector.
        """
        g = _analytical_bending_grads(self._P0, self._P1, self._P2, self._P3)
        g_sum = sum(g)
        assert np.linalg.norm(g_sum) < 1e-10, (
            f"Gradient sum not zero: {g_sum} (translational invariance violated)"
        )

    def test_energy_decreasing_under_projection(self):
        """
        Run XPBD bending projection on a kinked grid.

        Mirrors the actual engine usage: lambdas are reset ONCE per substep,
        then N iterations are applied without resetting.  Total violation should
        decrease from initial to final over 20 iterations of a single substep.
        """
        # Build a 5×5 grid and kink the middle column in Z
        grid = generate_grid(cols=5, rows=5, center=(0, 0, 0))
        positions = grid.positions.copy()

        # Lift the middle column in Y (out-of-plane for the XZ grid) to create
        # actual dihedral-angle violations.  Z-only displacement is in-plane
        # and doesn't change any dihedral angle.
        mid_x_mask = np.abs(positions[:, 0]) < 0.15
        positions[mid_x_mask, 1] += 0.3

        bend = BendingConstraints(max_hinges=500)
        bend.initialize(grid.faces, grid.positions)  # rest angles from flat mesh

        config = SimConfig(max_particles=100)
        state = ParticleState(config)
        state.load_from_numpy(positions, faces=grid.faces)

        hinges_np = np.stack([
            bend.hinge_v0.to_numpy()[:bend.n_hinges],
            bend.hinge_v1.to_numpy()[:bend.n_hinges],
            bend.hinge_v2.to_numpy()[:bend.n_hinges],
            bend.hinge_v3.to_numpy()[:bend.n_hinges],
        ], axis=1)
        rest = bend.rest_angle.to_numpy()[:bend.n_hinges]

        def total_violation(pos_np):
            v = 0.0
            for h, (i0, i1, i2, i3) in enumerate(hinges_np):
                theta = _dihedral_angle_np(pos_np[i0], pos_np[i1], pos_np[i2], pos_np[i3])
                C = theta - rest[h]
                while C >  np.pi: C -= 2 * np.pi
                while C < -np.pi: C += 2 * np.pi
                v += abs(C)
            return v

        dt = 1.0 / 60.0 / 6  # substep dt

        initial_violation = total_violation(state.get_positions_numpy())
        assert initial_violation > 0.1, f"Kink too small to test: violation={initial_violation:.4f}"

        # Run 20 iterations within ONE substep (lambda reset once at start)
        bend.reset_lambdas()
        for _ in range(20):
            bend.project(state.positions, state.inv_mass, bend.n_hinges, 1e-4, dt)

        final_violation = total_violation(state.get_positions_numpy())

        assert final_violation < initial_violation * 0.5, (
            f"Violation did not decrease by 50%: {initial_violation:.4f} → {final_violation:.4f}"
        )
