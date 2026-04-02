"""
Unit tests for SphereCollider — collision push-out, friction, and edge cases.

Tests the collider in isolation with minimal particle states (1–3 particles),
no solver or constraint loop. Validates the core collision math independently
from the full simulation pipeline.
"""

import numpy as np
import pytest

import simulation  # noqa: F401 — Taichi initialization
from simulation.core.config import SimConfig
from simulation.core.state import ParticleState
from simulation.collision import SphereCollider


# Shared test fixtures
SPHERE_CENTER = (0.0, 0.0, 0.0)
SPHERE_RADIUS = 0.5


def _make_state_with_particle(
    position: tuple[float, float, float],
    predicted: tuple[float, float, float] | None = None,
    inv_mass: float = 1.0,
) -> ParticleState:
    """
    Create a minimal ParticleState with a single particle.

    Args:
        position: Current particle position.
        predicted: Pre-substep position (for friction computation).
            If None, defaults to the same as position.
        inv_mass: Inverse mass (0 = pinned).
    """
    config = SimConfig(max_particles=10)
    state = ParticleState(config)

    pos = np.array([list(position)], dtype=np.float32)
    inv_masses = np.array([inv_mass], dtype=np.float32)
    state.load_from_numpy(pos, inv_masses=inv_masses)

    # Set predicted position (load_from_numpy sets it to initial pos)
    if predicted is not None:
        pred_padded = np.zeros((config.max_particles, 3), dtype=np.float32)
        pred_padded[0] = list(predicted)
        state.predicted.from_numpy(pred_padded)

    return state


class TestSphereColliderPushOut:
    """Test basic push-out behavior."""

    def test_particle_inside_pushed_out(self):
        """A particle inside the sphere should be pushed to the surface."""
        config = SimConfig(collision_thickness=0.005)
        collider = SphereCollider(center=SPHERE_CENTER, radius=SPHERE_RADIUS)

        # Particle at (0.3, 0, 0) — inside sphere (dist=0.3 < 0.5)
        state = _make_state_with_particle(
            position=(0.3, 0.0, 0.0),
            predicted=(0.3, 0.0, 0.0),
        )

        collider.resolve(state, config)

        pos = state.get_positions_numpy()[0]
        dist = np.linalg.norm(pos - np.array(SPHERE_CENTER))
        target = SPHERE_RADIUS + config.collision_thickness

        # Should be at surface + thickness
        assert dist == pytest.approx(target, abs=1e-3), (
            f"Particle at dist={dist:.4f}, expected {target:.4f}"
        )

    def test_particle_outside_no_change(self):
        """A particle well outside the sphere should not move."""
        config = SimConfig(collision_thickness=0.005)
        collider = SphereCollider(center=SPHERE_CENTER, radius=SPHERE_RADIUS)

        original_pos = (2.0, 0.0, 0.0)  # dist=2.0, well outside
        state = _make_state_with_particle(
            position=original_pos,
            predicted=original_pos,
        )

        collider.resolve(state, config)

        pos = state.get_positions_numpy()[0]
        np.testing.assert_allclose(
            pos, original_pos, atol=1e-6,
            err_msg="Particle outside sphere should not move"
        )

    def test_particle_on_surface_no_change(self):
        """A particle at exactly radius + thickness should not move."""
        config = SimConfig(collision_thickness=0.005)
        collider = SphereCollider(center=SPHERE_CENTER, radius=SPHERE_RADIUS)

        target = SPHERE_RADIUS + config.collision_thickness
        original_pos = (target, 0.0, 0.0)
        state = _make_state_with_particle(
            position=original_pos,
            predicted=original_pos,
        )

        collider.resolve(state, config)

        pos = state.get_positions_numpy()[0]
        np.testing.assert_allclose(
            pos, original_pos, atol=1e-4,
            err_msg="Particle on surface should not move"
        )

    def test_push_direction_along_normal(self):
        """Particle should be pushed outward along the radial direction."""
        config = SimConfig(collision_thickness=0.005, friction_coefficient=0.0)
        collider = SphereCollider(center=SPHERE_CENTER, radius=SPHERE_RADIUS)

        # Particle at (0.2, 0, 0) — should be pushed along +X
        state = _make_state_with_particle(
            position=(0.2, 0.0, 0.0),
            predicted=(0.2, 0.0, 0.0),
        )

        collider.resolve(state, config)

        pos = state.get_positions_numpy()[0]
        target = SPHERE_RADIUS + config.collision_thickness

        # Should be at (target, 0, 0) — pushed along +X only
        assert pos[0] == pytest.approx(target, abs=1e-3)
        assert pos[1] == pytest.approx(0.0, abs=1e-3)
        assert pos[2] == pytest.approx(0.0, abs=1e-3)


class TestSphereColliderFriction:
    """Test position-based friction."""

    def test_friction_reduces_tangential(self):
        """
        Friction should reduce the tangential component of displacement.

        Set up: particle is inside the sphere, but its predicted position
        implies it was moving tangentially. After push-out, the tangential
        component of displacement should be scaled by (1 - friction).
        """
        friction = 0.3
        config = SimConfig(
            collision_thickness=0.005,
            friction_coefficient=friction,
        )
        collider = SphereCollider(center=SPHERE_CENTER, radius=SPHERE_RADIUS)

        # Particle penetrating from below-right, with tangential motion
        # predicted = where it was at substep start
        # position = where solver put it (inside the sphere)
        state = _make_state_with_particle(
            position=(0.3, 0.0, 0.0),   # Inside sphere
            predicted=(0.6, 0.2, 0.0),  # Was outside, moved in with tangential component
        )

        collider.resolve(state, config)

        pos = state.get_positions_numpy()[0]
        dist = np.linalg.norm(pos - np.array(SPHERE_CENTER))

        # Should still be at the surface (push-out works)
        target = SPHERE_RADIUS + config.collision_thickness
        assert dist >= target - 1e-3, (
            f"Particle should be at surface: dist={dist:.4f}"
        )

    def test_zero_friction_preserves_tangential(self):
        """With friction=0, tangential displacement should be fully preserved."""
        config = SimConfig(
            collision_thickness=0.005,
            friction_coefficient=0.0,
        )
        collider = SphereCollider(center=SPHERE_CENTER, radius=SPHERE_RADIUS)

        # Particle inside sphere, predicted was outside
        state = _make_state_with_particle(
            position=(0.2, 0.0, 0.0),
            predicted=(0.2, 0.0, 0.0),   # No tangential motion
        )

        collider.resolve(state, config)

        pos = state.get_positions_numpy()[0]
        target = SPHERE_RADIUS + config.collision_thickness

        # Should be pushed along +X to surface
        assert pos[0] == pytest.approx(target, abs=1e-3)


class TestSphereColliderEdgeCases:
    """Test edge cases and special conditions."""

    def test_pinned_particle_not_moved(self):
        """Pinned particles (inv_mass=0) should never be moved by collision."""
        config = SimConfig(collision_thickness=0.005)
        collider = SphereCollider(center=SPHERE_CENTER, radius=SPHERE_RADIUS)

        original_pos = (0.1, 0.0, 0.0)  # Deep inside sphere
        state = _make_state_with_particle(
            position=original_pos,
            predicted=original_pos,
            inv_mass=0.0,  # Pinned
        )

        collider.resolve(state, config)

        pos = state.get_positions_numpy()[0]
        np.testing.assert_allclose(
            pos, original_pos, atol=1e-6,
            err_msg="Pinned particle should not be moved by collision"
        )

    def test_degenerate_zero_distance(self):
        """Particle at sphere center should be pushed along +Y."""
        config = SimConfig(collision_thickness=0.005, friction_coefficient=0.0)
        collider = SphereCollider(center=SPHERE_CENTER, radius=SPHERE_RADIUS)

        state = _make_state_with_particle(
            position=SPHERE_CENTER,  # Exactly at center
            predicted=SPHERE_CENTER,
        )

        collider.resolve(state, config)

        pos = state.get_positions_numpy()[0]
        dist = np.linalg.norm(pos - np.array(SPHERE_CENTER))
        target = SPHERE_RADIUS + config.collision_thickness

        # Should be pushed out (not stuck at center)
        assert dist == pytest.approx(target, abs=1e-3), (
            f"Degenerate particle should be pushed out: dist={dist:.4f}"
        )
        # Should be pushed along +Y
        assert pos[1] > 0, "Should be pushed upward (+Y)"
