"""
Analytical sphere collider — Sprint 1 Layer 3a.

Validates the collision push-out + friction logic using a simple sphere
before Sprint 2 adds the full mesh-proxy body collider with spatial hash.

The collider is called inside the XPBD solver iteration loop (interleaved
collision — the Vestra pattern). For each cloth particle that penetrates
the sphere, it pushes the particle to the sphere surface + thickness and
applies position-based friction to the tangential displacement.

Friction is position-based (not velocity-based): the tangential component
of the displacement relative to the pre-substep position (`predicted`) is
scaled down by `(1 - friction_coefficient)`. This way, `update_velocities`
in the integrator naturally picks up the friction effect through the
reduced position delta.
"""


import taichi as ti

from simulation.core.config import SimConfig
from simulation.core.state import ParticleState


@ti.kernel
def _resolve_sphere_collision(
    positions: ti.template(),
    predicted: ti.template(),
    inv_mass: ti.template(),
    n_particles: ti.i32,
    center: ti.math.vec3,
    radius: ti.f32,
    thickness: ti.f32,
    friction: ti.f32,
):
    """
    Taichi kernel: push penetrating particles to sphere surface + apply
    position-based friction.

    For each particle:
        1. Compute distance from sphere center
        2. If penetrating (dist < radius + thickness):
           a. Push to surface along outward normal
           b. Decompose displacement into normal + tangential
           c. Scale tangential by (1 - friction)
        3. Handle degenerate case (particle at center) → escape along +Y
        4. Skip pinned particles (inv_mass == 0)
    """
    for i in range(n_particles):
        # Skip pinned particles
        if inv_mass[i] <= 0.0:
            continue

        d = positions[i] - center
        dist = d.norm()

        target_dist = radius + thickness

        if dist < target_dist:
            # Compute outward normal
            n_hat = ti.Vector([0.0, 1.0, 0.0])  # Default: escape along +Y
            if dist > 1e-8:
                n_hat = d / dist

            # 1. Push to surface
            surface_pos = center + n_hat * target_dist

            # 2. Position-based friction
            # Decompose displacement (from substep start) into
            # normal and tangential components
            disp = surface_pos - predicted[i]
            d_normal = disp.dot(n_hat) * n_hat
            d_tangential = disp - d_normal

            # Scale tangential displacement by (1 - friction)
            positions[i] = predicted[i] + d_normal + d_tangential * (1.0 - friction)


class SphereCollider:
    """
    Analytical sphere collision resolver.

    For each cloth particle inside the sphere (distance < radius + thickness),
    pushes it to the surface and applies tangential friction.

    This is a stepping stone to the full BodyCollider in Sprint 2. The
    resolve() interface is shared so the engine doesn't need to know which
    collider type is active.

    Usage:
        collider = SphereCollider(center=(0, 0.8, 0), radius=0.5)
        engine.collider = collider
        # Engine calls collider.resolve(state, config) inside solver loop
    """

    def __init__(
        self,
        center: tuple[float, float, float],
        radius: float,
    ) -> None:
        """
        Args:
            center: World-space center of the sphere (x, y, z).
            radius: Sphere radius in meters.
        """
        self.center = ti.Vector([center[0], center[1], center[2]])
        self.radius = radius

    def resolve(self, state: ParticleState, config: SimConfig) -> None:
        """
        Resolve sphere collisions for all particles.

        Called inside the XPBD solver iteration loop by the engine.
        Reads collision_thickness and friction_coefficient from config.

        Note: Velocity clamping after collision is NOT implemented here.
        The integrator already clamps velocity in predict_positions().
        TODO (Sprint 2): Add post-collision velocity clamping when the
        body mesh collider is introduced — the more complex geometry may
        produce larger position corrections that benefit from clamping.
        """
        _resolve_sphere_collision(
            state.positions,
            state.predicted,
            state.inv_mass,
            state.n_particles,
            self.center,
            self.radius,
            config.collision_thickness,
            config.friction_coefficient,
        )
