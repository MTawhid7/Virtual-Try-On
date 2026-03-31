"""
Time integration — semi-implicit Euler prediction and velocity update.

These are the "bookend" operations of each substep:
    1. predict_positions: v += g*dt, predicted = pos + v*dt
    2. update_velocities: v = (pos - old_pos) / dt * damping

Between these two, the solver + collision modify `positions` in place.

The predict/update pattern is solver-agnostic — used by both XPBD and PD.
"""

import taichi as ti

from simulation.core.state import ParticleState


@ti.kernel
def predict_positions(
    positions: ti.template(),
    predicted: ti.template(),
    velocities: ti.template(),
    inv_mass: ti.template(),
    n_particles: ti.i32,
    gravity: ti.f32,
    dt: ti.f32,
    max_displacement: ti.f32,
):
    """
    Semi-implicit Euler: apply gravity to velocity, then predict positions.

    - Pinned particles (inv_mass == 0) are not moved.
    - Velocity-clamped to prevent tunneling (from Vestra: max_displacement per substep).
    """
    for i in range(n_particles):
        if inv_mass[i] > 0.0:
            # Apply gravity (Y-axis)
            vel = velocities[i]
            vel[1] += gravity * dt

            # Velocity clamping — cap max displacement per substep
            speed = vel.norm()
            if speed * dt > max_displacement and speed > 1e-8:
                vel = vel * (max_displacement / (speed * dt))

            velocities[i] = vel

            # Save current position and predict new
            predicted[i] = positions[i]
            positions[i] = positions[i] + vel * dt
        else:
            # Pinned: predicted = current position (no movement)
            predicted[i] = positions[i]


@ti.kernel
def update_velocities(
    positions: ti.template(),
    predicted: ti.template(),
    velocities: ti.template(),
    inv_mass: ti.template(),
    n_particles: ti.i32,
    dt: ti.f32,
    damping: ti.f32,
):
    """
    Compute velocities from position delta and apply damping.

    After constraint solving and collision, the actual position differs from
    the predicted position. The velocity is derived from this displacement.
    """
    for i in range(n_particles):
        if inv_mass[i] > 0.0:
            velocities[i] = (positions[i] - predicted[i]) / dt * damping
        else:
            velocities[i] = ti.Vector([0.0, 0.0, 0.0])


class Integrator:
    """
    Stateless integrator wrapping the Taichi kernels.

    Usage:
        integrator = Integrator(config)
        integrator.predict(state)    # Before solver
        integrator.update(state)     # After solver + collision
    """

    def __init__(self, dt: float, gravity: float, damping: float,
                 max_displacement: float) -> None:
        self.dt = dt
        self.gravity = gravity
        self.damping = damping
        self.max_displacement = max_displacement

    def predict(self, state: ParticleState) -> None:
        """Apply gravity and predict positions (semi-implicit Euler)."""
        predict_positions(
            state.positions,
            state.predicted,
            state.velocities,
            state.inv_mass,
            state.n_particles,
            self.gravity,
            self.dt,
            self.max_displacement,
        )

    def update(self, state: ParticleState) -> None:
        """Compute velocities from position delta with damping."""
        update_velocities(
            state.positions,
            state.predicted,
            state.velocities,
            state.inv_mass,
            state.n_particles,
            self.dt,
            self.damping,
        )
