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
    air_drag: ti.f32,
):
    """
    Semi-implicit Euler: apply gravity to velocity, then predict positions.

    - Pinned particles (inv_mass == 0) are not moved.
    - Velocity-clamped to prevent tunneling (from Vestra: max_displacement per substep).
    - Optional air drag: exponential velocity decay applied before gravity.
    """
    for i in range(n_particles):
        if inv_mass[i] > 0.0:
            vel = velocities[i]

            # Air drag — exponential decay per substep (Taichi cloth tutorial pattern).
            # Applied before gravity so the gravity impulse is not immediately damped.
            vel = vel * ti.exp(-air_drag * dt)

            # Apply gravity (Y-axis)
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


@ti.kernel
def apply_velocity_cap(
    velocities: ti.template(),
    inv_mass: ti.template(),
    n_particles: ti.i32,
    max_speed: ti.f32,
):
    """
    Hard per-substep velocity ceiling. Scales any particle exceeding max_speed
    back to exactly max_speed, preserving direction.  Called after update_velocities()
    each substep so constraint-derived velocities can't propagate explosion into the
    next substep's predict step.  Complements max_displacement (which caps velocity
    going INTO predict) — together they bracket the full substep.
    """
    for i in range(n_particles):
        if inv_mass[i] <= 0.0:
            continue
        speed = velocities[i].norm()
        if speed > max_speed and speed > 1e-8:
            velocities[i] = velocities[i] * (max_speed / speed)


class Integrator:
    """
    Stateless integrator wrapping the Taichi kernels.

    Usage:
        integrator = Integrator(config)
        integrator.predict(state)    # Before solver
        integrator.update(state)     # After solver + collision
    """

    def __init__(self, dt: float, gravity: float, damping: float,
                 max_displacement: float, air_drag: float = 0.0) -> None:
        self.dt = dt
        self.gravity = gravity
        self.damping = damping
        self.max_displacement = max_displacement
        self.air_drag = air_drag

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
            self.air_drag,
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

    def cap_velocity(self, state: ParticleState, max_speed: float) -> None:
        """Hard velocity ceiling — unconditional backstop after constraint solving."""
        apply_velocity_cap(
            state.velocities,
            state.inv_mass,
            state.n_particles,
            max_speed,
        )
