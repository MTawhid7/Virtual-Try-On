//! IPC log-barrier function for contact handling.
//!
//! Implements the C² log-barrier from Li et al. 2020 (IPC):
//!
//! b(d, d_hat) = -(d - d_hat)^2 * ln(d / d_hat),  for 0 < d < d_hat
//! b(d, d_hat) = 0,                                for d >= d_hat
//!
//! The barrier has compact support (zero contribution when primitives are
//! far apart) and goes to +inf as d -> 0 (prevents interpenetration).

/// Compute the IPC log-barrier energy.
///
/// Returns 0 when `d >= d_hat`. Goes to +∞ as `d → 0⁺`.
///
/// # Arguments
/// * `d` — squared distance between primitives (must be > 0)
/// * `d_hat` — activation threshold (barrier is zero when d >= d_hat)
pub fn barrier_energy(d: f32, d_hat: f32) -> f32 {
    if d >= d_hat || d <= 0.0 {
        return 0.0;
    }
    let diff = d - d_hat;
    let ratio = d / d_hat;
    -(diff * diff) * ratio.ln()
}

/// Compute the first derivative of the barrier energy w.r.t. distance d.
///
/// ∂b/∂d = -(d - d̂)(2 ln(d/d̂) + (d - d̂)/d)
///
/// Returns 0 when `d >= d_hat`.
pub fn barrier_gradient(d: f32, d_hat: f32) -> f32 {
    let d_c = d.clamp(1e-12, d_hat);
    if d_c >= d_hat {
        return 0.0;
    }
    let diff = d_c - d_hat;
    let ln_ratio = (d_c / d_hat).ln();
    -(diff) * (2.0 * ln_ratio + diff / d_c)
}

/// Compute the second derivative of the barrier energy w.r.t. distance d.
///
/// ∂²b/∂d² = -2 ln(d/d̂) - (4(d - d̂))/d + (d - d̂)²/d²
///
/// Returns 0 when `d >= d_hat`.
pub fn barrier_hessian(d: f32, d_hat: f32) -> f32 {
    if d >= d_hat || d <= 0.0 {
        return 0.0;
    }
    let diff = d - d_hat;
    let ln_ratio = (d / d_hat).ln();
    let inv_d = 1.0 / d;
    -2.0 * ln_ratio - 4.0 * diff * inv_d + diff * diff * inv_d * inv_d
}

/// Compute the barrier energy scaled by a stiffness parameter κ.
///
/// Returns `κ * b(d, d_hat)`.
pub fn scaled_barrier_energy(d: f32, d_hat: f32, kappa: f32) -> f32 {
    kappa * barrier_energy(d, d_hat)
}

/// Compute the barrier gradient scaled by a stiffness parameter κ.
///
/// Returns `κ * ∂b/∂d`.  Clamped to `±1e4` to prevent the barrier from
/// overwhelming the PD solver's inertia term (M/h²).
pub fn scaled_barrier_gradient(d: f32, d_hat: f32, kappa: f32) -> f32 {
    let raw = barrier_gradient(d, d_hat);
    (kappa * raw).clamp(-1e4, 1e4)
}

/// Estimate initial barrier stiffness κ from mesh and material properties.
///
/// Based on IPC (Li et al. 2020): κ should produce forces comparable to
/// inertial forces at the barrier boundary for smooth deceleration.
///
/// κ₀ = avg_mass × gravity_mag / (d_hat × |∂b/∂d(d_hat/2, d_hat)|)
///
/// If `configured_kappa` is > 0, returns it directly (user override).
pub fn estimate_initial_kappa(
    configured_kappa: f32,
    avg_mass: f32,
    gravity_mag: f32,
    d_hat: f32,
) -> f32 {
    if configured_kappa > 0.0 {
        return configured_kappa;
    }
    // Evaluate the barrier gradient magnitude at the midpoint of the barrier zone
    let d_mid = d_hat * 0.5;
    let grad_at_mid = barrier_gradient(d_mid, d_hat).abs();
    if grad_at_mid < 1e-12 {
        return 1e4; // fallback
    }
    let kappa = avg_mass * gravity_mag / (d_hat * grad_at_mid);
    // Clamp to reasonable range
    kappa.clamp(1e2, 1e8)
}

/// Compute barrier energy with a thickness offset (C-IPC).
///
/// The effective distance is shifted by `thickness²` so that the barrier
/// reaches infinite repulsion when `d = thickness²` instead of `d = 0`.
/// This ensures cloth mid-surfaces maintain at least `thickness` separation.
///
/// b_thick(d, d_hat, δ) = b(d - δ², d_hat)  for d > δ²
///                       = +huge             for d ≤ δ²
pub fn barrier_energy_with_thickness(d: f32, d_hat: f32, thickness: f32) -> f32 {
    let t_sq = thickness * thickness;
    let d_eff = d - t_sq;
    if d_eff <= 0.0 {
        return 1e6; // Massive energy for thickness violation
    }
    barrier_energy(d_eff, d_hat)
}

/// Compute barrier gradient with a thickness offset (C-IPC).
///
/// ∂b_thick/∂d = ∂b/∂d_eff  (chain rule: d_eff = d - δ², so ∂d_eff/∂d = 1)
pub fn barrier_gradient_with_thickness(d: f32, d_hat: f32, thickness: f32) -> f32 {
    let t_sq = thickness * thickness;
    let d_eff = d - t_sq;
    if d_eff <= 0.0 {
        // Clamp to minimum distance for massive repulsive gradient
        return barrier_gradient(1e-12, d_hat);
    }
    barrier_gradient(d_eff, d_hat)
}

/// Compute scaled barrier gradient with thickness offset.
pub fn scaled_barrier_gradient_with_thickness(
    d: f32,
    d_hat: f32,
    kappa: f32,
    thickness: f32,
) -> f32 {
    let raw = barrier_gradient_with_thickness(d, d_hat, thickness);
    (kappa * raw).clamp(-1e4, 1e4)
}

/// Sum the barrier energy across a set of per-vertex signed distances
/// to a collider surface.  Used by the solver's energy-based line search
/// to verify that a proposed step does not increase total barrier energy unboundedly.
///
/// Returns `Σ κ · b(d_i², d_hat)` for all `d_i` where `d_i² < d_hat`.
pub fn barrier_energy_total(distances: &[f32], d_hat: f32, kappa: f32) -> f64 {
    let mut total = 0.0_f64;
    for &d in distances {
        let d_sq = d * d;
        if d <= 0.0 {
            // Penetrating — assign a very large energy proportional to depth
            total += 1e8_f64 * (-d as f64 + 1.0);
        } else if d_sq < d_hat {
            total += (kappa as f64) * (barrier_energy(d_sq, d_hat) as f64);
        }
    }
    total
}

