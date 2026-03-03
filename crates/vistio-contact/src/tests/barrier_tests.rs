use crate::barrier::*;

    #[test]
    fn barrier_zero_outside_threshold() {
        assert_eq!(barrier_energy(0.02, 0.01), 0.0);
        assert_eq!(barrier_energy(0.01, 0.01), 0.0);
        assert_eq!(barrier_gradient(0.02, 0.01), 0.0);
        assert_eq!(barrier_hessian(0.02, 0.01), 0.0);
    }

    #[test]
    fn barrier_positive_inside_threshold() {
        let d_hat = 0.01;
        let d = 0.005;
        let e = barrier_energy(d, d_hat);
        assert!(e > 0.0, "Barrier energy should be positive, got {e}");
    }

    #[test]
    fn barrier_increases_near_zero() {
        let d_hat = 0.01;
        let e1 = barrier_energy(0.005, d_hat);
        let e2 = barrier_energy(0.001, d_hat);
        assert!(e2 > e1, "Barrier should increase as d→0: {e2} > {e1}");
    }

    #[test]
    fn barrier_gradient_negative_repulsive() {
        // The gradient should be negative (repulsive) when d < d_hat,
        // pushing primitives apart (increasing d).
        let d_hat = 0.01;
        let g = barrier_gradient(0.005, d_hat);
        assert!(g < 0.0, "Gradient should be negative (repulsive), got {g}");
    }

    #[test]
    fn barrier_hessian_positive_definite() {
        // Hessian should be positive for stability of Newton-type solvers.
        let d_hat = 0.01;
        let h = barrier_hessian(0.005, d_hat);
        assert!(h > 0.0, "Hessian should be positive definite, got {h}");
    }

    #[test]
    fn barrier_continuous_at_threshold() {
        let d_hat = 0.01;
        // At d = d_hat - ε, the barrier should be very close to 0
        let e = barrier_energy(d_hat - 1e-6, d_hat);
        assert!(e < 1e-10, "Barrier should be nearly zero at threshold, got {e}");
    }

    #[test]
    fn scaled_barrier_scales_correctly() {
        let d = 0.005;
        let d_hat = 0.01;
        let kappa = 1000.0;
        let e = barrier_energy(d, d_hat);
        let se = scaled_barrier_energy(d, d_hat, kappa);
        assert!((se - kappa * e).abs() < 1e-6);
    }

    #[test]
    fn barrier_massive_at_nonpositive_d() {
        assert_eq!(barrier_energy(0.0, 0.01), 0.0);
        // But gradients should be massively repulsive to push vertices out
        let g = barrier_gradient(0.0, 0.01);
        assert!(g < -100.0, "Gradient should be massively repulsive even at or below 0, got {}", g);
    }

    #[test]
    fn adaptive_kappa_scales_with_mass() {
        // Heavier cloth should produce larger κ
        let k1 = estimate_initial_kappa(0.0, 0.001, 9.81, 1e-3);
        let k2 = estimate_initial_kappa(0.0, 0.01, 9.81, 1e-3);
        assert!(k2 > k1, "Heavier cloth should have larger κ: {k2} > {k1}");
        // Both should be positive and reasonable
        assert!(k1 > 0.0);
        assert!(k2 < 1e8);
    }

    #[test]
    fn adaptive_kappa_respects_override() {
        let k = estimate_initial_kappa(5000.0, 0.001, 9.81, 1e-3);
        assert_eq!(k, 5000.0, "Should return configured κ when > 0");
    }

    #[test]
    fn barrier_with_thickness_increases_min_distance() {
        let d_hat = 0.01;
        let thickness = 0.001; // 1mm
        let t_sq = thickness * thickness;

        // At d = t_sq (exactly the thickness offset), energy should be massive
        let e_at_thickness = barrier_energy_with_thickness(t_sq, d_hat, thickness);
        assert!(e_at_thickness > 1e5, "Energy should be huge at thickness boundary, got {e_at_thickness}");

        // At d = t_sq + d_hat (outside barrier zone), energy should be zero
        let e_outside = barrier_energy_with_thickness(t_sq + d_hat + 0.001, d_hat, thickness);
        assert_eq!(e_outside, 0.0, "Energy should be zero outside barrier zone");

        // At d = t_sq + d_hat/2 (midpoint), energy should be positive
        let e_mid = barrier_energy_with_thickness(t_sq + d_hat * 0.5, d_hat, thickness);
        assert!(e_mid > 0.0, "Energy should be positive inside barrier zone, got {e_mid}");
    }

    #[test]
    fn thickness_gradient_repulsive() {
        let d_hat = 0.01;
        let thickness = 0.001;
        let t_sq = thickness * thickness;

        // Gradient should be strongly repulsive inside the thickness zone
        let g = barrier_gradient_with_thickness(t_sq + d_hat * 0.01, d_hat, thickness);
        assert!(g < 0.0, "Gradient should be repulsive near thickness boundary, got {g}");
    }
