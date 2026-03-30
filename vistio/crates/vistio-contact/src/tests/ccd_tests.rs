use vistio_math::Vec3;
use crate::ccd::*;

    #[test]
    fn ccd_vertex_passes_through_triangle() {
        // Vertex moves from above to below a triangle
        let p0 = Vec3::new(0.25, 0.25, 1.0);
        let p1 = Vec3::new(0.25, 0.25, -1.0);
        let a0 = Vec3::new(0.0, 0.0, 0.0);
        let a1 = a0; // Stationary
        let b0 = Vec3::new(1.0, 0.0, 0.0);
        let b1 = b0;
        let c0 = Vec3::new(0.0, 1.0, 0.0);
        let c1 = c0;

        let toi = vertex_triangle_ccd(p0, p1, a0, a1, b0, b1, c0, c1);
        assert!(toi.is_some(), "Should detect collision");
        let t = toi.unwrap();
        assert!(t > 0.0 && t < 1.0, "TOI should be in (0,1), got {t}");
        // At t=0.5 the vertex is at z=0 (on the triangle plane)
        assert!((t - 0.5 * CCD_SAFETY_MARGIN).abs() < 0.1, "TOI should be near 0.45, got {t}");
    }

    #[test]
    fn ccd_vertex_misses_triangle() {
        // Vertex moves parallel to triangle, never crossing
        let p0 = Vec3::new(5.0, 5.0, 1.0);
        let p1 = Vec3::new(5.0, 5.0, -1.0);
        let a0 = Vec3::new(0.0, 0.0, 0.0);
        let a1 = a0;
        let b0 = Vec3::new(1.0, 0.0, 0.0);
        let b1 = b0;
        let c0 = Vec3::new(0.0, 1.0, 0.0);
        let c1 = c0;

        let toi = vertex_triangle_ccd(p0, p1, a0, a1, b0, b1, c0, c1);
        assert!(toi.is_none(), "Should not detect collision for miss");
    }

    #[test]
    fn ccd_vertex_moves_away() {
        // Vertex starts above and moves further away
        let p0 = Vec3::new(0.25, 0.25, 1.0);
        let p1 = Vec3::new(0.25, 0.25, 2.0);
        let a0 = Vec3::new(0.0, 0.0, 0.0);
        let a1 = a0;
        let b0 = Vec3::new(1.0, 0.0, 0.0);
        let b1 = b0;
        let c0 = Vec3::new(0.0, 1.0, 0.0);
        let c1 = c0;

        let toi = vertex_triangle_ccd(p0, p1, a0, a1, b0, b1, c0, c1);
        assert!(toi.is_none(), "Should not detect collision when moving away");
    }

    #[test]
    fn ccd_edge_edge_crossing() {
        // Two edges that cross each other during motion
        let a0 = Vec3::new(-1.0, 0.0, 0.5);
        let a1 = Vec3::new(-1.0, 0.0, -0.5);
        let b0 = Vec3::new(1.0, 0.0, 0.5);
        let b1 = Vec3::new(1.0, 0.0, -0.5);

        let c0 = Vec3::new(0.0, -1.0, -0.5);
        let c1 = Vec3::new(0.0, -1.0, 0.5);
        let d0 = Vec3::new(0.0, 1.0, -0.5);
        let d1 = Vec3::new(0.0, 1.0, 0.5);

        let toi = edge_edge_ccd(a0, a1, b0, b1, c0, c1, d0, d1);
        assert!(toi.is_some(), "Should detect edge-edge crossing");
    }

    #[test]
    fn ccd_edge_edge_no_crossing() {
        // Two parallel edges that never cross
        let a0 = Vec3::new(0.0, 0.0, 0.0);
        let a1 = Vec3::new(0.0, 0.0, 0.0);
        let b0 = Vec3::new(1.0, 0.0, 0.0);
        let b1 = Vec3::new(1.0, 0.0, 0.0);

        let c0 = Vec3::new(0.0, 5.0, 0.0);
        let c1 = Vec3::new(0.0, 5.0, 0.0);
        let d0 = Vec3::new(1.0, 5.0, 0.0);
        let d1 = Vec3::new(1.0, 5.0, 0.0);

        let toi = edge_edge_ccd(a0, a1, b0, b1, c0, c1, d0, d1);
        assert!(toi.is_none(), "Should not detect collision for parallel edges");
    }

    #[test]
    fn cubic_solver_triple_root() {
        // (t - 0.5)³ = t³ - 1.5t² + 0.75t - 0.125
        let roots = solve_cubic_roots(1.0, -1.5, 0.75, -0.125);
        assert!(!roots.is_empty());
        for r in &roots {
            assert!((r - 0.5).abs() < 0.01, "Root should be ~0.5, got {r}");
        }
    }

    #[test]
    fn cubic_solver_identity() {
        // t³ - 6t² + 11t - 6 = (t-1)(t-2)(t-3)
        let roots = solve_cubic_roots(1.0, -6.0, 11.0, -6.0);
        assert_eq!(roots.len(), 3);
        let mut sorted = roots.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((sorted[0] - 1.0).abs() < 0.01);
        assert!((sorted[1] - 2.0).abs() < 0.01);
        assert!((sorted[2] - 3.0).abs() < 0.01);
    }
