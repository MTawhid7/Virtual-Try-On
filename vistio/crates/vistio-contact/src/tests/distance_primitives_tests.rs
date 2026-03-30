use vistio_math::Vec3;
use crate::distance_primitives::*;

    // ─── Point-Triangle Tests ────────────────────────────────

    #[test]
    fn distance_point_on_face() {
        let a = Vec3::new(0.0, 0.0, 0.0);
        let b = Vec3::new(1.0, 0.0, 0.0);
        let c = Vec3::new(0.0, 1.0, 0.0);
        let p = Vec3::new(0.25, 0.25, 0.0); // On the face

        let (d2, dtype) = point_triangle_distance_squared(p, a, b, c);
        assert!(d2 < 1e-10, "Point on face should have zero distance, got {d2}");
        assert_eq!(dtype, DistanceType::PointTriangleFace);
    }

    #[test]
    fn distance_point_above_face() {
        let a = Vec3::new(0.0, 0.0, 0.0);
        let b = Vec3::new(1.0, 0.0, 0.0);
        let c = Vec3::new(0.0, 1.0, 0.0);
        let p = Vec3::new(0.25, 0.25, 0.5); // Above the face

        let (d2, dtype) = point_triangle_distance_squared(p, a, b, c);
        assert!((d2 - 0.25).abs() < 1e-6, "Expected d²=0.25, got {d2}");
        assert_eq!(dtype, DistanceType::PointTriangleFace);
    }

    #[test]
    fn distance_point_nearest_vertex() {
        let a = Vec3::new(0.0, 0.0, 0.0);
        let b = Vec3::new(1.0, 0.0, 0.0);
        let c = Vec3::new(0.0, 1.0, 0.0);
        let p = Vec3::new(-1.0, -1.0, 0.0); // Nearest to vertex A

        let (d2, dtype) = point_triangle_distance_squared(p, a, b, c);
        assert!((d2 - 2.0).abs() < 1e-6, "Expected d²=2.0, got {d2}");
        assert_eq!(dtype, DistanceType::PointTriangleVertex(0));
    }

    #[test]
    fn distance_point_nearest_edge() {
        let a = Vec3::new(0.0, 0.0, 0.0);
        let b = Vec3::new(1.0, 0.0, 0.0);
        let c = Vec3::new(0.0, 1.0, 0.0);
        let p = Vec3::new(0.5, -0.5, 0.0); // Nearest to edge AB

        let (d2, dtype) = point_triangle_distance_squared(p, a, b, c);
        assert!((d2 - 0.25).abs() < 1e-6, "Expected d²=0.25, got {d2}");
        assert_eq!(dtype, DistanceType::PointTriangleEdge(0));
    }

    #[test]
    fn distance_degenerate_triangle() {
        let a = Vec3::new(0.0, 0.0, 0.0);
        let b = Vec3::new(1.0, 0.0, 0.0);
        let c = Vec3::new(0.5, 0.0, 0.0); // Degenerate — all on a line
        let p = Vec3::new(0.5, 1.0, 0.0);

        let (d2, _dtype) = point_triangle_distance_squared(p, a, b, c);
        assert!((d2 - 1.0).abs() < 1e-6, "Expected d²=1.0, got {d2}");
    }

    // ─── Edge-Edge Tests ─────────────────────────────────────

    #[test]
    fn distance_edges_crossing() {
        let a0 = Vec3::new(0.0, 0.0, -1.0);
        let a1 = Vec3::new(0.0, 0.0, 1.0);
        let b0 = Vec3::new(-1.0, 1.0, 0.0);
        let b1 = Vec3::new(1.0, 1.0, 0.0);

        let (d2, _) = edge_edge_distance_squared(a0, a1, b0, b1);
        assert!((d2 - 1.0).abs() < 1e-6, "Expected d²=1.0, got {d2}");
    }

    #[test]
    fn distance_edges_parallel() {
        let a0 = Vec3::new(0.0, 0.0, 0.0);
        let a1 = Vec3::new(1.0, 0.0, 0.0);
        let b0 = Vec3::new(0.0, 1.0, 0.0);
        let b1 = Vec3::new(1.0, 1.0, 0.0);

        let (d2, _) = edge_edge_distance_squared(a0, a1, b0, b1);
        assert!((d2 - 1.0).abs() < 1e-6, "Expected d²=1.0, got {d2}");
    }

    #[test]
    fn distance_edges_touching() {
        let a0 = Vec3::new(0.0, 0.0, 0.0);
        let a1 = Vec3::new(1.0, 0.0, 0.0);
        let b0 = Vec3::new(0.5, 0.0, 0.0);
        let b1 = Vec3::new(0.5, 1.0, 0.0);

        let (d2, _) = edge_edge_distance_squared(a0, a1, b0, b1);
        assert!(d2 < 1e-10, "Touching edges should have zero distance, got {d2}");
    }

    #[test]
    fn distance_degenerate_edges() {
        // Both segments are points
        let a0 = Vec3::new(0.0, 0.0, 0.0);
        let a1 = Vec3::new(0.0, 0.0, 0.0);
        let b0 = Vec3::new(1.0, 0.0, 0.0);
        let b1 = Vec3::new(1.0, 0.0, 0.0);

        let (d2, dtype) = edge_edge_distance_squared(a0, a1, b0, b1);
        assert!((d2 - 1.0).abs() < 1e-6, "Expected d²=1.0, got {d2}");
        assert_eq!(dtype, DistanceType::Degenerate);
    }

    // ─── Gradient Tests ──────────────────────────────────────

    #[test]
    fn point_triangle_gradient_finite_difference() {
        let a = Vec3::new(0.0, 0.0, 0.0);
        let b = Vec3::new(1.0, 0.0, 0.0);
        let c = Vec3::new(0.0, 1.0, 0.0);
        let p = Vec3::new(0.25, 0.25, 0.5);

        let grads = point_triangle_distance_gradient(p, a, b, c);
        let (d2_0, _) = point_triangle_distance_squared(p, a, b, c);
        let eps = 1e-4;

        // Test gradient w.r.t. p.x via finite difference
        let p_shifted = Vec3::new(p.x + eps, p.y, p.z);
        let (d2_1, _) = point_triangle_distance_squared(p_shifted, a, b, c);
        let fd_px = (d2_1 - d2_0) / eps;

        assert!(
            (grads[0].x - fd_px).abs() < 0.01,
            "Gradient w.r.t. p.x: analytical={}, FD={}",
            grads[0].x, fd_px
        );
    }

    #[test]
    fn edge_edge_gradient_finite_difference() {
        let a0 = Vec3::new(0.0, 0.0, -0.5);
        let a1 = Vec3::new(0.0, 0.0, 0.5);
        let b0 = Vec3::new(-0.5, 1.0, 0.0);
        let b1 = Vec3::new(0.5, 1.0, 0.0);

        let grads = edge_edge_distance_gradient(a0, a1, b0, b1);
        let (d2_0, _) = edge_edge_distance_squared(a0, a1, b0, b1);
        let eps = 1e-4;

        // Test gradient w.r.t. a0.y via finite difference
        let a0_shifted = Vec3::new(a0.x, a0.y + eps, a0.z);
        let (d2_1, _) = edge_edge_distance_squared(a0_shifted, a1, b0, b1);
        let fd_a0y = (d2_1 - d2_0) / eps;

        assert!(
            (grads[0].y - fd_a0y).abs() < 0.05,
            "Gradient w.r.t. a0.y: analytical={}, FD={}",
            grads[0].y, fd_a0y
        );
    }
