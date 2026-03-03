use crate::ipc_response::*;

    #[test]
    fn ipc_contact_set_empty() {
        let set = IpcContactSet::new(0.01, 1e4);
        assert!(set.is_empty());
        assert_eq!(set.len(), 0);
    }

    #[test]
    fn ipc_detect_close_vertex_triangle() {
        let mut set = IpcContactSet::new(0.1, 1e4);

        // Position arrays: vertex 0 is close to triangle (1, 2, 3)
        let pos_x = vec![0.25, 0.0, 1.0, 0.0];
        let pos_y = vec![0.25, 0.0, 0.0, 1.0];
        let pos_z = vec![0.05, 0.0, 0.0, 0.0]; // Close but not touching

        set.detect_vertex_triangle(0, 1, 2, 3, &pos_x, &pos_y, &pos_z, false);

        assert_eq!(set.len(), 1, "Should detect one close contact");
        assert_eq!(set.contacts[0].contact_type, IpcContactType::VertexTriangle);
    }

    #[test]
    fn ipc_detect_far_vertex_triangle() {
        let mut set = IpcContactSet::new(0.01, 1e4);

        // Vertex is far from triangle
        let pos_x = vec![10.0, 0.0, 1.0, 0.0];
        let pos_y = vec![10.0, 0.0, 0.0, 1.0];
        let pos_z = vec![10.0, 0.0, 0.0, 0.0];

        set.detect_vertex_triangle(0, 1, 2, 3, &pos_x, &pos_y, &pos_z, false);

        assert!(set.is_empty(), "Should not detect far contact");
    }

    #[test]
    fn ipc_barrier_energy_positive() {
        let mut set = IpcContactSet::new(0.1, 1e4);

        let pos_x = vec![0.25, 0.0, 1.0, 0.0];
        let pos_y = vec![0.25, 0.0, 0.0, 1.0];
        let pos_z = vec![0.05, 0.0, 0.0, 0.0];

        set.detect_vertex_triangle(0, 1, 2, 3, &pos_x, &pos_y, &pos_z, false);

        let energy = set.total_barrier_energy(&pos_x, &pos_y, &pos_z);
        assert!(energy > 0.0, "Barrier energy should be positive for close contact, got {energy}");
    }

    #[test]
    fn ipc_gradient_nonzero() {
        let mut set = IpcContactSet::new(0.1, 1e4);

        let pos_x = vec![0.25, 0.0, 1.0, 0.0];
        let pos_y = vec![0.25, 0.0, 0.0, 1.0];
        let pos_z = vec![0.05, 0.0, 0.0, 0.0];

        set.detect_vertex_triangle(0, 1, 2, 3, &pos_x, &pos_y, &pos_z, false);

        let (gx, gy, gz) = set.compute_barrier_gradient(&pos_x, &pos_y, &pos_z, 4);

        // The vertex is above the triangle — gradient should push it away (positive z)
        let magnitude = gx[0] * gx[0] + gy[0] * gy[0] + gz[0] * gz[0];
        assert!(magnitude > 0.0, "Gradient magnitude should be nonzero, got {magnitude}");
    }

    #[test]
    fn ipc_constraint_violation() {
        let mut set = IpcContactSet::new(0.1, 1e4);

        let pos_x = vec![0.25, 0.0, 1.0, 0.0];
        let pos_y = vec![0.25, 0.0, 0.0, 1.0];
        let pos_z = vec![0.05, 0.0, 0.0, 0.0];

        set.detect_vertex_triangle(0, 1, 2, 3, &pos_x, &pos_y, &pos_z, false);

        let violation = set.max_constraint_violation(&pos_x, &pos_y, &pos_z);
        // absolute penetration logic asserts that if no physical penetration exists
        // (i.e. distance squared > 1e-12), the maximum constraint violation is 0.0
        assert_eq!(violation, 0.0, "Violation should be 0.0 for non-penetrating geometry, got {violation}");
    }
