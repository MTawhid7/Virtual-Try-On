import numpy as np


class TestPrewrapPanelsToBody:
    def test_back_panel_moves_to_back_surface(self):
        """Back torso panel vertices within body width should lie on the back-hemisphere ellipse at clearance.
        Vertices beyond body half-width are placed at z_back - clearance (body surface Z), not on the ellipse.
        """
        from simulation.mesh.gc_mesh_adapter import build_garment_mesh_gc, prewrap_panels_to_body
        from simulation.mesh.body_measurements import load_profile

        gm = build_garment_mesh_gc(
            "data/patterns/garmentcode/shirt_mean.json",
            mesh_resolution=2.0,
            body_z_offset=0.131,
        )
        clearance = 0.008
        prewrap_panels_to_body(gm, clearance=clearance)

        profile = load_profile("data/bodies/mannequin_profile.json")
        n = gm.positions.shape[0]
        offsets = gm.panel_offsets + [n]

        for k, pid in enumerate(gm.panel_ids):
            if "btorso" not in pid.lower() and "back" not in pid.lower():
                continue
            for vi in range(offsets[k], offsets[k + 1]):
                py = float(gm.positions[vi, 1])
                if py < 0.65 or py > 1.60:
                    continue
                sl = profile.at_y(py)
                cx, cz = sl.center_x, sl.center_z
                a_body = sl.width / 2
                a = a_body + clearance
                b = sl.depth / 2 + clearance
                wx = float(gm.positions[vi, 0])
                wz = float(gm.positions[vi, 2])
                # Vertices beyond body half-width are surface-snapped, not on the ellipse.
                sin_t = (wx - cx) / max(a_body, 1e-8)
                if abs(sin_t) > 1.0:
                    continue
                r = ((wx - cx) / a) ** 2 + ((wz - cz) / b) ** 2
                assert abs(r - 1.0) < 0.005, (
                    f"Back panel '{pid}' vertex not on body ellipse: r={r:.4f} at Y={py:.3f}"
                )
                assert wz <= cz + clearance * 2, (
                    f"Back panel '{pid}' vertex on wrong hemisphere: Z={wz:.4f} center_Z={cz:.4f}"
                )

    def test_front_panel_moves_to_front_surface(self):
        """Front torso panel vertices within body width should lie on the front-hemisphere ellipse at clearance.
        Vertices beyond body half-width are placed at z_front + clearance (body surface Z), not on the ellipse.
        """
        from simulation.mesh.gc_mesh_adapter import build_garment_mesh_gc, prewrap_panels_to_body
        from simulation.mesh.body_measurements import load_profile

        gm = build_garment_mesh_gc(
            "data/patterns/garmentcode/shirt_mean.json",
            mesh_resolution=2.0,
            body_z_offset=0.131,
        )
        clearance = 0.008
        prewrap_panels_to_body(gm, clearance=clearance)

        profile = load_profile("data/bodies/mannequin_profile.json")
        n = gm.positions.shape[0]
        offsets = gm.panel_offsets + [n]

        for k, pid in enumerate(gm.panel_ids):
            if "ftorso" not in pid.lower() and "front" not in pid.lower():
                continue
            for vi in range(offsets[k], offsets[k + 1]):
                py = float(gm.positions[vi, 1])
                if py < 0.65 or py > 1.60:
                    continue
                sl = profile.at_y(py)
                cx, cz = sl.center_x, sl.center_z
                a_body = sl.width / 2
                a = a_body + clearance
                b = sl.depth / 2 + clearance
                wx = float(gm.positions[vi, 0])
                wz = float(gm.positions[vi, 2])
                # Vertices beyond body half-width are surface-snapped, not on the ellipse.
                sin_t = (wx - cx) / max(a_body, 1e-8)
                if abs(sin_t) > 1.0:
                    continue
                r = ((wx - cx) / a) ** 2 + ((wz - cz) / b) ** 2
                assert abs(r - 1.0) < 0.005, (
                    f"Front panel '{pid}' vertex not on body ellipse: r={r:.4f} at Y={py:.3f}"
                )
                assert wz >= cz - clearance * 2, (
                    f"Front panel '{pid}' vertex on wrong hemisphere: Z={wz:.4f} center_Z={cz:.4f}"
                )

    def test_sleeve_panels_centered_on_stitch_targets(self):
        """Sleeve armhole centroids (XY only) should be near the torso armhole centroid after prewrap.
        Z is intentionally preserved from GarmentCode placement (front sleeves near z_front,
        back sleeves near z_back), so only XY alignment is checked.
        """
        from simulation.mesh.gc_mesh_adapter import build_garment_mesh_gc, prewrap_panels_to_body

        gm = build_garment_mesh_gc(
            "data/patterns/garmentcode/shirt_mean.json",
            mesh_resolution=2.0,
            body_z_offset=0.131,
        )
        prewrap_panels_to_body(gm, clearance=0.008)

        n = gm.positions.shape[0]
        offsets = gm.panel_offsets + [n]
        pairs = gm.stitch_pairs

        _TORSO_KWS = ("btorso", "ftorso", "back", "front")
        torso_vert_set: set[int] = set()
        for k2, pid2 in enumerate(gm.panel_ids):
            if any(kw in pid2.lower() for kw in _TORSO_KWS):
                torso_vert_set.update(range(offsets[k2], offsets[k2 + 1]))

        for k, pid in enumerate(gm.panel_ids):
            if any(kw in pid.lower() for kw in _TORSO_KWS):
                continue  # only check non-torso panels

            panel_set = set(range(offsets[k], offsets[k + 1]))
            my_verts, their_torso_verts = [], []
            for col, other_col in ((0, 1), (1, 0)):
                mask = np.isin(pairs[:, col], list(panel_set))
                if not np.any(mask):
                    continue
                sv = pairs[mask, col]
                tv = pairs[mask, other_col]
                cross = ~np.isin(tv, list(panel_set))
                my_verts.extend(sv[cross].tolist())
                torso_cross = cross & np.isin(tv, list(torso_vert_set))
                their_torso_verts.extend(tv[torso_cross].tolist())

            if not my_verts or not their_torso_verts:
                continue  # no cross-torso stitches — skip

            my_centroid = gm.positions[np.array(my_verts)].mean(axis=0)
            torso_centroid = gm.positions[np.array(their_torso_verts)].mean(axis=0)
            # XY-only gap: Z is intentionally preserved from GarmentCode placement.
            gap_xy = np.linalg.norm(my_centroid[:2] - torso_centroid[:2])
            assert gap_xy < 0.08, (
                f"Sleeve panel '{pid}' armhole XY centroid gap {gap_xy*100:.1f}cm from torso armhole "
                f"after prewrap (expected < 8 cm)"
            )

    def test_calibrate_garment_y(self):
        """After Y-calibration, the topmost garment vertex should equal profile.neck_y."""
        from simulation.mesh.gc_mesh_adapter import build_garment_mesh_gc, calibrate_garment_y
        from simulation.mesh.body_measurements import load_profile

        gm = build_garment_mesh_gc(
            "data/patterns/garmentcode/shirt_mean.json",
            mesh_resolution=2.0,
            body_z_offset=0.131,
        )
        calibrate_garment_y(gm, "data/bodies/mannequin_profile.json", anchor="neck")
        profile = load_profile("data/bodies/mannequin_profile.json")
        assert abs(float(gm.positions[:, 1].max()) - profile.neck_y) < 1e-4, (
            f"Garment top Y={float(gm.positions[:, 1].max()):.4f} "
            f"expected {profile.neck_y:.4f} after neck calibration"
        )

    def test_no_initial_penetrations(self):
        """After resolve_initial_penetrations(), no garment vertex should be inside the body."""
        import trimesh
        from simulation.mesh.gc_mesh_adapter import (
            build_garment_mesh_gc,
            calibrate_garment_y,
            prewrap_panels_to_body,
            resolve_initial_penetrations,
        )

        gm = build_garment_mesh_gc(
            "data/patterns/garmentcode/shirt_mean.json",
            mesh_resolution=2.0,
            body_z_offset=0.131,
        )
        calibrate_garment_y(gm, "data/bodies/mannequin_profile.json")
        prewrap_panels_to_body(gm, clearance=0.008)
        resolve_initial_penetrations(gm, clearance=0.008)

        body = trimesh.load("data/bodies/mannequin_physics.glb", force="mesh")
        inside = body.contains(gm.positions.astype(np.float64))
        assert not np.any(inside), (
            f"{int(inside.sum())} vertices still inside body after resolve_initial_penetrations()"
        )

    def test_inv_mass_clamped(self):
        """max_inv_mass clamp should cap degenerate tiny-triangle inv_mass values."""
        from simulation.mesh.grid import compute_area_weighted_inv_masses

        # Two triangles: a 0.3mm degenerate CDT seam triangle + a normal 5cm triangle
        pos = np.array(
            [[0, 0, 0], [3e-4, 0, 0], [0, 3e-4, 0],
             [0, 0.1, 0], [0.05, 0.1, 0], [0, 0.15, 0]],
            dtype=np.float32,
        )
        faces = np.array([[0, 1, 2], [3, 4, 5]], dtype=np.int32)
        inv = compute_area_weighted_inv_masses(pos, faces, density=0.3, max_inv_mass=5000.0)
        assert float(inv.max()) <= 5001.0, (
            f"max inv_mass = {inv.max():.1f}, expected ≤ 5000 after clamp"
        )
