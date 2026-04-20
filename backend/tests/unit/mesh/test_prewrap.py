import numpy as np


class TestPrewrapPanelsToBody:
    def test_back_panel_moves_to_back_surface(self):
        """Back torso panel vertices should end up just behind body back surface."""
        from simulation.mesh.gc_mesh_adapter import build_garment_mesh_gc, prewrap_panels_to_body
        from simulation.mesh.body_measurements import load_profile

        gm = build_garment_mesh_gc(
            "data/patterns/garmentcode/shirt_mean.json",
            mesh_resolution=2.0,
            body_z_offset=0.131,
        )
        clearance = 0.008
        prewrap_panels_to_body(gm, profile_path="data/bodies/mannequin_profile.json", clearance=clearance)

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
                expected_z = profile.at_y(py).z_back - clearance
                assert abs(float(gm.positions[vi, 2]) - expected_z) < 1e-4, (
                    f"Back panel '{pid}' vertex Z={gm.positions[vi, 2]:.4f} "
                    f"expected {expected_z:.4f} at Y={py:.3f}"
                )

    def test_front_panel_moves_to_front_surface(self):
        """Front torso panel vertices should end up just outside body front surface."""
        from simulation.mesh.gc_mesh_adapter import build_garment_mesh_gc, prewrap_panels_to_body
        from simulation.mesh.body_measurements import load_profile

        gm = build_garment_mesh_gc(
            "data/patterns/garmentcode/shirt_mean.json",
            mesh_resolution=2.0,
            body_z_offset=0.131,
        )
        clearance = 0.008
        prewrap_panels_to_body(gm, profile_path="data/bodies/mannequin_profile.json", clearance=clearance)

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
                expected_z = profile.at_y(py).z_front + clearance
                assert abs(float(gm.positions[vi, 2]) - expected_z) < 1e-4, (
                    f"Front panel '{pid}' vertex Z={gm.positions[vi, 2]:.4f} "
                    f"expected {expected_z:.4f} at Y={py:.3f}"
                )

    def test_sleeve_panels_centered_on_stitch_targets(self):
        """Sleeve panels should be translated so stitch-vertex centroid matches armhole centroid."""
        from simulation.mesh.gc_mesh_adapter import build_garment_mesh_gc, prewrap_panels_to_body

        gm = build_garment_mesh_gc(
            "data/patterns/garmentcode/shirt_mean.json",
            mesh_resolution=2.0,
            body_z_offset=0.131,
        )
        prewrap_panels_to_body(gm, profile_path="data/bodies/mannequin_profile.json", clearance=0.008)

        n = gm.positions.shape[0]
        offsets = gm.panel_offsets + [n]
        pairs = gm.stitch_pairs

        for k, pid in enumerate(gm.panel_ids):
            if any(kw in pid.lower() for kw in ("btorso", "ftorso", "back", "front")):
                continue  # only check non-torso panels

            panel_set = set(range(offsets[k], offsets[k + 1]))
            my_verts, their_verts = [], []
            for col, other_col in ((0, 1), (1, 0)):
                mask = np.isin(pairs[:, col], list(panel_set))
                if not np.any(mask):
                    continue
                sv = pairs[mask, col]
                tv = pairs[mask, other_col]
                cross = ~np.isin(tv, list(panel_set))
                my_verts.extend(sv[cross].tolist())
                their_verts.extend(tv[cross].tolist())

            if not my_verts:
                continue  # no cross-panel stitches — skip

            my_centroid = gm.positions[np.array(my_verts)].mean(axis=0)
            their_centroid = gm.positions[np.array(their_verts)].mean(axis=0)
            gap = np.linalg.norm(my_centroid - their_centroid)
            assert gap < 0.08, (
                f"Sleeve panel '{pid}' centroid gap {gap*100:.1f}cm after prewrap "
                f"(expected < 8 cm; was ~16 cm before centering)"
            )
