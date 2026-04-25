"""Measure mannequin body surface extents at torso Y-slices for panel placement."""
import trimesh
import numpy as np

mesh = trimesh.load("data/bodies/mannequin_physics.glb", force="mesh")
verts = np.array(mesh.vertices, dtype=np.float64)

print(f"Mesh: {len(verts)} verts, {len(mesh.faces)} faces")
print(f"Overall bounds:")
print(f"  X: [{verts[:,0].min():.4f}, {verts[:,0].max():.4f}]")
print(f"  Y: [{verts[:,1].min():.4f}, {verts[:,1].max():.4f}]")
print(f"  Z: [{verts[:,2].min():.4f}, {verts[:,2].max():.4f}]")
print()

# Measure Z extents at various Y-slices (using vertices within ±0.03m of target Y)
print("Z extents at torso Y-slices (±0.03m band):")
print(f"{'Y':>6s}  {'Z_min':>8s}  {'Z_max':>8s}  {'Z_center':>8s}  {'Z_gap':>8s}  {'N_verts':>7s}")
for y_target in np.arange(0.70, 1.60, 0.05):
    band = 0.03
    mask = (verts[:, 1] >= y_target - band) & (verts[:, 1] <= y_target + band)
    if mask.sum() == 0:
        continue
    subset = verts[mask]
    z_min = subset[:, 2].min()
    z_max = subset[:, 2].max()
    z_center = (z_min + z_max) / 2
    z_gap = z_max - z_min
    print(f"{y_target:6.2f}  {z_min:8.4f}  {z_max:8.4f}  {z_center:8.4f}  {z_gap:8.4f}  {mask.sum():7d}")

print()
print("X extents at torso Y-slices (torso only, |X| < 0.25):")
print(f"{'Y':>6s}  {'X_min':>8s}  {'X_max':>8s}  {'Width':>8s}")
for y_target in np.arange(0.85, 1.50, 0.05):
    band = 0.03
    mask = (verts[:, 1] >= y_target - band) & (verts[:, 1] <= y_target + band) & (np.abs(verts[:, 0]) < 0.25)
    if mask.sum() == 0:
        continue
    subset = verts[mask]
    x_min = subset[:, 0].min()
    x_max = subset[:, 0].max()
    print(f"{y_target:6.2f}  {x_min:8.4f}  {x_max:8.4f}  {x_max - x_min:8.4f}")

# Suggest placement
print()
print("=== Panel Placement Recommendations ===")
# Find average front/back at Y=0.85-1.40 (tank top torso region)
torso_mask = (verts[:, 1] >= 0.85) & (verts[:, 1] <= 1.40) & (np.abs(verts[:, 0]) < 0.20)
if torso_mask.sum() > 0:
    torso = verts[torso_mask]
    front_z = torso[:, 2].max()
    back_z = torso[:, 2].min()
    print(f"Torso (Y=0.85-1.40, |X|<0.20):")
    print(f"  Front surface Z_max = {front_z:.4f}")
    print(f"  Back surface  Z_min = {back_z:.4f}")
    print(f"  Depth (front-back)  = {front_z - back_z:.4f}")
    clearance = 0.015  # 1.5cm clearance
    print(f"  Recommended front panel Z = {front_z + clearance:.4f}  ({clearance*100:.1f}cm outside)")
    print(f"  Recommended back panel Z  = {back_z - clearance:.4f}  ({clearance*100:.1f}cm outside)")
    print(f"  Initial gap between panels = {(front_z + clearance) - (back_z - clearance):.4f}m")
