import json
from simulation.mesh.gc_mesh_adapter import build_garment_mesh_gc
gm = build_garment_mesh_gc("data/patterns/garmentcode/shirt_mean.json")
import numpy as np
pos = gm.positions
print("Overall Y min:", pos[:, 1].min(), "max:", pos[:, 1].max(), "height:", pos[:, 1].max()-pos[:, 1].min())
for k, pid in enumerate(gm.panel_ids):
    start, end = gm.panel_offsets[k], (gm.panel_offsets[k+1] if k+1 < len(gm.panel_offsets) else len(pos))
    p = pos[start:end]
    print(f"Panel {pid} Y min: {p[:, 1].min():.3f} max: {p[:, 1].max():.3f} height: {p[:, 1].max()-p[:, 1].min():.3f}")
