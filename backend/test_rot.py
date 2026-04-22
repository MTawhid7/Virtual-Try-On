from simulation.mesh.gc_mesh_adapter import build_garment_mesh_gc
# just load the boxmesh
from boxmesh import BoxMesh
import numpy as np
bm = BoxMesh("data/patterns/garmentcode/shirt_mean.json", res=2.0)
bm.load()
p = bm.panels['left_sleeve_f']
print("panel verts shape:", np.array(p.panel_vertices).shape)
try:
    pts = np.array(p.panel_vertices)
    pts_3d = np.hstack([pts, np.ones((len(pts), 1))])  # add Z=1
    out = p.rot_trans_panel(pts_3d.tolist())
    print("Handled 3D! Output shape:", np.array(out).shape)
except Exception as e:
    print("Error:", e)
