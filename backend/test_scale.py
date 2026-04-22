from simulation.mesh.gc_mesh_adapter import build_garment_mesh_gc
try:
    gm = build_garment_mesh_gc("data/patterns/garmentcode/shirt_mean.json", mesh_resolution=2.0)
    print("Loaded without scaling")
except Exception as e:
    print(e)
